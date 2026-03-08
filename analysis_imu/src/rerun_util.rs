use std::sync::Arc;

use super::{IcmReceiver, LsmReceiver};

use firmware_imu::icm42688::{self, ICM42688, IcmFifoIterator, IcmFifoProcessor, TimeTracker};
use firmware_imu::lsm6dsv320x::{
    self, AccelerationScaleHigh, FifoEntry, LsmFifoIterator, LsmFifoProcessor, LsmFifoTag,
};
use firmware_imu::lsm6dsv320x::{AccelerationScale, GyroscopeScale};
use firmware_imu::lsm6dsv320x::{GameRotationVectorRaw, LSM6DSV320X};

pub struct ClockSkewCorrector {
    scale: f64,
    system_start: u64,
    clock_start: u64,
    discard_counter: usize,
}
impl ClockSkewCorrector {
    pub fn new(system_start: u64, clock_start: u64) -> Self {
        Self {
            scale: 1.0,
            system_start,
            clock_start,
            discard_counter: 1000,
        }
    }
    pub fn update(&mut self, value_other_ns: u64) {
        // do smart things.
        if self.discard_counter > 0 {
            self.discard_counter -= 1;
            return;
        }
        let system_clock = std::time::SystemTime::now()
            .duration_since(std::time::UNIX_EPOCH)
            .unwrap()
            .as_nanos() as u64;
        let system_duration = system_clock - self.system_start;
        let clock_duration = value_other_ns - self.clock_start;
        self.scale = system_duration as f64 / clock_duration as f64;
        // println!("scale: {:?}", self.scale);
    }
    pub fn clock_correct(&self, value: u64) -> u64 {
        (value as f64 * self.scale) as u64
    }
    pub fn clock_scale(&self) -> f64 {
        self.scale
    }
}

pub struct SyncData {
    creation_instant: std::time::SystemTime,
    creation_timestamp: u64,
    lsm_start: std::sync::atomic::AtomicU64,
    lsm_current: std::sync::atomic::AtomicU64,
    /// ICM clock first data.
    icm_start: std::sync::atomic::AtomicU64,
    /// ICM clock - start + creation.
    icm_current: std::sync::atomic::AtomicU64,
    icm_corrected: std::sync::atomic::AtomicU64,
}
impl SyncData {
    pub fn new() -> Self {
        let t = std::time::SystemTime::now();
        let t_as_nsec = t.duration_since(std::time::UNIX_EPOCH).unwrap().as_nanos() as u64;
        Self {
            creation_instant: std::time::SystemTime::now(),
            creation_timestamp: t_as_nsec,
            lsm_start: 0.into(),
            lsm_current: 0.into(),
            icm_start: 0.into(),
            icm_current: 0.into(),
            icm_corrected: 0.into(),
        }
    }
}

pub fn lsm_pump(
    lsm_rec: LsmReceiver,
    rec: rerun::RecordingStream,
    sync: Arc<SyncData>,
) -> Result<(), Box<dyn std::error::Error>> {
    let processor = LsmFifoProcessor {
        accel_scale: AccelerationScale::G8,
        accel_high_scale: AccelerationScaleHigh::G32,
        gyro_scale: GyroscopeScale::Dps2000,
    };
    // should chunk this with https://docs.rs/rerun/latest/rerun/log/struct.Chunk.html
    // Or not, a chunk is for a single entity only...

    let mut current_t = 0;
    let mut clock_correct: Option<ClockSkewCorrector> = None;
    loop {
        let mut lsm_data = [0u8; 70];
        lsm_data.fill_with(|| lsm_rec.0.recv().unwrap());
        for v in LsmFifoIterator::new(&lsm_data) {
            let (data_type, bytes) = v?;
            //println!("{data_type:?} {bytes:?}");
            let r = processor.interpret(data_type, bytes);

            let lsm_start = sync.lsm_start.load(std::sync::atomic::Ordering::Relaxed);
            let offset = if let Some(c) = &clock_correct {
                c.clock_correct(current_t - lsm_start)
            } else {
                current_t - lsm_start
            };
            let t_cycle = sync.creation_instant + std::time::Duration::from_nanos(offset);
            match r {
                FifoEntry::GameRotationVector(game_rotation_vector_raw) => {
                    match game_rotation_vector_raw {
                        GameRotationVectorRaw::First { w, x } => {
                            println!("GameRotationVector {: <15} {: <15}", w.to_f32(), x.to_f32())
                        }
                        GameRotationVectorRaw::Second { y, z } => println!(
                            "GameRotationVector                              {: <15} {: <15}",
                            y.to_f32(),
                            z.to_f32()
                        ),
                    }
                }
                FifoEntry::AccelerometerNC(a) => {
                    rec.set_time("imu_time", t_cycle);
                    // println!("a{: >0}  {: >6.3?}", "", a.xyz_f32());
                    let (x, y, z) = a.xyz_f32();
                    rec.log("lsm/accel/x", &rerun::Scalars::single(x))?;
                    rec.log("lsm/accel/y", &rerun::Scalars::single(y))?;
                    rec.log("lsm/accel/z", &rerun::Scalars::single(z))?;
                }
                FifoEntry::HighGAccelerometer(a) => {
                    rec.set_time("imu_time", t_cycle);
                    // println!("a+{: >30}{: >6.3?}", "", a.xyz_f32());
                    let (x, y, z) = a.xyz_f32();
                    rec.log("lsm/accel_high/x", &rerun::Scalars::single(x))?;
                    rec.log("lsm/accel_high/y", &rerun::Scalars::single(y))?;
                    rec.log("lsm/accel_high/z", &rerun::Scalars::single(z))?;
                }
                FifoEntry::GyroscopeNC(g) => {
                    rec.set_time("imu_time", t_cycle);
                    // println!("g{: >60}{: >6.3?}", "", g.xyz_f32());
                    let (x, y, z) = g.xyz_f32();
                    rec.log("lsm/gyro/x", &rerun::Scalars::single(x))?;
                    rec.log("lsm/gyro/y", &rerun::Scalars::single(y))?;
                    rec.log("lsm/gyro/z", &rerun::Scalars::single(z))?;
                }
                FifoEntry::Timestamp(t) => {
                    let system_clock = std::time::SystemTime::now()
                        .duration_since(std::time::UNIX_EPOCH)
                        .unwrap();
                    // println!("t {: >60}{: >6.3?}  {: >6.3?}", "", t, tf32);
                    let start = if sync.lsm_start.load(std::sync::atomic::Ordering::Relaxed) == 0 {
                        let val = t.as_nsec_u64();
                        sync.lsm_start
                            .store(val, std::sync::atomic::Ordering::Relaxed);
                        clock_correct = Some(ClockSkewCorrector::new(sync.creation_timestamp, val));
                        val
                    } else {
                        sync.lsm_start.load(std::sync::atomic::Ordering::Relaxed)
                    };
                    current_t = t.as_nsec_u64();

                    let offset = current_t - start;
                    let lsm_current = offset + sync.creation_timestamp;
                    sync.lsm_current
                        .store(lsm_current, std::sync::atomic::Ordering::Relaxed);
                    let lsm_t_sec = (lsm_current / 1000) as f64 * 1e-6;
                    rec.log("lsm/t_current", &rerun::Scalars::single(lsm_t_sec))?;
                    let tf32 = t.as_secs_f32();
                    rec.log("lsm/t", &rerun::Scalars::single(tf32))?;

                    // Do the current difference here.
                    let current_icm = sync.icm_current.load(std::sync::atomic::Ordering::Relaxed);
                    let difference_s =
                        ((current_icm / 1000) as f64 - (lsm_current / 1000) as f64) * 1e-6;
                    let t_as_secs_f64 = system_clock.as_secs_f64();
                    let lsm_current_s_f64 = lsm_current as f64 * 1e-9;
                    let icm_current_s_f64 = current_icm as f64 * 1e-9;
                    let lsm_diff = lsm_current_s_f64 - t_as_secs_f64;
                    let icm_diff = icm_current_s_f64 - t_as_secs_f64;

                    let clock_correct = clock_correct.as_mut().unwrap();
                    clock_correct.update(current_t);
                    rec.log(
                        "time/lsm_skew",
                        &rerun::Scalars::single(clock_correct.clock_scale()),
                    )?;

                    let lsm_corrected = ((clock_correct.clock_correct(offset)
                        + sync.creation_timestamp) as f64
                        * 1e-9)
                        - t_as_secs_f64;
                    let icm_corrected = (sync
                        .icm_corrected
                        .load(std::sync::atomic::Ordering::Relaxed)
                        as f64
                        * 1e-9)
                        - t_as_secs_f64;

                    if lsm_diff.abs() > 1000.0
                        || icm_diff.abs() > 1000.0
                        || icm_corrected.abs() > 1000.0
                        || difference_s.abs() > 1000.0
                    {
                        continue;
                    }
                    rec.log("time/diff_s", &rerun::Scalars::single(difference_s))?;
                    rec.log("time/system", &rerun::Scalars::single(t_as_secs_f64))?;

                    rec.log("time/lsm_min_system", &rerun::Scalars::single(lsm_diff))?;
                    rec.log("time/lsm_corrected", &rerun::Scalars::single(lsm_corrected))?;
                    rec.log("time/icm_min_system", &rerun::Scalars::single(icm_diff))?;
                    rec.log("time/icm_corrected", &rerun::Scalars::single(icm_corrected))?;
                }
                FifoEntry::Temperature(t) => {
                    rec.log("lsm/temperature_c", &rerun::Scalars::single(t.to_c_f32()))?;
                }
                _ => {}
            }
        }
    }
}

pub fn icm_pump(
    icm_rec: IcmReceiver,
    rec: rerun::RecordingStream,
    sync: Arc<SyncData>,
) -> Result<(), Box<dyn std::error::Error>> {
    let processor = IcmFifoProcessor {
        gyro_scale: icm42688::GyroscopeScale::Dps250,
        accel_scale: icm42688::AccelerationScale::G16,
    };
    let mut time_tracker = TimeTracker::new();
    let mut clock_correct: Option<ClockSkewCorrector> = None;
    loop {
        // const PACKET_SIZE: usize = 20;
        const PACKET_SIZE: usize = 20 - 4;
        let mut lsm_data = [0u8; PACKET_SIZE * 10];
        lsm_data.fill_with(|| icm_rec.0.recv().unwrap());

        for v in IcmFifoIterator::new(&lsm_data) {
            let (hdr, data) = v?;
            {
                //println!("{:?}: {:?}", hdr, data);
                if hdr.data() {
                    let r = processor.interpret(hdr, data);
                    time_tracker.update(r.timestamp);
                    //println!("  {:?} {:#?}", time_tracker.time_us(), r);
                    let current_value = sync.icm_start.load(std::sync::atomic::Ordering::Relaxed);
                    let icm_start = if current_value == 0 {
                        let val = time_tracker.time_ns();
                        sync.icm_start
                            .store(val, std::sync::atomic::Ordering::Relaxed);
                        clock_correct = Some(ClockSkewCorrector::new(sync.creation_timestamp, val));
                        val
                    } else {
                        current_value
                    };
                    let clock_correct = clock_correct.as_mut().unwrap();
                    clock_correct.update(time_tracker.time_ns());
                    let offset = clock_correct.clock_correct(time_tracker.time_ns() - icm_start);
                    let t_corrected = sync.creation_timestamp + offset;
                    rec.log(
                        "time/icm_skew",
                        &rerun::Scalars::single(clock_correct.clock_scale()),
                    )?;

                    let t_cycle = sync.creation_instant + std::time::Duration::from_nanos(offset);
                    rec.set_time("imu_time", t_cycle);

                    let icm_current =
                        (time_tracker.time_ns() - icm_start) + sync.creation_timestamp;
                    sync.icm_current
                        .store(icm_current, std::sync::atomic::Ordering::Relaxed);
                    sync.icm_corrected
                        .store(t_corrected, std::sync::atomic::Ordering::Relaxed);
                    let lsm_t_sec = (icm_current / 1000) as f64 * 1e-6;
                    rec.log("icm/t_current", &rerun::Scalars::single(lsm_t_sec))?;
                    rec.log(
                        "icm/t",
                        &rerun::Scalars::single(time_tracker.time_us() as f64 * 1e-6),
                    )?;
                    if let Some(accel) = r.acceleration {
                        rec.set_time("imu_time", t_cycle);
                        // println!("  {: >8.3?}", accel.xyz_f32());
                        let (x, y, z) = accel.xyz_f32();
                        rec.log("icm/accel/x", &rerun::Scalars::single(x))?;
                        rec.log("icm/accel/y", &rerun::Scalars::single(y))?;
                        rec.log("icm/accel/z", &rerun::Scalars::single(z))?;
                    }
                    if let Some(gyro) = r.gyroscope {
                        rec.set_time("imu_time", t_cycle);
                        // println!("  {: >8.3?}", gyro.xyz_f32());
                        let (x, y, z) = gyro.xyz_f32();
                        rec.log("icm/gyro/x", &rerun::Scalars::single(x))?;
                        rec.log("icm/gyro/y", &rerun::Scalars::single(y))?;
                        rec.log("icm/gyro/z", &rerun::Scalars::single(z))?;
                    }

                    // Temperature
                    {
                        let v = r.temperature;
                        rec.log("icm/temperature_c", &rerun::Scalars::single(v.to_c_f32()))?;
                    }
                }
            }
        }
    }
}

pub fn things(
    lsm_rec: LsmReceiver,
    icm_rec: IcmReceiver,
) -> Result<(), Box<dyn std::error::Error>> {
    let rec =
        rerun::RecordingStreamBuilder::new("rerun_example_scalar_column_updates").connect_grpc()?;

    let sync_data = Arc::new(SyncData::new());

    let rec_lsm = rec.clone();
    let sync_lsm = sync_data.clone();
    let handle_lsm = std::thread::spawn(move || {
        lsm_pump(lsm_rec, rec_lsm, sync_lsm);
    });
    let rec_icm = rec.clone();
    let sync_icm = sync_data.clone();
    let handle_icm = std::thread::spawn(move || {
        icm_pump(icm_rec, rec_icm, sync_icm);
    });

    loop {}
}
