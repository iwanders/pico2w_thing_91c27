use super::{IcmReceiver, LsmReceiver};
use rerun::TimeColumn;

use firmware_imu::icm42688::{self, ICM42688, IcmFifoIterator, IcmFifoProcessor, TimeTracker};
use firmware_imu::lsm6dsv320x::{
    self, AccelerationScaleHigh, FifoEntry, LsmFifoIterator, LsmFifoProcessor, LsmFifoTag,
};
use firmware_imu::lsm6dsv320x::{AccelerationScale, GyroscopeScale};
use firmware_imu::lsm6dsv320x::{GameRotationVectorRaw, LSM6DSV320X};
pub fn lsm_pump(
    lsm_rec: LsmReceiver,
    rec: rerun::RecordingStream,
) -> Result<(), Box<dyn std::error::Error>> {
    let processor = LsmFifoProcessor {
        accel_scale: AccelerationScale::G8,
        accel_high_scale: AccelerationScaleHigh::G32,
        gyro_scale: GyroscopeScale::Dps4000,
    };
    //  should chunk this with https://docs.rs/rerun/latest/rerun/log/struct.Chunk.html
    loop {
        let mut lsm_data = [0u8; 70];
        lsm_data.fill_with(|| lsm_rec.0.recv().unwrap());
        let mut iter = LsmFifoIterator::new(&lsm_data);
        for v in iter {
            let (data_type, bytes) = v.unwrap();
            //println!("{data_type:?} {bytes:?}");
            let r = processor.interpret(data_type, bytes);

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
                    // println!("a{: >0}  {: >6.3?}", "", a.xyz_f32());
                    let (x, y, z) = a.xyz_f32();
                    rec.log("lsm/accel/x", &rerun::Scalars::single(x))?;
                    rec.log("lsm/accel/y", &rerun::Scalars::single(y))?;
                    rec.log("lsm/accel/z", &rerun::Scalars::single(z))?;
                }
                FifoEntry::HighGAccelerometer(a) => {
                    // println!("a+{: >30}{: >6.3?}", "", a.xyz_f32());
                    let (x, y, z) = a.xyz_f32();
                    rec.log("lsm/accel_high/x", &rerun::Scalars::single(x))?;
                    rec.log("lsm/accel_high/y", &rerun::Scalars::single(y))?;
                    rec.log("lsm/accel_high/z", &rerun::Scalars::single(z))?;
                }
                FifoEntry::GyroscopeNC(g) => {
                    // println!("g{: >60}{: >6.3?}", "", g.xyz_f32());
                    let (x, y, z) = g.xyz_f32();
                    rec.log("lsm/gyro/x", &rerun::Scalars::single(x))?;
                    rec.log("lsm/gyro/y", &rerun::Scalars::single(y))?;
                    rec.log("lsm/gyro/z", &rerun::Scalars::single(z))?;
                }
                FifoEntry::Timestamp(t) => {
                    // println!("t {: >60}{: >6.3?}", "", t);
                    let tf32 = t.as_secs_f32();
                    rec.log("lsm/t", &rerun::Scalars::single(tf32))?;
                }
                _ => {}
            }
        }
    }

    Ok(())
}

pub fn icm_pump(
    icm_rec: IcmReceiver,
    rec: rerun::RecordingStream,
) -> Result<(), Box<dyn std::error::Error>> {
    let processor = IcmFifoProcessor {
        gyro_scale: icm42688::GyroscopeScale::Dps250,
        accel_scale: icm42688::AccelerationScale::G16,
    };
    let mut time_tracker = TimeTracker::new();
    loop {
        // const PACKET_SIZE: usize = 20;
        const PACKET_SIZE: usize = 20 - 4;
        let mut lsm_data = [0u8; PACKET_SIZE * 10];
        lsm_data.fill_with(|| icm_rec.0.recv().unwrap());

        let mut iter = IcmFifoIterator::new(&lsm_data);
        for v in iter {
            if let Ok((hdr, data)) = v {
                //println!("{:?}: {:?}", hdr, data);
                if hdr.data() {
                    let r = processor.interpret(hdr, data);
                    time_tracker.update(r.timestamp);
                    //println!("  {:?} {:#?}", time_tracker.time_us(), r);
                    let tf32 = time_tracker.time_us() as f32 * 1e-6;
                    rec.log("icm/t", &rerun::Scalars::single(tf32))?;
                    if let Some(accel) = r.acceleration {
                        // println!("  {: >8.3?}", accel.xyz_f32());
                        let (x, y, z) = accel.xyz_f32();
                        rec.log("icm/accel/x", &rerun::Scalars::single(x))?;
                        rec.log("icm/accel/y", &rerun::Scalars::single(y))?;
                        rec.log("icm/accel/z", &rerun::Scalars::single(z))?;
                    }
                    if let Some(gyro) = r.gyroscope {
                        // println!("  {: >8.3?}", gyro.xyz_f32());
                        let (x, y, z) = gyro.xyz_f32();
                        rec.log("icm/gyro/x", &rerun::Scalars::single(x))?;
                        rec.log("icm/gyro/y", &rerun::Scalars::single(y))?;
                        rec.log("icm/gyro/z", &rerun::Scalars::single(z))?;
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

    let rec_lsm = rec.clone();
    let handle_lsm = std::thread::spawn(move || {
        lsm_pump(lsm_rec, rec_lsm);
    });
    let rec_icm = rec.clone();
    let handle_icm = std::thread::spawn(move || {
        icm_pump(icm_rec, rec_icm);
    });

    loop {}
}
