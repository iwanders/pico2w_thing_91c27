use embassy_executor::Spawner;
use embassy_rp::gpio::Output;
use embassy_rp::peripherals::USB;
use embassy_rp::usb::Driver;
use embassy_sync::blocking_mutex::raw::NoopRawMutex;
use embassy_time::{Duration, Timer};
use embassy_usb::class::cdc_acm::CdcAcmClass;
use embedded_hal_async::spi::SpiDevice;

use crate::icm42688;
use crate::icm42688::ICM42688;
use crate::lsm6dsv320x;
use crate::lsm6dsv320x::LSM6DSV320X;
use defmt::{info, println, warn};
use static_cell::StaticCell;

type CdcType = CdcAcmClass<'static, Driver<'static, USB>>;

type IcmSPI = embassy_rp::spi::Spi<'static, embassy_rp::peripherals::SPI0, embassy_rp::spi::Async>;
type IcmDeviceSPI = embassy_embedded_hal::shared_bus::asynch::spi::SpiDevice<
    'static,
    NoopRawMutex,
    IcmSPI,
    Output<'static>,
>;
type ICM = ICM42688<IcmDeviceSPI>;
type ICMError = icm42688::Error<<IcmDeviceSPI as embedded_hal_async::spi::ErrorType>::Error>;

type LsmSPI = embassy_rp::spi::Spi<'static, embassy_rp::peripherals::SPI1, embassy_rp::spi::Async>;
type LsmDeviceSPI = embassy_embedded_hal::shared_bus::asynch::spi::SpiDevice<
    'static,
    NoopRawMutex,
    LsmSPI,
    Output<'static>,
>;
type LSM = LSM6DSV320X<LsmDeviceSPI>;
type LSMError = lsm6dsv320x::Error<<LsmDeviceSPI as embedded_hal_async::spi::ErrorType>::Error>;

/*

Okay, so one task for each imu.

Each writes into a ringbuffer using a non blocking spsc

We have one task to shovel bytes from both ringbuffers to the data port.
*/

#[derive(Debug, Default)]
struct AtomicBufferStats {
    pub queue_pushed: core::sync::atomic::AtomicUsize,
    pub queue_overrun: core::sync::atomic::AtomicUsize,
    pub queue_size: core::sync::atomic::AtomicUsize,
    pub ic_overruns: core::sync::atomic::AtomicUsize,
}
impl AtomicBufferStats {
    pub fn to_read_only(&self) -> BufferStats {
        let queue_pushed = self
            .queue_pushed
            .load(core::sync::atomic::Ordering::Relaxed);
        let queue_overrun = self
            .queue_overrun
            .load(core::sync::atomic::Ordering::Relaxed);
        BufferStats {
            queue_pushed,
            queue_overrun,
            queue_size: self.queue_size.load(core::sync::atomic::Ordering::Relaxed),
            ic_overruns: self.ic_overruns.load(core::sync::atomic::Ordering::Relaxed),
            total_produced: queue_pushed + queue_overrun,
        }
    }
}

#[derive(Copy, Clone, Debug, defmt::Format, zerocopy::IntoBytes)]
#[repr(u8)]
enum DataType {
    Nop,
    Icm,
    Lsm,
}

#[derive(Debug, defmt::Format)]
struct BufferStats {
    pub queue_pushed: usize,
    pub queue_overrun: usize,
    pub queue_size: usize,
    pub ic_overruns: usize,
    pub total_produced: usize,
}
#[embassy_executor::task]
async fn icm_task(
    mut icm: ICM,
    mut producer: heapless::spsc::Producer<'static, u8>,
    buffer: &'static mut [u8],
    stats: &'static AtomicBufferStats,
) -> ! {
    //usb.run().await
    loop {
        let (status, fifo_bytes) = icm.get_status_fifo(buffer).await.unwrap();
        let relevant_data = &buffer[0..fifo_bytes];
        for (i, b) in relevant_data.iter().enumerate() {
            if producer.enqueue(*b).is_err() {
                // defmt::warn!("buffer overrrun in icm task");
                stats
                    .queue_overrun
                    .fetch_add(fifo_bytes - i, core::sync::atomic::Ordering::Relaxed);
                continue;
            } else {
                stats
                    .queue_pushed
                    .fetch_add(1, core::sync::atomic::Ordering::Relaxed);
            }
        }
        if status.fifo_full() {
            stats
                .ic_overruns
                .fetch_add(1, core::sync::atomic::Ordering::Relaxed);
        }
        stats
            .queue_size
            .store(producer.len(), core::sync::atomic::Ordering::Relaxed);
    }
}
#[embassy_executor::task]
async fn lsm_task(
    mut lsm: LSM,
    mut producer: heapless::spsc::Producer<'static, u8>,
    buffer: &'static mut [u8],
    stats: &'static AtomicBufferStats,
) -> ! {
    //usb.run().await
    loop {
        let s = lsm.get_fifo_status().await.unwrap();

        // let b = embassy_time::Instant::now();
        let read_len = s.unread() as usize;
        // defmt::info!("s: {:?}  ", read_len);
        lsm.get_fifo(&mut buffer[0..(read_len) * 7]).await.unwrap();
        // let e = embassy_time::Instant::now();
        // defmt::info!("s: {:?} took: {} us", s, (e - b).as_micros());
        let total_bytes = read_len * 7;
        let relevant_data = &buffer[0..total_bytes];

        // Lets write to the pipe in multiples of 7, that way we ensure that can can correctly parse the stream of
        // data.
        let available_in_pipe = producer.capacity() - producer.len();
        // println!("available_in_pipe: {}", available_in_pipe);
        let available_in_multiples = (available_in_pipe / 7) * 7;
        // println!("available_in_multiples: {}", available_in_multiples);
        // Timer::after_millis(1000).await;
        let use_data = relevant_data.len().min(available_in_multiples);
        for (i, b) in relevant_data[0..use_data].iter().enumerate() {
            if producer.enqueue(*b).is_err() {
                // defmt::warn!("buffer overrrun in icm task");
                stats
                    .queue_overrun
                    .fetch_add(total_bytes - i, core::sync::atomic::Ordering::Relaxed);
                continue;
            } else {
                stats
                    .queue_pushed
                    .fetch_add(1, core::sync::atomic::Ordering::Relaxed);
            }
        }
        if s.overrun_latched() {
            stats
                .ic_overruns
                .fetch_add(1, core::sync::atomic::Ordering::Relaxed);
        }
        stats
            .queue_size
            .store(producer.len(), core::sync::atomic::Ordering::Relaxed);
    }
}

#[embassy_executor::task]
async fn data_cdc_task(
    mut cdc: CdcType,
    mut icm_consumer: heapless::spsc::Consumer<'static, u8>,
    mut lsm_consumer: heapless::spsc::Consumer<'static, u8>,
) -> ! {
    const BUFFER_LEN: usize = 64;
    let buffer = {
        static FIFO_BUFFER: StaticCell<[u8; BUFFER_LEN]> = StaticCell::new();
        FIFO_BUFFER.init([0u8; BUFFER_LEN])
    };

    let mut bool_did_something = false;
    loop {
        for (t, c) in [
            (DataType::Lsm, &mut lsm_consumer),
            (DataType::Icm, &mut icm_consumer),
        ] {
            while c.len() > 60 {
                buffer[0] = t as u8;
                for i in 0..60 {
                    buffer[i + 4] = unsafe { c.dequeue_unchecked() };
                }
                bool_did_something = true;
                cdc.write_packet(buffer).await.unwrap();
            }
        }
        if !bool_did_something {
            Timer::after_nanos(1).await;
        }
        bool_did_something = false;
    }
}

pub async fn imu_entry(
    spawner: Spawner,
    mut icm: ICM,
    mut lsm: LSM,
    mut cdc: CdcType,
    mut output_pin: Output<'static>,
) {
    defmt::info!("Setting up imu!");

    // _icm_test(icm).await.unwrap();
    //
    configure_lsm(&mut lsm).await.unwrap();
    // _lsm_test(lsm).await.unwrap();
    configure_icm(&mut icm).await.unwrap();

    let icm_stats: &AtomicBufferStats = {
        static FIFO_BUFFER: StaticCell<AtomicBufferStats> = StaticCell::new();
        FIFO_BUFFER.init(AtomicBufferStats::default())
    };
    let lsm_stats: &AtomicBufferStats = {
        static FIFO_BUFFER: StaticCell<AtomicBufferStats> = StaticCell::new();
        FIFO_BUFFER.init(AtomicBufferStats::default())
    };

    let icm_buffer = {
        const BUFFER_LEN: usize = 4096;
        static FIFO_BUFFER: StaticCell<[u8; BUFFER_LEN]> = StaticCell::new();
        FIFO_BUFFER.init([0u8; BUFFER_LEN])
    };

    let icm_queue = {
        const BUFFER_LEN: usize = 4096;
        type QueueType = heapless::spsc::Queue<u8, BUFFER_LEN>;
        static STATIC_BUFFER: StaticCell<QueueType> = StaticCell::new();
        STATIC_BUFFER.init(QueueType::new())
    };
    let (icm_producer, icm_consumer) = icm_queue.split();

    // 247 kb/s at 16kHz
    spawner
        .spawn(icm_task(icm, icm_producer, icm_buffer, icm_stats))
        .unwrap();

    let lsm_buffer = {
        const BUFFER_LEN: usize = 4096;
        static FIFO_BUFFER: StaticCell<[u8; BUFFER_LEN]> = StaticCell::new();
        FIFO_BUFFER.init([0u8; BUFFER_LEN])
    };

    let lsm_queue = {
        const BUFFER_LEN: usize = 4096;
        type QueueType = heapless::spsc::Queue<u8, BUFFER_LEN>;
        static STATIC_BUFFER: StaticCell<QueueType> = StaticCell::new();
        STATIC_BUFFER.init(QueueType::new())
    };
    let (lsm_producer, lsm_consumer) = lsm_queue.split();

    // 159 kb/s at max rate.
    spawner
        .spawn(lsm_task(lsm, lsm_producer, lsm_buffer, lsm_stats))
        .unwrap();

    // And the USB task.
    spawner
        .spawn(data_cdc_task(cdc, icm_consumer, lsm_consumer))
        .unwrap();

    loop {
        defmt::println!("icm stats: {:#?}", icm_stats.to_read_only());
        defmt::println!("lsm stats: {:#?}", lsm_stats.to_read_only());
        Timer::after_millis(1000).await;
    }
    /*
     *
     for w in relevant_data.chunks(64) {
         cdc.write_packet(&w).await.unwrap();
     }
     Timer::after_millis(1).await;
    */

    /*
    loop {
        // This is roughly 430 kb/s, but the two of them in sequence results in less throughput.
        if true {
            let status = icm.read_status().await.unwrap();
            let f = icm.read_fifo_count().await.unwrap();
            defmt::info!(
                "status data {:?}, full {},  f: {:?},  ",
                status.data_ready(),
                status.fifo_full(),
                f
            );
            icm.get_fifo(&mut buffer[0..BUFFER_LEN.min(f as usize)])
                .await
                .unwrap();
        }

        // This keeps up and produces a 159kb/s transfer on the data cdc.
        if false {
            let s = lsm.get_fifo_status().await.unwrap();

            let b = embassy_time::Instant::now();
            let read_len = s.unread() as usize;
            lsm.get_fifo(&mut buffer[0..(read_len) * 7]).await.unwrap();
            let e = embassy_time::Instant::now();
            defmt::info!("s: {:?} took: {} us", s, (e - b).as_micros());

            let relevant_data = &buffer[0..(read_len * 7)];
            for w in relevant_data.chunks(64) {
                cdc.write_packet(&w).await.unwrap();
            }
        }
        // Timer::after_millis(1).await;
    }*/
}

async fn configure_lsm(lsm: &mut LSM) -> Result<(), LSMError> {
    lsm.reset().await?;
    Timer::after_millis(10).await;

    // Low accelerometer setup;
    use lsm6dsv320x::{AccelerationMode, AccelerationModeDataRate, OutputDataRate};
    lsm.control_acceleration(AccelerationModeDataRate {
        mode: AccelerationMode::HighPerformance,
        rate: OutputDataRate::Hz7680,
    })
    .await?;
    use lsm6dsv320x::{AccelerationFilterScale, AccelerationScale};
    lsm.filter_acceleration(AccelerationFilterScale {
        scale: AccelerationScale::G8,
    })
    .await?;

    // High acceleratometer setup;
    use lsm6dsv320x::{
        AccelerationDataRateHigh, AccelerationModeDataRateHigh, AccelerationScaleHigh,
    };
    lsm.control_acceleration_high(AccelerationModeDataRateHigh {
        scale: AccelerationScaleHigh::G320,
        rate: AccelerationDataRateHigh::Hz7680,
        ..Default::default()
    })
    .await?;

    // Gyroscope setup.
    use lsm6dsv320x::{GyroscopeMode, GyroscopeModeDataRate};
    lsm.control_gyroscope(GyroscopeModeDataRate {
        mode: GyroscopeMode::HighPerformance,
        rate: OutputDataRate::Hz3840,
    })
    .await?;
    use lsm6dsv320x::{GyroscopeBandwidthScale, GyroscopeScale};
    lsm.filter_gyroscope(GyroscopeBandwidthScale {
        scale: GyroscopeScale::DPS4000,
    })
    .await?;

    // Setup fifo.
    use lsm6dsv320x::{FifoControl, FifoMode, TemperatureBatch, TimestampBatch};
    lsm.control_fifo(FifoControl {
        timestamp: TimestampBatch::EveryBatch,
        mode: FifoMode::Continuous,
        temperature: TemperatureBatch::Hz60,
    })
    .await?;
    use lsm6dsv320x::FifoBatch;
    lsm.control_fifo_batch(FifoBatch {
        gyroscope: OutputDataRate::Hz7680,
        acceleration: OutputDataRate::Hz7680,
    })
    .await?;
    // And this last one to start collecting high G samples to the fifo.
    use lsm6dsv320x::{BatchDataRateConfig, TriggerBDRSource};
    lsm.control_bdr_config(BatchDataRateConfig {
        trigger_bdr: TriggerBDRSource::Acceleration,
        batch_acceleration_high: true,
        threshold: 0,
    })
    .await?;

    const ENABLE_SFLP_QUATERNION: bool = true;
    if ENABLE_SFLP_QUATERNION {
        use lsm6dsv320x::EmbeddedFunctionEnableA;
        lsm.embedded_functions_enable(EmbeddedFunctionEnableA::new().with_sflp_game_enable(true))
            .await?;
        use lsm6dsv320x::EmbeddedFunctionFifoA;

        // This is the quaternion, but it spans two fifo words and is a pain to work with.
        // lsm.embedded_functions_fifo(EmbeddedFunctionFifoA::new().with_sflp_game_fifo_enable(true))
        //     .await?;

        // This the gravity vector which is just three values long :)
        // lsm.embedded_functions_fifo(
        //     EmbeddedFunctionFifoA::new().with_sflp_gravity_fifo_enable(true),
        // )
        // .await?;

        // Both quaternion and gravity
        lsm.embedded_functions_fifo(
            EmbeddedFunctionFifoA::new()
                .with_sflp_gravity_fifo_enable(true)
                .with_sflp_game_fifo_enable(true),
        )
        .await?;
    }

    Ok(())
}

async fn configure_icm(icm: &mut ICM) -> Result<(), ICMError> {
    defmt::info!("Detected ICM");
    let _ = icm.reset().await;
    Timer::after_millis(10).await;

    // set the power register.
    use icm42688::{PowerConfig, SensorMode};
    icm.control_power(PowerConfig {
        gyroscope: SensorMode::LowNoise,
        acceleration: SensorMode::LowNoise,
    })
    .await?;

    // Set the rates.
    use icm42688::{GyroscopeConfig, GyroscopeOutputDataRate, GyroscopeScale};
    icm.control_gyro(GyroscopeConfig {
        scale: GyroscopeScale::Dps2000,
        rate: GyroscopeOutputDataRate::Hz8k,
    })
    .await?;

    use icm42688::{AccelerationConfig, AccelerationOutputDataRate, AccelerationScale};
    icm.control_accel(AccelerationConfig {
        scale: AccelerationScale::G2,
        rate: AccelerationOutputDataRate::Hz8k,
    })
    .await?;

    // Wait the required time after enabling the sensors.
    Timer::after_millis(40).await;

    // Enable the fifo.
    use icm42688::FifoConfig;
    icm.control_fifo_config(FifoConfig {
        resume_partial: true,
        watermark_gt_persist: true,
        high_resolution: false,
        fsync: false,
        batch_temperature: true,
        batch_gyro: true,
        batch_accel: true,
        watermark: 0,
    })
    .await?;
    //
    icm.control_fifo_mode(icm42688::FifoMode::StreamToFifo)
        .await?;
    Ok(())
}

async fn _icm_test<SPI: embedded_hal_async::spi::SpiDevice>(
    mut icm: ICM42688<SPI>,
) -> Result<(), icm42688::Error<<SPI as embedded_hal_async::spi::ErrorType>::Error>> {
    defmt::info!("Detected ICM");
    let _ = icm.reset().await;
    Timer::after_millis(10).await;

    // set the power register.
    use icm42688::{PowerConfig, SensorMode};
    icm.control_power(PowerConfig {
        gyroscope: SensorMode::LowNoise,
        acceleration: SensorMode::LowNoise,
    })
    .await?;

    // Set the rates.
    use icm42688::{GyroscopeConfig, GyroscopeOutputDataRate, GyroscopeScale};
    icm.control_gyro(GyroscopeConfig {
        scale: GyroscopeScale::Dps2000,
        rate: GyroscopeOutputDataRate::Hz200,
    })
    .await?;

    use icm42688::{AccelerationConfig, AccelerationOutputDataRate, AccelerationScale};
    icm.control_accel(AccelerationConfig {
        scale: AccelerationScale::G2,
        rate: AccelerationOutputDataRate::Hz3_125,
    })
    .await?;

    // Wait the required time after enabling the sensors.
    Timer::after_millis(40).await;

    // Enable the fifo.
    use icm42688::FifoConfig;
    icm.control_fifo_config(FifoConfig {
        resume_partial: true,
        watermark_gt_persist: true,
        high_resolution: false,
        fsync: false,
        batch_temperature: true,
        batch_gyro: true,
        batch_accel: true,
        watermark: 0,
    })
    .await?;
    //
    icm.control_fifo_mode(icm42688::FifoMode::StreamToFifo)
        .await?;

    loop {
        Timer::after_millis(100).await;
        let r = icm.read_temperature().await?;
        defmt::info!("t: {:?}", r);
        let a = icm.read_acceleration().await?;
        let g = icm.read_gyroscope().await?;
        defmt::info!("a: {:?},  g {:?}", a, g);
        let f = icm.read_fifo_count().await?;
        defmt::info!("f: {:?},  ", f);

        let mut buffer = [0u8; 128];
        let buffer_len = buffer.len();
        icm.get_fifo(&mut buffer[0..buffer_len.min(f as usize)])
            .await?;
        defmt::info!("b: {:?}", buffer);
    }
}

async fn _lsm_test<LsmSPI: SpiDevice>(
    mut lsm: LSM6DSV320X<LsmSPI>,
) -> Result<(), lsm6dsv320x::Error<LsmSPI::Error>> {
    lsm.reset().await?;
    Timer::after_millis(20).await; // wait a bit after the reset.

    // Low accelerometer setup;
    use lsm6dsv320x::{AccelerationMode, AccelerationModeDataRate, OutputDataRate};
    lsm.control_acceleration(AccelerationModeDataRate {
        mode: AccelerationMode::HighPerformance,
        rate: OutputDataRate::Hz480,
    })
    .await?;
    use lsm6dsv320x::{AccelerationFilterScale, AccelerationScale};
    lsm.filter_acceleration(AccelerationFilterScale {
        scale: AccelerationScale::G2,
    })
    .await?;

    // High acceleratometer setup;
    use lsm6dsv320x::{
        AccelerationDataRateHigh, AccelerationModeDataRateHigh, AccelerationScaleHigh,
    };
    lsm.control_acceleration_high(AccelerationModeDataRateHigh {
        scale: AccelerationScaleHigh::G320,
        rate: AccelerationDataRateHigh::Hz3840,
        ..Default::default()
    })
    .await?;

    // Gyroscope setup.
    use lsm6dsv320x::{GyroscopeMode, GyroscopeModeDataRate};
    lsm.control_gyroscope(GyroscopeModeDataRate {
        mode: GyroscopeMode::HighPerformance,
        rate: OutputDataRate::Hz3840,
    })
    .await?;
    use lsm6dsv320x::{GyroscopeBandwidthScale, GyroscopeScale};
    lsm.filter_gyroscope(GyroscopeBandwidthScale {
        scale: GyroscopeScale::DPS4000,
    })
    .await?;

    // Setup fifo.
    use lsm6dsv320x::{FifoControl, FifoMode, TemperatureBatch, TimestampBatch};
    lsm.control_fifo(FifoControl {
        timestamp: TimestampBatch::Disabled,
        mode: FifoMode::Continuous,
        temperature: TemperatureBatch::Hz60,
    })
    .await?;
    use lsm6dsv320x::FifoBatch;
    lsm.control_fifo_batch(FifoBatch {
        gyroscope: OutputDataRate::Hz3840,
        acceleration: OutputDataRate::Hz3840,
    })
    .await?;
    // And this last one to start collecting high G samples to the fifo.
    use lsm6dsv320x::{BatchDataRateConfig, TriggerBDRSource};
    lsm.control_bdr_config(BatchDataRateConfig {
        trigger_bdr: TriggerBDRSource::Acceleration,
        batch_acceleration_high: true,
        threshold: 0,
    })
    .await?;

    if false {
        let buffer = {
            const LEN: usize = 1792;
            static FIFO_BUFFER: StaticCell<[u8; LEN]> = StaticCell::new();
            FIFO_BUFFER.init([0u8; LEN])
        };
        loop {
            let s = lsm.get_fifo_status().await?;

            let b = embassy_time::Instant::now();
            lsm.get_fifo(&mut buffer[0..(s.unread() as usize) * 7])
                .await?;
            let e = embassy_time::Instant::now();
            defmt::info!("s: {:?} took: {} us", s, (e - b).as_micros());
            defmt::info!("b: {:?}", buffer[0..6]);
            // Okay, we can keep up with the data rate, it takes about 240 us to transfer 30 samples of
            // 7 bytes each. Even with a 1ms delay we can keep up.

            Timer::after_millis(1).await;
        }
    }

    Ok(())
}
