use embassy_executor::Spawner;
use embassy_rp::gpio::Output;
use embassy_rp::peripherals::USB;
use embassy_rp::usb::Driver;
use embassy_time::{Duration, Timer};
use embassy_usb::class::cdc_acm::CdcAcmClass;
use embedded_hal_async::spi::SpiDevice;

use crate::icm42688;
use crate::icm42688::ICM42688;
use crate::lsm6dsv320x;
use crate::lsm6dsv320x::LSM6DSV320X;
use defmt::{info, warn};
use static_cell::StaticCell;

#[embassy_executor::task]
async fn lsm_task(mut indicator: Output<'static>) -> ! {
    let delay = Duration::from_millis(50);
    loop {
        info!("led on!");

        indicator.set_high();

        Timer::after(delay).await;

        info!("led off!");

        indicator.set_low();
        Timer::after(delay).await;
    }
}

pub async fn imu_entry<
    IcmSPI: embedded_hal_async::spi::SpiDevice,
    LsmSPI: embedded_hal_async::spi::SpiDevice,
>(
    spawner: Spawner,
    icm: ICM42688<IcmSPI>,
    lsm: LSM6DSV320X<LsmSPI>,
    cdc: CdcAcmClass<'static, Driver<'static, USB>>,
    output_pin: Output<'static>,
) {
    defmt::info!("Setting up imu!");

    spawner.spawn(lsm_task(output_pin)).unwrap();

    // _lsm_test(lsm).await.unwrap();
    // _icm_test(icm).await.unwrap();
}

async fn _lsm_test<LsmSPI: embedded_hal_async::spi::SpiDevice>(
    mut lsm: LSM6DSV320X<LsmSPI>,
) -> Result<(), lsm6dsv320x::Error<<LsmSPI as embedded_hal_async::spi::ErrorType>::Error>> {
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
        rate: AccelerationDataRateHigh::Hz480,
        ..Default::default()
    })
    .await?;

    // Gyroscope setup.
    use lsm6dsv320x::{GyroscopeMode, GyroscopeModeDataRate};
    lsm.control_gyroscope(GyroscopeModeDataRate {
        mode: GyroscopeMode::HighPerformance,
        rate: OutputDataRate::Hz480,
    })
    .await?;
    use lsm6dsv320x::{GyroscopeBandwidthScale, GyroscopeScale};
    lsm.filter_gyroscope(GyroscopeBandwidthScale {
        scale: GyroscopeScale::DPS4000,
    })
    .await?;

    // Setup fifo.
    use lsm6dsv320x::{FifoControl, FifoMode, TemperatureBatch};
    lsm.control_fifo(FifoControl {
        mode: FifoMode::Continuous,
        temperature: TemperatureBatch::Hz60,
    })
    .await?;
    use lsm6dsv320x::FifoBatch;
    lsm.control_fifo_batch(FifoBatch {
        gyroscope: OutputDataRate::Hz480,
        acceleration: OutputDataRate::Hz480,
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
/*
 * let mut lsm_test = async move || -> Result<(), OurError> {

     Ok(())
 };
 let r = lsm_test().await;
*/
