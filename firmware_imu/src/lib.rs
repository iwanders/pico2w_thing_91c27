#![cfg_attr(target_arch = "arm", no_std)]
#![cfg_attr(not(test), no_main)]

use rp2350_support::{defmt_serial, static_files};

#[cfg(target_arch = "arm")]
use rp2350_support::{rp2350_util, usb_picotool_reset};

pub mod icm42688;
#[cfg(target_arch = "arm")]
pub mod imu;
pub mod lsm6dsv320x;

#[cfg(target_arch = "arm")]
pub mod program {
    #![allow(unreachable_code)]
    #![allow(unused_mut)]

    use super::*;

    use embassy_executor::Spawner;

    use defmt::{error, info, unwrap};
    use embassy_rp::gpio::{Level, Output};

    use embassy_rp::bind_interrupts;
    use embassy_time::{Duration, Timer};

    //use defmt_rtt as _;
    use static_cell::StaticCell;

    use embassy_rp::peripherals::USB;
    use embassy_rp::usb::{Driver, InterruptHandler as UsbInterruptHandler};
    use embassy_usb::class::cdc_acm::{CdcAcmClass, State as CdcState};
    use embassy_usb::UsbDevice;

    //use super::bme280::BME280;

    // List of files in this project (yes it could be created from build.rs), but this is fine for now.
    // These files are used to look up against when a panic happens.
    const PANIC_HANDLER_FILE_LIST: &[&str] = &[
        "lib.rs",
        "defmt_serial.rs",
        "rp2350_util.rs",
        "usb_picotool_reset.rs",
        "hw_test.rs",
        "lsm6dsv320x.rs",
        "mx25.rs",
        "hap.rs",
        "flash_memory.rs",
        "imu.rs",
    ];

    // Program metadata for `picotool info`.
    // This isn't needed, but it's recommended to have these minimal entries.
    #[unsafe(link_section = ".bi_entries")]
    #[used]
    pub static PICOTOOL_ENTRIES: [embassy_rp::binary_info::EntryAddr; 4] = [
        embassy_rp::binary_info::rp_program_name!(c"Pico Sensor Board"),
        embassy_rp::binary_info::rp_program_description!(c"Pico2w_thingy_91c27"),
        embassy_rp::binary_info::rp_cargo_version!(),
        embassy_rp::binary_info::rp_program_build_attribute!(),
    ];

    //use embassy_rp::pio::InterruptHandler as PioInterruptHandler;
    bind_interrupts!(pub struct Irqs {
        //PIO0_IRQ_0 => PioInterruptHandler<embassy_rp::peripherals::PIO0>;
        USBCTRL_IRQ => UsbInterruptHandler<USB>;
        //PIO1_IRQ_0 => embassy_rp::pio:: InterruptHandler<embassy_rp::peripherals::PIO1>;
    });

    #[embassy_executor::task]
    async fn defmt_serial_task(
        serial_logger: defmt_serial::SerialLogger<Driver<'static, USB>>,
    ) -> ! {
        defmt_serial::run(serial_logger).await
    }

    type MyUsbDriver = Driver<'static, USB>;
    type MyUsbDevice = UsbDevice<'static, MyUsbDriver>;

    #[embassy_executor::task]
    async fn usb_task(mut usb: MyUsbDevice) -> ! {
        usb.run().await
    }

    pub async fn main(spawner: Spawner) {
        let p = embassy_rp::init(Default::default());
        rp2350_util::panic_info_scratch::set_panic_files(PANIC_HANDLER_FILE_LIST);

        // Create the driver, from the HAL.
        let driver = Driver::new(p.USB, Irqs);

        // Create the USB serial id used.
        let serial_id_string = {
            let chipinfo = rp2350_util::chip_info::get_chip_info();
            static STATE: StaticCell<rp2350_util::chip_info::SerialAscii> = StaticCell::new();
            STATE.init(chipinfo.serial_ascii())
        };

        // Create embassy-usb Config
        let config = {
            // https://github.com/raspberrypi/picotool/blob/de8ae5ac334e1126993f72a5c67949712fd1e1a4/picoboot_connection/picoboot_connection.c#L94
            // https://github.com/raspberrypi/picotool/blob/de8ae5ac334e1126993f72a5c67949712fd1e1a4/picoboot_connection/picoboot_connection.h#L24
            // We MUST use these id's for the picoboot tool to be able to find the reset endpoint and boot the device into boot mode.
            const RPI_VENDOR_ID: u16 = 0x2e8a;
            const RPI_PRODUCT_ID: u16 = 0x0009;
            let mut config = embassy_usb::Config::new(RPI_VENDOR_ID, RPI_PRODUCT_ID);
            // Maybe this emoji will find some fun bugs xD
            config.manufacturer = Some(core::str::from_utf8(&[0xf0u8, 0x9f, 0x8e, 0x89]).unwrap());
            config.product = Some("Pico Sensor Board");
            config.serial_number = Some(serial_id_string.as_str());
            config.max_power = 250;
            config.max_packet_size_0 = 64;
            config
        };

        // Create embassy-usb DeviceBuilder using the driver and config.
        // It needs some buffers for building the descriptors.
        let mut builder = {
            static CONFIG_DESCRIPTOR: StaticCell<[u8; 256 * 2]> = StaticCell::new();
            static BOS_DESCRIPTOR: StaticCell<[u8; 256 * 2]> = StaticCell::new();
            static CONTROL_BUF: StaticCell<[u8; 64 * 2]> = StaticCell::new();

            let builder = embassy_usb::Builder::new(
                driver,
                config,
                CONFIG_DESCRIPTOR.init([0; 256 * 2]),
                BOS_DESCRIPTOR.init([0; 256 * 2]),
                &mut [], // no msos descriptors
                CONTROL_BUF.init([0; 64 * 2]),
            );
            builder
        };

        {
            static STATE: StaticCell<usb_picotool_reset::State> = StaticCell::new();
            let state = STATE.init(usb_picotool_reset::State::new());
            usb_picotool_reset::PicoResetClass::new(&mut builder, state)
        };

        // Create classes on the builder.
        let cdc_class_full = {
            static STATE: StaticCell<CdcState> = StaticCell::new();
            let state = STATE.init(CdcState::new());
            let s = CdcAcmClass::new(&mut builder, state, 64);
            s
        };

        let mut data_cdc: CdcAcmClass<'_, Driver<'_, USB>> = {
            static STATEZ: StaticCell<CdcState> = StaticCell::new();
            let state = STATEZ.init(CdcState::new());
            let s = CdcAcmClass::new(&mut builder, state, 64);
            s
        };

        // Build the builder.
        let usb = builder.build();

        // Run the USB device.
        unwrap!(spawner.spawn(usb_task(usb)));

        let (tx, mut rx) = cdc_class_full.split();

        let logger = defmt_serial::SerialLogger::new(tx);
        let s = spawner.spawn(defmt_serial_task(logger));
        if let Err(e) = s {
            info!("setup failed: {:?}", e);
        } else {
            info!("setup good");
        }

        for _i in 0..10 {
            let delay = Duration::from_millis(250);
            Timer::after(delay).await;
            info!("wait a bit");
        }
        info!("checking panic.");

        let mut w = embassy_rp::watchdog::Watchdog::new(p.WATCHDOG);

        // Serial is live, check if we previously had a panic, if so print its information.
        if let Some(panic_info) = rp2350_util::panic_info_scratch::take_panic(&mut w) {
            error!("Panicked:  {}:{}", panic_info.file(), panic_info.line());
            let delay = Duration::from_millis(5000);
            Timer::after(delay).await;
        }
        info!("Printing sysid");

        let delay = Duration::from_millis(500);
        Timer::after(delay).await;
        info!("sys id: {:?}", rp2350_util::chip_info::get_chip_info());
        let chipid = rp2350_util::otp::get_otp_chipid();
        info!(
            "sys id: {:x} {:x} {:x} {:x}",
            chipid[0], chipid[1], chipid[2], chipid[3]
        );

        // Test section
        let mut indicator = Output::new(p.PIN_26, Level::Low);
        let delay = Duration::from_millis(250);
        // imu::imu_entry(spawner);

        // LSM
        let lsm = {
            use embassy_rp::spi::{Config, Spi};
            let mut cs = Output::new(p.PIN_13, Level::High);
            let mut config = Config::default();

            // Max clock is 10 MHz for SPI.
            config.frequency = 4_000_000;

            let tx_dma = p.DMA_CH1;
            let rx_dma = p.DMA_CH2;
            let mut spi = Spi::new(p.SPI1, p.PIN_10, p.PIN_11, p.PIN_12, tx_dma, rx_dma, config);
            use embassy_embedded_hal::shared_bus::asynch::spi::SpiDevice;
            use embassy_sync::blocking_mutex::raw::NoopRawMutex;
            use embassy_sync::mutex::Mutex;
            static SPI_BUS: StaticCell<
                Mutex<NoopRawMutex, Spi<'_, embassy_rp::peripherals::SPI1, embassy_rp::spi::Async>>,
            > = StaticCell::new();
            let spi_bus = Mutex::new(spi);
            let spi_bus = SPI_BUS.init(spi_bus);
            let device = SpiDevice::new(spi_bus, cs);

            let mut lsm = lsm6dsv320x::LSM6DSV320X::new(device).await;

            match lsm {
                Ok(lsm) => lsm,
                Err(e) => {
                    defmt::error!("No lsm: {:?}", e);
                    loop {
                        indicator.set_high();
                        Timer::after(delay).await;
                        indicator.set_low();
                        Timer::after(delay).await;
                    }
                }
            }
        };

        let icm = {
            // ICM
            // spi: Peri<'static, embassy_rp::peripherals::SPI0>,
            // cs: Peri<'static, embassy_rp::peripherals::PIN_5>,
            // clk: Peri<'static, embassy_rp::peripherals::PIN_2>,
            // mosi: Peri<'static, embassy_rp::peripherals::PIN_3>,
            // miso: Peri<'static, embassy_rp::peripherals::PIN_4>,
            use embassy_rp::spi::{Config, Spi};
            let mut cs = Output::new(p.PIN_5, Level::High);
            let mut config = Config::default();

            // Max SPI rate is 24 O_o
            config.frequency = 24_000_000;

            let tx_dma = p.DMA_CH3;
            let rx_dma = p.DMA_CH4;
            let mut spi = Spi::new(p.SPI0, p.PIN_2, p.PIN_3, p.PIN_4, tx_dma, rx_dma, config);

            use embassy_embedded_hal::shared_bus::asynch::spi::SpiDevice;
            use embassy_sync::blocking_mutex::raw::NoopRawMutex;
            use embassy_sync::mutex::Mutex;
            static SPI_BUS: StaticCell<
                Mutex<NoopRawMutex, Spi<'_, embassy_rp::peripherals::SPI0, embassy_rp::spi::Async>>,
            > = StaticCell::new();
            let spi_bus = Mutex::new(spi);
            let spi_bus = SPI_BUS.init(spi_bus);
            let device = SpiDevice::new(spi_bus, cs);

            let mut icm = icm42688::ICM42688::new(device).await;
            match icm {
                Ok(icm) => icm,
                Err(e) => {
                    defmt::warn!("Failed to detect ICM: {:?}", e);
                    loop {
                        indicator.set_high();
                        Timer::after(delay).await;
                        indicator.set_low();
                        Timer::after(delay).await;
                    }
                }
            }
        };

        // Hand the data bus and the imu's over to the imu handling for task setup.
        imu::imu_entry(spawner, icm, lsm, data_cdc, indicator).await;

        /*
        let mut counter = 0;



        loop {
            info!("led on!");

            indicator.set_high();

            Timer::after(delay).await;

            info!("led off!");

            indicator.set_low();
            Timer::after(delay).await;

            let mut buf: [u8; 64] = [0u8; 64];
            if let Ok(Ok(n)) =
                embassy_time::with_timeout(Duration::from_millis(100), rx.read_packet(&mut buf))
                    .await
            {
                let data = &buf[..n];
                info!("data: {:x}", data);
            }
            defmt::info!("counter: {}", counter);
            counter += 1;
        }
        */
    }
}

// https://github.com/rust-lang/rust/blob/1bd4fdc943513e1004f498bbf289279c9784fc6f/compiler/rustc_data_structures/src/macros.rs#L3
#[macro_export]
macro_rules! static_assert_size {
    ($ty:ty, $size:expr) => {
        const _: [(); $size] = [(); ::core::mem::size_of::<$ty>()];
    };
}
