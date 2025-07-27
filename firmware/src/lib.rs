#![cfg_attr(target_arch = "arm", no_std)]
#![cfg_attr(not(test), no_main)]
pub mod bme280;

pub mod defmt_serial;

pub mod static_files;

#[cfg(target_arch = "arm")]
pub mod hw_test;
#[cfg(target_arch = "arm")]
pub mod i2s_input;
#[cfg(target_arch = "arm")]
pub mod rp2350_util;
#[cfg(target_arch = "arm")]
pub mod usb_picotool_reset;

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

    use super::bme280::BME280;

    // List of files in this project (yes it could be created from build.rs), but this is fine for now.
    // These files are used to look up against when a panic happens.
    const PANIC_HANDLER_FILE_LIST: &[&str] = &[
        "lib.rs",
        "defmt_serial.rs",
        "rp2350_util.rs",
        "usb_picotool_reset.rs",
        "hw_test.rs",
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

    bind_interrupts!(struct Irqs {
        //PIO0_IRQ_0 => PioInterruptHandler<PIO0>;
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
            static CONFIG_DESCRIPTOR: StaticCell<[u8; 256]> = StaticCell::new();
            static BOS_DESCRIPTOR: StaticCell<[u8; 256]> = StaticCell::new();
            static CONTROL_BUF: StaticCell<[u8; 64]> = StaticCell::new();

            let builder = embassy_usb::Builder::new(
                driver,
                config,
                CONFIG_DESCRIPTOR.init([0; 256]),
                BOS_DESCRIPTOR.init([0; 256]),
                &mut [], // no msos descriptors
                CONTROL_BUF.init([0; 64]),
            );
            builder
        };

        // Create classes on the builder.
        let cdc_class_full = {
            static STATE: StaticCell<CdcState> = StaticCell::new();
            let state = STATE.init(CdcState::new());
            let s = CdcAcmClass::new(&mut builder, state, 64);
            s
        };

        {
            static STATE: StaticCell<usb_picotool_reset::State> = StaticCell::new();
            let state = STATE.init(usb_picotool_reset::State::new());
            usb_picotool_reset::PicoResetClass::new(&mut builder, state)
        };

        // Build the builder.
        let usb = builder.build();

        // Run the USB device.
        unwrap!(spawner.spawn(usb_task(usb)));

        let (tx, mut rx) = cdc_class_full.split();

        //defmt_serial::runner(spawner, defmt_serial::WritableSerial::new(tx)).await;
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

        //let flash_dev_info = rp2350_util::chip_info::get_flash_dev_info();
        //info!("flash_dev_info id: {:?}", flash_dev_info);

        // Create the driver, from the HAL.
        /*
         */

        // Do stuff with the class!
        /*
        loop {
            cdc_class.wait_connection().await;
            info!("Connected");
            let _ = echo(&mut cdc_class).await;
            info!("Disconnected");
        }
        */

        // Test section
        //*
        info!("Going into test.");
        //hw_test::hw_test(unsafe { embassy_rp::Peripherals::steal() }).await;
        hw_test::test_wifi(unsafe { embassy_rp::Peripherals::steal() }, spawner).await;
        // */
        let mut indicator = Output::new(p.PIN_26, Level::Low);
        let delay = Duration::from_millis(250);

        /*
        struct BmePinTransfer {
            i2c: embassy_rp::peripherals::I2C0,
            sda: embassy_rp::peripherals::PIN_20,
            scl: embassy_rp::peripherals::PIN_21,
        }
        async fn test_bme(p: BmePinTransfer) {
            use embassy_rp::i2c::{Config, I2c};
            let config = Config::default();
            //config.frequency = 100_000;
            let mut i2c = I2c::new_blocking(p.i2c, p.scl, p.sda, config);
        */

        if false {
            use bme280::reg::*;
            use embassy_rp::i2c::I2c;
            use embassy_rp::i2c::InterruptHandler as I2cInterruptHandler;
            use embassy_rp::peripherals::I2C0;
            bind_interrupts!(struct Irqs {
                I2C0_IRQ => I2cInterruptHandler<I2C0>;
            });
            let mut config = embassy_rp::i2c::Config::default();
            config.frequency = 1_000_000; // gotta go fast!
            let i2c = I2c::new_async(p.I2C0, p.PIN_21, p.PIN_20, Irqs, config);
            let bme280_dev = BME280::new(bme280::ADDRESS_DEFAULT, i2c).await;
            defmt::debug!("bme280 dev: {:?}", bme280_dev);
            if let Ok(mut d) = bme280_dev {
                let _ = d.reset().await;
                let temp_sampling = bme280::Sampling::X1;
                let press_sampling = bme280::Sampling::X1;
                let mode = bme280::Mode::Forced;
                let humidity_sampling = bme280::Sampling::X1;
                defmt::debug!("status: {:b}", d.get_register(REG_BME280_STATUS).await);
                loop {
                    // Do this weird dance to activate the control register for humidity before triggering a value.
                    let _ = d.set_ctrl_hum(humidity_sampling).await;
                    let _ = d.set_ctrl_meas(temp_sampling, press_sampling, mode).await;
                    Timer::after_millis(10).await;
                    defmt::debug!("status: {:b}", d.get_register(REG_BME280_STATUS).await);
                    let readout = d.readout().await;
                    defmt::debug!("readout: {:?}", readout);
                    if let Ok(readout) = readout {
                        let compensated = d.compensation().compensate(&readout);
                        defmt::debug!("compensated: {:?}", compensated);
                    }
                }
            }
        }

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

            /*
            if counter > 20 && false {
                panic!();
            }
            if counter > 80 && false {
                error!("reboot in {}", delay);
                Timer::after(delay).await;

                //usb_picotool_reset::boot_to_bootsel_watchdog(&mut watchdog);
                rp2350_util::reboot::reboot(RebootSettings::flash(), Duration::from_millis(100));
            }*/
        }
        /**/
    }
}
