//! This example tests the RP Pico 2 W onboard LED.
//!
//! It does not work with the RP Pico 2 board. See `blinky.rs`.

#![no_std]
#![no_main]

mod defmt_serial;

pub mod hw_test;
pub mod rp2350_util;

use cyw43_pio::{PioSpi, RM2_CLOCK_DIVIDER};
use defmt::{error, info, unwrap};
use embassy_executor::Spawner;
use embassy_rp::gpio::{Input, Level, Output};
use embassy_rp::peripherals::{DMA_CH0, PIN_1, PIO0, SPI0};
use embassy_rp::pio::{InterruptHandler as PioInterruptHandler, Pio};

use embassy_rp::spi::Polarity;
use embassy_rp::{bind_interrupts, Peripherals};
use embassy_time::{Duration, Timer};

//use defmt_rtt as _;
use static_cell::StaticCell;

use embassy_rp::peripherals::USB;
use embassy_rp::usb::{Driver, InterruptHandler as UsbInterruptHandler};
use embassy_usb::class::cdc_acm::{CdcAcmClass, State as CdcState};
use embassy_usb::UsbDevice;

use crate::rp2350_util::reboot::RebootSettings;

mod usb_picotool_reset;

// List of files in this project (yes it could be created from build.rs), but this is fine for now.
// These files are used to look up against when a panic happens.
const PANIC_HANDLER_FILE_LIST: &'static [&'static str] = &[
    "lib.rs",
    "defmt_serial.rs",
    "rp2350_util.rs",
    "usb_picotool_reset.rs",
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
    PIO0_IRQ_0 => PioInterruptHandler<PIO0>;
    USBCTRL_IRQ => UsbInterruptHandler<USB>;
});

#[embassy_executor::task]
async fn cyw43_task(
    runner: cyw43::Runner<'static, Output<'static>, PioSpi<'static, PIO0, 0, DMA_CH0>>,
) -> ! {
    runner.run().await
}

#[embassy_executor::task]
async fn defmt_serial_task(serial_logger: defmt_serial::SerialLogger) -> ! {
    defmt_serial::run(serial_logger).await
}

type MyUsbDriver = Driver<'static, USB>;
type MyUsbDevice = UsbDevice<'static, MyUsbDriver>;

#[embassy_executor::task]
async fn usb_task(mut usb: MyUsbDevice) -> ! {
    usb.run().await
}

fn setup_wifi() {
    /*
    let mut p : Peripherals = unimplemented!();
    //let fw = include_bytes!("../../../cyw43-firmware/43439A0.bin");
    //let clm = include_bytes!("../../../cyw43-firmware/43439A0_clm.bin");
    //
    const ENABLE_WIFI_LED: bool = false;
    if ENABLE_WIFI_LED {
        let fw = &[];
        let clm = &[];

        // To make flashing faster for development, you may want to flash the firmwares independently
        // at hardcoded addresses, instead of baking them into the program with `include_bytes!`:
        //     probe-rs download ../../cyw43-firmware/43439A0.bin --binary-format bin --chip RP235x --base-address 0x10100000
        //     probe-rs download ../../cyw43-firmware/43439A0_clm.bin --binary-format bin --chip RP235x --base-address 0x10140000
        //let fw = unsafe { core::slice::from_raw_parts(0x10100000 as *const u8, 230321) };
        //let clm = unsafe { core::slice::from_raw_parts(0x10140000 as *const u8, 4752) };

        let pwr = Output::new(p.PIN_23, Level::Low);
        let cs = Output::new(p.PIN_25, Level::High);
        let mut pio = Pio::new(p.PIO0, Irqs);
        let spi = PioSpi::new(
            &mut pio.common,
            pio.sm0,
            RM2_CLOCK_DIVIDER,
            //cyw43_pio::DEFAULT_CLOCK_DIVIDER,
            pio.irq0,
            cs,
            p.PIN_24, // dio
            p.PIN_29, // clk
            p.DMA_CH0,
        );

        info!("doing things");

        static STATE: StaticCell<cyw43::State> = StaticCell::new();
        let state = STATE.init(cyw43::State::new());

        info!("cell made");

        // This looks to be where the firmware upload happens.
        let (_net_device, mut control, runner) = cyw43::new(state, pwr, spi, fw).await;

        info!("new cyw43");

        // This is where we stall.
        let s = spawner.spawn(cyw43_task(runner));
        if let Err(e) = s {
            info!("setup failed: {:?}", e);
        } else {
            info!("setup good");
        }

        control.init(clm).await;
        control
            .set_power_management(cyw43::PowerManagementMode::PowerSave)
            .await;
    }*/
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
        // Maybe this will find some fun bugs xD
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

    let _reset_class = {
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

    for _i in 0..5 {
        let delay = Duration::from_millis(250);
        Timer::after(delay).await;
        info!("wait a bit");
    }

    let mut w = embassy_rp::watchdog::Watchdog::new(p.WATCHDOG);

    // Serial is live, check if we previously had a panic, if so print its information.
    if let Some(panic_info) = rp2350_util::panic_info_scratch::take_panic(&mut w) {
        error!("Panicked:  {}:{}", panic_info.file(), panic_info.line());
        let delay = Duration::from_millis(5000);
        Timer::after(delay).await;
    }

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
    hw_test::hw_test(unsafe { Peripherals::steal() }).await;

    let mut indicator = Output::new(p.PIN_26, Level::Low);
    let delay = Duration::from_millis(250);

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
            embassy_time::with_timeout(Duration::from_millis(100), rx.read_packet(&mut buf)).await
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
