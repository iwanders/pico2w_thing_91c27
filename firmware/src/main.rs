//! This example tests the RP Pico 2 W onboard LED.
//!
//! It does not work with the RP Pico 2 board. See `blinky.rs`.

#![no_std]
#![no_main]

use cyw43_pio::{PioSpi, RM2_CLOCK_DIVIDER};
use defmt::{info, unwrap};
use embassy_executor::Spawner;
use embassy_rp::bind_interrupts;
use embassy_rp::gpio::{Level, Output};
use embassy_rp::peripherals::{DMA_CH0, PIO0};
use embassy_rp::pio::{InterruptHandler as PioInterruptHandler, Pio};
use embassy_time::{Duration, Timer};
use static_cell::StaticCell;
use {defmt_rtt as _, panic_probe as _};

use embassy_rp::peripherals::USB;
use embassy_rp::usb::{Driver, Instance, InterruptHandler as UsbInterruptHandler};
use embassy_usb::class::cdc_acm::{CdcAcmClass, State as CdcState};
use embassy_usb::driver::EndpointError;
use embassy_usb::UsbDevice;

mod usb_picotool_reset;

// Program metadata for `picotool info`.
// This isn't needed, but it's recommended to have these minimal entries.
#[unsafe(link_section = ".bi_entries")]
#[used]
pub static PICOTOOL_ENTRIES: [embassy_rp::binary_info::EntryAddr; 4] = [
    embassy_rp::binary_info::rp_program_name!(c"Blinky Example"),
    embassy_rp::binary_info::rp_program_description!(
        c"This example tests the RP Pico 2 W's onboard LED, connected to GPIO 0 of the cyw43 \
        (WiFi chip) via PIO 0 over the SPI bus."
    ),
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

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let p = embassy_rp::init(Default::default());

    let mut watchdog = embassy_rp::watchdog::Watchdog::new(p.WATCHDOG);
    // Create the driver, from the HAL.
    let driver = Driver::new(p.USB, Irqs);

    // Create embassy-usb Config
    let config = {
        // https://github.com/raspberrypi/picotool/blob/de8ae5ac334e1126993f72a5c67949712fd1e1a4/picoboot_connection/picoboot_connection.c#L94
        // https://github.com/raspberrypi/picotool/blob/de8ae5ac334e1126993f72a5c67949712fd1e1a4/picoboot_connection/picoboot_connection.h#L24
        const RPI_VENDOR_ID: u16 = 0x2e8a;
        const RPI_PRODUCT_ID: u16 = 0x0009;
        let mut config = embassy_usb::Config::new(RPI_VENDOR_ID, RPI_PRODUCT_ID);
        config.manufacturer = Some("Embassy");
        config.product = Some("USB-serial example");
        config.serial_number = Some("12345678");
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
    let mut cdc_class = {
        static STATE: StaticCell<CdcState> = StaticCell::new();
        let state = STATE.init(CdcState::new());
        let s = CdcAcmClass::new(&mut builder, state, 64);
        s
    };

    let reset_class = {
        static STATE: StaticCell<usb_picotool_reset::State> = StaticCell::new();
        let state = STATE.init(usb_picotool_reset::State::new());
        usb_picotool_reset::PicoResetClass::new(&mut builder, state)
    };

    // Build the builder.
    let usb = builder.build();

    // Run the USB device.
    unwrap!(spawner.spawn(usb_task(usb)));

    for _i in 0..5 {
        let delay = Duration::from_millis(250);
        Timer::after(delay).await;
        let _ = embassy_time::with_timeout(
            Duration::from_millis(5),
            cdc_class.write_packet("wait a bit\n".as_bytes()),
        )
        .await;
    }

    if false {
        let fw = include_bytes!("../../../cyw43-firmware/43439A0.bin");
        let clm = include_bytes!("../../../cyw43-firmware/43439A0_clm.bin");
        //let fw = &[];
        //let clm = &[];

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
            //DEFAULT_CLOCK_DIVIDER,
            pio.irq0,
            cs,
            p.PIN_24, // dio
            p.PIN_29, // clk
            p.DMA_CH0,
        );

        let _ = embassy_time::with_timeout(
            Duration::from_millis(5),
            cdc_class.write_packet("doing things\n".as_bytes()),
        )
        .await;

        static STATE: StaticCell<cyw43::State> = StaticCell::new();
        let state = STATE.init(cyw43::State::new());

        let _ = embassy_time::with_timeout(
            Duration::from_millis(5),
            cdc_class.write_packet("cell made\n".as_bytes()),
        )
        .await;

        // This looks to be where the firmware upload happens.
        let (_net_device, mut control, runner) = cyw43::new(state, pwr, spi, fw).await;

        let _ = embassy_time::with_timeout(
            Duration::from_millis(5),
            cdc_class.write_packet("new cyw43\n".as_bytes()),
        )
        .await;
        // This is where we stall.
        let s = spawner.spawn(cyw43_task(runner));
        if let Err(e) = s {
            let _ = embassy_time::with_timeout(
                Duration::from_millis(5),
                cdc_class.write_packet("setup failed\n".as_bytes()),
            )
            .await;
        } else {
            let _ = embassy_time::with_timeout(
                Duration::from_millis(5),
                cdc_class.write_packet("setup good\n".as_bytes()),
            )
            .await;
        }

        control.init(clm).await;
        control
            .set_power_management(cyw43::PowerManagementMode::PowerSave)
            .await;

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
    }
    let delay = Duration::from_millis(250);
    let mut counter = 0;
    loop {
        info!("led on!");
        //control.gpio_set(0, true).await;
        Timer::after(delay).await;

        info!("led off!");
        //control.gpio_set(0, false).await;
        Timer::after(delay).await;

        counter += 1;
        if counter > 5 {
            let _ = embassy_time::with_timeout(
                Duration::from_millis(5),
                cdc_class.write_packet("trying reboot, here's hoping.\n".as_bytes()),
            )
            .await;
            usb_picotool_reset::boot_to_bootsel(Some(&mut watchdog));
        }
    }
    /**/
}

type MyUsbDriver = Driver<'static, USB>;
type MyUsbDevice = UsbDevice<'static, MyUsbDriver>;

#[embassy_executor::task]
async fn usb_task(mut usb: MyUsbDevice) -> ! {
    usb.run().await
}

struct Disconnected {}

impl From<EndpointError> for Disconnected {
    fn from(val: EndpointError) -> Self {
        match val {
            EndpointError::BufferOverflow => panic!("Buffer overflow"),
            EndpointError::Disabled => Disconnected {},
        }
    }
}

async fn echo<'d, T: Instance + 'd>(
    class: &mut CdcAcmClass<'d, Driver<'d, T>>,
) -> Result<(), Disconnected> {
    let mut buf = [0; 64];
    loop {
        let n = class.read_packet(&mut buf).await?;
        let data = &buf[..n];
        info!("data: {:x}", data);
        class.write_packet(data).await?;
    }
}
