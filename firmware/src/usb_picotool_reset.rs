//! RPi picotool reset class usb interface handler.

// https://github.com/raspberrypi/pico-sdk/blob/9a4113fbbae65ee82d8cd6537963bc3d3b14bcca/src/common/pico_usb_reset_interface_headers/include/pico/usb_reset_interface.h
// Do we need the whole Microsoft OS 2.0 descriptor stuff??
//

use core::ops::BitXor;
use defmt::{error, info, println, warn};

use embassy_rp::rom_data;
use embassy_usb::control::{InResponse, OutResponse, Request};
use embassy_usb::driver::Driver;

use embassy_usb::{Builder, Handler};

pub const PICO_RESET_CLASS: u8 = 0xFF;
pub const PICO_RESET_SUBCLASS: u8 = 0x00;
pub const PICO_RESET_INTERFACE_PROTOCOL: u8 = 0x01;

use core::mem::MaybeUninit;

// Section 5.4.8.24. reboot, page 395
// Normal:
const PICO_BOOTROM_REBOOT_FLAGS_NORMAL: u32 = 0x0000;
// p0 is the boot diagnostic partition, lower 8 bits only.
// Bootsel:
// P0 is GPIO for actvitity indicator.
const PICO_BOOTROM_REBOOT_FLAGS_BOOTSEL: u32 = 0x02;
const PICO_BOOTROM_REBOOT_FLAGS_BOOTSEL_P1_DISABLE_MSD: u32 = 0x01;
const PICO_BOOTROM_REBOOT_FLAGS_BOOTSEL_P1_DISABLE_PICOBOOT: u32 = 0x02;

// Can be OR with any of these flags.
const PICO_BOOTROM_REBOOT_NO_RETURN_ON_SUCCESS: u32 = 0x0100;

struct Control {
    index: u16,
}

impl Control {
    fn new(index: u16) -> Self {
        Control { index }
    }
}

#[allow(unused)]
pub fn boot_to_bootsel_watchdog(w: &mut embassy_rp::watchdog::Watchdog) -> ! {
    // boot flags
    w.set_scratch(2, PICO_BOOTROM_REBOOT_FLAGS_BOOTSEL_P1_DISABLE_MSD);
    // gpio index.
    w.set_scratch(3, 0);

    // Magic values to trigger bootrom method.
    w.set_scratch(4, 0xb007c0d3);
    w.set_scratch(5, 0x4ff83f2du32.bitxor(0xb007c0d3));
    w.set_scratch(6, 2);
    w.set_scratch(7, 0xb007c0d3);

    // Invoke the watchdog
    w.trigger_reset();
    loop {}
}

pub fn boot_to_bootsel() -> ! {
    // Now we need to do the equivalent of;
    // rom_reset_usb_boot_extra(gpio, (request->wValue & 0x7f) | PICO_STDIO_USB_RESET_BOOTSEL_INTERFACE_DISABLE_MASK, active_low);
    // Well, that exists for the rp2040;
    // https://github.com/embassy-rs/embassy/blob/20ce4d7deb3b16ff32bd68f7adae9066ddd40e94/embassy-rp/src/rom_data/rp2040.rs#L229-L239
    //let f = embassy_rp::rom_data::reset_to_usb_boot::ptr();
    const BOOTSEL_MAIN: u32 =
        PICO_BOOTROM_REBOOT_FLAGS_BOOTSEL | PICO_BOOTROM_REBOOT_NO_RETURN_ON_SUCCESS;
    const DELAY_MS: u32 = 500;
    const P0_GPIO_PIN: u32 = 0;
    const P1_BOOT_PROPERTIES: u32 = PICO_BOOTROM_REBOOT_FLAGS_BOOTSEL_P1_DISABLE_MSD;

    // Well, that disable MSD doesn't stick?
    // The p0 and p1 parameters are generally written to watchdog scratch registers 2 & 3, and are interpreted post-reboot
    // by the boot path code.
    //
    //

    // I think the datasheet is wrong, it doesn't agree with pico-sdk, the following works to inhibit the MSD.
    // https://github.com/raspberrypi/pico-feedback/issues/466
    embassy_rp::rom_data::reboot(BOOTSEL_MAIN, DELAY_MS, P1_BOOT_PROPERTIES, P0_GPIO_PIN);

    // This is never reached.
    loop {}
}

impl Handler for Control {
    fn control_out(&mut self, req: Request, buf: &[u8]) -> Option<OutResponse> {
        // control_out is HostToDevice.
        error!("Got data out control: {:?}", req);
        const RESET_REQUEST_BOOTSEL: u8 = 0x01;
        const RESET_REQUEST_FLASH: u8 = 0x02;
        if req.index == self.index {
            if req.request == RESET_REQUEST_BOOTSEL {
                // sdk does some stuff with bootsel activity leds, probably unnecessary.
                error!("Booting");
                let flags_from_request = (req.value & 0x7f) as u32;
                const DELAY_MS: u32 = 100;
                const BOOTSEL_MAIN: u32 =
                    PICO_BOOTROM_REBOOT_FLAGS_BOOTSEL | PICO_BOOTROM_REBOOT_NO_RETURN_ON_SUCCESS;
                embassy_rp::rom_data::reboot(BOOTSEL_MAIN, DELAY_MS, flags_from_request, 0);
                return Some(OutResponse::Accepted);
            }
            if req.request == RESET_REQUEST_FLASH {
                error!("Normal reboot command, not doing anything.");
                return Some(OutResponse::Accepted);
            }
            return Some(OutResponse::Rejected);
        }
        None
    }
}

/// Internal state for WebUSB
pub struct State {
    control: MaybeUninit<Control>,
}

impl Default for State {
    fn default() -> Self {
        Self::new()
    }
}

impl State {
    /// Create a new `State`.
    pub const fn new() -> Self {
        State {
            control: MaybeUninit::uninit(),
        }
    }
}

pub struct PicoResetClass<'d, D: Driver<'d>> {
    _driver: core::marker::PhantomData<&'d D>,
}

impl<'d, D: Driver<'d>> PicoResetClass<'d, D> {
    pub fn new(builder: &mut Builder<'d, D>, state: &'d mut State) {
        let mut func = builder.function(
            PICO_RESET_CLASS,
            PICO_RESET_SUBCLASS,
            PICO_RESET_INTERFACE_PROTOCOL,
        );
        let mut iface = func.interface();
        let count = iface.interface_number().0;
        let mut alt = iface.alt_setting(
            PICO_RESET_CLASS,
            PICO_RESET_SUBCLASS,
            PICO_RESET_INTERFACE_PROTOCOL,
            None,
        );

        let control = state.control.write(Control::new(count as u16));

        drop(func);

        builder.handler(control);
    }
}
