//! RPi picotool reset class usb interface handler.

// https://github.com/raspberrypi/pico-sdk/blob/9a4113fbbae65ee82d8cd6537963bc3d3b14bcca/src/common/pico_usb_reset_interface_headers/include/pico/usb_reset_interface.h
// Do we need the whole Microsoft OS 2.0 descriptor stuff??
//
use core::marker::PhantomData;
use core::ops::BitXor;

use embassy_usb::control::{InResponse, Recipient, Request, RequestType};
use embassy_usb::driver::{Driver, Endpoint, EndpointError, EndpointIn, EndpointOut};
use embassy_usb::types::StringIndex;
use embassy_usb::{Builder, Handler};

const CS_INTERFACE: u8 = 0x24;
const HEADER_SUBTYPE: u8 = 0x01;

pub const PICO_RESET_CLASS: u8 = 0xFF;
pub const PICO_RESET_SUBCLASS: u8 = 0x00;
pub const PICO_RESET_INTERFACE_PROTOCOL: u8 = 0x01;

use core::mem::MaybeUninit;

// This was helpful:
// https://github.com/wezterm/picocalc-wezterm/blob/8dcf8aae0598afdeaf0ed2ba50c39dea6e30c011/src/keyboard.rs#L409
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

struct Control {}

impl Control {
    fn new() -> Self {
        Control {}
    }
}

pub fn boot_to_bootsel(w: Option<&mut embassy_rp::watchdog::Watchdog>) -> ! {
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
    if let Some(w) = w {
        w.set_scratch(2, P0_GPIO_PIN);
        w.set_scratch(3, P1_BOOT_PROPERTIES);

        w.set_scratch(4, 0xb007c0d3);
        w.set_scratch(5, 0x4ff83f2du32.bitxor(0xb007c0d3));
        w.set_scratch(6, 2);
        w.set_scratch(7, 0xb007c0d3);
        w.trigger_reset();
    }

    //embassy_rp::rom_data::reboot(BOOTSEL_MAIN, DELAY_MS, P0_GPIO_PIN, P1_BOOT_PROPERTIES);

    // This is never reached.
    loop {}
}

impl Handler for Control {
    fn control_in(&mut self, req: Request, _data: &mut [u8]) -> Option<InResponse<'_>> {
        // https://github.com/raspberrypi/pico-sdk/blob/9a4113fbbae65ee82d8cd6537963bc3d3b14bcca/src/common/pico_usb_reset_interface_headers/include/pico/usb_reset_interface.h
        const RESET_REQUEST_BOOTSEL: u16 = 0x01;
        const RESET_REQUEST_FLASH: u16 = 0x02;
        if req.value == RESET_REQUEST_BOOTSEL || true {
            // sdk does some stuff with bootsel activity leds, probably unnecessary.
            boot_to_bootsel(None);
        }
        return Some(InResponse::Accepted(&[]));
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

        // Audio control interface
        let mut iface = func.interface();
        let audio_if = iface.interface_number();
        let mut alt = iface.alt_setting(
            PICO_RESET_CLASS,
            PICO_RESET_SUBCLASS,
            PICO_RESET_INTERFACE_PROTOCOL,
            None,
        );

        let control = state.control.write(Control::new());

        drop(func);

        builder.handler(control);
    }
}
