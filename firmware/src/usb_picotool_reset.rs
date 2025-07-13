//! RPi picotool reset class usb interface handler.

// https://github.com/raspberrypi/pico-sdk/blob/9a4113fbbae65ee82d8cd6537963bc3d3b14bcca/src/common/pico_usb_reset_interface_headers/include/pico/usb_reset_interface.h
// Do we need the whole Microsoft OS 2.0 descriptor stuff??
//
use core::marker::PhantomData;

use embassy_usb::control::{InResponse, Recipient, Request, RequestType};
use embassy_usb::driver::{Driver, Endpoint, EndpointError, EndpointIn, EndpointOut};
use embassy_usb::{Builder, Handler};

const CS_INTERFACE: u8 = 0x24;
const HEADER_SUBTYPE: u8 = 0x01;

pub const PICO_RESET_CLASS: u8 = 0xFF;
pub const PICO_RESET_SUBCLASS: u8 = 0x00;
pub const PICO_RESET_INTERFACE_PROTOCOL: u8 = 0x01;

use core::mem::MaybeUninit;

struct Control {}

impl Control {
    fn new() -> Self {
        Control {}
    }
}

impl Handler for Control {
    fn control_in(&mut self, req: Request, _data: &mut [u8]) -> Option<InResponse<'_>> {
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
        let midi_if = u8::from(audio_if) + 1;
        let mut alt = iface.alt_setting(
            PICO_RESET_CLASS,
            PICO_RESET_SUBCLASS,
            PICO_RESET_INTERFACE_PROTOCOL,
            None,
        );
        alt.descriptor(
            CS_INTERFACE,
            &[HEADER_SUBTYPE, 0x00, 0x01, 0x09, 0x00, 0x01, midi_if],
        );

        let control = state.control.write(Control::new());

        drop(func);

        builder.handler(control);
    }
}
