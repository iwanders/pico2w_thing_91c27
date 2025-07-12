//! RPi picotool reset class usb interface handler.

// https://github.com/raspberrypi/pico-sdk/blob/9a4113fbbae65ee82d8cd6537963bc3d3b14bcca/src/common/pico_usb_reset_interface_headers/include/pico/usb_reset_interface.h

use core::marker::PhantomData;

use embassy_usb::control::{InResponse, Recipient, Request, RequestType};
use embassy_usb::driver::{Driver, Endpoint, EndpointError, EndpointIn, EndpointOut};
use embassy_usb::{Builder, Handler};

const CS_INTERFACE: u8 = 0x24;
const CS_ENDPOINT: u8 = 0x25;
const HEADER_SUBTYPE: u8 = 0x01;

pub const PICO_RESET_CLASS: u8 = 0xFF;
pub const PICO_RESET_SUBCLASS: u8 = 0x00;
pub const PICO_RESET_INTERFACE_PROTOCOL: u8 = 0x01;

use core::mem::MaybeUninit;

pub struct PicoResetClass {}
impl Handler for PicoResetClass {
    fn control_in(&mut self, req: Request, _data: &mut [u8]) -> Option<InResponse<'_>> {
        None
    }
}

impl PicoResetClass {
    pub fn new<'d, D: Driver<'d>>(builder: &mut Builder<'d, D>, max_packet_size: u16) -> Self {
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
        //let mut control = Control {
        //    _foo: Default::default(),
        //};
        //builder.handler(&mut control);

        /*
        let mut retthing = PicoResetClass {};

        drop(func);

        builder.handler(&mut retthing);
        retthing */
    }
}
