#![cfg_attr(not(test), no_std)]

#[cfg(test)]
extern crate std;

use trouble_host::prelude::*;
pub mod characteristic;
pub mod service;
pub mod util;
pub mod uuid;
use util::GattString;

// We probably should handle some gatt reads manually with:
// https://github.com/embassy-rs/trouble/pull/311
//

// This is also a bit of a problem;
// https://github.com/embassy-rs/trouble/issues/375
//

// Hmm, maybe this does what we need;
// https://github.com/sysgrok/rs-matter-embassy/blob/79a2a7786ad28e2ae186e4136e22c93a2c343599/rs-matter-embassy/src/ble.rs#L301
// it creates a service with 'External' types.
// It puts context and resources in a a struct; https://github.com/sysgrok/rs-matter-embassy/blob/ca6cef42001fb208875504eac7ab3cb9f22b7149/rs-matter-embassy/src/ble.rs#L148-L158
// That struct then has a handle_indications and handle_events method, that actually services the endpoints.
// Maybe it is okay if we define the server in this module, the
// https://github.com/embassy-rs/trouble/issues/391 issue mentions re-using a specific server?

#[gatt_service(uuid = service::ACCESSORY_INFORMATION)]
pub struct AccessoryInformationService {
    /// Describes hardware revision string; "<major>.<minor>.<revision>"
    #[characteristic(uuid=characteristic::HARDWARE_REVISION)]
    hardware_revision: GattString<16>,

    /// Manufacturer serial number, length must be greater than one.
    #[characteristic(uuid=characteristic::SERIAL_NUMBER)]
    serial_number: GattString<64>,

    /// Service instance ID, must be a 16 bit unsigned integer.
    #[characteristic(uuid=characteristic::SERVICE_INSTANCE)]
    service_instance: u16,

    /// Manufacturer specific model, length must be greater than one.
    #[characteristic(uuid=characteristic::SERIAL_NUMBER)]
    model: GattString<64>,

    /// Name for the device.
    #[characteristic(uuid=characteristic::NAME)]
    name: GattString<64>,

    /// Manufacturer name that created the device.
    #[characteristic(uuid=characteristic::MANUFACTURER)]
    manufacturer: GattString<64>,

    /// Firmware revision string; "<major>.<minor>.<revision>"
    #[characteristic(uuid=characteristic::FIRMWARE_REVISION)]
    firmware_revision: GattString<16>,

    /// Identify routine, triggers something, it does not contain data.
    #[characteristic(uuid=characteristic::IDENTIFY)]
    identify: bool,
}

macro_rules! as_gatt {
    ($l:ty) => {
        impl AsGatt for $l {
            const MIN_SIZE: usize = 0;

            const MAX_SIZE: usize = 512;

            fn as_gatt(&self) -> &[u8] {
                &[]
            }
        }
    };
}

/// Service properties struct.
#[derive(Default, Copy, Clone, Debug)]
pub struct ServiceProperties {
    pub primary: bool,
    pub hidden: bool,
    pub supports_configuration: bool,
}
as_gatt!(ServiceProperties);

// This is... very clunky :/
impl ServiceProperties {
    pub fn as_u16(&self) -> u16 {
        (if self.primary { 0x01 } else { 0x00 })
            | (if self.hidden { 0x02 } else { 0x00 })
            | (if self.supports_configuration {
                0x04
            } else {
                0x00
            })
    }
}
type FacadeDummyType = [u8; 0];

#[gatt_service(uuid = service::PROTOCOL_INFORMATION)]
pub struct ProtocolInformationService {
    /// Service instance ID, must be a 16 bit unsigned integer.
    #[characteristic(uuid=characteristic::SERVICE_INSTANCE)]
    service_instance: u16,

    /// Service signature, only two bytes.
    #[characteristic(uuid=characteristic::SERVICE_SIGNATURE, read)]
    service_signature: FacadeDummyType,

    /// Version string.
    #[characteristic(uuid=characteristic::VERSION, value="2.2.0".into())]
    version: GattString<16>,
}

pub struct HapPeripheralContext {
    protocol_service_properties: ServiceProperties,
}

impl HapPeripheralContext {
    pub fn new() -> Self {
        Self {
            protocol_service_properties: Default::default(),
        }
    }
    pub async fn process_gatt_event<'stack, 'server, P: PacketPool>(
        &mut self,
        protocol_service: &ProtocolInformationService,
        event: trouble_host::gatt::GattEvent<'stack, 'server, P>,
    ) -> Result<Option<trouble_host::gatt::GattEvent<'stack, 'server, P>>, trouble_host::Error>
    {
        return Ok(None);
        match event {
            GattEvent::Read(event) => {
                if event.handle() == protocol_service.service_signature.handle {
                    //warn!("Sending a reply for the battery handle.!");
                    //let peek = event.payload();
                    //self.service_signature.
                    //let data = &self.service_signature;
                    //let data = self.protocol_service_properties.as_u16().to_le_bytes();
                    let data = [55];
                    let rsp = trouble_host::att::AttRsp::Read { data: &data };
                    event.into_payload().reply(rsp).await?;

                    return Ok(None);
                } else {
                    // its not for us, wrap it back up
                    Ok(Some(GattEvent::Read(event)))
                }
            }
            remainder => Ok(Some(remainder)),
        }
    }
}

/*

impl ProtocolInformationServiceFacade {
    pub async fn handle<'stack, 'server, P: PacketPool>(
        &self,
        srv: &mut ProtocolInformationService,
        event: trouble_host::gatt::GattEvent<'stack, 'server, P>,
    ) -> Result<Option<trouble_host::gatt::GattEvent<'stack, 'server, P>>, trouble_host::Error>
    {
        match event {
            GattEvent::Read(event) => {
                if event.handle() == self.service_signature.handle {
                    //warn!("Sending a reply for the battery handle.!");
                    let peek = event.payload();
                    //self.service_signature.
                    //let data = &self.service_signature;
                    let data = srv.service_signature.as_u16().to_le_bytes();
                    let rsp = trouble_host::att::AttRsp::Read { data: &data };
                    event.into_payload().reply(rsp).await?;

                    return Ok(None);
                } else {
                    // its not for us, wrap it back up
                    Ok(Some(GattEvent::Read(event)))
                }
            }
            remainder => Ok(Some(remainder)),
        }
    }
}
*/
