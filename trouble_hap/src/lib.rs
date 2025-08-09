#![cfg_attr(not(test), no_std)]

// This mod MUST go first, so that the others see its macros.
pub(crate) mod fmt;

#[cfg(test)]
extern crate std;

use trouble_host::prelude::*;
pub mod characteristic;
pub mod service;
pub mod util;
pub mod uuid;
use util::GattString;

pub mod adv;

use embassy_sync::blocking_mutex::raw::RawMutex;

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
    pub hardware_revision: GattString<16>,

    /// Manufacturer serial number, length must be greater than one.
    #[characteristic(uuid=characteristic::SERIAL_NUMBER)]
    pub serial_number: GattString<64>,

    /// Service instance ID, must be a 16 bit unsigned integer.
    #[characteristic(uuid=characteristic::SERVICE_INSTANCE)]
    pub service_instance: u16,

    /// Manufacturer specific model, length must be greater than one.
    #[characteristic(uuid=characteristic::SERIAL_NUMBER)]
    pub model: GattString<64>,

    /// Name for the device.
    #[characteristic(uuid=characteristic::NAME)]
    pub name: GattString<64>,

    /// Manufacturer name that created the device.
    #[characteristic(uuid=characteristic::MANUFACTURER)]
    pub manufacturer: GattString<64>,

    /// Firmware revision string; "<major>.<minor>.<revision>"
    #[characteristic(uuid=characteristic::FIRMWARE_REVISION)]
    pub firmware_revision: GattString<16>,

    /// Identify routine, triggers something, it does not contain data.
    #[characteristic(uuid=characteristic::IDENTIFY)]
    pub identify: bool,
}

/// Helper to set all accessory information from static values in bulk.
#[derive(Copy, Clone, Debug)]
pub struct AccessoryInformationStatic {
    pub hardware_revision: &'static str,
    pub serial_number: &'static str,
    //pub service_instance: u16,
    pub model: &'static str,
    pub name: &'static str,
    pub manufacturer: &'static str,
    pub firmware_revision: &'static str,
}
impl Default for AccessoryInformationStatic {
    fn default() -> Self {
        Self {
            hardware_revision: "0.0.1",
            serial_number: "1234567890ABC",
            //service_instance: 0,
            model: "AmazingDevice",
            name: "Trouble_HAP",
            manufacturer: "TestManufacturer",
            firmware_revision: "0.0.1",
        }
    }
}

impl AccessoryInformationService {
    pub fn set_information_static<
        M: RawMutex,
        P: PacketPool,
        const AT: usize,
        const CT: usize,
        const CN: usize,
    >(
        &self,
        server: &AttributeServer<'_, M, P, AT, CT, CN>,
        value: &AccessoryInformationStatic,
    ) -> Result<(), Error> {
        self.hardware_revision
            .set(server, &util::GattString::from(value.hardware_revision))?;
        self.serial_number
            .set(server, &util::GattString::from(value.serial_number))?;
        self.model
            .set(server, &util::GattString::from(value.model))?;
        self.name.set(server, &util::GattString::from(value.name))?;
        self.manufacturer
            .set(server, &util::GattString::from(value.manufacturer))?;
        self.firmware_revision
            .set(server, &util::GattString::from(value.firmware_revision))?;
        Ok(())
    }
}

/// Service properties struct.
#[derive(Copy, Clone, Debug)]
pub struct ServiceProperties {
    pub primary: bool,
    pub hidden: bool,
    pub supports_configuration: bool,
}
impl Default for ServiceProperties {
    fn default() -> Self {
        Self {
            primary: true,
            hidden: false,
            supports_configuration: false,
        }
    }
}

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
        match event {
            GattEvent::Read(event) => {
                if event.handle() == protocol_service.service_signature.handle {
                    warn!("Got a read request on the service signature handle.");
                    //let peek = event.payload();
                    //self.service_signature.
                    //let data = &self.service_signature;
                    let data = self.protocol_service_properties.as_u16().to_le_bytes();
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
