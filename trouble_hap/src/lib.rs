#![cfg_attr(not(test), no_std)]

#[cfg(test)]
extern crate std;

use trouble_host::prelude::*;
pub mod characteristic;
pub mod service;
pub mod util;
pub mod uuid;
use util::GattString;

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

/// Service properties struct.
pub struct ServiceProperties {
    pub primary: bool,
    pub hidden: bool,
    pub supports_configuration: bool,
}
type ServicePropertiesType = [u8; 2];

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
    pub fn into_gatt(self) -> ServicePropertiesType {
        self.as_u16().to_le_bytes()
    }
}

#[gatt_service(uuid = service::PROTOCOL_INFORMATION)]
pub struct ProtocolInformationService {
    /// Service instance ID, must be a 16 bit unsigned integer.
    #[characteristic(uuid=characteristic::SERVICE_INSTANCE)]
    service_instance: u16,

    /// Service signature, only two bytes.
    #[characteristic(uuid=characteristic::SERVICE_SIGNATURE)]
    service_signature: ServicePropertiesType,

    /// Version string.
    #[characteristic(uuid=characteristic::VERSION, value="2.2.0".into())]
    version: GattString<16>,
}
