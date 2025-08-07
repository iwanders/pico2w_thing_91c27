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
    manufacturer: GattString<16>,

    /// Identify routine, triggers something, it does not contain data.
    #[characteristic(uuid=characteristic::IDENTIFY)]
    identify: bool,
}
