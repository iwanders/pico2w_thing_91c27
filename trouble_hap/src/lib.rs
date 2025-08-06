#![cfg_attr(not(test), no_std)]
//use heapless::String;

#[cfg(test)]
extern crate std;

use trouble_host::prelude::*;
pub mod characteristic;
pub mod service;
pub mod uuid;

// Service 0000003E-0000-1000-8000-0026BB765291.
#[gatt_service(uuid = service::ACCESSORY_INFORMATION)]
//#[gatt_service(uuid = BluetoothUuid16::new(0xFED3))]
pub struct AccessoryInformationService {
    #[characteristic(uuid=characteristic::HARDWARE_REVISION)]
    // String, paired read.
    // What type do we put here to easily make this all work??
    hardware_revision: u8,
    // 00000053-0000-1000-8000-0026BB765291

    // 00000030-0000-1000-8000-0026BB765291
    // // What's this?
    // E604E95D-A759-4817-87D3-AA005083A0D1
    // 00000021-0000-1000-8000-0026BB765291
    // 00000023-0000-1000-8000-0026BB765291
    // // And this??
    // 34AB8811-AC7F-4340-BAC3-FD6A85F9943B
    // 00000020-0000-1000-8000-0026BB765291
    // 00000052-0000-1000-8000-0026BB765291
    // 00000014-0000-1000-8000-0026BB765291
}
