#![cfg_attr(not(test), no_std)]
use heapless::String;

#[cfg(test)]
extern crate std;

use trouble_host::prelude::*;
pub mod uuid;

/// Accessory Information
#[gatt_service(uuid = uuid::HomekitUuid16::new(0x003e))]
//#[gatt_service(uuid = BluetoothUuid16::new(0xFED3))]
pub struct AccessoryInformationService {
    // 00000053-0000-1000-8000-0026BB765291
    // 00000030-0000-1000-8000-0026BB765291
    // E604E95D-A759-4817-87D3-AA005083A0D1
    // 00000021-0000-1000-8000-0026BB765291
    // 00000023-0000-1000-8000-0026BB765291
    // 34AB8811-AC7F-4340-BAC3-FD6A85F9943B
    // 00000020-0000-1000-8000-0026BB765291
    // 00000052-0000-1000-8000-0026BB765291
    // 00000014-0000-1000-8000-0026BB765291
}
