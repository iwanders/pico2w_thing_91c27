use crate::uuid::HomekitUuid16;
use trouble_host::attribute::Uuid;

//E604E95D-A759-4817-87D3-AA005083A0D1
const SERVICE_INSTANCE_BYTES: [u8; 16] = [
    0xE6, 0x04, 0xE9, 0x5D, 0xA7, 0x59, 0x48, 0x17, 0x87, 0xD3, 0xAA, 0x00, 0x50, 0x83, 0xA0, 0xD1,
];
/// Service Instance ID
pub const SERVICE_INSTANCE: Uuid = Uuid::new_long(SERVICE_INSTANCE_BYTES);

/// Hardware revision (semver).
pub const HARDWARE_REVISION: HomekitUuid16 = HomekitUuid16::new(0x0053);

/// Manufacturer specific serial number, length must be greater than 1.
pub const SERIAL_NUMBER: HomekitUuid16 = HomekitUuid16::new(0x0030);

/// Manufacturer specific model, length must be greater than 1.
pub const MODEL: HomekitUuid16 = HomekitUuid16::new(0x0021);

/// Describes a name, length greater than 1?
pub const NAME: HomekitUuid16 = HomekitUuid16::new(0x0023);

/// Manufacturer name, length greater than 1?
pub const MANUFACTURER: HomekitUuid16 = HomekitUuid16::new(0x0020);

/// Firmware revision (semver).
pub const FIRMWARE_REVISION: HomekitUuid16 = HomekitUuid16::new(0x0052);

/// Identify trigger.
pub const IDENTIFY: HomekitUuid16 = HomekitUuid16::new(0x0014);
