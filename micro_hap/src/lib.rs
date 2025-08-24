#![cfg_attr(not(test), no_std)]

// This mod MUST go first, so that the others see its macros.
// pub(crate) mod fmt;
pub mod fmt;

#[cfg(test)]
extern crate std;

pub mod adv;
pub mod characteristic;
pub mod descriptor;
pub mod service;
pub mod uuid;

pub mod ble;

use bitfield_struct::bitfield;

pub mod pairing;
pub mod tlv;

pub mod srp;

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

// Descriptor!? dc46f0fe-81d2-4616-b5d9-6abdd796939a
// Ooh this characteristic instance id descriptor must be EACH characteristic.
// (uuid = \"1234\", value = 42, read, write, notify, indicate)
//
// server.table().find_characteristic_by_value_handle(handle);
// exists, but there doesn't appear to be a way to set the descriptor values besides in the host-macro

// Todo:
// - Figure out how to dynamically assignthe descriptors?
// - Each characteristic is 7.3.5.1; HAP Characteristic Signature Read Procedure
//   - Presentation format is _also_ required, see 7.4.5

// How are we going to test this?
// Home assistant
// https://github.com/home-assistant/core/blob/b481aaba772960810fc6b2c5bb1d331729d91660/requirements_all.txt#L19
// uses
//  https://github.com/Jc2k/aiohomekit/  which does both bluetooth and wifi
//  It links to https://github.com/jlusiardi/homekit_python/
//  that may have support for peripheral and controller??
//  and doesn't build, swap ed25519; https://github.com/warner/python-ed25519/issues/20
//
// The majority of logic will actually not be on the BLE interface.
// Could even test against https://github.com/ewilken/hap-rs
// if we also had tcp.
//
// Http transport seems to just use the same underlying bytes as the ble side?
//
//
// Accessory may only expose a single primary interface. Linked services display as a group in the ui.
// Primary service is optional,

use zerocopy::{FromBytes, Immutable, IntoBytes, KnownLayout, TryFromBytes};

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

#[derive(PartialEq, Eq, FromBytes, IntoBytes, Immutable, KnownLayout, Debug, Copy, Clone)]
#[repr(transparent)]
pub struct CharId(pub u16);

#[derive(PartialEq, Eq, FromBytes, IntoBytes, Immutable, KnownLayout, Debug, Copy, Clone)]
#[repr(transparent)]
pub struct SvcId(pub u16);

/// Properties for a service.
#[bitfield(u16)]
#[derive(PartialEq, Eq, TryFromBytes, IntoBytes, Immutable)]
pub struct ServiceProperties {
    #[bits(1)]
    primary: bool,
    #[bits(1)]
    hidden: bool,
    #[bits(1)]
    configurable: bool,

    #[bits(13)]
    __: u16,
}

#[derive(Clone, Debug)]
pub struct Service {
    pub uuid: uuid::Uuid,
    pub iid: SvcId,
    // 8 = accessory information service, its the one with the most attributes.
    pub attributes: heapless::Vec<Attribute, 12>,
    pub ble_handle: Option<u16>,

    pub properties: ServiceProperties,
}
impl Service {
    pub fn get_attribute_by_iid(&self, chr: CharId) -> Option<&Attribute> {
        for a in self.attributes.iter() {
            if a.iid == chr {
                return Some(a);
            }
        }
        None
    }
}

#[derive(Clone, Debug)]
pub struct BleProperties {
    pub handle: u16,
    pub format: ble::sig::CharacteristicRepresentation,
    pub properties: ble::CharacteristicProperties,
}
impl BleProperties {
    pub fn from_handle(handle: u16) -> Self {
        Self {
            handle,
            format: Default::default(),
            properties: ble::CharacteristicProperties::new(),
        }
    }
    pub fn with_format_opaque(self) -> Self {
        let mut format = self.format;
        format.format = ble::sig::Format::Opaque;
        Self { format, ..self }
    }
    pub fn with_properties(self, properties: ble::CharacteristicProperties) -> Self {
        let x = Self { properties, ..self };
        info!("x: {:#?}", x);
        x
    }
}

#[derive(Clone, Debug)]
pub struct Attribute {
    pub uuid: uuid::Uuid,
    pub iid: CharId,
    // permission?
    // something something descriptors.
    pub user_description: Option<heapless::String<32>>,
    // valid_range: Option<(u16, u16)>,
    // step_value: Option<u16>,
    pub ble: Option<BleProperties>,
}
impl Attribute {
    pub fn ble_ref(&self) -> &BleProperties {
        self.ble.as_ref().unwrap()
    }
    pub fn ble_mut(&mut self) -> &mut BleProperties {
        self.ble.as_mut().unwrap()
    }
}

//  HAPSessionChannelState
#[derive(Clone, Debug, Default)]
pub struct ControlChannel {
    pub key: [u8; pairing::CHACHA20_POLY1305_KEY_BYTES],
    pub nonce: u64,
}
impl ControlChannel {
    pub fn decrypt(&mut self, buffer: &mut [u8]) -> Result<(), chacha20poly1305::Error> {
        todo!();
        /*
        use chacha20poly1305::aead::generic_array::typenum::Unsigned;
        use chacha20poly1305::{
            AeadInPlace, ChaCha20Poly1305, Nonce,
            aead::{AeadCore, KeyInit},
        };
        type NonceSize = <ChaCha20Poly1305 as AeadCore>::NonceSize;
        let cipher = ChaCha20Poly1305::new_from_slice(&self.key).expect("key should work");
        // Create the nonce
        let mut nonce_bytes: [u8; NonceSize::USIZE] = Default::default();
        nonce_bytes[4..].copy_from_slice(&self.nonce.to_le_bytes());
        let nonce = Nonce::from_slice(&nonce_bytes);

        // Finally, create the plaintext.
        let associated_data = &[];
        cipher.decrypt_in_place(&nonce, associated_data, &mut buffer)*/
    }
}

// https://github.com/apple/HomeKitADK/blob/fb201f98f5fdc7fef6a455054f08b59cca5d1ec8/HAP/HAPSession.h#L73
#[derive(Clone, Debug, Default)]
pub struct Session {
    // The followinf four are in the hap substruct.
    /// Accessory to Controller control channel.
    pub a_to_c: ControlChannel,
    /// Controller to Accessory control channel.
    pub c_to_a: ControlChannel,
    /// Whether the session originated from a transient pair setup procedure.
    pub transient: bool,
    /// Whether the security session is active.
    pub security_active: bool,
    // ble specific data here?
}

// Where are sessions created, and how to we find the correct session, and how are they assigned to the connections
// or client?
// Session is opaque; https://github.com/apple/HomeKitADK/blob/fb201f98f5fdc7fef6a455054f08b59cca5d1ec8/HAP/HAP.h#L221-L225
// but asserted here:
// https://github.com/apple/HomeKitADK/blob/fb201f98f5fdc7fef6a455054f08b59cca5d1ec8/HAP/HAPSession.h#L169-L170
//
// Which also means that the BLE side only has one: https://github.com/apple/HomeKitADK/blob/fb201f98f5fdc7fef6a455054f08b59cca5d1ec8/Applications/Main.c#L258
// Can a bluetooth accessory only have one connection?
// Ah yes, it is a peripheral, peripherals in general get only one connection.

#[cfg(test)]
mod test {
    pub fn init() {
        let _ = env_logger::builder()
            .is_test(true)
            .filter_level(log::LevelFilter::max())
            .try_init();
    }
}
