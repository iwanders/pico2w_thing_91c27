use crate::AccessoryInformationStatic;
use crate::{characteristic, descriptor, service};
use trouble_host::prelude::*;
mod util;
use embassy_sync::blocking_mutex::raw::RawMutex;
use util::GattString;
mod pdu;
use crate::BleProperties;
use bitfield_struct::bitfield;

use crate::{CharId, SvcId};
use zerocopy::{FromBytes, Immutable, IntoBytes, KnownLayout, TryFromBytes};

// Todo, we should probably detach this completely from the HapServices struct
// because it would be really nice if we can keep properties per service, characteristic and property.
//

// add a lightbulb service such that we have at least one service.
// accessory information service must have instance id 1.
// https://github.com/apple/HomeKitADK/blob/fb201f98f5fdc7fef6a455054f08b59cca5d1ec8/HAP/HAP.h#L3245-L3249
// Okay, the disconnect doesn't happen because of fragmentation now, the response on the first characteristic read
// is actually in a single packet.

// Maybe the instance ids and the like need to be monotonically increasing? Which is not explicitly stated.
// changing ids definitely fixed things.
//
// We're now on the pair verify characteristic response not being accepted.
//
// Pair verify is ONLY 'read' not open_read... so we probably need to implement a security reject, after which a pairing
// is triggered?
//
//
// For the permissions;
// https://github.com/apple/HomeKitADK/blob/fb201f98f5fdc7fef6a455054f08b59cca5d1ec8/Applications/Lightbulb/DB.c#L48
// seems to have the best overview?
//

// We currently get an InfoRequest, followed the reference implementation, but the phone still disconnects.
// The phone never sends this request though, so we must be doing something different.

pub mod sig;

#[derive(PartialEq, Eq, FromBytes, IntoBytes, Immutable, KnownLayout, Debug, Copy, Clone)]
#[repr(transparent)]
pub struct TId(pub u8);

#[derive(Debug, Copy, Clone)]
pub enum HapBleError {
    UnexpectedDataLength {
        expected: usize,
        actual: usize,
    },
    UnexpectedRequest,
    InvalidValue,
    /// Runtime buffer overrun.
    BufferOverrun,
    /// Overrun on allocation space
    AllocationOverrun,
    /// Something went wrong with decryption or encryption.
    EncryptionError,
}

impl From<crate::tlv::TLVError> for HapBleError {
    fn from(e: crate::tlv::TLVError) -> HapBleError {
        match e {
            crate::tlv::TLVError::NotEnoughData => HapBleError::InvalidValue,
            crate::tlv::TLVError::MissingEntry(_) => HapBleError::InvalidValue,
            crate::tlv::TLVError::UnexpectedValue => HapBleError::InvalidValue,
            crate::tlv::TLVError::BufferOverrun => HapBleError::BufferOverrun,
        }
    }
}

impl From<HapBleError> for trouble_host::Error {
    fn from(e: HapBleError) -> trouble_host::Error {
        match e {
            HapBleError::UnexpectedDataLength { expected, actual } => {
                trouble_host::Error::UnexpectedDataLength { expected, actual }
            }
            HapBleError::UnexpectedRequest => trouble_host::Error::Other,
            HapBleError::InvalidValue => trouble_host::Error::InvalidValue,
            HapBleError::BufferOverrun => trouble_host::Error::OutOfMemory,
            HapBleError::EncryptionError => trouble_host::Error::NoPermits,
            HapBleError::AllocationOverrun => trouble_host::Error::OutOfMemory,
        }
    }
}
impl From<chacha20poly1305::Error> for HapBleError {
    fn from(_: chacha20poly1305::Error) -> HapBleError {
        HapBleError::EncryptionError
    }
}

// MUST have an instance id of 1, service 3e
#[gatt_service(uuid = service::ACCESSORY_INFORMATION)]
pub struct AccessoryInformationService {
    /// Service instance ID, must be a 16 bit unsigned integer.
    // Service instance id for accessory information must be 1, 0 is invalid.
    // https://github.com/apple/HomeKitADK/blob/fb201f98f5fdc7fef6a455054f08b59cca5d1ec8/HAP/HAP.h#L3245-L3249
    //#[descriptor(uuid=descriptor::CHARACTERISTIC_INSTANCE_UUID, read, value=1u16.to_le_bytes())]
    #[characteristic(uuid=characteristic::SERVICE_INSTANCE, read, value = 1)]
    pub service_instance: u16,

    // 0x14
    /// Identify routine, triggers something, it does not contain data.
    #[characteristic(uuid=characteristic::IDENTIFY, read, write)]
    #[descriptor(uuid=descriptor::CHARACTERISTIC_INSTANCE_UUID, read, value=2u16.to_le_bytes())]
    pub identify: bool,

    // 0x20
    /// Manufacturer name that created the device.
    #[characteristic(uuid=characteristic::MANUFACTURER, read, write)]
    #[descriptor(uuid=descriptor::CHARACTERISTIC_INSTANCE_UUID, read, value=3u16.to_le_bytes())]
    pub manufacturer: GattString<64>,

    // 0x21
    /// Manufacturer specific model, length must be greater than one.
    #[descriptor(uuid=descriptor::CHARACTERISTIC_INSTANCE_UUID, read, value=4u16.to_le_bytes())]
    #[characteristic(uuid=characteristic::MODEL, read, write)]
    pub model: GattString<64>,

    // 0x0023
    /// Name for the device.
    #[descriptor(uuid=descriptor::CHARACTERISTIC_INSTANCE_UUID, read, value=5u16.to_le_bytes())]
    #[characteristic(uuid=characteristic::NAME, read, write)]
    pub name: GattString<64>,

    //0x0030
    /// Manufacturer serial number, length must be greater than one.
    #[descriptor(uuid=descriptor::CHARACTERISTIC_INSTANCE_UUID, read, value=6u16.to_le_bytes())]
    #[characteristic(uuid=characteristic::SERIAL_NUMBER, read, write)]
    pub serial_number: GattString<64>,

    //0x0052
    /// Firmware revision string; "<major>.<minor>.<revision>"
    #[descriptor(uuid=descriptor::CHARACTERISTIC_INSTANCE_UUID, read, value=7u16.to_le_bytes())]
    #[characteristic(uuid=characteristic::FIRMWARE_REVISION, read, write)]
    pub firmware_revision: GattString<16>,

    //0x0053
    /// Describes hardware revision string; "<major>.<minor>.<revision>"
    #[characteristic(uuid=characteristic::HARDWARE_REVISION, read, write)]
    #[descriptor(uuid=descriptor::CHARACTERISTIC_INSTANCE_UUID, read, value=8u16.to_le_bytes())]
    pub hardware_revision: GattString<16>,

    // 4ab8811-ac7f-4340-bac3-fd6a85f9943b
    /// ADK version thing from the example,
    #[characteristic(uuid=characteristic::ADK_VERSION, read, write)]
    #[descriptor(uuid=descriptor::CHARACTERISTIC_INSTANCE_UUID, read, value=9u16.to_le_bytes())]
    pub adk_version: GattString<16>,
}

impl AccessoryInformationService {
    fn populate_support(&self) -> Result<crate::Service, HapBleError> {
        let mut service = crate::Service {
            ble_handle: Some(self.handle),
            uuid: service::ACCESSORY_INFORMATION.into(),
            iid: SvcId(1),
            attributes: Default::default(),
            properties: Default::default(),
        };

        service
            .attributes
            .push(crate::Attribute {
                uuid: characteristic::SERVICE_INSTANCE.into(),
                iid: CharId(2),
                user_description: None,
                ble: Some(BleProperties::from_handle(self.service_instance.handle)),
            })
            .map_err(|_| HapBleError::AllocationOverrun)?;

        service
            .attributes
            .push(crate::Attribute {
                uuid: characteristic::IDENTIFY.into(),
                iid: CharId(2),
                user_description: None,
                ble: Some(BleProperties::from_handle(self.identify.handle)),
            })
            .map_err(|_| HapBleError::AllocationOverrun)?;

        service
            .attributes
            .push(crate::Attribute {
                uuid: characteristic::MANUFACTURER.into(),
                iid: CharId(3),
                user_description: None,
                ble: Some(BleProperties::from_handle(self.manufacturer.handle)),
            })
            .map_err(|_| HapBleError::AllocationOverrun)?;

        service
            .attributes
            .push(crate::Attribute {
                uuid: characteristic::MODEL.into(),
                iid: CharId(4),
                user_description: None,
                ble: Some(BleProperties::from_handle(self.model.handle)),
            })
            .map_err(|_| HapBleError::AllocationOverrun)?;

        service
            .attributes
            .push(crate::Attribute {
                uuid: characteristic::NAME.into(),
                iid: CharId(5),
                user_description: None,
                ble: Some(BleProperties::from_handle(self.name.handle)),
            })
            .map_err(|_| HapBleError::AllocationOverrun)?;

        service
            .attributes
            .push(crate::Attribute {
                uuid: characteristic::SERIAL_NUMBER.into(),
                iid: CharId(6),
                user_description: None,
                ble: Some(BleProperties::from_handle(self.serial_number.handle)),
            })
            .map_err(|_| HapBleError::AllocationOverrun)?;

        service
            .attributes
            .push(crate::Attribute {
                uuid: characteristic::FIRMWARE_REVISION.into(),
                iid: CharId(7),
                user_description: None,
                ble: Some(BleProperties::from_handle(self.firmware_revision.handle)),
            })
            .map_err(|_| HapBleError::AllocationOverrun)?;

        service
            .attributes
            .push(crate::Attribute {
                uuid: characteristic::HARDWARE_REVISION.into(),
                iid: CharId(8),
                user_description: None,
                ble: Some(BleProperties::from_handle(self.hardware_revision.handle)),
            })
            .map_err(|_| HapBleError::AllocationOverrun)?;

        service
            .attributes
            .push(crate::Attribute {
                uuid: characteristic::ADK_VERSION.into(),
                iid: CharId(9),
                user_description: None,
                ble: Some(BleProperties::from_handle(self.adk_version.handle)),
            })
            .map_err(|_| HapBleError::AllocationOverrun)?;

        Ok(service)
    }
}
/*
fn make_table() {
    // from https://github.com/embassy-rs/trouble/blame/404b0f77345522764582747e0acd50a22236b59e/examples/apps/src/ble_bas_peripheral.rs
    const MAX_ATTRIBUTES: usize = 10;
    use embassy_sync::blocking_mutex::raw::NoopRawMutex;
    let mut table: AttributeTable<'_, NoopRawMutex, { MAX_ATTRIBUTES }> = AttributeTable::new();
    let mut svc = table.add_service(Service {
        uuid: service::ACCESSORY_INFORMATION.into(),
    });
    let v = svc
        .add_characteristic::<FacadeDummyType, crate::uuid::HomekitUuid16>(
            characteristic::HARDWARE_REVISION,
            &[CharacteristicProp::Read, CharacteristicProp::Write],
            FacadeDummyType::default(),
            &mut [],
        )
        //.add_descriptor(uuid, props, data)
        .build();
    let handle = v.handle;

    // let server = GattServer::<C, NoopRawMutex, MAX_ATTRIBUTES, L2CAP_MTU>::new(stack, &table);
}*/

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
        self.hardware_revision.set(
            server,
            &util::GattString::try_from(value.hardware_revision)
                .map_err(|_| Error::InsufficientSpace)?,
        )?;
        self.serial_number.set(
            server,
            &util::GattString::try_from(value.serial_number)
                .map_err(|_| Error::InsufficientSpace)?,
        )?;
        self.model.set(
            server,
            &util::GattString::try_from(value.model).map_err(|_| Error::InsufficientSpace)?,
        )?;
        self.name.set(
            server,
            &util::GattString::try_from(value.name).map_err(|_| Error::InsufficientSpace)?,
        )?;
        self.manufacturer.set(
            server,
            &util::GattString::try_from(value.manufacturer)
                .map_err(|_| Error::InsufficientSpace)?,
        )?;
        self.firmware_revision.set(
            server,
            &util::GattString::try_from(value.firmware_revision)
                .map_err(|_| Error::InsufficientSpace)?,
        )?;
        Ok(())
    }
}

type FacadeDummyType = [u8; 0];

// 0xA2
#[gatt_service(uuid = service::PROTOCOL_INFORMATION)]
pub struct ProtocolInformationService {
    /// Service instance ID, must be a 16 bit unsigned integer.
    // May not be 1, value 1 is for accessory information.
    //#[descriptor(uuid=descriptor::CHARACTERISTIC_INSTANCE_UUID, read, value=0x02u16.to_le_bytes())]
    #[characteristic(uuid=characteristic::SERVICE_INSTANCE, read, value = 0x10)]
    service_instance: u16,

    /// Service signature, only two bytes.
    #[characteristic(uuid=characteristic::SERVICE_SIGNATURE, read, write)]
    #[descriptor(uuid=descriptor::CHARACTERISTIC_INSTANCE_UUID, read,  value=0x11u16.to_le_bytes())]
    service_signature: FacadeDummyType,

    /// Version string.
    #[characteristic(uuid=characteristic::VERSION, value="2.2.0".try_into().unwrap(), read, write)]
    #[descriptor(uuid=descriptor::CHARACTERISTIC_INSTANCE_UUID, read, value=0x12u16.to_le_bytes())]
    version: GattString<16>,
}
impl ProtocolInformationService {
    fn populate_support(&self) -> Result<crate::Service, HapBleError> {
        let mut service = crate::Service {
            ble_handle: Some(self.handle),
            uuid: service::PROTOCOL_INFORMATION.into(),
            iid: SvcId(0x10),
            attributes: Default::default(),
            properties: crate::ServiceProperties::new().with_configurable(true),
        };

        /*
        service
            .attributes
            .push(crate::Attribute {
                uuid: characteristic::SERVICE_INSTANCE.into(),
                iid: CharId(0x14),
                user_description: None,
                ble: Some(
                    BleProperties::from_handle(self.service_instance.handle).with_format_opaque(),
                ),
            })
            .map_err(|_| HapBleError::AllocationOverrun)?;
            */

        service
            .attributes
            .push(crate::Attribute {
                uuid: characteristic::SERVICE_SIGNATURE.into(),
                iid: CharId(0x11),
                user_description: None,
                ble: Some(
                    BleProperties::from_handle(self.service_signature.handle)
                        .with_format_opaque()
                        .with_properties(CharacteristicProperties::new().with_read(true)),
                ),
            })
            .map_err(|_| HapBleError::AllocationOverrun)?;

        service
            .attributes
            .push(crate::Attribute {
                uuid: characteristic::VERSION.into(),
                iid: CharId(0x12),
                user_description: None,
                ble: Some(
                    BleProperties::from_handle(self.version.handle)
                        .with_format_opaque()
                        .with_properties(CharacteristicProperties::new().with_read(true)),
                ),
            })
            .map_err(|_| HapBleError::AllocationOverrun)?;

        Ok(service)
    }
}

#[gatt_service(uuid = service::PAIRING)]
pub struct PairingService {
    /// Service instance ID, must be a 16 bit unsigned integer.
    // May not be 1, value 1 is for accessory information.
    //#[descriptor(uuid=descriptor::CHARACTERISTIC_INSTANCE_UUID, read, value=[0x03, 0x01])]
    #[characteristic(uuid=characteristic::SERVICE_INSTANCE, read, value = 0x20)]
    service_instance: u16,

    #[descriptor(uuid=descriptor::CHARACTERISTIC_INSTANCE_UUID, read, value=0x22u16.to_le_bytes())]
    #[characteristic(uuid=characteristic::PAIRING_PAIR_SETUP, read, write)]
    pair_setup: FacadeDummyType,

    #[descriptor(uuid=descriptor::CHARACTERISTIC_INSTANCE_UUID, read, value=0x23u16.to_le_bytes())]
    #[characteristic(uuid=characteristic::PAIRING_PAIR_VERIFY, read, write)]
    pair_verify: FacadeDummyType,

    #[descriptor(uuid=descriptor::CHARACTERISTIC_INSTANCE_UUID, read, value=0x24u16.to_le_bytes())]
    #[characteristic(uuid=characteristic::PAIRING_FEATURES, read, write)]
    features: FacadeDummyType,

    // Paired read and write only.
    #[descriptor(uuid=descriptor::CHARACTERISTIC_INSTANCE_UUID, read, value=0x25u16.to_le_bytes())]
    #[characteristic(uuid=characteristic::PAIRING_PAIRINGS, read, write)]
    pairings: FacadeDummyType,
}

impl PairingService {
    fn populate_support(&self) -> Result<crate::Service, HapBleError> {
        let mut service = crate::Service {
            ble_handle: Some(self.handle),
            uuid: service::PAIRING.into(),
            iid: SvcId(0x20),
            attributes: Default::default(),
            properties: Default::default(),
        };

        /*
        service
            .attributes
            .push(crate::Attribute {
                uuid: characteristic::SERVICE_INSTANCE.into(),
                iid: CharId(0x20),
                user_description: None,
                ble: Some(
                    BleProperties::from_handle(self.service_instance.handle).with_format_opaque(),
                ),
            })
            .map_err(|_| HapBleError::AllocationOverrun)?;
            */

        service
            .attributes
            .push(crate::Attribute {
                uuid: characteristic::PAIRING_PAIR_SETUP.into(),
                iid: CharId(0x22),
                user_description: None,
                ble: Some(
                    BleProperties::from_handle(self.pair_setup.handle)
                        .with_format_opaque()
                        .with_properties(CharacteristicProperties::new().with_open_rw(true)),
                ),
            })
            .map_err(|_| HapBleError::AllocationOverrun)?;
        service
            .attributes
            .push(crate::Attribute {
                uuid: characteristic::PAIRING_PAIR_VERIFY.into(),
                iid: CharId(0x23),
                user_description: None,
                ble: Some(
                    BleProperties::from_handle(self.pair_verify.handle)
                        .with_format_opaque()
                        .with_properties(CharacteristicProperties::new().with_open_rw(true)),
                ),
            })
            .map_err(|_| HapBleError::AllocationOverrun)?;

        service
            .attributes
            .push(crate::Attribute {
                uuid: characteristic::PAIRING_FEATURES.into(),
                iid: CharId(0x24),
                user_description: None,
                ble: Some(
                    BleProperties::from_handle(self.features.handle)
                        .with_format(crate::ble::sig::Format::U8)
                        .with_properties(CharacteristicProperties::new().with_read_open(true)),
                ),
            })
            .map_err(|_| HapBleError::AllocationOverrun)?;
        service
            .attributes
            .push(crate::Attribute {
                uuid: characteristic::PAIRING_PAIRINGS.into(),
                iid: CharId(0x25),
                user_description: None,
                ble: Some(
                    BleProperties::from_handle(self.pairings.handle)
                        .with_format_opaque()
                        .with_properties(CharacteristicProperties::new().with_rw(true)),
                ),
            })
            .map_err(|_| HapBleError::AllocationOverrun)?;
        Ok(service)
    }
}

#[gatt_service(uuid = service::LIGHTBULB)]
pub struct LightbulbService {
    //#[descriptor(uuid=descriptor::CHARACTERISTIC_INSTANCE_UUID, read, value=[0x04, 0x01])]
    #[characteristic(uuid=characteristic::SERVICE_INSTANCE, read, value = 0x30)]
    service_instance: u16,

    /// Service signature, only two bytes.
    #[characteristic(uuid=characteristic::SERVICE_SIGNATURE, read, write)]
    #[descriptor(uuid=descriptor::CHARACTERISTIC_INSTANCE_UUID, read,  value=0x31u16.to_le_bytes())]
    service_signature: FacadeDummyType,

    // 0x0023
    /// Name for the device.
    #[descriptor(uuid=descriptor::CHARACTERISTIC_INSTANCE_UUID, read, value=0x32u16.to_le_bytes())]
    #[characteristic(uuid=characteristic::NAME, read, write )]
    pub name: GattString<64>,

    #[descriptor(uuid=descriptor::CHARACTERISTIC_INSTANCE_UUID, read, value=0x33u16.to_le_bytes())]
    #[characteristic(uuid=characteristic::ON, read, write )]
    on: bool,
}

// https://github.com/apple/HomeKitADK/blob/fb201f98f5fdc7fef6a455054f08b59cca5d1ec8/HAP/HAPBLEPDU%2BTLV.c#L93
/// Properties for a characteristic
#[bitfield(u16)]
#[derive(PartialEq, Eq, FromBytes, IntoBytes, Immutable)]
pub struct CharacteristicProperties {
    #[bits(1)]
    pub read_open: bool, // readableWithoutSecurity
    #[bits(1)]
    pub write_open: bool,
    #[bits(1)]
    pub supports_authorization: bool,
    #[bits(1)]
    pub requires_timed_write: bool,

    #[bits(1)]
    pub read: bool,

    #[bits(1)]
    pub write: bool,

    #[bits(1)]
    pub hidden: bool,

    #[bits(1)]
    pub supports_event_notification: bool,

    #[bits(1)]
    pub supports_disconnect_notification: bool,

    #[bits(1)]
    pub supports_broadcast_notification: bool,

    #[bits(6)]
    __: u16,
}
impl CharacteristicProperties {
    pub fn with_open_rw(self, state: bool) -> Self {
        self.with_read_open(state).with_write_open(state)
    }
    pub fn with_rw(self, state: bool) -> Self {
        self.with_read(state).with_write(state)
    }
}

/// Simple helper struct that's used to capture input to the gatt event handler.
pub struct HapServices<'a> {
    pub information: &'a AccessoryInformationService,
    pub protocol: &'a ProtocolInformationService,
    pub pairing: &'a PairingService,
}

use pdu::{BleTLVType, BodyBuilder, ParsePdu, WriteIntoLength};

struct Reply {
    payload: BufferResponse,
    handle: u16,
}
#[derive(PartialEq, Debug, Copy, Clone)]
pub struct BufferResponse(pub usize);

pub struct HapPeripheralContext {
    //protocol_service_properties: ServiceProperties,
    buffer: core::cell::RefCell<&'static mut [u8]>,
    pair_ctx: core::cell::RefCell<&'static mut crate::pairing::PairContext>,

    prepared_reply: Option<Reply>,
    should_encrypt_reply: bool,

    information_service: crate::Service,
    protocol_service: crate::Service,
    pairing_service: crate::Service,
    // pair_ctx: crate::pairing::PairContext<'static, 'static, 'static, 'static>,
}
impl HapPeripheralContext {
    fn services(&self) -> [&crate::Service; 3] {
        [
            &self.information_service,
            &self.protocol_service,
            &self.pairing_service,
        ]
    }

    pub fn get_attribute_by_char(&self, chr: CharId) -> Option<&crate::Attribute> {
        for s in self.services() {
            if let Some(a) = s.get_attribute_by_iid(chr) {
                return Some(a);
            }
        }
        None
    }

    pub fn get_service_by_char(&self, chr: CharId) -> Option<&crate::Service> {
        for s in self.services() {
            if let Some(_attribute) = s.get_attribute_by_iid(chr) {
                return Some(s);
            }
        }
        None
    }

    pub fn get_service_by_svc(&self, srv: SvcId) -> Option<&crate::Service> {
        for s in self.services() {
            if s.iid == srv {
                return Some(s);
            }
        }
        None
    }

    pub fn new(
        buffer: &'static mut [u8],
        pair_ctx: &'static mut crate::pairing::PairContext,
        information_service: &AccessoryInformationService,
        protocol_service: &ProtocolInformationService,
        pairing_service: &PairingService,
    ) -> Result<Self, HapBleError> {
        Ok(Self {
            //protocol_service_properties: Default::default(),
            buffer: buffer.into(),
            pair_ctx: pair_ctx.into(),
            prepared_reply: None,
            should_encrypt_reply: false,
            information_service: information_service.populate_support()?,
            protocol_service: protocol_service.populate_support()?,
            pairing_service: pairing_service.populate_support()?,
        })
    }

    pub async fn service_signature_request(
        &mut self,
        req: &pdu::ServiceSignatureReadRequest,
    ) -> Result<BufferResponse, HapBleError> {
        // https://github.com/apple/HomeKitADK/blob/fb201f98f5fdc7fef6a455054f08b59cca5d1ec8/HAP/HAPBLEProcedure.c#L249

        info!("service signature req: {:?}", req);
        let resp = req.header.to_success();

        let mut buffer = self.buffer.borrow_mut();

        let len = resp.write_into_length(*buffer)?;

        let svc = self.get_service_by_svc(req.svc_id);

        if let Some(svc) = svc {
            // WHat is the actual output?
            // Something something TLV...
            //
            // https://github.com/apple/HomeKitADK/blob/fb201f98f5fdc7fef6a455054f08b59cca5d1ec8/HAP/HAPBLEService%2BSignature.c#L10
            // which then goes into
            // https://github.com/apple/HomeKitADK/blob/fb201f98f5fdc7fef6a455054f08b59cca5d1ec8/HAP/HAPBLEPDU%2BTLV.c#L600
            // and linked services in
            // https://github.com/apple/HomeKitADK/blob/fb201f98f5fdc7fef6a455054f08b59cca5d1ec8/HAP/HAPBLEPDU%2BTLV.c#L640
            // NONCOMPLIANCE: Ignoring linked services.
            // I have no idea what a linked service is yet... so lets ehm, just ignore that?
            // read was:
            // 00 06 93 10 00
            // Reply was
            //    02 93 00 06 00 0f 02 04 00 10 00
            // So Payload is 6 body length, then 0f 02 04 and no linked svc.
            // What does this service property do!?
            //   [2, 81, 0, 6, 0, f, 2, 4, 0, 10, 0]
            //
            // protocol.service signature;
            // good: 02  5b  00  06  00  0f  02  04  00  10  00
            // ours: 02, cd, 00, 06, 00, 0f, 02, 00, 00, 10, 00
            // now   02, 43, 00, 06, 00, 0f, 02, 00, 00, 10, 00
            //       02, 69, 00, 06, 00, 0f, 02, 00, 00, 10, 00
            // now   02, 4d, 00, 06, 00, 0f, 02, 04, 00, 10, 00
            // check!
            let len = BodyBuilder::new_at(*buffer, len)
                .add_u16(BleTLVType::HAPServiceProperties, svc.properties.0)
                .add_u16s(BleTLVType::HAPLinkedServices, &[])
                .end();

            Ok(BufferResponse(len))
        } else {
            // What do we return if the id is not known??
            todo!()
        }
    }
    pub async fn characteristic_signature_request(
        &mut self,
        req: &pdu::CharacteristicSignatureReadRequest,
    ) -> Result<BufferResponse, HapBleError> {
        // https://github.com/apple/HomeKitADK/blob/fb201f98f5fdc7fef6a455054f08b59cca5d1ec8/HAP/HAPBLEProcedure.c#L289

        // To hdump  00 01  c8  11  00
        // reply:       02  c8  00  35  00  04  10  91  52  76  bb  26  00  00  80  00  10  00  00  a5  00  00  00  07  02  10  00  06  10  91  52  76  bb  26  00  00  80  00  10  00  00  a2  00  00  00  0a  02  10  00  0c  07  1b  00  00  27  01  00  00
        //          00, 01, 05, 02, 02
        //              02, 05, 00, 35, 00, 04, 10, 91, 52, 76, bb, 26, 00, 00, 80, 00, 10, 00, 00, a5, 00, 00, 00, 07, 02, 02, 00, 06, 10, 91, 52, 76, bb, 26, 00, 00, 80, 00, 10, 00, 00, a2, 00, 00, 00, 0a, 02, 10, 00, 0c, 07, 1b, 00, 00, 27, 01, 00, 00
        //                  tid st <len  >
        //                                 <chr                                                                 > <svcid         ><svctype                                                               ><properties    >
        //
        // Bad:       00 01 d2 02 52
        //              02  d2  00  35  00  04  10  91  52  76  bb  26  00  00  80  00  10  00  00  a5  00  00  00  07  02  02  00  06  10  91  52  76  bb  26  00  00  80  00  10  00  00  a2  00  00  00  0a  02  10  00  0c  07  1b  00  00  27  01  00  00
        // from 2025_08_15_1817_homekitadk_pair_success;
        // Good:      00 01 b4 11 00
        //              02  b4  00  35  00  04  10  91  52  76  bb  26  00  00  80  00  10  00  00  a5  00  00  00  07  02  10  00  06  10  91  52  76  bb  26  00  00  80  00  10  00  00  a2  00  00  00  0a  02  10  00  0c  07  1b  00  00  27  01  00  00
        //                                                                                                                  ^^
        // good;
        // 07  02  10  00
        //         ^^^^^^  is the SVC id.
        //
        //
        //
        // more bad
        //              02, 21, 00, 35, 00, 04, 10, 91, 52, 76, bb, 26, 00, 00, 80, 00, 10, 00, 00, a5, 00, 00, 00, 07, 02, 02, 00, 06, 10, 91, 52, 76, bb, 26, 00, 00, 80, 00, 10, 00, 00, a2, 00, 00, 00, 0a, 02, 10, 00, 0c, 07, 1b, 00, 00, 27, 01, 00, 00
        //
        //
        // On pair-verify;
        // good: 02  6d  00  35  00  04  10  91  52  76  bb  26  00  00  80  00  10  00  00  4e  00  00  00  07  02  20  00  06  10  91  52  76  bb  26  00  00  80  00  10  00  00  55  00  00  00  0a  02  03  00  0c  07  1b  00  00  27  01  00  00
        // bad:  02, f8, 00, 35, 00, 04, 10, 91, 52, 76, bb, 26, 00, 00, 80, 00, 10, 00, 00, 4e, 00, 00, 00, 07, 02, 20, 00, 06, 10, 91, 52, 76, bb, 26, 00, 00, 80, 00, 10, 00, 00, 55, 00, 00, 00, 0a, 02, 10, 00, 0c, 07, 1b, 00, 00, 27, 01, 00, 00
        //       02, 84, 00, 35, 00, 04, 10, 91, 52, 76, bb, 26, 00, 00, 80, 00, 10, 00, 00, 4e, 00, 00, 00, 07, 02, 20, 00, 06, 10, 91, 52, 76, bb, 26, 00, 00, 80, 00, 10, 00, 00, 55, 00, 00, 00, 0a, 02, 03, 00, 0c, 07, 1b, 00, 00, 27, 01, 00, 00
        //      [02, 84, 00, 35, 00, 04, 10, 91, 52, 76, bb, 26, 00, 00, 80, 00, 10, 00, 00, 4e, 00, 00, 00, 07, 02, 20, 00, 06, 10, 91, 52, 76, bb, 26, 00, 00, 80, 00, 10, 00, 00, 55, 00, 00, 00, 0a, 02, 03, 00, 0c, 07, 1b, 00, 00, 27, 01, 00, 00]
        //       02, ed, 00, 35, 00, 04, 10, 91, 52, 76, bb, 26, 00, 00, 80, 00, 10, 00, 00, 4e, 00, 00, 00, 07, 02, 20, 00, 06, 10, 91, 52, 76, bb, 26, 00, 00, 80, 00, 10, 00, 00, 55, 00, 00, 00, 0a, 02, 03, 00, 0c, 07, 1b, 00, 00, 27, 01, 00, 00
        // bad:
        //  [2025-08-16T20:50:24Z WARN  micro_hap::ble] Writing pairing.pair_verify  [0, 1, 132, 35, 0] 0x:  00 01 84 23 00
        //  [2025-08-16T20:50:24Z WARN  micro_hap::ble] Replying with: Read { data: [02, 84, 00, 35, 00, 04, 10, 91, 52, 76, bb, 26, 00, 00, 80, 00, 10, 00, 00, 4e, 00, 00, 00, 07, 02, 20, 00, 06, 10, 91, 52, 76, bb, 26, 00, 00, 80, 00, 10, 00, 00, 55, 00, 00, 00, 0a, 02, 03, 00, 0c, 07, 1b, 00, 00, 27, 01, 00, 00] }
        // good:
        //
        //
        // Pair setup reply good is:
        //
        // 02  de  00  35  00  04  10  91  52  76  bb  26  00  00  80  00  10  00  00  4c  00  00  00  07  02  20  00  06  10  91  52  76  bb  26  00  00  80  00  10  00  00  55  00  00  00  0a  02  03  00  0c  07  1b  00  00  27  01  00  00
        // Our pair setup reply is:
        //[02, c6, 00, 35, 00, 04, 10, 91, 52, 76, bb, 26, 00, 00, 80, 00, 10, 00, 00, 4c, 00, 00, 00, 07, 02, 20, 00, 06, 10, 91, 52, 76, bb, 26, 00, 00, 80, 00, 10, 00, 00, 55, 00, 00, 00, 0a, 02, 00, 00, 0c, 07, 1b, 00, 00, 27, 01, 00, 00]
        //[02, 5e, 00, 35, 00, 04, 10, 91, 52, 76, bb, 26, 00, 00, 80, 00, 10, 00, 00, 4c, 00, 00, 00, 07, 02, 20, 00, 06, 10, 91, 52, 76, bb, 26, 00, 00, 80, 00, 10, 00, 00, 55, 00, 00, 00, 0a, 02, 03, 00, 0c, 07, 1b, 00, 00, 27, 01, 00, 00]
        //                                                                                                                                                                                         ^^^^^^
        //
        // Version ours:
        //  02, 96, 00, 35, 00, 04, 10, 3b, 94, f9, 85, 6a, fd, c3, ba, 40, 43, 7f, ac, 11, 88, ab, 34, 07, 02, 01, 00, 06, 10, 91, 52, 76, bb, 26, 00, 00, 80, 00, 10, 00, 00, 3e, 00, 00, 00, 0a, 02, 00, 00, 0c, 07, 04, 00, 00, 27, 01, 00, 00
        // version theirs
        //  02  39  00  35  00  04  10  91  52  76  bb  26  00  00  80  00  10  00  00  37  00  00  00  07  02  10  00  06  10  91  52  76  bb  26  00  00  80  00  10  00  00  a2  00  00  00  0a  02  10  00  0c  07  19  00  00  27  01  00  00
        //
        //
        // --
        // protocol.service signature.
        // [2025-08-16T22:20:56Z WARN  micro_hap::ble] Writing protocol.service_signature  0x[00, 01, 80, 11, 00]
        // [2025-08-16T22:20:56Z WARN  micro_hap::ble] Replying with: Read { data: [
        // 02, 80, 00, 35, 00, 04, 10, 91, 52, 76, bb, 26, 00, 00, 80, 00, 10, 00, 00, a5, 00, 00, 00, 07, 02, 10, 00, 06, 10, 91, 52, 76, bb, 26, 00, 00, 80, 00, 10, 00, 00, a2, 00, 00, 00, 0a, 02, 00, 00, 0c, 07, 1b, 00, 00, 27, 01, 00, 00] }
        // 02, 80, 00, 35, 00, 04, 10, 91, 52, 76, bb, 26, 00, 00, 80, 00, 10, 00, 00, a5, 00, 00, 00, 07, 02, 10, 00, 06, 10, 91, 52, 76, bb, 26, 00, 00, 80, 00, 10, 00, 00, a2, 00, 00, 00, 0a, 02, 10, 00, 0c, 07, 1b, 00, 00, 27, 01, 00, 00
        // Actual:
        //    00 01 b4 11 00
        // 02  b4  00  35  00  04  10  91  52  76  bb  26  00  00  80  00  10  00  00  a5  00  00  00  07  02  10  00  06  10  91  52  76  bb  26  00  00  80  00  10  00  00  a2  00  00  00  0a  02  10  00  0c  07  1b  00  00  27  01  00  00
        // --                                                                                                                                                                                          ^^
        // That one is good now.
        //
        //
        // ==== With fixed service permissions
        // [2025-08-16T23:04:21Z WARN  micro_hap::ble] Writing protocol.service_signature  0x[00, 01, 2a, 11, 00]
        // [2025-08-16T23:04:21Z WARN  micro_hap::ble] Replying with: Read { data:
        // [02, 2a, 00, 35, 00, 04, 10, 91, 52, 76, bb, 26, 00, 00, 80, 00, 10, 00, 00, a5, 00, 00, 00, 07, 02, 10, 00, 06, 10, 91, 52, 76, bb, 26, 00, 00, 80, 00, 10, 00, 00, a2, 00, 00, 00, 0a, 02, 10, 00, 0c, 07, 1b, 00, 00, 27, 01, 00, 00] }
        // truth:
        //                                                                                    00  01  b4  11  00
        //  02  b4  00  35  00  04  10  91  52  76  bb  26  00  00  80  00  10  00  00  a5  00  00  00  07  02  10  00  06  10  91  52  76  bb  26  00  00  80  00  10  00  00  a2  00  00  00  0a  02  10  00  0c  07  1b  00  00  27  01  00  00
        // Checks out.
        //
        // Writing protocol.version  [0, 1, 174, 18, 0]
        // [2025-08-16T23:04:21Z WARN  micro_hap::ble] Replying with: Read { data: [
        //  02, ae, 00, 35, 00, 04, 10, 91, 52, 76, bb, 26, 00, 00, 80, 00, 10, 00, 00, 37, 00, 00, 00, 07, 02, 10, 00, 06, 10, 91, 52, 76, bb, 26, 00, 00, 80, 00, 10, 00, 00, a2, 00, 00, 00, 0a, 02, 00, 00, 0c, 07, 1b, 00, 00, 27, 01, 00, 00] }
        // truth:
        //  02  39  00  35  00  04  10  91  52  76  bb  26  00  00  80  00  10  00  00  37  00  00  00  07  02  10  00  06  10  91  52  76  bb  26  00  00  80  00  10  00  00  a2  00  00  00  0a  02  10  00  0c  07  19  00  00  27  01  00  00
        // -->                                                                                                                                                                                          ^^
        // now:
        // [02, 94, 00, 35, 00, 04, 10, 91, 52, 76, bb, 26, 00, 00, 80, 00, 10, 00, 00, 37, 00, 00, 00, 07, 02, 10, 00, 06, 10, 91, 52, 76, bb, 26, 00, 00, 80, 00, 10, 00, 00, a2, 00, 00, 00, 0a, 02, 10, 00, 0c, 07, 1b, 00, 00, 27, 01, 00, 00]
        // Checks out.
        //
        //
        //
        //
        // Writing pairing.pair_setup  [0, 1, 21, 34, 0]
        // [2025-08-16T23:10:08Z WARN  micro_hap::ble] Replying with: Read { data:
        // [02, 15, 00, 35, 00, 04, 10, 91, 52, 76, bb, 26, 00, 00, 80, 00, 10, 00, 00, 4c, 00, 00, 00, 07, 02, 20, 00, 06, 10, 91, 52, 76, bb, 26, 00, 00, 80, 00, 10, 00, 00, 55, 00, 00, 00, 0a, 02, 03, 00, 0c, 07, 1b, 00, 00, 27, 01, 00, 00] }
        // truth:
        // Does pair verify first! Not pair setup??
        //  02  de  00  35  00  04  10  91  52  76  bb  26  00  00  80  00  10  00  00  4c  00  00  00  07  02  20  00  06  10  91  52  76  bb  26  00  00  80  00  10  00  00  55  00  00  00  0a  02  03  00  0c  07  1b  00  00  27  01  00  00
        // Checks out though.
        //
        // Writing pairing.pair_verify  [0, 1, 62, 35, 0]
        //
        // [2025-08-16T23:10:08Z WARN  micro_hap::ble] Replying with: Read { data:
        // [02, 3e, 00, 35, 00, 04, 10, 91, 52, 76, bb, 26, 00, 00, 80, 00, 10, 00, 00, 4e, 00, 00, 00, 07, 02, 20, 00, 06, 10, 91, 52, 76, bb, 26, 00, 00, 80, 00, 10, 00, 00, 55, 00, 00, 00, 0a, 02, 03, 00, 0c, 07, 1b, 00, 00, 27, 01, 00, 00] }
        // Truth:
        //  02  6d  00  35  00  04  10  91  52  76  bb  26  00  00  80  00  10  00  00  4e  00  00  00  07  02  20  00  06  10  91  52  76  bb  26  00  00  80  00  10  00  00  55  00  00  00  0a  02  03  00  0c  07  1b  00  00  27  01  00  00
        // checks out.
        //

        if let Some(chr) = self.get_attribute_by_char(req.char_id) {
            let mut buffer = self.buffer.borrow_mut();
            // NONCOMPLIANCE: should drop connection when requesting characteristics on the pairing characteristics.
            let srv = self.get_service_by_char(req.char_id).unwrap();
            // https://github.com/apple/HomeKitADK/blob/fb201f98f5fdc7fef6a455054f08b59cca5d1ec8/HAP/HAPBLECharacteristic%2BSignature.c#L10
            let reply = req.header.to_success();
            let len = reply.write_into_length(*buffer)?;

            let len = BodyBuilder::new_at(*buffer, len)
                .add_characteristic_uuid(&chr.uuid)
                // .add_service(SvcId(0x10)) // what is this set to?
                .add_service(srv.iid) // what is this set to?
                .add_service_uuid(&srv.uuid)
                .add_characteristic_properties(chr.ble_ref().properties)
                .add_optional_user_description(&chr.user_description)
                .add_format(&chr.ble_ref().format)
                .end();

            Ok(BufferResponse(len))
        } else {
            todo!()
        }
    }

    pub async fn characteristic_read_request(
        &mut self,
        req: &pdu::CharacteristicReadRequest,
    ) -> Result<BufferResponse, HapBleError> {
        // Well ehm, what do we do here?
        let char_id = req.char_id; // its unaligned, so copy it before we use it.
        if let Some(chr) = self.get_attribute_by_char(char_id) {
            if chr.uuid == crate::characteristic::VERSION.into() {
                // Hardcode this for now, we'll make something smarter later.
                let mut buffer = self.buffer.borrow_mut();
                let reply = req.header.to_success();
                let len = reply.write_into_length(*buffer)?;

                let len = BodyBuilder::new_at(*buffer, len)
                    // Something here that writes "2.2.0"
                    .add_value(&"2.2.0".as_bytes())
                    .end();
                Ok(BufferResponse(len))
            } else if chr.uuid == crate::characteristic::PAIRING_FEATURES.into() {
                //info!("pairing features");
                let mut buffer = self.buffer.borrow_mut();
                let reply = req.header.to_success();
                let len = reply.write_into_length(*buffer)?;
                // Software authentication is 0x2, this is non-const in the reference implementation though.
                let len = BodyBuilder::new_at(*buffer, len)
                    // Changed to null match the reference.
                    .add_value(&0u8.as_bytes())
                    //.add_value(&2u8.as_bytes())
                    .end();
                Ok(BufferResponse(len))
            } else {
                error!(
                    "Got read for characteristic that is not yet handled: {:?}",
                    char_id
                );
                todo!();
            }
        } else {
            error!("Got read for unknown characteristic: {:?}", char_id);
            todo!();
        }
    }

    #[allow(unreachable_code)]
    pub async fn info_request(
        &mut self,
        req: &pdu::InfoRequest,
    ) -> Result<BufferResponse, HapBleError> {
        let _ = req;
        // https://github.com/apple/HomeKitADK/blob/fb201f98f5fdc7fef6a455054f08b59cca5d1ec8/HAP/HAPAccessory%2BInfo.c#L71
        // This is a blind attempt at implementing this request based on the reference.
        // I needed this when I set the software authentication bit, but it seems that the reference doesn't actually
        // use that code path, so putting a todo here to ensure we fail hard.
        todo!("this needs checking against the reference");
        let char_id = req.char_id; // its unaligned, so copy it before we use it.
        if let Some(chr) = self.get_attribute_by_char(char_id) {
            if chr.uuid == crate::characteristic::SERVICE_SIGNATURE.into() {
                let mut buffer = self.buffer.borrow_mut();
                let reply = req.header.to_success();
                let len = reply.write_into_length(*buffer)?;

                let pair_ctx = self.pair_ctx.borrow();
                let setup_hash = crate::adv::calculate_setup_hash(
                    &pair_ctx.accessory.device_id,
                    &pair_ctx.accessory.setup_id,
                );
                let len = BodyBuilder::new_at(*buffer, len)
                    // 1 is good enough for the ip side, probably also for bluetooth?
                    .add_u16(pdu::InfoResponseTLVType::StateNumber, 1)
                    // Config number, we should increment this every reconfiguration I think? Ignore for now.
                    // https://github.com/apple/HomeKitADK/blob/fb201f98f5fdc7fef6a455054f08b59cca5d1ec8/HAP/HAPAccessoryServer.c#L1058
                    .add_u8(pdu::InfoResponseTLVType::ConfigNumber, 1)
                    // https://github.com/apple/HomeKitADK/blob/fb201f98f5fdc7fef6a455054f08b59cca5d1ec8/HAP/HAPAccessory%2BInfo.c#L136
                    .add_info_device_id(&pair_ctx.accessory.device_id)
                    // Feature flags, 2 is software authentication only.
                    .add_u8(pdu::InfoResponseTLVType::FeatureFlags, 2)
                    // Next is param-model. is that always a string?
                    .add_slice(
                        pdu::InfoResponseTLVType::ModelName,
                        pair_ctx.accessory.model.as_bytes(),
                    )
                    // And then protocol version.
                    .add_slice(
                        pdu::InfoResponseTLVType::ProtocolVersion,
                        "2.2.0".as_bytes(),
                    )
                    // Status flag... https://github.com/apple/HomeKitADK/blob/fb201f98f5fdc7fef6a455054f08b59cca5d1ec8/HAP/HAPAccessoryServer.c#L924
                    // Lets just report not paired all the time: 1.
                    // Lets try paired now, since we're in a secure session? 0
                    .add_u8(pdu::InfoResponseTLVType::StatusFlag, 0)
                    // Category
                    .add_u16(
                        pdu::InfoResponseTLVType::CategoryIdentifier,
                        pair_ctx.accessory.category,
                    )
                    // Finally, the setup hash... Does this value matter?
                    .add_slice(pdu::InfoResponseTLVType::SetupHash, &setup_hash)
                    .end();
                Ok(BufferResponse(len))
            } else {
                error!(
                    "Got info for characteristic that is not yet handled: {:?}",
                    char_id
                );
                todo!();
            }
        } else {
            error!("Got info read for unknown characteristic: {:?}", char_id);
            todo!();
        }
    }

    async fn reply_read_payload<'stack, P: trouble_host::PacketPool>(
        //&self,
        data: &[u8],
        event: ReadEvent<'stack, '_, P>,
    ) -> Result<(), trouble_host::Error> {
        // let buffer = self.buffer.borrow();

        //let data = self.get_response(reply.payload);
        let reply = trouble_host::att::AttRsp::Read { data: &data };
        warn!("Replying with: {:02x?}", reply);
        // We see this print,
        //
        // but nothing ever ends up in the aether
        event.into_payload().reply(reply).await?;
        Ok(())
    }

    fn get_response(&self, reply: BufferResponse) -> core::cell::Ref<'_, [u8]> {
        // self.buffer.borrow().map(|z| &z[0..reply.payload.0])
        core::cell::Ref::<'_, &'static mut [u8]>::map(self.buffer.borrow(), |z| &z[0..reply.0])
    }

    pub fn encrypted_reply(
        &mut self,
        value: BufferResponse,
    ) -> Result<BufferResponse, HapBleError> {
        let should_encrypt = self.should_encrypt_reply;
        if should_encrypt {
            // Perform the encryption, then respond with the buffer that is encrypted.
            let mut ctx = self.pair_ctx.borrow_mut();
            let mut buff = self.buffer.borrow_mut();
            info!("Encrypting reply: {:0>2x?}", &buff[0..value.0]);

            let res = ctx.session.a_to_c.encrypt(&mut **buff, value.0)?;
            info!("Encrypted reply: {:0>2x?}", &res);

            Ok(BufferResponse(res.len()))
        } else {
            Ok(value)
        }
    }

    pub async fn handle_read_outgoing(
        &mut self,
        handle: u16,
    ) -> Result<Option<core::cell::Ref<'_, [u8]>>, HapBleError> {
        if self.prepared_reply.as_ref().map(|e| e.handle) == Some(handle) {
            let reply = self.prepared_reply.take().unwrap();
            // Ensure that we send the encrypted data!
            let buffered_response = self.encrypted_reply(reply.payload)?;
            Ok(Some(self.get_response(buffered_response)))
        } else {
            Ok(None)
        }
    }

    pub async fn handle_write_incoming<'hap, 'support>(
        &mut self,
        hap: &HapServices<'hap>,
        support: &crate::pairing::PairSupport<'support>,
        data: &[u8],
        handle: u16,
    ) -> Result<Option<BufferResponse>, HapBleError> {
        let security_active = self.pair_ctx.borrow().session.security_active;
        self.should_encrypt_reply = security_active;
        let mut tmp_buffer = [0u8; 1024];
        let data = if security_active {
            warn!("handle_write_incoming raw {:0>2x?}", data);
            // Raw write data [49, f0, c7, b1, 91, d4, d9, f9, 44, b9, 50, f0, c4, 67, a6, 6, c8, 6d, f9, fe, dc]
            // Raw write data [ed, 4c, 8a, f4, 7e, ca, bf, 1a, 1, 9, 55, 6e, 95, 24, dc, a, 7a, 7d, 83, 3d, 30]
            // Yes, these are encrypted.
            //
            // Collect the context
            let buffer = &mut tmp_buffer;
            let mut pair_ctx = self.pair_ctx.borrow_mut();

            // Copy the payload into the buffer
            buffer.fill(0);
            // parsed.copy_body(&mut *buffer)?;
            buffer[0..data.len()].copy_from_slice(data);

            pair_ctx
                .session
                .c_to_a
                .decrypt(&mut buffer[0..data.len()])?
        } else {
            data
        };
        warn!("handle_write_incoming {:0>2x?}", data);

        let header = pdu::RequestHeader::parse_pdu(data)?;
        warn!("Write header {:x?}", header);

        #[allow(unreachable_code)]
        let resp = match header.opcode {
            pdu::OpCode::ServiceSignatureRead => {
                let req = pdu::ServiceSignatureReadRequest::parse_pdu(data)?;
                self.service_signature_request(&req).await?
            }
            pdu::OpCode::CharacteristicSignatureRead => {
                // second one is on [0, 1, 44, 2, 2]
                let req = pdu::CharacteristicSignatureReadRequest::parse_pdu(data)?;
                warn!("Got req: {:?}", req);
                self.characteristic_signature_request(&req).await?
            }
            pdu::OpCode::CharacteristicRead => {
                let req = pdu::CharacteristicReadRequest::parse_pdu(data)?;
                warn!("Got req: {:?}", req);
                self.characteristic_read_request(&req).await?
            }
            pdu::OpCode::CharacteristicWrite => {
                info!("handle is: {}", handle);
                info!("pair setup handle is {}", hap.pairing.pair_setup.handle);
                info!("write raw req event data: {:02x?}", data);
                let parsed = pdu::CharacteristicWriteRequest::parse_pdu(data)?;
                info!("got write on pair setup with: {:?}", parsed);

                // Write the body to our internal buffer here.
                let mut buffer = self.buffer.borrow_mut();
                buffer.fill(0);
                parsed.copy_body(&mut *buffer)?;
                let mut pair_ctx = self.pair_ctx.borrow_mut();

                // So now we craft the reply, technically this could happen on the read... should it happen on the read?

                let full_len = buffer.len();
                let (first_half, mut second_half) = buffer.split_at_mut(full_len / 2);
                let outgoing_len = if handle == hap.pairing.pair_setup.handle {
                    crate::pairing::pair_setup_handle_incoming(
                        &mut **pair_ctx,
                        support,
                        &*first_half,
                    )
                    .map_err(|_| HapBleError::InvalidValue)?;

                    // Put the reply in the second half.
                    let outgoing_len = crate::pairing::pair_setup_handle_outgoing(
                        &mut **pair_ctx,
                        support,
                        &mut second_half,
                    )
                    .map_err(|_| HapBleError::InvalidValue)?;
                    outgoing_len
                } else if handle == hap.pairing.pair_verify.handle {
                    crate::pair_verify::handle_incoming(&mut **pair_ctx, support, &*first_half)
                        .map_err(|_| HapBleError::InvalidValue)?;

                    // Put the reply in the second half.
                    let outgoing_len = crate::pair_verify::handle_outgoing(
                        &mut **pair_ctx,
                        support,
                        &mut second_half,
                    )
                    .map_err(|_| HapBleError::InvalidValue)?;
                    outgoing_len
                } else {
                    todo!("Need dispatch to correct method");
                };
                let reply = parsed.header.header.to_success();
                let len = reply.write_into_length(first_half)?;

                let len = BodyBuilder::new_at(first_half, len)
                    .add_value(&second_half[0..outgoing_len])
                    .end();

                BufferResponse(len)
            }
            pdu::OpCode::Info => {
                // https://github.com/apple/HomeKitADK/blob/fb201f98f5fdc7fef6a455054f08b59cca5d1ec8/HAP/HAPBLEProcedure.c#L623
                // we don't have this in the recording?!?
                if !security_active {
                    //return Err(HapBleError::EncryptionError);
                }
                let req = pdu::InfoRequest::parse_pdu(data)?;
                info!("Info req: {:?}", req);
                self.info_request(&req).await?
            }
            _ => {
                return {
                    error!("Failed to handle: {:?}", header);
                    Err(HapBleError::UnexpectedRequest.into())
                };
            }
        };
        Ok(Some(resp))
    }

    #[cfg(test)]
    // helper function to store the reply into the prepared reply.
    async fn handle_write_incoming_test<'hap, 'support>(
        &mut self,
        hap: &HapServices<'hap>,
        support: &crate::pairing::PairSupport<'support>,
        data: &[u8],
        handle: u16,
    ) -> Result<Option<BufferResponse>, HapBleError> {
        let resp = self.handle_write_incoming(hap, support, &data, handle);

        info!("pair verify handle: {:?}", hap.pairing.pair_verify.handle());
        if let Some(resp) = resp.await? {
            self.prepared_reply = Some(Reply {
                payload: resp,
                handle: handle,
            });
            Ok(Some(resp))
        } else {
            panic!("testing something unhandled?")
        }
    }

    pub async fn process_gatt_event<'stack, 'server, 'hap, 'support, P: PacketPool>(
        &mut self,
        hap: &HapServices<'hap>,
        support: &crate::pairing::PairSupport<'support>,
        event: trouble_host::gatt::GattEvent<'stack, 'server, P>,
    ) -> Result<Option<trouble_host::gatt::GattEvent<'stack, 'server, P>>, trouble_host::Error>
    {
        // we seem to miss 'read by type' requests on handlex 0x0010 - 0x0012

        match event {
            GattEvent::Read(event) => {
                if event.handle() == hap.information.hardware_revision.handle {
                    warn!("Reading information.hardware_revision");
                } else if event.handle() == hap.information.serial_number.handle {
                    warn!("Reading information.serial_number ");
                } else if event.handle() == hap.information.model.handle {
                    warn!("Reading information.model ");
                } else if event.handle() == hap.information.name.handle {
                    warn!("Reading information.name ");
                } else if event.handle() == hap.information.manufacturer.handle {
                    warn!("Reading information.manufacturer ");
                } else if event.handle() == hap.information.firmware_revision.handle {
                    warn!("Reading information.firmware_revision ");
                } else if event.handle() == hap.information.service_instance.handle {
                    warn!("Reading information.service_instance ");
                }

                if event.handle() == hap.protocol.service_instance.handle {
                    warn!("Reading protocol.service_instance");
                } else if event.handle() == hap.protocol.service_signature.handle {
                    warn!("Reading protocol.service_signature ");
                } else if event.handle() == hap.protocol.version.handle {
                    warn!("Reading protocol.version ");
                }

                if event.handle() == hap.pairing.service_instance.handle {
                    warn!("Reading pairing.service_instance");
                } else if event.handle() == hap.pairing.pair_setup.handle {
                    warn!("Reading pairing.pair_setup ");
                } else if event.handle() == hap.pairing.pair_verify.handle {
                    warn!("Reading pairing.pair_verify ");
                } else if event.handle() == hap.pairing.features.handle {
                    warn!("Reading pairing.features ");
                } else if event.handle() == hap.pairing.pairings.handle {
                    warn!("Reading pairing.pairings ");
                }

                if let Some(buffer_thing) = self.handle_read_outgoing(event.handle()).await? {
                    Self::reply_read_payload(&*buffer_thing, event).await?;

                    return Ok(None);
                }

                Ok(Some(GattEvent::Read(event)))
            }
            GattEvent::Write(event) => {
                warn!("Raw write data {:0>2x?}", event.data());

                if event.handle() == hap.information.hardware_revision.handle {
                    warn!("Writing information.hardware_revision {:?}", event.data());
                } else if event.handle() == hap.information.serial_number.handle {
                    warn!("Writing information.serial_number  {:?}", event.data());
                } else if event.handle() == hap.information.model.handle {
                    warn!("Writing information.model  {:?}", event.data());
                } else if event.handle() == hap.information.name.handle {
                    warn!("Writing information.name  {:?}", event.data());
                } else if event.handle() == hap.information.manufacturer.handle {
                    warn!("Writing information.manufacturer  {:?}", event.data());
                } else if event.handle() == hap.information.firmware_revision.handle {
                    warn!("Writing information.firmware_revision  {:?}", event.data());
                } else if event.handle() == hap.information.service_instance.handle {
                    warn!("Writing information.service_instance  {:?}", event.data());
                }

                if event.handle() == hap.protocol.service_instance.handle {
                    warn!("Writing protocol.service_instance  {:?}", event.data());
                } else if event.handle() == hap.protocol.service_signature.handle {
                    warn!(
                        "Writing protocol.service_signature  0x{:02x?}",
                        event.data()
                    );
                    // Writing protocol.service_signature  [0, 6, 107, 2, 0]
                    // Yes, that matches the hap service signature read

                    // Maybe the write request has to go through and it is followed by a read?
                } else if event.handle() == hap.protocol.version.handle {
                    warn!("Writing protocol.version  {:?}", event.data());
                }

                if event.handle() == hap.pairing.service_instance.handle {
                    warn!("Writing pairing.service_instance");
                } else if event.handle() == hap.pairing.pair_setup.handle {
                    warn!("Writing pairing.pair_setup  {:?}", event.data());
                    // [0, 1, 62, 0, 34]
                } else if event.handle() == hap.pairing.pair_verify.handle {
                    warn!("Writing pairing.pair_verify  {:?}", event.data());
                } else if event.handle() == hap.pairing.features.handle {
                    warn!("Writing pairing.features  {:?}", event.data());
                } else if event.handle() == hap.pairing.pairings.handle {
                    warn!("Writing pairing.pairings  {:?}", event.data());
                }

                let resp = self.handle_write_incoming(hap, support, &event.data(), event.handle());
                if let Some(resp) = resp.await? {
                    self.prepared_reply = Some(Reply {
                        payload: resp,
                        handle: event.handle(),
                    });

                    let reply = trouble_host::att::AttRsp::Write;
                    event.into_payload().reply(reply).await?;
                    return Ok(None);
                } else {
                    todo!("unhandled event");
                }
            }
            remainder => Ok(Some(remainder)),
        }
    }
}

#[cfg(test)]
mod test {
    use super::*;
    use static_cell::StaticCell;

    #[gatt_server]
    struct Server {
        accessory_information: AccessoryInformationService,
        protocol: ProtocolInformationService,
        pairing: PairingService,
    }
    impl Server<'_> {
        pub fn as_hap(&self) -> HapServices<'_> {
            HapServices {
                information: &self.accessory_information,
                protocol: &self.protocol,
                pairing: &self.pairing,
            }
        }
    }

    #[tokio::test]
    async fn test_message_exchanges() -> Result<(), HapBleError> {
        crate::test::init();
        let name = "micro_hap";
        let server = Server::new_with_config(GapConfig::Peripheral(PeripheralConfig {
            name,
            appearance: &appearance::power_device::GENERIC_POWER_DEVICE,
        }))
        .unwrap();

        // Setup the accessory information.
        let value = crate::AccessoryInformationStatic {
            name,
            // Possibly device id 57:3B:20:A7:E7:C4 ?
            device_id: crate::DeviceId([0x57, 0x3b, 0x20, 0xA7, 0xE7, 0xC4]),
            ..Default::default()
        };
        let _ = server
            .accessory_information
            .set_information_static(&server, &value)
            .unwrap();

        let hap = server.as_hap();
        let _ = hap;

        let buffer: &mut [u8] = {
            static STATE: StaticCell<[u8; 2048]> = StaticCell::new();
            STATE.init([0u8; 2048])
        };

        let pair_ctx = {
            static STATE: StaticCell<crate::pairing::PairContext> = StaticCell::new();
            STATE.init_with(crate::pairing::PairContext::default)
        };
        pair_ctx.accessory = value;

        // We need real commissioning for this.
        // from recording 2025_08_24_1639.
        pair_ctx.info.salt = [
            0x3d, 0xc2, 0x81, 0xab, 0x08, 0xed, 0x4d, 0x8c, 0x52, 0x0c, 0xb2, 0x5f, 0xc2, 0x51,
            0x9c, 0x1f,
        ];
        pair_ctx.info.verifier = [
            0xe3, 0x7e, 0xa0, 0xd4, 0x45, 0xab, 0x91, 0xcc, 0xee, 0x92, 0x33, 0x20, 0x9e, 0xb3,
            0x8f, 0xfc, 0xd7, 0x04, 0x20, 0xd1, 0x95, 0x34, 0x73, 0x5a, 0x17, 0x2e, 0xca, 0xef,
            0xe3, 0x8d, 0x1a, 0x21, 0xfb, 0x5e, 0x2d, 0x18, 0x1b, 0xb0, 0x80, 0x77, 0x12, 0xf7,
            0x2d, 0x2e, 0x64, 0x67, 0xc7, 0xa8, 0xb5, 0xc0, 0xe3, 0xab, 0xe4, 0x60, 0x58, 0x9f,
            0xde, 0x39, 0x62, 0xdc, 0x70, 0x01, 0x42, 0x1a, 0x07, 0x47, 0x16, 0x63, 0xf7, 0xd7,
            0xee, 0x9b, 0xf9, 0x7b, 0x35, 0xc4, 0x3b, 0x5d, 0x0a, 0xd6, 0x07, 0xdb, 0x47, 0x84,
            0x05, 0x22, 0x9b, 0xc8, 0x0f, 0xb3, 0xb4, 0x39, 0xc7, 0x18, 0xc9, 0xb0, 0x85, 0x8d,
            0x19, 0xf5, 0x56, 0xc6, 0xee, 0x9b, 0xd8, 0x87, 0x8a, 0x39, 0xf9, 0x21, 0x35, 0xaa,
            0x42, 0x50, 0x6d, 0xa3, 0x5a, 0x3f, 0x67, 0x55, 0x6a, 0x5c, 0x6c, 0x92, 0x07, 0x44,
            0xd3, 0xd6, 0x97, 0x6b, 0x5a, 0x5c, 0xcf, 0x6b, 0xdf, 0xf5, 0x1d, 0x4c, 0xde, 0x3f,
            0x2d, 0xf7, 0x95, 0x3c, 0x70, 0xde, 0x65, 0xcf, 0x22, 0x96, 0xe8, 0x12, 0x8f, 0xa7,
            0x9a, 0xa7, 0x68, 0xfe, 0x00, 0x18, 0x7f, 0x6d, 0xed, 0x98, 0xc9, 0x6b, 0xfc, 0xd2,
            0x9b, 0xa9, 0x08, 0x93, 0x3e, 0x3e, 0x7f, 0x7c, 0x63, 0x03, 0x49, 0xdf, 0x52, 0x18,
            0xcf, 0x9f, 0xf3, 0xbb, 0x11, 0xb5, 0xa3, 0x05, 0x03, 0x6b, 0xba, 0xf8, 0x91, 0x60,
            0xc2, 0xf1, 0x1e, 0x5f, 0x0c, 0x81, 0x08, 0x25, 0xda, 0xed, 0xef, 0xa0, 0xfe, 0x73,
            0xbf, 0xd8, 0xe3, 0xdb, 0xdc, 0xf6, 0x54, 0x42, 0x9a, 0xea, 0xf2, 0x69, 0x46, 0x14,
            0x0c, 0x86, 0x97, 0x56, 0x95, 0x8b, 0x5b, 0x1f, 0x87, 0x99, 0x5c, 0xaf, 0x6a, 0xf4,
            0xe5, 0x66, 0xe9, 0xf9, 0x7b, 0xa5, 0x1f, 0xf8, 0x8e, 0xa7, 0x81, 0xcc, 0x4e, 0xdd,
            0x20, 0x94, 0x2d, 0x31, 0x78, 0xb6, 0x26, 0xf6, 0x41, 0x07, 0xa7, 0xad, 0x97, 0x18,
            0xff, 0x7a, 0x0f, 0x3c, 0x55, 0x4b, 0xc3, 0x4d, 0x58, 0xc9, 0x56, 0xed, 0x6b, 0x69,
            0xc4, 0x56, 0xf4, 0xf0, 0x5f, 0x58, 0x7f, 0x98, 0xfa, 0x4a, 0xf7, 0x8e, 0xda, 0x49,
            0xc8, 0x69, 0x88, 0xae, 0x9c, 0x39, 0x1f, 0xa2, 0xc4, 0x58, 0x78, 0x35, 0xba, 0x73,
            0x01, 0xae, 0xa2, 0xa9, 0x4d, 0x90, 0xf3, 0x98, 0x14, 0xb9, 0x6f, 0x4f, 0x21, 0x01,
            0xdd, 0xad, 0x1a, 0x52, 0x45, 0x13, 0xe9, 0x08, 0xb0, 0x89, 0x54, 0xee, 0xe3, 0x44,
            0x08, 0xd4, 0x77, 0x4b, 0xab, 0x65, 0x6e, 0xba, 0xec, 0xf9, 0xce, 0x9d, 0x5f, 0xd5,
            0x4a, 0xde, 0xdf, 0x8f, 0x67, 0x47, 0x65, 0xe2, 0x2f, 0x8f, 0x9f, 0x53, 0xab, 0x56,
            0xb1, 0x22, 0x6c, 0xe3, 0x5c, 0x8e, 0x97, 0x2f, 0x9f, 0x82, 0xf1, 0xd2, 0x11, 0x2f,
            0x1a, 0xc3, 0x2a, 0x60, 0x28, 0x83,
        ];

        // ed_LTSK random bytes:
        let ed_ltsk = [
            0x15, 0xf5, 0xa7, 0xdb, 0xa0, 0x11, 0x21, 0xea, 0x23, 0xea, 0x88, 0x7f, 0x0a, 0x14,
            0xb0, 0x27, 0xb6, 0xe6, 0xd4, 0x2d, 0xd1, 0x5b, 0xc9, 0x59, 0x19, 0x94, 0xbc, 0x22,
            0xee, 0x52, 0xfa, 0xa9,
        ];

        let mut ctx = HapPeripheralContext::new(
            buffer,
            pair_ctx,
            &server.accessory_information,
            &server.protocol,
            &server.pairing,
        )?;

        let counter = core::cell::RefCell::new(0);
        let random_buffer = vec![
            0x75, 0x35, 0xcb, 0x53, 0x6e, 0xbb, 0x8c, 0x63, 0x94, 0xf5, 0x85, 0xe6, 0x7d, 0xc5,
            0x65, 0x2d, 0x83, 0xe4, 0xea, 0x76, 0x4c, 0xa3, 0x61, 0xe3, 0x85, 0xca, 0x07, 0x57,
            0x29, 0x47, 0x2d, 0x55,
        ];
        let rng = move || {
            let mut c = counter.borrow_mut();
            let v = random_buffer[*c];
            *c += 1;
            v
        };

        let support = crate::pairing::PairSupport { rng: &rng, ed_ltsk };

        let service_signature_req = [0, 6, 0x3a, 0x10, 0];
        let service_signature_req =
            pdu::ServiceSignatureReadRequest::parse_pdu(&service_signature_req)?;
        let resp = ctx
            .service_signature_request(&service_signature_req)
            .await?;
        {
            let reply = ctx.get_response(resp);
            warn!("reply: {:?}", reply);

            let expected_req = [2, 0x3a, 0, 6, 0, 0xf, 2, 4, 0, 0x10, 0];
            assert_eq!(&*reply, expected_req);
        }

        // and then it sends... which we have no clue what it is yet.
        let payload = [0x00, 0x01, 0xae, 0x20, 0x00];
        // [micro_hap::ble] Writing protocol.service_signature  [0, 1, ae, 2, 2]
        let service_signature_req = pdu::ServiceSignatureReadRequest::parse_pdu(&payload)?;
        let resp = ctx
            .service_signature_request(&service_signature_req)
            .await?;
        // Oh... that's a characteristic signature read request.
        // we need the PDU types, and interpret based on that.
        let _ = resp;

        // Test service signature response ------------
        {
            let incoming_data: &[u8] = &[0x00, 0x06, 0x0d, 0x10, 0x00];
            let handle = 0x11;

            let outgoing_data: &[u8] = &[
                0x02, 0x0d, 0x00, 0x06, 0x00, 0x0f, 0x02, 0x04, 0x00, 0x10, 0x00,
            ];
            ctx.handle_write_incoming_test(&hap, &support, incoming_data, handle)
                .await?;

            let resp = ctx.handle_read_outgoing(handle).await?;
            let resp_buffer = resp.expect("expecting a outgoing response");

            assert_eq!(&*resp_buffer, outgoing_data);
        }
        {
            let incoming_data: &[u8] = &[0x00, 0x01, 0x9c, 0x11, 0x00];
            let handle = 0x11;

            let outgoing_data: &[u8] = &[
                0x02, 0x9c, 0x00, 0x35, 0x00, 0x04, 0x10, 0x91, 0x52, 0x76, 0xbb, 0x26, 0x00, 0x00,
                0x80, 0x00, 0x10, 0x00, 0x00, 0xa5, 0x00, 0x00, 0x00, 0x07, 0x02, 0x10, 0x00, 0x06,
                0x10, 0x91, 0x52, 0x76, 0xbb, 0x26, 0x00, 0x00, 0x80, 0x00, 0x10, 0x00, 0x00, 0xa2,
                0x00, 0x00, 0x00, 0x0a, 0x02, 0x10, 0x00, 0x0c, 0x07, 0x1b, 0x00, 0x00, 0x27, 0x01,
                0x00, 0x00,
            ];
            ctx.handle_write_incoming_test(&hap, &support, incoming_data, handle)
                .await?;

            let resp = ctx.handle_read_outgoing(handle).await?;
            let resp_buffer = resp.expect("expecting a outgoing response");

            assert_eq!(&*resp_buffer, outgoing_data);
        }
        //   ----------------------------------

        // Check pairing features  ------------
        {
            let incoming_data: &[u8] = &[0x00, 0x01, 0x56, 0x24, 0x00];
            let handle = 0x24;
            let outgoing_data: &[u8] = &[
                0x02, 0x56, 0x00, 0x35, 0x00, 0x04, 0x10, 0x91, 0x52, 0x76, 0xbb, 0x26, 0x00, 0x00,
                0x80, 0x00, 0x10, 0x00, 0x00, 0x4f, 0x00, 0x00, 0x00, 0x07, 0x02, 0x20, 0x00, 0x06,
                0x10, 0x91, 0x52, 0x76, 0xbb, 0x26, 0x00, 0x00, 0x80, 0x00, 0x10, 0x00, 0x00, 0x55,
                0x00, 0x00, 0x00, 0x0a, 0x02, 0x01, 0x00, 0x0c, 0x07, 0x04, 0x00, 0x00, 0x27, 0x01,
                0x00, 0x00,
            ];
            ctx.handle_write_incoming_test(&hap, &support, incoming_data, handle)
                .await?;

            let resp = ctx.handle_read_outgoing(handle).await?;
            let resp_buffer = resp.expect("expecting a outgoing response");

            // This is different
            // Expected:
            // 2, 86, 0, 53, 0, 4, 16, 145, 82, 118, 187, 38, 0, 0, 128, 0, 16, 0, 0, 79, 0, 0, 0, 7, 2, 32, 0, 6, 16, 145, 82, 118, 187, 38, 0, 0, 128, 0, 16, 0, 0, 85, 0, 0, 0, 10, 2, 1, 0, 12, 7, 27, 0, 0, 39, 1, 0, 0
            // Got:
            // 2, 86, 0, 53, 0, 4, 16, 145, 82, 118, 187, 38, 0, 0, 128, 0, 16, 0, 0, 79, 0, 0, 0, 7, 2, 32, 0, 6, 16, 145, 82, 118, 187, 38, 0, 0, 128, 0, 16, 0, 0, 85, 0, 0, 0, 10, 2, 1, 0, 12, 7, 4, 0, 0, 39, 1, 0, 0
            //                                                                                                                                                                                         ^^

            assert_eq!(&*resp_buffer, outgoing_data);
        }
        {
            let incoming_data: &[u8] = &[0x00, 0x03, 0x58, 0x24, 0x00];
            let handle = 0x24;
            let outgoing_data: &[u8] = &[0x02, 0x58, 0x00, 0x03, 0x00, 0x01, 0x01, 0x00];
            ctx.handle_write_incoming_test(&hap, &support, incoming_data, handle)
                .await?;

            let resp = ctx.handle_read_outgoing(handle).await?;
            let resp_buffer = resp.expect("expecting a outgoing response");

            // This is different
            //   left: [2, 88, 0, 3, 0, 1, 1, 2]
            //   right: [2, 88, 0, 3, 0, 1, 1, 0]

            assert_eq!(&*resp_buffer, outgoing_data);
        }
        //   ----------------------------------

        // Incoming info request  ------------
        if false {
            // This appears to only happen during the software authentication workflow, which we may not need?
            let incoming_data: &[u8] = &[0x00, 0x12, 0xbe, 0x11, 0x00];
            let handle = 0x11;

            // We don't know what outgoing should be here.
            ctx.handle_write_incoming_test(&hap, &support, incoming_data, handle)
                .await?;

            let resp = ctx.handle_read_outgoing(handle).await?;
            let resp_buffer = resp.expect("expecting a outgoing response");
            info!("outgoing: {:0>2x?}", &*resp_buffer);
        }

        // Writing to pair setup.  ------------
        // it would be nice if we understood why the handle ids are different.
        let handle_pair_setup = 84;
        let handle_pair_verify = 87;
        // m1 & m2
        {
            let incoming_data: &[u8] = &[
                0x00, 0x02, 0xf3, 0x22, 0x00, 0x0b, 0x00, 0x01, 0x06, 0x00, 0x01, 0x00, 0x06, 0x01,
                0x01, 0x09, 0x01, 0x01,
            ];
            let outgoing: &[u8] = &[
                0x02, 0xf3, 0x00, 0x9d, 0x01, 0x01, 0xff, 0x06, 0x01, 0x02, 0x03, 0xff, 0x64, 0x43,
                0x37, 0x03, 0x65, 0x86, 0x5d, 0x21, 0x46, 0xa6, 0x85, 0x54, 0x0d, 0x38, 0x8a, 0x51,
                0x84, 0xb8, 0x35, 0x51, 0x90, 0x69, 0x57, 0xd2, 0x38, 0x5b, 0xab, 0xdc, 0x5a, 0xf3,
                0x97, 0xbc, 0xdc, 0x35, 0x24, 0x31, 0x99, 0x17, 0xcc, 0xf2, 0x5c, 0xbb, 0x6e, 0x3c,
                0x4a, 0x5b, 0x35, 0x83, 0x7c, 0x20, 0x60, 0x0f, 0x45, 0x79, 0x39, 0x3e, 0xfa, 0x90,
                0xfc, 0xa0, 0x5c, 0x71, 0xe5, 0x1b, 0x39, 0xf8, 0x8c, 0x4e, 0xd8, 0xe6, 0xcf, 0xb9,
                0xdc, 0x05, 0xb6, 0x18, 0x75, 0x2b, 0xa4, 0xc8, 0x90, 0x06, 0x66, 0x80, 0x85, 0x92,
                0x7a, 0x80, 0xd4, 0x08, 0x3a, 0xfc, 0x36, 0x40, 0xad, 0xa3, 0x7b, 0xdc, 0xa2, 0x6b,
                0x49, 0x71, 0x0a, 0x25, 0xc1, 0x97, 0x27, 0x7f, 0x8f, 0x8e, 0x28, 0xa1, 0xf9, 0xff,
                0x6a, 0x87, 0x32, 0x29, 0x72, 0x24, 0x59, 0x4a, 0xf3, 0xfa, 0xcd, 0xe5, 0xae, 0xe7,
                0x3e, 0x90, 0xa5, 0xb0, 0xfa, 0x9e, 0x80, 0x2b, 0xe0, 0x53, 0x33, 0xf2, 0xe7, 0x4b,
                0x6b, 0xdd, 0x56, 0x69, 0x9b, 0x40, 0xed, 0x24, 0xbd, 0x98, 0x23, 0xc2, 0x7b, 0x68,
                0xb7, 0xd9, 0x8f, 0xd6, 0xb4, 0x52, 0x90, 0x42, 0x07, 0xd5, 0x48, 0x63, 0xe0, 0xc6,
                0xd7, 0x18, 0x95, 0xc6, 0xc0, 0x8f, 0x80, 0xe7, 0xc6, 0x02, 0x7c, 0x06, 0x19, 0x8f,
                0x9f, 0xcc, 0xa7, 0x80, 0x67, 0x85, 0x2b, 0xa8, 0x8d, 0x11, 0xcd, 0xdd, 0xa9, 0x98,
                0xa4, 0x75, 0xe8, 0xde, 0xec, 0xfc, 0xf4, 0x92, 0x0d, 0x26, 0xb4, 0x10, 0xbc, 0xc4,
                0x48, 0x98, 0x07, 0x5b, 0x5e, 0x0f, 0x63, 0x47, 0x33, 0xe0, 0x50, 0xc0, 0xbe, 0x8a,
                0x9d, 0x31, 0xe0, 0x44, 0x7d, 0x26, 0x62, 0xf1, 0xc4, 0x98, 0x2b, 0x6d, 0x08, 0x5b,
                0xde, 0xac, 0xea, 0x83, 0xf7, 0x8a, 0x6f, 0xa6, 0x2d, 0x6d, 0x01, 0x9a, 0x54, 0x8a,
                0xc5, 0xf9, 0x7d, 0x03, 0x81, 0xc7, 0x65, 0x77, 0xe1, 0x64, 0x9c, 0xad, 0x5f, 0x28,
                0x78, 0xc8, 0x25, 0x57, 0x89, 0x00, 0xff, 0x7e, 0xc9, 0x9f, 0x4e, 0x87, 0x43, 0xe9,
                0x1a, 0x05, 0x6d, 0xcd, 0x50, 0x2c, 0xa2, 0x85, 0x52, 0xef, 0x7a, 0x8a, 0xf1, 0xe0,
                0x3a, 0x38, 0x2a, 0x76, 0x1c, 0x61, 0xaf, 0x06, 0xb3, 0xf9, 0x3d, 0x8b, 0xb6, 0x1b,
                0xab, 0x6c, 0x14, 0xa3, 0x7b, 0xe0, 0x4c, 0x45, 0x3c, 0xb5, 0x95, 0x2e, 0x96, 0xc5,
                0xb5, 0x23, 0xc7, 0x9e, 0xf6, 0xdd, 0xa3, 0xa2, 0x67, 0x6d, 0x7d, 0x54, 0x44, 0xe1,
                0x3b, 0x4c, 0xaa, 0xf3, 0x99, 0x89, 0xc9, 0xa0, 0x23, 0x6f, 0xf2, 0x94, 0x60, 0x7b,
                0x64, 0x1f, 0x1f, 0xea, 0xa2, 0x11, 0x63, 0x42, 0x10, 0xfb, 0x3c, 0xeb, 0x97, 0x9f,
                0x07, 0xc5, 0x9e, 0x7c, 0x54, 0x2b, 0xd6, 0x6d, 0x21, 0x5d, 0x3e, 0x26, 0x50, 0x80,
                0x0b, 0xa1, 0xce, 0xdb, 0xc0, 0x99, 0x3c, 0x16, 0x02, 0x10, 0x3d, 0xc2, 0x81, 0xab,
                0x08, 0xed, 0x4d, 0x8c, 0x52, 0x0c, 0xb2, 0x5f, 0xc2, 0x51, 0x9c, 0x1f,
            ];
            ctx.handle_write_incoming_test(&hap, &support, incoming_data, handle_pair_setup)
                .await?;

            let resp = ctx.handle_read_outgoing(handle_pair_setup).await?;
            let resp_buffer = resp.expect("expecting a outgoing response");
            info!("outgoing: {:0>2x?}", &*resp_buffer);
            assert_eq!(&*resp_buffer, outgoing);
        }
        // m3 & m4
        {
            let incoming_data: &[u8] = &[
                0x00, 0x02, 0x5d, 0x22, 0x00, 0xd0, 0x01, 0x01, 0xff, 0x06, 0x01, 0x03, 0x03, 0xff,
                0xc9, 0x50, 0x62, 0x39, 0x72, 0xb4, 0x32, 0x9a, 0x6c, 0xbd, 0x1d, 0xc6, 0x52, 0x46,
                0xea, 0xdc, 0x2d, 0xe6, 0xd1, 0xd1, 0xa2, 0x16, 0x3f, 0x24, 0x21, 0xb1, 0x04, 0x40,
                0x08, 0xc2, 0xd7, 0xbf, 0xb2, 0xeb, 0x3e, 0x70, 0x7d, 0x8d, 0x1d, 0xca, 0x28, 0xed,
                0x48, 0xff, 0x79, 0x95, 0xea, 0x3b, 0x7f, 0xc1, 0x31, 0x5f, 0x8b, 0x72, 0xb3, 0xb6,
                0x33, 0xf3, 0x98, 0x80, 0x7f, 0x5c, 0x2c, 0xd6, 0x11, 0x5a, 0xa8, 0xf6, 0x35, 0x10,
                0xf5, 0xd0, 0x2a, 0x21, 0x75, 0xdc, 0x03, 0xd7, 0x5e, 0x9e, 0xbb, 0x82, 0x4e, 0x20,
                0xef, 0x66, 0xed, 0x63, 0x95, 0xe5, 0x49, 0x3e, 0x13, 0x13, 0x32, 0xa3, 0x9e, 0x55,
                0xaf, 0xea, 0xd0, 0xd4, 0x1a, 0x80, 0x18, 0xaa, 0x97, 0x4d, 0xf0, 0x1b, 0x25, 0xaf,
                0x96, 0x47, 0x34, 0x92, 0x9c, 0x5a, 0x42, 0xd4, 0xc7, 0xde, 0x42, 0xae, 0xc6, 0x8c,
                0xaa, 0x94, 0x88, 0xd0, 0xa2, 0x1f, 0xbc, 0xec, 0x90, 0xa5, 0x00, 0x00, 0xe4, 0xa1,
                0x74, 0x83, 0x0e, 0x69, 0x55, 0xf5, 0x3d, 0x2d, 0xc5, 0xf4, 0x11, 0x4e, 0xff, 0xae,
                0x7a, 0x85, 0xa3, 0x8c, 0x44, 0x0e, 0x12, 0x23, 0x87, 0x00, 0xa8, 0x19, 0x2a, 0x82,
                0xc7, 0xf2, 0x6d, 0x8d, 0xb9, 0x90, 0x04, 0xe7, 0xc5, 0xe8, 0x32, 0x4c, 0xcc, 0xad,
                0x7f, 0xbc, 0x5f, 0x9d, 0x96, 0xcd, 0xaa, 0xef, 0x03, 0x6a, 0xf9, 0xf2, 0xd6, 0x63,
                0x2c, 0xe9, 0xb0, 0xb3, 0x20, 0x18, 0xaf, 0x55, 0xd4, 0x50, 0x53, 0x47, 0xfe, 0xdc,
                0x60, 0x4f, 0xdd, 0xf9, 0xf2, 0x78, 0x24, 0xa6, 0xfc, 0x3a, 0x10, 0xcb, 0x97, 0x7c,
                0x5c, 0xbc, 0x4b, 0x9a, 0x83, 0xd2, 0x2b, 0x89, 0x6b, 0xc9, 0x36, 0x0b, 0xe1, 0x37,
                0x7e, 0x84, 0xfd, 0x3d, 0x25, 0x45, 0x45, 0x51, 0xf2, 0x4e, 0xba, 0x79, 0x01, 0xca,
                0xf9, 0x9c, 0x93, 0xa8, 0x69, 0x03, 0x81, 0x33, 0xd5, 0x56, 0x8d, 0xcf, 0x95, 0x10,
                0x7c, 0x20, 0xeb, 0x0c, 0xe8, 0x00, 0x1f, 0x1b, 0x9a, 0xdf, 0x0d, 0x01, 0xea, 0xa7,
                0x6f, 0xa4, 0xe1, 0xa7, 0x30, 0xe5, 0x62, 0x8a, 0x43, 0xe1, 0xa2, 0xf9, 0xb6, 0x13,
                0x3c, 0x69, 0x86, 0x3a, 0x97, 0x60, 0x98, 0xc5, 0xb4, 0x49, 0x56, 0xdc, 0x52, 0x69,
                0xe5, 0x8a, 0xda, 0xa3, 0x7f, 0x7b, 0xd4, 0xe5, 0x5d, 0xcb, 0xa0, 0xf6, 0x83, 0x4f,
                0x46, 0xa1, 0x74, 0xaa, 0xdf, 0xaf, 0xaa, 0x84, 0x39, 0xca, 0x01, 0xd7, 0xc9, 0xe0,
                0x6d, 0x36, 0x9b, 0x2c, 0x82, 0xe9, 0x85, 0x74, 0xef, 0x6c, 0x6c, 0xdf, 0xe7, 0xc4,
                0xcf, 0x71, 0x81, 0xf4, 0x07, 0x72, 0x59, 0x5b, 0xaa, 0xd5, 0x13, 0xeb, 0xda, 0x1c,
                0x33, 0xde, 0x43, 0xda, 0xac, 0xd2, 0xb4, 0x8e, 0x95, 0x79, 0xc6, 0x7f, 0x9e, 0x53,
                0xc9, 0x0c, 0xbe, 0x29, 0x2c, 0x5f, 0x7f, 0x35, 0x4e, 0x1a, 0x04, 0x40, 0x10, 0x6e,
                0x7f, 0x5d, 0x4b, 0xbd, 0x13, 0x0d, 0xf6, 0x52, 0x08, 0xdf, 0x83, 0x0c, 0xea, 0xf5,
                0x05, 0x47, 0x8b, 0x73, 0xc8, 0xdd, 0x04, 0x8f, 0xcf, 0xf7, 0xa3, 0x94, 0xed, 0x26,
                0x5c, 0x5e, 0x47, 0xb6, 0x18, 0xf6, 0xe0, 0xe2, 0x7f, 0xe1, 0xe1, 0x7b, 0xbc, 0x87,
                0x85, 0x81, 0x58, 0x64, 0x45, 0x94, 0x8f, 0xdc, 0xe8, 0x0a, 0xad, 0xeb, 0xa3, 0xf5,
                0x5b, 0x09, 0xf5, 0xbc, 0xb2, 0xa0, 0x09, 0x01, 0x01,
            ];
            let outgoing: &[u8] = &[
                0x02, 0x5d, 0x00, 0x47, 0x00, 0x01, 0x45, 0x06, 0x01, 0x04, 0x04, 0x40, 0xa0, 0xf2,
                0xc0, 0x55, 0x21, 0x57, 0xda, 0x19, 0x85, 0xcd, 0x0c, 0x0f, 0xcb, 0x76, 0x80, 0xa7,
                0xc3, 0x11, 0xf3, 0x2d, 0xe7, 0xb1, 0xed, 0x3b, 0xb9, 0xc8, 0xae, 0xab, 0xee, 0x07,
                0x1a, 0xbc, 0x0c, 0xe5, 0xd8, 0xf2, 0x81, 0x8d, 0x6b, 0x92, 0xe8, 0x78, 0xcb, 0x5b,
                0xb6, 0xe4, 0xea, 0x0b, 0x31, 0xe2, 0xfd, 0xcd, 0x01, 0x2b, 0xaa, 0x73, 0x78, 0x7c,
                0x3f, 0xfe, 0x14, 0x3b, 0xdc, 0x19,
            ];
            ctx.handle_write_incoming_test(&hap, &support, incoming_data, handle_pair_setup)
                .await?;

            let resp = ctx.handle_read_outgoing(handle_pair_setup).await?;
            let resp_buffer = resp.expect("expecting a outgoing response");
            info!("outgoing: {:0>2x?}", &*resp_buffer);
            assert_eq!(&*resp_buffer, outgoing);
        }
        // m5 & m6
        {
            let incoming_data: &[u8] = &[
                0x00, 0x02, 0x06, 0x22, 0x00, 0xa4, 0x00, 0x01, 0x9f, 0x05, 0x9a, 0x46, 0xf1, 0xed,
                0x7b, 0x03, 0x13, 0x81, 0x27, 0xf4, 0x49, 0xf5, 0xd2, 0x58, 0x84, 0x3e, 0xab, 0x62,
                0x16, 0xcc, 0xb2, 0x81, 0x22, 0xcf, 0x0c, 0x24, 0x3e, 0xbb, 0x7c, 0xc1, 0xb0, 0x58,
                0x20, 0xb7, 0x49, 0x29, 0x86, 0xca, 0x7c, 0x5d, 0x22, 0x83, 0x4c, 0xdb, 0x3c, 0xeb,
                0x1f, 0x18, 0xb8, 0xc6, 0x5e, 0xdf, 0xed, 0x23, 0xcd, 0x15, 0xa7, 0x6b, 0x5f, 0xcd,
                0x3b, 0x50, 0xe2, 0x2d, 0x2f, 0x96, 0x34, 0x65, 0x35, 0x7d, 0xda, 0xdc, 0xbc, 0x18,
                0x5a, 0xe4, 0x27, 0x39, 0xfb, 0xe7, 0xf6, 0x41, 0xef, 0xe4, 0xb6, 0x51, 0x23, 0x68,
                0x55, 0x3b, 0xb2, 0x24, 0x3a, 0xed, 0xda, 0x1b, 0x5c, 0x6d, 0x9d, 0xf3, 0xe1, 0xfd,
                0x8b, 0xfd, 0x8a, 0xff, 0xf0, 0xb3, 0x09, 0xe0, 0x42, 0x8d, 0x8a, 0xe4, 0xea, 0x54,
                0xc1, 0x3e, 0xe1, 0x63, 0xec, 0xf8, 0x5a, 0x48, 0x03, 0x3f, 0xef, 0x17, 0xe0, 0x1f,
                0x18, 0x1c, 0x19, 0xb6, 0x0e, 0xad, 0x62, 0x1b, 0x90, 0x4f, 0xbe, 0x6a, 0x5f, 0x0a,
                0xdd, 0x6c, 0xb0, 0x42, 0x5a, 0xf0, 0xa3, 0xf9, 0xec, 0x62, 0xb0, 0x06, 0x01, 0x05,
                0x09, 0x01, 0x01,
            ];
            let outgoing: &[u8] = &[
                0x02, 0x06, 0x00, 0x8e, 0x00, 0x01, 0x8c, 0x06, 0x01, 0x06, 0x05, 0x87, 0xc0, 0x2d,
                0xab, 0x99, 0xf3, 0x9c, 0x2f, 0xeb, 0x40, 0xb4, 0x88, 0xe7, 0x8c, 0x9e, 0x0f, 0x7a,
                0x13, 0xe2, 0x4a, 0x5a, 0xd7, 0x66, 0xc9, 0x2b, 0x45, 0xd6, 0x72, 0x77, 0x81, 0x12,
                0xac, 0x57, 0x15, 0x8d, 0xb1, 0xae, 0x62, 0x7a, 0x69, 0x0c, 0xee, 0xf8, 0xeb, 0x31,
                0x3d, 0x39, 0xbf, 0x6b, 0x1f, 0xc8, 0x16, 0x57, 0x0b, 0x06, 0xf0, 0x56, 0x45, 0x87,
                0xfa, 0x33, 0x92, 0x9a, 0x69, 0x3c, 0xeb, 0x49, 0xc8, 0x9c, 0x06, 0xfb, 0xbe, 0xbf,
                0xec, 0xf7, 0x60, 0xba, 0x0b, 0xeb, 0x4d, 0x8a, 0xbe, 0x62, 0x9a, 0xe7, 0x16, 0xdc,
                0xa1, 0x61, 0x48, 0x3d, 0x2b, 0x78, 0x11, 0x75, 0xef, 0xe2, 0xb0, 0x63, 0xfe, 0x6c,
                0x3a, 0x86, 0x8c, 0x1c, 0x22, 0x9d, 0xcf, 0x3a, 0xb8, 0x61, 0x38, 0x18, 0x33, 0x09,
                0x54, 0xe8, 0xf1, 0xb9, 0x17, 0xa3, 0x81, 0xfe, 0x70, 0x50, 0x06, 0x7a, 0x9f, 0xd1,
                0x29, 0x85, 0x55, 0x77, 0x63, 0x9e, 0x04,
            ];
            ctx.handle_write_incoming_test(&hap, &support, incoming_data, handle_pair_setup)
                .await?;

            let resp = ctx.handle_read_outgoing(handle_pair_setup).await?;
            let resp_buffer = resp.expect("expecting a outgoing response");
            info!("outgoing: {:0>2x?}", &*resp_buffer);
            assert_eq!(&*resp_buffer, outgoing);
        }

        // next up it reads a bunch of service and characteristic instance descriptor values. ALl of them in fact.

        // Writing to pair verify.  ------------
        // Then, we see a write request to pair verify.
        // Update the random for for the m2 cv_SK;

        let counter = core::cell::RefCell::new(0);
        let random_buffer = vec![
            0xe5, 0xb9, 0xee, 0xdd, 0x57, 0xd0, 0x40, 0x13, 0xf3, 0xaa, 0x88, 0xd9, 0x4b, 0x0d,
            0x51, 0xee, 0x92, 0x58, 0x9d, 0xfd, 0xa1, 0x5f, 0x96, 0x65, 0x2f, 0xee, 0x88, 0xd6,
            0x3e, 0x0c, 0x83, 0xc4,
        ];
        let rng = move || {
            let mut c = counter.borrow_mut();
            let v = random_buffer[*c];
            *c += 1;
            v
        };
        let mut support = support;
        support.rng = &rng;

        {
            // Pair verify. m1 & m2
            let incoming_data: &[u8] = &[
                0x00, 0x02, 0x34, 0x23, 0x00, 0x2a, 0x00, 0x01, 0x25, 0x06, 0x01, 0x01, 0x03, 0x20,
                0x1d, 0x74, 0xbd, 0x6a, 0x38, 0xdb, 0xea, 0x23, 0x6c, 0x1a, 0xcb, 0x88, 0x9a, 0xa7,
                0xb9, 0x6d, 0xde, 0x7f, 0x9c, 0xd5, 0x78, 0x34, 0x34, 0x12, 0xed, 0x1f, 0xf0, 0xac,
                0xf1, 0x02, 0x99, 0x01, 0x09, 0x01, 0x01,
            ];
            let outgoing: &[u8] = &[
                0x02, 0x34, 0x00, 0x8e, 0x00, 0x01, 0x8c, 0x06, 0x01, 0x02, 0x03, 0x20, 0x8f, 0x47,
                0x6d, 0xf6, 0x0c, 0xec, 0xdb, 0xe9, 0xc7, 0xf5, 0x4a, 0x6c, 0x2a, 0x6d, 0xbf, 0xd7,
                0x1e, 0xef, 0xd7, 0xf4, 0xf6, 0xf2, 0x73, 0xae, 0xf9, 0x5a, 0x41, 0xfc, 0x93, 0xec,
                0xa2, 0x0e, 0x05, 0x65, 0xbb, 0x5e, 0xc0, 0x3f, 0x8d, 0xb9, 0x8a, 0x1c, 0xd6, 0x47,
                0xc7, 0x83, 0x11, 0xe4, 0x26, 0xff, 0xd1, 0xbe, 0x30, 0xed, 0x7a, 0xef, 0xa0, 0x2c,
                0xed, 0x09, 0xe5, 0x17, 0xf0, 0x81, 0x0f, 0xaa, 0xc5, 0xd4, 0x93, 0x68, 0x5e, 0x32,
                0xc2, 0xf1, 0x48, 0xcb, 0xde, 0x0c, 0x03, 0x82, 0xd2, 0xbf, 0xa4, 0x9a, 0xb4, 0xe7,
                0x91, 0x77, 0x43, 0xa0, 0xd5, 0x72, 0x76, 0x08, 0xa1, 0x0f, 0x73, 0xa0, 0x84, 0x7b,
                0x42, 0xac, 0x79, 0x24, 0xaf, 0x0c, 0xf8, 0x22, 0x0e, 0x53, 0x25, 0x0d, 0xb2, 0xb1,
                0x98, 0xc0, 0x6b, 0xd2, 0xee, 0x1a, 0xaa, 0x9b, 0xb9, 0x7e, 0x6f, 0xab, 0x12, 0xa2,
                0xcf, 0x69, 0x7e, 0x04, 0xb0, 0x61, 0x0a,
            ];
            ctx.handle_write_incoming_test(&hap, &support, incoming_data, handle_pair_verify)
                .await?;

            let resp = ctx.handle_read_outgoing(handle_pair_verify).await?;
            let resp_buffer = resp.expect("expecting a outgoing response");
            info!("outgoing: {:0>2x?}", &*resp_buffer);
            assert_eq!(&*resp_buffer, outgoing);
        }

        {
            // Pair verify. m3?
            let incoming_data: &[u8] = &[
                0x00, 0x02, 0xa9, 0x23, 0x00, 0x82, 0x00, 0x01, 0x7d, 0x05, 0x78, 0x74, 0xda, 0xba,
                0x6e, 0x05, 0x61, 0x68, 0xce, 0x07, 0x34, 0x28, 0xa6, 0xda, 0xb7, 0x4d, 0x78, 0x73,
                0x1a, 0x24, 0x42, 0x74, 0x20, 0x75, 0xdc, 0x90, 0x16, 0xea, 0xc2, 0x03, 0xd7, 0x06,
                0x6e, 0xf7, 0x30, 0x20, 0x21, 0xf6, 0xab, 0x7b, 0xa3, 0xe1, 0xa6, 0xb0, 0x16, 0x62,
                0x4f, 0xf0, 0x22, 0xf2, 0x03, 0xa3, 0x4a, 0x25, 0x1b, 0x78, 0xb0, 0x8b, 0x7c, 0x10,
                0xa8, 0x70, 0xb8, 0xe8, 0xa5, 0xba, 0x7e, 0xab, 0xcd, 0x38, 0x25, 0x78, 0x88, 0xb2,
                0xe3, 0x3e, 0xd4, 0x38, 0xde, 0x06, 0x1f, 0xce, 0x08, 0x5b, 0xb1, 0xf4, 0x0b, 0xef,
                0x8e, 0x00, 0x19, 0xc8, 0x47, 0xd3, 0x73, 0xef, 0xbb, 0xe1, 0x98, 0x34, 0x6e, 0x72,
                0xca, 0x24, 0x70, 0x30, 0xfb, 0x41, 0x59, 0xc0, 0x7a, 0x19, 0xa5, 0xba, 0xca, 0xe3,
                0x43, 0xa5, 0xb9, 0x3d, 0x93, 0x06, 0x01, 0x03, 0x09, 0x01, 0x01,
            ];
            let outgoing: &[u8] = &[0x02, 0xa9, 0x00, 0x05, 0x00, 0x01, 0x03, 0x06, 0x01, 0x04];
            ctx.handle_write_incoming_test(&hap, &support, incoming_data, handle_pair_verify)
                .await?;

            let resp = ctx.handle_read_outgoing(handle_pair_verify).await?;
            let resp_buffer = resp.expect("expecting a outgoing response");
            info!("outgoing: {:0>2x?}", &*resp_buffer);
            assert_eq!(&*resp_buffer, outgoing);
        }

        Ok(())
    }

    #[test]
    fn test_characteristics() {
        crate::test::init();
        let v = CharacteristicProperties::new()
            .with_read(true)
            .with_read_open(true)
            .with_hidden(true);
        assert_eq!(v.0, 0x0001 | 0x0010 | 0x0040);

        let z = CharacteristicProperties::from_bits(0x03);
        info!("0x03: {z:#?}");
        let z = CharacteristicProperties::from_bits(0x10);
        info!("0x10: {z:#?}");
    }
}
