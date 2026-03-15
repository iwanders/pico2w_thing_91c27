use bitfield_struct::bitfield;

// #[cfg(target_arch = "arm")]
// use defmt::{info, println, warn};
use embedded_hal_async::spi::SpiDevice;
use zerocopy::{FromBytes, Immutable, IntoBytes, TryFromBytes};

// See also the notes in the lsm6dsv320x file.

// Okay, so allows external clock... 20 bit fifo, 19 bit gyro & 18 bit accel, some motion processing engine
// and another temperature sensor.
//
// 20 bit data format details are in section 6... current data output is not in 20 bit extensions.
// The 20 bit.... is 20 bit data format in the fifo, but that's NOT how many bits we get data for;
//  > When 20-bits data format is used, gyroscope data consists of 19-bits of actual data and the LSB is always set to 0,
//  > accelerometer data consists of 18-bits of actual data and the two lowest order bits are always set to 0
//  p37, 6.1
//
// device powers up in sleep mode.
//
// Data is big endian!

//
// Fifo has four different package types, see page 37
// Default timestamp resolution is 1us, see page 82
//
//
// > The only register settings that user can modify during sensor operation are for ODR selection, FSR selection, and sensor mode
// > changes (register parameters GYRO_ODR, ACCEL_ODR, GYRO_FS_SEL, ACCEL_FS_SEL, GYRO_MODE, ACCEL_MODE). User must not
// > modify any other register values during sensor operation.
// p60
//

pub mod regs {
    pub const REGISTER_READ: u8 = 1 << 7;
    pub const WHO_AM_I: u8 = 0x75;

    /// Contains SPI settings and reset.
    pub const DEVICE_CONFIG: u8 = 0x11;

    /// First fifo config
    pub const FIFO_CONFIG: u8 = 0x16;

    /// Temperature data registers, first of two.
    pub const TEMP_DATA1: u8 = 0x1D;

    /// Power management, temp sensor, gyro and accel.
    pub const PWR_MGMT0: u8 = 0x4E;
    /// First acceleration data register.
    pub const ACCEL_DATA_X1: u8 = 0x1F;
    /// First gyroscope data register.
    pub const GYRO_DATA_X1: u8 = 0x25;

    /// Data interface config?
    pub const INTF_CONFIG0: u8 = 0x4c;

    /// Gyroscope config
    pub const GYRO_CONFIG0: u8 = 0x4f;
    /// Acceleration config.
    pub const ACCEL_CONFIG0: u8 = 0x50;

    /// Status register in front of fifo.
    pub const INT_STATUS: u8 = 0x2D;
    /// Fifo entry count start
    pub const FIFO_COUNTH: u8 = 0x2E;

    /// The fifo config that determines which values go to the fifo.
    pub const FIFO_CONFIG1: u8 = 0x5F;

    /// The actual fifo data.
    pub const FIFO_DATA: u8 = 0x30;
}

#[derive(Copy, Clone, Debug, Eq, PartialEq, Default, defmt::Format)]
#[repr(u8)]
pub enum GyroscopeOutputDataRate {
    Hz32k = 0b0001,
    Hz16k = 0b0010,
    Hz8k = 0b0011,
    Hz4k = 0b0100,
    Hz2k = 0b0101,
    #[default]
    Hz1k = 0b0110,
    Hz200 = 0b0111,
    Hz50 = 0b1001,
    Hz25 = 0b1010,
    /// 12.5 Hz
    Hz12_5 = 0b1011,
    Hz500 = 0b1111,
}
#[derive(Copy, Clone, Debug, Eq, PartialEq, Default, defmt::Format)]
#[repr(u8)]
pub enum GyroscopeScale {
    #[default]
    Dps2000 = 0b000,
    Dps1000 = 0b001,
    Dps500 = 0b010,
    Dps250 = 0b011,
    Dps125 = 0b100,
    /// +/- 62.5 degrees
    Dps62_5 = 0b101,
    /// +/- 61.25 degrees
    Dps31_25 = 0b110,
    /// +/- 15.625 degrees
    Dps15_625 = 0b111,
}
impl GyroscopeScale {
    pub const fn lsb_per_dps_f32(&self) -> f32 {
        match self {
            GyroscopeScale::Dps2000 => i16::MAX as f32 / 2000.0,
            GyroscopeScale::Dps1000 => i16::MAX as f32 / 1000.0,
            GyroscopeScale::Dps500 => i16::MAX as f32 / 500.0,
            GyroscopeScale::Dps250 => i16::MAX as f32 / 250.0,
            GyroscopeScale::Dps125 => i16::MAX as f32 / 125.0,
            GyroscopeScale::Dps62_5 => i16::MAX as f32 / 62.5,
            GyroscopeScale::Dps31_25 => i16::MAX as f32 / 31.25,
            GyroscopeScale::Dps15_625 => i16::MAX as f32 / 15.625,
        }
    }
}

#[derive(Copy, Clone, Debug, Eq, PartialEq, Default, defmt::Format)]
pub struct GyroscopeConfig {
    pub scale: GyroscopeScale,
    pub rate: GyroscopeOutputDataRate,
}
impl GyroscopeConfig {
    pub fn to_reg(&self) -> u8 {
        self.rate as u8 | (self.scale as u8) << 5
    }
}

#[derive(Copy, Clone, Debug, Eq, PartialEq, Default, defmt::Format)]
#[repr(u8)]
pub enum AccelerationScale {
    #[default]
    G16 = 0b000,
    G8 = 0b001,
    G4 = 0b010,
    G2 = 0b011,
}
impl AccelerationScale {
    pub const fn lsb_per_g_f32(&self) -> f32 {
        match self {
            AccelerationScale::G16 => i16::MAX as f32 / 16.0,
            AccelerationScale::G8 => i16::MAX as f32 / 8.0,
            AccelerationScale::G4 => i16::MAX as f32 / 4.0,
            AccelerationScale::G2 => i16::MAX as f32 / 2.0,
        }
    }
}

#[derive(Copy, Clone, Debug, Eq, PartialEq, Default, defmt::Format)]
#[repr(u8)]
pub enum AccelerationOutputDataRate {
    Hz32k = 0b0001,
    Hz16k = 0b0010,
    Hz8k = 0b0011,
    Hz4k = 0b0100,
    Hz2k = 0b0101,
    #[default]
    Hz1k = 0b0110,
    Hz200 = 0b0111,
    Hz100 = 0b1000,
    Hz50 = 0b1001,
    Hz25 = 0b1010,

    /// 12.5 Hz
    Hz12_5 = 0b1011,
    /// 6.25 Hz
    Hz6_25 = 0b1100,
    /// 3.125 Hz
    Hz3_125 = 0b1101,
    /// 1.5626 Hz
    Hz1_5626 = 0b1110,
    Hz500 = 0b1111,
}

#[derive(Copy, Clone, Debug, Eq, PartialEq, Default, defmt::Format)]
pub struct AccelerationConfig {
    pub scale: AccelerationScale,
    pub rate: AccelerationOutputDataRate,
}
impl AccelerationConfig {
    pub fn to_reg(&self) -> u8 {
        self.rate as u8 | (self.scale as u8) << 5
    }
}
// Temp data in FIFO is; (FIFO_TEMP_DATA / 2.07) + 25
// Temp data direct is ; (TEMP_DATA / 132.48) + 25
//
#[derive(Copy, Clone, Debug, Eq, PartialEq, Default, defmt::Format)]
#[repr(u8)]
pub enum SensorMode {
    // ignoring standby and low power.
    #[default]
    Off = 0b00,
    LowNoise = 0b11,
}

#[derive(Copy, Clone, Debug, Eq, PartialEq, Default, defmt::Format)]
pub struct PowerConfig {
    pub gyroscope: SensorMode,
    pub acceleration: SensorMode,
}
impl PowerConfig {
    fn to_reg(self) -> u8 {
        // temperature sensor defaults to on.
        (self.acceleration as u8) | (self.gyroscope as u8) << 2
    }
}

#[derive(Copy, Clone, Debug, Eq, PartialEq, Default, defmt::Format, FromBytes, IntoBytes)]
pub struct XYZVectorI16 {
    pub x: i16,
    pub y: i16,
    pub z: i16,
}
impl XYZVectorI16 {
    pub fn swap_bytes(&self) -> Self {
        Self {
            x: self.x.swap_bytes(),
            y: self.y.swap_bytes(),
            z: self.z.swap_bytes(),
        }
    }
}

#[derive(Copy, Clone, Debug, Eq, PartialEq, Default, defmt::Format, IntoBytes)]
#[repr(u8)]
pub enum FifoMode {
    #[default]
    Bypass = 0b00,
    StreamToFifo = 0b01,
    StopOnFull = 0b10,
}
impl FifoMode {
    fn to_reg(self) -> u8 {
        (self as u8) << 6
    }
}

#[derive(Copy, Clone, Debug, Eq, PartialEq, Default, defmt::Format)]
pub struct FifoConfig {
    pub resume_partial: bool,
    pub watermark_gt_persist: bool,
    pub high_resolution: bool,
    pub fsync: bool,
    pub batch_temperature: bool,
    pub batch_gyro: bool,
    pub batch_accel: bool,
    pub watermark: u16,
}

impl FifoConfig {
    pub fn to_regs(self) -> [u8; 3] {
        [
            (self.resume_partial as u8) << 6
                | (self.watermark_gt_persist as u8) << 5
                | (self.high_resolution as u8) << 4
                | (self.fsync as u8) << 3
                | (self.batch_temperature as u8) << 2
                | (self.batch_gyro as u8) << 1
                | (self.batch_accel as u8),
            self.watermark as u8,
            (self.watermark >> 8) as u8,
        ]
    }
}

#[bitfield(u8)]
#[derive(PartialEq, Eq, TryFromBytes, IntoBytes, Immutable, defmt::Format)]
pub struct RegIntStatus {
    #[bits(1)] // Bit 0
    pub agc_rdy: bool,

    #[bits(1)] // bit 1
    pub fifo_full: bool,

    #[bits(1)] // bit 2
    pub fifo_threshold: bool,

    #[bits(1)] // bit 3
    pub data_ready: bool,

    #[bits(1)] // bit 4
    pub reset_done: bool,

    #[bits(1)] // bit 5
    pub pll_ready: bool,

    #[bits(1)] // bit 6
    pub ui_fsync: bool,

    #[bits(1)] // bit 7
    _pad: bool,
}
use bitfield_struct::bitenum;

#[bitenum]
#[repr(u8)]
#[derive(PartialEq, Eq, defmt::Format, Debug)]
pub enum FifoHeaderTimestamp {
    /// Packet does not contain timestamp
    #[fallback]
    None = 0b00,
    /// Packet contains ODR timestamp.
    ODRTimestamp = 0b10,
    /// Packet contains FSYNC time and packet is first ODR after FSYNC.
    FsyncTime = 0b11,
}
/// Fifo Header field, p40, 6.2
#[bitfield(u8)]
#[derive(PartialEq, Eq, FromBytes, IntoBytes, Immutable, defmt::Format)]
pub struct FifoHeader {
    /// if ODR of gyro changes.
    #[bits(1)] // bit 0
    pub odr_gyro: bool,

    /// if ODR of accel changes.
    #[bits(1)] // bit 1
    pub odr_accel: bool,

    /// Timestamp field.
    #[bits(2)] // Bit 2-3
    pub timestamp: FifoHeaderTimestamp,

    /// Packet has 20 bit data for gyro and accel.
    #[bits(1)] // bit 4
    pub data_20bit: bool,

    /// Packet thas gyroscope data
    #[bits(1)] // bit 5
    pub gyro: bool,

    /// Packet has acceleration data
    #[bits(1)] // bit 6
    pub accel: bool,

    /// Fifo is empty.
    #[bits(1)] // bit 7
    pub is_empty: bool,
}
impl FifoHeader {
    pub fn data(&self) -> bool {
        !self.is_empty()
    }
    /// Packet length, including the header!
    pub fn packet_len(&self) -> usize {
        // Length options are;
        // packet 1: 8
        // packet 2: 8
        // packet 3: 16
        // packet 4: 20
        let mut len = 0;
        if self.data() {
            len += 1; // header
            len += self.accel() as usize * 6;
            len += self.gyro() as usize * 6;
            len += 1; // temperature
            len += if self.timestamp() != FifoHeaderTimestamp::None {
                2
            } else {
                0
            }; // timestamp.
            len += self.data_20bit() as usize * (3 + 1); // temperature also goes from 1 to two bytes.
        }
        len
    }
}

/// Error used by the accessory interface
#[derive(thiserror::Error, Debug, Copy, Clone, PartialEq, Eq, defmt::Format)]
pub enum IcmFifoError {
    /// Not enough bytes provided
    #[error("not enough data")]
    NotEnoughData,
}

// Probably make an iterator that returns (DataTag, &[u8])
pub struct IcmFifoIterator<'d> {
    data: &'d [u8],
    position: usize,
}
impl<'d> IcmFifoIterator<'d> {
    pub fn new(data: &'d [u8]) -> Self {
        Self { data, position: 0 }
    }
}
impl<'d> Iterator for IcmFifoIterator<'d> {
    type Item = Result<(FifoHeader, &'d [u8]), IcmFifoError>;

    fn next(&mut self) -> Option<Self::Item> {
        if self.position == self.data.len() {
            // Good end, it ends on complete data.
            return None;
        }
        let header = FifoHeader::from_bits(self.data[self.position]);
        let packet_len = header.packet_len();
        if (self.position + packet_len) > self.data.len() {
            Some(Err(IcmFifoError::NotEnoughData))
        } else {
            // The actual data after the header.
            let packet_data = &self.data[self.position + 1..(self.position + 1 + packet_len - 1)];
            self.position += packet_len;
            Some(Ok((header, packet_data)))
        }
    }
}

#[derive(Debug, Copy, Clone)]
pub enum FifoTemperature {
    OneByte(i8),
    TwoByte(i16),
}
impl Default for FifoTemperature {
    fn default() -> Self {
        FifoTemperature::OneByte(0)
    }
}
impl FifoTemperature {
    pub fn to_c_f32(self) -> f32 {
        match self {
            // 16 bit:
            // Temperature in Degrees Centigrade = (TEMP_DATA / 132.48) + 25
            // 8 bit:
            // Temperature in Degrees Centigrade = (FIFO_TEMP_DATA / 2.07) + 25
            // p67
            FifoTemperature::OneByte(t) => (t as f32 / 2.07) + 25.0,
            FifoTemperature::TwoByte(t) => (t as f32 / 132.48) + 25.0,
        }
    }
}

#[derive(Debug, Copy, Clone)]
pub struct AccelerationExtraBits(u16);
impl AccelerationExtraBits {
    pub fn from_extra(data: &[u8]) -> Self {
        let x = data[0] >> 4;
        let y = data[1] >> 4;
        let z = data[2] >> 4;

        let v = (x as u16) << 8 | (y as u16) << 4 | (z as u16);

        Self(v)
    }
    pub fn x(&self) -> u8 {
        (self.0 >> 8) as u8
    }
    pub fn y(&self) -> u8 {
        ((self.0 >> 4) as u8) & 0b1111
    }
    pub fn z(&self) -> u8 {
        (self.0 as u8) & 0b1111
    }
    pub fn xyz(&self) -> (u8, u8, u8) {
        (self.x(), self.y(), self.z())
    }
}

#[derive(Debug, Copy, Clone)]
pub struct GyroscopeExtraBits(u16);
impl GyroscopeExtraBits {
    pub fn from_extra(data: &[u8]) -> Self {
        let x = data[0] & 0b1111;
        let y = data[1] & 0b1111;
        let z = data[2] & 0b1111;

        let v = (x as u16) << 8 | (y as u16) << 4 | (z as u16);

        Self(v)
    }
    pub fn x(&self) -> u8 {
        (self.0 >> 8) as u8
    }
    pub fn y(&self) -> u8 {
        ((self.0 >> 4) as u8) & 0b1111
    }
    pub fn z(&self) -> u8 {
        (self.0 as u8) & 0b1111
    }
    pub fn xyz(&self) -> (u8, u8, u8) {
        (self.x(), self.y(), self.z())
    }
}

#[derive(Debug, Default, Copy, Clone)]
pub struct FifoAccelerometer {
    pub scale: AccelerationScale,
    pub x: i16,
    pub y: i16,
    pub z: i16,
    pub extra_bits: Option<AccelerationExtraBits>,
}
impl FifoAccelerometer {
    fn i16_to_f32_extra(&self, v: i16, extra: u8) -> f32 {
        let v = v as i32;
        let v = v << 4;
        let v = (v | (extra as i32)) >> 2;
        // > accelerometer data consists of 18-bits of actual data and the two lowest order bits are always set to 0
        // p37, section 6.1
        // When extra, scale is always +/- 16G, p37, so 8192 LSB per g.
        v as f32 / 8192.0
    }

    pub fn xyz_f32(&self) -> (f32, f32, f32) {
        if let Some(extra) = self.extra_bits {
            let extra = extra.xyz();
            (
                self.i16_to_f32_extra(self.x, extra.0),
                self.i16_to_f32_extra(self.y, extra.1),
                self.i16_to_f32_extra(self.z, extra.2),
            )
        } else {
            (
                self.x as f32 / self.scale.lsb_per_g_f32(),
                self.y as f32 / self.scale.lsb_per_g_f32(),
                self.z as f32 / self.scale.lsb_per_g_f32(),
            )
        }
    }
}

#[derive(Debug, Default, Copy, Clone)]
pub struct FifoGyroscope {
    pub scale: GyroscopeScale,
    pub x: i16,
    pub y: i16,
    pub z: i16,
    pub extra_bits: Option<GyroscopeExtraBits>,
}
impl FifoGyroscope {
    fn i16_to_f32_extra(&self, v: i16, extra: u8) -> f32 {
        let v = v as i32;
        let v = v << 4;
        let v = (v | (extra as i32)) >> 1;
        // > gyroscope data consists of 19-bits of actual data and the LSB is always set to 0
        // p37, section 6.1
        // When extra, scale is always +/- 16G, p37, so 8192 LSB per g.
        v as f32 / 131.0
    }

    pub fn xyz_f32(&self) -> (f32, f32, f32) {
        if let Some(extra) = self.extra_bits {
            let extra = extra.xyz();
            (
                self.i16_to_f32_extra(self.x, extra.0),
                self.i16_to_f32_extra(self.y, extra.1),
                self.i16_to_f32_extra(self.z, extra.2),
            )
        } else {
            (
                self.x as f32 / self.scale.lsb_per_dps_f32(),
                self.y as f32 / self.scale.lsb_per_dps_f32(),
                self.z as f32 / self.scale.lsb_per_dps_f32(),
            )
        }
    }
}

#[derive(Debug, Default, Copy, Clone)]
pub struct FifoTimestamp(pub u16);

#[derive(Debug, Default, Copy, Clone)]
pub struct TimeTracker {
    pub rollovers: u64,
    pub previous: FifoTimestamp,
}
impl TimeTracker {
    pub fn new() -> Self {
        TimeTracker {
            rollovers: 0,
            previous: Default::default(),
        }
    }
    pub fn update(&mut self, timestamp: FifoTimestamp) {
        if timestamp.0 < self.previous.0 {
            self.rollovers += 1;
        }
        self.previous = timestamp;
    }
    pub fn time_us(&self) -> u64 {
        // FIFO TIMESTAMP INTERVAL SCALING  p59
        ((self.rollovers * 0x1_00_00 + self.previous.0 as u64) * 32) / 30
    }
    pub fn time_ns(&self) -> u64 {
        self.time_us() * 1000
    }
}

#[derive(Debug, Default, Copy, Clone)]
pub struct FifoEntry {
    pub header: FifoHeader,
    pub acceleration: Option<FifoAccelerometer>,
    pub gyroscope: Option<FifoGyroscope>,
    pub temperature: FifoTemperature,
    pub timestamp: FifoTimestamp,
}

/// This is a helper to parse data from the fifo.
#[derive(Debug, Default, Copy, Clone)]
pub struct IcmFifoProcessor {
    pub gyro_scale: GyroscopeScale,
    pub accel_scale: AccelerationScale,
}
impl IcmFifoProcessor {
    pub fn interpret(&self, header: FifoHeader, data: &[u8]) -> FifoEntry {
        let mut r = FifoEntry::default();
        if data.len() != header.packet_len() - 1 {
            panic!();
        }
        r.header = header;
        if header.is_empty() {
            return r;
        }
        let mut p = 0usize;
        let start_20bit = (header.accel() as usize * 6) + (header.gyro() as usize * 6) + 2 + 2;
        // Accel is always first.
        if header.accel() {
            r.acceleration = Some(FifoAccelerometer {
                scale: self.accel_scale,
                x: i16::from_be_bytes(data[p..p + 2].try_into().unwrap()),
                y: i16::from_be_bytes(data[p + 2..p + 4].try_into().unwrap()),
                z: i16::from_be_bytes(data[p + 4..p + 6].try_into().unwrap()),

                extra_bits: if header.data_20bit() {
                    let v = AccelerationExtraBits::from_extra(&data[start_20bit..start_20bit + 3]);
                    Some(v)
                } else {
                    None
                },
            });
            p += 6;
        }
        if header.gyro() {
            r.gyroscope = Some(FifoGyroscope {
                scale: self.gyro_scale,
                x: i16::from_be_bytes(data[p..p + 2].try_into().unwrap()),
                y: i16::from_be_bytes(data[p + 2..p + 4].try_into().unwrap()),
                z: i16::from_be_bytes(data[p + 4..p + 6].try_into().unwrap()),
                extra_bits: if header.data_20bit() {
                    let v = GyroscopeExtraBits::from_extra(&data[start_20bit..start_20bit + 3]);
                    Some(v)
                } else {
                    None
                },
            });
            p += 6;
        }
        // Next is temperature.
        if header.data_20bit() {
            // two byte
            r.temperature =
                FifoTemperature::TwoByte(i16::from_be_bytes(data[p..p + 2].try_into().unwrap()));
            p += 2;
        } else {
            // one byte.
            r.temperature = FifoTemperature::OneByte(data[p] as i8);
            p += 1;
        }
        if header.timestamp() != FifoHeaderTimestamp::None {
            r.timestamp = FifoTimestamp(u16::from_be_bytes(data[p..p + 2].try_into().unwrap()))
        }

        r
    }
}

#[derive(Debug, Copy, Clone, PartialEq, defmt::Format)]
pub enum Error<SpiError: embedded_hal_async::spi::Error> {
    /// Underlying SpiError device error
    Spi(SpiError),
    /// The response on the who am i register during initialisation was incorrect.
    UnexpectedWhoAmI,
}

impl<SpiError: embedded_hal_async::spi::Error> From<SpiError> for Error<SpiError> {
    fn from(e: SpiError) -> Self {
        Error::<SpiError>::Spi(e)
    }
}
#[derive(Clone, PartialEq)]
pub struct ICM42688<Spi> {
    spi: Spi,
}
impl<Spi> core::fmt::Debug for ICM42688<Spi> {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        f.write_fmt(format_args!("<ICM42688>"))
    }
}
impl<Spi> defmt::Format for ICM42688<Spi> {
    fn format(&self, f: defmt::Formatter) {
        defmt::write!(f, "<ICM42688>")
    }
}

#[derive(Copy, Clone, Debug, Eq, PartialEq, Default, defmt::Format)]
#[repr(u8)]
pub enum AccelerationMode {
    #[default]
    HighPerformance = 0b000,
    HighAccuracy = 0b001,
    ODRTrigger = 0b011,
    LowPowerMean2 = 0b100,
    LowPowerMean4 = 0b101,
    LowPowerMean8 = 0b110,
    Normal = 0b111,
}

#[derive(Copy, Clone, Debug, Eq, PartialEq, Default, defmt::Format, FromBytes, IntoBytes)]
pub struct MilliCelsius(pub i32);

impl<Spi: embedded_hal_async::spi::SpiDevice> ICM42688<Spi>
where
    Spi: SpiDevice<u8>,
    Spi::Error: embedded_hal_async::spi::Error,
{
    async fn write(&mut self, register: u8, data: &[u8]) -> Result<(), Error<Spi::Error>> {
        use embedded_hal_async::spi::Operation;

        self.spi
            .transaction(&mut [Operation::Write(&[register]), Operation::Write(data)])
            .await?;
        Ok(())
    }
    async fn read(&mut self, register: u8, data: &mut [u8]) -> Result<(), Error<Spi::Error>> {
        use embedded_hal_async::spi::Operation;
        self.spi
            .transaction(&mut [
                Operation::Write(&[register | regs::REGISTER_READ]),
                Operation::Read(data),
            ])
            .await?;

        Ok(())
    }

    pub async fn new(spi: Spi) -> Result<Self, Error<Spi::Error>> {
        // Verify we can read the whoami register.
        use regs::WHO_AM_I;
        let mut z = Self { spi };
        let mut read = [0u8; 1];
        z.read(WHO_AM_I, &mut read).await?;

        if read[0] == 0x47 {
            Ok(z)
        } else {
            Err(Error::UnexpectedWhoAmI)
        }
    }

    /// Reset the device, wait at least 1ms after before read/writing registers.
    pub async fn reset(&mut self) -> Result<(), Error<Spi::Error>> {
        const SOFT_RESET_CONFIG: u8 = 1;
        self.write(regs::DEVICE_CONFIG, &[SOFT_RESET_CONFIG]).await
    }

    /// Get temperature in millicelsius.
    pub async fn read_temperature(&mut self) -> Result<MilliCelsius, Error<Spi::Error>> {
        let mut output = 0i16;
        self.read(regs::TEMP_DATA1, output.as_mut_bytes()).await?;
        let raw = output.swap_bytes();
        // (TEMP_DATA / 132.48) + 25
        // Calculate scaling factor for millicelsius, and then scale by 1000.0 get get three points of resolution.
        const SCALAR: i32 = ((1.0 / 132.48) * 1000.0 * 1000.0) as i32;
        let milli_celsius = (raw as i32 * SCALAR) / 1000 + 25000;
        Ok(MilliCelsius(milli_celsius))
    }

    /// Control the power register.
    pub async fn control_power(&mut self, config: PowerConfig) -> Result<(), Error<Spi::Error>> {
        self.write(regs::PWR_MGMT0, &[config.to_reg()]).await
    }

    pub async fn read_acceleration(&mut self) -> Result<XYZVectorI16, Error<Spi::Error>> {
        let mut output = XYZVectorI16::default();
        let buff = output.as_mut_bytes();
        self.read(regs::ACCEL_DATA_X1, buff).await?;
        Ok(output.swap_bytes())
    }

    pub async fn read_gyroscope(&mut self) -> Result<XYZVectorI16, Error<Spi::Error>> {
        let mut output = XYZVectorI16::default();
        let buff = output.as_mut_bytes();
        self.read(regs::GYRO_DATA_X1, buff).await?;
        Ok(output.swap_bytes())
    }

    pub async fn control_fifo_mode(&mut self, config: FifoMode) -> Result<(), Error<Spi::Error>> {
        self.write(regs::FIFO_CONFIG, &[config.to_reg()]).await
    }

    /// Sets the interface config register to report the fifo in bytes instead of packets.
    pub async fn interface_config_fifo_count_bytes(&mut self) -> Result<(), Error<Spi::Error>> {
        self.write(regs::INTF_CONFIG0, &[0b0011_0011]).await
    }

    pub async fn control_fifo_config(
        &mut self,
        config: FifoConfig,
    ) -> Result<(), Error<Spi::Error>> {
        self.write(regs::FIFO_CONFIG1, &config.to_regs()).await
    }

    // Should probably combine the bottom one with the register before it that is INT_STATUS.
    pub async fn read_fifo_count(&mut self) -> Result<u16, Error<Spi::Error>> {
        let mut output = u16::default();
        self.read(regs::FIFO_COUNTH, output.as_mut_bytes()).await?;
        Ok(output.swap_bytes())
    }

    pub async fn read_status(&mut self) -> Result<RegIntStatus, Error<Spi::Error>> {
        let mut raw_reg = 0u8;
        self.read(regs::INT_STATUS, raw_reg.as_mut_bytes()).await?;
        Ok(RegIntStatus::from_bits(raw_reg))
    }

    /// Read bytes from the fifo.
    pub async fn get_fifo(&mut self, values: &mut [u8]) -> Result<(), Error<Spi::Error>> {
        self.read(regs::FIFO_DATA, values).await
    }

    pub async fn control_gyro(&mut self, config: GyroscopeConfig) -> Result<(), Error<Spi::Error>> {
        self.write(regs::GYRO_CONFIG0, &[config.to_reg()]).await
    }

    pub async fn get_status_fifo(
        &mut self,
        values: &mut [u8],
    ) -> Result<(RegIntStatus, usize), Error<Spi::Error>> {
        let mut raw_data = [0u8; 3];
        self.read(regs::INT_STATUS, raw_data.as_mut_bytes()).await?;
        let status = RegIntStatus::from_bits(raw_data[0]);
        let count = u16::from_be_bytes([raw_data[1], raw_data[2]]);
        let up_to = values.len().min(count as usize);
        self.read(regs::FIFO_DATA, &mut values[0..up_to]).await?;
        Ok((status, count as usize))
    }

    pub async fn control_accel(
        &mut self,
        config: AccelerationConfig,
    ) -> Result<(), Error<Spi::Error>> {
        self.write(regs::ACCEL_CONFIG0, &[config.to_reg()]).await
    }
}

#[cfg(test)]
mod test {
    use super::*;

    #[test]
    fn test_icm_fifo_header() {
        // const DATA_PRESENT: u8 = 1 << 7;

        let row1_header = 0b0111_1011;
        let header = FifoHeader::from_bits(row1_header);
        assert_eq!(header.packet_len(), 20);
        assert_eq!(header.accel(), true);
        assert_eq!(header.gyro(), true);
        assert_eq!(header.data_20bit(), true);
        assert_eq!(header.timestamp(), FifoHeaderTimestamp::ODRTimestamp);

        let row3_header = 0b0110_1011;
        let header = FifoHeader::from_bits(row3_header);
        assert_eq!(header.packet_len(), 16);
        assert_eq!(header.accel(), true);
        assert_eq!(header.gyro(), true);
        assert_eq!(header.data_20bit(), false);
        assert_eq!(header.timestamp(), FifoHeaderTimestamp::ODRTimestamp);
        let row5_header = 0b0100_0011;
        let header = FifoHeader::from_bits(row5_header);
        assert_eq!(header.packet_len(), 8);
        assert_eq!(header.accel(), true);
        assert_eq!(header.gyro(), false);
        assert_eq!(header.data_20bit(), false);
        assert_eq!(header.timestamp(), FifoHeaderTimestamp::None);
    }
}
