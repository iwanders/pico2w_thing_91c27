use embedded_hal_async::spi::SpiDevice;
// https://docs.rs/embedded-hal/1.0.0/embedded_hal/spi/index.html#for-driver-authors
// > If your device has a CS pin, use SpiDevice. Do not manually manage the CS pin, the SpiDevice implementation will
// > do it for you. By using SpiDevice, your driver will cooperate nicely with other drivers for other devices in the same shared SPI bus.
// That's us!

// https://github.com/rust-embedded/embedded-hal/issues/572
// https://docs.embassy.dev/embassy-embedded-hal/git/default/shared_bus/asynch/spi/index.html

// all pages refer to the pdf marked DS14623 - Rev 2
// strike that, we also need an6119.
//
// Low G, high G and gyro can be toggled independently and run at different data rates.
// High G is only available if low G is in high performance or high accuracy mode.

// Tsu(CS) is 20ns, so we need a 20ns delay at the start of an SPI transaction.
// https://docs.rs/embedded-hal/1.0.0/embedded_hal/spi/index.html#cs-to-clock-delays
// Drivers should NOT use Operation::DelayNs for this... well then.
// I should patch up https://github.com/embassy-rs/embassy/blob/77a8bc27e9c34e363f321132ebb9e8d8ff684a9f/embassy-embedded-hal/src/shared_bus/asynch/spi.rs#L1-L212
// Seems we can ignore it for now.

use zerocopy::{FromBytes, IntoBytes};

// Should we use bitfield-struct?
// THis seems to work:
// #[bitfield(u64)]
// #[derive(PartialEq, Eq, TryFromBytes, IntoBytes, Immutable, defmt::Format)] // <- Attributes after `bitfield` are carried over
// struct MyBitfield {

pub mod regs {

    pub const REGISTER_READ: u8 = 1 << 7;
    pub const WHO_AM_I: u8 = 0x0F;

    /// Control register with software reset and spi register advance.
    pub const CTRL3: u8 = 0x12;

    /// Accelerometer control reg 1, mode and data rate.
    pub const CTRL1: u8 = 0x10;

    /// Gyroscope control register 2, mode and rate.
    pub const CTRL2: u8 = 0x11;

    /// Control register 6, Gyroscope Scale and bandwidths.
    pub const CTRL6: u8 = 0x15;

    /// Acceleration control 8, filter & scale selection.
    pub const CTRL8: u8 = 0x17;

    /// Acceleration Linear X low, two bytes, two's complement. Followed by y and z.
    pub const OUTX_L_A: u8 = 0x28;

    /// Temperature data output register low, two bytes, two's complement, followed by H.
    pub const OUT_TEMP_L: u8 = 0x20;

    /// Timestamp data in, 4 bytes, u32, 1LSB is 21.7 us typical.
    pub const TIMESTAMP0: u8 = 0x40;

    /// Control register for high g accelerometer.
    pub const CTRL1_XL_HG: u8 = 0x4e;

    /// Data to the accelerometer full-scale and ODR settings, or according to the high-g mode configuration.
    pub const UI_OUTX_L_A_OIS_HG: u8 = 0x34;

    /// Angular rate sensor pitch axis x output.
    pub const OUTX_L_G: u8 = 0x22;

    /// First fifo status register.
    pub const FIFO_STATUS1: u8 = 0x1b;

    /// Controls the batch rate for gyroscope and accelerometer data.
    pub const FIFO_CTRL3: u8 = 0x09;
    /// Fifo control register 4.
    pub const FIFO_CTRL4: u8 = 0x0a;

    /// Fifo tag register.
    pub const FIFO_DATA_OUT_TAG: u8 = 0x78;
    /// First fifo data register.
    pub const FIFO_DATA_OUT_X_L: u8 = 0x79;

    /// Counter batch data rate
    pub const COUNTER_BDR_REG1: u8 = 0x0b;
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
pub struct LSM6DSV320X<Spi> {
    spi: Spi,
}
impl<Spi> core::fmt::Debug for LSM6DSV320X<Spi> {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        f.write_fmt(format_args!("<LSM6DSV320X>"))
    }
}
impl<Spi> defmt::Format for LSM6DSV320X<Spi> {
    fn format(&self, f: defmt::Formatter) {
        defmt::write!(f, "<LSM6DSV320X>")
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

#[derive(Copy, Clone, Debug, Eq, PartialEq, Default, defmt::Format)]
#[repr(u8)]
pub enum OutputDataRate {
    #[default]
    PowerDown = 0b0000,
    /// Low Power.
    Hz1Dot875 = 0b0001,
    /// High Performance, Normal.
    Hz7Dot5 = 0b0010,
    /// Low Power, High Performance, Normal.
    Hz15 = 0b0011,
    /// Low Power, High Performance, Normal.
    Hz30 = 0b0100,
    /// Low Power, High Performance, Normal.
    Hz60 = 0b0101,
    /// Low Power, High Performance, Normal.
    Hz120 = 0b0110,
    /// Low Power, High Performance, Normal.
    Hz240 = 0b0111,
    /// High Performance, Normal.
    Hz480 = 0b1000,
    /// High Performance, Normal.
    Hz960 = 0b1001,
    /// High Performance, Normal.
    Hz1920 = 0b1010,
    /// High Performance only.
    Hz3840 = 0b1011,
    /// High Performance only.
    Hz7680 = 0b1100,
}
pub struct AccelerationModeDataRate {
    pub mode: AccelerationMode,
    pub rate: OutputDataRate,
}

#[derive(Copy, Clone, Debug, Eq, PartialEq, Default, defmt::Format)]
#[repr(u8)]
pub enum AccelerationScale {
    #[default]
    G2 = 0b00,
    G4 = 0b01,
    G8 = 0b10,
    G16 = 0b11,
}
pub struct AccelerationFilterScale {
    pub scale: AccelerationScale,
    // Skip filter stuff for now, it is spread out over ctrl8 and ctrl0.
}

#[derive(Copy, Clone, Debug, Eq, PartialEq, Default, defmt::Format, FromBytes, IntoBytes)]
pub struct XYZVectorI16 {
    pub x: i16,
    pub y: i16,
    pub z: i16,
}

#[derive(Copy, Clone, Debug, Eq, PartialEq, Default, defmt::Format)]
#[repr(u8)]
pub enum AccelerationScaleHigh {
    #[default]
    G32 = 0b000,
    G64 = 0b001,
    G128 = 0b010,
    G256 = 0b011,
    G320 = 0b100,
}

#[derive(Copy, Clone, Debug, Eq, PartialEq, Default, defmt::Format)]
#[repr(u8)]
pub enum AccelerationDataRateHigh {
    #[default]
    PowerDown = 0b000,
    Hz480 = 0b011,
    Hz960 = 0b100,
    Hz1920 = 0b101,
    Hz3840 = 0b110,
    Hz7680 = 0b111,
}
/// Enables or disables readout of the high-g acceleration from the UI_* output registers, set to enabled for `read_acceleration_high`.
///
/// If disabled, that register can be used for the low G OIS/EIS things.
#[derive(Copy, Clone, Debug, Eq, PartialEq, Default, defmt::Format)]
#[repr(u8)]
pub enum AccelerationHighOut {
    #[default]
    Enabled = 1,
    Disabled = 0,
}
#[derive(Copy, Clone, Debug, Eq, PartialEq, Default, defmt::Format)]
pub struct AccelerationModeDataRateHigh {
    pub regout: AccelerationHighOut,
    pub scale: AccelerationScaleHigh,
    pub rate: AccelerationDataRateHigh,
}
impl AccelerationModeDataRateHigh {
    fn to_reg(&self) -> u8 {
        (self.regout as u8) << 7 | (self.rate as u8) << 3 | (self.scale as u8)
    }
}

#[derive(Copy, Clone, Debug, Eq, PartialEq, Default, defmt::Format)]
#[repr(u8)]
pub enum GyroscopeScale {
    #[default]
    DPS250 = 0b001,
    DPS500 = 0b010,
    DPS1000 = 0b011,
    DPS2000 = 0b100,
    DPS4000 = 0b101,
}

#[derive(Copy, Clone, Debug, Eq, PartialEq, Default, defmt::Format)]
pub struct GyroscopeBandwidthScale {
    pub scale: GyroscopeScale,
}
impl GyroscopeBandwidthScale {
    fn to_reg(&self) -> u8 {
        self.scale as u8
    }
}

#[derive(Copy, Clone, Debug, Eq, PartialEq, Default, defmt::Format)]
#[repr(u8)]
pub enum GyroscopeMode {
    #[default]
    HighPerformance = 0b000,
    HighAccuracy = 0b001,
    ODRTrigger = 0b011,
    LowPowerMean2 = 0b100,
    LowPowerMean4 = 0b101,
    LowPowerMean8 = 0b110,
    Normal = 0b111,
}

#[derive(Copy, Clone, Debug, Eq, PartialEq, Default, defmt::Format)]
pub struct GyroscopeModeDataRate {
    pub mode: GyroscopeMode,
    pub rate: OutputDataRate,
}
impl GyroscopeModeDataRate {
    fn to_reg(&self) -> u8 {
        (self.mode as u8) << 4 | self.rate as u8
    }
}

#[derive(Copy, Clone, Debug, Eq, PartialEq, Default, defmt::Format)]
#[repr(u8)]
pub enum TemperatureBatch {
    #[default]
    Disabled = 0b00,
    Hz1dot876 = 0b01,
    Hz15 = 0b10,
    Hz60 = 0b11,
}
#[derive(Copy, Clone, Debug, Eq, PartialEq, Default, defmt::Format)]
#[repr(u8)]
pub enum FifoMode {
    #[default]
    Bypass = 0b00,
    /// Start recording batches and put them into the fifo, stop when it is full. To start gain enter Bypass mode before
    /// reenabling this mode.
    FifoModeStopWhenFull = 0b001,
    ContinuousWTMToFull = 0b010,
    /// Starts operating in continuous mode, then switches to fifo in the event condition.
    ContinuousToFifo = 0b011,
    BypassToContinuous = 0b100,
    Continuous = 0b110,
    BypassToFifo = 0b111,
}

#[derive(Copy, Clone, Debug, Eq, PartialEq, Default, defmt::Format)]
pub struct FifoControl {
    pub temperature: TemperatureBatch,
    pub mode: FifoMode,
}
impl FifoControl {
    fn to_reg(&self) -> u8 {
        (self.temperature as u8) << 4 | self.mode as u8
    }
}
#[derive(Copy, Clone, Debug, Eq, PartialEq, Default, defmt::Format)]
pub struct FifoBatch {
    pub gyroscope: OutputDataRate,
    pub acceleration: OutputDataRate,
}
impl FifoBatch {
    fn to_reg(&self) -> u8 {
        (self.gyroscope as u8) << 4 | self.acceleration as u8
    }
}

#[derive(Copy, Clone, Eq, PartialEq, Default, FromBytes, IntoBytes)]
pub struct FifoStatus {
    status1: u8,
    status2: u8,
}

impl FifoStatus {
    pub fn unread(&self) -> u16 {
        (self.status2 as u16 & 0b1) << 8 | self.status1 as u16
    }
    pub fn overrun(&self) -> bool {
        (self.status2 & (1 << 6)) != 0
    }
    pub fn overrun_latched(&self) -> bool {
        (self.status2 & (1 << 3)) != 0
    }
    pub fn watermark(&self) -> bool {
        (self.status2 & (1 << 7)) != 0
    }
    pub fn full(&self) -> bool {
        (self.status2 & (1 << 5)) != 0
    }
}
impl core::fmt::Debug for FifoStatus {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        f.debug_struct("FifoStatus")
            .field("unread", &self.unread())
            .field("overrun", &self.overrun())
            .field("overrun_latched", &self.overrun_latched())
            .field("watermark", &self.watermark())
            .field("full", &self.full())
            .finish()
    }
}
impl defmt::Format for FifoStatus {
    fn format(&self, f: defmt::Formatter) {
        defmt::write!(
            f,
            "FifoStatus{{ unread: {}, overrun: {}, overrun_latched: {}, watermark: {}, full: {} }}",
            self.unread(),
            self.overrun(),
            self.overrun_latched(),
            self.watermark(),
            self.full()
        )
    }
}

#[derive(Copy, Clone, Debug, Eq, PartialEq, Default, defmt::Format, FromBytes, IntoBytes)]
pub struct MilliCelsius(pub i32);

impl<Spi: embedded_hal_async::spi::SpiDevice> LSM6DSV320X<Spi>
where
    Spi: SpiDevice<u8>,
    Spi::Error: embedded_hal_async::spi::Error,
{
    async fn write(&mut self, register: u8, data: &[u8]) -> Result<(), Error<Spi::Error>> {
        use embedded_hal_async::spi::Operation;

        self.spi
            .transaction(&mut [Operation::Write(&[register]), Operation::Write(&data)])
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

        if read[0] == 0x73 {
            Ok(z)
        } else {
            Err(Error::UnexpectedWhoAmI)
        }
    }

    pub async fn reset(&mut self) -> Result<(), Error<Spi::Error>> {
        // Write to contr CTRL3, p67
        const BOOT: u8 = 1 << 7;
        const BDU: u8 = 1 << 6;
        const IF_INC: u8 = 1 << 2;
        const SW_RESET: u8 = 1;

        self.write(regs::CTRL3, &[BOOT | BDU | IF_INC | SW_RESET])
            .await
    }

    pub async fn control_acceleration(
        &mut self,
        config: AccelerationModeDataRate,
    ) -> Result<(), Error<Spi::Error>> {
        let reg_value = (config.mode as u8) << 4 | config.rate as u8;
        self.write(regs::CTRL1, &[reg_value]).await
    }

    pub async fn filter_acceleration(
        &mut self,
        config: AccelerationFilterScale,
    ) -> Result<(), Error<Spi::Error>> {
        let reg_value = config.scale as u8;
        self.write(regs::CTRL8, &[reg_value]).await
    }

    pub async fn read_acceleration(&mut self) -> Result<XYZVectorI16, Error<Spi::Error>> {
        let mut output = XYZVectorI16::default();
        let buff = output.as_mut_bytes();
        self.read(regs::OUTX_L_A, buff).await?;
        Ok(output)
    }

    pub async fn read_acceleration_high(&mut self) -> Result<XYZVectorI16, Error<Spi::Error>> {
        let mut output = XYZVectorI16::default();
        let buff = output.as_mut_bytes();
        self.read(regs::UI_OUTX_L_A_OIS_HG, buff).await?;
        Ok(output)
    }

    /// Get temperature in millicelsius.
    pub async fn read_temperature(&mut self) -> Result<MilliCelsius, Error<Spi::Error>> {
        let mut output = 0i16;
        self.read(regs::OUT_TEMP_L, output.as_mut_bytes()).await?;
        Ok(MilliCelsius(output as i32 + 25000))
    }

    /// Read the timestamp, 1 LSB = 21.7 us typical.
    /// TODO: needs to be enabled in FUNCTIONS_ENABLE
    pub async fn read_timestamp(&mut self) -> Result<u32, Error<Spi::Error>> {
        let mut output = 0u32;
        self.read(regs::TIMESTAMP0, output.as_mut_bytes()).await?;
        Ok(output)
    }

    pub async fn control_acceleration_high(
        &mut self,
        config: AccelerationModeDataRateHigh,
    ) -> Result<(), Error<Spi::Error>> {
        self.write(regs::CTRL1_XL_HG, &[config.to_reg()]).await
    }

    pub async fn filter_gyroscope(
        &mut self,
        config: GyroscopeBandwidthScale,
    ) -> Result<(), Error<Spi::Error>> {
        self.write(regs::CTRL6, &[config.to_reg()]).await
    }
    pub async fn control_gyroscope(
        &mut self,
        config: GyroscopeModeDataRate,
    ) -> Result<(), Error<Spi::Error>> {
        self.write(regs::CTRL2, &[config.to_reg()]).await
    }

    pub async fn read_gyroscope(&mut self) -> Result<XYZVectorI16, Error<Spi::Error>> {
        let mut output = XYZVectorI16::default();
        self.read(regs::OUTX_L_G, output.as_mut_bytes()).await?;
        Ok(output)
    }

    pub async fn control_fifo(&mut self, config: FifoControl) -> Result<(), Error<Spi::Error>> {
        self.write(regs::FIFO_CTRL4, &[config.to_reg()]).await
    }

    pub async fn control_fifo_batch(&mut self, config: FifoBatch) -> Result<(), Error<Spi::Error>> {
        self.write(regs::FIFO_CTRL3, &[config.to_reg()]).await
    }

    pub async fn control_fifo_counter(&mut self) -> Result<(), Error<Spi::Error>> {
        // this seriously needs work.
        // The high-g accelerometer BDR is not configurable and is always equal to the configured ODR.
        const XL_HG_BATCH_EN: u8 = 1 << 3;
        self.write(regs::COUNTER_BDR_REG1, &[XL_HG_BATCH_EN]).await
    }

    pub async fn get_fifo_status(&mut self) -> Result<FifoStatus, Error<Spi::Error>> {
        let mut output = FifoStatus::default();
        self.read(regs::FIFO_STATUS1, output.as_mut_bytes()).await?;
        Ok(output)
    }

    /// This does seem to allow more than reading a single value...
    /// CFG_CHANGE is probably useful to store / emit into the fifo at some point?
    ///
    /// AN6119; Rounding from FIFO_DATA_OUT_Z_H to FIFO_DATA_OUT_TAG is done automatically.
    pub async fn get_fifo(&mut self, values: &mut [u8]) -> Result<(), Error<Spi::Error>> {
        self.read(regs::FIFO_DATA_OUT_TAG, values).await
    }
}
