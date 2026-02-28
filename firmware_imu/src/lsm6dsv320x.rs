// ====================================================================================================================
//
// Looks like STM started an official driver: https://github.com/STMicroelectronics/lsm6dsv320x-rs
// This file predates that repo, but it's probably better for people to use their code instead of my code.
//
// ====================================================================================================================

use bitfield_struct::bitfield;
use embedded_hal_async::spi::SpiDevice;
use zerocopy::{FromBytes, Immutable, IntoBytes, TryFromBytes};

#[cfg(target_arch = "arm")]
use defmt::println;

// https://docs.rs/embedded-hal/1.0.0/embedded_hal/spi/index.html#for-driver-authors
// > If your device has a CS pin, use SpiDevice. Do not manually manage the CS pin, the SpiDevice implementation will
// > do it for you. By using SpiDevice, your driver will cooperate nicely with other drivers for other devices in the same shared SPI bus.
// That's us!

// all pages refer to the pdf marked DS14623 - Rev 2
// strike that, we also need an6119, that details fifo reading wraparound, and the compression algorithm.
//
// Low G, high G and gyro can be toggled independently and run at different data rates.
// High G is only available if low G is in high performance or high accuracy mode.

// Todo:
//  fix the hack in control_fifo_counter
//  probably coalesce some configuration properties.

// https://github.com/rust-embedded/embedded-hal/issues/572
// instead use;
// https://docs.embassy.dev/embassy-embedded-hal/git/default/shared_bus/asynch/spi/index.html

// Should we use bitfield-struct?
// THis seems to work:
// #[bitfield(u64)]
// #[derive(PartialEq, Eq, TryFromBytes, IntoBytes, Immutable, defmt::Format)] // <- Attributes after `bitfield` are carried over
// struct MyBitfield {

//
// The FIFO words are prefixed with a data tag which determines how to interpret them.
// high level description on page 44, tags on page 114
//  Where do the actual value-definitions live? are those just the identical register name? so page 85 for the
//  timestamp? WHich is just a 32 bit wordt in 21.7 uS increments.
//

pub mod regs {

    #[derive(Copy, Clone, Debug, defmt::Format)]
    pub enum Category {
        Normal,
        EmbeddedFunctions,
    }

    /// Constant to apply to the register address in order to specify a read.
    pub const REGISTER_READ: u8 = 1 << 7;

    /// Embedded function, SHUB, FSM register access.
    pub const FUNC_CFG_ACCESS: u8 = 0x01;

    /// The whoami register that always returns a constant value.
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

    /// Embedded function enable register (A)
    pub const EMB_FUNC_EN_A: (Category, u8) = (Category::EmbeddedFunctions, 0x04);
    /// Embedded function fifo register (A)
    pub const EMB_FUNC_FIFO_EN_A: (Category, u8) = (Category::EmbeddedFunctions, 0x44);
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
pub enum TimestampBatch {
    #[default]
    Disabled = 0b00,
    /// max(BDR_XL[Hz],BDR_GY[Hz])
    EveryBatch = 0b01,
    /// max(BDR_XL[Hz],BDR_GY[Hz]) / 8
    Every8thBatch = 0b10,
    /// max(BDR_XL[Hz],BDR_GY[Hz]) / 32
    Every32thBatch = 0b11,
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
    pub timestamp: TimestampBatch,
    pub temperature: TemperatureBatch,
    pub mode: FifoMode,
}
impl FifoControl {
    fn to_reg(&self) -> u8 {
        (self.timestamp as u8) << 6 | (self.temperature as u8) << 4 | self.mode as u8
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

#[derive(Copy, Clone, Debug, Eq, PartialEq, Default, defmt::Format)]
#[repr(u8)]
pub enum TriggerBDRSource {
    #[default]
    Acceleration = 0b00,
    Gyroscope = 0b01,
    GyroscopeEIS = 0b10,
    AccelerationHigh = 0b11,
}

#[derive(Copy, Clone, Debug, Eq, PartialEq, Default, defmt::Format)]
pub struct BatchDataRateConfig {
    pub trigger_bdr: TriggerBDRSource,
    pub batch_acceleration_high: bool,
    pub threshold: u16,
}
impl BatchDataRateConfig {
    fn to_reg1(&self) -> u8 {
        (self.trigger_bdr as u8) << 5
            | (self.batch_acceleration_high as u8) << 3
            | (self.threshold >> 8) as u8
    }
    fn to_reg2(&self) -> u8 {
        self.threshold as u8
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

    async fn write_category(
        &mut self,
        register: &(regs::Category, u8),
        data: &[u8],
    ) -> Result<(), Error<Spi::Error>> {
        match register.0 {
            regs::Category::Normal => self.write(register.1, data).await,
            regs::Category::EmbeddedFunctions => {
                self.write(
                    regs::FUNC_CFG_ACCESS,
                    &[FunctionConfigAccess::new().with_emb_func_reg_access(true).0],
                )
                .await?;
                let r = self.write(register.1, data).await;
                self.write(regs::FUNC_CFG_ACCESS, &[FunctionConfigAccess::new().0])
                    .await?;
                r
            }
        }
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

    pub async fn reset_old(&mut self) -> Result<(), Error<Spi::Error>> {
        // Write to contr CTRL3, p67
        const BOOT: u8 = 1 << 7;
        const BDU: u8 = 1 << 6;
        const IF_INC: u8 = 1 << 2;
        const SW_RESET: u8 = 1;

        self.write(regs::CTRL3, &[BOOT | BDU | IF_INC | SW_RESET])
            .await
    }

    // Or should reset be this:
    pub async fn reset(&mut self) -> Result<(), Error<Spi::Error>> {
        self.write(
            regs::FUNC_CFG_ACCESS,
            &[FunctionConfigAccess::new().with_sw_por(true).0],
        )
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

    /// Configure whether the high g values are batched, when the batch data rate increases and what the threshold is.
    pub async fn control_bdr_config(
        &mut self,
        config: BatchDataRateConfig,
    ) -> Result<(), Error<Spi::Error>> {
        self.write(
            regs::COUNTER_BDR_REG1,
            &[config.to_reg1(), config.to_reg2()],
        )
        .await
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

    /// Configures the embedded function enable register A.
    pub async fn embedded_functions_enable(
        &mut self,
        function_enable: EmbeddedFunctionEnableA,
    ) -> Result<(), Error<Spi::Error>> {
        self.write_category(&regs::EMB_FUNC_EN_A, &[function_enable.0])
            .await
    }
    /// Configures embedded function fifo register.
    pub async fn embedded_functions_fifo(
        &mut self,
        fifo_cfg: EmbeddedFunctionFifoA,
    ) -> Result<(), Error<Spi::Error>> {
        self.write_category(&regs::EMB_FUNC_FIFO_EN_A, &[fifo_cfg.0])
            .await
    }
}

/// FUNC_CFG_ACCESS register value, 9.1, p56.
#[bitfield(u8)]
#[derive(PartialEq, Eq, FromBytes, IntoBytes, Immutable, defmt::Format)]
pub struct FunctionConfigAccess {
    pub ois_ctrl_from_ui: bool,
    pub if2_reset: bool,
    pub sw_por: bool,
    pub fsm_wr_ctrl_en: bool,
    #[bits(2)]
    pub _unused: u8,
    pub shub_reg_access: bool,
    pub emb_func_reg_access: bool,
}

#[bitfield(u8)]
#[derive(PartialEq, Eq, FromBytes, IntoBytes, Immutable, defmt::Format)]
pub struct EmbeddedFunctionEnableA {
    pub unused: bool,
    pub sflp_game_enable: bool,
    pub unused2: bool,
    pub pedo_enable: bool,
    pub tilt_enable: bool,
    pub sign_motion_enable: bool,
    pub unused3: bool,
    pub mlc_before_fsm_enable: bool,
}

#[bitfield(u8)]
#[derive(PartialEq, Eq, FromBytes, IntoBytes, Immutable, defmt::Format)]
pub struct EmbeddedFunctionFifoA {
    pub unused: bool,
    pub sflp_game_fifo_enable: bool,
    pub unused2: bool,
    pub unused3: bool,
    pub sflp_gravity_fifo_enable: bool,
    pub sflp_gyroscope_bias_fifo_enable: bool,
    pub step_counter_fifo_enable: bool,
    pub mlc_fifo_enable: bool,
}

#[repr(u8)]
#[derive(PartialEq, Eq, TryFromBytes, IntoBytes, Immutable, defmt::Format, Debug)]
pub enum LsmFifoTag {
    FIFOempty = 0x00,
    // NC is non compressed?
    GyroscopeNC = 0x01,
    AccelerometerNC = 0x02,
    Temperature = 0x03,
    Timestamp = 0x04,
    ConfigChange = 0x05, // CFG_Change
    AccelerometerNCT2 = 0x06,
    AccelerometerNCT1 = 0x07,
    Accelerometer2xC = 0x08,
    Accelerometer3xC = 0x09,
    GyroscopeNCT2 = 0x0A,
    GyroscopeNCT1 = 0x0B,
    Gyroscope2xC = 0x0C,
    Gyroscope3xC = 0x0D,
    SensorHubTarget0 = 0x0E,
    SensorHubTarget1 = 0x0F,
    SensorHubTarget2 = 0x10,
    SensorHubTarget3 = 0x11,
    StepCounter = 0x12,
    SFLPGamerotationVector = 0x13,
    SFLPGyroscopeBias = 0x16,
    SFLPGravityVector = 0x17,
    HighGAccelerometerPeakValue = 0x18,
    SensorhubNack = 0x19,
    MLCResult = 0x1A,
    MLCFilter = 0x1B,
    MLCFeature = 0x1C,
    HighGAccelerometer = 0x1D,
    EnhancedEISGyroscope = 0x1E,
    FSMResults = 0x1F,
}

/// Error used by the accessory interface
#[derive(thiserror::Error, Debug, Copy, Clone, PartialEq, Eq, defmt::Format)]
pub enum LsmFifoError {
    /// Not enough bytes provided
    #[error("not enough data")]
    NotEnoughData,
    /// If the data tag byte could not be interpreted.
    #[error("first byte did not hold a tag {0:?}")]
    FirstByteNoTag(u8),
}

/// Fifo Data Out Tag register value, 9.90, p114.
#[bitfield(u8)]
#[derive(PartialEq, Eq, FromBytes, IntoBytes, Immutable, defmt::Format)]
pub struct FifoDataOutTag {
    #[bits(1)] // bit 0
    pub _unused: bool,

    /// 2-bit counter which identifies sensor time slot
    ///
    /// What does this mean? What is a sensor time slot? is it time in between the timestamp values?
    /// Is this an implementation detail? Are individual sensors just retrieved in subslots to avoid interference?
    #[bits(2)] // bit 1-2
    pub count: u8,
    /// FIFO tag, identifies the sensor in the following registers
    ///
    ///  DATA_OUT_X_L, DATA_OUT_X_H
    ///  DATA_OUT_Y_L, DATA_OUT_Y_H,
    ///  DATA_OUT_Z_L, DATA_OUT_Z_H,
    #[bits(5)] // Bit 2-7
    pub sensor: u8,
}
impl FifoDataOutTag {
    pub fn sensor_type(&self) -> Result<LsmFifoTag, LsmFifoError> {
        LsmFifoTag::try_read_from_bytes(&[self.sensor()])
            .map_err(|_e| LsmFifoError::FirstByteNoTag(self.sensor()))
    }
}

/// This is a helper to parse data from the fifo.
#[derive(Debug, Default, Copy, Clone)]
pub struct LsmFifoProcessor {
    pub gyro_scale: GyroscopeScale,
    pub accel_scale: AccelerationScale,
    pub accel_high_scale: AccelerationScaleHigh,
}
impl LsmFifoProcessor {
    pub fn read_data(&self, data: &[u8]) -> Result<(), LsmFifoError> {
        // Data tag is always 7 long? One tag + x_{l,h} + y_{l,h} + z_{l,h}
        // Is it though? it parses poorly at 7 byte steps, regardless of the offset...
        // Section 6.5.8; FIFO reading procedure, p46.
        // Each fifo word is 7 bytes.
        for v in data.chunks_exact(7) {
            let tag = FifoDataOutTag::read_from_bytes(&v[0..1])
                .map_err(|_e| LsmFifoError::NotEnoughData)?;
            let sensor_type = tag.sensor_type()?;

            println!("sensor_type: {:?}", sensor_type);
        }

        todo!()
    }

    pub fn interpret(&self, tag: LsmFifoTag, data: &[u8]) -> FifoEntry {
        match tag {
            LsmFifoTag::GyroscopeNC => FifoEntry::GyroscopeNC(FifoGyroscopeNc {
                scale: self.gyro_scale,
                x: i16::from_le_bytes(data[0..2].try_into().unwrap()),
                y: i16::from_le_bytes(data[2..4].try_into().unwrap()),
                z: i16::from_le_bytes(data[4..6].try_into().unwrap()),
            }),
            LsmFifoTag::AccelerometerNC => FifoEntry::AccelerometerNC(FifoAccelerometerNC {
                scale: self.accel_scale,
                x: i16::from_le_bytes(data[0..2].try_into().unwrap()),
                y: i16::from_le_bytes(data[2..4].try_into().unwrap()),
                z: i16::from_le_bytes(data[4..6].try_into().unwrap()),
            }),
            LsmFifoTag::HighGAccelerometer => {
                FifoEntry::HighGAccelerometer(FifoHighGAccelerometer {
                    scale: self.accel_high_scale,
                    x: i16::from_le_bytes(data[0..2].try_into().unwrap()),
                    y: i16::from_le_bytes(data[2..4].try_into().unwrap()),
                    z: i16::from_le_bytes(data[4..6].try_into().unwrap()),
                })
            }

            // [41, 252, 0, 0, 0, 0] le
            LsmFifoTag::Temperature => FifoEntry::Temperature(FifoTemperature {
                t: i16::from_le_bytes([data[0], data[1]]),
            }),
            LsmFifoTag::Timestamp => FifoEntry::Timestamp(FifoTimestamp {
                // Is this actually BE? check how timestamp should be.
                // Yes, it is big endian, see page 85, section 9.42.
                // Scale is 21.7us typical, but depends on INTERNAL_FREQ_FINE 9.54, p92
                t: u32::from_be_bytes(data[2..6].try_into().unwrap()),
            }),
            LsmFifoTag::SFLPGamerotationVector => {
                println!("d: {:?}", data);
                // Ugh, this is a different format, it spans two words!
                // SFLPGamerotationVector [0, 0, 234, 165, 4, 177]
                // d: [0, 0, 234, 165, 4, 177]
                //  GameRotationVector(GameRotationVector)
                // SFLPGamerotationVector [1, 0, 230, 187, 148, 23]
                // d: [1, 0, 230, 187, 148, 23]

                FifoEntry::GameRotationVector(GameRotationVector {})
            }

            v => todo!("unimplemented tag: {v:?}"),
        }
    }
}
#[derive(Debug, Default, Copy, Clone)]
struct FifoGyroscopeNc {
    scale: GyroscopeScale,
    x: i16,
    y: i16,
    z: i16,
}
#[derive(Debug, Default, Copy, Clone)]
struct FifoAccelerometerNC {
    scale: AccelerationScale,
    x: i16,
    y: i16,
    z: i16,
}
#[derive(Debug, Default, Copy, Clone)]
struct FifoHighGAccelerometer {
    scale: AccelerationScaleHigh,
    x: i16,
    y: i16,
    z: i16,
}

#[derive(Debug, Default, Copy, Clone)]
struct FifoTimestamp {
    /// Scale is 21.7us typical, but depends on INTERNAL_FREQ_FINE 9.54, p92
    t: u32,
}

#[derive(Debug, Default, Copy, Clone)]
struct FifoTemperature {
    // In 256 LSB / C,The output of the temperature sensor is 0 LSB (typ.) at 25°C. p16 4.3
    t: i16,
}

#[derive(Debug, Default, Copy, Clone)]
struct GameRotationVector {}

#[derive(Debug, Default, Copy, Clone)]
pub enum FifoEntry {
    #[default]
    Empty,
    GyroscopeNC(FifoGyroscopeNc),
    AccelerometerNC(FifoAccelerometerNC),
    HighGAccelerometer(FifoHighGAccelerometer),
    Timestamp(FifoTimestamp),
    Temperature(FifoTemperature),
    GameRotationVector(GameRotationVector),
}

// Probably make an iterator that returns (DataTag, &[u8])
pub struct LsmFifoIterator<'d> {
    data: &'d [u8],
    position: usize,
}
impl<'d> LsmFifoIterator<'d> {
    pub fn new(data: &'d [u8]) -> Self {
        println!("data len: {}", data.len());
        Self { data, position: 0 }
    }
}
impl<'d> Iterator for LsmFifoIterator<'d> {
    type Item = Result<(LsmFifoTag, &'d [u8]), LsmFifoError>;

    fn next(&mut self) -> Option<Self::Item> {
        if self.position == self.data.len() {
            // Good end, it ends on complete data.
            return None;
        }
        if (self.position + 7) <= self.data.len() {
            let tag = match FifoDataOutTag::read_from_bytes(&[self.data[self.position]]) {
                Ok(v) => v,
                Err(_) => return Some(Err(LsmFifoError::NotEnoughData)),
            };
            let data_type = match tag.sensor_type() {
                Ok(v) => v,
                Err(_) => return Some(Err(LsmFifoError::FirstByteNoTag(tag.sensor()))),
            };
            //println!("tag: {:?}", tag);
            let res = Some(Ok((
                data_type,
                &self.data[self.position + 1..self.position + 7],
            )));
            self.position += 7;
            return res;
        } else {
            // incomplete word count.
            return Some(Err(LsmFifoError::NotEnoughData));
        }
    }
}

#[cfg(test)]
mod test {
    use super::*;
    #[test]
    fn test_lsm_fifo_processor() {
        let data = [
            12, 253, 255, 0, 0, 1, 0, 20, 146, 0, 10, 0, 255, 15, 237, 0, 0, 248, 255, 114, 0, 39,
            0, 0, 0, 0, 32, 188, 23, 134, 0, 14, 0, 6, 16, 238, 250, 255, 8, 0, 92, 0, 33, 0, 0, 0,
            0, 32, 188, 9, 253, 255, 0, 0, 1, 0, 17, 128, 0, 249, 255, 27, 16, 232, 16, 0, 247,
            255, 94, 0, 34, 0, 0, 0, 0, 32, 188, 18, 136, 0, 0, 0, 17, 16, 235, 14, 0, 237, 255,
            120, 0, 36, 0, 0, 0, 0, 32, 188, 12, 252, 255, 255, 255, 1, 0, 20, 135, 0, 8, 0, 2, 16,
            237, 252, 255, 23, 0, 98, 0, 39, 0, 0, 0, 0, 32, 188, 23, 120, 0, 21, 0, 2, 16, 238,
            252, 255, 249, 255, 109, 0, 33, 0, 0, 0, 0, 32, 188, 9, 250, 255, 254, 255, 0, 0, 17,
            122, 0, 21, 0, 251, 15, 232, 5, 0, 245, 255, 113, 0, 34, 0, 0, 0, 0, 32, 188, 18, 135,
            0, 249, 255, 248, 15, 235, 10, 0, 9, 0, 93, 0, 36, 0, 0, 0, 0, 32, 188, 12, 249, 255,
            253, 255, 0, 0, 20, 136, 0, 255, 255, 244, 15, 237, 17, 0, 237, 255, 112, 0, 39, 0, 0,
            0, 0, 32, 188, 23, 143, 0, 16, 0, 245, 15, 238, 251, 255, 251, 255, 102, 0, 33, 0, 0,
            0, 0, 32, 188, 9, 249, 255, 252, 255, 1, 0, 17, 145, 0, 255, 255, 248, 15, 232, 15, 0,
            6, 0, 107, 0, 34, 0, 0, 0, 0, 32, 188, 18, 136, 0, 8, 0, 11, 16, 235, 5, 0, 248, 255,
            104, 0, 36, 0, 0, 0, 0, 32, 188, 12, 250, 255, 252, 255, 2, 0, 20, 118, 0, 17, 0, 14,
            16, 237, 1, 0, 246, 255, 97, 0, 39, 0, 0, 0, 0, 32, 188, 23, 120, 0, 255, 255, 8, 16,
            238, 12, 0, 9, 0, 110, 0, 33, 0, 0, 0, 0, 32, 188, 9, 250, 255, 252, 255, 2, 0, 17,
            123, 0, 1, 0, 15, 16, 232, 7, 0, 246, 255, 103, 0, 34, 0, 0, 0, 0, 32, 188, 18, 119, 0,
            9, 0, 4, 16, 235, 10, 0, 246, 255, 106, 0, 36, 0, 0, 0, 0, 32, 188, 12, 249, 255, 251,
            255, 2, 0, 20, 131, 0, 248, 255, 17, 16, 237, 255, 255, 0, 0, 123, 0, 39, 0, 0, 0, 0,
            32, 188, 23, 121, 0, 247, 255, 12, 16, 238, 13, 0, 4, 0, 97, 0, 33, 0, 0, 0, 0, 32,
            188, 9, 248, 255, 251, 255, 2, 0, 17, 122, 0, 250, 255, 251, 15, 232, 15, 0, 11, 0,
            101, 0, 34, 0, 0, 0, 0, 32, 188, 18, 117, 0, 242, 255, 7, 16, 235, 5, 0, 236,
        ];

        let processor = LsmFifoProcessor {
            accel_scale: AccelerationScale::G8,
            accel_high_scale: AccelerationScaleHigh::G320,
            gyro_scale: GyroscopeScale::DPS4000,
        };
        let mut iter = LsmFifoIterator::new(&data[0..(data.len() / 7) * 7]);
        for v in iter {
            let (data_type, bytes) = v.unwrap();
            println!("{data_type:?} {bytes:?}");
            let r = processor.interpret(data_type, bytes);
            println!(" {r:?}");
        }
        // Timestamp looks surprisingly... constant.
        panic!();
    }
}
