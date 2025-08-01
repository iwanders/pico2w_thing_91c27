use embedded_hal_async::spi::SpiDevice;
use zerocopy::{FromBytes, IntoBytes};

// See also the notes in the lsm6dsv320x file.

// Okay, so allows external clock... 20 bit fifo, 19 bit gyro & 18 bit accel, some motion processing engine
// and another temperature sensor.
//
// 20 bit data format details are in section 6.
//
// device powers up in sleep mode.
//
// Data is big endian!

pub mod regs {
    pub const REGISTER_READ: u8 = 1 << 7;
    pub const WHO_AM_I: u8 = 0x75;

    /// Contains SPI settings and reset.
    pub const DEVICE_CONFIG: u8 = 0x11;

    /// Temperature data registers, first of two.
    pub const TEMP_DATA1: u8 = 0x1D;

    /// Power management, temp sensor, gyro and accel.
    pub const PWR_MGMT0: u8 = 0x4E;
    /// First acceleration data register.
    pub const ACCEL_DATA_X1: u8 = 0x1F;
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
    Hz12dot5 = 0b1011,
    Hz500 = 0b1111,
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
    Hz12dot5 = 0b1011,
    Hz6dot25 = 0b1100,
    Hz3dot125 = 0b1101,
    Hz1dot5626 = 0b1110,
    Hz500 = 0b1111,
}

#[derive(Copy, Clone, Debug, Eq, PartialEq, Default, defmt::Format)]
#[repr(u8)]
pub enum GyroscopeScale {
    #[default]
    DPS2000 = 0b000,
    DPS1000 = 0b001,
    DPS500 = 0b010,
    DPS250 = 0b011,
    DPS125 = 0b100,
    DPS62dot5 = 0b101,
    DPS31dot25 = 0b110,
    DPS15dot625 = 0b111,
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
    fn to_reg(&self) -> u8 {
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

impl<Spi: embedded_hal_async::spi::SpiDevice> ICM42688<Spi>
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
        defmt::debug!("Read back: {:?}", data);

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
    pub async fn read_temperature(&mut self) -> Result<i16, Error<Spi::Error>> {
        let mut output = 0i16;
        self.read(regs::TEMP_DATA1, output.as_mut_bytes()).await?;
        Ok(output.swap_bytes())
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
}
