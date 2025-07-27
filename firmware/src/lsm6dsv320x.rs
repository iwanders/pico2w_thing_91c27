use embedded_hal_async::spi::SpiDevice;
// https://docs.rs/embedded-hal/1.0.0/embedded_hal/spi/index.html#for-driver-authors
// > If your device has a CS pin, use SpiDevice. Do not manually manage the CS pin, the SpiDevice implementation will
// > do it for you. By using SpiDevice, your driver will cooperate nicely with other drivers for other devices in the same shared SPI bus.
// That's us!

// https://github.com/rust-embedded/embedded-hal/issues/572
// https://docs.embassy.dev/embassy-embedded-hal/git/default/shared_bus/asynch/spi/index.html

// all pages refer to the pdf marked DS14623 - Rev 2
//
// Low G, high G and gyro can be toggled independently and run at different data rates.
// High G is only available if low G is in high performance or high accuracy mode.

// Tsu(CS) is 20ns, so we need a 20ns delay at the start of an SPI transaction.
// https://docs.rs/embedded-hal/1.0.0/embedded_hal/spi/index.html#cs-to-clock-delays
// Drivers should NOT use Operation::DelayNs for this... well then.
// I should patch up https://github.com/embassy-rs/embassy/blob/77a8bc27e9c34e363f321132ebb9e8d8ff684a9f/embassy-embedded-hal/src/shared_bus/asynch/spi.rs#L1-L212
// Seems we can ignore it for now.

use zerocopy::{FromBytes, IntoBytes};

// https://github.com/google/zerocopy/issues/1497 If only...

pub mod regs {

    pub const REGISTER_READ: u8 = 1 << 7;
    pub const WHO_AM_I: u8 = 0x0F;

    /// Control register with software reset and spi register advance.
    pub const CTRL3: u8 = 0x12;

    /// Accelerometer control reg 1, mode and data rate.
    pub const CTRL1: u8 = 0x10;
    /// Acceleration control 8, filter & scale selection.
    pub const CTRL8: u8 = 0x17;

    /// Acceleration Linear X low, two bytes, two's complement. Followed by y and z.
    pub const OUTX_L_A: u8 = 0x28;
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
    HighAccuracy = 0b010,
    ODRTrigger = 0b011,
    LowPowerMean2 = 0b100,
    LowPowerMean4 = 0b101,
    LowPowerMean8 = 0b110,
    Normal = 0b111,
}

#[derive(Copy, Clone, Debug, Eq, PartialEq, Default, defmt::Format)]
#[repr(u8)]
pub enum AccelerationDataRate {
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
    pub rate: AccelerationDataRate,
}

#[derive(Copy, Clone, Debug, Eq, PartialEq, Default, defmt::Format)]
#[repr(u8)]
pub enum AccelerationRange {
    #[default]
    G2 = 0b00,
    G4 = 0b01,
    G8 = 0b10,
    G16 = 0b11,
}
pub struct AccelerationFilterScale {
    pub scale: AccelerationRange,
    // Skip filter stuff for now, it is spread out over ctrl8 and ctrl0.
}

#[derive(Copy, Clone, Debug, Eq, PartialEq, Default, defmt::Format, FromBytes, IntoBytes)]
pub struct AccelerationXYZ {
    pub x: i16,
    pub y: i16,
    pub z: i16,
}

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

    pub async fn read_acceleration(&mut self) -> Result<AccelerationXYZ, Error<Spi::Error>> {
        let mut output = AccelerationXYZ::default();
        let buff = output.as_mut_bytes();
        self.read(regs::OUTX_L_A, buff).await?;
        Ok(output)
    }
}
