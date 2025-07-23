use embedded_hal_async::i2c::I2c;
use embedded_hal_async::i2c::SevenBitAddress;

// Address is right-aligned; https://docs.rs/embedded-hal-async/latest/embedded_hal_async/i2c/type.SevenBitAddress.html

pub mod reg {
    pub const REG_BME280_ID: u8 = 0xd0;
}

pub const DEFAULT_ADDRESS: SevenBitAddress = 0x76;
pub const ALTERNATE_ADDRESS: SevenBitAddress = 0x77; // untested.

#[derive(Debug, Clone, PartialEq)]
pub enum Error<I2cError: embedded_hal_async::i2c::Error> {
    /// Underlying I2C device error
    I2c(I2cError),
    /// An incorrect id was returned from the ID register, device address collision?
    IncorrectId,
}
impl<I2cError: embedded_hal_async::i2c::Error> From<I2cError> for Error<I2cError> {
    fn from(e: I2cError) -> Self {
        Error::<I2cError>::I2c(e)
    }
}

impl<I2cError: embedded_hal_async::i2c::Error> defmt::Format for Error<I2cError> {
    fn format(&self, f: defmt::Formatter) {
        match *self {
            Error::I2c(ref e) => defmt::write!(f, "Error:I2c({:?})", defmt::Debug2Format(&e)),
            Error::IncorrectId => defmt::write!(f, "Error:ResetTimeout",),
        }
    }
}

#[derive(Clone, PartialEq)]
pub struct BME280<I2c> {
    bus: I2c,
    address: SevenBitAddress,
}
impl<I2c> core::fmt::Debug for BME280<I2c> {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        f.write_fmt(format_args!("BME280:{}", self.address))
    }
}
impl<I2c> defmt::Format for BME280<I2c> {
    fn format(&self, f: defmt::Formatter) {
        defmt::write!(f, "BME280:{}", self.address)
    }
}

impl<I2C, I2cError: embedded_hal_async::i2c::Error> BME280<I2C>
where
    I2C: I2c<Error = I2cError>,
    I2cError: core::fmt::Debug,
{
    pub async fn new(address: SevenBitAddress, i2c: I2C) -> Result<Self, Error<I2cError>> {
        let mut i2c = i2c;
        let mut read = [0u8; 1];
        let mut buf = [0u8; 1];
        buf[0] = reg::REG_BME280_ID;
        i2c.write_read(address, &buf, &mut read).await?;
        if read[0] == 0x60 {
            Ok(BME280 { address, bus: i2c })
        } else {
            Err(Error::IncorrectId)
        }
    }
}
