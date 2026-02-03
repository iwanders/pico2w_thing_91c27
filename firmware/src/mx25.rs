//! Driver for Macronix mx25xxx45 family flash memory

use embedded_hal_async::spi::SpiDevice;
use zerocopy::{FromBytes, IntoBytes};

pub mod regs {
    pub const REGISTER_READ: u8 = 1 << 7;
    pub const JEDEC_ID: u8 = 0x9F;

    pub const JEDEC_MACRONIX_ID: u8 = 0xC2;
    pub const JEDEC_MEMORY_TYPE_MX25L25645G: u8 = 0x20;
    pub const JEDEC_MEMORY_DENSITY_MX25L25645G: u8 = 0x19;
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
pub struct Mx25<Spi> {
    spi: Spi,
}

impl<Spi> defmt::Format for Mx25<Spi> {
    fn format(&self, f: defmt::Formatter) {
        defmt::write!(f, "<Mx25>")
    }
}

impl<Spi: embedded_hal_async::spi::SpiDevice> Mx25<Spi>
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
            .transaction(&mut [Operation::Write(&[register]), Operation::Read(data)])
            .await?;

        Ok(())
    }

    pub async fn new(spi: Spi) -> Result<Self, Error<Spi::Error>> {
        // Verify we can read the whoami register.
        let mut z = Self { spi };
        let mut read = [0u8; 4];
        z.read(regs::JEDEC_ID, &mut read).await?;

        defmt::info!("Flash jedec: {:?} {:x}", z, read);
        // Macronix has manufacturer ID c2.
        if read[0] == regs::JEDEC_MACRONIX_ID
            && read[1] == regs::JEDEC_MEMORY_TYPE_MX25L25645G
            && read[2] == regs::JEDEC_MEMORY_DENSITY_MX25L25645G
        {
            defmt::info!("Flash is responsive!");
            Ok(z)
        } else {
            defmt::error!("Flash test failed!");
            Err(Error::UnexpectedWhoAmI)
        }
    }

    /// Reset the device, wait at least 1ms after before read/writing registers.
    pub async fn reset(&mut self) -> Result<(), Error<Spi::Error>> {
        // const SOFT_RESET_CONFIG: u8 = 1;
        // self.write(regs::DEVICE_CONFIG, &[SOFT_RESET_CONFIG]).await
        todo!()
    }
}

pub async fn test_mx25<Spi>(flash: Mx25<Spi>) -> !
where
    Spi: SpiDevice<u8>,
    Spi::Error: embedded_hal_async::spi::Error,
{
    loop {
        embassy_time::Timer::after_millis(100).await;
    }
}
