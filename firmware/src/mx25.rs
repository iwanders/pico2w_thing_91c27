//! Driver for Macronix mx25xxx45 family flash memory

use bitfield_struct::bitfield;
use embedded_hal_async::spi::SpiDevice;
use zerocopy::{FromBytes, Immutable, IntoBytes, TryFromBytes};

pub mod regs {
    pub const JEDEC_ID: u8 = 0x9F;
    pub const STATUS: u8 = 0x05;
    pub const CONFIG: u8 = 0x15;
}
pub mod instructions {
    pub const EN4B: u8 = 0xB7;
    pub const EX4B: u8 = 0xE9;
}

// Not really registers, but whatever.
pub const JEDEC_MACRONIX_ID: u8 = 0xC2;
pub const JEDEC_MEMORY_TYPE_MX25L25645G: u8 = 0x20;
pub const JEDEC_MEMORY_DENSITY_MX25L25645G: u8 = 0x19;

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

#[bitfield(u8)]
#[derive(PartialEq, Eq, TryFromBytes, IntoBytes, Immutable, defmt::Format)]
pub struct StatusRegister {
    #[bits(1)] // bit 0
    write_in_progress: bool,

    #[bits(1)] // bit 1
    write_enable: bool,

    #[bits(1)] // bit 2
    bp0_protected: bool,
    #[bits(1)] // bit 3
    bp1_protected: bool,
    #[bits(1)] // bit 4
    bp2_protected: bool,
    #[bits(1)] // bit 5
    bp3_protected: bool,
    #[bits(1)] // bit 6
    quad_enable: bool,
    #[bits(1)] // bit 7
    status_write_protect: bool,
}

#[bitfield(u8)]
#[derive(PartialEq, Eq, TryFromBytes, IntoBytes, Immutable, defmt::Format)]
pub struct ConfigRegister {
    #[bits(2)] // bit 0-1
    output_driver_strength: u8,

    #[bits(1)] // bit 2
    _reserved_bit2: bool,

    #[bits(1)] // bit 3
    top_bottom_select: bool,

    #[bits(1)] // bit 4
    preamble_bit_enable: bool,
    #[bits(1)] // bit 5
    four_byte_address: bool,
    #[bits(2)] // bit 6-7
    dummy_cycle: u8,
}

/*
 On 128mbit and higher; requires 4 byte addresses. Either use:
    - Set 4 byte address in config registry.
    - Use extended adddress registry to toggle which section (EAR)
    - Use 4 byte address command set.

    Chip erase wipes everything, regardless of EAR selection

 On write enable:
    Instructions like PP/PP4B, 4PP/4PP4B, SE/SE4B, BE32K/BE32K4B, BE/BE4B, CE, and WRSR that are intended to change the device content, should be
    preceded by the WREN instruction

    The WEL bit is reset in the following situations: <pretty much everything>
*/

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
        if read[0] == JEDEC_MACRONIX_ID
            && read[1] == JEDEC_MEMORY_TYPE_MX25L25645G
            && read[2] == JEDEC_MEMORY_DENSITY_MX25L25645G
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
    pub async fn status(&mut self) -> Result<StatusRegister, Error<Spi::Error>> {
        let mut read = [0u8; 1];
        self.read(regs::STATUS, &mut read).await?;
        Ok(StatusRegister::try_read_from_prefix(&read).unwrap().0)
    }

    pub async fn config(&mut self) -> Result<ConfigRegister, Error<Spi::Error>> {
        let mut read = [0u8; 1];
        self.read(regs::CONFIG, &mut read).await?;
        Ok(ConfigRegister::try_read_from_prefix(&read).unwrap().0)
    }

    pub async fn set_four_byte_mode(&mut self, state: bool) -> Result<(), Error<Spi::Error>> {
        if state {
            self.write(instructions::EN4B, &[]).await
        } else {
            self.write(instructions::EX4B, &[]).await
        }
    }
}

pub async fn test_mx25<Spi>(mut flash: Mx25<Spi>) -> Result<(), Error<Spi::Error>>
where
    Spi: SpiDevice<u8>,
    Spi::Error: embedded_hal_async::spi::Error,
{
    let mut four_byte_toggle = false;
    loop {
        embassy_time::Timer::after_millis(1000).await;
        defmt::info!("Toggling to : {:?}", four_byte_toggle);
        flash.set_four_byte_mode(four_byte_toggle).await?;
        defmt::info!("Status: {:?}", flash.status().await?);
        defmt::info!("Config: {:?}", flash.config().await?);
        four_byte_toggle = !four_byte_toggle;
    }
}
