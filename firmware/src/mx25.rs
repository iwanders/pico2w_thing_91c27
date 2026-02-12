//! Driver for Macronix mx25xxx45 family flash memory

use crate::flash_memory::FlashMemory;
use bitfield_struct::bitfield;
use defmt::{info, warn};
use embedded_hal_async::spi::SpiDevice;
use zerocopy::{FromBytes, Immutable, IntoBytes, TryFromBytes};

pub mod regs {
    pub const JEDEC_ID: u8 = 0x9F;
    pub const STATUS: u8 = 0x05;
    pub const CONFIG: u8 = 0x15;
}
pub mod instructions {
    // All instructions follow; ACTUAL_READABLE_NAME_<ABBREVIATION_IN_DATASHEET>

    /// Enable four byte addressing mode, sets four_byte_address in config register.
    pub const FOUR_BYTE_ADDRESS_ENABLE_EN4B: u8 = 0xB7;
    /// Disable four byte addressing mode, sets four_byte_address in config register.
    pub const FOUR_BYTE_ADDRESS_DISABLE_EX4B: u8 = 0xE9;
    /// Write enable instruction, sets Write Enable Latch in status register.
    pub const WRITE_ENABLE_WREN: u8 = 0x06;
    /// Write disable instruction, clears Write Enable Latch in status register.
    pub const WRITE_DISABLE_WRDI: u8 = 0x04;

    /// Enter secured OTP
    pub const SECURED_OTP_ENTER_ENSO: u8 = 0xB1;
    /// Exit secured OTP
    pub const SECURED_OTP_EXIT_EXSO: u8 = 0xC1;

    /// Four byte fast read.
    pub const FAST_READ_4B_READ4B: u8 = 0x0c;
    /// Write to a page
    pub const PAGE_PROGRAM_4B_PP4B: u8 = 0x12;
    /// Normal 3 byte address read.
    pub const READ_NORMAL_READ: u8 = 0x03;

    /// Sector Erase
    pub const ERASE_SECTOR_4KB_SE4B: u8 = 0x21;

    /// Reset enable
    pub const RESET_ENABLE_RSTEN: u8 = 0x66;
    pub const RESET_MEMORY_RST: u8 = 0x99;
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
    /// Page program got too many bytes.
    ProgramExceedsPage,
    /// A write is still in progress and the device is occupied.
    WriteInProgress,
    /// Erase instruction has incorrect start, not on boundary of part to erase.
    EraseIncorrectStart,
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

 Program commands are executed on:
    - byte basis
    - page basis (256 bytes)
    - or word basis.
 Erase command is executed on:
    - 4k byte sector
    - 32k byte block
    - 64k byte block
    - whole chip.

    The device allow the interruption of Sector-Erase, Block-Erase or Page-Program operations and conduct other operations.
*/

impl<Spi: embedded_hal_async::spi::SpiDevice> Mx25<Spi>
where
    Spi: SpiDevice<u8>,
    Spi::Error: embedded_hal_async::spi::Error,
{
    async fn command(&mut self, register: u8) -> Result<(), Error<Spi::Error>> {
        use embedded_hal_async::spi::Operation;

        self.spi
            .transaction(&mut [Operation::Write(&[register])])
            .await?;
        Ok(())
    }
    async fn write_read(&mut self, write: &[u8], data: &mut [u8]) -> Result<(), Error<Spi::Error>> {
        use embedded_hal_async::spi::Operation;
        self.spi
            .transaction(&mut [Operation::Write(write), Operation::Read(data)])
            .await?;

        Ok(())
    }

    async fn verify_nothing_in_progress(&mut self) -> Result<(), Error<Spi::Error>> {
        let status = self.status().await?;
        if status.write_in_progress() {
            Err(Error::WriteInProgress)
        } else {
            Ok(())
        }
    }

    pub async fn cmd_read_normal(
        &mut self,
        address: u32,
        read_values: &mut [u8],
    ) -> Result<(), Error<Spi::Error>> {
        use embedded_hal_async::spi::Operation;
        let address = address.as_bytes();
        self.spi
            .transaction(&mut [
                Operation::Write(&[instructions::READ_NORMAL_READ]),
                Operation::Write(&address[0..3]),
                Operation::Read(read_values),
            ])
            .await?;
        Ok(())
    }
    pub async fn cmd_read_fast_4b(
        &mut self,
        address: u32,
        read_values: &mut [u8],
    ) -> Result<(), Error<Spi::Error>> {
        use embedded_hal_async::spi::Operation;

        self.spi
            .transaction(&mut [
                Operation::Write(&[instructions::FAST_READ_4B_READ4B]),
                Operation::Write(address.as_bytes()),
                Operation::Write(&[0]), // dummy byte
                Operation::Read(read_values),
            ])
            .await?;
        Ok(())
    }
    pub async fn cmd_page_program_4b(
        &mut self,
        address: u32,
        write_values: &[u8],
    ) -> Result<(), Error<Spi::Error>> {
        use embedded_hal_async::spi::Operation;
        if write_values.len() > 256 {
            return Err(Error::ProgramExceedsPage);
        }
        if write_values.len() == 0 {
            return Ok(()); // we must sent 1-n bytes.
        }

        // Wrapping is bad, lets not support that, lower address byte specifies starting address
        // within selected page, so length is limited based on the lower byte;
        let lower_u8 = (address & 0xFF) as u8;
        let max_data_length = 256 - (lower_u8 as u16);
        if write_values.len() > max_data_length as usize {
            return Err(Error::ProgramExceedsPage);
        }

        // Now we should be set to go...?
        self.set_write_mode(true).await?; // This is cleared automatically.
        self.spi
            .transaction(&mut [
                Operation::Write(&[instructions::PAGE_PROGRAM_4B_PP4B]),
                Operation::Write(address.as_bytes()),
                Operation::Write(write_values),
            ])
            .await?;
        //
        Ok(())
    }

    pub async fn cmd_erase_sector_4b(&mut self, address: u32) -> Result<(), Error<Spi::Error>> {
        use embedded_hal_async::spi::Operation;
        if address & 0xFFF != 0 {
            // Address doesn't land on boundary.
            return Err(Error::EraseIncorrectStart);
        }
        self.set_write_mode(true).await?; // This is cleared automatically.
        self.spi
            .transaction(&mut [
                Operation::Write(&[instructions::ERASE_SECTOR_4KB_SE4B]),
                Operation::Write(address.as_bytes()),
            ])
            .await?;

        Ok(())
    }

    pub async fn new(spi: Spi) -> Result<Self, Error<Spi::Error>> {
        // Verify we can read the whoami register.
        let mut z = Self { spi };
        let mut read = [0u8; 4];
        z.write_read(&[regs::JEDEC_ID], &mut read).await?;

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

    /// Reset the device.
    pub async fn reset(&mut self) -> Result<(), Error<Spi::Error>> {
        // Reset sequence is arm first, cs high and low, then trigger.
        self.command(instructions::RESET_ENABLE_RSTEN).await?;
        // let CS go high, then reset.
        self.command(instructions::RESET_MEMORY_RST).await
    }

    /// Retrieves the status register.
    pub async fn status(&mut self) -> Result<StatusRegister, Error<Spi::Error>> {
        let mut read = [0u8; 1];
        self.write_read(&[regs::STATUS], &mut read).await?;
        Ok(StatusRegister::try_read_from_prefix(&read).unwrap().0)
    }

    /// Retrieves the configuration register.
    pub async fn config(&mut self) -> Result<ConfigRegister, Error<Spi::Error>> {
        let mut read = [0u8; 1];
        self.write_read(&[regs::CONFIG], &mut read).await?;
        Ok(ConfigRegister::try_read_from_prefix(&read).unwrap().0)
    }

    /// Sets four byte mode.
    async fn set_four_byte_mode(&mut self, state: bool) -> Result<(), Error<Spi::Error>> {
        if state {
            self.command(instructions::FOUR_BYTE_ADDRESS_ENABLE_EN4B)
                .await
        } else {
            self.command(instructions::FOUR_BYTE_ADDRESS_DISABLE_EX4B)
                .await
        }
    }
    /// Sets write enabled mode.
    async fn set_write_mode(&mut self, state: bool) -> Result<(), Error<Spi::Error>> {
        if state {
            self.command(instructions::WRITE_ENABLE_WREN).await
        } else {
            self.command(instructions::WRITE_DISABLE_WRDI).await
        }
    }

    /// Sets whether or not we're in the otp mode.
    async fn set_secured_otp_mode(&mut self, state: bool) -> Result<(), Error<Spi::Error>> {
        if state {
            self.command(instructions::SECURED_OTP_ENTER_ENSO).await
        } else {
            self.command(instructions::SECURED_OTP_EXIT_EXSO).await
        }
    }

    pub async fn read_secure(
        &mut self,
        address: u16,
        data_read: &mut [u8],
    ) -> Result<(), Error<Spi::Error>> {
        self.set_secured_otp_mode(true).await?;
        self.cmd_read_normal(address as u32, data_read).await?;
        self.set_secured_otp_mode(false).await?;
        Ok(())
    }
}

impl<Spi: embedded_hal_async::spi::SpiDevice> FlashMemory for Mx25<Spi>
where
    Spi: SpiDevice<u8>,
    Spi::Error: embedded_hal_async::spi::Error,
{
    type Error = Error<Spi::Error>;
    const PAGE_SIZE: usize = 256;
    const SECTOR_SIZE: usize = 4096;

    async fn flash_write_page(&mut self, offset: u32, data: &[u8]) -> Result<(), Self::Error> {
        self.cmd_page_program_4b(offset, data).await
    }

    async fn flash_erase_sector(&mut self, offset: u32) -> Result<(), Self::Error> {
        self.cmd_erase_sector_4b(offset).await
    }

    async fn flash_read(&mut self, offset: u32, data: &mut [u8]) -> Result<(), Self::Error> {
        self.cmd_read_fast_4b(offset, data).await
    }

    async fn flash_flush(&mut self) -> Result<(), Self::Error> {
        let mut status = self.status().await?;
        while status.write_in_progress() {
            embassy_time::Timer::after_micros(1).await;
            status = self.status().await?
        }
        Ok(())
    }
}

pub async fn test_mx25<Spi>(mut flash: Mx25<Spi>) -> Result<(), Error<Spi::Error>>
where
    Spi: SpiDevice<u8>,
    Spi::Error: embedded_hal_async::spi::Error,
{
    use embassy_time::Instant;
    if false {
        let mut first_256 = [0u8; 256];
        for (i, v) in first_256.iter_mut().enumerate() {
            *v = i as u8;
        }
        flash.cmd_page_program_4b(0, &first_256).await?;
    }

    if true {
        // Erase the first two sectors.
        let start = Instant::now();
        flash.cmd_erase_sector_4b(0).await?;
        flash.flash_flush().await?;
        flash.cmd_erase_sector_4b(4096).await?;
        flash.flash_flush().await?;
        let end = Instant::now();
        defmt::info!("Erase and flush took {:?} us", (end - start).as_micros());
    }

    if true {
        // Dump the first two sectors.
        const BLOCK_SIZE: usize = 512;
        let upper = 4096 * 2;
        for i in 0..(upper / BLOCK_SIZE) {
            let p = BLOCK_SIZE * i;
            let mut d = [0u8; BLOCK_SIZE as usize];
            flash.cmd_read_fast_4b(p as u32, &mut d).await?;
            defmt::info!("// {:?}", p);
            defmt::info!("{:?},", d);
            embassy_time::Timer::after_millis(200).await;
        }
    }

    use crate::flash_memory::RecordManager;

    defmt::info!("Status: {:?}", flash.status().await?);

    if false {
        let mut start = Instant::now();
        let mut mgr =
            RecordManager::new(&mut flash, 0..(Mx25::<Spi>::SECTOR_SIZE as u32) * 2).await?;
        let mut end = Instant::now();
        defmt::info!("Init took : {:?} us", (end - start).as_micros());

        let mut total = embassy_time::Duration::from_nanos(0);
        // We got two sectors.
        let mut counter = 0u32;
        while !mgr.write_will_wrap(4) {
            start = Instant::now();
            let new_record = mgr.new_record(&mut flash, counter.as_bytes()).await?;
            end = Instant::now();
            let duration = end - start;
            total += duration;
            defmt::info!(
                "new record : {:?} us, {}, {}",
                duration.as_micros(),
                new_record.position,
                new_record.counter
            );
            counter += 1;
        }
        defmt::info!("total duration for writing: {:?} ms", total.as_millis(),);
    }

    if false {
        let mut start = Instant::now();
        let mut mgr =
            RecordManager::new(&mut flash, 0..(Mx25::<Spi>::SECTOR_SIZE as u32) * 2).await?;
        let mut end = Instant::now();
        defmt::info!("Init took : {:?} us", (end - start).as_micros());
        let will_wrap = mgr.write_will_wrap(4);
        defmt::info!("will_wrap {:?}  ", will_wrap);
        defmt::info!("available_before_wrap {:?}  ", mgr.available_before_wrap());

        if false {
            let mut counter = 0u32;
            start = Instant::now();
            let new_record = mgr.new_record(&mut flash, counter.as_bytes()).await?;
            end = Instant::now();
            defmt::info!("Write took : {:?} us", (end - start).as_micros());
        }
    }

    let mut counter = 0u32;
    loop {
        defmt::info!("Status: {:?}", flash.status().await?);
        embassy_time::Timer::after_millis(1000).await;
        let mut read_values = [0u8; 256];
        flash.cmd_read_fast_4b(0, &mut read_values).await?;
        defmt::info!("Toggling to : {:?}", read_values);
        counter += 1;

        if counter > 5 {
            flash.reset().await?;
            break;
        }
    }

    loop {
        embassy_time::Timer::after_millis(1000).await;
    }
}
