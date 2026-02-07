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

#[allow(async_fn_in_trait)]
pub trait FlashMemory {
    type Error;
    /// Write data to flash, up to 256 bytes, may not cross page boundary.
    async fn flash_write_page(&mut self, offset: u32, data: &[u8]) -> Result<(), Self::Error>;
    /// Clear a sector, offset is at the sector start boundary.
    async fn flash_erase_sector_4k(&mut self, offset: u32) -> Result<(), Self::Error>;
    /// Read data from flash.
    async fn flash_read(&mut self, offset: u32, data: &mut [u8]) -> Result<(), Self::Error>;
}

impl<Spi: embedded_hal_async::spi::SpiDevice> FlashMemory for Mx25<Spi>
where
    Spi: SpiDevice<u8>,
    Spi::Error: embedded_hal_async::spi::Error,
{
    type Error = Error<Spi::Error>;

    async fn flash_write_page(&mut self, offset: u32, data: &[u8]) -> Result<(), Self::Error> {
        self.cmd_page_program_4b(offset, data).await
    }

    async fn flash_erase_sector_4k(&mut self, offset: u32) -> Result<(), Self::Error> {
        self.cmd_erase_sector_4b(offset).await
    }

    async fn flash_read(&mut self, offset: u32, data: &mut [u8]) -> Result<(), Self::Error> {
        self.cmd_read_fast_4b(offset, data).await
    }
}

/// This iterator helps cut up data on segment boundaries.
///
/// The flash memory often works on pages (256 bytes) or sectors (4k byte blocks). But our data may start at an arbitrary
/// offset into the memory.
///
/// Consider a situation for pages, which are aligned on 256 boundaries.
///                v data offset 253
/// data:          0 | 1 | 2 | 3 | 4 | 5 | 6 | 7
/// segment0  0..253  54  56 |
/// segment1                 257 258 259 260 261
///
/// Since the data is 8 bytes long and starts at 253, it crossess a page boundary. This means that this iterator will
/// return two entries.
///  - The first segment falls in aligned 0..256, with data indices 0..3, and overlapping range: 253..256
///  - The second segment falls in aligned 256..512, with data indices 3..7, and overlapping range 256..261
struct AlignedSegmentIter {
    data_offset: usize,
    data_end: usize,
    segment_index: usize,
    segment_size: usize,
}
impl AlignedSegmentIter {
    fn new<const SEGMENT_SIZE: usize>(start_offset: usize, data_length: usize) -> Self {
        let data_end = start_offset + data_length;
        let segment_index = start_offset / SEGMENT_SIZE;
        AlignedSegmentIter {
            data_offset: start_offset,
            data_end,
            segment_index,
            segment_size: SEGMENT_SIZE,
        }
    }
}

/// Struct to hold the return of the overlapping segments iterator.
pub struct AlignedSegment {
    /// The aligned segment, purely based on the segment start and end at the boundaries. Mostly for debugging.
    pub aligned: core::ops::Range<usize>,
    /// The range in the aligned block for which we have data.
    pub overlap: core::ops::Range<usize>,
    /// The interval in the data which maps to the section 'overlap' in the aligned segments.
    pub data: core::ops::Range<usize>,
}

impl Iterator for AlignedSegmentIter {
    type Item = AlignedSegment;

    fn next(&mut self) -> Option<Self::Item> {
        let segment_start = self.segment_index * self.segment_size;
        if segment_start > self.data_end {
            return None;
        }

        let segment_end = segment_start + self.segment_size;
        let overlap_start = self.data_offset.max(segment_start);
        let overlap_end = self.data_end.min(segment_end);
        if overlap_start == overlap_end {
            return None;
        }

        if overlap_start <= overlap_end {
            let overlap = overlap_start..overlap_end;
            let data = (overlap_start - self.data_offset)..(overlap_end - self.data_offset);
            self.segment_index += 1;
            Some(AlignedSegment {
                aligned: segment_start..segment_end,
                overlap,
                data,
            })
        } else {
            None
        }
    }
}

/// A chunker specifically for a data slice at a position, in segments of 256, where the start need not align on a
/// boundary.
pub struct ProgramChunker<'a> {
    chunker: AlignedSegmentIter,
    data: &'a [u8],
}
impl<'a> ProgramChunker<'a> {
    pub fn new(offset: usize, data: &'a [u8]) -> Self {
        Self {
            chunker: AlignedSegmentIter::new::<256>(offset, data.len()),
            data,
        }
    }
}

pub struct ProgramChunk<'a> {
    pub offset: usize,
    pub data: &'a [u8],
}

impl<'a> Iterator for ProgramChunker<'a> {
    type Item = ProgramChunk<'a>;

    fn next(&mut self) -> Option<Self::Item> {
        if let Some(AlignedSegment { overlap, data, .. }) = self.chunker.next() {
            Some(ProgramChunk {
                offset: overlap.start,
                data: &self.data[data],
            })
        } else {
            None
        }
    }
}

/// A chunker for erase instructions, returns the 4k sector boundaries for all sectors overlapping the range. This will
/// contain all the sectors that are touched by the erase range, and may thus delete more than erase_range.
pub struct EraseChunker {
    chunker: AlignedSegmentIter,
}
pub struct EraseChunk {
    pub offset: usize,
}
impl EraseChunker {
    pub fn new(erase_range: core::ops::Range<usize>) -> Self {
        Self {
            chunker: AlignedSegmentIter::new::<4096>(erase_range.start, erase_range.len()),
        }
    }
}
impl Iterator for EraseChunker {
    type Item = EraseChunk;

    fn next(&mut self) -> Option<Self::Item> {
        if let Some(AlignedSegment { aligned, .. }) = self.chunker.next() {
            Some(EraseChunk {
                offset: aligned.start,
            })
        } else {
            None
        }
    }
}

pub async fn test_mx25<Spi>(mut flash: Mx25<Spi>) -> Result<(), Error<Spi::Error>>
where
    Spi: SpiDevice<u8>,
    Spi::Error: embedded_hal_async::spi::Error,
{
    if false {
        let mut first_256 = [0u8; 256];
        for (i, v) in first_256.iter_mut().enumerate() {
            *v = i as u8;
        }
        flash.cmd_page_program_4b(0, &first_256).await?;
    }

    if false {
        flash.cmd_erase_sector_4b(0).await?;
    }

    defmt::info!("Status: {:?}", flash.status().await?);
    defmt::info!("Status: {:?}", flash.status().await?);
    defmt::info!("Status: {:?}", flash.status().await?);
    defmt::info!("Status: {:?}", flash.status().await?);
    defmt::info!("Status: {:?}", flash.status().await?);
    defmt::info!("Status: {:?}", flash.status().await?);
    defmt::info!("Status: {:?}", flash.status().await?);
    defmt::info!("Status: {:?}", flash.status().await?);
    defmt::info!("Status: {:?}", flash.status().await?);
    defmt::info!("Status: {:?}", flash.status().await?);
    defmt::info!("Status: {:?}", flash.status().await?);
    defmt::info!("Status: {:?}", flash.status().await?);
    defmt::info!("Status: {:?}", flash.status().await?);

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

#[cfg(test)]
mod test {
    use super::*;

    struct TestFlash {
        data: Vec<u8>,
    }
    impl TestFlash {
        pub fn new(length: usize) -> Self {
            Self {
                data: vec![0; length],
            }
        }
    }

    impl FlashMemory for TestFlash {
        type Error = Box<dyn std::error::Error>;

        async fn flash_write_page(&mut self, offset: u32, data: &[u8]) -> Result<(), Self::Error> {
            // Check out of bounds.
            if offset + data.len() > self.data.len() {
                return Err("Write out of bounds".into());
            }

            // Check if the write crosses a 256 byte page boundary.
            if offset / 256 != (offset + data.len() - 1) / 256 {
                return Err("Write crosses page boundary".into());
            }

            // Write the data to the memory.
            for (i, &b) in data.iter().enumerate() {
                self.data[offset as usize + i] = b;
            }
            Ok(())
        }

        async fn flash_erase_sector_4k(&mut self, offset: u32) -> Result<(), Self::Error> {
            if offset % 4096 != 0 {
                return Err("Sector not aligned".into());
            }
            for i in 0..4096 {
                self.data[offset as usize + i] = 0;
            }
            Ok(())
        }

        async fn flash_read(&mut self, offset: u32, data: &mut [u8]) -> Result<(), Self::Error> {
            if offset + data.len() > self.data.len() {
                return Err("Read out of bounds".into());
            }
            for (i, b) in data.iter_mut().enumerate() {
                *b = self.data[offset as usize + i];
            }
            Ok(())
        }
    }

    #[test]
    fn test_flash_write_page() -> Result<(), Box<dyn std::error::Error>> {
        let mut flash
        }

        async fn flash_erase_sector_4k(&mut self, offset: u32) -> Result<(), Self::Error> {
            todo!()
        }

        async fn flash_read(&mut self, offset: u32, data: &mut [u8]) -> Result<(), Self::Error> {
            todo!()
        }
    }

    #[test]
    fn test_aligned_segment_iter() -> Result<(), Box<dyn std::error::Error>> {
        let mut o = AlignedSegmentIter::new::<256>(0, 512).collect::<Vec<_>>();
        assert_eq!(o.len(), 2);
        assert_eq!(o[0].aligned, 0..256usize);
        assert_eq!(o[0].data, 0..256usize);
        assert_eq!(o[1].aligned, 256..512usize);
        assert_eq!(o[1].data, 256..512usize);

        let mut o = AlignedSegmentIter::new::<256>(10, 512).collect::<Vec<_>>();
        assert_eq!(o.len(), 3);
        assert_eq!(o[0].overlap, 10..256); // first segment is 0 - 256
        assert_eq!(o[0].aligned, 0..256); // first segment is 0 - 256
        assert_eq!(o[0].data, (0)..(256 - 10)); // And data is 0 to 256 - 10
        assert_eq!(o[1].overlap, 256..512);
        assert_eq!(o[1].data, 256 - 10..512 - 10);
        assert_eq!(o[2].aligned, 512..768);
        assert_eq!(o[2].overlap, 512..522);
        assert_eq!(o[2].data, (512 - 10usize)..512);

        // Consider a situation for pages, which are aligned on 256 boundaries.
        //                v data offset 253
        // data:          0 | 1 | 2 | 3 | 4 | 5 | 6 | 7
        // segment0  0..253  54  56 |
        // segment1                 257 258 259 260 261
        //
        // Since the data is 8 bytes long and starts at 253, it crossess a page boundary. This means that this iterator will
        // return two entries.
        //  - The first segment falls in aligned 0..256, with data indices 0..3, and overlapping range: 253..256
        //  - The second segment falls in aligned 256..512, with data indices 3..7, and overlapping range 256..261
        let data_length = 8;
        let data_offset = 253;
        let mut o = AlignedSegmentIter::new::<256>(data_offset, data_length).collect::<Vec<_>>();
        assert_eq!(o.len(), 2);
        assert_eq!(o[0].aligned, 0..256);
        assert_eq!(o[0].overlap, 253..256);
        assert_eq!(o[0].data, 0..3);
        assert_eq!(o[1].aligned, 256..512);
        assert_eq!(o[1].overlap, 256..261);
        assert_eq!(o[1].data, 3..8);

        Ok(())
    }
    #[test]
    fn test_program_chunker() -> Result<(), Box<dyn std::error::Error>> {
        let z = [0, 2, 3, 4, 5];
        let mut c = ProgramChunker::new(1, &z);
        let n = c.next().unwrap();
        assert_eq!(n.offset, 1);
        assert_eq!(n.data, &z);

        // Empty data should result in empty chunker.
        let mut c = ProgramChunker::new(0, &[]);
        assert!(c.next().is_none());

        let mut c = ProgramChunker::new(254, &z);
        let n = c.next().unwrap();
        assert_eq!(n.offset, 254); // Should fit across two chunks.
        assert_eq!(n.data, &z[0..2]);
        let n = c.next().unwrap();
        assert_eq!(n.offset, 256); // Should fit across two chunks.
        assert_eq!(n.data, &z[2..]);

        Ok(())
    }
    #[test]
    fn test_erase_chunker() -> Result<(), Box<dyn std::error::Error>> {
        let mut c = EraseChunker::new(0..10);
        let n = c.next().unwrap();
        assert_eq!(n.offset, 0); // first sector.

        // Empty data should result in empty chunker.
        let mut c = EraseChunker::new(0..0);
        assert!(c.next().is_none());

        let mut c = EraseChunker::new(4000..4200);
        let n = c.next().unwrap();
        assert_eq!(n.offset, 0); // Should fit across two sectors.
        let n = c.next().unwrap();
        assert_eq!(n.offset, 4096); // Should fit across two sectors.

        Ok(())
    }
}
