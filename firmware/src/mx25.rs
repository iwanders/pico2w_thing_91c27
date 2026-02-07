//! Driver for Macronix mx25xxx45 family flash memory

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

#[allow(async_fn_in_trait)]
pub trait FlashMemory {
    type Error;
    /// The size of a writable page, usually 256.
    const PAGE_SIZE: usize;

    /// The size of a sector, usually 4096.
    const SECTOR_SIZE: usize;

    /// Write data to flash, up to PAGE_SIZE bytes, may not cross page boundary.
    async fn flash_write_page(&mut self, offset: u32, data: &[u8]) -> Result<(), Self::Error>;

    /// Clear a sector of SECTOR_SIZE bytes, offset is at the sector start boundary.
    async fn flash_erase_sector(&mut self, offset: u32) -> Result<(), Self::Error>;

    /// Read data from flash, arbitrary position.
    async fn flash_read(&mut self, offset: u32, data: &mut [u8]) -> Result<(), Self::Error>;

    /// Flush the pending writes. This blocks until writes or erases are completed.
    async fn flash_flush(&mut self) -> Result<(), Self::Error>;

    /// Read into a type that is convertible to a byte slice
    async fn flash_read_into<T: zerocopy::FromBytes + zerocopy::IntoBytes>(
        &mut self,
        offset: u32,
        data: &mut T,
    ) -> Result<(), Self::Error> {
        self.flash_read(offset, data.as_mut_bytes()).await
    }

    async fn flash_write(&mut self, offset: u32, data: &[u8]) -> Result<(), Self::Error> {
        let z = ProgramChunker::new(Self::PAGE_SIZE, offset as usize, data);
        for c in z {
            self.flash_write_page(c.offset as u32, c.data).await?;
            self.flash_flush().await?;
        }

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
        todo!();
    }
}

// Need to ensure that the metadata is at the end of the write, such that it is written only at the end.
// Record can be any length.
//
// On flash:
//  (Data Complete u32)
//              <----- position
//   Length: u32,
//   Increment_counter: u32,
//   Data: [u8]
//   Data_complete: u32, // mostly an u32 for lazy packing reasons.
//
// If data complete is not actually written to be 0, it must not be used and the entry is considered 'burned'.

#[derive(
    PartialEq,
    Eq,
    TryFromBytes,
    IntoBytes,
    Immutable,
    defmt::Format,
    Copy,
    Clone,
    PartialOrd,
    Ord,
    Hash,
    Debug,
)]
#[repr(C)]
pub struct Record {
    pub position: u32,
    pub length: u32,
    pub counter: u32,
}

#[derive(PartialEq, Eq, FromBytes, IntoBytes, Immutable, defmt::Format, Default)]
#[repr(C)]
struct Metadata {
    data_complete: u32,
    length: u32,
    counter: u32,
}
const DATA_COMPLETED: u32 = 0xFFFF_FFF0;

#[derive(PartialEq, Eq, TryFromBytes, IntoBytes, Immutable, defmt::Format)]
#[repr(C)]
pub struct RecordManager {
    arena_start: u32,
    arena_length: u32,
    /// The most advanced valid record that was written in full.
    valid_record: Record,
    /// The most advanced dirty record that may not have been written in full.
    dirty_record: Record,
}

impl RecordManager {
    async fn initialise<F: FlashMemory>(
        &mut self,
        flash: &mut F,
    ) -> Result<(Record, Record), F::Error> {
        const ITERATION_LIMIT: usize = 1024; // IW: todo; this is a bit of a hack...
        let mut previous_metadata: Metadata = Default::default();
        let mut current_position = self.valid_record.position - 4;
        flash
            .flash_read_into(current_position, &mut previous_metadata)
            .await?;
        let mut valid_record = Record {
            position: self.valid_record.position,
            length: 0,
            counter: 0,
        };
        let mut dirty_record = Record {
            position: self.dirty_record.position,
            length: 0,
            counter: 0,
        };
        for _ in 0..ITERATION_LIMIT {
            if previous_metadata.length == u32::MAX && previous_metadata.counter == u32::MAX {
                // Previous slot was never used.
                // This means that previous position is where the record is.
                break;
            }
            // Length and counter are real, so now we retrieve the next metadata block to see if the data was
            // actually finished.
            let mut current: Metadata = Default::default();
            flash
                .flash_read_into(
                    current_position + previous_metadata.length + 8,
                    &mut current,
                )
                .await?;
            if current.data_complete == DATA_COMPLETED {
                // Found a valid record, update the valid record data.
                println!("valid record: {:?}", valid_record);
                valid_record.position = current_position;
                valid_record.counter = previous_metadata.counter;
                valid_record.length = previous_metadata.length;
                if previous_metadata.counter > current.counter || current.counter == u32::MAX {
                    // Amazing, we found a completed data record, and the counter is higher, so we found the boundary.
                    break;
                }
            }
            // Data may or may not be complete, but we did have stuff written here and as such need to advance the dirty
            // record, since that's the last record at which data exists.
            dirty_record.position = current_position;
            dirty_record.length = previous_metadata.length;
            dirty_record.counter = previous_metadata.counter;

            current_position += previous_metadata.length + 8;
            previous_metadata = current;
        }

        println!("valid_record : {:?}", valid_record);
        println!("dirty_record : {:?}", dirty_record);
        Ok((valid_record, dirty_record))
    }
    pub async fn new<F: FlashMemory>(
        flash: &mut F,
        arena: core::ops::Range<u32>,
    ) -> Result<Self, F::Error> {
        let mut z = RecordManager {
            arena_start: arena.start,
            arena_length: arena.len() as u32,
            valid_record: Record {
                position: arena.start + 4, // one ahead of the dummy sentinel data_complete
                length: 0,
                counter: 0,
            },
            dirty_record: Record {
                position: arena.start + 4, // one ahead of the dummy sentinel data_complete
                length: 0,
                counter: 0,
            },
        };
        let recent = z.initialise(flash).await?;
        (z.valid_record, z.dirty_record) = recent;
        Ok(z)
    }
    pub async fn update_record<F: FlashMemory>(
        &mut self,
        flash: &mut F,
        data: &[u8],
    ) -> Result<(), F::Error> {
        // Figure out where this goes.
        // Basically  two options:
        //  It goes after the dirty record entry.
        //  We need to wrap around, erase sectors etc.
        //
        //
        // Update the counter.
        // Write the new data to the flash.
        //

        let next_free = if self.dirty_record.counter != 0 {
            self.dirty_record.position + self.dirty_record.length + 12
        } else {
            self.dirty_record.position
        };
        let with_data = next_free + data.len() as u32 + 12;
        let new_counter = self.dirty_record.counter + 1;
        info!("with data: {}", with_data);
        if with_data < (self.arena_start + self.arena_length) {
            // It will fit. yay.
            // Write; length, counter, data
            let length_counter: [u32; 2] = [data.len() as u32, new_counter];
            flash
                .flash_write(next_free, length_counter.as_bytes())
                .await?;
            flash.flash_write(next_free + 8, data).await?;
            let write_done: u32 = DATA_COMPLETED;
            flash
                .flash_write(next_free + 8 + data.len() as u32, write_done.as_bytes())
                .await?;
            self.valid_record.position = next_free;
            self.valid_record.length = data.len() as u32;
            self.valid_record.counter = new_counter;
            self.dirty_record = self.valid_record;
        } else {
            todo!();
        }

        Ok(())
    }
    pub fn valid_record(&self) -> Option<Record> {
        if self.valid_record.counter != 0 {
            Some(self.valid_record)
        } else {
            None
        }
    }
    pub fn dirty_record(&self) -> Record {
        self.dirty_record
    }

    pub async fn record_read_into<T: zerocopy::FromBytes + zerocopy::IntoBytes, F: FlashMemory>(
        &mut self,
        flash: &mut F,
        record: &Record,
        data: &mut T,
    ) -> Result<(), F::Error> {
        println!("Reading at {}", record.position + 8);
        flash.flash_read_into(record.position + 8, data).await
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
    fn new(segment_size: usize, start_offset: usize, data_length: usize) -> Self {
        let data_end = start_offset + data_length;
        let segment_index = start_offset / segment_size;
        AlignedSegmentIter {
            data_offset: start_offset,
            data_end,
            segment_index,
            segment_size,
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
    pub fn new(page_size: usize, offset: usize, data: &'a [u8]) -> Self {
        Self {
            chunker: AlignedSegmentIter::new(page_size, offset, data.len()),
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
    pub fn new(sector_size: usize, erase_range: core::ops::Range<usize>) -> Self {
        Self {
            chunker: AlignedSegmentIter::new(sector_size, erase_range.start, erase_range.len()),
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
                data: vec![0xff; length],
            }
        }
    }

    impl FlashMemory for TestFlash {
        type Error = Box<dyn std::error::Error>;
        const PAGE_SIZE: usize = 256;

        const SECTOR_SIZE: usize = 4096;

        async fn flash_write_page(&mut self, offset: u32, data: &[u8]) -> Result<(), Self::Error> {
            // Check out of bounds.
            if offset as usize + data.len() > self.data.len() {
                return Err("Write out of bounds".into());
            }

            // Check if the write crosses a 256 byte page boundary.
            if offset as usize / Self::PAGE_SIZE
                != ((offset as usize + data.len() - 1) / Self::PAGE_SIZE)
            {
                return Err("Write crosses page boundary".into());
            }

            // Write the data to the memory.
            for (i, &b) in data.iter().enumerate() {
                let current = self.data[offset as usize + 1];
                self.data[offset as usize + i] = !(!current | !b);
            }
            Ok(())
        }

        async fn flash_erase_sector(&mut self, offset: u32) -> Result<(), Self::Error> {
            if offset % Self::SECTOR_SIZE as u32 != 0 {
                return Err("Sector not aligned".into());
            }
            for i in 0..Self::SECTOR_SIZE {
                self.data[offset as usize + i] = 0xFF;
            }
            Ok(())
        }

        async fn flash_read(&mut self, offset: u32, data: &mut [u8]) -> Result<(), Self::Error> {
            if offset as usize + data.len() > self.data.len() {
                return Err("Read out of bounds".into());
            }
            for (i, b) in data.iter_mut().enumerate() {
                *b = self.data[offset as usize + i];
            }
            Ok(())
        }
        /// Flush the pending writes. This blocks until writes or erases are completed.
        async fn flash_flush(&mut self) -> Result<(), Self::Error> {
            Ok(())
        }
    }

    #[test]
    fn test_record_manager() -> Result<(), Box<dyn std::error::Error>> {
        smol::block_on(async || -> Result<(), Box<dyn std::error::Error>> {
            use super::FlashMemory;
            let mut flash = TestFlash::new(TestFlash::SECTOR_SIZE * 4);

            let mut mgr =
                RecordManager::new(&mut flash, 0..((TestFlash::SECTOR_SIZE * 4) as u32)).await?;
            assert_eq!(mgr.dirty_record().position, 4);
            assert_eq!(mgr.dirty_record().length, 0);
            assert_eq!(mgr.valid_record().is_none(), true);

            let data: u32 = 5;
            mgr.update_record(&mut flash, data.as_bytes()).await?;
            let record = mgr.valid_record();
            assert_eq!(record.is_some(), true);
            let record = record.unwrap();
            assert_eq!(record.counter, 1);
            assert_eq!(record.length, 4);
            assert_eq!(record.position, 4);
            println!("flash start: {:?}", &flash.data[0..64]);

            let mut read_back: u32 = 0;
            mgr.record_read_into(&mut flash, &record, &mut read_back)
                .await?;
            assert_eq!(read_back, 5);

            Ok(())
        }())
    }

    #[test]
    fn test_aligned_segment_iter() -> Result<(), Box<dyn std::error::Error>> {
        let mut o = AlignedSegmentIter::new(256, 0, 512).collect::<Vec<_>>();
        assert_eq!(o.len(), 2);
        assert_eq!(o[0].aligned, 0..256usize);
        assert_eq!(o[0].data, 0..256usize);
        assert_eq!(o[1].aligned, 256..512usize);
        assert_eq!(o[1].data, 256..512usize);

        let mut o = AlignedSegmentIter::new(256, 10, 512).collect::<Vec<_>>();
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
        let mut o = AlignedSegmentIter::new(256, data_offset, data_length).collect::<Vec<_>>();
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
        let mut c = ProgramChunker::new(256, 1, &z);
        let n = c.next().unwrap();
        assert_eq!(n.offset, 1);
        assert_eq!(n.data, &z);

        // Empty data should result in empty chunker.
        let mut c = ProgramChunker::new(256, 0, &[]);
        assert!(c.next().is_none());

        let mut c = ProgramChunker::new(256, 254, &z);
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
        let mut c = EraseChunker::new(4096, 0..10);
        let n = c.next().unwrap();
        assert_eq!(n.offset, 0); // first sector.

        // Empty data should result in empty chunker.
        let mut c = EraseChunker::new(4096, 0..0);
        assert!(c.next().is_none());

        let mut c = EraseChunker::new(4096, 4000..4200);
        let n = c.next().unwrap();
        assert_eq!(n.offset, 0); // Should fit across two sectors.
        let n = c.next().unwrap();
        assert_eq!(n.offset, 4096); // Should fit across two sectors.

        Ok(())
    }
}
