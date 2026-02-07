use zerocopy::{FromBytes, Immutable, IntoBytes, TryFromBytes};

/// Trait to interact with flash memory.
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

// Need to ensure that the metadata is at the end of the write, such that it is written only at the end.
// Record can be any length.
//
// The first 4 byte and the last four bytes of the arena are special.
//  The last 4 bytes if they are not 0xFF.. hold the address of the valid record when the front gets wiped and re-used.
//  The first 4 bytes are not used to allow uniform handling of iterating to find the data.
//
// We do not wrap entries around when we reach the end of the arena.
//
// On flash:
//   Begin Marker u32.
//              <----- position
//   Length: u32,
//   Increment_counter: u32,
//   Data: [u8]
//   Data_complete: u32, // mostly an u32 for lazy packing reasons.
//
//  ---
//   End marker u32.
//
// If data complete is not actually written to be DATA_COMPLETED, it must not be used and the entry is considered 'burned'.
//
//

/// State machine for the wrapping situation.
///
/// We need some special handling for the wrap-around. If there is data to be written that requires the wrap around
/// The following events are done in sequence.
///  - The position of the currently valid entry is burned into the end marker.
///  - As many sectors as possible from the beginning are erased.
///  - The end erase done bool is set in the end marker (just the top bit)
///  - The new data is written to the beginning in the now erased area.
///  - The 'end marker is burned' bit is set in the end marker.
///  - The second half is erased and the begin marker is set to zero to indicate erasure has happened.
#[derive(Eq, PartialEq, defmt::Format, Copy, Clone, PartialOrd, Ord, Hash, Debug)]
#[repr(u8)]
enum WrappingState {
    /// Normal happy writing.
    NormalWrite = 0,
    /// Valid entry address has been written to marker. Time to erase the beginning.
    ValidEntryInEnd = 1,
    /// Erase of everything up to the valid entry at the start has been done.
    BeginningEraseDone = 2,
    /// Data has been written to the front again.
    BeginningDataWrite = 3,
    /// The end marker is burned, it should not be used to retrieve the valid entry, because the beginning has data.
    EndMarkerDestroy = 4,
    /// The end part should be erased next.
    EndErase = 5,
    /// The end erasure is done.
    EndEraseDone = 6,
}

impl From<u8> for WrappingState {
    fn from(value: u8) -> Self {
        for z in [
            WrappingState::EndEraseDone,
            WrappingState::EndErase,
            WrappingState::EndMarkerDestroy,
            WrappingState::BeginningDataWrite,
            WrappingState::BeginningEraseDone,
            WrappingState::ValidEntryInEnd,
            WrappingState::NormalWrite,
        ] {
            if value & (1 << z as u8) == 0 {
                return z;
            }
        }
        // Anything else... :<
        WrappingState::NormalWrite
    }
}

#[derive(PartialEq, Eq, defmt::Format, Copy, Clone, PartialOrd, Ord, Hash, Debug)]
#[repr(transparent)]
struct Marker(u8);
impl Marker {
    pub fn new() -> Self {
        Self(0xFF)
    }
    pub fn with_state_set(&self, state: WrappingState) -> Self {
        Self(self.0 & !(1 << state as u8))
    }
    pub fn to_state(&self) -> WrappingState {
        self.0.into()
    }
}
#[cfg(test)]
mod marker_test {
    use super::*;
    #[test]
    fn test_marker_test() {
        let m = Marker::new();
        assert_eq!(m.0, 0b1111_1111);
        let z = m.with_state_set(WrappingState::EndErase);
        assert_eq!(z.0, 0b1101_1111);
        assert_eq!(WrappingState::from(z.0), WrappingState::EndErase);
        let e = z.with_state_set(WrappingState::EndEraseDone);
        assert_eq!(e.0, 0b1001_1111);
        assert_eq!(WrappingState::from(e.0), WrappingState::EndEraseDone);
        // Seems to work, we can burn individual bits to advance the state.
        let v = m.with_state_set(WrappingState::ValidEntryInEnd);
        assert_eq!(v.0, 0b1111_1101);
        assert_eq!(WrappingState::from(v.0), WrappingState::ValidEntryInEnd);
    }
}

const DATA_COMPLETED: u32 = 0xFFFF_FFF0;
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
    Default,
)]
#[repr(C)]
pub struct Record {
    pub position: u32,
    pub length: u32,
    pub counter: u32,
}
impl Record {
    pub fn to_range(&self) -> core::ops::Range<usize> {
        (self.position as usize)..((self.position + self.length) as usize)
    }
}

#[derive(PartialEq, Eq, FromBytes, IntoBytes, Immutable, defmt::Format, Default, Debug)]
#[repr(C)]
struct Metadata {
    data_complete: u32,
    length: u32,
    counter: u32,
}

#[derive(PartialEq, Eq, TryFromBytes, IntoBytes, Immutable, defmt::Format, Default)]
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
        let mut current_position = self.valid_record.position;
        println!("current_position: {:?}", current_position);
        flash
            .flash_read_into(current_position - 4, &mut previous_metadata)
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
            println!("previous_metadata: {:?}", previous_metadata);
            if previous_metadata.length == u32::MAX && previous_metadata.counter == u32::MAX {
                // Previous slot was never used.
                // This means that previous position is where the record is.
                break;
            }
            // Length and counter are real, so now we retrieve the next metadata block to see if the data was
            // actually finished.
            let mut current: Metadata = Default::default();
            println!(
                "current_position: {:?} length: {}",
                current_position, previous_metadata.length
            );
            flash
                .flash_read_into(
                    current_position - 4 + previous_metadata.length + 12,
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

            current_position += previous_metadata.length + 12;
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
            ..Default::default()
        };
        z.valid_record.position = z.writeable_start();
        z.dirty_record.position = z.writeable_start();
        let recent = z.initialise(flash).await?;
        (z.valid_record, z.dirty_record) = recent;
        Ok(z)
    }

    fn writable_end(&self) -> u32 {
        self.arena_start + self.arena_length - 4
    }
    fn writeable_start(&self) -> u32 {
        self.arena_start + 4
    }

    pub fn next_record(&self, data_length: usize) -> Record {
        let length = data_length as u32;
        let next_free = if self.dirty_record.counter != 0 {
            self.dirty_record.position + self.dirty_record.length + 12
        } else {
            self.dirty_record.position
        };
        let with_data = next_free + length + 12;
        let new_counter = self.dirty_record.counter + 1;

        let will_fit = with_data < self.writable_end();
        if will_fit {
            Record {
                position: next_free,
                length,
                counter: new_counter,
            }
        } else {
            Record {
                position: self.writeable_start(),
                length,
                counter: new_counter,
            }
        }
    }

    pub async fn update_record<F: FlashMemory>(
        &mut self,
        flash: &mut F,
        data: &[u8],
    ) -> Result<Record, F::Error> {
        // Figure out where this goes.
        // Basically  two options:
        //  It goes after the dirty record entry.
        //  We need to wrap around, erase sectors etc.
        //
        //
        // Update the counter.
        // Write the new data to the flash.

        let new_record = self.next_record(data.len());

        if new_record.position < self.valid_record.position {
            // We're wrapping, burn the end marker.
            // Wipe from the start to ensure this data can fit.
            flash
                .flash_write(self.writable_end(), self.valid_record.position.as_bytes())
                .await?;

            // This here leaves a bit of a problem... we don't actually check if the current valid record overlaps
            // with the section we have to clear, if that happens... we can't handle this situation since we need to
            // wipe the current record on disk... this should not happen as long as the values are < half arena
            // which seems exceedingly rare..

            // Clear the front sectors.
            for c in EraseChunker::new(F::SECTOR_SIZE, new_record.to_range()) {
                flash.flash_erase_sector(c.offset as u32).await?;
                flash.flash_flush().await?;
            }

            // Now that the front sectors are empty, we can just continue with the write to the first sector.
        }

        // It will fit. yay.
        // Write; length, counter, data
        let length_counter: [u32; 2] = [data.len() as u32, new_record.counter];
        flash
            .flash_write(new_record.position, length_counter.as_bytes())
            .await?;
        flash.flash_write(new_record.position + 8, data).await?;
        let write_done: u32 = DATA_COMPLETED;
        flash
            .flash_write(
                new_record.position + 8 + data.len() as u32,
                write_done.as_bytes(),
            )
            .await?;
        self.valid_record.position = new_record.position;
        self.valid_record.length = data.len() as u32;
        self.valid_record.counter = new_record.counter;
        self.dirty_record = self.valid_record;

        Ok(self.valid_record)
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

#[derive(Debug)]
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
#[derive(Debug)]
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
                let current = self.data[offset as usize + i];
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

            // Write another data entry.
            let data: u32 = 7;
            mgr.update_record(&mut flash, data.as_bytes()).await?;
            let record = mgr.valid_record();
            assert_eq!(record.is_some(), true);
            let record = record.unwrap();
            assert_eq!(record.counter, 2);
            assert_eq!(record.length, 4);
            assert_eq!(record.position, 4 + 12 + 4);
            println!("flash start: {:?}", &flash.data[0..64]);

            let mut read_back: u32 = 0;
            mgr.record_read_into(&mut flash, &record, &mut read_back)
                .await?;
            assert_eq!(read_back, 7);

            // If we recreate the manager, we should be able to retrieve the record.
            let mut mgr =
                RecordManager::new(&mut flash, 0..((TestFlash::SECTOR_SIZE * 4) as u32)).await?;
            let record = mgr.valid_record();
            assert_eq!(record.is_some(), true);
            let record = record.unwrap();
            let mut read_back: u32 = 0;
            mgr.record_read_into(&mut flash, &record, &mut read_back)
                .await?;
            assert_eq!(read_back, 7);

            // Next... Lets fill the flash for a large amount..
            let mut record = record;
            let mut counter = 0u64;
            while record.position
                < (3 * TestFlash::SECTOR_SIZE as u32 + (3 * TestFlash::SECTOR_SIZE as u32 / 4))
            {
                counter += 1;
                record = mgr.update_record(&mut flash, counter.as_bytes()).await?;
            }

            // There's now less than 1/4 * TestFlash::SECTOR_SIZE left... so adding a large record will wrap around.
            let mut large_data = [1u8; TestFlash::SECTOR_SIZE / 2];
            record = mgr.update_record(&mut flash, &large_data).await?;

            let mut mgr =
                RecordManager::new(&mut flash, 0..((TestFlash::SECTOR_SIZE * 4) as u32)).await?;
            let record = mgr.valid_record();
            assert_eq!(record.is_some(), true);
            let record = record.unwrap();
            large_data.fill(0);
            mgr.record_read_into(&mut flash, &record, &mut large_data)
                .await?;
            assert_eq!(large_data, [1u8; TestFlash::SECTOR_SIZE / 2]);

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
