//! Most of this file is handling of the [`RecordManager`].
//!
//! This manages a range of sectors on the flash, it allows writing a single record there. The most recent update is
//! always used. It gracefully (well, hopefully) handles any write interruptions that may occur due to power down.
//!
//! Need to ensure that the metadata is at the end of the write, such that it is written only after the data.
//! Record can be any length, but is always consecutive.
//!
//! The first 4 bytes of the arena are not used.
//! At the end of the arena is the EndMarker. The endmarker is used when we need to wrap the data and perform clearing.
//!
//! The length and counter are stored inverted, such that partial writes result in small numbers instead of big numbers.
//!
//! On flash:
//! ```nocode
//!   Begin Marker u32, this is a throwaway entry to allow uniform handling.
//!              <----- position
//!   Length: u32,            \
//!   Increment_counter: u32, | FlashPrefix
//!   Prefix complete: u32    / prefix complete necesarry to detect power downs while the prefix is written.
//!   Data: [u8]
//!   Data_complete: u32, // FlashSuffix <- denotes the data was written in full and creates a valid record.
//!
//!  ---
//!   End marker object (2xu32).
//! ```
//!
//! When wrapping needs to happen, the following is performed:
//!  - The position of the currently valid entry is burned into the end marker.
//!    - Reinitialisation will now take the valid entry from the end marker.
//!    - The next free value is always at the start.
//!  - The sectors before the sector with valid entry is erased.
//!    - When it is necessary to write only.
//!    - If interruption happens here, the sectors before the valid entry will be erased again.
//!  - The next record is written to the front.
//!  - The end marker is written to to denote there's new data at at the front now.
//!    - Power interruption will no longer cause the front data to be erased again.
//!    - The entry stored in the end marker is no longer used.
//!  - The sectors from the entry in the end marker but not the last are erased.
//!    - We do this in two steps such that if we end up erasing the location of the entry in the end marker
//!      but not the endmarker in full, we don't get an empty interval of sectors to clear.
//!  - The end marker gets another update to denote its sector is now to be done.
//!  - The last sector gets removed, clearing the end marker.
//!
//!
use crate::static_assert_size;
use zerocopy::{FromBytes, Immutable, IntoBytes};

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
#[derive(Eq, PartialEq, defmt::Format, Copy, Clone, PartialOrd, Ord, Hash, Debug, Default)]
#[repr(u8)]
enum WrappingState {
    /// Normal happy writing.
    #[default]
    NormalWrite = 0,
    /// Valid entry address has been written to marker. Time to erase the beginning.
    ValidEntryInEnd = 1,
    /// This is a transient state internally, never ends up in the flash.
    BeginningEraseDone = 2,
    /// Data has been written to the front again completely, never ends up in the flash.
    BeginningDataWrite = 3,
    /// The end marker is burned, it should not be used to retrieve the valid entry, because the beginning has data.
    EndMarkerDestroy = 4,
    /// The end part should be erased next.
    EndErase = 5,
    /// The end erasure is done.
    EndEraseDone = 6,
}
impl WrappingState {
    pub fn advanced(&mut self) -> WrappingState {
        match self {
            WrappingState::NormalWrite => WrappingState::ValidEntryInEnd,
            WrappingState::ValidEntryInEnd => WrappingState::BeginningEraseDone,
            WrappingState::BeginningEraseDone => WrappingState::BeginningDataWrite,
            WrappingState::BeginningDataWrite => WrappingState::EndMarkerDestroy,
            WrappingState::EndMarkerDestroy => WrappingState::EndErase,
            WrappingState::EndErase => WrappingState::EndEraseDone,
            WrappingState::EndEraseDone => WrappingState::NormalWrite,
        }
    }
    pub fn is_servicable(&mut self) -> bool {
        match self {
            WrappingState::NormalWrite => false, // nothing to service, we're happily writing.
            WrappingState::ValidEntryInEnd => true,
            WrappingState::BeginningEraseDone => false, // we cleared the begining, it is ready for a write.
            WrappingState::BeginningDataWrite => true,
            WrappingState::EndMarkerDestroy => true,
            WrappingState::EndErase => true,
            WrappingState::EndEraseDone => true,
        }
    }
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

#[derive(
    PartialEq,
    Eq,
    defmt::Format,
    Copy,
    Clone,
    PartialOrd,
    Ord,
    Hash,
    Debug,
    FromBytes,
    IntoBytes,
    Immutable,
    Default,
)]
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
const DATA_COMPLETED: u32 = 0xFFFF_FF80; // mostly to easily be able to recognise it as 128
#[derive(PartialEq, Eq, defmt::Format, Copy, Clone, PartialOrd, Ord, Hash, Debug, Default)]
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
    pub fn into_completed_metadata(&self) -> Metadata {
        Metadata {
            length: self.length,
            counter: self.counter,
        }
    }
}

// We use this module to ensure without a doubt that the private member attributes of these on-flash structs aren't
// accidentally accessed
mod module_to_make_private {
    use super::*;

    const PREFIX_COMPLETE: u32 = 1337;
    #[derive(PartialEq, Eq, FromBytes, IntoBytes, Immutable, defmt::Format, Default, Debug)]
    #[repr(C)]
    pub struct FlashSuffix {
        data_complete: u32,
    }
    crate::static_assert_size!(FlashSuffix, 4);
    impl FlashSuffix {
        pub const SIZE: u32 = core::mem::size_of::<FlashSuffix>() as u32;
        pub fn is_complete(&self) -> bool {
            self.data_complete == DATA_COMPLETED
        }

        pub fn completed(prefix: &FlashPrefix) -> Self {
            Self {
                data_complete: DATA_COMPLETED,
            }
        }
    }

    #[derive(
        PartialEq, Eq, FromBytes, IntoBytes, Immutable, defmt::Format, Default, Debug, Copy, Clone,
    )]
    #[repr(C)]
    pub struct FlashPrefix {
        length: u32,
        counter: u32,
        prefix_complete: u32,
    }
    crate::static_assert_size!(FlashPrefix, 12);
    impl FlashPrefix {
        pub const SIZE: u32 = core::mem::size_of::<FlashPrefix>() as u32;
        pub fn is_complete(&self) -> bool {
            self.prefix_complete == PREFIX_COMPLETE
        }
        pub fn is_unused(&self) -> bool {
            self.length == u32::MAX && self.counter == u32::MAX && self.prefix_complete == u32::MAX
        }

        pub fn to_metadata(&self) -> Option<Metadata> {
            Metadata::from_prefix(self)
        }
    }

    /// This is what is on the flash, the 'anchor' is after the data_complete byte, such that individual entries
    /// write their length first, followed by counter. That way, if power loss happens the length is already (partially)
    /// written and we just jump over a (possibly partially) used section.
    ///
    /// todo; should we negate length? To ensure that u32, if only first byte is written doesn't result in 0x0FF_FFFF which
    /// is very large and effectively ruins all data in the arena size.
    #[derive(PartialEq, Eq, FromBytes, IntoBytes, Immutable, defmt::Format, Default, Debug)]
    #[repr(C)]
    pub struct FlashMetadata {
        pub suffix: FlashSuffix,
        pub prefix: FlashPrefix,
    }
    crate::static_assert_size!(
        FlashMetadata,
        FlashSuffix::SIZE as usize + FlashPrefix::SIZE as usize
    );
    impl FlashMetadata {
        pub const SIZE: u32 = core::mem::size_of::<FlashMetadata>() as u32;
    }

    #[derive(PartialEq, Eq, defmt::Format, Default, Debug)]
    pub struct Metadata {
        pub length: u32,
        pub counter: u32,
    }
    impl Metadata {
        pub fn from_prefix(prefix: &FlashPrefix) -> Option<Metadata> {
            if prefix.is_complete() {
                Some(Metadata {
                    length: !prefix.length,
                    counter: !prefix.counter,
                })
            } else {
                None
            }
        }
        pub fn into_prefix(&self) -> FlashPrefix {
            FlashPrefix {
                length: !self.length,
                counter: !self.counter,
                prefix_complete: PREFIX_COMPLETE,
            }
        }
    }
}
use module_to_make_private::{FlashMetadata, FlashPrefix, FlashSuffix, Metadata};

#[derive(PartialEq, Eq, FromBytes, IntoBytes, Immutable, defmt::Format, Default, Debug)]
#[repr(C)]
struct EndMarker {
    position: u32,
    // Do not change order, we must ensure that position is written BEFORE the marker.
    marker: Marker,
    _pad: [u8; 3],
}

impl EndMarker {
    pub const SIZE: usize = core::mem::size_of::<EndMarker>();
    fn holds_valid_entry(&self) -> Option<u32> {
        let marker = self.marker.to_state();
        if marker == WrappingState::ValidEntryInEnd
            || marker == WrappingState::BeginningEraseDone
            || marker == WrappingState::BeginningDataWrite
        {
            Some(self.position)
        } else {
            None
        }
    }
    pub fn destroyed_entry(&self) -> Option<u32> {
        let marker = self.marker.to_state();
        if marker == WrappingState::EndMarkerDestroy
            || marker == WrappingState::BeginningEraseDone
            || marker == WrappingState::BeginningDataWrite
        {
            Some(self.position)
        } else {
            None
        }
    }
    pub fn valid_entry(position: u32) -> Self {
        Self {
            marker: Marker::new().with_state_set(WrappingState::ValidEntryInEnd),
            position,
            ..Default::default()
        }
    }
}
static_assert_size!(EndMarker, 8);

/// A manager to store a record in flash, handling power interruptions and wear levelling.
#[derive(PartialEq, Eq, defmt::Format, Default)]
#[repr(C)]
pub struct RecordManager {
    arena_start: u32,
    arena_length: u32,
    /// The most advanced valid record that was written in full.
    valid_record: Record,
    /// The position of the next free data in the flash available for writing.
    next_free: u32,

    wrapping_state: WrappingState,
}

impl RecordManager {
    async fn initialise<F: FlashMemory>(&mut self, flash: &mut F) -> Result<(), F::Error> {
        const ITERATION_LIMIT: usize = 1024; // IW: todo; this is a bit of a hack...

        // Special case, check the end token.
        let mut end_marker: EndMarker = Default::default();
        flash
            .flash_read_into(self.writable_end(), &mut end_marker)
            .await?;
        self.wrapping_state = end_marker.marker.to_state();

        // Handle pinned valid entry.
        if let Some(valid_end_entry) = end_marker.holds_valid_entry() {
            // println!("Valid entry in end marker: {:?}", valid_end_entry);
            let mut entry_flash_metadata: FlashMetadata = Default::default();
            flash
                .flash_read_into(
                    valid_end_entry - FlashSuffix::SIZE,
                    &mut entry_flash_metadata,
                )
                .await?;
            let prefix = entry_flash_metadata.prefix;
            // Next, retrieve the suffix.
            if let Some(metadata) = prefix.to_metadata() {
                self.valid_record = Record {
                    position: valid_end_entry,
                    length: metadata.length,
                    counter: metadata.counter,
                };
                self.next_free = self.writeable_start();
            }
            // println!("metadata counter: {:?}", metadata.counter);
            /*
            if end_marker.marker.to_state() == WrappingState::ValidEntryInEnd {
                println!("Beginning erase done, trying to find relevant area");
                // Advance from the start, until we've found an empty record.
                let mut current_position = self.writeable_start();

                for _ in 0..ITERATION_LIMIT {
                    let mut current_flash: FlashMetadata = Default::default();
                    let prefix_len = FlashPrefix::SIZE;
                    let suffix_len = FlashSuffix::SIZE;
                    flash
                        .flash_read_into(current_position - suffix_len, &mut current_flash)
                        .await?;
                    let mut current: Metadata = current_flash.prefix.to_metadata().unwrap();
                    if current.length == 0 && current.counter == 0 {
                        // Empty slot located!
                        self.dirty_record = Record {
                            position: current_position,
                            length: 0,
                            counter: current.counter,
                        };
                        return Ok(());
                    }
                    current_position += current.length + FlashMetadata::SIZE;
                }
            } */
            return Ok(());
        }

        // No end marker... just do the normal thing.
        // println!("No end marker");

        let mut current_position = self.writeable_start();

        let mut prefix = FlashPrefix::default();
        let mut suffix = FlashSuffix::default();

        let mut finished_init: bool = false;

        while current_position < self.writable_end() {
            // println!("In init parser at {}", current_position);
            let mut previous_flash_metadata: FlashMetadata = Default::default();
            flash
                .flash_read_into(
                    current_position - FlashSuffix::SIZE,
                    &mut previous_flash_metadata,
                )
                .await?;

            prefix = previous_flash_metadata.prefix;
            suffix = previous_flash_metadata.suffix;

            if prefix.is_unused() {
                // Previous slot was never used. This means that previous position is where the record is.
                finished_init = true;
                // println!("prefix is unused, so slot was unused");
                break;
            }

            if !prefix.is_complete() {
                // Prefix is not complete, lets advance by the prefix and try again, this can happen with partial
                // prefix writes.
                current_position += FlashPrefix::SIZE;
                // println!("cont on  !prefix.is_complete()");
                continue;
            }

            // If we get here, prefix MUST be complete.
            let metadata = prefix.to_metadata();
            if metadata.is_none() {
                // Dunno what happened, invariant seems violated.
                break;
            }

            let metadata = metadata.unwrap(); // safe because check above.

            if metadata.length == 0 && metadata.counter == 0 {
                // Shield once more, if this is the case, this metadata is not actually used yet and we can just return.
                break;
            }

            // Prefix is fully complete, we can now retrieve the suffix (and next prefix).
            let next_suffix_prefix = current_position + FlashPrefix::SIZE + metadata.length;

            // Length and counter are real, so now we retrieve the next metadata block to see if the data was
            // actually finished.
            let mut current_flash: FlashMetadata = Default::default();
            flash
                .flash_read_into(next_suffix_prefix, &mut current_flash)
                .await?;

            // If the suffix state everything is complete, this is a valid record.
            if current_flash.suffix.is_complete() {
                // println!("New valid entry at {}", current_position);
                // Found a valid record, update the valid record data.
                self.valid_record.position = current_position;
                self.valid_record.counter = metadata.counter;
                self.valid_record.length = metadata.length;
            }

            // Advance the current position to beyond the previous record, even it was not completed in full.
            current_position += metadata.length + FlashPrefix::SIZE + FlashSuffix::SIZE;

            // Data may or may not be complete, but we did have stuff written here and as such need to advance the dirty
            // record, since there may be data up to the suffix, which we can't use.
            self.next_free = current_position;

            // If the next prefix is unused, we know we are done.
            if current_flash.prefix.is_unused() {
                // println!("current flash prefix is unused");
                finished_init = true;
                break;
            }

            // Alternatively, if the prefix is NOT complete, we can advance by its size and continue
            if !current_flash.prefix.is_complete() {
                current_position += FlashPrefix::SIZE;
                // println!(
                // "Prefix incomplete, advancing by prefix to {:?}",
                // current_position
                // );
                self.next_free = current_position;
            }
        }

        // println!("self.next_free: {}", self.next_free);
        if !finished_init {
            //panic!("never finished init");
        }
        // Dirty is at least the valid record.

        // println!("init valid_record : {:?}", self.valid_record);
        // println!("init dirty_record : {:?}", self.dirty_record);
        Ok(())
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
        z.next_free = z.writeable_start();
        z.initialise(flash).await?;
        Ok(z)
    }

    fn writable_end(&self) -> u32 {
        self.arena_start + self.arena_length - EndMarker::SIZE as u32
    }
    fn writeable_start(&self) -> u32 {
        self.arena_start + FlashSuffix::SIZE
    }

    pub fn next_record(&self, data_length: usize) -> Record {
        let length = data_length as u32;
        // println!("valid_record: {:?}", self.valid_record);
        let next_free = self.next_free;
        // println!("next_free: {:?}", next_free);
        let with_data = next_free + length + FlashMetadata::SIZE;
        let new_counter = self.valid_record.counter + 1;

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

    pub async fn service_wrapping<F: FlashMemory>(
        &mut self,
        flash: &mut F,
    ) -> Result<(), F::Error> {
        // println!("self.wrapping_state: {:?}", self.wrapping_state);
        loop {
            match self.wrapping_state {
                WrappingState::NormalWrite => {
                    let mut end_marker: EndMarker;

                    if let Some(valid_entry) = self.valid_record() {
                        end_marker = EndMarker::valid_entry(valid_entry.position as u32);
                    } else {
                        // Can't be wrapping if we don't have data yet, somehow this function got called while the arena is still
                        // emtpy, so it should be good to return.
                        return Ok(());
                    }
                    end_marker.marker = end_marker.marker.with_state_set(self.wrapping_state);
                    // We're beginning the wrapping process.
                    // First order of business is encoding the valid entry into the end marker.
                    flash
                        .flash_write(self.writable_end(), end_marker.as_bytes())
                        .await?;
                    self.wrapping_state = end_marker.marker.to_state();
                }
                WrappingState::ValidEntryInEnd => {
                    let mut end_marker: EndMarker = Default::default();
                    flash
                        .flash_read_into(self.writable_end(), &mut end_marker)
                        .await?;

                    // end_marker.marker = end_marker.marker.with_state_set(self.wrapping_state);
                    // Next, we clear as many sectors as possible.
                    let start = self.arena_start as usize;
                    let valid_entry_pos = end_marker.position as usize;
                    // We do NOT want to clear the sector that has the valid entry pos.
                    let before_valid_sector = valid_entry_pos - (valid_entry_pos % F::SECTOR_SIZE);
                    for e in EraseChunker::new(F::SECTOR_SIZE, start..before_valid_sector) {
                        flash.flash_erase_sector(e.offset as u32).await?;
                        flash.flash_flush().await?;
                    }
                    // Now that the clear is complete, we need to advance wrapping state and the end marker.
                    end_marker.marker = end_marker
                        .marker
                        .with_state_set(WrappingState::BeginningEraseDone);
                    if false {
                        flash
                            .flash_write(self.writable_end(), end_marker.as_bytes())
                            .await?;
                    }
                    self.wrapping_state = end_marker.marker.to_state();
                }
                WrappingState::BeginningEraseDone => {
                    // Okay, so the start should be good to start writing data, we advance the state, but don't actually
                    // write to the flash to ensure we just clear it again if we don't finish the write & burn of
                    // the end marker.
                    break;
                }
                WrappingState::BeginningDataWrite => {
                    // Front data was cleared, and then it was written to, so we can not destroy the end marker
                    // as the front has valid data.
                    let mut end_marker: EndMarker = Default::default();
                    flash
                        .flash_read_into(self.writable_end(), &mut end_marker)
                        .await?;
                    end_marker.marker = end_marker
                        .marker
                        .with_state_set(WrappingState::EndMarkerDestroy);
                    // println!(
                    //     "In beginning data write: {:?}",
                    //     end_marker.marker.to_state()
                    // );
                    flash
                        .flash_write(self.writable_end(), end_marker.as_bytes())
                        .await?;
                    self.wrapping_state = end_marker.marker.to_state();
                }
                WrappingState::EndMarkerDestroy => {
                    // The end marker is no longer used as we have valid data at the front.
                    // We remove everything up to the last sector.
                    let mut end_marker: EndMarker = Default::default();
                    flash
                        .flash_read_into(self.writable_end(), &mut end_marker)
                        .await?;
                    // println!(
                    //     "In EndMarkerDestroy data write: {:?}, with {:?}",
                    //     end_marker.marker.to_state(),
                    //     end_marker
                    // );
                    if let Some(entry) = end_marker.destroyed_entry() {
                        let start = entry as usize;
                        let end = (self.arena_start + self.arena_length) as usize - F::SECTOR_SIZE;
                        for e in EraseChunker::new(F::SECTOR_SIZE, start..end) {
                            flash.flash_erase_sector(e.offset as u32).await?;
                            flash.flash_flush().await?;
                        }

                        // Write the marker that denotes everything up to the last sector is removed.
                        end_marker.marker =
                            end_marker.marker.with_state_set(WrappingState::EndErase);
                        flash
                            .flash_write(self.writable_end(), end_marker.as_bytes())
                            .await?;
                        self.wrapping_state = end_marker.marker.to_state();
                    } else {
                        // Hmm, the state is that the marker is burnt, but reading it doesn't actually have a valid entry?
                        // self.wrapping_state = WrappingState::NormalWrite; // Just assume we're done?
                        // break;
                        println!("end_marker: {:?}", end_marker);
                        println!("end_marker: {:?}", end_marker.marker.to_state());
                        todo!(); // Dunno how this would happen, feels like a logic bug.
                    }
                }
                WrappingState::EndErase => {
                    // Only the last sector remains to put everything back into a clear state.
                    let start = (self.arena_start + self.arena_length) as usize - EndMarker::SIZE;
                    let end = (self.arena_start + self.arena_length) as usize;
                    for e in EraseChunker::new(F::SECTOR_SIZE, start..end) {
                        flash.flash_erase_sector(e.offset as u32).await?;
                        flash.flash_flush().await?;
                    }
                    // This wipes the end marker.
                    self.wrapping_state = WrappingState::NormalWrite;
                    break;
                }
                WrappingState::EndEraseDone => {
                    break;
                }
            }
        }
        Ok(())
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

        // If the data is wrapping... service wrapping until the front section is available for writing.
        if new_record.position < self.valid_record.position {
            self.service_wrapping(flash).await?;
        }
        // println!(" self.wrapping_state: {:?}", self.wrapping_state);
        if self.wrapping_state.is_servicable() {
            // println!("Servicifing wrapping state {:?}", self.wrapping_state);
            self.service_wrapping(flash).await?;
        }

        // println!("New record: {:?}", new_record);
        // We write the data to the next record.
        let new_metadata = new_record.into_completed_metadata();
        let prefix = new_metadata.into_prefix();
        let suffix = FlashSuffix::completed(&prefix);

        let mut position = new_record.position;
        flash.flash_write(position, prefix.as_bytes()).await?;
        position += FlashPrefix::SIZE;
        // println!("Writing data: {:?}", data);
        flash.flash_write(position, data).await?;
        position += data.len() as u32;
        flash.flash_write(position, suffix.as_bytes()).await?;

        // Write succeeded, so update its records.
        self.valid_record.position = new_record.position;
        self.valid_record.length = data.len() as u32;
        self.valid_record.counter = new_record.counter;
        self.next_free = new_record.position + data.len() as u32 + FlashMetadata::SIZE;

        // If we were in beginning erase done, we have now done beginning data write, and can wipe the remainder.
        if self.wrapping_state == WrappingState::BeginningEraseDone {
            self.wrapping_state = WrappingState::BeginningDataWrite;
            self.service_wrapping(flash).await?;
        }

        Ok(self.valid_record)
    }
    pub fn valid_record(&self) -> Option<Record> {
        if self.valid_record.counter != 0 {
            Some(self.valid_record)
        } else {
            None
        }
    }
    pub async fn record_read_into<T: zerocopy::FromBytes + zerocopy::IntoBytes, F: FlashMemory>(
        &mut self,
        flash: &mut F,
        record: &Record,
        data: &mut T,
    ) -> Result<(), F::Error> {
        //println!("Reading at {}", record.position + 8);
        flash
            .flash_read_into(record.position + FlashPrefix::SIZE, data)
            .await
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
    const DO_PRINTS: bool = false;

    #[allow(unused_macros)]
    /// Helper print macro that can be enabled or disabled.
    macro_rules! trace {
        () => (if DO_PRINTS {print!("\n");});
        ($($arg:tt)*) => {
            if DO_PRINTS {
                print!($($arg)*);
            }
        }
    }
    macro_rules! traceln {
        () => (if DO_PRINTS {println!("\n");});
        ($($arg:tt)*) => {
            if DO_PRINTS {
                println!($($arg)*);
            }
        }
    }

    struct TestFlash {
        data: Vec<u8>,

        /// The fuel this flash has, each byte modification consumes one fuel. If zero, no modification happens.
        /// if None, fuel functionality is disabled.
        fuel: Option<usize>,
    }
    impl TestFlash {
        pub fn new(length: usize) -> Self {
            Self {
                data: vec![0xff; length],
                fuel: None,
            }
        }
        pub fn fuel(&self) -> Option<usize> {
            self.fuel
        }
        pub fn set_fuel(&mut self, v: Option<usize>) {
            self.fuel = v
        }
        pub fn get_arena_u32(&self) -> (u32, u32) {
            (0, (self.data.len() as u32))
        }
    }

    #[derive(Debug, Copy, Clone, thiserror::Error)]
    enum TestFlashError {
        #[error("write crossess page boundary")]
        WriteExceedsPage,
        #[error("no fuel left to write with")]
        NoFuelLeft,
        #[error("read out of bounds at {0:?}")]
        OutOfBounds(usize),
        #[error("erase is not at a sector boundary {0:?}")]
        EraseOffsetIncorrect(usize),
    }

    impl FlashMemory for TestFlash {
        type Error = TestFlashError;
        const PAGE_SIZE: usize = 256;

        const SECTOR_SIZE: usize = 4096;

        async fn flash_write_page(&mut self, offset: u32, data: &[u8]) -> Result<(), Self::Error> {
            trace!("  Attempting to write {data:?} at {offset:?}");
            // Check out of bounds.
            if offset as usize + data.len() > self.data.len() {
                traceln!(" exceeds bounds");
                return Err(TestFlashError::OutOfBounds(offset as usize + data.len()));
            }

            // Check if the write crosses a 256 byte page boundary.
            if offset as usize / Self::PAGE_SIZE
                != ((offset as usize + data.len() - 1) / Self::PAGE_SIZE)
            {
                traceln!(" exceeds page");
                return Err(TestFlashError::WriteExceedsPage);
            }
            let c = &self.data[offset as usize..(offset as usize) + data.len()];
            trace!(" current; {:?}  ", c);

            // Write the data to the memory.
            for (i, &b) in data.iter().enumerate() {
                let current = self.data[offset as usize + i];
                if let Some(f) = self.fuel.as_mut() {
                    if *f == 0 {
                        let c = &self.data[offset as usize..(offset as usize) + data.len()];
                        traceln!(
                            " aborting at {i} global pos {}, new data is {c:?}",
                            offset as usize + i
                        );
                        return Err(TestFlashError::NoFuelLeft);
                    } else {
                        if current != b {
                            *f -= 1;
                        }
                    }
                }

                // Ugly guard against overwrites, only the end marker may see overwrites.
                if current != 0xFF && current != b && (offset as usize + i < (self.data.len() - 8))
                {
                    panic!(
                        "current is not 0xFF at {}, it was {:x}  with new value {:x}",
                        offset as usize + i,
                        current,
                        b
                    );
                }
                //self.data[offset as usize + i] = !(!current | !b);
                self.data[offset as usize + i] = current & b;
            }
            let c = &self.data[offset as usize..(offset as usize) + data.len()];
            trace!(" new; {:?}  ", c);
            traceln!(" write concluded");

            Ok(())
        }

        async fn flash_erase_sector(&mut self, offset: u32) -> Result<(), Self::Error> {
            if offset % Self::SECTOR_SIZE as u32 != 0 {
                return Err(TestFlashError::EraseOffsetIncorrect(offset as usize));
            }
            trace!("  Starting sector erase at: {:?}", offset);
            for i in 0..Self::SECTOR_SIZE {
                if let Some(f) = self.fuel.as_mut() {
                    if *f == 0 {
                        traceln!("  aborted at: {:?}", offset as usize + i);
                        return Err(TestFlashError::NoFuelLeft);
                    } else {
                        if self.data[offset as usize + i] != 0xFF {
                            *f -= 1;
                        }
                    }
                }
                self.data[offset as usize + i] = 0xFF;
            }
            traceln!("  concluded.");
            Ok(())
        }

        async fn flash_read(&mut self, offset: u32, data: &mut [u8]) -> Result<(), Self::Error> {
            if offset as usize + data.len() > self.data.len() {
                panic!();
                return Err(TestFlashError::OutOfBounds(offset as usize + data.len()));
            }
            // traceln!(
            //     "  Reading at {offset}: {:?}",
            //     &self.data[offset as usize..(offset as usize + data.len())]
            // );
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
    fn test_record_manager_stepwise() -> Result<(), Box<dyn std::error::Error>> {
        smol::block_on(async || -> Result<(), Box<dyn std::error::Error>> {
            use super::FlashMemory;
            let mut flash = TestFlash::new(TestFlash::SECTOR_SIZE * 4);

            let mut mgr =
                RecordManager::new(&mut flash, 0..((TestFlash::SECTOR_SIZE * 4) as u32)).await?;
            // assert_eq!(mgr.dirty_record().position, 4);
            // assert_eq!(mgr.dirty_record().length, 0);
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
            assert_eq!(record.position, 4 + FlashMetadata::SIZE + 4);
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
            println!("Z Reading record: {:?}", record);

            let mut mgr = RecordManager::new(&mut flash, 0..((TestFlash::SECTOR_SIZE * 4) as u32))
                .await
                .unwrap();
            let record = mgr.valid_record();
            assert_eq!(record.is_some(), true);
            let record = record.unwrap();
            println!("BIG read record: {:?}", record);
            assert_eq!(record.position, 4);
            large_data.fill(0);
            mgr.record_read_into(&mut flash, &record, &mut large_data)
                .await?;
            assert_eq!(large_data, [1u8; TestFlash::SECTOR_SIZE / 2]);

            // On the next write, we should do the remainder of the flash servicing... lets also make yet another mgr
            let mut mgr = RecordManager::new(&mut flash, 0..((TestFlash::SECTOR_SIZE * 4) as u32))
                .await
                .unwrap();
            let data: u32 = 55;
            let record = mgr.update_record(&mut flash, data.as_bytes()).await?;
            assert_eq!(record.position, 2068);

            // Verify the end of the flash is filled with 1s, becuase that means we correctly cleared it.
            assert_eq!(
                flash.data[(TestFlash::SECTOR_SIZE * 3)..]
                    .iter()
                    .all(|&x| x == 0xFF),
                true,
                "Flash should be filled with 1s at the end"
            );

            Ok(())
        }())
    }

    #[test]
    fn test_record_manager_loss_restore() -> Result<(), Box<dyn std::error::Error>> {
        smol::block_on(async || -> Result<(), Box<dyn std::error::Error>> {
            use super::FlashMemory;
            let flash_size = TestFlash::SECTOR_SIZE * 3;
            let mut flash = TestFlash::new(flash_size);
            let (start, end) = flash.get_arena_u32();

            let mut highest_seen = 0;

            // Lets test four times over...?
            for f in 0..flash_size * 4 {
                // We get this much fuel only.
                flash.set_fuel(Some(f));

                let mut current_value: Option<u64> = None;

                // Create a new manager.
                let mut mgr = RecordManager::new(&mut flash, start..end).await.unwrap();
                //traceln!("flash section: {:?}", &flash.data[0..256]);

                // Check if there is an existing value.
                if let Some(existing_record) = mgr.valid_record() {
                    // THere is an existing record, lets read it!
                    let mut v: u64 = 0;
                    mgr.record_read_into(&mut flash, &existing_record, &mut v)
                        .await?;
                    current_value = Some(v);
                }

                // Verify the existing value is equal to the current highest seen.
                let current_on_flash = current_value.unwrap_or(1);
                if highest_seen != current_on_flash
                    && (current_on_flash.saturating_sub(1) != highest_seen)
                {
                    assert!(false, "the highest seen and current on flash differ by more than expected {highest_seen} and {current_on_flash}");
                }
                highest_seen = current_on_flash;

                // Try to write a new record with value + 1
                let new_value = highest_seen + 1;
                traceln!("Writing: {:?}", new_value);
                let res = mgr.update_record(&mut flash, new_value.as_bytes()).await;
                match res {
                    Ok(new_rec) => {
                        // Write succeeded, verify the new record is equal to the new value.
                        let mut v: u64 = 0;
                        mgr.record_read_into(&mut flash, &new_rec, &mut v)
                            .await
                            .unwrap();
                        println!("new_rec: {:?}", new_rec);
                        assert_eq!(new_value, v);
                    }
                    Err(_) => {
                        // Write failed, in this case the old value should still be present.
                        let mut v: u64 = 0;
                        if let Some(old_record) = mgr.valid_record() {
                            traceln!(
                                "Reading old record, which ought to be valid: {:?}",
                                old_record
                            );
                            mgr.record_read_into(&mut flash, &old_record, &mut v)
                                .await
                                .unwrap();
                            // THe write may have gone through, despite it returning error.
                            assert!(current_on_flash == v || current_on_flash == v - 1);
                        } else {
                            // This should only happen on the first cycle.
                            assert_eq!(highest_seen, 1);
                        }
                    }
                }
            }

            Ok(())
        }())
    }

    /// This is a variation of the loss_restore test, that fuzzess the system with:
    /// Randomly running out of fuel. Reinitialising the mgr.
    /// Random data lengths.
    #[test]
    fn test_record_manager_fuzz() -> Result<(), Box<dyn std::error::Error>> {
        // RUST_BACKTRACE=1 RUST_LOG=INFO cargo t --features std --target x86_64-unknown-linux-gnu -- --nocapture test_record_manager_fu
        // RUST_BACKTRACE=1 RUST_LOG=INFO cargo t --features std --release --target x86_64-unknown-linux-gnu -- --nocapture test_record_manager_fu

        // const OUTER_ITERATIONS: usize = 100_000;
        #[cfg(not(debug_assertions))]
        const OUTER_ITERATIONS: usize = 10_000;
        #[cfg(debug_assertions)]
        const OUTER_ITERATIONS: usize = 100;

        smol::block_on(async || -> Result<(), Box<dyn std::error::Error>> {
            use rand::RngExt;
            use rand_chacha::rand_core::SeedableRng;
            use rand_chacha::ChaCha8Rng;
            let mut rng = ChaCha8Rng::seed_from_u64(1);
            use super::FlashMemory;
            // Small flash size, just two sectors to ensure we have a great many wrap arounds.
            let flash_size = TestFlash::SECTOR_SIZE * 2;
            let mut flash = TestFlash::new(flash_size);
            let (start, end) = flash.get_arena_u32();

            let mut highest_seen = 0;
            let mut highest_counter = 0;

            let mut mgr_persistence: Option<RecordManager> = None;

            // Lets test a great many times.
            let inner_iterations = 1_000;
            for z in 0..OUTER_ITERATIONS {
                if z % 1000 == 0 {
                    println!("z: {z} / {OUTER_ITERATIONS}");
                }
                // We get this much fuel only.
                flash.set_fuel(Some(rng.random_range(1..=200)));
                for f in 0..inner_iterations {
                    if mgr_persistence.is_none() {
                        traceln!("Reset!");
                        mgr_persistence =
                            Some(RecordManager::new(&mut flash, start..end).await.unwrap());
                    }
                    if flash.fuel == Some(0) {
                        // Refuel.
                        flash.set_fuel(Some(rng.random_range(1..=200)));
                    }

                    let mut current_value: Option<u64> = None;

                    // Create a new manager.
                    let mut mgr = mgr_persistence.as_mut().unwrap();
                    //traceln!("flash section: {:?}", &flash.data);

                    // Check if there is an existing value.
                    if let Some(existing_record) = mgr.valid_record() {
                        // THere is an existing record, lets read it!
                        let mut v: u64 = 0;
                        mgr.record_read_into(&mut flash, &existing_record, &mut v)
                            .await?;
                        current_value = Some(v);
                    }

                    // Verify the existing value is equal to the current highest seen.
                    let current_on_flash = current_value.unwrap_or(1);
                    if highest_seen != current_on_flash
                        && (current_on_flash.saturating_sub(1) != highest_seen)
                    {
                        assert!(false, "the highest seen and current on flash differ by more than expected {highest_seen} and {current_on_flash}");
                    }
                    highest_seen = current_on_flash;

                    // Try to write a new record with value + 1
                    let new_value = highest_seen + 1;
                    traceln!("Writing: {:?}", new_value);
                    let res = mgr.update_record(&mut flash, new_value.as_bytes()).await;
                    let mut should_drop = false;
                    match res {
                        Ok(new_rec) => {
                            assert!(
                                highest_counter < new_rec.counter,
                                "new counter must always be higher than old counter highest: {highest_counter} new {:?}", new_rec
                            );
                            highest_counter = new_rec.counter;

                            // Write succeeded, verify the new record is equal to the new value.
                            let mut v: u64 = 0;
                            mgr.record_read_into(&mut flash, &new_rec, &mut v)
                                .await
                                .unwrap();
                            let p = new_rec.position as usize;
                            assert_eq!(new_value, v);
                        }
                        Err(_) => {
                            should_drop = true;
                            // Write failed, in this case the old value should still be present.
                            let mut v: u64 = 0;
                            if let Some(old_record) = mgr.valid_record() {
                                mgr.record_read_into(&mut flash, &old_record, &mut v)
                                    .await
                                    .unwrap();
                                // THe write may have gone through, despite it returning error.
                                assert!(current_on_flash == v || current_on_flash == v - 1);
                            } else {
                                // This should only happen on the first cycle.
                                assert_eq!(highest_seen, 1);
                            }
                        }
                    }
                    if should_drop {
                        mgr_persistence = None;
                    }
                }
            }
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

        assert_eq!(WrappingState::from(236), WrappingState::EndMarkerDestroy);
    }
}
