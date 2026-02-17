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
//!  ---
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

    fn flash_sector_size(&self) -> usize {
        Self::SECTOR_SIZE
    }
    fn flash_page_size(&self) -> usize {
        Self::PAGE_SIZE
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
    /// The end part should be erased next, this is never written to the flash.
    EndErase = 5,
}
impl WrappingState {
    pub fn is_servicable(&mut self) -> bool {
        match self {
            WrappingState::NormalWrite => false, // nothing to service, we're happily writing.
            WrappingState::ValidEntryInEnd => true,
            WrappingState::BeginningEraseDone => false, // we cleared the begining, it is ready for a write.
            WrappingState::BeginningDataWrite => true,
            WrappingState::EndMarkerDestroy => true,
            WrappingState::EndErase => true,
        }
    }
}

impl From<u8> for WrappingState {
    fn from(value: u8) -> Self {
        for z in [
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

#[derive(defmt::Format, Copy, Clone, PartialEq, Eq, Debug, FromBytes, IntoBytes, Immutable)]
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
impl Default for Marker {
    fn default() -> Self {
        Self::new()
    }
}

/// A record in flash, this struct is returned by [`RecordManager::valid_record`]
///
/// Fields are public, but in general this is not constructed (or modified) directly, they're public mostly for
/// inspection.
#[derive(PartialEq, Eq, defmt::Format, Copy, Clone, PartialOrd, Ord, Hash, Debug, Default)]
#[repr(C)]
pub struct Record {
    /// Absolute position of the record in the flash.
    pub position: u32,
    /// Length of the data payload, not including the prefix or suffix.
    pub length: u32,
    /// The counter value, this is an increasing value as records get written.
    pub counter: u32,
}
impl Record {
    /// Converts the position and length to a range of usize.
    pub fn to_range(&self) -> core::ops::Range<usize> {
        (self.position as usize)..((self.position + self.length) as usize)
    }

    /// Convert this into the metadata for this record.
    pub fn into_completed_metadata(&self) -> Metadata {
        Metadata {
            length: self.length,
            counter: self.counter,
        }
    }

    /// Returns the length of this record.
    pub fn length(&self) -> usize {
        self.length as usize
    }
}

// We use this module to ensure without a doubt that the private member attributes of these on-flash structs aren't
// accidentally accessed, it prevents easy errors with the whole bit-flipped count and length.
mod module_to_make_private {
    use super::*;

    const DATA_COMPLETED: u32 = 0xFFFF_FF80; // Single byte change and visible as 128 in the entries.
    const PREFIX_COMPLETE: u32 = 0xFFFF_FFF1; // F1 PreF1X, 241 in decimal, also single byte change

    /// The suffix for a record in the flash memory., which goes after the record payload / data.
    #[derive(PartialEq, Eq, FromBytes, IntoBytes, Immutable, defmt::Format, Default, Debug)]
    #[repr(C)]
    pub struct FlashSuffix {
        data_complete: u32,
    }
    crate::static_assert_size!(FlashSuffix, 4);
    impl FlashSuffix {
        pub const SIZE: u32 = core::mem::size_of::<FlashSuffix>() as u32;

        /// Returns whether this suffix is marking data as completed.
        pub fn is_complete(&self) -> bool {
            self.data_complete == DATA_COMPLETED
        }

        /// Create a completed suffix given the provided prefix.
        ///
        /// Currently doesn't actually use the prefix, but this made sense... >_<
        pub fn completed(_prefix: &FlashPrefix) -> Self {
            Self {
                data_complete: DATA_COMPLETED,
            }
        }
    }

    /// The prefix for a record in the flash memory.
    #[derive(
        PartialEq, Eq, FromBytes, IntoBytes, Immutable, defmt::Format, Default, Debug, Copy, Clone,
    )]
    #[repr(C)]
    pub struct FlashPrefix {
        /// Length, value is stored inverted, such that the default bytes of 0xFFFF_FFFF result in zero, and partial
        /// writes result in partial loss of the data, but given that we can never write a length that exceeds the
        /// arena, we can't actually lose too much data in most cases.
        length: u32,
        /// Counter of this record, highest record is the most recent one, also written inverted.
        counter: u32,
        /// Prefix complete is just a sentinel value, it is not inverted but just check against [`PREFIX_COMPLETE`].
        prefix_complete: u32,
    }
    crate::static_assert_size!(FlashPrefix, 12);
    impl FlashPrefix {
        pub const SIZE: u32 = core::mem::size_of::<FlashPrefix>() as u32;

        /// Is this prefix actually complete? Was the prefix complete sentinel written?
        pub fn is_complete(&self) -> bool {
            self.prefix_complete == PREFIX_COMPLETE
        }

        /// If this prefix is completely made up of 0xFF bytes, it is unused.
        pub fn is_unused(&self) -> bool {
            self.length == u32::MAX && self.counter == u32::MAX && self.prefix_complete == u32::MAX
        }

        /// Convert the prefix to the record metadata, this returns a None if the prefix was not complete.
        pub fn to_metadata(&self) -> Option<Metadata> {
            Metadata::from_prefix(self)
        }
    }

    /// This is the combined suffix, prefix pair, which in general we retrieve both at the same time.
    /// This is what is on the flash, the 'anchor' is after the suffix byte, such that individual entries
    /// write their length first, followed by counter. That way, if power loss happens the length is already (partially)
    /// written and we just jump over a (possibly partially) used section.
    ///
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

    /// Record metadata.
    ///
    /// Think of this as the non-flash prefix.
    #[derive(PartialEq, Eq, defmt::Format, Default, Debug)]
    pub struct Metadata {
        pub length: u32,
        pub counter: u32,
    }
    impl Metadata {
        /// Create a metadata from the prefix, only if the prefix is actually complete.
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

        /// Convert the metadata info a complete prefix.
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

/// End marker at the far end of the data to handle wrap around.
#[derive(PartialEq, Eq, FromBytes, IntoBytes, Immutable, defmt::Format, Default, Debug)]
#[repr(C)]
struct EndMarker {
    /// The position in the flash at which the valid record can be found while we don't seek the valid record from the
    /// start of the arena.
    position: u32,
    // Do not change order, we must ensure that position is written BEFORE the marker.
    marker: Marker,
    _pad: [u8; 3],
}
// Enforce the order of the position and marker is not changed without reading the comment.
const _: [(); 1] = [(); (core::mem::offset_of!(EndMarker, position)
    < core::mem::offset_of!(EndMarker, marker)) as usize];

impl EndMarker {
    pub const SIZE: usize = core::mem::size_of::<EndMarker>();

    /// Returns the address of a valid entry.
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

    /// Returns the address, even if the marker is destroyed, this is necessary to calculate the section that is
    /// still to be erased.
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

    /// Creates a valid entry endmarker with the provided position.
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
///
/// There is always only a single most recent record. The record can be arbitrary (and varying) length between subsequent
/// calls to new_record.
///
/// The record manager should get an arena of at least two sectors and the arena should be aligned on the sector size.
/// To be able to wrap around, the new record must fit inside the sectors not overlapping with the currently valid
/// record.
#[derive(PartialEq, Eq, defmt::Format, Default)]
#[repr(C)]
pub struct RecordManager {
    /// Start of the region of data in the flash we own.
    arena_start: u32,
    /// Length of the data on flash we own.
    arena_length: u32,
    /// The most advanced valid record that was written in full.
    valid_record: Record,
    /// The position of the next free data in the flash available for writing.
    next_free: u32,

    /// Internal variable to keep track of the wrapping state.
    wrapping_state: WrappingState,
}

impl RecordManager {
    /// Create a new record manager.
    ///
    /// Arena MUST be length equal to integer times sector size, it must start on a sector boundary and it must be
    /// at least two sectors in size.
    pub async fn new<F: FlashMemory>(
        flash: &mut F,
        arena: core::ops::Range<u32>,
    ) -> Result<Self, F::Error> {
        // create the struct.
        let mut z = RecordManager {
            arena_start: arena.start,
            arena_length: arena.len() as u32,
            ..Default::default()
        };
        // Populate the remainder of the functions with methods.
        z.valid_record.position = z.writeable_start();
        z.next_free = z.writeable_start();
        // Seek to the current valid record.
        z.initialise(flash).await?;
        Ok(z)
    }

    /// Internal method to actually iterate over the flash and correctly set the next free and valid record values.
    async fn initialise<F: FlashMemory>(&mut self, flash: &mut F) -> Result<(), F::Error> {
        // First, handle the special case, check the end token, because if that's present we can't iterate from the
        // front as that may have already been erased.
        let mut end_marker: EndMarker = Default::default();
        flash
            .flash_read_into(self.writable_end(), &mut end_marker)
            .await?;
        self.wrapping_state = end_marker.marker.to_state();

        // Handle pinned valid entry in the end marker.
        if let Some(valid_end_entry) = end_marker.holds_valid_entry() {
            // println!("Valid entry in end marker: {:?}", valid_end_entry);
            let mut entry_flash_metadata: FlashMetadata = Default::default();
            flash
                .flash_read_into(
                    valid_end_entry - FlashSuffix::SIZE,
                    &mut entry_flash_metadata,
                )
                .await?;

            // The metadata prefix should always be good since the end marker is only written when the prefix has
            // actually been written in full.
            if let Some(metadata) = entry_flash_metadata.prefix.to_metadata() {
                self.valid_record = Record {
                    position: valid_end_entry,
                    length: metadata.length,
                    counter: metadata.counter,
                };
                // If there is an entry in the end marker, the next write will always be at the start of the range.
                self.next_free = self.writeable_start();
            }
            return Ok(());
        }

        // No end marker... just do the normal thing.
        // println!("No end marker");
        let mut current_position = self.writeable_start();

        // Since we need to retrieve the suffix of payload, we are also likely to retrieve the next prefix.
        // This is not always the case though, only if the prefix is not invalid, so we keep track of the location
        // of the prefix that's stored.
        let mut prefix_location: Option<u32> = None;
        let mut prefix = FlashPrefix::default();

        while current_position < self.writable_end() {
            // Check if the prefix was already retrieved with the suffix of the previous record.
            let current_read_location = current_position;
            let prefix_in_cache = prefix_location
                .map(|z| z == current_read_location)
                .unwrap_or(false);

            if !prefix_in_cache {
                // println!("In init parser at {}", current_position);
                let mut previous_metadata: FlashPrefix = Default::default();
                flash
                    .flash_read_into(current_read_location, &mut previous_metadata)
                    .await?;
                //println!("Reading at {:?}", current_position - FlashSuffix::SIZE);
                prefix_location = None;
                prefix = previous_metadata;
            }

            if prefix.is_unused() {
                // Previous slot was never used. This means that previous position is where the record is.
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

            // If we get here, prefix MUST be complete, so to_metadata will return true.
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
            // actually finished. We also retrieve the next prefix here, and if usable skip over reading it separately.
            let mut current_flash: FlashMetadata = Default::default();
            flash
                .flash_read_into(next_suffix_prefix, &mut current_flash)
                .await?;
            //println!("Reading at {:?}", next_suffix_prefix);

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
                // Next slot is unused, so we can break out of the loop.
                break;
            }

            // Alternatively, if the prefix is NOT complete, we can advance by its size and continue
            if !current_flash.prefix.is_complete() {
                current_position += FlashPrefix::SIZE;
                // println!(
                // "Prefix incomplete, advancing by prefix to {:?}",
                // current_position
                // );
            } else {
                // We actually retrieved the next prefix location when we read the suffix.
                prefix_location = Some(current_position);
                prefix = current_flash.prefix;
            }
        }
        self.next_free = current_position;

        Ok(())
    }

    /// The end of the writeable data available for records.
    fn writable_end(&self) -> u32 {
        self.arena_start + self.arena_length - EndMarker::SIZE as u32
    }

    /// The start of the writable data, historically this skipped a sentinel at the start.
    fn writeable_start(&self) -> u32 {
        self.arena_start
    }

    /// Determine the next record location, determining if wrapping needs to happen.
    fn next_record(&self, data_length: usize) -> Record {
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

    /// Remaining data before the wrapping happens.
    pub fn available_before_wrap(&self) -> usize {
        (self.writable_end() - self.next_free) as usize
    }

    /// Return whether or not the next write of this length will result in the data wrapping.
    pub fn write_will_wrap(&self, length: usize) -> bool {
        let new_record = self.next_record(length);

        new_record.position < self.valid_record.position
    }

    /// Worker function to handle the wrapping around.
    async fn service_wrapping<F: FlashMemory>(&mut self, flash: &mut F) -> Result<(), F::Error> {
        // println!("self.wrapping_state: {:?}", self.wrapping_state);
        loop {
            // This is a loop, but it's very much bounded in two sections:
            // Normal to BeginningEraseDone, which writes the end marker, erases the front sector.
            // Then we drop out of the function for data to be written.
            // Then we are resumed from BeginningDataWrite to EndErase, which removes the remaining populated sectors
            // after new data is written to the front.
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
                        defmt::error!("end_marker: {:?}", end_marker);
                        defmt::error!("end_marker state: {:?}", end_marker.marker.to_state());
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
            }
        }
        Ok(())
    }

    /// Add a new record holding the provided data.
    pub async fn new_record<F: FlashMemory>(
        &mut self,
        flash: &mut F,
        data: &[u8],
    ) -> Result<Record, F::Error> {
        // Get the proposed location.
        let new_record = self.next_record(data.len());

        // If the data is wrapping... service wrapping until the front section is available for writing.
        if new_record.position < self.valid_record.position {
            self.service_wrapping(flash).await?;
        }

        // And if the wrapping state is servicable, also service it, this is here to ensure that we handle situations
        // where data was written to the front, but the data was not yet erased in the end.
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

        // Write succeeded, so update our internal records.
        self.valid_record.position = new_record.position;
        self.valid_record.length = data.len() as u32;
        self.valid_record.counter = new_record.counter;
        // And advance the next free position.
        self.next_free = new_record.position + data.len() as u32 + FlashMetadata::SIZE;

        // If we were in beginning erase done, we have now done beginning data write, and can wipe the remainder.
        // Technically this is also handled by the service at the top, but it is nicer to do the whole wrapping logic
        // in a single invocation to this function.
        if self.wrapping_state == WrappingState::BeginningEraseDone {
            self.wrapping_state = WrappingState::BeginningDataWrite;
            self.service_wrapping(flash).await?;
        }

        Ok(self.valid_record)
    }

    /// Programs the specified data to the specified record. This can only convert bits from 1 to 0.
    /// It does not do any checking for lengths or whether the record position is in our arena.
    pub async fn program_record<F: FlashMemory>(
        &mut self,
        flash: &mut F,
        record: &Record,
        data: &[u8],
    ) -> Result<(), F::Error> {
        flash
            .flash_write(record.position + FlashPrefix::SIZE, data)
            .await
    }

    /// Returns the valid record, if any.
    pub fn valid_record(&self) -> Option<Record> {
        if self.valid_record.counter != 0 {
            Some(self.valid_record)
        } else {
            None
        }
    }

    /// Read the provided record into a zerocopy value.
    pub async fn record_read_into<T: zerocopy::FromBytes + zerocopy::IntoBytes, F: FlashMemory>(
        &mut self,
        flash: &mut F,
        record: &Record,
        data: &mut T,
    ) -> Result<(), F::Error> {
        self.record_read(flash, record, data.as_mut_bytes()).await
    }

    /// Read the provided record into a value, starting from offset.
    pub async fn record_read_offset_into<
        T: zerocopy::FromBytes + zerocopy::IntoBytes,
        F: FlashMemory,
    >(
        &mut self,
        flash: &mut F,
        record: &Record,
        offset: usize,
        data: &mut T,
    ) -> Result<(), F::Error> {
        self.record_read_offset(flash, record, offset, data.as_mut_bytes())
            .await
    }

    /// Read the provided record into bytes
    pub async fn record_read<F: FlashMemory>(
        &mut self,
        flash: &mut F,
        record: &Record,
        data: &mut [u8],
    ) -> Result<(), F::Error> {
        flash
            .flash_read(record.position + FlashPrefix::SIZE, data)
            .await
    }

    /// Read the provided record into bytes, starting reading at the provided offset.
    pub async fn record_read_offset<F: FlashMemory>(
        &mut self,
        flash: &mut F,
        record: &Record,
        offset: usize,
        data: &mut [u8],
    ) -> Result<(), F::Error> {
        flash
            .flash_read(record.position + FlashPrefix::SIZE + offset as u32, data)
            .await
    }

    /// Performs a full erase of the arena owned by this manager.
    ///
    /// This is a destructive operation that will erase all data in the arena. It should only be used when you are sure
    /// that no data needs to be preserved.
    pub async fn erase<F: FlashMemory>(&mut self, flash: &mut F) -> Result<(), F::Error> {
        let start = self.arena_start as usize;
        let end = (self.arena_start + self.arena_length) as usize;
        for e in EraseChunker::new(F::SECTOR_SIZE, start..end) {
            flash.flash_erase_sector(e.offset as u32).await?;
            flash.flash_flush().await?;
        }

        Ok(())
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

/// A chunker specifically for a data slice at a position, in segments of 256, where the start offset need not align on
/// a boundary, the generated segments always do.
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

    /// Test flash that holds a consecutive block of data.
    struct TestFlash {
        data: Vec<u8>,

        /// The fuel this flash has, each byte modification consumes one fuel. If zero, no modification will happen.
        /// If None, fuel functionality is disabled.
        fuel: Option<usize>,

        /// An offset applied to all indices to map from a virtual address to the address in the data vector.
        start_address: usize,
    }
    impl TestFlash {
        pub fn new(length: usize) -> Self {
            Self {
                data: vec![0xff; length],
                fuel: None,
                start_address: 0,
            }
        }
        pub fn fuel(&self) -> Option<usize> {
            self.fuel
        }
        pub fn set_fuel(&mut self, v: Option<usize>) {
            self.fuel = v
        }
        pub fn get_arena_u32(&self) -> (u32, u32) {
            (
                self.start_address as u32,
                self.start_address as u32 + (self.data.len() as u32),
            )
        }

        pub fn set_start_address(&mut self, address: usize) {
            self.start_address = address;
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
            let offset = offset - self.start_address as u32;
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
            let offset = offset - self.start_address as u32;
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
            let offset = offset - self.start_address as u32;
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
            let start = TestFlash::SECTOR_SIZE * 7;
            // let start = 0;
            let end = start + TestFlash::SECTOR_SIZE * 4;
            flash.set_start_address(start);

            let mut mgr = RecordManager::new(&mut flash, (start as u32)..(end as u32)).await?;
            // assert_eq!(mgr.dirty_record().position, 4);
            // assert_eq!(mgr.dirty_record().length, 0);
            assert_eq!(mgr.valid_record().is_none(), true);

            let data: u32 = 5;
            mgr.new_record(&mut flash, data.as_bytes()).await?;
            let record = mgr.valid_record();
            assert_eq!(record.is_some(), true);
            let record = record.unwrap();
            assert_eq!(record.counter, 1);
            assert_eq!(record.length, 4);
            assert_eq!(record.position, 0 + start as u32);
            println!("flash start: {:?}", &flash.data[0..64]);

            let mut read_back: u32 = 0;
            mgr.record_read_into(&mut flash, &record, &mut read_back)
                .await?;
            assert_eq!(read_back, 5);

            // Write another data entry.
            let data: u32 = 7;
            mgr.new_record(&mut flash, data.as_bytes()).await?;
            let record = mgr.valid_record();
            assert_eq!(record.is_some(), true);
            let record = record.unwrap();
            assert_eq!(record.counter, 2);
            assert_eq!(record.length, 4);
            assert_eq!(record.position, FlashMetadata::SIZE + 4 + start as u32);
            println!("flash start: {:?}", &flash.data[0..64]);

            let mut read_back: u32 = 0;
            mgr.record_read_into(&mut flash, &record, &mut read_back)
                .await?;
            assert_eq!(read_back, 7);

            // If we recreate the manager, we should be able to retrieve the record.
            let mut mgr = RecordManager::new(&mut flash, (start as u32)..(end as u32)).await?;
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
            while (record.position - start as u32)
                < (3 * TestFlash::SECTOR_SIZE as u32 + (3 * TestFlash::SECTOR_SIZE as u32 / 4))
            {
                counter += 1;
                record = mgr.new_record(&mut flash, counter.as_bytes()).await?;
            }

            // There's now less than 1/4 * TestFlash::SECTOR_SIZE left... so adding a large record will wrap around.
            let mut large_data = [1u8; TestFlash::SECTOR_SIZE / 2];
            record = mgr.new_record(&mut flash, &large_data).await?;
            println!("Z Reading record: {:?}", record);

            let mut mgr = RecordManager::new(&mut flash, (start as u32)..(end as u32))
                .await
                .unwrap();
            let record = mgr.valid_record();
            assert_eq!(record.is_some(), true);
            let record = record.unwrap();
            println!("BIG read record: {:?}", record);
            assert_eq!(record.position, 0 + start as u32);
            large_data.fill(0);
            mgr.record_read_into(&mut flash, &record, &mut large_data)
                .await?;
            assert_eq!(large_data, [1u8; TestFlash::SECTOR_SIZE / 2]);

            // On the next write, we should do the remainder of the flash servicing... lets also make yet another mgr
            let mut mgr = RecordManager::new(&mut flash, (start as u32)..(end as u32))
                .await
                .unwrap();
            let data: u32 = 55;
            let record = mgr.new_record(&mut flash, data.as_bytes()).await?;
            assert_eq!(record.position, 2064 + start as u32);

            // Verify the end of the flash is filled with 1s, becuase that means we correctly cleared it.
            assert_eq!(
                flash.data[(TestFlash::SECTOR_SIZE * 3)..]
                    .iter()
                    .all(|&x| x == 0xFF),
                true,
                "Flash should be filled with 1s at the end"
            );

            // Do a full erase.
            mgr.erase(&mut flash).await?;
            assert_eq!(flash.data.iter().all(|&x| x == 0xFF), true,);

            let mut mgr = RecordManager::new(&mut flash, (start as u32)..(end as u32))
                .await
                .unwrap();
            assert_eq!(mgr.valid_record().is_none(), true);

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
                let res = mgr.new_record(&mut flash, new_value.as_bytes()).await;
                match res {
                    Ok(new_rec) => {
                        // Write succeeded, verify the new record is equal to the new value.
                        let mut v: u64 = 0;
                        mgr.record_read_into(&mut flash, &new_rec, &mut v)
                            .await
                            .unwrap();
                        traceln!("new_rec: {:?}", new_rec);
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
    /// Randomly running out of fuel, which also reinitialising the mgr.
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
                    let res = mgr.new_record(&mut flash, new_value.as_bytes()).await;
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

        // Seems to work, we can burn individual bits to advance the state.
        let v = m.with_state_set(WrappingState::ValidEntryInEnd);
        assert_eq!(v.0, 0b1111_1101);
        assert_eq!(WrappingState::from(v.0), WrappingState::ValidEntryInEnd);

        assert_eq!(WrappingState::from(236), WrappingState::EndMarkerDestroy);
    }
}
