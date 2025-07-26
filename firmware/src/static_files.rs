use serde::{Deserialize, Serialize};

// Format in the data partition is
// FileInfo0
// FileInfo1...
//    if offset is zero, it is the file list terminator.
// DataFile0
// DataFile1

#[derive(Deserialize, Serialize, Debug)]
struct FileInfo<'a> {
    /// The filename of this file.
    filename: &'a str,
    /// The size of this file.
    #[serde(with = "postcard::fixint::le")]
    size: u32,
    /// The offset to the data of this file. Offset is relative to the start of the serialized data.
    #[serde(with = "postcard::fixint::le")]
    offset: u32,
}

#[derive(Debug)]
pub struct StaticFileReader<'a> {
    raw: &'a [u8],
}
impl<'a> StaticFileReader<'a> {
    pub fn new(raw: &'a [u8]) -> Self {
        Self { raw }
    }
    pub fn iter(&self) -> FileIterator {
        FileIterator {
            offset: 0,
            raw: self.raw,
        }
    }
}

#[derive(Deserialize, Serialize)]
pub struct FileEntry<'a> {
    file: FileInfo<'a>,
    raw: &'a [u8],
}
impl<'a> FileEntry<'a> {
    pub fn data(&self) -> &[u8] {
        &self.raw[self.file.offset as usize..(self.file.offset + self.file.size) as usize]
    }
    pub fn file_name(&self) -> &str {
        &self.file.filename
    }
    pub fn len(&self) -> usize {
        self.file.size as usize
    }
}

impl<'a> core::fmt::Debug for FileEntry<'a> {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        f.debug_struct("FileEntry")
            .field("file_name", &self.file_name())
            .field("len", &self.len())
            .finish()
    }
}

pub struct FileIterator<'a> {
    offset: usize,
    raw: &'a [u8],
}
impl<'a> Iterator for FileIterator<'a> {
    type Item = FileEntry<'a>;
    fn next(&mut self) -> Option<Self::Item> {
        if let Ok((file, unused)) = postcard::take_from_bytes::<FileInfo>(&self.raw[self.offset..])
        {
            if file.offset != 0 {
                self.offset += self.raw[self.offset..].len() - unused.len();
                Some(FileEntry {
                    file,
                    raw: self.raw,
                })
            } else {
                None
            }
        } else {
            None
        }
    }
}

/// Write static files function, used by the reader above.
///
///
/// Return is the used slice from the buffer.
pub fn write_static_files<'a>(
    buffer: &'a mut [u8],
    input: &[(&str, &[u8])],
) -> Result<&'a [u8], postcard::Error> {
    #[derive(Eq, PartialEq)]
    enum Stage {
        CalculateOffset,
        WriteAll,
    }
    // Okay, we have to do two passess, the first pass to determine the file table.
    // Then write the data, then update the offsets.
    let mut space_used = 0;
    let mut file_start_offset = 0;
    for stage in [Stage::CalculateOffset, Stage::WriteAll] {
        let mut file_position = 0;
        let mut write_position = 0;
        for (i, (f, d)) in input.iter().enumerate() {
            let info = FileInfo {
                filename: f,
                size: d.len() as u32,
                offset: (file_position + file_start_offset) as u32,
            };
            let written = postcard::to_slice(&info, &mut buffer[write_position..])?.len();
            write_position += written;
            // Last entry, insert sentinel.
            if i == input.len() - 1 {
                // Insert the sentinel.
                let info = FileInfo {
                    filename: "",
                    size: 0,
                    offset: 0,
                };
                let written = postcard::to_slice(&info, &mut buffer[write_position..])?.len();
                write_position += written;
            }

            // Now, we do the real pass where we know where the files are going to end up.
            if stage == Stage::WriteAll {
                // Try to write the file.
                if (file_start_offset + file_position + d.len()) >= buffer.len() {
                    // it doesn't fit!
                    return Err(postcard::Error::SerializeBufferFull);
                }
                buffer[file_start_offset + file_position
                    ..file_start_offset + file_position + d.len()]
                    .copy_from_slice(d);
                file_position += d.len();
            }
        }
        // Now, we know where files actually start, so we can use populate that.
        file_start_offset = write_position;
        // And we know how much data we've used in total.
        space_used = file_start_offset + file_position;
    }
    Ok(&buffer[0..space_used])
}

#[cfg(test)]
mod test {
    use super::*;
    #[test]
    fn test_fileinfo_stuff() -> Result<(), Box<dyn std::error::Error>> {
        use postcard::{from_bytes, to_vec};
        let mut flash: heapless::Vec<u8, 4096> = Default::default();
        flash.resize_default(1024);

        let inputs = [
            ("file_a.txt", &b"aaaaaaaaaaaaaaaaaaaaaa"[..]),
            ("file_b.txt", &b"bbbbbbbbbbbbbbbbbbbbbbbb"[..]),
        ];

        let used_slice = write_static_files(&mut flash, &inputs)?;
        let used_len = used_slice.len();
        let used_flash = &flash[0..used_len];
        println!("flash: {flash:?}");
        println!("used_flash: {used_flash:?}");

        let reader = StaticFileReader::new(&used_flash);

        for (f, expected) in reader.iter().zip(inputs.iter()) {
            println!("f: {f:?}");
            assert_eq!(f.file_name(), expected.0);
            assert_eq!(f.data(), expected.1);
        }

        Ok(())
    }
}
