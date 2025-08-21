// For interpreting the TLV wrapper itself
use zerocopy::{FromBytes, Immutable, IntoBytes, KnownLayout, TryFromBytes};

#[derive(Debug, Copy, Clone)]
pub enum TLVError {
    NotEnoughData,
    MissingEntry,
    UnexpectedValue,
}

impl From<TLVError> for trouble_host::Error {
    fn from(e: TLVError) -> trouble_host::Error {
        match e {
            TLVError::NotEnoughData => trouble_host::Error::OutOfMemory,
            TLVError::MissingEntry => trouble_host::Error::InvalidValue,
            TLVError::UnexpectedValue => trouble_host::Error::InvalidValue,
        }
    }
}

/// Reader iterator for TLV sequences;
/// Yielding entries that hold:
///   type_id: u8,
///   length: u8,
///   data: [u8;length]
pub struct TLVReader<'a> {
    buffer: &'a [u8],
    position: usize,
}

impl<'a> TLVReader<'a> {
    pub fn new(buffer: &'a [u8]) -> Self {
        Self::new_at(buffer, 0)
    }
    pub fn new_at(buffer: &'a [u8], start: usize) -> Self {
        Self {
            position: start,
            buffer,
        }
    }

    pub fn read_into(self, tlvs: &mut [&mut TLV<'a>]) -> Result<(), TLVError> {
        for entry in self {
            let entry = entry?;
            for v in tlvs.iter_mut() {
                if v.type_id == entry.type_id {
                    (**v).length = entry.length;
                    (**v).data = &entry.data;
                }
            }
        }
        Ok(())
    }

    pub fn require_into(self, tlvs: &mut [&mut TLV<'a>]) -> Result<(), TLVError> {
        self.read_into(tlvs)?;
        for t in tlvs.iter() {
            if t.is_none() {
                return Err(TLVError::MissingEntry);
            }
        }
        Ok(())
    }
}

/// A borrowed TLV entry, it is a thin wrapper around the segment in the original buffer.
/// This may act as an optional when the length is zero, but the lifetime is still tied on an original buffer.
#[derive(PartialEq, Eq, Debug, Copy, Clone)]
pub struct TLV<'a> {
    pub type_id: u8,
    pub length: u8,
    pub data: &'a [u8],
}

impl<'a> TLV<'a> {
    /// Create a new TLV of the specified type, tied to the data buffer.
    pub fn tied<T: Into<u8>>(data: &'a [u8], type_id: T) -> Self {
        Self {
            type_id: type_id.into(),
            length: 0,
            data: &data[0..0],
        }
    }
    /// The TLV contains something, its length is non zero.
    pub fn is_some(&self) -> bool {
        self.length != 0
    }

    /// The TLV contains nothing, its length is zero.
    pub fn is_none(&self) -> bool {
        self.length == 0
    }

    /// Try to interpret the data as a zerocopy-enabled type.
    pub fn try_from<T: TryFromBytes + KnownLayout + Immutable>(&self) -> Result<&T, TLVError> {
        T::try_ref_from_prefix(self.data)
            .map_err(|_| TLVError::UnexpectedValue)
            .map(|(a, _remaining)| a)
    }

    pub fn to_u32(&self) -> Result<u32, TLVError> {
        if self.length == 1 {
            Ok(u8::read_from_bytes(self.data).unwrap() as u32)
        } else if self.length == 2 {
            Ok(u16::read_from_bytes(self.data).unwrap() as u32)
        } else if self.length == 4 {
            Ok(u32::read_from_bytes(self.data).unwrap())
        } else {
            Err(TLVError::UnexpectedValue)
        }
    }
}

impl<'a> Iterator for TLVReader<'a> {
    type Item = Result<TLV<'a>, TLVError>;
    fn next(&mut self) -> Option<Self::Item> {
        if (self.position + 1) < self.buffer.len() {
            let type_id = self.buffer[self.position];
            let length = self.buffer[self.position + 1];
            let data_end = self.position + 2 + length as usize;
            if data_end <= self.buffer.len() {
                let data = &self.buffer[self.position + 2..data_end];
                self.position = data_end;
                return Some(Ok(TLV {
                    type_id,
                    length,
                    data,
                }));
            } else {
                // Ensure the iterator will only ever yields None from now.
                self.position = self.buffer.len() + 1;
                Some(Err(TLVError::NotEnoughData))
            }
        } else {
            None
        }
    }
}

pub struct TLVWriter<'a> {
    position: usize,
    buffer: &'a mut [u8],
}
impl<'a> TLVWriter<'a> {
    pub fn new(buffer: &'a mut [u8]) -> Self {
        Self::new_at(buffer, 0)
    }
    pub fn new_at(buffer: &'a mut [u8], start: usize) -> Self {
        buffer[start..start + 2].fill(0);
        Self {
            position: start,
            buffer,
        }
    }

    pub fn end(&self) -> usize {
        self.position
    }

    pub fn add_u32<T: Into<u8>>(self, t: T, value: u32) -> Self {
        let tt: u8 = t.into();
        // Reduce the width to the necessary length.
        if value <= u8::MAX.into() {
            self.add_slice(tt, &(value as u8).to_le_bytes())
        } else if value <= u16::MAX.into() {
            self.add_slice(tt, &(value as u16).to_le_bytes())
        } else {
            self.add_slice(tt, &value.to_le_bytes())
        }
    }

    pub fn add_entry<T: Into<u8>, V: IntoBytes + Immutable>(self, t: T, value: &V) -> Self {
        self.add_slice(t, &value.as_bytes())
    }

    pub fn add_slice<T: Into<u8>>(mut self, t: T, value: &[u8]) -> Self {
        let tt: u8 = t.into();
        self.write_u8s(tt, value);
        self
    }

    fn push_internal<T: IntoBytes + Immutable>(&mut self, value: &T) {
        let as_bytes = value.as_bytes();
        self.buffer[self.position..self.position + as_bytes.len()].copy_from_slice(as_bytes);
        self.position += as_bytes.len();
    }

    fn write_u8s(&mut self, tt: u8, mut values: &[u8]) {
        while !values.is_empty() {
            let this_length = values.len().min(255);
            self.push_internal(&tt);
            self.push_internal(&((this_length) as u8));

            self.buffer[self.position..self.position + this_length]
                .copy_from_slice(&values[0..this_length]);
            self.position += this_length;
            values = &values[this_length..]
        }
    }
}

#[cfg(test)]
mod test {
    use super::*;

    // https://github.com/apple/HomeKitADK/blob/fb201f98f5fdc7fef6a455054f08b59cca5d1ec8/Tests/HAPTLVTest.c#L103
    #[test]
    fn test_tlv_pairing_parse() {
        crate::test::init();

        // Single TLV.
        // This is a bad test... why aren't the numbers different!?
        let tlv_payload = [0x01, 0x01, 0x01];
        let mut reader = TLVReader::new(&tlv_payload);
        let z = reader.next().unwrap().unwrap();
        assert_eq!(z.type_id, 0x01);
        assert_eq!(z.length, 0x01);
        assert_eq!(z.data, &[0x01]);

        // iw; Single tlv good test.
        let tlv_payload = [0x05, 0x01, 0x03];
        let mut reader = TLVReader::new(&tlv_payload);
        let z = reader.next().unwrap().unwrap();
        assert_eq!(z.type_id, 0x05);
        assert_eq!(z.length, 0x01);
        assert_eq!(z.data, &[0x03]);

        // HomeKit Pair Setup M1
        let data = [0x00, 0x01, 0x00, 0x06, 0x01, 0x01];
        let mut reader = TLVReader::new(&data);
        let a = reader.next().unwrap().unwrap();
        let b = reader.next().unwrap().unwrap();
        assert_eq!(a.type_id, 0x00);
        assert_eq!(a.length, 0x01);
        assert_eq!(a.data, &[0x00]);
        assert_eq!(b.type_id, 0x06);
        assert_eq!(b.length, 0x01);
        assert_eq!(b.data, &[0x01]);

        let mut a = TLV::tied(&data, 0x00);
        let mut b = TLV::tied(&data, 0x06);

        let collected = TLVReader::new(&data).read_into(&mut [&mut a, &mut b]);
        assert_eq!(collected.is_ok(), true);
        assert_eq!(a.type_id, 0x00);
        assert_eq!(a.length, 0x01);
        assert_eq!(a.data, &[0x00]);
        assert_eq!(b.type_id, 0x06);
        assert_eq!(b.length, 0x01);
        assert_eq!(b.data, &[0x01]);
    }

    #[test]
    fn test_tlv_to_u32() {
        crate::test::init();

        let entry = TLVReader::new(&[0x00, 0x01, 0x37]).next().unwrap().unwrap();
        assert_eq!(entry.to_u32().unwrap(), 0x37);
        let entry = TLVReader::new(&[0x00, 0x02, 0x37, 0x02])
            .next()
            .unwrap()
            .unwrap();
        assert_eq!(
            entry.to_u32().unwrap(),
            u16::from_le_bytes([0x37, 0x02]) as u32
        );
        let entry = TLVReader::new(&[0x00, 0x04, 0x37, 0x02, 0x00, 0x03])
            .next()
            .unwrap()
            .unwrap();
        assert_eq!(
            entry.to_u32().unwrap(),
            u32::from_le_bytes([0x37, 0x02, 0x00, 0x03])
        );
    }
    #[test]
    fn test_tlv_write() {
        let mut buffer = [0u8; 32];
        let tv = 0x00;
        let length = TLVWriter::new(&mut buffer).add_entry(tv, &0x37u8).end();
        assert_eq!(length, 3);
        assert_eq!(&buffer[0..3], &[tv, 0x01, 0x37]);

        buffer.fill(0);
        let length = TLVWriter::new(&mut buffer).add_u32(tv, 0x37).end();
        assert_eq!(length, 3);
        assert_eq!(&buffer[0..3], &[tv, 0x01, 0x37]);

        // Test really long write from
        // https://github.com/apple/HomeKitADK/blob/fb201f98f5fdc7fef6a455054f08b59cca5d1ec8/Tests/HAPTLVTest.c#L127
        //
        //
        #[rustfmt::skip]
        let payload = [
            /*0x03, 0xff,*/ 0x4b, 0x20, 0x6a, 0x23, 0x3c, 0x2b, 0x33, 0xf0, 0x29, 0xf1, 0xd2, 0x82, 0xa4, 0xe1, 0x4c, 0xb9,
            0x39, 0x96, 0x24, 0x99, 0x48, 0x3f, 0x48, 0xdb, 0xd7, 0x51, 0x3c, 0xb4, 0x3f, 0x9d, 0xcd, 0x73, 0x32, 0x61,
            0x2e, 0xaf, 0x0f, 0xca, 0x70, 0x6d, 0xd1, 0x15, 0x93, 0xa5, 0x69, 0x81, 0xe1, 0xcd, 0x21, 0x09, 0x5c, 0x09,
            0x38, 0x84, 0x96, 0x19, 0xbc, 0xcb, 0xd7, 0x1d, 0xf6, 0x5b, 0x2e, 0xe5, 0xe3, 0x74, 0x81, 0x78, 0xb7, 0x43,
            0x14, 0x7f, 0x53, 0xad, 0x86, 0x5c, 0x19, 0xf0, 0x5f, 0xb7, 0x73, 0x10, 0x5b, 0xf4, 0xcb, 0x5a, 0x4a, 0x09,
            0x84, 0xda, 0x4b, 0xaa, 0x63, 0x08, 0xc3, 0xd0, 0x46, 0xff, 0x3e, 0x24, 0xd6, 0xcf, 0xaa, 0xaf, 0xd8, 0x44,
            0x76, 0xa9, 0x1d, 0x24, 0x0e, 0x5c, 0x58, 0xa4, 0x96, 0xf4, 0x85, 0x87, 0x94, 0x42, 0xc3, 0xb9, 0xd8, 0x1d,
            0xe8, 0xb8, 0x14, 0x0d, 0x36, 0x8e, 0xc0, 0x0e, 0x67, 0x8e, 0xce, 0xb8, 0x0d, 0x1f, 0x22, 0x9c, 0xb5, 0x4d,
            0x30, 0x9b, 0x81, 0x09, 0x23, 0xac, 0xcd, 0xc9, 0x8f, 0x89, 0x14, 0xb0, 0x74, 0x75, 0xd1, 0xfb, 0xee, 0x69,
            0xb9, 0x2f, 0xaa, 0x6f, 0xce, 0x83, 0xfe, 0xbe, 0xae, 0xd1, 0x52, 0x8d, 0x21, 0x1b, 0x9e, 0x2b, 0xb2, 0xe1,
            0x88, 0x2f, 0x0f, 0xfc, 0x2d, 0xf2, 0x0b, 0xe2, 0x51, 0x0d, 0xb4, 0xf4, 0xd9, 0x0e, 0x25, 0xce, 0x99, 0x6d,
            0x42, 0x76, 0x50, 0xb3, 0x75, 0xcc, 0x98, 0x31, 0xc3, 0x4d, 0x50, 0x90, 0xd1, 0x2e, 0x3c, 0x3f, 0x94, 0x23,
            0xef, 0xc6, 0x99, 0xf7, 0x60, 0x7a, 0x97, 0x30, 0x61, 0x43, 0xeb, 0x7a, 0x4d, 0x56, 0xfd, 0x26, 0x27, 0x88,
            0x8b, 0xf4, 0xa7, 0x4d, 0x28, 0x94, 0x1e, 0x9a, 0xbf, 0xe2, 0x48, 0x19, 0xb7, 0x29, 0x96, 0xda, 0x3e, 0x4d,
            0x84, 0xee, 0xcf, 0xe3, 0x82,

            /*0x03, 0x81,*/ 0x1f, 0x03, 0x9d, 0x75, 0x26, 0x37, 0xfd, 0x60, 0xbb, 0xbc, 0x47, 0xbd, 0x2e, 0x9a, 0xc8, 0xa0,
            0x7d, 0x6e, 0x00, 0x09, 0x05, 0xe3, 0xc5, 0x78, 0x7b, 0x8d, 0x34, 0x1b, 0x4c, 0x1a, 0x02, 0xef, 0x3a, 0xcc,
            0xf1, 0x34, 0xf2, 0x4a, 0x28, 0x9d, 0xc9, 0xa4, 0xdd, 0x0a, 0x2b, 0xee, 0xd3, 0x5c, 0x4e, 0x66, 0x18, 0xa2,
            0x27, 0x00, 0x09, 0xb7, 0x32, 0x8e, 0x8a, 0x0b, 0x4a, 0x15, 0x04, 0xf9, 0x5e, 0x88, 0xf0, 0x6a, 0xf0, 0x02,
            0x5b, 0xb4, 0x89, 0xfb, 0x3b, 0xc5, 0xdd, 0x36, 0xe4, 0xdd, 0xa7, 0x4f, 0xb7, 0xdf, 0x22, 0xbb, 0x03, 0x3c,
            0xd7, 0xcd, 0xe1, 0x39, 0x17, 0x6f, 0x1d, 0xfa, 0xfd, 0xa1, 0x23, 0x3a, 0xf3, 0x15, 0x56, 0x08, 0xda, 0x57,
            0x19, 0xd8, 0x10, 0x78, 0xd3, 0x89, 0xe1, 0x97, 0xa4, 0x0f, 0x77, 0x48, 0xae, 0x10, 0xf0, 0xcd, 0xd3, 0xb2,
            0x71, 0xdb, 0x84, 0x23, 0x8f,
        ];
        let mut buffer = [0u8; 512];
        let length = TLVWriter::new(&mut buffer).add_slice(0x03, &payload).end();
        assert_eq!(length, 388);
        assert_eq!(&buffer[0..2], &[0x03, 0xff]);
        assert_eq!(&buffer[255 + 2..255 + 4], &[0x03, 0x81]);
        assert_eq!(&buffer[255..255 + 6], &[0xe3, 0x82, 0x03, 0x81, 0x1f, 0x03]);
    }
}
