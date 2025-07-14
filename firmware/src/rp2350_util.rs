use embassy_rp::rom_data;

// The serial number provided before reboot must match the serial number that becomes available.
// We need to extract get_sys_info,  datasheet section 5.4.8.17
// https://github.com/raspberrypi/pico-sdk/blob/9a4113fbbae65ee82d8cd6537963bc3d3b14bcca/src/rp2_common/pico_unique_id/unique_id.c#L35-L44

const PICO_UNIQUE_BOARD_ID_SIZE_BYTES: usize = 8;

#[derive(Debug, Copy, Clone, Default, Eq, PartialEq, defmt::Format)]
pub struct SerialAscii([u8; PICO_UNIQUE_BOARD_ID_SIZE_BYTES * 2]);

#[derive(Debug, Copy, Clone, Default, Eq, PartialEq, defmt::Format)]
pub struct SerialNumber(pub u64);

#[derive(Debug, Copy, Clone, Default, defmt::Format)]
pub struct ChipInfo {
    /// Value of the CHIP_INFO_PACKAGE_SEL register, no documentation on that?
    pub package_sel: u32,
    /// RP2350 device id.
    pub device_id: u32,
    /// RP2350 wafer id.
    pub wafer_id: u32,
}

impl ChipInfo {
    pub fn serial_number(&self) -> SerialNumber {
        let mut serial_number_bytes: [u8; PICO_UNIQUE_BOARD_ID_SIZE_BYTES] = Default::default();
        serial_number_bytes[4..].copy_from_slice(&self.wafer_id.to_le_bytes());
        serial_number_bytes[0..4].copy_from_slice(&self.device_id.to_le_bytes());
        SerialNumber(u64::from_le_bytes(serial_number_bytes))
    }
    pub fn serial_ascii(&self) -> SerialAscii {
        let mut res: SerialAscii = Default::default();
        // Lets just avoid format here and do it manually, it needs to be upper case for the USB
        // serial matching.
        const LOOKUP: &[u8; 16] = b"0123456789ABCDEF";

        let mut serial_number_bytes: [u8; PICO_UNIQUE_BOARD_ID_SIZE_BYTES] = Default::default();
        serial_number_bytes[4..].copy_from_slice(&self.wafer_id.to_le_bytes());
        serial_number_bytes[0..4].copy_from_slice(&self.device_id.to_le_bytes());

        for i in (0..(res.0.len() / 2)) {
            let low = serial_number_bytes[i] & 0xF;
            let high = (serial_number_bytes[i] >> 4) & 0xF;
            res.0[res.0.len() - i * 2 - 1] = LOOKUP[low as usize];
            res.0[res.0.len() - i * 2 - 1 - 1] = LOOKUP[high as usize];
        }
        res
    }
}
impl SerialAscii {
    pub fn as_str(&self) -> &str {
        // Unwrap is safe because this is only ever created by serial_ascii, which always creates
        // valid ascii.
        unsafe { core::str::from_utf8_unchecked(&self.0) }
    }
    pub fn as_slice(&self) -> &[u8] {
        &self.0
    }
    pub fn as_array(&self) -> &[u8; PICO_UNIQUE_BOARD_ID_SIZE_BYTES * 2] {
        &self.0
    }
}

pub fn get_chip_info() -> ChipInfo {
    const SYS_INFO_CHIP_INFO: u32 = 0x0001;
    const FLAGS: u32 = SYS_INFO_CHIP_INFO;
    let mut words: [u32; 9] = Default::default();
    let rc = unsafe { rom_data::get_sys_info(words.as_mut_ptr(), words.len(), FLAGS) };
    if rc < 0 {
        panic!("get sys info failed; {rc}");
    }
    if rc != 4 {
        panic!("get sys info returned unexpected count; {rc}");
    }
    if (words[0] & SYS_INFO_CHIP_INFO) == 0 {
        panic!("get sys info did not return chip info {}", words[0]);
    }
    // Okay, now finally we can read the data, since chip info is always the first entry in the returned data.
    ChipInfo {
        package_sel: words[1],
        device_id: words[2],
        wafer_id: words[3],
    }
}
