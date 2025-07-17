use core::fmt::Write;

use embassy_rp::watchdog::Watchdog;

pub mod chip_info {
    use embassy_rp::rom_data;
    // The serial number provided before reboot must match the serial number that becomes available.
    // We need to extract get_sys_info,  datasheet section 5.4.8.17
    // https://github.com/raspberrypi/pico-sdk/blob/9a4113fbbae65ee82d8cd6537963bc3d3b14bcca/src/rp2_common/pico_unique_id/unique_id.c#L35-L44

    pub const PICO_UNIQUE_BOARD_ID_SIZE_BYTES: usize = 8;

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

            for i in 0..(res.0.len() / 2) {
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
}

pub mod reboot {

    use embassy_time::Duration;

    use core::ops::BitXor;
    // Section 5.4.8.24. reboot, page 395
    // Normal:
    pub const PICO_BOOTROM_REBOOT_FLAGS_NORMAL: u32 = 0x0000;
    // p0 is the boot diagnostic partition, lower 8 bits only.
    // Bootsel:
    // P0 is GPIO for actvitity indicator.
    pub const PICO_BOOTROM_REBOOT_FLAGS_BOOTSEL: u32 = 0x02;
    pub const PICO_BOOTROM_REBOOT_FLAGS_BOOTSEL_P1_DISABLE_MSD: u32 = 0x01;
    pub const PICO_BOOTROM_REBOOT_FLAGS_BOOTSEL_P1_DISABLE_PICOBOOT: u32 = 0x02;

    // Can be OR with any of these flags.
    pub const PICO_BOOTROM_REBOOT_NO_RETURN_ON_SUCCESS: u32 = 0x0100;

    #[allow(unused)]
    pub fn dev_boot_to_bootsel_watchdog(w: &mut embassy_rp::watchdog::Watchdog) -> ! {
        // This method also works, and was the main identifier for the p0 & p1 situation.
        // boot flags
        w.set_scratch(2, PICO_BOOTROM_REBOOT_FLAGS_BOOTSEL_P1_DISABLE_MSD);
        // gpio index.
        w.set_scratch(3, 0);

        // Magic values to trigger bootrom method.
        w.set_scratch(4, 0xb007c0d3);
        w.set_scratch(5, 0x4ff83f2du32.bitxor(0xb007c0d3));
        w.set_scratch(6, 2);
        w.set_scratch(7, 0xb007c0d3);

        // Invoke the watchdog
        w.trigger_reset();
        loop {}
    }

    pub fn dev_boot_to_bootsel() -> ! {
        // Now we need to do the equivalent of;
        // rom_reset_usb_boot_extra(gpio, (request->wValue & 0x7f) | PICO_STDIO_USB_RESET_BOOTSEL_INTERFACE_DISABLE_MASK, active_low);
        // Well, that exists for the rp2040;
        // https://github.com/embassy-rs/embassy/blob/20ce4d7deb3b16ff32bd68f7adae9066ddd40e94/embassy-rp/src/rom_data/rp2040.rs#L229-L239
        //let f = embassy_rp::rom_data::reset_to_usb_boot::ptr();
        const BOOTSEL_MAIN: u32 =
            PICO_BOOTROM_REBOOT_FLAGS_BOOTSEL | PICO_BOOTROM_REBOOT_NO_RETURN_ON_SUCCESS;
        const DELAY_MS: u32 = 500;
        const P0_GPIO_PIN: u32 = 0;
        const P1_BOOT_PROPERTIES: u32 = PICO_BOOTROM_REBOOT_FLAGS_BOOTSEL_P1_DISABLE_MSD;

        // Well, that disable MSD doesn't stick?
        // The p0 and p1 parameters are generally written to watchdog scratch registers 2 & 3, and are interpreted post-reboot
        // by the boot path code.
        //
        //

        // I think the datasheet is wrong, it doesn't agree with pico-sdk, the following works to inhibit the MSD.
        // https://github.com/raspberrypi/pico-feedback/issues/466
        embassy_rp::rom_data::reboot(BOOTSEL_MAIN, DELAY_MS, P1_BOOT_PROPERTIES, P0_GPIO_PIN);

        // This is never reached.
        loop {}
    }

    #[derive(Debug, Copy, Clone, Default, Eq, PartialEq, defmt::Format)]
    pub enum RebootSettings {
        #[default]
        Normal,
        Bootsel {
            picoboot: bool,
            mass_storage: bool,
        },
    }
    impl RebootSettings {
        /// Reboot settings for flashing the microcontroller.
        ///
        /// This disables the mass storage interface to avoid mounting it etc.
        pub fn flash() -> Self {
            RebootSettings::Bootsel {
                picoboot: true,
                mass_storage: false,
            }
        }
    }

    pub fn reboot_raw(main_flags: u32, delay_ms: u32, boot_properties: u32) {
        const P0_GPIO_PIN: u32 = 0;
        // I think the datasheet is wrong, it doesn't agree with pico-sdk, the following works to inhibit the MSD.
        // https://github.com/raspberrypi/pico-feedback/issues/466
        embassy_rp::rom_data::reboot(main_flags, delay_ms, boot_properties, P0_GPIO_PIN);
    }

    pub fn reboot(boot_type: RebootSettings, delay: Duration) -> ! {
        let (main, boot_properties) = match boot_type {
            RebootSettings::Normal => (0, 0),
            RebootSettings::Bootsel {
                picoboot,
                mass_storage,
            } => (
                PICO_BOOTROM_REBOOT_FLAGS_BOOTSEL,
                if picoboot {
                    0
                } else {
                    PICO_BOOTROM_REBOOT_FLAGS_BOOTSEL_P1_DISABLE_PICOBOOT
                } | if mass_storage {
                    0
                } else {
                    PICO_BOOTROM_REBOOT_FLAGS_BOOTSEL_P1_DISABLE_MSD
                },
            ),
        };

        const P0_GPIO_PIN: u32 = 0;

        let delay_ms = delay.as_millis().min(u32::MAX as u64) as u32;
        // I think the datasheet is wrong, it doesn't agree with pico-sdk, the following works to inhibit the MSD.
        // https://github.com/raspberrypi/pico-feedback/issues/466
        embassy_rp::rom_data::reboot(
            main | PICO_BOOTROM_REBOOT_NO_RETURN_ON_SUCCESS,
            delay_ms,
            boot_properties,
            P0_GPIO_PIN,
        );

        // This is never reached.
        loop {}
    }
}

const PANIC_STORAGE_SIZE: usize = 32;
#[repr(C)]
#[derive(Debug, Copy, Clone, Default, Eq, PartialEq, defmt::Format)]
pub struct PanicStorage {
    // First register.
    pub populated: u32,
    // Second register.
    pub line: u16,
    pub column: u16,
    // next 6 registers, 6 * 4 = 24.
    pub file: [u8; PANIC_STORAGE_SIZE - (4 + 2 + 2)],
}
const _: () = [(); 1][(core::mem::size_of::<PanicStorage>() == PANIC_STORAGE_SIZE) as usize ^ 1];

impl PanicStorage {
    pub fn instantiate(buffer: &[u8]) -> Option<&PanicStorage> {
        if buffer.len() != Self::size() {
            None
        } else {
            unsafe { core::mem::transmute::<_, *const PanicStorage>(buffer.as_ptr()).as_ref() }
        }
    }

    pub fn to_array(&self) -> [u8; PANIC_STORAGE_SIZE] {
        // get a byte pointer to self.
        let selfptr = self as *const PanicStorage;
        let self_u8 = selfptr.cast::<u8>();
        let self_slice = unsafe { core::slice::from_raw_parts(self_u8, PANIC_STORAGE_SIZE) };
        let mut res: [u8; PANIC_STORAGE_SIZE] = Default::default();
        res.copy_from_slice(self_slice);
        res
    }

    pub const fn size() -> usize {
        core::mem::size_of::<PanicStorage>()
    }

    pub fn from_panic(info: &core::panic::PanicInfo) -> PanicStorage {
        let mut storage = PanicStorage::default();
        if let Some(location) = info.location() {
            storage.populated = 1;
            storage.line = location.line() as u16;
            storage.column = location.column() as u16;
            let f = location.file();
            // Take the last part of the string if the file is too long.

            for (i, c) in f
                .chars()
                .skip((f.len() as isize - storage.file.len() as isize).max(0) as usize)
                .enumerate()
            {
                c.encode_utf8(&mut storage.file[i..(i + 1)]);
            }
        }

        // If this fails... well tough luck, we are already in a panic handler.
        //storage.fmt_error = write!(storage.message, "{}", info.message()).is_err();
        storage
    }

    pub fn from_address(address: usize) -> PanicStorage {
        unsafe {
            let sl = core::slice::from_raw_parts(address as *const u8, Self::size());
            Self::instantiate(sl).unwrap().clone()
        }
    }
}

pub fn write_scratch(w: &mut Watchdog, data: [u8; 32]) {
    for (i, v) in data.chunks(4).enumerate() {
        let mut u32b: [u8; 4] = Default::default();
        u32b.copy_from_slice(v);
        w.set_scratch(i, u32::from_le_bytes(u32b));
    }
}
pub fn read_scratch(w: &mut Watchdog) -> [u8; 32] {
    let mut res: [u8; 32] = Default::default();
    for i in 0..8 {
        let v = w.get_scratch(i);
        res[i * 4..(i + 1) * 4].copy_from_slice(&v.to_le_bytes());
    }
    res
}

pub const PANIC_INFO_WATCHDOG_SCRATCH: usize = 1;

#[panic_handler]
fn panic(info: &core::panic::PanicInfo) -> ! {
    use crate::rp2350_util::reboot::RebootSettings;
    use embassy_time::Duration;

    let storage = PanicStorage::from_panic(info);

    // Obviously, this panic handler is superior.
    unsafe {
        let p = embassy_rp::Peripherals::steal();
        let mut w = embassy_rp::watchdog::Watchdog::new(p.WATCHDOG);
        let data = storage.to_array();
        write_scratch(&mut w, data);
    }
    reboot::reboot(RebootSettings::Normal, Duration::from_millis(100));
}
