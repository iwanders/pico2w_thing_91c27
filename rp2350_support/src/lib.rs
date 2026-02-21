#![cfg_attr(target_arch = "arm", no_std)]
#![cfg_attr(not(test), no_main)]

pub mod defmt_serial;
pub mod static_files;

#[cfg(target_arch = "arm")]
pub mod rp2350_util;
#[cfg(target_arch = "arm")]
pub mod usb_picotool_reset;
