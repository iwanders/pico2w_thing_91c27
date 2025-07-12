#![no_std]
#![no_main]

use rp235x_hal::entry;
#[entry]
fn main() -> ! {
    loop {}
}

#[panic_handler]
fn panic_hanlder<'a, 'b>(v: &'a core::panic::PanicInfo<'b>) -> ! {
    loop {}
}
