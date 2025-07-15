// We allow static mut refs here because we need to access a global in a non-thread-safe way, we do so from a critical
// section.
#![allow(static_mut_refs)]

use embassy_rp::peripherals::USB;
use embassy_rp::usb::Driver;
use static_cell::StaticCell;

// Modelled after
// https://github.com/knurling-rs/defmt/blob/8e517f8d7224237893e39337a61de8ef98b341f2/firmware/defmt-itm/src/lib.rs#L44
// Is this useful?
// https://github.com/embassy-rs/embassy/blob/9651cfca51a273ba46d34ce8197fc0e63389b09e/examples/rp/src/bin/assign_resources.rs

use core::sync::atomic::{AtomicBool, Ordering};
use embassy_usb::class::cdc_acm::Sender;

pub struct SerialLogger {
    s: Sender<'static, Driver<'static, USB>>,
}

static mut ENCODER: defmt::Encoder = defmt::Encoder::new();

static DEFMT_OVERRUN: AtomicBool = AtomicBool::new(false);
static TAKEN: AtomicBool = AtomicBool::new(false);
static mut CS_RESTORE: critical_section::RestoreState = critical_section::RestoreState::invalid();

impl SerialLogger {
    pub fn new(s: Sender<'static, Driver<'static, USB>>) -> Self {
        Self { s }
    }
}

use embassy_time::{Duration, Timer};

// Buffer for defmt messages.
const DEFMT_SERIAL_BUFFER: usize = 4096;

type Queue = heapless::spsc::Queue<u8, DEFMT_SERIAL_BUFFER>;
type Producer = heapless::spsc::Producer<'static, u8, DEFMT_SERIAL_BUFFER>;

static mut TX_THING: Option<Producer> = None;

pub async fn run(logger: SerialLogger) -> ! {
    let mut logger = logger;
    let queue: &'static mut Queue = {
        static STATE: StaticCell<Queue> = StaticCell::new();
        STATE.init(Queue::new())
    };
    let (tx, mut rx) = queue.split();

    unsafe {
        TX_THING = Some(tx);
    }

    loop {
        // Take stuff from rx and shove into serial port.

        // Swap the overrun buffer to alert user of an overrun.
        if DEFMT_OVERRUN.swap(false, Ordering::Relaxed) {
            defmt::error!("overrun");
        }

        // Consume all data and shove it onto the serial port using its max packet size.
        let mut have_data = rx.ready();
        if have_data {
            while have_data {
                const USB_CDC_MAX_PACKET_SIZE_LIMIT: usize = 64;
                let mut buffer = [0u8; USB_CDC_MAX_PACKET_SIZE_LIMIT];
                let count = rx.len();
                let this_packet_len = count.min(logger.s.max_packet_size() as usize);

                // Drain the queue for this packet length bytes.
                for i in 0..this_packet_len {
                    if let Some(v) = rx.dequeue() {
                        buffer[i] = v;
                    }
                }
                if let Err(e) = logger.s.write_packet(&buffer[0..this_packet_len]).await {
                    // ehh... panic? Maybe it's transient? Lets just push a message about it.
                    defmt::error!("failed writing: {:?}", e);
                }
                have_data = rx.ready();
            }
        } else {
            // No data, wait for a millisecond
            let delay = Duration::from_millis(1);
            Timer::after(delay).await;
        }
    }
}

#[defmt::global_logger]
struct Logger;

fn do_write(bytes: &[u8]) {
    // NOTE(unsafe) this function will be invoked *after* run has been started and the global logger has been populated.
    unsafe {
        if let Some(tx) = TX_THING.as_mut() {
            for b in bytes {
                if let Err(v) = tx.enqueue(*b) {
                    let _discard = v;
                    // Store that we had a buffer overrun, much sad, no recovery here.
                    DEFMT_OVERRUN.store(true, Ordering::Relaxed);
                }
            }
        }
    }
}

unsafe impl defmt::Logger for Logger {
    fn acquire() {
        // safety: Must be paired with corresponding call to release(), see below
        let restore = unsafe { critical_section::acquire() };

        // safety: accessing the atomic without CAS is OK because we have acquired a critical section.
        if TAKEN.load(Ordering::Relaxed) {
            panic!("defmt logger taken reentrantly")
        }

        // safety: accessing the atomic without CAS is OK because we have acquired a critical section.
        TAKEN.store(true, Ordering::Relaxed);

        // safety: accessing the `static mut` is OK because we have acquired a critical section.
        unsafe { CS_RESTORE = restore };

        // safety: accessing the `static mut` is OK because we have acquired a critical section.
        unsafe {
            let encoder: &mut defmt::Encoder = &mut *core::ptr::addr_of_mut!(ENCODER);
            encoder.start_frame(do_write)
        }
    }
    unsafe fn flush() {
        // ...
    }
    unsafe fn release() {
        // safety: accessing the `static mut` is OK because we have acquired a critical section.
        unsafe {
            let encoder: &mut defmt::Encoder = &mut *core::ptr::addr_of_mut!(ENCODER);
            encoder.end_frame(do_write);
        }

        // safety: accessing the atomic without CAS is OK because we have acquired a critical section.
        TAKEN.store(false, Ordering::Relaxed);

        // safety: accessing the `static mut` is OK because we have acquired a critical section.
        let restore = unsafe { CS_RESTORE };

        // safety: Must be paired with corresponding call to acquire(), see above
        unsafe {
            critical_section::release(restore);
        }
    }
    unsafe fn write(bytes: &[u8]) {
        // safety: accessing the `static mut` is OK because we have acquired a critical section.
        unsafe {
            let encoder: &mut defmt::Encoder = &mut *core::ptr::addr_of_mut!(ENCODER);
            encoder.write(bytes, do_write);
        }
    }
}
