use defmt::{info, unwrap};
use embassy_executor::Spawner;
use embassy_rp::peripherals::USB;
use embassy_rp::usb::Driver;
use static_cell::StaticCell;

// Modelled after
// https://github.com/knurling-rs/defmt/blob/8e517f8d7224237893e39337a61de8ef98b341f2/firmware/defmt-itm/src/lib.rs#L44
//

use core::{
    mem::MaybeUninit,
    sync::atomic::{AtomicBool, Ordering},
};
use embassy_usb::class::cdc_acm::Sender;

// https://github.com/embassy-rs/embassy/blob/9651cfca51a273ba46d34ce8197fc0e63389b09e/examples/rp/src/bin/assign_resources.rs

pub struct SerialLogger {
    s: Sender<'static, Driver<'static, USB>>,
}
//static GLOBAL_LOGGER: StaticCell<SerialLogger> = StaticCell::new();
static mut ENCODER: defmt::Encoder = defmt::Encoder::new();

static TAKEN: AtomicBool = AtomicBool::new(false);
static mut CS_RESTORE: critical_section::RestoreState = critical_section::RestoreState::invalid();

impl SerialLogger {
    pub fn new(s: Sender<'static, Driver<'static, USB>>) -> Self {
        Self { s }
    }
}

use core::cell::RefCell;
use critical_section::Mutex;
use embassy_time::{Duration, Timer};

type Queue = heapless::spsc::Queue<u8, 1024>;
type Producer = heapless::spsc::Producer<'static, u8, 1024>;

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
        if rx.ready() {
            let mut buffer = [0u8; 1024];
            let count = rx.len();
            for i in 0..count {
                if let Some(v) = rx.dequeue() {
                    buffer[i] = v;
                }
            }
            if let Err(_e) = logger.s.write_packet(&buffer[0..count]).await {
                // ehh... panic?
            }
        } else {
            let delay = Duration::from_millis(10);
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
                tx.enqueue(*b);
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

/*
use embassy_usb::class::cdc_acm::Sender;
use embassy_usb::driver::{Driver, EndpointError};
pub struct WritableSerial<'d, D: Driver<'d>> {
    d: Sender<'d, D>,
    //_foo: core::marker::PhantomData<&'d ()>,
}
impl<'d, D: Driver<'d>> WritableSerial<'d, D> {
    pub fn new(d: Sender<'d, D>) -> Self {
        Self { d }
    }
}

#[derive(Debug, Copy, Clone)]
pub struct WrappedEndpointError(EndpointError);
impl embedded_io_async::Error for WrappedEndpointError {
    fn kind(&self) -> embedded_io_async::ErrorKind {
        match self.0 {
            EndpointError::BufferOverflow => embedded_io_async::ErrorKind::OutOfMemory,
            EndpointError::Disabled => embedded_io_async::ErrorKind::NotConnected,
        }
    }
}

// I'm on the latest released verison, so we need this newtype thing.
impl<'d, D: Driver<'d>> embedded_io_async::ErrorType for WritableSerial<'d, D> {
    type Error = WrappedEndpointError;
}

impl<'d, D: Driver<'d>> embedded_io_async::Write for WritableSerial<'d, D> {
    async fn write(&mut self, buf: &[u8]) -> Result<usize, Self::Error> {
        let len = core::cmp::min(buf.len(), self.d.max_packet_size() as usize);
        self.d
            .write_packet(&buf[..len])
            .await
            .map_err(|z| WrappedEndpointError(z))?;
        Ok(len)
    }
}

pub async fn runner<'d, D: Driver<'d>>(_spawner: Spawner, w: WritableSerial<'d, D>) {
    loop {
        info!("toggling leds");
    }
}
*/
