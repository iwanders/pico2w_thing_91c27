use defmt::{info, unwrap};
use embassy_executor::Spawner;
use static_cell::StaticCell;

// https://github.com/embassy-rs/embassy/blob/9651cfca51a273ba46d34ce8197fc0e63389b09e/examples/rp/src/bin/assign_resources.rs

struct GlobalLogger {}
static GLOBAL_LOGGER: StaticCell<GlobalLogger> = StaticCell::new();

#[defmt::global_logger]
struct Logger;

unsafe impl defmt::Logger for Logger {
    fn acquire() {
        // ...
    }
    unsafe fn flush() {
        // ...
    }
    unsafe fn release() {
        // ...
    }
    unsafe fn write(bytes: &[u8]) {
        // ...
    }
}

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
