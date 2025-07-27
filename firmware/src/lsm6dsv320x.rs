use embedded_hal_async::spi::SpiDevice;
// https://docs.rs/embedded-hal/1.0.0/embedded_hal/spi/index.html#for-driver-authors
// > If your device has a CS pin, use SpiDevice. Do not manually manage the CS pin, the SpiDevice implementation will
// > do it for you. By using SpiDevice, your driver will cooperate nicely with other drivers for other devices in the same shared SPI bus.
// That's us!

// https://github.com/rust-embedded/embedded-hal/issues/572
// https://docs.embassy.dev/embassy-embedded-hal/git/default/shared_bus/asynch/spi/index.html

#[derive(Debug, Copy, Clone, PartialEq, defmt::Format)]
pub enum Error<SpiError: embedded_hal_async::spi::Error> {
    /// Underlying SpiError device error
    Spi(SpiError),
    Foo, // dummy placeholder
}

#[derive(Clone, PartialEq)]
pub struct LSM6DSV320X<Spi> {
    spi: Spi,
}
impl<Spi> core::fmt::Debug for LSM6DSV320X<Spi> {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        f.write_fmt(format_args!("<LSM6DSV320X>"))
    }
}
impl<Spi> defmt::Format for LSM6DSV320X<Spi> {
    fn format(&self, f: defmt::Formatter) {
        defmt::write!(f, "<LSM6DSV320X>")
    }
}

impl<Spi: embedded_hal_async::spi::SpiDevice> LSM6DSV320X<Spi>
where
    Spi: SpiDevice<u8>,
    Spi::Error: embedded_hal_async::spi::Error,
{
    pub async fn new(spi: Spi) -> Result<Self, Error<Spi::Error>> {
        Ok(Self { spi })
    }
}
