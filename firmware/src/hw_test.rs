use embassy_rp::gpio::{Input, Level, Output};
use embassy_rp::{bind_interrupts, Peripherals};
use embassy_time::{Duration, Timer};

struct IcmPinTransfer {
    spi: embassy_rp::peripherals::SPI0,
    cs: embassy_rp::peripherals::PIN_5,
    clk: embassy_rp::peripherals::PIN_2,
    mosi: embassy_rp::peripherals::PIN_3,
    miso: embassy_rp::peripherals::PIN_4,
}

async fn test_icm(p: IcmPinTransfer) {
    use embassy_rp::spi::{Config, Spi};
    let mut cs = Output::new(p.cs, Level::High);
    let mut config = Config::default();
    config.frequency = 24_000_000;

    // Page 53
    // CS goes low for selection.
    // Data is latched on the rising edge of SCLK, max clock is 24 MHz O_o
    // Reads are two or more bytes, first byte is SPI address, following is the data.
    // First bit of the first byte indicates read, then following 7 bites are register address.
    let mut spi = Spi::new_blocking(p.spi, p.clk, p.mosi, p.miso, config);
    const REGISTER_READ: u8 = 1 << 7;
    const REG_WHO_AM_I: u8 = 0x75;

    let mut read = [0u8; 2];
    let mut buf = [0u8; 2];
    buf[0] = REGISTER_READ | REG_WHO_AM_I;
    cs.set_low();
    let z = spi.blocking_transfer(&mut read, &buf);
    cs.set_high();
    defmt::info!("ICM who am i: {:?} {:x}", z, read);
    if read[1] == 0x47 {
        defmt::info!("ICM is responsive!");
    } else {
        defmt::error!("ICM test failed!");
    }
}

struct LSM6DSV320XPinTransfer {
    spi: embassy_rp::peripherals::SPI1,
    cs: embassy_rp::peripherals::PIN_13,
    clk: embassy_rp::peripherals::PIN_10,
    mosi: embassy_rp::peripherals::PIN_11,
    miso: embassy_rp::peripherals::PIN_12,
}
async fn test_lsm(p: LSM6DSV320XPinTransfer) {
    use embassy_rp::spi::{Config, Spi};
    let mut cs = Output::new(p.cs, Level::High);
    let mut config = Config::default();

    config.frequency = 10_000_000;

    // page 26
    // CS goes low for selection.
    // Data is latched on the rising edge of SCLK, max clock is 24 MHz O_o
    // Reads are two or more bytes, first byte is SPI address, following is the data.
    // First bit of the first byte indicates read, then following 7 bites are register address.
    let mut spi = Spi::new_blocking(p.spi, p.clk, p.mosi, p.miso, config);
    const REGISTER_READ: u8 = 1 << 7;
    const REG_WHO_AM_I: u8 = 0x0F;

    let mut read = [0u8; 2];
    let mut buf = [0u8; 2];
    buf[0] = REGISTER_READ | REG_WHO_AM_I;
    cs.set_low();
    let z = spi.blocking_transfer(&mut read, &buf);
    cs.set_high();
    defmt::info!("LSM who am i: {:?} {:x}", z, read);
    if read[1] == 0x73 {
        defmt::info!("LSM is responsive!");
    } else {
        defmt::error!("LSM test failed!");
    }
}

struct BmePinTransfer {
    i2c: embassy_rp::peripherals::I2C0,
    sda: embassy_rp::peripherals::PIN_20,
    scl: embassy_rp::peripherals::PIN_21,
}
async fn test_bme(p: BmePinTransfer) {
    use embassy_rp::i2c::{Config, I2c};
    let mut config = Config::default();
    //config.frequency = 100_000;
    let mut i2c = I2c::new_blocking(p.i2c, p.scl, p.sda, config);

    const I2C_BUS_ADDRESS: u8 = 0x76;
    const REG_CHIP_ID: u8 = 0xD0;
    let mut read = [0u8; 1];
    let mut buf = [0u8; 1];
    buf[0] = REG_CHIP_ID;
    let z = i2c.blocking_write_read(I2C_BUS_ADDRESS, &buf, &mut read);
    let z = 3;
    defmt::info!("BME who am i: {:?} {:x}", z, read);
    if read[0] == 0x60 {
        defmt::info!("BME is responsive!");
    } else {
        defmt::error!("BME test failed!");
    }
}
struct FlashPinTransfer {
    spi: embassy_rp::peripherals::SPI0,
    cs: embassy_rp::peripherals::PIN_17,
    clk: embassy_rp::peripherals::PIN_18,
    mosi: embassy_rp::peripherals::PIN_19,
    miso: embassy_rp::peripherals::PIN_16,
}
async fn test_flash(p: FlashPinTransfer) {
    use embassy_rp::spi::{Config, Phase, Spi};
    let mut cs = Output::new(p.cs, Level::High);
    let mut config = Config::default();
    config.frequency = 10_000_000; // 133MHz max!?
    let mut spi = Spi::new_blocking(p.spi, p.clk, p.mosi, p.miso, config);

    const REG_JEDEC_ID: u8 = 0x9F;

    let mut read = [0u8; 4];
    let mut buf = [0u8; 4];

    // Jedec;
    // [0, c2, 20, 19]
    // Manufacturer id does not match 0xEF??
    // Oh, its not a winbond chip.
    // Macronix MX25L25645G... 256 MBIT

    buf[0] = REG_JEDEC_ID;
    cs.set_low();
    let z = spi.blocking_transfer(&mut read, &buf);
    cs.set_high();
    defmt::info!("Flash jedec: {:?} {:x}", z, read);
    const JEDEC_MACRONIX_ID: u8 = 0xC2;
    const JEDEC_MEMORY_TYPE_MX25L25645G: u8 = 0x20;
    const JEDEC_MEMORY_DENSITY_MX25L25645G: u8 = 0x19;
    // Macronix has manufacturer ID c2.
    if read[1] == JEDEC_MACRONIX_ID
        && read[2] == JEDEC_MEMORY_TYPE_MX25L25645G
        && read[3] == JEDEC_MEMORY_DENSITY_MX25L25645G
    {
        defmt::info!("Flash is responsive!");
    } else {
        defmt::error!("Flash test failed!");
    }
}

struct SdCardPinTransfer {
    spi: embassy_rp::peripherals::SPI0,
    cs: embassy_rp::peripherals::PIN_27,
    clk: embassy_rp::peripherals::PIN_18,
    mosi: embassy_rp::peripherals::PIN_19,
    miso: embassy_rp::peripherals::PIN_16,
    detect: embassy_rp::peripherals::PIN_22,
}
async fn test_sdcard(p: SdCardPinTransfer) {
    use embassy_rp::spi::{Config, Phase, Spi};
    use embedded_hal_bus::spi::ExclusiveDevice;

    //use embedded_hal_bus::spi::ExclusiveDevice;
    use embedded_sdmmc::sdcard::{DummyCsPin, SdCard};
    let detect = Input::new(p.detect, embassy_rp::gpio::Pull::Up);
    // Real cs pin
    let cs = Output::new(p.cs, Level::High);
    let detect_state = detect.get_level();
    let mut config = Config::default();
    config.frequency = 1_000_000; // 133MHz max!?
    defmt::info!("sd card detect pin high: {:?}", detect_state == Level::High);

    if detect_state == Level::Low {
        //let mut cs = Output::new(p.cs, Level::High);
        let spi = Spi::new_blocking(p.spi, p.clk, p.mosi, p.miso, config);
        // Use a dummy cs pin here, for embedded-hal SpiDevice compatibility reasons
        let spi_dev = ExclusiveDevice::new_no_delay(spi, DummyCsPin);
        //
        let sdcard = SdCard::new(spi_dev, cs, embassy_time::Delay);
        defmt::info!("Card size is {} bytes", sdcard.num_bytes().unwrap());
    } else {
        defmt::warn!("No SD card detected, can't test sd card functionaltiy.");
    }
}

pub async fn hw_test(p: Peripherals) -> ! {
    let delay = Duration::from_millis(250);

    const TEST_ICM: bool = false;
    const TEST_LSM: bool = false;
    const TEST_BME: bool = false;
    const TEST_FLASH: bool = false;
    const TEST_SDCARD: bool = false;
    const TEST_BATTERY_VOLTAGE: bool = true;

    if TEST_ICM {
        test_icm(IcmPinTransfer {
            spi: p.SPI0,
            cs: p.PIN_5,
            clk: p.PIN_2,
            mosi: p.PIN_3,
            miso: p.PIN_4,
        })
        .await;
    } else if TEST_FLASH {
        Timer::after(delay).await;
        test_flash(FlashPinTransfer {
            spi: p.SPI0,
            cs: p.PIN_17,
            clk: p.PIN_18,
            mosi: p.PIN_19,
            miso: p.PIN_16,
        })
        .await;
    } else if TEST_SDCARD {
        test_sdcard(SdCardPinTransfer {
            spi: p.SPI0,
            cs: p.PIN_27,
            clk: p.PIN_18,
            mosi: p.PIN_19,
            miso: p.PIN_16,
            detect: p.PIN_22,
        })
        .await;
    }

    if TEST_LSM {
        Timer::after(delay).await;
        test_lsm(LSM6DSV320XPinTransfer {
            spi: p.SPI1,
            cs: p.PIN_13,
            clk: p.PIN_10,
            mosi: p.PIN_11,
            miso: p.PIN_12,
        })
        .await;
    }

    if TEST_BME {
        Timer::after(delay).await;
        test_bme(BmePinTransfer {
            i2c: p.I2C0,
            sda: p.PIN_20,
            scl: p.PIN_21,
        })
        .await;
    }

    if TEST_BATTERY_VOLTAGE {
        use embassy_rp::adc::{Adc, Channel, Config, InterruptHandler};
        use embassy_rp::gpio::Pull;
        fn convert_to_celsius(raw_temp: u16) -> f32 {
            // According to chapter 12.4.6 Temperature Sensor in RP235x datasheet
            let temp = 27.0 - (raw_temp as f32 * 3.3 / 4096.0 - 0.706) / 0.001721;
            let sign = if temp < 0.0 { -1.0 } else { 1.0 };
            let rounded_temp_x10: i16 = ((temp * 10.0) + 0.5 * sign) as i16;
            (rounded_temp_x10 as f32) / 10.0
        }
        bind_interrupts!(struct Irqs {
            ADC_IRQ_FIFO => InterruptHandler;
        });

        let mut adc = Adc::new(p.ADC, Irqs, Config::default());

        let mut p28 = Channel::new_pin(p.PIN_28, Pull::None);
        let mut ts = Channel::new_temp_sensor(p.ADC_TEMP_SENSOR);
        for _ in 0..5 {
            let level = adc.read(&mut p28).await.unwrap();
            defmt::info!("Pin 28 ADC: {}", level);
            let temp = adc.read(&mut ts).await.unwrap();
            defmt::info!("Temp: {} degrees", convert_to_celsius(temp));
            Timer::after_secs(1).await;
        }
    }

    let mut indicator = Output::new(p.PIN_26, Level::Low);
    let delay = Duration::from_millis(250);

    loop {
        defmt::info!("led on!");

        indicator.set_high();

        Timer::after(delay).await;

        defmt::info!("led off!");
    }
}
