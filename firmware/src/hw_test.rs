#![allow(dead_code)]
use embassy_executor::Spawner;
use embassy_rp::gpio::{Input, Level, Output};
use embassy_rp::Peri;

use embassy_rp::{bind_interrupts, Peripherals};
use embassy_time::{Duration, Timer};

struct IcmPinTransfer {
    spi: Peri<'static, embassy_rp::peripherals::SPI0>,
    cs: Peri<'static, embassy_rp::peripherals::PIN_5>,
    clk: Peri<'static, embassy_rp::peripherals::PIN_2>,
    mosi: Peri<'static, embassy_rp::peripherals::PIN_3>,
    miso: Peri<'static, embassy_rp::peripherals::PIN_4>,
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
    spi: Peri<'static, embassy_rp::peripherals::SPI1>,
    cs: Peri<'static, embassy_rp::peripherals::PIN_13>,
    clk: Peri<'static, embassy_rp::peripherals::PIN_10>,
    mosi: Peri<'static, embassy_rp::peripherals::PIN_11>,
    miso: Peri<'static, embassy_rp::peripherals::PIN_12>,
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
    i2c: Peri<'static, embassy_rp::peripherals::I2C0>,
    sda: Peri<'static, embassy_rp::peripherals::PIN_20>,
    scl: Peri<'static, embassy_rp::peripherals::PIN_21>,
}
async fn test_bme(p: BmePinTransfer) {
    use embassy_rp::i2c::{Config, I2c};
    let config = Config::default();
    //config.frequency = 100_000;
    let mut i2c = I2c::new_blocking(p.i2c, p.scl, p.sda, config);

    const I2C_BUS_ADDRESS: u8 = 0x76;
    const REG_CHIP_ID: u8 = 0xD0;
    let mut read = [0u8; 1];
    let mut buf = [0u8; 1];
    buf[0] = REG_CHIP_ID;
    let z = i2c.blocking_write_read(I2C_BUS_ADDRESS, &buf, &mut read);

    defmt::info!("BME who am i: {:?} {:x}", z, read);
    if read[0] == 0x60 {
        defmt::info!("BME is responsive!");
    } else {
        defmt::error!("BME test failed!");
    }
}
struct FlashPinTransfer {
    spi: Peri<'static, embassy_rp::peripherals::SPI0>,
    cs: Peri<'static, embassy_rp::peripherals::PIN_17>,
    clk: Peri<'static, embassy_rp::peripherals::PIN_18>,
    mosi: Peri<'static, embassy_rp::peripherals::PIN_19>,
    miso: Peri<'static, embassy_rp::peripherals::PIN_16>,
}
async fn test_flash(p: FlashPinTransfer) {
    use embassy_rp::spi::{Config, Spi};
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
    spi: Peri<'static, embassy_rp::peripherals::SPI0>,
    cs: Peri<'static, embassy_rp::peripherals::PIN_27>,
    clk: Peri<'static, embassy_rp::peripherals::PIN_18>,
    mosi: Peri<'static, embassy_rp::peripherals::PIN_19>,
    miso: Peri<'static, embassy_rp::peripherals::PIN_16>,
    detect: Peri<'static, embassy_rp::peripherals::PIN_22>,
}
async fn test_sdcard(p: SdCardPinTransfer) {
    use embassy_rp::spi::{Config, Spi};
    use embedded_hal_bus::spi::ExclusiveDevice;

    //use embedded_hal_bus::spi::ExclusiveDevice;
    use embedded_sdmmc::sdcard::{DummyCsPin, SdCard};
    let detect = Input::new(p.detect, embassy_rp::gpio::Pull::Up);
    // Real cs pin
    let cs = Output::new(p.cs, Level::High);
    let detect_state = detect.get_level();
    let mut config = Config::default();
    config.frequency = 24_000_000; // 133MHz max!?
    defmt::info!("sd card detect pin high: {:?}", detect_state == Level::High);

    if detect_state == Level::Low {
        //let mut cs = Output::new(p.cs, Level::High);
        let spi = Spi::new_blocking(p.spi, p.clk, p.mosi, p.miso, config);
        // Use a dummy cs pin here, for embedded-hal SpiDevice compatibility reasons
        let spi_dev = ExclusiveDevice::new_no_delay(spi, DummyCsPin);
        //
        let sdcard = SdCard::new(spi_dev, cs, embassy_time::Delay);

        let mut indicator = Output::new(unsafe { Peripherals::steal().PIN_26 }, Level::Low);

        loop {
            indicator.set_high();
            let bytes = sdcard.num_bytes();
            indicator.set_low();
            Timer::after_millis(1).await;
        }
        // defmt::info!("Card size is {} bytes", sdcard.num_bytes().unwrap());
    } else {
        defmt::warn!("No SD card detected, can't test sd card functionaltiy.");
    }
}

struct MicPinTransfer {
    pio_dev: Peri<'static, embassy_rp::peripherals::PIO1>,
    dma_chan: Peri<'static, embassy_rp::peripherals::DMA_CH0>,
    ws: Peri<'static, embassy_rp::peripherals::PIN_7>,
    clk: Peri<'static, embassy_rp::peripherals::PIN_8>,
    data: Peri<'static, embassy_rp::peripherals::PIN_9>,
}
async fn test_mic(p: MicPinTransfer) {
    use embassy_rp::pio::Pio;

    //return;
    // ICS 43434
    // Slave data port's format is i2s, two's complement.
    // 64 SCK cycles for seach WS sterio frame.
    // When set to left; data will be output following WS's falling edge.
    // When set to right; data will be output following WS's rising edge.
    // In this format, the MSB of each word is delayed by one SCK cycle from
    // the start of each half-frame.
    // There is no commonly accepted unit of measurement to express the instantaneous level of a digital signal output
    // Uhh, may be worth reading that AN-1112 Application Note, Microphone Specifications Explained.
    // AN-1140, Microphone Array Beamforming <- also interesting.
    //
    // Low power mode is entered when the sampling frequency is between 6.25 and 18.75 kHz. Below 3.125 kHz is goes into
    // sleep.
    // LR is grounded, so we're in the left frame.
    defmt::error!("Microphone test with i2s and all that.");
    // Okay, so as long as we bit-bang higher than 3.125 kHz, we should get some data.
    //let mut ws = Output::new(p.ws, Level::High);
    //let mut clk = Output::new(p.clk, Level::High);
    //let input = Input::new(p.data, embassy_rp::gpio::Pull::None);

    // lets target 200 kHz clock.
    //let nanos_per_second = 1_000_000_000;
    //let target_clock_delay_ns = nanos_per_second / (48_000 * 64);
    let clock_half = Duration::from_nanos(600);
    defmt::info!("clock_half: {:#?}", clock_half);
    Timer::after_millis(100).await;

    let mut counter = 0;
    // At lower sample rate the serial port can keep up, but there's definitely clipping going on.
    // cat /dev/ttyACM1 | pacat --rate=24000 --channels=2 --format=s32le --raw --volume=32000 -v
    const SAMPLE_RATE: u32 = 24_000;
    const BIT_DEPTH: u32 = 32;
    const CHANNELS: u32 = 2;
    const DUMP_SAMPLES_TO_SERIAL_LOOP: bool = false;

    Timer::after_millis(100).await;
    defmt::debug!("after setup");
    bind_interrupts!(struct Irqs {

        PIO1_IRQ_0 => embassy_rp::pio:: InterruptHandler<embassy_rp::peripherals::PIO1>;
    });

    // Setup pio state machine for i2s output
    let Pio {
        mut common, sm1, ..
    } = Pio::new(p.pio_dev, Irqs);

    Timer::after_millis(100).await;
    defmt::debug!("pio created");
    let bit_clock_pin = p.clk;
    let left_right_clock_pin = p.ws;
    let data_pin = p.data;

    Timer::after_millis(100).await;
    defmt::debug!("loading program");
    use crate::i2s_input::{PioI2sIn, PioI2sInProgram};
    let program = PioI2sInProgram::new(&mut common);

    for _ in 0..5 {
        Timer::after_millis(100).await;
        defmt::debug!("after program");
    }

    Timer::after_millis(100).await;
    for _ in 0..5 {
        Timer::after_millis(100).await;
        defmt::debug!("making i2s new in");
    }

    let mut i2s = PioI2sIn::new(
        &mut common,
        sm1,
        p.dma_chan,
        data_pin,
        bit_clock_pin,
        left_right_clock_pin,
        SAMPLE_RATE,
        BIT_DEPTH,
        CHANNELS,
        &program,
    );
    Timer::after_millis(100).await;
    for _ in 0..5 {
        Timer::after_millis(100).await;
        defmt::debug!("after new in");
    }

    use static_cell::StaticCell;

    // create two audio buffers (back and front) which will take turns being
    // filled with new audio data and being sent to the pio fifo using dma
    const BUFFER_SIZE: usize = 960;
    static DMA_BUFFER: StaticCell<[u32; BUFFER_SIZE * 2]> = StaticCell::new();
    let dma_buffer = DMA_BUFFER.init_with(|| [0u32; BUFFER_SIZE * 2]);
    let (mut back_buffer, mut front_buffer) = dma_buffer.split_at_mut(BUFFER_SIZE);

    const CHUNKS_TO_COLLECT: usize = 20;
    let huge_buffer = {
        static DMA_BUFFER: StaticCell<[i32; BUFFER_SIZE * 20]> = StaticCell::new();
        DMA_BUFFER.init_with(|| [0i32; BUFFER_SIZE * 20])
    };

    if DUMP_SAMPLES_TO_SERIAL_LOOP {
        loop {
            let dma_future = i2s.read(front_buffer);

            dma_future.await;

            core::mem::swap(&mut back_buffer, &mut front_buffer);

            // Shift the buffer by one byte, this ensures that the sign byte is at the correct place.
            for z in back_buffer.iter_mut() {
                *z = *z << 1;
            }

            // make an i32 window.
            let back_u32 = unsafe {
                core::slice::from_raw_parts(back_buffer.as_ptr().cast::<u8>(), 4 * BUFFER_SIZE)
            };
            unsafe { crate::defmt_serial::push_serial(&back_u32) };
            Timer::after_micros(10).await;
        }
    }
    for i in 0..CHUNKS_TO_COLLECT {
        // trigger transfer of front buffer data to the pio fifo
        // but don't await the returned future, yet
        let dma_future = i2s.read(front_buffer);

        // now await the dma future. once the dma finishes, the next buffer needs to be queued
        // within DMA_DEPTH / SAMPLE_RATE = 8 / 48000 seconds = 166us
        dma_future.await;

        core::mem::swap(&mut back_buffer, &mut front_buffer);

        counter += 1;
        if counter % 100 == 0 || true {
            // make an i32 window.
            let back_i32 = unsafe {
                core::slice::from_raw_parts(back_buffer.as_ptr().cast::<i32>(), BUFFER_SIZE)
            };
            //defmt::info!("i2s bitbang data: {:#?}", back_i32);
            huge_buffer[(i * BUFFER_SIZE)..(i + 1) * BUFFER_SIZE].copy_from_slice(&back_i32);
        }
    }
    for i in 0..CHUNKS_TO_COLLECT {
        // We have 32 bits of data now, but the relevant part is the left 24 bits, the LSB section between bit 26 and
        // bit 32 is empty.
        let mut buffer = [0i32; BUFFER_SIZE];
        for (i, v) in huge_buffer[(i * BUFFER_SIZE)..(i + 1) * BUFFER_SIZE]
            .iter()
            .enumerate()
        {
            buffer[i] = v << 1; // Why do we need this? :<
        }

        defmt::info!("buff {}: {:#x}", i, buffer);
    }
    loop {
        Timer::after_millis(100).await;
    }
}

use cyw43_pio::PioSpi;
use embassy_rp::peripherals::{DMA_CH0, PIO0};

#[embassy_executor::task]
async fn cyw43_task(
    runner: cyw43::Runner<'static, Output<'static>, PioSpi<'static, PIO0, 0, DMA_CH0>>,
) -> ! {
    runner.run().await
}

pub async fn test_wifi(p: Peripherals, spawner: Spawner) -> ! {
    use cyw43_pio::{PioSpi, RM2_CLOCK_DIVIDER};
    use defmt::info;
    use embassy_rp::peripherals::PIO0;
    use embassy_rp::pio::{InterruptHandler as PioInterruptHandler, Pio};
    bind_interrupts!(struct Irqs {
        PIO0_IRQ_0 => PioInterruptHandler<PIO0>;
    });
    use static_cell::StaticCell;

    let (fw, clm) =
        if let Some(p) = crate::rp2350_util::rom_data::get_partition_by_name("static_files") {
            let (start, end) = p.get_first_last_bytes();
            let len = end - start;
            let partition_data =
                unsafe { crate::rp2350_util::xip::flash_slice(start as usize, len as usize) };
            let reader = crate::static_files::StaticFileReader::new(partition_data);

            let fw = reader.file_data("43439A0.bin");
            let clm = reader.file_data("43439A0_clm.bin");

            if fw.is_some() && clm.is_some() {
                (fw.unwrap(), clm.unwrap())
            } else {
                defmt::warn!("Did not find necessary fw.");
                loop {
                    Timer::after_millis(100).await;
                }
            }
        } else {
            defmt::warn!("Could not find static files and load firmware.");
            loop {
                Timer::after_millis(100).await;
            }
        };

    Timer::after_millis(100).await;

    defmt::println!("clm start: {}", &clm[0..20]);
    defmt::println!("fw start: {}", &fw[0..20]);
    Timer::after_millis(100).await;
    Timer::after_millis(100).await;

    let pwr = Output::new(p.PIN_23, Level::Low);
    let cs = Output::new(p.PIN_25, Level::High);
    let mut pio = Pio::new(p.PIO0, Irqs);
    let spi = PioSpi::new(
        &mut pio.common,
        pio.sm0,
        RM2_CLOCK_DIVIDER,
        //cyw43_pio::DEFAULT_CLOCK_DIVIDER,
        pio.irq0,
        cs,
        p.PIN_24, // dio
        p.PIN_29, // clk
        p.DMA_CH0,
    );

    info!("doing things");

    static STATE: StaticCell<cyw43::State> = StaticCell::new();
    let state = STATE.init(cyw43::State::new());

    info!("cell made");

    // This looks to be where the firmware upload happens.
    let (_net_device, mut control, runner) = cyw43::new(state, pwr, spi, fw).await;

    info!("new cyw43");

    // This is where we stall.
    let s = spawner.spawn(cyw43_task(runner));
    if let Err(e) = s {
        info!("setup failed: {:?}", e);
    } else {
        info!("setup good");
    }

    control.init(clm).await;
    control
        .set_power_management(cyw43::PowerManagementMode::PowerSave)
        .await;

    let delay = Duration::from_millis(250);
    loop {
        defmt::info!("led on!");

        control.gpio_set(0, true).await;

        Timer::after(delay).await;

        defmt::info!("led off!");
        control.gpio_set(0, false).await;
        Timer::after(delay).await;
    }
}

pub async fn hw_test(p: Peripherals) -> ! {
    let delay = Duration::from_millis(250);

    Timer::after(delay).await;
    const TEST_ICM: bool = false;
    const TEST_LSM: bool = false;
    const TEST_BME: bool = false;
    const TEST_FLASH: bool = false;
    const TEST_SDCARD: bool = false;
    const TEST_BATTERY_VOLTAGE: bool = false;
    const TEST_MIC: bool = false;
    const TEST_COLLECT_ENTROPY: bool = true;

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
        /*
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
        */
    }
    if TEST_COLLECT_ENTROPY {
        for _ in 0..10 {
            defmt::info!("RNG: {}", embassy_rp::clocks::RoscRng::next_u8());
            Timer::after_secs(1).await;
            // value 164 occurs a LOT... but that could be because we have this 1 s periodic wait, if we just pull some
            // values from it and then wait a bit, pull some more values, it's probably fine.
        }
        //        defmt::info!("RNG: {:x}", embassy_rp::clocks::RoscRng {}.next_u32());
        let mut rng = crate::rp2350_util::random_util::instantiate_rng();
        for _ in 0..10 {
            use rand::Rng;
            defmt::info!("RNG: {:x}", rng.next_u32());
        }
    }

    if TEST_MIC {
        test_mic(MicPinTransfer {
            pio_dev: p.PIO1,
            dma_chan: p.DMA_CH0,
            ws: p.PIN_7,
            clk: p.PIN_8,
            data: p.PIN_9,
        })
        .await;
    }

    let mut indicator = Output::new(p.PIN_26, Level::Low);
    let delay = Duration::from_millis(250);

    loop {
        defmt::info!("led on!");

        indicator.set_high();

        Timer::after(delay).await;

        defmt::info!("led off!");
        indicator.set_low();

        Timer::after(delay).await;
    }
}
