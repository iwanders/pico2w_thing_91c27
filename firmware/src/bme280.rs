use embedded_hal_async::i2c::I2c;
use embedded_hal_async::i2c::SevenBitAddress;

// Address is right-aligned; https://docs.rs/embedded-hal-async/latest/embedded_hal_async/i2c/type.SevenBitAddress.html

// https://github.com/ryankurte/rust-embedded-driver old, but definitely somewhat helpful.

// embedded_hal_async::delay Hmm, we probably want this to ensure we can easily wait until a measurement is done.

pub mod reg {
    // Humidity
    pub const REG_BME280_HUM_LSB: u8 = 0xFE;
    pub const REG_BME280_HUM_MSB: u8 = 0xFD;
    // Temperature
    pub const REG_BME280_TEMP_XLSB: u8 = 0xFC;
    pub const REG_BME280_TEMP_LSB: u8 = 0xFB;
    pub const REG_BME280_TEMP_MSB: u8 = 0xFA;
    // Pressure
    pub const REG_BME280_PRESS_XLSB: u8 = 0xF9;
    pub const REG_BME280_PRESS_LSB: u8 = 0xF8;
    pub const REG_BME280_PRESS_MSB: u8 = 0xF7;
    // Misc
    pub const REG_BME280_CONFIG: u8 = 0xF5;
    pub const REG_BME280_CTRL_MEAS: u8 = 0xF4;
    pub const REG_BME280_STATUS: u8 = 0xF3;
    pub const REG_BME280_CTRL_HUM: u8 = 0xF2;
    // Whoa, 26..41 calibration data.
    // misc continues.
    pub const REG_BME280_CALIB_26: u8 = 0xE1;
    // Reset and id.
    pub const REG_BME280_RESET: u8 = 0xE0;
    pub const REG_BME280_ID: u8 = 0xd0;
    // More calibration data.
    //
    pub const REG_BME280_CALIB_00: u8 = 0x88;
}

pub const ADDRESS_DEFAULT: SevenBitAddress = 0x76;
pub const ADDRESS_ALTERNATE: SevenBitAddress = 0x77; // untested.

// defmt::Format only really works if embedded_hal_async has the defmt feature.
#[derive(Debug, Copy, Clone, PartialEq, defmt::Format)]
pub enum Error<I2cError: embedded_hal_async::i2c::Error> {
    /// Underlying I2C device error
    I2c(I2cError),
    /// An incorrect id was returned from the ID register, device address collision?
    IncorrectId,
}
impl<I2cError: embedded_hal_async::i2c::Error> From<I2cError> for Error<I2cError> {
    fn from(e: I2cError) -> Self {
        Error::<I2cError>::I2c(e)
    }
}

/*
impl<I2cError: embedded_hal_async::i2c::Error> defmt::Format for Error<I2cError> {
    fn format(&self, f: defmt::Formatter) {
        match *self {
            Error::I2c(ref e) => {
                defmt::write!(f, "Error:I2c({:?})", defmt::Debug2Format(&e.kind()))
            }
            Error::IncorrectId => defmt::write!(f, "Error:ResetTimeout",),
        }
    }
}*/

#[derive(Clone, PartialEq)]
pub struct BME280<I2c> {
    bus: I2c,
    address: SevenBitAddress,
}
impl<I2c> core::fmt::Debug for BME280<I2c> {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        f.write_fmt(format_args!("<BME280:0x{:x}>", self.address))
    }
}
impl<I2c> defmt::Format for BME280<I2c> {
    fn format(&self, f: defmt::Formatter) {
        defmt::write!(f, "<BME280:0x{:x}>", self.address)
    }
}

/*
Three modes, sleep is default, forced does one measurement and goes back to sleep, and normal which periodically
measures.

Measurement cycle performs temperature, pressure and humidity in sequence, but they can optionally be skipped.

They are fed to the IIR filter if enabled.
*/

#[derive(Debug, Copy, Clone, PartialEq, Default, defmt::Format)]
#[repr(u8)]
pub enum Mode {
    #[default]
    Sleep = 0b00,
    Forced = 0b01,
    Normal = 0b11,
}
#[derive(Debug, Copy, Clone, PartialEq, Default, defmt::Format)]
#[repr(u8)]
pub enum Sampling {
    Skipped = 0b000,
    /// 1x oversampling
    #[default]
    X1 = 0b001,
    /// 2x oversampling
    X2 = 0b010,
    X4 = 0b011,
    X8 = 0b100,
    X16 = 0b101,
}

/// Readout structure of all measurements.
///
/// We always just read everything, because as the datasheet says also reading humidity is almost as fast.
#[derive(Debug, Copy, Clone, PartialEq, Default, defmt::Format)]
pub struct Readout {
    pub humidity: u16,
    pub temperature: u32,
    pub pressure: u32,
}

/// Compensation trimming paramaters
#[derive(Debug, Copy, Clone, PartialEq, Default, defmt::Format)]
pub struct Compensation {
    pub dig_t1: u16, // 0x88, 0x89
    pub dig_t2: i16, // 0x8a, 0x8b
    pub dig_t3: i16, // 0x8c, 0x8d

    pub dig_p1: u16, // 0x8e, 0x8f
    pub dig_p2: i16, // ...
    pub dig_p3: i16, // ...
    pub dig_p4: i16, // ...
    pub dig_p5: i16, // ...
    pub dig_p6: i16, // ...
    pub dig_p7: i16, // ...
    pub dig_p8: i16, // ...
    pub dig_p9: i16, // 0x9e, 0x9f

    pub dig_h1: u8,  // 0xA1
    pub dig_h2: i16, // 0xE1, 0xE2
    pub dig_h3: u8,  // 0xe3
    pub dig_h4: i16, // 0xe4, 0xe5[3:0]
    pub dig_h5: i16, // 0xe5[7:4], 0xe6
}

impl<I2C, I2cError: embedded_hal_async::i2c::Error> BME280<I2C>
where
    I2C: I2c<Error = I2cError>,
    I2cError: core::fmt::Debug,
{
    pub async fn new(address: SevenBitAddress, i2c: I2C) -> Result<Self, Error<I2cError>> {
        let mut i2c = i2c;
        let mut read = [0u8; 1];
        let buf = [reg::REG_BME280_ID];
        i2c.write_read(address, &buf, &mut read).await?;
        if read[0] == 0x60 {
            Ok(BME280 { address, bus: i2c })
        } else {
            Err(Error::IncorrectId)
        }
    }

    pub async fn set_ctrl_meas(
        &mut self,
        temp_sampling: Sampling,
        press_sampling: Sampling,
        mode: Mode,
    ) -> Result<(), Error<I2cError>> {
        let reg_value = (temp_sampling as u8) << 5 | (press_sampling as u8) << 2 | mode as u8;
        let buf = [reg::REG_BME280_CTRL_MEAS, reg_value];
        self.bus
            .write(self.address, &buf)
            .await
            .map_err(|e| e.into())
    }

    /// Changes to this register only become active after the ctrl register is set.
    pub async fn set_ctrl_hum(&mut self, hum_sampling: Sampling) -> Result<(), Error<I2cError>> {
        self.bus
            .write(
                self.address,
                &[reg::REG_BME280_CTRL_HUM, hum_sampling as u8],
            )
            .await
            .map_err(|e| e.into())
    }

    pub async fn reset(&mut self) -> Result<(), Error<I2cError>> {
        let buf = [reg::REG_BME280_RESET, 0xB6]; // write magic value to trigger reset.
        self.bus
            .write(self.address, &buf)
            .await
            .map_err(|e| e.into())
    }

    pub async fn readout(&mut self) -> Result<Readout, Error<I2cError>> {
        let mut buf = [0u8; 8]; // write magic value to trigger reset.
        self.bus
            .write_read(self.address, &[reg::REG_BME280_PRESS_MSB], &mut buf)
            .await
            .map_err(|e| core::convert::Into::<Error<I2cError>>::into(e))?;
        defmt::warn!("buf: {:?}", buf);
        let pressure = u32::from_be_bytes([buf[0], buf[1], buf[2], 0]) >> 12;
        let temperature = u32::from_be_bytes([buf[3], buf[4], buf[5], 0]) >> 12;
        let humidity = u16::from_be_bytes([buf[6], buf[7]]);
        Ok(Readout {
            temperature,
            humidity,
            pressure,
        })
    }

    pub async fn get_register(&mut self, register: u8) -> Result<u8, Error<I2cError>> {
        let mut read = [0u8; 1];
        let buf = [register];
        self.bus.write_read(self.address, &buf, &mut read).await?;
        Ok(read[0])
    }

    /// This is a dump function that gets raw register values, dumps them through defmt, for debugging.
    pub async fn dump_registers(&mut self) -> Result<(), Error<I2cError>> {
        // Obtain first calibration data:

        {
            let mut read = [0u8; (0xa1 - 0x88) + 1];
            let buf = [reg::REG_BME280_CALIB_00];
            self.bus.write_read(self.address, &buf, &mut read).await?;
            defmt::debug!("(REG_BME280_CALIB_00, {})", read);
        }

        {
            let mut read = [0u8; (0xF0 - 0xE1) + 1];
            let buf = [reg::REG_BME280_CALIB_26];
            self.bus.write_read(self.address, &buf, &mut read).await?;
            defmt::debug!("(REG_BME280_CALIB_26, {})", read);
        }

        {
            let mut read = [0u8; 4];
            let buf = [reg::REG_BME280_CTRL_HUM];
            self.bus.write_read(self.address, &buf, &mut read).await?;
            defmt::debug!("(REG_BME280_CTRL_HUM, {})", read);
        }

        {
            let mut read = [0u8; 8];
            let buf = [reg::REG_BME280_PRESS_MSB];
            self.bus.write_read(self.address, &buf, &mut read).await?;
            defmt::debug!("(REG_BME280_PRESS_MSB, {})", read);
        }

        Ok(())
    }
}

/*
 *
 7.610433 DEBUG (REG_BME280_CALIB_00, [84, 109, 175, 103, 50, 0, 239, 139, 117, 214, 208, 11, 203, 30, 139, 255, 249, 255, 180, 45, 232, 209, 136, 19, 0, 75])
 7.610695 DEBUG (REG_BME280_CALIB_26, [128, 1, 0, 16, 45, 3, 30, 181, 65, 255, 255, 255, 255, 255, 255, 255])
 7.610820 DEBUG (REG_BME280_CTRL_HUM, [1, 0, 36, 0])
 7.610961 DEBUG (REG_BME280_PRESS_MSB, [88, 221, 0, 129, 1, 0, 93, 1])

 pressure/temp up, finger on the sensor at least.
 280.572000 DEBUG (REG_BME280_CALIB_00, [84, 109, 175, 103, 50, 0, 239, 139, 117, 214, 208, 11, 203, 30, 139, 255, 249, 255, 180, 45, 232, 209, 136, 19, 0, 75])
 280.572264 DEBUG (REG_BME280_CALIB_26, [128, 1, 0, 16, 45, 3, 30, 181, 65, 255, 255, 255, 251, 255, 255, 255])
 280.572388 DEBUG (REG_BME280_CTRL_HUM, [1, 0, 36, 0])
 280.572529 DEBUG (REG_BME280_PRESS_MSB, [89, 209, 0, 132, 82, 0, 101, 117])

 probably colder:
 6.679185 DEBUG (REG_BME280_CALIB_00, [84, 109, 175, 103, 50, 0, 239, 139, 117, 214, 208, 11, 203, 30, 139, 255, 249, 255, 180, 45, 232, 209, 136, 19, 0, 75])
 6.679447 DEBUG (REG_BME280_CALIB_26, [128, 1, 0, 16, 45, 3, 30, 181, 65, 255, 255, 255, 255, 255, 255, 255])
 6.679570 DEBUG (REG_BME280_CTRL_HUM, [1, 0, 36, 0])
 6.679710 DEBUG (REG_BME280_PRESS_MSB, [88, 149, 0, 128, 87, 0, 92, 57])

*/
