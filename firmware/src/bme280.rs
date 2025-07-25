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
impl<I2cError: embedded_hal_async::i2c::Error + core::fmt::Debug> core::fmt::Display
    for Error<I2cError>
{
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        f.write_fmt(format_args!("{:?}", *self))
    }
}

impl<I2cError: embedded_hal_async::i2c::Error + core::fmt::Debug + core::fmt::Display>
    core::error::Error for Error<I2cError>
{
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
    pub temperature: i32,
    pub pressure: i32,
}

#[derive(Debug, Copy, Clone, PartialEq, Default, defmt::Format)]
pub struct CentiCelsius(pub i32);

#[derive(Debug, Copy, Clone, PartialEq, Default, defmt::Format)]
pub struct Measurement {
    pub temperature: CentiCelsius,
    pub pressure: PressureQ248,
}
#[derive(Debug, Copy, Clone, PartialEq, Default, defmt::Format)]
pub struct TemperatureFine(pub i32);
#[derive(Debug, Copy, Clone, PartialEq, Default, defmt::Format)]
pub struct TemperatureCompensation {
    pub temperature: CentiCelsius,
    pub fine: TemperatureFine,
}

#[derive(Copy, Clone, PartialEq, Default, defmt::Format)]
pub struct PressureQ248(pub u32);

impl PressureQ248 {
    pub fn as_f32(&self) -> f32 {
        (self.0 as f32) / 256.0
    }
    pub fn as_f64(&self) -> f64 {
        (self.0 as f64) / 256.0
    }
    pub fn as_u32(&self) -> u32 {
        (self.0 / 256) as u32
    }
    pub fn as_u32_parts(&self) -> (u32, u8) {
        ((self.0 / 256) as u32, (self.0 & 0xff) as u8)
    }
}
impl core::fmt::Debug for PressureQ248 {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        let (i, r) = self.as_u32_parts();
        f.debug_struct("PressureQ248")
            .field("i", &i)
            .field("r", &r)
            .finish()
    }
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
impl Compensation {
    pub fn compensate_temp(&self, read_temp: i32) -> TemperatureCompensation {
        let dig_t1 = self.dig_t1 as i32;
        let dig_t2 = self.dig_t2 as i32;
        let dig_t3 = self.dig_t3 as i32;
        let var1: i32 = (((read_temp >> 3) - (dig_t1 << 1)) * (dig_t2)) >> 11;
        let intermediate: i32 = (read_temp >> 4) - dig_t1; // does this hurt precision with an intermediate??
        let var2: i32 = (((intermediate * intermediate) >> 12) * dig_t3) >> 14;
        let t_fine = var1 + var2;
        let t = (t_fine * 5 + 128) >> 8;

        TemperatureCompensation {
            temperature: CentiCelsius(t),
            fine: TemperatureFine(t_fine),
        }
    }
    pub fn compensate_pressure(
        &self,
        temperature_fine: TemperatureFine,
        read_pressure: i32,
    ) -> PressureQ248 {
        let dig_p1 = self.dig_p1 as i64;
        let dig_p2 = self.dig_p2 as i64;
        let dig_p3 = self.dig_p3 as i64;
        let dig_p4 = self.dig_p4 as i64;
        let dig_p5 = self.dig_p5 as i64;
        let dig_p6 = self.dig_p6 as i64;
        let dig_p7 = self.dig_p7 as i64;
        let dig_p8 = self.dig_p8 as i64;
        let dig_p9 = self.dig_p9 as i64;
        let var1 = (temperature_fine.0 as i64) - 128000;
        let var2 = var1 * var1 * dig_p6;
        let var2 = var2 + ((var1 * dig_p5) << 17);
        let var2 = var2 + (dig_p4 << 35);
        let var1 = ((var1 * var1 * dig_p3) >> 8) + ((var1 * dig_p2) << 12);
        let var1 = (((1 << 47) + var1) * dig_p1) >> 33;
        if var1 == 0 {
            return PressureQ248(0);
        }
        let p = 1048576 - (read_pressure as i64);
        let p = (((p << 31) - var2) * 3125) / var1;
        let var1 = ((dig_p9) * (p >> 13) * (p >> 13)) >> 25;
        let var2 = (dig_p8 * p) >> 19;
        let p = ((p + var1 + var2) >> 8) + (dig_p7 << 4);
        let p = p as u32;
        PressureQ248(p)
    }

    pub fn compensate(&self, readout: &Readout) -> Measurement {
        let compensated_t = self.compensate_temp(readout.temperature);
        let pressure = self.compensate_pressure(compensated_t.fine, readout.pressure);
        Measurement {
            temperature: compensated_t.temperature,
            pressure,
        }
    }
}

#[derive(Clone, PartialEq)]
pub struct BME280<I2c> {
    bus: I2c,
    address: SevenBitAddress,
    compensation: Compensation,
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
            let compensation = Self::get_compensation(address, &mut i2c).await?;
            Ok(BME280 {
                address,
                bus: i2c,
                compensation,
            })
        } else {
            Err(Error::IncorrectId)
        }
    }

    pub fn compensation(&self) -> &Compensation {
        &self.compensation
    }

    pub async fn get_compensation(
        address: SevenBitAddress,
        bus: &mut I2C,
    ) -> Result<Compensation, Error<I2cError>> {
        let mut compensation: Compensation = Default::default();

        // Block 0 first.
        {
            let mut read = [0u8; (0xa1 - 0x88) + 1];
            let buf = [reg::REG_BME280_CALIB_00];
            bus.write_read(address, &buf, &mut read).await?;
            // First register holds 7:0, second register holds 15:8,
            compensation.dig_t1 = u16::from_le_bytes([read[0], read[1]]);
            compensation.dig_t2 = i16::from_le_bytes([read[2], read[3]]);
            compensation.dig_t3 = i16::from_le_bytes([read[4], read[5]]);
            // And then the pressure registers.
            compensation.dig_p1 = u16::from_le_bytes([read[6], read[7]]);
            compensation.dig_p2 = i16::from_le_bytes([read[8], read[9]]);
            compensation.dig_p3 = i16::from_le_bytes([read[10], read[11]]);
            compensation.dig_p4 = i16::from_le_bytes([read[12], read[13]]);
            compensation.dig_p5 = i16::from_le_bytes([read[14], read[15]]);
            compensation.dig_p6 = i16::from_le_bytes([read[16], read[17]]);
            compensation.dig_p7 = i16::from_le_bytes([read[18], read[19]]);
            compensation.dig_p8 = i16::from_le_bytes([read[20], read[21]]);
            compensation.dig_p9 = i16::from_le_bytes([read[22], read[23]]);
        }

        // And then block 26.
        {
            let mut read = [0u8; (0xF0 - 0xE1) + 1];
            let buf = [reg::REG_BME280_CALIB_26];
            bus.write_read(address, &buf, &mut read).await?;
            //println!("REG_BME280_CALIB_26: {read:?}");
        }

        Ok(compensation)
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
        defmt::debug!("buf: {:?}", buf);
        let pressure = ((buf[0] as i32) << 12) | ((buf[1] as i32) << 4) | ((buf[2] as i32) >> 4);
        let temperature = ((buf[3] as i32) << 12) | ((buf[4] as i32) << 4) | ((buf[5] as i32) >> 4);
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

#[cfg(test)]
mod test {
    use super::*;
    use embedded_hal_async::i2c;
    use embedded_hal_async::i2c::ErrorType;
    use reg::*;

    impl ErrorType for MockedBME280 {
        type Error = i2c::ErrorKind;
    }
    use std::collections::hash_map::Entry;
    use std::collections::HashMap;
    struct MockedBME280 {
        registers: HashMap<u8, Box<[u8]>>,
    }
    impl MockedBME280 {
        pub fn from(entries: &[(u8, Box<[u8]>)]) -> Self {
            Self {
                registers: entries
                    .into_iter()
                    .chain([(REG_BME280_ID, (vec![0x60u8].into_boxed_slice()))].iter())
                    .cloned()
                    .collect(),
            }
        }
    }

    impl embedded_hal_async::i2c::I2c for MockedBME280 {
        async fn read(&mut self, _address: u8, _buffer: &mut [u8]) -> Result<(), Self::Error> {
            todo!();
        }

        async fn write(&mut self, _address: u8, _bytes: &[u8]) -> Result<(), Self::Error> {
            todo!();
        }

        async fn write_read(
            &mut self,
            _address: u8,
            bytes: &[u8],
            buffer: &mut [u8],
        ) -> Result<(), Self::Error> {
            // first write register is the address that's being read.
            let read_reg = bytes.get(0).expect("expected register address");
            if let Entry::Occupied(e) = self.registers.entry(*read_reg) {
                // e MUST be longer than buffer, else it is an error and we lack data.
                buffer.copy_from_slice(&e.get()[0..buffer.len()]);
                return Ok(());
            }
            // Okay, so no exact match, we could do smarts here and iterate to see if we can serve this register.

            todo!()
        }

        async fn transaction<'a>(
            &mut self,
            _address: u8,
            _operations: &mut [i2c::Operation<'a>],
        ) -> Result<(), Self::Error> {
            todo!();
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

    fn s1_readout(raw_read: [u8; 8]) -> Vec<(u8, Box<[u8]>)> {
        vec![
            (
                REG_BME280_CALIB_00,
                vec![
                    84, 109, 175, 103, 50, 0, 239, 139, 117, 214, 208, 11, 203, 30, 139, 255, 249,
                    255, 180, 45, 232, 209, 136, 19, 0, 75,
                ]
                .into_boxed_slice(),
            ),
            (
                REG_BME280_CALIB_26,
                vec![
                    128, 1, 0, 16, 45, 3, 30, 181, 65, 255, 255, 255, 255, 255, 255, 255,
                ]
                .into_boxed_slice(),
            ),
            (REG_BME280_CTRL_HUM, vec![1, 0, 36, 0].into_boxed_slice()),
            (REG_BME280_PRESS_MSB, raw_read.to_vec().into_boxed_slice()),
        ]
    }

    /// Just after starting, probably warmer than the others.
    fn first_capture() -> MockedBME280 {
        MockedBME280::from(&s1_readout([88, 221, 0, 129, 1, 0, 93, 1]))
    }

    ///  pressure/temp up, finger on the sensor at least.
    fn second_capture_finger() -> MockedBME280 {
        MockedBME280::from(&s1_readout([89, 209, 0, 132, 82, 0, 101, 117]))
    }
    ///  probably colder
    fn third_capture_colder() -> MockedBME280 {
        MockedBME280::from(&s1_readout([88, 149, 0, 128, 87, 0, 92, 57]))
    }

    #[test]
    fn test_conversions() -> Result<(), Box<dyn std::error::Error>> {
        let cases = [
            (
                first_capture(),
                Measurement {
                    temperature: CentiCelsius(2550),
                    pressure: PressureQ248(24943604),
                },
            ),
            (
                second_capture_finger(),
                Measurement {
                    temperature: CentiCelsius(2980),
                    pressure: PressureQ248(24945210),
                },
            ),
            (
                third_capture_colder(),
                Measurement {
                    temperature: CentiCelsius(2464),
                    pressure: PressureQ248(24959691),
                },
            ),
        ];
        for (bme_i2c, expected) in cases {
            let mut bme = smol::block_on(async { BME280::new(ADDRESS_DEFAULT, bme_i2c).await })?;
            let readout = smol::block_on(async { bme.readout().await })?;
            println!("readout: {:?}", readout);
            let measurement = bme.compensation().compensate(&readout);
            println!("measurement: {:?}", measurement);
            assert_eq!(measurement.temperature, expected.temperature);
            assert_eq!(measurement.pressure, expected.pressure);
        }

        Ok(())
    }
}
