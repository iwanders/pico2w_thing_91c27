use std::time::Duration;

use anyhow::Context as _;
use firmware_imu::icm42688;
use firmware_imu::icm42688::ICM42688;
use firmware_imu::lsm6dsv320x;
use firmware_imu::lsm6dsv320x::LSM6DSV320X;

use serialport::SerialPort;

/// Aptly named for the barrage of stuff that comes out of this.
struct Firehose {
    port: Box<dyn SerialPort>,
}

impl Firehose {
    /// Create a new Lights instance, attaching to the provided serial port.
    pub fn new(port_name: &str) -> anyhow::Result<Firehose> {
        let port = serialport::new(port_name, 9600) // Baud rate is a dummy anyway.
            .timeout(Duration::from_millis(100))
            .open()
            .with_context(|| format!("Port '{}' not available ", &port_name))?;
        Ok(Firehose { port })
    }
    pub fn read_data<'a>(&mut self, data: &'a mut [u8]) -> anyhow::Result<&'a [u8]> {
        self.port
            .read(data)
            .map(|v| &data[0..v])
            .with_context(|| "")
    }
}

pub fn main() -> Result<(), Box<dyn std::error::Error>> {
    // The first argument is the serial port name. For example, on Linux it might be "/dev/ttyUSB0". On Windows it might be "COM3".
    let port_name = std::env::args()
        .nth(1)
        .expect("Please provide a serial port name as an argument");

    // Create a new Firehose instance.
    let mut firehose = Firehose::new(&port_name)?;

    let mut data = [0u8; 512];
    let z = firehose.read_data(&mut data)?;
    println!("z: {z:?}");

    Ok(())
}
