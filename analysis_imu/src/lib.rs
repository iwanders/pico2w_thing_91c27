use std::time::Duration;

use anyhow::Context as _;
use firmware_imu::icm42688;
use firmware_imu::icm42688::ICM42688;
use firmware_imu::lsm6dsv320x;
use firmware_imu::lsm6dsv320x::LSM6DSV320X;
use std::sync::mpsc::{Receiver, Sender};

use serialport::SerialPort;

// Manual copy, boo.
#[derive(Copy, Clone, Debug, zerocopy::IntoBytes)]
#[repr(u8)]
enum DataType {
    Nop,
    Icm,
    Lsm,
}

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
    pub fn read_exact<'a>(&mut self, data: &'a mut [u8]) -> anyhow::Result<()> {
        self.port
            .read_exact(data)
            .context("timed out reading bytes")
    }
}

struct LsmReceiver(Receiver<u8>);
struct IcmReceiver(Receiver<u8>);

struct ImuSplitter {
    firehose: Firehose,
    lsm_sender: Sender<u8>,
    icm_sender: Sender<u8>,
}
impl ImuSplitter {
    pub fn new(firehose: Firehose) -> (Self, LsmReceiver, IcmReceiver) {
        let (lsm_sender, lsm_rec) = std::sync::mpsc::channel::<u8>();
        let (icm_sender, icm_rec) = std::sync::mpsc::channel::<u8>();
        (
            Self {
                firehose,
                lsm_sender,
                icm_sender,
            },
            LsmReceiver(lsm_rec),
            IcmReceiver(icm_rec),
        )
    }
    pub fn pump(&mut self) {
        loop {
            let mut data = [0u8; 64];
            self.firehose.read_exact(&mut data).unwrap();
            if &data[1..4] != &[0, 0, 0] {
                println!("alignment problem 😱")
            }
            if data[0] == DataType::Icm as u8 {
                for b in data[4..].iter() {
                    self.icm_sender.send(*b);
                }
            } else if data[0] == DataType::Lsm as u8 {
                for b in data[4..].iter() {
                    self.lsm_sender.send(*b);
                }
            } else {
                println!("payload type problem!");
            }
        }
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
    firehose.read_exact(&mut data)?;
    println!("z: {data:?}");

    let (mut splitter, lsm_rec, icm_rec) = ImuSplitter::new(firehose);

    // Create a thread to pump splitter.
    let handle = std::thread::spawn(move || {
        splitter.pump();
    });

    // Very nice, we have two data pipes.
    if false {
        loop {
            for k in icm_rec.0.try_iter() {
                // for k in lsm_rec.0.try_iter() {
                print!("{k} ");
            }
        }
    }

    // Next, we need to actually do the hard work and re-assemble the fifo data into something that is useful.
    // We want this to be able to run on the imu as well, so lets do something like a sliding window approach
    // where we get bytes into a buffer, analyse them and then determine how many bytes were consumed by the message
    // which works for the ICM... but not that well for the LSM which has a way more convenient fifo...

    // For testing, lets just create two buffers of 512 each.

    let mut icm_data = [0u8; 512];
    icm_data.fill_with(|| icm_rec.0.recv().unwrap());
    let mut lsm_data = [0u8; 512];
    lsm_data.fill_with(|| lsm_rec.0.recv().unwrap());

    println!("lsm_data: {lsm_data:?}");

    println!("icm_data: {icm_data:?}");
    // 255, 182, 31, 92, 56, 24, 0, 62, 0, 4, 0, 3, 249, 171, 68, 104,
    // 255, 198, 31, 92, 56, 30, 0, 63, 0, 4, 0, 3, 249, 171, 186, 104,
    // 255, 190, 31, 98, 56, 44, 0, 63, 0, 4, 0, 3, 249, 172, 47, 104,
    // Currently 16 bytes long? Why isn't this 20 with the 20 bit extension data?

    Ok(())
}
