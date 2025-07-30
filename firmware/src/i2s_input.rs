use embassy_rp::Peri;
/// Pio backed I2s input (hacked up from the output example)
///
/// Specifically for; ICS-43434
/// The slave serial data port’s format is I2S, 24-bit, twos complement. There must be 64 SCK cycles in each WS stereo frame. The LR
/// control pin determines whether the ICS-43434 outputs data in the left or right channel. When set to the left channel, the data will be
/// output following WS’s falling edge and when set to output on the right channel, data will be output following WS’s rising edge.
///
/// Data from the DMA buffer needs to be bit shifted one to the left, after which the top 24 bits are good data.
/// It always samples both left and right, even if only one channel is selected.
///
/// The whole clock calculation seems right, but it should be possible to sample at a lowe rrate, that's untested.
///
/// Beware the order of the pins in new::new!
use embassy_rp::dma::{AnyChannel, Channel, Transfer};
use embassy_rp::pio::{
    Common, Config, Direction, FifoJoin, Instance, LoadedProgram, PioPin, ShiftConfig,
    ShiftDirection, StateMachine,
};
use fixed::traits::ToFixed;
use pio;

/// This struct represents an i2s output driver program
pub struct PioI2sInProgram<'a, PIO: Instance> {
    prg: LoadedProgram<'a, PIO>,
}

impl<'a, PIO: Instance> PioI2sInProgram<'a, PIO> {
    // This program now matches the waveforms in DS-000064-ICS-43434-v1.2.pdf, p13 for left and right, but the data it
    // produces is still wrong.
    pub fn new(common: &mut Common<'a, PIO>) -> Self {
        let prg = pio::pio_asm!(
            ".side_set 2",
            "    set x 30          side 0b00", // side, switched to 0xBW, bit, word
            "left_data:",
            "    in pins 1          side 0b10",
            "    jmp x-- left_data  side 0b00",
            // Read last entry to insert clock.
            "    in pins, 1         side 0b10",
            // Switch to right channel, r
            "    set x 30           side 0b01",
            "right_data:",
            "    in pins 1          side 0b11",
            "    jmp x-- right_data side 0b01",
            "    in pins, 1         side 0b11",
        );

        let prg = common.load_program(&prg.program);

        Self { prg }
    }
}

/// Pio backed I2s output driver
pub struct PioI2sIn<'a, P: Instance, const S: usize> {
    dma: Peri<'a, AnyChannel>,
    sm: StateMachine<'a, P, S>,
}

impl<'a, P: Instance, const S: usize> PioI2sIn<'a, P, S> {
    /// Configure a state machine to output I2s
    pub fn new(
        common: &mut Common<'a, P>,
        mut sm: StateMachine<'a, P, S>,
        dma: Peri<'a, impl Channel>,
        data_pin: Peri<'a, impl PioPin>,
        bit_clock_pin: Peri<'a, impl PioPin>,
        lr_clock_pin: Peri<'a, impl PioPin>,
        sample_rate: u32,
        bit_depth: u32,
        channels: u32,
        program: &PioI2sInProgram<'a, P>,
    ) -> Self {
        //into_ref!(dma);

        let data_pin = common.make_pio_pin(data_pin);
        let bit_clock_pin = common.make_pio_pin(bit_clock_pin);
        let left_right_clock_pin = common.make_pio_pin(lr_clock_pin);

        let cfg = {
            let mut cfg = Config::default();
            // NOTE: ORDER OF SIDE SET PINS MUST MATCH _SOMETHING_ else it hangs.
            cfg.use_program(&program.prg, &[&left_right_clock_pin, &bit_clock_pin]);
            cfg.set_in_pins(&[&data_pin]); //  not set and output pins since they're not connected to the Tx buffer?
            let clock_frequency = sample_rate * bit_depth * channels;
            cfg.clock_divider =
                (embassy_rp::clocks::clk_sys_freq() as f64 / clock_frequency as f64 / 2.)
                    .to_fixed();
            cfg.shift_in = ShiftConfig {
                threshold: 32, // ICS43434 always requires 32 clock cycles per frame, even though only 24 bits are used.
                direction: ShiftDirection::Left,
                auto_fill: true,
            };
            // join fifos to have twice the time to start the next dma transfer
            cfg.fifo_join = FifoJoin::RxOnly;
            cfg
        };
        sm.set_config(&cfg);
        sm.set_pin_dirs(Direction::Out, &[&bit_clock_pin, &left_right_clock_pin]);
        sm.set_pin_dirs(Direction::In, &[&data_pin]);

        sm.set_enable(true);

        Self {
            dma: dma.into(),
            sm,
        }
    }

    /// Return an in-progress dma transfer future. Awaiting it will guarantee a complete transfer.
    pub fn read<'b>(&'b mut self, buff: &'b mut [u32]) -> Transfer<'b, AnyChannel> {
        self.sm.rx().dma_pull(self.dma.reborrow(), buff, false)
    }
}
