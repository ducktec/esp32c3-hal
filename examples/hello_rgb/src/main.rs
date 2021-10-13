//! RGB LED Demo
//!
//! This example drives an SK68XX RGB LED that is connected to the GPIO8 pin. This RGB LED
//! is existing on that pin for the ESP32-C3-DevKitM-1 and ESP32-C3-DevKitC-02 dev boards.
//!
//! The demo will switch between low-brightness red, green and blue every second.

#![no_std]
#![no_main]

use esp32c_rt::entry;
#[allow(unused_imports)]
use panic_halt;

use esp32c3_hal::pulse_control::{ClockSource, OutputChannel};
use esp32c3_hal::{pac, prelude::*, PulseControl, RtcCntl, Timer, IO};
use nb::block;
use rgb::*;
use sk68xx::set_color;

#[entry]
fn main() -> ! {
    let peripherals = pac::Peripherals::take().unwrap();

    let rtccntl = RtcCntl::new(peripherals.RTCCNTL);
    let mut timer0 = Timer::new(peripherals.TIMG0);
    let mut io = IO::new(peripherals.GPIO, peripherals.IO_MUX);

    // Disable watchdogs
    rtccntl.set_super_wdt_enable(false);
    rtccntl.set_wdt_enable(false);
    timer0.disable();

    // Start timer with 1 second interval
    timer0.start(10_000_000u64);

    // Enable RMT peripheral's clock and reset
    peripherals
        .SYSTEM
        .perip_clk_en0
        .modify(|_, w| w.rmt_clk_en().set_bit());

    peripherals
        .SYSTEM
        .perip_rst_en0
        .modify(|_, w| w.rmt_rst().clear_bit());

    // Configure RMT peripheral globally
    let pulse = PulseControl::new(peripherals.RMT, ClockSource::APB, 0, 0, 0).unwrap();

    // Get reference to used channel (we use channel 0 for our purposes)
    let mut rgb_channel = pulse.channel0;

    rgb_channel
        .set_idle_output_level(false)
        .set_carrier_modulation(false)
        .set_channel_divider(1)
        .set_idle_output(true);

    rgb_channel.configure_pin(&mut io.pins.gpio8);

    // Configure colors to display
    let red = RGB {
        r: 5_u8,
        g: 0_u8,
        b: 0_u8,
    };

    let green = RGB {
        r: 0_u8,
        g: 5_u8,
        b: 0_u8,
    };

    let blue = RGB {
        r: 0_u8,
        g: 0_u8,
        b: 5_u8,
    };

    loop {
        // Iterate over colors and set for a second
        for color in [red, green, blue] {
            set_color(color, &rgb_channel).unwrap();
            block!(timer0.wait()).unwrap();
        }
    }
}

mod sk68xx {
    use core::slice::IterMut;
    use esp32c3_hal::pulse_control::{OutputChannel, PulseCode, RepeatMode, TransmissionError};
    use rgb::RGB8;

    pub type SK68XXpulseSequence = [PulseCode; 24];

    const SK68XX_T0H_CYCLES: u16 = 4;
    const SK68XX_T0L_CYCLES: u16 = 20;
    const SK68XX_T1H_CYCLES: u16 = 12;
    const SK68XX_T1L_CYCLES: u16 = 12;

    #[derive(Debug)]
    pub enum Error {
        SequenceExceeded,
    }

    pub fn set_color<Channel: OutputChannel>(
        value: RGB8,
        rgb_channel: &Channel,
    ) -> Result<(), TransmissionError> {
        rgb_channel.send_pulse_sequence(
            RepeatMode::SingleShot,
            &convert_rgb_to_pulse(value).unwrap(),
        )?;

        Ok(())
    }

    pub fn convert_rgb_to_pulse(value: RGB8) -> Result<SK68XXpulseSequence, Error> {
        let mut seq: SK68XXpulseSequence = [PulseCode {
            level1: true,
            length1: 1,
            level2: false,
            length2: 9,
        }; 24];
        let mut mut_iter = seq.iter_mut();

        convert_rgb_channel_to_pulses(value.g, &mut mut_iter)?;
        convert_rgb_channel_to_pulses(value.r, &mut mut_iter)?;
        convert_rgb_channel_to_pulses(value.b, &mut mut_iter)?;

        Ok(seq)
    }

    fn convert_rgb_channel_to_pulses(
        channel_value: u8,
        mut_iter: &mut IterMut<PulseCode>,
    ) -> Result<(), Error> {
        for position in [128, 64, 32, 16, 8, 4, 2, 1] {
            *mut_iter.next().ok_or(Error::SequenceExceeded)? = match channel_value & position {
                0 => PulseCode {
                    level1: true,
                    length1: SK68XX_T0H_CYCLES,
                    level2: false,
                    length2: SK68XX_T0L_CYCLES,
                },
                _ => PulseCode {
                    level1: true,
                    length1: SK68XX_T1H_CYCLES,
                    level2: false,
                    length2: SK68XX_T1L_CYCLES,
                },
            }
        }

        Ok(())
    }
}
