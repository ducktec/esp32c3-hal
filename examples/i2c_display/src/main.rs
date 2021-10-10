//! I2C display example
//!
//! This example displays text on an external OLED display with a SSD1306 driver
//! that is connected via the I2C peripheral.
//!
//! The following wiring is assumed:
//! - SDA => GPIO2
//! - SCL => GPIO3
//!
//! The example will toggle between displaying two different text messages
//! "esp32c3-hal - v0.1.0" and "Hello World" every five seconds.

#![no_std]
#![no_main]

use esp32c_rt::entry;
#[allow(unused_imports)]
use panic_halt;

use esp32c3_hal::{i2c, pac, prelude::*, RtcCntl, Timer, I2C, IO};
use nb::block;

use embedded_graphics::{
    mono_font::{ascii::FONT_6X10, ascii::FONT_9X18_BOLD, MonoTextStyleBuilder},
    pixelcolor::BinaryColor,
    prelude::*,
    text::{Alignment, Text},
};
use ssd1306::{prelude::*, I2CDisplayInterface, Ssd1306};

#[entry]
fn main() -> ! {
    let peripherals = pac::Peripherals::take().unwrap();
    let rtccntl = RtcCntl::new(peripherals.RTCCNTL);
    let mut timer0 = Timer::new(peripherals.TIMG0);
    let io = IO::new(peripherals.GPIO, peripherals.IO_MUX);

    // Disable watchdogs to prevent reset of the entire chip
    rtccntl.set_super_wdt_enable(false);
    rtccntl.set_wdt_enable(false);
    timer0.disable();

    // Enable the I2C peripheral clock
    peripherals
        .SYSTEM
        .perip_clk_en0
        .modify(|_, w| w.i2c_ext0_clk_en().set_bit());

    // Take the I2C peripheral out of any pre-existing reset state
    // (shouldn't be the case after a fresh startup, but better be safe)
    peripherals
        .SYSTEM
        .perip_rst_en0
        .modify(|_, w| w.i2c_ext0_rst().clear_bit());

    // Create a new peripheral object with the described wiring
    // and standard I2C clock speed
    let i2c = I2C::new(
        peripherals.I2C0,
        i2c::Pins {
            sda: io.pins.gpio2,
            scl: io.pins.gpio3,
        },
        100_000,
    )
    .unwrap();

    // Start timer (5 second interval)
    timer0.start(50_000_000u64);

    // Initialize display
    let interface = I2CDisplayInterface::new(i2c);
    let mut display = Ssd1306::new(interface, DisplaySize128x64, DisplayRotation::Rotate0)
        .into_buffered_graphics_mode();
    display.init().unwrap();

    // Specify different text styles
    let text_style = MonoTextStyleBuilder::new()
        .font(&FONT_6X10)
        .text_color(BinaryColor::On)
        .build();
    let text_style_big = MonoTextStyleBuilder::new()
        .font(&FONT_9X18_BOLD)
        .text_color(BinaryColor::On)
        .build();

    loop {
        // Fill display bufffer with a centered text with two lines (and two text styles)
        Text::with_alignment(
            "esp32c3-hal",
            display.bounding_box().center() + Point::new(0, 0),
            text_style_big,
            Alignment::Center,
        )
        .draw(&mut display)
        .unwrap();

        Text::with_alignment(
            "v0.1.0",
            display.bounding_box().center() + Point::new(0, 14),
            text_style,
            Alignment::Center,
        )
        .draw(&mut display)
        .unwrap();

        // Write buffer to display
        display.flush().unwrap();
        // Clear display buffer
        display.clear();

        // Wait 5 seconds
        block!(timer0.wait()).unwrap();

        // Write single-line centered text "Hello World" to buffer
        Text::with_alignment(
            "Hello World!",
            display.bounding_box().center() + Point::new(0, 0),
            text_style_big,
            Alignment::Center,
        )
        .draw(&mut display)
        .unwrap();

        // Write buffer to display
        display.flush().unwrap();
        // Clear display buffer
        display.clear();

        // Wait 5 seconds
        block!(timer0.wait()).unwrap();
    }
}
