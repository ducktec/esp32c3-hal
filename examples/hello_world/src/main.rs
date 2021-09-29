//! "Hello World" example
//!
//! This example writes "Hello World" to the serial0 (UART0)
//! output every second.
//!
//! Without any additional GPIO configuration (as with this example)
//! the serial0 peripheral is routed to the GPIO20/21 (U0TXD/U0RXD)
//! pins after startup.

#![no_std]
#![no_main]

use esp32c_rt::entry;
#[allow(unused_imports)]
use panic_halt;

use core::fmt::Write;
use esp32c3_hal::{pac, prelude::*, RtcCntl, Serial, Timer};
use nb::block;

#[entry]
fn main() -> ! {
    let peripherals = pac::Peripherals::take().unwrap();
    let rtccntl = RtcCntl::new(peripherals.RTCCNTL);
    let mut timer0 = Timer::new(peripherals.TIMG0);
    let mut serial0 = Serial::new(peripherals.UART0).unwrap();

    // Disable watchdogs (they otherwise reset the SoC)
    rtccntl.set_super_wdt_enable(false);
    rtccntl.set_wdt_enable(false);
    timer0.disable();

    // Initialize the timer with an interval of 1 second
    // TODO: Switch to coherent units.
    timer0.start(10_000_000u64);

    // Write "Hello World" forever
    loop {
        writeln!(serial0, "Hello World").unwrap();
        block!(timer0.wait()).unwrap();
    }
}
