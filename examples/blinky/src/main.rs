//! Blinky example
//!
//! This example toggles a LED that is connected to the GPIO2
//! port with an frequency of 0.5 Herz (1s on, 1s off).
//!
//! This example does NOT toggle the LED that is present on
//! the devKit boards. That LED is RGB (SK68XXMINI-HS) and
//! requires a pulse-encoded drive logic (e.g. using the
//! RMT peripheral).

#![no_std]
#![no_main]

use esp32c_rt::entry;
#[allow(unused_imports)]
use panic_halt;

use esp32c3_hal::{pac, prelude::*, RtcCntl, Timer, IO};
use nb::block;

#[entry]
fn main() -> ! {
    let peripherals = pac::Peripherals::take().unwrap();
    let rtccntl = RtcCntl::new(peripherals.RTCCNTL);
    let mut timer0 = Timer::new(peripherals.TIMG0);
    let io = IO::new(peripherals.GPIO, peripherals.IO_MUX);

    // Disable watchdogs (they otherwise reset the SoC)
    rtccntl.set_super_wdt_enable(false);
    rtccntl.set_wdt_enable(false);
    timer0.disable();

    // Get handle to GPIO output
    let mut led = io.pins.gpio2.into_push_pull_output();

    // Initialize the timer with an interval of 1 second
    // TODO: Switch to coherent units.
    timer0.start(10_000_000u64);

    // Blink to eternity
    // (the timer is repeatedly re-triggered by the previously
    // initialized target value, every time the `wait()` function is
    // invoked)
    loop {
        led.set_high().unwrap();
        block!(timer0.wait()).unwrap();
        led.set_low().unwrap();
        block!(timer0.wait()).unwrap();
    }
}
