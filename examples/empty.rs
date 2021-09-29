//! Empty example
//!
//! This example initializes the ESP32C3 and then enters
//! an endless loop.
//!
//! The goal of this example is to demonstrate that the
//! basic initialization works and that the watchdogs
//! can successfully disabled and the SoC does not get
//! stuck in a boot loop.

#![no_std]
#![no_main]

use esp32c_rt::entry;
#[allow(unused_imports)]
use panic_halt;

use esp32c3_hal::{pac, prelude::*, RtcCntl, Timer};

#[entry]
fn main() -> ! {
    let peripherals = pac::Peripherals::take().unwrap();
    let rtccntl = RtcCntl::new(peripherals.RTCCNTL);
    let mut timer0 = Timer::new(peripherals.TIMG0);

    // disable watchdogs (they otherwise reset the SoC)
    rtccntl.set_super_wdt_enable(false);
    rtccntl.set_wdt_enable(false);
    timer0.disable();

    // do nothing here
    loop {}
}
