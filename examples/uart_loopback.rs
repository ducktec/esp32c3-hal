//! Uart loopback example
//!
//! This example reads an arbitrary number of bytes from the serial0
//! (UART0) peripheral and writes them back to the same peripheral,
//! thus creating a loopback.
//!
//! Without any additional GPIO configuration (as with this example)
//! the serial0 peripheral is routed to the GPIO20/21 (U0TXD/U0RXD)
//! pins after startup.

#![no_std]
#![no_main]

use esp32c_rt::entry;
#[allow(unused_imports)]
use panic_halt;

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

    // read from the serial interface and immediately loop the received
    // content back
    loop {
        let byte = block!(serial0.read()).unwrap();
        block!(serial0.write(byte)).unwrap();
    }
}
