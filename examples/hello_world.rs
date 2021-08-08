#![no_std]
#![no_main]

#[allow(unused_imports)]
use panic_halt;
use riscv_rt::entry;

use core::fmt::Write;
use esp32c3_hal::{pac, prelude::*, RtcCntl, Serial, Timer};
use nb::block;

#[entry]
fn main() -> ! {
    let peripherals = pac::Peripherals::take().unwrap();

    let rtccntl = RtcCntl::new(peripherals.RTCCNTL);
    let mut timer0 = Timer::new(peripherals.TIMG0);
    let mut serial0 = Serial::new(peripherals.UART0).unwrap();

    rtccntl.set_super_wdt_enable(false);
    rtccntl.set_wdt_enable(false);
    timer0.disable();

    timer0.start(10_000_000u64);

    // write some text
    loop {
        writeln!(serial0, "Hello World").unwrap();
        block!(timer0.wait()).unwrap();
    }
}
