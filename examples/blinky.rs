#![no_std]
#![no_main]

#[allow(unused_imports)]
use panic_halt;
use riscv_rt::entry;

use esp32c3_hal::{pac, prelude::*, RtcCntl, Timer, IO};
use nb::block;

#[entry]
fn main() -> ! {
    let peripherals = pac::Peripherals::take().unwrap();

    let rtccntl = RtcCntl::new(peripherals.RTCCNTL);
    let mut timer0 = Timer::new(peripherals.TIMG0);
    let io = IO::new(peripherals.GPIO, peripherals.IO_MUX);

    let mut led = io.pins.gpio2.into_push_pull_output();

    rtccntl.set_super_wdt_enable(false);
    rtccntl.set_wdt_enable(false);
    timer0.disable();

    timer0.start(10_000_000u64);

    // write some text
    loop {
        led.set_high().unwrap();
        block!(timer0.wait()).unwrap();
        led.set_low().unwrap();
        block!(timer0.wait()).unwrap();
    }
}
