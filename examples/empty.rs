#![no_std]
#![no_main]

#[allow(unused_imports)]
use panic_halt;
use riscv_rt::entry;

extern crate esp32c3_hal;

use esp32c3_hal::{pac, prelude::*, RtcCntl, Timer};

#[entry]
fn main() -> ! {
    let peripherals = pac::Peripherals::take().unwrap();

    let rtccntl = RtcCntl::new(peripherals.RTCCNTL);
    let mut timer = Timer::new(peripherals.TIMG0);

    rtccntl.set_super_wdt_enable(false);
    rtccntl.set_wdt_enable(false);
    timer.disable();

    // do nothing here
    loop {}
}
