#![no_std]
#![no_main]

#[allow(unused_imports)]
use panic_halt;
use riscv_rt::entry;

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

    // read from the serial interface and immediately loop the received
    // content back
    loop {
        let byte = block!(serial0.read()).unwrap();
        block!(serial0.write(byte)).unwrap();
    }
}
