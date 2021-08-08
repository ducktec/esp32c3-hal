#![no_std]
#![no_main]

#[allow(unused_imports)]
use panic_halt;
use riscv_rt::entry;

extern crate esp32c3_hal;

use esp32c3_hal::{pac, prelude::*, RtcCntl, Timer};
use nb::block;

#[entry]
fn main() -> ! {
    let peripherals = pac::Peripherals::take().unwrap();

    let rtccntl = RtcCntl::new(peripherals.RTCCNTL);
    let mut timer0 = Timer::new(peripherals.TIMG0);
    let mut timer1 = Timer::new(peripherals.TIMG1);

    rtccntl.set_super_wdt_enable(false);
    rtccntl.set_wdt_enable(false);
    timer0.disable();
    timer1.disable();

    write_text(&peripherals.UART0);

    timer1.start(10_000_000u64);

    // do nothing here
    loop {
        write_text(&peripherals.UART0);
        block!(timer1.wait()).unwrap();
    }
}

fn write_text(uart: &pac::UART0) {
    // Disable UART RX interrupts
    uart.int_clr.write(|w| {
        w.rxfifo_full_int_clr()
            .set_bit()
            .rxfifo_ovf_int_clr()
            .set_bit()
            .rxfifo_tout_int_clr()
            .set_bit()
    });

    uart.int_ena.write(|w| {
        w.rxfifo_full_int_ena()
            .clear_bit()
            .rxfifo_ovf_int_ena()
            .clear_bit()
            .rxfifo_tout_int_ena()
            .clear_bit()
    });

    // Disable UART TX interrupts
    uart.int_clr.write(|w| {
        w.txfifo_empty_int_clr()
            .set_bit()
            .tx_brk_done_int_clr()
            .set_bit()
            .tx_brk_idle_done_int_clr()
            .set_bit()
            .tx_done_int_clr()
            .set_bit()
    });

    uart.int_ena.write(|w| {
        w.txfifo_empty_int_ena()
            .clear_bit()
            .tx_brk_done_int_ena()
            .clear_bit()
            .tx_brk_idle_done_int_ena()
            .clear_bit()
            .tx_done_int_ena()
            .clear_bit()
    });

    // Write (small) text to UART buffer
    for elem in "Hello World!\n".as_bytes() {
        uart.fifo
            .write(|w| unsafe { w.rxfifo_rd_byte().bits(*elem) });
    }
}
