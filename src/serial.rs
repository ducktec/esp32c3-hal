//! UART Functionality (UART)
//!
//! Reference: ESP32-C3 TRM v0.3 Section 20 (as TRM)
//!
//! The ESP32-C3 has two identical UART groups UART0 and UART1.
//!
//! TODO:
//! - Interrupt support

use crate::pac::{uart::RegisterBlock, UART0, UART1};
use embedded_hal::serial::*;

const UART_FIFO_SIZE: u16 = 128;

/// UART-specific errors
#[derive(Debug)]
pub enum Error {}

/// UART peripheral (UART)
pub struct Serial<T> {
    uart: T,
}

impl<T: Instance> Serial<T> {
    pub fn new(uart: T) -> Result<Self, Error> {
        let mut serial = Serial { uart };
        serial.uart.disable_rx_interrupts();
        serial.uart.disable_tx_interrupts();
        Ok(serial)
    }
}

pub trait Instance {
    /// Return registerblock of uart instance as if it were UART0
    fn as_uart0(&self) -> &RegisterBlock;

    /// Clear and disable all tx-related interrupts
    fn disable_tx_interrupts(&mut self) {
        // Disable UART TX interrupts
        self.as_uart0().int_clr.write(|w| {
            w.txfifo_empty_int_clr()
                .set_bit()
                .tx_brk_done_int_clr()
                .set_bit()
                .tx_brk_idle_done_int_clr()
                .set_bit()
                .tx_done_int_clr()
                .set_bit()
        });

        self.as_uart0().int_ena.write(|w| {
            w.txfifo_empty_int_ena()
                .clear_bit()
                .tx_brk_done_int_ena()
                .clear_bit()
                .tx_brk_idle_done_int_ena()
                .clear_bit()
                .tx_done_int_ena()
                .clear_bit()
        });
    }

    /// Clear and disable all rx-related interrupts
    fn disable_rx_interrupts(&mut self) {
        // Disable UART RX interrupts
        self.as_uart0().int_clr.write(|w| {
            w.rxfifo_full_int_clr()
                .set_bit()
                .rxfifo_ovf_int_clr()
                .set_bit()
                .rxfifo_tout_int_clr()
                .set_bit()
        });

        self.as_uart0().int_ena.write(|w| {
            w.rxfifo_full_int_ena()
                .clear_bit()
                .rxfifo_ovf_int_ena()
                .clear_bit()
                .rxfifo_tout_int_ena()
                .clear_bit()
        });
    }

    /// Get the number of occupied entries in the tx fifo buffer
    fn get_tx_fifo_count(&mut self) -> u16 {
        self.as_uart0().status.read().txfifo_cnt().bits()
    }

    /// Get the number of occupied entries in the rx fifo buffer
    fn get_rx_fifo_count(&mut self) -> u16 {
        self.as_uart0().status.read().rxfifo_cnt().bits()
    }

    /// Check if the UART TX statemachine is is idle
    fn is_tx_idle(&mut self) -> bool {
        self.as_uart0().fsm_status.read().st_utx_out().bits() == 0x0u8
    }

    /// Check if the UART RX statemachine is is idle
    fn is_rx_idle(&mut self) -> bool {
        self.as_uart0().fsm_status.read().st_urx_out().bits() == 0x0u8
    }
}

/// Write half of a serial interface
impl<T: Instance> Write<u8> for Serial<T> {
    type Error = Error;

    /// Writes a single word to the serial interface
    fn write(&mut self, word: u8) -> nb::Result<(), Self::Error> {
        if self.uart.get_tx_fifo_count() >= UART_FIFO_SIZE {
            Err(nb::Error::WouldBlock)
        } else {
            self.uart
                .as_uart0()
                .fifo
                .write(|w| unsafe { w.rxfifo_rd_byte().bits(word) });
            Ok(())
        }
    }

    /// Ensures that none of the previously written words are still buffered
    fn flush(&mut self) -> nb::Result<(), Self::Error> {
        if self.uart.is_tx_idle() {
            Ok(())
        } else {
            Err(nb::Error::WouldBlock)
        }
    }
}

/// Write half of a serial interface
impl<T: Instance> Read<u8> for Serial<T> {
    type Error = Error;

    /// Reads a single word from the serial interface
    fn read(&mut self) -> nb::Result<u8, Self::Error> {
        if self.uart.get_rx_fifo_count() > 0 {
            Ok(self.uart.as_uart0().fifo.read().rxfifo_rd_byte().bits())
        } else {
            Err(nb::Error::WouldBlock)
        }
    }
}

impl<T: Instance> core::fmt::Write for Serial<T> {
    fn write_str(&mut self, s: &str) -> core::fmt::Result {
        s.as_bytes()
            .iter()
            .try_for_each(|c| nb::block!(self.write(*c)))
            .map_err(|_| core::fmt::Error)
    }
}

/// Specific instance implementation for the UART0 peripheral
impl Instance for UART0 {
    #[inline(always)]
    fn as_uart0(&self) -> &RegisterBlock {
        self
    }
}

/// Specific instance implementation for the UART1 peripheral
impl Instance for UART1 {
    #[inline(always)]
    fn as_uart0(&self) -> &RegisterBlock {
        self
    }
}
