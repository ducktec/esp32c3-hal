//! UART Functionality (UART)
//!
//! Reference: ESP32-C3 TRM v0.3 Section 20 (as TRM)
//!
//! The ESP32-C3 has two identical UART groups UART0 and UART1.
//!
//! TODO:
//! -

use crate::pac::{uart::RegisterBlock, UART0, UART1};
use embedded_hal::serial::*;

/// UART-specific write-related errors
pub enum WriteError {}

/// UART-specific read-related errors
pub enum ReadError {}

/// UART peripheral (UART)
pub struct Serial<T> {
    uart: T,
}

impl<T> Serial<T>
where
    T: Instance,
{
    pub fn new(uart: T) -> Self {
        let serial = Serial { uart };
        serial
    }
}

pub trait Instance {
    /// Return registerblock of uart instance as if it were UART0
    fn as_uart0(&self) -> &RegisterBlock;
}

/// Write half of a serial interface
impl<T> Write<u8> for Serial<T>
where
    T: Instance,
{
    type Error = crate::serial::WriteError;

    /// Writes a single word to the serial interface
    fn write(&mut self, _word: u8) -> nb::Result<(), Self::Error> {
        unimplemented!()
    }

    /// Ensures that none of the previously written words are still buffered
    fn flush(&mut self) -> nb::Result<(), Self::Error> {
        unimplemented!()
    }
}

/// Write half of a serial interface
impl<T> Read<u8> for Serial<T>
where
    T: Instance,
{
    type Error = crate::serial::ReadError;

    /// Reads a single word from the serial interface
    fn read(&mut self) -> nb::Result<u8, Self::Error> {
        unimplemented!()
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
