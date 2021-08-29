//! I2C Functionality (I2C)
//!
//! Reference: ESP32-C3 TRM v0.3 Section XX (as TRM)
//!
//! The ESP32-C3 has a single TWAI Peripheral.
//!
//! TODO:
//! - Prototypical support
//! - Interrupt support

use crate::pac::{i2c::RegisterBlock, I2C0};
use embedded_hal::blocking::i2c::*;

/// I2C-specific errors
#[derive(Debug)]
pub enum Error {}

/// I2C peripheral (I2C)
#[allow(dead_code)]
pub struct I2C<T> {
    instance: T,
}

impl<T: Instance> I2C<T> {
    pub fn new(instance: T) -> Result<Self, Error> {
        let i2c = I2C { instance };
        Ok(i2c)
    }
}

impl<T: Instance> Read for I2C<T> {
    type Error = Error;

    fn read(&mut self, _address: u8, _buffer: &mut [u8]) -> Result<(), Self::Error> {
        todo!()
    }
}

impl<T: Instance> Write for I2C<T> {
    type Error = Error;

    fn write(&mut self, _addr: u8, _bytes: &[u8]) -> Result<(), Self::Error> {
        todo!()
    }
}

impl<T: Instance> WriteIter for I2C<T> {
    type Error = Error;

    fn write<B>(&mut self, _addr: u8, _bytes: B) -> Result<(), Self::Error>
    where
        B: IntoIterator<Item = u8>,
    {
        todo!()
    }
}

impl<T: Instance> WriteIterRead for I2C<T> {
    type Error = Error;

    fn write_iter_read<B>(
        &mut self,
        _address: u8,
        _bytes: B,
        _buffer: &mut [u8],
    ) -> Result<(), Self::Error>
    where
        B: IntoIterator<Item = u8>,
    {
        todo!()
    }
}

impl<T: Instance> WriteRead for I2C<T> {
    type Error = Error;

    fn write_read(
        &mut self,
        _address: u8,
        _bytes: &[u8],
        _buffer: &mut [u8],
    ) -> Result<(), Self::Error> {
        todo!()
    }
}

pub trait Instance {
    /// Return registerblock of uart instance as if it were I2C0
    fn as_i2c0(&self) -> &RegisterBlock;
}

/// Specific instance implementation for the I2C0 peripheral
impl Instance for I2C0 {
    #[inline(always)]
    fn as_i2c0(&self) -> &RegisterBlock {
        self
    }
}
