//! Remote Control Peripheral (RMT)
//!
//! Reference: ESP32-C3 TRM v0.3 Section 24 (as TRM)
//!
//! The ESP32-C3 includes a remote control peripheral (RMT) that
//! is designed to handle infrared remote control signals. For that
//! purpose, it can convert bitstreams of data (from the RAM) into
//! pulse codes (and even modulate those codes into a carrier wave).
//!
//! It can also convert received pulse codes (again, with carrier
//! wave support) into data bits.
//!
//! A secondary use case for this peripheral is to drive RGB(W) LEDs
//! that bear an internal IC and use a pulse code protocol.
//!
//! The RMT peripheral has 4 channels, 2 TX and 2 RX.

use crate::pac::RMT;

/// RMT-specific errors
#[derive(Debug)]
pub enum Error {}

/// RMT peripheral (RMT)
pub struct PulseControl {
    reg: RMT,
}

impl PulseControl {
    /// Create a new pulse controller instance
    pub fn new(instance: RMT) -> Result<Self, Error> {
        let pulse = PulseControl { reg: instance };

        // Setup Pin
        // pin.set_to_push_pull_output()
        //     .connect_peripheral_to_output(OutputSignal::LEDC_LS_SIG0);

        Ok(pulse)
    }

    /// Return the raw interface to the underlying LEDC peripheral
    pub fn free(self) -> RMT {
        self.reg
    }
}
