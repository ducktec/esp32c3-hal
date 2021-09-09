//! LED PWM Controller Functionality (LEDC)
//!
//! Reference: ESP32-C3 TRM v0.3 Section 23 (as TRM)
//!
//! The ESP32-C3 has a single LED PWM Controller peripheral that includes
//! 4 independent timers and 6 independent channels (generators) that can be
//! mapped to arbitrary GPIO pins via the GPIO matrix mux.
//!
//! While the peripheral is called labeled LED PWM Controller, the
//! peripheral can also be used to drive any other PWM signal.

use crate::gpio::{OutputPin, OutputSignal};
use crate::pac::LEDC;

/// LEDC-specific transmission errors
#[derive(Debug)]
pub enum Error {}

/// LED peripheral (LEDC)
pub struct LED {
    reg: LEDC,
}

impl LED {
    /// Create a new LED controller instance
    pub fn new<P: OutputPin>(instance: LEDC, mut pin: P) -> Result<Self, Error> {
        let led = LED { reg: instance };

        // Setup Pin
        // pin.set_to_push_pull_output()
        //     .connect_peripheral_to_output(OutputSignal::LEDC_LS_SIG0);

        Ok(led)
    }

    /// Return the raw interface to the underlying LEDC peripheral
    pub fn free(self) -> LEDC {
        self.reg
    }
}
