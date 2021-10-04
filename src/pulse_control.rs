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
//!
//! Non-FIFO mode and non-blocking mode (i.e., with interrupts) is
//! currently not supported!

#![deny(missing_docs)]

use crate::pac::RMT;

/// RMT-specific errors
#[derive(Debug)]
pub enum Error {
    /// The global configuration for the RMT peripheral is invalid
    /// (e.g. the fractional parameters are outOfBound)
    InvalidGlobalConfig,
}

/// RMT peripheral (RMT)
pub struct PulseControl {
    reg: RMT,
    config: Configuration,
}

/// Specify the clock source for the RMT peripheral
#[derive(Debug, Copy, Clone)]
pub enum ClockSource {
    /// Application-level clock
    APB = 1,
    /// 20 MHz internal oscillator
    RTC20M = 2,
    /// External clock source
    XTAL = 3,
}

struct Configuration {
    clk_source: ClockSource,
    divider_absolute: u8,
    divider_frac_a: u8,
    divider_frac_b: u8,
}

impl PulseControl {
    /// Create a new pulse controller instance
    pub fn new(
        instance: RMT,
        clk_source: ClockSource,
        div_abs: u8,
        div_frac_a: u8,
        div_frac_b: u8,
    ) -> Result<Self, Error> {
        let pc = PulseControl {
            reg: instance,
            config: Configuration {
                clk_source: clk_source,
                divider_absolute: div_abs,
                divider_frac_a: div_frac_a,
                divider_frac_b: div_frac_b,
            },
        };

        pc.config_global()?;

        // Setup Pin
        // pin.set_to_push_pull_output()
        //     .connect_peripheral_to_output(OutputSignal::LEDC_LS_SIG0);

        Ok(pc)
    }

    /// Return the raw interface to the underlying RMT peripheral
    pub fn free(self) -> RMT {
        self.reg
    }

    /// Assign the global (peripheral-wide) configuration. This
    /// is mostly the divider setup and the clock source selection
    ///
    /// Please note, per TRM v0.3, the dividing factor for the source
    /// clock is calculated as follows:
    ///
    /// divider = absolute_part + 1 + (fractional_part_a / fractional_part_b)
    ///
    fn config_global(&self) -> Result<(), Error> {
        // Before assigning, confirm that the fractional parameters for
        // the divider are within bounds
        if self.config.divider_frac_a > 64 || self.config.divider_frac_b > 64 {
            return Err(Error::InvalidGlobalConfig);
        }

        // TODO: Confirm that the selected clock source is enabled in the
        // system / rtc_cntl peripheral? Particularly relevant for clock sources
        // other than APB_CLK

        // Configure peripheral
        self.reg.sys_conf.modify(|_, w| unsafe {
            // Enable clock
            w.clk_en()
                .set_bit()
                // Force Clock on
                .mem_clk_force_on()
                .set_bit()
                // Enable Source clock
                .sclk_active()
                .set_bit()
                // Disable forced power down of the peripheral (just to be sure)
                .mem_force_pd()
                .clear_bit()
                // Enable FIFO mode
                .apb_fifo_mask()
                .clear_bit()
                // Select clock source
                .sclk_sel()
                .bits(self.config.clk_source as u8)
                // Set absolute part of divider
                .sclk_div_num()
                .bits(self.config.divider_absolute)
                // Set fractional parts of divider to 0
                .sclk_div_a()
                .bits(self.config.divider_frac_a)
                .sclk_div_b()
                .bits(self.config.divider_frac_b)
        });

        // Disable all interrupts
        self.reg.int_ena.write(|w| unsafe { w.bits(0) });

        // Clear all interrupts
        self.reg.int_clr.write(|w| unsafe { w.bits(0) });

        Ok(())
    }
}
