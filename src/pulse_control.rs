//! Remote Control Peripheral (RMT)
//!
//! Reference: ESP32-C3 TRM v0.3 Section 24 (as TRM)
//!
//! The ESP32-C3 includes a remote control peripheral (RMT) that
//! is designed to handle infrared remote control signals. For that
//! purpose, it can convert bitstreams of data (from the RAM) into
//! pulse codes and even modulate those codes into a carrier wave.
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
//!
//! Also, the input channels are currently not supported!

#![deny(missing_docs)]

use crate::gpio::{OutputPin, OutputSignal};
use crate::pac::RMT;

/// Errors that can occur when the peripheral is configured
#[derive(Debug)]
pub enum SetupError {
    /// The global configuration for the RMT peripheral is invalid
    /// (e.g. the fractional parameters are outOfBound)
    InvalidGlobalConfig,
    /// A pin was already assigned to the channel, at this point in
    /// time, only one assigned pin per channel is supported
    PinAlreadyAssigned,
}

/// Errors that can occur during a transmission attempt
#[derive(Debug)]
pub enum TransmissionError {
    /// Generic Transmission Error
    Failure,
    /// The maximum number of transmissions (`=(2^10)-1`) was exceeded
    RepetitionOverflow,
}

/// Specifies the mode with which pulses are sent out in
/// send channels (ch0 and ch1)
#[derive(Debug, Copy, Clone, PartialEq)]
pub enum RepeatMode {
    /// Send sequence once
    SingleShot,
    /// Send sequence N times (`N < (2^10)`)
    RepeatNtimes(u16),
    /// Repeat sequence until stopped by additional function call
    Forever,
}

/// RMT peripheral (RMT)
pub struct PulseControl {
    reg: RMT,
    config: Configuration,
    /// Output Channel 0
    pub channel0: Channel0,
    /// Output Channel 1
    pub channel1: Channel1,
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

/// Object representing the state of one pulse code per ESP32-C3 TRM
/// Allows for the assignment of two levels and their lenghts
#[derive(Clone, Copy, Debug)]
pub struct PulseCode {
    /// Logical output level in the first pulse code interval
    pub level1: bool,
    /// Length of the first pulse code interval
    pub length1: u16,
    /// Logical output level in the second pulse code interval
    pub level2: bool,
    /// Length of the second pulse code interval
    pub length2: u16,
}

/// Convert a pulse code structure into a u32 value that can be written
/// into the data registers
impl From<&PulseCode> for u32 {
    fn from(p: &PulseCode) -> u32 {
        // The Pulse Code format in the RAM appears to be
        // little-endian

        // The length1 value resides in bits [14:0]
        let mut entry: u32 = p.length1.into();

        // If level1 is high, set bit 15, otherwise clear it
        if p.level1 {
            entry |= 1 << 15;
        } else {
            entry &= !(1 << 15);
        }

        // If level2 is high, set bit 31, otherwise clear it
        if p.level2 {
            entry |= 1 << 31;
        } else {
            entry &= !(1 << 31);
        }

        // The length2 value resides in bits [30:16]
        entry |= (p.length2 as u32) << 16;

        entry
    }
}

impl PulseControl {
    /// Create a new pulse controller instance
    pub fn new(
        instance: RMT,
        clk_source: ClockSource,
        div_abs: u8,
        div_frac_a: u8,
        div_frac_b: u8,
    ) -> Result<Self, SetupError> {
        let pc = PulseControl {
            reg: instance,
            config: Configuration {
                clk_source: clk_source,
                divider_absolute: div_abs,
                divider_frac_a: div_frac_a,
                divider_frac_b: div_frac_b,
            },
            channel0: Channel0 {},
            channel1: Channel1 {},
        };

        pc.config_global()?;

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
    fn config_global(&self) -> Result<(), SetupError> {
        // Before assigning, confirm that the fractional parameters for
        // the divider are within bounds
        if self.config.divider_frac_a > 64 || self.config.divider_frac_b > 64 {
            return Err(SetupError::InvalidGlobalConfig);
        }

        // TODO: Confirm that the selected clock source is enabled in the
        // system / rtc_cntl peripheral? Particularly relevant for clock sources
        // other than APB_CLK

        // TODO: Confirm that the peripheral is not in a reset state and that the
        // peripheral clock (for the registers) is enabled

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

macro_rules! impl_output_channel {
    ($cxi:ident: ($conf0_reg:ident, $data_reg:ident, $lim_reg:ident, $int_complete:ident, $int_err:ident, $int_seq:ident, $int_complete_clr:ident, $int_err_clr:ident, $int_seq_clr:ident)
        ) => {
        /// TX Output Channel
        pub struct $cxi {}
        impl $cxi {
            /// Set the logical level that the connected pin is pulled to
            /// while the channel is idle
            pub fn set_idle_output_level(&mut self, level: bool) -> &mut Self {
                unsafe { &*RMT::ptr() }
                    .$conf0_reg
                    .modify(|_, w| w.idle_out_lv().bit(level));
                self
            }

            /// Enable/Disable the output while the channel is idle
            pub fn set_idle_output(&mut self, state: bool) -> &mut Self {
                unsafe { &*RMT::ptr() }
                    .$conf0_reg
                    .modify(|_, w| w.idle_out_en().bit(state));
                self
            }

            /// Set channel clock divider value
            pub fn set_channel_divider(&mut self, divider: u8) -> &mut Self {
                unsafe { &*RMT::ptr() }
                    .$conf0_reg
                    .modify(|_, w| unsafe { w.div_cnt().bits(divider) });
                self
            }

            /// Enable/Disable carrier modulation
            pub fn set_carrier_modulation(&mut self, state: bool) -> &mut Self {
                unsafe { &*RMT::ptr() }
                    .$conf0_reg
                    .modify(|_, w| w.carrier_en().bit(state));
                self
            }

            /// Assign a pin that should be driven by this channel
            ///
            /// (Note that we only take a reference here, so the ownership remais with the calling
            /// entity. The configured pin thus can be re-configured independently.)
            pub fn configure_pin<RmtPin: OutputPin>(&mut self, pin: &mut RmtPin) -> &mut Self {
                // Configure Pin as output anc connect to signal
                pin.set_to_push_pull_output()
                    .connect_peripheral_to_output(OutputSignal::RMT_SIG_0);

                self
            }

            /// Send an pulse sequence in a blocking fashion
            pub fn send_pulse_sequence(
                &self,
                repeat_mode: &RepeatMode,
                sequence: &[PulseCode],
            ) -> Result<(), TransmissionError> {
                // Write the sequence
                self.write_sequence(sequence);

                // Write the end marker (value "0")
                self.load_fifo(0 as u32);

                // Check how we need to configure the continuous mode flag
                let (cont_mode, count_mode, reps) = match repeat_mode {
                    RepeatMode::SingleShot => (false, false, 0),
                    RepeatMode::RepeatNtimes(val) => {
                        if *val >= 1024 {
                            return Err(TransmissionError::RepetitionOverflow);
                        }

                        (true, true, *val)
                    }
                    RepeatMode::Forever => (true, false, 0),
                };

                // Configure counting mode and repetitions
                unsafe { &*RMT::ptr() }.$lim_reg.modify(|_, w| unsafe {
                    // Set number of repetitions
                    w.tx_loop_num()
                        .bits(reps)
                        // Enable loop counting
                        .tx_loop_cnt_en()
                        .bit(count_mode)
                        // Reset any pre-existing counting value
                        .loop_count_reset()
                        .set_bit()
                });

                // Setup configuration
                unsafe { &*RMT::ptr() }.$conf0_reg.modify(|_, w| {
                    // Set config update bit and configure continuous
                    w.conf_update().set_bit().tx_conti_mode().bit(cont_mode)
                });

                // Clear the relevant interrupts
                //
                // (since this is a write-through register, we can do this
                // safely for multiple separate channel instances without
                // having concurrency issues)
                unsafe { &*RMT::ptr() }.int_clr.write(|w| {
                    w.$int_complete_clr()
                        .set_bit()
                        .$int_err_clr()
                        .set_bit()
                        .$int_seq_clr()
                        .set_bit()
                });

                // Trigger the release of the sequence by the RMT peripheral
                unsafe { &*RMT::ptr() }
                    .$conf0_reg
                    .modify(|_, w| w.tx_start().set_bit());

                // If we're in forever mode, we return right away, otherwise we wait
                // for completion
                if *repeat_mode != RepeatMode::Forever {
                    // Wait for interrupt being raised, either completion or error
                    loop {
                        let interrupts = unsafe { &*RMT::ptr() }.int_raw.read();
                        match (
                            interrupts.$int_complete().bit(),
                            interrupts.$int_seq().bit(),
                            interrupts.$int_err().bit(),
                        ) {
                            // SingleShot completed and no error -> success
                            (true, false, false) => break,
                            // Sequence completed and no error -> success
                            (false, true, false) => {
                                // Stop transmitting (only necessary in sequence case)
                                unsafe { &*RMT::ptr() }
                                    .$conf0_reg
                                    .modify(|_, w| w.tx_stop().set_bit());
                                break;
                            }
                            // Neither completed nor error -> continue busy waiting
                            (false, false, false) => (),
                            // Anything else constitutes an error state
                            _ => return Err(TransmissionError::Failure),
                        }
                    }
                }

                Ok(())
            }

            /// Stop any ongoing (repetitive) transmission
            ///
            /// This function needs to be called to stop sending when
            /// previously a sequence was sent with `RepeatMode::Forever`.
            pub fn stop_transmission(&self) {
                unsafe { &*RMT::ptr() }
                    .$conf0_reg
                    .modify(|_, w| w.tx_stop().set_bit());
            }

            /// Convert a sequence of pulse code structs into u32 and write them
            /// into the RMT fifo buffer
            fn write_sequence(&self, sequence: &[PulseCode]) {
                for pulse in sequence {
                    self.load_fifo(pulse.into());
                }
            }

            /// Write a singular pulse code sequence (two levels with each assigned a duration)
            /// into the RMT fifo buffer
            fn load_fifo(&self, value: u32) {
                unsafe { &*RMT::ptr() }
                    .$data_reg
                    .write(|w| unsafe { w.bits(value) });
            }
        }
    };
}

impl_output_channel!(
    Channel0:
        (
            ch0conf0,
            ch0data,
            ch0_tx_lim,
            ch0_tx_end_int_raw,
            ch0_err_int_raw,
            ch0_tx_loop_int_raw,
            ch0_tx_end_int_clr,
            ch0_err_int_clr,
            ch0_tx_loop_int_clr
        )
);
impl_output_channel!(
    Channel1:
        (
            ch1conf0,
            ch1data,
            ch1_tx_lim,
            ch1_tx_end_int_raw,
            ch1_err_int_raw,
            ch1_tx_loop_int_raw,
            ch1_tx_end_int_clr,
            ch1_err_int_clr,
            ch1_tx_loop_int_clr
        )
);
