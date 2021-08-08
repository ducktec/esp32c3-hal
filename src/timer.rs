//! Timer Functionality (TIMG)
//!
//! Reference: ESP32-C3 TRM v0.2 Section 8 (as TRM)
//!
//! The ESP32-C3 has two identical timer groups TIMG0 and TIMG1, which both consist
//! of a timer (T0) and a Watchdog Timer (WDT)
//!
//! TODO:
//! - Support timer interrupts
//! - Support WDT

use embedded_hal::timer::{Cancel, CountDown, Periodic};
use embedded_hal::watchdog::WatchdogDisable;

use crate::pac::{timg::RegisterBlock, TIMG0, TIMG1};
use crate::units::Ticks54;
use void::Void;

/// Timer groups (TIMG)
pub struct Timer<T> {
    timg: T,
}

/// TIMG-related errors
pub enum Error {
    /// Report that the timer is active and certain management
    /// operations cannot be performed safely
    TimerActive,
    /// Report that the timer is inactive and thus not
    /// ever reaching any potentially configured alarm value
    TimerInactive,
    /// Report that the alarm functionality is disabled
    AlarmInactive,
}

impl<T> Timer<T>
where
    T: Instance,
{
    pub fn new(timg: T) -> Self {
        let timer = Timer { timg };
        timer
    }
}

pub trait Instance {
    /// Return registerblock of timer instance as if it were TIMG0
    fn as_timg0(&self) -> &RegisterBlock;

    /// Load a new 54-bit counter value in the counter registers
    fn load_counter_value(&mut self, value: Ticks54) {
        // Break down the 64-bit value to two 32-bit register values
        let (high, low) = value.into_u32_tuple();
        self.as_timg0()
            .t0loadlo
            .write(|w| unsafe { w.t0_load_lo().bits(low) });
        self.as_timg0()
            .t0loadhi
            .write(|w| unsafe { w.t0_load_hi().bits(high) });

        // Trigger reload from the previously updated registers
        self.as_timg0()
            .t0load
            .write(|w| unsafe { w.t0_load().bits(1) });
    }

    /// Read the current counter value from the timer
    fn get_counter_value(&mut self) -> Ticks54 {
        // Trigger update of the `t0hi` and `t0lo` registers by writing update bit
        self.as_timg0().t0update.write(|w| w.t0_update().set_bit());
        // Read `t0hi` and `t0lo` (32-bit) registers and convert to 54-bit ticks value
        Ticks54::from((
            self.as_timg0().t0hi.read().bits(),
            self.as_timg0().t0lo.read().bits(),
        ))
    }

    /// Reset the timer's counter value to 0x00 (TRM 8.2.4)
    fn reset_counter(&mut self) {
        self.as_timg0()
            .t0loadlo
            .write(|w| unsafe { w.t0_load_lo().bits(0) });
        self.as_timg0()
            .t0loadhi
            .write(|w| unsafe { w.t0_load_hi().bits(0) });
        // Trigger reload from the previously updated registers
        self.as_timg0()
            .t0load
            .write(|w| unsafe { w.t0_load().bits(1) });
    }

    /// Enable or disable the counting of the timer
    fn set_counter_active(&mut self, state: bool) {
        self.as_timg0().t0config.modify(|_, w| w.t0_en().bit(state));
    }

    /// Get the status of the counter
    fn is_counter_active(&mut self) -> bool {
        self.as_timg0().t0config.read().t0_en().bit_is_set()
    }

    /// Specify if the counter should decrement
    fn set_counter_decrementing(&mut self, decrementing: bool) {
        // Field should be set to 0 if decrementing and to 1 if incrementing
        self.as_timg0()
            .t0config
            .modify(|_, w| w.t0_increase().bit(!decrementing));
    }

    /// Configure if the counter should reload its counter value once
    /// it reaches its alarm value
    fn set_auto_reload(&mut self, auto_reload: bool) {
        self.as_timg0()
            .t0config
            .modify(|_, w| w.t0_autoreload().bit(auto_reload));
    }

    /// Enable or disable the alarm of the timer
    fn set_alarm_active(&mut self, state: bool) {
        self.as_timg0()
            .t0config
            .modify(|_, w| w.t0_alarm_en().bit(state));
    }

    /// Get the status of the alarm
    fn is_alarm_active(&mut self) -> bool {
        self.as_timg0().t0config.read().t0_alarm_en().bit_is_set()
    }

    /// Load a new 54-bit value into the alarm registers
    fn load_alarm_value(&mut self, value: Ticks54) {
        // Break down the 64-bit value to two 32-bit register values
        let (high, low) = value.into_u32_tuple();
        self.as_timg0()
            .t0alarmlo
            .write(|w| unsafe { w.t0_alarm_lo().bits(low) });
        self.as_timg0()
            .t0alarmhi
            .write(|w| unsafe { w.t0_alarm_hi().bits(high) });
    }

    /// Read the current alarm value from the timer
    fn get_alarm_value(&mut self) -> Ticks54 {
        // Read `t0alarmhi` and `t0alarmlo` (32-bit) registers and convert to 54-bit ticks value
        Ticks54::from((
            self.as_timg0().t0alarmhi.read().bits(),
            self.as_timg0().t0alarmlo.read().bits(),
        ))
    }

    /// Enable the alarm interrupt
    fn enable_interrupt(&mut self) {
        self.as_timg0()
            .int_ena_timers
            .modify(|_, w| w.t0_int_ena().set_bit());
    }

    /// Disable the alarm interrupt
    fn disable_interrupt(&mut self) {
        self.as_timg0()
            .int_ena_timers
            .modify(|_, w| w.t0_int_ena().clear_bit());
    }

    /// Assign a custom prescaler value for the timer
    ///
    /// By default, the prescaler value is set to 1, resulting
    /// in a division by 2.
    ///
    /// See TRM 8.2.1 for details.
    fn set_prescaler(&mut self, prescaler_value: u16) -> Result<(), Error> {
        // The timer must be disabled for this operation
        match self.as_timg0().t0config.read().t0_en().bit_is_clear() {
            true => {
                // Update divider value and trigger reload
                // TODO: Check that it's feasible to assign and write through in one operation
                self.as_timg0().t0config.modify(|_, w| unsafe {
                    w.t0_divider()
                        .bits(prescaler_value)
                        .t0_divcnt_rst()
                        .set_bit()
                });
                Ok(())
            }
            false => Err(Error::TimerActive),
        }
    }

    /// Get the currently used prescaler value
    fn get_prescaler(&mut self) -> u16 {
        self.as_timg0().t0config.read().t0_divider().bits()
    }

    /// Select the clock source for the timer
    ///
    /// Can be either `XTALclk` or `APBclk`
    ///
    /// See TRM 8.2.1 for details.
    fn set_use_xtal(&mut self, use_xtal: bool) {
        self.as_timg0()
            .t0config
            .modify(|_, w| w.t0_use_xtal().bit(use_xtal));

        // Ensure that the clock is actually activated
        self.as_timg0()
            .clk
            .modify(|_, w| w.timer_clk_is_active().set_bit());
    }

    /// Get the current configuration of the clock source
    fn get_use_xtal(&mut self) -> bool {
        self.as_timg0().t0config.read().t0_use_xtal().bit()
    }

    /// Change the status of the watchdog functionality of the timer group
    fn set_wdt_enabled(&mut self, enabled: bool) {
        // Disable the write protection for the TIMG WDT registers
        self.as_timg0()
            .wdtwprotect
            .write(|w| unsafe { w.wdt_wkey().bits(0x50D8_3AA1u32) });

        // Disable the TIMG WDT
        self.as_timg0()
            .wdtconfig0
            .write(|w| w.wdt_en().bit(enabled));

        // Re-Enable the write protection for the TIMG WDT registers
        self.as_timg0()
            .wdtwprotect
            .write(|w| unsafe { w.wdt_wkey().bits(0u32) });
    }

    /// Return if the WDT functionality of the timer group is activated
    fn is_wdt_enabled(&mut self) -> bool {
        self.as_timg0().wdtconfig0.read().wdt_en().bit()
    }
}

impl<T> CountDown for Timer<T>
where
    T: Instance,
{
    type Time = Ticks54;

    /// Start the timer with a `timeout`
    fn start<Time>(&mut self, timeout: Time)
    where
        Time: Into<Ticks54>,
    {
        // FIXME: Use more generic Time unit!

        // stop any ongoing counting activity
        self.timg.set_counter_active(false);
        self.timg.set_alarm_active(false);

        // reset the current counter value to 0x00
        self.timg.reset_counter();

        // set the alarm values
        // FIXME: consider fequency/prescaler
        self.timg.load_alarm_value(timeout.into());

        // configure and start counting
        self.timg.set_counter_decrementing(false);
        self.timg.set_auto_reload(true);
        self.timg.set_counter_active(true);
        self.timg.set_alarm_active(true);
    }

    /// Return `Ok` if the timer has wrapped
    /// Automatically clears the flag and restarts the time
    fn wait(&mut self) -> nb::Result<(), Void> {
        if !self.timg.is_counter_active() {
            panic!("Called wait on an inactive timer!");
        }

        if self
            .timg
            .as_timg0()
            .int_raw_timers
            .read()
            .t0_int_raw()
            .bit_is_clear()
        {
            Err(nb::Error::WouldBlock)
        } else {
            // Alarm has been triggered, reset the interrupt
            // (periodic due to auto-reload)
            self.timg
                .as_timg0()
                .int_clr_timers
                .write(|w| w.t0_int_clr().set_bit());
            self.timg.set_alarm_active(true);
            Ok(())
        }
    }
}

impl<T> Periodic for Timer<T> where T: Instance {}

impl<T> Cancel for Timer<T>
where
    T: Instance,
{
    type Error = Error;

    /// Stop running and armed timer.
    ///
    /// This will return an error if the counter was not running or the
    /// alarm was not enabled.
    fn cancel(&mut self) -> Result<(), Error> {
        if !self.timg.is_counter_active() {
            return Err(Error::TimerInactive);
        } else if !self.timg.is_alarm_active() {
            return Err(Error::AlarmInactive);
        }
        self.timg.set_counter_active(false);
        Ok(())
    }
}

impl<T> WatchdogDisable for Timer<T>
where
    T: Instance,
{
    fn disable(&mut self) {
        self.timg.set_wdt_enabled(false);
    }
}

impl Instance for TIMG0 {
    #[inline(always)]
    fn as_timg0(&self) -> &RegisterBlock {
        self
    }
}

impl Instance for TIMG1 {
    #[inline(always)]
    fn as_timg0(&self) -> &RegisterBlock {
        self
    }
}
