//! Low Power Management (and general SoC management)
//! (for now, only the WDT functionality is suported)

use crate::pac::RTCCNTL;

/// A wrapper for the `RTCCNTL` peripheral.
pub struct RtcCntl {
    rtc_cntl: RTCCNTL,
}

impl RtcCntl {
    /// Takes ownership of the `RTCCNTL` peripheral, returning the `RtcCntl` wrapper.
    pub fn new(rtc_cntl: RTCCNTL) -> Self {
        Self { rtc_cntl }
    }

    /// Set the status of the Super Watchdog timer (SWD)
    pub fn set_super_wdt_enable(&self, enable: bool) {
        self.set_swd_write_protection(false);

        // Set autofeed so that the Super WDT is never triggerd
        self.rtc_cntl
            .swd_conf
            .write(|w| w.swd_auto_feed_en().bit(!enable));

        self.set_swd_write_protection(true);
    }

    // Set status of super watchdog register write protection
    fn set_swd_write_protection(&self, enable: bool) {
        // Only if the register has the value `0x8F1D_312Au32` any write operations to
        // the SWD registers will be allowed
        let wkey: u32 = match enable {
            true => 0u32,
            false => 0x8F1D_312Au32,
        };

        self.rtc_cntl
            .swd_wprotect
            .write(|w| unsafe { w.swd_wkey().bits(wkey) });
    }

    /// Set the status of the (regular) Watchdog timer (WDT)
    pub fn set_wdt_enable(&self, enable: bool) {
        self.set_wdt_write_protection(false);

        // Disable the RCT_CNTL WDT
        self.rtc_cntl.wdtconfig0.write(|w| w.wdt_en().bit(enable));

        self.set_wdt_write_protection(true);
    }

    // Set status of watchdog register write protection
    fn set_wdt_write_protection(&self, enable: bool) {
        // Only if the register has the value `0x50D8_3AA1u32` any write operations to
        // the SWD registers will be allowed
        let wkey: u32 = match enable {
            true => 0u32,
            false => 0x50D8_3AA1u32,
        };

        self.rtc_cntl
            .wdtwprotect
            .write(|w| unsafe { w.wdt_wkey().bits(wkey) });
    }
}
