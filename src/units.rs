//! Commonly used units throughout this HAL implementation
//!
use core::cmp::{Eq, Ord, Ordering, PartialOrd};

/// 54-bit `Ticks`
#[derive(Clone, Copy, Debug)]
pub struct Ticks54(u64);

impl Ticks54 {
    pub fn into_u64(self) -> u64 {
        // ensure that we ever only return the lower 54-bits set
        self.0 & 0x3FFFFFFFFFFFFF
    }

    pub fn into_u32_tuple(self) -> (u32, u32) {
        // Break down the 64-bit value to two 32-bit register values
        let val = self.into_u64();
        let high = (val >> 32) as u32;
        let low = (val & 0xffffffff) as u32;

        (high, low)
    }
}

impl From<u32> for Ticks54 {
    fn from(val: u32) -> Self {
        Ticks54(val as u64)
    }
}

impl From<(u32, u32)> for Ticks54 {
    fn from((high, low): (u32, u32)) -> Self {
        Ticks54(((high as u64) << 32) + (low as u64))
    }
}

impl From<u64> for Ticks54 {
    fn from(val: u64) -> Self {
        Ticks54(val as u64)
    }
}

impl PartialOrd for Ticks54 {
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        Some(self.cmp(other))
    }
}

impl Ord for Ticks54 {
    fn cmp(&self, other: &Self) -> Ordering {
        self.0.cmp(&other.0)
    }
}

impl PartialEq for Ticks54 {
    fn eq(&self, other: &Self) -> bool {
        self.0 == other.0
    }
}

impl Eq for Ticks54 {}
