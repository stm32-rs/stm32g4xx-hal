//! Delays
use core::cmp;
use cortex_m::peripheral::SYST;
use hal::blocking::delay::{DelayMs, DelayUs};

use crate::prelude::*;
use crate::rcc::Clocks;
use crate::time::{Hertz, MicroSecond};

/// System timer (SysTick) as a delay provider
pub struct Delay {
    clk: Hertz,
    syst: SYST,
}

impl Delay {
    /// Configures the system timer (SysTick) as a delay provider
    pub fn new(syst: SYST, clocks: &Clocks) -> Self {
        Delay {
            syst,
            clk: clocks.ahb_clk / 8,
        }
    }

    pub fn delay<T>(&mut self, delay: T)
    where
        T: Into<MicroSecond>,
    {
        let mut cycles = delay.into().cycles(self.clk);
        while cycles > 0 {
            let reload = cmp::min(cycles, 0x00ff_ffff);
            cycles -= reload;
            self.syst.set_reload(reload);
            self.syst.clear_current();
            self.syst.enable_counter();
            while !self.syst.has_wrapped() {}
            self.syst.disable_counter();
        }
    }

    /// Releases the system timer (SysTick) resource
    pub fn release(self) -> SYST {
        self.syst
    }
}

impl DelayUs<u32> for Delay {
    fn delay_us(&mut self, us: u32) {
        self.delay(us.us())
    }
}

impl DelayUs<u16> for Delay {
    fn delay_us(&mut self, us: u16) {
        self.delay_us(us as u32)
    }
}

impl DelayUs<u8> for Delay {
    fn delay_us(&mut self, us: u8) {
        self.delay_us(us as u32)
    }
}

impl DelayMs<u32> for Delay {
    fn delay_ms(&mut self, ms: u32) {
        self.delay_us(ms.saturating_mul(1_000));
    }
}

impl DelayMs<u16> for Delay {
    fn delay_ms(&mut self, ms: u16) {
        self.delay_ms(ms as u32);
    }
}

impl DelayMs<u8> for Delay {
    fn delay_ms(&mut self, ms: u8) {
        self.delay_ms(ms as u32);
    }
}

pub trait DelayExt {
    fn delay(self, clocks: &Clocks) -> Delay;
}

impl DelayExt for SYST {
    fn delay(self, clocks: &Clocks) -> Delay {
        Delay::new(self, clocks)
    }
}
