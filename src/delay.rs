use crate::rcc::Clocks;
use crate::time::MicroSecond;
pub use cortex_m::delay::*;
use cortex_m::{peripheral::SYST, prelude::_embedded_hal_blocking_delay_DelayUs};

pub trait SYSTDelayExt {
    fn delay(self, clocks: &Clocks) -> Delay;
}

impl SYSTDelayExt for SYST {
    fn delay(self, clocks: &Clocks) -> Delay {
        Delay::new(self, clocks.ahb_clk.0)
    }
}

pub trait DelayExt {
    fn delay<T>(&mut self, delay: T)
    where
        T: Into<MicroSecond>;
}

impl DelayExt for Delay {
    fn delay<T>(&mut self, delay: T)
    where
        T: Into<MicroSecond>,
    {
        self.delay_us(delay.into().0)
    }
}
