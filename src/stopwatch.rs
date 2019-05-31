//! A monotonic nondecreasing timer
use crate::rcc::Rcc;
use crate::stm32::TIM2;
use crate::time::{Hertz, Instant, MicroSecond};

pub trait StopwatchExt {
    fn stopwatch(self, rcc: &mut Rcc) -> Stopwatch;
}

pub struct Stopwatch {
    clk: Hertz,
    tim: TIM2,
}

impl Stopwatch {
    pub fn new(tim: TIM2, rcc: &mut Rcc) -> Self {
        assert!(rcc.clocks.apb_tim_clk.0 > 1_000_000);
        rcc.rb.apbenr1.modify(|_, w| w.tim2en().set_bit());
        rcc.rb.apbrstr1.modify(|_, w| w.tim2rst().set_bit());
        rcc.rb.apbrstr1.modify(|_, w| w.tim2rst().clear_bit());
        tim.cr1.modify(|_, w| w.urs().set_bit());
        tim.cr1.modify(|_, w| w.cen().set_bit());
        Stopwatch {
            tim,
            clk: rcc.clocks.apb_tim_clk,
        }
    }

    pub fn set_clock<T>(&mut self, clk: T)
    where
        T: Into<Hertz>,
    {
        let clk = clk.into();
        assert!(clk.0 > 1_000_000);
        self.clk = clk;
    }

    pub fn release(self) -> TIM2 {
        self.tim
    }

    pub fn now(&self) -> Instant {
        let low = self.tim.cnt.read().cnt_l().bits() as u32;
        let high = self.tim.cnt.read().cnt_h().bits() as u32;
        Instant(low | (high << 16))
    }

    pub fn elapsed(&self, ts: Instant) -> MicroSecond {
        let now = self.now().0;
        let cycles = now.wrapping_sub(ts.0);
        self.clk.duration(cycles)
    }

    pub fn trace<F>(&self, mut closure: F) -> MicroSecond
    where
        F: FnMut() -> (),
    {
        let started = self.now().0;
        closure();
        let now = self.now().0;
        self.clk.duration(now.wrapping_sub(started))
    }
}

impl StopwatchExt for TIM2 {
    fn stopwatch(self, rcc: &mut Rcc) -> Stopwatch {
        Stopwatch::new(self, rcc)
    }
}
