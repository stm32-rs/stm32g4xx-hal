//! Delay providers
//!
//! There are currently two delay providers. In general you should prefer to use
//! [Delay](Delay), however if you do not have access to `SYST` you can use
//! [DelayFromCountDownTimer](DelayFromCountDownTimer) with any timer that
//! implements the [CountDown](embedded_hal::timer::CountDown) trait. This can be
//! useful if you're using [RTIC](https://rtic.rs)'s schedule API, which occupies
//! the `SYST` peripheral.
//!
//! # Examples
//!
//! ## Delay
//!
//! ```no_run
//! let rcc =  Peripherals::take().unwrap().contrain();
//! let rcc = rcc.freeze(Config::hsi());
//! let mut delay = cp.SYST.delay(&rcc.clocks);
//!
//! delay.delay(500.ms() );
//!
//! // Release SYST from the delay
//! let syst = delay.free();
//! ```
//!
//! ## DelayFromCountDownTimer
//!
//! ```no_run
//! let timer2 = device
//!     .TIM2
//!     .timer(100.ms(), device.peripheral.TIM2, &mut device.clocks);
//! let mut delay = DelayFromCountDownTimer::new(timer2);
//!
//! delay.delay_ms(500);
//!
//! // Release the timer from the delay
//! let timer2 = delay.free();
//! ```

use crate::rcc::Clocks;
use crate::time::MicroSecond;
pub use cortex_m::delay::*;
use cortex_m::peripheral::SYST;
use fugit::ExtU32Ceil;

use crate::nb::block;
use crate::time::ExtU32;
use embedded_hal_old::blocking::delay::{DelayMs, DelayUs};
use embedded_hal::delay::DelayNs;

pub trait CountDown: embedded_hal_old::timer::CountDown {
    fn max_period(&self) -> MicroSecond;
}

pub trait SYSTDelayExt {
    fn delay(self, clocks: &Clocks) -> Delay;
}

impl SYSTDelayExt for SYST {
    fn delay(self, clocks: &Clocks) -> Delay {
        Delay::new(self, clocks.ahb_clk.raw())
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
        self.delay_us(delay.into().ticks())
    }
}

/// CountDown Timer as a delay provider
pub struct DelayFromCountDownTimer<T>(T);

impl<T> DelayFromCountDownTimer<T> {
    /// Creates delay provider from a CountDown timer
    pub fn new(timer: T) -> Self {
        Self(timer)
    }

    /// Releases the Timer
    pub fn free(self) -> T {
        self.0
    }
}

macro_rules! impl_delay_from_count_down_timer  {
    ($(($Delay:ident, $delay:ident, $num:expr)),+) => {
        $(

            impl<T> $Delay<u32> for DelayFromCountDownTimer<T>
            where
                T: CountDown<Time = MicroSecond>,
            {
                fn $delay(&mut self, t: u32) {
                    let mut time_left_us = t as u64 * $num;

                    let max_sleep = self.0.max_period();
                    let max_sleep_us = max_sleep.to_micros() as u64;

                    if time_left_us > max_sleep_us {
                        self.0.start(max_sleep);

                        // Process the time one max_sleep duration at a time
                        // to avoid overflowing both u32 and the timer
                        for _ in 0..(time_left_us / max_sleep_us) {
                            block!(self.0.wait()).ok();
                            time_left_us -= max_sleep_us;
                        }
                    }

                    assert!(time_left_us <= u32::MAX as u64);
                    assert!(time_left_us <= max_sleep_us);

                    let time_left: MicroSecond = (time_left_us as u32).micros();

                    // Only sleep
                    if time_left.ticks() > 0 {
                        self.0.start(time_left);
                        block!(self.0.wait()).ok();
                    }
                }
            }

            impl<T> $Delay<u16> for DelayFromCountDownTimer<T>
            where
                T: CountDown<Time = MicroSecond>,
            {
                fn $delay(&mut self, t: u16) {
                    $Delay::$delay(self, t as u32);
                }
            }

            impl<T> $Delay<u8> for DelayFromCountDownTimer<T>
            where
                T: CountDown<Time = MicroSecond>,
            {
                fn $delay(&mut self, t: u8) {
                    $Delay::$delay(self, t as u32);
                }
            }
        )+
    }
}

impl_delay_from_count_down_timer! {
    (DelayMs, delay_ms, 1_000),
    (DelayUs, delay_us, 1)
}

// TODO: decouple the timer API from embedded-hal 0.2, stm32f4xx-hal with constant timer
// frequencies looks like a good example
impl<T: CountDown<Time = MicroSecond>> DelayNs for DelayFromCountDownTimer<T> {
    // From a quick look at the clock diagram timer resolution can go down to ~3ns on most timers
    // 170MHz (PLL-R as SysClk) / 1 (AHB Prescaler) / 1 (APBx prescaler) * 2 = 340MHz timer clock
    // TODO: the current fallback to 1us resolution is a stopgap until the module is reworked
    fn delay_ns(&mut self, ns: u32) {
        DelayNs::delay_us(self, ns.div_ceil(1000))
    }
    fn delay_us(&mut self, mut us: u32) {
        let max_sleep = self.0.max_period();
        let max_us = max_sleep.to_micros();

        while us > max_us {
            self.0.start(max_us.micros_at_least());
            let _ = nb::block!(self.0.wait());
            us -= max_us;
        }
        self.0.start(us.micros_at_least());
        let _ = nb::block!(self.0.wait());
    }
    fn delay_ms(&mut self, mut ms: u32) {
        const MAX_MILLIS: u32 = u32::MAX / 1000;
        while ms > MAX_MILLIS {
            ms -= MAX_MILLIS;
            DelayNs::delay_us(self, MAX_MILLIS * 1000);
        }
        DelayNs::delay_us(self, ms * 1000);
    }
}
