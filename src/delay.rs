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

use crate::nb::block;
use crate::time::{Hertz, U32Ext};
use embedded_hal::{
    blocking::delay::{DelayMs, DelayUs},
    timer::CountDown,
};

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
                T: CountDown<Time = Hertz>,
            {
                fn $delay(&mut self, t: u32) {
                    let mut time_left = t;

                    // Due to the LpTimer having only a 3 bit scaler, it is
                    // possible that the max timeout we can set is
                    // (128 * 65536) / clk_hz milliseconds.
                    // Assuming the fastest clk_hz = 480Mhz this is roughly ~17ms,
                    // or a frequency of ~57.2Hz. We use a 60Hz frequency for each
                    // loop step here to ensure that we stay within these bounds.
                    let looping_delay = $num / 60;
                    let looping_delay_hz = Hertz($num / looping_delay);

                    self.0.start(looping_delay_hz);
                    while time_left > looping_delay {
                        block!(self.0.wait()).ok();
                        time_left -= looping_delay;
                    }

                    if time_left > 0 {
                        self.0.start(($num / time_left).hz());
                        block!(self.0.wait()).ok();
                    }
                }
            }

            impl<T> $Delay<u16> for DelayFromCountDownTimer<T>
            where
                T: CountDown<Time = Hertz>,
            {
                fn $delay(&mut self, t: u16) {
                    self.$delay(t as u32);
                }
            }

            impl<T> $Delay<u8> for DelayFromCountDownTimer<T>
            where
                T: CountDown<Time = Hertz>,
            {
                fn $delay(&mut self, t: u8) {
                    self.$delay(t as u32);
                }
            }
        )+
    }
}

impl_delay_from_count_down_timer! {
    (DelayMs, delay_ms, 1_000),
    (DelayUs, delay_us, 1_000_000)
}
