//! Timers
//!
//! Pins can be used for PWM output in both push-pull mode (`Alternate`) and open-drain mode
//! (`AlternateOD`).

use cast::{u16, u32};
use cortex_m::peripheral::syst::SystClkSource;
use cortex_m::peripheral::{DCB, DWT, SYST};
use embedded_hal::timer::{Cancel, CountDown, Periodic};
use void::Void;

use crate::stm32::RCC;

use crate::rcc::{self, Clocks};
use crate::time::Hertz;

/// Timer wrapper
pub struct Timer<TIM> {
    pub(crate) tim: TIM,
    pub(crate) clk: Hertz,
}

/// Hardware timers
pub struct CountDownTimer<TIM> {
    tim: TIM,
    clk: Hertz,
}

impl<TIM> Timer<TIM>
where
    CountDownTimer<TIM>: CountDown<Time = Hertz>,
{
    /// Starts timer in count down mode at a given frequency
    pub fn start_count_down<T>(self, timeout: T) -> CountDownTimer<TIM>
    where
        T: Into<Hertz>,
    {
        let Self { tim, clk } = self;
        let mut timer = CountDownTimer { tim, clk };
        timer.start(timeout);
        timer
    }
}

impl<TIM> Periodic for CountDownTimer<TIM> {}

/// Interrupt events
pub enum Event {
    /// CountDownTimer timed out / count down ended
    TimeOut,
}

#[derive(Debug, Eq, PartialEq, Copy, Clone)]
pub enum Error {
    /// CountDownTimer is disabled
    Disabled,
}

impl Timer<SYST> {
    /// Initialize timer
    pub fn syst(mut syst: SYST, clocks: &Clocks) -> Self {
        syst.set_clock_source(SystClkSource::Core);
        Self {
            tim: syst,
            clk: clocks.sys_clk,
        }
    }

    pub fn release(self) -> SYST {
        self.tim
    }
}

impl CountDownTimer<SYST> {
    /// Starts listening for an `event`
    pub fn listen(&mut self, event: Event) {
        match event {
            Event::TimeOut => self.tim.enable_interrupt(),
        }
    }

    /// Stops listening for an `event`
    pub fn unlisten(&mut self, event: Event) {
        match event {
            Event::TimeOut => self.tim.disable_interrupt(),
        }
    }
}

impl CountDown for CountDownTimer<SYST> {
    type Time = Hertz;

    fn start<T>(&mut self, timeout: T)
    where
        T: Into<Hertz>,
    {
        let rvr = self.clk.0 / timeout.into().0 - 1;

        assert!(rvr < (1 << 24));

        self.tim.set_reload(rvr);
        self.tim.clear_current();
        self.tim.enable_counter();
    }

    fn wait(&mut self) -> nb::Result<(), Void> {
        if self.tim.has_wrapped() {
            Ok(())
        } else {
            Err(nb::Error::WouldBlock)
        }
    }
}

impl Cancel for CountDownTimer<SYST> {
    type Error = Error;

    fn cancel(&mut self) -> Result<(), Self::Error> {
        if !self.tim.is_counter_enabled() {
            return Err(Self::Error::Disabled);
        }

        self.tim.disable_counter();
        Ok(())
    }
}

/// A monotonic non-decreasing timer
///
/// This uses the timer in the debug watch trace peripheral. This means, that if the
/// core is stopped, the timer does not count up. This may be relevant if you are using
/// cortex_m_semihosting::hprintln for debugging in which case the timer will be stopped
/// while printing
#[derive(Clone, Copy)]
pub struct MonoTimer {
    frequency: Hertz,
}

impl MonoTimer {
    /// Creates a new `Monotonic` timer
    pub fn new(mut dwt: DWT, mut dcb: DCB, clocks: &Clocks) -> Self {
        dcb.enable_trace();
        dwt.enable_cycle_counter();

        // now the CYCCNT counter can't be stopped or reset
        drop(dwt);

        MonoTimer {
            frequency: clocks.ahb_clk,
        }
    }

    /// Returns the frequency at which the monotonic timer is operating at
    pub fn frequency(self) -> Hertz {
        self.frequency
    }

    /// Returns an `Instant` corresponding to "now"
    pub fn now(self) -> Instant {
        Instant {
            now: DWT::get_cycle_count(),
        }
    }
}

/// A measurement of a monotonically non-decreasing clock
#[derive(Clone, Copy)]
pub struct Instant {
    now: u32,
}

impl Instant {
    /// Ticks elapsed since the `Instant` was created
    pub fn elapsed(self) -> u32 {
        DWT::get_cycle_count().wrapping_sub(self.now)
    }
}

pub trait Instance: crate::Sealed + rcc::Enable + rcc::Reset + rcc::GetBusFreq {}

impl<TIM> Timer<TIM>
where
    TIM: Instance,
{
    /// Initialize timer
    pub fn new(tim: TIM, clocks: &Clocks) -> Self {
        unsafe {
            //NOTE(unsafe) this reference will only be used for atomic writes with no side effects
            let rcc = &(*RCC::ptr());
            // Enable and reset the timer peripheral
            TIM::enable(rcc);
            TIM::reset(rcc);
        }

        Self {
            clk: TIM::get_timer_frequency(clocks),
            tim,
        }
    }
}

macro_rules! hal {
    ($($TIM:ty: ($tim:ident),)+) => {
        $(
            impl Instance for $TIM { }

            impl CountDownTimer<$TIM> {
                /// Starts listening for an `event`
                ///
                /// Note, you will also have to enable the TIM2 interrupt in the NVIC to start
                /// receiving events.
                pub fn listen(&mut self, event: Event) {
                    match event {
                        Event::TimeOut => {
                            // Enable update event interrupt
                            self.tim.dier.write(|w| w.uie().set_bit());
                        }
                    }
                }

                /// Clears interrupt associated with `event`.
                ///
                /// If the interrupt is not cleared, it will immediately retrigger after
                /// the ISR has finished.
                pub fn clear_interrupt(&mut self, event: Event) {
                    match event {
                        Event::TimeOut => {
                            // Clear interrupt flag
                            self.tim.sr.write(|w| w.uif().clear_bit());
                        }
                    }
                }

                /// Stops listening for an `event`
                pub fn unlisten(&mut self, event: Event) {
                    match event {
                        Event::TimeOut => {
                            // Enable update event interrupt
                            self.tim.dier.write(|w| w.uie().clear_bit());
                        }
                    }
                }

                /// Releases the TIM peripheral
                pub fn release(self) -> $TIM {
                    // pause counter
                    self.tim.cr1.modify(|_, w| w.cen().clear_bit());
                    self.tim
                }
            }

            impl CountDown for CountDownTimer<$TIM> {
                type Time = Hertz;

                fn start<T>(&mut self, timeout: T)
                where
                    T: Into<Hertz>,
                {
                    // pause
                    self.tim.cr1.modify(|_, w| w.cen().clear_bit());
                    // reset counter
                    self.tim.cnt.reset();

                    let frequency = timeout.into().0;
                    let ticks = self.clk.0 / frequency;

                    let psc = u16((ticks - 1) / (1 << 16)).unwrap();
                    self.tim.psc.write(|w| unsafe {w.psc().bits(psc)} );

                    let arr = u16(ticks / u32(psc + 1)).unwrap();
                    self.tim.arr.write(|w| unsafe { w.bits(u32(arr)) });

                    // Trigger update event to load the registers
                    self.tim.cr1.modify(|_, w| w.urs().set_bit());
                    self.tim.egr.write(|w| w.ug().set_bit());
                    self.tim.cr1.modify(|_, w| w.urs().clear_bit());

                    // start counter
                    self.tim.cr1.modify(|_, w| w.cen().set_bit());
                }

                fn wait(&mut self) -> nb::Result<(), Void> {
                    if self.tim.sr.read().uif().bit_is_clear() {
                        Err(nb::Error::WouldBlock)
                    } else {
                        self.tim.sr.modify(|_, w| w.uif().clear_bit());
                        Ok(())
                    }
                }
            }

            impl Cancel for CountDownTimer<$TIM>
            {
                type Error = Error;

                fn cancel(&mut self) -> Result<(), Self::Error> {
                    // let is_counter_enabled = self.tim.cr1.read().cen().is_enabled();
                    let is_counter_enabled = self.tim.cr1.read().cen().bit_is_set();
                    if !is_counter_enabled {
                        return Err(Self::Error::Disabled);
                    }

                    // disable counter
                    self.tim.cr1.modify(|_, w| w.cen().clear_bit());
                    Ok(())
                }
            }
        )+
    }
}

hal! {
    crate::stm32::TIM1: (tim1),
    crate::stm32::TIM2: (tim2),
    crate::stm32::TIM3: (tim3),
    crate::stm32::TIM4: (tim4),
    crate::stm32::TIM6: (tim6),
    crate::stm32::TIM7: (tim7),
    crate::stm32::TIM8: (tim8),

    crate::stm32::TIM15: (tim15),
    crate::stm32::TIM16: (tim16),
    crate::stm32::TIM17: (tim17),
}

#[cfg(any(
    feature = "stm32g471",
    feature = "stm32g473",
    feature = "stm32g474",
    feature = "stm32g483",
    feature = "stm32g484"
))]
hal! {
    crate::stm32::TIM5: (tim5),
}

#[cfg(any(
    feature = "stm32g473",
    feature = "stm32g474",
    feature = "stm32g483",
    feature = "stm32g484"
))]
hal! {
    crate::stm32::TIM20: (tim20),
}

use crate::gpio::{
    gpioa::*, gpiob::*, gpioc::*, gpiod::*, gpioe::*, gpiof::*, Alternate, AlternateOD,
};

// Output channels marker traits
pub trait PinC1<TIM> {}
pub trait PinC2<TIM> {}
pub trait PinC3<TIM> {}
pub trait PinC4<TIM> {}

macro_rules! channel_impl {
    ( $( $TIM:ident, $PINC:ident, $PINX:ident, $AF:literal; )+ ) => {
        $(
            impl $PINC<crate::stm32::$TIM> for $PINX<Alternate<$AF>> {}
            impl $PINC<crate::stm32::$TIM> for $PINX<AlternateOD<$AF>> {}
        )+
    };
}
channel_impl!(
    TIM1, PinC1, PA8, 4;
    TIM1, PinC1, PC0, 1;
    TIM1, PinC2, PA9, 3;
    TIM1, PinC2, PC1, 1;
    TIM1, PinC2, PE11, 0;
    TIM1, PinC3, PA10, 4;
    TIM1, PinC3, PC2, 1;
    TIM1, PinC3, PE13, 0;
    TIM1, PinC4, PA11, 7;
    TIM1, PinC4, PC3, 1;
    TIM1, PinC4, PE14, 0;
);

channel_impl!(
    TIM2, PinC1, PA0, 0;
    TIM2, PinC1, PA5, 0;
    TIM2, PinC1, PA15, 1;
    TIM2, PinC1, PD3, 0;
    TIM2, PinC2, PA1, 1;
    TIM2, PinC2, PB3, 1;
    TIM2, PinC2, PD4, 0;
    TIM2, PinC3, PA2, 0;
    TIM2, PinC3, PB10, 0;
    TIM2, PinC4, PA3, 0;
    TIM2, PinC4, PA10, 7;
    TIM2, PinC4, PB11, 0;
    TIM2, PinC4, PD7, 0;
    TIM2, PinC4, PD6, 0;
);

channel_impl!(
    TIM3, PinC1, PA6, 1;
    TIM3, PinC1, PB4, 2;
    TIM3, PinC1, PC6, 0;
    TIM3, PinC1, PE2, 0;
    TIM3, PinC2, PA4, 0;
    TIM3, PinC2, PA7, 1;
    TIM3, PinC2, PA9, 7;
    TIM3, PinC2, PB5, 1;
    TIM3, PinC2, PC7, 0;
    TIM3, PinC2, PE3, 1;
    TIM3, PinC3, PB0, 1;
    TIM3, PinC3, PC8, 0;
    TIM3, PinC3, PE4, 1;
    TIM3, PinC4, PB1, 0;
    TIM3, PinC4, PB7, 7;
    TIM3, PinC4, PC9, 0;
    TIM3, PinC4, PE5, 1;
);

channel_impl!(
    TIM4, PinC1, PA11, 5;
    TIM4, PinC1, PB6, 1;
    TIM4, PinC1, PD12, 0;
    TIM4, PinC2, PA12, 6;
    TIM4, PinC2, PB7, 2;
    TIM4, PinC2, PD13, 0;
    TIM4, PinC3, PA13, 6;
    TIM4, PinC3, PB8, 1;
    TIM4, PinC3, PD14, 0;
    TIM4, PinC4, PB9, 1;
    TIM4, PinC4, PF6, 1;
    TIM4, PinC4, PD15, 0;
);

#[cfg(any(
    feature = "stm32g471",
    feature = "stm32g473",
    feature = "stm32g474",
    feature = "stm32g483",
    feature = "stm32g484"
))]
channel_impl!(
    TIM5, PinC1, PA0, 1;
    TIM5, PinC1, PF6, 4;
    TIM5, PinC1, PB2, 2;
    TIM5, PinC2, PA1, 2;
    TIM5, PinC2, PC12, 0;
    TIM5, PinC2, PF7, 1;
    TIM5, PinC3, PA0, 1;
    TIM5, PinC3, PE8, 0;
    TIM5, PinC3, PF8, 1;
    TIM5, PinC4, PA3, 1;
    TIM5, PinC4, PE9, 0;
    TIM5, PinC4, PF9, 3;
);

channel_impl!(
    TIM8, PinC1, PA15, 2;
    TIM8, PinC1, PB6, 2;
    TIM8, PinC2, PA14, 4;
    TIM8, PinC2, PB8, 7;
    TIM8, PinC3, PB9, 8;
    TIM8, PinC3, PC8, 2;
    TIM8, PinC4, PD1, 0;
);

channel_impl!(
    TIM15, PinC1, PA2, 4;
    TIM15, PinC1, PB14, 0;
    TIM15, PinC1, PB15, 1;
    TIM15, PinC1, PF9, 1;
    TIM15, PinC2, PA3, 4;
    TIM15, PinC2, PF10, 1;
);

channel_impl!(
    TIM16, PinC1, PA6, 0;
    TIM16, PinC1, PA12, 0;
    TIM16, PinC1, PB4, 1;
    TIM16, PinC1, PB8, 0;
    TIM16, PinC1, PE0, 2;
);

channel_impl!(
    TIM17, PinC1, PA7, 0;
    TIM17, PinC1, PB5, 9;
    TIM17, PinC1, PB9, 0;
    TIM17, PinC1, PE1, 0;
);

#[cfg(any(
    feature = "stm32g473",
    feature = "stm32g474",
    feature = "stm32g483",
    feature = "stm32g484"
))]
channel_impl!(
    TIM20, PinC1, PB2, 3;
    TIM20, PinC1, PE2, 4;
    TIM20, PinC1, PF12, 0;
    TIM20, PinC2, PC2, 3;
    TIM20, PinC2, PE3, 3;
    TIM20, PinC2, PF13, 0;
    TIM20, PinC3, PA2, 2;
    TIM20, PinC3, PC8, 3;
    TIM20, PinC3, PF2, 0;
    TIM20, PinC3, PF14, 0;
    TIM20, PinC4, PE1, 1;
    TIM20, PinC4, PF3, 0;
    TIM20, PinC4, PF15, 0;
);
