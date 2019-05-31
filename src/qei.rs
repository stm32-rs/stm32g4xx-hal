//! Quadrature Encoder Interface
use crate::gpio::gpioa::{PA0, PA1, PA6, PA7, PA8, PA9};
use crate::gpio::{AltFunction, DefaultMode};
use crate::hal::{self, Direction};
use crate::rcc::Rcc;
use crate::stm32::{TIM1, TIM2, TIM3};

pub trait Pins<TIM> {
    fn setup(&self);
}

impl Pins<TIM1> for (PA8<DefaultMode>, PA9<DefaultMode>) {
    fn setup(&self) {
        self.0.set_alt_mode(AltFunction::AF2);
        self.1.set_alt_mode(AltFunction::AF2);
    }
}

impl Pins<TIM2> for (PA0<DefaultMode>, PA1<DefaultMode>) {
    fn setup(&self) {
        self.0.set_alt_mode(AltFunction::AF2);
        self.1.set_alt_mode(AltFunction::AF2);
    }
}

impl Pins<TIM3> for (PA6<DefaultMode>, PA7<DefaultMode>) {
    fn setup(&self) {
        self.0.set_alt_mode(AltFunction::AF2);
        self.1.set_alt_mode(AltFunction::AF2);
    }
}

pub struct Qei<TIM, PINS> {
    tim: TIM,
    pins: PINS,
}

pub trait QeiExt<TIM, PINS>
where
    PINS: Pins<TIM>,
{
    fn qei(self, pins: PINS, rcc: &mut Rcc) -> Qei<TIM, PINS>;
}

macro_rules! qei {
    ($($TIMX:ident: ($tim:ident, $timXen:ident, $timXrst:ident, $apbenr:ident, $apbrstr:ident, $arr:ident, $cnt:ident),)+) => {
        $(
            impl<PINS> Qei<$TIMX, PINS> where PINS: Pins<$TIMX> {
                fn $tim(tim: $TIMX, pins: PINS, rcc: &mut Rcc) -> Self {
                    pins.setup();
                    // enable and reset peripheral to a clean slate state
                    rcc.rb.$apbenr.modify(|_, w| w.$timXen().set_bit());
                    rcc.rb.$apbrstr.modify(|_, w| w.$timXrst().set_bit());
                    rcc.rb.$apbrstr.modify(|_, w| w.$timXrst().clear_bit());

                    // Configure TxC1 and TxC2 as captures
                    tim.ccmr1_output.write(|w| unsafe {
                        w.cc1s().bits(0b01).cc2s().bits(0b01)
                    });

                    // Encoder mode, count up/down on both TI1FP1 and TI2FP2
                    tim.smcr.write(|w| unsafe { w.sms().bits(0b011) });

                    // Enable and configure to capture on rising edge
                    tim.ccer.write(|w| {
                        w.cc1e()
                            .set_bit()
                            .cc2e()
                            .set_bit()
                            .cc1p()
                            .clear_bit()
                            .cc2p()
                            .clear_bit()
                            .cc1np()
                            .clear_bit()
                            .cc2np()
                            .clear_bit()
                    });

                    tim.arr.write(|w| unsafe { w.$arr().bits(0xffff) });
                    tim.cr1.write(|w| w.cen().set_bit());
                    Qei { tim, pins }
                }

                pub fn release(self) -> ($TIMX, PINS) {
                    (self.tim, self.pins)
                }
            }

            impl<PINS> hal::Qei for Qei<$TIMX, PINS> {
                type Count = u16;

                fn count(&self) -> u16 {
                    self.tim.cnt.read().$cnt().bits()
                }

                fn direction(&self) -> Direction {
                    if self.tim.cr1.read().dir().bit_is_clear() {
                        hal::Direction::Upcounting
                    } else {
                        hal::Direction::Downcounting
                    }
                }
            }

            impl<PINS> QeiExt<$TIMX, PINS> for $TIMX where PINS: Pins<$TIMX> {
                fn qei(self, pins: PINS, rcc: &mut Rcc) -> Qei<$TIMX, PINS> {
                    Qei::$tim(self, pins, rcc)
                }
            }
        )+
    }
}

qei! {
    TIM1: (tim1,  tim1en, tim1rst, apbenr2, apbrstr2, arr, cnt),
    TIM2: (tim2,  tim2en, tim2rst, apbenr1, apbrstr1, arr_l, cnt_l),
    TIM3: (tim3,  tim3en, tim3rst, apbenr1, apbrstr1, arr_l, cnt_l),
}
