//! # Pulse Width Modulation

use core::marker::PhantomData;
use core::mem;

use crate::gpio::gpioa::*;
use crate::gpio::gpiob::*;
use crate::gpio::gpioc::*;
use crate::gpio::gpiod::*;
use crate::gpio::gpiof::*;
use crate::gpio::{AltFunction, DefaultMode};
use crate::rcc::Rcc;
use crate::stm32::*;
use crate::time::Hertz;
use hal;

pub struct C1;
pub struct C2;
pub struct C3;
pub struct C4;
pub struct C5;
pub struct C6;

pub trait Pins<TIM> {
    type Channel;
    fn setup(&self);
}

impl<TIM, CH1, CH2> Pins<TIM> for (CH1, CH2)
where
    CH1: Pins<TIM>,
    CH2: Pins<TIM>,
{
    type Channel = (Pwm<TIM, C1>, Pwm<TIM, C2>);

    fn setup(&self) {
        self.0.setup();
        self.1.setup();
    }
}

impl<TIM, CH1, CH2, CH3> Pins<TIM> for (CH1, CH2, CH3)
where
    CH1: Pins<TIM>,
    CH2: Pins<TIM>,
    CH3: Pins<TIM>,
{
    type Channel = (Pwm<TIM, C1>, Pwm<TIM, C2>, Pwm<TIM, C3>);

    fn setup(&self) {
        self.0.setup();
        self.1.setup();
        self.2.setup();
    }
}

impl<TIM, CH1, CH2, CH3, CH4> Pins<TIM> for (CH1, CH2, CH3, CH4)
where
    CH1: Pins<TIM>,
    CH2: Pins<TIM>,
    CH3: Pins<TIM>,
    CH4: Pins<TIM>,
{
    type Channel = (Pwm<TIM, C1>, Pwm<TIM, C2>, Pwm<TIM, C3>, Pwm<TIM, C4>);

    fn setup(&self) {
        self.0.setup();
        self.1.setup();
        self.2.setup();
        self.3.setup();
    }
}

pub trait PwmExt: Sized {
    fn pwm<PINS, T>(self, _: PINS, frequency: T, rcc: &mut Rcc) -> PINS::Channel
    where
        PINS: Pins<Self>,
        T: Into<Hertz>;
}

pub struct Pwm<TIM, CHANNEL> {
    _channel: PhantomData<CHANNEL>,
    _tim: PhantomData<TIM>,
}

macro_rules! pwm_hal {
    ($($TIMX:ident:
        ($CH:ty, $ccxe:ident, $ccmrx_output:ident, $ocxpe:ident, $ocxm:ident, $ccrx:ident, $ccrx_l:ident, $ccrx_h:ident),)+
    ) => {
        $(
            impl hal::PwmPin for Pwm<$TIMX, $CH> {
                type Duty = u32;

                fn disable(&mut self) {
                    unsafe {
                        (*$TIMX::ptr()).ccer.modify(|_, w| w.$ccxe().clear_bit());
                    }
                }

                fn enable(&mut self) {
                    unsafe {
                        let tim = &*$TIMX::ptr();
                        tim.$ccmrx_output.modify(|_, w| w.$ocxpe().set_bit().$ocxm().bits(6));
                        tim.ccer.modify(|_, w| w.$ccxe().set_bit());
                    }
                }

                fn get_duty(&self) -> u32 {
                    unsafe { (*$TIMX::ptr()).$ccrx.read().bits() }
                }

                fn get_max_duty(&self) -> u32 {
                    unsafe { (*$TIMX::ptr()).arr.read().bits() }
                }

                fn set_duty(&mut self, duty: u32) {
                    unsafe { (*$TIMX::ptr()).$ccrx.write(|w| w.bits(duty)) }
                }
            }
        )+
    };

    ($($TIMX:ident:
        ($CH:ty, $ccxe:ident, $ccmrx_output:ident, $ocxpe:ident, $ocxm:ident, $ccrx:ident $(,$moe:ident)*),)+
    ) => {
        $(
            impl hal::PwmPin for Pwm<$TIMX, $CH> {
                type Duty = u16;

                fn disable(&mut self) {
                    unsafe {
                        (*$TIMX::ptr()).ccer.modify(|_, w| w.$ccxe().clear_bit());
                    }
                }

                fn enable(&mut self) {
                    unsafe {
                        let tim = &*$TIMX::ptr();
                        tim.$ccmrx_output.modify(|_, w| w.$ocxpe().set_bit().$ocxm().bits(6));
                        tim.ccer.modify(|_, w| w.$ccxe().set_bit());
                        $(
                            tim.bdtr.modify(|_, w| w.$moe().set_bit());
                        )*
                    }
                }

                fn get_duty(&self) -> u16 {
                    unsafe { (*$TIMX::ptr()).$ccrx.read().$ccrx().bits() }
                }

                fn get_max_duty(&self) -> u16 {
                    unsafe { (*$TIMX::ptr()).arr.read().arr().bits() }
                }

                fn set_duty(&mut self, duty: u16) {
                    unsafe { (*$TIMX::ptr()).$ccrx.write(|w| w.$ccrx().bits(duty)) }
                }
            }
        )+
    };
}

macro_rules! pwm_pins {
    ($TIMX:ident, [ $(($ch:ty, $pin:ty, $af_mode:expr),)+ ]) => {
        $(
            impl Pins<$TIMX> for $pin {
                type Channel = Pwm<$TIMX, $ch>;

                fn setup(&self) {
                    self.set_alt_mode($af_mode);
                }
            }
        )+
    };
}

macro_rules! pwm {
    ($($TIMX:ident: ($apbXenr:ident, $apbXrstr:ident, $timX:ident, $timXen:ident, $timXrst:ident, $arr:ident $(,$arr_h:ident)*),)+) => {
        $(
            impl PwmExt for $TIMX {
                fn pwm<PINS, T>(self, pins: PINS, freq: T, rcc: &mut Rcc) -> PINS::Channel
                where
                    PINS: Pins<Self>,
                    T: Into<Hertz>,
                {
                    $timX(self, pins, freq.into(), rcc)
                }
            }

            fn $timX<PINS>(tim: $TIMX, pins: PINS, freq: Hertz, rcc: &mut Rcc) -> PINS::Channel
            where
                PINS: Pins<$TIMX>,
            {
                pins.setup();
                rcc.rb.$apbXenr.modify(|_, w| w.$timXen().set_bit());
                rcc.rb.$apbXrstr.modify(|_, w| w.$timXrst().set_bit());
                rcc.rb.$apbXrstr.modify(|_, w| w.$timXrst().clear_bit());
                let reload = rcc.clocks.apb_tim_clk.0 / freq.0;
                let psc = (reload - 1) / 0xffff;
                let arr = reload / (psc + 1);
                tim.psc.write(|w| unsafe { w.psc().bits(psc as u16) });
                tim.arr.write(|w| unsafe { w.$arr().bits(arr as u16) });
                $(
                    tim.arr.modify(|_, w| unsafe { w.$arr_h().bits((arr >> 16) as u16) });
                )*
                tim.cr1.write(|w| w.cen().set_bit());
                unsafe { mem::uninitialized() }
            }
        )+
    }
}

pwm_pins!(TIM1, [
    (C1, PA8<DefaultMode>, AltFunction::AF2),
    (C1, PC8<DefaultMode>, AltFunction::AF2),
    (C2, PA9<DefaultMode>, AltFunction::AF2),
    (C2, PB3<DefaultMode>, AltFunction::AF1),
    (C2, PC9<DefaultMode>, AltFunction::AF2),
    (C3, PA10<DefaultMode>, AltFunction::AF2),
    (C3, PB6<DefaultMode>, AltFunction::AF1),
    (C3, PC10<DefaultMode>, AltFunction::AF2),
    (C4, PA11<DefaultMode>, AltFunction::AF2),
    (C4, PC11<DefaultMode>, AltFunction::AF2),
]);

pwm_pins!(TIM2, [
    (C1, PA0<DefaultMode>, AltFunction::AF2),
    (C1, PA5<DefaultMode>, AltFunction::AF2),
    (C1, PA15<DefaultMode>, AltFunction::AF2),
    (C1, PC4<DefaultMode>, AltFunction::AF2),
    (C2, PA1<DefaultMode>, AltFunction::AF2),
    (C2, PB3<DefaultMode>, AltFunction::AF2),
    (C2, PC5<DefaultMode>, AltFunction::AF2),
    (C3, PA2<DefaultMode>, AltFunction::AF2),
    (C3, PB10<DefaultMode>, AltFunction::AF2),
    (C4, PA3<DefaultMode>, AltFunction::AF2),
    (C4, PB11<DefaultMode>, AltFunction::AF2),
    (C4, PC7<DefaultMode>, AltFunction::AF2),
]);

pwm_pins!(TIM3, [
    (C1, PA6<DefaultMode>, AltFunction::AF1),
    (C1, PB4<DefaultMode>, AltFunction::AF1),
    (C1, PC6<DefaultMode>, AltFunction::AF1),
    (C2, PA7<DefaultMode>, AltFunction::AF1),
    (C2, PB5<DefaultMode>, AltFunction::AF1),
    (C2, PC7<DefaultMode>, AltFunction::AF1),
    (C3, PB0<DefaultMode>, AltFunction::AF1),
    (C3, PC8<DefaultMode>, AltFunction::AF1),
    (C4, PB1<DefaultMode>, AltFunction::AF1),
    (C4, PC9<DefaultMode>, AltFunction::AF1),
]);

pwm_pins!(TIM14, [
    (C1, PA4<DefaultMode>, AltFunction::AF4),
    (C1, PA7<DefaultMode>, AltFunction::AF4),
    (C1, PB1<DefaultMode>, AltFunction::AF0),
    (C1, PC12<DefaultMode>, AltFunction::AF2),
    (C1, PF0<DefaultMode>, AltFunction::AF2),
]);

pwm_pins!(TIM15, [
    (C1, PA2<DefaultMode>, AltFunction::AF5),
    (C1, PB14<DefaultMode>, AltFunction::AF5),
    (C1, PC1<DefaultMode>, AltFunction::AF2),
]);

pwm_pins!(TIM16, [
    (C1, PA6<DefaultMode>, AltFunction::AF5),
    (C1, PB8<DefaultMode>, AltFunction::AF2),
    (C1, PD0<DefaultMode>, AltFunction::AF2),
]);

pwm_pins!(TIM17, [
    (C1, PA7<DefaultMode>, AltFunction::AF6),
    (C1, PB9<DefaultMode>, AltFunction::AF2),
    (C1, PD1<DefaultMode>, AltFunction::AF2),
]);

pwm_hal! {
    TIM1: (C1, cc1e, ccmr1_output, oc1pe, oc1m, ccr1, moe),
    TIM1: (C2, cc2e, ccmr1_output, oc2pe, oc2m, ccr2, moe),
    TIM1: (C3, cc3e, ccmr2_output, oc3pe, oc3m, ccr3, moe),
    TIM1: (C4, cc4e, ccmr2_output, oc4pe, oc4m, ccr4, moe),
    TIM1: (C5, cc5e, ccmr3_output, oc5pe, oc5m, ccr5, moe),
    TIM1: (C6, cc6e, ccmr3_output, oc6pe, oc6m, ccr6, moe),
    TIM14: (C1, cc1e, ccmr1_output, oc1pe, oc1m, ccr1),
    TIM15: (C1, cc1e, ccmr1_output, oc1pe, oc1m, ccr1, moe),
    // TODO(dotcypress): patch SVD
    // TIM15: (C2, cc2e, ccmr1_output, oc2pe, oc2m, ccr2, moe),
    TIM16: (C1, cc1e, ccmr1_output, oc1pe, oc1m, ccr1, moe),
    TIM17: (C1, cc1e, ccmr1_output, oc1pe, oc1m, ccr1, moe),
}

pwm_hal! {
    TIM2: (C1, cc1e, ccmr1_output, oc1pe, oc1m, ccr1, ccr1_l, ccr1_h),
    TIM2: (C2, cc2e, ccmr1_output, oc2pe, oc2m, ccr2, ccr2_l, ccr2_h),
    TIM2: (C3, cc3e, ccmr2_output, oc3pe, oc3m, ccr3, ccr3_l, ccr3_h),
    TIM2: (C4, cc4e, ccmr2_output, oc4pe, oc4m, ccr4, ccr4_l, ccr4_h),
    TIM3: (C1, cc1e, ccmr1_output, oc1pe, oc1m, ccr1, ccr1_l, ccr1_h),
    TIM3: (C2, cc2e, ccmr1_output, oc2pe, oc2m, ccr2, ccr2_l, ccr2_h),
    TIM3: (C3, cc3e, ccmr2_output, oc3pe, oc3m, ccr3, ccr3_l, ccr3_h),
    TIM3: (C4, cc4e, ccmr2_output, oc4pe, oc4m, ccr4, ccr4_l, ccr4_h),
}

pwm! {
    TIM1: (apbenr2, apbrstr2, tim1, tim1en, tim1rst, arr),
    TIM2: (apbenr1, apbrstr1, tim2, tim2en, tim2rst, arr_l, arr_h),
    TIM3: (apbenr1, apbrstr1, tim3, tim3en, tim3rst, arr_l, arr_h),
    TIM14: (apbenr2, apbrstr2, tim14, tim14en, tim14rst, arr),
    TIM15: (apbenr2, apbrstr2, tim15, tim15en, tim15rst, arr),
    TIM16: (apbenr2, apbrstr2, tim16, tim16en, tim16rst, arr),
    TIM17: (apbenr2, apbrstr2, tim17, tim17en, tim17rst, arr),
}
