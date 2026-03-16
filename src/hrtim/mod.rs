pub mod adc_trigger;
pub mod capture;
pub mod external_event;
pub mod fault;

use core::mem::MaybeUninit;

use crate::{
    gpio,
    rcc::{Enable, Reset},
    stm32::{HRTIM_COMMON, HRTIM_TIMA, HRTIM_TIMB, HRTIM_TIMC, HRTIM_TIMD, HRTIM_TIME, HRTIM_TIMF},
};
use stm32_hrtim::{
    control::{HrPwmControl, HrTimOngoingCalibration},
    output::{self, Output1Pin, Output2Pin},
    HrParts, HrPwmBuilder,
};

pub use stm32_hrtim;

pub trait HrControltExt {
    fn hr_control(self, rcc: &mut crate::rcc::Rcc) -> HrTimOngoingCalibration;
}

impl HrControltExt for crate::stm32::HRTIM_COMMON {
    fn hr_control(self, rcc: &mut crate::rcc::Rcc) -> HrTimOngoingCalibration {
        HRTIM_COMMON::enable(rcc);
        HRTIM_COMMON::reset(rcc);

        // TODO: Verify that the HRTIM gets a clock of 100-170MHz as input
        // SAFETY: We have enabled the rcc
        unsafe { HrTimOngoingCalibration::hr_control() }
    }
}

pub trait HrPwmBuilderExt<TIM, PSCL, P1: Output1Pin<TIM>, P2: Output2Pin<TIM>> {
    fn finalize(self, control: &mut HrPwmControl) -> HrParts<TIM, PSCL>;
}
macro_rules! impl_finalize {
    ($($TIMX:ident),+) => {$(
        impl<PSCL: stm32_hrtim::HrtimPrescaler, P1: Out1Pin<$TIMX>, P2: Out2Pin<$TIMX>> HrPwmBuilderExt<$TIMX, PSCL, P1, P2>
            for HrPwmBuilder<$TIMX, PSCL, stm32_hrtim::PreloadSource, P1, P2>
        {
            fn finalize(
                self,
                control: &mut HrPwmControl,
            ) -> HrParts<$TIMX, PSCL> {
                let (pin1, pin2) = self._init(control);
                pin1.connect_to_hrtim();
                pin2.connect_to_hrtim();
                unsafe { MaybeUninit::uninit().assume_init() }
            }
        }
    )+};
}

impl_finalize! {
    HRTIM_TIMA,
    HRTIM_TIMB,
    HRTIM_TIMC,
    HRTIM_TIMD,
    HRTIM_TIME,
    HRTIM_TIMF
}

use gpio::{
    gpioa::{PA10, PA11, PA8, PA9},
    gpioc::PC8,
};

use gpio::{
    gpiob::{PB12, PB13, PB14, PB15},
    gpioc::PC9,
};

use gpio::gpioc::{PC6, PC7};

/// Implemented for types that can be used as output1 for HRTIM timer instances
pub trait Out1Pin<TIM>: Output1Pin<TIM> {
    /// Connect pin to hrtim timer
    fn connect_to_hrtim(self);
}

/// Implemented for types that can be used as output2 for HRTIM timer instances
pub trait Out2Pin<TIM>: Output2Pin<TIM> {
    /// Connect pin to hrtim timer
    fn connect_to_hrtim(self);
}

impl<TIM> Out1Pin<TIM> for output::NoPin {
    fn connect_to_hrtim(self) {}
}

impl<TIM> Out2Pin<TIM> for output::NoPin {
    fn connect_to_hrtim(self) {}
}

macro_rules! pins_helper {
    ($TIMX:ty, $OutYPin:ident, $OutputYPin:ident, $HrOutY:ident, $CHY:ident<$CHY_AF:literal>) => {
        unsafe impl $OutputYPin<$TIMX> for $CHY<gpio::DefaultMode> {}

        impl $OutYPin<$TIMX> for $CHY<gpio::DefaultMode> {
            // Pin<Gpio, Index, Alternate<PushPull, AF>>
            fn connect_to_hrtim(self) {
                let _: $CHY<gpio::Alternate<{ $CHY_AF }>> = self.into_alternate();
            }
        }
    };
}

macro_rules! pins {
    ($($TIMX:ty: CH1: $CH1:ident<$CH1_AF:literal>, CH2: $CH2:ident<$CH2_AF:literal>),+) => {$(
        pins_helper!($TIMX, Out1Pin, Output1Pin, HrOut1, $CH1<$CH1_AF>);
        pins_helper!($TIMX, Out2Pin, Output2Pin, HrOut2, $CH2<$CH2_AF>);
    )+};
}

pins! {
    HRTIM_TIMA: CH1: PA8<13>,  CH2: PA9<13>,
    HRTIM_TIMB: CH1: PA10<13>, CH2: PA11<13>,
    HRTIM_TIMC: CH1: PB12<13>, CH2: PB13<13>,
    HRTIM_TIMD: CH1: PB14<13>, CH2: PB15<13>,
    HRTIM_TIME: CH1: PC8<3>,   CH2: PC9<3>,
    HRTIM_TIMF: CH1: PC6<13>,  CH2: PC7<13>
}
