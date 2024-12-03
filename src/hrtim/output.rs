use crate::stm32::{HRTIM_TIMA, HRTIM_TIMB, HRTIM_TIMC, HRTIM_TIMD, HRTIM_TIME, HRTIM_TIMF};
use core::marker::PhantomData;

use super::event::EventSource;
use crate::{
    gpio::{
        self,
        gpioa::{PA10, PA11, PA8, PA9},
        gpiob::{PB12, PB13, PB14, PB15},
        gpioc::{PC6, PC7, PC8, PC9},
        Alternate, AF13, AF3,
    },
    pwm::{ComplementaryImpossible, Pins},
    stm32::HRTIM_COMMON,
};

mod sealed {
    pub trait Sealed<T> {}
}

macro_rules! hrtim_out {
    ($($TIMX:ident: $out_type:ident: $tXYoen:ident, $tXYodis:ident, $tXYods:ident, $setXYr:ident, $rstXYr:ident,)+) => {$(
        impl<PSCL> HrOutput<$TIMX, PSCL> for $out_type<$TIMX, PSCL> {
            fn enable(&mut self) {
                let common = unsafe { &*HRTIM_COMMON::ptr() };
                common.oenr().write(|w| { w.$tXYoen().set_bit() });
            }

            fn disable(&mut self) {
                let common = unsafe { &*HRTIM_COMMON::ptr() };
                common.odisr().write(|w| { w.$tXYodis().set_bit() });
            }

            fn enable_set_event<ES: EventSource<$TIMX, PSCL>>(&mut self, _set_event: &ES) {
                let tim = unsafe { &*$TIMX::ptr() };
                unsafe { tim.$setXYr().modify(|r, w| w.bits(r.bits() | ES::BITS)); }
            }
            fn disable_set_event<ES: EventSource<$TIMX, PSCL>>(&mut self, _set_event: &ES) {
                let tim = unsafe { &*$TIMX::ptr() };
                unsafe { tim.$setXYr().modify(|r, w| w.bits(r.bits() & !ES::BITS)); }
            }

            fn enable_rst_event<ES: EventSource<$TIMX, PSCL>>(&mut self, _reset_event: &ES) {
                let tim = unsafe { &*$TIMX::ptr() };
                unsafe { tim.$rstXYr().modify(|r, w| w.bits(r.bits() | ES::BITS)); }
            }
            fn disable_rst_event<ES: EventSource<$TIMX, PSCL>>(&mut self, _reset_event: &ES) {
                let tim = unsafe { &*$TIMX::ptr() };
                unsafe { tim.$rstXYr().modify(|r, w| w.bits(r.bits() & !ES::BITS)); }
            }

            fn get_state(&self) -> State {
                let ods;
                let oen;

                unsafe {
                    let common = &*HRTIM_COMMON::ptr();
                    ods = common.odsr().read().$tXYods().bit_is_set();
                    oen = common.oenr().read().$tXYoen().bit_is_set();
                }

                match (oen, ods) {
                    (true, _) => State::Running,
                    (false, false) => State::Idle,
                    (false, true) => State::Fault
                }
            }
        }
    )+};
}

hrtim_out! {
    HRTIM_TIMA: HrOut1: ta1oen, ta1odis, ta1ods, set1r, rst1r,
    HRTIM_TIMA: HrOut2: ta2oen, ta2odis, ta2ods, set2r, rst2r,

    HRTIM_TIMB: HrOut1: tb1oen, tb1odis, tb1ods, set1r, rst1r,
    HRTIM_TIMB: HrOut2: tb2oen, tb2odis, tb2ods, set2r, rst2r,

    HRTIM_TIMC: HrOut1: tc1oen, tc1odis, tc1ods, set1r, rst1r,
    HRTIM_TIMC: HrOut2: tc2oen, tc2odis, tc2ods, set2r, rst2r,

    HRTIM_TIMD: HrOut1: td1oen, td1odis, td1ods, set1r, rst1r,
    HRTIM_TIMD: HrOut2: td2oen, td2odis, td2ods, set2r, rst2r,

    HRTIM_TIME: HrOut1: te1oen, te1odis, te1ods, set1r, rst1r,
    HRTIM_TIME: HrOut2: te2oen, te2odis, te2ods, set2r, rst2r,

    HRTIM_TIMF: HrOut1: tf1oen, tf1odis, tf1ods, set1r, rst1r,
    HRTIM_TIMF: HrOut2: tf2oen, tf2odis, tf2ods, set2r, rst2r,
}

pub trait HrOutput<TIM, PSCL> {
    /// Enable this output
    fn enable(&mut self);

    /// Disable this output
    fn disable(&mut self);

    /// Set this output to active every time the specified event occurs
    ///
    /// NOTE: Enabling the same event for both SET and RESET
    /// will make that event TOGGLE the output
    fn enable_set_event<ES: EventSource<TIM, PSCL>>(&mut self, set_event: &ES);

    /// Stop listening to the specified event
    fn disable_set_event<ES: EventSource<TIM, PSCL>>(&mut self, set_event: &ES);

    /// Set this output to *not* active every time the specified event occurs
    ///
    /// NOTE: Enabling the same event for both SET and RESET
    /// will make that event TOGGLE the output
    fn enable_rst_event<ES: EventSource<TIM, PSCL>>(&mut self, reset_event: &ES);

    /// Stop listening to the specified event
    fn disable_rst_event<ES: EventSource<TIM, PSCL>>(&mut self, reset_event: &ES);

    /// Get current state of the output
    fn get_state(&self) -> State;
}

#[derive(Debug, PartialEq, Copy, Clone)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum State {
    Idle,
    Running,
    Fault,
}

impl State {
    pub fn is_idle(self) -> bool {
        matches!(self, State::Idle)
    }

    pub fn is_running(self) -> bool {
        matches!(self, State::Running)
    }

    pub fn is_fault(self) -> bool {
        matches!(self, State::Fault)
    }
}

pub trait ToHrOut<TIM>: sealed::Sealed<TIM> {
    type Out<PSCL>;

    fn connect_to_hrtim(self);
}

impl<TIM, PA, PB> sealed::Sealed<TIM> for (PA, PB)
where
    PA: ToHrOut<TIM>,
    PB: ToHrOut<TIM>,
{
}

impl<TIM, PA, PB> ToHrOut<TIM> for (PA, PB)
where
    PA: ToHrOut<TIM>,
    PB: ToHrOut<TIM>,
{
    type Out<PSCL> = (PA::Out<PSCL>, PB::Out<PSCL>);

    fn connect_to_hrtim(self) {
        self.0.connect_to_hrtim();
        self.1.connect_to_hrtim();
    }
}

pub struct HrOut1<TIM, PSCL>(PhantomData<(TIM, PSCL)>);
pub struct HrOut2<TIM, PSCL>(PhantomData<(TIM, PSCL)>);

macro_rules! pins {
    ($($TIMX:ty: CH1: $CH1:ident<$CH1_AF:ident>, CH2: $CH2:ident<$CH2_AF:ident>)+) => {
        $(
            impl sealed::Sealed<$TIMX> for $CH1<gpio::Input<gpio::Floating>> {}
            impl sealed::Sealed<$TIMX> for $CH2<gpio::Input<gpio::Floating>> {}

            impl ToHrOut<$TIMX> for $CH1<gpio::Input<gpio::Floating>> {
                type Out<PSCL> = HrOut1<$TIMX, PSCL>;

                fn connect_to_hrtim(self) {
                    let _: $CH1<Alternate<$CH1_AF>> = self.into_alternate();
                }
            }

            impl ToHrOut<$TIMX> for $CH2<gpio::Input<gpio::Floating>> {
                type Out<PSCL> = HrOut2<$TIMX, PSCL>;

                fn connect_to_hrtim(self) {
                    let _: $CH2<Alternate<$CH2_AF>> = self.into_alternate();
                }
            }
        )+
    }
}

pins! {
    HRTIM_TIMA: CH1: PA8<AF13>, CH2: PA9<AF13>

    HRTIM_TIMB: CH1: PA10<AF13>, CH2: PA11<AF13>
    HRTIM_TIMC: CH1: PB12<AF13>, CH2: PB13<AF13>
    HRTIM_TIMD: CH1: PB14<AF13>, CH2: PB15<AF13>

    HRTIM_TIME: CH1: PC8<AF3>, CH2: PC9<AF3>
    HRTIM_TIMF: CH1: PC6<AF13>, CH2: PC7<AF13>
}

impl<T> Pins<T, (), ComplementaryImpossible> for () {
    type Channel = ();
}

impl<T> sealed::Sealed<T> for () {}
impl<T> ToHrOut<T> for () {
    type Out<PSCL> = ();

    fn connect_to_hrtim(self) {}
}

pub struct CH1<PSCL>(PhantomData<PSCL>);
pub struct CH2<PSCL>(PhantomData<PSCL>);
