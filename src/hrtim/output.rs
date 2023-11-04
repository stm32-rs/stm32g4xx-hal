use core::marker::PhantomData;
use stm32g4::stm32g474::{
    HRTIM_MASTER, HRTIM_TIMA, HRTIM_TIMB, HRTIM_TIMC, HRTIM_TIMD, HRTIM_TIME, HRTIM_TIMF,
};
use crate::hrtim::external_event::ExternalEventSource;

use super::event::{EventSource, NeighborTimerEventSource};
use crate::{
    gpio::{
        gpioa::{PA10, PA11, PA8, PA9},
        gpiob::{PB12, PB13, PB14, PB15},
        gpioc::{PC6, PC7, PC8, PC9},
        Alternate, AF13, AF3,
    },
    pwm::{ActiveHigh, ComplementaryImpossible, Pins, Pwm},
    stm32::HRTIM_COMMON,
};

macro_rules! hrtim_out_common {
    ($TIMX:ident, $set_event:expr, $register:ident, $action:ident) => {{
        let tim = unsafe { &*$TIMX::ptr() };

        match $set_event {
            EventSource::Cr1 { .. } => tim.$register.modify(|_r, w| w.cmp1().$action()),
            EventSource::Cr2 { .. } => tim.$register.modify(|_r, w| w.cmp2().$action()),
            EventSource::Cr3 { .. } => tim.$register.modify(|_r, w| w.cmp3().$action()),
            EventSource::Cr4 { .. } => tim.$register.modify(|_r, w| w.cmp4().$action()),
            EventSource::Period { .. } => tim.$register.modify(|_r, w| w.per().$action()),

            EventSource::MasterCr1 { .. } => tim.$register.modify(|_r, w| w.mstcmp1().$action()),
            EventSource::MasterCr2 { .. } => tim.$register.modify(|_r, w| w.mstcmp2().$action()),
            EventSource::MasterCr3 { .. } => tim.$register.modify(|_r, w| w.mstcmp3().$action()),
            EventSource::MasterCr4 { .. } => tim.$register.modify(|_r, w| w.mstcmp4().$action()),
            EventSource::MasterPeriod { .. } => tim.$register.modify(|_r, w| w.mstper().$action()),

            EventSource::ExternalEvent(e) => match e {
                ExternalEventSource::Eevnt1 { .. } => {
                    tim.$register.modify(|_r, w| w.extevnt1().$action())
                },
                ExternalEventSource::Eevnt2 { .. } => {
                    tim.$register.modify(|_r, w| w.extevnt2().$action())
                },
                ExternalEventSource::Eevnt3 { .. } => {
                    tim.$register.modify(|_r, w| w.extevnt3().$action())
                },
                ExternalEventSource::Eevnt4 { .. } => {
                    tim.$register.modify(|_r, w| w.extevnt4().$action())
                },
                ExternalEventSource::Eevnt5 { .. } => {
                    tim.$register.modify(|_r, w| w.extevnt5().$action())
                },
                ExternalEventSource::Eevnt6 { .. } => {
                    tim.$register.modify(|_r, w| w.extevnt6().$action())
                },
                ExternalEventSource::Eevnt7 { .. } => {
                    tim.$register.modify(|_r, w| w.extevnt7().$action())
                },
                ExternalEventSource::Eevnt8 { .. } => {
                    tim.$register.modify(|_r, w| w.extevnt8().$action())
                },
                ExternalEventSource::Eevnt9 { .. } => {
                    tim.$register.modify(|_r, w| w.extevnt9().$action())
                },
                ExternalEventSource::Eevnt10 { .. } => {
                    tim.$register.modify(|_r, w| w.extevnt10().$action())
                },
            }

            EventSource::NeighborTimer { n } => match n {
                NeighborTimerEventSource::TimEvent1 { .. } => {
                    tim.$register.modify(|_r, w| w.timevnt1().$action())
                }
                NeighborTimerEventSource::TimEvent2 { .. } => {
                    tim.$register.modify(|_r, w| w.timevnt2().$action())
                }
                NeighborTimerEventSource::TimEvent3 { .. } => {
                    tim.$register.modify(|_r, w| w.timevnt3().$action())
                }
                NeighborTimerEventSource::TimEvent4 { .. } => {
                    tim.$register.modify(|_r, w| w.timevnt4().$action())
                }
                NeighborTimerEventSource::TimEvent5 { .. } => {
                    tim.$register.modify(|_r, w| w.timevnt5().$action())
                }
                NeighborTimerEventSource::TimEvent6 { .. } => {
                    tim.$register.modify(|_r, w| w.timevnt6().$action())
                }
                NeighborTimerEventSource::TimEvent7 { .. } => {
                    tim.$register.modify(|_r, w| w.timevnt7().$action())
                }
                NeighborTimerEventSource::TimEvent8 { .. } => {
                    tim.$register.modify(|_r, w| w.timevnt8().$action())
                }
                NeighborTimerEventSource::TimEvent9 { .. } => {
                    tim.$register.modify(|_r, w| w.timevnt9().$action())
                }
            },
        }
    }};
}

macro_rules! hrtim_out {
    ($($TIMX:ident: $out_type:ident: $tXYoen:ident, $tXYodis:ident, $tXYods:ident, $setXYr:ident, $rstXYr:ident,)+) => {$(
        impl<PSCL> HrOutput<PSCL, $TIMX> for $out_type<$TIMX, PSCL> {
            fn enable(&mut self) {
                let common = unsafe { &*HRTIM_COMMON::ptr() };
                common.oenr.write(|w| { w.$tXYoen().set_bit() });
            }

            fn disable(&mut self) {
                let common = unsafe { &*HRTIM_COMMON::ptr() };
                common.odisr.write(|w| { w.$tXYodis().set_bit() });
            }

            fn enable_set_event(&mut self, set_event: impl Into<EventSource<PSCL, $TIMX>>) {
                hrtim_out_common!($TIMX, set_event.into(), $setXYr, set_bit)
            }
            fn disable_set_event(&mut self, set_event: impl Into<EventSource<PSCL, $TIMX>>) {
                hrtim_out_common!($TIMX, set_event.into(), $setXYr, clear_bit)
            }

            fn enable_rst_event(&mut self, reset_event: impl Into<EventSource<PSCL, $TIMX>>) {
                hrtim_out_common!($TIMX, reset_event.into(), $rstXYr, set_bit)
            }
            fn disable_rst_event(&mut self, reset_event: impl Into<EventSource<PSCL, $TIMX>>) {
                hrtim_out_common!($TIMX, reset_event.into(), $rstXYr, clear_bit)
            }

            fn get_state(&self) -> State {
                let ods;
                let oen;

                unsafe {
                    let common = &*HRTIM_COMMON::ptr();
                    ods = common.odsr.read().$tXYods().bit_is_set();
                    oen = common.oenr.read().$tXYoen().bit_is_set();
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
    HRTIM_TIMA: HrOut1: ta1oen, ta1odis, ta1ods, seta1r, rsta1r,
    HRTIM_TIMA: HrOut2: ta2oen, ta2odis, ta2ods, seta2r, rsta2r,

    HRTIM_TIMB: HrOut1: tb1oen, tb1odis, tb1ods, setb1r, rstb1r,
    HRTIM_TIMB: HrOut2: tb2oen, tb2odis, tb2ods, setb2r, rstb2r,

    HRTIM_TIMC: HrOut1: tc1oen, tc1odis, tc1ods, setc1r, rstc1r,
    HRTIM_TIMC: HrOut2: tc2oen, tc2odis, tc2ods, setc2r, rstc2r,

    HRTIM_TIMD: HrOut1: td1oen, td1odis, td1ods, setd1r, rstd1r,
    HRTIM_TIMD: HrOut2: td2oen, td2odis, td2ods, setd2r, rstd2r,

    HRTIM_TIME: HrOut1: te1oen, te1odis, te1ods, sete1r, rste1r,
    HRTIM_TIME: HrOut2: te2oen, te2odis, te2ods, sete2r, rste2r,

    HRTIM_TIMF: HrOut1: tf1oen, tf1odis, tf1ods, setf1r, rstf1r,
    HRTIM_TIMF: HrOut2: tf2oen, tf2odis, tf2ods, setf2r, rstf2r,
}

pub trait HrOutput<PSCL, TIM> {
    /// Enable this output
    fn enable(&mut self);

    /// Disable this output
    fn disable(&mut self);

    /// Set this output to active every time the specified event occurs
    ///
    /// NOTE: Enabling the same event for both SET and RESET
    /// will make that event TOGGLE the output
    fn enable_set_event(&mut self, set_event: impl Into<EventSource<PSCL, TIM>>);

    /// Stop listening to the specified event
    fn disable_set_event(&mut self, set_event: impl Into<EventSource<PSCL, TIM>>);

    /// Set this output to *not* active every time the specified event occurs
    ///
    /// NOTE: Enabling the same event for both SET and RESET
    /// will make that event TOGGLE the output
    fn enable_rst_event(&mut self, reset_event: impl Into<EventSource<PSCL, TIM>>);

    /// Stop listening to the specified event
    fn disable_rst_event(&mut self, reset_event: impl Into<EventSource<PSCL, TIM>>);

    /// Get current state of the output
    fn get_state(&self) -> State;
}

#[derive(Debug, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum State {
    Idle,
    Running,
    Fault,
}

pub unsafe trait ToHrOut {
    type Out<PSCL>: ToHrOut;
}

unsafe impl<PA, PB> ToHrOut for (PA, PB)
where
    PA: ToHrOut,
    PB: ToHrOut,
{
    type Out<PSCL> = (PA::Out<PSCL>, PB::Out<PSCL>);
}

pub struct HrOut1<TIM, PSCL>(PhantomData<(TIM, PSCL)>);
pub struct HrOut2<TIM, PSCL>(PhantomData<(TIM, PSCL)>);

macro_rules! pins {
    ($($TIMX:ty: CH1: $CH1:ty, CH2: $CH2:ty)+) => {
        $(
            impl<PSCL> Pins<$TIMX, CH1<PSCL>, ComplementaryImpossible> for $CH1 {
                type Channel = Pwm<$TIMX, CH1<PSCL>, ComplementaryImpossible, ActiveHigh, ActiveHigh>;
            }

            impl<PSCL> Pins<$TIMX, CH2<PSCL>, ComplementaryImpossible> for $CH2 {
                type Channel = Pwm<$TIMX, CH2<PSCL>, ComplementaryImpossible, ActiveHigh, ActiveHigh>;
            }

            unsafe impl ToHrOut for $CH1 {
                type Out<PSCL> = HrOut1<$TIMX, PSCL>;
            }

            unsafe impl ToHrOut for $CH2 {
                type Out<PSCL> = HrOut2<$TIMX, PSCL>;
            }

            unsafe impl<PSCL> ToHrOut for HrOut1<$TIMX, PSCL> {
                type Out<P> = HrOut1<$TIMX, P>;
            }

            unsafe impl<PSCL> ToHrOut for HrOut2<$TIMX, PSCL> {
                type Out<P> = HrOut2<$TIMX, P>;
            }
        )+
    }
}

pins! {
    HRTIM_TIMA: CH1: PA8<Alternate<AF13>>, CH2: PA9<Alternate<AF13>>

    HRTIM_TIMB: CH1: PA10<Alternate<AF13>>, CH2: PA11<Alternate<AF13>>
    HRTIM_TIMC: CH1: PB12<Alternate<AF13>>, CH2: PB13<Alternate<AF13>>
    HRTIM_TIMD: CH1: PB14<Alternate<AF13>>, CH2: PB15<Alternate<AF13>>

    HRTIM_TIME: CH1: PC8<Alternate<AF3>>, CH2: PC9<Alternate<AF3>>
    HRTIM_TIMF: CH1: PC6<Alternate<AF13>>, CH2: PC7<Alternate<AF13>>
}

impl Pins<HRTIM_MASTER, (), ComplementaryImpossible> for () {
    type Channel = ();
}

unsafe impl ToHrOut for () {
    type Out<PSCL> = ();
}

// automatically implement Pins trait for tuples of individual pins
macro_rules! pins_tuples {
    // Tuple of two pins
    ($(($CHA:ident, $CHB:ident)),*) => {
        $(
            impl<TIM, PSCL, CHA, CHB, TA, TB> Pins<TIM, ($CHA<PSCL>, $CHB<PSCL>), (TA, TB)> for (CHA, CHB)
            where
                CHA: Pins<TIM, $CHA<PSCL>, TA>,
                CHB: Pins<TIM, $CHB<PSCL>, TB>,
            {
                type Channel = (Pwm<TIM, $CHA<PSCL>, TA, ActiveHigh, ActiveHigh>, Pwm<TIM, $CHB<PSCL>, TB, ActiveHigh, ActiveHigh>);
            }

            impl<PSCL> HrtimChannel<PSCL> for ($CHA<PSCL>, $CHB<PSCL>) {}
        )*
    };
}

pins_tuples! {
    (CH1, CH2),
    (CH2, CH1)
}

pub struct CH1<PSCL>(PhantomData<PSCL>);
pub struct CH2<PSCL>(PhantomData<PSCL>);

impl<PSCL> HrtimChannel<PSCL> for () {}
pub trait HrtimChannel<PSCL> {}

impl<PSCL> HrtimChannel<PSCL> for CH1<PSCL> {}
impl<PSCL> HrtimChannel<PSCL> for CH2<PSCL> {}
