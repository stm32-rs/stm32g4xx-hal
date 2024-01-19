use crate::stm32::{
    HRTIM_MASTER, HRTIM_TIMA, HRTIM_TIMB, HRTIM_TIMC, HRTIM_TIMD, HRTIM_TIME, HRTIM_TIMF,
};
use core::marker::PhantomData;

use super::{
    capture::{self, HrCapt},
    control::HrPwmControl,
};

pub struct HrTim<TIM, PSCL> {
    _timer: PhantomData<TIM>,
    _prescaler: PhantomData<PSCL>,
    capture_ch1: HrCapt<TIM, PSCL, capture::Ch1>,
    capture_ch2: HrCapt<TIM, PSCL, capture::Ch2>,
}

pub trait HrTimer<TIM, PSCL>: Sized {
    /// Get period of timer in number of ticks
    ///
    /// This is also the maximum duty usable for `HrCompareRegister::set_duty`
    fn get_period(&self) -> u16;

    /// Set period of timer in number of ticks
    ///
    /// NOTE: This will affect the maximum duty usable for `HrCompareRegister::set_duty`
    fn set_period(&mut self, period: u16);

    /// Start timer
    fn start(&mut self, _hr_control: &mut HrPwmControl);

    /// Stop timer
    fn stop(&mut self, _hr_control: &mut HrPwmControl);

    /// Stop timer and reset counter
    fn stop_and_reset(&mut self, _hr_control: &mut HrPwmControl);

    fn clear_repetition_interrupt(&mut self);

    /// Make a handle to this timers reset event to use as adc trigger
    fn as_reset_adc_trigger(&self) -> super::adc_trigger::TimerReset<Self>;

    /// Make a handle to this timers period event to use as adc trigger
    fn as_period_adc_trigger(&self) -> super::adc_trigger::TimerPeriod<Self>;
}

macro_rules! hrtim_timer {
    ($(
        $TIMX:ident:
        $cntXr:ident,
        $cntx:ident,
        $perXr:ident,
        $tXcen:ident,
        $perx:ident,
        $rep:ident,
        $repx:ident,
        $dier:ident,
        $repie:ident,
        $icr:ident,
        $repc:ident,
        $(($rstXr:ident))*,
    )+) => {$(
        impl<PSCL> HrTimer<$TIMX, PSCL> for HrTim<$TIMX, PSCL> {
            fn get_period(&self) -> u16 {
                let tim = unsafe { &*$TIMX::ptr() };

                tim.$perXr.read().$perx().bits()
            }
            fn set_period(&mut self, period: u16) {
                let tim = unsafe { &*$TIMX::ptr() };

                tim.$perXr.write(|w| unsafe { w.$perx().bits(period as u16) });
            }

            /// Start timer
            fn start(&mut self, _hr_control: &mut HrPwmControl) {
                // Start timer

                // SAFETY: Since we hold _hr_control there is no risk for a race condition
                let master = unsafe { &*HRTIM_MASTER::ptr() };
                master.mcr.modify(|_r, w| { w.$tXcen().set_bit() });
            }

            /// Stop timer
            fn stop(&mut self, _hr_control: &mut HrPwmControl) {
                // Stop counter
                // SAFETY: Since we hold _hr_control there is no risk for a race condition
                let master = unsafe { &*HRTIM_MASTER::ptr() };
                master.mcr.modify(|_r, w| { w.$tXcen().set_bit() });
            }

            /// Stop timer and reset counter
            fn stop_and_reset(&mut self, _hr_control: &mut HrPwmControl) {
                self.stop(_hr_control);

                // Reset counter
                let tim = unsafe { &*$TIMX::ptr() };
                unsafe { tim.$cntXr.write(|w| w.$cntx().bits(0)); }
            }

            /// Make a handle to this timers reset event to use as adc trigger
            fn as_reset_adc_trigger(&self) -> super::adc_trigger::TimerReset<Self> {
                super::adc_trigger::TimerReset(PhantomData)
            }

            /// Make a handle to this timers period event to use as adc trigger
            fn as_period_adc_trigger(&self) -> super::adc_trigger::TimerPeriod<Self> {
                super::adc_trigger::TimerPeriod(PhantomData)
            }

            fn clear_repetition_interrupt(&mut self) {
                let tim = unsafe { &*$TIMX::ptr() };

                tim.$icr.write(|w| w.$repc().set_bit());
            }
        }

        impl<PSCL> HrTim<$TIMX, PSCL> {
            pub fn set_repetition_counter(&mut self, repetition_counter: u8) {
                let tim = unsafe { &*$TIMX::ptr() };

                unsafe { tim.$rep.write(|w| w.$repx().bits(repetition_counter)); }
            }

            pub fn enable_repetition_interrupt(&mut self, enable: bool) {
                let tim = unsafe { &*$TIMX::ptr() };

                tim.$dier.modify(|_r, w| w.$repie().bit(enable));
            }

            pub fn capture_ch1(&mut self) -> &mut HrCapt<$TIMX, PSCL, capture::Ch1> {
                &mut self.capture_ch1
            }

            pub fn capture_ch2(&mut self) -> &mut HrCapt<$TIMX, PSCL, capture::Ch2> {
                &mut self.capture_ch2
            }
        }

        $(// Only for Non-Master timers
            impl<PSCL> HrTim<$TIMX, PSCL> {
                /// Reset this timer every time the specified event occurs
                ///
                /// Behaviour depends on `timer_mode`:
                ///
                /// * `HrTimerMode::SingleShotNonRetriggable`: Enabling the timer enables it but does not start it.
                ///   A first reset event starts the counting and any subsequent reset is ignored until the counter
                ///   reaches the PER value. The PER event is then generated and the counter is stopped. A reset event
                ///   restarts the counting from 0x0000.
                /// * `HrTimerMode:SingleShotRetriggable`: Enabling the timer enables it but does not start it.
                ///   A reset event starts the counting if the counter is stopped, otherwise it clears the counter.
                ///   When the counter reaches the PER value, the PER event is generated and the counter is stopped.
                ///   A reset event restarts the counting from 0x0000.
                /// * `HrTimerMode::Continuous`: Enabling the timer enables and starts it simultaneously.
                ///   When the counter reaches the PER value, it rolls-over to 0x0000 and resumes counting.
                ///   The counter can be reset at any time
                pub fn enable_reset_event<E: super::event::TimerResetEventSource<$TIMX, PSCL>>(&mut self, _event: E) {
                    let tim = unsafe { &*$TIMX::ptr() };

                    unsafe { tim.$rstXr.modify(|r, w| w.bits(r.bits() | E::BITS)); }
                }

                /// Stop listening to the specified event
                pub fn disable_reset_event<E: super::event::TimerResetEventSource<$TIMX, PSCL>>(&mut self, _event: E) {
                    let tim = unsafe { &*$TIMX::ptr() };

                    unsafe { tim.$rstXr.modify(|r, w| w.bits(r.bits() & !E::BITS)); }
                }
            }

            /// Timer Period event
            impl<DST, PSCL> super::event::EventSource<DST, PSCL> for HrTim<$TIMX, PSCL> {
                // $rstXr
                const BITS: u32 = 1 << 2;
            }

            /// Timer Update event
            ///
            /// TODO: What dows this mean?
            impl<PSCL> super::capture::CaptureEvent<$TIMX, PSCL> for HrTim<$TIMX, PSCL> {
                const BITS: u32 = 1 << 1;
            }
        )*
    )+}
}

macro_rules! hrtim_timer_adc_trigger {
    ($($TIMX:ident:
        [$(($AdcTrigger:ident: [
            $((PER: $adc_trigger_bits_period:expr),)*
            $((RST: $adc_trigger_bits_reset:expr)),*
        ])),+]
    ),+) => {
        $($(
            $(impl<PSCL> $AdcTrigger for super::adc_trigger::TimerReset<HrTim<$TIMX, PSCL>> {
                const BITS: u32 = $adc_trigger_bits_reset;
            })*

            $(impl<PSCL> $AdcTrigger for super::adc_trigger::TimerPeriod<HrTim<$TIMX, PSCL>> {
                const BITS: u32 = $adc_trigger_bits_period;
            })*
        )*)*
    }
}

use super::adc_trigger::Adc13Trigger as Adc13;
use super::adc_trigger::Adc24Trigger as Adc24;
use super::adc_trigger::Adc579Trigger as Adc579;
use super::adc_trigger::Adc6810Trigger as Adc6810;

hrtim_timer! {
    HRTIM_MASTER: mcntr, mcnt, mper, mcen, mper, mrep, mrep, mdier, mrepie, micr, mrepc,,

    HRTIM_TIMA: cntar, cntx, perar, tacen, perx, repar, repx, timadier, repie, timaicr, repc, (rstar),
    HRTIM_TIMB: cntr, cntx, perbr, tbcen, perx, repbr, repx, timbdier, repie, timbicr, repc, (rstbr),
    HRTIM_TIMC: cntcr, cntx, percr, tccen, perx, repcr, repx, timcdier, repie, timcicr, repc, (rstcr),
    HRTIM_TIMD: cntdr, cntx, perdr, tdcen, perx, repdr, repx, timddier, repie, timdicr, repc, (rstdr),
    HRTIM_TIME: cnter, cntx, perer, tecen, perx, reper, repx, timedier, repie, timeicr, repc, (rster),
    HRTIM_TIMF: cntfr, cntx, perfr, tfcen, perx, repfr, repx, timfdier, repie, timficr, repc, (rstfr),
}

hrtim_timer_adc_trigger! {
    HRTIM_MASTER: [(Adc13: [(PER: 1 << 4),]), (Adc24: [(PER: 1 << 4),]), (Adc579: [(PER: 4),]), (Adc6810: [(PER: 4),])],

    HRTIM_TIMA: [(Adc13: [(PER: 1 << 13), (RST: 1 << 14)]), (Adc24: [(PER: 1 << 13),               ]), (Adc579: [(PER: 12), (RST: 13)]), (Adc6810: [(PER: 12),          ])],
    HRTIM_TIMB: [(Adc13: [(PER: 1 << 18), (RST: 1 << 19)]), (Adc24: [(PER: 1 << 17),               ]), (Adc579: [(PER: 16), (RST: 17)]), (Adc6810: [(PER: 15),          ])],
    HRTIM_TIMC: [(Adc13: [(PER: 1 << 23),               ]), (Adc24: [(PER: 1 << 21), (RST: 1 << 22)]), (Adc579: [(PER: 20),          ]), (Adc6810: [(PER: 18), (RST: 19)])],
    HRTIM_TIMD: [(Adc13: [(PER: 1 << 27),               ]), (Adc24: [(PER: 1 << 26), (RST: 1 << 27)]), (Adc579: [(PER: 23),          ]), (Adc6810: [(PER: 22), (RST: 23)])],
    HRTIM_TIME: [(Adc13: [(PER: 1 << 31),               ]), (Adc24: [                (RST: 1 << 31)]), (Adc579: [(PER: 26),          ]), (Adc6810: [                    ])],
    HRTIM_TIMF: [(Adc13: [(PER: 1 << 24), (RST: 1 << 28)]), (Adc24: [(PER: 1 << 24),               ]), (Adc579: [(PER: 30), (RST: 31)]), (Adc6810: [(PER: 31),          ])]
}

impl<DST, PSCL> super::event::TimerResetEventSource<DST, PSCL> for HrTim<HRTIM_MASTER, PSCL> {
    const BITS: u32 = 1 << 4; // MSTPER
}
