use core::marker::PhantomData;
use stm32g4::stm32g474::{
    HRTIM_MASTER, HRTIM_TIMA, HRTIM_TIMB, HRTIM_TIMC, HRTIM_TIMD, HRTIM_TIME, HRTIM_TIMF,
};

use super::{
    control::HrPwmControl,
    event::{
        TimerAResetEventSource, TimerBResetEventSource, TimerCResetEventSource,
        TimerDResetEventSource, TimerEResetEventSource, TimerFResetEventSource,
    },
};

pub struct HrTim<TIM, PSCL> {
    _timer: PhantomData<TIM>,
    _prescaler: PhantomData<PSCL>,
}

pub trait HrTimer<TIM, PSCL> {
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
}

macro_rules! hrtim_timer {
    ($($TIMX:ident: $cntXr:ident, $cntx:ident, $perXr:ident, $tXcen:ident, $perx:ident, $rep:ident, $repx:ident, $dier:ident, $repie:ident, $icr:ident, $repc:ident, $([$rstXr:ident, $TimerXResetEventSource:ident],)*)+) => {$(
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
        }

        impl<PSCL> HrTim<$TIMX, PSCL> {
            $(
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
            pub fn enable_reset_event(&mut self, set_event: $TimerXResetEventSource) {
                let tim = unsafe { &*$TIMX::ptr() };

                unsafe { tim.$rstXr.modify(|r, w| w.bits(r.bits() | set_event as u32)); }
            }

            /// Stop listening to the specified event
            pub fn disable_reset_event(&mut self, set_event: $TimerXResetEventSource) {
                let tim = unsafe { &*$TIMX::ptr() };

                unsafe { tim.$rstXr.modify(|r, w| w.bits(r.bits() & !(set_event as u32))); }
            })*

            pub fn set_repetition_counter(&mut self, repetition_counter: u8) {
                let tim = unsafe { &*$TIMX::ptr() };

                unsafe { tim.$rep.write(|w| w.$repx().bits(repetition_counter)); }
            }

            pub fn enable_repetition_interrupt(&mut self, enable: bool) {
                let tim = unsafe { &*$TIMX::ptr() };

                tim.$dier.modify(|_r, w| w.$repie().bit(enable));
            }

            pub fn clear_repetition_interrupt(&mut self) {
                let tim = unsafe { &*$TIMX::ptr() };

                tim.$icr.write(|w| w.$repc().set_bit());
            }
        }
    )+};
}

hrtim_timer! {
    HRTIM_MASTER: mcntr, mcnt, mper, mcen, mper, mrep, mrep, mdier, mrepie, micr, mrepc,

    HRTIM_TIMA: cntar, cntx, perar, tacen, perx, repar, repx, timadier, repie, timaicr, repc, [rstar, TimerAResetEventSource],
    HRTIM_TIMB: cntr, cntx, perbr, tbcen, perx, repbr, repx, timbdier, repie, timbicr, repc, [rstbr, TimerBResetEventSource],
    HRTIM_TIMC: cntcr, cntx, percr, tccen, perx, repcr, repx, timcdier, repie, timcicr, repc, [rstcr, TimerCResetEventSource],
    HRTIM_TIMD: cntdr, cntx, perdr, tdcen, perx, repdr, repx, timddier, repie, timdicr, repc, [rstdr, TimerDResetEventSource],
    HRTIM_TIME: cnter, cntx, perer, tecen, perx, reper, repx, timedier, repie, timeicr, repc, [rster, TimerEResetEventSource],
    HRTIM_TIMF: cntfr, cntx, perfr, tfcen, perx, repfr, repx, timfdier, repie, timficr, repc, [rstfr, TimerFResetEventSource],
}
