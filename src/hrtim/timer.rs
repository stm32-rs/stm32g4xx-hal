use crate::stm32::{
    HRTIM_MASTER, HRTIM_TIMA, HRTIM_TIMB, HRTIM_TIMC, HRTIM_TIMD, HRTIM_TIME, HRTIM_TIMF,
};
use core::marker::PhantomData;

use super::{
    capture::{self, HrCapt, HrCapture},
    control::HrPwmCtrl,
    HrtimPrescaler,
};

pub struct HrTim<TIM, PSCL, CPT1, CPT2> {
    _timer: PhantomData<TIM>,
    _prescaler: PhantomData<PSCL>,
    capture_ch1: CPT1,
    capture_ch2: CPT2,
}

/// This is the DMA channel of a HRTIM timer
///
/// Every HRTIM timer including the master timer has a DMA channel
pub struct DmaChannel<TIM> {
    _x: PhantomData<TIM>,
}

pub trait HrTimer {
    type Timer;
    type Prescaler: HrtimPrescaler;

    /// Get period of timer in number of ticks
    ///
    /// This is also the maximum duty usable for `HrCompareRegister::set_duty`
    ///
    /// NOTE: The effective period in number of ticks will be twice as large as
    /// returned by this function when running in UpDown mode or PushPull mode.
    /// 4 times as large when having both modes active
    fn get_period(&self) -> u16;

    /// Set period of timer in number of ticks
    ///
    /// NOTE: This will affect the maximum duty usable for `HrCompareRegister::set_duty`
    fn set_period(&mut self, period: u16);

    /// Start timer
    fn start(&mut self, _hr_control: &mut HrPwmCtrl);

    /// Stop timer
    fn stop(&mut self, _hr_control: &mut HrPwmCtrl);

    /// Stop timer and reset counter
    fn stop_and_reset(&mut self, _hr_control: &mut HrPwmCtrl);

    fn clear_repetition_interrupt(&mut self);

    /// Make a handle to this timers reset/roll-over event to use as adc trigger
    fn as_reset_adc_trigger(&self) -> super::adc_trigger::TimerReset<Self::Timer>;

    /// Make a handle to this timers period event to use as adc trigger
    fn as_period_adc_trigger(&self) -> super::adc_trigger::TimerPeriod<Self::Timer>;

    /// Disable register updates
    ///
    /// Calling this function temporarily disables the transfer from preload to active registers,
    /// whatever the selected update event. This allows to modify several registers.
    /// The regular update event takes place once [`Self::enable_register_updates`] is called.
    fn disable_register_updates(&mut self, _hr_control: &mut HrPwmCtrl);

    /// Enable register updates
    ///
    /// See [`Self::disable_register_updates`].
    ///
    /// NOTE: Register updates are enabled by default, no need to call this
    /// unless [`Self::disable_register_updates`] has been called.
    fn enable_register_updates(&mut self, _hr_control: &mut HrPwmCtrl);
}

pub trait HrSlaveTimer: HrTimer {
    type CptCh1;
    type CptCh2;

    /// Start listening to the specified event
    fn enable_reset_event<E: super::event::TimerResetEventSource<Self::Timer, Self::Prescaler>>(
        &mut self,
        _event: &E,
    );

    /// Stop listening to the specified event
    fn disable_reset_event<E: super::event::TimerResetEventSource<Self::Timer, Self::Prescaler>>(
        &mut self,
        _event: &E,
    );
}

/// Trait for unsplit slave timer which still contains its capture modules
pub trait HrSlaveTimerCpt: HrSlaveTimer {
    type CaptureCh1: HrCapture;
    type CaptureCh2: HrCapture;

    fn capture_ch1(&mut self) -> &mut <Self as HrSlaveTimerCpt>::CaptureCh1;
    fn capture_ch2(&mut self) -> &mut <Self as HrSlaveTimerCpt>::CaptureCh2;
    fn split_capture(
        self,
    ) -> (
        HrTim<Self::Timer, Self::Prescaler, (), ()>,
        HrCapt<Self::Timer, Self::Prescaler, capture::Ch1, capture::NoDma>,
        HrCapt<Self::Timer, Self::Prescaler, capture::Ch2, capture::NoDma>,
    );
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
        $tXudis:ident,
        $(($rstXr:ident))*,
    )+) => {$(
        impl<PSCL: HrtimPrescaler, CPT1, CPT2> HrTimer for HrTim<$TIMX, PSCL, CPT1, CPT2> {
            type Prescaler = PSCL;
            type Timer = $TIMX;

            fn get_period(&self) -> u16 {
                let tim = unsafe { &*$TIMX::ptr() };

                tim.$perXr.read().$perx().bits()
            }
            fn set_period(&mut self, period: u16) {
                let tim = unsafe { &*$TIMX::ptr() };

                tim.$perXr.write(|w| unsafe { w.$perx().bits(period as u16) });
            }

            /// Start timer
            fn start(&mut self, _hr_control: &mut HrPwmCtrl) {
                // Start timer

                // SAFETY: Since we hold _hr_control there is no risk for a race condition
                let master = unsafe { &*HRTIM_MASTER::ptr() };
                master.mcr.modify(|_r, w| { w.$tXcen().set_bit() });
            }

            /// Stop timer
            fn stop(&mut self, _hr_control: &mut HrPwmCtrl) {
                // Stop counter
                // SAFETY: Since we hold _hr_control there is no risk for a race condition
                let master = unsafe { &*HRTIM_MASTER::ptr() };
                master.mcr.modify(|_r, w| { w.$tXcen().set_bit() });
            }

            /// Stop timer and reset counter
            fn stop_and_reset(&mut self, _hr_control: &mut HrPwmCtrl) {
                self.stop(_hr_control);

                // Reset counter
                let tim = unsafe { &*$TIMX::ptr() };
                unsafe { tim.$cntXr.write(|w| w.$cntx().bits(0)); }
            }

            /// Make a handle to this timers reset event to use as adc trigger
            fn as_reset_adc_trigger(&self) -> super::adc_trigger::TimerReset<Self::Timer> {
                super::adc_trigger::TimerReset(PhantomData)
            }

            /// Make a handle to this timers period event to use as adc trigger
            fn as_period_adc_trigger(&self) -> super::adc_trigger::TimerPeriod<Self::Timer> {
                super::adc_trigger::TimerPeriod(PhantomData)
            }

            fn clear_repetition_interrupt(&mut self) {
                let tim = unsafe { &*$TIMX::ptr() };

                tim.$icr.write(|w| w.$repc().set_bit());
            }

            /// Disable register updates
            ///
            /// Calling this function temporarily disables the transfer from preload to active registers,
            /// whatever the selected update event. This allows to modify several registers.
            /// The regular update event takes place once [`Self::enable_register_updates`] is called.
            fn disable_register_updates(&mut self, _hr_control: &mut HrPwmCtrl) {
                use super::HRTIM_COMMON;
                let common = unsafe { &*HRTIM_COMMON::ptr() };
                common.cr1.modify(|_r, w| w.$tXudis().set_bit());
            }

            /// Enable register updates
            ///
            /// See [`Self::disable_register_updates`].
            ///
            /// NOTE: Register updates are enabled by default, no need to call this
            /// unless [`Self::disable_register_updates`] has been called.
            fn enable_register_updates<'a>(&mut self, _hr_control: &mut HrPwmCtrl) {
                use super::HRTIM_COMMON;
                let common = unsafe { &*HRTIM_COMMON::ptr() };
                common.cr1.modify(|_r, w| w.$tXudis().clear_bit());
            }
        }

        impl<PSCL, CPT1, CPT2> HrTim<$TIMX, PSCL, CPT1, CPT2> {
            pub fn set_repetition_counter(&mut self, repetition_counter: u8) {
                let tim = unsafe { &*$TIMX::ptr() };

                unsafe { tim.$rep.write(|w| w.$repx().bits(repetition_counter)); }
            }

            pub fn enable_repetition_interrupt(&mut self, enable: bool) {
                let tim = unsafe { &*$TIMX::ptr() };

                tim.$dier.modify(|_r, w| w.$repie().bit(enable));
            }
        }

        $(
            impl<PSCL: HrtimPrescaler, CPT1, CPT2> HrSlaveTimer for HrTim<$TIMX, PSCL, CPT1, CPT2> {
                type CptCh1 = HrCapt<Self::Timer, Self::Prescaler, capture::Ch1, capture::NoDma>;
                type CptCh2 = HrCapt<Self::Timer, Self::Prescaler, capture::Ch2, capture::NoDma>;

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
                fn enable_reset_event<E: super::event::TimerResetEventSource<Self::Timer, Self::Prescaler>>(&mut self, _event: &E) {
                    let tim = unsafe { &*$TIMX::ptr() };

                    unsafe { tim.$rstXr.modify(|r, w| w.bits(r.bits() | E::BITS)); }
                }

                /// Stop listening to the specified event
                fn disable_reset_event<E: super::event::TimerResetEventSource<Self::Timer, Self::Prescaler>>(&mut self, _event: &E) {
                    let tim = unsafe { &*$TIMX::ptr() };

                    unsafe { tim.$rstXr.modify(|r, w| w.bits(r.bits() & !E::BITS)); }
                }
            }

            impl<PSCL: HrtimPrescaler> HrSlaveTimerCpt for HrTim<$TIMX, PSCL, HrCapt<$TIMX, PSCL, capture::Ch1, capture::NoDma>, HrCapt<$TIMX, PSCL, capture::Ch2, capture::NoDma>> {
                type CaptureCh1 = <Self as HrSlaveTimer>::CptCh1;
                type CaptureCh2 = <Self as HrSlaveTimer>::CptCh2;

                /// Access the timers first capture channel
                fn capture_ch1(&mut self) -> &mut Self::CaptureCh1 {
                    &mut self.capture_ch1
                }

                /// Access the timers second capture channel
                fn capture_ch2(&mut self) -> &mut Self::CaptureCh2 {
                    &mut self.capture_ch2
                }

                fn split_capture(self) -> (HrTim<$TIMX, PSCL, (), ()>, HrCapt<$TIMX, PSCL, capture::Ch1, capture::NoDma>, HrCapt<$TIMX, PSCL, capture::Ch2, capture::NoDma>) {
                    let HrTim{
                        _timer,
                        _prescaler,
                        capture_ch1,
                        capture_ch2,
                    } = self;

                    (
                        HrTim{
                            _timer,
                            _prescaler,
                            capture_ch1: (),
                            capture_ch2: (),
                        },
                        capture_ch1,
                        capture_ch2,
                    )
                }
            }

            /// Timer Period event
            impl<DST, PSCL, CPT1, CPT2> super::event::EventSource<DST, PSCL> for HrTim<$TIMX, PSCL, CPT1, CPT2> {
                // $rstXr
                const BITS: u32 = 1 << 2;
            }

            /// Timer Update event
            ///
            /// TODO: What dows this mean?
            impl<PSCL, CPT1, CPT2> super::capture::CaptureEvent<$TIMX, PSCL> for HrTim<$TIMX, PSCL, CPT1, CPT2> {
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
            $(impl $AdcTrigger for super::adc_trigger::TimerReset<$TIMX> {
                const BITS: u32 = $adc_trigger_bits_reset;
            })*

            $(impl $AdcTrigger for super::adc_trigger::TimerPeriod<$TIMX> {
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
    HRTIM_MASTER: mcntr, mcnt, mper, mcen, mper, mrep, mrep, mdier, mrepie, micr, mrepc, mudis,,

    HRTIM_TIMA: cntar, cntx, perar, tacen, perx, repar, repx, timadier, repie, timaicr, repc, taudis, (rstar),
    HRTIM_TIMB: cntr, cntx, perbr, tbcen, perx, repbr, repx, timbdier, repie, timbicr, repc, tbudis, (rstbr),
    HRTIM_TIMC: cntcr, cntx, percr, tccen, perx, repcr, repx, timcdier, repie, timcicr, repc, tcudis, (rstcr),
    HRTIM_TIMD: cntdr, cntx, perdr, tdcen, perx, repdr, repx, timddier, repie, timdicr, repc, tdudis, (rstdr),
    HRTIM_TIME: cnter, cntx, perer, tecen, perx, reper, repx, timedier, repie, timeicr, repc, teudis, (rster),
    HRTIM_TIMF: cntfr, cntx, perfr, tfcen, perx, repfr, repx, timfdier, repie, timficr, repc, tfudis, (rstfr),
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

/// Master Timer Period event
impl<DST, PSCL, CPT1, CPT2> super::event::TimerResetEventSource<DST, PSCL>
    for HrTim<HRTIM_MASTER, PSCL, CPT1, CPT2>
{
    const BITS: u32 = 1 << 4; // MSTPER
}

/// Master Timer Period event
impl<DST, PSCL, CPT1, CPT2> super::event::EventSource<DST, PSCL>
    for HrTim<HRTIM_MASTER, PSCL, CPT1, CPT2>
{
    const BITS: u32 = 1 << 7; // MSTPER
}
