use core::marker::PhantomData;

use crate::stm32::{
    HRTIM_MASTER, HRTIM_TIMA, HRTIM_TIMB, HRTIM_TIMC, HRTIM_TIMD, HRTIM_TIME, HRTIM_TIMF,
};

pub trait HrCompareRegister {
    fn get_duty(&self) -> u16;
    fn set_duty(&mut self, duty: u16);
}

pub struct HrCr1<TIM, PSCL>(PhantomData<(TIM, PSCL)>);
pub struct HrCr2<TIM, PSCL>(PhantomData<(TIM, PSCL)>);
pub struct HrCr3<TIM, PSCL>(PhantomData<(TIM, PSCL)>);
pub struct HrCr4<TIM, PSCL>(PhantomData<(TIM, PSCL)>);

use super::adc_trigger::Adc13Trigger as Adc13;
use super::adc_trigger::Adc24Trigger as Adc24;
use super::adc_trigger::Adc579Trigger as Adc579;
use super::adc_trigger::Adc6810Trigger as Adc6810;

macro_rules! hrtim_cr_helper {
    (HRTIM_MASTER: $cr_type:ident:
        $cmpXYr:ident,
        [$(($Trigger:ty: $trigger_bits:expr)),*],
        [$(($event_dst:ident, $tim_event_index:expr)),*],
        $bit_index:literal
    ) => {
        // Strip bit_index since master timer has other bits that are common across all destinations
        hrtim_cr_helper!(HRTIM_MASTER: $cr_type: $cmpXYr, [$(($Trigger: $trigger_bits)),*], [$(($event_dst, $tim_event_index)),*]);
    };

    ($TIMX:ident: $cr_type:ident:
        $cmpXYr:ident,
        [$(($Trigger:ty: $trigger_bits:expr)),*],
        [$(($event_dst:ident, $tim_event_index:expr)),*]
        $(, $bit_index:literal)*
    ) => {
        impl<PSCL> HrCompareRegister for $cr_type<$TIMX, PSCL> {
            fn get_duty(&self) -> u16 {
                let tim = unsafe { &*$TIMX::ptr() };

                tim.$cmpXYr().read().cmp().bits()
            }
            fn set_duty(&mut self, duty: u16) {
                let tim = unsafe { &*$TIMX::ptr() };

                tim.$cmpXYr().write(|w| unsafe { w.cmp().bits(duty) });
            }
        }

        $(
            /// Compare match event
            impl<PSCL> super::event::EventSource<$TIMX, PSCL> for $cr_type<$TIMX, PSCL> {
                const BITS: u32 = 1 << $bit_index;
            }
        )*

        $(
            /// Compare match event for neighbor timer
            impl<PSCL> super::event::EventSource<$event_dst, PSCL> for $cr_type<$TIMX, PSCL> {
                const BITS: u32 = 1 << ($tim_event_index + 11); // TIMEVNT1 is at bit 12, TIMEVNT2 at bit 13 etc
            }
        )*

        $(
            impl<PSCL> $Trigger for $cr_type<$TIMX, PSCL> {
                const BITS: u32 = $trigger_bits;
            }
        )*
    };
}

macro_rules! hrtim_cr {
    ($($TIMX:ident: [
        [$(($cr1_trigger:ident: $cr1_trigger_bits:expr)),*], [$(($cr1_event_dst:ident, $cr1_tim_event_index:expr)),*],
        [$(($cr2_trigger:ident: $cr2_trigger_bits:expr)),*], [$(($cr2_event_dst:ident, $cr2_tim_event_index:expr)),*],
        [$(($cr3_trigger:ident: $cr3_trigger_bits:expr)),*], [$(($cr3_event_dst:ident, $cr3_tim_event_index:expr)),*],
        [$(($cr4_trigger:ident: $cr4_trigger_bits:expr)),*], [$(($cr4_event_dst:ident, $cr4_tim_event_index:expr)),*]
    ]),+) => {$(
        hrtim_cr_helper!($TIMX: HrCr1: cmp1r, [$(($cr1_trigger: $cr1_trigger_bits)),*], [$(($cr1_event_dst, $cr1_tim_event_index)),*], 3);
        hrtim_cr_helper!($TIMX: HrCr2: cmp2r, [$(($cr2_trigger: $cr2_trigger_bits)),*], [$(($cr2_event_dst, $cr2_tim_event_index)),*], 4);
        hrtim_cr_helper!($TIMX: HrCr3: cmp3r, [$(($cr3_trigger: $cr3_trigger_bits)),*], [$(($cr3_event_dst, $cr3_tim_event_index)),*], 5);
        hrtim_cr_helper!($TIMX: HrCr4: cmp4r, [$(($cr4_trigger: $cr4_trigger_bits)),*], [$(($cr4_event_dst, $cr4_tim_event_index)),*], 6);
    )+};
}

// See RM0440 Table 218. 'Events mapping across timer A to F'
hrtim_cr! {
    HRTIM_MASTER: [
        [(Adc13: 1 << 0),  (Adc24: 1 << 0),  (Adc579: 0),  (Adc6810: 0) ], [],
        [(Adc13: 1 << 1),  (Adc24: 1 << 1),  (Adc579: 1),  (Adc6810: 1) ], [],
        [(Adc13: 1 << 2),  (Adc24: 1 << 2),  (Adc579: 2),  (Adc6810: 2) ], [],
        [(Adc13: 1 << 3),  (Adc24: 1 << 3),  (Adc579: 3),  (Adc6810: 3) ], []
    ],

    HRTIM_TIMA: [
        [                                                               ], [(HRTIM_TIMB, 1), (HRTIM_TIMD, 1)                  ],
        [                  (Adc24: 1 << 10),               (Adc6810: 10)], [(HRTIM_TIMB, 2), (HRTIM_TIMC, 1)                  ],
        [(Adc13: 1 << 11),                   (Adc579: 10)               ], [(HRTIM_TIMC, 2), (HRTIM_TIMF, 1)                  ],
        [(Adc13: 1 << 12), (Adc24: 1 << 12), (Adc579: 11), (Adc6810: 11)], [(HRTIM_TIMD, 2), (HRTIM_TIME, 1)                  ]
    ],

    HRTIM_TIMB: [
        [                                                               ], [(HRTIM_TIMA, 1), (HRTIM_TIMF, 2)                 ],
        [                  (Adc24: 1 << 14),               (Adc6810: 13)], [(HRTIM_TIMA, 2), (HRTIM_TIMC, 3), (HRTIM_TIMD, 3)],
        [(Adc13: 1 << 16),                   (Adc579: 14)               ], [(HRTIM_TIMC, 4), (HRTIM_TIME, 2)                 ],
        [(Adc13: 1 << 17), (Adc24: 1 << 16), (Adc579: 15), (Adc6810: 14)], [(HRTIM_TIMD, 4), (HRTIM_TIME, 3), (HRTIM_TIMF, 3)]
    ],

    HRTIM_TIMC: [
        [                                                               ], [(HRTIM_TIME, 4), (HRTIM_TIMF, 4)                 ],
        [                  (Adc24: 1 << 18),               (Adc6810: 16)], [(HRTIM_TIMA, 3), (HRTIM_TIME, 5)                 ],
        [(Adc13: 1 << 21),                   (Adc579: 18)               ], [(HRTIM_TIMA, 4), (HRTIM_TIMB, 3)                 ],
        [(Adc13: 1 << 22), (Adc24: 1 << 20), (Adc579: 19), (Adc6810: 17)], [(HRTIM_TIMB, 4), (HRTIM_TIMD, 5), (HRTIM_TIMF, 5)]
    ],

    HRTIM_TIMD: [
        [                                                               ], [(HRTIM_TIMA, 5), (HRTIM_TIME, 6)                 ],
        [                  (Adc24: 1 << 23),               (Adc6810: 20)], [(HRTIM_TIMA, 6), (HRTIM_TIMC, 5), (HRTIM_TIME, 7)],
        [(Adc13: 1 << 25),                   (Adc579: 21)               ], [(HRTIM_TIMB, 5), (HRTIM_TIMF, 6)                 ],
        [(Adc13: 1 << 26), (Adc24: 1 << 25), (Adc579: 22), (Adc6810: 21)], [(HRTIM_TIMB, 6), (HRTIM_TIMC, 6), (HRTIM_TIMF, 7)]
    ],

    HRTIM_TIME: [
        [                                                               ], [(HRTIM_TIMB, 7), (HRTIM_TIMD, 6)                 ],
        [                  (Adc24: 1 << 28),               (Adc6810: 24)], [(HRTIM_TIMB, 8), (HRTIM_TIMF, 8)                 ],
        [(Adc13: 1 << 29), (Adc24: 1 << 29), (Adc579: 24), (Adc6810: 25)], [(HRTIM_TIMA, 7), (HRTIM_TIMC, 7), (HRTIM_TIMF, 9)],
        [(Adc13: 1 << 30), (Adc24: 1 << 30), (Adc579: 25), (Adc6810: 26)], [(HRTIM_TIMA, 8), (HRTIM_TIMC, 8), (HRTIM_TIMD, 7)]
    ],

    HRTIM_TIMF: [
        [                  (Adc24: 1 << 15)                             ], [(HRTIM_TIMD, 8)                                  ],
        [(Adc13: 1 << 10), (Adc24: 1 << 11), (Adc579: 27), (Adc6810: 28)], [(HRTIM_TIMC, 9)                                  ],
        [(Adc13: 1 << 15),                   (Adc579: 28), (Adc6810: 29)], [(HRTIM_TIMB, 9), (HRTIM_TIMD, 9), (HRTIM_TIME, 8)],
        [(Adc13: 1 << 20), (Adc24: 1 << 19), (Adc579: 29), (Adc6810: 30)], [(HRTIM_TIMA, 9), (HRTIM_TIME, 9)                 ]
    ]
}

macro_rules! hrtim_master_cr {
    ($($cr_type:ident: $cr_index:expr),*) => {$(
        /// Compare match event for neighbor timer
        impl<DST, PSCL> super::event::EventSource<DST, PSCL> for $cr_type<HRTIM_MASTER, PSCL> {
            const BITS: u32 = 1 << ($cr_index + 7); // MSTCMP1 is at bit 8 etc
        }

        impl<DST, PSCL> super::event::TimerResetEventSource<DST, PSCL> for $cr_type<HRTIM_MASTER, PSCL> {
            const BITS: u32 = 1 << ($cr_index + 4); // MSTCMP1 is at bit 5
        }
    )*};
}

hrtim_master_cr! {
    HrCr1: 1,
    HrCr2: 2,
    HrCr3: 3,
    HrCr4: 4
}

macro_rules! hrtim_timer_rst {
    ($($TIMX:ident: $cr_type:ident: $bit_index:literal),*) => {$(
        impl<DST, PSCL> super::event::TimerResetEventSource<DST, PSCL> for $cr_type<$TIMX, PSCL> {
            const BITS: u32 = 1 << $bit_index;
        }
    )*};
}

hrtim_timer_rst! {
    HRTIM_TIMA: HrCr2: 2,
    HRTIM_TIMA: HrCr4: 3,

    HRTIM_TIMB: HrCr2: 2,
    HRTIM_TIMB: HrCr4: 3,

    HRTIM_TIMC: HrCr2: 2,
    HRTIM_TIMC: HrCr4: 3,

    HRTIM_TIMD: HrCr2: 2,
    HRTIM_TIMD: HrCr4: 3,

    HRTIM_TIME: HrCr2: 2,
    HRTIM_TIME: HrCr4: 3,

    HRTIM_TIMF: HrCr2: 2,
    HRTIM_TIMF: HrCr4: 3
}
