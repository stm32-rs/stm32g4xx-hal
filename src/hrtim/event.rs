use core::marker::PhantomData;

use crate::stm32::{
    HRTIM_MASTER, HRTIM_TIMA, HRTIM_TIMB, HRTIM_TIMC, HRTIM_TIMD, HRTIM_TIME, HRTIM_TIMF,
};

use super::{compare_register::{HrCr1, HrCr2, HrCr3, HrCr4}, external_event::ExternalEventSource};
use crate::hrtim::timer::HrTim;

macro_rules! impl_into_es {
    ($dst:ident: [$(($t:ty, $ES:ident),)*]) => {$(
        impl_into_es!($dst, $t, $ES);
    )*};

    ($dst:ident, $t:ty, $ES:ident) => {
        impl<PSCL> From<&$t> for EventSource<PSCL, $dst> {
            fn from(_: &$t) -> Self {
                EventSource::$ES{ _x: PhantomData }
            }
        }
    };
    ($dst:ident) => {
        impl_into_es! {
            $dst: [
                (HrCr1<$dst, PSCL>, Cr1),
                (HrCr2<$dst, PSCL>, Cr2),
                (HrCr3<$dst, PSCL>, Cr3),
                (HrCr4<$dst, PSCL>, Cr4),
                (HrTim<$dst, PSCL>, Period),

                (HrCr1<HRTIM_MASTER, PSCL>, MasterCr1),
                (HrCr2<HRTIM_MASTER, PSCL>, MasterCr2),
                (HrCr3<HRTIM_MASTER, PSCL>, MasterCr3),
                (HrCr4<HRTIM_MASTER, PSCL>, MasterCr4),
                (HrTim<HRTIM_MASTER, PSCL>, MasterPeriod),
            ]
        }
    };
}

impl_into_es!(HRTIM_TIMA);
impl_into_es!(HRTIM_TIMB);
impl_into_es!(HRTIM_TIMC);
impl_into_es!(HRTIM_TIMD);
impl_into_es!(HRTIM_TIME);
impl_into_es!(HRTIM_TIMF);

macro_rules! impl_into_neighbor_es {
    (
        DST: $dst:ident: [
           ($src1:ident, $cr1:ident),
           ($src2:ident, $cr2:ident),
           ($src3:ident, $cr3:ident),
           ($src4:ident, $cr4:ident),
           ($src5:ident, $cr5:ident),
           ($src6:ident, $cr6:ident),
           ($src7:ident, $cr7:ident),
           ($src8:ident, $cr8:ident),
           ($src9:ident, $cr9:ident),
        ]
    ) => {
        impl_into_neighbor_es!($dst, $src1, $cr1, TimEvent1);
        impl_into_neighbor_es!($dst, $src2, $cr2, TimEvent2);
        impl_into_neighbor_es!($dst, $src3, $cr3, TimEvent3);
        impl_into_neighbor_es!($dst, $src4, $cr4, TimEvent4);
        impl_into_neighbor_es!($dst, $src5, $cr5, TimEvent5);
        impl_into_neighbor_es!($dst, $src6, $cr6, TimEvent6);
        impl_into_neighbor_es!($dst, $src7, $cr7, TimEvent7);
        impl_into_neighbor_es!($dst, $src8, $cr8, TimEvent8);
        impl_into_neighbor_es!($dst, $src9, $cr9, TimEvent9);
    };

    ($dst:ident, $src:ident, $cr:ident, $TimEventX:ident) => {
        impl<PSCL> From<&$cr<$src, PSCL>> for EventSource<PSCL, $dst> {
            fn from(_: &$cr<$src, PSCL>) -> Self {
                EventSource::NeighborTimer {
                    n: NeighborTimerEventSource::$TimEventX { _x: PhantomData },
                }
            }
        }
    };
}

impl_into_neighbor_es! {
    DST: HRTIM_TIMA: [
        // src
        (HRTIM_TIMB, HrCr1),
        (HRTIM_TIMB, HrCr2),
        (HRTIM_TIMC, HrCr2),
        (HRTIM_TIMC, HrCr3),
        (HRTIM_TIMD, HrCr1),
        (HRTIM_TIMD, HrCr2),
        (HRTIM_TIME, HrCr3),
        (HRTIM_TIME, HrCr4),
        (HRTIM_TIMF, HrCr4),
    ]
}

impl_into_neighbor_es! {
    DST: HRTIM_TIMB: [
        // src
        (HRTIM_TIMA, HrCr1),
        (HRTIM_TIMA, HrCr2),
        (HRTIM_TIMC, HrCr3),
        (HRTIM_TIMC, HrCr4),
        (HRTIM_TIMD, HrCr3),
        (HRTIM_TIMD, HrCr4),
        (HRTIM_TIME, HrCr1),
        (HRTIM_TIME, HrCr2),
        (HRTIM_TIMF, HrCr3),
    ]
}

impl_into_neighbor_es! {
    DST: HRTIM_TIMC: [
        // src
        (HRTIM_TIMA, HrCr2),
        (HRTIM_TIMA, HrCr3),
        (HRTIM_TIMB, HrCr2),
        (HRTIM_TIMB, HrCr3),
        (HRTIM_TIMD, HrCr2),
        (HRTIM_TIMD, HrCr4),
        (HRTIM_TIME, HrCr3),
        (HRTIM_TIME, HrCr4),
        (HRTIM_TIMF, HrCr2),
    ]
}

// TODO: Continue for TIMD, TIME and TIMF, see RM0440 Table 218. 'Events mapping across timer A to F'

pub enum EventSource<PSCL, DST> {
    /// Compare match with compare register 1 of this timer
    Cr1 { _x: PhantomData<(PSCL, DST)> },

    /// Compare match with compare register 2 of this timer
    Cr2 { _x: PhantomData<(PSCL, DST)> },

    /// Compare match with compare register 3 of this timer
    Cr3 { _x: PhantomData<(PSCL, DST)> },

    /// Compare match with compare register 4 of this timer
    Cr4 { _x: PhantomData<(PSCL, DST)> },

    /// On complete period
    Period { _x: PhantomData<(PSCL, DST)> },

    /// Compare match with compare register 1 of master timer
    MasterCr1 { _x: PhantomData<(PSCL, DST)> },

    /// Compare match with compare register 2 of master timer
    MasterCr2 { _x: PhantomData<(PSCL, DST)> },

    /// Compare match with compare register 3 of master timer
    MasterCr3 { _x: PhantomData<(PSCL, DST)> },

    /// Compare match with compare register 4 of master timer
    MasterCr4 { _x: PhantomData<(PSCL, DST)> },

    /// On complete master period
    MasterPeriod { _x: PhantomData<(PSCL, DST)> },

    ExternalEvent(ExternalEventSource), // This is fine 

    NeighborTimer {
        n: NeighborTimerEventSource<PSCL, DST>,
    },
}

/// Compare events from neighbor timers
///
/// See RM0440 Table 218. 'Events mapping across timer A to F'
pub enum NeighborTimerEventSource<PSCL, DST> {
    /// Timer event 1
    ///
    /// This is different depending on destination timer:
    /// |dest | source |
    /// |-----|--------|
    /// |TimA | B CR1  |
    /// |TimB | A CR1  |
    /// |TimC | A CR2  |
    /// |TimD | A CR1  |
    /// |TimE | A CR4  |
    /// |TimF | A CR3  |
    TimEvent1 {
        _x: PhantomData<(PSCL, DST)>,
    },

    /// Timer event x
    ///
    /// This is different depending on destination timer:
    /// |dest | source |
    /// |-----|--------|
    /// |TimA | x CRy  |
    /// |TimB | x CRy  |
    /// |TimC | x CRy  |
    /// |TimD | x CRy  |
    /// |TimE | x CRy  |
    /// |TimF | x CRy  |
    //TimEventx,

    /// Timer event 2
    ///
    /// This is different depending on destination timer:
    /// |dest | source |
    /// |-----|--------|
    /// |TimA | B CR2  |
    /// |TimB | A CR2  |
    /// |TimC | A CR3  |
    /// |TimD | A CR4  |
    /// |TimE | B CR3  |
    /// |TimF | B CR1  |
    TimEvent2 {
        _x: PhantomData<(PSCL, DST)>,
    },

    /// Timer event 3
    ///
    /// This is different depending on destination timer:
    /// |dest | source |
    /// |-----|--------|
    /// |TimA | C CR2  |
    /// |TimB | C CR3  |
    /// |TimC | B CR2  |
    /// |TimD | B CR2  |
    /// |TimE | B CR4  |
    /// |TimF | B CR4  |
    TimEvent3 {
        _x: PhantomData<(PSCL, DST)>,
    },

    /// Timer event 4
    ///
    /// This is different depending on destination timer:
    /// |dest | source |
    /// |-----|--------|
    /// |TimA | C CR3  |
    /// |TimB | C CR4  |
    /// |TimC | B CR3  |
    /// |TimD | B CR4  |
    /// |TimE | C CR1  |
    /// |TimF | C CR1  |
    TimEvent4 {
        _x: PhantomData<(PSCL, DST)>,
    },
    // TODO: Document those
    TimEvent5 {
        _x: PhantomData<(PSCL, DST)>,
    },
    TimEvent6 {
        _x: PhantomData<(PSCL, DST)>,
    },
    TimEvent7 {
        _x: PhantomData<(PSCL, DST)>,
    },
    TimEvent8 {
        _x: PhantomData<(PSCL, DST)>,
    },
    TimEvent9 {
        _x: PhantomData<(PSCL, DST)>,
    },
}

macro_rules! hr_timer_reset_event_source_common {
    ($(#[$($attrss:tt)*])* pub enum $t:ident { [COMMON], $(#[$($attrss2:tt)*] $vals:tt = 1 << $x:literal,)*}) => {
        $(#[$($attrss)*])*
        pub enum $t {
            $(#[$($attrss2)*] $vals = 1 << $x,)*

            /// The timer counter is reset upon external event 10.
            Eevnt10 = 1 << 18,

            /// The timer counter is reset upon external event 9.
            Eevnt9 = 1 << 17,

            /// The timer counter is reset upon external event 8.
            Eevnt8 = 1 << 16,

            /// The timer counter is reset upon external event 7.
            Eevnt7 = 1 << 15,

            /// The timer counter is reset upon external event 6.
            Eevnt6 = 1 << 14,

            /// The timer counter is reset upon external event 5.
            Eevnt5 = 1 << 13,

            /// The timer counter is reset upon external event 4.
            Eevnt4 = 1 << 12,

            /// The timer counter is reset upon external event 3.
            Eevnt3 = 1 << 11,

            /// The timer counter is reset upon external event 2.
            Eevnt2 = 1 << 10,

            /// The timer counter is reset upon external event 1.
            Eevnt1 = 1 << 9,

            /// The timer counter is reset upon master timer compare 4 event.
            MasterCmp4 = 1 << 8,

            /// The timer counter is reset upon master timer compare 3 event.
            MasterCmp3 = 1 << 7,

            /// The timer counter is reset upon master timer compare 2 event.
            MasterCmp2 = 1 << 6,

            /// The timer counter is reset upon master timer compare 1 event.
            MasterCmp1 = 1 << 5,

            /// The timer counter is reset upon master timer period event.
            MasterPeriod = 1 << 4,

            /// The timer counter is reset upon timer its own compare 4 event
            Cmp4 = 1 << 3,

            /// The timer counter is reset upon timer its own compare 2 event
            Cmp2 = 1 << 2,

            /// The timer counter is reset upon update event.
            Update = 1 << 1,
        }
    };
}

hr_timer_reset_event_source_common!(
    /// A
    pub enum TimerAResetEventSource {
        [COMMON],
        /// The timer counter is reset upon timer F compare 2 event.
        TimFCmp2 = 1 << 31,

        /// The timer counter is reset upon timer E compare 4 event.
        TimECmp4 = 1 << 30,

        /// The timer counter is reset upon timer E compare 2 event.
        TimECmp2 = 1 << 29,

        /// The timer counter is reset upon timer E compare 1 event.
        TimECmp1 = 1 << 28,

        /// The timer counter is reset upon timer D compare 4 event.
        TimDCmp4 = 1 << 27,

        /// The timer counter is reset upon timer D compare 2 event.
        TimDCmp2 = 1 << 26,

        /// The timer counter is reset upon timer D compare 1 event.
        TimDCmp1 = 1 << 25,

        /// The timer counter is reset upon timer C compare 4 event.
        TimCCmp4 = 1 << 24,

        /// The timer counter is reset upon timer C compare 2 event.
        TimCCmp2 = 1 << 23,

        /// The timer counter is reset upon timer C compare 1 event.
        TimCCmp1 = 1 << 22,

        /// The timer counter is reset upon timer B compare 4 event.
        TimBCmp4 = 1 << 21,

        /// The timer counter is reset upon timer B compare 2 event.
        TimBCmp2 = 1 << 20,

        /// The timer counter is reset upon timer B compare 1 event.
        TimBCmp1 = 1 << 19,

        /// The timer counter is reset upon timer F compare 1 event.
        TimFCmp1 = 1 << 0,
    }
);

hr_timer_reset_event_source_common!(
    /// B
    pub enum TimerBResetEventSource {
        [COMMON],

        /// The timer counter is reset upon timer F compare 2 event.
        TimFCmp2 = 1 << 31,

        /// The timer counter is reset upon timer E compare 4 event.
        TimECmp4 = 1 << 30,

        /// The timer counter is reset upon timer E compare 2 event.
        TimECmp2 = 1 << 29,

        /// The timer counter is reset upon timer E compare 1 event.
        TimECmp1 = 1 << 28,

        /// The timer counter is reset upon timer D compare 4 event.
        TimDCmp4 = 1 << 27,

        /// The timer counter is reset upon timer D compare 2 event.
        TimDCmp2 = 1 << 26,

        /// The timer counter is reset upon timer D compare 1 event.
        TimDCmp1 = 1 << 25,

        /// The timer counter is reset upon timer C compare 4 event.
        TimCCmp4 = 1 << 24,

        /// The timer counter is reset upon timer C compare 2 event.
        TimCCmp2 = 1 << 23,

        /// The timer counter is reset upon timer C compare 1 event.
        TimCCmp1 = 1 << 22,

        /// The timer counter is reset upon timer A compare 4 event.
        TimACmp4 = 1 << 21,

        /// The timer counter is reset upon timer A compare 2 event.
        TimACmp2 = 1 << 20,

        /// The timer counter is reset upon timer A compare 1 event.
        TimACmp1 = 1 << 19,

        /// The timer counter is reset upon timer F compare 1 event.
        TimFCmp1 = 1 << 0,
    }
);

hr_timer_reset_event_source_common!(
    /// C
    pub enum TimerCResetEventSource {
        [COMMON],

        /// The timer counter is reset upon timer F compare 2 event.
        TimFCmp2 = 1 << 31,

        /// The timer counter is reset upon timer E compare 4 event.
        TimECmp4 = 1 << 30,

        /// The timer counter is reset upon timer E compare 2 event.
        TimECmp2 = 1 << 29,

        /// The timer counter is reset upon timer E compare 1 event.
        TimECmp1 = 1 << 28,

        /// The timer counter is reset upon timer D compare 4 event.
        TimDCmp4 = 1 << 27,

        /// The timer counter is reset upon timer D compare 2 event.
        TimDCmp2 = 1 << 26,

        /// The timer counter is reset upon timer D compare 1 event.
        TimDCmp1 = 1 << 25,

        /// The timer counter is reset upon timer B compare 4 event.
        TimBCmp4 = 1 << 24,

        /// The timer counter is reset upon timer B compare 2 event.
        TimBCmp2 = 1 << 23,

        /// The timer counter is reset upon timer B compare 1 event.
        TimBCmp1 = 1 << 22,

        /// The timer counter is reset upon timer A compare 4 event.
        TimACmp4 = 1 << 21,

        /// The timer counter is reset upon timer A compare 2 event.
        TimACmp2 = 1 << 20,

        /// The timer counter is reset upon timer A compare 1 event.
        TimACmp1 = 1 << 19,


        /// The timer counter is reset upon timer F compare 1 event.
        TimFCmp1 = 1 << 0,
    }
);

hr_timer_reset_event_source_common!(
    /// D
    pub enum TimerDResetEventSource {
        [COMMON],

        /// The timer counter is reset upon timer F compare 2 event.
        TimFCmp2 = 1 << 31,

        /// The timer counter is reset upon timer E compare 4 event.
        TimECmp4 = 1 << 30,

        /// The timer counter is reset upon timer E compare 2 event.
        TimECmp2 = 1 << 29,

        /// The timer counter is reset upon timer E compare 1 event.
        TimECmp1 = 1 << 28,

        /// The timer counter is reset upon timer C compare 4 event.
        TimCCmp4 = 1 << 27,

        /// The timer counter is reset upon timer C compare 2 event.
        TimCCmp2 = 1 << 26,

        /// The timer counter is reset upon timer C compare 1 event.
        TimCCmp1 = 1 << 25,

        /// The timer counter is reset upon timer B compare 4 event.
        TimBCmp4 = 1 << 24,

        /// The timer counter is reset upon timer B compare 2 event.
        TimBCmp2 = 1 << 23,

        /// The timer counter is reset upon timer B compare 1 event.
        TimBCmp1 = 1 << 22,

        /// The timer counter is reset upon timer A compare 4 event.
        TimACmp4 = 1 << 21,

        /// The timer counter is reset upon timer A compare 2 event.
        TimACmp2 = 1 << 20,

        /// The timer counter is reset upon timer A compare 1 event.
        TimACmp1 = 1 << 19,

        /// The timer counter is reset upon timer F compare 1 event.
        TimFCmp1 = 1 << 0,
    }
);

hr_timer_reset_event_source_common!(
    /// E
    pub enum TimerEResetEventSource {
        [COMMON],

        /// The timer counter is reset upon timer F compare 2 event.
        TimFCmp2 = 1 << 31,

        /// The timer counter is reset upon timer D compare 4 event.
        TimDCmp4 = 1 << 30,

        /// The timer counter is reset upon timer D compare 2 event.
        TimDCmp2 = 1 << 29,

        /// The timer counter is reset upon timer D compare 1 event.
        TimDCmp1 = 1 << 28,

        /// The timer counter is reset upon timer C compare 4 event.
        TimCCmp4 = 1 << 27,

        /// The timer counter is reset upon timer C compare 2 event.
        TimCCmp2 = 1 << 26,

        /// The timer counter is reset upon timer C compare 1 event.
        TimCCmp1 = 1 << 25,

        /// The timer counter is reset upon timer B compare 4 event.
        TimBCmp4 = 1 << 24,

        /// The timer counter is reset upon timer B compare 2 event.
        TimBCmp2 = 1 << 23,

        /// The timer counter is reset upon timer B compare 1 event.
        TimBCmp1 = 1 << 22,

        /// The timer counter is reset upon timer A compare 4 event.
        TimACmp4 = 1 << 21,

        /// The timer counter is reset upon timer A compare 2 event.
        TimACmp2 = 1 << 20,

        /// The timer counter is reset upon timer A compare 1 event.
        TimACmp1 = 1 << 19,


        /// The timer counter is reset upon timer F compare 1 event.
        TimFCmp1 = 1 << 0,
    }
);

hr_timer_reset_event_source_common!(
    /// F
    pub enum TimerFResetEventSource {
        [COMMON],

        /// The timer counter is reset upon timer E compare 2 event.
        TimECmp2 = 1 << 31,

        /// The timer counter is reset upon timer D compare 4 event.
        TimDCmp4 = 1 << 30,

        /// The timer counter is reset upon timer D compare 2 event.
        TimDCmp2 = 1 << 29,

        /// The timer counter is reset upon timer D compare 1 event.
        TimDCmp1 = 1 << 28,

        /// The timer counter is reset upon timer C compare 4 event.
        TimCCmp4 = 1 << 27,

        /// The timer counter is reset upon timer C compare 2 event.
        TimCCmp2 = 1 << 26,

        /// The timer counter is reset upon timer C compare 1 event.
        TimCCmp1 = 1 << 25,

        /// The timer counter is reset upon timer B compare 4 event.
        TimBCmp4 = 1 << 24,

        /// The timer counter is reset upon timer B compare 2 event.
        TimBCmp2 = 1 << 23,

        /// The timer counter is reset upon timer B compare 1 event.
        TimBCmp1 = 1 << 22,

        /// The timer counter is reset upon timer A compare 4 event.
        TimACmp4 = 1 << 21,

        /// The timer counter is reset upon timer A compare 2 event.
        TimACmp2 = 1 << 20,

        /// The timer counter is reset upon timer A compare 1 event.
        TimACmp1 = 1 << 19,

        /// The timer counter is reset upon timer E compare 1 event.
        TimECmp1 = 1 << 0,
    }
);
