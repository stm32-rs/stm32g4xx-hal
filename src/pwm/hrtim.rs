mod external_event;
mod fault;
mod calibration;

use core::marker::PhantomData;
use core::mem::MaybeUninit;

use fugit::HertzU64;

use crate::gpio::gpioa::{PA10, PA11, PA8, PA9};
use crate::gpio::gpiob::{PB12, PB13, PB14, PB15};
use crate::gpio::gpioc::{PC6, PC7, PC8, PC9};
use crate::gpio::{Alternate, AF13, AF3};
use crate::pwm::hrtim::fault::{FaultAction, FaultSource};
use crate::stm32::{
    HRTIM_COMMON, HRTIM_MASTER, HRTIM_TIMA, HRTIM_TIMB, HRTIM_TIMC, HRTIM_TIMD, HRTIM_TIME,
    HRTIM_TIMF,
};

use self::calibration::HrTimOngoingCalibration;
use self::fault::{
    FltMonitor1, FltMonitor2, FltMonitor3, FltMonitor4, FltMonitor5, FltMonitor6, FltMonitorSys,
};

use super::{
    ActiveHigh, Alignment, ComplementaryImpossible, Pins, Polarity, Pwm, PwmPinEnable, TimerType,
};
use crate::rcc::{GetBusFreq, Rcc};
use crate::time::Hertz;

pub struct CH1<PSCL>(PhantomData<PSCL>);
pub struct CH2<PSCL>(PhantomData<PSCL>);

/// Internal enum that keeps track of the count settings before PWM is finalized
enum CountSettings {
    Frequency(Hertz),
    Period(u16),
}

#[derive(Copy, Clone, PartialEq, Debug)]
pub enum HrTimerMode {
    SingleShotNonRetriggerable,
    SingleShotRetriggerable,
    Continuous,
}

#[derive(Copy, Clone, PartialEq, Debug)]
pub enum HrCountingDirection {
    /// Asymetrical up counting mode
    ///
    ///

    ///                   *                  *
    ///  Counting up   *  |               *  |
    ///             *                  *
    ///          *        |         *        |
    ///       *                  *           
    ///    *              |   *              |
    /// *                  *
    /// --------------------------------------
    ///
    /// ```txt
    /// |         *-------*                  *------
    ///           |       |                  |
    /// |         |       |                  |
    ///           |       |                  |
    /// ----------*       *------------------*
    /// ```
    ///
    /// This is the most common mode with least amount of quirks
    Up,

    /// Symmetrical up-down counting mode
    ///
    ///
    /// ```txt
    /// Period-->                  *                      Counting     *
    ///           Counting up   *  |  *     Counting        Up      *  |
    ///                      *           *     down              *
    ///                   *        |        *                 *        |
    ///                *                       *           *
    ///             *              |              *     *              |
    /// 0     -->*                                   *                  
    /// ---------------------------------------------------------------------------
    ///          |         *---------------*         |         *---------------*
    ///                    |       |       |                   |       |       |
    ///          |         |               |         |         |               |
    ///                    |       |       |                   |       |       |
    ///          ----------*               *-------------------*               *---
    /// ```
    ///
    /// NOTE: This is incompatible with
    /// * Auto-delay
    /// * Balanded Idle
    /// * Triggered-half mode
    ///
    /// There is also differences in (including but not limited to) the following areas:
    /// * Counter roll over event
    /// * The events registered with `enable_set_event` will work as normal wen counting up, however when counting down, they will work as rst events.
    /// * The events registered with `enable_rst_event` will work as normal wen counting up, however when counting down, they will work as set events.
    UpDown,
}

// Needed to calculate frequency
impl Into<super::Alignment> for HrCountingDirection {
    fn into(self) -> super::Alignment {
        match self {
            HrCountingDirection::Up => super::Alignment::Left,
            HrCountingDirection::UpDown => super::Alignment::Center,
        }
    }
}

#[derive(Copy, Clone, PartialEq, Debug)]
pub enum InterleavedMode {
    Disabled,

    /// Dual interleaved or Half mode
    ///
    /// Automatically force
    /// * Cr1 to PERIOD / 2 (not visable through `get_duty`).
    /// Automatically updates when changing period
    ///
    /// NOTE: Affects Cr1
    Dual,

    /// Triple interleaved mode
    ///
    /// Automatically force
    /// * Cr1 to 1 * PERIOD / 3 and
    /// * Cr2 to 2 * PERIOD / 3
    /// (not visable through `get_duty`). Automatically updates when changing period.
    ///
    /// NOTE: Must not be used simultaneously with other modes
    /// using CMP2 (dual channel dac trigger and triggered-half modes).
    Triple,

    /// Quad interleaved mode
    ///
    /// Automatically force
    /// * Cr1 to 1 * PERIOD / 4,
    /// * Cr2 to 2 * PERIOD / 4 and
    /// * Cr3 to 3 * PERIOD / 4
    /// (not visable through `get_duty`). Automatically updates when changing period.
    ///
    /// NOTE: Must not be used simultaneously with other modes
    /// using CMP2 (dual channel dac trigger and triggered-half modes).
    Quad,
}

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

impl<PSCL> HrtimChannel<PSCL> for () {}

/*pub struct Hrtimer<PSCL, TIM> {
    _prescaler: PhantomData<PSCL>,
    _timer: PhantomData<TIM>,
    period: u16, // $perXr.$perx

    cmp_value1: u16, // $cmpX1r.cmp1x
    cmp_value2: u16, // $cmpX2r.cmp2x
    cmp_value3: u16, // $cmpX3r.cmp3x
    cmp_value4: u16, // $cmpX4r.cmp4x
}*/

// HrPwmExt trait
/// Allows the pwm() method to be added to the peripheral register structs from the device crate
pub trait HrPwmExt: Sized {
    /// The requested frequency will be rounded to the nearest achievable frequency; the actual frequency may be higher or lower than requested.
    fn pwm<PINS, T, U, V>(
        self,
        _pins: PINS,
        frequency: T,
        control: &mut HrPwmControl,
        rcc: &mut Rcc,
    ) -> PINS::Channel
    where
        PINS: Pins<Self, U, V> + ToHrOut,
        T: Into<Hertz>,
        U: HrtimChannel<Pscl128>;
}

pub trait HrPwmAdvExt: Sized {
    type PreloadSource;

    fn pwm_advanced<PINS, CHANNEL, COMP>(
        self,
        _pins: PINS,
        rcc: &mut Rcc,
    ) -> HrPwmBuilder<Self, Pscl128, Self::PreloadSource, PINS::Out<Pscl128>>
    where
        PINS: Pins<Self, CHANNEL, COMP> + ToHrOut,
        CHANNEL: HrtimChannel<Pscl128>;
}

/// HrPwmBuilder is used to configure advanced HrTim PWM features
pub struct HrPwmBuilder<TIM, PSCL, PS, OUT> {
    _tim: PhantomData<TIM>,
    _prescaler: PhantomData<PSCL>,
    _out: PhantomData<OUT>,
    timer_mode: HrTimerMode,
    counting_direction: HrCountingDirection,
    base_freq: HertzU64,
    count: CountSettings,
    preload_source: Option<PS>,
    fault_enable_bits: u8,
    fault1_bits: u8,
    fault2_bits: u8,
    enable_push_pull: bool,
    interleaved_mode: InterleavedMode, // Also includes half mode
    repetition_counter: u8,
    deadtime: Option<DeadtimeConfig>,
    enable_repetition_interrupt: bool,
    out1_polarity: Polarity,
    out2_polarity: Polarity,
}

pub enum PreloadSource {
    /// Preloaded registers are updated on counter roll over or counter reset
    OnCounterReset,

    /// Preloaded registers are updated by master timer update
    OnMasterTimerUpdate,

    /// Prealoaded registers are updaten when the counter rolls over and the repetition counter is 0
    OnRepetitionUpdate,
}

#[derive(Copy, Clone, Debug)]
pub struct DeadtimeConfig {
    /// Prescaler for both rising and falling deadtime
    prescaler: DeadtimePrescaler,

    /// 9-bits
    deadtime_rising_value: u16,

    /// Is deadtime negative
    deadtime_rising_sign: bool,

    /// 9-bits
    deadtime_falling_value: u16,

    /// Is deadtime negative
    deadtime_falling_sign: bool,
}

impl DeadtimeConfig {
    /// See RM0440 Table 221 'Deadtime resolution and max absolute values'
    pub fn prescaler(mut self, value: DeadtimePrescaler) -> Self {
        self.prescaler = value;
        self
    }

    /// Panic if value can not fit in 9 bits
    pub fn deadtime_rising_value(mut self, value: u16) -> Self {
        // 9 bits
        assert!(value < (1 << 9));

        self.deadtime_rising_value = value;

        self
    }

    pub fn deadtime_rising_sign(mut self, is_negative: bool) -> Self {
        self.deadtime_rising_sign = is_negative;
        self
    }

    /// Panic if value can not fit in 9 bits
    pub fn deadtime_falling_value(mut self, value: u16) -> Self {
        // 9 bits
        assert!(value < (1 << 9));

        self.deadtime_falling_value = value;

        self
    }

    pub fn deadtime_falling_sign(mut self, is_negative: bool) -> Self {
        self.deadtime_falling_sign = is_negative;
        self
    }
}

impl Default for DeadtimeConfig {
    fn default() -> Self {
        Self {
            prescaler: DeadtimePrescaler::Thrtim,
            deadtime_rising_value: 170, // about 1us when f_sys = 170MHz
            deadtime_rising_sign: false,
            deadtime_falling_value: 170, // about 1us when f_sys = 170MHz
            deadtime_falling_sign: false,
        }
    }
}

#[derive(Copy, Clone, Debug)]
pub enum DeadtimePrescaler {
    ThrtimDiv8 = 0b000,
    ThrtimDiv4 = 0b001,
    ThrtimDiv2 = 0b010,
    Thrtim = 0b011,
    ThrtimMul2 = 0b100,
    ThrtimMul4 = 0b101,
    ThrtimMul8 = 0b110,
    ThrtimMul16 = 0b111,
}

pub enum MasterPreloadSource {
    /// Prealoaded registers are updaten when the master counter rolls over and the master repetition counter is 0
    OnMasterRepetitionUpdate,
}

pub trait HrCompareRegister {
    fn get_duty(&self) -> u16;
    fn set_duty(&mut self, duty: u16);
}

pub struct HrCr1<TIM, PSCL>(PhantomData<(TIM, PSCL)>);
pub struct HrCr2<TIM, PSCL>(PhantomData<(TIM, PSCL)>);
pub struct HrCr3<TIM, PSCL>(PhantomData<(TIM, PSCL)>);
pub struct HrCr4<TIM, PSCL>(PhantomData<(TIM, PSCL)>);

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

macro_rules! impl_into_es {
    ($dst:ident: [$(($t:ty, $ES:ident),)*]) => {$(
        impl_into_es!($dst, $t, $ES);
    )*};

    ($dst:ident, $t:ty, $ES:ident) => {
        impl<PSCL> Into<EventSource<PSCL, $dst>> for &$t {
            fn into(self) -> EventSource<PSCL, $dst> {
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
        impl<PSCL> Into<EventSource<PSCL, $dst>> for &$cr<$src, PSCL> {
            fn into(self) -> EventSource<PSCL, $dst> {
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

macro_rules! hrtim_finalize_body {
    ($this:expr, $PreloadSource:ident, $TIMX:ident: (
        $timXcr:ident, $ck_psc:ident, $perXr:ident, $perx:ident, $tXcen:ident, $rep:ident, $repx:ident, $dier:ident, $repie:ident $(, $timXcr2:ident, $fltXr:ident, $outXr:ident, $dtXr:ident)*),
    ) => {{
        let tim = unsafe { &*$TIMX::ptr() };
        let (period, prescaler_bits) = match $this.count {
            CountSettings::Period(period) => (period as u32, PSCL::BITS as u16),
            CountSettings::Frequency( freq ) => {
                <TimerHrTim<PSCL>>::calculate_frequency($this.base_freq, freq, $this.counting_direction.into())
            },
        };

        let (half, intlvd) = match $this.interleaved_mode {
            InterleavedMode::Disabled => (false, 0b00),
            InterleavedMode::Dual => (true, 0b00),
            InterleavedMode::Triple => (false, 0b01),
            InterleavedMode::Quad => (false, 0b10),
        };

        // Write prescaler and any special modes
        tim.$timXcr.modify(|_r, w| unsafe {
            w
                // Enable Continuous mode
                .cont().bit($this.timer_mode == HrTimerMode::Continuous)
                .retrig().bit($this.timer_mode == HrTimerMode::SingleShotRetriggerable)

                // TODO: add support for more modes

                // Interleaved mode
                .intlvd().bits(intlvd)

                // half/double interleaved mode
                .half().bit(half)

                // Set prescaler
                .$ck_psc().bits(prescaler_bits as u8)
        });

        $(
            tim.$timXcr2.modify(|_r, w|
                // Set counting direction
                w.udm().bit($this.counting_direction == HrCountingDirection::UpDown)
            );

            // Only available for timers with outputs(not HRTIM_MASTER)
            let _ = tim.$outXr;
            tim.$timXcr.modify(|_r, w|
                // Push-Pull mode
                w.pshpll().bit($this.enable_push_pull)
            );
        )*

        // Write period
        tim.$perXr.write(|w| unsafe { w.$perx().bits(period as u16) });

        // Enable fault sources and lock configuration
        $(unsafe {
            // Enable fault sources
            let fault_enable_bits = $this.fault_enable_bits as u32;
            tim.$fltXr.write(|w| w
                .flt1en().bit(fault_enable_bits & (1 << 0) != 0)
                .flt2en().bit(fault_enable_bits & (1 << 1) != 0)
                .flt3en().bit(fault_enable_bits & (1 << 2) != 0)
                .flt4en().bit(fault_enable_bits & (1 << 3) != 0)
                .flt5en().bit(fault_enable_bits & (1 << 4) != 0)
                .flt6en().bit(fault_enable_bits & (1 << 5) != 0)
            );

            // ... and lock configuration
            tim.$fltXr.modify(|_r, w| w.fltlck().set_bit());

            tim.$outXr.modify(|_r, w| w
                // Set actions on fault for both outputs
                .fault1().bits($this.fault1_bits)
                .fault2().bits($this.fault2_bits)

                // Set output polarity for both outputs
                .pol1().bit($this.out1_polarity == Polarity::ActiveLow)
                .pol2().bit($this.out2_polarity == Polarity::ActiveLow)
            );
            if let Some(deadtime) = $this.deadtime {
                let DeadtimeConfig {
                    prescaler,
                    deadtime_rising_value,
                    deadtime_rising_sign,
                    deadtime_falling_value,
                    deadtime_falling_sign,
                } = deadtime;

                // SAFETY: DeadtimeConfig makes sure rising and falling values are valid
                // and DeadtimePrescaler has its own garantuee
                tim.$dtXr.modify(|_r, w| w
                    .dtprsc().bits(prescaler as u8)
                    .dtrx().bits(deadtime_rising_value)
                    .sdtrx().bit(deadtime_rising_sign)
                    .dtfx().bits(deadtime_falling_value)
                    .sdtfx().bit(deadtime_falling_sign)

                    // Lock configuration
                    .dtflkx().set_bit()
                    .dtfslkx().set_bit()
                    .dtrlkx().set_bit()
                    .dtrslkx().set_bit()
                );
                tim.$outXr.modify(|_r, w| w.dten().set_bit());
            }
        })*


        hrtim_finalize_body!($PreloadSource, $this, tim, $timXcr);

        // Set repetition counter
        unsafe { tim.$rep.write(|w| w.$repx().bits($this.repetition_counter)); }

        // Enable interrupts
        tim.$dier.modify(|_r, w| w.$repie().bit($this.enable_repetition_interrupt));

        // Start timer
        //let master = unsafe { &*HRTIM_MASTER::ptr() };
        //master.mcr.modify(|_r, w| { w.$tXcen().set_bit() });

        unsafe {
            MaybeUninit::uninit().assume_init()
        }
    }};

    (PreloadSource, $this:expr, $tim:expr, $timXcr:ident) => {{
        match $this.preload_source {
            Some(PreloadSource::OnCounterReset) => {
                $tim.$timXcr.modify(|_r, w| w
                    .tx_rstu().set_bit()
                    .preen().set_bit()
                )
            },
            Some(PreloadSource::OnMasterTimerUpdate) => {
                $tim.$timXcr.modify(|_r, w| w
                    .mstu().set_bit()
                    .preen().set_bit()
                )
            }
            Some(PreloadSource::OnRepetitionUpdate) => {
                $tim.$timXcr.modify(|_r, w| w
                    .tx_repu().set_bit()
                    .preen().set_bit()
                )
            }
            None => ()
        }
    }};

    (MasterPreloadSource, $this:expr, $tim:expr, $timXcr:ident) => {{
        match $this.preload_source {
            Some(MasterPreloadSource::OnMasterRepetitionUpdate) => {
                $tim.$timXcr.modify(|_r, w| w
                    .mrepu().set_bit()
                    .preen().set_bit()
                )
            }
            None => ()
        }
    }};
}

macro_rules! hrtim_common_methods {
    ($TIMX:ident, $PS:ident) => {
        /// Set the PWM frequency; will overwrite the previous prescaler and period
        /// The requested frequency will be rounded to the nearest achievable frequency; the actual frequency may be higher or lower than requested.
        pub fn frequency<T: Into<Hertz>>(mut self, freq: T) -> Self {
            self.count = CountSettings::Frequency(freq.into());

            self
        }

        /// Set the prescaler; PWM count runs at base_frequency/(prescaler+1)
        pub fn prescaler<P>(
            self,
            _prescaler: P,
        ) -> HrPwmBuilder<$TIMX, P, $PS, <OUT as ToHrOut>::Out<P>>
        where
            P: HrtimPrescaler,
        {
            let HrPwmBuilder {
                _tim,
                _prescaler: _,
                _out,
                timer_mode,
                fault_enable_bits,
                fault1_bits,
                fault2_bits,
                enable_push_pull,
                interleaved_mode,
                counting_direction,
                base_freq,
                count,
                preload_source,
                repetition_counter,
                deadtime,
                enable_repetition_interrupt,
                out1_polarity,
                out2_polarity,
            } = self;

            let period = match count {
                CountSettings::Frequency(_) => u16::MAX,
                CountSettings::Period(period) => period,
            };

            let count = CountSettings::Period(period);

            HrPwmBuilder {
                _tim,
                _prescaler: PhantomData,
                _out: PhantomData,
                timer_mode,
                fault_enable_bits,
                fault1_bits,
                fault2_bits,
                enable_push_pull,
                interleaved_mode,
                counting_direction,
                base_freq,
                count,
                preload_source,
                repetition_counter,
                deadtime,
                enable_repetition_interrupt,
                out1_polarity,
                out2_polarity,
            }
        }

        pub fn timer_mode(mut self, timer_mode: HrTimerMode) -> Self {
            self.timer_mode = timer_mode;

            self
        }

        // TODO: Allow setting multiple?
        pub fn preload(mut self, preload_source: $PS) -> Self {
            self.preload_source = Some(preload_source);

            self
        }

        /// Set the period; PWM count runs from 0 to period, repeating every (period+1) counts
        pub fn period(mut self, period: u16) -> Self {
            self.count = CountSettings::Period(period);

            self
        }

        /// Set repetition counter, useful to reduce interrupts generated
        /// from timer by a factor (repetition_counter + 1)
        pub fn repetition_counter(mut self, repetition_counter: u8) -> Self {
            self.repetition_counter = repetition_counter;

            self
        }

        pub fn enable_repetition_interrupt(mut self) -> Self {
            self.enable_repetition_interrupt = true;

            self
        }
    };
}

// Implement PWM configuration for timer
macro_rules! hrtim_hal {
    ($($TIMX:ident: ($timXcr:ident, $timXcr2:ident, $perXr:ident, $tXcen:ident, $rep:ident, $repx:ident, $dier:ident, $repie:ident, $fltXr:ident, $outXr:ident, $dtXr:ident),)+) => {
        $(

            // Implement HrPwmExt trait for hrtimer
            impl HrPwmExt for $TIMX {
                fn pwm<PINS, T, U, V>(
                    self,
                    pins: PINS,
                    frequency: T,
                    control: &mut HrPwmControl,
                    rcc: &mut Rcc,
                ) -> PINS::Channel
                where
                    PINS: Pins<Self, U, V> + ToHrOut,
                    T: Into<Hertz>,
                    U: HrtimChannel<Pscl128>,
                {
                    let _= self.pwm_advanced(pins, rcc).frequency(frequency).finalize(control);

                    unsafe { MaybeUninit::<PINS::Channel>::uninit().assume_init() }
                }
            }

            impl HrPwmAdvExt for $TIMX {
                type PreloadSource = PreloadSource;

                fn pwm_advanced<PINS, CHANNEL, COMP>(
                    self,
                    _pins: PINS,
                    rcc: &mut Rcc,
                ) -> HrPwmBuilder<Self, Pscl128, Self::PreloadSource, PINS::Out<Pscl128>>
                where
                    PINS: Pins<Self, CHANNEL, COMP> + ToHrOut,
                    CHANNEL: HrtimChannel<Pscl128>
                {
                    // TODO: That 32x factor... Is that included below, or should we
                    // do that? Also that will likely risk overflowing u32 since
                    // 170MHz * 32 = 5.44GHz > u32::MAX.Hz()
                    let clk = HertzU64::from(HRTIM_COMMON::get_timer_frequency(&rcc.clocks)) * 32;

                    HrPwmBuilder {
                        _tim: PhantomData,
                        _prescaler: PhantomData,
                        _out: PhantomData,
                        timer_mode: HrTimerMode::Continuous,
                        fault_enable_bits: 0b000000,
                        fault1_bits: 0b00,
                        fault2_bits: 0b00,
                        counting_direction: HrCountingDirection::Up,
                        base_freq: clk,
                        count: CountSettings::Period(u16::MAX),
                        preload_source: None,
                        enable_push_pull: false,
                        interleaved_mode: InterleavedMode::Disabled,
                        repetition_counter: 0,
                        deadtime: None,
                        enable_repetition_interrupt: false,
                        out1_polarity: Polarity::ActiveHigh,
                        out2_polarity: Polarity::ActiveHigh,
                    }
                }
            }

            impl<PSCL, OUT>
                HrPwmBuilder<$TIMX, PSCL, PreloadSource, OUT>
            where
                PSCL: HrtimPrescaler,
                OUT: ToHrOut,
            {
                pub fn finalize(self, _control: &mut HrPwmControl) -> (HrTim<$TIMX, PSCL>, (HrCr1<$TIMX, PSCL>, HrCr2<$TIMX, PSCL>, HrCr3<$TIMX, PSCL>, HrCr4<$TIMX, PSCL>), OUT) {
                    hrtim_finalize_body!(self, PreloadSource, $TIMX: ($timXcr, ck_pscx, $perXr, perx, $tXcen, $rep, $repx, $dier, $repie, $timXcr2, $fltXr, $outXr, $dtXr),)
                }

                hrtim_common_methods!($TIMX, PreloadSource);

                pub fn with_fault_source<FS>(mut self, _fault_source: FS) -> Self
                    where FS: FaultSource
                {
                    self.fault_enable_bits = self.fault_enable_bits | FS::ENABLE_BITS;

                    self
                }

                pub fn fault_action1(mut self, fault_action1: FaultAction) -> Self {
                    self.fault1_bits = fault_action1 as _;

                    self
                }

                pub fn fault_action2(mut self, fault_action2: FaultAction) -> Self {
                    self.fault2_bits = fault_action2 as _;

                    self
                }

                pub fn out1_polarity(mut self, polarity: Polarity) -> Self {
                    self.out1_polarity = polarity;

                    self
                }

                pub fn out2_polarity(mut self, polarity: Polarity) -> Self {
                    self.out2_polarity = polarity;

                    self
                }

                /// Enable or disable Push-Pull mode
                ///
                /// Enabling Push-Pull mode will make output 1 and 2
                /// alternate every period with one being
                /// inactive and the other getting to output its wave form
                /// as normal
                ///
                ///         ----           .                ----
                ///out1    |    |          .               |    |
                ///        |    |          .               |    |
                /// --------    ----------------------------    --------------------
                ///        .                ------         .                ------
                ///out2    .               |      |        .               |      |
                ///        .               |      |        .               |      |
                /// ------------------------    ----------------------------      --
                ///
                /// NOTE: setting this will overide any 'Swap Mode' set
                pub fn push_pull_mode(mut self, enable: bool) -> Self {
                    // TODO: add check for incompatible modes
                    self.enable_push_pull = enable;

                    self
                }

                /// Set counting direction
                ///
                /// See [`HrCountingDirection`]
                pub fn counting_direction(mut self, counting_direction: HrCountingDirection) -> Self {
                    self.counting_direction = counting_direction;

                    self
                }

                /// Set interleaved or half modes
                ///
                /// NOTE: Check [`InterleavedMode`] for more info about special cases
                pub fn interleaved_mode(mut self, mode: InterleavedMode) -> Self {
                    self.interleaved_mode = mode;

                    self
                }

                pub fn deadtime(mut self, deadtime: DeadtimeConfig) -> Self {
                    self.deadtime = Some(deadtime);

                    self
                }

                //pub fn swap_mode(mut self, enable: bool) -> Self
            }
        )+
    };
}

macro_rules! hrtim_hal_master {
    ($($TIMX:ident: ($timXcr:ident, $ck_psc:ident, $perXr:ident, $perx:ident, $rep:ident, $tXcen:ident, $dier:ident, $repie:ident),)+) => {$(
        impl HrPwmAdvExt for $TIMX {
            type PreloadSource = MasterPreloadSource;

            fn pwm_advanced<PINS, CHANNEL, COMP>(
                self,
                _pins: PINS,
                rcc: &mut Rcc,
            ) -> HrPwmBuilder<Self, Pscl128, Self::PreloadSource, PINS::Out<Pscl128>>
            where
                PINS: Pins<Self, CHANNEL, COMP> + ToHrOut, // TODO: figure out
                CHANNEL: HrtimChannel<Pscl128>
            {
                // TODO: That 32x factor... Is that included below, or should we
                // do that? Also that will likely risk overflowing u32 since
                // 170MHz * 32 = 5.44GHz > u32::MAX.Hz()
                let clk = HertzU64::from(HRTIM_COMMON::get_timer_frequency(&rcc.clocks)) * 32;

                HrPwmBuilder {
                    _tim: PhantomData,
                    _prescaler: PhantomData,
                    _out: PhantomData,
                    timer_mode: HrTimerMode::Continuous,
                    fault_enable_bits: 0b000000,
                    fault1_bits: 0b00,
                    fault2_bits: 0b00,
                    counting_direction: HrCountingDirection::Up,
                    base_freq: clk,
                    count: CountSettings::Period(u16::MAX),
                    preload_source: None,
                    enable_push_pull: false,
                    interleaved_mode: InterleavedMode::Disabled,
                    repetition_counter: 0,
                    deadtime: None,
                    enable_repetition_interrupt: false,
                    out1_polarity: Polarity::ActiveHigh,
                    out2_polarity: Polarity::ActiveHigh,
                }
            }
        }

        impl<PSCL, OUT>
            HrPwmBuilder<$TIMX, PSCL, MasterPreloadSource, OUT>
        where
            PSCL: HrtimPrescaler,
            OUT: ToHrOut,
        {
            pub fn finalize(self, _control: &mut HrPwmControl) -> (HrTim<$TIMX, PSCL>, (HrCr1<$TIMX, PSCL>, HrCr2<$TIMX, PSCL>, HrCr3<$TIMX, PSCL>, HrCr4<$TIMX, PSCL>)) {
                hrtim_finalize_body!(self, MasterPreloadSource, $TIMX: ($timXcr, $ck_psc, $perXr, $perx, $tXcen, $rep, $rep, $dier, $repie),)
            }

            hrtim_common_methods!($TIMX, MasterPreloadSource);
        }
    )*}
}

macro_rules! hrtim_pin_hal {
    ($($TIMX:ident:
        ($CH:ident, $perXr:ident, $cmpXYr:ident, $cmpYx:ident, $cmpY:ident, $tXYoen:ident, $tXYodis:ident),)+
     ) => {
        $(
            impl<PSCL, COMP, POL, NPOL> hal::PwmPin for Pwm<$TIMX, $CH<PSCL>, COMP, POL, NPOL>
                where Pwm<$TIMX, $CH<PSCL>, COMP, POL, NPOL>: PwmPinEnable {
                type Duty = u16;

                // You may not access self in the following methods!
                // See unsafe above

                fn disable(&mut self) {
                    self.ccer_disable();
                }

                fn enable(&mut self) {
                    self.ccer_enable();
                }

                fn get_duty(&self) -> Self::Duty {
                    let tim = unsafe { &*$TIMX::ptr() };

                    tim.$cmpXYr.read().$cmpYx().bits()
                }

                fn get_max_duty(&self) -> Self::Duty {
                    let tim = unsafe { &*$TIMX::ptr() };

                    let arr = tim.$perXr.read().perx().bits();

                    // One PWM cycle is ARR+1 counts long
                    // Valid PWM duty cycles are 0 to ARR+1
                    // However, if ARR is 65535 on a 16-bit timer, we can't add 1
                    // In that case, 100% duty cycle is not possible, only 65535/65536
                    if arr == Self::Duty::MAX {
                        arr
                    }
                    else {
                        arr + 1
                    }
                }

                /// Set duty cycle
                ///
                /// NOTE: Please observe limits(RM0440 "Period and compare registers min and max values"):
                /// | Prescaler | Min duty | Max duty |
                /// |-----------|----------|----------|
                /// |         1 |   0x0060 |   0xFFDF |
                /// |         2 |   0x0030 |   0xFFEF |
                /// |         4 |   0x0018 |   0xFFF7 |
                /// |         8 |   0x000C |   0xFFFB |
                /// |        16 |   0x0006 |   0xFFFD |
                /// |        32 |   0x0003 |   0xFFFD |
                /// |        64 |   0x0003 |   0xFFFD |
                /// |       128 |   0x0003 |   0xFFFD |
                ///
                /// Also, writing 0 as duty is only valid for CR1 and CR3 during a set of
                /// specific conditions(see RM0440 "Null duty cycle exception case"):
                /// – the output SET event is generated by the PERIOD event
                /// – the output RESET if generated by the compare 1 (respectively compare 3) event
                /// – the compare 1 (compare 3) event is active within the timer unit itself, and not used
                /// for other timing units
                fn set_duty(&mut self, duty: Self::Duty) {
                    let tim = unsafe { &*$TIMX::ptr() };

                    tim.$cmpXYr.write(|w| unsafe { w.$cmpYx().bits(duty) });
                }
            }

            // Enable implementation for ComplementaryImpossible
            impl<POL, NPOL, PSCL> PwmPinEnable for Pwm<$TIMX, $CH<PSCL>, ComplementaryImpossible, POL, NPOL> {
                fn ccer_enable(&mut self) {
                    // TODO: Should this part only be in Pwm::enable?
                    // Enable output Y on channel X
                    // This is a set-only register, no risk for data race
                    let common = unsafe { &*HRTIM_COMMON::ptr() };
                    common.oenr.write(|w| { w.$tXYoen().set_bit() });
                }
                fn ccer_disable(&mut self) {
                    // TODO: Should this part only be in Pwm::disable
                    // Disable output Y on channel X
                    // This is a write only register, no risk for data race
                    let common = unsafe { &*HRTIM_COMMON::ptr() };
                    common.odisr.write(|w| { w.$tXYodis().set_bit() });
                }
            }
        )+
    }
}

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

macro_rules! hrtim_cr_helper {
    ($TIMX:ident: $cr_type:ident: $cmpXYr:ident, $cmpYx:ident) => {
        impl<PSCL> HrCompareRegister for $cr_type<$TIMX, PSCL> {
            fn get_duty(&self) -> u16 {
                let tim = unsafe { &*$TIMX::ptr() };

                tim.$cmpXYr.read().$cmpYx().bits()
            }
            fn set_duty(&mut self, duty: u16) {
                let tim = unsafe { &*$TIMX::ptr() };

                tim.$cmpXYr.write(|w| unsafe { w.$cmpYx().bits(duty) });
            }
        }
    };
}

macro_rules! hrtim_cr {
    ($($TIMX:ident: [
        $cmpX1r:ident, $cmpX2r:ident, $cmpX3r:ident, $cmpX4r:ident,
        $cmp1x:ident, $cmp2x:ident, $cmp3x:ident, $cmp4x:ident
    ],)+) => {$(
        hrtim_cr_helper!($TIMX: HrCr1: $cmpX1r, $cmp1x);
        hrtim_cr_helper!($TIMX: HrCr2: $cmpX2r, $cmp2x);
        hrtim_cr_helper!($TIMX: HrCr3: $cmpX3r, $cmp3x);
        hrtim_cr_helper!($TIMX: HrCr4: $cmpX4r, $cmp4x);
    )+};
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

hrtim_cr! {
    HRTIM_MASTER: [mcmp1r, mcmp2r, mcmp3r, mcmp4r, mcmp1, mcmp2, mcmp3, mcmp4],

    HRTIM_TIMA: [cmp1ar, cmp2ar, cmp3ar, cmp4ar, cmp1x, cmp2x, cmp3x, cmp4x],
    HRTIM_TIMB: [cmp1br, cmp2br, cmp3br, cmp4br, cmp1x, cmp2x, cmp3x, cmp4x],
    HRTIM_TIMC: [cmp1cr, cmp2cr, cmp3cr, cmp4cr, cmp1x, cmp2x, cmp3x, cmp4x],
    HRTIM_TIMD: [cmp1dr, cmp2dr, cmp3dr, cmp4dr, cmp1x, cmp2x, cmp3x, cmp4x],
    HRTIM_TIME: [cmp1er, cmp2er, cmp3er, cmp4er, cmp1x, cmp2x, cmp3x, cmp4x],
    HRTIM_TIMF: [cmp1fr, cmp2fr, cmp3fr, cmp4fr, cmp1x, cmp2x, cmp3x, cmp4x],
}

hrtim_hal! {
    HRTIM_TIMA: (timacr, timacr2, perar, tacen, repar, repx, timadier, repie, fltar, outar, dtar),
    HRTIM_TIMB: (timbcr, timbcr2, perbr, tbcen, repbr, repx, timbdier, repie, fltbr, outbr, dtbr),
    HRTIM_TIMC: (timccr, timccr2, percr, tccen, repcr, repx, timcdier, repie, fltcr, outcr, dtcr),
    HRTIM_TIMD: (timdcr, timdcr2, perdr, tdcen, repdr, repx, timddier, repie, fltdr, outdr, dtdr),
    HRTIM_TIME: (timecr, timecr2, perer, tecen, reper, repx, timedier, repie, flter, outer, dter),
    HRTIM_TIMF: (timfcr, timfcr2, perfr, tfcen, repfr, repx, timfdier, repie, fltfr, outfr, dtfr),
}

hrtim_hal_master! {
    HRTIM_MASTER: (mcr, ck_psc, mper, mper, mrep, mcen, mdier, mrepie),
}

hrtim_pin_hal! {
    HRTIM_TIMA: (CH1, perar, cmp1ar, cmp1x, cmp1, ta1oen, ta1odis),
    HRTIM_TIMA: (CH2, perar, cmp3ar, cmp3x, cmp3, ta2oen, ta2odis),

    HRTIM_TIMB: (CH1, perbr, cmp1br, cmp1x, cmp1, tb1oen, tb1odis),
    HRTIM_TIMB: (CH2, perbr, cmp3br, cmp3x, cmp3, tb2oen, tb2odis),

    HRTIM_TIMC: (CH1, percr, cmp1cr, cmp1x, cmp1, tc1oen, tc1odis),
    HRTIM_TIMC: (CH2, percr, cmp3cr, cmp3x, cmp3, tc2oen, tc2odis),

    HRTIM_TIMD: (CH1, perdr, cmp1dr, cmp1x, cmp1, td1oen, td1odis),
    HRTIM_TIMD: (CH2, perdr, cmp3dr, cmp3x, cmp3, td2oen, td2odis),

    HRTIM_TIME: (CH1, perer, cmp1er, cmp1x, cmp1, te1oen, te1odis),
    HRTIM_TIME: (CH2, perer, cmp3er, cmp3x, cmp3, te2oen, te2odis),

    HRTIM_TIMF: (CH1, perfr, cmp1fr, cmp1x, cmp1, tf1oen, tf1odis),
    HRTIM_TIMF: (CH2, perfr, cmp3fr, cmp3x, cmp3, tf2oen, tf2odis),
}

pub unsafe trait HrtimPrescaler: Default {
    const BITS: u8;
    const VALUE: u8;

    /// Minimum allowed value for compare registers used with the timer with this prescaler
    ///
    /// NOTE: That for CR1 and CR3, 0 is also allowed
    const MIN_CR: u16;

    /// Maximum allowed value for compare registers used with the timer with this prescaler
    const MAX_CR: u16;
}

macro_rules! impl_pscl {
    ($($t:ident => $b:literal, $v:literal, $min:literal, $max:literal)+) => {$(
        #[derive(Copy, Clone, Default)]
        pub struct $t;
        unsafe impl HrtimPrescaler for $t {
            const BITS: u8 = $b;
            const VALUE: u8 = $v;
            const MIN_CR: u16 = $min;
            const MAX_CR: u16 = $max;
        }
    )+};
}

impl_pscl! {
    Pscl1   => 0b000,   1, 0x0060, 0xFFDF
    Pscl2   => 0b001,   2, 0x0030, 0xFFEF
    Pscl4   => 0b010,   4, 0x0018, 0xFFF7
    Pscl8   => 0b011,   8, 0x000C, 0xFFFB
    Pscl16  => 0b100,  16, 0x0006, 0xFFFD
    Pscl32  => 0b101,  32, 0x0003, 0xFFFD
    Pscl64  => 0b110,  64, 0x0003, 0xFFFD
    Pscl128 => 0b111, 128, 0x0003, 0xFFFD
}

/// HrTim timer
struct TimerHrTim<PSC>(PhantomData<PSC>);

impl<PSC: HrtimPrescaler> super::TimerType for TimerHrTim<PSC> {
    // Period calculator for 16-bit hrtimers
    //
    // NOTE: This function will panic if the calculated period can not fit into 16 bits
    fn calculate_frequency(base_freq: HertzU64, freq: Hertz, alignment: Alignment) -> (u32, u16) {
        let ideal_period = super::Timer32Bit::calculate_frequency(base_freq, freq, alignment).0 + 1;

        let prescale = u32::from(PSC::VALUE);

        // Round to the nearest period
        let period = (ideal_period + (prescale >> 1)) / prescale - 1;

        // It IS possible to fail this assert
        assert!(period <= 0xFFFF);

        (period, PSC::BITS.into())
    }
}

pub trait HrtimChannel<PSCL> {}

impl<PSCL> HrtimChannel<PSCL> for CH1<PSCL> {}
impl<PSCL> HrtimChannel<PSCL> for CH2<PSCL> {}