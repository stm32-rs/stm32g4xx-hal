use core::marker::PhantomData;
use core::mem::MaybeUninit;

use fugit::HertzU64;

use crate::comparator::{COMP1, COMP2, COMP3, COMP4, COMP5, COMP6};
use crate::gpio::gpioa::{PA10, PA11, PA12, PA13, PA15, PA8, PA9};
use crate::gpio::gpiob::{PB0, PB10, PB11, PB14, PB15};
use crate::gpio::gpioc::{PC10, PC6, PC7, PC8, PC9};
use crate::gpio::{self, AF3};
use crate::gpio::{Alternate, AF13};
use crate::stm32::{
    HRTIM_COMMON, HRTIM_MASTER, HRTIM_TIMA, HRTIM_TIMB, HRTIM_TIMC, HRTIM_TIMD, HRTIM_TIME,
    HRTIM_TIMF,
};

use super::{
    ActiveHigh, Alignment, ComplementaryImpossible, FaultMonitor, Pins, Pwm, PwmPinEnable,
    TimerType,
};
use crate::rcc::{Enable, GetBusFreq, Rcc, Reset};
use crate::stm32::RCC;
use crate::time::Hertz;

pub struct CH1<PSCL>(PhantomData<PSCL>);
pub struct CH2<PSCL>(PhantomData<PSCL>);

/// Internal enum that keeps track of the count settings before PWM is finalized
enum CountSettings {
    Frequency(Hertz),
    Period(u16),
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

            impl ToHrOut for $CH1 {
                type Out = HrOut1<$TIMX>;
            }

            impl ToHrOut for $CH2 {
                type Out = HrOut2<$TIMX>;
            }
        )+
    }
}

pins! {
    HRTIM_TIMA: CH1: PA8<Alternate<AF13>>, CH2: PA9<Alternate<AF13>>

    HRTIM_TIMB: CH1: PA10<Alternate<AF13>>, CH2: PA11<Alternate<AF13>>
    HRTIM_TIMC: CH1: PA12<Alternate<AF13>>, CH2: PA13<Alternate<AF13>>
    HRTIM_TIMD: CH1: PB14<Alternate<AF13>>, CH2: PB15<Alternate<AF13>>

    HRTIM_TIME: CH1: PC8<Alternate<AF3>>, CH2: PC9<Alternate<AF3>>
    HRTIM_TIMF: CH1: PC6<Alternate<AF13>>, CH2: PC7<Alternate<AF13>>
}

impl Pins<HRTIM_MASTER, (), ComplementaryImpossible> for () {
    type Channel = ();
}

impl ToHrOut for () {
    type Out = ();
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
    ) -> HrPwmBuilder<Self, Pscl128, Self::PreloadSource, PINS::Out>
    where
        PINS: Pins<Self, CHANNEL, COMP> + ToHrOut,
        CHANNEL: HrtimChannel<Pscl128>;
}

/// HrPwmBuilder is used to configure advanced HrTim PWM features
pub struct HrPwmBuilder<TIM, PSCL, PS, OUT> {
    _tim: PhantomData<TIM>,
    _prescaler: PhantomData<PSCL>,
    _out: PhantomData<OUT>,
    alignment: Alignment,
    base_freq: HertzU64,
    count: CountSettings,
    preload_source: Option<PS>,
    fault_enable_bits: u8,
    fault1_bits: u8,
    fault2_bits: u8,
    enable_push_pull: bool,
    //deadtime: NanoSecond,
}

pub enum PreloadSource {
    /// Preloaded registers are updated on counter roll over or counter reset
    OnCounterReset,
}

pub enum MasterPreloadSource {
    /// Prealoaded registers are updaten when the master counter rolls over and the master repetition counter is 0
    OnMasterRepetitionUpdate,
}

pub trait HrCompareRegister {
    fn get_duty(&self) -> u16;
    fn set_duty(&mut self, duty: u16);
}

pub struct HrCr1<TIM>(PhantomData<TIM>);
pub struct HrCr2<TIM>(PhantomData<TIM>);
pub struct HrCr3<TIM>(PhantomData<TIM>);
pub struct HrCr4<TIM>(PhantomData<TIM>);

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
}

pub trait HrOutput {
    /// Enable this output
    fn enable(&mut self);

    /// Disable this output
    fn disable(&mut self);

    /// Set this output to active every time the specified event occurs
    ///
    /// NOTE: Enabling the same event for both SET and RESET
    /// will make that event TOGGLE the output
    fn enable_set_event(&mut self, set_event: EventSource);

    /// Stop listening to the specified event
    fn disable_set_event(&mut self, set_event: EventSource);

    /// Set this output to *not* active every time the specified event occurs
    ///
    /// NOTE: Enabling the same event for both SET and RESET
    /// will make that event TOGGLE the output
    fn enable_rst_event(&mut self, reset_event: EventSource);

    /// Stop listening to the specified event
    fn disable_rst_event(&mut self, reset_event: EventSource);

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

pub enum EventSource {
    /// Compare match with compare register 1 of this timer
    Cr1,

    /// Compare match with compare register 2 of this timer
    Cr2,

    /// Compare match with compare register 3 of this timer
    Cr3,

    /// Compare match with compare register 4 of this timer
    Cr4,

    /// On complete period
    Period,

    /// Compare match with compare register 1 of master timer
    MasterCr1,

    /// Compare match with compare register 2 of master timer
    MasterCr2,

    /// Compare match with compare register 3 of master timer
    MasterCr3,

    /// Compare match with compare register 4 of master timer
    MasterCr4,

    /// On complete master period
    MasterPeriod,
    // TODO: These are unique for every timer output
    //Extra(E)
}
pub trait ToHrOut {
    type Out;
}

impl<PA, PB> ToHrOut for (PA, PB)
where
    PA: ToHrOut,
    PB: ToHrOut,
{
    type Out = (PA::Out, PB::Out);
}

pub struct HrOut1<TIM>(PhantomData<TIM>);
pub struct HrOut2<TIM>(PhantomData<TIM>);

macro_rules! hrtim_finalize_body {
    ($this:expr, $PreloadSource:ident, $TIMX:ident: (
        $timXcr:ident, $ck_psc:ident, $perXr:ident, $perx:ident, $tXcen:ident $(, $fltXr:ident, $outXr:ident)*),
    ) => {{
        let tim = unsafe { &*$TIMX::ptr() };
        let (period, prescaler_bits) = match $this.count {
            CountSettings::Period(period) => (period as u32, PSCL::BITS as u16),
            CountSettings::Frequency( freq ) => {
                <TimerHrTim<PSCL>>::calculate_frequency($this.base_freq, freq, $this.alignment)
            },
        };

        // Write prescaler and any special modes
        tim.$timXcr.modify(|_r, w| unsafe {
            w
                // Enable Continous mode
                .cont().set_bit()

                // TODO: add support for more modes

                // Set prescaler
                .$ck_psc().bits(prescaler_bits as u8)
        });

        $(
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

            // Set actions on fault for both outputs
            tim.$outXr.modify(|_r, w| w
                .fault1().bits($this.fault1_bits)
                .fault2().bits($this.fault2_bits)
            );
        })*


        hrtim_finalize_body!($PreloadSource, $this, tim, $timXcr);

        // Start timer
        let master = unsafe { &*HRTIM_MASTER::ptr() };
        master.mcr.modify(|_r, w| { w.$tXcen().set_bit() });

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
        pub fn prescaler<P>(self, _prescaler: P) -> HrPwmBuilder<$TIMX, P, $PS, OUT>
        where
            P: HrtimPrescaler,
        {
            let HrPwmBuilder {
                _tim,
                _prescaler: _,
                _out,
                fault_enable_bits,
                fault1_bits,
                fault2_bits,
                enable_push_pull,
                alignment,
                base_freq,
                count,
                preload_source,
            } = self;

            let period = match count {
                CountSettings::Frequency(_) => u16::MAX,
                CountSettings::Period(period) => period,
            };

            let count = CountSettings::Period(period);

            HrPwmBuilder {
                _tim,
                _prescaler: PhantomData,
                _out,
                fault_enable_bits,
                fault1_bits,
                fault2_bits,
                enable_push_pull,
                alignment,
                base_freq,
                count,
                preload_source,
                //deadtime: 0.nanos(),
            }
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
    };
}

// Implement PWM configuration for timer
macro_rules! hrtim_hal {
    ($($TIMX:ident: ($timXcr:ident, $perXr:ident, $tXcen:ident, $fltXr:ident, $outXr:ident),)+) => {
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
                ) -> HrPwmBuilder<Self, Pscl128, Self::PreloadSource, PINS::Out>
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
                        fault_enable_bits: 0b000000,
                        fault1_bits: 0b00,
                        fault2_bits: 0b00,
                        alignment: Alignment::Left,
                        base_freq: clk,
                        count: CountSettings::Period(u16::MAX),
                        preload_source: None,
                        enable_push_pull: false,
                        //deadtime: 0.nanos(),
                    }
                }
            }

            impl<PSCL, OUT>
                HrPwmBuilder<$TIMX, PSCL, PreloadSource, OUT>
            where
                PSCL: HrtimPrescaler,
            {
                pub fn finalize(self, _control: &mut HrPwmControl) -> (HrTim<$TIMX, PSCL>, (HrCr1<$TIMX>, HrCr2<$TIMX>, HrCr3<$TIMX>, HrCr4<$TIMX>), OUT) {
                    hrtim_finalize_body!(self, PreloadSource, $TIMX: ($timXcr, ck_pscx, $perXr, perx, $tXcen, $fltXr, $outXr),)
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
                    self.enable_push_pull = enable;

                    self
                }

                //pub fn half_mode(mut self, enable: bool) -> Self

                //pub fn interleaved_mode(mut self, mode: _) -> Self

                //pub fn swap_mode(mut self, enable: bool) -> Self
            }
        )+
    };
}

macro_rules! hrtim_hal_master {
    ($($TIMX:ident: ($timXcr:ident, $ck_psc:ident, $perXr:ident, $perx:ident, $tXcen:ident),)+) => {$(
        impl HrPwmAdvExt for $TIMX {
            type PreloadSource = MasterPreloadSource;

            fn pwm_advanced<PINS, CHANNEL, COMP>(
                self,
                _pins: PINS,
                rcc: &mut Rcc,
            ) -> HrPwmBuilder<Self, Pscl128, Self::PreloadSource, PINS::Out>
            where
                PINS: /*Pins<Self, CHANNEL, COMP> +*/ ToHrOut,
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
                    fault_enable_bits: 0b000000,
                    fault1_bits: 0b00,
                    fault2_bits: 0b00,
                    alignment: Alignment::Left,
                    base_freq: clk,
                    count: CountSettings::Period(u16::MAX),
                    preload_source: None,
                    enable_push_pull: false,
                    //deadtime: 0.nanos(),
                }
            }
        }

        impl<PSCL, OUT>
            HrPwmBuilder<$TIMX, PSCL, MasterPreloadSource, OUT>
        where
            PSCL: HrtimPrescaler,
        {
            pub fn finalize(self, _control: &mut HrPwmControl) -> (HrTim<$TIMX, PSCL>, (HrCr1<$TIMX>, HrCr2<$TIMX>, HrCr3<$TIMX>, HrCr4<$TIMX>)) {
                hrtim_finalize_body!(self, MasterPreloadSource, $TIMX: ($timXcr, $ck_psc, $perXr, $perx, $tXcen),)
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
                /// NOTE: Please observe limits:
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
                /// Also, writing 0 as duty is only valid for CR1 and CR3
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
    ($TIMX:ident, $set_event:ident, $register:ident, $action:ident) => {{
        let tim = unsafe { &*$TIMX::ptr() };

        match $set_event {
            EventSource::Cr1 => tim.$register.modify(|_r, w| w.cmp1().$action()),
            EventSource::Cr2 => tim.$register.modify(|_r, w| w.cmp2().$action()),
            EventSource::Cr3 => tim.$register.modify(|_r, w| w.cmp3().$action()),
            EventSource::Cr4 => tim.$register.modify(|_r, w| w.cmp4().$action()),
            EventSource::Period => tim.$register.modify(|_r, w| w.per().$action()),

            EventSource::MasterCr1 => tim.$register.modify(|_r, w| w.mstcmp1().$action()),
            EventSource::MasterCr2 => tim.$register.modify(|_r, w| w.mstcmp2().$action()),
            EventSource::MasterCr3 => tim.$register.modify(|_r, w| w.mstcmp3().$action()),
            EventSource::MasterCr4 => tim.$register.modify(|_r, w| w.mstcmp4().$action()),
            EventSource::MasterPeriod => tim.$register.modify(|_r, w| w.mstper().$action()),
        }
    }};
}

macro_rules! hrtim_out {
    ($($TIMX:ident: $out_type:ident: $tXYoen:ident, $tXYodis:ident, $tXYods:ident, $setXYr:ident, $rstXYr:ident,)+) => {$(
        impl HrOutput for $out_type<$TIMX> {
            fn enable(&mut self) {
                let common = unsafe { &*HRTIM_COMMON::ptr() };
                common.oenr.write(|w| { w.$tXYoen().set_bit() });
            }

            fn disable(&mut self) {
                let common = unsafe { &*HRTIM_COMMON::ptr() };
                common.odisr.write(|w| { w.$tXYodis().set_bit() });
            }

            fn enable_set_event(&mut self, set_event: EventSource) {
                hrtim_out_common!($TIMX, set_event, $setXYr, set_bit)
            }
            fn disable_set_event(&mut self, set_event: EventSource) {
                hrtim_out_common!($TIMX, set_event, $setXYr, clear_bit)
            }

            fn enable_rst_event(&mut self, reset_event: EventSource) {
                hrtim_out_common!($TIMX, reset_event, $rstXYr, set_bit)
            }
            fn disable_rst_event(&mut self, reset_event: EventSource) {
                hrtim_out_common!($TIMX, reset_event, $rstXYr, clear_bit)
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
        impl HrCompareRegister for $cr_type<$TIMX> {
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
    ($($TIMX:ident: $perXr:ident, $perx:ident,)+) => {$(
        impl<PSCL> HrTimer<$TIMX, PSCL> for HrTim<$TIMX, PSCL> {
            fn get_period(&self) -> u16 {
                let tim = unsafe { &*$TIMX::ptr() };

                tim.$perXr.read().$perx().bits()
            }
            fn set_period(&mut self, period: u16) {
                let tim = unsafe { &*$TIMX::ptr() };

                tim.$perXr.write(|w| unsafe { w.$perx().bits(period as u16) });
            }
        }
    )+};
}

hrtim_timer! {
    HRTIM_MASTER: mper, mper,

    HRTIM_TIMA: perar, perx,
    HRTIM_TIMB: perbr, perx,
    HRTIM_TIMC: percr, perx,
    HRTIM_TIMD: perdr, perx,
    HRTIM_TIME: perer, perx,
    HRTIM_TIMF: perfr, perx,
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
    HRTIM_TIMA: (timacr, perar, tacen, fltar, outar),
    HRTIM_TIMB: (timbcr, perbr, tbcen, fltbr, outbr),
    HRTIM_TIMC: (timccr, percr, tccen, fltcr, outcr),
    HRTIM_TIMD: (timdcr, perdr, tdcen, fltdr, outdr),
    HRTIM_TIME: (timecr, perer, tecen, flter, outer),
    HRTIM_TIMF: (timfcr, perfr, tfcen, fltfr, outfr),
}

hrtim_hal_master! {
    HRTIM_MASTER: (mcr, ck_psc, mper, mper, mcen),
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

pub trait HrtimPrescaler {
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
        impl HrtimPrescaler for $t {
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

pub enum FaultAction {
    /// Output never enters fault mode
    None = 0b00,

    /// Output forced to `active` level on fault
    ForceActive = 0b01,

    /// Output forced to `inactive` level on fault
    ForceInactive = 0b10,

    /// The output is floating/tri stated on fault
    Floating = 0b11,
}

pub trait FaultSource: Copy {
    const ENABLE_BITS: u8;
}

pub struct SourceBuilder<I> {
    _input: I,
    src_bits: u8,

    /// FLTxP
    is_active_high: bool,

    /// FLTxF[3:0]
    filter_bits: u8,
}

impl<I> SourceBuilder<I> {
    unsafe fn new(input: I, src_bits: u8) -> Self {
        SourceBuilder {
            _input: input,
            src_bits,
            is_active_high: false,
            filter_bits: 0b0000,
        }
    }
}

macro_rules! impl_faults {
    ($(
        $input:ident => $source:ident:
            PINS=[($pin:ident, $af:ident), $(($pin_b:ident, $af_b:ident),)*],
            COMP=$compX:ident, $enable_bits:literal,
            $fltinrZ:ident, $fltWsrc_0:ident, $fltWsrc_1:ident, $fltWp:ident, $fltWf:ident, $fltWe:ident, $fltWlck:ident,
    )+) => {$(

        // This should NOT be Copy/Clone
        pub struct $input {
            _x: PhantomData<()>
        }

        #[derive(Copy, Clone)]
        pub struct $source {
            _x: PhantomData<()>
        }

        impl $input {
            pub fn bind_pin<IM>(self, pin: $pin<gpio::Input<IM>>) -> SourceBuilder<$input> {
                pin.into_alternate::<$af>();
                unsafe { SourceBuilder::new(self, 0b00) }
            }

            $(
                pub fn bind_pin_b<IM>(self, pin: $pin_b<gpio::Input<IM>>) -> SourceBuilder<$input> {
                    pin.into_alternate::<$af_b>();
                    unsafe { SourceBuilder::new(self, 0b00) }
                }
            )*

            pub fn bind_comp(self, _comp: &crate::comparator::Comparator<$compX, crate::comparator::Enabled>) -> SourceBuilder<$input> {
                unsafe { SourceBuilder::new(self, 0b01) }
            }

            /*pub fn bind_external(?) {
                SourceBuilder::new(self, 0b10);
            }*/
        }

        impl SourceBuilder<$input> {
            pub fn finalize(self, _control: &mut HrPwmControl) -> $source {
                let SourceBuilder{ _input, src_bits, is_active_high, filter_bits } = self;

                // Setup fault source
                unsafe {
                    let common = &*HRTIM_COMMON::ptr();

                    common.fltinr2.modify(|_r, w| w.$fltWsrc_1().bit(src_bits & 0b10 != 0));
                    common.$fltinrZ.modify(|_r, w| w
                        .$fltWsrc_0().bit(src_bits & 0b01 != 0)
                        .$fltWp().bit(is_active_high)
                        .$fltWf().bits(filter_bits)
                        .$fltWe().set_bit() // Enable
                    );

                    // ... and lock configuration
                    common.$fltinrZ.modify(|_r, w| w.$fltWlck().set_bit());
                }

                $source {
                    _x: PhantomData
                }
            }

            pub fn polarity(mut self, polarity: super::Polarity) -> Self {
                self.is_active_high = polarity == super::Polarity::ActiveHigh;
                self
            }

            // TODO: add more settings
            /* pub fn blanking(?) -> Self */

            pub fn filter(mut self, filter: FaultSamplingFilter) -> Self {
                self.filter_bits = filter as u8;
                self
            }
        }

        impl FaultSource for $source {
            const ENABLE_BITS: u8 = $enable_bits;
        }
    )+}
}

// TODO: Lookup to ensure the alternate function are the same for other devices than stm32g474
#[cfg(feature = "stm32g474")]
impl_faults!(
    FaultInput1 => FaultSource1: PINS=[(PA12, AF13),], COMP=COMP2, 0b000001, fltinr1, flt1src, flt1src_1, flt1p, flt1f, flt1e, flt1lck,
    FaultInput2 => FaultSource2: PINS=[(PA15, AF13),], COMP=COMP4, 0b000010, fltinr1, flt2src, flt2src_1, flt2p, flt2f, flt2e, flt2lck,
    FaultInput3 => FaultSource3: PINS=[(PB10, AF13),], COMP=COMP6, 0b000100, fltinr1, flt3src, flt3src_1, flt3p, flt3f, flt3e, flt3lck,
    FaultInput4 => FaultSource4: PINS=[(PB11, AF13),], COMP=COMP1, 0b001000, fltinr1, flt4src, flt4src_1, flt4p, flt4f, flt4e, flt4lck,
    FaultInput5 => FaultSource5: PINS=[(PB0, AF13), (PC7, AF3),], COMP=COMP3, 0b010000, fltinr2, flt5src, flt5src_1, flt5p, flt5f, flt5e, flt5lck,
    FaultInput6 => FaultSource6: PINS=[(PC10, AF13),], COMP=COMP5, 0b100000, fltinr2, flt6src_0, flt6src_1, flt6p, flt6f, flt6e, flt6lck,
);

pub struct FaultInputs {
    pub fault_input1: FaultInput1,
    pub fault_input2: FaultInput2,
    pub fault_input3: FaultInput3,
    pub fault_input4: FaultInput4,
    pub fault_input5: FaultInput5,
    pub fault_input6: FaultInput6,
}

pub struct HrPwmControl {
    _x: PhantomData<()>,

    pub fault_sys: FltMonitorSys,
    pub fault_1: FltMonitor1,
    pub fault_2: FltMonitor2,
    pub fault_3: FltMonitor3,
    pub fault_4: FltMonitor4,
    pub fault_5: FltMonitor5,
    pub fault_6: FltMonitor6,
}

/// The divsion ratio between f_hrtim and the fault signal sampling clock for digital filters
pub enum FaultSamplingClkDiv {
    /// No division
    ///
    /// fault signal sampling clock f_flts = f_hrtim
    None = 0b00,

    /// 1/2
    ///
    /// fault signal sampling clock f_flts = f_hrtim / 2
    Two = 0b01,

    /// 1/4
    ///
    /// fault signal sampling clock f_flts = f_hrtim / 4
    Four = 0b10,

    /// 1/8
    ///
    /// fault signal sampling clock f_flts = f_hrtim / 8
    Eight = 0b11,
}

pub enum FaultSamplingFilter {
    /// No filtering, fault acts asynchronously
    ///
    /// Note that this bypasses any f_flts (FaultSamplingClkDiv)
    None = 0b0000,

    /// Sample directly at rate f_hrtim, with a count of 2
    ///
    /// Note that this bypasses: any f_flts (FaultSamplingClkDiv)
    HrtimN2 = 0b0001,

    /// Sample directly at rate f_hrtim, with a count of 4
    ///
    /// Note that this bypasses any f_flts (FaultSamplingClkDiv)
    HrtimN4 = 0b0010,

    /// Sample directly at rate f_hrtim, with a count of 8
    ///
    /// Note that this bypasses any f_flts (FaultSamplingClkDiv)
    HrtimN8 = 0b0011,

    /// Sample at rate f_flts / 2, with a count of 6
    FltsDiv2N6 = 0b0100,

    /// Sample at rate f_flts / 2, with a count of 8
    FltsDiv2N8 = 0b0101,

    /// Sample at rate f_flts / 4, with a count of 6
    FltsDiv4N6 = 0b0110,

    /// Sample at rate f_flts / 4, with a count of 8
    FltsDiv4N8 = 0b0111,

    /// Sample at rate f_flts / 8, with a count of 6
    FltsDiv8N6 = 0b1000,

    /// Sample at rate f_flts / 8, with a count of 8
    FltsDiv8N8 = 0b1001,

    /// Sample at rate f_flts / 16, with a count of 5
    FltsDiv16N5 = 0b1010,

    /// Sample at rate f_flts / 16, with a count of 6
    FltsDiv16N6 = 0b1011,

    /// Sample at rate f_flts / 16, with a count of 8
    FltsDiv16N8 = 0b1100,

    /// Sample at rate f_flts / 32, with a count of 5
    FltsDiv32N5 = 0b1101,

    /// Sample at rate f_flts / 32, with a count of 6
    FltsDiv32N6 = 0b1110,

    /// Sample at rate f_flts / 32, with a count of 8
    FltsDiv32N8 = 0b1111,
}

pub trait HrControltExt {
    fn hr_control(self, _rcc: &mut Rcc) -> HrTimOngoingCalibration;
}

impl HrControltExt for HRTIM_COMMON {
    fn hr_control(self, _rcc: &mut Rcc) -> HrTimOngoingCalibration {
        let common = unsafe { &*HRTIM_COMMON::ptr() };

        unsafe {
            let rcc_ptr = &*RCC::ptr();

            HRTIM_COMMON::enable(rcc_ptr);
            HRTIM_COMMON::reset(rcc_ptr);
        }

        // Start calibration procedure
        common
            .dllcr
            .write(|w| w.cal().set_bit().calen().clear_bit());

        HrTimOngoingCalibration {
            adc_trigger1_bits: 0,
            adc_trigger2_bits: 0,
            adc_trigger3_bits: 0,
            adc_trigger4_bits: 0,

            adc_trigger5_bits: 0,
            adc_trigger6_bits: 0,
            adc_trigger7_bits: 0,
            adc_trigger8_bits: 0,
            adc_trigger9_bits: 0,
            adc_trigger10_bits: 0,

            adc_trigger1_postscaler: AdcTriggerPostscaler::None,
            adc_trigger2_postscaler: AdcTriggerPostscaler::None,
            adc_trigger3_postscaler: AdcTriggerPostscaler::None,
            adc_trigger4_postscaler: AdcTriggerPostscaler::None,

            adc_trigger5_postscaler: AdcTriggerPostscaler::None,
            adc_trigger6_postscaler: AdcTriggerPostscaler::None,
            adc_trigger7_postscaler: AdcTriggerPostscaler::None,
            adc_trigger8_postscaler: AdcTriggerPostscaler::None,
            adc_trigger9_postscaler: AdcTriggerPostscaler::None,
            adc_trigger10_postscaler: AdcTriggerPostscaler::None,

            divider: FaultSamplingClkDiv::None,
        }
    }
}

pub struct HrTimOngoingCalibration {
    adc_trigger1_bits: u32,
    adc_trigger2_bits: u32,
    adc_trigger3_bits: u32,
    adc_trigger4_bits: u32,

    adc_trigger5_bits: u8,
    adc_trigger6_bits: u8,
    adc_trigger7_bits: u8,
    adc_trigger8_bits: u8,
    adc_trigger9_bits: u8,
    adc_trigger10_bits: u8,

    adc_trigger1_postscaler: AdcTriggerPostscaler,
    adc_trigger2_postscaler: AdcTriggerPostscaler,
    adc_trigger3_postscaler: AdcTriggerPostscaler,
    adc_trigger4_postscaler: AdcTriggerPostscaler,

    adc_trigger5_postscaler: AdcTriggerPostscaler,
    adc_trigger6_postscaler: AdcTriggerPostscaler,
    adc_trigger7_postscaler: AdcTriggerPostscaler,
    adc_trigger8_postscaler: AdcTriggerPostscaler,
    adc_trigger9_postscaler: AdcTriggerPostscaler,
    adc_trigger10_postscaler: AdcTriggerPostscaler,

    divider: FaultSamplingClkDiv,
}

impl HrTimOngoingCalibration {
    /// SAFETY: Calibration needs to be done before calling this
    unsafe fn init(self) -> (HrPwmControl, FaultInputs) {
        use Adc13Trigger as Ad13T;
        use Adc24Trigger as Ad24T;

        let common = unsafe { &*HRTIM_COMMON::ptr() };

        let Self {
            adc_trigger1_bits: ad1_bits,
            adc_trigger2_bits: ad2_bits,
            adc_trigger3_bits: ad3_bits,
            adc_trigger4_bits: ad4_bits,

            adc_trigger5_bits: ad5_bits,
            adc_trigger6_bits: ad6_bits,
            adc_trigger7_bits: ad7_bits,
            adc_trigger8_bits: ad8_bits,
            adc_trigger9_bits: ad9_bits,
            adc_trigger10_bits: ad10_bits,

            adc_trigger1_postscaler,
            adc_trigger2_postscaler,
            adc_trigger3_postscaler,
            adc_trigger4_postscaler,

            adc_trigger5_postscaler,
            adc_trigger6_postscaler,
            adc_trigger7_postscaler,
            adc_trigger8_postscaler,
            adc_trigger9_postscaler,
            adc_trigger10_postscaler,

            divider,
        } = self;

        unsafe {
            // Enable periodic calibration
            // with f_hrtim at 170MHz, these settings leads to
            // a period of about 6.2ms
            common
                .dllcr
                .modify(|_r, w| w.calrte().bits(0b00).cal().set_bit().calen().clear_bit());
            common.fltinr2.write(|w| w.fltsd().bits(divider as u8));

            common.adc1r.write(|w| {
                w.eper()
                    .bit(ad1_bits | Ad13T::TimEPeriod as u32 != 0)
                    .ec4()
                    .bit(ad1_bits | Ad13T::TimECmp4 as u32 != 0)
                    .ec3()
                    .bit(ad1_bits | Ad13T::TimECmp3 as u32 != 0)
                    //.frst()
                    .dper()
                    .bit(ad1_bits | Ad13T::TimDPeriod as u32 != 0)
                    .dc4()
                    .bit(ad1_bits | Ad13T::TimDCmp4 as u32 != 0)
                    .dc3()
                    .bit(ad1_bits | Ad13T::TimDCmp3 as u32 != 0)
                    .fper()
                    .bit(ad1_bits | Ad13T::TimFPeriod as u32 != 0)
                    .cper()
                    .bit(ad1_bits | Ad13T::TimCPeriod as u32 != 0)
                    .cc4()
                    .bit(ad1_bits | Ad13T::TimCCmp4 as u32 != 0)
                    .cc3()
                    .bit(ad1_bits | Ad13T::TimCCmp3 as u32 != 0)
                    .fc4()
                    .bit(ad1_bits | Ad13T::TimFCmp4 as u32 != 0)
                    //.brst()
                    .bper()
                    .bit(ad1_bits | Ad13T::TimBPeriod as u32 != 0)
                    .bc4()
                    .bit(ad1_bits | Ad13T::TimBCmp4 as u32 != 0)
                    .bc3()
                    .bit(ad1_bits | Ad13T::TimBCmp3 as u32 != 0)
                    .fc3()
                    .bit(ad1_bits | Ad13T::TimFCmp3 as u32 != 0)
                    //.arst()
                    .aper()
                    .bit(ad1_bits | Ad13T::TimAPeriod as u32 != 0)
                    .ac4()
                    .bit(ad1_bits | Ad13T::TimACmp4 as u32 != 0)
                    .ac3()
                    .bit(ad1_bits | Ad13T::TimACmp3 as u32 != 0)
                    .fc2()
                    .bit(ad1_bits | Ad13T::TimFCmp2 as u32 != 0)
                    //.eev5().bit(ad1_bits | Ad13T::_ as u32)
                    //.eev4().bit(ad1_bits | Ad13T::_ as u32)
                    //.eev3().bit(ad1_bits | Ad13T::_ as u32)
                    //.eev2().bit(ad1_bits | Ad13T::_ as u32)
                    //.eev1().bit(ad1_bits | Ad13T::_ as u32)
                    .mper()
                    .bit(ad1_bits | Ad13T::MasterPeriod as u32 != 0)
                    .mc4()
                    .bit(ad1_bits | Ad13T::MasterCmp4 as u32 != 0)
                    .mc3()
                    .bit(ad1_bits | Ad13T::MasterCmp3 as u32 != 0)
                    .mc2()
                    .bit(ad1_bits | Ad13T::MasterCmp2 as u32 != 0)
                    .mc1()
                    .bit(ad1_bits | Ad13T::MasterCmp1 as u32 != 0)
            });

            common.adc3r.write(|w| {
                w.eper()
                    .bit(ad3_bits | Ad13T::TimEPeriod as u32 != 0)
                    .ec4()
                    .bit(ad3_bits | Ad13T::TimECmp4 as u32 != 0)
                    .ec3()
                    .bit(ad3_bits | Ad13T::TimECmp3 as u32 != 0)
                    //.frst()
                    .dper()
                    .bit(ad3_bits | Ad13T::TimDPeriod as u32 != 0)
                    .dc4()
                    .bit(ad3_bits | Ad13T::TimDCmp4 as u32 != 0)
                    .dc3()
                    .bit(ad3_bits | Ad13T::TimDCmp3 as u32 != 0)
                    .fper()
                    .bit(ad3_bits | Ad13T::TimFPeriod as u32 != 0)
                    .cper()
                    .bit(ad3_bits | Ad13T::TimCPeriod as u32 != 0)
                    .cc4()
                    .bit(ad3_bits | Ad13T::TimCCmp4 as u32 != 0)
                    .cc3()
                    .bit(ad3_bits | Ad13T::TimCCmp3 as u32 != 0)
                    .fc4()
                    .bit(ad3_bits | Ad13T::TimFCmp4 as u32 != 0)
                    //.brst()
                    .bper()
                    .bit(ad3_bits | Ad13T::TimBPeriod as u32 != 0)
                    .bc4()
                    .bit(ad3_bits | Ad13T::TimBCmp4 as u32 != 0)
                    .bc3()
                    .bit(ad3_bits | Ad13T::TimBCmp3 as u32 != 0)
                    .fc3()
                    .bit(ad3_bits | Ad13T::TimFCmp3 as u32 != 0)
                    //.arst()
                    .aper()
                    .bit(ad3_bits | Ad13T::TimAPeriod as u32 != 0)
                    .ac4()
                    .bit(ad3_bits | Ad13T::TimACmp4 as u32 != 0)
                    .ac3()
                    .bit(ad3_bits | Ad13T::TimACmp3 as u32 != 0)
                    .fc2()
                    .bit(ad3_bits | Ad13T::TimFCmp2 as u32 != 0)
                    //.eev5().bit(ad3_bits | Ad13T::_ as u32)
                    //.eev4().bit(ad3_bits | Ad13T::_ as u32)
                    //.eev3().bit(ad3_bits | Ad13T::_ as u32)
                    //.eev2().bit(ad3_bits | Ad13T::_ as u32)
                    //.eev1().bit(ad3_bits | Ad13T::_ as u32)
                    .mper()
                    .bit(ad3_bits | Ad13T::MasterPeriod as u32 != 0)
                    .mc4()
                    .bit(ad3_bits | Ad13T::MasterCmp4 as u32 != 0)
                    .mc3()
                    .bit(ad3_bits | Ad13T::MasterCmp3 as u32 != 0)
                    .mc2()
                    .bit(ad3_bits | Ad13T::MasterCmp2 as u32 != 0)
                    .mc1()
                    .bit(ad3_bits | Ad13T::MasterCmp1 as u32 != 0)
            });

            common.adc2r.write(|w| {
                w
                    //.erst()
                    .ec4()
                    .bit(ad2_bits | Ad24T::TimECmp4 as u32 != 0)
                    .ec3()
                    .bit(ad2_bits | Ad24T::TimECmp3 as u32 != 0)
                    .ec2()
                    .bit(ad2_bits | Ad24T::TimECmp2 as u32 != 0)
                    //.drst().bit(ad2_bits | Ad24T::_ as u32 != 0)
                    .dper()
                    .bit(ad2_bits | Ad24T::TimDPeriod as u32 != 0)
                    .dc4()
                    .bit(ad2_bits | Ad24T::TimDCmp4 as u32 != 0)
                    .fper()
                    .bit(ad2_bits | Ad24T::TimFPeriod as u32 != 0)
                    .dc2()
                    .bit(ad2_bits | Ad24T::TimDCmp2 as u32 != 0)
                    //.crst().bit(ad2_bits | Ad24T::_ as u32 != 0)
                    .cper()
                    .bit(ad2_bits | Ad24T::TimCPeriod as u32 != 0)
                    .cc4()
                    .bit(ad2_bits | Ad24T::TimCCmp4 as u32 != 0)
                    .fc4()
                    .bit(ad2_bits | Ad24T::TimFCmp4 as u32 != 0)
                    .cc2()
                    .bit(ad2_bits | Ad24T::TimCCmp2 as u32 != 0)
                    .bper()
                    .bit(ad2_bits | Ad24T::TimBPeriod as u32 != 0)
                    .bc4()
                    .bit(ad2_bits | Ad24T::TimBCmp4 as u32 != 0)
                    .fc3()
                    .bit(ad2_bits | Ad24T::TimFCmp3 as u32 != 0)
                    .bc2()
                    .bit(ad2_bits | Ad24T::TimBCmp2 as u32 != 0)
                    .aper()
                    .bit(ad2_bits | Ad24T::TimAPeriod as u32 != 0)
                    .ac4()
                    .bit(ad2_bits | Ad24T::TimACmp4 as u32 != 0)
                    .fc2()
                    .bit(ad2_bits | Ad24T::TimFCmp2 as u32 != 0)
                    .ac2()
                    .bit(ad2_bits | Ad24T::TimACmp2 as u32 != 0)
                    //.eev10()
                    //.eev9()
                    //.eev8()
                    //.eev7()
                    //.eev6()
                    .mper()
                    .bit(ad2_bits | Ad24T::MasterPeriod as u32 != 0)
                    .mc4()
                    .bit(ad2_bits | Ad24T::MasterCmp4 as u32 != 0)
                    .mc3()
                    .bit(ad2_bits | Ad24T::MasterCmp3 as u32 != 0)
                    .mc2()
                    .bit(ad2_bits | Ad24T::MasterCmp2 as u32 != 0)
                    .mc1()
                    .bit(ad2_bits | Ad24T::MasterCmp1 as u32 != 0)
            });

            common.adc4r.write(|w| {
                w
                    //.erst()
                    .ec4()
                    .bit(ad4_bits | Ad24T::TimECmp4 as u32 != 0)
                    .ec3()
                    .bit(ad4_bits | Ad24T::TimECmp3 as u32 != 0)
                    .ec2()
                    .bit(ad4_bits | Ad24T::TimECmp2 as u32 != 0)
                    //.drst().bit(ad4_bits | Ad24T::_ as u32 != 0)
                    .dper()
                    .bit(ad4_bits | Ad24T::TimDPeriod as u32 != 0)
                    .dc4()
                    .bit(ad4_bits | Ad24T::TimDCmp4 as u32 != 0)
                    .fper()
                    .bit(ad4_bits | Ad24T::TimFPeriod as u32 != 0)
                    .dc2()
                    .bit(ad4_bits | Ad24T::TimDCmp2 as u32 != 0)
                    //.crst().bit(ad4_bits | Ad24T::_ as u32 != 0)
                    .cper()
                    .bit(ad4_bits | Ad24T::TimCPeriod as u32 != 0)
                    .cc4()
                    .bit(ad4_bits | Ad24T::TimCCmp4 as u32 != 0)
                    .fc4()
                    .bit(ad4_bits | Ad24T::TimFCmp4 as u32 != 0)
                    .cc2()
                    .bit(ad4_bits | Ad24T::TimCCmp2 as u32 != 0)
                    .bper()
                    .bit(ad4_bits | Ad24T::TimBPeriod as u32 != 0)
                    .bc4()
                    .bit(ad4_bits | Ad24T::TimBCmp4 as u32 != 0)
                    .fc3()
                    .bit(ad4_bits | Ad24T::TimFCmp3 as u32 != 0)
                    .bc2()
                    .bit(ad4_bits | Ad24T::TimBCmp2 as u32 != 0)
                    .aper()
                    .bit(ad4_bits | Ad24T::TimAPeriod as u32 != 0)
                    .ac4()
                    .bit(ad4_bits | Ad24T::TimACmp4 as u32 != 0)
                    .fc2()
                    .bit(ad4_bits | Ad24T::TimFCmp2 as u32 != 0)
                    .ac2()
                    .bit(ad4_bits | Ad24T::TimACmp2 as u32 != 0)
                    //.eev10()
                    //.eev9()
                    //.eev8()
                    //.eev7()
                    //.eev6()
                    .mper()
                    .bit(ad4_bits | Ad24T::MasterPeriod as u32 != 0)
                    .mc4()
                    .bit(ad4_bits | Ad24T::MasterCmp4 as u32 != 0)
                    .mc3()
                    .bit(ad4_bits | Ad24T::MasterCmp3 as u32 != 0)
                    .mc2()
                    .bit(ad4_bits | Ad24T::MasterCmp2 as u32 != 0)
                    .mc1()
                    .bit(ad4_bits | Ad24T::MasterCmp1 as u32 != 0)
            });

            common.adcer.write(|w| {
                w.adc5trg()
                    .variant(ad5_bits)
                    .adc6trg()
                    .variant(ad6_bits)
                    .adc7trg()
                    .variant(ad7_bits)
                    .adc8trg()
                    .variant(ad8_bits)
                    .adc9trg()
                    .variant(ad9_bits)
                    .adc10trg()
                    .variant(ad10_bits)
            });

            common.adcps1.write(|w| {
                w.adc1psc()
                    .bits(adc_trigger1_postscaler as u8)
                    .adc2psc()
                    .bits(adc_trigger2_postscaler as u8)
                    .adc3psc()
                    .bits(adc_trigger3_postscaler as u8)
                    .adc4psc()
                    .bits(adc_trigger4_postscaler as u8)
                    .adc5psc()
                    .bits(adc_trigger5_postscaler as u8)
            });

            common.adcps2.write(|w| {
                w.adc6psc()
                    .bits(adc_trigger6_postscaler as u8)
                    .adc7psc()
                    .bits(adc_trigger7_postscaler as u8)
                    .adc8psc()
                    .bits(adc_trigger8_postscaler as u8)
                    .adc9psc()
                    .bits(adc_trigger9_postscaler as u8)
                    .adc10psc()
                    .bits(adc_trigger10_postscaler as u8)
            });

            // TODO: Adc trigger 5-10
        }

        (
            HrPwmControl {
                _x: PhantomData,
                fault_sys: FltMonitorSys { _x: PhantomData },
                fault_1: FltMonitor1 { _x: PhantomData },
                fault_2: FltMonitor2 { _x: PhantomData },
                fault_3: FltMonitor3 { _x: PhantomData },
                fault_4: FltMonitor4 { _x: PhantomData },
                fault_5: FltMonitor5 { _x: PhantomData },
                fault_6: FltMonitor6 { _x: PhantomData },
            },
            FaultInputs {
                fault_input1: FaultInput1 { _x: PhantomData },
                fault_input2: FaultInput2 { _x: PhantomData },
                fault_input3: FaultInput3 { _x: PhantomData },
                fault_input4: FaultInput4 { _x: PhantomData },
                fault_input5: FaultInput5 { _x: PhantomData },
                fault_input6: FaultInput6 { _x: PhantomData },
            },
        )
    }

    pub fn wait_for_calibration(self) -> (HrPwmControl, FaultInputs) {
        let common = unsafe { &*HRTIM_COMMON::ptr() };
        while common.isr.read().dllrdy().bit_is_clear() {
            // Wait until ready
        }

        // Calibration is now done, it is safe to continue
        unsafe { self.init() }
    }

    pub fn set_adc_trigger1(mut self, trigger: Adc13Trigger) -> Self {
        self.adc_trigger1_bits |= trigger as u32;
        self
    }

    pub fn set_adc_trigger2(mut self, trigger: Adc24Trigger) -> Self {
        self.adc_trigger2_bits |= trigger as u32;
        self
    }

    pub fn set_adc_trigger3(mut self, trigger: Adc13Trigger) -> Self {
        self.adc_trigger3_bits |= trigger as u32;
        self
    }

    pub fn set_adc_trigger4(mut self, trigger: Adc24Trigger) -> Self {
        self.adc_trigger4_bits |= trigger as u32;
        self
    }

    pub fn set_adc_trigger5(mut self, trigger: Adc579Trigger) -> Self {
        self.adc_trigger5_bits = trigger as u8;
        self
    }

    pub fn set_adc_trigger6(mut self, trigger: Adc6810Trigger) -> Self {
        self.adc_trigger6_bits = trigger as u8;
        self
    }

    pub fn set_adc_trigger7(mut self, trigger: Adc579Trigger) -> Self {
        self.adc_trigger7_bits = trigger as u8;
        self
    }

    pub fn set_adc_trigger8(mut self, trigger: Adc6810Trigger) -> Self {
        self.adc_trigger8_bits = trigger as u8;
        self
    }

    pub fn set_adc_trigger9(mut self, trigger: Adc579Trigger) -> Self {
        self.adc_trigger9_bits = trigger as u8;
        self
    }

    pub fn set_adc_trigger10(mut self, trigger: Adc6810Trigger) -> Self {
        self.adc_trigger10_bits = trigger as u8;
        self
    }

    pub fn set_adc1_trigger_psc(mut self, post_scaler: AdcTriggerPostscaler) -> Self {
        self.adc_trigger1_postscaler = post_scaler;
        self
    }

    pub fn set_adc2_trigger_psc(mut self, post_scaler: AdcTriggerPostscaler) -> Self {
        self.adc_trigger2_postscaler = post_scaler;
        self
    }

    pub fn set_adc3_trigger_psc(mut self, post_scaler: AdcTriggerPostscaler) -> Self {
        self.adc_trigger3_postscaler = post_scaler;
        self
    }

    pub fn set_adc4_trigger_psc(mut self, post_scaler: AdcTriggerPostscaler) -> Self {
        self.adc_trigger4_postscaler = post_scaler;
        self
    }

    pub fn set_fault_sampling_division(mut self, divider: FaultSamplingClkDiv) -> Self {
        self.divider = divider;
        self
    }

    // TODO: Adc trigger 5-10
}

pub enum Adc13Trigger {
    /// bit 31 ADCxTEPER - Trigger on HRTIM_TIME period
    TimEPeriod = 1 << 31,

    /// bit 30 ADCxTEC4 - Trigger on HRTIM_TIME compare match for compare register 4
    TimECmp4 = 1 << 30,

    /// bit 29 ADCxTEC3 - Trigger on HRTIM_TIME compare match for compare register 3
    TimECmp3 = 1 << 29,

    // /// bit 28 ADCxTFRST
    // _ = 1 << 28,
    /// bit 27 ADCxTDPER - Trigger on HRTIM_TIMD period
    TimDPeriod = 1 << 27,

    /// bit 26 ADCxTDC4 - Trigger on HRTIM_TIMD compare match for compare register 4
    TimDCmp4 = 1 << 26,

    /// bit 25 ADCxTDC3 - Trigger on HRTIM_TIMD compare match for compare register 3
    TimDCmp3 = 1 << 25,

    /// bit 24 ADCxTFPER - Trigger on HRTIM_TIMF period
    TimFPeriod = 1 << 24,

    /// bit 23 ADCxTCPER - Trigger on HRTIM_TIMC period
    TimCPeriod = 1 << 23,

    /// bit 22 ADCxTCC4 - Trigger on HRTIM_TIMC compare match for compare register 4
    TimCCmp4 = 1 << 22,

    /// bit 21 ADCxTCC3 - Trigger on HRTIM_TIMC compare match for compare register 3
    TimCCmp3 = 1 << 21,

    /// bit 20 ADCxTFC4 - Trigger on HRTIM_TIMF compare match for compare register 4
    TimFCmp4 = 1 << 20,

    // /// bit 19 ADCxTBRST
    // _ = 1 << 19,
    /// bit 18 ADCxTBPER - Trigger on HRTIM_TIMB period
    TimBPeriod = 1 << 18,

    /// bit 17 ADCxTBC4 - Trigger on HRTIM_TIMB compare match for compare register 4
    TimBCmp4 = 1 << 17,

    /// bit 16 ADCxTBC3 - Trigger on HRTIM_TIMB compare match for compare register 3
    TimBCmp3 = 1 << 16,

    /// bit 15 ADCxTFC3 - Trigger on HRTIM_TIMF compare match for compare register 3
    TimFCmp3 = 1 << 15,

    // /// bit 14 ADCxTARST
    // _ = 1 << 14,
    /// bit 13 ADCxTAPER - Trigger on HRTIM_TIMA period
    TimAPeriod = 1 << 13,

    /// bit 12 ADCxTAC4 - Trigger on HRTIM_TIMA compare match for compare register 4
    TimACmp4 = 1 << 12,

    /// bit 11 ADCxTAC3 - Trigger on HRTIM_TIMA compare match for compare register 3
    TimACmp3 = 1 << 11,

    /// bit 10 ADCxTFC2 - Trigger on HRTIM_TIMF compare match for compare register 2
    TimFCmp2 = 1 << 10,

    // /// bit 9 ADCxEEV5
    // _ = 1 << 9,

    // /// bit 8 ADCxEEV4
    // _ = 1 << 8,

    // /// bit 7 ADCxEEV3
    // _ = 1 << 7,

    // /// bit 6 ADCxEEV2
    // _ = 1 << 6,
    /// bit 5 ADCxEEV1
    // _ = 1 << 5,

    /// bit 4 ADCxMPER - Trigger on HRTIM_MASTER period
    MasterPeriod = 1 << 4,

    /// bit 3 ADCxMC4 - Trigger on HRTIM_MASTER compare match for compare register 4
    MasterCmp4 = 1 << 3,

    /// bit 2 ADCxMC3 - Trigger on HRTIM_MASTER compare match for compare register 3
    MasterCmp3 = 1 << 2,

    /// bit 1 ADCxMC2 - Trigger on HRTIM_MASTER compare match for compare register 2
    MasterCmp2 = 1 << 1,

    /// bit 0 ADCxMC1 - Trigger on HRTIM_MASTER compare match for compare register 1
    MasterCmp1 = 1 << 0,
}

pub enum AdcTriggerPostscaler {
    None = 0,
    Div2 = 1,
    Div3 = 2,
    Div4 = 3,
    Div5 = 4,
    Div6 = 5,
    Div7 = 6,
    Div8 = 7,
    Div9 = 8,
    Div10 = 9,
    Div11 = 10,
    Div12 = 11,
    Div13 = 12,
    Div14 = 13,
    Div15 = 14,
    Div16 = 15,
    Div17 = 16,
    Div18 = 17,
    Div19 = 18,
    Div20 = 19,
    Div21 = 20,
    Div22 = 21,
    Div23 = 22,
    Div24 = 23,
    Div25 = 24,
    Div26 = 25,
    Div27 = 26,
    Div28 = 27,
    Div29 = 28,
    Div30 = 29,
    Div31 = 30,
    Div32 = 31,
}

pub enum Adc24Trigger {
    // /// bit 31 ADCxTERST
    // _ = 1 << 31,
    /// bit 30 ADCxTEC4 - Trigger on HRTIM_TIME compare match for compare register 4
    TimECmp4 = 1 << 30,

    /// bit 29 ADCxTEC3 - Trigger on HRTIM_TIME compare match for compare register 3
    TimECmp3 = 1 << 29,

    /// bit 28 ADCxTEC2 - Trigger on HRTIM_TIME compare match for compare register 2
    TimECmp2 = 1 << 28,

    // /// bit 27 ADCxTDRST
    // _ = 1 << 27,
    /// bit 26 ADCxTDPER - Trigger on HRTIM_TIMD period
    TimDPeriod = 1 << 26,

    /// bit 25 ADCxTDC4 - Trigger on HRTIM_TIMD compare match for compare register 4
    TimDCmp4 = 1 << 25,

    /// bit 24 ADCxTFPER - Trigger on HRTIM_TIMF period
    TimFPeriod = 1 << 24,

    /// bit 23 ADCxTDC2 - Trigger on HRTIM_TIMD compare match for compare register 2
    TimDCmp2 = 1 << 23,

    /// bit 22 ADCxTCRST
    // _ = 1 << 22,

    /// bit 21 ADCxTCPER - Trigger on HRTIM_TIMC period
    TimCPeriod = 1 << 21,

    /// bit 20 ADCxTCC4 - Trigger on HRTIM_TIMC compare match for compare register 4
    TimCCmp4 = 1 << 20,

    /// bit 19 ADCxTFC4 - Trigger on HRTIM_TIMF compare match for compare register 4
    TimFCmp4 = 1 << 19,

    /// bit 18 ADCxTCC2 - Trigger on HRTIM_TIMC compare match for compare register 2
    TimCCmp2 = 1 << 18,

    /// bit 17 ADCxTBPER - Trigger on HRTIM_TIMB period
    TimBPeriod = 1 << 17,

    /// bit 16 ADCxTBC4  - Trigger on HRTIM_TIMB compare match for compare register 4
    TimBCmp4 = 1 << 16,

    /// bit 15 ADCxTFC3 - Trigger on HRTIM_TIMF compare match for compare register 3
    TimFCmp3 = 1 << 15,

    /// bit 14 ADCxTBC2 - Trigger on HRTIM_TIMB compare match for compare register 2
    TimBCmp2 = 1 << 14,

    /// bit 13 ADCxTAPER - Trigger on HRTIM_TIMA period
    TimAPeriod = 1 << 13,

    /// bit 12 ADCxTAC4 - Trigger on HRTIM_TIMA compare match for compare register 4
    TimACmp4 = 1 << 12,

    /// bit 11 ADCxTFC2 - Trigger on HRTIM_TIMF compare match for compare register 2
    TimFCmp2 = 1 << 11,

    /// bit 10 ADCxTAC2 - Trigger on HRTIM_TIMA compare match for compare register 2
    TimACmp2 = 1 << 10,

    // /// bit 9 ADCxEEV10
    // _ = 1 << 9,

    // /// bit 8 ADCxEEV9
    // _ = 1 << 8,

    // /// bit 7 ADCxEEV8
    // _ = 1 << 7,

    // /// bit 6 ADCxEEV7
    // _ = 1 << 6,

    // /// bit 5 ADCxEEV6
    // _ = 1 << 5,
    /// bit 4 ADCxMPER - Trigger on HRTIM_MASTER period
    MasterPeriod = 1 << 4,

    /// bit 3 ADCxMC4 - Trigger on HRTIM_MASTER compare match for compare register 4
    MasterCmp4 = 1 << 3,

    /// bit 2 ADCxMC3 - Trigger on HRTIM_MASTER compare match for compare register 3
    MasterCmp3 = 1 << 2,

    /// bit 1 ADCxMC2 - Trigger on HRTIM_MASTER compare match for compare register 2
    MasterCmp2 = 1 << 1,

    /// bit 0 ADCxMC1 - Trigger on HRTIM_MASTER compare match for compare register 1
    MasterCmp1 = 1 << 0,
}

pub enum Adc579Trigger {
    // /// Trigger on F reset and counter roll-over
    // _ = 31
    /// Trigger on HRTIM_TIMF period
    TimFPeriod = 30,

    /// Trigger on HRTIM_TIMF compare match for compare register 4
    TimFCmp4 = 29,

    /// Trigger on HRTIM_TIMF compare match for compare register 3
    TimFCmp3 = 28,

    /// Trigger on HRTIM_TIMF compare match for compare register 2
    TimFCmp2 = 27,

    /// Trigger on HRTIM_TIME period
    TimEPeriod = 26,

    /// Trigger on HRTIM_TIME compare match for compare register 4
    TimECmp4 = 25,

    /// Trigger on HRTIM_TIME compare match for compare register 3
    TimECmp3 = 24,

    /// Trigger on HRTIM_TIMD period
    TimDPeriod = 23,

    /// Trigger on HRTIM_TIMD compare match for compare register 4
    TimDCmp4 = 22,

    /// Trigger on HRTIM_TIMD compare match for compare register 3
    TimDCmp3 = 21,

    /// Trigger on HRTIM_TIMC period
    TimCPeriod = 20,

    /// Trigger on HRTIM_TIMC compare match for compare register 4
    TimCCmp4 = 19,

    /// Trigger on HRTIM_TIMC compare match for compare register 3
    TimCCmp3 = 18,

    // /// Trigger on B reset and counter roll-over
    // _ = 17
    /// Trigger on HRTIM_TIMB period
    TimBPeriod = 16,

    /// Trigger on HRTIM_TIMB compare match for compare register 4
    TimBCmp4 = 15,

    /// Trigger on HRTIM_TIMB compare match for compare register 3
    TimBCmp3 = 14,

    // /// Trigger on A reset and counter roll-over
    // _ = 13
    /// Trigger on HRTIM_TIMA period
    TimAPeriod = 12,

    /// Trigger on HRTIM_TIMA compare match for compare register 4
    TimACmp4 = 11,

    /// Trigger on HRTIM_TIMA compare match for compare register 3
    TimACmp3 = 10,

    // /// Trigger on EEV5
    // _ = 9,

    // ///  Trigger on EEV4
    // _ = 8,

    // ///  Trigger on EEV3
    // _ = 7,

    // ///  Trigger on EEV2
    // _ = 6,

    // ///  Trigger on EEV1
    // _ = 5,
    /// Trigger on HRTIM_MASTER period
    MasterPeriod = 4,

    /// Trigger on HRTIM_MASTER compare match for compare register 4
    MasterCmp4 = 3,

    /// Trigger on HRTIM_MASTER compare match for compare register 3
    MasterCmp3 = 2,

    /// Trigger on HRTIM_MASTER compare match for compare register 2
    MasterCmp2 = 1,

    /// Trigger on HRTIM_MASTER compare match for compare register 1
    MasterCmp1 = 0,
}

pub enum Adc6810Trigger {
    /// Trigger on HRTIM_TIMF period
    TimFPeriod = 31,

    /// Trigger on HRTIM_TIMF compare match for compare register 4
    TimFCmp4 = 30,

    /// Trigger on HRTIM_TIMF compare match for compare register 3
    TimFCmp3 = 29,

    /// Trigger on HRTIM_TIMF compare match for compare register 2
    TimFCmp2 = 28,

    // /// Trigger on E reset and counter roll-over
    // _ = 27
    /// Trigger on HRTIM_TIME compare match for compare register 4
    TimECmp4 = 26,

    /// Trigger on HRTIM_TIME compare match for compare register 3
    TimECmp3 = 25,

    /// Trigger on HRTIM_TIME compare match for compare register 2
    TimECmp2 = 24,

    // /// Trigger on D reset and counter roll-over
    // _ = 23
    /// Trigger on HRTIM_TIMD period
    TimDPeriod = 22,

    /// Trigger on HRTIM_TIMD compare match for compare register 4
    TimDCmp4 = 21,

    /// Trigger on HRTIM_TIMD compare match for compare register 2
    TimDCmp2 = 20,

    // /// Trigger on D reset and counter roll-over
    // _ = 19
    /// Trigger on HRTIM_TIMC period
    TimCPeriod = 18,

    /// Trigger on HRTIM_TIMC compare match for compare register 4
    TimCCmp4 = 17,

    /// Trigger on HRTIM_TIMC compare match for compare register 2
    TimCCmp2 = 16,

    /// Trigger on HRTIM_TIMB period
    TimBPeriod = 15,

    /// Trigger on HRTIM_TIMB compare match for compare register 4
    TimBCmp4 = 14,

    /// Trigger on HRTIM_TIMB compare match for compare register 2
    TimBCmp2 = 13,

    /// Trigger on HRTIM_TIMA period
    TimAPeriod = 12,

    /// Trigger on HRTIM_TIMA compare match for compare register 4
    TimACmp4 = 11,

    /// Trigger on HRTIM_TIMA compare match for compare register 2
    TimACmp2 = 10,

    // /// Trigger on EEV10
    // _ = 9,

    // ///  Trigger on EEV9
    // _ = 8,

    // ///  Trigger on EEV8
    // _ = 7,

    // ///  Trigger on EEV7
    // _ = 6,

    // ///  Trigger on EEV6
    // _ = 5,
    /// Trigger on HRTIM_MASTER period
    MasterPeriod = 4,

    /// Trigger on HRTIM_MASTER compare match for compare register 4
    MasterCmp4 = 3,

    /// Trigger on HRTIM_MASTER compare match for compare register 3
    MasterCmp3 = 2,

    /// Trigger on HRTIM_MASTER compare match for compare register 2
    MasterCmp2 = 1,

    /// Trigger on HRTIM_MASTER compare match for compare register 1
    MasterCmp1 = 0,
}

macro_rules! impl_flt_monitor {
    ($($t:ident: ($fltx:ident, $fltxc:ident),)+) => {$(
        pub struct $t {
            _x: PhantomData<()>
        }

        impl FaultMonitor for $t {
            fn is_fault_active(&self) -> bool {
                let common = unsafe { &*HRTIM_COMMON::ptr() };
                common.isr.read().$fltx().bit()
            }

            fn clear_fault(&mut self) {
                let common = unsafe { &*HRTIM_COMMON::ptr() };
                common.icr.write(|w| w.$fltxc().set_bit());
            }

            // TODO: Should we have our own trait since it does not seem possible to implement this
            fn set_fault(&mut self) {
                todo!()
            }
        }
    )+};
}

impl_flt_monitor!(
    FltMonitorSys: (sysflt, sysfltc),
    FltMonitor1: (flt1, flt1c),
    FltMonitor2: (flt2, flt2c),
    FltMonitor3: (flt3, flt3c),
    FltMonitor4: (flt4, flt4c),
    FltMonitor5: (flt5, flt5c),
    FltMonitor6: (flt6, flt6c),
);
