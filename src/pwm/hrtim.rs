use core::marker::PhantomData;
use core::mem::MaybeUninit;

use fugit::HertzU64;

use crate::gpio::gpioa::{PA10, PA11, PA12, PA13, PA15, PA8, PA9};
use crate::gpio::gpiob::{PB0, PB10, PB11, PB14, PB15};
use crate::gpio::gpioc::{PC10, PC7, PC8, PC9};
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
    //HRTIM_TIMF: CH1: PC6<Alternate<AF13>>, CH2: PC7<Alternate<AF13>>
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

/*pub struct Hrtimer<PSCL, TIM> {
    _prescaler: PhantomData<PSCL>,
    _timer: PhantomData<TIM>,
    period: u16, // $perXr.perx

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
    fn pwm_advanced<PINS, CHANNEL, COMP>(
        self,
        _pins: PINS,
        rcc: &mut Rcc,
    ) -> HrPwmBuilder<Self, Pscl128, PINS::Out>
    where
        PINS: Pins<Self, CHANNEL, COMP> + ToHrOut,
        CHANNEL: HrtimChannel<Pscl128>;
}

/// HrPwmBuilder is used to configure advanced HrTim PWM features
pub struct HrPwmBuilder<TIM, PSCL, OUT> {
    _tim: PhantomData<TIM>,
    _prescaler: PhantomData<PSCL>,
    _out: PhantomData<OUT>,
    alignment: Alignment,
    base_freq: HertzU64,
    count: CountSettings,
    fault_enable_bits: u8,
    fault1_bits: u8,
    fault2_bits: u8,
    enable_push_pull: bool,
    //deadtime: NanoSecond,
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

    Period,
    /*
    /// Compare match with compare register 1 of master timer
    MasterCr1,

    /// Compare match with compare register 2 of master timer
    MasterCr2,

    /// Compare match with compare register 3 of master timer
    MasterCr3,

    /// Compare match with compare register 4 of master timer
    MasterCr4,

    MasterPeriod,*/
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

// Implement PWM configuration for timer
macro_rules! hrtim_hal {
    ($($TIMX:ident: ($timX:ident, $timXcr:ident, $perXr:ident, $tXcen:ident, $fltXr:ident, $outXr:ident),)+) => {
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
                fn pwm_advanced<PINS, CHANNEL, COMP>(
                    self,
                    _pins: PINS,
                    rcc: &mut Rcc,
                ) -> HrPwmBuilder<Self, Pscl128, PINS::Out>
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
                        enable_push_pull: false,
                        //deadtime: 0.nanos(),
                    }
                }
            }

            impl<PSCL, OUT>
                HrPwmBuilder<$TIMX, PSCL, OUT>
            where
                PSCL: HrtimPrescaler,
            {
                pub fn finalize(self, _control: &mut HrPwmControl) -> (HrTim<$TIMX, PSCL>, (HrCr1<$TIMX>, HrCr2<$TIMX>, HrCr3<$TIMX>, HrCr4<$TIMX>), OUT) {
                    let tim = unsafe { &*$TIMX::ptr() };

                    let (period, prescaler_bits) = match self.count {
                        CountSettings::Period(period) => (period as u32, PSCL::BITS as u16),
                        CountSettings::Frequency( freq ) => {
                            <TimerHrTim<PSCL>>::calculate_frequency(self.base_freq, freq, self.alignment)
                        },
                    };

                    // Write prescaler and any special modes
                    tim.$timXcr.write(|w| unsafe {
                        w
                            // Enable Continous mode
                            .cont().set_bit()

                            // Push-Pull mode
                            .pshpll().bit(self.enable_push_pull)

                            // TODO: add support for more modes

                            // Set prescaler
                            .ck_pscx().bits(prescaler_bits as u8)
                    });

                    // Write period
                    tim.$perXr.write(|w| unsafe { w.perx().bits(period as u16) });

                    // Enable fault sources and lock configuration
                    unsafe {
                        // Enable fault sources
                        let fault_enable_bits = self.fault_enable_bits as u32;
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
                            .fault1().bits(self.fault1_bits)
                            .fault2().bits(self.fault2_bits)
                        );
                    }

                    // Start timer
                    cortex_m::interrupt::free(|_| {
                        let master = unsafe { &*HRTIM_MASTER::ptr() };
                        master.mcr.modify(|_r, w| { w.$tXcen().set_bit() });
                    });

                    unsafe {
                        MaybeUninit::uninit().assume_init()
                    }
                }

                /// Set the PWM frequency; will overwrite the previous prescaler and period
                /// The requested frequency will be rounded to the nearest achievable frequency; the actual frequency may be higher or lower than requested.
                pub fn frequency<T: Into<Hertz>>(mut self, freq: T) -> Self {
                    self.count = CountSettings::Frequency( freq.into() );

                    self
                }

                /// Set the prescaler; PWM count runs at base_frequency/(prescaler+1)
                pub fn prescaler<P>(self, _prescaler: P) -> HrPwmBuilder<$TIMX, P, OUT>
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
                        //deadtime: 0.nanos(),
                    }
                }

                pub fn with_fault_source<FS>(self, _fault_source: FS) -> HrPwmBuilder<$TIMX, PSCL, OUT>
                    where FS: FaultSource
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
                    } = self;

                    HrPwmBuilder {
                        _tim,
                        fault_enable_bits: fault_enable_bits | FS::ENABLE_BITS,
                        fault1_bits,
                        fault2_bits,
                        _prescaler: PhantomData,
                        _out,
                        enable_push_pull,
                        alignment,
                        base_freq,
                        count,
                    }
                }

                pub fn fault_action1(mut self, fault_action1: FaultAction) -> Self {
                    self.fault1_bits = fault_action1 as _;
                    self
                }

                pub fn fault_action2(mut self, fault_action2: FaultAction) -> Self {
                    self.fault2_bits = fault_action2 as _;
                    self
                }

                /// Set the period; PWM count runs from 0 to period, repeating every (period+1) counts
                pub fn period(mut self, period: u16) -> Self {
                    self.count = CountSettings::Period(period);

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
    }
}

macro_rules! hrtim_pin_hal {
    ($($TIMX:ident:
        ($CH:ident, $perXr:ident, $cmpXYr:ident, $cmpYx:ident, $cmpY:ident, $tXYoen:ident, $tXYodis:ident, $setXYr:ident, $rstXYr:ident),)+
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

                fn set_duty(&mut self, duty: Self::Duty) {
                    let tim = unsafe { &*$TIMX::ptr() };

                    tim.$cmpXYr.write(|w| unsafe { w.$cmpYx().bits(duty) });
                }
            }

            // Enable implementation for ComplementaryImpossible
            impl<POL, NPOL, PSCL> PwmPinEnable for Pwm<$TIMX, $CH<PSCL>, ComplementaryImpossible, POL, NPOL> {
                fn ccer_enable(&mut self) {
                    let tim = unsafe { &*$TIMX::ptr() };
                    // Select period as a SET-event
                    tim.$setXYr.write(|w| { w.per().set_bit() } );

                    // Select cmpY as a RESET-event
                    tim.$rstXYr.write(|w| { w.$cmpY().set_bit() } );

                    // TODO: Should this part only be in Pwm::enable?
                    // Enable output Y on channel X
                    // This is a set-only register, no risk for data race
                    let common = unsafe { &*HRTIM_COMMON::ptr() };
                    common.oenr.write(|w| { w.$tXYoen().set_bit() });
                }
                fn ccer_disable(&mut self) {
                    let tim = unsafe { &*$TIMX::ptr() };
                    // Clear SET-events
                    tim.$setXYr.reset();

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
        }
    }};
}

macro_rules! hrtim_out {
    ($($TIMX:ident: $out_type:ident: $tXYoen:ident, $tXYodis:ident, $tXYods:ident, $setXYr:ident, $rstXYr:ident,)+) => {$(
        impl HrOutput for $out_type<$TIMX> {
            fn enable(&mut self) {
                cortex_m::interrupt::free(|_| {
                    let common = unsafe { &*HRTIM_COMMON::ptr() };
                    common.oenr.write(|w| { w.$tXYoen().set_bit() });
                });
            }

            fn disable(&mut self) {
                cortex_m::interrupt::free(|_| {
                    let common = unsafe { &*HRTIM_COMMON::ptr() };
                    common.odisr.write(|w| { w.$tXYodis().set_bit() });
                });
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
    HRTIM_TIMA: HrOut2: ta2oen, ta1odis, ta2ods, seta2r, rsta2r,

    HRTIM_TIMB: HrOut1: tb1oen, tb1odis, tb1ods, setb1r, rstb1r,
    HRTIM_TIMB: HrOut2: tb2oen, tb2odis, tb2ods, setb2r, rstb2r,

    HRTIM_TIMC: HrOut1: tc1oen, tc1odis, tc1ods, setc1r, rstc1r,
    HRTIM_TIMC: HrOut2: tc2oen, tc2odis, tc2ods, setc2r, rstc2r,

    HRTIM_TIMD: HrOut1: td1oen, td1odis, td1ods, setd1r, rstd1r,
    HRTIM_TIMD: HrOut2: td2oen, td2odis, td2ods, setd2r, rstd2r,

    HRTIM_TIME: HrOut1: te1oen, te1odis, te1ods, sete1r, rste1r,
    HRTIM_TIME: HrOut2: te2oen, te2odis, te2ods, sete2r, rste2r,

    // TODO: Somehow, there is no rstf1r
    //HRTIM_TIMF: HrOut1: tf1oen, tf1odis, tf1ods, setf1r, rstf1r,

    // TODO: Somehow, there is no tf2oen
    //HRTIM_TIMF: HrOut2: tf2oen, tf2odis, tf2ods, setf2r, rstf2r,
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
    ($($TIMX:ident: [$cmpX1r:ident, $cmpX2r:ident, $cmpX3r:ident, $cmpX4r:ident],)+) => {$(
        hrtim_cr_helper!($TIMX: HrCr1: $cmpX1r, cmp1x);
        hrtim_cr_helper!($TIMX: HrCr2: $cmpX2r, cmp2x);
        hrtim_cr_helper!($TIMX: HrCr3: $cmpX3r, cmp3x);
        hrtim_cr_helper!($TIMX: HrCr4: $cmpX4r, cmp4x);
    )+};
}

macro_rules! hrtim_timer {
    ($($TIMX:ident: $perXr:ident,)+) => {$(
        impl<PSCL> HrTimer<$TIMX, PSCL> for HrTim<$TIMX, PSCL> {
            fn get_period(&self) -> u16 {
                let tim = unsafe { &*$TIMX::ptr() };

                tim.$perXr.read().perx().bits()
            }
            fn set_period(&mut self, period: u16) {
                let tim = unsafe { &*$TIMX::ptr() };

                tim.$perXr.write(|w| unsafe { w.perx().bits(period as u16) });
            }
        }
    )+};
}

hrtim_timer! {
    HRTIM_TIMA: perar,
    HRTIM_TIMB: perbr,
    HRTIM_TIMC: percr,
    HRTIM_TIMD: perdr,
    HRTIM_TIME: perer,
    HRTIM_TIMF: perfr,
}

hrtim_cr! {
    HRTIM_TIMA: [cmp1ar, cmp2ar, cmp3ar, cmp4ar],
    HRTIM_TIMB: [cmp1br, cmp2br, cmp3br, cmp4br],
    HRTIM_TIMC: [cmp1cr, cmp2cr, cmp3cr, cmp4cr],
    HRTIM_TIMD: [cmp1dr, cmp2dr, cmp3dr, cmp4dr],
    HRTIM_TIME: [cmp1er, cmp2er, cmp3er, cmp4er],
    HRTIM_TIMF: [cmp1fr, cmp2fr, cmp3fr, cmp4fr],
}

hrtim_hal! {
    // TODO: HRTIM_MASTER
    HRTIM_TIMA: (hrtim_tima, timacr, perar, tacen, fltar, outar),
    HRTIM_TIMB: (hrtim_timb, timbcr, perbr, tbcen, fltbr, outbr),
    HRTIM_TIMC: (hrtim_timc, timccr, percr, tccen, fltcr, outcr),
    HRTIM_TIMD: (hrtim_timd, timdcr, perdr, tdcen, fltdr, outdr),
    HRTIM_TIME: (hrtim_time, timecr, perer, tecen, flter, outer),
    HRTIM_TIMF: (hrtim_timf, timfcr, perfr, tfcen, fltfr, outfr),
}

hrtim_pin_hal! {
    HRTIM_TIMA: (CH1, perar, cmp1ar, cmp1x, cmp1, ta1oen, ta1odis, seta1r, rsta1r),
    HRTIM_TIMA: (CH2, perar, cmp3ar, cmp3x, cmp3, ta2oen, ta2odis, seta2r, rsta2r),

    HRTIM_TIMB: (CH1, perbr, cmp1br, cmp1x, cmp1, tb1oen, tb1odis, setb1r, rstb1r),
    HRTIM_TIMB: (CH2, perbr, cmp3br, cmp3x, cmp3, tb2oen, tb2odis, setb2r, rstb2r),

    HRTIM_TIMC: (CH1, percr, cmp1cr, cmp1x, cmp1, tc1oen, tc1odis, setc1r, rstc1r),
    HRTIM_TIMC: (CH2, percr, cmp3cr, cmp3x, cmp3, tc2oen, tc2odis, setc2r, rstc2r),


    HRTIM_TIMD: (CH1, perdr, cmp1dr, cmp1x, cmp1, td1oen, td1odis, setd1r, rstd1r),
    HRTIM_TIMD: (CH2, perdr, cmp3dr, cmp3x, cmp3, td2oen, td2odis, setd2r, rstd2r),

    HRTIM_TIME: (CH1, perer, cmp1er, cmp1x, cmp1, te1oen, te1odis, sete1r, rste1r),
    HRTIM_TIME: (CH2, perer, cmp3er, cmp3x, cmp3, te2oen, te2odis, sete2r, rste2r),

    // TODO: tf1oen and rstf1r are not defined
    //HRTIM_TIMF: (CH1, perfr, cmp1fr, cmp1x, cmp1, tf1oen, tf1odis, setf1r, rstf1r),

    // TODO: tf2oen is not defined
    //HRTIM_TIMF: (CH2, perfr, cmp3fr, cmp3x, cmp3, tf2oen, tf2odis, setf2r, rstf2r),
}

pub trait HrtimPrescaler {
    const BITS: u8;
    const VALUE: u8;
}

macro_rules! impl_pscl {
    ($($t:ident => $b:literal, $c:literal,)+) => {$(
        #[derive(Copy, Clone)]
        pub struct $t;
        impl HrtimPrescaler for $t {
            const BITS: u8 = $b;
            const VALUE: u8 = $c;
        }
    )+};
}

impl_pscl! {
    Pscl1   => 0b000,   1,
    Pscl2   => 0b001,   2,
    Pscl4   => 0b010,   4,
    Pscl8   => 0b011,   8,
    Pscl16  => 0b100,  16,
    Pscl32  => 0b101,  32,
    Pscl64  => 0b110,  64,
    Pscl128 => 0b111, 128,
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
    is_active_high: bool,
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

            /*pub fn bind_comp(self, comp: $compX) -> SourceBuilder<$input> {
                unsafe { SourceBuilder::new(self, 0b01) }
            }*/

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
            /* pub fn filter(?) -> Self */
            /* pub fn blanking(?) -> Self */
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
    /// fault signal sampling clock = f_hrtim
    None = 0b00,

    /// 1/2
    ///
    /// fault signal sampling clock = f_hrtim / 2
    Two = 0b01,

    /// 1/4
    ///
    /// fault signal sampling clock = f_hrtim / 4
    Four = 0b10,

    /// 1/8
    ///
    /// fault signal sampling clock = f_hrtim / 8
    Eight = 0b11,
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
            divider: FaultSamplingClkDiv::None,
        }
    }
}

pub struct HrTimOngoingCalibration {
    divider: FaultSamplingClkDiv,
}

impl HrTimOngoingCalibration {
    /// SAFETY: Calibration needs to be done before calling this
    unsafe fn init(self) -> (HrPwmControl, FaultInputs) {
        let common = unsafe { &*HRTIM_COMMON::ptr() };

        unsafe {
            // Enable periodic calibration
            // with f_hrtim at 170MHz, these settings leads to
            // a period of about 6.2ms
            common
                .dllcr
                .modify(|_r, w| w.calrte().bits(0b00).cal().set_bit().calen().clear_bit());
            common.fltinr2.write(|w| w.fltsd().bits(self.divider as u8));
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
