use core::marker::PhantomData;
use core::mem::MaybeUninit;

use fugit::HertzU64;

use crate::gpio::gpioa::{PA10, PA11, PA12, PA13, PA8, PA9};
use crate::gpio::gpiob::{PB14, PB15};
use crate::gpio::gpioc::{PC8, PC9};
use crate::gpio::{Alternate, AF13, AF3};
use crate::stm32::{
    HRTIM_COMMON, HRTIM_MASTER, HRTIM_TIMA, HRTIM_TIMB, HRTIM_TIMC, HRTIM_TIMD, HRTIM_TIME,
    HRTIM_TIMF,
};

use super::{
    ActiveHigh, Alignment, ComplementaryImpossible, FaultDisabled, Pins, Pwm, PwmPinEnable,
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
    fn pwm<PINS, T, PSCL, U, V>(self, _pins: PINS, frequency: T, rcc: &mut Rcc) -> PINS::Channel
    where
        PINS: Pins<Self, U, V>,
        T: Into<Hertz>,
        U: HrtimChannel<PSCL>,
        PSCL: HrtimPrescaler;
}

pub trait HrPwmAdvExt: Sized {
    fn pwm_advanced<PINS, CHANNEL, COMP>(
        self,
        _pins: PINS,
        rcc: &mut Rcc,
    ) -> HrPwmBuilder<Self, Pscl128, FaultDisabled, PINS::Out>
    where
        PINS: Pins<Self, CHANNEL, COMP> + ToHrOut,
        CHANNEL: HrtimChannel<Pscl128>;
}

/// HrPwmBuilder is used to configure advanced HrTim PWM features
pub struct HrPwmBuilder<TIM, PSCL, FAULT, OUT> {
    _tim: PhantomData<TIM>,
    _fault: PhantomData<FAULT>,
    _prescaler: PhantomData<PSCL>,
    _out: PhantomData<OUT>,
    alignment: Alignment,
    base_freq: HertzU64,
    count: CountSettings,
    enable_push_pull: bool,
    //bkin_enabled: bool, // If the FAULT type parameter is FaultEnabled, either bkin or bkin2 must be enabled
    //bkin2_enabled: bool,
    //fault_polarity: Polarity,
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

// Implement HrPwmExt trait for hrtimer
macro_rules! pwm_ext_hal {
    ($TIMX:ident: $timX:ident) => {
        impl HrPwmExt for $TIMX {
            fn pwm<PINS, T, PSCL, U, V>(
                self,
                pins: PINS,
                frequency: T,
                rcc: &mut Rcc,
            ) -> PINS::Channel
            where
                PINS: Pins<Self, U, V>,
                T: Into<Hertz>,
                U: HrtimChannel<PSCL>,
                PSCL: HrtimPrescaler,
            {
                $timX(self, pins, frequency.into(), rcc)
            }
        }
    };
}

// Implement PWM configuration for timer
macro_rules! hrtim_hal {
    ($($TIMX:ident: ($timX:ident, $timXcr:ident, $perXr:ident, $tXcen:ident),)+) => {
        $(
            pwm_ext_hal!($TIMX: $timX);

            /// Configures PWM
            fn $timX<PINS, T, PSCL, U>(
                tim: $TIMX,
                _pins: PINS,
                freq: Hertz,
                rcc: &mut Rcc,
            ) -> PINS::Channel
            where
                PINS: super::Pins<$TIMX, T, U>,
                T: HrtimChannel<PSCL>,
                PSCL: HrtimPrescaler,
            {
                unsafe {
                    let rcc_ptr = &*RCC::ptr();
                    $TIMX::enable(rcc_ptr);
                    $TIMX::reset(rcc_ptr);
                }

                // TODO: That 32x factor... Is that done by $TIMX::get_timer_frequency
                // or should we do that? Also that will likely risk overflowing u32 since
                // 170MHz * 32 = 5.44GHz > u32::MAX.Hz()
                let clk = HertzU64::from($TIMX::get_timer_frequency(&rcc.clocks)) * 32;

                let (period, prescaler_bits) = <TimerHrTim<PSCL>>::calculate_frequency(clk, freq, Alignment::Left);

                // Write prescaler
                tim.$timXcr.write(|w| unsafe { w.cont().set_bit().ck_pscx().bits(prescaler_bits as u8) });

                // Write period
                tim.$perXr.write(|w| unsafe { w.perx().bits(period as u16) });

                // Start timer
                cortex_m::interrupt::free(|_| {
                    let master = unsafe { &*HRTIM_MASTER::ptr() };
                    master.mcr.modify(|_r, w| { w.$tXcen().set_bit() });
                });

                unsafe { MaybeUninit::<PINS::Channel>::uninit().assume_init() }
            }

            impl HrPwmAdvExt for $TIMX {
                fn pwm_advanced<PINS, CHANNEL, COMP>(
                    self,
                    _pins: PINS,
                    rcc: &mut Rcc,
                ) -> HrPwmBuilder<Self, Pscl128, FaultDisabled, PINS::Out>
                where
                    PINS: Pins<Self, CHANNEL, COMP> + ToHrOut,
                    CHANNEL: HrtimChannel<Pscl128>
                {
                    unsafe {
                        let rcc_ptr = &(*RCC::ptr());
                        $TIMX::enable(rcc_ptr);
                        $TIMX::reset(rcc_ptr);
                    }

                    // TODO: That 32x factor... Is that included below, or should we
                    // do that? Also that will likely risk overflowing u32 since
                    // 170MHz * 32 = 5.44GHz > u32::MAX.Hz()
                    let clk = HertzU64::from($TIMX::get_timer_frequency(&rcc.clocks)) * 32;

                    HrPwmBuilder {
                        _tim: PhantomData,
                        _fault: PhantomData,
                        _prescaler: PhantomData,
                        _out: PhantomData,
                        alignment: Alignment::Left,
                        base_freq: clk,
                        count: CountSettings::Period(u16::MAX),
                        enable_push_pull: false,
                        //bkin_enabled: false,
                        //bkin2_enabled: false,
                        //fault_polarity: Polarity::ActiveLow,
                        //deadtime: 0.nanos(),
                    }
                }
            }

            impl<PSCL, FAULT, OUT>
                HrPwmBuilder<$TIMX, PSCL, FAULT, OUT>
            where
                PSCL: HrtimPrescaler,
            {
                pub fn finalize(self) -> (HrTim<$TIMX, PSCL>, (HrCr1<$TIMX>, HrCr2<$TIMX>, HrCr3<$TIMX>, HrCr4<$TIMX>), OUT) {
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
                pub fn prescaler<P>(self, _prescaler: P) -> HrPwmBuilder<$TIMX, P, FAULT, OUT>
                where
                    P: HrtimPrescaler,
                {
                    let HrPwmBuilder {
                        _tim,
                        _fault,
                        _prescaler: _,
                        _out,
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
                        _fault,
                        _prescaler: PhantomData,
                        _out,
                        enable_push_pull,
                        alignment,
                        base_freq,
                        count,
                        //bkin_enabled: false,
                        //bkin2_enabled: false,
                        //fault_polarity: Polarity::ActiveLow,
                        //deadtime: 0.nanos(),
                    }
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
        ($CH:ident, $perXr:ident, $cmpXYr:ident, $cmpYx:ident, $cmpY:ident, $tXYoen:ident, $setXYr:ident, $rstXYr:ident),)+
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
                    // NOTE(unsafe) critical section prevents races
                    cortex_m::interrupt::free(|_| {
                        let common = unsafe { &*HRTIM_COMMON::ptr() };
                        common.oenr.modify(|_r, w| { w.$tXYoen().set_bit() });
                    });
                }
                fn ccer_disable(&mut self) {
                    let tim = unsafe { &*$TIMX::ptr() };
                    // Clear SET-events
                    tim.$setXYr.reset();

                    // TODO: Should this part only be in Pwm::disable
                    // Do we want a potentially floating output after after disable?
                    // Disable output Y on channel X
                    // NOTE(unsafe) critical section prevents races
                    cortex_m::interrupt::free(|_| {
                        let common = unsafe { &*HRTIM_COMMON::ptr() };
                        common.oenr.modify(|_r, w| { w.$tXYoen().clear_bit() });
                    });
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
    ($($TIMX:ident: $out_type:ident: $tXYoen:ident, $setXYr:ident, $rstXYr:ident,)+) => {$(
        impl HrOutput for $out_type<$TIMX> {
            fn enable(&mut self) {
                cortex_m::interrupt::free(|_| {
                    let common = unsafe { &*HRTIM_COMMON::ptr() };
                    common.oenr.modify(|_r, w| { w.$tXYoen().set_bit() });
                });
            }

            fn disable(&mut self) {
                cortex_m::interrupt::free(|_| {
                    let common = unsafe { &*HRTIM_COMMON::ptr() };
                    common.oenr.modify(|_r, w| { w.$tXYoen().clear_bit() });
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
        }
    )+};
}

hrtim_out! {
    HRTIM_TIMA: HrOut1: ta1oen, seta1r, rsta1r,
    HRTIM_TIMA: HrOut2: ta2oen, seta2r, rsta2r,

    HRTIM_TIMB: HrOut1: tb1oen, setb1r, rstb1r,
    HRTIM_TIMB: HrOut2: tb2oen, setb2r, rstb2r,

    HRTIM_TIMC: HrOut1: tc1oen, setc1r, rstc1r,
    HRTIM_TIMC: HrOut2: tc2oen, setc2r, rstc2r,

    HRTIM_TIMD: HrOut1: td1oen, setd1r, rstd1r,
    HRTIM_TIMD: HrOut2: td2oen, setd2r, rstd2r,

    HRTIM_TIME: HrOut1: te1oen, sete1r, rste1r,
    HRTIM_TIME: HrOut2: te2oen, sete2r, rste2r,

    // TODO: Somehow, there is no rstf1r
    //HRTIM_TIMF: HrOut1: tf1oen, setf1r, rstf1r,

    // TODO: Somehow, there is no tf2oen
    //HRTIM_TIMF: HrOut2: tf2oen, setf2r, rstf2r,
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
    HRTIM_TIMA: (hrtim_tima, timacr, perar, tacen),
    HRTIM_TIMB: (hrtim_timb, timbcr, perbr, tbcen),
    HRTIM_TIMC: (hrtim_timc, timccr, percr, tccen),
    HRTIM_TIMD: (hrtim_timd, timdcr, perdr, tdcen),
    HRTIM_TIME: (hrtim_time, timecr, perer, tecen),

    // TODO: why is there no rstf1r?
    //HRTIM_TIMF: (hrtim_timf1, timfcr, perfr, tfcen, setf1r, rstf1r, cmp1),
    HRTIM_TIMF: (hrtim_timf, timfcr, perfr, tfcen),
}

hrtim_pin_hal! {
    HRTIM_TIMA: (CH1, perar, cmp1ar, cmp1x, cmp1, ta1oen, seta1r, rsta1r),
    HRTIM_TIMA: (CH2, perar, cmp3ar, cmp3x, cmp3, ta2oen, seta2r, rsta2r),

    HRTIM_TIMB: (CH1, perbr, cmp1br, cmp1x, cmp1, tb1oen, setb1r, rstb1r),
    HRTIM_TIMB: (CH2, perbr, cmp3br, cmp3x, cmp3, tb2oen, setb2r, rstb2r),

    HRTIM_TIMC: (CH1, percr, cmp1cr, cmp1x, cmp1, tc1oen, setc1r, rstc1r),
    HRTIM_TIMC: (CH2, percr, cmp3cr, cmp3x, cmp3, tc2oen, setc2r, rstc2r),


    HRTIM_TIMD: (CH1, perdr, cmp1dr, cmp1x, cmp1, td1oen, setd1r, rstd1r),
    HRTIM_TIMD: (CH2, perdr, cmp3dr, cmp3x, cmp3, td2oen, setd2r, rstd2r),

    HRTIM_TIME: (CH1, perer, cmp1er, cmp1x, cmp1, te1oen, sete1r, rste1r),
    HRTIM_TIME: (CH2, perer, cmp3er, cmp3x, cmp3, te2oen, sete2r, rste2r),

    // TODO: tf1oen and rstf1r are not defined
    //HRTIM_TIMF: (CH1, perfr, cmp1fr, cmp1x, cmp1, tf1oen, setf1r, rstf1r),

    // TODO: tf2oen is not defined
    //HRTIM_TIMF: (CH2, perfr, cmp3fr, cmp3x, cmp3, tf2oen, setf2r, rstf2r),
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
