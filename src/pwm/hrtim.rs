/*
period: HRTIM_PERAR
enable output: HRTIM_OENR
enable preload: PREEN,
prescaler per timer: CKPSC[2:0] kan bara sättas en gång/när timern är disablad
NOTE: Timrar som kommunicerar output set/clear, counter reset, update, external event filter eller capture triggers måste ha samma prescaler

continous mode: CONT in TIMxCR

crossbar config for SET events: HRTIM_SET<TIM><OUTPUT>R
crossbar config for RST events: HRTIM_RST<TIM><OUTPUT>R
att sätta ett event i båda register betyder "TOGGLE"-event

tvinga 50% duty: HALF in HRTIM_TIMxCR smidigt vid variabel frekvens. Var gång HRTIM_PERxR(perioden) skrivs så kommer compare1-värdet bli halva perioden.
För att detta ska fungera måste utgången vara konfigurerad för en övergång för "compare1" och en vid "period". Se sida 856

Triggered-half mode: TRGHLF in HRTIM_TIMxCR2


**** push-pull mode ****: PSHPLL in HRTIM_TIMxCR, detta verkar styra två utgångar på ett sätt som alternerande, varannan period skickar ut signal på ena utgången och håller andra utgången inaktiv och virse versa. se sida 864


reset timers counters: TxRST in HRTIM_CR2 reset sker inte förens timrarna är enablade
polaritet: POLx 0: pos, 1:neg

starta timer: MCEN or TxCEN oklart...
*/

use core::marker::PhantomData;
use core::mem::MaybeUninit;

use crate::gpio::gpioa::{PA8, PA9};
use crate::gpio::{Alternate, AF13};
use crate::stm32::{
    HRTIM_COMMON, HRTIM_MASTER, HRTIM_TIMA, HRTIM_TIMB, HRTIM_TIMC, HRTIM_TIMD, HRTIM_TIME,
    HRTIM_TIMF,
};

use super::{ActiveHigh, Alignment, ComplementaryImpossible, Pins, Pwm, PwmPinEnable, TimerType};
use crate::rcc::{Enable, GetBusFreq, Rcc, Reset};
use crate::stm32::RCC;
use crate::time::Hertz;

pub struct CH1<PSCL>(PhantomData<PSCL>);
pub struct CH2<PSCL>(PhantomData<PSCL>);

macro_rules! pins {
    ($($TIMX:ty:
        CH1: [$($( #[ $pmeta1:meta ] )* $CH1:ty),*] CH2: [$($( #[ $pmeta2:meta ] )* $CH2:ty),*])+) => {
        $(
            $(
                $( #[ $pmeta1 ] )*
                impl<PSCL> Pins<$TIMX, CH1<PSCL>, ComplementaryImpossible> for $CH1 {
                    type Channel = Pwm<$TIMX, CH1<PSCL>, ComplementaryImpossible, ActiveHigh, ActiveHigh>;
                }
            )*
            $(
                $( #[ $pmeta2 ] )*
                impl<PSCL> Pins<$TIMX, CH2<PSCL>, ComplementaryImpossible> for $CH2 {
                    type Channel = Pwm<$TIMX, CH2<PSCL>, ComplementaryImpossible, ActiveHigh, ActiveHigh>;
                }
            )*
        )+
    }
}

pins! {
    HRTIM_TIMA:
        CH1: [
            PA8<Alternate<AF13>>
        ]
        CH2: [
            PA9<Alternate<AF13>>
        ]
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

                let clk = $TIMX::get_timer_frequency(&rcc.clocks);

                let (period, psc) = <TimerHrTim<PSCL>>::calculate_frequency(clk, freq, Alignment::Left);

                // Write prescaler
                tim.$timXcr.write(|w| unsafe { w.cont().set_bit().ck_pscx().bits(psc as u8) });

                // Write period
                tim.$perXr.write(|w| unsafe { w.perx().bits(period as u16) });

                // Start timer
                cortex_m::interrupt::free(|_| {
                    let master = unsafe { &*HRTIM_MASTER::ptr() };
                    master.mcr.modify(|_r, w| { w.$tXcen().set_bit() });
                });

                unsafe { MaybeUninit::<PINS::Channel>::uninit().assume_init() }
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
    fn calculate_frequency(base_freq: Hertz, freq: Hertz, alignment: Alignment) -> (u32, u16) {
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
