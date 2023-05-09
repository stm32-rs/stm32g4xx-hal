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

use crate::gpio::gpioa::{PA8, PA9};
use crate::gpio::{self, Alternate, AF13};
use crate::stm32::{
    HRTIM_COMMON, HRTIM_MASTER, HRTIM_TIMA, HRTIM_TIMB, HRTIM_TIMC, HRTIM_TIMD, HRTIM_TIME,
    HRTIM_TIMF,
};

use super::{ActiveHigh, Alignment, ComplementaryImpossible, Pins, Pwm, TimerType};
use crate::rcc::{Enable, GetBusFreq, Rcc, Reset};
use crate::stm32::RCC;
use crate::time::Hertz;

struct CH1<PSCL>(PhantomData<PSCL>);
struct CH2<PSCL>(PhantomData<PSCL>);

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

pub struct Hrtimer<PSCL, TIM> {
    _prescaler: PhantomData<PSCL>,
    _timer: PhantomData<TIM>,
    period: u16,

    cmp_value1: u16,
    cmp_value2: u16,
    cmp_value3: u16,
    cmp_value4: u16,
}

impl<PSCL> TimerType for CH1<PSCL>
where
    PSCL: HrtimPrescaler,
{
    fn calculate_frequency(base_freq: Hertz, freq: Hertz, alignment: Alignment) -> (u32, u16) {
        <CH1<PSCL>>::calculate_frequency(base_freq, freq, alignment)
    }
}

impl<PSCL> TimerType for CH2<PSCL>
where
    PSCL: HrtimPrescaler,
{
    fn calculate_frequency(base_freq: Hertz, freq: Hertz, alignment: Alignment) -> (u32, u16) {
        <CH2<PSCL>>::calculate_frequency(base_freq, freq, alignment)
    }
}

// Implement PWM configuration for timer
macro_rules! hrtim_hal {
    ($($TIMX:ident: ($timX:ident, $timXcr:ident, $perXr:ident, $tXcen:ident),)+) => {
        $(
            //pwm_ext_hal!($TIMX: $timX);

            /// Configures PWM
            fn $timX<PINS, T, U>(
                tim: $TIMX,
                master: &mut HRTIM_MASTER,
                common: &mut HRTIM_COMMON,
                _pins: PINS,
                freq: Hertz,
                rcc: &mut Rcc,
            ) -> PINS::Channel
            where
                PINS: super::Pins<$TIMX, T, U>,
                T: TimerType,
            {
                unsafe {
                    let rcc_ptr = &(*RCC::ptr());
                    $TIMX::enable(rcc_ptr);
                    $TIMX::reset(rcc_ptr);
                }

                let clk = $TIMX::get_timer_frequency(&rcc.clocks);

                let (period, psc) = <T>::calculate_frequency(clk, freq, Alignment::Left);

                // Write prescaler
                tim.$timXcr.write(|w| { unsafe { w.cont().set_bit().ck_pscx().bits(psc as u8) } });

                // Write period
                tim.$perXr.write(|w| { unsafe { w.perx().bits(period as u16) } });

                // Start timer
                master.mcr.modify(|_r, w| { unsafe { w.$tXcen().set_bit() }});

                // Enable output 1
                //self.enable();

                // Enable output 2
                //common.oenr.modify(|_r, w| { unsafe { w.tX2oen().set_bit() } });

                todo!()
            }
        )+
    }
}

macro_rules! hrtim_pin_hal {
    ($($TIMX:ident:
        ($CH:ty, $ccxe:ident, $ccxp:ident, $ccmrx_output:ident, $ocxpe:ident, $ocxm:ident,
         $ccrx:ident, $typ:ident $(,$ccxne:ident, $ccxnp:ident)*),)+
     ) => {
        $(
            impl<COMP, POL, NPOL> hal::PwmPin for Pwm<$TIMX, $CH, COMP, POL, NPOL>
                where Pwm<$TIMX, $CH, COMP, POL, NPOL>: PwmPinEnable {
                type Duty = u16;

                // You may not access self in the following methods!
                // See unsafe above

                fn disable(&mut self) {
                    // TODO: How do we do this while preventing data race due to shared register

                    // Disable output Y on channel X
                    //HRTIM_COMMON.oenr.modify(|_r, w| { unsafe { w.$tXYoen().clear_bit() } });
                }

                fn enable(&mut self) {
                    // TODO: How do we do this while preventing data race due to shared register

                    // Enable output Y on channel X
                    //HRTIM_COMMON.oenr.modify(|_r, w| { unsafe { w.$tXYoen().set_bit() } });
                }

                fn get_duty(&self) -> Self::Duty {
                    let tim = unsafe { &*$TIMX::ptr() };

                    // Even though the field is 20 bits long for 16-bit counters, only 16 bits are
                    // valid, so we convert to the appropriate type.
                    tim.$ccrx().read().ccr().bits() as $typ
                }

                fn get_max_duty(&self) -> Self::Duty {
                    let tim = unsafe { &*$TIMX::ptr() };

                    // Even though the field is 20 bits long for 16-bit counters, only 16 bits are
                    // valid, so we convert to the appropriate type.
                    let arr = tim.arr.read().arr().bits() as $typ;

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

                    tim.$ccrx().write(|w| unsafe { w.ccr().bits(duty.into()) });
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
    HRTIM_TIMF: (hrtim_timf, timfcr, perfr, tfcen),
}

hrtim_pin_hal! {
    HRTIM_TIMA: (CH1<PSCL>),
    HRTIM_TIMA: (CH2<PSCL>)
}

trait HrtimPrescaler {
    const BITS: u8;
    const VALUE: u8;
}

macro_rules! impl_pscl {
    ($($t:ident => $b:literal, $c:literal,)+) => {$(
        struct $t;
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
