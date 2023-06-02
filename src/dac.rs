//! DAC
//!
//! ## Origin
//!
//! This code has been taken from the stm32g0xx-hal project and modified to support
//! STM32G4xx MCUs.

use core::marker::PhantomData;
use core::mem::MaybeUninit;

use crate::gpio::gpioa::{PA4, PA5, PA6};
use crate::gpio::DefaultMode;
use crate::rcc::{self, *};
use crate::stm32::{DAC1, DAC2, DAC3, DAC4, RCC};
use hal::blocking::delay::DelayUs;

pub trait DacOut<V> {
    fn set_value(&mut self, val: V);
    fn get_value(&mut self) -> V;
}

pub struct GeneratorConfig {
    mode: u8,
    amp: u8,
}

impl GeneratorConfig {
    pub fn triangle(amplitude: u8) -> Self {
        Self {
            mode: 0b10,
            amp: amplitude,
        }
    }

    pub fn sawtooth(amplitude: u8) -> Self {
        Self {
            mode: 0b11,
            amp: amplitude,
        }
    }

    pub fn noise(seed: u8) -> Self {
        Self {
            mode: 0b01,
            amp: seed,
        }
    }
}

/// Enabled DAC (type state)
pub struct Enabled;
// / Enabled DAC without output buffer (type state)
//pub struct EnabledUnbuffered;
/// Enabled DAC wave generator (type state)
pub struct WaveGenerator;
/// Disabled DAC (type state)
pub struct Disabled;

pub trait ED {}
impl ED for Enabled {}
impl ED for EnabledUnbuffered {}
impl ED for WaveGenerator {}
impl ED for Disabled {}

macro_rules! impl_dac {
    ($DACxCHy:ident) => {
        pub struct $DACxCHy<const MODE_BITS: u8, ED> {
            _enabled: PhantomData<ED>,
        }
    };
}

impl_dac!(Dac1Ch1);
impl_dac!(Dac1Ch2);
impl_dac!(Dac2Ch1); // DAC2 only has 1 channel
impl_dac!(Dac3Ch1);
impl_dac!(Dac3Ch2);
impl_dac!(Dac4Ch1);
impl_dac!(Dac4Ch2);

/// Trait for GPIO pins that can be converted to DAC output pins
pub trait Pins<DAC> {
    type Output;
}

/// Dac output mode: external pin only
pub const M_EXT_PIN: u8 = 0b000;

/// Dac output mode: internal signal and external pin
pub const M_MIX_SIG: u8 = 0b001;

/// Dac output mode: internal signal only
pub const M_INT_SIG: u8 = 0b011;

pub struct Dac1IntSig1;
pub struct Dac1IntSig2;
pub struct Dac2IntSig1;
pub struct Dac3IntSig1;
pub struct Dac3IntSig2;
pub struct Dac4IntSig1;
pub struct Dac4IntSig2;

macro_rules! impl_pin_for_dac {
    ($DAC:ident: $pin:ty, $output:ty) => {
        #[allow(unused_parens)]
        impl Pins<$DAC> for $pin {
            #[allow(unused_parens)]
            type Output = $output;
        }
    };
}

// Implement all combinations of ch2 for the specified ch1 on DAC1
macro_rules! impl_dac1_ch2_combos {
    ($($pin_ch1:ty, $output_ch1:ty)*) => {
        $(impl_pin_for_dac!(DAC1: $pin_ch1,                                   // ch2: Not used
            $output_ch1
        );)*
        impl_pin_for_dac!(DAC1: ($($pin_ch1,)* PA5<DefaultMode>),             // ch2: Ext pin
            ($($output_ch1,)* Dac1Ch2<M_EXT_PIN, Disabled>)
        );
        impl_pin_for_dac!(DAC1: ($($pin_ch1,)* Dac1IntSig2),                    // ch2: Internal
            ($($output_ch1,)* Dac1Ch2<M_INT_SIG, Disabled>)
        );
        impl_pin_for_dac!(DAC1: ($($pin_ch1,)* (PA5<DefaultMode>, Dac1IntSig2)),// ch2: Mixed
            ($($output_ch1,)* Dac1Ch2<M_MIX_SIG, Disabled>)
        );
    };
}

impl_dac1_ch2_combos!(); // ch1: Not used
impl_dac1_ch2_combos!(PA4<DefaultMode>, Dac1Ch1<M_EXT_PIN, Disabled>); // ch1: Ext pin
impl_dac1_ch2_combos!(Dac1IntSig1, Dac1Ch1<M_INT_SIG, Disabled>); // ch1: Internal
impl_dac1_ch2_combos!((PA4<DefaultMode>, Dac1IntSig1), Dac1Ch1<M_MIX_SIG, Disabled>); // ch1: Mixed

// DAC2
impl_pin_for_dac!(DAC2: PA6<DefaultMode>, Dac2Ch1<M_EXT_PIN, Disabled>); // ch1: Ext pin
impl_pin_for_dac!(DAC2: Dac2IntSig1, Dac2Ch1<M_INT_SIG, Disabled>); // ch1: Internal
impl_pin_for_dac!(DAC2: (PA6<DefaultMode>, Dac2IntSig1), Dac2Ch1<M_MIX_SIG, Disabled>); // ch1: Mixed

// DAC3 int
impl_pin_for_dac!(DAC3: Dac3IntSig1, Dac3Ch1<M_INT_SIG, Disabled>);
impl_pin_for_dac!(DAC3: Dac3IntSig2, Dac3Ch2<M_INT_SIG, Disabled>);

// DAC4 int
impl_pin_for_dac!(DAC4: Dac4IntSig1, Dac4Ch1<M_INT_SIG, Disabled>);
impl_pin_for_dac!(DAC4: Dac4IntSig2, Dac4Ch2<M_INT_SIG, Disabled>);

pub fn dac<DAC, PINS>(_dac: DAC, _pins: PINS, _rcc: &mut Rcc) -> PINS::Output
where
    DAC: rcc::Enable + rcc::Reset,
    PINS: Pins<DAC>,
{
    unsafe {
        let rcc_ptr = &(*RCC::ptr());
        DAC::enable(rcc_ptr);
        DAC::reset(rcc_ptr);
    }

    #[allow(clippy::uninit_assumed_init)]
    unsafe {
        MaybeUninit::uninit().assume_init()
    }
}

macro_rules! dac_helper {
    ($($CX:ident: $DAC:ty: (
        $en:ident,
        $cen:ident,
        $cal_flag:ident,
        $trim:ident,
        $mode:ident,
        $dhrx:ident,
        $dac_dor:ident,
        $daccxdhr:ident,
        $wave:ident,
        $mamp:ident,
        $ten:ident,
        $swtrig:ident
    ),)+) => {
        $(
            impl<const MODE_BITS: u8> $CX<MODE_BITS, Disabled> {
                pub fn enable(self) -> $CX<MODE_BITS, Enabled> {
                    let dac = unsafe { &(*<$DAC>::ptr()) };

                    dac.dac_mcr.modify(|_, w| unsafe { w.$mode().bits(MODE_BITS) });
                    dac.dac_cr.modify(|_, w| w.$en().set_bit());

                    $CX {
                        _enabled: PhantomData,
                    }
                }

                /*pub fn enable_unbuffered(self) -> $CX<MODE_BITS, EnabledUnbuffered> {
                    let dac = unsafe { &(*<$DAC>::ptr()) };

                    dac.dac_mcr.modify(|_, w| unsafe { w.$mode().bits(2) });
                    dac.dac_cr.modify(|_, w| w.$en().set_bit());

                    $CX {
                        _enabled: PhantomData,
                    }
                }*/

                pub fn enable_generator(self, config: GeneratorConfig) -> $CX<MODE_BITS, WaveGenerator> {
                    let dac = unsafe { &(*<$DAC>::ptr()) };

                    dac.dac_mcr.modify(|_, w| unsafe { w.$mode().bits(MODE_BITS) });
                    dac.dac_cr.modify(|_, w| unsafe {
                        w.$wave().bits(config.mode);
                        w.$ten().set_bit();
                        w.$mamp().bits(config.amp);
                        w.$en().set_bit()
                    });

                    $CX {
                        _enabled: PhantomData,
                    }
                }
            }

            impl<const MODE_BITS: u8, ED> $CX<MODE_BITS, ED> {
                /// Calibrate the DAC output buffer by performing a "User
                /// trimming" operation. It is useful when the VDDA/VREF+
                /// voltage or temperature differ from the factory trimming
                /// conditions.
                ///
                /// The calibration is only valid when the DAC channel is
                /// operating with the buffer enabled. If applied in other
                /// modes it has no effect.
                ///
                /// After the calibration operation, the DAC channel is
                /// disabled.
                pub fn calibrate_buffer<T>(self, delay: &mut T) -> $CX<MODE_BITS, Disabled>
                where
                    T: DelayUs<u32>,
                {
                    let dac = unsafe { &(*<$DAC>::ptr()) };
                    dac.dac_cr.modify(|_, w| w.$en().clear_bit());
                    dac.dac_mcr.modify(|_, w| unsafe { w.$mode().bits(0) });
                    dac.dac_cr.modify(|_, w| w.$cen().set_bit());
                    let mut trim = 0;
                    while true {
                        dac.dac_ccr.modify(|_, w| unsafe { w.$trim().bits(trim) });
                        delay.delay_us(64_u32);
                        if dac.dac_sr.read().$cal_flag().bit() {
                            break;
                        }
                        trim += 1;
                    }
                    dac.dac_cr.modify(|_, w| w.$cen().clear_bit());

                    $CX {
                        _enabled: PhantomData,
                    }
                }

                /// Disable the DAC channel
                pub fn disable(self) -> $CX<MODE_BITS, Disabled> {
                    let dac = unsafe { &(*<$DAC>::ptr()) };
                    dac.dac_cr.modify(|_, w| unsafe {
                        w.$en().clear_bit().$wave().bits(0).$ten().clear_bit()
                    });

                    $CX {
                        _enabled: PhantomData,
                    }
                }
            }

            /// DacOut implementation available in any Enabled/Disabled
            /// state
            impl<const MODE_BITS: u8, ED> DacOut<u16> for $CX<MODE_BITS, ED> {
                fn set_value(&mut self, val: u16) {
                    let dac = unsafe { &(*<$DAC>::ptr()) };
                    dac.$dhrx.write(|w| unsafe { w.bits(val as u32) });
                }

                fn get_value(&mut self) -> u16 {
                    let dac = unsafe { &(*<$DAC>::ptr()) };
                    dac.$dac_dor.read().bits() as u16
                }
            }

            /// Wave generator state implementation
            impl<const MODE_BITS: u8> $CX<MODE_BITS, WaveGenerator> {
                pub fn trigger(&mut self) {
                    let dac = unsafe { &(*<$DAC>::ptr()) };
                    dac.dac_swtrgr.write(|w| { w.$swtrig().set_bit() });
                }
            }
        )+
    };
}

macro_rules! dac {
    ($($DAC:ident ch1: $DACxCH1:ident $(, ch2: $DACxCH2:ident)*)+) => {$(
        dac_helper!{$DACxCH1: $DAC: (
            en1,
            cen1,
            cal_flag1,
            otrim1,
            mode1,
            dac_dhr12r1,
            dac_dor1,
            dacc1dhr,
            wave1,
            mamp1,
            ten1,
            swtrig1
        ),
        $($DACxCH2: $DAC: (
            en2,
            cen2,
            cal_flag2,
            otrim2,
            mode2,
            dac_dhr12r2,
            dac_dor2,
            dacc2dhr,
            wave2,
            mamp2,
            ten2,
            swtrig2
        ),)*}
    )+};
}

pub trait DacExt: Sized {
    fn constrain<PINS>(self, pins: PINS, rcc: &mut Rcc) -> PINS::Output
    where
        PINS: Pins<Self>;
}

macro_rules! impl_dac_ext {
    ($($DAC:ty, )+) => {$(
        impl DacExt for $DAC {
            fn constrain<PINS>(self, pins: PINS, rcc: &mut Rcc) -> PINS::Output
            where
                PINS: Pins<$DAC>,
            {
                dac(self, pins, rcc)
            }
        }
    )+};
}

impl_dac_ext!(DAC1, DAC2, DAC3, DAC4,);

dac!(
    DAC1 ch1: Dac1Ch1, ch2: Dac1Ch2
    DAC2 ch1: Dac2Ch1
    DAC3 ch1: Dac3Ch1, ch2: Dac3Ch2
    DAC4 ch1: Dac4Ch1, ch2: Dac4Ch2
);
