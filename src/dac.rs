//! DAC
//!
//! ## Origin
//!
//! This code has been taken from the stm32g0xx-hal project and modified to support
//! STM32G4xx MCUs.

use core::marker::PhantomData;
use core::mem::MaybeUninit;
use core::ops::Deref;

use crate::gpio::{Analog, PA4, PA5, PA6};
use crate::pac;
use crate::rcc::{self, *};
use crate::stm32::dac1::mcr::HFSEL;
use embedded_hal::delay::DelayNs;

pub trait DacOut<V> {
    fn get_value(&mut self) -> V;
    fn set_value(&mut self, val: V);
}
impl<V, D: DacOutGet<V> + DacOutSet<V>> DacOut<V> for D {
    fn get_value(&mut self) -> V {
        DacOutGet::get_value(self)
    }
    fn set_value(&mut self, val: V) {
        DacOutSet::set_value(self, val);
    }
}
pub trait DacOutSet<V> {
    fn set_value(&mut self, val: V);
}
pub trait DacOutGet<V> {
    fn get_value(&mut self) -> V;
}

#[derive(Copy, Clone, Debug, PartialEq)]
pub enum CountingDirection {
    Decrement = 0,
    Increment = 1,
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

    pub fn noise(seed: u8) -> Self {
        Self {
            mode: 0b01,
            amp: seed,
        }
    }
}

/// Used as regular trigger source and sawtooth generator reset trigger source
///
/// # Safety
/// This trait should only be implemented for valid trigger sources with correct
/// bit patterns.
pub unsafe trait TriggerSource {
    /// See stinctrigsel in reference manual for bit patterns
    const BITS: u8;
}

/// Used as sawtooth generator inc trigger source
/// # Safety
/// This trait should only be implemented for valid trigger sources with correct
/// bit patterns.
pub unsafe trait IncTriggerSource {
    /// See strsttrigsel in reference manual for bit patterns
    const BITS: u8;
}

// https://www.youtube.com/watch?v=fUQaHyaXikw&ab_channel=STMicroelectronics
#[derive(Debug, Clone, Copy)]
pub struct SawtoothConfig {
    /// The sawtooth counter initial value (12 bits)
    reset_value: u16,

    dir: CountingDirection,
    step_size: u16,

    inc_trigger: u8,
    reset_trigger: u8,
}

impl Default for SawtoothConfig {
    fn default() -> Self {
        Self::with_slope(CountingDirection::Decrement, 0x10)
    }
}

impl SawtoothConfig {
    /// NOTE: The DAC output is used from 12 MSBs of the sawtooth counter so
    /// a step_size of 1 will only increment the output by 1 every 16th inc trigger
    pub const fn with_slope(dir: CountingDirection, step_size: u16) -> Self {
        Self {
            reset_value: u16::MAX,
            dir,
            step_size,
            inc_trigger: 0b0000,   // SW trigger using `trigger_inc`-method
            reset_trigger: 0b0000, // SW trigger using `trigger_reset`-method
        }
    }

    /// The sawtooth counter reset value (12 bits)
    pub const fn reset_value(mut self, reset_value: u16) -> Self {
        self.reset_value = reset_value;
        self
    }

    pub const fn slope(mut self, dir: CountingDirection, step_size: u16) -> Self {
        self.dir = dir;
        self.step_size = step_size;
        self
    }

    pub fn inc_trigger<T: IncTriggerSource>(mut self, _inc_trigger: T) -> Self {
        self.inc_trigger = T::BITS;
        self
    }

    pub fn reset_trigger<T: TriggerSource>(mut self, _reset_trigger: T) -> Self {
        self.reset_trigger = T::BITS;
        self
    }
}

/// Enabled DAC (type state)
pub struct Enabled;
// / Enabled DAC without output buffer (type state)
//pub struct EnabledUnbuffered;
/// Enabled DAC wave generator for triangle or noise wave form (type state)
pub struct WaveGenerator;
/// Enabled DAC wave generator for sawtooth wave form (type state)
pub struct SawtoothGenerator;
/// Disabled DAC (type state)
pub struct Disabled;

pub trait Generator {}
impl Generator for WaveGenerator {}
impl Generator for SawtoothConfig {}

pub trait Instance:
    rcc::Enable
    + rcc::Reset
    + crate::Ptr<RB = crate::pac::dac1::RegisterBlock>
    + Deref<Target = Self::RB>
{
}

pub struct DacCh<DAC: Instance, const CH: u8, const MODE_BITS: u8, ED> {
    _marker: PhantomData<(DAC, ED)>,
}

impl<DAC: Instance, const CH: u8, const MODE_BITS: u8, ED> DacCh<DAC, CH, MODE_BITS, ED> {
    fn new() -> Self {
        Self {
            _marker: PhantomData,
        }
    }
}

macro_rules! impl_dac {
    ($DACxCHy:ident, $DAC:ty, $CH:literal) => {
        pub type $DACxCHy<const MODE_BITS: u8, ED> = DacCh<$DAC, $CH, MODE_BITS, ED>;
    };
}

impl_dac!(Dac1Ch1, pac::DAC1, 0);
impl_dac!(Dac1Ch2, pac::DAC1, 1);
impl_dac!(Dac2Ch1, pac::DAC2, 0); // DAC2 only has 1 channel
impl_dac!(Dac3Ch1, pac::DAC3, 0);
impl_dac!(Dac3Ch2, pac::DAC3, 1);
impl_dac!(Dac4Ch1, pac::DAC4, 0);
impl_dac!(Dac4Ch2, pac::DAC4, 1);

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
        impl Pins<pac::$DAC> for $pin {
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
        impl_pin_for_dac!(DAC1: ($($pin_ch1,)* PA5<Analog>),             // ch2: Ext pin
            ($($output_ch1,)* Dac1Ch2<M_EXT_PIN, Disabled>)
        );
        impl_pin_for_dac!(DAC1: ($($pin_ch1,)* Dac1IntSig2),                    // ch2: Internal
            ($($output_ch1,)* Dac1Ch2<M_INT_SIG, Disabled>)
        );
        impl_pin_for_dac!(DAC1: ($($pin_ch1,)* (PA5<Analog>, Dac1IntSig2)),// ch2: Mixed
            ($($output_ch1,)* Dac1Ch2<M_MIX_SIG, Disabled>)
        );
    };
}

impl_dac1_ch2_combos!(); // ch1: Not used
impl_dac1_ch2_combos!(PA4<Analog>, Dac1Ch1<M_EXT_PIN, Disabled>); // ch1: Ext pin
impl_dac1_ch2_combos!(Dac1IntSig1, Dac1Ch1<M_INT_SIG, Disabled>); // ch1: Internal
impl_dac1_ch2_combos!((PA4<Analog>, Dac1IntSig1), Dac1Ch1<M_MIX_SIG, Disabled>); // ch1: Mixed

// DAC2
impl_pin_for_dac!(DAC2: PA6<Analog>, Dac2Ch1<M_EXT_PIN, Disabled>); // ch1: Ext pin
impl_pin_for_dac!(DAC2: Dac2IntSig1, Dac2Ch1<M_INT_SIG, Disabled>); // ch1: Internal
impl_pin_for_dac!(DAC2: (PA6<Analog>, Dac2IntSig1), Dac2Ch1<M_MIX_SIG, Disabled>); // ch1: Mixed

// DAC3 int
impl_pin_for_dac!(DAC3: Dac3IntSig1, Dac3Ch1<M_INT_SIG, Disabled>);
impl_pin_for_dac!(DAC3: Dac3IntSig2, Dac3Ch2<M_INT_SIG, Disabled>);
impl_pin_for_dac!(
    DAC3: (Dac3IntSig1, Dac3IntSig2),
    (Dac3Ch1<M_INT_SIG, Disabled>, Dac3Ch2<M_INT_SIG, Disabled>)
);

// DAC4 int
impl_pin_for_dac!(DAC4: Dac4IntSig1, Dac4Ch1<M_INT_SIG, Disabled>);
impl_pin_for_dac!(DAC4: Dac4IntSig2, Dac4Ch2<M_INT_SIG, Disabled>);
impl_pin_for_dac!(
    DAC4: (Dac4IntSig1, Dac4IntSig2),
    (Dac4Ch1<M_INT_SIG, Disabled>, Dac4Ch2<M_INT_SIG, Disabled>)
);

pub fn hfsel(rcc: &Rcc) -> HFSEL {
    match rcc.clocks.ahb_clk.to_MHz() {
        0..80 => pac::dac1::mcr::HFSEL::Disabled,
        80..160 => pac::dac1::mcr::HFSEL::More80mhz,
        160.. => pac::dac1::mcr::HFSEL::More160mhz,
    }
}

impl<DAC: Instance, const CH: u8, const MODE_BITS: u8> DacCh<DAC, CH, MODE_BITS, Disabled> {
    /// TODO: The DAC does not seem to work unless `calibrate_buffer` has been callen
    /// even when only using dac output internally
    pub fn enable(self, rcc: &mut Rcc) -> DacCh<DAC, CH, MODE_BITS, Enabled> {
        // We require rcc here just to ensure exclusive access to registers common to ch1 and ch2
        let dac = unsafe { &(*DAC::ptr()) };

        dac.mcr()
            .modify(|_, w| unsafe { w.hfsel().variant(hfsel(rcc)).mode(CH).bits(MODE_BITS) });
        dac.cr().modify(|_, w| w.en(CH).set_bit());

        DacCh::new()
    }

    pub fn enable_generator(
        self,
        config: GeneratorConfig,
        rcc: &mut Rcc,
    ) -> DacCh<DAC, CH, MODE_BITS, WaveGenerator> {
        // We require rcc here just to ensure exclusive access to registers common to ch1 and ch2
        let dac = unsafe { &(*DAC::ptr()) };

        dac.mcr()
            .modify(|_, w| unsafe { w.hfsel().variant(hfsel(rcc)).mode(CH).bits(MODE_BITS) });
        dac.cr().modify(|_, w| unsafe {
            w.wave(CH).bits(config.mode);
            w.ten(CH).set_bit();
            w.mamp(CH).bits(config.amp);
            w.en(CH).set_bit()
        });

        DacCh::new()
    }

    pub fn enable_sawtooth_generator(
        self,
        config: SawtoothConfig,
        rcc: &mut Rcc,
    ) -> DacCh<DAC, CH, MODE_BITS, SawtoothGenerator> {
        // TODO: We require rcc here just to ensure exclusive access to registers common to ch1 and ch2
        let dac = unsafe { &(*DAC::ptr()) };

        dac.mcr()
            .modify(|_, w| unsafe { w.hfsel().variant(hfsel(rcc)).mode(CH).bits(MODE_BITS) });

        unsafe {
            dac.stmodr().modify(|_, w| {
                w.stinctrigsel(CH)
                    .bits(config.inc_trigger)
                    .strsttrigsel(CH)
                    .bits(config.reset_trigger)
            });
        }

        dac.cr().modify(|_, w| unsafe { w.wave(CH).bits(0b11) });

        unsafe {
            dac.str(CH as usize).write(|w| {
                w.stdir()
                    .bit(config.dir == CountingDirection::Increment)
                    .stincdata()
                    .bits(config.step_size)
                    .strstdata()
                    .bits(config.reset_value)
            });
        }

        dac.cr().modify(|_, w| w.en(CH).set_bit());
        while dac.sr().read().dacrdy(CH).bit_is_clear() {}

        DacCh::new()
    }
}

impl<DAC: Instance, const CH: u8, const MODE_BITS: u8, ED> DacCh<DAC, CH, MODE_BITS, ED> {
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
    pub fn calibrate_buffer<D>(self, delay: &mut D) -> DacCh<DAC, CH, MODE_BITS, Disabled>
    where
        D: DelayNs,
    {
        let dac = unsafe { &(*DAC::ptr()) };
        dac.cr().modify(|_, w| w.en(CH).clear_bit());
        dac.mcr().modify(|_, w| unsafe { w.mode(CH).bits(0) });
        dac.cr().modify(|_, w| w.cen(CH).set_bit());
        let mut trim = 0;
        loop {
            dac.ccr().modify(|_, w| unsafe { w.otrim(CH).bits(trim) });
            delay.delay_us(64_u32);
            if dac.sr().read().cal_flag(CH).bit() {
                break;
            }
            trim += 1;
        }
        dac.cr().modify(|_, w| w.cen(CH).clear_bit());

        DacCh::new()
    }

    /// Disable the DAC channel
    pub fn disable(self) -> DacCh<DAC, CH, MODE_BITS, Disabled> {
        let dac = unsafe { &(*DAC::ptr()) };
        dac.cr()
            .modify(|_, w| unsafe { w.en(CH).clear_bit().wave(CH).bits(0).ten(CH).clear_bit() });

        DacCh::new()
    }
}

/// DacOut implementation available in any Enabled/Disabled
/// state
impl<DAC: Instance, const CH: u8, const MODE_BITS: u8> DacOutSet<u16>
    for DacCh<DAC, CH, MODE_BITS, Enabled>
{
    fn set_value(&mut self, val: u16) {
        let dac = unsafe { &(*DAC::ptr()) };
        dac.dhr12r(CH as usize)
            .write(|w| unsafe { w.bits(val as u32) });
    }
}

impl<DAC: Instance, const CH: u8, const MODE_BITS: u8, ED> DacOutGet<u16>
    for DacCh<DAC, CH, MODE_BITS, ED>
{
    fn get_value(&mut self) -> u16 {
        let dac = unsafe { &(*DAC::ptr()) };
        dac.dor(CH as usize).read().bits() as u16
    }
}

impl<DAC: Instance, const CH: u8, const MODE_BITS: u8> DacOutSet<u16>
    for DacCh<DAC, CH, MODE_BITS, SawtoothGenerator>
{
    fn set_value(&mut self, reset_value: u16) {
        let dac = unsafe { &(*<DAC>::ptr()) };
        dac.str(CH as usize)
            .modify(|_r, w| unsafe { w.strstdata().bits(reset_value) });
    }
}

/// Wave generator state implementation
impl<DAC: Instance, const CH: u8, const MODE_BITS: u8> DacCh<DAC, CH, MODE_BITS, WaveGenerator> {
    pub fn trigger(&mut self) {
        let dac = unsafe { &(*<DAC>::ptr()) };
        dac.swtrgr().write(|w| w.swtrig(CH).set_bit());
    }
}

impl<DAC: Instance, const CH: u8, const MODE_BITS: u8, ED> crate::stasis::Freeze
    for DacCh<DAC, CH, MODE_BITS, ED>
{
}

/// Sawtooth generator state implementation
impl<DAC: Instance, const CH: u8, const MODE_BITS: u8>
    DacCh<DAC, CH, MODE_BITS, SawtoothGenerator>
{
    pub fn trigger_reset(&mut self) {
        let dac = unsafe { &(*<DAC>::ptr()) };
        dac.swtrgr().write(|w| w.swtrig(CH).set_bit());
    }

    pub fn trigger_inc(&mut self) {
        let dac = unsafe { &(*<DAC>::ptr()) };
        // TODO: Update once arrayified
        if CH == 0 {
            dac.swtrgr().write(|w| w.swtrigb1().set_bit());
        } else {
            dac.swtrgr().write(|w| w.swtrigb2().set_bit());
        }
    }
}

pub trait DacExt: Sized {
    fn constrain<PINS>(self, pins: PINS, rcc: &mut Rcc) -> PINS::Output
    where
        PINS: Pins<Self>;
}

impl<DAC: Instance> DacExt for DAC {
    fn constrain<PINS>(self, _pins: PINS, rcc: &mut Rcc) -> PINS::Output
    where
        PINS: Pins<Self>,
    {
        Self::enable(rcc);
        Self::reset(rcc);

        #[allow(clippy::uninit_assumed_init)]
        unsafe {
            MaybeUninit::uninit().assume_init()
        }
    }
}

macro_rules! impl_dac_ext {
    ($($DAC:ty, )+) => {$(
        impl Instance for $DAC {}
    )+};
}

impl_dac_ext!(pac::DAC1, pac::DAC2, pac::DAC3, pac::DAC4,);
