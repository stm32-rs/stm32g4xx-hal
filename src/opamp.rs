#![deny(missing_docs)]

//! Integrated opamps.
//! ```
//! #![no_std]
//! #![no_main]
//!
//! extern crate panic_halt;
//!
//! use stm32g4xx_hal::opamp::{
//!     InternalOutput, IntoFollower, IntoOpenLoop, IntoPga, Gain, PgaModeInternal,
//! };
//! use stm32g4xx_hal::prelude::*;
//! use stm32g4xx_hal::pwr::PwrExt;
//!
//! #[cortex_m_rt::entry]
//! fn main() -> ! {
//!     // take peripherals
//!     let dp = stm32g4xx_hal::stm32::Peripherals::take().unwrap();
//!
//!     // setup clock
//!     let config = stm32g4xx_hal::rcc::Config::hsi();
//!     let pwr_cfg = dp.PWR.constrain().freeze();
//!     let mut rcc = dp.RCC.freeze(config, pwr_cfg);
//!
//!     // split gpio
//!     let gpioa = dp.GPIOA.split(&mut rcc);
//!     let gpiob = dp.GPIOB.split(&mut rcc);
//!
//!     // setup opamps
//!     let (opamp1, opamp2, opamp3, opamp4, _opamp5, _opamp6) = dp.OPAMP.split(&mut rcc);
//!
//!     let opamp1 = opamp1.follower(gpioa.pa1, gpioa.pa2);
//!     let opamp2 = opamp2.follower(gpioa.pa7, InternalOutput);
//!
//!     let opamp3 = opamp3.open_loop(gpiob.pb0, gpiob.pb2, gpiob.pb1);
//!     let opamp4 = opamp4.open_loop(gpiob.pb11, gpiob.pb10, InternalOutput);
//!
//!     // disable opamps
//!     let (opamp1, pa1, pa2) = opamp1.disable();
//!     let (opamp2, pa7) = opamp2.disable();
//!
//!     let (_opamp3, _pb0, _pb2, _pb1) = opamp3.disable();
//!     let (_opamp4, _pb11, _pb10) = opamp4.disable();
//!
//!     let opamp1 = opamp1.pga(pa1, PgaModeInternal::gain(Gain::Gain2), pa2);
//!     let opamp2 = opamp2.pga(
//!         pa7,
//!         PgaModeInternal::gain(Gain::Gain2),
//!         InternalOutput,
//!     );
//!
//!     let (_opamp1, _pa1, _pa2) = opamp1.disable();
//!     let (_opamp2, _pa7) = opamp2.disable();
//!
//!     loop {}
//! }
//! ```

// TODO: Add support for calibration

use core::marker::PhantomData;
use proto_hal::stasis;

/// PGA Gain
pub enum Gain {
    /// 2x Gain in non-inverting modes, -1x gain in inverting modes.
    Gain2 = 0,
    /// 4x Gain in non-inverting modes, -3x gain in inverting modes.
    Gain4 = 1,
    /// 8x Gain in non-inverting modes, -7x gain in inverting modes.
    Gain8 = 2,
    /// 16x Gain in non-inverting modes, -15x gain in inverting modes.
    Gain16 = 3,
    /// 32x Gain in non-inverting modes, -31x gain in inverting modes.
    Gain32 = 4,
    /// 64x Gain in non-inverting modes, -63x gain in inverting modes.
    Gain64 = 5,
}

/// State type for disabled opamp.
pub struct Disabled<Opamp> {
    opamp: PhantomData<Opamp>,
}
/// State type for opamp running in voltage follower mode.
pub struct Follower<Opamp, Input, Output> {
    opamp: PhantomData<Opamp>,
    input: Input,
    output: Output,
}
/// State type for opamp running in open-loop mode.
pub struct OpenLoop<Opamp, NonInverting, Inverting, Output> {
    opamp: PhantomData<Opamp>,
    non_inverting: NonInverting,
    inverting: Inverting,
    output: Output,
}
/// State type for opamp running in programmable-gain mode.
pub struct Pga<Opamp, NonInverting, Output> {
    opamp: PhantomData<Opamp>,
    non_inverting: PhantomData<NonInverting>,
    output: Output,
}
/// State type for an opamp that has been locked.
pub struct Locked<Opamp, Output> {
    opamp: PhantomData<Opamp>,
    output: PhantomData<Output>,
}

/// Represents the output when the opamp is connected to the
/// ADC internally, and not connected to a pin.
pub struct InternalOutput;

/// Trait for opamps that can be run in follower mode.
pub trait IntoFollower<Opamp, Input, Output> {
    /// Coonfigures the opamp as voltage follower.
    fn follower(self, input: Input) -> Follower<Opamp, Input, InternalOutput>;
}
/// Trait for opamps that can be run in open-loop mode.
pub trait IntoOpenLoop<Opamp, NonInverting, Inverting> {
    /// Configures the opamp for open-loop operation.
    fn open_loop(
        self,
        non_inverting: NonInverting,
        inverting: Inverting,
    ) -> OpenLoop<Opamp, NonInverting, Inverting, InternalOutput>;
}
/// Trait for opamps that can be run in programmable gain mode.
#[allow(private_bounds)]
pub trait IntoPga<Opamp, NonInverting>
where
    Opamp: ConfigurePgaReg<Opamp, NonInverting> + LookupPgaGain,
{
    /// Configures the opamp for programmable gain operation.
    fn pga(
        self,
        non_inverting: NonInverting,
        gain: Gain,
    ) -> Pga<Opamp, NonInverting, InternalOutput>;

    /// Trait for opamps that can be run in programmable gain mode,
    /// with external filtering.
    fn pga_external_filter<F>(
        self,
        non_inverting: NonInverting,
        filter: F,
        gain: Gain,
    ) -> Pga<Opamp, NonInverting, InternalOutput>
    where
        F: stasis::EntitlementLock<Resource = Opamp::Vinm0>,
        Opamp::Vinm0: stasis::Freeze;

    /// Configures the opamp for programmable gain operation, with
    /// external filtering.
    fn pga_external_bias<N>(
        self,
        non_inverting: NonInverting,
        inverting: N,
        gain: Gain,
    ) -> Pga<Opamp, NonInverting, InternalOutput>
    where
        N: stasis::EntitlementLock<Resource = Opamp::Vinm0>,
        Opamp::Vinm0: stasis::Freeze;

    /// Configures the opamp for programmable gain operation, with
    /// external filtering.
    fn pga_external_bias_and_filter<N, F>(
        self,
        non_inverting: NonInverting,
        inverting: N,
        filter: F,
        gain: Gain,
    ) -> Pga<Opamp, NonInverting, InternalOutput>
    where
        N: stasis::EntitlementLock<Resource = Opamp::Vinm0>,
        F: stasis::EntitlementLock<Resource = Opamp::Vinm1>,
        Opamp::Vinm0: stasis::Freeze,
        Opamp::Vinm1: stasis::Freeze;
}

/// Internal trait implementing the low level register write used to
/// configure the PGA.
trait ConfigurePgaReg<Opamp, NonInverting>
where
    Opamp: LookupPgaGain,
{
    /// Type of the associated vinm0 input.
    type Vinm0: stasis::Freeze;
    /// Type of the associated vinm1 input.
    type Vinm1: stasis::Freeze;

    /// Write the opamp CSR register configuring the opamp PGA.
    ///
    /// Safety: This is a raw register access.
    unsafe fn write_pga_reg(gain: Gain, mode: PgaMode);

    /// Configure the
    ///
    fn configure_pga(gain: Gain, mode: PgaMode) -> Pga<Opamp, NonInverting, InternalOutput> {
        unsafe { Self::write_pga_reg(gain, mode) };
        Pga {
            opamp: PhantomData,
            non_inverting: PhantomData,
            output: InternalOutput,
        }
    }
}

/// Internal enum, used when converting [`Gain`] to
/// register values
enum PgaMode {
    Pga,
    PgaExternalFilter,
    PgaExternalBias,
    PgaExternalBiasAndFilter,
}

/// Internal trait, implemented on each opamp struct
/// and used to lookup internal register values based
/// on [`Gain`].
trait LookupPgaGain {
    /// Type of the register that is returned (this is specific
    /// to each opamp instance)
    type PgaGainReg;

    /// Lookup table to convert a [`PgaMode`] and [`Gain`] to
    /// the internal register enumeration.
    fn pga_gain(mode: PgaMode, gain: Gain) -> Self::PgaGainReg;
}

/// A signal source that can be connected to the `OPAMP`'s non inverting input
pub trait NonInverting<OPAMP> {
    /// Type for selecting this as the non inverting input of this opamp
    type VP_SEL;
    /// Configuration value for selecting this as the non inverting input of this opamp
    const VP_SEL: Self::VP_SEL;
}

/// A signal source that can be connected to the `OPAMP`'s inverting input
pub trait Inverting<OPAMP> {
    /// Type for selecting this as the inverting input of this opamp
    type VM_SEL;
    /// Configuration value for selecting this as the inverting input of this opamp
    const VM_SEL: Self::VM_SEL;
}

macro_rules! impl_inverting {
    ($opamp:ty, [$($t:ty: $vinmX:ident),+]) => {
        $(paste::paste!{
            impl Inverting<$opamp> for $t {
                type VM_SEL = crate::stm32::opamp::[<$opamp:lower _csr>]::VM_SEL;
                const VM_SEL: Self::VM_SEL = Self::VM_SEL::$vinmX;
            }
        })*
    };
}

macro_rules! impl_non_inverting {
    ($opamp:ident, [$($t:ty: $vinpX:ident),+]) => {
        $(paste::paste!{
            impl NonInverting<$opamp> for $t {
                type VP_SEL = crate::stm32::opamp::[<$opamp:lower _csr>]::VP_SEL;
                const VP_SEL: Self::VP_SEL = Self::VP_SEL::$vinpX;
            }
        })+
    };
}

macro_rules! opamps {
    {
        $(
            $opamp:ident => $opampreg:ident
            :
            {
                vinm0: $vinm0:ty,
                vinm1: $vinm1:ty,

                inverting
                :
                {
                    $(
                        $inverting:ty
                        :
                        $inverting_mask:tt
                    ),*
                    $(,)?
                }
                ,
                non_inverting
                :
                {
                    $(
                        $non_inverting:ty
                        :
                        $non_inverting_mask:tt
                    ),*
                    $(,)?
                }
                ,
                output
                :
                $output:ty
                ,
            }
        ),*
        $(,)?
    } => {
        paste::paste!{
            $(
                /// Opamp
                pub struct $opamp;

                impl LookupPgaGain for $opamp {
                    type PgaGainReg = crate::stm32::opamp::[<$opampreg _csr>]::PGA_GAIN;

                    fn pga_gain(mode: PgaMode, gain: Gain) -> Self::PgaGainReg {
                        use crate::stm32::opamp::[<$opampreg _csr>]::PGA_GAIN;

                        match (mode, gain) {
                            (PgaMode::Pga, Gain::Gain2) => PGA_GAIN::Gain2,
                            (PgaMode::Pga, Gain::Gain4) => PGA_GAIN::Gain4,
                            (PgaMode::Pga, Gain::Gain8) => PGA_GAIN::Gain8,
                            (PgaMode::Pga, Gain::Gain16) => PGA_GAIN::Gain16,
                            (PgaMode::Pga, Gain::Gain32) => PGA_GAIN::Gain32,
                            (PgaMode::Pga, Gain::Gain64) => PGA_GAIN::Gain64,
                            (PgaMode::PgaExternalFilter, Gain::Gain2) => PGA_GAIN::Gain2FilteringVinm0,
                            (PgaMode::PgaExternalFilter, Gain::Gain4) => PGA_GAIN::Gain4FilteringVinm0,
                            (PgaMode::PgaExternalFilter, Gain::Gain8) => PGA_GAIN::Gain8FilteringVinm0,
                            (PgaMode::PgaExternalFilter, Gain::Gain16) => PGA_GAIN::Gain16FilteringVinm0,
                            (PgaMode::PgaExternalFilter, Gain::Gain32) => PGA_GAIN::Gain32FilteringVinm0,
                            (PgaMode::PgaExternalFilter, Gain::Gain64) => PGA_GAIN::Gain64FilteringVinm0,
                            (PgaMode::PgaExternalBias, Gain::Gain2) => PGA_GAIN::Gain2InputVinm0,
                            (PgaMode::PgaExternalBias, Gain::Gain4) => PGA_GAIN::Gain4InputVinm0,
                            (PgaMode::PgaExternalBias, Gain::Gain8) => PGA_GAIN::Gain8InputVinm0,
                            (PgaMode::PgaExternalBias, Gain::Gain16) => PGA_GAIN::Gain16InputVinm0,
                            (PgaMode::PgaExternalBias, Gain::Gain32) => PGA_GAIN::Gain32InputVinm0,
                            (PgaMode::PgaExternalBias, Gain::Gain64) => PGA_GAIN::Gain64InputVinm0,
                            (PgaMode::PgaExternalBiasAndFilter, Gain::Gain2) => PGA_GAIN::Gain2InputVinm0filteringVinm1,
                            (PgaMode::PgaExternalBiasAndFilter, Gain::Gain4) => PGA_GAIN::Gain4InputVinm0filteringVinm1,
                            (PgaMode::PgaExternalBiasAndFilter, Gain::Gain8) => PGA_GAIN::Gain8InputVinm0filteringVinm1,
                            (PgaMode::PgaExternalBiasAndFilter, Gain::Gain16) => PGA_GAIN::Gain16InputVinm0filteringVinm1,
                            (PgaMode::PgaExternalBiasAndFilter, Gain::Gain32) => PGA_GAIN::Gain32InputVinm0filteringVinm1,
                            (PgaMode::PgaExternalBiasAndFilter, Gain::Gain64) => PGA_GAIN::Gain64InputVinm0filteringVinm1,
                        }
                    }
                }

                impl $opamp {
                    #[inline(always)]
                    unsafe fn _reset() {
                        (*crate::stm32::OPAMP::ptr()).[<$opampreg _csr>]().reset()
                    }

                    #[inline(always)]
                    unsafe fn _disable_output() {
                        (*crate::stm32::OPAMP::ptr()).[<$opampreg _csr>]().modify(|_, w|
                            w.opaintoen().adcchannel());
                    }

                    #[inline(always)]
                    unsafe fn _enable_output() {
                        (*crate::stm32::OPAMP::ptr()).[<$opampreg _csr>]().modify(|_, w|
                            w.opaintoen().output_pin());
                    }

                    #[inline(always)]
                    unsafe fn _lock() {
                        // Write the lock bit
                        (*crate::stm32::OPAMP::ptr()).[<$opampreg _csr>]().modify(|_, w|
                            w.lock().set_bit());
                        // Write the lock bit for the corresponding TCMR register.
                        // We don't currently expose TCMR functionality, but presumably
                        // the user doesn't want anything changing if they care to set
                        // the lock bit.
                        (*crate::stm32::OPAMP::ptr()).[<$opampreg _tcmr>]().modify(|_, w|
                            w.lock().set_bit());
                    }
                }

                impl<Input, Output> stasis::Freeze for Follower<$opamp, Input, Output> { }

                impl<Input, Output> Follower<$opamp, Input, Output> {
                    /// Set the lock bit in the registers. After the lock bit is
                    /// set the opamp cannot be reconfigured until the chip is
                    /// reset.
                    pub fn lock(self) -> Locked<$opamp, Output> {
                        unsafe { $opamp::_lock() };
                        Locked { opamp: PhantomData, output: PhantomData }
                    }
                }

                impl<Input> Follower<$opamp, Input, $output> {
                    /// Disables the opamp and returns the resources it held.
                    pub fn disable(self) -> (Disabled<$opamp>, Input, $output) {
                        unsafe { $opamp::_reset() };
                        (Disabled { opamp: PhantomData }, self.input, self.output)
                    }

                    /// Disables the external output.
                    /// This will connect the opamp output to the internal ADC.
                    /// If the output was enabled, the output pin is returned.
                    pub fn disable_output(self) -> (Follower<$opamp, Input, InternalOutput>, $output) {
                        unsafe { $opamp::_disable_output(); }
                        (Follower::<$opamp, Input, InternalOutput> {
                            opamp: PhantomData,
                            input: self.input,
                            output: InternalOutput,
                        }, self.output)
                    }
                }

                impl<Input> Follower<$opamp, Input, InternalOutput> {
                    /// Disables the opamp and returns the resources it held.
                    pub fn disable(self) -> (Disabled<$opamp>, Input) {
                        unsafe { $opamp::_reset() };
                        (Disabled { opamp: PhantomData }, self.input)
                    }

                    /// Enables the external output pin.
                    pub fn enable_output<O>(self, output: O) -> Follower<$opamp, Input, O>
                        where O: stasis::EntitlementLock<Resource = $output>
                    {
                        unsafe { $opamp::_enable_output(); }

                        Follower {
                            opamp: PhantomData,
                            input: self.input,
                            output,
                        }
                    }
                }

                impl<NonInverting, Inverting, Output> stasis::Freeze for OpenLoop<$opamp, NonInverting, Inverting, Output> { }

                impl<NonInverting, Inverting, Output> OpenLoop<$opamp, NonInverting, Inverting, Output> {
                    /// Set the lock bit in the registers. After the lock bit is
                    /// set the opamp cannot be reconfigured until the chip is
                    /// reset.
                    pub fn lock(self) -> Locked<$opamp, Output> {
                        unsafe { $opamp::_lock() };
                        Locked { opamp: PhantomData, output: PhantomData }
                    }
                }

                impl<NonInverting, Inverting> OpenLoop<$opamp, NonInverting, Inverting, $output> {
                    /// Disables the opamp and returns the resources it held.
                    pub fn disable(self) -> (Disabled<$opamp>, NonInverting, Inverting, $output) {
                        unsafe { $opamp::_reset() };
                        (Disabled { opamp: PhantomData }, self.non_inverting, self.inverting, self.output)
                    }

                    /// Disables the external output.
                    /// This will connect the opamp output to the internal ADC.
                    /// If the output was enabled, the output pin is returned.
                    pub fn disable_output(self) -> (OpenLoop<$opamp, NonInverting, Inverting, InternalOutput>, $output) {
                        unsafe { $opamp::_disable_output(); }
                        (OpenLoop::<$opamp, NonInverting, Inverting, InternalOutput> {
                            opamp: PhantomData,
                            inverting: self.inverting,
                            non_inverting: self.non_inverting,
                            output: InternalOutput,
                        }, self.output)
                    }
                }

                impl<NonInverting, Inverting> OpenLoop<$opamp, NonInverting, Inverting, InternalOutput> {
                    /// Disables the opamp and returns the resources it held.
                    pub fn disable(self) -> (Disabled<$opamp>, NonInverting, Inverting) {
                        unsafe { $opamp::_reset() };
                        (Disabled { opamp: PhantomData }, self.non_inverting, self.inverting)
                    }

                    /// Enables the external output pin.
                    pub fn enable_output<O>(self, output: O) -> OpenLoop<$opamp, NonInverting, Inverting, O>
                        where O: stasis::EntitlementLock<Resource = $output>,
                    {
                        unsafe { $opamp::_enable_output(); }

                        OpenLoop {
                            opamp: PhantomData,
                            inverting: self.inverting,
                            non_inverting: self.non_inverting,
                            output,
                        }
                    }
                }

                impl<NonInverting, Output> stasis::Freeze for Pga<$opamp, NonInverting, Output> { }

                impl<NonInverting, Output> Pga<$opamp, NonInverting, Output> {
                    /// Set the lock bit in the registers. After the lock bit is
                    /// set the opamp cannot be reconfigured until the chip is
                    /// reset.
                    pub fn lock(self) -> Locked<$opamp, Output> {
                        unsafe { $opamp::_lock() };
                        Locked { opamp: PhantomData, output: PhantomData }
                    }
                }

                impl<NonInverting> Pga<$opamp, NonInverting, $output> {
                    /// Disables the opamp and returns the resources it held.
                    pub fn disable(self) -> (Disabled<$opamp>, $output) {
                        unsafe { $opamp::_reset() };
                        (Disabled { opamp: PhantomData }, self.output)
                    }

                    /// Disables the external output.
                    /// This will connect the opamp output to the internal ADC.
                    pub fn disable_output(self) -> (Pga<$opamp, NonInverting, InternalOutput>, $output) {
                        unsafe { $opamp::_disable_output(); }
                        (Pga {
                            opamp: PhantomData,
                            non_inverting: self.non_inverting,
                            output: InternalOutput,
                        }, self.output)
                    }
                }

                impl<NonInverting> Pga<$opamp, NonInverting, InternalOutput> {
                    /// Disables the opamp and returns the resources it held.
                    pub fn disable(self) -> Disabled<$opamp> {
                        unsafe { $opamp::_reset() };
                        Disabled { opamp: PhantomData }
                    }

                    /// Enables the external output pin.
                    pub fn enable_output<O>(self, output: O) -> Pga<$opamp, NonInverting, O>
                    where
                        O: stasis::EntitlementLock<Resource = $output>,
                    {
                        unsafe { $opamp::_enable_output(); }

                        Pga::<$opamp, NonInverting, O> {
                            opamp: PhantomData,
                            non_inverting: self.non_inverting,
                            output,
                        }
                    }
                }

                impl_inverting!($opamp, [$($inverting: $inverting_mask),*]);
                impl_non_inverting!($opamp, [$($non_inverting: $non_inverting_mask),* ]);

                opamps!{ @follower $opamp => $opampreg, $output }
                opamps!{ @open_loop $opamp => $opampreg, $output }
                opamps!{ @pga $opamp => $opampreg, $output : $vinm0, $vinm1 }

            )*

            /// Extension trait for the OPAMP block.
            pub trait OpampEx {
                /// Splits the OPAMP block into independant access objects.
                fn split(
                    self,
                    rcc: &mut crate::rcc::Rcc,
                ) -> (
                    $(Disabled::<$opamp>,)*
                );
            }

            impl OpampEx for crate::stm32::OPAMP {
                fn split(
                    self,
                    rcc: &mut crate::rcc::Rcc,
                ) -> (
                    $(Disabled::<$opamp>,)*
                ) {
                    rcc.rb.apb2enr().modify(|_, w| w.syscfgen().set_bit());

                    (
                        $(Disabled::<$opamp> { opamp: PhantomData },)*
                    )
                }
            }
        }
    };

    {
        @follower
        $opamp:ident => $opampreg:ident
        ,
        $output:ty
    } => {
        paste::paste!{
            impl<I, Input> IntoFollower<$opamp, Input, InternalOutput> for Disabled<$opamp>
                where
                    I: NonInverting<$opamp, VP_SEL=crate::stm32::opamp::[<$opampreg _csr>]::VP_SEL>,
                    Input: stasis::EntitlementLock<Resource = I>,
            {
                fn follower(
                    self,
                    input: Input,
                ) -> Follower<$opamp, Input, InternalOutput> {
                    unsafe {
                        (*crate::stm32::OPAMP::ptr())
                            .[<$opampreg _csr>]()
                            .write(|csr_w|
                                csr_w
                                    .vp_sel()
                                    .variant(I::VP_SEL)
                                    .vm_sel()
                                    .output()
                                    .opaintoen()
                                    .adcchannel()
                                    .opaen()
                                    .enabled()
                            );
                    }
                    Follower {opamp: PhantomData, input, output: InternalOutput}
                }
            }
        }
    };

    {
        @open_loop
        $opamp:ident => $opampreg:ident
        ,
        $output:ty
    } => {
        paste::paste!{
            impl <PP, NP, P, N> IntoOpenLoop
                <$opamp, P, N> for Disabled<$opamp>
                where
                    PP: NonInverting<$opamp, VP_SEL=crate::stm32::opamp::[<$opampreg _csr>]::VP_SEL>,
                    NP: Inverting<$opamp, VM_SEL=crate::stm32::opamp::[<$opampreg _csr>]::VM_SEL>,
                    P: stasis::EntitlementLock<Resource = PP>,
                    N: stasis::EntitlementLock<Resource = NP>,
            {
                fn open_loop(
                    self,
                    non_inverting: P,
                    inverting: N,
                ) -> OpenLoop<$opamp, P, N, InternalOutput> {
                    unsafe {
                        (*crate::stm32::OPAMP::ptr())
                            .[<$opampreg _csr>]()
                            .write(|csr_w|
                                csr_w.vp_sel()
                                    .variant(PP::VP_SEL)
                                    .vm_sel()
                                    .variant(NP::VM_SEL)
                                    .opaintoen()
                                    .adcchannel()
                                    .opaen()
                                    .enabled()
                            );
                    }
                    OpenLoop {opamp: PhantomData, non_inverting, inverting, output: InternalOutput}
                }
            }
        }
    };

    {
        @pga
        $opamp:ident => $opampreg:ident
        ,
        $output:ty
        :
        $vinm0:ty
        ,
        $vinm1:ty
    } => {
        paste::paste!{
            impl<PP, P> ConfigurePgaReg<$opamp, P> for $opamp
                where
                    PP: NonInverting<$opamp, VP_SEL=crate::stm32::opamp::[<$opampreg _csr>]::VP_SEL>,
                    P: stasis::EntitlementLock<Resource = PP>,
            {
                type Vinm0 = $vinm0;
                type Vinm1 = $vinm1;

                /// Configures the opamp for programmable gain operation.
                unsafe fn write_pga_reg(gain: Gain, mode: PgaMode) {
                    (*crate::stm32::OPAMP::ptr())
                        .[<$opampreg _csr>]()
                        .write(|csr_w|
                            csr_w.vp_sel()
                                .variant(PP::VP_SEL)
                                .vm_sel()
                                .pga()
                                .pga_gain()
                                .variant($opamp::pga_gain(mode, gain))
                                .opaintoen()
                                .adcchannel()
                                .opaen()
                                .enabled()
                        );
                }
            }

            impl<PP, P> IntoPga<$opamp, P> for Disabled<$opamp>
                where
                    PP: NonInverting<$opamp, VP_SEL=crate::stm32::opamp::[<$opampreg _csr>]::VP_SEL>,
                    P: stasis::EntitlementLock<Resource = PP>,
            {
                fn pga(
                    self,
                    _non_inverting: P,
                    gain: Gain,
                ) -> Pga<$opamp, P, InternalOutput>
                    where
                        PP: NonInverting<$opamp, VP_SEL=crate::stm32::opamp::[<$opampreg _csr>]::VP_SEL>,
                        P: stasis::EntitlementLock<Resource = PP>,
                {
                    $opamp::configure_pga(gain, PgaMode::Pga)
                }

                #[allow(private_bounds)]
                fn pga_external_filter<F>(
                    self,
                    _non_inverting: P,
                    _filter: F,
                    gain: Gain,
                ) -> Pga<$opamp, P, InternalOutput>
                    where
                        F: stasis::EntitlementLock<Resource = <$opamp as ConfigurePgaReg<$opamp, P>>::Vinm0>,
                        F::Resource: stasis::Freeze
                {
                    $opamp::configure_pga(gain, PgaMode::PgaExternalFilter)
                }

                #[allow(private_bounds)]
                fn pga_external_bias<N>(
                    self,
                    _non_inverting: P,
                    _inverting: N,
                    gain: Gain,
                ) -> Pga<$opamp, P, InternalOutput>
                    where
                        N: stasis::EntitlementLock<Resource = <$opamp as ConfigurePgaReg<$opamp, P>>::Vinm0>,
                        N::Resource: stasis::Freeze
                {
                    $opamp::configure_pga(gain, PgaMode::PgaExternalBias)
                }

                #[allow(private_bounds)]
                fn pga_external_bias_and_filter<N, F>(
                    self,
                    _non_inverting: P,
                    _inverting: N,
                    _filter: F,
                    gain: Gain,
                ) -> Pga<$opamp, P, InternalOutput>
                    where
                        N: stasis::EntitlementLock<Resource = <$opamp as ConfigurePgaReg<$opamp, P>>::Vinm0>,
                        F: stasis::EntitlementLock<Resource = <$opamp as ConfigurePgaReg<$opamp, P>>::Vinm1>,
                {
                    $opamp::configure_pga(gain, PgaMode::PgaExternalBiasAndFilter)
                }
            }
        }
    };
}

// TODO: Figure out a way to not duplicate this 3 times
#[cfg(any(feature = "stm32g431", feature = "stm32g441"))]
opamps! {
    Opamp1 => opamp1: {
        vinm0: crate::gpio::gpioa::PA3<crate::gpio::Analog>,
        vinm1: crate::gpio::gpioc::PC5<crate::gpio::Analog>,
        inverting: {
            crate::gpio::gpioa::PA3<crate::gpio::Analog>: Vinm0,
            crate::gpio::gpioc::PC5<crate::gpio::Analog>: Vinm1,
        },
        non_inverting: {
            crate::gpio::gpioa::PA1<crate::gpio::Analog>: Vinp0,
            crate::gpio::gpioa::PA3<crate::gpio::Analog>: Vinp1,
            crate::gpio::gpioa::PA7<crate::gpio::Analog>: Vinp2,
        },
        output: crate::gpio::gpioa::PA2<crate::gpio::Analog>,
    },
    Opamp2 => opamp2: {
        vinm0: crate::gpio::gpioa::PA5<crate::gpio::Analog>,
        vinm1: crate::gpio::gpioc::PC5<crate::gpio::Analog>,
        inverting: {
            crate::gpio::gpioa::PA5<crate::gpio::Analog>: Vinm0,
            crate::gpio::gpioc::PC5<crate::gpio::Analog>: Vinm1,
        },
        non_inverting: {
            crate::gpio::gpioa::PA7<crate::gpio::Analog>: Vinp0,
            crate::gpio::gpiob::PB14<crate::gpio::Analog>: Vinp1,
            crate::gpio::gpiob::PB0<crate::gpio::Analog>: Vinp2,
            crate::gpio::gpiod::PD14<crate::gpio::Analog>: Vinp3,
        },
        output: crate::gpio::gpioa::PA6<crate::gpio::Analog>,
    },
    Opamp3 => opamp3: {
        vinm0: crate::gpio::gpiob::PB2<crate::gpio::Analog>,
        vinm1: crate::gpio::gpiob::PB10<crate::gpio::Analog>,
        inverting: {
            crate::gpio::gpiob::PB2<crate::gpio::Analog>: Vinm0,
            crate::gpio::gpiob::PB10<crate::gpio::Analog>: Vinm1,
        },
        non_inverting: {
            crate::gpio::gpiob::PB0<crate::gpio::Analog>: Vinp0,
            crate::gpio::gpiob::PB13<crate::gpio::Analog>: Vinp1,
            crate::gpio::gpioa::PA1<crate::gpio::Analog>: Vinp2,
        },
        output: crate::gpio::gpiob::PB1<crate::gpio::Analog>,
    },
}

#[cfg(any(feature = "stm32g471", feature = "stm32g491", feature = "stm32g4a1"))]
opamps! {
    Opamp1 => opamp1: {
        vinm0: crate::gpio::gpioa::PA3<crate::gpio::Analog>,
        vinm1: crate::gpio::gpioc::PC5<crate::gpio::Analog>,
        inverting: {
            crate::gpio::gpioa::PA3<crate::gpio::Analog>: Vinm0,
            crate::gpio::gpioc::PC5<crate::gpio::Analog>: Vinm1,
        },
        non_inverting: {
            crate::gpio::gpioa::PA1<crate::gpio::Analog>: Vinp0,
            crate::gpio::gpioa::PA3<crate::gpio::Analog>: Vinp1,
            crate::gpio::gpioa::PA7<crate::gpio::Analog>: Vinp2,
        },
        output: crate::gpio::gpioa::PA2<crate::gpio::Analog>,
    },
    Opamp2 => opamp2: {
        vinm0: crate::gpio::gpioa::PA5<crate::gpio::Analog>,
        vinm1: crate::gpio::gpioc::PC5<crate::gpio::Analog>,
        inverting: {
            crate::gpio::gpioa::PA5<crate::gpio::Analog>: Vinm0,
            crate::gpio::gpioc::PC5<crate::gpio::Analog>: Vinm1,
        },
        non_inverting: {
            crate::gpio::gpioa::PA7<crate::gpio::Analog>: Vinp0,
            crate::gpio::gpiob::PB14<crate::gpio::Analog>: Vinp1,
            crate::gpio::gpiob::PB0<crate::gpio::Analog>: Vinp2,
            crate::gpio::gpiod::PD14<crate::gpio::Analog>: Vinp3,
        },
        output: crate::gpio::gpioa::PA6<crate::gpio::Analog>,
    },
    Opamp3 => opamp3: {
        vinm0: crate::gpio::gpiob::PB2<crate::gpio::Analog>,
        vinm1: crate::gpio::gpiob::PB10<crate::gpio::Analog>,
        inverting: {
            crate::gpio::gpiob::PB2<crate::gpio::Analog>: Vinm0,
            crate::gpio::gpiob::PB10<crate::gpio::Analog>: Vinm1,
        },
        non_inverting: {
            crate::gpio::gpiob::PB0<crate::gpio::Analog>: Vinp0,
            crate::gpio::gpiob::PB13<crate::gpio::Analog>: Vinp1,
            crate::gpio::gpioa::PA1<crate::gpio::Analog>: Vinp2,
        },
        output: crate::gpio::gpiob::PB1<crate::gpio::Analog>,
    },
    Opamp6 => opamp6: {
        vinm0: crate::gpio::gpioa::PA1<crate::gpio::Analog>,
        vinm1: crate::gpio::gpiob::PB1<crate::gpio::Analog>,
        inverting: {
            crate::gpio::gpioa::PA1<crate::gpio::Analog>: Vinm0,
            crate::gpio::gpiob::PB1<crate::gpio::Analog>: Vinm1,
        },
        non_inverting: {
            crate::gpio::gpiob::PB12<crate::gpio::Analog>: Vinp0,
            crate::gpio::gpiod::PD9<crate::gpio::Analog>: Vinp1,
            crate::gpio::gpiob::PB13<crate::gpio::Analog>: Vinp2,
        },
        output: crate::gpio::gpiob::PB11<crate::gpio::Analog>,
    },
}

#[cfg(any(
    feature = "stm32g473",
    feature = "stm32g474",
    feature = "stm32g483",
    feature = "stm32g484",
))]
opamps! {
    Opamp1 => opamp1: {
        vinm0: crate::gpio::gpioa::PA3<crate::gpio::Analog>,
        vinm1: crate::gpio::gpioc::PC5<crate::gpio::Analog>,
        inverting: {
            crate::gpio::gpioa::PA3<crate::gpio::Analog>: Vinm0,
            crate::gpio::gpioc::PC5<crate::gpio::Analog>: Vinm1,
        },
        non_inverting: {
            crate::gpio::gpioa::PA1<crate::gpio::Analog>: Vinp0,
            crate::gpio::gpioa::PA3<crate::gpio::Analog>: Vinp1,
            crate::gpio::gpioa::PA7<crate::gpio::Analog>: Vinp2,
        },
        output: crate::gpio::gpioa::PA2<crate::gpio::Analog>,
    },
    Opamp2 => opamp2: {
        vinm0: crate::gpio::gpioa::PA5<crate::gpio::Analog>,
        vinm1: crate::gpio::gpioc::PC5<crate::gpio::Analog>,
        inverting: {
            crate::gpio::gpioa::PA5<crate::gpio::Analog>: Vinm0,
            crate::gpio::gpioc::PC5<crate::gpio::Analog>: Vinm1,
        },
        non_inverting: {
            crate::gpio::gpioa::PA7<crate::gpio::Analog>: Vinp0,
            crate::gpio::gpiob::PB14<crate::gpio::Analog>: Vinp1,
            crate::gpio::gpiob::PB0<crate::gpio::Analog>: Vinp2,
            crate::gpio::gpiod::PD14<crate::gpio::Analog>: Vinp3,
        },
        output: crate::gpio::gpioa::PA6<crate::gpio::Analog>,
    },
    Opamp3 => opamp3: {
        vinm0: crate::gpio::gpiob::PB2<crate::gpio::Analog>,
        vinm1: crate::gpio::gpiob::PB10<crate::gpio::Analog>,
        inverting: {
            crate::gpio::gpiob::PB2<crate::gpio::Analog>: Vinm0,
            crate::gpio::gpiob::PB10<crate::gpio::Analog>: Vinm1,
        },
        non_inverting: {
            crate::gpio::gpiob::PB0<crate::gpio::Analog>: Vinp0,
            crate::gpio::gpiob::PB13<crate::gpio::Analog>: Vinp1,
            crate::gpio::gpioa::PA1<crate::gpio::Analog>: Vinp2,
        },
        output: crate::gpio::gpiob::PB1<crate::gpio::Analog>,
    },
    Opamp4 => opamp4: {
        vinm0: crate::gpio::gpiob::PB10<crate::gpio::Analog>,
        vinm1: crate::gpio::gpiod::PD8<crate::gpio::Analog>,
        inverting: {
            crate::gpio::gpiob::PB10<crate::gpio::Analog>: Vinm0,
            crate::gpio::gpiod::PD8<crate::gpio::Analog>: Vinm1,
        },
        non_inverting: {
            crate::gpio::gpiob::PB13<crate::gpio::Analog>: Vinp0,
            crate::gpio::gpiod::PD11<crate::gpio::Analog>: Vinp1,
            crate::gpio::gpiob::PB11<crate::gpio::Analog>: Vinp2,
        },
        output: crate::gpio::gpiob::PB12<crate::gpio::Analog>,
    },
    Opamp5 => opamp5: {
        vinm0: crate::gpio::gpiob::PB15<crate::gpio::Analog>,
        vinm1: crate::gpio::gpioa::PA3<crate::gpio::Analog>,
        inverting: {
            crate::gpio::gpiob::PB15<crate::gpio::Analog>: Vinm0,
            crate::gpio::gpioa::PA3<crate::gpio::Analog>: Vinm1,
        },
        non_inverting: {
            crate::gpio::gpiob::PB14<crate::gpio::Analog>: Vinp0,
            crate::gpio::gpiod::PD12<crate::gpio::Analog>: Vinp1,
            crate::gpio::gpioc::PC3<crate::gpio::Analog>: Vinp2,
        },
        output: crate::gpio::gpioa::PA8<crate::gpio::Analog>,
    },
    Opamp6 => opamp6: {
        vinm0: crate::gpio::gpioa::PA1<crate::gpio::Analog>,
        vinm1: crate::gpio::gpiob::PB1<crate::gpio::Analog>,
        inverting: {
            crate::gpio::gpioa::PA1<crate::gpio::Analog>: Vinm0,
            crate::gpio::gpiob::PB1<crate::gpio::Analog>: Vinm1,
        },
        non_inverting: {
            crate::gpio::gpiob::PB12<crate::gpio::Analog>: Vinp0,
            crate::gpio::gpiod::PD9<crate::gpio::Analog>: Vinp1,
            crate::gpio::gpiob::PB13<crate::gpio::Analog>: Vinp2,
        },
        output: crate::gpio::gpiob::PB11<crate::gpio::Analog>,
    },
}
