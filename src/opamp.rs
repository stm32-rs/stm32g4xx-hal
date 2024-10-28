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

use core::{borrow::Borrow, marker::PhantomData};

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
pub trait IntoFollower<Opamp, IntoInput, IntoOutput, Input, Output> {
    /// Coonfigures the opamp as voltage follower.
    fn follower(self, input: IntoInput, output: IntoOutput) -> Follower<Opamp, Input, Output>;
}
/// Trait for opamps that can be run in open-loop mode.
pub trait IntoOpenLoop<
    Opamp,
    IntoNonInverting,
    IntoInverting,
    IntoOutput,
    NonInverting,
    Inverting,
    Output,
>
{
    /// Configures the opamp for open-loop operation.
    fn open_loop(
        self,
        non_inverting: IntoNonInverting,
        inverting: IntoInverting,
        output: IntoOutput,
    ) -> OpenLoop<Opamp, NonInverting, Inverting, Output>;
}
/// Trait for opamps that can be run in programmable gain mode.
#[allow(private_bounds)]
pub trait IntoPga<Opamp, NonInverting, Output>
where
    Opamp: ConfigurePgaReg<Opamp, NonInverting> + LookupPgaGain,
{
    /// Configures the opamp for programmable gain operation.
    fn pga<B: Borrow<NonInverting>>(
        self,
        non_inverting: B,
        output: Output,
        gain: Gain,
    ) -> Pga<Opamp, NonInverting, Output>;

    /// Trait for opamps that can be run in programmable gain mode,
    /// with external filtering.
    fn pga_external_filter<
        B1: Borrow<NonInverting>,
        B2: Borrow<<Opamp as ConfigurePgaReg<Opamp, NonInverting>>::Vinm0>,
    >(
        self,
        non_inverting: B1,
        filter: B2,
        output: Output,
        gain: Gain,
    ) -> Pga<Opamp, NonInverting, Output>;

    /// Configures the opamp for programmable gain operation, with
    /// external filtering.
    fn pga_external_bias<
        B1: Borrow<NonInverting>,
        B2: Borrow<<Opamp as ConfigurePgaReg<Opamp, NonInverting>>::Vinm0>,
    >(
        self,
        non_inverting: B1,
        inverting: B2,
        output: Output,
        gain: Gain,
    ) -> Pga<Opamp, NonInverting, Output>;

    /// Configures the opamp for programmable gain operation, with
    /// external filtering.
    fn pga_external_bias_and_filter<
        B1: Borrow<NonInverting>,
        B2: Borrow<<Opamp as ConfigurePgaReg<Opamp, NonInverting>>::Vinm0>,
        B3: Borrow<<Opamp as ConfigurePgaReg<Opamp, NonInverting>>::Vinm1>,
    >(
        self,
        non_inverting: B1,
        inverting: B2,
        filter: B3,
        output: Output,
        gain: Gain,
    ) -> Pga<Opamp, NonInverting, Output>;
}

/// Internal trait implementing the low level register write used to
/// configure the PGA.
trait ConfigurePgaReg<Opamp, NonInverting>
where
    Opamp: LookupPgaGain,
{
    /// Type of the associated vinm0 input.
    type Vinm0;
    /// Type of the associated vinm1 input.
    type Vinm1;

    /// Write the opamp CSR register configuring the opamp PGA.
    ///
    /// Safety: This is a raw register access.
    unsafe fn write_pga_reg(gain: Gain, mode: PgaMode, output_enable: bool);

    /// Configure the
    ///
    fn configure_pga<Output>(
        output: Output,
        gain: Gain,
        mode: PgaMode,
        output_enable: bool,
    ) -> Pga<Opamp, NonInverting, Output> {
        unsafe { Self::write_pga_reg(gain, mode, output_enable) };
        Pga {
            opamp: PhantomData,
            non_inverting: PhantomData,
            output,
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
                    type PgaGainReg = crate::stm32::opamp::[<$opampreg _csr>]::PGA_GAIN_A;

                    fn pga_gain(mode: PgaMode, gain: Gain) -> Self::PgaGainReg {
                        use crate::stm32::opamp::[<$opampreg _csr>]::PGA_GAIN_A;

                        match (mode, gain) {
                            (PgaMode::Pga, Gain::Gain2) => PGA_GAIN_A::Gain2,
                            (PgaMode::Pga, Gain::Gain4) => PGA_GAIN_A::Gain4,
                            (PgaMode::Pga, Gain::Gain8) => PGA_GAIN_A::Gain8,
                            (PgaMode::Pga, Gain::Gain16) => PGA_GAIN_A::Gain16,
                            (PgaMode::Pga, Gain::Gain32) => PGA_GAIN_A::Gain32,
                            (PgaMode::Pga, Gain::Gain64) => PGA_GAIN_A::Gain64,
                            (PgaMode::PgaExternalFilter, Gain::Gain2) => PGA_GAIN_A::Gain2FilteringVinm0,
                            (PgaMode::PgaExternalFilter, Gain::Gain4) => PGA_GAIN_A::Gain4FilteringVinm0,
                            (PgaMode::PgaExternalFilter, Gain::Gain8) => PGA_GAIN_A::Gain8FilteringVinm0,
                            (PgaMode::PgaExternalFilter, Gain::Gain16) => PGA_GAIN_A::Gain16FilteringVinm0,
                            (PgaMode::PgaExternalFilter, Gain::Gain32) => PGA_GAIN_A::Gain32FilteringVinm0,
                            (PgaMode::PgaExternalFilter, Gain::Gain64) => PGA_GAIN_A::Gain64FilteringVinm0,
                            (PgaMode::PgaExternalBias, Gain::Gain2) => PGA_GAIN_A::Gain2InputVinm0,
                            (PgaMode::PgaExternalBias, Gain::Gain4) => PGA_GAIN_A::Gain4InputVinm0,
                            (PgaMode::PgaExternalBias, Gain::Gain8) => PGA_GAIN_A::Gain8InputVinm0,
                            (PgaMode::PgaExternalBias, Gain::Gain16) => PGA_GAIN_A::Gain16InputVinm0,
                            (PgaMode::PgaExternalBias, Gain::Gain32) => PGA_GAIN_A::Gain32InputVinm0,
                            (PgaMode::PgaExternalBias, Gain::Gain64) => PGA_GAIN_A::Gain64InputVinm0,
                            (PgaMode::PgaExternalBiasAndFilter, Gain::Gain2) => PGA_GAIN_A::Gain2InputVinm0filteringVinm1,
                            (PgaMode::PgaExternalBiasAndFilter, Gain::Gain4) => PGA_GAIN_A::Gain4InputVinm0filteringVinm1,
                            (PgaMode::PgaExternalBiasAndFilter, Gain::Gain8) => PGA_GAIN_A::Gain8InputVinm0filteringVinm1,
                            (PgaMode::PgaExternalBiasAndFilter, Gain::Gain16) => PGA_GAIN_A::Gain16InputVinm0filteringVinm1,
                            (PgaMode::PgaExternalBiasAndFilter, Gain::Gain32) => PGA_GAIN_A::Gain32InputVinm0filteringVinm1,
                            (PgaMode::PgaExternalBiasAndFilter, Gain::Gain64) => PGA_GAIN_A::Gain64InputVinm0filteringVinm1,
                        }
                    }
                }

                impl $opamp {
                    #[inline(always)]
                    unsafe fn _reset() {
                        (*crate::stm32::OPAMP::ptr()).[<$opampreg _csr>].reset()
                    }

                    #[inline(always)]
                    unsafe fn _disable_output() {
                        (*crate::stm32::OPAMP::ptr()).[<$opampreg _csr>].modify(|_, w|
                            w.opaintoen().adcchannel())
                    }

                    #[inline(always)]
                    unsafe fn _enable_output() {
                        (*crate::stm32::OPAMP::ptr()).[<$opampreg _csr>].modify(|_, w|
                            w.opaintoen().output_pin())
                    }

                    #[inline(always)]
                    unsafe fn _lock() {
                        // Write the lock bit
                        (*crate::stm32::OPAMP::ptr()).[<$opampreg _csr>].modify(|_, w|
                            w.lock().set_bit());
                        // Write the lock bit for the corresponding TCMR register.
                        // We don't currently expose TCMR functionality, but presumably
                        // the user doesn't want anything changing if they care to set
                        // the lock bit.
                        (*crate::stm32::OPAMP::ptr()).[<$opampreg _tcmr>].modify(|_, w|
                            w.lock().set_bit())
                    }
                }

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
                    pub fn enable_output(self, output:$output) -> Follower<$opamp, Input, $output> {
                        unsafe { $opamp::_enable_output(); }

                        Follower::<$opamp, Input, $output> {
                            opamp: PhantomData,
                            input: self.input,
                            output,
                        }
                    }
                }

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
                    pub fn enable_output(self, output:$output) -> OpenLoop<$opamp, NonInverting, Inverting, $output> {
                        unsafe { $opamp::_enable_output(); }

                        OpenLoop::<$opamp, NonInverting, Inverting, $output> {
                            opamp: PhantomData,
                            inverting: self.inverting,
                            non_inverting: self.non_inverting,
                            output,
                        }
                    }
                }

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
                        (Pga::<$opamp, NonInverting, InternalOutput> {
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
                    pub fn enable_output<IntoOutput>(self, output: IntoOutput) -> Pga<$opamp, NonInverting, $output>
                    where
                        IntoOutput: Into<$output>,
                    {
                        unsafe { $opamp::_enable_output(); }

                        Pga::<$opamp, NonInverting, $output> {
                            opamp: PhantomData,
                            non_inverting: self.non_inverting,
                            output: output.into(),
                        }
                    }
                }

                opamps!{ @follower $opamp => $opampreg, $output, $($non_inverting_mask, $non_inverting),* }
                opamps!{ @open_loop_tt $opamp => $opampreg, $output, $($non_inverting_mask, $non_inverting),* : ($($inverting_mask, $inverting),*) }
                opamps!{ @pga_tt $opamp => $opampreg, $output, $($non_inverting_mask, $non_inverting),* : $vinm0, $vinm1 }

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
                    rcc.rb.apb2enr.modify(|_, w| w.syscfgen().set_bit());

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
        ,
        $(
            $input_mask:tt
            ,
            $input:ty
        ),*
    } => {
        paste::paste!{
            $(
            impl <IntoInput, IntoOutput> IntoFollower <$opamp, IntoInput, IntoOutput, $input, $output> for Disabled<$opamp>
                where
                    IntoInput: Into<$input>,
                    IntoOutput: Into<$output>,
            {
                #[inline]
                fn follower(
                    self,
                    input: IntoInput,
                    output: IntoOutput,
                ) -> Follower<$opamp, $input, $output> {
                    self.follower(input, InternalOutput).enable_output(output.into())
                }
            }

            impl <IntoInput> IntoFollower <$opamp, IntoInput, InternalOutput, $input, InternalOutput> for Disabled<$opamp>
                where
                    IntoInput: Into<$input>,
            {
                fn follower(
                    self,
                    input: IntoInput,
                    _output: InternalOutput,
                ) -> Follower<$opamp, $input, InternalOutput> {
                    let input = input.into();
                    unsafe {
                        use crate::stm32::opamp::[<$opampreg _csr>]::OPAINTOEN_A;
                        (*crate::stm32::OPAMP::ptr())
                            .[<$opampreg _csr>]
                            .write(|csr_w|
                                csr_w
                                    .vp_sel()
                                    .$input_mask()
                                    .vm_sel()
                                    .output()
                                    .opaintoen()
                                    .variant(OPAINTOEN_A::Adcchannel)
                                    .opaen()
                                    .enabled()
                            );
                    }
                    Follower {opamp: PhantomData, input, output: InternalOutput}
                }
            }

        )*
        }
    };

    {
        @open_loop_tt
        $opamp:ident => $opampreg:ident
        ,
        $output:ty
        ,
        $($non_inverting_mask:tt, $non_inverting:ty),*
        :
        $invertings:tt
    } => {
        $(
            opamps!{ @open_loop $opamp => $opampreg, $output, $non_inverting_mask, $non_inverting, $invertings }
        )*
    };

    {
        @open_loop
        $opamp:ident => $opampreg:ident
        ,
        $output:ty
        ,
        $non_inverting_mask:tt
        ,
        $non_inverting:ty
        ,
        ($($inverting_mask:tt, $inverting:ty),*)
    } => {
        paste::paste!{
        $(
            impl <IntoNonInverting, IntoInverting, IntoOutput> IntoOpenLoop
                <$opamp, IntoNonInverting, IntoInverting, IntoOutput, $non_inverting, $inverting, $output> for Disabled<$opamp>
                where
                    IntoNonInverting: Into<$non_inverting>,
                    IntoInverting: Into<$inverting>,
                    IntoOutput: Into<$output>,
            {
                #[inline]
                fn open_loop(
                    self,
                    non_inverting: IntoNonInverting,
                    inverting: IntoInverting,
                    output: IntoOutput,
                ) -> OpenLoop<$opamp, $non_inverting, $inverting, $output> {
                    self.open_loop(non_inverting, inverting, InternalOutput).enable_output(output.into())
                }
            }
            impl <IntoNonInverting, IntoInverting> IntoOpenLoop
                <$opamp, IntoNonInverting, IntoInverting, InternalOutput, $non_inverting, $inverting, InternalOutput> for Disabled<$opamp>
                where
                    IntoNonInverting: Into<$non_inverting>,
                    IntoInverting: Into<$inverting>,
            {
                fn open_loop(
                    self,
                    non_inverting: IntoNonInverting,
                    inverting: IntoInverting,
                    _output: InternalOutput,
                ) -> OpenLoop<$opamp, $non_inverting, $inverting, InternalOutput> {
                    let non_inverting = non_inverting.into();
                    let inverting = inverting.into();
                    unsafe {
                        use crate::stm32::opamp::[<$opampreg _csr>]::OPAINTOEN_A;
                        (*crate::stm32::OPAMP::ptr())
                            .[<$opampreg _csr>]
                            .write(|csr_w|
                                csr_w.vp_sel()
                                    .$non_inverting_mask()
                                    .vm_sel()
                                    .$inverting_mask()
                                    .opaintoen()
                                    .variant(OPAINTOEN_A::Adcchannel)
                                    .opaen()
                                    .enabled()
                            );
                    }
                    OpenLoop {opamp: PhantomData, non_inverting, inverting, output: InternalOutput}
                }
            }
        )*
        }
    };

    {
        @pga_tt
        $opamp:ident => $opampreg:ident
        ,
        $output:ty
        ,
        $($non_inverting_mask:tt, $non_inverting:ty),*
        :
        $vinm0:ty
        ,
        $vinm1:ty
    } => {
        $(
            opamps!{ @pga $opamp => $opampreg, $output, $non_inverting_mask, $non_inverting, $vinm0, $vinm1 }
        )*
    };

    {
        @pga
        $opamp:ident => $opampreg:ident
        ,
        $output:ty
        ,
        $non_inverting_mask:tt
        ,
        $non_inverting:ty
        ,
        $vinm0:ty
        ,
        $vinm1:ty
    } => {
        paste::paste!{
            impl ConfigurePgaReg<$opamp, $non_inverting> for $opamp
            {
                type Vinm0 = $vinm0;
                type Vinm1 = $vinm1;

                /// Configures the opamp for programmable gain operation.
                unsafe fn write_pga_reg(gain: Gain, mode: PgaMode, output_enable: bool) {
                    use crate::stm32::opamp::[<$opampreg _csr>]::OPAINTOEN_A;

                    (*crate::stm32::OPAMP::ptr())
                        .[<$opampreg _csr>]
                        .write(|csr_w|
                            csr_w.vp_sel()
                                .$non_inverting_mask()
                                .vm_sel()
                                .pga()
                                .pga_gain()
                                .variant($opamp::pga_gain(mode, gain))
                                .opaintoen()
                                .variant(match output_enable {
                                    true => OPAINTOEN_A::OutputPin,
                                    false => OPAINTOEN_A::Adcchannel,
                                })
                                .opaen()
                                .enabled()
                        );
                }
            }

            impl IntoPga<$opamp, $non_inverting, $output> for Disabled<$opamp>
            {
                fn pga<B: Borrow<$non_inverting>>(
                    self,
                    _non_inverting: B,
                    output: $output,
                    gain: Gain,
                ) -> Pga<$opamp, $non_inverting, $output>
                {
                    $opamp::configure_pga(output.into(), gain, PgaMode::Pga, true)
                }

                #[allow(private_bounds)]
                fn pga_external_filter<
                    B1: Borrow<$non_inverting>,
                    B2: Borrow<<$opamp as ConfigurePgaReg<$opamp, $non_inverting>>::Vinm0>,
                >(
                    self,
                    _non_inverting: B1,
                    _filter: B2,
                    output: $output,
                    gain: Gain,
                ) -> Pga<$opamp, $non_inverting, $output> {
                    $opamp::configure_pga(output.into(), gain, PgaMode::PgaExternalFilter, true)
                }

                #[allow(private_bounds)]
                fn pga_external_bias<
                    B1: Borrow<$non_inverting>,
                    B2: Borrow<<$opamp as ConfigurePgaReg<$opamp, $non_inverting>>::Vinm0>,
                >(
                    self,
                    _non_inverting: B1,
                    _inverting: B2,
                    output: $output,
                    gain: Gain,
                ) -> Pga<$opamp, $non_inverting, $output> {
                    $opamp::configure_pga(output.into(), gain, PgaMode::PgaExternalBias, true)
                }

                #[allow(private_bounds)]
                fn pga_external_bias_and_filter<
                    B1: Borrow<$non_inverting>,
                    B2: Borrow<<$opamp as ConfigurePgaReg<$opamp, $non_inverting>>::Vinm0>,
                    B3: Borrow<<$opamp as ConfigurePgaReg<$opamp, $non_inverting>>::Vinm1>,
                >(
                    self,
                    _non_inverting: B1,
                    _inverting: B2,
                    _filter: B3,
                    output: $output,
                    gain: Gain,
                ) -> Pga<$opamp, $non_inverting, $output> {
                    $opamp::configure_pga(output.into(), gain, PgaMode::PgaExternalBiasAndFilter, true)
                }
            }

            impl IntoPga<$opamp, $non_inverting, InternalOutput> for Disabled<$opamp>
            {
                fn pga<B: Borrow<$non_inverting>>(
                    self,
                    _non_inverting: B,
                    _output: InternalOutput,
                    gain: Gain,
                ) -> Pga<$opamp, $non_inverting, InternalOutput>
                {
                    $opamp::configure_pga(InternalOutput, gain, PgaMode::Pga, true)
                }

                #[allow(private_bounds)]
                fn pga_external_filter<
                    B1: Borrow<$non_inverting>,
                    B2: Borrow<<$opamp as ConfigurePgaReg<$opamp, $non_inverting>>::Vinm0>,
                >(
                    self,
                    _non_inverting: B1,
                    _filter: B2,
                    _output: InternalOutput,
                    gain: Gain,
                ) -> Pga<$opamp, $non_inverting, InternalOutput> {
                    $opamp::configure_pga(InternalOutput, gain, PgaMode::PgaExternalFilter, true)
                }

                #[allow(private_bounds)]
                fn pga_external_bias<
                    B1: Borrow<$non_inverting>,
                    B2: Borrow<<$opamp as ConfigurePgaReg<$opamp, $non_inverting>>::Vinm0>,
                >(
                    self,
                    _non_inverting: B1,
                    _inverting: B2,
                    _output: InternalOutput,
                    gain: Gain,
                ) -> Pga<$opamp, $non_inverting, InternalOutput> {
                    $opamp::configure_pga(InternalOutput, gain, PgaMode::PgaExternalBias, true)
                }

                #[allow(private_bounds)]
                fn pga_external_bias_and_filter<
                    B1: Borrow<$non_inverting>,
                    B2: Borrow<<$opamp as ConfigurePgaReg<$opamp, $non_inverting>>::Vinm0>,
                    B3: Borrow<<$opamp as ConfigurePgaReg<$opamp, $non_inverting>>::Vinm1>,
                >(
                    self,
                    _non_inverting: B1,
                    _inverting: B2,
                    _filter: B3,
                    _output: InternalOutput,
                    gain: Gain,
                ) -> Pga<$opamp, $non_inverting, InternalOutput> {
                    $opamp::configure_pga(InternalOutput, gain, PgaMode::PgaExternalBiasAndFilter, true)
                }
            }
        }
    };
}

#[cfg(any(feature = "stm32g431", feature = "stm32g441", feature = "stm32g471",))]
opamps! {
    Opamp1 => opamp1: {
        vinm0: crate::gpio::gpioa::PA3<crate::gpio::Analog>,
        vinm1: crate::gpio::gpioc::PC5<crate::gpio::Analog>,
        inverting: {
            crate::gpio::gpioa::PA3<crate::gpio::Analog>: vinm0,
            crate::gpio::gpioc::PC5<crate::gpio::Analog>: vinm1,
        },
        non_inverting: {
            crate::gpio::gpioa::PA1<crate::gpio::Analog>: vinp0,
            crate::gpio::gpioa::PA3<crate::gpio::Analog>: vinp1,
            crate::gpio::gpioa::PA7<crate::gpio::Analog>: vinp2,
        },
        output: crate::gpio::gpioa::PA2<crate::gpio::Analog>,
    },
    Opamp2 => opamp2: {
        vinm0: crate::gpio::gpioa::PA5<crate::gpio::Analog>,
        vinm1: crate::gpio::gpioc::PC5<crate::gpio::Analog>,
        inverting: {
            crate::gpio::gpioa::PA5<crate::gpio::Analog>: vinm0,
            crate::gpio::gpioc::PC5<crate::gpio::Analog>: vinm1,
        },
        non_inverting: {
            crate::gpio::gpioa::PA7<crate::gpio::Analog>: vinp0,
            crate::gpio::gpiob::PB14<crate::gpio::Analog>: vinp1,
            crate::gpio::gpiob::PB0<crate::gpio::Analog>: vinp2,
            crate::gpio::gpiod::PD14<crate::gpio::Analog>: vinp3,
        },
        output: crate::gpio::gpioa::PA6<crate::gpio::Analog>,
    },
    Opamp3 => opamp3: {
        vinm0: crate::gpio::gpiob::PB2<crate::gpio::Analog>,
        vinm1: crate::gpio::gpiob::PB10<crate::gpio::Analog>,
        inverting: {
            crate::gpio::gpiob::PB2<crate::gpio::Analog>: vinm0,
            crate::gpio::gpiob::PB10<crate::gpio::Analog>: vinm1,
        },
        non_inverting: {
            crate::gpio::gpiob::PB0<crate::gpio::Analog>: vinp0,
            crate::gpio::gpiob::PB13<crate::gpio::Analog>: vinp1,
            crate::gpio::gpioa::PA1<crate::gpio::Analog>: vinp2,
        },
        output: crate::gpio::gpiob::PB1<crate::gpio::Analog>,
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
            crate::gpio::gpioa::PA3<crate::gpio::Analog>: vinm0,
            crate::gpio::gpioc::PC5<crate::gpio::Analog>: vinm1,
        },
        non_inverting: {
            crate::gpio::gpioa::PA1<crate::gpio::Analog>: vinp0,
            crate::gpio::gpioa::PA3<crate::gpio::Analog>: vinp1,
            crate::gpio::gpioa::PA7<crate::gpio::Analog>: vinp2,
        },
        output: crate::gpio::gpioa::PA2<crate::gpio::Analog>,
    },
    Opamp2 => opamp2: {
        vinm0: crate::gpio::gpioa::PA5<crate::gpio::Analog>,
        vinm1: crate::gpio::gpioc::PC5<crate::gpio::Analog>,
        inverting: {
            crate::gpio::gpioa::PA5<crate::gpio::Analog>: vinm0,
            crate::gpio::gpioc::PC5<crate::gpio::Analog>: vinm1,
        },
        non_inverting: {
            crate::gpio::gpioa::PA7<crate::gpio::Analog>: vinp0,
            crate::gpio::gpiob::PB14<crate::gpio::Analog>: vinp1,
            crate::gpio::gpiob::PB0<crate::gpio::Analog>: vinp2,
            crate::gpio::gpiod::PD14<crate::gpio::Analog>: vinp3,
        },
        output: crate::gpio::gpioa::PA6<crate::gpio::Analog>,
    },
    Opamp3 => opamp3: {
        vinm0: crate::gpio::gpiob::PB2<crate::gpio::Analog>,
        vinm1: crate::gpio::gpiob::PB10<crate::gpio::Analog>,
        inverting: {
            crate::gpio::gpiob::PB2<crate::gpio::Analog>: vinm0,
            crate::gpio::gpiob::PB10<crate::gpio::Analog>: vinm1,
        },
        non_inverting: {
            crate::gpio::gpiob::PB0<crate::gpio::Analog>: vinp0,
            crate::gpio::gpiob::PB13<crate::gpio::Analog>: vinp1,
            crate::gpio::gpioa::PA1<crate::gpio::Analog>: vinp2,
        },
        output: crate::gpio::gpiob::PB1<crate::gpio::Analog>,
    },
    Opamp4 => opamp4: {
        vinm0: crate::gpio::gpiob::PB10<crate::gpio::Analog>,
        vinm1: crate::gpio::gpiod::PD8<crate::gpio::Analog>,
        inverting: {
            crate::gpio::gpiob::PB10<crate::gpio::Analog>: vinm0,
            crate::gpio::gpiod::PD8<crate::gpio::Analog>: vinm1,
        },
        non_inverting: {
            crate::gpio::gpiob::PB13<crate::gpio::Analog>: vinp0,
            crate::gpio::gpiod::PD11<crate::gpio::Analog>: vinp1,
            crate::gpio::gpiob::PB11<crate::gpio::Analog>: vinp2,
        },
        output: crate::gpio::gpiob::PB12<crate::gpio::Analog>,
    },
    Opamp5 => opamp5: {
        vinm0: crate::gpio::gpiob::PB15<crate::gpio::Analog>,
        vinm1: crate::gpio::gpioa::PA3<crate::gpio::Analog>,
        inverting: {
            crate::gpio::gpiob::PB15<crate::gpio::Analog>: vinm0,
            crate::gpio::gpioa::PA3<crate::gpio::Analog>: vinm1,
        },
        non_inverting: {
            crate::gpio::gpiob::PB14<crate::gpio::Analog>: vinp0,
            crate::gpio::gpiod::PD12<crate::gpio::Analog>: vinp1,
            crate::gpio::gpioc::PC3<crate::gpio::Analog>: vinp2,
        },
        output: crate::gpio::gpioa::PA8<crate::gpio::Analog>,
    },
    Opamp6 => opamp6: {
        vinm0: crate::gpio::gpioa::PA1<crate::gpio::Analog>,
        vinm1: crate::gpio::gpiob::PB1<crate::gpio::Analog>,
        inverting: {
            crate::gpio::gpioa::PA1<crate::gpio::Analog>: vinm0,
            crate::gpio::gpiob::PB1<crate::gpio::Analog>: vinm1,
        },
        non_inverting: {
            crate::gpio::gpiob::PB12<crate::gpio::Analog>: vinp0,
            crate::gpio::gpiod::PD9<crate::gpio::Analog>: vinp1,
            crate::gpio::gpiob::PB13<crate::gpio::Analog>: vinp2,
        },
        output: crate::gpio::gpiob::PB11<crate::gpio::Analog>,
    },
}
