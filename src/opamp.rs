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

use crate::gpio::{
    gpioa::{PA1, PA2, PA3, PA5, PA6, PA7},
    gpiob::{PB0, PB1, PB10, PB13, PB14, PB2},
    gpioc::PC5,
    gpiod::PD14,
};

#[cfg(any(
    feature = "stm32g473",
    feature = "stm32g474",
    feature = "stm32g483",
    feature = "stm32g484",
))]
use crate::gpio::{
    gpioa::PA8,
    gpiob::{PB11, PB12, PB15},
    gpioc::PC3,
    gpiod::{PD11, PD12, PD8, PD9},
};

use crate::gpio::{Analog, Frozen, IsFrozen};
use core::marker::PhantomData;

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
    input: PhantomData<Input>,
    output: PhantomData<Output>,
}
/// State type for opamp running in open-loop mode.
pub struct OpenLoop<Opamp, NonInverting, Inverting, Output> {
    opamp: PhantomData<Opamp>,
    non_inverting: PhantomData<NonInverting>,
    inverting: PhantomData<Inverting>,
    output: PhantomData<Output>,
}
/// State type for opamp running in programmable-gain mode.
pub struct Pga<Opamp, NonInverting, Output> {
    opamp: PhantomData<Opamp>,
    non_inverting: PhantomData<NonInverting>,
    output: PhantomData<Output>,
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
    fn pga<IP, OP>(
        self,
        non_inverting: IP,
        output: OP,
        gain: Gain,
    ) -> Pga<Opamp, NonInverting, Output>
    where
        IP: Frozen<NonInverting>,
        OP: Frozen<Output>;

    /// Trait for opamps that can be run in programmable gain mode,
    /// with external filtering.
    fn pga_external_filter<
        P1: Frozen<NonInverting>,
        P2: Frozen<<Opamp as ConfigurePgaReg<Opamp, NonInverting>>::Vinm0>,
    >(
        self,
        non_inverting: P1,
        filter: P2,
        output: Output,
        gain: Gain,
    ) -> Pga<Opamp, NonInverting, Output>;

    /// Configures the opamp for programmable gain operation, with
    /// external filtering.
    fn pga_external_bias<
        B1: Frozen<NonInverting>,
        B2: Frozen<<Opamp as ConfigurePgaReg<Opamp, NonInverting>>::Vinm0>,
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
        B1: Frozen<NonInverting>,
        B2: Frozen<<Opamp as ConfigurePgaReg<Opamp, NonInverting>>::Vinm0>,
        B3: Frozen<<Opamp as ConfigurePgaReg<Opamp, NonInverting>>::Vinm1>,
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
        gain: Gain,
        mode: PgaMode,
        output_enable: bool,
    ) -> Pga<Opamp, NonInverting, Output> {
        unsafe { Self::write_pga_reg(gain, mode, output_enable) };
        Pga {
            opamp: PhantomData,
            non_inverting: PhantomData,
            output: PhantomData,
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
                inverting
                :
                {
                    vinm0: $vinm0:ty,
                    vinm1: $vinm1:ty,
                }
                ,
                non_inverting
                :
                {
                    $(
                        $non_inverting_mask:tt
                        :
                        $non_inverting:ty

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

                impl<Input> Follower<$opamp, Input, InternalOutput> {
                    /// Enables the external output pin.
                    pub fn enable_output(self, _output: $output) -> Follower<$opamp, Input, $output> {
                        unsafe { $opamp::_enable_output(); }

                        Follower::<$opamp, Input, $output> {
                            opamp: PhantomData,
                            input: self.input,
                            output: PhantomData,
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

                impl<NonInverting, Inverting> OpenLoop<$opamp, NonInverting, Inverting, InternalOutput> {
                    /// Enables the external output pin.
                    pub fn enable_output(self, _output:$output) -> OpenLoop<$opamp, NonInverting, Inverting, $output> {
                        unsafe { $opamp::_enable_output(); }

                        OpenLoop::<$opamp, NonInverting, Inverting, $output> {
                            opamp: PhantomData,
                            inverting: PhantomData,
                            non_inverting: PhantomData,
                            output: PhantomData,
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

                impl<NonInverting> Pga<$opamp, NonInverting, InternalOutput> {
                    /// Disables the opamp and returns the resources it held.
                    pub fn disable(self) -> Disabled<$opamp> {
                        unsafe { $opamp::_reset() };
                        Disabled { opamp: PhantomData }
                    }

                    /// Enables the external output pin.
                    pub fn enable_output<IntoOutput>(self, _output: IntoOutput) -> Pga<$opamp, NonInverting, $output>
                    where
                        IntoOutput: Into<$output>,
                    {
                        unsafe { $opamp::_enable_output(); }

                        Pga::<$opamp, NonInverting, $output> {
                            opamp: PhantomData,
                            non_inverting: self.non_inverting,
                            output: PhantomData,
                        }
                    }
                }

                opamps!{ @follower $opamp => $opampreg, $output, $($non_inverting_mask, $non_inverting),* }
                opamps!{ @open_loop_tt $opamp => $opampreg, $output, $($non_inverting_mask, $non_inverting),* : (vinm0, $vinm0, vinm1, $vinm1) }
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
            impl <Input, IntoOutput> IntoFollower <$opamp, Input, IntoOutput, $input, $output> for Disabled<$opamp>
                where
                    Input: Frozen<$input>,
                    IntoOutput: Into<$output>,
            {
                #[inline]
                fn follower(
                    self,
                    input: Input,
                    output: IntoOutput,
                ) -> Follower<$opamp, $input, $output> {
                    self.follower(input, InternalOutput).enable_output(output.into())
                }
            }

            impl <Input> IntoFollower <$opamp, Input, InternalOutput, $input, InternalOutput> for Disabled<$opamp>
                where
                    Input: Frozen<$input>,
            {
                fn follower(
                    self,
                    _input: Input,
                    _output: InternalOutput,
                ) -> Follower<$opamp, $input, InternalOutput> {
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
                    Follower {opamp: PhantomData, input: PhantomData, output: PhantomData}
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
            impl <NonInverting, Inverting, IntoOutput> IntoOpenLoop
                <$opamp, NonInverting, Inverting, IntoOutput, $non_inverting, $inverting, $output> for Disabled<$opamp>
                where
                    NonInverting: Frozen<$non_inverting>,
                    Inverting: Frozen<$inverting>,
                    IntoOutput: Into<$output>,
            {
                #[inline]
                fn open_loop(
                    self,
                    non_inverting: NonInverting,
                    inverting: Inverting,
                    output: IntoOutput,
                ) -> OpenLoop<$opamp, $non_inverting, $inverting, $output> {
                    self.open_loop(non_inverting, inverting, InternalOutput).enable_output(output.into())
                }
            }
            impl <NonInverting, Inverting> IntoOpenLoop
                <$opamp, NonInverting, Inverting, InternalOutput, $non_inverting, $inverting, InternalOutput> for Disabled<$opamp>
                where
                    NonInverting: Frozen<$non_inverting>,
                    Inverting: Frozen<$inverting>,
            {
                fn open_loop(
                    self,
                    _non_inverting: NonInverting,
                    _inverting: Inverting,
                    _output: InternalOutput,
                ) -> OpenLoop<$opamp, $non_inverting, $inverting, InternalOutput> {
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
                    OpenLoop {opamp: PhantomData, non_inverting: PhantomData, inverting: PhantomData, output: PhantomData }
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
                fn pga<IP, OP>(
                    self,
                    _non_inverting: IP,
                    _output: OP,
                    gain: Gain,
                ) -> Pga<$opamp, $non_inverting, $output>
                where
                    IP: Frozen<$non_inverting>,
                    OP: Frozen<$output>,
                {
                    $opamp::configure_pga(gain, PgaMode::Pga, true)
                }

                #[allow(private_bounds)]
                fn pga_external_filter<
                    P1: Frozen<$non_inverting>,
                    P2: Frozen<<$opamp as ConfigurePgaReg<$opamp, $non_inverting>>::Vinm0>,
                >(
                    self,
                    _non_inverting: P1,
                    _filter: P2,
                    _output: $output,
                    gain: Gain,
                ) -> Pga<$opamp, $non_inverting, $output> {
                    $opamp::configure_pga(gain, PgaMode::PgaExternalFilter, true)
                }

                #[allow(private_bounds)]
                fn pga_external_bias<
                    P1: Frozen<$non_inverting>,
                    P2: Frozen<<$opamp as ConfigurePgaReg<$opamp, $non_inverting>>::Vinm0>,
                >(
                    self,
                    _non_inverting: P1,
                    _inverting: P2,
                    _output: $output,
                    gain: Gain,
                ) -> Pga<$opamp, $non_inverting, $output> {
                    $opamp::configure_pga(gain, PgaMode::PgaExternalBias, true)
                }

                #[allow(private_bounds)]
                fn pga_external_bias_and_filter<
                    P1: Frozen<$non_inverting>,
                    P2: Frozen<<$opamp as ConfigurePgaReg<$opamp, $non_inverting>>::Vinm0>,
                    P3: Frozen<<$opamp as ConfigurePgaReg<$opamp, $non_inverting>>::Vinm1>,
                >(
                    self,
                    _non_inverting: P1,
                    _inverting: P2,
                    _filter: P3,
                    _output: $output,
                    gain: Gain,
                ) -> Pga<$opamp, $non_inverting, $output> {
                    $opamp::configure_pga(gain, PgaMode::PgaExternalBiasAndFilter, true)
                }
            }

            impl IntoPga<$opamp, $non_inverting, InternalOutput> for Disabled<$opamp>
            {
                fn pga<IP, OP>(
                    self,
                    _non_inverting: IP,
                    _output: OP,
                    gain: Gain,
                ) -> Pga<$opamp, $non_inverting, InternalOutput>
                where
                    IP: Frozen<$non_inverting>,
                    OP: Frozen<InternalOutput>
                {
                    $opamp::configure_pga(gain, PgaMode::Pga, true)
                }

                #[allow(private_bounds)]
                fn pga_external_filter<
                    P1: Frozen<$non_inverting>,
                    P2: Frozen<<$opamp as ConfigurePgaReg<$opamp, $non_inverting>>::Vinm0>,
                >(
                    self,
                    _non_inverting: P1,
                    _filter: P2,
                    _output: InternalOutput,
                    gain: Gain,
                ) -> Pga<$opamp, $non_inverting, InternalOutput> {
                    $opamp::configure_pga(gain, PgaMode::PgaExternalFilter, true)
                }

                #[allow(private_bounds)]
                fn pga_external_bias<
                    P1: Frozen<$non_inverting>,
                    P2: Frozen<<$opamp as ConfigurePgaReg<$opamp, $non_inverting>>::Vinm0>,
                >(
                    self,
                    _non_inverting: P1,
                    _inverting: P2,
                    _output: InternalOutput,
                    gain: Gain,
                ) -> Pga<$opamp, $non_inverting, InternalOutput> {
                    $opamp::configure_pga(gain, PgaMode::PgaExternalBias, true)
                }

                #[allow(private_bounds)]
                fn pga_external_bias_and_filter<
                    P1: Frozen<$non_inverting>,
                    P2: Frozen<<$opamp as ConfigurePgaReg<$opamp, $non_inverting>>::Vinm0>,
                    P3: Frozen<<$opamp as ConfigurePgaReg<$opamp, $non_inverting>>::Vinm1>,
                >(
                    self,
                    _non_inverting: P1,
                    _inverting: P2,
                    _filter: P3,
                    _output: InternalOutput,
                    gain: Gain,
                ) -> Pga<$opamp, $non_inverting, InternalOutput> {
                    $opamp::configure_pga(gain, PgaMode::PgaExternalBiasAndFilter, true)
                }
            }
        }
    };
}

#[cfg(any(feature = "stm32g431", feature = "stm32g441", feature = "stm32g471",))]
opamps! {
    Opamp1 => opamp1: {
        inverting: {
            vinm0: PA3<Analog, IsFrozen>,
            vinm1: PC5<Analog, IsFrozen>,
        },
        non_inverting: {
            vinp0: PA1<Analog, IsFrozen>,
            vinp1: PA3<Analog, IsFrozen>,
            vinp2: PA7<Analog, IsFrozen>,
        },
        output: PA2<Analog, IsFrozen>,
    },
    Opamp2 => opamp2: {
        inverting: {
            vinm0: PA5<Analog, IsFrozen>,
            vinm1: PC5<Analog, IsFrozen>,
        },
        non_inverting: {
            vinp0: PA7<Analog, IsFrozen>,
            vinp1: PB14<Analog, IsFrozen>,
            vinp2: PB0<Analog, IsFrozen>,
            vinp3: PD14<Analog, IsFrozen>,
        },
        output: PA6<Analog, IsFrozen>,
    },
    Opamp3 => opamp3: {
        inverting: {
            vinm0: PB2<Analog, IsFrozen>,
            vinm1: PB10<Analog, IsFrozen>,
        },
        non_inverting: {
            vinp0: PB0<Analog, IsFrozen>,
            vinp1: PB13<Analog, IsFrozen>,
            vinp2: PA1<Analog, IsFrozen>,
        },
        output: PB1<Analog, IsFrozen>,
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
        inverting: {
            vinm0: PA3<Analog, IsFrozen>,
            vinm1: PC5<Analog, IsFrozen>,
        },
        non_inverting: {
            vinp0: PA1<Analog, IsFrozen>,
            vinp1: PA3<Analog, IsFrozen>,
            vinp2: PA7<Analog, IsFrozen>
        },
        output: PA2<Analog, IsFrozen>,
    },
    Opamp2 => opamp2: {
        inverting: {
            vinm0: PA5<Analog, IsFrozen>,
            vinm1: PC5<Analog, IsFrozen>,
        },
        non_inverting: {
            vinp0: PA7<Analog, IsFrozen>,
            vinp1: PB14<Analog, IsFrozen>,
            vinp2: PB0<Analog, IsFrozen>,
            vinp3: PD14<Analog, IsFrozen>,
        },
        output: PA6<Analog, IsFrozen>,
    },
    Opamp3 => opamp3: {
        inverting: {
            vinm0: PB2<Analog, IsFrozen>,
            vinm1: PB10<Analog, IsFrozen>,
        },
        non_inverting: {
            vinp0: PB0<Analog, IsFrozen>,
            vinp1: PB13<Analog, IsFrozen>,
            vinp2: PA1<Analog, IsFrozen>,
        },
        output: PB1<Analog, IsFrozen>,
    },
    Opamp4 => opamp4: {
        inverting: {
            vinm0: PB10<Analog, IsFrozen>,
            vinm1: PD8<Analog, IsFrozen>,
        },
        non_inverting: {
            vinp0: PB13<Analog, IsFrozen>,
            vinp1: PD11<Analog, IsFrozen>,
            vinp2: PB11<Analog, IsFrozen>,
        },
        output: PB12<Analog, IsFrozen>,
    },
    Opamp5 => opamp5: {
        inverting: {
            vinm0: PB15<Analog, IsFrozen>,
            vinm1: PA3<Analog, IsFrozen>,
        },
        non_inverting: {
            vinp0: PB14<Analog, IsFrozen>,
            vinp1: PD12<Analog, IsFrozen>,
            vinp2: PC3<Analog, IsFrozen>,
        },
        output: PA8<Analog, IsFrozen>,
    },
    Opamp6 => opamp6: {
        inverting: {
            vinm0: PA1<Analog, IsFrozen>,
            vinm1: PB1<Analog, IsFrozen>,
        },
        non_inverting: {
            vinp0: PB12<Analog, IsFrozen>,
            vinp1: PD9<Analog, IsFrozen>,
            vinp2: PB13<Analog, IsFrozen>,
        },
        output: PB11<Analog, IsFrozen>,
    },
}
