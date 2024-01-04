#![deny(missing_docs)]

//! Integrated opamps.
//! ```
//! #![no_std]
//! #![no_main]
//!
//! extern crate panic_halt;
//!
//! use stm32g4xx_hal::opamp::{
//!     InternalOutput, IntoFollower, IntoOpenLoop, IntoPga, NonInvertingGain, PgaModeInternal,
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
//!     let opamp1 = opamp1.pga(pa1, PgaModeInternal::gain(NonInvertingGain::Gain2), pa2);
//!     let opamp2 = opamp2.pga(
//!         pa7,
//!         PgaModeInternal::gain(NonInvertingGain::Gain2),
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

/// Pga mode internal
///
/// This mode does not expose the inverting signal on any pin,
/// only connecting it to the programmable gain divider
pub struct PgaModeInternal(NonInvertingGain);
impl PgaModeInternal {
    /// Create new instance with the given gain setting
    pub fn gain(gain: NonInvertingGain) -> Self {
        PgaModeInternal(gain)
    }
}

/// Same as PgaModeInternal but the inverted signal is routed to pin
/// to allow external filter
pub struct PgaModeInvertedInputFiltered<Output> {
    gain: NonInvertingGain,
    pin: core::marker::PhantomData<Output>,
}

/// PGA Gain for non inverted modes
pub enum NonInvertingGain {
    /// 2x Gain
    Gain2 = 0,
    /// 4x Gain
    Gain4 = 1,
    /// 8x Gain
    Gain8 = 2,
    /// 16x Gain
    Gain16 = 3,
    /// 32x Gain
    Gain32 = 4,
    /// 64x Gain
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
    output: Option<Output>,
}
/// State type for opamp running in open-loop mode.
pub struct OpenLoop<Opamp, NonInverting, Inverting, Output> {
    opamp: PhantomData<Opamp>,
    non_inverting: NonInverting,
    inverting: Inverting,
    output: Option<Output>,
}
/// State type for opamp running in programmable-gain mode.
pub struct Pga<Opamp, NonInverting, MODE, Output> {
    opamp: PhantomData<Opamp>,
    non_inverting: PhantomData<NonInverting>,
    config: MODE,
    output: Option<Output>,
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
pub trait IntoPga<Opamp, MODE, IntoOutput, NonInverting, Output> {
    /// Configures the opamp for programmable gain operation.
    fn pga<B: Borrow<NonInverting>>(
        self,
        non_inverting: B,
        config: MODE,
        output: IntoOutput,
    ) -> Pga<Opamp, NonInverting, MODE, Output>;
}

macro_rules! opamps {
    {
        $(
            $opamp:ident => $opampreg:ident
            :
            {
                vinm0: $vinm0:ty,

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

                impl From<&PgaModeInternal> for crate::stm32::opamp::[<$opampreg _csr>]::PGA_GAIN_A {
                    fn from(x: &PgaModeInternal) -> crate::stm32::opamp::[<$opampreg _csr>]::PGA_GAIN_A {
                        use crate::stm32::opamp::[<$opampreg _csr>]::PGA_GAIN_A;

                        match x.0 {
                            NonInvertingGain::Gain2 => PGA_GAIN_A::Gain2,
                            NonInvertingGain::Gain4 => PGA_GAIN_A::Gain4,
                            NonInvertingGain::Gain8 => PGA_GAIN_A::Gain8,
                            NonInvertingGain::Gain16 => PGA_GAIN_A::Gain16,
                            NonInvertingGain::Gain32 => PGA_GAIN_A::Gain32,
                            NonInvertingGain::Gain64 => PGA_GAIN_A::Gain64,
                        }
                    }
                }

                impl<Output> From<&PgaModeInvertedInputFiltered<Output>> for crate::stm32::opamp::[<$opampreg _csr>]::PGA_GAIN_A {
                    fn from(x: &PgaModeInvertedInputFiltered<Output>) -> crate::stm32::opamp::[<$opampreg _csr>]::PGA_GAIN_A {
                        use crate::stm32::opamp::[<$opampreg _csr>]::PGA_GAIN_A;

                        match x.gain {
                            NonInvertingGain::Gain2 => PGA_GAIN_A::Gain2FilteringVinm0,
                            NonInvertingGain::Gain4 => PGA_GAIN_A::Gain4FilteringVinm0,
                            NonInvertingGain::Gain8 => PGA_GAIN_A::Gain8FilteringVinm0,
                            NonInvertingGain::Gain16 => PGA_GAIN_A::Gain16FilteringVinm0,
                            NonInvertingGain::Gain32 => PGA_GAIN_A::Gain32FilteringVinm0,
                            NonInvertingGain::Gain64 => PGA_GAIN_A::Gain64FilteringVinm0,
                        }
                    }
                }

                impl $opamp {
                    /// Reset the opamp. Used by internal implementations.
                    unsafe fn _reset() {
                        (*crate::stm32::OPAMP::ptr()).[<$opampreg _csr>].reset()
                    }

                    /// Disable the output (connect the opamp to the internal ADC
                    /// channel). Used by internal implementations.
                    unsafe fn _disable_output() {
                        (*crate::stm32::OPAMP::ptr()).[<$opampreg _csr>].write(|w|
                            w.opaintoen().adcchannel())
                    }

                    /// Enable the output (connect the opamp to a pin).
                    /// Used by internal implementations.
                    unsafe fn _enable_output() {
                        (*crate::stm32::OPAMP::ptr()).[<$opampreg _csr>].write(|w|
                            w.opaintoen().output_pin())
                    }

                    /// Lock the opamp registers
                    /// Used by internal implementations.
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

                impl<Input> Follower<$opamp, Input, $output> {
                    /// Disables the opamp and returns the resources it held.
                    pub fn disable(self) -> (Disabled<$opamp>, Input, $output) {
                        unsafe { $opamp::_reset() };
                        // Safe to unwrap `self.output` because `self.output` is only None
                        // when the Output type is InternalOutput
                        (Disabled { opamp: PhantomData }, self.input, self.output.unwrap())
                    }

                    /// Disables the external output.
                    /// This will connect the opamp output to the internal ADC.
                    /// If the output was enabled, the output pin is returned.
                    pub fn disable_output(mut self) -> (Follower<$opamp, Input, InternalOutput>, $output) {
                        unsafe { $opamp::_disable_output(); }

                        // Safe to unwrap `self.output` because `self.output` is only None
                        // when the Output type is InternalOutput
                        (Follower::<$opamp, Input, InternalOutput> {
                            opamp: PhantomData,
                            input: self.input,
                            output: None,
                        }, self.output.take().unwrap())
                    }

                    /// Set the lock bit in the registers. After the lock bit is
                    /// set the opamp cannot be reconfigured until the chip is
                    /// reset.
                    pub fn lock(self) -> Locked<$opamp, $output> {
                        unsafe { $opamp::_lock() };
                        Locked { opamp: PhantomData, output: PhantomData }
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
                            output: Some(output),
                        }
                    }

                    /// Set the lock bit in the registers. After the lock bit is
                    /// set the opamp cannot be reconfigured until the chip is
                    /// reset.
                    pub fn lock(self) -> Locked<$opamp, InternalOutput> {
                        unsafe { $opamp::_lock() };
                        Locked { opamp: PhantomData, output: PhantomData }
                    }
                }

                impl<NonInverting, Inverting> OpenLoop<$opamp, NonInverting, Inverting, $output> {
                    /// Disables the opamp and returns the resources it held.
                    pub fn disable(self) -> (Disabled<$opamp>, NonInverting, Inverting, $output) {
                        unsafe { $opamp::_reset() };
                        // Safe to unwrap `self.output` because `self.output` is only None
                        // when the Output type is InternalOutput
                        (Disabled { opamp: PhantomData }, self.non_inverting, self.inverting, self.output.unwrap())
                    }

                    /// Disables the external output.
                    /// This will connect the opamp output to the internal ADC.
                    /// If the output was enabled, the output pin is returned.
                    pub fn disable_output(mut self) -> (OpenLoop<$opamp, NonInverting, Inverting, InternalOutput>, $output) {
                        unsafe { $opamp::_disable_output(); }

                        // Safe to unwrap `self.output` because `self.output` is only None
                        // when the Output type is InternalOutput
                        (OpenLoop::<$opamp, NonInverting, Inverting, InternalOutput> {
                            opamp: PhantomData,
                            inverting: self.inverting,
                            non_inverting: self.non_inverting,
                            output: None,
                        }, self.output.take().unwrap())
                    }

                    /// Set the lock bit in the registers. After the lock bit is
                    /// set the opamp cannot be reconfigured until the chip is
                    /// reset.
                    pub fn lock(self) -> Locked<$opamp, $output> {
                        unsafe { $opamp::_lock() };
                        Locked { opamp: PhantomData, output: PhantomData }
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
                            output: Some(output),
                        }
                    }

                    /// Set the lock bit in the registers. After the lock bit is
                    /// set the opamp cannot be reconfigured until the chip is
                    /// reset.
                    pub fn lock(self) -> Locked<$opamp, InternalOutput> {
                        unsafe { $opamp::_lock() };
                        Locked { opamp: PhantomData, output: PhantomData }
                    }
                }

                impl<NonInverting, MODE> Pga<$opamp, NonInverting, MODE, $output> {
                    /// Disables the opamp and returns the resources it held.
                    pub fn disable(self) -> (Disabled<$opamp>, MODE, $output) {
                        unsafe { $opamp::_reset() };
                        // Safe to unwrap `self.output` because `self.output` is only None
                        // when the Output type is InternalOutput
                        (Disabled { opamp: PhantomData }, self.config, self.output.unwrap())
                    }

                    /// Disables the external output.
                    /// This will connect the opamp output to the internal ADC.
                    pub fn disable_output(mut self) -> (Pga<$opamp, NonInverting, MODE, InternalOutput>, $output) {
                        unsafe { $opamp::_disable_output(); }

                        // Safe to unwrap `self.output` because `self.output` is only None
                        // when the Output type is InternalOutput
                        (Pga::<$opamp, NonInverting, MODE, InternalOutput> {
                            opamp: PhantomData,
                            non_inverting: self.non_inverting,
                            config: self.config,
                            output: None,
                        }, self.output.take().unwrap())
                    }

                    /// Set the lock bit in the registers. After the lock bit is
                    /// set the opamp cannot be reconfigured until the chip is
                    /// reset.
                    pub fn lock(self) -> Locked<$opamp, $output> {
                        unsafe { $opamp::_lock() };
                        Locked { opamp: PhantomData, output: PhantomData }
                    }
                }

                impl<NonInverting, MODE> Pga<$opamp, NonInverting, MODE, InternalOutput> {
                    /// Disables the opamp and returns the resources it held.
                    pub fn disable(self) -> (Disabled<$opamp>, MODE) {
                        unsafe { $opamp::_reset() };
                        (Disabled { opamp: PhantomData }, self.config)
                    }

                    /// Enables the external output pin.
                    pub fn enable_output(self, output: $output) -> Pga<$opamp, NonInverting, MODE, $output> {
                        unsafe { $opamp::_enable_output(); }

                        Pga::<$opamp, NonInverting, MODE, $output> {
                            opamp: PhantomData,
                            non_inverting: self.non_inverting,
                            config: self.config,
                            output: Some(output),
                        }
                    }

                    /// Set the lock bit in the registers. After the lock bit is
                    /// set the opamp cannot be reconfigured until the chip is
                    /// reset.
                    pub fn lock(self) -> Locked<$opamp, InternalOutput> {
                        unsafe { $opamp::_lock() };
                        Locked { opamp: PhantomData, output: PhantomData }
                    }
                }

                opamps!{ @follower $opamp => $opampreg, $output, $($non_inverting_mask, $non_inverting),* }
                opamps!{ @open_loop_tt $opamp => $opampreg, $output, $($non_inverting_mask, $non_inverting),* : ($($inverting_mask, $inverting),*) }
                opamps!{ @pga_tt $opamp => $opampreg, $output, $($non_inverting_mask, $non_inverting),* : $vinm0 }

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
                fn follower(
                    self,
                    input: IntoInput,
                    output: IntoOutput,
                ) -> Follower<$opamp, $input, $output> {
                    let input = input.into();
                    let output = output.into();
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
                                    .variant(OPAINTOEN_A::OutputPin)
                                    .opaen()
                                    .enabled()
                            );
                    }
                    Follower {opamp: PhantomData, input, output: Some(output)}
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
                    Follower {opamp: PhantomData, input, output: None}
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
                fn open_loop(
                    self,
                    non_inverting: IntoNonInverting,
                    inverting: IntoInverting,
                    output: IntoOutput,
                ) -> OpenLoop<$opamp, $non_inverting, $inverting, $output> {
                    let non_inverting = non_inverting.into();
                    let inverting = inverting.into();
                    let output = output.into();
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
                                    .variant(OPAINTOEN_A::OutputPin)
                                    .opaen()
                                    .enabled()
                            );
                    }
                    OpenLoop {opamp: PhantomData, non_inverting, inverting, output: Some(output)}
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
                    OpenLoop {opamp: PhantomData, non_inverting, inverting, output: None}
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
    } => {
        $(
            opamps!{ @pga $opamp => $opampreg, $output, $non_inverting_mask, $non_inverting, crate::opamp::PgaModeInternal }
            opamps!{ @pga $opamp => $opampreg, $output, $non_inverting_mask, $non_inverting, crate::opamp::PgaModeInvertedInputFiltered<$vinm0> }
            // TODO: Add `PGA mode, non-inverting gain setting (x2/x4/x8/x16/x32/x64) or inverting gain setting (x-1/x-3/x-7/x-15/x-31/x-63)`
            // TODO: Add `PGA mode, non-inverting gain setting (x2/x4/x8/x16/x32/x64) or inverting gain setting (x-1/x-3/x-7/x-15/x-31/x-63) with filtering`
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
        $mode:ty
    } => {
        paste::paste!{
            impl<IntoOutput> IntoPga<$opamp, $mode, IntoOutput, $non_inverting, $output> for Disabled<$opamp>
                where
                    IntoOutput: Into<$output>,
            {
                fn pga<B: Borrow<$non_inverting>>(
                    self,
                    _non_inverting: B,
                    config: $mode,
                    output: IntoOutput,
                ) -> Pga<$opamp, $non_inverting, $mode, $output> {
                    let output = output.into();
                    unsafe {
                        use crate::stm32::opamp::[<$opampreg _csr>]::OPAINTOEN_A;

                        (*crate::stm32::OPAMP::ptr())
                            .[<$opampreg _csr>]
                            .write(|csr_w|
                                csr_w.vp_sel()
                                    .$non_inverting_mask()
                                    .vm_sel()
                                    .pga()
                                    .pga_gain()
                                    .variant((&config).into())
                                    .opaintoen()
                                    .variant(OPAINTOEN_A::OutputPin)
                                    .opaen()
                                    .enabled()
                            );
                    }
                    Pga {opamp: PhantomData, non_inverting: PhantomData, config, output: Some(output)}
                }
            }
            impl IntoPga<$opamp, $mode, InternalOutput, $non_inverting, InternalOutput> for Disabled<$opamp>
            {
                fn pga<B: Borrow<$non_inverting>>(
                    self,
                    _non_inverting: B,
                    config: $mode,
                    _output: InternalOutput,
                ) -> Pga<$opamp, $non_inverting, $mode, InternalOutput> {
                    unsafe {
                        use crate::stm32::opamp::[<$opampreg _csr>]::OPAINTOEN_A;

                        (*crate::stm32::OPAMP::ptr())
                            .[<$opampreg _csr>]
                            .write(|csr_w|
                                csr_w.vp_sel()
                                    .$non_inverting_mask()
                                    .vm_sel()
                                    .pga()
                                    .pga_gain()
                                    .variant((&config).into())
                                    .opaintoen()
                                    .variant(OPAINTOEN_A::Adcchannel)
                                    .opaen()
                                    .enabled()
                            );
                    }
                    Pga {opamp: PhantomData, non_inverting: PhantomData, config, output: None}
                }
            }
        }
    };
}

#[cfg(any(feature = "stm32g431", feature = "stm32g441", feature = "stm32g471",))]
opamps! {
    Opamp1 => opamp1: {
        vinm0: crate::gpio::gpioa::PA3<crate::gpio::Analog>,
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
