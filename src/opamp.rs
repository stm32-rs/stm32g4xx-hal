#![deny(missing_docs)]

//! Integrated opamps.
//! ```
//! #![no_std]
//! #![no_main]
//!
//! extern crate panic_halt;
//!
//! use stm32g4xx_hal::prelude::*;
//! use stm32g4xx_hal::gpio::gpioa::*;
//! use stm32g4xx_hal::gpio::gpiob::*;
//! use stm32g4xx_hal::gpio::Analog;
//!
//! #[cortex_m_rt::entry]
//! fn main() -> ! {
//!     // take peripherals
//!     let dp = stm32g4xx_hal::stm32::Peripherals::take().unwrap();
//!
//!     // setup clock
//!     let config = stm32g4xx_hal::rcc::Config::hsi();
//!     let mut rcc = dp.RCC.freeze(config);
//!
//!     // split gpio
//!     let gpioa = dp.GPIOA.split(&mut rcc);
//!     let gpiob = dp.GPIOB.split(&mut rcc);
//!
//!     // setup opamps
//!     let (opamp1, opamp2, opamp3, opamp4, _opamp5, _opamp6) = dp.OPAMP.split(&mut rcc);
//!
//!     let opamp1 = opamp1.follower(gpioa.pa1, Some(gpioa.pa2));
//!     let opamp2 = opamp2.follower(gpioa.pa7, Option::<PA6<Analog>>::None);
//!
//!     let opamp3 = opamp3.open_loop(gpiob.pb0, gpiob.pb2, Some(gpiob.pb1));
//!     let opamp4 = opamp4.open_loop(gpiob.pb11, gpiob.pb10, Option::<PB12<Analog>>::None);
//!
//!     // disable opamps
//!     let (opamp1, pa1, some_pa2) = opamp1.disable();
//!     let (opamp2, pa7, _none) = opamp2.disable();
//!
//!     let (_opamp3, _pb0, _pb2, _some_pb1) = opamp3.disable();
//!     let (_opamp4, _pb11, _pb10, _none) = opamp4.disable();
//!
//!     let opamp1 = opamp1.pga(pa1, some_pa2);
//!     let opamp2 = opamp2.pga(pa7, Option::<PA6<Analog>>::None);
//!
//!     let (_opamp1, _pa1, _some_pa2) = opamp1.disable();
//!     let (_opamp2, _pa7, _none) = opamp2.disable();
//!
//!     loop {}
//! }
//! ```

// TODO: Add support for locking using the `LOCK` bit in `OPAMPx_CSR`
// TODO: Add support for calibration
// TODO: The output can not be a Option<Output> if we want to handle "route to pin vs adc"
//       in a compile time way. See OPAINTOEN in OPAMPx_CSR

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
pub struct Locked<Opamp> {
    opamp: PhantomData<Opamp>,
}

/// Trait for opamps that can be run in follower mode.
pub trait IntoFollower<Opamp, IntoInput, IntoOutput, Input, Output>
where
    IntoOutput: Into<Output>,
{
    /// Coonfigures the opamp as voltage follower.
    fn follower(
        self,
        input: IntoInput,
        output: Option<IntoOutput>,
    ) -> Follower<Opamp, Input, Output>;
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
> where
    IntoOutput: Into<Output>,
{
    /// Configures the opamp for open-loop operation.
    fn open_loop(
        self,
        non_inverting: IntoNonInverting,
        inverting: IntoInverting,
        output: Option<IntoOutput>,
    ) -> OpenLoop<Opamp, NonInverting, Inverting, Output>;
}
/// Trait for opamps that can be run in programmable gain mode.
pub trait IntoPga<Opamp, MODE, IntoOutput, NonInverting, Output>
where
    IntoOutput: Into<Output>,
{
    /// Configures the opamp for programmable gain operation.
    fn pga<B: Borrow<NonInverting>>(
        self,
        non_inverting: B,
        config: MODE,
        output: Option<IntoOutput>,
    ) -> Pga<Opamp, NonInverting, MODE, Output>;
}

macro_rules! opamps {
    {
        $(
            $opamp:ident
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

                impl From<&PgaModeInternal> for crate::stm32::opamp::[<$opamp _csr>]::PGA_GAIN_A {
                    fn from(x: &PgaModeInternal) -> crate::stm32::opamp::[<$opamp _csr>]::PGA_GAIN_A {
                        use crate::stm32::opamp::[<$opamp _csr>]::PGA_GAIN_A;

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

                impl<Output> From<&PgaModeInvertedInputFiltered<Output>> for crate::stm32::opamp::[<$opamp _csr>]::PGA_GAIN_A {
                    fn from(x: &PgaModeInvertedInputFiltered<Output>) -> crate::stm32::opamp::[<$opamp _csr>]::PGA_GAIN_A {
                        use crate::stm32::opamp::[<$opamp _csr>]::PGA_GAIN_A;

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

                impl<Input> Follower<$opamp, Input, $output> {

                    /// Disables the opamp and returns the resources it held.
                    pub fn disable(self) -> (Disabled<$opamp>, Input, Option<$output>) {
                        unsafe { (*crate::stm32::OPAMP::ptr()).[<$opamp _csr>].reset() }
                        (Disabled { opamp: PhantomData }, self.input, self.output)
                    }

                    /// Enables the external output pin.
                    pub fn enable_output(&mut self, output:$output) {
                        self.output = Some(output);
                        unsafe {
                            (*crate::stm32::OPAMP::ptr()).[<$opamp _csr>].write(|w|
                                w.opaintoen().output_pin());
                        }
                    }

                    /// Disables the external output.
                    /// This will connect the opamp output to the internal ADC.
                    /// If the output was enabled, the output pin is returned.
                    pub fn disable_output(&mut self) -> Option<$output> {
                        unsafe {
                            (*crate::stm32::OPAMP::ptr()).[<$opamp _csr>].write(|w|
                                w.opaintoen().adcchannel());
                        }
                        self.output.take()
                    }

                    /// Set the lock bit in the registers. After the lock bit is
                    /// set the opamp cannot be reconfigured until the chip is
                    /// reset.
                    pub fn lock(self) -> Locked<$opamp> {
                        unsafe {
                            (*crate::stm32::OPAMP::ptr()).[<$opamp _csr>].modify(|_, w|
                                w.lock().set_bit());
                        }
                        Locked { opamp: PhantomData }
                    }
                }

                impl<NonInverting, Inverting> OpenLoop<$opamp, NonInverting, Inverting, $output> {

                    /// Disables the opamp and returns the resources it held.
                    pub fn disable(self) -> (Disabled<$opamp>, NonInverting, Inverting, Option<$output>) {
                        unsafe { (*crate::stm32::OPAMP::ptr()).[<$opamp _csr>].reset() }
                        (Disabled { opamp: PhantomData }, self.non_inverting, self.inverting, self.output)
                    }

                    /// Enables the external output pin.
                    pub fn enable_output(&mut self, output:$output) {
                        self.output = Some(output);
                        unsafe {
                            (*crate::stm32::OPAMP::ptr()).[<$opamp _csr>].write(|w|
                                w.opaintoen().output_pin());
                        }
                    }

                    /// Disables the external output.
                    /// This will connect the opamp output to the internal ADC.
                    /// If the output was enabled, the output pin is returned.
                    pub fn disable_output(&mut self) -> Option<$output> {
                        unsafe {
                            (*crate::stm32::OPAMP::ptr()).[<$opamp _csr>].write(|w|
                                w.opaintoen().adcchannel());
                        }
                        self.output.take()
                    }

                    /// Set the lock bit in the registers. After the lock bit is
                    /// set the opamp cannot be reconfigured until the chip is
                    /// reset.
                    pub fn lock(self) -> Locked<$opamp> {
                        unsafe {
                            (*crate::stm32::OPAMP::ptr()).[<$opamp _csr>].modify(|_, w|
                                w.lock().set_bit());
                        }
                        Locked { opamp: PhantomData }
                    }
                }

                impl<NonInverting, MODE> Pga<$opamp, NonInverting, MODE, $output> {

                    /// Disables the opamp and returns the resources it held.
                    pub fn disable(self) -> (Disabled<$opamp>, MODE, Option<$output>) {
                        unsafe { (*crate::stm32::OPAMP::ptr()).[<$opamp _csr>].reset() }
                        (Disabled { opamp: PhantomData }, self.config, self.output)
                    }

                    /// Enables the external output pin.
                    pub fn enable_output(&mut self, output: $output) {
                        self.output = Some(output);
                        unsafe {
                            (*crate::stm32::OPAMP::ptr()).[<$opamp _csr>].write(|w|
                                w.opaintoen().output_pin());
                        }
                    }

                    /// Disables the external output.
                    /// This will connect the opamp output to the internal ADC.
                    /// If the output was enabled, the output pin is returned.
                    pub fn disable_output(&mut self) -> Option<$output> {
                        unsafe {
                            (*crate::stm32::OPAMP::ptr()).[<$opamp _csr>].write(|w|
                                w.opaintoen().adcchannel());
                        }
                        self.output.take()
                    }

                    /// Set the lock bit in the registers. After the lock bit is
                    /// set the opamp cannot be reconfigured until the chip is
                    /// reset.
                    pub fn lock(self) -> Locked<$opamp> {
                        unsafe {
                            (*crate::stm32::OPAMP::ptr()).[<$opamp _csr>].modify(|_, w|
                                w.lock().set_bit());
                        }
                        Locked { opamp: PhantomData }
                    }
                }

                opamps!{ @follower $opamp, $output, $($non_inverting_mask, $non_inverting),* }
                opamps!{ @open_loop_tt $opamp, $output, $($non_inverting_mask, $non_inverting),* : ($($inverting_mask, $inverting),*) }
                opamps!{ @pga_tt $opamp, $output, $($non_inverting_mask, $non_inverting),* : $vinm0 }

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
        $opamp:ident
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
            $(impl <IntoInput, IntoOutput> IntoFollower <$opamp, IntoInput, IntoOutput, $input, $output> for Disabled<$opamp>
                where
                    IntoInput: Into<$input>,
                    IntoOutput: Into<$output>,
            {
                fn follower(
                    self,
                    input: IntoInput,
                    output: Option<IntoOutput>,
                ) -> Follower<$opamp, $input, $output> {
                    let input = input.into();
                    let output = output.map(|output| output.into());
                    unsafe {
                        use crate::stm32::opamp::[<$opamp _csr>]::OPAINTOEN_A;
                        (*crate::stm32::OPAMP::ptr())
                            .[<$opamp _csr>]
                            .write(|csr_w|
                                csr_w
                                    .vp_sel()
                                    .$input_mask()
                                    .vm_sel()
                                    .output()
                                    .opaintoen()
                                    .variant(match output {
                                        Some(_) => OPAINTOEN_A::OutputPin,
                                        None => OPAINTOEN_A::Adcchannel,
                                    })
                                    .opaen()
                                    .enabled()
                            );
                    }
                    Follower {opamp: PhantomData, input, output}
                }
            })*
        }
    };

    {
        @open_loop_tt
        $opamp:ident
        ,
        $output:ty
        ,
        $($non_inverting_mask:tt, $non_inverting:ty),*
        :
        $invertings:tt
    } => {
        $(
            opamps!{ @open_loop $opamp, $output, $non_inverting_mask, $non_inverting, $invertings }
        )*
    };

    {
        @open_loop
        $opamp:ident
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
            $(impl <IntoNonInverting, IntoInverting, IntoOutput> IntoOpenLoop
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
                    output: Option<IntoOutput>,
                ) -> OpenLoop<$opamp, $non_inverting, $inverting, $output> {
                    let non_inverting = non_inverting.into();
                    let inverting = inverting.into();
                    let output = output.map(|output| output.into());
                    unsafe {
                        use crate::stm32::opamp::[<$opamp _csr>]::OPAINTOEN_A;
                        (*crate::stm32::OPAMP::ptr())
                            .[<$opamp _csr>]
                            .write(|csr_w|
                                csr_w.vp_sel()
                                    .$non_inverting_mask()
                                    .vm_sel()
                                    .$inverting_mask()
                                    .opaintoen()
                                    .variant(match output {
                                        Some(_) => OPAINTOEN_A::OutputPin,
                                        None => OPAINTOEN_A::Adcchannel,
                                    })
                                    .opaen()
                                    .enabled()
                            );
                    }
                    OpenLoop {opamp: PhantomData, non_inverting, inverting, output}
                }
            })*
        }
    };

    {
        @pga_tt
        $opamp:ident
        ,
        $output:ty
        ,
        $($non_inverting_mask:tt, $non_inverting:ty),*
        :
        $vinm0:ty
    } => {
        $(
            opamps!{ @pga $opamp, $output, $non_inverting_mask, $non_inverting, crate::opamp::PgaModeInternal }
            opamps!{ @pga $opamp, $output, $non_inverting_mask, $non_inverting, crate::opamp::PgaModeInvertedInputFiltered<$vinm0> }
            // TODO: Add `PGA mode, non-inverting gain setting (x2/x4/x8/x16/x32/x64) or inverting gain setting (x-1/x-3/x-7/x-15/x-31/x-63)`
            // TODO: Add `PGA mode, non-inverting gain setting (x2/x4/x8/x16/x32/x64) or inverting gain setting (x-1/x-3/x-7/x-15/x-31/x-63) with filtering`
        )*
    };

    {
        @pga
        $opamp:ident
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
                    output: Option<IntoOutput>,
                ) -> Pga<$opamp, $non_inverting, $mode, $output> {
                    let output = output.map(|output| output.into());
                    unsafe {
                        use crate::stm32::opamp::[<$opamp _csr>]::OPAINTOEN_A;

                        (*crate::stm32::OPAMP::ptr())
                            .[<$opamp _csr>]
                            .write(|csr_w|
                                csr_w.vp_sel()
                                    .$non_inverting_mask()
                                    .vm_sel()
                                    .pga()
                                    .pga_gain()
                                    .variant((&config).into())
                                    .opaintoen()
                                    .variant(match output {
                                        Some(_) => OPAINTOEN_A::OutputPin,
                                        None => OPAINTOEN_A::Adcchannel,
                                    })
                                    .opaen()
                                    .enabled()
                            );
                    }
                    Pga {opamp: PhantomData, non_inverting: PhantomData, config, output}
                }
            }
        }
    };
}

#[cfg(any(feature = "stm32g431", feature = "stm32g441", feature = "stm32g471",))]
opamps! {
    opamp1: {
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
    opamp2: {
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
    opamp3: {
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
    opamp1: {
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
    opamp2: {
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
    opamp3: {
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
    opamp4: {
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
    opamp5: {
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
    opamp6: {
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
