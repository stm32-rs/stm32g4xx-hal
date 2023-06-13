//! Comparator
//!
//! ## Origin
//!
//! This code has been taken from the stm32g0xx-hal project and modified slightly to support
//! STM32G4xx MCUs.

use core::marker::PhantomData;

use crate::dac;
use crate::exti::{Event as ExtiEvent, ExtiExt};
use crate::gpio::gpioa::{PA0, PA1, PA2, PA3, PA4, PA5, PA7};
use crate::gpio::gpiob::{PB0, PB1, PB2};
use crate::gpio::*;

#[cfg(any(feature = "stm32g474"))]
use crate::gpio::{
    gpioa::{PA11, PA12, PA6},
    gpiob::{PB6, PB7, PB8, PB9},
    gpioc::PC2,
    gpiof::PF4,
};

#[cfg(any(
    feature = "stm32g473",
    feature = "stm32g483",
    feature = "stm32g474",
    feature = "stm32g484"
))]
use crate::gpio::gpioa::{PA10, PA8, PA9};

#[cfg(any(
    feature = "stm32g473",
    feature = "stm32g483",
    feature = "stm32g474",
    feature = "stm32g484"
))]
use crate::gpio::gpiob::{PB10, PB11, PB12, PB13, PB14, PB15};

#[cfg(any(
    feature = "stm32g473",
    feature = "stm32g483",
    feature = "stm32g474",
    feature = "stm32g484"
))]
use crate::gpio::gpioc::{PC6, PC7, PC8};

#[cfg(any(
    feature = "stm32g473",
    feature = "stm32g483",
    feature = "stm32g474",
    feature = "stm32g484"
))]
use crate::gpio::gpiod::{PD10, PD11, PD12, PD13, PD14, PD15};

use crate::gpio::gpioc::{PC0, PC1};
use crate::gpio::gpioe::{PE7, PE8};
use crate::gpio::gpiof::PF1;
use crate::rcc::{Clocks, Rcc};
use crate::stm32::{COMP, EXTI};

/// Enabled Comparator (type state)
pub struct Enabled;

/// Enabled and locked (config is read only)
pub struct Locked;

/// Disabled Comparator (type state)
pub struct Disabled;

pub trait ED {}
impl ED for Enabled {}
impl ED for Disabled {}

pub trait EnabledState {}
impl EnabledState for Enabled {}
impl EnabledState for Locked {}

macro_rules! impl_comp {
    ($($t:ident: $reg_t:ident, $reg:ident,)+) => {$(
        pub struct $t {
            _rb: PhantomData<()>,
        }

        impl $t {
            pub fn csr(&self) -> &$crate::stm32::comp::$reg_t {
                // SAFETY: The COMP1 type is only constructed with logical ownership of
                // these registers.
                &unsafe { &*COMP::ptr() }.$reg
            }
        }
    )+};
}

impl_comp! {
    COMP1: C1CSR, c1csr,
    COMP2: C2CSR, c2csr,
    COMP3: C3CSR, c3csr,
    COMP4: C4CSR, c4csr,
}
#[cfg(any(
    feature = "stm32g473",
    feature = "stm32g483",
    feature = "stm32g474",
    feature = "stm32g484"
))]
impl_comp! {
    COMP5: C5CSR, c5csr,
    COMP6: C6CSR, c6csr,
    COMP7: C7CSR, c7csr,
}

// TODO: Split COMP in PAC

#[derive(Copy, Clone, Eq, PartialEq)]
pub struct Config {
    //power_mode: PowerMode,
    hysteresis: Hysteresis,
    inverted: bool,
    //output_xor: bool,
}

impl Default for Config {
    fn default() -> Self {
        Self {
            hysteresis: Hysteresis::None,
            inverted: false,
            //power_mode: PowerMode::HighSpeed,
            //output_xor: false,
        }
    }
}

impl Config {
    pub fn hysteresis(mut self, hysteresis: Hysteresis) -> Self {
        self.hysteresis = hysteresis;
        self
    }

    pub fn output_inverted(mut self) -> Self {
        self.inverted = true;
        self
    }

    pub fn output_polarity(mut self, inverted: bool) -> Self {
        self.inverted = inverted;
        self
    }

    /*
    pub fn power_mode(mut self, power_mode: PowerMode) -> Self {
        self.power_mode = power_mode;
        self
    }*/

    /*
    /// Sets the output to be Comparator 1 XOR Comparator 2.
    /// Used to implement window comparator mode.
    pub fn output_xor(mut self) -> Self {
        self.output_xor = true;
        self
    }*/
}

#[derive(Copy, Clone, Eq, PartialEq)]
pub enum Hysteresis {
    None = 0b000,
    H10mV = 0b001,
    H20mV = 0b010,
    H30mV = 0b011,
    H40mV = 0b100,
    H50mV = 0b101,
    H60mV = 0b110,
    H70mV = 0b111,
}

/*#[derive(Copy, Clone, Eq, PartialEq)]
pub enum PowerMode {
    HighSpeed = 0b00,
    MediumSpeed = 0b01,
}*/

/// Comparator positive input
pub trait PositiveInput<C> {
    fn setup(&self, comp: &C);
}

/// Comparator negative input
pub trait NegativeInput<C> {
    /// Does this input use the internal reference Vrefint
    ///
    /// This only true for RefintInput
    const USE_VREFINT: bool;

    /// Does this input rely on dividing Vrefint using an internal resistor divider
    ///
    /// This is only relevant for `RefintInput` other than `RefintInput::VRefint`
    fn use_resistor_divider(&self) -> bool;

    fn setup(&self, comp: &C);
}

/*
/// Comparator 1 positive input used as positive input for Comparator 2.
/// Used to implement window comparator mode.
#[derive(Copy, Clone, Eq, PartialEq)]
pub struct Comp1InP;

/// Comparator 2 positive input used as positive input for Comparator 1.
/// Used to implement window comparator mode.
#[derive(Copy, Clone, Eq, PartialEq)]
pub struct Comp2InP;


macro_rules! window_input_pin {
    ($COMP:ident, $pin:ty) => {
        impl PositiveInput<$COMP> for $pin {
            fn setup(&self, comp: &$COMP) {
                comp.csr().modify(|_, w| w.winmode().set_bit())
            }
        }
    };
}

window_input_pin!(COMP1, Comp2InP);
window_input_pin!(COMP2, Comp1InP);
*/

macro_rules! positive_input_pin {
    ($COMP:ident, $pin_0:ident, $pin_1:ident) => {
        impl PositiveInput<$COMP> for &$pin_0<Analog> {
            fn setup(&self, comp: &$COMP) {
                comp.csr().modify(|_, w| w.inpsel().bit(false))
            }
        }

        impl PositiveInput<$COMP> for &$pin_1<Analog> {
            fn setup(&self, comp: &$COMP) {
                comp.csr().modify(|_, w| w.inpsel().bit(true))
            }
        }
    };
}

positive_input_pin!(COMP1, PA1, PB1);
positive_input_pin!(COMP2, PA7, PA3);
positive_input_pin!(COMP3, PA0, PC1);
positive_input_pin!(COMP4, PB0, PE7);

#[cfg(any(
    feature = "stm32g473",
    feature = "stm32g483",
    feature = "stm32g474",
    feature = "stm32g484"
))]
positive_input_pin!(COMP5, PB13, PD12);

#[cfg(any(
    feature = "stm32g473",
    feature = "stm32g483",
    feature = "stm32g474",
    feature = "stm32g484"
))]
positive_input_pin!(COMP6, PB11, PD11);

#[cfg(any(
    feature = "stm32g473",
    feature = "stm32g483",
    feature = "stm32g474",
    feature = "stm32g484"
))]
positive_input_pin!(COMP7, PB14, PD14);

macro_rules! negative_input_pin_helper {
    ($COMP:ident, $input:ty, $bits:expr) => {
        impl NegativeInput<$COMP> for $input {
            const USE_VREFINT: bool = false;

            fn use_resistor_divider(&self) -> bool {
                false
            }

            fn setup(&self, comp: &$COMP) {
                comp.csr().modify(|_, w| unsafe { w.inmsel().bits($bits) })
            }
        }
    };
}

macro_rules! negative_input_pin {
    ($($COMP:ident: $pin_0:ty, $pin_1:ty,)+) => {$(
        negative_input_pin_helper!($COMP, $pin_0, 0b110);
        negative_input_pin_helper!($COMP, $pin_1, 0b111);
    )+};
}

negative_input_pin! {
    COMP1: PA4<Analog>, PA0<Analog>,
    COMP2: PA5<Analog>, PA2<Analog>,
    COMP3: PF1<Analog>, PC0<Analog>,
    COMP4: PE8<Analog>, PB2<Analog>,
}

#[cfg(any(
    feature = "stm32g473",
    feature = "stm32g483",
    feature = "stm32g474",
    feature = "stm32g484"
))]
negative_input_pin! {
    COMP5: PB10<Analog>, PD13<Analog>,
    COMP6: PD10<Analog>, PB15<Analog>,
    COMP7: PD15<Analog>, PB12<Analog>,
}

#[derive(Copy, Clone, Eq, PartialEq)]
pub enum RefintInput {
    /// VRefint * 1/4
    VRefintM14 = 0b000,
    /// VRefint * 1/2
    VRefintM12 = 0b001,
    /// VRefint * 3/4
    VRefintM34 = 0b010,
    /// VRefint
    VRefint = 0b011,
}

macro_rules! refint_input {
    ($($COMP:ident, )+) => {$(
        impl NegativeInput<$COMP> for RefintInput {
            const USE_VREFINT: bool = true;

            fn use_resistor_divider(&self) -> bool {
                *self != RefintInput::VRefint
            }

            fn setup(&self, comp: &$COMP) {
                comp.csr()
                    .modify(|_, w| unsafe { w.inmsel().bits(*self as u8) })
            }
        }
    )+};
}

refint_input!(COMP1, COMP2, COMP3, COMP4,);

#[cfg(any(
    feature = "stm32g473",
    feature = "stm32g483",
    feature = "stm32g474",
    feature = "stm32g484"
))]
refint_input!(COMP5, COMP6, COMP7,);

macro_rules! dac_input_helper {
    ($COMP:ident: $channel:ident, $MODE:ident, $bits:expr) => {
        impl<ED> NegativeInput<$COMP> for &dac::$channel<{ dac::$MODE }, ED> {
            const USE_VREFINT: bool = false;

            fn use_resistor_divider(&self) -> bool {
                false
            }

            fn setup(&self, comp: &$COMP) {
                comp.csr().modify(|_, w| unsafe { w.inmsel().bits($bits) })
            }
        }
    };
}

macro_rules! dac_input {
    ($COMP:ident: $channel:ident, $bits:expr) => {
        dac_input_helper!($COMP: $channel, M_MIX_SIG, $bits);
        dac_input_helper!($COMP: $channel, M_INT_SIG, $bits);
    };
}

dac_input!(COMP1: Dac3Ch1, 0b100);
dac_input!(COMP1: Dac1Ch1, 0b101);

dac_input!(COMP2: Dac3Ch2, 0b100);
dac_input!(COMP2: Dac1Ch2, 0b101);

dac_input!(COMP3: Dac3Ch1, 0b100);
dac_input!(COMP3: Dac1Ch1, 0b101);

dac_input!(COMP4: Dac3Ch2, 0b100);
dac_input!(COMP4: Dac1Ch1, 0b101);

#[cfg(any(
    feature = "stm32g473",
    feature = "stm32g483",
    feature = "stm32g474",
    feature = "stm32g484"
))]
dac_input!(COMP5: Dac4Ch1, 0b100);
#[cfg(any(
    feature = "stm32g473",
    feature = "stm32g483",
    feature = "stm32g474",
    feature = "stm32g484"
))]
dac_input!(COMP5: Dac1Ch2, 0b101);

#[cfg(any(
    feature = "stm32g473",
    feature = "stm32g483",
    feature = "stm32g474",
    feature = "stm32g484"
))]
dac_input!(COMP6: Dac4Ch2, 0b100);
#[cfg(any(
    feature = "stm32g473",
    feature = "stm32g483",
    feature = "stm32g474",
    feature = "stm32g484"
))]
dac_input!(COMP6: Dac2Ch1, 0b101);

#[cfg(any(
    feature = "stm32g473",
    feature = "stm32g483",
    feature = "stm32g474",
    feature = "stm32g484"
))]
dac_input!(COMP7: Dac4Ch1, 0b100);
#[cfg(any(
    feature = "stm32g473",
    feature = "stm32g483",
    feature = "stm32g474",
    feature = "stm32g484"
))]
dac_input!(COMP7: Dac2Ch1, 0b101);

pub struct Comparator<C, ED> {
    regs: C,
    _enabled: PhantomData<ED>,
}

pub trait ComparatorExt<COMP> {
    /// Initializes a comparator
    fn comparator<P: PositiveInput<COMP>, N: NegativeInput<COMP>>(
        self,
        positive_input: P,
        negative_input: N,
        config: Config,
        clocks: &Clocks,
    ) -> Comparator<COMP, Disabled>;
}

macro_rules! impl_comparator {
    ($COMP:ty, $comp:ident, $Event:expr) => {
        impl ComparatorExt<$COMP> for $COMP {
            fn comparator<P: PositiveInput<$COMP>, N: NegativeInput<$COMP>>(
                self,
                positive_input: P,
                negative_input: N,
                config: Config,
                clocks: &Clocks,
            ) -> Comparator<$COMP, Disabled> {
                positive_input.setup(&self);
                negative_input.setup(&self);
                // Delay for scaler voltage bridge initialization for certain negative inputs
                let voltage_scaler_delay = clocks.sys_clk.0 / (1_000_000 / 200); // 200us
                cortex_m::asm::delay(voltage_scaler_delay);
                self.csr().modify(|_, w| unsafe {
                    w.hyst()
                        .bits(config.hysteresis as u8)
                        .scalen()
                        .bit(N::USE_VREFINT)
                        .brgen()
                        .bit(negative_input.use_resistor_divider())
                        .pol()
                        .bit(config.inverted)
                });

                Comparator {
                    regs: self,
                    _enabled: PhantomData,
                }
            }
        }

        impl Comparator<$COMP, Disabled> {
            /// Initializes a comparator
            pub fn $comp<P: PositiveInput<$COMP>, N: NegativeInput<$COMP>>(
                comp: $COMP,
                positive_input: P,
                negative_input: N,
                config: Config,
                clocks: &Clocks,
            ) -> Self {
                comp.comparator(positive_input, negative_input, config, clocks)
            }

            /// Enables the comparator
            pub fn enable(self) -> Comparator<$COMP, Enabled> {
                self.regs.csr().modify(|_, w| w.en().set_bit());
                Comparator {
                    regs: self.regs,
                    _enabled: PhantomData,
                }
            }

            /// Enables raising the `ADC_COMP` interrupt at the specified output signal edge
            pub fn listen(&self, edge: SignalEdge, exti: &EXTI) {
                exti.listen($Event, edge);
            }
        }

        impl<ED: EnabledState> Comparator<$COMP, ED> {
            /// Returns the value of the output of the comparator
            pub fn output(&self) -> bool {
                self.regs.csr().read().value().bit_is_set()
            }
        }

        impl Comparator<$COMP, Enabled> {
            pub fn lock(self) -> Comparator<$COMP, Locked> {
                // Setting this bit turns all other bits into read only until restart
                self.regs.csr().modify(|_, w| w.lock().set_bit());
                Comparator {
                    regs: self.regs,
                    _enabled: PhantomData,
                }
            }

            /// Disables the comparator
            pub fn disable(self) -> Comparator<$COMP, Disabled> {
                self.regs.csr().modify(|_, w| w.en().clear_bit());
                Comparator {
                    regs: self.regs,
                    _enabled: PhantomData,
                }
            }
        }

        impl<ED> Comparator<$COMP, ED> {
            /// Disables raising interrupts for the output signal
            pub fn unlisten(&self, exti: &EXTI) {
                exti.unlisten($Event);
            }

            /// Returns `true` if the output signal interrupt is pending for the `edge`
            pub fn is_pending(&self, exti: &EXTI) -> bool {
                exti.is_pending($Event)
            }

            /// Unpends the output signal interrupt
            pub fn unpend(&self, exti: &EXTI) {
                exti.unpend($Event);
            }

            /// Configures a GPIO pin to output the signal of the comparator
            ///
            /// Multiple GPIO pins may be configured as the output simultaneously.
            pub fn output_pin<P: OutputPin<$COMP>>(&self, pin: P) {
                pin.setup();
            }
        }
    };
}

impl_comparator!(COMP1, comp1, ExtiEvent::COMP1);
impl_comparator!(COMP2, comp2, ExtiEvent::COMP2);
impl_comparator!(COMP3, comp1, ExtiEvent::COMP3);
impl_comparator!(COMP4, comp2, ExtiEvent::COMP4);

#[cfg(any(
    feature = "stm32g473",
    feature = "stm32g483",
    feature = "stm32g474",
    feature = "stm32g484"
))]
impl_comparator!(COMP5, comp1, ExtiEvent::COMP5);

#[cfg(any(
    feature = "stm32g473",
    feature = "stm32g483",
    feature = "stm32g474",
    feature = "stm32g484"
))]
impl_comparator!(COMP6, comp2, ExtiEvent::COMP6);

#[cfg(any(
    feature = "stm32g473",
    feature = "stm32g483",
    feature = "stm32g474",
    feature = "stm32g484"
))]
impl_comparator!(COMP7, comp2, ExtiEvent::COMP7);

/*
/// Uses two comparators to implement a window comparator.
/// See Figure 69 in RM0444 Rev 5.
pub struct WindowComparator<U, L, ED> {
    pub upper: Comparator<U, ED>,
    pub lower: Comparator<L, ED>,
}

pub trait WindowComparatorExt<UC, LC> {
    /// Uses two comparators to implement a window comparator
    ///
    /// See Figure 69 in RM0444 Rev 5. Ignores and overrides the `output_xor` setting in `config`.
    fn window_comparator<I: PositiveInput<UC>, L: NegativeInput<LC>, U: NegativeInput<UC>>(
        self,
        input: I,
        lower_threshold: L,
        upper_threshold: U,
        config: Config,
        clocks: &Clocks,
    ) -> WindowComparator<UC, LC, Disabled>;
}

macro_rules! impl_window_comparator {
    ($UPPER:ident, $LOWER:ident, $LOTHR:expr) => {
        impl WindowComparatorExt<$UPPER, $LOWER> for ($UPPER, $LOWER) {
            fn window_comparator<
                I: PositiveInput<$UPPER>,
                L: NegativeInput<$LOWER>,
                U: NegativeInput<$UPPER>,
            >(
                self,
                input: I,
                lower_threshold: L,
                upper_threshold: U,
                config: Config,
                clocks: &Clocks,
            ) -> WindowComparator<$UPPER, $LOWER, Disabled> {
                let (upper, lower) = self;

                let mut configu = config.clone();
                configu.output_xor = true;
                let upper = upper.comparator(input, upper_threshold, configu, clocks);

                let mut configl = config;
                configl.output_xor = false;
                let lower = lower.comparator($LOTHR, lower_threshold, configl, clocks);

                WindowComparator { upper, lower }
            }
        }

        impl WindowComparator<$UPPER, $LOWER, Disabled> {
            /// Enables the comparator
            pub fn enable(self) -> WindowComparator<$UPPER, $LOWER, Enabled> {
                WindowComparator {
                    upper: self.upper.enable(),
                    lower: self.lower.enable(),
                }
            }

            /// Enables raising the `ADC_COMP` interrupt at the specified signal edge
            pub fn listen(&self, edge: SignalEdge, exti: &mut EXTI) {
                self.upper.listen(edge, exti)
            }
        }

        impl WindowComparator<$UPPER, $LOWER, Enabled> {
            /// Disables the comparator
            pub fn disable(self) -> WindowComparator<$UPPER, $LOWER, Disabled> {
                WindowComparator {
                    upper: self.upper.disable(),
                    lower: self.lower.disable(),
                }
            }

            /// Returns the value of the output of the comparator
            pub fn output(&self) -> bool {
                self.upper.output()
            }

            /// Returns `true` if the input signal is above the lower threshold
            pub fn above_lower(&self) -> bool {
                self.lower.output()
            }
        }

        impl<ED> WindowComparator<$UPPER, $LOWER, ED> {
            /// Configures a GPIO pin to output the signal of the comparator
            ///
            /// Multiple GPIO pins may be configured as the output simultaneously.
            pub fn output_pin<P: OutputPin<$UPPER>>(&self, pin: P) {
                self.upper.output_pin(pin)
            }

            /// Disables raising interrupts for the output signal
            pub fn unlisten(&self, exti: &mut EXTI) {
                self.upper.unlisten(exti)
            }

            /// Returns `true` if the output signal interrupt is pending for the `edge`
            pub fn is_pending(&self, edge: SignalEdge, exti: &EXTI) -> bool {
                self.upper.is_pending(edge, exti)
            }

            /// Unpends the output signal interrupt
            pub fn unpend(&self, exti: &EXTI) {
                self.upper.unpend(exti)
            }
        }
    };
}

impl_window_comparator!(COMP1, COMP2, Comp1InP);
impl_window_comparator!(COMP2, COMP1, Comp2InP);

pub fn window_comparator12<
    I: PositiveInput<COMP1>,
    L: NegativeInput<COMP2>,
    U: NegativeInput<COMP1>,
>(
    comp: COMP,
    input: I,
    lower_threshold: L,
    upper_threshold: U,
    config: Config,
    rcc: &mut Rcc,
) -> WindowComparator<COMP1, COMP2, Disabled> {
    let (comp1, comp2) = comp.split(rcc);
    (comp1, comp2).window_comparator(input, lower_threshold, upper_threshold, config, &rcc.clocks)
}

pub fn window_comparator21<
    I: PositiveInput<COMP2>,
    L: NegativeInput<COMP1>,
    U: NegativeInput<COMP2>,
>(
    comp: COMP,
    input: I,
    lower_threshold: L,
    upper_threshold: U,
    config: Config,
    rcc: &mut Rcc,
) -> WindowComparator<COMP2, COMP1, Disabled> {
    let (comp1, comp2) = comp.split(rcc);
    (comp2, comp1).window_comparator(input, lower_threshold, upper_threshold, config, &rcc.clocks)
}*/

#[cfg(not(any(
    feature = "stm32g473",
    feature = "stm32g483",
    feature = "stm32g474",
    feature = "stm32g484"
)))]
type Comparators = (COMP1, COMP2, COMP3, COMP4);

#[cfg(any(
    feature = "stm32g473",
    feature = "stm32g483",
    feature = "stm32g474",
    feature = "stm32g484"
))]
type Comparators = (COMP1, COMP2, COMP3, COMP4, COMP5, COMP6, COMP7);

/// Enables the comparator peripheral, and splits the [`COMP`] into independent [`COMP1`] and [`COMP2`]
pub fn split(_comp: COMP, rcc: &mut Rcc) -> Comparators {
    // Enable COMP, SYSCFG, VREFBUF clocks
    rcc.rb.apb2enr.modify(|_, w| w.syscfgen().set_bit());

    // Reset COMP, SYSCFG, VREFBUF
    rcc.rb.apb2rstr.modify(|_, w| w.syscfgrst().set_bit());
    rcc.rb.apb2rstr.modify(|_, w| w.syscfgrst().clear_bit());

    (
        COMP1 { _rb: PhantomData },
        COMP2 { _rb: PhantomData },
        COMP3 { _rb: PhantomData },
        COMP4 { _rb: PhantomData },
        #[cfg(any(
            feature = "stm32g473",
            feature = "stm32g483",
            feature = "stm32g474",
            feature = "stm32g484"
        ))]
        COMP5 { _rb: PhantomData },
        #[cfg(any(
            feature = "stm32g473",
            feature = "stm32g483",
            feature = "stm32g474",
            feature = "stm32g484"
        ))]
        COMP6 { _rb: PhantomData },
        #[cfg(any(
            feature = "stm32g473",
            feature = "stm32g483",
            feature = "stm32g474",
            feature = "stm32g484"
        ))]
        COMP7 { _rb: PhantomData },
    )
}

pub trait ComparatorSplit {
    /// Enables the comparator peripheral, and splits the [`COMP`] into independent [`COMP1`] and [`COMP2`]
    fn split(self, rcc: &mut Rcc) -> Comparators;
}

impl ComparatorSplit for COMP {
    fn split(self, rcc: &mut Rcc) -> Comparators {
        split(self, rcc)
    }
}

pub trait OutputPin<COMP> {
    fn setup(self);
}

#[allow(unused_macros)] // TODO: add support for more devices
macro_rules! output_pin {
    ($COMP:ident, $pin:ident, $AF:ident, $mode_t:ident, $into:ident) => {
        impl OutputPin<$COMP> for $pin<Output<$mode_t>> {
            fn setup(self) {
                self.$into::<$AF>();
            }
        }
    };
    ($($COMP:ident: $pin:ident, $AF:ident,)+) => {$(
        output_pin!($COMP, $pin, $AF, PushPull, into_alternate);
        output_pin!($COMP, $pin, $AF, OpenDrain, into_alternate_open_drain);
    )+};
}

// TODO: look up alternate functions for more devices than g474
// https://www.mouser.se/datasheet/2/389/stm32g474cb-1600828.pdf#page=73
#[cfg(feature = "stm32g474")]
output_pin! {
    COMP1: PA0,  AF8,
    COMP1: PA6,  AF8,
    COMP1: PA11, AF8,
    COMP1: PB8,  AF8,
    COMP1: PF4,  AF2,

    COMP2: PA2,  AF8,
    COMP2: PA7,  AF8,
    COMP2: PA12, AF8,
    COMP2: PB9,  AF8,

    COMP3: PB7,  AF8,
    COMP3: PB15, AF3,
    COMP3: PC2,  AF3,

    COMP4: PB1,  AF8,
    COMP4: PB6,  AF8,
    COMP4: PB14, AF8,
}

// TODO: look up alternate functions for more devices than g474
// https://www.mouser.se/datasheet/2/389/stm32g474cb-1600828.pdf#page=73
#[cfg(any(
    //feature = "stm32g473",
    //feature = "stm32g483",
    feature = "stm32g474",
    //feature = "stm32g484"
))]
output_pin! {
    COMP5: PA9,  AF8,
    COMP5: PC7,  AF7,

    COMP6: PA10, AF8,
    COMP6: PC6,  AF7,

    COMP7: PA8, AF8,
    COMP7: PC8, AF7,
}
