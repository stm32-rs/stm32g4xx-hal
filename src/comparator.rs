//! Comparator

use core::marker::PhantomData;

use crate::dac;
use crate::exti::{Event as ExtiEvent, ExtiExt};
use crate::gpio::*;
use crate::gpio::gpiob::PB14;
use crate::gpio::gpiod::PD14;
use crate::rcc::{Clocks, Rcc};
use crate::stm32::comp::{C1CSR, C2CSR};
use crate::stm32::{COMP, EXTI};

/// Enabled Comparator (type state)
pub struct Enabled;

/// Disabled Comparator (type state)
pub struct Disabled;

pub trait ED {}
impl ED for Enabled {}
impl ED for Disabled {}

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

impl_comp!{
    COMP1: C1CSR, c1csr,
    COMP2: C2CSR, c2csr,
    COMP3: C3CSR, c3csr,
    COMP4: C4CSR, c4csr,
}
#[cfg(any(feature = "stm32g473", feature = "stm32g483", feature = "stm32g474", feature = "stm32g484"))]
impl_comp!{
    COMP5: C5CSR, c5csr,
    COMP6: C6CSR, c6csr,
    COMP7: C7CSR, c7csr,
}

// TODO: Split COMP in PAC

#[derive(Copy, Clone, Eq, PartialEq)]
pub struct Config {
    power_mode: PowerMode,
    hysteresis: Hysteresis,
    inverted: bool,
    output_xor: bool,
}

impl Default for Config {
    fn default() -> Self {
        Self {
            hysteresis: Hysteresis::None,
            inverted: false,
            power_mode: PowerMode::HighSpeed,
            output_xor: false,
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

    pub fn power_mode(mut self, power_mode: PowerMode) -> Self {
        self.power_mode = power_mode;
        self
    }

    /// Sets the output to be Comparator 1 XOR Comparator 2.
    /// Used to implement window comparator mode.
    pub fn output_xor(mut self) -> Self {
        self.output_xor = true;
        self
    }
}

#[derive(Copy, Clone, Eq, PartialEq)]
pub enum Hysteresis {
    None = 0b00,
    Low = 0b01,
    Medium = 0b10,
    High = 0b11,
}

#[derive(Copy, Clone, Eq, PartialEq)]
pub enum PowerMode {
    HighSpeed = 0b00,
    MediumSpeed = 0b01,
}

/// Comparator positive input
pub trait PositiveInput<C> {
    fn setup(&self, comp: &C);
}

/// Comparator negative input
pub trait NegativeInput<C> {
    fn setup(&self, comp: &C);
}

/// 1/4 Vref
#[derive(Copy, Clone, Eq, PartialEq)]
pub struct Vref1div4;

/// 1/2 Vref
#[derive(Copy, Clone, Eq, PartialEq)]
pub struct Vref1div2;

/// 3/4 Vref
#[derive(Copy, Clone, Eq, PartialEq)]
pub struct Vref3div4;

/// Vref
#[derive(Copy, Clone, Eq, PartialEq)]
pub struct Vref;

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
    ($COMP:ident, $pin_0:ty, $pin_1:ty) => {
        impl PositiveInput<$COMP> for $pin_0 {
            fn setup(&self, comp: &$COMP) {
                comp.csr().modify(|_, w| unsafe { w.inpsel().bit(0) })
            }
        }

        impl PositiveInput<$COMP> for $pin_1 {
            fn setup(&self, comp: &$COMP) {
                comp.csr().modify(|_, w| unsafe { w.inpsel().bit(1) })
            }
        }
    };
}

positive_input_pin!(COMP1, PA1<Analog>, PB1<Analog>);
positive_input_pin!(COMP2, PA7<Analog>, PA3<Analog>);
positive_input_pin!(COMP3, PA0<Analog>, PC1<Analog>);
positive_input_pin!(COMP4, PB0<Analog>, PE7<Analog>);

#[cfg(any(feature = "stm32g473", feature = "stm32g483", feature = "stm32g474", feature = "stm32g484"))]
positive_input_pin!(COMP5, PB13<Analog>, PD12<Analog>);

#[cfg(any(feature = "stm32g473", feature = "stm32g483", feature = "stm32g474", feature = "stm32g484"))]
positive_input_pin!(COMP6, PB11<Analog>, PD11<Analog>);

#[cfg(any(feature = "stm32g473", feature = "stm32g483", feature = "stm32g474", feature = "stm32g484"))]
positive_input_pin!(COMP7, PB14<Analog>, PD14<Analog>);

macro_rules! negative_input_pin_helper {
    ($COMP:ident, $input:ty, $bits:expr) => {
        impl NegativeInput<$COMP> for $input {
            fn setup(&self, comp: &$COMP) {
                comp.csr().modify(|_, w| unsafe { w.inmsel().bits($bits) })
            }
        }
    };
}

macro_rules! negative_input_pin {
    ($($COMP:ident: $dac_0:ty, $dac_1:ty, $pin_0:ty, $pin_1:ty,)+) => {$(
        negative_input_pin_helper($COMP, Vref1div4, 0b000);
        negative_input_pin_helper($COMP, Vref1div2, 0b001);
        negative_input_pin_helper($COMP, Vref3div4, 0b010);
        negative_input_pin_helper($COMP, Vref,      0b011);

        //negative_input_pin_helper($COMP, $dac_0,    0b100);
        //negative_input_pin_helper($COMP, $dac_1,    0b101);

        negative_input_pin_helper($COMP, $pin_0,    0b110);
        negative_input_pin_helper($COMP, $pin_1,    0b111);
    )+};
}

/// TODO: Add DAC support
struct TodoAddDac;

negative_input_pin!{
    COMP1: TodoAddDac, TodoAddDac, PA4<Analog>, PA0<Analog>,
    COMP2: TodoAddDac, TodoAddDac, PA5<Analog>, PA2<Analog>,
    COMP3: TodoAddDac, TodoAddDac, PF1<Analog>, PC0<Analog>,
    COMP4: TodoAddDac, TodoAddDac, PE8<Analog>, PB2<Analog>,
}

#[cfg(any(feature = "stm32g473", feature = "stm32g483", feature = "stm32g474", feature = "stm32g484"))]
negative_input_pin!{
    COMP5: TodoAddDac, TodoAddDac, PB10<Analog>, PD13<Analog>,
    COMP6: TodoAddDac, TodoAddDac, PD10<Analog>, PB15<Analog>,
    COMP7: TodoAddDac, TodoAddDac, PD15<Analog>, PB12<Analog>,
};

#[derive(Copy, Clone, Eq, PartialEq)]
pub enum RefintInput {
    /// VRefint * 1/4
    VRefintM14 = 0b0000,
    /// VRefint * 1/2
    VRefintM12 = 0b0001,
    /// VRefint * 3/4
    VRefintM34 = 0b0010,
    /// VRefint
    VRefint = 0b0011,
}

/*
macro_rules! dac_input {
    ($COMP:ident, $channel:ty, $bits:expr) => {
        impl<ED> NegativeInput<$COMP> for &$channel {
            fn setup(&self, comp: &$COMP) {
                comp.csr().modify(|_, w| unsafe { w.inmsel().bits($bits) })
            }
        }
    };
}

#[cfg(any(feature = "stm32g071", feature = "stm32g081"))]
dac_input!(COMP1, dac::Channel1<ED>, 0b0100);
#[cfg(any(feature = "stm32g071", feature = "stm32g081"))]
dac_input!(COMP1, dac::Channel2<ED>, 0b0101);

#[cfg(any(feature = "stm32g071", feature = "stm32g081"))]
dac_input!(COMP2, dac::Channel1<ED>, 0b0100);
#[cfg(any(feature = "stm32g071", feature = "stm32g081"))]
dac_input!(COMP2, dac::Channel2<ED>, 0b0101);*/

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
                let voltage_scaler_delay = clocks.sys_clk.raw() / (1_000_000 / 200); // 200us
                cortex_m::asm::delay(voltage_scaler_delay);
                self.csr().modify(|_, w| unsafe {
                    w.hyst()
                        .bits(config.hysteresis as u8)
                        //.brgen()
                        //.set_bit()
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

        impl Comparator<$COMP, Enabled> {
            /// Returns the value of the output of the comparator
            pub fn output(&self) -> bool {
                self.regs.csr().read().value().bit_is_set()
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
            pub fn is_pending(&self, edge: SignalEdge, exti: &EXTI) -> bool {
                exti.is_pending($Event, edge)
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
}

/// Enables the comparator peripheral, and splits the [`COMP`] into independent [`COMP1`] and [`COMP2`]
pub fn split(_comp: COMP, rcc: &mut Rcc) -> (COMP1, COMP2) {
    // Enable COMP, SYSCFG, VREFBUF clocks
    rcc.rb.apbenr2.modify(|_, w| w.syscfgen().set_bit());

    // Reset COMP, SYSCFG, VREFBUF
    rcc.rb.apbrstr2.modify(|_, w| w.syscfgrst().set_bit());
    rcc.rb.apbrstr2.modify(|_, w| w.syscfgrst().clear_bit());

    (COMP1 { _rb: PhantomData }, COMP2 { _rb: PhantomData })
}

pub trait ComparatorSplit {
    /// Enables the comparator peripheral, and splits the [`COMP`] into independent [`COMP1`] and [`COMP2`]
    fn split(self, rcc: &mut Rcc) -> (COMP1, COMP2);
}

impl ComparatorSplit for COMP {
    fn split(self, rcc: &mut Rcc) -> (COMP1, COMP2) {
        split(self, rcc)
    }
}

pub trait OutputPin<COMP> {
    fn setup(&self);
    fn release(self) -> Self;
}

macro_rules! output_pin_push_pull {
    ($COMP:ident, $pin:ty) => {
        impl OutputPin<$COMP> for $pin {
            fn setup(&self) {
                self.set_alt_mode(AltFunction::AF7)
            }

            fn release(self) -> Self {
                self.into_push_pull_output()
            }
        }
    };
}

macro_rules! output_pin_open_drain {
    ($COMP:ident, $pin:ty) => {
        impl OutputPin<$COMP> for $pin {
            fn setup(&self) {
                self.set_alt_mode(AltFunction::AF7)
            }

            fn release(self) -> Self {
                self.into_open_drain_output()
            }
        }
    };
}

output_pin_push_pull!(COMP1, PA0<Output<PushPull>>);
output_pin_open_drain!(COMP1, PA0<Output<OpenDrain>>);
output_pin_push_pull!(COMP1, PA6<Output<PushPull>>);
output_pin_open_drain!(COMP1, PA6<Output<OpenDrain>>);
output_pin_push_pull!(COMP1, PA11<Output<PushPull>>);
output_pin_open_drain!(COMP1, PA11<Output<OpenDrain>>);
output_pin_push_pull!(COMP1, PB0<Output<PushPull>>);
output_pin_open_drain!(COMP1, PB0<Output<OpenDrain>>);
output_pin_push_pull!(COMP1, PB10<Output<PushPull>>);
output_pin_open_drain!(COMP1, PB10<Output<OpenDrain>>);

output_pin_push_pull!(COMP2, PA2<Output<PushPull>>);
output_pin_open_drain!(COMP2, PA2<Output<OpenDrain>>);
output_pin_push_pull!(COMP2, PA7<Output<PushPull>>);
output_pin_open_drain!(COMP2, PA7<Output<OpenDrain>>);
output_pin_push_pull!(COMP2, PA12<Output<PushPull>>);
output_pin_open_drain!(COMP2, PA12<Output<OpenDrain>>);
output_pin_push_pull!(COMP2, PB5<Output<PushPull>>);
output_pin_open_drain!(COMP2, PB5<Output<OpenDrain>>);
output_pin_push_pull!(COMP2, PB11<Output<PushPull>>);
output_pin_open_drain!(COMP2, PB11<Output<OpenDrain>>);
