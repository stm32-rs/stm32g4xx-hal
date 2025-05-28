//! Comparator
//!
//! ## Origin
//!
//! This code has been taken from the stm32g0xx-hal project and modified slightly to support
//! STM32G4xx MCUs.

use core::marker::PhantomData;

use crate::dac;
use crate::exti::{Event as ExtiEvent, ExtiExt};
use crate::gpio::{self, Analog, OpenDrain, Output, PushPull, SignalEdge};

use crate::rcc::{Clocks, Rcc};
use crate::stasis;
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
    ($($t:ident: $reg:ident,)+) => {$(
        pub struct $t {
            _rb: PhantomData<()>,
        }

        impl $t {
            pub fn csr(&self) -> &$crate::stm32::comp::CCSR {
                // SAFETY: The COMP1 type is only constructed with logical ownership of
                // these registers.
                &unsafe { &*COMP::ptr() }.$reg()
            }
        }
    )+};
}

impl_comp! {
    COMP1: c1csr,
    COMP2: c2csr,
    COMP3: c3csr,
    COMP4: c4csr,
}
#[cfg(any(
    feature = "stm32g473",
    feature = "stm32g483",
    feature = "stm32g474",
    feature = "stm32g484"
))]
impl_comp! {
    COMP5: c5csr,
    COMP6: c6csr,
    COMP7: c7csr,
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

/// Comparator positive input
pub trait PositiveInput<C> {
    fn setup(s: impl stasis::EntitlementLock<Resource = Self>, comp: &mut C);
}

/// Comparator negative input
pub trait NegativeInput<C> {
    /// Does this input use the internal reference Vrefint
    ///
    /// This only true for [`RefintInput`]
    const USE_VREFINT: bool;

    /// Does this input rely on dividing Vrefint using an internal resistor divider
    ///
    /// This is only relevant for [`RefintInput`] other than [`refint_input::VRefint`]
    const USE_RESISTOR_DIVIDER: bool = false;

    fn setup(s: impl stasis::EntitlementLock<Resource = Self>, comp: &mut C);
}

macro_rules! positive_input_pin {
    ($COMP:ident, $pin_0:ident, $pin_1:ident) => {
        impl PositiveInput<$COMP> for gpio::$pin_0<Analog> {
            fn setup(_s: impl stasis::EntitlementLock<Resource = Self>, comp: &mut $COMP) {
                comp.csr().modify(|_, w| w.inpsel().bit(false));
            }
        }

        impl PositiveInput<$COMP> for gpio::$pin_1<Analog> {
            fn setup(_s: impl stasis::EntitlementLock<Resource = Self>, comp: &mut $COMP) {
                comp.csr().modify(|_, w| w.inpsel().bit(true));
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

            fn setup(_s: impl stasis::EntitlementLock<Resource = Self>, comp: &mut $COMP) {
                comp.csr().modify(|_, w| unsafe { w.inmsel().bits($bits) });
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
    COMP1: gpio::PA4<Analog>, gpio::PA0<Analog>,
    COMP2: gpio::PA5<Analog>, gpio::PA2<Analog>,
    COMP3: gpio::PF1<Analog>, gpio::PC0<Analog>,
    COMP4: gpio::PE8<Analog>, gpio::PB2<Analog>,
}

#[cfg(any(
    feature = "stm32g473",
    feature = "stm32g483",
    feature = "stm32g474",
    feature = "stm32g484"
))]
negative_input_pin! {
    COMP5: gpio::PB10<Analog>, gpio::PD13<Analog>,
    COMP6: gpio::PD10<Analog>, gpio::PB15<Analog>,
    COMP7: gpio::PD15<Analog>, gpio::PB12<Analog>,
}

pub mod refint_input {
    /// VRefint * 1/4
    #[derive(Copy, Clone)]
    pub struct VRefintM14;

    /// VRefint * 1/2
    #[derive(Copy, Clone)]
    pub struct VRefintM12;

    /// VRefint * 3/4
    #[derive(Copy, Clone)]
    pub struct VRefintM34;

    /// VRefint
    #[derive(Copy, Clone)]
    pub struct VRefint;
    macro_rules! impl_vrefint {
        ($t:ty, $bits:expr, $use_r_div:expr) => {
            impl super::RefintInput for $t {
                const BITS: u8 = $bits;
                const USE_RESISTOR_DIVIDER: bool = $use_r_div;
            }

            impl crate::stasis::Freeze for $t {}
        };
    }

    impl_vrefint!(VRefintM14, 0b000, true);
    impl_vrefint!(VRefintM12, 0b001, true);
    impl_vrefint!(VRefintM34, 0b010, true);
    impl_vrefint!(VRefint, 0b011, false);
}

pub trait RefintInput {
    const BITS: u8;
    const USE_RESISTOR_DIVIDER: bool;
}

macro_rules! refint_input {
    ($($COMP:ident, )+) => {$(
        impl<REF: RefintInput> NegativeInput<$COMP> for REF {
            const USE_VREFINT: bool = true;
            const USE_RESISTOR_DIVIDER: bool = <REF as RefintInput>::USE_RESISTOR_DIVIDER;

            fn setup(_s: impl stasis::EntitlementLock<Resource = Self>, comp: &mut $COMP) {
                comp.csr()
                    .modify(|_, w| unsafe { w.inmsel().bits(<REF as RefintInput>::BITS) });
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
        impl<ED> NegativeInput<$COMP> for dac::$channel<{ dac::$MODE }, ED> {
            const USE_VREFINT: bool = false;

            fn setup(_s: impl stasis::EntitlementLock<Resource = Self>, comp: &mut $COMP) {
                comp.csr().modify(|_, w| unsafe { w.inmsel().bits($bits) });
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
    fn comparator<P, N, PP, NP>(
        self,
        positive_input: P,
        negative_input: N,
        config: Config,
        clocks: &Clocks,
    ) -> Comparator<COMP, Disabled>
    where
        PP: PositiveInput<COMP>,
        NP: NegativeInput<COMP>,
        P: stasis::EntitlementLock<Resource = PP>,
        N: stasis::EntitlementLock<Resource = NP>;
}

macro_rules! impl_comparator {
    ($COMP:ty, $comp:ident, $Event:expr) => {
        impl ComparatorExt<$COMP> for $COMP {
            fn comparator<P, N, PP, NP>(
                mut self,
                positive_input: P, // TODO: Store these
                negative_input: N, // TODO: Store these
                config: Config,
                clocks: &Clocks,
            ) -> Comparator<$COMP, Disabled>
            where
                PP: PositiveInput<$COMP>,
                NP: NegativeInput<$COMP>,
                P: stasis::EntitlementLock<Resource = PP>,
                N: stasis::EntitlementLock<Resource = NP>,
            {
                PP::setup(positive_input, &mut self);
                NP::setup(negative_input, &mut self);
                // Delay for scaler voltage bridge initialization for certain negative inputs
                let voltage_scaler_delay = clocks.sys_clk.raw() / (1_000_000 / 200); // 200us
                cortex_m::asm::delay(voltage_scaler_delay);
                self.csr().modify(|_, w| unsafe {
                    w.hyst()
                        .bits(config.hysteresis as u8)
                        .scalen()
                        .bit(NP::USE_VREFINT)
                        .brgen()
                        .bit(NP::USE_RESISTOR_DIVIDER)
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
            pub fn $comp<P, N, PP, NP>(
                comp: $COMP,
                positive_input: P,
                negative_input: N,
                config: Config,
                clocks: &Clocks,
            ) -> Self
            where
                PP: PositiveInput<$COMP>,
                NP: NegativeInput<$COMP>,
                P: stasis::EntitlementLock<Resource = PP>,
                N: stasis::EntitlementLock<Resource = NP>,
            {
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
    rcc.rb.apb2enr().modify(|_, w| w.syscfgen().set_bit());

    // Reset COMP, SYSCFG, VREFBUF
    rcc.rb.apb2rstr().modify(|_, w| w.syscfgrst().set_bit());
    rcc.rb.apb2rstr().modify(|_, w| w.syscfgrst().clear_bit());

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
    ($COMP:ident, $pin:ident, $AF:literal, $mode_t:ident, $into:ident) => {
        impl OutputPin<$COMP> for gpio::$pin<Output<$mode_t>> {
            fn setup(self) {
                self.$into::<$AF>();
            }
        }
    };
    ($($COMP:ident: $pin:ident, $AF:literal,)+) => {$(
        output_pin!($COMP, $pin, $AF, PushPull, into_alternate);
        output_pin!($COMP, $pin, $AF, OpenDrain, into_alternate_open_drain);
    )+};
}

output_pin! {
    COMP1: PA0,  8,
    COMP1: PA6,  8,
    COMP1: PA11, 8,
    COMP1: PB8,  8,

    COMP2: PA2,  8,
    COMP2: PA7,  8,
    COMP2: PA12, 8,
    COMP2: PB9,  8,

    COMP3: PB7,  8,
    COMP3: PB15, 3,
    COMP3: PC2,  3,

    COMP4: PB1,  8,
    COMP4: PB6,  8,
    COMP4: PB14, 8,
}

#[cfg(feature = "gpio-g47x")]
output_pin! {
    COMP1: PF4,  2,

    COMP5: PA9,  8,
    COMP5: PC7,  7,

    COMP6: PA10, 8,
    COMP6: PC6,  7,

    COMP7: PA8, 8,
    COMP7: PC8, 7,
}
