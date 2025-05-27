//! General Purpose Input / Output
use core::marker::PhantomData;

use crate::rcc::{Enable, Rcc, Reset};
use crate::stm32::EXTI;
use crate::syscfg::SysCfg;

use crate::pac;

/// Default pin mode
pub type DefaultMode = Input<Floating>;

/// Extension trait to split a GPIO peripheral in independent pins and registers
pub trait GpioExt {
    /// The parts to split the GPIO into
    type Parts;

    /// Splits the GPIO block into independent pins and registers
    fn split(self, rcc: &mut Rcc) -> Self::Parts;
}

/// Input mode (type state)
pub struct Input<MODE> {
    _mode: PhantomData<MODE>,
}

/// Some alternate mode (type state)
pub struct Alternate<const A: u8, Otype = PushPull>(PhantomData<Otype>);

/// Floating input (type state)
pub struct Floating;

/// Pulled down input (type state)
pub struct PullDown;

/// Pulled up input (type state)
pub struct PullUp;

/// Open drain input or output (type state)
pub struct OpenDrain;

/// Analog mode (type state)
pub struct Analog;

/// Output mode (type state)
pub struct Output<MODE> {
    _mode: PhantomData<MODE>,
}

/// Push pull output (type state)
pub struct PushPull;

/// GPIO Pin speed selection
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Debug, PartialEq, Eq, Clone, Copy)]
pub enum Speed {
    /// Low speed
    Low = 0,
    /// Medium speed
    Medium = 1,
    /// High speed
    High = 2,
    /// Very high speed
    VeryHigh = 3,
}

impl From<Speed> for pac::gpioa::ospeedr::OUTPUT_SPEED {
    fn from(value: Speed) -> Self {
        match value {
            Speed::Low => Self::LowSpeed,
            Speed::Medium => Self::MediumSpeed,
            Speed::High => Self::HighSpeed,
            Speed::VeryHigh => Self::VeryHighSpeed,
        }
    }
}

/// GPIO interrupt trigger edge selection
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Debug, PartialEq, Eq, Clone, Copy)]
pub enum SignalEdge {
    /// Rising edge of voltage
    Rising,
    /// Falling edge of voltage
    Falling,
    /// Rising and falling edge of voltage
    RisingFalling,
}

macro_rules! af {
    ($($i:literal: $AFi:ident),+) => {
        $(
            #[doc = concat!("Alternate function ", $i, " (type state)" )]
            pub type $AFi<Otype = PushPull> = Alternate<$i, Otype>;
        )+
    };
}

af!(
    0: AF0,
    1: AF1,
    2: AF2,
    3: AF3,
    4: AF4,
    5: AF5,
    6: AF6,
    7: AF7,
    8: AF8,
    9: AF9,
    10: AF10,
    11: AF11,
    12: AF12,
    13: AF13,
    14: AF14,
    15: AF15
);

/// External Interrupt Pin
pub trait ExtiPin {
    fn make_interrupt_source(&mut self, syscfg: &mut SysCfg);
    fn trigger_on_edge(&mut self, exti: &mut EXTI, level: SignalEdge);
    fn enable_interrupt(&mut self, exti: &mut EXTI);
    fn disable_interrupt(&mut self, exti: &mut EXTI);
    fn clear_interrupt_pending_bit(&mut self);
    fn check_interrupt(&self) -> bool;
}

macro_rules! exti_erased {
    ($PIN:ty, $extigpionr:expr) => {
        impl<MODE> ExtiPin for $PIN {
            /// Make corresponding EXTI line sensitive to this pin
            fn make_interrupt_source(&mut self, syscfg: &mut SysCfg) {
                let offset = 4 * (self.i % 4);
                match self.i {
                    0..=3 => {
                        syscfg.exticr1().modify(|r, w| unsafe {
                            w.bits((r.bits() & !(0xf << offset)) | ($extigpionr << offset))
                        });
                    }
                    4..=7 => {
                        syscfg.exticr2().modify(|r, w| unsafe {
                            w.bits((r.bits() & !(0xf << offset)) | ($extigpionr << offset))
                        });
                    }
                    8..=11 => {
                        syscfg.exticr3().modify(|r, w| unsafe {
                            w.bits((r.bits() & !(0xf << offset)) | ($extigpionr << offset))
                        });
                    }
                    12..=15 => {
                        syscfg.exticr4().modify(|r, w| unsafe {
                            w.bits((r.bits() & !(0xf << offset)) | ($extigpionr << offset))
                        });
                    }
                    _ => {}
                }
            }

            /// Generate interrupt on rising edge, falling edge or both
            fn trigger_on_edge(&mut self, exti: &mut EXTI, edge: SignalEdge) {
                match edge {
                    SignalEdge::Rising => {
                        exti.rtsr1()
                            .modify(|r, w| unsafe { w.bits(r.bits() | (1 << self.i)) });
                        exti.ftsr1()
                            .modify(|r, w| unsafe { w.bits(r.bits() & !(1 << self.i)) });
                    }
                    SignalEdge::Falling => {
                        exti.ftsr1()
                            .modify(|r, w| unsafe { w.bits(r.bits() | (1 << self.i)) });
                        exti.rtsr1()
                            .modify(|r, w| unsafe { w.bits(r.bits() & !(1 << self.i)) });
                    }
                    SignalEdge::RisingFalling => {
                        exti.rtsr1()
                            .modify(|r, w| unsafe { w.bits(r.bits() | (1 << self.i)) });
                        exti.ftsr1()
                            .modify(|r, w| unsafe { w.bits(r.bits() | (1 << self.i)) });
                    }
                }
            }

            /// Enable external interrupts from this pin.
            fn enable_interrupt(&mut self, exti: &mut EXTI) {
                exti.imr1()
                    .modify(|r, w| unsafe { w.bits(r.bits() | (1 << self.i)) });
            }

            /// Disable external interrupts from this pin
            fn disable_interrupt(&mut self, exti: &mut EXTI) {
                exti.imr1()
                    .modify(|r, w| unsafe { w.bits(r.bits() & !(1 << self.i)) });
            }

            /// Clear the interrupt pending bit for this pin
            fn clear_interrupt_pending_bit(&mut self) {
                unsafe { (*EXTI::ptr()).pr1().write(|w| w.bits(1 << self.i)) };
            }

            /// Reads the interrupt pending bit for this pin
            fn check_interrupt(&self) -> bool {
                unsafe { ((*EXTI::ptr()).pr1().read().bits() & (1 << self.i)) != 0 }
            }
        }
    };
}

macro_rules! exti {
    ($PIN:ty, $extigpionr:expr, $i:expr, $exticri:ident) => {
        impl<MODE> ExtiPin for $PIN {
            /// Configure EXTI Line $i to trigger from this pin.
            fn make_interrupt_source(&mut self, syscfg: &mut SysCfg) {
                let offset = 4 * ($i % 4);
                syscfg.$exticri().modify(|r, w| unsafe {
                    let mut exticr = r.bits();
                    exticr = (exticr & !(0xf << offset)) | ($extigpionr << offset); //FIXME: clears other pins
                    w.bits(exticr)
                });
            }

            /// Generate interrupt on rising edge, falling edge or both
            fn trigger_on_edge(&mut self, exti: &mut EXTI, edge: SignalEdge) {
                match edge {
                    SignalEdge::Rising => {
                        exti.rtsr1()
                            .modify(|r, w| unsafe { w.bits(r.bits() | (1 << $i)) });
                        exti.ftsr1()
                            .modify(|r, w| unsafe { w.bits(r.bits() & !(1 << $i)) });
                    }
                    SignalEdge::Falling => {
                        exti.ftsr1()
                            .modify(|r, w| unsafe { w.bits(r.bits() | (1 << $i)) });
                        exti.rtsr1()
                            .modify(|r, w| unsafe { w.bits(r.bits() & !(1 << $i)) });
                    }
                    SignalEdge::RisingFalling => {
                        exti.rtsr1()
                            .modify(|r, w| unsafe { w.bits(r.bits() | (1 << $i)) });
                        exti.ftsr1()
                            .modify(|r, w| unsafe { w.bits(r.bits() | (1 << $i)) });
                    }
                }
            }

            /// Enable external interrupts from this pin.
            fn enable_interrupt(&mut self, exti: &mut EXTI) {
                exti.imr1()
                    .modify(|r, w| unsafe { w.bits(r.bits() | (1 << $i)) });
            }

            /// Disable external interrupts from this pin
            fn disable_interrupt(&mut self, exti: &mut EXTI) {
                exti.imr1()
                    .modify(|r, w| unsafe { w.bits(r.bits() & !(1 << $i)) });
            }

            /// Clear the interrupt pending bit for this pin
            fn clear_interrupt_pending_bit(&mut self) {
                unsafe { (*EXTI::ptr()).pr1().write(|w| w.bits(1 << $i)) };
            }

            /// Reads the interrupt pending bit for this pin
            fn check_interrupt(&self) -> bool {
                unsafe { ((*EXTI::ptr()).pr1().read().bits() & (1 << $i)) != 0 }
            }
        }
    };
}

macro_rules! gpio {
    ($GPIOX:ident, $gpiox:ident, $iopxenr:ident, $PXx:ident, $Pxn:expr, [
        $($PXi:ident: ($pxi:ident, $i:expr, $exticri:ident),)+
    ]) => {
        /// GPIO
        pub mod $gpiox {
            use core::marker::PhantomData;
            use crate::stm32::{EXTI, $GPIOX};
            use crate::exti::{ExtiExt, Event};
            use crate::rcc::Rcc;
            use super::*;

            /// GPIO parts
            pub struct Parts {
                $(
                    pub $pxi: $PXi<Input<Floating>>,
                )+
            }

            impl GpioExt for $GPIOX {
                type Parts = Parts;

                fn split(self, rcc: &mut Rcc) -> Parts {
                    $GPIOX::enable(&rcc.rb);
                    $GPIOX::reset(&rcc.rb);

                    Parts {
                        $(
                            $pxi: $PXi { _mode: PhantomData },
                        )+
                    }
                }
            }

            /// Partially erased pin
            pub struct $PXx<MODE> {
                i: u8,
                _mode: PhantomData<MODE>,
            }

            impl<MODE> $PXx<Output<MODE>> {
                pub fn set_high(&mut self) {
                    // NOTE(unsafe) atomic write to a stateless register
                    unsafe { (*$GPIOX::ptr()).bsrr().write(|w| w.bs(self.i).set_bit()) };
                }

                pub fn set_low(&mut self) {
                    // NOTE(unsafe) atomic write to a stateless register
                    unsafe { (*$GPIOX::ptr()).bsrr().write(|w| w.br(self.i).set_bit()) };
                }

                pub fn is_set_high(&self) -> bool {
                    !self.is_set_low()
                }

                pub fn is_set_low(&self) -> bool {
                    // NOTE(unsafe) atomic read with no side effects
                    unsafe { (*$GPIOX::ptr()).odr().read().odr(self.i).is_low()}
                }

                pub fn toggle(&mut self) {
                    if self.is_set_low() {
                        self.set_high()
                    } else {
                        self.set_low()
                    }
                }
            }

            impl<MODE> embedded_hal_old::digital::v2::OutputPin for $PXx<Output<MODE>> {
                type Error = ();

                fn set_high(&mut self) -> Result<(), ()> {
                    self.set_high();
                    Ok(())
                }

                fn set_low(&mut self) -> Result<(), ()> {
                    self.set_low();
                    Ok(())
                }
            }

            impl<MODE> embedded_hal::digital::ErrorType for $PXx<MODE> {
                type Error = core::convert::Infallible;
            }

            impl<MODE> embedded_hal::digital::OutputPin for $PXx<Output<MODE>> {
                fn set_high(&mut self) -> Result<(), Self::Error> {
                    self.set_high();
                    Ok(())
                }

                fn set_low(&mut self) -> Result<(), Self::Error> {
                    self.set_low();
                    Ok(())
                }
            }

            impl<MODE> embedded_hal_old::digital::v2::StatefulOutputPin for $PXx<Output<MODE>> {
                fn is_set_high(&self) -> Result<bool, ()> {
                    Ok(self.is_set_high())
                }

                fn is_set_low(&self) -> Result<bool, ()> {
                    Ok(self.is_set_low())
                }
            }

            impl<MODE> embedded_hal::digital::StatefulOutputPin for $PXx<Output<MODE>> {
                fn is_set_high(&mut self) -> Result<bool, Self::Error> {
                    Ok((*self).is_set_high())
                }

                fn is_set_low(&mut self) -> Result<bool, Self::Error> {
                    Ok((*self).is_set_low())
                }
            }

            impl $PXx<Output<OpenDrain>> {
                pub fn is_high(&self) -> bool {
                    !self.is_low()
                }

                pub fn is_low(&self) -> bool  {
                    // NOTE(unsafe) atomic read with no side effects
                    unsafe { (*$GPIOX::ptr()).idr().read().idr(self.i).is_low() }
                }
            }

            impl<MODE> embedded_hal_old::digital::v2::toggleable::Default for $PXx<Output<MODE>> {}

            impl embedded_hal_old::digital::v2::InputPin for $PXx<Output<OpenDrain>> {
                type Error = ();

                fn is_high(&self) -> Result<bool, ()> {
                    Ok(self.is_high())
                }

                fn is_low(&self) -> Result<bool, ()>  {
                    Ok(self.is_low())
                }
            }

            impl embedded_hal::digital::InputPin for $PXx<Output<OpenDrain>> {
                fn is_high(&mut self) -> Result<bool, Self::Error> {
                    Ok((*self).is_high())
                }

                fn is_low(&mut self) -> Result<bool, Self::Error>  {
                    Ok((*self).is_low())
                }
            }

            impl<MODE> $PXx<Input<MODE>> {
                pub fn is_high(&self) -> bool {
                    !self.is_low()
                }

                pub fn is_low(&self) -> bool {
                    // NOTE(unsafe) atomic read with no side effects
                    unsafe { (*$GPIOX::ptr()).idr().read().idr(self.i).is_low() }
                }
            }

            impl<MODE> embedded_hal_old::digital::v2::InputPin for $PXx<Input<MODE>> {
                type Error = ();

                fn is_high(&self) -> Result<bool, ()> {
                    Ok(self.is_high())
                }

                fn is_low(&self) -> Result<bool, ()> {
                    Ok(self.is_low())
                }
            }

            impl<MODE> embedded_hal::digital::InputPin for $PXx<Input<MODE>> {
                fn is_high(&mut self) -> Result<bool, Self::Error> {
                    Ok((*self).is_high())
                }

                fn is_low(&mut self) -> Result<bool, Self::Error> {
                    Ok((*self).is_low())
                }
            }

            exti_erased!($PXx<Output<MODE>>, $Pxn);
            exti_erased!($PXx<Input<MODE>>, $Pxn);

            $(
                pub struct $PXi<MODE> {
                    _mode: PhantomData<MODE>,
                }

                impl<MODE> crate::Sealed for $PXi<MODE> {}

                #[allow(clippy::from_over_into)]
                impl Into<$PXi<Input<PullDown>>> for $PXi<DefaultMode> {
                    fn into(self) -> $PXi<Input<PullDown>> {
                        self.into_pull_down_input()
                    }
                }

                #[allow(clippy::from_over_into)]
                impl Into<$PXi<Input<PullUp>>> for $PXi<DefaultMode> {
                    fn into(self) -> $PXi<Input<PullUp>> {
                        self.into_pull_up_input()
                    }
                }

                #[allow(clippy::from_over_into)]
                impl Into<$PXi<Analog>> for $PXi<DefaultMode> {
                    fn into(self) -> $PXi<Analog> {
                        self.into_analog()
                    }
                }

                #[allow(clippy::from_over_into)]
                impl Into<$PXi<Output<OpenDrain>>> for $PXi<DefaultMode> {
                    fn into(self) -> $PXi<Output<OpenDrain>> {
                        self.into_open_drain_output()
                    }
                }

                #[allow(clippy::from_over_into)]
                impl Into<$PXi<Output<PushPull>>> for $PXi<DefaultMode> {
                    fn into(self) -> $PXi<Output<PushPull>> {
                        self.into_push_pull_output()
                    }
                }

                impl<MODE> crate::stasis::Freeze for $PXi<MODE> { }

                impl<MODE> $PXi<MODE> {
                    /// Configures the pin to operate as a floating input pin
                    pub fn into_floating_input(self) -> $PXi<Input<Floating>> {
                        unsafe {
                            let gpio = &(*$GPIOX::ptr());
                            gpio.pupdr().modify(|_, w| {
                                w.pupdr($i).floating()
                            });
                            gpio.moder().modify(|_, w| {
                                w.moder($i).input()
                            })
                        };
                        $PXi { _mode: PhantomData }
                    }

                    /// Configures the pin to operate as a pulled down input pin
                    pub fn into_pull_down_input(self) -> $PXi<Input<PullDown>> {
                        unsafe {
                            let gpio = &(*$GPIOX::ptr());
                            gpio.pupdr().modify(|_, w| {
                                w.pupdr($i).pull_down()
                            });
                            gpio.moder().modify(|_, w| {
                                w.moder($i).input()
                            })
                        };
                        $PXi { _mode: PhantomData }
                    }

                    /// Configures the pin to operate as a pulled up input pin
                    pub fn into_pull_up_input(self) -> $PXi<Input<PullUp>> {
                        unsafe {
                            let gpio = &(*$GPIOX::ptr());
                            gpio.pupdr().modify(|_, w| {
                                w.pupdr($i).pull_up()
                            });
                            gpio.moder().modify(|_, w| {
                                w.moder($i).input()
                            })
                        };
                        $PXi { _mode: PhantomData }
                    }

                    /// Configures the pin to operate as an analog pin
                    pub fn into_analog(self) -> $PXi<Analog> {
                        unsafe {
                            let gpio = &(*$GPIOX::ptr());
                            gpio.pupdr().modify(|_, w| {
                                w.pupdr($i).floating()
                            });
                            gpio.moder().modify(|_, w| {
                                w.moder($i).analog()
                            });
                        }
                        $PXi { _mode: PhantomData }
                    }

                    /// Configures the pin to operate as an open drain output pin
                    pub fn into_open_drain_output(self) -> $PXi<Output<OpenDrain>> {
                        unsafe {
                            let gpio = &(*$GPIOX::ptr());
                            gpio.pupdr().modify(|_, w| {
                                w.pupdr($i).floating()
                            });
                            gpio.otyper().modify(|_, w| {
                                w.ot($i).open_drain()
                            });
                            gpio.moder().modify(|_, w| {
                                w.moder($i).output()
                            })
                        };
                        $PXi { _mode: PhantomData }
                    }

                    /// Configures the pin to operate as an push pull output pin
                    pub fn into_push_pull_output(self) -> $PXi<Output<PushPull>> {
                        unsafe {
                            let gpio = &(*$GPIOX::ptr());
                            gpio.pupdr().modify(|_, w| {
                                w.pupdr($i).floating()
                            });
                            gpio.otyper().modify(|_, w| {
                                w.ot($i).push_pull()
                            });
                            gpio.moder().modify(|_, w| {
                                w.moder($i).output()
                            })
                        };
                        $PXi { _mode: PhantomData }
                    }

                    /// Configures the pin as external trigger
                    pub fn listen(self, edge: SignalEdge, exti: &mut EXTI) -> $PXi<MODE> {
                        exti.listen(Event::from_code($i), edge);
                        $PXi { _mode: PhantomData }
                    }

                    /// Set pin speed
                    pub fn set_speed(&mut self, speed: Speed) {
                        unsafe {
                            (*$GPIOX::ptr()).ospeedr()
                                .modify(|_, w| w.ospeedr($i).variant(speed.into()));
                        }
                    }

                    /// Set pin speed
                    pub fn speed(mut self, speed: Speed) -> Self {
                        self.set_speed(speed);
                        self
                    }

                    pub fn into_alternate<const A: u8>(self) -> $PXi<Alternate<A>> {
                        let mode = A as u8;
                        unsafe {
                            let gpio = &(*$GPIOX::ptr());
                            if $i < 8 {
                                gpio.afrl().modify(|_, w| {
                                    w.afr($i).set(mode)
                                });
                            } else {
                                let offset = $i - 8;
                                gpio.afrh().modify(|_, w| {
                                    w.afr(offset).set(mode)
                                });
                            }
                            gpio.moder().modify(|_, w| {
                                w.moder($i).alternate()
                            });
                            gpio.otyper().modify(|_, w| {
                                w.ot($i).push_pull()
                            });
                        }
                        $PXi { _mode: PhantomData }
                    }

                    pub fn into_alternate_open_drain<const A: u8>(self) -> $PXi<Alternate<A, OpenDrain>> {
                        let mode = A as u8;
                        unsafe {
                            let gpio = &(*$GPIOX::ptr());
                            if $i < 8 {
                                gpio.afrl().modify(|_, w| {
                                    w.afr($i).set(mode)
                                });
                            } else {
                                let offset = $i - 8;
                                gpio.afrh().modify(|_, w| {
                                    w.afr(offset).set(mode)
                                });
                            }
                            gpio.otyper().modify(|_, w| {
                                w.ot($i).open_drain()
                            });
                            gpio.moder().modify(|_, w| {
                                w.moder($i).alternate()
                            });
                        }
                        $PXi { _mode: PhantomData }
                    }
                }

                impl<MODE> $PXi<Output<MODE>> {
                    /// Erases the pin number from the type
                    ///
                    /// This is useful when you want to collect the pins into an array where you
                    /// need all the elements to have the same type
                    pub fn downgrade(self) -> $PXx<Output<MODE>> {
                        $PXx { i: $i, _mode: self._mode }
                    }

                    pub fn set_high(&mut self) {
                        // NOTE(unsafe) atomic write to a stateless register
                        unsafe { (*$GPIOX::ptr()).bsrr().write(|w| w.bs($i).set_bit()) };
                    }

                    pub fn set_low(&mut self) {
                        // NOTE(unsafe) atomic write to a stateless register
                        unsafe { (*$GPIOX::ptr()).bsrr().write(|w| w.br($i).set_bit()) };
                    }

                    pub fn is_set_high(&self) -> bool {
                        !self.is_set_low()
                    }

                    pub fn is_set_low(&self) -> bool {
                        // NOTE(unsafe) atomic read with no side effects
                        unsafe { (*$GPIOX::ptr()).odr().read().odr($i).is_low()}
                    }

                    pub fn toggle(&mut self) {
                        if self.is_set_low() {
                            self.set_high()
                        } else {
                            self.set_low()
                        }
                    }
                }

                impl<MODE> embedded_hal_old::digital::v2::OutputPin for $PXi<Output<MODE>> {
                    type Error = ();

                    fn set_high(&mut self) -> Result<(), ()> {
                        self.set_high();
                        Ok(())
                    }

                    fn set_low(&mut self) -> Result<(), ()>{
                        self.set_low();
                        Ok(())
                    }
                }

                impl<MODE> embedded_hal::digital::ErrorType for $PXi<MODE> {
                    type Error = core::convert::Infallible;
                }

                impl<MODE> embedded_hal::digital::OutputPin for $PXi<Output<MODE>> {
                    fn set_high(&mut self) -> Result<(), Self::Error> {
                        self.set_high();
                        Ok(())
                    }

                    fn set_low(&mut self) -> Result<(), Self::Error>{
                        self.set_low();
                        Ok(())
                    }
                }

                impl<MODE> embedded_hal_old::digital::v2::StatefulOutputPin for $PXi<Output<MODE>> {
                    fn is_set_high(&self) -> Result<bool, ()> {
                        Ok(self.is_set_high())
                    }

                    fn is_set_low(&self) -> Result<bool, ()> {
                        Ok(self.is_set_low())
                    }
                }

                impl<MODE> embedded_hal::digital::StatefulOutputPin for $PXi<Output<MODE>> {
                    fn is_set_high(&mut self) -> Result<bool, Self::Error> {
                        Ok((*self).is_set_high())
                    }

                    fn is_set_low(&mut self) -> Result<bool, Self::Error> {
                        Ok((*self).is_set_low())
                    }
                }

                impl<MODE> embedded_hal_old::digital::v2::toggleable::Default for $PXi<Output<MODE>> {}

                impl $PXi<Output<OpenDrain>> {
                    pub fn is_high(&self) -> bool {
                        !self.is_low()
                    }

                    pub fn is_low(&self) -> bool  {
                        // NOTE(unsafe) atomic read with no side effects
                        unsafe { (*$GPIOX::ptr()).idr().read().idr($i).is_low() }
                    }
                }

                impl embedded_hal_old::digital::v2::InputPin for $PXi<Output<OpenDrain>> {
                    type Error = ();

                    fn is_high(&self) -> Result<bool, ()> {
                        Ok(self.is_high())
                    }

                    fn is_low(&self) -> Result<bool, ()>  {
                        Ok(self.is_low())
                    }
                }

                impl embedded_hal::digital::InputPin for $PXi<Output<OpenDrain>> {
                    fn is_high(&mut self) -> Result<bool, Self::Error> {
                        Ok((*self).is_high())
                    }

                    fn is_low(&mut self) -> Result<bool, Self::Error>  {
                        Ok((*self).is_low())
                    }
                }

                impl<MODE> $PXi<Input<MODE>> {
                    /// Erases the pin number from the type
                    ///
                    /// This is useful when you want to collect the pins into an array where you
                    /// need all the elements to have the same type
                    pub fn downgrade(self) -> $PXx<Input<MODE>> {
                        $PXx { i: $i, _mode: self._mode }
                    }

                    pub fn is_high(&self) -> bool {
                        !self.is_low()
                    }

                    pub fn is_low(&self) -> bool  {
                        // NOTE(unsafe) atomic read with no side effects
                        unsafe { (*$GPIOX::ptr()).idr().read().idr($i).is_low() }
                    }
                }

                impl<MODE> embedded_hal::digital::InputPin for $PXi<Input<MODE>> {
                    fn is_high(&mut self) -> Result<bool, Self::Error> {
                        Ok((*self).is_high())
                    }

                    fn is_low(&mut self) -> Result<bool, Self::Error> {
                        Ok((*self).is_low())
                    }
                }

                impl<MODE> embedded_hal_old::digital::v2::InputPin for $PXi<Input<MODE>> {
                    type Error = ();

                    fn is_high(&self) -> Result<bool, ()> {
                        Ok(self.is_high())
                    }

                    fn is_low(&self) -> Result<bool, ()> {
                        Ok(self.is_low())
                    }
                }

                exti!($PXi<Output<MODE>>, $Pxn, $i, $exticri);
                exti!($PXi<Input<MODE>>, $Pxn, $i, $exticri);
            )+

            impl<TYPE> $PXx<TYPE> {
                pub fn get_id (&self) -> u8 {
                    self.i
                }
            }
        }

        pub use $gpiox::{ $($PXi,)+ };
    }
}

gpio!(GPIOA, gpioa, gpioaen, PA, 0, [
    PA0: (pa0, 0, exticr1),
    PA1: (pa1, 1, exticr1),
    PA2: (pa2, 2, exticr1),
    PA3: (pa3, 3, exticr1),
    PA4: (pa4, 4, exticr2),
    PA5: (pa5, 5, exticr2),
    PA6: (pa6, 6, exticr2),
    PA7: (pa7, 7, exticr2),
    PA8: (pa8, 8, exticr3),
    PA9: (pa9, 9, exticr3),
    PA10: (pa10, 10, exticr3),
    PA11: (pa11, 11, exticr3),
    PA12: (pa12, 12, exticr4),
    PA13: (pa13, 13, exticr4),
    PA14: (pa14, 14, exticr4),
    PA15: (pa15, 15, exticr4),
]);

gpio!(GPIOB, gpiob, gpioben, PB, 1, [
    PB0: (pb0, 0, exticr1),
    PB1: (pb1, 1, exticr1),
    PB2: (pb2, 2, exticr1),
    PB3: (pb3, 3, exticr1),
    PB4: (pb4, 4, exticr2),
    PB5: (pb5, 5, exticr2),
    PB6: (pb6, 6, exticr2),
    PB7: (pb7, 7, exticr2),
    PB8: (pb8, 8, exticr3),
    PB9: (pb9, 9, exticr3),
    PB10: (pb10, 10, exticr3),
    PB11: (pb11, 11, exticr3),
    PB12: (pb12, 12, exticr4),
    PB13: (pb13, 13, exticr4),
    PB14: (pb14, 14, exticr4),
    PB15: (pb15, 15, exticr4),
]);

gpio!(GPIOC, gpioc, gpiocen, PC, 2, [
    PC0: (pc0, 0, exticr1),
    PC1: (pc1, 1, exticr1),
    PC2: (pc2, 2, exticr1),
    PC3: (pc3, 3, exticr1),
    PC4: (pc4, 4, exticr2),
    PC5: (pc5, 5, exticr2),
    PC6: (pc6, 6, exticr2),
    PC7: (pc7, 7, exticr2),
    PC8: (pc8, 8, exticr3),
    PC9: (pc9, 9, exticr3),
    PC10: (pc10, 10, exticr3),
    PC11: (pc11, 11, exticr3),
    PC12: (pc12, 12, exticr4),
    PC13: (pc13, 13, exticr4),
    PC14: (pc14, 14, exticr4),
    PC15: (pc15, 15, exticr4),
]);

gpio!(GPIOD, gpiod, gpioden, PD, 3, [
    PD0: (pd0, 0, exticr1),
    PD1: (pd1, 1, exticr1),
    PD2: (pd2, 2, exticr1),
    PD3: (pd3, 3, exticr1),
    PD4: (pd4, 4, exticr2),
    PD5: (pd5, 5, exticr2),
    PD6: (pd6, 6, exticr2),
    PD7: (pd7, 7, exticr2),
    PD8: (pd8, 8, exticr3),
    PD9: (pd9, 9, exticr3),
    PD10: (pd10, 10, exticr3),
    PD11: (pd11, 11, exticr3),
    PD12: (pd12, 12, exticr4),
    PD13: (pd13, 13, exticr4),
    PD14: (pd14, 14, exticr4),
    PD15: (pd15, 15, exticr4),
]);

gpio!(GPIOE, gpioe, gpioeen, PE, 4, [
    PE0: (pe0, 0, exticr1),
    PE1: (pe1, 1, exticr1),
    PE2: (pe2, 2, exticr1),
    PE3: (pe3, 3, exticr1),
    PE4: (pe4, 4, exticr2),
    PE5: (pe5, 5, exticr2),
    PE6: (pe6, 6, exticr2),
    PE7: (pe7, 7, exticr2),
    PE8: (pe8, 8, exticr3),
    PE9: (pe9, 9, exticr3),
    PE10: (pe10, 10, exticr3),
    PE11: (pe11, 11, exticr3),
    PE12: (pe12, 12, exticr4),
    PE13: (pe13, 13, exticr4),
    PE14: (pe14, 14, exticr4),
    PE15: (pe15, 15, exticr4),
]);

gpio!(GPIOF, gpiof, gpiofen, PF, 5, [
    PF0: (pf0, 0, exticr1),
    PF1: (pf1, 1, exticr1),
    PF2: (pf2, 2, exticr1),
    PF3: (pf3, 3, exticr1),
    PF4: (pf4, 4, exticr2),
    PF5: (pf5, 5, exticr2),
    PF6: (pf6, 6, exticr2),
    PF7: (pf7, 7, exticr2),
    PF8: (pf8, 8, exticr3),
    PF9: (pf9, 9, exticr3),
    PF10: (pf10, 10, exticr3),
    PF11: (pf11, 11, exticr3),
    PF12: (pf12, 12, exticr4),
    PF13: (pf13, 13, exticr4),
    PF14: (pf14, 14, exticr4),
    PF15: (pf15, 15, exticr4),
]);

gpio!(GPIOG, gpiog, gpiogen, PG, 6, [
    PG0: (pg0, 0, exticr1),
    PG1: (pg1, 1, exticr1),
    PG2: (pg2, 2, exticr1),
    PG3: (pg3, 3, exticr1),
    PG4: (pg4, 4, exticr2),
    PG5: (pg5, 5, exticr2),
    PG6: (pg6, 6, exticr2),
    PG7: (pg7, 7, exticr2),
    PG8: (pg8, 8, exticr3),
    PG9: (pg9, 9, exticr3),
    PG10: (pg10, 10, exticr3),
    PG11: (pg11, 11, exticr3),
    PG12: (pg12, 12, exticr4),
    PG13: (pg13, 13, exticr4),
    PG14: (pg14, 14, exticr4),
    PG15: (pg15, 15, exticr4),
]);
