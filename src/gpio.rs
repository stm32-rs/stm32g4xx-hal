//! General Purpose Input / Output
use core::marker::PhantomData;

use crate::rcc::Rcc;

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
pub enum Speed {
    Low = 0,
    Medium = 1,
    High = 2,
    VeryHigh = 3,
}

/// Trigger edgw
pub enum SignalEdge {
    Rising,
    Falling,
    All,
}

#[allow(dead_code)]
pub(crate) enum AltFunction {
    AF0 = 0,
    AF1 = 1,
    AF2 = 2,
    AF3 = 3,
    AF4 = 4,
    AF5 = 5,
    AF6 = 6,
    AF7 = 7,
}

macro_rules! gpio {
    ($GPIOX:ident, $gpiox:ident, $iopxenr:ident, $PXx:ident, $Pxn:expr, [
        $($PXi:ident: ($pxi:ident, $i:expr),)+
    ]) => {
        /// GPIO
        pub mod $gpiox {
            use core::marker::PhantomData;
            use hal::digital::v2::{toggleable, InputPin, OutputPin, StatefulOutputPin};
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
                    rcc.rb.ahb2enr.modify(|_, w| w.$iopxenr().set_bit());
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

            impl<MODE> OutputPin for $PXx<Output<MODE>> {
                type Error = ();

                fn set_high(&mut self) -> Result<(), ()> {
                    // NOTE(unsafe) atomic write to a stateless register
                    unsafe { (*$GPIOX::ptr()).bsrr.write(|w| w.bits(1 << self.i)) };
                    Ok(())
                }

                fn set_low(&mut self) -> Result<(), ()> {
                    // NOTE(unsafe) atomic write to a stateless register
                    unsafe { (*$GPIOX::ptr()).bsrr.write(|w| w.bits(1 << (self.i + 16))) };
                    Ok(())
                }
            }

            impl<MODE> StatefulOutputPin for $PXx<Output<MODE>> {
                fn is_set_high(&self) -> Result<bool, ()> {
                    let is_set_high = !self.is_set_low()?;
                    Ok(is_set_high)
                }

                fn is_set_low(&self) -> Result<bool, ()> {
                    // NOTE(unsafe) atomic read with no side effects
                    let is_set_low = unsafe { (*$GPIOX::ptr()).odr.read().bits() & (1 << self.i) == 0 };
                    Ok(is_set_low)
                }
            }

            impl<MODE> toggleable::Default for $PXx<Output<MODE>> {
            }

            impl<MODE> InputPin for $PXx<Output<MODE>> {
                type Error = ();

                fn is_high(&self) -> Result<bool, ()> {
                    let is_high = !self.is_low()?;
                    Ok(is_high)
                }

                fn is_low(&self) -> Result<bool, ()>  {
                    // NOTE(unsafe) atomic read with no side effects
                    let is_low = unsafe { (*$GPIOX::ptr()).idr.read().bits() & (1 << self.i) == 0 };
                    Ok(is_low)
                }
            }

            impl<MODE> InputPin for $PXx<Input<MODE>> {
                type Error = ();

                fn is_high(&self) -> Result<bool, ()> {
                    let is_high = !self.is_low()?;
                    Ok(is_high)
                }

                fn is_low(&self) -> Result<bool, ()> {
                    // NOTE(unsafe) atomic read with no side effects
                    let is_low = unsafe { (*$GPIOX::ptr()).idr.read().bits() & (1 << self.i) == 0 };
                    Ok(is_low)
                }
            }

            $(
                pub struct $PXi<MODE> {
                    _mode: PhantomData<MODE>,
                }

                impl Into<$PXi<Input<PullDown>>> for $PXi<DefaultMode> {
                    fn into(self) -> $PXi<Input<PullDown>> {
                        self.into_pull_down_input()
                    }
                }

                impl Into<$PXi<Input<PullUp>>> for $PXi<DefaultMode> {
                    fn into(self) -> $PXi<Input<PullUp>> {
                        self.into_pull_up_input()
                    }
                }

                impl Into<$PXi<Analog>> for $PXi<DefaultMode> {
                    fn into(self) -> $PXi<Analog> {
                        self.into_analog()
                    }
                }

                impl Into<$PXi<Output<OpenDrain>>> for $PXi<DefaultMode> {
                    fn into(self) -> $PXi<Output<OpenDrain>> {
                        self.into_open_drain_output()
                    }
                }

                impl Into<$PXi<Output<PushPull>>> for $PXi<DefaultMode> {
                    fn into(self) -> $PXi<Output<PushPull>> {
                        self.into_push_pull_output()
                    }
                }

                impl<MODE> $PXi<MODE> {
                    /// Configures the pin to operate as a floating input pin
                    pub fn into_floating_input(self) -> $PXi<Input<Floating>> {
                        let offset = 2 * $i;
                        unsafe {
                            let gpio = &(*$GPIOX::ptr());
                            gpio.pupdr.modify(|r, w| {
                                w.bits(r.bits() & !(0b11 << offset))
                            });
                            gpio.moder.modify(|r, w| {
                                w.bits(r.bits() & !(0b11 << offset))
                            })
                        };
                        $PXi { _mode: PhantomData }
                    }

                    /// Configures the pin to operate as a pulled down input pin
                    pub fn into_pull_down_input(self) -> $PXi<Input<PullDown>> {
                        let offset = 2 * $i;
                        unsafe {
                            let gpio = &(*$GPIOX::ptr());
                            gpio.pupdr.modify(|r, w| {
                                w.bits((r.bits() & !(0b11 << offset)) | (0b10 << offset))
                            });
                            gpio.moder.modify(|r, w| {
                                w.bits(r.bits() & !(0b11 << offset))
                            })
                        };
                        $PXi { _mode: PhantomData }
                    }

                    /// Configures the pin to operate as a pulled up input pin
                    pub fn into_pull_up_input(self) -> $PXi<Input<PullUp>> {
                        let offset = 2 * $i;
                        unsafe {
                            let gpio = &(*$GPIOX::ptr());
                            gpio.pupdr.modify(|r, w| {
                                w.bits((r.bits() & !(0b11 << offset)) | (0b01 << offset))
                            });
                            gpio.moder.modify(|r, w| {
                                w.bits(r.bits() & !(0b11 << offset))
                            })
                        };
                        $PXi { _mode: PhantomData }
                    }

                    /// Configures the pin to operate as an analog pin
                    pub fn into_analog(self) -> $PXi<Analog> {
                        let offset = 2 * $i;
                        unsafe {
                            let gpio = &(*$GPIOX::ptr());
                            gpio.pupdr.modify(|r, w| {
                                w.bits(r.bits() & !(0b11 << offset))
                            });
                            gpio.moder.modify(|r, w| {
                                w.bits((r.bits() & !(0b11 << offset)) | (0b11 << offset))
                            });
                        }
                        $PXi { _mode: PhantomData }
                    }

                    /// Configures the pin to operate as an open drain output pin
                    pub fn into_open_drain_output(self) -> $PXi<Output<OpenDrain>> {
                        let offset = 2 * $i;
                        unsafe {
                            let gpio = &(*$GPIOX::ptr());
                            gpio.pupdr.modify(|r, w| {
                                w.bits(r.bits() & !(0b11 << offset))
                            });
                            gpio.otyper.modify(|r, w| {
                                w.bits(r.bits() | (0b1 << $i))
                            });
                            gpio.moder.modify(|r, w| {
                                w.bits((r.bits() & !(0b11 << offset)) | (0b01 << offset))
                            })
                        };
                        $PXi { _mode: PhantomData }
                    }

                    /// Configures the pin to operate as an push pull output pin
                    pub fn into_push_pull_output(self) -> $PXi<Output<PushPull>> {
                        let offset = 2 * $i;
                        unsafe {
                            let gpio = &(*$GPIOX::ptr());
                            gpio.pupdr.modify(|r, w| {
                                w.bits(r.bits() & !(0b11 << offset))
                            });
                            gpio.otyper.modify(|r, w| {
                                w.bits(r.bits() & !(0b1 << $i))
                            });
                            gpio.moder.modify(|r, w| {
                                w.bits((r.bits() & !(0b11 << offset)) | (0b01 << offset))
                            })
                        };
                        $PXi { _mode: PhantomData }
                    }

                    /// Configures the pin as external trigger
                    pub fn listen(self, edge: SignalEdge, exti: &mut EXTI) -> $PXi<Input<PushPull>> {
                        let offset = 2 * $i;
                        unsafe {
                            &(*$GPIOX::ptr()).pupdr.modify(|r, w| {
                                w.bits(r.bits() & !(0b11 << offset))
                            });
                            &(*$GPIOX::ptr()).moder.modify(|r, w| {
                                w.bits(r.bits() & !(0b11 << offset))
                            })
                        };
                        exti.listen(Event::from_code($i), edge);
                        $PXi { _mode: PhantomData }
                    }

                    /// Set pin speed
                    pub fn set_speed(self, speed: Speed) -> Self {
                        let offset = 2 * $i;
                        unsafe {
                            &(*$GPIOX::ptr()).ospeedr.modify(|r, w| {
                                w.bits((r.bits() & !(0b11 << offset)) | ((speed as u32) << offset))
                            })
                        };
                        self
                    }

                    #[allow(dead_code)]
                    pub(crate) fn set_alt_mode(&self, mode: AltFunction) {
                        let mode = mode as u32;
                        let offset = 2 * $i;
                        let offset2 = 4 * $i;
                        unsafe {
                            let gpio = &(*$GPIOX::ptr());
                            if offset2 < 32 {
                                gpio.afrl.modify(|r, w| {
                                    w.bits((r.bits() & !(0b1111 << offset2)) | (mode << offset2))
                                });
                            } else {
                                let offset2 = offset2 - 32;
                                gpio.afrh.modify(|r, w| {
                                    w.bits((r.bits() & !(0b1111 << offset2)) | (mode << offset2))
                                });
                            }
                            gpio.moder.modify(|r, w| {
                                w.bits((r.bits() & !(0b11 << offset)) | (0b10 << offset))
                            });
                        }
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
                }

                impl<MODE> OutputPin for $PXi<Output<MODE>> {
                    type Error = ();

                    fn set_high(&mut self) -> Result<(), ()> {
                        // NOTE(unsafe) atomic write to a stateless register
                        unsafe { (*$GPIOX::ptr()).bsrr.write(|w| w.bits(1 << $i)) };
                        Ok(())
                    }

                    fn set_low(&mut self) -> Result<(), ()>{
                        // NOTE(unsafe) atomic write to a stateless register
                        unsafe { (*$GPIOX::ptr()).bsrr.write(|w| w.bits(1 << ($i + 16))) };
                        Ok(())
                    }
                }

                impl<MODE> StatefulOutputPin for $PXi<Output<MODE>> {
                    fn is_set_high(&self) -> Result<bool, ()> {
                        let is_set_high = !self.is_set_low()?;
                        Ok(is_set_high)
                    }

                    fn is_set_low(&self) -> Result<bool, ()> {
                        // NOTE(unsafe) atomic read with no side effects
                        let is_set_low = unsafe { (*$GPIOX::ptr()).odr.read().bits() & (1 << $i) == 0 };
                        Ok(is_set_low)
                    }
                }

                impl<MODE> toggleable::Default for $PXi<Output<MODE>> {
                }

                impl<MODE> InputPin for $PXi<Output<MODE>> {
                    type Error = ();

                    fn is_high(&self) -> Result<bool, ()> {
                        let is_high = !self.is_low()?;
                        Ok(is_high)
                    }

                    fn is_low(&self) -> Result<bool, ()>  {
                        // NOTE(unsafe) atomic read with no side effects
                        let is_low = unsafe { (*$GPIOX::ptr()).idr.read().bits() & (1 << $i) == 0 };
                        Ok(is_low)
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
                }

                impl<MODE> InputPin for $PXi<Input<MODE>> {
                    type Error = ();

                    fn is_high(&self) -> Result<bool, ()> {
                        let is_high = !self.is_low()?;
                        Ok(is_high)
                    }

                    fn is_low(&self) -> Result<bool, ()> {
                        // NOTE(unsafe) atomic read with no side effects
                        let is_low = unsafe { (*$GPIOX::ptr()).idr.read().bits() & (1 << $i) == 0 };
                        Ok(is_low)
                    }
                }
            )+

            impl<TYPE> $PXx<TYPE> {
                pub fn get_id (&self) -> u8 {
                    self.i
                }
            }
        }
    }
}

gpio!(GPIOA, gpioa, gpioaen, PA, 0, [
    PA0: (pa0, 0),
    PA1: (pa1, 1),
    PA2: (pa2, 2),
    PA3: (pa3, 3),
    PA4: (pa4, 4),
    PA5: (pa5, 5),
    PA6: (pa6, 6),
    PA7: (pa7, 7),
    PA8: (pa8, 8),
    PA9: (pa9, 9),
    PA10: (pa10, 10),
    PA11: (pa11, 11),
    PA12: (pa12, 12),
    PA13: (pa13, 13),
    PA14: (pa14, 14),
    PA15: (pa15, 15),
]);

gpio!(GPIOB, gpiob, gpioben, PB, 1, [
    PB0: (pb0, 0),
    PB1: (pb1, 1),
    PB2: (pb2, 2),
    PB3: (pb3, 3),
    PB4: (pb4, 4),
    PB5: (pb5, 5),
    PB6: (pb6, 6),
    PB7: (pb7, 7),
    PB8: (pb8, 8),
    PB9: (pb9, 9),
    PB10: (pb10, 10),
    PB11: (pb11, 11),
    PB12: (pb12, 12),
    PB13: (pb13, 13),
    PB14: (pb14, 14),
    PB15: (pb15, 15),
]);

gpio!(GPIOC, gpioc, gpiocen, PC, 2, [
    PC0: (pc0, 0),
    PC1: (pc1, 1),
    PC2: (pc2, 2),
    PC3: (pc3, 3),
    PC4: (pc4, 4),
    PC5: (pc5, 5),
    PC6: (pc6, 6),
    PC7: (pc7, 7),
    PC8: (pc8, 8),
    PC9: (pc9, 9),
    PC10: (pc10, 10),
    PC11: (pc11, 11),
    PC12: (pc12, 12),
    PC13: (pc13, 13),
    PC14: (pc14, 14),
    PC15: (pc15, 15),
]);

gpio!(GPIOD, gpiod, gpioden, PD, 3, [
    PD0: (pd0, 0),
    PD1: (pd1, 1),
    PD2: (pd2, 2),
    PD3: (pd3, 3),
    PD4: (pd4, 4),
    PD5: (pd5, 5),
    PD6: (pd6, 6),
    PD7: (pd7, 7),
    PD8: (pd8, 8),
    PD9: (pd9, 9),
    PD10: (pd10, 10),
    PD11: (pd11, 11),
    PD12: (pd12, 12),
    PD13: (pd13, 13),
    PD14: (pd14, 14),
    PD15: (pd15, 15),
]);

gpio!(GPIOF, gpiof, gpiofen, PF, 5, [
    PF0: (pf0, 0),
    PF1: (pf1, 1),
    PF2: (pf2, 2),
    PF3: (pf3, 3),
    PF4: (pf4, 4),
    PF5: (pf5, 5),
    PF6: (pf6, 6),
    PF7: (pf7, 7),
    PF8: (pf8, 8),
    PF9: (pf9, 9),
    PF10: (pf10, 10),
    PF11: (pf11, 11),
    PF12: (pf12, 12),
    PF13: (pf13, 13),
    PF14: (pf14, 14),
    PF15: (pf15, 15),
]);

gpio!(GPIOG, gpiog, gpiogen, PG, 6, [
    PG0: (pg0, 0),
    PG1: (pg1, 1),
    PG2: (pg2, 2),
    PG3: (pg3, 3),
    PG4: (pg4, 4),
    PG5: (pg5, 5),
    PG6: (pg6, 6),
    PG7: (pg7, 7),
    PG8: (pg8, 8),
    PG9: (pg9, 9),
    PG10: (pg10, 10),
    PG11: (pg11, 11),
    PG12: (pg12, 12),
    PG13: (pg13, 13),
    PG14: (pg14, 14),
    PG15: (pg15, 15),
]);
