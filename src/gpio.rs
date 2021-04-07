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

/// Alternate function 0 (type state)
pub struct AF0;

/// Alternate function 1 (type state)
pub struct AF1;

/// Alternate function 2 (type state)
pub struct AF2;

/// Alternate function 3 (type state)
pub struct AF3;

/// Alternate function 4 (type state)
pub struct AF4;

/// Alternate function 5 (type state)
pub struct AF5;

/// Alternate function 6 (type state)
pub struct AF6;

/// Alternate function 7 (type state)
pub struct AF7;

/// Alternate function 8 (type state)
pub struct AF8;

/// Alternate function 9 (type state)
pub struct AF9;

/// Alternate function 10 (type state)
pub struct AF10;

/// Alternate function 11 (type state)
pub struct AF11;

/// Alternate function 12 (type state)
pub struct AF12;

/// Alternate function 13 (type state)
pub struct AF13;

/// Alternate function 14 (type state)
pub struct AF14;

/// Alternate function 15 (type state)
pub struct AF15;

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

macro_rules! gpio {
    ([
        $({
            port: ($X:ident/$x:ident, pac: $gpioy:ident),
            pins: [
                $( $i:expr => {
                    reset: $mode:ty,
                    afr: $LH:ident/$lh:ident,
                    af: [$( $af:expr ),*]
                }, )+
            ],
        },)+
    ]) => {
        paste::paste!{
            gpio!([
                $({
                    GPIO: [<GPIO $X>],
                    gpio: [<gpio $x>],
                    gpio_mapped: $gpioy,
                    gpio_mapped_ioenr: [<gpio $x en>],
                    gpio_mapped_iorst: [<iop $x rst>],
                    partially_erased_pin: [<P $X x>],
                    pins: [
                        $(
                            [<P $X $i>]: (
                                [<p $x $i>],
                                $i,
                                $mode,
                                [<moder $i>],
                                [<AFR $LH>],
                                [<afr $lh>],
                                [<afr  $lh $i>],
                                [<bs $i>],
                                [<br $i>],
                                [<odr $i>],
                                [<idr $i>],
                                [<pupdr $i>],
                                [<ot $i>],
                                {
                                    $(
                                        [<AF $af>]: (
                                            [<into_af $af>],
                                            [<af $af>]
                                        ),
                                    )*
                                },
                            ),
                        )+
                    ],
                },)+
            ]);
        }
    };
    ([
        $({
            GPIO: $GPIOX:ident,
            gpio: $gpiox:ident,
            gpio_mapped: $gpioy:ident,
            gpio_mapped_ioenr: $iopxenr:ident,
            gpio_mapped_iorst: $iopxrst:ident,
            partially_erased_pin: $PXx:ident,
            pins: [
                $(
                    $PXi:ident: (
                        $pxi:ident,
                        $i:expr,
                        $MODE:ty,
                        $moderi:ident,
                        $AFR:ident,
                        $afr:ident,
                        $afri:ident,
                        $bsi:ident,
                        $bri:ident,
                        $odri:ident,
                        $idri:ident,
                        $pupdri:ident,
                        $oti:ident,
                        {
                            $(
                                $AFi:ty: (
                                    $into_afi:ident,
                                    $afi:ident
                                ),
                            )*
                        },
                    ),
                )+
            ],
        },)+
    ]) => {
        $(
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
                        $(
                            paste::paste!{
                                #[doc = "Configures `" $PXi "` to serve as alternate function: `" $AFi "`"]
                                pub fn $into_afi(
                                    self,
                                ) -> $PXi<$AFi> {
                                    let moder = unsafe { &(*$GPIOX::ptr()).moder };
                                    moder.modify(|_, w| w.$moderi().alternate());
                                    let afr = unsafe { &(*$GPIOX::ptr()).$afr };
                                    afr.modify(|_, w| w.$afri().$afi());
                                    $PXi { _mode: PhantomData }
                                }
                            }
                        )*
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
        )+
    };
}

include!["gpio_generated.rs"];
