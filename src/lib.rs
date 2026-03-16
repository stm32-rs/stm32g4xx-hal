#![no_std]
#![allow(non_camel_case_types)]

#[cfg(not(any(
    feature = "stm32g431",
    feature = "stm32g441",
    feature = "stm32g473",
    feature = "stm32g474",
    feature = "stm32g483",
    feature = "stm32g484",
    feature = "stm32g491",
    feature = "stm32g4a1"
)))]

compile_error!(
    "This crate requires one of the following features enabled:
        stm32g431
        stm32g441
        stm32g473
        stm32g474
        stm32g483
        stm32g484
        stm32g491
        stm32g4a1"
);

pub use cortex_m;
pub use nb;
pub use stm32g4;

pub use embedded_hal as hal;
pub use embedded_hal_old as hal_02;
pub use nb::block;

use enumflags2::{BitFlag, BitFlags};

#[cfg(feature = "stm32g431")]
pub use stm32g4::stm32g431 as stm32;

#[cfg(feature = "stm32g441")]
pub use stm32g4::stm32g441 as stm32;

#[cfg(feature = "stm32g473")]
pub use stm32g4::stm32g473 as stm32;

#[cfg(feature = "stm32g474")]
pub use stm32g4::stm32g474 as stm32;

#[cfg(feature = "stm32g483")]
pub use stm32g4::stm32g483 as stm32;

#[cfg(feature = "stm32g484")]
pub use stm32g4::stm32g484 as stm32;

#[cfg(feature = "stm32g491")]
pub use stm32g4::stm32g491 as stm32;

#[cfg(feature = "stm32g4a1")]
pub use stm32g4::stm32g4a1 as stm32;

pub use stm32 as pac;
use stm32g4::Periph;

#[cfg(feature = "rt")]
pub use crate::stm32::interrupt;

pub mod adc;
pub mod bb;
#[cfg(feature = "can")]
pub mod can;
pub mod comparator;
#[cfg(feature = "cordic")]
pub mod cordic;
// pub mod crc;
pub mod dac;
pub mod delay;
pub mod dma;
pub mod exti;
pub mod flash;
pub mod gpio;
pub mod i2c;
pub mod opamp;
pub mod prelude;
pub mod pwm;
pub mod pwr;
// pub mod qei;
pub mod rcc;
// pub mod rng;
pub mod serial;
pub mod signature;
pub mod spi;
pub mod stasis;
// pub mod stopwatch;
pub mod syscfg;
pub mod time;
pub mod timer;
// pub mod watchdog;
pub mod rng;

#[cfg(all(
    feature = "hrtim",
    not(any(feature = "stm32g474", feature = "stm32g484"))
))]
compile_error!("`hrtim` is only available for stm32g474 and stm32g484");

#[cfg(feature = "hrtim")]
pub mod hrtim;
pub mod independent_watchdog;
#[cfg(feature = "usb")]
pub mod usb;

pub trait ReadFlags {
    /// Enum of bit flags
    type Flag: BitFlag;

    /// Get all interrupts flags a once.
    fn flags(&self) -> BitFlags<Self::Flag>;
}

pub trait ClearFlags {
    /// Enum of manually clearable flags
    type Flag: BitFlag;

    /// Clear interrupts flags with `Self::Flags`s
    ///
    /// If event flag is not cleared, it will immediately retrigger interrupt
    /// after interrupt handler has finished.
    fn clear_flags(&mut self, flags: impl Into<BitFlags<Self::Flag>>);

    /// Clears all interrupts flags
    #[inline(always)]
    fn clear_all_flags(&mut self) {
        self.clear_flags(BitFlags::ALL)
    }
}

pub trait Listen {
    /// Enum of bit flags associated with events
    type Event: BitFlag;

    #[doc(hidden)]
    fn listen_event(
        &mut self,
        disable: Option<BitFlags<Self::Event>>,
        enable: Option<BitFlags<Self::Event>>,
    );

    /// Start listening for `Event`s
    ///
    /// Note, you will also have to enable the appropriate interrupt in the NVIC to start
    /// receiving events.
    #[inline(always)]
    fn listen(&mut self, event: impl Into<BitFlags<Self::Event>>) {
        self.listen_event(None, Some(event.into()));
    }

    /// Start listening for `Event`s, stop all other
    ///
    /// Note, you will also have to enable the appropriate interrupt in the NVIC to start
    /// receiving events.
    #[inline(always)]
    fn listen_only(&mut self, event: impl Into<BitFlags<Self::Event>>) {
        self.listen_event(Some(BitFlags::ALL), Some(event.into()));
    }

    /// Stop listening for `Event`s
    fn unlisten(&mut self, event: impl Into<BitFlags<Self::Event>>) {
        self.listen_event(Some(event.into()), None);
    }

    /// Start listening all `Event`s
    #[inline(always)]
    fn listen_all(&mut self) {
        self.listen(BitFlags::ALL)
    }

    /// Stop listening all `Event`s
    #[inline(always)]
    fn unlisten_all(&mut self) {
        self.unlisten(BitFlags::ALL)
    }
}

mod sealed {
    pub trait Sealed {}
}
pub(crate) use sealed::Sealed;

impl<RB, const A: usize> Sealed for Periph<RB, A> {}

pub trait Ptr: Sealed {
    /// RegisterBlock structure
    type RB;
    /// Pointer to the register block
    const PTR: *const Self::RB;
    /// Return the pointer to the register block
    #[inline(always)]
    fn ptr() -> *const Self::RB {
        Self::PTR
    }
}

impl<RB, const A: usize> Ptr for Periph<RB, A> {
    type RB = RB;
    const PTR: *const Self::RB = Self::PTR;
}

pub trait Steal: Sealed {
    /// Steal an instance of this peripheral
    ///
    /// # Safety
    ///
    /// Ensure that the new instance of the peripheral cannot be used in a way
    /// that may race with any existing instances, for example by only
    /// accessing read-only or write-only registers, or by consuming the
    /// original peripheral and using critical sections to coordinate
    /// access between multiple new instances.
    ///
    /// Additionally the HAL may rely on only one
    /// peripheral instance existing to ensure memory safety; ensure
    /// no stolen instances are passed to such software.
    unsafe fn steal() -> Self;
}

impl<RB, const A: usize> Steal for Periph<RB, A> {
    #[inline(always)]
    unsafe fn steal() -> Self {
        Self::steal()
    }
}

fn stripped_type_name<T>() -> &'static str {
    let s = core::any::type_name::<T>();
    let p = s.split("::");
    p.last().unwrap()
}
