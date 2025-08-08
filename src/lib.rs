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

pub extern crate cortex_m;
pub extern crate nb;
pub extern crate stm32g4;

pub use embedded_hal as hal;
pub use embedded_hal_old as hal_02;
pub use nb::block;

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
#[cfg(feature = "fmac")]
pub mod fmac;
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

mod sealed {
    pub trait Sealed {}
}
pub(crate) use sealed::Sealed;

impl<RB, const A: usize> Sealed for Periph<RB, A> {}

pub trait Ptr: Sealed {
    /// RegisterBlock structure
    type RB;
    /// Return the pointer to the register block
    fn ptr() -> *const Self::RB;
}

impl<RB, const A: usize> Ptr for Periph<RB, A> {
    type RB = RB;
    fn ptr() -> *const Self::RB {
        Self::ptr()
    }
}

fn stripped_type_name<T>() -> &'static str {
    let s = core::any::type_name::<T>();
    let p = s.split("::");
    p.last().unwrap()
}
