#![no_std]
#![allow(non_camel_case_types)]

#[cfg(not(any(
   feature = "stm32g431",
//   feature = "stm32g441",
//   feature = "stm32g491",
//   feature = "stm32g473",
//   feature = "stm32g483",
   feature = "stm32g474",
//   feature = "stm32g484",
)))]
compile_error!("This crate requires one of the following features enabled: stm32g431, stm32g441, stm32g491, stm32g473, stm32g483, stm32g474 or stm32g484");

use bare_metal;
use void;

pub use cortex_m;
pub use embedded_hal as hal;
pub use nb;
//pub extern crate stm32g4;

pub use nb::block;

#[cfg(feature = "stm32g431")]
pub use stm32g4::stm32g431 as pac;
#[cfg(feature = "stm32g474")]
pub use stm32g4::stm32g474 as pac;

pub use pac as stm32;

#[cfg(feature = "rt")]
pub use crate::pac::interrupt;

// pub mod adc;
// pub mod crc;
// pub mod dac;
pub mod delay;
// pub mod dma;
pub mod exti;
pub mod gpio;
// pub mod i2c;
pub mod prelude;
// pub mod pwm;
// pub mod qei;
pub mod rcc;
// pub mod rng;
// pub mod serial;
// pub mod spi;
// pub mod stopwatch;
pub mod time;
pub mod timer;
// pub mod watchdog;
