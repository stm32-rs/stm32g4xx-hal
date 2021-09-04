//! UART serial port support.
//!
//! This module provides support for asynchronous communication using UARTs/USARTs/LPUARTs in
//! configurations without hardware flow control. Correct usage is shown by the `uart`,
//! `uart-fifo`, and `uart-dma` examples.
//!
//! **Note that the APB clock needs to be at least 16 times faster than the UART baud rate for all
//! UARTs except for the LPUART.** The latter contains an internal 256x clock multiplier.
//!
//! Most of this code was originally taken from `stm32g0xx-hal`.
pub mod config;
pub mod usart;

pub use config::*;
pub use usart::*;
