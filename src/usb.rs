//! # USB peripheral.
//!
//! Mostly builds upon the [`stm32_usbd`] crate.
//!
//! ## Examples
//!
//! See [examples/usb_serial.rs] for a usage example.

use crate::stm32::{RCC, USB};

use stm32_usbd::UsbPeripheral;

pub use stm32_usbd::UsbBus;

pub struct Peripheral {
    pub usb: USB,
}

unsafe impl Sync for Peripheral {}

unsafe impl UsbPeripheral for Peripheral {
    const REGISTERS: *const () = USB::ptr() as *const ();
    const DP_PULL_UP_FEATURE: bool = true;
    const EP_MEMORY: *const () = 0x4000_6000 as _;
    const EP_MEMORY_SIZE: usize = 1024;
    const EP_MEMORY_ACCESS_2X16: bool = true;

    fn enable() {
        let rcc = unsafe { &*RCC::ptr() };

        cortex_m::interrupt::free(|_| {
            // Enable USB peripheral
            rcc.apb1enr1.modify(|_, w| w.usben().enabled());

            // Reset USB peripheral
            rcc.apb1rstr1.modify(|_, w| w.usbrst().reset());
            rcc.apb1rstr1.modify(|_, w| w.usbrst().clear_bit());
        });
    }

    fn startup_delay() {
        // There is a chip specific startup delay. It is not specified for the STM32G4 but the STM32F103 is 1 us to delay for 170 cycles minimum
        cortex_m::asm::delay(170);
    }
}

pub type UsbBusType = UsbBus<Peripheral>;
