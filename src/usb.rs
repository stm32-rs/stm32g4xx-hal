//! USB peripheral.
//!
//! Provides the required implementation for use of the [`stm32-usbd`] crate.

pub use stm32_usbd::UsbBus;

use crate::gpio;
use crate::pac::USB;
use crate::rcc::{Enable, Reset};
use core::fmt;
use stm32_usbd::UsbPeripheral;

/// Trait implemented by all pins that can be the "D-" pin for the USB peripheral
pub trait DmPin: crate::Sealed {}

/// Trait implemented by all pins that can be the "D+" pin for the USB peripheral
pub trait DpPin: crate::Sealed {}

impl DmPin for gpio::PA11<gpio::AF14> {}
impl DpPin for gpio::PA12<gpio::AF14> {}

pub struct Peripheral<Dm: DmPin, Dp: DpPin> {
    /// USB register block
    pub usb: USB,
    /// Data negative pin
    pub pin_dm: Dm,
    /// Data positive pin
    pub pin_dp: Dp,
}

impl<Dm, Dp> fmt::Debug for Peripheral<Dm, Dp>
where
    Dm: DmPin + fmt::Debug,
    Dp: DpPin + fmt::Debug,
{
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("Peripheral")
            .field("usb", &"USB")
            .field("pin_dm", &self.pin_dm)
            .field("pin_dp", &self.pin_dp)
            .finish()
    }
}

// SAFETY: Implementation of Peripheral is thread-safe by using cricitcal sections to ensure
// mutually exclusive access to the USB peripheral
unsafe impl<Dm: DmPin, Dp: DpPin> Sync for Peripheral<Dm, Dp> {}

// SAFETY: The peripheral has the same regiter blockout as the STM32 USBFS
unsafe impl<Dm: DmPin + Send, Dp: DpPin + Send> UsbPeripheral for Peripheral<Dm, Dp> {
    const REGISTERS: *const () = USB::ptr().cast::<()>();
    const DP_PULL_UP_FEATURE: bool = true;
    const EP_MEMORY: *const () = 0x4000_6000 as _;
    const EP_MEMORY_SIZE: usize = 1024;
    const EP_MEMORY_ACCESS_2X16: bool = true;

    fn enable() {
        cortex_m::interrupt::free(|_| unsafe {
            USB::enable_unchecked();
            USB::reset_unchecked();
        });
    }

    fn startup_delay() {
        // not required
    }
}
