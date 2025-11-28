use super::buffer::BufferLayoutConfig;
use super::Fmac;
use super::State;
use crate::stm32::FMAC;

impl<Layout: BufferLayoutConfig, S: State> Fmac<Layout, S> {
    /// Enable saturation error interrupt
    #[inline(always)]
    pub fn enable_saturation_error_interrupt(&mut self, enable: bool) {
        let fmac = unsafe { &*FMAC::ptr() };
        fmac.cr().modify(|_, w| w.satien().bit(enable));
    }

    /// Enable underflow error interrupt
    #[inline(always)]
    pub fn enable_underflow_error_interrupt(&mut self, enable: bool) {
        let fmac = unsafe { &*FMAC::ptr() };
        fmac.cr().modify(|_, w| w.unflien().bit(enable));
    }

    /// Enable overflow error interrupt
    #[inline(always)]
    pub fn enable_overflow_error_interrupt(&mut self, enable: bool) {
        let fmac = unsafe { &*FMAC::ptr() };
        fmac.cr().modify(|_, w| w.ovflien().bit(enable));
    }

    /// Enable write interrupts. The write interrupt is raised when the X1 (operand)
    /// buffer is not full, and there is space available to write to the FMAC.
    #[inline(always)]
    pub fn enable_write_interrupt(&mut self, enable: bool) {
        let fmac = unsafe { &*FMAC::ptr() };
        fmac.cr().modify(|_, w| w.wien().bit(enable));
    }

    /// Enable read interrupts. The read interrupt is raised when the Y (result)
    /// buffer is not empty, and there is data available to read from the FMAC.
    #[inline(always)]
    pub fn enable_read_interrupt(&mut self, enable: bool) {
        let fmac = unsafe { &*FMAC::ptr() };
        fmac.cr().modify(|_, w| w.rien().bit(enable));
    }
}
