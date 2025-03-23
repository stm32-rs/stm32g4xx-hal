use super::{marker, Pin, PinExt, SignalEdge};
use crate::exti::{Event, ExtiExt};
use crate::pac::EXTI;
use crate::syscfg::SysCfg;

/// External Interrupt Pin
pub trait ExtiPin {
    /// Make corresponding EXTI line sensitive to this pin
    fn make_interrupt_source(&mut self, syscfg: &mut SysCfg);

    /// Generate interrupt on rising edge, falling edge or both
    fn trigger_on_edge(&mut self, exti: &mut EXTI, level: SignalEdge);

    /// Enable external interrupts from this pin.
    fn enable_interrupt(&mut self, exti: &mut EXTI);

    /// Disable external interrupts from this pin
    fn disable_interrupt(&mut self, exti: &mut EXTI);

    /// Clear the interrupt pending bit for this pin
    fn clear_interrupt_pending_bit(&mut self);

    /// Reads the interrupt pending bit for this pin
    fn check_interrupt(&self) -> bool;
}

impl<const P: char, const N: u8, MODE> Pin<P, N, MODE> {
    /// Configures the pin as external trigger
    pub fn listen(self, edge: SignalEdge, exti: &mut EXTI) -> Self {
        exti.listen(Event::from_code(self.pin_id()), edge);
        self
    }
}

impl<PIN> ExtiPin for PIN
where
    PIN: PinExt,
    PIN::Mode: marker::Interruptible,
{
    #[inline(always)]
    fn make_interrupt_source(&mut self, syscfg: &mut SysCfg) {
        let i = self.pin_id();
        let port = self.port_id() as u32;
        let offset = 4 * (i % 4);
        match i {
            0..=3 => {
                syscfg.exticr1().modify(|r, w| unsafe {
                    w.bits((r.bits() & !(0xf << offset)) | (port << offset))
                });
            }
            4..=7 => {
                syscfg.exticr2().modify(|r, w| unsafe {
                    w.bits((r.bits() & !(0xf << offset)) | (port << offset))
                });
            }
            8..=11 => {
                syscfg.exticr3().modify(|r, w| unsafe {
                    w.bits((r.bits() & !(0xf << offset)) | (port << offset))
                });
            }
            12..=15 => {
                syscfg.exticr4().modify(|r, w| unsafe {
                    w.bits((r.bits() & !(0xf << offset)) | (port << offset))
                });
            }
            _ => unreachable!(),
        }
    }

    #[inline(always)]
    fn trigger_on_edge(&mut self, exti: &mut EXTI, edge: SignalEdge) {
        let i = self.pin_id();
        match edge {
            SignalEdge::Rising => {
                exti.rtsr1()
                    .modify(|r, w| unsafe { w.bits(r.bits() | (1 << i)) });
                exti.ftsr1()
                    .modify(|r, w| unsafe { w.bits(r.bits() & !(1 << i)) });
            }
            SignalEdge::Falling => {
                exti.ftsr1()
                    .modify(|r, w| unsafe { w.bits(r.bits() | (1 << i)) });
                exti.rtsr1()
                    .modify(|r, w| unsafe { w.bits(r.bits() & !(1 << i)) });
            }
            SignalEdge::RisingFalling => {
                exti.rtsr1()
                    .modify(|r, w| unsafe { w.bits(r.bits() | (1 << i)) });
                exti.ftsr1()
                    .modify(|r, w| unsafe { w.bits(r.bits() | (1 << i)) });
            }
        }
    }

    #[inline(always)]
    fn enable_interrupt(&mut self, exti: &mut EXTI) {
        exti.imr1()
            .modify(|r, w| unsafe { w.bits(r.bits() | (1 << self.pin_id())) });
    }

    #[inline(always)]
    fn disable_interrupt(&mut self, exti: &mut EXTI) {
        exti.imr1()
            .modify(|r, w| unsafe { w.bits(r.bits() & !(1 << self.pin_id())) });
    }

    #[inline(always)]
    fn clear_interrupt_pending_bit(&mut self) {
        unsafe { (*EXTI::ptr()).pr1().write(|w| w.bits(1 << self.pin_id())) };
    }

    #[inline(always)]
    fn check_interrupt(&self) -> bool {
        unsafe { ((*EXTI::ptr()).pr1().read().bits() & (1 << self.pin_id())) != 0 }
    }
}
