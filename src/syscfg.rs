use crate::rcc::Rcc;
use crate::stm32::SYSCFG;
use core::ops::Deref;

/// Extension trait that constrains the `SYSCFG` peripheral
pub trait SysCfgExt {
    /// Constrains the `SYSCFG` peripheral so it plays nicely with the other abstractions
    fn constrain(self, rcc: &mut Rcc) -> SysCfg;
}

impl SysCfgExt for SYSCFG {
    fn constrain(self, rcc: &mut Rcc) -> SysCfg {
        // Enable SYSCFG peripheral clock in APB2ENR register
        rcc.apb2enr().modify(|_, w| w.syscfgen().set_bit());

        SysCfg(self)
    }
}

pub struct SysCfg(SYSCFG);

impl Deref for SysCfg {
    type Target = SYSCFG;

    #[inline(always)]
    fn deref(&self) -> &Self::Target {
        &self.0
    }
}
