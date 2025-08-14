use crate::rcc::{Enable, Rcc, Reset};
use crate::stm32::SYSCFG;
use core::ops::Deref;

/// Extension trait that constrains the `SYSCFG` peripheral
pub trait SysCfgExt {
    /// Constrains the `SYSCFG` peripheral so it plays nicely with the other abstractions
    fn constrain(self, rcc: &mut Rcc) -> SysCfg;
}

impl SysCfgExt for SYSCFG {
    fn constrain(self, rcc: &mut Rcc) -> SysCfg {
        SYSCFG::enable(rcc);
        SYSCFG::reset(rcc);

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

impl SysCfg {
    /// Enable/disable I2C fast mode plus on the I2C bus index provided as a const generic parameter.
    ///
    /// Pins that are configured as I2C alternate functions will be configured as fast mode plus.
    /// The alternate function mode of the pin must be set before FMP is enabled in SysCfg.
    ///
    /// When FM+ mode is activated on a pin, the GPIO speed configuration (OSPEEDR) is is ignored and overridden.
    ///
    pub fn i2c_fmp_enable<const BUS: u8>(&mut self, en: bool) {
        match BUS {
            1 => (*self).cfgr1().modify(|_, w| w.i2c1_fmp().bit(en)),
            2 => (*self).cfgr1().modify(|_, w| w.i2c2_fmp().bit(en)),
            3 => (*self).cfgr1().modify(|_, w| w.i2c3_fmp().bit(en)),
            4 => (*self).cfgr1().modify(|_, w| w.i2c4_fmp().bit(en)),
            _ => panic!("Invalid I2C bus"),
        };
    }
}
