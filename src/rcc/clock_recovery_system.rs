use crate::stm32::CRS;

use super::Rcc;

use crate::rcc::Enable;

pub struct CrsConfig {}

pub struct Crs {
    rb: CRS,
}

impl Crs {
    /// Sets up the clock recovery system for the HSI48 oscillator.
    /// TODO: make this configurable for more than just USB applications.
    pub fn configure(self, crs_config: CrsConfig, rcc: &Rcc) -> Self {
        // TODO: This needs to ensure that the HSI48 is enabled
        // and then setup the CRS. For now this just needs to use
        // the USB sync as the trigger system.

        rcc.enable_hsi48();

        // Enable the clock recovery system
        CRS::enable(&rcc.rb);

        // Set to b10 for USB SOF as source
        self.rb
            .cfgr
            .modify(|_, w| unsafe { w.syncsrc().bits(0b10) });

        self.rb.cr.modify(|_, w| {
            // Set autotrim enabled.
            w.autotrimen().set_bit();
            // Enable CRS
            w.cen().set_bit()
        });

        self
    }
}

pub trait CrsExt {
    /// Constrains the `CRS` peripheral so that it can only be setup once.
    fn constrain(self) -> Crs;
}

impl CrsExt for CRS {
    fn constrain(self) -> Crs {
        Crs { rb: self }
    }
}
