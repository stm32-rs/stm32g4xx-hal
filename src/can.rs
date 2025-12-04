//! # Controller Area Network (CAN) Interface
//!

use crate::gpio::alt::CanCommon;
use crate::rcc::{self, Rcc};

pub trait Instance: CanCommon + rcc::Instance + crate::Ptr {}

/// Storage type for the CAN controller
#[derive(Debug)]
pub struct Can<CAN: Instance> {
    rb: CAN,
}
#[allow(dead_code)]
impl<CAN: Instance> Can<CAN> {
    /// Returns a reference to the inner peripheral
    fn inner(&self) -> &CAN {
        &self.rb
    }
}

/// Extension trait for CAN controller
pub trait CanExt: Sized + Instance
where
    Can<Self>: fdcan::Instance,
{
    fn fdcan(
        self,
        pins: (impl Into<Self::Tx>, impl Into<Self::Rx>),
        rcc: &mut Rcc,
    ) -> fdcan::FdCan<Can<Self>, fdcan::ConfigMode> {
        Self::enable(rcc);
        let _pins = (pins.0.into(), pins.1.into());

        self.fdcan_unchecked()
    }

    fn fdcan_unchecked(self) -> fdcan::FdCan<Can<Self>, fdcan::ConfigMode>;
}

impl<CAN: Instance> Can<CAN>
where
    Self: fdcan::message_ram::Instance,
{
    pub fn new(rb: CAN) -> fdcan::FdCan<Self, fdcan::ConfigMode> {
        fdcan::FdCan::new(Self { rb }).into_config_mode()
    }
}

unsafe impl<CAN: Instance> fdcan::Instance for Can<CAN>
where
    Self: fdcan::message_ram::Instance,
{
    const REGISTERS: *mut fdcan::RegisterBlock = CAN::PTR as *mut _;
}

impl<CAN: Instance> CanExt for CAN
where
    Can<Self>: fdcan::message_ram::Instance,
{
    fn fdcan_unchecked(self) -> fdcan::FdCan<Can<Self>, fdcan::ConfigMode> {
        Can::new(self)
    }
}

mod fdcan1 {
    use super::{Can, Instance};
    use crate::stm32::FDCAN1;
    use fdcan;

    impl Instance for FDCAN1 {}

    unsafe impl fdcan::message_ram::Instance for Can<FDCAN1> {
        const MSG_RAM: *mut fdcan::message_ram::RegisterBlock = (0x4000_a400 as *mut _);
    }
}

#[cfg(feature = "fdcan2")]
mod fdcan2 {
    use super::{Can, Instance};
    use crate::stm32::FDCAN2;
    use fdcan;
    use fdcan::message_ram;

    impl Instance for FDCAN2 {}

    unsafe impl fdcan::message_ram::Instance for Can<FDCAN2> {
        const MSG_RAM: *mut message_ram::RegisterBlock = (0x4000_a750 as *mut _);
    }
}

#[cfg(feature = "fdcan3")]
mod fdcan3 {
    use super::{Can, Instance};
    use crate::stm32::FDCAN3;
    use fdcan;
    use fdcan::message_ram;

    impl Instance for FDCAN3 {}

    unsafe impl fdcan::message_ram::Instance for Can<FDCAN3> {
        const MSG_RAM: *mut message_ram::RegisterBlock = (0x4000_aaa0 as *mut _);
    }
}
