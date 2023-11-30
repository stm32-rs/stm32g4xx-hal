//! # Controller Area Network (CAN) Interface
//!

use crate::rcc::{self, Rcc};
use fdcan;

mod sealed {
    /// A TX pin configured for CAN communication
    pub trait Tx<CAN> {}
    /// An RX pin configured for CAN communication
    pub trait Rx<CAN> {}
}

/// Select an FDCAN Clock Source
#[allow(clippy::upper_case_acronyms)]
#[allow(dead_code)]
enum FdCanClockSource {
    /// Select HSE as the FDCAN clock source
    HSE = 0b00,
    /// Select PLL "Q" clock as the FDCAN clock source
    PLLQ = 0b01,
    /// Select "P" clock as the FDCAN clock source
    PCLK = 0b10,
    //Reserved = 0b10,
}

/// Storage type for the CAN controller
#[derive(Debug)]
pub struct Can<FDCAN> {
    rb: FDCAN,
}
#[allow(dead_code)]
impl<FDCAN> Can<FDCAN> {
    /// Returns a reference to the inner peripheral
    fn inner(&self) -> &FDCAN {
        &self.rb
    }
}

/// Extension trait for CAN controller
pub trait CanExt: Sized
where
    Self: rcc::Instance,
    Can<Self>: fdcan::Instance,
{
    fn fdcan<TX, RX>(
        self,
        _tx: TX,
        _rx: RX,
        rcc: &Rcc,
    ) -> fdcan::FdCan<Can<Self>, fdcan::ConfigMode>
    where
        TX: sealed::Tx<Self>,
        RX: sealed::Rx<Self>,
    {
        Self::enable(&rcc.rb);

        if rcc.rb.ccipr.read().fdcansel().is_hse() {
            // Select P clock as FDCAN clock source
            rcc.rb.ccipr.modify(|_, w| {
                // This is sound, as `FdCanClockSource` only contains valid values for this field.
                unsafe {
                    w.fdcansel().bits(FdCanClockSource::PCLK as u8);
                }

                w
            });
        }

        self.fdcan_unchecked()
    }

    fn fdcan_unchecked(self) -> fdcan::FdCan<Can<Self>, fdcan::ConfigMode>;
}
/// Implements sealed::{Tx,Rx} for pins associated with a CAN peripheral
macro_rules! pins {
    ($PER:ident =>
        (tx: [ $($( #[ $pmetatx:meta ] )* $tx:ident<$txaf:ident>),+ $(,)? ],
         rx: [ $($( #[ $pmetarx:meta ] )* $rx:ident<$rxaf:ident>),+ $(,)? ])) => {
        $(
            $( #[ $pmetatx ] )*
            impl sealed::Tx<$PER> for $tx<crate::gpio::Alternate<$txaf>> {}
        )+
        $(
            $( #[ $pmetarx ] )*
            impl sealed::Rx<$PER> for $rx<crate::gpio::Alternate<$rxaf>> {}
        )+
    };
}

mod fdcan1 {
    use super::sealed;
    use super::{Can, CanExt};
    use crate::gpio::{
        gpioa::{PA11, PA12},
        gpiob::{PB8, PB9},
        gpiod::{PD0, PD1},
        AF9,
    };
    use crate::stm32::FDCAN1;
    use fdcan;

    // All STM32G4 models with CAN support these pins
    pins! {
        FDCAN1 => (
            tx: [
                PA12<AF9>,
                PB9<AF9>,
                PD1<AF9>,
            ],
            rx: [
                PA11<AF9>,
                PB8<AF9>,
                PD0<AF9>,
            ]
        )
    }

    impl Can<FDCAN1> {
        pub fn fdcan1(rb: FDCAN1) -> fdcan::FdCan<Self, fdcan::ConfigMode> {
            fdcan::FdCan::new(Self { rb }).into_config_mode()
        }
    }
    impl CanExt for FDCAN1 {
        fn fdcan_unchecked(self) -> fdcan::FdCan<Can<Self>, fdcan::ConfigMode> {
            Can::fdcan1(self)
        }
    }
    unsafe impl fdcan::Instance for Can<FDCAN1> {
        const REGISTERS: *mut fdcan::RegisterBlock = FDCAN1::ptr() as *mut _;
    }
    unsafe impl fdcan::message_ram::Instance for Can<FDCAN1> {
        const MSG_RAM: *mut fdcan::message_ram::RegisterBlock = (0x4000_a400 as *mut _);
    }
}

#[cfg(any(
    feature = "stm32g471",
    feature = "stm32g473",
    feature = "stm32g474",
    feature = "stm32g483",
    feature = "stm32g484",
    feature = "stm32g491",
    feature = "stm32g4A1",
))]
mod fdcan2 {
    use super::sealed;
    use super::{Can, CanExt};
    use crate::gpio::{
        gpiob::{PB12, PB13, PB5, PB6},
        AF9,
    };
    use crate::stm32::FDCAN2;
    use fdcan;
    use fdcan::message_ram;

    pins! {
        FDCAN2 => (
            tx: [
                PB6<AF9>,
                PB13<AF9>,
            ],
            rx: [
                PB5<AF9>,
                PB12<AF9>,
        ])
    }

    impl Can<FDCAN2> {
        pub fn fdcan2(rb: FDCAN2) -> fdcan::FdCan<Self, fdcan::ConfigMode> {
            fdcan::FdCan::new(Self { rb }).into_config_mode()
        }
    }
    impl CanExt for FDCAN2 {
        fn fdcan_unchecked(self) -> fdcan::FdCan<Can<Self>, fdcan::ConfigMode> {
            Can::fdcan2(self)
        }
    }
    unsafe impl fdcan::Instance for Can<FDCAN2> {
        const REGISTERS: *mut fdcan::RegisterBlock = FDCAN2::ptr() as *mut _;
    }
    unsafe impl fdcan::message_ram::Instance for Can<FDCAN2> {
        const MSG_RAM: *mut message_ram::RegisterBlock = (0x4000_a750 as *mut _);
    }
}

#[cfg(any(
    feature = "stm32g473",
    feature = "stm32g474",
    feature = "stm32g483",
    feature = "stm32g484",
))]
mod fdcan3 {
    use super::sealed;
    use super::{Can, CanExt};
    use crate::gpio::{
        gpioa::{PA15, PA8},
        gpiob::{PB3, PB4},
        AF11,
    };
    use crate::stm32::FDCAN3;
    use fdcan;
    use fdcan::message_ram;

    pins! {
        FDCAN3 => (
            tx: [
                PA15<AF11>,
                PB4<AF11>,
            ],
            rx: [
                PA8<AF11>,
                PB3<AF11>,
        ])
    }

    impl Can<FDCAN3> {
        pub fn fdcan3(rb: FDCAN3) -> fdcan::FdCan<Self, fdcan::ConfigMode> {
            fdcan::FdCan::new(Self { rb }).into_config_mode()
        }
    }
    impl CanExt for FDCAN3 {
        fn fdcan_unchecked(self) -> fdcan::FdCan<Can<Self>, fdcan::ConfigMode> {
            Can::fdcan3(self)
        }
    }
    unsafe impl fdcan::Instance for Can<FDCAN3> {
        const REGISTERS: *mut fdcan::RegisterBlock = FDCAN3::ptr() as *mut _;
    }
    unsafe impl fdcan::message_ram::Instance for Can<FDCAN3> {
        const MSG_RAM: *mut message_ram::RegisterBlock = (0x4000_aaa0 as *mut _);
    }
}
