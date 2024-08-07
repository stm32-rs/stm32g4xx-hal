pub use embedded_hal::{
    delay::DelayNs,
    digital::{InputPin, OutputPin, StatefulOutputPin},
    i2c::I2c,
    pwm::SetDutyCycle,
    spi::SpiBus,
};

pub use embedded_hal_old::{
    adc::OneShot as _,
    watchdog::{Watchdog as _, WatchdogEnable as _},
};

// pub use crate::analog::adc::AdcExt as _;

// #[cfg(any(feature = "stm32g07x", feature = "stm32g081"))]
// pub use crate::analog::dac::DacExt as _;
// #[cfg(any(feature = "stm32g07x", feature = "stm32g081"))]
// pub use crate::analog::dac::DacOut as _;
// #[cfg(any(feature = "stm32g07x", feature = "stm32g081"))]
// pub use crate::analog::dac::DacPin as _;
// #[cfg(any(feature = "stm32g07x", feature = "stm32g081"))]
// pub use crate::comparator::ComparatorExt as _;
// pub use crate::crc::CrcExt as _;
pub use crate::delay::DelayExt as _;
pub use crate::delay::SYSTDelayExt as _;
// pub use crate::dma::CopyDma as _;
// pub use crate::dma::ReadDma as _;
// pub use crate::dma::WriteDma as _;
pub use crate::exti::ExtiExt as _;
pub use crate::gpio::GpioExt as _;
pub use crate::i2c::I2cExt as _;
pub use crate::opamp::prelude::*;
pub use crate::opamp::OpampEx as _;
pub use crate::rcc::LSCOExt as _;
pub use crate::rcc::MCOExt as _;
pub use crate::rcc::RccExt as _;
// pub use crate::rng::RngCore as _;
// pub use crate::rng::RngExt as _;
pub use crate::serial::SerialExt as _;
pub use crate::spi::SpiExt as _;
pub use crate::time::U32Ext as _;
// pub use crate::timer::opm::OpmExt as _;
pub use crate::pwm::PwmExt as _;
// pub use crate::timer::qei::QeiExt as _;
// pub use crate::timer::stopwatch::StopwatchExt as _;
// pub use crate::timer::TimerExt as _;
// pub use crate::watchdog::IWDGExt as _;
// pub use crate::watchdog::WWDGExt as _;
