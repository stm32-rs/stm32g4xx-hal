//! ## Origin
//!
//! This code has been taken from the stm32g0xx-hal project and modified slightly to support
//! STM32G4xx MCUs.

//#![deny(warnings)]
#![deny(unsafe_code)]
#![no_main]
#![no_std]

mod utils;
use utils::logger::info;
extern crate cortex_m_rt as rt;

use fugit::ExtU32 as _;
use hal::{
    adc::AdcClaim as _,
    comparator::{ComparatorExt, ComparatorSplit, Config},
    delay::SYSTDelayExt as _,
    gpio::GpioExt,
    rcc::RccExt,
    stm32,
};
use proto_hal::stasis::Freeze;
use rt::entry;
use stm32g4xx_hal::{self as hal, adc::config::SampleTime, delay::DelayExt as _};

#[entry]
fn main() -> ! {
    let cp = cortex_m::Peripherals::take().unwrap();
    let dp = stm32::Peripherals::take().unwrap();
    let mut rcc = dp.RCC.constrain();

    let gpioa = dp.GPIOA.split(&mut rcc);

    let (comp1, ..) = dp.COMP.split(&mut rcc);

    let (pa1, [pa1_token]) = gpioa // <- The pin to keep track of
        .pa1
        .into_analog()
        .freeze();
    let pa0 = gpioa.pa0.into_analog(); // <- Reference voltage

    // Only pa1_token and pa0 consumed here
    let comp1 = comp1.comparator(pa1_token, pa0, Config::default(), &rcc.clocks);
    let _comp1 = comp1.enable(); // <-- TODO: Do things with comparator

    let mut delay = cp.SYST.delay(&rcc.clocks);
    let mut adc = dp.ADC1.claim_and_configure(
        stm32g4xx_hal::adc::ClockSource::SystemClock,
        &rcc,
        stm32g4xx_hal::adc::config::AdcConfig::default(),
        &mut delay,
        false,
    );

    // Can not reconfigure pa1 here
    loop {
        // Can still use pa1 here
        let sample = adc.convert(&pa1, SampleTime::Cycles_640_5);
        info!("Reading: {}", sample);
        delay.delay(1000.millis());
    }
}
