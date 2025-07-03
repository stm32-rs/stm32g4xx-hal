//#![deny(warnings)]
#![deny(unsafe_code)]
#![no_main]
#![no_std]
#![allow(clippy::uninlined_format_args)]

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
use rt::entry;
use stm32g4xx_hal::{
    self as hal,
    adc::{config::SampleTime, AdcCommonExt},
    delay::DelayExt as _,
    stasis::Freeze,
};

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
    let adc12_common = dp.ADC12_COMMON.claim(Default::default(), &mut rcc);
    let mut adc = adc12_common.claim_and_configure(
        dp.ADC1,
        stm32g4xx_hal::adc::config::AdcConfig::default(),
        &mut delay,
    );

    // Can not reconfigure pa1 here
    loop {
        // Can still use pa1 here
        let sample = adc.convert(&pa1, SampleTime::Cycles_640_5);
        info!("Reading: {}", sample);
        delay.delay(1000.millis());
    }
}
