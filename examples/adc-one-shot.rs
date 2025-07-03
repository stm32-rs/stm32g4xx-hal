#![no_main]
#![no_std]
#![allow(clippy::uninlined_format_args)]

use crate::hal::{
    adc::{config::SampleTime, AdcClaim},
    pwr::PwrExt,
    rcc::Config,
    stm32::Peripherals,
};
use hal::prelude::*;
use stm32g4xx_hal::{self as hal, adc::AdcCommonExt};

use cortex_m_rt::entry;

#[macro_use]
mod utils;
use utils::logger::info;

#[entry]
fn main() -> ! {
    utils::logger::init();

    info!("start");

    let dp = Peripherals::take().unwrap();
    let cp = cortex_m::Peripherals::take().expect("cannot take core peripherals");

    info!("rcc");

    let rcc = dp.RCC.constrain();
    let pwr = dp.PWR.constrain().freeze();
    let mut rcc = rcc.freeze(Config::hsi(), pwr);

    info!("Setup Adc1");
    let mut delay = cp.SYST.delay(&rcc.clocks);
    let adc12_common = dp.ADC12_COMMON.claim(Default::default(), &mut rcc);
    let mut adc = adc12_common.claim_and_configure(
        dp.ADC2,
        stm32g4xx_hal::adc::config::AdcConfig::default(),
        &mut delay,
    );

    info!("Setup Gpio");

    let gpioa = dp.GPIOA.split(&mut rcc);
    let pa7 = gpioa.pa7.into_analog();

    info!("Enter Loop");

    loop {
        info!("Convert");
        let sample = adc.convert(&pa7, SampleTime::Cycles_640_5);
        info!("sample to mv");
        let millivolts = adc.sample_to_millivolts(sample);
        info!("pa7: {}mV", millivolts);
    }
}
