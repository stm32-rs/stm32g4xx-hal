#![no_main]
#![no_std]

use crate::hal::{
    adc::{config::SampleTime, AdcClaim},
    delay::SYSTDelayExt,
    pwr::PwrExt,
    rcc::Config,
    stm32::Peripherals,
};
use hal::prelude::*;
use stm32g4xx_hal as hal;

use cortex_m_rt::entry;

use log::info;

#[macro_use]
mod utils;

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
    let mut adc = dp.ADC2.claim_and_configure(
        stm32g4xx_hal::adc::ClockSource::SystemClock,
        &rcc,
        stm32g4xx_hal::adc::config::AdcConfig::default(),
        &mut delay,
        false,
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
