#![no_main]
#![no_std]
#![allow(clippy::uninlined_format_args)]

use crate::hal::{
    adc::{
        config::{Continuous, Resolution, SampleTime, Sequence},
        AdcClaim, Vref,
    },
    delay::SYSTDelayExt,
    gpio::GpioExt,
    pwr::PwrExt,
    rcc::{Config, RccExt},
    signature::{VrefCal, VDDA_CALIB},
    stm32::Peripherals,
};
use stm32g4xx_hal::{
    self as hal,
    adc::{temperature::Temperature, AdcCommonExt},
};

use cortex_m_rt::entry;

use utils::logger::info;

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

    info!("Setup Gpio");
    let gpioa = dp.GPIOA.split(&mut rcc);
    let pa0 = gpioa.pa0.into_analog();

    info!("Setup Adc1");
    let mut delay = cp.SYST.delay(&rcc.clocks);
    let mut adc12_common = dp.ADC12_COMMON.claim(Default::default(), &mut rcc);
    let mut adc = adc12_common.claim(dp.ADC1, &mut delay);

    adc12_common.enable_temperature();
    adc12_common.enable_vref();
    adc.set_auto_delay(true);
    adc.set_continuous(Continuous::Continuous);
    adc.reset_sequence();
    adc.configure_channel(&pa0, Sequence::One, SampleTime::Cycles_640_5);
    adc.configure_channel(&Vref, Sequence::Two, SampleTime::Cycles_640_5);
    adc.configure_channel(&Temperature, Sequence::Three, SampleTime::Cycles_640_5);
    let adc = adc.enable();

    info!("Enter Loop");
    let mut adc = adc.start_conversion();

    loop {
        adc = adc.wait_for_conversion_sequence().unwrap_active();
        let millivolts = adc.sample_to_millivolts(adc.current_sample());
        info!("pa3: {}mV", millivolts);

        adc = adc.wait_for_conversion_sequence().unwrap_active();
        let vref_sample = adc.current_sample();
        let millivolts = Vref::sample_to_millivolts(vref_sample);
        let vdda = VDDA_CALIB * VrefCal::get().read() as u32 / vref_sample as u32;
        info!("vref: {}mV", millivolts);

        adc = adc.wait_for_conversion_sequence().unwrap_active();
        let temp = Temperature::temperature_to_degrees_centigrade(
            adc.current_sample(),
            vdda as f32 / 1000.,
            Resolution::Twelve,
        );
        info!("temp: {}°C", temp);
    }
}
