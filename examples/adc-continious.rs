#![no_main]
#![no_std]

use crate::hal::{
    adc::{
        config::{Continuous, SampleTime, Sequence},
        AdcClaim, ClockSource, Temperature, Vref,
    },
    delay::SYSTDelayExt,
    gpio::GpioExt,
    pwr::PwrExt,
    rcc::{Config, RccExt},
    stm32::Peripherals,
};
use stm32g4xx_hal as hal;

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
    let mut adc = dp
        .ADC1
        .claim(ClockSource::SystemClock, &rcc, &mut delay, true);

    adc.enable_temperature(&dp.ADC12_COMMON);
    adc.enable_vref(&dp.ADC12_COMMON);
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
        let millivolts = Vref::sample_to_millivolts(adc.current_sample());
        info!("vref: {}mV", millivolts);

        adc = adc.wait_for_conversion_sequence().unwrap_active();
        let millivolts = Temperature::temperature_to_degrees_centigrade(adc.current_sample());
        info!("temp: {}℃C", millivolts); // Note: Temperature seems quite low...
    }
}
