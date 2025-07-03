#![no_std]
#![no_main]
#![allow(clippy::uninlined_format_args)]

use cortex_m_rt::entry;

use crate::hal::{
    adc::{
        config::{Continuous, Dma as AdcDma, Resolution, SampleTime, Sequence},
        AdcClaim,
    },
    delay::SYSTDelayExt,
    dma::{channel::DMAExt, config::DmaConfig, TransferExt},
    gpio::GpioExt,
    pwr::PwrExt,
    rcc::{Config, RccExt},
    stm32::Peripherals,
};
use stm32g4xx_hal::{
    self as hal,
    adc::{temperature::Temperature, AdcCommonExt},
};

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

    let mut channels = dp.DMA1.split(&rcc);
    let config = DmaConfig::default()
        .transfer_complete_interrupt(false)
        .circular_buffer(false)
        .memory_increment(true);

    info!("Setup Gpio");
    let gpioa = dp.GPIOA.split(&mut rcc);
    let pa0 = gpioa.pa0.into_analog();

    info!("Setup Adc1");
    let mut delay = cp.SYST.delay(&rcc.clocks);

    let mut adc12_common = dp.ADC12_COMMON.claim(Default::default(), &mut rcc);
    let mut adc = adc12_common.claim(dp.ADC1, &mut delay);

    adc12_common.enable_temperature();
    adc.set_continuous(Continuous::Single);
    adc.reset_sequence();
    adc.configure_channel(&pa0, Sequence::One, SampleTime::Cycles_640_5);
    adc.configure_channel(&Temperature, Sequence::Two, SampleTime::Cycles_640_5);

    info!("Setup DMA");
    let first_buffer = cortex_m::singleton!(: [u16; 2] = [0; 2]).unwrap();
    let mut transfer = channels.ch1.into_peripheral_to_memory_transfer(
        adc.enable_dma(AdcDma::Single),
        &mut first_buffer[..],
        config,
    );

    transfer.start(|adc| adc.start_conversion());

    info!("Wait for Conversion");
    while !transfer.get_transfer_complete_flag() {}
    info!("Conversion Done");

    transfer.pause(|adc| adc.cancel_conversion());
    let (ch1, adc, first_buffer) = transfer.free();
    let adc = adc.disable();

    channels.ch1 = ch1;

    let millivolts = adc.sample_to_millivolts(first_buffer[0]);
    info!("pa3: {}mV", millivolts);

    // Assume vdda is 3.3V, see adc-continious.rs for an example of measuring VDDA using VREF
    let temp =
        Temperature::temperature_to_degrees_centigrade(first_buffer[1], 3.3, Resolution::Twelve);
    info!("temp: {}°C", temp);

    #[allow(clippy::empty_loop)]
    loop {}
}
