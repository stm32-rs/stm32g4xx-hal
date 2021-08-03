#![no_std]
#![no_main]

use cortex_m_rt::entry;

use crate::hal::{
    adc::{
        config::{Continuous, Dma as AdcDma, SampleTime, Sequence},
        AdcClaim, ClockSource, Temperature,
    },
    delay::SYSTDelayExt,
    dma::{config::DmaConfig, stream::DMAExt, TransferExt},
    gpio::GpioExt,
    rcc::{Config, RccExt},
    stm32::Peripherals,
};
use stm32g4xx_hal as hal;

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
    let mut rcc = rcc.freeze(Config::hsi());

    let mut streams = dp.DMA1.split(&rcc);
    let config = DmaConfig::default()
        .transfer_complete_interrupt(false)
        .circular_buffer(false)
        .memory_increment(true);

    info!("Setup Gpio");
    let gpioa = dp.GPIOA.split(&mut rcc);
    let pa0 = gpioa.pa0.into_analog();

    info!("Setup Adc1");
    let mut delay = cp.SYST.delay(&rcc.clocks);
    let mut adc = dp.ADC1.claim(ClockSource::SystemClock, &rcc, &mut delay);

    adc.enable_temperature(&dp.ADC12_COMMON);
    adc.set_continuous(Continuous::Single);
    adc.reset_sequence();
    adc.configure_channel(&pa0, Sequence::One, SampleTime::Cycles_640_5);
    adc.configure_channel(&Temperature, Sequence::Two, SampleTime::Cycles_640_5);

    info!("Setup DMA");
    let first_buffer = cortex_m::singleton!(: [u16; 2] = [0; 2]).unwrap();
    let mut transfer = streams.0.into_peripheral_to_memory_transfer(
        adc.enable_dma(AdcDma::Single),
        &mut first_buffer[..],
        config,
    );

    transfer.start(|adc| adc.start_conversion());

    info!("Wait for Conversion");
    while !transfer.get_transfer_complete_flag() {}
    info!("Conversion Done");

    transfer.pause(|adc| adc.cancel_conversion());
    let (s0, adc, first_buffer) = transfer.free();
    let adc = adc.disable();

    streams.0 = s0;

    let millivolts = adc.sample_to_millivolts(first_buffer[0]);
    info!("pa3: {}mV", millivolts);
    let millivolts = Temperature::temperature_to_degrees_centigrade(first_buffer[1]);
    info!("temp: {}℃C", millivolts); // Note: Temperature seems quite low...

    #[allow(clippy::empty_loop)]
    loop {}
}
