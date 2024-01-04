//! Integrated opamps.

#![no_std]
#![no_main]

use stm32g4xx_hal::adc::AdcClaim;
use stm32g4xx_hal::adc::ClockSource;
use stm32g4xx_hal::opamp::InternalOutput;
use stm32g4xx_hal::opamp::{
    IntoFollower, IntoOpenLoop, IntoPga, NonInvertingGain, PgaModeInternal,
};
use stm32g4xx_hal::prelude::*;
use stm32g4xx_hal::pwr::PwrExt;

use utils::logger::info;

#[macro_use]
mod utils;

#[cortex_m_rt::entry]
fn main() -> ! {
    utils::logger::init();

    // take peripherals
    let dp = stm32g4xx_hal::stm32::Peripherals::take().unwrap();
    let cp = cortex_m::Peripherals::take().expect("cannot take core peripherals");

    // setup clock and power
    let pwr = dp.PWR.constrain().freeze();
    let config = stm32g4xx_hal::rcc::Config::hsi();
    let mut rcc = dp.RCC.freeze(config, pwr);

    // split gpio
    let gpioa = dp.GPIOA.split(&mut rcc);
    let gpiob = dp.GPIOB.split(&mut rcc);

    // setup opamps
    let (opamp1, opamp2, opamp3, ..) = dp.OPAMP.split(&mut rcc);

    // Set up opamp1 and opamp2 in follower mode
    let opamp1 = opamp1.follower(gpioa.pa1, gpioa.pa2);
    let opamp2 = opamp2.follower(gpioa.pa7, InternalOutput);

    // Set up opamp1 and opamp2 in open loop mode
    let opamp3 = opamp3.open_loop(gpiob.pb0, gpiob.pb2, gpiob.pb1);

    // disable opamps
    let (opamp1, pa1, pa2) = opamp1.disable();
    let (opamp2, pa7) = opamp2.disable();

    let (_opamp3, _pb0, _pb2, _pb1) = opamp3.disable();

    // Configure opamp1 with pa1 as non-inverting input and set gain to x2
    let opamp1 = opamp1.pga(
        pa1,
        PgaModeInternal::gain(NonInvertingGain::Gain2),
        pa2, // Route output to pin pa2
    );

    // Lock opamp1. After the opamp is locked the register cannot be written
    // until the device is reset (even if using unsafe register accesses).
    let _opamp1 = opamp1.lock();

    // Configure op with pa7 as non-inverting input and set gain to x4
    let opamp2 = opamp2.pga(
        pa7,
        PgaModeInternal::gain(NonInvertingGain::Gain4),
        InternalOutput, // Do not route output to any external pin, use internal AD instead
    );

    let mut delay = cp.SYST.delay(&rcc.clocks);
    let mut adc = dp
        .ADC2
        .claim(ClockSource::SystemClock, &rcc, &mut delay, true);

    loop {
        // Here we can sample the output of opamp2 as if it was a regular AD pin
        let sample = adc.convert(
            &opamp2,
            stm32g4xx_hal::adc::config::SampleTime::Cycles_640_5,
        );

        let millivolts = adc.sample_to_millivolts(sample);
        info!("opamp2 thus 4x pa7: {}mV", millivolts);

        delay.delay_ms(100);
    }

    #[allow(unreachable_code)]
    {
        let (_opamp2, _mode) = opamp2.disable();

        loop {
            delay.delay_ms(100);
        }
    }
}
