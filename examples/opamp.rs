//! Integrated opamps.

#![no_std]
#![no_main]

use stm32g4xx_hal::adc::AdcClaim;
use stm32g4xx_hal::adc::ClockSource;
use stm32g4xx_hal::gpio::gpioa::*;
use stm32g4xx_hal::gpio::gpiob::*;
use stm32g4xx_hal::gpio::Analog;
use stm32g4xx_hal::opamp::opamp1::IntoPga as _;
use stm32g4xx_hal::opamp::opamp2::IntoPga as _;
use stm32g4xx_hal::opamp::NonInvertingGain;
use stm32g4xx_hal::opamp::PgaModeInternal;
use stm32g4xx_hal::prelude::*;

use utils::logger::info;

#[macro_use]
mod utils;

#[cortex_m_rt::entry]
fn main() -> ! {
    utils::logger::init();

    // take peripherals
    let dp = stm32g4xx_hal::stm32::Peripherals::take().unwrap();
    let cp = cortex_m::Peripherals::take().expect("cannot take core peripherals");

    // setup clock
    let config = stm32g4xx_hal::rcc::Config::hsi();
    let mut rcc = dp.RCC.freeze(config);

    // split gpio
    let gpioa = dp.GPIOA.split(&mut rcc);
    let gpiob = dp.GPIOB.split(&mut rcc);

    // setup opamps
    let (opamp1, opamp2, opamp3, opamp4, ..) = dp.OPAMP.split(&mut rcc);

    let opamp1 = opamp1.follower(gpioa.pa1, Some(gpioa.pa2));
    let opamp2 = opamp2.follower(gpioa.pa7, Option::<PA6<Analog>>::None);

    let opamp3 = opamp3.open_loop(gpiob.pb0, gpiob.pb2, Some(gpiob.pb1));
    let opamp4 = opamp4.open_loop(gpiob.pb11, gpiob.pb10, Option::<PB12<Analog>>::None);

    // disable opamps
    let (opamp1, pa1, some_pa2) = opamp1.disable();
    let (opamp2, pa7, _none) = opamp2.disable();

    let (_opamp3, _pb0, _pb2, _some_pb1) = opamp3.disable();
    let (_opamp4, _pb11, _pb10, _none) = opamp4.disable();

    let _opamp1 = opamp1.pga(
        pa1,
        PgaModeInternal::gain(NonInvertingGain::Gain2),
        some_pa2,
    );
    let mut opamp2 = opamp2.pga(
        pa7,
        PgaModeInternal::gain(NonInvertingGain::Gain4),
        //Some(gpioa.pa6),
        Option::<PA6<Analog>>::None,
    );

    let mut delay = cp.SYST.delay(&rcc.clocks);
    let mut adc = dp
        .ADC2
        .claim(ClockSource::SystemClock, &rcc, &mut delay, true);

    loop {
        // Here we can sample the output of opamp2 as if it was a regular AD pin
        let sample = adc.convert(&mut opamp2, stm32g4xx_hal::adc::config::SampleTime::Cycles_640_5);

        let millivolts = adc.sample_to_millivolts(sample);
        info!("opamp2 thus 4x pa7: {}mV", millivolts);

        delay.delay_ms(100);
    }

    #[allow(unreachable_code)]
    {
        //let (_opamp1, _pa1, _mode, _some_pa2) = _opamp1.disable();
        let (_opamp2, _pa7, _mode, _none) = opamp2.disable();

        loop {}
    }
}
