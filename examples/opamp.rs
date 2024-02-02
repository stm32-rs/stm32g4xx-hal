//! Integrated opamps.

#![no_std]
#![no_main]

use stm32g4xx_hal as hal;

use hal::delay::DelayFromCountDownTimer;
use hal::timer::Timer;
use hal::time::ExtU32;
use hal::adc::AdcClaim;
use hal::adc::ClockSource;
use hal::gpio::gpioa::*;
use hal::gpio::Analog;
use hal::opamp::opamp1::IntoPga as _;
use hal::opamp::opamp2::IntoPga as _;
use hal::opamp::NonInvertingGain;
use hal::opamp::PgaModeInternal;
use hal::prelude::*;
use hal::pwr::PwrExt;

use utils::logger::info;

#[macro_use]
mod utils;

#[cortex_m_rt::entry]
fn main() -> ! {
    utils::logger::init();

    // take peripherals
    let dp = stm32g4xx_hal::stm32::Peripherals::take().unwrap();
    // let cp = cortex_m::Peripherals::take().expect("cannot take core peripherals");

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
    let opamp1 = opamp1.follower(gpioa.pa1, Some(gpioa.pa2));
    let opamp2 = opamp2.follower(gpioa.pa7, Option::<PA6<Analog>>::None);

    // Set up opamp1 and opamp2 in open loop mode
    let opamp3 = opamp3.open_loop(gpiob.pb0, gpiob.pb2, Some(gpiob.pb1));

    // disable opamps
    let (opamp1, pa1, some_pa2) = opamp1.disable();
    let (opamp2, pa7, _none) = opamp2.disable();

    let (_opamp3, _pb0, _pb2, _some_pb1) = opamp3.disable();

    // Configure opamp1 with pa1 as non-inverting input and set gain to x2
    let _opamp1 = opamp1.pga(
        pa1,
        PgaModeInternal::gain(NonInvertingGain::Gain2),
        some_pa2, // Route output to pin pa2
    );

    // Configure op with pa7 as non-inverting input and set gain to x4
    let opamp2 = opamp2.pga(
        pa7,
        PgaModeInternal::gain(NonInvertingGain::Gain4),
        Option::<PA6<Analog>>::None, // Do not route output to any external pin, use internal AD instead
    );

    // let mut delay = cp.SYST.delay(&rcc.clocks);
    let mut delay = DelayFromCountDownTimer::new(
        Timer::new(dp.TIM6, &rcc.clocks).start_count_down(100u32.millis()),
    );
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
        let (_opamp1, _pa1, _mode) = _opamp1.disable();
        let (_opamp2, _pa7, _mode) = opamp2.disable();

        loop {
            delay.delay_ms(100);
        }
    }
}
