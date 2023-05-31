//! ## Origin
//!
//! This code has been taken from the stm32g0xx-hal project and modified to support
//! STM32G4xx MCUs.

// #![deny(warnings)]
#![deny(unsafe_code)]
#![no_main]
#![no_std]

use embedded_hal::Direction;
use hal::dac::{DacExt, DacOut, GeneratorConfig};
use hal::delay::SYSTDelayExt;
use hal::gpio::GpioExt;
use hal::rcc::RccExt;
use stm32g4xx_hal as hal;
mod utils;
extern crate cortex_m_rt as rt;

use hal::stm32;
use rt::entry;

#[entry]
fn main() -> ! {
    let dp = stm32::Peripherals::take().expect("cannot take peripherals");
    let cp = cortex_m::Peripherals::take().expect("cannot take core peripherals");

    let mut rcc = dp.RCC.constrain();
    let mut delay = cp.SYST.delay(&rcc.clocks);

    let gpioa = dp.GPIOA.split(&mut rcc);
    let (dac1ch1, dac1ch2) = dp.DAC1.constrain((gpioa.pa4, gpioa.pa5), &mut rcc);

    let mut dac_manual = dac1ch1.calibrate_buffer(&mut delay).enable();
    let mut dac_generator = dac1ch2.enable_generator(GeneratorConfig::noise(11));

    let mut dir = Direction::Upcounting;
    let mut val = 0;

    loop {
        dac_generator.trigger();
        dac_manual.set_value(val);
        match val {
            0 => dir = Direction::Upcounting,
            4095 => dir = Direction::Downcounting,
            _ => (),
        };

        match dir {
            Direction::Upcounting => val += 1,
            Direction::Downcounting => val -= 1,
        }
    }
}
