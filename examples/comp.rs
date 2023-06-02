//! ## Origin
//!
//! This code has been taken from the stm32g0xx-hal project and modified slightly to support
//! STM32G4xx MCUs.

//#![deny(warnings)]
#![deny(unsafe_code)]
#![no_main]
#![no_std]

use hal::comparator::{ComparatorExt, ComparatorSplit, Config, Hysteresis, RefintInput};
use hal::gpio::GpioExt;
use hal::prelude::OutputPin;
use hal::rcc::RccExt;
use stm32g4xx_hal as hal;
mod utils;
extern crate cortex_m_rt as rt;

use hal::stm32;
use rt::entry;

#[entry]
fn main() -> ! {
    let dp = stm32::Peripherals::take().expect("cannot take peripherals");
    let mut rcc = dp.RCC.constrain();

    let gpioa = dp.GPIOA.split(&mut rcc);

    let (comp1, comp2, ..) = dp.COMP.split(&mut rcc);

    let pa1 = gpioa.pa1.into_analog();
    let pa0 = gpioa.pa0.into_analog();
    let comp1 = comp1.comparator(pa1, pa0, Config::default(), &rcc.clocks);
    let comp1 = comp1.enable();

    // led1 pa1 will be updated manually when to match comp1 value
    let mut led1 = gpioa.pa5.into_push_pull_output();

    let pa7 = gpioa.pa7.into_analog();
    let comp2 = comp2.comparator(
        pa7,
        RefintInput::VRefintM12,
        Config::default()
            .hysteresis(Hysteresis::None)
            .output_inverted(),
        &rcc.clocks,
    );
    let led2 = gpioa.pa12.into_push_pull_output();
    // Configure PA12 to the comparator's alternate function so it gets
    // changed directly by the comparator.
    comp2.output_pin(led2);
    let _comp2 = comp2.enable().lock();

    loop {
        // Read comp1 output and update led1 accordingly
        match comp1.output() {
            true => led1.set_high().unwrap(),
            false => led1.set_low().unwrap(),
        }
    }
}
