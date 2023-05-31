// #![deny(warnings)]
#![deny(unsafe_code)]
#![no_main]
#![no_std]

use embedded_hal::Direction;
use hal::comparator::{self, ComparatorExt, ComparatorSplit};
use hal::dac::{Dac1IntSig1, DacExt, DacOut};
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
    let dac1ch1 = dp.DAC1.constrain((gpioa.pa4, Dac1IntSig1), &mut rcc);

    let mut dac = dac1ch1.calibrate_buffer(&mut delay).enable();

    let (comp1, _comp2, ..) = dp.COMP.split(&mut rcc);
    let pa1 = gpioa.pa1.into_analog();
    let comp = comp1.comparator(
        pa1,
        &dac,
        comparator::Config::default().hysteresis(comparator::Hysteresis::None),
        &rcc.clocks,
    );

    let led2 = gpioa.pa0.into_push_pull_output();
    // Configure PA12 to the comparator's alternate function so it gets
    // changed directly by the comparator.
    comp.output_pin(led2);
    let _comp1 = comp.enable().lock();

    let mut dir = Direction::Upcounting;
    let mut val = 0;

    loop {
        dac.set_value(val);
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
