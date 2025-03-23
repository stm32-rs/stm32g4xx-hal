//! ## Origin
//!
//! This code has been taken from the stm32g0xx-hal project and modified slightly to support
//! STM32G4xx MCUs.

//#![deny(warnings)]
#![deny(unsafe_code)]
#![no_main]
#![no_std]

mod utils;

use cortex_m_rt::entry;

#[entry]
fn main() -> ! {
    use stm32g4xx_hal::{
        comparator::{refint_input, ComparatorExt, ComparatorSplit, Config, Hysteresis},
        gpio::{GpioExt, PushPull},
        pac,
        rcc::RccExt,
    };

    let dp = pac::Peripherals::take().expect("cannot take peripherals");
    let mut rcc = dp.RCC.constrain();

    let gpioa = dp.GPIOA.split(&mut rcc);

    let (comp1, comp2, ..) = dp.COMP.split(&mut rcc);

    let comp1 = comp1.comparator(gpioa.pa1, gpioa.pa0, Config::default(), &rcc.clocks);
    let comp1 = comp1.enable();

    // led1 pa1 will be updated manually when to match comp1 value
    let mut led1 = gpioa.pa5.into_push_pull_output();

    let comp2 = comp2.comparator(
        gpioa.pa7,
        refint_input::VRefintM12,
        Config::default()
            .hysteresis(Hysteresis::None)
            .output_inverted(),
        &rcc.clocks,
    );
    let led2 = gpioa.pa12;
    // Configure PA12 to the comparator's alternate function so it gets
    // changed directly by the comparator.
    comp2.output_pin::<PushPull>(led2);
    let _comp2 = comp2.enable().lock();

    loop {
        // Read comp1 output and update led1 accordingly
        led1.set_state(comp1.output().into());
    }
}
