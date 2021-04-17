#![deny(warnings)]
#![deny(unsafe_code)]
#![no_main]
#![no_std]

extern crate cortex_m;
extern crate cortex_m_rt as rt;
extern crate panic_halt;
extern crate stm32g4xx_hal as hal;

use hal::prelude::*;
use hal::rcc::Config;
use hal::stm32;
use rt::entry;

#[entry]
fn main() -> ! {
    let dp = stm32::Peripherals::take().expect("cannot take peripherals");
    let cp = cortex_m::Peripherals::take().expect("cannot take core peripherals");
    let mut rcc = dp.RCC.freeze(Config::hsi());

    let gpioa = dp.GPIOA.split(&mut rcc);
    let mut led = gpioa.pa5.into_push_pull_output();

    let mut delay = cp.SYST.delay(&rcc.clocks);

    loop {
        led.toggle().unwrap();
        delay.delay(500.ms());
    }
}
