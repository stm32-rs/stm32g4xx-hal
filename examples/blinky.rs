#![deny(warnings)]
#![deny(unsafe_code)]
#![no_main]
#![no_std]

use hal::prelude::*;
use hal::stm32;
use stm32g4xx_hal as hal;

use cortex_m_rt::entry;
use log::info;

#[macro_use]
mod utils;

#[entry]
fn main() -> ! {
    utils::logger::init();

    info!("start");
    let dp = stm32::Peripherals::take().expect("cannot take peripherals");
    let mut rcc = dp.RCC.constrain();

    info!("Init Led");
    let gpioa = dp.GPIOA.split(&mut rcc);
    let mut led = gpioa.pa5.into_push_pull_output();

    loop {
        info!("Set Led low");
        for _ in 0..100_000 {
            led.set_low().unwrap();
        }
        info!("Set Led High");
        for _ in 0..100_000 {
            led.set_high().unwrap();
        }
    }
}
