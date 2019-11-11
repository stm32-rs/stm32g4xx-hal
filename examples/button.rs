//#![deny(warnings)]
#![deny(unsafe_code)]
#![no_main]
#![no_std]

use cortex_m;
use cortex_m_rt as rt;
//extern crate panic_halt;
use panic_semihosting;
use stm32g4xx_hal as hal;

use crate::hal::prelude::*;
use crate::hal::stm32;
use rt::entry;

#[entry]
fn main() -> ! {
    let dp = stm32::Peripherals::take().expect("cannot take peripherals");
    let cp = cortex_m::Peripherals::take().expect("cannot take core peripherals");

    let mut rcc = dp.RCC.constrain();
    let mut delay = cp.SYST.delay(&rcc.clocks);

    let gpioc = dp.GPIOC.split(&mut rcc);
    let button = gpioc.pc13.into_pull_down_input();

    let gpioa = dp.GPIOA.split(&mut rcc);
    let mut led = gpioa.pa5.into_push_pull_output();

    loop {
        let wait =  match button.is_high() {
            Ok(true) => 300.ms(),
            Ok(false) => 100.ms(),
            _ => unreachable!(),
        };
        delay.delay(wait);
        led.toggle().unwrap();
    }
}
