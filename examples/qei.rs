#![deny(warnings)]
#![deny(unsafe_code)]
#![no_main]
#![no_std]

extern crate cortex_m;
extern crate cortex_m_rt as rt;
extern crate panic_halt;
extern crate stm32g4xx_hal as hal;

use cortex_m_semihosting::hprintln;
use hal::prelude::*;
use hal::stm32;
use rt::entry;

#[entry]
fn main() -> ! {
    let dp = stm32::Peripherals::take().expect("cannot take peripherals");
    
    let mut rcc = dp.RCC.constrain();
    let gpioa = dp.GPIOA.split(&mut rcc);
    let switch = gpioa.pa2.into_pull_up_input();
    let qei = dp.TIM2.qei((gpioa.pa0, gpioa.pa1), &mut rcc);

    loop {
        if switch.is_low().unwrap() {
            hprintln!("{:?}", qei.count()).unwrap();
        }
    }
}
