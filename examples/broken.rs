//This example puts the timer in PWM mode using the specified pin with a frequency of 100Hz and a duty cycle of 50%.
#![no_main]
#![no_std]

use cortex_m_rt::entry;
use utils::logger::println;
use hal::gpio::AF6;
use hal::prelude::*;
use hal::stm32;
use stm32g4xx_hal as hal;
extern crate cortex_m_rt as rt;

#[macro_use]
mod utils;

#[entry]
fn main() -> ! {
    let dp = stm32::Peripherals::take().expect("cannot take peripherals");
    let mut rcc = dp.RCC.constrain();

    let gpioa = dp.GPIOA.split(&mut rcc);
    let _pin: stm32g4xx_hal::gpio::gpioa::PA8<stm32g4xx_hal::gpio::Alternate<AF6>> =
        gpioa.pa8.into_alternate();
    let mut pin2 = gpioa.pa9.into_pull_up_input();
    defmt::println!("hej");
    pin2.is_high().unwrap(); // <--- Booom

    loop {}
}
