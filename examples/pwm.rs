//This example puts the timer in PWM mode using the specified pin with a frequency of 100Hz and a duty cycle of 50%.
#![no_main]
#![no_std]

use cortex_m_rt::entry;
use hal::gpio::{AF6, PA8};
use hal::prelude::*;
use hal::stm32;
use hal::time::RateExtU32;
use stm32g4xx_hal as hal;
extern crate cortex_m_rt as rt;
use hal::prelude::SetDutyCycle;

#[macro_use]
mod utils;

#[entry]
fn main() -> ! {
    utils::logger::init();

    let dp = stm32::Peripherals::take().expect("cannot take peripherals");
    let mut rcc = dp.RCC.constrain();
    let gpioa = dp.GPIOA.split(&mut rcc);
    let pin: PA8<AF6> = gpioa.pa8.into_alternate();

    let mut pwm = dp.TIM1.pwm(pin, 100.Hz(), &mut rcc);

    let _ = pwm.set_duty_cycle_percent(50);
    pwm.enable();

    loop {
        cortex_m::asm::nop()
    }
}
