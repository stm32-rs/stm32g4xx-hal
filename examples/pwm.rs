#![deny(warnings)]
#![deny(unsafe_code)]
#![no_main]
#![no_std]

extern crate cortex_m;
extern crate cortex_m_rt as rt;
extern crate panic_halt;
extern crate stm32g4xx_hal as hal;

use cortex_m::asm;
use hal::prelude::*;
use hal::stm32;
use rt::{entry, exception, ExceptionFrame};

#[entry]
fn main() -> ! {
    let dp = stm32::Peripherals::take().expect("cannot take peripherals");

    let mut rcc = dp.RCC.constrain();
    let gpioa = dp.GPIOA.split(&mut rcc);
    let mut pwm = dp.TIM14.pwm(gpioa.pa4, 10.khz(), &mut rcc);

    let max = pwm.get_max_duty();
    pwm.set_duty(max / 2);

    pwm.enable();
    asm::bkpt();

    pwm.set_duty(max / 4);
    asm::bkpt();

    pwm.set_duty(max / 8);
    asm::bkpt();

    pwm.set_duty(max);
    asm::bkpt();

    loop {}
}

#[exception]
fn HardFault(ef: &ExceptionFrame) -> ! {
    panic!("Hard fault {:#?}", ef);
}

#[exception]
fn DefaultHandler(irqn: i16) {
    panic!("Unhandled exception (IRQn = {})", irqn);
}
