#![deny(warnings)]
#![deny(unsafe_code)]
#![no_main]
#![no_std]

extern crate cortex_m;
extern crate cortex_m_rt as rt;
extern crate nb;
extern crate panic_halt;
extern crate stm32g4xx_hal as hal;

use core::fmt::Write;

use nb::block;
use hal::prelude::*;
use hal::serial::Config;
use hal::stm32;
use rt::entry;

#[entry]
fn main() -> ! {
    let dp = stm32::Peripherals::take().expect("cannot take peripherals");
    let mut rcc = dp.RCC.constrain();
    let gpio = dp.GPIOA.split(&mut rcc);
    let mut usart = dp
        .LPUART1
        .usart(gpio.pa2, gpio.pa3, Config::default(), &mut rcc)
        .unwrap();
    
    write!(usart, "Hello\r\n").unwrap();
    
    let mut cnt = 0;
    loop {
        let byte = block!(usart.read()).unwrap();
        write!(usart, "{}: {}\r\n", cnt, byte).unwrap();
        cnt += 1;
    }
}
