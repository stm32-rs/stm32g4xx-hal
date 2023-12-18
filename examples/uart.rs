#![deny(warnings)]
#![deny(unsafe_code)]
#![no_main]
#![no_std]

use core::fmt::Write;

use hal::prelude::*;
use hal::pwr::PwrExt;
use hal::serial::FullConfig;
use hal::{rcc, stm32};
use stm32g4xx_hal as hal;

use cortex_m_rt::entry;
use nb::block;
use utils::logger::info;

#[macro_use]
mod utils;

#[entry]
fn main() -> ! {
    utils::logger::init();

    info!("start");
    let dp = stm32::Peripherals::take().expect("cannot take peripherals");
    let pwr = dp.PWR.constrain().freeze();
    let mut rcc = dp.RCC.freeze(rcc::Config::hsi(), pwr);

    info!("Init UART");
    // on Nucleo-G474, the pins marked TX/RX are connected to USART1 by default, whereas USART2 is
    // connected to the ST-Link's UART.
    /*let gpioa = dp.GPIOA.split(&mut rcc);
    let tx = gpioa.pa2.into_alternate();
    let rx = gpioa.pa3.into_alternate();
    let mut usart = dp
        .USART2
        .usart(tx, rx, FullConfig::default(), &mut rcc)
        .unwrap();*/
    /*let gpioc = dp.GPIOC.split(&mut rcc);
    let tx = gpioc.pc4.into_alternate();
    let rx = gpioc.pc5.into_alternate();
    let mut usart = dp
        .USART1
        .usart(tx, rx, FullConfig::default(), &mut rcc)
        .unwrap();*/

    let gpioc = dp.GPIOC.split(&mut rcc);
    let tx = gpioc.pc10.into_alternate();
    let rx = gpioc.pc11.into_alternate();
    let mut usart = dp
        .USART3
        .usart(tx, rx, FullConfig::default(), &mut rcc)
        .unwrap();

    writeln!(usart, "Hello USART3, yay!!\r\n").unwrap();

    let mut cnt = 0;
    loop {
        match block!(usart.read()) {
            Ok(byte) => writeln!(usart, "{}: {}\r", cnt, byte).unwrap(),
            Err(e) => writeln!(usart, "E: {:?}\r", e).unwrap(),
        };
        cnt += 1;
    }
}
