#![deny(warnings)]
#![deny(unsafe_code)]
#![no_main]
#![no_std]

extern crate cortex_m;
extern crate cortex_m_rt as rt;
extern crate cortex_m_semihosting as sh;
extern crate nb;
extern crate panic_halt;
extern crate stm32g4xx_hal as hal;

use hal::prelude::*;
use hal::rcc::Config;
use hal::spi;
use hal::stm32;
use nb::block;
use rt::entry;

#[entry]
fn main() -> ! {
    let dp = stm32::Peripherals::take().expect("cannot take peripherals");
    let mut rcc = dp.RCC.freeze(Config::pll());
    let gpioa = dp.GPIOA.split(&mut rcc);

    let sck = gpioa.pa1;
    let mosi = gpioa.pa2;
    let miso = gpioa.pa6;

    let mut spi = dp
        .SPI1
        .spi((sck, miso, mosi), spi::MODE_0, 100.khz(), &mut rcc);

    loop {
        block!(spi.send(128)).unwrap();
    }
}
