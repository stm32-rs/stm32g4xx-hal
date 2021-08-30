#![deny(warnings)]
#![deny(unsafe_code)]
#![no_main]
#![no_std]

use hal::i2c::Config;
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

    let dp = stm32::Peripherals::take().expect("cannot take peripherals");
    let mut rcc = dp.RCC.constrain();
    let gpiob = dp.GPIOB.split(&mut rcc);

    let sda = gpiob.pb9.into_alternate_open_drain();
    let scl = gpiob.pb8.into_alternate_open_drain();

    let mut i2c = dp.I2C1.i2c(sda, scl, Config::new(40.khz()), &mut rcc);
    // Alternatively, it is possible to specify the exact timing as follows (see the documentation
    // of with_timing() for an explanation of the constant):
    //let mut i2c = dp
    //  .I2C1
    //   .i2c(sda, scl, Config::with_timing(0x3042_0f13), &mut rcc);

    let buf: [u8; 1] = [0];
    loop {
        match i2c.write(0x3c, &buf) {
            Ok(_) => info!("ok"),
            Err(err) => info!("error: {:?}", err),
        }
    }
}
