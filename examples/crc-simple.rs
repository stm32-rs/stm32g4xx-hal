#![deny(warnings)]
#![deny(unsafe_code)]
#![no_main]
#![no_std]

use hal::crc::*;
use hal::stm32;
use stm32g4xx_hal as hal;

use cortex_m_rt::entry;
use defmt_rtt as _; // global logger

#[macro_use]
mod utils;

#[entry]
fn main() -> ! {
    utils::logger::init();

    defmt::info!("Init RCC & CRC");
    let dp = stm32::Peripherals::take().expect("cannot take peripherals");

    let crc = dp.CRC.constrain();
    let mut crc = crc
        .polynomial(Polynomial::L16(0x1021))
        .freeze();

    //defmt::info!("Check CRC");
    let data = [0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13];
    crc.feed(&data);

    let result = crc.result();

    defmt::info!("Result is: {:x}", result);

    if result == 0x78cb {
        defmt::warn!("> CRC is correct");
    }

    #[allow(clippy::empty_loop)]
    loop {}
}
