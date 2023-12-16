#![no_main]
#![no_std]

use stm32g4 as _;

#[macro_use]
mod utils;

use utils::logger;

#[cfg(feature = "defmt")]
#[defmt::panic_handler]
fn panic() -> ! {
    cortex_m::asm::udf()
}

pub fn exit() -> ! {
    loop {
        cortex_m::asm::bkpt();
    }
}

#[cortex_m_rt::entry]
fn main() -> ! {
    utils::logger::init();

    logger::info!("main");

    panic!("Something bad");
}
