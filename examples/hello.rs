#![deny(warnings)]
#![deny(unsafe_code)]
#![no_main]
#![no_std]

extern crate cortex_m;
extern crate cortex_m_rt as rt;
extern crate stm32g4xx_hal as hal;

use rt::entry;

#[macro_use]
mod utils;

use utils::logger::println;

#[entry]
fn main() -> ! {
    println!("Hello, STM32G4!");

    #[allow(clippy::empty_loop)]
    loop {}
}
