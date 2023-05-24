#![deny(warnings)]
#![deny(unsafe_code)]
#![no_main]
#![no_std]

extern crate cortex_m;
extern crate cortex_m_rt as rt;
extern crate panic_semihosting;
extern crate stm32g4xx_hal as hal;

use cortex_m_semihosting::hprintln;
use rt::entry;

use defmt_rtt as _;

#[entry]
fn main() -> ! {
    hprintln!("Hello, STM32G4!").unwrap();

    #[allow(clippy::empty_loop)]
    loop {}
}
