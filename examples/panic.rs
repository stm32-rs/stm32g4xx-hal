#![no_main]
#![no_std]

use defmt_rtt as _;

use panic_probe as _;

use stm32g4 as _;

use defmt::Format;

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
    defmt::info!("main");

    panic!("Something bad");
    // defmt::panic!()
}
