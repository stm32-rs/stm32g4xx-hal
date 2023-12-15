#![no_main]
#![no_std]

use cortex_m_rt::entry;
use hal::prelude::*;
use hal::stm32;
use stm32g4xx_hal as hal;
use hal::flash::FlashSize;
use hal::flash::FlashExt;
extern crate cortex_m_rt as rt;

#[macro_use]
mod utils;

use utils::logger::println;

#[entry]
fn main() -> ! {
    utils::logger::init();

    let dp = stm32::Peripherals::take().expect("cannot take peripherals");
    let mut rcc = dp.RCC.constrain();

    let mut flash = dp.FLASH.constrain();
    let is_dual_bank = flash.is_dual_bank();

    println!("Is dual bank: {}", is_dual_bank);

    let mut flash_writer = flash.writer::<2048>(FlashSize::Sz256K);

    let bytes = flash_writer.read(0x0000_0000, 4).unwrap();
    println!("Bank 1 first 4 bytes: {:?}", bytes);

    if is_dual_bank {
        let address = 0x0001_F000;
        let bytes = flash_writer.read(address, 4).unwrap();
        println!("Bank 2 first 4 bytes before write: {:?}", bytes);
        
        flash_writer.page_erase(address).unwrap();
        //let bytes = flash_writer.read(address, 4).unwrap();
        //println!("Bank 2 first 4 bytes after erase: {:?}", bytes);

        let bytes = flash_writer.write(address, &[1, 2, 3, 4], true).unwrap();
        let bytes = flash_writer.read(address, 4).unwrap();
        println!("Bank 2 first 4 bytes after write: {:?}", bytes);
    }

    loop {
        cortex_m::asm::nop()
    }
}
