#![no_main]
#![no_std]

use cortex_m_rt::entry;
use hal::flash::FlashExt;
use hal::flash::FlashSize;
use hal::stm32;
use stm32g4xx_hal as hal;
extern crate cortex_m_rt as rt;

#[macro_use]
mod utils;

use utils::logger::println;

#[entry]
fn main() -> ! {
    utils::logger::init();

    let dp = stm32::Peripherals::take().expect("cannot take peripherals");

    let mut flash = dp.FLASH.constrain();
    let is_dual_bank = flash.is_dual_bank();

    println!("Is dual bank: {}", is_dual_bank);

    let mut flash_writer = flash.writer::<2048>(FlashSize::Sz256K);

    let mut words = [0; 1];
    flash_writer.read_exact(0x0000_0000, &mut words[..]);
    println!("Bank 1 - first 4 bytes: {:#X}", words[0]);

    if is_dual_bank {
        let address = 0x0004_0000;
        flash_writer.read_exact(address, &mut words[..]);
        println!("Bank 2 - First 4 bytes before write: {:#X}", words[0]);

        flash_writer.page_erase(address).unwrap();
        flash_writer.read_exact(address, &mut words[..]);
        println!("Bank 2 - First 4 bytes after erase: {:#X}", words[0]);

        let new_value = words[0].wrapping_add(2);
        println!("Bank 2 - Writing: {:#X} to first 4 bytes", new_value);
        flash_writer
            .write(address, &new_value.to_ne_bytes(), true)
            .unwrap();
        flash_writer.read_exact(address, &mut words[..]);
        println!("Bank 2 - First 4 bytes after write: {:#X}", words[0]);
    }

    println!("Done!");

    loop {
        cortex_m::asm::nop()
    }
}
