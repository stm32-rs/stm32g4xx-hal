// This example is to test the SPI without any external devices.
// It puts "Hello world!" on the mosi-line and logs whatever is received on the miso-line to the info level.
// The idea is that you should connect miso and mosi, so you will also receive "Hello world!".

#![no_main]
#![no_std]

use crate::hal::{
    block, delay::DelayFromCountDownTimer, gpio::gpioa::PA5, gpio::gpioa::PA6, gpio::gpioa::PA7,
    gpio::Alternate, gpio::AF5, prelude::*, rcc::Config, spi, stm32::Peripherals, timer::Timer,
};

use cortex_m_rt::entry;
use log::info;
use stm32g4xx_hal as hal;

#[macro_use]
mod utils;

#[entry]
fn main() -> ! {
    utils::logger::init();

    let dp = Peripherals::take().unwrap();
    let rcc = dp.RCC.constrain();
    let mut rcc = rcc.freeze(Config::hsi());
    let timer2 = Timer::new(dp.TIM2, &rcc.clocks);
    let mut delay_tim2 = DelayFromCountDownTimer::new(timer2.start_count_down(100.ms()));

    let gpioa = dp.GPIOA.split(&mut rcc);
    let sclk: PA5<Alternate<AF5>> = gpioa.pa5.into_alternate();
    let miso: PA6<Alternate<AF5>> = gpioa.pa6.into_alternate();
    let mosi: PA7<Alternate<AF5>> = gpioa.pa7.into_alternate();

    let mut spi = dp
        .SPI1
        .spi((sclk, miso, mosi), spi::MODE_0, 400.khz(), &mut rcc);
    let mut cs = gpioa.pa8.into_push_pull_output();
    cs.set_high().unwrap();

    // "Hello world!"
    let message: [char; 12] = ['H', 'e', 'l', 'l', 'o', ' ', 'w', 'o', 'r', 'l', 'd', '!'];
    let mut received_byte: u8;

    loop {
        for byte in message.iter() {
            cs.set_low().unwrap();
            spi.send(*byte as u8).unwrap();
            received_byte = block!(spi.read()).unwrap();
            cs.set_high().unwrap();

            info!("{}", received_byte as char);
        }
        delay_tim2.delay_ms(1000_u16);
    }
}
