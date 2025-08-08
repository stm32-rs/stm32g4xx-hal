// This example is to test the SPI without any external devices.
// It puts "Hello world!" on the mosi-line and logs whatever is received on the miso-line to the info level.
// The idea is that you should connect miso and mosi, so you will also receive "Hello world!".

#![no_main]
#![no_std]

use hal::{
    delay::DelayFromCountDownTimer,
    gpio::{AF5, PA5, PA6, PA7},
    prelude::*,
    pwr::PwrExt,
    rcc::Config,
    spi,
    stm32::Peripherals,
    time::{ExtU32, RateExtU32},
    timer::Timer,
};

use cortex_m_rt::entry;
use stm32g4xx_hal as hal;
use utils::logger::info;

#[macro_use]
mod utils;

#[entry]
fn main() -> ! {
    utils::logger::init();

    let dp = Peripherals::take().unwrap();
    let rcc = dp.RCC.constrain();
    let pwr = dp.PWR.constrain().freeze();
    let mut rcc = rcc.freeze(Config::hsi(), pwr);
    let timer2 = Timer::new(dp.TIM2, &rcc.clocks);
    let mut delay_tim2 = DelayFromCountDownTimer::new(timer2.start_count_down(100.millis()));

    let gpioa = dp.GPIOA.split(&mut rcc);
    let sclk: PA5<AF5> = gpioa.pa5.into_alternate();
    let miso: PA6<AF5> = gpioa.pa6.into_alternate();
    let mosi: PA7<AF5> = gpioa.pa7.into_alternate();

    let mut spi = dp
        .SPI1
        .spi((sclk, miso, mosi), spi::MODE_0, 400.kHz(), &mut rcc);
    let mut cs = gpioa.pa8.into_push_pull_output();
    cs.set_high();

    // "Hello world!"
    let message = b"Hello world!";
    let mut received_msg = [0u8; 12];

    loop {
        cs.set_low();
        // write only, nothing is received
        spi.write(message).unwrap();
        cs.set_high();

        cs.set_low();
        // transmit and receive at the same time
        spi.transfer(&mut received_msg, message).unwrap();
        info!("{:?}", &received_msg);
        cs.set_high();

        delay_tim2.delay_ms(1000);
    }
}
