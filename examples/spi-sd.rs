// This example uses SPI to initialize an SD card.

#![no_main]
#![no_std]

extern crate embedded_sdmmc;

use fugit::RateExtU32;
use hal::gpio::gpiob::PB14;
use hal::gpio::gpiob::PB15;
use hal::gpio::gpiof::PF9;
use hal::gpio::Alternate;
use hal::gpio::AF5;
use hal::prelude::*;
use hal::pwr::PwrExt;
use hal::rcc::Config;
use hal::spi;

use hal::stm32::Peripherals;
use stm32g4xx_hal as hal;

use embedded_sdmmc::{TimeSource, Timestamp};

use cortex_m_rt::entry;

#[macro_use]
mod utils;

#[entry]
fn main() -> ! {
    utils::logger::init();

    let dp = Peripherals::take().unwrap();
    let rcc = dp.RCC.constrain();
    let pwr = dp.PWR.constrain().freeze();
    let mut rcc = rcc.freeze(Config::hsi(), pwr);
    let gpiob = dp.GPIOB.split(&mut rcc);
    let gpiof = dp.GPIOF.split(&mut rcc);

    let cs = {
        let mut cs = gpiof.pf8.into_push_pull_output();
        cs.set_high().unwrap();
        cs
    };

    let sck: PF9<Alternate<AF5>> = gpiof.pf9.into_alternate();
    let miso: PB14<Alternate<AF5>> = gpiob.pb14.into_alternate();
    let mosi: PB15<Alternate<AF5>> = gpiob.pb15.into_alternate();

    let spi = dp
        .SPI2
        .spi((sck, miso, mosi), spi::MODE_0, 400.kHz(), &mut rcc);

    struct Clock;

    impl TimeSource for Clock {
        fn get_timestamp(&self) -> Timestamp {
            Timestamp {
                year_since_1970: 0,
                zero_indexed_month: 0,
                zero_indexed_day: 0,
                hours: 0,
                minutes: 0,
                seconds: 0,
            }
        }
    }

    let mut cont = embedded_sdmmc::Controller::new(embedded_sdmmc::SdMmcSpi::new(spi, cs), Clock);

    cont.device().init().unwrap();

    #[allow(clippy::empty_loop)]
    loop {}
}
