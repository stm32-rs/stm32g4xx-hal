#![no_main]
#![no_std]

use hal::{
    delay::SYSTDelayExt,
    rcc::{Config, RccExt},
    stm32::Peripherals,
    time::{Time, Date, Year, Month, MonthDay, Hour, Minute, Second, ExtU32},
    hal::prelude::*, rtc::RtcExt
};
use stm32g4xx_hal as hal;

use cortex_m_rt::entry;

use utils::logger::info;

#[macro_use]
mod utils;

#[entry]
fn main() -> ! {
    utils::logger::init();

    info!("start");

    let dp = Peripherals::take().unwrap();
    let cp = cortex_m::Peripherals::take().expect("cannot take core peripherals");

    info!("rcc");

    let rcc = dp.RCC.constrain();
    let mut rcc = rcc.freeze(Config::hsi());

    info!("Setup rtc");
    let mut delay = cp.SYST.delay(&rcc.clocks);
    let mut rtc = dp
        .RTC
        .constrain(&mut rcc);

    info!("Setup date");
    rtc.set_date(&Date::new(Year(2023), Month(6), MonthDay(4)));
    rtc.set_time(&Time::new(18.hours(), 43.minutes(), 2.secs(), true));

    info!("Enter Loop");

    loop {
        info!("Timestamp: {} {}", rtc.get_date(), rtc.get_time());
        delay.delay_ms(500);
    }
}
