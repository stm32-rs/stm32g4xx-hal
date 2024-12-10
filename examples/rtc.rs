#![no_main]
#![no_std]

use hal::{
    delay::SYSTDelayExt,
    rcc::RccExt,
    rtc::RtcExt,
    stm32::Peripherals,
    time::{Date, ExtU32, Month, MonthDay, Time, Year},
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

    let mut rcc = dp.RCC.constrain();

    info!("Setup rtc");
    let mut delay = cp.SYST.delay(&rcc.clocks);
    let mut rtc = dp.RTC.constrain(&mut rcc);

    info!("Setup date");
    rtc.set_date(&Date::new(Year(2024), Month(8), MonthDay(5)));
    rtc.set_time(&Time::new(0.hours(), 59.minutes(), 2.secs(), true));

    info!("Enter Loop");

    loop {
        info!("Timestamp: {:?} {:?}", rtc.get_date(), rtc.get_time());
        delay.delay_ms(500);
    }
}
