// Originaly from stm32h7xx-hal, adapted for stm32g4xx-hal

#![no_main]
#![no_std]

//#[macro_use]
mod utils;
use stm32g4xx_hal::{independent_watchdog::IndependentWatchdog, stm32::Peripherals, time::ExtU32};

use cortex_m_rt::entry;
use utils::logger::info;

#[entry]
fn main() -> ! {
    utils::logger::init();
    let dp = Peripherals::take().unwrap();

    let mut watchdog = IndependentWatchdog::new(dp.IWDG);

    info!("");
    info!("stm32g4xx-hal example - Watchdog");
    info!("");

    // If the watchdog is working correctly this print should
    // appear again and again as the chip gets restarted
    info!("Watchdog restarted!           ");

    // Enable the watchdog with a limit of 32.76 seconds (which is the maximum this watchdog can do) and wait forever
    // -> restart the chip
    watchdog.start(32_760.millis());

    // Alternatively, there's also a windowed option where if the watchdog is fed before the window time, it will reset the chip as well
    // watchdog.start_windowed(100.millis(), 200.millis());

    loop {
        // We can feed the watchdog like this:
        // watchdog.feed();
        cortex_m::asm::nop()
    }
}
