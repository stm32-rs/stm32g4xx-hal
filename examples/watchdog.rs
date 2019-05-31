#![deny(warnings)]
#![deny(unsafe_code)]
#![no_main]
#![no_std]

extern crate cortex_m;
extern crate cortex_m_rt as rt;
extern crate cortex_m_semihosting;
extern crate panic_halt;
extern crate stm32g4xx_hal as hal;

use cortex_m_semihosting::hprintln;
use hal::prelude::*;
use hal::stm32;
use rt::{entry, exception, ExceptionFrame};

#[entry]
fn main() -> ! {
    let dp = stm32::Peripherals::take().expect("cannot take peripherals");
    let cp = cortex_m::Peripherals::take().expect("cannot take core peripherals");

    let mut rcc = dp.RCC.constrain();
    let mut delay = cp.SYST.delay(&rcc.clocks);
    hprintln!("Watchdog").unwrap();

    let mut watchdog = dp.WWDG.constrain(&mut rcc);
    // let mut watchdog = dp.IWDG.constrain();

    watchdog.start(100.ms());

    delay.delay(90.ms());
    // delay.delay(110.ms());

    cortex_m::asm::bkpt();

    loop {}
}

#[exception]
fn HardFault(ef: &ExceptionFrame) -> ! {
    panic!("Hard fault {:#?}", ef);
}
