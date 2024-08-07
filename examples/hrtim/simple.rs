#![no_std]
#![no_main]

/// Example showcasing the use of the HRTIM peripheral generating two 20kHz pwm signals. One with 33% duty and the other with 50%

#[path = "../utils/mod.rs"]
mod utils;

use cortex_m_rt::entry;

use defmt_rtt as _; // global logger
use panic_probe as _;

use utils::logger::info;

#[entry]
fn main() -> ! {
    use hal::gpio::gpioa::PA8;
    use hal::gpio::gpioa::PA9;
    use hal::gpio::Alternate;
    use hal::gpio::AF13;
    use hal::hrtim::{control::HrControltExt, HrPwmExt};
    use hal::prelude::*;
    use hal::pwr::PwrExt;
    use hal::rcc;
    use hal::stm32;
    use hal::time::RateExtU32;
    use stm32g4xx_hal as hal;
    extern crate cortex_m_rt as rt;

    info!("Initializing...");

    let dp = stm32::Peripherals::take().expect("cannot take peripherals");
    // Set system frequency to 16MHz * 75/4/2 = 150MHz
    // This would lead to HrTim running at 150MHz * 32 = 4.8GHz...
    let pwr = dp.PWR.constrain().freeze();
    let mut rcc = dp.RCC.freeze(
        rcc::Config::pll().pll_cfg(rcc::PllConfig {
            mux: rcc::PLLSrc::HSI,
            n: rcc::PllNMul::MUL_75,
            m: rcc::PllMDiv::DIV_4,
            r: Some(rcc::PllRDiv::DIV_2),
            ..Default::default()
        }),
        pwr,
    );

    let gpioa = dp.GPIOA.split(&mut rcc);
    let pin_a: PA8<Alternate<AF13>> = gpioa.pa8.into_alternate();
    let pin_b: PA9<Alternate<AF13>> = gpioa.pa9.into_alternate();

    //        .               .               .
    //        .  33%          .               .
    //         ----           .----           .----
    //out1    |    |          |    |          |    |
    //        |    |          |    |          |    |
    //   ------    ------------    ------------    ---------
    //                        .               .
    //          50%           .               .
    //         --------       .--------       .--------
    //out2    |        |      |        |      |        |
    //        |        |      |        |      |        |
    //   ------        --------        --------        -----
    //        .               .               .
    //        .               .               .

    let (hr_control, ..) = dp.HRTIM_COMMON.hr_control(&mut rcc).wait_for_calibration();
    let mut hr_control = hr_control.constrain();
    let (mut p1, mut p2) =
        dp.HRTIM_TIMA
            .pwm((pin_a, pin_b), 20_u32.kHz(), &mut hr_control, &mut rcc);
    let max_duty = p1.get_max_duty();

    p1.set_duty(max_duty / 3); // Set output 1 to about 33%
    p2.set_duty(max_duty / 2); // Set output 2 to 50%

    // Enable the outputs
    p1.enable();
    p2.enable();

    info!("Running");

    loop {
        cortex_m::asm::nop()
    }
}
