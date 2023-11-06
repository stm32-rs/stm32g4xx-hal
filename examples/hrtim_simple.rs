//This example puts the timer in PWM mode using the specified pin with a frequency of 100Hz and a duty cycle of 50%.
#![no_main]
#![no_std]

use cortex_m_rt::entry;

//mod utils;

use defmt_rtt as _; // global logger
use panic_probe as _;

#[cfg(not(any(feature = "stm32g474", feature = "stm32g484")))]
#[entry]
fn main() -> ! {
    #[allow(clippy::empty_loop)]
    loop {}
}

#[cfg(any(feature = "stm32g474", feature = "stm32g484"))]
#[entry]
fn main() -> ! {
    use hal::gpio::gpioa::PA8;
    use hal::gpio::gpioa::PA9;
    use hal::gpio::Alternate;
    use hal::gpio::AF13;
    use hal::prelude::*;
    use hal::pwm::hrtim::{HrControltExt, HrPwmExt};
    use hal::rcc;
    use hal::stm32;
    use hal::time::RateExtU32;
    use stm32g4xx_hal as hal;
    extern crate cortex_m_rt as rt;
    let dp = stm32::Peripherals::take().expect("cannot take peripherals");

    // Set system frequency to 16MHz * 75/4/2 = 150MHz
    // This would lead to HrTim running at 150MHz * 32 = 4.8GHz...
    let mut rcc = dp.RCC.freeze(rcc::Config::pll().pll_cfg(rcc::PllConfig {
        n: rcc::PllNMul::MUL_75,
        m: rcc::PllMDiv::DIV_4,
        r: Some(rcc::PllRDiv::DIV_2),
        ..Default::default()
    }));

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

    let (mut control, _) = dp.HRTIM_COMMON.hr_control(&mut rcc).wait_for_calibration();
    let (mut p1, mut p2) = dp
        .HRTIM_TIMA
        .pwm((pin_a, pin_b), 20_u32.kHz(), &mut control, &mut rcc);
    let max_duty = p1.get_max_duty();

    p1.set_duty(max_duty / 3); // Set output 1 to about 33%
    p2.set_duty(max_duty / 2); // Set output 2 to 50%

    // Enable the outputs
    p1.enable();
    p2.enable();

    loop {
        cortex_m::asm::nop()
    }
}