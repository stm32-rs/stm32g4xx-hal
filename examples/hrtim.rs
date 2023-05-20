//This example puts the timer in PWM mode using the specified pin with a frequency of 100Hz and a duty cycle of 50%.
#![no_main]
#![no_std]

use cortex_m_rt::entry;
use fugit::RateExtU32;
use hal::gpio::gpioa::PA8;
use hal::gpio::gpioa::PA9;
use hal::gpio::Alternate;
use hal::gpio::AF13;
use hal::prelude::*;
use hal::pwm::hrtim::HrPwmExt;
use hal::pwm::hrtim::Pscl4;
use hal::rcc;
use hal::stm32;
use stm32g4xx_hal as hal;
mod utils;
extern crate cortex_m_rt as rt;

#[entry]
fn main() -> ! {
    let dp = stm32::Peripherals::take().expect("cannot take peripherals");

    //Set system frequency to 16MHz * 75/4/2 = 150MHz
    let mut rcc = dp.RCC.freeze(rcc::Config::pll().pll_cfg(rcc::PllConfig {
        n: rcc::PllNMul::MUL_75,
        m: rcc::PllMDiv::DIV_4,
        r: Some(rcc::PllRDiv::DIV_2),
        ..Default::default()
    }));

    let gpioa = dp.GPIOA.split(&mut rcc);
    let pin_a: PA8<Alternate<AF13>> = gpioa.pa8.into_alternate();
    let pin_b: PA9<Alternate<AF13>> = gpioa.pa9.into_alternate();

    type Prescaler = Pscl4; // Prescaler of 4

    let (mut p1, mut p2) =
        dp.HRTIM_TIMA
            .pwm::<_, _, Prescaler, _, _>((pin_a, pin_b), 10_u32.kHz(), &mut rcc);
    p1.set_duty(0x1FFF);
    p2.set_duty(0xEFFF);

    p1.enable();
    p2.enable();

    //        .               .
    //        .  30%          .
    //         ----           .                 ----
    //out1    |    |          .                |    |
    //        |    |          .                |    |
    // --------    ----------------------------    --------------------
    //        .                ----           .                ----
    //out2    .               |    |                          |    |
    //        .               |    |                          |    |
    // ------------------------    ----------------------------    ----
    /*let (timer, cr1, _cr2, _cr3, _cr4, (out1, out2)) = dp.HRTIM_TIMA.pwm_hrtim::<_, Prescaler>((pin_a, pin_b), 10_u32.kHz(), &mut rcc)
        .period(0xFFFF)
        .mode(Mode::PushPull)   // Set push pull mode, out1 and out2 are
                                // alternated every period with one being
                                // inactive and the other getting to output its wave form
                                // as normal
        .finalize();

    out1.rst_event(EventSource::Cr1); // Set low on compare match with cr1
    out2.rst_event(EventSource::Cr1);

    out1.set_event(EventSource::Period); // Set high at new period
    out2.set_event(EventSource::Period);
                        // ^
                        // |
                        //  *---- Perhaps multiple EventSources could be Or:ed together?

    cr1.set_duty(timer.get_period() / 3);
    timer.set_period(foo);*/

    loop {
        cortex_m::asm::nop()
    }
}
