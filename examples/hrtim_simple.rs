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

    // Set system frequency to 16MHz * 75/4/2 = 150MHz
    // This would lead to HrTim running at 150MHz * 32 = 4.8GHz...
    let mut rcc = dp.RCC.freeze(rcc::Config::pll().pll_cfg(rcc::PllConfig {
        n: rcc::PllNMul::MUL_75,
        m: rcc::PllMDiv::DIV_4,
        r: Some(rcc::PllRDiv::DIV_2),
        ..Default::default()
    }));

    // ...with a prescaler of 4 this gives us a HrTimer with a tick rate of 1.2GHz
    // With max the max period set, this would be 1.2GHz/2^16 ~= 18kHz...
    type Prescaler = Pscl4; // Prescaler of 4

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

    // ...Meaning that we will have plenty of resolution when producing a 20kHz waveform
    let (mut p1, mut p2) =
        dp.HRTIM_TIMA
            .pwm::<_, _, Prescaler, _, _>((pin_a, pin_b), 20_u32.kHz(), &mut rcc);
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
