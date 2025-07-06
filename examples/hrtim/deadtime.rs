#![no_std]
#![no_main]
#![allow(clippy::uninlined_format_args)]

#[path = "../utils/mod.rs"]
mod utils;
use utils::logger::info;

use cortex_m_rt::entry;
use stm32_hrtim::{
    compare_register::HrCompareRegister, deadtime::DeadtimeConfig, output::HrOutput,
    timer::HrTimer, HrParts, HrPwmAdvExt, Pscl1,
};
use stm32g4xx_hal::{
    delay::{DelayExt, SYSTDelayExt},
    gpio::GpioExt,
    hrtim::{HrControltExt, HrPwmBuilderExt},
    pwr::PwrExt,
    rcc::{self, RccExt},
    stm32::{CorePeripherals, Peripherals},
    time::ExtU32,
};

#[entry]
fn main() -> ! {
    info!("Initializing...");

    let dp = Peripherals::take().expect("cannot take peripherals");
    let cp = CorePeripherals::take().expect("cannot take core");
    // Set system frequency to 16MHz * 15/1/2 = 120MHz
    // This would lead to HrTim running at 120MHz * 32 = 3.84GHz...
    let pwr = dp.PWR.constrain().freeze();
    let mut rcc = dp.RCC.freeze(
        rcc::Config::pll().pll_cfg(rcc::PllConfig {
            mux: rcc::PllSrc::HSI,
            n: rcc::PllNMul::MUL_15,
            m: rcc::PllMDiv::DIV_1,
            r: Some(rcc::PllRDiv::DIV_2),

            ..Default::default()
        }),
        pwr,
    );

    let mut delay = cp.SYST.delay(&rcc.clocks);

    // ...with a prescaler of 1 this gives us a HrTimer with a tick rate of 960MHz
    // With max the max period set, this would be 960MHz/2^16 ~= 14.6kHz...
    let prescaler = Pscl1;

    let gpioa = dp.GPIOA.split(&mut rcc);
    let pin_a = gpioa.pa8; // D7 On nucleo-G474
    let pin_b = gpioa.pa9; // D8 On nucleo-G474

    //        .               .               .
    //        .duty           .               .
    //         ----           .----           .
    //out1    |    |          |    |          .
    //        |    |          |    |          .
    // --------    ------------    -----------.
    //        .       ------          ------  .
    //out2    .      |      |        |      | .
    //        .      |      |        |      | .
    // ---------------      ----------      --.
    //        .   |<>|      |<>|              .
    //        .   33ns      33ns              .
    let (hr_control, ..) = dp.HRTIM_COMMON.hr_control(&mut rcc).wait_for_calibration();
    let mut hr_control = hr_control.constrain();

    let deadtime = DeadtimeConfig::default()
        .prescaler(stm32_hrtim::deadtime::DeadtimePrescaler::ThrtimDiv8)
        .deadtime_rising_value(32) // 32 / (8 * 120MHz) = ~33.33ns
        .deadtime_falling_value(32); // 32 / (8 * 120MHz) = ~33.33ns
    let period = 0xF00; // (120MHz * 32) / 0xF00 = 1MHz
    let HrParts {
        mut timer,
        mut cr1,
        out: (mut out1, mut out2),
        ..
    } = dp
        .HRTIM_TIMA
        .pwm_advanced((pin_a, pin_b))
        .prescaler(prescaler)
        .period(period)
        .deadtime(deadtime)
        .out1_polarity(stm32_hrtim::Polarity::ActiveHigh)
        .out2_polarity(stm32_hrtim::Polarity::ActiveHigh)
        .finalize(&mut hr_control);

    out1.enable_rst_event(&cr1); // Set low on compare match with cr1
    out2.enable_rst_event(&cr1);

    out1.enable_set_event(&timer); // Set high at new period
    out2.enable_set_event(&timer);

    out1.enable();
    out2.enable();
    timer.start(&mut hr_control.control);

    loop {
        // Step duty cycle from 10% to 90%
        for i in 1..10 {
            let p = timer.get_period();
            info!("p: {}", p);
            info!("Duty: {}%", i * 10);
            cr1.set_duty(i * period / 10);

            delay.delay(2000_u32.millis());
        }
    }
}
