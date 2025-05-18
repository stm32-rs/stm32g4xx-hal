#![no_std]
#![no_main]

#[path = "../utils/mod.rs"]
mod utils;
use utils::logger::info;

/// Example showcasing the use of the HRTIM peripheral together with a digital input to implement a cycle by cycle current limit.
/// Once the digital input goes high, the output is set low thus limiting the pulse width and in turn the current.
use cortex_m_rt::entry;
use stm32_hrtim::{
    compare_register::HrCompareRegister,
    external_event::{self, ToExternalEventSource},
    output::HrOutput,
    timer::HrTimer,
    timer_eev_cfg::EevCfgs,
    HrParts, HrPwmAdvExt, Polarity, Pscl4,
};
use stm32g4xx_hal::{
    gpio::GpioExt,
    hrtim::{external_event::EevInputExt, HrControltExt, HrPwmBuilderExt},
    pwr::PwrExt,
    rcc::{self, RccExt},
    stm32::Peripherals,
};

#[entry]
fn main() -> ! {
    let dp = Peripherals::take().expect("cannot take peripherals");
    // Set system frequency to 16MHz * 75/4/2 = 150MHz
    // This would lead to HrTim running at 150MHz * 32 = 4.8GHz...
    let pwr = dp.PWR.constrain().freeze();

    let mut rcc = dp.RCC.freeze(
        rcc::Config::pll().pll_cfg(rcc::PllConfig {
            mux: rcc::PllSrc::HSI,
            n: rcc::PllNMul::MUL_75,
            m: rcc::PllMDiv::DIV_4,
            r: Some(rcc::PllRDiv::DIV_2),
            ..Default::default()
        }),
        pwr,
    );

    let gpioa = dp.GPIOA.split(&mut rcc);
    let gpiob = dp.GPIOB.split(&mut rcc);

    let (mut hr_control, _flt_inputs, eev_inputs) =
        dp.HRTIM_COMMON.hr_control(&mut rcc).wait_for_calibration();

    let eev_input3 = eev_inputs
        .eev_input3
        .bind(gpiob.pb7.into_pull_down_input())
        .edge_or_polarity(external_event::EdgeOrPolarity::Polarity(
            Polarity::ActiveHigh,
        ))
        .finalize(&mut hr_control);

    let mut hr_control = hr_control.constrain();

    // ...with a prescaler of 4 this gives us a HrTimer with a tick rate of 1.2GHz
    // With max the max period set, this would be 1.2GHz/2^16 ~= 18kHz...
    let prescaler = Pscl4;

    let pin_a = gpioa.pa8;

    //        .               .  *            .
    //        .  33%          .  *            .               .               .
    //        .-----.         .--*            .-----.         .-----.         .-----
    //out1    |     |         |  |            |     |         |     |         |
    //        |     |         |  *            |     |         |     |         |
    //   ------     -----------  --------------     -----------     -----------
    //        .               .  *            .               .               .
    //        .               .  *            .               .               .
    //        .               .  *--------*   .               .               .
    //eev     .               .  |        |   .               .               .
    //        .               .  |        |   .               .               .
    //   -------------------------        ------------------------------------------
    //        .               .  *            .               .               .
    //        .               .  *            .               .               .
    let HrParts {
        mut timer,
        mut cr1,
        mut out,
        ..
    } = dp
        .HRTIM_TIMA
        .pwm_advanced(pin_a)
        .prescaler(prescaler)
        .eev_cfg(EevCfgs::default())
        .period(0xFFFF)
        .finalize(&mut hr_control);

    out.enable_rst_event(&cr1); // Set low on compare match with cr1
    out.enable_rst_event(&eev_input3);
    out.enable_set_event(&timer); // Set high at new period
    cr1.set_duty(timer.get_period() / 3);

    out.enable();
    timer.start(&mut hr_control.control);

    info!("Started");

    loop {
        cortex_m::asm::nop()
    }
}
