#![no_std]
#![no_main]

#[path = "../utils/mod.rs"]
mod utils;
use utils::logger::info;

/// Example showcasing the use of the HRTIM peripheral together with a comparator to implement a current fault.
/// Once the digital input goes high, the output is forced low and put into a fault state.
use cortex_m_rt::entry;
use stm32_hrtim::{
    compare_register::HrCompareRegister,
    fault::{FaultAction, FaultMonitor},
    output::HrOutput,
    timer::HrTimer,
    HrParts, HrPwmAdvExt, Polarity, Pscl4,
};
use stm32g4xx_hal::{
    delay::{DelayExt, SYSTDelayExt},
    gpio::GpioExt,
    hrtim::{fault::FaultInput, HrControltExt, HrPwmBuilderExt},
    pwr::PwrExt,
    rcc::{self, RccExt},
    stm32::{CorePeripherals, Peripherals},
    time::ExtU32,
};

#[entry]
fn main() -> ! {
    let dp = Peripherals::take().expect("cannot take peripherals");
    let cp = CorePeripherals::take().expect("cannot take core");
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

    let mut delay = cp.SYST.delay(&rcc.clocks);

    let gpioa = dp.GPIOA.split(&mut rcc);
    let gpiob = dp.GPIOB.split(&mut rcc);
    let (hr_control, flt_inputs, _) = dp.HRTIM_COMMON.hr_control(&mut rcc).wait_for_calibration();
    let mut hr_control = hr_control.constrain();

    let pb10 = gpiob.pb10.into_pull_down_input().into_alternate();
    let fault_source3 = flt_inputs
        .fault_input3
        .bind(pb10)
        .polarity(Polarity::ActiveHigh)
        .finalize(&mut hr_control);

    // ...with a prescaler of 4 this gives us a HrTimer with a tick rate of 1.2GHz
    // With max the max period set, this would be 1.2GHz/2^16 ~= 18kHz...
    let prescaler = Pscl4;

    let pin_a = gpioa.pa8;

    //        .               .               .  *
    //        .  33%          .               .  *            .               .
    //        .-----.         .-----.         .--.            .               .
    //out1    |     |         |     |         |  |            .               .
    //        |     |         |     |         |  |            .               .
    //   ------     -----------     -----------  -----------------------------------
    //        .               .               .  *            .               .
    //        .               .               .  *            .               .
    //        .               .               .  *--------    .               .
    //fault   .               .               .  |        |   .               .
    //        .               .               .  |        |   .               .
    //   -----------------------------------------        --------------------------
    //        .               .               .  *            .               .
    //        .               .               .  *            .               .
    let HrParts {
        mut timer,
        mut cr1,
        mut out,
        ..
    } = dp
        .HRTIM_TIMA
        .pwm_advanced(pin_a)
        .prescaler(prescaler)
        .period(0xFFFF)
        .with_fault_source(fault_source3)
        .fault_action1(FaultAction::ForceInactive)
        .fault_action2(FaultAction::ForceInactive)
        .finalize(&mut hr_control);

    out.enable_rst_event(&cr1); // Set low on compare match with cr1
    out.enable_set_event(&timer); // Set high at new period
    cr1.set_duty(timer.get_period() / 3);

    out.enable();
    timer.start(&mut hr_control.control);

    info!("Started");

    loop {
        for _ in 0..5 {
            delay.delay(500_u32.millis());
            info!("State: {:?}", out.get_state());
        }
        if hr_control.fault_3.is_fault_active() {
            hr_control.fault_3.clear_fault(); // Clear fault every 5s
            out.enable();
            info!("failt cleared, and output reenabled");
        }
    }
}
