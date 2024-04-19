#![no_std]
#![no_main]

/// Example showcasing the use of the HRTIM peripheral together with a comparator to implement a current fault.
/// Once the digital input goes high, the output is forced low and put into a fault state.

#[path = "../utils/mod.rs"]
mod utils;

use cortex_m_rt::entry;

use defmt_rtt as _; // global logger
use panic_probe as _;

use utils::logger::info;

#[entry]
fn main() -> ! {
    use hal::gpio::gpioa::PA8;
    use hal::gpio::Alternate;
    use hal::gpio::AF13;
    use hal::hrtim::compare_register::HrCompareRegister;
    use hal::hrtim::fault::FaultAction;
    use hal::hrtim::timer::HrTimer;
    use hal::hrtim::HrPwmAdvExt;
    use hal::hrtim::Pscl4;
    use hal::hrtim::{control::HrControltExt, output::HrOutput};
    use hal::prelude::*;
    use hal::pwm::FaultMonitor;
    use hal::pwr::PwrExt;
    use hal::rcc;
    use hal::stm32;
    use hal::time::ExtU32;
    use stm32g4xx_hal as hal;

    let dp = stm32::Peripherals::take().expect("cannot take peripherals");
    let cp = stm32::CorePeripherals::take().expect("cannot take core");
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

    let mut delay = cp.SYST.delay(&rcc.clocks);

    let gpioa = dp.GPIOA.split(&mut rcc);
    let gpiob = dp.GPIOB.split(&mut rcc);
    let (hr_control, flt_inputs, _) = dp.HRTIM_COMMON.hr_control(&mut rcc).wait_for_calibration();
    let mut hr_control = hr_control.constrain();

    let fault_source3 = flt_inputs
        .fault_input3
        .bind_pin(gpiob.pb10.into_pull_down_input())
        .polarity(hal::pwm::Polarity::ActiveHigh)
        .finalize(&mut hr_control);

    // ...with a prescaler of 4 this gives us a HrTimer with a tick rate of 1.2GHz
    // With max the max period set, this would be 1.2GHz/2^16 ~= 18kHz...
    let prescaler = Pscl4;

    let pin_a: PA8<Alternate<AF13>> = gpioa.pa8.into_alternate();

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
    let (mut timer, (mut cr1, _cr2, _cr3, _cr4), mut out1, ..) = dp
        .HRTIM_TIMA
        .pwm_advanced(pin_a, &mut rcc)
        .prescaler(prescaler)
        .period(0xFFFF)
        //.with_fault_source(fault_source1)
        //.with_fault_source(fault_source2)
        .with_fault_source(fault_source3) // Set fault source
        //.with_fault_source(fault_source4)
        //.with_fault_source(fault_source5)
        //.with_fault_source(fault_source6)
        .fault_action1(FaultAction::ForceInactive)
        .fault_action2(FaultAction::ForceInactive)
        // alternated every period with one being
        // inactive and the other getting to output its wave form
        // as normal
        .finalize(&mut hr_control);

    out1.enable_rst_event(&cr1); // Set low on compare match with cr1
    out1.enable_set_event(&timer); // Set high at new period
    cr1.set_duty(timer.get_period() / 3);
    //unsafe {((HRTIM_COMMON::ptr() as *mut u8).offset(0x14) as *mut u32).write_volatile(1); }
    out1.enable();
    timer.start(&mut hr_control);

    info!("Started");

    loop {
        for _ in 0..5 {
            delay.delay(500_u32.millis());
            info!("State: {}", out1.get_state());
        }
        if hr_control.fault_3.is_fault_active() {
            hr_control.fault_3.clear_fault(); // Clear fault every 5s
            out1.enable();
            info!("failt cleared, and output reenabled");
        }
    }
}
