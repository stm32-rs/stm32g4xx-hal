//This example puts the timer in PWM mode using the specified pin with a frequency of 100Hz and a duty cycle of 50%.
#![no_main]
#![no_std]

use cortex_m_rt::entry;
use fugit::ExtU32;
use hal::comparator::{ComparatorExt, ComparatorSplit, Config, Hysteresis, RefintInput};
use hal::dac::{Dac1IntSig1, DacExt, DacOut};
use hal::gpio::gpioa::PA8;
use hal::gpio::Alternate;
use hal::gpio::AF13;
use hal::prelude::*;
use hal::pwm::hrtim::EventSource;
use hal::pwm::hrtim::FaultAction;
use hal::pwm::hrtim::HrCompareRegister;
use hal::pwm::hrtim::HrPwmAdvExt;
use hal::pwm::hrtim::HrTimer;
use hal::pwm::hrtim::Pscl4;
use hal::pwm::hrtim::{HrControltExt, HrOutput};
use hal::pwm::FaultMonitor;
use hal::rcc;
use hal::stm32;
use stm32g4xx_hal as hal;
//mod utils;

use defmt_rtt as _; // global logger
use panic_probe as _;

#[entry]
fn main() -> ! {
    let dp = stm32::Peripherals::take().expect("cannot take peripherals");
    let cp = stm32::CorePeripherals::take().expect("cannot take core");
    // Set system frequency to 16MHz * 75/4/2 = 150MHz
    // This would lead to HrTim running at 150MHz * 32 = 4.8GHz...
    let mut rcc = dp.RCC.freeze(rcc::Config::pll().pll_cfg(rcc::PllConfig {
        mux: rcc::PLLSrc::HSI,
        n: rcc::PllNMul::MUL_75,
        m: rcc::PllMDiv::DIV_4,
        r: Some(rcc::PllRDiv::DIV_2),
        ..Default::default()
    }));

    let mut delay = cp.SYST.delay(&rcc.clocks);

    let gpioa = dp.GPIOA.split(&mut rcc);

    let dac1ch1 = dp.DAC1.constrain(Dac1IntSig1, &mut rcc);
    let mut dac = dac1ch1.calibrate_buffer(&mut delay).enable();

    // Use dac to define the fault threshold
    // 2^12 / 2 = 2^11 for about half of VCC
    let fault_limit = 1 << 11;
    dac.set_value(fault_limit);

    let (comp1, ..) = dp.COMP.split(&mut rcc);

    let pa1 = gpioa.pa1.into_analog();
    let comp1 = comp1
        .comparator(
            &pa1,
            &dac,
            Config::default()
                .hysteresis(Hysteresis::None)
                .output_inverted(),
            &rcc.clocks,
        )
        .enable();

    let (mut fault_control, flt_inputs) =
        dp.HRTIM_COMMON.hr_control(&mut rcc).wait_for_calibration();

    let fault_source4 = flt_inputs
        .fault_input4
        .bind_comp(&comp1)
        .polarity(hal::pwm::Polarity::ActiveLow)
        .finalize(&mut fault_control);

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
    let (timer, (mut cr1, _cr2, _cr3, _cr4), mut out1) = dp
        .HRTIM_TIMA
        .pwm_advanced(pin_a, &mut rcc)
        .prescaler(prescaler)
        .period(0xFFFF)
        .with_fault_source(fault_source4) // Set fault source
        .fault_action1(FaultAction::ForceInactive)
        .fault_action2(FaultAction::ForceInactive)
        .finalize(&mut fault_control);

    out1.enable_rst_event(EventSource::Cr1); // Set low on compare match with cr1
    out1.enable_set_event(EventSource::Period); // Set high at new period
    cr1.set_duty(timer.get_period() / 3);
    //unsafe {((HRTIM_COMMON::ptr() as *mut u8).offset(0x14) as *mut u32).write_volatile(1); }
    out1.enable();

    defmt::info!("Started");

    loop {
        for _ in 0..5 {
            delay.delay(500_u32.millis());
            defmt::info!(
                "State: {}, comp: {}, is_fault_active: {}",
                out1.get_state(),
                comp1.output(),
                fault_control.fault_4.is_fault_active()
            );
        }
        if fault_control.fault_4.is_fault_active() {
            fault_control.fault_4.clear_fault(); // Clear fault every 5s
            out1.enable();
            defmt::info!("failt cleared, and output reenabled");
        }
    }
}
