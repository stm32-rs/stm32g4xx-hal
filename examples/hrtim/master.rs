//This example puts the timer in PWM mode using the specified pin with a frequency of 100Hz and a duty cycle of 50%.
#![no_main]
#![no_std]

use cortex_m_rt::entry;

#[cfg(not(any(feature = "stm32g474", feature = "stm32g484")))]
#[entry]
fn main() -> ! {
    #[allow(clippy::empty_loop)]
    loop {}
}

use defmt_rtt as _; // global logger
use panic_probe as _;

#[cfg(any(feature = "stm32g474", feature = "stm32g484"))]
#[entry]
fn main() -> ! {
    use fugit::ExtU32;
    use hal::gpio::gpioa::PA8;
    use hal::gpio::gpioa::PA9;
    use hal::gpio::Alternate;
    use hal::gpio::AF13;
    use hal::hrtim::compare_register::HrCompareRegister;
    use hal::hrtim::control::HrControltExt;
    use hal::hrtim::output::HrOutput;
    use hal::hrtim::timer::HrTimer;
    use hal::hrtim::HrPwmAdvExt;
    use hal::hrtim::{
        event::TimerAResetEventSource, HrTimerMode, MasterPreloadSource, PreloadSource, Pscl4,
    };
    use hal::prelude::*;
    use hal::pwr::PwrExt;
    use hal::rcc;
    use hal::stm32;
    use stm32g4xx_hal as hal;
    extern crate cortex_m_rt as rt;

    let dp = stm32::Peripherals::take().expect("cannot take peripherals");
    let cp = stm32::CorePeripherals::take().expect("cannot take core");
    // Set system frequency to 16MHz * 15/1/2 = 120MHz
    // This would lead to HrTim running at 120MHz * 32 = 3.84...
    let pwr = dp.PWR.constrain().freeze();
    let mut rcc = dp.RCC.freeze(
        rcc::Config::pll().pll_cfg(rcc::PllConfig {
            mux: rcc::PLLSrc::HSI,
            n: rcc::PllNMul::MUL_15,
            m: rcc::PllMDiv::DIV_1,
            r: Some(rcc::PllRDiv::DIV_2),

            ..Default::default()
        }),
        pwr,
    );

    let mut delay = cp.SYST.delay(&rcc.clocks);

    // ...with a prescaler of 4 this gives us a HrTimer with a tick rate of 960MHz
    // With max the max period set, this would be 960MHz/2^16 ~= 15kHz...
    let prescaler = Pscl4;

    let gpioa = dp.GPIOA.split(&mut rcc);
    let pin_a: PA8<Alternate<AF13>> = gpioa.pa8.into_alternate();
    let pin_b: PA9<Alternate<AF13>> = gpioa.pa9.into_alternate();

    //        .               .               .               .
    //        .  30%          .               .               .
    //         ----           .               .----           .
    //out1    |    |          .               |    |          .
    //        |    |          .               |    |          .
    // --------    ----------------------------    --------------------
    //        .               .----           .               .----
    //out2    .               |    |          .               |    |
    //        .               |    |          .               |    |
    // ------------------------    ----------------------------    ----
    //        .               .               .               .
    //        .               .               .               .
    let (hr_control, ..) = dp.HRTIM_COMMON.hr_control(&mut rcc).wait_for_calibration();
    let mut hr_control = hr_control.constrain();

    let (mut timer, (mut cr1, _cr2, _cr3, _cr4), (mut out1, mut out2)) = dp
        .HRTIM_TIMA
        .pwm_advanced((pin_a, pin_b), &mut rcc)
        .prescaler(prescaler)
        .push_pull_mode(true) // Set push pull mode, out1 and out2 are
        // alternated every period with one being
        // inactive and the other getting to output its wave form
        // as normal
        .preload(PreloadSource::OnMasterTimerUpdate)
        .timer_mode(HrTimerMode::SingleShotRetriggerable)
        .finalize(&mut hr_control);

    let (mut mtimer, (mut mcr1, _mcr2, _mcr3, _mcr4)) = dp
        .HRTIM_MASTER
        .pwm_advanced((), &mut rcc)
        .prescaler(prescaler)
        .preload(MasterPreloadSource::OnMasterRepetitionUpdate)
        .period(0xFFFF)
        .finalize(&mut hr_control);

    // Run in sync with master timer
    timer.enable_reset_event(TimerAResetEventSource::MasterPeriod);

    out1.enable_rst_event(&mcr1); // Set low on compare match with cr1
    out2.enable_rst_event(&mcr1);

    out1.enable_set_event(&mtimer); // Set high at new period
    out2.enable_set_event(&mtimer);

    out1.enable();
    out2.enable();

    let tima = unsafe { &*stm32g4xx_hal::stm32::HRTIM_TIMA::ptr() };
    defmt::info!("set1r: {}", tima.seta1r.read().bits());
    defmt::info!("rst1r: {}", tima.rsta1r.read().bits());

    defmt::info!("set2r: {}", tima.seta2r.read().bits());
    defmt::info!("rst2r: {}", tima.rsta2r.read().bits());

    defmt::info!("Running");

    loop {
        // Step frequency from 18kHz to about 180kHz(half of that when only looking at one pin)
        for i in 1..10 {
            let new_period = u16::MAX / i;

            mcr1.set_duty(new_period / 3);
            cr1.set_duty(new_period / 3 - 1000);
            mtimer.set_period(new_period);
            timer.set_period(new_period - 1000);

            defmt::info!(
                "period: {}, duty: {}, get_duty: {}, get_period: {}",
                new_period,
                new_period / 3,
                mcr1.get_duty(),
                mtimer.get_period()
            );

            delay.delay(500_u32.millis());
        }
    }
}
