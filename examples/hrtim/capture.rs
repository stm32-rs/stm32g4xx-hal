#![no_std]
#![no_main]

/// Example showcasing the use of the HRTIM peripheral's capture function to detect phase shift between a digital event and the output of HRTIM_TIMA

#[path = "../utils/mod.rs"]
mod utils;

use cortex_m_rt::entry;

use defmt_rtt as _; // global logger
use panic_probe as _;

use utils::logger::info;

#[entry]
fn main() -> ! {
    use stm32g4xx_hal as hal;

    use hal::{
        gpio::{gpioa::PA8, Alternate, GpioExt, AF13},
        hrtim::{
            capture::HrCapture, compare_register::HrCompareRegister, control::HrControltExt,
            external_event, external_event::ToExternalEventSource, output::HrOutput,
            timer::HrTimer, HrPwmAdvExt, Pscl128,
        },
        pwr::PwrExt,
        rcc::{self, RccExt},
        stm32::Peripherals,
    };
    use info;

    info!("start");

    let dp = Peripherals::take().unwrap();

    // Set system frequency to 16MHz * 15/1/2 = 120MHz
    // This would lead to HrTim running at 120MHz * 32 = 3.84...
    info!("rcc");
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

    info!("Setup Gpio");
    let gpioa = dp.GPIOA.split(&mut rcc);
    let gpiob = dp.GPIOB.split(&mut rcc);

    // PA8 (D7 on Nucleo G474RE)
    let pin_a: PA8<Alternate<AF13>> = gpioa.pa8.into_alternate();

    // ...with a prescaler of 128 this gives us a HrTimer with a tick rate of 30MHz
    // With max the max period set, this would be 30MHz/2^16 ~= 458Hz...
    let prescaler = Pscl128;

    //        .               .
    //        .  50%          .
    //         ------          ------
    //out1    |      |        |      |
    //        |      |        |      |
    // --------      ----------      --------
    let period = 0xFFFF;
    let (mut hr_control, _flt_inputs, eev_inputs) =
        dp.HRTIM_COMMON.hr_control(&mut rcc).wait_for_calibration();

    // PB5 (D4 on Nucleo G474RE)
    let eev_input6 = eev_inputs
        .eev_input6
        .bind(gpiob.pb5.into_pull_down_input())
        .edge_or_polarity(external_event::EdgeOrPolarity::Edge(
            external_event::Edge::Falling,
        ))
        .finalize(&mut hr_control);

    let mut hr_control = hr_control.constrain();
    let (mut timer, (mut cr1, _cr2, _cr3, _cr4), mut out1) = dp
        .HRTIM_TIMA
        .pwm_advanced(pin_a, &mut rcc)
        .prescaler(prescaler)
        .period(period)
        .finalize(&mut hr_control);

    out1.enable_rst_event(&cr1); // Set low on compare match with cr1
    out1.enable_set_event(&timer); // Set high at new period

    cr1.set_duty(period / 2);
    timer.start(&mut hr_control);
    out1.enable();

    let capture = timer.capture_ch1();
    capture.enable_interrupt(true, &mut hr_control);
    capture.add_event(&eev_input6);

    let mut old_duty = 0;
    loop {
        for duty in (u32::from(period) / 10)..(9 * u32::from(period) / 10) {
            if !capture.is_pending() {
                continue;
            }
            let value = capture.get_signed();
            cr1.set_duty(duty as u16);
            capture.clear_interrupt();
            info!(
                "Capture: {:?}, duty: {}, diff: {}",
                value,
                old_duty,
                value - old_duty as i32
            );
            old_duty = duty;
        }
    }
}
