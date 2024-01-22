#![no_std]
#![no_main]

use cortex_m_rt::entry;

use defmt_rtt as _; // global logger
use panic_probe as _;

#[entry]
fn main() -> ! {
    use stm32g4xx_hal as hal;

    use defmt::info;
    use hal::{
        gpio::{gpioa::PA8, Alternate, GpioExt, AF13},
        hrtim::{
            capture::HrCapture, compare_register::HrCompareRegister, control::HrControltExt,
            external_event, external_event::ToExternalEventSource, output::HrOutput,
            timer::HrTimer, HrPwmAdvExt, Pscl4,
        },
        pwm,
        pwr::PwrExt,
        rcc::{self, RccExt},
        stm32::Peripherals,
    };

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
    let pin_a: PA8<Alternate<AF13>> = gpioa.pa8.into_alternate();

    // ...with a prescaler of 4 this gives us a HrTimer with a tick rate of 960MHz
    // With max the max period set, this would be 960MHz/2^16 ~= 15kHz...
    let prescaler = Pscl4;

    //        .               .
    //        .  50%          .
    //         ------          ------
    //out1    |      |        |      |
    //        |      |        |      |
    // --------      ----------      --------
    //        .    ^     ^
    //        .    |     |
    //AD samlp    pa0   temp
    let period = 0xFFFF;
    let (mut hr_control, _flt_inputs, eev_inputs) =
        dp.HRTIM_COMMON.hr_control(&mut rcc).wait_for_calibration();

    let eev_input3 = eev_inputs
        .eev_input3
        .bind(gpiob.pb7.into_pull_down_input())
        .edge_or_polarity(external_event::EdgeOrPolarity::Polarity(
            pwm::Polarity::ActiveHigh,
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

    let capture = timer.capture_ch1();
    capture.enable_interrupt(true, &mut hr_control);
    capture.add_event(&eev_input3);

    loop {
        if !capture.is_pending() {
            continue;
        }
        capture.clear_interrupt();

        info!("Capture: {:?}", capture.get());
    }
}
