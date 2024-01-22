//This example puts the timer in PWM mode using the specified pin with a frequency of 100Hz and a duty cycle of 50%.
#![no_main]
#![no_std]

use cortex_m_rt::entry;

use defmt_rtt as _; // global logger
use panic_probe as _;

#[entry]
fn main() -> ! {
    use hal::comparator;
    use hal::comparator::{ComparatorExt, ComparatorSplit, Hysteresis};
    use hal::dac::{self, DacExt, DacOut};
    use hal::gpio::gpioa::PA8;
    use hal::gpio::Alternate;
    use hal::gpio::SignalEdge;
    use hal::gpio::AF13;
    use hal::hrtim::compare_register::HrCompareRegister;
    use hal::hrtim::external_event::{self, ToExternalEventSource};
    use hal::hrtim::timer::HrTimer;
    use hal::hrtim::timer_eev_cfg::{EevCfg, EevCfgs};
    use hal::hrtim::HrPwmAdvExt;
    use hal::hrtim::Pscl4;
    use hal::hrtim::{control::HrControltExt, output::HrOutput};
    use hal::prelude::*;
    use hal::pwm;
    use hal::pwr::PwrExt;
    use hal::rcc;
    use hal::stm32;
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

    let exti = dp.EXTI;

    let mut delay = cp.SYST.delay(&rcc.clocks);

    let gpioa = dp.GPIOA.split(&mut rcc);

    let input = gpioa.pa1.into_analog();
    let pin_a: PA8<Alternate<AF13>> = gpioa.pa8.into_alternate();

    let dac1ch1 = dp.DAC1.constrain(dac::Dac1IntSig1, &mut rcc);
    let mut dac = dac1ch1.calibrate_buffer(&mut delay).enable();

    // Use dac to define the fault threshold
    // 2^12 / 2 = 2^11 for about half of VCC
    let limit = 1 << 11;
    dac.set_value(limit);

    let (comp1, ..) = dp.COMP.split(&mut rcc);

    let comp1 = comp1.comparator(
        &input,
        &dac,
        comparator::Config::default().hysteresis(Hysteresis::None),
        //.output_inverted(),
        &rcc.clocks,
    );
    comp1.listen(SignalEdge::Rising, &exti);
    let comp1 = comp1.enable().lock();

    let (mut hr_control, _flt_inputs, eev_inputs) =
        dp.HRTIM_COMMON.hr_control(&mut rcc).wait_for_calibration();

    let eev_input4 = eev_inputs
        .eev_input4
        .bind(&comp1)
        .edge_or_polarity(external_event::EdgeOrPolarity::Polarity(
            pwm::Polarity::ActiveHigh,
        ))
        .finalize(&mut hr_control);

    let mut hr_control = hr_control.constrain();

    // ...with a prescaler of 4 this gives us a HrTimer with a tick rate of 1.2GHz
    // With max the max period set, this would be 1.2GHz/2^16 ~= 18kHz...
    let prescaler = Pscl4;

    //        .               .  *            .
    //        .  33%          .  *            .               .               .
    //        .-----.         .--*            .               .-----.         .-----
    //out1    |     |         |  |            .               |     |         |
    //        |     |         |  *            .               |     |         |
    //   ------     -----------  ------------------------------     -----------
    //        .               .  *            .               .               .
    //        .               .  *            .               .               .
    //        .               .  *-------------*              .               .
    //eev     .               .  |            .|              .               .
    //        .               .  |            .|              .               .
    //   -------------------------            .--------------------------------------
    //        .               .  *            .               .               .
    //        .               .  *            .               .               .
    let (mut timer, (mut cr1, _cr2, _cr3, _cr4), mut out1) = dp
        .HRTIM_TIMA
        .pwm_advanced(pin_a, &mut rcc)
        .prescaler(prescaler)
        .eev_cfg(EevCfgs::default().eev4(EevCfg::default()))
        .period(0xFFFF)
        .finalize(&mut hr_control);

    out1.enable_rst_event(&cr1); // Set low on compare match with cr1
    out1.enable_rst_event(eev_input4);
    out1.enable_set_event(&timer); // Set high at new period
    cr1.set_duty(timer.get_period() / 3);

    out1.enable();
    timer.start(&mut hr_control);

    defmt::info!("Started");

    loop {
        defmt::info!(
            "Comp: {}, pending: {}",
            comp1.output(),
            comp1.is_pending(&exti)
        );
    }
}
