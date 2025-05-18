#![no_std]
#![no_main]

#[path = "../utils/mod.rs"]
mod utils;
use utils::logger::info;

/// Example showcasing the use of the HRTIM peripheral together with a comparator to implement a cycle by cycle current limit.
/// Once the comparator input exceeds the reference set by the DAC, the output is set low thus limiting the pulse width and in turn the current.
use cortex_m_rt::entry;
use stm32_hrtim::{
    compare_register::HrCompareRegister,
    external_event::{self, ToExternalEventSource},
    output::HrOutput,
    timer::HrTimer,
    timer_eev_cfg::{EevCfg, EevCfgs},
    HrParts, HrPwmAdvExt, Polarity, Pscl4,
};
use stm32g4xx_hal::{
    comparator::{self, ComparatorExt, ComparatorSplit},
    dac::{self, DacExt, DacOut},
    delay::SYSTDelayExt,
    gpio::{GpioExt, SignalEdge},
    hrtim::{external_event::EevInputExt, HrControltExt, HrPwmBuilderExt},
    pwr::PwrExt,
    rcc::{self, RccExt},
    stm32::{CorePeripherals, Peripherals},
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

    let exti = dp.EXTI;

    let mut delay = cp.SYST.delay(&rcc.clocks);

    let gpioa = dp.GPIOA.split(&mut rcc);

    let input = gpioa.pa1.into_analog();
    let pin_a = gpioa.pa8;

    let dac1ch1 = dp.DAC1.constrain(dac::Dac1IntSig1, &mut rcc);
    let mut dac = dac1ch1.calibrate_buffer(&mut delay).enable(&mut rcc);

    // Use dac to define the fault threshold
    // 2^12 / 2 = 2^11 for about half of VCC
    let limit = 1 << 11;
    dac.set_value(limit);

    let (comp1, ..) = dp.COMP.split(&mut rcc);

    let comp1 = comp1.comparator(
        input,
        dac,
        comparator::Config::default().hysteresis(comparator::Hysteresis::None),
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
            Polarity::ActiveHigh,
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
    let HrParts {
        mut timer,
        mut cr1,
        mut out,
        ..
    } = dp
        .HRTIM_TIMA
        .pwm_advanced(pin_a)
        .prescaler(prescaler)
        .eev_cfg(EevCfgs::default().eev4(EevCfg::default()))
        .period(0xFFFF)
        .finalize(&mut hr_control);

    out.enable_rst_event(&cr1); // Set low on compare match with cr1
    out.enable_rst_event(&eev_input4);
    out.enable_set_event(&timer); // Set high at new period
    cr1.set_duty(timer.get_period() / 3);

    out.enable();
    timer.start(&mut hr_control.control);

    info!("Started");

    loop {
        info!(
            "Comp: {}, pending: {}",
            comp1.output(),
            comp1.is_pending(&exti)
        );
    }
}
