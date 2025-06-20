#![no_std]
#![no_main]

#[path = "../utils/mod.rs"]
mod utils;
use utils::logger::info;

/// Example showcasing the use of the HRTIM peripheral together with a comparator to implement a current fault.
/// Once the comparator input exceeds the reference set by the DAC, the output is forced low and put into a fault state.
use cortex_m_rt::entry;
use fugit::ExtU32 as _;
use stm32_hrtim::{
    compare_register::HrCompareRegister,
    fault::{FaultAction, FaultMonitor},
    output::HrOutput,
    timer::HrTimer,
    HrParts, HrPwmAdvExt, Polarity, Pscl4,
};
use stm32g4xx_hal::{
    self as hal,
    adc::{AdcClaim, AdcCommonExt},
    comparator::{self, ComparatorExt, ComparatorSplit},
    dac::{Dac3IntSig1, DacExt, DacOut},
    delay::{DelayExt as _, SYSTDelayExt},
    gpio::GpioExt,
    hrtim::{fault::FaultInput, HrControltExt, HrPwmBuilderExt},
    pwr::PwrExt,
    rcc::{self, RccExt},
    stasis::Freeze,
    stm32::{CorePeripherals, Peripherals},
};

#[entry]
fn main() -> ! {
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
    let adc12_common = dp
        .ADC12_COMMON
        .claim(hal::adc::config::ClockMode::AdcHclkDiv4, &mut rcc);
    let mut adc1 = adc12_common.claim_and_configure(dp.ADC1, Default::default(), &mut delay);

    let gpioa = dp.GPIOA.split(&mut rcc);
    let gpioc = dp.GPIOC.split(&mut rcc);

    let dac3ch1 = dp.DAC3.constrain(Dac3IntSig1, &mut rcc);
    let mut dac = dac3ch1.enable(&mut rcc);

    // Use dac to define the fault threshold
    // 2^12 / 2 = 2^11 for about half of VCC
    let fault_limit = 60;
    dac.set_value(fault_limit);

    let (_comp1, _comp2, comp3, ..) = dp.COMP.split(&mut rcc);

    let (pc1, [pc1_token]) = gpioc.pc1.into_analog().freeze();
    let comp3 = comp3
        .comparator(
            pc1_token,
            dac,
            comparator::Config::default()
                .hysteresis(comparator::Hysteresis::None)
                .output_inverted(),
            &rcc.clocks,
        )
        .enable();

    let (hr_control, flt_inputs, _) = dp.HRTIM_COMMON.hr_control(&mut rcc).wait_for_calibration();
    let mut hr_control = hr_control.constrain();

    let fault_source5 = flt_inputs
        .fault_input5
        .bind(comp3)
        .polarity(Polarity::ActiveHigh)
        .finalize(&mut hr_control);

    // ...with a prescaler of 4 this gives us a HrTimer with a tick rate of 960MHz
    // With max the max period set, this would be 960MHz/2^16 ~= 15kHz...
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
        out: mut out1,
        ..
    } = dp
        .HRTIM_TIMA
        .pwm_advanced(pin_a)
        .prescaler(prescaler)
        .period(0xFFFF)
        .with_fault_source(fault_source5) // Set fault source
        .fault_action1(FaultAction::ForceInactive)
        .fault_action2(FaultAction::ForceInactive)
        .finalize(&mut hr_control);

    out1.enable_rst_event(&cr1); // Set low on compare match with cr1
    out1.enable_set_event(&timer); // Set high at new period
    cr1.set_duty(timer.get_period() / 3);

    out1.enable();
    timer.start(&mut hr_control.control);

    info!("Started");

    loop {
        for _ in 0..5 {
            delay.delay(500_u32.millis());
            info!(
                "State: {:?}, comp: {}, is_fault_active: _, pc1: {}",
                out1.get_state(),
                //comp3.output(), TODO
                hr_control.fault_5.is_fault_active(),
                adc1.convert(&pc1, hal::adc::config::SampleTime::Cycles_92_5)
            );
        }
        if hr_control.fault_5.is_fault_active() {
            hr_control.fault_5.clear_fault(); // Clear fault every 5s
            out1.enable();
            info!("failt cleared, and output reenabled");
        }
    }
}
