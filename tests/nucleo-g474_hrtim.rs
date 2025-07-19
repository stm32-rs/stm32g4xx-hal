#![no_std]
#![no_main]

// Requires a jumper from A1<->A2 (arduino naming) aka PA1<->PA4

#[path = "../examples/utils/mod.rs"]
mod utils;

use utils::logger::debug;

mod common;

use common::test_pwm;
use fugit::{ExtU32, HertzU32, MicrosDurationU32};
use stm32_hrtim::{
    compare_register::HrCompareRegister,
    control::HrPwmControl,
    deadtime::DeadtimeConfig,
    output::{HrOutput, ToHrOut},
    timer::HrTimer,
    DacResetOnCounterReset, DacStepOnCmp2, HrParts, HrPwmAdvExt as _, Pscl64,
};
use stm32g4xx_hal::{
    delay::SYSTDelayExt,
    gpio::{GpioExt, PinExt},
    hrtim::{HrControltExt, HrtimPin},
    pwr::PwrExt,
    rcc::{self, Rcc, RccExt},
};

use hal::stm32;
use stm32g4xx_hal as hal;

const PERIOD: u16 = 60_000;
const F_SYS: HertzU32 = HertzU32::MHz(120);
const CYCLES_PER_US: u32 = F_SYS.raw() / 1_000_000;
type Timer = crate::common::Timer<CYCLES_PER_US>;

#[embedded_test::tests]
mod tests {
    use embedded_hal::delay::DelayNs;
    use stm32g4xx_hal::{
        adc::{self, temperature::Temperature, AdcClaim, AdcCommonExt},
        dac::{self, DacExt, DacOut, SawtoothConfig},
    };

    #[test]
    fn simple() {
        use super::*;

        let mut cp = stm32::CorePeripherals::take().unwrap();
        let dp = stm32::Peripherals::take().unwrap();
        let timer = Timer::enable_timer(&mut cp);

        // Set system frequency to 16MHz * 15/1/2 = 120MHz
        // This would lead to HrTim running at 120MHz * 32 = 3.84...
        let mut rcc = setup_rcc_120MHz(dp.PWR, dp.RCC);
        assert_eq!(rcc.clocks.sys_clk, F_SYS);
        let mut delay = cp.SYST.delay(&rcc.clocks);

        let gpioa = dp.GPIOA.split(&mut rcc);
        let _pa1_important_dont_use_as_output = gpioa.pa1.into_floating_input();
        let pin = gpioa.pa8;
        let pin_num = pin.pin_id();

        let (
            HrParts {
                timer: mut hrtimer,
                mut cr1,
                mut out,
                ..
            },
            mut hr_control,
        ) = setup_hrtim(pin, None, dp.HRTIM_TIMA, dp.HRTIM_COMMON, &mut rcc);

        cr1.set_duty(PERIOD / 2);
        out.enable_rst_event(&cr1); // Set low on compare match with cr1
        out.enable_set_event(&hrtimer); // Set high at new period
        out.enable();
        hrtimer.start(&mut hr_control.control);

        let t_hi = 500.micros();
        let t_lo = 500.micros();
        let t_max_deviation = 2.micros();
        test_pwm(&timer, pin_num, t_lo, t_hi, t_max_deviation, 10);

        delay.delay_ms(20); // Give the host some time to read logging messages
    }

    #[test]
    fn deadtime_68us_68us() {
        use super::*;

        deadtime_test(68, 68);
    }

    #[test]
    fn deadtime_68us_1us() {
        use super::*;

        deadtime_test(68, 1);
    }

    #[test]
    fn deadtime_1us_68us() {
        use super::*;

        deadtime_test(1, 68);
    }

    /// Test adc trigger and DAC trigger
    ///
    /// Setup hrtim with DAC to generate a Sawtooth waveform. Use a compare register to accurately
    /// place the measurement point.
    ///
    /// |          /|          /|          /|
    /// |    *   /  |        /  |        /  |
    /// |    * /    |      /    |      /    |
    /// |    X      |    /      |    /      |
    /// |  / *      |  /        |  /        |
    /// |/   *      |/          |/          |
    /// |--->*         
    ///     cr3 indicates where in waveform to trigger ADC
    ///
    #[test]
    fn adc_trigger() {
        use super::*;

        let mut cp = stm32::CorePeripherals::take().unwrap();
        let dp = stm32::Peripherals::take().unwrap();

        // Set system frequency to 16MHz * 15/1/2 = 120MHz
        // This would lead to HrTim running at 120MHz * 32 = 3.84...
        let mut rcc = setup_rcc_120MHz(dp.PWR, dp.RCC);
        assert_eq!(rcc.clocks.sys_clk, F_SYS);
        let mut delay = cp.SYST.delay(&rcc.clocks);

        let gpioa = dp.GPIOA.split(&mut rcc);
        let pa1 = gpioa.pa1.into_analog();
        let pa4 = gpioa.pa4;
        let pin = gpioa.pa8;

        let (mut hrtimer, mut hr_control) =
            setup_hrtim(pin, None, dp.HRTIM_TIMA, dp.HRTIM_COMMON, &mut rcc);

        hrtimer.cr1.set_duty(PERIOD / 2);
        hrtimer.out.enable_rst_event(&hrtimer.cr1); // Set low on compare match with cr1
        hrtimer.out.enable_set_event(&hrtimer.timer); // Set high at new period

        let dac_step_per_trigger = 16;
        let cr2_ticks_per_dac_trigger = dac_step_per_trigger;
        hrtimer.cr2.set_duty(cr2_ticks_per_dac_trigger);
        let dac1ch1 = dp.DAC1.constrain(pa4, &mut rcc);
        let mut ref_dac = dac1ch1.enable_sawtooth_generator(
            SawtoothConfig::with_slope(dac::CountingDirection::Decrement, dac_step_per_trigger)
                .inc_trigger(&hrtimer.cr2)
                .reset_trigger(&hrtimer.timer),
            &mut rcc,
        );

        hr_control.adc_trigger1.enable_source(&hrtimer.cr3);

        defmt::debug!("Setup Adc1");
        let mut adc12_common = dp
            .ADC12_COMMON
            .claim(adc::config::ClockMode::AdcHclkDiv4, &mut rcc);
        let mut adc = adc12_common.claim(dp.ADC1, &mut delay);

        adc.set_external_trigger((
            adc::config::TriggerMode::RisingEdge,
            (&hr_control.adc_trigger1).into(),
        ));
        adc12_common.enable_temperature();
        adc.set_continuous(hal::adc::config::Continuous::Discontinuous);
        adc.reset_sequence();
        adc.configure_channel(
            &pa1,
            adc::config::Sequence::One,
            adc::config::SampleTime::Cycles_12_5,
        );
        let mut adc = adc.enable().into_dynamic_adc();
        hrtimer.timer.start(&mut hr_control.control);

        ref_dac.set_value(4095);

        defmt::debug!("Awaiting first dac rst trigger");
        while ref_dac.get_value() == 0 {}

        //out.enable();

        adc.start_conversion();

        for _ in 0..2 {
            let margin = 256;
            for sampling_point in (margin..(PERIOD - margin)).step_by(16) {
                hrtimer.cr3.set_duty(sampling_point); // Set sampling point
                let sampling_point_as_12bit = sampling_point >> 4;

                // Dac generates a sawtooth with negative slope which starts at 4095 every new period
                let expected_dac_value = 4095 - sampling_point_as_12bit;
                adc.wait_for_conversion_sequence();
                let adc_value = adc.current_sample();
                let diff = adc_value.abs_diff(expected_dac_value);
                assert!(diff < 20);
            }
        }
    }
}

#[allow(non_snake_case)]
fn setup_rcc_120MHz(pwr: stm32::PWR, rcc: stm32::RCC) -> Rcc {
    debug!("rcc");
    // Set system frequency to 16MHz * 15/1/2 = 120MHz
    // This would lead to HrTim running at 120MHz * 32 = 3.84...
    let pwr = pwr.constrain().freeze();
    rcc.freeze(
        rcc::Config::pll().pll_cfg(rcc::PllConfig {
            mux: rcc::PllSrc::HSI,
            n: rcc::PllNMul::MUL_15,
            m: rcc::PllMDiv::DIV_1,
            r: Some(rcc::PllRDiv::DIV_2),

            ..Default::default()
        }),
        pwr,
    )
}

fn setup_hrtim<P>(
    pins: P,
    deadtime_cfg: Option<DeadtimeConfig>,
    hrtim_tima: stm32::HRTIM_TIMA,
    hrtim_common: stm32::HRTIM_COMMON,
    rcc: &mut Rcc,
) -> (
    HrParts<
        stm32::HRTIM_TIMA,
        Pscl64,
        <P as ToHrOut<stm32::HRTIM_TIMA, DacResetOnCounterReset, DacStepOnCmp2>>::Out<Pscl64>,
        DacResetOnCounterReset,
        DacStepOnCmp2,
    >,
    HrPwmControl,
)
where
    P: HrtimPin<stm32::HRTIM_TIMA>
        + ToHrOut<stm32::HRTIM_TIMA, DacResetOnCounterReset, DacStepOnCmp2>,
{
    use stm32g4xx_hal::hrtim::HrPwmBuilderExt;
    let (hr_control, ..) = hrtim_common.hr_control(rcc).wait_for_calibration();
    let mut hr_control = hr_control.constrain();

    // ...with a prescaler of 64 this gives us a HrTimer with a tick rate of 60MHz
    // With a period of 60_000 set, this would be 60MHz/60_000 = 1kHz
    let prescaler = Pscl64;

    let mut hr_tim_builder = hrtim_tima
        .pwm_advanced(pins)
        .prescaler(prescaler)
        .period(PERIOD);

    if let Some(dt) = deadtime_cfg {
        hr_tim_builder = hr_tim_builder.deadtime(dt);
    }
    (
        hr_tim_builder
            .dac_trigger_cfg(DacResetOnCounterReset, DacStepOnCmp2)
            .finalize(&mut hr_control),
        hr_control,
    )
}

fn deadtime_test(deadtime_rising_us: u32, deadtime_falling_us: u32) {
    let mut cp = stm32::CorePeripherals::take().unwrap();
    let dp = stm32::Peripherals::take().unwrap();
    let timer = Timer::enable_timer(&mut cp);

    let deadtime_cfg = DeadtimeConfig::default()
        .prescaler(stm32_hrtim::deadtime::DeadtimePrescaler::ThrtimMul16)
        .deadtime_rising_value(
            (deadtime_rising_us * F_SYS.to_MHz() / 16)
                .try_into()
                .unwrap(),
        ) // ~(1/120MHz)*16*500 ~= 67us
        .deadtime_falling_value(
            (deadtime_falling_us * F_SYS.to_MHz() / 16)
                .try_into()
                .unwrap(),
        );

    let mut rcc = setup_rcc_120MHz(dp.PWR, dp.RCC);
    assert_eq!(rcc.clocks.sys_clk, F_SYS);

    let gpioa = dp.GPIOA.split(&mut rcc);
    let _pa1_important_dont_use_as_output = gpioa.pa1.into_floating_input();
    let pin = gpioa.pa8;
    let pin_num = pin.pin_id();
    let complementary_pin = gpioa.pa9;
    let complementary_pin_num = complementary_pin.pin_id();

    let (
        HrParts {
            timer: mut hrtimer,
            mut cr1,
            out: (mut out1, mut out2),
            ..
        },
        mut hr_control,
    ) = setup_hrtim(
        (pin, complementary_pin),
        Some(deadtime_cfg),
        dp.HRTIM_TIMA,
        dp.HRTIM_COMMON,
        &mut rcc,
    );

    cr1.set_duty(PERIOD / 2);
    out1.enable_rst_event(&cr1); // Set low on compare match with cr1
    out1.enable_set_event(&hrtimer); // Set high at new period
    out1.enable();
    out2.enable();
    hrtimer.start(&mut hr_control.control);

    let t_max_deviation = 2.micros();
    {
        let t_hi = (500 - deadtime_rising_us).micros();
        let t_lo = (500 + deadtime_rising_us).micros();
        test_pwm(&timer, pin_num, t_lo, t_hi, t_max_deviation, 10);
    }

    {
        let t_hi = (500 - deadtime_falling_us).micros();
        let t_lo = (500 + deadtime_falling_us).micros();
        test_pwm(
            &timer,
            complementary_pin_num,
            t_lo,
            t_hi,
            t_max_deviation,
            10,
        );
    }

    let gpioa = unsafe { &*stm32::GPIOA::PTR };

    // Check that both output are not active at the same time
    let before = timer.now();
    while (timer.now() - before) < MicrosDurationU32::millis(1000) {
        let idr = gpioa.idr().read();

        let p = idr.idr(pin_num).is_high();
        let compl_p = idr.idr(complementary_pin_num).is_high();
        assert!(!(p && compl_p), "Both outputs active at the same time");
    }
}
