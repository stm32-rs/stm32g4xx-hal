#![no_std]
#![no_main]

// Requires a jumper from A1<->A2 (arduino naming) aka PA1<->PA4

#[path = "../examples/utils/mod.rs"]
mod utils;

mod common;

use core::ops::Sub;

use fugit::{ExtU32, HertzU32, MicrosDurationU32};
use stm32_hrtim::compare_register::HrCompareRegister;
use stm32_hrtim::control::HrPwmControl;
use stm32_hrtim::external_event::{self, ExternalEventSource, ToExternalEventSource};
use stm32_hrtim::output::HrOut1;
use stm32_hrtim::pac::HRTIM_TIMA;
use stm32_hrtim::timer_eev_cfg::{EevCfg, EevCfgs, EventFilter};
use stm32_hrtim::{
    DacResetOnCounterReset, DacStepOnCmp2, HrParts, HrPwmAdvExt, HrtimPrescaler, Pscl128,
};
use stm32g4xx_hal::adc::{self, AdcClaim, AdcCommonExt};
use stm32g4xx_hal::comparator::{self, Comparator, ComparatorExt, ComparatorSplit};
use stm32g4xx_hal::dac::{self, DacExt, DacOut, SawtoothConfig};
use stm32g4xx_hal::delay::{self, SYSTDelayExt};
use stm32g4xx_hal::gpio::{self, GpioExt};
use stm32g4xx_hal::hrtim::external_event::EevInputExt;
use stm32g4xx_hal::hrtim::{HrControltExt, HrPwmBuilderExt};
use stm32g4xx_hal::pwr::PwrExt;
use stm32g4xx_hal::rcc::{self, RccExt};

use hal::stm32;
use stm32g4xx_hal as hal;
use stm32g4xx_hal::stasis::{Freeze, Frozen};

pub const F_SYS: HertzU32 = HertzU32::MHz(120);
pub const CYCLES_PER_US: u32 = F_SYS.raw() / 1_000_000;
type Timer = crate::common::Timer<CYCLES_PER_US>;

pub fn enable_timer(cp: &mut stm32::CorePeripherals) {
    cp.DCB.enable_trace();
    cp.DWT.enable_cycle_counter();
}

pub fn now() -> MicrosDurationU32 {
    (stm32::DWT::cycle_count() / CYCLES_PER_US).micros()
}

#[embedded_test::tests]
mod tests {
    use fugit::ExtU32;
    use stm32_hrtim::{
        compare_register::HrCompareRegister, output::HrOutput, timer::HrTimer, HrtimPrescaler,
        Pscl128,
    };
    use stm32g4xx_hal::{dac::DacOut, stm32::GPIOA};

    use crate::{
        abs_diff, common::{await_hi, await_lo}, now, F_SYS
    };

    ///                | \
    ///                |    \
    /// value_dac ---> | +     \
    ///                |         *----->
    /// ref_dac   ---> | -     /
    ///                |    /
    ///                | /
    ///
    #[test]
    fn hrtim_slope_compensated() {
        type Prescaler = Pscl128;
        type Us = fugit::Duration<u32, 1, 1000_000>;

        let prescaler = Pscl128;
        let period = 0xFFFF;
        let dac_step_per_trigger = 550;
        let cr2_ticks_per_dac_trigger = 1024;

        let crate::Peripherals {
            mut hrtimer,
            mut hr_control,
            eev_input4,
            comp,
            mut value_dac,
            mut ref_dac,
            adc,
            pa2,
            rcc,
            delay,
            timer
        } = super::setup(
            prescaler,
            period,
            dac_step_per_trigger,
            cr2_ticks_per_dac_trigger,
        );
        let pin_num = 8;

        hrtimer.cr1.set_duty(period / 10); // Set blanking window to give the ref_dac time to reset
                                         //out.enable_rst_event(&cr1); // Set low on compare match with cr1
        hrtimer.out.enable_rst_event(&eev_input4); // Set low on compare match with cr1
        hrtimer.out.enable_set_event(&hrtimer.timer); // Set high at new period

        hrtimer.out.enable();
        hrtimer.timer.start(&mut hr_control.control);

        defmt::println!("state: {}", hrtimer.out.get_state());

        //out.enable_rst_event(&eev_input4);

        let gpioa = unsafe { &*GPIOA::PTR };
        let f_pwm = (F_SYS * 32) / (Prescaler::VALUE as u32 * period as u32);
        let period_duration: Us = f_pwm.into_duration();
        defmt::println!("period_duration: {}", period_duration);
        let timeout: Us = 2 * period_duration;

        ref_dac.set_value(4095);
        value_dac.set_value(2048);

        loop {
            let out = comp.output();
            let ref_dac = ref_dac.get_value();
            let value_dac = value_dac.get_value();
            let cnt = hrtimer.timer.get_counter_value();
            defmt::println!(
                "out: {}, ref: {}, val: {}, cnt: {}",
                out,
                ref_dac,
                value_dac,
                cnt
            );
        }

        // TODO: My device seems to have quite a bit of offset error on ref_dac DAC3CH1
        for s_ref in 4..=255 {
            let s_ref = s_ref << 4; // Convert from 8 to 12 bits
            if s_ref & 0xFF == 0 {
                defmt::println!("{}/{}...", s_ref, 4095);
            }

            ref_dac.set_value(s_ref);
            for s_value in 0..=255 {
                let s_value = s_value << 4; // Convert from 8 to 12 bits

                let value_dac_as_u16: u16 = s_value << 4; // Convert from 12 to 16 bits
                let expected_duty = value_dac_as_u16.saturating_sub(s_ref);
                let expected_pw = (timeout * u32::from(expected_duty)) / u32::from(period);

                defmt::debug!("Awaiting first rising edge...");
                value_dac.set_value(s_value);
                loop {
                    let out = comp.output();
                    defmt::println!("out: {}", out);
                }
                let duration_until_lo = await_lo(&timer, pin_num, timeout).unwrap();
                let first_lo_duration = await_hi(&timer, pin_num, timeout);

                let (period, duty) = match first_lo_duration {
                    Ok(lo_duration) => {
                        let duty = await_lo(&timer, pin_num, timeout).unwrap();
                        let period = lo_duration + duty;
                        (period, duty)
                    }
                    Err(_) => (timeout, 0u32.micros()),
                };

                let duty_tolerance: Us = 20.micros();
                let period_tolerance: Us = 20.micros();

                let out = comp.output();
                assert!(
                    abs_diff(expected_pw, duty) < duty_tolerance,
                    "expected_duty: {}, duty: {}",
                    expected_duty,
                    duty,
                );
                assert!(
                    abs_diff(period_duration, period) < period_tolerance,
                    "timeout: {}, period: {}",
                    period_duration,
                    period,
                );
            }
        }
    }
}

fn abs_diff<T: Ord + Sub + Copy>(a: T, b: T) -> T::Output {
    a.max(b) - a.min(b)
}

/// Vrefint = 1.212V typical
/// Vref+ = 3.3V for nucleo
/// 1.212V/3.3V*4095 = 1504 adc value
const VREF_ADC_BITS: u16 = 1504;

struct Peripherals<PSCL> {
    hrtimer:
        HrParts<HRTIM_TIMA, PSCL, HrOut1<HRTIM_TIMA, PSCL, DacResetOnCounterReset, DacStepOnCmp2>, DacResetOnCounterReset, DacStepOnCmp2>,
    hr_control: HrPwmControl,
    eev_input4: ExternalEventSource<4, false>,
    comp: Comparator<comparator::COMP1, comparator::Enabled>,
    value_dac: dac::Dac1Ch1<{ dac::M_EXT_PIN }, dac::Enabled>,
    ref_dac: Frozen<dac::Dac3Ch1<{ dac::M_INT_SIG }, dac::SawtoothGenerator>, 1>,
    adc: adc::Adc<stm32::ADC1, adc::Disabled>,
    pa2: gpio::gpioa::PA2<gpio::Analog>,
    rcc: rcc::Rcc,
    delay: delay::SystDelay,
    timer: Timer,
}

fn setup<PSCL: HrtimPrescaler>(
    prescaler: PSCL,
    period: u16,
    dac_step_per_trigger: u16,
    cr2_ticks_per_dac_trigger: u16,
) -> Peripherals<PSCL> {
    //op1+ PA1 -> A1
    //DAC1_OUT1 PA4 -> A2

    // TODO: Is it ok to steal these?
    let mut cp = stm32::CorePeripherals::take().unwrap();
    let dp = stm32::Peripherals::take().unwrap();
    let timer = Timer::enable_timer(&mut cp);
    // Set system frequency to 16MHz * 15/1/2 = 120MHz
    // This would lead to HrTim running at 120MHz * 32 = 3.84...
    defmt::info!("rcc");
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
    assert_eq!(rcc.clocks.sys_clk, F_SYS);
    enable_timer(&mut cp);
    let mut delay = cp.SYST.delay(&rcc.clocks);

    let adc12_common = dp
        .ADC12_COMMON
        .claim(adc::config::ClockMode::AdcHclkDiv4, &mut rcc);
    let adc = adc12_common.claim(dp.ADC1, &mut delay);

    let gpioa = dp.GPIOA.split(&mut rcc);
    let pa1 = gpioa.pa1.into_analog();
    let pa2 = gpioa.pa2.into_analog();
    let pa4 = gpioa.pa4.into_analog();
    let pa8 = gpioa.pa8;
    let pa9 = gpioa.pa9;

    let dac1ch1 = dp.DAC1.constrain(pa4, &mut rcc);
    let dac3ch1 = dp.DAC3.constrain(dac::Dac3IntSig1, &mut rcc);

    let mut value_dac = dac1ch1.calibrate_buffer(&mut delay).enable(&mut rcc);
    let ref_dac = dac3ch1;
    value_dac.set_value(0);

    let (comp, ..) = dp.COMP.split(&mut rcc);

    type Comparator = comparator::Comparator<comparator::COMP1, comparator::Enabled>;
    type Prescaler = Pscl128;

    let (mut hr_control, _, eev_inputs) =
        dp.HRTIM_COMMON.hr_control(&mut rcc).wait_for_calibration();

    let eev_input4 = eev_inputs
        .eev_input4
        .bind(&unsafe {
            // Safety
            // We will set up the real comparator before eev_input4 is
            // connected to the timer output
            core::mem::transmute::<(), Comparator>(())
        })
        .edge_or_polarity(external_event::EdgeOrPolarity::Polarity(
            stm32_hrtim::Polarity::ActiveHigh,
        ))
        .finalize(&mut hr_control);

    let mut hr_control = hr_control.constrain();
    let eev_cfgs =
        EevCfgs::default().eev4(EevCfg::default().filter(EventFilter::BlankingResetToCmp1));
    let mut hrtimer: HrParts<_, PSCL, HrOut1<_, PSCL, DacResetOnCounterReset, DacStepOnCmp2>, DacResetOnCounterReset, DacStepOnCmp2> = dp
        .HRTIM_TIMA
        .pwm_advanced(pa8)
        .prescaler(prescaler)
        .period(period)
        .dac_trigger_cfg(DacResetOnCounterReset, DacStepOnCmp2)
        .eev_cfg(eev_cfgs)
        .finalize(&mut hr_control);

    hrtimer.cr2.set_duty(cr2_ticks_per_dac_trigger);
    let ref_dac = ref_dac.enable_sawtooth_generator(
        SawtoothConfig::with_slope(
            stm32g4xx_hal::dac::CountingDirection::Decrement,
            dac_step_per_trigger,
        )
        .inc_trigger(&hrtimer.cr2)
        .reset_trigger(&hrtimer.timer),
        &mut rcc,
    );

    let (ref_dac, [comp_ref]) = ref_dac.freeze();
    let comp: Comparator = comp
        .comparator(pa1, comp_ref, comparator::Config::default(), &rcc.clocks)
        .enable();

    Peripherals {
        hrtimer,
        hr_control,
        eev_input4,
        comp,
        value_dac,
        ref_dac,
        adc,
        pa2,
        rcc,
        delay,
        timer,
    }
}
