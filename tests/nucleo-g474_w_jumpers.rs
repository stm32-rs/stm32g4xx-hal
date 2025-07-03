#![no_std]
#![no_main]
#![allow(clippy::uninlined_format_args)]

// Requires a jumper from A1<->A2 (arduino naming) aka PA1<->PA4

#[path = "../examples/utils/mod.rs"]
mod utils;

use utils::logger::println;

mod common;

use stm32g4xx_hal::adc::{self, AdcClaim, AdcCommonExt};
use stm32g4xx_hal::comparator::{self, ComparatorSplit};
use stm32g4xx_hal::dac::{self, DacExt, DacOut};
use stm32g4xx_hal::delay::{self, SYSTDelayExt};
use stm32g4xx_hal::gpio::{self, GpioExt};
use stm32g4xx_hal::opamp::{self, Opamp1, OpampEx};
use stm32g4xx_hal::rcc::{self, RccExt};

use hal::stm32;
use stm32g4xx_hal as hal;

#[embedded_test::tests]
mod tests {
    use embedded_hal::delay::DelayNs;
    use stm32g4xx_hal::{
        adc::{self},
        comparator::{self, ComparatorExt},
        dac::DacOut,
        opamp::{self, IntoFollower, IntoPga},
        stasis::Freeze,
    };

    use crate::VREF_ADC_BITS;

    #[test]
    fn dac() {
        use super::*;

        // TODO: Is it ok to steal these?
        let cp = unsafe { stm32::CorePeripherals::steal() };
        let dp = unsafe { stm32::Peripherals::steal() };
        let mut rcc = dp.RCC.constrain();
        let mut delay = cp.SYST.delay(&rcc.clocks);

        let gpioa = dp.GPIOA.split(&mut rcc);
        let _pa1_important_dont_use_as_output = gpioa.pa1.into_floating_input();
        let pa4 = gpioa.pa4.into_analog();
        let dac1ch1 = dp.DAC1.constrain(pa4, &mut rcc);

        let gpioa = unsafe { &*stm32::GPIOA::PTR };

        // dac_manual will have its value set manually
        let mut dac = dac1ch1.calibrate_buffer(&mut delay).enable(&mut rcc);

        dac.set_value(0);
        delay.delay_ms(1);
        assert!(is_pax_low(gpioa, 1));

        dac.set_value(4095);
        delay.delay_ms(1);
        assert!(!is_pax_low(gpioa, 1));
    }

    #[test]
    fn opamp_follower_dac_adc() {
        let super::Peripherals {
            opamp,
            mut value_dac,
            mut adc,
            pa1,
            mut delay,
            ..
        } = super::setup_opamp_comp_dac();
        let opamp = opamp.follower(pa1);
        delay.delay_ms(10);

        let sample_time = adc::config::SampleTime::Cycles_640_5;

        // TODO: Seems I can not go lower than 50 with my device, offset error of opamp?
        for setpoint in [50, 100, 500, 1000, 2000, 4095] {
            value_dac.set_value(setpoint);
            delay.delay_ms(1);
            let reading = adc.convert(&opamp, sample_time);
            assert!(
                reading.abs_diff(setpoint) < 10,
                "reading: {reading}, setpoint {setpoint}"
            );
        }
    }

    #[test]
    fn opamp_follower_ext_pin_dac_adc() {
        let super::Peripherals {
            opamp,
            mut value_dac,
            mut adc,
            pa1,
            pa2,
            mut delay,
            ..
        } = super::setup_opamp_comp_dac();
        let (pa2, [pa2_token]) = pa2.freeze();
        let _opamp = opamp.follower(pa1).enable_output(pa2_token);
        delay.delay_ms(10);

        let sample_time = adc::config::SampleTime::Cycles_640_5;

        // TODO: Seems I can not go lower than 50 with my device, offset error of opamp?
        for setpoint in [50, 100, 500, 1000, 2000, 4095] {
            value_dac.set_value(setpoint);
            delay.delay_ms(1);
            let reading = adc.convert(&pa2, sample_time);
            assert!(
                reading.abs_diff(setpoint) < 10,
                "reading: {reading}, setpoint {setpoint}"
            );
        }
    }

    #[test]
    fn opamp_pga_dac_adc() {
        let super::Peripherals {
            opamp,
            mut value_dac,
            mut adc,
            pa1,
            mut delay,
            ..
        } = super::setup_opamp_comp_dac();
        let opamp = opamp.pga(pa1, opamp::Gain::Gain2);

        let sample_time = adc::config::SampleTime::Cycles_640_5;

        for setpoint in [50, 100, 500, 1000, 2000, 4095] {
            value_dac.set_value(setpoint);
            delay.delay_ms(1);
            let reading = adc.convert(&opamp, sample_time);
            assert!(
                reading.abs_diff((setpoint * 2).min(4095)) < 20,
                "reading: {reading}, setpoint {setpoint}"
            );
        }
    }

    ///            | \
    ///            |    \
    ///   dac ---> | +     \
    ///            |         *----->
    /// Vref--->   | -     /
    ///            |    /
    ///            | /
    ///
    #[test]
    fn comp_dac_vref() {
        let super::Peripherals {
            comp,
            mut value_dac,
            pa1,
            rcc,
            mut delay,
            ..
        } = super::setup_opamp_comp_dac();

        let r = comparator::refint_input::VRefint;
        let ref_setpoint = VREF_ADC_BITS;

        let comp = comp
            .comparator(pa1, r, comparator::Config::default(), &rcc.clocks)
            .enable();

        for value_setpoint in [ref_setpoint - 40, ref_setpoint + 40] {
            value_dac.set_value(value_setpoint);
            delay.delay_ms(1);
            let out = comp.output();
            assert!(
                comp.output() == (value_setpoint > ref_setpoint),
                "setpoint: {value_setpoint}, expected: '{}', got '{}'",
                if value_setpoint > ref_setpoint {
                    "HI"
                } else {
                    "LO"
                },
                if out { "HI" } else { "LO" }
            );
        }
    }

    ///            | \
    ///            |    \
    ///   dac ---> | +     \
    ///            |         *----->
    /// Vref/2---> | -     /
    ///            |    /
    ///            | /
    ///
    #[test]
    fn comp_dac_half_vref() {
        let super::Peripherals {
            comp,
            mut value_dac,
            pa1,
            rcc,
            mut delay,
            ..
        } = super::setup_opamp_comp_dac();

        let r = comparator::refint_input::VRefintM12;
        let ref_setpoint = VREF_ADC_BITS / 2;

        let comp = comp
            .comparator(pa1, r, comparator::Config::default(), &rcc.clocks)
            .enable();

        for value_setpoint in [ref_setpoint - 40, ref_setpoint + 40] {
            value_dac.set_value(value_setpoint);
            delay.delay_ms(1);
            let out = comp.output();
            assert!(
                comp.output() == (value_setpoint > ref_setpoint),
                "setpoint: {value_setpoint}, expected: '{}', got '{}'",
                if value_setpoint > ref_setpoint {
                    "HI"
                } else {
                    "LO"
                },
                if out { "HI" } else { "LO" }
            );
        }
    }

    ///                | \
    ///                |    \
    /// value_dac ---> | +     \
    ///                |         *----->
    /// ref_dac   ---> | -     /
    ///                |    /
    ///                | /
    ///
    #[test]
    fn comp_dac_dac() {
        use super::*;
        let super::Peripherals {
            comp,
            mut value_dac,
            ref_dac,
            pa1,
            rcc,
            mut delay,
            ..
        } = super::setup_opamp_comp_dac();
        let (mut ref_dac, [comp_ref]) = ref_dac.freeze();
        let comp = comp
            .comparator(pa1, comp_ref, comparator::Config::default(), &rcc.clocks)
            .enable();

        // TODO: My device seems to have quite a bit of offset error on ref_dac DAC3CH1
        for s_ref in 4..=255 {
            let s_ref = s_ref << 4; // Convert from 8 to 12 bits
            if s_ref & 0xFF == 0 {
                println!("{}/4095...", s_ref);
            }
            ref_dac.set_value(s_ref);
            for s_value in 0..=255 {
                let s_value = s_value << 4; // Convert from 8 to 12 bits
                value_dac.set_value(s_value);
                delay.delay_us(50);
                let out = comp.output();
                if s_value.abs_diff(s_ref) > 20 {
                    assert!(
                        out == (s_value > s_ref),
                        "s_value: {s_value}, s_ref: {s_ref}, out: {out}"
                    );
                }
            }
        }
    }
}

fn is_pax_low(gpioa: &stm32::gpioa::RegisterBlock, x: u8) -> bool {
    gpioa.idr().read().idr(x).is_low()
}

/// Vrefint = 1.212V typical
/// Vref+ = 3.3V for nucleo
/// 1.212V/3.3V*4095 = 1504 adc value
const VREF_ADC_BITS: u16 = 1504;

struct Peripherals {
    opamp: opamp::Disabled<Opamp1>,
    comp: comparator::COMP1,
    value_dac: dac::Dac1Ch1<{ dac::M_EXT_PIN }, dac::Enabled>,
    ref_dac: dac::Dac3Ch1<{ dac::M_INT_SIG }, dac::Enabled>,
    adc: adc::Adc<stm32::ADC1, adc::Disabled>,
    pa1: gpio::gpioa::PA1<gpio::Analog>,
    pa2: gpio::gpioa::PA2<gpio::Analog>,
    rcc: rcc::Rcc,
    delay: delay::SystDelay,
}

fn setup_opamp_comp_dac() -> Peripherals {
    //op1+ PA1 -> A1
    //DAC1_OUT1 PA4 -> A2

    let cp = stm32::CorePeripherals::take().unwrap();
    let dp = stm32::Peripherals::take().unwrap();
    let mut rcc = dp.RCC.constrain();
    let mut delay = cp.SYST.delay(&rcc.clocks);

    let adc12_common = dp.ADC12_COMMON.claim(Default::default(), &mut rcc);
    let adc = adc12_common.claim(dp.ADC1, &mut delay);

    let gpioa = dp.GPIOA.split(&mut rcc);
    let pa1 = gpioa.pa1.into_analog();
    let pa2 = gpioa.pa2.into_analog();
    let pa4 = gpioa.pa4.into_analog();

    let dac1ch1 = dp.DAC1.constrain(pa4, &mut rcc);
    let dac3ch1 = dp.DAC3.constrain(dac::Dac3IntSig1, &mut rcc);

    let mut value_dac = dac1ch1.calibrate_buffer(&mut delay).enable(&mut rcc);
    let mut ref_dac = dac3ch1.enable(&mut rcc); // TODO: should calibrate_buffer be available on dacs without outputs?
    value_dac.set_value(0);
    ref_dac.set_value(0);

    let (opamp, ..) = dp.OPAMP.split(&mut rcc);
    let (comp, ..) = dp.COMP.split(&mut rcc);
    Peripherals {
        opamp,
        comp,
        value_dac,
        ref_dac,
        adc,
        pa1,
        pa2,
        rcc,
        delay,
    }
}
