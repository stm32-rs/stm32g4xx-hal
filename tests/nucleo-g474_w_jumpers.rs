#![no_std]
#![no_main]
#![allow(clippy::uninlined_format_args)]

//! Requires jumpers between
//! - A1<->A2 (arduino naming) aka PA1<->PA4
//! - D11<->D12 (arduino naming) aka PA6<->PA7

#[path = "../examples/utils/mod.rs"]
mod utils;

mod common;

#[embedded_test::tests]
mod tests {
    use crate::utils::logger::{info, println};

    use embedded_hal::delay::DelayNs;
    use stm32g4xx_hal::{
        adc::{self, AdcClaim, AdcCommonExt},
        comparator::{self, ComparatorExt, ComparatorSplit},
        dac::{self, DacExt, DacOut},
        delay, gpio,
        opamp::{self, IntoFollower, IntoPga, Opamp1},
        pac,
        prelude::*,
        rcc, spi,
        stasis::Freeze,
        time::RateExtU32,
    };

    use crate::VREF_ADC_BITS;

    type Spi1 = spi::Spi<
        pac::SPI1,
        (
            gpio::gpioa::PA5<gpio::AF5>,
            gpio::gpioa::PA6<gpio::AF5>,
            gpio::gpioa::PA7<gpio::AF5>,
        ),
    >;
    struct Peripherals {
        opamp: opamp::Disabled<Opamp1>,
        comp: comparator::COMP1,
        value_dac: dac::Dac1Ch1<{ dac::M_EXT_PIN }, dac::Enabled>,
        ref_dac: dac::Dac3Ch1<{ dac::M_INT_SIG }, dac::Enabled>,
        adc: adc::Adc<pac::ADC1, adc::Disabled>,
        pa1: gpio::gpioa::PA1<gpio::Analog>,
        pa2: gpio::gpioa::PA2<gpio::Analog>,
        rcc: rcc::Rcc,
        delay: delay::SystDelay,
        spi: Spi1,
        cs: gpio::gpioa::PA8<gpio::Output<gpio::PushPull>>,
    }

    #[init]
    fn init() -> Peripherals {
        //op1+ PA1 -> A1
        //DAC1_OUT1 PA4 -> A2

        let cp = pac::CorePeripherals::take().unwrap();
        let dp = pac::Peripherals::take().unwrap();
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

        let sclk = gpioa.pa5.into_alternate();
        let miso = gpioa.pa6.into_alternate();
        let mosi = gpioa.pa7.into_alternate();

        // 1/8 SPI/SysClk ratio seems to be the upper limit for continuous transmission
        // one byte at a time
        // 1/4 works well when writing two packed bytes at once
        // At 1/2 the clock stays on for ~80% of the time
        let spi = dp
            .SPI1
            .spi((sclk, miso, mosi), spi::MODE_0, 8.MHz(), &mut rcc);
        let mut cs = gpioa.pa8.into_push_pull_output();
        cs.set_high();

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
            spi,
            cs,
        }
    }

    #[test]
    fn dac() {
        use super::*;

        // TODO: Is it ok to steal these?
        let cp = unsafe { pac::CorePeripherals::steal() };
        let dp = unsafe { pac::Peripherals::steal() };
        let mut rcc = dp.RCC.constrain();
        let mut delay = cp.SYST.delay(&rcc.clocks);

        let gpioa = dp.GPIOA.split(&mut rcc);
        let _pa1_important_dont_use_as_output = gpioa.pa1.into_floating_input();
        let pa4 = gpioa.pa4.into_analog();
        let dac1ch1 = dp.DAC1.constrain(pa4, &mut rcc);

        let gpioa = unsafe { &*pac::GPIOA::PTR };

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
    fn opamp_follower_dac_adc(state: Peripherals) {
        let Peripherals {
            opamp,
            mut value_dac,
            mut adc,
            pa1,
            mut delay,
            ..
        } = state;
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
    fn opamp_follower_ext_pin_dac_adc(state: Peripherals) {
        let Peripherals {
            opamp,
            mut value_dac,
            mut adc,
            pa1,
            pa2,
            mut delay,
            ..
        } = state;
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
    fn opamp_pga_dac_adc(state: Peripherals) {
        let Peripherals {
            opamp,
            mut value_dac,
            mut adc,
            pa1,
            mut delay,
            ..
        } = state;
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
    fn comp_dac_vref(state: Peripherals) {
        let Peripherals {
            comp,
            mut value_dac,
            pa1,
            rcc,
            mut delay,
            ..
        } = state;

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
    fn comp_dac_half_vref(state: Peripherals) {
        let Peripherals {
            comp,
            mut value_dac,
            pa1,
            rcc,
            mut delay,
            ..
        } = state;

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
    fn comp_dac_dac(state: Peripherals) {
        let Peripherals {
            comp,
            mut value_dac,
            ref_dac,
            pa1,
            rcc,
            mut delay,
            ..
        } = state;
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

    #[test]
    fn spi_loopback(state: Peripherals) {
        let mut spi = state.spi;
        let mut cs = state.cs;

        // Odd number of bits to test packing edge case
        const MESSAGE: &[u8] = "Hello world, but longer".as_bytes();
        cs.set_low();
        spi.write(MESSAGE).unwrap();
        SpiBus::<u8>::flush(&mut spi).unwrap();
        cs.set_high();

        let received = &mut [0u8; MESSAGE.len()];
        spi.read(received).unwrap();

        cortex_m::asm::delay(100);
        cs.set_low();
        spi.transfer(received, MESSAGE).unwrap();
        // downside of having 8 and 16 bit impls on the same struct is you have to specify which flush
        // impl to call, although internally they call the same function
        SpiBus::<u8>::flush(&mut spi).unwrap();
        cs.set_high();

        info!("Received {:?}", core::str::from_utf8(received).ok());
        assert_eq!(MESSAGE, received);

        cs.set_low();
        spi.transfer_in_place(received).unwrap();
        SpiBus::<u8>::flush(&mut spi).unwrap();
        cs.set_high();

        info!("Received {:?}", core::str::from_utf8(received).ok());
        assert_eq!(MESSAGE, received);

        // Switch between 8 and 16 bit frames on the fly
        const TX_16B: &[u16] = &[0xf00f, 0xfeef, 0xfaaf];
        let mut rx_16b = [0u16; TX_16B.len()];

        cs.set_low();
        spi.transfer(&mut rx_16b, TX_16B).unwrap();
        // internally works the same as SpiBus::<u8>::flush()
        SpiBus::<u16>::flush(&mut spi).unwrap();
        cs.set_high();

        info!("Received {:?}", &rx_16b);
        assert_eq!(TX_16B, rx_16b);

        cs.set_low();
        spi.transfer_in_place(&mut rx_16b).unwrap();
        SpiBus::<u16>::flush(&mut spi).unwrap();
        cs.set_high();

        info!("Received {:?}", &rx_16b);
        assert_eq!(TX_16B, rx_16b);
    }
}

fn is_pax_low(gpioa: &stm32g4xx_hal::pac::gpioa::RegisterBlock, x: u8) -> bool {
    gpioa.idr().read().idr(x).is_low()
}

/// Vrefint = 1.212V typical
/// Vref+ = 3.3V for nucleo
/// 1.212V/3.3V*4095 = 1504 adc value
const VREF_ADC_BITS: u16 = 1504;
