#![no_std]
#![no_main]

#[path = "../examples/utils/mod.rs"]
mod utils;

use utils::logger::debug;

use core::ops::FnMut;
use core::result::Result;
use fugit::{ExtU32, HertzU32, MicrosDurationU32};
use hal::delay::DelayExt;
use hal::stm32;
use stm32g4xx_hal as hal;

pub const F_SYS: HertzU32 = HertzU32::MHz(16);
pub const CYCLES_PER_US: u32 = F_SYS.raw() / 1_000_000;

pub fn enable_timer(cp: &mut stm32::CorePeripherals) {
    cp.DCB.enable_trace();
    cp.DWT.enable_cycle_counter();
}

pub fn now() -> MicrosDurationU32 {
    (stm32::DWT::cycle_count() / CYCLES_PER_US).micros()
}

#[defmt_test::tests]
mod tests {
    use embedded_hal::{delay::DelayNs, pwm::SetDutyCycle};
    use fixed::types::I1F15;
    use fugit::RateExtU32;
    use stm32g4xx_hal::{
        adc::{self, temperature::Temperature, AdcClaim, AdcCommonExt, Vref},
        cordic::{
            op::{dynamic::Mode, Magnitude, SinCos, Sqrt},
            prec::P60,
            scale::N0,
            types::{Q15, Q31},
            Ext,
        },
        dac::{DacExt, DacOut},
        delay::SYSTDelayExt,
        gpio::{GpioExt, AF6, PA8},
        pwm::PwmExt,
        rcc::RccExt,
        signature::{VrefCal, VDDA_CALIB},
        stm32::GPIOA,
    };

    #[test]
    fn gpio_push_pull() {
        use super::*;

        // TODO: Is it ok to steal these?
        let cp = unsafe { stm32::CorePeripherals::steal() };
        let dp = unsafe { stm32::Peripherals::steal() };
        let mut rcc = dp.RCC.constrain();
        let mut delay = cp.SYST.delay(&rcc.clocks);
        defmt::dbg!(rcc.clocks.sys_clk);

        let gpioa = dp.GPIOA.split(&mut rcc);
        let _pa1_important_dont_use_as_output = gpioa.pa1.into_floating_input();
        let mut pin = gpioa.pa8.into_push_pull_output();

        pin.set_high();
        delay.delay(1.millis()); // Give the pin plenty of time to go high
        assert!(pin.is_high());
        {
            let gpioa = unsafe { &*GPIOA::PTR };
            assert!(!is_pax_low(gpioa, 8));
        }

        pin.set_low();
        delay.delay(1.millis()); // Give the pin plenty of time to go low
        assert!(pin.is_low());
        {
            let gpioa = unsafe { &*GPIOA::PTR };
            assert!(is_pax_low(gpioa, 8));
        }
    }

    #[test]
    fn gpio_open_drain() {
        use super::*;

        // TODO: Is it ok to steal these?
        let cp = unsafe { stm32::CorePeripherals::steal() };
        let dp = unsafe { stm32::Peripherals::steal() };
        let mut rcc = dp.RCC.constrain();
        let mut delay = cp.SYST.delay(&rcc.clocks);

        let gpioa = dp.GPIOA.split(&mut rcc);
        let _pa1_important_dont_use_as_output = gpioa.pa1.into_floating_input();
        let mut pin = gpioa.pa8.into_open_drain_output();

        // Enable pull-up resistor
        {
            let gpioa = unsafe { &*GPIOA::PTR };
            gpioa.pupdr().modify(|_, w| w.pupdr8().pull_up());
        }

        pin.set_high();
        delay.delay(1.millis()); // Give the pin plenty of time to go high
        assert!(pin.is_high());
        {
            let gpioa = unsafe { &*GPIOA::PTR };
            assert!(!is_pax_low(gpioa, 8));
        }

        pin.set_low();
        delay.delay(1.millis()); // Give the pin plenty of time to go low
        assert!(pin.is_low());
        {
            let gpioa = unsafe { &*GPIOA::PTR };
            assert!(is_pax_low(gpioa, 8));
        }
    }

    #[test]
    fn pwm() {
        use super::*;

        // TODO: Is it ok to steal these?
        let mut cp = unsafe { stm32::CorePeripherals::steal() };
        let dp = unsafe { stm32::Peripherals::steal() };
        enable_timer(&mut cp);

        let mut rcc = dp.RCC.constrain();
        assert_eq!(rcc.clocks.sys_clk, F_SYS);

        let gpioa = dp.GPIOA.split(&mut rcc);
        let _pa1_important_dont_use_as_output = gpioa.pa1.into_floating_input();
        let pin: PA8<AF6> = gpioa.pa8.into_alternate();

        let mut pwm = dp.TIM1.pwm(pin, 1000u32.Hz(), &mut rcc);

        pwm.set_duty_cycle_percent(50).unwrap();
        pwm.enable();

        let gpioa = unsafe { &*GPIOA::PTR };

        let min: MicrosDurationU32 = 495u32.micros();
        let max: MicrosDurationU32 = 505u32.micros();

        debug!("Awaiting first rising edge...");
        let duration_until_lo = await_lo(gpioa, max).unwrap();
        let first_lo_duration = await_hi(gpioa, max).unwrap();

        let mut hi_duration = 0.micros();
        let mut lo_duration = 0.micros();

        for _ in 0..10 {
            // Make sure the timer half periods are within 495-505us

            hi_duration = await_lo(gpioa, max).unwrap();
            assert!(
                hi_duration > min && hi_duration < max,
                "hi: {} < {} < {}",
                min,
                hi_duration,
                max
            );

            lo_duration = await_hi(gpioa, max).unwrap();
            assert!(
                lo_duration > min && lo_duration < max,
                "lo: {} < {} < {}",
                min,
                lo_duration,
                max
            );
        }

        // Prints deferred until here to not mess up timing
        debug!("Waited ~{} until low", duration_until_lo);
        debug!("First low half period: {}", first_lo_duration);

        debug!("High half period: {}", hi_duration);
        debug!("Low half period: {}", lo_duration);

        debug!("Done!");

        pwm.disable();
    }

    #[test]
    fn cordic() {
        fn is_almost_equals(a: f32, b: f32) -> bool {
            (a - b).abs() < 0.001
        }

        use super::*;

        let dp = unsafe { stm32::Peripherals::steal() };
        let mut rcc = dp.RCC.constrain();

        let mut cordic = dp
            .CORDIC
            .constrain(&mut rcc)
            .freeze::<Q15, Q31, P60, SinCos>(); // 16 bit arguments, 32 bit results, compute sine and cosine, 60 iterations

        // static operation (zero overhead)

        cordic.start(I1F15::from_num(-0.25 /* -45 degreees */));

        let (sin, cos) = cordic.result();

        debug!("sin: {}, cos: {}", sin.to_num::<f32>(), cos.to_num::<f32>());
        assert!(is_almost_equals(sin.to_num::<f32>(), -0.707));
        assert!(is_almost_equals(cos.to_num::<f32>(), 0.707));

        // dynamic operation

        let mut cordic = cordic.into_dynamic();

        let sqrt = cordic.run::<Sqrt<N0>>(I1F15::from_num(0.25));
        debug!("sqrt: {}", sqrt.to_num::<f32>());
        assert!(is_almost_equals(sqrt.to_num::<f32>(), 0.5));
        let magnitude = cordic.run::<Magnitude>((I1F15::from_num(0.25), I1F15::from_num(0.5)));
        debug!("magnitude: {}", magnitude.to_num::<f32>());
        assert!(is_almost_equals(magnitude.to_num::<f32>(), 0.559));
    }

    #[test]
    fn adc() {
        use super::*;

        // TODO: Is it ok to steal these?
        let cp = unsafe { stm32::CorePeripherals::steal() };
        let dp = unsafe { stm32::Peripherals::steal() };
        let mut rcc = dp.RCC.constrain();
        let mut delay = cp.SYST.delay(&rcc.clocks);

        let mut adc12_common = dp.ADC12_COMMON.claim(Default::default(), &mut rcc);
        let mut adc = adc12_common.claim(dp.ADC1, &mut delay);

        adc12_common.enable_temperature();
        adc12_common.enable_vref();
        let sample_time = adc::config::SampleTime::Cycles_640_5;

        let vref = adc.convert(&Vref, sample_time);
        let vref_cal = VrefCal::get().read();
        let vdda = VDDA_CALIB * vref_cal as u32 / vref as u32;
        debug!("vdda: {}mV", vdda);
        assert!((3200..3400).contains(&vdda));

        let vref = Vref::sample_to_millivolts_ext(vref, vdda, adc::config::Resolution::Twelve);
        debug!("vref: {}mV", vref);
        assert!((1182..1232).contains(&vref)); // From G474 datasheet

        let temperature_reading = adc.convert(&Temperature, sample_time);
        let temp = Temperature::temperature_to_degrees_centigrade(
            temperature_reading,
            vdda as f32 / 1000.,
            adc::config::Resolution::Twelve,
        );
        debug!("temp: {}°C", temp);
        assert!((20.0..35.0).contains(&temp), "20.0 < {} < 35.0", temp);
    }

    #[test]
    fn adc_dma() {
        use super::*;
        use hal::{
            adc,
            dma::{channel::DMAExt, config::DmaConfig, TransferExt},
        };

        // TODO: Is it ok to steal these?
        let cp = unsafe { stm32::CorePeripherals::steal() };
        let dp = unsafe { stm32::Peripherals::steal() };
        let mut rcc = dp.RCC.constrain();
        let mut delay = cp.SYST.delay(&rcc.clocks);

        let channels = dp.DMA1.split(&rcc);
        let config = DmaConfig::default()
            .transfer_complete_interrupt(false)
            .circular_buffer(false)
            .memory_increment(true);

        let mut adc12_common = dp.ADC12_COMMON.claim(Default::default(), &mut rcc);
        let mut adc = adc12_common.claim(dp.ADC1, &mut delay);

        let sample_time = adc::config::SampleTime::Cycles_640_5;

        adc12_common.enable_vref();
        adc12_common.enable_temperature();
        adc.set_continuous(adc::config::Continuous::Single);
        adc.reset_sequence();
        adc.configure_channel(&Vref, adc::config::Sequence::One, sample_time);
        adc.configure_channel(&Temperature, adc::config::Sequence::Two, sample_time);

        defmt::info!("Setup DMA");
        let first_buffer = cortex_m::singleton!(: [u16; 2] = [0; 2]).unwrap();
        let mut transfer = channels.ch1.into_peripheral_to_memory_transfer(
            adc.enable_dma(adc::config::Dma::Single),
            &mut first_buffer[..],
            config,
        );

        transfer.start(|adc| adc.start_conversion());

        defmt::info!("Wait for Conversion");
        while !transfer.get_transfer_complete_flag() {}
        defmt::info!("Conversion Done");

        transfer.pause(|adc| adc.cancel_conversion());
        let (_ch1, adc, first_buffer) = transfer.free();
        let _adc = adc.disable();

        let vref_reading = first_buffer[0];
        let temperature_reading = first_buffer[1];

        let vref_cal = VrefCal::get().read();
        let vdda = VDDA_CALIB * vref_cal as u32 / vref_reading as u32;
        debug!("vdda: {}mV", vdda);
        assert!((3200..3400).contains(&vdda));

        let vref =
            Vref::sample_to_millivolts_ext(vref_reading, vdda, adc::config::Resolution::Twelve);
        debug!("vref: {}mV", vref);
        assert!((1182..1232).contains(&vref)); // From G474 datasheet

        let temp = Temperature::temperature_to_degrees_centigrade(
            temperature_reading,
            vdda as f32 / 1000.,
            adc::config::Resolution::Twelve,
        );
        debug!("temp: {}°C", temp);
        assert!((20.0..35.0).contains(&temp), "20.0 < {} < 35.0", temp);
    }

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
        let pa4 = gpioa.pa4.into_floating_input();
        let dac1ch1 = dp.DAC1.constrain(pa4, &mut rcc);

        let gpioa = unsafe { &*GPIOA::PTR };

        // dac_manual will have its value set manually
        let mut dac = dac1ch1.calibrate_buffer(&mut delay).enable(&mut rcc);

        dac.set_value(0);
        delay.delay_ms(1);
        assert!(is_pax_low(gpioa, 4));

        dac.set_value(4095);
        delay.delay_ms(1);
        assert!(!is_pax_low(gpioa, 4));
    }
}

fn is_pax_low(gpioa: &stm32::gpioa::RegisterBlock, x: u8) -> bool {
    gpioa.idr().read().idr(x).is_low()
}

#[derive(Debug, defmt::Format)]
struct ErrorTimedOut;

fn await_lo(
    gpioa: &stm32::gpioa::RegisterBlock,
    timeout: MicrosDurationU32,
) -> Result<MicrosDurationU32, ErrorTimedOut> {
    await_p(|| is_pax_low(gpioa, 8), timeout)
}

fn await_hi(
    gpioa: &stm32::gpioa::RegisterBlock,
    timeout: MicrosDurationU32,
) -> Result<MicrosDurationU32, ErrorTimedOut> {
    await_p(|| !is_pax_low(gpioa, 8), timeout)
}

fn await_p(
    mut p: impl FnMut() -> bool,
    timeout: MicrosDurationU32,
) -> Result<MicrosDurationU32, ErrorTimedOut> {
    let before = now();

    loop {
        let passed_time = now() - before;
        if p() {
            return Ok(passed_time);
        }
        if passed_time > timeout {
            return Err(ErrorTimedOut);
        }
    }
}
