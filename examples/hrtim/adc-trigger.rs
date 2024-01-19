#![no_std]
#![no_main]

use cortex_m_rt::entry;

use defmt_rtt as _; // global logger
use panic_probe as _;

#[cfg(not(any(feature = "stm32g474", feature = "stm32g484")))]
#[entry]
fn main() -> ! {
    #[allow(clippy::empty_loop)]
    loop {}
}

#[cfg(any(feature = "stm32g474", feature = "stm32g484"))]
#[entry]
fn main() -> ! {
    use hal::adc;
    use stm32g4xx_hal as hal;

    use defmt::info;
    use hal::{
        adc::{
            config::{Continuous, Dma as AdcDma, SampleTime, Sequence},
            AdcClaim, ClockSource, Temperature, Vref,
        },
        delay::SYSTDelayExt,
        dma::{self, config::DmaConfig, stream::DMAExt, TransferExt},
        gpio::{gpioa::PA8, gpioa::PA9, Alternate, GpioExt, AF13},
        hrtim::compare_register::HrCompareRegister,
        hrtim::control::HrControltExt,
        hrtim::output::HrOutput,
        hrtim::timer::HrTimer,
        hrtim::HrPwmAdvExt,
        hrtim::Pscl4,
        pwr::PwrExt,
        rcc::{self, RccExt},
        stm32::Peripherals,
    };

    const VREF: f32 = 3.3;

    info!("start");

    let dp = Peripherals::take().unwrap();
    let cp = cortex_m::Peripherals::take().expect("cannot take core peripherals");

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

    let mut delay = cp.SYST.delay(&rcc.clocks);

    let dma::stream::StreamsTuple(dma1ch1, ..) = dp.DMA1.split(&rcc);
    let config = DmaConfig::default()
        .transfer_complete_interrupt(true)
        .circular_buffer(true)
        .memory_increment(true);

    info!("Setup Gpio");
    let gpioa = dp.GPIOA.split(&mut rcc);
    let pa0 = gpioa.pa0.into_analog();

    let pin_a: PA8<Alternate<AF13>> = gpioa.pa8.into_alternate();
    let pin_b: PA9<Alternate<AF13>> = gpioa.pa9.into_alternate();

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
    let (hr_control, ..) = dp.HRTIM_COMMON.hr_control(&mut rcc).wait_for_calibration();
    let mut hr_control = hr_control.constrain();
    let (mut timer, (mut cr1, _cr2, mut cr3, mut cr4), (mut out1, mut out2)) = dp
        .HRTIM_TIMA
        .pwm_advanced((pin_a, pin_b), &mut rcc)
        .prescaler(prescaler)
        .period(period)
        .finalize(&mut hr_control);

    cr1.set_duty(period / 2);
    cr3.set_duty(period / 3);
    cr4.set_duty((2 * u32::from(period) / 3) as u16);

    hr_control.adc_trigger1.enable_source(&cr3);
    hr_control.adc_trigger1.enable_source(&cr4);

    out1.enable_rst_event(&cr1); // Set low on compare match with cr1
    out2.enable_rst_event(&cr1);

    out1.enable_set_event(&timer); // Set high at new period
    out2.enable_set_event(&timer);

    info!("Setup Adc1");
    let mut adc = dp
        .ADC1
        .claim(ClockSource::SystemClock, &rcc, &mut delay, true);

    adc.set_external_trigger((
        adc::config::TriggerMode::RisingEdge,
        &hr_control.adc_trigger1,
    ));
    adc.enable_temperature(&dp.ADC12_COMMON);
    adc.set_continuous(Continuous::Discontinuous);
    adc.reset_sequence();
    adc.configure_channel(&pa0, Sequence::One, SampleTime::Cycles_640_5);
    adc.configure_channel(&Temperature, Sequence::Two, SampleTime::Cycles_640_5);

    info!("Setup DMA");
    let first_buffer = cortex_m::singleton!(: [u16; 10] = [0; 10]).unwrap();

    let mut transfer = dma1ch1.into_circ_peripheral_to_memory_transfer(
        adc.enable_dma(AdcDma::Continuous),
        &mut first_buffer[..],
        config,
    );

    transfer.start(|adc| adc.start_conversion());

    out1.enable();
    out2.enable();

    timer.start(&mut hr_control);

    loop {
        let mut b = [0_u16; 4];
        let r = transfer.read_exact(&mut b);

        info!("read: {}", r);
        assert!(r == b.len());

        let millivolts = Vref::sample_to_millivolts((b[0] + b[2]) / 2);
        info!("pa3: {}mV", millivolts);
        let temp = Temperature::temperature_to_degrees_centigrade(
            (b[1] + b[3]) / 2,
            VREF,
            adc::config::Resolution::Twelve,
        );
        info!("temp: {}℃C", temp);
    }
}
