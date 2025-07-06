#![no_std]
#![no_main]
#![allow(clippy::uninlined_format_args)]

#[path = "../utils/mod.rs"]
mod utils;
use utils::logger::info;

/// Example showcasing the use of the HRTIM peripheral's capture function to detect phase shift between a digital event and the output of HRTIM_TIMA
use cortex_m_rt::entry;
use stm32_hrtim::{
    capture,
    compare_register::HrCompareRegister,
    external_event::{self, ToExternalEventSource},
    output::HrOutput,
    timer::{HrSlaveTimerCpt, HrTimer, TimerSplitCapture},
    HrParts, HrPwmAdvExt, Pscl128,
};
use stm32g4xx_hal::{
    dma::{channel::DMAExt, config::DmaConfig, TransferExt},
    gpio::GpioExt,
    hrtim::{external_event::EevInputExt, HrControltExt, HrPwmBuilderExt},
    pwr::PwrExt,
    rcc::{self, RccExt},
    stm32::Peripherals,
};

#[entry]
fn main() -> ! {
    info!("start");

    let dp = Peripherals::take().unwrap();

    // Set system frequency to 16MHz * 15/1/2 = 120MHz
    // This would lead to HrTim running at 120MHz * 32 = 3.84...
    info!("rcc");
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

    info!("Setup Gpio");
    let gpioa = dp.GPIOA.split(&mut rcc);
    let gpiob = dp.GPIOB.split(&mut rcc);

    // PA8 (D7 on Nucleo G474RE)
    let pin_a = gpioa.pa8;

    // PB5 (D4 on Nucleo G474RE)
    let input = gpiob.pb5.into_pull_down_input();

    // ...with a prescaler of 128 this gives us a HrTimer with a tick rate of 30MHz
    // With max the max period set, this would be 30MHz/2^16 ~= 458Hz...
    let prescaler = Pscl128;

    //        t1     t2       .
    //        |      |        .
    //        v      v        .
    //        .               .
    //        .  50%          .
    //         ------          ------
    //out1    |      |        |      |
    //        |      |        |      |
    // --------      ----------      --------
    let period = 0xFFFF;
    let (mut hr_control, _flt_inputs, eev_inputs) =
        dp.HRTIM_COMMON.hr_control(&mut rcc).wait_for_calibration();

    let eev_input6 = eev_inputs
        .eev_input6
        .bind(input)
        .edge_or_polarity(external_event::EdgeOrPolarity::Edge(
            external_event::Edge::Both,
        ))
        .finalize(&mut hr_control);

    let mut hr_control = hr_control.constrain();
    let HrParts {
        timer,
        mut cr1,
        out: mut out1,
        dma_channel,
        ..
    } = dp
        .HRTIM_TIMA
        .pwm_advanced(pin_a)
        .prescaler(prescaler)
        .period(period)
        .finalize(&mut hr_control);
    out1.enable_rst_event(&cr1); // Set low on compare match with cr1
    out1.enable_set_event(&timer); // Set high at new period
    cr1.set_duty(period / 2);

    let TimerSplitCapture {
        mut timer,
        ch1: mut capture,
        ..
    } = timer.split_capture();
    timer.start(&mut hr_control.control);
    out1.enable();

    capture.enable_interrupt(true, &mut hr_control);
    capture.add_event(&eev_input6);

    info!("Setup DMA");
    let channels = dp.DMA1.split(&rcc);
    let config = DmaConfig::default()
        .transfer_complete_interrupt(false)
        .circular_buffer(true)
        .memory_increment(true);

    let first_buffer = cortex_m::singleton!(: [u32; 16] = [0; 16]).unwrap();
    let mut transfer = channels.ch1.into_circ_peripheral_to_memory_transfer(
        capture.enable_dma(dma_channel),
        &mut first_buffer[..],
        config,
    );

    transfer.start(|_| ());

    let mut old_duty = 0;
    loop {
        for duty in (u32::from(period) / 10)..(9 * u32::from(period) / 10) {
            let mut data = [0; 2];
            transfer.read_exact(&mut data);
            let [t1, t2] = data.map(|x| capture::dma_value_to_signed(x, period));
            cr1.set_duty(duty as u16);
            info!("Capture: t1: {}, t2: {}, duty: {}, ", t1, t2, old_duty);
            old_duty = duty;
        }
    }
}
