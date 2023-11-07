//This example puts the timer in PWM mode using the specified pin with a frequency of 100Hz and a duty cycle of 50%.
#![no_main]
#![no_std]

use cortex_m_rt::entry;
use fugit::ExtU32;
use hal::comparator;
use hal::comparator::{Hysteresis, ComparatorSplit, ComparatorExt};
use hal::dac::{self, DacExt, DacOut};
use hal::gpio::gpioa::PA8;
use hal::gpio::Alternate;
use hal::gpio::AF13;
use hal::hrtim::compare_register::HrCompareRegister;
use hal::hrtim::event::EventSource;
use hal::hrtim::external_event::{self, ToExternalEventSource};
use hal::hrtim::timer_eev_cfg::EevCfgs;
use hal::hrtim::timer::HrTimer;
use hal::hrtim::HrPwmAdvExt;
use hal::hrtim::Pscl4;
use hal::hrtim::{control::HrControltExt, output::HrOutput};
use hal::prelude::*;
use hal::pwm;
use hal::pwr::{self, PwrExt};
use hal::rcc;
use hal::stm32;
use stm32g4xx_hal as hal;

use defmt_rtt as _; // global logger
use panic_probe as _;

#[entry]
fn main() -> ! {
    let dp = stm32::Peripherals::take().expect("cannot take peripherals");
    let cp = stm32::CorePeripherals::take().expect("cannot take core");
    // Set system frequency to 16MHz * 75/4/2 = 150MHz
    // This would lead to HrTim running at 150MHz * 32 = 4.8GHz...
    let pwr = dp
        .PWR
        .constrain()
        .vos(pwr::VoltageScale::Range1 {
            enable_boost: false,
        })
        .freeze();

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

    let mut delay = cp.SYST.delay(&rcc.clocks);

    let gpioa = dp.GPIOA.split(&mut rcc);
    let gpiob = dp.GPIOB.split(&mut rcc);

    let input = gpioa.pa1.into_analog();
    let pin_a: PA8<Alternate<AF13>> = gpioa.pa8.into_alternate();

    let dac3ch1 = dp.DAC3.constrain(dac::Dac3IntSig1, &mut rcc);
    let mut dac = dac3ch1.enable();

    // Use dac to define the fault threshold
    // 2^12 / 2 = 2^11 for about half of VCC
    let fault_limit = 60;
    dac.set_value(fault_limit);

    let (comp1, ..) = dp.COMP.split(&mut rcc);

    let comp1 = comp1
        .comparator(
            &input,
            &dac,
            comparator::Config::default()
                .hysteresis(Hysteresis::None),
                //.output_inverted(),
            &rcc.clocks,
        )
        .enable();

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
        .eev_cfg(EevCfgs::default())
        .period(0xFFFF)
        .finalize(&mut hr_control);

    out1.enable_rst_event(&cr1); // Set low on compare match with cr1
    out1.enable_rst_event(eev_input4);
    out1.enable_set_event(&timer); // Set high at new period
    cr1.set_duty(timer.get_period() / 3);
    //unsafe {((HRTIM_COMMON::ptr() as *mut u8).offset(0x14) as *mut u32).write_volatile(1); }
    out1.enable();
    timer.start(&mut hr_control);

    defmt::info!("Started");

    loop {}
}
