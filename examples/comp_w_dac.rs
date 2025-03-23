// #![deny(warnings)]
#![deny(unsafe_code)]
#![no_main]
#![no_std]

mod utils;
extern crate cortex_m_rt as rt;

use rt::entry;

#[entry]
fn main() -> ! {
    use stm32g4xx_hal::{
        comparator::{self, ComparatorExt, ComparatorSplit},
        dac::{Dac1IntSig1, DacExt, DacOut},
        delay::SYSTDelayExt,
        gpio::{GpioExt, PushPull},
        pac,
        rcc::RccExt,
        stasis::Freeze,
    };

    let dp = pac::Peripherals::take().expect("cannot take peripherals");
    let cp = cortex_m::Peripherals::take().expect("cannot take core peripherals");

    let mut rcc = dp.RCC.constrain();
    let mut delay = cp.SYST.delay(&rcc.clocks);

    let gpioa = dp.GPIOA.split(&mut rcc);

    // Set up DAC to output to pa4 and to internal signal Dac1IntSig1
    // which just so happens is compatible with comp1
    let dac1ch1 = dp.DAC1.constrain((gpioa.pa4, Dac1IntSig1), &mut rcc);
    let dac = dac1ch1.calibrate_buffer(&mut delay).enable(&mut rcc);
    let (mut dac, [dac_token]) = dac.freeze();

    let (comp1, _comp2, ..) = dp.COMP.split(&mut rcc);

    // Set up comparator with pa1 as positive, and the DAC as negative input
    let comp = comp1.comparator(
        gpioa.pa1,
        dac_token,
        comparator::Config::default().hysteresis(comparator::Hysteresis::None),
        &rcc.clocks,
    );

    let led2 = gpioa.pa0;
    // Configure PA12 to the comparator's alternate function so it gets
    // changed directly by the comparator.
    comp.output_pin::<PushPull>(led2);
    let _comp1 = comp.enable().lock();

    enum Direction {
        Upcounting,
        Downcounting,
    }

    let mut dir = Direction::Upcounting;
    let mut val = 0;

    // Manually step the DAC's value to produce a triangle wave
    //
    // This will produce a pwm-like signal at pa0 with the duty controlled by pa1
    // where
    // * 0V   at p1 =>   0% duty
    // * VDDA at p1 => 100% duty
    loop {
        dac.set_value(val);
        match val {
            0 => dir = Direction::Upcounting,
            4095 => dir = Direction::Downcounting,
            _ => (),
        };

        match dir {
            Direction::Upcounting => val += 1,
            Direction::Downcounting => val -= 1,
        }
    }
}
