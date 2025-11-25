//! Simple Dual DAC example for STM32G4 microcontrollers.
//!
//! This simple example configures DAC1 in DualDac mode and outputs a wrapping
//! ramp on PA4, and its inverse on PA5.
//!
//! This example highlights using the [`hal::dac::format::SampleQ15`] format for
//! native Q1.15 fixed point conversion by the DAC hardware when moving from holding register
//! to output register.
//!
//! Setting a channel to 0.0 will output a DC voltage of Vref/2, negative
//! values will be in the range [0.0..Vref/2], and positive values will be [Vref/2..Vref].
//!

#![deny(unsafe_code)]
#![no_main]
#![no_std]

use fixed::types::I1F15;
use hal::dac::dual::DualDacExt;
use hal::delay::SYSTDelayExt;
use hal::gpio::GpioExt;
use hal::rcc::RccExt;
use stm32g4xx_hal as hal;
mod utils;
extern crate cortex_m_rt as rt;

use hal::stm32;
use rt::entry;

#[entry]
fn main() -> ! {
    let dp = stm32::Peripherals::take().expect("cannot take peripherals");
    let cp = cortex_m::Peripherals::take().expect("cannot take core peripherals");

    let mut rcc = dp.RCC.constrain();
    let mut delay = cp.SYST.delay(&rcc.clocks);

    let gpioa = dp.GPIOA.split(&mut rcc);

    // Get a DualDac instance from DAC1 with channel outputs on PA4 and PA5
    let dac = dp.DAC1.into_dual::<hal::dac::format::SampleQ15, _>(
        (gpioa.pa4, gpioa.pa5),
        &mut rcc,
        &mut delay,
    );

    // Enable the DAC
    let mut dac = dac.enable();

    // Create an initial value that we'll sweep using the minimum fixed point value (-1.0)
    let mut value = I1F15::MIN;

    loop {
        // Output value on channel 1, and inverted value on channel 2
        // This uses the DAC's dual channel hold register, and native signed format mode
        // to write both channels simultaneously with a single register write operation.
        dac.set_channels(value, -value);

        // Increment and wrap the value by the minimum fixed point type increment delta
        value = value.wrapping_add(I1F15::DELTA);
    }
}
