//! ## Origin
//!
//! This code has been taken from the stm32g0xx-hal project and modified to support
//! STM32G4xx MCUs.

// #![deny(warnings)]
#![deny(unsafe_code)]
#![no_main]
#![no_std]

use embedded_hal::delay::DelayNs;
use hal::dac::DacExt;
use hal::delay::SYSTDelayExt;
use hal::gpio::GpioExt;
use hal::rcc::RccExt;
use stm32g4xx_hal as hal;
use stm32g4xx_hal::dac::{CountingDirection, SawtoothConfig};
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
    let dac1ch1 = dp.DAC1.constrain(gpioa.pa4, &mut rcc);

    // Just like the ADC, the DAC (by default) converts 12bit values between digital
    // and analog. We will configure the dac to automatically update its output
    // froms its internal sawtooth generator.
    //
    // Internally however, for the sawtooth generation the dac uses a
    // 16 bit counter where only the 12 msb are sent to the dac output and the 4 msb
    // are ignored by the output and only used for counting. The counter is reset to
    // `reset_value << 4` at every `trigger_reset`, meaning the output's 12 bits
    // will be set to `reset_value`.
    //
    // Every trigger_inc this 16 bit counter is incremented/decremented by the specified
    // step size. Since the 4 msb are not mapped to the output, for example using a step
    // size of 1 will only produce a change on the output every 2^4=16th trigger_inc.
    //
    // We will configure the sawtooth generator with a negative slope. This means that
    // the generator will subtract the step_size from the counter every trigger_inc.
    // This produces something like the following waveform.
    //
    // \              |\              |\              |\              |
    //   \            |  \            |  \            |  \            |
    //     \          |    \          |    \          |    \          |
    //       \        |      \        |      \        |      \        |
    //         \      |        \      |        \      |        \      |
    //           \    |          \    |          \    |          \    |
    //             \  |            \  |            \  |            \  |
    //               \|              \|              \|              \|
    let step_size = 16;
    let reset_value = 4000;
    let trigger_count = (reset_value << 4) / step_size;
    let delay_us = 1;

    // dac_generator will have its value set automatically from its internal sawtooth generator
    let mut dac_generator = dac1ch1.enable_sawtooth_generator(
        SawtoothConfig::with_slope(CountingDirection::Decrement, step_size)
            .reset_value(reset_value),
        &mut rcc,
    );

    loop {
        for _ in 0..trigger_count {
            dac_generator.trigger_inc();
            delay.delay_us(delay_us);
        }
        delay.delay_us(delay_us);
        dac_generator.trigger_reset();
    }
}
