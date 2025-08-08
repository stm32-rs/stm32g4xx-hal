//! Example of using the [`Rng`] peripheral.
//!
//! This example demonstrates common use cases of the [`rand::Rng`] trait using the G4 TRNG.
//!
//! ```DEFMT_LOG=debug cargo run --release --example rand --features stm32g431,defmt -- --chip STM32G431KBTx```

#![deny(warnings)]
#![deny(unsafe_code)]
#![no_main]
#![no_std]

use hal::prelude::*;
use hal::rng::RngExt;
use hal::stm32;
use rand::distr::Bernoulli;
use rand::distr::Distribution;
use rand::distr::Uniform;
use rand::seq::IndexedRandom;
use rand::{Rng, TryRngCore};
use stm32g4xx_hal as hal;

use cortex_m_rt::entry;

#[macro_use]
mod utils;

use utils::logger::info;

#[entry]
fn main() -> ! {
    utils::logger::init();

    info!("start");
    let dp = stm32::Peripherals::take().expect("cannot take peripherals");
    let cp = cortex_m::Peripherals::take().expect("cannot take core peripherals");

    let mut rcc = dp.RCC.constrain();

    let mut delay = cp.SYST.delay(&rcc.clocks);

    // Constrain and start the random number generator peripheral
    let rng = dp.RNG.constrain(&mut rcc).start();

    // Create a Uniform distribution sampler between 100 and 1000
    let between = Uniform::try_from(100..1000).unwrap();

    // Create a Bernoulli distribution sampler with a 20% probability of returning true
    let bernoulli = Bernoulli::new(0.2).unwrap();

    // A slice of values for IndexedRandom::choose
    let slice = ["foo", "bar", "baz"];

    loop {
        let random_float = rng.unwrap_err().random::<f32>();
        info!("Random float: {}", random_float);

        let random_u8 = rng.unwrap_err().random::<u8>();
        info!("Random u8: {}", random_u8);

        let random_array: [f32; 8] = rng.unwrap_err().random();
        info!("Random array {}", &random_array);

        let random_dist = between.sample(&mut rng.unwrap_err());
        info!("Random dist: {}", random_dist);

        let random_range = rng.unwrap_err().random_range(-10..10);
        info!("Random range: {}", random_range);

        let random_choice = slice.choose(&mut rng.unwrap_err());
        info!("Random choice: {}", random_choice);

        let random_bernoulli = bernoulli.sample(&mut rng.unwrap_err());
        info!("Random bernoulli: {}", random_bernoulli);

        delay.delay_ms(1000);
    }
}
