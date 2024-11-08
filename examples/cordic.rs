#![deny(warnings)]
#![deny(unsafe_code)]
#![no_main]
#![no_std]

extern crate cortex_m;
extern crate cortex_m_rt as rt;
extern crate stm32g4xx_hal as hal;

use hal::cordic::{
    op::{dynamic::Mode as _, Magnitude, SinCos, Sqrt},
    prec::P60,
    scale::N0,
    types::{I1F15, Q15, Q31},
    Ext as _,
};
use hal::prelude::*;
use hal::pwr::PwrExt;
use hal::rcc::Config;
use hal::stm32;
use rt::entry;

#[macro_use]
mod utils;

use utils::logger::println;

#[entry]
fn main() -> ! {
    let dp = stm32::Peripherals::take().expect("cannot take peripherals");
    let pwr = dp.PWR.constrain().freeze();
    let mut rcc = dp.RCC.freeze(Config::hsi(), pwr);

    let mut cordic = dp
        .CORDIC
        .constrain(&mut rcc)
        .freeze::<Q15, Q31, P60, SinCos>(); // 16 bit arguments, 32 bit results, compute sine and cosine, 60 iterations

    // static operation (zero overhead)

    cordic.start(I1F15::from_num(-0.25 /* -45 degreees */));

    let (sin, cos) = cordic.result();

    println!("sin: {}, cos: {}", sin.to_num::<f32>(), cos.to_num::<f32>());

    // dynamic operation

    let mut cordic = cordic.into_dynamic();

    let sqrt = cordic.run::<Sqrt<N0>>(I1F15::from_num(0.25));
    println!("sqrt: {}", sqrt.to_num::<f32>());
    let magnitude = cordic.run::<Magnitude>((I1F15::from_num(0.25), I1F15::from_num(0.5)));
    println!("magnitude: {}", magnitude.to_num::<f32>());

    loop {}
}
