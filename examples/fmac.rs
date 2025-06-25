#![deny(warnings)]
#![deny(unsafe_code)]
#![no_main]
#![no_std]

use cortex_m::asm::wfi;
use hal::prelude::*;
use hal::stm32;
use stm32g4xx_hal as hal;

use cortex_m_rt::entry;

#[macro_use]
mod utils;

use utils::logger::info;

use stm32g4xx_hal::fmac::{
    buffer::{BufferLayout, Watermark},
    function::IIR,
    Buffer, FmacExt,
};

use fixed::types::I1F15;

#[entry]
fn main() -> ! {
    utils::logger::init();

    info!("start");
    let dp = stm32::Peripherals::take().expect("cannot take peripherals");
    let mut rcc = dp.RCC.constrain();

    // The FMAC has an internal memory area of 256x16bit words that must be allocated to the
    // three different buffers named X1 (input buffer), X2 (coefficients), and Y (output buffer).
    //
    // The BufferLayout struct takes three generic consts and will calculate the base offsets at compile time,
    // which must be passed as a generic parameter to the constrain method of the FMAC instance.
    //
    // Create an FMAC instance with a memory layout of 32 inputs, 2 coefficients, and 1 output buffer words
    //
    // For IIR filters, RM0440 indicates X2 coeffecient buffer should be 2N + 2M where
    // N is the number of feedforward coefficients and M is the number of feedback coefficients.
    //
    // Following constrain(), the buffer layout has been applied to the peripheral and cannot be changed.
    let mut fmac = dp
        .FMAC
        .constrain::<BufferLayout<32, 4, 1>>(&mut rcc)
        .reset();

    // Configure an IIR filter with 1 feedforward, and 1 feedback coefficient,
    // with both coefficients set to 1.0.
    //
    // This is equivalent to a multiply accumulate operation
    // Y[n] = ∑ X1[n] * X2[0] + Y[n-1] * X2[1]
    //
    // The inputs will be set to ~0.01 (to nearest fixed point representation), and the output will increment
    // by 0.01 into Y for each input sample, and the final read of Y should be about 0.319

    // Fill the input buffer with 0.01
    fmac.preload_buffer(Buffer::X1, |_index| I1F15::from_num(0.01));

    // Fill the coefficients with I1F15::MAX (max value representable by I1F15, ~1.0)
    fmac.preload_buffer(Buffer::X2, |_index| I1F15::MAX);
    fmac.preload_buffer(Buffer::Y, |_index| I1F15::ZERO);

    // Watermarks can be set on the X1 and Y buffers
    // to control when the input empty and output full
    // flags are asserted to cause early interrupts or DMA
    // to avoid underflow or overflow conditions.
    fmac.set_watermark(Buffer::X1, Watermark::Threshold1);
    fmac.set_watermark(Buffer::Y, Watermark::Threshold1);

    // Select the IIR function, specifying the number of
    // feedforward and feedback coefficients, and the gain
    fmac.select_function(IIR {
        feedforward_coeffs: 1,
        feedback_coeffs: 1,
        gain: 0,
    });

    let fmac = fmac.start();

    info!("Input buffer full: {}", fmac.is_input_full());

    info!("Waiting for result");
    while !fmac.is_result_available() {}

    info!("Reading results");

    let mut count = 0;
    loop {
        while let Some(output) = fmac.read() {
            count += 1;
            info!("Output {}: {}", count, output.to_num::<f32>());
        }
    }
}
