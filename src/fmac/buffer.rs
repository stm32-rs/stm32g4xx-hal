//! FMAC Buffer configuration

/// Buffer watermark thresholds
#[repr(u8)]
pub enum Watermark {
    Threshold1 = 0,
    Threshold2 = 1,
    Threshold4 = 2,
    Threshold8 = 3,
}

/// FMAC Buffer layout configuration trait
pub trait BufferLayoutConfig {
    const X1_BASE: u8;
    const X2_BASE: u8;
    const Y_BASE: u8;

    const X1_SIZE: u8;
    const X2_SIZE: u8;
    const Y_SIZE: u8;

    // Verify that the total size doesn't exceed FMAC memory
    const _ASSERT_SIZE: () = assert!(
        (Self::Y_BASE + Self::Y_SIZE) as u16 <= 256,
        "Total buffer size exceeds FMAC memory capacity"
    );
}

/// FMAC buffer layout
///
/// Note: For IIR filters, minimum coefficient size is 2N + 2M
/// where N is the number of feedforward coefficients and M is the number of feedback coefficients.
///
/// The base address of the layout can be specified, and defaults to 0.
/// This allows for more flexibility in memory allocation for configuring
/// multiple layouts within the FMAC memory.
pub struct BufferLayout<
    const INPUT_SIZE: u8,
    const COEFFICIENT_SIZE: u8,
    const OUTPUT_SIZE: u8,
    const BASE: u8 = 0,
>;

impl<const INPUT_SIZE: u8, const COEFFICIENT_SIZE: u8, const OUTPUT_SIZE: u8, const BASE: u8>
    BufferLayoutConfig for BufferLayout<INPUT_SIZE, COEFFICIENT_SIZE, OUTPUT_SIZE, BASE>
{
    const X1_BASE: u8 = BASE;
    const X2_BASE: u8 = Self::X1_BASE + INPUT_SIZE;
    const Y_BASE: u8 = Self::X2_BASE + COEFFICIENT_SIZE;

    const X1_SIZE: u8 = INPUT_SIZE;
    const X2_SIZE: u8 = COEFFICIENT_SIZE;
    const Y_SIZE: u8 = OUTPUT_SIZE;
}
