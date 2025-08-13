//! FMAC Functions

/// Programmable gain parameter in the range 0dB to 42dB in 6dB increments.
#[repr(u8)]
#[derive(Copy, Clone)]
pub enum Gain {
    ZeroDB = 0,
    SixDB = 1,
    TwelveDB = 2,
    EighteenDB = 3,
    TwentyFourDB = 4,
    ThirtyDB = 5,
    ThirtySixDB = 6,
    FortyTwoDB = 7,
}

/// FMAC Function trait. This defines the function specific data that is loaded into the PARAM register
pub trait Function {
    /// The function ID loaded into the FUNC field
    const ID: u8;

    #[inline(always)]
    fn id(&self) -> u8 {
        Self::ID
    }

    /// Returns the value of the P parameter
    fn p(&self) -> u8;

    /// Returns the value of the Q parameter, or None if not used
    fn q(&self) -> Option<u8> {
        None
    }

    /// Returns the value of the R parameter, or None if not used
    fn r(&self) -> Option<u8> {
        None
    }
}

/// Load X1 buffer with specified length
pub struct LoadX1(pub u8);
/// Load X2 buffer with length of first set, and second set of coefficients
/// The second length may be zero if the second set of coefficients is not used.
pub struct LoadX2(pub u8, pub u8);
/// Load Y buffer with specified length
pub struct LoadY(pub u8);

impl Function for LoadX1 {
    const ID: u8 = 1;

    fn p(&self) -> u8 {
        self.0
    }
}

impl Function for LoadX2 {
    const ID: u8 = 2;

    fn p(&self) -> u8 {
        self.0
    }

    fn q(&self) -> Option<u8> {
        Some(self.1)
    }
}

impl Function for LoadY {
    const ID: u8 = 3;

    fn p(&self) -> u8 {
        self.0
    }
}

/// FIR / Convolution / Dot Product function
pub struct Convolution {
    /// Number of coefficients in the X2 buffer
    pub length: u8,
    /// Gain parameter in the range 0dB to 42dB in 6dB increments
    pub gain: Gain,
}

/// Finite Impulse Response (FIR) / Convolution / Dot Product function
impl Function for Convolution {
    const ID: u8 = 8;

    #[inline(always)]
    fn p(&self) -> u8 {
        self.length
    }

    #[inline(always)]
    fn r(&self) -> Option<u8> {
        Some(self.gain as u8)
    }
}

/// Infinite Impulse Response (IIR) filter / Multiply Accumulate
pub struct IIR {
    /// The number of feedforward coefficients in the X2 buffer
    /// X2 buffer size should be at least 2*feedforward_coeffs + 2*feedback_coeffs
    pub feedforward_coeffs: u8,
    /// The number of feedback coefficients in the X2 buffer
    /// X2 buffer size should be at least 2*feedforward_coeffs + 2*feedback_coeffs
    pub feedback_coeffs: u8,
    /// Gain parameter in the range 0dB to 42dB in 6dB increments
    pub gain: Gain,
}

impl Function for IIR {
    const ID: u8 = 9;

    #[inline(always)]
    fn p(&self) -> u8 {
        self.feedforward_coeffs
    }

    fn q(&self) -> Option<u8> {
        Some(self.feedback_coeffs)
    }

    fn r(&self) -> Option<u8> {
        Some(self.gain as u8)
    }
}
