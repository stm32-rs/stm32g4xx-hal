//! STM32G4 DAC Data Sample Formats

use fixed::types::{I1F15, I1F7};

/// DAC Data Alignment
pub enum Alignment {
    Left,
    Right,
}

pub enum SampleDepth {
    Bits12,
    Bits8,
}

pub trait SampleFormat {
    /// Bit depth of the sample format (8 or 12 bits)
    const DEPTH: SampleDepth;
    /// Alignment of the samples in the DAC hold register.
    /// Left alignment is used for signed 2's complement data
    const ALIGNMENT: Alignment;
    /// Signedness of the sample format
    const SIGNED: bool;
    /// Shift amount for Channel 2 samples into the 32 bit hold register
    const CH2_SHIFT: u8;
    /// Mask for Channel 1 samples in the 32 bit hold register
    const CH1_MASK: u32;

    /// Scalar type for this sample format
    /// Q1.7, Q1.11, Q1.15 use fixed types, unsigned are u8 or u16
    type Scalar: ToDac;
}

/// Conversion trait for DAC data. Used to support conversion from
/// fixed point types to their bit representation as u16.
pub trait ToDac {
    fn to_dac(&self) -> u16;
}

impl ToDac for I1F7 {
    fn to_dac(&self) -> u16 {
        self.to_bits() as u16
    }
}

impl ToDac for I1F15 {
    fn to_dac(&self) -> u16 {
        self.to_bits() as u16
    }
}

impl ToDac for u16 {
    #[inline(always)]
    fn to_dac(&self) -> u16 {
        *self
    }
}

impl ToDac for u8 {
    #[inline(always)]
    fn to_dac(&self) -> u16 {
        *self as u16
    }
}

/// Unsigned 8-bit samples
pub struct SampleU8;
impl SampleFormat for SampleU8 {
    const DEPTH: SampleDepth = SampleDepth::Bits8;
    const ALIGNMENT: Alignment = Alignment::Right;
    const SIGNED: bool = false;
    const CH2_SHIFT: u8 = 8;
    const CH1_MASK: u32 = 0xFF;
    type Scalar = u8;
}

/// Unsigned 12-bit samples
pub struct SampleU12;
impl SampleFormat for SampleU12 {
    const DEPTH: SampleDepth = SampleDepth::Bits12;
    const ALIGNMENT: Alignment = Alignment::Right;
    const SIGNED: bool = false;
    const CH2_SHIFT: u8 = 16;
    const CH1_MASK: u32 = 0xFFFF;
    type Scalar = u16;
}

/// Fixed point signed Q1.7 samples
pub struct SampleQ7;
impl SampleFormat for SampleQ7 {
    const DEPTH: SampleDepth = SampleDepth::Bits8;
    const ALIGNMENT: Alignment = Alignment::Right;
    const SIGNED: bool = true;
    const CH2_SHIFT: u8 = 8;
    const CH1_MASK: u32 = 0xFF;
    type Scalar = I1F7;
}

/// Fixed point signed Q1.11 samples
pub struct SampleQ11;
impl SampleFormat for SampleQ11 {
    const DEPTH: SampleDepth = SampleDepth::Bits12;
    const ALIGNMENT: Alignment = Alignment::Left;
    const SIGNED: bool = true;
    const CH2_SHIFT: u8 = 16;
    const CH1_MASK: u32 = 0xFFFF;
    type Scalar = I1F15;
}

/// Fixed point signed Q1.15 samples
pub struct SampleQ15;
impl SampleFormat for SampleQ15 {
    const DEPTH: SampleDepth = SampleDepth::Bits12;
    const ALIGNMENT: Alignment = Alignment::Left;
    const SIGNED: bool = true;
    const CH2_SHIFT: u8 = 16;
    const CH1_MASK: u32 = 0xFFFF;
    type Scalar = I1F15;
}
