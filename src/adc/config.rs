/// Contains types related to ADC configuration
use embedded_hal_old::adc::Channel;
use fugit::HertzU32;

use crate::rcc;

/// The place in the sequence a given channel should be captured
#[derive(Debug, PartialEq, PartialOrd, Copy, Clone)]
pub enum Sequence {
    /// 1
    One,
    /// 2
    Two,
    /// 3
    Three,
    /// 4
    Four,
    /// 5
    Five,
    /// 6
    Six,
    /// 7
    Seven,
    /// 8
    Eight,
    /// 9
    Nine,
    /// 10
    Ten,
    /// 11
    Eleven,
    /// 12
    Twelve,
    /// 13
    Thirteen,
    /// 14
    Fourteen,
    /// 15
    Fifteen,
    /// 16
    Sixteen,
}

impl From<Sequence> for u8 {
    fn from(s: Sequence) -> u8 {
        match s {
            Sequence::One => 0,
            Sequence::Two => 1,
            Sequence::Three => 2,
            Sequence::Four => 3,
            Sequence::Five => 4,
            Sequence::Six => 5,
            Sequence::Seven => 6,
            Sequence::Eight => 7,
            Sequence::Nine => 8,
            Sequence::Ten => 9,
            Sequence::Eleven => 10,
            Sequence::Twelve => 11,
            Sequence::Thirteen => 12,
            Sequence::Fourteen => 13,
            Sequence::Fifteen => 14,
            Sequence::Sixteen => 15,
        }
    }
}

impl From<u8> for Sequence {
    fn from(bits: u8) -> Self {
        match bits {
            0 => Sequence::One,
            1 => Sequence::Two,
            2 => Sequence::Three,
            3 => Sequence::Four,
            4 => Sequence::Five,
            5 => Sequence::Six,
            6 => Sequence::Seven,
            7 => Sequence::Eight,
            8 => Sequence::Nine,
            9 => Sequence::Ten,
            10 => Sequence::Eleven,
            11 => Sequence::Twelve,
            12 => Sequence::Thirteen,
            13 => Sequence::Fourteen,
            14 => Sequence::Fifteen,
            15 => Sequence::Sixteen,
            _ => unimplemented!(),
        }
    }
}

/// The number of cycles to sample a given channel for
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Debug, PartialEq, Copy, Clone)]
pub enum SampleTime {
    /// 2.5 cycles
    Cycles_2_5,
    /// 6.5 cycles
    Cycles_6_5,
    /// 12_5 cycles
    Cycles_12_5,
    /// 24.5 cycles
    Cycles_24_5,
    /// 47.5 cycles
    Cycles_47_5,
    /// 92.5 cycles
    Cycles_92_5,
    /// 247.5 cycles
    Cycles_247_5,
    /// 640.5 cycles
    Cycles_640_5,
}

impl From<u8> for SampleTime {
    fn from(f: u8) -> SampleTime {
        match f {
            0 => SampleTime::Cycles_2_5,
            1 => SampleTime::Cycles_6_5,
            2 => SampleTime::Cycles_12_5,
            3 => SampleTime::Cycles_24_5,
            4 => SampleTime::Cycles_47_5,
            5 => SampleTime::Cycles_92_5,
            6 => SampleTime::Cycles_247_5,
            7 => SampleTime::Cycles_640_5,
            _ => unimplemented!(),
        }
    }
}

impl From<SampleTime> for u8 {
    fn from(l: SampleTime) -> u8 {
        match l {
            SampleTime::Cycles_2_5 => 0,
            SampleTime::Cycles_6_5 => 1,
            SampleTime::Cycles_12_5 => 2,
            SampleTime::Cycles_24_5 => 3,
            SampleTime::Cycles_47_5 => 4,
            SampleTime::Cycles_92_5 => 5,
            SampleTime::Cycles_247_5 => 6,
            SampleTime::Cycles_640_5 => 7,
        }
    }
}

/// ADC Clock Source selection
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Debug, Copy, Clone, Default)]
pub enum ClockSource {
    /// Use the System Clock as Clock Source
    #[default]
    SystemClock,
    /// use the Internal PLL as Clock Source
    PllP,
}

impl From<ClockSource> for u8 {
    fn from(c: ClockSource) -> u8 {
        match c {
            ClockSource::PllP => 0b01,
            ClockSource::SystemClock => 0b10,
        }
    }
}

/// ClockMode config for the ADC
/// Check the datasheet for the maximum speed the ADC supports
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Debug, Copy, Clone)]
pub enum ClockMode {
    /// (Asynchronous clock mode), adc_ker_ck. generated at product level (refer to Section 6: Reset and clock control (RCC)
    AdcKerCk {
        /// Clock prescaler
        ///
        /// The final clock frequency will be `f_src / prescaler`
        prescaler: Prescaler,

        /// Clock source for the ADC
        ///
        /// The final clock frequency will be `f_src / clock`
        src: ClockSource,
    },
    /// (Synchronous clock mode). adc_hclk/1
    /// This configuration must be enabled only if the AHB clock prescaler is set to 1 (HPRE\[3:0\] = 0xxx in RCC_CFGR register) and if the system clock has a 50% duty cycle.
    AdcHclkDiv1,
    /// Synchronous clock mode. adc_hclk/2
    AdcHclkDiv2,
    /// Synchronous clock mode. adc_hclk/4
    AdcHclkDiv4,
}

impl ClockMode {
    /// Validate configuration
    pub fn validate(&self, rcc: &mut crate::rcc::Rcc) -> HertzU32 {
        fn prescaler_to_factor(p: Prescaler) -> u32 {
            match p {
                Prescaler::Div_1 => 1,
                Prescaler::Div_2 => 2,
                Prescaler::Div_4 => 4,
                Prescaler::Div_6 => 6,
                Prescaler::Div_8 => 8,
                Prescaler::Div_10 => 10,
                Prescaler::Div_12 => 12,
                Prescaler::Div_16 => 16,
                Prescaler::Div_32 => 32,
                Prescaler::Div_64 => 64,
                Prescaler::Div_128 => 128,
                Prescaler::Div_256 => 256,
            }
        }

        let f = match self {
            ClockMode::AdcKerCk {
                prescaler: clock,
                src: ClockSource::PllP,
            } => {
                rcc.clocks
                    .pll_clk
                    .p
                    .expect("Pll-P selected as clock source for ADC but is disabled")
                    .raw()
                    / prescaler_to_factor(*clock)
            }
            ClockMode::AdcKerCk {
                prescaler: clock,
                src: ClockSource::SystemClock,
            } => rcc.clocks.sys_clk.raw() / prescaler_to_factor(*clock),

            //01: adc_hclk/1 (Synchronous clock mode). This configuration must be enabled only if the
            //AHB clock prescaler is set (HPRE[3:0] = 0xxx in RCC_CFGR register) and if the system
            //clock has a 50% duty cycle.
            ClockMode::AdcHclkDiv1 => {
                assert!(rcc.rb.cfgr().read().hpre().is_div1());
                rcc.clocks.ahb_clk.raw()
            }
            ClockMode::AdcHclkDiv2 => rcc.clocks.ahb_clk.raw() / 2,
            ClockMode::AdcHclkDiv4 => rcc.clocks.ahb_clk.raw() / 4,
        };

        HertzU32::Hz(f)
    }

    pub(crate) fn to_bits(self, rcc: &mut rcc::Rcc) -> ClockBits {
        assert!(self.validate(rcc) <= HertzU32::MHz(60));
        match self {
            ClockMode::AdcKerCk {
                prescaler: clock,
                src,
            } => ClockBits {
                ckmode: 0b00,
                presc: clock.into(),
                adcsel: src.into(),
            },
            ClockMode::AdcHclkDiv1 => ClockBits {
                ckmode: 0b01,
                presc: 0,
                adcsel: 0,
            },
            ClockMode::AdcHclkDiv2 => ClockBits {
                ckmode: 0b10,
                presc: 0,
                adcsel: 0,
            },
            ClockMode::AdcHclkDiv4 => ClockBits {
                ckmode: 0b11,
                presc: 0,
                adcsel: 0,
            },
        }
    }
}

impl Default for ClockMode {
    fn default() -> Self {
        Self::AdcKerCk {
            prescaler: Default::default(),
            src: Default::default(),
        }
    }
}

pub(crate) struct ClockBits {
    pub(crate) ckmode: u8,
    pub(crate) presc: u8,
    pub(crate) adcsel: u8,
}

/// Clock config for the ADC
/// Check the datasheet for the maximum speed the ADC supports
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Debug, Copy, Clone, Default)]
pub enum Prescaler {
    /// Clock not divided
    #[default]
    Div_1,
    /// Clock divided by 2
    Div_2,
    /// Clock divided by 4
    Div_4,
    /// Clock divided by 6
    Div_6,
    /// Clock divided by 8
    Div_8,
    /// Clock divided by 10
    Div_10,
    /// Clock divided by 12
    Div_12,
    /// Clock divided by 16
    Div_16,
    /// Clock divided by 32
    Div_32,
    /// Clock divided by 64
    Div_64,
    /// Clock divided by 128
    Div_128,
    /// Clock divided by 256
    Div_256,
}

impl From<Prescaler> for u8 {
    fn from(c: Prescaler) -> u8 {
        match c {
            Prescaler::Div_1 => 0b000,
            Prescaler::Div_2 => 0b001,
            Prescaler::Div_4 => 0b010,
            Prescaler::Div_6 => 0b011,
            Prescaler::Div_8 => 0b100,
            Prescaler::Div_10 => 0b101,
            Prescaler::Div_12 => 0b110,
            Prescaler::Div_16 => 0b111,
            Prescaler::Div_32 => 0b1000,
            Prescaler::Div_64 => 0b1001,
            Prescaler::Div_128 => 0b1010,
            Prescaler::Div_256 => 0b1011,
        }
    }
}

impl From<u8> for Prescaler {
    fn from(b: u8) -> Prescaler {
        match b {
            0b000 => Prescaler::Div_1,
            0b001 => Prescaler::Div_2,
            0b010 => Prescaler::Div_4,
            0b011 => Prescaler::Div_6,
            0b100 => Prescaler::Div_8,
            0b101 => Prescaler::Div_10,
            0b110 => Prescaler::Div_12,
            0b111 => Prescaler::Div_16,
            0b1000 => Prescaler::Div_32,
            0b1001 => Prescaler::Div_64,
            0b1010 => Prescaler::Div_128,
            0b1011 => Prescaler::Div_256,
            _ => unimplemented!(),
        }
    }
}

/// Resolution to sample at
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Debug, Copy, Clone)]
pub enum Resolution {
    /// 12-bit
    Twelve,
    /// 10-bit
    Ten,
    /// 8-bit
    Eight,
    /// 6-bit
    Six,
}
impl Resolution {
    /// Return the maximum value of a sample with the given Resolution
    pub fn to_max_sample(self) -> u32 {
        match self {
            Resolution::Twelve => (1 << 12) - 1,
            Resolution::Ten => (1 << 10) - 1,
            Resolution::Eight => (1 << 8) - 1,
            Resolution::Six => (1 << 6) - 1,
        }
    }
}
impl From<Resolution> for u8 {
    fn from(r: Resolution) -> u8 {
        match r {
            Resolution::Twelve => 0b00,
            Resolution::Ten => 0b01,
            Resolution::Eight => 0b10,
            Resolution::Six => 0b11,
        }
    }
}
impl From<u8> for Resolution {
    fn from(r: u8) -> Resolution {
        match r {
            0b00 => Resolution::Twelve,
            0b01 => Resolution::Ten,
            0b10 => Resolution::Eight,
            0b11 => Resolution::Six,
            _ => unimplemented!(),
        }
    }
}

/// Possible external triggers the ADC can listen to
///
/// This applies to ADC3, ADC4 and ADC5
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Debug, Clone, Copy, Default)]
pub enum ExternalTrigger12 {
    /// TIM1 compare channel 1
    #[default]
    Tim_1_cc_1,
    /// TIM1 compare channel 2
    Tim_1_cc_2,
    /// TIM1 compare channel 3
    Tim_1_cc_3,
    /// TIM2 compare channel 2
    Tim_2_cc_2,
    /// TIM3 trigger out
    Tim_3_trgo,
    /// TIM4 compare channel 4
    Tim_4_cc_4,
    /// External interupt line 11
    Exti_11,
    /// TIM8 trigger out
    Tim_8_trgo,
    /// TIM8 trigger out 2
    Tim_8_trgo_2,
    /// TIM1 trigger out
    Tim_1_trgo,
    /// TIM1 trigger out 2
    Tim_1_trgo_2,
    /// TIM2 trigger out
    Tim_2_trgo,
    /// TIM4 trigger out
    Tim_4_trgo,
    /// TIM6 trigger out
    Tim_6_trgo,
    /// TIM15 trigger out
    Tim_15_trgo,
    /// TIM3 compare channel 4
    Tim_3_cc_4,
    /// TIM20 trigger out
    Tim_20_trgo,
    /// TIM20 trigger out 2
    Tim_20_trgo_2,
    /// TIM20 compare channel 1
    Tim_20_cc_1,
    /// TIM20 compare channel 2
    Tim_20_cc_2,
    /// TIM20 compare channel 3
    Tim_20_cc_3,
    /// hrtim_adc_trg1
    Hrtim_adc_trg_1,
    /// hrtim_adc_trg3
    Hrtim_adc_trg_3,
    /// hrtim_adc_trg5
    Hrtim_adc_trg_5,
    /// hrtim_adc_trg6
    Hrtim_adc_trg_6,
    /// hrtim_adc_trg7
    Hrtim_adc_trg_7,
    /// hrtim_adc_trg8
    Hrtim_adc_trg_8,
    /// hrtim_adc_trg9
    Hrtim_adc_trg_9,
    /// hrtim_adc_trg10
    Hrtim_adc_trg_10,
    /// LP_timeout
    Lp_timeout,
    /// TIM7 trigger out
    Tim_7_trgo,
}

/// Possible external triggers the ADC can listen to
///
/// This applies to ADC3, ADC4 and ADC5
///
#[cfg(feature = "adc3")]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Debug, Clone, Copy, Default)]
pub enum ExternalTrigger345 {
    /// TIM3 compare channel 1
    #[default]
    Tim_3_cc_1,
    /// TIM2 compare channel 3
    Tim_2_cc_3,
    /// TIM1 compare channel 3
    Tim_1_cc_3,
    /// TIM8 compare channel 1
    Tim_8_cc_1,
    /// TIM3 trigger out
    Tim_3_trgo,
    /// External interupt line 2
    Exti_2,
    /// TIM4 compare channel 1
    Tim_4_cc_1,
    /// TIM8 trigger out
    Tim_8_trgo,
    /// TIM8 trigger out 2
    Tim_8_trgo_2,
    /// TIM1 trigger out
    Tim_1_trgo,
    /// TIM1 trigger out 2
    Tim_1_trgo_2,
    /// TIM2 trigger out
    Tim_2_trgo,
    /// TIM4 trigger out
    Tim_4_trgo,
    /// TIM6 trigger out
    Tim_6_trgo,
    /// TIM15 trigger out
    Tim_15_trgo,
    /// TIM2 compare channel 1
    Tim_2_cc_1,
    /// TIM20 trigger out
    Tim_20_trgo,
    /// TIM20 trigger out 2
    Tim_20_trgo_2,
    /// TIM20 compare channel 1
    Tim_20_cc_1,
    /// hrtim_adc_trg2
    Hrtim_adc_trg_2,
    /// hrtim_adc_trg4
    Hrtim_adc_trg_4,
    /// hrtim_adc_trg1
    Hrtim_adc_trg_1,
    /// hrtim_adc_trg3
    Hrtim_adc_trg_3,
    /// hrtim_adc_trg5
    Hrtim_adc_trg_5,
    /// hrtim_adc_trg6
    Hrtim_adc_trg_6,
    /// hrtim_adc_trg7
    Hrtim_adc_trg_7,
    /// hrtim_adc_trg8
    Hrtim_adc_trg_8,
    /// hrtim_adc_trg9
    Hrtim_adc_trg_9,
    /// hrtim_adc_trg10
    Hrtim_adc_trg_10,
    /// LP_timeout
    Lp_timeout,
    /// TIM7 trigger out
    Tim_7_trgo,
}

impl From<ExternalTrigger12> for u8 {
    fn from(et: ExternalTrigger12) -> u8 {
        match et {
            ExternalTrigger12::Tim_1_cc_1 => 0b00000,
            ExternalTrigger12::Tim_1_cc_2 => 0b00001,
            ExternalTrigger12::Tim_1_cc_3 => 0b00010,
            ExternalTrigger12::Tim_2_cc_2 => 0b00011,
            ExternalTrigger12::Tim_3_trgo => 0b00100,
            ExternalTrigger12::Tim_4_cc_4 => 0b00101,
            ExternalTrigger12::Exti_11 => 0b00110,
            ExternalTrigger12::Tim_8_trgo => 0b00111,
            ExternalTrigger12::Tim_8_trgo_2 => 0b01000,
            ExternalTrigger12::Tim_1_trgo => 0b01001,
            ExternalTrigger12::Tim_1_trgo_2 => 0b01010,
            ExternalTrigger12::Tim_2_trgo => 0b01011,
            ExternalTrigger12::Tim_4_trgo => 0b01100,
            ExternalTrigger12::Tim_6_trgo => 0b01101,
            ExternalTrigger12::Tim_15_trgo => 0b01110,
            ExternalTrigger12::Tim_3_cc_4 => 0b01111,
            ExternalTrigger12::Tim_20_trgo => 0b10000,
            ExternalTrigger12::Tim_20_trgo_2 => 0b10001,
            ExternalTrigger12::Tim_20_cc_1 => 0b10010,
            ExternalTrigger12::Tim_20_cc_2 => 0b10011,
            ExternalTrigger12::Tim_20_cc_3 => 0b10100,
            ExternalTrigger12::Hrtim_adc_trg_1 => 0b10101,
            ExternalTrigger12::Hrtim_adc_trg_3 => 0b10110,
            ExternalTrigger12::Hrtim_adc_trg_5 => 0b10111,
            ExternalTrigger12::Hrtim_adc_trg_6 => 0b11000,
            ExternalTrigger12::Hrtim_adc_trg_7 => 0b11001,
            ExternalTrigger12::Hrtim_adc_trg_8 => 0b11010,
            ExternalTrigger12::Hrtim_adc_trg_9 => 0b11011,
            ExternalTrigger12::Hrtim_adc_trg_10 => 0b11100,
            ExternalTrigger12::Lp_timeout => 0b11101,
            ExternalTrigger12::Tim_7_trgo => 0b11110,
            // Reserved => 0b11111
        }
    }
}

#[cfg(feature = "adc3")]
impl From<ExternalTrigger345> for u8 {
    fn from(et: ExternalTrigger345) -> u8 {
        match et {
            ExternalTrigger345::Tim_3_cc_1 => 0b00000,
            ExternalTrigger345::Tim_2_cc_3 => 0b00001,
            ExternalTrigger345::Tim_1_cc_3 => 0b00010,
            ExternalTrigger345::Tim_8_cc_1 => 0b00011,
            ExternalTrigger345::Tim_3_trgo => 0b00100,
            ExternalTrigger345::Exti_2 => 0b00101,
            ExternalTrigger345::Tim_4_cc_1 => 0b00110,
            ExternalTrigger345::Tim_8_trgo => 0b00111,
            ExternalTrigger345::Tim_8_trgo_2 => 0b01000,
            ExternalTrigger345::Tim_1_trgo => 0b01001,
            ExternalTrigger345::Tim_1_trgo_2 => 0b01010,
            ExternalTrigger345::Tim_2_trgo => 0b01011,
            ExternalTrigger345::Tim_4_trgo => 0b01100,
            ExternalTrigger345::Tim_6_trgo => 0b01101,
            ExternalTrigger345::Tim_15_trgo => 0b01110,
            ExternalTrigger345::Tim_2_cc_1 => 0b01111,
            ExternalTrigger345::Tim_20_trgo => 0b10000,
            ExternalTrigger345::Tim_20_trgo_2 => 0b10001,
            ExternalTrigger345::Tim_20_cc_1 => 0b10010,
            ExternalTrigger345::Hrtim_adc_trg_2 => 0b10011,
            ExternalTrigger345::Hrtim_adc_trg_4 => 0b10100,
            ExternalTrigger345::Hrtim_adc_trg_1 => 0b10101,
            ExternalTrigger345::Hrtim_adc_trg_3 => 0b10110,
            ExternalTrigger345::Hrtim_adc_trg_5 => 0b10111,
            ExternalTrigger345::Hrtim_adc_trg_6 => 0b11000,
            ExternalTrigger345::Hrtim_adc_trg_7 => 0b11001,
            ExternalTrigger345::Hrtim_adc_trg_8 => 0b11010,
            ExternalTrigger345::Hrtim_adc_trg_9 => 0b11011,
            ExternalTrigger345::Hrtim_adc_trg_10 => 0b11100,
            ExternalTrigger345::Lp_timeout => 0b11101,
            ExternalTrigger345::Tim_7_trgo => 0b11110,
            // Reserved => 0b11111
        }
    }
}

/// Possible oversampling shift
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Debug, Copy, Clone)]
pub enum OverSamplingShift {
    /// No right shift
    NoShift,
    /// Shift of 1 toward the right
    Shift_1,
    /// Shift of 2 toward the right
    Shift_2,
    /// Shift of 3 toward the right
    Shift_3,
    /// Shift of 4 toward the right
    Shift_4,
    /// Shift of 5 toward the right
    Shift_5,
    /// Shift of 6 toward the right
    Shift_6,
    /// Shift of 7 toward the right
    Shift_7,
    /// Shift of 8 toward the right
    Shift_8,
}
impl From<OverSamplingShift> for u8 {
    fn from(oss: OverSamplingShift) -> u8 {
        match oss {
            OverSamplingShift::NoShift => 0,
            OverSamplingShift::Shift_1 => 1,
            OverSamplingShift::Shift_2 => 2,
            OverSamplingShift::Shift_3 => 3,
            OverSamplingShift::Shift_4 => 4,
            OverSamplingShift::Shift_5 => 5,
            OverSamplingShift::Shift_6 => 6,
            OverSamplingShift::Shift_7 => 7,
            OverSamplingShift::Shift_8 => 8,
        }
    }
}

/// Possible oversampling modes
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Debug, Copy, Clone)]
pub enum OverSampling {
    /// Oversampling 2x
    Ratio_2,
    /// Oversampling 4x
    Ratio_4,
    /// Oversampling 8x
    Ratio_8,
    /// Oversampling 16x
    Ratio_16,
    /// Oversampling 32x
    Ratio_32,
    /// Oversampling 64x
    Ratio_64,
    /// Oversampling 128x
    Ratio_128,
    /// Oversampling 256x
    Ratio_256,
}
impl From<OverSampling> for u8 {
    fn from(os: OverSampling) -> u8 {
        match os {
            OverSampling::Ratio_2 => 0,
            OverSampling::Ratio_4 => 1,
            OverSampling::Ratio_8 => 2,
            OverSampling::Ratio_16 => 3,
            OverSampling::Ratio_32 => 4,
            OverSampling::Ratio_64 => 5,
            OverSampling::Ratio_128 => 6,
            OverSampling::Ratio_256 => 7,
        }
    }
}

/// Possible trigger modes
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Debug, Copy, Clone)]
pub enum TriggerMode {
    /// Don't listen to external trigger
    Disabled,
    /// Listen for rising edges of external trigger
    RisingEdge,
    /// Listen for falling edges of external trigger
    FallingEdge,
    /// Listen for both rising and falling edges of external trigger
    BothEdges,
}
impl From<TriggerMode> for u8 {
    fn from(tm: TriggerMode) -> u8 {
        match tm {
            TriggerMode::Disabled => 0,
            TriggerMode::RisingEdge => 1,
            TriggerMode::FallingEdge => 2,
            TriggerMode::BothEdges => 3,
        }
    }
}

/// Data register alignment
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Debug, Copy, Clone)]
pub enum Align {
    /// Right align output data
    Right,
    /// Left align output data
    Left,
}
impl From<Align> for bool {
    fn from(a: Align) -> bool {
        match a {
            Align::Right => false,
            Align::Left => true,
        }
    }
}

/// Continuous mode enable/disable
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum Continuous {
    /// Single mode, continuous disabled
    Single,
    /// Continuous mode enabled
    Continuous,

    /// Discontinuous mode enabled
    ///
    /// Will perform `subgroup_len` number samples per trigger
    Discontinuous,
}

/// Number of channels to sample per trigger in discontinuous mode
///
/// NOTE: This only applies to discontinuous
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Debug, Copy, Clone)]
pub enum SubGroupLength {
    /// One single sample per trigger
    One = 0b000,

    /// Two samples per trigger
    Two = 0b001,

    /// Three samples per trigger
    Three = 0b010,

    /// Four samples per trigger
    Four = 0b011,

    /// Five samples per trigger
    Five = 0b100,

    /// Six samples per trigger
    Six = 0b101,

    /// Seven samples per trigger
    Seven = 0b110,

    /// Eight samples per trigger
    Eight = 0b111,
}

/// DMA mode
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Debug, Copy, Clone)]
pub enum Dma {
    /// No DMA, disabled
    Disabled,
    /// Single DMA, DMA will be disabled after each conversion sequence
    Single,
    /// Continuous DMA, DMA will remain enabled after conversion
    Continuous,
}

/// End-of-conversion interrupt enabled/disabled
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Debug, Copy, Clone)]
pub enum Eoc {
    /// End-of-conversion interrupt disabled
    Disabled,
    /// End-of-conversion interrupt enabled per conversion
    Conversion,
    /// End-of-conversion interrupt enabled per sequence
    Sequence,
}

/// Input Type Selection
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Debug, Copy, Clone)]
pub enum InputType {
    /// Single-Ended Input Channels
    SingleEnded,
    /// Differential Input Channels
    Differential,
}
impl From<InputType> for bool {
    fn from(it: InputType) -> bool {
        match it {
            InputType::SingleEnded => false,
            InputType::Differential => true,
        }
    }
}

/// Sets the input type per channel
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Debug, Clone, Copy, Default)]
pub struct DifferentialSelection(pub(crate) u32);
impl DifferentialSelection {
    /// Set pin to Single-Ended or Differential
    pub fn set<PIN, ADC>(&mut self, it: InputType)
    where
        PIN: Channel<ADC, ID = u8>,
    {
        match it {
            InputType::SingleEnded => {
                self.singleended_by_id(PIN::channel());
            }
            InputType::Differential => {
                self.differential_by_id(PIN::channel());
            }
        }
    }

    /// Set to single ended by id
    fn singleended_by_id(&mut self, id: u8) {
        self.0 &= !(1 << id);
    }

    /// Set to differential by id
    fn differential_by_id(&mut self, id: u8) {
        self.0 |= 1 << id;
    }

    /// get the differential setting of channel
    pub fn get_channel(&self, channel: u8) -> InputType {
        if self.0 & (1 << channel) > 0 {
            InputType::Differential
        } else {
            InputType::SingleEnded
        }
    }

    /// Sets all channels to SingleEnded
    pub fn clear_all(&mut self) {
        self.0 = 0;
    }
}

/// Configuration for the adc.
/// There are some additional parameters on the adc peripheral that can be
/// added here when needed but this covers several basic usecases.
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Debug, Copy, Clone)]
pub struct AdcConfig<ET> {
    pub(crate) resolution: Resolution,
    pub(crate) align: Align,
    pub(crate) external_trigger: (TriggerMode, ET),
    pub(crate) continuous: Continuous,
    pub(crate) subgroup_len: SubGroupLength,
    pub(crate) dma: Dma,
    pub(crate) end_of_conversion_interrupt: Eoc,
    pub(crate) overrun_interrupt: bool,
    pub(crate) default_sample_time: SampleTime,
    pub(crate) vdda: Option<u32>,
    pub(crate) auto_delay: bool,

    /// Sets the differential input type of the Adc
    pub difsel: DifferentialSelection,
}

impl<ET: Copy> AdcConfig<ET> {
    /// change the resolution field
    #[inline(always)]
    pub fn resolution(mut self, resolution: Resolution) -> Self {
        self.resolution = resolution;
        self
    }
    /// change the align field
    #[inline(always)]
    pub fn align(mut self, align: Align) -> Self {
        self.align = align;
        self
    }
    /// change the continuous field
    #[inline(always)]
    pub fn continuous(mut self, continuous: Continuous) -> Self {
        self.continuous = continuous;
        self
    }

    /// change the subgroup_len field, only relevant in discontinous mode
    pub fn subgroup_len(mut self, subgroup_len: SubGroupLength) -> Self {
        self.subgroup_len = subgroup_len;
        self
    }

    /// change the dma field
    #[inline(always)]
    pub fn dma(mut self, dma: Dma) -> Self {
        self.dma = dma;
        self
    }
    /// change the end_of_conversion_interrupt field
    #[inline(always)]
    pub fn end_of_conversion_interrupt(mut self, end_of_conversion_interrupt: Eoc) -> Self {
        self.end_of_conversion_interrupt = end_of_conversion_interrupt;
        self
    }

    /// Enable/disable overrun interrupt
    ///
    /// This is triggered when the AD finishes a conversion before the last value was read by CPU/DMA
    pub fn overrun_interrupt(mut self, enable: bool) -> Self {
        self.overrun_interrupt = enable;
        self
    }

    /// change the default_sample_time field
    #[inline(always)]
    pub fn default_sample_time(mut self, default_sample_time: SampleTime) -> Self {
        self.default_sample_time = default_sample_time;
        self
    }

    /// Specify the reference voltage for the ADC.
    ///
    /// # Args
    /// * `vdda_mv` - The ADC reference voltage in millivolts.
    #[inline(always)]
    pub fn reference_voltage(mut self, vdda_mv: u32) -> Self {
        self.vdda = Some(vdda_mv);
        self
    }

    /// Specify the single-ened or differential channel selection
    #[inline(always)]
    pub fn difsel(mut self, df: DifferentialSelection) -> Self {
        self.difsel = df;
        self
    }

    /// Enable of disable the auto delay function
    #[inline(always)]
    pub fn auto_delay(mut self, delay: bool) -> Self {
        self.auto_delay = delay;
        self
    }
}

impl AdcConfig<ExternalTrigger12> {
    /// change the external_trigger field
    #[inline(always)]
    pub fn external_trigger(
        mut self,
        trigger_mode: TriggerMode,
        trigger: ExternalTrigger12,
    ) -> Self {
        self.external_trigger = (trigger_mode, trigger);
        self
    }
}

#[cfg(feature = "adc3")]
impl AdcConfig<ExternalTrigger345> {
    /// change the external_trigger field
    #[inline(always)]
    pub fn external_trigger(
        mut self,
        trigger_mode: TriggerMode,
        trigger: ExternalTrigger345,
    ) -> Self {
        self.external_trigger = (trigger_mode, trigger);
        self
    }
}

impl<ET: Default> Default for AdcConfig<ET> {
    fn default() -> Self {
        Self {
            resolution: Resolution::Twelve,
            align: Align::Right,
            external_trigger: (TriggerMode::Disabled, ET::default()),
            continuous: Continuous::Single,
            subgroup_len: SubGroupLength::One,
            dma: Dma::Disabled,
            end_of_conversion_interrupt: Eoc::Disabled,
            overrun_interrupt: false,
            default_sample_time: SampleTime::Cycles_640_5,
            vdda: None,
            difsel: DifferentialSelection::default(),
            auto_delay: false,
        }
    }
}
