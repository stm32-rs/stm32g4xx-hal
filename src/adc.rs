//! Analog to digital converter configuration.
//! https://github.com/stm32-rs/stm32l4xx-hal/blob/master/src/adc.rs

#![deny(missing_docs)]

/*
    Currently unused but this is the formula for using temperature calibration:
    Temperature in °C = ( ( (TS_CAL2_TEMP-TS_CAL1_TEMP) / (TS_CAL2-TS_CAL1) ) * (TS_DATA-TS_CAL1) ) + 30°C
*/

//use crate::dma::traits::PeriAddress;
pub use crate::time::U32Ext as _;
use crate::{
    dma::{mux::DmaMuxResources, traits::TargetAddress, PeripheralToMemory},
    gpio::*,
    opamp,
    rcc::{Enable, Rcc, Reset},
    signature::{VtempCal130, VtempCal30, VDDA_CALIB},
    stm32,
};
use core::fmt;
use core::marker::PhantomData;
use embedded_hal::{
    adc::{Channel, OneShot},
    blocking::delay::DelayUs,
};

use self::config::ExternalTrigger12;

#[cfg(any(
    feature = "stm32g471",
    feature = "stm32g473",
    feature = "stm32g474",
    feature = "stm32g483",
    feature = "stm32g484",
    feature = "stm32g491",
    feature = "stm32g4a1",
))]
use self::config::ExternalTrigger345;

/// Vref internal signal, used for calibration
pub struct Vref;
impl Vref {
    /// Converts a sample value to millivolts using calibrated VDDA and configured resolution
    #[inline(always)]
    pub fn sample_to_millivolts_ext(sample: u16, vdda: u32, resolution: config::Resolution) -> u16 {
        let mx_s = resolution.to_max_sample();
        ((u32::from(sample) * vdda) / mx_s) as u16
    }
    /// Converts a sample value to millivolts using calibrated VDDA and configured resolution
    #[inline(always)]
    pub fn sample_to_millivolts(sample: u16) -> u16 {
        Self::sample_to_millivolts_ext(sample, VDDA_CALIB, config::Resolution::Twelve)
    }
}

/// Vbat internal signal, used for monitoring the battery (if used)
pub struct Vbat;

/// Core temperature internal signal
pub struct Temperature;
impl Temperature {
    /// Precompute the inverse of `VTEMP_CAL_VREFANALOG`, in volts,
    /// for floating point calculations
    const INV_VREFANALOG_VOLTS: f32 = 1000. / VDDA_CALIB as f32;
    /// Temperature at which temperature sensor has been calibrated in production
    /// for data into [`VtempCal30`] (tolerance: +-5 DegC) (unit: DegC).
    const VTEMP_CAL_T30: u16 = 30;
    /// Temperature at which temperature sensor has been calibrated in production
    /// for data into [`VtempCal130`] (tolerance: +-5 DegC) (unit: DegC).
    const VTEMP_CAL_T130: u16 = 130;

    /// Convert a sample to 12 bits. Reference voltages were captured at 12 bits.
    const fn to_12b(sample: u16, resolution: config::Resolution) -> u16 {
        match resolution {
            config::Resolution::Six => sample << 6,
            config::Resolution::Eight => sample << 4,
            config::Resolution::Ten => sample << 2,
            config::Resolution::Twelve => sample,
        }
    }

    /// Convert a raw sample from `Temperature` to deg C.
    ///
    /// ## Arguments
    /// * `sample`: ADC sample taken on the [`Temperature`] channel.
    /// * `vdda`: Analog reference voltage (vref+) when the temperature
    /// sample was taken, in volts.
    /// * `resolution`: Configured ADC resolution.
    #[inline(always)]
    pub fn temperature_to_degrees_centigrade(
        sample: u16,
        vdda: f32,
        resolution: config::Resolution,
    ) -> f32 {
        // Reference measurements were taken at 12 bits
        let sample_12b = Self::to_12b(sample, resolution);

        // Normalize for the difference in VDDA
        let sample_normalized = sample_12b as f32 * (vdda * Self::INV_VREFANALOG_VOLTS);

        ((sample_normalized - VtempCal30::get().read() as f32)
            * ((Self::VTEMP_CAL_T130 - Self::VTEMP_CAL_T30) as f32))
            / ((VtempCal130::get().read() - VtempCal30::get().read()) as f32)
            + Self::VTEMP_CAL_T30 as f32
    }

    /// Convert a raw sample from `Temperature` to deg C
    ///
    /// ## Arguments
    /// * `sample`: ADC sample taken on the [`Temperature`] channel.
    /// * `vdda`: Analog reference voltage (vref+) when the temperature
    /// sample was taken, in millivolts.
    /// * `resolution`: Configured ADC resolution.
    #[inline(always)]
    pub fn temperature_to_degrees_centigrade_coarse(
        sample: u16,
        vdda: u32,
        resolution: config::Resolution,
    ) -> i16 {
        // Reference measurements were taken at 12 bits
        let sample_12b = Self::to_12b(sample, resolution);

        // Normalize for the difference in VDDA
        let sample_normalized = ((sample_12b as u32 * vdda) / VDDA_CALIB) as u16;

        let t = ((sample_normalized as i32 - VtempCal30::get().read() as i32)
            * ((Self::VTEMP_CAL_T130 - Self::VTEMP_CAL_T30) as i32))
            / ((VtempCal130::get().read() - VtempCal30::get().read()) as i32)
            + Self::VTEMP_CAL_T30 as i32;

        t as i16
    }
}

macro_rules! adc_pins {
    ($($pin:ty => ($adc:ident, $chan:expr)),+ $(,)*) => {
        $(
            impl Channel<stm32::$adc> for $pin {
                type ID = u8;
                fn channel() -> u8 { $chan }
            }
        )+
    };
}

macro_rules! adc_op_pga {
    ($($opamp:ty => ($adc:ident, $chan:expr)),+ $(,)*) => {
        $(
            impl<A, B> Channel<stm32::$adc> for $opamp {
                type ID = u8;
                fn channel() -> u8 { $chan }
            }
        )+
    };
}

macro_rules! adc_op_follower {
    ($($opamp:ty => ($adc:ident, $chan:expr)),+ $(,)*) => {
        $(
            impl<A> Channel<stm32::$adc> for $opamp {
                type ID = u8;
                fn channel() -> u8 { $chan }
            }
        )+
    };
}

/// Contains types related to ADC configuration
pub mod config {
    use embedded_hal::adc::Channel;

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

    /// ClockMode config for the ADC
    /// Check the datasheet for the maximum speed the ADC supports
    #[derive(Debug, Clone, Copy)]
    pub enum ClockMode {
        /// (Asynchronous clock mode), adc_ker_ck. generated at product level (refer to Section 6: Reset and clock control (RCC)
        Asynchronous,
        /// (Synchronous clock mode). adc_hclk/1 This configuration must be enabled only if the AHB clock prescaler is set to 1 (HPRE[3:0] = 0xxx in RCC_CFGR register) and if the system clock has a 50% duty cycle.
        Synchronous_Div_1,
        /// Synchronous clock mode. adc_hclk/2
        Synchronous_Div_2,
        /// Synchronous clock mode. adc_hclk/4
        Synchronous_Div_4,
    }

    impl From<ClockMode> for u8 {
        fn from(c: ClockMode) -> u8 {
            match c {
                ClockMode::Asynchronous => 0b00,
                ClockMode::Synchronous_Div_1 => 0b001,
                ClockMode::Synchronous_Div_2 => 0b010,
                ClockMode::Synchronous_Div_4 => 0b011,
            }
        }
    }

    impl From<u8> for ClockMode {
        fn from(b: u8) -> ClockMode {
            match b {
                0b000 => ClockMode::Asynchronous,
                0b001 => ClockMode::Synchronous_Div_1,
                0b010 => ClockMode::Synchronous_Div_2,
                0b011 => ClockMode::Synchronous_Div_4,
                _ => unimplemented!(),
            }
        }
    }

    /// Clock config for the ADC
    /// Check the datasheet for the maximum speed the ADC supports
    #[derive(Debug, Clone, Copy)]
    pub enum Clock {
        /// Clock not divided
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

    impl From<Clock> for u8 {
        fn from(c: Clock) -> u8 {
            match c {
                Clock::Div_1 => 0b000,
                Clock::Div_2 => 0b001,
                Clock::Div_4 => 0b010,
                Clock::Div_6 => 0b011,
                Clock::Div_8 => 0b100,
                Clock::Div_10 => 0b101,
                Clock::Div_12 => 0b110,
                Clock::Div_16 => 0b111,
                Clock::Div_32 => 0b1000,
                Clock::Div_64 => 0b1001,
                Clock::Div_128 => 0b1010,
                Clock::Div_256 => 0b1011,
            }
        }
    }

    impl From<u8> for Clock {
        fn from(b: u8) -> Clock {
            match b {
                0b000 => Clock::Div_1,
                0b001 => Clock::Div_2,
                0b010 => Clock::Div_4,
                0b011 => Clock::Div_6,
                0b100 => Clock::Div_8,
                0b101 => Clock::Div_10,
                0b110 => Clock::Div_12,
                0b111 => Clock::Div_16,
                0b1000 => Clock::Div_32,
                0b1001 => Clock::Div_64,
                0b1010 => Clock::Div_128,
                0b1011 => Clock::Div_256,
                _ => unimplemented!(),
            }
        }
    }

    /// Resolution to sample at
    #[derive(Debug, Clone, Copy)]
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
    #[cfg(any(
        feature = "stm32g471",
        feature = "stm32g473",
        feature = "stm32g474",
        feature = "stm32g483",
        feature = "stm32g484",
        feature = "stm32g491",
        feature = "stm32g4a1",
    ))]
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

    #[cfg(any(
        feature = "stm32g471",
        feature = "stm32g473",
        feature = "stm32g474",
        feature = "stm32g483",
        feature = "stm32g484",
        feature = "stm32g491",
        feature = "stm32g4a1",
    ))]
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
    #[derive(Debug, Clone, Copy)]
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
    #[derive(Debug, Clone, Copy)]
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
    #[derive(Debug, Clone, Copy)]
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
    #[derive(Debug, Clone, Copy)]
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
    #[derive(Debug, Clone, Copy)]
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
    #[derive(Debug, Clone, Copy)]
    pub enum Dma {
        /// No DMA, disabled
        Disabled,
        /// Single DMA, DMA will be disabled after each conversion sequence
        Single,
        /// Continuous DMA, DMA will remain enabled after conversion
        Continuous,
    }

    /// End-of-conversion interrupt enabled/disabled
    #[derive(Debug, Clone, Copy)]
    pub enum Eoc {
        /// End-of-conversion interrupt disabled
        Disabled,
        /// End-of-conversion interrupt enabled per conversion
        Conversion,
        /// End-of-conversion interrupt enabled per sequence
        Sequence,
    }

    /// Input Type Selection
    #[derive(Debug, Clone, Copy)]
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
    #[derive(Debug, Clone, Copy)]
    pub struct AdcConfig<ET> {
        pub(crate) clock_mode: ClockMode,
        pub(crate) clock: Clock,
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

    impl<ET> AdcConfig<ET> {
        /// change the clock_mode field
        #[inline(always)]
        pub fn clock_mode(mut self, clock_mode: ClockMode) -> Self {
            self.clock_mode = clock_mode;
            self
        }
        /// change the clock field
        #[inline(always)]
        pub fn clock(mut self, clock: Clock) -> Self {
            self.clock = clock;
            self
        }
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

    #[cfg(any(
        feature = "stm32g471",
        feature = "stm32g473",
        feature = "stm32g474",
        feature = "stm32g483",
        feature = "stm32g484",
        feature = "stm32g491",
        feature = "stm32g4a1",
    ))]
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
                clock_mode: ClockMode::Synchronous_Div_1,
                clock: Clock::Div_2,
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
}

/// Type-State for Adc, indicating a deep-powered-down-pheripheral
#[derive(Debug)]
pub struct PoweredDown;
/// Type-State for Adc, indicating a non-configured peripheral
#[derive(Debug)]
pub struct Disabled;
/// Type-State for Adc, indicating a configured peripheral
#[derive(Debug)]
pub struct Configured;
/// Type-State for Adc, indicating an peripheral configured for DMA
#[derive(Debug)]
pub struct DMA;
/// Type-State for Adc, indicating am active measuring peripheral
#[derive(Debug)]
pub struct Active;

/// Enum for the wait_for_conversion_sequence function,
/// which can return either a stopped ADC typestate or a
/// continuing ADC typestate.
pub enum Conversion<ADC: TriggerType> {
    /// Contains an Active Conversion ADC
    Active(Adc<ADC, Active>),
    /// Contains an Stopped ADC
    Stopped(Adc<ADC, Configured>),
}
impl<ADC: TriggerType> Conversion<ADC> {
    /// unwraps the enum and panics if the result is not an Active ADC
    #[inline(always)]
    pub fn unwrap_active(self) -> Adc<ADC, Active> {
        match self {
            Conversion::Active(adc) => adc,
            _ => {
                panic!("Conversion is Stopped, not Active!")
            }
        }
    }

    /// unwraps the enum and panics if the result is not a Stopped ADC
    #[inline(always)]
    pub fn unwrap_stopped(self) -> Adc<ADC, Configured> {
        match self {
            Conversion::Stopped(adc) => adc,
            _ => {
                panic!("Conversion is Continuing, not Stopped!")
            }
        }
    }

    /// Returns true if the adc is Active.
    #[inline(always)]
    pub fn is_active(&self) -> bool {
        match *self {
            Conversion::Active(..) => true,
            Conversion::Stopped(..) => false,
        }
    }

    /// Returns true if the adc is Stopped.
    #[inline(always)]
    pub fn is_stopped(&self) -> bool {
        !self.is_active()
    }

    /// Converts from Conversion<C, E> to Option<C>.
    ///
    /// Converts self into an Option<C>, consuming self, and discarding the adc, if it is stopped.
    #[inline(always)]
    pub fn active(self) -> Option<Adc<ADC, Active>> {
        match self {
            Conversion::Active(adc) => Some(adc),
            Conversion::Stopped(..) => None,
        }
    }

    /// Converts from Conversion<C, E> to Option<E>.
    ///
    /// Converts self into an Option<E>, consuming self, and discarding the adc, if it is still active.
    #[inline(always)]
    pub fn stopped(self) -> Option<Adc<ADC, Configured>> {
        match self {
            Conversion::Active(..) => None,
            Conversion::Stopped(adc) => Some(adc),
        }
    }
}

/// Analog to Digital Converter
/// # Status
/// Most options relating to regular conversions are implemented. One-shot and sequences of conversions
/// have been tested and work as expected.
///
/// GPIO to channel mapping should be correct for all supported F4 devices. The mappings were taken from
/// CubeMX. The mappings are feature gated per 4xx device but there are actually sub variants for some
/// devices and some pins may be missing on some variants. The implementation has been split up and commented
/// to show which pins are available on certain device variants but currently the library doesn't enforce this.
/// To fully support the right pins would require 10+ more features for the various variants.
/// ## Todo
/// * Injected conversions
/// * Analog watchdog config
/// * Discontinuous mode
/// # Examples
/// ## One-shot conversion
/// ```
/// use stm32f4xx_hal::{
///   gpio::gpioa,
///   adc::{
///     Adc,
///     config::AdcConfig,
///     config::SampleTime,
///   },
/// };
///
/// let mut adc = Adc::adc1(device.ADC1, true, AdcConfig::default());
/// let pa3 = gpioa.pa3.into_analog();
/// let sample = adc.convert(&pa3, SampleTime::Cycles_480);
/// let millivolts = adc.sample_to_millivolts(sample);
/// info!("pa3: {}mV", millivolts);
/// ```
///
/// ## Sequence conversion
/// ```
/// use stm32f4xx_hal::{
///   gpio::gpioa,
///   adc::{
///     Adc,
///     config::AdcConfig,
///     config::SampleTime,
///     config::Sequence,
///     config::Eoc,
///     config::Clock,
///   },
/// };
///
/// let config = AdcConfig::default()
///     //We'll either need DMA or an interrupt per conversion to convert
///     //multiple values in a sequence
///     .end_of_conversion_interrupt(Eoc::Conversion)
///     //And since we're looking for one interrupt per conversion the
///     //clock will need to be fairly slow to avoid overruns breaking
///     //the sequence. If you are running in debug mode and logging in
///     //the interrupt, good luck... try setting pclk2 really low.
///     //(Better yet use DMA)
///     .clock(Clock::Pclk2_div_8);
/// let mut adc = Adc::adc1(device.ADC1, true, config);
/// let pa0 = gpioa.pa0.into_analog();
/// let pa3 = gpioa.pa3.into_analog();
/// adc.configure_channel(&pa0, Sequence::One, SampleTime::Cycles_112);
/// adc.configure_channel(&pa3, Sequence::Two, SampleTime::Cycles_480);
/// adc.configure_channel(&pa0, Sequence::Three, SampleTime::Cycles_112);
/// adc.start_conversion();
/// ```
///
/// ## External trigger
///
/// A common mistake on STM forums is enabling continuous mode but that causes it to start
/// capturing on the first trigger and capture as fast as possible forever, regardless of
/// future triggers. Continuous mode is disabled by default but I thought it was worth
/// highlighting.
///
/// Getting the timer config right to make sure it's sending the event the ADC is listening
/// to can be a bit of a pain but the key fields are highlighted below. Try hooking a timer
/// channel up to an external pin with an LED or oscilloscope attached to check it's really
/// generating pulses if the ADC doesn't seem to be triggering.
/// ```
/// use stm32f4xx_hal::{
///   gpio::gpioa,
///   adc::{
///     Adc,
///     config::AdcConfig,
///     config::SampleTime,
///     config::Sequence,
///     config::Eoc,
///     config::Clock,
///   },
/// };
///
///  let config = AdcConfig::default()
///      //Set the trigger you want
///      .external_trigger(TriggerMode::RisingEdge, ExternalTrigger::Tim_1_cc_1);
///  let mut adc = Adc::adc1(device.ADC1, true, config);
///  let pa0 = gpioa.pa0.into_analog();
///  adc.configure_channel(&pa0, Sequence::One, SampleTime::Cycles_112);
///  //Make sure it's enabled but don't start the conversion
///  adc.enable();
///
/// //Configure the timer
/// let mut tim = Timer::tim1(device.TIM1, 1.hz(), clocks);
/// unsafe {
///     let tim = &(*TIM1::ptr());
///
///     //Channel 1
///     //Disable the channel before configuring it
///     tim.ccer.modify(|_, w| w.cc1e().clear_bit());
///
///     tim.ccmr1_output().modify(|_, w| w
///       //Preload enable for channel
///       .oc1pe().set_bit()
///
///       //Set mode for channel, the default mode is "frozen" which won't work
///       .oc1m().pwm_mode1()
///     );
///
///     //Set the duty cycle, 0 won't work in pwm mode but might be ok in
///     //toggle mode or match mode
///     let max_duty = tim.arr.read().arr().bits() as u16;
///     tim.ccr1.modify(|_, w| w.ccr().bits(max_duty / 2));
///
///     //Enable the channel
///     tim.ccer.modify(|_, w| w.cc1e().set_bit());
///
///     //Enable the TIM main Output
///     tim.bdtr.modify(|_, w| w.moe().set_bit());
/// }
/// ```
#[derive(Clone, Copy)]
pub struct DynamicAdc<ADC: TriggerType> {
    /// Current config of the ADC, kept up to date by the various set methods
    config: config::AdcConfig<ADC::ExternalTrigger>,
    /// The adc peripheral
    adc_reg: ADC,
    /// VDDA in millivolts calculated from the factory calibration and vrefint
    calibrated_vdda: u32,
}
impl<ADC: TriggerType> fmt::Debug for DynamicAdc<ADC> {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        write!(
            f,
            "DynamicAdc: {{ calibrated_vdda: {:?}, {:?}, ... }}",
            self.calibrated_vdda, self.config
        )
    }
}

/// Typestate wrapper around DynamicAdc
pub struct Adc<ADC: TriggerType, STATUS> {
    adc: DynamicAdc<ADC>,
    _status: PhantomData<STATUS>,
}
impl<ADC: TriggerType, STATUS> fmt::Debug for Adc<ADC, STATUS>
where
    STATUS: fmt::Debug,
{
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        write!(
            f,
            "Adc<{:?}>: {{ calibrated_vdda: {:?}, {:?}, ... }}",
            self._status, self.adc.calibrated_vdda, self.adc.config
        )
    }
}

/// ADC Clock Source selection
#[derive(Debug, Clone, Copy)]
pub enum ClockSource {
    /// Use the System Clock as Clock Source
    SystemClock,
    /// use the Internal PLL as Clock Source
    PLL_P,
}

impl From<ClockSource> for u8 {
    fn from(c: ClockSource) -> u8 {
        match c {
            ClockSource::PLL_P => 0b01,
            ClockSource::SystemClock => 0b10,
        }
    }
}

/// used to create an ADC instance from the stm32::Adc
pub trait AdcClaim<TYPE: TriggerType> {
    /// create a disabled ADC instance from the stm32::Adc
    fn claim(
        self,
        cs: ClockSource,
        rcc: &Rcc,
        delay: &mut impl DelayUs<u8>,
        reset: bool,
    ) -> Adc<TYPE, Disabled>;

    /// create an enabled ADC instance from the stm32::Adc
    fn claim_and_configure(
        self,
        cs: ClockSource,
        rcc: &Rcc,
        config: config::AdcConfig<TYPE::ExternalTrigger>,
        delay: &mut impl DelayUs<u8>,
        reset: bool,
    ) -> Adc<TYPE, Configured>;
}

trait AdcConfig {
    fn configure_clock_source(cs: ClockSource, rcc: &Rcc);
}

/// Specifies what External trigger type the ADC uses
pub trait TriggerType {
    /// Specifies what External trigger type the ADC uses
    type ExternalTrigger: fmt::Debug;
}

#[inline(always)]
fn configure_clock_source12(cs: ClockSource, rcc: &Rcc) {
    // Select system clock as ADC clock source
    rcc.rb.ccipr.modify(|_, w| {
        // This is sound, as `0b10` is a valid value for this field.
        unsafe {
            w.adc12sel().bits(cs.into());
        }

        w
    });
}

#[inline(always)]
#[allow(dead_code)]
fn configure_clock_source345(cs: ClockSource, rcc: &Rcc) {
    // Select system clock as ADC clock source
    rcc.rb.ccipr.modify(|_, w| {
        // This is sound, as `0b10` is a valid value for this field.
        unsafe {
            w.adc345sel().bits(cs.into());
        }

        w
    });
}

macro_rules! adc {

    (vbat => ($common_type:ident)) => {
        /// Enables the vbat internal channel
        #[inline(always)]
        pub fn enable_vbat(&self, common: &stm32::$common_type) {
            self.adc.enable_vbat(common)
        }

        /// Enables the vbat internal channel
        #[inline(always)]
        pub fn disable_vbat(&self, common: &stm32::$common_type) {
            self.adc.disable_vbat(common)
        }
    };

    (vbat_check => ($common_type:ident)) => {
        /// Returns if the vbat internal channel is enabled
        #[inline(always)]
        pub fn is_vbat_enabled(&mut self, common: &stm32::$common_type) -> bool {
            self.adc.is_vbat_enabled(common)
        }
    };

    (vtemp => ($common_type:ident)) => {
        /// Enables the temp internal channel.
        #[inline(always)]
        pub fn enable_temperature(&mut self, common: &stm32::$common_type) {
            self.adc.enable_temperature(common)
        }

        /// Disables the temp internal channel
        #[inline(always)]
        pub fn disable_temperature(&mut self, common: &stm32::$common_type) {
            self.adc.disable_temperature(common)
        }
    };

    (vtemp_check => ($common_type:ident)) => {
        /// Returns if the temp internal channel is enabled
        #[inline(always)]
        pub fn is_temperature_enabled(&mut self, common: &stm32::$common_type) -> bool {
            self.adc.is_temperature_enabled(common)
        }
    };

    (vref => ($common_type:ident)) => {
        /// Enables the vref internal channel.
        #[inline(always)]
        pub fn enable_vref(&mut self, common: &stm32::$common_type) {
            self.adc.enable_vref(common)
        }

        /// Disables the vref internal channel
        #[inline(always)]
        pub fn disable_vref(&mut self, common: &stm32::$common_type) {
            self.adc.disable_vref(common)
        }
    };

    (vref_check => ($common_type:ident)) => {
        /// Returns if the vref internal channel is enabled
        #[inline(always)]
        pub fn is_vref_enabled(&mut self, common: &stm32::$common_type) -> bool {
            self.adc.is_vref_enabled(common)
        }
    };

    // Note that ADC1 supports measurement of VREF, VBAT, and the internal temperature sensor.
    (additionals: ADC1 => ($common_type:ident)) => {
        adc!(vbat => ($common_type));
        adc!(vtemp => ($common_type));
        adc!(vref => ($common_type));
    };

    (additionals_checks: ADC1 => ($common_type:ident)) => {
        adc!(vbat_check => ($common_type));
        adc!(vtemp_check => ($common_type));
        adc!(vref_check => ($common_type));
    };


    (additionals: ADC2 => ($common_type:ident)) => {
    };

    (additionals_checks: ADC2 => ($common_type:ident)) => {
    };

    (additionals: ADC3 => ($common_type:ident)) => {
        adc!(vbat=> ($common_type));
        adc!(vref=> ($common_type));
    };

    (additionals_checks: ADC3 => ($common_type:ident)) => {
        adc!(vbat_check => ($common_type));
        adc!(vref_check => ($common_type));
    };

    (additionals: ADC4 => ($common_type:ident)) => {
        adc!(vref => ($common_type));
    };

    (additionals_checks: ADC4 => ($common_type:ident)) => {
        adc!(vref_check => ($common_type));
    };

    (additionals: ADC5 => ($common_type:ident)) => {
        adc!(vbat => ($common_type));
        adc!(vref => ($common_type));
    };

    (additionals_checks: ADC5 => ($common_type:ident)) => {
        adc!(vbat_check => ($common_type));
        adc!(vref_check => ($common_type));
    };

    // Provide a stub implementation for ADCs that do not have a means of sampling VREF.
    (additionals: $adc_type:ident => ($common_type:ident)) => {
    };

    ($($adc_type:ident => ($trigger_type:ident, $configure_clocks_fn_name:ident, $mux:expr, ($common_type:ident) )),+ $(,)*) => {
        $(
            impl TriggerType for stm32::$adc_type {
                type ExternalTrigger = $trigger_type;
            }

            impl AdcConfig for stm32::$adc_type {
                #[inline(always)]
                fn configure_clock_source(cs: ClockSource, rcc: &Rcc) {
                    $configure_clocks_fn_name(cs, rcc);
                }
            }

            impl DynamicAdc<stm32::$adc_type> {
                /// Converts a sample value to millivolts using calibrated VDDA and configured resolution
                #[inline(always)]
                pub fn sample_to_millivolts(&self, sample: u16) -> u16 {
                    Vref::sample_to_millivolts_ext(sample, self.calibrated_vdda, self.config.resolution)
                }

                /// Disables the Voltage Regulator and release the ADC
                #[inline(always)]
                pub fn release(mut self) -> stm32::$adc_type {
                    self.enable_deeppwd_down();

                    self.adc_reg
                }

                /// Powers-up an powered-down Adc
                #[inline(always)]
                pub fn power_up(&mut self, delay: &mut impl DelayUs<u8>) {
                    if self.is_deeppwd_enabled() {
                        self.disable_deeppwd_down();
                    }
                    if !self.is_vreg_enabled() {
                        self.enable_vreg(delay);
                    }
                    if self.is_enabled() {
                        self.disable();
                    }
                }

                /// Puts a Disabled Adc into Powered Mode
                #[inline(always)]
                pub fn power_down(&mut self) {
                    self.disable_vreg();
                }

                /// Enables the Deep Power Down Modus
                #[inline(always)]
                pub fn enable_deeppwd_down(&mut self) {
                    self.adc_reg.cr.modify(|_, w| w.deeppwd().set_bit());
                }

                /// Disables the Deep Power Down Modus
                #[inline(always)]
                pub fn disable_deeppwd_down(&mut self) {
                    self.adc_reg.cr.modify(|_, w| w.deeppwd().clear_bit());
                }


                /// Enables the Voltage Regulator
                #[inline(always)]
                pub fn enable_vreg(&mut self, delay: &mut impl DelayUs<u8>) {
                    self.adc_reg.cr.modify(|_, w| w.advregen().set_bit());
                    while !self.adc_reg.cr.read().advregen().bit_is_set() {}

                    // According to the STM32G4xx Reference Manual, section 21.4.6, we need
                    // to wait for T_ADCVREG_STUP after enabling the internal voltage
                    // regulator. For the STM32G431, this is 20 us. We choose 25 us to
                    // account for bad clocks.
                    delay.delay_us(25);
                }

                /// Disables the Voltage Regulator
                #[inline(always)]
                pub fn disable_vreg(&mut self) {
                    self.adc_reg.cr.modify(|_, w| w.advregen().clear_bit());
                }

                /// Returns if the ADC is enabled (ADEN)
                #[inline(always)]
                pub fn is_enabled(&self) -> bool {
                    self.adc_reg.cr.read().aden().bit_is_set()
                }

                /// Disables the adc, since we don't know in what state we get it.
                #[inline(always)]
                pub fn disable(&mut self) {
                    // Disable any ongoing conversions
                    self.cancel_conversion();

                    // Turn off ADC
                    self.adc_reg.cr.modify(|_, w| w.addis().set_bit());
                    while self.adc_reg.cr.read().addis().bit_is_set() {}

                    // Wait until the ADC has turned off
                    while self.adc_reg.cr.read().aden().bit_is_set() {}
                }

                /// Enables the adc
                #[inline(always)]
                pub fn enable(&mut self) {
                    self.calibrate_all();
                    self.apply_config(self.config);

                    self.adc_reg.isr.modify(|_, w| w.adrdy().set_bit());
                    self.adc_reg.cr.modify(|_, w| w.aden().set_bit());

                    // Wait for adc to get ready
                    while !self.adc_reg.isr.read().adrdy().bit_is_set() {}

                    // Clear ready flag
                    self.adc_reg.isr.modify(|_, w| w.adrdy().set_bit());

                    self.clear_end_of_conversion_flag();
                }

                /// enable the adc and configure for DMA.
                pub fn enable_dma(&mut self, dma: config::Dma) {
                    self.set_dma(dma);
                    self.enable();
                }

                /// Applies all fields in AdcConfig
                #[inline(always)]
                fn apply_config(&mut self, config: config::AdcConfig<$trigger_type>) {
                    self.set_clock_mode(config.clock_mode);
                    self.set_clock(config.clock);
                    self.set_resolution(config.resolution);
                    self.set_align(config.align);
                    self.set_external_trigger(config.external_trigger);
                    self.set_continuous(config.continuous);
                    self.set_subgroup_len(config.subgroup_len);
                    self.set_dma(config.dma);
                    self.set_end_of_conversion_interrupt(config.end_of_conversion_interrupt);
                    self.set_overrun_interrupt(config.overrun_interrupt);
                    self.set_default_sample_time(config.default_sample_time);
                    self.set_channel_input_type(config.difsel);
                    self.set_auto_delay(config.auto_delay);

                    if let Some(vdda) = config.vdda {
                        self.calibrated_vdda = vdda;
                    }
                }

                /// Sets the clock_mode for the adc
                #[inline(always)]
                pub fn set_clock_mode(&mut self, clock_mode: config::ClockMode) {
                    self.config.clock_mode = clock_mode;
                    unsafe {
                        let common = &(*stm32::$common_type::ptr());
                        common.ccr.modify(|_, w| w.ckmode().bits(clock_mode.into()));
                    }
                }

                /// Sets the clock for the adc
                #[inline(always)]
                pub fn set_clock(&mut self, clock: config::Clock) {
                    self.config.clock = clock;
                    unsafe {
                        let common = &(*stm32::$common_type::ptr());
                        common.ccr.modify(|_, w| w.presc().bits(clock.into()));
                    }
                }

                /// Sets the sampling resolution
                #[inline(always)]
                pub fn set_resolution(&mut self, resolution: config::Resolution) {
                    self.config.resolution = resolution;
                    self.adc_reg.cfgr.modify(|_, w| w.res().bits(resolution.into()));
                }


                /// Enable oversampling
                #[inline(always)]
                pub fn set_oversampling(&mut self, oversampling: config::OverSampling, shift: config::OverSamplingShift) {
                    self.adc_reg.cfgr2.modify(|_, w| unsafe { w.ovsr().bits(oversampling.into())
                                                      .ovss().bits(shift.into())
                                                      .rovse().set_bit()});
                }

                /// Sets the DR register alignment to left or right
                #[inline(always)]
                pub fn set_align(&mut self, align: config::Align) {
                    self.config.align = align;
                    self.adc_reg.cfgr.modify(|_, w| w.align().bit(align.into()));
                }

                /// Sets which external trigger to use and if it is disabled, rising, falling or both
                #[inline(always)]
                pub fn set_external_trigger(&mut self, (edge, extsel): (config::TriggerMode, $trigger_type)) {
                    self.config.external_trigger = (edge, extsel);
                    self.adc_reg.cfgr.modify(|_, w| unsafe { w
                        .extsel().bits(extsel.into())
                        .exten().bits(edge.into())
                    });
                }

                /// Sets auto delay to true or false
                #[inline(always)]
                pub fn set_auto_delay(&mut self, delay: bool) {
                    self.config.auto_delay = delay;
                    self.adc_reg.cfgr.modify(|_, w| w.autdly().bit(delay) );
                }

                /// Enables and disables dis-/continuous mode
                #[inline(always)]
                pub fn set_continuous(&mut self, continuous: config::Continuous) {
                    self.config.continuous = continuous;
                    self.adc_reg.cfgr.modify(|_, w| w
                        .cont().bit(continuous == config::Continuous::Continuous)
                        .discen().bit(continuous == config::Continuous::Discontinuous)
                    );
                }

                #[inline(always)]
                // NOTE: The software is allowed to write these bits only when ADSTART = 0
                fn set_subgroup_len(&mut self, subgroup_len: config::SubGroupLength) {
                    self.config.subgroup_len = subgroup_len;
                    self.adc_reg.cfgr.modify(|_, w| w.discnum().bits(subgroup_len as u8))
                }

                /// Sets DMA to disabled, single or continuous
                #[inline(always)]
                pub fn set_dma(&mut self, dma: config::Dma) {
                    self.config.dma = dma;
                    let (dds, en) = match dma {
                        config::Dma::Disabled => (false, false),
                        config::Dma::Single => (false, true),
                        config::Dma::Continuous => (true, true),
                    };
                    self.adc_reg.cfgr.modify(|_, w| w
                        //DDS stands for "DMA disable selection"
                        //0 means do one DMA then stop
                        //1 means keep sending DMA requests as long as DMA=1
                        .dmacfg().bit(dds)
                        .dmaen().bit(en)
                    );
                }

                /// Sets if the end-of-conversion behaviour.
                /// The end-of-conversion interrupt occur either per conversion or for the whole sequence.
                #[inline(always)]
                pub fn set_end_of_conversion_interrupt(&mut self, eoc: config::Eoc) {
                    self.config.end_of_conversion_interrupt = eoc;
                    let (en, eocs) = match eoc {
                        config::Eoc::Disabled => (false, false),
                        config::Eoc::Conversion => (true, true),
                        config::Eoc::Sequence => (true, false),
                    };
                    self.adc_reg.ier.modify(|_, w|w
                        .eosie().bit(eocs)
                        .eocie().bit(en)
                    );
                }

                /// Enable/disable overrun interrupt
                ///
                /// This is triggered when the AD finishes a conversion before the last value was read by CPU/DMA
                pub fn set_overrun_interrupt(&mut self, enable: bool) {
                    self.adc_reg.ier.modify(|_, w| w.ovrie().bit(enable));
                }

                /// Sets the default sample time that is used for one-shot conversions.
                /// [configure_channel](#method.configure_channel) and [start_conversion](#method.start_conversion) can be \
                /// used for configurations where different sampling times are required per channel.
                #[inline(always)]
                pub fn set_default_sample_time(&mut self, sample_time: config::SampleTime) {
                    self.config.default_sample_time = sample_time;
                }

                /// Sets the differential selection per channel.
                #[inline(always)]
                pub fn set_channel_input_type(&mut self, df: config::DifferentialSelection) {
                    self.config.difsel = df;

                    self.adc_reg.difsel.modify(|_, w| {w
                        .difsel_0().bit(df.get_channel(0).into() )
                        .difsel_1().bit(df.get_channel(1).into() )
                        .difsel_2().bit(df.get_channel(2).into() )
                        .difsel_3().bit(df.get_channel(3).into() )
                        .difsel_4().bit(df.get_channel(4).into() )
                        .difsel_5().bit(df.get_channel(5).into() )
                        .difsel_6().bit(df.get_channel(6).into() )
                        .difsel_7().bit(df.get_channel(7).into() )
                        .difsel_8().bit(df.get_channel(8).into() )
                        .difsel_9().bit(df.get_channel(9).into() )
                        .difsel_10().bit(df.get_channel(10).into() )
                        .difsel_11().bit(df.get_channel(11).into() )
                        .difsel_12().bit(df.get_channel(12).into() )
                        .difsel_13().bit(df.get_channel(13).into() )
                        .difsel_14().bit(df.get_channel(14).into() )
                        .difsel_15().bit(df.get_channel(15).into() )
                        .difsel_16().bit(df.get_channel(16).into() )
                        .difsel_17().bit(df.get_channel(17).into() )
                        .difsel_18().bit(df.get_channel(18).into() )
                    });
                }

                /// Reset the sequence
                #[inline(always)]
                pub fn reset_sequence(&mut self) {
                    //The reset state is One conversion selected
                    self.adc_reg.sqr1.modify(|_, w| w.l().bits(config::Sequence::One.into()));
                }

                /// Returns the current sequence length. Primarily useful for configuring DMA.
                #[inline(always)]
                pub fn sequence_length(&mut self) -> u8 {
                    self.adc_reg.sqr1.read().l().bits() + 1
                }

                /// Returns the address of the ADC data register. Primarily useful for configuring DMA.
                #[inline(always)]
                pub fn data_register_address(&self) -> u32 {
                    &self.adc_reg.dr as *const _ as u32
                }

                /// Calibrate the adc for <Input Type>
                #[inline(always)]
                pub fn calibrate(&mut self, it: config::InputType) {
                    match it {
                        config::InputType::SingleEnded => {
                            self.adc_reg.cr.modify(|_, w| w.adcaldif().clear_bit() );
                        },
                        config::InputType::Differential => {
                            self.adc_reg.cr.modify(|_, w| w.adcaldif().set_bit() );
                        },
                    }

                    self.adc_reg.cr.modify(|_, w| w.adcal().set_bit() );
                    while self.adc_reg.cr.read().adcal().bit_is_set() {}
                }

                /// Calibrate the Adc for all Input Types
                #[inline(always)]
                pub fn calibrate_all(&mut self) {
                    self.calibrate(config::InputType::Differential);
                    self.calibrate(config::InputType::SingleEnded);
                }

                /// Configure a channel for sampling.
                /// It will make sure the sequence is at least as long as the `sequence` provided.
                /// # Arguments
                /// * `channel` - channel to configure
                /// * `sequence` - where in the sequence to sample the channel. Also called rank in some STM docs/code
                /// * `sample_time` - how long to sample for. See datasheet and ref manual to work out how long you need\
                /// to sample for at a given ADC clock frequency
                pub fn configure_channel<CHANNEL>(&mut self, _channel: &CHANNEL, sequence: config::Sequence, sample_time: config::SampleTime)
                where
                    CHANNEL: Channel<stm32::$adc_type, ID=u8>
                {

                    //Check the sequence is long enough
                    self.adc_reg.sqr1.modify(|r, w| {
                        let prev: config::Sequence = r.l().bits().into();
                        if prev < sequence {
                            w.l().bits(sequence.into())
                        } else {
                            w
                        }
                    });

                    let channel = CHANNEL::channel();

                    //Set the channel in the right sequence field
                    match sequence {
                        config::Sequence::One      => self.adc_reg.sqr1.modify(|_, w| unsafe {w.sq1().bits(channel) }),
                        config::Sequence::Two      => self.adc_reg.sqr1.modify(|_, w| unsafe {w.sq2().bits(channel) }),
                        config::Sequence::Three    => self.adc_reg.sqr1.modify(|_, w| unsafe {w.sq3().bits(channel) }),
                        config::Sequence::Four     => self.adc_reg.sqr1.modify(|_, w| unsafe {w.sq4().bits(channel) }),
                        config::Sequence::Five     => self.adc_reg.sqr2.modify(|_, w| unsafe {w.sq5().bits(channel) }),
                        config::Sequence::Six      => self.adc_reg.sqr2.modify(|_, w| unsafe {w.sq6().bits(channel) }),
                        config::Sequence::Seven    => self.adc_reg.sqr2.modify(|_, w| unsafe {w.sq7().bits(channel) }),
                        config::Sequence::Eight    => self.adc_reg.sqr2.modify(|_, w| unsafe {w.sq8().bits(channel) }),
                        config::Sequence::Nine     => self.adc_reg.sqr2.modify(|_, w| unsafe {w.sq9().bits(channel) }),
                        config::Sequence::Ten      => self.adc_reg.sqr3.modify(|_, w| unsafe {w.sq10().bits(channel) }),
                        config::Sequence::Eleven   => self.adc_reg.sqr3.modify(|_, w| unsafe {w.sq11().bits(channel) }),
                        config::Sequence::Twelve   => self.adc_reg.sqr3.modify(|_, w| unsafe {w.sq12().bits(channel) }),
                        config::Sequence::Thirteen => self.adc_reg.sqr3.modify(|_, w| unsafe {w.sq13().bits(channel) }),
                        config::Sequence::Fourteen => self.adc_reg.sqr3.modify(|_, w| unsafe {w.sq14().bits(channel) }),
                        config::Sequence::Fifteen  => self.adc_reg.sqr4.modify(|_, w| unsafe {w.sq15().bits(channel) }),
                        config::Sequence::Sixteen  => self.adc_reg.sqr4.modify(|_, w| unsafe {w.sq16().bits(channel) }),
                    }

                    //Set the sample time for the channel
                    let st = u8::from(sample_time);
                    match channel {
                        0 => self.adc_reg.smpr1.modify(|_, w| w.smp0().bits(st) ),
                        1 => self.adc_reg.smpr1.modify(|_, w| w.smp1().bits(st) ),
                        2 => self.adc_reg.smpr1.modify(|_, w| w.smp2().bits(st) ),
                        3 => self.adc_reg.smpr1.modify(|_, w| w.smp3().bits(st) ),
                        4 => self.adc_reg.smpr1.modify(|_, w| w.smp4().bits(st) ),
                        5 => self.adc_reg.smpr1.modify(|_, w| w.smp5().bits(st) ),
                        6 => self.adc_reg.smpr1.modify(|_, w| w.smp6().bits(st) ),
                        7 => self.adc_reg.smpr1.modify(|_, w| w.smp7().bits(st) ),
                        8 => self.adc_reg.smpr1.modify(|_, w| w.smp8().bits(st) ),
                        9 => self.adc_reg.smpr1.modify(|_, w| w.smp9().bits(st) ),
                        10 => self.adc_reg.smpr2.modify(|_, w| w.smp10().bits(st) ),
                        11 => self.adc_reg.smpr2.modify(|_, w| w.smp11().bits(st) ),
                        12 => self.adc_reg.smpr2.modify(|_, w| w.smp12().bits(st) ),
                        13 => self.adc_reg.smpr2.modify(|_, w| w.smp13().bits(st) ),
                        14 => self.adc_reg.smpr2.modify(|_, w| w.smp14().bits(st) ),
                        15 => self.adc_reg.smpr2.modify(|_, w| w.smp15().bits(st) ),
                        16 => self.adc_reg.smpr2.modify(|_, w| w.smp16().bits(st) ),
                        17 => self.adc_reg.smpr2.modify(|_, w| w.smp17().bits(st) ),
                        18 => self.adc_reg.smpr2.modify(|_, w| w.smp18().bits(st) ),
                        _ => unimplemented!(),
                    }
                }
                /// Synchronously convert a single sample
                /// Note that it reconfigures the adc sequence and doesn't restore it
                pub fn convert<PIN>(&mut self, pin: &PIN, sample_time: config::SampleTime) -> u16
                where
                    PIN: Channel<stm32::$adc_type, ID=u8>
                {
                    let saved_config = self.config;
                    self.adc_reg.cfgr.modify(|_, w| w
                        .dmaen().clear_bit() //Disable dma
                        .cont().clear_bit() //Disable continuous mode
                        .exten().bits(config::TriggerMode::Disabled.into()) //Disable trigger
                    );
                    self.adc_reg.ier.modify(|_, w| w
                        .eocie().clear_bit() //Disable end of conversion interrupt
                    );

                    self.enable();
                    self.reset_sequence();
                    self.configure_channel(pin, config::Sequence::One, sample_time);
                    self.start_conversion();

                    //Wait for the sequence to complete
                    self.wait_for_conversion_sequence();

                    let result = self.current_sample();

                    self.disable();

                    //Reset the config
                    self.apply_config(saved_config);

                    result
                }

                /// Resets the end-of-conversion flag
                #[inline(always)]
                pub fn clear_end_of_conversion_flag(&mut self) {
                    self.adc_reg.isr.modify(|_, w| w.eoc().set_bit());
                }

                /// Block until the conversion is completed and return to configured
                pub fn wait_for_conversion_sequence(&mut self) {
                    while !self.adc_reg.isr.read().eoc().bit_is_set() {}
                }

                /// get current sample
                #[inline(always)]
                pub fn current_sample(&self) -> u16 {
                    self.adc_reg.dr.read().rdata().bits()
                }

                /// Starts conversion sequence. Waits for the hardware to indicate it's actually started.
                #[inline(always)]
                pub fn start_conversion(&mut self) {
                    //Start conversion
                    self.adc_reg.cr.modify(|_, w| w.adstart().set_bit());
                }

                /// Cancels an ongoing conversion
                #[inline(always)]
                pub fn cancel_conversion(&mut self) {
                    self.adc_reg.cr.modify(|_, w| w.adstp().set_bit());
                    while self.adc_reg.cr.read().adstart().bit_is_set() {}
                }

                /// Returns if the Voltage Regulator is enabled
                #[inline(always)]
                pub fn is_vreg_enabled(&self) -> bool {
                    self.adc_reg.cr.read().advregen().bit_is_set()
                }

                /// Returns if Deep Power Down is enabled
                #[inline(always)]
                pub fn is_deeppwd_enabled(&self) -> bool {
                    self.adc_reg.cr.read().deeppwd().bit_is_set()
                }

                /// Returns if a conversion is active
                #[inline(always)]
                pub fn is_conversion_active(&self) -> bool {
                    self.adc_reg.cr.read().adstart().bit_is_set()
                }

                /// Enables the vbat internal channel
                #[inline(always)]
                pub fn enable_vbat(&self, common: &stm32::$common_type) {
                    common.ccr.modify(|_, w| w.vbatsel().set_bit());
                }

                /// Enables the vbat internal channel
                #[inline(always)]
                pub fn disable_vbat(&self, common: &stm32::$common_type) {
                    common.ccr.modify(|_, w| w.vbatsel().clear_bit());
                }

                /// Returns if the vbat internal channel is enabled
                #[inline(always)]
                pub fn is_vbat_enabled(&mut self, common: &stm32::$common_type) -> bool {
                    common.ccr.read().vbatsel().bit_is_set()
                }

                /// Enables the temp internal channel.
                #[inline(always)]
                pub fn enable_temperature(&mut self, common: &stm32::$common_type) {
                    common.ccr.modify(|_, w| w.vsensesel().set_bit());
                }

                /// Disables the temp internal channel
                #[inline(always)]
                pub fn disable_temperature(&mut self, common: &stm32::$common_type) {
                    common.ccr.modify(|_, w| w.vsensesel().clear_bit());
                }

                /// Returns if the temp internal channel is enabled
                #[inline(always)]
                pub fn is_temperature_enabled(&mut self, common: &stm32::$common_type) -> bool {
                    common.ccr.read().vsensesel().bit_is_set()
                }

                /// Enables the vref internal channel.
                #[inline(always)]
                pub fn enable_vref(&mut self, common: &stm32::$common_type) {
                    common.ccr.modify(|_, w| w.vrefen().set_bit());
                }

                /// Disables the vref internal channel
                #[inline(always)]
                pub fn disable_vref(&mut self, common: &stm32::$common_type) {
                    common.ccr.modify(|_, w| w.vrefen().clear_bit());
                }

                /// Returns if the vref internal channel is enabled
                #[inline(always)]
                pub fn is_vref_enabled(&mut self, common: &stm32::$common_type) -> bool {
                    common.ccr.read().vrefen().bit_is_set()
                }

                /// Read overrun flag
                #[inline(always)]
                pub fn get_overrun_flag(&self) -> bool {
                    self.adc_reg.isr.read().ovr().bit()
                }

                /// Resets the overrun flag
                #[inline(always)]
                pub fn clear_overrun_flag(&mut self) {
                    self.adc_reg.isr.modify(|_, w| w.ovr().set_bit());
                }
            }

            //TODO: claim now configures the clock for all ADCs in the group (12 and 345).
            //Ideally we should make this a function of the common group and claim all adc in that group at once.
            //The situation now is that the clock source setting can change between claims, changing the existing
            //setting of the allready claimed ADC.
            impl AdcClaim<stm32::$adc_type> for stm32::$adc_type {
                /// Enables the ADC clock, resets the peripheral (optionally), runs calibration and applies the supplied config
                /// # Arguments
                /// * `reset` - should a reset be performed. This is provided because on some devices multiple ADCs share the same common reset
                /// TODO: fix needing SYST
                #[inline(always)]
                fn claim(self, cs: ClockSource, rcc: &Rcc, delay: &mut impl DelayUs<u8>, reset: bool) -> Adc<stm32::$adc_type, Disabled> {
                    unsafe {
                        let rcc_ptr = &(*stm32::RCC::ptr());
                        stm32::$adc_type::enable(rcc_ptr);
                        if reset {stm32::$adc_type::reset(rcc_ptr);}
                    }
                    Self::configure_clock_source(cs, rcc);

                    let dynadc = DynamicAdc {
                        config: config::AdcConfig::default(),
                        adc_reg: self,
                        calibrated_vdda: VDDA_CALIB,
                    };

                    let adc: Adc::<stm32::$adc_type, PoweredDown> = Adc {
                        adc: dynadc,
                        _status: PhantomData,
                    };

                    adc.power_up(delay)
                }

                /// claims and configures the Adc
                #[inline(always)]
                fn claim_and_configure(self, cs: ClockSource, rcc: &Rcc, config: config::AdcConfig<$trigger_type>, delay: &mut impl DelayUs<u8>, reset :bool) -> Adc<stm32::$adc_type, Configured> {
                    let mut adc = self.claim(cs, rcc, delay, reset);
                    adc.adc.config = config;

                    // If the user specified a VDDA, use that over the internally determined value.
                    if let Some(vdda) = config.vdda {
                        adc.adc.calibrated_vdda = vdda;
                    }

                    adc.enable()
                }
            }

            impl<STATUS> Adc<stm32::$adc_type, STATUS> {
                /// Converts a sample value to millivolts using calibrated VDDA and configured resolution
                #[inline(always)]
                pub fn sample_to_millivolts(&self, sample: u16) -> u16 {
                    self.adc.sample_to_millivolts(sample)
                }
            }

            impl Adc<stm32::$adc_type, PoweredDown> {
                /// Powers-up an powered-down Adc
                #[inline(always)]
                pub fn power_up(mut self, delay: &mut impl DelayUs<u8>) -> Adc<stm32::$adc_type, Disabled> {
                    self.adc.power_up(delay);

                    Adc {
                        adc: self.adc,
                        _status: PhantomData,
                    }
                }

                /// Puts a Disabled Adc into Powered Mode
                #[inline(always)]
                pub fn power_down(mut adc: Adc<stm32::$adc_type, Disabled>) -> Self {
                    adc.adc.power_down();

                    Adc {
                        adc: adc.adc,
                        _status: PhantomData,
                    }
                }

                /// Disables the Voltage Regulator and release the ADC
                #[inline(always)]
                pub fn release(self) -> stm32::$adc_type {
                    self.adc.release()
                }

                /// Releases the Adc as a DynamicAdc.
                /// While this is not unsafe; using methods while the Adc is in the wrong state will mess it up.
                #[inline(always)]
                pub fn into_dynamic_adc(self) -> DynamicAdc<stm32::$adc_type> {
                    self.adc
                }

                /// Retrieves the DynamicAdc.
                /// This will put the adc in power down state.
                #[inline(always)]
                pub fn from_dynamic_adc(mut dynadc: DynamicAdc<stm32::$adc_type>) -> Self {
                    if dynadc.is_conversion_active() {
                        dynadc.cancel_conversion();
                    }
                    if dynadc.is_enabled() {
                        dynadc.disable();
                    }
                    if dynadc.is_deeppwd_enabled() {
                        dynadc.disable_deeppwd_down();
                    }

                    dynadc.power_down();

                    Adc {
                        adc: dynadc,
                        _status: PhantomData,
                    }
                }

                /// Enables the Deep Power Down Modus
                #[inline(always)]
                pub fn enable_deeppwd_down(&mut self) {
                    self.adc.enable_deeppwd_down()
                }

                /// Disables the Deep Power Down Modus
                #[inline(always)]
                pub fn disable_deeppwd_down(&mut self) {
                    self.adc.disable_deeppwd_down()
                }
            }

            impl Adc<stm32::$adc_type, Disabled> {
                adc!(additionals: $adc_type => ($common_type));
                adc!(additionals_checks: $adc_type => ($common_type));

                /// Enables the adc
                #[inline(always)]
                pub fn enable(mut self) -> Adc<stm32::$adc_type, Configured> {
                    self.adc.enable();

                    Adc {
                        adc: self.adc,
                        _status: PhantomData,
                    }
                }

                /// Enables the adc
                #[inline(always)]
                pub fn configure_and_enable(mut self, config: config::AdcConfig<$trigger_type>) -> Adc<stm32::$adc_type, Configured> {
                    self.adc.apply_config(config);
                    self.enable()
                }

                /// enable the adc and configure for DMA.
                /// panics if set to Dma::Disabled
                #[inline(always)]
                pub fn enable_dma(mut self, dma: config::Dma) -> Adc<stm32::$adc_type, DMA> {
                    if let config::Dma::Disabled = dma {
                        panic!("Requesting Enabling DMA with DisableDma parameter");
                    }

                    self.adc.enable_dma(dma);

                    Adc {
                        adc: self.adc,
                        _status: PhantomData,
                    }
                }

                /// Puts a disabled Adc into PoweredDown Mode
                #[inline(always)]
                pub fn power_down(mut self) -> Adc<stm32::$adc_type, PoweredDown> {
                    self.adc.power_down();

                    Adc {
                        adc: self.adc,
                        _status: PhantomData,
                    }
                }

                /// Sets the clock_mode for the adc
                #[inline(always)]
                pub fn set_clock_mode(&mut self, clock_mode: config::ClockMode) {
                    self.adc.set_clock_mode(clock_mode)
                }

                /// Sets the clock for the adc
                #[inline(always)]
                pub fn set_clock(&mut self, clock: config::Clock) {
                    self.adc.set_clock(clock)
                }

                /// Sets the oversampling
                #[inline(always)]
                pub fn set_oversampling(&mut self, oversampling: config::OverSampling, shift: config::OverSamplingShift) {
                    self.adc.set_oversampling(oversampling, shift)
                }

                /// Sets the sampling resolution
                #[inline(always)]
                pub fn set_resolution(&mut self, resolution: config::Resolution) {
                    self.adc.set_resolution(resolution)
                }

                /// Sets the DR register alignment to left or right
                #[inline(always)]
                pub fn set_align(&mut self, align: config::Align) {
                    self.adc.set_align(align)
                }

                /// Sets which external trigger to use and if it is disabled, rising, falling or both
                #[inline(always)]
                pub fn set_external_trigger(&mut self, (edge, extsel): (config::TriggerMode, $trigger_type)) {
                    self.adc.set_external_trigger( (edge, extsel) )
                }

                /// Sets auto delay to true or false
                #[inline(always)]
                pub fn set_auto_delay(&mut self, delay: bool) {
                    self.adc.set_auto_delay(delay)
                }

                /// Enables and disables continuous mode
                #[inline(always)]
                pub fn set_continuous(&mut self, continuous: config::Continuous) {
                    self.adc.set_continuous(continuous)
                }

                /// Set subgroup length, number of AD readings per trigger event (only relevant in Discontinuous mode)
                pub fn set_subgroup_len(&mut self, subgroup_len: config::SubGroupLength) {
                    self.adc.set_subgroup_len(subgroup_len);
                }

                /// Sets DMA to disabled, single or continuous
                #[inline(always)]
                pub fn set_dma(&mut self, dma: config::Dma) {
                    self.adc.set_dma(dma)
                }

                /// Sets if the end-of-conversion behaviour.
                /// The end-of-conversion interrupt occur either per conversion or for the whole sequence.
                #[inline(always)]
                pub fn set_end_of_conversion_interrupt(&mut self, eoc: config::Eoc) {
                    self.adc.set_end_of_conversion_interrupt(eoc)
                }

                /// Enable/disable overrun interrupt
                ///
                /// This is triggered when the AD finishes a conversion before the last value was read by CPU/DMA
                #[inline(always)]
                pub fn set_overrun_interrupt(&mut self, enable: bool) {
                    self.adc.set_overrun_interrupt(enable)
                }

                /// Sets the default sample time that is used for one-shot conversions.
                /// [configure_channel](#method.configure_channel) and [start_conversion](#method.start_conversion) can be \
                /// used for configurations where different sampling times are required per channel.
                #[inline(always)]
                pub fn set_default_sample_time(&mut self, sample_time: config::SampleTime) {
                    self.adc.set_default_sample_time(sample_time)
                }

                /// Sets the differential selection per channel.
                #[inline(always)]
                pub fn set_channel_input_type(&mut self, df: config::DifferentialSelection) {
                    self.adc.set_channel_input_type(df)
                }

                /// Reset the sequence
                #[inline(always)]
                pub fn reset_sequence(&mut self) {
                    self.adc.reset_sequence()
                }

                /// Returns the current sequence length. Primarily useful for configuring DMA.
                #[inline(always)]
                pub fn sequence_length(&mut self) -> u8 {
                    self.adc.sequence_length()
                }

                /// Calibrate the adc for <Input Type>
                #[inline(always)]
                pub fn calibrate(&mut self, it: config::InputType) {
                    self.adc.calibrate(it)
                }

                /// Calibrate the Adc for all Input Types
                #[inline(always)]
                pub fn calibrate_all(&mut self) {
                    self.adc.calibrate_all();
                }

                /// Configure a channel for sampling.
                /// It will make sure the sequence is at least as long as the `sequence` provided.
                /// # Arguments
                /// * `channel` - channel to configure
                /// * `sequence` - where in the sequence to sample the channel. Also called rank in some STM docs/code
                /// * `sample_time` - how long to sample for. See datasheet and ref manual to work out how long you need\
                /// to sample for at a given ADC clock frequency
                #[inline(always)]
                pub fn configure_channel<CHANNEL>(&mut self, channel: &CHANNEL, sequence: config::Sequence, sample_time: config::SampleTime)
                where
                    CHANNEL: Channel<stm32::$adc_type, ID=u8>
                {
                    self.adc.configure_channel(channel, sequence, sample_time)
                }

                /// Synchronously convert a single sample
                /// Note that it reconfigures the adc sequence and doesn't restore it
                #[inline(always)]
                pub fn convert<PIN>(&mut self, pin: &PIN, sample_time: config::SampleTime) -> u16
                where
                    PIN: Channel<stm32::$adc_type, ID=u8>
                {
                    self.adc.convert(pin, sample_time)
                }
            }

            impl Adc<stm32::$adc_type, Configured> {
                adc!(additionals_checks: $adc_type => ($common_type));

                /// Disables the adc
                #[inline(always)]
                pub fn disable(mut self) -> Adc<stm32::$adc_type, Disabled> {
                    self.adc.disable();

                    Adc {
                        adc: self.adc,
                        _status: PhantomData,
                    }
                }

                /// Starts conversion sequence. Waits for the hardware to indicate it's actually started.
                #[inline(always)]
                pub fn start_conversion(mut self) -> Adc<stm32::$adc_type, Active> {
                    self.adc.clear_end_of_conversion_flag();
                    self.adc.start_conversion();

                    Adc {
                        adc: self.adc,
                        _status: PhantomData,
                    }
                }

                /// Returns the current sample stored in the ADC data register
                #[inline(always)]
                pub fn current_sample(&self) -> u16 {
                    self.adc.current_sample()
                }

                /// Synchronously convert a single sample
                /// Note that it reconfigures the adc sequence and doesn't restore it
                #[inline(always)]
                pub fn convert<PIN>(&mut self, pin: &PIN, sample_time: config::SampleTime) -> u16
                where
                    PIN: Channel<stm32::$adc_type, ID=u8>
                {
                    self.adc.reset_sequence();
                    self.adc.configure_channel(pin, config::Sequence::One, sample_time);
                    self.adc.start_conversion();

                    //Wait for the sequence to complete
                    self.adc.wait_for_conversion_sequence();

                    self.adc.current_sample()
                }
            }

            impl Conversion<stm32::$adc_type> {
                /// Wait in a potential infite loop untill the ADC has stopped the conversion.
                /// Everytime an sample is retrieved 'func' is called.
                /// Note: when the ADC has stopped the conversion, for the last sample, func is NOT run.
                pub fn wait_untill_stopped<F>(mut self, mut func: F) -> Adc<stm32::$adc_type, Configured> where F: FnMut(u16, &Adc<stm32::$adc_type, Active>) {
                    let adc = loop {
                        match self {
                            Conversion::Stopped(adc) => break adc,
                            Conversion::Active(adc) => {
                                self = adc.wait_for_conversion_sequence();
                                if let Conversion::Active(adc) = self {
                                    let sample = adc.current_sample();
                                    func(sample, &adc);

                                    self = Conversion::Active(adc);
                                }
                            }
                        }
                    };
                    adc
                }
            }

            impl Adc<stm32::$adc_type, Active> {
                /// Block until the conversion is completed and return to configured
                pub fn wait_for_conversion_sequence(mut self) -> Conversion<stm32::$adc_type> {
                    self.adc.wait_for_conversion_sequence();

                    if !self.adc.is_conversion_active() {
                        let inactive: Adc<_, Configured> = Adc {
                            adc: self.adc,
                            _status: PhantomData,
                        };

                        Conversion::Stopped(inactive)
                    }
                    else {
                        Conversion::Active(self)
                    }
                }


                /// Returns if a conversion has been completed
                /// Calling this before `wait_for_conversion_sequence`
                /// should make that function return immediatly
                pub fn is_conversion_done(&self) -> bool {
                    !self.adc.is_conversion_active()
                }

                /// Cancels an ongoing conversion
                #[inline(always)]
                pub fn cancel_conversion(mut self) -> Adc<stm32::$adc_type, Configured> {
                    self.adc.cancel_conversion();

                    Adc {
                        adc: self.adc,
                        _status: PhantomData,
                    }
                }

                /// get current sample
                #[inline(always)]
                pub fn current_sample(&self) -> u16 {
                    self.adc.current_sample()
                }

                /// clear end conversion flag
                #[inline(always)]
                pub fn clear_end_conversion_flag(&mut self) {
                    self.adc.clear_end_of_conversion_flag();
                }
            }

            impl Adc<stm32::$adc_type, DMA> {
                /// Starts conversion sequence. Waits for the hardware to indicate it's actually started.
                #[inline(always)]
                pub fn start_conversion(&mut self) {
                    self.adc.start_conversion()
                }

                /// Cancels an ongoing conversion
                #[inline(always)]
                pub fn cancel_conversion(&mut self) {
                    self.adc.cancel_conversion()
                }

                /// Stop the Adc
                #[inline(always)]
                pub fn stop(&mut self) {
                    self.adc.disable()
                }

                /// Disable the Adc
                #[inline(always)]
                pub fn disable(mut self) -> Adc<stm32::$adc_type, Disabled> {
                    self.adc.set_dma(config::Dma::Disabled);
                    self.adc.disable();

                    Adc {
                        adc: self.adc,
                        _status: PhantomData,
                    }
                }

                /// Read overrun flag
                #[inline(always)]
                pub fn get_overrun_flag(&self) -> bool {
                    self.adc.get_overrun_flag()
                }

                /// Resets the overrun flag
                #[inline(always)]
                pub fn clear_overrun_flag(&mut self) {
                    self.adc.clear_overrun_flag();
                }
            }

            unsafe impl TargetAddress<PeripheralToMemory> for Adc<stm32::$adc_type, DMA> {
                #[inline(always)]
                fn address(&self) -> u32 {
                    self.adc.data_register_address()
                }

                type MemSize = u16;

                const REQUEST_LINE: Option<u8> = Some($mux as u8);
            }

            impl<PIN> OneShot<stm32::$adc_type, u16, PIN> for Adc<stm32::$adc_type, Disabled>
            where
                PIN: Channel<stm32::$adc_type, ID=u8>,
            {
                type Error = ();

                fn read(&mut self, pin: &mut PIN) -> nb::Result<u16, Self::Error> {
                    Ok(self.convert(pin, self.adc.config.default_sample_time) )
                }
            }
        )+
    };
}

#[cfg(any(
    feature = "stm32g431",
    feature = "stm32g441",
    feature = "stm32g471",
    feature = "stm32g473",
    feature = "stm32g474",
    feature = "stm32g483",
    feature = "stm32g484",
    feature = "stm32g491",
    feature = "stm32g4a1",
))]
adc!(ADC1 => (ExternalTrigger12, configure_clock_source12, DmaMuxResources::ADC1, (ADC12_COMMON) ));

#[cfg(any(
    feature = "stm32g431",
    feature = "stm32g441",
    feature = "stm32g471",
    feature = "stm32g473",
    feature = "stm32g474",
    feature = "stm32g483",
    feature = "stm32g484",
    feature = "stm32g491",
    feature = "stm32g4a1",
))]
adc!(ADC2 => (ExternalTrigger12, configure_clock_source12, DmaMuxResources::ADC2, (ADC12_COMMON) ));

#[cfg(any(
    feature = "stm32g471",
    feature = "stm32g473",
    feature = "stm32g474",
    feature = "stm32g483",
    feature = "stm32g484",
    feature = "stm32g491",
    feature = "stm32g4a1",
))]
adc!(ADC3 => (ExternalTrigger345, configure_clock_source345, DmaMuxResources::ADC3, (ADC345_COMMON) ));

#[cfg(any(
    feature = "stm32g473",
    feature = "stm32g474",
    feature = "stm32g483",
    feature = "stm32g484",
))]
adc!(ADC4 => (ExternalTrigger345, configure_clock_source345, DmaMuxResources::ADC4, (ADC345_COMMON) ));

#[cfg(any(
    feature = "stm32g473",
    feature = "stm32g474",
    feature = "stm32g483",
    feature = "stm32g484",
))]
adc!(ADC5 => (ExternalTrigger345, configure_clock_source345, DmaMuxResources::ADC5, (ADC345_COMMON) ));

#[cfg(any(feature = "stm32g431", feature = "stm32g441", feature = "stm32g471",))]
adc_pins!(
    gpioa::PA0<Analog> => (ADC1, 1),
    gpioa::PA0<Analog> => (ADC2, 1),
    gpioa::PA1<Analog> => (ADC1, 2),
    gpioa::PA1<Analog> => (ADC2, 2),
    gpioa::PA2<Analog> => (ADC1, 3),
    gpioa::PA3<Analog> => (ADC1, 4),
    gpioa::PA4<Analog> => (ADC2, 17),
    gpioa::PA5<Analog> => (ADC2, 13),
    gpioa::PA6<Analog> => (ADC2, 3),
    gpioa::PA7<Analog> => (ADC2, 4),

    gpiob::PB0<Analog> => (ADC1, 15),
    gpiob::PB1<Analog> => (ADC1, 12),
    gpiob::PB2<Analog> => (ADC2, 12),
    gpiob::PB11<Analog> => (ADC1, 14),
    gpiob::PB11<Analog> => (ADC2, 14),
    gpiob::PB12<Analog> => (ADC1, 11),
    gpiob::PB15<Analog> => (ADC2, 15),

    gpioc::PC0<Analog> => (ADC1, 6),
    gpioc::PC0<Analog> => (ADC2, 6),
    gpioc::PC1<Analog> => (ADC1, 7),
    gpioc::PC1<Analog> => (ADC2, 7),
    gpioc::PC2<Analog> => (ADC1, 8),
    gpioc::PC2<Analog> => (ADC2, 8),
    gpioc::PC3<Analog> => (ADC1, 9),
    gpioc::PC3<Analog> => (ADC2, 9),
    gpioc::PC4<Analog> => (ADC2, 5),
    gpioc::PC5<Analog> => (ADC2, 11),

    gpiof::PF0<Analog> => (ADC1, 10),
    gpiof::PF1<Analog> => (ADC2, 10),

    Temperature => (ADC1, 16),
    Vbat => (ADC1, 17),
    Vref => (ADC1, 18),
);

#[cfg(any(
    feature = "stm32g473",
    feature = "stm32g474",
    feature = "stm32g483",
    feature = "stm32g484",
))]
adc_pins!(
    gpioa::PA0<Analog> => (ADC1, 1),
    gpioa::PA0<Analog> => (ADC2, 1),
    gpioa::PA1<Analog> => (ADC1, 2),
    gpioa::PA1<Analog> => (ADC2, 2),
    gpioa::PA2<Analog> => (ADC1, 3),
    gpioa::PA3<Analog> => (ADC1, 4),
    gpioa::PA4<Analog> => (ADC2, 17),
    gpioa::PA5<Analog> => (ADC2, 13),
    gpioa::PA6<Analog> => (ADC2, 3),
    gpioa::PA7<Analog> => (ADC2, 4),
    gpioa::PA8<Analog> => (ADC5, 1),
    gpioa::PA9<Analog> => (ADC5, 2),

    gpiob::PB0<Analog> => (ADC3, 12),
    gpiob::PB0<Analog> => (ADC1, 15),
    gpiob::PB1<Analog> => (ADC3, 1),
    gpiob::PB1<Analog> => (ADC1, 12),
    gpiob::PB2<Analog> => (ADC2, 12),
    gpiob::PB11<Analog> => (ADC1, 14),
    gpiob::PB11<Analog> => (ADC2, 14),
    gpiob::PB12<Analog> => (ADC4, 3),
    gpiob::PB12<Analog> => (ADC1, 11),
    gpiob::PB13<Analog> => (ADC3, 5),
    gpiob::PB14<Analog> => (ADC4, 4),
    gpiob::PB14<Analog> => (ADC1, 5),
    gpiob::PB15<Analog> => (ADC4, 5),
    gpiob::PB15<Analog> => (ADC2, 15),

    gpioc::PC0<Analog> => (ADC1, 6),
    gpioc::PC0<Analog> => (ADC2, 6),
    gpioc::PC1<Analog> => (ADC1, 7),
    gpioc::PC1<Analog> => (ADC2, 7),
    gpioc::PC2<Analog> => (ADC1, 8),
    gpioc::PC2<Analog> => (ADC2, 8),
    gpioc::PC3<Analog> => (ADC1, 9),
    gpioc::PC3<Analog> => (ADC2, 9),
    gpioc::PC4<Analog> => (ADC2, 5),
    gpioc::PC5<Analog> => (ADC2, 11),

    gpioe::PE7<Analog> => (ADC3, 4),
    gpioe::PE8<Analog> => (ADC3, 6),
    gpioe::PE8<Analog> => (ADC4, 6),
    gpioe::PE8<Analog> => (ADC5, 6),
    gpioe::PE9<Analog> => (ADC3, 2),
    gpioe::PE10<Analog> => (ADC3, 14),
    gpioe::PE10<Analog> => (ADC4, 14),
    gpioe::PE10<Analog> => (ADC5, 14),
    gpioe::PE11<Analog> => (ADC3, 15),
    gpioe::PE11<Analog> => (ADC4, 15),
    gpioe::PE11<Analog> => (ADC5, 15),
    gpioe::PE12<Analog> => (ADC3, 16),
    gpioe::PE12<Analog> => (ADC4, 16),
    gpioe::PE12<Analog> => (ADC5, 16),
    gpioe::PE13<Analog> => (ADC3, 3),
    gpioe::PE14<Analog> => (ADC4, 1),
    gpioe::PE15<Analog> => (ADC4, 2),

    gpiod::PD8<Analog> => (ADC4, 12),
    gpiod::PD8<Analog> => (ADC5, 12),
    gpiod::PD9<Analog> => (ADC4, 13),
    gpiod::PD9<Analog> => (ADC5, 13),
    gpiod::PD10<Analog> => (ADC3, 7),
    gpiod::PD10<Analog> => (ADC4, 7),
    gpiod::PD10<Analog> => (ADC5, 7),
    gpiod::PD11<Analog> => (ADC3, 8),
    gpiod::PD11<Analog> => (ADC4, 8),
    gpiod::PD11<Analog> => (ADC5, 8),
    gpiod::PD12<Analog> => (ADC3, 9),
    gpiod::PD12<Analog> => (ADC4, 9),
    gpiod::PD12<Analog> => (ADC5, 9),
    gpiod::PD13<Analog> => (ADC3, 10),
    gpiod::PD13<Analog> => (ADC4, 10),
    gpiod::PD13<Analog> => (ADC5, 10),
    gpiod::PD14<Analog> => (ADC3, 11),
    gpiod::PD14<Analog> => (ADC4, 11),
    gpiod::PD14<Analog> => (ADC5, 11),

    gpiof::PF0<Analog> => (ADC1, 10),
    gpiof::PF1<Analog> => (ADC2, 10),

    Temperature => (ADC1, 16),
    Vbat => (ADC1, 17),
    Vbat => (ADC3, 17),
    Vbat => (ADC5, 17),
    Vref => (ADC1, 18),
    Vref => (ADC3, 18),
    Vref => (ADC4, 18),
    Vref => (ADC5, 18),
);

// See https://www.st.com/resource/en/reference_manual/rm0440-stm32g4-series-advanced-armbased-32bit-mcus-stmicroelectronics.pdf#page=782
adc_op_pga!(
    // TODO: Add all opamp types: OpenLoop, Follower(for all opamps)
    // TODO: Should we restrict type parameters A and B?
    // TODO: Also allow AD-channels shared by pins
    opamp::opamp1::Pga<A, B> => (ADC1, 13),
    opamp::opamp2::Pga<A, B> => (ADC2, 16),

    opamp::opamp3::Pga<A, B> => (ADC2, 18),
);

adc_op_follower!(
    opamp::opamp1::Follower<A> => (ADC1, 13),
    opamp::opamp2::Follower<A> => (ADC2, 16),

    opamp::opamp3::Follower<A> => (ADC2, 18),
);

#[cfg(any(
    feature = "stm32g473",
    feature = "stm32g474",
    feature = "stm32g483",
    feature = "stm32g484",
    feature = "stm32g491",
    feature = "stm32g4a1",
))]
adc_op_pga!(
    opamp::opamp3::Pga<A, B> => (ADC3, 13),
    opamp::opamp4::Pga<A, B> => (ADC5, 5),
    opamp::opamp5::Pga<A, B> => (ADC5, 3),
    opamp::opamp6::Pga<A, B> => (ADC4, 17),
);

#[cfg(any(feature = "stm32g491", feature = "stm32g4a1",))]
adc_op_pga!(
    opamp::opamp6::Pga<A, B> => (ADC3, 17),
);

#[cfg(any(feature = "stm32g491", feature = "stm32g4a1",))]
adc_pins!(
    gpioa::PA0<Analog> => (ADC1, 1),
    gpioa::PA0<Analog> => (ADC2, 1),
    gpioa::PA1<Analog> => (ADC1, 2),
    gpioa::PA1<Analog> => (ADC2, 2),
    gpioa::PA2<Analog> => (ADC1, 3),
    gpioa::PA3<Analog> => (ADC1, 4),
    gpioa::PA4<Analog> => (ADC2, 17),
    gpioa::PA5<Analog> => (ADC2, 13),
    gpioa::PA6<Analog> => (ADC2, 3),
    gpioa::PA7<Analog> => (ADC2, 4),

    gpiob::PB0<Analog> => (ADC3, 12),
    gpiob::PB0<Analog> => (ADC1, 15),
    gpiob::PB1<Analog> => (ADC3, 1),
    gpiob::PB1<Analog> => (ADC1, 12),
    gpiob::PB2<Analog> => (ADC2, 12),
    gpiob::PB11<Analog> => (ADC1, 14),
    gpiob::PB11<Analog> => (ADC2, 14),
    gpiob::PB12<Analog> => (ADC1, 11),
    gpiob::PB13<Analog> => (ADC3, 5),
    gpiob::PB14<Analog> => (ADC1, 5),
    gpiob::PB15<Analog> => (ADC2, 15),

    gpioc::PC0<Analog> => (ADC1, 6),
    gpioc::PC0<Analog> => (ADC2, 6),
    gpioc::PC1<Analog> => (ADC1, 7),
    gpioc::PC1<Analog> => (ADC2, 7),
    gpioc::PC2<Analog> => (ADC1, 8),
    gpioc::PC2<Analog> => (ADC2, 8),
    gpioc::PC3<Analog> => (ADC1, 9),
    gpioc::PC3<Analog> => (ADC2, 9),
    gpioc::PC4<Analog> => (ADC2, 5),
    gpioc::PC5<Analog> => (ADC2, 11),

    gpioe::PE7<Analog> => (ADC3, 4),
    gpioe::PE8<Analog> => (ADC3, 6),
    gpioe::PE9<Analog> => (ADC3, 2),
    gpioe::PE10<Analog> => (ADC3, 14),
    gpioe::PE11<Analog> => (ADC3, 15),
    gpioe::PE12<Analog> => (ADC3, 16),
    gpioe::PE13<Analog> => (ADC3, 3),

    gpiod::PD10<Analog> => (ADC3, 7),
    gpiod::PD11<Analog> => (ADC3, 8),
    gpiod::PD12<Analog> => (ADC3, 9),
    gpiod::PD13<Analog> => (ADC3, 10),
    gpiod::PD14<Analog> => (ADC3, 11),

    gpiof::PF0<Analog> => (ADC1, 10),
    gpiof::PF1<Analog> => (ADC2, 10),

    Temperature => (ADC1, 16),
    Vbat => (ADC1, 17),
    Vbat => (ADC3, 17),
    Vref => (ADC1, 18),
    Vref => (ADC3, 18),
);
