use crate::dac::Instance;
use crate::stm32::{DAC1, DAC2, DAC3, DAC4};

pub trait DacTriggerSource<DAC: Instance> {
    const SIGNAL: u8;

    #[inline(always)]
    fn signal(&self) -> u8 {
        Self::SIGNAL
    }
}

pub trait DacIncrementSource<DAC: Instance> {
    const SIGNAL: u8;

    #[inline(always)]
    fn signal(&self) -> u8 {
        Self::SIGNAL
    }
}

/// Implements the struct representing a DAC trigger source.
/// Sources which are valid for a DAC instance will have a [`DacTriggerSource`]
/// trait implemented on them with the const SIGNAL set to the signal
/// interconnection from RM0440 Table 187
macro_rules! impl_dac_trigger {
    ($($source:ident)+) => {$(
        pub struct $source;
    )+};
}

impl_dac_trigger!(Software);
impl_dac_trigger!(Timer1);
impl_dac_trigger!(Timer2);
impl_dac_trigger!(Timer3);
impl_dac_trigger!(Timer4);
impl_dac_trigger!(Timer5);
impl_dac_trigger!(Timer6);
impl_dac_trigger!(Timer7);
impl_dac_trigger!(Timer8);
impl_dac_trigger!(Timer15);

impl_dac_trigger!(HrtimDacReset1);
impl_dac_trigger!(HrtimDacReset2);
impl_dac_trigger!(HrtimDacReset3);
impl_dac_trigger!(HrtimDacReset4);
impl_dac_trigger!(HrtimDacReset5);
impl_dac_trigger!(HrtimDacReset6);
impl_dac_trigger!(HrtimDacTrigger1);
impl_dac_trigger!(HrtimDacTrigger2);
impl_dac_trigger!(HrtimDacTrigger3);
impl_dac_trigger!(HrtimDacStep1);
impl_dac_trigger!(HrtimDacStep2);
impl_dac_trigger!(HrtimDacStep3);
impl_dac_trigger!(HrtimDacStep4);
impl_dac_trigger!(HrtimDacStep5);
impl_dac_trigger!(HrtimDacStep6);

impl_dac_trigger!(Exti9);
impl_dac_trigger!(Exti10);

/// Macro to implement the DacTriggerSource trait for each supported trigger source
/// specific to each DAC peripheral instance.
macro_rules! impl_dac_channel_trigger {
    ($($DAC:ty, $source:ident, $signal:expr)+) => {$(
        impl DacTriggerSource<$DAC> for $source {
            const SIGNAL: u8 = $signal;
        }
    )+};
}

/// Macro to implement the DacIncrementSource trait for each supported
/// increment trigger source specific to each DAC peripheral instance.
macro_rules! impl_dac_increment_trigger {
    ($($DAC:ty, $source:ident, $signal:expr)+) => {$(
        impl DacIncrementSource<$DAC> for $source {
            const SIGNAL: u8 = $signal;
        }
    )+};
}

impl_dac_channel_trigger!(DAC1, Software, 0);
impl_dac_channel_trigger!(DAC1, Timer8, 1);
impl_dac_channel_trigger!(DAC1, Timer7, 2);
impl_dac_channel_trigger!(DAC1, Timer15, 3);
impl_dac_channel_trigger!(DAC1, Timer2, 4);
impl_dac_channel_trigger!(DAC1, Timer4, 5);
impl_dac_channel_trigger!(DAC1, Exti9, 6);
impl_dac_channel_trigger!(DAC1, Timer6, 7);
impl_dac_channel_trigger!(DAC1, Timer3, 8);
impl_dac_channel_trigger!(DAC1, HrtimDacReset1, 9);
impl_dac_channel_trigger!(DAC1, HrtimDacReset2, 10);
impl_dac_channel_trigger!(DAC1, HrtimDacReset3, 11);
impl_dac_channel_trigger!(DAC1, HrtimDacReset4, 12);
impl_dac_channel_trigger!(DAC1, HrtimDacReset5, 13);
impl_dac_channel_trigger!(DAC1, HrtimDacReset6, 14);
impl_dac_channel_trigger!(DAC1, HrtimDacTrigger1, 15);

impl_dac_increment_trigger!(DAC1, Software, 0);
impl_dac_increment_trigger!(DAC1, Timer8, 1);
impl_dac_increment_trigger!(DAC1, Timer7, 2);
impl_dac_increment_trigger!(DAC1, Timer15, 3);
impl_dac_increment_trigger!(DAC1, Timer2, 4);
impl_dac_increment_trigger!(DAC1, Timer4, 5);
impl_dac_increment_trigger!(DAC1, Exti10, 6);
impl_dac_increment_trigger!(DAC1, Timer6, 7);
impl_dac_increment_trigger!(DAC1, Timer3, 8);
impl_dac_increment_trigger!(DAC1, HrtimDacStep1, 9);
impl_dac_increment_trigger!(DAC1, HrtimDacStep2, 10);
impl_dac_increment_trigger!(DAC1, HrtimDacStep3, 11);
impl_dac_increment_trigger!(DAC1, HrtimDacStep4, 12);
impl_dac_increment_trigger!(DAC1, HrtimDacStep5, 13);
impl_dac_increment_trigger!(DAC1, HrtimDacStep6, 14);

impl_dac_channel_trigger!(DAC2, Software, 0);
impl_dac_channel_trigger!(DAC2, Timer8, 1);
impl_dac_channel_trigger!(DAC2, Timer7, 2);
impl_dac_channel_trigger!(DAC2, Timer15, 3);
impl_dac_channel_trigger!(DAC2, Timer2, 4);
impl_dac_channel_trigger!(DAC2, Timer4, 5);
impl_dac_channel_trigger!(DAC2, Exti9, 6);
impl_dac_channel_trigger!(DAC2, Timer6, 7);
impl_dac_channel_trigger!(DAC2, Timer3, 8);
impl_dac_channel_trigger!(DAC2, HrtimDacReset1, 9);
impl_dac_channel_trigger!(DAC2, HrtimDacReset2, 10);
impl_dac_channel_trigger!(DAC2, HrtimDacReset3, 11);
impl_dac_channel_trigger!(DAC2, HrtimDacReset4, 12);
impl_dac_channel_trigger!(DAC2, HrtimDacReset5, 13);
impl_dac_channel_trigger!(DAC2, HrtimDacReset6, 14);
impl_dac_channel_trigger!(DAC2, HrtimDacTrigger2, 15);

impl_dac_increment_trigger!(DAC2, Software, 0);
impl_dac_increment_trigger!(DAC2, Timer8, 1);
impl_dac_increment_trigger!(DAC2, Timer7, 2);
impl_dac_increment_trigger!(DAC2, Timer15, 3);
impl_dac_increment_trigger!(DAC2, Timer2, 4);
impl_dac_increment_trigger!(DAC2, Timer4, 5);
impl_dac_increment_trigger!(DAC2, Exti10, 6);
impl_dac_increment_trigger!(DAC2, Timer6, 7);
impl_dac_increment_trigger!(DAC2, Timer3, 8);
impl_dac_increment_trigger!(DAC2, HrtimDacStep1, 9);
impl_dac_increment_trigger!(DAC2, HrtimDacStep2, 10);
impl_dac_increment_trigger!(DAC2, HrtimDacStep3, 11);
impl_dac_increment_trigger!(DAC2, HrtimDacStep4, 12);
impl_dac_increment_trigger!(DAC2, HrtimDacStep5, 13);
impl_dac_increment_trigger!(DAC2, HrtimDacStep6, 14);

impl_dac_channel_trigger!(DAC3, Software, 1);
impl_dac_channel_trigger!(DAC3, Timer1, 1);
impl_dac_channel_trigger!(DAC3, Timer7, 2);
impl_dac_channel_trigger!(DAC3, Timer15, 3);
impl_dac_channel_trigger!(DAC3, Timer2, 4);
impl_dac_channel_trigger!(DAC3, Timer4, 5);
impl_dac_channel_trigger!(DAC3, Exti9, 6);
impl_dac_channel_trigger!(DAC3, Timer6, 7);
impl_dac_channel_trigger!(DAC3, Timer3, 8);
impl_dac_channel_trigger!(DAC3, HrtimDacReset1, 9);
impl_dac_channel_trigger!(DAC3, HrtimDacReset2, 10);
impl_dac_channel_trigger!(DAC3, HrtimDacReset3, 11);
impl_dac_channel_trigger!(DAC3, HrtimDacReset4, 12);
impl_dac_channel_trigger!(DAC3, HrtimDacReset5, 13);
impl_dac_channel_trigger!(DAC3, HrtimDacReset6, 14);
impl_dac_channel_trigger!(DAC3, HrtimDacTrigger3, 15);

impl_dac_increment_trigger!(DAC3, Software, 0);
impl_dac_increment_trigger!(DAC3, Timer1, 1);
impl_dac_increment_trigger!(DAC3, Timer7, 2);
impl_dac_increment_trigger!(DAC3, Timer15, 3);
impl_dac_increment_trigger!(DAC3, Timer2, 4);
impl_dac_increment_trigger!(DAC3, Timer4, 5);
impl_dac_increment_trigger!(DAC3, Exti10, 6);
impl_dac_increment_trigger!(DAC3, Timer6, 7);
impl_dac_increment_trigger!(DAC3, Timer3, 8);
impl_dac_increment_trigger!(DAC3, HrtimDacStep1, 9);
impl_dac_increment_trigger!(DAC3, HrtimDacStep2, 10);
impl_dac_increment_trigger!(DAC3, HrtimDacStep3, 11);
impl_dac_increment_trigger!(DAC3, HrtimDacStep4, 12);
impl_dac_increment_trigger!(DAC3, HrtimDacStep5, 13);
impl_dac_increment_trigger!(DAC3, HrtimDacStep6, 14);

impl_dac_channel_trigger!(DAC4, Software, 1);
impl_dac_channel_trigger!(DAC4, Timer8, 1);
impl_dac_channel_trigger!(DAC4, Timer7, 2);
impl_dac_channel_trigger!(DAC4, Timer15, 3);
impl_dac_channel_trigger!(DAC4, Timer2, 4);
impl_dac_channel_trigger!(DAC4, Timer4, 5);
impl_dac_channel_trigger!(DAC4, Exti9, 6);
impl_dac_channel_trigger!(DAC4, Timer6, 7);
impl_dac_channel_trigger!(DAC4, Timer3, 8);
impl_dac_channel_trigger!(DAC4, HrtimDacReset1, 9);
impl_dac_channel_trigger!(DAC4, HrtimDacReset2, 10);
impl_dac_channel_trigger!(DAC4, HrtimDacReset3, 11);
impl_dac_channel_trigger!(DAC4, HrtimDacReset4, 12);
impl_dac_channel_trigger!(DAC4, HrtimDacReset5, 13);
impl_dac_channel_trigger!(DAC4, HrtimDacReset6, 14);
impl_dac_channel_trigger!(DAC4, HrtimDacTrigger1, 15);

impl_dac_increment_trigger!(DAC4, Software, 0);
impl_dac_increment_trigger!(DAC4, Timer8, 1);
impl_dac_increment_trigger!(DAC4, Timer7, 2);
impl_dac_increment_trigger!(DAC4, Timer15, 3);
impl_dac_increment_trigger!(DAC4, Timer2, 4);
impl_dac_increment_trigger!(DAC4, Timer4, 5);
impl_dac_increment_trigger!(DAC4, Exti10, 6);
impl_dac_increment_trigger!(DAC4, Timer6, 7);
impl_dac_increment_trigger!(DAC4, Timer3, 8);
impl_dac_increment_trigger!(DAC4, HrtimDacStep1, 9);
impl_dac_increment_trigger!(DAC4, HrtimDacStep2, 10);
impl_dac_increment_trigger!(DAC4, HrtimDacStep3, 11);
impl_dac_increment_trigger!(DAC4, HrtimDacStep4, 12);
impl_dac_increment_trigger!(DAC4, HrtimDacStep5, 13);
impl_dac_increment_trigger!(DAC4, HrtimDacStep6, 14);
