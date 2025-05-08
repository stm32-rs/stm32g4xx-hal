use crate::adc;
use stm32_hrtim::adc_trigger::{
    AdcTrigger1, AdcTrigger10, AdcTrigger2, AdcTrigger3, AdcTrigger4, AdcTrigger5, AdcTrigger6,
    AdcTrigger7, AdcTrigger8, AdcTrigger9,
};

macro_rules! impl_adc1234_trigger {
    ($($t:ident: [$variant345:ident $(, $variant12:ident)*]),*) => {$(
        $(impl From<&$t> for adc::config::ExternalTrigger12 {
            fn from(_val: &$t) -> Self {
                adc::config::ExternalTrigger12::$variant12
            }
        })*

        impl From<&$t> for adc::config::ExternalTrigger345 {
            fn from(_val: &$t) -> Self {
                adc::config::ExternalTrigger345::$variant345
            }
        }
    )*}
}

macro_rules! impl_adc5678910_trigger {
    ($($t:ident: [$variant:ident]),*) => {$(
        impl From<&$t> for adc::config::ExternalTrigger12 {
            fn from(_val: &$t) -> Self {
                adc::config::ExternalTrigger12::$variant
            }
        }

        impl From<&$t> for adc::config::ExternalTrigger345 {
            fn from(_val: &$t) -> Self {
                adc::config::ExternalTrigger345::$variant
            }
        }
    )*}
}

impl_adc1234_trigger! {//adc345,          adc12
    AdcTrigger1: [Hrtim_adc_trg_1, Hrtim_adc_trg_1],
    AdcTrigger2: [Hrtim_adc_trg_2],
    AdcTrigger3: [Hrtim_adc_trg_3, Hrtim_adc_trg_3],
    AdcTrigger4: [Hrtim_adc_trg_4]
}

impl_adc5678910_trigger! {
    AdcTrigger5: [Hrtim_adc_trg_5],
    AdcTrigger6: [Hrtim_adc_trg_6],
    AdcTrigger7: [Hrtim_adc_trg_7],
    AdcTrigger8: [Hrtim_adc_trg_8],
    AdcTrigger9: [Hrtim_adc_trg_9],
    AdcTrigger10: [Hrtim_adc_trg_10]
}
