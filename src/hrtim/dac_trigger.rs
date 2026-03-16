use stm32_hrtim::DacResetOnCounterReset;
use stm32_hrtim::{
    compare_register::HrCr2, output::HrOut1, timer::HrTim, DacResetOnOut1Set, DacResetTrigger,
    DacStepOnCmp2, DacStepOnOut1Rst, DacStepTrigger,
};

use crate::dac::{IncTriggerSource, TriggerSource as RstTriggerSource};
use crate::stm32;

// TODO: use crate::stasis instead of references
macro_rules! impl_dac_triggers {
    ($($TIM:ident: $bits:expr),*) => {$(
        unsafe impl<PSCL> IncTriggerSource for &HrCr2<stm32::$TIM, PSCL, DacStepOnCmp2> {
            const BITS: u8 = $bits;
        }
        unsafe impl<PSCL, R: DacResetTrigger> IncTriggerSource for &HrOut1<stm32::$TIM, PSCL, R, DacStepOnOut1Rst> {
            const BITS: u8 = $bits;
        }

        unsafe impl<PSCL, CPT1, CPT2> RstTriggerSource for &HrTim<stm32::$TIM, PSCL, CPT1, CPT2, DacResetOnCounterReset> {
            const BITS: u8 = $bits;
        }
        unsafe impl<PSCL, S: DacStepTrigger> RstTriggerSource for &HrOut1<stm32::$TIM, PSCL, DacResetOnOut1Set, S> {
            const BITS: u8 = $bits;
        }
    )*};
}

// RM0440 DAC1 interconnection
impl_dac_triggers! {
    HRTIM_TIMA: 9,
    HRTIM_TIMB: 10,
    HRTIM_TIMC: 11,
    HRTIM_TIMD: 12,
    HRTIM_TIME: 13,
    HRTIM_TIMF: 14
}
