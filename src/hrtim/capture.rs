use stm32_hrtim::{
    capture::{Ch1, Ch2, Dma, HrCapt},
    pac::{HRTIM_TIMA, HRTIM_TIMB, HRTIM_TIMC, HRTIM_TIMD, HRTIM_TIME, HRTIM_TIMF},
};

use crate::dma::{mux::DmaMuxResources, traits::TargetAddress, PeripheralToMemory};

macro_rules! impl_dma_traits {
    ($TIMX:ident, $CH:ident, $cptXr:ident) => {
        unsafe impl<PSCL> TargetAddress<PeripheralToMemory> for HrCapt<$TIMX, PSCL, $CH, Dma> {
            #[inline(always)]
            fn address(&self) -> u32 {
                let tim = unsafe { &*$TIMX::ptr() };
                &tim.$cptXr() as *const _ as u32
            }

            type MemSize = u32;

            const REQUEST_LINE: Option<u8> = Some(DmaMuxResources::$TIMX as u8);
        }
    };
    ($($TIMX:ident),+) => {
        $(
            impl_dma_traits!($TIMX, Ch1, cpt1r);
            impl_dma_traits!($TIMX, Ch2, cpt2r);
        )+
    }
}

impl_dma_traits! {
    HRTIM_TIMA, HRTIM_TIMB, HRTIM_TIMC, HRTIM_TIMD, HRTIM_TIME, HRTIM_TIMF
}
