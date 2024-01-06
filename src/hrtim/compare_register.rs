use core::marker::PhantomData;

use crate::stm32::{
    HRTIM_MASTER, HRTIM_TIMA, HRTIM_TIMB, HRTIM_TIMC, HRTIM_TIMD, HRTIM_TIME, HRTIM_TIMF,
};

pub trait HrCompareRegister {
    fn get_duty(&self) -> u16;
    fn set_duty(&mut self, duty: u16);
}

pub struct HrCr1<TIM, PSCL>(PhantomData<(TIM, PSCL)>);
pub struct HrCr2<TIM, PSCL>(PhantomData<(TIM, PSCL)>);
pub struct HrCr3<TIM, PSCL>(PhantomData<(TIM, PSCL)>);
pub struct HrCr4<TIM, PSCL>(PhantomData<(TIM, PSCL)>);


use super::adc_trigger::Adc13Trigger as Adc13;
use super::adc_trigger::Adc24Trigger as Adc24;
use super::adc_trigger::Adc579Trigger as Adc579;
use super::adc_trigger::Adc6810Trigger as Adc6810;

macro_rules! hrtim_cr_helper {
    ($TIMX:ident: $cr_type:ident: $cmpXYr:ident, $cmpYx:ident, $(($Trigger:ty: $trigger_bits:expr)),*) => {
        impl<PSCL> HrCompareRegister for $cr_type<$TIMX, PSCL> {
            fn get_duty(&self) -> u16 {
                let tim = unsafe { &*$TIMX::ptr() };

                tim.$cmpXYr.read().$cmpYx().bits()
            }
            fn set_duty(&mut self, duty: u16) {
                let tim = unsafe { &*$TIMX::ptr() };

                tim.$cmpXYr.write(|w| unsafe { w.$cmpYx().bits(duty) });
            }
        }

        $(impl<PSCL> $Trigger for $cr_type<$TIMX, PSCL> {
            const BITS: u32 = $trigger_bits;
        })*
    };
}


macro_rules! hrtim_cr {
    ($($TIMX:ident: [
        [$cmpX1r:ident, $cmp1x:ident, $(($cr1_trigger:ident: $cr1_trigger_bits:expr)),*],
        [$cmpX2r:ident, $cmp2x:ident, $(($cr2_trigger:ident: $cr2_trigger_bits:expr)),*],
        [$cmpX3r:ident, $cmp3x:ident, $(($cr3_trigger:ident: $cr3_trigger_bits:expr)),*],
        [$cmpX4r:ident, $cmp4x:ident, $(($cr4_trigger:ident: $cr4_trigger_bits:expr)),*]
    ]),+) => {$(
        hrtim_cr_helper!($TIMX: HrCr1: $cmpX1r, $cmp1x, $(($cr1_trigger: $cr1_trigger_bits)),*);
        hrtim_cr_helper!($TIMX: HrCr2: $cmpX2r, $cmp2x, $(($cr2_trigger: $cr2_trigger_bits)),*);
        hrtim_cr_helper!($TIMX: HrCr3: $cmpX3r, $cmp3x, $(($cr3_trigger: $cr3_trigger_bits)),*);
        hrtim_cr_helper!($TIMX: HrCr4: $cmpX4r, $cmp4x, $(($cr4_trigger: $cr4_trigger_bits)),*);
    )+};
}

hrtim_cr! {
    HRTIM_MASTER: [
        [mcmp1r,mcmp1,  (Adc13: 1 << 0),  (Adc24: 1 << 0),  (Adc579: 0),  (Adc6810: 0) ], 
        [mcmp2r,mcmp2,  (Adc13: 1 << 1),  (Adc24: 1 << 1),  (Adc579: 1),  (Adc6810: 1) ], 
        [mcmp3r,mcmp3,  (Adc13: 1 << 2),  (Adc24: 1 << 2),  (Adc579: 2),  (Adc6810: 2) ], 
        [mcmp4r,mcmp4,  (Adc13: 1 << 3),  (Adc24: 1 << 3),  (Adc579: 3),  (Adc6810: 3) ]
    ],
    
    HRTIM_TIMA: [
        [cmp1ar, cmp1x,                                                                ],
        [cmp2ar, cmp2x,                   (Adc24: 1 << 10),               (Adc6810: 10)],
        [cmp3ar, cmp3x, (Adc13: 1 << 11),                   (Adc579: 10)               ],
        [cmp4ar, cmp4x, (Adc13: 1 << 12), (Adc24: 1 << 12), (Adc579: 11), (Adc6810: 11)]
    ],
    
    HRTIM_TIMB: [
        [cmp1br, cmp1x,                                                                ],
        [cmp2br, cmp2x,                   (Adc24: 1 << 14),               (Adc6810: 13)],
        [cmp3br, cmp3x, (Adc13: 1 << 16),                   (Adc579: 14)               ],
        [cmp4br, cmp4x, (Adc13: 1 << 17), (Adc24: 1 << 16), (Adc579: 15), (Adc6810: 14)]
    ],

    HRTIM_TIMC: [
        [cmp1cr, cmp1x,                                                                ],
        [cmp2cr, cmp2x,                   (Adc24: 1 << 18),               (Adc6810: 16)],
        [cmp3cr, cmp3x, (Adc13: 1 << 21),                   (Adc579: 18)               ],
        [cmp4cr, cmp4x, (Adc13: 1 << 22), (Adc24: 1 << 20), (Adc579: 19), (Adc6810: 17)]
    ],

    HRTIM_TIMD: [
        [cmp1dr, cmp1x,                                                                ],
        [cmp2dr, cmp2x,                   (Adc24: 1 << 23),               (Adc6810: 20)],
        [cmp3dr, cmp3x, (Adc13: 1 << 25),                   (Adc579: 21)               ],
        [cmp4dr, cmp4x, (Adc13: 1 << 26), (Adc24: 1 << 25), (Adc579: 22), (Adc6810: 21)]
    ],

    HRTIM_TIME: [
        [cmp1er, cmp1x,                                                                ],
        [cmp2er, cmp2x,                   (Adc24: 1 << 28),               (Adc6810: 24)],
        [cmp3er, cmp3x, (Adc13: 1 << 29), (Adc24: 1 << 29), (Adc579: 24), (Adc6810: 25)],
        [cmp4er, cmp4x, (Adc13: 1 << 30), (Adc24: 1 << 30), (Adc579: 25), (Adc6810: 26)]
    ],

    HRTIM_TIMF: [
        [cmp1fr, cmp1x,                   (Adc24: 1 << 15)                             ],
        [cmp2fr, cmp2x, (Adc13: 1 << 10), (Adc24: 1 << 11), (Adc579: 27), (Adc6810: 28)],
        [cmp3fr, cmp3x, (Adc13: 1 << 15),                   (Adc579: 28), (Adc6810: 29)],
        [cmp4fr, cmp4x, (Adc13: 1 << 20), (Adc24: 1 << 19), (Adc579: 29), (Adc6810: 30)]
    ]
}
