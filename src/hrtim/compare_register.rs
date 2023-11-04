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

macro_rules! hrtim_cr_helper {
    ($TIMX:ident: $cr_type:ident: $cmpXYr:ident, $cmpYx:ident) => {
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
    };
}

macro_rules! hrtim_cr {
    ($($TIMX:ident: [
        $cmpX1r:ident, $cmpX2r:ident, $cmpX3r:ident, $cmpX4r:ident,
        $cmp1x:ident, $cmp2x:ident, $cmp3x:ident, $cmp4x:ident
    ],)+) => {$(
        hrtim_cr_helper!($TIMX: HrCr1: $cmpX1r, $cmp1x);
        hrtim_cr_helper!($TIMX: HrCr2: $cmpX2r, $cmp2x);
        hrtim_cr_helper!($TIMX: HrCr3: $cmpX3r, $cmp3x);
        hrtim_cr_helper!($TIMX: HrCr4: $cmpX4r, $cmp4x);
    )+};
}

hrtim_cr! {
    HRTIM_MASTER: [mcmp1r, mcmp2r, mcmp3r, mcmp4r, mcmp1, mcmp2, mcmp3, mcmp4],

    HRTIM_TIMA: [cmp1ar, cmp2ar, cmp3ar, cmp4ar, cmp1x, cmp2x, cmp3x, cmp4x],
    HRTIM_TIMB: [cmp1br, cmp2br, cmp3br, cmp4br, cmp1x, cmp2x, cmp3x, cmp4x],
    HRTIM_TIMC: [cmp1cr, cmp2cr, cmp3cr, cmp4cr, cmp1x, cmp2x, cmp3x, cmp4x],
    HRTIM_TIMD: [cmp1dr, cmp2dr, cmp3dr, cmp4dr, cmp1x, cmp2x, cmp3x, cmp4x],
    HRTIM_TIME: [cmp1er, cmp2er, cmp3er, cmp4er, cmp1x, cmp2x, cmp3x, cmp4x],
    HRTIM_TIMF: [cmp1fr, cmp2fr, cmp3fr, cmp4fr, cmp1x, cmp2x, cmp3x, cmp4x],
}
