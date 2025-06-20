use stm32_hrtim::external_event::{EevInput, EevSrcBits, SourceBuilder};

use crate::gpio::{
    self,
    gpiob::{PB3, PB4, PB5, PB6, PB7, PB8, PB9},
    gpioc::{PC11, PC12, PC5, PC6},
};

pub trait EevInputExt<const N: u8> {
    fn bind<const IS_FAST: bool, SRC>(self, src: SRC) -> SourceBuilder<N, IS_FAST>
    where
        SRC: EevSrcBits<N>;
}

macro_rules! impl_eev_input {
    ($($N:literal: COMP=[$compX:ident $(, ($compY:ident, $compY_src_bits:literal))*], PINS=[$(($pin:ident, $af:literal)),*])*) => {$(
        $(unsafe impl EevSrcBits<$N> for $pin<gpio::Input>{
            const SRC_BITS: u8 = 0b00;
            fn cfg(self) {
                self.into_alternate::<$af>();
            }
        })*

        unsafe impl<ED> EevSrcBits<$N> for &crate::comparator::Comparator<crate::comparator::$compX, ED>
            where ED: crate::comparator::EnabledState
        {
            const SRC_BITS: u8 = 0b01;
        }

        $(
            unsafe impl<ED> EevSrcBits<$N> for &crate::comparator::Comparator<crate::comparator::$compY, ED>
                where ED: crate::comparator::EnabledState
            {
                const SRC_BITS: u8 = $compY_src_bits;
            }
        )*

        impl EevInputExt<$N> for EevInput<$N> {
            fn bind<const IS_FAST: bool, SRC>(self, src: SRC) -> SourceBuilder<$N, IS_FAST>
                where SRC: EevSrcBits<$N>
            {
                src.cfg();
                unsafe { SourceBuilder::new(SRC::SRC_BITS) }
            }
        }
    )*};
}

impl_eev_input! {
    1: COMP = [COMP2], PINS = [(PC12, 3)]
    2: COMP = [COMP4], PINS = [(PC11, 3)]
    3: COMP = [COMP6], PINS = [(PB7, 13)]
    4: COMP = [COMP1, (COMP5, 0b10)], PINS = [(PB6, 13)]
    5: COMP = [COMP3, (COMP7, 0b10)], PINS = [(PB9, 13)]
    6: COMP = [COMP2, (COMP1, 0b10)], PINS = [(PB5, 13)]
    7: COMP = [COMP4], PINS = [(PB4, 13)]
    8: COMP = [COMP6, (COMP3, 0b10)], PINS = [(PB8, 13)]
    9: COMP = [COMP5, (COMP4, 0b11)], PINS = [(PB3, 13)]
    10: COMP = [COMP7], PINS = [(PC5, 13), (PC6, 3)]
}
