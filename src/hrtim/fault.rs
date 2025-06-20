use stm32_hrtim::fault::{self, SourceBuilder};

use crate::{
    comparator::{COMP1, COMP2, COMP3, COMP4, COMP5, COMP6},
    gpio::{
        self,
        gpioa::{PA12, PA15},
        gpiob::{PB0, PB10, PB11},
        gpioc::{PC10, PC7},
    },
};

// TODO: Come up with a better names

pub trait FaultInput<S> {
    fn bind(self, src: S) -> SourceBuilder<Self>
    where
        Self: Sized;
}

macro_rules! impl_faults {
    ($(
        $input:ident:
            PINS=[($pin:ident, $af:literal) $(,($pin_b:ident, $af_b:literal))*],
            COMP=$compX:ident,
    )+) => {$(
        impl FaultInput<$pin<gpio::Alternate<$af>>> for fault::$input {
            fn bind(self, _pin: $pin<gpio::Alternate<$af>>) -> SourceBuilder<fault::$input> {
                unsafe { SourceBuilder::new(self, 0b00) }
            }
        }

        $(
            impl FaultInput<$pin_b<gpio::Alternate<$af_b>>> for fault::$input {
                fn bind(self, _pin: $pin_b<gpio::Alternate<$af_b>>) -> SourceBuilder<fault::$input> {
                    unsafe { SourceBuilder::new(self, 0b00) }
                }
            }
        )*

        impl FaultInput<crate::comparator::Comparator<$compX, crate::comparator::Enabled>> for fault::$input {
            fn bind(self, _comp: crate::comparator::Comparator<$compX, crate::comparator::Enabled>) -> SourceBuilder<fault::$input> {
                unsafe { SourceBuilder::new(self, 0b01) }
            }
        }

        /*pub fn bind_external(?) {
            SourceBuilder::new(self, 0b10);
        }*/
    )+}
}

impl_faults!(
    FaultInput1: PINS=[(PA12, 13)], COMP=COMP2,
    FaultInput2: PINS=[(PA15, 13)], COMP=COMP4,
    FaultInput3: PINS=[(PB10, 13)], COMP=COMP6,
    FaultInput4: PINS=[(PB11, 13)], COMP=COMP1,
    FaultInput5: PINS=[(PB0, 13), (PC7, 3)], COMP=COMP3,
    FaultInput6: PINS=[(PC10, 13)], COMP=COMP5,
);
