use core::marker::PhantomData;

use crate::comparator::{self, Comparator, COMP1, COMP2, COMP3, COMP4, COMP5, COMP6, COMP7};
use crate::gpio::gpiob::{PB3, PB4, PB5, PB6, PB7, PB8, PB9};
use crate::gpio::gpioc::{PC11, PC12, PC5, PC6};
use crate::gpio::{self, AF13, AF3};
use crate::stm32::HRTIM_COMMON;

use super::{control::HrTimCalibrated, event::EventSource};

#[derive(Copy, Clone, PartialEq)]
pub enum ExternalEventSource {
    Eevnt1 { _x: PhantomData<()> },
    Eevnt2 { _x: PhantomData<()> },
    Eevnt3 { _x: PhantomData<()> },
    Eevnt4 { _x: PhantomData<()> },
    Eevnt5 { _x: PhantomData<()> },
    Eevnt6 { _x: PhantomData<()> },
    Eevnt7 { _x: PhantomData<()> },
    Eevnt8 { _x: PhantomData<()> },
    Eevnt9 { _x: PhantomData<()> },
    Eevnt10 { _x: PhantomData<()> },
}

pub struct EevInputs {
    pub eev_input1: EevInput<1>,
    pub eev_input2: EevInput<2>,
    pub eev_input3: EevInput<3>,
    pub eev_input4: EevInput<4>,
    pub eev_input5: EevInput<5>,
    pub eev_input6: EevInput<6>,
    pub eev_input7: EevInput<7>,
    pub eev_input8: EevInput<8>,
    pub eev_input9: EevInput<9>,
    pub eev_input10: EevInput<10>,
}

impl EevInputs {
    pub(crate) unsafe fn new() -> Self {
        EevInputs {
            eev_input1: EevInput { _x: PhantomData },
            eev_input2: EevInput { _x: PhantomData },
            eev_input3: EevInput { _x: PhantomData },
            eev_input4: EevInput { _x: PhantomData },
            eev_input5: EevInput { _x: PhantomData },
            eev_input6: EevInput { _x: PhantomData },
            eev_input7: EevInput { _x: PhantomData },
            eev_input8: EevInput { _x: PhantomData },
            eev_input9: EevInput { _x: PhantomData },
            eev_input10: EevInput { _x: PhantomData },
        }
    }
}

pub struct EevInput<const N: u8> {
    _x: PhantomData<()>,
}

macro_rules! impl_eev_input {
    ($N:literal: $compX:ident, PINS=[($pin:ident, $af:ident) $(, ($pin_b:ident, $af_b:ident))*]) => {
        impl EevInput<$N> {
            pub fn bind_pin<const IS_FAST: bool, IM>(self, pin: $pin<gpio::Input<IM>>) -> SourceBuilder<$N, IS_FAST> {
                pin.into_alternate::<$af>();
                unsafe { SourceBuilder::new(0b00) }
            }

            $(
                // TODO: Is there a nicer way to do this?
                pub fn bind_pin_b<const IS_FAST: bool, IM>(self, pin: $pin_b<gpio::Input<IM>>) -> SourceBuilder<$N, IS_FAST> {
                    pin.into_alternate::<$af_b>();
                    unsafe { SourceBuilder::new(0b00) }
                }
            )*

            pub fn bind_comp<const IS_FAST: bool>(self, _comp: &crate::comparator::Comparator<$compX, crate::comparator::Enabled>) -> SourceBuilder<$N, IS_FAST> {
                unsafe { SourceBuilder::new(0b01) }
            }
        }
    };
}

impl_eev_input!(1: COMP2, PINS=[(PC12, AF3)]);
impl_eev_input!(2: COMP4, PINS=[(PC11, AF3)]);
impl_eev_input!(3: COMP6, PINS=[(PB7, AF13)]);
impl_eev_input!(4: COMP1, PINS=[(PB6, AF13)]);
impl_eev_input!(5: COMP3, PINS=[(PB9, AF13)]);
impl_eev_input!(6: COMP2, PINS=[(PB5, AF13)]);
impl_eev_input!(7: COMP4, PINS=[(PB4, AF13)]);
impl_eev_input!(8: COMP6, PINS=[(PB8, AF13)]);
impl_eev_input!(9: COMP5, PINS=[(PB3, AF13)]);
impl_eev_input!(10: COMP7, PINS=[(PC5, AF13), (PC6, AF3)]);

pub enum EdgeOrPolarity {
    Edge(Edge),
    Polarity(Polarity),
}

pub enum Edge {
    Rising = 0b01,
    Falling = 0b10,
    Both = 0b11,
}

pub enum Polarity {
    ActiveHigh,
    ActiveLow,
}

pub enum EevSamplingFilter {
    /// No filtering, fault acts asynchronously
    ///
    /// Note that this bypasses any f_eevs (FaultSamplingClkDiv)
    None = 0b0000,

    /// Sample directly at rate f_hrtim, with a count of 2
    ///
    /// Note that this bypasses: any f_eevs (FaultSamplingClkDiv)
    HrtimN2 = 0b0001,

    /// Sample directly at rate f_hrtim, with a count of 4
    ///
    /// Note that this bypasses any f_eevs (FaultSamplingClkDiv)
    HrtimN4 = 0b0010,

    /// Sample directly at rate f_hrtim, with a count of 8
    ///
    /// Note that this bypasses any f_eevs (FaultSamplingClkDiv)
    HrtimN8 = 0b0011,

    /// Sample at rate f_eevs / 2, with a count of 6
    EevsDiv2N6 = 0b0100,

    /// Sample at rate f_eevs / 2, with a count of 8
    EevsDiv2N8 = 0b0101,

    /// Sample at rate f_eevs / 4, with a count of 6
    EevsDiv4N6 = 0b0110,

    /// Sample at rate f_eevs / 4, with a count of 8
    EevsDiv4N8 = 0b0111,

    /// Sample at rate f_eevs / 8, with a count of 6
    EevsDiv8N6 = 0b1000,

    /// Sample at rate f_eevs / 8, with a count of 8
    EevsDiv8N8 = 0b1001,

    /// Sample at rate f_eevs / 16, with a count of 5
    EevsDiv16N5 = 0b1010,

    /// Sample at rate f_eevs / 16, with a count of 6
    EevsDiv16N6 = 0b1011,

    /// Sample at rate f_eevs / 16, with a count of 8
    EevsDiv16N8 = 0b1100,

    /// Sample at rate f_eevs / 32, with a count of 5
    EevsDiv32N5 = 0b1101,

    /// Sample at rate f_eevs / 32, with a count of 6
    EevsDiv32N6 = 0b1110,

    /// Sample at rate f_eevs / 32, with a count of 8
    EevsDiv32N8 = 0b1111,
}

pub trait ExternalEventBuilder1To5 {}
pub trait ExternalEventBuilder6To10 {}
pub struct SourceBuilder<const N: u8, const IS_FAST: bool> {
    /// EExSRC
    src_bits: u8,

    /// EExSNS
    edge_or_polarity_bits: u8,

    /// EExPOL
    polarity_bit: bool,

    /// EExF
    filter_bits: u8,
}

impl<const N: u8, const IS_FAST: bool> SourceBuilder<N, IS_FAST> {
    unsafe fn new(src_bits: u8) -> Self {
        Self {
            src_bits,
            edge_or_polarity_bits: 0, // Level sensitive
            polarity_bit: false,      // Active high
            filter_bits: 0,           // No filter
        }
    }
}

impl<const N: u8> SourceBuilder<N, false> {
    pub fn edge_or_polarity(mut self, edge_or_polarity: EdgeOrPolarity) -> Self {
        (self.edge_or_polarity_bits, self.polarity_bit) = match edge_or_polarity {
            EdgeOrPolarity::Polarity(Polarity::ActiveHigh) => (0b00, false),
            EdgeOrPolarity::Polarity(Polarity::ActiveLow) => (0b00, true),
            EdgeOrPolarity::Edge(Edge::Rising) => (0b01, false),
            EdgeOrPolarity::Edge(Edge::Falling) => (0b10, false),
            EdgeOrPolarity::Edge(Edge::Both) => (0b11, false),
        };

        self
    }
}

impl<const N: u8> SourceBuilder<N, true> {
    /// Edge sensitivity not available in fast mode
    pub fn polarity(mut self, polarity: Polarity) -> Self {
        (self.edge_or_polarity_bits, self.polarity_bit) = match polarity {
            Polarity::ActiveHigh => (0b00, false),
            Polarity::ActiveLow => (0b00, true),
        };

        self
    }
}

impl<const N: u8> SourceBuilder<N, false>
where
    SourceBuilder<N, false>: ExternalEventBuilder6To10,
{
    pub fn filter(mut self, filter: EevSamplingFilter) -> Self {
        self.filter_bits = filter as _;
        self
    }
}

pub trait ToExternalEventSource {
    fn finalize<PSCL, TIM>(self, _calibrated: &mut HrTimCalibrated) -> ExternalEventSource;
}

#[derive(Copy, Clone)]
struct ExternalEventMuxOut<const N: u8> {
    _x: PhantomData<()>,
}

macro_rules! impl_eev1_5_to_es {
    ($eev:ident, $N:literal, $eeXsrc:ident, $eeXpol:ident, $eeXsns:ident, $eeXfast:ident) => {
        impl<const IS_FAST: bool> ExternalEventBuilder1To5 for SourceBuilder<$N, IS_FAST> {}

        impl<const IS_FAST: bool> ToExternalEventSource for SourceBuilder<$N, IS_FAST> {
            fn finalize<PSCL, TIM>(self, _calibrated: &mut HrTimCalibrated) -> ExternalEventSource {
                let SourceBuilder {
                    src_bits,
                    edge_or_polarity_bits,
                    polarity_bit,
                    filter_bits: _,
                } = self;

                let common = unsafe { &*HRTIM_COMMON::ptr() };

                // SAFETY: Thanks to, `HrTimCalibrated`, we know we have exclusive access to the register,
                //         we also know no timers are started.
                unsafe {
                    common.eecr1.modify(|_r, w| {
                        w.$eeXsrc()
                            .bits(src_bits)
                            .$eeXpol()
                            .bit(polarity_bit)
                            .$eeXsns()
                            .bits(edge_or_polarity_bits)
                            .$eeXfast()
                            .bit(IS_FAST)
                    });
                }

                ExternalEventSource::$eev { _x: PhantomData }
            }
        }
    };
}

macro_rules! impl_eev6_10_to_es {
    ($eev:ident, $N:literal, $eeXsrc:ident, $eeXpol:ident, $eeXsns:ident, $eeXf:ident) => {
        impl ExternalEventBuilder6To10 for SourceBuilder<$N, false> {}

        impl ToExternalEventSource for SourceBuilder<$N, false> {
            fn finalize<PSCL, TIM>(self, _calibrated: &mut HrTimCalibrated) -> ExternalEventSource {
                let SourceBuilder {
                    src_bits,
                    edge_or_polarity_bits,
                    polarity_bit,
                    filter_bits,
                } = self;

                let common = unsafe { &*HRTIM_COMMON::ptr() };

                // SAFETY: Thanks to, `HrTimCalibrated`, we know we have exclusive access to the register,
                //         we also know no timers are started.
                unsafe {
                    common.eecr2.modify(|_r, w| {
                        w.$eeXsrc()
                            .bits(src_bits)
                            .$eeXpol()
                            .bit(polarity_bit)
                            .$eeXsns()
                            .bits(edge_or_polarity_bits)
                    });
                    common.eecr3.modify(|_r, w| w.$eeXf().bits(filter_bits));
                }

                ExternalEventSource::$eev { _x: PhantomData }
            }
        }
    };
}

impl_eev1_5_to_es!(Eevnt1, 1, ee1src, ee1pol, ee1sns, ee1fast);
impl_eev1_5_to_es!(Eevnt2, 2, ee2src, ee2pol, ee2sns, ee2fast);
impl_eev1_5_to_es!(Eevnt3, 3, ee3src, ee3pol, ee3sns, ee3fast);
impl_eev1_5_to_es!(Eevnt4, 4, ee4src, ee4pol, ee4sns, ee4fast);
impl_eev1_5_to_es!(Eevnt5, 5, ee5src, ee5pol, ee5sns, ee5fast);

impl_eev6_10_to_es!(Eevnt6, 6, ee6src, ee6pol, ee6sns, ee6f);
impl_eev6_10_to_es!(Eevnt7, 7, ee7src, ee7pol, ee7sns, ee7f);
impl_eev6_10_to_es!(Eevnt8, 8, ee8src, ee8pol, ee8sns, ee8f);
impl_eev6_10_to_es!(Eevnt9, 9, ee9src, ee9pol, ee9sns, ee9f);
impl_eev6_10_to_es!(Eevnt10, 10, ee10src, ee10pol, ee10sns, ee10f);
