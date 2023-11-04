use core::marker::PhantomData;

use stm32g4::stm32g474::HRTIM_COMMON;

use crate::comparator::{self, Comparator, COMP1, COMP2, COMP4, COMP6, COMP3, COMP5, COMP7};

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

impl<PSCL, DST> From<ExternalEventSource> for EventSource<PSCL, DST> {
    fn from(val: ExternalEventSource) -> Self {
        EventSource::ExternalEvent(val)
    }
}

pub trait ExternalEventExt<const EEV_N: u8, const SRC: u8> {
    fn external_event(&self) -> ExternalEventBuilder<EEV_N, SRC>;
}

macro_rules! impl_eev_ext_comp {
    ($N:literal: $comp:ident) => {
        impl<E: comparator::EnabledState> ExternalEventExt<$N, 0b10> for Comparator<$comp, E> {
            fn external_event(&self) -> ExternalEventBuilder<$N, 0b10> {
                ExternalEventBuilder {
                    edge_or_polarity_bits: 0, // Polarity sensitive
                    polarity_bit: false,      // Active high
                    filter_bits: 0,           // No filter
                    fast_bit: false,          // Not fast
                }
            }
        }
    };
}

impl_eev_ext_comp!(1: COMP2);
impl_eev_ext_comp!(2: COMP4);
impl_eev_ext_comp!(3: COMP6);
impl_eev_ext_comp!(4: COMP1);
impl_eev_ext_comp!(5: COMP3);
impl_eev_ext_comp!(6: COMP2);
impl_eev_ext_comp!(7: COMP4);
impl_eev_ext_comp!(8: COMP6);
impl_eev_ext_comp!(9: COMP5);
impl_eev_ext_comp!(10: COMP7);

impl_eev_ext_pin!(1: PC12);
impl_eev_ext_pin!(2: PC11);
impl_eev_ext_pin!(3: PB7);
impl_eev_ext_pin!(4: PB6);
impl_eev_ext_pin!(5: PB9);
impl_eev_ext_pin!(6: PB5);
impl_eev_ext_pin!(7: PB4);
impl_eev_ext_pin!(8: PB8);
impl_eev_ext_pin!(9: PB3);
impl_eev_ext_pin!(10: PC5);
impl_eev_ext_pin!(10: PC6);



pub struct ExternalEventBuilder<const N: u8, const SRC: u8> {
    /// EExSNS
    edge_or_polarity_bits: u8,

    /// EExPOL
    polarity_bit: bool,

    /// EExF
    filter_bits: u8,

    /// EExFAST
    fast_bit: bool,
}

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

impl<const N: u8, const SRC: u8> ExternalEventBuilder<N, SRC> {
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

impl<const N: u8, const SRC: u8> ExternalEventBuilder<N, SRC>
where
    ExternalEventBuilder<N, SRC>: ExternalEventBuilder1To5,
{
    pub fn fast(mut self) -> Self {
        self.fast_bit = true;
        self
    }
}

impl<const N: u8, const SRC: u8> ExternalEventBuilder<N, SRC>
where
    ExternalEventBuilder<N, SRC>: ExternalEventBuilder6To10,
{
    pub fn filter(mut self, filter: EevSamplingFilter) -> Self {
        self.filter_bits = filter as _;
        self
    }
}

pub trait ExternalEventToEventSource {
    fn finalize<PSCL, TIM>(self, _calibrated: &mut HrTimCalibrated) -> ExternalEventSource;
}

macro_rules! impl_eev1_5_to_es {
    ($eev:ident, $N:literal, $eeXsrc:ident, $eeXpol:ident, $eeXsns:ident, $eeXfast:ident) => {
        impl<const SRC: u8> ExternalEventBuilder1To5 for ExternalEventBuilder<$N, SRC> {}

        impl<const SRC: u8> ExternalEventToEventSource for ExternalEventBuilder<$N, SRC> {
            fn finalize<PSCL, TIM>(self, _calibrated: &mut HrTimCalibrated) -> ExternalEventSource {
                let ExternalEventBuilder {
                    edge_or_polarity_bits,
                    polarity_bit,
                    filter_bits: _,
                    fast_bit,
                } = self;

                let common = unsafe { &*HRTIM_COMMON::ptr() };

                // SAFETY: Thanks to, `HrTimCalibrated`, we know we have exclusive access to the register,
                //         we also know no timers are started.
                unsafe {
                    common.eecr1.modify(|_r, w| {
                        w.$eeXsrc()
                            .bits(SRC)
                            .$eeXpol()
                            .bit(polarity_bit)
                            .$eeXsns()
                            .bits(edge_or_polarity_bits)
                            .$eeXfast()
                            .bit(fast_bit)
                    });
                }

                ExternalEventSource::$eev { _x: PhantomData }
            }
        }
    };
}

macro_rules! impl_eev6_10_to_es {
    ($eev:ident, $N:literal, $eeXsrc:ident, $eeXpol:ident, $eeXsns:ident, $eeXf:ident) => {
        impl<const SRC: u8> ExternalEventBuilder6To10 for ExternalEventBuilder<$N, SRC> {}

        impl<const SRC: u8> ExternalEventToEventSource for ExternalEventBuilder<$N, SRC> {
            fn finalize<PSCL, TIM>(self, _calibrated: &mut HrTimCalibrated) -> ExternalEventSource {
                let ExternalEventBuilder {
                    edge_or_polarity_bits,
                    polarity_bit,
                    filter_bits,
                    fast_bit: _,
                } = self;

                let common = unsafe { &*HRTIM_COMMON::ptr() };

                // SAFETY: Thanks to, `HrTimCalibrated`, we know we have exclusive access to the register,
                //         we also know no timers are started.
                unsafe {
                    common.eecr2.modify(|_r, w| {
                        w.$eeXsrc()
                            .bits(SRC)
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
