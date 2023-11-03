use crate::comparator::{Comparator, self, COMP1};

pub trait ExternalEventExt<const EEV_N: u8, const SRC: u8> {
    fn external_event(&self) -> ExternalEventBuilder<EEV_N, SRC>;
}

macro_rules! impl_eev_ext_comp {
    ($comp:ident: $N:literal) => {
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

impl_eev_ext_comp!(COMP1: 4);

struct ExternalEventBuilder<const N: u8, const SRC: u8> {
    edge_or_polarity_bits: u8,
    polarity_bit: bool,
    filter_bits: u8,
    fast_bit: bool,
}

enum EdgeOrPolarity {
    Edge(Edge),
    Polarity(Polarity),
}

enum Edge {
    Rising = 0b01,
    Falling = 0b10,
    Both = 0b11,
}

enum Polarity {
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

impl<const N: u8, const SRC: u8> ExternalEventBuilder<N, SRC> {
    fn edge_or_polarity(mut self, edge_or_polarity: EdgeOrPolarity) -> Self {
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
    where ExternalEventBuilder<N, SRC>: ExternalEventBuilder1To5
{
    fn fast(mut self) -> Self {
        self.fast_bit = true;
        self
    }
}

impl<const N: u8, const SRC: u8> ExternalEventBuilder<N, SRC>
    where ExternalEventBuilder<N, SRC>: ExternalEventBuilder6To10
{
    fn filter(mut self, filter: EevSamplingFilter) -> Self {
        self.filter_bits = filter as _;
        self
    }
}

trait ExternalEventBuilder1To5 {}
trait ExternalEventBuilder6To10 {}