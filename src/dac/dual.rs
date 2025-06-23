#![deny(missing_docs)]

//! Dual DAC
//!
//! This module provides access to the dual DAC channels of the STM32G4 series microcontrollers.
//! It allows for simultaneous access to both DAC channels and supports DMA transfers.
//!

use core::marker::PhantomData;

use embedded_hal::delay::DelayNs;

use crate::dac::{
    format::SampleFormat, trigger::DacTriggerSource, DacCh, DacChannel, Disabled, Enabled,
    Instance, M_EXT_PIN,
};
use crate::dac::{hfsel, DacExt as _, Pins};

use crate::dac::format::{self, ToDac as _};
use crate::dma::mux::DmaMuxResources;
use crate::dma::traits::TargetAddress;
use crate::dma::MemoryToPeripheral;
use crate::rcc::Rcc;

/// Extension trait for [`Instance`] to create a [`DualDac`] from the given pins and RCC.
pub trait DualDacExt: Instance + Sized {
    /// Create a dual DAC instance from the given pins and RCC.
    ///
    /// DAC calibration is performed before enabling the DAC, so
    /// a DelayNs implementation is required.
    ///
    /// This uses the dual hold registers (DHR12xD, DHR8RD) and
    /// can write to both DAC channels simultaneously with DMA support.
    fn into_dual<F: SampleFormat, PINS>(
        self,
        pins: PINS,
        rcc: &mut Rcc,
        delay: &mut impl DelayNs,
    ) -> DualDac<Self, F, Disabled>
    where
        PINS: Pins<
            Self,
            Output = (
                DacCh<Self, 0, M_EXT_PIN, Disabled>,
                DacCh<Self, 1, M_EXT_PIN, Disabled>,
            ),
        >;
}

/// Dual DAC handle
pub struct DualDac<DAC: Instance, F: SampleFormat, State> {
    _ch1: DacCh<DAC, 0, M_EXT_PIN, State>,
    _ch2: DacCh<DAC, 1, M_EXT_PIN, State>,
    _phantom: PhantomData<F>,
}

impl<DAC: Instance, F: SampleFormat> DualDac<DAC, F, Disabled> {
    /// Enable DMA double mode for the specified channel.
    /// This creates a single DMA request for every
    /// two external hardware triggers (excluding software triggers).
    #[inline(always)]
    pub fn enable_dma_double(&mut self, channel: u8, enable: bool) {
        let dac = unsafe { &(*DAC::ptr()) };
        if enable {
            dac.mcr().modify(|_, w| w.dmadouble(channel).set_bit());
        } else {
            dac.mcr().modify(|_, w| w.dmadouble(channel).clear_bit());
        }
    }

    /// Enable DMA for the specified channel
    #[inline(always)]
    pub fn enable_dma(&mut self, channel: DacChannel, enable: bool) {
        let dac = unsafe { &(*DAC::ptr()) };
        if enable {
            dac.cr().modify(|_, w| w.dmaen(channel as u8).set_bit());
        } else {
            dac.cr().modify(|_, w| w.dmaen(channel as u8).clear_bit());
        }
    }

    /// Enable trigger for the specified channel and the [`DacTriggerSource`]
    /// provided as a generic type parameter.
    ///
    /// This will cause the DAC to copy the hold register to the output register
    /// when the trigger is raised by a timer signal or external interrupt,
    /// and issue a new DMA request if DMA is enabled.
    #[inline(always)]
    pub fn enable_trigger<Source>(&mut self, channel: DacChannel)
    where
        Source: DacTriggerSource<DAC>,
    {
        // Set the TSELx bits to the trigger signal identifier from the DacTriggerSource impl
        let dac = unsafe { &(*DAC::ptr()) };
        match channel {
            DacChannel::Ch1 => {
                unsafe { dac.cr().modify(|_, w| w.tsel1().bits(Source::SIGNAL)) };
            }
            DacChannel::Ch2 => {
                unsafe { dac.cr().modify(|_, w| w.tsel2().bits(Source::SIGNAL)) };
            }
        }

        // Enable the TENx flag in DAC CR
        dac.cr().modify(|_, w| w.ten(channel as u8).set_bit());
    }

    /// Enable both DAC channels
    #[inline(always)]
    pub fn enable(self) -> DualDac<DAC, F, Enabled> {
        let dac = unsafe { &(*DAC::ptr()) };

        dac.cr().modify(|_, w| w.en(0).set_bit().en(1).set_bit());
        DualDac {
            _ch1: DacCh::new(),
            _ch2: DacCh::new(),
            _phantom: PhantomData,
        }
    }
}

impl<DAC: Instance, F: SampleFormat> DualDac<DAC, F, Enabled> {
    /// Write to both DAC channels simultaneously.
    #[inline(always)]
    pub fn set_channels(&mut self, ch1: F::Scalar, ch2: F::Scalar) {
        let dac = unsafe { &(*DAC::ptr()) };

        let bits = (ch2.to_dac() as u32) << F::CH2_SHIFT | (ch1.to_dac() as u32) & F::CH1_MASK;

        match F::DEPTH {
            format::SampleDepth::Bits12 => match F::ALIGNMENT {
                format::Alignment::Left => {
                    dac.dhr12ld().write(|w| unsafe { w.bits(bits) });
                }
                format::Alignment::Right => {
                    dac.dhr12rd().write(|w| unsafe { w.bits(bits) });
                }
            },
            format::SampleDepth::Bits8 => {
                // NOTE: 8-bit is always right aligned
                dac.dhr8rd().write(|w| unsafe { w.bits(bits) });
            }
        }
    }
}

impl<DAC: Instance> DualDacExt for DAC {
    fn into_dual<F: SampleFormat, PINS>(
        self,
        pins: PINS,
        rcc: &mut Rcc,
        delay: &mut impl DelayNs,
    ) -> DualDac<DAC, F, Disabled>
    where
        PINS: Pins<
            Self,
            Output = (
                DacCh<Self, 0, M_EXT_PIN, Disabled>,
                DacCh<Self, 1, M_EXT_PIN, Disabled>,
            ),
        >,
    {
        let (ch1, ch2) = self.constrain(pins, rcc);

        let ch1 = ch1.calibrate_buffer(delay);
        let ch2 = ch2.calibrate_buffer(delay);

        // Configure HFSEL
        let dac = unsafe { &(*DAC::ptr()) };
        let hfsel = hfsel(rcc);
        dac.mcr().modify(|_, w| w.hfsel().variant(hfsel));

        // Apply the format configuration to both channels
        for channel in 0..2 {
            match F::SIGNED {
                true => {
                    dac.mcr().modify(|_, w| w.sinformat(channel).set_bit());
                }
                false => {
                    dac.mcr().modify(|_, w| w.sinformat(channel).clear_bit());
                }
            };
        }

        DualDac {
            _ch1: ch1,
            _ch2: ch2,
            _phantom: PhantomData,
        }
    }
}

unsafe impl<DAC: Instance, F: SampleFormat, State> TargetAddress<MemoryToPeripheral>
    for DualDac<DAC, F, State>
{
    type MemSize = u32;
    const REQUEST_LINE: Option<u8> = Some(DmaMuxResources::DAC1_CH1 as u8);

    fn address(&self) -> u32 {
        let dac = unsafe { &(*DAC::ptr()) };
        match F::SIGNED {
            true => dac.dhr12ld() as *const _ as u32,
            false => dac.dhr12rd() as *const _ as u32,
        }
    }
}
