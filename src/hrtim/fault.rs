use core::marker::PhantomData;

use crate::comparator::{COMP1, COMP2, COMP3, COMP4, COMP5, COMP6};
use crate::gpio::gpioa::{PA12, PA15};
use crate::gpio::gpiob::{PB0, PB10, PB11};
use crate::gpio::gpioc::{PC10, PC7};
use crate::gpio::{self, AF13, AF3};
use crate::hrtim::control::HrPwmControl;
use crate::stm32::HRTIM_COMMON;

use super::control::HrPwmCtrl;

/// Allows a FaultMonitor to monitor faults
pub trait FaultMonitor {
    fn enable_interrupt(&mut self, hr_control: &mut HrPwmCtrl);

    /// Returns true if a fault is preventing PWM output
    fn is_fault_active(&self) -> bool;

    /// Clear the fault interrupt flag
    ///
    /// This will *NOT* resume normal PWM operation. The affected outputs need to be re-enabled to resume operation;
    /// This will do nothing if the fault is still active.
    fn clear_fault(&mut self);
}

pub enum FaultAction {
    /// Output never enters fault mode
    None = 0b00,

    /// Output forced to `active` level on fault
    ForceActive = 0b01,

    /// Output forced to `inactive` level on fault
    ForceInactive = 0b10,

    /// The output is floating/tri stated on fault
    Floating = 0b11,
}

/// # Safety
/// Only implement for actual fault sources with correct `ENABLE_BITS`
pub unsafe trait FaultSource: Copy {
    const ENABLE_BITS: u8;
}

pub struct SourceBuilder<I> {
    _input: I,
    src_bits: u8,

    /// FLTxP
    is_active_high: bool,

    /// FLTxF[3:0]
    filter_bits: u8,
}

impl<I> SourceBuilder<I> {
    unsafe fn new(input: I, src_bits: u8) -> Self {
        SourceBuilder {
            _input: input,
            src_bits,
            is_active_high: false,
            filter_bits: 0b0000,
        }
    }
}

macro_rules! impl_faults {
    ($(
        $input:ident => $source:ident:
            PINS=[($pin:ident, $af:ident) $(,($pin_b:ident, $af_b:ident))*],
            COMP=$compX:ident, $enable_bits:literal,
            $fltinrZ:ident, $fltWsrc_0:ident, $fltWsrc_1:ident, $fltWp:ident, $fltWf:ident, $fltWe:ident, $fltWlck:ident,
    )+) => {$(

        // This should NOT be Copy/Clone
        pub struct $input {
            pub(crate) _x: PhantomData<()>
        }

        #[derive(Copy, Clone)]
        pub struct $source {
            _x: PhantomData<()>
        }

        impl $input {
            pub fn bind_pin<IM>(self, pin: $pin<gpio::Input<IM>>) -> SourceBuilder<$input> {
                pin.into_alternate::<$af>();
                unsafe { SourceBuilder::new(self, 0b00) }
            }

            $(
                // TODO: Is there a nicer way to do this?
                pub fn bind_pin_b<IM>(self, pin: $pin_b<gpio::Input<IM>>) -> SourceBuilder<$input> {
                    pin.into_alternate::<$af_b>();
                    unsafe { SourceBuilder::new(self, 0b00) }
                }
            )*

            pub fn bind_comp(self, _comp: &crate::comparator::Comparator<$compX, crate::comparator::Enabled>) -> SourceBuilder<$input> {
                unsafe { SourceBuilder::new(self, 0b01) }
            }

            /*pub fn bind_external(?) {
                SourceBuilder::new(self, 0b10);
            }*/
        }

        impl SourceBuilder<$input> {
            pub fn finalize(self, _control: &mut HrPwmControl) -> $source {
                let SourceBuilder{ _input, src_bits, is_active_high, filter_bits } = self;

                // Setup fault source
                unsafe {
                    let common = &*HRTIM_COMMON::ptr();

                    common.fltinr2().modify(|_r, w| w.$fltWsrc_1().bit(src_bits & 0b10 != 0));
                    common.$fltinrZ().modify(|_r, w| w
                        .$fltWsrc_0().bit(src_bits & 0b01 != 0)
                        .$fltWp().bit(is_active_high)
                        .$fltWf().bits(filter_bits)
                        .$fltWe().set_bit() // Enable
                    );

                    // ... and lock configuration
                    common.$fltinrZ().modify(|_r, w| w.$fltWlck().set_bit());
                }

                $source {
                    _x: PhantomData
                }
            }

            pub fn polarity(mut self, polarity: super::Polarity) -> Self {
                self.is_active_high = polarity == super::Polarity::ActiveHigh;
                self
            }

            // TODO: add more settings
            /* pub fn blanking(?) -> Self */

            pub fn filter(mut self, filter: FaultSamplingFilter) -> Self {
                self.filter_bits = filter as u8;
                self
            }
        }

        unsafe impl FaultSource for $source {
            const ENABLE_BITS: u8 = $enable_bits;
        }
    )+}
}

impl_faults!(
    FaultInput1 => FaultSource1: PINS=[(PA12, AF13)], COMP=COMP2, 0b000001, fltinr1, flt1src, flt1src_1, flt1p, flt1f, flt1e, flt1lck,
    FaultInput2 => FaultSource2: PINS=[(PA15, AF13)], COMP=COMP4, 0b000010, fltinr1, flt2src, flt2src_1, flt2p, flt2f, flt2e, flt2lck,
    FaultInput3 => FaultSource3: PINS=[(PB10, AF13)], COMP=COMP6, 0b000100, fltinr1, flt3src, flt3src_1, flt3p, flt3f, flt3e, flt3lck,
    FaultInput4 => FaultSource4: PINS=[(PB11, AF13)], COMP=COMP1, 0b001000, fltinr1, flt4src, flt4src_1, flt4p, flt4f, flt4e, flt4lck,
    FaultInput5 => FaultSource5: PINS=[(PB0, AF13), (PC7, AF3)], COMP=COMP3, 0b010000, fltinr2, flt5src, flt5src_1, flt5p, flt5f, flt5e, flt5lck,
    FaultInput6 => FaultSource6: PINS=[(PC10, AF13)], COMP=COMP5, 0b100000, fltinr2, flt6src_0, flt6src_1, flt6p, flt6f, flt6e, flt6lck,
);

pub struct FaultInputs {
    pub fault_input1: FaultInput1,
    pub fault_input2: FaultInput2,
    pub fault_input3: FaultInput3,
    pub fault_input4: FaultInput4,
    pub fault_input5: FaultInput5,
    pub fault_input6: FaultInput6,
}

impl FaultInputs {
    pub(crate) unsafe fn new() -> Self {
        FaultInputs {
            fault_input1: FaultInput1 { _x: PhantomData },
            fault_input2: FaultInput2 { _x: PhantomData },
            fault_input3: FaultInput3 { _x: PhantomData },
            fault_input4: FaultInput4 { _x: PhantomData },
            fault_input5: FaultInput5 { _x: PhantomData },
            fault_input6: FaultInput6 { _x: PhantomData },
        }
    }
}

pub enum FaultSamplingFilter {
    /// No filtering, fault acts asynchronously
    ///
    /// Note that this bypasses any f_flts (SamplingClkDiv)
    None = 0b0000,

    /// Sample directly at rate f_hrtim, with a count of 2
    ///
    /// Note that this bypasses: any f_flts (SamplingClkDiv)
    HrtimN2 = 0b0001,

    /// Sample directly at rate f_hrtim, with a count of 4
    ///
    /// Note that this bypasses any f_flts (SamplingClkDiv)
    HrtimN4 = 0b0010,

    /// Sample directly at rate f_hrtim, with a count of 8
    ///
    /// Note that this bypasses any f_flts (SamplingClkDiv)
    HrtimN8 = 0b0011,

    /// Sample at rate f_flts / 2, with a count of 6
    FltsDiv2N6 = 0b0100,

    /// Sample at rate f_flts / 2, with a count of 8
    FltsDiv2N8 = 0b0101,

    /// Sample at rate f_flts / 4, with a count of 6
    FltsDiv4N6 = 0b0110,

    /// Sample at rate f_flts / 4, with a count of 8
    FltsDiv4N8 = 0b0111,

    /// Sample at rate f_flts / 8, with a count of 6
    FltsDiv8N6 = 0b1000,

    /// Sample at rate f_flts / 8, with a count of 8
    FltsDiv8N8 = 0b1001,

    /// Sample at rate f_flts / 16, with a count of 5
    FltsDiv16N5 = 0b1010,

    /// Sample at rate f_flts / 16, with a count of 6
    FltsDiv16N6 = 0b1011,

    /// Sample at rate f_flts / 16, with a count of 8
    FltsDiv16N8 = 0b1100,

    /// Sample at rate f_flts / 32, with a count of 5
    FltsDiv32N5 = 0b1101,

    /// Sample at rate f_flts / 32, with a count of 6
    FltsDiv32N6 = 0b1110,

    /// Sample at rate f_flts / 32, with a count of 8
    FltsDiv32N8 = 0b1111,
}

macro_rules! impl_flt_monitor {
    ($($t:ident: ($fltx:ident, $fltxc:ident, $fltxie:ident),)+) => {$(
        pub struct $t {
            pub(crate) _x: PhantomData<()>
        }

        impl FaultMonitor for $t {
            fn enable_interrupt(&mut self, _hr_control: &mut HrPwmCtrl) {
                let common = unsafe { &*HRTIM_COMMON::ptr() };
                common.ier().modify(|_r, w| w.$fltxie().set_bit());
            }

            fn is_fault_active(&self) -> bool {
                let common = unsafe { &*HRTIM_COMMON::ptr() };
                common.isr().read().$fltx().bit()
            }

            fn clear_fault(&mut self) {
                let common = unsafe { &*HRTIM_COMMON::ptr() };
                common.icr().write(|w| w.$fltxc().set_bit());
            }
        }
    )+};
}

impl_flt_monitor!(
    FltMonitorSys: (sysflt, sysfltc, sysfltie),
    FltMonitor1: (flt1, flt1c, flt1ie),
    FltMonitor2: (flt2, flt2c, flt2ie),
    FltMonitor3: (flt3, flt3c, flt3ie),
    FltMonitor4: (flt4, flt4c, flt4ie),
    FltMonitor5: (flt5, flt5c, flt5ie),
    FltMonitor6: (flt6, flt6c, flt6ie),
);
