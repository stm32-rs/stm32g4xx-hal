use core::marker::PhantomData;

use crate::{
    hrtim::fault::{
        FltMonitor1, FltMonitor2, FltMonitor3, FltMonitor4, FltMonitor5, FltMonitor6, FltMonitorSys,
    },
    rcc::{Enable, Rcc, Reset},
    stm32::{HRTIM_COMMON, RCC},
};

use super::{external_event::EevInputs, fault::FaultInputs};

pub trait HrControltExt {
    fn hr_control(self, _rcc: &mut Rcc) -> HrTimOngoingCalibration;
}

impl HrControltExt for HRTIM_COMMON {
    fn hr_control(self, _rcc: &mut Rcc) -> HrTimOngoingCalibration {
        let common = unsafe { &*HRTIM_COMMON::ptr() };

        unsafe {
            let rcc_ptr = &*RCC::ptr();

            HRTIM_COMMON::enable(rcc_ptr);
            HRTIM_COMMON::reset(rcc_ptr);
        }

        // Start calibration procedure
        common
            .dllcr
            .write(|w| w.cal().set_bit().calen().clear_bit());

        HrTimOngoingCalibration {
            adc_trigger1_postscaler: AdcTriggerPostscaler::None,
            adc_trigger2_postscaler: AdcTriggerPostscaler::None,
            adc_trigger3_postscaler: AdcTriggerPostscaler::None,
            adc_trigger4_postscaler: AdcTriggerPostscaler::None,

            adc_trigger5_postscaler: AdcTriggerPostscaler::None,
            adc_trigger6_postscaler: AdcTriggerPostscaler::None,
            adc_trigger7_postscaler: AdcTriggerPostscaler::None,
            adc_trigger8_postscaler: AdcTriggerPostscaler::None,
            adc_trigger9_postscaler: AdcTriggerPostscaler::None,
            adc_trigger10_postscaler: AdcTriggerPostscaler::None,

            flt_divider: SamplingClkDiv::None,
            eev_divider: SamplingClkDiv::None,
        }
    }
}

pub struct HrTimOngoingCalibration {
    adc_trigger1_postscaler: AdcTriggerPostscaler,
    adc_trigger2_postscaler: AdcTriggerPostscaler,
    adc_trigger3_postscaler: AdcTriggerPostscaler,
    adc_trigger4_postscaler: AdcTriggerPostscaler,

    adc_trigger5_postscaler: AdcTriggerPostscaler,
    adc_trigger6_postscaler: AdcTriggerPostscaler,
    adc_trigger7_postscaler: AdcTriggerPostscaler,
    adc_trigger8_postscaler: AdcTriggerPostscaler,
    adc_trigger9_postscaler: AdcTriggerPostscaler,
    adc_trigger10_postscaler: AdcTriggerPostscaler,

    flt_divider: SamplingClkDiv,
    eev_divider: SamplingClkDiv,
}

impl HrTimOngoingCalibration {
    /// SAFETY: Calibration needs to be done before calling this
    unsafe fn init(self) {
        let common = unsafe { &*HRTIM_COMMON::ptr() };

        let Self {
            adc_trigger1_postscaler,
            adc_trigger2_postscaler,
            adc_trigger3_postscaler,
            adc_trigger4_postscaler,

            adc_trigger5_postscaler,
            adc_trigger6_postscaler,
            adc_trigger7_postscaler,
            adc_trigger8_postscaler,
            adc_trigger9_postscaler,
            adc_trigger10_postscaler,

            flt_divider,
            eev_divider,
        } = self;

        unsafe {
            // Enable periodic calibration
            // with f_hrtim at 170MHz, these settings leads to
            // a period of about 6.2ms
            common
                .dllcr
                .modify(|_r, w| w.calrte().bits(0b00).cal().set_bit().calen().clear_bit());
            common.fltinr2.write(|w| w.fltsd().bits(flt_divider as u8));
            common.eecr3.write(|w| w.eevsd().bits(eev_divider as u8));

            common.adcps1.write(|w| {
                w.adc1psc()
                    .bits(adc_trigger1_postscaler as u8)
                    .adc2psc()
                    .bits(adc_trigger2_postscaler as u8)
                    .adc3psc()
                    .bits(adc_trigger3_postscaler as u8)
                    .adc4psc()
                    .bits(adc_trigger4_postscaler as u8)
                    .adc5psc()
                    .bits(adc_trigger5_postscaler as u8)
            });

            common.adcps2.write(|w| {
                w.adc6psc()
                    .bits(adc_trigger6_postscaler as u8)
                    .adc7psc()
                    .bits(adc_trigger7_postscaler as u8)
                    .adc8psc()
                    .bits(adc_trigger8_postscaler as u8)
                    .adc9psc()
                    .bits(adc_trigger9_postscaler as u8)
                    .adc10psc()
                    .bits(adc_trigger10_postscaler as u8)
            });

            // TODO: Adc trigger 5-10
        }
    }

    pub fn wait_for_calibration(self) -> (HrTimCalibrated, FaultInputs, EevInputs) {
        let common = unsafe { &*HRTIM_COMMON::ptr() };
        while common.isr.read().dllrdy().bit_is_clear() {
            // Wait until ready
        }

        // Calibration is now done, it is safe to continue
        unsafe { self.init() };

        (
            HrTimCalibrated { _x: PhantomData },
            unsafe { FaultInputs::new() },
            unsafe { EevInputs::new() },
        )
    }

    pub fn set_adc1_trigger_psc(mut self, post_scaler: AdcTriggerPostscaler) -> Self {
        self.adc_trigger1_postscaler = post_scaler;
        self
    }

    pub fn set_adc2_trigger_psc(mut self, post_scaler: AdcTriggerPostscaler) -> Self {
        self.adc_trigger2_postscaler = post_scaler;
        self
    }

    pub fn set_adc3_trigger_psc(mut self, post_scaler: AdcTriggerPostscaler) -> Self {
        self.adc_trigger3_postscaler = post_scaler;
        self
    }

    pub fn set_adc4_trigger_psc(mut self, post_scaler: AdcTriggerPostscaler) -> Self {
        self.adc_trigger4_postscaler = post_scaler;
        self
    }

    pub fn set_fault_sampling_division(mut self, divider: SamplingClkDiv) -> Self {
        self.flt_divider = divider;
        self
    }

    pub fn set_eev_sampling_division(mut self, divider: SamplingClkDiv) -> Self {
        self.eev_divider = divider;
        self
    }
}

/// This object may be used for things that needs to be done before any timers have been started but after the calibration has been completed. Its existence is proof that no timers have started.
///
/// Once done with setup, use the `constrain` to get a `HrPwmControl` which can be used to start the timers.
pub struct HrTimCalibrated {
    _x: PhantomData<()>,
}

impl HrTimCalibrated {
    pub fn constrain(self) -> HrPwmControl {
        HrPwmControl {
            _x: PhantomData,
            fault_sys: FltMonitorSys { _x: PhantomData },
            fault_1: FltMonitor1 { _x: PhantomData },
            fault_2: FltMonitor2 { _x: PhantomData },
            fault_3: FltMonitor3 { _x: PhantomData },
            fault_4: FltMonitor4 { _x: PhantomData },
            fault_5: FltMonitor5 { _x: PhantomData },
            fault_6: FltMonitor6 { _x: PhantomData },
        }
    }
}

pub struct HrPwmControl {
    _x: PhantomData<()>,

    pub fault_sys: FltMonitorSys,
    pub fault_1: FltMonitor1,
    pub fault_2: FltMonitor2,
    pub fault_3: FltMonitor3,
    pub fault_4: FltMonitor4,
    pub fault_5: FltMonitor5,
    pub fault_6: FltMonitor6,
}

use super::adc_trigger::{Adc13Trigger, Adc24Trigger, Adc579Trigger, Adc6810Trigger};

impl HrPwmControl {
    pub fn enable_adc_trigger1_source<T: Adc13Trigger>(&mut self, _trigger: T) {
        let common = unsafe { &*HRTIM_COMMON::ptr() };
        unsafe {
            common.adc1r.modify(|r, w| w.bits(r.bits() | T::BITS));
        }
    }

    pub fn enable_adc_trigger2_source<T: Adc24Trigger>(&mut self, _trigger: T) {
        let common = unsafe { &*HRTIM_COMMON::ptr() };
        unsafe {
            common.adc2r.modify(|r, w| w.bits(r.bits() | T::BITS));
        }
    }

    pub fn enable_adc_trigger3_source<T: Adc13Trigger>(&mut self, _trigger: T) {
        let common = unsafe { &*HRTIM_COMMON::ptr() };
        unsafe {
            common.adc3r.modify(|r, w| w.bits(r.bits() | T::BITS));
        }
    }

    pub fn enable_adc_trigger4_source<T: Adc24Trigger>(&mut self, _trigger: T) {
        let common = unsafe { &*HRTIM_COMMON::ptr() };
        unsafe {
            common.adc4r.modify(|r, w| w.bits(r.bits() | T::BITS));
        }
    }

    pub fn enable_adc_trigger5_source<T: Adc579Trigger>(&mut self, _trigger: T) {
        let common = unsafe { &*HRTIM_COMMON::ptr() };
        common
            .adcer
            .modify(|_r, w| w.adc5trg().variant(T::BITS as u8));
    }

    pub fn enable_adc_trigger6_source<T: Adc6810Trigger>(&mut self, _trigger: T) {
        let common = unsafe { &*HRTIM_COMMON::ptr() };
        common
            .adcer
            .modify(|_r, w| w.adc6trg().variant(T::BITS as u8));
    }

    pub fn enable_adc_trigger7_source<T: Adc579Trigger>(&mut self, _trigger: T) {
        let common = unsafe { &*HRTIM_COMMON::ptr() };
        common
            .adcer
            .modify(|_r, w| w.adc7trg().variant(T::BITS as u8));
    }

    pub fn enable_adc_trigger8_source<T: Adc6810Trigger>(&mut self, _trigger: T) {
        let common = unsafe { &*HRTIM_COMMON::ptr() };
        common
            .adcer
            .modify(|_r, w| w.adc8trg().variant(T::BITS as u8));
    }

    pub fn enable_adc_trigger9_source<T: Adc579Trigger>(&mut self, _trigger: T) {
        let common = unsafe { &*HRTIM_COMMON::ptr() };
        common
            .adcer
            .modify(|_r, w| w.adc9trg().variant(T::BITS as u8));
    }

    pub fn enable_adc_trigger10_source<T: Adc6810Trigger>(&mut self, _trigger: T) {
        let common = unsafe { &*HRTIM_COMMON::ptr() };
        common
            .adcer
            .modify(|_r, w| w.adc10trg().variant(T::BITS as u8));
    }
}

pub enum AdcTriggerPostscaler {
    None = 0,
    Div2 = 1,
    Div3 = 2,
    Div4 = 3,
    Div5 = 4,
    Div6 = 5,
    Div7 = 6,
    Div8 = 7,
    Div9 = 8,
    Div10 = 9,
    Div11 = 10,
    Div12 = 11,
    Div13 = 12,
    Div14 = 13,
    Div15 = 14,
    Div16 = 15,
    Div17 = 16,
    Div18 = 17,
    Div19 = 18,
    Div20 = 19,
    Div21 = 20,
    Div22 = 21,
    Div23 = 22,
    Div24 = 23,
    Div25 = 24,
    Div26 = 25,
    Div27 = 26,
    Div28 = 27,
    Div29 = 28,
    Div30 = 29,
    Div31 = 30,
    Div32 = 31,
}

/// The divsion ratio between f_hrtim and the fault signal sampling clock for digital filters
pub enum SamplingClkDiv {
    /// No division
    ///
    /// fault signal sampling clock f_flts = f_hrtim
    None = 0b00,

    /// 1/2
    ///
    /// fault signal sampling clock f_flts = f_hrtim / 2
    Two = 0b01,

    /// 1/4
    ///
    /// fault signal sampling clock f_flts = f_hrtim / 4
    Four = 0b10,

    /// 1/8
    ///
    /// fault signal sampling clock f_flts = f_hrtim / 8
    Eight = 0b11,
}
