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
            adc_trigger1_bits: 0,
            adc_trigger2_bits: 0,
            adc_trigger3_bits: 0,
            adc_trigger4_bits: 0,

            adc_trigger5_bits: 0,
            adc_trigger6_bits: 0,
            adc_trigger7_bits: 0,
            adc_trigger8_bits: 0,
            adc_trigger9_bits: 0,
            adc_trigger10_bits: 0,

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
    adc_trigger1_bits: u32,
    adc_trigger2_bits: u32,
    adc_trigger3_bits: u32,
    adc_trigger4_bits: u32,

    adc_trigger5_bits: u8,
    adc_trigger6_bits: u8,
    adc_trigger7_bits: u8,
    adc_trigger8_bits: u8,
    adc_trigger9_bits: u8,
    adc_trigger10_bits: u8,

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
        use Adc13Trigger as Ad13T;
        use Adc24Trigger as Ad24T;

        let common = unsafe { &*HRTIM_COMMON::ptr() };

        let Self {
            adc_trigger1_bits: ad1_bits,
            adc_trigger2_bits: ad2_bits,
            adc_trigger3_bits: ad3_bits,
            adc_trigger4_bits: ad4_bits,

            adc_trigger5_bits: ad5_bits,
            adc_trigger6_bits: ad6_bits,
            adc_trigger7_bits: ad7_bits,
            adc_trigger8_bits: ad8_bits,
            adc_trigger9_bits: ad9_bits,
            adc_trigger10_bits: ad10_bits,

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

            common.adc1r.write(|w| {
                w.eper()
                    .bit(ad1_bits | Ad13T::TimEPeriod as u32 != 0)
                    .ec4()
                    .bit(ad1_bits | Ad13T::TimECmp4 as u32 != 0)
                    .ec3()
                    .bit(ad1_bits | Ad13T::TimECmp3 as u32 != 0)
                    //.frst()
                    .dper()
                    .bit(ad1_bits | Ad13T::TimDPeriod as u32 != 0)
                    .dc4()
                    .bit(ad1_bits | Ad13T::TimDCmp4 as u32 != 0)
                    .dc3()
                    .bit(ad1_bits | Ad13T::TimDCmp3 as u32 != 0)
                    .fper()
                    .bit(ad1_bits | Ad13T::TimFPeriod as u32 != 0)
                    .cper()
                    .bit(ad1_bits | Ad13T::TimCPeriod as u32 != 0)
                    .cc4()
                    .bit(ad1_bits | Ad13T::TimCCmp4 as u32 != 0)
                    .cc3()
                    .bit(ad1_bits | Ad13T::TimCCmp3 as u32 != 0)
                    .fc4()
                    .bit(ad1_bits | Ad13T::TimFCmp4 as u32 != 0)
                    //.brst()
                    .bper()
                    .bit(ad1_bits | Ad13T::TimBPeriod as u32 != 0)
                    .bc4()
                    .bit(ad1_bits | Ad13T::TimBCmp4 as u32 != 0)
                    .bc3()
                    .bit(ad1_bits | Ad13T::TimBCmp3 as u32 != 0)
                    .fc3()
                    .bit(ad1_bits | Ad13T::TimFCmp3 as u32 != 0)
                    //.arst()
                    .aper()
                    .bit(ad1_bits | Ad13T::TimAPeriod as u32 != 0)
                    .ac4()
                    .bit(ad1_bits | Ad13T::TimACmp4 as u32 != 0)
                    .ac3()
                    .bit(ad1_bits | Ad13T::TimACmp3 as u32 != 0)
                    .fc2()
                    .bit(ad1_bits | Ad13T::TimFCmp2 as u32 != 0)
                    //.eev5().bit(ad1_bits | Ad13T::_ as u32)
                    //.eev4().bit(ad1_bits | Ad13T::_ as u32)
                    //.eev3().bit(ad1_bits | Ad13T::_ as u32)
                    //.eev2().bit(ad1_bits | Ad13T::_ as u32)
                    //.eev1().bit(ad1_bits | Ad13T::_ as u32)
                    .mper()
                    .bit(ad1_bits | Ad13T::MasterPeriod as u32 != 0)
                    .mc4()
                    .bit(ad1_bits | Ad13T::MasterCmp4 as u32 != 0)
                    .mc3()
                    .bit(ad1_bits | Ad13T::MasterCmp3 as u32 != 0)
                    .mc2()
                    .bit(ad1_bits | Ad13T::MasterCmp2 as u32 != 0)
                    .mc1()
                    .bit(ad1_bits | Ad13T::MasterCmp1 as u32 != 0)
            });

            common.adc3r.write(|w| {
                w.eper()
                    .bit(ad3_bits | Ad13T::TimEPeriod as u32 != 0)
                    .ec4()
                    .bit(ad3_bits | Ad13T::TimECmp4 as u32 != 0)
                    .ec3()
                    .bit(ad3_bits | Ad13T::TimECmp3 as u32 != 0)
                    //.frst()
                    .dper()
                    .bit(ad3_bits | Ad13T::TimDPeriod as u32 != 0)
                    .dc4()
                    .bit(ad3_bits | Ad13T::TimDCmp4 as u32 != 0)
                    .dc3()
                    .bit(ad3_bits | Ad13T::TimDCmp3 as u32 != 0)
                    .fper()
                    .bit(ad3_bits | Ad13T::TimFPeriod as u32 != 0)
                    .cper()
                    .bit(ad3_bits | Ad13T::TimCPeriod as u32 != 0)
                    .cc4()
                    .bit(ad3_bits | Ad13T::TimCCmp4 as u32 != 0)
                    .cc3()
                    .bit(ad3_bits | Ad13T::TimCCmp3 as u32 != 0)
                    .fc4()
                    .bit(ad3_bits | Ad13T::TimFCmp4 as u32 != 0)
                    //.brst()
                    .bper()
                    .bit(ad3_bits | Ad13T::TimBPeriod as u32 != 0)
                    .bc4()
                    .bit(ad3_bits | Ad13T::TimBCmp4 as u32 != 0)
                    .bc3()
                    .bit(ad3_bits | Ad13T::TimBCmp3 as u32 != 0)
                    .fc3()
                    .bit(ad3_bits | Ad13T::TimFCmp3 as u32 != 0)
                    //.arst()
                    .aper()
                    .bit(ad3_bits | Ad13T::TimAPeriod as u32 != 0)
                    .ac4()
                    .bit(ad3_bits | Ad13T::TimACmp4 as u32 != 0)
                    .ac3()
                    .bit(ad3_bits | Ad13T::TimACmp3 as u32 != 0)
                    .fc2()
                    .bit(ad3_bits | Ad13T::TimFCmp2 as u32 != 0)
                    //.eev5().bit(ad3_bits | Ad13T::_ as u32)
                    //.eev4().bit(ad3_bits | Ad13T::_ as u32)
                    //.eev3().bit(ad3_bits | Ad13T::_ as u32)
                    //.eev2().bit(ad3_bits | Ad13T::_ as u32)
                    //.eev1().bit(ad3_bits | Ad13T::_ as u32)
                    .mper()
                    .bit(ad3_bits | Ad13T::MasterPeriod as u32 != 0)
                    .mc4()
                    .bit(ad3_bits | Ad13T::MasterCmp4 as u32 != 0)
                    .mc3()
                    .bit(ad3_bits | Ad13T::MasterCmp3 as u32 != 0)
                    .mc2()
                    .bit(ad3_bits | Ad13T::MasterCmp2 as u32 != 0)
                    .mc1()
                    .bit(ad3_bits | Ad13T::MasterCmp1 as u32 != 0)
            });

            common.adc2r.write(|w| {
                w
                    //.erst()
                    .ec4()
                    .bit(ad2_bits | Ad24T::TimECmp4 as u32 != 0)
                    .ec3()
                    .bit(ad2_bits | Ad24T::TimECmp3 as u32 != 0)
                    .ec2()
                    .bit(ad2_bits | Ad24T::TimECmp2 as u32 != 0)
                    //.drst().bit(ad2_bits | Ad24T::_ as u32 != 0)
                    .dper()
                    .bit(ad2_bits | Ad24T::TimDPeriod as u32 != 0)
                    .dc4()
                    .bit(ad2_bits | Ad24T::TimDCmp4 as u32 != 0)
                    .fper()
                    .bit(ad2_bits | Ad24T::TimFPeriod as u32 != 0)
                    .dc2()
                    .bit(ad2_bits | Ad24T::TimDCmp2 as u32 != 0)
                    //.crst().bit(ad2_bits | Ad24T::_ as u32 != 0)
                    .cper()
                    .bit(ad2_bits | Ad24T::TimCPeriod as u32 != 0)
                    .cc4()
                    .bit(ad2_bits | Ad24T::TimCCmp4 as u32 != 0)
                    .fc4()
                    .bit(ad2_bits | Ad24T::TimFCmp4 as u32 != 0)
                    .cc2()
                    .bit(ad2_bits | Ad24T::TimCCmp2 as u32 != 0)
                    .bper()
                    .bit(ad2_bits | Ad24T::TimBPeriod as u32 != 0)
                    .bc4()
                    .bit(ad2_bits | Ad24T::TimBCmp4 as u32 != 0)
                    .fc3()
                    .bit(ad2_bits | Ad24T::TimFCmp3 as u32 != 0)
                    .bc2()
                    .bit(ad2_bits | Ad24T::TimBCmp2 as u32 != 0)
                    .aper()
                    .bit(ad2_bits | Ad24T::TimAPeriod as u32 != 0)
                    .ac4()
                    .bit(ad2_bits | Ad24T::TimACmp4 as u32 != 0)
                    .fc2()
                    .bit(ad2_bits | Ad24T::TimFCmp2 as u32 != 0)
                    .ac2()
                    .bit(ad2_bits | Ad24T::TimACmp2 as u32 != 0)
                    //.eev10()
                    //.eev9()
                    //.eev8()
                    //.eev7()
                    //.eev6()
                    .mper()
                    .bit(ad2_bits | Ad24T::MasterPeriod as u32 != 0)
                    .mc4()
                    .bit(ad2_bits | Ad24T::MasterCmp4 as u32 != 0)
                    .mc3()
                    .bit(ad2_bits | Ad24T::MasterCmp3 as u32 != 0)
                    .mc2()
                    .bit(ad2_bits | Ad24T::MasterCmp2 as u32 != 0)
                    .mc1()
                    .bit(ad2_bits | Ad24T::MasterCmp1 as u32 != 0)
            });

            common.adc4r.write(|w| {
                w
                    //.erst()
                    .ec4()
                    .bit(ad4_bits | Ad24T::TimECmp4 as u32 != 0)
                    .ec3()
                    .bit(ad4_bits | Ad24T::TimECmp3 as u32 != 0)
                    .ec2()
                    .bit(ad4_bits | Ad24T::TimECmp2 as u32 != 0)
                    //.drst().bit(ad4_bits | Ad24T::_ as u32 != 0)
                    .dper()
                    .bit(ad4_bits | Ad24T::TimDPeriod as u32 != 0)
                    .dc4()
                    .bit(ad4_bits | Ad24T::TimDCmp4 as u32 != 0)
                    .fper()
                    .bit(ad4_bits | Ad24T::TimFPeriod as u32 != 0)
                    .dc2()
                    .bit(ad4_bits | Ad24T::TimDCmp2 as u32 != 0)
                    //.crst().bit(ad4_bits | Ad24T::_ as u32 != 0)
                    .cper()
                    .bit(ad4_bits | Ad24T::TimCPeriod as u32 != 0)
                    .cc4()
                    .bit(ad4_bits | Ad24T::TimCCmp4 as u32 != 0)
                    .fc4()
                    .bit(ad4_bits | Ad24T::TimFCmp4 as u32 != 0)
                    .cc2()
                    .bit(ad4_bits | Ad24T::TimCCmp2 as u32 != 0)
                    .bper()
                    .bit(ad4_bits | Ad24T::TimBPeriod as u32 != 0)
                    .bc4()
                    .bit(ad4_bits | Ad24T::TimBCmp4 as u32 != 0)
                    .fc3()
                    .bit(ad4_bits | Ad24T::TimFCmp3 as u32 != 0)
                    .bc2()
                    .bit(ad4_bits | Ad24T::TimBCmp2 as u32 != 0)
                    .aper()
                    .bit(ad4_bits | Ad24T::TimAPeriod as u32 != 0)
                    .ac4()
                    .bit(ad4_bits | Ad24T::TimACmp4 as u32 != 0)
                    .fc2()
                    .bit(ad4_bits | Ad24T::TimFCmp2 as u32 != 0)
                    .ac2()
                    .bit(ad4_bits | Ad24T::TimACmp2 as u32 != 0)
                    //.eev10()
                    //.eev9()
                    //.eev8()
                    //.eev7()
                    //.eev6()
                    .mper()
                    .bit(ad4_bits | Ad24T::MasterPeriod as u32 != 0)
                    .mc4()
                    .bit(ad4_bits | Ad24T::MasterCmp4 as u32 != 0)
                    .mc3()
                    .bit(ad4_bits | Ad24T::MasterCmp3 as u32 != 0)
                    .mc2()
                    .bit(ad4_bits | Ad24T::MasterCmp2 as u32 != 0)
                    .mc1()
                    .bit(ad4_bits | Ad24T::MasterCmp1 as u32 != 0)
            });

            common.adcer.write(|w| {
                w.adc5trg()
                    .variant(ad5_bits)
                    .adc6trg()
                    .variant(ad6_bits)
                    .adc7trg()
                    .variant(ad7_bits)
                    .adc8trg()
                    .variant(ad8_bits)
                    .adc9trg()
                    .variant(ad9_bits)
                    .adc10trg()
                    .variant(ad10_bits)
            });

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

    pub fn enable_adc_trigger1_source(mut self, trigger: Adc13Trigger) -> Self {
        self.adc_trigger1_bits |= trigger as u32;
        self
    }

    pub fn enable_adc_trigger2_source(mut self, trigger: Adc24Trigger) -> Self {
        self.adc_trigger2_bits |= trigger as u32;
        self
    }

    pub fn enable_adc_trigger3_source(mut self, trigger: Adc13Trigger) -> Self {
        self.adc_trigger3_bits |= trigger as u32;
        self
    }

    pub fn enable_adc_trigger4_source(mut self, trigger: Adc24Trigger) -> Self {
        self.adc_trigger4_bits |= trigger as u32;
        self
    }

    pub fn enable_adc_trigger5_source(mut self, trigger: Adc579Trigger) -> Self {
        self.adc_trigger5_bits = trigger as u8;
        self
    }

    pub fn enable_adc_trigger6_source(mut self, trigger: Adc6810Trigger) -> Self {
        self.adc_trigger6_bits = trigger as u8;
        self
    }

    pub fn enable_adc_trigger7_source(mut self, trigger: Adc579Trigger) -> Self {
        self.adc_trigger7_bits = trigger as u8;
        self
    }

    pub fn enable_adc_trigger8_source(mut self, trigger: Adc6810Trigger) -> Self {
        self.adc_trigger8_bits = trigger as u8;
        self
    }

    pub fn enable_adc_trigger9_source(mut self, trigger: Adc579Trigger) -> Self {
        self.adc_trigger9_bits = trigger as u8;
        self
    }

    pub fn enable_adc_trigger10_source(mut self, trigger: Adc6810Trigger) -> Self {
        self.adc_trigger10_bits = trigger as u8;
        self
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

    // TODO: Adc trigger 5-10
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

pub enum Adc13Trigger {
    /// bit 31 ADCxTEPER - Trigger on HRTIM_TIME period
    TimEPeriod = 1 << 31,

    /// bit 30 ADCxTEC4 - Trigger on HRTIM_TIME compare match for compare register 4
    TimECmp4 = 1 << 30,

    /// bit 29 ADCxTEC3 - Trigger on HRTIM_TIME compare match for compare register 3
    TimECmp3 = 1 << 29,

    /// bit 28 ADCxTFRST - Trigger on HRTIM_TIMF reset or counter roll-over
    TimFRst = 1 << 28,

    /// bit 27 ADCxTDPER - Trigger on HRTIM_TIMD period
    TimDPeriod = 1 << 27,

    /// bit 26 ADCxTDC4 - Trigger on HRTIM_TIMD compare match for compare register 4
    TimDCmp4 = 1 << 26,

    /// bit 25 ADCxTDC3 - Trigger on HRTIM_TIMD compare match for compare register 3
    TimDCmp3 = 1 << 25,

    /// bit 24 ADCxTFPER - Trigger on HRTIM_TIMF period
    TimFPeriod = 1 << 24,

    /// bit 23 ADCxTCPER - Trigger on HRTIM_TIMC period
    TimCPeriod = 1 << 23,

    /// bit 22 ADCxTCC4 - Trigger on HRTIM_TIMC compare match for compare register 4
    TimCCmp4 = 1 << 22,

    /// bit 21 ADCxTCC3 - Trigger on HRTIM_TIMC compare match for compare register 3
    TimCCmp3 = 1 << 21,

    /// bit 20 ADCxTFC4 - Trigger on HRTIM_TIMF compare match for compare register 4
    TimFCmp4 = 1 << 20,

    /// bit 19 ADCxTBRST - Trigger on HRTIM_TIMB reset or counter roll-over
    TimBRst = 1 << 19,

    /// bit 18 ADCxTBPER - Trigger on HRTIM_TIMB period
    TimBPeriod = 1 << 18,

    /// bit 17 ADCxTBC4 - Trigger on HRTIM_TIMB compare match for compare register 4
    TimBCmp4 = 1 << 17,

    /// bit 16 ADCxTBC3 - Trigger on HRTIM_TIMB compare match for compare register 3
    TimBCmp3 = 1 << 16,

    /// bit 15 ADCxTFC3 - Trigger on HRTIM_TIMF compare match for compare register 3
    TimFCmp3 = 1 << 15,

    /// bit 14 ADCxTARST - Trigger on HRTIM_TIMA reset or counter roll-over
    TimARst = 1 << 14,

    /// bit 13 ADCxTAPER - Trigger on HRTIM_TIMA period
    TimAPeriod = 1 << 13,

    /// bit 12 ADCxTAC4 - Trigger on HRTIM_TIMA compare match for compare register 4
    TimACmp4 = 1 << 12,

    /// bit 11 ADCxTAC3 - Trigger on HRTIM_TIMA compare match for compare register 3
    TimACmp3 = 1 << 11,

    /// bit 10 ADCxTFC2 - Trigger on HRTIM_TIMF compare match for compare register 2
    TimFCmp2 = 1 << 10,

    // /// bit 9 ADCxEEV5
    // _ = 1 << 9,

    // /// bit 8 ADCxEEV4
    // _ = 1 << 8,

    // /// bit 7 ADCxEEV3
    // _ = 1 << 7,

    // /// bit 6 ADCxEEV2
    // _ = 1 << 6,
    /// bit 5 ADCxEEV1
    // _ = 1 << 5,

    /// bit 4 ADCxMPER - Trigger on HRTIM_MASTER period
    MasterPeriod = 1 << 4,

    /// bit 3 ADCxMC4 - Trigger on HRTIM_MASTER compare match for compare register 4
    MasterCmp4 = 1 << 3,

    /// bit 2 ADCxMC3 - Trigger on HRTIM_MASTER compare match for compare register 3
    MasterCmp3 = 1 << 2,

    /// bit 1 ADCxMC2 - Trigger on HRTIM_MASTER compare match for compare register 2
    MasterCmp2 = 1 << 1,

    /// bit 0 ADCxMC1 - Trigger on HRTIM_MASTER compare match for compare register 1
    MasterCmp1 = 1 << 0,
}

pub enum Adc24Trigger {
    /// bit 31 ADCxTERST - Trigger on HRTIM_TIME reset or counter roll-over
    TimERst = 1 << 31,

    /// bit 30 ADCxTEC4 - Trigger on HRTIM_TIME compare match for compare register 4
    TimECmp4 = 1 << 30,

    /// bit 29 ADCxTEC3 - Trigger on HRTIM_TIME compare match for compare register 3
    TimECmp3 = 1 << 29,

    /// bit 28 ADCxTEC2 - Trigger on HRTIM_TIME compare match for compare register 2
    TimECmp2 = 1 << 28,

    /// bit 27 ADCxTDRST - Trigger on HRTIM_TIMD reset or counter roll-over
    TimDRst = 1 << 27,

    /// bit 26 ADCxTDPER - Trigger on HRTIM_TIMD period
    TimDPeriod = 1 << 26,

    /// bit 25 ADCxTDC4 - Trigger on HRTIM_TIMD compare match for compare register 4
    TimDCmp4 = 1 << 25,

    /// bit 24 ADCxTFPER - Trigger on HRTIM_TIMF period
    TimFPeriod = 1 << 24,

    /// bit 23 ADCxTDC2 - Trigger on HRTIM_TIMD compare match for compare register 2
    TimDCmp2 = 1 << 23,

    /// bit 22 ADCxTCRST - Trigger on HRTIM_TIMC reset or counter roll-over
    TimCRst = 1 << 22,

    /// bit 21 ADCxTCPER - Trigger on HRTIM_TIMC period
    TimCPeriod = 1 << 21,

    /// bit 20 ADCxTCC4 - Trigger on HRTIM_TIMC compare match for compare register 4
    TimCCmp4 = 1 << 20,

    /// bit 19 ADCxTFC4 - Trigger on HRTIM_TIMF compare match for compare register 4
    TimFCmp4 = 1 << 19,

    /// bit 18 ADCxTCC2 - Trigger on HRTIM_TIMC compare match for compare register 2
    TimCCmp2 = 1 << 18,

    /// bit 17 ADCxTBPER - Trigger on HRTIM_TIMB period
    TimBPeriod = 1 << 17,

    /// bit 16 ADCxTBC4  - Trigger on HRTIM_TIMB compare match for compare register 4
    TimBCmp4 = 1 << 16,

    /// bit 15 ADCxTFC3 - Trigger on HRTIM_TIMF compare match for compare register 3
    TimFCmp3 = 1 << 15,

    /// bit 14 ADCxTBC2 - Trigger on HRTIM_TIMB compare match for compare register 2
    TimBCmp2 = 1 << 14,

    /// bit 13 ADCxTAPER - Trigger on HRTIM_TIMA period
    TimAPeriod = 1 << 13,

    /// bit 12 ADCxTAC4 - Trigger on HRTIM_TIMA compare match for compare register 4
    TimACmp4 = 1 << 12,

    /// bit 11 ADCxTFC2 - Trigger on HRTIM_TIMF compare match for compare register 2
    TimFCmp2 = 1 << 11,

    /// bit 10 ADCxTAC2 - Trigger on HRTIM_TIMA compare match for compare register 2
    TimACmp2 = 1 << 10,

    // /// bit 9 ADCxEEV10
    // _ = 1 << 9,

    // /// bit 8 ADCxEEV9
    // _ = 1 << 8,

    // /// bit 7 ADCxEEV8
    // _ = 1 << 7,

    // /// bit 6 ADCxEEV7
    // _ = 1 << 6,

    // /// bit 5 ADCxEEV6
    // _ = 1 << 5,
    /// bit 4 ADCxMPER - Trigger on HRTIM_MASTER period
    MasterPeriod = 1 << 4,

    /// bit 3 ADCxMC4 - Trigger on HRTIM_MASTER compare match for compare register 4
    MasterCmp4 = 1 << 3,

    /// bit 2 ADCxMC3 - Trigger on HRTIM_MASTER compare match for compare register 3
    MasterCmp3 = 1 << 2,

    /// bit 1 ADCxMC2 - Trigger on HRTIM_MASTER compare match for compare register 2
    MasterCmp2 = 1 << 1,

    /// bit 0 ADCxMC1 - Trigger on HRTIM_MASTER compare match for compare register 1
    MasterCmp1 = 1 << 0,
}

pub enum Adc579Trigger {
    // /// Trigger on F reset and counter roll-over
    // _ = 31
    /// Trigger on HRTIM_TIMF period
    TimFPeriod = 30,

    /// Trigger on HRTIM_TIMF compare match for compare register 4
    TimFCmp4 = 29,

    /// Trigger on HRTIM_TIMF compare match for compare register 3
    TimFCmp3 = 28,

    /// Trigger on HRTIM_TIMF compare match for compare register 2
    TimFCmp2 = 27,

    /// Trigger on HRTIM_TIME period
    TimEPeriod = 26,

    /// Trigger on HRTIM_TIME compare match for compare register 4
    TimECmp4 = 25,

    /// Trigger on HRTIM_TIME compare match for compare register 3
    TimECmp3 = 24,

    /// Trigger on HRTIM_TIMD period
    TimDPeriod = 23,

    /// Trigger on HRTIM_TIMD compare match for compare register 4
    TimDCmp4 = 22,

    /// Trigger on HRTIM_TIMD compare match for compare register 3
    TimDCmp3 = 21,

    /// Trigger on HRTIM_TIMC period
    TimCPeriod = 20,

    /// Trigger on HRTIM_TIMC compare match for compare register 4
    TimCCmp4 = 19,

    /// Trigger on HRTIM_TIMC compare match for compare register 3
    TimCCmp3 = 18,

    // /// Trigger on B reset and counter roll-over
    // _ = 17
    /// Trigger on HRTIM_TIMB period
    TimBPeriod = 16,

    /// Trigger on HRTIM_TIMB compare match for compare register 4
    TimBCmp4 = 15,

    /// Trigger on HRTIM_TIMB compare match for compare register 3
    TimBCmp3 = 14,

    // /// Trigger on A reset and counter roll-over
    // _ = 13
    /// Trigger on HRTIM_TIMA period
    TimAPeriod = 12,

    /// Trigger on HRTIM_TIMA compare match for compare register 4
    TimACmp4 = 11,

    /// Trigger on HRTIM_TIMA compare match for compare register 3
    TimACmp3 = 10,

    // /// Trigger on EEV5
    // _ = 9,

    // ///  Trigger on EEV4
    // _ = 8,

    // ///  Trigger on EEV3
    // _ = 7,

    // ///  Trigger on EEV2
    // _ = 6,

    // ///  Trigger on EEV1
    // _ = 5,
    /// Trigger on HRTIM_MASTER period
    MasterPeriod = 4,

    /// Trigger on HRTIM_MASTER compare match for compare register 4
    MasterCmp4 = 3,

    /// Trigger on HRTIM_MASTER compare match for compare register 3
    MasterCmp3 = 2,

    /// Trigger on HRTIM_MASTER compare match for compare register 2
    MasterCmp2 = 1,

    /// Trigger on HRTIM_MASTER compare match for compare register 1
    MasterCmp1 = 0,
}

pub enum Adc6810Trigger {
    /// Trigger on HRTIM_TIMF period
    TimFPeriod = 31,

    /// Trigger on HRTIM_TIMF compare match for compare register 4
    TimFCmp4 = 30,

    /// Trigger on HRTIM_TIMF compare match for compare register 3
    TimFCmp3 = 29,

    /// Trigger on HRTIM_TIMF compare match for compare register 2
    TimFCmp2 = 28,

    // /// Trigger on E reset and counter roll-over
    // _ = 27
    /// Trigger on HRTIM_TIME compare match for compare register 4
    TimECmp4 = 26,

    /// Trigger on HRTIM_TIME compare match for compare register 3
    TimECmp3 = 25,

    /// Trigger on HRTIM_TIME compare match for compare register 2
    TimECmp2 = 24,

    // /// Trigger on D reset and counter roll-over
    // _ = 23
    /// Trigger on HRTIM_TIMD period
    TimDPeriod = 22,

    /// Trigger on HRTIM_TIMD compare match for compare register 4
    TimDCmp4 = 21,

    /// Trigger on HRTIM_TIMD compare match for compare register 2
    TimDCmp2 = 20,

    // /// Trigger on D reset and counter roll-over
    // _ = 19
    /// Trigger on HRTIM_TIMC period
    TimCPeriod = 18,

    /// Trigger on HRTIM_TIMC compare match for compare register 4
    TimCCmp4 = 17,

    /// Trigger on HRTIM_TIMC compare match for compare register 2
    TimCCmp2 = 16,

    /// Trigger on HRTIM_TIMB period
    TimBPeriod = 15,

    /// Trigger on HRTIM_TIMB compare match for compare register 4
    TimBCmp4 = 14,

    /// Trigger on HRTIM_TIMB compare match for compare register 2
    TimBCmp2 = 13,

    /// Trigger on HRTIM_TIMA period
    TimAPeriod = 12,

    /// Trigger on HRTIM_TIMA compare match for compare register 4
    TimACmp4 = 11,

    /// Trigger on HRTIM_TIMA compare match for compare register 2
    TimACmp2 = 10,

    // /// Trigger on EEV10
    // _ = 9,

    // ///  Trigger on EEV9
    // _ = 8,

    // ///  Trigger on EEV8
    // _ = 7,

    // ///  Trigger on EEV7
    // _ = 6,

    // ///  Trigger on EEV6
    // _ = 5,
    /// Trigger on HRTIM_MASTER period
    MasterPeriod = 4,

    /// Trigger on HRTIM_MASTER compare match for compare register 4
    MasterCmp4 = 3,

    /// Trigger on HRTIM_MASTER compare match for compare register 3
    MasterCmp3 = 2,

    /// Trigger on HRTIM_MASTER compare match for compare register 2
    MasterCmp2 = 1,

    /// Trigger on HRTIM_MASTER compare match for compare register 1
    MasterCmp1 = 0,
}
