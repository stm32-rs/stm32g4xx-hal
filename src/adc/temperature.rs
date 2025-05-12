use crate::signature::{VtempCal130, VtempCal30, VDDA_CALIB};

use super::config;

/*
    Currently unused but this is the formula for using temperature calibration:
    Temperature in °C = ( ( (TS_CAL2_TEMP-TS_CAL1_TEMP) / (TS_CAL2-TS_CAL1) ) * (TS_DATA-TS_CAL1) ) + 30°C
*/

/// Core temperature internal signal
pub struct Temperature;
impl crate::stasis::Freeze for Temperature {}

impl Temperature {
    /// Precompute the inverse of `VTEMP_CAL_VREFANALOG`, in volts,
    /// for floating point calculations
    const INV_VREFANALOG_VOLTS: f32 = 1000. / VDDA_CALIB as f32;
    /// Temperature at which temperature sensor has been calibrated in production
    /// for data into [`VtempCal30`] (tolerance: +-5 DegC) (unit: DegC).
    const VTEMP_CAL_T30: u16 = 30;
    /// Temperature at which temperature sensor has been calibrated in production
    /// for data into [`VtempCal130`] (tolerance: +-5 DegC) (unit: DegC).
    const VTEMP_CAL_T130: u16 = 130;

    /// Convert a sample to 12 bits. Reference voltages were captured at 12 bits.
    const fn to_12b(sample: u16, resolution: config::Resolution) -> u16 {
        match resolution {
            config::Resolution::Six => sample << 6,
            config::Resolution::Eight => sample << 4,
            config::Resolution::Ten => sample << 2,
            config::Resolution::Twelve => sample,
        }
    }

    /// Convert a raw sample from `Temperature` to deg C.
    ///
    /// ## Arguments
    /// * `sample`: ADC sample taken on the [`Temperature`] channel.
    /// * `vdda`: Analog reference voltage (vref+) when the temperature
    ///   sample was taken, in volts.
    /// * `resolution`: Configured ADC resolution.
    #[inline(always)]
    pub fn temperature_to_degrees_centigrade(
        sample: u16,
        vdda: f32,
        resolution: config::Resolution,
    ) -> f32 {
        // Reference measurements were taken at 12 bits
        let sample_12b = Self::to_12b(sample, resolution);

        // Normalize for the difference in VDDA
        let sample_normalized = sample_12b as f32 * (vdda * Self::INV_VREFANALOG_VOLTS);

        ((sample_normalized - VtempCal30::get().read() as f32)
            * ((Self::VTEMP_CAL_T130 - Self::VTEMP_CAL_T30) as f32))
            / ((VtempCal130::get().read() - VtempCal30::get().read()) as f32)
            + Self::VTEMP_CAL_T30 as f32
    }

    /// Convert a raw sample from `Temperature` to deg C
    ///
    /// ## Arguments
    /// * `sample`: ADC sample taken on the [`Temperature`] channel.
    /// * `vdda`: Analog reference voltage (vref+) when the temperature
    ///   sample was taken, in millivolts.
    /// * `resolution`: Configured ADC resolution.
    #[inline(always)]
    pub fn temperature_to_degrees_centigrade_coarse(
        sample: u16,
        vdda: u32,
        resolution: config::Resolution,
    ) -> i16 {
        // Reference measurements were taken at 12 bits
        let sample_12b = Self::to_12b(sample, resolution);

        // Normalize for the difference in VDDA
        let sample_normalized = ((sample_12b as u32 * vdda) / VDDA_CALIB) as u16;

        let t = ((sample_normalized as i32 - VtempCal30::get().read() as i32)
            * ((Self::VTEMP_CAL_T130 - Self::VTEMP_CAL_T30) as i32))
            / ((VtempCal130::get().read() - VtempCal30::get().read()) as i32)
            + Self::VTEMP_CAL_T30 as i32;

        t as i16
    }
}
