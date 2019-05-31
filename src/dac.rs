//! DAC
use core::mem;

use crate::delay::Delay;
use crate::gpio::gpioa::{PA4, PA5};
use crate::gpio::DefaultMode;
use crate::prelude::*;
use crate::rcc::Rcc;
use crate::stm32::DAC;

pub struct C1;
pub struct C2;

pub trait DacOut<V> {
    fn set_value(&mut self, val: V);
    fn get_value(&mut self) -> V;
}

pub trait DacPin {
    fn enable(&mut self);
    fn calibrate(&mut self, delay: &mut Delay);
}

pub trait Pins<DAC> {
    type Output;
}

impl Pins<DAC> for PA4<DefaultMode> {
    type Output = C1;
}

impl Pins<DAC> for PA5<DefaultMode> {
    type Output = C2;
}

impl Pins<DAC> for (PA4<DefaultMode>, PA5<DefaultMode>) {
    type Output = (C1, C2);
}

pub fn dac<PINS>(_dac: DAC, _pins: PINS, rcc: &mut Rcc) -> PINS::Output
where
    PINS: Pins<DAC>,
{
    // Enable DAC clocks
    rcc.rb.apbenr1.modify(|_, w| w.dac1en().set_bit());

    // Reset DAC
    rcc.rb.apbrstr1.modify(|_, w| w.dac1rst().set_bit());
    rcc.rb.apbrstr1.modify(|_, w| w.dac1rst().clear_bit());

    unsafe { mem::uninitialized() }
}

macro_rules! dac {
    ($CX:ident, $en:ident, $cen:ident, $cal_flag:ident, $trim:ident, $mode:ident, $dhrx:ident, $dac_dor:ident, $daccxdhr:ident) => {
        impl DacPin for $CX {
            fn enable(&mut self) {
                let dac = unsafe { &(*DAC::ptr()) };
                dac.dac_cr.modify(|_, w| w.$en().set_bit());
            }

            fn calibrate(&mut self, delay: &mut Delay) {
                let dac = unsafe { &(*DAC::ptr()) };
                dac.dac_cr.modify(|_, w| w.$en().clear_bit());
                dac.dac_mcr.modify(|_, w| unsafe { w.$mode().bits(0) });
                dac.dac_cr.modify(|_, w| w.$cen().set_bit());
                let mut trim = 0;
                while true {
                    dac.dac_ccr.modify(|_, w| unsafe { w.$trim().bits(trim) });
                    delay.delay_us(64_u32);
                    if dac.dac_sr.read().$cal_flag().bit() {
                        break;
                    }
                    trim += 1;
                }
                dac.dac_cr.modify(|_, w| w.$cen().clear_bit());
            }
        }

        impl DacOut<u16> for $CX {
            fn set_value(&mut self, val: u16) {
                let dac = unsafe { &(*DAC::ptr()) };
                dac.$dhrx.write(|w| unsafe { w.bits(val as u32) });
            }

            fn get_value(&mut self) -> u16 {
                let dac = unsafe { &(*DAC::ptr()) };
                dac.$dac_dor.read().bits() as u16
            }
        }
    };
}

pub trait DacExt {
    fn constrain<PINS>(self, pins: PINS, rcc: &mut Rcc) -> PINS::Output
    where
        PINS: Pins<DAC>;
}

impl DacExt for DAC {
    fn constrain<PINS>(self, pins: PINS, rcc: &mut Rcc) -> PINS::Output
    where
        PINS: Pins<DAC>,
    {
        dac(self, pins, rcc)
    }
}

dac!(
    C1,
    en1,
    cen1,
    cal_flag1,
    otrim1,
    mode1,
    dac_dhr12r1,
    dac_dor1,
    dacc1dhr
);

dac!(
    C2,
    en2,
    cen2,
    cal_flag2,
    otrim2,
    mode2,
    dac_dhr12r2,
    dac_dor2,
    dacc2dhr
);
