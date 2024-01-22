use core::marker::PhantomData;

use stm32g4::stm32g474::{HRTIM_TIMA, HRTIM_TIMB, HRTIM_TIMC, HRTIM_TIMD, HRTIM_TIME, HRTIM_TIMF};

pub struct Ch1;
pub struct Ch2;

pub struct HrCapt<TIM, PSCL, CH> {
    _x: PhantomData<(TIM, PSCL, CH)>,
}

#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Copy, Clone, Debug)]
pub enum CountingDirection {
    Up = 0,
    Down = 1,
}

/// Implemented for
/// * TIM's update event
/// * EEVT1-10
/// TODO: This sould be implemeted
/// * All neighbor timers CMP1, CPM2, OUT1_RST and OUT1_SET events
pub trait CaptureEvent<TIM, PSCL> {
    const BITS: u32;
}

pub trait HrCapture {
    fn get(&self) -> (u16, CountingDirection);

    /// Get number of ticks relative to beginning of upcounting
    ///
    /// where captures during down counting count as negative (before the upcount)
    fn get_signed(&self) -> i32 {
        let (value, dir) = self.get();
        let dir_bit = dir as i32;
        dir_bit << 31 | i32::from(value)
    }

    /// Get number of ticks relative to beginning of upcounting
    ///
    /// where captures during down counting count as larger (after upcount)
    fn get_unsigned(&self) -> u32 {
        let (value, dir) = self.get();
        let dir_bit = dir as u32;
        dir_bit << 16 | u32::from(value)
    }

    fn clear_interrupt(&mut self);

    fn is_pending(&self) -> bool;
}

macro_rules! impl_capture {
    ($($TIMX:ident: $CH:ident, $cptXYr:ident, $cptXYcr:ident, $cptXx:ident, $dier:ident, $icr:ident, $isr:ident, $cptXie:ident, $cptXc:ident, $cptX:ident),+) => {
        $(impl<PSCL> HrCapt<$TIMX, PSCL, $CH> {
            /// Add event to capture
            ///
            /// If multiple events are added, they will be ORed together meaning
            /// that a capture will be trigger if any one of the events triggers
            pub fn add_event<E: CaptureEvent<$TIMX, PSCL>>(&mut self, _event: &E) {
                let tim = unsafe { &*$TIMX::ptr() };

                // SAFETY: We are the only one with access to cptXYcr
                unsafe {
                    tim.$cptXYcr.modify(|r, w| w.bits(r.bits() | E::BITS));
                }
            }

            /// Remove event to capture
            pub fn remove_event<E: CaptureEvent<$TIMX, PSCL>>(&mut self, _event: &E) {
                let tim = unsafe { &*$TIMX::ptr() };

                // SAFETY: We are the only one with access to cptXYcr
                unsafe {
                    tim.$cptXYcr.modify(|r, w| w.bits(r.bits() & !E::BITS));
                }
            }

            /// Force capture trigger now
            pub fn trigger_now(&mut self) {
                // SAFETY: We are the only one with access to cptXYcr
                let tim = unsafe { &*$TIMX::ptr() };

                tim.$cptXYcr.modify(|_, w| w.swcpt().set_bit());
            }

            // TODO: It would be sufficient to instead of hr_control only require exclusive access to the owning timer
            // however that would be hard to do since typically the capture device is a field of that same timer.
            // Would it make more sense to have this method direcly on HrTim instead?
            pub fn enable_interrupt(&mut self, enable: bool, _hr_control: &mut super::HrPwmControl) {
                let tim = unsafe { &*$TIMX::ptr() };

                tim.$dier.modify(|_r, w| w.$cptXie().bit(enable));
            }
        }

        impl<PSCL> HrCapture for HrCapt<$TIMX, PSCL, $CH> {
            fn get(&self) -> (u16, CountingDirection) {
                let tim = unsafe { &*$TIMX::ptr() };
                let data = tim.$cptXYr.read();
                let dir = match data.dir().bit() {
                    true => CountingDirection::Down,
                    false => CountingDirection::Up,
                };
                let value = data.$cptXx().bits();

                (value, dir)
            }

            fn clear_interrupt(&mut self) {
                let tim = unsafe { &*$TIMX::ptr() };

                // No need for exclusive access since this is a write only register
                tim.$icr.write(|w| w.$cptXc().set_bit());
            }

            fn is_pending(&self) -> bool {
                let tim = unsafe { &*$TIMX::ptr() };

                // No need for exclusive access since this is a read only register
                tim.$isr.read().$cptX().bit()
            }
        })+
    };
}

impl_capture! {
    HRTIM_TIMA: Ch1, cpt1ar, cpt1acr, cpt1x, timadier, timaicr, timaisr, cpt1ie, cpt1c, cpt1,
    HRTIM_TIMA: Ch2, cpt2ar, cpt2acr, cpt2x, timadier, timaicr, timaisr, cpt2ie, cpt2c, cpt2,

    HRTIM_TIMB: Ch1, cpt1br, cpt1bcr, cpt1x, timbdier, timbicr, timbisr, cpt1ie, cpt1c, cpt1,
    HRTIM_TIMB: Ch2, cpt2br, cpt2bcr, cpt2x, timbdier, timbicr, timbisr, cpt2ie, cpt2c, cpt2,

    HRTIM_TIMC: Ch1, cpt1cr, cpt1ccr, cpt1x, timcdier, timcicr, timcisr, cpt1ie, cpt1c, cpt1,
    HRTIM_TIMC: Ch2, cpt2cr, cpt2ccr, cpt2x, timcdier, timcicr, timcisr, cpt2ie, cpt2c, cpt2,

    HRTIM_TIMD: Ch1, cpt1dr, cpt1dcr, cpt1x, timddier, timdicr, timdisr, cpt1ie, cpt1c, cpt1,
    HRTIM_TIMD: Ch2, cpt2dr, cpt2dcr, cpt2x, timddier, timdicr, timdisr, cpt2ie, cpt2c, cpt2,

    HRTIM_TIME: Ch1, cpt1er, cpt1ecr, cpt1x, timedier, timeicr, timeisr, cpt1ie, cpt1c, cpt1,
    HRTIM_TIME: Ch2, cpt2er, cpt2ecr, cpt2x, timedier, timeicr, timeisr, cpt2ie, cpt2c, cpt2,

    HRTIM_TIMF: Ch1, cpt1fr, cpt1fcr, cpt1x, timfdier, timficr, timfisr, cpt1ie, cpt1c, cpt1,
    HRTIM_TIMF: Ch2, cpt2fr, cpt2fcr, cpt2x, timfdier, timficr, timfisr, cpt2ie, cpt2c, cpt2
}
