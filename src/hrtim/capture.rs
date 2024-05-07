use super::timer;
use crate::dma::mux::DmaMuxResources;
use crate::dma::traits::TargetAddress;
use crate::dma::PeripheralToMemory;
use crate::stm32::{HRTIM_TIMA, HRTIM_TIMB, HRTIM_TIMC, HRTIM_TIMD, HRTIM_TIME, HRTIM_TIMF};
use core::marker::PhantomData;

pub struct Ch1;
pub struct Ch2;

pub struct Dma;
pub struct NoDma;

pub struct HrCapt<TIM, PSCL, CH, DMA> {
    _x: PhantomData<(TIM, PSCL, CH, DMA)>,
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

/// Trait for capture channels used for capturing edges
///
/// ```
/// let capture: HrCapt<_, _, _> = todo!();
/// if capture.is_pending() {
///     let (value, dir) = capture.get_last();
///     capture.clear_interrupt();
///     defmt::info!("Edge captured at counter value: {}, with: {}", value, dir);
/// }
/// ```
///
/// or alternatively
///
/// ```
/// let capture: HrCapt<_, _, _> = todo!();
/// if let Some((value, dir)) = capture.get() {
///     defmt::info!("Edge captured at counter value: {}, with: {}", value, dir);
/// }
/// ```
pub trait HrCapture {
    /// Try to get the capture value
    ///
    /// Returns none if edge has been captured since last time
    ///
    /// NOTE: This function will use [`Self::is_pending`] to chech if there is a value available and
    /// [`Self::clear_interrupt`] to clear it.
    fn get(&mut self) -> Option<(u16, CountingDirection)> {
        if self.is_pending() {
            let value = self.get_last();
            self.clear_interrupt();
            Some(value)
        } else {
            None
        }
    }

    /// Get number of ticks relative to beginning of upcounting
    ///
    /// where captures during down counting count as negative (before the upcount)
    ///
    /// ```txt
    ///              Counter
    /// ----------------------------------   <--- period
    /// \               ^               /
    ///    \            |            /
    ///       \         |         /
    ///          \      |      /
    /// Down count  \   |   /   Up count
    ///                \|/
    /// <-------------- 0 --------------> t
    /// Negative result | positive result
    /// ```
    ///
    /// NOTE: This function will use [`Self::is_pending`] to chech if there is a value available and
    /// [`Self::clear_interrupt`] to clear it.
    fn get_signed(&mut self, period: u16) -> Option<i32> {
        if self.is_pending() {
            let value = self.get_last_signed(period);
            self.clear_interrupt();
            Some(value)
        } else {
            None
        }
    }

    fn get_last(&self) -> (u16, CountingDirection);

    /// Get number of ticks relative to beginning of upcounting
    ///
    /// where captures during down counting count as negative (before the upcount)
    ///
    /// ````
    ///              Counter
    /// ----------------------------------   <--- period
    /// \               ^               /
    ///    \            |            /
    ///       \         |         /
    ///          \      |      /
    /// Down count  \   |   /   Up count
    ///                \|/
    /// <-------------- 0 --------------> t
    /// Negative result | positive result
    /// ````
    fn get_last_signed(&self, period: u16) -> i32 {
        let (value, dir) = self.get_last();

        // The capture counter always counts up and restarts at period
        match dir {
            CountingDirection::Up => i32::from(value),
            CountingDirection::Down => i32::from(value) - i32::from(period),
        }
    }

    fn clear_interrupt(&mut self);

    fn is_pending(&self) -> bool;
}

pub fn dma_value_to_dir_and_value(x: u32) -> (u16, CountingDirection) {
    let value = (x & 0xFFFF) as u16;
    match x & (1 << 16) != 0 {
        true => (value, CountingDirection::Down),
        false => (value, CountingDirection::Up),
    }
}

pub fn dma_value_to_signed(x: u32, period: u16) -> i32 {
    let (value, dir) = dma_value_to_dir_and_value(x);

    // The capture counter always counts up and restarts at period
    match dir {
        CountingDirection::Up => i32::from(value),
        CountingDirection::Down => i32::from(value) - i32::from(period),
    }
}

macro_rules! impl_capture {
    ($($TIMX:ident: $CH:ident, $cptXYr:ident, $cptXYcr:ident, $cptXx:ident, $dier:ident, $icr:ident, $isr:ident, $cptXie:ident, $cptXde:ident, $cptXc:ident, $cptX:ident, $mux:expr),+) => {$(
        impl<PSCL> HrCapt<$TIMX, PSCL, $CH, NoDma> {
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

            pub fn enable_dma(self, _ch: timer::DmaChannel<$TIMX>) -> HrCapt<$TIMX, PSCL, $CH, Dma> {
                // SAFETY: We own the only insance of this timers dma channel, no one else can do this
                let tim = unsafe { &*$TIMX::ptr() };
                tim.$dier.modify(|_r, w| w.$cptXde().set_bit());
                HrCapt {
                    _x: PhantomData
                }
            }
        }

        impl<PSCL, DMA> HrCapture for HrCapt<$TIMX, PSCL, $CH, DMA> {
            fn get_last(&self) -> (u16, CountingDirection) {
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
        }

        unsafe impl<PSCL> TargetAddress<PeripheralToMemory> for HrCapt<$TIMX, PSCL, $CH, Dma> {
            #[inline(always)]
            fn address(&self) -> u32 {
                let tim = unsafe { &*$TIMX::ptr() };
                &tim.$cptXYr as *const _ as u32
            }

            type MemSize = u32;

            const REQUEST_LINE: Option<u8> = Some($mux as u8);
        }
    )+};
}

impl_capture! {
    HRTIM_TIMA: Ch1, cpt1ar, cpt1acr, cpt1x, timadier, timaicr, timaisr, cpt1ie, cpt1de, cpt1c, cpt1, DmaMuxResources::HRTIM_TIMA,
    HRTIM_TIMA: Ch2, cpt2ar, cpt2acr, cpt2x, timadier, timaicr, timaisr, cpt2ie, cpt2de, cpt2c, cpt2, DmaMuxResources::HRTIM_TIMA,

    HRTIM_TIMB: Ch1, cpt1br, cpt1bcr, cpt1x, timbdier, timbicr, timbisr, cpt1ie, cpt1de, cpt1c, cpt1, DmaMuxResources::HRTIM_TIMB,
    HRTIM_TIMB: Ch2, cpt2br, cpt2bcr, cpt2x, timbdier, timbicr, timbisr, cpt2ie, cpt2de, cpt2c, cpt2, DmaMuxResources::HRTIM_TIMB,

    HRTIM_TIMC: Ch1, cpt1cr, cpt1ccr, cpt1x, timcdier, timcicr, timcisr, cpt1ie, cpt1de, cpt1c, cpt1, DmaMuxResources::HRTIM_TIMC,
    HRTIM_TIMC: Ch2, cpt2cr, cpt2ccr, cpt2x, timcdier, timcicr, timcisr, cpt2ie, cpt2de, cpt2c, cpt2, DmaMuxResources::HRTIM_TIMC,

    HRTIM_TIMD: Ch1, cpt1dr, cpt1dcr, cpt1x, timddier, timdicr, timdisr, cpt1ie, cpt1de, cpt1c, cpt1, DmaMuxResources::HRTIM_TIMD,
    HRTIM_TIMD: Ch2, cpt2dr, cpt2dcr, cpt2x, timddier, timdicr, timdisr, cpt2ie, cpt2de, cpt2c, cpt2, DmaMuxResources::HRTIM_TIMD,

    HRTIM_TIME: Ch1, cpt1er, cpt1ecr, cpt1x, timedier, timeicr, timeisr, cpt1ie, cpt1de, cpt1c, cpt1, DmaMuxResources::HRTIM_TIME,
    HRTIM_TIME: Ch2, cpt2er, cpt2ecr, cpt2x, timedier, timeicr, timeisr, cpt2ie, cpt2de, cpt2c, cpt2, DmaMuxResources::HRTIM_TIME,

    HRTIM_TIMF: Ch1, cpt1fr, cpt1fcr, cpt1x, timfdier, timficr, timfisr, cpt1ie, cpt1de, cpt1c, cpt1, DmaMuxResources::HRTIM_TIMF,
    HRTIM_TIMF: Ch2, cpt2fr, cpt2fcr, cpt2x, timfdier, timficr, timfisr, cpt2ie, cpt2de, cpt2c, cpt2, DmaMuxResources::HRTIM_TIMF
}
