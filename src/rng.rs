//! True Random Number Generator (TRNG)

use rand::TryRngCore;

use crate::{rcc::Rcc, stm32::RNG};
use core::{fmt::Formatter, marker::PhantomData};

pub enum RngError {
    NotReady,
    SeedError,
    ClockError,
}

impl core::fmt::Debug for RngError {
    fn fmt(&self, f: &mut Formatter<'_>) -> core::fmt::Result {
        match self {
            RngError::NotReady => write!(f, "RNG Not ready"),
            RngError::SeedError => write!(f, "RNG Seed error"),
            RngError::ClockError => write!(f, "RNG Clock error"),
        }
    }
}

impl core::fmt::Display for RngError {
    fn fmt(&self, f: &mut Formatter<'_>) -> core::fmt::Result {
        core::fmt::Debug::fmt(self, f)
    }
}

/// Extension trait for the RNG register block
pub trait RngExt {
    fn constrain(self, rcc: &mut Rcc) -> Rng<Stopped>;
}

impl RngExt for RNG {
    /// Constrain the RNG register block and return an Rng<Stopped>.
    /// Enables the RNG peripheral in the AHB2ENR register.
    /// Enables the HSI48 clock used by RNG
    fn constrain(self, rcc: &mut Rcc) -> Rng<Stopped> {
        // Enable RNG in AHB2ENR
        rcc.ahb2enr().modify(|_, w| w.rngen().set_bit());

        // Enable HSI48 clock used by the RNG
        rcc.enable_hsi48();

        Rng {
            _state: PhantomData,
        }
    }
}

/// Type states for the RNG peripheral
pub struct Stopped;
pub struct Running;

/// True Random Number Generator (TRNG)
pub struct Rng<State> {
    _state: core::marker::PhantomData<State>,
}

impl Rng<Stopped> {
    /// Start the RNG peripheral
    ///
    /// Enables clock error detection and starts random number generation.
    ///
    /// Retrurns an [`Rng`] in the `Running` state
    pub fn start(self) -> Rng<Running> {
        unsafe {
            (*RNG::ptr())
                .cr()
                .modify(|_, w| w.rngen().set_bit().ced().clear_bit())
        };

        Rng {
            _state: PhantomData,
        }
    }
}

impl Rng<Running> {
    /// Stop the RNG peripheral
    ///
    /// Returns an [`Rng`] in the `Stopped` state
    pub fn stop(self) -> Rng<Stopped> {
        unsafe { (*RNG::ptr()).cr().modify(|_, w| w.rngen().clear_bit()) };

        Rng {
            _state: PhantomData,
        }
    }

    /// Check if the RNG is ready
    #[inline(always)]
    pub fn is_ready(&self) -> bool {
        unsafe { (*RNG::ptr()).sr().read().drdy().bit_is_set() }
    }

    /// Check if the seed error flag is set
    pub fn is_seed_error(&self) -> bool {
        unsafe { (*RNG::ptr()).sr().read().seis().bit_is_set() }
    }

    /// Check if the clock error flag is set
    pub fn is_clock_error(&self) -> bool {
        unsafe { (*RNG::ptr()).sr().read().ceis().bit_is_set() }
    }

    /// Blocking read of a random u32 from the RNG in polling mode.
    ///
    /// Returns an [`RngError`] if the RNG reports an error condition.
    /// Polls the data ready flag until a random word is ready to be read.
    ///
    /// For non-blocking operation use [`read_non_blocking()`]
    pub fn read_blocking(&self) -> Result<u32, RngError> {
        loop {
            match self.read_non_blocking() {
                Ok(value) => return Ok(value),
                Err(RngError::NotReady) => continue,
                Err(e) => return Err(e),
            }
        }
    }

    /// Non blocking read of a random u32 from the RNG in polling mode.
    ///
    /// Returns an [`RngError`] if the RNG is not ready or reports an error condition.
    ///
    /// For blocking reads use [`read_blocking()`]
    pub fn read_non_blocking(&self) -> Result<u32, RngError> {
        // Read the SR register to check if there is an error condition,
        // and if the DRDY bit is set to indicate a valid random number is available.
        let status = unsafe { (*RNG::ptr()).sr().read() };

        // Check if the seed or clock error bits are set
        if status.seis().bit_is_set() {
            return Err(RngError::SeedError);
        }

        if status.ceis().bit_is_set() {
            return Err(RngError::ClockError);
        }

        if status.drdy().bit_is_set() {
            // Data is ready. Read the DR register and return the value.
            Ok(unsafe { (*RNG::ptr()).dr().read().bits() })
        } else {
            Err(RngError::NotReady)
        }
    }
}

/// Implement [`rand::TryRngCore`] for the RNG peripheral.
///
/// Since this is a fallible trait, to use this as a [`rand::RngCore`] with [`rand::Rng`],
/// the [`unwrap_err`] method can be used for compatibility with [`rand::Rng`] but will panic on error.
///
/// See https://docs.rs/rand/latest/rand/trait.TryRngCore.html
impl TryRngCore for &Rng<Running> {
    type Error = RngError;

    fn try_next_u32(&mut self) -> Result<u32, Self::Error> {
        self.read_blocking()
    }

    fn try_next_u64(&mut self) -> Result<u64, Self::Error> {
        let mut result = 0u64;
        for _ in 0..2 {
            result |= self.try_next_u32()? as u64;
            result <<= 32;
        }
        Ok(result)
    }

    fn try_fill_bytes(&mut self, dst: &mut [u8]) -> Result<(), Self::Error> {
        for chunk in dst.chunks_mut(size_of::<u32>()) {
            let value = self.try_next_u32()?;
            chunk.copy_from_slice(&value.to_ne_bytes());
        }
        Ok(())
    }
}
