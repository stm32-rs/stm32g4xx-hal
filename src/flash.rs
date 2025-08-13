//! Flash memory

use crate::stm32::{flash, FLASH};

use core::convert::TryInto;

pub const FLASH_START: u32 = 0x0800_0000;
pub const FLASH_END: u32 = 0x080F_FFFF;

const _RDPRT_KEY: u16 = 0x00A5;
const KEY1: u32 = 0x45670123;
const KEY2: u32 = 0xCDEF89AB;

const OPT_KEY1: u32 = 0x08192A3B;
const OPT_KEY2: u32 = 0x4C5D6E7F;

pub const SZ_1K: u32 = 1024;

pub type Result<T> = core::result::Result<T, Error>;

#[derive(Debug, Clone, Copy, PartialEq, Eq, Ord, PartialOrd)]
pub enum Error {
    AddressLargerThanFlash,
    AddressMisaligned,
    LengthNotMultiple2,
    LengthTooLong,
    EraseError,
    ProgrammingError,
    WriteError,
    VerifyError,
    UnlockError,
    OptUnlockError,
    LockError,
    OptLockError,
    ArrayMustBeDivisibleBy8,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Ord, PartialOrd)]
pub enum FlashSize {
    Sz16K = 16,
    Sz32K = 32,
    Sz64K = 64,
    Sz128K = 128,
    Sz256K = 256,
    Sz384K = 384,
    Sz512K = 512,
    Sz768K = 768,
    Sz1M = 1024,
}
impl FlashSize {
    const fn kbytes(self) -> u32 {
        SZ_1K * self as u32
    }
}

pub struct FlashWriter<'a, const SECTOR_SZ_KB: u32> {
    flash: &'a mut Parts,
    flash_sz: FlashSize,
    verify: bool,
}
impl<const SECTOR_SZ_KB: u32> FlashWriter<'_, SECTOR_SZ_KB> {
    #[allow(unused)]
    fn unlock_options(&mut self) -> Result<()> {
        // Check if flash is busy
        while self.flash.sr.sr().read().bsy().bit_is_set() {}

        // Unlock
        self.unlock()?;

        // Write the OPT Keys to the OPTKEYR register
        unsafe {
            self.flash
                ._optkeyr
                .optkeyr()
                .write(|w| w.optkeyr().bits(OPT_KEY1));
        }
        unsafe {
            self.flash
                ._optkeyr
                .optkeyr()
                .write(|w| w.optkeyr().bits(OPT_KEY2));
        }

        // Verify success
        match self.flash.cr.cr().read().lock().bit_is_clear() {
            true => Ok(()),
            false => Err(Error::OptUnlockError),
        }
    }

    fn unlock(&mut self) -> Result<()> {
        // Wait for any ongoing operations
        while self.flash.sr.sr().read().bsy().bit_is_set() {}

        // NOTE(unsafe) write Keys to the key register. This is safe because the
        // only side effect of these writes is to unlock the flash control
        // register, which is the intent of this function. Do not rearrange the
        // order of these writes or the control register will be permanently
        // locked out until reset.
        unsafe {
            self.flash.keyr.keyr().write(|w| w.keyr().bits(KEY1));
        }
        unsafe {
            self.flash.keyr.keyr().write(|w| w.keyr().bits(KEY2));
        }

        // Verify success
        match self.flash.cr.cr().read().lock().bit_is_clear() {
            true => Ok(()),
            false => Err(Error::UnlockError),
        }
    }

    fn lock(&mut self) -> Result<()> {
        //Wait for ongoing flash operations
        while self.flash.sr.sr().read().bsy().bit_is_set() {}

        // Set lock bit
        self.flash.cr.cr().modify(|_, w| w.lock().set_bit());

        // Verify success
        match self.flash.cr.cr().read().lock().bit_is_set() {
            true => Ok(()),
            false => Err(Error::LockError),
        }
    }

    fn valid_address(&self, offset: u32) -> Result<()> {
        if FLASH_START
            .checked_add(offset)
            .ok_or(Error::AddressLargerThanFlash)?
            > FLASH_END
        {
            Err(Error::AddressLargerThanFlash)
        } else if offset & 0x1 != 0 {
            Err(Error::AddressMisaligned)
        } else {
            Ok(())
        }
    }

    fn valid_length(&self, offset: u32, length: usize, force_padding: bool) -> Result<()> {
        if offset
            .checked_add(length as u32)
            .ok_or(Error::LengthTooLong)?
            > self.flash_sz.kbytes()
        {
            return Err(Error::LengthTooLong);
        }

        if !force_padding && length % 8 != 0 {
            return Err(Error::ArrayMustBeDivisibleBy8);
        }

        Ok(())
    }

    /// Erase sector which contains `start_offset`
    pub fn page_erase(&mut self, start_offset: u32) -> Result<()> {
        self.valid_address(start_offset)?;

        // Unlock Flash
        self.unlock()?;

        // Set Page Erase
        self.flash.cr.cr().modify(|_, w| w.per().set_bit());

        let page = start_offset / SECTOR_SZ_KB;

        // Write address bits
        // NOTE(unsafe) This sets the page address in the Address Register.
        // The side-effect of this write is that the page will be erased when we
        // set the STRT bit in the CR below. The address is validated by the
        // call to self.valid_address() above.
        unsafe {
            self.flash
                .cr
                .cr()
                .modify(|_, w| w.pnb().bits(page.try_into().unwrap()));
        }

        // Start Operation
        self.flash.cr.cr().modify(|_, w| w.strt().set_bit());

        // Wait for operation to finish
        while self.flash.sr.sr().read().bsy().bit_is_set() {}

        // Check for errors
        let sr = self.flash.sr.sr().read();

        // Remove Page Erase Operation bit
        self.flash.cr.cr().modify(|_, w| w.per().clear_bit());

        // Re-lock flash
        self.lock()?;

        if sr.wrperr().bit_is_set() {
            self.flash.sr.sr().modify(|_, w| w.wrperr().set_bit());
            Err(Error::EraseError)
        } else {
            if self.verify {
                // By subtracting 1 from the sector size and masking with
                // start_offset, we make 'start' point to the beginning of the
                // page. We do this because the entire page should have been
                // erased, regardless of where in the page the given
                // 'start_offset' was.
                let size = SECTOR_SZ_KB;
                let start = start_offset & !(size - 1);
                for idx in (start..start + size).step_by(2) {
                    let write_address = (FLASH_START + idx) as *const u16;
                    let verify: u16 = unsafe { core::ptr::read_volatile(write_address) };
                    if verify != 0xFFFF {
                        return Err(Error::VerifyError);
                    }
                }
            }

            Ok(())
        }
    }

    /// Erase the Flash Sectors from `FLASH_START + start_offset` to `length`
    pub fn erase(&mut self, start_offset: u32, length: usize) -> Result<()> {
        self.valid_length(start_offset, length, true)?;

        // Erase every sector touched by start_offset + length
        for offset in
            (start_offset..start_offset + length as u32).step_by(SECTOR_SZ_KB.try_into().unwrap())
        {
            self.page_erase(offset)?;
        }

        // Report Success
        Ok(())
    }

    /// Retrieve a slice of data from `FLASH_START + offset`
    pub fn read(&self, offset: u32, length: usize) -> Result<&[u8]> {
        self.valid_address(offset)?;

        if offset + length as u32 > self.flash_sz.kbytes() {
            return Err(Error::LengthTooLong);
        }

        let address = (FLASH_START + offset) as *const _;

        Ok(
            // NOTE(unsafe) read with no side effects. The data returned will
            // remain valid for its lifetime because we take an immutable
            // reference to this FlashWriter, and any operation that would
            // invalidate the data returned would first require taking a mutable
            // reference to this FlashWriter.
            unsafe { core::slice::from_raw_parts(address, length) },
        )
    }

    /// Write data to `FLASH_START + offset`.
    ///
    /// If `force_data_padding` is true, the incoming data will be padded with 0xFF
    /// to satisfy the requirement that 2 words (8 bytes) must be written at a time.
    /// If `force_data_padding` is false and `data.len()` is not divisible by 8,
    /// the error `ArrayMustBeDivisibleBy8` will be returned.
    pub fn write(&mut self, offset: u32, data: &[u8], force_data_padding: bool) -> Result<()> {
        self.valid_length(offset, data.len(), force_data_padding)?;

        // Unlock Flash
        self.unlock()?;

        // According to RM0440 Rev 7, "It is only possible to program double word (2 x 32-bit data)"
        for idx in (0..data.len()).step_by(8) {
            // Check the starting address
            self.valid_address(offset + idx as u32)?;
            // Check the ending address to make sure there is no overflow
            self.valid_address(offset + 8 + idx as u32)?;

            let write_address1 = (FLASH_START + offset + idx as u32) as *mut u32;
            let write_address2 = (FLASH_START + offset + 4 + idx as u32) as *mut u32;

            let word1: u32;
            let word2: u32;

            // Check if there is enough data to make 2 words, if there isn't, pad the data with 0xFF
            if idx + 8 > data.len() {
                let mut tmp_buffer = [255u8; 8];
                tmp_buffer[idx..data.len()].copy_from_slice(&data[(idx + idx)..(data.len() + idx)]);
                let tmp_dword = u64::from_le_bytes(tmp_buffer);
                word1 = tmp_dword as u32;
                word2 = (tmp_dword >> 32) as u32;
            } else {
                word1 = (data[idx] as u32)
                    | ((data[idx + 1] as u32) << 8)
                    | ((data[idx + 2] as u32) << 16)
                    | ((data[idx + 3] as u32) << 24);

                word2 = (data[idx + 4] as u32)
                    | ((data[idx + 5] as u32) << 8)
                    | ((data[idx + 6] as u32) << 16)
                    | ((data[idx + 7] as u32) << 24);
            }

            // Set Page Programming to 1
            self.flash.cr.cr().modify(|_, w| w.pg().set_bit());

            while self.flash.sr.sr().read().bsy().bit_is_set() {}

            // NOTE(unsafe) Write to FLASH area with no side effects
            unsafe { core::ptr::write_volatile(write_address1, word1) };
            unsafe { core::ptr::write_volatile(write_address2, word2) };

            // Wait for write
            while self.flash.sr.sr().read().bsy().bit_is_set() {}

            // Set Page Programming to 0
            self.flash.cr.cr().modify(|_, w| w.pg().clear_bit());

            // Check for errors
            if self.flash.sr.sr().read().pgaerr().bit_is_set() {
                self.flash.sr.sr().modify(|_, w| w.pgaerr().clear_bit());

                self.lock()?;
                return Err(Error::ProgrammingError);
            } else if self.flash.sr.sr().read().wrperr().bit_is_set() {
                self.flash.sr.sr().modify(|_, w| w.wrperr().clear_bit());

                self.lock()?;
                return Err(Error::WriteError);
            } else if self.verify {
                // Verify written WORD
                // NOTE(unsafe) read with no side effects within FLASH area
                let verify1: u32 = unsafe { core::ptr::read_volatile(write_address1) };
                let verify2: u32 = unsafe { core::ptr::read_volatile(write_address2) };
                if verify1 != word1 && verify2 != word2 {
                    self.lock()?;
                    return Err(Error::VerifyError);
                }
            }
        }

        // Lock Flash and report success
        self.lock()?;
        Ok(())
    }

    /// Enable/disable verifying that each erase or write operation completed
    /// successfuly.
    ///
    /// When enabled, after each erase operation every address is read to make
    /// sure it contains the erase value of 0xFFFF. After each write operation,
    /// every address written is read and compared to the value that should have
    /// been written. If any address does not contain the expected value, the
    /// function will return Err.
    /// When disabled, no verification is performed, erase/write operations are
    /// assumed to have succeeded.
    pub fn change_verification(&mut self, verify: bool) {
        self.verify = verify;
    }
}

/// Extension trait to constrain the FLASH peripheral
pub trait FlashExt {
    /// Constrains the FLASH peripheral to play nicely with the other abstractions
    fn constrain(self) -> Parts;
}

impl FlashExt for FLASH {
    fn constrain(self) -> Parts {
        Parts {
            acr: ACR { _0: () },
            cr: CR { _0: () },
            eccr: ECCR { _0: () },
            keyr: KEYR { _0: () },
            _optkeyr: OPTKEYR { _0: () },
            _optr: OPTR { _0: () },
            _pcrop1sr: PCROP1SR { _0: () },
            _pcrop1er: PCROP1ER { _0: () },
            pdkeyr: PDKEYR { _0: () },
            sec1r: SEC1R { _0: () },
            sr: SR { _0: () },
            _wrp1ar: WRP1AR { _0: () },
            _wrp1br: WRP1BR { _0: () },
        }
    }
}

/// Constrained FLASH peripheral
pub struct Parts {
    /// Opaque ACR register
    pub acr: ACR,

    /// Opaque CR register
    pub(crate) cr: CR,

    /// Opaque ECCR register
    #[allow(unused)]
    pub(crate) eccr: ECCR,

    /// Opaque KEYR register
    pub(crate) keyr: KEYR,

    /// Opaque OPTKEYR register
    pub(crate) _optkeyr: OPTKEYR,

    /// Opaque optr register
    pub(crate) _optr: OPTR,

    /// Opaque PCROP1SR register
    pub(crate) _pcrop1sr: PCROP1SR,

    /// Opaque PCROP1ER register
    pub(crate) _pcrop1er: PCROP1ER,

    /// Opaque PDKEYR register
    #[allow(unused)]
    pub(crate) pdkeyr: PDKEYR,

    /// Opaque SEC1R register
    #[allow(unused)]
    pub(crate) sec1r: SEC1R,

    /// Opaque SR register
    pub(crate) sr: SR,

    /// Opaque WRP1AR register
    pub(crate) _wrp1ar: WRP1AR,

    /// Opaque WRP1BR register
    pub(crate) _wrp1br: WRP1BR,
}
impl Parts {
    #[cfg(any(feature = "stm32g431", feature = "stm32g441",))]
    pub fn writer(&mut self, flash_sz: FlashSize) -> FlashWriter<'_, { 2 * SZ_1K }> {
        FlashWriter {
            flash: self,
            flash_sz,
            verify: true,
        }
    }
    #[cfg(any(
        feature = "stm32g473",
        feature = "stm32g474",
        feature = "stm32g483",
        feature = "stm32g484",
        feature = "stm32g491",
        feature = "stm32g4a1",
    ))]
    pub fn writer<const PAGE_SIZE_KB: u32>(
        &mut self,
        flash_sz: FlashSize,
    ) -> FlashWriter<'_, PAGE_SIZE_KB> {
        FlashWriter {
            flash: self,
            flash_sz,
            verify: true,
        }
    }
}

/// Opaque ACR register
pub struct ACR {
    _0: (),
}

#[allow(dead_code)]
impl ACR {
    pub(crate) fn acr(&mut self) -> &flash::ACR {
        // NOTE(unsafe) this proxy grants exclusive access to this register
        unsafe { (*FLASH::ptr()).acr() }
    }
}

/// Opaque CR register
pub struct CR {
    _0: (),
}

#[allow(dead_code)]
impl CR {
    pub(crate) fn cr(&mut self) -> &flash::CR {
        // NOTE(unsafe) this proxy grants exclusive access to this register
        unsafe { (*FLASH::ptr()).cr() }
    }
}

/// Opaque ECCR register
pub struct ECCR {
    _0: (),
}

#[allow(dead_code)]
impl ECCR {
    pub(crate) fn eccr(&mut self) -> &flash::ECCR {
        // NOTE(unsafe) this proxy grants exclusive access to this register
        unsafe { (*FLASH::ptr()).eccr() }
    }
}

/// Opaque KEYR register
pub struct KEYR {
    _0: (),
}

#[allow(dead_code)]
impl KEYR {
    pub(crate) fn keyr(&mut self) -> &flash::KEYR {
        // NOTE(unsafe) this proxy grants exclusive access to this register
        unsafe { (*FLASH::ptr()).keyr() }
    }
}

/// Opaque OPTKEYR register
pub struct OPTKEYR {
    _0: (),
}

#[allow(dead_code)]
impl OPTKEYR {
    pub(crate) fn optkeyr(&mut self) -> &flash::OPTKEYR {
        // NOTE(unsafe) this proxy grants exclusive access to this register
        unsafe { (*FLASH::ptr()).optkeyr() }
    }
}

/// Opaque OPTR register
pub struct OPTR {
    _0: (),
}

#[allow(dead_code)]
impl OPTR {
    pub(crate) fn optr(&mut self) -> &flash::OPTR {
        // NOTE(unsafe) this proxy grants exclusive access to this register
        unsafe { (*FLASH::ptr()).optr() }
    }
}

/// Opaque PCROP1SR register
pub struct PCROP1SR {
    _0: (),
}

#[allow(dead_code)]
impl PCROP1SR {
    pub(crate) fn pcrop1sr(&mut self) -> &flash::PCROP1SR {
        // NOTE(unsafe) this proxy grants exclusive access to this register
        unsafe { (*FLASH::ptr()).pcrop1sr() }
    }
}

/// Opaque PCROP1ER register
pub struct PCROP1ER {
    _0: (),
}

#[allow(dead_code)]
impl PCROP1ER {
    pub(crate) fn pcrop1er(&mut self) -> &flash::PCROP1ER {
        // NOTE(unsafe) this proxy grants exclusive access to this register
        unsafe { (*FLASH::ptr()).pcrop1er() }
    }
}

/// Opaque PDKEYR register
pub struct PDKEYR {
    _0: (),
}

#[allow(dead_code)]
impl PDKEYR {
    pub(crate) fn pdkeyr(&mut self) -> &flash::PDKEYR {
        // NOTE(unsafe) this proxy grants exclusive access to this register
        unsafe { (*FLASH::ptr()).pdkeyr() }
    }
}

/// Opaque SEC1R register
pub struct SEC1R {
    _0: (),
}

#[allow(dead_code)]
impl SEC1R {
    pub(crate) fn sec1r(&mut self) -> &flash::SEC1R {
        // NOTE(unsafe) this proxy grants exclusive access to this register
        unsafe { (*FLASH::ptr()).sec1r() }
    }
}

/// Opaque SR register
pub struct SR {
    _0: (),
}

#[allow(dead_code)]
impl SR {
    pub(crate) fn sr(&mut self) -> &flash::SR {
        // NOTE(unsafe) this proxy grants exclusive access to this register
        unsafe { (*FLASH::ptr()).sr() }
    }
}

/// Opaque WRP1AR register
pub struct WRP1AR {
    _0: (),
}

#[allow(dead_code)]
impl WRP1AR {
    pub(crate) fn wrp1ar(&mut self) -> &flash::WRP1AR {
        // NOTE(unsafe) this proxy grants exclusive access to this register
        unsafe { (*FLASH::ptr()).wrp1ar() }
    }
}

/// Opaque WRP1BR register
pub struct WRP1BR {
    _0: (),
}

#[allow(dead_code)]
impl WRP1BR {
    pub(crate) fn wrp1br(&mut self) -> &flash::WRP1BR {
        // NOTE(unsafe) this proxy grants exclusive access to this register
        unsafe { (*FLASH::ptr()).wrp1br() }
    }
}
