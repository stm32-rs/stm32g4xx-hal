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
    const fn bytes(self) -> u32 {
        SZ_1K * self as u32
    }
}

pub struct FlashWriter<'a, const SECTOR_SIZE: u32> {
    flash: &'a mut Parts,
    flash_sz: FlashSize,
    verify: bool,
    is_dual_bank: bool,
}
impl<'a, const SECTOR_SIZE: u32> FlashWriter<'a, SECTOR_SIZE> {
    #[allow(unused)]
    fn unlock_options(&mut self) -> Result<()> {
        // Check if flash is busy
        self.wait_for_ready();

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
        self.wait_for_ready();

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
        self.wait_for_ready();

        // Set lock bit
        self.flash.cr.cr().modify(|_, w| w.lock().set_bit());

        // Verify success
        match self.flash.cr.cr().read().lock().bit_is_set() {
            true => Ok(()),
            false => Err(Error::LockError),
        }
    }

    fn valid_address(&self, offset: u32) -> Result<()> {
        let offset = if offset >= 0x4_0000 && self.is_dual_bank {
            offset - 0x4_0000
        } else {
            offset
        };

        let bank_size = if self.is_dual_bank {
            // Each bank is only half the size in dual bank mode
            self.flash_sz.bytes() / 2
        } else {
            self.flash_sz.bytes()
        };

        if offset > bank_size {
            return Err(Error::AddressLargerThanFlash);
        }

        Ok(())
    }

    fn valid_length(&self, offset: u32, length: usize, force_padding: bool) -> Result<()> {
        if !force_padding && length % 8 != 0 {
            return Err(Error::ArrayMustBeDivisibleBy8);
        }

        let end = offset
            .checked_add(length as u32)
            .ok_or(Error::LengthTooLong)?;

        match self.valid_address(end) {
            Err(Error::AddressLargerThanFlash) => Err(Error::LengthTooLong),
            x => x,
        }
    }

    /// Erase sector which contains `start_offset`
    pub fn page_erase(&mut self, start_offset: u32) -> Result<()> {
        self.valid_address(start_offset)?;

        if start_offset % SECTOR_SIZE != 0 {
            return Err(Error::AddressMisaligned);
        }

        // Unlock Flash
        self.unlock()?;

        // Wait for operation to finish
        self.wait_for_ready();

        self.check_and_clear_errors()?;

        let page = start_offset / SECTOR_SIZE;
        //defmt::dbg!(page);

        // Write address bits and Set Page Erase
        // NOTE(unsafe) This sets the page address in the Address Register.
        // The side-effect of this write is that the page will be erased when we
        // set the STRT bit in the CR below. The address is validated by the
        // call to self.valid_address() above.
        // Category 3 device
        #[cfg(any(
            feature = "stm32g471",
            feature = "stm32g473",
            feature = "stm32g474",
            feature = "stm32g483"
        ))]
        unsafe {
            const PAGES_PER_BANK: u32 = 128;
            let (is_bank2, page) = if self.is_dual_bank() && page >= PAGES_PER_BANK {
                //defmt::info!("Bank 2");
                // page 0 in bank 2 is at the same address as page 128 would have had been, assuming the same page size
                (true, page - PAGES_PER_BANK)
            } else {
                (false, page)
            };

            defmt::dbg!(is_bank2);
            defmt::dbg!(page);

            self.flash.cr.cr().modify(|_, w| {
                w.bits((is_bank2 as u32) << 11) // BKER // TODO remove this once it's added to the PAC
                    .pnb()
                    .bits(page.try_into().unwrap())
                    .per()
                    .set_bit()
            });
            defmt::info!("{:b}", self.flash.cr.cr().read().bits());
        }

        // Not category 3 device
        #[cfg(not(any(
            feature = "stm32g471",
            feature = "stm32g473",
            feature = "stm32g474",
            feature = "stm32g483"
        )))]
        unsafe {
            self.flash
                .cr
                .cr()
                .modify(|_, w| w.pnb().bits(page.try_into().unwrap()).per().set_bit());
        }

        // Start Operation
        self.flash.cr.cr().modify(|_, w| w.strt().set_bit());

        self.wait_for_ready();

        // Remove Page Erase Operation bit
        self.flash.cr.cr().modify(|_, w| w.per().clear_bit());

        // Re-lock flash
        self.lock()?;

        self.check_and_clear_errors()?;

        if self.verify {
            let read = self.read(start_offset, SECTOR_SIZE as usize / 4)?;
            for read in read.into_iter() {
                assert_eq!(*read, u32::MAX);
            }
        }

        Ok(())
    }

    /// Erase the Flash Sectors from `FLASH_START + start_offset` to `length`
    pub fn erase(&mut self, start_offset: u32, length: usize) -> Result<()> {
        self.valid_address(start_offset)?;

        // Make sure `start_offset` points to the begining of the page
        // so as to avoid erasing any data before it in the same page
        if start_offset % SECTOR_SIZE != 0 {
            return Err(Error::AddressMisaligned);
        }

        self.valid_length(start_offset, length, true)?;

        // Erase every sector touched by start_offset + length
        for offset in
            (start_offset..start_offset + length as u32).step_by(SECTOR_SIZE.try_into().unwrap())
        {
            defmt::dbg!(offset);
            self.page_erase(offset)?;
        }

        // Report Success
        Ok(())
    }

    /// Retrieve a slice of data from `FLASH_START + offset`
    pub fn read(&mut self, offset: u32, length: usize) -> Result<&[u32]> {
        self.valid_address(offset)?;

        //if offset + length as u32 > self.flash_sz.kbytes() {
        //    return Err(Error::LengthTooLong);
        //}

        let todo_dont = ();
        self.check_and_clear_errors().unwrap();

        let address = (FLASH_START + offset) as *const u32;

        // WARNING: I am getting the feeling that wee need to read this using volatile u32 reads

        Ok(
            // NOTE(unsafe) read with no side effects. The data returned will
            // remain valid for its lifetime because we take an immutable
            // reference to this FlashWriter, and any operation that would
            // invalidate the data returned would first require taking a mutable
            // reference to this FlashWriter.
            unsafe { core::slice::from_raw_parts(address, (length + 4 - 1) / 4) },
        )
    }

    pub fn read_exact(&mut self, start_offset: u32, dst: &mut [u8]) {
        self.check_and_clear_errors().unwrap();
        let todo_maybe_dont = ();

        let address = (FLASH_START + start_offset) as *const u32;
        for (i, dst) in dst.chunks_mut(4).enumerate() {
            let word = unsafe { core::ptr::read_volatile(address.add(i)) };
            dst.copy_from_slice(&word.to_ne_bytes()[..dst.len()]);
        }
    }

    /// Write data to `FLASH_START + offset`.
    ///
    /// If `force_data_padding` is true, the incoming data will be padded with 0xFF
    /// to satisfy the requirement that 2 words (8 bytes) must be written at a time.
    /// If `force_data_padding` is false and `data.len()` is not divisible by 8,
    /// the error `ArrayMustBeDivisibleBy8` will be returned.
    pub fn write(&mut self, offset: u32, data: &[u8], force_data_padding: bool) -> Result<()> {
        self.valid_address(offset)?;
        self.valid_length(offset, data.len(), force_data_padding)?;
        assert_eq!(offset % 8, 0);

        // Unlock Flash
        self.unlock()?;

        self.wait_for_ready();
        self.check_and_clear_errors()?;

        // Set Page Programming to 1
        self.flash.cr.cr().modify(|_, w| w.pg().set_bit());

        // According to RM0440 Rev 7, "It is only possible to program double word (2 x 32-bit data)"
        for (idx, data) in data.chunks(8).enumerate() {
            let write_address1 = (FLASH_START + offset + 8 * idx as u32) as *mut u32;
            let write_address2 = unsafe { write_address1.add(1) };
            //defmt::dbg!(write_address1);
            //defmt::dbg!(write_address2);

            // Check if there is enough data to make 2 words, if there isn't, pad the data with 0xFF
            let mut tmp_buffer = [0xFF; 8];
            tmp_buffer[..data.len()].copy_from_slice(data);
            let word1 = u32::from_le_bytes(tmp_buffer[0..4].try_into().unwrap()); // Unwrap: We know the size is exactly 4
            let word2 = u32::from_le_bytes(tmp_buffer[4..8].try_into().unwrap());

            // NOTE(unsafe) Write to FLASH area with no side effects
            unsafe { core::ptr::write_volatile(write_address1, word1) };
            unsafe { core::ptr::write_volatile(write_address2, word2) };

            // Wait for write
            self.wait_for_ready();

            // Clear EOP bit(is cleared by writing 1)
            if self.flash.sr.sr().read().eop().bit_is_set() {
                self.flash.sr.sr().modify(|_, w| w.eop().set_bit());
            }
            self.check_and_clear_errors()?; // TODO dont
        }

        // Set Page Programming to 0
        self.flash.cr.cr().modify(|_, w| w.pg().clear_bit());

        self.check_and_clear_errors()?;

        if self.verify {
            let read = self.read(offset, data.len() / 4)?;
            for (data, read) in data.chunks(4).zip(read) {
                assert_eq!(data, read.to_ne_bytes());
                let todo_remaining_bytes = ();
            }
        }

        // Lock Flash and report success
        self.lock()?;
        Ok(())
    }

    /// NOTE: This will (try to) lock the flash if there is an error
    fn check_and_clear_errors(&mut self) -> Result<()> {
        let status = self.flash.sr.sr().read();
        assert!(status.fasterr().bit_is_clear());
        assert!(status.miserr().bit_is_clear());
        assert!(status.operr().bit_is_clear());
        assert!(status.optverr().bit_is_clear());
        assert!(status.progerr().bit_is_clear());
        assert!(status.rderr().bit_is_clear());
        assert!(status.sizerr().bit_is_clear());
        assert!(status.pgserr().bit_is_clear());

        Ok(if status.pgaerr().bit_is_set() {
            self.flash.sr.sr().modify(|_, w| w.pgaerr().clear_bit());
            panic!();
            self.lock()?;
            return Err(Error::ProgrammingError);
        } else if status.wrperr().bit_is_set() {
            self.flash.sr.sr().modify(|_, w| w.wrperr().clear_bit());

            self.lock()?;
            return Err(Error::WriteError);
        })
    }

    fn wait_for_ready(&mut self) {
        // Wait for operation to finish
        while self.flash.sr.sr().read().bsy().bit_is_set() {}
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

    #[inline(always)]
    pub fn is_dual_bank(&mut self) -> bool {
        self.is_dual_bank
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
    pub fn writer(&mut self, flash_sz: FlashSize) -> FlashWriter<{ 2 * SZ_1K }> {
        FlashWriter {
            flash: self,
            flash_sz,
            verify: true,
        }
    }
    #[cfg(any(
        feature = "stm32g471",
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
    ) -> FlashWriter<PAGE_SIZE_KB> {
        let is_dual_bank = self.is_dual_bank();
        FlashWriter {
            is_dual_bank,
            flash: self,
            flash_sz,
            verify: true,
        }
    }

    pub fn is_dual_bank(&mut self) -> bool {
        // TODO: use dbank() instead of `1 << 22` once it gets added to pac

        // Category 3 device
        #[cfg(any(
            feature = "stm32g471",
            feature = "stm32g473",
            feature = "stm32g474",
            feature = "stm32g483"
        ))]
        let dbank = self._optr.optr().read().bits() & (1 << 22) != 0;

        // Not category 3 device
        #[cfg(not(any(
            feature = "stm32g471",
            feature = "stm32g473",
            feature = "stm32g474",
            feature = "stm32g483"
        )))]
        let dbank = false;

        dbank
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
        unsafe { &(*FLASH::ptr()).acr }
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
        unsafe { &(*FLASH::ptr()).cr }
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
        unsafe { &(*FLASH::ptr()).eccr }
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
        unsafe { &(*FLASH::ptr()).keyr }
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
        unsafe { &(*FLASH::ptr()).optkeyr }
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
        unsafe { &(*FLASH::ptr()).optr }
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
        unsafe { &(*FLASH::ptr()).pcrop1sr }
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
        unsafe { &(*FLASH::ptr()).pcrop1er }
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
        unsafe { &(*FLASH::ptr()).pdkeyr }
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
        unsafe { &(*FLASH::ptr()).sec1r }
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
        unsafe { &(*FLASH::ptr()).sr }
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
        unsafe { &(*FLASH::ptr()).wrp1ar }
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
        unsafe { &(*FLASH::ptr()).wrp1br }
    }
}
