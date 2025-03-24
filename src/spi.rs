use crate::dma::mux::DmaMuxResources;
use crate::dma::traits::TargetAddress;
use crate::dma::MemoryToPeripheral;
use crate::gpio::alt::SpiCommon;
use crate::rcc::Rcc;
#[cfg(feature = "spi4")]
use crate::stm32::SPI4;
use crate::stm32::{RCC, SPI1, SPI2, SPI3};
use crate::time::Hertz;
use core::ops::Deref;
use core::ptr;

use embedded_hal::spi::ErrorKind;
pub use embedded_hal::spi::{Mode, Phase, Polarity, MODE_0, MODE_1, MODE_2, MODE_3};

/// SPI error
#[derive(Debug)]
pub enum Error {
    /// Overrun occurred
    Overrun,
    /// Mode fault occurred
    ModeFault,
    /// CRC error
    Crc,
}

impl embedded_hal::spi::Error for Error {
    fn kind(&self) -> ErrorKind {
        match self {
            Self::Overrun => ErrorKind::Overrun,
            Self::ModeFault => ErrorKind::ModeFault,
            Self::Crc => ErrorKind::Other,
        }
    }
}

pub trait Instance:
    crate::Sealed + crate::rcc::Instance + SpiCommon + Deref<Target = crate::pac::spi1::RegisterBlock>
{
    const TX_MUX: DmaMuxResources;
    fn ptr() -> *const crate::pac::spi1::RegisterBlock;
}

impl Instance for SPI1 {
    const TX_MUX: DmaMuxResources = DmaMuxResources::SPI1_TX;
    fn ptr() -> *const crate::pac::spi1::RegisterBlock {
        Self::ptr()
    }
}
impl Instance for SPI2 {
    const TX_MUX: DmaMuxResources = DmaMuxResources::SPI2_TX;
    fn ptr() -> *const crate::pac::spi1::RegisterBlock {
        Self::ptr()
    }
}
impl Instance for SPI3 {
    const TX_MUX: DmaMuxResources = DmaMuxResources::SPI3_TX;
    fn ptr() -> *const crate::pac::spi1::RegisterBlock {
        Self::ptr()
    }
}
#[cfg(feature = "spi4")]
impl Instance for SPI4 {
    const TX_MUX: DmaMuxResources = DmaMuxResources::SPI4_TX;
    fn ptr() -> *const crate::pac::spi1::RegisterBlock {
        Self::ptr()
    }
}

#[derive(Debug)]
pub struct Spi<SPI: Instance> {
    spi: SPI,
    pins: (SPI::Sck, SPI::Miso, SPI::Mosi),
}

pub trait SpiExt: Instance + Sized {
    fn spi(
        self,
        pins: (
            impl Into<Self::Sck>,
            impl Into<Self::Miso>,
            impl Into<Self::Mosi>,
        ),
        mode: Mode,
        freq: Hertz,
        rcc: &mut Rcc,
    ) -> Spi<Self>;
}

impl<SPI: Instance> SpiExt for SPI {
    fn spi(
        self,
        pins: (
            impl Into<Self::Sck>,
            impl Into<Self::Miso>,
            impl Into<Self::Mosi>,
        ),
        mode: Mode,
        freq: Hertz,
        rcc: &mut Rcc,
    ) -> Spi<Self> {
        Spi::new(self, pins, mode, freq, rcc)
    }
}

pub trait FrameSize: Copy + Default {
    const DFF: bool;
}

impl FrameSize for u8 {
    const DFF: bool = false;
}
impl FrameSize for u16 {
    const DFF: bool = true;
}

impl<SPI: Instance> Spi<SPI> {
    pub fn new(
        spi: SPI,
        pins: (
            impl Into<SPI::Sck>,
            impl Into<SPI::Miso>,
            impl Into<SPI::Mosi>,
        ),
        mode: Mode,
        speed: Hertz,
        rcc: &mut Rcc,
    ) -> Self {
        // Enable and reset SPI
        unsafe {
            let rcc_ptr = &(*RCC::ptr());
            SPI::enable(rcc_ptr);
            SPI::reset(rcc_ptr);
        }

        // disable SS output
        spi.cr2().write(|w| w.ssoe().clear_bit());

        let spi_freq = speed.raw();
        let bus_freq = SPI::get_frequency(&rcc.clocks).raw();
        let br = match bus_freq / spi_freq {
            0 => unreachable!(),
            1..=2 => 0b000,
            3..=5 => 0b001,
            6..=11 => 0b010,
            12..=23 => 0b011,
            24..=47 => 0b100,
            48..=95 => 0b101,
            96..=191 => 0b110,
            _ => 0b111,
        };

        spi.cr2()
            .write(|w| unsafe { w.frxth().set_bit().ds().bits(0b111).ssoe().clear_bit() });

        spi.cr1().write(|w| unsafe {
            w.cpha().bit(mode.phase == Phase::CaptureOnSecondTransition);
            w.cpol().bit(mode.polarity == Polarity::IdleHigh);
            w.mstr().set_bit();
            w.br().bits(br);
            w.lsbfirst().clear_bit();
            w.ssm().set_bit();
            w.ssi().set_bit();
            w.rxonly().clear_bit();
            w.dff().clear_bit();
            w.bidimode().clear_bit();
            w.ssi().set_bit();
            w.spe().set_bit()
        });

        Spi {
            spi,
            pins: (pins.0.into(), pins.1.into(), pins.2.into()),
        }
    }

    #[allow(clippy::type_complexity)]
    pub fn release(self) -> (SPI, (SPI::Sck, SPI::Miso, SPI::Mosi)) {
        (self.spi, self.pins)
    }

    pub fn enable_tx_dma(self) -> Spi<SPI> {
        self.spi.cr2().modify(|_, w| w.txdmaen().set_bit());
        Spi {
            spi: self.spi,
            pins: self.pins,
        }
    }
}

impl<SPI: Instance> Spi<SPI> {
    fn nb_read<W: FrameSize>(&mut self) -> nb::Result<W, Error> {
        let sr = self.spi.sr().read();
        Err(if sr.ovr().bit_is_set() {
            nb::Error::Other(Error::Overrun)
        } else if sr.modf().bit_is_set() {
            nb::Error::Other(Error::ModeFault)
        } else if sr.crcerr().bit_is_set() {
            nb::Error::Other(Error::Crc)
        } else if sr.rxne().bit_is_set() {
            // NOTE(read_volatile) read only 1 byte (the svd2rust API only allows
            // reading a half-word)
            return Ok(unsafe { ptr::read_volatile(&self.spi.dr() as *const _ as *const W) });
        } else {
            nb::Error::WouldBlock
        })
    }
    fn nb_write<W: FrameSize>(&mut self, word: W) -> nb::Result<(), Error> {
        let sr = self.spi.sr().read();
        Err(if sr.ovr().bit_is_set() {
            nb::Error::Other(Error::Overrun)
        } else if sr.modf().bit_is_set() {
            nb::Error::Other(Error::ModeFault)
        } else if sr.crcerr().bit_is_set() {
            nb::Error::Other(Error::Crc)
        } else if sr.txe().bit_is_set() {
            let dr = self.spi.dr().as_ptr() as *mut W;
            // NOTE(write_volatile) see note above
            unsafe { ptr::write_volatile(dr, word) };
            return Ok(());
        } else {
            nb::Error::WouldBlock
        })
    }
    fn set_tx_only(&mut self) {
        self.spi
            .cr1()
            .modify(|_, w| w.bidimode().set_bit().bidioe().set_bit());
    }
    fn set_bidi(&mut self) {
        self.spi
            .cr1()
            .modify(|_, w| w.bidimode().clear_bit().bidioe().clear_bit());
    }
}

impl<SPI: Instance> embedded_hal::spi::ErrorType for Spi<SPI> {
    type Error = Error;
}

impl<SPI: Instance> embedded_hal::spi::SpiBus<u8> for Spi<SPI> {
    fn read(&mut self, words: &mut [u8]) -> Result<(), Self::Error> {
        if words.is_empty() {
            return Ok(());
        }
        // clear tx-only status in the case the previous operation was a write
        self.set_bidi();
        // prefill write fifo so that the clock doen't stop while fetch the read byte
        // one frame should be enough?
        nb::block!(self.nb_write(0u8))?;
        let len = words.len();
        for w in words[..len - 1].iter_mut() {
            // TODO: 16 bit frames, bidirectional pins
            nb::block!(self.nb_write(0u8))?;
            *w = nb::block!(self.nb_read())?;
        }
        // safety: length > 0 checked at start of function
        *words.last_mut().unwrap() = nb::block!(self.nb_read())?;
        Ok(())
    }

    fn write(&mut self, words: &[u8]) -> Result<(), Self::Error> {
        self.set_tx_only();
        for w in words {
            nb::block!(self.nb_write(*w))?;
        }
        Ok(())
    }

    fn transfer(&mut self, read: &mut [u8], write: &[u8]) -> Result<(), Self::Error> {
        if read.is_empty() {
            return self.write(write);
        } else if write.is_empty() {
            return self.read(read);
        }

        self.set_bidi();
        // same prefill as in read, this time with actual data
        nb::block!(self.nb_write(write[0]))?;
        let common_len = core::cmp::min(read.len(), write.len());
        // take 1 less because write skips the first element
        let zipped = read
            .iter_mut()
            .zip(write.iter().skip(1))
            .take(common_len - 1);
        for (r, w) in zipped {
            nb::block!(self.nb_write(*w))?;
            *r = nb::block!(self.nb_read())?;
        }
        read[common_len - 1] = nb::block!(self.nb_read())?;

        if read.len() > common_len {
            self.read(&mut read[common_len..])
        } else {
            self.write(&write[common_len..])
        }
    }
    fn transfer_in_place(&mut self, words: &mut [u8]) -> Result<(), Self::Error> {
        if words.is_empty() {
            return Ok(());
        }
        self.set_bidi();
        nb::block!(self.nb_write(words[0]))?;
        let cells = core::cell::Cell::from_mut(words).as_slice_of_cells();

        for rw in cells.windows(2) {
            let r = &rw[0];
            let w = &rw[1];

            nb::block!(self.nb_write(w.get()))?;
            r.set(nb::block!(self.nb_read())?);
        }
        *words.last_mut().unwrap() = nb::block!(self.nb_read())?;
        Ok(())
    }
    fn flush(&mut self) -> Result<(), Self::Error> {
        // stop receiving data
        self.set_tx_only();
        // wait for tx fifo to be drained by the peripheral
        while self.spi.sr().read().ftlvl() != 0 {
            core::hint::spin_loop()
        }
        // drain rx fifo

        while match self.nb_read::<u8>() {
            Ok(_) => true,
            Err(nb::Error::WouldBlock) => false,
            Err(nb::Error::Other(e)) => return Err(e),
        } {
            core::hint::spin_loop();
        }
        Ok(())
    }
}

impl<SPI: Instance> embedded_hal_old::spi::FullDuplex<u8> for Spi<SPI> {
    type Error = Error;

    fn read(&mut self) -> nb::Result<u8, Error> {
        self.nb_read()
    }

    fn send(&mut self, byte: u8) -> nb::Result<(), Error> {
        self.nb_write(byte)
    }
}
unsafe impl<SPI: Instance> TargetAddress<MemoryToPeripheral> for Spi<SPI> {
    #[inline(always)]
    fn address(&self) -> u32 {
        // unsafe: only the Tx part accesses the Tx register
        &unsafe { &*<SPI>::ptr() }.dr() as *const _ as u32
    }

    type MemSize = u8;

    const REQUEST_LINE: Option<u8> = Some(SPI::TX_MUX as u8);
}

impl<SPI: Instance> embedded_hal_old::blocking::spi::transfer::Default<u8> for Spi<SPI> {}

impl<SPI: Instance> embedded_hal_old::blocking::spi::write::Default<u8> for Spi<SPI> {}
