use crate::dma::mux::DmaMuxResources;
use crate::dma::traits::TargetAddress;
use crate::dma::MemoryToPeripheral;
use crate::gpio;
use crate::rcc::{Enable, GetBusFreq, Rcc, Reset};
#[cfg(any(
    feature = "stm32g473",
    feature = "stm32g474",
    feature = "stm32g483",
    feature = "stm32g484"
))]
use crate::stm32::SPI4;
use crate::stm32::{spi1, SPI1, SPI2, SPI3};
use crate::time::Hertz;
use core::{ops::Deref, ptr};

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

/// A filler type for when the SCK pin is unnecessary
pub struct NoSck;
/// A filler type for when the Miso pin is unnecessary
pub struct NoMiso;
/// A filler type for when the Mosi pin is unnecessary
pub struct NoMosi;

pub trait Pins<SPI> {}

pub trait PinSck<SPI> {}

pub trait PinMiso<SPI> {}

pub trait PinMosi<SPI> {}

impl<SPI, SCK, MISO, MOSI> Pins<SPI> for (SCK, MISO, MOSI)
where
    SCK: PinSck<SPI>,
    MISO: PinMiso<SPI>,
    MOSI: PinMosi<SPI>,
{
}

#[derive(Debug)]
pub struct Spi<SPI, PINS> {
    spi: SPI,
    pins: PINS,
}

pub trait SpiExt<SPI>: Sized {
    fn spi<PINS, T>(self, pins: PINS, mode: Mode, freq: T, rcc: &mut Rcc) -> Spi<SPI, PINS>
    where
        PINS: Pins<SPI>,
        T: Into<Hertz>;
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

pub trait Instance:
    crate::Sealed + Deref<Target = spi1::RegisterBlock> + Enable + Reset + GetBusFreq
{
    const DMA_MUX_RESOURCE: DmaMuxResources;
}

unsafe impl<SPI: Instance, PINS> TargetAddress<MemoryToPeripheral> for Spi<SPI, PINS> {
    #[inline(always)]
    fn address(&self) -> u32 {
        self.spi.dr() as *const _ as u32
    }
    type MemSize = u8;
    const REQUEST_LINE: Option<u8> = Some(SPI::DMA_MUX_RESOURCE as u8);
}

macro_rules! spi {
    ($SPIX:ident, $spiX:ident,
        sck: [ $($( #[ $pmetasck:meta ] )* $SCK:ident<$ASCK:ident>,)+ ],
        miso: [ $($( #[ $pmetamiso:meta ] )* $MISO:ident<$AMISO:ident>,)+ ],
        mosi: [ $($( #[ $pmetamosi:meta ] )* $MOSI:ident<$AMOSI:ident>,)+ ],
        $mux:expr,
    ) => {
        impl PinSck<$SPIX> for NoSck {}
        impl PinMiso<$SPIX> for NoMiso {}
        impl PinMosi<$SPIX> for NoMosi {}

        $(
            $( #[ $pmetasck ] )*
            impl PinSck<$SPIX> for gpio::$SCK<gpio::$ASCK> {}
        )*
        $(
            $( #[ $pmetamiso ] )*
            impl PinMiso<$SPIX> for gpio::$MISO<gpio::$AMISO> {}
        )*
        $(
            $( #[ $pmetamosi ] )*
            impl PinMosi<$SPIX> for gpio::$MOSI<gpio::$AMOSI> {}
        )*

        impl Instance for $SPIX {
            const DMA_MUX_RESOURCE: DmaMuxResources = $mux;
        }
    }
}

impl<SPI: Instance, PINS> Spi<SPI, PINS> {
    pub fn release(self) -> (SPI, PINS) {
        (self.spi, self.pins)
    }

    pub fn enable_tx_dma(self) -> Spi<SPI, PINS> {
        self.spi.cr2().modify(|_, w| w.txdmaen().set_bit());
        Spi {
            spi: self.spi,
            pins: self.pins,
        }
    }

    #[inline]
    fn nb_read<W: FrameSize>(&mut self) -> nb::Result<W, Error> {
        let sr = self.spi.sr().read();
        Err(if sr.ovr().bit_is_set() {
            nb::Error::Other(Error::Overrun)
        } else if sr.modf().bit_is_set() {
            nb::Error::Other(Error::ModeFault)
        } else if sr.crcerr().bit_is_set() {
            nb::Error::Other(Error::Crc)
        } else if sr.rxne().bit_is_set() {
            return Ok(self.read_unchecked());
        } else {
            nb::Error::WouldBlock
        })
    }
    #[inline]
    fn nb_write<W: FrameSize>(&mut self, word: W) -> nb::Result<(), Error> {
        let sr = self.spi.sr().read();
        Err(if sr.ovr().bit_is_set() {
            nb::Error::Other(Error::Overrun)
        } else if sr.modf().bit_is_set() {
            nb::Error::Other(Error::ModeFault)
        } else if sr.crcerr().bit_is_set() {
            nb::Error::Other(Error::Crc)
        } else if sr.txe().bit_is_set() {
            self.write_unchecked(word);
            return Ok(());
        } else {
            nb::Error::WouldBlock
        })
    }
    #[inline]
    fn nb_read_no_err<W: FrameSize>(&mut self) -> nb::Result<W, core::convert::Infallible> {
        if self.spi.sr().read().rxne().bit_is_set() {
            Ok(self.read_unchecked())
        } else {
            Err(nb::Error::WouldBlock)
        }
    }
    #[inline]
    fn read_unchecked<W: FrameSize>(&mut self) -> W {
        // NOTE(read_volatile) read only 1 byte (the svd2rust API only allows
        // reading a half-word)
        unsafe { ptr::read_volatile(self.spi.dr().as_ptr() as *const W) }
    }
    #[inline]
    fn write_unchecked<W: FrameSize>(&mut self, word: W) {
        // NOTE(write_volatile) see note above
        let dr = self.spi.dr().as_ptr() as *mut W;
        unsafe { ptr::write_volatile(dr, word) };
    }
    /// disables rx
    #[inline]
    pub fn set_tx_only(&mut self) {
        // very counter-intuitively, setting spi bidi mode on disables rx while transmitting
        // it's made for half-duplex spi, which they called bidirectional in the manual
        self.spi
            .cr1()
            .modify(|_, w| w.bidimode().bidirectional().bidioe().output_enabled());
    }
    /// re-enables rx if it was disabled
    #[inline]
    pub fn set_bidi(&mut self) {
        self.spi
            .cr1()
            .modify(|_, w| w.bidimode().unidirectional().bidioe().output_disabled());
    }
    fn tx_fifo_cap(&self) -> u8 {
        match self.spi.sr().read().ftlvl().bits() {
            0 => 4,
            1 => 3,
            2 => 2,
            _ => 0,
        }
    }
    fn flush_inner(&mut self) -> Result<(), Error> {
        // stop receiving data
        self.set_tx_only();
        self.spi.cr2().modify(|_, w| w.frxth().set_bit());
        // drain rx fifo
        while match self.nb_read::<u8>() {
            Ok(_) => true,
            Err(nb::Error::WouldBlock) => false,
            Err(nb::Error::Other(e)) => return Err(e),
        } {
            core::hint::spin_loop()
        }
        // wait for idle
        while self.spi.sr().read().bsy().bit() {
            core::hint::spin_loop()
        }
        Ok(())
    }
}

fn setup_spi_regs(regs: &spi1::RegisterBlock, spi_freq: u32, bus_freq: u32, mode: Mode) {
    // disable SS output
    regs.cr2().write(|w| w.ssoe().clear_bit());

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

    regs.cr2()
        .write(|w| unsafe { w.frxth().set_bit().ds().bits(0b111).ssoe().clear_bit() });

    regs.cr1().write(|w| unsafe {
        w.cpha()
            .bit(mode.phase == Phase::CaptureOnSecondTransition)
            .cpol()
            .bit(mode.polarity == Polarity::IdleHigh)
            .mstr()
            .set_bit()
            .br()
            .bits(br)
            .lsbfirst()
            .clear_bit()
            .ssm()
            .set_bit()
            .ssi()
            .set_bit()
            .rxonly()
            .clear_bit()
            .bidimode()
            .unidirectional()
            .ssi()
            .set_bit()
            .spe()
            .set_bit()
    });
}

impl<SPI: Instance> SpiExt<SPI> for SPI {
    fn spi<PINS, T>(self, pins: PINS, mode: Mode, freq: T, rcc: &mut Rcc) -> Spi<SPI, PINS>
    where
        PINS: Pins<SPI>,
        T: Into<Hertz>,
    {
        Self::enable(rcc);
        Self::reset(rcc);

        let spi_freq = freq.into().raw();
        let bus_freq = SPI::get_frequency(&rcc.clocks).raw();
        setup_spi_regs(&self, spi_freq, bus_freq, mode);

        Spi { spi: self, pins }
    }
}

impl<SPI: Instance, PINS: Pins<SPI>> embedded_hal::spi::ErrorType for Spi<SPI, PINS> {
    type Error = Error;
}

impl<SPI: Instance, PINS: Pins<SPI>> embedded_hal::spi::SpiBus<u8> for Spi<SPI, PINS> {
    fn read(&mut self, words: &mut [u8]) -> Result<(), Self::Error> {
        let len = words.len();
        if len == 0 {
            return Ok(());
        }

        // flush data from previous operations, otherwise we'd get unwanted data
        self.flush_inner()?;
        // FIFO threshold to 16 bits
        self.spi.cr2().modify(|_, w| w.frxth().clear_bit());
        self.set_bidi();

        let half_len = len / 2;
        // leftover write/read operation because bytes are not a multiple of 2
        let pair_left = len % 2;

        // prefill write fifo so that the clock doen't stop while fetch the read byte
        let prefill = core::cmp::min(self.tx_fifo_cap() as usize / 2, half_len);
        for _ in 0..prefill {
            nb::block!(self.nb_write(0u16))?;
        }

        for r in words.chunks_exact_mut(2).take(half_len - prefill) {
            let r_two: u16 = nb::block!(self.nb_read_no_err()).unwrap();
            nb::block!(self.nb_write(0u16))?;
            // safety: chunks have exact length of 2
            unsafe {
                *r.as_mut_ptr().cast() = r_two.to_le_bytes();
            }
        }

        let odd_idx = len.saturating_sub(2 * prefill + pair_left);
        // FIFO threshold to 8 bits
        self.spi.cr2().modify(|_, w| w.frxth().set_bit());
        if pair_left == 1 {
            nb::block!(self.nb_write(0u8))?;
            words[odd_idx] = nb::block!(self.nb_read_no_err()).unwrap();
        }

        for r in words[odd_idx + pair_left..].iter_mut() {
            *r = nb::block!(self.nb_read())?;
        }
        Ok(())
    }

    fn write(&mut self, words: &[u8]) -> Result<(), Self::Error> {
        self.set_tx_only();
        for w in words {
            nb::block!(self.nb_write(*w))?
        }
        Ok(())
    }

    fn transfer(&mut self, read: &mut [u8], write: &[u8]) -> Result<(), Self::Error> {
        if read.is_empty() {
            return self.write(write);
        } else if write.is_empty() {
            return self.read(read);
        }

        self.flush_inner()?;
        self.set_bidi();
        let common_len = core::cmp::min(read.len(), write.len());
        let half_len = common_len / 2;
        let pair_left = common_len % 2;

        // write two bytes at once
        let mut write_iter = write.chunks_exact(2).map(|two|
                    // safety: chunks_exact guarantees that chunks have 2 elements
                    // second byte in send queue goes to the top of the 16-bit data register for
                    // packing
                    u16::from_le_bytes(unsafe { *two.as_ptr().cast() }));

        // FIFO threshold to 16 bits
        self.spi.cr2().modify(|_, w| w.frxth().clear_bit());

        // same prefill as in read, this time with actual data
        let prefill = core::cmp::min(self.tx_fifo_cap() as usize / 2, half_len);
        for b in write_iter.by_ref().take(prefill) {
            nb::block!(self.nb_write(b))?;
        }

        // write ahead of reading
        let zipped = read
            .chunks_exact_mut(2)
            .zip(write_iter)
            .take(half_len - prefill);
        for (r, w) in zipped {
            let r_two: u16 = nb::block!(self.nb_read_no_err()).unwrap();
            nb::block!(self.nb_write(w))?;
            // same as above, length is checked by chunks_exact
            unsafe {
                *r.as_mut_ptr().cast() = r_two.to_le_bytes();
            }
        }

        // FIFO threshold to 8 bits
        self.spi.cr2().modify(|_, w| w.frxth().set_bit());

        if pair_left == 1 {
            let write_idx = common_len - 1;
            if prefill == 0 {
                nb::block!(self.nb_write(write[write_idx]))?;
                read[write_idx - 2 * prefill] = nb::block!(self.nb_read_no_err()).unwrap();
            } else {
                // there's already data in the fifo, so read that before writing more
                read[write_idx - 2 * prefill] = nb::block!(self.nb_read_no_err()).unwrap();
                nb::block!(self.nb_write(write[write_idx]))?;
            }
        }

        // read words left in the fifo
        for r in read[common_len - 2 * prefill..common_len].iter_mut() {
            *r = nb::block!(self.nb_read())?
        }

        if read.len() > common_len {
            self.read(&mut read[common_len..])
        } else {
            self.write(&write[common_len..])
        }
    }
    fn transfer_in_place(&mut self, words: &mut [u8]) -> Result<(), Self::Error> {
        let len = words.len();
        if len == 0 {
            return Ok(());
        }

        self.flush_inner()?;
        self.set_bidi();
        self.spi.cr2().modify(|_, w| w.frxth().clear_bit());
        let half_len = len / 2;
        let pair_left = len % 2;

        let prefill = core::cmp::min(self.tx_fifo_cap() as usize / 2, half_len);
        let words_alias: &mut [[u8; 2]] = unsafe {
            let ptr = words.as_mut_ptr();
            core::slice::from_raw_parts_mut(ptr as *mut [u8; 2], half_len)
        };

        for b in words_alias.iter_mut().take(prefill) {
            nb::block!(self.nb_write(u16::from_le_bytes(*b)))?;
        }

        // data is in fifo isn't zero as long as words.len() > 1 so read-then-write is fine
        for i in 0..words_alias.len() - prefill {
            let read: u16 = nb::block!(self.nb_read_no_err()).unwrap();
            words_alias[i] = read.to_le_bytes();
            let write = u16::from_le_bytes(words_alias[i + prefill]);
            nb::block!(self.nb_write(write))?;
        }
        self.spi.cr2().modify(|_, w| w.frxth().set_bit());

        if pair_left == 1 {
            let read_idx = len - 2 * prefill - 1;
            if prefill == 0 {
                nb::block!(self.nb_write(*words.last().unwrap()))?;
                words[read_idx] = nb::block!(self.nb_read_no_err()).unwrap();
            } else {
                words[read_idx] = nb::block!(self.nb_read_no_err()).unwrap();
                nb::block!(self.nb_write(*words.last().unwrap()))?;
            }
        }

        // read words left in the fifo
        for r in words.iter_mut().skip(len - 2 * prefill) {
            *r = nb::block!(self.nb_read())?;
        }
        Ok(())
    }
    fn flush(&mut self) -> Result<(), Self::Error> {
        self.flush_inner()
    }
}
impl<SPI: Instance, PINS: Pins<SPI>> embedded_hal::spi::SpiBus<u16> for Spi<SPI, PINS> {
    fn read(&mut self, words: &mut [u16]) -> Result<(), Self::Error> {
        let len = words.len();
        if len == 0 {
            return Ok(());
        }
        // flush data from previous operations, otherwise we'd get unwanted data
        self.flush_inner()?;
        // FIFO threshold to 16 bits
        self.spi.cr2().modify(|_, w| w.frxth().clear_bit());
        self.set_bidi();
        // prefill write fifo so that the clock doen't stop while fetch the read byte
        let prefill = core::cmp::min(self.tx_fifo_cap() as usize / 2, len);
        for _ in 0..prefill {
            nb::block!(self.nb_write(0u16))?;
        }

        for w in &mut words[..len - prefill] {
            *w = nb::block!(self.nb_read_no_err()).unwrap();
            nb::block!(self.nb_write(0u16))?;
        }
        for w in &mut words[len - prefill..] {
            *w = nb::block!(self.nb_read())?;
        }
        Ok(())
    }
    fn write(&mut self, words: &[u16]) -> Result<(), Self::Error> {
        self.set_tx_only();
        for w in words {
            nb::block!(self.nb_write(*w))?
        }
        Ok(())
    }
    fn transfer(&mut self, read: &mut [u16], write: &[u16]) -> Result<(), Self::Error> {
        if read.is_empty() {
            return self.write(write);
        } else if write.is_empty() {
            return self.read(read);
        }

        self.flush_inner()?;
        // FIFO threshold to 16 bits
        self.spi.cr2().modify(|_, w| w.frxth().clear_bit());
        self.set_bidi();
        let common_len = core::cmp::min(read.len(), write.len());
        // same prefill as in read, this time with actual data
        let prefill = core::cmp::min(self.tx_fifo_cap() as usize / 2, common_len);

        let mut write_iter = write.iter();
        for w in write_iter.by_ref().take(prefill) {
            nb::block!(self.nb_write(*w))?;
        }

        let zipped = read.iter_mut().zip(write_iter).take(common_len - prefill);
        for (r, w) in zipped {
            *r = nb::block!(self.nb_read_no_err()).unwrap();
            nb::block!(self.nb_write(*w))?;
        }

        for r in &mut read[common_len - prefill..common_len] {
            *r = nb::block!(self.nb_read())?
        }

        if read.len() > common_len {
            self.read(&mut read[common_len..])
        } else {
            self.write(&write[common_len..])
        }
    }
    fn transfer_in_place(&mut self, words: &mut [u16]) -> Result<(), Self::Error> {
        let len = words.len();
        if len == 0 {
            return Ok(());
        }

        self.flush_inner()?;
        self.set_bidi();
        self.spi.cr2().modify(|_, w| w.frxth().clear_bit());
        let prefill = core::cmp::min(self.tx_fifo_cap() as usize / 2, len);

        for w in &words[..prefill] {
            nb::block!(self.nb_write(*w))?;
        }

        for read_idx in 0..len - prefill {
            let write_idx = read_idx + prefill;
            words[read_idx] = nb::block!(self.nb_read_no_err()).unwrap();
            nb::block!(self.nb_write(words[write_idx]))?;
        }

        for r in &mut words[len - prefill..] {
            *r = nb::block!(self.nb_read())?;
        }
        Ok(())
    }
    fn flush(&mut self) -> Result<(), Self::Error> {
        self.flush_inner()
    }
}

impl<SPI: Instance, PINS: Pins<SPI>> embedded_hal_old::spi::FullDuplex<u8> for Spi<SPI, PINS> {
    type Error = Error;

    fn read(&mut self) -> nb::Result<u8, Error> {
        self.nb_read()
    }

    fn send(&mut self, byte: u8) -> nb::Result<(), Error> {
        self.nb_write(byte)
    }
}

impl<SPI: Instance, PINS: Pins<SPI>> embedded_hal_old::blocking::spi::transfer::Default<u8>
    for Spi<SPI, PINS>
{
}

impl<SPI: Instance, PINS: Pins<SPI>> embedded_hal_old::blocking::spi::write::Default<u8>
    for Spi<SPI, PINS>
{
}

spi!(
    SPI1,
    spi1,
    sck: [
        PA5<AF5>,
        PB3<AF5>,
        #[cfg(any(
            feature = "stm32g473",
            feature = "stm32g474",
            feature = "stm32g483",
            feature = "stm32g484"
        ))]
        PG2<AF5>,
    ],
    miso: [
        PA6<AF5>,
        PB4<AF5>,
        #[cfg(any(
            feature = "stm32g473",
            feature = "stm32g474",
            feature = "stm32g483",
            feature = "stm32g484"
        ))]
        PG3<AF5>,
    ],
    mosi: [
        PA7<AF5>,
        PB5<AF5>,
        #[cfg(any(
            feature = "stm32g473",
            feature = "stm32g474",
            feature = "stm32g483",
            feature = "stm32g484"
        ))]
        PG4<AF5>,
    ],
    DmaMuxResources::SPI1_TX,
);

spi!(
    SPI2,
    spi2,
    sck: [
        PF1<AF5>,
        PF9<AF5>,
        PF10<AF5>,
        PB13<AF5>,
    ],
    miso: [
        PA10<AF5>,
        PB14<AF5>,
    ],
    mosi: [
        PA11<AF5>,
        PB15<AF5>,
    ],
    DmaMuxResources::SPI2_TX,
);

spi!(
    SPI3,
    spi3,
    sck: [
        PB3<AF6>,
        PC10<AF6>,
        #[cfg(any(
            feature = "stm32g473",
            feature = "stm32g474",
            feature = "stm32g483",
            feature = "stm32g484"
        ))]
        PG9<AF6>,
    ],
    miso: [
        PB4<AF6>,
        PC11<AF6>,
    ],
    mosi: [
        PB5<AF6>,
        PC12<AF6>,
    ],
    DmaMuxResources::SPI3_TX,
);

#[cfg(any(
    feature = "stm32g473",
    feature = "stm32g474",
    feature = "stm32g483",
    feature = "stm32g484"
))]
spi!(
    SPI4,
    spi4,
    sck: [
        PE2<AF5>,
        PE12<AF5>,
    ],
    miso: [
        PE5<AF5>,
        PE13<AF5>,
    ],
    mosi: [
        PE6<AF5>,
        PE14<AF5>,
    ],
    DmaMuxResources::SPI4_TX,
);
