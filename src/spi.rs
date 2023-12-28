use crate::dma::mux::DmaMuxResources;
use crate::dma::traits::TargetAddress;
use crate::dma::MemoryToPeripheral;
use crate::gpio::{gpioa::*, gpiob::*, gpioc::*, gpiof::*, Alternate, AF5, AF6};
#[cfg(any(
    feature = "stm32g471",
    feature = "stm32g473",
    feature = "stm32g474",
    feature = "stm32g483",
    feature = "stm32g484"
))]
use crate::gpio::{gpioe::*, gpiog::*};
use crate::rcc::{Enable, GetBusFreq, Rcc, RccBus, Reset};
#[cfg(any(
    feature = "stm32g471",
    feature = "stm32g473",
    feature = "stm32g474",
    feature = "stm32g483",
    feature = "stm32g484"
))]
use crate::stm32::SPI4;
use crate::stm32::{RCC, SPI1, SPI2, SPI3};
use crate::time::Hertz;
use core::cell::UnsafeCell;
use core::ptr;

pub use hal::spi::{Mode, Phase, Polarity, MODE_0, MODE_1, MODE_2, MODE_3};

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

impl embedded_hal_one::spi::Error for Error {
    fn kind(&self) -> embedded_hal_one::spi::ErrorKind {
        match self {
            Self::Overrun => embedded_hal_one::spi::ErrorKind::Overrun,
            Self::ModeFault => embedded_hal_one::spi::ErrorKind::ModeFault,
            Self::Crc => embedded_hal_one::spi::ErrorKind::Other,
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

macro_rules! spi {
    ($SPIX:ident, $spiX:ident,
        sck: [ $($( #[ $pmetasck:meta ] )* $SCK:ty,)+ ],
        miso: [ $($( #[ $pmetamiso:meta ] )* $MISO:ty,)+ ],
        mosi: [ $($( #[ $pmetamosi:meta ] )* $MOSI:ty,)+ ],
        $mux:expr,
    ) => {
        impl PinSck<$SPIX> for NoSck {}

        impl PinMiso<$SPIX> for NoMiso {}

        impl PinMosi<$SPIX> for NoMosi {}

        $(
            $( #[ $pmetasck ] )*
            impl PinSck<$SPIX> for $SCK {}
        )*
        $(
            $( #[ $pmetamiso ] )*
            impl PinMiso<$SPIX> for $MISO {}
        )*
        $(
            $( #[ $pmetamosi ] )*
            impl PinMosi<$SPIX> for $MOSI {}
        )*

        impl<PINS: Pins<$SPIX>> Spi<$SPIX, PINS> {
            pub fn $spiX<T>(
                spi: $SPIX,
                pins: PINS,
                mode: Mode,
                speed: T,
                rcc: &mut Rcc
            ) -> Self
            where
            T: Into<Hertz>
            {
                 // Enable and reset SPI
                unsafe {
                    let rcc_ptr = &(*RCC::ptr());
                    $SPIX::enable(rcc_ptr);
                    $SPIX::reset(rcc_ptr);
                }

                // disable SS output
                spi.cr2.write(|w| w.ssoe().clear_bit());

                let spi_freq = speed.into().raw();
                let bus_freq = <$SPIX as RccBus>::Bus::get_frequency(&rcc.clocks).raw();
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

                spi.cr2.write(|w| unsafe {
                    w.frxth().set_bit().ds().bits(0b111).ssoe().clear_bit()
                });

                spi.cr1.write(|w| unsafe {
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
                        .dff()
                        .clear_bit()
                        .bidimode()
                        .clear_bit()
                        .ssi()
                        .set_bit()
                        .spe()
                        .set_bit()
                });

                Spi { spi, pins }
            }

            pub fn release(self) -> ($SPIX, PINS) {
                (self.spi, self.pins)
            }

            pub fn enable_tx_dma(self) -> Spi<$SPIX, PINS> {
                self.spi.cr2.modify(|_, w| w.txdmaen().set_bit());
                Spi {
                    spi: self.spi,
                    pins: self.pins,
                }
            }
        }

        impl<PINS> Spi<$SPIX, PINS> {
            fn nb_read<W: FrameSize>(&mut self) -> nb::Result<W, Error> {
                let sr = self.spi.sr.read();
                Err(if sr.ovr().bit_is_set() {
                    nb::Error::Other(Error::Overrun)
                } else if sr.modf().bit_is_set() {
                    nb::Error::Other(Error::ModeFault)
                } else if sr.crcerr().bit_is_set() {
                    nb::Error::Other(Error::Crc)
                } else if sr.rxne().bit_is_set() {
                    // NOTE(read_volatile) read only 1 byte (the svd2rust API only allows
                    // reading a half-word)
                    return Ok(unsafe {
                        ptr::read_volatile(&self.spi.dr as *const _ as *const W)
                    });
                } else {
                    nb::Error::WouldBlock
                })
            }
            fn nb_write<W: FrameSize>(&mut self, word: W) -> nb::Result<(), Error> {
                let sr = self.spi.sr.read();
                Err(if sr.ovr().bit_is_set() {
                    nb::Error::Other(Error::Overrun)
                } else if sr.modf().bit_is_set() {
                    nb::Error::Other(Error::ModeFault)
                } else if sr.crcerr().bit_is_set() {
                    nb::Error::Other(Error::Crc)
                } else if sr.txe().bit_is_set() {
                    let dr = &self.spi.dr as *const _ as *const UnsafeCell<W>;
                    // NOTE(write_volatile) see note above
                    unsafe { ptr::write_volatile(UnsafeCell::raw_get(dr), word) };
                    return Ok(());
                } else {
                    nb::Error::WouldBlock
                })
            }
            fn set_tx_only(&mut self) {
                self.spi
                    .cr1
                    .modify(|_, w| w.bidimode().set_bit().bidioe().set_bit());
            }
            fn set_bidi(&mut self) {
                self.spi
                    .cr1
                    .modify(|_, w| w.bidimode().clear_bit().bidioe().clear_bit());
            }
        }

        impl SpiExt<$SPIX> for $SPIX {
            fn spi<PINS, T>(self, pins: PINS, mode: Mode, freq: T, rcc: &mut Rcc) -> Spi<$SPIX, PINS>
            where
                PINS: Pins<$SPIX>,
                T: Into<Hertz>
                {
                    Spi::$spiX(self, pins, mode, freq, rcc)
                }
        }

        impl<PINS> embedded_hal_one::spi::ErrorType for Spi<$SPIX, PINS> {
            type Error = Error;
        }

        impl<PINS> embedded_hal_one::spi::SpiBus for Spi<$SPIX, PINS> {
            fn read(&mut self, words: &mut [u8]) -> Result<(), Self::Error> {
                if words.len() == 0 { return Ok(()) }
                // clear tx-only status in the case the previous operation was a write
                self.set_bidi();
                // prefill write fifo so that the clock doen't stop while fetch the read byte
                // one frame should be enough?
                nb::block!(self.nb_write(0u8))?;
                let len = words.len();
                for w in words[..len-1].iter_mut() {
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
                Ok(for w in words {
                    nb::block!(self.nb_write(*w))?
                })
            }

            fn transfer(&mut self, read: &mut [u8], write: &[u8]) -> Result<(), Self::Error> {
                if read.len() == 0 {
                    return self.write(write)
                } else if write.len() == 0 {
                    return self.read(read)
                }

                self.set_bidi();
                // same prefill as in read, this time with actual data
                nb::block!(self.nb_write(write[0]))?;
                let common_len = core::cmp::min(read.len(), write.len());
                // take 1 less because write skips the first element
                let zipped = read.iter_mut().zip(write.into_iter().skip(1)).take(common_len - 1);
                for (r, w) in zipped {
                    nb::block!(self.nb_write(*w))?;
                    *r = nb::block!(self.nb_read())?;
                }
                read[common_len-1] = nb::block!(self.nb_read())?;
                
                if read.len() > common_len {
                    self.read(&mut read[common_len..])
                } else {
                    self.write(&write[common_len..])
                }
            }
            fn transfer_in_place(&mut self, words: &mut [u8]) -> Result<(), Self::Error> {
                if words.len() == 0 { return Ok(()) }
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
                while self.spi.sr.read().ftlvl() != 0 { core::hint::spin_loop() };
                // drain rx fifo
                Ok(while match self.nb_read::<u8>() {
                    Ok(_) => true,
                    Err(nb::Error::WouldBlock) => false,
                    Err(nb::Error::Other(e)) => return Err(e)
                } { core::hint::spin_loop() })
            }
        }

        impl<PINS> hal::spi::FullDuplex<u8> for Spi<$SPIX, PINS> {
            type Error = Error;

            fn read(&mut self) -> nb::Result<u8, Error> {
                self.nb_read()
            }

            fn send(&mut self, byte: u8) -> nb::Result<(), Error> {
                self.nb_write(byte)
            }
        }
        unsafe impl<Pin> TargetAddress<MemoryToPeripheral> for Spi<$SPIX, Pin> {
            #[inline(always)]
            fn address(&self) -> u32 {
                // unsafe: only the Tx part accesses the Tx register
                &unsafe { &*<$SPIX>::ptr() }.dr as *const _ as u32
            }

            type MemSize = u8;

            const REQUEST_LINE: Option<u8> = Some($mux as u8);
        }


        impl<PINS> ::hal::blocking::spi::transfer::Default<u8> for Spi<$SPIX, PINS> {}

        impl<PINS> ::hal::blocking::spi::write::Default<u8> for Spi<$SPIX, PINS> {}
    }
}

spi!(
    SPI1,
    spi1,
    sck: [
        PA5<Alternate<AF5>>,
        PB3<Alternate<AF5>>,
        #[cfg(any(
            feature = "stm32g471",
            feature = "stm32g473",
            feature = "stm32g474",
            feature = "stm32g483",
            feature = "stm32g484"
        ))]
        PG2<Alternate<AF5>>,
    ],
    miso: [
        PA6<Alternate<AF5>>,
        PB4<Alternate<AF5>>,
        #[cfg(any(
            feature = "stm32g471",
            feature = "stm32g473",
            feature = "stm32g474",
            feature = "stm32g483",
            feature = "stm32g484"
        ))]
        PG3<Alternate<AF5>>,
    ],
    mosi: [
        PA7<Alternate<AF5>>,
        PB5<Alternate<AF5>>,
        #[cfg(any(
            feature = "stm32g471",
            feature = "stm32g473",
            feature = "stm32g474",
            feature = "stm32g483",
            feature = "stm32g484"
        ))]
        PG4<Alternate<AF5>>,
    ],
    DmaMuxResources::SPI1_TX,
);

spi!(
    SPI2,
    spi2,
    sck: [
        PF1<Alternate<AF5>>,
        PF9<Alternate<AF5>>,
        PF10<Alternate<AF5>>,
        PB13<Alternate<AF5>>,
    ],
    miso: [
        PA10<Alternate<AF5>>,
        PB14<Alternate<AF5>>,
    ],
    mosi: [
        PA11<Alternate<AF5>>,
        PB15<Alternate<AF5>>,
    ],
    DmaMuxResources::SPI2_TX,
);

spi!(
    SPI3,
    spi3,
    sck: [
        PB3<Alternate<AF6>>,
        PC10<Alternate<AF6>>,
        #[cfg(any(
            feature = "stm32g471",
            feature = "stm32g473",
            feature = "stm32g474",
            feature = "stm32g483",
            feature = "stm32g484"
        ))]
        PG9<Alternate<AF6>>,
    ],
    miso: [
        PB4<Alternate<AF6>>,
        PC11<Alternate<AF6>>,
    ],
    mosi: [
        PB5<Alternate<AF6>>,
        PC12<Alternate<AF6>>,
    ],
    DmaMuxResources::SPI3_TX,
);

#[cfg(any(
    feature = "stm32g471",
    feature = "stm32g473",
    feature = "stm32g474",
    feature = "stm32g483",
    feature = "stm32g484"
))]
spi!(
    SPI4,
    spi4,
    sck: [
        PE2<Alternate<AF5>>,
        PE12<Alternate<AF5>>,
    ],
    miso: [
        PE5<Alternate<AF5>>,
        PE13<Alternate<AF5>>,
    ],
    mosi: [
        PE6<Alternate<AF5>>,
        PE14<Alternate<AF5>>,
    ],
    DmaMuxResources::SPI4_TX,
);
