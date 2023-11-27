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

macro_rules! spi {
    ($SPIX:ident, $spiX:ident,
        sck: [ $($( #[ $pmetasck:meta ] )* $SCK:ty,)+ ],
        miso: [ $($( #[ $pmetamiso:meta ] )* $MISO:ty,)+ ],
        mosi: [ $($( #[ $pmetamosi:meta ] )* $MOSI:ty,)+ ],
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

        impl<PINS> hal::spi::FullDuplex<u8> for Spi<$SPIX, PINS> {
            type Error = Error;

            fn read(&mut self) -> nb::Result<u8, Error> {
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
                        ptr::read_volatile(&self.spi.dr as *const _ as *const u8)
                    });
                } else {
                    nb::Error::WouldBlock
                })
            }

            fn send(&mut self, byte: u8) -> nb::Result<(), Error> {
                let sr = self.spi.sr.read();

                Err(if sr.ovr().bit_is_set() {
                    nb::Error::Other(Error::Overrun)
                } else if sr.modf().bit_is_set() {
                    nb::Error::Other(Error::ModeFault)
                } else if sr.crcerr().bit_is_set() {
                    nb::Error::Other(Error::Crc)
                } else if sr.txe().bit_is_set() {
                    // NOTE(write_volatile) see note above
                    unsafe { ptr::write_volatile(&self.spi.dr as *const _ as *mut u8, byte) }
                    return Ok(());
                } else {
                    nb::Error::WouldBlock
                })
            }
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
);
