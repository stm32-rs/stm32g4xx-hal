use core::fmt::{self, Debug};
use core::marker::PhantomData;

use crate::dma::{
    mux::DmaMuxResources, traits::TargetAddress, MemoryToPeripheral, PeripheralToMemory,
};
use crate::gpio::{self, PushPull};
use crate::rcc::{Enable, GetBusFreq, Rcc, RccBus, Reset};
use crate::stm32::*;

use cortex_m::interrupt;
use nb::block;

use embedded_hal_old::serial::Write;
use embedded_io::{ReadReady, WriteReady};

use crate::serial::config::*;
/// Serial error
#[derive(Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Error {
    /// Framing error
    Framing,
    /// Noise error
    Noise,
    /// RX buffer overrun
    Overrun,
    /// Parity check error
    Parity,
}

impl embedded_io::Error for Error {
    fn kind(&self) -> embedded_io::ErrorKind {
        match self {
            Error::Framing => embedded_io::ErrorKind::InvalidData,
            Error::Noise => embedded_io::ErrorKind::InvalidData,
            Error::Overrun => embedded_io::ErrorKind::Other,
            Error::Parity => embedded_io::ErrorKind::InvalidData,
        }
    }
}

/// Interrupt event
pub enum Event {
    /// TXFIFO reaches the threshold
    TXFT = 1 << 27,
    /// This bit is set by hardware when the threshold programmed in RXFTCFG in USART_CR3 register is reached.
    RXFT = 1 << 26,

    /// RXFIFO full
    RXFF = 1 << 24,
    /// TXFIFO empty
    TXFE = 1 << 23,

    /// Active when a communication is ongoing on the RX line
    BUSY = 1 << 16,

    /// Receiver timeout.This bit is set by hardware when the timeout value,
    /// programmed in the RTOR register has lapsed, without any communication.
    RTOF = 1 << 11,
    /// Transmit data register empty. New data can be sent
    Txe = 1 << 7,

    /// Transmission Complete. The last data written in the USART_TDR has been transmitted out of the shift register.
    TC = 1 << 6,
    /// New data has been received
    Rxne = 1 << 5,
    /// Idle line state detected
    Idle = 1 << 4,

    /// Overrun error
    ORE = 1 << 3,

    /// Noise detection flag
    NE = 1 << 2,

    /// Framing error
    FE = 1 << 1,

    /// Parity error
    PE = 1 << 0,
}
impl Event {
    fn val(self) -> u32 {
        self as u32
    }
}

pub use gpio::alt::SerialAsync as CommonPins;

// Implemented by all USART/UART instances
pub trait Instance: crate::Sealed + crate::Ptr + Enable + Reset + CommonPins {}

/// Serial receiver
pub struct Rx<USART: Instance, Dma = NoDMA> {
    pin: Option<USART::Rx<PushPull>>,
    usart: USART,
    _dma: PhantomData<Dma>,
}

/// Serial transmitter
pub struct Tx<USART: Instance, Dma = NoDMA, Otype = PushPull> {
    pin: Option<USART::Tx<Otype>>,
    usart: USART,
    _dma: PhantomData<Dma>,
}

/// Serial abstraction
pub struct Serial<USART: Instance, Otype = PushPull> {
    tx: Tx<USART, NoDMA, Otype>,
    rx: Rx<USART, NoDMA>,
}

/// Type state for Tx/Rx, indicating operation without DMA
#[derive(Debug)]
pub struct NoDMA;
/// Type state for Tx/Rx, indicating configuration for DMA
#[derive(Debug)]
pub struct DMA;

#[allow(non_upper_case_globals)]
pub trait SerialExt<Config>: Sized + Instance {
    fn usart<Otype>(
        self,
        pins: (impl Into<Self::Tx<Otype>>, impl Into<Self::Rx<PushPull>>),
        config: impl Into<Config>,
        rcc: &mut Rcc,
    ) -> Result<Serial<Self, Otype>, InvalidConfig>;
    fn tx<Otype>(
        self,
        tx: impl Into<Self::Tx<Otype>>,
        config: impl Into<Config>,
        rcc: &mut Rcc,
    ) -> Result<Tx<Self, NoDMA, Otype>, InvalidConfig>;
    fn rx(
        self,
        rx: impl Into<Self::Rx<PushPull>>,
        config: impl Into<Config>,
        rcc: &mut Rcc,
    ) -> Result<Rx<Self, NoDMA>, InvalidConfig>;
}

impl<USART: Instance, Otype> fmt::Write for Serial<USART, Otype>
where
    Self: embedded_hal_old::serial::Write<u8>,
    <Self as embedded_hal_old::serial::Write<u8>>::Error: Debug,
{
    fn write_str(&mut self, s: &str) -> fmt::Result {
        for c in s.as_bytes() {
            block!(self.write(*c)).unwrap(); // self.write does not fail
        }
        Ok(())
    }
}

impl<USART: Instance, Dma, Otype> fmt::Write for Tx<USART, Dma, Otype>
where
    Self: embedded_hal_old::serial::Write<u8>,
    <Self as embedded_hal_old::serial::Write<u8>>::Error: Debug,
{
    fn write_str(&mut self, s: &str) -> fmt::Result {
        for c in s.as_bytes() {
            block!(self.write(*c)).unwrap(); // self.write does not fail
        }
        Ok(())
    }
}

macro_rules! uart_shared {
    ($USARTX:ident, $dmamux_rx:ident, $dmamux_tx:ident) => {
        impl Instance for $USARTX {}

        impl<Dma> Rx<$USARTX, Dma> {
            /// Starts listening for an interrupt event
            pub fn listen(&mut self) {
                self.usart.cr1().modify(|_, w| w.rxneie().set_bit());
            }

            /// Stop listening for an interrupt event
            pub fn unlisten(&mut self) {
                self.usart.cr1().modify(|_, w| w.rxneie().clear_bit());
            }

            /// Return true if the rx register is not empty (and can be read)
            pub fn is_rxne(&self) -> bool {
                self.usart.isr().read().rxne().bit_is_set()
            }

            /// Returns true if the rx fifo threshold has been reached.
            pub fn fifo_threshold_reached(&self) -> bool {
                self.usart.isr().read().rxft().bit_is_set()
            }
        }

        impl Rx<$USARTX, NoDMA> {
            pub fn enable_dma(self) -> Rx<$USARTX, DMA> {
                // NOTE(unsafe) critical section prevents races
                cortex_m::interrupt::free(|_| {
                    self.usart.cr3().modify(|_, w| w.dmar().set_bit());
                });

                Rx {
                    pin: self.pin,
                    usart: self.usart,
                    _dma: PhantomData,
                }
            }
            fn data_ready(&mut self) -> nb::Result<(), Error> {
                let usart = &self.usart;
                let isr = usart.isr().read();
                Err(if isr.pe().bit_is_set() {
                    usart.icr().write(|w| w.pecf().clear());
                    nb::Error::Other(Error::Parity)
                } else if isr.fe().bit_is_set() {
                    usart.icr().write(|w| w.fecf().clear());
                    nb::Error::Other(Error::Framing)
                } else if isr.nf().bit_is_set() {
                    usart.icr().write(|w| w.ncf().clear());
                    nb::Error::Other(Error::Noise)
                } else if isr.ore().bit_is_set() {
                    usart.icr().write(|w| w.orecf().clear());
                    nb::Error::Other(Error::Overrun)
                } else if isr.rxne().bit_is_set() {
                    return Ok(());
                } else {
                    nb::Error::WouldBlock
                })
            }
        }

        impl Rx<$USARTX, DMA> {
            pub fn disable_dma(self) -> Rx<$USARTX, NoDMA> {
                // NOTE(unsafe) critical section prevents races
                interrupt::free(|_| {
                    self.usart.cr3().modify(|_, w| w.dmar().clear_bit());
                });

                Rx {
                    pin: self.pin,
                    usart: self.usart,
                    _dma: PhantomData,
                }
            }
        }

        impl embedded_hal_old::serial::Read<u8> for Rx<$USARTX, NoDMA> {
            type Error = Error;

            fn read(&mut self) -> nb::Result<u8, Error> {
                self.data_ready()
                    .map(|_| self.usart.rdr().read().bits() as u8)
            }
        }

        impl<Otype> embedded_hal_old::serial::Read<u8> for Serial<$USARTX, Otype> {
            type Error = Error;

            fn read(&mut self) -> nb::Result<u8, Error> {
                self.rx.read()
            }
        }

        impl<Dma, Otype> Tx<$USARTX, Dma, Otype> {
            /// Starts listening for an interrupt event
            pub fn listen(&mut self) {
                self.usart.cr1().modify(|_, w| w.txeie().set_bit());
            }

            /// Stop listening for an interrupt event
            pub fn unlisten(&mut self) {
                self.usart.cr1().modify(|_, w| w.txeie().clear_bit());
            }

            /// Return true if the tx register is empty (and can accept data)
            pub fn is_txe(&self) -> bool {
                self.usart.isr().read().txe().bit_is_set()
            }

            /// Returns true if the tx fifo threshold has been reached.
            pub fn fifo_threshold_reached(&self) -> bool {
                self.usart.isr().read().txft().bit_is_set()
            }
        }

        impl<Otype> Tx<$USARTX, NoDMA, Otype> {
            pub fn enable_dma(self) -> Tx<$USARTX, DMA, Otype> {
                // NOTE(unsafe) critical section prevents races
                interrupt::free(|_| {
                    self.usart.cr3().modify(|_, w| w.dmat().set_bit());
                });

                Tx {
                    pin: self.pin,
                    usart: self.usart,
                    _dma: PhantomData,
                }
            }
        }

        impl<Otype> Tx<$USARTX, DMA, Otype> {
            pub fn disable_dma(self) -> Tx<$USARTX, NoDMA, Otype> {
                // NOTE(unsafe) critical section prevents races
                interrupt::free(|_| {
                    self.usart.cr3().modify(|_, w| w.dmat().clear_bit());
                });

                Tx {
                    pin: self.pin,
                    usart: self.usart,
                    _dma: PhantomData,
                }
            }
        }

        impl<Otype> embedded_hal_old::serial::Write<u8> for Tx<$USARTX, NoDMA, Otype> {
            type Error = Error;

            fn flush(&mut self) -> nb::Result<(), Self::Error> {
                if self.usart.isr().read().tc().bit_is_set() {
                    Ok(())
                } else {
                    Err(nb::Error::WouldBlock)
                }
            }

            fn write(&mut self, byte: u8) -> nb::Result<(), Self::Error> {
                if self.usart.isr().read().txe().bit_is_set() {
                    self.usart.tdr().write(|w| unsafe { w.bits(byte as u32) });
                    Ok(())
                } else {
                    Err(nb::Error::WouldBlock)
                }
            }
        }

        impl<Otype> embedded_hal_old::serial::Write<u8> for Serial<$USARTX, Otype> {
            type Error = Error;

            fn flush(&mut self) -> nb::Result<(), Self::Error> {
                self.tx.flush()
            }

            fn write(&mut self, byte: u8) -> nb::Result<(), Self::Error> {
                self.tx.write(byte)
            }
        }

        impl<Otype> embedded_io::ErrorType for Tx<$USARTX, NoDMA, Otype> {
            type Error = Error;
        }
        impl<Otype> WriteReady for Tx<$USARTX, NoDMA, Otype> {
            fn write_ready(&mut self) -> Result<bool, Self::Error> {
                Ok(self.usart.isr().read().txe().bit_is_set())
            }
        }
        // writes until fifo (or tdr) is full
        impl<Otype> embedded_io::Write for Tx<$USARTX, NoDMA, Otype> {
            fn write(&mut self, buf: &[u8]) -> Result<usize, Self::Error> {
                if buf.len() == 0 {
                    return Ok(0);
                }
                while !self.write_ready()? {
                    core::hint::spin_loop()
                }
                // can't know fifo capacity in advance
                let count = buf
                    .into_iter()
                    .take_while(|_| self.usart.isr().read().txe().bit_is_set())
                    .map(|b| {
                        self.usart
                            .tdr()
                            .write(|w| unsafe { w.tdr().bits(*b as u16) })
                    })
                    .count();

                Ok(count)
            }
            fn flush(&mut self) -> Result<(), Error> {
                nb::block!(embedded_hal_old::serial::Write::<u8>::flush(self))
            }
        }

        impl embedded_io::ErrorType for Rx<$USARTX, NoDMA> {
            type Error = Error;
        }
        impl ReadReady for Rx<$USARTX, NoDMA> {
            fn read_ready(&mut self) -> Result<bool, Self::Error> {
                match self.data_ready() {
                    Ok(()) => Ok(true),
                    Err(nb::Error::WouldBlock) => Ok(false),
                    Err(nb::Error::Other(e)) => Err(e),
                }
            }
        }
        impl embedded_io::Read for Rx<$USARTX, NoDMA> {
            fn read(&mut self, buf: &mut [u8]) -> Result<usize, Self::Error> {
                if buf.len() == 0 {
                    return Ok(0);
                }
                let mut count = 0;

                while !self.read_ready()? {
                    core::hint::spin_loop()
                }
                while self.read_ready()? && count < buf.len() {
                    buf[count] = self.usart.rdr().read().bits() as u8;
                    count += 1
                }
                Ok(count)
            }
        }

        impl<Otype> embedded_io::ErrorType for Serial<$USARTX, Otype> {
            type Error = Error;
        }
        impl<Otype> WriteReady for Serial<$USARTX, Otype> {
            fn write_ready(&mut self) -> Result<bool, Self::Error> {
                self.tx.write_ready()
            }
        }
        impl<Otype> embedded_io::Write for Serial<$USARTX, Otype> {
            fn write(&mut self, buf: &[u8]) -> Result<usize, Self::Error> {
                embedded_io::Write::write(&mut self.tx, buf)
            }
            fn flush(&mut self) -> Result<(), Error> {
                embedded_io::Write::flush(&mut self.tx)
            }
        }
        impl<Otype> ReadReady for Serial<$USARTX, Otype> {
            fn read_ready(&mut self) -> Result<bool, Self::Error> {
                self.rx.read_ready()
            }
        }
        impl<Otype> embedded_io::Read for Serial<$USARTX, Otype> {
            fn read(&mut self, buf: &mut [u8]) -> Result<usize, Self::Error> {
                embedded_io::Read::read(&mut self.rx, buf)
            }
        }

        impl<Otype> Serial<$USARTX, Otype> {
            /// Separates the serial struct into separate channel objects for sending (Tx) and
            /// receiving (Rx)
            pub fn split(self) -> (Tx<$USARTX, NoDMA, Otype>, Rx<$USARTX, NoDMA>) {
                (self.tx, self.rx)
            }

            /// Joins the objects created by `split()` back into one Serial object.
            ///
            /// This function can be used in combination with `release()` to deinitialize the
            /// peripheral after it has been split.
            pub fn join(tx: Tx<$USARTX, NoDMA, Otype>, rx: Rx<$USARTX, NoDMA>) -> Self {
                Serial { tx, rx }
            }

            /// Disables the USART and returns the peripheral as well the pins.
            ///
            /// This function makes the components available for further use. For example, the
            /// USART can later be reinitialized with a different baud rate or other configuration
            /// changes.
            pub fn release(
                self,
            ) -> (
                $USARTX,
                (
                    Option<<$USARTX as CommonPins>::Tx<Otype>>,
                    Option<<$USARTX as CommonPins>::Rx<PushPull>>,
                ),
            ) {
                // Disable the UART as well as its clock.
                self.tx.usart.cr1().modify(|_, w| w.ue().clear_bit());
                unsafe {
                    $USARTX::disable_unchecked();
                }
                (self.tx.usart, (self.tx.pin, self.rx.pin))
            }
        }

        unsafe impl<Otype> TargetAddress<MemoryToPeripheral> for Tx<$USARTX, DMA, Otype> {
            #[inline(always)]
            fn address(&self) -> u32 {
                // unsafe: only the Tx part accesses the Tx register
                unsafe { &*<$USARTX>::ptr() }.tdr() as *const _ as u32
            }

            type MemSize = u8;

            const REQUEST_LINE: Option<u8> = Some(DmaMuxResources::$dmamux_tx as u8);
        }

        unsafe impl TargetAddress<PeripheralToMemory> for Rx<$USARTX, DMA> {
            #[inline(always)]
            fn address(&self) -> u32 {
                // unsafe: only the Rx part accesses the Rx register
                unsafe { &*<$USARTX>::ptr() }.rdr() as *const _ as u32
            }

            type MemSize = u8;

            const REQUEST_LINE: Option<u8> = Some(DmaMuxResources::$dmamux_rx as u8);
        }
    };
}

macro_rules! uart_lp {
    ($USARTX:ident,
        $usartX:ident, $clk_mul:expr
    ) => {
        impl SerialExt<LowPowerConfig> for $USARTX {
            fn usart<Otype>(
                self,
                pins: (impl Into<Self::Tx<Otype>>, impl Into<Self::Rx<PushPull>>),
                config: impl Into<LowPowerConfig>,
                rcc: &mut Rcc,
            ) -> Result<Serial<Self, Otype>, InvalidConfig> {
                Serial::$usartX(self, pins, config, rcc)
            }
            fn tx<Otype>(
                self,
                tx: impl Into<Self::Tx<Otype>>,
                config: impl Into<LowPowerConfig>,
                rcc: &mut Rcc,
            ) -> Result<Tx<Self, NoDMA, Otype>, InvalidConfig> {
                Serial::<Self, _>::_new(self, (Some(tx), None::<Self::Rx<PushPull>>), config, rcc)
                    .map(|s| s.split().0)
            }
            fn rx(
                self,
                rx: impl Into<Self::Rx<PushPull>>,
                config: impl Into<LowPowerConfig>,
                rcc: &mut Rcc,
            ) -> Result<Rx<Self, NoDMA>, InvalidConfig> {
                Serial::<Self, _>::_new(self, (None::<Self::Tx<PushPull>>, Some(rx)), config, rcc)
                    .map(|s| s.split().1)
            }
        }

        impl<Otype> Serial<$USARTX, Otype> {
            pub fn $usartX(
                usart: $USARTX,
                pins: (
                    impl Into<<$USARTX as CommonPins>::Tx<Otype>>,
                    impl Into<<$USARTX as CommonPins>::Rx<PushPull>>,
                ),
                config: impl Into<LowPowerConfig>,
                rcc: &mut Rcc,
            ) -> Result<Self, InvalidConfig> {
                Self::_new(usart, (Some(pins.0), Some(pins.1)), config, rcc)
            }
            fn _new(
                usart: $USARTX,
                pins: (
                    Option<impl Into<<$USARTX as CommonPins>::Tx<Otype>>>,
                    Option<impl Into<<$USARTX as CommonPins>::Rx<PushPull>>>,
                ),
                config: impl Into<LowPowerConfig>,
                rcc: &mut Rcc,
            ) -> Result<Self, InvalidConfig> {
                let config = config.into();

                // Enable clock for USART
                $USARTX::enable(rcc);
                $USARTX::reset(rcc);

                // TODO: By default, all UARTs are clocked from PCLK. We could modify RCC_CCIPR to
                // try SYSCLK if PCLK is not high enough. We could also select 8x oversampling
                // instead of 16x.

                let clk = <$USARTX as RccBus>::Bus::get_frequency(&rcc.clocks).raw() as u64;
                let bdr = config.baudrate.0 as u64;
                let div = ($clk_mul * clk) / bdr;
                if div < 16 {
                    // We need 16x oversampling.
                    return Err(InvalidConfig);
                }
                usart.brr().write(|w| unsafe { w.bits(div as u32) });
                // Reset the UART and disable it (UE=0)
                usart.cr1().reset();
                // Reset other registers to disable advanced USART features
                usart.cr2().reset();
                usart.cr3().reset();

                usart.cr2().write(|w| unsafe {
                    w.stop().bits(config.stopbits.bits());
                    w.swap().bit(config.swap);
                    w.txinv().bit(config.tx_invert);
                    w.rxinv().bit(config.rx_invert)
                });

                usart.cr3().write(|w| unsafe {
                    w.txftcfg().bits(config.tx_fifo_threshold.bits());
                    w.rxftcfg().bits(config.rx_fifo_threshold.bits());
                    w.txftie().bit(config.tx_fifo_interrupt);
                    w.rxftie().bit(config.rx_fifo_interrupt)
                });

                // Enable the UART and perform remaining configuration.
                usart.cr1().write(|w| {
                    w.ue().set_bit();
                    w.te().set_bit();
                    w.re().set_bit();
                    w.m1().bit(config.wordlength == WordLength::DataBits7);
                    w.m0().bit(config.wordlength == WordLength::DataBits9);
                    w.pce().bit(config.parity != Parity::ParityNone);
                    w.ps().bit(config.parity == Parity::ParityOdd);
                    w.fifoen().bit(config.fifo_enable)
                });

                Ok(Serial {
                    tx: Tx {
                        pin: pins.0.map(Into::into),
                        usart,
                        _dma: PhantomData,
                    },
                    rx: Rx {
                        pin: pins.1.map(Into::into),
                        usart: unsafe { $USARTX::steal() },
                        _dma: PhantomData,
                    },
                })
            }

            /// Starts listening for an interrupt event
            pub fn listen(&mut self, event: Event) {
                match event {
                    Event::Rxne => self.tx.usart.cr1().modify(|_, w| w.rxneie().set_bit()),
                    Event::Txe => self.tx.usart.cr1().modify(|_, w| w.txeie().set_bit()),
                    Event::Idle => self.tx.usart.cr1().modify(|_, w| w.idleie().set_bit()),
                    _ => unimplemented!(),
                };
            }

            /// Stop listening for an interrupt event
            pub fn unlisten(&mut self, event: Event) {
                match event {
                    Event::Rxne => self.tx.usart.cr1().modify(|_, w| w.rxneie().clear_bit()),
                    Event::Txe => self.tx.usart.cr1().modify(|_, w| w.txeie().clear_bit()),
                    Event::Idle => self.tx.usart.cr1().modify(|_, w| w.idleie().clear_bit()),
                    _ => unimplemented!(),
                };
            }

            /// Check if interrupt event is pending
            pub fn is_pending(&mut self, event: Event) -> bool {
                (self.tx.usart.isr().read().bits() & event.val()) != 0
            }

            /// Clear pending interrupt
            pub fn unpend(&mut self, event: Event) {
                // mask the allowed bits
                let mask: u32 = 0x123BFF;
                self.tx
                    .usart
                    .icr()
                    .write(|w| unsafe { w.bits(event.val() & mask) });
            }
        }
    };
}

macro_rules! uart_full {
    ($USARTX:ident,
        $usartX:ident
    ) => {
        impl SerialExt<FullConfig> for $USARTX {
            fn usart<Otype>(
                self,
                pins: (impl Into<Self::Tx<Otype>>, impl Into<Self::Rx<PushPull>>),
                config: impl Into<FullConfig>,
                rcc: &mut Rcc,
            ) -> Result<Serial<Self, Otype>, InvalidConfig> {
                Serial::$usartX(self, pins, config, rcc)
            }
            fn tx<Otype>(
                self,
                tx: impl Into<Self::Tx<Otype>>,
                config: impl Into<FullConfig>,
                rcc: &mut Rcc,
            ) -> Result<Tx<Self, NoDMA, Otype>, InvalidConfig> {
                Serial::<Self, _>::_new(self, (Some(tx), None::<Self::Rx<PushPull>>), config, rcc)
                    .map(|s| s.split().0)
            }
            fn rx(
                self,
                rx: impl Into<Self::Rx<PushPull>>,
                config: impl Into<FullConfig>,
                rcc: &mut Rcc,
            ) -> Result<Rx<Self, NoDMA>, InvalidConfig> {
                Serial::<Self, _>::_new(self, (None::<Self::Tx<PushPull>>, Some(rx)), config, rcc)
                    .map(|s| s.split().1)
            }
        }

        impl<Otype> Serial<$USARTX, Otype> {
            pub fn $usartX(
                usart: $USARTX,
                pins: (
                    impl Into<<$USARTX as CommonPins>::Tx<Otype>>,
                    impl Into<<$USARTX as CommonPins>::Rx<PushPull>>,
                ),
                config: impl Into<FullConfig>,
                rcc: &mut Rcc,
            ) -> Result<Self, InvalidConfig> {
                Self::_new(usart, (Some(pins.0), Some(pins.1)), config, rcc)
            }
            fn _new(
                usart: $USARTX,
                pins: (
                    Option<impl Into<<$USARTX as CommonPins>::Tx<Otype>>>,
                    Option<impl Into<<$USARTX as CommonPins>::Rx<PushPull>>>,
                ),
                config: impl Into<FullConfig>,
                rcc: &mut Rcc,
            ) -> Result<Self, InvalidConfig> {
                let config = config.into();

                // Enable clock for USART
                $USARTX::enable(rcc);
                $USARTX::reset(rcc);

                // TODO: By default, all UARTs are clocked from PCLK. We could modify RCC_CCIPR to
                // try SYSCLK if PCLK is not high enough. We could also select 8x oversampling
                // instead of 16x.

                let clk = <$USARTX as RccBus>::Bus::get_frequency(&rcc.clocks).raw() as u64;
                let bdr = config.baudrate.0 as u64;
                let clk_mul = 1;
                let div = (clk_mul * clk) / bdr;
                if div < 16 {
                    // We need 16x oversampling.
                    return Err(InvalidConfig);
                }
                usart.brr().write(|w| unsafe { w.bits(div as u32) });

                // Reset the UART and disable it (UE=0)
                usart.cr1().reset();
                usart.cr2().reset();
                usart.cr3().reset();

                usart.cr2().write(|w| unsafe {
                    w.stop().bits(config.stopbits.bits());
                    w.swap().bit(config.swap);
                    w.txinv().bit(config.tx_invert);
                    w.rxinv().bit(config.rx_invert)
                });

                if let Some(timeout) = config.receiver_timeout {
                    usart.cr1().write(|w| w.rtoie().set_bit());
                    usart.cr2().modify(|_, w| w.rtoen().set_bit());
                    usart.rtor().write(|w| unsafe { w.rto().bits(timeout) });
                }

                usart.cr3().write(|w| unsafe {
                    w.txftcfg().bits(config.tx_fifo_threshold.bits());
                    w.rxftcfg().bits(config.rx_fifo_threshold.bits());
                    w.txftie().bit(config.tx_fifo_interrupt);
                    w.rxftie().bit(config.rx_fifo_interrupt)
                });

                // Enable the UART and perform remaining configuration.
                usart.cr1().modify(|_, w| {
                    w.ue().set_bit();
                    w.te().set_bit();
                    w.re().set_bit();
                    w.m1().bit(config.wordlength == WordLength::DataBits7);
                    w.m0().bit(config.wordlength == WordLength::DataBits9);
                    w.pce().bit(config.parity != Parity::ParityNone);
                    w.ps().bit(config.parity == Parity::ParityOdd);
                    w.fifoen().bit(config.fifo_enable)
                });

                Ok(Serial {
                    tx: Tx {
                        pin: pins.0.map(Into::into),
                        usart,
                        _dma: PhantomData,
                    },
                    rx: Rx {
                        pin: pins.1.map(Into::into),
                        usart: unsafe { $USARTX::steal() },
                        _dma: PhantomData,
                    },
                })
            }

            /// Starts listening for an interrupt event
            pub fn listen(&mut self, event: Event) {
                match event {
                    Event::Rxne => self.tx.usart.cr1().modify(|_, w| w.rxneie().set_bit()),
                    Event::Txe => self.tx.usart.cr1().modify(|_, w| w.txeie().set_bit()),
                    Event::Idle => self.tx.usart.cr1().modify(|_, w| w.idleie().set_bit()),
                    _ => unimplemented!(),
                };
            }

            /// Stop listening for an interrupt event
            pub fn unlisten(&mut self, event: Event) {
                match event {
                    Event::Rxne => self.tx.usart.cr1().modify(|_, w| w.rxneie().clear_bit()),
                    Event::Txe => self.tx.usart.cr1().modify(|_, w| w.txeie().clear_bit()),
                    Event::Idle => self.tx.usart.cr1().modify(|_, w| w.idleie().clear_bit()),
                    _ => unimplemented!(),
                };
            }

            /// Check if interrupt event is pending
            pub fn is_pending(&mut self, event: Event) -> bool {
                (self.tx.usart.isr().read().bits() & event.val()) != 0
            }

            /// Clear pending interrupt
            pub fn unpend(&mut self, event: Event) {
                // mask the allowed bits
                let mask: u32 = 0x123BFF;
                self.tx
                    .usart
                    .icr()
                    .write(|w| unsafe { w.bits(event.val() & mask) });
            }
        }

        impl<Dma> Rx<$USARTX, Dma> {
            /// Check if receiver timeout has lapsed
            /// Returns the current state of the ISR RTOF bit
            pub fn timeout_lapsed(&self) -> bool {
                let usart = unsafe { &(*$USARTX::ptr()) };
                usart.isr().read().rtof().bit_is_set()
            }

            /// Clear pending receiver timeout interrupt
            pub fn clear_timeout(&mut self) {
                let usart = unsafe { &(*$USARTX::ptr()) };
                usart.icr().write(|w| w.rtocf().clear());
            }
        }
    };
}

uart_shared!(USART1, USART1_RX, USART1_TX);

uart_shared!(USART2, USART2_RX, USART2_TX);

uart_shared!(USART3, USART3_RX, USART3_TX);

uart_shared!(UART4, USART4_RX, USART4_TX);

#[cfg(not(any(feature = "stm32g431", feature = "stm32g441")))]
uart_shared!(UART5, USART5_RX, USART5_TX);

uart_shared!(LPUART1, LPUART1_RX, LPUART1_TX);

uart_full!(USART1, usart1);
uart_full!(USART2, usart2);
uart_full!(USART3, usart3);

uart_full!(UART4, uart4);
#[cfg(not(any(feature = "stm32g431", feature = "stm32g441")))]
uart_full!(UART5, uart5);

// LPUART Should be given its own implementation when it needs to be used with features not present on
// the basic feature set such as: Dual clock domain, FIFO or prescaler.
// Or when Synchronous mode is implemented for the basic feature set, since the LP feature set does not have support.
uart_lp!(LPUART1, lpuart1, 256);
