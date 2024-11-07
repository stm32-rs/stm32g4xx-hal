use core::convert::Infallible;
use core::fmt;
use core::marker::PhantomData;

use crate::dma::traits::Stream;
use crate::dma::{mux::DmaMuxResources, traits::TargetAddress, PeripheralToMemory, MemoryToPeripheral};
use crate::gpio::{gpioa::*, gpiob::*, gpioc::*, gpiod::*, gpioe::*, gpiog::*};
use crate::gpio::{Alternate, AlternateOD, AF12, AF5, AF7, AF8};
use crate::prelude::*;
use crate::rcc::{Enable, GetBusFreq, Rcc, RccBus, Reset};
use crate::stm32::*;

use cortex_m::interrupt;
use nb::block;

use crate::serial::config::*;
/// Serial error
#[derive(Debug)]
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

/// Serial receiver
pub struct Rx<USART, Pin, Dma> {
    pin: Pin,
    _usart: PhantomData<USART>,
    _dma: PhantomData<Dma>,
}

/// Serial transmitter
pub struct Tx<USART, Pin, Dma> {
    pin: Pin,
    usart: USART,
    dma: Dma,
}

/// Serial abstraction
pub struct Serial<USART, TXPin, RXPin> {
    tx: Tx<USART, TXPin, NoDMA>,
    rx: Rx<USART, RXPin, NoDMA>,
}

/// Serial TX pin
pub trait TxPin<USART> {}

/// Serial RX pin
pub trait RxPin<USART> {}

pub struct NoTx;

impl<USART> TxPin<USART> for NoTx {}

/// Type state for Tx/Rx, indicating operation without DMA
#[derive(Debug)]
pub struct NoDMA;
/// Type state for Tx/Rx, indicating configuration for DMA
#[derive(Debug)]
pub struct DMA<T, M> {
    stream: T,
    memory: M,
}

pub struct DMAOld;

pub trait SerialExt<USART, Config> {
    fn usart<TX, RX>(
        self,
        tx: TX,
        rx: RX,
        config: Config,
        rcc: &mut Rcc,
    ) -> Result<Serial<USART, TX, RX>, InvalidConfig>
    where
        TX: TxPin<USART>,
        RX: RxPin<USART>;
}

impl<USART, TX, RX> fmt::Write for Serial<USART, TX, RX>
where
    Serial<USART, TX, RX>: hal::serial::Write<u8>,
{
    fn write_str(&mut self, s: &str) -> fmt::Result {
        let _ = s.as_bytes().iter().map(|c| block!(self.write(*c))).last();
        Ok(())
    }
}

impl<USART, Pin> fmt::Write for Tx<USART, Pin, NoDMA>
where
    Tx<USART, Pin, NoDMA>: hal::serial::Write<u8>,
{
    fn write_str(&mut self, s: &str) -> fmt::Result {
        let _ = s.as_bytes().iter().map(|c| block!(self.write(*c))).last();
        Ok(())
    }
}

impl<USART, Pin, T, M> embedded_io::ErrorType for Tx<USART, Pin, DMA<T, M>> {
    type Error = Infallible;
}

macro_rules! uart_shared {
    ($USARTX:ident, $dmamux_rx:ident, $dmamux_tx:ident,
        tx: [ $($( #[ $pmeta1:meta ] )* ($PTX:ident, $TAF:expr),)+ ],
        rx: [ $($( #[ $pmeta2:meta ] )* ($PRX:ident, $RAF:expr),)+ ]) => {

        $(
            $( #[ $pmeta1 ] )*
            impl TxPin<$USARTX> for $PTX<Alternate<$TAF>> {
            }
            impl TxPin<$USARTX> for $PTX<AlternateOD<$TAF>> {
            }
        )+

        $(
            $( #[ $pmeta2 ] )*
            impl RxPin<$USARTX> for $PRX<Alternate<$RAF>> {
            }
        )+

        impl<Pin, Dma> Rx<$USARTX, Pin, Dma> {
            /// Starts listening for an interrupt event
            pub fn listen(&mut self) {
                let usart = unsafe { &(*$USARTX::ptr()) };
                usart.cr1().modify(|_, w| w.rxneie().set_bit());
            }

            /// Stop listening for an interrupt event
            pub fn unlisten(&mut self) {
                let usart = unsafe { &(*$USARTX::ptr()) };
                usart.cr1().modify(|_, w| w.rxneie().clear_bit());
            }

            /// Return true if the rx register is not empty (and can be read)
            pub fn is_rxne(&self) -> bool {
                let usart = unsafe { &(*$USARTX::ptr()) };
                usart.isr().read().rxne().bit_is_set()
            }

            /// Returns true if the rx fifo threshold has been reached.
            pub fn fifo_threshold_reached(&self) -> bool {
                let usart = unsafe { &(*$USARTX::ptr()) };
                usart.isr().read().rxft().bit_is_set()
            }
        }

        impl<Pin> Rx<$USARTX, Pin, NoDMA> {
            pub fn enable_dma(self) -> Rx<$USARTX, Pin, DMAOld> {
                // NOTE(unsafe) critical section prevents races
                cortex_m::interrupt::free(|_| unsafe {
                    let cr3 = &(*$USARTX::ptr()).cr3();
                    cr3.modify(|_, w| w.dmar().set_bit());
                });

                Rx {
                    pin: self.pin,
                    _usart: PhantomData,
                    _dma: PhantomData,
                }
            }
        }

        impl<Pin> Rx<$USARTX, Pin, DMAOld> {
            pub fn disable_dma(self) -> Rx<$USARTX, Pin, NoDMA> {
                // NOTE(unsafe) critical section prevents races
                interrupt::free(|_| unsafe {
                    let cr3 = &(*$USARTX::ptr()).cr3();
                    cr3.modify(|_, w| w.dmar().clear_bit());
                });

                Rx {
                    pin: self.pin,
                    _usart: PhantomData,
                    _dma: PhantomData,
                }
            }
        }

        impl<Pin> hal::serial::Read<u8> for Rx<$USARTX, Pin, NoDMA> {
            type Error = Error;

            fn read(&mut self) -> nb::Result<u8, Error> {
                let usart = unsafe { &(*$USARTX::ptr()) };
                let isr = usart.isr().read();
                Err(
                    if isr.pe().bit_is_set() {
                        usart.icr().write(|w| w.pecf().set_bit());
                        nb::Error::Other(Error::Parity)
                    } else if isr.fe().bit_is_set() {
                        usart.icr().write(|w| w.fecf().set_bit());
                        nb::Error::Other(Error::Framing)
                    } else if isr.nf().bit_is_set() {
                        usart.icr().write(|w| w.ncf().set_bit());
                        nb::Error::Other(Error::Noise)
                    } else if isr.ore().bit_is_set() {
                        usart.icr().write(|w| w.orecf().set_bit());
                        nb::Error::Other(Error::Overrun)
                    } else if isr.rxne().bit_is_set() {
                        return Ok(usart.rdr().read().bits() as u8)
                    } else {
                        nb::Error::WouldBlock
                    }
                )
            }
        }

        impl<TX, RX> hal::serial::Read<u8> for Serial<$USARTX, TX, RX> {
            type Error = Error;

            fn read(&mut self) -> nb::Result<u8, Error> {
                self.rx.read()
            }
        }

        impl<Pin, Dma> Tx<$USARTX, Pin, Dma> {
            /// Starts listening for an interrupt event
            pub fn listen(&mut self) {
                let usart = unsafe { &(*$USARTX::ptr()) };
                usart.cr1().modify(|_, w| w.txeie().set_bit());
            }

            /// Stop listening for an interrupt event
            pub fn unlisten(&mut self) {
                let usart = unsafe { &(*$USARTX::ptr()) };
                usart.cr1().modify(|_, w| w.txeie().clear_bit());
            }

            /// Return true if the tx register is empty (and can accept data)
            pub fn is_txe(&self) -> bool {
                let usart = unsafe { &(*$USARTX::ptr()) };
                usart.isr().read().txe().bit_is_set()
            }

            /// Returns true if the tx fifo threshold has been reached.
            pub fn fifo_threshold_reached(&self) -> bool {
                let usart = unsafe { &(*$USARTX::ptr()) };
                usart.isr().read().txft().bit_is_set()
            }

            pub fn clear_transmission_complete(&mut self) {
                let usart = unsafe { &(*$USARTX::ptr()) };
                usart.icr().write(|w| w.tccf().set_bit());
            }
        }

        impl<Pin> Tx<$USARTX, Pin, NoDMA> {
            pub fn enable_dma<STREAM, BUF>(self, mut stream: STREAM, memory: BUF) -> Tx<$USARTX, Pin, DMA<STREAM, BUF>>
            where
                STREAM: crate::dma::traits::Stream<Config = crate::dma::config::DmaConfig>,
                BUF: embedded_dma::StaticReadBuffer<Word = u8>
            {
                // NOTE(unsafe) critical section prevents races
                interrupt::free(|_| unsafe {
                    let cr3 = &(*$USARTX::ptr()).cr3();
                    cr3.modify(|_, w| w.dmat().set_bit());
                });

                stream.disable();

                core::sync::atomic::fence(core::sync::atomic::Ordering::SeqCst);

                // Set peripheral to memory mode
                stream.set_direction(crate::dma::DmaDirection::MemoryToPeripheral);

                // NOTE(unsafe) We now own this buffer and we won't call any &mut
                // methods on it until the end of the DMA transfer
                let (buf_ptr, buf_len) = unsafe { memory.read_buffer() };

                // Set the memory address
                //
                // # Safety
                //
                // Must be a valid memory address
                unsafe {
                    stream.set_memory_address(
                        buf_ptr as u32,
                    );
                }

                // Set the peripheral address or the source address in memory-to-memory mode
                //
                // # Safety
                //
                // Must be a valid peripheral address or source address
                let write_register_address = &unsafe { &*<$USARTX>::ptr() }.tdr() as *const _ as u32;
                unsafe { stream.set_peripheral_address(write_register_address); }

                assert!(
                    buf_len <= 65535,
                    "Hardware does not support more than 65535 transfers"
                );

                // Set the DMAMUX request line
                stream.set_request_line(DmaMuxResources::$dmamux_tx as u8);

                let msize = core::mem::size_of::<u8>() / 2;

                stream.clear_interrupts();

                // NOTE(unsafe) These values are correct because of the
                // invariants of TargetAddress
                unsafe {
                    stream.set_memory_size(msize as u8);
                    stream.set_peripheral_size(msize as u8);
                }

                let config = crate::dma::config::DmaConfig::default()
                    .transfer_complete_interrupt(false)
                    .circular_buffer(false)
                    .memory_increment(true)
                    .priority(crate::dma::config::Priority::Low);
                stream.apply_config(config);

                Tx {
                    pin: self.pin,
                    usart: self.usart,
                    dma: DMA{ stream, memory },
                }
            }
        }

        impl<Pin, T, M> embedded_io::Write for Tx<$USARTX, Pin, DMA<T, M>>
        where
            T: Stream,
            M: embedded_dma::StaticReadBuffer + core::ops::DerefMut,
            <M as core::ops::Deref>::Target: core::ops::IndexMut<core::ops::Range<usize>, Output = [u8]>,
        {
            fn write(&mut self, buf: &[u8]) -> Result<usize, Self::Error> {
                use core::sync::atomic::fence;

                let (_p, buffer_capacity) = unsafe { self.dma.memory.read_buffer() };
                let count = buf.len().min(buffer_capacity);

                embedded_io::Write::flush(self)?;

                fence(core::sync::atomic::Ordering::SeqCst);
                self.dma.stream.disable();

                fence(core::sync::atomic::Ordering::SeqCst);

                self.dma.memory[0..count].copy_from_slice(&buf[0..count]);
                self.dma.stream.set_number_of_transfers(count as u16);

                fence(core::sync::atomic::Ordering::SeqCst);

                unsafe {
                    self.clear_transmission_complete();
                    self.dma.stream.enable();
                }

                Ok(count)
            }

            fn flush(&mut self) -> Result<(), Self::Error> {
                while T::get_number_of_transfers() > 0 {
                    assert!(!T::get_transfer_error_flag());
                }
                self.dma.stream.clear_transfer_complete_interrupt();
                assert!(!T::get_transfer_error_flag());
                Ok(())
            }
        }

        impl<Pin> hal::serial::Write<u8> for Tx<$USARTX, Pin, NoDMA> {
            type Error = Error;

            fn flush(&mut self) -> nb::Result<(), Self::Error> {
                let usart = unsafe { &(*$USARTX::ptr()) };
                if usart.isr().read().tc().bit_is_set() {
                    Ok(())
                } else {
                    Err(nb::Error::WouldBlock)
                }
            }

            fn write(&mut self, byte: u8) -> nb::Result<(), Self::Error> {
                let usart = unsafe { &(*$USARTX::ptr()) };
                if usart.isr().read().txe().bit_is_set() {
                    usart.tdr().write(|w| unsafe { w.bits(byte as u32) });
                    Ok(())
                } else {
                    Err(nb::Error::WouldBlock)
                }
            }
        }

        impl<TX, RX> hal::serial::Write<u8> for Serial<$USARTX, TX, RX> {
            type Error = Error;

            fn flush(&mut self) -> nb::Result<(), Self::Error> {
                self.tx.flush()
            }

            fn write(&mut self, byte: u8) -> nb::Result<(), Self::Error> {
                self.tx.write(byte)
            }
        }


        impl<TX, RX> Serial<$USARTX, TX, RX> {

            /// Separates the serial struct into separate channel objects for sending (Tx) and
            /// receiving (Rx)
            pub fn split(self) -> (Tx<$USARTX, TX, NoDMA>, Rx<$USARTX, RX, NoDMA>) {
                (self.tx, self.rx)
            }

            /// Joins the objects created by `split()` back into one Serial object.
            ///
            /// This function can be used in combination with `release()` to deinitialize the
            /// peripheral after it has been split.
            pub fn join(
                tx: Tx<$USARTX, TX, NoDMA>,
                rx: Rx<$USARTX, RX, NoDMA>,
            ) -> Self {
                Serial{
                    tx,
                    rx,
                }
            }

            /// Disables the USART and returns the peripheral as well the pins.
            ///
            /// This function makes the components available for further use. For example, the
            /// USART can later be reinitialized with a different baud rate or other configuration
            /// changes.
            pub fn release(self) -> ($USARTX, TX, RX) {
                // Disable the UART as well as its clock.
                self.tx.usart.cr1().modify(|_, w| w.ue().clear_bit());
                unsafe {
                    let rcc_ptr = &(*RCC::ptr());
                    $USARTX::disable(rcc_ptr);
                }
                (self.tx.usart, self.tx.pin, self.rx.pin)
            }
        }

        unsafe impl<Pin> TargetAddress<MemoryToPeripheral> for Tx<$USARTX, Pin, DMAOld> {
            #[inline(always)]
            fn address(&self) -> u32 {
                // unsafe: only the Tx part accesses the Tx register
                &unsafe { &*<$USARTX>::ptr() }.tdr() as *const _ as u32
            }

            type MemSize = u8;

            const REQUEST_LINE: Option<u8> = Some(DmaMuxResources::$dmamux_tx as u8);
        }

        unsafe impl<Pin> TargetAddress<PeripheralToMemory> for Rx<$USARTX, Pin, DMAOld> {
            #[inline(always)]
            fn address(&self) -> u32 {
                // unsafe: only the Rx part accesses the Rx register
                &unsafe { &*<$USARTX>::ptr() }.rdr() as *const _ as u32
            }

            type MemSize = u8;

            const REQUEST_LINE: Option<u8> = Some(DmaMuxResources::$dmamux_rx as u8);
        }
    }
}

macro_rules! uart_lp {
    ($USARTX:ident,
        $usartX:ident, $clk_mul:expr
    ) => {
        impl SerialExt<$USARTX, LowPowerConfig> for $USARTX {
            fn usart<TX, RX>(
                self,
                tx: TX,
                rx: RX,
                config: LowPowerConfig,
                rcc: &mut Rcc,
            ) -> Result<Serial<$USARTX, TX, RX>, InvalidConfig>
            where
                TX: TxPin<$USARTX>,
                RX: RxPin<$USARTX>,
            {
                Serial::$usartX(self, tx, rx, config, rcc)
            }
        }

        impl<TX, RX> Serial<$USARTX, TX, RX>
        where
            TX: TxPin<$USARTX>,
            RX: RxPin<$USARTX>,
        {
            pub fn $usartX(
                usart: $USARTX,
                tx: TX,
                rx: RX,
                config: LowPowerConfig,
                rcc: &mut Rcc,
            ) -> Result<Self, InvalidConfig> {
                // Enable clock for USART
                unsafe {
                    let rcc_ptr = &(*RCC::ptr());
                    $USARTX::enable(rcc_ptr);
                    $USARTX::reset(rcc_ptr);
                }

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
                    w.stop()
                        .bits(config.stopbits.bits())
                        .swap()
                        .bit(config.swap)
                });

                usart.cr3().write(|w| unsafe {
                    w.txftcfg()
                        .bits(config.tx_fifo_threshold.bits())
                        .rxftcfg()
                        .bits(config.rx_fifo_threshold.bits())
                        .txftie()
                        .bit(config.tx_fifo_interrupt)
                        .rxftie()
                        .bit(config.rx_fifo_interrupt)
                });

                // Enable the UART and perform remaining configuration.
                usart.cr1().write(|w| {
                    w.ue()
                        .set_bit()
                        .te()
                        .set_bit()
                        .re()
                        .set_bit()
                        .m0()
                        .bit(config.wordlength == WordLength::DataBits9)
                        .m1()
                        .bit(config.wordlength == WordLength::DataBits7)
                        .pce()
                        .bit(config.parity != Parity::ParityNone)
                        .ps()
                        .bit(config.parity == Parity::ParityOdd)
                        .fifoen()
                        .bit(config.fifo_enable)
                });

                Ok(Serial {
                    tx: Tx {
                        pin: tx,
                        usart,
                        dma: NoDMA,
                    },
                    rx: Rx {
                        pin: rx,
                        _usart: PhantomData,
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
        impl SerialExt<$USARTX, FullConfig> for $USARTX {
            fn usart<TX, RX>(
                self,
                tx: TX,
                rx: RX,
                config: FullConfig,
                rcc: &mut Rcc,
            ) -> Result<Serial<$USARTX, TX, RX>, InvalidConfig>
            where
                TX: TxPin<$USARTX>,
                RX: RxPin<$USARTX>,
            {
                Serial::$usartX(self, tx, rx, config, rcc)
            }
        }

        impl<TX, RX> Serial<$USARTX, TX, RX>
        where
            TX: TxPin<$USARTX>,
            RX: RxPin<$USARTX>,
        {
            pub fn $usartX(
                usart: $USARTX,
                tx: TX,
                rx: RX,
                config: FullConfig,
                rcc: &mut Rcc,
            ) -> Result<Self, InvalidConfig> {
                // Enable clock for USART
                unsafe {
                    let rcc_ptr = &(*RCC::ptr());
                    $USARTX::enable(rcc_ptr);
                    $USARTX::reset(rcc_ptr);
                }

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
                    w.stop()
                        .bits(config.stopbits.bits())
                        .swap()
                        .bit(config.swap)
                });

                if let Some(timeout) = config.receiver_timeout {
                    usart.cr1().write(|w| w.rtoie().set_bit());
                    usart.cr2().modify(|_, w| w.rtoen().set_bit());
                    usart.rtor().write(|w| unsafe { w.rto().bits(timeout) });
                }

                usart.cr3().write(|w| unsafe {
                    w.txftcfg()
                        .bits(config.tx_fifo_threshold.bits())
                        .rxftcfg()
                        .bits(config.rx_fifo_threshold.bits())
                        .txftie()
                        .bit(config.tx_fifo_interrupt)
                        .rxftie()
                        .bit(config.rx_fifo_interrupt)
                });

                // Enable the UART and perform remaining configuration.
                usart.cr1().modify(|_, w| {
                    w.ue()
                        .set_bit()
                        .te()
                        .set_bit()
                        .re()
                        .set_bit()
                        .m0()
                        .bit(config.wordlength == WordLength::DataBits7)
                        .m1()
                        .bit(config.wordlength == WordLength::DataBits9)
                        .pce()
                        .bit(config.parity != Parity::ParityNone)
                        .ps()
                        .bit(config.parity == Parity::ParityOdd)
                        .fifoen()
                        .bit(config.fifo_enable)
                });

                Ok(Serial {
                    tx: Tx {
                        pin: tx,
                        usart,
                        dma: NoDMA,
                    },
                    rx: Rx {
                        pin: rx,
                        _usart: PhantomData,
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

        impl<Pin, Dma> Rx<$USARTX, Pin, Dma> {
            /// Check if receiver timeout has lapsed
            /// Returns the current state of the ISR RTOF bit
            pub fn timeout_lapsed(&self) -> bool {
                let usart = unsafe { &(*$USARTX::ptr()) };
                usart.isr().read().rtof().bit_is_set()
            }

            /// Clear pending receiver timeout interrupt
            pub fn clear_timeout(&mut self) {
                let usart = unsafe { &(*$USARTX::ptr()) };
                usart.icr().write(|w| w.rtocf().set_bit());
            }
        }
    };
}

uart_shared!(USART1, USART1_RX, USART1_TX,
tx: [
    (PA9, AF7),
    (PB6, AF7),
    (PC4, AF7),
    (PE0, AF7),
    #[cfg(any(feature = "stm32g471", feature = "stm32g473", feature = "stm32g474", feature = "stm32g483", feature = "stm32g484"))]
    (PG9, AF7),
],
rx: [
    (PA10, AF7),
    (PB7, AF7),
    (PC5, AF7),
    (PE1, AF7),
]);

uart_shared!(USART2, USART2_RX, USART2_TX,
    tx: [
        (PA2, AF7),
        (PA14, AF7),
        (PB3, AF7),
        (PD5, AF7),
    ],
    rx: [
        (PA3, AF7),
        (PA15, AF7),
        (PB4, AF7),
        (PD6, AF7),
    ]
);

uart_shared!(USART3, USART3_RX, USART3_TX,
    tx: [
        (PB9, AF7),
        (PB10, AF7),
        (PC10, AF7),
        (PD8, AF7),
    ],
    rx: [
        (PB8, AF7),
        (PB11, AF7),
        (PC11, AF7),
        (PD9, AF7),
        (PE15, AF7),
    ]
);

uart_shared!(UART4, USART4_RX, USART4_TX,
    tx: [
        (PC10, AF5),
    ],
    rx: [
        (PC11, AF5),
    ]
);

#[cfg(not(any(feature = "stm32g431", feature = "stm32g441")))]
uart_shared!(UART5, USART5_RX, USART5_TX,
    tx: [
        (PC12, AF5),
    ],
    rx: [
        (PD2, AF5),
    ]
);

uart_shared!(LPUART1, LPUART1_RX, LPUART1_TX,
    tx: [
        (PA2, AF12),
        (PB11, AF8),
        (PC1, AF8),
        #[cfg(any(feature = "stm32g471", feature = "stm32g473", feature = "stm32g474", feature = "stm32g483", feature = "stm32g484"))]
        (PG7, AF8),
    ],
    rx: [
        (PA3, AF12),
        (PB10, AF8),
        (PC0, AF8),
        #[cfg(any(feature = "stm32g471", feature = "stm32g473", feature = "stm32g474", feature = "stm32g483", feature = "stm32g484"))]
        (PG8, AF8),
    ]
);

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
