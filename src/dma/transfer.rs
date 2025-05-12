use crate::{
    adc,
    dma::{
        traits, Channel, Direction, DmaDirection, MemoryToMemory, MemoryToPeripheral,
        PeripheralToMemory, TargetAddress,
    },
};
use core::{
    marker::PhantomData,
    mem,
    ops::{Deref, Index, Range},
    ptr,
    sync::atomic::{fence, Ordering},
};
use embedded_dma::{StaticReadBuffer, StaticWriteBuffer};

use super::config::DmaConfig;

/// Marker type for a transfer with a mutable source and backed by a
pub struct MutTransfer;
/// Marker type for a transfer with a constant source and backed by a
pub struct ConstTransfer;

/// DMA Transfer.
pub struct Transfer<CHANNEL, PERIPHERAL, DIR, BUF, TXFRT>
where
    CHANNEL: Channel,
    PERIPHERAL: TargetAddress<DIR>,
    DIR: Direction,
{
    channel: CHANNEL,
    peripheral: PERIPHERAL,
    _direction: PhantomData<DIR>,
    _transfer_type: PhantomData<TXFRT>,
    buf: BUF,
}

macro_rules! transfer_def {
    ($Marker:ty, $init:ident, $Buffer:tt, $rw_buffer:ident $(, $mut:tt)*;
     $($constraint:stmt)*) => {
        impl<CHANNEL, PERIPHERAL, DIR, BUF>
            Transfer<CHANNEL, PERIPHERAL, DIR, BUF, $Marker>
        where
            CHANNEL: Channel,
            DIR: Direction,
            PERIPHERAL: TargetAddress<DIR>,
            BUF: $Buffer<Word = <PERIPHERAL as TargetAddress<DIR>>::MemSize>,
        {
            /// Configures the DMA source and destination and applies supplied
            /// configuration. In a memory to memory transfer, the `peripheral` argument
            /// is the source of the data.
            ///
            /// # Panics
            ///
            /// * When the transfer length is greater than (2^16 - 1)
            pub(crate) fn $init(
                mut channel: CHANNEL,
                peripheral: PERIPHERAL,
                $($mut)* memory: BUF,
                config: DmaConfig,
            ) -> Self {
                channel.disable();

                fence(Ordering::SeqCst);

                // Used in the case that we can constant `memory`
                $($constraint)*

                // Set peripheral to memory mode
                channel.set_direction(DIR::direction());

                // NOTE(unsafe) We now own this buffer and we won't call any &mut
                // methods on it until the end of the DMA transfer
                let (buf_ptr, buf_len) = unsafe { memory.$rw_buffer() };

                // Set the memory address
                //
                // # Safety
                //
                // Must be a valid memory address
                unsafe {
                    channel.set_memory_address(
                        buf_ptr as u32,
                    );
                }

                // Set the peripheral address or the source address in memory-to-memory mode
                //
                // # Safety
                //
                // Must be a valid peripheral address or source address
                unsafe {
                    channel.set_peripheral_address(peripheral.address());
                }

                assert!(
                    buf_len <= 65535,
                    "Hardware does not support more than 65535 transfers"
                );
                let buf_len = buf_len as u16;
                channel.set_number_of_transfers(buf_len);

                // Set the DMAMUX request line if needed
                if let Some(request_line) = PERIPHERAL::REQUEST_LINE {
                    channel.set_request_line(request_line);
                }

                let mut transfer = Self {
                    channel,
                    peripheral,
                    _direction: PhantomData,
                    _transfer_type: PhantomData,
                    buf: memory,
                };
                transfer.apply_config(config);

                transfer
            }

            /// Starts the transfer, the closure will be executed right after enabling
            /// the channel.
            #[inline(always)]
            pub fn restart<F>(&mut self, f: F)
            where
                F: FnOnce(&mut PERIPHERAL),
            {
                self.channel.disable();
                fence(Ordering::SeqCst);

                let (_, buf_len) = unsafe { self.buf.$rw_buffer() };
                self.channel.set_number_of_transfers(buf_len as u16);
                f(&mut self.peripheral);

                // Preserve the instruction and bus ordering of preceding buffer access
                // to the subsequent access by the DMA peripheral due to enabling it.
                fence(Ordering::SeqCst);
                unsafe {
                    self.channel.enable();
                }
            }
        }
    };
}

transfer_def!(MutTransfer, init, StaticWriteBuffer, write_buffer, mut;);
transfer_def!(ConstTransfer, init_const, StaticReadBuffer, read_buffer;
                 assert!(DIR::direction() != DmaDirection::PeripheralToMemory));

impl<CHANNEL, PERIPHERAL, DIR, BUF, TXFRT> Transfer<CHANNEL, PERIPHERAL, DIR, BUF, TXFRT>
where
    CHANNEL: Channel,
    DIR: Direction,
    PERIPHERAL: TargetAddress<DIR>,
{
    /// Applies all fields in DmaConfig.
    fn apply_config(&mut self, config: DmaConfig) {
        let msize = mem::size_of::<<PERIPHERAL as TargetAddress<DIR>>::MemSize>() / 2;

        self.channel.clear_interrupts();

        // NOTE(unsafe) These values are correct because of the
        // invariants of TargetAddress
        unsafe {
            self.channel.set_memory_size(msize as u8);
            self.channel.set_peripheral_size(msize as u8);
        }

        self.channel.apply_config(config);
    }

    pub fn peek_buffer<F, T>(&mut self, func: F) -> T
    where
        F: FnOnce(&BUF, usize) -> T,
    {
        // Single buffer mode
        self.channel.disable();

        // Protect the instruction sequence of preceding DMA disable/inactivity
        // verification/poisoning and subsequent (old completed) buffer content
        // access.
        // Cortex-M7: Also protect the corresponding data access sequence.
        // NOTE: The data cache also needs to be flushed (if enabled).
        fence(Ordering::SeqCst);

        // Check how many data in the transfer are remaining.
        let remaining_data = self.channel.get_number_of_transfers();

        let result = func(&self.buf, remaining_data as usize);

        // Acknowledge the TCIF.
        self.clear_transfer_complete_interrupt();

        // Protect the instruction sequence of preceding (new) buffer content access
        // and subsequent DMA enable/address update. See the matching fence() above.
        fence(Ordering::SeqCst);

        unsafe {
            self.channel.enable();
        }
        result
    }

    /// Starts the transfer, the closure will be executed right after enabling
    /// the channel.
    pub fn start<F>(&mut self, f: F)
    where
        F: FnOnce(&mut PERIPHERAL),
    {
        // Preserve the instruction and bus ordering of preceding buffer access
        // to the subsequent access by the DMA peripheral due to enabling it.
        fence(Ordering::SeqCst);

        unsafe {
            self.channel.enable();
        }
        f(&mut self.peripheral);
    }

    /// Pauses the dma channel, the closure will be executed right before
    /// disabling the channel.
    pub fn pause<F>(&mut self, f: F)
    where
        F: FnOnce(&mut PERIPHERAL),
    {
        f(&mut self.peripheral);
        fence(Ordering::SeqCst);
        self.channel.disable();
        fence(Ordering::SeqCst);
    }

    /// Stops the channel and returns the underlying resources.
    pub fn free(mut self) -> (CHANNEL, PERIPHERAL, BUF) {
        self.channel.disable();

        // Protect the instruction and bus sequence of the preceding disable and
        // the subsequent buffer access.
        fence(Ordering::SeqCst);

        self.channel.clear_interrupts();

        unsafe {
            let channel = ptr::read(&self.channel);
            let peripheral = ptr::read(&self.peripheral);
            let buf = ptr::read(&self.buf);
            mem::forget(self);
            (channel, peripheral, buf)
        }
    }

    /// Clear all interrupts for the DMA channel.
    #[inline(always)]
    pub fn clear_interrupts(&mut self) {
        self.channel.clear_interrupts();
    }

    /// Clear transfer complete interrupt (tcif) for the DMA channel.
    #[inline(always)]
    pub fn clear_transfer_complete_interrupt(&mut self) {
        self.channel.clear_transfer_complete_interrupt();
    }

    /// Clear transfer error interrupt (teif) for the DMA channel.
    #[inline(always)]
    pub fn clear_transfer_error_interrupt(&mut self) {
        self.channel.clear_transfer_error_interrupt();
    }

    #[inline(always)]
    pub fn get_transfer_complete_flag(&self) -> bool {
        self.channel.get_transfer_complete_flag()
    }

    #[inline(always)]
    pub fn get_transfer_error_flag(&self) -> bool {
        self.channel.get_transfer_error_flag()
    }

    /// Clear half transfer interrupt (htif) for the DMA channel.
    #[inline(always)]
    pub fn clear_half_transfer_interrupt(&mut self) {
        self.channel.clear_half_transfer_interrupt();
    }

    #[inline(always)]
    pub fn get_half_transfer_flag(&self) -> bool {
        self.channel.get_half_transfer_flag()
    }
}

impl<CHANNEL, PERIPHERAL, DIR, BUF, TXFRT> Drop for Transfer<CHANNEL, PERIPHERAL, DIR, BUF, TXFRT>
where
    CHANNEL: Channel,
    PERIPHERAL: TargetAddress<DIR>,
    DIR: Direction,
{
    fn drop(&mut self) {
        self.channel.disable();

        // Protect the instruction and bus sequence of the preceding disable and
        // the subsequent buffer access.
        fence(Ordering::SeqCst);
    }
}

/// Circular DMA Transfer.
pub struct CircTransfer<CHANNEL, PERIPHERAL, BUF>
where
    CHANNEL: Channel,
    PERIPHERAL: TargetAddress<PeripheralToMemory>,
{
    transfer: Transfer<CHANNEL, PERIPHERAL, PeripheralToMemory, BUF, MutTransfer>,
    r_pos: usize,
}

impl<CHANNEL, PERIPHERAL, BUF> CircTransfer<CHANNEL, PERIPHERAL, BUF>
where
    CHANNEL: Channel,
    BUF: StaticWriteBuffer + Deref,
    <BUF as Deref>::Target:
        Index<Range<usize>, Output = [<PERIPHERAL as TargetAddress<PeripheralToMemory>>::MemSize]>,
    PERIPHERAL: TargetAddress<PeripheralToMemory>,
    <PERIPHERAL as TargetAddress<PeripheralToMemory>>::MemSize: Copy,
{
    /// Return the number of elements available to read
    pub fn elements_available(&mut self) -> usize {
        let blen = unsafe { self.transfer.buf.static_write_buffer().1 };
        let ndtr = self.transfer.channel.get_number_of_transfers() as usize;
        let pos_at = self.r_pos;

        // the position the DMA would write to next
        let pos_to = blen - ndtr;

        if pos_at > pos_to {
            // the buffer wraps around
            blen + pos_to - pos_at
        } else {
            // the buffer does not wrap around
            pos_to - pos_at
        }
    }

    /// Read the same number of elements as the provided buffer can hold.
    /// Blocks until the number of elements are available.
    ///
    /// # Panic
    ///
    /// This function panics if it tries to red more elements than the
    /// buffer of the transfer itself can hold.
    // TODO: fix the above limitation...
    pub fn read_exact(
        &mut self,
        dat: &mut [<PERIPHERAL as TargetAddress<PeripheralToMemory>>::MemSize],
    ) -> usize {
        let blen = unsafe { self.transfer.buf.static_write_buffer().1 };
        let pos = self.r_pos;
        let read = dat.len();

        assert!(
            blen > read,
            "Trying to read more than the DMA provided buffer can hold!"
        );

        while self.elements_available() < read {}

        fence(Ordering::SeqCst);

        if pos + read <= blen {
            // the read operation does not wrap around the
            // circular buffer, perform a single read
            dat[0..read].copy_from_slice(&self.transfer.buf[pos..pos + read]);
            self.r_pos = pos + read;
            if self.r_pos >= blen {
                self.r_pos = 0;
            }
        } else {
            // the read operation wraps around the circular buffer,
            let left = blen - pos;
            // copy until the end of the buffer
            dat[0..left].copy_from_slice(&self.transfer.buf[pos..blen]);
            // copy from the beginning of the buffer until the amount to read
            dat[left..read].copy_from_slice(&self.transfer.buf[0..read - left]);
            self.r_pos = read - left;
        }

        fence(Ordering::SeqCst);

        // return the number of bytes read
        read
    }

    pub fn read_available<'a>(
        &mut self,
        data: &'a mut [<PERIPHERAL as TargetAddress<PeripheralToMemory>>::MemSize],
    ) -> &'a mut [<PERIPHERAL as TargetAddress<PeripheralToMemory>>::MemSize] {
        let blen = unsafe { self.transfer.buf.static_write_buffer().1 };
        let available = self.elements_available();
        let len = data.len().min(available).min(blen - 1);
        let result = &mut data[0..len];
        self.read_exact(result);

        result
    }

    /// Starts the transfer, the closure will be executed right after enabling
    /// the channel.
    pub fn start<F>(&mut self, f: F)
    where
        F: FnOnce(&mut PERIPHERAL),
    {
        self.transfer.start(f)
    }

    /// Pauses the dma channel, the closure will be executed right before
    /// disabling the channel.
    pub fn pause<F>(&mut self, f: F)
    where
        F: FnOnce(&mut PERIPHERAL),
    {
        self.transfer.pause(f)
    }

    /// Stops the channel and returns the underlying resources.
    pub fn free(self) -> (CHANNEL, PERIPHERAL, BUF) {
        self.transfer.free()
    }

    /// Clear all interrupts for the DMA channel.
    #[inline(always)]
    pub fn clear_interrupts(&mut self) {
        self.transfer.clear_interrupts();
    }

    /// Clear transfer complete interrupt (tcif) for the DMA channel.
    #[inline(always)]
    pub fn clear_transfer_complete_interrupt(&mut self) {
        self.transfer.clear_transfer_complete_interrupt();
    }

    /// Clear transfer error interrupt (teif) for the DMA channel.
    #[inline(always)]
    pub fn clear_transfer_error_interrupt(&mut self) {
        self.transfer.clear_transfer_error_interrupt();
    }

    #[inline(always)]
    pub fn get_transfer_complete_flag(&self) -> bool {
        self.transfer.get_transfer_complete_flag()
    }

    #[inline(always)]
    pub fn get_transfer_error_flag(&self) -> bool {
        self.transfer.get_transfer_error_flag()
    }

    /// Clear half transfer interrupt (htif) for the DMA channel.
    #[inline(always)]
    pub fn clear_half_transfer_interrupt(&mut self) {
        self.transfer.clear_half_transfer_interrupt();
    }

    #[inline(always)]
    pub fn get_half_transfer_flag(&self) -> bool {
        self.transfer.get_half_transfer_flag()
    }
}

impl<ADC: adc::Instance, CHANNEL, BUF>
    CircTransfer<CHANNEL, crate::adc::Adc<ADC, crate::adc::DMA>, BUF>
where
    CHANNEL: Channel,
    BUF: StaticWriteBuffer + Deref,
    <BUF as Deref>::Target: Index<Range<usize>, Output = [u16]>,
{
    /// This is set when the AD finishes a conversion before the DMA has hade time to transfer the previous value
    pub fn get_overrun_flag(&self) -> bool {
        self.transfer.peripheral.get_overrun_flag()
    }

    pub fn clear_overrun_flag(&mut self) {
        self.transfer.peripheral.clear_overrun_flag();
    }
}

macro_rules! impl_serial_timeout {
    ($($uart:ident, )*) => {$(
        impl<CHANNEL, BUF, Pin> CircTransfer<CHANNEL, crate::serial::Rx<crate::stm32::$uart, Pin, crate::serial::DMA>, BUF>
        where
            CHANNEL: Channel,
            /*BUF: StaticWriteBuffer + Deref*/ {
            pub fn timeout_lapsed(&self) -> bool {
                self.transfer.peripheral.timeout_lapsed()
            }

            pub fn clear_timeout(&mut self) {
                self.transfer.peripheral.clear_timeout();
            }
        }
    )*};
}

impl_serial_timeout!(USART1, USART2, USART3, UART4,);
#[cfg(not(any(feature = "stm32g431", feature = "stm32g441")))]
impl_serial_timeout!(UART5,);

pub trait TransferExt<CHANNEL>
where
    CHANNEL: traits::Channel,
{
    fn into_memory_to_memory_transfer<PERIPHERAL, BUF, T>(
        self,
        per: PERIPHERAL,
        buf: BUF,
        config: DmaConfig,
    ) -> Transfer<CHANNEL, PERIPHERAL, MemoryToMemory<T>, BUF, MutTransfer>
    where
        T: Into<u32>,
        PERIPHERAL: TargetAddress<MemoryToMemory<T>>,
        BUF: StaticWriteBuffer<Word = <PERIPHERAL as TargetAddress<MemoryToMemory<T>>>::MemSize>;
    fn into_peripheral_to_memory_transfer<PERIPHERAL, BUF>(
        self,
        per: PERIPHERAL,
        buf: BUF,
        config: DmaConfig,
    ) -> Transfer<CHANNEL, PERIPHERAL, PeripheralToMemory, BUF, MutTransfer>
    where
        PERIPHERAL: TargetAddress<PeripheralToMemory>,
        BUF: StaticWriteBuffer<Word = <PERIPHERAL as TargetAddress<PeripheralToMemory>>::MemSize>;
    fn into_memory_to_peripheral_transfer<PERIPHERAL, BUF>(
        self,
        per: PERIPHERAL,
        buf: BUF,
        config: DmaConfig,
    ) -> Transfer<CHANNEL, PERIPHERAL, MemoryToPeripheral, BUF, ConstTransfer>
    where
        PERIPHERAL: TargetAddress<MemoryToPeripheral>,
        BUF: StaticReadBuffer<Word = <PERIPHERAL as TargetAddress<MemoryToPeripheral>>::MemSize>;
    fn into_peripheral_to_peripheral_transfer<PSRC, PDST>(
        self,
        src: PSRC,
        dst: PDST,
        config: DmaConfig,
    ) -> Transfer<CHANNEL, PSRC, PeripheralToMemory, &'static mut [PDST::MemSize; 1], MutTransfer>
    where
        PSRC: TargetAddress<PeripheralToMemory>,
        PDST: TargetAddress<MemoryToPeripheral>,
        [<PDST as TargetAddress<MemoryToPeripheral>>::MemSize; 1]: embedded_dma::WriteTarget,
        &'static mut [<PDST as TargetAddress<MemoryToPeripheral>>::MemSize; 1]:
            StaticWriteBuffer<Word = <PSRC as TargetAddress<PeripheralToMemory>>::MemSize>;
    fn into_circ_peripheral_to_memory_transfer<PERIPHERAL, BUF>(
        self,
        per: PERIPHERAL,
        buf: BUF,
        config: DmaConfig,
    ) -> CircTransfer<CHANNEL, PERIPHERAL, BUF>
    where
        PERIPHERAL: TargetAddress<PeripheralToMemory>,
        <PERIPHERAL as TargetAddress<PeripheralToMemory>>::MemSize: Copy,
        BUF: StaticWriteBuffer<Word = <PERIPHERAL as TargetAddress<PeripheralToMemory>>::MemSize>,
        BUF: Deref,
        <BUF as Deref>::Target: Index<
            Range<usize>,
            Output = [<PERIPHERAL as TargetAddress<PeripheralToMemory>>::MemSize],
        >;
}

impl<DMA, const N: u8> TransferExt<Self> for crate::dma::channel::C<DMA, N>
where
    DMA: crate::dma::channel::Instance,
    Self: traits::Channel,
{
    fn into_memory_to_memory_transfer<PERIPHERAL, BUF, T>(
        self,
        per: PERIPHERAL,
        buf: BUF,
        mut config: DmaConfig,
    ) -> Transfer<Self, PERIPHERAL, MemoryToMemory<T>, BUF, MutTransfer>
    where
        T: Into<u32>,
        PERIPHERAL: TargetAddress<MemoryToMemory<T>>,
        BUF: StaticWriteBuffer<Word = <PERIPHERAL as TargetAddress<MemoryToMemory<T>>>::MemSize>,
    {
        config.circular_buffer = false;
        Transfer::init(self, per, buf, config)
    }
    fn into_peripheral_to_memory_transfer<PERIPHERAL, BUF>(
        self,
        per: PERIPHERAL,
        buf: BUF,
        config: DmaConfig,
    ) -> Transfer<Self, PERIPHERAL, PeripheralToMemory, BUF, MutTransfer>
    where
        PERIPHERAL: TargetAddress<PeripheralToMemory>,
        BUF: StaticWriteBuffer<Word = <PERIPHERAL as TargetAddress<PeripheralToMemory>>::MemSize>,
    {
        Transfer::init(self, per, buf, config)
    }
    fn into_memory_to_peripheral_transfer<PERIPHERAL, BUF>(
        self,
        per: PERIPHERAL,
        buf: BUF,
        config: DmaConfig,
    ) -> Transfer<Self, PERIPHERAL, MemoryToPeripheral, BUF, ConstTransfer>
    where
        PERIPHERAL: TargetAddress<MemoryToPeripheral>,
        BUF: StaticReadBuffer<Word = <PERIPHERAL as TargetAddress<MemoryToPeripheral>>::MemSize>,
    {
        Transfer::init_const(self, per, buf, config)
    }
    fn into_peripheral_to_peripheral_transfer<PSRC, PDST>(
        self,
        src: PSRC,
        dst: PDST,
        config: DmaConfig,
    ) -> Transfer<Self, PSRC, PeripheralToMemory, &'static mut [PDST::MemSize; 1], MutTransfer>
    where
        PSRC: TargetAddress<PeripheralToMemory>,
        PDST: TargetAddress<MemoryToPeripheral>,
        [<PDST as TargetAddress<MemoryToPeripheral>>::MemSize; 1]: embedded_dma::WriteTarget,
        &'static mut [<PDST as TargetAddress<MemoryToPeripheral>>::MemSize; 1]:
            StaticWriteBuffer<Word = <PSRC as TargetAddress<PeripheralToMemory>>::MemSize>,
    {
        let data_addr: u32 = dst.address();
        let ptr: &mut PDST::MemSize = unsafe { &mut *(data_addr as *mut _) };
        //TODO: check that this is correct; nightly has core::array::from_mut...
        let dst: &mut [PDST::MemSize; 1] = core::array::from_mut(ptr);
        Transfer::init(self, src, dst, config)
    }
    fn into_circ_peripheral_to_memory_transfer<PERIPHERAL, BUF>(
        self,
        per: PERIPHERAL,
        buf: BUF,
        config: DmaConfig,
    ) -> CircTransfer<Self, PERIPHERAL, BUF>
    where
        PERIPHERAL: TargetAddress<PeripheralToMemory>,
        <PERIPHERAL as TargetAddress<PeripheralToMemory>>::MemSize: Copy,
        BUF: StaticWriteBuffer<Word = <PERIPHERAL as TargetAddress<PeripheralToMemory>>::MemSize>,
        BUF: Deref,
        <BUF as Deref>::Target: Index<
            Range<usize>,
            Output = [<PERIPHERAL as TargetAddress<PeripheralToMemory>>::MemSize],
        >,
    {
        CircTransfer {
            transfer: Transfer::init(self, per, buf, config),
            r_pos: 0,
        }
    }
}
