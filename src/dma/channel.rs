//! DMA

use super::{
    config,
    traits::sealed::{Bits, Sealed},
    traits::*,
    DmaDirection,
};
use core::marker::PhantomData;

use crate::rcc::Rcc;
use crate::stm32;

use stm32::{DMA1, DMA2, DMAMUX};

use crate::dma::config::DmaConfig;
use core::ops::Deref;

impl Sealed for DMA1 {}
impl Sealed for DMA2 {}

/// Type aliases for register blocks
pub type DMARegisterBlock = stm32::dma1::RegisterBlock;
pub type DMAMUXRegisterBlock = stm32::dmamux::RegisterBlock;

/// Trait that represents an instance of a DMA peripheral
pub trait Instance: Deref<Target = DMARegisterBlock> + Sealed {
    const IS_DMA1: bool;

    /// Gives a pointer to the RegisterBlock.
    fn ptr() -> *const DMARegisterBlock;

    /// Gives a pointer to the DMAMUX used for this DMA.
    fn mux_ptr() -> *const DMAMUXRegisterBlock;
}

impl Instance for DMA1 {
    const IS_DMA1: bool = true;

    #[inline(always)]
    fn ptr() -> *const DMARegisterBlock {
        DMA1::ptr()
    }

    #[inline(always)]
    fn mux_ptr() -> *const DMAMUXRegisterBlock {
        DMAMUX::ptr()
    }
}

impl Instance for DMA2 {
    const IS_DMA1: bool = false;

    #[inline(always)]
    fn ptr() -> *const DMARegisterBlock {
        DMA2::ptr()
    }

    #[inline(always)]
    fn mux_ptr() -> *const DMAMUXRegisterBlock {
        DMAMUX::ptr()
    }
}

/// DMA interrupts
#[derive(Debug, Clone, Copy)]
pub struct DmaInterrupts {
    transfer_complete: bool,
    transfer_error: bool,
    half_transfer: bool,
}

/// Alias for a tuple with all DMA channels.
pub struct Channels<T> {
    pub ch1: C<T, 0>,
    pub ch2: C<T, 1>,
    pub ch3: C<T, 2>,
    pub ch4: C<T, 3>,
    pub ch5: C<T, 4>,
    pub ch6: C<T, 5>,
    #[cfg(not(any(feature = "stm32g431", feature = "stm32g441",)))]
    pub ch7: C<T, 6>,
    #[cfg(not(any(feature = "stm32g431", feature = "stm32g441",)))]
    pub ch8: C<T, 7>,
}

pub trait DMAExt<I> {
    fn split(self, rcc: &Rcc) -> Channels<I>;
}

impl DMAExt<Self> for DMA1 {
    fn split(self, rcc: &Rcc) -> Channels<DMA1> {
        // Enable DMAMux is not yet enabled
        if !rcc.rb.ahb1enr().read().dmamux1en().bit_is_set() {
            // Enable peripheral
            rcc.rb.ahb1enr().modify(|_, w| w.dmamux1en().set_bit());
        }

        // Enable peripheral
        rcc.rb.ahb1enr().modify(|_, w| w.dma1en().set_bit());

        Channels::new(self)
    }
}

impl DMAExt<Self> for DMA2 {
    fn split(self, rcc: &Rcc) -> Channels<DMA2> {
        // Enable DMAMux is not yet enabled
        if !rcc.rb.ahb1enr().read().dmamux1en().bit_is_set() {
            // Enable peripheral
            rcc.rb.ahb1enr().modify(|_, w| w.dmamux1en().set_bit());
        }

        // Enable peripheral
        rcc.rb.ahb1enr().modify(|_, w| w.dma2en().set_bit());

        Channels::new(self)
    }
}

impl<I: Instance> Channels<I> {
    /// Splits the DMA peripheral into channels.
    pub(crate) fn new(_regs: I) -> Self {
        Self {
            ch1: C { _dma: PhantomData },
            ch2: C { _dma: PhantomData },
            ch3: C { _dma: PhantomData },
            ch4: C { _dma: PhantomData },
            ch5: C { _dma: PhantomData },
            ch6: C { _dma: PhantomData },
            #[cfg(not(any(feature = "stm32g431", feature = "stm32g441",)))]
            ch7: C { _dma: PhantomData },
            #[cfg(not(any(feature = "stm32g431", feature = "stm32g441",)))]
            ch8: C { _dma: PhantomData },
        }
    }
}

/// Channel on DMA
///
/// Note that `C<DMA, 0>` here maps to Channel1
pub struct C<DMA, const N: u8> {
    _dma: PhantomData<DMA>,
}

impl<I: Instance, const N: u8> Sealed for C<I, N> {}

impl<I: Instance, const N: u8> C<I, N> {
    fn ch(&self) -> &stm32::dma1::CH {
        // NOTE(unsafe) grants access only to this channels registers
        unsafe { &*I::ptr() }.ch(N as usize)
    }
}

impl<I: Instance, const N: u8> Channel for C<I, N> {
    type Interrupts = DmaInterrupts;

    fn apply_config(&mut self, config: DmaConfig) {
        self.set_priority(config.priority);
        self.set_memory_increment(config.memory_increment);
        self.set_peripheral_increment(config.peripheral_increment);
        self.set_circular_buffer(config.circular_buffer);
        self.set_transfer_complete_interrupt_enable(config.transfer_complete_interrupt);
        self.set_half_transfer_interrupt_enable(config.half_transfer_interrupt);
        self.set_transfer_error_interrupt_enable(config.transfer_error_interrupt);
    }

    #[inline(always)]
    fn clear_interrupts(&mut self) {
        //NOTE(unsafe) Atomic write with no side-effects and we only access the bits
        // that belongs to the ChannelX
        let dma = unsafe { &*I::ptr() };
        dma.ifcr().write(
            |w| {
                w.ctcif(N)
                    .set_bit() //Clear transfer complete interrupt flag
                    .chtif(N)
                    .set_bit() //Clear half transfer interrupt flag
                    .cteif(N)
                    .set_bit() //Clear transfer error interrupt flag
                    .cgif(N)
                    .set_bit()
            }, //Clear global interrupt flag
        );
        let _ = dma.isr().read();
        let _ = dma.isr().read(); // Delay 2 peripheral clocks
    }

    #[inline(always)]
    fn clear_transfer_complete_flag(&mut self) {
        //NOTE(unsafe) Atomic write with no side-effects and we only access the bits
        // that belongs to the ChannelX
        let dma = unsafe { &*I::ptr() };
        dma.ifcr().write(|w| w.ctcif(N).set_bit());
    }

    #[inline(always)]
    fn clear_transfer_complete_interrupt(&mut self) {
        self.clear_transfer_complete_flag();
        //NOTE(unsafe) Atomic read with no side-effects.
        let dma = unsafe { &*I::ptr() };
        let _ = dma.isr().read();
        let _ = dma.isr().read(); // Delay 2 peripheral clocks
    }

    #[inline(always)]
    fn clear_transfer_error_interrupt(&mut self) {
        //NOTE(unsafe) Atomic write with no side-effects and we only access the bits
        // that belongs to the ChannelX
        let dma = unsafe { &*I::ptr() };
        dma.ifcr().write(|w| w.cteif(N).set_bit());
        let _ = dma.isr().read();
        let _ = dma.isr().read(); // Delay 2 peripheral clocks
    }

    #[inline(always)]
    fn get_transfer_complete_flag(&self) -> bool {
        //NOTE(unsafe) Atomic read with no side effects
        let dma = unsafe { &*I::ptr() };
        dma.isr().read().tcif(N).bit_is_set()
    }

    #[inline(always)]
    fn get_transfer_error_flag(&self) -> bool {
        //NOTE(unsafe) Atomic read with no side effects
        let dma = unsafe { &*I::ptr() };
        dma.isr().read().teif(N).bit_is_set()
    }

    #[inline(always)]
    unsafe fn enable(&mut self) {
        //NOTE(unsafe) We only access the registers that belongs to the ChannelX
        let dma_ch = &unsafe { &*I::ptr() }.ch(N as usize);
        dma_ch.cr().modify(|_, w| w.en().set_bit());
    }

    #[inline(always)]
    fn is_enabled(&self) -> bool {
        //NOTE(unsafe) Atomic read with no side effects
        let dma_ch = &unsafe { &*I::ptr() }.ch(N as usize);
        dma_ch.cr().read().en().bit_is_set()
    }

    fn disable(&mut self) {
        if self.is_enabled() {
            //NOTE(unsafe) We only access the registers that belongs to the ChannelX
            let dma_ch = &unsafe { &*I::ptr() }.ch(N as usize);

            // Aborting an on-going transfer might cause interrupts to fire, disable
            // them
            let interrupts = self.get_interrupts_enable();
            self.disable_interrupts();

            dma_ch.cr().modify(|_, w| w.en().clear_bit());
            while self.is_enabled() {}

            self.clear_interrupts();
            self.enable_interrupts(interrupts);
        }
    }

    #[inline(always)]
    fn set_request_line(&mut self, request_line: u8) {
        //NOTE(unsafe) We only access the registers that belongs to the ChannelX
        let dmamux = unsafe { &*I::mux_ptr() };
        #[cfg(feature = "cat2")]
        let channels_per_dma = 6;

        #[cfg(any(feature = "cat3", feature = "cat4"))]
        let channels_per_dma = 8;

        let mux_ch = if I::IS_DMA1 {
            // DMA1 ch 0..=(channels_per_dma-1) are dmamux ch 0..=channels_per_dma
            N as usize
        } else {
            // DMA2 ch 0..=ch(channels_per_dma-1) are dmamux ch channels_per_dma..=(2*channels_per_dma-1)
            N as usize + channels_per_dma
        };
        unsafe {
            dmamux
                .ccr(mux_ch)
                .modify(|_, w| w.dmareq_id().bits(request_line));
        }
    }

    #[inline(always)]
    fn set_priority(&mut self, priority: config::Priority) {
        //NOTE(unsafe) We only access the registers that belongs to the ChannelX
        let dma_ch = &self.ch();
        dma_ch
            .cr()
            .modify(|_, w| unsafe { w.pl().bits(priority.bits()) });
    }

    #[inline(always)]
    fn disable_interrupts(&mut self) {
        //NOTE(unsafe) We only access the registers that belongs to the ChannelX
        let dmacr = &self.ch().cr();
        dmacr.modify(|_, w| w.tcie().clear_bit().teie().clear_bit().htie().clear_bit());
        let _ = dmacr.read();
        let _ = dmacr.read(); // Delay 2 peripheral clocks
    }

    #[inline(always)]
    fn enable_interrupts(&mut self, interrupt: Self::Interrupts) {
        //NOTE(unsafe) We only access the registers that belongs to the ChannelX
        let dma_ch = &self.ch();
        dma_ch.cr().modify(|_, w| {
            w.tcie()
                .bit(interrupt.transfer_complete)
                .teie()
                .bit(interrupt.transfer_error)
                .htie()
                .bit(interrupt.half_transfer)
        });
    }

    #[inline(always)]
    fn get_interrupts_enable(&self) -> Self::Interrupts {
        //NOTE(unsafe) We only access the registers that belongs to the ChannelX
        let dma_ch = self.ch();
        let cr = dma_ch.cr().read();

        DmaInterrupts {
            transfer_complete: cr.tcie().bit_is_set(),
            half_transfer: cr.htie().bit_is_set(),
            transfer_error: cr.teie().bit_is_set(),
        }
    }

    #[inline(always)]
    fn set_transfer_complete_interrupt_enable(&mut self, transfer_complete_interrupt: bool) {
        //NOTE(unsafe) We only access the registers that belongs to the ChannelX
        let dmacr = &self.ch().cr();
        dmacr.modify(|_, w| w.tcie().bit(transfer_complete_interrupt));
        let _ = dmacr.read();
        let _ = dmacr.read(); // Delay 2 peripheral clocks
    }

    #[inline(always)]
    fn set_transfer_error_interrupt_enable(&mut self, transfer_error_interrupt: bool) {
        //NOTE(unsafe) We only access the registers that belongs to the ChannelX
        let dmacr = &self.ch().cr();
        dmacr.modify(|_, w| w.teie().bit(transfer_error_interrupt));
        let _ = dmacr.read();
        let _ = dmacr.read(); // Delay 2 peripheral clocks
    }

    #[inline(always)]
    fn set_half_transfer_interrupt_enable(&mut self, half_transfer_interrupt: bool) {
        //NOTE(unsafe) We only access the registers that belongs to the ChannelX
        let dmacr = &self.ch().cr();
        dmacr.modify(|_, w| w.htie().bit(half_transfer_interrupt));
        let _ = dmacr.read();
        let _ = dmacr.read(); // Delay 2 peripheral clocks
    }

    #[inline(always)]
    fn get_half_transfer_flag(&self) -> bool {
        //NOTE(unsafe) Atomic read with no side effects
        let dma = unsafe { &*I::ptr() };
        dma.isr().read().htif(N).bit_is_set()
    }

    #[inline(always)]
    fn clear_half_transfer_interrupt(&mut self) {
        //NOTE(unsafe) Atomic write with no side-effects and we only access the bits
        // that belongs to the ChannelX
        let dma = unsafe { &*I::ptr() };
        dma.ifcr().write(|w| w.chtif(N).set_bit());
        let _ = dma.isr().read();
        let _ = dma.isr().read(); // Delay 2 peripheral clocks
    }

    #[inline(always)]
    unsafe fn set_peripheral_address(&mut self, value: u32) {
        //NOTE(unsafe) We only access the registers that belongs to the ChannelX
        let dma_ch = self.ch();
        dma_ch.par().write(|w| w.pa().bits(value));
    }

    #[inline(always)]
    unsafe fn set_memory_address(&mut self, value: u32) {
        //NOTE(unsafe) We only access the registers that belongs to the ChannelX
        let dma_ch = self.ch();
        dma_ch.mar().write(|w| w.ma().bits(value));
    }

    #[inline(always)]
    fn get_memory_address(&self) -> u32 {
        //NOTE(unsafe) We only access the registers that belongs to the ChannelX
        let dma_ch = self.ch();
        dma_ch.mar().read().ma().bits()
    }

    #[inline(always)]
    fn set_number_of_transfers(&mut self, value: u16) {
        //NOTE(unsafe) We only access the registers that belongs to the ChannelX
        let dma_ch = self.ch();
        dma_ch.ndtr().write(|w| unsafe { w.ndt().bits(value) });
    }

    #[inline(always)]
    fn get_number_of_transfers(&self) -> u16 {
        //NOTE(unsafe) We only access the registers that belongs to the ChannelX
        let dma_ch = self.ch();
        dma_ch.ndtr().read().ndt().bits()
    }
    #[inline(always)]
    unsafe fn set_memory_size(&mut self, size: u8) {
        //NOTE(unsafe) We only access the registers that belongs to the ChannelX
        let dma_ch = self.ch();
        dma_ch.cr().modify(|_, w| w.msize().bits(size));
    }

    #[inline(always)]
    unsafe fn set_peripheral_size(&mut self, size: u8) {
        //NOTE(unsafe) We only access the registers that belongs to the ChannelX
        let dma_ch = self.ch();
        dma_ch.cr().modify(|_, w| w.psize().bits(size));
    }

    #[inline(always)]
    fn set_memory_increment(&mut self, increment: bool) {
        //NOTE(unsafe) We only access the registers that belongs to the ChannelX
        let dma_ch = self.ch();
        dma_ch.cr().modify(|_, w| w.minc().bit(increment));
    }

    #[inline(always)]
    fn set_peripheral_increment(&mut self, increment: bool) {
        //NOTE(unsafe) We only access the registers that belongs to the ChannelX
        let dma_ch = self.ch();
        dma_ch.cr().modify(|_, w| w.pinc().bit(increment));
    }

    #[inline(always)]
    fn set_direction(&mut self, direction: DmaDirection) {
        //NOTE(unsafe) We only access the registers that belongs to the ChannelX
        let dma_ch = self.ch();
        dma_ch.cr().modify(|_, w| match direction {
            DmaDirection::PeripheralToMemory => w.dir().clear_bit().mem2mem().clear_bit(),
            DmaDirection::MemoryToPeripheral => w.dir().set_bit().mem2mem().clear_bit(),
            DmaDirection::MemoryToMemory => w.mem2mem().set_bit().dir().clear_bit(),
        });
    }

    #[inline(always)]
    fn set_circular_buffer(&mut self, circular_buffer: bool) {
        //NOTE(unsafe) We only access the registers that belongs to the ChannelX
        let dma_ch = self.ch();
        dma_ch.cr().modify(|_, w| w.circ().bit(circular_buffer));
    }
}

impl<I: Instance, const N: u8> C<I, N> {
    #[inline(always)]
    pub fn clear_half_transfer_interrupt(&mut self) {
        //NOTE(unsafe) Atomic write with no side-effects and we only access the bits
        // that belongs to the ChannelX
        let dma = unsafe { &*I::ptr() };
        dma.ifcr().write(|w| w.chtif(N).set_bit());
        let _ = dma.isr().read();
        let _ = dma.isr().read(); // Delay 2 peripheral clocks
    }

    #[inline(always)]
    pub fn get_half_transfer_flag() -> bool {
        //NOTE(unsafe) Atomic read with no side effects
        let dma = unsafe { &*I::ptr() };
        dma.isr().read().htif(N).bit_is_set()
    }

    #[inline(always)]
    pub fn set_half_transfer_interrupt_enable(&mut self, half_transfer_interrupt: bool) {
        //NOTE(unsafe) We only access the registers that belongs to the ChannelX
        let dmacr = &self.ch().cr();
        dmacr.modify(|_, w| w.htie().bit(half_transfer_interrupt));
        let _ = dmacr.read();
        let _ = dmacr.read(); // Delay 2 peripheral clocks
    }
}
