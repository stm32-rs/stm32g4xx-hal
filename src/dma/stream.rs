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

/// Stream 0 on DMA
pub struct Stream0<DMA> {
    _dma: PhantomData<DMA>,
}
/// Stream 1 on DMA
pub struct Stream1<DMA> {
    _dma: PhantomData<DMA>,
}
/// Stream 2 on DMA
pub struct Stream2<DMA> {
    _dma: PhantomData<DMA>,
}
/// Stream 3 on DMA
pub struct Stream3<DMA> {
    _dma: PhantomData<DMA>,
}
/// Stream 4 on DMA
pub struct Stream4<DMA> {
    _dma: PhantomData<DMA>,
}
/// Stream 5 on DMA
pub struct Stream5<DMA> {
    _dma: PhantomData<DMA>,
}
/// Stream 6 on DMA
pub struct Stream6<DMA> {
    _dma: PhantomData<DMA>,
}
/// Stream 7 on DMA
pub struct Stream7<DMA> {
    _dma: PhantomData<DMA>,
}

impl<DMA> Sealed for Stream0<DMA> {}
impl<DMA> Sealed for Stream1<DMA> {}
impl<DMA> Sealed for Stream2<DMA> {}
impl<DMA> Sealed for Stream3<DMA> {}
impl<DMA> Sealed for Stream4<DMA> {}
impl<DMA> Sealed for Stream5<DMA> {}
impl<DMA> Sealed for Stream6<DMA> {}
impl<DMA> Sealed for Stream7<DMA> {}

/// Alias for a tuple with all DMA streams.
pub struct StreamsTuple<T>(
    pub Stream0<T>,
    pub Stream1<T>,
    pub Stream2<T>,
    pub Stream3<T>,
    pub Stream4<T>,
    pub Stream5<T>,
    #[cfg(not(any(feature = "stm32g431", feature = "stm32g441",)))] pub Stream6<T>,
    #[cfg(not(any(feature = "stm32g431", feature = "stm32g441",)))] pub Stream7<T>,
);

pub trait DMAExt<I> {
    fn split(self, rcc: &Rcc) -> StreamsTuple<I>;
}

impl DMAExt<Self> for DMA1 {
    fn split(self, rcc: &Rcc) -> StreamsTuple<DMA1> {
        // Enable DMAMux is not yet enabled
        if !rcc.rb.ahb1enr.read().dmamuxen().bit_is_set() {
            // Enable peripheral
            rcc.rb.ahb1enr.modify(|_, w| w.dmamuxen().set_bit());
        }

        // Enable peripheral
        rcc.rb.ahb1enr.modify(|_, w| w.dma1en().set_bit());

        StreamsTuple::new(self)
    }
}

impl DMAExt<Self> for DMA2 {
    fn split(self, rcc: &Rcc) -> StreamsTuple<DMA2> {
        // Enable DMAMux is not yet enabled
        if !rcc.rb.ahb1enr.read().dmamuxen().bit_is_set() {
            // Enable peripheral
            rcc.rb.ahb1enr.modify(|_, w| w.dmamuxen().set_bit());
        }

        // Enable peripheral
        rcc.rb.ahb1enr.modify(|_, w| w.dma2en().set_bit());

        StreamsTuple::new(self)
    }
}

impl<I: Instance> StreamsTuple<I> {
    /// Splits the DMA peripheral into streams.
    pub(crate) fn new(_regs: I) -> Self {
        Self(
            Stream0 { _dma: PhantomData },
            Stream1 { _dma: PhantomData },
            Stream2 { _dma: PhantomData },
            Stream3 { _dma: PhantomData },
            Stream4 { _dma: PhantomData },
            Stream5 { _dma: PhantomData },
            #[cfg(not(any(feature = "stm32g431", feature = "stm32g441",)))]
            Stream6 { _dma: PhantomData },
            #[cfg(not(any(feature = "stm32g431", feature = "stm32g441",)))]
            Stream7 { _dma: PhantomData },
        )
    }
}

// Macro that creates a struct representing a stream on either DMA controller
//
// The implementation does the heavy lifting of mapping to the right fields on
// the stream
macro_rules! dma_stream {
    ($(($name:ident, $number:expr,
        regs => $ccr:ident, $cparX:ident, $cmarX:ident, $cndtrX:ident,
        fields => $tcif:ident, $htif:ident, $teif:ident, $gif:ident, $tcisr:ident, $htisr:ident, $teisr:ident, $gisr:ident,
        dmamux => $dma1_cXcr:ident, $dma2_cXcr:ident, )
    ),+$(,)*) => {
        $(
            impl<I: Instance> Stream for $name<I> {

                const NUMBER: usize = $number;
                type Config = DmaConfig;
                type Interrupts = DmaInterrupts;

                fn apply_config(&mut self, config: DmaConfig) {
                    self.set_priority(config.priority);
                    self.set_memory_increment(config.memory_increment);
                    self.set_peripheral_increment(config.peripheral_increment);
                    self.set_circular_buffer(config.circular_buffer);
                    self.set_transfer_complete_interrupt_enable(
                        config.transfer_complete_interrupt
                    );
                    self.set_half_transfer_interrupt_enable(config.half_transfer_interrupt);
                    self.set_transfer_error_interrupt_enable(
                        config.transfer_error_interrupt
                    );
               }

                #[inline(always)]
                fn clear_interrupts(&mut self) {
                    //NOTE(unsafe) Atomic write with no side-effects and we only access the bits
                    // that belongs to the StreamX
                    let dma = unsafe { &*I::ptr() };
                    dma.ifcr.write(|w| w
                                    .$tcif().set_bit() //Clear transfer complete interrupt flag
                                    .$htif().set_bit() //Clear half transfer interrupt flag
                                    .$teif().set_bit() //Clear transfer error interrupt flag
                                    .$gif().set_bit() //Clear global interrupt flag
                    );
                    let _ = dma.isr.read();
                    let _ = dma.isr.read(); // Delay 2 peripheral clocks
                }

                #[inline(always)]
                fn clear_transfer_complete_flag(&mut self) {
                    //NOTE(unsafe) Atomic write with no side-effects and we only access the bits
                    // that belongs to the StreamX
                    let dma = unsafe { &*I::ptr() };
                    dma.ifcr.write(|w| w.$tcif().set_bit());
                }

                #[inline(always)]
                fn clear_transfer_complete_interrupt(&mut self) {
                    self.clear_transfer_complete_flag();
                    //NOTE(unsafe) Atomic read with no side-effects.
                    let dma = unsafe { &*I::ptr() };
                    let _ = dma.isr.read();
                    let _ = dma.isr.read(); // Delay 2 peripheral clocks
                }

                #[inline(always)]
                fn clear_transfer_error_interrupt(&mut self) {
                    //NOTE(unsafe) Atomic write with no side-effects and we only access the bits
                    // that belongs to the StreamX
                    let dma = unsafe { &*I::ptr() };
                    dma.ifcr.write(|w| w.$teif().set_bit());
                    let _ = dma.isr.read();
                    let _ = dma.isr.read(); // Delay 2 peripheral clocks
                }

                #[inline(always)]
                fn get_transfer_complete_flag() -> bool {
                    //NOTE(unsafe) Atomic read with no side effects
                    let dma = unsafe { &*I::ptr() };
                    dma.isr.read().$tcisr().bit_is_set()
                }

                #[inline(always)]
                fn get_transfer_error_flag() -> bool {
                    //NOTE(unsafe) Atomic read with no side effects
                    let dma = unsafe { &*I::ptr() };
                    dma.isr.read().$teisr().bit_is_set()
                }

                #[inline(always)]
                unsafe fn enable(&mut self) {
                    //NOTE(unsafe) We only access the registers that belongs to the StreamX
                    let dma = &*I::ptr();
                    dma.$ccr.modify(|_, w| w.en().set_bit());
                }

                #[inline(always)]
                fn is_enabled() -> bool {
                    //NOTE(unsafe) Atomic read with no side effects
                    let dma = unsafe { &*I::ptr() };
                    dma.$ccr.read().en().bit_is_set()
                }

                fn disable(&mut self) {
                    if Self::is_enabled() {
                        //NOTE(unsafe) We only access the registers that belongs to the StreamX
                        let dma = unsafe { &*I::ptr() };

                        // Aborting an on-going transfer might cause interrupts to fire, disable
                        // them
                        let interrupts = Self::get_interrupts_enable();
                        self.disable_interrupts();

                        dma.$ccr.modify(|_, w| w.en().clear_bit());
                        while Self::is_enabled() {}

                        self.clear_interrupts();
                        self.enable_interrupts(interrupts);
                    }
                }

                #[inline(always)]
                fn set_request_line(&mut self, request_line: u8) {
                    //NOTE(unsafe) We only access the registers that belongs to the StreamX
                    let dmamux = unsafe { &*I::mux_ptr() };
                    unsafe {
                        if I::IS_DMA1 {
                            dmamux.$dma1_cXcr
                                .modify(|_, w| w.dmareq_id().bits(request_line));
                        } else {
                            dmamux.$dma2_cXcr
                                .modify(|_, w| w.dmareq_id().bits(request_line));
                        };
                    }
                }

                #[inline(always)]
                fn set_priority(&mut self, priority: config::Priority) {
                    //NOTE(unsafe) We only access the registers that belongs to the StreamX
                    let dma = unsafe { &*I::ptr() };
                    dma.$ccr.modify(|_, w| unsafe { w.pl().bits(priority.bits()) });
                }

                #[inline(always)]
                fn disable_interrupts(&mut self) {
                    //NOTE(unsafe) We only access the registers that belongs to the StreamX
                    let dmacr = &unsafe { &*I::ptr() }.$ccr;
                    dmacr.modify(|_, w| w
                        .tcie().clear_bit()
                        .teie().clear_bit()
                        .htie().clear_bit());
                    let _ = dmacr.read();
                    let _ = dmacr.read(); // Delay 2 peripheral clocks
                }

                #[inline(always)]
                fn enable_interrupts(&mut self, interrupt: Self::Interrupts) {
                    //NOTE(unsafe) We only access the registers that belongs to the StreamX
                    let dma = unsafe { &*I::ptr() };
                    dma.$ccr.modify(|_, w| w
                                                   .tcie().bit(interrupt.transfer_complete)
                                                   .teie().bit(interrupt.transfer_error)
                                                   .htie().bit(interrupt.half_transfer)
                    );
                }

                #[inline(always)]
                fn get_interrupts_enable() -> Self::Interrupts {
                    //NOTE(unsafe) We only access the registers that belongs to the StreamX
                    let dma = unsafe { &*I::ptr() };
                    let cr = dma.$ccr.read();

                    DmaInterrupts {
                        transfer_complete: cr.tcie().bit_is_set(),
                        half_transfer: cr.htie().bit_is_set(),
                        transfer_error: cr.teie().bit_is_set()
                    }
                }


                #[inline(always)]
                fn set_transfer_complete_interrupt_enable(&mut self, transfer_complete_interrupt: bool) {
                    //NOTE(unsafe) We only access the registers that belongs to the StreamX
                    let dmacr = &unsafe { &*I::ptr() }.$ccr;
                    dmacr.modify(|_, w| w.tcie().bit(transfer_complete_interrupt));
                    let _ = dmacr.read();
                    let _ = dmacr.read(); // Delay 2 peripheral clocks
                }

                #[inline(always)]
                fn set_transfer_error_interrupt_enable(&mut self, transfer_error_interrupt: bool) {
                    //NOTE(unsafe) We only access the registers that belongs to the StreamX
                    let dmacr = &unsafe { &*I::ptr() }.$ccr;
                    dmacr.modify(|_, w| w.teie().bit(transfer_error_interrupt));
                    let _ = dmacr.read();
                    let _ = dmacr.read(); // Delay 2 peripheral clocks
                }

                #[inline(always)]
                fn set_half_transfer_interrupt_enable(&mut self, half_transfer_interrupt: bool) {
                    //NOTE(unsafe) We only access the registers that belongs to the StreamX
                    let dmacr = &unsafe { &*I::ptr() }.$ccr;
                    dmacr.modify(|_, w| w.htie().bit(half_transfer_interrupt));
                    let _ = dmacr.read();
                    let _ = dmacr.read(); // Delay 2 peripheral clocks
                }

                #[inline(always)]
                fn get_half_transfer_flag() -> bool {
                    //NOTE(unsafe) Atomic read with no side effects
                    let dma = unsafe { &*I::ptr() };
                    dma.isr.read().$htisr().bit_is_set()
                }

                #[inline(always)]
                fn clear_half_transfer_interrupt(&mut self) {
                    //NOTE(unsafe) Atomic write with no side-effects and we only access the bits
                    // that belongs to the StreamX
                    let dma = unsafe { &*I::ptr() };
                    dma.ifcr.write(|w| w.$htif().set_bit());
                    let _ = dma.isr.read();
                    let _ = dma.isr.read(); // Delay 2 peripheral clocks
                }

                #[inline(always)]
                unsafe fn set_peripheral_address(&mut self, value: u32) {
                    //NOTE(unsafe) We only access the registers that belongs to the StreamX
                    let dma = &*I::ptr();
                    dma.$cparX.write(|w| w.pa().bits(value));
                }

                #[inline(always)]
                unsafe fn set_memory_address(&mut self, value: u32) {
                    //NOTE(unsafe) We only access the registers that belongs to the StreamX
                    let dma = &*I::ptr();
                    dma.$cmarX.write(|w| w.ma().bits(value) );
                }

                #[inline(always)]
                fn get_memory_address(&self) -> u32 {
                    //NOTE(unsafe) We only access the registers that belongs to the StreamX
                    let dma = unsafe { &*I::ptr() };
                    dma.$cmarX.read().ma().bits()
                }

                #[inline(always)]
                fn set_number_of_transfers(&mut self, value: u16) {
                    //NOTE(unsafe) We only access the registers that belongs to the StreamX
                    let dma = unsafe { &*I::ptr() };
                    dma.$cndtrX.write(|w| unsafe {w.ndt().bits(value) });
                }

                #[inline(always)]
                fn get_number_of_transfers() -> u16 {
                    //NOTE(unsafe) We only access the registers that belongs to the StreamX
                    let dma = unsafe { &*I::ptr() };
                    dma.$cndtrX.read().ndt().bits()
                }
                #[inline(always)]
                unsafe fn set_memory_size(&mut self, size: u8) {
                    //NOTE(unsafe) We only access the registers that belongs to the StreamX
                    let dma = &*I::ptr();
                    dma.$ccr.modify(|_, w| w.msize().bits(size));
                }

                #[inline(always)]
                unsafe fn set_peripheral_size(&mut self, size: u8) {
                    //NOTE(unsafe) We only access the registers that belongs to the StreamX
                    let dma = &*I::ptr();
                    dma.$ccr.modify(|_, w| w.psize().bits(size));
                }

                #[inline(always)]
                fn set_memory_increment(&mut self, increment: bool) {
                    //NOTE(unsafe) We only access the registers that belongs to the StreamX
                    let dma = unsafe { &*I::ptr() };
                    dma.$ccr.modify(|_, w| w.minc().bit(increment));
                }

                #[inline(always)]
                fn set_peripheral_increment(&mut self, increment: bool) {
                    //NOTE(unsafe) We only access the registers that belongs to the StreamX
                    let dma = unsafe { &*I::ptr() };
                    dma.$ccr.modify(|_, w| w.pinc().bit(increment));
                }

                #[inline(always)]
                fn set_direction(&mut self, direction: DmaDirection) {
                    //NOTE(unsafe) We only access the registers that belongs to the StreamX
                    let dma = unsafe { &*I::ptr() };
                    dma.$ccr.modify(|_, w| {
                        match direction {
                            DmaDirection::PeripheralToMemory =>
                                w.dir().clear_bit().mem2mem().clear_bit(),
                            DmaDirection::MemoryToPeripheral =>
                                w.dir().set_bit().mem2mem().clear_bit(),
                            DmaDirection::MemoryToMemory =>
                                w.mem2mem().set_bit().dir().clear_bit(),
                        }
                    });
                }

                #[inline(always)]
                fn set_circular_buffer(&mut self, circular_buffer: bool) {
                    //NOTE(unsafe) We only access the registers that belongs to the StreamX
                    let dma = unsafe { &*I::ptr() };
                    dma.$ccr.modify(|_, w| w.circ().bit(circular_buffer));
                }
            }

            impl<I: Instance> $name<I> {
                #[inline(always)]
                pub fn clear_half_transfer_interrupt(&mut self) {
                    //NOTE(unsafe) Atomic write with no side-effects and we only access the bits
                    // that belongs to the StreamX
                    let dma = unsafe { &*I::ptr() };
                    dma.ifcr.write(|w| w.$htif().set_bit());
                    let _ = dma.isr.read();
                    let _ = dma.isr.read(); // Delay 2 peripheral clocks
                }

                #[inline(always)]
                pub fn get_half_transfer_flag() -> bool {
                    //NOTE(unsafe) Atomic read with no side effects
                    let dma = unsafe { &*I::ptr() };
                    dma.isr.read().$htisr().bit_is_set()
                }

                #[inline(always)]
                pub fn set_half_transfer_interrupt_enable(&mut self, half_transfer_interrupt: bool) {
                    //NOTE(unsafe) We only access the registers that belongs to the StreamX
                    let dmacr = &unsafe { &*I::ptr() }.$ccr;
                    dmacr.modify(|_, w| w.htie().bit(half_transfer_interrupt));
                    let _ = dmacr.read();
                    let _ = dmacr.read(); // Delay 2 peripheral clocks
                }
            }
        )+
    };
}

// Cat 3 and 4 devices
#[cfg(any(
    feature = "stm32g471",
    feature = "stm32g473",
    feature = "stm32g474",
    feature = "stm32g483",
    feature = "stm32g484",
    feature = "stm32g491",
    feature = "stm32g49a",
))]
dma_stream!(
    // Note: the field names start from one, unlike the RM where they start from
    // zero. May need updating if it gets fixed upstream.
    (
        Stream0, 0,
        regs => ccr1, cpar1, cmar1, cndtr1,
        fields => tcif1, htif1, teif1, gif1, tcif1, htif1, teif1, gif1,
        dmamux => c0cr, c8cr,
    ),
    (
        Stream1, 1,
        regs => ccr2, cpar2, cmar2, cndtr2,
        fields => tcif2, htif2, teif2, gif2, tcif2, htif2, teif2, gif2,
        dmamux => c1cr, c9cr,
    ),
    (
        Stream2, 2,
        regs => ccr3, cpar3, cmar3, cndtr3,
        fields => tcif3, htif3, teif3, gif3, tcif3, htif3, teif3, gif3,
        dmamux => c2cr, c10cr,
    ),
    (
        Stream3, 3,
        regs => ccr4, cpar4, cmar4, cndtr4,
        fields => tcif4, htif4, teif4, gif4, tcif4, htif4, teif4, gif4,
        dmamux => c3cr, c11cr,
    ),
    (
        Stream4, 4,
        regs => ccr5, cpar5, cmar5, cndtr5,
        fields => tcif5, htif5, teif5, gif5, tcif5, htif5, teif5, gif5,
        dmamux => c4cr, c12cr,
    ),
    (
        Stream5, 5,
        regs => ccr6, cpar6, cmar6, cndtr6,
        fields => tcif6, htif6, teif6, gif6, tcif6, htif6, teif6, gif6,
        dmamux => c5cr, c13cr,
    ),
    (
        Stream6, 6,
        regs => ccr7, cpar7, cmar7, cndtr7,
        fields => tcif7, htif7, teif7, gif7, tcif7, htif7, teif7, gif7,
        dmamux => c6cr, c14cr,
    ),
    (
        Stream7, 7,
        regs => ccr8, cpar8, cmar8, cndtr8,
        fields => tcif8, htif8, teif8, gif8, tcif8, htif8, teif8, gif8,
        dmamux => c7cr, c15cr,
    ),
);

// Cat 2 devices
#[cfg(any(feature = "stm32g431", feature = "stm32g441",))]
dma_stream!(
    // Note: the field names start from one, unlike the RM where they start from
    // zero. May need updating if it gets fixed upstream.
    (
        Stream0, 0,
        regs => ccr1, cpar1, cmar1, cndtr1,
        fields => tcif1, htif1, teif1, gif1, tcif1, htif1, teif1, gif1,
        dmamux => c0cr, c6cr,
    ),
    (
        Stream1, 1,
        regs => ccr2, cpar2, cmar2, cndtr2,
        fields => tcif2, htif2, teif2, gif2, tcif2, htif2, teif2, gif2,
        dmamux => c1cr, c7cr,
    ),
    (
        Stream2, 2,
        regs => ccr3, cpar3, cmar3, cndtr3,
        fields => tcif3, htif3, teif3, gif3, tcif3, htif3, teif3, gif3,
        dmamux => c2cr, c8cr,
    ),
    (
        Stream3, 3,
        regs => ccr4, cpar4, cmar4, cndtr4,
        fields => tcif4, htif4, teif4, gif4, tcif4, htif4, teif4, gif4,
        dmamux => c3cr, c9cr,
    ),
    (
        Stream4, 4,
        regs => ccr5, cpar5, cmar5, cndtr5,
        fields => tcif5, htif5, teif5, gif5, tcif5, htif5, teif5, gif5,
        dmamux => c4cr, c10cr,
    ),
    (
        Stream5, 5,
        regs => ccr6, cpar6, cmar6, cndtr6,
        fields => tcif6, htif6, teif6, gif6, tcif6, htif6, teif6, gif6,
        dmamux => c5cr, c11cr,
    ),
);
