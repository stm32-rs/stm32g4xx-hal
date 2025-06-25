//! Filter math accelerator (FMAC) peripheral
//!
//! The FMAC can perform FIR, IIR (direct form 1), and vector functions such as dot product

use fixed::types::I1F15;

use crate::{
    fmac::{
        buffer::{BufferLayoutConfig, Watermark},
        dma::{FmacDmaReader, FmacDmaWriter},
        function::{Function, LoadX1, LoadX2, LoadY},
    },
    rcc::Rcc,
    stm32::FMAC,
};
use core::marker::PhantomData;

pub mod buffer;
pub mod dma;
pub mod function;
pub mod interrupt;

pub trait State {}

pub struct Stopped;
impl State for Stopped {}

pub struct Running;
impl State for Running {}

pub struct StoppedDma;
impl State for StoppedDma {}

pub struct RunningDma;
impl State for RunningDma {}

pub struct Fmac<Layout: BufferLayoutConfig, S: State> {
    _phantom: PhantomData<(Layout, S)>,
}

impl<Layout: BufferLayoutConfig, S: State> Fmac<Layout, S> {}

/// Buffer selection
pub enum Buffer {
    X1,
    X2,
    Y,
}

impl<Layout: BufferLayoutConfig> Fmac<Layout, StoppedDma> {
    /// Start the FMAC in DMA mode, consuming self and return Fmac in the RunningDma state
    #[inline(always)]
    pub fn start_dma(self) -> Fmac<Layout, RunningDma> {
        let fmac = unsafe { &*FMAC::ptr() };
        fmac.param().modify(|_, w| w.start().set_bit());
        Fmac::next_state()
    }
}

impl<Layout: BufferLayoutConfig> Fmac<Layout, Stopped> {
    /// Configure the FMAC buffers using the layout provided to the constrain method
    fn configure_buffers(&mut self) {
        let fmac = unsafe { &*FMAC::ptr() };
        fmac.x1bufcfg().modify(|_, w| unsafe {
            w.x1_base()
                .bits(Layout::X1_BASE)
                .x1_buf_size()
                .bits(Layout::X1_SIZE)
        });

        fmac.x2bufcfg().modify(|_, w| unsafe {
            w.x2_base()
                .bits(Layout::X2_BASE)
                .x2_buf_size()
                .bits(Layout::X2_SIZE)
        });

        fmac.ybufcfg().modify(|_, w| unsafe {
            w.y_base()
                .bits(Layout::Y_BASE)
                .y_buf_size()
                .bits(Layout::Y_SIZE)
        });
    }

    /// Preloads a [`Buffer`] with data. The closure is called to get the next element to write,
    /// and will be called Layout::<buffer>_SIZE times, expecting an I1F15 fixed point in return.
    ///
    /// This is only for initialization and should not be used during normal operation.
    #[inline(always)]
    pub fn preload_buffer(&mut self, buffer: Buffer, mut f: impl FnMut(u8) -> I1F15) {
        let fmac = unsafe { &*FMAC::ptr() };

        let word_count = match buffer {
            Buffer::X1 => Layout::X1_SIZE,
            Buffer::X2 => Layout::X2_SIZE,
            Buffer::Y => Layout::Y_SIZE,
        };

        match buffer {
            Buffer::X1 => self.select_function(LoadX1(word_count)),
            Buffer::X2 => self.select_function(LoadX2(word_count, 0)),
            Buffer::Y => self.select_function(LoadY(word_count)),
        }

        // Set the start bit before writing data. This will be reset by hardware when the load operation is complete
        fmac.param().modify(|_, w| w.start().set_bit());

        for index in 0..word_count {
            // Get a word to write from the closure
            let word = f(index);
            fmac.wdata()
                .write(|w| unsafe { w.bits(word.to_bits() as u32) });
        }

        // Wait for the start bit to be cleared by hardware
        while fmac.param().read().start().bit_is_set() {}
    }

    /// Select the FMAC function, and set the input parameters
    #[inline(always)]
    pub fn select_function(&mut self, function: impl Function) {
        let fmac = unsafe { &*FMAC::ptr() };
        fmac.param()
            .modify(|_, w| unsafe { w.func().bits(function.id()) });

        self.set_parameters(
            function.p(),
            function.q().unwrap_or(0),
            function.r().unwrap_or(0),
        );
    }

    /// Set FMAC input parameters. These are function dependent
    #[inline(always)]
    fn set_parameters(&mut self, p: u8, q: u8, r: u8) {
        let fmac = unsafe { &*FMAC::ptr() };
        fmac.param()
            .modify(|_, w| unsafe { w.p().bits(p).q().bits(q).r().bits(r) });
    }

    /// Set FMAC clipping mode
    ///
    /// true: Values from accumulator are saturated to the Q1.15 range [-1,1]
    /// false: Values from accumulator will wrap
    #[inline(always)]
    pub fn set_clipping(&mut self, clipping: bool) {
        let fmac = unsafe { &*FMAC::ptr() };
        fmac.cr().modify(|_, w| w.clipen().bit(clipping));
    }

    /// Set the [`Watermark`] level of the specified [`Buffer`]
    ///
    /// The X2 buffer does not support watermarks, as it is typically for
    /// constant coefficients. It will silently ignore the watermark setting.
    ///
    /// For the X1 buffer, the watermark level defines the threshold for setting the Y1FULL flag.
    ///
    /// For the Y buffer, the watermark level defines the threshold for setting the YEMPTY flag.
    /// It must be set to Threshold1 when using DMA mode.
    #[inline(always)]
    pub fn set_watermark(&mut self, buffer: Buffer, watermark: Watermark) {
        let fmac = unsafe { &*FMAC::ptr() };
        match buffer {
            Buffer::X1 => {
                fmac.x1bufcfg()
                    .modify(|_, w| unsafe { w.full_wm().bits(watermark as u8) });
            }
            Buffer::Y => {
                fmac.ybufcfg()
                    .modify(|_, w| unsafe { w.empty_wm().bits(watermark as u8) });
            }
            _ => {}
        }
    }

    /// Start the FMAC, consuming self and return Fmac in the Running state
    #[inline(always)]
    pub fn start(self) -> Fmac<Layout, Running> {
        let fmac = unsafe { &*FMAC::ptr() };
        fmac.param().modify(|_, w| w.start().set_bit());
        Fmac::next_state()
    }

    /// Acquire DMA reader and writer handles for the FMAC.
    /// This returns a tuple of Fmac in the StoppedDma state, FmacDmaReader, and FmacDmaWriter.
    /// The start_dma() method must be called before the FMAC starts after the reader/writer have been assigned
    /// to Transfers.
    #[inline(always)]
    pub fn acquire_dma(self) -> (Fmac<Layout, StoppedDma>, FmacDmaReader, FmacDmaWriter) {
        let fmac = unsafe { &*FMAC::ptr() };

        (
            Fmac::next_state(),
            FmacDmaReader::new(fmac.rdata() as *const _ as *mut u32),
            FmacDmaWriter::new(fmac.wdata() as *const _ as *mut u32),
        )
    }
}

impl<Layout: BufferLayoutConfig, S: State> Fmac<Layout, S> {
    /// Transition to the next state
    pub fn next_state() -> Self {
        Fmac {
            _phantom: PhantomData,
        }
    }

    /// Reset the FMAC. Resets read pointers, internal logic,
    /// status register, and clears parameters.
    ///
    /// Following a reset, the FMAC is in the stopped state,
    /// and can be restarted again by selecting a new Function
    /// and issuing a start.
    #[inline(always)]
    pub fn reset(self) -> Fmac<Layout, Stopped> {
        let fmac = unsafe { &*FMAC::ptr() };
        fmac.cr().modify(|_, w| w.reset().set_bit());
        // Wait for reset to be cleared by hardware
        while fmac.cr().read().reset().bit_is_set() {}
        Fmac::next_state()
    }

    /// Get the saturation error status.
    /// If this bit is set, it can only be cleared by resetting the FMAC.
    #[inline(always)]
    pub fn saturation_error_status(&self) -> bool {
        let fmac = unsafe { &*FMAC::ptr() };
        fmac.sr().read().sat().bit_is_set()
    }

    /// Get the underflow error status.
    /// If this bit is set, it can only be cleared by resetting the FMAC.
    #[inline(always)]
    pub fn underflow_error_status(&self) -> bool {
        let fmac = unsafe { &*FMAC::ptr() };
        fmac.sr().read().unfl().bit_is_set()
    }

    /// Get the ovrerflow error status.
    /// If this bit is set, it can only be cleared by resetting the FMAC.
    #[inline(always)]
    pub fn overflow_error_status(&self) -> bool {
        let fmac = unsafe { &*FMAC::ptr() };
        fmac.sr().read().ovfl().bit_is_set()
    }

    /// Get the X1 Buffer full status.
    ///
    /// If the flag is set, the X1 buffer is full and no more data can be written to it.
    /// The flag is set and cleared by hardware, or a reset.
    #[inline(always)]
    pub fn x1_full_status(&self) -> bool {
        let fmac = unsafe { &*FMAC::ptr() };
        fmac.sr().read().x1full().bit_is_set()
    }

    /// Get the Y Buffer empty status
    ///
    /// If the flag is set, the Y buffer is empty and no more data can be read from it.
    /// The flag is set and cleared by hardware, or a reset.
    #[inline(always)]
    pub fn y_empty_status(&self) -> bool {
        let fmac = unsafe { &*FMAC::ptr() };
        fmac.sr().read().yempty().bit_is_set()
    }

    /// Enable read DMA from the RDATA register
    #[inline(always)]
    pub fn enable_read_dma(&mut self, enable: bool) {
        let fmac = unsafe { &*FMAC::ptr() };
        fmac.cr().modify(|_, w| w.dmaren().bit(enable));
    }

    /// Enable write DMA to the WDATA register
    #[inline(always)]
    pub fn enable_write_dma(&mut self, enable: bool) {
        let fmac = unsafe { &*FMAC::ptr() };
        fmac.cr().modify(|_, w| w.dmawen().bit(enable));
    }
}

impl<Layout: BufferLayoutConfig> Fmac<Layout, RunningDma> {
    #[inline(always)]
    pub fn write(&mut self, data: I1F15) {
        let fmac = unsafe { &*FMAC::ptr() };
        fmac.wdata()
            .write(|w| unsafe { w.bits(data.to_bits() as u32) });
    }
}

impl<Layout: BufferLayoutConfig> Fmac<Layout, Running> {
    /// Stop the FMAC operation, clearing the START bit in the PARAM register.
    /// Consumes self and returns Fmac in the Stopped state, where it can be reconfigured or started again.
    ///
    /// # Returns
    ///
    /// * Fmac in the Stopped state
    pub fn stop(self) -> Fmac<Layout, Stopped> {
        let fmac = unsafe { &*FMAC::ptr() };
        fmac.param().modify(|_, w| w.start().clear_bit());
        Fmac::next_state()
    }

    /// Check if there is a result available in the output buffer.
    ///
    /// # Returns
    ///
    /// * true if there are pending results which can be read
    /// * false if the Y buffer is empty
    pub fn is_result_available(&self) -> bool {
        let fmac = unsafe { &*FMAC::ptr() };
        fmac.sr().read().yempty().bit_is_clear()
    }

    /// Check if the input buffer (X1) is full
    ///
    /// # Returns
    ///
    /// * true if the input buffer is full
    /// * false if the input buffer is not full
    pub fn is_input_full(&self) -> bool {
        let fmac = unsafe { &*FMAC::ptr() };
        fmac.sr().read().x1full().bit_is_set()
    }

    /// Read the result from the RDATA register.
    /// The Y buffer read pointer is automatically incremented on each read.
    ///
    /// # Returns
    ///
    /// * None if the Y buffer is empty
    /// * Some Fixed point Q1.15 value result as [`fixed::I1F15`] type
    #[inline(always)]
    pub fn read(&self) -> Option<I1F15> {
        if self.is_result_available() {
            let fmac = unsafe { &*FMAC::ptr() };
            let result = fmac.rdata().read().bits() as i16;
            Some(I1F15::from_bits(result))
        } else {
            None
        }
    }

    /// Write input data to the WDATA register.
    /// The X1 buffer write pointer is automatically incremented on each write.
    ///
    /// # Arguments
    ///
    /// * `data` - The input data to write to the FMAC.
    #[inline(always)]
    pub fn write(&mut self, data: I1F15) {
        let fmac = unsafe { &*FMAC::ptr() };
        fmac.wdata()
            .write(|w| unsafe { w.bits(data.to_bits() as u32) });
    }
}

/// Extension trait for constraining the FMAC peripheral.
pub trait FmacExt {
    /// Constrain the FMAC peripheral.
    ///
    /// # Arguments
    ///
    /// * `rcc` - The RCC peripheral for enabling the FMAC clock.
    fn constrain<Layout: BufferLayoutConfig>(self, rcc: &mut Rcc) -> Fmac<Layout, Stopped>;
}

impl FmacExt for FMAC {
    fn constrain<Layout: BufferLayoutConfig>(self, rcc: &mut Rcc) -> Fmac<Layout, Stopped> {
        rcc.rb.ahb1rstr().modify(|_, w| w.fmacrst().set_bit());
        rcc.rb.ahb1rstr().modify(|_, w| w.fmacrst().clear_bit());

        rcc.rb.ahb1enr().modify(|_, w| w.fmacen().set_bit());

        let mut fmac = Fmac::<Layout, Stopped>::next_state().reset();
        fmac.configure_buffers();
        fmac
    }
}
