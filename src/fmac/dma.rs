//! DMA support for the FMAC peripheral.

use embedded_dma::ReadBuffer;

use crate::dma::{
    mux::DmaMuxResources, traits::TargetAddress, MemoryToPeripheral, PeripheralToMemory,
};

/// Write handle for DMA transfers to the FMAC peripheral.
pub struct FmacDmaWriter {
    wdata: *mut u32,
}

impl FmacDmaWriter {
    pub(crate) fn new(wdata: *mut u32) -> Self {
        Self { wdata }
    }
}

/// Read handle for DMA transfers from the FMAC peripheral.
pub struct FmacDmaReader {
    rdata: *mut u32,
}

impl FmacDmaReader {
    pub(crate) fn new(rdata: *mut u32) -> Self {
        Self { rdata }
    }
}

unsafe impl TargetAddress<PeripheralToMemory> for FmacDmaReader {
    type MemSize = u32;
    const REQUEST_LINE: Option<u8> = Some(DmaMuxResources::FMAC_Read as u8);

    fn address(&self) -> u32 {
        self.rdata as u32
    }
}

unsafe impl ReadBuffer for FmacDmaReader {
    type Word = u32;

    unsafe fn read_buffer(&self) -> (*const Self::Word, usize) {
        (self.rdata as *const u32, 1)
    }
}

unsafe impl TargetAddress<MemoryToPeripheral> for FmacDmaWriter {
    type MemSize = u32;
    const REQUEST_LINE: Option<u8> = Some(DmaMuxResources::FMAC_Read as u8);

    fn address(&self) -> u32 {
        self.wdata as u32
    }
}
