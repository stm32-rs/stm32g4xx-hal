//! Direct Memory Access.
//!
//! [Transfer::init](struct.Transfer.html#method.init) is only implemented for
//! valid combinations of peripheral-channel-direction, providing compile
//! time checking.
//!
//! This module implements Memory To Memory, Peripheral To Memory and Memory to
//! Peripheral transfers.
//!
//! Given that the Cortex-M7 core is capable of reordering accesses between
//! normal and device memory, we insert DMB instructions to ensure correct
//! operation. See ARM DAI 0321A, Section 3.2 which discusses the use of DMB
//! instructions in DMA controller configuration.
//!
//! Adapted from
//! <https://github.com/stm32-rs/stm32h7xx-hal/blob/master/src/dma/mod.rs>

use core::fmt::Debug;

pub mod channel; // DMA MUX // DMA1 and DMA2
pub mod config;
pub(crate) mod mux;
pub mod traits;
pub mod transfer;

use traits::{sealed::Bits, Channel, Direction, TargetAddress};
pub use transfer::{Transfer, TransferExt};

/// Errors.
#[derive(PartialEq, Debug, Copy, Clone)]
pub enum DMAError {
    /// DMA not ready to change buffers.
    NotReady,
    /// The user provided a buffer that is not big enough while double buffering.
    SmallBuffer,
    /// DMA started transfer on the inactive buffer while the user was processing it.
    Overflow,
}

/// Possible DMA's directions.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum DmaDirection {
    /// Memory to Memory transfer.
    MemoryToMemory,
    /// Peripheral to Memory transfer.
    PeripheralToMemory,
    /// Memory to Peripheral transfer.
    MemoryToPeripheral,
}

/// DMA from a peripheral to a memory location.
#[derive(Debug, Clone, Copy)]
pub struct PeripheralToMemory;

impl PeripheralToMemory {
    pub fn new() -> Self {
        PeripheralToMemory
    }
}

impl Default for PeripheralToMemory {
    fn default() -> Self {
        Self::new()
    }
}

impl Direction for PeripheralToMemory {
    #[inline(always)]
    fn direction() -> DmaDirection {
        DmaDirection::PeripheralToMemory
    }
}

/// DMA from one memory location to another memory location.
#[derive(Debug, Clone, Copy)]
pub struct MemoryToMemory<T>
where
    T: Into<u32>,
{
    data: T,
}

impl<T> MemoryToMemory<T>
where
    T: Into<u32>,
{
    pub fn new(t: T) -> Self {
        Self { data: t }
    }
}

impl<T> Direction for MemoryToMemory<T>
where
    T: Into<u32>,
{
    #[inline(always)]
    fn direction() -> DmaDirection {
        DmaDirection::MemoryToMemory
    }
}

/// DMA from a memory location to a peripheral.
#[derive(Debug, Clone, Copy)]
pub struct MemoryToPeripheral;

impl MemoryToPeripheral {
    pub fn new() -> Self {
        MemoryToPeripheral
    }
}

impl Default for MemoryToPeripheral {
    fn default() -> Self {
        Self::new()
    }
}

impl Direction for MemoryToPeripheral {
    fn direction() -> DmaDirection {
        DmaDirection::MemoryToPeripheral
    }
}

unsafe impl TargetAddress<Self> for MemoryToMemory<u8> {
    fn address(&self) -> u32 {
        self.data.into()
    }
    type MemSize = u8;
}

unsafe impl TargetAddress<Self> for MemoryToMemory<u16> {
    fn address(&self) -> u32 {
        self.data.into()
    }
    type MemSize = u16;
}

unsafe impl TargetAddress<Self> for MemoryToMemory<u32> {
    fn address(&self) -> u32 {
        self.data
    }
    type MemSize = u32;
}
