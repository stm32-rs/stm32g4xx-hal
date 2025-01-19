//! Traits for DMA types
//!
//! Adapted from
//! <https://github.com/stm32-rs/stm32f4xx-hal/blob/master/src/dma/traits.rs>

use super::*;

pub(crate) mod sealed {
    /// Converts value to bits for setting a register value.
    pub trait Bits<T> {
        /// Returns the bit value.
        fn bits(self) -> T;
    }
    pub trait Sealed {}
}
use config::DmaConfig;
use sealed::Sealed;

/// Minimal trait for DMA channels
pub trait Channel: Sealed {
    /// Structure representing interrupts
    type Interrupts: Copy + Debug;

    /// Apply the configation structure to this channel.
    fn apply_config(&mut self, config: DmaConfig);

    /// Clear all interrupts for the DMA channel.
    fn clear_interrupts(&mut self);

    /// Clear transfer complete interrupt flag (tcif) for the DMA channel
    /// but do not insert artificial delays.
    fn clear_transfer_complete_flag(&mut self);

    /// Clear transfer complete interrupt (tcif) for the DMA channel and delay
    /// to ensure the change has settled through the bridge, peripheral, and
    /// synchronizers.
    fn clear_transfer_complete_interrupt(&mut self);

    /// Clear transfer error interrupt (teif) for the DMA channel.
    fn clear_transfer_error_interrupt(&mut self);

    /// Get transfer complete flag.
    fn get_transfer_complete_flag(&self) -> bool;

    /// Get transfer error flag.
    fn get_transfer_error_flag(&self) -> bool;

    /// Enable the DMA channel.
    ///
    /// # Safety
    ///
    /// The user must ensure that all registers are properly configured.
    unsafe fn enable(&mut self);

    /// Returns the state of the DMA channel.
    fn is_enabled(&self) -> bool;

    /// Disable the DMA channel.
    ///
    /// Disabling the channel during an on-going transfer needs to be performed
    /// in a certain way to prevent problems if the channel is to be re-enabled
    /// shortly after, because of that, this method will also clear all the
    /// channel's interrupt flags if the channel is active.
    fn disable(&mut self);

    /// Sets the request or trigger line for this channel
    fn set_request_line(&mut self, request_line: u8);

    /// Set the priority the DMA channel.
    fn set_priority(&mut self, priority: config::Priority);

    /// Disable all interrupts for the DMA channel.
    fn disable_interrupts(&mut self);

    /// Configure interrupts for the DMA channel
    fn enable_interrupts(&mut self, interrupts: Self::Interrupts);

    /// Get the value of all the interrupts for this DMA channel
    fn get_interrupts_enable(&self) -> Self::Interrupts;

    /// Enable/disable the transfer complete interrupt (tcie) of the DMA channel.
    fn set_transfer_complete_interrupt_enable(&mut self, transfer_complete_interrupt: bool);

    /// Enable/disable the transfer error interrupt (teie) of the DMA channel.
    fn set_transfer_error_interrupt_enable(&mut self, transfer_error_interrupt: bool);

    /// Set the peripheral address (par) for the DMA channel.
    ///
    /// # Safety
    ///
    /// Value should be a word aligned valid peripheral address
    unsafe fn set_peripheral_address(&mut self, value: u32);

    /// Set the memory address (m0ar or m1ar) for the DMA channel.
    ///
    /// # Safety
    ///
    /// Value should be a word aligned valid memory address
    unsafe fn set_memory_address(&mut self, value: u32);

    /// Enable/disable the half transfer interrupt (htie) of the DMA channel.
    fn set_half_transfer_interrupt_enable(&mut self, transfer_complete_interrupt: bool);

    /// Clear half transfer interrupt (htif) for the DMA channel.
    fn clear_half_transfer_interrupt(&mut self);

    /// Get half transfer flag.
    fn get_half_transfer_flag(&self) -> bool;

    /// Get the memory address for the DMA channel.
    fn get_memory_address(&self) -> u32;

    /// Enable/disable memory increment (minc) for the DMA channel.
    fn set_memory_increment(&mut self, increment: bool);

    /// Enable/disable peripheral increment (pinc) for the DMA channel.
    fn set_peripheral_increment(&mut self, increment: bool);

    /// Set the number of transfers (ndt) for the DMA channel.
    fn set_number_of_transfers(&mut self, value: u16);

    /// Get the number of transfers (ndt) for the DMA channel.
    fn get_number_of_transfers(&self) -> u16;

    /// Set the memory size (msize) for the DMA channel.
    ///
    /// # Safety
    ///
    /// This must have the same alignment of the buffer used in the transfer.
    ///
    /// Valid values:
    ///     * 0 -> byte
    ///     * 1 -> half word
    ///     * 2 -> word
    ///     * 3 -> double word
    unsafe fn set_memory_size(&mut self, size: u8);

    /// Set the peripheral memory size (psize) for the DMA channel.
    ///
    /// # Safety
    ///
    /// This must have the same alignment of the peripheral data used in the
    /// transfer.
    ///
    /// Valid values:
    ///     * 0 -> byte
    ///     * 1 -> half word
    ///     * 2 -> word
    unsafe fn set_peripheral_size(&mut self, size: u8);

    /// Set the direction (dir) of the DMA channel.
    fn set_direction(&mut self, direction: DmaDirection);

    /// Enable/disable circular buffering for the DMA channel.
    fn set_circular_buffer(&mut self, circular_buffer: bool);
}

/// DMA direction.
pub trait Direction {
    /// Returns the `DmaDirection` of the type.
    fn direction() -> DmaDirection;
}

/// Mark a target that the DMA can use. This is a peripheral (PeripheralToMemory
/// or MemoryToPeripheral) or a memory (MemoryToMemory)
///
/// This trait is generic over transfer direction, so a given target can
/// implement this trait multiple times for different directions.
///
/// Each implementation has an associated memory size (u32/u16/u8) and
/// optionally an associated request line in the DMA's DMAMUX.
///
/// # Safety
///
/// Both the memory size and the address must be correct for the memory region
/// and for the DMA.
pub unsafe trait TargetAddress<D: Direction> {
    /// Memory size of the target address
    type MemSize;

    /// The address to be used by the DMA channel
    fn address(&self) -> u32;

    /// An optional associated request line
    const REQUEST_LINE: Option<u8> = None;
}
