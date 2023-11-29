use super::Bits;

/// Priority of the DMA stream, defaults to `Medium`. If two requests have
/// the same software priority level, the stream with the lower number takes
/// priority over the stream with the higher number. For example, Stream 2
/// takes priority over Stream 4.
#[derive(Debug, Clone, Copy, Default)]
pub enum Priority {
    /// Low priority.
    Low,
    /// Medium priority.
    #[default]
    Medium,
    /// High priority.
    High,
    /// Very high priority.
    VeryHigh,
}

impl Bits<u8> for Priority {
    fn bits(self) -> u8 {
        match self {
            Priority::Low => 0,
            Priority::Medium => 1,
            Priority::High => 2,
            Priority::VeryHigh => 3,
        }
    }
}

/// Contains the complete set of configuration for a DMA stream.
#[derive(Debug, Default, Clone, Copy)]
pub struct DmaConfig {
    pub(crate) priority: Priority,
    pub(crate) memory_increment: bool,
    pub(crate) peripheral_increment: bool,
    pub(crate) circular_buffer: bool,
    pub(crate) transfer_complete_interrupt: bool,
    pub(crate) half_transfer_interrupt: bool,
    pub(crate) transfer_error_interrupt: bool,
    pub(crate) double_buffer: bool,
}

impl DmaConfig {
    /// Set the priority.
    #[inline(always)]
    pub fn priority(mut self, priority: Priority) -> Self {
        self.priority = priority;
        self
    }
    /// Set the memory_increment.
    #[inline(always)]
    pub fn memory_increment(mut self, memory_increment: bool) -> Self {
        self.memory_increment = memory_increment;
        self
    }
    /// Set the peripheral_increment.
    #[inline(always)]
    pub fn peripheral_increment(mut self, peripheral_increment: bool) -> Self {
        self.peripheral_increment = peripheral_increment;
        self
    }
    /// Set the to use a circular_buffer.
    #[inline(always)]
    pub fn circular_buffer(mut self, circular_buffer: bool) -> Self {
        self.circular_buffer = circular_buffer;
        self
    }
    /// Set the transfer_complete_interrupt.
    #[inline(always)]
    pub fn transfer_complete_interrupt(mut self, transfer_complete_interrupt: bool) -> Self {
        self.transfer_complete_interrupt = transfer_complete_interrupt;
        self
    }
    /// Set the half_transfer_interrupt.
    #[inline(always)]
    pub fn half_transfer_interrupt(mut self, half_transfer_interrupt: bool) -> Self {
        self.half_transfer_interrupt = half_transfer_interrupt;
        self
    }
    /// Set the transfer_error_interrupt.
    #[inline(always)]
    pub fn transfer_error_interrupt(mut self, transfer_error_interrupt: bool) -> Self {
        self.transfer_error_interrupt = transfer_error_interrupt;
        self
    }
    /// Set the double_buffer.
    #[inline(always)]
    pub fn double_buffer(mut self, double_buffer: bool) -> Self {
        self.double_buffer = double_buffer;
        self
    }
}
