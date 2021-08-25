use crate::prelude::*;
use crate::time::Bps;

#[derive(PartialEq, PartialOrd, Clone, Copy)]
pub enum WordLength {
    DataBits7,
    DataBits8,
    DataBits9,
}

#[derive(PartialEq, PartialOrd, Clone, Copy)]
pub enum Parity {
    ParityNone,
    ParityEven,
    ParityOdd,
}

#[derive(PartialEq, PartialOrd, Clone, Copy, Debug)]
pub enum StopBits {
    #[doc = "1 stop bit"]
    STOP1 = 0b00,
    #[doc = "0.5 stop bits"]
    STOP0P5 = 0b01,
    #[doc = "2 stop bits"]
    STOP2 = 0b10,
    #[doc = "1.5 stop bits"]
    STOP1P5 = 0b11,
}

impl StopBits {
    pub fn bits(self) -> u8 {
        self as u8
    }
}

#[derive(PartialEq, PartialOrd, Clone, Copy, Debug)]
pub enum FifoThreshold {
    #[doc = "1/8 of its depth"]
    FIFO_1_BYTE = 0b000,
    #[doc = "1/4 of its depth"]
    FIFO_2_BYTES = 0b001,
    #[doc = "1/2 of its depth"]
    FIFO_4_BYTES = 0b010,
    #[doc = "3/4 of its depth"]
    FIFO_6_BYTES = 0b011,
    #[doc = "7/8 of its depth"]
    FIFO_7_BYTES = 0b100,
    #[doc = "fifo empty/full"]
    FIFO_8_BYTES = 0b101,
}

impl FifoThreshold {
    pub fn bits(self) -> u8 {
        self as u8
    }
}
#[derive(PartialEq, PartialOrd, Clone, Copy)]
pub struct BasicConfig {
    pub(crate) baudrate: Bps,
    pub(crate) wordlength: WordLength,
    pub(crate) parity: Parity,
    pub(crate) stopbits: StopBits,
    pub(crate) swap: bool,
}

#[derive(PartialEq, PartialOrd, Clone, Copy)]
pub struct FullConfig {
    pub(crate) baudrate: Bps,
    pub(crate) wordlength: WordLength,
    pub(crate) parity: Parity,
    pub(crate) stopbits: StopBits,
    pub(crate) swap: bool,
    pub(crate) fifo_enable: bool,
    pub(crate) tx_fifo_threshold: FifoThreshold,
    pub(crate) rx_fifo_threshold: FifoThreshold,
    pub(crate) tx_fifo_interrupt: bool,
    pub(crate) rx_fifo_interrupt: bool,
    #[doc = "Number of bits no activity on rx line"]
    pub(crate) receiver_timeout: Option<u32>,
}

impl BasicConfig {
    pub fn baudrate(mut self, baudrate: Bps) -> Self {
        self.baudrate = baudrate;
        self
    }

    pub fn parity_none(mut self) -> Self {
        self.parity = Parity::ParityNone;
        self
    }

    pub fn parity_even(mut self) -> Self {
        self.parity = Parity::ParityEven;
        self
    }

    pub fn parity_odd(mut self) -> Self {
        self.parity = Parity::ParityOdd;
        self
    }

    pub fn wordlength_8(mut self) -> Self {
        self.wordlength = WordLength::DataBits8;
        self
    }

    pub fn wordlength_9(mut self) -> Self {
        self.wordlength = WordLength::DataBits9;
        self
    }

    pub fn stopbits(mut self, stopbits: StopBits) -> Self {
        self.stopbits = stopbits;
        self
    }

    /// Swap the Tx/Rx pins
    ///
    /// The peripheral will transmit on the pin given as the `rx` argument.
    pub fn swap_pins(mut self) -> Self {
        self.swap = true;
        self
    }
}

impl FullConfig {
    pub fn baudrate(mut self, baudrate: Bps) -> Self {
        self.baudrate = baudrate;
        self
    }

    pub fn parity_none(mut self) -> Self {
        self.parity = Parity::ParityNone;
        self
    }

    pub fn parity_even(mut self) -> Self {
        self.parity = Parity::ParityEven;
        self
    }

    pub fn parity_odd(mut self) -> Self {
        self.parity = Parity::ParityOdd;
        self
    }

    pub fn wordlength_8(mut self) -> Self {
        self.wordlength = WordLength::DataBits8;
        self
    }

    pub fn wordlength_9(mut self) -> Self {
        self.wordlength = WordLength::DataBits9;
        self
    }

    pub fn stopbits(mut self, stopbits: StopBits) -> Self {
        self.stopbits = stopbits;
        self
    }

    /// Swap the Tx/Rx pins
    ///
    /// The peripheral will transmit on the pin given as the `rx` argument.
    pub fn swap_pins(mut self) -> Self {
        self.swap = true;
        self
    }

    pub fn fifo_enable(mut self) -> Self {
        self.fifo_enable = true;
        self
    }

    pub fn tx_fifo_threshold(mut self, threshold: FifoThreshold) -> Self {
        self.tx_fifo_threshold = threshold;
        self
    }

    pub fn rx_fifo_threshold(mut self, threshold: FifoThreshold) -> Self {
        self.rx_fifo_threshold = threshold;
        self
    }

    pub fn tx_fifo_enable_interrupt(mut self) -> Self {
        self.tx_fifo_interrupt = true;
        self
    }

    pub fn rx_fifo_enable_interrupt(mut self) -> Self {
        self.rx_fifo_interrupt = true;
        self
    }

    /// Configure receiver timout in microseconds. Call after baudrate is set.
    pub fn receiver_timeout_us(mut self, timeout_us: u32) -> Self {
        let t = timeout_us as u64 * self.baudrate.0 as u64 / 1_000_000u64;
        self.receiver_timeout = Some(t as u32);
        self
    }
}

#[derive(Debug)]
pub struct InvalidConfig;

impl Default for BasicConfig {
    fn default() -> BasicConfig {
        let baudrate = 19_200.bps();
        BasicConfig {
            baudrate,
            wordlength: WordLength::DataBits8,
            parity: Parity::ParityNone,
            stopbits: StopBits::STOP1,
            swap: false,
        }
    }
}

impl Default for FullConfig {
    fn default() -> FullConfig {
        let baudrate = 115_200.bps();
        FullConfig {
            baudrate,
            wordlength: WordLength::DataBits8,
            parity: Parity::ParityNone,
            stopbits: StopBits::STOP1,
            swap: false,
            fifo_enable: false,
            tx_fifo_threshold: FifoThreshold::FIFO_8_BYTES,
            rx_fifo_threshold: FifoThreshold::FIFO_8_BYTES,
            tx_fifo_interrupt: false,
            rx_fifo_interrupt: false,
            receiver_timeout: None,
        }
    }
}
