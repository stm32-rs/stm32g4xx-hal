#[derive(Copy, Clone, Debug)]
pub struct DeadtimeConfig {
    /// Prescaler for both rising and falling deadtime
    pub(crate) prescaler: DeadtimePrescaler,

    /// 9-bits
    pub(crate) deadtime_rising_value: u16,

    /// Is deadtime negative
    pub(crate) deadtime_rising_sign: bool,

    /// 9-bits
    pub(crate) deadtime_falling_value: u16,

    /// Is deadtime negative
    pub(crate) deadtime_falling_sign: bool,
}

impl DeadtimeConfig {
    /// See RM0440 Table 221 'Deadtime resolution and max absolute values'
    pub fn prescaler(mut self, value: DeadtimePrescaler) -> Self {
        self.prescaler = value;
        self
    }

    /// Panic if value can not fit in 9 bits
    pub fn deadtime_rising_value(mut self, value: u16) -> Self {
        // 9 bits
        assert!(value < (1 << 9));

        self.deadtime_rising_value = value;

        self
    }

    pub fn deadtime_rising_sign(mut self, is_negative: bool) -> Self {
        self.deadtime_rising_sign = is_negative;
        self
    }

    /// Panic if value can not fit in 9 bits
    pub fn deadtime_falling_value(mut self, value: u16) -> Self {
        // 9 bits
        assert!(value < (1 << 9));

        self.deadtime_falling_value = value;

        self
    }

    pub fn deadtime_falling_sign(mut self, is_negative: bool) -> Self {
        self.deadtime_falling_sign = is_negative;
        self
    }
}

impl Default for DeadtimeConfig {
    fn default() -> Self {
        Self {
            prescaler: DeadtimePrescaler::Thrtim,
            deadtime_rising_value: 170, // about 1us when f_sys = 170MHz
            deadtime_rising_sign: false,
            deadtime_falling_value: 170, // about 1us when f_sys = 170MHz
            deadtime_falling_sign: false,
        }
    }
}

#[derive(Copy, Clone, Debug)]
pub enum DeadtimePrescaler {
    ThrtimDiv8 = 0b000,
    ThrtimDiv4 = 0b001,
    ThrtimDiv2 = 0b010,
    Thrtim = 0b011,
    ThrtimMul2 = 0b100,
    ThrtimMul4 = 0b101,
    ThrtimMul8 = 0b110,
    ThrtimMul16 = 0b111,
}
