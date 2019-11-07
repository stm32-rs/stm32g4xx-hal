use crate::time::Hertz;

/// Prescaler
#[derive(Clone, Copy)]
pub enum Prescaler {
    NotDivided,
    Div2,
    Div4,
    Div8,
    Div16,
    Div32,
    Div64,
    Div128,
    Div256,
    Div512,
}

/// System clock mux source
pub enum SysClockSrc {
    HSI,
    HSE(Hertz),
    HSE_BYPASS(Hertz),
    PLL, // Specifically PLL R Clock
}

/// 48MHz Clock Source
pub enum USB48MHzClkSrc {
    PLLQ,
    HSI48,
}

/// ADC Clock Source
pub enum ADCClkSrc {
    SYSCLK,
    PLLP,
}

/// U(S)ART Clock Source
pub enum USARTClkSrc {
    SYSCLK,
    HSI,
    LSE(Hertz),
    APB,
}

/// I2C Clock Source
pub enum I2CClkSrc {
    SYSCLK,
    HSI,
    APB,
}

/// SAI Clock Source
pub enum SAI1ClkSrc {
    I2S_CKIN(Hertz),
    SYSCLK,
    PLLQ,
    HSI,
}

/// QUADSPI Kernel Clock Source
pub enum QUADSPIClkSrc {
    SYSCLK,
    PLLQ,
    HSI,
}

/// LPTIM1 Clock Source
pub enum LPTIM1ClkSrc {
    LSI,
    LSE,
    HSI,
    APB,
    EXT, // See datasheet for details.
}

/// RTC Clock Source
pub enum RTCClkSrc {
    LSE,
    LSI,
    HSE_BY32(Hertz)
}

/// FDCAN1 Clock Source
pub enum FDCAN1ClkSrc {
    HSE(Hertz),
    PLLQ,
    PCLK,
}

/// Microcontroller clock output source
pub enum MCOSrc {
    LSE,
    LSI,
    HSI,
    HSE(Hertz),
    SYSCLK,
    PLL, // Specifically PLL R Clock
    HSI48
}

/// Low-speed clocks output source
pub enum LSCOSrc {
    LSI,
    LSE,
}

/// PLL clock input source
#[derive(Clone, Copy)]
pub enum PLLSrc {
    HSI,
    HSE(Hertz),
    HSE_BYPASS(Hertz),
}

/// PLL P Divider
#[derive(Clone, Copy)]
pub enum PLLPDiv {
    PLLP,
    Reserved,
    Div2,
    Div3,
    Div4,
    Div5,
    Div6,
    Div7,
    Div8,
    Div9,
    Div10,
    Div11,
    Div12,
    Div13,
    Div14,
    Div15,
    Div16,
    Div17,
    Div18,
    Div19,
    Div20,
    Div21,
    Div22,
    Div23,
    Div24,
    Div25,
    Div26,
    Div27,
    Div28,
    Div29,
    Div30,
    Div31,
}

impl From<u8> for PLLPDiv {
    fn from(bits: u8) -> Self {
        assert!(bits > 1 && bits <= 31);
        match bits {
            2 => PLLPDiv::Div2,
            3 => PLLPDiv::Div3,
            4 => PLLPDiv::Div4,
            5 => PLLPDiv::Div5,
            6 => PLLPDiv::Div6,
            7 => PLLPDiv::Div7,
            8 => PLLPDiv::Div8,
            9 => PLLPDiv::Div9,
            10 => PLLPDiv::Div10,
            11 => PLLPDiv::Div11,
            12 => PLLPDiv::Div12,
            13 => PLLPDiv::Div13,
            14 => PLLPDiv::Div14,
            15 => PLLPDiv::Div15,
            16 => PLLPDiv::Div16,
            17 => PLLPDiv::Div17,
            18 => PLLPDiv::Div18,
            19 => PLLPDiv::Div19,
            20 => PLLPDiv::Div20,
            21 => PLLPDiv::Div21,
            22 => PLLPDiv::Div22,
            23 => PLLPDiv::Div23,
            24 => PLLPDiv::Div24,
            25 => PLLPDiv::Div25,
            26 => PLLPDiv::Div26,
            27 => PLLPDiv::Div27,
            28 => PLLPDiv::Div28,
            29 => PLLPDiv::Div29,
            30 => PLLPDiv::Div30,
            31 => PLLPDiv::Div31,
            _  => unreachable!()
        }
    }
}

impl From<PLLPDiv> for u8 {
    fn from(div: PLLPDiv) -> Self {
        match div {
            PLLPDiv::PLLP   => 0b00000,
            PLLPDiv::Reserved => 0b00001,
            PLLPDiv::Div2   => 0b00010,
            PLLPDiv::Div3   => 0b00011,
            PLLPDiv::Div4   => 0b00100,
            PLLPDiv::Div5   => 0b00101,
            PLLPDiv::Div6   => 0b00110,
            PLLPDiv::Div7   => 0b00111,
            PLLPDiv::Div8   => 0b01000,
            PLLPDiv::Div9   => 0b01001,
            PLLPDiv::Div10  => 0b01010,
            PLLPDiv::Div11  => 0b01011,
            PLLPDiv::Div12  => 0b01100,
            PLLPDiv::Div13  => 0b01101,
            PLLPDiv::Div14  => 0b01110,
            PLLPDiv::Div15  => 0b01111,
            PLLPDiv::Div16  => 0b10000,
            PLLPDiv::Div17  => 0b10001,
            PLLPDiv::Div18  => 0b10010,
            PLLPDiv::Div19  => 0b10011,
            PLLPDiv::Div20  => 0b10100,
            PLLPDiv::Div21  => 0b10101,
            PLLPDiv::Div22  => 0b10110,
            PLLPDiv::Div23  => 0b10111,
            PLLPDiv::Div24  => 0b11000,
            PLLPDiv::Div25  => 0b11001,
            PLLPDiv::Div26  => 0b11010,
            PLLPDiv::Div27  => 0b11011,
            PLLPDiv::Div28  => 0b11100,
            PLLPDiv::Div29  => 0b11101,
            PLLPDiv::Div30  => 0b11110,
            PLLPDiv::Div31  => 0b11111,
        }
    }
}
/// R and Q PLL Divider
#[derive(Clone, Copy)]
pub enum PLLQRDiv {
    Div2,
    Div4,
    Div6,
    Div8
}

/// PLL divider
pub type PLLDiv = u8;

/// PLL multiplier
pub type PLLMul = u8;

/// PLL config
#[derive(Clone, Copy)]
pub struct PllConfig {
    pub mux: PLLSrc,
    pub m: PLLDiv,
    pub n: PLLMul,
    pub r: Option<PLLQRDiv>,
    pub q: Option<PLLQRDiv>,
    pub p: Option<PLLPDiv>,
}

/// This default gives a 64MHz PLL R Clk, no Q or P clocks.
impl Default for PllConfig {
    fn default() -> PllConfig {
        PllConfig {
            mux: PLLSrc::HSI,
            m: 2,
            n: 16,
            r: Some(PLLQRDiv::Div2),
            q: None,
            p: None,
        }
    }
}

/// Clocks configutation
pub struct Config {
    pub(crate) sys_mux: SysClockSrc,
    pub(crate) pll_cfg: PllConfig,
    pub(crate) ahb_psc: Prescaler,
    pub(crate) apb1_psc: Prescaler,
    pub(crate) apb2_psc: Prescaler,
}

impl Config {
    pub fn new(mux: SysClockSrc) -> Self {
        Config::default().clock_src(mux)
    }

    pub fn pll() -> Self {
        Config::default().clock_src(SysClockSrc::PLL)
    }

    pub fn hsi() -> Self {
        Config::default().clock_src(SysClockSrc::HSI)
    }

//    pub fn lsi() -> Self {
//        Config::default().clock_src(SysClockSrc::LSI)
//    }

    pub fn clock_src(mut self, mux: SysClockSrc) -> Self {
        self.sys_mux = mux;
        self
    }

    pub fn pll_cfg(mut self, cfg: PllConfig) -> Self {
        self.pll_cfg = cfg;
        self
    }

    pub fn ahb_psc(mut self, psc: Prescaler) -> Self {
        self.ahb_psc = psc;
        self
    }

    pub fn apb1_psc(mut self, psc: Prescaler) -> Self {
        self.apb1_psc = psc;
        self
    }

    pub fn apb2_psc(mut self, psc: Prescaler) -> Self {
        self.apb2_psc = psc;
        self
    }
}

impl Default for Config {
    fn default() -> Config {
        Config {
            sys_mux: SysClockSrc::HSI,
            pll_cfg: PllConfig::default(),
            ahb_psc: Prescaler::NotDivided,
            apb1_psc: Prescaler::NotDivided,
            apb2_psc: Prescaler::NotDivided,
        }
    }
}
