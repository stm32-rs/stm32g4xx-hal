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
    LSI,
    PLL,
    HSI,
    HSE(Hertz),
    HSE_BYPASS(Hertz),
    LSE(Hertz),
    LSE_BYPASS(Hertz),
}

/// Microcontroller clock output source
pub enum MCOSrc {
    LSI,
    PLL,
    SysClk,
    HSI,
    HSE,
    LSE,
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
    pub r: PLLDiv,
    pub q: Option<PLLDiv>,
    pub p: Option<PLLDiv>,
}

impl Default for PllConfig {
    fn default() -> PllConfig {
        PllConfig {
            mux: PLLSrc::HSI,
            m: 1,
            n: 8,
            r: 2,
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

    pub fn lsi() -> Self {
        Config::default().clock_src(SysClockSrc::LSI)
    }

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
