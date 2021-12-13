use crate::stm32::{rcc, FLASH, PWR, RCC};
use crate::time::{Hertz, U32Ext};

mod clockout;
mod config;
mod enable;

pub use clockout::*;
pub use config::*;

pub trait Instance: crate::Sealed + Enable + Reset + GetBusFreq {}

/// HSI speed
pub const HSI_FREQ: u32 = 16_000_000;

/// Clock frequencies
#[derive(Clone, Copy, Debug)]
pub struct Clocks {
    /// System frequency
    pub sys_clk: Hertz,
    /// Core frequency
    pub core_clk: Hertz,
    /// AHB frequency
    pub ahb_clk: Hertz,
    /// APB frequency
    pub apb_clk: Hertz,
    /// APB timers frequency
    pub apb_tim_clk: Hertz,
    /// PLL frequency
    pub pll_clk: PLLClocks,
}

/// PLL Clock frequencies
#[derive(Clone, Copy, Debug)]
pub struct PLLClocks {
    /// R frequency
    pub r: Hertz,
    /// Q frequency
    pub q: Option<Hertz>,
    /// P frequency
    pub p: Option<Hertz>,
}

impl Default for Clocks {
    fn default() -> Clocks {
        let freq = HSI_FREQ.hz();
        Clocks {
            sys_clk: freq,
            ahb_clk: freq,
            core_clk: freq,
            apb_clk: freq,
            apb_tim_clk: freq,
            pll_clk: PLLClocks {
                r: 32.mhz(),
                q: None,
                p: None,
            },
        }
    }
}

/// Constrained RCC peripheral
pub struct Rcc {
    /// Clock configuration
    pub clocks: Clocks,
    pub(crate) rb: RCC,
}

impl Rcc {
    /// Apply clock configuration
    pub fn freeze(self, rcc_cfg: Config) -> Self {
        let pll_clk = self.config_pll(rcc_cfg.pll_cfg);

        let (sys_clk, sw_bits) = match rcc_cfg.sys_mux {
            SysClockSrc::HSI => {
                self.enable_hsi();
                (HSI_FREQ.hz(), 0b01)
            }
            SysClockSrc::HSE(freq) => {
                self.enable_hse(false);
                (freq, 0b10)
            }
            SysClockSrc::PLL => (pll_clk.r, 0b11),
        };

        let sys_freq = sys_clk.0;
        let (ahb_freq, ahb_psc_bits) = match rcc_cfg.ahb_psc {
            Prescaler::Div2 => (sys_freq / 2, 0b1000),
            Prescaler::Div4 => (sys_freq / 4, 0b1001),
            Prescaler::Div8 => (sys_freq / 8, 0b1010),
            Prescaler::Div16 => (sys_freq / 16, 0b1011),
            Prescaler::Div64 => (sys_freq / 64, 0b1100),
            Prescaler::Div128 => (sys_freq / 128, 0b1101),
            Prescaler::Div256 => (sys_freq / 256, 0b1110),
            Prescaler::Div512 => (sys_freq / 512, 0b1111),
            _ => (sys_clk.0, 0b0000),
        };
        let (apb_freq, apb_psc_bits) = match rcc_cfg.apb_psc {
            Prescaler::Div2 => (sys_clk.0 / 2, 0b100),
            Prescaler::Div4 => (sys_clk.0 / 4, 0b101),
            Prescaler::Div8 => (sys_clk.0 / 8, 0b110),
            Prescaler::Div16 => (sys_clk.0 / 16, 0b111),
            _ => (sys_clk.0, 0b000),
        };

        unsafe {
            // Adjust flash wait states
            let flash = &(*FLASH::ptr());
            flash.acr.modify(|_, w| {
                w.latency().bits(if sys_clk.0 <= 24_000_000 {
                    0b000
                } else if sys_clk.0 <= 48_000_000 {
                    0b001
                } else {
                    0b010
                })
            })
        }

        self.rb.cfgr.modify(|_, w| unsafe {
            w.hpre()
                .bits(ahb_psc_bits)
                .ppre1()
                .bits(apb_psc_bits)
                .ppre2()
                .bits(apb_psc_bits)
                .sw()
                .bits(sw_bits)
        });

        while self.rb.cfgr.read().sws().bits() != sw_bits {}

        Rcc {
            rb: self.rb,
            clocks: Clocks {
                pll_clk,
                sys_clk,
                core_clk: ahb_freq.hz(),
                ahb_clk: ahb_freq.hz(),
                apb_clk: apb_freq.hz(),
                apb_tim_clk: apb_freq.hz(),
            },
        }
    }

    pub fn unlock_rtc(&mut self) {
        self.rb.apb1enr1.modify(|_, w| w.pwren().set_bit());
        let pwr = unsafe { &(*PWR::ptr()) };
        pwr.cr1.modify(|_, w| w.dbp().set_bit());
    }

    fn config_pll(&self, pll_cfg: PllConfig) -> PLLClocks {
        assert!(pll_cfg.m > 0 && pll_cfg.m <= 8);
        assert!(pll_cfg.r > 1 && pll_cfg.r <= 8);

        // Disable PLL
        self.rb.cr.write(|w| w.pllon().clear_bit());
        while self.rb.cr.read().pllrdy().bit_is_set() {}

        let (freq, pll_sw_bits) = match pll_cfg.mux {
            PLLSrc::HSI => {
                self.enable_hsi();
                (HSI_FREQ, 0b10)
            }
            PLLSrc::HSE(freq) => {
                self.enable_hse(false);
                (freq.0, 0b11)
            }
            PLLSrc::HSE_BYPASS(freq) => {
                self.enable_hse(true);
                (freq.0, 0b11)
            }
        };

        let pll_freq = freq / (pll_cfg.m as u32) * (pll_cfg.n as u32);
        let r = (pll_freq / (pll_cfg.r as u32)).hz();
        let q = match pll_cfg.q {
            Some(div) if div > 1 && div <= 8 => {
                self.rb.pllcfgr.write(move |w| w.pllq().bits(div - 1));
                let req = freq / div as u32;
                Some(req.hz())
            }
            _ => None,
        };

        let p = match pll_cfg.p {
            Some(div) if div > 1 && div <= 8 => {
                self.rb.pllcfgr.write(move |w| w.pllp().bit(div == 17));
                let req = freq / div as u32;
                Some(req.hz())
            }
            _ => None,
        };

        self.rb.pllcfgr.write(move |w| unsafe {
            w.pllsrc()
                .bits(pll_sw_bits)
                .pllm()
                .bits(pll_cfg.m - 1)
                .plln()
                .bits(pll_cfg.n)
                .pllr()
                .bits((pll_cfg.r / 2) - 1)
                .pllren()
                .set_bit()
        });

        // Enable PLL
        self.rb.cr.modify(|_, w| w.pllon().set_bit());
        while self.rb.cr.read().pllrdy().bit_is_clear() {}

        PLLClocks { r, q, p }
    }

    pub(crate) fn enable_hsi(&self) {
        self.rb.cr.write(|w| w.hsion().set_bit());
        while self.rb.cr.read().hsirdy().bit_is_clear() {}
    }

    pub(crate) fn enable_hse(&self, bypass: bool) {
        self.rb
            .cr
            .write(|w| w.hseon().set_bit().hsebyp().bit(bypass));
        while self.rb.cr.read().hserdy().bit_is_clear() {}
    }

    pub(crate) fn enable_lse(&self, bypass: bool) {
        self.rb
            .bdcr
            .write(|w| w.lseon().set_bit().lsebyp().bit(bypass));
        while self.rb.bdcr.read().lserdy().bit_is_clear() {}
    }

    pub(crate) fn enable_lsi(&self) {
        self.rb.csr.write(|w| w.lsion().set_bit());
        while self.rb.csr.read().lsirdy().bit_is_clear() {}
    }
}

/// Extension trait that constrains the `RCC` peripheral
pub trait RccExt {
    /// Constrains the `RCC` peripheral so it plays nicely with the other abstractions
    fn constrain(self) -> Rcc;

    /// Constrains the `RCC` peripheral and apply clock configuration
    fn freeze(self, rcc_cfg: Config) -> Rcc;
}

impl RccExt for RCC {
    fn constrain(self) -> Rcc {
        Rcc {
            rb: self,
            clocks: Clocks::default(),
        }
    }

    fn freeze(self, rcc_cfg: Config) -> Rcc {
        self.constrain().freeze(rcc_cfg)
    }
}

use crate::stm32::rcc::RegisterBlock as RccRB;

pub struct AHB1 {
    _0: (),
}
impl AHB1 {
    #[inline(always)]
    fn enr(rcc: &RccRB) -> &rcc::AHB1ENR {
        &rcc.ahb1enr
    }
    #[inline(always)]
    fn rstr(rcc: &RccRB) -> &rcc::AHB1RSTR {
        &rcc.ahb1rstr
    }
    #[inline(always)]
    fn smenr(rcc: &RccRB) -> &rcc::AHB1SMENR {
        &rcc.ahb1smenr
    }
}

pub struct AHB2 {
    _0: (),
}
impl AHB2 {
    #[inline(always)]
    fn enr(rcc: &RccRB) -> &rcc::AHB2ENR {
        &rcc.ahb2enr
    }
    #[inline(always)]
    fn rstr(rcc: &RccRB) -> &rcc::AHB2RSTR {
        &rcc.ahb2rstr
    }
    #[inline(always)]
    fn smenr(rcc: &RccRB) -> &rcc::AHB2SMENR {
        &rcc.ahb2smenr
    }
}

pub struct AHB3 {
    _0: (),
}
impl AHB3 {
    #[inline(always)]
    fn enr(rcc: &RccRB) -> &rcc::AHB3ENR {
        &rcc.ahb3enr
    }
    #[inline(always)]
    fn rstr(rcc: &RccRB) -> &rcc::AHB3RSTR {
        &rcc.ahb3rstr
    }
    #[inline(always)]
    fn smenr(rcc: &RccRB) -> &rcc::AHB3SMENR {
        &rcc.ahb3smenr
    }
}

pub struct APB1_1 {
    _0: (),
}
impl APB1_1 {
    #[inline(always)]
    fn enr(rcc: &RccRB) -> &rcc::APB1ENR1 {
        &rcc.apb1enr1
    }
    #[inline(always)]
    fn rstr(rcc: &RccRB) -> &rcc::APB1RSTR1 {
        &rcc.apb1rstr1
    }
    #[inline(always)]
    fn smenr(rcc: &RccRB) -> &rcc::APB1SMENR1 {
        &rcc.apb1smenr1
    }
}

pub struct APB1_2 {
    _0: (),
}
impl APB1_2 {
    #[inline(always)]
    fn enr(rcc: &RccRB) -> &rcc::APB1ENR2 {
        &rcc.apb1enr2
    }
    #[inline(always)]
    fn rstr(rcc: &RccRB) -> &rcc::APB1RSTR2 {
        &rcc.apb1rstr2
    }
    #[inline(always)]
    fn smenr(rcc: &RccRB) -> &rcc::APB1SMENR2 {
        &rcc.apb1smenr2
    }
}

pub struct APB2 {
    _0: (),
}
impl APB2 {
    #[inline(always)]
    fn enr(rcc: &RccRB) -> &rcc::APB2ENR {
        &rcc.apb2enr
    }
    #[inline(always)]
    fn rstr(rcc: &RccRB) -> &rcc::APB2RSTR {
        &rcc.apb2rstr
    }
    #[inline(always)]
    fn smenr(rcc: &RccRB) -> &rcc::APB2SMENR {
        &rcc.apb2smenr
    }
}

/// Bus associated to peripheral
pub trait RccBus: crate::Sealed {
    /// Bus type;
    type Bus;
}

/// Enable/disable peripheral
pub trait Enable: RccBus {
    fn enable(rcc: &RccRB);
    fn disable(rcc: &RccRB);
}

/// Reset peripheral
pub trait Reset: RccBus {
    fn reset(rcc: &RccRB);
}

pub trait GetBusFreq {
    fn get_frequency(clocks: &Clocks) -> Hertz;
    fn get_timer_frequency(clocks: &Clocks) -> Hertz {
        Self::get_frequency(clocks)
    }
}

impl<T> GetBusFreq for T
where
    T: RccBus,
    T::Bus: GetBusFreq,
{
    fn get_frequency(clocks: &Clocks) -> Hertz {
        T::Bus::get_frequency(clocks)
    }
    fn get_timer_frequency(clocks: &Clocks) -> Hertz {
        T::Bus::get_timer_frequency(clocks)
    }
}

impl GetBusFreq for AHB1 {
    fn get_frequency(clocks: &Clocks) -> Hertz {
        clocks.ahb_clk
    }
}

impl GetBusFreq for AHB2 {
    fn get_frequency(clocks: &Clocks) -> Hertz {
        clocks.ahb_clk
    }
}

impl GetBusFreq for AHB3 {
    fn get_frequency(clocks: &Clocks) -> Hertz {
        clocks.ahb_clk
    }
}

impl GetBusFreq for APB1_1 {
    fn get_frequency(clocks: &Clocks) -> Hertz {
        clocks.apb_clk
    }
    fn get_timer_frequency(clocks: &Clocks) -> Hertz {
        // let pclk_mul = if clocks.ppre1 == 1 { 1 } else { 2 };
        // Hertz(clocks.pclk1.0 * pclk_mul)
        clocks.apb_tim_clk
    }
}

impl GetBusFreq for APB1_2 {
    fn get_frequency(clocks: &Clocks) -> Hertz {
        clocks.apb_clk
    }
    fn get_timer_frequency(clocks: &Clocks) -> Hertz {
        // let pclk_mul = if clocks.ppre1 == 1 { 1 } else { 2 };
        // Hertz(clocks.pclk1.0 * pclk_mul)
        clocks.apb_tim_clk
    }
}

impl GetBusFreq for APB2 {
    fn get_frequency(clocks: &Clocks) -> Hertz {
        clocks.apb_clk
    }
    fn get_timer_frequency(clocks: &Clocks) -> Hertz {
        // let pclk_mul = if clocks.ppre2 == 1 { 1 } else { 2 };
        // Hertz(clocks.pclk2.0 * pclk_mul)
        clocks.apb_tim_clk
    }
}
