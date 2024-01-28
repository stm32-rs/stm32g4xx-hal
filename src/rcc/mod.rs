use crate::pwr::{self, PowerConfiguration};
use crate::stm32::{rcc, FLASH, PWR, RCC};
use crate::time::{Hertz, RateExtU32};

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
    /// APB 1 frequency
    pub apb1_clk: Hertz,
    /// APB 1 timers frequency (Timers 2-7)
    pub apb1_tim_clk: Hertz,
    /// APB 2 frequency
    pub apb2_clk: Hertz,
    /// APB 2 timers frequency (Timers 1, 8, 20, 15, 16, 17 and HRTIM1)
    pub apb2_tim_clk: Hertz,
    /// PLL frequency
    pub pll_clk: PLLClocks,
}

/// PLL Clock frequencies
#[derive(Clone, Copy, Debug)]
pub struct PLLClocks {
    /// R frequency
    pub r: Option<Hertz>,
    /// Q frequency
    pub q: Option<Hertz>,
    /// P frequency
    pub p: Option<Hertz>,
}

impl Default for Clocks {
    fn default() -> Clocks {
        let freq = HSI_FREQ.Hz();
        Clocks {
            sys_clk: freq,
            ahb_clk: freq,
            core_clk: freq,
            apb1_clk: freq,
            apb1_tim_clk: freq,
            apb2_clk: freq,
            apb2_tim_clk: freq,
            pll_clk: PLLClocks {
                r: None,
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
    pub fn freeze(mut self, rcc_cfg: Config, pwr_cfg: PowerConfiguration) -> Self {
        let pll_clk = self.config_pll(rcc_cfg.pll_cfg);

        let (sys_clk, sw_bits) = match rcc_cfg.sys_mux {
            SysClockSrc::HSI => {
                self.enable_hsi();
                (HSI_FREQ.Hz(), 0b01)
            }
            SysClockSrc::HSE(freq) => {
                self.enable_hse(false);
                (freq, 0b10)
            }
            SysClockSrc::PLL => {
                // If PLL is selected as sysclock then the r output should have been configured
                if pll_clk.r.is_none() {
                    panic!("PLL output selected as sysclock but PLL output R is not configured")
                }
                (pll_clk.r.unwrap(), 0b11)
            }
        };

        let sys_freq = sys_clk.raw();
        let (ahb_freq, ahb_psc_bits) = match rcc_cfg.ahb_psc {
            Prescaler::Div2 => (sys_freq / 2, 0b1000),
            Prescaler::Div4 => (sys_freq / 4, 0b1001),
            Prescaler::Div8 => (sys_freq / 8, 0b1010),
            Prescaler::Div16 => (sys_freq / 16, 0b1011),
            Prescaler::Div64 => (sys_freq / 64, 0b1100),
            Prescaler::Div128 => (sys_freq / 128, 0b1101),
            Prescaler::Div256 => (sys_freq / 256, 0b1110),
            Prescaler::Div512 => (sys_freq / 512, 0b1111),
            _ => (sys_freq, 0b0000),
        };
        let (apb1_freq, apb1_psc_bits) = match rcc_cfg.apb1_psc {
            Prescaler::Div2 => (sys_freq / 2, 0b100),
            Prescaler::Div4 => (sys_freq / 4, 0b101),
            Prescaler::Div8 => (sys_freq / 8, 0b110),
            Prescaler::Div16 => (sys_freq / 16, 0b111),
            _ => (sys_freq, 0b000),
        };
        let (apb2_freq, apb2_psc_bits) = match rcc_cfg.apb2_psc {
            Prescaler::Div2 => (sys_freq / 2, 0b100),
            Prescaler::Div4 => (sys_freq / 4, 0b101),
            Prescaler::Div8 => (sys_freq / 8, 0b110),
            Prescaler::Div16 => (sys_freq / 16, 0b111),
            _ => (sys_freq, 0b000),
        };

        let present_vos_mode = pwr::current_vos();
        let target_vos_mode = pwr_cfg.vos();

        match (present_vos_mode, target_vos_mode) {
            // From VoltageScale::Range1 boost
            (
                pwr::VoltageScale::Range1 { enable_boost: true },
                pwr::VoltageScale::Range1 { enable_boost: true },
            ) => (), // No change
            (
                pwr::VoltageScale::Range1 { enable_boost: true },
                pwr::VoltageScale::Range1 {
                    enable_boost: false,
                },
            ) => todo!(),
            (pwr::VoltageScale::Range1 { enable_boost: true }, pwr::VoltageScale::Range2) => {
                todo!()
            }

            // From VoltageScale::Range1 normal
            (
                pwr::VoltageScale::Range1 {
                    enable_boost: false,
                },
                pwr::VoltageScale::Range1 { enable_boost: true },
            ) => {
                self.range1_normal_to_boost(
                    &pwr_cfg,
                    sys_freq,
                    apb1_psc_bits,
                    apb2_psc_bits,
                    sw_bits,
                    ahb_psc_bits,
                );
            }
            (
                pwr::VoltageScale::Range1 {
                    enable_boost: false,
                },
                pwr::VoltageScale::Range1 {
                    enable_boost: false,
                },
            ) => (), // No change
            (
                pwr::VoltageScale::Range1 {
                    enable_boost: false,
                },
                pwr::VoltageScale::Range2,
            ) => todo!(),

            // From VoltageScale::Range2
            (pwr::VoltageScale::Range2, pwr::VoltageScale::Range1 { enable_boost: true }) => {
                todo!()
            }
            (
                pwr::VoltageScale::Range2,
                pwr::VoltageScale::Range1 {
                    enable_boost: false,
                },
            ) => todo!(),
            (pwr::VoltageScale::Range2, pwr::VoltageScale::Range2) => (), // No change
        }

        Self::configure_wait_states(&pwr_cfg, sys_freq);

        self.rb.cfgr.modify(|_, w| unsafe {
            w.hpre()
                .bits(ahb_psc_bits)
                .ppre1()
                .bits(apb1_psc_bits)
                .ppre2()
                .bits(apb2_psc_bits)
                .sw()
                .bits(sw_bits)
        });

        while self.rb.cfgr.read().sws().bits() != sw_bits {}

        // From RM:
        // The timer clock frequencies are automatically defined by hardware. There are two cases:
        // 1. If the APB prescaler equals 1, the timer clock frequencies are set to the same
        // frequency as that of the APB domain.
        // 2. Otherwise, they are set to twice (×2) the frequency of the APB domain.
        let apb1_tim_clk = match rcc_cfg.apb1_psc {
            Prescaler::NotDivided => apb1_freq,
            _ => apb1_freq * 2,
        };

        let apb2_tim_clk = match rcc_cfg.apb2_psc {
            Prescaler::NotDivided => apb2_freq,
            _ => apb2_freq * 2,
        };

        Rcc {
            rb: self.rb,
            clocks: Clocks {
                pll_clk,
                sys_clk,
                core_clk: ahb_freq.Hz(),
                ahb_clk: ahb_freq.Hz(),
                apb1_clk: apb1_freq.Hz(),
                apb1_tim_clk: apb1_tim_clk.Hz(),
                apb2_clk: apb2_freq.Hz(),
                apb2_tim_clk: apb2_tim_clk.Hz(),
            },
        }
    }

    pub fn unlock_rtc(&mut self) {
        self.rb.apb1enr1.modify(|_, w| w.pwren().set_bit());
        let pwr = unsafe { &(*PWR::ptr()) };
        pwr.cr1.modify(|_, w| w.dbp().set_bit());
    }

    fn config_pll(&self, pll_cfg: PllConfig) -> PLLClocks {
        // Disable PLL
        self.rb.cr.modify(|_, w| w.pllon().clear_bit());
        while self.rb.cr.read().pllrdy().bit_is_set() {}

        // Enable the input clock feeding the PLL
        let (pll_input_freq, pll_src_bits) = match pll_cfg.mux {
            PLLSrc::HSI => {
                self.enable_hsi();
                (HSI_FREQ, 0b10)
            }
            PLLSrc::HSE(freq) => {
                self.enable_hse(false);
                (freq.raw(), 0b11)
            }
            PLLSrc::HSE_BYPASS(freq) => {
                self.enable_hse(true);
                (freq.raw(), 0b11)
            }
        };

        // Calculate the frequency of the internal PLL VCO.
        let pll_freq = pll_input_freq / pll_cfg.m.divisor() * pll_cfg.n.multiplier();

        // Calculate the output frequencies for the P, Q, and R outputs
        let p = pll_cfg
            .p
            .map(|p| ((pll_freq / p.divisor()).Hz(), p.register_setting()));

        let q = pll_cfg
            .q
            .map(|q| ((pll_freq / q.divisor()).Hz(), q.register_setting()));

        let r = pll_cfg
            .r
            .map(|r| ((pll_freq / r.divisor()).Hz(), r.register_setting()));

        // Set the M input divider, the N multiplier for the PLL, and the PLL source.
        self.rb.pllcfgr.modify(|_, w| unsafe {
            // Set N, M, and source
            let w = w
                .plln()
                .bits(pll_cfg.n.register_setting())
                .pllm()
                .bits(pll_cfg.m.register_setting())
                .pllsrc()
                .bits(pll_src_bits);

            // Set and enable P if requested
            let w = match p {
                Some((_, register_setting)) => {
                    w.pllpdiv().bits(register_setting).pllpen().set_bit()
                }
                None => w,
            };

            // Set and enable Q if requested
            let w = match q {
                Some((_, register_setting)) => w.pllq().bits(register_setting).pllqen().set_bit(),
                None => w,
            };

            // Set and enable R if requested
            let w = match r {
                Some((_, register_setting)) => w.pllr().bits(register_setting).pllren().set_bit(),
                None => w,
            };

            w
        });

        // Enable PLL
        self.rb.cr.modify(|_, w| w.pllon().set_bit());
        while self.rb.cr.read().pllrdy().bit_is_clear() {}

        PLLClocks {
            r: r.map(|r| r.0),
            q: q.map(|q| q.0),
            p: p.map(|p| p.0),
        }
    }

    fn configure_wait_states(pwr_cfg: &PowerConfiguration, sys_freq: u32) {
        // Calculate wait states depending on voltage scale and sys_freq
        //
        // See 'Number of wait states according to CPU clock (HCLK) frequency' in RM0440
        let latency = match pwr_cfg.vos() {
            pwr::VoltageScale::Range1 { enable_boost: true } => match sys_freq {
                0..=34_000_000 => 0b0000,
                34_000_001..=68_000_000 => 0b0001,
                68_000_001..=102_000_000 => 0b0010,
                102_000_001..=136_000_000 => 0b0011,
                136_000_001..=170_000_000 => 0b0100,
                170_000_001.. => panic!(
                    "Too high f_sys: {}, max with voltage scale in 'range1 boost mode' is: 170MHz",
                    sys_freq
                ),
            },
            pwr::VoltageScale::Range1 {
                enable_boost: false,
            } => match sys_freq {
                0..=30_000_000 => 0b0000,
                30_000_001..=60_000_000 => 0b0001,
                60_000_001..=90_000_000 => 0b0010,
                90_000_001..=120_000_000 => 0b0011,
                120_000_001..=150_000_000 => 0b0100,
                150_000_001.. => panic!(
                    "Too high f_sys: {}, max with voltage scale in 'range1 normal mode' is: 150MHz",
                    sys_freq
                ),
            },
            pwr::VoltageScale::Range2 => match sys_freq {
                0..=12_000_000 => 0b0000,
                12_000_001..=24_000_000 => 0b0001,
                24_000_001..=26_000_000 => 0b0010,
                26_000_001.. => panic!(
                    "Too high f_sys: {}, max with voltage scale in 'range2' is: 26MHz",
                    sys_freq
                ),
            },
        };

        unsafe {
            // Adjust flash wait states
            let flash = &(*FLASH::ptr());
            flash.acr.modify(|_, w| w.latency().bits(latency))
        }
    }

    fn range1_normal_to_boost(
        &mut self,
        pwr_cfg: &PowerConfiguration,
        sys_freq: u32,
        apb1_psc_bits: u8,
        apb2_psc_bits: u8,
        sw_bits: u8,
        ahb_psc_bits: u8,
    ) {
        // (From RM0440 chapter "Power control (PWR)")
        // The sequence to switch from Range11 normal mode to Range1 boost mode is:
        // 1. The system clock must be divided by 2 using the AHB prescaler before switching to a
        // higher system frequency.
        let half_apb = (self.rb.cfgr.read().hpre().bits() + 1).clamp(0b1000, 0b1111);
        self.rb
            .cfgr
            .modify(|_r, w| unsafe { w.hpre().bits(half_apb) });
        while self.rb.cfgr.read().hpre().bits() != half_apb {}

        // 2. Clear the R1MODE bit is in the PWR_CR5 register.
        unsafe { pwr::set_boost(true) };

        // 3. Adjust the number of wait states according to the new frequency target in range1 boost mode
        Self::configure_wait_states(pwr_cfg, sys_freq);

        // 4. Configure and switch to new system frequency.
        self.rb.cfgr.modify(|_, w| unsafe {
            w.ppre1()
                .bits(apb1_psc_bits)
                .ppre2()
                .bits(apb2_psc_bits)
                .sw()
                .bits(sw_bits)
        });

        while self.rb.cfgr.read().sws().bits() != sw_bits {}

        // 5. Wait for at least 1us and then reconfigure the AHB prescaler to get the needed HCLK
        // clock frequency.
        let us_per_s = 1_000_000;
        // Number of cycles @ sys_freq for 1us, rounded up, this will
        // likely end up being 2us since the AHB prescaler is changed
        let delay_cycles = (sys_freq + us_per_s - 1) / us_per_s;
        cortex_m::asm::delay(delay_cycles);

        self.rb
            .cfgr
            .modify(|_, w| unsafe { w.hpre().bits(ahb_psc_bits) });
    }

    pub(crate) fn enable_hsi(&self) {
        self.rb.cr.modify(|_, w| w.hsion().set_bit());
        while self.rb.cr.read().hsirdy().bit_is_clear() {}
    }

    pub(crate) fn enable_hse(&self, bypass: bool) {
        self.rb
            .cr
            .modify(|_, w| w.hseon().set_bit().hsebyp().bit(bypass));
        while self.rb.cr.read().hserdy().bit_is_clear() {}
    }

    pub(crate) fn enable_lse(&self, bypass: bool) {
        self.rb
            .bdcr
            .modify(|_, w| w.lseon().set_bit().lsebyp().bit(bypass));
        while self.rb.bdcr.read().lserdy().bit_is_clear() {}
    }

    pub(crate) fn enable_lsi(&self) {
        self.rb.csr.modify(|_, w| w.lsion().set_bit());
        while self.rb.csr.read().lsirdy().bit_is_clear() {}
    }

    pub fn get_reset_reason(&self) -> ResetReason {
        let csr = self.rb.csr.read();

        ResetReason {
            low_power: csr.lpwrstf().bit(),
            window_watchdog: csr.wwdgrstf().bit(),
            independent_watchdog: csr.iwdgrstf().bit(),
            software: csr.sftrstf().bit(),
            brown_out: csr.borrstf().bit(),
            reset_pin: csr.pinrstf().bit(),
            option_byte: csr.oblrstf().bit(),
        }
    }

    pub fn clear_reset_reason(&mut self) {
        self.rb.csr.modify(|_, w| w.rmvf().set_bit());
    }
}

pub struct ResetReason {
    /// Low-power reset flag
    ///
    /// Set by hardware when a reset occurs to illegal Stop, Standby or Shutdown mode entry.
    pub low_power: bool,

    /// Window watchdog reset flag
    ///
    /// Set by hardware when a window watchdog reset occurs.
    pub window_watchdog: bool,

    /// Independent window watchdog reset flag
    ///
    /// Set by hardware when an independent watchdog reset occurs.
    pub independent_watchdog: bool,

    /// Software reset flag
    ///
    /// Set by hardware when a software reset occurs.
    pub software: bool,

    /// Brown out reset flag
    ///
    /// Set by hardware when a brown out reset occurs.
    pub brown_out: bool,

    /// Pin reset flag
    ///
    /// Set by hardware when a reset from the NRST pin occurs.
    pub reset_pin: bool,

    /// Option byte loader reset flag
    ///
    /// Set by hardware when a reset from the Option Byte loading occurs.
    pub option_byte: bool,
}

/// Extension trait that constrains the `RCC` peripheral
pub trait RccExt {
    /// Constrains the `RCC` peripheral so it plays nicely with the other abstractions
    fn constrain(self) -> Rcc;

    /// Constrains the `RCC` peripheral and apply clock configuration
    fn freeze(self, rcc_cfg: Config, pwr_config: PowerConfiguration) -> Rcc;
}

impl RccExt for RCC {
    fn constrain(self) -> Rcc {
        Rcc {
            rb: self,
            clocks: Clocks::default(),
        }
    }

    fn freeze(self, rcc_cfg: Config, pwr_config: PowerConfiguration) -> Rcc {
        self.constrain().freeze(rcc_cfg, pwr_config)
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
    #[allow(unused)]
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
    #[allow(unused)]
    #[inline(always)]
    fn smenr(rcc: &RccRB) -> &rcc::AHB2SMENR {
        &rcc.ahb2smenr
    }
}

pub struct AHB3 {
    _0: (),
}
impl AHB3 {
    #[allow(unused)]
    #[inline(always)]
    fn enr(rcc: &RccRB) -> &rcc::AHB3ENR {
        &rcc.ahb3enr
    }
    #[allow(unused)]
    #[inline(always)]
    fn rstr(rcc: &RccRB) -> &rcc::AHB3RSTR {
        &rcc.ahb3rstr
    }
    #[allow(unused)]
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
    #[allow(unused)]
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
    #[allow(unused)]
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
    #[allow(unused)]
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
    fn enable_for_sleep_stop(rcc: &RccRB);
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
        clocks.apb1_clk
    }
    fn get_timer_frequency(clocks: &Clocks) -> Hertz {
        clocks.apb1_tim_clk
    }
}

impl GetBusFreq for APB1_2 {
    fn get_frequency(clocks: &Clocks) -> Hertz {
        clocks.apb1_clk
    }
    fn get_timer_frequency(clocks: &Clocks) -> Hertz {
        clocks.apb1_tim_clk
    }
}

impl GetBusFreq for APB2 {
    fn get_frequency(clocks: &Clocks) -> Hertz {
        clocks.apb2_clk
    }
    fn get_timer_frequency(clocks: &Clocks) -> Hertz {
        clocks.apb2_tim_clk
    }
}
