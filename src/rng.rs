use core::cmp;
use core::mem;

use crate::hal::blocking::rng;
use crate::rcc::Rcc;
use crate::stm32::RNG;

#[derive(Clone, Copy)]
pub enum RngClkSource {
    HSI = 1,
    SysClock = 2,
    PLLQ = 3,
}

#[derive(Clone, Copy)]
pub enum RngClkDivider {
    NotDivided = 0,
    Div2 = 1,
    Div4 = 2,
    Div8 = 3,
}

pub struct Config {
    clk_src: RngClkSource,
    clk_div: RngClkDivider,
}

impl Config {
    pub fn new(clk_src: RngClkSource) -> Self {
        Config::default().clock_src(clk_src)
    }

    pub fn clock_src(mut self, clk_src: RngClkSource) -> Self {
        self.clk_src = clk_src;
        self
    }

    pub fn clock_div(mut self, clk_div: RngClkDivider) -> Self {
        self.clk_div = clk_div;
        self
    }
}

impl Default for Config {
    fn default() -> Config {
        Config {
            clk_src: RngClkSource::HSI,
            clk_div: RngClkDivider::NotDivided,
        }
    }
}

#[derive(Debug)]
pub enum ErrorKind {
    ClockError,
    SeedError,
}

pub trait RngExt {
    fn constrain(self, cfg: Config, rcc: &mut Rcc) -> Rng;
}

impl RngExt for RNG {
    fn constrain(self, cfg: Config, rcc: &mut Rcc) -> Rng {
        rcc.rb.ahbenr.modify(|_, w| w.rngen().set_bit());
        rcc.rb.ahbrstr.modify(|_, w| w.rngrst().set_bit());
        rcc.rb.ahbrstr.modify(|_, w| w.rngrst().clear_bit());
        rcc.rb
            .ccipr
            .modify(|_, w| unsafe { w.rngsel().bits(cfg.clk_src as u8) });
        rcc.rb
            .ccipr
            .modify(|_, w| unsafe { w.rngdiv().bits(cfg.clk_div as u8) });
        self.cr.modify(|_, w| w.rngen().set_bit());
        Rng { rb: self }
    }
}

pub trait RngCore<W> {
    fn gen(&mut self) -> Result<W, ErrorKind>;
    fn gen_range(&mut self, low: W, high: W) -> Result<W, ErrorKind>;
    fn fill(&mut self, dest: &mut [W]) -> Result<(), ErrorKind>;
}

pub struct Rng {
    rb: RNG,
}

impl Rng {
    pub fn next(&mut self) -> Result<u32, ErrorKind> {
        loop {
            let status = self.rb.sr.read();
            if status.drdy().bit() {
                return Ok(self.rb.dr.read().rndata().bits());
            }
            if status.cecs().bit() {
                return Err(ErrorKind::ClockError);
            }
            if status.secs().bit() {
                return Err(ErrorKind::SeedError);
            }
        }
    }

    pub fn release(self) -> RNG {
        self.rb
    }

    pub fn gen_bool(&mut self) -> Result<bool, ErrorKind> {
        let val = self.next()?;
        Ok(val & 1 == 1)
    }

    pub fn gen_ratio(&mut self, numerator: u32, denominator: u32) -> Result<bool, ErrorKind> {
        assert!(denominator > 0 || denominator > numerator);
        let val = self.gen_range(0, denominator)?;
        Ok(numerator > val)
    }

    pub fn choose<'a, T>(&mut self, values: &'a [T]) -> Result<&'a T, ErrorKind> {
        let val = self.gen_range(0, values.len())?;
        Ok(&values[val])
    }

    pub fn choose_mut<'a, T>(&mut self, values: &'a mut [T]) -> Result<&'a mut T, ErrorKind> {
        let val = self.gen_range(0, values.len())?;
        Ok(&mut values[val])
    }

    pub fn shuffle<T>(&mut self, values: &mut [T]) -> Result<(), ErrorKind> {
        for i in (1..values.len()).rev() {
            values.swap(i, self.gen_range(0, i + 1)?);
        }
        Ok(())
    }
}

impl rng::Read for Rng {
    type Error = ErrorKind;

    fn read(&mut self, buffer: &mut [u8]) -> Result<(), Self::Error> {
        self.fill(buffer)
    }
}

macro_rules! rng_core {
    ($($type:ty),+) => {
        $(
            impl RngCore<$type> for Rng {
                fn gen(&mut self) -> Result<$type, ErrorKind> {
                    let val = self.next()?;
                    Ok(val as $type)
                }

                fn gen_range(&mut self, low: $type, high: $type) -> Result<$type, ErrorKind> {
                    assert!(high > low);
                    let range = high - low;
                    let val: $type = self.gen()?;
                    Ok(low + val % range)
                }

                fn fill(&mut self, buffer: &mut [$type]) -> Result<(), ErrorKind> {
                    const BATCH_SIZE: usize = 4 / mem::size_of::<$type>();
                    let mut i = 0_usize;
                    while i < buffer.len() {
                        let random_word = self.next()?;
                        let bytes: [$type; BATCH_SIZE] = unsafe { mem::transmute(random_word) };
                        let n = cmp::min(BATCH_SIZE, buffer.len() - i);
                        buffer[i..i + n].copy_from_slice(&bytes[..n]);
                        i += n;
                    }
                    Ok(())
                }
            }
        )+
    };
}

rng_core!(usize, u32, u16, u8);
