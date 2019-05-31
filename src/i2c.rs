//! I2C
use hal::blocking::i2c::{Read, Write, WriteRead};

use core::cmp;
use crate::gpio::{gpioa::*, gpiob::*};
use crate::gpio::{AltFunction, OpenDrain, Output};
use crate::rcc::Rcc;
use crate::stm32::{I2C1, I2C2};
use crate::time::Hertz;

pub struct Config {
    speed: Option<Hertz>,
    timing: Option<u32>,
    analog_filter: bool,
    digital_filter: u8,
}

impl Config {
    pub fn new<T>(speed: T) -> Self
    where
        T: Into<Hertz>,
    {
        Config {
            speed: Some(speed.into()),
            timing: None,
            analog_filter: true,
            digital_filter: 0,
        }
    }
    
    pub fn with_timing(timing: u32) -> Self {
        Config {
            timing: Some(timing),
            speed: None,
            analog_filter: true,
            digital_filter: 0,
        }
    }

    pub fn disable_analog_filter(mut self) -> Self {
        self.analog_filter = false;
        self
    }

    pub fn enable_digital_filter(mut self, cycles: u8) -> Self {
        assert!(cycles <= 16);
        self.digital_filter = cycles;
        self
    }

    fn timing_bits(&self, i2c_clk: Hertz) -> u32 {
        if let Some(timing) = self.timing {
            return timing
        }
        let speed = self.speed.unwrap();
        let (psc, scll, sclh, sdadel, scldel) = if speed.0 <= 100_000 {
            let psc = 3;
            let scll = cmp::max(((i2c_clk.0 >> (psc + 1)) / speed.0) - 1, 255);
            let sclh = scll - 4;
            let sdadel = 2;
            let scldel = 4;
            (psc, scll, sclh, sdadel, scldel)
        } else {
            let psc = 1;
            let scll = cmp::max(((i2c_clk.0 >> (psc + 1)) / speed.0) - 1, 255);
            let sclh = scll - 6;
            let sdadel = 1;
            let scldel = 3;
            (psc, scll, sclh, sdadel, scldel)
        };
        psc << 28 | scldel << 20 | sdadel << 16 | sclh << 8 | scll
    }
}

/// I2C abstraction
pub struct I2c<I2C, SDA, SCL> {
    i2c: I2C,
    sda: SDA,
    scl: SCL,
}

// I2C SDA pin
pub trait SDAPin<I2C> {
    fn setup(&self);
}

// I2C SCL pin
pub trait SCLPin<I2C> {
    fn setup(&self);
}

// I2C error
#[derive(Debug)]
pub enum Error {
    Overrun,
    Nack,
    PECError,
    BusError,
    ArbitrationLost,
}

pub trait I2cExt<I2C> {
    fn i2c<SDA, SCL>(self, sda: SDA, scl: SCL, config: Config, rcc: &mut Rcc) -> I2c<I2C, SDA, SCL>
    where
        SDA: SDAPin<I2C>,
        SCL: SCLPin<I2C>;
}

macro_rules! i2c {
    ($I2CX:ident, $i2cx:ident, $i2cxen:ident, $i2crst:ident,
        sda: [ $($PSDA:ty,)+ ],
        scl: [ $($PSCL:ty,)+ ],
    ) => {
        $(
            impl SDAPin<$I2CX> for $PSDA {
                fn setup(&self) {
                    self.set_alt_mode(AltFunction::AF6)
                }
            }
        )+

        $(
            impl SCLPin<$I2CX> for $PSCL {
                fn setup(&self) {
                    self.set_alt_mode(AltFunction::AF6)
                }
            }
        )+

        impl I2cExt<$I2CX> for $I2CX {
            fn i2c<SDA, SCL>(
                self,
                sda: SDA,
                scl: SCL,
                config: Config,
                rcc: &mut Rcc,
            ) -> I2c<$I2CX, SDA, SCL>
            where
                SDA: SDAPin<$I2CX>,
                SCL: SCLPin<$I2CX>,
            {
                I2c::$i2cx(self, sda, scl, config, rcc)
            }
        }

        impl<SDA, SCL> I2c<$I2CX, SDA, SCL> {
            pub fn $i2cx(i2c: $I2CX, sda: SDA, scl: SCL, config: Config, rcc: &mut Rcc) -> Self
            where
                SDA: SDAPin<$I2CX>,
                SCL: SCLPin<$I2CX>,
            {

                sda.setup();
                scl.setup();

                // Enable clock for I2C
                rcc.rb.apbenr1.modify(|_, w| w.$i2cxen().set_bit());

                // Reset I2C
                rcc.rb.apbrstr1.modify(|_, w| w.$i2crst().set_bit());
                rcc.rb.apbrstr1.modify(|_, w| w.$i2crst().clear_bit());

                // Make sure the I2C unit is disabled so we can configure it
                i2c.cr1.modify(|_, w| w.pe().clear_bit());

                // Setup protocol timings
                let timing_bits = config.timing_bits(rcc.clocks.apb_clk);
                i2c.timingr.write(|w| unsafe { w.bits(timing_bits) });

                // Enable the I2C processing
                i2c.cr1.modify(|_, w| unsafe {
                    w.pe()
                        .set_bit()
                        .dnf()
                        .bits(config.digital_filter)
                        .anfoff()
                        .bit(!config.analog_filter)
                });

                I2c { i2c, sda, scl }
            }

            pub fn release(self) -> ($I2CX, SDA, SCL) {
                (self.i2c, self.sda, self.scl)
            }

            fn send_byte(&self, byte: u8) -> Result<(), Error> {
                // Wait until we're ready for sending
                while self.i2c.isr.read().txe().bit_is_clear() {}

                // Push out a byte of data
                self.i2c.txdr.write(|w| unsafe { w.txdata().bits(byte) });

                // While until byte is transferred
                loop {
                    let isr = self.i2c.isr.read();
                    if isr.berr().bit_is_set() {
                        self.i2c.icr.write(|w| w.berrcf().set_bit());
                        return Err(Error::BusError);
                    } else if isr.arlo().bit_is_set() {
                        self.i2c.icr.write(|w| w.arlocf().set_bit());
                        return Err(Error::ArbitrationLost);
                    } else if isr.nackf().bit_is_set() {
                        self.i2c.icr.write(|w| w.nackcf().set_bit());
                        return Err(Error::Nack);
                    }
                    return Ok(())
                }
            }

            fn recv_byte(&self) -> Result<u8, Error> {
                while self.i2c.isr.read().rxne().bit_is_clear() {}

                let value = self.i2c.rxdr.read().rxdata().bits();
                Ok(value)
            }
        }

        impl<SDA, SCL> WriteRead for I2c<$I2CX, SDA, SCL> {
            type Error = Error;

            fn write_read(
                &mut self,
                addr: u8,
                bytes: &[u8],
                buffer: &mut [u8],
            ) -> Result<(), Self::Error> {
                self.write(addr, bytes)?;
                self.read(addr, buffer)?;

                Ok(())
            }
        }

        impl<SDA, SCL> Write for I2c<$I2CX, SDA, SCL> {
            type Error = Error;

            fn write(&mut self, addr: u8, bytes: &[u8]) -> Result<(), Self::Error> {
                self.i2c.cr2.modify(|_, w| unsafe {
                    w.start()
                        .set_bit()
                        .nbytes()
                        .bits(bytes.len() as u8)
                        .sadd()
                        .bits((addr << 1) as u16)
                        .rd_wrn()
                        .clear_bit()
                        .autoend()
                        .set_bit()
                });

                while self.i2c.isr.read().busy().bit_is_clear() {}

                // Send bytes
                for c in bytes {
                    self.send_byte(*c)?;
                }

                Ok(())
            }
        }

        impl<SDA, SCL> Read for I2c<$I2CX, SDA, SCL> {
            type Error = Error;

            fn read(&mut self, addr: u8, buffer: &mut [u8]) -> Result<(), Self::Error> {
                self.i2c.cr2.modify(|_, w| unsafe {
                    w.start()
                        .set_bit()
                        .nbytes()
                        .bits(buffer.len() as u8)
                        .sadd()
                        .bits((addr << 1) as u16)
                        .autoend()
                        .set_bit()
                });

                // Wait until address was sent
                while self.i2c.isr.read().busy().bit_is_clear() {}

                // Receive bytes into buffer
                for c in buffer {
                    *c = self.recv_byte()?;
                }
                Ok(())
            }
        }
    };
}

i2c!(
    I2C1,
    i2c1,
    i2c1en,
    i2c1rst,
    sda: [
        PA10<Output<OpenDrain>>,
        PB7<Output<OpenDrain>>,
        PB9<Output<OpenDrain>>,
    ],
    scl: [
        PA9<Output<OpenDrain>>,
        PB6<Output<OpenDrain>>,
        PB8<Output<OpenDrain>>,
    ],
);

i2c!(
    I2C2,
    i2c2,
    i2c2en,
    i2c2rst,
    sda: [
        PA12<Output<OpenDrain>>,
        PB11<Output<OpenDrain>>,
        PB14<Output<OpenDrain>>,
    ],
    scl: [
        PA11<Output<OpenDrain>>,
        PB10<Output<OpenDrain>>,
        PB13<Output<OpenDrain>>,
    ],
);
