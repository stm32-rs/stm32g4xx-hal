//! I2C Fast Mode Plus example with an AS5600 magnetic angle sensor.
//!
//! This example expects the AS5600 to be connected to the I2C bus on PB7 (SDA) and PA15 (SCL),
//! and a 24MHz HSE oscillator to configure the PLL for 168MHz system clock.
//!
//! The I2C bus is configured with Fast Mode Plus (FMP) enabled in SysCfg, and a 1MHz I2C clock rate.
//!
//! ```DEFMT_LOG=debug cargo run --release --example rand --features stm32g431,defmt -- --chip STM32G431KBTx```

#![deny(warnings)]
#![deny(unsafe_code)]
#![no_main]
#![no_std]

use fugit::HertzU32 as Hertz;
use hal::prelude::*;
use hal::rcc::SysClockSrc;
use hal::stm32;
use hal::time::RateExtU32;
use stm32g4xx_hal as hal;
use stm32g4xx_hal::syscfg::SysCfgExt;

use as5600::As5600;
use cortex_m_rt::entry;

#[macro_use]
mod utils;
use utils::logger::error;
use utils::logger::info;

#[entry]
fn main() -> ! {
    utils::logger::init();

    let dp = stm32::Peripherals::take().expect("cannot take peripherals");
    //let cp = cortex_m::Peripherals::take().expect("cannot take core peripherals");

    let pwr_cfg = dp
        .PWR
        .constrain()
        .vos(stm32g4xx_hal::pwr::VoltageScale::Range1 { enable_boost: true })
        .freeze();

    let pll_cfg = hal::rcc::PllConfig {
        mux: hal::rcc::PllSrc::HSE(Hertz::MHz(24)),
        m: hal::rcc::PllMDiv::DIV_2,
        n: hal::rcc::PllNMul::MUL_28,
        r: Some(hal::rcc::PllRDiv::DIV_2),
        q: None,
        p: None,
    };

    info!("Configuring PLL");
    let rcc_cfg = hal::rcc::Config::new(SysClockSrc::PLL)
        .boost(true)
        .pll_cfg(pll_cfg);

    let mut rcc = dp.RCC.freeze(rcc_cfg, pwr_cfg);
    info!("System clock frequency: {}", rcc.clocks.sys_clk.to_Hz());

    let gpioa = dp.GPIOA.split(&mut rcc);
    let gpiob = dp.GPIOB.split(&mut rcc);

    let sda = gpiob.pb7.into_alternate_open_drain();
    let scl = gpioa.pa15.into_alternate_open_drain();

    let mut syscfg = dp.SYSCFG.constrain(&mut rcc);

    // Enable Fast Mode Plus for I2C1
    syscfg.i2c_fmp_enable::<1>(true);

    // Configure I2C for 1MHz
    let i2c = dp.I2C1.i2c(sda, scl, 1.MHz(), &mut rcc);

    let mut as5600 = As5600::new(i2c);

    loop {
        match as5600.angle() {
            Ok(angle) => {
                // Convert angle to degrees
                let angle_degrees = angle as f32 * 360.0 / 4096.0;
                info!("Angle: {}°", angle_degrees);
            }
            Err(e) => match e {
                as5600::error::Error::Communication(_) => error!("I2C communication error"),
                as5600::error::Error::Status(_error) => error!("AS5600 status error"),
                as5600::error::Error::Configuration(_error) => error!("AS5600 configuration error"),
                _ => error!("Other AS5600 error"),
            },
        }
    }
}
