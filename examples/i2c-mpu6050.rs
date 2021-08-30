#![deny(warnings)]
#![deny(unsafe_code)]
#![no_main]
#![no_std]

use hal::i2c::Config;
use hal::prelude::*;
use hal::stm32;
use stm32g4xx_hal as hal;

use cortex_m_rt::entry;
use log::info;
use mpu6050::*;

#[macro_use]
mod utils;

#[entry]
fn main() -> ! {
    utils::logger::init();

    let dp = stm32::Peripherals::take().expect("cannot take peripherals");
    let cp = cortex_m::Peripherals::take().expect("cannot take core peripherals");

    let mut rcc = dp.RCC.constrain();
    let gpiob = dp.GPIOB.split(&mut rcc);

    let sda = gpiob.pb9.into_alternate_open_drain();
    let scl = gpiob.pb8.into_alternate_open_drain();

    let i2c = dp.I2C1.i2c(sda, scl, Config::new(100.khz()), &mut rcc);

    let mut mpu = Mpu6050::new(i2c);
    let mut delay = cp.SYST.delay(&rcc.clocks);
    mpu.init(&mut delay).expect("cannot initialize the MPU6050");

    loop {
        let acc = mpu.get_acc().expect("cannot read accelerometer");
        let gyro = mpu.get_gyro().expect("cannot read gyro");
        let temp = mpu.get_temp().expect("cannot read temperature");
        info!("acc: {:?}, gyro: {:?}, temp: {:?}C", acc, gyro, temp);
        delay.delay(250.ms());
    }
}
