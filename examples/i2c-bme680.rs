//This example reads data from the Bosch BME680 (gas, humidity, temperature, pressure) sensor using I2C.
#![deny(unsafe_code)]
#![no_main]
#![no_std]

use bme680::*;
use core::time::Duration;
use embedded_hal::blocking::delay::DelayMs;
use hal::delay::DelayFromCountDownTimer;
use hal::i2c::Config;
use hal::prelude::*;
use hal::stm32;
use hal::time::{ExtU32, RateExtU32};
use hal::timer::Timer;
use stm32g4xx_hal as hal;

use cortex_m_rt::entry;
use log::info;

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

    let i2c = dp.I2C1.i2c(sda, scl, Config::new(100.kHz()), &mut rcc);

    let mut delayer = cp.SYST.delay(&rcc.clocks);
    let timer2 = Timer::new(dp.TIM2, &rcc.clocks);
    let mut delay = DelayFromCountDownTimer::new(timer2.start_count_down(100.millis()));

    let mut dev =
        Bme680::init(i2c, &mut delayer, I2CAddress::Secondary).expect("Init function failed");

    let settings = SettingsBuilder::new()
        .with_humidity_oversampling(OversamplingSetting::OS2x)
        .with_pressure_oversampling(OversamplingSetting::OS4x)
        .with_temperature_oversampling(OversamplingSetting::OS8x)
        .with_temperature_filter(IIRFilterSize::Size3)
        .with_gas_measurement(Duration::from_millis(1500), 320, 25)
        .with_temperature_offset(-2.2)
        .with_run_gas(true)
        .build();

    let _profile_dur = dev.get_profile_dur(&settings.0).unwrap();
    dev.set_sensor_settings(&mut delayer, settings).unwrap();
    dev.set_sensor_mode(&mut delayer, PowerMode::ForcedMode)
        .unwrap();
    let _sensor_settings = dev.get_sensor_settings(settings.1);

    loop {
        delay.delay_ms(500u32);
        let _power_mode = dev.get_sensor_mode();
        dev.set_sensor_mode(&mut delayer, PowerMode::ForcedMode)
            .unwrap();
        let (data, _state) = dev.get_sensor_data(&mut delayer).unwrap();
        info!("Temperature {}°C", data.temperature_celsius());
        info!("Pressure {}hPa", data.pressure_hpa());
        info!("Humidity {}%", data.humidity_percent());
        info!("Gas Resistence {}Ω", data.gas_resistance_ohm());
    }
}
