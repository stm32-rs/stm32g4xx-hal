#![no_std]
#![no_main]

#[path ="../examples/utils/mod.rs"]
mod utils;

use utils::logger::println;

use core::ops::FnMut;
use core::result::Result;
use fugit::{ExtU32, MicrosDurationU32};
use hal::delay::{DelayExt, SystDelay};
use hal::stm32;
use stm32g4xx_hal as hal;

#[defmt_test::tests]
mod tests {
    use embedded_hal::pwm::SetDutyCycle;
    use fugit::RateExtU32;
    use stm32g4xx_hal::{
        delay::SYSTDelayExt, gpio::GpioExt, pwm::PwmExt, rcc::RccExt, stm32::GPIOA,
    };

    #[test]
    fn foo() {
        use super::*;

        let cp = stm32::CorePeripherals::take().expect("cannot take peripherals");
        let dp = stm32::Peripherals::take().expect("cannot take peripherals");
        let mut rcc = dp.RCC.constrain();
        let mut delay = cp.SYST.delay(&rcc.clocks);

        let gpioa = dp.GPIOA.split(&mut rcc);
        let pin = gpioa.pa8.into_alternate();

        let mut pwm = dp.TIM1.pwm(pin, 1000u32.Hz(), &mut rcc);

        let _ = pwm.set_duty_cycle_percent(50);
        pwm.enable();

        let gpioa = unsafe { &*GPIOA::ptr() };

        let min: MicrosDurationU32 = 490u32.micros();// Some extra on min for cpu overhead
        let max: MicrosDurationU32 = 505u32.micros();

        println!("Awaiting rising edge...");
        await_lo(&gpioa, &mut delay, max).unwrap();
        await_hi(&gpioa, &mut delay, max).unwrap();

        for _ in 0..10 {
            // Make sure the timer half periods are within 490-505us

            let hi_duration = await_lo(&gpioa, &mut delay, max).unwrap();
            let lo_duration = await_hi(&gpioa, &mut delay, max).unwrap();

            assert!(hi_duration > min);
            assert!(lo_duration > min);
        }
    }
}

#[derive(Debug)]
struct ErrorTimedOut;

fn await_lo(
    gpioa: &stm32::gpioa::RegisterBlock,
    delay: &mut SystDelay,
    timeout: MicrosDurationU32,
) -> Result<MicrosDurationU32, ErrorTimedOut> {
    await_p(|| gpioa.idr().read().idr(8).is_low(), delay, timeout)
}

fn await_hi(
    gpioa: &stm32::gpioa::RegisterBlock,
    delay: &mut SystDelay,
    timeout: MicrosDurationU32,
) -> Result<MicrosDurationU32, ErrorTimedOut> {
    await_p(|| gpioa.idr().read().idr(8).is_high(), delay, timeout)
}

fn await_p(
    mut p: impl FnMut() -> bool,
    delay: &mut SystDelay,
    timeout: MicrosDurationU32,
) -> Result<MicrosDurationU32, ErrorTimedOut> {
    for i in 0..timeout.ticks() {
        if p() {
            return Ok(i.micros());
        }
        delay.delay(1_u32.micros());
    }
    Err(ErrorTimedOut)
}
