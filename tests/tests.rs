#![no_std]
#![no_main]

#[path = "../examples/utils/mod.rs"]
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
    use embedded_hal::{
        digital::{InputPin, OutputPin},
        pwm::SetDutyCycle,
    };
    use fugit::RateExtU32;
    use stm32g4xx_hal::{
        delay::SYSTDelayExt,
        gpio::{GpioExt, AF6},
        pwm::PwmExt,
        rcc::RccExt,
        stm32::GPIOA,
    };

    #[test]
    fn gpio_push_pull() {
        use super::*;

        // TODO: Is it ok to steal these?
        let cp = unsafe { stm32::CorePeripherals::steal() };
        let dp = unsafe { stm32::Peripherals::steal() };
        let mut rcc = dp.RCC.constrain();
        let mut delay = cp.SYST.delay(&rcc.clocks);

        let gpioa = dp.GPIOA.split(&mut rcc);
        let mut pin = gpioa.pa8.into_push_pull_output();

        pin.set_high().unwrap();
        delay.delay(1.millis()); // Give the pin plenty of time to go high
        assert!(pin.is_high().unwrap());
        {
            let gpioa = unsafe { &*GPIOA::PTR };
            assert!(!is_pa8_low(gpioa));
        }

        pin.set_low().unwrap();
        delay.delay(1.millis()); // Give the pin plenty of time to go low
        assert!(pin.is_low().unwrap());
        {
            let gpioa = unsafe { &*GPIOA::PTR };
            assert!(is_pa8_low(gpioa));
        }
    }

    #[test]
    fn gpio_open_drain() {
        use super::*;

        // TODO: Is it ok to steal these?
        let cp = unsafe { stm32::CorePeripherals::steal() };
        let dp = unsafe { stm32::Peripherals::steal() };
        let mut rcc = dp.RCC.constrain();
        let mut delay = cp.SYST.delay(&rcc.clocks);

        let gpioa = dp.GPIOA.split(&mut rcc);
        let mut pin = gpioa.pa8.into_open_drain_output();

        // Enable pull-up resistor
        {
            let gpioa = unsafe { &*GPIOA::PTR };
            gpioa.pupdr().modify(|_, w| w.pupdr8().pull_up());
        }

        pin.set_high().unwrap();
        delay.delay(1.millis()); // Give the pin plenty of time to go high
        assert!(pin.is_high().unwrap());
        {
            let gpioa = unsafe { &*GPIOA::PTR };
            assert!(!is_pa8_low(gpioa));
        }

        pin.set_low().unwrap();
        delay.delay(1.millis()); // Give the pin plenty of time to go low
        assert!(pin.is_low().unwrap());
        {
            let gpioa = unsafe { &*GPIOA::PTR };
            assert!(is_pa8_low(gpioa));
        }
    }

    #[test]
    fn pwm() {
        use super::*;

        // TODO: Is it ok to steal these?
        let cp = unsafe { stm32::CorePeripherals::steal() };
        let dp = unsafe { stm32::Peripherals::steal() };
        let mut rcc = dp.RCC.constrain();
        let mut delay = cp.SYST.delay(&rcc.clocks);

        let gpioa = dp.GPIOA.split(&mut rcc);
        let pin: stm32g4xx_hal::gpio::gpioa::PA8<stm32g4xx_hal::gpio::Alternate<AF6>> =
            gpioa.pa8.into_alternate();

        let mut pwm = dp.TIM1.pwm(pin, 1000u32.Hz(), &mut rcc);

        pwm.set_duty_cycle_percent(50).unwrap();
        pwm.enable();

        let gpioa = unsafe { &*GPIOA::PTR };

        //let get_pin_state = || unsafe {  (0x4800_0004 as *const u32).read_volatile() };

        // TODO: This is a very bad way to measure time
        let min: MicrosDurationU32 = 490u32.micros(); // Some extra on min for cpu overhead
        let max: MicrosDurationU32 = 505u32.micros();

        {
            println!("Awaiting first rising edge...");
            let duration_until_lo = await_lo(&gpioa, &mut delay, max).unwrap();
            println!("Low..., Waited ~{} until low", duration_until_lo);
            let lo_duration = await_hi(&gpioa, &mut delay, max).unwrap();
            println!("High..., Low half period: {}", lo_duration);
        }

        for _ in 0..10 {
            // Make sure the timer half periods are within 490-505us

            let hi_duration = await_lo(&gpioa, &mut delay, max).unwrap();
            println!("Low..., High half period: {}", hi_duration);
            assert!(hi_duration > min, "{} > {}", hi_duration, min);

            let lo_duration = await_hi(&gpioa, &mut delay, max).unwrap();
            println!("High..., Low half period: {}", lo_duration);
            assert!(lo_duration > min, "{} > {}", lo_duration, min);
        }

        println!("Done!");
        for i in (0..5).rev() {
            println!("{}", i);
            delay.delay(1000.millis());
        }

        pwm.disable();
    }
}

fn is_pa8_low(gpioa: &stm32::gpioa::RegisterBlock) -> bool {
    gpioa.idr().read().idr(8).is_low()
}

#[derive(Debug, defmt::Format)]
struct ErrorTimedOut;

fn await_lo(
    gpioa: &stm32::gpioa::RegisterBlock,
    delay: &mut SystDelay,
    timeout: MicrosDurationU32,
) -> Result<MicrosDurationU32, ErrorTimedOut> {
    await_p(|| is_pa8_low(gpioa), delay, timeout)
}

fn await_hi(
    gpioa: &stm32::gpioa::RegisterBlock,
    delay: &mut SystDelay,
    timeout: MicrosDurationU32,
) -> Result<MicrosDurationU32, ErrorTimedOut> {
    await_p(|| !is_pa8_low(gpioa), delay, timeout)
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
