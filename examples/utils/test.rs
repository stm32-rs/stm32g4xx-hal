#![allow(dead_code)]

use fugit::MicrosDurationU32;

use stm32g4xx_hal::stm32;

pub fn is_pax_low(gpioa: &stm32::gpioa::RegisterBlock, x: u8) -> bool {
    gpioa.idr().read().idr(x).is_low()
}

#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Debug)]
pub struct ErrorTimedOut;

pub fn await_lo(
    gpioa: &stm32::gpioa::RegisterBlock,
    pin: u8,
    timeout: MicrosDurationU32,
    now: impl FnMut() -> MicrosDurationU32,
) -> Result<MicrosDurationU32, ErrorTimedOut> {
    await_p(|| is_pax_low(gpioa, pin), timeout, now)
}

pub fn await_hi(
    gpioa: &stm32::gpioa::RegisterBlock,
    pin: u8,
    timeout: MicrosDurationU32,
    now: impl FnMut() -> MicrosDurationU32,
) -> Result<MicrosDurationU32, ErrorTimedOut> {
    await_p(|| !is_pax_low(gpioa, pin), timeout, now)
}

pub fn await_p(
    mut p: impl FnMut() -> bool,
    timeout: MicrosDurationU32,
    mut now: impl FnMut() -> MicrosDurationU32,
) -> Result<MicrosDurationU32, ErrorTimedOut> {
    let before = now();

    loop {
        let passed_time = now() - before;
        if p() {
            return Ok(passed_time);
        }
        if passed_time > timeout {
            return Err(ErrorTimedOut);
        }
    }
}
