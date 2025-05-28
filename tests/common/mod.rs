use fugit::{ExtU32, MicrosDurationU32};
use stm32g4xx_hal::stm32;

#[non_exhaustive]
pub struct Timer<const CYCLES_PER_US: u32>;

impl<const CYCLES_PER_US: u32> Timer<CYCLES_PER_US> {
    #[allow(dead_code)]
    pub fn enable_timer(cp: &mut stm32::CorePeripherals) -> Self {
        cp.DCB.enable_trace();
        cp.DWT.enable_cycle_counter();

        Timer
    }

    /// Returns duration since timer start
    pub fn now(&self) -> MicrosDurationU32 {
        (stm32::DWT::cycle_count() / CYCLES_PER_US).micros()
    }
}

#[allow(dead_code)]
pub fn is_pax_low(pin: u8) -> bool {
    let gpioa = unsafe { &*GPIOA::PTR };
    gpioa.idr().read().idr(pin).is_low()
}

#[allow(dead_code)]
#[derive(Debug, defmt::Format)]
pub struct ErrorTimedOut;

#[allow(dead_code)]
pub fn await_lo<const CYCLES_PER_US: u32>(
    timer: &Timer<CYCLES_PER_US>,
    pin: u8,
    timeout: MicrosDurationU32,
) -> Result<MicrosDurationU32, ErrorTimedOut> {
    await_p(timer, || is_pax_low(pin), timeout)
}

#[allow(dead_code)]
pub fn await_hi<const CYCLES_PER_US: u32>(
    timer: &Timer<CYCLES_PER_US>,
    pin: u8,
    timeout: MicrosDurationU32,
) -> Result<MicrosDurationU32, ErrorTimedOut> {
    await_p(timer, || !is_pax_low(pin), timeout)
}

#[allow(dead_code)]
pub fn await_p<const CYCLES_PER_US: u32>(
    timer: &Timer<CYCLES_PER_US>,
    mut p: impl FnMut() -> bool,
    timeout: MicrosDurationU32,
) -> Result<MicrosDurationU32, ErrorTimedOut> {
    let before = timer.now();

    loop {
        let passed_time = timer.now() - before;
        if p() {
            return Ok(passed_time);
        }
        if passed_time > timeout {
            return Err(ErrorTimedOut);
        }
    }
}
