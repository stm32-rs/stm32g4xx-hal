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
    let gpioa = unsafe { &*stm32::GPIOA::PTR };
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

#[allow(dead_code)]
pub fn test_pwm<const CYCLES_PER_US: u32>(
    timer: &Timer<CYCLES_PER_US>,
    pin_num: u8,
    t_lo: MicrosDurationU32,
    t_hi: MicrosDurationU32,
    t_max_deviation: MicrosDurationU32,
    first_start_mult: u32,
) {
    defmt::debug!("Awaiting first rising edge...");

    let t_lo_min = t_lo - t_max_deviation;
    let t_lo_max = t_lo + t_max_deviation;
    let t_hi_min = t_hi - t_max_deviation;
    let t_hi_max = t_hi + t_max_deviation;

    let mut hi_duration = 0u32.micros();
    let mut lo_duration = 0.micros();

    let duration_until_lo = await_lo(timer, pin_num, t_hi_max).unwrap();
    let first_lo_duration = await_hi(timer, pin_num, t_lo_max * first_start_mult).unwrap(); // Some extra time until started

    for _ in 0..100 {
        // Make sure the timer half periods are within 495-505us
        hi_duration = await_lo(timer, pin_num, t_hi_max).unwrap();
        //defmt::debug!("hi_duration: {}", hi_duration);
        defmt::assert!(
            hi_duration > t_hi_min && hi_duration < t_hi_max,
            "hi: {} < {} < {}",
            t_hi_min,
            hi_duration,
            t_hi_max
        );

        lo_duration = await_hi(timer, pin_num, t_lo_max).unwrap();
        //defmt::debug!("lo_duration: {}", lo_duration);
        defmt::assert!(
            lo_duration > t_lo_min && lo_duration < t_lo_max,
            "lo: {} < {} < {}",
            t_lo_min,
            lo_duration,
            t_lo_max
        );
        //defmt::debug!("");
    }

    // Prints deferred until here to not mess up timing
    defmt::debug!("Waited ~{} until low", duration_until_lo);
    defmt::debug!("First low half period: {}", first_lo_duration);

    defmt::debug!("High half period: {}", hi_duration);
    defmt::debug!("Low half period: {}", lo_duration);

    defmt::debug!("Done!");
}
