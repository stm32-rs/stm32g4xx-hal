/// This code has been taken from the stm32g0xx-hal project and modified slightly to support
/// STM32G4xx MCUs.
pub use fugit::{
    Duration, ExtU32, HertzU32 as Hertz, HoursDurationU32 as Hour,
    MicrosDurationU32 as MicroSecond, MinutesDurationU32 as Minute, NanosDurationU32 as NanoSecond,
    RateExtU32, SecsDurationU32 as Second,
};

/// Baudrate
#[derive(Debug, Eq, PartialEq, PartialOrd, Clone, Copy)]
pub struct Bps(pub u32);

/// A measurement of a monotonically nondecreasing clock
pub type Instant = fugit::TimerInstantU32<1_000_000>;

/// WeekDay (1-7)
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct WeekDay(pub u32);

/// Date (1-31)
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct MonthDay(pub u32);

/// Week (1-52)
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Week(pub u32);

/// Month (1-12)
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Month(pub u32);

/// Year
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Year(pub u32);

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Time {
    pub hours: Hour,
    pub minutes: Minute,
    pub seconds: Second,
    pub daylight_savings: bool,
}

impl Time {
    pub fn new(hours: Hour, minutes: Minute, seconds: Second, daylight_savings: bool) -> Self {
        Self {
            hours,
            minutes,
            seconds,
            daylight_savings,
        }
    }
}

#[cfg(feature = "defmt")]
impl defmt::Format for Time {
    fn format(&self, f: defmt::Formatter) {
        // format the bitfields of the register as struct fields
        defmt::write!(
            f,
            "{:02}:{:02}:{:02}",
            self.hours.ticks(),
            self.minutes.ticks(),
            self.seconds.ticks()
        )
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Date {
    pub year: Year,
    pub month: Month,
    pub day: MonthDay,
}

impl Date {
    pub fn new(year: Year, month: Month, day: MonthDay) -> Self {
        Self { day, month, year }
    }
}

#[cfg(feature = "defmt")]
impl defmt::Format for Date {
    fn format(&self, f: defmt::Formatter) {
        // format the bitfields of the register as struct fields
        defmt::write!(
            f,
            "{:04}-{:02}-{:02}",
            self.year.0,
            self.month.0,
            self.day.0
        )
    }
}

pub trait U32Ext {
    /// Wrap in `Bps`
    fn bps(self) -> Bps;

    /// Day in month
    fn day(self) -> MonthDay;

    /// Month
    fn month(self) -> Month;

    /// Year
    fn year(self) -> Year;
}

impl U32Ext for u32 {
    fn bps(self) -> Bps {
        assert!(self > 0);
        Bps(self)
    }
    fn day(self) -> MonthDay {
        MonthDay(self)
    }

    fn month(self) -> Month {
        Month(self)
    }

    fn year(self) -> Year {
        Year(self)
    }
}

/// Converts clock cycles at a given frequency into a Duration with arbitrary fraction
pub fn duration<const NOM: u32, const DENOM: u32>(
    hz: Hertz,
    cycles: u32,
) -> Duration<u32, NOM, DENOM> {
    let cycles = cycles as u64;
    let clk = hz.raw() as u64;
    let duration_ticks = cycles.saturating_mul(DENOM as u64) / clk / NOM as u64;
    Duration::<u32, NOM, DENOM>::from_ticks(duration_ticks as u32)
}

/// Converts a Duration with arbitrary fraction into a number of cycles at the specified frequency
pub fn cycles<const NOM: u32, const DENOM: u32>(ms: Duration<u32, NOM, DENOM>, clk: Hertz) -> u32 {
    assert!(ms.ticks() > 0);
    let clk = clk.raw() as u64;
    let period = ms.ticks() as u64;
    let cycles = clk.saturating_mul(period).saturating_mul(NOM as u64) / DENOM as u64;
    cycles as u32
}
