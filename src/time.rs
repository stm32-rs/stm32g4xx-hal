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
    pub hours: u8,
    pub minutes: u8,
    pub seconds: u8,
    pub daylight_savings: bool,
}

impl Time {
    pub fn new(hours: Hour, minutes: Minute, seconds: Second, daylight_savings: bool) -> Self {
        use core::convert::TryInto;
        Self {
            hours: hours.ticks().try_into().unwrap(),
            minutes: minutes.ticks().try_into().unwrap(),
            seconds: seconds.ticks().try_into().unwrap(),
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
            self.hours,
            self.minutes,
            self.seconds
        )
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Date {
    pub year: u16,
    pub month: u8,
    pub day: u8,
}

impl Date {
    pub fn new(year: Year, month: Month, day: MonthDay) -> Self {
        use core::convert::TryInto;
        Self {
            day: day.0.try_into().unwrap(),
            month: month.0.try_into().unwrap(),
            year: year.0.try_into().unwrap(),
        }
    }
}

#[cfg(feature = "defmt")]
impl defmt::Format for Date {
    fn format(&self, f: defmt::Formatter) {
        // format the bitfields of the register as struct fields
        defmt::write!(f, "{:04}-{:02}-{:02}", self.year, self.month, self.day)
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

pub fn duration(hz: Hertz, cycles: u32) -> MicroSecond {
    let cycles = cycles as u64;
    let clk = hz.raw() as u64;
    let us = cycles.saturating_mul(1_000_000_u64) / clk;
    MicroSecond::from_ticks(us as u32)
}

pub fn cycles(ms: MicroSecond, clk: Hertz) -> u32 {
    assert!(ms.ticks() > 0);
    let clk = clk.raw() as u64;
    let period = ms.ticks() as u64;
    let cycles = clk.saturating_mul(period) / 1_000_000_u64;
    cycles as u32
}
