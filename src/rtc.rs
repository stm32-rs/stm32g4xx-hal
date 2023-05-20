//! Real Time Clock
use crate::rcc::{RTCSrc, Rcc};
use crate::stm32::RTC;
use crate::time::*;

#[derive(Debug, PartialEq, Eq)]
pub enum RtcHourFormat {
    H24,
    H12,
}

#[derive(Debug, PartialEq, Eq)]
pub enum RtcCalibrationFrequency {
    F1Hz,
    F512Hz,
}

pub enum Event {
    WakeupTimer,
    AlarmA,
    AlarmB,
    Timestamp,
}

#[derive(Debug, Default, PartialEq, Eq)]
pub struct Alarm {
    day: Option<u32>,
    hours: Option<u32>,
    minutes: Option<u32>,
    seconds: Option<u32>,
    subseconds: u16,
    subseconds_mask_bits: u8,
    use_weekday: bool,
}

impl Alarm {
    pub fn new() -> Self {
        Self::default()
    }

    pub fn set_month_day(mut self, day: u32) -> Self {
        self.use_weekday = false;
        self.day = Some(day);
        self
    }

    pub fn set_week_day(mut self, day: u32) -> Self {
        self.use_weekday = true;
        self.day = Some(day);
        self
    }

    pub fn set_hours(mut self, val: u32) -> Self {
        self.hours = Some(val);
        self
    }

    pub fn set_minutes(mut self, val: u32) -> Self {
        self.minutes = Some(val);
        self
    }

    pub fn set_seconds(mut self, val: u32) -> Self {
        self.seconds = Some(val);
        self
    }

    pub fn set_subseconds(mut self, subseconds: u16, mask_bits: u8) -> Self {
        self.subseconds_mask_bits = mask_bits;
        self.subseconds = subseconds;
        self
    }

    pub fn mask_day(mut self) -> Self {
        self.day = None;
        self
    }

    pub fn mask_hours(mut self) -> Self {
        self.hours = None;
        self
    }

    pub fn mask_minutes(mut self) -> Self {
        self.minutes = None;
        self
    }

    pub fn mask_seconds(mut self) -> Self {
        self.seconds = None;
        self
    }
}

impl From<Time> for Alarm {
    fn from(time: Time) -> Self {
        Self::default()
            .set_hours(time.hours)
            .set_minutes(time.minutes)
            .set_seconds(time.seconds)
    }
}

pub struct Rtc {
    rb: RTC,
}

impl Rtc {
    pub fn new(rtc: RTC, src: RTCSrc, rcc: &mut Rcc) -> Self {
        rcc.enable_rtc(src);
        Rtc { rb: rtc }
    }

    pub fn set_hour_format(&mut self, fmt: RtcHourFormat) {
        self.modify(|rb| {
            rb.cr.modify(|_, w| w.fmt().bit(fmt == RtcHourFormat::H12));
        });
    }

    pub fn set_date(&mut self, date: &Date) {
        let (yt, yu) = bcd2_encode(date.year - 1970);
        let (mt, mu) = bcd2_encode(date.month);
        let (dt, du) = bcd2_encode(date.day);

        self.modify(|rb| {
            rb.dr.write(|w| unsafe {
                w.dt()
                    .bits(dt)
                    .du()
                    .bits(du)
                    .mt()
                    .bit(mt > 0)
                    .mu()
                    .bits(mu)
                    .yt()
                    .bits(yt)
                    .yu()
                    .bits(yu)
                    .wdu()
                    .bits(date.day as u8)
            });
        });
    }

    pub fn set_time(&mut self, time: &Time) {
        let (ht, hu) = bcd2_encode(time.hours);
        let (mnt, mnu) = bcd2_encode(time.minutes);
        let (st, su) = bcd2_encode(time.seconds);
        self.modify(|rb| {
            rb.tr.write(|w| unsafe {
                w.ht()
                    .bits(ht)
                    .hu()
                    .bits(hu)
                    .mnt()
                    .bits(mnt)
                    .mnu()
                    .bits(mnu)
                    .st()
                    .bits(st)
                    .su()
                    .bits(su)
                    .pm()
                    .clear_bit()
            });
            rb.cr.modify(|_, w| w.fmt().bit(time.daylight_savings));
        });
    }

    pub fn get_time(&self) -> Time {
        let timer = self.rb.tr.read();
        Time::new(
            bcd2_decode(timer.ht().bits(), timer.hu().bits()).hours(),
            bcd2_decode(timer.mnt().bits(), timer.mnu().bits()).minutes(),
            bcd2_decode(timer.st().bits(), timer.su().bits()).secs(),
            self.rb.cr.read().fmt().bit(),
        )
    }

    pub fn get_date(&self) -> Date {
        let date = self.rb.dr.read();
        Date::new(
            (bcd2_decode(date.yt().bits(), date.yu().bits()) + 1970).year(),
            bcd2_decode(date.mt().bit() as u8, date.mu().bits()).month(),
            bcd2_decode(date.dt().bits(), date.du().bits()).day(),
        )
    }

    pub fn get_week_day(&self) -> u8 {
        self.rb.dr.read().wdu().bits()
    }

    pub fn set_alarm_a(&mut self, alarm: impl Into<Alarm>) {
        let alarm = alarm.into();
        let (dt, du) = bcd2_encode(alarm.day.unwrap_or_default() as u32);
        let (ht, hu) = bcd2_encode(alarm.hours.unwrap_or_default() as u32);
        let (mt, mu) = bcd2_encode(alarm.minutes.unwrap_or_default() as u32);
        let (st, su) = bcd2_encode(alarm.seconds.unwrap_or_default() as u32);

        self.modify(|rb| {
            rb.alrmassr.write(|w| unsafe {
                w.maskss()
                    .bits(alarm.subseconds_mask_bits)
                    .ss()
                    .bits(alarm.subseconds)
            });
            rb.alrmar.write(|w| unsafe {
                w.wdsel().bit(alarm.use_weekday);
                w.msk1().bit(alarm.seconds.is_none());
                w.msk2().bit(alarm.minutes.is_none());
                w.msk3().bit(alarm.hours.is_none());
                w.msk4().bit(alarm.day.is_none());
                w.dt().bits(dt);
                w.du().bits(du);
                w.ht().bits(ht);
                w.hu().bits(hu);
                w.mnt().bits(mt);
                w.mnu().bits(mu);
                w.st().bits(st);
                w.su().bits(su)
            });

            rb.cr.modify(|_, w| w.alrae().set_bit());
        });
    }

    pub fn set_alarm_b(&mut self, alarm: Alarm) {
        let (dt, du) = bcd2_encode(alarm.day.unwrap_or_default() as u32);
        let (ht, hu) = bcd2_encode(alarm.hours.unwrap_or_default() as u32);
        let (mt, mu) = bcd2_encode(alarm.minutes.unwrap_or_default() as u32);
        let (st, su) = bcd2_encode(alarm.seconds.unwrap_or_default() as u32);

        self.modify(|rb| {
            rb.alrmbssr.write(|w| unsafe {
                w.maskss()
                    .bits(alarm.subseconds_mask_bits)
                    .ss()
                    .bits(alarm.subseconds)
            });
            rb.alrmbr.write(|w| unsafe {
                w.wdsel().bit(alarm.use_weekday);
                w.msk1().bit(alarm.seconds.is_none());
                w.msk2().bit(alarm.minutes.is_none());
                w.msk3().bit(alarm.hours.is_none());
                w.msk4().bit(alarm.day.is_none());
                w.dt().bits(dt);
                w.du().bits(du);
                w.ht().bits(ht);
                w.hu().bits(hu);
                w.mnt().bits(mt);
                w.mnu().bits(mu);
                w.st().bits(st);
                w.su().bits(su)
            });

            rb.cr.modify(|_, w| w.alrbe().set_bit());
        });
    }

    pub fn listen(&mut self, ev: Event) {
        self.modify(|rb| match ev {
            Event::WakeupTimer => rb.cr.modify(|_, w| w.wutie().set_bit()),
            Event::AlarmA => rb.cr.modify(|_, w| w.alraie().set_bit()),
            Event::AlarmB => rb.cr.modify(|_, w| w.alrbie().set_bit()),
            Event::Timestamp => rb.cr.modify(|_, w| w.tsie().set_bit()),
        })
    }

    pub fn unlisten(&mut self, ev: Event) {
        self.modify(|rb| match ev {
            Event::WakeupTimer => rb.cr.modify(|_, w| w.wutie().clear_bit()),
            Event::AlarmA => rb.cr.modify(|_, w| w.alraie().clear_bit()),
            Event::AlarmB => rb.cr.modify(|_, w| w.alrbie().clear_bit()),
            Event::Timestamp => rb.cr.modify(|_, w| w.tsie().clear_bit()),
        })
    }

    pub fn is_pending(&self, ev: Event) -> bool {
        match ev {
            Event::WakeupTimer => self.rb.sr.read().wutf().bit_is_set(),
            Event::AlarmA => self.rb.sr.read().alraf().bit_is_set(),
            Event::AlarmB => self.rb.sr.read().alrbf().bit_is_set(),
            Event::Timestamp => self.rb.sr.read().tsf().bit_is_set(),
        }
    }

    pub fn unpend(&mut self, ev: Event) {
        self.modify(|rb| match ev {
            Event::WakeupTimer => rb.scr.write(|w| w.cwutf().set_bit()),
            Event::AlarmA => rb.scr.write(|w| w.calraf().set_bit()),
            Event::AlarmB => rb.scr.write(|w| w.calrbf().set_bit()),
            Event::Timestamp => rb.scr.write(|w| w.ctsf().set_bit()),
        });
    }

    pub fn enable_calibration_output<PIN: RtcOutputPin>(
        &mut self,
        pin: PIN,
        freq: RtcCalibrationFrequency,
    ) {
        pin.setup();
        self.modify(|rb| {
            rb.cr.modify(|_, w| unsafe {
                w.osel()
                    .bits(0b0)
                    .out2en()
                    .bit(pin.channel())
                    .cosel()
                    .bit(freq == RtcCalibrationFrequency::F1Hz)
                    .tampoe()
                    .clear_bit()
                    .coe()
                    .set_bit()
            });
        });
    }

    fn modify<F>(&mut self, mut closure: F)
    where
        F: FnMut(&mut RTC),
    {
        // Disable write protection
        self.rb.wpr.write(|w| unsafe { w.bits(0xCA) });
        self.rb.wpr.write(|w| unsafe { w.bits(0x53) });
        // Enter init mode
        let isr = self.rb.icsr.read();
        if isr.initf().bit_is_clear() {
            self.rb.icsr.write(|w| w.init().set_bit());
            self.rb.icsr.write(|w| unsafe { w.bits(0xFFFF_FFFF) });
            while self.rb.icsr.read().initf().bit_is_clear() {}
        }
        // Invoke closure
        closure(&mut self.rb);
        // Exit init mode
        self.rb.icsr.write(|w| w.init().clear_bit());
        // Enable_write_protection
        self.rb.wpr.write(|w| unsafe { w.bits(0xFF) });
    }
}

pub trait RtcExt {
    fn constrain(self, rcc: &mut Rcc) -> Rtc;
}

impl RtcExt for RTC {
    fn constrain(self, rcc: &mut Rcc) -> Rtc {
        Rtc::new(self, RTCSrc::LSI, rcc)
    }
}

pub trait RtcOutputPin {
    fn setup(&self);
    fn channel(&self) -> bool;
    fn release(self) -> Self;
}

/*macro_rules! rtc_out_pins {
    ($($pin:ty: ($af_mode:expr, $ch:expr),)+) => {
        $(
            impl RtcOutputPin for $pin {
                fn setup(&self) {
                    self.set_alt_mode($af_mode);
                }

                fn channel(&self) -> bool {
                    $ch
                }

                fn release(self) -> Self {
                    self.into_analog()
                }
            }
        )+
    }
}

rtc_out_pins! {
    PA4<DefaultMode>: (AltFunction::AF3, true),
    PC13<DefaultMode>: (AltFunction::AF3, false),
}*/

fn bcd2_encode(word: u32) -> (u8, u8) {
    let mut value = word as u8;
    let mut bcd_high: u8 = 0;
    while value >= 10 {
        bcd_high += 1;
        value -= 10;
    }
    let bcd_low = ((bcd_high << 4) | value) as u8;
    (bcd_high, bcd_low)
}

fn bcd2_decode(fst: u8, snd: u8) -> u32 {
    let value = snd | fst << 4;
    let value = (value & 0x0F) + ((value & 0xF0) >> 4) * 10;
    value as u32
}
