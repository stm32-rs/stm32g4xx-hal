//! External interrupt controller
use crate::gpio::SignalEdge;
use crate::stm32::EXTI;

/// EXTI trigger event
#[derive(PartialEq, PartialOrd, Clone, Copy)]
pub enum Event {
    GPIO0 = 0,
    GPIO1 = 1,
    GPIO2 = 2,
    GPIO3 = 3,
    GPIO4 = 4,
    GPIO5 = 5,
    GPIO6 = 6,
    GPIO7 = 7,
    GPIO8 = 8,
    GPIO9 = 9,
    GPIO10 = 10,
    GPIO11 = 11,
    GPIO12 = 12,
    GPIO13 = 13,
    GPIO14 = 14,
    GPIO15 = 15,
    PVD = 16,
    RTC_ALARM = 17,
    USB = 18,
    LCE = 19,
    RTC_WAKEUP = 20,
    COMP1 = 21,
    COMP2 = 22,
    I2C1 = 23,
    I2C2 = 24,
    USART1 = 25,
    USART2 = 26,
    I2C3 = 27,
    USART3 = 28,
    COMP3 = 29,
    COMP4 = 30,
    COMP5 = 31,
    COMP6 = 32,
    COMP7 = 33,
    UART4 = 34,
    UART5 = 35,
    LPUART1 = 36,
    LPTIM1 = 37,
    PVM1 = 40,
    PVM2 = 41,
    I2C4 = 42,
    UCPD1 = 43,
}

impl Event {
    pub(crate) fn from_code(n: u8) -> Event {
        match n {
            0 => Event::GPIO0,
            1 => Event::GPIO1,
            2 => Event::GPIO2,
            3 => Event::GPIO3,
            4 => Event::GPIO4,
            5 => Event::GPIO5,
            6 => Event::GPIO6,
            7 => Event::GPIO7,
            8 => Event::GPIO8,
            9 => Event::GPIO9,
            10 => Event::GPIO10,
            11 => Event::GPIO11,
            12 => Event::GPIO12,
            13 => Event::GPIO13,
            14 => Event::GPIO14,
            15 => Event::GPIO15,
            _ => unreachable!(),
        }
    }
}

pub trait ExtiExt {
    fn wakeup(&self, ev: Event);
    fn listen(&self, ev: Event, edge: SignalEdge);
    fn unlisten(&self, ev: Event);
    fn unpend(&self, ev: Event);
    fn is_pending(&self, ev: Event) -> bool;
}

impl ExtiExt for EXTI {
    fn listen(&self, ev: Event, edge: SignalEdge) {
        let line = ev as u8;
        assert!(line <= 17);
        let mask = 1 << line;
        match edge {
            SignalEdge::Rising => {
                self.rtsr1()
                    .modify(|r, w| unsafe { w.bits(r.bits() | mask) });
            }
            SignalEdge::Falling => {
                self.ftsr1()
                    .modify(|r, w| unsafe { w.bits(r.bits() | mask) });
            }
            SignalEdge::RisingFalling => {
                self.rtsr1()
                    .modify(|r, w| unsafe { w.bits(r.bits() | mask) });
                self.ftsr1()
                    .modify(|r, w| unsafe { w.bits(r.bits() | mask) });
            }
        }
        self.wakeup(ev);
    }

    fn wakeup(&self, ev: Event) {
        match ev as u8 {
            line if line < 32 => self
                .imr1()
                .modify(|r, w| unsafe { w.bits(r.bits() | (1 << line)) }),
            line => self
                .imr2()
                .modify(|r, w| unsafe { w.bits(r.bits() | (1 << (line - 32))) }),
        };
    }

    fn unlisten(&self, ev: Event) {
        self.unpend(ev);
        match ev as u8 {
            line if line < 32 => {
                let mask = !(1 << line);
                self.imr1()
                    .modify(|r, w| unsafe { w.bits(r.bits() & mask) });
                if line <= 18 {
                    self.rtsr1()
                        .modify(|r, w| unsafe { w.bits(r.bits() & mask) });
                    self.ftsr1()
                        .modify(|r, w| unsafe { w.bits(r.bits() & mask) });
                }
            }
            line => {
                let mask = !(1 << (line - 32));
                self.imr2()
                    .modify(|r, w| unsafe { w.bits(r.bits() & mask) });
            }
        }
    }

    fn is_pending(&self, ev: Event) -> bool {
        let line = ev as u8;
        if line > 18 {
            return false;
        }
        let mask = 1 << line;
        self.pr1().read().bits() & mask != 0
    }

    fn unpend(&self, ev: Event) {
        let line = ev as u8;
        if line <= 18 {
            self.pr1().modify(|_, w| unsafe { w.bits(1 << line) });
        }
    }
}
