#![no_main]
#![no_std]

extern crate panic_halt;

use core::cell::RefCell;
use core::ops::DerefMut;
use cortex_m;
use cortex_m::interrupt::{free, Mutex};
use cortex_m_rt::entry;
use stm32g4xx_hal::{
    gpio::gpioa::PA5,
    gpio::{Output, PushPull},
    prelude::*,
    stm32,
    stm32::interrupt,
    timer::{Timer},
};

static LED: Mutex<RefCell<Option<PA5<Output<PushPull>>>>> = Mutex::new(RefCell::new(None));
static TIMER_TIM2: Mutex<RefCell<Option<Timer<stm32::TIM2>>>> = Mutex::new(RefCell::new(None));

#[interrupt]
fn TIM2() {
    free(|cs| {
        if let Some(ref mut tim2) = TIMER_TIM2.borrow(cs).borrow_mut().deref_mut() {
            tim2.clear_irq();
            //tim2.clear_interrupt(Event::TimeOut);
        }
        if let Some(ref mut led) = LED.borrow(cs).borrow_mut().deref_mut() {
            led.toggle().unwrap();
        }
    });
}

#[entry]
fn main() -> ! {
    let dp = stm32::Peripherals::take().unwrap();

    // Set up the system clock
    let mut rcc = dp.RCC.constrain();

    // Set up the LED
    let gpioa = dp.GPIOA.split(&mut rcc);
    let led = gpioa.pa5.into_push_pull_output();

    // Set up the interrupt timer
    let mut timer = dp.TIM2.timer(&mut rcc);
    timer.start(1000.ms());
    timer.listen();

    free(|cs| {
        TIMER_TIM2.borrow(cs).replace(Some(timer));
        LED.borrow(cs).replace(Some(led));
    });

    // Enable interrupt
    stm32::NVIC::unpend(stm32::Interrupt::TIM2);
    unsafe {
        stm32::NVIC::unmask(stm32::Interrupt::TIM2);
    }

    loop {}
}