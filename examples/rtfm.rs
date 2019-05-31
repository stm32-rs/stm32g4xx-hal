#![no_std]
#![no_main]
#![deny(warnings)]

extern crate cortex_m;
extern crate cortex_m_rt as rt;
extern crate panic_halt;
extern crate rtfm;
extern crate stm32g4xx_hal as hal;

use cortex_m_semihosting::hprintln;
use hal::exti::Event;
use hal::gpio::gpioa::PA5;
use hal::gpio::{Output, PushPull, SignalEdge};
use hal::prelude::*;
use hal::stm32;
use hal::timer::Timer;
use rtfm::app;

#[app(device = hal::stm32)]
const APP: () = {
    static mut EXTI: stm32::EXTI = ();
    static mut TIMER: Timer<stm32::TIM17> = ();
    static mut LED: PA5<Output<PushPull>> = ();

    #[init]
    fn init() {
        let mut rcc = device.RCC.constrain();
        let gpioa = device.GPIOA.split(&mut rcc);
        let gpioc = device.GPIOC.split(&mut rcc);

        let mut timer = device.TIM17.timer(&mut rcc);
        timer.start(3.hz());
        timer.listen();

        gpioc.pc13.listen(SignalEdge::Falling, &mut device.EXTI);

        LED = gpioa.pa5.into_push_pull_output();
        TIMER = timer;
        EXTI = device.EXTI;
    }

    #[interrupt(binds = TIM17, resources = [TIMER, LED])]
    fn on_timer_tick() {
        resources.LED.toggle().unwrap();
        resources.TIMER.clear_irq();
    }

    #[interrupt(binds = EXTI4_15, resources = [EXTI])]
    fn on_button_click() {
        hprintln!(
            "{}",
            resources
                .EXTI
                .is_pending(Event::GPIO13, SignalEdge::Falling)
        )
        .unwrap();
        resources.EXTI.unpend(Event::GPIO13);
    }
};
