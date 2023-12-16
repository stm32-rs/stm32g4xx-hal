#![no_main]
#![no_std]

use stm32g4xx_hal::{
    //delay::{DelayExt, SYSTDelayExt},
    gpio::{gpioc, ExtiPin, GpioExt, Input, PullDown, SignalEdge},
    rcc::RccExt,
    stm32,
    stm32::{interrupt, Interrupt},
    syscfg::SysCfgExt,
};

use core::cell::RefCell;
use core::sync::atomic::{AtomicBool, Ordering};
use cortex_m::{asm::wfi, interrupt::Mutex};
use cortex_m_rt::entry;
use embedded_hal::digital::v2::OutputPin;

type ButtonPin = gpioc::PC13<Input<PullDown>>;

// Make LED pin globally available
static G_BUTTON: Mutex<RefCell<Option<ButtonPin>>> = Mutex::new(RefCell::new(None));
static G_LED_ON: AtomicBool = AtomicBool::new(true);

#[macro_use]
mod utils;

use utils::logger::println;

// Define an interupt handler, i.e. function to call when interrupt occurs.
// This specific interrupt will "trip" when the button is pressed
#[interrupt]
fn EXTI15_10() {
    static mut BUTTON: Option<ButtonPin> = None;

    println!("Got IRQ!");

    let button = BUTTON.get_or_insert_with(|| {
        println!("Transfer Button into EXTI interrupt");
        cortex_m::interrupt::free(|cs| {
            // Move LED pin here, leaving a None in its place
            G_BUTTON.borrow(cs).replace(None).unwrap()
        })
    });

    let state: bool = G_LED_ON.load(Ordering::Relaxed);
    G_LED_ON.store(!state, Ordering::Relaxed);
    button.clear_interrupt_pending_bit();
}

#[entry]
fn main() -> ! {
    utils::logger::init();

    let mut dp = stm32::Peripherals::take().expect("cannot take peripherals");
    let mut rcc = dp.RCC.constrain();
    let mut syscfg = dp.SYSCFG.constrain();

    println!("Led Init");
    // Configure PA5 pin to blink LED
    let gpioa = dp.GPIOA.split(&mut rcc);
    let mut led = gpioa.pa5.into_push_pull_output();

    println!("Button Init");
    let gpioc = dp.GPIOC.split(&mut rcc);
    let mut button = gpioc.pc13.into_pull_down_input();
    button.make_interrupt_source(&mut syscfg);
    button.trigger_on_edge(&mut dp.EXTI, SignalEdge::Rising);
    button.enable_interrupt(&mut dp.EXTI);

    println!("Set Button into Global Mutex");
    // Move the pin into our global storage
    cortex_m::interrupt::free(|cs| *G_BUTTON.borrow(cs).borrow_mut() = Some(button));

    println!("Enable EXTI Interrupt");
    unsafe {
        cortex_m::peripheral::NVIC::unmask(Interrupt::EXTI15_10);
    }

    //let mut delay = cp.SYST.delay(&rcc.clocks);

    println!("Start Loop");
    loop {
        wfi();
        println!("Check");

        if G_LED_ON.load(Ordering::Relaxed) {
            println!("Turn Led On!");
            led.set_high().unwrap();
        } else {
            println!("Turn Led Off!");
            led.set_low().unwrap();
        }
    }
}
