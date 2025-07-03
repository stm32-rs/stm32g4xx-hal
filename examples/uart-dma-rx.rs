#![deny(warnings)]
#![deny(unsafe_code)]
#![no_main]
#![no_std]
#![allow(clippy::uninlined_format_args)]

extern crate cortex_m_rt as rt;

use hal::dma::{channel::DMAExt, config::DmaConfig, TransferExt};
use hal::prelude::*;
use hal::pwr::PwrExt;
use hal::serial::*;
use hal::{rcc, stm32};
use stm32g4xx_hal as hal;

use cortex_m_rt::entry;
use utils::logger::info;

#[macro_use]
mod utils;

#[entry]
fn main() -> ! {
    utils::logger::init();

    info!("Initializing...");

    let dp = stm32::Peripherals::take().expect("cannot take peripherals");

    let rcc = dp.RCC.constrain();
    let pwr = dp.PWR.constrain().freeze();
    let mut rcc = rcc.freeze(rcc::Config::hsi(), pwr);

    let channels = dp.DMA1.split(&rcc);
    let config = DmaConfig::default()
        .transfer_complete_interrupt(false)
        .circular_buffer(true)
        .memory_increment(true);

    info!("Init UART");
    //let gpioa = dp.GPIOA.split(&mut rcc);
    //let tx = gpioa.pa2.into_alternate();
    //let rx = gpioa.pa3.into_alternate();
    let gpioc = dp.GPIOC.split(&mut rcc);
    let tx = gpioc.pc10.into_alternate();
    let rx = gpioc.pc11.into_alternate();

    let usart = dp
        //.USART2
        .USART3
        .usart(
            tx,
            rx,
            FullConfig::default()
                .baudrate(115200.bps())
                .receiver_timeout_us(1000), // Timeout after 1ms
            &mut rcc,
        )
        .unwrap();

    //let mut led = gpioa.pa5.into_push_pull_output();

    info!("Start reading");

    let rx_buffer = cortex_m::singleton!(: [u8; 256] = [0; 256]).unwrap();

    let (_tx, rx) = usart.split();

    let mut transfer = channels.ch1.into_circ_peripheral_to_memory_transfer(
        rx.enable_dma(),
        &mut rx_buffer[..],
        config,
    );

    transfer.start(|_rx| {});
    loop {
        while !transfer.timeout_lapsed() {}
        transfer.clear_timeout();

        let mut data = [0; 256];
        loop {
            let data = transfer.read_available(&mut data);
            if data.is_empty() {
                break;
            }
            if let Ok(data) = core::str::from_utf8(&*data) {
                info!("Received: '{}'", data);
            } else {
                info!("Received: {:?}", data);
            }
            info!("Sup");
        }
        //led.toggle().unwrap();
    }
}
