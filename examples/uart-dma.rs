#![deny(warnings)]
#![deny(unsafe_code)]
#![no_main]
#![no_std]

extern crate cortex_m_rt as rt;

use core::fmt::Write;

use hal::dma::{config::DmaConfig, stream::DMAExt, TransferExt};
use hal::prelude::*;
use hal::pwr::PwrExt;
use hal::serial::*;
use hal::time::ExtU32;
use hal::{rcc, stm32};
use stm32g4xx_hal as hal;

use cortex_m_rt::entry;
use log::info;

#[macro_use]
mod utils;

#[entry]
fn main() -> ! {
    utils::logger::init();

    let dp = stm32::Peripherals::take().expect("cannot take peripherals");
    let cp = cortex_m::Peripherals::take().expect("cannot take core peripherals");

    let rcc = dp.RCC.constrain();
    let pwr = dp.PWR.constrain().freeze();
    let mut rcc = rcc.freeze(rcc::Config::hsi(), pwr);

    let streams = dp.DMA1.split(&rcc);
    let config = DmaConfig::default()
        .transfer_complete_interrupt(false)
        .circular_buffer(false)
        .memory_increment(true);

    info!("Init UART");
    let gpioa = dp.GPIOA.split(&mut rcc);
    let tx = gpioa.pa2.into_alternate();
    let rx = gpioa.pa3.into_alternate();
    let mut usart = dp
        .USART2
        .usart(
            tx,
            rx,
            FullConfig::default().baudrate(115200.bps()),
            &mut rcc,
        )
        .unwrap();

    let mut delay_syst = cp.SYST.delay(&rcc.clocks);
    let mut led = gpioa.pa5.into_push_pull_output();

    info!("Start writing");

    writeln!(usart, "Hello without DMA\r").unwrap();

    let tx_buffer = cortex_m::singleton!(: [u8; 17] = *b"Hello with DMA!\r\n").unwrap();

    let (tx, _rx) = usart.split();

    // Setup DMA for USART2 TX with dma channel 1.
    let mut transfer =
        streams
            .0
            .into_memory_to_peripheral_transfer(tx.enable_dma(), &mut tx_buffer[..], config);

    transfer.start(|_tx| {});
    loop {
        while !transfer.get_transfer_complete_flag() {}

        delay_syst.delay(1000.millis());
        led.toggle().unwrap();
        transfer.restart(|_tx| {});
    }
}
