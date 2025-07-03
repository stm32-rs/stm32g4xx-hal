#![deny(warnings)]
#![deny(unsafe_code)]
#![no_main]
#![no_std]
#![allow(clippy::uninlined_format_args)]

extern crate cortex_m_rt as rt;

use core::fmt::Write;

use embedded_io::{Read, ReadReady};
use hal::prelude::*;
use hal::pwr::PwrExt;
use hal::serial::*;
use hal::{rcc, stm32};
use stm32g4xx_hal as hal;

use cortex_m_rt::entry;

#[macro_use]
mod utils;
use utils::logger::info;

#[entry]
fn main() -> ! {
    utils::logger::init();

    info!("start");
    let dp = stm32::Peripherals::take().expect("cannot take peripherals");
    let pwr = dp.PWR.constrain().freeze();
    let mut rcc = dp.RCC.freeze(rcc::Config::hsi(), pwr);

    info!("Init UART");
    let gpioa = dp.GPIOA.split(&mut rcc);
    let tx = gpioa.pa2.into_alternate();
    let rx = gpioa.pa3.into_alternate();
    let mut usart = dp
        .USART2
        .usart(
            tx,
            rx,
            FullConfig::default()
                .baudrate(115200.bps())
                .fifo_enable()
                .rx_fifo_enable_interrupt()
                .rx_fifo_threshold(FifoThreshold::FIFO_4_BYTES),
            &mut rcc,
        )
        .unwrap();

    writeln!(usart, "Hello USART2\r\n").unwrap();

    let (mut tx1, mut rx1) = usart.split();

    let mut buffer = [0; 4];
    let mut cnt = 0;
    loop {
        if rx1.fifo_threshold_reached() {
            loop {
                match rx1.read_ready() {
                    Ok(true) => (),
                    Ok(false) => break, // no more data available in fifo
                    Err(e) => {
                        // Handle other error Overrun, Framing, Noise or Parity
                        utils::logger::error!("Error: {:?}", e);
                    }
                }

                let count = rx1.read(&mut buffer).unwrap();
                let bytes = &buffer[count];
                writeln!(tx1, "{}: {}\r", cnt, bytes).unwrap();
                cnt += count;
            }
        }
    }
}
