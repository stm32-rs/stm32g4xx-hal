#![deny(warnings)]
#![deny(unsafe_code)]
#![no_main]
#![no_std]

extern crate cortex_m_rt as rt;

use core::fmt::Write;

use hal::prelude::*;
use hal::pwr::PwrExt;
use hal::serial::*;
use hal::{rcc, stm32};
use stm32g4xx_hal as hal;

use cortex_m_rt::entry;
use log::info;

#[macro_use]
mod utils;

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

    let mut cnt = 0;
    loop {
        if rx1.fifo_threshold_reached() {
            loop {
                match rx1.read() {
                    Err(nb::Error::WouldBlock) => {
                        // no more data available in fifo
                        break;
                    }
                    Err(nb::Error::Other(_err)) => {
                        // Handle other error Overrun, Framing, Noise or Parity
                    }
                    Ok(byte) => {
                        writeln!(tx1, "{}: {}\r", cnt, byte).unwrap();
                        cnt += 1;
                    }
                }
            }
        }
    }
}
