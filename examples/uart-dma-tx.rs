#![deny(warnings)]
#![deny(unsafe_code)]
#![no_main]
#![no_std]

extern crate cortex_m_rt as rt;

use core::fmt::Write as _;
use embedded_io::Write as _;
use hal::dma::stream::DMAExt;
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
    let mut led1 = gpioa.pa5.into_push_pull_output();
    let mut led2 = gpioa.pa6.into_push_pull_output();

    info!("Start writing");

    writeln!(usart, "Hello without DMA\r").unwrap();

    let tx_buffer = cortex_m::singleton!(: [u8; 64] = [0; 64]).unwrap();

    let (tx, _rx) = usart.split();

    // Setup DMA for USART2 TX with dma channel 1.
    let mut tx = tx.enable_dma(streams.0, &mut tx_buffer[..]);

    loop {
        for i in 0.. {
            // This will write to the DMA buffer and return directly, proveded the buffer is empty
            // and there is enough room in the buffer for the entire message. Otherwise it will block.
            led1.set_high().unwrap();
            writeln!(&mut tx, "Hello with DMA").unwrap();
            led1.set_low().unwrap();

            delay_syst.delay(100.millis());

            // This will not block due the message above since we have a delay. However do watch out for the
            // `{i}` since this will invoke another DMA transfer. This will block until the first part of
            // the message before the `{i}` has been flushed. Same thing for the part after the `{i}`.
            led2.set_high().unwrap();
            writeln!(&mut tx, "Partially blocking DMA write: {i}!").unwrap();
            led2.set_low().unwrap();

            // Something like this ensures there is only one DMA transfer. This, assuming the DMA buffer is empty and
            // the data fits, guarantees a non blocking write:
            //let mut bytes = heapless::Vec::<_, 64>::new();
            //writeln!(&mut bytes, "Non blocking write of complex message: {i} % 10 = {}!", i % 10);
            //tx.write(bytes.as_slice()).unwrap();

            delay_syst.delay(1000.millis());
        }
    }
}
