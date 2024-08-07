// This example is to test the SPI with DMA support. Will output a pattern
// in a loop

#![no_main]
#![no_std]

use crate::hal::{
    delay::DelayFromCountDownTimer,
    gpio::gpioa::PA5,
    gpio::gpioa::PA6,
    gpio::gpioa::PA7,
    gpio::Alternate,
    gpio::AF5,
    prelude::*,
    pwr::PwrExt,
    rcc::Config,
    spi,
    stm32::Peripherals,
    time::{ExtU32, RateExtU32},
    timer::Timer,
};

use cortex_m_rt::entry;
use stm32g4xx_hal as hal;
use stm32g4xx_hal::dma::config::DmaConfig;
use stm32g4xx_hal::dma::stream::DMAExt;
use stm32g4xx_hal::dma::TransferExt;

#[macro_use]
mod utils;

const BUFFER_SIZE: usize = 254;

#[entry]
fn main() -> ! {
    utils::logger::init();

    let dp = Peripherals::take().unwrap();
    let rcc = dp.RCC.constrain();
    let pwr = dp.PWR.constrain().freeze();
    let mut rcc = rcc.freeze(Config::hsi(), pwr);
    let timer2 = Timer::new(dp.TIM2, &rcc.clocks);
    let mut delay_tim2 = DelayFromCountDownTimer::new(timer2.start_count_down(100.millis()));

    let gpioa = dp.GPIOA.split(&mut rcc);
    let sclk: PA5<Alternate<AF5>> = gpioa.pa5.into_alternate();
    let miso: PA6<Alternate<AF5>> = gpioa.pa6.into_alternate();
    let mosi: PA7<Alternate<AF5>> = gpioa.pa7.into_alternate();

    let spi = dp
        .SPI1
        .spi((sclk, miso, mosi), spi::MODE_0, 400.kHz(), &mut rcc);
    let streams = dp.DMA1.split(&rcc);
    let config = DmaConfig::default()
        .transfer_complete_interrupt(false)
        .circular_buffer(true)
        .memory_increment(true);

    let mut buf: [u8; BUFFER_SIZE] = [0; BUFFER_SIZE];
    /* Set a some datas */
    for (index, item) in buf.iter_mut().enumerate().take(BUFFER_SIZE) {
        *item = index as u8;
    }
    let dma_buf = cortex_m::singleton!(: [u8; BUFFER_SIZE] = buf).unwrap();
    let mut transfer_dma =
        streams
            .0
            .into_memory_to_peripheral_transfer(spi.enable_tx_dma(), &mut dma_buf[..], config);
    transfer_dma.start(|_spi| {});
    loop {
        delay_tim2.delay_ms(1000_u16);
    }
}
