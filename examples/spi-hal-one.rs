// This example is to test the SPI without any external devices,
// it writes to the MOSI line and logs whatever is received on the MISO line
//
// MISO and MOSI should be connected to make the assertions work

#![no_main]
#![no_std]

use crate::hal::{
    prelude::*,
    pwr::PwrExt,
    rcc::Config,
    spi,
    stm32::Peripherals,
    time::RateExtU32,
};

use cortex_m_rt::entry;
use stm32g4xx_hal as hal;

#[macro_use]
mod utils;
use utils::logger::info;

#[entry]
fn main() -> ! {
    utils::logger::init();
    info!("Logger init");

    let dp = Peripherals::take().unwrap();
    let rcc = dp.RCC.constrain();
    let pwr = dp.PWR.constrain().freeze();
    let mut rcc = rcc.freeze(
        Config::hsi(),
        pwr
    );

    // let gpioa = dp.GPIOA.split(&mut rcc);
    let gpioa = dp.GPIOA.split(&mut rcc);
    let sclk = gpioa.pa5.into_alternate();
    let miso = gpioa.pa6.into_alternate();
    let mosi = gpioa.pa7.into_alternate();

    // 1/8 SPI/SysClk ratio seems to be the upper limit for continuous transmission
    // one byte at a time
    // 1/4 works well when writing two packed bytes at once
    // At 1/2 the clock stays on for ~80% of the time
    let mut spi = dp
        .SPI1
        .spi((sclk, miso, mosi), spi::MODE_0, 8.MHz(), &mut rcc);
    let mut cs = gpioa.pa8.into_push_pull_output();
    cs.set_high();

    // Odd number of bits to test packing edge case
    const MESSAGE: &[u8] = "Hello world, but longer".as_bytes();
    cs.set_low();
    spi.write(MESSAGE).unwrap();
    SpiBus::<u8>::flush(&mut spi).unwrap();
    cs.set_high();

    let received = &mut [0u8; MESSAGE.len()];
    spi.read(received).unwrap();

    cortex_m::asm::delay(100);
    cs.set_low();
    spi.transfer(received, MESSAGE).unwrap();
    // downside of having 8 and 16 bit impls on the same struct is you have to specify which flush
    // impl to call, although internally they call the same function
    SpiBus::<u8>::flush(&mut spi).unwrap();
    cs.set_high();
    
    info!("Received {:?}", core::str::from_utf8(received).ok());
    assert_eq!(MESSAGE, received);

    cs.set_low();
    spi.transfer_in_place(received).unwrap();
    SpiBus::<u8>::flush(&mut spi).unwrap();
    cs.set_high();

    info!("Received {:?}", core::str::from_utf8(received).ok());
    assert_eq!(MESSAGE, received);

    // Switch between 8 and 16 bit frames on the fly
    const TX_16B: &[u16] = &[0xf00f, 0xfeef, 0xfaaf];
    let rx_16b = &mut [0u16; TX_16B.len()];

    cs.set_low();
    spi.transfer(rx_16b, TX_16B).unwrap();
    // internally works the same as SpiBus::<u8>::flush()
    SpiBus::<u16>::flush(&mut spi).unwrap();
    cs.set_high();

    info!("Received {:?}", rx_16b);
    assert_eq!(TX_16B, rx_16b);

    cs.set_low();
    spi.transfer_in_place(rx_16b).unwrap();
    SpiBus::<u16>::flush(&mut spi).unwrap();
    cs.set_high();

    info!("Received {:?}", rx_16b);
    assert_eq!(TX_16B, rx_16b);

    loop {
        cortex_m::asm::nop();
    }
}
