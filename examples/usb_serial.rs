//! CDC-ACM serial port example using polling in a busy loop.
//! This example currently requires an 8MHz external oscillator
//! and assumed an LED is connected to port A6.
//!
//! Further work could be done to setup the HSI48 and the clock
//! recovery system to generate the USB clock.

#![no_std]
#![no_main]

use defmt_rtt as _;

use hal::rcc::PllMDiv;
use hal::rcc::PllNMul;
use hal::rcc::PllQDiv;
use hal::rcc::PllRDiv;
use hal::usb::configure_usb_clock_source;
use hal::usb::ClockSource;
use panic_probe as _;

use stm32g4 as _;

#[cfg(feature = "defmt-logging")]
#[defmt::panic_handler]
fn panic() -> ! {
    cortex_m::asm::udf()
}

pub fn exit() -> ! {
    loop {
        cortex_m::asm::bkpt();
    }
}

use hal::rcc::{Config, PLLSrc, Prescaler};

use hal::rcc::clock_recovery_system::{CrsConfig, CrsExt};

use stm32g4xx_hal as hal;

use hal::prelude::*;
use hal::stm32;
use hal::usb::{Peripheral, UsbBus};

use usb_device::prelude::*;
use usbd_serial::{SerialPort, USB_CLASS_CDC};

#[cortex_m_rt::entry]
fn main() -> ! {
    let dp = stm32::Peripherals::take().unwrap();

    let rcc = dp.RCC.constrain();

    // This sets the clocks up as follows
    // - 8 MHz external oscillator
    // - Sysclck and HCLK at 144 MHz
    // - APB1 = PCLK1 =  72 MHz
    // - APB2 = PCLK2 =  72 MHz
    // - USB = 48 MHz
    let mut rcc = rcc.freeze(
        Config::new(hal::rcc::SysClockSrc::HSE(8.mhz()))
            .pll_cfg(hal::rcc::PllConfig {
                mux: PLLSrc::HSE(8.mhz()),
                m: PllMDiv::DIV_1,
                n: PllNMul::MUL_36,
                r: Some(PllRDiv::DIV_2),
                q: Some(PllQDiv::DIV_6),
                p: None,
            })
            .ahb_psc(Prescaler::Div2)
            .apb1_psc(Prescaler::Div2)
            .apb2_psc(Prescaler::Div2),
    );

    // Example of setting up the HSI48 with the clock recovery system.
    let crs = dp.CRS.constrain();
    let crs_config = CrsConfig {};
    crs.configure(crs_config, &rcc);
    configure_usb_clock_source(ClockSource::Hsi48, &rcc);

    // Configure an LED
    let gpioa = dp.GPIOA.split(&mut rcc);

    let mut led = gpioa.pa6.into_push_pull_output();

    let usb = Peripheral { usb: dp.USB };
    let usb_bus = UsbBus::new(usb);

    let rx_buffer: [u8; 128] = [0; 128];
    let tx_buffer: [u8; 128] = [0; 128];

    let mut serial = SerialPort::new_with_store(&usb_bus, rx_buffer, tx_buffer);

    let mut usb_dev = UsbDeviceBuilder::new(&usb_bus, UsbVidPid(0x16c0, 0x27dd))
        .manufacturer("Fake company")
        .product("Serial port")
        .serial_number("TEST")
        .device_class(USB_CLASS_CDC)
        .build();

    loop {
        if !usb_dev.poll(&mut [&mut serial]) {
            continue;
        }

        let mut buf = [0u8; 64];

        match serial.read(&mut buf) {
            Ok(count) if count > 0 => {
                led.set_low().ok(); // Turn on

                // Echo back in upper case
                for c in buf[0..count].iter_mut() {
                    if 0x61 <= *c && *c <= 0x7a {
                        *c &= !0x20;
                    }
                }

                let mut write_offset = 0;
                while write_offset < count {
                    match serial.write(&buf[write_offset..count]) {
                        Ok(len) if len > 0 => {
                            write_offset += len;
                        }
                        _ => {}
                    }
                }
            }
            _ => {}
        }

        led.set_high().ok(); // Turn off
    }
}
