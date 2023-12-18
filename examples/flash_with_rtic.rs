#![allow(unsafe_code)]
#![allow(warnings)]
#![allow(missing_docs)]
#![allow(unused_variables)]
#![no_main]
#![no_std]

mod utils;

#[rtic::app(device = stm32g4xx_hal::stm32g4::stm32g474, peripherals = true)]
mod app {
    use crate::utils::logger;
    use stm32g4xx_hal::flash::{FlashExt, FlashSize, FlashWriter, Parts};
    use stm32g4xx_hal::prelude::*;
    use stm32g4xx_hal::pwr::PwrExt;
    use stm32g4xx_hal::rcc::{PllConfig, RccExt};

    const LOG_LEVEL: log::LevelFilter = log::LevelFilter::Info;

    // Resources shared between tasks
    #[shared]
    struct Shared {}

    // Local resources to specific tasks (cannot be shared)
    #[local]
    struct Local {}

    fn compare_arrays(a: &[u8], b: &[u8]) -> bool {
        if a.len() != b.len() {
            return false;
        }
        for i in 0..a.len() {
            if a[i] != b[i] {
                return false;
            }
        }
        true
    }

    #[init]
    fn init(cx: init::Context) -> (Shared, Local, init::Monotonics) {
        crate::utils::logger::init();

        let dp = cx.device;
        let cp = cx.core;

        let rcc = dp.RCC.constrain();
        let mut pll_config = stm32g4xx_hal::rcc::PllConfig::default();

        // Sysclock is based on PLL_R
        pll_config.mux = stm32g4xx_hal::rcc::PLLSrc::HSI; // 16MHz
        pll_config.n = stm32g4xx_hal::rcc::PllNMul::MUL_32;
        pll_config.m = stm32g4xx_hal::rcc::PllMDiv::DIV_2; // f(vco) = 16MHz*32/2 = 256MHz
        pll_config.r = Some(stm32g4xx_hal::rcc::PllRDiv::DIV_2); // f(sysclock) = 256MHz/2 = 128MHz

        // Note to future self: The AHB clock runs the timers, among other things.
        // Please refer to the Clock Tree manual to determine if it is worth
        // changing to a lower speed for battery life savings.
        let mut clock_config = stm32g4xx_hal::rcc::Config::default()
            .pll_cfg(pll_config)
            .clock_src(stm32g4xx_hal::rcc::SysClockSrc::PLL);

        // After clock configuration, the following should be true:
        // Sysclock is 128MHz
        // AHB clock is 128MHz
        // APB1 clock is 128MHz
        // APB2 clock is 128MHz
        // The ADC will ultimately be put into synchronous mode and will derive
        // its clock from the AHB bus clock, with a prescalar of 2 or 4.

        let pwr = dp.PWR.constrain().freeze();
        let mut rcc = rcc.freeze(clock_config, pwr);

        unsafe {
            let mut flash = &(*stm32g4xx_hal::stm32::FLASH::ptr());
            flash.acr.modify(|_, w| {
                w.latency().bits(0b1000) // 8 wait states
            });
        }

        // *** FLASH Memory ***
        let one_byte = [0x12 as u8];
        let two_bytes = [0xAB, 0xCD as u8];
        let three_bytes = [0x12, 0x34, 0x56 as u8];
        let four_bytes = [0xAB, 0xCD, 0xEF, 0xBA as u8];
        let eight_bytes = [0x12, 0x34, 0x56, 0x78, 0x9A, 0xBC, 0xDE, 0xF0 as u8];
        let sixteen_bytes = [
            0x12, 0x34, 0x56, 0x78, 0x9A, 0xBC, 0xDE, 0xF0, 0x12, 0x34, 0x56, 0x78, 0x9A, 0xBC,
            0xDE, 0xF0 as u8,
        ];
        let mut flash = dp.FLASH.constrain();
        let mut flash_writer = flash.writer::<2048>(FlashSize::Sz256K);

        const FLASH_SPACING: u32 = 16; // Separate flash writes by 16 bytes
        const FLASH_EXAMPLE_START_ADDRESS: u32 = 0x1FC00;

        logger::info!(
            "Erasing 128 bytes at address {}",
            FLASH_EXAMPLE_START_ADDRESS
        );
        flash_writer
            .erase(FLASH_EXAMPLE_START_ADDRESS, 128)
            .unwrap(); // Erase entire page

        for i in 0..6 {
            match i {
                0 => {
                    // This test should fail, as the data needs to be divisible by 8 and force padding is false
                    let result = flash_writer.write(
                        FLASH_EXAMPLE_START_ADDRESS + i * FLASH_SPACING,
                        &one_byte,
                        false,
                    );
                    assert!(result.is_err());
                    assert_eq!(
                        result.err().unwrap(),
                        stm32g4xx_hal::flash::Error::ArrayMustBeDivisibleBy8
                    );

                    // This test should pass, as the data needs to be divisible by 8 and force padding is true, so the one_byte array will be padded with 7 bytes of 0xFF
                    let result = flash_writer.write(
                        FLASH_EXAMPLE_START_ADDRESS + i * FLASH_SPACING,
                        &one_byte,
                        true,
                    );
                    assert!(result.is_ok());
                    logger::info!(
                        "Wrote 1 byte to address {}",
                        FLASH_EXAMPLE_START_ADDRESS + i * FLASH_SPACING
                    );
                }
                1 => {
                    // This test should fail, as the data needs to be divisible by 8 and force padding is false
                    let result = flash_writer.write(
                        FLASH_EXAMPLE_START_ADDRESS + i * FLASH_SPACING,
                        &two_bytes,
                        false,
                    );
                    assert!(result.is_err());
                    assert_eq!(
                        result.err().unwrap(),
                        stm32g4xx_hal::flash::Error::ArrayMustBeDivisibleBy8
                    );

                    // This test should pass, as the data needs to be divisible by 8 and force padding is true, so the one_byte array will be padded with 7 bytes of 0xFF
                    let result = flash_writer.write(
                        FLASH_EXAMPLE_START_ADDRESS + i * FLASH_SPACING,
                        &two_bytes,
                        true,
                    );
                    assert!(result.is_ok());
                    logger::info!(
                        "Wrote 2 bytes to address {}",
                        FLASH_EXAMPLE_START_ADDRESS + i * FLASH_SPACING
                    );
                }
                2 => {
                    // This test should fail, as the data needs to be divisible by 8 and force padding is false
                    let result = flash_writer.write(
                        FLASH_EXAMPLE_START_ADDRESS + i * FLASH_SPACING,
                        &three_bytes,
                        false,
                    );
                    assert!(result.is_err());
                    assert_eq!(
                        result.err().unwrap(),
                        stm32g4xx_hal::flash::Error::ArrayMustBeDivisibleBy8
                    );

                    // This test should pass, as the data needs to be divisible by 8 and force padding is true, so the one_byte array will be padded with 7 bytes of 0xFF
                    let result = flash_writer.write(
                        FLASH_EXAMPLE_START_ADDRESS + i * FLASH_SPACING,
                        &three_bytes,
                        true,
                    );
                    assert!(result.is_ok());
                    logger::info!(
                        "Wrote 3 bytes to address {}",
                        FLASH_EXAMPLE_START_ADDRESS + i * FLASH_SPACING
                    );
                }
                3 => {
                    // This test should fail, as the data needs to be divisible by 8 and force padding is false
                    let result = flash_writer.write(
                        FLASH_EXAMPLE_START_ADDRESS + i * FLASH_SPACING,
                        &four_bytes,
                        false,
                    );
                    assert!(result.is_err());
                    assert_eq!(
                        result.err().unwrap(),
                        stm32g4xx_hal::flash::Error::ArrayMustBeDivisibleBy8
                    );

                    // This test should pass, as the data needs to be divisible by 8 and force padding is true, so the one_byte array will be padded with 7 bytes of 0xFF
                    let result = flash_writer.write(
                        FLASH_EXAMPLE_START_ADDRESS + i * FLASH_SPACING,
                        &four_bytes,
                        true,
                    );
                    assert!(result.is_ok());
                    logger::info!(
                        "Wrote 4 bytes to address {}",
                        FLASH_EXAMPLE_START_ADDRESS + i * FLASH_SPACING
                    );
                }
                4 => {
                    flash_writer
                        .write(
                            FLASH_EXAMPLE_START_ADDRESS + i * FLASH_SPACING,
                            &eight_bytes,
                            false,
                        )
                        .unwrap();
                    logger::info!(
                        "Wrote 8 bytes to address {}",
                        FLASH_EXAMPLE_START_ADDRESS + i * FLASH_SPACING
                    );
                }
                5 => {
                    flash_writer
                        .write(FLASH_EXAMPLE_START_ADDRESS + i * 16, &sixteen_bytes, false)
                        .unwrap();
                    logger::info!(
                        "Wrote 16 bytes to address {}",
                        FLASH_EXAMPLE_START_ADDRESS + i * FLASH_SPACING
                    );
                }
                _ => (),
            }
        }

        logger::info!("Validating data written data by performing read and compare");

        for i in 0..6 {
            match i {
                0 => {
                    let bytes = flash_writer
                        .read(FLASH_EXAMPLE_START_ADDRESS as u32, one_byte.len())
                        .unwrap();
                    assert!(compare_arrays(&bytes, &one_byte));
                    logger::info!("Validated 1 byte data");
                }
                1 => {
                    let bytes = flash_writer
                        .read(
                            FLASH_EXAMPLE_START_ADDRESS + i * FLASH_SPACING,
                            two_bytes.len(),
                        )
                        .unwrap();
                    assert!(compare_arrays(&bytes, &two_bytes));
                    logger::info!("Validated 2 byte data");
                }
                2 => {
                    let bytes = flash_writer
                        .read(
                            FLASH_EXAMPLE_START_ADDRESS + i * FLASH_SPACING,
                            three_bytes.len(),
                        )
                        .unwrap();
                    assert!(compare_arrays(&bytes, &three_bytes));
                    logger::info!("Validated 3 byte data");
                }
                3 => {
                    let bytes = flash_writer
                        .read(
                            FLASH_EXAMPLE_START_ADDRESS + i * FLASH_SPACING,
                            four_bytes.len(),
                        )
                        .unwrap();
                    assert!(compare_arrays(&bytes, &four_bytes));
                    logger::info!("Validated 4 byte data");
                }
                4 => {
                    let bytes = flash_writer
                        .read(
                            FLASH_EXAMPLE_START_ADDRESS + i * FLASH_SPACING,
                            eight_bytes.len(),
                        )
                        .unwrap();
                    assert!(compare_arrays(&bytes, &eight_bytes));
                    logger::info!("Validated 8 byte data");
                }
                5 => {
                    let bytes = flash_writer
                        .read(
                            FLASH_EXAMPLE_START_ADDRESS + i * FLASH_SPACING,
                            sixteen_bytes.len(),
                        )
                        .unwrap();
                    assert!(compare_arrays(&bytes, &sixteen_bytes));
                    logger::info!("Validated 5 byte data");
                }
                _ => (),
            }
        }

        logger::info!(
            "Finished flash example at address {}",
            FLASH_EXAMPLE_START_ADDRESS
        );

        (
            // Initialization of shared resources
            Shared {},
            // Initialization of task local resources
            Local {},
            // Move the monotonic timer to the RTIC run-time, this enables
            // scheduling
            init::Monotonics(),
        )
    }

    // Background task, runs whenever no other tasks are running
    #[idle]
    fn idle(mut cx: idle::Context) -> ! {
        loop {
            // Sleep until next interrupt
            cortex_m::asm::wfi();
        }
    }
}
