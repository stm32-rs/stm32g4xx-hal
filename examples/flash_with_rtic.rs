#![allow(unsafe_code)]
#![allow(warnings)]
#![allow(missing_docs)]
#![allow(unused_variables)]
#![no_main]
#![no_std]

mod utils;

#[rtic::app(device = stm32g4xx_hal::stm32, peripherals = true)]
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

    #[init]
    fn init(cx: init::Context) -> (Shared, Local, init::Monotonics) {
        crate::utils::logger::init();

        let dp = cx.device;
        let cp = cx.core;

        let rcc = dp.RCC.constrain();
        let pwr = dp.PWR.constrain().freeze();
        let mut rcc = rcc.freeze(stm32g4xx_hal::rcc::Config::hsi(), pwr);
        let mut delay = cp.SYST.delay(&rcc.clocks);

        // *** FLASH Memory ***
        let one_byte = [0x12];
        let two_bytes = [0xAB, 0xCD];
        let three_bytes = [0x12, 0x34, 0x56];
        let four_bytes = [0xAB, 0xCD, 0xEF, 0xBA];
        let eight_bytes = [0x12, 0x34, 0x56, 0x78, 0x9A, 0xBC, 0xDE, 0xF0];
        let sixteen_bytes = [
            0x12, 0x34, 0x56, 0x78, 0x9A, 0xBC, 0xDE, 0xF0, 0x12, 0x34, 0x56, 0x78, 0x9A, 0xBC,
            0xDE, 0xF0,
        ];
        let mut flash = dp.FLASH.constrain();
        let mut flash_writer = flash.writer::<2048>(FlashSize::Sz256K);

        const FLASH_SPACING: u32 = 16; // Separate flash writes by 16 bytes
        const FLASH_EXAMPLE_START_ADDRESS: u32 = 0x1FC00;

        let address = |i| FLASH_EXAMPLE_START_ADDRESS + i as u32 * FLASH_SPACING;

        logger::info!(
            "Erasing 128 bytes at address {:#X}",
            FLASH_EXAMPLE_START_ADDRESS
        );
        flash_writer
            .erase(FLASH_EXAMPLE_START_ADDRESS, 128)
            .unwrap(); // Erase entire page

        let datasets = [
            &one_byte[..],
            &two_bytes[..],
            &three_bytes[..],
            &four_bytes[..],
            &eight_bytes[..],
            &sixteen_bytes[..],
        ];

        for (i, data) in datasets.into_iter().enumerate() {
            let address = address(i);
            let mut do_write = |force_padding| flash_writer.write(address, &data, force_padding);

            if data.len() % 8 != 0 {
                // This test should fail, as the data needs to be divisible by 8 and force padding is false
                assert_eq!(
                    do_write(false),
                    Err(stm32g4xx_hal::flash::Error::ArrayMustBeDivisibleBy8)
                );
            }

            // This test should pass, as the data needs to be divisible by 8 and force padding is true, so the one_byte array will be padded with 7 bytes of 0xFF
            do_write(true).unwrap();
            logger::info!("Wrote {} byte(s) to address {:#X}", data.len(), address);
        }
        logger::info!("Validating data written data by performing read and compare");

        for (i, data) in datasets.into_iter().enumerate() {
            logger::info!("Stuff: {}", i);
            let mut bytes = [0; 16];
            flash_writer.read_exact(address(i), &mut bytes[..data.len()]);

            assert_eq!(&bytes[..data.len()], *data);
            logger::info!("Validated {} byte data", data.len());
        }

        logger::info!(
            "Finished flash example at address {:#X}",
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
