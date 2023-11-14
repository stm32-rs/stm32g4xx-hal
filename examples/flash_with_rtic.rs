#![allow(unsafe_code)]
#![allow(warnings)]
#![allow(missing_docs)]
#![allow(unused_variables)]
#![no_main]
#![no_std]

#[rtic::app(device = stm32g4xx_hal::stm32g4::stm32g474, peripherals = true)]
mod app {
    use stm32g4xx_hal::flash::{FlashExt, FlashSize, FlashWriter, Parts};
    use stm32g4xx_hal::prelude::*;
    use stm32g4xx_hal::rcc::{PllConfig, RccExt};

    use panic_halt as _; // you can put a breakpoint on `rust_begin_unwind` to catch panics

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
        // let dp = Peripherals::take().unwrap();
        // let cp = cortex_m::Peripherals::take().expect("cannot take core peripherals");

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

        let mut rcc = rcc.freeze(clock_config);

        unsafe {
            let mut flash = &(*stm32g4xx_hal::stm32::FLASH::ptr());
            flash.acr.modify(|_, w| {
                w.latency().bits(0b1000) // 8 wait states
            });
        }

        // *** FLASH Memory ***
        //let mut data = [0xBE, 0xEF, 0xCA, 0xFE];
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
        const FLASH_SPACING: usize = 16; // Separate flash writes by 16 bytes

        flash_writer.erase(0x1FC00, 128).unwrap(); // Erase entire page

        for i in 0..6 {
            match i {
                0 => flash_writer
                    .write(0x1FC00 + i * FLASH_SPACING, &one_byte)
                    .unwrap(),
                1 => flash_writer
                    .write(0x1FC00 + i * FLASH_SPACING, &two_bytes)
                    .unwrap(),
                2 => flash_writer
                    .write(0x1FC00 + i * FLASH_SPACING, &three_bytes)
                    .unwrap(),
                3 => flash_writer
                    .write(0x1FC00 + i * FLASH_SPACING, &four_bytes)
                    .unwrap(),
                4 => flash_writer
                    .write(0x1FC00 + i * FLASH_SPACING, &eight_bytes)
                    .unwrap(),
                5 => flash_writer
                    .write(0x1FC00 + i * 16, &sixteen_bytes)
                    .unwrap(),
                _ => (),
            }
        }

        for i in 0..6 {
            match i {
                0 => {
                    let bytes = flash_writer.read(0x1FC00 as u32, one_byte.len()).unwrap();
                    assert!(compare_arrays(&bytes, &one_byte));
                }
                1 => {
                    let bytes = flash_writer
                        .read(0x1FC00 + i * FLASH_SPACING, two_bytes.len())
                        .unwrap();
                    assert!(compare_arrays(&bytes, &two_bytes));
                }
                2 => {
                    let bytes = flash_writer
                        .read(0x1FC00 + i * FLASH_SPACING, three_bytes.len())
                        .unwrap();
                    assert!(compare_arrays(&bytes, &three_bytes));
                }
                3 => {
                    let bytes = flash_writer
                        .read(0x1FC00 + i * FLASH_SPACING, four_bytes.len())
                        .unwrap();
                    assert!(compare_arrays(&bytes, &four_bytes));
                }
                4 => {
                    let bytes = flash_writer
                        .read(0x1FC00 + i * FLASH_SPACING, eight_bytes.len())
                        .unwrap();
                    assert!(compare_arrays(&bytes, &eight_bytes));
                }
                5 => {
                    let bytes = flash_writer
                        .read(0x1FC00 + i * FLASH_SPACING, sixteen_bytes.len())
                        .unwrap();
                    assert!(compare_arrays(&bytes, &sixteen_bytes));
                }
                _ => (),
            }
        }

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
