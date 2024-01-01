#![allow(unsafe_code)]
cfg_if::cfg_if! {
    if #[cfg(all(feature = "log-rtt", feature = "defmt"))] {
        #[allow(unused_imports)]
        pub use defmt::{info, trace, warn, debug, error};

    } else {
        #[allow(unused_imports)]
        pub use log::{info, trace, warn, debug, error};
    }
}

cfg_if::cfg_if! {
    if #[cfg(feature = "log-itm")] {
        use panic_itm as _;

        use lazy_static::lazy_static;
        use log::LevelFilter;

        pub use cortex_m_log::log::Logger;

        use cortex_m_log::{
            destination::Itm as ItmDest,
            printer::itm::InterruptSync,
            modes::InterruptFree,
            printer::itm::ItmSync
        };

        lazy_static! {
            pub static ref LOGGER: Logger<ItmSync<InterruptFree>> = Logger {
                level: LevelFilter::Info,
                inner: unsafe {
                    InterruptSync::new(
                        // We must not use Peripherals::steal() here to get an ITM instance, as the
                        // code might expect to be able to call Peripherals::take() later on.
                        ItmDest::new(core::mem::transmute(()))
                    )
                },
            };
        }

        #[allow(unused_macros)]
        macro_rules! println {
            ($($tt:tt)*) => {
                log::info!($($tt,)*);
            };
        }

        #[allow(unused_imports)]
        pub(crate) use println;

        #[allow(dead_code)]
        pub fn init() {
            cortex_m_log::log::init(&LOGGER).unwrap();
        }

    }
    else if #[cfg(all(feature = "log-rtt"/*, feature = "defmt"*/))] {
        use defmt_rtt as _; // global logger
        use panic_probe as _;
        #[allow(unused_imports)]
        pub use defmt::Logger;
        #[allow(unused_imports)]
        pub use defmt::println;

        #[allow(dead_code)]
        pub fn init() {}
    }
    else if #[cfg(all(feature = "log-rtt", not(feature = "defmt")))] {
        // TODO
    }
    else if #[cfg(feature = "log-semihost")] {
        use panic_semihosting as _;

        use lazy_static::lazy_static;
        use log::LevelFilter;

        pub use cortex_m_log::log::Logger;
        use cortex_m_log::printer::semihosting;
        use cortex_m_log::printer::semihosting::Semihosting;
        use cortex_m_log::modes::InterruptOk;
        use cortex_m_semihosting::hio::HStdout;

        lazy_static! {
            static ref LOGGER: Logger<Semihosting<InterruptOk, HStdout>> = Logger {
                level: LevelFilter::Info,
                inner: semihosting::InterruptOk::<_>::stdout().expect("Get Semihosting stdout"),
            };
        }

        #[allow(unused_macros)]
        macro_rules! println {
            ($($tt:tt)*) => {
                cortex_m_semihosting::hprintln!($($tt,)*).unwrap();
            };
        }

        #[allow(unused_imports)]
        pub(crate) use println;

        #[allow(dead_code)]
        pub fn init() {
            cortex_m_log::log::init(&LOGGER).unwrap();
        }
    }
    else {
        use panic_halt as _;

        #[allow(dead_code)]
        pub fn init() {}
    }
}
