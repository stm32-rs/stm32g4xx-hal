use super::*;
use crate::bb;

macro_rules! bus_enable {
    ($PER:ident => ($busX:ty, $bit:literal)) => {
        impl Enable for crate::stm32::$PER {
            #[inline(always)]
            fn enable(rcc: &mut RCC) {
                unsafe {
                    bb::set(Self::Bus::enr(rcc), $bit);
                }
                // // Stall the pipeline to work around erratum 2.1.13 (DM00037591)
                // cortex_m::asm::dsb();
            }
            #[inline(always)]
            fn disable(rcc: &mut RCC) {
                unsafe {
                    bb::clear(Self::Bus::enr(rcc), $bit);
                }
            }
            #[inline(always)]
            fn is_enabled() -> bool {
                let rcc = unsafe { &*RCC::ptr() };
                (Self::Bus::enr(rcc).read().bits() >> $bit) & 0x1 != 0
            }
        }
    };
}

macro_rules! bus_smenable {
    ($PER:ident => ($busX:ty, $bit:literal)) => {
        impl SMEnable for crate::stm32::$PER {
            #[inline(always)]
            fn sleep_mode_enable(rcc: &mut RCC) {
                unsafe {
                    bb::set(Self::Bus::smenr(rcc), $bit);
                }
                // // Stall the pipeline to work around erratum 2.1.13 (DM00037591)
                // cortex_m::asm::dsb();
            }
            #[inline(always)]
            fn sleep_mode_disable(rcc: &mut RCC) {
                unsafe {
                    bb::clear(Self::Bus::smenr(rcc), $bit);
                }
            }
            #[inline(always)]
            fn is_sleep_mode_enabled() -> bool {
                let rcc = unsafe { &*RCC::ptr() };
                (Self::Bus::smenr(rcc).read().bits() >> $bit) & 0x1 != 0
            }
        }
    };
}

macro_rules! bus_reset {
    ($PER:ident => ($busX:ty, $bit:literal)) => {
        impl Reset for crate::stm32::$PER {
            #[inline(always)]
            fn reset(rcc: &mut RCC) {
                unsafe {
                    bb::set(Self::Bus::rstr(rcc), $bit);
                    bb::clear(Self::Bus::rstr(rcc), $bit);
                }
            }
        }
    };
}

macro_rules! bus {
    ($($PER:ident => ($busX:ty, $bit:literal),)+) => {
        $(
            impl RccBus for crate::stm32::$PER {
                type Bus = $busX;
            }
            impl crate::rcc::Instance for crate::stm32::$PER {}
            bus_enable!($PER => ($busX, $bit));
            bus_smenable!($PER => ($busX, $bit));
            bus_reset!($PER => ($busX, $bit));
        )+
    }
}

bus! {
    DMA1 => (AHB1, 0),
    DMA2 => (AHB1, 1),
    DMAMUX => (AHB1, 2),
    CORDIC => (AHB1, 3),
    FMAC => (AHB1, 4),
    FLASH => (AHB1, 5),
    CRC => (AHB1, 12),
}

bus! {
    GPIOA => (AHB2, 0),
    GPIOB => (AHB2, 1),
    GPIOC => (AHB2, 2),
    GPIOD => (AHB2, 3),
    GPIOE => (AHB2, 4),
    GPIOF => (AHB2, 5),
    GPIOG => (AHB2, 6),
    ADC12_COMMON => (AHB2, 13),
    DAC1 => (AHB2, 16),
    DAC2 => (AHB2, 17),
    DAC3 => (AHB2, 18),
    DAC4 => (AHB2, 19),
    RNG => (AHB2, 26),
}

#[cfg(any(
    feature = "stm32g473",
    feature = "stm32g474",
    feature = "stm32g483",
    feature = "stm32g484",
    feature = "stm32g491",
    feature = "stm32g4a1",
))]
bus! {
    ADC345_COMMON => (AHB2, 14),
}

#[cfg(any(feature = "stm32g431", feature = "stm32g441", feature = "stm32g484",))]
bus! {
    AES => (AHB2, 24),
}

#[cfg(any(
    feature = "stm32g473",
    feature = "stm32g474",
    feature = "stm32g483",
    feature = "stm32g484"
))]
bus! {
    FMC => (AHB3, 0),
    QUADSPI => (AHB3, 8),
}

bus! {
    TIM2 => (APB1_1, 0),
    TIM3 => (APB1_1, 1),
    TIM4 => (APB1_1, 2),
    TIM6 => (APB1_1, 4),
    TIM7 => (APB1_1, 5),
    CRS => (APB1_1, 8),
    SPI2 => (APB1_1, 14),
    SPI3 => (APB1_1, 15),
    USART2 => (APB1_1, 17),
    USART3 => (APB1_1, 18),
    UART4 => (APB1_1, 19),
    I2C1 => (APB1_1, 21),
    I2C2 => (APB1_1, 22),
    USB => (APB1_1, 23),
    FDCAN1 => (APB1_1, 25),
    PWR => (APB1_1, 28),
    I2C3 => (APB1_1, 30),
    LPTIMER1 => (APB1_1, 31),
    LPUART1 => (APB1_2, 0),
    UCPD1 => (APB1_2, 8),
}

#[cfg(any(
    feature = "stm32g473",
    feature = "stm32g474",
    feature = "stm32g483",
    feature = "stm32g484",
    feature = "stm32g491",
    feature = "stm32g4a1"
))]
bus! {
    FDCAN2 => (APB1_1, 25),
}

#[cfg(any(
    feature = "stm32g473",
    feature = "stm32g474",
    feature = "stm32g483",
    feature = "stm32g484",
))]
bus! {
    FDCAN3 => (APB1_1, 25),
}

#[cfg(any(
    feature = "stm32g473",
    feature = "stm32g474",
    feature = "stm32g483",
    feature = "stm32g484"
))]
bus! {
    TIM5 => (APB1_1, 3),
    I2C4 => (APB1_2, 1),
}

#[cfg(any(
    feature = "stm32g473",
    feature = "stm32g474",
    feature = "stm32g483",
    feature = "stm32g484",
    feature = "stm32g491",
    feature = "stm32g4a1",
))]
bus! {
    UART5 => (APB1_1, 20),
}

bus! {
    SYSCFG => (APB2, 0),
    TIM1 => (APB2, 11),
    SPI1 => (APB2, 12),
    TIM8 => (APB2, 13),
    USART1 => (APB2, 14),
    TIM15 => (APB2, 16),
    TIM16 => (APB2, 17),
    TIM17 => (APB2, 18),
    SAI => (APB2, 21),
}

#[cfg(any(
    feature = "stm32g473",
    feature = "stm32g474",
    feature = "stm32g483",
    feature = "stm32g484"
))]
bus! {
    SPI4 => (APB2, 15),
}

#[cfg(any(
    feature = "stm32g473",
    feature = "stm32g474",
    feature = "stm32g483",
    feature = "stm32g484",
    feature = "stm32g491",
    feature = "stm32g4a1",
))]
bus! {
    TIM20 => (APB2, 20),
}

#[cfg(any(feature = "stm32g474", feature = "stm32g484"))]
bus! {
    HRTIM_TIMA => (APB2, 26),
    HRTIM_TIMB => (APB2, 26),
    HRTIM_TIMC => (APB2, 26),
    HRTIM_TIMD => (APB2, 26),
    HRTIM_TIME => (APB2, 26),
    HRTIM_TIMF => (APB2, 26),
}

#[cfg(any(feature = "stm32g474", feature = "stm32g484"))]
bus! {
    HRTIM_COMMON => (APB2, 26),
}
