// DMAMUX driver for STM32G4xx series microcontrollers

use crate::stm32::DMAMUX;

/// Extension trait to split a DMA peripheral into independent channels
pub trait DmaMuxExt {
    /// The type to split the DMA into
    type Channels;

    /// Split the DMA into independent channels
    fn split(self) -> Self::Channels;
}

pub enum DmaMuxIndex {
    dmamux_req_gen0 = 0,
    dmamux_req_gen1 = 1,
    dmamux_req_gen2 = 2,
    dmamux_req_gen3 = 3,
    
    //#[cfg(any(feature = "stm32g484", feature = "stm32g494"))]
    ADC1 = 5,
    DAC1_CH1 = 6,
    DAC1_CH2 = 7,
    TIM6_UP = 8,
    TIM7_UP = 9,
    SPI1_RX = 10,
    SPI1_TX = 11,
    SPI2_RX = 12,
    SPI2_TX = 13,
    SPI3_RX = 14,
    SPI3_TX = 15,
    I2C1_RX = 16,
    I2C1_TX = 17,
    I2C2_RX = 18,
    I2C2_TX = 19,
    I2C3_RX = 20,
    I2C3_TX = 21,
    I2C4_RX = 22,
    I2C4_TX = 23,
    USART1_RX = 24,
    USART1_TX = 25,
    USART2_RX = 26,
    USART2_TX = 27,
    USART3_RX = 28,
    USART3_TX = 29,
    UART4_RX = 30,
    UART4_TX = 31,
    UART5_RX = 32,
    UART5_TX = 33,
    LPUART1_RX = 34,
    LPUART1_TX = 35,
    ADC2 = 36,
    ADC3 = 37,
    ADC4 = 38,
    ADC5 = 39,
    QUADSPI = 40,
    DAC2_CH1 = 41,
    TIM1_CH1 = 42,
    TIM1_CH2 = 43,
    TIM1_CH3 = 44,
    TIM1_CH4 = 45,
    TIM1_UP = 46,
    TIM1_TRIG = 47,
    TIM1_COM = 48,
    TIM8_CH1 = 49,
    TIM8_CH2 = 50,
    TIM8_CH3 = 51,
    TIM8_CH4 = 52,
    TIM8_UP = 53,
    TIM8_TRIG = 54,
    TIM8_COM = 55,
    TIM2_CH1 = 56,
    TIM2_CH2 = 57,
    TIM2_CH3 = 58,
    TIM2_CH4 = 59,
    TIM2_UP = 60,
    TIM3_CH1 = 61,
    TIM3_CH2 = 62,
    TIM3_CH3 = 63,
    TIM3_CH4 = 64,
    TIM3_UP = 65,
    TIM3_TRIG = 66,
    TIM4_CH1 = 67,
    TIM4_CH2 = 68,
    TIM4_CH3 = 69,
    TIM4_CH4 = 70,
    TIM4_UP = 71,
    TIM5_CH1 = 72,
    TIM5_CH2 = 73,
    TIM5_CH3 = 74,
    TIM5_CH4 = 75,
    TIM5_UP = 76,
    TIM5_TRIG = 77,
    TIM15_CH1 = 78,
    TIM15_UP = 79,
    TIM15_TRIG = 80,
    TIM15_COM = 81,
    TIM16_CH1 = 82,
    TIM16_UP = 83,
    TIM17_CH1 = 84,
    TIM17_UP = 85,
    TIM20_CH1 = 86,
    TIM20_CH2 = 87,
    TIM20_CH3 = 88,
    TIM20_CH4 = 89,
    TIM20_UP = 90,
    AES_IN = 91,
    AES_OUT = 92,
    TIM20_TRIG = 93,
    TIM20_COM = 94,
    HRTIM_MASTER = 95,
    HRTIM_TIMA = 96,
    HRTIM_TIMB = 97,
    HRTIM_TIMC = 98,
    HRTIM_TIMD = 99,
    HRTIM_TIME = 100,
    HRTIM_TIMF = 101,
    DAC3_CH1 = 102,
    DAC3_CH2 = 103,
    DAC4_CH1 = 104,
    DAC4_CH2 = 105,
    SPI4_RX = 106,
    SPI4_TX = 107,
    SAI1_A = 108,
    SAI1_B = 109,
    FMAC_READ = 110,
    FMAC_WRITE = 111,
    CORDIC_READ = 112,
    CORDIC_WRITE = 113,
    UCPD1_RX = 114,
    UCPD1_TX = 115,
}

impl DmaMuxIndex {
    pub fn val(self) -> u8 {
        self as u8
    }
}

pub enum DmaMuxTriggerSync {
    EXTI_LINE0 = 0,
    EXTI_LINE1 = 1,
    EXTI_LINE2 = 2,
    EXTI_LINE3 = 3,
    EXTI_LINE4 = 4,
    EXTI_LINE5 = 5,
    EXTI_LINE6 = 6,
    EXTI_LINE7 = 7,
    EXTI_LINE8 = 8,
    EXTI_LINE9 = 9,
    EXTI_LINE10 = 10,
    EXTI_LINE11 = 11,
    EXTI_LINE12 = 12,
    EXTI_LINE13 = 13,
    EXTI_LINE14 = 14,
    EXTI_LINE15 = 15,
    dmamux_evt0 = 16,
    dmamux_evt1 = 17,
    dmamux_evt2 = 18,
    dmamux_evt3 = 19,

    //#[cfg(feature = "stm32g474")]
    LPTIM1_OUT = 20,
}
impl DmaMuxTriggerSync {
    pub fn val(self) -> u8 {
        self as u8
    }
}

pub trait DmaMuxChannel {
    fn select_peripheral(&mut self, index: DmaMuxIndex);
}

macro_rules! dma_mux {
    (
        channels: {
            $( $Ci:ident: ($chi:ident, $cr:ident), )+
        },
    ) => {

        /// DMAMUX channels
        pub struct Channels {
            $( pub $chi: $Ci, )+
        }

        $(
            /// Singleton that represents a DMAMUX channel
            pub struct $Ci {
                _0: (),
            }

            impl DmaMuxChannel for $Ci {
                fn select_peripheral(&mut self, index: DmaMuxIndex) {
                    let reg = unsafe { &(*DMAMUX::ptr()).$cr };
                    reg.write( |w| unsafe {
                        w.dmareq_id().bits(index.val())
                        .ege().set_bit()
                    });

                }
            }
        )+

    }
}

//#[cfg(any(feature = "stm32g474", feature = "stm32g484"))]
dma_mux!(
    channels: {
        C0: (ch0, c0cr),
        C1: (ch1, c1cr),
        C2: (ch2, c2cr),
        C3: (ch3, c3cr),
        C4: (ch4, c4cr),
        C5: (ch5, c5cr),
        C6: (ch6, c6cr),
        C7: (ch7, c7cr),
        C8: (ch8, c8cr),
        C9: (ch9, c9cr),
        C10: (ch10, c10cr),
        C11: (ch11, c11cr),
        C12: (ch12, c12cr),
        C13: (ch13, c13cr),
        C14: (ch14, c14cr),
        C15: (ch15, c15cr),
    },
);

impl DmaMuxExt for DMAMUX {
    type Channels = Channels;

    fn split(self) -> Self::Channels {
        Channels {
            ch0: C0 { _0: () },
            ch1: C1 { _0: () },
            ch2: C2 { _0: () },
            ch3: C3 { _0: () },
            ch4: C4 { _0: () },
            ch5: C5 { _0: () },
            ch6: C6 { _0: () },
            ch7: C7 { _0: () },
            ch8: C8 { _0: () },
            ch9: C9 { _0: () },
            ch10: C10 { _0: () },
            ch11: C11 { _0: () },
            ch12: C12 { _0: () },
            ch13: C13 { _0: () },
            ch14: C14 { _0: () },
            ch15: C15 { _0: () },            
        }
    }
}
