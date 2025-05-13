use crate::{
    gpio::{gpioa, gpiob, gpioc, gpiof, Analog},
    opamp,
};

#[cfg(feature = "adc3")]
use crate::gpio::{gpiod, gpioe};

use super::{adc_channel_helper, adc_opamp, adc_pins, temperature::Temperature, Vbat, Vref};

#[cfg(any(feature = "stm32g431", feature = "stm32g441"))]
adc_pins!(
    gpioa::PA0<Analog> => (ADC1, 1),
    gpioa::PA0<Analog> => (ADC2, 1),
    gpioa::PA1<Analog> => (ADC1, 2),
    gpioa::PA1<Analog> => (ADC2, 2),
    gpioa::PA2<Analog> => (ADC1, 3),
    gpioa::PA3<Analog> => (ADC1, 4),
    gpioa::PA4<Analog> => (ADC2, 17),
    gpioa::PA5<Analog> => (ADC2, 13),
    gpioa::PA6<Analog> => (ADC2, 3),
    gpioa::PA7<Analog> => (ADC2, 4),

    gpiob::PB0<Analog> => (ADC1, 15),
    gpiob::PB1<Analog> => (ADC1, 12),
    gpiob::PB2<Analog> => (ADC2, 12),
    gpiob::PB11<Analog> => (ADC1, 14),
    gpiob::PB11<Analog> => (ADC2, 14),
    gpiob::PB12<Analog> => (ADC1, 11),
    gpiob::PB15<Analog> => (ADC2, 15),

    gpioc::PC0<Analog> => (ADC1, 6),
    gpioc::PC0<Analog> => (ADC2, 6),
    gpioc::PC1<Analog> => (ADC1, 7),
    gpioc::PC1<Analog> => (ADC2, 7),
    gpioc::PC2<Analog> => (ADC1, 8),
    gpioc::PC2<Analog> => (ADC2, 8),
    gpioc::PC3<Analog> => (ADC1, 9),
    gpioc::PC3<Analog> => (ADC2, 9),
    gpioc::PC4<Analog> => (ADC2, 5),
    gpioc::PC5<Analog> => (ADC2, 11),

    gpiof::PF0<Analog> => (ADC1, 10),
    gpiof::PF1<Analog> => (ADC2, 10),

    Temperature => (ADC1, 16),
    Vbat => (ADC1, 17),
    Vref => (ADC1, 18),
);

#[cfg(any(
    feature = "stm32g473",
    feature = "stm32g474",
    feature = "stm32g483",
    feature = "stm32g484",
))]
adc_pins!(
    gpioa::PA0<Analog> => (ADC1, 1),
    gpioa::PA0<Analog> => (ADC2, 1),
    gpioa::PA1<Analog> => (ADC1, 2),
    gpioa::PA1<Analog> => (ADC2, 2),
    gpioa::PA2<Analog> => (ADC1, 3),
    gpioa::PA3<Analog> => (ADC1, 4),
    gpioa::PA4<Analog> => (ADC2, 17),
    gpioa::PA5<Analog> => (ADC2, 13),
    gpioa::PA6<Analog> => (ADC2, 3),
    gpioa::PA7<Analog> => (ADC2, 4),
    gpioa::PA8<Analog> => (ADC5, 1),
    gpioa::PA9<Analog> => (ADC5, 2),

    gpiob::PB0<Analog> => (ADC3, 12),
    gpiob::PB0<Analog> => (ADC1, 15),
    gpiob::PB1<Analog> => (ADC3, 1),
    gpiob::PB1<Analog> => (ADC1, 12),
    gpiob::PB2<Analog> => (ADC2, 12),
    gpiob::PB11<Analog> => (ADC1, 14),
    gpiob::PB11<Analog> => (ADC2, 14),
    gpiob::PB12<Analog> => (ADC4, 3),
    gpiob::PB12<Analog> => (ADC1, 11),
    gpiob::PB13<Analog> => (ADC3, 5),
    gpiob::PB14<Analog> => (ADC4, 4),
    gpiob::PB14<Analog> => (ADC1, 5),
    gpiob::PB15<Analog> => (ADC4, 5),
    gpiob::PB15<Analog> => (ADC2, 15),

    gpioc::PC0<Analog> => (ADC1, 6),
    gpioc::PC0<Analog> => (ADC2, 6),
    gpioc::PC1<Analog> => (ADC1, 7),
    gpioc::PC1<Analog> => (ADC2, 7),
    gpioc::PC2<Analog> => (ADC1, 8),
    gpioc::PC2<Analog> => (ADC2, 8),
    gpioc::PC3<Analog> => (ADC1, 9),
    gpioc::PC3<Analog> => (ADC2, 9),
    gpioc::PC4<Analog> => (ADC2, 5),
    gpioc::PC5<Analog> => (ADC2, 11),

    gpioe::PE7<Analog> => (ADC3, 4),
    gpioe::PE8<Analog> => (ADC3, 6),
    gpioe::PE8<Analog> => (ADC4, 6),
    gpioe::PE8<Analog> => (ADC5, 6),
    gpioe::PE9<Analog> => (ADC3, 2),
    gpioe::PE10<Analog> => (ADC3, 14),
    gpioe::PE10<Analog> => (ADC4, 14),
    gpioe::PE10<Analog> => (ADC5, 14),
    gpioe::PE11<Analog> => (ADC3, 15),
    gpioe::PE11<Analog> => (ADC4, 15),
    gpioe::PE11<Analog> => (ADC5, 15),
    gpioe::PE12<Analog> => (ADC3, 16),
    gpioe::PE12<Analog> => (ADC4, 16),
    gpioe::PE12<Analog> => (ADC5, 16),
    gpioe::PE13<Analog> => (ADC3, 3),
    gpioe::PE14<Analog> => (ADC4, 1),
    gpioe::PE15<Analog> => (ADC4, 2),

    gpiod::PD8<Analog> => (ADC4, 12),
    gpiod::PD8<Analog> => (ADC5, 12),
    gpiod::PD9<Analog> => (ADC4, 13),
    gpiod::PD9<Analog> => (ADC5, 13),
    gpiod::PD10<Analog> => (ADC3, 7),
    gpiod::PD10<Analog> => (ADC4, 7),
    gpiod::PD10<Analog> => (ADC5, 7),
    gpiod::PD11<Analog> => (ADC3, 8),
    gpiod::PD11<Analog> => (ADC4, 8),
    gpiod::PD11<Analog> => (ADC5, 8),
    gpiod::PD12<Analog> => (ADC3, 9),
    gpiod::PD12<Analog> => (ADC4, 9),
    gpiod::PD12<Analog> => (ADC5, 9),
    gpiod::PD13<Analog> => (ADC3, 10),
    gpiod::PD13<Analog> => (ADC4, 10),
    gpiod::PD13<Analog> => (ADC5, 10),
    gpiod::PD14<Analog> => (ADC3, 11),
    gpiod::PD14<Analog> => (ADC4, 11),
    gpiod::PD14<Analog> => (ADC5, 11),

    gpiof::PF0<Analog> => (ADC1, 10),
    gpiof::PF1<Analog> => (ADC2, 10),

    Temperature => (ADC1, 16),
    Vbat => (ADC1, 17),
    Vbat => (ADC3, 17),
    Vbat => (ADC5, 17),
    Vref => (ADC1, 18),
    Vref => (ADC3, 18),
    Vref => (ADC4, 18),
    Vref => (ADC5, 18),
);

// See https://www.st.com/resource/en/reference_manual/rm0440-stm32g4-series-advanced-armbased-32bit-mcus-stmicroelectronics.pdf#page=782
adc_opamp!(
    // TODO: Add all opamp types: OpenLoop, Follower(for all opamps)
    // TODO: Should we restrict type parameters A and B?
    // TODO: Also allow AD-channels shared by pins
    opamp::Opamp1 => (ADC1, 13),
    opamp::Opamp2 => (ADC2, 16),
    opamp::Opamp3 => (ADC2, 18),
);

#[cfg(any(
    feature = "stm32g473",
    feature = "stm32g474",
    feature = "stm32g483",
    feature = "stm32g484",
    feature = "stm32g491",
    feature = "stm32g4a1",
))]
adc_opamp!(
    opamp::Opamp3 => (ADC3, 13),
);

#[cfg(any(
    feature = "stm32g473",
    feature = "stm32g474",
    feature = "stm32g483",
    feature = "stm32g484",
))]
adc_opamp!(
    opamp::Opamp4 => (ADC5, 5),
    opamp::Opamp5 => (ADC5, 3),
    opamp::Opamp6 => (ADC4, 17),
);

#[cfg(any(feature = "stm32g491", feature = "stm32g4a1",))]
adc_opamp!(
    opamp::Opamp6 => (ADC3, 17),
);

#[cfg(any(feature = "stm32g491", feature = "stm32g4a1",))]
adc_pins!(
    gpioa::PA0<Analog> => (ADC1, 1),
    gpioa::PA0<Analog> => (ADC2, 1),
    gpioa::PA1<Analog> => (ADC1, 2),
    gpioa::PA1<Analog> => (ADC2, 2),
    gpioa::PA2<Analog> => (ADC1, 3),
    gpioa::PA3<Analog> => (ADC1, 4),
    gpioa::PA4<Analog> => (ADC2, 17),
    gpioa::PA5<Analog> => (ADC2, 13),
    gpioa::PA6<Analog> => (ADC2, 3),
    gpioa::PA7<Analog> => (ADC2, 4),

    gpiob::PB0<Analog> => (ADC3, 12),
    gpiob::PB0<Analog> => (ADC1, 15),
    gpiob::PB1<Analog> => (ADC3, 1),
    gpiob::PB1<Analog> => (ADC1, 12),
    gpiob::PB2<Analog> => (ADC2, 12),
    gpiob::PB11<Analog> => (ADC1, 14),
    gpiob::PB11<Analog> => (ADC2, 14),
    gpiob::PB12<Analog> => (ADC1, 11),
    gpiob::PB13<Analog> => (ADC3, 5),
    gpiob::PB14<Analog> => (ADC1, 5),
    gpiob::PB15<Analog> => (ADC2, 15),

    gpioc::PC0<Analog> => (ADC1, 6),
    gpioc::PC0<Analog> => (ADC2, 6),
    gpioc::PC1<Analog> => (ADC1, 7),
    gpioc::PC1<Analog> => (ADC2, 7),
    gpioc::PC2<Analog> => (ADC1, 8),
    gpioc::PC2<Analog> => (ADC2, 8),
    gpioc::PC3<Analog> => (ADC1, 9),
    gpioc::PC3<Analog> => (ADC2, 9),
    gpioc::PC4<Analog> => (ADC2, 5),
    gpioc::PC5<Analog> => (ADC2, 11),

    gpioe::PE7<Analog> => (ADC3, 4),
    gpioe::PE8<Analog> => (ADC3, 6),
    gpioe::PE9<Analog> => (ADC3, 2),
    gpioe::PE10<Analog> => (ADC3, 14),
    gpioe::PE11<Analog> => (ADC3, 15),
    gpioe::PE12<Analog> => (ADC3, 16),
    gpioe::PE13<Analog> => (ADC3, 3),

    gpiod::PD10<Analog> => (ADC3, 7),
    gpiod::PD11<Analog> => (ADC3, 8),
    gpiod::PD12<Analog> => (ADC3, 9),
    gpiod::PD13<Analog> => (ADC3, 10),
    gpiod::PD14<Analog> => (ADC3, 11),

    gpiof::PF0<Analog> => (ADC1, 10),
    gpiof::PF1<Analog> => (ADC2, 10),

    Temperature => (ADC1, 16),
    Vbat => (ADC1, 17),
    Vbat => (ADC3, 17),
    Vref => (ADC1, 18),
    Vref => (ADC3, 18),
);
