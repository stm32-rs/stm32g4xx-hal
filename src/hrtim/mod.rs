pub mod adc_trigger;
pub mod capture;
pub mod compare_register;
pub mod control;
pub mod deadtime;
pub mod event;
pub mod external_event;
pub mod fault;
pub mod output;
pub mod timer;
pub mod timer_eev_cfg;

use core::marker::PhantomData;
use core::mem::MaybeUninit;

use crate::hrtim::compare_register::{HrCr1, HrCr2, HrCr3, HrCr4};
use crate::hrtim::fault::{FaultAction, FaultSource};
use crate::hrtim::timer::HrTim;
use crate::stm32::{
    HRTIM_COMMON, HRTIM_MASTER, HRTIM_TIMA, HRTIM_TIMB, HRTIM_TIMC, HRTIM_TIMD, HRTIM_TIME,
    HRTIM_TIMF,
};
use capture::{HrCaptCh1, HrCaptCh2};
use fugit::HertzU64;

use self::control::HrPwmControl;

use self::deadtime::DeadtimeConfig;
use self::output::ToHrOut;
use self::timer_eev_cfg::EevCfgs;

use crate::pwm::{self, Alignment, Polarity, TimerType};
use crate::rcc::{GetBusFreq, Rcc};
use crate::time::Hertz;

/// Internal enum that keeps track of the count settings before PWM is finalized
enum CountSettings {
    Frequency(Hertz),
    Period(u16),
}

#[derive(Copy, Clone, PartialEq, Debug)]
pub enum HrTimerMode {
    SingleShotNonRetriggerable,
    SingleShotRetriggerable,
    Continuous,
}

#[derive(Copy, Clone, PartialEq, Debug)]
pub enum HrCountingDirection {
    /// Asymetrical up counting mode
    ///
    ///

    ///                   *                  *
    ///  Counting up   *  |               *  |
    ///             *                  *
    ///          *        |         *        |
    ///       *                  *           
    ///    *              |   *              |
    /// *                  *
    /// --------------------------------------
    ///
    /// ```txt
    /// |         *-------*                  *------
    ///           |       |                  |
    /// |         |       |                  |
    ///           |       |                  |
    /// ----------*       *------------------*
    /// ```
    ///
    /// This is the most common mode with least amount of quirks
    Up,

    /// Symmetrical up-down counting mode
    ///
    ///
    /// ```txt
    /// Period-->                  *                      Counting     *
    ///           Counting up   *  |  *     Counting        Up      *  |
    ///                      *           *     down              *
    ///                   *        |        *                 *        |
    ///                *                       *           *
    ///             *              |              *     *              |
    /// 0     -->*                                   *                  
    /// ---------------------------------------------------------------------------
    ///          |         *---------------*         |         *---------------*
    ///                    |       |       |                   |       |       |
    ///          |         |               |         |         |               |
    ///                    |       |       |                   |       |       |
    ///          ----------*               *-------------------*               *---
    /// ```
    ///
    /// NOTE: This is incompatible with
    /// * Auto-delay
    /// * Balanded Idle
    /// * Triggered-half mode
    ///
    /// There is also differences in (including but not limited to) the following areas:
    /// * Counter roll over event
    /// * The events registered with `enable_set_event` will work as normal wen counting up, however when counting down, they will work as rst events.
    /// * The events registered with `enable_rst_event` will work as normal wen counting up, however when counting down, they will work as set events.
    UpDown,
}

// Needed to calculate frequency
impl From<HrCountingDirection> for pwm::Alignment {
    fn from(val: HrCountingDirection) -> Self {
        match val {
            HrCountingDirection::Up => pwm::Alignment::Left,
            HrCountingDirection::UpDown => pwm::Alignment::Center,
        }
    }
}

#[derive(Copy, Clone, PartialEq, Debug)]
pub enum InterleavedMode {
    Disabled,

    /// Dual interleaved or Half mode
    ///
    /// Automatically force
    /// * Cr1 to PERIOD / 2 (not visable through `get_duty`).
    ///
    /// Automatically updates when changing period
    ///
    /// NOTE: Affects Cr1
    Dual,

    /// Triple interleaved mode
    ///
    /// Automatically force
    /// * Cr1 to 1 * PERIOD / 3 and
    /// * Cr2 to 2 * PERIOD / 3
    ///
    /// (not visable through `get_duty`). Automatically updates when changing period.
    ///
    /// NOTE: Must not be used simultaneously with other modes
    /// using CMP2 (dual channel dac trigger and triggered-half modes).
    Triple,

    /// Quad interleaved mode
    ///
    /// Automatically force
    /// * Cr1 to 1 * PERIOD / 4,
    /// * Cr2 to 2 * PERIOD / 4 and
    /// * Cr3 to 3 * PERIOD / 4
    ///
    /// (not visable through `get_duty`). Automatically updates when changing period.
    ///
    /// NOTE: Must not be used simultaneously with other modes
    /// using CMP2 (dual channel dac trigger and triggered-half modes).
    Quad,
}

pub trait HrPwmAdvExt: Sized {
    type PreloadSource;

    fn pwm_advanced<PINS>(
        self,
        _pins: PINS,
        rcc: &mut Rcc,
    ) -> HrPwmBuilder<Self, Pscl128, Self::PreloadSource, PINS>
    where
        PINS: ToHrOut<Self>;
}

/// HrPwmBuilder is used to configure advanced HrTim PWM features
pub struct HrPwmBuilder<TIM, PSCL, PS, PINS> {
    _tim: PhantomData<TIM>,
    _prescaler: PhantomData<PSCL>,
    pins: PINS,
    timer_mode: HrTimerMode,
    counting_direction: HrCountingDirection,
    base_freq: HertzU64,
    count: CountSettings,
    preload_source: Option<PS>,
    fault_enable_bits: u8,
    fault1_bits: u8,
    fault2_bits: u8,
    enable_push_pull: bool,
    interleaved_mode: InterleavedMode, // Also includes half mode
    repetition_counter: u8,
    deadtime: Option<DeadtimeConfig>,
    enable_repetition_interrupt: bool,
    eev_cfg: EevCfgs<TIM>,
    out1_polarity: Polarity,
    out2_polarity: Polarity,
}

pub struct HrParts<TIM, PSCL, OUT> {
    pub timer: HrTim<TIM, PSCL, HrCaptCh1<TIM, PSCL>, HrCaptCh2<TIM, PSCL>>,

    pub cr1: HrCr1<TIM, PSCL>,
    pub cr2: HrCr2<TIM, PSCL>,
    pub cr3: HrCr3<TIM, PSCL>,
    pub cr4: HrCr4<TIM, PSCL>,

    pub out: OUT,
    pub dma_channel: timer::DmaChannel<TIM>,
}

pub enum PreloadSource {
    /// Preloaded registers are updated on counter roll over or counter reset
    OnCounterReset,

    /// Preloaded registers are updated by master timer update
    OnMasterTimerUpdate,

    /// Prealoaded registers are updaten when the counter rolls over and the repetition counter is 0
    OnRepetitionUpdate,
}

pub enum MasterPreloadSource {
    /// Prealoaded registers are updaten when the master counter rolls over and the master repetition counter is 0
    OnMasterRepetitionUpdate,
}

macro_rules! hrtim_finalize_body {
    ($this:expr, $PreloadSource:ident, $TIMX:ident, $($out:ident)*) => {{
        let tim = unsafe { &*$TIMX::ptr() };
        let (period, prescaler_bits) = match $this.count {
            CountSettings::Period(period) => (period as u32, PSCL::BITS as u16),
            CountSettings::Frequency( freq ) => {
                <TimerHrTim<PSCL>>::calculate_frequency($this.base_freq, freq, $this.counting_direction.into())
            },
        };

        let (half, intlvd) = match $this.interleaved_mode {
            InterleavedMode::Disabled => (false, 0b00),
            InterleavedMode::Dual => (true, 0b00),
            InterleavedMode::Triple => (false, 0b01),
            InterleavedMode::Quad => (false, 0b10),
        };

        // Write prescaler and any special modes
        tim.cr().modify(|_r, w| unsafe {
            w
                // Enable Continuous mode
                .cont().bit($this.timer_mode == HrTimerMode::Continuous)
                .retrig().bit($this.timer_mode == HrTimerMode::SingleShotRetriggerable)

                // TODO: add support for more modes

                // Interleaved mode
                .intlvd().bits(intlvd)

                // half/double interleaved mode
                .half().bit(half)

                // Set prescaler
                .ckpsc().bits(prescaler_bits as u8)
        });

        $(
            // Only available for timers with outputs(not HRTIM_MASTER)
            #[allow(unused)]
            let $out = ();

            tim.cr2().modify(|_r, w|
                // Set counting direction
                w.udm().bit($this.counting_direction == HrCountingDirection::UpDown)
            );

            tim.cr().modify(|_r, w|
                // Push-Pull mode
                w.pshpll().bit($this.enable_push_pull)
            );
        )*

        // Write period
        tim.perr().write(|w| unsafe { w.per().bits(period as u16) });

        // Enable fault sources and lock configuration
        $(unsafe {
            // Only available for timers with outputs(not HRTIM_MASTER)
            #[allow(unused)]
            let $out = ();

            // Enable fault sources
            let fault_enable_bits = $this.fault_enable_bits as u32;
            tim.fltr().write(|w| w
                .flt1en().bit(fault_enable_bits & (1 << 0) != 0)
                .flt2en().bit(fault_enable_bits & (1 << 1) != 0)
                .flt3en().bit(fault_enable_bits & (1 << 2) != 0)
                .flt4en().bit(fault_enable_bits & (1 << 3) != 0)
                .flt5en().bit(fault_enable_bits & (1 << 4) != 0)
                .flt6en().bit(fault_enable_bits & (1 << 5) != 0)
            );

            // ... and lock configuration
            tim.fltr().modify(|_r, w| w.fltlck().set_bit());

            tim.outr().modify(|_r, w| w
                // Set actions on fault for both outputs
                .fault1().bits($this.fault1_bits)
                .fault2().bits($this.fault2_bits)

                // Set output polarity for both outputs
                .pol1().bit($this.out1_polarity == Polarity::ActiveLow)
                .pol2().bit($this.out2_polarity == Polarity::ActiveLow)
            );
            if let Some(deadtime) = $this.deadtime {
                let DeadtimeConfig {
                    prescaler,
                    deadtime_rising_value,
                    deadtime_rising_sign,
                    deadtime_falling_value,
                    deadtime_falling_sign,
                } = deadtime;

                // SAFETY: DeadtimeConfig makes sure rising and falling values are valid
                // and DeadtimePrescaler has its own garantuee
                tim.dtr().modify(|_r, w| w
                    .dtprsc().bits(prescaler as u8)
                    .dtr().bits(deadtime_rising_value)
                    .sdtr().bit(deadtime_rising_sign)
                    .dtf().bits(deadtime_falling_value)
                    .sdtf().bit(deadtime_falling_sign)

                    // Lock configuration
                    .dtflk().set_bit()
                    .dtfslk().set_bit()
                    .dtrlk().set_bit()
                    .dtrslk().set_bit()
                );
                tim.outr().modify(|_r, w| w.dten().set_bit());
            }

            // External event configs
            let eev_cfg = $this.eev_cfg.clone();
            tim.eefr1().write(|w| w
                .ee1ltch().bit(eev_cfg.eev1.latch_bit).ee1fltr().bits(eev_cfg.eev1.filter_bits)
                .ee2ltch().bit(eev_cfg.eev2.latch_bit).ee2fltr().bits(eev_cfg.eev2.filter_bits)
                .ee3ltch().bit(eev_cfg.eev3.latch_bit).ee3fltr().bits(eev_cfg.eev3.filter_bits)
                .ee4ltch().bit(eev_cfg.eev4.latch_bit).ee4fltr().bits(eev_cfg.eev4.filter_bits)
                .ee5ltch().bit(eev_cfg.eev5.latch_bit).ee5fltr().bits(eev_cfg.eev5.filter_bits)
            );
            tim.eefr2().write(|w| w
                .ee6ltch().bit(eev_cfg.eev6.latch_bit).ee6fltr().bits(eev_cfg.eev6.filter_bits)
                .ee7ltch().bit(eev_cfg.eev7.latch_bit).ee7fltr().bits(eev_cfg.eev7.filter_bits)
                .ee8ltch().bit(eev_cfg.eev8.latch_bit).ee8fltr().bits(eev_cfg.eev8.filter_bits)
                .ee9ltch().bit(eev_cfg.eev9.latch_bit).ee9fltr().bits(eev_cfg.eev9.filter_bits)
                .ee10ltch().bit(eev_cfg.eev10.latch_bit).ee10fltr().bits(eev_cfg.eev10.filter_bits)
            );
            tim.eefr3().write(|w| w
                .eevace().bit(eev_cfg.event_counter_enable_bit)
                // External Event A Counter Reset"]
                //.eevacres().bit()
                .eevarstm().bit(eev_cfg.event_counter_reset_mode_bit)
                .eevasel().bits(eev_cfg.event_counter_source_bits)
                .eevacnt().bits(eev_cfg.event_counter_threshold_bits)
            );
        })*


        hrtim_finalize_body!($PreloadSource, $this, tim);

        // Set repetition counter
        unsafe { tim.repr().write(|w| w.rep().bits($this.repetition_counter)); }

        // Enable interrupts
        tim.dier().modify(|_r, w| w.repie().bit($this.enable_repetition_interrupt));

        // Start timer
        //let master = unsafe { &*HRTIM_MASTER::ptr() };
        //master.mcr.modify(|_r, w| { w.$tXcen().set_bit() });

        // Connect pins and let HRTIM take over control over them
        $this.pins.connect_to_hrtim();

        unsafe {
            MaybeUninit::uninit().assume_init()
        }
    }};

    (PreloadSource, $this:expr, $tim:expr) => {{
        match $this.preload_source {
            Some(PreloadSource::OnCounterReset) => {
                $tim.cr().modify(|_r, w| w
                    .trstu().set_bit()
                    .preen().set_bit()
                );
            },
            Some(PreloadSource::OnMasterTimerUpdate) => {
                $tim.cr().modify(|_r, w| w
                    .mstu().set_bit()
                    .preen().set_bit()
                );
            }
            Some(PreloadSource::OnRepetitionUpdate) => {
                $tim.cr().modify(|_r, w| w
                    .trepu().set_bit()
                    .preen().set_bit()
                );
            }
            None => ()
        }
    }};

    (MasterPreloadSource, $this:expr, $tim:expr) => {{
        match $this.preload_source {
            Some(MasterPreloadSource::OnMasterRepetitionUpdate) => {
                $tim.cr().modify(|_r, w| w
                    .mrepu().set_bit()
                    .preen().set_bit()
                );
            }
            None => ()
        }
    }};
}

macro_rules! hrtim_common_methods {
    ($TIMX:ident, $PS:ident) => {
        /// Set the PWM frequency; will overwrite the previous prescaler and period
        /// The requested frequency will be rounded to the nearest achievable frequency; the actual frequency may be higher or lower than requested.
        pub fn frequency<T: Into<Hertz>>(mut self, freq: T) -> Self {
            self.count = CountSettings::Frequency(freq.into());

            self
        }

        /// Set the prescaler; PWM count runs at base_frequency/(prescaler+1)
        pub fn prescaler<P>(self, _prescaler: P) -> HrPwmBuilder<$TIMX, P, $PS, PINS>
        where
            P: HrtimPrescaler,
        {
            let HrPwmBuilder {
                _tim,
                _prescaler: _,
                pins,
                timer_mode,
                fault_enable_bits,
                fault1_bits,
                fault2_bits,
                enable_push_pull,
                interleaved_mode,
                counting_direction,
                base_freq,
                count,
                preload_source,
                repetition_counter,
                deadtime,
                enable_repetition_interrupt,
                eev_cfg,
                out1_polarity,
                out2_polarity,
            } = self;

            let period = match count {
                CountSettings::Frequency(_) => u16::MAX,
                CountSettings::Period(period) => period,
            };

            let count = CountSettings::Period(period);

            HrPwmBuilder {
                _tim,
                _prescaler: PhantomData,
                pins,
                timer_mode,
                fault_enable_bits,
                fault1_bits,
                fault2_bits,
                enable_push_pull,
                interleaved_mode,
                counting_direction,
                base_freq,
                count,
                preload_source,
                repetition_counter,
                deadtime,
                enable_repetition_interrupt,
                eev_cfg,
                out1_polarity,
                out2_polarity,
            }
        }

        pub fn timer_mode(mut self, timer_mode: HrTimerMode) -> Self {
            self.timer_mode = timer_mode;
            self
        }

        // TODO: Allow setting multiple?
        pub fn preload(mut self, preload_source: $PS) -> Self {
            self.preload_source = Some(preload_source);
            self
        }

        /// Set the period; PWM count runs from 0 to period, repeating every (period+1) counts
        pub fn period(mut self, period: u16) -> Self {
            self.count = CountSettings::Period(period);
            self
        }

        /// Set repetition counter, useful to reduce interrupts generated
        /// from timer by a factor (repetition_counter + 1)
        pub fn repetition_counter(mut self, repetition_counter: u8) -> Self {
            self.repetition_counter = repetition_counter;
            self
        }

        pub fn enable_repetition_interrupt(mut self) -> Self {
            self.enable_repetition_interrupt = true;
            self
        }

        pub fn eev_cfg(mut self, eev_cfg: EevCfgs<$TIMX>) -> Self {
            self.eev_cfg = eev_cfg;
            self
        }
    };
}

// Implement PWM configuration for timer
macro_rules! hrtim_hal {
    ($($TIMX:ident: $($out:ident)*,)+) => {
        $(
            impl HrPwmAdvExt for $TIMX {
                type PreloadSource = PreloadSource;

                fn pwm_advanced<PINS>(
                    self,
                    pins: PINS,
                    rcc: &mut Rcc,
                ) -> HrPwmBuilder<Self, Pscl128, Self::PreloadSource, PINS>
                where
                    PINS: ToHrOut<$TIMX>,
                {
                    // TODO: That 32x factor... Is that included below, or should we
                    // do that? Also that will likely risk overflowing u32 since
                    // 170MHz * 32 = 5.44GHz > u32::MAX.Hz()
                    let clk = HertzU64::from(HRTIM_COMMON::get_timer_frequency(&rcc.clocks)) * 32;

                    HrPwmBuilder {
                        _tim: PhantomData,
                        _prescaler: PhantomData,
                        pins,
                        timer_mode: HrTimerMode::Continuous,
                        fault_enable_bits: 0b000000,
                        fault1_bits: 0b00,
                        fault2_bits: 0b00,
                        counting_direction: HrCountingDirection::Up,
                        base_freq: clk,
                        count: CountSettings::Period(u16::MAX),
                        preload_source: None,
                        enable_push_pull: false,
                        interleaved_mode: InterleavedMode::Disabled,
                        repetition_counter: 0,
                        deadtime: None,
                        enable_repetition_interrupt: false,
                        eev_cfg: EevCfgs::default(),
                        out1_polarity: Polarity::ActiveHigh,
                        out2_polarity: Polarity::ActiveHigh,
                    }
                }
            }

            impl<PSCL, PINS>
                HrPwmBuilder<$TIMX, PSCL, PreloadSource, PINS>
            where
                PSCL: HrtimPrescaler,
                PINS: ToHrOut<$TIMX>,
            {
                pub fn finalize(self, _control: &mut HrPwmControl) -> HrParts<$TIMX, PSCL, PINS::Out<PSCL>> {
                    hrtim_finalize_body!(
                        self, PreloadSource,
                        $TIMX, $($out)*
                    )
                }

                hrtim_common_methods!($TIMX, PreloadSource);

                pub fn with_fault_source<FS>(mut self, _fault_source: FS) -> Self
                    where FS: FaultSource
                {
                    self.fault_enable_bits |= FS::ENABLE_BITS;

                    self
                }

                pub fn fault_action1(mut self, fault_action1: FaultAction) -> Self {
                    self.fault1_bits = fault_action1 as _;

                    self
                }

                pub fn fault_action2(mut self, fault_action2: FaultAction) -> Self {
                    self.fault2_bits = fault_action2 as _;

                    self
                }

                pub fn out1_polarity(mut self, polarity: Polarity) -> Self {
                    self.out1_polarity = polarity;

                    self
                }

                pub fn out2_polarity(mut self, polarity: Polarity) -> Self {
                    self.out2_polarity = polarity;

                    self
                }

                /// Enable or disable Push-Pull mode
                ///
                /// Enabling Push-Pull mode will make output 1 and 2
                /// alternate every period with one being
                /// inactive and the other getting to output its wave form
                /// as normal
                ///
                ///         ----           .                ----
                ///out1    |    |          .               |    |
                ///        |    |          .               |    |
                /// --------    ----------------------------    --------------------
                ///        .                ------         .                ------
                ///out2    .               |      |        .               |      |
                ///        .               |      |        .               |      |
                /// ------------------------    ----------------------------      --
                ///
                /// NOTE: setting this will overide any 'Swap Mode' set
                pub fn push_pull_mode(mut self, enable: bool) -> Self {
                    // TODO: add check for incompatible modes
                    self.enable_push_pull = enable;

                    self
                }

                /// Set counting direction
                ///
                /// See [`HrCountingDirection`]
                pub fn counting_direction(mut self, counting_direction: HrCountingDirection) -> Self {
                    self.counting_direction = counting_direction;

                    self
                }

                /// Set interleaved or half modes
                ///
                /// NOTE: Check [`InterleavedMode`] for more info about special cases
                pub fn interleaved_mode(mut self, mode: InterleavedMode) -> Self {
                    self.interleaved_mode = mode;

                    self
                }

                pub fn deadtime(mut self, deadtime: DeadtimeConfig) -> Self {
                    self.deadtime = Some(deadtime);

                    self
                }

                //pub fn swap_mode(mut self, enable: bool) -> Self
            }
        )+
    };
}

impl HrPwmAdvExt for HRTIM_MASTER {
    type PreloadSource = MasterPreloadSource;

    fn pwm_advanced<PINS>(
        self,
        pins: PINS,
        rcc: &mut Rcc,
    ) -> HrPwmBuilder<Self, Pscl128, Self::PreloadSource, PINS>
    where
        PINS: ToHrOut<HRTIM_MASTER>,
    {
        // TODO: That 32x factor... Is that included below, or should we
        // do that? Also that will likely risk overflowing u32 since
        // 170MHz * 32 = 5.44GHz > u32::MAX.Hz()
        let clk = HertzU64::from(HRTIM_COMMON::get_timer_frequency(&rcc.clocks)) * 32;

        HrPwmBuilder {
            _tim: PhantomData,
            _prescaler: PhantomData,
            pins,
            timer_mode: HrTimerMode::Continuous,
            fault_enable_bits: 0b000000,
            fault1_bits: 0b00,
            fault2_bits: 0b00,
            counting_direction: HrCountingDirection::Up,
            base_freq: clk,
            count: CountSettings::Period(u16::MAX),
            preload_source: None,
            enable_push_pull: false,
            interleaved_mode: InterleavedMode::Disabled,
            repetition_counter: 0,
            deadtime: None,
            enable_repetition_interrupt: false,
            eev_cfg: EevCfgs::default(),
            out1_polarity: Polarity::ActiveHigh,
            out2_polarity: Polarity::ActiveHigh,
        }
    }
}

impl<PSCL, PINS> HrPwmBuilder<HRTIM_MASTER, PSCL, MasterPreloadSource, PINS>
where
    PSCL: HrtimPrescaler,
    PINS: ToHrOut<HRTIM_MASTER>,
{
    pub fn finalize(self, _control: &mut HrPwmControl) -> HrParts<HRTIM_MASTER, PSCL, ()> {
        hrtim_finalize_body!(self, MasterPreloadSource, HRTIM_MASTER,)
    }

    hrtim_common_methods!(HRTIM_MASTER, MasterPreloadSource);
}

hrtim_hal! {
    HRTIM_TIMA: out,
    HRTIM_TIMB: out,
    HRTIM_TIMC: out,
    HRTIM_TIMD: out,
    HRTIM_TIME: out,
    HRTIM_TIMF: out,
}

/// # Safety
/// Only implement for valid prescalers with correct values
pub unsafe trait HrtimPrescaler: Default {
    const BITS: u8;
    const VALUE: u8;

    /// Minimum allowed value for compare registers used with the timer with this prescaler
    ///
    /// NOTE: That for CR1 and CR3, 0 is also allowed
    const MIN_CR: u16;

    /// Maximum allowed value for compare registers used with the timer with this prescaler
    const MAX_CR: u16;
}

macro_rules! impl_pscl {
    ($($t:ident => $b:literal, $v:literal, $min:literal, $max:literal)+) => {$(
        #[derive(Copy, Clone, Default)]
        pub struct $t;
        unsafe impl HrtimPrescaler for $t {
            const BITS: u8 = $b;
            const VALUE: u8 = $v;
            const MIN_CR: u16 = $min;
            const MAX_CR: u16 = $max;
        }
    )+};
}

impl_pscl! {
    Pscl1   => 0b000,   1, 0x0060, 0xFFDF
    Pscl2   => 0b001,   2, 0x0030, 0xFFEF
    Pscl4   => 0b010,   4, 0x0018, 0xFFF7
    Pscl8   => 0b011,   8, 0x000C, 0xFFFB
    Pscl16  => 0b100,  16, 0x0006, 0xFFFD
    Pscl32  => 0b101,  32, 0x0003, 0xFFFD
    Pscl64  => 0b110,  64, 0x0003, 0xFFFD
    Pscl128 => 0b111, 128, 0x0003, 0xFFFD
}

/// HrTim timer
struct TimerHrTim<PSC>(PhantomData<PSC>);

impl<PSC: HrtimPrescaler> pwm::TimerType for TimerHrTim<PSC> {
    // Period calculator for 16-bit hrtimers
    //
    // NOTE: This function will panic if the calculated period can not fit into 16 bits
    fn calculate_frequency(base_freq: HertzU64, freq: Hertz, alignment: Alignment) -> (u32, u16) {
        let ideal_period = pwm::Timer32Bit::calculate_frequency(base_freq, freq, alignment).0 + 1;

        let prescale = u32::from(PSC::VALUE);

        // Round to the nearest period
        let period = (ideal_period + (prescale >> 1)) / prescale - 1;

        // It IS possible to fail this assert
        assert!(period <= 0xFFFF);

        (period, PSC::BITS.into())
    }
}
