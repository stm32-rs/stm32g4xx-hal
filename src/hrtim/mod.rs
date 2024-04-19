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

use crate::hrtim::capture::HrCapt;
use crate::hrtim::compare_register::{HrCr1, HrCr2, HrCr3, HrCr4};
use crate::hrtim::fault::{FaultAction, FaultSource};
use crate::hrtim::timer::HrTim;
use crate::stm32::{
    HRTIM_COMMON, HRTIM_MASTER, HRTIM_TIMA, HRTIM_TIMB, HRTIM_TIMC, HRTIM_TIMD, HRTIM_TIME,
    HRTIM_TIMF,
};
use fugit::HertzU64;

use self::control::HrPwmControl;

use self::deadtime::DeadtimeConfig;
use self::output::{HrtimChannel, ToHrOut, CH1, CH2};
use self::timer_eev_cfg::EevCfgs;

use crate::pwm::{
    self, Alignment, ComplementaryImpossible, Pins, Polarity, Pwm, PwmPinEnable, TimerType,
};
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
    /// Automatically updates when changing period
    ///
    /// NOTE: Affects Cr1
    Dual,

    /// Triple interleaved mode
    ///
    /// Automatically force
    /// * Cr1 to 1 * PERIOD / 3 and
    /// * Cr2 to 2 * PERIOD / 3
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
    /// (not visable through `get_duty`). Automatically updates when changing period.
    ///
    /// NOTE: Must not be used simultaneously with other modes
    /// using CMP2 (dual channel dac trigger and triggered-half modes).
    Quad,
}

// HrPwmExt trait
/// Allows the pwm() method to be added to the peripheral register structs from the device crate
pub trait HrPwmExt: Sized {
    /// The requested frequency will be rounded to the nearest achievable frequency; the actual frequency may be higher or lower than requested.
    fn pwm<PINS, T, U, V>(
        self,
        _pins: PINS,
        frequency: T,
        control: &mut HrPwmControl,
        rcc: &mut Rcc,
    ) -> PINS::Channel
    where
        PINS: Pins<Self, U, V> + ToHrOut,
        T: Into<Hertz>,
        U: HrtimChannel<Pscl128>;
}

pub trait HrPwmAdvExt: Sized {
    type PreloadSource;

    fn pwm_advanced<PINS, CHANNEL, COMP>(
        self,
        _pins: PINS,
        rcc: &mut Rcc,
    ) -> HrPwmBuilder<Self, Pscl128, Self::PreloadSource, PINS::Out<Pscl128>>
    where
        PINS: Pins<Self, CHANNEL, COMP> + ToHrOut,
        CHANNEL: HrtimChannel<Pscl128>;
}

/// HrPwmBuilder is used to configure advanced HrTim PWM features
pub struct HrPwmBuilder<TIM, PSCL, PS, OUT> {
    _tim: PhantomData<TIM>,
    _prescaler: PhantomData<PSCL>,
    _out: PhantomData<OUT>,
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
    ($this:expr, $PreloadSource:ident, $TIMX:ident: (
        $timXcr:ident, $ck_psc:ident, $perXr:ident, $perx:ident, $tXcen:ident, $rep:ident, $repx:ident, $dier:ident, $repie:ident
        $(, $timXcr2:ident, $fltXr:ident, $eefXr1:ident, $eefXr2:ident, $Xeefr3:ident, $outXr:ident, $dtXr:ident)*),
    ) => {{
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
        tim.$timXcr.modify(|_r, w| unsafe {
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
                .$ck_psc().bits(prescaler_bits as u8)
        });

        $(
            tim.$timXcr2.modify(|_r, w|
                // Set counting direction
                w.udm().bit($this.counting_direction == HrCountingDirection::UpDown)
            );

            // Only available for timers with outputs(not HRTIM_MASTER)
            let _ = tim.$outXr;
            tim.$timXcr.modify(|_r, w|
                // Push-Pull mode
                w.pshpll().bit($this.enable_push_pull)
            );
        )*

        // Write period
        tim.$perXr.write(|w| unsafe { w.$perx().bits(period as u16) });

        // Enable fault sources and lock configuration
        $(unsafe {
            // Enable fault sources
            let fault_enable_bits = $this.fault_enable_bits as u32;
            tim.$fltXr.write(|w| w
                .flt1en().bit(fault_enable_bits & (1 << 0) != 0)
                .flt2en().bit(fault_enable_bits & (1 << 1) != 0)
                .flt3en().bit(fault_enable_bits & (1 << 2) != 0)
                .flt4en().bit(fault_enable_bits & (1 << 3) != 0)
                .flt5en().bit(fault_enable_bits & (1 << 4) != 0)
                .flt6en().bit(fault_enable_bits & (1 << 5) != 0)
            );

            // ... and lock configuration
            tim.$fltXr.modify(|_r, w| w.fltlck().set_bit());

            tim.$outXr.modify(|_r, w| w
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
                tim.$dtXr.modify(|_r, w| w
                    .dtprsc().bits(prescaler as u8)
                    .dtrx().bits(deadtime_rising_value)
                    .sdtrx().bit(deadtime_rising_sign)
                    .dtfx().bits(deadtime_falling_value)
                    .sdtfx().bit(deadtime_falling_sign)

                    // Lock configuration
                    .dtflkx().set_bit()
                    .dtfslkx().set_bit()
                    .dtrlkx().set_bit()
                    .dtrslkx().set_bit()
                );
                tim.$outXr.modify(|_r, w| w.dten().set_bit());
            }

            // External event configs
            let eev_cfg = $this.eev_cfg.clone();
            tim.$eefXr1.write(|w| w
                .ee1ltch().bit(eev_cfg.eev1.latch_bit).ee1fltr().bits(eev_cfg.eev1.filter_bits)
                .ee2ltch().bit(eev_cfg.eev2.latch_bit).ee2fltr().bits(eev_cfg.eev2.filter_bits)
                .ee3ltch().bit(eev_cfg.eev3.latch_bit).ee3fltr().bits(eev_cfg.eev3.filter_bits)
                .ee4ltch().bit(eev_cfg.eev4.latch_bit).ee4fltr().bits(eev_cfg.eev4.filter_bits)
                .ee5ltch().bit(eev_cfg.eev5.latch_bit).ee5fltr().bits(eev_cfg.eev5.filter_bits)
            );
            tim.$eefXr2.write(|w| w
                .ee6ltch().bit(eev_cfg.eev6.latch_bit).ee6fltr().bits(eev_cfg.eev6.filter_bits)
                .ee7ltch().bit(eev_cfg.eev7.latch_bit).ee7fltr().bits(eev_cfg.eev7.filter_bits)
                .ee8ltch().bit(eev_cfg.eev8.latch_bit).ee8fltr().bits(eev_cfg.eev8.filter_bits)
                .ee9ltch().bit(eev_cfg.eev9.latch_bit).ee9fltr().bits(eev_cfg.eev9.filter_bits)
                .ee10ltch().bit(eev_cfg.eev10.latch_bit).ee10fltr().bits(eev_cfg.eev10.filter_bits)
            );
            tim.$Xeefr3.write(|w| w
                .eevace().bit(eev_cfg.event_counter_enable_bit)
                // External Event A Counter Reset"]
                //.eevacres().bit()
                .eevarstm().bit(eev_cfg.event_counter_reset_mode_bit)
                .eevasel().bits(eev_cfg.event_counter_source_bits)
                .eevacnt().bits(eev_cfg.event_counter_threshold_bits)
            );
        })*


        hrtim_finalize_body!($PreloadSource, $this, tim, $timXcr);

        // Set repetition counter
        unsafe { tim.$rep.write(|w| w.$repx().bits($this.repetition_counter)); }

        // Enable interrupts
        tim.$dier.modify(|_r, w| w.$repie().bit($this.enable_repetition_interrupt));

        // Start timer
        //let master = unsafe { &*HRTIM_MASTER::ptr() };
        //master.mcr.modify(|_r, w| { w.$tXcen().set_bit() });

        unsafe {
            MaybeUninit::uninit().assume_init()
        }
    }};

    (PreloadSource, $this:expr, $tim:expr, $timXcr:ident) => {{
        match $this.preload_source {
            Some(PreloadSource::OnCounterReset) => {
                $tim.$timXcr.modify(|_r, w| w
                    .tx_rstu().set_bit()
                    .preen().set_bit()
                )
            },
            Some(PreloadSource::OnMasterTimerUpdate) => {
                $tim.$timXcr.modify(|_r, w| w
                    .mstu().set_bit()
                    .preen().set_bit()
                )
            }
            Some(PreloadSource::OnRepetitionUpdate) => {
                $tim.$timXcr.modify(|_r, w| w
                    .tx_repu().set_bit()
                    .preen().set_bit()
                )
            }
            None => ()
        }
    }};

    (MasterPreloadSource, $this:expr, $tim:expr, $timXcr:ident) => {{
        match $this.preload_source {
            Some(MasterPreloadSource::OnMasterRepetitionUpdate) => {
                $tim.$timXcr.modify(|_r, w| w
                    .mrepu().set_bit()
                    .preen().set_bit()
                )
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
        pub fn prescaler<P>(
            self,
            _prescaler: P,
        ) -> HrPwmBuilder<$TIMX, P, $PS, <OUT as ToHrOut>::Out<P>>
        where
            P: HrtimPrescaler,
        {
            let HrPwmBuilder {
                _tim,
                _prescaler: _,
                _out,
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
                _out: PhantomData,
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
    ($($TIMX:ident: ($timXcr:ident, $timXcr2:ident, $perXr:ident, $tXcen:ident, $rep:ident, $repx:ident, $dier:ident, $repie:ident,
        $fltXr:ident, $eefXr1:ident, $eefXr2:ident, $Xeefr3:ident, $outXr:ident, $dtXr:ident),)+) => {
        $(

            // Implement HrPwmExt trait for hrtimer
            impl HrPwmExt for $TIMX {
                fn pwm<PINS, T, U, V>(
                    self,
                    pins: PINS,
                    frequency: T,
                    control: &mut HrPwmControl,
                    rcc: &mut Rcc,
                ) -> PINS::Channel
                where
                    PINS: Pins<Self, U, V> + ToHrOut,
                    T: Into<Hertz>,
                    U: HrtimChannel<Pscl128>,
                {
                    let _= self.pwm_advanced(pins, rcc).frequency(frequency).finalize(control);

                    unsafe { MaybeUninit::<PINS::Channel>::uninit().assume_init() }
                }
            }

            impl HrPwmAdvExt for $TIMX {
                type PreloadSource = PreloadSource;

                fn pwm_advanced<PINS, CHANNEL, COMP>(
                    self,
                    _pins: PINS,
                    rcc: &mut Rcc,
                ) -> HrPwmBuilder<Self, Pscl128, Self::PreloadSource, PINS::Out<Pscl128>>
                where
                    PINS: Pins<Self, CHANNEL, COMP> + ToHrOut,
                    CHANNEL: HrtimChannel<Pscl128>
                {
                    // TODO: That 32x factor... Is that included below, or should we
                    // do that? Also that will likely risk overflowing u32 since
                    // 170MHz * 32 = 5.44GHz > u32::MAX.Hz()
                    let clk = HertzU64::from(HRTIM_COMMON::get_timer_frequency(&rcc.clocks)) * 32;

                    HrPwmBuilder {
                        _tim: PhantomData,
                        _prescaler: PhantomData,
                        _out: PhantomData,
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

            impl<PSCL, OUT>
                HrPwmBuilder<$TIMX, PSCL, PreloadSource, OUT>
            where
                PSCL: HrtimPrescaler,
                OUT: ToHrOut,
            {
                pub fn finalize(self, _control: &mut HrPwmControl) -> (
                    HrTim<$TIMX, PSCL,
                        HrCapt<$TIMX, PSCL, capture::Ch1, capture::NoDma>,
                        HrCapt<$TIMX, PSCL, capture::Ch2, capture::NoDma>>,
                        (
                            HrCr1<$TIMX, PSCL>,
                            HrCr2<$TIMX, PSCL>,
                            HrCr3<$TIMX, PSCL>,
                            HrCr4<$TIMX, PSCL>
                        ),
                        OUT
                    ) {

                    hrtim_finalize_body!(
                        self, PreloadSource,
                        $TIMX: ($timXcr, ck_pscx, $perXr, perx, $tXcen, $rep, $repx, $dier, $repie, $timXcr2, $fltXr, $eefXr1, $eefXr2, $Xeefr3, $outXr, $dtXr),
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

macro_rules! hrtim_hal_master {
    ($($TIMX:ident: ($timXcr:ident, $ck_psc:ident, $perXr:ident, $perx:ident, $rep:ident, $tXcen:ident, $dier:ident, $repie:ident),)+) => {$(
        impl HrPwmAdvExt for $TIMX {
            type PreloadSource = MasterPreloadSource;

            fn pwm_advanced<PINS, CHANNEL, COMP>(
                self,
                _pins: PINS,
                rcc: &mut Rcc,
            ) -> HrPwmBuilder<Self, Pscl128, Self::PreloadSource, PINS::Out<Pscl128>>
            where
                PINS: Pins<Self, CHANNEL, COMP> + ToHrOut, // TODO: figure out
                CHANNEL: HrtimChannel<Pscl128>
            {
                // TODO: That 32x factor... Is that included below, or should we
                // do that? Also that will likely risk overflowing u32 since
                // 170MHz * 32 = 5.44GHz > u32::MAX.Hz()
                let clk = HertzU64::from(HRTIM_COMMON::get_timer_frequency(&rcc.clocks)) * 32;

                HrPwmBuilder {
                    _tim: PhantomData,
                    _prescaler: PhantomData,
                    _out: PhantomData,
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

        impl<PSCL, OUT>
            HrPwmBuilder<$TIMX, PSCL, MasterPreloadSource, OUT>
        where
            PSCL: HrtimPrescaler,
            OUT: ToHrOut,
        {
            pub fn finalize(self, _control: &mut HrPwmControl) -> (
                HrTim<$TIMX, PSCL,
                    HrCapt<$TIMX, PSCL, capture::Ch1, capture::NoDma>,
                    HrCapt<$TIMX, PSCL, capture::Ch2, capture::NoDma>,
                >, (
                    HrCr1<$TIMX, PSCL>,
                    HrCr2<$TIMX, PSCL>,
                    HrCr3<$TIMX, PSCL>,
                    HrCr4<$TIMX, PSCL>
                )) {

                hrtim_finalize_body!(self, MasterPreloadSource, $TIMX: ($timXcr, $ck_psc, $perXr, $perx, $tXcen, $rep, $rep, $dier, $repie),)
            }

            hrtim_common_methods!($TIMX, MasterPreloadSource);
        }
    )*}
}

macro_rules! hrtim_pin_hal {
    ($($TIMX:ident:
        ($CH:ident, $perXr:ident, $cmpXYr:ident, $cmpYx:ident, $cmpY:ident, $tXYoen:ident, $tXYodis:ident),)+
     ) => {
        $(
            impl<PSCL, COMP, POL, NPOL> hal::PwmPin for Pwm<$TIMX, $CH<PSCL>, COMP, POL, NPOL>
                where Pwm<$TIMX, $CH<PSCL>, COMP, POL, NPOL>: PwmPinEnable {
                type Duty = u16;

                // You may not access self in the following methods!
                // See unsafe above

                fn disable(&mut self) {
                    self.ccer_disable();
                }

                fn enable(&mut self) {
                    self.ccer_enable();
                }

                fn get_duty(&self) -> Self::Duty {
                    let tim = unsafe { &*$TIMX::ptr() };

                    tim.$cmpXYr.read().$cmpYx().bits()
                }

                fn get_max_duty(&self) -> Self::Duty {
                    let tim = unsafe { &*$TIMX::ptr() };

                    let arr = tim.$perXr.read().perx().bits();

                    // One PWM cycle is ARR+1 counts long
                    // Valid PWM duty cycles are 0 to ARR+1
                    // However, if ARR is 65535 on a 16-bit timer, we can't add 1
                    // In that case, 100% duty cycle is not possible, only 65535/65536
                    if arr == Self::Duty::MAX {
                        arr
                    }
                    else {
                        arr + 1
                    }
                }

                /// Set duty cycle
                ///
                /// NOTE: Please observe limits(RM0440 "Period and compare registers min and max values"):
                /// | Prescaler | Min duty | Max duty |
                /// |-----------|----------|----------|
                /// |         1 |   0x0060 |   0xFFDF |
                /// |         2 |   0x0030 |   0xFFEF |
                /// |         4 |   0x0018 |   0xFFF7 |
                /// |         8 |   0x000C |   0xFFFB |
                /// |        16 |   0x0006 |   0xFFFD |
                /// |        32 |   0x0003 |   0xFFFD |
                /// |        64 |   0x0003 |   0xFFFD |
                /// |       128 |   0x0003 |   0xFFFD |
                ///
                /// Also, writing 0 as duty is only valid for CR1 and CR3 during a set of
                /// specific conditions(see RM0440 "Null duty cycle exception case"):
                /// – the output SET event is generated by the PERIOD event
                /// – the output RESET if generated by the compare 1 (respectively compare 3) event
                /// – the compare 1 (compare 3) event is active within the timer unit itself, and not used
                /// for other timing units
                fn set_duty(&mut self, duty: Self::Duty) {
                    let tim = unsafe { &*$TIMX::ptr() };

                    tim.$cmpXYr.write(|w| unsafe { w.$cmpYx().bits(duty) });
                }
            }

            // Enable implementation for ComplementaryImpossible
            impl<POL, NPOL, PSCL> PwmPinEnable for Pwm<$TIMX, $CH<PSCL>, ComplementaryImpossible, POL, NPOL> {
                fn ccer_enable(&mut self) {
                    // TODO: Should this part only be in Pwm::enable?
                    // Enable output Y on channel X
                    // This is a set-only register, no risk for data race
                    let common = unsafe { &*HRTIM_COMMON::ptr() };
                    common.oenr.write(|w| { w.$tXYoen().set_bit() });
                }
                fn ccer_disable(&mut self) {
                    // TODO: Should this part only be in Pwm::disable
                    // Disable output Y on channel X
                    // This is a write only register, no risk for data race
                    let common = unsafe { &*HRTIM_COMMON::ptr() };
                    common.odisr.write(|w| { w.$tXYodis().set_bit() });
                }
            }
        )+
    }
}

hrtim_hal! {
    HRTIM_TIMA: (timacr, timacr2, perar, tacen, repar, repx, timadier, repie, fltar, eefar1, eefar2, aeefr3, outar, dtar),
    HRTIM_TIMB: (timbcr, timbcr2, perbr, tbcen, repbr, repx, timbdier, repie, fltbr, eefbr1, eefbr2, beefr3, outbr, dtbr),
    HRTIM_TIMC: (timccr, timccr2, percr, tccen, repcr, repx, timcdier, repie, fltcr, eefcr1, eefcr2, ceefr3, outcr, dtcr),
    HRTIM_TIMD: (timdcr, timdcr2, perdr, tdcen, repdr, repx, timddier, repie, fltdr, eefdr1, eefdr2, deefr3, outdr, dtdr),
    HRTIM_TIME: (timecr, timecr2, perer, tecen, reper, repx, timedier, repie, flter, eefer1, eefer2, eeefr3, outer, dter),
    HRTIM_TIMF: (timfcr, timfcr2, perfr, tfcen, repfr, repx, timfdier, repie, fltfr, eeffr1, eeffr2, feefr3, outfr, dtfr),
}

hrtim_hal_master! {
    HRTIM_MASTER: (mcr, ck_psc, mper, mper, mrep, mcen, mdier, mrepie),
}

hrtim_pin_hal! {
    HRTIM_TIMA: (CH1, perar, cmp1ar, cmp1x, cmp1, ta1oen, ta1odis),
    HRTIM_TIMA: (CH2, perar, cmp3ar, cmp3x, cmp3, ta2oen, ta2odis),

    HRTIM_TIMB: (CH1, perbr, cmp1br, cmp1x, cmp1, tb1oen, tb1odis),
    HRTIM_TIMB: (CH2, perbr, cmp3br, cmp3x, cmp3, tb2oen, tb2odis),

    HRTIM_TIMC: (CH1, percr, cmp1cr, cmp1x, cmp1, tc1oen, tc1odis),
    HRTIM_TIMC: (CH2, percr, cmp3cr, cmp3x, cmp3, tc2oen, tc2odis),

    HRTIM_TIMD: (CH1, perdr, cmp1dr, cmp1x, cmp1, td1oen, td1odis),
    HRTIM_TIMD: (CH2, perdr, cmp3dr, cmp3x, cmp3, td2oen, td2odis),

    HRTIM_TIME: (CH1, perer, cmp1er, cmp1x, cmp1, te1oen, te1odis),
    HRTIM_TIME: (CH2, perer, cmp3er, cmp3x, cmp3, te2oen, te2odis),

    HRTIM_TIMF: (CH1, perfr, cmp1fr, cmp1x, cmp1, tf1oen, tf1odis),
    HRTIM_TIMF: (CH2, perfr, cmp3fr, cmp3x, cmp3, tf2oen, tf2odis),
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
