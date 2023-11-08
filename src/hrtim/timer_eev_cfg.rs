use core::marker::PhantomData;

pub struct EevCfgs<TIM> {
    pub eev1: EevCfg<TIM>,
    pub eev2: EevCfg<TIM>,
    pub eev3: EevCfg<TIM>,
    pub eev4: EevCfg<TIM>,
    pub eev5: EevCfg<TIM>,
    pub eev6: EevCfg<TIM>,
    pub eev7: EevCfg<TIM>,
    pub eev8: EevCfg<TIM>,
    pub eev9: EevCfg<TIM>,
    pub eev10: EevCfg<TIM>,

    // TODO: Expose these
    // TODO: Note there are some peculiarities here with fast mode
    // One way to prevent missuse would be to require a borrowed ExternalEventSource<IS_FAST> when setting
    // filter/latching as well as the event_counter related settings below.
    pub(crate) event_counter_enable_bit: bool,
    pub(crate) event_counter_reset_mode_bit: bool,
    pub(crate) event_counter_source_bits: u8,
    pub(crate) event_counter_threshold_bits: u8,
}

macro_rules! impl_setter {
    ($eevX:ident) => {
        pub fn $eevX(mut self, cfg: EevCfg<TIM>) -> Self {
            self.$eevX = cfg;
            self
        }
    };
}

impl<TIM> EevCfgs<TIM> {
    impl_setter!(eev1);
    impl_setter!(eev2);
    impl_setter!(eev3);
    impl_setter!(eev4);
    impl_setter!(eev5);
    impl_setter!(eev6);
    impl_setter!(eev7);
    impl_setter!(eev8);
    impl_setter!(eev9);
    impl_setter!(eev10);
}

impl<TIM> Clone for EevCfgs<TIM> {
    fn clone(&self) -> Self {
        Self {
            eev1: self.eev1.clone(),
            eev2: self.eev2.clone(),
            eev3: self.eev3.clone(),
            eev4: self.eev4.clone(),
            eev5: self.eev5.clone(),
            eev6: self.eev6.clone(),
            eev7: self.eev7.clone(),
            eev8: self.eev8.clone(),
            eev9: self.eev9.clone(),
            eev10: self.eev10.clone(),
            event_counter_enable_bit: self.event_counter_enable_bit.clone(),
            event_counter_reset_mode_bit: self.event_counter_reset_mode_bit.clone(),
            event_counter_source_bits: self.event_counter_source_bits.clone(),
            event_counter_threshold_bits: self.event_counter_threshold_bits.clone(),
        }
    }
}

pub struct EevCfg<TIM> {
    _x: PhantomData<TIM>,
    pub(crate) filter_bits: u8,
    pub(crate) latch_bit: bool,
}

impl<TIM> Clone for EevCfg<TIM> {
    fn clone(&self) -> Self {
        Self {
            _x: PhantomData,
            filter_bits: self.filter_bits.clone(),
            latch_bit: self.latch_bit.clone(),
        }
    }
}

impl<TIM> EevCfg<TIM> {
    /// NOTE: This can not be set if eev is in fast mode AND using `EevCfg::latching`
    pub fn filter(mut self, filter: EventFilter) -> Self {
        self.filter_bits = filter as u8;
        self
    }

    /// NOTE: This can not be set if eev is in fast mode AND using a `EevCfg::filter`
    pub fn latching(mut self) -> Self {
        self.latch_bit = true;
        self
    }
}

/// Note: Whenever a compare register is used for filtering, the value must be strictly above 0.
pub enum EventFilter {
    /// No filtering
    None = 0b0000,

    /// Blanking from reset/rollover to Cmp1
    BlankingResetToCmp1 = 0b0001,

    /// This depends on counter mode:
    /// * Up-counting mode: Blanking from reset/rollover to Cmp2
    /// * Up-down mode: Blanking from Cmp1 to Cmp2(only during up counting)
    BlankingResetToCmp2OrCmp1ToCmp2InUdm = 0b0010,

    /// Blanking from reset/rollover to Cmp3
    BlankingResetToCmp3 = 0b0011,

    /// This depends on counter mode:
    /// * Up-counting mode: Blanking from reset/rollover to Cmp4
    /// * Up-down mode: Blanking from Cmp3 to Cmp4(only during up counting)
    BlankingResetToCmp4OrCmp3ToCmp4InUdm = 0b0100,

    /// (RM 0440 table 226 'Filtering signals mapping per timer')
    BlankingSource1 = 0b0101,

    /// (RM 0440 table 226 'Filtering signals mapping per timer')
    BlankingSource2 = 0b0110,

    /// (RM 0440 table 226 'Filtering signals mapping per timer')
    BlankingSource3 = 0b0111,

    /// (RM 0440 table 226 'Filtering signals mapping per timer')
    BlankingSource4 = 0b1000,

    /// (RM 0440 table 226 'Filtering signals mapping per timer')
    BlankingSource5 = 0b1001,

    /// (RM 0440 table 226 'Filtering signals mapping per timer')
    BlankingSource6 = 0b1010,

    /// (RM 0440 table 226 'Filtering signals mapping per timer')
    BlankingSource7 = 0b1011,

    /// (RM 0440 table 226 'Filtering signals mapping per timer')
    BlankingSource8 = 0b1100,

    /// This depends on counter mode:
    /// * Up-counting mode: Windowing from reset/rollover to Cmp2
    /// * Up-down mode: Windowing from Cmp2 to Cmp3(only during up counting)
    WindowingResetToCmp2OrCmp2ToCmp3InUdm = 0b1101,

    /// This depends on counter mode:
    /// * Up-counting mode: Windowing from reset/rollover to Cmp3
    /// * Up-down mode: Windowing from Cmp2 to Cmp3(only during down counting)
    WindowingResetToCmp3OrCmp2ToCmp3InUdm = 0b1110,

    /// This depends on counter mode:
    /// * Up-counting mode: Windowing from reset/rollover to other timer `TIMWIN`'s Cmp2 event
    /// * Up-down mode: Windowing from other timer `TIMWIN`'s Cmp2 during up counting to Cmp3 during down counting
    ///
    /// `TIMWIN` (RM 0440 table 227 'Windowing signals mapping per timer'):
    ///
    /// | Destination |`TIMA`|`TIMB`|`TIMC`|`TIMD`|`TIME`|`TIMF`|
    /// |-------------|------|------|------|------|------|------|
    /// | TIMWIN      |`TIMB`|`TIMA`|`TIMD`|`TIMC`|`TIMF`|`TIME`|
    WindowingResetToOtherCmp2OrCmp2UpToCmp3DownInUdm = 0b1111,
}

impl<TIM> Default for EevCfg<TIM> {
    fn default() -> Self {
        Self {
            _x: PhantomData,
            filter_bits: EventFilter::None as u8,
            latch_bit: false,
        }
    }
}

impl<TIM> Default for EevCfgs<TIM> {
    fn default() -> Self {
        Self {
            eev1: EevCfg::default(),
            eev2: Default::default(),
            eev3: Default::default(),
            eev4: Default::default(),
            eev5: Default::default(),
            eev6: Default::default(),
            eev7: Default::default(),
            eev8: Default::default(),
            eev9: Default::default(),
            eev10: Default::default(),
            event_counter_enable_bit: false,
            event_counter_reset_mode_bit: false,
            event_counter_source_bits: 0,
            event_counter_threshold_bits: 0,
        }
    }
}
