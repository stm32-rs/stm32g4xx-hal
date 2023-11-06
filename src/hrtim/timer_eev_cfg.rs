use core::marker::PhantomData;

struct EevCfgs<TIM> {
    eev1: EevCfg<TIM>,
    eev2: EevCfg<TIM>,
    eev3: EevCfg<TIM>,
    eev4: EevCfg<TIM>,
    eev5: EevCfg<TIM>,
    eev6: EevCfg<TIM>,
    eev7: EevCfg<TIM>,
    eev8: EevCfg<TIM>,
    eev9: EevCfg<TIM>,
    eev10: EevCfg<TIM>,
}

#[derive(Default)]
struct EevCfg<TIM> {
    _x: PhantomData<TIM>,
    filter_bits: u8,
    latch_bit: u8,
}

impl<TIM> EevCfg<TIM> {
    pub fn filter(mut self, stuff: u8) {
        self.filter_bits = stuff;
        self
    }

    /// NOTE: This can not be set if eev is in fast mode AND using a 
    pub fn latching(mut self) -> Self {
        self.latch_bit = true;
        self
    }
}

/// Note: Whenever a compare register is used for filtering, the value must be strictly above 0.
pub enum EventFilter {
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

    BlankingSource1 = 0b0101,
    BlankingSource2 = 0b0110,
    BlankingSource3 = 0b0111,
    BlankingSource4 = 0b1000,
    BlankingSource5 = 0b1001,
    BlankingSource6 = 0b1010,
    BlankingSource7 = 0b1011,
    BlankingSource8 = 0b1100,

    /// This depends on counter mode:
    /// * Up-counting mode: Windowing from reset/rollover to Cmp2
    /// * Up-down mode: Windowing from Cmp2 to Cmp3(only during up counting)
    WindowingResetToCmp2OrCmp2ToCmp3InUdm = 0b1101,

    /// This depends on counter mode:
    /// * Up-counting mode: Windowing from reset/rollover to Cmp3
    /// * Up-down mode: Windowing from Cmp2 to Cmp3(only during down counting)
    WindowingResetToCmp3OrCmp2ToCmp3InUdm = 0b1110,

    Windowing from another timing unit: TIMWIN source (see Table 227 for details) in upcounting mode (UDM bit reset)
In up-down counting mode (UDM bit set): windowing from compare 2 during the up-counting
phase to compare 3 during the down-counting phase.
    Windowing
}