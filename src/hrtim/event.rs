/// Event that can be used to set/reset an output
pub trait EventSource<DST, PSCL> {
    const BITS: u32;
}

/// Done:
/// * [x] Eev1-10
/// * [x] Master period
/// * [x] Master CMP1-4
/// * [x] Cmp2, Cmp4
/// * [x] Timer Update
/// * [ ] Neighbor timers compare events
/// Event that can be used reset the timer counter
pub trait TimerResetEventSource<DST, PSCL> {
    const BITS: u32;
}
