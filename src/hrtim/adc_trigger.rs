use core::marker::PhantomData;

pub trait Adc13Trigger {
    const BITS: u32;
}

pub trait Adc24Trigger {
    const BITS: u32;
}

pub trait Adc579Trigger {
    const BITS: u32;
}

pub trait Adc6810Trigger {
    const BITS: u32;
}

pub struct TimerReset<T>(pub(crate) PhantomData<T>);
pub struct TimerPeriod<T>(pub(crate) PhantomData<T>);
