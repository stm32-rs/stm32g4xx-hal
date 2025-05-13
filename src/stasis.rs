// Originally taken from https://github.com/AdinAck/proto-hal/tree/main

use core::{
    marker::PhantomData,
    ops::{Deref, DerefMut},
};

/// A trait providing an interface to freeze stateful types.
pub trait Freeze: Sized {
    fn freeze<const N: usize>(self) -> (Frozen<Self, N>, [Entitlement<Self>; N]) {
        (
            Frozen { resource: self },
            core::array::from_fn(|_| Entitlement { _p: PhantomData }), // this may introduce overhead, will have to investigate (seems not to)
        )
    }
}

/// A struct to represent an entitlement
/// to a type frozen in a particular state.
pub struct Entitlement<Resource>
where
    Resource: Freeze,
{
    _p: PhantomData<Resource>,
}

impl<P> From<P> for Entitlement<P>
where
    P: Freeze,
{
    fn from(_: P) -> Self {
        Self { _p: PhantomData }
    }
}

/// A struct to hold stateful types where
/// the state is frozen.
pub struct Frozen<Resource, const ENTITLEMENTS: usize>
where
    Resource: Freeze,
{
    resource: Resource,
}

impl<Resource: Freeze, const ENTITLEMENTS: usize> Deref for Frozen<Resource, ENTITLEMENTS> {
    type Target = Resource;

    fn deref(&self) -> &Self::Target {
        &self.resource
    }
}

impl<Resource: Freeze, const ENTITLEMENTS: usize> DerefMut for Frozen<Resource, ENTITLEMENTS> {
    fn deref_mut(&mut self) -> &mut Self::Target {
        &mut self.resource
    }
}

impl<Resource: Freeze, const ENTITLEMENTS: usize> Frozen<Resource, ENTITLEMENTS> {
    pub fn release(self, entitlements: [Entitlement<Resource>; ENTITLEMENTS]) -> Resource {
        #[allow(clippy::drop_non_drop)]
        drop(entitlements);

        self.resource
    }
}

pub trait EntitlementLock: Sized {
    type Resource: Freeze;
}

impl<Resource: Freeze> EntitlementLock for Resource {
    type Resource = Self;
}

impl<Resource: Freeze> EntitlementLock for Entitlement<Resource> {
    type Resource = Resource;
}

impl<Resource: Freeze, const N: usize> EntitlementLock for Frozen<Resource, N> {
    type Resource = Resource;
}

/// Indicates a type-state is
/// entitled to another type-state.
///
/// # Safety
///
/// If a type implements this trait
/// erroneously, the generated
/// peripheral interfaces will be invalid.
pub unsafe trait Entitled<State> {}

/// A marker type for
/// an unsatisfied entitlement.
pub struct Unsatisfied;
