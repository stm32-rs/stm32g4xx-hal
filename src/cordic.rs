#![deny(missing_docs)]
#![allow(private_bounds)] // part of the design philosophy

//! CORDIC access.
//!
//! Example:
//!
//! ```rust
//! #[entry]
//! fn main() -> ! {
//!     let dp = stm32::Peripherals::take().expect("cannot take peripherals");
//!     let pwr = dp.PWR.constrain().freeze();
//!     let mut rcc = dp.RCC.freeze(Config::hsi(), pwr);
//!
//!     let mut cordic = dp
//!         .CORDIC
//!         .constrain(&mut rcc)
//!         .freeze::<I1F15, I1F31, P60, SinCos>(); // 16 bit arguments, 32 bit results, 60 iterations, compute sine and cosine
//!
//!     // static operation (zero overhead)
//!
//!     cordic.start(I1F15::from_num(-0.25 /* -45 degreees */));
//!
//!     let (sin, cos) = cordic.result();
//!
//!     println!("sin: {}, cos: {}", sin.to_num::<f32>(), cos.to_num::<f32>());
//!
//!     // dynamic operation
//!
//!     let mut cordic = cordic.into_dynamic();
//!
//!     let sqrt = cordic.run::<Sqrt<N0>>(I1F15::from_num(0.25));
//!     println!("sqrt: {}", sqrt.to_num::<f32>());
//!     let magnitude = cordic.run::<Magnitude>((I1F15::from_num(0.25), I1F15::from_num(0.5)));
//!     println!("magnitude: {}", magnitude.to_num::<f32>());
//!
//!     loop {}
//! }
//! ```

use crate::{
    rcc::Rcc,
    {stm32::cordic::csr, stm32::CORDIC},
};
use core::marker::PhantomData;

/// Extension trait for constraining the Cordic peripheral.
pub trait Ext {
    /// Constrain the Cordic peripheral.
    fn constrain(self, rcc: &mut Rcc) -> CordicReset;
}

impl Ext for CORDIC {
    #[inline]
    fn constrain(self, rcc: &mut Rcc) -> CordicReset {
        rcc.rb.ahb1enr().modify(|_, w| w.cordicen().set_bit());

        // SAFETY: we assume the resource is already
        // in a reset state
        // BONUS: this line enforces that the
        // abstraction is of zero-size
        unsafe { core::mem::transmute(()) }
    }
}

/// Traits and structures related to data types.
pub mod types {
    use super::csr;
    use fixed::traits::Fixed;

    pub use fixed::types::{I1F15, I1F31};

    pub(crate) mod sealed {
        use super::Ext;

        /// Trait for tags to represent Cordic argument or result data.
        pub trait Tag {
            /// Internal fixed point representation.
            type Repr: Ext<Tag = Self>;
        }
    }

    /// q1.15 fixed point number.
    pub struct Q15;
    /// q1.31 fixed point number.
    pub struct Q31;

    impl sealed::Tag for Q15 {
        type Repr = I1F15;
    }

    impl sealed::Tag for Q31 {
        type Repr = I1F31;
    }

    /// Extension trait for fixed point types.
    pub trait Ext: Fixed {
        /// Tag representing this type.
        type Tag: sealed::Tag<Repr = Self>;

        /// Convert to bits of the register width,
        fn to_register(self) -> u32;
        /// Convert from bits of the register width,
        fn from_register(bits: u32) -> Self;
    }

    impl Ext for I1F15 {
        type Tag = Q15;

        #[inline]
        fn to_register(self) -> u32 {
            self.to_bits() as u16 as u32
        }

        #[inline]
        fn from_register(bits: u32) -> Self {
            Self::from_bits(bits as u16 as i16)
        }
    }

    impl Ext for I1F31 {
        type Tag = Q31;

        #[inline]
        fn to_register(self) -> u32 {
            self.to_bits() as u32
        }

        #[inline]
        fn from_register(bits: u32) -> Self {
            Self::from_bits(bits as i32)
        }
    }

    /// Traits and structures related to argument type-states.
    pub(crate) mod arg {
        use super::{csr, sealed::Tag};

        pub type Raw = csr::ARGSIZE;

        /// Trait for argument type-states.
        pub trait State: Tag {
            /// Raw representation of the configuration
            /// in the form of a bitfield variant.
            const RAW: Raw;

            /// Configure the resource to be represented
            /// by this type-state.
            fn set(w: csr::ARGSIZE_W<csr::CSRrs>) -> Self;
        }

        macro_rules! impls {
            ( $( ($NAME:ty, $RAW:ident) $(,)? )+ ) => {
                $(
                    impl State for $NAME {
                        const RAW: Raw = Raw::$RAW;

                        #[inline]
                        fn set(w: csr::ARGSIZE_W<csr::CSRrs>) -> Self {
                            w.variant(Self::RAW);

                            Self
                        }
                    }
                )+
            };
        }

        impls! {
            (super::Q31, Bits32),
            (super::Q15, Bits16),
        }
    }

    /// Traits and structures related to result type-states.
    pub(crate) mod res {
        use super::{csr, sealed::Tag};

        pub type Raw = csr::RESSIZE;

        /// Trait for argument type-states.
        pub trait State: Tag {
            /// Raw representation of the configuration
            /// in the form of a bitfield variant.
            const RAW: Raw;

            /// Configure the resource to be represented
            /// by this type-state.
            fn set(w: csr::RESSIZE_W<csr::CSRrs>) -> Self;
        }

        macro_rules! impls {
            ( $( ($NAME:ty, $RAW:ident) $(,)? )+ ) => {
                $(
                    impl State for $NAME {
                        const RAW: Raw = Raw::$RAW;

                        #[inline]
                        fn set(w: csr::RESSIZE_W<csr::CSRrs>) -> Self {
                            w.variant(Self::RAW);

                            Self
                        }
                    }
                )+
            };
        }

        impls! {
            (super::Q31, Bits32),
            (super::Q15, Bits16),
        }
    }
}

/// Traits and structures related to data counts (argument or result).
pub(crate) mod reg_count {
    use super::{csr, op::data_count, types, PhantomData};

    pub struct NReg<T, Count>
    where
        T: types::sealed::Tag,
        Count: data_count::Property<T>,
    {
        _t: PhantomData<T>,
        _count: PhantomData<Count>,
    }

    pub mod arg {
        use super::{csr, data_count, types, NReg, PhantomData};

        type Raw = csr::NARGS;

        pub(crate) trait State {
            fn set(w: csr::NARGS_W<csr::CSRrs>) -> Self;
        }

        impl<Arg, Count> State for NReg<Arg, Count>
        where
            Arg: types::arg::State,
            Count: data_count::Property<Arg>,
        {
            #[inline]
            fn set(w: csr::NARGS_W<csr::CSRrs>) -> Self {
                w.variant(
                    const {
                        match (Arg::RAW, Count::COUNT) {
                            (types::arg::Raw::Bits32, data_count::Count::Two) => Raw::Num2,
                            (_, _) => Raw::Num1,
                        }
                    },
                );

                Self {
                    _t: PhantomData,
                    _count: PhantomData,
                }
            }
        }
    }

    pub mod res {
        use super::{csr, data_count, types, NReg, PhantomData};

        type Raw = csr::NRES;

        pub(crate) trait State {
            fn set(w: csr::NRES_W<csr::CSRrs>) -> Self;
        }

        impl<Res, Count> State for NReg<Res, Count>
        where
            Res: types::res::State,
            Count: data_count::Property<Res>,
        {
            #[inline]
            fn set(w: csr::NRES_W<csr::CSRrs>) -> Self {
                w.variant(
                    const {
                        match (Res::RAW, Count::COUNT) {
                            (types::res::Raw::Bits32, data_count::Count::Two) => Raw::Num2,
                            (_, _) => Raw::Num1,
                        }
                    },
                );

                Self {
                    _t: PhantomData,
                    _count: PhantomData,
                }
            }
        }
    }
}

/// Traits and structures related to function scale type-states.
pub mod scale {
    use super::csr;

    pub(crate) type Raw = u8;

    /// Trait for function scale type-states.
    pub(crate) trait State {
        /// Raw representation of the configuration
        /// in the form of a bitfield variant.
        const RAW: Raw;

        /// Configure the resource to be represented
        /// by this type-state.
        fn set(w: csr::SCALE_W<csr::CSRrs>) -> Self;
    }

    /// Scale of 0.
    pub struct N0;
    /// Scale of 1.
    pub struct N1;
    /// Scale of 2.
    pub struct N2;
    /// Scale of 3.
    pub struct N3;
    /// Scale of 4.
    pub struct N4;
    /// Scale of 5.
    pub struct N5;
    /// Scale of 6.
    pub struct N6;
    /// Scale of 7.
    pub struct N7;

    macro_rules! impls {
        ( $( ($NAME:ident, $BITS:expr) $(,)? )+ ) => {
            $(
                impl State for $NAME {
                    const RAW: u8 = $BITS;

                    #[inline]
                    fn set(w: csr::SCALE_W<csr::CSRrs>) -> Self {
                        // SAFETY: all bits are valid
                        unsafe { w.bits(<Self as State>::RAW) };

                        Self
                    }
                }
            )+
        };
    }

    impls! {
        (N0, 0),
        (N1, 1),
        (N2, 2),
        (N3, 3),
        (N4, 4),
        (N5, 5),
        (N6, 6),
        (N7, 7),
    }
}

/// Traits and structures related to precision type-states.
pub mod prec {
    use super::csr;

    /// Trait for precision type-states.
    pub(crate) trait State {
        /// Bit representation of the precision.
        const BITS: u8;

        /// Configure the resource to be represented
        /// by this type-state.
        fn set(w: csr::PRECISION_W<csr::CSRrs>) -> Self;
    }

    /// 4 iterations.
    pub struct P4;
    /// 8 iterations.
    pub struct P8;
    /// 12 iterations.
    pub struct P12;
    /// 16 iterations.
    pub struct P16;
    /// 20 iterations.
    pub struct P20;
    /// 24 iterations.
    pub struct P24;
    /// 28 iterations.
    pub struct P28;
    /// 32 iterations.
    pub struct P32;
    /// 36 iterations.
    pub struct P36;
    /// 40 iterations.
    pub struct P40;
    /// 44 iterations.
    pub struct P44;
    /// 48 iterations.
    pub struct P48;
    /// 52 iterations.
    pub struct P52;
    /// 56 iterations.
    pub struct P56;
    /// 60 iterations.
    pub struct P60;

    macro_rules! impls {
        ( $( ($NAME:ident, $BITS:expr) $(,)? )+ ) => {
            $(
                impl State for $NAME {
                    const BITS: u8 = $BITS;

                    #[inline]
                    fn set(w: csr::PRECISION_W<csr::CSRrs>) -> Self {
                        // SAFETY: reliant on valid type-state
                        // implementations.
                        unsafe { w.bits(<Self as State>::BITS) };

                        Self
                    }
                }
            )+
        };
    }

    impls! {
        (P4, 1),
        (P8, 2),
        (P12, 3),
        (P16, 4),
        (P20, 5),
        (P24, 6),
        (P28, 7),
        (P32, 8),
        (P36, 9),
        (P40, 10),
        (P44, 11),
        (P48, 12),
        (P52, 13),
        (P56, 14),
        (P60, 15),
    }
}

/// Traits and structures related to function type-states.
pub(crate) mod func {
    use super::csr;

    pub type Raw = csr::FUNC;

    /// Trait for function type-states.
    pub trait State {
        /// Raw representation of the function.
        const RAW: Raw;

        /// Configure the resource to be represented
        /// by this type-state.
        fn set(w: csr::FUNC_W<csr::CSRrs>) -> Self;
    }

    pub struct Cos;
    pub struct Sin;
    pub struct ATan2;
    pub struct Magnitude;
    pub struct ATan;
    pub struct CosH;
    pub struct SinH;
    pub struct ATanH;
    pub struct Ln;
    pub struct Sqrt;

    macro_rules! impls {
        ( $( ($NAME:ident, $RAW:ident) $(,)? )+ ) => {
            $(
                impl State for $NAME {
                    const RAW: Raw = Raw::$RAW;

                    #[inline]
                    fn set(w: csr::FUNC_W<csr::CSRrs>) -> Self {
                        w.variant(Self::RAW);

                        Self
                    }
                }
            )+
        };
    }

    impls! {
        (Cos, Cosine),
        (Sin, Sine),
        (ATan2, Phase),
        (Magnitude, Modulus),
        (ATan, Arctangent),
        (CosH, HyperbolicCosine),
        (SinH, HyperbolicSine),
        (ATanH, Arctanh),
        (Ln, NaturalLogarithm),
        (Sqrt, SquareRoot),
    }
}

/// Traits and structures related to operating the Cordic.
pub mod op {
    use core::marker::PhantomData;

    use super::{func, prec, reg_count, scale, types, Cordic};

    /// Traits and structures related to the operation signature.
    pub(crate) mod signature {
        use super::types;
        use types::arg::State as _;
        use types::res::State as _;

        type WData = crate::stm32::cordic::WDATA;
        type RData = crate::stm32::cordic::RDATA;

        /// The signature is a property of the operation type-state.
        pub trait Property<T>
        where
            T: types::Ext,
        {
            /// Write arguments to the argument register.
            ///
            /// # Safety:
            /// Cordic must be configured to expect the
            /// correct number of register writes.
            unsafe fn write(self, reg: &WData)
            where
                T::Tag: types::arg::State;

            /// Read results from the result register.
            ///
            /// # Safety:
            /// Cordic must be configured to expect the
            /// correct number of register reades.
            unsafe fn read(reg: &RData) -> Self
            where
                T::Tag: types::res::State;
        }

        impl<T> Property<T> for T
        where
            T: types::Ext,
        {
            #[inline]
            unsafe fn write(self, reg: &WData)
            where
                T::Tag: types::arg::State,
            {
                let data = match const { T::Tag::RAW } {
                    types::arg::Raw::Bits16 => {
                        // $RM0440 17.4.2
                        // since we are only using the lower half of the register,
                        // the Cordic **will** read the upper half if the function
                        // accepts two arguments, so we fill it with +1 as per the
                        // stated default.
                        self.to_register() | (0x7fff << 16)
                    }
                    types::arg::Raw::Bits32 => self.to_register(),
                };

                // SAFETY: all bits are valid
                reg.write(|w| unsafe { w.arg().bits(data) });
            }

            #[inline]
            unsafe fn read(reg: &RData) -> Self
            where
                T::Tag: types::res::State,
            {
                T::from_register(reg.read().res().bits())
            }
        }

        impl<T> Property<T> for (T, T)
        where
            T: types::Ext,
        {
            #[inline]
            unsafe fn write(self, reg: &WData)
            where
                T::Tag: types::arg::State,
            {
                let (primary, secondary) = self;

                match const { T::Tag::RAW } {
                    types::arg::Raw::Bits16 => {
                        // $RM0440 17.4.2
                        // SAFETY: all bits are valid
                        reg.write(|w| unsafe {
                            w.arg()
                                .bits(primary.to_register() | (secondary.to_register() << 16))
                        });
                    }
                    types::arg::Raw::Bits32 => {
                        // SAFETY: all bits are valid
                        reg.write(|w| unsafe { w.arg().bits(primary.to_register()) });
                        // SAFETY: all bits are valid
                        reg.write(|w| unsafe { w.arg().bits(secondary.to_register()) });
                    }
                };
            }

            #[inline]
            unsafe fn read(reg: &RData) -> Self
            where
                T::Tag: types::res::State,
            {
                match const { T::Tag::RAW } {
                    types::res::Raw::Bits16 => {
                        let data = reg.read().res().bits();

                        // $RM0440 17.4.3
                        (
                            T::from_register(data & 0xffff),
                            T::from_register(data >> 16),
                        )
                    }
                    types::res::Raw::Bits32 => (
                        T::from_register(reg.read().res().bits()),
                        T::from_register(reg.read().res().bits()),
                    ),
                }
            }
        }
    }

    /// For internal use. A means of indirectly specifying a signature property
    /// based solely on the number of elements.
    pub(crate) mod data_count {
        use super::types;

        pub enum Count {
            One,
            Two,
        }

        pub struct One;
        pub struct Two;

        pub trait Property<T>
        where
            T: types::sealed::Tag,
        {
            type Signature: super::signature::Property<T::Repr>;

            const COUNT: Count;
        }

        impl<T> Property<T> for One
        where
            T: types::sealed::Tag,
        {
            type Signature = T::Repr;

            const COUNT: Count = Count::One;
        }

        impl<T> Property<T> for Two
        where
            T: types::sealed::Tag,
        {
            type Signature = (T::Repr, T::Repr);

            const COUNT: Count = Count::Two;
        }
    }

    pub(crate) mod sealed {
        use super::types;

        /// An operation is a feature of the Cordic.
        ///
        /// Operations are permutations of:
        /// - nargs
        /// - nres
        /// - scale
        /// - func
        pub trait Feature {
            /// The required argument register writes.
            type NArgs<Arg>
            where
                Arg: types::arg::State + types::sealed::Tag;
            /// The required result register reads.
            type NRes<Res>
            where
                Res: types::res::State + types::sealed::Tag;
            /// The scale to be applied.
            type Scale;
            /// The function to evaluate.
            type Func;

            /// The number of arguments required.
            type ArgCount;
            /// The number of results produced.
            type ResCount;
        }
    }

    /// An operation of the Cordic.
    ///
    /// Enables writing and reading values
    /// to and from the Cordic.
    #[allow(unused)]
    struct Operation<'a, Arg, Res, Op>
    where
        Arg: types::arg::State,
        Res: types::res::State,
        Op: sealed::Feature,
    {
        nargs: &'a Op::NArgs<Arg>,
        nres: &'a Op::NRes<Res>,
        scale: &'a Op::Scale,
        func: &'a Op::Func,
    }

    impl<Arg, Res, Op> Operation<'_, Arg, Res, Op>
    where
        Arg: types::arg::State,
        Res: types::res::State,
        Op: sealed::Feature,
    {
        /// Write arguments to the argument register.
        #[inline]
        fn write<Args>(&mut self, args: Args, reg: &crate::stm32::cordic::WDATA)
        where
            Arg: types::sealed::Tag,
            Args: signature::Property<Arg::Repr>,
            Op::ArgCount: data_count::Property<Arg, Signature = Args>,
        {
            // SAFETY: Cordic is necessarily configured properly if
            // an instance of `Operation` exists.
            unsafe {
                signature::Property::<Arg::Repr>::write(args, reg);
            }
        }

        /// Read results from the result register.
        #[inline]
        fn read(
            &mut self,
            reg: &crate::stm32::cordic::RDATA,
        ) -> <Op::ResCount as data_count::Property<Res>>::Signature
        where
            Op::ResCount: data_count::Property<Res>,
        {
            // SAFETY: Cordic is necessarily configured properly if
            // an instance of `Operation` exists.
            unsafe { signature::Property::<Res::Repr>::read(reg) }
        }
    }

    /// Cosine of an angle theta divided by pi.
    pub struct Cos;
    /// Sine of an angle theta divided by pi.
    pub struct Sin;
    /// Sine (primary) and cosine (secondary) of an angle theta divided by pi.
    pub struct SinCos;
    /// Modulus (secondary) multiplied by cosine of an angle theta divided by pi (primary).
    pub struct CosM;
    /// Modulus (secondary) multiplied by sine of an angle theta divided by pi (primary).
    pub struct SinM;
    /// Modulus (secondary) multiplied by sine (primary) and cosine (secondary) of an angle theta divided by pi (primary).
    pub struct SinCosM;
    /// Arctangent of x (primary) and y (secondary).
    pub struct ATan2;
    /// Magnitude of x (primary) and y (secondary).
    pub struct Magnitude;
    /// Arctangent (primary) and magnitude (secondary) of x (primary) and y (secondary).
    pub struct ATan2Magnitude;
    /// Arctangent of x.
    ///
    /// This function can be scaled by 0-7.
    pub struct ATan<Scale: scale::State> {
        _scale: PhantomData<Scale>,
    }
    /// Hyperbolic cosine of x.
    pub struct CosH;
    /// Hyperbolic sine of x.
    pub struct SinH;
    /// Hyperbolic sine (primary) and cosine (secondary) of x.
    pub struct SinHCosH;
    /// Hyperbolic arctangent of x.
    pub struct ATanH;
    /// Natural logarithm of x.
    ///
    /// This function can be scaled by 1-4.
    pub struct Ln<Scale: scale::State> {
        _scale: PhantomData<Scale>,
    }
    /// Square root of x.
    ///
    /// This function can be scaled by 0-2.
    pub struct Sqrt<Scale: scale::State> {
        _scale: PhantomData<Scale>,
    }

    macro_rules! impls {
        ( $( ($TAG:ident<$SCALE:ident>, $NARGS:ident, $NRES:ident, $FUNC:ident) $(,)?)+ ) => {
            $(
                impl sealed::Feature for $TAG
                {
                    type NArgs<Arg> = reg_count::NReg<Arg, Self::ArgCount>
                    where
                        Arg: types::arg::State + types::sealed::Tag;
                    type NRes<Res> = reg_count::NReg<Res, Self::ResCount>
                    where
                        Res: types::res::State + types::sealed::Tag;
                    type Scale = scale::$SCALE;
                    type Func = func::$FUNC;

                    type ArgCount = data_count::$NARGS;
                    type ResCount = data_count::$NRES;
                }
            )+
        };
    }

    macro_rules! impls_multi_scale {
        ( $( ($TAG:ident<$($SCALE:ident $(,)?)+>, $NARGS:ident, $NRES:ident, $FUNC:ident) $(,)?)+ ) => {
            $(
                $(
                    impl sealed::Feature for $TAG<scale::$SCALE> {
                        type NArgs<Arg> = reg_count::NReg<Arg, Self::ArgCount>
                        where
                            Arg: types::arg::State + types::sealed::Tag;
                        type NRes<Res> = reg_count::NReg<Res, Self::ResCount>
                        where
                            Res: types::res::State + types::sealed::Tag;
                        type Scale = scale::$SCALE;
                        type Func = func::$FUNC;

                        type ArgCount = data_count::$NARGS;
                        type ResCount = data_count::$NRES;
                    }
                )+
            )+
        };
    }

    impls! {
        (Cos<N0>, One, One, Cos),
        (Sin<N0>, One, One, Sin),
        (SinCos<N0>, One, Two, Sin),
        (CosM<N0>, Two, One, Cos),
        (SinM<N0>, Two, One, Sin),
        (SinCosM<N0>, Two, Two, Sin),
        (ATan2<N0>, Two, One, ATan2),
        (Magnitude<N0>, Two, One, Magnitude),
        (ATan2Magnitude<N0>, Two, Two, ATan2),
        (CosH<N1>, One, One, CosH),
        (SinH<N1>, One, One, SinH),
        (SinHCosH<N1>, One, Two, SinH),
        (ATanH<N1>, One, One, ATanH),
    }

    impls_multi_scale! {
        (ATan<N0, N1, N2, N3, N4, N5, N6, N7>, One, One, ATan),
        (Ln<N1, N2, N3, N4>, One, One, Ln),
        (Sqrt<N0, N1, N2>, One, One, Sqrt),
    }

    impl<Arg, Res, Prec, Op> Cordic<Arg, Res, Prec, Op>
    where
        Arg: types::arg::State,
        Res: types::res::State,
        Prec: prec::State,
        Op: sealed::Feature,
    {
        /// Start the configured operation.
        #[inline]
        pub fn start(&mut self, args: <Op::ArgCount as data_count::Property<Arg>>::Signature)
        where
            Op::ArgCount: data_count::Property<Arg>,
        {
            let config = &self.config;
            let mut op = Operation::<Arg, Res, Op> {
                nargs: &config.nargs,
                nres: &config.nres,
                scale: &config.scale,
                func: &config.func,
            };

            op.write(args, self.rb.wdata());
        }

        /// Get the result of an operation.
        #[inline]
        pub fn result(&mut self) -> <Op::ResCount as data_count::Property<Res>>::Signature
        where
            Op::ResCount: data_count::Property<Res>,
        {
            let config = &self.config;
            let mut op = Operation::<Arg, Res, Op> {
                nargs: &config.nargs,
                nres: &config.nres,
                scale: &config.scale,
                func: &config.func,
            };

            op.read(self.rb.rdata())
        }
    }

    /// Traits and structures for dynamic function operation.
    pub mod dynamic {
        use super::{
            super::{prec, reg_count, Cordic},
            data_count, func, scale,
            sealed::Feature,
            types, Operation,
        };

        /// Any operation can be invoked with this type-state.
        pub struct Any;

        impl Feature for Any {
            type NArgs<Arg>
                = ()
            where
                Arg: types::arg::State + types::sealed::Tag;
            type NRes<Res>
                = ()
            where
                Res: types::res::State + types::sealed::Tag;
            type Scale = ();
            type Func = ();

            type ArgCount = ();
            type ResCount = ();
        }

        /// A Cordic in dynamic mode.
        pub trait Mode<Arg, Res>
        where
            Arg: types::arg::State,
            Res: types::res::State,
        {
            /// Run an operation with provided arguments and get the result.
            ///
            /// *Note: This employs the polling strategy.
            /// For less overhead, use static operations.*
            fn run<Op>(
                &mut self,
                args: <Op::ArgCount as data_count::Property<Arg>>::Signature,
            ) -> <Op::ResCount as data_count::Property<Res>>::Signature
            where
                Op: Feature,
                Op::NArgs<Arg>: reg_count::arg::State,
                Op::NRes<Res>: reg_count::res::State,
                Op::Scale: scale::State,
                Op::Func: func::State,
                Op::ArgCount: data_count::Property<Arg>,
                Op::ResCount: data_count::Property<Res>;
        }

        impl<Arg, Res, Prec> Mode<Arg, Res> for Cordic<Arg, Res, Prec, Any>
        where
            Arg: types::arg::State,
            Res: types::res::State,
            Prec: prec::State,
        {
            #[inline]
            fn run<Op>(
                &mut self,
                args: <Op::ArgCount as data_count::Property<Arg>>::Signature,
            ) -> <Op::ResCount as data_count::Property<Res>>::Signature
            where
                Op: Feature,
                Op::NArgs<Arg>: reg_count::arg::State,
                Op::NRes<Res>: reg_count::res::State,
                Op::Scale: scale::State,
                Op::Func: func::State,
                Op::ArgCount: data_count::Property<Arg>,
                Op::ResCount: data_count::Property<Res>,
            {
                use func::State as _;
                use reg_count::{arg::State as _, res::State as _};
                use scale::State as _;

                let (nargs, nres, scale, func) = self.rb.csr().from_modify(|_, w| {
                    (
                        Op::NArgs::set(w.nargs()),
                        Op::NRes::set(w.nres()),
                        Op::Scale::set(w.scale()),
                        Op::Func::set(w.func()),
                    )
                });

                let mut op = Operation::<Arg, Res, Op> {
                    nargs: &nargs,
                    nres: &nres,
                    scale: &scale,
                    func: &func,
                };

                op.write(args, self.rb.wdata());
                self.when_ready(|cordic| op.read(cordic.rb.rdata()))
            }
        }
    }
}

/// Configuration for the Cordic.
struct Config<Arg, Res, NArgs, NRes, Scale, Prec, Func> {
    arg: Arg,
    res: Res,
    nargs: NArgs,
    nres: NRes,
    scale: Scale,
    prec: Prec,
    func: Func,
}

/// Cordic co-processor interface.
pub struct Cordic<Arg, Res, Prec, Op>
where
    Arg: types::arg::State,
    Res: types::res::State,
    Prec: prec::State,
    Op: op::sealed::Feature,
{
    rb: CORDIC,
    #[allow(clippy::type_complexity)]
    config: Config<Arg, Res, Op::NArgs<Arg>, Op::NRes<Res>, Op::Scale, Prec, Op::Func>,
}

// root impl
impl<Arg, Res, Prec, Op> Cordic<Arg, Res, Prec, Op>
where
    Arg: types::arg::State,
    Res: types::res::State,
    Prec: prec::State,
    Op: op::sealed::Feature,
{
    /// Configure the resource as dictated by the resulting
    /// type-states. The produced binding represents
    /// a frozen configuration, since it is represented
    /// by types. A new binding will need to be made --
    /// and the old binding invalidated -- in order to change
    /// the configuration.
    ///
    /// *Note: The configuration is inferred from context because
    /// it is represented by generic type-states.*
    #[inline]
    pub fn freeze<NewArg, NewRes, NewPrec, NewOp>(self) -> Cordic<NewArg, NewRes, NewPrec, NewOp>
    where
        NewArg: types::arg::State,
        NewRes: types::res::State,
        NewPrec: prec::State,
        NewOp: op::sealed::Feature,
        NewOp::NArgs<NewArg>: reg_count::arg::State,
        NewOp::NRes<NewRes>: reg_count::res::State,
        NewOp::Scale: scale::State,
        NewOp::Func: func::State,
    {
        use func::State as _;
        use reg_count::arg::State as _;
        use reg_count::res::State as _;
        use scale::State as _;

        let config = self.rb.csr().from_modify(|_, w| Config {
            arg: NewArg::set(w.argsize()),
            res: NewRes::set(w.ressize()),
            nargs: NewOp::NArgs::set(w.nargs()),
            nres: NewOp::NRes::set(w.nres()),
            scale: NewOp::Scale::set(w.scale()),
            prec: NewPrec::set(w.precision()),
            func: NewOp::Func::set(w.func()),
        });

        Cordic {
            rb: self.rb,
            config,
        }
    }

    /// Determine whether a result is pending or not.
    #[inline]
    pub fn is_ready(&self) -> bool {
        self.rb.csr().read().rrdy().bit_is_set()
    }

    /// Dispatch an operation once a result is
    /// pending.
    ///
    /// *Note: This employs the polling strategy.
    /// For less overhead, reading the result
    /// with `result()` will lock the core
    /// until a result is ready.*
    #[inline]
    pub fn when_ready<F, T>(&mut self, f: F) -> T
    where
        F: FnOnce(&mut Self) -> T,
    {
        while !self.is_ready() {}

        f(self)
    }

    /// Convert into a Cordic interface that supports
    /// runtime function selection.
    #[inline]
    pub fn into_dynamic(self) -> Cordic<Arg, Res, Prec, op::dynamic::Any> {
        Cordic {
            rb: self.rb,
            config: Config {
                arg: self.config.arg,
                res: self.config.res,
                nargs: (),
                nres: (),
                scale: (),
                prec: self.config.prec,
                func: (),
            },
        }
    }
}

/// $RM0440 17.4.1
pub type CordicReset = Cordic<types::Q31, types::Q31, prec::P20, op::Cos>;

impl<Arg, Res, Prec, Op> Cordic<Arg, Res, Prec, Op>
where
    Arg: types::arg::State,
    Res: types::res::State,
    Prec: prec::State,
    Op: op::sealed::Feature,
{
    #[inline]
    fn into_reset(self) -> CordicReset {
        self.freeze()
    }
}

// listen
impl<Arg, Res, Prec, Op> Cordic<Arg, Res, Prec, Op>
where
    Arg: types::arg::State,
    Res: types::res::State,
    Prec: prec::State,
    Op: op::sealed::Feature,
{
    /// Enable the result ready interrupt.
    #[inline]
    pub fn listen(&mut self) {
        self.rb.csr().modify(|_, w| w.ien().set_bit());
    }

    /// Disable the result ready interrupt.
    #[inline]
    pub fn unlisten(&mut self) {
        self.rb.csr().modify(|_, w| w.ien().clear_bit());
    }
}

// release
impl<Arg, Res, Prec, Op> Cordic<Arg, Res, Prec, Op>
where
    Arg: types::arg::State,
    Res: types::res::State,
    Prec: prec::State,
    Op: op::sealed::Feature,
{
    /// Release the Cordic resource binding as a noop.
    ///
    /// # Safety
    ///
    /// The Cordic peripheral is not reset.
    #[inline]
    pub unsafe fn release(self) -> CORDIC {
        self.rb
    }

    /// Release the Cordic resource binding after reset.
    #[inline]
    pub fn release_and_reset(self, rcc: &mut Rcc) -> CORDIC {
        let reset = self.into_reset();

        rcc.rb.ahb1enr().modify(|_, w| w.cordicen().clear_bit());

        // SAFETY: the resource has been reset
        unsafe { reset.release() }
    }
}
