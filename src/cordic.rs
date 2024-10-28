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
//!         .freeze::<I1F15, I1F31, SinCos, P60>(); // 16 bit arguments, 32 bit results, compute sine and cosine, 60 iterations
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

use crate::{rcc::Rcc, stm32::CORDIC};
use core::marker::PhantomData;

/// Extension trait for constraining the CORDIC peripheral.
pub trait Ext {
    /// Constrain the CORDIC peripheral.
    fn constrain(self, rcc: &mut Rcc) -> CordicReset;
}

impl Ext for CORDIC {
    #[inline]
    fn constrain(self, rcc: &mut Rcc) -> CordicReset {
        rcc.rb.ahb1enr.modify(|_, w| w.cordicen().set_bit());

        // SAFETY: the resource is assumed to be
        // in the "reset" configuration.
        unsafe { Cordic::wrap(self) }
    }
}

/// Traits and structures related to data types.
pub mod types {
    use fixed::traits::Fixed;

    pub use fixed::types::{I1F15, I1F31};

    /// q1.15 fixed point number.
    pub struct Q15;
    /// q1.31 fixed point number.
    pub struct Q31;

    /// Extension trait for fixed point types.
    pub trait Ext: Fixed {
        /// Type-state representing this type.
        type Repr: DataType<Inner = Self>;

        /// Convert to bits of the register width,
        fn to_register(self) -> u32;
        /// Convert from bits of the register width,
        fn from_register(bits: u32) -> Self;
    }

    impl Ext for I1F15 {
        type Repr = Q15;

        fn to_register(self) -> u32 {
            self.to_bits() as u16 as u32
        }
        fn from_register(bits: u32) -> Self {
            Self::from_bits(bits as u16 as i16)
        }
    }

    impl Ext for I1F31 {
        type Repr = Q31;

        fn to_register(self) -> u32 {
            self.to_bits() as u32
        }

        fn from_register(bits: u32) -> Self {
            Self::from_bits(bits as i32)
        }
    }

    /// Trait for newtypes to represent CORDIC argument or result data.
    pub trait DataType {
        /// Internal fixed point representation.
        type Inner: Ext<Repr = Self>;
    }

    impl DataType for Q15 {
        type Inner = I1F15;
    }

    impl DataType for Q31 {
        type Inner = I1F31;
    }

    /// Traits and structures related to argument type-states.
    pub mod arg {
        pub(crate) type Raw = crate::stm32::cordic::csr::ARGSIZE_A;

        /// Trait for argument type-states.
        pub(crate) trait State: super::DataType {
            /// Raw representation of the configuration
            /// in the form of a bitfield variant.
            const RAW: Raw;

            /// Configure the resource to be represented
            /// by this type-state.
            #[inline]
            fn set<const OFFSET: u8>(w: crate::stm32::cordic::csr::ARGSIZE_W<OFFSET>) {
                w.variant(Self::RAW);
            }
        }

        macro_rules! impls {
            ( $( ($NAME:ty, $RAW:ident) $(,)? )+ ) => {
                $(
                    impl State for $NAME {
                        const RAW: Raw = Raw::$RAW;
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
    pub mod res {
        pub(crate) type Raw = crate::stm32::cordic::csr::RESSIZE_A;

        /// Trait for argument type-states.
        pub(crate) trait State: super::DataType {
            /// Raw representation of the configuration
            /// in the form of a bitfield variant.
            const RAW: Raw;

            /// Configure the resource to be represented
            /// by this type-state.
            #[inline]
            fn set<const OFFSET: u8>(w: crate::stm32::cordic::csr::RESSIZE_W<OFFSET>) {
                w.variant(Self::RAW);
            }
        }

        macro_rules! impls {
            ( $( ($NAME:ty, $RAW:ident) $(,)? )+ ) => {
                $(
                    impl State for $NAME {
                        const RAW: Raw = Raw::$RAW;
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

/// Traits and structures related to function type-states.
pub mod func {
    use super::*;

    /// For internal use. A means of indirectly specifying a signature property
    /// based solely on the number of elements.
    mod data_count {
        use super::types;

        pub enum Count {
            One,
            Two,
        }

        /// One (primary) function argument/result.
        pub struct One;
        /// Two (primary and secondary) function arguments/results.
        pub struct Two;

        pub trait Property<T>
        where
            T: types::DataType,
        {
            type Signature: super::signature::Property<T::Inner>;

            const COUNT: Count;
        }

        impl<T> Property<T> for One
        where
            T: types::DataType,
        {
            type Signature = T::Inner;

            const COUNT: Count = Count::One;
        }

        impl<T> Property<T> for Two
        where
            T: types::DataType,
        {
            type Signature = (T::Inner, T::Inner);

            const COUNT: Count = Count::Two;
        }
    }

    /// Traits and structures related to data counts (argument or result).
    pub(crate) mod reg_count {
        use super::{super::types, data_count, PhantomData};

        pub struct NReg<T, Count>
        where
            T: types::DataType,
            Count: data_count::Property<T>,
        {
            _t: PhantomData<T>,
            _count: PhantomData<Count>,
        }

        pub mod arg {
            use super::{data_count, types, NReg};

            type Raw = crate::stm32::cordic::csr::NARGS_A;

            pub(crate) trait State<T> {
                fn set<const OFFSET: u8>(w: crate::stm32::cordic::csr::NARGS_W<OFFSET>);
            }

            impl<Arg, Count> State<Arg> for NReg<Arg, Count>
            where
                Arg: types::arg::State,
                Count: data_count::Property<Arg>,
            {
                fn set<const OFFSET: u8>(w: crate::stm32::cordic::csr::NARGS_W<OFFSET>) {
                    w.variant(
                        const {
                            match (Arg::RAW, Count::COUNT) {
                                (types::arg::Raw::Bits32, data_count::Count::Two) => Raw::Num2,
                                (_, _) => Raw::Num1,
                            }
                        },
                    );
                }
            }
        }

        pub mod res {
            use super::{data_count, types, NReg};

            type Raw = crate::stm32::cordic::csr::NRES_A;

            pub(crate) trait State<T> {
                fn set<const OFFSET: u8>(w: crate::stm32::cordic::csr::NRES_W<OFFSET>);
            }

            impl<Res, Count> State<Res> for NReg<Res, Count>
            where
                Res: types::res::State,
                Count: data_count::Property<Res>,
            {
                fn set<const OFFSET: u8>(w: crate::stm32::cordic::csr::NRES_W<OFFSET>) {
                    w.variant(
                        const {
                            match (Res::RAW, Count::COUNT) {
                                (types::res::Raw::Bits32, data_count::Count::Two) => Raw::Num2,
                                (_, _) => Raw::Num1,
                            }
                        },
                    );
                }
            }
        }
    }

    /// Traits and structures related to function scale type-states.
    pub mod scale {
        pub(crate) type Raw = u8;

        /// Trait for function scale type-states.
        pub(crate) trait State {
            /// Raw representation of the configuration
            /// in the form of a bitfield variant.
            const RAW: Raw;

            /// Configure the resource to be represented
            /// by this type-state.
            #[inline]
            fn set<const OFFSET: u8>(w: crate::stm32::cordic::csr::SCALE_W<OFFSET>) {
                w.bits(<Self as State>::RAW);
            }
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

    /// Traits and structures related to the function signature.
    pub mod signature {
        use super::{data_count, reg_count, types};
        use types::arg::State as _;
        use types::res::State as _;

        type WData = crate::stm32g4::Reg<crate::stm32::cordic::wdata::WDATA_SPEC>;
        type RData = crate::stm32g4::Reg<crate::stm32::cordic::rdata::RDATA_SPEC>;

        /// The signature is a property of the function type-state.
        pub trait Property<T>
        where
            T: types::Ext,
        {
            /// Number of register operations required.
            type NReg;

            /// Write arguments to the argument register.
            fn write(self, reg: &WData)
            where
                T::Repr: types::arg::State;

            /// Read results from the result register.
            fn read(reg: &RData) -> Self
            where
                T::Repr: types::res::State;
        }

        impl<T> Property<T> for T
        where
            T: types::Ext,
        {
            type NReg = reg_count::NReg<T::Repr, data_count::One>;

            fn write(self, reg: &WData)
            where
                T::Repr: types::arg::State,
            {
                let data = match const { T::Repr::RAW } {
                    types::arg::Raw::Bits16 => {
                        // $RM0440 17.4.2
                        // since we are only using the lower half of the register,
                        // the CORDIC **will** read the upper half if the function
                        // accepts two arguments, so we fill it with +1 as per the
                        // stated default.
                        self.to_register() | (0x7fff << 16)
                    }
                    types::arg::Raw::Bits32 => self.to_register(),
                };

                reg.write(|w| w.arg().bits(data));
            }

            fn read(reg: &RData) -> Self
            where
                T::Repr: types::res::State,
            {
                T::from_register(reg.read().res().bits())
            }
        }

        impl<T> Property<T> for (T, T)
        where
            T: types::Ext,
        {
            type NReg = reg_count::NReg<T::Repr, data_count::Two>;

            fn write(self, reg: &WData)
            where
                T::Repr: types::arg::State,
            {
                let (primary, secondary) = self;

                match const { T::Repr::RAW } {
                    types::arg::Raw::Bits16 => {
                        // $RM0440 17.4.2
                        reg.write(|w| {
                            w.arg()
                                .bits(primary.to_register() | (secondary.to_register() << 16))
                        });
                    }
                    types::arg::Raw::Bits32 => {
                        reg.write(|w| w.arg().bits(primary.to_register()));
                        reg.write(|w| w.arg().bits(secondary.to_register()));
                    }
                };
            }

            fn read(reg: &RData) -> Self
            where
                T::Repr: types::res::State,
            {
                match const { T::Repr::RAW } {
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

    pub(crate) type Raw = crate::stm32::cordic::csr::FUNC_A;

    /// Trait for function type-states.
    pub trait State {
        /// Raw representation of the function.
        const RAW: Raw;

        /// Configure the resource to be represented
        /// by this type-state.
        #[inline]
        fn set<const OFFSET: u8>(w: crate::stm32::cordic::csr::FUNC_W<OFFSET>) {
            w.variant(Self::RAW);
        }
    }

    /// Define specific combinations
    /// of states and properties for each function.
    pub trait Feature<Arg, Res>: State
    where
        Arg: types::arg::State,
        Res: types::res::State,
    {
        /// The number of arguments required.
        type Arguments: signature::Property<Arg::Inner>;
        /// The number of arguments produced.
        type Results: signature::Property<Res::Inner>;

        /// The operation to perform.
        type Op: State;
        /// The scale to be applied.
        type Scale: scale::State;
        /// The required argument register writes.
        type NArgs: reg_count::arg::State<Arg>;
        /// The required result register reads.
        type NRes: reg_count::res::State<Res>;
    }

    impl<Arg, Res, Func, Prec> Cordic<Arg, Res, Func, Prec>
    where
        Arg: types::arg::State,
        Res: types::res::State,
        Func: Feature<Arg, Res>,
        Prec: prec::State,
        Func::Arguments: signature::Property<Arg::Inner>,
        Func::Results: signature::Property<Res::Inner>,
    {
        /// Start the configured operation.
        pub fn start(&mut self, args: Func::Arguments) {
            use signature::Property as _;

            args.write(&self.rb.wdata);
        }

        /// Get the result of an operation.
        pub fn result(&mut self) -> Func::Results {
            use signature::Property as _;

            Func::Results::read(&self.rb.rdata)
        }
    }

    // function types with argument count encoded

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
        ( $( ($NAME:ident < $SCALE:ident >, $RAW:ident, $NARGS:ident, $NRES:ident, start( $($START_PARAM:ident),+ )) $(,)?)+ ) => {
            $(
                impl State for $NAME {
                    const RAW: Raw = Raw::$RAW;
                }

                impl<Arg, Res> Feature<Arg, Res> for $NAME
                where
                    Arg: types::arg::State,
                    Res: types::res::State,
                {
                    type Arguments = <data_count::$NARGS as data_count::Property<Arg>>::Signature;
                    type Results = <data_count::$NRES as data_count::Property<Res>>::Signature;

                    type Op = Self;
                    type Scale = scale::$SCALE;
                    type NArgs = <Self::Arguments as signature::Property<Arg::Inner>>::NReg;
                    type NRes = <Self::Results as signature::Property<Res::Inner>>::NReg;
                }
            )+
        };
    }

    macro_rules! impls_multi_scale {
        // root / config (almost identical to single scale)
        ( $( ($NAME:ident < $( $SCALE:ident  $(,)? )+ >, $RAW:ident, $NARGS:ident, $NRES:ident, start $START_PARAM:tt ) $(,)?)+ ) => {
            $(
                $(
                    impl State for $NAME<scale::$SCALE> {
                        const RAW: Raw = Raw::$RAW;
                    }

                    impl<Arg, Res> Feature<Arg, Res> for $NAME<scale::$SCALE>
                    where
                        Arg: types::arg::State,
                        Res: types::res::State,
                    {
                        type Arguments = <data_count::$NARGS as data_count::Property<Arg>>::Signature;
                        type Results = <data_count::$NRES as data_count::Property<Res>>::Signature;

                        type Op = Self;
                        type Scale = scale::$SCALE;
                        type NArgs = <Self::Arguments as signature::Property<Arg::Inner>>::NReg;
                        type NRes = <Self::Results as signature::Property<Res::Inner>>::NReg;
                    }
                )+
            )+
        };
    }

    impls! {
        (Cos<N0>, Cosine, One, One, start(angle)),
        (Sin<N0>, Sine, One, One, start(angle)),
        (SinCos<N0>, Sine, One, Two, start(angle)),
        (CosM<N0>, Cosine, Two, One, start(angle, modulus)),
        (SinM<N0>, Sine, Two, One, start(angle, modulus)),
        (SinCosM<N0>, Sine, Two, Two, start(angle, modulus)),
        (ATan2<N0>, Phase, Two, One, start(x, y)),
        (Magnitude<N0>, Modulus, Two, One, start(x, y)),
        (ATan2Magnitude<N0>, Phase, Two, Two, start(x, y)),
        (CosH<N1>, HyperbolicCosine, One, One, start(x)),
        (SinH<N1>, HyperbolicSine, One, One, start(x)),
        (SinHCosH<N1>, HyperbolicSine, One, Two, start(x)),
        (ATanH<N1>, Arctanh, One, One, start(x)),
    }

    impls_multi_scale! {
        (ATan<N0, N1, N2, N3, N4, N5, N6, N7>, Arctangent, One, One, start(x)),
        (Ln<N1, N2, N3, N4>, NaturalLogarithm, One, One, start(x)),
        (Sqrt<N0, N1, N2>, SquareRoot, One, One, start(x)),
    }

    /// Traits and structures for dynamic function operation.
    pub mod dynamic {
        use super::{prec, signature, types, Cordic, Feature};

        /// Any function can be invoked with this type-state.
        pub struct Any;

        /// A Cordic in dynamic mode.
        pub trait Mode<Arg, Res>
        where
            Arg: types::arg::State,
            Res: types::res::State,
        {
            /// Run a function with provided arguments and get the result.
            ///
            /// *Note: This employs the polling strategy.
            /// For less overhead, use static operations.*
            fn run<Func>(&mut self, args: Func::Arguments) -> Func::Results
            where
                Func: Feature<Arg, Res>;
        }

        impl<Arg, Res, Prec> Mode<Arg, Res> for Cordic<Arg, Res, Any, Prec>
        where
            Arg: types::arg::State,
            Res: types::res::State,
            Prec: prec::State,
        {
            fn run<Func>(&mut self, args: Func::Arguments) -> Func::Results
            where
                Func: Feature<Arg, Res>,
            {
                use signature::Property as _;

                self.apply_config::<Arg, Res, Func, Prec>();

                args.write(&self.rb.wdata);
                self.when_ready(|cordic| Func::Results::read(&cordic.rb.rdata))
            }
        }
    }
}

/// Traits and structures related to precision type-states.
pub mod prec {
    /// Trait for precision type-states.
    pub(crate) trait State {
        /// Bit representation of the precision.
        const BITS: u8;

        /// Configure the resource to be represented
        /// by this type-state.
        fn set<const OFFSET: u8>(w: crate::stm32::cordic::csr::PRECISION_W<OFFSET>);
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
                    fn set<const OFFSET: u8>(w: crate::stm32::cordic::csr::PRECISION_W<OFFSET>) {
                        // SAFETY: reliant on valid type-state
                        // implementations.
                        unsafe { w.bits(<Self as State>::BITS) };
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

/// Cordic co-processor interface.
pub struct Cordic<Arg, Res, Func, Prec>
where
    Arg: types::arg::State,
    Res: types::res::State,
    Prec: prec::State,
{
    rb: CORDIC,
    _arg_size: PhantomData<Arg>,
    _res_size: PhantomData<Res>,
    _func: PhantomData<Func>,
    _prec: PhantomData<Prec>,
}

// root impl
impl<Arg, Res, Func, Prec> Cordic<Arg, Res, Func, Prec>
where
    Arg: types::arg::State,
    Res: types::res::State,
    Prec: prec::State,
{
    fn apply_config<NewArg, NewRes, NewFunc, NewPrec>(&mut self)
    where
        NewArg: types::arg::State,
        NewRes: types::res::State,
        NewFunc: func::Feature<NewArg, NewRes>,
        NewPrec: prec::State,
    {
        self.rb.csr.write(|w| {
            use func::reg_count::arg::State as _;
            use func::reg_count::res::State as _;
            use func::scale::State as _;

            NewArg::set(w.argsize());
            NewRes::set(w.ressize());
            NewPrec::set(w.precision());
            NewFunc::set(w.func());
            NewFunc::NArgs::set(w.nargs());
            NewFunc::NRes::set(w.nres());
            NewFunc::Scale::set(w.scale());

            w
        });
    }

    /// Configure the resource as dictated by the resulting
    /// type-states. The produced binding represents
    /// a frozen configuration, since it is represented
    /// by types. A new binding will need to be made --
    /// and the old binding invalidated -- in order to change
    /// the configuration.
    ///
    /// *Note: The configuration is inferred from context because
    /// it is represented by generic type-states.*
    pub fn freeze<NewArg, NewRes, NewFunc, NewPrec>(
        mut self,
    ) -> Cordic<NewArg, NewRes, NewFunc, NewPrec>
    where
        NewArg: types::arg::State,
        NewRes: types::res::State,
        NewFunc: func::Feature<NewArg, NewRes>,
        NewPrec: prec::State,
    {
        self.apply_config::<NewArg, NewRes, NewFunc, NewPrec>();

        // SAFETY: the resource has been configured
        // to represent the new type-states.
        unsafe { Cordic::wrap(self.rb) }
    }

    /// Determine whether a result is pending or not.
    #[inline]
    pub fn is_ready(&self) -> bool {
        self.rb.csr.read().rrdy().bit_is_set()
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
    pub fn into_dynamic(self) -> Cordic<Arg, Res, func::dynamic::Any, Prec> {
        unsafe { Cordic::wrap(self.rb) }
    }

    /// Wrap the resource as a noop.
    ///
    /// # Safety
    ///
    /// If the resource configuration and
    /// type-states are incongruent, the invariance
    /// is broken and actions may exhibit
    /// undefined behavior.
    pub const unsafe fn wrap(rb: CORDIC) -> Self {
        Self {
            rb,
            _arg_size: PhantomData,
            _res_size: PhantomData,
            _func: PhantomData,
            _prec: PhantomData,
        }
    }
}

/// $RM0440 17.4.1
pub type CordicReset = Cordic<types::Q31, types::Q31, func::Cos, prec::P20>;

// reset
impl<Arg, Res, Func, Prec> Cordic<Arg, Res, Func, Prec>
where
    Arg: types::arg::State,
    Res: types::res::State,
    Func: func::Feature<Arg, Res>,
    Prec: prec::State,
{
    /// Configure the resource back to the "reset"
    /// state.
    #[inline]
    fn reset(self) -> CordicReset {
        self.freeze()
    }
}

// listen
impl<Arg, Res, Func, Prec> Cordic<Arg, Res, Func, Prec>
where
    Arg: types::arg::State,
    Res: types::res::State,
    Func: func::Feature<Arg, Res>,
    Prec: prec::State,
{
    /// Enable the result ready interrupt.
    #[inline]
    pub fn listen(&mut self) {
        self.rb.csr.modify(|_, w| w.ien().set_bit());
    }

    /// Disable the result ready interrupt.
    #[inline]
    pub fn unlisten(&mut self) {
        self.rb.csr.modify(|_, w| w.ien().clear_bit());
    }
}

// release
impl<Arg, Res, Func, Prec> Cordic<Arg, Res, Func, Prec>
where
    Arg: types::arg::State,
    Res: types::res::State,
    Func: func::Feature<Arg, Res>,
    Prec: prec::State,
{
    /// Release the CORDIC resource binding as a noop.
    ///
    /// # Safety
    ///
    /// The CORDIC peripheral is not reset.
    #[inline]
    pub unsafe fn release(self) -> CORDIC {
        self.rb
    }

    /// Release the CORDIC resource binding after reset.
    #[inline]
    pub fn release_and_reset(self, rcc: &mut Rcc) -> CORDIC {
        let reset = self.reset();

        rcc.rb.ahb1enr.modify(|_, w| w.cordicen().clear_bit());

        // SAFETY: the resource has been reset
        unsafe { reset.release() }
    }
}
