macro_rules! opamps {
    {
        $(
            $opamp:ident
            :
            {
                inverting
                :
                {
                    $(
                        $inverting:ty
                        :
                        $inverting_mask:tt
                    ),*
                    $(,)?
                }
                ,
                non_inverting
                :
                {
                    $(
                        $non_inverting:ty
                        :
                        $non_inverting_mask:tt
                    ),*
                    $(,)?
                }
                ,
                output
                :
                $output:ty
                ,
            }
        ),*
        $(,)?
    } => {
        paste::paste!{
            pub mod prelude {
                $(pub use super::$opamp::IntoFollower as _;)*
            }

            $(
                pub mod $opamp {
                    pub struct Disabled;

                    pub struct Follower<Input> {
                        input: Input,
                        output: Option<$output>,
                    }

                    pub struct OpenLoop<NonInverting, Inverting> {
                        non_inverting: NonInverting,
                        inverting: Inverting,
                        output: Option<$output>,
                    }

                    pub trait IntoFollower <IntoInput, IntoOutput, Input>
                        where
                            IntoOutput: Into<$output>,
                    {
                        fn follower(self, input: IntoInput, output: Option<IntoOutput>) -> Follower<Input>;
                    }

                    pub trait IntoOpenLoop <IntoNonInverting, IntoInverting, IntoOutput, NonInverting, Inverting>
                        where
                            IntoOutput: Into<$output>,
                    {
                        fn open_loop(self, non_inverting: IntoNonInverting, inverting: IntoInverting, output: Option<IntoOutput>)
                            -> OpenLoop<NonInverting, Inverting>;
                    }

                    impl<Input> Follower<Input> {
                        pub fn disable(self) -> (Disabled, Input, Option<$output>) {
                            unsafe { (*crate::stm32::OPAMP::ptr()).[<$opamp _csr>].reset() }
                            (Disabled, self.input, self.output)
                        }

                        pub fn enable_output(&mut self, output:$output) {
                            self.output = Some(output);
                            unsafe {
                                (*crate::stm32::OPAMP::ptr()).[<$opamp _csr>].write(|w|
                                    w.opaintoen().bit(true));
                            }
                        }

                        pub fn disable_output(&mut self) -> Option<$output> {
                            unsafe {
                                (*crate::stm32::OPAMP::ptr()).[<$opamp _csr>].write(|w|
                                    w.opaintoen().bit(false));
                            }
                            self.output.take()
                        }
                    }

                    impl<NonInverting, Inverting> OpenLoop<NonInverting, Inverting> {
                        pub fn disable(self) -> (Disabled, NonInverting, Inverting) {
                            unsafe { (*crate::stm32::OPAMP::ptr()).[<$opamp _csr>].reset() }
                            (Disabled, self.non_inverting, self.inverting)
                        }

                        pub fn enable_output(&mut self, output:$output) {
                            self.output = Some(output);
                            unsafe {
                                (*crate::stm32::OPAMP::ptr()).[<$opamp _csr>].write(|w|
                                    w.opaintoen().bit(true));
                            }
                        }

                        pub fn disable_output(&mut self) -> Option<$output> {
                            unsafe {
                                (*crate::stm32::OPAMP::ptr()).[<$opamp _csr>].write(|w|
                                    w.opaintoen().bit(false));
                            }
                            self.output.take()
                        }
                    }

                    opamps!{ @follower $opamp, $output, $($non_inverting_mask, $non_inverting),* }
                    opamps!{ @open_loop_tt $opamp, $output, $($non_inverting_mask, $non_inverting),* : ($($inverting_mask, $inverting),*) }

                }
            )*

            pub trait OpampEx {
                fn split(
                    self,
                    rcc: &mut crate::rcc::Rcc,
                ) -> (
                    $($opamp::Disabled,)*
                );
            }

            impl OpampEx for crate::stm32::OPAMP {
                fn split(
                    self,
                    rcc: &mut crate::rcc::Rcc,
                ) -> (
                    $($opamp::Disabled,)*
                ) {
                    rcc.rb.apb2enr.write(|w| w.syscfgen().set_bit());

                    (
                        $($opamp::Disabled,)*
                    )
                }
            }
        }
    };

    {
        @follower
        $opamp:ident
        ,
        $output:ty
        ,
        $(
            $input_mask:tt
            ,
            $input:ty
        ),*
    } => {
        paste::paste!{
            $(impl <IntoInput, IntoOutput> IntoFollower <IntoInput, IntoOutput, $input> for Disabled
                where
                    IntoInput: Into<$input>,
                    IntoOutput: Into<$output>,
            {
                fn follower(
                    self,
                    input: IntoInput,
                    output: Option<IntoOutput>,
                ) -> Follower<$input> {
                    let input = input.into();
                    let output = match output {
                        Some(output) => Some(output.into()),
                        None => None,
                    };
                    unsafe {
                        (*crate::stm32::OPAMP::ptr())
                            .[<$opamp _csr>]
                            .write(|csr_w|
                                csr_w
                                    .vp_sel()
                                    .bits($input_mask)
                                    .vm_sel()
                                    .bits(0b11)
                                    .opaen()
                                    .bit(true)
                            );
                    }
                    let mut res = Follower {input, output: None};
                    if let Some(output) = output {
                        res.enable_output(output);
                    }
                    res
                }
            })*
        }
    };

    {
        @open_loop_tt
        $opamp:ident
        ,
        $output:ty
        ,
        $($non_inverting_mask:tt, $non_inverting:ty),*
        :
        $invertings:tt
    } => {
        $(
            opamps!{ @open_loop $opamp, $output, $non_inverting_mask, $non_inverting, $invertings }
        )*
    };

    {
        @open_loop
        $opamp:ident
        ,
        $output:ty
        ,
        $non_inverting_mask:tt
        ,
        $non_inverting:ty
        ,
        ($($inverting_mask:tt, $inverting:ty),*)
    } => {
        paste::paste!{
            $(impl <IntoNonInverting, IntoInverting, IntoOutput> IntoOpenLoop
                <IntoNonInverting, IntoInverting, IntoOutput, $non_inverting, $inverting> for Disabled
                where
                    IntoNonInverting: Into<$non_inverting>,
                    IntoInverting: Into<$inverting>,
                    IntoOutput: Into<$output>,
            {
                fn open_loop(
                    self,
                    non_inverting: IntoNonInverting,
                    inverting: IntoInverting,
                    output: Option<IntoOutput>,
                ) -> OpenLoop<$non_inverting, $inverting> {
                    let non_inverting = non_inverting.into();
                    let inverting = inverting.into();
                    let output = match output {
                        Some(output) => Some(output.into()),
                        None => None,
                    };
                    unsafe {
                        (*crate::stm32::OPAMP::ptr())
                            .[<$opamp _csr>]
                            .write(|csr_w|
                                csr_w.vp_sel()
                                    .bits($non_inverting_mask)
                                    .vm_sel()
                                    .bits($inverting_mask)
                                    .opaen()
                                    .bit(true)
                            );
                    }
                    let mut res = OpenLoop {non_inverting, inverting, output: None};
                    if let Some(output) = output {
                        res.enable_output(output);
                    }
                    res
                }
            })*
        }
    };
}

opamps! {
    opamp1: {
        inverting: {
            crate::gpio::gpioa::PA3<crate::gpio::Analog>: 0b00,
            crate::gpio::gpioc::PC5<crate::gpio::Analog>: 0b01,
        },
        non_inverting: {
            crate::gpio::gpioa::PA1<crate::gpio::Analog>: 0b00,
            crate::gpio::gpioa::PA3<crate::gpio::Analog>: 0b01,
            crate::gpio::gpioa::PA7<crate::gpio::Analog>: 0b10,
        },
        output: crate::gpio::gpioa::PA2<crate::gpio::Analog>,
    },
    opamp2: {
        inverting: {
            crate::gpio::gpioa::PA5<crate::gpio::Analog>: 0b00,
            crate::gpio::gpioc::PC5<crate::gpio::Analog>: 0b01,
        },
        non_inverting: {
            crate::gpio::gpioa::PA7<crate::gpio::Analog>: 0b00,
            crate::gpio::gpiob::PB14<crate::gpio::Analog>: 0b01,
            crate::gpio::gpiob::PB0<crate::gpio::Analog>: 0b10,
            crate::gpio::gpiod::PD14<crate::gpio::Analog>: 0b11,
        },
        output: crate::gpio::gpioa::PA6<crate::gpio::Analog>,
    },
    opamp3: {
        inverting: {
            crate::gpio::gpiob::PB2<crate::gpio::Analog>: 0b00,
            crate::gpio::gpiob::PB10<crate::gpio::Analog>: 0b01,
        },
        non_inverting: {
            crate::gpio::gpiob::PB0<crate::gpio::Analog>: 0b00,
            crate::gpio::gpiob::PB13<crate::gpio::Analog>: 0b01,
            crate::gpio::gpioa::PA1<crate::gpio::Analog>: 0b10,
        },
        output: crate::gpio::gpiob::PB1<crate::gpio::Analog>,
    },
    opamp4: {
        inverting: {
            crate::gpio::gpiob::PB10<crate::gpio::Analog>: 0b00,
            crate::gpio::gpiod::PD8<crate::gpio::Analog>: 0b01,
        },
        non_inverting: {
            crate::gpio::gpiob::PB13<crate::gpio::Analog>: 0b00,
            crate::gpio::gpiod::PD11<crate::gpio::Analog>: 0b01,
            crate::gpio::gpiob::PB11<crate::gpio::Analog>: 0b10,
        },
        output: crate::gpio::gpiob::PB12<crate::gpio::Analog>,
    },
    opamp5: {
        inverting: {
            crate::gpio::gpiob::PB15<crate::gpio::Analog>: 0b00,
            crate::gpio::gpioa::PA3<crate::gpio::Analog>: 0b01,
        },
        non_inverting: {
            crate::gpio::gpiob::PB14<crate::gpio::Analog>: 0b00,
            crate::gpio::gpiod::PD12<crate::gpio::Analog>: 0b01,
            crate::gpio::gpioc::PC3<crate::gpio::Analog>: 0b10,
        },
        output: crate::gpio::gpioa::PA8<crate::gpio::Analog>,
    },
    opamp6: {
        inverting: {
            crate::gpio::gpioa::PA1<crate::gpio::Analog>: 0b00,
            crate::gpio::gpiob::PB1<crate::gpio::Analog>: 0b01,
        },
        non_inverting: {
            crate::gpio::gpiob::PB12<crate::gpio::Analog>: 0b00,
            crate::gpio::gpiod::PD9<crate::gpio::Analog>: 0b01,
            crate::gpio::gpiob::PB13<crate::gpio::Analog>: 0b10,
        },
        output: crate::gpio::gpiob::PB11<crate::gpio::Analog>,
    },
}
