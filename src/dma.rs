//! Direct Memory Access Engine
use crate::rcc::Rcc;
use crate::stm32::DMA;

#[derive(Debug)]
pub enum Error {
    Overrun,
    BufferError,
}

#[derive(Debug)]
pub enum Event {
    HalfTransfer,
    TransferComplete,
}

#[derive(Clone, Copy, PartialEq)]
pub enum Half {
    First,
    Second,
}

pub trait DmaExt {
    type Channels;

    fn split(self, rcc: &mut Rcc) -> Self::Channels;
}

macro_rules! dma {
    ($($DMAX:ident: ($dmaXen:ident, $dmaXrst:ident, {
        $($CX:ident: (
            $ccrX:ident,
            $CCRX:ident,
            $cndtrX:ident,
            $CNDTRX:ident,
            $cparX:ident,
            $CPARX:ident,
            $cmarX:ident,
            $CMARX:ident,
            $htifX:ident,
            $tcifX:ident,
            $chtifX:ident,
            $ctcifX:ident,
            $cgifX:ident
        ),)+
    }),)+) => {
        $(
            impl DmaExt for $DMAX {
                type Channels = Channels;

                fn split(self, rcc: &mut Rcc) -> Channels {
                    rcc.rb.ahbenr.modify(|_, w| w.$dmaXen().set_bit());
                    $(
                        self.$ccrX.reset();
                    )+
                    Channels((), $($CX { }),+)
                }
            }

            pub struct Channels((), $(pub $CX),+);

            $(
                pub struct $CX;

                impl $CX {
                    pub fn listen(&mut self, event: Event) {
                        let dma = unsafe { &(*$DMAX::ptr()) };
                        match event {
                            Event::HalfTransfer => dma.$ccrX.modify(|_, w| w.htie().set_bit()),
                            Event::TransferComplete => {
                                dma.$ccrX.modify(|_, w| w.tcie().set_bit())
                            }
                        }
                    }

                    pub fn unlisten(&mut self, event: Event) {
                        let dma = unsafe { &(*$DMAX::ptr()) };
                        match event {
                            Event::HalfTransfer => {
                                dma.$ccrX.modify(|_, w| w.htie().clear_bit())
                            },
                            Event::TransferComplete => {
                                dma.$ccrX.modify(|_, w| w.tcie().clear_bit())
                            }
                        }
                    }
                }
            )+
        )+
    }
}

dma! {
    DMA: (dmaen, dma1rst, {
        C1: (
            ccr1, CCR1,
            cndtr1, CNDTR1,
            cpar1, CPAR1,
            cmar1, CMAR1,
            htif1, tcif1,
            chtif1, ctcif1,
            cgif1
        ),
        C2: (
            ccr2, CCR2,
            cndtr2, CNDTR2,
            cpar2, CPAR2,
            cmar2, CMAR2,
            htif2, tcif2,
            chtif2, ctcif2,
            cgif2
        ),
        C3: (
            ccr3, CCR3,
            cndtr3, CNDTR3,
            cpar3, CPAR3,
            cmar3, CMAR3,
            htif3, tcif3,
            chtif3, ctcif3,
            cgif3
        ),
        C4: (
            ccr4, CCR4,
            cndtr4, CNDTR4,
            cpar4, CPAR4,
            cmar4, CMAR4,
            htif4, tcif4,
            chtif4, ctcif4,
            cgif4
        ),
        C5: (
            ccr5, CCR5,
            cndtr5, CNDTR5,
            cpar5, CPAR5,
            cmar5, CMAR5,
            htif5, tcif5,
            chtif5, ctcif5,
            cgif5
        ),
        C6: (
            ccr6, CCR6,
            cndtr6, CNDTR6,
            cpar6, CPAR6,
            cmar6, CMAR6,
            htif6, tcif6,
            chtif6, ctcif6,
            cgif6
        ),
        C7: (
            ccr7, CCR7,
            cndtr7, CNDTR7,
            cpar7, CPAR7,
            cmar7, CMAR7,
            htif7, tcif7,
            chtif7, ctcif7,
            cgif7
        ),
    }),
}
