use crate::gpio::{self, AF10};
use crate::rcc::{Enable, Rcc, Reset};
use crate::stm32::QUADSPI;

pub trait Pins {}

pub trait PinClk {}

pub trait PinIo0Bank1 {}
pub trait PinIo1Bank1 {}
pub trait PinIo2Bank1 {}
pub trait PinIo3Bank1 {}
pub trait PinNcsBank1 {}

pub trait PinIo0Bank2 {}
pub trait PinIo1Bank2 {}
pub trait PinIo2Bank2 {}
pub trait PinIo3Bank2 {}
pub trait PinNcsBank2 {}

/// A filler type for when the bank 1 NCS pin is also used for bank 2
pub struct SharedNcs;

impl<
        CLK,
        IO0_BANK1,
        IO1_BANK1,
        IO2_BANK1,
        IO3_BANK1,
        NCS_BANK1,
        IO0_BANK2,
        IO1_BANK2,
        IO2_BANK2,
        IO3_BANK2,
    > Pins
    for (
        CLK,
        IO0_BANK1,
        IO1_BANK1,
        IO2_BANK1,
        IO3_BANK1,
        NCS_BANK1,
        IO0_BANK2,
        IO1_BANK2,
        IO2_BANK2,
        IO3_BANK2,
    )
where
    CLK: PinClk,
    IO0_BANK1: PinIo0Bank1,
    IO1_BANK1: PinIo1Bank1,
    IO2_BANK1: PinIo2Bank1,
    IO3_BANK1: PinIo3Bank1,
    NCS_BANK1: PinNcsBank1,
    IO0_BANK2: PinIo0Bank2,
    IO1_BANK2: PinIo1Bank2,
    IO2_BANK2: PinIo2Bank2,
    IO3_BANK2: PinIo3Bank2,
{
}

pub struct Qspi {
    pub(super) inst: QUADSPI,
}

#[derive(PartialEq)]
pub enum FlashMode {
    Flash1,
    Flash2,
    Dual,
}

#[derive(PartialEq)]
pub enum ClockMode {
    Mode0,
    Mode3,
}

pub trait QuadSpiExt {
    #[allow(clippy::too_many_arguments)]
    fn qspi<PINS>(
        self,
        pins: PINS,
        rcc: &mut Rcc,
        clock_prescalar: u8,
        fifo_threshold: u8,
        sample_shifting: bool,
        flash_size: u8,
        chip_select_high_time: u8,
        clock_mode: ClockMode,
        flash_mode: FlashMode,
    ) -> Qspi;
}

impl QuadSpiExt for QUADSPI {
    #[allow(clippy::too_many_arguments)]
    fn qspi<PINS>(
        self,
        _pins: PINS,
        rcc: &mut Rcc,
        clock_prescalar: u8,
        fifo_threshold: u8,
        sample_shifting: bool,
        flash_size: u8,
        chip_select_high_time: u8,
        clock_mode: ClockMode,
        flash_mode: FlashMode,
    ) -> Qspi {
        QUADSPI::enable(rcc);
        QUADSPI::reset(rcc);

        // Disable during configuration and re-enable afterwards
        self.cr().modify(|_, w| w.en().clear_bit());

        self.cr().write(|w| unsafe {
            w.fthres()
                .bits(fifo_threshold - 1)
                .prescaler()
                .set(clock_prescalar)
                .sshift()
                .bit(sample_shifting)
                .fsel()
                .bit(flash_mode == FlashMode::Flash2)
                .dfm()
                .bit(flash_mode == FlashMode::Dual)
        });

        self.dcr().write(|w| {
            w.fsize()
                .set(flash_size)
                .csht()
                .set(chip_select_high_time)
                .ckmode()
                .bit(clock_mode == ClockMode::Mode3)
        });

        self.cr().modify(|_, w| w.en().set_bit());

        Qspi { inst: self }
    }
}

#[derive(Clone, Copy)]
pub enum LineMode {
    Single,
    Dual,
    Quad,
}

#[derive(Clone, Copy)]
struct CommandInstruction {
    pub mode: LineMode,
    pub data: u8,
}

#[derive(Clone, Copy)]
pub enum CommandArgumentData {
    OneByte([u8; 1]),
    TwoBytes([u8; 2]),
    ThreeBytes([u8; 3]),
    FourBytes([u8; 4]),
}

impl CommandArgumentData {
    pub fn to_u32(&self) -> u32 {
        match self {
            CommandArgumentData::OneByte(x) => x[0] as u32,
            CommandArgumentData::TwoBytes(x) => u16::from_le_bytes(*x) as u32,
            CommandArgumentData::ThreeBytes(x) => u32::from_le_bytes([x[0], x[1], x[2], 0]),
            CommandArgumentData::FourBytes(x) => u32::from_le_bytes(*x),
        }
    }
}

impl From<[u8; 1]> for CommandArgumentData {
    fn from(value: [u8; 1]) -> Self {
        Self::OneByte(value)
    }
}

impl From<[u8; 2]> for CommandArgumentData {
    fn from(value: [u8; 2]) -> Self {
        Self::TwoBytes(value)
    }
}

impl From<[u8; 3]> for CommandArgumentData {
    fn from(value: [u8; 3]) -> Self {
        Self::ThreeBytes(value)
    }
}

impl From<[u8; 4]> for CommandArgumentData {
    fn from(value: [u8; 4]) -> Self {
        Self::FourBytes(value)
    }
}

#[derive(Clone, Copy)]
struct CommandArgument {
    pub mode: LineMode,
    pub data: CommandArgumentData,
}

#[derive(PartialEq, Clone, Copy)]
pub enum DdrMode {
    Disabled,
    Enabled,
    EnabledWithHold,
}

pub struct Command {
    ddr_mode: DdrMode,

    instruction: Option<CommandInstruction>,
    address: Option<CommandArgument>,
    alternate_bytes: Option<CommandArgument>,
}

impl Command {
    pub fn new(ddr_mode: DdrMode) -> Self {
        Self {
            ddr_mode,

            instruction: None,
            address: None,
            alternate_bytes: None,
        }
    }

    pub fn with_instruction(mut self, line_mode: LineMode, instruction: u8) -> Self {
        self.instruction = Some(CommandInstruction {
            mode: line_mode,
            data: instruction,
        });
        self
    }

    pub fn with_address<T>(mut self, line_mode: LineMode, adddress: T) -> Self
    where
        T: Into<CommandArgumentData>,
    {
        self.address = Some(CommandArgument {
            mode: line_mode,
            data: adddress.into(),
        });
        self
    }

    pub fn with_alternate_bytes<T>(mut self, line_mode: LineMode, alternate_bytes: T) -> Self
    where
        T: Into<CommandArgumentData>,
    {
        self.alternate_bytes = Some(CommandArgument {
            mode: line_mode,
            data: alternate_bytes.into(),
        });
        self
    }
}

pub struct IoCommand {
    ddr_mode: DdrMode,
    send_instruction_once: bool,

    instruction: Option<CommandInstruction>,
    address: Option<CommandArgument>,
    alternate_bytes: Option<CommandArgument>,

    dummy_cycles: u8,
    data_mode: LineMode,
}

impl IoCommand {
    pub fn new(ddr_mode: DdrMode, line_mode: LineMode) -> Self {
        Self {
            ddr_mode,
            send_instruction_once: false,

            instruction: None,
            address: None,
            alternate_bytes: None,

            dummy_cycles: 0,
            data_mode: line_mode,
        }
    }

    pub fn with_send_instruction_once(mut self) -> Self {
        self.send_instruction_once = true;
        self
    }

    pub fn with_instruction(mut self, line_mode: LineMode, instruction: u8) -> Self {
        self.instruction = Some(CommandInstruction {
            mode: line_mode,
            data: instruction,
        });
        self
    }

    pub fn with_address<T>(mut self, line_mode: LineMode, adddress: T) -> Self
    where
        T: Into<CommandArgumentData>,
    {
        self.address = Some(CommandArgument {
            mode: line_mode,
            data: adddress.into(),
        });
        self
    }

    pub fn with_alternate_bytes<T>(mut self, line_mode: LineMode, alternate_bytes: T) -> Self
    where
        T: Into<CommandArgumentData>,
    {
        self.alternate_bytes = Some(CommandArgument {
            mode: line_mode,
            data: alternate_bytes.into(),
        });
        self
    }

    pub fn with_dummy_cycles(mut self, dummy_cycles: u8) -> Self {
        self.dummy_cycles = dummy_cycles;
        self
    }
}

enum FunctionalMode {
    Read,
    Write,
    #[allow(unused)]
    Poll,
    Mapped,
}

impl Qspi {
    #[allow(clippy::too_many_arguments)]
    fn config(
        inst: &mut QUADSPI,
        functional_mode: FunctionalMode,
        send_instruction_once: bool,
        ddr_mode: DdrMode,
        instruction: Option<CommandInstruction>,
        address: Option<CommandArgument>,
        alternate_bytes: Option<CommandArgument>,
        dummy_cycles: u8,
        data_mode: Option<LineMode>,
    ) {
        // The operation will be triggered by a write to:
        // - CCR: if the AR and DR are not required;
        // - AR: if DR is not required; or
        // - DR: otherwise.

        if let Some(altb) = alternate_bytes {
            inst.abr().write(|w| unsafe { w.bits(altb.data.to_u32()) });
        }

        inst.ccr().write(|w| {
            if let Some(inst) = instruction {
                w.instruction().set(inst.data);
                match inst.mode {
                    LineMode::Single => w.imode().single_line(),
                    LineMode::Dual => w.imode().two_lines(),
                    LineMode::Quad => w.imode().four_lines(),
                };
            } else {
                w.imode().no_instruction();
            }

            if let Some(addr) = address {
                match addr.mode {
                    LineMode::Single => w.admode().single_line(),
                    LineMode::Dual => w.admode().two_lines(),
                    LineMode::Quad => w.admode().four_lines(),
                };
                match addr.data {
                    CommandArgumentData::OneByte(_) => w.adsize().bit8(),
                    CommandArgumentData::TwoBytes(_) => w.adsize().bit16(),
                    CommandArgumentData::ThreeBytes(_) => w.adsize().bit24(),
                    CommandArgumentData::FourBytes(_) => w.adsize().bit32(),
                };
            } else {
                w.admode().no_address();
            }

            if let Some(altb) = alternate_bytes {
                match altb.mode {
                    LineMode::Single => w.abmode().single_line(),
                    LineMode::Dual => w.abmode().two_lines(),
                    LineMode::Quad => w.abmode().four_lines(),
                };
                match altb.data {
                    CommandArgumentData::OneByte(_) => w.absize().bit8(),
                    CommandArgumentData::TwoBytes(_) => w.absize().bit16(),
                    CommandArgumentData::ThreeBytes(_) => w.absize().bit24(),
                    CommandArgumentData::FourBytes(_) => w.absize().bit32(),
                };
            } else {
                w.abmode().no_alternate_bytes();
            }

            w.dcyc().set(dummy_cycles);

            if let Some(datm) = data_mode {
                match datm {
                    LineMode::Single => w.dmode().single_line(),
                    LineMode::Dual => w.dmode().two_lines(),
                    LineMode::Quad => w.dmode().four_lines(),
                };
            } else {
                w.dmode().no_data();
            }

            match functional_mode {
                FunctionalMode::Read => w.fmode().indirect_read(),
                FunctionalMode::Write => w.fmode().indirect_write(),
                FunctionalMode::Poll => w.fmode().automatic_polling(),
                FunctionalMode::Mapped => w.fmode().memory_mapped(),
            };

            w.sioo()
                .bit(send_instruction_once)
                .dhhc()
                .bit(ddr_mode == DdrMode::EnabledWithHold)
                .ddrm()
                .bit(ddr_mode != DdrMode::Disabled)
        });

        if let Some(addr) = address {
            inst.ar().write(|w| unsafe { w.bits(addr.data.to_u32()) });
        }
    }

    fn wait_not_busy(&self) {
        while self.inst.sr().read().busy().is_busy() {}
    }

    pub fn command(&mut self, command: Command) {
        self.wait_not_busy();

        Self::config(
            &mut self.inst,
            FunctionalMode::Write,
            false,
            command.ddr_mode,
            command.instruction,
            command.address,
            command.alternate_bytes,
            0,
            None,
        );

        while self.inst.sr().read().tcf().is_not_complete() {}
    }

    pub fn read(&mut self, command: IoCommand, data: &mut [u8]) {
        self.wait_not_busy();

        self.inst
            .dlr()
            .write(|w| w.dl().set(u32::try_from(data.len()).unwrap() - 1));

        Self::config(
            &mut self.inst,
            FunctionalMode::Read,
            command.send_instruction_once,
            command.ddr_mode,
            command.instruction,
            command.address,
            command.alternate_bytes,
            command.dummy_cycles,
            Some(command.data_mode),
        );

        for d in data {
            *d = self.inst.dr8().read().bits();
        }
    }

    pub fn write(&mut self, command: IoCommand, data: &[u8]) {
        self.wait_not_busy();

        self.inst
            .dlr()
            .write(|w| w.dl().set(u32::try_from(data.len()).unwrap() - 1));

        Self::config(
            &mut self.inst,
            FunctionalMode::Write,
            command.send_instruction_once,
            command.ddr_mode,
            command.instruction,
            command.address,
            command.alternate_bytes,
            command.dummy_cycles,
            Some(command.data_mode),
        );

        for d in data {
            self.inst.dr8().write(|w| w.set(*d));
        }

        while self.inst.sr().read().tcf().is_not_complete() {}
    }

    pub fn memory_mapped(&mut self, command: IoCommand) {
        self.wait_not_busy();

        Self::config(
            &mut self.inst,
            FunctionalMode::Mapped,
            command.send_instruction_once,
            command.ddr_mode,
            command.instruction,
            command.address,
            command.alternate_bytes,
            command.dummy_cycles,
            Some(command.data_mode),
        );
    }

    pub fn abort(&mut self) {
        self.inst.cr().modify(|_, w| w.abort().abort_requested());
    }
}

impl PinIo0Bank1 for gpio::PA3<AF10> {}
impl PinIo0Bank1 for gpio::PB10<AF10> {}
impl PinIo0Bank1 for gpio::PE10<AF10> {}
impl PinIo0Bank1 for gpio::PF10<AF10> {}

impl PinIo0Bank1 for gpio::PB1<AF10> {}
impl PinIo0Bank1 for gpio::PE12<AF10> {}
impl PinIo0Bank1 for gpio::PF8<AF10> {}

impl PinIo1Bank1 for gpio::PB0<AF10> {}
impl PinIo1Bank1 for gpio::PE13<AF10> {}
impl PinIo1Bank1 for gpio::PF9<AF10> {}

impl PinIo2Bank1 for gpio::PA7<AF10> {}
impl PinIo2Bank1 for gpio::PE14<AF10> {}
impl PinIo2Bank1 for gpio::PF7<AF10> {}

impl PinIo3Bank1 for gpio::PA6<AF10> {}
impl PinIo3Bank1 for gpio::PE15<AF10> {}
impl PinIo3Bank1 for gpio::PF6<AF10> {}

impl PinNcsBank1 for gpio::PA2<AF10> {}
impl PinNcsBank1 for gpio::PB11<AF10> {}
impl PinNcsBank1 for gpio::PE11<AF10> {}

impl PinIo0Bank2 for gpio::PC1<AF10> {}
impl PinIo0Bank2 for gpio::PD4<AF10> {}

impl PinIo1Bank2 for gpio::PB2<AF10> {}
impl PinIo1Bank2 for gpio::PC2<AF10> {}
impl PinIo1Bank2 for gpio::PD5<AF10> {}

impl PinIo1Bank2 for gpio::PC3<AF10> {}
impl PinIo1Bank2 for gpio::PD6<AF10> {}

impl PinIo1Bank2 for gpio::PC4<AF10> {}
impl PinIo1Bank2 for gpio::PD7<AF10> {}

impl PinNcsBank2 for gpio::PD3<AF10> {}
