#![feature(const_generics)]
#![feature(box_syntax)]
#![feature(exclusive_range_pattern)]
#![allow(incomplete_features)]
#![allow(unused_unsafe)]

use bytemuck;
use derivative::Derivative;
use std::fmt;

pub mod arm;
pub mod bitutils;

use arm::instruction;
use arm::prelude::*;
use bitutils::*;

#[derive(Copy, Clone)]
pub struct SmallAsciiString<const N: usize> {
    data: [u8; N],
}

impl<const N: usize> fmt::Debug for SmallAsciiString<N> {
    fn fmt(&self, fmt: &mut fmt::Formatter) -> fmt::Result {
        fmt.write_str(std::str::from_utf8(&self.data).unwrap())
    }
}

type SmallAsciiString12 = SmallAsciiString<12>;
type SmallAsciiString4 = SmallAsciiString<4>;
type SmallAsciiString2 = SmallAsciiString<2>;

trait FormatBinary {
    fn fmt(&self, fmt: &mut fmt::Formatter) -> fmt::Result;
}

impl<T> FormatBinary for T
where
    T: bytemuck::Pod,
{
    fn fmt(&self, fmt: &mut fmt::Formatter) -> fmt::Result {
        let bytes = bytemuck::bytes_of(self);
        let mut list = fmt.debug_list();
        for b in bytes.iter() {
            list.entry(&format_args!("{:b}", b));
        }
        list.finish()
    }
}

fn fmt_bin(v: &u32, fmt: &mut fmt::Formatter) -> fmt::Result {
    v.fmt(fmt)
}

#[repr(C)]
#[derive(Derivative, Clone, Copy)]
#[derivative(Debug)]
pub struct RomHeader {
    #[derivative(Debug(format_with = "fmt_bin"))]
    entry_point: u32,
    #[derivative(Debug = "ignore")]
    nintendo_logo: [u8; 156],
    game_title: SmallAsciiString12,
    game_code: SmallAsciiString4,
    maker_code: SmallAsciiString2,
    fixed_value: u8,    // 96h
    main_unit_code: u8, // 00h for current GBA models
    device_type: u8,    // usually 00h
    #[derivative(Debug = "ignore")]
    _reserved1: [u8; 7], // should be zero filled
    software_version: u8, // usually 00h
    complement_check: u8, // header checksum
    #[derivative(Debug = "ignore")]
    _reserved2: [u8; 2], // should be zero filled
}

pub struct Memory {
    pub system_rom: [u8; 16 * 1024],
    pub wram_256: [u8; 256 * 1024],
    pub wram_32: [u8; 32 * 1024],
    pub io_regs: [u8; 16 * 1024],
    pub rom: Vec<u8>,
}

impl std::default::Default for Memory {
    fn default() -> Self {
        Memory {
            system_rom: [0; 16 * 1024],
            wram_256: [0; 256 * 1024],
            wram_32: [0; 32 * 1024],
            io_regs: [0; 16 * 1024],
            rom: Vec::new(),
        }
    }
}

#[derive(Debug)]
pub enum MemoryError {
    OutOfRange(DebugIsHex<u32>),
}

impl Memory {
    pub fn new() -> Self {
        Default::default()
    }

    pub fn with_rom(rom: Vec<u8>) -> Self {
        Memory {
            rom,
            ..Default::default()
        }
    }

    pub fn read_word(&self, address: u32) -> Result<u32, MemoryError> {
        let ptr = self.index(address)?;
        Ok(unsafe { *(ptr as *const u32) })
    }

    pub fn read_halfword(&self, addr: u32) -> Result<u16, MemoryError> {
        let ptr = self.index(addr)?;
        Ok(unsafe { *(ptr as *const u16) })
    }

    pub fn write_word(&mut self, address: u32, word: u32) -> Result<(), MemoryError> {
        let ptr = self.index_mut(address)?;
        unsafe {
            *(ptr as *mut u32) = word;
        }
        Ok(())
    }

    pub fn index(&self, address: u32) -> Result<*const u8, MemoryError> {
        unsafe {
            match address {
                0..=0x00003FFF => Ok(self.system_rom.as_ptr().offset(address as isize - 0)),
                0x02000000..=0x0203FFFF => {
                    Ok(self.wram_256.as_ptr().offset(address as isize - 0x02000000))
                }
                0x03000000..=0x03007FFF => {
                    Ok(self.wram_32.as_ptr().offset(address as isize - 0x03000000))
                }
                0x04000000..=0x04000FFF => {
                    Ok(self.io_regs.as_ptr().offset(address as isize - 0x04000000))
                }
                0x08000000..=0x09FFFFFF => {
                    Ok(self.rom.as_ptr().offset(address as isize - 0x08000000))
                }
                _ => Err(MemoryError::OutOfRange(hex!(address))),
            }
        }
    }

    pub fn index_mut(&mut self, address: u32) -> Result<*mut u8, MemoryError> {
        Ok(self.index(address)? as *mut u8)
    }
}

#[derive(Clone, Copy, Debug)]
pub enum ProcessorMode {
    User,
    FIQ,
    IRQ,
    Supervisor,
    Abort,
    Undefined,
    System,
}

fn full_bank_index(mode: ProcessorMode) -> usize {
    use ProcessorMode::*;
    match mode {
        User | System => 0,
        FIQ => 1,
        IRQ => 2,
        Supervisor => 3,
        Abort => 4,
        Undefined => 5,
    }
}

#[derive(Clone, Debug)]
pub struct Registers {
    unbanked: [u32; 8],
    single_banked: [[u32; 5]; 2],
    fully_banked: [[u32; 2]; 6],
    pc: u32,
}

impl Registers {
    pub fn zeroed() -> Self {
        Registers {
            unbanked: [0; 8],
            single_banked: [[0; 5]; 2],
            fully_banked: [[0; 2]; 6],
            pc: 0,
        }
    }

    pub fn index(&self, i: usize, mode: ProcessorMode) -> &u32 {
        match i {
            0..=7 => &self.unbanked[i],
            8..=12 => match mode {
                ProcessorMode::FIQ => &self.single_banked[1][i - 8],
                _ => &self.single_banked[0][i - 8],
            },
            13 | 14 => &self.fully_banked[full_bank_index(mode)][i - 13],
            15 => &self.pc,
            _ => panic!(),
        }
    }

    pub fn index_mut(&mut self, i: usize, mode: ProcessorMode) -> &mut u32 {
        match i {
            0..=7 => &mut self.unbanked[i],
            8..=12 => match mode {
                ProcessorMode::FIQ => &mut self.single_banked[1][i - 8],
                _ => &mut self.single_banked[0][i - 8],
            },
            13 | 14 => &mut self.fully_banked[full_bank_index(mode)][i - 13],
            15 => &mut self.pc,
            _ => panic!(),
        }
    }

    pub fn list(&self, mode: ProcessorMode) -> [u32; 16] {
        let mut out = [0; 16];

        for i in 0..16 {
            out[i] = *self.index(i, mode);
        }

        out
    }
}

#[derive(Clone, Copy, Default, Debug)]
pub struct SPSR(pub u32);

impl SPSR {
    pub fn thumb_mode(&self) -> bool {
        get_bit(self.0, 5)
    }

    pub fn set_thumb_mode(&mut self, t: bool) {
        self.0 = set_bits(self.0, 5, 1, t as u32)
    }

    pub fn disable_irq_interrupts(&self) -> bool {
        get_bit(self.0, 7)
    }

    pub fn set_disable_irq_interrupts(&mut self, i: bool) {
        self.0 = set_bits(self.0, 7, 1, i as u32)
    }

    pub fn disable_fiq_interrupts(&self) -> bool {
        get_bit(self.0, 6)
    }

    pub fn set_disable_fiq_interrupts(&mut self, f: bool) {
        self.0 = set_bits(self.0, 6, 1, f as u32)
    }

    pub fn carry(&self) -> bool {
        get_bit(self.0, 29)
    }

    pub fn set_carry(&mut self, c: bool) {
        self.0 = set_bits(self.0, 29, 1, c as u32)
    }

    pub fn negative(&self) -> bool {
        get_bit(self.0, 31)
    }

    pub fn set_negative(&mut self, n: bool) {
        self.0 = set_bits(self.0, 31, 1, n as u32)
    }

    pub fn zero(&self) -> bool {
        get_bit(self.0, 30)
    }

    pub fn set_zero(&mut self, z: bool) {
        self.0 = set_bits(self.0, 30, 1, z as u32)
    }

    pub fn overflow(&self) -> bool {
        get_bit(self.0, 28)
    }

    pub fn set_overflow(&mut self, o: bool) {
        self.0 = set_bits(self.0, 28, 1, o as u32)
    }
}

#[derive(Clone, Default, Debug)]
pub struct SPSRFlags {
    supervisor: SPSR,
    irq: SPSR,
    abort: SPSR,
}

impl SPSRFlags {
    pub fn new() -> Self {
        Default::default()
    }

    pub fn get_for(&self, mode: ProcessorMode) -> &SPSR {
        match mode {
            ProcessorMode::Supervisor => &self.supervisor,
            ProcessorMode::IRQ => &self.irq,
            ProcessorMode::Abort => &self.abort,
            _ => panic!(format!(
                "Attempted to retrieve SPSR for invalid mode: {:?}",
                mode
            )),
        }
    }

    pub fn get_for_mut(&mut self, mode: ProcessorMode) -> &mut SPSR {
        match mode {
            ProcessorMode::Supervisor => &mut self.supervisor,
            ProcessorMode::IRQ => &mut self.irq,
            ProcessorMode::Abort => &mut self.abort,
            _ => panic!(format!(
                "Attempted to retrieve SPSR for invalid mode: {:?}",
                mode
            )),
        }
    }
}

pub struct Emulator {
    pub memory: Box<Memory>,
    pub registers: Registers,
    pub processor_mode: ProcessorMode,
    pub spsr_flags: SPSRFlags,
    pub cpsr_flags: SPSR,
}

impl Emulator {
    pub fn program_counter(&self) -> u32 {
        *self.registers.index(15, self.processor_mode)
    }
    pub fn set_program_counter(&mut self, c: u32) {
        *self.registers.index_mut(15, self.processor_mode) = c;
    }

    pub fn fetch_thumb_instruction(&self, addr: u32) -> Option<ThumbInstruction> {
        let instruction = unsafe { *(self.memory.index(addr).unwrap() as *const u16) };
        parse_thumb_instruction(instruction)
    }

    pub fn fetch_arm_instruction(&self, addr: u32) -> Option<(CondFlags, Instruction)> {
        let instruction = unsafe { *(self.memory.index(addr).unwrap() as *const u32) };
        parse_instruction(instruction)
    }

    pub unsafe fn step(&mut self) {
        const LR: usize = 14;
        const SP: usize = 13;

        let pc = *self.registers.index(15, self.processor_mode);

        if self.cpsr_flags.thumb_mode() {
            let _registerPrinted = self
                .registers
                .list(self.processor_mode)
                .iter()
                .cloned()
                .map(|e| hex!(e))
                .collect::<Vec<_>>();

            let instruction = self.fetch_thumb_instruction(pc).unwrap();

            *self.registers.index_mut(15, self.processor_mode) = pc + 4;
            let pc = *self.registers.index(15, self.processor_mode);
            use ThumbInstruction::*;
            // do shit
            match instruction {
                AddSubtractRegister { is_sub, rm, rn, rd } => {
                    let rnval = *self.registers.index(rn as usize, self.processor_mode);
                    let rmval = *self.registers.index(rm as usize, self.processor_mode);

                    let r = if is_sub {
                        let r = rnval.wrapping_sub(rmval);
                        self.cpsr_flags.set_carry(r > rnval || r > rmval);
                        r
                    } else {
                        let r = rnval.wrapping_add(rmval);
                        self.cpsr_flags.set_carry(r < rnval || r < rmval);
                        // TODO: figure out how the fuck to do V
                        r
                    };

                    self.cpsr_flags.set_negative(get_bit(r, 31));
                    self.cpsr_flags.set_zero(r == 0);

                    *self.registers.index_mut(rd as usize, self.processor_mode) = r;
                }
                BranchExchange { link, rm_h2, sbz } => {
                    let addr = *self.registers.index(rm_h2 as usize, self.processor_mode);
                    if link {
                        *self.registers.index_mut(LR, self.processor_mode) = (pc - 2) | 1;
                    }
                    self.cpsr_flags.set_thumb_mode(get_bit(addr, 0));
                    *self.registers.index_mut(15, self.processor_mode) = addr & 0xFFFFFFFE;
                    return;
                }
                DataProcessingRegister { opcode, rm, rd } => {
                    let r = match opcode {
                        0b1110 => {
                            // BIC
                            let r = *self.registers.index(rd as usize, self.processor_mode)
                                & (!*self.registers.index(rm as usize, self.processor_mode));
                            *self.registers.index_mut(rd as usize, self.processor_mode) = r;
                            r
                        }
                        _ => unimplemented!(),
                    };

                    self.cpsr_flags.set_negative(get_bit(r, 31));
                    self.cpsr_flags.set_zero(r == 0);
                }
                Push(r_bit, r_list) => {
                    let r_count = r_list.count_ones() + r_bit as u32;
                    let mut start_address =
                        self.registers.index(13, self.processor_mode) - r_count * 4;

                    for i in 0..8 {
                        if get_bit(r_list, i) {
                            *(self.memory.index(start_address).unwrap() as *mut u32) =
                                *self.registers.index(i as usize, self.processor_mode);
                            start_address += 4;
                        }
                    }
                    if r_bit {
                        *(self.memory.index(start_address).unwrap() as *mut u32) =
                            *self.registers.index(14, self.processor_mode);
                    }

                    *self.registers.index_mut(13, self.processor_mode) -= 4 * r_count;
                }
                BranchLong { h, offset_11 } => match h {
                    0b10 => {
                        *self.registers.index_mut(14, self.processor_mode) =
                            ((pc as i32) + (sign_extend32(offset_11 as u32, 11) << 12)) as u32;
                    }
                    0b11 => {
                        let _pc = pc;
                        *self.registers.index_mut(15, self.processor_mode) =
                            *self.registers.index(14, self.processor_mode)
                                + (offset_11 << 1) as u32;
                        *self.registers.index_mut(14, self.processor_mode) = (_pc - 2) | 1;
                        return;
                    }
                    0b01 => unimplemented!(),
                    _ => unreachable!(),
                },
                ConditionalBranch { cond, offset } => {
                    let passed = match instruction::parse_cond_flags(cond) {
                        CondFlags::Always => true,
                        CondFlags::CarrySet => self.cpsr_flags.carry(),
                        CondFlags::Equal => self.cpsr_flags.zero(),
                        CondFlags::NotEqual => !self.cpsr_flags.zero(),
                        c => unimplemented!("{:?}", c),
                    };

                    if passed {
                        *self.registers.index_mut(15, self.processor_mode) =
                            (pc as i32 + (sign_extend32(offset as u32, 8) << 1)) as u32;
                        return;
                    }
                }
                SWI(_immed_8) => {
                    // set up return
                    *self.registers.index_mut(14, ProcessorMode::Supervisor) = pc - 2;
                    *self.spsr_flags.get_for_mut(ProcessorMode::Supervisor) = self.cpsr_flags;

                    self.cpsr_flags.0 = set_bits(self.cpsr_flags.0, 0, 5, 0b10011); // supervisor mode
                    self.cpsr_flags.set_thumb_mode(false);
                    self.cpsr_flags.set_disable_irq_interrupts(true);

                    // TOODO: Figure out high vectors?
                    *self.registers.index_mut(15, self.processor_mode) = 0x08;
                    self.processor_mode = ProcessorMode::Supervisor;
                    return;
                }
                ShiftByIMmediate {
                    opcode,
                    immediate,
                    rm,
                    rd,
                } => {
                    let rmval = *self.registers.index(rm as usize, self.processor_mode);
                    match opcode {
                    0b00 => { // Logical Shift Left
                        *self.registers.index_mut(rd as usize, self.processor_mode) =
                            rmval << immediate;
                        if immediate > 0 {
                            self.cpsr_flags
                                .set_carry(get_bit(rmval, 32 - immediate as usize));
                        }
                        self.cpsr_flags
                            .set_negative(get_bit(rmval << immediate, 31));
                        self.cpsr_flags.set_zero(rmval << immediate == 0);
                    }
                    0b10 => { // arithmetic shift right
                        let r = if immediate == 0 {
                            self.cpsr_flags.set_carry(get_bit(rmval, 31));
                            match get_bit(rmval, 31) {
                                true => 0xFFFFFFFF,
                                false => 0
                            }
                        } else {
                            use std::mem;
                            self.cpsr_flags.set_carry(get_bit(rmval, immediate as usize - 1));
                            mem::transmute(mem::transmute::<_, i32>(rmval).wrapping_shr(immediate as u32))
                        };
                        *self.registers.index_mut(rd as usize, self.processor_mode) = r;
                        self.cpsr_flags.set_negative(get_bit(r, 31));
                        self.cpsr_flags.set_zero(r == 0);
                    }
                    _ => unimplemented!(),
                }},
                LoadWordPCRelative { rd, immed_8 } => {
                    let addr = (pc & 0xFFFFFFFC) + immed_8 as u32 * 4;
                    *self.registers.index_mut(rd as usize, self.processor_mode) =
                        *(self.memory.index(addr).unwrap() as *mut u32);
                }
                LoadStoreRegisterOffset { opcode, rm, rd, rn } => {
                    let address = *self.registers.index(rn as usize, self.processor_mode) + *self.registers.index(rm as usize, self.processor_mode);

                    match opcode {
                        0b100 => { // LDR(2)
                            *self.registers.index_mut(rd as usize, self.processor_mode) = self.memory.read_word(address).unwrap();
                        },
                        _ => unimplemented!()
                    }
                }
                LoadStoreWordByteImmediateOffset { b, l, offset, rn, rd } => {
                    if b { unimplemented!() }

                    let address = *self.registers.index(rn as usize, self.processor_mode) + offset as u32 * 4;

                    match l {
                        true => *self.registers.index_mut(rd as usize, self.processor_mode) = self.memory.read_word(address).unwrap(),
                        false => self.memory.write_word(address, *self.registers.index(rd as usize, self.processor_mode)).unwrap(),
                    }
                }
                LoadStoreMultipleIncrementAfter {
                    l,
                    rn,
                    register_list,
                } => {
                    let start_address = *self.registers.index(rn as usize, self.processor_mode);
                    let mut address = start_address;

                    for i in 0..8 {
                        if !get_bit(register_list, i) {
                            continue;
                        }

                        match l {
                            true => {
                                *self.registers.index_mut(i as usize, self.processor_mode) =
                                    *(self.memory.index(address).unwrap() as *mut u32)
                            }
                            false => {
                                *(self.memory.index(address).unwrap() as *mut u32) =
                                    *self.registers.index_mut(i as usize, self.processor_mode)
                            }
                        }

                        address = address + 4;
                    }

                    if !get_bit(register_list, rn as usize) {
                        *self.registers.index_mut(rn as usize, self.processor_mode) +=
                            address - start_address;
                    }
                }
                Mov { h1, h2, rd, rm } => {
                    let rd = (h1 as u8) << 3 | rd;
                    let rm = (h2 as u8) << 3 | rm;

                    *self.registers.index_mut(rd as usize, self.processor_mode) =
                        *self.registers.index(rm as usize, self.processor_mode);
                }
                AddSubtractCmpMoveImmediate {
                    opcode,
                    rd,
                    immediate,
                } => {
                    let r = match opcode {
                        0b00 => {
                            *self.registers.index_mut(rd as usize, self.processor_mode) =
                                immediate as u32;
                            immediate as u32
                        }
                        0b11 => {
                            let rdval = *self.registers.index(rd as usize, self.processor_mode);
                            let r = rdval.wrapping_sub(immediate as u32);
                            self.cpsr_flags.set_carry(r > rdval || r > immediate as u32);
                            *self.registers.index_mut(rd as usize, self.processor_mode) = r;
                            r
                        }
                        _ => unimplemented!(),
                    };

                    self.cpsr_flags.set_negative(get_bit(r, 31));
                    self.cpsr_flags.set_zero(r == 0);
                }
                _ => unimplemented!(),
            }

            *self.registers.index_mut(15, self.processor_mode) = pc - 2;
            return;
        }

        //
        //  END OF THUMB MODE
        //

        let _registers_printed = self
            .registers
            .list(self.processor_mode)
            .iter()
            .cloned()
            .map(|e| hex!(e))
            .collect::<Vec<_>>();

        let (cond_flags, instruction) = self.fetch_arm_instruction(pc).unwrap();

        *self.registers.index_mut(15, self.processor_mode) = pc + 8;
        let pc = *self.registers.index(15, self.processor_mode);

        match cond_flags {
            CondFlags::Always => (),
            _ => unimplemented!(),
        };

        use Instruction::*;
        match instruction {
            Branch {
                link,
                signed_immed_24,
            } => {
                if link {
                    *self.registers.index_mut(LR, self.processor_mode) = pc - 4;
                }
                *self.registers.index_mut(15, self.processor_mode) =
                    (pc as i32 + sign_extend32(signed_immed_24 << 2, 26)) as u32;
                return;
            }
            BranchExchange { rm } => {
                let addr = *self.registers.index(rm as usize, self.processor_mode);
                self.cpsr_flags.set_thumb_mode(get_bit(addr, 0));
                // arm state
                *self.registers.index_mut(15, self.processor_mode) = addr & 0xFFFFFFFE;
                return;
            }
            DataProcessingImmediate {
                s,
                opcode,
                rn,
                rd,
                rotate,
                immediate,
            } => {
                let shifter_operand = (immediate as u32).rotate_right(rotate as u32 * 2);
                match opcode {
                    OpCode::MOV => {
                        *self.registers.index_mut(rd as usize, self.processor_mode) =
                            shifter_operand;
                    }
                    OpCode::ADD => {
                        if s {
                            unimplemented!()
                        }
                        *self.registers.index_mut(rd as usize, self.processor_mode) = self
                            .registers
                            .index(rn as usize, self.processor_mode)
                            .wrapping_add(shifter_operand);
                    }
                    _ => unimplemented!(),
                }
            }
            MoveRegisterToStatusRegister {
                r,
                field_mask,
                sbo: _,
                sbz: _,
                rm,
            } => {
                let operand = *self.registers.index(rm as usize, self.processor_mode);
                let psr = if r {
                    self.spsr_flags.get_for_mut(self.processor_mode)
                } else {
                    &mut self.cpsr_flags
                };

                for i in 0..4 {
                    if get_bit(field_mask, i) {
                        psr.0 = set_bits(psr.0, i * 8, 8, get_bits(operand, i * 8, 8));
                    }
                }
            }
            LoadStoreWordOrUnsignedByte_Immediate {
                p,
                u,
                b,
                w,
                l,
                rn,
                rd,
                offset_12,
            } => {
                if b {
                    unimplemented!()
                } // byte access

                let calc_new_addr = |base, offset: u32| {
                    if u {
                        base + offset
                    } else {
                        base - offset
                    }
                };

                let address = if p {
                    let base = *self.registers.index(rn as usize, self.processor_mode);
                    let address = calc_new_addr(base, offset_12 as u32);

                    // writeback
                    if w {
                        *self.registers.index_mut(rn as usize, self.processor_mode) = address;
                    }

                    address
                } else {
                    let address = *self.registers.index(rn as usize, self.processor_mode);
                    if w {
                        unimplemented!()
                    } // unpriviliged access?

                    // increment base register
                    *self.registers.index_mut(rn as usize, self.processor_mode) = calc_new_addr(
                        *self.registers.index(rn as usize, self.processor_mode),
                        offset_12 as u32,
                    );

                    // ret old address
                    address
                };

                if l {
                    *self.registers.index_mut(rd as usize, self.processor_mode) =
                        self.memory.read_word(address).unwrap();
                } else {
                    let val = *self.registers.index(rd as usize, self.processor_mode);
                    self.memory.write_word(address, val).unwrap();
                }
            }
            _ => unimplemented!(),
        }

        *self.registers.index_mut(15, self.processor_mode) = pc - 4;
    }
}
