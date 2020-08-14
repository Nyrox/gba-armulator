use crate::bitutils::*;

#[derive(Copy, Clone, Debug)]
pub enum CondFlags {
    Always,
}

#[derive(Copy, Clone, Debug)]
pub enum OpCode {
    AND,
    EOR,
    SUB,
    RSB,
    ADD,
    ADC,
    SBC,
    RSC,
    TST,
    TEQ,
    CMP,
    CMN,
    ORR,
    MOV,
    BIC,
    MVN,
}

impl OpCode {
    pub fn new(val: u8) -> Result<Self, ()> {
        use OpCode::*;

        if val > 0b1111 {
            Err(())
        } else {
            Ok([
                AND, EOR, SUB, RSB, ADD, ADC, SBC, RSC, TST, TEQ, CMP, CMN, ORR, MOV, BIC, MVN,
            ][val as usize])
        }
    }
}

#[derive(Copy, Clone, Debug)]
pub enum ShiftType {
    LeftShiftLogical,
}

impl ShiftType {
    pub fn new(val: u8) -> Result<Self, ()> {
        match val {
            0b000 => Ok(ShiftType::LeftShiftLogical),
            _ => Err(()),
        }
    }
}

#[derive(Copy, Clone, Debug)]
pub enum Instruction {
    DataProcessingImmediateShift {
        s: bool,
        opcode: OpCode,
        rn: u8,
        rd: u8,
        shift_imm: u8,
        shift: ShiftType,
        rm: u8,
    },
    DataProcessingImmediate {
        s: bool,
        opcode: OpCode,
        rn: u8,
        rd: u8,
        rotate: u8,
        immediate: u8,
    },
    Branch {
        link: bool,
        signed_immed_24: u32,
    },
    MoveImmediateToStatusRegister {
        r: bool,
        field_mask: u8,
        sbo: u8,
        rotate_imm: u8,
        immediate: u8,
    },
    MoveRegisterToStatusRegister {
        r: bool,
        field_mask: u8,
        sbo: u8,
        sbz: u8,
        rm: u8,
    },
    LoadStoreWordOrUnsignedByte_Immediate {
        p: bool,
        u: bool,
        b: bool,
        w: bool,
        l: bool,
        rn: u8,
        rd: u8,
        offset_12: u16,
    },
}

pub fn parse_instruction(instr: u32) -> (CondFlags, Instruction) {
    use Instruction::*;

    let opblock = get_bits(instr, 20, 8);
    let cond_block = get_bits(instr, 28, 4);

    dbg!(hex!(instr));

    let cond_flags = match cond_block {
        0b1110 => CondFlags::Always,
        _ => {
            dbg!(hex!(cond_block));
            unimplemented!()
        }
    };

    let op = match opblock {
        // 000 block

        // MSR Register
        _ if opblock & 0xFB == 0x12 && get_bits(instr, 4, 4) == 0 => MoveRegisterToStatusRegister {
            r: get_bit(instr, 22),
            field_mask: get_bits(instr, 16, 4) as u8,
            sbo: get_bits(instr, 12, 4) as u8,
            sbz: get_bits(instr, 8, 4) as u8,
            rm: get_bits(instr, 0, 4) as u8,
        },
        // rule out misc. instruction
        _ if opblock & 0xF9 == 0x10 => unimplemented!(),
        _ if opblock & 0x80 == 0x0 && get_bit(instr, 4) && get_bit(instr, 7) => unimplemented!(),

        // data processing immediate shift
        0b00000000..=0b00011111 if get_bit(instr, 4) => DataProcessingImmediateShift {
            opcode: OpCode::new(get_bits(instr, 21, 4) as u8).unwrap(),
            s: get_bit(instr, 20),
            rn: get_bits(instr, 16, 4) as u8,
            rd: get_bits(instr, 12, 4) as u8,
            shift_imm: get_bits(instr, 7, 4) as u8,
            shift: ShiftType::new(get_bits(instr, 4, 3) as u8).unwrap(),
            rm: get_bits(instr, 0, 4) as u8,
        },

        // 001 block
        // move immediate to status register
        _ if opblock & 0xFB == 0x32 => MoveImmediateToStatusRegister {
            r: get_bit(instr, 22),
            field_mask: get_bits(instr, 16, 4) as u8,
            sbo: get_bits(instr, 12, 4) as u8,
            rotate_imm: get_bits(instr, 8, 4) as u8,
            immediate: get_bits(instr, 0, 8) as u8,
        },
        // undefined instruction
        _ if opblock & 0xFB == 0x30 => unimplemented!(),

        // data processing immediate
        0b00100000..=0b00111111 => DataProcessingImmediate {
            opcode: OpCode::new(get_bits(instr, 21, 4) as u8).unwrap(),
            s: get_bit(instr, 20),
            rn: get_bits(instr, 16, 4) as u8,
            rd: get_bits(instr, 12, 4) as u8,
            rotate: get_bits(instr, 8, 4) as u8,
            immediate: get_bits(instr, 0, 8) as u8,
        },

        // load and stores
        0b01000000..=0b01011111 => LoadStoreWordOrUnsignedByte_Immediate {
            p: get_bit(instr, 24),
            u: get_bit(instr, 23),
            b: get_bit(instr, 22),
            w: get_bit(instr, 21),
            l: get_bit(instr, 20),
            rn: get_bits(instr, 16, 4) as u8,
            rd: get_bits(instr, 12, 4) as u8,
            offset_12: get_bits(instr, 0, 12) as u16,
        },

        // branches
        0b10100000..=0b10111111 => Branch {
            link: get_bit(instr, 24),
            signed_immed_24: get_bits(instr, 0, 24),
        },

        _ => {
            dbg!(hex!(instr));
            dbg!(hex!(opblock));
            unimplemented!()
        }
    };

    (cond_flags, op)
}
