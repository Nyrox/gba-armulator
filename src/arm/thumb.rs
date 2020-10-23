use crate::bitutils::*;

#[derive(Clone, Debug)]
pub enum ThumbInstruction {
    Push(bool, u8),
    Pop(bool, u8),
    SWI(u8),
    ShiftByIMmediate { opcode: u8, immediate: u8, rm: u8, rd: u8 },
    ConditionalBranch { cond: u8, offset: u8 },
    BranchLong { h: u8, offset_11: u16 },
    Mov { h1: bool, h2: bool, rm: u8, rd: u8 },
    LoadWordPCRelative { rd: u8, immed_8: u8 },
    LoadStoreHalfwordImmediateOffset { l: bool, offset: u8, rn: u8, rd: u8 },
    LoadStoreMultipleIncrementAfter { l: bool, rn: u8, register_list: u8 },
    AddSubtractRegister { is_sub: bool, rm: u8, rn: u8, rd: u8 },
    AddSubtractImmediate { is_sub: bool, immediate: u8, rn: u8, rd: u8 },
    AddSubtractCmpMoveImmediate { opcode: u8, rd: u8, immediate: u8 },
    DataProcessingRegister { opcode: u8, rm: u8, rd: u8 },
    Undefined,
}

pub fn parse_thumb_instruction(instr: u16) -> ThumbInstruction {
    use ThumbInstruction::*;

    let opcode = (instr >> 8) as u8;
    let arg_byte = instr as u8;

    match opcode {
        0b00000000..=0b00010111 => ShiftByIMmediate {
            opcode: get_bits(instr, 11, 2) as u8,
            immediate: get_bits(instr, 6, 5) as u8,
            rm: get_bits(instr, 3, 3) as u8,
            rd: get_bits(instr, 0, 3) as u8,
        },
        0b00011000..=0b00011011 => AddSubtractRegister {
            is_sub: get_bit(instr, 9),
            rm: get_bits(instr, 6, 3) as u8,
            rn: get_bits(instr, 3, 3) as u8,
            rd: get_bits(instr, 0, 3) as u8,
        },
        0b00011100..=0b00011111 => AddSubtractImmediate {
            is_sub: get_bit(instr, 9),
            immediate: get_bits(instr, 6, 3) as u8,
            rn: get_bits(instr, 3, 3) as u8,
            rd: get_bits(instr, 0, 3) as u8,
        },
        0b00100000..=0b00111111 => AddSubtractCmpMoveImmediate {
            opcode: get_bits(instr, 11, 2) as u8,
            rd: get_bits(instr, 8, 3) as u8,
            immediate: get_bits(instr, 0, 8) as u8,
        },
        0b01000000..=0b01000011 => DataProcessingRegister {
            opcode: get_bits(instr, 6, 4) as u8 ,
            rm: get_bits(instr, 3, 3) as u8,
            rd: get_bits(instr, 0, 3) as u8,
        },
        0b10110100 => Push(false, arg_byte),
        0b10110101 => Push(true, arg_byte),
        0b10111100 => Pop(false, arg_byte),
        0b10111101 => Pop(true, arg_byte),
        0b11011111 => SWI(arg_byte),
        0b01001000..=0b01001111 => LoadWordPCRelative {
            rd: get_bits(instr, 8, 3) as u8,
            immed_8: get_bits(instr, 0, 8) as u8,
        },
        0b10000000..=0b10001111 => LoadStoreHalfwordImmediateOffset {
            l: get_bit(instr, 11),
            offset: get_bits(instr, 6, 5) as u8,
            rn: get_bits(instr, 3, 3) as u8,
            rd: get_bits(instr, 3, 3) as u8,
        },
        0b11000000..=0b11001111 => LoadStoreMultipleIncrementAfter {
            l: get_bit(instr, 11),
            rn: get_bits(instr, 8, 3) as u8,
            register_list: get_bits(instr, 0, 8) as u8,
        },
        0b11010000..0b11011111 => ConditionalBranch {
            cond: get_bits(instr, 8, 2) as u8,
            offset: get_bits(instr, 0, 8) as u8,
        },
        // BL, BLX
        0b11100000..0b11111111 => BranchLong {
            h: get_bits(instr, 11, 2) as u8,
            offset_11: get_bits(instr, 0, 11) as u16,
        },
        // format 8
        0b01000110 => Mov {
            h1: get_bit(arg_byte, 7),
            h2: get_bit(arg_byte, 6),
            rm: get_bits(arg_byte, 3, 3),
            rd: get_bits(arg_byte, 0, 3),
        },
        _ => unimplemented!(),
    }
}
