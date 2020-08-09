
use crate::bitutils::*;


#[derive(Clone, Debug)]
pub enum ThumbInstruction {
    Push(bool, u8),
    Pop(bool, u8),
    SWI(u8),
    BranchLong { h: u8, offset_11: u16 },
    Mov { h1: bool, h2: bool, rm: u8, rd: u8 },
    MovImmed { rd: u8, immed: u8 },
    Undefined,
}

pub fn parse_thumb_instruction(instr: u16) -> ThumbInstruction {
    use ThumbInstruction::*;

    let opcode = (instr >> 8) as u8;
    let arg_byte = instr as u8;

    match opcode {
        0b10110100 => Push(false, arg_byte),
        0b10110101 => Push(true, arg_byte),
        0b10111100 => Pop(false, arg_byte),
        0b10111101 => Pop(true, arg_byte),
        0b11011111 => SWI(arg_byte),
        // mov immed
        0b00100000 .. 0b00100111 => MovImmed { rd: opcode & 0x07, immed: arg_byte },
        // BL, BLX
        0b11100000 .. 0b11111111 => BranchLong {
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
        _ => Undefined,
    }
}