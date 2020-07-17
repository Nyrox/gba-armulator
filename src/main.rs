#![feature(const_generics)]
#![feature(box_syntax)]
#![feature(exclusive_range_pattern)]

#![allow(incomplete_features)]

use std::fs;
use std::io::prelude::*;
use bytemuck;
use std::mem;
use derivative::Derivative;
use std::fmt;

#[derive(Copy, Clone)]
struct SmallAsciiString<const N: usize> {
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

impl<T> FormatBinary for T where T: bytemuck::Pod {
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
struct RomHeader {
    #[derivative(Debug(format_with="fmt_bin"))]
    entry_point: u32,
    #[derivative(Debug="ignore")]
    nintendo_logo: [u8; 156],
    game_title: SmallAsciiString12,
    game_code: SmallAsciiString4,
    maker_code: SmallAsciiString2,
    fixed_value: u8, // 96h
    main_unit_code: u8, // 00h for current GBA models
    device_type: u8, // usually 00h
    #[derivative(Debug="ignore")]
    _reserved1: [u8; 7], // should be zero filled
    software_version: u8, // usually 00h
    complement_check: u8, // header checksum
    #[derivative(Debug="ignore")]
    _reserved2: [u8; 2], // should be zero filled
}


fn sign_extend32(data: u32, size: u32) -> i32 {
    assert!(size > 0 && size <= 32);
    ((data << (32 - size)) as i32) >> (32 - size)
}


struct DebugIsHex<T> {
    inner: T,
}

impl<T> std::fmt::Debug for DebugIsHex<T> where T: std::fmt::LowerHex {
    fn fmt(&self, f: &mut std::fmt::Formatter) -> std::fmt::Result { 
        f.write_fmt(format_args!("0x{:x}", self.inner))
    }
}

macro_rules! hex {
    ($id: ident) => {
        DebugIsHex { inner: $id }
    };
}


struct Memory {
    pub system_rom: [u8; 16 * 1024],
    pub wram_256: [u8; 256 * 1024],
    pub wram_32: [u8; 32 * 1024],
    pub rom: Vec<u8>,
}

impl std::default::Default for Memory {
    fn default() -> Self {
        Memory {
            system_rom: [0; 16 * 1024],
            wram_256: [0; 256 * 1024],
            wram_32: [0; 32 * 1024],
            rom: Vec::new(),
        }
    }
}

#[derive(Debug)]
enum MemoryError {
    OutOfRange(DebugIsHex<u32>),
}

impl Memory {
    pub fn new() -> Self {
        Default::default()
    }

    pub fn with_rom(rom: Vec<u8>) -> Self {
        Memory { rom, ..Default::default() }
    }

    pub fn index(&self, address: u32) -> Result<*const u8, MemoryError> {
        unsafe {
            match address {
                0..0x00003FFF => {
                    Ok(self.system_rom.as_ptr().offset(address as isize - 0))
                },
                0x03000000..0x03007FFF => {
                    Ok(self.wram_32.as_ptr().offset(address as isize - 0x03000000))
                }
                0x08000000..0x09FFFFFF => {
                    Ok(self.rom.as_ptr().offset(address as isize - 0x08000000))
                }
                _ => Err(MemoryError::OutOfRange(hex!(address)))
            }
        }
    }

    pub fn index_mut(&mut self, address: u32) -> Result<*mut u8, MemoryError> {
        Ok(self.index(address)? as *mut u8)
    }
}

fn main() {
    let mut rom: Vec<u8> = {
        let mut rom_buffer = Vec::new();
        let mut rom_file = fs::File::open(r"./roms/Pokemon - Leaf Green Version (U) (V1.1).gba").unwrap();
        // let mut rom_file = fs::File::open(r"./roms/Pokemon - Sapphire Version (U) (V1.1).gba").unwrap();
        rom_file.read_to_end(&mut rom_buffer).unwrap();
        rom_buffer
    };

    let mut memory = box Memory::with_rom(rom);
    
    assert_eq!(mem::size_of::<RomHeader>(), 192);
    let header = unsafe {
        *(memory.rom.as_ptr() as *const RomHeader)
    };
    
    println!("{:#?}", header);

    let mut registers: [u32; 16] = [0; 16];
    let mut cpsr_flags: [u8; 4] = [0; 4];
    let mut spsr_flags: [u8; 4] = [0; 4];

    let mut pc: &mut u32 = unsafe {
        let ptr: *mut u32 = &mut registers[15] as *mut u32;
        &mut *ptr
    };

    *pc = 0x08000000;
    loop {
        print!("\n");
        let instruction = unsafe {
            *(memory.index(*pc).unwrap() as *const u32)
        };

        *pc = *pc + 8;
        

        let _registerPrinted = registers.iter().map(|e| hex!(e)).collect::<Vec<_>>();
        dbg!(_registerPrinted);
        dbg!(cpsr_flags);
        dbg!(format!("0x{:x}", (*pc) - 8));

        let instr_bytes = bytemuck::bytes_of(&instruction);
        
        let cond_block = instr_bytes[3] & 0b11110000;

        dbg!(format!("{:#b}", instruction));

        let op_block = (instruction & 0x0FF00000) >> 20 as u8;
        dbg!(format!("{:08b}", op_block));

        let cond = match cond_block >> 4 {
            0b1110 => true,
            _ => {
                println!("{:b}", cond_block);
                unimplemented!()
            }
        };

        if !cond { panic!() }


        fn get_bit(num: u32, bit: u32) -> bool {
            let b = (num >> bit) & 1;
            b == 1
        }

        fn get_bits(num: u32, offset: u32, len: u32) -> u32 {
            (num >> offset) & !(!0 << len)
        }        

        match op_block {
            // branch
            _ if ((op_block & 0xF0) >> 4) == 0b1010 => {
                dbg!("branch");
                if (cond_block & 0b0001) != 0 {
                    dbg!("linked branch");
                }
                let offset = instruction & 0x00FFFFFF;
                *pc = (*pc as i32 + sign_extend32(offset << 2, 26)) as u32;
                continue;
            },
            // move immediate to status register
            _ if (op_block & 0xFB) == 0x32 => {
                dbg!("move immediate to status register");
                unimplemented!();
            }
            // after the previous we know, this bit pattern is data-processing immediate or undefined
            _ if (op_block & 0xE0) == 0x20 => {
                dbg!("data processing immediate");

                let opcode = get_bits(instruction, 21, 4);
                let s_flag = op_block & 0x001;

                dbg!(opcode, s_flag);

                match opcode {
                    // mov
                    0b1101 => {
                        let immed_8 = get_bits(instruction, 0, 8);
                        let rotate = get_bits(instruction, 8, 4);

                        let rd = get_bits(instruction, 12, 4);

                        if s_flag != 0 { unimplemented!() }

                        registers[rd as usize] = immed_8 >> rotate;
                    },
                    // add
                    0b0100 => {
                        if s_flag != 0 { unimplemented!() }

                        let immed_8 = get_bits(instruction, 0, 8);
                        let rotate = get_bits(instruction, 8, 4);

                        let rd = get_bits(instruction, 12, 4);
                        let rn = get_bits(instruction, 16, 4);

                        dbg!(rd, rn, immed_8, rotate);

                        registers[rd as usize] = registers[rn as usize] + (immed_8 >> rotate);
                    }
                    _ => unimplemented!()
                }
            },
            // move register to status register with register operand
            _ if (instruction & 0x0FB00010) == 0x01200000 => {
                dbg!("MSR - Reg");

                let r_bit = get_bit(instruction, 22);
                dbg!(r_bit);

                let field_mask = (instruction & 0xF0000) >> 16;
                dbg!(field_mask);

                let operand = registers[(instruction & 0xF) as usize];

                if r_bit { unimplemented!() }

                for i in 0..=3 {
                    if get_bit(field_mask, i as u32) {
                        cpsr_flags[i] = ((operand >> (i * 8)) & 0xFF) as u8;
                    }
                }
            }
            // some kind of load/store
            _ if (op_block & 0xC0) == 0x40 => {
                
                // immediate
                let immediate = !get_bit(instruction, 25);
                let load = get_bit(instruction, 20);
                let is_byte = get_bit(instruction, 22);
                
                let rd = get_bits(instruction, 12, 4);
                let rn = get_bits(instruction, 16, 4);

                if is_byte { unimplemented!() }
                if !immediate { unimplemented!() }                

                let offset = get_bits(instruction, 0, 12);
                let base = registers[rn as usize];

                let address = match get_bit(instruction, 23) {
                    true => base + offset,
                    false => base - offset,
                };

                let mem_loc = memory.index_mut(address).unwrap() as *mut u32;
                unsafe {
                    if load {
                        registers[rd as usize] = *mem_loc;
                    }
                    else{
                        *mem_loc = registers[rd as usize];
                    }
                }
            } 
            _ => unimplemented!()
        }

        *pc = *pc - 4;
    }
}