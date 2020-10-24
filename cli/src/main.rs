#![feature(box_syntax)]


use emucore;
use emucore::*;
use std::fs;
use std::mem;
use std::io::{stdout, Write};
use std::io;

use crossterm::{
    execute,
    style::{Color, Print, ResetColor, SetBackgroundColor, SetForegroundColor},
    ExecutableCommand,
    event,
};

fn main() {
    unsafe { _main() }
}

unsafe fn _main(){
    // let mut rom = fs::read(r"./roms/Pokemon - Leaf Green Version (U) (V1.1).gba").unwrap();
    // let mut rom = fs:read(r"./roms/Pokemon - Sapphire Version (U) (V1.1).gba").unwrap();
    let mut rom = fs::read(r"./armwrestler-gba-fixed.gba").unwrap();

    // load bios
    // let mut bios = fs::read(r"./gba_bios.bin").unwrap();
    // assert_eq!(bios.len(), 16 * 1024);
    // for i in 0..bios.len() {
    //     *memory.index_mut(i as u32).unwrap() = bios[i];
    // }

    let mut emulator = Emulator {
        memory: box Memory::with_rom(rom),
        registers: Registers::zeroed(),
        processor_mode: ProcessorMode::User,
        spsr_flags: SPSRFlags::new(),
        cpsr_flags: SPSR(0),
    };

    assert_eq!(mem::size_of::<RomHeader>(), 192);
    let header = unsafe { *(emulator.memory.rom.as_ptr() as *const RomHeader) };

    println!("{:#?}", header);

    emulator.set_program_counter(0x08000000);

	loop {
		let mut input = String::new();
		io::stdin().read_line(&mut input).unwrap();

		match input {

			_ => {
				println!("Unrecognized input");
				continue;
			}
		}
	}


}
