#![feature(box_syntax)]

use emucore;
use emucore::*;
use std::fs;
use std::io;
use std::io::{stdin, stdout, Write};
use std::mem;

use emucore::bitutils::DebugIsHex;

fn main() {
    unsafe { _main() }
}

unsafe fn _main() {
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

    println!("{:#?}\n\n", header);

    emulator.set_program_counter(0x08000000);

    loop {
        let pc = emulator.program_counter();

        print!("\n");
        print!(
            "Execution Mode: {}\n",
            match emulator.cpsr_flags.thumb_mode() {
                true => "Thumb",
                false => "ARM",
            }
        );
        match emulator.cpsr_flags.thumb_mode() {
            true => print!(
                "Next instruction: {:?}\nParsed: {:#?}\n",
                hex!(emulator.memory.read_halfword(pc).unwrap()),
                emulator.fetch_thumb_instruction(pc)
            ),
            false => print!(
                "Next instruction: {:?}\nParsed: {:#?}\n",
                hex!(emulator.memory.read_word(pc).unwrap()),
                emulator.fetch_arm_instruction(pc)
            ),
        };

        println!();
        print!("PC: {:?}\n", hex!(pc));
        print!("Command: ");
        stdout().flush().unwrap();
        let mut buf = String::new();
        stdin().read_line(&mut buf).unwrap();

        match &buf.as_str()[..(buf.len() - 2)] {
            // skip newline
            "exit" | "quit" | "q" | "stop" => {
                return;
            }
            "step" => {
                emulator.step();
            }
            s if &s[..4] == "step" => {
                if let Ok(n) = s[4..].trim().parse::<u32>() {
                    for _ in 0..n {
                        emulator.step();
                    }
                } else {
                    println!(
                        "Expected an integer literal after 'step', found: {}",
                        &s[..4]
                    );
                    continue;
                }
            }
            _ => {
                println!("Unrecognized input");
                continue;
            }
        }
    }
}
