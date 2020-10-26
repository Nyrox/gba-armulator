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

    use std::collections::HashSet;
    let mut breakpoints: HashSet<u32> = HashSet::new();

    let mut skip_output = false;

    loop {
        let pc = emulator.program_counter();

        print!("\n\n---\n");

        if !skip_output {
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

            println!("Breakpoints: {}", breakpoints.len());
            println!();
            print!("PC: {:?}\n", hex!(pc));
        }
        print!("Command: ");
        skip_output = false;

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
            "status" => {
                continue;
            }
            "breakpoints" => {
                skip_output = true;

                println!("Breakpoints:");
                for b in breakpoints.iter() {
                    println!("\t{:?}", hex!(b));
                }

                continue;
            }
            s if s.starts_with("break") => {
                skip_output = true;

                let parse = |s: &str| {
                    if s.starts_with("0x") {
                        return u32::from_str_radix(&s[2..], 16);
                    } else if s.starts_with("0b") {
                        return u32::from_str_radix(&s[2..], 2);
                    } else {
                        return s.parse();
                    }
                };

                if let Ok(n) = parse(s[5..].trim()) {
                    if breakpoints.contains(&n) {
                        breakpoints.remove(&n);
                        println!("Removed breakpoint: {:?}", hex!(n));
                    } else {
                        breakpoints.insert(n);
                        println!("Added breakpoint: {:?}", hex!(n));
                    }
                } else {
                    println!(
                        "Expected an integer literal after 'break', found {}",
                        &s[5..]
                    );
                }
            }
            s if s.starts_with("step") => {
                if let Ok(n) = s[4..].trim().parse::<u32>() {
                    for _ in 0..n {
                        emulator.step();
                        if breakpoints.contains(&emulator.program_counter()) {
                            println!(
                                "Encountered breakpoint: [{:?}]",
                                hex!(emulator.program_counter())
                            );
                            break;
                        }
                    }
                } else {
                    println!(
                        "Expected an integer literal after 'step', found: {}",
                        &s[5..]
                    );
                    continue;
                }
            }
            _ => {
                skip_output = true;
                println!("Unrecognized input");
                continue;
            }
        }
    }
}
