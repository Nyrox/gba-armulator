pub mod instruction;
pub mod thumb;


pub mod prelude {
    use super::*;
    
    pub use instruction::{Instruction, parse_instruction, CondFlags, OpCode, ShiftType};
    pub use thumb::{ThumbInstruction, parse_thumb_instruction};
}