pub mod instruction;
pub mod thumb;

pub mod prelude {
    use super::*;

    pub use instruction::{parse_instruction, CondFlags, Instruction, OpCode, ShiftType};
    pub use thumb::{parse_thumb_instruction, ThumbInstruction};
}
