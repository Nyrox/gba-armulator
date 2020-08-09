pub mod instruction;
pub mod thumb;


pub mod prelude {
    use super::*;
    
    pub use instruction::Instruction;
    pub use thumb::{ThumbInstruction, parse_thumb_instruction};
}