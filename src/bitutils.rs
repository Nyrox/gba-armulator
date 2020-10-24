use num_traits::{One, Zero};

pub fn get_bit<T>(num: T, bit: usize) -> bool
where
    T: num_traits::PrimInt,
{
    let b = (num >> bit) & One::one();
    b == One::one()
}

pub fn get_bits<T>(num: T, offset: usize, len: usize) -> T
where
    T: num_traits::PrimInt,
{
    (num >> offset) & !(!T::zero() << len)
}

pub fn set_bits<T>(num: T, offset: usize, len: usize, newval: T) -> T
where
    T: num_traits::PrimInt,
{
    // for masking off the newval to the len we want
    let outer_mask = !(!T::zero() << len);
    // for masking out the region we want to set
    let inner_mask = outer_mask << offset;

    (num & (!inner_mask)) | (newval & outer_mask) << offset
}

pub fn sign_extend32(data: u32, size: u32) -> i32 {
    assert!(size > 0 && size <= 32);
    ((data << (32 - size)) as i32) >> (32 - size)
}

pub struct DebugIsHex<T> {
    pub inner: T,
}

impl<T> std::fmt::Debug for DebugIsHex<T>
where
    T: std::fmt::LowerHex,
{
    fn fmt(&self, f: &mut std::fmt::Formatter) -> std::fmt::Result {
        f.write_fmt(format_args!("0x{:x}", self.inner))
    }
}

#[macro_export]
macro_rules! hex {
    ($id: ident) => {
        DebugIsHex { inner: $id }
    };
}

pub use hex;

pub struct DebugIsBin<T> {
    pub inner: T,
}

impl<T> std::fmt::Debug for DebugIsBin<T>
where
    T: std::fmt::Binary,
{
    fn fmt(&self, f: &mut std::fmt::Formatter) -> std::fmt::Result {
        f.write_fmt(format_args!("0b{:b}", self.inner))
    }
}

#[macro_export]
macro_rules! bin {
    ($id: ident) => {
        DebugIsBin { inner: $id }
    };
}

pub use bin;
