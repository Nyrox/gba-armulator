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