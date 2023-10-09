pub fn get_bit(byte: u8, n: u8) -> u8 {
    (byte >> n) & 1
}

pub fn get_bits(mut byte: u8, bit_start: u8, length: u8) -> u8 {
    let mask_shift: u8 = if bit_start < length { 0 } else { bit_start - length + 1 };
    let mask: u8 = ((1 << length) - 1) << mask_shift;
    byte &= mask;
    byte >>= mask_shift;
    byte
}

pub fn set_bit(byte: &mut u8, n: u8, enable: bool) {
    if enable {
        *byte |= 1_u8 << n;
    } else {
        *byte &= !(1_u8 << n);
    }
}

pub fn set_bits(byte: &mut u8, bit_start: u8, length: u8, mut data: u8) {
    let mask_shift: u8 = if bit_start < length { 0 } else { bit_start - length + 1 };
    let mask: u8 = ((1 << length) - 1) << mask_shift;
    data <<= mask_shift;
    data &= mask;
    *byte &= !(mask);
    *byte |= data;
}