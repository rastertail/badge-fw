pub const fn expand(pattern_hi: usize, pattern_lo: usize) -> [u8; 256] {
    let mut table = [0b10001000u8; 256];

    let mut i = 0;
    while i < 256 {
        if i & pattern_hi > 0 {
            table[i] |= 0b01000000;
        }
        if i & pattern_lo > 0 {
            table[i] |= 0b00000100;
        }
        i += 1;
    }

    table
}

pub const LOOKUP_A: [u8; 256] = expand(0b10000000, 0b01000000);
pub const LOOKUP_B: [u8; 256] = expand(0b00100000, 0b00010000);
pub const LOOKUP_C: [u8; 256] = expand(0b00001000, 0b00000100);
pub const LOOKUP_D: [u8; 256] = expand(0b00000010, 0b00000001);
