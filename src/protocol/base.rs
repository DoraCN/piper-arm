//! Protocol base helpers: byte <-> integer conversions (big-endian),
//! matching the protocol's byte order and signedness rules.

use crate::error::{Error, Result};

/// Read a slice of `data[start..end]` as a big-endian unsigned integer.
///
/// Read a big-endian byte slice as an unsigned integer.
pub fn bytes_to_int(data: &[u8], start: usize, end: usize) -> Result<u32> {
    if end > data.len() || start > end {
        return Err(Error::UnexpectedFrameLength {
            id: 0,
            len: data.len(),
        });
    }
    let mut v: u32 = 0;
    for &b in &data[start..end] {
        v = (v << 8) | u32::from(b);
    }
    Ok(v)
}

/// Interpret a big-endian `u32` as an `i32`.
pub fn as_signed(v: u32) -> i32 {
    v as i32
}

/// Convert a raw unsigned 8-bit value into a (possibly signed) 8-bit value.
pub fn to_signed_8(v: u32, signed: bool) -> i32 {
    let v = v & 0xFF;
    if signed && v & 0x80 != 0 {
        (v as i32) - 0x100
    } else {
        v as i32
    }
}

/// Convert a raw unsigned 16-bit value into a (possibly signed) 16-bit value.
pub fn to_signed_16(v: u32, signed: bool) -> i32 {
    let v = v & 0xFFFF;
    if signed && v & 0x8000 != 0 {
        (v as i32) - 0x10000
    } else {
        v as i32
    }
}

/// Convert a raw unsigned 32-bit value into a signed 32-bit value.
pub fn to_signed_32(v: u32) -> i32 {
    v as i32
}

/// Convert an integer value into a list of bytes in big-endian order.
///
/// `signed` controls the value range validation; negative signed values are
/// stored as two's complement.
pub fn int_to_bytes(value: i64, nbytes: usize, signed: bool) -> Result<[u8; 8]> {
    let (min, max) = if signed {
        match nbytes {
            1 => (i64::from(i8::MIN), i64::from(i8::MAX)),
            2 => (i64::from(i16::MIN), i64::from(i16::MAX)),
            4 => (i64::from(i32::MIN), i64::from(i32::MAX)),
            _ => (0, u32::MAX as i64),
        }
    } else {
        match nbytes {
            1 => (0, 0xFF),
            2 => (0, 0xFFFF),
            4 => (0, u32::MAX as i64),
            _ => (0, u32::MAX as i64),
        }
    };
    if value < min || value > max {
        return Err(Error::ValueError(format!(
            "value {value} out of range [{min}, {max}] for {nbytes} byte(s) signed={signed}"
        )));
    }
    let mut out = [0u8; 8];
    let mask: u64 = if nbytes == 8 { u64::MAX } else { (1u64 << (nbytes * 8)) - 1 };
    let v = (value as u64) & mask;
    for i in 0..nbytes {
        out[nbytes - 1 - i] = ((v >> (8 * i)) & 0xFF) as u8;
    }
    Ok(out)
}

/// Convert a 16-bit integer into two big-endian bytes.
pub fn u16_to_bytes(value: u16) -> [u8; 2] {
    [(value >> 8) as u8, value as u8]
}

/// Convert a 32-bit integer into four big-endian bytes.
pub fn i32_to_bytes(value: i32) -> [u8; 4] {
    let v = value as u32;
    [
        (v >> 24) as u8,
        (v >> 16) as u8,
        (v >> 8) as u8,
        v as u8,
    ]
}

/// Convert a float to an unsigned integer representation for the MIT
/// pass-through mode: `int((x - offset) * (2^bits - 1) / (x_max - x_min))`.
pub fn float_to_uint(x: f64, x_min: f64, x_max: f64, bits: u32) -> u64 {
    let span = x_max - x_min;
    ((x - x_min) * ((1u64 << bits) - 1) as f64 / span) as u64
}
