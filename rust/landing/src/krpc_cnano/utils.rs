use crate::sys::*;
pub type __int32_t = i32;
pub type __uint32_t = u32;
pub type __uint64_t = u64;
pub type int32_t = __int32_t;
pub type uint32_t = __uint32_t;
pub type uint64_t = __uint64_t;
#[no_mangle]
pub unsafe extern "C" fn krpc_float64_to_float32(
    mut float64: *mut ::core::ffi::c_void,
    mut float32: *mut ::core::ffi::c_void,
) {
    let mut data: uint64_t = *(float64 as *mut uint64_t);
    let mut sign: uint64_t = data >> 32 as ::core::ffi::c_int & 0x80000000 as uint64_t;
    let mut exp: uint64_t = data >> 52 as ::core::ffi::c_int & 0x7ff as uint64_t;
    let mut man: uint64_t = data >> 29 as ::core::ffi::c_int & 0x7fffff as uint64_t;
    let mut expValue: int32_t = exp.wrapping_sub(1023 as uint64_t) as int32_t;
    if expValue == 1024 as int32_t {
        expValue = 128 as ::core::ffi::c_int as int32_t;
    } else if expValue < -(127 as int32_t) {
        expValue = -(127 as ::core::ffi::c_int) as int32_t;
    } else if expValue > 128 as int32_t {
        expValue = 128 as ::core::ffi::c_int as int32_t;
        man = 0 as uint64_t;
    }
    exp = ((expValue + 127 as int32_t) << 23 as ::core::ffi::c_int) as uint64_t;
    *(float32 as *mut uint32_t) = (sign | exp | man) as uint32_t;
}
#[no_mangle]
pub unsafe extern "C" fn krpc_float32_to_float64(
    mut float32: *mut ::core::ffi::c_void,
    mut float64: *mut ::core::ffi::c_void,
) {
    let mut data: uint64_t = *(float32 as *mut uint32_t) as uint64_t;
    let mut sign: uint64_t = (data >> 31 as ::core::ffi::c_int & 0x1 as uint64_t)
        << 63 as ::core::ffi::c_int;
    let mut exp: uint64_t = data >> 23 as ::core::ffi::c_int & 0xff as uint64_t;
    let mut man: uint64_t = (data & 0x7fffff as uint64_t) << 29 as ::core::ffi::c_int;
    let mut expValue: int32_t = exp.wrapping_sub(127 as uint64_t) as int32_t;
    if expValue == -(127 as int32_t) {
        expValue = -(1023 as ::core::ffi::c_int) as int32_t;
    }
    if expValue == 128 as int32_t {
        expValue = 1024 as ::core::ffi::c_int as int32_t;
    }
    exp = (expValue + 1023 as int32_t) as uint64_t;
    exp <<= 52 as ::core::ffi::c_int;
    *(float64 as *mut uint64_t) = sign | exp | man;
}
