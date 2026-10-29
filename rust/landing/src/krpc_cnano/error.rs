use crate::sys::*;
pub type krpc_error_t = ::core::ffi::c_int;
pub const KRPC_ERROR_BUFFER_TOO_SMALL: krpc_error_t = -8;
pub const KRPC_ERROR_DECODING_FAILED: krpc_error_t = -7;
pub const KRPC_ERROR_ENCODING_FAILED: krpc_error_t = -6;
pub const KRPC_ERROR_RPC_FAILED: krpc_error_t = -5;
pub const KRPC_ERROR_NO_RESULTS: krpc_error_t = -4;
pub const KRPC_ERROR_CONNECTION_FAILED: krpc_error_t = -3;
pub const KRPC_ERROR_EOF: krpc_error_t = -2;
pub const KRPC_ERROR_IO: krpc_error_t = -1;
pub const KRPC_OK: krpc_error_t = 0;
#[no_mangle]
pub unsafe extern "C" fn krpc_get_error(
    mut error: krpc_error_t,
) -> *const ::core::ffi::c_char {
    match error as ::core::ffi::c_int {
        -1 => return b"I/O failure\0" as *const u8 as *const ::core::ffi::c_char,
        -2 => return b"end of file\0" as *const u8 as *const ::core::ffi::c_char,
        -3 => return b"connection failed\0" as *const u8 as *const ::core::ffi::c_char,
        -4 => return b"no results\0" as *const u8 as *const ::core::ffi::c_char,
        -5 => return b"rpc failed\0" as *const u8 as *const ::core::ffi::c_char,
        -6 => return b"encoding failed\0" as *const u8 as *const ::core::ffi::c_char,
        -7 => return b"decoding failed\0" as *const u8 as *const ::core::ffi::c_char,
        -8 => return b"buffer too small\0" as *const u8 as *const ::core::ffi::c_char,
        _ => return b"\0" as *const u8 as *const ::core::ffi::c_char,
    };
}
