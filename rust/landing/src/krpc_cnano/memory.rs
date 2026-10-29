use crate::sys::*;
pub type size_t = usize;
pub type __uint8_t = u8;
pub type uint8_t = __uint8_t;
pub const __ASSERT_FUNCTION: [::core::ffi::c_char; 52] = unsafe {
    ::core::mem::transmute::<
        [u8; 52],
        [::core::ffi::c_char; 52],
    >(*b"void *krpc_recalloc(void *, size_t, size_t, size_t)\0")
};
#[no_mangle]
pub unsafe extern "C" fn krpc_malloc(mut size: size_t) -> *mut ::core::ffi::c_void {
    return malloc(size);
}
#[no_mangle]
pub unsafe extern "C" fn krpc_calloc(
    mut num: size_t,
    mut size: size_t,
) -> *mut ::core::ffi::c_void {
    return calloc(num, size);
}
#[no_mangle]
pub unsafe extern "C" fn krpc_recalloc(
    mut ptr: *mut ::core::ffi::c_void,
    mut num: size_t,
    mut inc: size_t,
    mut size: size_t,
) -> *mut ::core::ffi::c_void {
    if inc > 0 as size_t {} else {
        __assert_fail(
            b"inc > 0\0" as *const u8 as *const ::core::ffi::c_char,
            b"../ThirdParty/krpc-cnano-0.6.0/src/memory.c\0" as *const u8
                as *const ::core::ffi::c_char,
            14 as ::core::ffi::c_uint,
            __ASSERT_FUNCTION.as_ptr(),
        );
    };
    ptr = realloc(ptr, num.wrapping_add(inc).wrapping_mul(size));
    memset(
        (ptr as *mut uint8_t).offset(num.wrapping_mul(size) as isize)
            as *mut ::core::ffi::c_void,
        0 as ::core::ffi::c_int,
        inc.wrapping_mul(size),
    );
    return ptr;
}
#[no_mangle]
pub unsafe extern "C" fn krpc_free(mut ptr: *mut ::core::ffi::c_void) {
    free(ptr);
}
