use crate::sys::*;
extern "C" {
    fn pb_field_iter_begin(
        iter: *mut pb_field_iter_t,
        desc: *const pb_msgdesc_t,
        message: *mut ::core::ffi::c_void,
    ) -> bool;
    fn pb_field_iter_begin_extension(
        iter: *mut pb_field_iter_t,
        extension: *mut pb_extension_t,
    ) -> bool;
    fn pb_field_iter_next(iter: *mut pb_field_iter_t) -> bool;
    fn pb_field_iter_find(iter: *mut pb_field_iter_t, tag: uint32_t) -> bool;
    fn pb_field_iter_find_extension(iter: *mut pb_field_iter_t) -> bool;
}
pub type __int8_t = i8;
pub type __uint8_t = u8;
pub type __int16_t = i16;
pub type __uint16_t = u16;
pub type __int32_t = i32;
pub type __uint32_t = u32;
pub type __int64_t = i64;
pub type __uint64_t = u64;
pub type __int_least8_t = __int8_t;
pub type __uint_least8_t = __uint8_t;
pub type __int_least16_t = __int16_t;
pub type __uint_least16_t = __uint16_t;
pub type int32_t = __int32_t;
pub type int64_t = __int64_t;
pub type uint8_t = __uint8_t;
pub type uint32_t = __uint32_t;
pub type uint64_t = __uint64_t;
pub type int_least8_t = __int_least8_t;
pub type int_least16_t = __int_least16_t;
pub type uint_least8_t = __uint_least8_t;
pub type uint_least16_t = __uint_least16_t;
pub type uint_fast8_t = ::core::ffi::c_uchar;
pub type size_t = usize;
pub type pb_byte_t = uint8_t;
pub type pb_type_t = pb_byte_t;
pub type pb_size_t = uint_least16_t;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct pb_istream_s {
    pub callback: Option<
        unsafe extern "C" fn(*mut pb_istream_t, *mut pb_byte_t, size_t) -> bool,
    >,
    pub state: *mut ::core::ffi::c_void,
    pub bytes_left: size_t,
    pub errmsg: *const ::core::ffi::c_char,
}
pub type pb_istream_t = pb_istream_s;
pub type pb_ostream_t = pb_ostream_s;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct pb_field_iter_s {
    pub descriptor: *const pb_msgdesc_t,
    pub message: *mut ::core::ffi::c_void,
    pub index: pb_size_t,
    pub field_info_index: pb_size_t,
    pub required_field_index: pb_size_t,
    pub submessage_index: pb_size_t,
    pub tag: pb_size_t,
    pub data_size: pb_size_t,
    pub array_size: pb_size_t,
    pub type_0: pb_type_t,
    pub pField: *mut ::core::ffi::c_void,
    pub pData: *mut ::core::ffi::c_void,
    pub pSize: *mut ::core::ffi::c_void,
    pub submsg_desc: *const pb_msgdesc_t,
}
pub type pb_msgdesc_t = pb_msgdesc_s;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct pb_msgdesc_s {
    pub field_info: *const uint32_t,
    pub submsg_info: *const *const pb_msgdesc_t,
    pub default_value: *const pb_byte_t,
    pub field_callback: Option<
        unsafe extern "C" fn(
            *mut pb_istream_t,
            *mut pb_ostream_t,
            *const pb_field_iter_t,
        ) -> bool,
    >,
    pub field_count: pb_size_t,
    pub required_field_count: pb_size_t,
    pub largest_tag: pb_size_t,
}
pub type pb_field_iter_t = pb_field_iter_s;
pub type pb_field_t = pb_field_iter_t;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct pb_bytes_array_s {
    pub size: pb_size_t,
    pub bytes: [pb_byte_t; 1],
}
pub type pb_bytes_array_t = pb_bytes_array_s;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct pb_callback_s {
    pub funcs: C2RustUnnamed,
    pub arg: *mut ::core::ffi::c_void,
}
#[derive(Copy, Clone)]
#[repr(C)]
pub union C2RustUnnamed {
    pub decode: Option<
        unsafe extern "C" fn(
            *mut pb_istream_t,
            *const pb_field_t,
            *mut *mut ::core::ffi::c_void,
        ) -> bool,
    >,
    pub encode: Option<
        unsafe extern "C" fn(
            *mut pb_ostream_t,
            *const pb_field_t,
            *const *mut ::core::ffi::c_void,
        ) -> bool,
    >,
}
pub type pb_callback_t = pb_callback_s;
pub type pb_wire_type_t = ::core::ffi::c_uint;
pub const PB_WT_PACKED: pb_wire_type_t = 255;
pub const PB_WT_32BIT: pb_wire_type_t = 5;
pub const PB_WT_STRING: pb_wire_type_t = 2;
pub const PB_WT_64BIT: pb_wire_type_t = 1;
pub const PB_WT_VARINT: pb_wire_type_t = 0;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct pb_extension_type_s {
    pub decode: Option<
        unsafe extern "C" fn(
            *mut pb_istream_t,
            *mut pb_extension_t,
            uint32_t,
            pb_wire_type_t,
        ) -> bool,
    >,
    pub encode: Option<
        unsafe extern "C" fn(*mut pb_ostream_t, *const pb_extension_t) -> bool,
    >,
    pub arg: *const ::core::ffi::c_void,
}
pub type pb_extension_t = pb_extension_s;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct pb_extension_s {
    pub type_0: *const pb_extension_type_t,
    pub dest: *mut ::core::ffi::c_void,
    pub next: *mut pb_extension_t,
    pub found: bool,
}
pub type pb_extension_type_t = pb_extension_type_s;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct pb_fields_seen_t {
    pub bitfield: [uint32_t; 2],
}
#[derive(Copy, Clone)]
#[repr(C)]
pub union C2RustUnnamed_0 {
    pub state: *mut ::core::ffi::c_void,
    pub c_state: *const ::core::ffi::c_void,
}
#[derive(Copy, Clone)]
#[repr(C)]
pub union C2RustUnnamed_1 {
    pub fixed64: uint64_t,
    pub bytes: [pb_byte_t; 8],
}
#[derive(Copy, Clone)]
#[repr(C)]
pub union C2RustUnnamed_2 {
    pub fixed32: uint32_t,
    pub bytes: [pb_byte_t; 4],
}
pub const true_0: ::core::ffi::c_int = 1 as ::core::ffi::c_int;
pub const false_0: ::core::ffi::c_int = 0 as ::core::ffi::c_int;
pub const NULL: *mut ::core::ffi::c_void = ::core::ptr::null_mut::<
    ::core::ffi::c_void,
>();
pub const PB_MAX_REQUIRED_FIELDS: ::core::ffi::c_int = 64 as ::core::ffi::c_int;
pub const PB_LTYPE_BOOL: ::core::ffi::c_uint = 0;
pub const PB_LTYPE_VARINT: ::core::ffi::c_uint = 1;
pub const PB_LTYPE_UVARINT: ::core::ffi::c_uint = 0x2 as ::core::ffi::c_uint;
pub const PB_LTYPE_SVARINT: ::core::ffi::c_uint = 0x3 as ::core::ffi::c_uint;
pub const PB_LTYPE_FIXED32: ::core::ffi::c_uint = 4;
pub const PB_LTYPE_FIXED64: ::core::ffi::c_uint = 5;
pub const PB_LTYPE_LAST_PACKABLE: ::core::ffi::c_uint = 0x5 as ::core::ffi::c_uint;
pub const PB_LTYPE_BYTES: ::core::ffi::c_uint = 6;
pub const PB_LTYPE_STRING: ::core::ffi::c_uint = 7;
pub const PB_LTYPE_SUBMESSAGE: ::core::ffi::c_uint = 8;
pub const PB_LTYPE_SUBMSG_W_CB: ::core::ffi::c_uint = 0x9 as ::core::ffi::c_uint;
pub const PB_LTYPE_EXTENSION: ::core::ffi::c_uint = 0xa as ::core::ffi::c_uint;
pub const PB_LTYPE_FIXED_LENGTH_BYTES: ::core::ffi::c_uint = 11;
pub const PB_LTYPE_MASK: ::core::ffi::c_uint = 0xf as ::core::ffi::c_uint;
pub const PB_HTYPE_REQUIRED: ::core::ffi::c_uint = 0;
pub const PB_HTYPE_OPTIONAL: ::core::ffi::c_uint = 0x10 as ::core::ffi::c_uint;
pub const PB_HTYPE_REPEATED: ::core::ffi::c_uint = 0x20 as ::core::ffi::c_uint;
pub const PB_HTYPE_ONEOF: ::core::ffi::c_uint = 0x30 as ::core::ffi::c_uint;
pub const PB_HTYPE_MASK: ::core::ffi::c_uint = 0x30 as ::core::ffi::c_uint;
pub const PB_ATYPE_STATIC: ::core::ffi::c_uint = 0 as ::core::ffi::c_uint;
pub const PB_ATYPE_POINTER: ::core::ffi::c_uint = 128;
pub const PB_ATYPE_CALLBACK: ::core::ffi::c_uint = 64;
pub const PB_ATYPE_MASK: ::core::ffi::c_uint = 0xc0 as ::core::ffi::c_uint;
pub const PB_SIZE_MAX: pb_size_t = -(1 as ::core::ffi::c_int) as pb_size_t;
pub const PB_DECODE_NOINIT: ::core::ffi::c_uint = 0x1 as ::core::ffi::c_uint;
pub const PB_DECODE_DELIMITED: ::core::ffi::c_uint = 0x2 as ::core::ffi::c_uint;
pub const PB_DECODE_NULLTERMINATED: ::core::ffi::c_uint = 0x4 as ::core::ffi::c_uint;
unsafe extern "C" fn buf_read(
    mut stream: *mut pb_istream_t,
    mut buf: *mut pb_byte_t,
    mut count: size_t,
) -> bool {
    let mut source: *const pb_byte_t = (*stream).state as *const pb_byte_t;
    (*stream).state = ((*stream).state as *mut pb_byte_t).offset(count as isize)
        as *mut ::core::ffi::c_void;
    if !buf.is_null() {
        memcpy(
            buf as *mut ::core::ffi::c_void,
            source as *const ::core::ffi::c_void,
            count.wrapping_mul(::core::mem::size_of::<pb_byte_t>() as size_t),
        );
    }
    return true_0 != 0;
}
#[no_mangle]
pub unsafe extern "C" fn pb_read(
    mut stream: *mut pb_istream_t,
    mut buf: *mut pb_byte_t,
    mut count: size_t,
) -> bool {
    if count == 0 as size_t {
        return true_0 != 0;
    }
    if buf.is_null()
        && (*stream).callback
            != Some(
                buf_read
                    as unsafe extern "C" fn(
                        *mut pb_istream_t,
                        *mut pb_byte_t,
                        size_t,
                    ) -> bool,
            )
    {
        let mut tmp: [pb_byte_t; 16] = [0; 16];
        while count > 16 as size_t {
            if !pb_read(stream, &raw mut tmp as *mut pb_byte_t, 16 as size_t) {
                return false_0 != 0;
            }
            count = count.wrapping_sub(16 as size_t);
        }
        return pb_read(stream, &raw mut tmp as *mut pb_byte_t, count);
    }
    if (*stream).bytes_left < count {
        (*stream).errmsg = (if !(*stream).errmsg.is_null() {
            (*stream).errmsg
        } else {
            b"end-of-stream\0" as *const u8 as *const ::core::ffi::c_char
        });
        return false_0 != 0;
    }
    if !(*stream).callback.expect("non-null function pointer")(stream, buf, count) {
        (*stream).errmsg = (if !(*stream).errmsg.is_null() {
            (*stream).errmsg
        } else {
            b"io error\0" as *const u8 as *const ::core::ffi::c_char
        });
        return false_0 != 0;
    }
    if (*stream).bytes_left < count {
        (*stream).bytes_left = 0 as size_t;
    } else {
        (*stream).bytes_left = (*stream).bytes_left.wrapping_sub(count);
    }
    return true_0 != 0;
}
unsafe extern "C" fn pb_readbyte(
    mut stream: *mut pb_istream_t,
    mut buf: *mut pb_byte_t,
) -> bool {
    if (*stream).bytes_left == 0 as size_t {
        (*stream).errmsg = (if !(*stream).errmsg.is_null() {
            (*stream).errmsg
        } else {
            b"end-of-stream\0" as *const u8 as *const ::core::ffi::c_char
        });
        return false_0 != 0;
    }
    if !(*stream).callback.expect("non-null function pointer")(stream, buf, 1 as size_t)
    {
        (*stream).errmsg = (if !(*stream).errmsg.is_null() {
            (*stream).errmsg
        } else {
            b"io error\0" as *const u8 as *const ::core::ffi::c_char
        });
        return false_0 != 0;
    }
    (*stream).bytes_left = (*stream).bytes_left.wrapping_sub(1);
    return true_0 != 0;
}
#[no_mangle]
pub unsafe extern "C" fn pb_istream_from_buffer(
    mut buf: *const pb_byte_t,
    mut msglen: size_t,
) -> pb_istream_t {
    let mut stream: pb_istream_t = pb_istream_s {
        callback: None,
        state: ::core::ptr::null_mut::<::core::ffi::c_void>(),
        bytes_left: 0,
        errmsg: ::core::ptr::null::<::core::ffi::c_char>(),
    };
    let mut state: C2RustUnnamed_0 = C2RustUnnamed_0 {
        state: ::core::ptr::null_mut::<::core::ffi::c_void>(),
    };
    stream.callback = Some(
        buf_read
            as unsafe extern "C" fn(*mut pb_istream_t, *mut pb_byte_t, size_t) -> bool,
    ) as Option<unsafe extern "C" fn(*mut pb_istream_t, *mut pb_byte_t, size_t) -> bool>;
    state.c_state = buf as *const ::core::ffi::c_void;
    stream.state = state.state;
    stream.bytes_left = msglen;
    stream.errmsg = ::core::ptr::null::<::core::ffi::c_char>();
    return stream;
}
unsafe extern "C" fn pb_decode_varint32_eof(
    mut stream: *mut pb_istream_t,
    mut dest: *mut uint32_t,
    mut eof: *mut bool,
) -> bool {
    let mut byte: pb_byte_t = 0;
    let mut result: uint32_t = 0;
    if !pb_readbyte(stream, &raw mut byte) {
        if (*stream).bytes_left == 0 as size_t {
            if !eof.is_null() {
                *eof = true_0 != 0;
            }
        }
        return false_0 != 0;
    }
    if byte as ::core::ffi::c_int & 0x80 as ::core::ffi::c_int == 0 as ::core::ffi::c_int
    {
        result = byte as uint32_t;
    } else {
        let mut bitpos: uint_fast8_t = 7 as uint_fast8_t;
        result = (byte as ::core::ffi::c_int & 0x7f as ::core::ffi::c_int) as uint32_t;
        loop {
            if !pb_readbyte(stream, &raw mut byte) {
                return false_0 != 0;
            }
            if bitpos as ::core::ffi::c_int >= 32 as ::core::ffi::c_int {
                let mut sign_extension: pb_byte_t = (if (bitpos as ::core::ffi::c_int)
                    < 63 as ::core::ffi::c_int
                {
                    0xff as ::core::ffi::c_int
                } else {
                    0x1 as ::core::ffi::c_int
                }) as pb_byte_t;
                let mut valid_extension: bool = byte as ::core::ffi::c_int
                    & 0x7f as ::core::ffi::c_int == 0 as ::core::ffi::c_int
                    || result >> 31 as ::core::ffi::c_int != 0 as uint32_t
                        && byte as ::core::ffi::c_int
                            == sign_extension as ::core::ffi::c_int;
                if bitpos as ::core::ffi::c_int >= 64 as ::core::ffi::c_int
                    || !valid_extension
                {
                    (*stream).errmsg = (if !(*stream).errmsg.is_null() {
                        (*stream).errmsg
                    } else {
                        b"varint overflow\0" as *const u8 as *const ::core::ffi::c_char
                    });
                    return false_0 != 0;
                }
            } else if bitpos as ::core::ffi::c_int == 28 as ::core::ffi::c_int {
                if byte as ::core::ffi::c_int & 0x70 as ::core::ffi::c_int
                    != 0 as ::core::ffi::c_int
                    && byte as ::core::ffi::c_int & 0x78 as ::core::ffi::c_int
                        != 0x78 as ::core::ffi::c_int
                {
                    (*stream).errmsg = (if !(*stream).errmsg.is_null() {
                        (*stream).errmsg
                    } else {
                        b"varint overflow\0" as *const u8 as *const ::core::ffi::c_char
                    });
                    return false_0 != 0;
                }
                result
                    |= ((byte as ::core::ffi::c_int & 0xf as ::core::ffi::c_int)
                        as uint32_t) << bitpos as ::core::ffi::c_int;
            } else {
                result
                    |= ((byte as ::core::ffi::c_int & 0x7f as ::core::ffi::c_int)
                        as uint32_t) << bitpos as ::core::ffi::c_int;
            }
            bitpos = (bitpos as ::core::ffi::c_int + 7 as ::core::ffi::c_int)
                as uint_fast8_t;
            if !(byte as ::core::ffi::c_int & 0x80 as ::core::ffi::c_int != 0) {
                break;
            }
        }
    }
    *dest = result;
    return true_0 != 0;
}
#[no_mangle]
pub unsafe extern "C" fn pb_decode_varint32(
    mut stream: *mut pb_istream_t,
    mut dest: *mut uint32_t,
) -> bool {
    return pb_decode_varint32_eof(stream, dest, ::core::ptr::null_mut::<bool>());
}
#[no_mangle]
pub unsafe extern "C" fn pb_decode_varint(
    mut stream: *mut pb_istream_t,
    mut dest: *mut uint64_t,
) -> bool {
    let mut byte: pb_byte_t = 0;
    let mut bitpos: uint_fast8_t = 0 as uint_fast8_t;
    let mut result: uint64_t = 0 as uint64_t;
    loop {
        if !pb_readbyte(stream, &raw mut byte) {
            return false_0 != 0;
        }
        if bitpos as ::core::ffi::c_int >= 63 as ::core::ffi::c_int
            && byte as ::core::ffi::c_int & 0xfe as ::core::ffi::c_int
                != 0 as ::core::ffi::c_int
        {
            (*stream).errmsg = (if !(*stream).errmsg.is_null() {
                (*stream).errmsg
            } else {
                b"varint overflow\0" as *const u8 as *const ::core::ffi::c_char
            });
            return false_0 != 0;
        }
        result
            |= ((byte as ::core::ffi::c_int & 0x7f as ::core::ffi::c_int) as uint64_t)
                << bitpos as ::core::ffi::c_int;
        bitpos = (bitpos as ::core::ffi::c_int + 7 as ::core::ffi::c_int)
            as uint_fast8_t;
        if !(byte as ::core::ffi::c_int & 0x80 as ::core::ffi::c_int != 0) {
            break;
        }
    }
    *dest = result;
    return true_0 != 0;
}
unsafe extern "C" fn pb_skip_varint(mut stream: *mut pb_istream_t) -> bool {
    let mut byte: pb_byte_t = 0;
    loop {
        if !pb_read(stream, &raw mut byte, 1 as size_t) {
            return false_0 != 0;
        }
        if !(byte as ::core::ffi::c_int & 0x80 as ::core::ffi::c_int != 0) {
            break;
        }
    }
    return true_0 != 0;
}
unsafe extern "C" fn pb_skip_string(mut stream: *mut pb_istream_t) -> bool {
    let mut length: uint32_t = 0;
    if !pb_decode_varint32(stream, &raw mut length) {
        return false_0 != 0;
    }
    if length as size_t != length as size_t {
        (*stream).errmsg = (if !(*stream).errmsg.is_null() {
            (*stream).errmsg
        } else {
            b"size too large\0" as *const u8 as *const ::core::ffi::c_char
        });
        return false_0 != 0;
    }
    return pb_read(stream, ::core::ptr::null_mut::<pb_byte_t>(), length as size_t);
}
#[no_mangle]
pub unsafe extern "C" fn pb_decode_tag(
    mut stream: *mut pb_istream_t,
    mut wire_type: *mut pb_wire_type_t,
    mut tag: *mut uint32_t,
    mut eof: *mut bool,
) -> bool {
    let mut temp: uint32_t = 0;
    *eof = false_0 != 0;
    *wire_type = PB_WT_VARINT;
    *tag = 0 as uint32_t;
    if !pb_decode_varint32_eof(stream, &raw mut temp, eof) {
        return false_0 != 0;
    }
    *tag = temp >> 3 as ::core::ffi::c_int;
    *wire_type = (temp & 7 as uint32_t) as pb_wire_type_t;
    return true_0 != 0;
}
#[no_mangle]
pub unsafe extern "C" fn pb_skip_field(
    mut stream: *mut pb_istream_t,
    mut wire_type: pb_wire_type_t,
) -> bool {
    match wire_type as ::core::ffi::c_uint {
        0 => return pb_skip_varint(stream),
        1 => return pb_read(stream, ::core::ptr::null_mut::<pb_byte_t>(), 8 as size_t),
        2 => return pb_skip_string(stream),
        5 => return pb_read(stream, ::core::ptr::null_mut::<pb_byte_t>(), 4 as size_t),
        _ => {
            (*stream).errmsg = (if !(*stream).errmsg.is_null() {
                (*stream).errmsg
            } else {
                b"invalid wire_type\0" as *const u8 as *const ::core::ffi::c_char
            });
            return false_0 != 0;
        }
    };
}
unsafe extern "C" fn read_raw_value(
    mut stream: *mut pb_istream_t,
    mut wire_type: pb_wire_type_t,
    mut buf: *mut pb_byte_t,
    mut size: *mut size_t,
) -> bool {
    let mut max_size: size_t = *size;
    match wire_type as ::core::ffi::c_uint {
        0 => {
            *size = 0 as size_t;
            loop {
                *size = (*size).wrapping_add(1);
                if *size > max_size {
                    (*stream).errmsg = (if !(*stream).errmsg.is_null() {
                        (*stream).errmsg
                    } else {
                        b"varint overflow\0" as *const u8 as *const ::core::ffi::c_char
                    });
                    return false_0 != 0;
                }
                if !pb_read(stream, buf, 1 as size_t) {
                    return false_0 != 0;
                }
                let fresh0 = buf;
                buf = buf.offset(1);
                if !(*fresh0 as ::core::ffi::c_int & 0x80 as ::core::ffi::c_int != 0) {
                    break;
                }
            }
            return true_0 != 0;
        }
        1 => {
            *size = 8 as size_t;
            return pb_read(stream, buf, 8 as size_t);
        }
        5 => {
            *size = 4 as size_t;
            return pb_read(stream, buf, 4 as size_t);
        }
        2 | _ => {
            (*stream).errmsg = (if !(*stream).errmsg.is_null() {
                (*stream).errmsg
            } else {
                b"invalid wire_type\0" as *const u8 as *const ::core::ffi::c_char
            });
            return false_0 != 0;
        }
    };
}
#[no_mangle]
pub unsafe extern "C" fn pb_make_string_substream(
    mut stream: *mut pb_istream_t,
    mut substream: *mut pb_istream_t,
) -> bool {
    let mut size: uint32_t = 0;
    if !pb_decode_varint32(stream, &raw mut size) {
        return false_0 != 0;
    }
    *substream = *stream;
    if (*substream).bytes_left < size as size_t {
        (*stream).errmsg = (if !(*stream).errmsg.is_null() {
            (*stream).errmsg
        } else {
            b"parent stream too short\0" as *const u8 as *const ::core::ffi::c_char
        });
        return false_0 != 0;
    }
    (*substream).bytes_left = size as size_t;
    (*stream).bytes_left = (*stream).bytes_left.wrapping_sub(size as size_t);
    return true_0 != 0;
}
#[no_mangle]
pub unsafe extern "C" fn pb_close_string_substream(
    mut stream: *mut pb_istream_t,
    mut substream: *mut pb_istream_t,
) -> bool {
    if (*substream).bytes_left != 0 {
        if !pb_read(
            substream,
            ::core::ptr::null_mut::<pb_byte_t>(),
            (*substream).bytes_left,
        ) {
            return false_0 != 0;
        }
    }
    (*stream).state = (*substream).state;
    (*stream).errmsg = (*substream).errmsg;
    return true_0 != 0;
}
unsafe extern "C" fn decode_basic_field(
    mut stream: *mut pb_istream_t,
    mut wire_type: pb_wire_type_t,
    mut field: *mut pb_field_iter_t,
) -> bool {
    match (*field).type_0 as ::core::ffi::c_uint & PB_LTYPE_MASK {
        PB_LTYPE_BOOL => {
            if wire_type as ::core::ffi::c_uint
                != PB_WT_VARINT as ::core::ffi::c_int as ::core::ffi::c_uint
                && wire_type as ::core::ffi::c_uint
                    != PB_WT_PACKED as ::core::ffi::c_int as ::core::ffi::c_uint
            {
                (*stream).errmsg = (if !(*stream).errmsg.is_null() {
                    (*stream).errmsg
                } else {
                    b"wrong wire type\0" as *const u8 as *const ::core::ffi::c_char
                });
                return false_0 != 0;
            }
            return pb_dec_bool(stream, field);
        }
        PB_LTYPE_VARINT | PB_LTYPE_UVARINT | PB_LTYPE_SVARINT => {
            if wire_type as ::core::ffi::c_uint
                != PB_WT_VARINT as ::core::ffi::c_int as ::core::ffi::c_uint
                && wire_type as ::core::ffi::c_uint
                    != PB_WT_PACKED as ::core::ffi::c_int as ::core::ffi::c_uint
            {
                (*stream).errmsg = (if !(*stream).errmsg.is_null() {
                    (*stream).errmsg
                } else {
                    b"wrong wire type\0" as *const u8 as *const ::core::ffi::c_char
                });
                return false_0 != 0;
            }
            return pb_dec_varint(stream, field);
        }
        PB_LTYPE_FIXED32 => {
            if wire_type as ::core::ffi::c_uint
                != PB_WT_32BIT as ::core::ffi::c_int as ::core::ffi::c_uint
                && wire_type as ::core::ffi::c_uint
                    != PB_WT_PACKED as ::core::ffi::c_int as ::core::ffi::c_uint
            {
                (*stream).errmsg = (if !(*stream).errmsg.is_null() {
                    (*stream).errmsg
                } else {
                    b"wrong wire type\0" as *const u8 as *const ::core::ffi::c_char
                });
                return false_0 != 0;
            }
            return pb_decode_fixed32(stream, (*field).pData);
        }
        PB_LTYPE_FIXED64 => {
            if wire_type as ::core::ffi::c_uint
                != PB_WT_64BIT as ::core::ffi::c_int as ::core::ffi::c_uint
                && wire_type as ::core::ffi::c_uint
                    != PB_WT_PACKED as ::core::ffi::c_int as ::core::ffi::c_uint
            {
                (*stream).errmsg = (if !(*stream).errmsg.is_null() {
                    (*stream).errmsg
                } else {
                    b"wrong wire type\0" as *const u8 as *const ::core::ffi::c_char
                });
                return false_0 != 0;
            }
            return pb_decode_fixed64(stream, (*field).pData);
        }
        PB_LTYPE_BYTES => {
            if wire_type as ::core::ffi::c_uint
                != PB_WT_STRING as ::core::ffi::c_int as ::core::ffi::c_uint
            {
                (*stream).errmsg = (if !(*stream).errmsg.is_null() {
                    (*stream).errmsg
                } else {
                    b"wrong wire type\0" as *const u8 as *const ::core::ffi::c_char
                });
                return false_0 != 0;
            }
            return pb_dec_bytes(stream, field);
        }
        PB_LTYPE_STRING => {
            if wire_type as ::core::ffi::c_uint
                != PB_WT_STRING as ::core::ffi::c_int as ::core::ffi::c_uint
            {
                (*stream).errmsg = (if !(*stream).errmsg.is_null() {
                    (*stream).errmsg
                } else {
                    b"wrong wire type\0" as *const u8 as *const ::core::ffi::c_char
                });
                return false_0 != 0;
            }
            return pb_dec_string(stream, field);
        }
        PB_LTYPE_SUBMESSAGE | PB_LTYPE_SUBMSG_W_CB => {
            if wire_type as ::core::ffi::c_uint
                != PB_WT_STRING as ::core::ffi::c_int as ::core::ffi::c_uint
            {
                (*stream).errmsg = (if !(*stream).errmsg.is_null() {
                    (*stream).errmsg
                } else {
                    b"wrong wire type\0" as *const u8 as *const ::core::ffi::c_char
                });
                return false_0 != 0;
            }
            return pb_dec_submessage(stream, field);
        }
        PB_LTYPE_FIXED_LENGTH_BYTES => {
            if wire_type as ::core::ffi::c_uint
                != PB_WT_STRING as ::core::ffi::c_int as ::core::ffi::c_uint
            {
                (*stream).errmsg = (if !(*stream).errmsg.is_null() {
                    (*stream).errmsg
                } else {
                    b"wrong wire type\0" as *const u8 as *const ::core::ffi::c_char
                });
                return false_0 != 0;
            }
            return pb_dec_fixed_length_bytes(stream, field);
        }
        _ => {
            (*stream).errmsg = (if !(*stream).errmsg.is_null() {
                (*stream).errmsg
            } else {
                b"invalid field type\0" as *const u8 as *const ::core::ffi::c_char
            });
            return false_0 != 0;
        }
    };
}
unsafe extern "C" fn decode_static_field(
    mut stream: *mut pb_istream_t,
    mut wire_type: pb_wire_type_t,
    mut field: *mut pb_field_iter_t,
) -> bool {
    match (*field).type_0 as ::core::ffi::c_uint & PB_HTYPE_MASK {
        PB_HTYPE_REQUIRED => return decode_basic_field(stream, wire_type, field),
        PB_HTYPE_OPTIONAL => {
            if !(*field).pSize.is_null() {
                *((*field).pSize as *mut bool) = true_0 != 0;
            }
            return decode_basic_field(stream, wire_type, field);
        }
        PB_HTYPE_REPEATED => {
            if wire_type as ::core::ffi::c_uint
                == PB_WT_STRING as ::core::ffi::c_int as ::core::ffi::c_uint
                && (*field).type_0 as ::core::ffi::c_uint & PB_LTYPE_MASK
                    <= PB_LTYPE_LAST_PACKABLE
            {
                let mut status: bool = true_0 != 0;
                let mut substream: pb_istream_t = pb_istream_s {
                    callback: None,
                    state: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                    bytes_left: 0,
                    errmsg: ::core::ptr::null::<::core::ffi::c_char>(),
                };
                let mut size: *mut pb_size_t = (*field).pSize as *mut pb_size_t;
                (*field).pData = ((*field).pField as *mut ::core::ffi::c_char)
                    .offset(
                        ((*field).data_size as ::core::ffi::c_int
                            * *size as ::core::ffi::c_int) as isize,
                    ) as *mut ::core::ffi::c_void;
                if !pb_make_string_substream(stream, &raw mut substream) {
                    return false_0 != 0;
                }
                while substream.bytes_left > 0 as size_t
                    && (*size as ::core::ffi::c_int)
                        < (*field).array_size as ::core::ffi::c_int
                {
                    if !decode_basic_field(&raw mut substream, PB_WT_PACKED, field) {
                        status = false_0 != 0;
                        break;
                    } else {
                        *size = (*size).wrapping_add(1);
                        (*field).pData = ((*field).pData as *mut ::core::ffi::c_char)
                            .offset((*field).data_size as ::core::ffi::c_int as isize)
                            as *mut ::core::ffi::c_void;
                    }
                }
                if substream.bytes_left != 0 as size_t {
                    (*stream).errmsg = (if !(*stream).errmsg.is_null() {
                        (*stream).errmsg
                    } else {
                        b"array overflow\0" as *const u8 as *const ::core::ffi::c_char
                    });
                    return false_0 != 0;
                }
                if !pb_close_string_substream(stream, &raw mut substream) {
                    return false_0 != 0;
                }
                return status;
            } else {
                let mut size_0: *mut pb_size_t = (*field).pSize as *mut pb_size_t;
                (*field).pData = ((*field).pField as *mut ::core::ffi::c_char)
                    .offset(
                        ((*field).data_size as ::core::ffi::c_int
                            * *size_0 as ::core::ffi::c_int) as isize,
                    ) as *mut ::core::ffi::c_void;
                let fresh1 = *size_0;
                *size_0 = (*size_0).wrapping_add(1);
                if fresh1 as ::core::ffi::c_int
                    >= (*field).array_size as ::core::ffi::c_int
                {
                    (*stream).errmsg = (if !(*stream).errmsg.is_null() {
                        (*stream).errmsg
                    } else {
                        b"array overflow\0" as *const u8 as *const ::core::ffi::c_char
                    });
                    return false_0 != 0;
                }
                return decode_basic_field(stream, wire_type, field);
            }
        }
        PB_HTYPE_ONEOF => {
            if ((*field).type_0 as ::core::ffi::c_uint & PB_LTYPE_MASK
                == PB_LTYPE_SUBMESSAGE
                || (*field).type_0 as ::core::ffi::c_uint & PB_LTYPE_MASK
                    == PB_LTYPE_SUBMSG_W_CB)
                && *((*field).pSize as *mut pb_size_t) as ::core::ffi::c_int
                    != (*field).tag as ::core::ffi::c_int
            {
                memset(
                    (*field).pData,
                    0 as ::core::ffi::c_int,
                    (*field).data_size as size_t,
                );
                if !(*(*field).submsg_desc).default_value.is_null()
                    || (*(*field).submsg_desc).field_callback.is_some()
                    || !(*(*(*field).submsg_desc)
                        .submsg_info
                        .offset(0 as ::core::ffi::c_int as isize))
                        .is_null()
                {
                    let mut submsg_iter: pb_field_iter_t = pb_field_iter_s {
                        descriptor: ::core::ptr::null::<pb_msgdesc_t>(),
                        message: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                        index: 0,
                        field_info_index: 0,
                        required_field_index: 0,
                        submessage_index: 0,
                        tag: 0,
                        data_size: 0,
                        array_size: 0,
                        type_0: 0,
                        pField: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                        pData: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                        pSize: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                        submsg_desc: ::core::ptr::null::<pb_msgdesc_t>(),
                    };
                    if pb_field_iter_begin(
                        &raw mut submsg_iter,
                        (*field).submsg_desc,
                        (*field).pData,
                    ) {
                        if !pb_message_set_to_defaults(&raw mut submsg_iter) {
                            (*stream).errmsg = (if !(*stream).errmsg.is_null() {
                                (*stream).errmsg
                            } else {
                                b"failed to set defaults\0" as *const u8
                                    as *const ::core::ffi::c_char
                            });
                            return false_0 != 0;
                        }
                    }
                }
            }
            *((*field).pSize as *mut pb_size_t) = (*field).tag;
            return decode_basic_field(stream, wire_type, field);
        }
        _ => {
            (*stream).errmsg = (if !(*stream).errmsg.is_null() {
                (*stream).errmsg
            } else {
                b"invalid field type\0" as *const u8 as *const ::core::ffi::c_char
            });
            return false_0 != 0;
        }
    };
}
unsafe extern "C" fn decode_pointer_field(
    mut stream: *mut pb_istream_t,
    mut wire_type: pb_wire_type_t,
    mut field: *mut pb_field_iter_t,
) -> bool {
    (*stream).errmsg = (if !(*stream).errmsg.is_null() {
        (*stream).errmsg
    } else {
        b"no malloc support\0" as *const u8 as *const ::core::ffi::c_char
    });
    return false_0 != 0;
}
unsafe extern "C" fn decode_callback_field(
    mut stream: *mut pb_istream_t,
    mut wire_type: pb_wire_type_t,
    mut field: *mut pb_field_iter_t,
) -> bool {
    if (*(*field).descriptor).field_callback.is_none() {
        return pb_skip_field(stream, wire_type);
    }
    if wire_type as ::core::ffi::c_uint
        == PB_WT_STRING as ::core::ffi::c_int as ::core::ffi::c_uint
    {
        let mut substream: pb_istream_t = pb_istream_s {
            callback: None,
            state: ::core::ptr::null_mut::<::core::ffi::c_void>(),
            bytes_left: 0,
            errmsg: ::core::ptr::null::<::core::ffi::c_char>(),
        };
        let mut prev_bytes_left: size_t = 0;
        if !pb_make_string_substream(stream, &raw mut substream) {
            return false_0 != 0;
        }
        loop {
            prev_bytes_left = substream.bytes_left;
            if !(*(*field).descriptor)
                .field_callback
                .expect(
                    "non-null function pointer",
                )(&raw mut substream, ::core::ptr::null_mut::<pb_ostream_t>(), field)
            {
                (*stream).errmsg = if !(*stream).errmsg.is_null() {
                    (*stream).errmsg
                } else if !substream.errmsg.is_null() {
                    substream.errmsg
                } else {
                    b"callback failed\0" as *const u8 as *const ::core::ffi::c_char
                };
                return false_0 != 0;
            }
            if !(substream.bytes_left > 0 as size_t
                && substream.bytes_left < prev_bytes_left)
            {
                break;
            }
        }
        if !pb_close_string_substream(stream, &raw mut substream) {
            return false_0 != 0;
        }
        return true_0 != 0;
    } else {
        let mut substream_0: pb_istream_t = pb_istream_s {
            callback: None,
            state: ::core::ptr::null_mut::<::core::ffi::c_void>(),
            bytes_left: 0,
            errmsg: ::core::ptr::null::<::core::ffi::c_char>(),
        };
        let mut buffer: [pb_byte_t; 10] = [0; 10];
        let mut size: size_t = ::core::mem::size_of::<[pb_byte_t; 10]>() as size_t;
        if !read_raw_value(
            stream,
            wire_type,
            &raw mut buffer as *mut pb_byte_t,
            &raw mut size,
        ) {
            return false_0 != 0;
        }
        substream_0 = pb_istream_from_buffer(&raw mut buffer as *mut pb_byte_t, size);
        return (*(*field).descriptor)
            .field_callback
            .expect(
                "non-null function pointer",
            )(&raw mut substream_0, ::core::ptr::null_mut::<pb_ostream_t>(), field);
    };
}
unsafe extern "C" fn decode_field(
    mut stream: *mut pb_istream_t,
    mut wire_type: pb_wire_type_t,
    mut field: *mut pb_field_iter_t,
) -> bool {
    match (*field).type_0 as ::core::ffi::c_uint & PB_ATYPE_MASK {
        PB_ATYPE_STATIC => return decode_static_field(stream, wire_type, field),
        PB_ATYPE_POINTER => return decode_pointer_field(stream, wire_type, field),
        PB_ATYPE_CALLBACK => return decode_callback_field(stream, wire_type, field),
        _ => {
            (*stream).errmsg = (if !(*stream).errmsg.is_null() {
                (*stream).errmsg
            } else {
                b"invalid field type\0" as *const u8 as *const ::core::ffi::c_char
            });
            return false_0 != 0;
        }
    };
}
unsafe extern "C" fn default_extension_decoder(
    mut stream: *mut pb_istream_t,
    mut extension: *mut pb_extension_t,
    mut tag: uint32_t,
    mut wire_type: pb_wire_type_t,
) -> bool {
    let mut iter: pb_field_iter_t = pb_field_iter_s {
        descriptor: ::core::ptr::null::<pb_msgdesc_t>(),
        message: ::core::ptr::null_mut::<::core::ffi::c_void>(),
        index: 0,
        field_info_index: 0,
        required_field_index: 0,
        submessage_index: 0,
        tag: 0,
        data_size: 0,
        array_size: 0,
        type_0: 0,
        pField: ::core::ptr::null_mut::<::core::ffi::c_void>(),
        pData: ::core::ptr::null_mut::<::core::ffi::c_void>(),
        pSize: ::core::ptr::null_mut::<::core::ffi::c_void>(),
        submsg_desc: ::core::ptr::null::<pb_msgdesc_t>(),
    };
    if !pb_field_iter_begin_extension(&raw mut iter, extension) {
        (*stream).errmsg = (if !(*stream).errmsg.is_null() {
            (*stream).errmsg
        } else {
            b"invalid extension\0" as *const u8 as *const ::core::ffi::c_char
        });
        return false_0 != 0;
    }
    if iter.tag as uint32_t != tag || iter.message.is_null() {
        return true_0 != 0;
    }
    (*extension).found = true_0 != 0;
    return decode_field(stream, wire_type, &raw mut iter);
}
unsafe extern "C" fn decode_extension(
    mut stream: *mut pb_istream_t,
    mut tag: uint32_t,
    mut wire_type: pb_wire_type_t,
    mut extension: *mut pb_extension_t,
) -> bool {
    let mut pos: size_t = (*stream).bytes_left;
    while !extension.is_null() && pos == (*stream).bytes_left {
        let mut status: bool = false;
        if (*(*extension).type_0).decode.is_some() {
            status = (*(*extension).type_0)
                .decode
                .expect("non-null function pointer")(stream, extension, tag, wire_type);
        } else {
            status = default_extension_decoder(stream, extension, tag, wire_type);
        }
        if !status {
            return false_0 != 0;
        }
        extension = (*extension).next;
    }
    return true_0 != 0;
}
unsafe extern "C" fn pb_field_set_to_default(mut field: *mut pb_field_iter_t) -> bool {
    let mut type_0: pb_type_t = 0;
    type_0 = (*field).type_0;
    if type_0 as ::core::ffi::c_uint & PB_LTYPE_MASK == PB_LTYPE_EXTENSION {
        let mut ext: *mut pb_extension_t = *((*field).pData
            as *const *mut pb_extension_t);
        while !ext.is_null() {
            let mut ext_iter: pb_field_iter_t = pb_field_iter_s {
                descriptor: ::core::ptr::null::<pb_msgdesc_t>(),
                message: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                index: 0,
                field_info_index: 0,
                required_field_index: 0,
                submessage_index: 0,
                tag: 0,
                data_size: 0,
                array_size: 0,
                type_0: 0,
                pField: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                pData: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                pSize: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                submsg_desc: ::core::ptr::null::<pb_msgdesc_t>(),
            };
            if pb_field_iter_begin_extension(&raw mut ext_iter, ext) {
                (*ext).found = false_0 != 0;
                if !pb_message_set_to_defaults(&raw mut ext_iter) {
                    return false_0 != 0;
                }
            }
            ext = (*ext).next;
        }
    } else if type_0 as ::core::ffi::c_uint & PB_ATYPE_MASK == PB_ATYPE_STATIC {
        let mut init_data: bool = true_0 != 0;
        if type_0 as ::core::ffi::c_uint & PB_HTYPE_MASK == PB_HTYPE_OPTIONAL
            && !(*field).pSize.is_null()
        {
            *((*field).pSize as *mut bool) = false_0 != 0;
        } else if type_0 as ::core::ffi::c_uint & PB_HTYPE_MASK == PB_HTYPE_REPEATED
            || type_0 as ::core::ffi::c_uint & PB_HTYPE_MASK == PB_HTYPE_ONEOF
        {
            *((*field).pSize as *mut pb_size_t) = 0 as pb_size_t;
            init_data = false_0 != 0;
        }
        if init_data {
            if ((*field).type_0 as ::core::ffi::c_uint & PB_LTYPE_MASK
                == PB_LTYPE_SUBMESSAGE
                || (*field).type_0 as ::core::ffi::c_uint & PB_LTYPE_MASK
                    == PB_LTYPE_SUBMSG_W_CB)
                && (!(*(*field).submsg_desc).default_value.is_null()
                    || (*(*field).submsg_desc).field_callback.is_some()
                    || !(*(*(*field).submsg_desc)
                        .submsg_info
                        .offset(0 as ::core::ffi::c_int as isize))
                        .is_null())
            {
                let mut submsg_iter: pb_field_iter_t = pb_field_iter_s {
                    descriptor: ::core::ptr::null::<pb_msgdesc_t>(),
                    message: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                    index: 0,
                    field_info_index: 0,
                    required_field_index: 0,
                    submessage_index: 0,
                    tag: 0,
                    data_size: 0,
                    array_size: 0,
                    type_0: 0,
                    pField: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                    pData: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                    pSize: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                    submsg_desc: ::core::ptr::null::<pb_msgdesc_t>(),
                };
                if pb_field_iter_begin(
                    &raw mut submsg_iter,
                    (*field).submsg_desc,
                    (*field).pData,
                ) {
                    if !pb_message_set_to_defaults(&raw mut submsg_iter) {
                        return false_0 != 0;
                    }
                }
            } else {
                memset(
                    (*field).pData,
                    0 as ::core::ffi::c_int,
                    (*field).data_size as size_t,
                );
            }
        }
    } else if type_0 as ::core::ffi::c_uint & PB_ATYPE_MASK == PB_ATYPE_POINTER {
        let ref mut fresh10 = *((*field).pField as *mut *mut ::core::ffi::c_void);
        *fresh10 = NULL;
        if type_0 as ::core::ffi::c_uint & PB_HTYPE_MASK == PB_HTYPE_REPEATED
            || type_0 as ::core::ffi::c_uint & PB_HTYPE_MASK == PB_HTYPE_ONEOF
        {
            *((*field).pSize as *mut pb_size_t) = 0 as pb_size_t;
        }
    } else {
        type_0 as ::core::ffi::c_uint & PB_ATYPE_MASK == PB_ATYPE_CALLBACK;
    }
    return true_0 != 0;
}
unsafe extern "C" fn pb_message_set_to_defaults(mut iter: *mut pb_field_iter_t) -> bool {
    let mut defstream: pb_istream_t = pb_istream_s {
        callback: None,
        state: ::core::ptr::null_mut::<::core::ffi::c_void>(),
        bytes_left: 0 as size_t,
        errmsg: ::core::ptr::null::<::core::ffi::c_char>(),
    };
    let mut tag: uint32_t = 0 as uint32_t;
    let mut wire_type: pb_wire_type_t = PB_WT_VARINT;
    let mut eof: bool = false;
    if !(*(*iter).descriptor).default_value.is_null() {
        defstream = pb_istream_from_buffer(
            (*(*iter).descriptor).default_value,
            -(1 as ::core::ffi::c_int) as size_t,
        );
        if !pb_decode_tag(
            &raw mut defstream,
            &raw mut wire_type,
            &raw mut tag,
            &raw mut eof,
        ) {
            return false_0 != 0;
        }
    }
    loop {
        if !pb_field_set_to_default(iter) {
            return false_0 != 0;
        }
        if tag != 0 as uint32_t && (*iter).tag as uint32_t == tag {
            if !decode_field(&raw mut defstream, wire_type, iter) {
                return false_0 != 0;
            }
            if !pb_decode_tag(
                &raw mut defstream,
                &raw mut wire_type,
                &raw mut tag,
                &raw mut eof,
            ) {
                return false_0 != 0;
            }
            if !(*iter).pSize.is_null() {
                *((*iter).pSize as *mut bool) = false_0 != 0;
            }
        }
        if !pb_field_iter_next(iter) {
            break;
        }
    }
    return true_0 != 0;
}
unsafe extern "C" fn pb_decode_inner(
    mut stream: *mut pb_istream_t,
    mut fields: *const pb_msgdesc_t,
    mut dest_struct: *mut ::core::ffi::c_void,
    mut flags: ::core::ffi::c_uint,
) -> bool {
    let mut extension_range_start: uint32_t = 0 as uint32_t;
    let mut extensions: *mut pb_extension_t = ::core::ptr::null_mut::<pb_extension_t>();
    let mut fixed_count_field: pb_size_t = PB_SIZE_MAX;
    let mut fixed_count_size: pb_size_t = 0 as pb_size_t;
    let mut fixed_count_total_size: pb_size_t = 0 as pb_size_t;
    let mut fields_seen: pb_fields_seen_t = pb_fields_seen_t {
        bitfield: [
            0 as ::core::ffi::c_int as uint32_t,
            0 as ::core::ffi::c_int as uint32_t,
        ],
    };
    let allbits: uint32_t = !(0 as ::core::ffi::c_int as uint32_t);
    let mut iter: pb_field_iter_t = pb_field_iter_s {
        descriptor: ::core::ptr::null::<pb_msgdesc_t>(),
        message: ::core::ptr::null_mut::<::core::ffi::c_void>(),
        index: 0,
        field_info_index: 0,
        required_field_index: 0,
        submessage_index: 0,
        tag: 0,
        data_size: 0,
        array_size: 0,
        type_0: 0,
        pField: ::core::ptr::null_mut::<::core::ffi::c_void>(),
        pData: ::core::ptr::null_mut::<::core::ffi::c_void>(),
        pSize: ::core::ptr::null_mut::<::core::ffi::c_void>(),
        submsg_desc: ::core::ptr::null::<pb_msgdesc_t>(),
    };
    if pb_field_iter_begin(&raw mut iter, fields, dest_struct) {
        if flags & PB_DECODE_NOINIT == 0 as ::core::ffi::c_uint {
            if !pb_message_set_to_defaults(&raw mut iter) {
                (*stream).errmsg = (if !(*stream).errmsg.is_null() {
                    (*stream).errmsg
                } else {
                    b"failed to set defaults\0" as *const u8
                        as *const ::core::ffi::c_char
                });
                return false_0 != 0;
            }
        }
    }
    while (*stream).bytes_left != 0 {
        let mut tag: uint32_t = 0;
        let mut wire_type: pb_wire_type_t = PB_WT_VARINT;
        let mut eof: bool = false;
        if !pb_decode_tag(stream, &raw mut wire_type, &raw mut tag, &raw mut eof) {
            if eof {
                break;
            }
            return false_0 != 0;
        } else if tag == 0 as uint32_t {
            if flags & PB_DECODE_NULLTERMINATED != 0 {
                break;
            }
            (*stream).errmsg = (if !(*stream).errmsg.is_null() {
                (*stream).errmsg
            } else {
                b"zero tag\0" as *const u8 as *const ::core::ffi::c_char
            });
            return false_0 != 0;
        } else if !pb_field_iter_find(&raw mut iter, tag)
            || iter.type_0 as ::core::ffi::c_uint & PB_LTYPE_MASK == PB_LTYPE_EXTENSION
        {
            if extension_range_start == 0 as uint32_t {
                if pb_field_iter_find_extension(&raw mut iter) {
                    extensions = *(iter.pData as *const *mut pb_extension_t);
                    extension_range_start = iter.tag as uint32_t;
                }
                if extensions.is_null() {
                    extension_range_start = -(1 as ::core::ffi::c_int) as uint32_t;
                }
            }
            if tag >= extension_range_start {
                let mut pos: size_t = (*stream).bytes_left;
                if !decode_extension(stream, tag, wire_type, extensions) {
                    return false_0 != 0;
                }
                if pos != (*stream).bytes_left {
                    continue;
                }
            }
            if !pb_skip_field(stream, wire_type) {
                return false_0 != 0;
            }
        } else {
            if iter.type_0 as ::core::ffi::c_uint & PB_HTYPE_MASK == PB_HTYPE_REPEATED
                && iter.pSize == &raw mut iter.array_size as *mut ::core::ffi::c_void
            {
                if fixed_count_field as ::core::ffi::c_int
                    != iter.index as ::core::ffi::c_int
                {
                    if fixed_count_field as ::core::ffi::c_int
                        != PB_SIZE_MAX as ::core::ffi::c_int
                        && fixed_count_size as ::core::ffi::c_int
                            != fixed_count_total_size as ::core::ffi::c_int
                    {
                        (*stream).errmsg = (if !(*stream).errmsg.is_null() {
                            (*stream).errmsg
                        } else {
                            b"wrong size for fixed count field\0" as *const u8
                                as *const ::core::ffi::c_char
                        });
                        return false_0 != 0;
                    }
                    fixed_count_field = iter.index;
                    fixed_count_size = 0 as pb_size_t;
                    fixed_count_total_size = iter.array_size;
                }
                iter.pSize = &raw mut fixed_count_size as *mut ::core::ffi::c_void;
            }
            if iter.type_0 as ::core::ffi::c_uint & PB_HTYPE_MASK == PB_HTYPE_REQUIRED
                && (iter.required_field_index as ::core::ffi::c_int)
                    < PB_MAX_REQUIRED_FIELDS
            {
                let mut tmp: uint32_t = (1 as ::core::ffi::c_int as uint32_t)
                    << (iter.required_field_index as ::core::ffi::c_int
                        & 31 as ::core::ffi::c_int);
                fields_seen
                    .bitfield[(iter.required_field_index as ::core::ffi::c_int
                    >> 5 as ::core::ffi::c_int) as usize] |= tmp;
            }
            if !decode_field(stream, wire_type, &raw mut iter) {
                return false_0 != 0;
            }
        }
    }
    if fixed_count_field as ::core::ffi::c_int != PB_SIZE_MAX as ::core::ffi::c_int
        && fixed_count_size as ::core::ffi::c_int
            != fixed_count_total_size as ::core::ffi::c_int
    {
        (*stream).errmsg = (if !(*stream).errmsg.is_null() {
            (*stream).errmsg
        } else {
            b"wrong size for fixed count field\0" as *const u8
                as *const ::core::ffi::c_char
        });
        return false_0 != 0;
    }
    let mut req_field_count: pb_size_t = (*iter.descriptor).required_field_count;
    if req_field_count as ::core::ffi::c_int > 0 as ::core::ffi::c_int {
        let mut i: pb_size_t = 0;
        if req_field_count as ::core::ffi::c_int > PB_MAX_REQUIRED_FIELDS {
            req_field_count = PB_MAX_REQUIRED_FIELDS as pb_size_t;
        }
        i = 0 as pb_size_t;
        while (i as ::core::ffi::c_int)
            < req_field_count as ::core::ffi::c_int >> 5 as ::core::ffi::c_int
        {
            if fields_seen.bitfield[i as usize] != allbits {
                (*stream).errmsg = (if !(*stream).errmsg.is_null() {
                    (*stream).errmsg
                } else {
                    b"missing required field\0" as *const u8
                        as *const ::core::ffi::c_char
                });
                return false_0 != 0;
            }
            i = i.wrapping_add(1);
        }
        if req_field_count as ::core::ffi::c_int & 31 as ::core::ffi::c_int
            != 0 as ::core::ffi::c_int
        {
            if fields_seen
                .bitfield[(req_field_count as ::core::ffi::c_int
                >> 5 as ::core::ffi::c_int) as usize]
                != allbits
                    >> (32 as ::core::ffi::c_int
                        - (req_field_count as ::core::ffi::c_int
                            & 31 as ::core::ffi::c_int)) as uint_least8_t
                        as ::core::ffi::c_int
            {
                (*stream).errmsg = (if !(*stream).errmsg.is_null() {
                    (*stream).errmsg
                } else {
                    b"missing required field\0" as *const u8
                        as *const ::core::ffi::c_char
                });
                return false_0 != 0;
            }
        }
    }
    return true_0 != 0;
}
#[no_mangle]
pub unsafe extern "C" fn pb_decode_ex(
    mut stream: *mut pb_istream_t,
    mut fields: *const pb_msgdesc_t,
    mut dest_struct: *mut ::core::ffi::c_void,
    mut flags: ::core::ffi::c_uint,
) -> bool {
    let mut status: bool = false;
    if flags & PB_DECODE_DELIMITED == 0 as ::core::ffi::c_uint {
        status = pb_decode_inner(stream, fields, dest_struct, flags);
    } else {
        let mut substream: pb_istream_t = pb_istream_s {
            callback: None,
            state: ::core::ptr::null_mut::<::core::ffi::c_void>(),
            bytes_left: 0,
            errmsg: ::core::ptr::null::<::core::ffi::c_char>(),
        };
        if !pb_make_string_substream(stream, &raw mut substream) {
            return false_0 != 0;
        }
        status = pb_decode_inner(&raw mut substream, fields, dest_struct, flags);
        if !pb_close_string_substream(stream, &raw mut substream) {
            status = false_0 != 0;
        }
    }
    return status;
}
#[no_mangle]
pub unsafe extern "C" fn pb_decode(
    mut stream: *mut pb_istream_t,
    mut fields: *const pb_msgdesc_t,
    mut dest_struct: *mut ::core::ffi::c_void,
) -> bool {
    let mut status: bool = false;
    status = pb_decode_inner(stream, fields, dest_struct, 0 as ::core::ffi::c_uint);
    return status;
}
#[no_mangle]
pub unsafe extern "C" fn pb_release(
    mut fields: *const pb_msgdesc_t,
    mut dest_struct: *mut ::core::ffi::c_void,
) {}
#[no_mangle]
pub unsafe extern "C" fn pb_decode_bool(
    mut stream: *mut pb_istream_t,
    mut dest: *mut bool,
) -> bool {
    let mut value: uint32_t = 0;
    if !pb_decode_varint32(stream, &raw mut value) {
        return false_0 != 0;
    }
    *dest = value != 0 as uint32_t;
    return true_0 != 0;
}
#[no_mangle]
pub unsafe extern "C" fn pb_decode_svarint(
    mut stream: *mut pb_istream_t,
    mut dest: *mut int64_t,
) -> bool {
    let mut value: uint64_t = 0;
    if !pb_decode_varint(stream, &raw mut value) {
        return false_0 != 0;
    }
    if value & 1 as uint64_t != 0 {
        *dest = !(value >> 1 as ::core::ffi::c_int) as int64_t;
    } else {
        *dest = (value >> 1 as ::core::ffi::c_int) as int64_t;
    }
    return true_0 != 0;
}
#[no_mangle]
pub unsafe extern "C" fn pb_decode_fixed32(
    mut stream: *mut pb_istream_t,
    mut dest: *mut ::core::ffi::c_void,
) -> bool {
    let mut u: C2RustUnnamed_2 = C2RustUnnamed_2 { fixed32: 0 };
    if !pb_read(stream, &raw mut u.bytes as *mut pb_byte_t, 4 as size_t) {
        return false_0 != 0;
    }
    *(dest as *mut uint32_t) = u.fixed32;
    return true_0 != 0;
}
#[no_mangle]
pub unsafe extern "C" fn pb_decode_fixed64(
    mut stream: *mut pb_istream_t,
    mut dest: *mut ::core::ffi::c_void,
) -> bool {
    let mut u: C2RustUnnamed_1 = C2RustUnnamed_1 { fixed64: 0 };
    if !pb_read(stream, &raw mut u.bytes as *mut pb_byte_t, 8 as size_t) {
        return false_0 != 0;
    }
    *(dest as *mut uint64_t) = u.fixed64;
    return true_0 != 0;
}
unsafe extern "C" fn pb_dec_bool(
    mut stream: *mut pb_istream_t,
    mut field: *const pb_field_iter_t,
) -> bool {
    return pb_decode_bool(stream, (*field).pData as *mut bool);
}
unsafe extern "C" fn pb_dec_varint(
    mut stream: *mut pb_istream_t,
    mut field: *const pb_field_iter_t,
) -> bool {
    if (*field).type_0 as ::core::ffi::c_uint & PB_LTYPE_MASK == PB_LTYPE_UVARINT {
        let mut value: uint64_t = 0;
        let mut clamped: uint64_t = 0;
        if !pb_decode_varint(stream, &raw mut value) {
            return false_0 != 0;
        }
        if (*field).data_size as usize == ::core::mem::size_of::<uint64_t>() as usize {
            let ref mut fresh2 = *((*field).pData as *mut uint64_t);
            *fresh2 = value;
            clamped = *fresh2;
        } else if (*field).data_size as usize
            == ::core::mem::size_of::<uint32_t>() as usize
        {
            let ref mut fresh3 = *((*field).pData as *mut uint32_t);
            *fresh3 = value as uint32_t;
            clamped = *fresh3 as uint64_t;
        } else if (*field).data_size as usize
            == ::core::mem::size_of::<uint_least16_t>() as usize
        {
            let ref mut fresh4 = *((*field).pData as *mut uint_least16_t);
            *fresh4 = value as uint_least16_t;
            clamped = *fresh4 as uint64_t;
        } else if (*field).data_size as usize
            == ::core::mem::size_of::<uint_least8_t>() as usize
        {
            let ref mut fresh5 = *((*field).pData as *mut uint_least8_t);
            *fresh5 = value as uint_least8_t;
            clamped = *fresh5 as uint64_t;
        } else {
            (*stream).errmsg = (if !(*stream).errmsg.is_null() {
                (*stream).errmsg
            } else {
                b"invalid data_size\0" as *const u8 as *const ::core::ffi::c_char
            });
            return false_0 != 0;
        }
        if clamped != value {
            (*stream).errmsg = (if !(*stream).errmsg.is_null() {
                (*stream).errmsg
            } else {
                b"integer too large\0" as *const u8 as *const ::core::ffi::c_char
            });
            return false_0 != 0;
        }
        return true_0 != 0;
    } else {
        let mut value_0: uint64_t = 0;
        let mut svalue: int64_t = 0;
        let mut clamped_0: int64_t = 0;
        if (*field).type_0 as ::core::ffi::c_uint & PB_LTYPE_MASK == PB_LTYPE_SVARINT {
            if !pb_decode_svarint(stream, &raw mut svalue) {
                return false_0 != 0;
            }
        } else {
            if !pb_decode_varint(stream, &raw mut value_0) {
                return false_0 != 0;
            }
            if (*field).data_size as usize == ::core::mem::size_of::<int64_t>() as usize
            {
                svalue = value_0 as int64_t;
            } else {
                svalue = value_0 as int32_t as int64_t;
            }
        }
        if (*field).data_size as usize == ::core::mem::size_of::<int64_t>() as usize {
            let ref mut fresh6 = *((*field).pData as *mut int64_t);
            *fresh6 = svalue;
            clamped_0 = *fresh6;
        } else if (*field).data_size as usize
            == ::core::mem::size_of::<int32_t>() as usize
        {
            let ref mut fresh7 = *((*field).pData as *mut int32_t);
            *fresh7 = svalue as int32_t;
            clamped_0 = *fresh7 as int64_t;
        } else if (*field).data_size as usize
            == ::core::mem::size_of::<int_least16_t>() as usize
        {
            let ref mut fresh8 = *((*field).pData as *mut int_least16_t);
            *fresh8 = svalue as int_least16_t;
            clamped_0 = *fresh8 as int64_t;
        } else if (*field).data_size as usize
            == ::core::mem::size_of::<int_least8_t>() as usize
        {
            let ref mut fresh9 = *((*field).pData as *mut int_least8_t);
            *fresh9 = svalue as int_least8_t;
            clamped_0 = *fresh9 as int64_t;
        } else {
            (*stream).errmsg = (if !(*stream).errmsg.is_null() {
                (*stream).errmsg
            } else {
                b"invalid data_size\0" as *const u8 as *const ::core::ffi::c_char
            });
            return false_0 != 0;
        }
        if clamped_0 != svalue {
            (*stream).errmsg = (if !(*stream).errmsg.is_null() {
                (*stream).errmsg
            } else {
                b"integer too large\0" as *const u8 as *const ::core::ffi::c_char
            });
            return false_0 != 0;
        }
        return true_0 != 0;
    };
}
unsafe extern "C" fn pb_dec_bytes(
    mut stream: *mut pb_istream_t,
    mut field: *const pb_field_iter_t,
) -> bool {
    let mut size: uint32_t = 0;
    let mut alloc_size: size_t = 0;
    let mut dest: *mut pb_bytes_array_t = ::core::ptr::null_mut::<pb_bytes_array_t>();
    if !pb_decode_varint32(stream, &raw mut size) {
        return false_0 != 0;
    }
    if size > PB_SIZE_MAX as uint32_t {
        (*stream).errmsg = (if !(*stream).errmsg.is_null() {
            (*stream).errmsg
        } else {
            b"bytes overflow\0" as *const u8 as *const ::core::ffi::c_char
        });
        return false_0 != 0;
    }
    alloc_size = (size as size_t).wrapping_add(2 as size_t);
    if size as size_t > alloc_size {
        (*stream).errmsg = (if !(*stream).errmsg.is_null() {
            (*stream).errmsg
        } else {
            b"size too large\0" as *const u8 as *const ::core::ffi::c_char
        });
        return false_0 != 0;
    }
    if (*field).type_0 as ::core::ffi::c_uint & PB_ATYPE_MASK == PB_ATYPE_POINTER {
        (*stream).errmsg = (if !(*stream).errmsg.is_null() {
            (*stream).errmsg
        } else {
            b"no malloc support\0" as *const u8 as *const ::core::ffi::c_char
        });
        return false_0 != 0;
    } else {
        if alloc_size > (*field).data_size as size_t {
            (*stream).errmsg = (if !(*stream).errmsg.is_null() {
                (*stream).errmsg
            } else {
                b"bytes overflow\0" as *const u8 as *const ::core::ffi::c_char
            });
            return false_0 != 0;
        }
        dest = (*field).pData as *mut pb_bytes_array_t;
    }
    (*dest).size = size as pb_size_t;
    return pb_read(stream, &raw mut (*dest).bytes as *mut pb_byte_t, size as size_t);
}
unsafe extern "C" fn pb_dec_string(
    mut stream: *mut pb_istream_t,
    mut field: *const pb_field_iter_t,
) -> bool {
    let mut size: uint32_t = 0;
    let mut alloc_size: size_t = 0;
    let mut dest: *mut pb_byte_t = (*field).pData as *mut pb_byte_t;
    if !pb_decode_varint32(stream, &raw mut size) {
        return false_0 != 0;
    }
    if size == -(1 as ::core::ffi::c_int) as uint32_t {
        (*stream).errmsg = (if !(*stream).errmsg.is_null() {
            (*stream).errmsg
        } else {
            b"size too large\0" as *const u8 as *const ::core::ffi::c_char
        });
        return false_0 != 0;
    }
    alloc_size = size.wrapping_add(1 as uint32_t) as size_t;
    if alloc_size < size as size_t {
        (*stream).errmsg = (if !(*stream).errmsg.is_null() {
            (*stream).errmsg
        } else {
            b"size too large\0" as *const u8 as *const ::core::ffi::c_char
        });
        return false_0 != 0;
    }
    if (*field).type_0 as ::core::ffi::c_uint & PB_ATYPE_MASK == PB_ATYPE_POINTER {
        (*stream).errmsg = (if !(*stream).errmsg.is_null() {
            (*stream).errmsg
        } else {
            b"no malloc support\0" as *const u8 as *const ::core::ffi::c_char
        });
        return false_0 != 0;
    } else if alloc_size > (*field).data_size as size_t {
        (*stream).errmsg = (if !(*stream).errmsg.is_null() {
            (*stream).errmsg
        } else {
            b"string overflow\0" as *const u8 as *const ::core::ffi::c_char
        });
        return false_0 != 0;
    }
    *dest.offset(size as isize) = 0 as pb_byte_t;
    if !pb_read(stream, dest, size as size_t) {
        return false_0 != 0;
    }
    return true_0 != 0;
}
unsafe extern "C" fn pb_dec_submessage(
    mut stream: *mut pb_istream_t,
    mut field: *const pb_field_iter_t,
) -> bool {
    let mut status: bool = true_0 != 0;
    let mut submsg_consumed: bool = false_0 != 0;
    let mut substream: pb_istream_t = pb_istream_s {
        callback: None,
        state: ::core::ptr::null_mut::<::core::ffi::c_void>(),
        bytes_left: 0,
        errmsg: ::core::ptr::null::<::core::ffi::c_char>(),
    };
    if !pb_make_string_substream(stream, &raw mut substream) {
        return false_0 != 0;
    }
    if (*field).submsg_desc.is_null() {
        (*stream).errmsg = (if !(*stream).errmsg.is_null() {
            (*stream).errmsg
        } else {
            b"invalid field descriptor\0" as *const u8 as *const ::core::ffi::c_char
        });
        return false_0 != 0;
    }
    if (*field).type_0 as ::core::ffi::c_uint & PB_LTYPE_MASK == PB_LTYPE_SUBMSG_W_CB
        && !(*field).pSize.is_null()
    {
        let mut callback: *mut pb_callback_t = ((*field).pSize as *mut pb_callback_t)
            .offset(-(1 as ::core::ffi::c_int as isize));
        if (*callback).funcs.decode.is_some() {
            status = (*callback)
                .funcs
                .decode
                .expect(
                    "non-null function pointer",
                )(
                &raw mut substream,
                field as *const pb_field_t,
                &raw mut (*callback).arg,
            );
            if substream.bytes_left == 0 as size_t {
                submsg_consumed = true_0 != 0;
            }
        }
    }
    if status as ::core::ffi::c_int != 0 && !submsg_consumed {
        let mut flags: ::core::ffi::c_uint = 0 as ::core::ffi::c_uint;
        if (*field).type_0 as ::core::ffi::c_uint & PB_ATYPE_MASK == PB_ATYPE_STATIC
            && (*field).type_0 as ::core::ffi::c_uint & PB_HTYPE_MASK
                != PB_HTYPE_REPEATED
        {
            flags = PB_DECODE_NOINIT;
        }
        status = pb_decode_inner(
            &raw mut substream,
            (*field).submsg_desc,
            (*field).pData,
            flags,
        );
    }
    if !pb_close_string_substream(stream, &raw mut substream) {
        return false_0 != 0;
    }
    return status;
}
unsafe extern "C" fn pb_dec_fixed_length_bytes(
    mut stream: *mut pb_istream_t,
    mut field: *const pb_field_iter_t,
) -> bool {
    let mut size: uint32_t = 0;
    if !pb_decode_varint32(stream, &raw mut size) {
        return false_0 != 0;
    }
    if size > PB_SIZE_MAX as uint32_t {
        (*stream).errmsg = (if !(*stream).errmsg.is_null() {
            (*stream).errmsg
        } else {
            b"bytes overflow\0" as *const u8 as *const ::core::ffi::c_char
        });
        return false_0 != 0;
    }
    if size == 0 as uint32_t {
        memset((*field).pData, 0 as ::core::ffi::c_int, (*field).data_size as size_t);
        return true_0 != 0;
    }
    if size != (*field).data_size as uint32_t {
        (*stream).errmsg = (if !(*stream).errmsg.is_null() {
            (*stream).errmsg
        } else {
            b"incorrect fixed length bytes size\0" as *const u8
                as *const ::core::ffi::c_char
        });
        return false_0 != 0;
    }
    return pb_read(
        stream,
        (*field).pData as *mut pb_byte_t,
        (*field).data_size as size_t,
    );
}
#[repr(C)]
pub struct pb_ostream_s {
    _private: [u8; 0],
}
