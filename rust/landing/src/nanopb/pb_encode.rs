use crate::sys::*;
extern "C" {
    fn pb_default_field_callback(
        istream: *mut pb_istream_t,
        ostream: *mut pb_ostream_t,
        field: *const pb_field_t,
    ) -> bool;
    fn pb_field_iter_begin(
        iter: *mut pb_field_iter_t,
        desc: *const pb_msgdesc_t,
        message: *mut ::core::ffi::c_void,
    ) -> bool;
    fn pb_field_iter_begin_const(
        iter: *mut pb_field_iter_t,
        desc: *const pb_msgdesc_t,
        message: *const ::core::ffi::c_void,
    ) -> bool;
    fn pb_field_iter_begin_extension_const(
        iter: *mut pb_field_iter_t,
        extension: *const pb_extension_t,
    ) -> bool;
    fn pb_field_iter_next(iter: *mut pb_field_iter_t) -> bool;
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
pub type size_t = usize;
pub type pb_byte_t = uint8_t;
pub type pb_type_t = pb_byte_t;
pub type pb_size_t = uint_least16_t;
pub type pb_istream_t = pb_istream_s;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct pb_ostream_s {
    pub callback: Option<
        unsafe extern "C" fn(*mut pb_ostream_t, *const pb_byte_t, size_t) -> bool,
    >,
    pub state: *mut ::core::ffi::c_void,
    pub max_size: size_t,
    pub bytes_written: size_t,
    pub errmsg: *const ::core::ffi::c_char,
}
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
pub const true_0: ::core::ffi::c_int = 1 as ::core::ffi::c_int;
pub const false_0: ::core::ffi::c_int = 0 as ::core::ffi::c_int;
pub const NULL: *mut ::core::ffi::c_void = ::core::ptr::null_mut::<
    ::core::ffi::c_void,
>();
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
pub const PB_HTYPE_REQUIRED: ::core::ffi::c_uint = 0 as ::core::ffi::c_uint;
pub const PB_HTYPE_OPTIONAL: ::core::ffi::c_uint = 0x10 as ::core::ffi::c_uint;
pub const PB_HTYPE_REPEATED: ::core::ffi::c_uint = 0x20 as ::core::ffi::c_uint;
pub const PB_HTYPE_ONEOF: ::core::ffi::c_uint = 0x30 as ::core::ffi::c_uint;
pub const PB_HTYPE_MASK: ::core::ffi::c_uint = 0x30 as ::core::ffi::c_uint;
pub const PB_ATYPE_STATIC: ::core::ffi::c_uint = 0 as ::core::ffi::c_uint;
pub const PB_ATYPE_POINTER: ::core::ffi::c_uint = 0x80 as ::core::ffi::c_uint;
pub const PB_ATYPE_CALLBACK: ::core::ffi::c_uint = 0x40 as ::core::ffi::c_uint;
pub const PB_ATYPE_MASK: ::core::ffi::c_uint = 0xc0 as ::core::ffi::c_uint;
pub const PB_ENCODE_DELIMITED: ::core::ffi::c_uint = 0x2 as ::core::ffi::c_uint;
pub const PB_ENCODE_NULLTERMINATED: ::core::ffi::c_uint = 0x4 as ::core::ffi::c_uint;
unsafe extern "C" fn buf_write(
    mut stream: *mut pb_ostream_t,
    mut buf: *const pb_byte_t,
    mut count: size_t,
) -> bool {
    let mut dest: *mut pb_byte_t = (*stream).state as *mut pb_byte_t;
    (*stream).state = dest.offset(count as isize) as *mut ::core::ffi::c_void;
    memcpy(
        dest as *mut ::core::ffi::c_void,
        buf as *const ::core::ffi::c_void,
        count.wrapping_mul(::core::mem::size_of::<pb_byte_t>() as size_t),
    );
    return true_0 != 0;
}
#[no_mangle]
pub unsafe extern "C" fn pb_ostream_from_buffer(
    mut buf: *mut pb_byte_t,
    mut bufsize: size_t,
) -> pb_ostream_t {
    let mut stream: pb_ostream_t = pb_ostream_s {
        callback: None,
        state: ::core::ptr::null_mut::<::core::ffi::c_void>(),
        max_size: 0,
        bytes_written: 0,
        errmsg: ::core::ptr::null::<::core::ffi::c_char>(),
    };
    stream.callback = Some(
        buf_write
            as unsafe extern "C" fn(*mut pb_ostream_t, *const pb_byte_t, size_t) -> bool,
    )
        as Option<
            unsafe extern "C" fn(*mut pb_ostream_t, *const pb_byte_t, size_t) -> bool,
        >;
    stream.state = buf as *mut ::core::ffi::c_void;
    stream.max_size = bufsize;
    stream.bytes_written = 0 as size_t;
    stream.errmsg = ::core::ptr::null::<::core::ffi::c_char>();
    return stream;
}
#[no_mangle]
pub unsafe extern "C" fn pb_write(
    mut stream: *mut pb_ostream_t,
    mut buf: *const pb_byte_t,
    mut count: size_t,
) -> bool {
    if count > 0 as size_t && (*stream).callback.is_some() {
        if (*stream).bytes_written.wrapping_add(count) < (*stream).bytes_written
            || (*stream).bytes_written.wrapping_add(count) > (*stream).max_size
        {
            (*stream).errmsg = (if !(*stream).errmsg.is_null() {
                (*stream).errmsg
            } else {
                b"stream full\0" as *const u8 as *const ::core::ffi::c_char
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
    }
    (*stream).bytes_written = (*stream).bytes_written.wrapping_add(count);
    return true_0 != 0;
}
unsafe extern "C" fn safe_read_bool(mut pSize: *const ::core::ffi::c_void) -> bool {
    let mut p: *const ::core::ffi::c_char = pSize as *const ::core::ffi::c_char;
    let mut i: size_t = 0;
    i = 0 as size_t;
    while i < ::core::mem::size_of::<bool>() as usize {
        if *p.offset(i as isize) as ::core::ffi::c_int != 0 as ::core::ffi::c_int {
            return true_0 != 0;
        }
        i = i.wrapping_add(1);
    }
    return false_0 != 0;
}
unsafe extern "C" fn encode_array(
    mut stream: *mut pb_ostream_t,
    mut field: *mut pb_field_iter_t,
) -> bool {
    let mut i: pb_size_t = 0;
    let mut count: pb_size_t = 0;
    let mut size: size_t = 0;
    count = *((*field).pSize as *mut pb_size_t);
    if count as ::core::ffi::c_int == 0 as ::core::ffi::c_int {
        return true_0 != 0;
    }
    if (*field).type_0 as ::core::ffi::c_uint & PB_ATYPE_MASK != PB_ATYPE_POINTER
        && count as ::core::ffi::c_int > (*field).array_size as ::core::ffi::c_int
    {
        (*stream).errmsg = (if !(*stream).errmsg.is_null() {
            (*stream).errmsg
        } else {
            b"array max size exceeded\0" as *const u8 as *const ::core::ffi::c_char
        });
        return false_0 != 0;
    }
    if (*field).type_0 as ::core::ffi::c_uint & PB_LTYPE_MASK <= PB_LTYPE_LAST_PACKABLE {
        if !pb_encode_tag(stream, PB_WT_STRING, (*field).tag as uint32_t) {
            return false_0 != 0;
        }
        if (*field).type_0 as ::core::ffi::c_uint & PB_LTYPE_MASK == PB_LTYPE_FIXED32 {
            size = (4 as size_t).wrapping_mul(count as size_t);
        } else if (*field).type_0 as ::core::ffi::c_uint & PB_LTYPE_MASK
            == PB_LTYPE_FIXED64
        {
            size = (8 as size_t).wrapping_mul(count as size_t);
        } else {
            let mut sizestream: pb_ostream_t = pb_ostream_s {
                callback: None,
                state: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                max_size: 0 as size_t,
                bytes_written: 0 as size_t,
                errmsg: ::core::ptr::null::<::core::ffi::c_char>(),
            };
            let mut pData_orig: *mut ::core::ffi::c_void = (*field).pData;
            i = 0 as pb_size_t;
            while (i as ::core::ffi::c_int) < count as ::core::ffi::c_int {
                if !pb_enc_varint(&raw mut sizestream, field) {
                    (*stream).errmsg = (if !(*stream).errmsg.is_null() {
                        (*stream).errmsg
                    } else {
                        (if !sizestream.errmsg.is_null() {
                            sizestream.errmsg
                        } else {
                            b"(none)\0" as *const u8 as *const ::core::ffi::c_char
                        })
                    });
                    return false_0 != 0;
                }
                (*field).pData = ((*field).pData as *mut ::core::ffi::c_char)
                    .offset((*field).data_size as ::core::ffi::c_int as isize)
                    as *mut ::core::ffi::c_void;
                i = i.wrapping_add(1);
            }
            (*field).pData = pData_orig;
            size = sizestream.bytes_written;
        }
        if !pb_encode_varint(stream, size as uint64_t) {
            return false_0 != 0;
        }
        if (*stream).callback.is_none() {
            return pb_write(stream, ::core::ptr::null::<pb_byte_t>(), size);
        }
        i = 0 as pb_size_t;
        while (i as ::core::ffi::c_int) < count as ::core::ffi::c_int {
            if (*field).type_0 as ::core::ffi::c_uint & PB_LTYPE_MASK == PB_LTYPE_FIXED32
                || (*field).type_0 as ::core::ffi::c_uint & PB_LTYPE_MASK
                    == PB_LTYPE_FIXED64
            {
                if !pb_enc_fixed(stream, field) {
                    return false_0 != 0;
                }
            } else if !pb_enc_varint(stream, field) {
                return false_0 != 0
            }
            (*field).pData = ((*field).pData as *mut ::core::ffi::c_char)
                .offset((*field).data_size as ::core::ffi::c_int as isize)
                as *mut ::core::ffi::c_void;
            i = i.wrapping_add(1);
        }
    } else {
        i = 0 as pb_size_t;
        while (i as ::core::ffi::c_int) < count as ::core::ffi::c_int {
            if (*field).type_0 as ::core::ffi::c_uint & PB_ATYPE_MASK == PB_ATYPE_POINTER
                && ((*field).type_0 as ::core::ffi::c_uint & PB_LTYPE_MASK
                    == PB_LTYPE_STRING
                    || (*field).type_0 as ::core::ffi::c_uint & PB_LTYPE_MASK
                        == PB_LTYPE_BYTES)
            {
                let mut status: bool = false;
                let mut pData_orig_0: *mut ::core::ffi::c_void = (*field).pData;
                (*field).pData = *((*field).pData as *const *mut ::core::ffi::c_void);
                if (*field).pData.is_null() {
                    status = pb_encode_tag_for_field(stream, field) as ::core::ffi::c_int
                        != 0
                        && pb_encode_varint(stream, 0 as uint64_t) as ::core::ffi::c_int
                            != 0;
                } else {
                    status = encode_basic_field(stream, field);
                }
                (*field).pData = pData_orig_0;
                if !status {
                    return false_0 != 0;
                }
            } else if !encode_basic_field(stream, field) {
                return false_0 != 0
            }
            (*field).pData = ((*field).pData as *mut ::core::ffi::c_char)
                .offset((*field).data_size as ::core::ffi::c_int as isize)
                as *mut ::core::ffi::c_void;
            i = i.wrapping_add(1);
        }
    }
    return true_0 != 0;
}
unsafe extern "C" fn pb_check_proto3_default_value(
    mut field: *const pb_field_iter_t,
) -> bool {
    let mut type_0: pb_type_t = (*field).type_0;
    if type_0 as ::core::ffi::c_uint & PB_ATYPE_MASK == PB_ATYPE_STATIC {
        if type_0 as ::core::ffi::c_uint & PB_HTYPE_MASK == PB_HTYPE_REQUIRED {
            return false_0 != 0
        } else if type_0 as ::core::ffi::c_uint & PB_HTYPE_MASK == PB_HTYPE_REPEATED {
            return *((*field).pSize as *const pb_size_t) as ::core::ffi::c_int
                == 0 as ::core::ffi::c_int
        } else if type_0 as ::core::ffi::c_uint & PB_HTYPE_MASK == PB_HTYPE_ONEOF {
            return *((*field).pSize as *const pb_size_t) as ::core::ffi::c_int
                == 0 as ::core::ffi::c_int
        } else if type_0 as ::core::ffi::c_uint & PB_HTYPE_MASK == PB_HTYPE_OPTIONAL
            && !(*field).pSize.is_null()
        {
            return safe_read_bool((*field).pSize) as ::core::ffi::c_int == false_0
        } else if !(*(*field).descriptor).default_value.is_null() {
            return false_0 != 0
        }
        if type_0 as ::core::ffi::c_uint & PB_LTYPE_MASK <= PB_LTYPE_LAST_PACKABLE {
            let mut i: pb_size_t = 0;
            let mut p: *const ::core::ffi::c_char = (*field).pData
                as *const ::core::ffi::c_char;
            i = 0 as pb_size_t;
            while (i as ::core::ffi::c_int) < (*field).data_size as ::core::ffi::c_int {
                if *p.offset(i as isize) as ::core::ffi::c_int != 0 as ::core::ffi::c_int
                {
                    return false_0 != 0;
                }
                i = i.wrapping_add(1);
            }
            return true_0 != 0;
        } else if type_0 as ::core::ffi::c_uint & PB_LTYPE_MASK == PB_LTYPE_BYTES {
            let mut bytes: *const pb_bytes_array_t = (*field).pData
                as *const pb_bytes_array_t;
            return (*bytes).size as ::core::ffi::c_int == 0 as ::core::ffi::c_int;
        } else if type_0 as ::core::ffi::c_uint & PB_LTYPE_MASK == PB_LTYPE_STRING {
            return *((*field).pData as *const ::core::ffi::c_char) as ::core::ffi::c_int
                == '\0' as i32
        } else if type_0 as ::core::ffi::c_uint & PB_LTYPE_MASK
            == PB_LTYPE_FIXED_LENGTH_BYTES
        {
            return (*field).data_size as ::core::ffi::c_int == 0 as ::core::ffi::c_int
        } else if type_0 as ::core::ffi::c_uint & PB_LTYPE_MASK == PB_LTYPE_SUBMESSAGE
            || type_0 as ::core::ffi::c_uint & PB_LTYPE_MASK == PB_LTYPE_SUBMSG_W_CB
        {
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
            if pb_field_iter_begin(&raw mut iter, (*field).submsg_desc, (*field).pData) {
                loop {
                    if !pb_check_proto3_default_value(&raw mut iter) {
                        return false_0 != 0;
                    }
                    if !pb_field_iter_next(&raw mut iter) {
                        break;
                    }
                }
            }
            return true_0 != 0;
        }
    } else if type_0 as ::core::ffi::c_uint & PB_ATYPE_MASK == PB_ATYPE_POINTER {
        return (*field).pData.is_null()
    } else if type_0 as ::core::ffi::c_uint & PB_ATYPE_MASK == PB_ATYPE_CALLBACK {
        if type_0 as ::core::ffi::c_uint & PB_LTYPE_MASK == PB_LTYPE_EXTENSION {
            let mut extension: *const pb_extension_t = *((*field).pData
                as *const *const pb_extension_t);
            return extension.is_null();
        } else if (*(*field).descriptor).field_callback
            == Some(
                pb_default_field_callback
                    as unsafe extern "C" fn(
                        *mut pb_istream_t,
                        *mut pb_ostream_t,
                        *const pb_field_t,
                    ) -> bool,
            )
        {
            let mut pCallback: *mut pb_callback_t = (*field).pData as *mut pb_callback_t;
            return (*pCallback).funcs.encode.is_none();
        } else {
            return (*(*field).descriptor).field_callback.is_none()
        }
    }
    return false_0 != 0;
}
unsafe extern "C" fn encode_basic_field(
    mut stream: *mut pb_ostream_t,
    mut field: *const pb_field_iter_t,
) -> bool {
    if (*field).pData.is_null() {
        return true_0 != 0;
    }
    if !pb_encode_tag_for_field(stream, field) {
        return false_0 != 0;
    }
    match (*field).type_0 as ::core::ffi::c_uint & PB_LTYPE_MASK {
        PB_LTYPE_BOOL => return pb_enc_bool(stream, field),
        PB_LTYPE_VARINT | PB_LTYPE_UVARINT | PB_LTYPE_SVARINT => {
            return pb_enc_varint(stream, field);
        }
        PB_LTYPE_FIXED32 | PB_LTYPE_FIXED64 => return pb_enc_fixed(stream, field),
        PB_LTYPE_BYTES => return pb_enc_bytes(stream, field),
        PB_LTYPE_STRING => return pb_enc_string(stream, field),
        PB_LTYPE_SUBMESSAGE | PB_LTYPE_SUBMSG_W_CB => {
            return pb_enc_submessage(stream, field);
        }
        PB_LTYPE_FIXED_LENGTH_BYTES => return pb_enc_fixed_length_bytes(stream, field),
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
unsafe extern "C" fn encode_callback_field(
    mut stream: *mut pb_ostream_t,
    mut field: *const pb_field_iter_t,
) -> bool {
    if (*(*field).descriptor).field_callback.is_some() {
        if !(*(*field).descriptor)
            .field_callback
            .expect(
                "non-null function pointer",
            )(::core::ptr::null_mut::<pb_istream_t>(), stream, field)
        {
            (*stream).errmsg = (if !(*stream).errmsg.is_null() {
                (*stream).errmsg
            } else {
                b"callback error\0" as *const u8 as *const ::core::ffi::c_char
            });
            return false_0 != 0;
        }
    }
    return true_0 != 0;
}
unsafe extern "C" fn encode_field(
    mut stream: *mut pb_ostream_t,
    mut field: *mut pb_field_iter_t,
) -> bool {
    if (*field).type_0 as ::core::ffi::c_uint & PB_HTYPE_MASK == PB_HTYPE_ONEOF {
        if *((*field).pSize as *const pb_size_t) as ::core::ffi::c_int
            != (*field).tag as ::core::ffi::c_int
        {
            return true_0 != 0;
        }
    } else if (*field).type_0 as ::core::ffi::c_uint & PB_HTYPE_MASK == PB_HTYPE_OPTIONAL
    {
        if !(*field).pSize.is_null() {
            if safe_read_bool((*field).pSize) as ::core::ffi::c_int == false_0 {
                return true_0 != 0;
            }
        } else if (*field).type_0 as ::core::ffi::c_uint & PB_ATYPE_MASK
            == PB_ATYPE_STATIC
        {
            if pb_check_proto3_default_value(field) {
                return true_0 != 0;
            }
        }
    }
    if (*field).pData.is_null() {
        if (*field).type_0 as ::core::ffi::c_uint & PB_HTYPE_MASK == PB_HTYPE_REQUIRED {
            (*stream).errmsg = (if !(*stream).errmsg.is_null() {
                (*stream).errmsg
            } else {
                b"missing required field\0" as *const u8 as *const ::core::ffi::c_char
            });
            return false_0 != 0;
        }
        return true_0 != 0;
    }
    if (*field).type_0 as ::core::ffi::c_uint & PB_ATYPE_MASK == PB_ATYPE_CALLBACK {
        return encode_callback_field(stream, field)
    } else if (*field).type_0 as ::core::ffi::c_uint & PB_HTYPE_MASK == PB_HTYPE_REPEATED
    {
        return encode_array(stream, field)
    } else {
        return encode_basic_field(stream, field)
    };
}
unsafe extern "C" fn default_extension_encoder(
    mut stream: *mut pb_ostream_t,
    mut extension: *const pb_extension_t,
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
    if !pb_field_iter_begin_extension_const(&raw mut iter, extension) {
        (*stream).errmsg = (if !(*stream).errmsg.is_null() {
            (*stream).errmsg
        } else {
            b"invalid extension\0" as *const u8 as *const ::core::ffi::c_char
        });
        return false_0 != 0;
    }
    return encode_field(stream, &raw mut iter);
}
unsafe extern "C" fn encode_extension_field(
    mut stream: *mut pb_ostream_t,
    mut field: *const pb_field_iter_t,
) -> bool {
    let mut extension: *const pb_extension_t = *((*field).pData
        as *const *const pb_extension_t);
    while !extension.is_null() {
        let mut status: bool = false;
        if (*(*extension).type_0).encode.is_some() {
            status = (*(*extension).type_0)
                .encode
                .expect("non-null function pointer")(stream, extension);
        } else {
            status = default_extension_encoder(stream, extension);
        }
        if !status {
            return false_0 != 0;
        }
        extension = (*extension).next;
    }
    return true_0 != 0;
}
#[no_mangle]
pub unsafe extern "C" fn pb_encode(
    mut stream: *mut pb_ostream_t,
    mut fields: *const pb_msgdesc_t,
    mut src_struct: *const ::core::ffi::c_void,
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
    if !pb_field_iter_begin_const(&raw mut iter, fields, src_struct) {
        return true_0 != 0;
    }
    loop {
        if iter.type_0 as ::core::ffi::c_uint & PB_LTYPE_MASK == PB_LTYPE_EXTENSION {
            if !encode_extension_field(stream, &raw mut iter) {
                return false_0 != 0;
            }
        } else if !encode_field(stream, &raw mut iter) {
            return false_0 != 0
        }
        if !pb_field_iter_next(&raw mut iter) {
            break;
        }
    }
    return true_0 != 0;
}
#[no_mangle]
pub unsafe extern "C" fn pb_encode_ex(
    mut stream: *mut pb_ostream_t,
    mut fields: *const pb_msgdesc_t,
    mut src_struct: *const ::core::ffi::c_void,
    mut flags: ::core::ffi::c_uint,
) -> bool {
    if flags & PB_ENCODE_DELIMITED != 0 as ::core::ffi::c_uint {
        return pb_encode_submessage(stream, fields, src_struct)
    } else if flags & PB_ENCODE_NULLTERMINATED != 0 as ::core::ffi::c_uint {
        let zero: pb_byte_t = 0 as pb_byte_t;
        if !pb_encode(stream, fields, src_struct) {
            return false_0 != 0;
        }
        return pb_write(stream, &raw const zero, 1 as size_t);
    } else {
        return pb_encode(stream, fields, src_struct)
    };
}
#[no_mangle]
pub unsafe extern "C" fn pb_get_encoded_size(
    mut size: *mut size_t,
    mut fields: *const pb_msgdesc_t,
    mut src_struct: *const ::core::ffi::c_void,
) -> bool {
    let mut stream: pb_ostream_t = pb_ostream_s {
        callback: None,
        state: ::core::ptr::null_mut::<::core::ffi::c_void>(),
        max_size: 0 as size_t,
        bytes_written: 0 as size_t,
        errmsg: ::core::ptr::null::<::core::ffi::c_char>(),
    };
    if !pb_encode(&raw mut stream, fields, src_struct) {
        return false_0 != 0;
    }
    *size = stream.bytes_written;
    return true_0 != 0;
}
unsafe extern "C" fn pb_encode_varint_32(
    mut stream: *mut pb_ostream_t,
    mut low: uint32_t,
    mut high: uint32_t,
) -> bool {
    let mut i: size_t = 0 as size_t;
    let mut buffer: [pb_byte_t; 10] = [0; 10];
    let mut byte: pb_byte_t = (low & 0x7f as uint32_t) as pb_byte_t;
    low >>= 7 as ::core::ffi::c_int;
    while i < 4 as size_t && (low != 0 as uint32_t || high != 0 as uint32_t) {
        byte = (byte as ::core::ffi::c_int | 0x80 as ::core::ffi::c_int) as pb_byte_t;
        let fresh0 = i;
        i = i.wrapping_add(1);
        buffer[fresh0 as usize] = byte;
        byte = (low & 0x7f as uint32_t) as pb_byte_t;
        low >>= 7 as ::core::ffi::c_int;
    }
    if high != 0 {
        byte = (byte as uint32_t | (high & 0x7 as uint32_t) << 4 as ::core::ffi::c_int)
            as pb_byte_t;
        high >>= 3 as ::core::ffi::c_int;
        while high != 0 {
            byte = (byte as ::core::ffi::c_int | 0x80 as ::core::ffi::c_int)
                as pb_byte_t;
            let fresh1 = i;
            i = i.wrapping_add(1);
            buffer[fresh1 as usize] = byte;
            byte = (high & 0x7f as uint32_t) as pb_byte_t;
            high >>= 7 as ::core::ffi::c_int;
        }
    }
    let fresh2 = i;
    i = i.wrapping_add(1);
    buffer[fresh2 as usize] = byte;
    return pb_write(stream, &raw mut buffer as *mut pb_byte_t, i);
}
#[no_mangle]
pub unsafe extern "C" fn pb_encode_varint(
    mut stream: *mut pb_ostream_t,
    mut value: uint64_t,
) -> bool {
    if value <= 0x7f as uint64_t {
        let mut byte: pb_byte_t = value as pb_byte_t;
        return pb_write(stream, &raw mut byte, 1 as size_t);
    } else {
        return pb_encode_varint_32(
            stream,
            value as uint32_t,
            (value >> 32 as ::core::ffi::c_int) as uint32_t,
        )
    };
}
#[no_mangle]
pub unsafe extern "C" fn pb_encode_svarint(
    mut stream: *mut pb_ostream_t,
    mut value: int64_t,
) -> bool {
    let mut zigzagged: uint64_t = 0;
    let mut mask: uint64_t = -(1 as ::core::ffi::c_int) as uint64_t
        >> 1 as ::core::ffi::c_int;
    if value < 0 as int64_t {
        zigzagged = !((value as uint64_t & mask) << 1 as ::core::ffi::c_int);
    } else {
        zigzagged = (value as uint64_t) << 1 as ::core::ffi::c_int;
    }
    return pb_encode_varint(stream, zigzagged);
}
#[no_mangle]
pub unsafe extern "C" fn pb_encode_fixed32(
    mut stream: *mut pb_ostream_t,
    mut value: *const ::core::ffi::c_void,
) -> bool {
    return pb_write(stream, value as *const pb_byte_t, 4 as size_t);
}
#[no_mangle]
pub unsafe extern "C" fn pb_encode_fixed64(
    mut stream: *mut pb_ostream_t,
    mut value: *const ::core::ffi::c_void,
) -> bool {
    return pb_write(stream, value as *const pb_byte_t, 8 as size_t);
}
#[no_mangle]
pub unsafe extern "C" fn pb_encode_tag(
    mut stream: *mut pb_ostream_t,
    mut wiretype: pb_wire_type_t,
    mut field_number: uint32_t,
) -> bool {
    let mut tag: uint64_t = (field_number as uint64_t) << 3 as ::core::ffi::c_int
        | wiretype as uint64_t;
    return pb_encode_varint(stream, tag);
}
#[no_mangle]
pub unsafe extern "C" fn pb_encode_tag_for_field(
    mut stream: *mut pb_ostream_t,
    mut field: *const pb_field_iter_t,
) -> bool {
    let mut wiretype: pb_wire_type_t = PB_WT_VARINT;
    match (*field).type_0 as ::core::ffi::c_uint & PB_LTYPE_MASK {
        PB_LTYPE_BOOL | PB_LTYPE_VARINT | PB_LTYPE_UVARINT | PB_LTYPE_SVARINT => {
            wiretype = PB_WT_VARINT;
        }
        PB_LTYPE_FIXED32 => {
            wiretype = PB_WT_32BIT;
        }
        PB_LTYPE_FIXED64 => {
            wiretype = PB_WT_64BIT;
        }
        PB_LTYPE_BYTES
        | PB_LTYPE_STRING
        | PB_LTYPE_SUBMESSAGE
        | PB_LTYPE_SUBMSG_W_CB
        | PB_LTYPE_FIXED_LENGTH_BYTES => {
            wiretype = PB_WT_STRING;
        }
        _ => {
            (*stream).errmsg = (if !(*stream).errmsg.is_null() {
                (*stream).errmsg
            } else {
                b"invalid field type\0" as *const u8 as *const ::core::ffi::c_char
            });
            return false_0 != 0;
        }
    }
    return pb_encode_tag(stream, wiretype, (*field).tag as uint32_t);
}
#[no_mangle]
pub unsafe extern "C" fn pb_encode_string(
    mut stream: *mut pb_ostream_t,
    mut buffer: *const pb_byte_t,
    mut size: size_t,
) -> bool {
    if !pb_encode_varint(stream, size as uint64_t) {
        return false_0 != 0;
    }
    return pb_write(stream, buffer, size);
}
#[no_mangle]
pub unsafe extern "C" fn pb_encode_submessage(
    mut stream: *mut pb_ostream_t,
    mut fields: *const pb_msgdesc_t,
    mut src_struct: *const ::core::ffi::c_void,
) -> bool {
    let mut substream: pb_ostream_t = pb_ostream_s {
        callback: None,
        state: ::core::ptr::null_mut::<::core::ffi::c_void>(),
        max_size: 0 as size_t,
        bytes_written: 0 as size_t,
        errmsg: ::core::ptr::null::<::core::ffi::c_char>(),
    };
    let mut size: size_t = 0;
    let mut status: bool = false;
    if !pb_encode(&raw mut substream, fields, src_struct) {
        (*stream).errmsg = substream.errmsg;
        return false_0 != 0;
    }
    size = substream.bytes_written;
    if !pb_encode_varint(stream, size as uint64_t) {
        return false_0 != 0;
    }
    if (*stream).callback.is_none() {
        return pb_write(stream, ::core::ptr::null::<pb_byte_t>(), size);
    }
    if (*stream).bytes_written.wrapping_add(size) > (*stream).max_size {
        (*stream).errmsg = (if !(*stream).errmsg.is_null() {
            (*stream).errmsg
        } else {
            b"stream full\0" as *const u8 as *const ::core::ffi::c_char
        });
        return false_0 != 0;
    }
    substream.callback = (*stream).callback;
    substream.state = (*stream).state;
    substream.max_size = size;
    substream.bytes_written = 0 as size_t;
    substream.errmsg = ::core::ptr::null::<::core::ffi::c_char>();
    status = pb_encode(&raw mut substream, fields, src_struct);
    (*stream).bytes_written = (*stream)
        .bytes_written
        .wrapping_add(substream.bytes_written);
    (*stream).state = substream.state;
    (*stream).errmsg = substream.errmsg;
    if substream.bytes_written != size {
        (*stream).errmsg = (if !(*stream).errmsg.is_null() {
            (*stream).errmsg
        } else {
            b"submsg size changed\0" as *const u8 as *const ::core::ffi::c_char
        });
        return false_0 != 0;
    }
    return status;
}
unsafe extern "C" fn pb_enc_bool(
    mut stream: *mut pb_ostream_t,
    mut field: *const pb_field_iter_t,
) -> bool {
    let mut value: uint32_t = (if safe_read_bool((*field).pData) as ::core::ffi::c_int
        != 0
    {
        1 as ::core::ffi::c_int
    } else {
        0 as ::core::ffi::c_int
    }) as uint32_t;
    return pb_encode_varint(stream, value as uint64_t);
}
unsafe extern "C" fn pb_enc_varint(
    mut stream: *mut pb_ostream_t,
    mut field: *const pb_field_iter_t,
) -> bool {
    if (*field).type_0 as ::core::ffi::c_uint & PB_LTYPE_MASK == PB_LTYPE_UVARINT {
        let mut value: uint64_t = 0 as uint64_t;
        if (*field).data_size as usize
            == ::core::mem::size_of::<uint_least8_t>() as usize
        {
            value = *((*field).pData as *const uint_least8_t) as uint64_t;
        } else if (*field).data_size as usize
            == ::core::mem::size_of::<uint_least16_t>() as usize
        {
            value = *((*field).pData as *const uint_least16_t) as uint64_t;
        } else if (*field).data_size as usize
            == ::core::mem::size_of::<uint32_t>() as usize
        {
            value = *((*field).pData as *const uint32_t) as uint64_t;
        } else if (*field).data_size as usize
            == ::core::mem::size_of::<uint64_t>() as usize
        {
            value = *((*field).pData as *const uint64_t);
        } else {
            (*stream).errmsg = (if !(*stream).errmsg.is_null() {
                (*stream).errmsg
            } else {
                b"invalid data_size\0" as *const u8 as *const ::core::ffi::c_char
            });
            return false_0 != 0;
        }
        return pb_encode_varint(stream, value);
    } else {
        let mut value_0: int64_t = 0 as int64_t;
        if (*field).data_size as usize == ::core::mem::size_of::<int_least8_t>() as usize
        {
            value_0 = *((*field).pData as *const int_least8_t) as int64_t;
        } else if (*field).data_size as usize
            == ::core::mem::size_of::<int_least16_t>() as usize
        {
            value_0 = *((*field).pData as *const int_least16_t) as int64_t;
        } else if (*field).data_size as usize
            == ::core::mem::size_of::<int32_t>() as usize
        {
            value_0 = *((*field).pData as *const int32_t) as int64_t;
        } else if (*field).data_size as usize
            == ::core::mem::size_of::<int64_t>() as usize
        {
            value_0 = *((*field).pData as *const int64_t);
        } else {
            (*stream).errmsg = (if !(*stream).errmsg.is_null() {
                (*stream).errmsg
            } else {
                b"invalid data_size\0" as *const u8 as *const ::core::ffi::c_char
            });
            return false_0 != 0;
        }
        if (*field).type_0 as ::core::ffi::c_uint & PB_LTYPE_MASK == PB_LTYPE_SVARINT {
            return pb_encode_svarint(stream, value_0)
        } else {
            return pb_encode_varint(stream, value_0 as uint64_t)
        }
    };
}
unsafe extern "C" fn pb_enc_fixed(
    mut stream: *mut pb_ostream_t,
    mut field: *const pb_field_iter_t,
) -> bool {
    if (*field).data_size as usize == ::core::mem::size_of::<uint32_t>() as usize {
        return pb_encode_fixed32(stream, (*field).pData)
    } else if (*field).data_size as usize == ::core::mem::size_of::<uint64_t>() as usize
    {
        return pb_encode_fixed64(stream, (*field).pData)
    } else {
        (*stream).errmsg = (if !(*stream).errmsg.is_null() {
            (*stream).errmsg
        } else {
            b"invalid data_size\0" as *const u8 as *const ::core::ffi::c_char
        });
        return false_0 != 0;
    };
}
unsafe extern "C" fn pb_enc_bytes(
    mut stream: *mut pb_ostream_t,
    mut field: *const pb_field_iter_t,
) -> bool {
    let mut bytes: *const pb_bytes_array_t = ::core::ptr::null::<pb_bytes_array_t>();
    bytes = (*field).pData as *const pb_bytes_array_t;
    if bytes.is_null() {
        return pb_encode_string(stream, ::core::ptr::null::<pb_byte_t>(), 0 as size_t);
    }
    if (*field).type_0 as ::core::ffi::c_uint & PB_ATYPE_MASK == PB_ATYPE_STATIC
        && (*bytes).size as ::core::ffi::c_ulong
            > ((*field).data_size as ::core::ffi::c_ulong)
                .wrapping_sub(2 as ::core::ffi::c_ulong)
    {
        (*stream).errmsg = (if !(*stream).errmsg.is_null() {
            (*stream).errmsg
        } else {
            b"bytes size exceeded\0" as *const u8 as *const ::core::ffi::c_char
        });
        return false_0 != 0;
    }
    return pb_encode_string(
        stream,
        &raw const (*bytes).bytes as *const pb_byte_t,
        (*bytes).size as size_t,
    );
}
unsafe extern "C" fn pb_enc_string(
    mut stream: *mut pb_ostream_t,
    mut field: *const pb_field_iter_t,
) -> bool {
    let mut size: size_t = 0 as size_t;
    let mut max_size: size_t = (*field).data_size as size_t;
    let mut str: *const ::core::ffi::c_char = (*field).pData
        as *const ::core::ffi::c_char;
    if (*field).type_0 as ::core::ffi::c_uint & PB_ATYPE_MASK == PB_ATYPE_POINTER {
        max_size = -(1 as ::core::ffi::c_int) as size_t;
    } else {
        if max_size == 0 as size_t {
            (*stream).errmsg = (if !(*stream).errmsg.is_null() {
                (*stream).errmsg
            } else {
                b"zero-length string\0" as *const u8 as *const ::core::ffi::c_char
            });
            return false_0 != 0;
        }
        max_size = max_size.wrapping_sub(1 as size_t);
    }
    if str.is_null() {
        size = 0 as size_t;
    } else {
        let mut p: *const ::core::ffi::c_char = str;
        while size < max_size && *p as ::core::ffi::c_int != '\0' as i32 {
            size = size.wrapping_add(1);
            p = p.offset(1);
        }
        if *p as ::core::ffi::c_int != '\0' as i32 {
            (*stream).errmsg = (if !(*stream).errmsg.is_null() {
                (*stream).errmsg
            } else {
                b"unterminated string\0" as *const u8 as *const ::core::ffi::c_char
            });
            return false_0 != 0;
        }
    }
    return pb_encode_string(stream, str as *const pb_byte_t, size);
}
unsafe extern "C" fn pb_enc_submessage(
    mut stream: *mut pb_ostream_t,
    mut field: *const pb_field_iter_t,
) -> bool {
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
        if (*callback).funcs.encode.is_some() {
            if !(*callback)
                .funcs
                .encode
                .expect(
                    "non-null function pointer",
                )(stream, field as *const pb_field_t, &raw mut (*callback).arg)
            {
                return false_0 != 0;
            }
        }
    }
    return pb_encode_submessage(stream, (*field).submsg_desc, (*field).pData);
}
unsafe extern "C" fn pb_enc_fixed_length_bytes(
    mut stream: *mut pb_ostream_t,
    mut field: *const pb_field_iter_t,
) -> bool {
    return pb_encode_string(
        stream,
        (*field).pData as *const pb_byte_t,
        (*field).data_size as size_t,
    );
}
#[repr(C)]
pub struct pb_istream_s {
    _private: [u8; 0],
}
