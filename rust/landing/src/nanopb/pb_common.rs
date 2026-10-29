use crate::sys::*;
pub type __int8_t = i8;
pub type __uint8_t = u8;
pub type __uint16_t = u16;
pub type __uint32_t = u32;
pub type __int_least8_t = __int8_t;
pub type __uint_least16_t = __uint16_t;
pub type uint8_t = __uint8_t;
pub type uint32_t = __uint32_t;
pub type int_least8_t = __int_least8_t;
pub type uint_least16_t = __uint_least16_t;
pub type size_t = usize;
pub type pb_byte_t = uint8_t;
pub type pb_type_t = pb_byte_t;
pub type pb_size_t = uint_least16_t;
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
pub union C2RustUnnamed_0 {
    pub p1: *mut ::core::ffi::c_void,
    pub p2: *const ::core::ffi::c_void,
}
pub const true_0: ::core::ffi::c_int = 1 as ::core::ffi::c_int;
pub const false_0: ::core::ffi::c_int = 0 as ::core::ffi::c_int;
pub const NULL: *mut ::core::ffi::c_void = ::core::ptr::null_mut::<
    ::core::ffi::c_void,
>();
pub const PB_LTYPE_SUBMESSAGE: ::core::ffi::c_uint = 0x8 as ::core::ffi::c_uint;
pub const PB_LTYPE_SUBMSG_W_CB: ::core::ffi::c_uint = 0x9 as ::core::ffi::c_uint;
pub const PB_LTYPE_EXTENSION: ::core::ffi::c_uint = 0xa as ::core::ffi::c_uint;
pub const PB_LTYPE_MASK: ::core::ffi::c_uint = 0xf as ::core::ffi::c_uint;
pub const PB_HTYPE_REQUIRED: ::core::ffi::c_uint = 0 as ::core::ffi::c_uint;
pub const PB_HTYPE_REPEATED: ::core::ffi::c_uint = 0x20 as ::core::ffi::c_uint;
pub const PB_HTYPE_MASK: ::core::ffi::c_uint = 0x30 as ::core::ffi::c_uint;
pub const PB_ATYPE_STATIC: ::core::ffi::c_uint = 0 as ::core::ffi::c_uint;
pub const PB_ATYPE_POINTER: ::core::ffi::c_uint = 0x80 as ::core::ffi::c_uint;
pub const PB_ATYPE_MASK: ::core::ffi::c_uint = 0xc0 as ::core::ffi::c_uint;
unsafe extern "C" fn load_descriptor_values(mut iter: *mut pb_field_iter_t) -> bool {
    let mut word0: uint32_t = 0;
    let mut data_offset: uint32_t = 0;
    let mut size_offset: int_least8_t = 0;
    if (*iter).index as ::core::ffi::c_int
        >= (*(*iter).descriptor).field_count as ::core::ffi::c_int
    {
        return false_0 != 0;
    }
    word0 = *(*(*iter).descriptor).field_info.offset((*iter).field_info_index as isize);
    (*iter).type_0 = (word0 >> 8 as ::core::ffi::c_int & 0xff as uint32_t) as pb_type_t;
    match word0 & 3 as uint32_t {
        0 => {
            (*iter).array_size = 1 as pb_size_t;
            (*iter).tag = (word0 >> 2 as ::core::ffi::c_int & 0x3f as uint32_t)
                as pb_size_t;
            size_offset = (word0 >> 24 as ::core::ffi::c_int & 0xf as uint32_t)
                as int_least8_t;
            data_offset = word0 >> 16 as ::core::ffi::c_int & 0xff as uint32_t;
            (*iter).data_size = (word0 >> 28 as ::core::ffi::c_int & 0xf as uint32_t)
                as pb_size_t;
        }
        1 => {
            let mut word1: uint32_t = *(*(*iter).descriptor)
                .field_info
                .offset(
                    ((*iter).field_info_index as ::core::ffi::c_int
                        + 1 as ::core::ffi::c_int) as isize,
                );
            (*iter).array_size = (word0 >> 16 as ::core::ffi::c_int & 0xfff as uint32_t)
                as pb_size_t;
            (*iter).tag = (word0 >> 2 as ::core::ffi::c_int & 0x3f as uint32_t
                | (word1 >> 28 as ::core::ffi::c_int) << 6 as ::core::ffi::c_int)
                as pb_size_t;
            size_offset = (word0 >> 28 as ::core::ffi::c_int & 0xf as uint32_t)
                as int_least8_t;
            data_offset = word1 & 0xffff as uint32_t;
            (*iter).data_size = (word1 >> 16 as ::core::ffi::c_int & 0xfff as uint32_t)
                as pb_size_t;
        }
        2 => {
            let mut word1_0: uint32_t = *(*(*iter).descriptor)
                .field_info
                .offset(
                    ((*iter).field_info_index as ::core::ffi::c_int
                        + 1 as ::core::ffi::c_int) as isize,
                );
            let mut word2: uint32_t = *(*(*iter).descriptor)
                .field_info
                .offset(
                    ((*iter).field_info_index as ::core::ffi::c_int
                        + 2 as ::core::ffi::c_int) as isize,
                );
            let mut word3: uint32_t = *(*(*iter).descriptor)
                .field_info
                .offset(
                    ((*iter).field_info_index as ::core::ffi::c_int
                        + 3 as ::core::ffi::c_int) as isize,
                );
            (*iter).array_size = (word0 >> 16 as ::core::ffi::c_int) as pb_size_t;
            (*iter).tag = (word0 >> 2 as ::core::ffi::c_int & 0x3f as uint32_t
                | (word1_0 >> 8 as ::core::ffi::c_int) << 6 as ::core::ffi::c_int)
                as pb_size_t;
            size_offset = (word1_0 & 0xff as uint32_t) as int_least8_t;
            data_offset = word2;
            (*iter).data_size = word3 as pb_size_t;
        }
        _ => {
            let mut word1_1: uint32_t = *(*(*iter).descriptor)
                .field_info
                .offset(
                    ((*iter).field_info_index as ::core::ffi::c_int
                        + 1 as ::core::ffi::c_int) as isize,
                );
            let mut word2_0: uint32_t = *(*(*iter).descriptor)
                .field_info
                .offset(
                    ((*iter).field_info_index as ::core::ffi::c_int
                        + 2 as ::core::ffi::c_int) as isize,
                );
            let mut word3_0: uint32_t = *(*(*iter).descriptor)
                .field_info
                .offset(
                    ((*iter).field_info_index as ::core::ffi::c_int
                        + 3 as ::core::ffi::c_int) as isize,
                );
            let mut word4: uint32_t = *(*(*iter).descriptor)
                .field_info
                .offset(
                    ((*iter).field_info_index as ::core::ffi::c_int
                        + 4 as ::core::ffi::c_int) as isize,
                );
            (*iter).array_size = word4 as pb_size_t;
            (*iter).tag = (word0 >> 2 as ::core::ffi::c_int & 0x3f as uint32_t
                | (word1_1 >> 8 as ::core::ffi::c_int) << 6 as ::core::ffi::c_int)
                as pb_size_t;
            size_offset = (word1_1 & 0xff as uint32_t) as int_least8_t;
            data_offset = word2_0;
            (*iter).data_size = word3_0 as pb_size_t;
        }
    }
    if (*iter).message.is_null() {
        (*iter).pField = NULL;
        (*iter).pSize = NULL;
    } else {
        (*iter).pField = ((*iter).message as *mut ::core::ffi::c_char)
            .offset(data_offset as isize) as *mut ::core::ffi::c_void;
        if size_offset != 0 {
            (*iter).pSize = ((*iter).pField as *mut ::core::ffi::c_char)
                .offset(-(size_offset as ::core::ffi::c_int as isize))
                as *mut ::core::ffi::c_void;
        } else if (*iter).type_0 as ::core::ffi::c_uint & PB_HTYPE_MASK
            == PB_HTYPE_REPEATED
            && ((*iter).type_0 as ::core::ffi::c_uint & PB_ATYPE_MASK == PB_ATYPE_STATIC
                || (*iter).type_0 as ::core::ffi::c_uint & PB_ATYPE_MASK
                    == PB_ATYPE_POINTER)
        {
            (*iter).pSize = &raw mut (*iter).array_size as *mut ::core::ffi::c_void;
        } else {
            (*iter).pSize = NULL;
        }
        if (*iter).type_0 as ::core::ffi::c_uint & PB_ATYPE_MASK == PB_ATYPE_POINTER
            && !(*iter).pField.is_null()
        {
            (*iter).pData = *((*iter).pField as *mut *mut ::core::ffi::c_void);
        } else {
            (*iter).pData = (*iter).pField;
        }
    }
    if (*iter).type_0 as ::core::ffi::c_uint & PB_LTYPE_MASK == PB_LTYPE_SUBMESSAGE
        || (*iter).type_0 as ::core::ffi::c_uint & PB_LTYPE_MASK == PB_LTYPE_SUBMSG_W_CB
    {
        (*iter).submsg_desc = *(*(*iter).descriptor)
            .submsg_info
            .offset((*iter).submessage_index as isize);
    } else {
        (*iter).submsg_desc = ::core::ptr::null::<pb_msgdesc_t>();
    }
    return true_0 != 0;
}
unsafe extern "C" fn advance_iterator(mut iter: *mut pb_field_iter_t) {
    (*iter).index = (*iter).index.wrapping_add(1);
    if (*iter).index as ::core::ffi::c_int
        >= (*(*iter).descriptor).field_count as ::core::ffi::c_int
    {
        (*iter).index = 0 as pb_size_t;
        (*iter).field_info_index = 0 as pb_size_t;
        (*iter).submessage_index = 0 as pb_size_t;
        (*iter).required_field_index = 0 as pb_size_t;
    } else {
        let mut prev_descriptor: uint32_t = *(*(*iter).descriptor)
            .field_info
            .offset((*iter).field_info_index as isize);
        let mut prev_type: pb_type_t = (prev_descriptor >> 8 as ::core::ffi::c_int
            & 0xff as uint32_t) as pb_type_t;
        let mut descriptor_len: pb_size_t = ((1 as ::core::ffi::c_int)
            << (prev_descriptor & 3 as uint32_t)) as pb_size_t;
        (*iter).field_info_index = ((*iter).field_info_index as ::core::ffi::c_int
            + descriptor_len as ::core::ffi::c_int) as pb_size_t;
        (*iter).required_field_index = ((*iter).required_field_index
            as ::core::ffi::c_int
            + (prev_type as ::core::ffi::c_uint & PB_HTYPE_MASK == PB_HTYPE_REQUIRED)
                as ::core::ffi::c_int) as pb_size_t;
        (*iter).submessage_index = ((*iter).submessage_index as ::core::ffi::c_int
            + (prev_type as ::core::ffi::c_uint & PB_LTYPE_MASK == PB_LTYPE_SUBMESSAGE
                || prev_type as ::core::ffi::c_uint & PB_LTYPE_MASK
                    == PB_LTYPE_SUBMSG_W_CB) as ::core::ffi::c_int) as pb_size_t;
    };
}
#[no_mangle]
pub unsafe extern "C" fn pb_field_iter_begin(
    mut iter: *mut pb_field_iter_t,
    mut desc: *const pb_msgdesc_t,
    mut message: *mut ::core::ffi::c_void,
) -> bool {
    memset(
        iter as *mut ::core::ffi::c_void,
        0 as ::core::ffi::c_int,
        ::core::mem::size_of::<pb_field_iter_t>() as size_t,
    );
    (*iter).descriptor = desc;
    (*iter).message = message;
    return load_descriptor_values(iter);
}
#[no_mangle]
pub unsafe extern "C" fn pb_field_iter_begin_extension(
    mut iter: *mut pb_field_iter_t,
    mut extension: *mut pb_extension_t,
) -> bool {
    let mut msg: *const pb_msgdesc_t = (*(*extension).type_0).arg as *const pb_msgdesc_t;
    let mut status: bool = false;
    let mut word0: uint32_t = *(*msg)
        .field_info
        .offset(0 as ::core::ffi::c_int as isize);
    if word0 >> 8 as ::core::ffi::c_int & PB_ATYPE_MASK as uint32_t
        == PB_ATYPE_POINTER as uint32_t
    {
        status = pb_field_iter_begin(
            iter,
            msg,
            &raw mut (*extension).dest as *mut ::core::ffi::c_void,
        );
    } else {
        status = pb_field_iter_begin(iter, msg, (*extension).dest);
    }
    (*iter).pSize = &raw mut (*extension).found as *mut ::core::ffi::c_void;
    return status;
}
#[no_mangle]
pub unsafe extern "C" fn pb_field_iter_next(mut iter: *mut pb_field_iter_t) -> bool {
    advance_iterator(iter);
    load_descriptor_values(iter);
    return (*iter).index as ::core::ffi::c_int != 0 as ::core::ffi::c_int;
}
#[no_mangle]
pub unsafe extern "C" fn pb_field_iter_find(
    mut iter: *mut pb_field_iter_t,
    mut tag: uint32_t,
) -> bool {
    if (*iter).tag as uint32_t == tag {
        return true_0 != 0
    } else if tag > (*(*iter).descriptor).largest_tag as uint32_t {
        return false_0 != 0
    } else {
        let mut start: pb_size_t = (*iter).index;
        let mut fieldinfo: uint32_t = 0;
        if tag < (*iter).tag as uint32_t {
            (*iter).index = (*(*iter).descriptor).field_count;
        }
        loop {
            advance_iterator(iter);
            fieldinfo = *(*(*iter).descriptor)
                .field_info
                .offset((*iter).field_info_index as isize);
            if fieldinfo >> 2 as ::core::ffi::c_int & 0x3f as uint32_t
                == tag & 0x3f as uint32_t
            {
                load_descriptor_values(iter);
                if (*iter).tag as uint32_t == tag
                    && (*iter).type_0 as ::core::ffi::c_uint & PB_LTYPE_MASK
                        != PB_LTYPE_EXTENSION
                {
                    return true_0 != 0;
                }
            }
            if !((*iter).index as ::core::ffi::c_int != start as ::core::ffi::c_int) {
                break;
            }
        }
        load_descriptor_values(iter);
        return false_0 != 0;
    };
}
#[no_mangle]
pub unsafe extern "C" fn pb_field_iter_find_extension(
    mut iter: *mut pb_field_iter_t,
) -> bool {
    if (*iter).type_0 as ::core::ffi::c_uint & PB_LTYPE_MASK == PB_LTYPE_EXTENSION {
        return true_0 != 0
    } else {
        let mut start: pb_size_t = (*iter).index;
        let mut fieldinfo: uint32_t = 0;
        loop {
            advance_iterator(iter);
            fieldinfo = *(*(*iter).descriptor)
                .field_info
                .offset((*iter).field_info_index as isize);
            if fieldinfo >> 8 as ::core::ffi::c_int & 0xff as uint32_t
                & PB_LTYPE_MASK as uint32_t == PB_LTYPE_EXTENSION as uint32_t
            {
                return load_descriptor_values(iter);
            }
            if !((*iter).index as ::core::ffi::c_int != start as ::core::ffi::c_int) {
                break;
            }
        }
        load_descriptor_values(iter);
        return false_0 != 0;
    };
}
unsafe extern "C" fn pb_const_cast(
    mut p: *const ::core::ffi::c_void,
) -> *mut ::core::ffi::c_void {
    let mut t: C2RustUnnamed_0 = C2RustUnnamed_0 {
        p1: ::core::ptr::null_mut::<::core::ffi::c_void>(),
    };
    t.p2 = p;
    return t.p1;
}
#[no_mangle]
pub unsafe extern "C" fn pb_field_iter_begin_const(
    mut iter: *mut pb_field_iter_t,
    mut desc: *const pb_msgdesc_t,
    mut message: *const ::core::ffi::c_void,
) -> bool {
    return pb_field_iter_begin(iter, desc, pb_const_cast(message));
}
#[no_mangle]
pub unsafe extern "C" fn pb_field_iter_begin_extension_const(
    mut iter: *mut pb_field_iter_t,
    mut extension: *const pb_extension_t,
) -> bool {
    return pb_field_iter_begin_extension(
        iter,
        pb_const_cast(extension as *const ::core::ffi::c_void) as *mut pb_extension_t,
    );
}
#[no_mangle]
pub unsafe extern "C" fn pb_default_field_callback(
    mut istream: *mut pb_istream_t,
    mut ostream: *mut pb_ostream_t,
    mut field: *const pb_field_t,
) -> bool {
    if (*field).data_size as usize == ::core::mem::size_of::<pb_callback_t>() as usize {
        let mut pCallback: *mut pb_callback_t = (*field).pData as *mut pb_callback_t;
        if !pCallback.is_null() {
            if !istream.is_null() && (*pCallback).funcs.decode.is_some() {
                return (*pCallback)
                    .funcs
                    .decode
                    .expect(
                        "non-null function pointer",
                    )(istream, field, &raw mut (*pCallback).arg);
            }
            if !ostream.is_null() && (*pCallback).funcs.encode.is_some() {
                return (*pCallback)
                    .funcs
                    .encode
                    .expect(
                        "non-null function pointer",
                    )(ostream, field, &raw mut (*pCallback).arg);
            }
        }
    }
    return true_0 != 0;
}
#[repr(C)]
pub struct pb_istream_s {
    _private: [u8; 0],
}
#[repr(C)]
pub struct pb_ostream_s {
    _private: [u8; 0],
}
