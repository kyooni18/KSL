use crate::sys::*;
extern "C" {
    static krpc_schema_ProcedureCall_msg: pb_msgdesc_t;
    static krpc_schema_Argument_msg: pb_msgdesc_t;
    static krpc_schema_Tuple_msg: pb_msgdesc_t;
    static krpc_schema_List_msg: pb_msgdesc_t;
    static krpc_schema_Set_msg: pb_msgdesc_t;
    static krpc_schema_Dictionary_msg: pb_msgdesc_t;
    static krpc_schema_DictionaryEntry_msg: pb_msgdesc_t;
    fn pb_encode(
        stream: *mut pb_ostream_t,
        fields: *const pb_msgdesc_t,
        src_struct: *const ::core::ffi::c_void,
    ) -> bool;
    fn pb_encode_tag_for_field(
        stream: *mut pb_ostream_t,
        field: *const pb_field_iter_t,
    ) -> bool;
    fn pb_encode_varint(stream: *mut pb_ostream_t, value: uint64_t) -> bool;
    fn pb_encode_svarint(stream: *mut pb_ostream_t, value: int64_t) -> bool;
    fn pb_encode_string(
        stream: *mut pb_ostream_t,
        buffer: *const pb_byte_t,
        size: size_t,
    ) -> bool;
    fn pb_encode_fixed32(
        stream: *mut pb_ostream_t,
        value: *const ::core::ffi::c_void,
    ) -> bool;
    fn pb_encode_fixed64(
        stream: *mut pb_ostream_t,
        value: *const ::core::ffi::c_void,
    ) -> bool;
    fn pb_encode_submessage(
        stream: *mut pb_ostream_t,
        fields: *const pb_msgdesc_t,
        src_struct: *const ::core::ffi::c_void,
    ) -> bool;
}
pub type size_t = usize;
pub type __uint8_t = u8;
pub type __uint16_t = u16;
pub type __int32_t = i32;
pub type __uint32_t = u32;
pub type __int64_t = i64;
pub type __uint64_t = u64;
pub type __uint_least16_t = __uint16_t;
pub type int32_t = __int32_t;
pub type int64_t = __int64_t;
pub type uint8_t = __uint8_t;
pub type uint32_t = __uint32_t;
pub type uint64_t = __uint64_t;
pub type uint_least16_t = __uint_least16_t;
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
#[derive(Copy, Clone)]
#[repr(C)]
pub struct _krpc_schema_ProcedureCall {
    pub arguments: pb_callback_t,
    pub service_id: uint32_t,
    pub procedure_id: uint32_t,
}
pub type krpc_schema_ProcedureCall = _krpc_schema_ProcedureCall;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct _krpc_schema_Argument {
    pub position: uint32_t,
    pub value: pb_callback_t,
}
pub type krpc_schema_Argument = _krpc_schema_Argument;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct _krpc_schema_Tuple {
    pub items: pb_callback_t,
}
pub type krpc_schema_Tuple = _krpc_schema_Tuple;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct _krpc_schema_List {
    pub items: pb_callback_t,
}
pub type krpc_schema_List = _krpc_schema_List;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct _krpc_schema_Set {
    pub items: pb_callback_t,
}
pub type krpc_schema_Set = _krpc_schema_Set;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct _krpc_schema_Dictionary {
    pub entries: pb_callback_t,
}
pub type krpc_schema_Dictionary = _krpc_schema_Dictionary;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct _krpc_schema_DictionaryEntry {
    pub key: pb_callback_t,
    pub value: pb_callback_t,
}
pub type krpc_schema_DictionaryEntry = _krpc_schema_DictionaryEntry;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct krpc_bytes_s {
    pub size: size_t,
    pub data: *mut uint8_t,
}
pub type krpc_bytes_t = krpc_bytes_s;
pub type krpc_object_t = uint64_t;
pub type krpc_enum_t = ::core::ffi::c_int;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct krpc_argument_t {
    pub message: krpc_schema_Argument,
}
#[derive(Copy, Clone)]
#[repr(C)]
pub struct krpc_call_t {
    pub message: krpc_schema_ProcedureCall,
    pub numArguments: size_t,
    pub arguments: *mut krpc_argument_t,
}
pub const NULL: *mut ::core::ffi::c_void = ::core::ptr::null_mut::<
    ::core::ffi::c_void,
>();
#[no_mangle]
pub unsafe extern "C" fn krpc_encode_callback_cstring(
    mut stream: *mut pb_ostream_t,
    mut field: *const pb_field_t,
    mut arg: *const *mut ::core::ffi::c_void,
) -> bool {
    if !pb_encode_tag_for_field(stream, field as *const pb_field_iter_t) {
        return false_0 != 0;
    }
    let mut str: *const ::core::ffi::c_char = *arg as *const ::core::ffi::c_char;
    if !pb_encode_string(stream, str as *const pb_byte_t, strlen(str)) {
        return false_0 != 0;
    }
    return true_0 != 0;
}
unsafe extern "C" fn krpc_encode_callback_arguments(
    mut stream: *mut pb_ostream_t,
    mut field: *const pb_field_t,
    mut arg: *const *mut ::core::ffi::c_void,
) -> bool {
    let mut call: *mut krpc_call_t = *arg as *mut krpc_call_t;
    let mut i: size_t = 0 as size_t;
    while i < (*call).numArguments {
        if !pb_encode_tag_for_field(stream, field as *const pb_field_iter_t) {
            return false_0 != 0;
        }
        let mut argument: *mut krpc_schema_Argument = &raw mut (*(*call)
            .arguments
            .offset(i as isize))
            .message;
        if !pb_encode_submessage(
            stream,
            &raw const krpc_schema_Argument_msg,
            argument as *const ::core::ffi::c_void,
        ) {
            return false_0 != 0;
        }
        i = i.wrapping_add(1);
    }
    return true_0 != 0;
}
#[no_mangle]
pub unsafe extern "C" fn krpc_call(
    mut call: *mut krpc_call_t,
    mut serviceId: uint32_t,
    mut procedureId: uint32_t,
    mut numArguments: size_t,
    mut arguments: *mut krpc_argument_t,
) -> krpc_error_t {
    (*call).message.service_id = serviceId;
    (*call).message.procedure_id = procedureId;
    (*call).numArguments = numArguments;
    (*call).arguments = arguments;
    (*call).message.arguments.funcs.encode = Some(
        krpc_encode_callback_arguments
            as unsafe extern "C" fn(
                *mut pb_ostream_t,
                *const pb_field_t,
                *const *mut ::core::ffi::c_void,
            ) -> bool,
    )
        as Option<
            unsafe extern "C" fn(
                *mut pb_ostream_t,
                *const pb_field_t,
                *const *mut ::core::ffi::c_void,
            ) -> bool,
        >;
    (*call).message.arguments.arg = call as *mut ::core::ffi::c_void;
    return KRPC_OK;
}
#[no_mangle]
pub unsafe extern "C" fn krpc_add_argument(
    mut call: *mut krpc_call_t,
    mut position: uint32_t,
    mut encode: Option<
        unsafe extern "C" fn(
            *mut pb_ostream_t,
            *const pb_field_t,
            *const *mut ::core::ffi::c_void,
        ) -> bool,
    >,
    mut arg: *const ::core::ffi::c_void,
) -> krpc_error_t {
    let mut argument: *mut krpc_argument_t = (*call).arguments.offset(position as isize);
    (*argument).message.position = position;
    (*argument).message.value.funcs.encode = encode;
    (*argument).message.value.arg = arg as *mut ::core::ffi::c_void;
    return KRPC_OK;
}
#[no_mangle]
pub unsafe extern "C" fn krpc_encode_double(
    mut stream: *mut pb_ostream_t,
    mut value: ::core::ffi::c_double,
) -> krpc_error_t {
    if !pb_encode_fixed64(stream, &raw mut value as *const ::core::ffi::c_void) {
        return KRPC_ERROR_ENCODING_FAILED;
    }
    return KRPC_OK;
}
#[no_mangle]
pub unsafe extern "C" fn krpc_encode_float(
    mut stream: *mut pb_ostream_t,
    mut value: ::core::ffi::c_float,
) -> krpc_error_t {
    if !pb_encode_fixed32(stream, &raw mut value as *const ::core::ffi::c_void) {
        return KRPC_ERROR_ENCODING_FAILED;
    }
    return KRPC_OK;
}
#[no_mangle]
pub unsafe extern "C" fn krpc_encode_int32(
    mut stream: *mut pb_ostream_t,
    mut value: int32_t,
) -> krpc_error_t {
    if !pb_encode_svarint(stream, value as int64_t) {
        return KRPC_ERROR_ENCODING_FAILED;
    }
    return KRPC_OK;
}
#[no_mangle]
pub unsafe extern "C" fn krpc_encode_int64(
    mut stream: *mut pb_ostream_t,
    mut value: int64_t,
) -> krpc_error_t {
    if !pb_encode_svarint(stream, value) {
        return KRPC_ERROR_ENCODING_FAILED;
    }
    return KRPC_OK;
}
#[no_mangle]
pub unsafe extern "C" fn krpc_encode_uint32(
    mut stream: *mut pb_ostream_t,
    mut value: uint32_t,
) -> krpc_error_t {
    if !pb_encode_varint(stream, value as uint64_t) {
        return KRPC_ERROR_ENCODING_FAILED;
    }
    return KRPC_OK;
}
#[no_mangle]
pub unsafe extern "C" fn krpc_encode_uint64(
    mut stream: *mut pb_ostream_t,
    mut value: uint64_t,
) -> krpc_error_t {
    if !pb_encode_varint(stream, value) {
        return KRPC_ERROR_ENCODING_FAILED;
    }
    return KRPC_OK;
}
#[no_mangle]
pub unsafe extern "C" fn krpc_encode_bool(
    mut stream: *mut pb_ostream_t,
    mut value: bool,
) -> krpc_error_t {
    if !pb_encode_varint(stream, value as uint64_t) {
        return KRPC_ERROR_ENCODING_FAILED;
    }
    return KRPC_OK;
}
#[no_mangle]
pub unsafe extern "C" fn krpc_encode_object(
    mut stream: *mut pb_ostream_t,
    mut value: krpc_object_t,
) -> krpc_error_t {
    if !pb_encode_varint(stream, value as uint64_t) {
        return KRPC_ERROR_ENCODING_FAILED;
    }
    return KRPC_OK;
}
#[no_mangle]
pub unsafe extern "C" fn krpc_encode_enum(
    mut stream: *mut pb_ostream_t,
    mut value: krpc_enum_t,
) -> krpc_error_t {
    if !pb_encode_svarint(stream, value as int64_t) {
        return KRPC_ERROR_ENCODING_FAILED;
    }
    return KRPC_OK;
}
#[no_mangle]
pub unsafe extern "C" fn krpc_encode_size_double(
    mut size: *mut size_t,
    mut value: ::core::ffi::c_double,
) -> krpc_error_t {
    *size = 8 as size_t;
    return KRPC_OK;
}
#[no_mangle]
pub unsafe extern "C" fn krpc_encode_size_float(
    mut size: *mut size_t,
    mut value: ::core::ffi::c_float,
) -> krpc_error_t {
    *size = 4 as size_t;
    return KRPC_OK;
}
#[no_mangle]
pub unsafe extern "C" fn krpc_encode_size_int32(
    mut size: *mut size_t,
    mut value: int32_t,
) -> krpc_error_t {
    let mut stream: pb_ostream_t = pb_ostream_s {
        callback: None,
        state: ::core::ptr::null_mut::<::core::ffi::c_void>(),
        max_size: 0 as size_t,
        bytes_written: 0 as size_t,
        errmsg: ::core::ptr::null::<::core::ffi::c_char>(),
    };
    if !pb_encode_svarint(&raw mut stream, value as int64_t) {
        return KRPC_ERROR_ENCODING_FAILED;
    }
    *size = stream.bytes_written;
    return KRPC_OK;
}
#[no_mangle]
pub unsafe extern "C" fn krpc_encode_size_int64(
    mut size: *mut size_t,
    mut value: int64_t,
) -> krpc_error_t {
    let mut stream: pb_ostream_t = pb_ostream_s {
        callback: None,
        state: ::core::ptr::null_mut::<::core::ffi::c_void>(),
        max_size: 0 as size_t,
        bytes_written: 0 as size_t,
        errmsg: ::core::ptr::null::<::core::ffi::c_char>(),
    };
    if !pb_encode_svarint(&raw mut stream, value) {
        return KRPC_ERROR_ENCODING_FAILED;
    }
    *size = stream.bytes_written;
    return KRPC_OK;
}
#[no_mangle]
pub unsafe extern "C" fn krpc_encode_size_uint32(
    mut size: *mut size_t,
    mut value: uint32_t,
) -> krpc_error_t {
    let mut stream: pb_ostream_t = pb_ostream_s {
        callback: None,
        state: ::core::ptr::null_mut::<::core::ffi::c_void>(),
        max_size: 0 as size_t,
        bytes_written: 0 as size_t,
        errmsg: ::core::ptr::null::<::core::ffi::c_char>(),
    };
    if !pb_encode_varint(&raw mut stream, value as uint64_t) {
        return KRPC_ERROR_ENCODING_FAILED;
    }
    *size = stream.bytes_written;
    return KRPC_OK;
}
#[no_mangle]
pub unsafe extern "C" fn krpc_encode_size_uint64(
    mut size: *mut size_t,
    mut value: uint64_t,
) -> krpc_error_t {
    let mut stream: pb_ostream_t = pb_ostream_s {
        callback: None,
        state: ::core::ptr::null_mut::<::core::ffi::c_void>(),
        max_size: 0 as size_t,
        bytes_written: 0 as size_t,
        errmsg: ::core::ptr::null::<::core::ffi::c_char>(),
    };
    if !pb_encode_varint(&raw mut stream, value) {
        return KRPC_ERROR_ENCODING_FAILED;
    }
    *size = stream.bytes_written;
    return KRPC_OK;
}
#[no_mangle]
pub unsafe extern "C" fn krpc_encode_size_bool(
    mut size: *mut size_t,
    mut value: bool,
) -> krpc_error_t {
    let mut stream: pb_ostream_t = pb_ostream_s {
        callback: None,
        state: ::core::ptr::null_mut::<::core::ffi::c_void>(),
        max_size: 0 as size_t,
        bytes_written: 0 as size_t,
        errmsg: ::core::ptr::null::<::core::ffi::c_char>(),
    };
    if !pb_encode_varint(&raw mut stream, value as uint64_t) {
        return KRPC_ERROR_ENCODING_FAILED;
    }
    *size = stream.bytes_written;
    return KRPC_OK;
}
#[no_mangle]
pub unsafe extern "C" fn krpc_encode_size_object(
    mut size: *mut size_t,
    mut value: krpc_object_t,
) -> krpc_error_t {
    let mut stream: pb_ostream_t = pb_ostream_s {
        callback: None,
        state: ::core::ptr::null_mut::<::core::ffi::c_void>(),
        max_size: 0 as size_t,
        bytes_written: 0 as size_t,
        errmsg: ::core::ptr::null::<::core::ffi::c_char>(),
    };
    if !pb_encode_varint(&raw mut stream, value as uint64_t) {
        return KRPC_ERROR_ENCODING_FAILED;
    }
    *size = stream.bytes_written;
    return KRPC_OK;
}
#[no_mangle]
pub unsafe extern "C" fn krpc_encode_size_enum(
    mut size: *mut size_t,
    mut value: krpc_enum_t,
) -> krpc_error_t {
    let mut stream: pb_ostream_t = pb_ostream_s {
        callback: None,
        state: ::core::ptr::null_mut::<::core::ffi::c_void>(),
        max_size: 0 as size_t,
        bytes_written: 0 as size_t,
        errmsg: ::core::ptr::null::<::core::ffi::c_char>(),
    };
    if !pb_encode_svarint(&raw mut stream, value as int64_t) {
        return KRPC_ERROR_ENCODING_FAILED;
    }
    *size = stream.bytes_written;
    return KRPC_OK;
}
#[no_mangle]
pub unsafe extern "C" fn krpc_encode_callback_double(
    mut stream: *mut pb_ostream_t,
    mut field: *const pb_field_t,
    mut arg: *const *mut ::core::ffi::c_void,
) -> bool {
    if !pb_encode_tag_for_field(stream, field as *const pb_field_iter_t) {
        return false_0 != 0;
    }
    let mut value: *mut ::core::ffi::c_double = *arg as *mut ::core::ffi::c_double;
    if !pb_encode_varint(stream, 8 as uint64_t) {
        return false_0 != 0;
    }
    if krpc_encode_double(stream, *value) as ::core::ffi::c_int
        != KRPC_OK as ::core::ffi::c_int
    {
        return false_0 != 0;
    }
    return true_0 != 0;
}
#[no_mangle]
pub unsafe extern "C" fn krpc_encode_callback_float(
    mut stream: *mut pb_ostream_t,
    mut field: *const pb_field_t,
    mut arg: *const *mut ::core::ffi::c_void,
) -> bool {
    if !pb_encode_tag_for_field(stream, field as *const pb_field_iter_t) {
        return false_0 != 0;
    }
    let mut value: *mut ::core::ffi::c_float = *arg as *mut ::core::ffi::c_float;
    if !pb_encode_varint(stream, 4 as uint64_t) {
        return false_0 != 0;
    }
    if krpc_encode_float(stream, *value) as ::core::ffi::c_int
        != KRPC_OK as ::core::ffi::c_int
    {
        return false_0 != 0;
    }
    return true_0 != 0;
}
#[no_mangle]
pub unsafe extern "C" fn krpc_encode_callback_int32(
    mut stream: *mut pb_ostream_t,
    mut field: *const pb_field_t,
    mut arg: *const *mut ::core::ffi::c_void,
) -> bool {
    if !pb_encode_tag_for_field(stream, field as *const pb_field_iter_t) {
        return false_0 != 0;
    }
    let mut value: *mut int32_t = *arg as *mut int32_t;
    let mut size: size_t = 0;
    if krpc_encode_size_int32(&raw mut size, *value) as ::core::ffi::c_int
        != KRPC_OK as ::core::ffi::c_int
    {
        return false_0 != 0;
    }
    if !pb_encode_varint(stream, size as uint64_t) {
        return false_0 != 0;
    }
    if krpc_encode_int32(stream, *value) as ::core::ffi::c_int
        != KRPC_OK as ::core::ffi::c_int
    {
        return false_0 != 0;
    }
    return true_0 != 0;
}
#[no_mangle]
pub unsafe extern "C" fn krpc_encode_callback_int64(
    mut stream: *mut pb_ostream_t,
    mut field: *const pb_field_t,
    mut arg: *const *mut ::core::ffi::c_void,
) -> bool {
    if !pb_encode_tag_for_field(stream, field as *const pb_field_iter_t) {
        return false_0 != 0;
    }
    let mut value: *mut int64_t = *arg as *mut int64_t;
    let mut size: size_t = 0;
    if krpc_encode_size_int64(&raw mut size, *value) as ::core::ffi::c_int
        != KRPC_OK as ::core::ffi::c_int
    {
        return false_0 != 0;
    }
    if !pb_encode_varint(stream, size as uint64_t) {
        return false_0 != 0;
    }
    if krpc_encode_int64(stream, *value) as ::core::ffi::c_int
        != KRPC_OK as ::core::ffi::c_int
    {
        return false_0 != 0;
    }
    return true_0 != 0;
}
#[no_mangle]
pub unsafe extern "C" fn krpc_encode_callback_uint32(
    mut stream: *mut pb_ostream_t,
    mut field: *const pb_field_t,
    mut arg: *const *mut ::core::ffi::c_void,
) -> bool {
    if !pb_encode_tag_for_field(stream, field as *const pb_field_iter_t) {
        return false_0 != 0;
    }
    let mut value: *mut uint32_t = *arg as *mut uint32_t;
    let mut size: size_t = 0;
    if krpc_encode_size_uint32(&raw mut size, *value) as ::core::ffi::c_int
        != KRPC_OK as ::core::ffi::c_int
    {
        return false_0 != 0;
    }
    if !pb_encode_varint(stream, size as uint64_t) {
        return false_0 != 0;
    }
    if krpc_encode_uint32(stream, *value) as ::core::ffi::c_int
        != KRPC_OK as ::core::ffi::c_int
    {
        return false_0 != 0;
    }
    return true_0 != 0;
}
#[no_mangle]
pub unsafe extern "C" fn krpc_encode_callback_uint64(
    mut stream: *mut pb_ostream_t,
    mut field: *const pb_field_t,
    mut arg: *const *mut ::core::ffi::c_void,
) -> bool {
    if !pb_encode_tag_for_field(stream, field as *const pb_field_iter_t) {
        return false_0 != 0;
    }
    let mut value: *mut uint64_t = *arg as *mut uint64_t;
    let mut size: size_t = 0;
    if krpc_encode_size_uint64(&raw mut size, *value) as ::core::ffi::c_int
        != KRPC_OK as ::core::ffi::c_int
    {
        return false_0 != 0;
    }
    if !pb_encode_varint(stream, size as uint64_t) {
        return false_0 != 0;
    }
    if krpc_encode_uint64(stream, *value) as ::core::ffi::c_int
        != KRPC_OK as ::core::ffi::c_int
    {
        return false_0 != 0;
    }
    return true_0 != 0;
}
#[no_mangle]
pub unsafe extern "C" fn krpc_encode_callback_bool(
    mut stream: *mut pb_ostream_t,
    mut field: *const pb_field_t,
    mut arg: *const *mut ::core::ffi::c_void,
) -> bool {
    if !pb_encode_tag_for_field(stream, field as *const pb_field_iter_t) {
        return false_0 != 0;
    }
    let mut value: *mut bool = *arg as *mut bool;
    let mut size: size_t = 0;
    if krpc_encode_size_bool(&raw mut size, *value) as ::core::ffi::c_int
        != KRPC_OK as ::core::ffi::c_int
    {
        return false_0 != 0;
    }
    if !pb_encode_varint(stream, size as uint64_t) {
        return false_0 != 0;
    }
    if krpc_encode_bool(stream, *value) as ::core::ffi::c_int
        != KRPC_OK as ::core::ffi::c_int
    {
        return false_0 != 0;
    }
    return true_0 != 0;
}
#[no_mangle]
pub unsafe extern "C" fn krpc_encode_callback_object(
    mut stream: *mut pb_ostream_t,
    mut field: *const pb_field_t,
    mut arg: *const *mut ::core::ffi::c_void,
) -> bool {
    if !pb_encode_tag_for_field(stream, field as *const pb_field_iter_t) {
        return false_0 != 0;
    }
    let mut value: *mut krpc_object_t = *arg as *mut krpc_object_t;
    let mut size: size_t = 0;
    if krpc_encode_size_object(&raw mut size, *value) as ::core::ffi::c_int
        != KRPC_OK as ::core::ffi::c_int
    {
        return false_0 != 0;
    }
    if !pb_encode_varint(stream, size as uint64_t) {
        return false_0 != 0;
    }
    if krpc_encode_object(stream, *value) as ::core::ffi::c_int
        != KRPC_OK as ::core::ffi::c_int
    {
        return false_0 != 0;
    }
    return true_0 != 0;
}
#[no_mangle]
pub unsafe extern "C" fn krpc_encode_callback_enum(
    mut stream: *mut pb_ostream_t,
    mut field: *const pb_field_t,
    mut arg: *const *mut ::core::ffi::c_void,
) -> bool {
    if !pb_encode_tag_for_field(stream, field as *const pb_field_iter_t) {
        return false_0 != 0;
    }
    let mut value: *mut krpc_enum_t = *arg as *mut krpc_enum_t;
    let mut size: size_t = 0;
    if krpc_encode_size_enum(&raw mut size, *value) as ::core::ffi::c_int
        != KRPC_OK as ::core::ffi::c_int
    {
        return false_0 != 0;
    }
    if !pb_encode_varint(stream, size as uint64_t) {
        return false_0 != 0;
    }
    if krpc_encode_enum(stream, *value) as ::core::ffi::c_int
        != KRPC_OK as ::core::ffi::c_int
    {
        return false_0 != 0;
    }
    return true_0 != 0;
}
#[no_mangle]
pub unsafe extern "C" fn krpc_encode_string(
    mut stream: *mut pb_ostream_t,
    mut str: *const ::core::ffi::c_char,
) -> krpc_error_t {
    if !pb_encode_string(stream, str as *const pb_byte_t, strlen(str)) {
        return KRPC_ERROR_ENCODING_FAILED;
    }
    return KRPC_OK;
}
#[no_mangle]
pub unsafe extern "C" fn krpc_encode_bytes(
    mut stream: *mut pb_ostream_t,
    mut bytes: krpc_bytes_t,
) -> krpc_error_t {
    if bytes.size > 0 as size_t && bytes.data.is_null() {
        return KRPC_ERROR_ENCODING_FAILED;
    }
    if !pb_encode_string(stream, bytes.data, bytes.size) {
        return KRPC_ERROR_ENCODING_FAILED;
    }
    return KRPC_OK;
}
#[no_mangle]
pub unsafe extern "C" fn krpc_encode_size_string(
    mut size: *mut size_t,
    mut str: *const ::core::ffi::c_char,
) -> krpc_error_t {
    let mut stream: pb_ostream_t = pb_ostream_s {
        callback: None,
        state: ::core::ptr::null_mut::<::core::ffi::c_void>(),
        max_size: 0 as size_t,
        bytes_written: 0 as size_t,
        errmsg: ::core::ptr::null::<::core::ffi::c_char>(),
    };
    let mut error: krpc_error_t = KRPC_OK;
    error = krpc_encode_string(&raw mut stream, str);
    if error as ::core::ffi::c_int != KRPC_OK as ::core::ffi::c_int {
        return error;
    }
    *size = stream.bytes_written;
    return KRPC_OK;
}
#[no_mangle]
pub unsafe extern "C" fn krpc_encode_size_bytes(
    mut size: *mut size_t,
    mut bytes: krpc_bytes_t,
) -> krpc_error_t {
    let mut stream: pb_ostream_t = pb_ostream_s {
        callback: None,
        state: ::core::ptr::null_mut::<::core::ffi::c_void>(),
        max_size: 0 as size_t,
        bytes_written: 0 as size_t,
        errmsg: ::core::ptr::null::<::core::ffi::c_char>(),
    };
    let mut error: krpc_error_t = KRPC_OK;
    error = krpc_encode_bytes(&raw mut stream, bytes);
    if error as ::core::ffi::c_int != KRPC_OK as ::core::ffi::c_int {
        return error;
    }
    *size = stream.bytes_written;
    return KRPC_OK;
}
#[no_mangle]
pub unsafe extern "C" fn krpc_encode_callback_string(
    mut stream: *mut pb_ostream_t,
    mut field: *const pb_field_t,
    mut arg: *const *mut ::core::ffi::c_void,
) -> bool {
    if !pb_encode_tag_for_field(stream, field as *const pb_field_iter_t) {
        return false_0 != 0;
    }
    let mut value: *const ::core::ffi::c_char = *(*arg
        as *mut *const ::core::ffi::c_char);
    let mut size: size_t = 0;
    if krpc_encode_size_string(&raw mut size, value) as ::core::ffi::c_int
        != KRPC_OK as ::core::ffi::c_int
    {
        return false_0 != 0;
    }
    if !pb_encode_varint(stream, size as uint64_t) {
        return false_0 != 0;
    }
    if krpc_encode_string(stream, value) as ::core::ffi::c_int
        != KRPC_OK as ::core::ffi::c_int
    {
        return false_0 != 0;
    }
    return true_0 != 0;
}
#[no_mangle]
pub unsafe extern "C" fn krpc_encode_callback_bytes(
    mut stream: *mut pb_ostream_t,
    mut field: *const pb_field_t,
    mut arg: *const *mut ::core::ffi::c_void,
) -> bool {
    if !pb_encode_tag_for_field(stream, field as *const pb_field_iter_t) {
        return false_0 != 0;
    }
    let mut value: *mut krpc_bytes_t = *arg as *mut krpc_bytes_t;
    let mut size: size_t = 0;
    if KRPC_OK as ::core::ffi::c_int
        != krpc_encode_size_bytes(&raw mut size, *value) as ::core::ffi::c_int
    {
        return false_0 != 0;
    }
    if !pb_encode_varint(stream, size as uint64_t) {
        return false_0 != 0;
    }
    if krpc_encode_bytes(stream, *value) as ::core::ffi::c_int
        != KRPC_OK as ::core::ffi::c_int
    {
        return false_0 != 0;
    }
    return true_0 != 0;
}
#[no_mangle]
pub unsafe extern "C" fn krpc_encode_message_ProcedureCall(
    mut stream: *mut pb_ostream_t,
    mut value: *const krpc_schema_ProcedureCall,
) -> krpc_error_t {
    if !pb_encode(
        stream,
        &raw const krpc_schema_ProcedureCall_msg,
        value as *const ::core::ffi::c_void,
    ) {
        return KRPC_ERROR_ENCODING_FAILED;
    }
    return KRPC_OK;
}
#[no_mangle]
pub unsafe extern "C" fn krpc_encode_message_Tuple(
    mut stream: *mut pb_ostream_t,
    mut value: *const krpc_schema_Tuple,
) -> krpc_error_t {
    if !pb_encode(
        stream,
        &raw const krpc_schema_Tuple_msg,
        value as *const ::core::ffi::c_void,
    ) {
        return KRPC_ERROR_ENCODING_FAILED;
    }
    return KRPC_OK;
}
#[no_mangle]
pub unsafe extern "C" fn krpc_encode_message_List(
    mut stream: *mut pb_ostream_t,
    mut value: *const krpc_schema_List,
) -> krpc_error_t {
    if !pb_encode(
        stream,
        &raw const krpc_schema_List_msg,
        value as *const ::core::ffi::c_void,
    ) {
        return KRPC_ERROR_ENCODING_FAILED;
    }
    return KRPC_OK;
}
#[no_mangle]
pub unsafe extern "C" fn krpc_encode_message_Set(
    mut stream: *mut pb_ostream_t,
    mut value: *const krpc_schema_Set,
) -> krpc_error_t {
    if !pb_encode(
        stream,
        &raw const krpc_schema_Set_msg,
        value as *const ::core::ffi::c_void,
    ) {
        return KRPC_ERROR_ENCODING_FAILED;
    }
    return KRPC_OK;
}
#[no_mangle]
pub unsafe extern "C" fn krpc_encode_message_Dictionary(
    mut stream: *mut pb_ostream_t,
    mut value: *const krpc_schema_Dictionary,
) -> krpc_error_t {
    if !pb_encode(
        stream,
        &raw const krpc_schema_Dictionary_msg,
        value as *const ::core::ffi::c_void,
    ) {
        return KRPC_ERROR_ENCODING_FAILED;
    }
    return KRPC_OK;
}
#[no_mangle]
pub unsafe extern "C" fn krpc_encode_message_DictionaryEntry(
    mut stream: *mut pb_ostream_t,
    mut value: *const krpc_schema_DictionaryEntry,
) -> krpc_error_t {
    if !pb_encode(
        stream,
        &raw const krpc_schema_DictionaryEntry_msg,
        value as *const ::core::ffi::c_void,
    ) {
        return KRPC_ERROR_ENCODING_FAILED;
    }
    return KRPC_OK;
}
#[no_mangle]
pub unsafe extern "C" fn krpc_encode_size_message_ProcedureCall(
    mut size: *mut size_t,
    mut value: *const krpc_schema_ProcedureCall,
) -> krpc_error_t {
    let mut stream: pb_ostream_t = pb_ostream_s {
        callback: None,
        state: ::core::ptr::null_mut::<::core::ffi::c_void>(),
        max_size: 0 as size_t,
        bytes_written: 0 as size_t,
        errmsg: ::core::ptr::null::<::core::ffi::c_char>(),
    };
    let mut error: krpc_error_t = KRPC_OK;
    error = krpc_encode_message_ProcedureCall(&raw mut stream, value);
    if error as ::core::ffi::c_int != KRPC_OK as ::core::ffi::c_int {
        return error;
    }
    *size = stream.bytes_written;
    return KRPC_OK;
}
#[no_mangle]
pub unsafe extern "C" fn krpc_encode_size_message_Tuple(
    mut size: *mut size_t,
    mut value: *const krpc_schema_Tuple,
) -> krpc_error_t {
    let mut stream: pb_ostream_t = pb_ostream_s {
        callback: None,
        state: ::core::ptr::null_mut::<::core::ffi::c_void>(),
        max_size: 0 as size_t,
        bytes_written: 0 as size_t,
        errmsg: ::core::ptr::null::<::core::ffi::c_char>(),
    };
    let mut error: krpc_error_t = KRPC_OK;
    error = krpc_encode_message_Tuple(&raw mut stream, value);
    if error as ::core::ffi::c_int != KRPC_OK as ::core::ffi::c_int {
        return error;
    }
    *size = stream.bytes_written;
    return KRPC_OK;
}
#[no_mangle]
pub unsafe extern "C" fn krpc_encode_size_message_List(
    mut size: *mut size_t,
    mut value: *const krpc_schema_List,
) -> krpc_error_t {
    let mut stream: pb_ostream_t = pb_ostream_s {
        callback: None,
        state: ::core::ptr::null_mut::<::core::ffi::c_void>(),
        max_size: 0 as size_t,
        bytes_written: 0 as size_t,
        errmsg: ::core::ptr::null::<::core::ffi::c_char>(),
    };
    let mut error: krpc_error_t = KRPC_OK;
    error = krpc_encode_message_List(&raw mut stream, value);
    if error as ::core::ffi::c_int != KRPC_OK as ::core::ffi::c_int {
        return error;
    }
    *size = stream.bytes_written;
    return KRPC_OK;
}
#[no_mangle]
pub unsafe extern "C" fn krpc_encode_size_message_Set(
    mut size: *mut size_t,
    mut value: *const krpc_schema_Set,
) -> krpc_error_t {
    let mut stream: pb_ostream_t = pb_ostream_s {
        callback: None,
        state: ::core::ptr::null_mut::<::core::ffi::c_void>(),
        max_size: 0 as size_t,
        bytes_written: 0 as size_t,
        errmsg: ::core::ptr::null::<::core::ffi::c_char>(),
    };
    let mut error: krpc_error_t = KRPC_OK;
    error = krpc_encode_message_Set(&raw mut stream, value);
    if error as ::core::ffi::c_int != KRPC_OK as ::core::ffi::c_int {
        return error;
    }
    *size = stream.bytes_written;
    return KRPC_OK;
}
#[no_mangle]
pub unsafe extern "C" fn krpc_encode_size_message_Dictionary(
    mut size: *mut size_t,
    mut value: *const krpc_schema_Dictionary,
) -> krpc_error_t {
    let mut stream: pb_ostream_t = pb_ostream_s {
        callback: None,
        state: ::core::ptr::null_mut::<::core::ffi::c_void>(),
        max_size: 0 as size_t,
        bytes_written: 0 as size_t,
        errmsg: ::core::ptr::null::<::core::ffi::c_char>(),
    };
    let mut error: krpc_error_t = KRPC_OK;
    error = krpc_encode_message_Dictionary(&raw mut stream, value);
    if error as ::core::ffi::c_int != KRPC_OK as ::core::ffi::c_int {
        return error;
    }
    *size = stream.bytes_written;
    return KRPC_OK;
}
#[no_mangle]
pub unsafe extern "C" fn krpc_encode_size_message_DictionaryEntry(
    mut size: *mut size_t,
    mut value: *const krpc_schema_DictionaryEntry,
) -> krpc_error_t {
    let mut stream: pb_ostream_t = pb_ostream_s {
        callback: None,
        state: ::core::ptr::null_mut::<::core::ffi::c_void>(),
        max_size: 0 as size_t,
        bytes_written: 0 as size_t,
        errmsg: ::core::ptr::null::<::core::ffi::c_char>(),
    };
    let mut error: krpc_error_t = KRPC_OK;
    error = krpc_encode_message_DictionaryEntry(&raw mut stream, value);
    if error as ::core::ffi::c_int != KRPC_OK as ::core::ffi::c_int {
        return error;
    }
    *size = stream.bytes_written;
    return KRPC_OK;
}
#[no_mangle]
pub unsafe extern "C" fn krpc_encode_callback_message_ProcedureCall(
    mut stream: *mut pb_ostream_t,
    mut field: *const pb_field_t,
    mut arg: *const *mut ::core::ffi::c_void,
) -> bool {
    if !pb_encode_tag_for_field(stream, field as *const pb_field_iter_t) {
        return false_0 != 0;
    }
    let mut value: *mut krpc_schema_ProcedureCall = *arg
        as *mut krpc_schema_ProcedureCall;
    let mut size: size_t = 0;
    if krpc_encode_size_message_ProcedureCall(&raw mut size, value) as ::core::ffi::c_int
        != KRPC_OK as ::core::ffi::c_int
    {
        return false_0 != 0;
    }
    if !pb_encode_varint(stream, size as uint64_t) {
        return false_0 != 0;
    }
    if krpc_encode_message_ProcedureCall(stream, value) as ::core::ffi::c_int
        != KRPC_OK as ::core::ffi::c_int
    {
        return false_0 != 0;
    }
    return true_0 != 0;
}
#[no_mangle]
pub unsafe extern "C" fn krpc_encode_callback_message_Tuple(
    mut stream: *mut pb_ostream_t,
    mut field: *const pb_field_t,
    mut arg: *const *mut ::core::ffi::c_void,
) -> bool {
    if !pb_encode_tag_for_field(stream, field as *const pb_field_iter_t) {
        return false_0 != 0;
    }
    let mut value: *mut krpc_schema_Tuple = *arg as *mut krpc_schema_Tuple;
    let mut size: size_t = 0;
    if krpc_encode_size_message_Tuple(&raw mut size, value) as ::core::ffi::c_int
        != KRPC_OK as ::core::ffi::c_int
    {
        return false_0 != 0;
    }
    if !pb_encode_varint(stream, size as uint64_t) {
        return false_0 != 0;
    }
    if krpc_encode_message_Tuple(stream, value) as ::core::ffi::c_int
        != KRPC_OK as ::core::ffi::c_int
    {
        return false_0 != 0;
    }
    return true_0 != 0;
}
#[no_mangle]
pub unsafe extern "C" fn krpc_encode_callback_message_List(
    mut stream: *mut pb_ostream_t,
    mut field: *const pb_field_t,
    mut arg: *const *mut ::core::ffi::c_void,
) -> bool {
    if !pb_encode_tag_for_field(stream, field as *const pb_field_iter_t) {
        return false_0 != 0;
    }
    let mut value: *mut krpc_schema_List = *arg as *mut krpc_schema_List;
    let mut size: size_t = 0;
    if krpc_encode_size_message_List(&raw mut size, value) as ::core::ffi::c_int
        != KRPC_OK as ::core::ffi::c_int
    {
        return false_0 != 0;
    }
    if !pb_encode_varint(stream, size as uint64_t) {
        return false_0 != 0;
    }
    if krpc_encode_message_List(stream, value) as ::core::ffi::c_int
        != KRPC_OK as ::core::ffi::c_int
    {
        return false_0 != 0;
    }
    return true_0 != 0;
}
#[no_mangle]
pub unsafe extern "C" fn krpc_encode_callback_message_Set(
    mut stream: *mut pb_ostream_t,
    mut field: *const pb_field_t,
    mut arg: *const *mut ::core::ffi::c_void,
) -> bool {
    if !pb_encode_tag_for_field(stream, field as *const pb_field_iter_t) {
        return false_0 != 0;
    }
    let mut value: *mut krpc_schema_Set = *arg as *mut krpc_schema_Set;
    let mut size: size_t = 0;
    if krpc_encode_size_message_Set(&raw mut size, value) as ::core::ffi::c_int
        != KRPC_OK as ::core::ffi::c_int
    {
        return false_0 != 0;
    }
    if !pb_encode_varint(stream, size as uint64_t) {
        return false_0 != 0;
    }
    if krpc_encode_message_Set(stream, value) as ::core::ffi::c_int
        != KRPC_OK as ::core::ffi::c_int
    {
        return false_0 != 0;
    }
    return true_0 != 0;
}
#[no_mangle]
pub unsafe extern "C" fn krpc_encode_callback_message_Dictionary(
    mut stream: *mut pb_ostream_t,
    mut field: *const pb_field_t,
    mut arg: *const *mut ::core::ffi::c_void,
) -> bool {
    if !pb_encode_tag_for_field(stream, field as *const pb_field_iter_t) {
        return false_0 != 0;
    }
    let mut value: *mut krpc_schema_Dictionary = *arg as *mut krpc_schema_Dictionary;
    let mut size: size_t = 0;
    if krpc_encode_size_message_Dictionary(&raw mut size, value) as ::core::ffi::c_int
        != KRPC_OK as ::core::ffi::c_int
    {
        return false_0 != 0;
    }
    if !pb_encode_varint(stream, size as uint64_t) {
        return false_0 != 0;
    }
    if krpc_encode_message_Dictionary(stream, value) as ::core::ffi::c_int
        != KRPC_OK as ::core::ffi::c_int
    {
        return false_0 != 0;
    }
    return true_0 != 0;
}
#[no_mangle]
pub unsafe extern "C" fn krpc_encode_callback_message_DictionaryEntry(
    mut stream: *mut pb_ostream_t,
    mut field: *const pb_field_t,
    mut arg: *const *mut ::core::ffi::c_void,
) -> bool {
    if !pb_encode_tag_for_field(stream, field as *const pb_field_iter_t) {
        return false_0 != 0;
    }
    let mut value: *mut krpc_schema_DictionaryEntry = *arg
        as *mut krpc_schema_DictionaryEntry;
    let mut size: size_t = 0;
    if krpc_encode_size_message_DictionaryEntry(&raw mut size, value)
        as ::core::ffi::c_int != KRPC_OK as ::core::ffi::c_int
    {
        return false_0 != 0;
    }
    if !pb_encode_varint(stream, size as uint64_t) {
        return false_0 != 0;
    }
    if krpc_encode_message_DictionaryEntry(stream, value) as ::core::ffi::c_int
        != KRPC_OK as ::core::ffi::c_int
    {
        return false_0 != 0;
    }
    return true_0 != 0;
}
pub const true_0: ::core::ffi::c_int = 1 as ::core::ffi::c_int;
pub const false_0: ::core::ffi::c_int = 0 as ::core::ffi::c_int;
#[repr(C)]
pub struct pb_istream_s {
    _private: [u8; 0],
}
