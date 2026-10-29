use crate::sys::*;
extern "C" {
    static krpc_schema_Services_msg: pb_msgdesc_t;
    static krpc_schema_Tuple_msg: pb_msgdesc_t;
    static krpc_schema_List_msg: pb_msgdesc_t;
    static krpc_schema_Set_msg: pb_msgdesc_t;
    static krpc_schema_Dictionary_msg: pb_msgdesc_t;
    static krpc_schema_DictionaryEntry_msg: pb_msgdesc_t;
    static krpc_schema_Stream_msg: pb_msgdesc_t;
    static krpc_schema_Event_msg: pb_msgdesc_t;
    static krpc_schema_Status_msg: pb_msgdesc_t;
    fn krpc_malloc(size: size_t) -> *mut ::core::ffi::c_void;
    fn krpc_free(ptr: *mut ::core::ffi::c_void);
    fn pb_decode(
        stream: *mut pb_istream_t,
        fields: *const pb_msgdesc_t,
        dest_struct: *mut ::core::ffi::c_void,
    ) -> bool;
    fn pb_istream_from_buffer(buf: *const pb_byte_t, msglen: size_t) -> pb_istream_t;
    fn pb_read(stream: *mut pb_istream_t, buf: *mut pb_byte_t, count: size_t) -> bool;
    fn pb_decode_varint(stream: *mut pb_istream_t, dest: *mut uint64_t) -> bool;
    fn pb_decode_varint32(stream: *mut pb_istream_t, dest: *mut uint32_t) -> bool;
    fn pb_decode_svarint(stream: *mut pb_istream_t, dest: *mut int64_t) -> bool;
    fn pb_decode_fixed32(
        stream: *mut pb_istream_t,
        dest: *mut ::core::ffi::c_void,
    ) -> bool;
    fn pb_decode_fixed64(
        stream: *mut pb_istream_t,
        dest: *mut ::core::ffi::c_void,
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
pub struct _krpc_schema_ProcedureResult {
    pub error: pb_callback_t,
    pub value: pb_callback_t,
}
pub type krpc_schema_ProcedureResult = _krpc_schema_ProcedureResult;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct _krpc_schema_Services {
    pub services: pb_callback_t,
}
pub type krpc_schema_Services = _krpc_schema_Services;
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
pub struct _krpc_schema_Stream {
    pub id: uint64_t,
}
pub type krpc_schema_Stream = _krpc_schema_Stream;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct _krpc_schema_Event {
    pub has_stream: bool,
    pub stream: krpc_schema_Stream,
}
pub type krpc_schema_Event = _krpc_schema_Event;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct _krpc_schema_Status {
    pub version: [::core::ffi::c_char; 16],
    pub bytes_read: uint64_t,
    pub bytes_written: uint64_t,
    pub bytes_read_rate: ::core::ffi::c_float,
    pub bytes_written_rate: ::core::ffi::c_float,
    pub rpcs_executed: uint64_t,
    pub rpc_rate: ::core::ffi::c_float,
    pub one_rpc_per_update: bool,
    pub max_time_per_update: uint32_t,
    pub adaptive_rate_control: bool,
    pub blocking_recv: bool,
    pub recv_timeout: uint32_t,
    pub time_per_rpc_update: ::core::ffi::c_float,
    pub poll_time_per_rpc_update: ::core::ffi::c_float,
    pub exec_time_per_rpc_update: ::core::ffi::c_float,
    pub stream_rpcs: uint32_t,
    pub stream_rpcs_executed: uint64_t,
    pub stream_rpc_rate: ::core::ffi::c_float,
    pub time_per_stream_update: ::core::ffi::c_float,
}
pub type krpc_schema_Status = _krpc_schema_Status;
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
pub struct krpc_result_t {
    pub message: krpc_schema_ProcedureResult,
    pub size: size_t,
    pub buffer: *mut uint8_t,
}
pub const NULL: *mut ::core::ffi::c_void = ::core::ptr::null_mut::<
    ::core::ffi::c_void,
>();
unsafe extern "C" fn krpc_return_value_decoder(
    mut stream: *mut pb_istream_t,
    mut field: *const pb_field_t,
    mut arg: *mut *mut ::core::ffi::c_void,
) -> bool {
    let mut result: *mut krpc_result_t = *arg as *mut krpc_result_t;
    let mut size: uint32_t = (*stream).bytes_left as uint32_t;
    let mut buffer: *mut uint8_t = krpc_malloc(
        (::core::mem::size_of::<uint8_t>() as size_t).wrapping_mul(size as size_t),
    ) as *mut uint8_t;
    if !pb_read(stream, buffer as *mut pb_byte_t, size as size_t) {
        return false_0 != 0;
    }
    (*result).size = size as size_t;
    (*result).buffer = buffer;
    return true_0 != 0;
}
#[no_mangle]
pub unsafe extern "C" fn krpc_init_result(
    mut result: *mut krpc_result_t,
) -> krpc_error_t {
    (*result).message.value.funcs.decode = Some(
        krpc_return_value_decoder
            as unsafe extern "C" fn(
                *mut pb_istream_t,
                *const pb_field_t,
                *mut *mut ::core::ffi::c_void,
            ) -> bool,
    )
        as Option<
            unsafe extern "C" fn(
                *mut pb_istream_t,
                *const pb_field_t,
                *mut *mut ::core::ffi::c_void,
            ) -> bool,
        >;
    (*result).message.value.arg = result as *mut ::core::ffi::c_void;
    return KRPC_OK;
}
#[no_mangle]
pub unsafe extern "C" fn krpc_free_result(
    mut result: *mut krpc_result_t,
) -> krpc_error_t {
    if !(*result).buffer.is_null() {
        krpc_free((*result).buffer as *mut ::core::ffi::c_void);
        (*result).buffer = ::core::ptr::null_mut::<uint8_t>();
    }
    return KRPC_OK;
}
#[no_mangle]
pub unsafe extern "C" fn krpc_get_return_value(
    mut result: *mut krpc_result_t,
    mut stream: *mut pb_istream_t,
) -> krpc_error_t {
    *stream = pb_istream_from_buffer((*result).buffer, (*result).size);
    return KRPC_OK;
}
#[no_mangle]
pub unsafe extern "C" fn krpc_decode_double(
    mut stream: *mut pb_istream_t,
    mut value: *mut ::core::ffi::c_double,
) -> krpc_error_t {
    if !pb_decode_fixed64(stream, value as *mut ::core::ffi::c_void) {
        return KRPC_ERROR_DECODING_FAILED;
    }
    return KRPC_OK;
}
#[no_mangle]
pub unsafe extern "C" fn krpc_decode_float(
    mut stream: *mut pb_istream_t,
    mut value: *mut ::core::ffi::c_float,
) -> krpc_error_t {
    if !pb_decode_fixed32(stream, value as *mut ::core::ffi::c_void) {
        return KRPC_ERROR_DECODING_FAILED;
    }
    return KRPC_OK;
}
#[no_mangle]
pub unsafe extern "C" fn krpc_decode_int32(
    mut stream: *mut pb_istream_t,
    mut value: *mut int32_t,
) -> krpc_error_t {
    let mut tmp: int64_t = 0;
    if !pb_decode_svarint(stream, &raw mut tmp) {
        return KRPC_ERROR_DECODING_FAILED;
    }
    *value = tmp as int32_t;
    return KRPC_OK;
}
#[no_mangle]
pub unsafe extern "C" fn krpc_decode_int64(
    mut stream: *mut pb_istream_t,
    mut value: *mut int64_t,
) -> krpc_error_t {
    if !pb_decode_svarint(stream, value) {
        return KRPC_ERROR_DECODING_FAILED;
    }
    return KRPC_OK;
}
#[no_mangle]
pub unsafe extern "C" fn krpc_decode_uint32(
    mut stream: *mut pb_istream_t,
    mut value: *mut uint32_t,
) -> krpc_error_t {
    if !pb_decode_varint32(stream, value) {
        return KRPC_ERROR_DECODING_FAILED;
    }
    return KRPC_OK;
}
#[no_mangle]
pub unsafe extern "C" fn krpc_decode_uint64(
    mut stream: *mut pb_istream_t,
    mut value: *mut uint64_t,
) -> krpc_error_t {
    if !pb_decode_varint(stream, value) {
        return KRPC_ERROR_DECODING_FAILED;
    }
    return KRPC_OK;
}
#[no_mangle]
pub unsafe extern "C" fn krpc_decode_bool(
    mut stream: *mut pb_istream_t,
    mut value: *mut bool,
) -> krpc_error_t {
    let mut tmp: uint32_t = 0;
    if !pb_decode_varint32(stream, &raw mut tmp) {
        return KRPC_ERROR_DECODING_FAILED;
    }
    *value = tmp != 0;
    return KRPC_OK;
}
#[no_mangle]
pub unsafe extern "C" fn krpc_decode_object(
    mut stream: *mut pb_istream_t,
    mut value: *mut krpc_object_t,
) -> krpc_error_t {
    if !pb_decode_varint(stream, value as *mut uint64_t) {
        return KRPC_ERROR_DECODING_FAILED;
    }
    return KRPC_OK;
}
#[no_mangle]
pub unsafe extern "C" fn krpc_decode_enum(
    mut stream: *mut pb_istream_t,
    mut value: *mut krpc_enum_t,
) -> krpc_error_t {
    let mut tmp: int64_t = 0;
    if !pb_decode_svarint(stream, &raw mut tmp) {
        return KRPC_ERROR_DECODING_FAILED;
    }
    *value = tmp as krpc_enum_t;
    return KRPC_OK;
}
#[no_mangle]
pub unsafe extern "C" fn krpc_decode_string(
    mut stream: *mut pb_istream_t,
    mut value: *mut *mut ::core::ffi::c_char,
) -> krpc_error_t {
    let mut length: uint32_t = 0;
    if !pb_decode_varint32(stream, &raw mut length) {
        return KRPC_ERROR_DECODING_FAILED;
    }
    if (*value).is_null() {
        *value = krpc_malloc(length.wrapping_add(1 as uint32_t) as size_t)
            as *mut ::core::ffi::c_char;
    }
    if !pb_read(stream, *value as *mut pb_byte_t, length as size_t) {
        return KRPC_ERROR_DECODING_FAILED;
    }
    *(*value).offset(length as isize) = 0 as ::core::ffi::c_char;
    return KRPC_OK;
}
#[no_mangle]
pub unsafe extern "C" fn krpc_decode_bytes(
    mut stream: *mut pb_istream_t,
    mut value: *mut krpc_bytes_t,
) -> krpc_error_t {
    let mut size: uint32_t = 0;
    if !pb_decode_varint32(stream, &raw mut size) {
        return KRPC_ERROR_DECODING_FAILED;
    }
    let mut buffer: *mut pb_byte_t = ::core::ptr::null_mut::<pb_byte_t>();
    if size == 0 as uint32_t {
        (*value).size = 0 as size_t;
        return KRPC_OK;
    }
    if (*value).size > 0 as size_t {
        if (*value).size < size as size_t {
            return KRPC_ERROR_BUFFER_TOO_SMALL;
        }
        buffer = (*value).data as *mut pb_byte_t;
    } else {
        buffer = krpc_malloc(size as size_t) as *mut pb_byte_t;
        if buffer.is_null() {
            return KRPC_ERROR_DECODING_FAILED;
        }
        (*value).size = size as size_t;
        (*value).data = buffer as *mut uint8_t;
    }
    if !pb_read(stream, buffer, size as size_t) {
        return KRPC_ERROR_DECODING_FAILED;
    }
    return KRPC_OK;
}
#[no_mangle]
pub unsafe extern "C" fn krpc_decode_message_Tuple(
    mut stream: *mut pb_istream_t,
    mut value: *mut krpc_schema_Tuple,
) -> krpc_error_t {
    if !pb_decode(
        stream,
        &raw const krpc_schema_Tuple_msg,
        value as *mut ::core::ffi::c_void,
    ) {
        return KRPC_ERROR_DECODING_FAILED;
    }
    return KRPC_OK;
}
#[no_mangle]
pub unsafe extern "C" fn krpc_decode_message_List(
    mut stream: *mut pb_istream_t,
    mut value: *mut krpc_schema_List,
) -> krpc_error_t {
    if !pb_decode(
        stream,
        &raw const krpc_schema_List_msg,
        value as *mut ::core::ffi::c_void,
    ) {
        return KRPC_ERROR_DECODING_FAILED;
    }
    return KRPC_OK;
}
#[no_mangle]
pub unsafe extern "C" fn krpc_decode_message_Set(
    mut stream: *mut pb_istream_t,
    mut value: *mut krpc_schema_Set,
) -> krpc_error_t {
    if !pb_decode(
        stream,
        &raw const krpc_schema_Set_msg,
        value as *mut ::core::ffi::c_void,
    ) {
        return KRPC_ERROR_DECODING_FAILED;
    }
    return KRPC_OK;
}
#[no_mangle]
pub unsafe extern "C" fn krpc_decode_message_Dictionary(
    mut stream: *mut pb_istream_t,
    mut value: *mut krpc_schema_Dictionary,
) -> krpc_error_t {
    if !pb_decode(
        stream,
        &raw const krpc_schema_Dictionary_msg,
        value as *mut ::core::ffi::c_void,
    ) {
        return KRPC_ERROR_DECODING_FAILED;
    }
    return KRPC_OK;
}
#[no_mangle]
pub unsafe extern "C" fn krpc_decode_message_DictionaryEntry(
    mut stream: *mut pb_istream_t,
    mut value: *mut krpc_schema_DictionaryEntry,
) -> krpc_error_t {
    if !pb_decode(
        stream,
        &raw const krpc_schema_DictionaryEntry_msg,
        value as *mut ::core::ffi::c_void,
    ) {
        return KRPC_ERROR_DECODING_FAILED;
    }
    return KRPC_OK;
}
#[no_mangle]
pub unsafe extern "C" fn krpc_decode_message_Event(
    mut stream: *mut pb_istream_t,
    mut value: *mut krpc_schema_Event,
) -> krpc_error_t {
    if !pb_decode(
        stream,
        &raw const krpc_schema_Event_msg,
        value as *mut ::core::ffi::c_void,
    ) {
        return KRPC_ERROR_DECODING_FAILED;
    }
    return KRPC_OK;
}
#[no_mangle]
pub unsafe extern "C" fn krpc_decode_message_Stream(
    mut stream: *mut pb_istream_t,
    mut value: *mut krpc_schema_Stream,
) -> krpc_error_t {
    if !pb_decode(
        stream,
        &raw const krpc_schema_Stream_msg,
        value as *mut ::core::ffi::c_void,
    ) {
        return KRPC_ERROR_DECODING_FAILED;
    }
    return KRPC_OK;
}
#[no_mangle]
pub unsafe extern "C" fn krpc_decode_message_Services(
    mut stream: *mut pb_istream_t,
    mut value: *mut krpc_schema_Services,
) -> krpc_error_t {
    if !pb_decode(
        stream,
        &raw const krpc_schema_Services_msg,
        value as *mut ::core::ffi::c_void,
    ) {
        return KRPC_ERROR_DECODING_FAILED;
    }
    return KRPC_OK;
}
#[no_mangle]
pub unsafe extern "C" fn krpc_decode_message_Status(
    mut stream: *mut pb_istream_t,
    mut value: *mut krpc_schema_Status,
) -> krpc_error_t {
    if !pb_decode(
        stream,
        &raw const krpc_schema_Status_msg,
        value as *mut ::core::ffi::c_void,
    ) {
        return KRPC_ERROR_DECODING_FAILED;
    }
    return KRPC_OK;
}
pub const true_0: ::core::ffi::c_int = 1 as ::core::ffi::c_int;
pub const false_0: ::core::ffi::c_int = 0 as ::core::ffi::c_int;
#[repr(C)]
pub struct pb_ostream_s {
    _private: [u8; 0],
}
