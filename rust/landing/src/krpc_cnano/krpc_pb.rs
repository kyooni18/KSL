use crate::sys::*;
extern "C" {
    fn pb_default_field_callback(
        istream: *mut pb_istream_t,
        ostream: *mut pb_ostream_t,
        field: *const pb_field_t,
    ) -> bool;
}
pub type __uint8_t = u8;
pub type __uint16_t = u16;
pub type __int32_t = i32;
pub type __uint32_t = u32;
pub type __uint64_t = u64;
pub type __uint_least16_t = __uint16_t;
pub type int32_t = __int32_t;
pub type uint8_t = __uint8_t;
pub type uint32_t = __uint32_t;
pub type uint64_t = __uint64_t;
pub type uint_least16_t = __uint_least16_t;
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
pub type _krpc_schema_ConnectionRequest_Type = ::core::ffi::c_uint;
pub const krpc_schema_ConnectionRequest_Type_STREAM: _krpc_schema_ConnectionRequest_Type = 1;
pub const krpc_schema_ConnectionRequest_Type_RPC: _krpc_schema_ConnectionRequest_Type = 0;
pub type krpc_schema_ConnectionRequest_Type = _krpc_schema_ConnectionRequest_Type;
pub type _krpc_schema_ConnectionResponse_Status = ::core::ffi::c_uint;
pub const krpc_schema_ConnectionResponse_Status_WRONG_TYPE: _krpc_schema_ConnectionResponse_Status = 3;
pub const krpc_schema_ConnectionResponse_Status_TIMEOUT: _krpc_schema_ConnectionResponse_Status = 2;
pub const krpc_schema_ConnectionResponse_Status_MALFORMED_MESSAGE: _krpc_schema_ConnectionResponse_Status = 1;
pub const krpc_schema_ConnectionResponse_Status_OK: _krpc_schema_ConnectionResponse_Status = 0;
pub type krpc_schema_ConnectionResponse_Status = _krpc_schema_ConnectionResponse_Status;
pub type _krpc_schema_Type_TypeCode = ::core::ffi::c_uint;
pub const krpc_schema_Type_TypeCode_DICTIONARY: _krpc_schema_Type_TypeCode = 303;
pub const krpc_schema_Type_TypeCode_SET: _krpc_schema_Type_TypeCode = 302;
pub const krpc_schema_Type_TypeCode_LIST: _krpc_schema_Type_TypeCode = 301;
pub const krpc_schema_Type_TypeCode_TUPLE: _krpc_schema_Type_TypeCode = 300;
pub const krpc_schema_Type_TypeCode_SERVICES: _krpc_schema_Type_TypeCode = 204;
pub const krpc_schema_Type_TypeCode_STATUS: _krpc_schema_Type_TypeCode = 203;
pub const krpc_schema_Type_TypeCode_STREAM: _krpc_schema_Type_TypeCode = 202;
pub const krpc_schema_Type_TypeCode_PROCEDURE_CALL: _krpc_schema_Type_TypeCode = 201;
pub const krpc_schema_Type_TypeCode_EVENT: _krpc_schema_Type_TypeCode = 200;
pub const krpc_schema_Type_TypeCode_ENUMERATION: _krpc_schema_Type_TypeCode = 101;
pub const krpc_schema_Type_TypeCode_CLASS: _krpc_schema_Type_TypeCode = 100;
pub const krpc_schema_Type_TypeCode_BYTES: _krpc_schema_Type_TypeCode = 9;
pub const krpc_schema_Type_TypeCode_STRING: _krpc_schema_Type_TypeCode = 8;
pub const krpc_schema_Type_TypeCode_BOOL: _krpc_schema_Type_TypeCode = 7;
pub const krpc_schema_Type_TypeCode_UINT64: _krpc_schema_Type_TypeCode = 6;
pub const krpc_schema_Type_TypeCode_UINT32: _krpc_schema_Type_TypeCode = 5;
pub const krpc_schema_Type_TypeCode_SINT64: _krpc_schema_Type_TypeCode = 4;
pub const krpc_schema_Type_TypeCode_SINT32: _krpc_schema_Type_TypeCode = 3;
pub const krpc_schema_Type_TypeCode_FLOAT: _krpc_schema_Type_TypeCode = 2;
pub const krpc_schema_Type_TypeCode_DOUBLE: _krpc_schema_Type_TypeCode = 1;
pub const krpc_schema_Type_TypeCode_NONE: _krpc_schema_Type_TypeCode = 0;
pub type krpc_schema_Type_TypeCode = _krpc_schema_Type_TypeCode;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct _krpc_schema_ConnectionRequest {
    pub type_0: krpc_schema_ConnectionRequest_Type,
    pub client_name: pb_callback_t,
    pub client_identifier: pb_callback_t,
}
pub type krpc_schema_ConnectionRequest = _krpc_schema_ConnectionRequest;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct _krpc_schema_ConnectionResponse {
    pub status: krpc_schema_ConnectionResponse_Status,
    pub message: pb_callback_t,
    pub client_identifier: pb_callback_t,
}
pub type krpc_schema_ConnectionResponse = _krpc_schema_ConnectionResponse;
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
pub struct _krpc_schema_Request {
    pub calls_count: pb_size_t,
    pub calls: [krpc_schema_ProcedureCall; 32],
}
pub type krpc_schema_Request = _krpc_schema_Request;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct _krpc_schema_Argument {
    pub position: uint32_t,
    pub value: pb_callback_t,
}
pub type krpc_schema_Argument = _krpc_schema_Argument;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct _krpc_schema_ProcedureResult {
    pub error: pb_callback_t,
    pub value: pb_callback_t,
}
pub type krpc_schema_ProcedureResult = _krpc_schema_ProcedureResult;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct _krpc_schema_Response {
    pub error: pb_callback_t,
    pub results_count: pb_size_t,
    pub results: [krpc_schema_ProcedureResult; 32],
}
pub type krpc_schema_Response = _krpc_schema_Response;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct _krpc_schema_Error {
    pub service: pb_callback_t,
    pub name: pb_callback_t,
    pub description: pb_callback_t,
    pub stack_trace: pb_callback_t,
}
pub type krpc_schema_Error = _krpc_schema_Error;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct _krpc_schema_StreamUpdate {
    pub results: pb_callback_t,
}
pub type krpc_schema_StreamUpdate = _krpc_schema_StreamUpdate;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct _krpc_schema_StreamResult {
    pub id: uint64_t,
    pub has_result: bool,
    pub result: krpc_schema_ProcedureResult,
}
pub type krpc_schema_StreamResult = _krpc_schema_StreamResult;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct _krpc_schema_Services {
    pub services: pb_callback_t,
}
pub type krpc_schema_Services = _krpc_schema_Services;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct _krpc_schema_Service {
    pub name: pb_callback_t,
    pub procedures: pb_callback_t,
    pub classes: pb_callback_t,
    pub enumerations: pb_callback_t,
    pub exceptions: pb_callback_t,
    pub documentation: pb_callback_t,
    pub deprecated: bool,
    pub deprecated_reason: pb_callback_t,
}
pub type krpc_schema_Service = _krpc_schema_Service;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct _krpc_schema_Class {
    pub name: pb_callback_t,
    pub documentation: pb_callback_t,
    pub deprecated: bool,
    pub deprecated_reason: pb_callback_t,
}
pub type krpc_schema_Class = _krpc_schema_Class;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct _krpc_schema_Enumeration {
    pub name: pb_callback_t,
    pub values: pb_callback_t,
    pub documentation: pb_callback_t,
    pub deprecated: bool,
    pub deprecated_reason: pb_callback_t,
}
pub type krpc_schema_Enumeration = _krpc_schema_Enumeration;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct _krpc_schema_EnumerationValue {
    pub name: pb_callback_t,
    pub value: int32_t,
    pub documentation: pb_callback_t,
    pub deprecated: bool,
    pub deprecated_reason: pb_callback_t,
}
pub type krpc_schema_EnumerationValue = _krpc_schema_EnumerationValue;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct _krpc_schema_Exception {
    pub name: pb_callback_t,
    pub documentation: pb_callback_t,
    pub deprecated: bool,
    pub deprecated_reason: pb_callback_t,
}
pub type krpc_schema_Exception = _krpc_schema_Exception;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct _krpc_schema_Type {
    pub code: krpc_schema_Type_TypeCode,
    pub service: pb_callback_t,
    pub name: pb_callback_t,
    pub types: pb_callback_t,
}
pub type krpc_schema_Type = _krpc_schema_Type;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct _krpc_schema_Procedure {
    pub name: pb_callback_t,
    pub parameters: pb_callback_t,
    pub has_return_type: bool,
    pub return_type: krpc_schema_Type,
    pub return_is_nullable: bool,
    pub documentation: pb_callback_t,
    pub game_scenes: pb_callback_t,
    pub deprecated: bool,
    pub deprecated_reason: pb_callback_t,
}
pub type krpc_schema_Procedure = _krpc_schema_Procedure;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct _krpc_schema_Parameter {
    pub name: pb_callback_t,
    pub has_type: bool,
    pub type_0: krpc_schema_Type,
    pub default_value: pb_callback_t,
    pub nullable: bool,
}
pub type krpc_schema_Parameter = _krpc_schema_Parameter;
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
pub struct _krpc_schema_MultiplexedRequest {
    pub has_connection_request: bool,
    pub connection_request: krpc_schema_ConnectionRequest,
    pub has_request: bool,
    pub request: krpc_schema_Request,
}
pub type krpc_schema_MultiplexedRequest = _krpc_schema_MultiplexedRequest;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct _krpc_schema_MultiplexedResponse {
    pub has_response: bool,
    pub response: krpc_schema_Response,
    pub has_stream_update: bool,
    pub stream_update: krpc_schema_StreamUpdate,
}
pub type krpc_schema_MultiplexedResponse = _krpc_schema_MultiplexedResponse;
pub const NULL: *mut ::core::ffi::c_void = ::core::ptr::null_mut::<
    ::core::ffi::c_void,
>();
pub const PB_HTYPE_REQUIRED: ::core::ffi::c_uint = 0 as ::core::ffi::c_uint;
pub const PB_HTYPE_OPTIONAL: ::core::ffi::c_uint = 0x10 as ::core::ffi::c_uint;
pub const PB_HTYPE_SINGULAR: ::core::ffi::c_uint = 0x10 as ::core::ffi::c_uint;
pub const PB_HTYPE_REPEATED: ::core::ffi::c_uint = 0x20 as ::core::ffi::c_uint;
pub const krpc_schema_ConnectionRequest_DEFAULT: *mut ::core::ffi::c_void = NULL;
pub const krpc_schema_ConnectionResponse_DEFAULT: *mut ::core::ffi::c_void = NULL;
pub const krpc_schema_Request_CALLBACK: *mut ::core::ffi::c_void = NULL;
pub const krpc_schema_Request_DEFAULT: *mut ::core::ffi::c_void = NULL;
pub const krpc_schema_ProcedureCall_DEFAULT: *mut ::core::ffi::c_void = NULL;
pub const krpc_schema_Argument_DEFAULT: *mut ::core::ffi::c_void = NULL;
pub const krpc_schema_Response_DEFAULT: *mut ::core::ffi::c_void = NULL;
pub const krpc_schema_ProcedureResult_DEFAULT: *mut ::core::ffi::c_void = NULL;
pub const krpc_schema_Error_DEFAULT: *mut ::core::ffi::c_void = NULL;
pub const krpc_schema_StreamUpdate_DEFAULT: *mut ::core::ffi::c_void = NULL;
pub const krpc_schema_StreamResult_CALLBACK: *mut ::core::ffi::c_void = NULL;
pub const krpc_schema_StreamResult_DEFAULT: *mut ::core::ffi::c_void = NULL;
pub const krpc_schema_Services_DEFAULT: *mut ::core::ffi::c_void = NULL;
pub const krpc_schema_Service_DEFAULT: *mut ::core::ffi::c_void = NULL;
pub const krpc_schema_Procedure_DEFAULT: *mut ::core::ffi::c_void = NULL;
pub const krpc_schema_Parameter_DEFAULT: *mut ::core::ffi::c_void = NULL;
pub const krpc_schema_Class_DEFAULT: *mut ::core::ffi::c_void = NULL;
pub const krpc_schema_Enumeration_DEFAULT: *mut ::core::ffi::c_void = NULL;
pub const krpc_schema_EnumerationValue_DEFAULT: *mut ::core::ffi::c_void = NULL;
pub const krpc_schema_Exception_DEFAULT: *mut ::core::ffi::c_void = NULL;
pub const krpc_schema_Type_DEFAULT: *mut ::core::ffi::c_void = NULL;
pub const krpc_schema_Tuple_DEFAULT: *mut ::core::ffi::c_void = NULL;
pub const krpc_schema_List_DEFAULT: *mut ::core::ffi::c_void = NULL;
pub const krpc_schema_Set_DEFAULT: *mut ::core::ffi::c_void = NULL;
pub const krpc_schema_Dictionary_DEFAULT: *mut ::core::ffi::c_void = NULL;
pub const krpc_schema_DictionaryEntry_DEFAULT: *mut ::core::ffi::c_void = NULL;
pub const krpc_schema_Stream_CALLBACK: *mut ::core::ffi::c_void = NULL;
pub const krpc_schema_Stream_DEFAULT: *mut ::core::ffi::c_void = NULL;
pub const krpc_schema_Event_CALLBACK: *mut ::core::ffi::c_void = NULL;
pub const krpc_schema_Event_DEFAULT: *mut ::core::ffi::c_void = NULL;
pub const krpc_schema_Status_CALLBACK: *mut ::core::ffi::c_void = NULL;
pub const krpc_schema_Status_DEFAULT: *mut ::core::ffi::c_void = NULL;
pub const krpc_schema_MultiplexedRequest_CALLBACK: *mut ::core::ffi::c_void = NULL;
pub const krpc_schema_MultiplexedRequest_DEFAULT: *mut ::core::ffi::c_void = NULL;
pub const krpc_schema_MultiplexedResponse_CALLBACK: *mut ::core::ffi::c_void = NULL;
pub const krpc_schema_MultiplexedResponse_DEFAULT: *mut ::core::ffi::c_void = NULL;
#[no_mangle]
pub static mut krpc_schema_ConnectionRequest_msg: pb_msgdesc_t = unsafe {
    pb_msgdesc_s {
        field_info: &raw const krpc_schema_ConnectionRequest_field_info
            as *const uint32_t,
        submsg_info: &raw const krpc_schema_ConnectionRequest_submsg_info
            as *const *const pb_msgdesc_t,
        default_value: ::core::ptr::null::<pb_byte_t>(),
        field_callback: Some(
            pb_default_field_callback
                as unsafe extern "C" fn(
                    *mut pb_istream_t,
                    *mut pb_ostream_t,
                    *const pb_field_t,
                ) -> bool,
        ),
        field_count: (0 as ::core::ffi::c_int + 1 as ::core::ffi::c_int
            + 1 as ::core::ffi::c_int + 1 as ::core::ffi::c_int) as pb_size_t,
        required_field_count: (0 as ::core::ffi::c_int
            + (PB_HTYPE_SINGULAR == PB_HTYPE_REQUIRED) as ::core::ffi::c_int
            + (PB_HTYPE_SINGULAR == PB_HTYPE_REQUIRED) as ::core::ffi::c_int
            + (PB_HTYPE_SINGULAR == PB_HTYPE_REQUIRED) as ::core::ffi::c_int)
            as pb_size_t,
        largest_tag: (0 as ::core::ffi::c_int * 0 as ::core::ffi::c_int
            + 1 as ::core::ffi::c_int * 0 as ::core::ffi::c_int
            + 2 as ::core::ffi::c_int * 0 as ::core::ffi::c_int
            + 3 as ::core::ffi::c_int) as pb_size_t,
    }
};
#[no_mangle]
pub static mut krpc_schema_ConnectionRequest_field_info: [uint32_t; 6] = [0; 6];
#[no_mangle]
pub static mut krpc_schema_ConnectionRequest_submsg_info: [*const pb_msgdesc_t; 1] = [
    ::core::ptr::null::<pb_msgdesc_t>(),
];
#[no_mangle]
pub static mut krpc_schema_ConnectionResponse_msg: pb_msgdesc_t = unsafe {
    pb_msgdesc_s {
        field_info: &raw const krpc_schema_ConnectionResponse_field_info
            as *const uint32_t,
        submsg_info: &raw const krpc_schema_ConnectionResponse_submsg_info
            as *const *const pb_msgdesc_t,
        default_value: ::core::ptr::null::<pb_byte_t>(),
        field_callback: Some(
            pb_default_field_callback
                as unsafe extern "C" fn(
                    *mut pb_istream_t,
                    *mut pb_ostream_t,
                    *const pb_field_t,
                ) -> bool,
        ),
        field_count: (0 as ::core::ffi::c_int + 1 as ::core::ffi::c_int
            + 1 as ::core::ffi::c_int + 1 as ::core::ffi::c_int) as pb_size_t,
        required_field_count: (0 as ::core::ffi::c_int
            + (PB_HTYPE_SINGULAR == PB_HTYPE_REQUIRED) as ::core::ffi::c_int
            + (PB_HTYPE_SINGULAR == PB_HTYPE_REQUIRED) as ::core::ffi::c_int
            + (PB_HTYPE_SINGULAR == PB_HTYPE_REQUIRED) as ::core::ffi::c_int)
            as pb_size_t,
        largest_tag: (0 as ::core::ffi::c_int * 0 as ::core::ffi::c_int
            + 1 as ::core::ffi::c_int * 0 as ::core::ffi::c_int
            + 2 as ::core::ffi::c_int * 0 as ::core::ffi::c_int
            + 3 as ::core::ffi::c_int) as pb_size_t,
    }
};
#[no_mangle]
pub static mut krpc_schema_ConnectionResponse_submsg_info: [*const pb_msgdesc_t; 1] = [
    ::core::ptr::null::<pb_msgdesc_t>(),
];
#[no_mangle]
pub static mut krpc_schema_ConnectionResponse_field_info: [uint32_t; 6] = [0; 6];
#[no_mangle]
pub static mut krpc_schema_Request_msg: pb_msgdesc_t = unsafe {
    pb_msgdesc_s {
        field_info: &raw const krpc_schema_Request_field_info as *const uint32_t,
        submsg_info: &raw const krpc_schema_Request_submsg_info
            as *const *const pb_msgdesc_t,
        default_value: ::core::ptr::null::<pb_byte_t>(),
        field_callback: None,
        field_count: (0 as ::core::ffi::c_int + 1 as ::core::ffi::c_int) as pb_size_t,
        required_field_count: (0 as ::core::ffi::c_int
            + (PB_HTYPE_REPEATED == PB_HTYPE_REQUIRED) as ::core::ffi::c_int)
            as pb_size_t,
        largest_tag: (0 as ::core::ffi::c_int * 0 as ::core::ffi::c_int
            + 1 as ::core::ffi::c_int) as pb_size_t,
    }
};
#[no_mangle]
pub static mut krpc_schema_Request_submsg_info: [*const pb_msgdesc_t; 2] = unsafe {
    [&raw const krpc_schema_ProcedureCall_msg, ::core::ptr::null::<pb_msgdesc_t>()]
};
#[no_mangle]
pub static mut krpc_schema_Request_field_info: [uint32_t; 3] = [0; 3];
#[no_mangle]
pub static mut krpc_schema_ProcedureCall_msg: pb_msgdesc_t = unsafe {
    pb_msgdesc_s {
        field_info: &raw const krpc_schema_ProcedureCall_field_info as *const uint32_t,
        submsg_info: &raw const krpc_schema_ProcedureCall_submsg_info
            as *const *const pb_msgdesc_t,
        default_value: ::core::ptr::null::<pb_byte_t>(),
        field_callback: Some(
            pb_default_field_callback
                as unsafe extern "C" fn(
                    *mut pb_istream_t,
                    *mut pb_ostream_t,
                    *const pb_field_t,
                ) -> bool,
        ),
        field_count: (0 as ::core::ffi::c_int + 1 as ::core::ffi::c_int
            + 1 as ::core::ffi::c_int + 1 as ::core::ffi::c_int) as pb_size_t,
        required_field_count: (0 as ::core::ffi::c_int
            + (PB_HTYPE_REPEATED == PB_HTYPE_REQUIRED) as ::core::ffi::c_int
            + (PB_HTYPE_SINGULAR == PB_HTYPE_REQUIRED) as ::core::ffi::c_int
            + (PB_HTYPE_SINGULAR == PB_HTYPE_REQUIRED) as ::core::ffi::c_int)
            as pb_size_t,
        largest_tag: (0 as ::core::ffi::c_int * 0 as ::core::ffi::c_int
            + 3 as ::core::ffi::c_int * 0 as ::core::ffi::c_int
            + 4 as ::core::ffi::c_int * 0 as ::core::ffi::c_int
            + 5 as ::core::ffi::c_int) as pb_size_t,
    }
};
#[no_mangle]
pub static mut krpc_schema_ProcedureCall_submsg_info: [*const pb_msgdesc_t; 2] = unsafe {
    [&raw const krpc_schema_Argument_msg, ::core::ptr::null::<pb_msgdesc_t>()]
};
#[no_mangle]
pub static mut krpc_schema_ProcedureCall_field_info: [uint32_t; 5] = [0; 5];
#[no_mangle]
pub static mut krpc_schema_Argument_msg: pb_msgdesc_t = unsafe {
    pb_msgdesc_s {
        field_info: &raw const krpc_schema_Argument_field_info as *const uint32_t,
        submsg_info: &raw const krpc_schema_Argument_submsg_info
            as *const *const pb_msgdesc_t,
        default_value: ::core::ptr::null::<pb_byte_t>(),
        field_callback: Some(
            pb_default_field_callback
                as unsafe extern "C" fn(
                    *mut pb_istream_t,
                    *mut pb_ostream_t,
                    *const pb_field_t,
                ) -> bool,
        ),
        field_count: (0 as ::core::ffi::c_int + 1 as ::core::ffi::c_int
            + 1 as ::core::ffi::c_int) as pb_size_t,
        required_field_count: (0 as ::core::ffi::c_int
            + (PB_HTYPE_SINGULAR == PB_HTYPE_REQUIRED) as ::core::ffi::c_int
            + (PB_HTYPE_SINGULAR == PB_HTYPE_REQUIRED) as ::core::ffi::c_int)
            as pb_size_t,
        largest_tag: (0 as ::core::ffi::c_int * 0 as ::core::ffi::c_int
            + 1 as ::core::ffi::c_int * 0 as ::core::ffi::c_int
            + 2 as ::core::ffi::c_int) as pb_size_t,
    }
};
#[no_mangle]
pub static mut krpc_schema_Argument_submsg_info: [*const pb_msgdesc_t; 1] = [
    ::core::ptr::null::<pb_msgdesc_t>(),
];
#[no_mangle]
pub static mut krpc_schema_Argument_field_info: [uint32_t; 4] = [0; 4];
#[no_mangle]
pub static mut krpc_schema_Response_field_info: [uint32_t; 5] = [0; 5];
#[no_mangle]
pub static mut krpc_schema_Response_submsg_info: [*const pb_msgdesc_t; 3] = unsafe {
    [
        &raw const krpc_schema_Error_msg,
        &raw const krpc_schema_ProcedureResult_msg,
        ::core::ptr::null::<pb_msgdesc_t>(),
    ]
};
#[no_mangle]
pub static mut krpc_schema_Response_msg: pb_msgdesc_t = unsafe {
    pb_msgdesc_s {
        field_info: &raw const krpc_schema_Response_field_info as *const uint32_t,
        submsg_info: &raw const krpc_schema_Response_submsg_info
            as *const *const pb_msgdesc_t,
        default_value: ::core::ptr::null::<pb_byte_t>(),
        field_callback: Some(
            pb_default_field_callback
                as unsafe extern "C" fn(
                    *mut pb_istream_t,
                    *mut pb_ostream_t,
                    *const pb_field_t,
                ) -> bool,
        ),
        field_count: (0 as ::core::ffi::c_int + 1 as ::core::ffi::c_int
            + 1 as ::core::ffi::c_int) as pb_size_t,
        required_field_count: (0 as ::core::ffi::c_int
            + (PB_HTYPE_OPTIONAL == PB_HTYPE_REQUIRED) as ::core::ffi::c_int
            + (PB_HTYPE_REPEATED == PB_HTYPE_REQUIRED) as ::core::ffi::c_int)
            as pb_size_t,
        largest_tag: (0 as ::core::ffi::c_int * 0 as ::core::ffi::c_int
            + 1 as ::core::ffi::c_int * 0 as ::core::ffi::c_int
            + 2 as ::core::ffi::c_int) as pb_size_t,
    }
};
#[no_mangle]
pub static mut krpc_schema_ProcedureResult_msg: pb_msgdesc_t = unsafe {
    pb_msgdesc_s {
        field_info: &raw const krpc_schema_ProcedureResult_field_info as *const uint32_t,
        submsg_info: &raw const krpc_schema_ProcedureResult_submsg_info
            as *const *const pb_msgdesc_t,
        default_value: ::core::ptr::null::<pb_byte_t>(),
        field_callback: Some(
            pb_default_field_callback
                as unsafe extern "C" fn(
                    *mut pb_istream_t,
                    *mut pb_ostream_t,
                    *const pb_field_t,
                ) -> bool,
        ),
        field_count: (0 as ::core::ffi::c_int + 1 as ::core::ffi::c_int
            + 1 as ::core::ffi::c_int) as pb_size_t,
        required_field_count: (0 as ::core::ffi::c_int
            + (PB_HTYPE_OPTIONAL == PB_HTYPE_REQUIRED) as ::core::ffi::c_int
            + (PB_HTYPE_SINGULAR == PB_HTYPE_REQUIRED) as ::core::ffi::c_int)
            as pb_size_t,
        largest_tag: (0 as ::core::ffi::c_int * 0 as ::core::ffi::c_int
            + 1 as ::core::ffi::c_int * 0 as ::core::ffi::c_int
            + 2 as ::core::ffi::c_int) as pb_size_t,
    }
};
#[no_mangle]
pub static mut krpc_schema_ProcedureResult_submsg_info: [*const pb_msgdesc_t; 2] = unsafe {
    [&raw const krpc_schema_Error_msg, ::core::ptr::null::<pb_msgdesc_t>()]
};
#[no_mangle]
pub static mut krpc_schema_ProcedureResult_field_info: [uint32_t; 5] = [0; 5];
#[no_mangle]
pub static mut krpc_schema_Error_field_info: [uint32_t; 9] = [0; 9];
#[no_mangle]
pub static mut krpc_schema_Error_submsg_info: [*const pb_msgdesc_t; 1] = [
    ::core::ptr::null::<pb_msgdesc_t>(),
];
#[no_mangle]
pub static mut krpc_schema_Error_msg: pb_msgdesc_t = unsafe {
    pb_msgdesc_s {
        field_info: &raw const krpc_schema_Error_field_info as *const uint32_t,
        submsg_info: &raw const krpc_schema_Error_submsg_info
            as *const *const pb_msgdesc_t,
        default_value: ::core::ptr::null::<pb_byte_t>(),
        field_callback: Some(
            pb_default_field_callback
                as unsafe extern "C" fn(
                    *mut pb_istream_t,
                    *mut pb_ostream_t,
                    *const pb_field_t,
                ) -> bool,
        ),
        field_count: (0 as ::core::ffi::c_int + 1 as ::core::ffi::c_int
            + 1 as ::core::ffi::c_int + 1 as ::core::ffi::c_int
            + 1 as ::core::ffi::c_int) as pb_size_t,
        required_field_count: (0 as ::core::ffi::c_int
            + (PB_HTYPE_SINGULAR == PB_HTYPE_REQUIRED) as ::core::ffi::c_int
            + (PB_HTYPE_SINGULAR == PB_HTYPE_REQUIRED) as ::core::ffi::c_int
            + (PB_HTYPE_SINGULAR == PB_HTYPE_REQUIRED) as ::core::ffi::c_int
            + (PB_HTYPE_SINGULAR == PB_HTYPE_REQUIRED) as ::core::ffi::c_int)
            as pb_size_t,
        largest_tag: (0 as ::core::ffi::c_int * 0 as ::core::ffi::c_int
            + 1 as ::core::ffi::c_int * 0 as ::core::ffi::c_int
            + 2 as ::core::ffi::c_int * 0 as ::core::ffi::c_int
            + 3 as ::core::ffi::c_int * 0 as ::core::ffi::c_int
            + 4 as ::core::ffi::c_int) as pb_size_t,
    }
};
#[no_mangle]
pub static mut krpc_schema_StreamUpdate_submsg_info: [*const pb_msgdesc_t; 2] = unsafe {
    [&raw const krpc_schema_StreamResult_msg, ::core::ptr::null::<pb_msgdesc_t>()]
};
#[no_mangle]
pub static mut krpc_schema_StreamUpdate_field_info: [uint32_t; 3] = [0; 3];
#[no_mangle]
pub static mut krpc_schema_StreamUpdate_msg: pb_msgdesc_t = unsafe {
    pb_msgdesc_s {
        field_info: &raw const krpc_schema_StreamUpdate_field_info as *const uint32_t,
        submsg_info: &raw const krpc_schema_StreamUpdate_submsg_info
            as *const *const pb_msgdesc_t,
        default_value: ::core::ptr::null::<pb_byte_t>(),
        field_callback: Some(
            pb_default_field_callback
                as unsafe extern "C" fn(
                    *mut pb_istream_t,
                    *mut pb_ostream_t,
                    *const pb_field_t,
                ) -> bool,
        ),
        field_count: (0 as ::core::ffi::c_int + 1 as ::core::ffi::c_int) as pb_size_t,
        required_field_count: (0 as ::core::ffi::c_int
            + (PB_HTYPE_REPEATED == PB_HTYPE_REQUIRED) as ::core::ffi::c_int)
            as pb_size_t,
        largest_tag: (0 as ::core::ffi::c_int * 0 as ::core::ffi::c_int
            + 1 as ::core::ffi::c_int) as pb_size_t,
    }
};
#[no_mangle]
pub static mut krpc_schema_StreamResult_submsg_info: [*const pb_msgdesc_t; 2] = unsafe {
    [&raw const krpc_schema_ProcedureResult_msg, ::core::ptr::null::<pb_msgdesc_t>()]
};
#[no_mangle]
pub static mut krpc_schema_StreamResult_field_info: [uint32_t; 4] = [0; 4];
#[no_mangle]
pub static mut krpc_schema_StreamResult_msg: pb_msgdesc_t = unsafe {
    pb_msgdesc_s {
        field_info: &raw const krpc_schema_StreamResult_field_info as *const uint32_t,
        submsg_info: &raw const krpc_schema_StreamResult_submsg_info
            as *const *const pb_msgdesc_t,
        default_value: ::core::ptr::null::<pb_byte_t>(),
        field_callback: None,
        field_count: (0 as ::core::ffi::c_int + 1 as ::core::ffi::c_int
            + 1 as ::core::ffi::c_int) as pb_size_t,
        required_field_count: (0 as ::core::ffi::c_int
            + (PB_HTYPE_SINGULAR == PB_HTYPE_REQUIRED) as ::core::ffi::c_int
            + (PB_HTYPE_OPTIONAL == PB_HTYPE_REQUIRED) as ::core::ffi::c_int)
            as pb_size_t,
        largest_tag: (0 as ::core::ffi::c_int * 0 as ::core::ffi::c_int
            + 1 as ::core::ffi::c_int * 0 as ::core::ffi::c_int
            + 2 as ::core::ffi::c_int) as pb_size_t,
    }
};
#[no_mangle]
pub static mut krpc_schema_Services_msg: pb_msgdesc_t = unsafe {
    pb_msgdesc_s {
        field_info: &raw const krpc_schema_Services_field_info as *const uint32_t,
        submsg_info: &raw const krpc_schema_Services_submsg_info
            as *const *const pb_msgdesc_t,
        default_value: ::core::ptr::null::<pb_byte_t>(),
        field_callback: Some(
            pb_default_field_callback
                as unsafe extern "C" fn(
                    *mut pb_istream_t,
                    *mut pb_ostream_t,
                    *const pb_field_t,
                ) -> bool,
        ),
        field_count: (0 as ::core::ffi::c_int + 1 as ::core::ffi::c_int) as pb_size_t,
        required_field_count: (0 as ::core::ffi::c_int
            + (PB_HTYPE_REPEATED == PB_HTYPE_REQUIRED) as ::core::ffi::c_int)
            as pb_size_t,
        largest_tag: (0 as ::core::ffi::c_int * 0 as ::core::ffi::c_int
            + 1 as ::core::ffi::c_int) as pb_size_t,
    }
};
#[no_mangle]
pub static mut krpc_schema_Services_field_info: [uint32_t; 3] = [0; 3];
#[no_mangle]
pub static mut krpc_schema_Services_submsg_info: [*const pb_msgdesc_t; 2] = unsafe {
    [&raw const krpc_schema_Service_msg, ::core::ptr::null::<pb_msgdesc_t>()]
};
#[no_mangle]
pub static mut krpc_schema_Service_field_info: [uint32_t; 16] = [0; 16];
#[no_mangle]
pub static mut krpc_schema_Service_submsg_info: [*const pb_msgdesc_t; 5] = unsafe {
    [
        &raw const krpc_schema_Procedure_msg,
        &raw const krpc_schema_Class_msg,
        &raw const krpc_schema_Enumeration_msg,
        &raw const krpc_schema_Exception_msg,
        ::core::ptr::null::<pb_msgdesc_t>(),
    ]
};
#[no_mangle]
pub static mut krpc_schema_Service_msg: pb_msgdesc_t = unsafe {
    pb_msgdesc_s {
        field_info: &raw const krpc_schema_Service_field_info as *const uint32_t,
        submsg_info: &raw const krpc_schema_Service_submsg_info
            as *const *const pb_msgdesc_t,
        default_value: ::core::ptr::null::<pb_byte_t>(),
        field_callback: Some(
            pb_default_field_callback
                as unsafe extern "C" fn(
                    *mut pb_istream_t,
                    *mut pb_ostream_t,
                    *const pb_field_t,
                ) -> bool,
        ),
        field_count: (0 as ::core::ffi::c_int + 1 as ::core::ffi::c_int
            + 1 as ::core::ffi::c_int + 1 as ::core::ffi::c_int + 1 as ::core::ffi::c_int
            + 1 as ::core::ffi::c_int + 1 as ::core::ffi::c_int + 1 as ::core::ffi::c_int
            + 1 as ::core::ffi::c_int) as pb_size_t,
        required_field_count: (0 as ::core::ffi::c_int
            + (PB_HTYPE_SINGULAR == PB_HTYPE_REQUIRED) as ::core::ffi::c_int
            + (PB_HTYPE_REPEATED == PB_HTYPE_REQUIRED) as ::core::ffi::c_int
            + (PB_HTYPE_REPEATED == PB_HTYPE_REQUIRED) as ::core::ffi::c_int
            + (PB_HTYPE_REPEATED == PB_HTYPE_REQUIRED) as ::core::ffi::c_int
            + (PB_HTYPE_REPEATED == PB_HTYPE_REQUIRED) as ::core::ffi::c_int
            + (PB_HTYPE_SINGULAR == PB_HTYPE_REQUIRED) as ::core::ffi::c_int
            + (PB_HTYPE_SINGULAR == PB_HTYPE_REQUIRED) as ::core::ffi::c_int
            + (PB_HTYPE_SINGULAR == PB_HTYPE_REQUIRED) as ::core::ffi::c_int)
            as pb_size_t,
        largest_tag: (0 as ::core::ffi::c_int * 0 as ::core::ffi::c_int
            + 1 as ::core::ffi::c_int * 0 as ::core::ffi::c_int
            + 2 as ::core::ffi::c_int * 0 as ::core::ffi::c_int
            + 3 as ::core::ffi::c_int * 0 as ::core::ffi::c_int
            + 4 as ::core::ffi::c_int * 0 as ::core::ffi::c_int
            + 5 as ::core::ffi::c_int * 0 as ::core::ffi::c_int
            + 6 as ::core::ffi::c_int * 0 as ::core::ffi::c_int
            + 7 as ::core::ffi::c_int * 0 as ::core::ffi::c_int
            + 8 as ::core::ffi::c_int) as pb_size_t,
    }
};
#[no_mangle]
pub static mut krpc_schema_Procedure_field_info: [uint32_t; 15] = [0; 15];
#[no_mangle]
pub static mut krpc_schema_Procedure_msg: pb_msgdesc_t = unsafe {
    pb_msgdesc_s {
        field_info: &raw const krpc_schema_Procedure_field_info as *const uint32_t,
        submsg_info: &raw const krpc_schema_Procedure_submsg_info
            as *const *const pb_msgdesc_t,
        default_value: ::core::ptr::null::<pb_byte_t>(),
        field_callback: Some(
            pb_default_field_callback
                as unsafe extern "C" fn(
                    *mut pb_istream_t,
                    *mut pb_ostream_t,
                    *const pb_field_t,
                ) -> bool,
        ),
        field_count: (0 as ::core::ffi::c_int + 1 as ::core::ffi::c_int
            + 1 as ::core::ffi::c_int + 1 as ::core::ffi::c_int + 1 as ::core::ffi::c_int
            + 1 as ::core::ffi::c_int + 1 as ::core::ffi::c_int + 1 as ::core::ffi::c_int
            + 1 as ::core::ffi::c_int) as pb_size_t,
        required_field_count: (0 as ::core::ffi::c_int
            + (PB_HTYPE_SINGULAR == PB_HTYPE_REQUIRED) as ::core::ffi::c_int
            + (PB_HTYPE_REPEATED == PB_HTYPE_REQUIRED) as ::core::ffi::c_int
            + (PB_HTYPE_OPTIONAL == PB_HTYPE_REQUIRED) as ::core::ffi::c_int
            + (PB_HTYPE_SINGULAR == PB_HTYPE_REQUIRED) as ::core::ffi::c_int
            + (PB_HTYPE_SINGULAR == PB_HTYPE_REQUIRED) as ::core::ffi::c_int
            + (PB_HTYPE_REPEATED == PB_HTYPE_REQUIRED) as ::core::ffi::c_int
            + (PB_HTYPE_SINGULAR == PB_HTYPE_REQUIRED) as ::core::ffi::c_int
            + (PB_HTYPE_SINGULAR == PB_HTYPE_REQUIRED) as ::core::ffi::c_int)
            as pb_size_t,
        largest_tag: (0 as ::core::ffi::c_int * 0 as ::core::ffi::c_int
            + 1 as ::core::ffi::c_int * 0 as ::core::ffi::c_int
            + 2 as ::core::ffi::c_int * 0 as ::core::ffi::c_int
            + 3 as ::core::ffi::c_int * 0 as ::core::ffi::c_int
            + 4 as ::core::ffi::c_int * 0 as ::core::ffi::c_int
            + 5 as ::core::ffi::c_int * 0 as ::core::ffi::c_int
            + 6 as ::core::ffi::c_int * 0 as ::core::ffi::c_int
            + 7 as ::core::ffi::c_int * 0 as ::core::ffi::c_int
            + 8 as ::core::ffi::c_int) as pb_size_t,
    }
};
#[no_mangle]
pub static mut krpc_schema_Procedure_submsg_info: [*const pb_msgdesc_t; 3] = unsafe {
    [
        &raw const krpc_schema_Parameter_msg,
        &raw const krpc_schema_Type_msg,
        ::core::ptr::null::<pb_msgdesc_t>(),
    ]
};
#[no_mangle]
pub static mut krpc_schema_Parameter_field_info: [uint32_t; 8] = [0; 8];
#[no_mangle]
pub static mut krpc_schema_Parameter_submsg_info: [*const pb_msgdesc_t; 2] = unsafe {
    [&raw const krpc_schema_Type_msg, ::core::ptr::null::<pb_msgdesc_t>()]
};
#[no_mangle]
pub static mut krpc_schema_Parameter_msg: pb_msgdesc_t = unsafe {
    pb_msgdesc_s {
        field_info: &raw const krpc_schema_Parameter_field_info as *const uint32_t,
        submsg_info: &raw const krpc_schema_Parameter_submsg_info
            as *const *const pb_msgdesc_t,
        default_value: ::core::ptr::null::<pb_byte_t>(),
        field_callback: Some(
            pb_default_field_callback
                as unsafe extern "C" fn(
                    *mut pb_istream_t,
                    *mut pb_ostream_t,
                    *const pb_field_t,
                ) -> bool,
        ),
        field_count: (0 as ::core::ffi::c_int + 1 as ::core::ffi::c_int
            + 1 as ::core::ffi::c_int + 1 as ::core::ffi::c_int
            + 1 as ::core::ffi::c_int) as pb_size_t,
        required_field_count: (0 as ::core::ffi::c_int
            + (PB_HTYPE_SINGULAR == PB_HTYPE_REQUIRED) as ::core::ffi::c_int
            + (PB_HTYPE_OPTIONAL == PB_HTYPE_REQUIRED) as ::core::ffi::c_int
            + (PB_HTYPE_SINGULAR == PB_HTYPE_REQUIRED) as ::core::ffi::c_int
            + (PB_HTYPE_SINGULAR == PB_HTYPE_REQUIRED) as ::core::ffi::c_int)
            as pb_size_t,
        largest_tag: (0 as ::core::ffi::c_int * 0 as ::core::ffi::c_int
            + 1 as ::core::ffi::c_int * 0 as ::core::ffi::c_int
            + 2 as ::core::ffi::c_int * 0 as ::core::ffi::c_int
            + 3 as ::core::ffi::c_int * 0 as ::core::ffi::c_int
            + 4 as ::core::ffi::c_int) as pb_size_t,
    }
};
#[no_mangle]
pub static mut krpc_schema_Class_field_info: [uint32_t; 8] = [0; 8];
#[no_mangle]
pub static mut krpc_schema_Class_msg: pb_msgdesc_t = unsafe {
    pb_msgdesc_s {
        field_info: &raw const krpc_schema_Class_field_info as *const uint32_t,
        submsg_info: &raw const krpc_schema_Class_submsg_info
            as *const *const pb_msgdesc_t,
        default_value: ::core::ptr::null::<pb_byte_t>(),
        field_callback: Some(
            pb_default_field_callback
                as unsafe extern "C" fn(
                    *mut pb_istream_t,
                    *mut pb_ostream_t,
                    *const pb_field_t,
                ) -> bool,
        ),
        field_count: (0 as ::core::ffi::c_int + 1 as ::core::ffi::c_int
            + 1 as ::core::ffi::c_int + 1 as ::core::ffi::c_int
            + 1 as ::core::ffi::c_int) as pb_size_t,
        required_field_count: (0 as ::core::ffi::c_int
            + (PB_HTYPE_SINGULAR == PB_HTYPE_REQUIRED) as ::core::ffi::c_int
            + (PB_HTYPE_SINGULAR == PB_HTYPE_REQUIRED) as ::core::ffi::c_int
            + (PB_HTYPE_SINGULAR == PB_HTYPE_REQUIRED) as ::core::ffi::c_int
            + (PB_HTYPE_SINGULAR == PB_HTYPE_REQUIRED) as ::core::ffi::c_int)
            as pb_size_t,
        largest_tag: (0 as ::core::ffi::c_int * 0 as ::core::ffi::c_int
            + 1 as ::core::ffi::c_int * 0 as ::core::ffi::c_int
            + 2 as ::core::ffi::c_int * 0 as ::core::ffi::c_int
            + 3 as ::core::ffi::c_int * 0 as ::core::ffi::c_int
            + 4 as ::core::ffi::c_int) as pb_size_t,
    }
};
#[no_mangle]
pub static mut krpc_schema_Class_submsg_info: [*const pb_msgdesc_t; 1] = [
    ::core::ptr::null::<pb_msgdesc_t>(),
];
#[no_mangle]
pub static mut krpc_schema_Enumeration_msg: pb_msgdesc_t = unsafe {
    pb_msgdesc_s {
        field_info: &raw const krpc_schema_Enumeration_field_info as *const uint32_t,
        submsg_info: &raw const krpc_schema_Enumeration_submsg_info
            as *const *const pb_msgdesc_t,
        default_value: ::core::ptr::null::<pb_byte_t>(),
        field_callback: Some(
            pb_default_field_callback
                as unsafe extern "C" fn(
                    *mut pb_istream_t,
                    *mut pb_ostream_t,
                    *const pb_field_t,
                ) -> bool,
        ),
        field_count: (0 as ::core::ffi::c_int + 1 as ::core::ffi::c_int
            + 1 as ::core::ffi::c_int + 1 as ::core::ffi::c_int + 1 as ::core::ffi::c_int
            + 1 as ::core::ffi::c_int) as pb_size_t,
        required_field_count: (0 as ::core::ffi::c_int
            + (PB_HTYPE_SINGULAR == PB_HTYPE_REQUIRED) as ::core::ffi::c_int
            + (PB_HTYPE_REPEATED == PB_HTYPE_REQUIRED) as ::core::ffi::c_int
            + (PB_HTYPE_SINGULAR == PB_HTYPE_REQUIRED) as ::core::ffi::c_int
            + (PB_HTYPE_SINGULAR == PB_HTYPE_REQUIRED) as ::core::ffi::c_int
            + (PB_HTYPE_SINGULAR == PB_HTYPE_REQUIRED) as ::core::ffi::c_int)
            as pb_size_t,
        largest_tag: (0 as ::core::ffi::c_int * 0 as ::core::ffi::c_int
            + 1 as ::core::ffi::c_int * 0 as ::core::ffi::c_int
            + 2 as ::core::ffi::c_int * 0 as ::core::ffi::c_int
            + 3 as ::core::ffi::c_int * 0 as ::core::ffi::c_int
            + 4 as ::core::ffi::c_int * 0 as ::core::ffi::c_int
            + 5 as ::core::ffi::c_int) as pb_size_t,
    }
};
#[no_mangle]
pub static mut krpc_schema_Enumeration_submsg_info: [*const pb_msgdesc_t; 2] = unsafe {
    [&raw const krpc_schema_EnumerationValue_msg, ::core::ptr::null::<pb_msgdesc_t>()]
};
#[no_mangle]
pub static mut krpc_schema_Enumeration_field_info: [uint32_t; 10] = [0; 10];
#[no_mangle]
pub static mut krpc_schema_EnumerationValue_msg: pb_msgdesc_t = unsafe {
    pb_msgdesc_s {
        field_info: &raw const krpc_schema_EnumerationValue_field_info
            as *const uint32_t,
        submsg_info: &raw const krpc_schema_EnumerationValue_submsg_info
            as *const *const pb_msgdesc_t,
        default_value: ::core::ptr::null::<pb_byte_t>(),
        field_callback: Some(
            pb_default_field_callback
                as unsafe extern "C" fn(
                    *mut pb_istream_t,
                    *mut pb_ostream_t,
                    *const pb_field_t,
                ) -> bool,
        ),
        field_count: (0 as ::core::ffi::c_int + 1 as ::core::ffi::c_int
            + 1 as ::core::ffi::c_int + 1 as ::core::ffi::c_int + 1 as ::core::ffi::c_int
            + 1 as ::core::ffi::c_int) as pb_size_t,
        required_field_count: (0 as ::core::ffi::c_int
            + (PB_HTYPE_SINGULAR == PB_HTYPE_REQUIRED) as ::core::ffi::c_int
            + (PB_HTYPE_SINGULAR == PB_HTYPE_REQUIRED) as ::core::ffi::c_int
            + (PB_HTYPE_SINGULAR == PB_HTYPE_REQUIRED) as ::core::ffi::c_int
            + (PB_HTYPE_SINGULAR == PB_HTYPE_REQUIRED) as ::core::ffi::c_int
            + (PB_HTYPE_SINGULAR == PB_HTYPE_REQUIRED) as ::core::ffi::c_int)
            as pb_size_t,
        largest_tag: (0 as ::core::ffi::c_int * 0 as ::core::ffi::c_int
            + 1 as ::core::ffi::c_int * 0 as ::core::ffi::c_int
            + 2 as ::core::ffi::c_int * 0 as ::core::ffi::c_int
            + 3 as ::core::ffi::c_int * 0 as ::core::ffi::c_int
            + 4 as ::core::ffi::c_int * 0 as ::core::ffi::c_int
            + 5 as ::core::ffi::c_int) as pb_size_t,
    }
};
#[no_mangle]
pub static mut krpc_schema_EnumerationValue_field_info: [uint32_t; 9] = [0; 9];
#[no_mangle]
pub static mut krpc_schema_EnumerationValue_submsg_info: [*const pb_msgdesc_t; 1] = [
    ::core::ptr::null::<pb_msgdesc_t>(),
];
#[no_mangle]
pub static mut krpc_schema_Exception_field_info: [uint32_t; 8] = [0; 8];
#[no_mangle]
pub static mut krpc_schema_Exception_submsg_info: [*const pb_msgdesc_t; 1] = [
    ::core::ptr::null::<pb_msgdesc_t>(),
];
#[no_mangle]
pub static mut krpc_schema_Exception_msg: pb_msgdesc_t = unsafe {
    pb_msgdesc_s {
        field_info: &raw const krpc_schema_Exception_field_info as *const uint32_t,
        submsg_info: &raw const krpc_schema_Exception_submsg_info
            as *const *const pb_msgdesc_t,
        default_value: ::core::ptr::null::<pb_byte_t>(),
        field_callback: Some(
            pb_default_field_callback
                as unsafe extern "C" fn(
                    *mut pb_istream_t,
                    *mut pb_ostream_t,
                    *const pb_field_t,
                ) -> bool,
        ),
        field_count: (0 as ::core::ffi::c_int + 1 as ::core::ffi::c_int
            + 1 as ::core::ffi::c_int + 1 as ::core::ffi::c_int
            + 1 as ::core::ffi::c_int) as pb_size_t,
        required_field_count: (0 as ::core::ffi::c_int
            + (PB_HTYPE_SINGULAR == PB_HTYPE_REQUIRED) as ::core::ffi::c_int
            + (PB_HTYPE_SINGULAR == PB_HTYPE_REQUIRED) as ::core::ffi::c_int
            + (PB_HTYPE_SINGULAR == PB_HTYPE_REQUIRED) as ::core::ffi::c_int
            + (PB_HTYPE_SINGULAR == PB_HTYPE_REQUIRED) as ::core::ffi::c_int)
            as pb_size_t,
        largest_tag: (0 as ::core::ffi::c_int * 0 as ::core::ffi::c_int
            + 1 as ::core::ffi::c_int * 0 as ::core::ffi::c_int
            + 2 as ::core::ffi::c_int * 0 as ::core::ffi::c_int
            + 3 as ::core::ffi::c_int * 0 as ::core::ffi::c_int
            + 4 as ::core::ffi::c_int) as pb_size_t,
    }
};
#[no_mangle]
pub static mut krpc_schema_Type_submsg_info: [*const pb_msgdesc_t; 2] = unsafe {
    [&raw const krpc_schema_Type_msg, ::core::ptr::null::<pb_msgdesc_t>()]
};
#[no_mangle]
pub static mut krpc_schema_Type_msg: pb_msgdesc_t = unsafe {
    pb_msgdesc_s {
        field_info: &raw const krpc_schema_Type_field_info as *const uint32_t,
        submsg_info: &raw const krpc_schema_Type_submsg_info
            as *const *const pb_msgdesc_t,
        default_value: ::core::ptr::null::<pb_byte_t>(),
        field_callback: Some(
            pb_default_field_callback
                as unsafe extern "C" fn(
                    *mut pb_istream_t,
                    *mut pb_ostream_t,
                    *const pb_field_t,
                ) -> bool,
        ),
        field_count: (0 as ::core::ffi::c_int + 1 as ::core::ffi::c_int
            + 1 as ::core::ffi::c_int + 1 as ::core::ffi::c_int
            + 1 as ::core::ffi::c_int) as pb_size_t,
        required_field_count: (0 as ::core::ffi::c_int
            + (PB_HTYPE_SINGULAR == PB_HTYPE_REQUIRED) as ::core::ffi::c_int
            + (PB_HTYPE_SINGULAR == PB_HTYPE_REQUIRED) as ::core::ffi::c_int
            + (PB_HTYPE_SINGULAR == PB_HTYPE_REQUIRED) as ::core::ffi::c_int
            + (PB_HTYPE_REPEATED == PB_HTYPE_REQUIRED) as ::core::ffi::c_int)
            as pb_size_t,
        largest_tag: (0 as ::core::ffi::c_int * 0 as ::core::ffi::c_int
            + 1 as ::core::ffi::c_int * 0 as ::core::ffi::c_int
            + 2 as ::core::ffi::c_int * 0 as ::core::ffi::c_int
            + 3 as ::core::ffi::c_int * 0 as ::core::ffi::c_int
            + 4 as ::core::ffi::c_int) as pb_size_t,
    }
};
#[no_mangle]
pub static mut krpc_schema_Type_field_info: [uint32_t; 8] = [0; 8];
#[no_mangle]
pub static mut krpc_schema_Tuple_msg: pb_msgdesc_t = unsafe {
    pb_msgdesc_s {
        field_info: &raw const krpc_schema_Tuple_field_info as *const uint32_t,
        submsg_info: &raw const krpc_schema_Tuple_submsg_info
            as *const *const pb_msgdesc_t,
        default_value: ::core::ptr::null::<pb_byte_t>(),
        field_callback: Some(
            pb_default_field_callback
                as unsafe extern "C" fn(
                    *mut pb_istream_t,
                    *mut pb_ostream_t,
                    *const pb_field_t,
                ) -> bool,
        ),
        field_count: (0 as ::core::ffi::c_int + 1 as ::core::ffi::c_int) as pb_size_t,
        required_field_count: (0 as ::core::ffi::c_int
            + (PB_HTYPE_REPEATED == PB_HTYPE_REQUIRED) as ::core::ffi::c_int)
            as pb_size_t,
        largest_tag: (0 as ::core::ffi::c_int * 0 as ::core::ffi::c_int
            + 1 as ::core::ffi::c_int) as pb_size_t,
    }
};
#[no_mangle]
pub static mut krpc_schema_Tuple_submsg_info: [*const pb_msgdesc_t; 1] = [
    ::core::ptr::null::<pb_msgdesc_t>(),
];
#[no_mangle]
pub static mut krpc_schema_Tuple_field_info: [uint32_t; 3] = [0; 3];
#[no_mangle]
pub static mut krpc_schema_List_msg: pb_msgdesc_t = unsafe {
    pb_msgdesc_s {
        field_info: &raw const krpc_schema_List_field_info as *const uint32_t,
        submsg_info: &raw const krpc_schema_List_submsg_info
            as *const *const pb_msgdesc_t,
        default_value: ::core::ptr::null::<pb_byte_t>(),
        field_callback: Some(
            pb_default_field_callback
                as unsafe extern "C" fn(
                    *mut pb_istream_t,
                    *mut pb_ostream_t,
                    *const pb_field_t,
                ) -> bool,
        ),
        field_count: (0 as ::core::ffi::c_int + 1 as ::core::ffi::c_int) as pb_size_t,
        required_field_count: (0 as ::core::ffi::c_int
            + (PB_HTYPE_REPEATED == PB_HTYPE_REQUIRED) as ::core::ffi::c_int)
            as pb_size_t,
        largest_tag: (0 as ::core::ffi::c_int * 0 as ::core::ffi::c_int
            + 1 as ::core::ffi::c_int) as pb_size_t,
    }
};
#[no_mangle]
pub static mut krpc_schema_List_field_info: [uint32_t; 3] = [0; 3];
#[no_mangle]
pub static mut krpc_schema_List_submsg_info: [*const pb_msgdesc_t; 1] = [
    ::core::ptr::null::<pb_msgdesc_t>(),
];
#[no_mangle]
pub static mut krpc_schema_Set_submsg_info: [*const pb_msgdesc_t; 1] = [
    ::core::ptr::null::<pb_msgdesc_t>(),
];
#[no_mangle]
pub static mut krpc_schema_Set_field_info: [uint32_t; 3] = [0; 3];
#[no_mangle]
pub static mut krpc_schema_Set_msg: pb_msgdesc_t = unsafe {
    pb_msgdesc_s {
        field_info: &raw const krpc_schema_Set_field_info as *const uint32_t,
        submsg_info: &raw const krpc_schema_Set_submsg_info
            as *const *const pb_msgdesc_t,
        default_value: ::core::ptr::null::<pb_byte_t>(),
        field_callback: Some(
            pb_default_field_callback
                as unsafe extern "C" fn(
                    *mut pb_istream_t,
                    *mut pb_ostream_t,
                    *const pb_field_t,
                ) -> bool,
        ),
        field_count: (0 as ::core::ffi::c_int + 1 as ::core::ffi::c_int) as pb_size_t,
        required_field_count: (0 as ::core::ffi::c_int
            + (PB_HTYPE_REPEATED == PB_HTYPE_REQUIRED) as ::core::ffi::c_int)
            as pb_size_t,
        largest_tag: (0 as ::core::ffi::c_int * 0 as ::core::ffi::c_int
            + 1 as ::core::ffi::c_int) as pb_size_t,
    }
};
#[no_mangle]
pub static mut krpc_schema_Dictionary_submsg_info: [*const pb_msgdesc_t; 2] = unsafe {
    [&raw const krpc_schema_DictionaryEntry_msg, ::core::ptr::null::<pb_msgdesc_t>()]
};
#[no_mangle]
pub static mut krpc_schema_Dictionary_field_info: [uint32_t; 3] = [0; 3];
#[no_mangle]
pub static mut krpc_schema_Dictionary_msg: pb_msgdesc_t = unsafe {
    pb_msgdesc_s {
        field_info: &raw const krpc_schema_Dictionary_field_info as *const uint32_t,
        submsg_info: &raw const krpc_schema_Dictionary_submsg_info
            as *const *const pb_msgdesc_t,
        default_value: ::core::ptr::null::<pb_byte_t>(),
        field_callback: Some(
            pb_default_field_callback
                as unsafe extern "C" fn(
                    *mut pb_istream_t,
                    *mut pb_ostream_t,
                    *const pb_field_t,
                ) -> bool,
        ),
        field_count: (0 as ::core::ffi::c_int + 1 as ::core::ffi::c_int) as pb_size_t,
        required_field_count: (0 as ::core::ffi::c_int
            + (PB_HTYPE_REPEATED == PB_HTYPE_REQUIRED) as ::core::ffi::c_int)
            as pb_size_t,
        largest_tag: (0 as ::core::ffi::c_int * 0 as ::core::ffi::c_int
            + 1 as ::core::ffi::c_int) as pb_size_t,
    }
};
#[no_mangle]
pub static mut krpc_schema_DictionaryEntry_submsg_info: [*const pb_msgdesc_t; 1] = [
    ::core::ptr::null::<pb_msgdesc_t>(),
];
#[no_mangle]
pub static mut krpc_schema_DictionaryEntry_msg: pb_msgdesc_t = unsafe {
    pb_msgdesc_s {
        field_info: &raw const krpc_schema_DictionaryEntry_field_info as *const uint32_t,
        submsg_info: &raw const krpc_schema_DictionaryEntry_submsg_info
            as *const *const pb_msgdesc_t,
        default_value: ::core::ptr::null::<pb_byte_t>(),
        field_callback: Some(
            pb_default_field_callback
                as unsafe extern "C" fn(
                    *mut pb_istream_t,
                    *mut pb_ostream_t,
                    *const pb_field_t,
                ) -> bool,
        ),
        field_count: (0 as ::core::ffi::c_int + 1 as ::core::ffi::c_int
            + 1 as ::core::ffi::c_int) as pb_size_t,
        required_field_count: (0 as ::core::ffi::c_int
            + (PB_HTYPE_SINGULAR == PB_HTYPE_REQUIRED) as ::core::ffi::c_int
            + (PB_HTYPE_SINGULAR == PB_HTYPE_REQUIRED) as ::core::ffi::c_int)
            as pb_size_t,
        largest_tag: (0 as ::core::ffi::c_int * 0 as ::core::ffi::c_int
            + 1 as ::core::ffi::c_int * 0 as ::core::ffi::c_int
            + 2 as ::core::ffi::c_int) as pb_size_t,
    }
};
#[no_mangle]
pub static mut krpc_schema_DictionaryEntry_field_info: [uint32_t; 5] = [0; 5];
#[no_mangle]
pub static mut krpc_schema_Stream_msg: pb_msgdesc_t = unsafe {
    pb_msgdesc_s {
        field_info: &raw const krpc_schema_Stream_field_info as *const uint32_t,
        submsg_info: &raw const krpc_schema_Stream_submsg_info
            as *const *const pb_msgdesc_t,
        default_value: ::core::ptr::null::<pb_byte_t>(),
        field_callback: None,
        field_count: (0 as ::core::ffi::c_int + 1 as ::core::ffi::c_int) as pb_size_t,
        required_field_count: (0 as ::core::ffi::c_int
            + (PB_HTYPE_SINGULAR == PB_HTYPE_REQUIRED) as ::core::ffi::c_int)
            as pb_size_t,
        largest_tag: (0 as ::core::ffi::c_int * 0 as ::core::ffi::c_int
            + 1 as ::core::ffi::c_int) as pb_size_t,
    }
};
#[no_mangle]
pub static mut krpc_schema_Stream_submsg_info: [*const pb_msgdesc_t; 1] = [
    ::core::ptr::null::<pb_msgdesc_t>(),
];
#[no_mangle]
pub static mut krpc_schema_Stream_field_info: [uint32_t; 2] = [0; 2];
#[no_mangle]
pub static mut krpc_schema_Event_msg: pb_msgdesc_t = unsafe {
    pb_msgdesc_s {
        field_info: &raw const krpc_schema_Event_field_info as *const uint32_t,
        submsg_info: &raw const krpc_schema_Event_submsg_info
            as *const *const pb_msgdesc_t,
        default_value: ::core::ptr::null::<pb_byte_t>(),
        field_callback: None,
        field_count: (0 as ::core::ffi::c_int + 1 as ::core::ffi::c_int) as pb_size_t,
        required_field_count: (0 as ::core::ffi::c_int
            + (PB_HTYPE_OPTIONAL == PB_HTYPE_REQUIRED) as ::core::ffi::c_int)
            as pb_size_t,
        largest_tag: (0 as ::core::ffi::c_int * 0 as ::core::ffi::c_int
            + 1 as ::core::ffi::c_int) as pb_size_t,
    }
};
#[no_mangle]
pub static mut krpc_schema_Event_submsg_info: [*const pb_msgdesc_t; 2] = unsafe {
    [&raw const krpc_schema_Stream_msg, ::core::ptr::null::<pb_msgdesc_t>()]
};
#[no_mangle]
pub static mut krpc_schema_Event_field_info: [uint32_t; 3] = [0; 3];
#[no_mangle]
pub static mut krpc_schema_Status_msg: pb_msgdesc_t = unsafe {
    pb_msgdesc_s {
        field_info: &raw const krpc_schema_Status_field_info as *const uint32_t,
        submsg_info: &raw const krpc_schema_Status_submsg_info
            as *const *const pb_msgdesc_t,
        default_value: ::core::ptr::null::<pb_byte_t>(),
        field_callback: None,
        field_count: (0 as ::core::ffi::c_int + 1 as ::core::ffi::c_int
            + 1 as ::core::ffi::c_int + 1 as ::core::ffi::c_int + 1 as ::core::ffi::c_int
            + 1 as ::core::ffi::c_int + 1 as ::core::ffi::c_int + 1 as ::core::ffi::c_int
            + 1 as ::core::ffi::c_int + 1 as ::core::ffi::c_int + 1 as ::core::ffi::c_int
            + 1 as ::core::ffi::c_int + 1 as ::core::ffi::c_int + 1 as ::core::ffi::c_int
            + 1 as ::core::ffi::c_int + 1 as ::core::ffi::c_int + 1 as ::core::ffi::c_int
            + 1 as ::core::ffi::c_int + 1 as ::core::ffi::c_int
            + 1 as ::core::ffi::c_int) as pb_size_t,
        required_field_count: (0 as ::core::ffi::c_int
            + (PB_HTYPE_SINGULAR == PB_HTYPE_REQUIRED) as ::core::ffi::c_int
            + (PB_HTYPE_SINGULAR == PB_HTYPE_REQUIRED) as ::core::ffi::c_int
            + (PB_HTYPE_SINGULAR == PB_HTYPE_REQUIRED) as ::core::ffi::c_int
            + (PB_HTYPE_SINGULAR == PB_HTYPE_REQUIRED) as ::core::ffi::c_int
            + (PB_HTYPE_SINGULAR == PB_HTYPE_REQUIRED) as ::core::ffi::c_int
            + (PB_HTYPE_SINGULAR == PB_HTYPE_REQUIRED) as ::core::ffi::c_int
            + (PB_HTYPE_SINGULAR == PB_HTYPE_REQUIRED) as ::core::ffi::c_int
            + (PB_HTYPE_SINGULAR == PB_HTYPE_REQUIRED) as ::core::ffi::c_int
            + (PB_HTYPE_SINGULAR == PB_HTYPE_REQUIRED) as ::core::ffi::c_int
            + (PB_HTYPE_SINGULAR == PB_HTYPE_REQUIRED) as ::core::ffi::c_int
            + (PB_HTYPE_SINGULAR == PB_HTYPE_REQUIRED) as ::core::ffi::c_int
            + (PB_HTYPE_SINGULAR == PB_HTYPE_REQUIRED) as ::core::ffi::c_int
            + (PB_HTYPE_SINGULAR == PB_HTYPE_REQUIRED) as ::core::ffi::c_int
            + (PB_HTYPE_SINGULAR == PB_HTYPE_REQUIRED) as ::core::ffi::c_int
            + (PB_HTYPE_SINGULAR == PB_HTYPE_REQUIRED) as ::core::ffi::c_int
            + (PB_HTYPE_SINGULAR == PB_HTYPE_REQUIRED) as ::core::ffi::c_int
            + (PB_HTYPE_SINGULAR == PB_HTYPE_REQUIRED) as ::core::ffi::c_int
            + (PB_HTYPE_SINGULAR == PB_HTYPE_REQUIRED) as ::core::ffi::c_int
            + (PB_HTYPE_SINGULAR == PB_HTYPE_REQUIRED) as ::core::ffi::c_int)
            as pb_size_t,
        largest_tag: (0 as ::core::ffi::c_int * 0 as ::core::ffi::c_int
            + 1 as ::core::ffi::c_int * 0 as ::core::ffi::c_int
            + 2 as ::core::ffi::c_int * 0 as ::core::ffi::c_int
            + 3 as ::core::ffi::c_int * 0 as ::core::ffi::c_int
            + 4 as ::core::ffi::c_int * 0 as ::core::ffi::c_int
            + 5 as ::core::ffi::c_int * 0 as ::core::ffi::c_int
            + 6 as ::core::ffi::c_int * 0 as ::core::ffi::c_int
            + 7 as ::core::ffi::c_int * 0 as ::core::ffi::c_int
            + 8 as ::core::ffi::c_int * 0 as ::core::ffi::c_int
            + 9 as ::core::ffi::c_int * 0 as ::core::ffi::c_int
            + 10 as ::core::ffi::c_int * 0 as ::core::ffi::c_int
            + 11 as ::core::ffi::c_int * 0 as ::core::ffi::c_int
            + 12 as ::core::ffi::c_int * 0 as ::core::ffi::c_int
            + 13 as ::core::ffi::c_int * 0 as ::core::ffi::c_int
            + 14 as ::core::ffi::c_int * 0 as ::core::ffi::c_int
            + 15 as ::core::ffi::c_int * 0 as ::core::ffi::c_int
            + 16 as ::core::ffi::c_int * 0 as ::core::ffi::c_int
            + 17 as ::core::ffi::c_int * 0 as ::core::ffi::c_int
            + 18 as ::core::ffi::c_int * 0 as ::core::ffi::c_int
            + 19 as ::core::ffi::c_int) as pb_size_t,
    }
};
#[no_mangle]
pub static mut krpc_schema_Status_submsg_info: [*const pb_msgdesc_t; 1] = [
    ::core::ptr::null::<pb_msgdesc_t>(),
];
#[no_mangle]
pub static mut krpc_schema_Status_field_info: [uint32_t; 21] = [0; 21];
#[no_mangle]
pub static mut krpc_schema_MultiplexedRequest_field_info: [uint32_t; 5] = [0; 5];
#[no_mangle]
pub static mut krpc_schema_MultiplexedRequest_msg: pb_msgdesc_t = unsafe {
    pb_msgdesc_s {
        field_info: &raw const krpc_schema_MultiplexedRequest_field_info
            as *const uint32_t,
        submsg_info: &raw const krpc_schema_MultiplexedRequest_submsg_info
            as *const *const pb_msgdesc_t,
        default_value: ::core::ptr::null::<pb_byte_t>(),
        field_callback: None,
        field_count: (0 as ::core::ffi::c_int + 1 as ::core::ffi::c_int
            + 1 as ::core::ffi::c_int) as pb_size_t,
        required_field_count: (0 as ::core::ffi::c_int
            + (PB_HTYPE_OPTIONAL == PB_HTYPE_REQUIRED) as ::core::ffi::c_int
            + (PB_HTYPE_OPTIONAL == PB_HTYPE_REQUIRED) as ::core::ffi::c_int)
            as pb_size_t,
        largest_tag: (0 as ::core::ffi::c_int * 0 as ::core::ffi::c_int
            + 1 as ::core::ffi::c_int * 0 as ::core::ffi::c_int
            + 2 as ::core::ffi::c_int) as pb_size_t,
    }
};
#[no_mangle]
pub static mut krpc_schema_MultiplexedRequest_submsg_info: [*const pb_msgdesc_t; 3] = unsafe {
    [
        &raw const krpc_schema_ConnectionRequest_msg,
        &raw const krpc_schema_Request_msg,
        ::core::ptr::null::<pb_msgdesc_t>(),
    ]
};
#[no_mangle]
pub static mut krpc_schema_MultiplexedResponse_msg: pb_msgdesc_t = unsafe {
    pb_msgdesc_s {
        field_info: &raw const krpc_schema_MultiplexedResponse_field_info
            as *const uint32_t,
        submsg_info: &raw const krpc_schema_MultiplexedResponse_submsg_info
            as *const *const pb_msgdesc_t,
        default_value: ::core::ptr::null::<pb_byte_t>(),
        field_callback: None,
        field_count: (0 as ::core::ffi::c_int + 1 as ::core::ffi::c_int
            + 1 as ::core::ffi::c_int) as pb_size_t,
        required_field_count: (0 as ::core::ffi::c_int
            + (PB_HTYPE_OPTIONAL == PB_HTYPE_REQUIRED) as ::core::ffi::c_int
            + (PB_HTYPE_OPTIONAL == PB_HTYPE_REQUIRED) as ::core::ffi::c_int)
            as pb_size_t,
        largest_tag: (0 as ::core::ffi::c_int * 0 as ::core::ffi::c_int
            + 1 as ::core::ffi::c_int * 0 as ::core::ffi::c_int
            + 2 as ::core::ffi::c_int) as pb_size_t,
    }
};
#[no_mangle]
pub static mut krpc_schema_MultiplexedResponse_submsg_info: [*const pb_msgdesc_t; 3] = unsafe {
    [
        &raw const krpc_schema_Response_msg,
        &raw const krpc_schema_StreamUpdate_msg,
        ::core::ptr::null::<pb_msgdesc_t>(),
    ]
};
#[no_mangle]
pub static mut krpc_schema_MultiplexedResponse_field_info: [uint32_t; 5] = [0; 5];
unsafe extern "C" fn run_static_initializers() {
    krpc_schema_ConnectionRequest_field_info = [
        (0 as ::core::ffi::c_int
            | (1 as ::core::ffi::c_int) << 2 as ::core::ffi::c_int
                & 0xff as ::core::ffi::c_int) as uint32_t
            | (0 as uint32_t | 0x10 as uint32_t | 0x2 as uint32_t)
                << 8 as ::core::ffi::c_int
            | (0 as ::core::ffi::c_ulong as uint32_t & 0xff as uint32_t)
                << 16 as ::core::ffi::c_int
            | (0 as ::core::ffi::c_int as uint32_t & 0xf as uint32_t)
                << 24 as ::core::ffi::c_int
            | (::core::mem::size_of::<krpc_schema_ConnectionRequest_Type>() as uint32_t
                & 0xf as uint32_t) << 28 as ::core::ffi::c_int,
        (1 as ::core::ffi::c_int
            | (2 as ::core::ffi::c_int) << 2 as ::core::ffi::c_int
                & 0xff as ::core::ffi::c_int) as uint32_t
            | (0x40 as uint32_t | 0x10 as uint32_t | 0x7 as uint32_t)
                << 8 as ::core::ffi::c_int
            | (1 as ::core::ffi::c_int as uint32_t & 0xfff as uint32_t)
                << 16 as ::core::ffi::c_int
            | (0 as ::core::ffi::c_int as uint32_t & 0xf as uint32_t)
                << 28 as ::core::ffi::c_int,
        8 as ::core::ffi::c_ulong as uint32_t & 0xffff as uint32_t
            | (::core::mem::size_of::<pb_callback_t>() as uint32_t & 0xfff as uint32_t)
                << 16 as ::core::ffi::c_int
            | (2 as ::core::ffi::c_int as uint32_t & 0x3c0 as uint32_t)
                << 22 as ::core::ffi::c_int,
        (1 as ::core::ffi::c_int
            | (3 as ::core::ffi::c_int) << 2 as ::core::ffi::c_int
                & 0xff as ::core::ffi::c_int) as uint32_t
            | (0x40 as uint32_t | 0x10 as uint32_t | 0x6 as uint32_t)
                << 8 as ::core::ffi::c_int
            | (1 as ::core::ffi::c_int as uint32_t & 0xfff as uint32_t)
                << 16 as ::core::ffi::c_int
            | (0 as ::core::ffi::c_int as uint32_t & 0xf as uint32_t)
                << 28 as ::core::ffi::c_int,
        24 as ::core::ffi::c_ulong as uint32_t & 0xffff as uint32_t
            | (::core::mem::size_of::<pb_callback_t>() as uint32_t & 0xfff as uint32_t)
                << 16 as ::core::ffi::c_int
            | (3 as ::core::ffi::c_int as uint32_t & 0x3c0 as uint32_t)
                << 22 as ::core::ffi::c_int,
        0 as ::core::ffi::c_int as uint32_t,
    ];
    krpc_schema_ConnectionResponse_field_info = [
        (0 as ::core::ffi::c_int
            | (1 as ::core::ffi::c_int) << 2 as ::core::ffi::c_int
                & 0xff as ::core::ffi::c_int) as uint32_t
            | (0 as uint32_t | 0x10 as uint32_t | 0x2 as uint32_t)
                << 8 as ::core::ffi::c_int
            | (0 as ::core::ffi::c_ulong as uint32_t & 0xff as uint32_t)
                << 16 as ::core::ffi::c_int
            | (0 as ::core::ffi::c_int as uint32_t & 0xf as uint32_t)
                << 24 as ::core::ffi::c_int
            | (::core::mem::size_of::<krpc_schema_ConnectionResponse_Status>()
                as uint32_t & 0xf as uint32_t) << 28 as ::core::ffi::c_int,
        (1 as ::core::ffi::c_int
            | (2 as ::core::ffi::c_int) << 2 as ::core::ffi::c_int
                & 0xff as ::core::ffi::c_int) as uint32_t
            | (0x40 as uint32_t | 0x10 as uint32_t | 0x7 as uint32_t)
                << 8 as ::core::ffi::c_int
            | (1 as ::core::ffi::c_int as uint32_t & 0xfff as uint32_t)
                << 16 as ::core::ffi::c_int
            | (0 as ::core::ffi::c_int as uint32_t & 0xf as uint32_t)
                << 28 as ::core::ffi::c_int,
        8 as ::core::ffi::c_ulong as uint32_t & 0xffff as uint32_t
            | (::core::mem::size_of::<pb_callback_t>() as uint32_t & 0xfff as uint32_t)
                << 16 as ::core::ffi::c_int
            | (2 as ::core::ffi::c_int as uint32_t & 0x3c0 as uint32_t)
                << 22 as ::core::ffi::c_int,
        (1 as ::core::ffi::c_int
            | (3 as ::core::ffi::c_int) << 2 as ::core::ffi::c_int
                & 0xff as ::core::ffi::c_int) as uint32_t
            | (0x40 as uint32_t | 0x10 as uint32_t | 0x6 as uint32_t)
                << 8 as ::core::ffi::c_int
            | (1 as ::core::ffi::c_int as uint32_t & 0xfff as uint32_t)
                << 16 as ::core::ffi::c_int
            | (0 as ::core::ffi::c_int as uint32_t & 0xf as uint32_t)
                << 28 as ::core::ffi::c_int,
        24 as ::core::ffi::c_ulong as uint32_t & 0xffff as uint32_t
            | (::core::mem::size_of::<pb_callback_t>() as uint32_t & 0xfff as uint32_t)
                << 16 as ::core::ffi::c_int
            | (3 as ::core::ffi::c_int as uint32_t & 0x3c0 as uint32_t)
                << 22 as ::core::ffi::c_int,
        0 as ::core::ffi::c_int as uint32_t,
    ];
    krpc_schema_Argument_field_info = [
        (0 as ::core::ffi::c_int
            | (1 as ::core::ffi::c_int) << 2 as ::core::ffi::c_int
                & 0xff as ::core::ffi::c_int) as uint32_t
            | (0 as uint32_t | 0x10 as uint32_t | 0x2 as uint32_t)
                << 8 as ::core::ffi::c_int
            | (0 as ::core::ffi::c_ulong as uint32_t & 0xff as uint32_t)
                << 16 as ::core::ffi::c_int
            | (0 as ::core::ffi::c_int as uint32_t & 0xf as uint32_t)
                << 24 as ::core::ffi::c_int
            | (::core::mem::size_of::<uint32_t>() as uint32_t & 0xf as uint32_t)
                << 28 as ::core::ffi::c_int,
        (1 as ::core::ffi::c_int
            | (2 as ::core::ffi::c_int) << 2 as ::core::ffi::c_int
                & 0xff as ::core::ffi::c_int) as uint32_t
            | (0x40 as uint32_t | 0x10 as uint32_t | 0x6 as uint32_t)
                << 8 as ::core::ffi::c_int
            | (1 as ::core::ffi::c_int as uint32_t & 0xfff as uint32_t)
                << 16 as ::core::ffi::c_int
            | (0 as ::core::ffi::c_int as uint32_t & 0xf as uint32_t)
                << 28 as ::core::ffi::c_int,
        8 as ::core::ffi::c_ulong as uint32_t & 0xffff as uint32_t
            | (::core::mem::size_of::<pb_callback_t>() as uint32_t & 0xfff as uint32_t)
                << 16 as ::core::ffi::c_int
            | (2 as ::core::ffi::c_int as uint32_t & 0x3c0 as uint32_t)
                << 22 as ::core::ffi::c_int,
        0 as ::core::ffi::c_int as uint32_t,
    ];
    krpc_schema_ProcedureCall_field_info = [
        (1 as ::core::ffi::c_int
            | (3 as ::core::ffi::c_int) << 2 as ::core::ffi::c_int
                & 0xff as ::core::ffi::c_int) as uint32_t
            | (0x40 as uint32_t | 0x20 as uint32_t | 0x8 as uint32_t)
                << 8 as ::core::ffi::c_int
            | (1 as ::core::ffi::c_int as uint32_t & 0xfff as uint32_t)
                << 16 as ::core::ffi::c_int
            | (0 as ::core::ffi::c_int as uint32_t & 0xf as uint32_t)
                << 28 as ::core::ffi::c_int,
        0 as ::core::ffi::c_ulong as uint32_t & 0xffff as uint32_t
            | (::core::mem::size_of::<pb_callback_t>() as uint32_t & 0xfff as uint32_t)
                << 16 as ::core::ffi::c_int
            | (3 as ::core::ffi::c_int as uint32_t & 0x3c0 as uint32_t)
                << 22 as ::core::ffi::c_int,
        (0 as ::core::ffi::c_int
            | (4 as ::core::ffi::c_int) << 2 as ::core::ffi::c_int
                & 0xff as ::core::ffi::c_int) as uint32_t
            | (0 as uint32_t | 0x10 as uint32_t | 0x2 as uint32_t)
                << 8 as ::core::ffi::c_int
            | (16 as ::core::ffi::c_ulong as uint32_t & 0xff as uint32_t)
                << 16 as ::core::ffi::c_int
            | (0 as ::core::ffi::c_int as uint32_t & 0xf as uint32_t)
                << 24 as ::core::ffi::c_int
            | (::core::mem::size_of::<uint32_t>() as uint32_t & 0xf as uint32_t)
                << 28 as ::core::ffi::c_int,
        (0 as ::core::ffi::c_int
            | (5 as ::core::ffi::c_int) << 2 as ::core::ffi::c_int
                & 0xff as ::core::ffi::c_int) as uint32_t
            | (0 as uint32_t | 0x10 as uint32_t | 0x2 as uint32_t)
                << 8 as ::core::ffi::c_int
            | (20 as ::core::ffi::c_ulong as uint32_t & 0xff as uint32_t)
                << 16 as ::core::ffi::c_int
            | (0 as ::core::ffi::c_int as uint32_t & 0xf as uint32_t)
                << 24 as ::core::ffi::c_int
            | (::core::mem::size_of::<uint32_t>() as uint32_t & 0xf as uint32_t)
                << 28 as ::core::ffi::c_int,
        0 as ::core::ffi::c_int as uint32_t,
    ];
    krpc_schema_Request_field_info = [
        (1 as ::core::ffi::c_int
            | (1 as ::core::ffi::c_int) << 2 as ::core::ffi::c_int
                & 0xff as ::core::ffi::c_int) as uint32_t
            | (0 as uint32_t | 0x20 as uint32_t | 0x8 as uint32_t)
                << 8 as ::core::ffi::c_int
            | ((::core::mem::size_of::<[krpc_schema_ProcedureCall; 32]>() as usize)
                .wrapping_div(
                    ::core::mem::size_of::<krpc_schema_ProcedureCall>() as usize,
                ) as uint32_t & 0xfff as uint32_t) << 16 as ::core::ffi::c_int
            | ((8 as ::core::ffi::c_ulong as ::core::ffi::c_int
                - 0 as ::core::ffi::c_ulong as ::core::ffi::c_int) as uint32_t
                & 0xf as uint32_t) << 28 as ::core::ffi::c_int,
        8 as ::core::ffi::c_ulong as uint32_t & 0xffff as uint32_t
            | (::core::mem::size_of::<krpc_schema_ProcedureCall>() as uint32_t
                & 0xfff as uint32_t) << 16 as ::core::ffi::c_int
            | (1 as ::core::ffi::c_int as uint32_t & 0x3c0 as uint32_t)
                << 22 as ::core::ffi::c_int,
        0 as ::core::ffi::c_int as uint32_t,
    ];
    krpc_schema_Error_field_info = [
        (1 as ::core::ffi::c_int
            | (1 as ::core::ffi::c_int) << 2 as ::core::ffi::c_int
                & 0xff as ::core::ffi::c_int) as uint32_t
            | (0x40 as uint32_t | 0x10 as uint32_t | 0x7 as uint32_t)
                << 8 as ::core::ffi::c_int
            | (1 as ::core::ffi::c_int as uint32_t & 0xfff as uint32_t)
                << 16 as ::core::ffi::c_int
            | (0 as ::core::ffi::c_int as uint32_t & 0xf as uint32_t)
                << 28 as ::core::ffi::c_int,
        0 as ::core::ffi::c_ulong as uint32_t & 0xffff as uint32_t
            | (::core::mem::size_of::<pb_callback_t>() as uint32_t & 0xfff as uint32_t)
                << 16 as ::core::ffi::c_int
            | (1 as ::core::ffi::c_int as uint32_t & 0x3c0 as uint32_t)
                << 22 as ::core::ffi::c_int,
        (1 as ::core::ffi::c_int
            | (2 as ::core::ffi::c_int) << 2 as ::core::ffi::c_int
                & 0xff as ::core::ffi::c_int) as uint32_t
            | (0x40 as uint32_t | 0x10 as uint32_t | 0x7 as uint32_t)
                << 8 as ::core::ffi::c_int
            | (1 as ::core::ffi::c_int as uint32_t & 0xfff as uint32_t)
                << 16 as ::core::ffi::c_int
            | (0 as ::core::ffi::c_int as uint32_t & 0xf as uint32_t)
                << 28 as ::core::ffi::c_int,
        16 as ::core::ffi::c_ulong as uint32_t & 0xffff as uint32_t
            | (::core::mem::size_of::<pb_callback_t>() as uint32_t & 0xfff as uint32_t)
                << 16 as ::core::ffi::c_int
            | (2 as ::core::ffi::c_int as uint32_t & 0x3c0 as uint32_t)
                << 22 as ::core::ffi::c_int,
        (1 as ::core::ffi::c_int
            | (3 as ::core::ffi::c_int) << 2 as ::core::ffi::c_int
                & 0xff as ::core::ffi::c_int) as uint32_t
            | (0x40 as uint32_t | 0x10 as uint32_t | 0x7 as uint32_t)
                << 8 as ::core::ffi::c_int
            | (1 as ::core::ffi::c_int as uint32_t & 0xfff as uint32_t)
                << 16 as ::core::ffi::c_int
            | (0 as ::core::ffi::c_int as uint32_t & 0xf as uint32_t)
                << 28 as ::core::ffi::c_int,
        32 as ::core::ffi::c_ulong as uint32_t & 0xffff as uint32_t
            | (::core::mem::size_of::<pb_callback_t>() as uint32_t & 0xfff as uint32_t)
                << 16 as ::core::ffi::c_int
            | (3 as ::core::ffi::c_int as uint32_t & 0x3c0 as uint32_t)
                << 22 as ::core::ffi::c_int,
        (1 as ::core::ffi::c_int
            | (4 as ::core::ffi::c_int) << 2 as ::core::ffi::c_int
                & 0xff as ::core::ffi::c_int) as uint32_t
            | (0x40 as uint32_t | 0x10 as uint32_t | 0x7 as uint32_t)
                << 8 as ::core::ffi::c_int
            | (1 as ::core::ffi::c_int as uint32_t & 0xfff as uint32_t)
                << 16 as ::core::ffi::c_int
            | (0 as ::core::ffi::c_int as uint32_t & 0xf as uint32_t)
                << 28 as ::core::ffi::c_int,
        48 as ::core::ffi::c_ulong as uint32_t & 0xffff as uint32_t
            | (::core::mem::size_of::<pb_callback_t>() as uint32_t & 0xfff as uint32_t)
                << 16 as ::core::ffi::c_int
            | (4 as ::core::ffi::c_int as uint32_t & 0x3c0 as uint32_t)
                << 22 as ::core::ffi::c_int,
        0 as ::core::ffi::c_int as uint32_t,
    ];
    krpc_schema_ProcedureResult_field_info = [
        (1 as ::core::ffi::c_int
            | (1 as ::core::ffi::c_int) << 2 as ::core::ffi::c_int
                & 0xff as ::core::ffi::c_int) as uint32_t
            | (0x40 as uint32_t | 0x10 as uint32_t | 0x8 as uint32_t)
                << 8 as ::core::ffi::c_int
            | (1 as ::core::ffi::c_int as uint32_t & 0xfff as uint32_t)
                << 16 as ::core::ffi::c_int
            | (0 as ::core::ffi::c_int as uint32_t & 0xf as uint32_t)
                << 28 as ::core::ffi::c_int,
        0 as ::core::ffi::c_ulong as uint32_t & 0xffff as uint32_t
            | (::core::mem::size_of::<pb_callback_t>() as uint32_t & 0xfff as uint32_t)
                << 16 as ::core::ffi::c_int
            | (1 as ::core::ffi::c_int as uint32_t & 0x3c0 as uint32_t)
                << 22 as ::core::ffi::c_int,
        (1 as ::core::ffi::c_int
            | (2 as ::core::ffi::c_int) << 2 as ::core::ffi::c_int
                & 0xff as ::core::ffi::c_int) as uint32_t
            | (0x40 as uint32_t | 0x10 as uint32_t | 0x6 as uint32_t)
                << 8 as ::core::ffi::c_int
            | (1 as ::core::ffi::c_int as uint32_t & 0xfff as uint32_t)
                << 16 as ::core::ffi::c_int
            | (0 as ::core::ffi::c_int as uint32_t & 0xf as uint32_t)
                << 28 as ::core::ffi::c_int,
        16 as ::core::ffi::c_ulong as uint32_t & 0xffff as uint32_t
            | (::core::mem::size_of::<pb_callback_t>() as uint32_t & 0xfff as uint32_t)
                << 16 as ::core::ffi::c_int
            | (2 as ::core::ffi::c_int as uint32_t & 0x3c0 as uint32_t)
                << 22 as ::core::ffi::c_int,
        0 as ::core::ffi::c_int as uint32_t,
    ];
    krpc_schema_Response_field_info = [
        (1 as ::core::ffi::c_int
            | (1 as ::core::ffi::c_int) << 2 as ::core::ffi::c_int
                & 0xff as ::core::ffi::c_int) as uint32_t
            | (0x40 as uint32_t | 0x10 as uint32_t | 0x8 as uint32_t)
                << 8 as ::core::ffi::c_int
            | (1 as ::core::ffi::c_int as uint32_t & 0xfff as uint32_t)
                << 16 as ::core::ffi::c_int
            | (0 as ::core::ffi::c_int as uint32_t & 0xf as uint32_t)
                << 28 as ::core::ffi::c_int,
        0 as ::core::ffi::c_ulong as uint32_t & 0xffff as uint32_t
            | (::core::mem::size_of::<pb_callback_t>() as uint32_t & 0xfff as uint32_t)
                << 16 as ::core::ffi::c_int
            | (1 as ::core::ffi::c_int as uint32_t & 0x3c0 as uint32_t)
                << 22 as ::core::ffi::c_int,
        (1 as ::core::ffi::c_int
            | (2 as ::core::ffi::c_int) << 2 as ::core::ffi::c_int
                & 0xff as ::core::ffi::c_int) as uint32_t
            | (0 as uint32_t | 0x20 as uint32_t | 0x8 as uint32_t)
                << 8 as ::core::ffi::c_int
            | ((::core::mem::size_of::<[krpc_schema_ProcedureResult; 32]>() as usize)
                .wrapping_div(
                    ::core::mem::size_of::<krpc_schema_ProcedureResult>() as usize,
                ) as uint32_t & 0xfff as uint32_t) << 16 as ::core::ffi::c_int
            | ((24 as ::core::ffi::c_ulong as ::core::ffi::c_int
                - 16 as ::core::ffi::c_ulong as ::core::ffi::c_int) as uint32_t
                & 0xf as uint32_t) << 28 as ::core::ffi::c_int,
        24 as ::core::ffi::c_ulong as uint32_t & 0xffff as uint32_t
            | (::core::mem::size_of::<krpc_schema_ProcedureResult>() as uint32_t
                & 0xfff as uint32_t) << 16 as ::core::ffi::c_int
            | (2 as ::core::ffi::c_int as uint32_t & 0x3c0 as uint32_t)
                << 22 as ::core::ffi::c_int,
        0 as ::core::ffi::c_int as uint32_t,
    ];
    krpc_schema_StreamResult_field_info = [
        (0 as ::core::ffi::c_int
            | (1 as ::core::ffi::c_int) << 2 as ::core::ffi::c_int
                & 0xff as ::core::ffi::c_int) as uint32_t
            | (0 as uint32_t | 0x10 as uint32_t | 0x2 as uint32_t)
                << 8 as ::core::ffi::c_int
            | (0 as ::core::ffi::c_ulong as uint32_t & 0xff as uint32_t)
                << 16 as ::core::ffi::c_int
            | (0 as ::core::ffi::c_int as uint32_t & 0xf as uint32_t)
                << 24 as ::core::ffi::c_int
            | (::core::mem::size_of::<uint64_t>() as uint32_t & 0xf as uint32_t)
                << 28 as ::core::ffi::c_int,
        (1 as ::core::ffi::c_int
            | (2 as ::core::ffi::c_int) << 2 as ::core::ffi::c_int
                & 0xff as ::core::ffi::c_int) as uint32_t
            | (0 as uint32_t | 0x10 as uint32_t | 0x8 as uint32_t)
                << 8 as ::core::ffi::c_int
            | (1 as ::core::ffi::c_int as uint32_t & 0xfff as uint32_t)
                << 16 as ::core::ffi::c_int
            | ((16 as ::core::ffi::c_ulong as ::core::ffi::c_int
                - 8 as ::core::ffi::c_ulong as ::core::ffi::c_int) as uint32_t
                & 0xf as uint32_t) << 28 as ::core::ffi::c_int,
        16 as ::core::ffi::c_ulong as uint32_t & 0xffff as uint32_t
            | (::core::mem::size_of::<krpc_schema_ProcedureResult>() as uint32_t
                & 0xfff as uint32_t) << 16 as ::core::ffi::c_int
            | (2 as ::core::ffi::c_int as uint32_t & 0x3c0 as uint32_t)
                << 22 as ::core::ffi::c_int,
        0 as ::core::ffi::c_int as uint32_t,
    ];
    krpc_schema_StreamUpdate_field_info = [
        (1 as ::core::ffi::c_int
            | (1 as ::core::ffi::c_int) << 2 as ::core::ffi::c_int
                & 0xff as ::core::ffi::c_int) as uint32_t
            | (0x40 as uint32_t | 0x20 as uint32_t | 0x8 as uint32_t)
                << 8 as ::core::ffi::c_int
            | (1 as ::core::ffi::c_int as uint32_t & 0xfff as uint32_t)
                << 16 as ::core::ffi::c_int
            | (0 as ::core::ffi::c_int as uint32_t & 0xf as uint32_t)
                << 28 as ::core::ffi::c_int,
        0 as ::core::ffi::c_ulong as uint32_t & 0xffff as uint32_t
            | (::core::mem::size_of::<pb_callback_t>() as uint32_t & 0xfff as uint32_t)
                << 16 as ::core::ffi::c_int
            | (1 as ::core::ffi::c_int as uint32_t & 0x3c0 as uint32_t)
                << 22 as ::core::ffi::c_int,
        0 as ::core::ffi::c_int as uint32_t,
    ];
    krpc_schema_Exception_field_info = [
        (1 as ::core::ffi::c_int
            | (1 as ::core::ffi::c_int) << 2 as ::core::ffi::c_int
                & 0xff as ::core::ffi::c_int) as uint32_t
            | (0x40 as uint32_t | 0x10 as uint32_t | 0x7 as uint32_t)
                << 8 as ::core::ffi::c_int
            | (1 as ::core::ffi::c_int as uint32_t & 0xfff as uint32_t)
                << 16 as ::core::ffi::c_int
            | (0 as ::core::ffi::c_int as uint32_t & 0xf as uint32_t)
                << 28 as ::core::ffi::c_int,
        0 as ::core::ffi::c_ulong as uint32_t & 0xffff as uint32_t
            | (::core::mem::size_of::<pb_callback_t>() as uint32_t & 0xfff as uint32_t)
                << 16 as ::core::ffi::c_int
            | (1 as ::core::ffi::c_int as uint32_t & 0x3c0 as uint32_t)
                << 22 as ::core::ffi::c_int,
        (1 as ::core::ffi::c_int
            | (2 as ::core::ffi::c_int) << 2 as ::core::ffi::c_int
                & 0xff as ::core::ffi::c_int) as uint32_t
            | (0x40 as uint32_t | 0x10 as uint32_t | 0x7 as uint32_t)
                << 8 as ::core::ffi::c_int
            | (1 as ::core::ffi::c_int as uint32_t & 0xfff as uint32_t)
                << 16 as ::core::ffi::c_int
            | (0 as ::core::ffi::c_int as uint32_t & 0xf as uint32_t)
                << 28 as ::core::ffi::c_int,
        16 as ::core::ffi::c_ulong as uint32_t & 0xffff as uint32_t
            | (::core::mem::size_of::<pb_callback_t>() as uint32_t & 0xfff as uint32_t)
                << 16 as ::core::ffi::c_int
            | (2 as ::core::ffi::c_int as uint32_t & 0x3c0 as uint32_t)
                << 22 as ::core::ffi::c_int,
        (0 as ::core::ffi::c_int
            | (3 as ::core::ffi::c_int) << 2 as ::core::ffi::c_int
                & 0xff as ::core::ffi::c_int) as uint32_t
            | (0 as uint32_t | 0x10 as uint32_t | 0 as uint32_t)
                << 8 as ::core::ffi::c_int
            | (32 as ::core::ffi::c_ulong as uint32_t & 0xff as uint32_t)
                << 16 as ::core::ffi::c_int
            | (0 as ::core::ffi::c_int as uint32_t & 0xf as uint32_t)
                << 24 as ::core::ffi::c_int
            | (::core::mem::size_of::<bool>() as uint32_t & 0xf as uint32_t)
                << 28 as ::core::ffi::c_int,
        (1 as ::core::ffi::c_int
            | (4 as ::core::ffi::c_int) << 2 as ::core::ffi::c_int
                & 0xff as ::core::ffi::c_int) as uint32_t
            | (0x40 as uint32_t | 0x10 as uint32_t | 0x7 as uint32_t)
                << 8 as ::core::ffi::c_int
            | (1 as ::core::ffi::c_int as uint32_t & 0xfff as uint32_t)
                << 16 as ::core::ffi::c_int
            | (0 as ::core::ffi::c_int as uint32_t & 0xf as uint32_t)
                << 28 as ::core::ffi::c_int,
        40 as ::core::ffi::c_ulong as uint32_t & 0xffff as uint32_t
            | (::core::mem::size_of::<pb_callback_t>() as uint32_t & 0xfff as uint32_t)
                << 16 as ::core::ffi::c_int
            | (4 as ::core::ffi::c_int as uint32_t & 0x3c0 as uint32_t)
                << 22 as ::core::ffi::c_int,
        0 as ::core::ffi::c_int as uint32_t,
    ];
    krpc_schema_EnumerationValue_field_info = [
        (1 as ::core::ffi::c_int
            | (1 as ::core::ffi::c_int) << 2 as ::core::ffi::c_int
                & 0xff as ::core::ffi::c_int) as uint32_t
            | (0x40 as uint32_t | 0x10 as uint32_t | 0x7 as uint32_t)
                << 8 as ::core::ffi::c_int
            | (1 as ::core::ffi::c_int as uint32_t & 0xfff as uint32_t)
                << 16 as ::core::ffi::c_int
            | (0 as ::core::ffi::c_int as uint32_t & 0xf as uint32_t)
                << 28 as ::core::ffi::c_int,
        0 as ::core::ffi::c_ulong as uint32_t & 0xffff as uint32_t
            | (::core::mem::size_of::<pb_callback_t>() as uint32_t & 0xfff as uint32_t)
                << 16 as ::core::ffi::c_int
            | (1 as ::core::ffi::c_int as uint32_t & 0x3c0 as uint32_t)
                << 22 as ::core::ffi::c_int,
        (0 as ::core::ffi::c_int
            | (2 as ::core::ffi::c_int) << 2 as ::core::ffi::c_int
                & 0xff as ::core::ffi::c_int) as uint32_t
            | (0 as uint32_t | 0x10 as uint32_t | 0x1 as uint32_t)
                << 8 as ::core::ffi::c_int
            | (16 as ::core::ffi::c_ulong as uint32_t & 0xff as uint32_t)
                << 16 as ::core::ffi::c_int
            | (0 as ::core::ffi::c_int as uint32_t & 0xf as uint32_t)
                << 24 as ::core::ffi::c_int
            | (::core::mem::size_of::<int32_t>() as uint32_t & 0xf as uint32_t)
                << 28 as ::core::ffi::c_int,
        (1 as ::core::ffi::c_int
            | (3 as ::core::ffi::c_int) << 2 as ::core::ffi::c_int
                & 0xff as ::core::ffi::c_int) as uint32_t
            | (0x40 as uint32_t | 0x10 as uint32_t | 0x7 as uint32_t)
                << 8 as ::core::ffi::c_int
            | (1 as ::core::ffi::c_int as uint32_t & 0xfff as uint32_t)
                << 16 as ::core::ffi::c_int
            | (0 as ::core::ffi::c_int as uint32_t & 0xf as uint32_t)
                << 28 as ::core::ffi::c_int,
        24 as ::core::ffi::c_ulong as uint32_t & 0xffff as uint32_t
            | (::core::mem::size_of::<pb_callback_t>() as uint32_t & 0xfff as uint32_t)
                << 16 as ::core::ffi::c_int
            | (3 as ::core::ffi::c_int as uint32_t & 0x3c0 as uint32_t)
                << 22 as ::core::ffi::c_int,
        (0 as ::core::ffi::c_int
            | (4 as ::core::ffi::c_int) << 2 as ::core::ffi::c_int
                & 0xff as ::core::ffi::c_int) as uint32_t
            | (0 as uint32_t | 0x10 as uint32_t | 0 as uint32_t)
                << 8 as ::core::ffi::c_int
            | (40 as ::core::ffi::c_ulong as uint32_t & 0xff as uint32_t)
                << 16 as ::core::ffi::c_int
            | (0 as ::core::ffi::c_int as uint32_t & 0xf as uint32_t)
                << 24 as ::core::ffi::c_int
            | (::core::mem::size_of::<bool>() as uint32_t & 0xf as uint32_t)
                << 28 as ::core::ffi::c_int,
        (1 as ::core::ffi::c_int
            | (5 as ::core::ffi::c_int) << 2 as ::core::ffi::c_int
                & 0xff as ::core::ffi::c_int) as uint32_t
            | (0x40 as uint32_t | 0x10 as uint32_t | 0x7 as uint32_t)
                << 8 as ::core::ffi::c_int
            | (1 as ::core::ffi::c_int as uint32_t & 0xfff as uint32_t)
                << 16 as ::core::ffi::c_int
            | (0 as ::core::ffi::c_int as uint32_t & 0xf as uint32_t)
                << 28 as ::core::ffi::c_int,
        48 as ::core::ffi::c_ulong as uint32_t & 0xffff as uint32_t
            | (::core::mem::size_of::<pb_callback_t>() as uint32_t & 0xfff as uint32_t)
                << 16 as ::core::ffi::c_int
            | (5 as ::core::ffi::c_int as uint32_t & 0x3c0 as uint32_t)
                << 22 as ::core::ffi::c_int,
        0 as ::core::ffi::c_int as uint32_t,
    ];
    krpc_schema_Enumeration_field_info = [
        (1 as ::core::ffi::c_int
            | (1 as ::core::ffi::c_int) << 2 as ::core::ffi::c_int
                & 0xff as ::core::ffi::c_int) as uint32_t
            | (0x40 as uint32_t | 0x10 as uint32_t | 0x7 as uint32_t)
                << 8 as ::core::ffi::c_int
            | (1 as ::core::ffi::c_int as uint32_t & 0xfff as uint32_t)
                << 16 as ::core::ffi::c_int
            | (0 as ::core::ffi::c_int as uint32_t & 0xf as uint32_t)
                << 28 as ::core::ffi::c_int,
        0 as ::core::ffi::c_ulong as uint32_t & 0xffff as uint32_t
            | (::core::mem::size_of::<pb_callback_t>() as uint32_t & 0xfff as uint32_t)
                << 16 as ::core::ffi::c_int
            | (1 as ::core::ffi::c_int as uint32_t & 0x3c0 as uint32_t)
                << 22 as ::core::ffi::c_int,
        (1 as ::core::ffi::c_int
            | (2 as ::core::ffi::c_int) << 2 as ::core::ffi::c_int
                & 0xff as ::core::ffi::c_int) as uint32_t
            | (0x40 as uint32_t | 0x20 as uint32_t | 0x8 as uint32_t)
                << 8 as ::core::ffi::c_int
            | (1 as ::core::ffi::c_int as uint32_t & 0xfff as uint32_t)
                << 16 as ::core::ffi::c_int
            | (0 as ::core::ffi::c_int as uint32_t & 0xf as uint32_t)
                << 28 as ::core::ffi::c_int,
        16 as ::core::ffi::c_ulong as uint32_t & 0xffff as uint32_t
            | (::core::mem::size_of::<pb_callback_t>() as uint32_t & 0xfff as uint32_t)
                << 16 as ::core::ffi::c_int
            | (2 as ::core::ffi::c_int as uint32_t & 0x3c0 as uint32_t)
                << 22 as ::core::ffi::c_int,
        (1 as ::core::ffi::c_int
            | (3 as ::core::ffi::c_int) << 2 as ::core::ffi::c_int
                & 0xff as ::core::ffi::c_int) as uint32_t
            | (0x40 as uint32_t | 0x10 as uint32_t | 0x7 as uint32_t)
                << 8 as ::core::ffi::c_int
            | (1 as ::core::ffi::c_int as uint32_t & 0xfff as uint32_t)
                << 16 as ::core::ffi::c_int
            | (0 as ::core::ffi::c_int as uint32_t & 0xf as uint32_t)
                << 28 as ::core::ffi::c_int,
        32 as ::core::ffi::c_ulong as uint32_t & 0xffff as uint32_t
            | (::core::mem::size_of::<pb_callback_t>() as uint32_t & 0xfff as uint32_t)
                << 16 as ::core::ffi::c_int
            | (3 as ::core::ffi::c_int as uint32_t & 0x3c0 as uint32_t)
                << 22 as ::core::ffi::c_int,
        (0 as ::core::ffi::c_int
            | (4 as ::core::ffi::c_int) << 2 as ::core::ffi::c_int
                & 0xff as ::core::ffi::c_int) as uint32_t
            | (0 as uint32_t | 0x10 as uint32_t | 0 as uint32_t)
                << 8 as ::core::ffi::c_int
            | (48 as ::core::ffi::c_ulong as uint32_t & 0xff as uint32_t)
                << 16 as ::core::ffi::c_int
            | (0 as ::core::ffi::c_int as uint32_t & 0xf as uint32_t)
                << 24 as ::core::ffi::c_int
            | (::core::mem::size_of::<bool>() as uint32_t & 0xf as uint32_t)
                << 28 as ::core::ffi::c_int,
        (1 as ::core::ffi::c_int
            | (5 as ::core::ffi::c_int) << 2 as ::core::ffi::c_int
                & 0xff as ::core::ffi::c_int) as uint32_t
            | (0x40 as uint32_t | 0x10 as uint32_t | 0x7 as uint32_t)
                << 8 as ::core::ffi::c_int
            | (1 as ::core::ffi::c_int as uint32_t & 0xfff as uint32_t)
                << 16 as ::core::ffi::c_int
            | (0 as ::core::ffi::c_int as uint32_t & 0xf as uint32_t)
                << 28 as ::core::ffi::c_int,
        56 as ::core::ffi::c_ulong as uint32_t & 0xffff as uint32_t
            | (::core::mem::size_of::<pb_callback_t>() as uint32_t & 0xfff as uint32_t)
                << 16 as ::core::ffi::c_int
            | (5 as ::core::ffi::c_int as uint32_t & 0x3c0 as uint32_t)
                << 22 as ::core::ffi::c_int,
        0 as ::core::ffi::c_int as uint32_t,
    ];
    krpc_schema_Class_field_info = [
        (1 as ::core::ffi::c_int
            | (1 as ::core::ffi::c_int) << 2 as ::core::ffi::c_int
                & 0xff as ::core::ffi::c_int) as uint32_t
            | (0x40 as uint32_t | 0x10 as uint32_t | 0x7 as uint32_t)
                << 8 as ::core::ffi::c_int
            | (1 as ::core::ffi::c_int as uint32_t & 0xfff as uint32_t)
                << 16 as ::core::ffi::c_int
            | (0 as ::core::ffi::c_int as uint32_t & 0xf as uint32_t)
                << 28 as ::core::ffi::c_int,
        0 as ::core::ffi::c_ulong as uint32_t & 0xffff as uint32_t
            | (::core::mem::size_of::<pb_callback_t>() as uint32_t & 0xfff as uint32_t)
                << 16 as ::core::ffi::c_int
            | (1 as ::core::ffi::c_int as uint32_t & 0x3c0 as uint32_t)
                << 22 as ::core::ffi::c_int,
        (1 as ::core::ffi::c_int
            | (2 as ::core::ffi::c_int) << 2 as ::core::ffi::c_int
                & 0xff as ::core::ffi::c_int) as uint32_t
            | (0x40 as uint32_t | 0x10 as uint32_t | 0x7 as uint32_t)
                << 8 as ::core::ffi::c_int
            | (1 as ::core::ffi::c_int as uint32_t & 0xfff as uint32_t)
                << 16 as ::core::ffi::c_int
            | (0 as ::core::ffi::c_int as uint32_t & 0xf as uint32_t)
                << 28 as ::core::ffi::c_int,
        16 as ::core::ffi::c_ulong as uint32_t & 0xffff as uint32_t
            | (::core::mem::size_of::<pb_callback_t>() as uint32_t & 0xfff as uint32_t)
                << 16 as ::core::ffi::c_int
            | (2 as ::core::ffi::c_int as uint32_t & 0x3c0 as uint32_t)
                << 22 as ::core::ffi::c_int,
        (0 as ::core::ffi::c_int
            | (3 as ::core::ffi::c_int) << 2 as ::core::ffi::c_int
                & 0xff as ::core::ffi::c_int) as uint32_t
            | (0 as uint32_t | 0x10 as uint32_t | 0 as uint32_t)
                << 8 as ::core::ffi::c_int
            | (32 as ::core::ffi::c_ulong as uint32_t & 0xff as uint32_t)
                << 16 as ::core::ffi::c_int
            | (0 as ::core::ffi::c_int as uint32_t & 0xf as uint32_t)
                << 24 as ::core::ffi::c_int
            | (::core::mem::size_of::<bool>() as uint32_t & 0xf as uint32_t)
                << 28 as ::core::ffi::c_int,
        (1 as ::core::ffi::c_int
            | (4 as ::core::ffi::c_int) << 2 as ::core::ffi::c_int
                & 0xff as ::core::ffi::c_int) as uint32_t
            | (0x40 as uint32_t | 0x10 as uint32_t | 0x7 as uint32_t)
                << 8 as ::core::ffi::c_int
            | (1 as ::core::ffi::c_int as uint32_t & 0xfff as uint32_t)
                << 16 as ::core::ffi::c_int
            | (0 as ::core::ffi::c_int as uint32_t & 0xf as uint32_t)
                << 28 as ::core::ffi::c_int,
        40 as ::core::ffi::c_ulong as uint32_t & 0xffff as uint32_t
            | (::core::mem::size_of::<pb_callback_t>() as uint32_t & 0xfff as uint32_t)
                << 16 as ::core::ffi::c_int
            | (4 as ::core::ffi::c_int as uint32_t & 0x3c0 as uint32_t)
                << 22 as ::core::ffi::c_int,
        0 as ::core::ffi::c_int as uint32_t,
    ];
    krpc_schema_Type_field_info = [
        (0 as ::core::ffi::c_int
            | (1 as ::core::ffi::c_int) << 2 as ::core::ffi::c_int
                & 0xff as ::core::ffi::c_int) as uint32_t
            | (0 as uint32_t | 0x10 as uint32_t | 0x2 as uint32_t)
                << 8 as ::core::ffi::c_int
            | (0 as ::core::ffi::c_ulong as uint32_t & 0xff as uint32_t)
                << 16 as ::core::ffi::c_int
            | (0 as ::core::ffi::c_int as uint32_t & 0xf as uint32_t)
                << 24 as ::core::ffi::c_int
            | (::core::mem::size_of::<krpc_schema_Type_TypeCode>() as uint32_t
                & 0xf as uint32_t) << 28 as ::core::ffi::c_int,
        (1 as ::core::ffi::c_int
            | (2 as ::core::ffi::c_int) << 2 as ::core::ffi::c_int
                & 0xff as ::core::ffi::c_int) as uint32_t
            | (0x40 as uint32_t | 0x10 as uint32_t | 0x7 as uint32_t)
                << 8 as ::core::ffi::c_int
            | (1 as ::core::ffi::c_int as uint32_t & 0xfff as uint32_t)
                << 16 as ::core::ffi::c_int
            | (0 as ::core::ffi::c_int as uint32_t & 0xf as uint32_t)
                << 28 as ::core::ffi::c_int,
        8 as ::core::ffi::c_ulong as uint32_t & 0xffff as uint32_t
            | (::core::mem::size_of::<pb_callback_t>() as uint32_t & 0xfff as uint32_t)
                << 16 as ::core::ffi::c_int
            | (2 as ::core::ffi::c_int as uint32_t & 0x3c0 as uint32_t)
                << 22 as ::core::ffi::c_int,
        (1 as ::core::ffi::c_int
            | (3 as ::core::ffi::c_int) << 2 as ::core::ffi::c_int
                & 0xff as ::core::ffi::c_int) as uint32_t
            | (0x40 as uint32_t | 0x10 as uint32_t | 0x7 as uint32_t)
                << 8 as ::core::ffi::c_int
            | (1 as ::core::ffi::c_int as uint32_t & 0xfff as uint32_t)
                << 16 as ::core::ffi::c_int
            | (0 as ::core::ffi::c_int as uint32_t & 0xf as uint32_t)
                << 28 as ::core::ffi::c_int,
        24 as ::core::ffi::c_ulong as uint32_t & 0xffff as uint32_t
            | (::core::mem::size_of::<pb_callback_t>() as uint32_t & 0xfff as uint32_t)
                << 16 as ::core::ffi::c_int
            | (3 as ::core::ffi::c_int as uint32_t & 0x3c0 as uint32_t)
                << 22 as ::core::ffi::c_int,
        (1 as ::core::ffi::c_int
            | (4 as ::core::ffi::c_int) << 2 as ::core::ffi::c_int
                & 0xff as ::core::ffi::c_int) as uint32_t
            | (0x40 as uint32_t | 0x20 as uint32_t | 0x8 as uint32_t)
                << 8 as ::core::ffi::c_int
            | (1 as ::core::ffi::c_int as uint32_t & 0xfff as uint32_t)
                << 16 as ::core::ffi::c_int
            | (0 as ::core::ffi::c_int as uint32_t & 0xf as uint32_t)
                << 28 as ::core::ffi::c_int,
        40 as ::core::ffi::c_ulong as uint32_t & 0xffff as uint32_t
            | (::core::mem::size_of::<pb_callback_t>() as uint32_t & 0xfff as uint32_t)
                << 16 as ::core::ffi::c_int
            | (4 as ::core::ffi::c_int as uint32_t & 0x3c0 as uint32_t)
                << 22 as ::core::ffi::c_int,
        0 as ::core::ffi::c_int as uint32_t,
    ];
    krpc_schema_Parameter_field_info = [
        (1 as ::core::ffi::c_int
            | (1 as ::core::ffi::c_int) << 2 as ::core::ffi::c_int
                & 0xff as ::core::ffi::c_int) as uint32_t
            | (0x40 as uint32_t | 0x10 as uint32_t | 0x7 as uint32_t)
                << 8 as ::core::ffi::c_int
            | (1 as ::core::ffi::c_int as uint32_t & 0xfff as uint32_t)
                << 16 as ::core::ffi::c_int
            | (0 as ::core::ffi::c_int as uint32_t & 0xf as uint32_t)
                << 28 as ::core::ffi::c_int,
        0 as ::core::ffi::c_ulong as uint32_t & 0xffff as uint32_t
            | (::core::mem::size_of::<pb_callback_t>() as uint32_t & 0xfff as uint32_t)
                << 16 as ::core::ffi::c_int
            | (1 as ::core::ffi::c_int as uint32_t & 0x3c0 as uint32_t)
                << 22 as ::core::ffi::c_int,
        (1 as ::core::ffi::c_int
            | (2 as ::core::ffi::c_int) << 2 as ::core::ffi::c_int
                & 0xff as ::core::ffi::c_int) as uint32_t
            | (0 as uint32_t | 0x10 as uint32_t | 0x8 as uint32_t)
                << 8 as ::core::ffi::c_int
            | (1 as ::core::ffi::c_int as uint32_t & 0xfff as uint32_t)
                << 16 as ::core::ffi::c_int
            | ((24 as ::core::ffi::c_ulong as ::core::ffi::c_int
                - 16 as ::core::ffi::c_ulong as ::core::ffi::c_int) as uint32_t
                & 0xf as uint32_t) << 28 as ::core::ffi::c_int,
        24 as ::core::ffi::c_ulong as uint32_t & 0xffff as uint32_t
            | (::core::mem::size_of::<krpc_schema_Type>() as uint32_t
                & 0xfff as uint32_t) << 16 as ::core::ffi::c_int
            | (2 as ::core::ffi::c_int as uint32_t & 0x3c0 as uint32_t)
                << 22 as ::core::ffi::c_int,
        (1 as ::core::ffi::c_int
            | (3 as ::core::ffi::c_int) << 2 as ::core::ffi::c_int
                & 0xff as ::core::ffi::c_int) as uint32_t
            | (0x40 as uint32_t | 0x10 as uint32_t | 0x6 as uint32_t)
                << 8 as ::core::ffi::c_int
            | (1 as ::core::ffi::c_int as uint32_t & 0xfff as uint32_t)
                << 16 as ::core::ffi::c_int
            | (0 as ::core::ffi::c_int as uint32_t & 0xf as uint32_t)
                << 28 as ::core::ffi::c_int,
        80 as ::core::ffi::c_ulong as uint32_t & 0xffff as uint32_t
            | (::core::mem::size_of::<pb_callback_t>() as uint32_t & 0xfff as uint32_t)
                << 16 as ::core::ffi::c_int
            | (3 as ::core::ffi::c_int as uint32_t & 0x3c0 as uint32_t)
                << 22 as ::core::ffi::c_int,
        (0 as ::core::ffi::c_int
            | (4 as ::core::ffi::c_int) << 2 as ::core::ffi::c_int
                & 0xff as ::core::ffi::c_int) as uint32_t
            | (0 as uint32_t | 0x10 as uint32_t | 0 as uint32_t)
                << 8 as ::core::ffi::c_int
            | (96 as ::core::ffi::c_ulong as uint32_t & 0xff as uint32_t)
                << 16 as ::core::ffi::c_int
            | (0 as ::core::ffi::c_int as uint32_t & 0xf as uint32_t)
                << 24 as ::core::ffi::c_int
            | (::core::mem::size_of::<bool>() as uint32_t & 0xf as uint32_t)
                << 28 as ::core::ffi::c_int,
        0 as ::core::ffi::c_int as uint32_t,
    ];
    krpc_schema_Procedure_field_info = [
        (1 as ::core::ffi::c_int
            | (1 as ::core::ffi::c_int) << 2 as ::core::ffi::c_int
                & 0xff as ::core::ffi::c_int) as uint32_t
            | (0x40 as uint32_t | 0x10 as uint32_t | 0x7 as uint32_t)
                << 8 as ::core::ffi::c_int
            | (1 as ::core::ffi::c_int as uint32_t & 0xfff as uint32_t)
                << 16 as ::core::ffi::c_int
            | (0 as ::core::ffi::c_int as uint32_t & 0xf as uint32_t)
                << 28 as ::core::ffi::c_int,
        0 as ::core::ffi::c_ulong as uint32_t & 0xffff as uint32_t
            | (::core::mem::size_of::<pb_callback_t>() as uint32_t & 0xfff as uint32_t)
                << 16 as ::core::ffi::c_int
            | (1 as ::core::ffi::c_int as uint32_t & 0x3c0 as uint32_t)
                << 22 as ::core::ffi::c_int,
        (1 as ::core::ffi::c_int
            | (2 as ::core::ffi::c_int) << 2 as ::core::ffi::c_int
                & 0xff as ::core::ffi::c_int) as uint32_t
            | (0x40 as uint32_t | 0x20 as uint32_t | 0x8 as uint32_t)
                << 8 as ::core::ffi::c_int
            | (1 as ::core::ffi::c_int as uint32_t & 0xfff as uint32_t)
                << 16 as ::core::ffi::c_int
            | (0 as ::core::ffi::c_int as uint32_t & 0xf as uint32_t)
                << 28 as ::core::ffi::c_int,
        16 as ::core::ffi::c_ulong as uint32_t & 0xffff as uint32_t
            | (::core::mem::size_of::<pb_callback_t>() as uint32_t & 0xfff as uint32_t)
                << 16 as ::core::ffi::c_int
            | (2 as ::core::ffi::c_int as uint32_t & 0x3c0 as uint32_t)
                << 22 as ::core::ffi::c_int,
        (1 as ::core::ffi::c_int
            | (3 as ::core::ffi::c_int) << 2 as ::core::ffi::c_int
                & 0xff as ::core::ffi::c_int) as uint32_t
            | (0 as uint32_t | 0x10 as uint32_t | 0x8 as uint32_t)
                << 8 as ::core::ffi::c_int
            | (1 as ::core::ffi::c_int as uint32_t & 0xfff as uint32_t)
                << 16 as ::core::ffi::c_int
            | ((40 as ::core::ffi::c_ulong as ::core::ffi::c_int
                - 32 as ::core::ffi::c_ulong as ::core::ffi::c_int) as uint32_t
                & 0xf as uint32_t) << 28 as ::core::ffi::c_int,
        40 as ::core::ffi::c_ulong as uint32_t & 0xffff as uint32_t
            | (::core::mem::size_of::<krpc_schema_Type>() as uint32_t
                & 0xfff as uint32_t) << 16 as ::core::ffi::c_int
            | (3 as ::core::ffi::c_int as uint32_t & 0x3c0 as uint32_t)
                << 22 as ::core::ffi::c_int,
        (0 as ::core::ffi::c_int
            | (4 as ::core::ffi::c_int) << 2 as ::core::ffi::c_int
                & 0xff as ::core::ffi::c_int) as uint32_t
            | (0 as uint32_t | 0x10 as uint32_t | 0 as uint32_t)
                << 8 as ::core::ffi::c_int
            | (96 as ::core::ffi::c_ulong as uint32_t & 0xff as uint32_t)
                << 16 as ::core::ffi::c_int
            | (0 as ::core::ffi::c_int as uint32_t & 0xf as uint32_t)
                << 24 as ::core::ffi::c_int
            | (::core::mem::size_of::<bool>() as uint32_t & 0xf as uint32_t)
                << 28 as ::core::ffi::c_int,
        (1 as ::core::ffi::c_int
            | (5 as ::core::ffi::c_int) << 2 as ::core::ffi::c_int
                & 0xff as ::core::ffi::c_int) as uint32_t
            | (0x40 as uint32_t | 0x10 as uint32_t | 0x7 as uint32_t)
                << 8 as ::core::ffi::c_int
            | (1 as ::core::ffi::c_int as uint32_t & 0xfff as uint32_t)
                << 16 as ::core::ffi::c_int
            | (0 as ::core::ffi::c_int as uint32_t & 0xf as uint32_t)
                << 28 as ::core::ffi::c_int,
        104 as ::core::ffi::c_ulong as uint32_t & 0xffff as uint32_t
            | (::core::mem::size_of::<pb_callback_t>() as uint32_t & 0xfff as uint32_t)
                << 16 as ::core::ffi::c_int
            | (5 as ::core::ffi::c_int as uint32_t & 0x3c0 as uint32_t)
                << 22 as ::core::ffi::c_int,
        (1 as ::core::ffi::c_int
            | (6 as ::core::ffi::c_int) << 2 as ::core::ffi::c_int
                & 0xff as ::core::ffi::c_int) as uint32_t
            | (0x40 as uint32_t | 0x20 as uint32_t | 0x2 as uint32_t)
                << 8 as ::core::ffi::c_int
            | (1 as ::core::ffi::c_int as uint32_t & 0xfff as uint32_t)
                << 16 as ::core::ffi::c_int
            | (0 as ::core::ffi::c_int as uint32_t & 0xf as uint32_t)
                << 28 as ::core::ffi::c_int,
        120 as ::core::ffi::c_ulong as uint32_t & 0xffff as uint32_t
            | (::core::mem::size_of::<pb_callback_t>() as uint32_t & 0xfff as uint32_t)
                << 16 as ::core::ffi::c_int
            | (6 as ::core::ffi::c_int as uint32_t & 0x3c0 as uint32_t)
                << 22 as ::core::ffi::c_int,
        (0 as ::core::ffi::c_int
            | (7 as ::core::ffi::c_int) << 2 as ::core::ffi::c_int
                & 0xff as ::core::ffi::c_int) as uint32_t
            | (0 as uint32_t | 0x10 as uint32_t | 0 as uint32_t)
                << 8 as ::core::ffi::c_int
            | (136 as ::core::ffi::c_ulong as uint32_t & 0xff as uint32_t)
                << 16 as ::core::ffi::c_int
            | (0 as ::core::ffi::c_int as uint32_t & 0xf as uint32_t)
                << 24 as ::core::ffi::c_int
            | (::core::mem::size_of::<bool>() as uint32_t & 0xf as uint32_t)
                << 28 as ::core::ffi::c_int,
        (1 as ::core::ffi::c_int
            | (8 as ::core::ffi::c_int) << 2 as ::core::ffi::c_int
                & 0xff as ::core::ffi::c_int) as uint32_t
            | (0x40 as uint32_t | 0x10 as uint32_t | 0x7 as uint32_t)
                << 8 as ::core::ffi::c_int
            | (1 as ::core::ffi::c_int as uint32_t & 0xfff as uint32_t)
                << 16 as ::core::ffi::c_int
            | (0 as ::core::ffi::c_int as uint32_t & 0xf as uint32_t)
                << 28 as ::core::ffi::c_int,
        144 as ::core::ffi::c_ulong as uint32_t & 0xffff as uint32_t
            | (::core::mem::size_of::<pb_callback_t>() as uint32_t & 0xfff as uint32_t)
                << 16 as ::core::ffi::c_int
            | (8 as ::core::ffi::c_int as uint32_t & 0x3c0 as uint32_t)
                << 22 as ::core::ffi::c_int,
        0 as ::core::ffi::c_int as uint32_t,
    ];
    krpc_schema_Service_field_info = [
        (1 as ::core::ffi::c_int
            | (1 as ::core::ffi::c_int) << 2 as ::core::ffi::c_int
                & 0xff as ::core::ffi::c_int) as uint32_t
            | (0x40 as uint32_t | 0x10 as uint32_t | 0x7 as uint32_t)
                << 8 as ::core::ffi::c_int
            | (1 as ::core::ffi::c_int as uint32_t & 0xfff as uint32_t)
                << 16 as ::core::ffi::c_int
            | (0 as ::core::ffi::c_int as uint32_t & 0xf as uint32_t)
                << 28 as ::core::ffi::c_int,
        0 as ::core::ffi::c_ulong as uint32_t & 0xffff as uint32_t
            | (::core::mem::size_of::<pb_callback_t>() as uint32_t & 0xfff as uint32_t)
                << 16 as ::core::ffi::c_int
            | (1 as ::core::ffi::c_int as uint32_t & 0x3c0 as uint32_t)
                << 22 as ::core::ffi::c_int,
        (1 as ::core::ffi::c_int
            | (2 as ::core::ffi::c_int) << 2 as ::core::ffi::c_int
                & 0xff as ::core::ffi::c_int) as uint32_t
            | (0x40 as uint32_t | 0x20 as uint32_t | 0x8 as uint32_t)
                << 8 as ::core::ffi::c_int
            | (1 as ::core::ffi::c_int as uint32_t & 0xfff as uint32_t)
                << 16 as ::core::ffi::c_int
            | (0 as ::core::ffi::c_int as uint32_t & 0xf as uint32_t)
                << 28 as ::core::ffi::c_int,
        16 as ::core::ffi::c_ulong as uint32_t & 0xffff as uint32_t
            | (::core::mem::size_of::<pb_callback_t>() as uint32_t & 0xfff as uint32_t)
                << 16 as ::core::ffi::c_int
            | (2 as ::core::ffi::c_int as uint32_t & 0x3c0 as uint32_t)
                << 22 as ::core::ffi::c_int,
        (1 as ::core::ffi::c_int
            | (3 as ::core::ffi::c_int) << 2 as ::core::ffi::c_int
                & 0xff as ::core::ffi::c_int) as uint32_t
            | (0x40 as uint32_t | 0x20 as uint32_t | 0x8 as uint32_t)
                << 8 as ::core::ffi::c_int
            | (1 as ::core::ffi::c_int as uint32_t & 0xfff as uint32_t)
                << 16 as ::core::ffi::c_int
            | (0 as ::core::ffi::c_int as uint32_t & 0xf as uint32_t)
                << 28 as ::core::ffi::c_int,
        32 as ::core::ffi::c_ulong as uint32_t & 0xffff as uint32_t
            | (::core::mem::size_of::<pb_callback_t>() as uint32_t & 0xfff as uint32_t)
                << 16 as ::core::ffi::c_int
            | (3 as ::core::ffi::c_int as uint32_t & 0x3c0 as uint32_t)
                << 22 as ::core::ffi::c_int,
        (1 as ::core::ffi::c_int
            | (4 as ::core::ffi::c_int) << 2 as ::core::ffi::c_int
                & 0xff as ::core::ffi::c_int) as uint32_t
            | (0x40 as uint32_t | 0x20 as uint32_t | 0x8 as uint32_t)
                << 8 as ::core::ffi::c_int
            | (1 as ::core::ffi::c_int as uint32_t & 0xfff as uint32_t)
                << 16 as ::core::ffi::c_int
            | (0 as ::core::ffi::c_int as uint32_t & 0xf as uint32_t)
                << 28 as ::core::ffi::c_int,
        48 as ::core::ffi::c_ulong as uint32_t & 0xffff as uint32_t
            | (::core::mem::size_of::<pb_callback_t>() as uint32_t & 0xfff as uint32_t)
                << 16 as ::core::ffi::c_int
            | (4 as ::core::ffi::c_int as uint32_t & 0x3c0 as uint32_t)
                << 22 as ::core::ffi::c_int,
        (1 as ::core::ffi::c_int
            | (5 as ::core::ffi::c_int) << 2 as ::core::ffi::c_int
                & 0xff as ::core::ffi::c_int) as uint32_t
            | (0x40 as uint32_t | 0x20 as uint32_t | 0x8 as uint32_t)
                << 8 as ::core::ffi::c_int
            | (1 as ::core::ffi::c_int as uint32_t & 0xfff as uint32_t)
                << 16 as ::core::ffi::c_int
            | (0 as ::core::ffi::c_int as uint32_t & 0xf as uint32_t)
                << 28 as ::core::ffi::c_int,
        64 as ::core::ffi::c_ulong as uint32_t & 0xffff as uint32_t
            | (::core::mem::size_of::<pb_callback_t>() as uint32_t & 0xfff as uint32_t)
                << 16 as ::core::ffi::c_int
            | (5 as ::core::ffi::c_int as uint32_t & 0x3c0 as uint32_t)
                << 22 as ::core::ffi::c_int,
        (1 as ::core::ffi::c_int
            | (6 as ::core::ffi::c_int) << 2 as ::core::ffi::c_int
                & 0xff as ::core::ffi::c_int) as uint32_t
            | (0x40 as uint32_t | 0x10 as uint32_t | 0x7 as uint32_t)
                << 8 as ::core::ffi::c_int
            | (1 as ::core::ffi::c_int as uint32_t & 0xfff as uint32_t)
                << 16 as ::core::ffi::c_int
            | (0 as ::core::ffi::c_int as uint32_t & 0xf as uint32_t)
                << 28 as ::core::ffi::c_int,
        80 as ::core::ffi::c_ulong as uint32_t & 0xffff as uint32_t
            | (::core::mem::size_of::<pb_callback_t>() as uint32_t & 0xfff as uint32_t)
                << 16 as ::core::ffi::c_int
            | (6 as ::core::ffi::c_int as uint32_t & 0x3c0 as uint32_t)
                << 22 as ::core::ffi::c_int,
        (0 as ::core::ffi::c_int
            | (7 as ::core::ffi::c_int) << 2 as ::core::ffi::c_int
                & 0xff as ::core::ffi::c_int) as uint32_t
            | (0 as uint32_t | 0x10 as uint32_t | 0 as uint32_t)
                << 8 as ::core::ffi::c_int
            | (96 as ::core::ffi::c_ulong as uint32_t & 0xff as uint32_t)
                << 16 as ::core::ffi::c_int
            | (0 as ::core::ffi::c_int as uint32_t & 0xf as uint32_t)
                << 24 as ::core::ffi::c_int
            | (::core::mem::size_of::<bool>() as uint32_t & 0xf as uint32_t)
                << 28 as ::core::ffi::c_int,
        (1 as ::core::ffi::c_int
            | (8 as ::core::ffi::c_int) << 2 as ::core::ffi::c_int
                & 0xff as ::core::ffi::c_int) as uint32_t
            | (0x40 as uint32_t | 0x10 as uint32_t | 0x7 as uint32_t)
                << 8 as ::core::ffi::c_int
            | (1 as ::core::ffi::c_int as uint32_t & 0xfff as uint32_t)
                << 16 as ::core::ffi::c_int
            | (0 as ::core::ffi::c_int as uint32_t & 0xf as uint32_t)
                << 28 as ::core::ffi::c_int,
        104 as ::core::ffi::c_ulong as uint32_t & 0xffff as uint32_t
            | (::core::mem::size_of::<pb_callback_t>() as uint32_t & 0xfff as uint32_t)
                << 16 as ::core::ffi::c_int
            | (8 as ::core::ffi::c_int as uint32_t & 0x3c0 as uint32_t)
                << 22 as ::core::ffi::c_int,
        0 as ::core::ffi::c_int as uint32_t,
    ];
    krpc_schema_Services_field_info = [
        (1 as ::core::ffi::c_int
            | (1 as ::core::ffi::c_int) << 2 as ::core::ffi::c_int
                & 0xff as ::core::ffi::c_int) as uint32_t
            | (0x40 as uint32_t | 0x20 as uint32_t | 0x8 as uint32_t)
                << 8 as ::core::ffi::c_int
            | (1 as ::core::ffi::c_int as uint32_t & 0xfff as uint32_t)
                << 16 as ::core::ffi::c_int
            | (0 as ::core::ffi::c_int as uint32_t & 0xf as uint32_t)
                << 28 as ::core::ffi::c_int,
        0 as ::core::ffi::c_ulong as uint32_t & 0xffff as uint32_t
            | (::core::mem::size_of::<pb_callback_t>() as uint32_t & 0xfff as uint32_t)
                << 16 as ::core::ffi::c_int
            | (1 as ::core::ffi::c_int as uint32_t & 0x3c0 as uint32_t)
                << 22 as ::core::ffi::c_int,
        0 as ::core::ffi::c_int as uint32_t,
    ];
    krpc_schema_Tuple_field_info = [
        (1 as ::core::ffi::c_int
            | (1 as ::core::ffi::c_int) << 2 as ::core::ffi::c_int
                & 0xff as ::core::ffi::c_int) as uint32_t
            | (0x40 as uint32_t | 0x20 as uint32_t | 0x6 as uint32_t)
                << 8 as ::core::ffi::c_int
            | (1 as ::core::ffi::c_int as uint32_t & 0xfff as uint32_t)
                << 16 as ::core::ffi::c_int
            | (0 as ::core::ffi::c_int as uint32_t & 0xf as uint32_t)
                << 28 as ::core::ffi::c_int,
        0 as ::core::ffi::c_ulong as uint32_t & 0xffff as uint32_t
            | (::core::mem::size_of::<pb_callback_t>() as uint32_t & 0xfff as uint32_t)
                << 16 as ::core::ffi::c_int
            | (1 as ::core::ffi::c_int as uint32_t & 0x3c0 as uint32_t)
                << 22 as ::core::ffi::c_int,
        0 as ::core::ffi::c_int as uint32_t,
    ];
    krpc_schema_List_field_info = [
        (1 as ::core::ffi::c_int
            | (1 as ::core::ffi::c_int) << 2 as ::core::ffi::c_int
                & 0xff as ::core::ffi::c_int) as uint32_t
            | (0x40 as uint32_t | 0x20 as uint32_t | 0x6 as uint32_t)
                << 8 as ::core::ffi::c_int
            | (1 as ::core::ffi::c_int as uint32_t & 0xfff as uint32_t)
                << 16 as ::core::ffi::c_int
            | (0 as ::core::ffi::c_int as uint32_t & 0xf as uint32_t)
                << 28 as ::core::ffi::c_int,
        0 as ::core::ffi::c_ulong as uint32_t & 0xffff as uint32_t
            | (::core::mem::size_of::<pb_callback_t>() as uint32_t & 0xfff as uint32_t)
                << 16 as ::core::ffi::c_int
            | (1 as ::core::ffi::c_int as uint32_t & 0x3c0 as uint32_t)
                << 22 as ::core::ffi::c_int,
        0 as ::core::ffi::c_int as uint32_t,
    ];
    krpc_schema_Set_field_info = [
        (1 as ::core::ffi::c_int
            | (1 as ::core::ffi::c_int) << 2 as ::core::ffi::c_int
                & 0xff as ::core::ffi::c_int) as uint32_t
            | (0x40 as uint32_t | 0x20 as uint32_t | 0x6 as uint32_t)
                << 8 as ::core::ffi::c_int
            | (1 as ::core::ffi::c_int as uint32_t & 0xfff as uint32_t)
                << 16 as ::core::ffi::c_int
            | (0 as ::core::ffi::c_int as uint32_t & 0xf as uint32_t)
                << 28 as ::core::ffi::c_int,
        0 as ::core::ffi::c_ulong as uint32_t & 0xffff as uint32_t
            | (::core::mem::size_of::<pb_callback_t>() as uint32_t & 0xfff as uint32_t)
                << 16 as ::core::ffi::c_int
            | (1 as ::core::ffi::c_int as uint32_t & 0x3c0 as uint32_t)
                << 22 as ::core::ffi::c_int,
        0 as ::core::ffi::c_int as uint32_t,
    ];
    krpc_schema_DictionaryEntry_field_info = [
        (1 as ::core::ffi::c_int
            | (1 as ::core::ffi::c_int) << 2 as ::core::ffi::c_int
                & 0xff as ::core::ffi::c_int) as uint32_t
            | (0x40 as uint32_t | 0x10 as uint32_t | 0x6 as uint32_t)
                << 8 as ::core::ffi::c_int
            | (1 as ::core::ffi::c_int as uint32_t & 0xfff as uint32_t)
                << 16 as ::core::ffi::c_int
            | (0 as ::core::ffi::c_int as uint32_t & 0xf as uint32_t)
                << 28 as ::core::ffi::c_int,
        0 as ::core::ffi::c_ulong as uint32_t & 0xffff as uint32_t
            | (::core::mem::size_of::<pb_callback_t>() as uint32_t & 0xfff as uint32_t)
                << 16 as ::core::ffi::c_int
            | (1 as ::core::ffi::c_int as uint32_t & 0x3c0 as uint32_t)
                << 22 as ::core::ffi::c_int,
        (1 as ::core::ffi::c_int
            | (2 as ::core::ffi::c_int) << 2 as ::core::ffi::c_int
                & 0xff as ::core::ffi::c_int) as uint32_t
            | (0x40 as uint32_t | 0x10 as uint32_t | 0x6 as uint32_t)
                << 8 as ::core::ffi::c_int
            | (1 as ::core::ffi::c_int as uint32_t & 0xfff as uint32_t)
                << 16 as ::core::ffi::c_int
            | (0 as ::core::ffi::c_int as uint32_t & 0xf as uint32_t)
                << 28 as ::core::ffi::c_int,
        16 as ::core::ffi::c_ulong as uint32_t & 0xffff as uint32_t
            | (::core::mem::size_of::<pb_callback_t>() as uint32_t & 0xfff as uint32_t)
                << 16 as ::core::ffi::c_int
            | (2 as ::core::ffi::c_int as uint32_t & 0x3c0 as uint32_t)
                << 22 as ::core::ffi::c_int,
        0 as ::core::ffi::c_int as uint32_t,
    ];
    krpc_schema_Dictionary_field_info = [
        (1 as ::core::ffi::c_int
            | (1 as ::core::ffi::c_int) << 2 as ::core::ffi::c_int
                & 0xff as ::core::ffi::c_int) as uint32_t
            | (0x40 as uint32_t | 0x20 as uint32_t | 0x8 as uint32_t)
                << 8 as ::core::ffi::c_int
            | (1 as ::core::ffi::c_int as uint32_t & 0xfff as uint32_t)
                << 16 as ::core::ffi::c_int
            | (0 as ::core::ffi::c_int as uint32_t & 0xf as uint32_t)
                << 28 as ::core::ffi::c_int,
        0 as ::core::ffi::c_ulong as uint32_t & 0xffff as uint32_t
            | (::core::mem::size_of::<pb_callback_t>() as uint32_t & 0xfff as uint32_t)
                << 16 as ::core::ffi::c_int
            | (1 as ::core::ffi::c_int as uint32_t & 0x3c0 as uint32_t)
                << 22 as ::core::ffi::c_int,
        0 as ::core::ffi::c_int as uint32_t,
    ];
    krpc_schema_Stream_field_info = [
        (0 as ::core::ffi::c_int
            | (1 as ::core::ffi::c_int) << 2 as ::core::ffi::c_int
                & 0xff as ::core::ffi::c_int) as uint32_t
            | (0 as uint32_t | 0x10 as uint32_t | 0x2 as uint32_t)
                << 8 as ::core::ffi::c_int
            | (0 as ::core::ffi::c_ulong as uint32_t & 0xff as uint32_t)
                << 16 as ::core::ffi::c_int
            | (0 as ::core::ffi::c_int as uint32_t & 0xf as uint32_t)
                << 24 as ::core::ffi::c_int
            | (::core::mem::size_of::<uint64_t>() as uint32_t & 0xf as uint32_t)
                << 28 as ::core::ffi::c_int,
        0 as ::core::ffi::c_int as uint32_t,
    ];
    krpc_schema_Event_field_info = [
        (1 as ::core::ffi::c_int
            | (1 as ::core::ffi::c_int) << 2 as ::core::ffi::c_int
                & 0xff as ::core::ffi::c_int) as uint32_t
            | (0 as uint32_t | 0x10 as uint32_t | 0x8 as uint32_t)
                << 8 as ::core::ffi::c_int
            | (1 as ::core::ffi::c_int as uint32_t & 0xfff as uint32_t)
                << 16 as ::core::ffi::c_int
            | ((8 as ::core::ffi::c_ulong as ::core::ffi::c_int
                - 0 as ::core::ffi::c_ulong as ::core::ffi::c_int) as uint32_t
                & 0xf as uint32_t) << 28 as ::core::ffi::c_int,
        8 as ::core::ffi::c_ulong as uint32_t & 0xffff as uint32_t
            | (::core::mem::size_of::<krpc_schema_Stream>() as uint32_t
                & 0xfff as uint32_t) << 16 as ::core::ffi::c_int
            | (1 as ::core::ffi::c_int as uint32_t & 0x3c0 as uint32_t)
                << 22 as ::core::ffi::c_int,
        0 as ::core::ffi::c_int as uint32_t,
    ];
    krpc_schema_Status_field_info = [
        (1 as ::core::ffi::c_int
            | (1 as ::core::ffi::c_int) << 2 as ::core::ffi::c_int
                & 0xff as ::core::ffi::c_int) as uint32_t
            | (0 as uint32_t | 0x10 as uint32_t | 0x7 as uint32_t)
                << 8 as ::core::ffi::c_int
            | (1 as ::core::ffi::c_int as uint32_t & 0xfff as uint32_t)
                << 16 as ::core::ffi::c_int
            | (0 as ::core::ffi::c_int as uint32_t & 0xf as uint32_t)
                << 28 as ::core::ffi::c_int,
        0 as ::core::ffi::c_ulong as uint32_t & 0xffff as uint32_t
            | (::core::mem::size_of::<[::core::ffi::c_char; 16]>() as uint32_t
                & 0xfff as uint32_t) << 16 as ::core::ffi::c_int
            | (1 as ::core::ffi::c_int as uint32_t & 0x3c0 as uint32_t)
                << 22 as ::core::ffi::c_int,
        (0 as ::core::ffi::c_int
            | (2 as ::core::ffi::c_int) << 2 as ::core::ffi::c_int
                & 0xff as ::core::ffi::c_int) as uint32_t
            | (0 as uint32_t | 0x10 as uint32_t | 0x2 as uint32_t)
                << 8 as ::core::ffi::c_int
            | (16 as ::core::ffi::c_ulong as uint32_t & 0xff as uint32_t)
                << 16 as ::core::ffi::c_int
            | (0 as ::core::ffi::c_int as uint32_t & 0xf as uint32_t)
                << 24 as ::core::ffi::c_int
            | (::core::mem::size_of::<uint64_t>() as uint32_t & 0xf as uint32_t)
                << 28 as ::core::ffi::c_int,
        (0 as ::core::ffi::c_int
            | (3 as ::core::ffi::c_int) << 2 as ::core::ffi::c_int
                & 0xff as ::core::ffi::c_int) as uint32_t
            | (0 as uint32_t | 0x10 as uint32_t | 0x2 as uint32_t)
                << 8 as ::core::ffi::c_int
            | (24 as ::core::ffi::c_ulong as uint32_t & 0xff as uint32_t)
                << 16 as ::core::ffi::c_int
            | (0 as ::core::ffi::c_int as uint32_t & 0xf as uint32_t)
                << 24 as ::core::ffi::c_int
            | (::core::mem::size_of::<uint64_t>() as uint32_t & 0xf as uint32_t)
                << 28 as ::core::ffi::c_int,
        (0 as ::core::ffi::c_int
            | (4 as ::core::ffi::c_int) << 2 as ::core::ffi::c_int
                & 0xff as ::core::ffi::c_int) as uint32_t
            | (0 as uint32_t | 0x10 as uint32_t | 0x4 as uint32_t)
                << 8 as ::core::ffi::c_int
            | (32 as ::core::ffi::c_ulong as uint32_t & 0xff as uint32_t)
                << 16 as ::core::ffi::c_int
            | (0 as ::core::ffi::c_int as uint32_t & 0xf as uint32_t)
                << 24 as ::core::ffi::c_int
            | (::core::mem::size_of::<::core::ffi::c_float>() as uint32_t
                & 0xf as uint32_t) << 28 as ::core::ffi::c_int,
        (0 as ::core::ffi::c_int
            | (5 as ::core::ffi::c_int) << 2 as ::core::ffi::c_int
                & 0xff as ::core::ffi::c_int) as uint32_t
            | (0 as uint32_t | 0x10 as uint32_t | 0x4 as uint32_t)
                << 8 as ::core::ffi::c_int
            | (36 as ::core::ffi::c_ulong as uint32_t & 0xff as uint32_t)
                << 16 as ::core::ffi::c_int
            | (0 as ::core::ffi::c_int as uint32_t & 0xf as uint32_t)
                << 24 as ::core::ffi::c_int
            | (::core::mem::size_of::<::core::ffi::c_float>() as uint32_t
                & 0xf as uint32_t) << 28 as ::core::ffi::c_int,
        (0 as ::core::ffi::c_int
            | (6 as ::core::ffi::c_int) << 2 as ::core::ffi::c_int
                & 0xff as ::core::ffi::c_int) as uint32_t
            | (0 as uint32_t | 0x10 as uint32_t | 0x2 as uint32_t)
                << 8 as ::core::ffi::c_int
            | (40 as ::core::ffi::c_ulong as uint32_t & 0xff as uint32_t)
                << 16 as ::core::ffi::c_int
            | (0 as ::core::ffi::c_int as uint32_t & 0xf as uint32_t)
                << 24 as ::core::ffi::c_int
            | (::core::mem::size_of::<uint64_t>() as uint32_t & 0xf as uint32_t)
                << 28 as ::core::ffi::c_int,
        (0 as ::core::ffi::c_int
            | (7 as ::core::ffi::c_int) << 2 as ::core::ffi::c_int
                & 0xff as ::core::ffi::c_int) as uint32_t
            | (0 as uint32_t | 0x10 as uint32_t | 0x4 as uint32_t)
                << 8 as ::core::ffi::c_int
            | (48 as ::core::ffi::c_ulong as uint32_t & 0xff as uint32_t)
                << 16 as ::core::ffi::c_int
            | (0 as ::core::ffi::c_int as uint32_t & 0xf as uint32_t)
                << 24 as ::core::ffi::c_int
            | (::core::mem::size_of::<::core::ffi::c_float>() as uint32_t
                & 0xf as uint32_t) << 28 as ::core::ffi::c_int,
        (0 as ::core::ffi::c_int
            | (8 as ::core::ffi::c_int) << 2 as ::core::ffi::c_int
                & 0xff as ::core::ffi::c_int) as uint32_t
            | (0 as uint32_t | 0x10 as uint32_t | 0 as uint32_t)
                << 8 as ::core::ffi::c_int
            | (52 as ::core::ffi::c_ulong as uint32_t & 0xff as uint32_t)
                << 16 as ::core::ffi::c_int
            | (0 as ::core::ffi::c_int as uint32_t & 0xf as uint32_t)
                << 24 as ::core::ffi::c_int
            | (::core::mem::size_of::<bool>() as uint32_t & 0xf as uint32_t)
                << 28 as ::core::ffi::c_int,
        (0 as ::core::ffi::c_int
            | (9 as ::core::ffi::c_int) << 2 as ::core::ffi::c_int
                & 0xff as ::core::ffi::c_int) as uint32_t
            | (0 as uint32_t | 0x10 as uint32_t | 0x2 as uint32_t)
                << 8 as ::core::ffi::c_int
            | (56 as ::core::ffi::c_ulong as uint32_t & 0xff as uint32_t)
                << 16 as ::core::ffi::c_int
            | (0 as ::core::ffi::c_int as uint32_t & 0xf as uint32_t)
                << 24 as ::core::ffi::c_int
            | (::core::mem::size_of::<uint32_t>() as uint32_t & 0xf as uint32_t)
                << 28 as ::core::ffi::c_int,
        (0 as ::core::ffi::c_int
            | (10 as ::core::ffi::c_int) << 2 as ::core::ffi::c_int
                & 0xff as ::core::ffi::c_int) as uint32_t
            | (0 as uint32_t | 0x10 as uint32_t | 0 as uint32_t)
                << 8 as ::core::ffi::c_int
            | (60 as ::core::ffi::c_ulong as uint32_t & 0xff as uint32_t)
                << 16 as ::core::ffi::c_int
            | (0 as ::core::ffi::c_int as uint32_t & 0xf as uint32_t)
                << 24 as ::core::ffi::c_int
            | (::core::mem::size_of::<bool>() as uint32_t & 0xf as uint32_t)
                << 28 as ::core::ffi::c_int,
        (0 as ::core::ffi::c_int
            | (11 as ::core::ffi::c_int) << 2 as ::core::ffi::c_int
                & 0xff as ::core::ffi::c_int) as uint32_t
            | (0 as uint32_t | 0x10 as uint32_t | 0 as uint32_t)
                << 8 as ::core::ffi::c_int
            | (61 as ::core::ffi::c_ulong as uint32_t & 0xff as uint32_t)
                << 16 as ::core::ffi::c_int
            | (0 as ::core::ffi::c_int as uint32_t & 0xf as uint32_t)
                << 24 as ::core::ffi::c_int
            | (::core::mem::size_of::<bool>() as uint32_t & 0xf as uint32_t)
                << 28 as ::core::ffi::c_int,
        (0 as ::core::ffi::c_int
            | (12 as ::core::ffi::c_int) << 2 as ::core::ffi::c_int
                & 0xff as ::core::ffi::c_int) as uint32_t
            | (0 as uint32_t | 0x10 as uint32_t | 0x2 as uint32_t)
                << 8 as ::core::ffi::c_int
            | (64 as ::core::ffi::c_ulong as uint32_t & 0xff as uint32_t)
                << 16 as ::core::ffi::c_int
            | (0 as ::core::ffi::c_int as uint32_t & 0xf as uint32_t)
                << 24 as ::core::ffi::c_int
            | (::core::mem::size_of::<uint32_t>() as uint32_t & 0xf as uint32_t)
                << 28 as ::core::ffi::c_int,
        (0 as ::core::ffi::c_int
            | (13 as ::core::ffi::c_int) << 2 as ::core::ffi::c_int
                & 0xff as ::core::ffi::c_int) as uint32_t
            | (0 as uint32_t | 0x10 as uint32_t | 0x4 as uint32_t)
                << 8 as ::core::ffi::c_int
            | (68 as ::core::ffi::c_ulong as uint32_t & 0xff as uint32_t)
                << 16 as ::core::ffi::c_int
            | (0 as ::core::ffi::c_int as uint32_t & 0xf as uint32_t)
                << 24 as ::core::ffi::c_int
            | (::core::mem::size_of::<::core::ffi::c_float>() as uint32_t
                & 0xf as uint32_t) << 28 as ::core::ffi::c_int,
        (0 as ::core::ffi::c_int
            | (14 as ::core::ffi::c_int) << 2 as ::core::ffi::c_int
                & 0xff as ::core::ffi::c_int) as uint32_t
            | (0 as uint32_t | 0x10 as uint32_t | 0x4 as uint32_t)
                << 8 as ::core::ffi::c_int
            | (72 as ::core::ffi::c_ulong as uint32_t & 0xff as uint32_t)
                << 16 as ::core::ffi::c_int
            | (0 as ::core::ffi::c_int as uint32_t & 0xf as uint32_t)
                << 24 as ::core::ffi::c_int
            | (::core::mem::size_of::<::core::ffi::c_float>() as uint32_t
                & 0xf as uint32_t) << 28 as ::core::ffi::c_int,
        (0 as ::core::ffi::c_int
            | (15 as ::core::ffi::c_int) << 2 as ::core::ffi::c_int
                & 0xff as ::core::ffi::c_int) as uint32_t
            | (0 as uint32_t | 0x10 as uint32_t | 0x4 as uint32_t)
                << 8 as ::core::ffi::c_int
            | (76 as ::core::ffi::c_ulong as uint32_t & 0xff as uint32_t)
                << 16 as ::core::ffi::c_int
            | (0 as ::core::ffi::c_int as uint32_t & 0xf as uint32_t)
                << 24 as ::core::ffi::c_int
            | (::core::mem::size_of::<::core::ffi::c_float>() as uint32_t
                & 0xf as uint32_t) << 28 as ::core::ffi::c_int,
        (0 as ::core::ffi::c_int
            | (16 as ::core::ffi::c_int) << 2 as ::core::ffi::c_int
                & 0xff as ::core::ffi::c_int) as uint32_t
            | (0 as uint32_t | 0x10 as uint32_t | 0x2 as uint32_t)
                << 8 as ::core::ffi::c_int
            | (80 as ::core::ffi::c_ulong as uint32_t & 0xff as uint32_t)
                << 16 as ::core::ffi::c_int
            | (0 as ::core::ffi::c_int as uint32_t & 0xf as uint32_t)
                << 24 as ::core::ffi::c_int
            | (::core::mem::size_of::<uint32_t>() as uint32_t & 0xf as uint32_t)
                << 28 as ::core::ffi::c_int,
        (0 as ::core::ffi::c_int
            | (17 as ::core::ffi::c_int) << 2 as ::core::ffi::c_int
                & 0xff as ::core::ffi::c_int) as uint32_t
            | (0 as uint32_t | 0x10 as uint32_t | 0x2 as uint32_t)
                << 8 as ::core::ffi::c_int
            | (88 as ::core::ffi::c_ulong as uint32_t & 0xff as uint32_t)
                << 16 as ::core::ffi::c_int
            | (0 as ::core::ffi::c_int as uint32_t & 0xf as uint32_t)
                << 24 as ::core::ffi::c_int
            | (::core::mem::size_of::<uint64_t>() as uint32_t & 0xf as uint32_t)
                << 28 as ::core::ffi::c_int,
        (0 as ::core::ffi::c_int
            | (18 as ::core::ffi::c_int) << 2 as ::core::ffi::c_int
                & 0xff as ::core::ffi::c_int) as uint32_t
            | (0 as uint32_t | 0x10 as uint32_t | 0x4 as uint32_t)
                << 8 as ::core::ffi::c_int
            | (96 as ::core::ffi::c_ulong as uint32_t & 0xff as uint32_t)
                << 16 as ::core::ffi::c_int
            | (0 as ::core::ffi::c_int as uint32_t & 0xf as uint32_t)
                << 24 as ::core::ffi::c_int
            | (::core::mem::size_of::<::core::ffi::c_float>() as uint32_t
                & 0xf as uint32_t) << 28 as ::core::ffi::c_int,
        (0 as ::core::ffi::c_int
            | (19 as ::core::ffi::c_int) << 2 as ::core::ffi::c_int
                & 0xff as ::core::ffi::c_int) as uint32_t
            | (0 as uint32_t | 0x10 as uint32_t | 0x4 as uint32_t)
                << 8 as ::core::ffi::c_int
            | (100 as ::core::ffi::c_ulong as uint32_t & 0xff as uint32_t)
                << 16 as ::core::ffi::c_int
            | (0 as ::core::ffi::c_int as uint32_t & 0xf as uint32_t)
                << 24 as ::core::ffi::c_int
            | (::core::mem::size_of::<::core::ffi::c_float>() as uint32_t
                & 0xf as uint32_t) << 28 as ::core::ffi::c_int,
        0 as ::core::ffi::c_int as uint32_t,
    ];
    krpc_schema_MultiplexedRequest_field_info = [
        (1 as ::core::ffi::c_int
            | (1 as ::core::ffi::c_int) << 2 as ::core::ffi::c_int
                & 0xff as ::core::ffi::c_int) as uint32_t
            | (0 as uint32_t | 0x10 as uint32_t | 0x8 as uint32_t)
                << 8 as ::core::ffi::c_int
            | (1 as ::core::ffi::c_int as uint32_t & 0xfff as uint32_t)
                << 16 as ::core::ffi::c_int
            | ((8 as ::core::ffi::c_ulong as ::core::ffi::c_int
                - 0 as ::core::ffi::c_ulong as ::core::ffi::c_int) as uint32_t
                & 0xf as uint32_t) << 28 as ::core::ffi::c_int,
        8 as ::core::ffi::c_ulong as uint32_t & 0xffff as uint32_t
            | (::core::mem::size_of::<krpc_schema_ConnectionRequest>() as uint32_t
                & 0xfff as uint32_t) << 16 as ::core::ffi::c_int
            | (1 as ::core::ffi::c_int as uint32_t & 0x3c0 as uint32_t)
                << 22 as ::core::ffi::c_int,
        (1 as ::core::ffi::c_int
            | (2 as ::core::ffi::c_int) << 2 as ::core::ffi::c_int
                & 0xff as ::core::ffi::c_int) as uint32_t
            | (0 as uint32_t | 0x10 as uint32_t | 0x8 as uint32_t)
                << 8 as ::core::ffi::c_int
            | (1 as ::core::ffi::c_int as uint32_t & 0xfff as uint32_t)
                << 16 as ::core::ffi::c_int
            | ((56 as ::core::ffi::c_ulong as ::core::ffi::c_int
                - 48 as ::core::ffi::c_ulong as ::core::ffi::c_int) as uint32_t
                & 0xf as uint32_t) << 28 as ::core::ffi::c_int,
        56 as ::core::ffi::c_ulong as uint32_t & 0xffff as uint32_t
            | (::core::mem::size_of::<krpc_schema_Request>() as uint32_t
                & 0xfff as uint32_t) << 16 as ::core::ffi::c_int
            | (2 as ::core::ffi::c_int as uint32_t & 0x3c0 as uint32_t)
                << 22 as ::core::ffi::c_int,
        0 as ::core::ffi::c_int as uint32_t,
    ];
    krpc_schema_MultiplexedResponse_field_info = [
        (1 as ::core::ffi::c_int
            | (1 as ::core::ffi::c_int) << 2 as ::core::ffi::c_int
                & 0xff as ::core::ffi::c_int) as uint32_t
            | (0 as uint32_t | 0x10 as uint32_t | 0x8 as uint32_t)
                << 8 as ::core::ffi::c_int
            | (1 as ::core::ffi::c_int as uint32_t & 0xfff as uint32_t)
                << 16 as ::core::ffi::c_int
            | ((8 as ::core::ffi::c_ulong as ::core::ffi::c_int
                - 0 as ::core::ffi::c_ulong as ::core::ffi::c_int) as uint32_t
                & 0xf as uint32_t) << 28 as ::core::ffi::c_int,
        8 as ::core::ffi::c_ulong as uint32_t & 0xffff as uint32_t
            | (::core::mem::size_of::<krpc_schema_Response>() as uint32_t
                & 0xfff as uint32_t) << 16 as ::core::ffi::c_int
            | (1 as ::core::ffi::c_int as uint32_t & 0x3c0 as uint32_t)
                << 22 as ::core::ffi::c_int,
        (1 as ::core::ffi::c_int
            | (2 as ::core::ffi::c_int) << 2 as ::core::ffi::c_int
                & 0xff as ::core::ffi::c_int) as uint32_t
            | (0 as uint32_t | 0x10 as uint32_t | 0x8 as uint32_t)
                << 8 as ::core::ffi::c_int
            | (1 as ::core::ffi::c_int as uint32_t & 0xfff as uint32_t)
                << 16 as ::core::ffi::c_int
            | ((1064 as ::core::ffi::c_ulong as ::core::ffi::c_int
                - 1056 as ::core::ffi::c_ulong as ::core::ffi::c_int) as uint32_t
                & 0xf as uint32_t) << 28 as ::core::ffi::c_int,
        1064 as ::core::ffi::c_ulong as uint32_t & 0xffff as uint32_t
            | (::core::mem::size_of::<krpc_schema_StreamUpdate>() as uint32_t
                & 0xfff as uint32_t) << 16 as ::core::ffi::c_int
            | (2 as ::core::ffi::c_int as uint32_t & 0x3c0 as uint32_t)
                << 22 as ::core::ffi::c_int,
        0 as ::core::ffi::c_int as uint32_t,
    ];
}
#[used]
#[cfg_attr(target_os = "linux", link_section = ".init_array")]
#[cfg_attr(target_os = "windows", link_section = ".CRT$XIB")]
#[cfg_attr(target_os = "macos", link_section = "__DATA,__mod_init_func")]
static INIT_ARRAY: [unsafe extern "C" fn(); 1] = [run_static_initializers];
#[repr(C)]
pub struct pb_istream_s {
    _private: [u8; 0],
}
#[repr(C)]
pub struct pb_ostream_s {
    _private: [u8; 0],
}
