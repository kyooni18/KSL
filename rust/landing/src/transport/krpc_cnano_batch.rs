use crate::sys::*;
extern "C" {
    fn krpc_read(
        connection: krpc_connection_t,
        buf: *mut uint8_t,
        count: size_t,
    ) -> krpc_error_t;
    fn krpc_write(
        connection: krpc_connection_t,
        buf: *const uint8_t,
        count: size_t,
    ) -> krpc_error_t;
    fn krpc_cnano_transport_uses_standard_rpc(connection: krpc_connection_t) -> bool;
    fn krpc_cnano_transport_begin_deadline(connection: krpc_connection_t) -> bool;
    fn krpc_cnano_transport_end_deadline(connection: krpc_connection_t);
    static krpc_schema_Request_msg: pb_msgdesc_t;
    static krpc_schema_Response_msg: pb_msgdesc_t;
    static krpc_schema_MultiplexedRequest_msg: pb_msgdesc_t;
    static krpc_schema_MultiplexedResponse_msg: pb_msgdesc_t;
    fn pb_decode_ex(
        stream: *mut pb_istream_t,
        fields: *const pb_msgdesc_t,
        dest_struct: *mut ::core::ffi::c_void,
        flags: ::core::ffi::c_uint,
    ) -> bool;
    fn pb_read(stream: *mut pb_istream_t, buf: *mut pb_byte_t, count: size_t) -> bool;
    fn pb_encode_ex(
        stream: *mut pb_ostream_t,
        fields: *const pb_msgdesc_t,
        src_struct: *const ::core::ffi::c_void,
        flags: ::core::ffi::c_uint,
    ) -> bool;
}
pub type size_t = usize;
pub type __uint8_t = u8;
pub type __uint16_t = u16;
pub type __uint32_t = u32;
pub type __int64_t = i64;
pub type __uint_least16_t = __uint16_t;
pub type int64_t = __int64_t;
pub type uint8_t = __uint8_t;
pub type uint32_t = __uint32_t;
pub type uint_least16_t = __uint_least16_t;
pub type intptr_t = isize;
pub type KrpcCNanoTransportStatus = ::core::ffi::c_uint;
pub const KRPC_CNANO_TRANSPORT_ZERO_PROGRESS: KrpcCNanoTransportStatus = 6;
pub const KRPC_CNANO_TRANSPORT_INVALID: KrpcCNanoTransportStatus = 5;
pub const KRPC_CNANO_TRANSPORT_CLOSED: KrpcCNanoTransportStatus = 4;
pub const KRPC_CNANO_TRANSPORT_IO_ERROR: KrpcCNanoTransportStatus = 3;
pub const KRPC_CNANO_TRANSPORT_TIMEOUT: KrpcCNanoTransportStatus = 2;
pub const KRPC_CNANO_TRANSPORT_EOF: KrpcCNanoTransportStatus = 1;
pub const KRPC_CNANO_TRANSPORT_OK: KrpcCNanoTransportStatus = 0;
pub type KrpcCNanoTransportProtocol = ::core::ffi::c_uint;
pub const KRPC_CNANO_PROTOCOL_TCP_RPC: KrpcCNanoTransportProtocol = 1;
pub const KRPC_CNANO_PROTOCOL_MULTIPLEXED: KrpcCNanoTransportProtocol = 0;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct KrpcCNanoTransport {
    pub ops: *const KrpcCNanoTransportOps,
    pub context: *mut ::core::ffi::c_void,
    pub last_status: KrpcCNanoTransportStatus,
    pub bytes_read: size_t,
    pub bytes_written: size_t,
    pub timeout_ms: ::core::ffi::c_int,
    pub deadline_ns: int64_t,
    pub deadline_active: bool,
    pub is_open: bool,
}
#[derive(Copy, Clone)]
#[repr(C)]
pub struct KrpcCNanoTransportOps {
    pub open: KrpcCNanoTransportOpenFn,
    pub close: KrpcCNanoTransportCloseFn,
    pub read: KrpcCNanoTransportReadFn,
    pub write: KrpcCNanoTransportWriteFn,
    pub protocol: KrpcCNanoTransportProtocol,
}
pub type KrpcCNanoTransportWriteFn = Option<
    unsafe extern "C" fn(
        *mut ::core::ffi::c_void,
        *const uint8_t,
        size_t,
        *mut size_t,
        ::core::ffi::c_int,
    ) -> KrpcCNanoTransportStatus,
>;
pub type KrpcCNanoTransportReadFn = Option<
    unsafe extern "C" fn(
        *mut ::core::ffi::c_void,
        *mut uint8_t,
        size_t,
        *mut size_t,
        ::core::ffi::c_int,
    ) -> KrpcCNanoTransportStatus,
>;
pub type KrpcCNanoTransportCloseFn = Option<
    unsafe extern "C" fn(*mut ::core::ffi::c_void) -> KrpcCNanoTransportStatus,
>;
pub type KrpcCNanoTransportOpenFn = Option<
    unsafe extern "C" fn(*mut ::core::ffi::c_void) -> KrpcCNanoTransportStatus,
>;
pub type krpc_connection_t = *mut KrpcCNanoTransport;
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
pub type _krpc_schema_ConnectionRequest_Type = ::core::ffi::c_uint;
pub const krpc_schema_ConnectionRequest_Type_STREAM: _krpc_schema_ConnectionRequest_Type = 1;
pub const krpc_schema_ConnectionRequest_Type_RPC: _krpc_schema_ConnectionRequest_Type = 0;
pub type krpc_schema_ConnectionRequest_Type = _krpc_schema_ConnectionRequest_Type;
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
pub struct _krpc_schema_StreamUpdate {
    pub results: pb_callback_t,
}
pub type krpc_schema_StreamUpdate = _krpc_schema_StreamUpdate;
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
#[derive(Copy, Clone)]
#[repr(C)]
pub struct krpc_result_t {
    pub message: krpc_schema_ProcedureResult,
    pub size: size_t,
    pub buffer: *mut uint8_t,
}
#[derive(Copy, Clone)]
#[repr(C)]
pub struct BatchErrorState {
    pub failed: bool,
}
pub const true_0: ::core::ffi::c_int = 1 as ::core::ffi::c_int;
pub const false_0: ::core::ffi::c_int = 0 as ::core::ffi::c_int;
pub const SIZE_MAX: ::core::ffi::c_ulong = 18446744073709551615 as ::core::ffi::c_ulong;
pub const NULL: *mut ::core::ffi::c_void = ::core::ptr::null_mut::<
    ::core::ffi::c_void,
>();
pub const KRPC_CNANO_BATCH_MAX: ::core::ffi::c_uint = 32 as ::core::ffi::c_uint;
pub const PB_DECODE_DELIMITED: ::core::ffi::c_uint = 0x2 as ::core::ffi::c_uint;
pub const PB_ENCODE_DELIMITED: ::core::ffi::c_uint = 0x2 as ::core::ffi::c_uint;
unsafe extern "C" fn batch_write_callback(
    mut stream: *mut pb_ostream_t,
    mut buffer: *const uint8_t,
    mut count: size_t,
) -> bool {
    let mut connection: krpc_connection_t = (*stream).state as intptr_t
        as krpc_connection_t;
    return krpc_write(connection, buffer, count) as ::core::ffi::c_int
        == KRPC_OK as ::core::ffi::c_int;
}
unsafe extern "C" fn batch_read_callback(
    mut stream: *mut pb_istream_t,
    mut buffer: *mut uint8_t,
    mut count: size_t,
) -> bool {
    let mut connection: krpc_connection_t = (*stream).state as intptr_t
        as krpc_connection_t;
    let mut result: krpc_error_t = krpc_read(connection, buffer, count);
    if result as ::core::ffi::c_int == KRPC_ERROR_EOF as ::core::ffi::c_int {
        (*stream).bytes_left = 0 as size_t;
    }
    return result as ::core::ffi::c_int == KRPC_OK as ::core::ffi::c_int;
}
unsafe extern "C" fn batch_error_decoder(
    mut stream: *mut pb_istream_t,
    mut field: *const pb_field_t,
    mut arg: *mut *mut ::core::ffi::c_void,
) -> bool {
    let mut state: *mut BatchErrorState = *arg as *mut BatchErrorState;
    if !state.is_null() {
        (*state).failed = true_0 != 0;
    }
    let mut scratch: [uint8_t; 64] = [0; 64];
    while (*stream).bytes_left != 0 {
        let mut take: size_t = if (*stream).bytes_left
            < ::core::mem::size_of::<[uint8_t; 64]>() as usize
        {
            (*stream).bytes_left
        } else {
            ::core::mem::size_of::<[uint8_t; 64]>() as size_t
        };
        if !pb_read(stream, &raw mut scratch as *mut pb_byte_t, take) {
            return false_0 != 0;
        }
    }
    return true_0 != 0;
}
#[no_mangle]
pub unsafe extern "C" fn krpc_cnano_invoke_batch(
    mut connection: krpc_connection_t,
    mut calls: *const krpc_schema_ProcedureCall,
    mut results: *mut krpc_result_t,
    mut result_failed: *mut bool,
    mut count: size_t,
) -> krpc_error_t {
    if connection.is_null() || calls.is_null() || results.is_null()
        || count == 0 as size_t || count > KRPC_CNANO_BATCH_MAX as size_t
    {
        return KRPC_ERROR_ENCODING_FAILED;
    }
    let mut ostream: pb_ostream_t = pb_ostream_s {
        callback: None,
        state: ::core::ptr::null_mut::<::core::ffi::c_void>(),
        max_size: 0,
        bytes_written: 0,
        errmsg: ::core::ptr::null::<::core::ffi::c_char>(),
    };
    ostream.callback = Some(
        batch_write_callback
            as unsafe extern "C" fn(*mut pb_ostream_t, *const uint8_t, size_t) -> bool,
    )
        as Option<
            unsafe extern "C" fn(*mut pb_ostream_t, *const pb_byte_t, size_t) -> bool,
        >;
    ostream.state = connection as intptr_t as *mut ::core::ffi::c_void;
    ostream.max_size = SIZE_MAX as size_t;
    if krpc_cnano_transport_uses_standard_rpc(connection) {
        let mut request: krpc_schema_Request = _krpc_schema_Request {
            calls_count: 0 as pb_size_t,
            calls: [
                _krpc_schema_ProcedureCall {
                    arguments: pb_callback_s {
                        funcs: C2RustUnnamed { decode: None },
                        arg: NULL,
                    },
                    service_id: 0 as uint32_t,
                    procedure_id: 0 as uint32_t,
                },
                _krpc_schema_ProcedureCall {
                    arguments: pb_callback_s {
                        funcs: C2RustUnnamed { decode: None },
                        arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                    },
                    service_id: 0,
                    procedure_id: 0,
                },
                _krpc_schema_ProcedureCall {
                    arguments: pb_callback_s {
                        funcs: C2RustUnnamed { decode: None },
                        arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                    },
                    service_id: 0,
                    procedure_id: 0,
                },
                _krpc_schema_ProcedureCall {
                    arguments: pb_callback_s {
                        funcs: C2RustUnnamed { decode: None },
                        arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                    },
                    service_id: 0,
                    procedure_id: 0,
                },
                _krpc_schema_ProcedureCall {
                    arguments: pb_callback_s {
                        funcs: C2RustUnnamed { decode: None },
                        arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                    },
                    service_id: 0,
                    procedure_id: 0,
                },
                _krpc_schema_ProcedureCall {
                    arguments: pb_callback_s {
                        funcs: C2RustUnnamed { decode: None },
                        arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                    },
                    service_id: 0,
                    procedure_id: 0,
                },
                _krpc_schema_ProcedureCall {
                    arguments: pb_callback_s {
                        funcs: C2RustUnnamed { decode: None },
                        arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                    },
                    service_id: 0,
                    procedure_id: 0,
                },
                _krpc_schema_ProcedureCall {
                    arguments: pb_callback_s {
                        funcs: C2RustUnnamed { decode: None },
                        arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                    },
                    service_id: 0,
                    procedure_id: 0,
                },
                _krpc_schema_ProcedureCall {
                    arguments: pb_callback_s {
                        funcs: C2RustUnnamed { decode: None },
                        arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                    },
                    service_id: 0,
                    procedure_id: 0,
                },
                _krpc_schema_ProcedureCall {
                    arguments: pb_callback_s {
                        funcs: C2RustUnnamed { decode: None },
                        arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                    },
                    service_id: 0,
                    procedure_id: 0,
                },
                _krpc_schema_ProcedureCall {
                    arguments: pb_callback_s {
                        funcs: C2RustUnnamed { decode: None },
                        arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                    },
                    service_id: 0,
                    procedure_id: 0,
                },
                _krpc_schema_ProcedureCall {
                    arguments: pb_callback_s {
                        funcs: C2RustUnnamed { decode: None },
                        arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                    },
                    service_id: 0,
                    procedure_id: 0,
                },
                _krpc_schema_ProcedureCall {
                    arguments: pb_callback_s {
                        funcs: C2RustUnnamed { decode: None },
                        arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                    },
                    service_id: 0,
                    procedure_id: 0,
                },
                _krpc_schema_ProcedureCall {
                    arguments: pb_callback_s {
                        funcs: C2RustUnnamed { decode: None },
                        arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                    },
                    service_id: 0,
                    procedure_id: 0,
                },
                _krpc_schema_ProcedureCall {
                    arguments: pb_callback_s {
                        funcs: C2RustUnnamed { decode: None },
                        arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                    },
                    service_id: 0,
                    procedure_id: 0,
                },
                _krpc_schema_ProcedureCall {
                    arguments: pb_callback_s {
                        funcs: C2RustUnnamed { decode: None },
                        arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                    },
                    service_id: 0,
                    procedure_id: 0,
                },
                _krpc_schema_ProcedureCall {
                    arguments: pb_callback_s {
                        funcs: C2RustUnnamed { decode: None },
                        arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                    },
                    service_id: 0,
                    procedure_id: 0,
                },
                _krpc_schema_ProcedureCall {
                    arguments: pb_callback_s {
                        funcs: C2RustUnnamed { decode: None },
                        arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                    },
                    service_id: 0,
                    procedure_id: 0,
                },
                _krpc_schema_ProcedureCall {
                    arguments: pb_callback_s {
                        funcs: C2RustUnnamed { decode: None },
                        arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                    },
                    service_id: 0,
                    procedure_id: 0,
                },
                _krpc_schema_ProcedureCall {
                    arguments: pb_callback_s {
                        funcs: C2RustUnnamed { decode: None },
                        arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                    },
                    service_id: 0,
                    procedure_id: 0,
                },
                _krpc_schema_ProcedureCall {
                    arguments: pb_callback_s {
                        funcs: C2RustUnnamed { decode: None },
                        arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                    },
                    service_id: 0,
                    procedure_id: 0,
                },
                _krpc_schema_ProcedureCall {
                    arguments: pb_callback_s {
                        funcs: C2RustUnnamed { decode: None },
                        arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                    },
                    service_id: 0,
                    procedure_id: 0,
                },
                _krpc_schema_ProcedureCall {
                    arguments: pb_callback_s {
                        funcs: C2RustUnnamed { decode: None },
                        arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                    },
                    service_id: 0,
                    procedure_id: 0,
                },
                _krpc_schema_ProcedureCall {
                    arguments: pb_callback_s {
                        funcs: C2RustUnnamed { decode: None },
                        arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                    },
                    service_id: 0,
                    procedure_id: 0,
                },
                _krpc_schema_ProcedureCall {
                    arguments: pb_callback_s {
                        funcs: C2RustUnnamed { decode: None },
                        arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                    },
                    service_id: 0,
                    procedure_id: 0,
                },
                _krpc_schema_ProcedureCall {
                    arguments: pb_callback_s {
                        funcs: C2RustUnnamed { decode: None },
                        arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                    },
                    service_id: 0,
                    procedure_id: 0,
                },
                _krpc_schema_ProcedureCall {
                    arguments: pb_callback_s {
                        funcs: C2RustUnnamed { decode: None },
                        arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                    },
                    service_id: 0,
                    procedure_id: 0,
                },
                _krpc_schema_ProcedureCall {
                    arguments: pb_callback_s {
                        funcs: C2RustUnnamed { decode: None },
                        arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                    },
                    service_id: 0,
                    procedure_id: 0,
                },
                _krpc_schema_ProcedureCall {
                    arguments: pb_callback_s {
                        funcs: C2RustUnnamed { decode: None },
                        arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                    },
                    service_id: 0,
                    procedure_id: 0,
                },
                _krpc_schema_ProcedureCall {
                    arguments: pb_callback_s {
                        funcs: C2RustUnnamed { decode: None },
                        arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                    },
                    service_id: 0,
                    procedure_id: 0,
                },
                _krpc_schema_ProcedureCall {
                    arguments: pb_callback_s {
                        funcs: C2RustUnnamed { decode: None },
                        arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                    },
                    service_id: 0,
                    procedure_id: 0,
                },
                _krpc_schema_ProcedureCall {
                    arguments: pb_callback_s {
                        funcs: C2RustUnnamed { decode: None },
                        arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                    },
                    service_id: 0,
                    procedure_id: 0,
                },
            ],
        };
        request.calls_count = count as pb_size_t;
        let mut i: size_t = 0 as size_t;
        while i < count {
            request.calls[i as usize] = *calls.offset(i as isize);
            i = i.wrapping_add(1);
        }
        if !pb_encode_ex(
            &raw mut ostream,
            &raw const krpc_schema_Request_msg,
            &raw mut request as *const ::core::ffi::c_void,
            PB_ENCODE_DELIMITED,
        ) {
            return KRPC_ERROR_ENCODING_FAILED;
        }
    } else {
        let mut request_0: krpc_schema_MultiplexedRequest = _krpc_schema_MultiplexedRequest {
            has_connection_request: false_0 != 0,
            connection_request: _krpc_schema_ConnectionRequest {
                type_0: krpc_schema_ConnectionRequest_Type_RPC,
                client_name: pb_callback_s {
                    funcs: C2RustUnnamed { decode: None },
                    arg: NULL,
                },
                client_identifier: pb_callback_s {
                    funcs: C2RustUnnamed { decode: None },
                    arg: NULL,
                },
            },
            has_request: false_0 != 0,
            request: _krpc_schema_Request {
                calls_count: 0 as pb_size_t,
                calls: [
                    _krpc_schema_ProcedureCall {
                        arguments: pb_callback_s {
                            funcs: C2RustUnnamed { decode: None },
                            arg: NULL,
                        },
                        service_id: 0 as uint32_t,
                        procedure_id: 0 as uint32_t,
                    },
                    _krpc_schema_ProcedureCall {
                        arguments: pb_callback_s {
                            funcs: C2RustUnnamed { decode: None },
                            arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                        },
                        service_id: 0,
                        procedure_id: 0,
                    },
                    _krpc_schema_ProcedureCall {
                        arguments: pb_callback_s {
                            funcs: C2RustUnnamed { decode: None },
                            arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                        },
                        service_id: 0,
                        procedure_id: 0,
                    },
                    _krpc_schema_ProcedureCall {
                        arguments: pb_callback_s {
                            funcs: C2RustUnnamed { decode: None },
                            arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                        },
                        service_id: 0,
                        procedure_id: 0,
                    },
                    _krpc_schema_ProcedureCall {
                        arguments: pb_callback_s {
                            funcs: C2RustUnnamed { decode: None },
                            arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                        },
                        service_id: 0,
                        procedure_id: 0,
                    },
                    _krpc_schema_ProcedureCall {
                        arguments: pb_callback_s {
                            funcs: C2RustUnnamed { decode: None },
                            arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                        },
                        service_id: 0,
                        procedure_id: 0,
                    },
                    _krpc_schema_ProcedureCall {
                        arguments: pb_callback_s {
                            funcs: C2RustUnnamed { decode: None },
                            arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                        },
                        service_id: 0,
                        procedure_id: 0,
                    },
                    _krpc_schema_ProcedureCall {
                        arguments: pb_callback_s {
                            funcs: C2RustUnnamed { decode: None },
                            arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                        },
                        service_id: 0,
                        procedure_id: 0,
                    },
                    _krpc_schema_ProcedureCall {
                        arguments: pb_callback_s {
                            funcs: C2RustUnnamed { decode: None },
                            arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                        },
                        service_id: 0,
                        procedure_id: 0,
                    },
                    _krpc_schema_ProcedureCall {
                        arguments: pb_callback_s {
                            funcs: C2RustUnnamed { decode: None },
                            arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                        },
                        service_id: 0,
                        procedure_id: 0,
                    },
                    _krpc_schema_ProcedureCall {
                        arguments: pb_callback_s {
                            funcs: C2RustUnnamed { decode: None },
                            arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                        },
                        service_id: 0,
                        procedure_id: 0,
                    },
                    _krpc_schema_ProcedureCall {
                        arguments: pb_callback_s {
                            funcs: C2RustUnnamed { decode: None },
                            arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                        },
                        service_id: 0,
                        procedure_id: 0,
                    },
                    _krpc_schema_ProcedureCall {
                        arguments: pb_callback_s {
                            funcs: C2RustUnnamed { decode: None },
                            arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                        },
                        service_id: 0,
                        procedure_id: 0,
                    },
                    _krpc_schema_ProcedureCall {
                        arguments: pb_callback_s {
                            funcs: C2RustUnnamed { decode: None },
                            arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                        },
                        service_id: 0,
                        procedure_id: 0,
                    },
                    _krpc_schema_ProcedureCall {
                        arguments: pb_callback_s {
                            funcs: C2RustUnnamed { decode: None },
                            arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                        },
                        service_id: 0,
                        procedure_id: 0,
                    },
                    _krpc_schema_ProcedureCall {
                        arguments: pb_callback_s {
                            funcs: C2RustUnnamed { decode: None },
                            arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                        },
                        service_id: 0,
                        procedure_id: 0,
                    },
                    _krpc_schema_ProcedureCall {
                        arguments: pb_callback_s {
                            funcs: C2RustUnnamed { decode: None },
                            arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                        },
                        service_id: 0,
                        procedure_id: 0,
                    },
                    _krpc_schema_ProcedureCall {
                        arguments: pb_callback_s {
                            funcs: C2RustUnnamed { decode: None },
                            arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                        },
                        service_id: 0,
                        procedure_id: 0,
                    },
                    _krpc_schema_ProcedureCall {
                        arguments: pb_callback_s {
                            funcs: C2RustUnnamed { decode: None },
                            arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                        },
                        service_id: 0,
                        procedure_id: 0,
                    },
                    _krpc_schema_ProcedureCall {
                        arguments: pb_callback_s {
                            funcs: C2RustUnnamed { decode: None },
                            arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                        },
                        service_id: 0,
                        procedure_id: 0,
                    },
                    _krpc_schema_ProcedureCall {
                        arguments: pb_callback_s {
                            funcs: C2RustUnnamed { decode: None },
                            arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                        },
                        service_id: 0,
                        procedure_id: 0,
                    },
                    _krpc_schema_ProcedureCall {
                        arguments: pb_callback_s {
                            funcs: C2RustUnnamed { decode: None },
                            arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                        },
                        service_id: 0,
                        procedure_id: 0,
                    },
                    _krpc_schema_ProcedureCall {
                        arguments: pb_callback_s {
                            funcs: C2RustUnnamed { decode: None },
                            arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                        },
                        service_id: 0,
                        procedure_id: 0,
                    },
                    _krpc_schema_ProcedureCall {
                        arguments: pb_callback_s {
                            funcs: C2RustUnnamed { decode: None },
                            arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                        },
                        service_id: 0,
                        procedure_id: 0,
                    },
                    _krpc_schema_ProcedureCall {
                        arguments: pb_callback_s {
                            funcs: C2RustUnnamed { decode: None },
                            arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                        },
                        service_id: 0,
                        procedure_id: 0,
                    },
                    _krpc_schema_ProcedureCall {
                        arguments: pb_callback_s {
                            funcs: C2RustUnnamed { decode: None },
                            arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                        },
                        service_id: 0,
                        procedure_id: 0,
                    },
                    _krpc_schema_ProcedureCall {
                        arguments: pb_callback_s {
                            funcs: C2RustUnnamed { decode: None },
                            arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                        },
                        service_id: 0,
                        procedure_id: 0,
                    },
                    _krpc_schema_ProcedureCall {
                        arguments: pb_callback_s {
                            funcs: C2RustUnnamed { decode: None },
                            arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                        },
                        service_id: 0,
                        procedure_id: 0,
                    },
                    _krpc_schema_ProcedureCall {
                        arguments: pb_callback_s {
                            funcs: C2RustUnnamed { decode: None },
                            arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                        },
                        service_id: 0,
                        procedure_id: 0,
                    },
                    _krpc_schema_ProcedureCall {
                        arguments: pb_callback_s {
                            funcs: C2RustUnnamed { decode: None },
                            arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                        },
                        service_id: 0,
                        procedure_id: 0,
                    },
                    _krpc_schema_ProcedureCall {
                        arguments: pb_callback_s {
                            funcs: C2RustUnnamed { decode: None },
                            arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                        },
                        service_id: 0,
                        procedure_id: 0,
                    },
                    _krpc_schema_ProcedureCall {
                        arguments: pb_callback_s {
                            funcs: C2RustUnnamed { decode: None },
                            arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                        },
                        service_id: 0,
                        procedure_id: 0,
                    },
                ],
            },
        };
        request_0.has_request = true_0 != 0;
        request_0.request.calls_count = count as pb_size_t;
        let mut i_0: size_t = 0 as size_t;
        while i_0 < count {
            request_0.request.calls[i_0 as usize] = *calls.offset(i_0 as isize);
            i_0 = i_0.wrapping_add(1);
        }
        if !pb_encode_ex(
            &raw mut ostream,
            &raw const krpc_schema_MultiplexedRequest_msg,
            &raw mut request_0 as *const ::core::ffi::c_void,
            PB_ENCODE_DELIMITED,
        ) {
            return KRPC_ERROR_ENCODING_FAILED;
        }
    }
    if !krpc_cnano_transport_begin_deadline(connection) {
        return KRPC_ERROR_IO;
    }
    loop {
        let mut rpc_response: krpc_schema_Response = _krpc_schema_Response {
            error: pb_callback_s {
                funcs: C2RustUnnamed { decode: None },
                arg: NULL,
            },
            results_count: 0 as pb_size_t,
            results: [
                _krpc_schema_ProcedureResult {
                    error: pb_callback_s {
                        funcs: C2RustUnnamed { decode: None },
                        arg: NULL,
                    },
                    value: pb_callback_s {
                        funcs: C2RustUnnamed { decode: None },
                        arg: NULL,
                    },
                },
                _krpc_schema_ProcedureResult {
                    error: pb_callback_s {
                        funcs: C2RustUnnamed { decode: None },
                        arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                    },
                    value: pb_callback_s {
                        funcs: C2RustUnnamed { decode: None },
                        arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                    },
                },
                _krpc_schema_ProcedureResult {
                    error: pb_callback_s {
                        funcs: C2RustUnnamed { decode: None },
                        arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                    },
                    value: pb_callback_s {
                        funcs: C2RustUnnamed { decode: None },
                        arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                    },
                },
                _krpc_schema_ProcedureResult {
                    error: pb_callback_s {
                        funcs: C2RustUnnamed { decode: None },
                        arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                    },
                    value: pb_callback_s {
                        funcs: C2RustUnnamed { decode: None },
                        arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                    },
                },
                _krpc_schema_ProcedureResult {
                    error: pb_callback_s {
                        funcs: C2RustUnnamed { decode: None },
                        arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                    },
                    value: pb_callback_s {
                        funcs: C2RustUnnamed { decode: None },
                        arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                    },
                },
                _krpc_schema_ProcedureResult {
                    error: pb_callback_s {
                        funcs: C2RustUnnamed { decode: None },
                        arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                    },
                    value: pb_callback_s {
                        funcs: C2RustUnnamed { decode: None },
                        arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                    },
                },
                _krpc_schema_ProcedureResult {
                    error: pb_callback_s {
                        funcs: C2RustUnnamed { decode: None },
                        arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                    },
                    value: pb_callback_s {
                        funcs: C2RustUnnamed { decode: None },
                        arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                    },
                },
                _krpc_schema_ProcedureResult {
                    error: pb_callback_s {
                        funcs: C2RustUnnamed { decode: None },
                        arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                    },
                    value: pb_callback_s {
                        funcs: C2RustUnnamed { decode: None },
                        arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                    },
                },
                _krpc_schema_ProcedureResult {
                    error: pb_callback_s {
                        funcs: C2RustUnnamed { decode: None },
                        arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                    },
                    value: pb_callback_s {
                        funcs: C2RustUnnamed { decode: None },
                        arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                    },
                },
                _krpc_schema_ProcedureResult {
                    error: pb_callback_s {
                        funcs: C2RustUnnamed { decode: None },
                        arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                    },
                    value: pb_callback_s {
                        funcs: C2RustUnnamed { decode: None },
                        arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                    },
                },
                _krpc_schema_ProcedureResult {
                    error: pb_callback_s {
                        funcs: C2RustUnnamed { decode: None },
                        arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                    },
                    value: pb_callback_s {
                        funcs: C2RustUnnamed { decode: None },
                        arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                    },
                },
                _krpc_schema_ProcedureResult {
                    error: pb_callback_s {
                        funcs: C2RustUnnamed { decode: None },
                        arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                    },
                    value: pb_callback_s {
                        funcs: C2RustUnnamed { decode: None },
                        arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                    },
                },
                _krpc_schema_ProcedureResult {
                    error: pb_callback_s {
                        funcs: C2RustUnnamed { decode: None },
                        arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                    },
                    value: pb_callback_s {
                        funcs: C2RustUnnamed { decode: None },
                        arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                    },
                },
                _krpc_schema_ProcedureResult {
                    error: pb_callback_s {
                        funcs: C2RustUnnamed { decode: None },
                        arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                    },
                    value: pb_callback_s {
                        funcs: C2RustUnnamed { decode: None },
                        arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                    },
                },
                _krpc_schema_ProcedureResult {
                    error: pb_callback_s {
                        funcs: C2RustUnnamed { decode: None },
                        arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                    },
                    value: pb_callback_s {
                        funcs: C2RustUnnamed { decode: None },
                        arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                    },
                },
                _krpc_schema_ProcedureResult {
                    error: pb_callback_s {
                        funcs: C2RustUnnamed { decode: None },
                        arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                    },
                    value: pb_callback_s {
                        funcs: C2RustUnnamed { decode: None },
                        arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                    },
                },
                _krpc_schema_ProcedureResult {
                    error: pb_callback_s {
                        funcs: C2RustUnnamed { decode: None },
                        arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                    },
                    value: pb_callback_s {
                        funcs: C2RustUnnamed { decode: None },
                        arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                    },
                },
                _krpc_schema_ProcedureResult {
                    error: pb_callback_s {
                        funcs: C2RustUnnamed { decode: None },
                        arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                    },
                    value: pb_callback_s {
                        funcs: C2RustUnnamed { decode: None },
                        arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                    },
                },
                _krpc_schema_ProcedureResult {
                    error: pb_callback_s {
                        funcs: C2RustUnnamed { decode: None },
                        arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                    },
                    value: pb_callback_s {
                        funcs: C2RustUnnamed { decode: None },
                        arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                    },
                },
                _krpc_schema_ProcedureResult {
                    error: pb_callback_s {
                        funcs: C2RustUnnamed { decode: None },
                        arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                    },
                    value: pb_callback_s {
                        funcs: C2RustUnnamed { decode: None },
                        arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                    },
                },
                _krpc_schema_ProcedureResult {
                    error: pb_callback_s {
                        funcs: C2RustUnnamed { decode: None },
                        arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                    },
                    value: pb_callback_s {
                        funcs: C2RustUnnamed { decode: None },
                        arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                    },
                },
                _krpc_schema_ProcedureResult {
                    error: pb_callback_s {
                        funcs: C2RustUnnamed { decode: None },
                        arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                    },
                    value: pb_callback_s {
                        funcs: C2RustUnnamed { decode: None },
                        arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                    },
                },
                _krpc_schema_ProcedureResult {
                    error: pb_callback_s {
                        funcs: C2RustUnnamed { decode: None },
                        arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                    },
                    value: pb_callback_s {
                        funcs: C2RustUnnamed { decode: None },
                        arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                    },
                },
                _krpc_schema_ProcedureResult {
                    error: pb_callback_s {
                        funcs: C2RustUnnamed { decode: None },
                        arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                    },
                    value: pb_callback_s {
                        funcs: C2RustUnnamed { decode: None },
                        arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                    },
                },
                _krpc_schema_ProcedureResult {
                    error: pb_callback_s {
                        funcs: C2RustUnnamed { decode: None },
                        arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                    },
                    value: pb_callback_s {
                        funcs: C2RustUnnamed { decode: None },
                        arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                    },
                },
                _krpc_schema_ProcedureResult {
                    error: pb_callback_s {
                        funcs: C2RustUnnamed { decode: None },
                        arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                    },
                    value: pb_callback_s {
                        funcs: C2RustUnnamed { decode: None },
                        arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                    },
                },
                _krpc_schema_ProcedureResult {
                    error: pb_callback_s {
                        funcs: C2RustUnnamed { decode: None },
                        arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                    },
                    value: pb_callback_s {
                        funcs: C2RustUnnamed { decode: None },
                        arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                    },
                },
                _krpc_schema_ProcedureResult {
                    error: pb_callback_s {
                        funcs: C2RustUnnamed { decode: None },
                        arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                    },
                    value: pb_callback_s {
                        funcs: C2RustUnnamed { decode: None },
                        arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                    },
                },
                _krpc_schema_ProcedureResult {
                    error: pb_callback_s {
                        funcs: C2RustUnnamed { decode: None },
                        arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                    },
                    value: pb_callback_s {
                        funcs: C2RustUnnamed { decode: None },
                        arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                    },
                },
                _krpc_schema_ProcedureResult {
                    error: pb_callback_s {
                        funcs: C2RustUnnamed { decode: None },
                        arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                    },
                    value: pb_callback_s {
                        funcs: C2RustUnnamed { decode: None },
                        arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                    },
                },
                _krpc_schema_ProcedureResult {
                    error: pb_callback_s {
                        funcs: C2RustUnnamed { decode: None },
                        arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                    },
                    value: pb_callback_s {
                        funcs: C2RustUnnamed { decode: None },
                        arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                    },
                },
                _krpc_schema_ProcedureResult {
                    error: pb_callback_s {
                        funcs: C2RustUnnamed { decode: None },
                        arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                    },
                    value: pb_callback_s {
                        funcs: C2RustUnnamed { decode: None },
                        arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                    },
                },
            ],
        };
        let mut response_error: BatchErrorState = BatchErrorState {
            failed: 0 as ::core::ffi::c_int != 0,
        };
        let mut item_errors: [BatchErrorState; 32] = [BatchErrorState {
            failed: false,
        }; 32];
        memset(
            &raw mut item_errors as *mut BatchErrorState as *mut ::core::ffi::c_void,
            0 as ::core::ffi::c_int,
            ::core::mem::size_of::<[BatchErrorState; 32]>() as size_t,
        );
        rpc_response.error.funcs.decode = Some(
            batch_error_decoder
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
        rpc_response.error.arg = &raw mut response_error as *mut ::core::ffi::c_void;
        let mut i_1: size_t = 0 as size_t;
        while i_1 < count {
            rpc_response.results[i_1 as usize] = (*results.offset(i_1 as isize)).message;
            rpc_response.results[i_1 as usize].error.funcs.decode = Some(
                batch_error_decoder
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
            rpc_response.results[i_1 as usize].error.arg = (&raw mut item_errors
                as *mut BatchErrorState)
                .offset(i_1 as isize) as *mut BatchErrorState
                as *mut ::core::ffi::c_void;
            i_1 = i_1.wrapping_add(1);
        }
        let mut istream: pb_istream_t = pb_istream_s {
            callback: None,
            state: ::core::ptr::null_mut::<::core::ffi::c_void>(),
            bytes_left: 0,
            errmsg: ::core::ptr::null::<::core::ffi::c_char>(),
        };
        istream.callback = Some(
            batch_read_callback
                as unsafe extern "C" fn(*mut pb_istream_t, *mut uint8_t, size_t) -> bool,
        )
            as Option<
                unsafe extern "C" fn(*mut pb_istream_t, *mut pb_byte_t, size_t) -> bool,
            >;
        istream.state = connection as intptr_t as *mut ::core::ffi::c_void;
        istream.bytes_left = SIZE_MAX as size_t;
        if krpc_cnano_transport_uses_standard_rpc(connection) {
            if !pb_decode_ex(
                &raw mut istream,
                &raw const krpc_schema_Response_msg,
                &raw mut rpc_response as *mut ::core::ffi::c_void,
                PB_DECODE_DELIMITED,
            ) {
                krpc_cnano_transport_end_deadline(connection);
                return KRPC_ERROR_DECODING_FAILED;
            }
        } else {
            let mut response: krpc_schema_MultiplexedResponse = _krpc_schema_MultiplexedResponse {
                has_response: false_0 != 0,
                response: _krpc_schema_Response {
                    error: pb_callback_s {
                        funcs: C2RustUnnamed { decode: None },
                        arg: NULL,
                    },
                    results_count: 0 as pb_size_t,
                    results: [
                        _krpc_schema_ProcedureResult {
                            error: pb_callback_s {
                                funcs: C2RustUnnamed { decode: None },
                                arg: NULL,
                            },
                            value: pb_callback_s {
                                funcs: C2RustUnnamed { decode: None },
                                arg: NULL,
                            },
                        },
                        _krpc_schema_ProcedureResult {
                            error: pb_callback_s {
                                funcs: C2RustUnnamed { decode: None },
                                arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                            },
                            value: pb_callback_s {
                                funcs: C2RustUnnamed { decode: None },
                                arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                            },
                        },
                        _krpc_schema_ProcedureResult {
                            error: pb_callback_s {
                                funcs: C2RustUnnamed { decode: None },
                                arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                            },
                            value: pb_callback_s {
                                funcs: C2RustUnnamed { decode: None },
                                arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                            },
                        },
                        _krpc_schema_ProcedureResult {
                            error: pb_callback_s {
                                funcs: C2RustUnnamed { decode: None },
                                arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                            },
                            value: pb_callback_s {
                                funcs: C2RustUnnamed { decode: None },
                                arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                            },
                        },
                        _krpc_schema_ProcedureResult {
                            error: pb_callback_s {
                                funcs: C2RustUnnamed { decode: None },
                                arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                            },
                            value: pb_callback_s {
                                funcs: C2RustUnnamed { decode: None },
                                arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                            },
                        },
                        _krpc_schema_ProcedureResult {
                            error: pb_callback_s {
                                funcs: C2RustUnnamed { decode: None },
                                arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                            },
                            value: pb_callback_s {
                                funcs: C2RustUnnamed { decode: None },
                                arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                            },
                        },
                        _krpc_schema_ProcedureResult {
                            error: pb_callback_s {
                                funcs: C2RustUnnamed { decode: None },
                                arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                            },
                            value: pb_callback_s {
                                funcs: C2RustUnnamed { decode: None },
                                arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                            },
                        },
                        _krpc_schema_ProcedureResult {
                            error: pb_callback_s {
                                funcs: C2RustUnnamed { decode: None },
                                arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                            },
                            value: pb_callback_s {
                                funcs: C2RustUnnamed { decode: None },
                                arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                            },
                        },
                        _krpc_schema_ProcedureResult {
                            error: pb_callback_s {
                                funcs: C2RustUnnamed { decode: None },
                                arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                            },
                            value: pb_callback_s {
                                funcs: C2RustUnnamed { decode: None },
                                arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                            },
                        },
                        _krpc_schema_ProcedureResult {
                            error: pb_callback_s {
                                funcs: C2RustUnnamed { decode: None },
                                arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                            },
                            value: pb_callback_s {
                                funcs: C2RustUnnamed { decode: None },
                                arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                            },
                        },
                        _krpc_schema_ProcedureResult {
                            error: pb_callback_s {
                                funcs: C2RustUnnamed { decode: None },
                                arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                            },
                            value: pb_callback_s {
                                funcs: C2RustUnnamed { decode: None },
                                arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                            },
                        },
                        _krpc_schema_ProcedureResult {
                            error: pb_callback_s {
                                funcs: C2RustUnnamed { decode: None },
                                arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                            },
                            value: pb_callback_s {
                                funcs: C2RustUnnamed { decode: None },
                                arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                            },
                        },
                        _krpc_schema_ProcedureResult {
                            error: pb_callback_s {
                                funcs: C2RustUnnamed { decode: None },
                                arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                            },
                            value: pb_callback_s {
                                funcs: C2RustUnnamed { decode: None },
                                arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                            },
                        },
                        _krpc_schema_ProcedureResult {
                            error: pb_callback_s {
                                funcs: C2RustUnnamed { decode: None },
                                arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                            },
                            value: pb_callback_s {
                                funcs: C2RustUnnamed { decode: None },
                                arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                            },
                        },
                        _krpc_schema_ProcedureResult {
                            error: pb_callback_s {
                                funcs: C2RustUnnamed { decode: None },
                                arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                            },
                            value: pb_callback_s {
                                funcs: C2RustUnnamed { decode: None },
                                arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                            },
                        },
                        _krpc_schema_ProcedureResult {
                            error: pb_callback_s {
                                funcs: C2RustUnnamed { decode: None },
                                arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                            },
                            value: pb_callback_s {
                                funcs: C2RustUnnamed { decode: None },
                                arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                            },
                        },
                        _krpc_schema_ProcedureResult {
                            error: pb_callback_s {
                                funcs: C2RustUnnamed { decode: None },
                                arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                            },
                            value: pb_callback_s {
                                funcs: C2RustUnnamed { decode: None },
                                arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                            },
                        },
                        _krpc_schema_ProcedureResult {
                            error: pb_callback_s {
                                funcs: C2RustUnnamed { decode: None },
                                arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                            },
                            value: pb_callback_s {
                                funcs: C2RustUnnamed { decode: None },
                                arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                            },
                        },
                        _krpc_schema_ProcedureResult {
                            error: pb_callback_s {
                                funcs: C2RustUnnamed { decode: None },
                                arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                            },
                            value: pb_callback_s {
                                funcs: C2RustUnnamed { decode: None },
                                arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                            },
                        },
                        _krpc_schema_ProcedureResult {
                            error: pb_callback_s {
                                funcs: C2RustUnnamed { decode: None },
                                arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                            },
                            value: pb_callback_s {
                                funcs: C2RustUnnamed { decode: None },
                                arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                            },
                        },
                        _krpc_schema_ProcedureResult {
                            error: pb_callback_s {
                                funcs: C2RustUnnamed { decode: None },
                                arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                            },
                            value: pb_callback_s {
                                funcs: C2RustUnnamed { decode: None },
                                arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                            },
                        },
                        _krpc_schema_ProcedureResult {
                            error: pb_callback_s {
                                funcs: C2RustUnnamed { decode: None },
                                arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                            },
                            value: pb_callback_s {
                                funcs: C2RustUnnamed { decode: None },
                                arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                            },
                        },
                        _krpc_schema_ProcedureResult {
                            error: pb_callback_s {
                                funcs: C2RustUnnamed { decode: None },
                                arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                            },
                            value: pb_callback_s {
                                funcs: C2RustUnnamed { decode: None },
                                arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                            },
                        },
                        _krpc_schema_ProcedureResult {
                            error: pb_callback_s {
                                funcs: C2RustUnnamed { decode: None },
                                arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                            },
                            value: pb_callback_s {
                                funcs: C2RustUnnamed { decode: None },
                                arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                            },
                        },
                        _krpc_schema_ProcedureResult {
                            error: pb_callback_s {
                                funcs: C2RustUnnamed { decode: None },
                                arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                            },
                            value: pb_callback_s {
                                funcs: C2RustUnnamed { decode: None },
                                arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                            },
                        },
                        _krpc_schema_ProcedureResult {
                            error: pb_callback_s {
                                funcs: C2RustUnnamed { decode: None },
                                arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                            },
                            value: pb_callback_s {
                                funcs: C2RustUnnamed { decode: None },
                                arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                            },
                        },
                        _krpc_schema_ProcedureResult {
                            error: pb_callback_s {
                                funcs: C2RustUnnamed { decode: None },
                                arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                            },
                            value: pb_callback_s {
                                funcs: C2RustUnnamed { decode: None },
                                arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                            },
                        },
                        _krpc_schema_ProcedureResult {
                            error: pb_callback_s {
                                funcs: C2RustUnnamed { decode: None },
                                arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                            },
                            value: pb_callback_s {
                                funcs: C2RustUnnamed { decode: None },
                                arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                            },
                        },
                        _krpc_schema_ProcedureResult {
                            error: pb_callback_s {
                                funcs: C2RustUnnamed { decode: None },
                                arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                            },
                            value: pb_callback_s {
                                funcs: C2RustUnnamed { decode: None },
                                arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                            },
                        },
                        _krpc_schema_ProcedureResult {
                            error: pb_callback_s {
                                funcs: C2RustUnnamed { decode: None },
                                arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                            },
                            value: pb_callback_s {
                                funcs: C2RustUnnamed { decode: None },
                                arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                            },
                        },
                        _krpc_schema_ProcedureResult {
                            error: pb_callback_s {
                                funcs: C2RustUnnamed { decode: None },
                                arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                            },
                            value: pb_callback_s {
                                funcs: C2RustUnnamed { decode: None },
                                arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                            },
                        },
                        _krpc_schema_ProcedureResult {
                            error: pb_callback_s {
                                funcs: C2RustUnnamed { decode: None },
                                arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                            },
                            value: pb_callback_s {
                                funcs: C2RustUnnamed { decode: None },
                                arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                            },
                        },
                    ],
                },
                has_stream_update: false_0 != 0,
                stream_update: _krpc_schema_StreamUpdate {
                    results: pb_callback_s {
                        funcs: C2RustUnnamed { decode: None },
                        arg: NULL,
                    },
                },
            };
            response.response = rpc_response;
            if !pb_decode_ex(
                &raw mut istream,
                &raw const krpc_schema_MultiplexedResponse_msg,
                &raw mut response as *mut ::core::ffi::c_void,
                PB_DECODE_DELIMITED,
            ) {
                krpc_cnano_transport_end_deadline(connection);
                return KRPC_ERROR_DECODING_FAILED;
            }
            if !response.has_response {
                if response.has_stream_update {
                    continue;
                }
                krpc_cnano_transport_end_deadline(connection);
                return KRPC_ERROR_NO_RESULTS;
            } else {
                rpc_response = response.response;
            }
        }
        if response_error.failed {
            krpc_cnano_transport_end_deadline(connection);
            return KRPC_ERROR_RPC_FAILED;
        }
        if rpc_response.results_count as ::core::ffi::c_int == 0 as ::core::ffi::c_int {
            if krpc_cnano_transport_uses_standard_rpc(connection) {
                krpc_cnano_transport_end_deadline(connection);
                return KRPC_ERROR_NO_RESULTS;
            }
        } else {
            if rpc_response.results_count as ::core::ffi::c_int
                != count as pb_size_t as ::core::ffi::c_int
            {
                krpc_cnano_transport_end_deadline(connection);
                return KRPC_ERROR_NO_RESULTS;
            }
            let mut i_2: size_t = 0 as size_t;
            while i_2 < count {
                (*results.offset(i_2 as isize)).message = rpc_response
                    .results[i_2 as usize];
                if !result_failed.is_null() {
                    *result_failed.offset(i_2 as isize) = item_errors[i_2 as usize]
                        .failed;
                }
                i_2 = i_2.wrapping_add(1);
            }
            krpc_cnano_transport_end_deadline(connection);
            return KRPC_OK;
        }
    };
}
