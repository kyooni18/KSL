use crate::sys::*;
extern "C" {
    static krpc_schema_ConnectionRequest_msg: pb_msgdesc_t;
    static krpc_schema_ConnectionResponse_msg: pb_msgdesc_t;
    static krpc_schema_Request_msg: pb_msgdesc_t;
    static krpc_schema_Response_msg: pb_msgdesc_t;
    static krpc_schema_Error_msg: pb_msgdesc_t;
    static krpc_schema_MultiplexedRequest_msg: pb_msgdesc_t;
    static krpc_schema_MultiplexedResponse_msg: pb_msgdesc_t;
    fn krpc_encode_callback_cstring(
        stream: *mut pb_ostream_t,
        field: *const pb_field_t,
        arg: *const *mut ::core::ffi::c_void,
    ) -> bool;
    fn pb_decode(
        stream: *mut pb_istream_t,
        fields: *const pb_msgdesc_t,
        dest_struct: *mut ::core::ffi::c_void,
    ) -> bool;
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
    fn krpc_close(connection: krpc_connection_t) -> krpc_error_t;
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
pub type _krpc_schema_ConnectionResponse_Status = ::core::ffi::c_uint;
pub const krpc_schema_ConnectionResponse_Status_WRONG_TYPE: _krpc_schema_ConnectionResponse_Status = 3;
pub const krpc_schema_ConnectionResponse_Status_TIMEOUT: _krpc_schema_ConnectionResponse_Status = 2;
pub const krpc_schema_ConnectionResponse_Status_MALFORMED_MESSAGE: _krpc_schema_ConnectionResponse_Status = 1;
pub const krpc_schema_ConnectionResponse_Status_OK: _krpc_schema_ConnectionResponse_Status = 0;
pub type krpc_schema_ConnectionResponse_Status = _krpc_schema_ConnectionResponse_Status;
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
pub const PB_DECODE_DELIMITED: ::core::ffi::c_uint = 0x2 as ::core::ffi::c_uint;
pub const PB_ENCODE_DELIMITED: ::core::ffi::c_uint = 0x2 as ::core::ffi::c_uint;
#[no_mangle]
pub unsafe extern "C" fn krpc_connect(
    mut connection: krpc_connection_t,
    mut client_name: *const ::core::ffi::c_char,
) -> krpc_error_t {
    let mut stream: pb_ostream_t = krpc_pb_ostream_from_connection(connection);
    if krpc_cnano_transport_uses_standard_rpc(connection) {
        let mut request: krpc_schema_ConnectionRequest = _krpc_schema_ConnectionRequest {
            type_0: krpc_schema_ConnectionRequest_Type_RPC,
            client_name: pb_callback_s {
                funcs: C2RustUnnamed { decode: None },
                arg: NULL,
            },
            client_identifier: pb_callback_s {
                funcs: C2RustUnnamed { decode: None },
                arg: NULL,
            },
        };
        request.type_0 = krpc_schema_ConnectionRequest_Type_RPC;
        request.client_name.funcs.encode = Some(
            krpc_encode_callback_cstring
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
        request.client_name.arg = client_name as *mut ::core::ffi::c_void;
        if !pb_encode_ex(
            &raw mut stream,
            &raw const krpc_schema_ConnectionRequest_msg,
            &raw mut request as *const ::core::ffi::c_void,
            PB_ENCODE_DELIMITED,
        ) {
            krpc_close(connection);
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
        request_0.has_connection_request = true_0 != 0;
        request_0.connection_request.type_0 = krpc_schema_ConnectionRequest_Type_RPC;
        request_0.connection_request.client_name.funcs.encode = Some(
            krpc_encode_callback_cstring
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
        request_0.connection_request.client_name.arg = client_name
            as *mut ::core::ffi::c_void;
        if !pb_encode_ex(
            &raw mut stream,
            &raw const krpc_schema_MultiplexedRequest_msg,
            &raw mut request_0 as *const ::core::ffi::c_void,
            PB_ENCODE_DELIMITED,
        ) {
            krpc_close(connection);
            return KRPC_ERROR_ENCODING_FAILED;
        }
    }
    let mut stream_0: pb_istream_t = krpc_pb_istream_from_connection(connection);
    let mut response: krpc_schema_ConnectionResponse = _krpc_schema_ConnectionResponse {
        status: krpc_schema_ConnectionResponse_Status_OK,
        message: pb_callback_s {
            funcs: C2RustUnnamed { decode: None },
            arg: NULL,
        },
        client_identifier: pb_callback_s {
            funcs: C2RustUnnamed { decode: None },
            arg: NULL,
        },
    };
    if !pb_decode_ex(
        &raw mut stream_0,
        &raw const krpc_schema_ConnectionResponse_msg,
        &raw mut response as *mut ::core::ffi::c_void,
        PB_DECODE_DELIMITED,
    ) {
        krpc_close(connection);
        return KRPC_ERROR_DECODING_FAILED;
    }
    if response.status as ::core::ffi::c_uint
        != krpc_schema_ConnectionResponse_Status_OK as ::core::ffi::c_int
            as ::core::ffi::c_uint
    {
        krpc_close(connection);
        return KRPC_ERROR_CONNECTION_FAILED;
    }
    return KRPC_OK;
}
static mut krpc_error_message: [::core::ffi::c_char; 512] = [0; 512];
#[no_mangle]
pub unsafe extern "C" fn krpc_get_error_message() -> *const ::core::ffi::c_char {
    return &raw mut krpc_error_message as *mut ::core::ffi::c_char;
}
unsafe extern "C" fn krpc_append_error_message(
    mut str: *const ::core::ffi::c_char,
    mut length: size_t,
) {
    let mut used: size_t = strlen(
        &raw mut krpc_error_message as *mut ::core::ffi::c_char,
    );
    let mut space: size_t = (KRPC_ERROR_MESSAGE_LENGTH as size_t)
        .wrapping_sub(used)
        .wrapping_sub(1 as size_t);
    if length > space {
        length = space;
    }
    memcpy(
        (&raw mut krpc_error_message as *mut ::core::ffi::c_char).offset(used as isize)
            as *mut ::core::ffi::c_void,
        str as *const ::core::ffi::c_void,
        length,
    );
    krpc_error_message[used.wrapping_add(length) as usize] = '\0' as i32
        as ::core::ffi::c_char;
}
unsafe extern "C" fn krpc_decode_callback_error_string(
    mut stream: *mut pb_istream_t,
    mut field: *const pb_field_t,
    mut arg: *mut *mut ::core::ffi::c_void,
) -> bool {
    if (*stream).bytes_left == 0 as size_t {
        return true_0 != 0;
    }
    if krpc_error_message[0 as ::core::ffi::c_int as usize] as ::core::ffi::c_int
        != '\0' as i32
    {
        let mut separator: *const ::core::ffi::c_char = *arg
            as *const ::core::ffi::c_char;
        krpc_append_error_message(separator, strlen(separator));
    }
    let mut chunk: [::core::ffi::c_char; 32] = [0; 32];
    while (*stream).bytes_left > 0 as size_t {
        let mut size: size_t = if (*stream).bytes_left
            < ::core::mem::size_of::<[::core::ffi::c_char; 32]>() as usize
        {
            (*stream).bytes_left
        } else {
            ::core::mem::size_of::<[::core::ffi::c_char; 32]>() as size_t
        };
        if !pb_read(
            stream,
            &raw mut chunk as *mut ::core::ffi::c_char as *mut pb_byte_t,
            size,
        ) {
            return false_0 != 0;
        }
        krpc_append_error_message(&raw mut chunk as *mut ::core::ffi::c_char, size);
    }
    return true_0 != 0;
}
unsafe extern "C" fn krpc_decode_callback_error(
    mut stream: *mut pb_istream_t,
    mut field: *const pb_field_t,
    mut arg: *mut *mut ::core::ffi::c_void,
) -> bool {
    let mut error_code: *mut krpc_error_t = *arg as *mut krpc_error_t;
    *error_code = KRPC_ERROR_RPC_FAILED;
    let mut error: krpc_schema_Error = _krpc_schema_Error {
        service: pb_callback_s {
            funcs: C2RustUnnamed { decode: None },
            arg: NULL,
        },
        name: pb_callback_s {
            funcs: C2RustUnnamed { decode: None },
            arg: NULL,
        },
        description: pb_callback_s {
            funcs: C2RustUnnamed { decode: None },
            arg: NULL,
        },
        stack_trace: pb_callback_s {
            funcs: C2RustUnnamed { decode: None },
            arg: NULL,
        },
    };
    krpc_error_message[0 as ::core::ffi::c_int as usize] = '\0' as i32
        as ::core::ffi::c_char;
    error.service.funcs.decode = Some(
        krpc_decode_callback_error_string
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
    error.service.arg = b"\0" as *const u8 as *const ::core::ffi::c_char
        as *mut ::core::ffi::c_void;
    error.name.funcs.decode = Some(
        krpc_decode_callback_error_string
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
    error.name.arg = b".\0" as *const u8 as *const ::core::ffi::c_char
        as *mut ::core::ffi::c_void;
    error.description.funcs.decode = Some(
        krpc_decode_callback_error_string
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
    error.description.arg = b": \0" as *const u8 as *const ::core::ffi::c_char
        as *mut ::core::ffi::c_void;
    error.stack_trace.funcs.decode = Some(
        krpc_decode_callback_error_string
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
    error.stack_trace.arg = b"\nServer stack trace:\n\0" as *const u8
        as *const ::core::ffi::c_char as *mut ::core::ffi::c_void;
    if !pb_decode(
        stream,
        &raw const krpc_schema_Error_msg,
        &raw mut error as *mut ::core::ffi::c_void,
    ) {
        return KRPC_ERROR_DECODING_FAILED as ::core::ffi::c_int != 0;
    }
    return true_0 != 0;
}
#[no_mangle]
pub unsafe extern "C" fn krpc_invoke(
    mut connection: krpc_connection_t,
    mut result: *mut krpc_schema_ProcedureResult,
    mut call: *mut krpc_schema_ProcedureCall,
) -> krpc_error_t {
    let mut ostream: pb_ostream_t = krpc_pb_ostream_from_connection(connection);
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
        request.calls[0 as ::core::ffi::c_int as usize] = *call;
        request.calls_count = 1 as pb_size_t;
        if !pb_encode_ex(
            &raw mut ostream,
            &raw const krpc_schema_Request_msg,
            &raw mut request as *const ::core::ffi::c_void,
            PB_ENCODE_DELIMITED,
        ) {
            return KRPC_ERROR_ENCODING_FAILED;
        }
    } else {
        let mut m_request: krpc_schema_MultiplexedRequest = _krpc_schema_MultiplexedRequest {
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
        m_request.has_request = true_0 != 0;
        m_request.request.calls[0 as ::core::ffi::c_int as usize] = *call;
        m_request.request.calls_count = 1 as pb_size_t;
        if !pb_encode_ex(
            &raw mut ostream,
            &raw const krpc_schema_MultiplexedRequest_msg,
            &raw mut m_request as *const ::core::ffi::c_void,
            PB_ENCODE_DELIMITED,
        ) {
            return KRPC_ERROR_ENCODING_FAILED;
        }
    }
    let mut istream: pb_istream_t = krpc_pb_istream_from_connection(connection);
    let mut response_message: krpc_schema_Response = _krpc_schema_Response {
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
    response_message.results[0 as ::core::ffi::c_int as usize] = *result;
    let mut rpc_error: krpc_error_t = KRPC_OK;
    response_message.error.funcs.decode = Some(
        krpc_decode_callback_error
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
    response_message.error.arg = &raw mut rpc_error as *mut ::core::ffi::c_void;
    response_message.results[0 as ::core::ffi::c_int as usize].error.funcs.decode = Some(
        krpc_decode_callback_error
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
    response_message.results[0 as ::core::ffi::c_int as usize].error.arg = &raw mut rpc_error
        as *mut ::core::ffi::c_void;
    if krpc_cnano_transport_uses_standard_rpc(connection) {
        if !pb_decode_ex(
            &raw mut istream,
            &raw const krpc_schema_Response_msg,
            &raw mut response_message as *mut ::core::ffi::c_void,
            PB_DECODE_DELIMITED,
        ) {
            return KRPC_ERROR_DECODING_FAILED;
        }
    } else {
        let mut m_response: krpc_schema_MultiplexedResponse = _krpc_schema_MultiplexedResponse {
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
        m_response.response = response_message;
        if !pb_decode_ex(
            &raw mut istream,
            &raw const krpc_schema_MultiplexedResponse_msg,
            &raw mut m_response as *mut ::core::ffi::c_void,
            PB_DECODE_DELIMITED,
        ) {
            return KRPC_ERROR_DECODING_FAILED;
        }
        response_message = m_response.response;
    }
    if rpc_error as ::core::ffi::c_int != KRPC_OK as ::core::ffi::c_int {
        return KRPC_ERROR_RPC_FAILED;
    }
    if response_message.results_count as ::core::ffi::c_int != 1 as ::core::ffi::c_int {
        return KRPC_ERROR_NO_RESULTS;
    }
    *result = response_message.results[0 as ::core::ffi::c_int as usize];
    return KRPC_OK;
}
unsafe extern "C" fn write_callback(
    mut stream: *mut pb_ostream_t,
    mut buf: *const uint8_t,
    mut count: size_t,
) -> bool {
    let mut connection: krpc_connection_t = (*stream).state as intptr_t
        as krpc_connection_t;
    if krpc_write(connection, buf, count) as ::core::ffi::c_int
        != KRPC_OK as ::core::ffi::c_int
    {
        return false_0 != 0;
    }
    return true_0 != 0;
}
unsafe extern "C" fn read_callback(
    mut stream: *mut pb_istream_t,
    mut buf: *mut uint8_t,
    mut count: size_t,
) -> bool {
    let mut connection: krpc_connection_t = (*stream).state as intptr_t
        as krpc_connection_t;
    let mut result: krpc_error_t = krpc_read(connection, buf, count);
    if result as ::core::ffi::c_int == KRPC_ERROR_EOF as ::core::ffi::c_int {
        (*stream).bytes_left = 0 as size_t;
    }
    if result as ::core::ffi::c_int != KRPC_OK as ::core::ffi::c_int {
        return false_0 != 0;
    }
    return true_0 != 0;
}
unsafe extern "C" fn krpc_pb_ostream_from_connection(
    mut connection: krpc_connection_t,
) -> pb_ostream_t {
    let mut stream: pb_ostream_t = pb_ostream_s {
        callback: Some(
            write_callback
                as unsafe extern "C" fn(
                    *mut pb_ostream_t,
                    *const uint8_t,
                    size_t,
                ) -> bool,
        ),
        state: connection as intptr_t as *mut ::core::ffi::c_void,
        max_size: SIZE_MAX as size_t,
        bytes_written: 0 as size_t,
        errmsg: ::core::ptr::null::<::core::ffi::c_char>(),
    };
    return stream;
}
unsafe extern "C" fn krpc_pb_istream_from_connection(
    mut connection: krpc_connection_t,
) -> pb_istream_t {
    let mut stream: pb_istream_t = pb_istream_s {
        callback: Some(
            read_callback
                as unsafe extern "C" fn(*mut pb_istream_t, *mut uint8_t, size_t) -> bool,
        ),
        state: connection as intptr_t as *mut ::core::ffi::c_void,
        bytes_left: SIZE_MAX as size_t,
        errmsg: ::core::ptr::null::<::core::ffi::c_char>(),
    };
    return stream;
}
pub const NULL: *mut ::core::ffi::c_void = ::core::ptr::null_mut::<
    ::core::ffi::c_void,
>();
pub const SIZE_MAX: ::core::ffi::c_ulong = 18446744073709551615 as ::core::ffi::c_ulong;
pub const KRPC_ERROR_MESSAGE_LENGTH: ::core::ffi::c_int = 512 as ::core::ffi::c_int;
pub const true_0: ::core::ffi::c_int = 1 as ::core::ffi::c_int;
pub const false_0: ::core::ffi::c_int = 0 as ::core::ffi::c_int;
