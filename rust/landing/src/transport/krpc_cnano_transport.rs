//! Project-owned communication shim for kRPC C-Nano 0.6.0.
//!
//! The same C-Nano client speaks ordinary kRPC RPC over TCP for live KSP,
//! multiplexed serial I/O for ShuttleSim, and deterministic fake I/O in offline
//! tests. C-Nano has no Streams or Events; telemetry uses synchronous RPC
//! polling.  The exported symbols (`krpc_open`, `krpc_read`, ... and the
//! `krpc_cnano_*` accessors) are the ones the C-Nano core links against.

use crate::sys::*;
use core::ffi::{c_char, c_int, c_void};

pub type krpc_error_t = c_int;
pub const KRPC_OK: krpc_error_t = 0;
pub const KRPC_ERROR_IO: krpc_error_t = -1;
pub const KRPC_ERROR_EOF: krpc_error_t = -2;

pub const KRPC_CNANO_SERIAL_PATH_CAPACITY: usize = 512;

pub type KrpcCNanoTransportStatus = core::ffi::c_uint;
pub const KRPC_CNANO_TRANSPORT_OK: KrpcCNanoTransportStatus = 0;
pub const KRPC_CNANO_TRANSPORT_EOF: KrpcCNanoTransportStatus = 1;
pub const KRPC_CNANO_TRANSPORT_TIMEOUT: KrpcCNanoTransportStatus = 2;
pub const KRPC_CNANO_TRANSPORT_IO_ERROR: KrpcCNanoTransportStatus = 3;
pub const KRPC_CNANO_TRANSPORT_CLOSED: KrpcCNanoTransportStatus = 4;
pub const KRPC_CNANO_TRANSPORT_INVALID: KrpcCNanoTransportStatus = 5;
pub const KRPC_CNANO_TRANSPORT_ZERO_PROGRESS: KrpcCNanoTransportStatus = 6;

pub type KrpcCNanoTransportProtocol = core::ffi::c_uint;
pub const KRPC_CNANO_PROTOCOL_MULTIPLEXED: KrpcCNanoTransportProtocol = 0;
pub const KRPC_CNANO_PROTOCOL_TCP_RPC: KrpcCNanoTransportProtocol = 1;

pub type KrpcCNanoTransportOpenFn = Option<unsafe extern "C" fn(*mut c_void) -> KrpcCNanoTransportStatus>;
pub type KrpcCNanoTransportCloseFn = Option<unsafe extern "C" fn(*mut c_void) -> KrpcCNanoTransportStatus>;
pub type KrpcCNanoTransportReadFn =
    Option<unsafe extern "C" fn(*mut c_void, *mut u8, usize, *mut usize, c_int) -> KrpcCNanoTransportStatus>;
pub type KrpcCNanoTransportWriteFn =
    Option<unsafe extern "C" fn(*mut c_void, *const u8, usize, *mut usize, c_int) -> KrpcCNanoTransportStatus>;

#[derive(Copy, Clone)]
#[repr(C)]
pub struct KrpcCNanoTransportOps {
    pub open: KrpcCNanoTransportOpenFn,
    pub close: KrpcCNanoTransportCloseFn,
    pub read: KrpcCNanoTransportReadFn,
    pub write: KrpcCNanoTransportWriteFn,
    pub protocol: KrpcCNanoTransportProtocol,
}

#[derive(Copy, Clone)]
#[repr(C)]
pub struct KrpcCNanoTransport {
    pub ops: *const KrpcCNanoTransportOps,
    pub context: *mut c_void,
    pub last_status: KrpcCNanoTransportStatus,
    pub bytes_read: usize,
    pub bytes_written: usize,
    pub timeout_ms: c_int,
    pub deadline_ns: i64,
    pub deadline_active: bool,
    pub is_open: bool,
}

#[derive(Copy, Clone)]
#[repr(C)]
pub struct KrpcCNanoTransportConfig {
    pub storage: *mut KrpcCNanoTransport,
    pub ops: *const KrpcCNanoTransportOps,
    pub context: *mut c_void,
    pub timeout_ms: c_int,
}

pub type krpc_connection_t = *mut KrpcCNanoTransport;
pub type krpc_connection_config_t = KrpcCNanoTransportConfig;

/// Production POSIX serial/pseudo-terminal context.
#[derive(Copy, Clone)]
#[repr(C)]
pub struct KrpcCNanoPosixSerial {
    pub fd: c_int,
    pub timeout_ms: c_int,
    pub baud_rate: c_int,
    pub configure_termios: bool,
    pub path: [c_char; KRPC_CNANO_SERIAL_PATH_CAPACITY],
}

#[derive(Copy, Clone)]
#[repr(C)]
pub struct KrpcCNanoPosixTcp {
    pub fd: c_int,
    pub timeout_ms: c_int,
    pub port: c_int,
    pub host: [c_char; 256],
}

unsafe fn transport_now_ns(now_ns: *mut i64) -> bool {
    if now_ns.is_null() {
        return false;
    }
    let mut now = timespec { tv_sec: 0, tv_nsec: 0 };
    if clock_gettime(CLOCK_MONOTONIC, &mut now) != 0 {
        return false;
    }
    *now_ns = (now.tv_sec as i64).wrapping_mul(1_000_000_000).wrapping_add(now.tv_nsec as i64);
    true
}

unsafe fn transport_deadline_ns(timeout_ms: c_int, deadline_ns: *mut i64) -> bool {
    let mut now_ns: i64 = 0;
    if timeout_ms <= 0 || deadline_ns.is_null() || !transport_now_ns(&mut now_ns) {
        return false;
    }
    *deadline_ns = now_ns.wrapping_add((timeout_ms as i64).wrapping_mul(1_000_000));
    true
}

unsafe fn transport_deadline_remaining_ms(deadline_ns: i64) -> c_int {
    let mut now_ns: i64 = 0;
    if !transport_now_ns(&mut now_ns) {
        return -1;
    }
    let remaining_ns = deadline_ns.wrapping_sub(now_ns);
    if remaining_ns <= 0 {
        return 0;
    }
    let remaining_ms = remaining_ns.wrapping_add(999_999) / 1_000_000;
    if remaining_ms > c_int::MAX as i64 {
        c_int::MAX
    } else {
        remaining_ms as c_int
    }
}

fn transport_error(status: KrpcCNanoTransportStatus) -> krpc_error_t {
    if status == KRPC_CNANO_TRANSPORT_EOF {
        KRPC_ERROR_EOF
    } else {
        KRPC_ERROR_IO
    }
}

unsafe fn transport_config_valid(config: *const krpc_connection_config_t) -> bool {
    !config.is_null()
        && !(*config).storage.is_null()
        && !(*config).ops.is_null()
        && (*(*config).ops).read.is_some()
        && (*(*config).ops).write.is_some()
        && (*config).timeout_ms >= 0
}

#[no_mangle]
pub unsafe extern "C" fn krpc_cnano_transport_config(
    storage: *mut KrpcCNanoTransport,
    ops: *const KrpcCNanoTransportOps,
    context: *mut c_void,
    timeout_ms: c_int,
) -> KrpcCNanoTransportConfig {
    KrpcCNanoTransportConfig { storage, ops, context, timeout_ms }
}

#[no_mangle]
pub unsafe extern "C" fn krpc_open(
    connection: *mut krpc_connection_t,
    config: *const krpc_connection_config_t,
) -> krpc_error_t {
    if connection.is_null() || !transport_config_valid(config) {
        return KRPC_ERROR_IO;
    }
    let storage = (*config).storage;
    core::ptr::write_bytes(storage, 0, 1);
    (*storage).ops = (*config).ops;
    (*storage).context = (*config).context;
    (*storage).timeout_ms = (*config).timeout_ms;
    (*storage).deadline_ns = 0;
    (*storage).deadline_active = false;
    (*storage).last_status = KRPC_CNANO_TRANSPORT_OK;

    let mut status = KRPC_CNANO_TRANSPORT_OK;
    if let Some(open) = (*(*storage).ops).open {
        status = open((*storage).context);
    }
    (*storage).last_status = status;
    if status != KRPC_CNANO_TRANSPORT_OK {
        *connection = core::ptr::null_mut();
        return transport_error(status);
    }
    (*storage).is_open = true;
    *connection = storage;
    KRPC_OK
}

#[no_mangle]
pub unsafe extern "C" fn krpc_close(connection: krpc_connection_t) -> krpc_error_t {
    if connection.is_null() {
        return KRPC_ERROR_IO;
    }
    if !(*connection).is_open {
        return KRPC_OK;
    }
    let mut status = KRPC_CNANO_TRANSPORT_OK;
    if !(*connection).ops.is_null() {
        if let Some(close) = (*(*connection).ops).close {
            status = close((*connection).context);
        }
    }
    (*connection).is_open = false;

    // Preserve the transport cause that made C-Nano close the connection.
    if status != KRPC_CNANO_TRANSPORT_OK {
        (*connection).last_status = status;
        return transport_error(status);
    }
    KRPC_OK
}

#[derive(Copy, Clone, PartialEq)]
enum Direction {
    Read,
    Write,
}

unsafe fn transfer(
    connection: krpc_connection_t,
    buffer: *mut u8,
    count: usize,
    direction: Direction,
) -> krpc_error_t {
    let operation = if connection.is_null() || !(*connection).is_open || (*connection).ops.is_null() {
        None
    } else if direction == Direction::Read {
        (*(*connection).ops).read
    } else {
        None
    };
    let write_operation = if direction == Direction::Write && !connection.is_null() && !(*connection).ops.is_null() {
        (*(*connection).ops).write
    } else {
        None
    };
    let closed = connection.is_null()
        || !(*connection).is_open
        || (*connection).ops.is_null()
        || (if direction == Direction::Read { operation.is_none() } else { write_operation.is_none() })
        || (buffer.is_null() && count != 0);
    if closed {
        if !connection.is_null() {
            (*connection).last_status = KRPC_CNANO_TRANSPORT_CLOSED;
        }
        return KRPC_ERROR_IO;
    }

    let mut total: usize = 0;
    while total < count {
        let mut transferred: usize = 0;
        let mut timeout_ms = (*connection).timeout_ms;
        if (*connection).deadline_active {
            timeout_ms = transport_deadline_remaining_ms((*connection).deadline_ns);
            if timeout_ms < 0 {
                (*connection).last_status = KRPC_CNANO_TRANSPORT_INVALID;
                return KRPC_ERROR_IO;
            }
            if timeout_ms == 0 {
                (*connection).last_status = KRPC_CNANO_TRANSPORT_TIMEOUT;
                return KRPC_ERROR_IO;
            }
        }
        let status = if direction == Direction::Read {
            (operation.unwrap())((*connection).context, buffer.add(total), count - total, &mut transferred, timeout_ms)
        } else {
            (write_operation.unwrap())(
                (*connection).context,
                buffer.add(total) as *const u8,
                count - total,
                &mut transferred,
                timeout_ms,
            )
        };

        if transferred > count - total || (status != KRPC_CNANO_TRANSPORT_OK && transferred != 0) {
            (*connection).last_status = KRPC_CNANO_TRANSPORT_INVALID;
            return KRPC_ERROR_IO;
        }
        if status != KRPC_CNANO_TRANSPORT_OK {
            (*connection).last_status = status;
            return transport_error(status);
        }
        if transferred == 0 {
            (*connection).last_status = KRPC_CNANO_TRANSPORT_ZERO_PROGRESS;
            return KRPC_ERROR_IO;
        }

        total += transferred;
        if direction == Direction::Read {
            (*connection).bytes_read += transferred;
        } else {
            (*connection).bytes_written += transferred;
        }
    }

    (*connection).last_status = KRPC_CNANO_TRANSPORT_OK;
    KRPC_OK
}

#[no_mangle]
pub unsafe extern "C" fn krpc_read(connection: krpc_connection_t, buffer: *mut u8, count: usize) -> krpc_error_t {
    transfer(connection, buffer, count, Direction::Read)
}

#[no_mangle]
pub unsafe extern "C" fn krpc_write(connection: krpc_connection_t, buffer: *const u8, count: usize) -> krpc_error_t {
    transfer(connection, buffer as *mut u8, count, Direction::Write)
}

#[no_mangle]
pub unsafe extern "C" fn krpc_cnano_transport_last_status(connection: krpc_connection_t) -> KrpcCNanoTransportStatus {
    if !connection.is_null() {
        (*connection).last_status
    } else {
        KRPC_CNANO_TRANSPORT_INVALID
    }
}

#[no_mangle]
pub unsafe extern "C" fn krpc_cnano_transport_status_string(status: KrpcCNanoTransportStatus) -> *const c_char {
    let text: &'static [u8] = match status {
        KRPC_CNANO_TRANSPORT_OK => b"ok\0",
        KRPC_CNANO_TRANSPORT_EOF => b"eof\0",
        KRPC_CNANO_TRANSPORT_TIMEOUT => b"timeout\0",
        KRPC_CNANO_TRANSPORT_IO_ERROR => b"io-error\0",
        KRPC_CNANO_TRANSPORT_CLOSED => b"closed\0",
        KRPC_CNANO_TRANSPORT_INVALID => b"invalid\0",
        KRPC_CNANO_TRANSPORT_ZERO_PROGRESS => b"zero-progress\0",
        _ => b"unknown\0",
    };
    text.as_ptr().cast()
}

#[no_mangle]
pub unsafe extern "C" fn krpc_cnano_transport_is_open(connection: krpc_connection_t) -> bool {
    !connection.is_null() && (*connection).is_open
}

#[no_mangle]
pub unsafe extern "C" fn krpc_cnano_transport_bytes_read(connection: krpc_connection_t) -> usize {
    if !connection.is_null() {
        (*connection).bytes_read
    } else {
        0
    }
}

#[no_mangle]
pub unsafe extern "C" fn krpc_cnano_transport_bytes_written(connection: krpc_connection_t) -> usize {
    if !connection.is_null() {
        (*connection).bytes_written
    } else {
        0
    }
}

#[no_mangle]
pub unsafe extern "C" fn krpc_cnano_transport_reset_counters(connection: krpc_connection_t) {
    if !connection.is_null() {
        (*connection).bytes_read = 0;
        (*connection).bytes_written = 0;
    }
}

#[no_mangle]
pub unsafe extern "C" fn krpc_cnano_transport_timeout_ms(connection: krpc_connection_t) -> c_int {
    if !connection.is_null() {
        (*connection).timeout_ms
    } else {
        0
    }
}

#[no_mangle]
pub unsafe extern "C" fn krpc_cnano_transport_uses_standard_rpc(connection: krpc_connection_t) -> bool {
    !connection.is_null()
        && !(*connection).ops.is_null()
        && (*(*connection).ops).protocol == KRPC_CNANO_PROTOCOL_TCP_RPC
}

#[no_mangle]
pub unsafe extern "C" fn krpc_cnano_transport_begin_deadline(connection: krpc_connection_t) -> bool {
    if connection.is_null() {
        return false;
    }
    (*connection).deadline_active = false;
    (*connection).deadline_ns = 0;
    if (*connection).timeout_ms == 0 {
        return true;
    }
    if !transport_deadline_ns((*connection).timeout_ms, &mut (*connection).deadline_ns) {
        return false;
    }
    (*connection).deadline_active = true;
    true
}

#[no_mangle]
pub unsafe extern "C" fn krpc_cnano_transport_end_deadline(connection: krpc_connection_t) {
    if connection.is_null() {
        return;
    }
    (*connection).deadline_active = false;
    (*connection).deadline_ns = 0;
}

// ---------------------------------------------------------------------------
// POSIX serial and TCP back ends

fn serial_speed(baud_rate: c_int) -> speed_t {
    match baud_rate {
        9600 => libc::B9600,
        19200 => libc::B19200,
        38400 => libc::B38400,
        57600 => libc::B57600,
        115200 => libc::B115200,
        230400 => libc::B230400,
        // Apple's termios.h stops at 230400; faster rates go through IOSSIOSPEED.
        #[cfg(not(target_vendor = "apple"))]
        460800 => libc::B460800,
        #[cfg(not(target_vendor = "apple"))]
        921600 => libc::B921600,
        _ => 0,
    }
}

/// `IOSSIOSPEED` from `<IOKit/serial/ioss.h>`: `_IOW('T', 2, speed_t)`.
#[cfg(target_vendor = "apple")]
const IOSSIOSPEED: libc::c_ulong =
    0x8000_0000 | ((core::mem::size_of::<speed_t>() as libc::c_ulong & 0x1fff) << 16) | ((b'T' as libc::c_ulong) << 8) | 2;

unsafe extern "C" fn serial_open(context: *mut c_void) -> KrpcCNanoTransportStatus {
    let serial = context as *mut KrpcCNanoPosixSerial;
    if serial.is_null() || (*serial).path[0] == 0 || (*serial).timeout_ms <= 0 || (*serial).baud_rate <= 0 {
        return KRPC_CNANO_TRANSPORT_INVALID;
    }
    if (*serial).fd >= 0 {
        close((*serial).fd);
    }
    (*serial).fd = open((*serial).path.as_ptr(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (*serial).fd < 0 {
        return KRPC_CNANO_TRANSPORT_IO_ERROR;
    }

    if (*serial).configure_termios && isatty((*serial).fd) != 0 {
        let mut tty: termios = core::mem::zeroed();
        if tcgetattr((*serial).fd, &mut tty) != 0 {
            close((*serial).fd);
            (*serial).fd = -1;
            return KRPC_CNANO_TRANSPORT_IO_ERROR;
        }
        cfmakeraw(&mut tty);
        let speed = serial_speed((*serial).baud_rate);
        #[cfg(target_vendor = "apple")]
        {
            // macOS termios only exposes standard constants through 230400 on
            // current SDKs. IOSSIOSPEED is the supported path for the 921600
            // baud default and other arbitrary high-rate USB serial links. Set
            // a valid base speed first, then apply the exact requested rate.
            let base = if speed == 0 { libc::B230400 } else { speed };
            if cfsetispeed(&mut tty, base) != 0
                || cfsetospeed(&mut tty, base) != 0
                || tcsetattr((*serial).fd, TCSANOW, &tty) != 0
            {
                close((*serial).fd);
                (*serial).fd = -1;
                return KRPC_CNANO_TRANSPORT_IO_ERROR;
            }
            if speed == 0 {
                let exact: speed_t = (*serial).baud_rate as speed_t;
                if libc::ioctl((*serial).fd, IOSSIOSPEED, &exact as *const speed_t) < 0 {
                    close((*serial).fd);
                    (*serial).fd = -1;
                    return KRPC_CNANO_TRANSPORT_IO_ERROR;
                }
            }
        }
        #[cfg(not(target_vendor = "apple"))]
        {
            if speed == 0
                || cfsetispeed(&mut tty, speed) != 0
                || cfsetospeed(&mut tty, speed) != 0
                || tcsetattr((*serial).fd, TCSANOW, &tty) != 0
            {
                close((*serial).fd);
                (*serial).fd = -1;
                return KRPC_CNANO_TRANSPORT_INVALID;
            }
        }
        let _ = tcflush((*serial).fd, TCIOFLUSH);
    }
    KRPC_CNANO_TRANSPORT_OK
}

unsafe extern "C" fn serial_close(context: *mut c_void) -> KrpcCNanoTransportStatus {
    let serial = context as *mut KrpcCNanoPosixSerial;
    if serial.is_null() {
        return KRPC_CNANO_TRANSPORT_INVALID;
    }
    if (*serial).fd >= 0 {
        let fd = (*serial).fd;
        (*serial).fd = -1;
        if close(fd) != 0 {
            return KRPC_CNANO_TRANSPORT_IO_ERROR;
        }
    }
    KRPC_CNANO_TRANSPORT_OK
}

unsafe fn serial_wait(fd: c_int, events: i16, timeout_ms: c_int) -> KrpcCNanoTransportStatus {
    let mut pfd = pollfd { fd, events, revents: 0 };
    let mut deadline_ns: i64 = 0;
    if !transport_deadline_ns(timeout_ms, &mut deadline_ns) {
        return KRPC_CNANO_TRANSPORT_INVALID;
    }
    loop {
        let remaining_ms = transport_deadline_remaining_ms(deadline_ns);
        if remaining_ms < 0 {
            return KRPC_CNANO_TRANSPORT_IO_ERROR;
        }
        if remaining_ms == 0 {
            return KRPC_CNANO_TRANSPORT_TIMEOUT;
        }
        pfd.revents = 0;
        let result = poll(&mut pfd, 1, remaining_ms);
        if result > 0 {
            if pfd.revents & (POLLERR | POLLNVAL) != 0 {
                return KRPC_CNANO_TRANSPORT_IO_ERROR;
            }
            // A pty may report POLLIN|POLLHUP together for the peer's final
            // bytes. Consume readable data before treating HUP as EOF.
            if pfd.revents & events != 0 {
                return KRPC_CNANO_TRANSPORT_OK;
            }
            if pfd.revents & POLLHUP != 0 {
                return KRPC_CNANO_TRANSPORT_EOF;
            }
            continue;
        }
        if result == 0 {
            return KRPC_CNANO_TRANSPORT_TIMEOUT;
        }
        if *__errno_location() == EINTR {
            continue;
        }
        return KRPC_CNANO_TRANSPORT_IO_ERROR;
    }
}

unsafe fn io_read(
    fd: c_int,
    buffer: *mut u8,
    requested: usize,
    transferred: *mut usize,
    timeout_ms: c_int,
    socket: bool,
) -> KrpcCNanoTransportStatus {
    if !transferred.is_null() {
        *transferred = 0;
    }
    if fd < 0 || (buffer.is_null() && requested != 0) || transferred.is_null() {
        return KRPC_CNANO_TRANSPORT_CLOSED;
    }
    if requested == 0 {
        return KRPC_CNANO_TRANSPORT_OK;
    }
    let wait = serial_wait(fd, POLLIN as i16, timeout_ms);
    if wait != KRPC_CNANO_TRANSPORT_OK {
        return wait;
    }
    loop {
        let n = if socket {
            recv(fd, buffer.cast(), requested, 0)
        } else {
            read(fd, buffer.cast(), requested)
        };
        if n > 0 {
            *transferred = n as usize;
            return KRPC_CNANO_TRANSPORT_OK;
        }
        if n == 0 {
            return KRPC_CNANO_TRANSPORT_EOF;
        }
        let error = *__errno_location();
        if error == EINTR {
            continue;
        }
        if error == EAGAIN || error == EWOULDBLOCK {
            return KRPC_CNANO_TRANSPORT_TIMEOUT;
        }
        return KRPC_CNANO_TRANSPORT_IO_ERROR;
    }
}

unsafe fn io_write(
    fd: c_int,
    buffer: *const u8,
    requested: usize,
    transferred: *mut usize,
    timeout_ms: c_int,
    socket: bool,
) -> KrpcCNanoTransportStatus {
    if !transferred.is_null() {
        *transferred = 0;
    }
    if fd < 0 || (buffer.is_null() && requested != 0) || transferred.is_null() {
        return KRPC_CNANO_TRANSPORT_CLOSED;
    }
    if requested == 0 {
        return KRPC_CNANO_TRANSPORT_OK;
    }
    let wait = serial_wait(fd, POLLOUT as i16, timeout_ms);
    if wait != KRPC_CNANO_TRANSPORT_OK {
        return wait;
    }
    loop {
        let n = if socket {
            send(fd, buffer.cast(), requested, 0)
        } else {
            write(fd, buffer.cast(), requested)
        };
        if n > 0 {
            *transferred = n as usize;
            return KRPC_CNANO_TRANSPORT_OK;
        }
        if n == 0 {
            return KRPC_CNANO_TRANSPORT_ZERO_PROGRESS;
        }
        let error = *__errno_location();
        if error == EINTR {
            continue;
        }
        if error == EAGAIN || error == EWOULDBLOCK {
            return KRPC_CNANO_TRANSPORT_TIMEOUT;
        }
        return KRPC_CNANO_TRANSPORT_IO_ERROR;
    }
}

unsafe extern "C" fn serial_read(
    context: *mut c_void,
    buffer: *mut u8,
    requested: usize,
    transferred: *mut usize,
    timeout_ms: c_int,
) -> KrpcCNanoTransportStatus {
    let serial = context as *mut KrpcCNanoPosixSerial;
    if !transferred.is_null() {
        *transferred = 0;
    }
    if serial.is_null() {
        return KRPC_CNANO_TRANSPORT_CLOSED;
    }
    io_read((*serial).fd, buffer, requested, transferred, timeout_ms, false)
}

unsafe extern "C" fn serial_write(
    context: *mut c_void,
    buffer: *const u8,
    requested: usize,
    transferred: *mut usize,
    timeout_ms: c_int,
) -> KrpcCNanoTransportStatus {
    let serial = context as *mut KrpcCNanoPosixSerial;
    if !transferred.is_null() {
        *transferred = 0;
    }
    if serial.is_null() {
        return KRPC_CNANO_TRANSPORT_CLOSED;
    }
    io_write((*serial).fd, buffer, requested, transferred, timeout_ms, false)
}

unsafe extern "C" fn tcp_open(context: *mut c_void) -> KrpcCNanoTransportStatus {
    let tcp = context as *mut KrpcCNanoPosixTcp;
    if tcp.is_null() || (*tcp).host[0] == 0 || (*tcp).port <= 0 || (*tcp).port > 65535 || (*tcp).timeout_ms <= 0 {
        return KRPC_CNANO_TRANSPORT_INVALID;
    }
    if (*tcp).fd >= 0 {
        close((*tcp).fd);
    }
    (*tcp).fd = -1;

    let mut port = [0 as c_char; 16];
    snprintf(port.as_mut_ptr(), port.len(), b"%d\0".as_ptr().cast(), (*tcp).port);
    let mut hints: addrinfo = core::mem::zeroed();
    hints.ai_family = AF_UNSPEC;
    hints.ai_socktype = SOCK_STREAM;
    let mut addresses: *mut addrinfo = core::ptr::null_mut();
    if getaddrinfo((*tcp).host.as_ptr(), port.as_ptr(), &hints, &mut addresses) != 0 {
        return KRPC_CNANO_TRANSPORT_IO_ERROR;
    }

    let mut status = KRPC_CNANO_TRANSPORT_IO_ERROR;
    let mut address = addresses;
    while !address.is_null() {
        let candidate = address;
        address = (*address).ai_next;
        let fd = socket((*candidate).ai_family, (*candidate).ai_socktype, (*candidate).ai_protocol);
        if fd < 0 {
            continue;
        }
        let flags = fcntl(fd, F_GETFL, 0);
        if flags < 0 || fcntl(fd, F_SETFL, flags | O_NONBLOCK) != 0 {
            close(fd);
            continue;
        }
        let result = connect(fd, (*candidate).ai_addr, (*candidate).ai_addrlen);
        if result != 0 && *__errno_location() != EINPROGRESS {
            close(fd);
            continue;
        }
        if result != 0 {
            status = serial_wait(fd, POLLOUT as i16, (*tcp).timeout_ms);
            if status != KRPC_CNANO_TRANSPORT_OK {
                close(fd);
                continue;
            }
            let mut socket_error: c_int = 0;
            let mut socket_error_size = core::mem::size_of::<c_int>() as socklen_t;
            if getsockopt(
                fd,
                SOL_SOCKET,
                SO_ERROR,
                (&mut socket_error as *mut c_int).cast(),
                &mut socket_error_size,
            ) != 0
                || socket_error != 0
            {
                close(fd);
                status = KRPC_CNANO_TRANSPORT_IO_ERROR;
                continue;
            }
        }
        (*tcp).fd = fd;
        status = KRPC_CNANO_TRANSPORT_OK;
        break;
    }
    freeaddrinfo(addresses);
    status
}

unsafe extern "C" fn tcp_close(context: *mut c_void) -> KrpcCNanoTransportStatus {
    let tcp = context as *mut KrpcCNanoPosixTcp;
    if tcp.is_null() {
        return KRPC_CNANO_TRANSPORT_INVALID;
    }
    if (*tcp).fd >= 0 {
        let fd = (*tcp).fd;
        (*tcp).fd = -1;
        if close(fd) != 0 {
            return KRPC_CNANO_TRANSPORT_IO_ERROR;
        }
    }
    KRPC_CNANO_TRANSPORT_OK
}

unsafe extern "C" fn tcp_read(
    context: *mut c_void,
    buffer: *mut u8,
    requested: usize,
    transferred: *mut usize,
    timeout_ms: c_int,
) -> KrpcCNanoTransportStatus {
    let tcp = context as *mut KrpcCNanoPosixTcp;
    if !transferred.is_null() {
        *transferred = 0;
    }
    if tcp.is_null() {
        return KRPC_CNANO_TRANSPORT_CLOSED;
    }
    io_read((*tcp).fd, buffer, requested, transferred, timeout_ms, true)
}

unsafe extern "C" fn tcp_write(
    context: *mut c_void,
    buffer: *const u8,
    requested: usize,
    transferred: *mut usize,
    timeout_ms: c_int,
) -> KrpcCNanoTransportStatus {
    let tcp = context as *mut KrpcCNanoPosixTcp;
    if !transferred.is_null() {
        *transferred = 0;
    }
    if tcp.is_null() {
        return KRPC_CNANO_TRANSPORT_CLOSED;
    }
    io_write((*tcp).fd, buffer, requested, transferred, timeout_ms, true)
}

static K_POSIX_TCP_OPS: KrpcCNanoTransportOps = KrpcCNanoTransportOps {
    open: Some(tcp_open),
    close: Some(tcp_close),
    read: Some(tcp_read),
    write: Some(tcp_write),
    protocol: KRPC_CNANO_PROTOCOL_TCP_RPC,
};

static K_POSIX_SERIAL_OPS: KrpcCNanoTransportOps = KrpcCNanoTransportOps {
    open: Some(serial_open),
    close: Some(serial_close),
    read: Some(serial_read),
    write: Some(serial_write),
    protocol: KRPC_CNANO_PROTOCOL_MULTIPLEXED,
};

unsafe fn copy_c_string(destination: *mut c_char, capacity: usize, source: *const c_char) {
    snprintf(destination, capacity, b"%s\0".as_ptr().cast(), source);
}

#[no_mangle]
pub unsafe extern "C" fn krpc_cnano_posix_tcp_init(
    tcp: *mut KrpcCNanoPosixTcp,
    host: *const c_char,
    port: c_int,
    timeout_ms: c_int,
) {
    if tcp.is_null() {
        return;
    }
    core::ptr::write_bytes(tcp, 0, 1);
    (*tcp).fd = -1;
    (*tcp).timeout_ms = timeout_ms;
    (*tcp).port = port;
    if !host.is_null() {
        copy_c_string((*tcp).host.as_mut_ptr(), (*tcp).host.len(), host);
    }
}

#[no_mangle]
pub unsafe extern "C" fn krpc_cnano_posix_tcp_ops() -> *const KrpcCNanoTransportOps {
    &K_POSIX_TCP_OPS
}

#[no_mangle]
pub unsafe extern "C" fn krpc_cnano_posix_serial_init(
    serial: *mut KrpcCNanoPosixSerial,
    path: *const c_char,
    timeout_ms: c_int,
    baud_rate: c_int,
    configure_termios: bool,
) {
    if serial.is_null() {
        return;
    }
    core::ptr::write_bytes(serial, 0, 1);
    (*serial).fd = -1;
    // Configuration normalization owns defaults. Preserve the supplied
    // transport contract here and fail closed in serial_open if it is invalid.
    (*serial).timeout_ms = timeout_ms;
    (*serial).baud_rate = baud_rate;
    (*serial).configure_termios = configure_termios;
    if !path.is_null() {
        copy_c_string((*serial).path.as_mut_ptr(), (*serial).path.len(), path);
    }
}

#[no_mangle]
pub unsafe extern "C" fn krpc_cnano_posix_serial_ops() -> *const KrpcCNanoTransportOps {
    &K_POSIX_SERIAL_OPS
}
