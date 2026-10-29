//! The C library surface the backend uses.
//!
//! The translated modules call libc directly, exactly as the C sources did.
//! Everything platform specific (type layouts, constants, the names of the stdio
//! streams, `errno`, the `ctype` tables, `assert`) is resolved here through the
//! `libc` crate, so the same sources build on Linux and macOS instead of
//! carrying one platform's headers around.

#![allow(non_camel_case_types, non_upper_case_globals, non_snake_case)]

use core::ffi::{c_char, c_int, c_uint, c_ushort};
use std::sync::OnceLock;

pub use libc::{
    addrinfo, cc_t, clock_t, clockid_t, in_addr, in_port_t, mode_t, nfds_t, off_t, pid_t, pollfd,
    pthread_attr_t, pthread_mutex_t, pthread_mutexattr_t, pthread_once_t, pthread_t, sa_family_t,
    sockaddr, sockaddr_in, socklen_t, speed_t, ssize_t, suseconds_t, tcflag_t, termios, time_t,
    timespec, timeval, tm, FILE,
};
pub use libc::{
    AF_INET, AF_UNSPEC, CLOCK_MONOTONIC, CLOCK_REALTIME, EAGAIN, EINPROGRESS, EINTR, EWOULDBLOCK,
    F_GETFL, F_SETFL, O_NOCTTY, O_NONBLOCK, O_RDWR, PF_INET, POLLERR, POLLHUP, POLLIN, POLLNVAL, POLLOUT,  PTHREAD_ONCE_INIT, R_OK, SOCK_DGRAM, SOCK_STREAM, SOL_SOCKET, SO_ERROR, SO_RCVTIMEO, TCIOFLUSH,
    TCSANOW,
};
pub type __time_t = libc::time_t;
pub type __suseconds_t = libc::suseconds_t;
pub type __syscall_slong_t = core::ffi::c_long;
pub type __ssize_t = libc::ssize_t;
pub type __mode_t = libc::mode_t;
pub type __clock_t = libc::clock_t;
pub const CLOCKS_PER_SEC: clock_t = 1_000_000;

extern "C" {
    pub fn clock() -> clock_t;
    pub fn inet_pton(af: c_int, src: *const c_char, dst: *mut core::ffi::c_void) -> c_int;
}

pub use libc::{
    access, bind, calloc, cfmakeraw, cfsetispeed, cfsetospeed, clock_gettime, close,
    connect, fclose, fcntl, ferror, fflush, fgets, fileno, fopen, fprintf, fputc, fread, free,
    freeaddrinfo, fsync, fwrite, getaddrinfo, getcwd, getenv, getline, getpid, getsockopt,
    gmtime_r, isatty, malloc, memcmp, memcpy, memmove, memset, mkdir, nanosleep, open,
    poll, pthread_attr_destroy, pthread_attr_init, pthread_attr_setstacksize,
    pthread_join, pthread_mutex_destroy, pthread_mutex_init, pthread_mutex_lock,
    pthread_mutex_unlock, qsort, read, realloc, realpath, recv, recvfrom, send,
    sendto, setsockopt, snprintf, socket, sscanf, strcasecmp, strchr, strcmp, strdup, strerror,
    strftime, strlen, strncmp, strstr, strtod, strtol, tcflush, tcgetattr, tcsetattr, time, write,
};

#[cfg(target_vendor = "apple")]
extern "C" {
    #[link_name = "__stdinp"]
    pub static mut stdin: *mut FILE;
    #[link_name = "__stdoutp"]
    pub static mut stdout: *mut FILE;
    #[link_name = "__stderrp"]
    pub static mut stderr: *mut FILE;
}
#[cfg(not(target_vendor = "apple"))]
extern "C" {
    pub static mut stdin: *mut FILE;
    pub static mut stdout: *mut FILE;
    pub static mut stderr: *mut FILE;
}

/// Address of the calling thread's `errno`.
#[cfg(any(target_os = "linux", target_os = "android"))]
pub unsafe extern "C" fn __errno_location() -> *mut c_int {
    libc::__errno_location()
}
#[cfg(target_vendor = "apple")]
pub unsafe extern "C" fn __errno_location() -> *mut c_int {
    libc::__error()
}
#[cfg(not(any(target_os = "linux", target_os = "android", target_vendor = "apple")))]
pub unsafe extern "C" fn __errno_location() -> *mut c_int {
    libc::__errno_location()
}

// glibc's `isxxx()` macros index a classification table with these bits.  The
// translated sources expand the macros, so the table is provided with the same
// bit assignments, built from the C-locale libc predicates.
pub const _ISupper: c_uint = 0x100;
pub const _ISlower: c_uint = 0x200;
pub const _ISalpha: c_uint = 0x400;
pub const _ISdigit: c_uint = 0x800;
pub const _ISxdigit: c_uint = 0x1000;
pub const _ISspace: c_uint = 0x2000;
pub const _ISprint: c_uint = 0x4000;
pub const _ISgraph: c_uint = 0x8000;
pub const _ISblank: c_uint = 0x1;
pub const _IScntrl: c_uint = 0x2;
pub const _ISpunct: c_uint = 0x4;
pub const _ISalnum: c_uint = 0x8;

struct CtypeTables {
    class: Box<[c_ushort; 384]>,
    lower: Box<[c_int; 384]>,
}

fn ctype_tables() -> &'static CtypeTables {
    static TABLES: OnceLock<CtypeTables> = OnceLock::new();
    TABLES.get_or_init(|| {
        let mut class = Box::new([0 as c_ushort; 384]);
        let mut lower = Box::new([0 as c_int; 384]);
        for slot in 0..384usize {
            let c = slot as c_int - 128;
            // Indices -128..-1 mirror 128..255 (a `char` argument that was not
            // cast to `unsigned char`); the C locale classifies none of them.
            let byte = if c < 0 { c + 256 } else { c };
            lower[slot] = c;
            if byte >= 128 {
                continue;
            }
            let table: [(unsafe extern "C" fn(c_int) -> c_int, c_uint); 12] = [
                (libc::isupper, _ISupper),
                (libc::islower, _ISlower),
                (libc::isalpha, _ISalpha),
                (libc::isdigit, _ISdigit),
                (libc::isxdigit, _ISxdigit),
                (libc::isspace, _ISspace),
                (libc::isprint, _ISprint),
                (libc::isgraph, _ISgraph),
                (libc::isblank, _ISblank),
                (libc::iscntrl, _IScntrl),
                (libc::ispunct, _ISpunct),
                (libc::isalnum, _ISalnum),
            ];
            let mut bits: c_uint = 0;
            for (predicate, bit) in table {
                if unsafe { predicate(byte) } != 0 {
                    bits |= bit;
                }
            }
            class[slot] = bits as c_ushort;
            lower[slot] = unsafe { libc::tolower(byte) };
        }
        CtypeTables { class, lower }
    })
}

pub unsafe extern "C" fn __ctype_b_loc() -> *mut *const c_ushort {
    static mut POINTER: *const c_ushort = core::ptr::null();
    POINTER = ctype_tables().class.as_ptr().add(128);
    &raw mut POINTER
}

pub unsafe extern "C" fn __ctype_tolower_loc() -> *mut *const c_int {
    static mut POINTER: *const c_int = core::ptr::null();
    POINTER = ctype_tables().lower.as_ptr().add(128);
    &raw mut POINTER
}

/// `assert()` failure: the glibc message format, then abort.
pub unsafe extern "C" fn __assert_fail(
    expression: *const c_char,
    file: *const c_char,
    line: c_uint,
    function: *const c_char,
) -> ! {
    fprintf(
        stderr,
        b"%s:%u: %s: Assertion `%s' failed.\n\0".as_ptr().cast(),
        file,
        line,
        function,
        expression,
    );
    libc::abort()
}

/// `pthread_create` taking the translated sources' `Option<unsafe extern "C" fn>` thread routine.
pub unsafe fn pthread_create(
    thread: *mut pthread_t,
    attr: *const pthread_attr_t,
    start_routine: Option<unsafe extern "C" fn(*mut core::ffi::c_void) -> *mut core::ffi::c_void>,
    arg: *mut core::ffi::c_void,
) -> c_int {
    // SAFETY: `unsafe extern "C" fn` and `extern "C" fn` have the same ABI.
    libc::pthread_create(thread, attr, core::mem::transmute(start_routine.unwrap()), arg)
}

/// `pthread_once` taking the translated sources' `Option<unsafe extern "C" fn>` routine.
pub unsafe fn pthread_once(control: *mut pthread_once_t, routine: Option<unsafe extern "C" fn()>) -> c_int {
    libc::pthread_once(control, core::mem::transmute(routine.unwrap()))
}
