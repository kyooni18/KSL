//! Links the system libraries the backend uses and records the build time.
//!
//! The C backend logged `__DATE__ " " __TIME__` (compiler local time) as the
//! "native build" identity of every flight log. `LANDING_BUILD_DATE` and
//! `LANDING_BUILD_TIME` reproduce those two strings, in the same formats.

use std::os::raw::{c_char, c_int, c_long};

#[repr(C)]
struct Tm {
    tm_sec: c_int,
    tm_min: c_int,
    tm_hour: c_int,
    tm_mday: c_int,
    tm_mon: c_int,
    tm_year: c_int,
    tm_wday: c_int,
    tm_yday: c_int,
    tm_isdst: c_int,
    tm_gmtoff: c_long,
    tm_zone: *const c_char,
}

extern "C" {
    fn time(t: *mut i64) -> i64;
    fn localtime_r(t: *const i64, out: *mut Tm) -> *mut Tm;
    fn strftime(s: *mut c_char, max: usize, format: *const c_char, tm: *const Tm) -> usize;
}

fn local_time(format: &[u8]) -> String {
    unsafe {
        let now = time(std::ptr::null_mut());
        let mut tm: Tm = std::mem::zeroed();
        localtime_r(&now, &mut tm);
        let mut buffer = [0 as c_char; 64];
        let n = strftime(buffer.as_mut_ptr(), buffer.len(), format.as_ptr().cast(), &tm);
        buffer[..n].iter().map(|&c| c as u8 as char).collect()
    }
}

fn main() {
    println!("cargo:rustc-link-lib=sqlite3");
    println!("cargo:rustc-link-lib=m");
    // __DATE__ is "Mmm dd yyyy" with the day padded by a space, __TIME__ is "hh:mm:ss".
    println!("cargo:rustc-env=LANDING_BUILD_DATE={}", local_time(b"%b %e %Y\0"));
    println!("cargo:rustc-env=LANDING_BUILD_TIME={}", local_time(b"%H:%M:%S\0"));
    println!("cargo:rerun-if-changed=build.rs");
}
