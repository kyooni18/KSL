use crate::sys::*;
extern "C" {
    fn cos(__x: ::core::ffi::c_double) -> ::core::ffi::c_double;
    fn sin(__x: ::core::ffi::c_double) -> ::core::ffi::c_double;
    fn sqrt(__x: ::core::ffi::c_double) -> ::core::ffi::c_double;
}
#[derive(Copy, Clone)]
#[repr(C)]
pub struct Vec3 {
    pub x: ::core::ffi::c_double,
    pub y: ::core::ffi::c_double,
    pub z: ::core::ffi::c_double,
}
#[no_mangle]
pub unsafe extern "C" fn ss_v3(
    mut x: ::core::ffi::c_double,
    mut y: ::core::ffi::c_double,
    mut z: ::core::ffi::c_double,
) -> Vec3 {
    let mut v: Vec3 = Vec3 { x: x, y: y, z: z };
    return v;
}
#[no_mangle]
pub unsafe extern "C" fn v3_add(mut a: Vec3, mut b: Vec3) -> Vec3 {
    return ss_v3(a.x + b.x, a.y + b.y, a.z + b.z);
}
#[no_mangle]
pub unsafe extern "C" fn v3_sub(mut a: Vec3, mut b: Vec3) -> Vec3 {
    return ss_v3(a.x - b.x, a.y - b.y, a.z - b.z);
}
#[no_mangle]
pub unsafe extern "C" fn v3_scale(mut a: Vec3, mut s: ::core::ffi::c_double) -> Vec3 {
    return ss_v3(a.x * s, a.y * s, a.z * s);
}
#[no_mangle]
pub unsafe extern "C" fn v3_dot(mut a: Vec3, mut b: Vec3) -> ::core::ffi::c_double {
    return a.x * b.x + a.y * b.y + a.z * b.z;
}
#[no_mangle]
pub unsafe extern "C" fn v3_cross(mut a: Vec3, mut b: Vec3) -> Vec3 {
    return ss_v3(a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x);
}
#[no_mangle]
pub unsafe extern "C" fn v3_norm(mut a: Vec3) -> ::core::ffi::c_double {
    return sqrt(v3_dot(a, a));
}
#[no_mangle]
pub unsafe extern "C" fn v3_normalized(mut a: Vec3) -> Vec3 {
    let mut n: ::core::ffi::c_double = v3_norm(a);
    return if n > 1e-12f64 {
        v3_scale(a, 1.0f64 / n)
    } else {
        ss_v3(
            0 as ::core::ffi::c_int as ::core::ffi::c_double,
            0 as ::core::ffi::c_int as ::core::ffi::c_double,
            0 as ::core::ffi::c_int as ::core::ffi::c_double,
        )
    };
}
#[no_mangle]
pub unsafe extern "C" fn v3_rotate_axis(
    mut v: Vec3,
    mut axis: Vec3,
    mut angle_rad: ::core::ffi::c_double,
) -> Vec3 {
    axis = v3_normalized(axis);
    let mut c: ::core::ffi::c_double = cos(angle_rad);
    let mut s: ::core::ffi::c_double = sin(angle_rad);
    return v3_add(
        v3_add(v3_scale(v, c), v3_scale(v3_cross(axis, v), s)),
        v3_scale(axis, v3_dot(axis, v) * (1.0f64 - c)),
    );
}
#[no_mangle]
pub unsafe extern "C" fn ss_clampd(
    mut x: ::core::ffi::c_double,
    mut lo: ::core::ffi::c_double,
    mut hi: ::core::ffi::c_double,
) -> ::core::ffi::c_double {
    return if x < lo { lo } else if x > hi { hi } else { x };
}
#[no_mangle]
pub unsafe extern "C" fn deg2rad(mut x: ::core::ffi::c_double) -> ::core::ffi::c_double {
    return x * (3.14159265358979323846f64 / 180.0f64);
}
#[no_mangle]
pub unsafe extern "C" fn rad2deg(mut x: ::core::ffi::c_double) -> ::core::ffi::c_double {
    return x * (180.0f64 / 3.14159265358979323846f64);
}
#[no_mangle]
pub unsafe extern "C" fn wrap_pi(mut x: ::core::ffi::c_double) -> ::core::ffi::c_double {
    let p: ::core::ffi::c_double = 3.14159265358979323846f64;
    let t: ::core::ffi::c_double = 2.0f64 * p;
    while x > p {
        x -= t;
    }
    while x < -p {
        x += t;
    }
    return x;
}
