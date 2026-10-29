use crate::sys::*;
extern "C" {
    fn cos(__x: ::core::ffi::c_double) -> ::core::ffi::c_double;
    fn sin(__x: ::core::ffi::c_double) -> ::core::ffi::c_double;
    fn sqrt(__x: ::core::ffi::c_double) -> ::core::ffi::c_double;
    fn ss_v3(
        x: ::core::ffi::c_double,
        y: ::core::ffi::c_double,
        z: ::core::ffi::c_double,
    ) -> Vec3;
    fn v3_add(a: Vec3, b: Vec3) -> Vec3;
    fn v3_scale(a: Vec3, s: ::core::ffi::c_double) -> Vec3;
    fn v3_cross(a: Vec3, b: Vec3) -> Vec3;
    fn v3_normalized(a: Vec3) -> Vec3;
}
#[derive(Copy, Clone)]
#[repr(C)]
pub struct Vec3 {
    pub x: ::core::ffi::c_double,
    pub y: ::core::ffi::c_double,
    pub z: ::core::ffi::c_double,
}
#[derive(Copy, Clone)]
#[repr(C)]
pub struct Quat {
    pub w: ::core::ffi::c_double,
    pub x: ::core::ffi::c_double,
    pub y: ::core::ffi::c_double,
    pub z: ::core::ffi::c_double,
}
#[no_mangle]
pub unsafe extern "C" fn quat_identity() -> Quat {
    let mut q: Quat = Quat {
        w: 1 as ::core::ffi::c_int as ::core::ffi::c_double,
        x: 0 as ::core::ffi::c_int as ::core::ffi::c_double,
        y: 0 as ::core::ffi::c_int as ::core::ffi::c_double,
        z: 0 as ::core::ffi::c_int as ::core::ffi::c_double,
    };
    return q;
}
#[no_mangle]
pub unsafe extern "C" fn quat_normalized(mut q: Quat) -> Quat {
    let mut n: ::core::ffi::c_double = sqrt(
        q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z,
    );
    if n < 1e-12f64 {
        return quat_identity();
    }
    let mut r: Quat = Quat {
        w: q.w / n,
        x: q.x / n,
        y: q.y / n,
        z: q.z / n,
    };
    return r;
}
#[no_mangle]
pub unsafe extern "C" fn quat_mul(mut a: Quat, mut b: Quat) -> Quat {
    let mut q: Quat = Quat {
        w: a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z,
        x: a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y,
        y: a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x,
        z: a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w,
    };
    return q;
}
#[no_mangle]
pub unsafe extern "C" fn quat_axis_angle(
    mut axis: Vec3,
    mut angle_rad: ::core::ffi::c_double,
) -> Quat {
    axis = v3_normalized(axis);
    let mut h: ::core::ffi::c_double = 0.5f64 * angle_rad;
    let mut s: ::core::ffi::c_double = sin(h);
    let mut q: Quat = Quat {
        w: cos(h),
        x: axis.x * s,
        y: axis.y * s,
        z: axis.z * s,
    };
    return quat_normalized(q);
}
#[no_mangle]
pub unsafe extern "C" fn quat_rotate(mut q: Quat, mut v: Vec3) -> Vec3 {
    q = quat_normalized(q);
    let mut u: Vec3 = ss_v3(q.x, q.y, q.z);
    let mut t: Vec3 = v3_scale(v3_cross(u, v), 2.0f64);
    return v3_add(v, v3_add(v3_scale(t, q.w), v3_cross(u, t)));
}
#[no_mangle]
pub unsafe extern "C" fn quat_from_basis(
    mut forward: Vec3,
    mut right: Vec3,
    mut up: Vec3,
) -> Quat {
    let mut m00: ::core::ffi::c_double = forward.x;
    let mut m01: ::core::ffi::c_double = right.x;
    let mut m02: ::core::ffi::c_double = up.x;
    let mut m10: ::core::ffi::c_double = forward.y;
    let mut m11: ::core::ffi::c_double = right.y;
    let mut m12: ::core::ffi::c_double = up.y;
    let mut m20: ::core::ffi::c_double = forward.z;
    let mut m21: ::core::ffi::c_double = right.z;
    let mut m22: ::core::ffi::c_double = up.z;
    let mut tr: ::core::ffi::c_double = m00 + m11 + m22;
    let mut q: Quat = Quat { w: 0., x: 0., y: 0., z: 0. };
    if tr > 0 as ::core::ffi::c_int as ::core::ffi::c_double {
        let mut s: ::core::ffi::c_double = sqrt(tr + 1.0f64) * 2.0f64;
        q.w = 0.25f64 * s;
        q.x = (m21 - m12) / s;
        q.y = (m02 - m20) / s;
        q.z = (m10 - m01) / s;
    } else if m00 > m11 && m00 > m22 {
        let mut s_0: ::core::ffi::c_double = sqrt(1.0f64 + m00 - m11 - m22) * 2.0f64;
        q.w = (m21 - m12) / s_0;
        q.x = 0.25f64 * s_0;
        q.y = (m01 + m10) / s_0;
        q.z = (m02 + m20) / s_0;
    } else if m11 > m22 {
        let mut s_1: ::core::ffi::c_double = sqrt(1.0f64 + m11 - m00 - m22) * 2.0f64;
        q.w = (m02 - m20) / s_1;
        q.x = (m01 + m10) / s_1;
        q.y = 0.25f64 * s_1;
        q.z = (m12 + m21) / s_1;
    } else {
        let mut s_2: ::core::ffi::c_double = sqrt(1.0f64 + m22 - m00 - m11) * 2.0f64;
        q.w = (m10 - m01) / s_2;
        q.x = (m02 + m20) / s_2;
        q.y = (m12 + m21) / s_2;
        q.z = 0.25f64 * s_2;
    }
    return quat_normalized(q);
}
