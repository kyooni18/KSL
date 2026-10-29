use crate::sys::*;
extern "C" {
    fn atan2(
        __y: ::core::ffi::c_double,
        __x: ::core::ffi::c_double,
    ) -> ::core::ffi::c_double;
    fn cos(__x: ::core::ffi::c_double) -> ::core::ffi::c_double;
    fn sin(__x: ::core::ffi::c_double) -> ::core::ffi::c_double;
    fn hypot(
        __x: ::core::ffi::c_double,
        __y: ::core::ffi::c_double,
    ) -> ::core::ffi::c_double;
    fn fabs(__x: ::core::ffi::c_double) -> ::core::ffi::c_double;
    fn remainder(
        __x: ::core::ffi::c_double,
        __y: ::core::ffi::c_double,
    ) -> ::core::ffi::c_double;
    fn fmax(
        __x: ::core::ffi::c_double,
        __y: ::core::ffi::c_double,
    ) -> ::core::ffi::c_double;
    fn fmin(
        __x: ::core::ffi::c_double,
        __y: ::core::ffi::c_double,
    ) -> ::core::ffi::c_double;
}
#[derive(Copy, Clone)]
#[repr(C)]
pub struct TaemVec3 {
    pub x: ::core::ffi::c_double,
    pub y: ::core::ffi::c_double,
    pub z: ::core::ffi::c_double,
}
#[derive(Copy, Clone)]
#[repr(C)]
pub struct TaemFrameWorld {
    pub radius_m: ::core::ffi::c_double,
    pub mu_m3_s2: ::core::ffi::c_double,
    pub rotation_axis_i: TaemVec3,
    pub rotation_rate_rad_s: ::core::ffi::c_double,
    pub rotation_phase_rad_at_ut0: ::core::ffi::c_double,
}
#[derive(Copy, Clone)]
#[repr(C)]
pub struct TaemRunway {
    pub latitude_rad: ::core::ffi::c_double,
    pub longitude_rad: ::core::ffi::c_double,
    pub elevation_m: ::core::ffi::c_double,
    pub heading_rad: ::core::ffi::c_double,
    pub length_m: ::core::ffi::c_double,
    pub width_m: ::core::ffi::c_double,
}
#[derive(Copy, Clone)]
#[repr(C)]
pub struct TaemRunwayFrame {
    pub threshold_unit_b: TaemVec3,
    pub forward_b: TaemVec3,
    pub right_b: TaemVec3,
    pub elevation_m: ::core::ffi::c_double,
    pub length_m: ::core::ffi::c_double,
    pub width_m: ::core::ffi::c_double,
}
#[derive(Copy, Clone)]
#[repr(C)]
pub struct TaemRunwayCoordinates {
    pub position_b_m: TaemVec3,
    pub radius_from_center_m: ::core::ffi::c_double,
    pub altitude_above_datum_m: ::core::ffi::c_double,
    pub altitude_above_runway_m: ::core::ffi::c_double,
    pub runway_along_m: ::core::ffi::c_double,
    pub runway_cross_m: ::core::ffi::c_double,
    pub runway_vertical_m: ::core::ffi::c_double,
}
#[derive(Copy, Clone)]
#[repr(C)]
pub struct TaemEnergyDiagnostic {
    pub gravitational_potential_j_kg: ::core::ffi::c_double,
    pub inertial_specific_energy_j_kg: ::core::ffi::c_double,
    pub effective_specific_energy_j_kg: ::core::ffi::c_double,
    pub inertial_energy_rate_w_kg: ::core::ffi::c_double,
    pub effective_energy_rate_w_kg: ::core::ffi::c_double,
    pub airspeed_mps: ::core::ffi::c_double,
    pub drag_work_rate_w_kg: ::core::ffi::c_double,
}
#[derive(Copy, Clone)]
#[repr(C)]
pub struct TaemGravityDiagnostic {
    pub gravity_mps2: ::core::ffi::c_double,
    pub surface_speed_mps: ::core::ffi::c_double,
    pub centrifugal_accel_mps2: ::core::ffi::c_double,
}
pub const true_0: ::core::ffi::c_int = 1 as ::core::ffi::c_int;
pub const false_0: ::core::ffi::c_int = 0 as ::core::ffi::c_int;
pub const TAEM_PI: ::core::ffi::c_double = 3.14159265358979323846264338327950288f64;
pub const TAEM_EPS: ::core::ffi::c_double = 1e-12f64;
unsafe extern "C" fn finite3(mut v: TaemVec3) -> bool {
    return v.x.is_finite() as i32 != 0 && v.y.is_finite() as i32 != 0
        && v.z.is_finite() as i32 != 0;
}
unsafe extern "C" fn valid_world(mut w: *const TaemFrameWorld) -> bool {
    return !w.is_null() && (*w).radius_m.is_finite() as i32 != 0
        && (*w).radius_m > 0.0f64 && (*w).mu_m3_s2.is_finite() as i32 != 0
        && (*w).mu_m3_s2 > 0.0f64
        && finite3((*w).rotation_axis_i) as ::core::ffi::c_int != 0
        && taem_vec3_norm((*w).rotation_axis_i) > TAEM_EPS
        && (*w).rotation_rate_rad_s.is_finite() as i32 != 0
        && (*w).rotation_phase_rad_at_ut0.is_finite() as i32 != 0;
}
unsafe extern "C" fn unit(mut a: TaemVec3) -> TaemVec3 {
    let mut n: ::core::ffi::c_double = taem_vec3_norm(a);
    return if n > TAEM_EPS {
        taem_vec3_scale(a, 1.0f64 / n)
    } else {
        TaemVec3 {
            x: 0.0f64,
            y: 0.0f64,
            z: 0.0f64,
        }
    };
}
unsafe extern "C" fn rotate_axis(
    mut v: TaemVec3,
    mut axis: TaemVec3,
    mut angle: ::core::ffi::c_double,
) -> TaemVec3 {
    axis = unit(axis);
    let mut c: ::core::ffi::c_double = cos(angle);
    let mut s: ::core::ffi::c_double = sin(angle);
    return taem_vec3_add(
        taem_vec3_add(
            taem_vec3_scale(v, c),
            taem_vec3_scale(taem_vec3_cross(axis, v), s),
        ),
        taem_vec3_scale(axis, taem_vec3_dot(axis, v) * (1.0f64 - c)),
    );
}
unsafe extern "C" fn omega_cross(
    mut w: *const TaemFrameWorld,
    mut v: TaemVec3,
) -> TaemVec3 {
    let mut omega: TaemVec3 = taem_vec3_scale(
        unit((*w).rotation_axis_i),
        (*w).rotation_rate_rad_s,
    );
    return taem_vec3_cross(omega, v);
}
unsafe extern "C" fn valid_runway_frame(
    mut w: *const TaemFrameWorld,
    mut f: *const TaemRunwayFrame,
) -> bool {
    if !valid_world(w) || f.is_null() || !finite3((*f).threshold_unit_b)
        || !finite3((*f).forward_b) || !finite3((*f).right_b)
        || (*f).elevation_m.is_finite() as i32 == 0
        || (*f).length_m.is_finite() as i32 == 0 || (*f).width_m.is_finite() as i32 == 0
        || (*f).length_m <= 0.0f64 || (*f).width_m <= 0.0f64
        || (*w).radius_m + (*f).elevation_m <= 0.0f64
    {
        return false_0 != 0;
    }
    let mut u: ::core::ffi::c_double = taem_vec3_norm((*f).threshold_unit_b);
    let mut a: ::core::ffi::c_double = taem_vec3_norm((*f).forward_b);
    let mut b: ::core::ffi::c_double = taem_vec3_norm((*f).right_b);
    return fabs(u - 1.0f64) < 1e-7f64 && fabs(a - 1.0f64) < 1e-7f64
        && fabs(b - 1.0f64) < 1e-7f64
        && fabs(taem_vec3_dot((*f).threshold_unit_b, (*f).forward_b)) < 1e-7f64
        && fabs(taem_vec3_dot((*f).threshold_unit_b, (*f).right_b)) < 1e-7f64
        && fabs(taem_vec3_dot((*f).forward_b, (*f).right_b)) < 1e-7f64
        && taem_vec3_dot(
            taem_vec3_cross((*f).forward_b, (*f).right_b),
            (*f).threshold_unit_b,
        ) < -0.999999f64;
}
#[no_mangle]
pub unsafe extern "C" fn taem_vec3_add(mut a: TaemVec3, mut b: TaemVec3) -> TaemVec3 {
    return TaemVec3 {
        x: a.x + b.x,
        y: a.y + b.y,
        z: a.z + b.z,
    };
}
#[no_mangle]
pub unsafe extern "C" fn taem_vec3_sub(mut a: TaemVec3, mut b: TaemVec3) -> TaemVec3 {
    return TaemVec3 {
        x: a.x - b.x,
        y: a.y - b.y,
        z: a.z - b.z,
    };
}
#[no_mangle]
pub unsafe extern "C" fn taem_vec3_scale(
    mut a: TaemVec3,
    mut s: ::core::ffi::c_double,
) -> TaemVec3 {
    return TaemVec3 {
        x: a.x * s,
        y: a.y * s,
        z: a.z * s,
    };
}
#[no_mangle]
pub unsafe extern "C" fn taem_vec3_dot(
    mut a: TaemVec3,
    mut b: TaemVec3,
) -> ::core::ffi::c_double {
    return a.x * b.x + a.y * b.y + a.z * b.z;
}
#[no_mangle]
pub unsafe extern "C" fn taem_vec3_cross(mut a: TaemVec3, mut b: TaemVec3) -> TaemVec3 {
    return TaemVec3 {
        x: a.y * b.z - a.z * b.y,
        y: a.z * b.x - a.x * b.z,
        z: a.x * b.y - a.y * b.x,
    };
}
#[no_mangle]
pub unsafe extern "C" fn taem_vec3_norm(mut a: TaemVec3) -> ::core::ffi::c_double {
    return hypot(hypot(a.x, a.y), a.z);
}
#[no_mangle]
pub unsafe extern "C" fn taem_wrap_pi(
    mut a: ::core::ffi::c_double,
) -> ::core::ffi::c_double {
    if a.is_finite() as i32 == 0 {
        return ::core::f32::NAN as ::core::ffi::c_double;
    }
    a = remainder(a, 2.0f64 * TAEM_PI);
    return if a <= -TAEM_PI { a + 2.0f64 * TAEM_PI } else { a };
}
#[no_mangle]
pub unsafe extern "C" fn taem_inertial_to_fixed(
    mut w: *const TaemFrameWorld,
    mut p_i: TaemVec3,
    mut v_i: TaemVec3,
    mut ut: ::core::ffi::c_double,
    mut p_b: *mut TaemVec3,
    mut v_b: *mut TaemVec3,
) -> bool {
    if !valid_world(w) || !finite3(p_i) || !finite3(v_i) || ut.is_finite() as i32 == 0
        || p_b.is_null() || v_b.is_null()
    {
        return false_0 != 0;
    }
    let mut angle: ::core::ffi::c_double = (*w).rotation_phase_rad_at_ut0
        + (*w).rotation_rate_rad_s * ut;
    *p_b = rotate_axis(p_i, (*w).rotation_axis_i, -angle);
    *v_b = rotate_axis(
        taem_vec3_sub(v_i, omega_cross(w, p_i)),
        (*w).rotation_axis_i,
        -angle,
    );
    return finite3(*p_b) as ::core::ffi::c_int != 0
        && finite3(*v_b) as ::core::ffi::c_int != 0;
}
#[no_mangle]
pub unsafe extern "C" fn taem_fixed_to_inertial(
    mut w: *const TaemFrameWorld,
    mut p_b: TaemVec3,
    mut v_b: TaemVec3,
    mut ut: ::core::ffi::c_double,
    mut p_i: *mut TaemVec3,
    mut v_i: *mut TaemVec3,
) -> bool {
    if !valid_world(w) || !finite3(p_b) || !finite3(v_b) || ut.is_finite() as i32 == 0
        || p_i.is_null() || v_i.is_null()
    {
        return false_0 != 0;
    }
    let mut angle: ::core::ffi::c_double = (*w).rotation_phase_rad_at_ut0
        + (*w).rotation_rate_rad_s * ut;
    *p_i = rotate_axis(p_b, (*w).rotation_axis_i, angle);
    *v_i = rotate_axis(
        taem_vec3_add(v_b, omega_cross(w, p_b)),
        (*w).rotation_axis_i,
        angle,
    );
    return finite3(*p_i) as ::core::ffi::c_int != 0
        && finite3(*v_i) as ::core::ffi::c_int != 0;
}
#[no_mangle]
pub unsafe extern "C" fn taem_local_north_east_up(
    mut p: TaemVec3,
    mut north: *mut TaemVec3,
    mut east: *mut TaemVec3,
    mut up: *mut TaemVec3,
) -> bool {
    if !finite3(p) || north.is_null() || east.is_null() || up.is_null()
        || taem_vec3_norm(p) <= TAEM_EPS
    {
        return false_0 != 0;
    }
    *up = unit(p);
    let mut z: TaemVec3 = TaemVec3 {
        x: 0.0f64,
        y: 0.0f64,
        z: 1.0f64,
    };
    *east = unit(taem_vec3_cross(z, *up));
    if taem_vec3_norm(*east) <= TAEM_EPS {
        *east = TaemVec3 {
            x: 0.0f64,
            y: 1.0f64,
            z: 0.0f64,
        };
    }
    *north = unit(taem_vec3_cross(*up, *east));
    return true_0 != 0;
}
#[no_mangle]
pub unsafe extern "C" fn taem_runway_frame_create(
    mut w: *const TaemFrameWorld,
    mut r: *const TaemRunway,
    mut f: *mut TaemRunwayFrame,
) -> bool {
    if !valid_world(w) || r.is_null() || f.is_null()
        || (*r).latitude_rad.is_finite() as i32 == 0
        || (*r).longitude_rad.is_finite() as i32 == 0
        || (*r).elevation_m.is_finite() as i32 == 0
        || (*r).heading_rad.is_finite() as i32 == 0
        || (*r).length_m.is_finite() as i32 == 0 || (*r).width_m.is_finite() as i32 == 0
        || fabs((*r).latitude_rad) > TAEM_PI / 2.0f64 || (*r).length_m <= 0.0f64
        || (*r).width_m <= 0.0f64 || (*w).radius_m + (*r).elevation_m <= 0.0f64
    {
        return false_0 != 0;
    }
    let mut cl: ::core::ffi::c_double = cos((*r).latitude_rad);
    let mut sl: ::core::ffi::c_double = sin((*r).latitude_rad);
    let mut co: ::core::ffi::c_double = cos((*r).longitude_rad);
    let mut so: ::core::ffi::c_double = sin((*r).longitude_rad);
    let mut up: TaemVec3 = TaemVec3 {
        x: cl * co,
        y: cl * so,
        z: sl,
    };
    let mut north: TaemVec3 = TaemVec3 {
        x: -sl * co,
        y: -sl * so,
        z: cl,
    };
    let mut east: TaemVec3 = TaemVec3 {
        x: -so,
        y: co,
        z: 0.0f64,
    };
    let mut ch: ::core::ffi::c_double = cos((*r).heading_rad);
    let mut sh: ::core::ffi::c_double = sin((*r).heading_rad);
    (*f).threshold_unit_b = up;
    (*f).forward_b = taem_vec3_add(
        taem_vec3_scale(north, ch),
        taem_vec3_scale(east, sh),
    );
    (*f).right_b = taem_vec3_sub(taem_vec3_scale(east, ch), taem_vec3_scale(north, sh));
    (*f).elevation_m = (*r).elevation_m;
    (*f).length_m = (*r).length_m;
    (*f).width_m = (*r).width_m;
    return true_0 != 0;
}
unsafe extern "C" fn sphere_exp(
    mut origin: TaemVec3,
    mut tangent: TaemVec3,
    mut angle: ::core::ffi::c_double,
) -> TaemVec3 {
    return unit(
        taem_vec3_add(
            taem_vec3_scale(origin, cos(angle)),
            taem_vec3_scale(tangent, sin(angle)),
        ),
    );
}
unsafe extern "C" fn sphere_log(
    mut origin: TaemVec3,
    mut target: TaemVec3,
    mut tangent: *mut TaemVec3,
    mut angle: *mut ::core::ffi::c_double,
) -> bool {
    let mut c: ::core::ffi::c_double = fmax(
        -1.0f64,
        fmin(1.0f64, taem_vec3_dot(origin, target)),
    );
    let mut s: ::core::ffi::c_double = taem_vec3_norm(taem_vec3_cross(origin, target));
    let mut a: ::core::ffi::c_double = atan2(s, c);
    if a >= TAEM_PI - 1e-9f64 {
        return false_0 != 0;
    }
    *angle = a;
    *tangent = unit(taem_vec3_sub(target, taem_vec3_scale(origin, c)));
    return a < 1e-10f64 || s > TAEM_EPS;
}
#[no_mangle]
pub unsafe extern "C" fn taem_runway_frame_reciprocal(
    mut w: *const TaemFrameWorld,
    mut s: *const TaemRunwayFrame,
    mut r: *mut TaemRunwayFrame,
) -> bool {
    if !valid_runway_frame(w, s) || r.is_null() {
        return false_0 != 0;
    }
    let mut u: TaemVec3 = unit((*s).threshold_unit_b);
    let mut f: TaemVec3 = unit((*s).forward_b);
    let mut right: TaemVec3 = unit((*s).right_b);
    let mut end: TaemVec3 = sphere_exp(
        u,
        f,
        (*s).length_m / ((*w).radius_m + (*s).elevation_m),
    );
    let mut axis: TaemVec3 = unit(taem_vec3_cross(u, f));
    let mut end_forward: TaemVec3 = rotate_axis(
        f,
        axis,
        (*s).length_m / ((*w).radius_m + (*s).elevation_m),
    );
    (*r).threshold_unit_b = end;
    (*r).forward_b = taem_vec3_scale(end_forward, -1.0f64);
    (*r).right_b = taem_vec3_scale(
        rotate_axis(right, axis, (*s).length_m / ((*w).radius_m + (*s).elevation_m)),
        -1.0f64,
    );
    (*r).elevation_m = (*s).elevation_m;
    (*r).length_m = (*s).length_m;
    (*r).width_m = (*s).width_m;
    return finite3((*r).threshold_unit_b) as ::core::ffi::c_int != 0
        && finite3((*r).forward_b) as ::core::ffi::c_int != 0
        && finite3((*r).right_b) as ::core::ffi::c_int != 0;
}
#[no_mangle]
pub unsafe extern "C" fn taem_runway_project(
    mut w: *const TaemFrameWorld,
    mut f: *const TaemRunwayFrame,
    mut p: TaemVec3,
    mut c: *mut TaemRunwayCoordinates,
) -> bool {
    if !valid_runway_frame(w, f) || c.is_null() || !finite3(p)
        || taem_vec3_norm(p) <= TAEM_EPS
    {
        return false_0 != 0;
    }
    let mut radius: ::core::ffi::c_double = taem_vec3_norm(p);
    let mut u: TaemVec3 = taem_vec3_scale(p, 1.0f64 / radius);
    let mut tangent: TaemVec3 = TaemVec3 { x: 0., y: 0., z: 0. };
    let mut angle: ::core::ffi::c_double = 0.;
    if !sphere_log(unit((*f).threshold_unit_b), u, &raw mut tangent, &raw mut angle) {
        return false_0 != 0;
    }
    let mut base: ::core::ffi::c_double = (*w).radius_m + (*f).elevation_m;
    if base <= 0.0f64 {
        return false_0 != 0;
    }
    let mut logv: TaemVec3 = taem_vec3_scale(tangent, angle * base);
    (*c).position_b_m = p;
    (*c).radius_from_center_m = radius;
    (*c).altitude_above_datum_m = radius - (*w).radius_m;
    (*c).altitude_above_runway_m = radius - base;
    (*c).runway_along_m = taem_vec3_dot(logv, unit((*f).forward_b));
    (*c).runway_cross_m = taem_vec3_dot(logv, unit((*f).right_b));
    (*c).runway_vertical_m = (*c).altitude_above_runway_m;
    return true_0 != 0;
}
#[no_mangle]
pub unsafe extern "C" fn taem_runway_unproject(
    mut w: *const TaemFrameWorld,
    mut f: *const TaemRunwayFrame,
    mut along: ::core::ffi::c_double,
    mut cross: ::core::ffi::c_double,
    mut above: ::core::ffi::c_double,
    mut p: *mut TaemVec3,
) -> bool {
    if !valid_runway_frame(w, f) || p.is_null() || along.is_finite() as i32 == 0
        || cross.is_finite() as i32 == 0 || above.is_finite() as i32 == 0
    {
        return false_0 != 0;
    }
    let mut base: ::core::ffi::c_double = (*w).radius_m + (*f).elevation_m;
    let mut radius: ::core::ffi::c_double = base + above;
    if base <= 0.0f64 || radius <= 0.0f64 {
        return false_0 != 0;
    }
    let mut u: TaemVec3 = unit((*f).threshold_unit_b);
    let mut e: TaemVec3 = unit(
        taem_vec3_add(
            taem_vec3_scale((*f).forward_b, along),
            taem_vec3_scale((*f).right_b, cross),
        ),
    );
    let mut distance: ::core::ffi::c_double = hypot(along, cross);
    let mut angle: ::core::ffi::c_double = distance / base;
    *p = taem_vec3_scale(sphere_exp(u, e, angle), radius);
    return finite3(*p);
}
#[no_mangle]
pub unsafe extern "C" fn taem_gravity_diagnostic(
    mut w: *const TaemFrameWorld,
    mut p: TaemVec3,
    mut d: *mut TaemGravityDiagnostic,
) -> bool {
    if !valid_world(w) || !finite3(p) || d.is_null() {
        return false_0 != 0;
    }
    let mut r: ::core::ffi::c_double = taem_vec3_norm(p);
    if r <= TAEM_EPS {
        return false_0 != 0;
    }
    let mut omegaxr: TaemVec3 = omega_cross(w, p);
    (*d).gravity_mps2 = (*w).mu_m3_s2 / (r * r);
    (*d).surface_speed_mps = taem_vec3_norm(omegaxr);
    (*d).centrifugal_accel_mps2 = taem_vec3_norm(omega_cross(w, omegaxr));
    return (*d).gravity_mps2.is_finite() as i32 != 0
        && (*d).surface_speed_mps.is_finite() as i32 != 0
        && (*d).centrifugal_accel_mps2.is_finite() as i32 != 0;
}
#[no_mangle]
pub unsafe extern "C" fn taem_energy_diagnostic(
    mut w: *const TaemFrameWorld,
    mut p_i: TaemVec3,
    mut v_i: TaemVec3,
    mut v_b: TaemVec3,
    mut air_b: TaemVec3,
    mut force_i: TaemVec3,
    mut force_b: TaemVec3,
    mut mass: ::core::ffi::c_double,
    mut drag: ::core::ffi::c_double,
    mut d: *mut TaemEnergyDiagnostic,
) -> bool {
    if !valid_world(w) || !finite3(p_i) || !finite3(v_i) || !finite3(v_b)
        || !finite3(air_b) || !finite3(force_i) || !finite3(force_b) || d.is_null()
        || mass.is_finite() as i32 == 0 || mass <= 0.0f64 || drag.is_finite() as i32 == 0
        || drag < 0.0f64
    {
        return false_0 != 0;
    }
    let mut r: ::core::ffi::c_double = taem_vec3_norm(p_i);
    if r <= TAEM_EPS {
        return false_0 != 0;
    }
    let mut grav: ::core::ffi::c_double = -(*w).mu_m3_s2 / r;
    (*d).gravitational_potential_j_kg = grav;
    (*d).inertial_specific_energy_j_kg = 0.5f64 * taem_vec3_dot(v_i, v_i) + grav;
    let mut omega_cross_position: TaemVec3 = omega_cross(w, p_i);
    (*d).effective_specific_energy_j_kg = 0.5f64 * taem_vec3_dot(v_b, v_b) + grav
        - 0.5f64 * taem_vec3_dot(omega_cross_position, omega_cross_position);
    (*d).inertial_energy_rate_w_kg = taem_vec3_dot(force_i, v_i) / mass;
    (*d).effective_energy_rate_w_kg = taem_vec3_dot(force_b, v_b) / mass;
    (*d).airspeed_mps = taem_vec3_norm(air_b);
    (*d).drag_work_rate_w_kg = -drag * (*d).airspeed_mps / mass;
    return (*d).gravitational_potential_j_kg.is_finite() as i32 != 0
        && (*d).inertial_specific_energy_j_kg.is_finite() as i32 != 0
        && (*d).effective_specific_energy_j_kg.is_finite() as i32 != 0
        && (*d).inertial_energy_rate_w_kg.is_finite() as i32 != 0
        && (*d).effective_energy_rate_w_kg.is_finite() as i32 != 0
        && (*d).airspeed_mps.is_finite() as i32 != 0
        && (*d).drag_work_rate_w_kg.is_finite() as i32 != 0;
}
#[no_mangle]
pub unsafe extern "C" fn taem_no_wind_energy_work_residual(
    mut d: *const TaemEnergyDiagnostic,
    mut residual: *mut ::core::ffi::c_double,
) -> bool {
    if d.is_null() || residual.is_null()
        || (*d).effective_energy_rate_w_kg.is_finite() as i32 == 0
        || (*d).drag_work_rate_w_kg.is_finite() as i32 == 0
    {
        return false_0 != 0;
    }
    *residual = (*d).effective_energy_rate_w_kg - (*d).drag_work_rate_w_kg;
    return (*residual).is_finite() as i32 != 0;
}
#[no_mangle]
pub unsafe extern "C" fn taem_ground_arc_work_rate(
    mut drag: ::core::ffi::c_double,
    mut mass: ::core::ffi::c_double,
    mut speed: ::core::ffi::c_double,
    mut gamma: ::core::ffi::c_double,
    mut radius: ::core::ffi::c_double,
    mut reference_radius: ::core::ffi::c_double,
    mut work_rate: *mut ::core::ffi::c_double,
) -> bool {
    if work_rate.is_null() || drag.is_finite() as i32 == 0 || drag < 0.0f64
        || mass.is_finite() as i32 == 0 || mass <= 0.0f64
        || speed.is_finite() as i32 == 0 || speed < 0.0f64
        || gamma.is_finite() as i32 == 0 || radius.is_finite() as i32 == 0
        || radius <= 0.0f64 || reference_radius.is_finite() as i32 == 0
        || reference_radius <= 0.0f64
    {
        return false_0 != 0;
    }
    *work_rate = -(drag / mass) * speed;
    return (*work_rate).is_finite() as i32 != 0;
}
