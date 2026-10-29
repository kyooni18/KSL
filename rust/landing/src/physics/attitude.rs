use crate::sys::*;
extern "C" {
    fn asin(__x: ::core::ffi::c_double) -> ::core::ffi::c_double;
    fn atan2(
        __y: ::core::ffi::c_double,
        __x: ::core::ffi::c_double,
    ) -> ::core::ffi::c_double;
    fn cos(__x: ::core::ffi::c_double) -> ::core::ffi::c_double;
    fn sin(__x: ::core::ffi::c_double) -> ::core::ffi::c_double;
    fn sqrt(__x: ::core::ffi::c_double) -> ::core::ffi::c_double;
    fn fmax(
        __x: ::core::ffi::c_double,
        __y: ::core::ffi::c_double,
    ) -> ::core::ffi::c_double;
    fn fmin(
        __x: ::core::ffi::c_double,
        __y: ::core::ffi::c_double,
    ) -> ::core::ffi::c_double;
    fn ss_v3(
        x: ::core::ffi::c_double,
        y: ::core::ffi::c_double,
        z: ::core::ffi::c_double,
    ) -> Vec3;
    fn v3_add(a: Vec3, b: Vec3) -> Vec3;
    fn v3_scale(a: Vec3, s: ::core::ffi::c_double) -> Vec3;
    fn v3_dot(a: Vec3, b: Vec3) -> ::core::ffi::c_double;
    fn v3_cross(a: Vec3, b: Vec3) -> Vec3;
    fn v3_norm(a: Vec3) -> ::core::ffi::c_double;
    fn v3_normalized(a: Vec3) -> Vec3;
    fn v3_rotate_axis(v: Vec3, axis: Vec3, angle_rad: ::core::ffi::c_double) -> Vec3;
    fn ss_clampd(
        x: ::core::ffi::c_double,
        lo: ::core::ffi::c_double,
        hi: ::core::ffi::c_double,
    ) -> ::core::ffi::c_double;
    fn deg2rad(x: ::core::ffi::c_double) -> ::core::ffi::c_double;
    fn wrap_pi(x: ::core::ffi::c_double) -> ::core::ffi::c_double;
    fn quat_normalized(q: Quat) -> Quat;
    fn quat_mul(a: Quat, b: Quat) -> Quat;
    fn quat_axis_angle(axis: Vec3, angle_rad: ::core::ffi::c_double) -> Quat;
    fn quat_rotate(q: Quat, v: Vec3) -> Vec3;
    fn quat_from_basis(forward: Vec3, right: Vec3, up: Vec3) -> Quat;
}
pub type size_t = usize;
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
#[derive(Copy, Clone)]
#[repr(C)]
pub struct LocalFrame {
    pub north: Vec3,
    pub east: Vec3,
    pub up: Vec3,
}
#[derive(Copy, Clone)]
#[repr(C)]
pub struct AttitudeModel {
    pub aoa_rad: ::core::ffi::c_double,
    pub bank_rad: ::core::ffi::c_double,
    pub aoa_rate_rad_s: ::core::ffi::c_double,
    pub bank_rate_rad_s: ::core::ffi::c_double,
    pub requested_aoa_rad: ::core::ffi::c_double,
    pub requested_bank_rad: ::core::ffi::c_double,
    pub cmd_aoa_rad: ::core::ffi::c_double,
    pub cmd_bank_rad: ::core::ffi::c_double,
    pub pitch_wn: ::core::ffi::c_double,
    pub pitch_zeta: ::core::ffi::c_double,
    pub roll_wn: ::core::ffi::c_double,
    pub roll_zeta: ::core::ffi::c_double,
    pub max_pitch_rate_rad_s: ::core::ffi::c_double,
    pub max_roll_rate_rad_s: ::core::ffi::c_double,
    pub max_pitch_accel_rad_s2: ::core::ffi::c_double,
    pub max_roll_accel_rad_s2: ::core::ffi::c_double,
    pub pitch_full_authority_q_pa: ::core::ffi::c_double,
    pub roll_full_authority_q_pa: ::core::ffi::c_double,
    pub direct: bool,
    pub direct_body_coordinates: bool,
    pub body_rates_initialized: bool,
    pub body_pitch_rate_rad_s: ::core::ffi::c_double,
    pub body_roll_rate_rad_s: ::core::ffi::c_double,
    pub body_yaw_rate_rad_s: ::core::ffi::c_double,
    pub input_pitch: ::core::ffi::c_double,
    pub input_roll: ::core::ffi::c_double,
    pub input_yaw: ::core::ffi::c_double,
    pub sideslip_rad: ::core::ffi::c_double,
    pub sideslip_rate_rad_s: ::core::ffi::c_double,
    pub direct_min_q_pa: ::core::ffi::c_double,
    pub direct_pitch_accel_per_kpa: ::core::ffi::c_double,
    pub direct_pitch_accel_max: ::core::ffi::c_double,
    pub direct_pitch_stiffness_per_kpa: ::core::ffi::c_double,
    pub direct_trim_aoa_rad: ::core::ffi::c_double,
    pub direct_pitch_damping_ratio: ::core::ffi::c_double,
    pub direct_roll_accel_per_kpa: ::core::ffi::c_double,
    pub direct_roll_accel_max: ::core::ffi::c_double,
    pub direct_roll_damping_s_inv: ::core::ffi::c_double,
    pub direct_yaw_accel_per_kpa: ::core::ffi::c_double,
    pub direct_yaw_accel_max: ::core::ffi::c_double,
    pub direct_yaw_stiffness_per_kpa: ::core::ffi::c_double,
}
pub type C2RustUnnamed = ::core::ffi::c_uint;
#[no_mangle]
pub unsafe extern "C" fn attitude_seed(mut a: *mut AttitudeModel) {
    memset(
        a as *mut ::core::ffi::c_void,
        0 as ::core::ffi::c_int,
        ::core::mem::size_of::<AttitudeModel>() as size_t,
    );
    (*a).pitch_wn = 1.4f64;
    (*a).pitch_zeta = 0.9f64;
    (*a).roll_wn = 1.8f64;
    (*a).roll_zeta = 0.85f64;
    (*a).max_pitch_rate_rad_s = deg2rad(8.0f64);
    (*a).max_roll_rate_rad_s = deg2rad(18.0f64);
    (*a).max_pitch_accel_rad_s2 = deg2rad(5.0f64);
    (*a).max_roll_accel_rad_s2 = deg2rad(15.0f64);
    (*a).pitch_full_authority_q_pa = 15.0f64;
    (*a).roll_full_authority_q_pa = 10.0f64;
    (*a).direct_min_q_pa = 500.0f64;
    (*a).direct_pitch_accel_per_kpa = deg2rad(4.0f64);
    (*a).direct_pitch_accel_max = deg2rad(40.0f64);
    (*a).direct_pitch_stiffness_per_kpa = 0.12f64;
    (*a).direct_trim_aoa_rad = deg2rad(6.0f64);
    (*a).direct_pitch_damping_ratio = 0.25f64;
    (*a).direct_roll_accel_per_kpa = deg2rad(6.0f64);
    (*a).direct_roll_accel_max = deg2rad(60.0f64);
    (*a).direct_roll_damping_s_inv = 0.8f64;
    (*a).direct_yaw_accel_per_kpa = deg2rad(3.0f64);
    (*a).direct_yaw_accel_max = deg2rad(30.0f64);
    (*a).direct_yaw_stiffness_per_kpa = 0.10f64;
}
#[no_mangle]
pub unsafe extern "C" fn attitude_set_inputs(
    mut a: *mut AttitudeModel,
    mut pitch: ::core::ffi::c_double,
    mut roll: ::core::ffi::c_double,
    mut yaw: ::core::ffi::c_double,
) {
    if a.is_null() {
        return;
    }
    (*a).direct = true_0 != 0;
    (*a).input_pitch = if pitch.is_finite() as i32 != 0 {
        ss_clampd(pitch, -1.0f64, 1.0f64)
    } else {
        0.0f64
    };
    (*a).input_roll = if roll.is_finite() as i32 != 0 {
        ss_clampd(roll, -1.0f64, 1.0f64)
    } else {
        0.0f64
    };
    (*a).input_yaw = if yaw.is_finite() as i32 != 0 {
        ss_clampd(yaw, -1.0f64, 1.0f64)
    } else {
        0.0f64
    };
}
#[no_mangle]
pub unsafe extern "C" fn attitude_set_command(
    mut a: *mut AttitudeModel,
    mut aoa: ::core::ffi::c_double,
    mut bank: ::core::ffi::c_double,
) {
    (*a).requested_aoa_rad = aoa;
    (*a).requested_bank_rad = wrap_pi(bank);
}
unsafe extern "C" fn axis_step(
    mut target: ::core::ffi::c_double,
    mut x: *mut ::core::ffi::c_double,
    mut v: *mut ::core::ffi::c_double,
    mut wn: ::core::ffi::c_double,
    mut zeta: ::core::ffi::c_double,
    mut vmax: ::core::ffi::c_double,
    mut amax: ::core::ffi::c_double,
    mut authority: ::core::ffi::c_double,
    mut dt: ::core::ffi::c_double,
    mut wrap: bool,
) {
    if x.is_null() || v.is_null() || target.is_finite() as i32 == 0
        || (*x).is_finite() as i32 == 0 || (*v).is_finite() as i32 == 0
        || wn.is_finite() as i32 == 0 || !(wn > 0.0f64) || zeta.is_finite() as i32 == 0
        || zeta < 0.0f64 || vmax.is_finite() as i32 == 0 || !(vmax > 0.0f64)
        || amax.is_finite() as i32 == 0 || !(amax > 0.0f64)
        || authority.is_finite() as i32 == 0 || authority < 0.0f64 || authority > 1.0f64
        || dt.is_finite() as i32 == 0 || !(dt > 0.0f64)
    {
        if !v.is_null() {
            *v = 0.0f64;
        }
        return;
    }
    let mut err: ::core::ffi::c_double = target - *x;
    if wrap {
        err = wrap_pi(err);
    }
    if authority <= 1e-12f64 {
        *v = 0.0f64;
        return;
    }
    let mut root: ::core::ffi::c_double = sqrt(authority);
    let mut effective_wn: ::core::ffi::c_double = wn * root;
    let mut effective_vmax: ::core::ffi::c_double = vmax * root;
    let mut effective_amax: ::core::ffi::c_double = amax * authority;
    let mut accel: ::core::ffi::c_double = effective_wn * effective_wn * err
        - 2.0f64 * zeta * effective_wn * *v;
    accel = ss_clampd(accel, -effective_amax, effective_amax);
    *v = ss_clampd(*v + accel * dt, -effective_vmax, effective_vmax);
    *x += *v * dt;
    if wrap {
        *x = wrap_pi(*x);
    }
}
unsafe extern "C" fn direct_step(
    mut a: *mut AttitudeModel,
    mut q: ::core::ffi::c_double,
    mut dt: ::core::ffi::c_double,
) {
    let mut kpa: ::core::ffi::c_double = q / 1000.0f64;
    let mut pitch_ctrl: ::core::ffi::c_double = fmin(
        (*a).direct_pitch_accel_max,
        (*a).direct_pitch_accel_per_kpa * kpa,
    );
    let mut stiffness: ::core::ffi::c_double = (*a).direct_pitch_stiffness_per_kpa * kpa;
    let mut pitch_damp: ::core::ffi::c_double = 2.0f64 * (*a).direct_pitch_damping_ratio
        * sqrt(fmax(stiffness, 0.0f64));
    let mut aoa_acc: ::core::ffi::c_double = pitch_ctrl * (*a).input_pitch
        - stiffness * ((*a).aoa_rad - (*a).direct_trim_aoa_rad)
        - pitch_damp * (*a).aoa_rate_rad_s;
    (*a).aoa_rate_rad_s += aoa_acc * dt;
    (*a).aoa_rad += (*a).aoa_rate_rad_s * dt;
    let mut roll_ctrl: ::core::ffi::c_double = fmin(
        (*a).direct_roll_accel_max,
        (*a).direct_roll_accel_per_kpa * kpa,
    );
    let mut roll_damp: ::core::ffi::c_double = (*a).direct_roll_damping_s_inv
        * sqrt(fmax(kpa, 0.0f64) / 10.0f64);
    let mut bank_acc: ::core::ffi::c_double = roll_ctrl * (*a).input_roll
        - roll_damp * (*a).bank_rate_rad_s;
    (*a).bank_rate_rad_s += bank_acc * dt;
    (*a).bank_rad = wrap_pi((*a).bank_rad + (*a).bank_rate_rad_s * dt);
    let mut yaw_ctrl: ::core::ffi::c_double = fmin(
        (*a).direct_yaw_accel_max,
        (*a).direct_yaw_accel_per_kpa * kpa,
    );
    let mut weathercock: ::core::ffi::c_double = (*a).direct_yaw_stiffness_per_kpa * kpa;
    let mut beta_acc: ::core::ffi::c_double = -yaw_ctrl * (*a).input_yaw
        - weathercock * (*a).sideslip_rad
        - 2.0f64 * 0.3f64 * sqrt(fmax(weathercock, 0.0f64)) * (*a).sideslip_rate_rad_s;
    (*a).sideslip_rate_rad_s += beta_acc * dt;
    (*a).sideslip_rad = ss_clampd(
        (*a).sideslip_rad + (*a).sideslip_rate_rad_s * dt,
        -0.5f64,
        0.5f64,
    );
    (*a).cmd_aoa_rad = (*a).aoa_rad;
    (*a).cmd_bank_rad = (*a).bank_rad;
}
#[no_mangle]
pub unsafe extern "C" fn attitude_step(
    mut a: *mut AttitudeModel,
    mut dynamic_pressure_pa: ::core::ffi::c_double,
    mut dt: ::core::ffi::c_double,
) {
    if a.is_null() {
        return;
    }
    if (*a).direct as ::core::ffi::c_int != 0
        && dynamic_pressure_pa.is_finite() as i32 != 0
        && dynamic_pressure_pa >= (*a).direct_min_q_pa && dt.is_finite() as i32 != 0
        && dt > 0.0f64
    {
        direct_step(a, dynamic_pressure_pa, dt);
        return;
    }
    (*a).cmd_aoa_rad = (*a).requested_aoa_rad;
    (*a).cmd_bank_rad = wrap_pi((*a).requested_bank_rad);
    let mut q: ::core::ffi::c_double = fmax(
        0.0f64,
        if dynamic_pressure_pa.is_finite() as i32 != 0 {
            dynamic_pressure_pa
        } else {
            0.0f64
        },
    );
    let mut pitch_authority: ::core::ffi::c_double = ss_clampd(
        q / fmax((*a).pitch_full_authority_q_pa, 1e-9f64),
        0.0f64,
        1.0f64,
    );
    let mut roll_authority: ::core::ffi::c_double = ss_clampd(
        q / fmax((*a).roll_full_authority_q_pa, 1e-9f64),
        0.0f64,
        1.0f64,
    );
    axis_step(
        (*a).cmd_aoa_rad,
        &raw mut (*a).aoa_rad,
        &raw mut (*a).aoa_rate_rad_s,
        (*a).pitch_wn,
        (*a).pitch_zeta,
        (*a).max_pitch_rate_rad_s,
        (*a).max_pitch_accel_rad_s2,
        pitch_authority,
        dt,
        false_0 != 0,
    );
    axis_step(
        (*a).cmd_bank_rad,
        &raw mut (*a).bank_rad,
        &raw mut (*a).bank_rate_rad_s,
        (*a).roll_wn,
        (*a).roll_zeta,
        (*a).max_roll_rate_rad_s,
        (*a).max_roll_accel_rad_s2,
        roll_authority,
        dt,
        true_0 != 0,
    );
}
#[no_mangle]
pub unsafe extern "C" fn attitude_step_body(
    mut a: *mut AttitudeModel,
    mut body: *mut Quat,
    mut position: Vec3,
    mut vair: Vec3,
    mut q: ::core::ffi::c_double,
    mut dt: ::core::ffi::c_double,
) -> bool {
    if a.is_null() || body.is_null() || !(*a).direct || !(*a).direct_body_coordinates
        || q.is_finite() as i32 == 0 || q < (*a).direct_min_q_pa
        || dt.is_finite() as i32 == 0 || dt <= 0.0f64
        || v3_norm(vair).is_finite() as i32 == 0 || v3_norm(vair) < 1e-6f64
    {
        return false_0 != 0;
    }
    if !(*a).body_rates_initialized {
        (*a).body_yaw_rate_rad_s = 0.0f64;
        (*a).body_roll_rate_rad_s = (*a).body_yaw_rate_rad_s;
        (*a).body_pitch_rate_rad_s = (*a).body_roll_rate_rad_s;
        (*a).body_rates_initialized = true_0 != 0;
    }
    let mut kpa: ::core::ffi::c_double = q / 1000.0f64;
    let mut stiffness: ::core::ffi::c_double = (*a).direct_pitch_stiffness_per_kpa * kpa;
    let mut damping: ::core::ffi::c_double = 2.0f64 * (*a).direct_pitch_damping_ratio
        * sqrt(fmax(stiffness, 0.0f64));
    let mut pitch_acc: ::core::ffi::c_double = fmin(
        (*a).direct_pitch_accel_max,
        (*a).direct_pitch_accel_per_kpa * kpa,
    ) * (*a).input_pitch - stiffness * ((*a).aoa_rad - (*a).direct_trim_aoa_rad)
        - damping * (*a).body_pitch_rate_rad_s;
    let mut roll_acc: ::core::ffi::c_double = fmin(
        (*a).direct_roll_accel_max,
        (*a).direct_roll_accel_per_kpa * kpa,
    ) * (*a).input_roll
        - (*a).direct_roll_damping_s_inv * sqrt(fmax(kpa, 0.0f64) / 10.0f64)
            * (*a).body_roll_rate_rad_s;
    let mut weathercock: ::core::ffi::c_double = (*a).direct_yaw_stiffness_per_kpa * kpa;
    let mut yaw_acc: ::core::ffi::c_double = fmin(
        (*a).direct_yaw_accel_max,
        (*a).direct_yaw_accel_per_kpa * kpa,
    ) * (*a).input_yaw + weathercock * (*a).sideslip_rad
        - 2.0f64 * 0.3f64 * sqrt(fmax(weathercock, 0.0f64)) * (*a).body_yaw_rate_rad_s;
    (*a).body_pitch_rate_rad_s += pitch_acc * dt;
    (*a).body_roll_rate_rad_s += roll_acc * dt;
    (*a).body_yaw_rate_rad_s += yaw_acc * dt;
    let mut omega: Vec3 = ss_v3(
        (*a).body_roll_rate_rad_s,
        (*a).body_pitch_rate_rad_s,
        (*a).body_yaw_rate_rad_s,
    );
    let mut rate: ::core::ffi::c_double = v3_norm(omega);
    if rate > 1e-12f64 {
        *body = quat_normalized(quat_mul(*body, quat_axis_angle(omega, rate * dt)));
    }
    let mut old_aoa: ::core::ffi::c_double = (*a).aoa_rad;
    let mut old_bank: ::core::ffi::c_double = (*a).bank_rad;
    let mut old_beta: ::core::ffi::c_double = (*a).sideslip_rad;
    let mut flight: Vec3 = v3_normalized(vair);
    let mut forward: Vec3 = quat_rotate(
        *body,
        ss_v3(
            1 as ::core::ffi::c_int as ::core::ffi::c_double,
            0 as ::core::ffi::c_int as ::core::ffi::c_double,
            0 as ::core::ffi::c_int as ::core::ffi::c_double,
        ),
    );
    let mut right: Vec3 = quat_rotate(
        *body,
        ss_v3(
            0 as ::core::ffi::c_int as ::core::ffi::c_double,
            1 as ::core::ffi::c_int as ::core::ffi::c_double,
            0 as ::core::ffi::c_int as ::core::ffi::c_double,
        ),
    );
    let mut down: Vec3 = quat_rotate(
        *body,
        ss_v3(
            0 as ::core::ffi::c_int as ::core::ffi::c_double,
            0 as ::core::ffi::c_int as ::core::ffi::c_double,
            1 as ::core::ffi::c_int as ::core::ffi::c_double,
        ),
    );
    (*a).aoa_rad = atan2(v3_dot(flight, down), v3_dot(flight, forward));
    (*a).sideslip_rad = asin(ss_clampd(v3_dot(flight, right), -1.0f64, 1.0f64));
    let mut wind_right: Vec3 = v3_cross(flight, v3_normalized(position));
    if v3_norm(wind_right) > 1e-8f64 {
        wind_right = v3_normalized(wind_right);
        let mut wind_up: Vec3 = v3_normalized(v3_cross(wind_right, flight));
        let mut body_up: Vec3 = v3_scale(down, -1.0f64);
        (*a).bank_rad = atan2(v3_dot(body_up, wind_right), v3_dot(body_up, wind_up));
    }
    (*a).aoa_rate_rad_s = wrap_pi((*a).aoa_rad - old_aoa) / dt;
    (*a).bank_rate_rad_s = wrap_pi((*a).bank_rad - old_bank) / dt;
    (*a).sideslip_rate_rad_s = wrap_pi((*a).sideslip_rad - old_beta) / dt;
    (*a).cmd_aoa_rad = (*a).aoa_rad;
    (*a).cmd_bank_rad = (*a).bank_rad;
    return true_0 != 0;
}
#[no_mangle]
pub unsafe extern "C" fn attitude_body_quat(
    mut p: Vec3,
    mut vair: Vec3,
    mut aoa: ::core::ffi::c_double,
    mut bank: ::core::ffi::c_double,
) -> Quat {
    let mut flight: Vec3 = v3_normalized(vair);
    let mut radial: Vec3 = v3_normalized(p);
    let mut right: Vec3 = v3_normalized(v3_cross(flight, radial));
    if v3_norm(right) < 1e-8f64 {
        right = ss_v3(
            0 as ::core::ffi::c_int as ::core::ffi::c_double,
            1 as ::core::ffi::c_int as ::core::ffi::c_double,
            0 as ::core::ffi::c_int as ::core::ffi::c_double,
        );
    }
    let mut normal: Vec3 = v3_normalized(v3_cross(right, flight));
    let mut body_fwd: Vec3 = v3_normalized(
        v3_add(v3_scale(flight, cos(aoa)), v3_scale(normal, sin(aoa))),
    );
    let mut body_up: Vec3 = v3_normalized(
        v3_add(v3_scale(normal, cos(aoa)), v3_scale(flight, -sin(aoa))),
    );
    body_up = v3_rotate_axis(body_up, body_fwd, bank);
    let mut body_right: Vec3 = v3_normalized(v3_cross(body_fwd, body_up));
    body_up = v3_normalized(v3_cross(body_right, body_fwd));
    let mut body_down: Vec3 = v3_scale(body_up, -1.0f64);
    return quat_from_basis(body_fwd, body_right, body_down);
}
#[no_mangle]
pub unsafe extern "C" fn attitude_body_quat_wind(
    mut p: Vec3,
    mut vair: Vec3,
    mut aoa: ::core::ffi::c_double,
    mut bank: ::core::ffi::c_double,
) -> Quat {
    let mut flight: Vec3 = v3_normalized(vair);
    let mut radial: Vec3 = v3_normalized(p);
    let mut right: Vec3 = v3_normalized(v3_cross(flight, radial));
    if v3_norm(right) < 1e-8f64 {
        right = ss_v3(
            0 as ::core::ffi::c_int as ::core::ffi::c_double,
            1 as ::core::ffi::c_int as ::core::ffi::c_double,
            0 as ::core::ffi::c_int as ::core::ffi::c_double,
        );
    }
    let mut up: Vec3 = v3_normalized(v3_cross(right, flight));
    up = v3_rotate_axis(up, flight, bank);
    let mut forward: Vec3 = v3_add(v3_scale(flight, cos(aoa)), v3_scale(up, sin(aoa)));
    let mut body_up: Vec3 = v3_add(v3_scale(up, cos(aoa)), v3_scale(flight, -sin(aoa)));
    let mut body_right: Vec3 = v3_normalized(v3_cross(forward, body_up));
    return quat_from_basis(forward, body_right, v3_scale(body_up, -1.0f64));
}
#[no_mangle]
pub unsafe extern "C" fn attitude_body_quat_euler(
    mut frame: LocalFrame,
    mut pitch: ::core::ffi::c_double,
    mut heading: ::core::ffi::c_double,
    mut roll: ::core::ffi::c_double,
) -> Quat {
    let mut horizontal: Vec3 = v3_add(
        v3_scale(frame.north, cos(heading)),
        v3_scale(frame.east, sin(heading)),
    );
    let mut forward: Vec3 = v3_add(
        v3_scale(horizontal, cos(pitch)),
        v3_scale(frame.up, sin(pitch)),
    );
    let mut right: Vec3 = v3_add(
        v3_scale(frame.east, cos(heading)),
        v3_scale(frame.north, -sin(heading)),
    );
    let mut up: Vec3 = v3_normalized(v3_cross(right, forward));
    up = v3_rotate_axis(up, forward, roll);
    right = v3_normalized(v3_cross(forward, up));
    return quat_from_basis(forward, right, v3_scale(up, -1.0f64));
}
unsafe extern "C" fn att_trim(
    mut s: *mut ::core::ffi::c_char,
) -> *mut ::core::ffi::c_char {
    while *(*__ctype_b_loc())
        .offset(*s as ::core::ffi::c_uchar as ::core::ffi::c_int as isize)
        as ::core::ffi::c_int
        & _ISspace as ::core::ffi::c_int as ::core::ffi::c_ushort as ::core::ffi::c_int
        != 0
    {
        s = s.offset(1);
    }
    let mut e: *mut ::core::ffi::c_char = s;
    while *e != 0 {
        e = e.offset(1);
    }
    while e > s
        && *(*__ctype_b_loc())
            .offset(
                *e.offset(-(1 as ::core::ffi::c_int) as isize) as ::core::ffi::c_uchar
                    as ::core::ffi::c_int as isize,
            ) as ::core::ffi::c_int
            & _ISspace as ::core::ffi::c_int as ::core::ffi::c_ushort
                as ::core::ffi::c_int != 0
    {
        e = e.offset(-1);
        *e = 0 as ::core::ffi::c_char;
    }
    return s;
}
#[no_mangle]
pub unsafe extern "C" fn attitude_load_ini(
    mut a: *mut AttitudeModel,
    mut path: *const ::core::ffi::c_char,
) -> bool {
    let mut f: *mut FILE = fopen(path, b"r\0" as *const u8 as *const ::core::ffi::c_char)
        as *mut FILE;
    if f.is_null() {
        return false_0 != 0;
    }
    let mut line: [::core::ffi::c_char; 256] = [0; 256];
    while !fgets(
            &raw mut line as *mut ::core::ffi::c_char,
            ::core::mem::size_of::<[::core::ffi::c_char; 256]>() as ::core::ffi::c_int,
            f,
        )
        .is_null()
    {
        let mut p: *mut ::core::ffi::c_char = att_trim(
            &raw mut line as *mut ::core::ffi::c_char,
        );
        if *p == 0 || *p as ::core::ffi::c_int == '#' as i32 {
            continue;
        }
        let mut eq: *mut ::core::ffi::c_char = strchr(p, '=' as i32);
        if eq.is_null() {
            continue;
        }
        *eq = 0 as ::core::ffi::c_char;
        let mut k: *mut ::core::ffi::c_char = att_trim(p);
        let mut v: *mut ::core::ffi::c_char = att_trim(
            eq.offset(1 as ::core::ffi::c_int as isize),
        );
        let mut x: ::core::ffi::c_double = strtod(
            v,
            ::core::ptr::null_mut::<*mut ::core::ffi::c_char>(),
        );
        if strcmp(k, b"pitch_wn\0" as *const u8 as *const ::core::ffi::c_char) == 0 {
            (*a).pitch_wn = x;
        } else if strcmp(
            k,
            b"direct_body_coordinates\0" as *const u8 as *const ::core::ffi::c_char,
        ) == 0
        {
            (*a).direct_body_coordinates = x == 1.0f64;
        } else if strcmp(k, b"pitch_zeta\0" as *const u8 as *const ::core::ffi::c_char)
            == 0
        {
            (*a).pitch_zeta = x;
        } else if strcmp(k, b"roll_wn\0" as *const u8 as *const ::core::ffi::c_char) == 0
        {
            (*a).roll_wn = x;
        } else if strcmp(k, b"roll_zeta\0" as *const u8 as *const ::core::ffi::c_char)
            == 0
        {
            (*a).roll_zeta = x;
        } else if strcmp(
            k,
            b"max_pitch_rate_deg_s\0" as *const u8 as *const ::core::ffi::c_char,
        ) == 0
        {
            (*a).max_pitch_rate_rad_s = deg2rad(x);
        } else if strcmp(
            k,
            b"max_roll_rate_deg_s\0" as *const u8 as *const ::core::ffi::c_char,
        ) == 0
        {
            (*a).max_roll_rate_rad_s = deg2rad(x);
        } else if strcmp(
            k,
            b"max_pitch_accel_deg_s2\0" as *const u8 as *const ::core::ffi::c_char,
        ) == 0
        {
            (*a).max_pitch_accel_rad_s2 = deg2rad(x);
        } else if strcmp(
            k,
            b"max_roll_accel_deg_s2\0" as *const u8 as *const ::core::ffi::c_char,
        ) == 0
        {
            (*a).max_roll_accel_rad_s2 = deg2rad(x);
        } else if strcmp(
            k,
            b"pitch_full_authority_q_pa\0" as *const u8 as *const ::core::ffi::c_char,
        ) == 0
        {
            (*a).pitch_full_authority_q_pa = x;
        } else if strcmp(
            k,
            b"roll_full_authority_q_pa\0" as *const u8 as *const ::core::ffi::c_char,
        ) == 0
        {
            (*a).roll_full_authority_q_pa = x;
        } else if strcmp(
            k,
            b"direct_min_q_pa\0" as *const u8 as *const ::core::ffi::c_char,
        ) == 0
        {
            (*a).direct_min_q_pa = x;
        } else if strcmp(
            k,
            b"direct_pitch_accel_per_kpa_deg\0" as *const u8
                as *const ::core::ffi::c_char,
        ) == 0
        {
            (*a).direct_pitch_accel_per_kpa = deg2rad(x);
        } else if strcmp(
            k,
            b"direct_pitch_accel_max_deg\0" as *const u8 as *const ::core::ffi::c_char,
        ) == 0
        {
            (*a).direct_pitch_accel_max = deg2rad(x);
        } else if strcmp(
            k,
            b"direct_pitch_stiffness_per_kpa\0" as *const u8
                as *const ::core::ffi::c_char,
        ) == 0
        {
            (*a).direct_pitch_stiffness_per_kpa = x;
        } else if strcmp(
            k,
            b"direct_trim_aoa_deg\0" as *const u8 as *const ::core::ffi::c_char,
        ) == 0
        {
            (*a).direct_trim_aoa_rad = deg2rad(x);
        } else if strcmp(
            k,
            b"direct_pitch_damping_ratio\0" as *const u8 as *const ::core::ffi::c_char,
        ) == 0
        {
            (*a).direct_pitch_damping_ratio = x;
        } else if strcmp(
            k,
            b"direct_roll_accel_per_kpa_deg\0" as *const u8 as *const ::core::ffi::c_char,
        ) == 0
        {
            (*a).direct_roll_accel_per_kpa = deg2rad(x);
        } else if strcmp(
            k,
            b"direct_roll_accel_max_deg\0" as *const u8 as *const ::core::ffi::c_char,
        ) == 0
        {
            (*a).direct_roll_accel_max = deg2rad(x);
        } else if strcmp(
            k,
            b"direct_roll_damping_s_inv\0" as *const u8 as *const ::core::ffi::c_char,
        ) == 0
        {
            (*a).direct_roll_damping_s_inv = x;
        }
    }
    fclose(f);
    return true_0 != 0;
}
pub const true_0: ::core::ffi::c_int = 1 as ::core::ffi::c_int;
pub const false_0: ::core::ffi::c_int = 0 as ::core::ffi::c_int;
pub const NULL: *mut ::core::ffi::c_void = ::core::ptr::null_mut::<
    ::core::ffi::c_void,
>();
