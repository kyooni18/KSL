use crate::sys::*;
extern "C" {
    fn acos(__x: ::core::ffi::c_double) -> ::core::ffi::c_double;
    fn cos(__x: ::core::ffi::c_double) -> ::core::ffi::c_double;
    fn sin(__x: ::core::ffi::c_double) -> ::core::ffi::c_double;
    fn exp(__x: ::core::ffi::c_double) -> ::core::ffi::c_double;
    fn log(__x: ::core::ffi::c_double) -> ::core::ffi::c_double;
    fn sqrt(__x: ::core::ffi::c_double) -> ::core::ffi::c_double;
    fn fmax(
        __x: ::core::ffi::c_double,
        __y: ::core::ffi::c_double,
    ) -> ::core::ffi::c_double;
    fn fmin(
        __x: ::core::ffi::c_double,
        __y: ::core::ffi::c_double,
    ) -> ::core::ffi::c_double;
    fn entry_remaining_specific_energy(
        latitude: ::core::ffi::c_double,
        altitude: ::core::ffi::c_double,
        air_relative_speed: ::core::ffi::c_double,
        target_latitude: ::core::ffi::c_double,
        target_altitude: ::core::ffi::c_double,
        target_speed: ::core::ffi::c_double,
        planet: *const PlanetModel,
    ) -> ::core::ffi::c_double;
    fn planet_atmospheric_density(
        p: *const PlanetModel,
        altitude: ::core::ffi::c_double,
    ) -> ::core::ffi::c_double;
}
pub type size_t = usize;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct EntryEnergyState {
    pub initialized: bool,
    pub integral_m_s: ::core::ffi::c_double,
    pub last_ut: ::core::ffi::c_double,
}
#[derive(Copy, Clone)]
#[repr(C)]
pub struct Vector3 {
    pub x: ::core::ffi::c_double,
    pub y: ::core::ffi::c_double,
    pub z: ::core::ffi::c_double,
}
#[derive(Copy, Clone)]
#[repr(C)]
pub struct VehicleProfile {
    pub model_id: [::core::ffi::c_char; 96],
    pub estimated_lift_to_drag: ::core::ffi::c_double,
    pub estimated_ballistic_coefficient: ::core::ffi::c_double,
    pub entry_angle_of_attack: ::core::ffi::c_double,
    pub maximum_angle_of_attack: ::core::ffi::c_double,
    pub maximum_bank_angle: ::core::ffi::c_double,
    pub maximum_dynamic_pressure: ::core::ffi::c_double,
    pub maximum_g_load: ::core::ffi::c_double,
    pub minimum_safe_speed: ::core::ffi::c_double,
    pub final_approach_speed: ::core::ffi::c_double,
    pub touchdown_speed: ::core::ffi::c_double,
    pub terminal_maximum_lift_angle_of_attack: ::core::ffi::c_double,
    pub allow_powered_approach: bool,
    pub maximum_approach_throttle: ::core::ffi::c_double,
    pub airbrake_action_group: ::core::ffi::c_uint,
}
#[derive(Copy, Clone)]
#[repr(C)]
pub struct PlanetModel {
    pub name: [::core::ffi::c_char; 96],
    pub radius: ::core::ffi::c_double,
    pub gravitational_parameter: ::core::ffi::c_double,
    pub rotational_speed: ::core::ffi::c_double,
    pub atmosphere_depth: ::core::ffi::c_double,
    pub surface_density: ::core::ffi::c_double,
    pub atmosphere_adiabatic_index: ::core::ffi::c_double,
    pub atmosphere_sample_count: size_t,
    pub atmosphere_altitude: [::core::ffi::c_double; 1024],
    pub atmosphere_pressure: [::core::ffi::c_double; 1024],
    pub atmosphere_density: [::core::ffi::c_double; 1024],
    pub north_axis: Vector3,
    pub prime_meridian_at_epoch: Vector3,
    pub epoch_ut: ::core::ffi::c_double,
}
#[derive(Copy, Clone)]
#[repr(C)]
pub struct EntryEnergyConfig {
    pub natural_frequency_rad_s: ::core::ffi::c_double,
    pub damping_ratio: ::core::ffi::c_double,
    pub integral_ratio: ::core::ffi::c_double,
    pub integral_limit_m_s: ::core::ffi::c_double,
    pub altitude_error_limit_m: ::core::ffi::c_double,
    pub minimum_drag_accel: ::core::ffi::c_double,
    pub minimum_lift_accel: ::core::ffi::c_double,
    pub protection_fraction: ::core::ffi::c_double,
    pub prediction_steps: ::core::ffi::c_uint,
}
#[derive(Copy, Clone)]
#[repr(C)]
pub struct EntryEnergyInput {
    pub ut: ::core::ffi::c_double,
    pub relative_velocity: ::core::ffi::c_double,
    pub altitude: ::core::ffi::c_double,
    pub latitude: ::core::ffi::c_double,
    pub flight_path_angle_deg: ::core::ffi::c_double,
    pub vertical_speed: ::core::ffi::c_double,
    pub measured_drag_accel: ::core::ffi::c_double,
    pub measured_lift_accel: ::core::ffi::c_double,
    pub dynamic_pressure: ::core::ffi::c_double,
    pub range_to_go: ::core::ffi::c_double,
    pub taem_velocity: ::core::ffi::c_double,
    pub taem_altitude: ::core::ffi::c_double,
    pub taem_latitude: ::core::ffi::c_double,
    pub transition_velocity: ::core::ffi::c_double,
    pub maximum_bank_deg: ::core::ffi::c_double,
    pub integrator_hold: bool,
    pub planet: *const PlanetModel,
    pub vehicle: *const VehicleProfile,
}
#[derive(Copy, Clone)]
#[repr(C)]
pub struct EntryEnergyOutput {
    pub valid: bool,
    pub degraded: bool,
    pub scale_height_m: ::core::ffi::c_double,
    pub reference_drag_accel: ::core::ffi::c_double,
    pub reference_drag_dv: ::core::ffi::c_double,
    pub profile_drag_accel: ::core::ffi::c_double,
    pub taem_drag_accel: ::core::ffi::c_double,
    pub maximum_drag_accel: ::core::ffi::c_double,
    pub predicted_range_m: ::core::ffi::c_double,
    pub required_range_m: ::core::ffi::c_double,
    pub range_error_m: ::core::ffi::c_double,
    pub profile_at_maximum: bool,
    pub profile_at_minimum: bool,
    pub hdot_reference: ::core::ffi::c_double,
    pub altitude_error_m: ::core::ffi::c_double,
    pub integral_m_s: ::core::ffi::c_double,
    pub vertical_lift_command: ::core::ffi::c_double,
    pub lift_to_drag: ::core::ffi::c_double,
    pub bank_magnitude_deg: ::core::ffi::c_double,
    pub saturated_lift_up: bool,
    pub saturated_bank_limit: bool,
    pub protection_active: bool,
}
pub const LANDER_PI: ::core::ffi::c_double = 3.14159265358979323846264338327950288f64;
pub const DEG2RAD: ::core::ffi::c_double = LANDER_PI / 180.0f64;
pub const RAD2DEG: ::core::ffi::c_double = 180.0f64 / LANDER_PI;
pub const STANDARD_GRAVITY: ::core::ffi::c_double = 9.80665f64;
#[no_mangle]
pub unsafe extern "C" fn entry_energy_default_config() -> EntryEnergyConfig {
    let mut c: EntryEnergyConfig = EntryEnergyConfig {
        natural_frequency_rad_s: 0.,
        damping_ratio: 0.,
        integral_ratio: 0.,
        integral_limit_m_s: 0.,
        altitude_error_limit_m: 0.,
        minimum_drag_accel: 0.,
        minimum_lift_accel: 0.,
        protection_fraction: 0.,
        prediction_steps: 0,
    };
    memset(
        &raw mut c as *mut ::core::ffi::c_void,
        0 as ::core::ffi::c_int,
        ::core::mem::size_of::<EntryEnergyConfig>() as size_t,
    );
    c.natural_frequency_rad_s = 0.07f64;
    c.damping_ratio = 0.8f64;
    c.integral_ratio = 0.1f64;
    c.integral_limit_m_s = 60000.0f64;
    c.altitude_error_limit_m = 12000.0f64;
    c.minimum_drag_accel = 0.02f64;
    c.minimum_lift_accel = 0.05f64;
    c.protection_fraction = 0.9f64;
    c.prediction_steps = 64 as ::core::ffi::c_uint;
    return c;
}
#[no_mangle]
pub unsafe extern "C" fn entry_energy_reset(mut state: *mut EntryEnergyState) {
    if state.is_null() {
        return;
    }
    memset(
        state as *mut ::core::ffi::c_void,
        0 as ::core::ffi::c_int,
        ::core::mem::size_of::<EntryEnergyState>() as size_t,
    );
    (*state).last_ut = ::core::f32::NAN as ::core::ffi::c_double;
}
#[no_mangle]
pub unsafe extern "C" fn entry_energy_scale_height(
    mut planet: *const PlanetModel,
    mut altitude: ::core::ffi::c_double,
) -> ::core::ffi::c_double {
    let fallback: ::core::ffi::c_double = 5600.0f64;
    if planet.is_null() || altitude.is_finite() as i32 == 0 {
        return fallback;
    }
    let mut step: ::core::ffi::c_double = 250.0f64;
    let mut low: ::core::ffi::c_double = fmax(0.0f64, altitude - step);
    let mut high: ::core::ffi::c_double = low + 2.0f64 * step;
    let mut rho_low: ::core::ffi::c_double = planet_atmospheric_density(planet, low);
    let mut rho_high: ::core::ffi::c_double = planet_atmospheric_density(planet, high);
    if !(rho_low > 0.0f64) || !(rho_high > 0.0f64) || !(rho_low > rho_high) {
        return fallback;
    }
    let mut hs: ::core::ffi::c_double = (high - low) / log(rho_low / rho_high);
    return if hs.is_finite() as i32 != 0 {
        fmin(fmax(hs, 2000.0f64), 12000.0f64)
    } else {
        fallback
    };
}
unsafe extern "C" fn profile_drag(
    mut v: ::core::ffi::c_double,
    mut v_taem: ::core::ffi::c_double,
    mut v_trans: ::core::ffi::c_double,
    mut v_now: ::core::ffi::c_double,
    mut d0: ::core::ffi::c_double,
    mut d_taem: ::core::ffi::c_double,
) -> ::core::ffi::c_double {
    let mut v1: ::core::ffi::c_double = if v_now < v_trans { v_now } else { v_trans };
    if v >= v1 {
        return d0;
    }
    let mut span: ::core::ffi::c_double = fmax(v1 - v_taem, 1.0f64);
    let mut x: ::core::ffi::c_double = fmin(fmax((v - v_taem) / span, 0.0f64), 1.0f64);
    return d_taem + (d0 - d_taem) * x;
}
unsafe extern "C" fn predicted_range(
    mut in_0: *const EntryEnergyInput,
    mut c: *const EntryEnergyConfig,
    mut d0: ::core::ffi::c_double,
    mut d_taem: ::core::ffi::c_double,
    mut remaining_energy: ::core::ffi::c_double,
) -> ::core::ffi::c_double {
    let mut v: ::core::ffi::c_double = (*in_0).relative_velocity;
    let mut vt: ::core::ffi::c_double = (*in_0).taem_velocity;
    let mut range: ::core::ffi::c_double = 0.0f64;
    let mut dmin: ::core::ffi::c_double = (*c).minimum_drag_accel;
    if v > vt {
        let mut dv: ::core::ffi::c_double = (v - vt)
            / (*c).prediction_steps as ::core::ffi::c_double;
        let mut n: ::core::ffi::c_uint = 0 as ::core::ffi::c_uint;
        while n < (*c).prediction_steps {
            let mut vv: ::core::ffi::c_double = vt
                + (n as ::core::ffi::c_double + 0.5f64) * dv;
            let mut d: ::core::ffi::c_double = profile_drag(
                vv,
                vt,
                (*in_0).transition_velocity,
                v,
                d0,
                d_taem,
            );
            range += vv / fmax(dmin, d) * dv;
            n = n.wrapping_add(1);
        }
    }
    let mut kinetic: ::core::ffi::c_double = 0.5f64 * (v * v - vt * vt);
    let mut other: ::core::ffi::c_double = remaining_energy - fmax(0.0f64, kinetic);
    if other != 0.0f64 {
        let mut d_now: ::core::ffi::c_double = profile_drag(
            fmax(v, vt),
            vt,
            (*in_0).transition_velocity,
            v,
            d0,
            d_taem,
        );
        let mut mean: ::core::ffi::c_double = sqrt(
            fmax(dmin, d_now) * fmax(dmin, d_taem),
        );
        range += other / fmax(dmin, mean);
    }
    return range;
}
#[no_mangle]
pub unsafe extern "C" fn entry_energy_update(
    mut state: *mut EntryEnergyState,
    mut in_0: *const EntryEnergyInput,
    mut c: *const EntryEnergyConfig,
) -> EntryEnergyOutput {
    let mut out: EntryEnergyOutput = EntryEnergyOutput {
        valid: false,
        degraded: false,
        scale_height_m: 0.,
        reference_drag_accel: 0.,
        reference_drag_dv: 0.,
        profile_drag_accel: 0.,
        taem_drag_accel: 0.,
        maximum_drag_accel: 0.,
        predicted_range_m: 0.,
        required_range_m: 0.,
        range_error_m: 0.,
        profile_at_maximum: false,
        profile_at_minimum: false,
        hdot_reference: 0.,
        altitude_error_m: 0.,
        integral_m_s: 0.,
        vertical_lift_command: 0.,
        lift_to_drag: 0.,
        bank_magnitude_deg: 0.,
        saturated_lift_up: false,
        saturated_bank_limit: false,
        protection_active: false,
    };
    memset(
        &raw mut out as *mut ::core::ffi::c_void,
        0 as ::core::ffi::c_int,
        ::core::mem::size_of::<EntryEnergyOutput>() as size_t,
    );
    if state.is_null() || in_0.is_null() || c.is_null() || (*in_0).planet.is_null()
        || (*in_0).vehicle.is_null() || (*in_0).relative_velocity.is_finite() as i32 == 0
        || !((*in_0).relative_velocity > 0.0f64)
        || (*in_0).altitude.is_finite() as i32 == 0
        || (*in_0).latitude.is_finite() as i32 == 0
        || (*in_0).flight_path_angle_deg.is_finite() as i32 == 0
        || (*in_0).vertical_speed.is_finite() as i32 == 0
        || (*in_0).range_to_go.is_finite() as i32 == 0
        || !((*in_0).taem_velocity > 0.0f64)
        || (*in_0).taem_altitude.is_finite() as i32 == 0
        || (*in_0).transition_velocity.is_finite() as i32 == 0
        || (*in_0).maximum_bank_deg.is_finite() as i32 == 0
        || (*in_0).ut.is_finite() as i32 == 0
        || (*c).prediction_steps < 8 as ::core::ffi::c_uint
    {
        return out;
    }
    if !(*state).initialized {
        entry_energy_reset(state);
        (*state).initialized = true_0 != 0;
    }
    let mut dt: ::core::ffi::c_double = if (*state).last_ut.is_finite() as i32 != 0 {
        (*in_0).ut - (*state).last_ut
    } else {
        0.0f64
    };
    if !(dt > 0.0f64) || dt > 5.0f64 {
        dt = 0.0f64;
    }
    (*state).last_ut = (*in_0).ut;
    let mut p: *const PlanetModel = (*in_0).planet;
    let mut veh: *const VehicleProfile = (*in_0).vehicle;
    let mut v: ::core::ffi::c_double = (*in_0).relative_velocity;
    let mut gamma: ::core::ffi::c_double = (*in_0).flight_path_angle_deg * DEG2RAD;
    let mut r: ::core::ffi::c_double = (*p).radius + (*in_0).altitude;
    let mut g: ::core::ffi::c_double = (*p).gravitational_parameter / (r * r);
    let mut drag: ::core::ffi::c_double = if (*in_0).measured_drag_accel.is_finite()
        as i32 != 0
    {
        fmax(0.0f64, (*in_0).measured_drag_accel)
    } else {
        0.0f64
    };
    let mut lift: ::core::ffi::c_double = if (*in_0).measured_lift_accel.is_finite()
        as i32 != 0
    {
        fmax(0.0f64, (*in_0).measured_lift_accel)
    } else {
        0.0f64
    };
    let mut drag_valid: bool = drag >= (*c).minimum_drag_accel;
    out.scale_height_m = entry_energy_scale_height(p, (*in_0).altitude);
    out.lift_to_drag = if drag_valid as ::core::ffi::c_int != 0 {
        lift / drag
    } else {
        fmax(0.05f64, (*veh).estimated_lift_to_drag)
    };
    let mut beta: ::core::ffi::c_double = if drag_valid as ::core::ffi::c_int != 0
        && (*in_0).dynamic_pressure > 0.0f64
    {
        (*in_0).dynamic_pressure / drag
    } else {
        (*veh).estimated_ballistic_coefficient
    };
    if !(beta > 1.0f64) || beta.is_finite() as i32 == 0 {
        beta = fmax(1.0f64, (*veh).estimated_ballistic_coefficient);
    }
    let mut rho_t: ::core::ffi::c_double = planet_atmospheric_density(
        p,
        (*in_0).taem_altitude,
    );
    out.taem_drag_accel = fmax(
        (*c).minimum_drag_accel,
        0.5f64 * fmax(0.0f64, rho_t) * (*in_0).taem_velocity * (*in_0).taem_velocity
            / beta,
    );
    let mut by_q: ::core::ffi::c_double = (*veh).maximum_dynamic_pressure / beta;
    let mut by_load: ::core::ffi::c_double = (*veh).maximum_g_load * STANDARD_GRAVITY
        / fmax(0.05f64, out.lift_to_drag);
    out.maximum_drag_accel = fmax(
        out.taem_drag_accel,
        (*c).protection_fraction * fmin(by_q, by_load),
    );
    let mut remaining: ::core::ffi::c_double = entry_remaining_specific_energy(
        (*in_0).latitude,
        (*in_0).altitude,
        v,
        (*in_0).taem_latitude,
        (*in_0).taem_altitude,
        (*in_0).taem_velocity,
        p,
    );
    out.required_range_m = (*in_0).range_to_go;
    let mut lo: ::core::ffi::c_double = log((*c).minimum_drag_accel);
    let mut hi: ::core::ffi::c_double = log(out.maximum_drag_accel);
    let mut r_lo: ::core::ffi::c_double = predicted_range(
        in_0,
        c,
        exp(lo),
        out.taem_drag_accel,
        remaining,
    );
    let mut r_hi: ::core::ffi::c_double = predicted_range(
        in_0,
        c,
        exp(hi),
        out.taem_drag_accel,
        remaining,
    );
    let mut d0: ::core::ffi::c_double = 0.;
    if !((*in_0).range_to_go < r_lo) {
        d0 = exp(lo);
        out.profile_at_minimum = true_0 != 0;
    } else if !((*in_0).range_to_go > r_hi) {
        d0 = exp(hi);
        out.profile_at_maximum = true_0 != 0;
    } else {
        let mut i: ::core::ffi::c_int = 0 as ::core::ffi::c_int;
        while i < 48 as ::core::ffi::c_int {
            let mut mid: ::core::ffi::c_double = 0.5f64 * (lo + hi);
            let mut rm: ::core::ffi::c_double = predicted_range(
                in_0,
                c,
                exp(mid),
                out.taem_drag_accel,
                remaining,
            );
            if rm > (*in_0).range_to_go {
                lo = mid;
            } else {
                hi = mid;
            }
            i += 1;
        }
        d0 = exp(0.5f64 * (lo + hi));
    }
    out.profile_drag_accel = d0;
    out.predicted_range_m = predicted_range(in_0, c, d0, out.taem_drag_accel, remaining);
    out.range_error_m = out.predicted_range_m - out.required_range_m;
    out.reference_drag_accel = profile_drag(
        v,
        (*in_0).taem_velocity,
        (*in_0).transition_velocity,
        v,
        d0,
        out.taem_drag_accel,
    );
    let mut dv: ::core::ffi::c_double = fmax(1.0f64, 0.002f64 * v);
    let mut d_hi: ::core::ffi::c_double = profile_drag(
        v,
        (*in_0).taem_velocity,
        (*in_0).transition_velocity,
        v + dv,
        d0,
        out.taem_drag_accel,
    );
    let mut d_lo: ::core::ffi::c_double = profile_drag(
        v - dv,
        (*in_0).taem_velocity,
        (*in_0).transition_velocity,
        v,
        d0,
        out.taem_drag_accel,
    );
    out.reference_drag_dv = (d_hi - d_lo) / (2.0f64 * dv);
    let mut hs: ::core::ffi::c_double = out.scale_height_m;
    let mut vdot: ::core::ffi::c_double = -drag - g * sin(gamma);
    let mut dref: ::core::ffi::c_double = fmax(
        (*c).minimum_drag_accel,
        out.reference_drag_accel,
    );
    out.hdot_reference = -hs * (out.reference_drag_dv * vdot / dref - 2.0f64 * vdot / v);
    let mut eh: ::core::ffi::c_double = if drag_valid as ::core::ffi::c_int != 0 {
        hs * log(dref / drag)
    } else {
        (*c).altitude_error_limit_m
    };
    eh = fmin(fmax(eh, -(*c).altitude_error_limit_m), (*c).altitude_error_limit_m);
    out.altitude_error_m = eh;
    let mut w: ::core::ffi::c_double = (*c).natural_frequency_rad_s;
    let mut z: ::core::ffi::c_double = (*c).damping_ratio;
    let mut ki: ::core::ffi::c_double = (*c).integral_ratio * w * w * w;
    let mut hddot: ::core::ffi::c_double = -w * w * eh
        - 2.0f64 * z * w * ((*in_0).vertical_speed - out.hdot_reference)
        - ki * (*state).integral_m_s;
    let mut vh: ::core::ffi::c_double = v * cos(gamma);
    let mut cg: ::core::ffi::c_double = fmax(0.2f64, cos(gamma));
    let mut vertical: ::core::ffi::c_double = (hddot + g - vh * vh / r
        + drag * sin(gamma)) / cg;
    let mut bank_limit: ::core::ffi::c_double = fmin(
        fmax(0.0f64, (*in_0).maximum_bank_deg),
        89.0f64,
    );
    let mut load: ::core::ffi::c_double = lift / STANDARD_GRAVITY;
    let mut protect: bool = (*in_0).dynamic_pressure
        > (*c).protection_fraction * (*veh).maximum_dynamic_pressure
        || load > (*c).protection_fraction * (*veh).maximum_g_load;
    out.protection_active = protect;
    let mut bank: ::core::ffi::c_double = 0.;
    if !(lift >= (*c).minimum_lift_accel) || bank_limit <= 0.0f64 {
        out.degraded = !(lift >= (*c).minimum_lift_accel);
        bank = 0.0f64;
        out.saturated_lift_up = true_0 != 0;
    } else if protect {
        bank = 0.0f64;
        out.saturated_lift_up = true_0 != 0;
    } else {
        let mut ratio: ::core::ffi::c_double = vertical / lift;
        let mut cmin: ::core::ffi::c_double = cos(bank_limit * DEG2RAD);
        if ratio >= 1.0f64 {
            bank = 0.0f64;
            out.saturated_lift_up = true_0 != 0;
        } else if ratio <= cmin {
            bank = bank_limit;
            out.saturated_bank_limit = true_0 != 0;
        } else {
            bank = acos(ratio) * RAD2DEG;
        }
    }
    out.vertical_lift_command = vertical;
    out.bank_magnitude_deg = bank;
    let mut hold: bool = (*in_0).integrator_hold as ::core::ffi::c_int != 0
        || out.saturated_lift_up as ::core::ffi::c_int != 0
        || out.saturated_bank_limit as ::core::ffi::c_int != 0 || !drag_valid
        || protect as ::core::ffi::c_int != 0;
    if !hold && dt > 0.0f64 {
        (*state).integral_m_s += eh * dt;
        (*state).integral_m_s = fmin(
            fmax((*state).integral_m_s, -(*c).integral_limit_m_s),
            (*c).integral_limit_m_s,
        );
    }
    out.integral_m_s = (*state).integral_m_s;
    out.degraded = out.degraded as ::core::ffi::c_int != 0 || !drag_valid;
    out.valid = out.bank_magnitude_deg.is_finite() as i32 != 0
        && out.reference_drag_accel.is_finite() as i32 != 0;
    return out;
}
pub const true_0: ::core::ffi::c_int = 1 as ::core::ffi::c_int;
