use crate::sys::*;
extern "C" {
    fn world_gravity_accel(w: *const KerbinWorld, position_i: Vec3) -> Vec3;
    fn world_atmosphere_velocity_i(w: *const KerbinWorld, position_i: Vec3) -> Vec3;
    fn world_atmosphere_sample_state(
        w: *const KerbinWorld,
        position_i: Vec3,
        ut: ::core::ffi::c_double,
    ) -> AtmosphereSample;
    fn aero_compute(
        w: *const KerbinWorld,
        a: *const AeroTable,
        position_i: Vec3,
        velocity_i: Vec3,
        ut: ::core::ffi::c_double,
        mass_kg: ::core::ffi::c_double,
        aoa_rad: ::core::ffi::c_double,
        bank_rad: ::core::ffi::c_double,
    ) -> AeroForces;
    fn attitude_step(
        a: *mut AttitudeModel,
        dynamic_pressure_pa: ::core::ffi::c_double,
        dt: ::core::ffi::c_double,
    );
    fn attitude_step_body(
        a: *mut AttitudeModel,
        body_q_i: *mut Quat,
        position_i: Vec3,
        air_velocity_i: Vec3,
        dynamic_pressure_pa: ::core::ffi::c_double,
        dt: ::core::ffi::c_double,
    ) -> bool;
    fn attitude_body_quat(
        position_i: Vec3,
        air_velocity_i: Vec3,
        aoa_rad: ::core::ffi::c_double,
        bank_rad: ::core::ffi::c_double,
    ) -> Quat;
    fn attitude_body_quat_wind(
        position_i: Vec3,
        air_velocity_i: Vec3,
        aoa_rad: ::core::ffi::c_double,
        bank_rad: ::core::ffi::c_double,
    ) -> Quat;
    fn v3_add(a: Vec3, b: Vec3) -> Vec3;
    fn v3_sub(a: Vec3, b: Vec3) -> Vec3;
    fn v3_scale(a: Vec3, s: ::core::ffi::c_double) -> Vec3;
    fn v3_norm(a: Vec3) -> ::core::ffi::c_double;
    fn v3_normalized(a: Vec3) -> Vec3;
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
pub struct AtmosphereSample {
    pub density_kg_m3: ::core::ffi::c_double,
    pub pressure_pa: ::core::ffi::c_double,
    pub temperature_k: ::core::ffi::c_double,
    pub speed_of_sound_mps: ::core::ffi::c_double,
}
#[derive(Copy, Clone)]
#[repr(C)]
pub struct AeroForces {
    pub lift_n: ::core::ffi::c_double,
    pub drag_n: ::core::ffi::c_double,
    pub side_n: ::core::ffi::c_double,
    pub force_i: Vec3,
    pub mach: ::core::ffi::c_double,
    pub dynamic_pressure_pa: ::core::ffi::c_double,
    pub airspeed_mps: ::core::ffi::c_double,
}
#[derive(Copy, Clone)]
#[repr(C)]
pub struct AtmospherePoint {
    pub altitude_m: ::core::ffi::c_double,
    pub sample: AtmosphereSample,
}
#[derive(Copy, Clone)]
#[repr(C)]
pub struct AtmosphereTable {
    pub p: [AtmospherePoint; 2048],
    pub count: size_t,
}
#[derive(Copy, Clone)]
#[repr(C)]
pub struct KerbinWorld {
    pub radius_m: ::core::ffi::c_double,
    pub mu_m3_s2: ::core::ffi::c_double,
    pub rotation_rate_rad_s: ::core::ffi::c_double,
    pub atmosphere_top_m: ::core::ffi::c_double,
    pub rotation_phase_rad_at_ut0: ::core::ffi::c_double,
    pub orbital_rate_rad_s: ::core::ffi::c_double,
    pub solar_phase_rad_at_ut0: ::core::ffi::c_double,
    pub stock_spatial_atmosphere: bool,
    pub atmosphere: AtmosphereTable,
}
#[derive(Copy, Clone)]
#[repr(C)]
pub struct AeroBookPoint {
    pub q_pa: ::core::ffi::c_double,
    pub mach: ::core::ffi::c_double,
    pub alpha_deg: ::core::ffi::c_double,
    pub lift_per_q_m2: ::core::ffi::c_double,
    pub drag_per_q_m2: ::core::ffi::c_double,
    pub support: ::core::ffi::c_double,
}
#[derive(Copy, Clone)]
#[repr(C)]
pub struct AeroTable {
    pub mach: [::core::ffi::c_double; 40],
    pub alpha_deg: [::core::ffi::c_double; 40],
    pub cl: [[::core::ffi::c_double; 40]; 40],
    pub cd: [[::core::ffi::c_double; 40]; 40],
    pub mach_count: size_t,
    pub alpha_count: size_t,
    pub reference_area_m2: ::core::ffi::c_double,
    pub book: [AeroBookPoint; 2048],
    pub book_count: size_t,
    pub book_enabled: bool,
    pub lift_scale: ::core::ffi::c_double,
    pub drag_scale: ::core::ffi::c_double,
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
#[derive(Copy, Clone)]
#[repr(C)]
pub struct FlightAirborneState {
    pub position_i_m: Vec3,
    pub velocity_i_mps: Vec3,
    pub attitude: AttitudeModel,
    pub body_q_i: Quat,
    pub aero: AeroForces,
}
#[derive(Copy, Clone)]
#[repr(C)]
pub struct FlightAirborneStepInput {
    pub mass_kg: ::core::ffi::c_double,
    pub ut: ::core::ffi::c_double,
    pub dt_s: ::core::ffi::c_double,
    pub burn_accel_mps2: ::core::ffi::c_double,
    pub engine_thrust_n: ::core::ffi::c_double,
    pub engine_thrust_dir_i: Vec3,
}
#[no_mangle]
pub unsafe extern "C" fn flight_physics_airborne_step(
    mut world: *const KerbinWorld,
    mut aero: *const AeroTable,
    mut input: *const FlightAirborneStepInput,
    mut state: *mut FlightAirborneState,
) {
    if world.is_null() || aero.is_null() || input.is_null() || state.is_null() {
        return;
    }
    let mut p: Vec3 = (*state).position_i_m;
    let mut v: Vec3 = (*state).velocity_i_mps;
    let mut ut: ::core::ffi::c_double = (*input).ut;
    let mut dt: ::core::ffi::c_double = (*input).dt_s;
    let mut attitude_atm: AtmosphereSample = world_atmosphere_sample_state(world, p, ut);
    let mut attitude_vair: Vec3 = v3_sub(v, world_atmosphere_velocity_i(world, p));
    let mut attitude_speed: ::core::ffi::c_double = v3_norm(attitude_vair);
    let mut attitude_q: ::core::ffi::c_double = 0.5f64 * attitude_atm.density_kg_m3
        * attitude_speed * attitude_speed;
    if (*state).attitude.direct as ::core::ffi::c_int != 0
        && (*state).attitude.direct_body_coordinates as ::core::ffi::c_int != 0
        && !(*state).attitude.body_rates_initialized
        && attitude_q >= (*state).attitude.direct_min_q_pa
    {
        (*state).body_q_i = attitude_body_quat_wind(
            p,
            attitude_vair,
            (*state).attitude.aoa_rad,
            (*state).attitude.bank_rad,
        );
    }
    let mut applied_aero: AeroForces = AeroForces {
        lift_n: 0.,
        drag_n: 0.,
        side_n: 0.,
        force_i: Vec3 { x: 0., y: 0., z: 0. },
        mach: 0.,
        dynamic_pressure_pa: 0.,
        airspeed_mps: 0.,
    };
    applied_aero = aero_compute(
        world,
        aero,
        p,
        v,
        ut,
        (*input).mass_kg,
        (*state).attitude.aoa_rad,
        (*state).attitude.bank_rad,
    );
    let mut accel: Vec3 = v3_add(
        world_gravity_accel(world, p),
        v3_scale(applied_aero.force_i, 1.0f64 / (*input).mass_kg),
    );
    if (*input).burn_accel_mps2 > 0.0f64 {
        let mut retro: Vec3 = v3_scale(v3_normalized(v), -(*input).burn_accel_mps2);
        accel = v3_add(accel, retro);
    }
    if (*input).engine_thrust_n > 0.0f64 && (*input).mass_kg > 0.0f64 {
        let mut axis: Vec3 = (*input).engine_thrust_dir_i;
        let mut dir: Vec3 = if v3_norm(axis) > 0.5f64 {
            v3_normalized(axis)
        } else {
            v3_scale(v3_normalized(v), -1.0f64)
        };
        accel = v3_add(
            accel,
            v3_scale(dir, (*input).engine_thrust_n / (*input).mass_kg),
        );
    }
    (*state).velocity_i_mps = v3_add(v, v3_scale(accel, dt));
    (*state).position_i_m = v3_add(p, v3_scale((*state).velocity_i_mps, dt));
    let mut vair: Vec3 = v3_sub(
        (*state).velocity_i_mps,
        world_atmosphere_velocity_i(world, (*state).position_i_m),
    );
    let mut body_step: bool = attitude_step_body(
        &raw mut (*state).attitude,
        &raw mut (*state).body_q_i,
        (*state).position_i_m,
        vair,
        attitude_q,
        dt,
    );
    if !body_step {
        (*state).attitude.body_rates_initialized = false_0 != 0;
        attitude_step(&raw mut (*state).attitude, attitude_q, dt);
        (*state).body_q_i = attitude_body_quat(
            (*state).position_i_m,
            vair,
            (*state).attitude.aoa_rad,
            (*state).attitude.bank_rad,
        );
    }
    (*state).aero = aero_compute(
        world,
        aero,
        (*state).position_i_m,
        (*state).velocity_i_mps,
        ut + dt,
        (*input).mass_kg,
        (*state).attitude.aoa_rad,
        (*state).attitude.bank_rad,
    );
}
pub const false_0: ::core::ffi::c_int = 0 as ::core::ffi::c_int;
