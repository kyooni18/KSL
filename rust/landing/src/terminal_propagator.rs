use crate::sys::*;
extern "C" {
    fn attitude_set_command(
        a: *mut AttitudeModel,
        aoa_rad: ::core::ffi::c_double,
        bank_rad: ::core::ffi::c_double,
    );
    fn terminal_state_validate(
        state: *const TerminalDynamicState,
        reason: *mut ::core::ffi::c_char,
        reason_size: size_t,
    ) -> bool;
    fn ceil(__x: ::core::ffi::c_double) -> ::core::ffi::c_double;
    fn flight_physics_airborne_step(
        world: *const KerbinWorld,
        aero: *const AeroTable,
        input: *const FlightAirborneStepInput,
        state: *mut FlightAirborneState,
    );
}
pub type size_t = usize;
pub type __uint32_t = u32;
pub type __uint64_t = u64;
pub type uint32_t = __uint32_t;
pub type uint64_t = __uint64_t;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct LandingSite {
    pub name: [::core::ffi::c_char; 128],
    pub latitude: ::core::ffi::c_double,
    pub longitude: ::core::ffi::c_double,
    pub altitude: ::core::ffi::c_double,
    pub runway_heading: ::core::ffi::c_double,
    pub runway_length: ::core::ffi::c_double,
    pub runway_width: ::core::ffi::c_double,
    pub allow_reciprocal_runway: bool,
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
pub struct GuidanceSettings {
    pub target_deorbit_capture_radius: ::core::ffi::c_double,
    pub entry_interface_altitude_margin: ::core::ffi::c_double,
    pub target_entry_range: ::core::ffi::c_double,
    pub entry_range_reference_speed: ::core::ffi::c_double,
    pub target_entry_flight_path_angle: ::core::ffi::c_double,
    pub maximum_entry_flight_path_angle: ::core::ffi::c_double,
    pub target_post_burn_periapsis_altitude: ::core::ffi::c_double,
    pub deorbit_maximum_throttle: ::core::ffi::c_double,
    pub deorbit_throttle_ramp_duration: ::core::ffi::c_double,
    pub taem_interface_altitude: ::core::ffi::c_double,
    pub taem_interface_range: ::core::ffi::c_double,
    pub taem_force_handoff_speed: ::core::ffi::c_double,
    pub mm304_handoff_radius: ::core::ffi::c_double,
    pub mm304_perpendicular_heading_half_width: ::core::ffi::c_double,
    pub mm304_handoff_along_track: ::core::ffi::c_double,
    pub mm304_handoff_cross_track: ::core::ffi::c_double,
    pub mm305_min_altitude: ::core::ffi::c_double,
    pub mm305_max_altitude: ::core::ffi::c_double,
    pub mm305_target_mach: ::core::ffi::c_double,
    pub mm305_mach_half_width: ::core::ffi::c_double,
    pub hac_acquisition_altitude: ::core::ffi::c_double,
    pub hac_acquisition_mach: ::core::ffi::c_double,
    pub deorbit_timing_uncertainty: ::core::ffi::c_double,
    pub deorbit_thrust_uncertainty_fraction: ::core::ffi::c_double,
    pub deorbit_delta_v_uncertainty: ::core::ffi::c_double,
    pub deorbit_mass_uncertainty_fraction: ::core::ffi::c_double,
    pub deorbit_position_uncertainty: ::core::ffi::c_double,
    pub deorbit_velocity_uncertainty: ::core::ffi::c_double,
    pub deorbit_pointing_uncertainty: ::core::ffi::c_double,
    pub deorbit_atmosphere_uncertainty_fraction: ::core::ffi::c_double,
    pub deorbit_robust_minimum_pass_fraction: ::core::ffi::c_double,
    pub hac_radius: ::core::ffi::c_double,
    pub hac_look_ahead_angle: ::core::ffi::c_double,
    pub final_approach_distance: ::core::ffi::c_double,
    pub final_alignment_speed: ::core::ffi::c_double,
    pub final_glide_slope: ::core::ffi::c_double,
    pub taem_glide_slope: ::core::ffi::c_double,
    pub gear_deployment_altitude: ::core::ffi::c_double,
    pub flare_altitude: ::core::ffi::c_double,
    pub touchdown_sink_rate: ::core::ffi::c_double,
    pub guidance_rate: ::core::ffi::c_double,
    pub prediction_interval: ::core::ffi::c_double,
    pub s_turn_minimum_leg_duration: ::core::ffi::c_double,
    pub entry_roll_rate: ::core::ffi::c_double,
    pub entry_roll_acceleration: ::core::ffi::c_double,
    pub taem_roll_rate: ::core::ffi::c_double,
    pub approach_roll_rate: ::core::ffi::c_double,
    pub use_time_warp: bool,
    pub taem_energy_law: bool,
    pub sts_trajectory_delivery: bool,
    pub minimum_planning_lead_time: ::core::ffi::c_double,
    pub taem_hac_law: bool,
    pub terminal_speedbrake: bool,
    pub adaptive_flare: bool,
    pub touchdown_aim_along: ::core::ffi::c_double,
    pub rollout_parachutes: bool,
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
pub struct TerminalModel {
    pub schema_version: uint32_t,
    pub snapshot_id: uint64_t,
    pub atmosphere_source_digest: uint64_t,
    pub aero_source_digest: uint64_t,
    pub aero_book_source_digest: uint64_t,
    pub attitude_source_digest: uint64_t,
    pub replay_validated: bool,
    pub planet_frame_aligned: bool,
    pub world: KerbinWorld,
    pub aero: AeroTable,
    pub attitude: AttitudeModel,
    pub vehicle: VehicleProfile,
    pub site: LandingSite,
    pub guidance: GuidanceSettings,
}
#[derive(Copy, Clone)]
#[repr(C)]
pub struct TerminalDynamicState {
    pub position_i_m: Vec3,
    pub velocity_i_mps: Vec3,
    pub attitude: AttitudeModel,
    pub mass_kg: ::core::ffi::c_double,
    pub ut_s: ::core::ffi::c_double,
    pub trim_aoa_ceiling_rad: ::core::ffi::c_double,
}
#[derive(Copy, Clone)]
#[repr(C)]
pub struct TerminalControl {
    pub angle_of_attack_rad: ::core::ffi::c_double,
    pub bank_rad: ::core::ffi::c_double,
    pub dt_s: ::core::ffi::c_double,
}
pub type TerminalStepStatus = ::core::ffi::c_uint;
pub const TERMINAL_STEP_INVALID_RESULT: TerminalStepStatus = 2;
pub const TERMINAL_STEP_INVALID_INPUT: TerminalStepStatus = 1;
pub const TERMINAL_STEP_OK: TerminalStepStatus = 0;
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
pub const NULL: *mut ::core::ffi::c_void = ::core::ptr::null_mut::<
    ::core::ffi::c_void,
>();
pub const TERMINAL_PHYSICS_MAX_DT_S: ::core::ffi::c_double = 0.02f64;
pub const TERMINAL_MAX_CONTROL_HOLD_S: ::core::ffi::c_double = 0.5f64;
unsafe extern "C" fn finite_vec3(mut v: Vec3) -> bool {
    return v.x.is_finite() as i32 != 0 && v.y.is_finite() as i32 != 0
        && v.z.is_finite() as i32 != 0;
}
#[no_mangle]
pub unsafe extern "C" fn terminal_propagator_step(
    mut m: *const TerminalModel,
    mut s: *mut TerminalDynamicState,
    mut u: *const TerminalControl,
) -> TerminalStepStatus {
    if m.is_null() || s.is_null() || u.is_null() || !((*u).dt_s > 0.0f64)
        || (*u).dt_s.is_finite() as i32 == 0 || (*u).dt_s > TERMINAL_MAX_CONTROL_HOLD_S
        || (*u).angle_of_attack_rad.is_finite() as i32 == 0
        || (*u).bank_rad.is_finite() as i32 == 0 || !((*s).mass_kg > 0.0f64)
        || (*s).mass_kg.is_finite() as i32 == 0 || (*s).ut_s.is_finite() as i32 == 0
        || !finite_vec3((*s).position_i_m) || !finite_vec3((*s).velocity_i_mps)
    {
        return TERMINAL_STEP_INVALID_INPUT;
    }
    if !(*m).replay_validated
        || !terminal_state_validate(
            s,
            ::core::ptr::null_mut::<::core::ffi::c_char>(),
            0 as size_t,
        )
    {
        return TERMINAL_STEP_INVALID_INPUT;
    }
    let mut plant: FlightAirborneState = FlightAirborneState {
        position_i_m: (*s).position_i_m,
        velocity_i_mps: (*s).velocity_i_mps,
        attitude: (*s).attitude,
        body_q_i: Quat {
            w: 1.0f64,
            x: 0.0f64,
            y: 0.0f64,
            z: 0.0f64,
        },
        aero: AeroForces {
            lift_n: 0 as ::core::ffi::c_int as ::core::ffi::c_double,
            drag_n: 0.,
            side_n: 0.,
            force_i: Vec3 { x: 0., y: 0., z: 0. },
            mach: 0.,
            dynamic_pressure_pa: 0.,
            airspeed_mps: 0.,
        },
    };
    attitude_set_command(
        &raw mut plant.attitude,
        (*u).angle_of_attack_rad,
        (*u).bank_rad,
    );
    let mut count_d: ::core::ffi::c_double = ceil((*u).dt_s / TERMINAL_PHYSICS_MAX_DT_S);
    if count_d.is_finite() as i32 == 0 || count_d < 1.0f64 || count_d > 25.0f64 {
        return TERMINAL_STEP_INVALID_INPUT;
    }
    let mut count: size_t = count_d as size_t;
    let mut step_dt: ::core::ffi::c_double = (*u).dt_s / count as ::core::ffi::c_double;
    let mut i: size_t = 0 as size_t;
    while i < count {
        let mut input: FlightAirborneStepInput = FlightAirborneStepInput {
            mass_kg: (*s).mass_kg,
            ut: (*s).ut_s + i as ::core::ffi::c_double * step_dt,
            dt_s: step_dt,
            burn_accel_mps2: 0.0f64,
            engine_thrust_n: 0.0f64,
            engine_thrust_dir_i: Vec3 { x: 0., y: 0., z: 0. },
        };
        flight_physics_airborne_step(
            &raw const (*m).world,
            &raw const (*m).aero,
            &raw mut input,
            &raw mut plant,
        );
        if !finite_vec3(plant.position_i_m) || !finite_vec3(plant.velocity_i_mps)
            || plant.attitude.aoa_rad.is_finite() as i32 == 0
            || plant.attitude.bank_rad.is_finite() as i32 == 0
        {
            return TERMINAL_STEP_INVALID_RESULT;
        }
        i = i.wrapping_add(1);
    }
    (*s).position_i_m = plant.position_i_m;
    (*s).velocity_i_mps = plant.velocity_i_mps;
    (*s).attitude = plant.attitude;
    (*s).ut_s += (*u).dt_s;
    return (if (*s).ut_s.is_finite() as i32 != 0 {
        TERMINAL_STEP_OK as ::core::ffi::c_int
    } else {
        TERMINAL_STEP_INVALID_RESULT as ::core::ffi::c_int
    }) as TerminalStepStatus;
}
