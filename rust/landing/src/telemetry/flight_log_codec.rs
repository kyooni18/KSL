use crate::sys::*;
extern "C" {
    fn jw_raw(w: *mut JsonWriter, text: *const ::core::ffi::c_char);
    fn jw_char(w: *mut JsonWriter, c: ::core::ffi::c_char);
    fn jw_string(w: *mut JsonWriter, text: *const ::core::ffi::c_char);
    fn jw_number(w: *mut JsonWriter, value: ::core::ffi::c_double);
    fn jw_integer(w: *mut JsonWriter, value: ::core::ffi::c_longlong);
    fn jw_bool(w: *mut JsonWriter, value: bool);
    fn jw_null(w: *mut JsonWriter);
    fn fabs(__x: ::core::ffi::c_double) -> ::core::ffi::c_double;
    fn fmax(
        __x: ::core::ffi::c_double,
        __y: ::core::ffi::c_double,
    ) -> ::core::ffi::c_double;
}
pub type size_t = usize;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct JsonWriter {
    pub data: *mut ::core::ffi::c_char,
    pub length: size_t,
    pub capacity: size_t,
    pub failed: bool,
}
pub type __uint32_t = u32;
pub type __uint64_t = u64;
pub type uint32_t = __uint32_t;
pub type uint64_t = __uint64_t;
pub type C2RustUnnamed = ::core::ffi::c_uint;
pub const VEHICLE_LOG_GROUP_ALL: C2RustUnnamed = 4095;
pub const VEHICLE_LOG_GROUP_SURFACES: C2RustUnnamed = 2048;
pub const VEHICLE_LOG_GROUP_PHYSICS: C2RustUnnamed = 1024;
pub const VEHICLE_LOG_GROUP_RUNTIME: C2RustUnnamed = 512;
pub const VEHICLE_LOG_GROUP_GUIDANCE: C2RustUnnamed = 256;
pub const VEHICLE_LOG_GROUP_CONTROL: C2RustUnnamed = 128;
pub const VEHICLE_LOG_GROUP_COMMAND: C2RustUnnamed = 64;
pub const VEHICLE_LOG_GROUP_VEHICLE: C2RustUnnamed = 32;
pub const VEHICLE_LOG_GROUP_AERO: C2RustUnnamed = 16;
pub const VEHICLE_LOG_GROUP_ATTITUDE: C2RustUnnamed = 8;
pub const VEHICLE_LOG_GROUP_MOTION: C2RustUnnamed = 4;
pub const VEHICLE_LOG_GROUP_POSITION: C2RustUnnamed = 2;
pub const VEHICLE_LOG_GROUP_STATE: C2RustUnnamed = 1;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct VehicleLogSurfaceSample {
    pub part_name: [::core::ffi::c_char; 64],
    pub role: [::core::ffi::c_char; 16],
    pub deployed: bool,
    pub deflection_override: bool,
    pub override_deflection: ::core::ffi::c_double,
}
#[derive(Copy, Clone)]
#[repr(C)]
pub struct VehicleLogSample {
    pub tick_sequence: uint64_t,
    pub wall_monotonic_seconds: ::core::ffi::c_double,
    pub ut: ::core::ffi::c_double,
    pub latitude: ::core::ffi::c_double,
    pub longitude: ::core::ffi::c_double,
    pub altitude: ::core::ffi::c_double,
    pub radar_altitude: ::core::ffi::c_double,
    pub position: [::core::ffi::c_double; 3],
    pub velocity: [::core::ffi::c_double; 3],
    pub true_air_speed: ::core::ffi::c_double,
    pub horizontal_speed: ::core::ffi::c_double,
    pub vertical_speed: ::core::ffi::c_double,
    pub flight_path_angle: ::core::ffi::c_double,
    pub pitch: ::core::ffi::c_double,
    pub roll: ::core::ffi::c_double,
    pub heading: ::core::ffi::c_double,
    pub angle_of_attack: ::core::ffi::c_double,
    pub sideslip: ::core::ffi::c_double,
    pub pitch_rate: ::core::ffi::c_double,
    pub roll_rate: ::core::ffi::c_double,
    pub yaw_rate: ::core::ffi::c_double,
    pub course_rate: ::core::ffi::c_double,
    pub coordinate_pitch_rate: ::core::ffi::c_double,
    pub coordinate_roll_rate: ::core::ffi::c_double,
    pub heading_rate: ::core::ffi::c_double,
    pub angle_of_attack_rate: ::core::ffi::c_double,
    pub body_pitch_rate: ::core::ffi::c_double,
    pub body_roll_rate: ::core::ffi::c_double,
    pub body_yaw_rate: ::core::ffi::c_double,
    pub dynamic_pressure: ::core::ffi::c_double,
    pub static_pressure: ::core::ffi::c_double,
    pub atmospheric_density: ::core::ffi::c_double,
    pub mach: ::core::ffi::c_double,
    pub stall_fraction: ::core::ffi::c_double,
    pub stall_fraction_is_measured: bool,
    pub lift_force: ::core::ffi::c_double,
    pub drag_force: ::core::ffi::c_double,
    pub g_force: ::core::ffi::c_double,
    pub energy_excess_range: ::core::ffi::c_double,
    pub has_force_vectors: bool,
    pub lift_vector: [::core::ffi::c_double; 3],
    pub drag_vector: [::core::ffi::c_double; 3],
    pub aero_acceleration: [::core::ffi::c_double; 3],
    pub physics_confidence: ::core::ffi::c_double,
    pub physics_model_residual: ::core::ffi::c_double,
    pub physics_model_residual_confidence: ::core::ffi::c_double,
    pub physics_certified_uncertainty: ::core::ffi::c_double,
    pub physics_force_residual_per_q: [::core::ffi::c_double; 3],
    pub physics_force_residual_sigma_per_q: [::core::ffi::c_double; 3],
    pub physics_force_residual_confidence: ::core::ffi::c_double,
    pub physics_airbrake_model_available: bool,
    pub physics_airbrake_model_confidence: ::core::ffi::c_double,
    pub physics_airbrake_drag_accel: ::core::ffi::c_double,
    pub physics_samples: ::core::ffi::c_uint,
    pub physics_live_samples: ::core::ffi::c_uint,
    pub mass: ::core::ffi::c_double,
    pub available_thrust: ::core::ffi::c_double,
    pub current_thrust: ::core::ffi::c_double,
    pub has_dry_mass: bool,
    pub dry_mass: ::core::ffi::c_double,
    pub has_center_of_mass: bool,
    pub center_of_mass: [::core::ffi::c_double; 3],
    pub has_center_of_mass_root: bool,
    pub center_of_mass_root: [::core::ffi::c_double; 3],
    pub has_torque: bool,
    pub available_torque: [::core::ffi::c_double; 3],
    pub has_inertia: bool,
    pub moment_of_inertia: [::core::ffi::c_double; 3],
    pub target_pitch: ::core::ffi::c_double,
    pub target_heading: ::core::ffi::c_double,
    pub target_roll: ::core::ffi::c_double,
    pub target_throttle: ::core::ffi::c_double,
    pub has_target_aoa: bool,
    pub target_aoa: ::core::ffi::c_double,
    pub target_gear: bool,
    pub target_brakes: bool,
    pub target_airbrakes: bool,
    pub gear: bool,
    pub brakes: bool,
    pub has_airbrakes: bool,
    pub airbrakes: bool,
    pub has_sas_state: bool,
    pub sas_enabled: bool,
    pub has_control_state: bool,
    pub control_state_pitch: ::core::ffi::c_double,
    pub control_state_roll: ::core::ffi::c_double,
    pub control_state_yaw: ::core::ffi::c_double,
    pub control_state_throttle: ::core::ffi::c_double,
    pub command_applied: bool,
    pub has_actuator_feedback: bool,
    pub has_control_diagnostics: bool,
    pub control_pitch: ::core::ffi::c_double,
    pub control_roll: ::core::ffi::c_double,
    pub control_yaw: ::core::ffi::c_double,
    pub control_pitch_error: ::core::ffi::c_double,
    pub control_pitch_trim: ::core::ffi::c_double,
    pub control_pitch_authority: ::core::ffi::c_double,
    pub control_pitch_aero_fraction: ::core::ffi::c_double,
    pub has_control_surface_state: bool,
    pub control_surface_state_ut: ::core::ffi::c_double,
    pub control_surface_count: size_t,
    pub control_surfaces: [VehicleLogSurfaceSample; 16],
    pub has_loop_wall_delta: bool,
    pub has_telemetry_latency: bool,
    pub has_guidance_compute: bool,
    pub has_apply_latency: bool,
    pub has_control_loop: bool,
    pub has_warp: bool,
    pub warp_rate: ::core::ffi::c_double,
    pub loop_wall_delta_ms: ::core::ffi::c_double,
    pub telemetry_latency_ms: ::core::ffi::c_double,
    pub guidance_compute_ms: ::core::ffi::c_double,
    pub apply_latency_ms: ::core::ffi::c_double,
    pub control_loop_ms: ::core::ffi::c_double,
    pub automation_engaged: bool,
    pub paused: bool,
    pub constraint_active: bool,
    pub phase: [::core::ffi::c_char; 40],
    pub control_profile: [::core::ffi::c_char; 32],
    pub vessel_situation: [::core::ffi::c_char; 64],
    pub has_plan_identity: bool,
    pub plan_id: uint64_t,
    pub plan_version: uint64_t,
    pub parent_plan_id: uint64_t,
    pub parent_plan_version: uint64_t,
    pub range_to_site: ::core::ffi::c_double,
    pub bearing_to_site: ::core::ffi::c_double,
    pub runway_along_track: ::core::ffi::c_double,
    pub runway_cross_track: ::core::ffi::c_double,
    pub specific_mechanical_energy: ::core::ffi::c_double,
    pub entry_plan_valid: bool,
    pub entry_reversal_scheduled: bool,
    pub entry_reversal_time_remaining: ::core::ffi::c_double,
    pub entry_reversal_ut: ::core::ffi::c_double,
    pub entry_reversal_range: ::core::ffi::c_double,
    pub entry_reversal_sign: ::core::ffi::c_double,
    pub event: [::core::ffi::c_char; 512],
}
#[derive(Copy, Clone)]
#[repr(C)]
pub struct VehicleLogDecision {
    pub emit: bool,
    pub keyframe: bool,
    pub durable: bool,
    pub groups: uint32_t,
}
#[derive(Copy, Clone)]
#[repr(C)]
pub struct VehicleLogEncoder {
    pub has_last: bool,
    pub last: VehicleLogSample,
    pub last_emit_wall: ::core::ffi::c_double,
    pub last_keyframe_wall: ::core::ffi::c_double,
}
pub const LANDING_CONTROL_SURFACE_MAX: ::core::ffi::c_int = 16 as ::core::ffi::c_int;
pub const true_0: ::core::ffi::c_int = 1 as ::core::ffi::c_int;
pub const false_0: ::core::ffi::c_int = 0 as ::core::ffi::c_int;
unsafe extern "C" fn changed_number(
    mut a: ::core::ffi::c_double,
    mut b: ::core::ffi::c_double,
    mut absolute_deadband: ::core::ffi::c_double,
    mut relative_deadband: ::core::ffi::c_double,
) -> bool {
    if a.is_nan() as i32 != 0 && b.is_nan() as i32 != 0 {
        return false_0 != 0;
    }
    if a.is_finite() as i32 == 0 || b.is_finite() as i32 == 0 {
        return a != b;
    }
    let mut delta: ::core::ffi::c_double = fabs(a - b);
    let mut scale: ::core::ffi::c_double = fmax(fabs(a), fabs(b));
    return delta > absolute_deadband + relative_deadband * scale;
}
unsafe extern "C" fn changed_vector(
    mut a: *const ::core::ffi::c_double,
    mut b: *const ::core::ffi::c_double,
    mut absolute_deadband: ::core::ffi::c_double,
    mut relative_deadband: ::core::ffi::c_double,
) -> bool {
    let mut i: ::core::ffi::c_int = 0 as ::core::ffi::c_int;
    while i < 3 as ::core::ffi::c_int {
        if changed_number(
            *a.offset(i as isize),
            *b.offset(i as isize),
            absolute_deadband,
            relative_deadband,
        ) {
            return true_0 != 0;
        }
        i += 1;
    }
    return false_0 != 0;
}
unsafe extern "C" fn changed_groups(
    mut a: *const VehicleLogSample,
    mut b: *const VehicleLogSample,
) -> uint32_t {
    let mut mask: uint32_t = 0 as uint32_t;
    if strcmp(
        &raw const (*a).phase as *const ::core::ffi::c_char,
        &raw const (*b).phase as *const ::core::ffi::c_char,
    ) != 0
        || strcmp(
            &raw const (*a).control_profile as *const ::core::ffi::c_char,
            &raw const (*b).control_profile as *const ::core::ffi::c_char,
        ) != 0
        || strcmp(
            &raw const (*a).vessel_situation as *const ::core::ffi::c_char,
            &raw const (*b).vessel_situation as *const ::core::ffi::c_char,
        ) != 0
        || (*a).automation_engaged as ::core::ffi::c_int
            != (*b).automation_engaged as ::core::ffi::c_int
        || (*a).paused as ::core::ffi::c_int != (*b).paused as ::core::ffi::c_int
        || (*a).constraint_active as ::core::ffi::c_int
            != (*b).constraint_active as ::core::ffi::c_int
        || (*a).gear as ::core::ffi::c_int != (*b).gear as ::core::ffi::c_int
        || (*a).brakes as ::core::ffi::c_int != (*b).brakes as ::core::ffi::c_int
        || (*a).has_airbrakes as ::core::ffi::c_int
            != (*b).has_airbrakes as ::core::ffi::c_int
        || (*a).airbrakes as ::core::ffi::c_int != (*b).airbrakes as ::core::ffi::c_int
        || (*a).has_sas_state as ::core::ffi::c_int
            != (*b).has_sas_state as ::core::ffi::c_int
        || (*a).sas_enabled as ::core::ffi::c_int
            != (*b).sas_enabled as ::core::ffi::c_int
        || (*a).entry_plan_valid as ::core::ffi::c_int
            != (*b).entry_plan_valid as ::core::ffi::c_int
        || (*a).entry_reversal_scheduled as ::core::ffi::c_int
            != (*b).entry_reversal_scheduled as ::core::ffi::c_int
        || (*a).has_plan_identity as ::core::ffi::c_int
            != (*b).has_plan_identity as ::core::ffi::c_int
        || (*a).plan_id != (*b).plan_id || (*a).plan_version != (*b).plan_version
        || (*a).parent_plan_id != (*b).parent_plan_id
        || (*a).parent_plan_version != (*b).parent_plan_version
    {
        mask |= VEHICLE_LOG_GROUP_STATE as ::core::ffi::c_int as uint32_t;
    }
    if changed_number(
        (*a).latitude,
        (*b).latitude,
        1e-6f64,
        0 as ::core::ffi::c_int as ::core::ffi::c_double,
    ) as ::core::ffi::c_int != 0
        || changed_number(
            (*a).longitude,
            (*b).longitude,
            1e-6f64,
            0 as ::core::ffi::c_int as ::core::ffi::c_double,
        ) as ::core::ffi::c_int != 0
        || changed_number((*a).altitude, (*b).altitude, 0.25f64, 1e-6f64)
            as ::core::ffi::c_int != 0
        || changed_number((*a).radar_altitude, (*b).radar_altitude, 0.25f64, 1e-6f64)
            as ::core::ffi::c_int != 0
        || changed_vector(
            &raw const (*a).position as *const ::core::ffi::c_double,
            &raw const (*b).position as *const ::core::ffi::c_double,
            0.25f64,
            1e-8f64,
        ) as ::core::ffi::c_int != 0
    {
        mask |= VEHICLE_LOG_GROUP_POSITION as ::core::ffi::c_int as uint32_t;
    }
    if changed_vector(
        &raw const (*a).velocity as *const ::core::ffi::c_double,
        &raw const (*b).velocity as *const ::core::ffi::c_double,
        0.15f64,
        1e-5f64,
    ) as ::core::ffi::c_int != 0
        || changed_number((*a).true_air_speed, (*b).true_air_speed, 0.20f64, 1e-4f64)
            as ::core::ffi::c_int != 0
        || changed_number((*a).horizontal_speed, (*b).horizontal_speed, 0.15f64, 1e-4f64)
            as ::core::ffi::c_int != 0
        || changed_number((*a).vertical_speed, (*b).vertical_speed, 0.10f64, 1e-4f64)
            as ::core::ffi::c_int != 0
        || changed_number(
            (*a).flight_path_angle,
            (*b).flight_path_angle,
            0.03f64,
            1e-4f64,
        ) as ::core::ffi::c_int != 0
    {
        mask |= VEHICLE_LOG_GROUP_MOTION as ::core::ffi::c_int as uint32_t;
    }
    if changed_number((*a).pitch, (*b).pitch, 0.04f64, 1e-4f64) as ::core::ffi::c_int
        != 0
        || changed_number((*a).roll, (*b).roll, 0.04f64, 1e-4f64) as ::core::ffi::c_int
            != 0
        || changed_number((*a).heading, (*b).heading, 0.04f64, 1e-4f64)
            as ::core::ffi::c_int != 0
        || changed_number((*a).angle_of_attack, (*b).angle_of_attack, 0.04f64, 1e-4f64)
            as ::core::ffi::c_int != 0
        || changed_number((*a).sideslip, (*b).sideslip, 0.04f64, 1e-4f64)
            as ::core::ffi::c_int != 0
        || changed_number((*a).pitch_rate, (*b).pitch_rate, 0.02f64, 1e-4f64)
            as ::core::ffi::c_int != 0
        || changed_number((*a).roll_rate, (*b).roll_rate, 0.02f64, 1e-4f64)
            as ::core::ffi::c_int != 0
        || changed_number((*a).yaw_rate, (*b).yaw_rate, 0.02f64, 1e-4f64)
            as ::core::ffi::c_int != 0
        || changed_number((*a).course_rate, (*b).course_rate, 0.02f64, 1e-4f64)
            as ::core::ffi::c_int != 0
        || changed_number(
            (*a).coordinate_pitch_rate,
            (*b).coordinate_pitch_rate,
            0.02f64,
            1e-4f64,
        ) as ::core::ffi::c_int != 0
        || changed_number(
            (*a).coordinate_roll_rate,
            (*b).coordinate_roll_rate,
            0.02f64,
            1e-4f64,
        ) as ::core::ffi::c_int != 0
        || changed_number((*a).heading_rate, (*b).heading_rate, 0.02f64, 1e-4f64)
            as ::core::ffi::c_int != 0
        || changed_number(
            (*a).angle_of_attack_rate,
            (*b).angle_of_attack_rate,
            0.02f64,
            1e-4f64,
        ) as ::core::ffi::c_int != 0
        || changed_number((*a).body_pitch_rate, (*b).body_pitch_rate, 0.02f64, 1e-4f64)
            as ::core::ffi::c_int != 0
        || changed_number((*a).body_roll_rate, (*b).body_roll_rate, 0.02f64, 1e-4f64)
            as ::core::ffi::c_int != 0
        || changed_number((*a).body_yaw_rate, (*b).body_yaw_rate, 0.02f64, 1e-4f64)
            as ::core::ffi::c_int != 0
    {
        mask |= VEHICLE_LOG_GROUP_ATTITUDE as ::core::ffi::c_int as uint32_t;
    }
    if changed_number((*a).dynamic_pressure, (*b).dynamic_pressure, 2.0f64, 0.005f64)
        as ::core::ffi::c_int != 0
        || changed_number((*a).static_pressure, (*b).static_pressure, 2.0f64, 0.005f64)
            as ::core::ffi::c_int != 0
        || changed_number(
            (*a).atmospheric_density,
            (*b).atmospheric_density,
            1e-5f64,
            0.01f64,
        ) as ::core::ffi::c_int != 0
        || changed_number((*a).mach, (*b).mach, 0.002f64, 0.001f64) as ::core::ffi::c_int
            != 0
        || changed_number((*a).stall_fraction, (*b).stall_fraction, 0.002f64, 0.01f64)
            as ::core::ffi::c_int != 0
        || (*a).stall_fraction_is_measured as ::core::ffi::c_int
            != (*b).stall_fraction_is_measured as ::core::ffi::c_int
        || changed_number(
            (*a).lift_force,
            (*b).lift_force,
            100 as ::core::ffi::c_int as ::core::ffi::c_double,
            0.005f64,
        ) as ::core::ffi::c_int != 0
        || changed_number(
            (*a).drag_force,
            (*b).drag_force,
            100 as ::core::ffi::c_int as ::core::ffi::c_double,
            0.005f64,
        ) as ::core::ffi::c_int != 0
        || changed_number((*a).g_force, (*b).g_force, 0.005f64, 0.002f64)
            as ::core::ffi::c_int != 0
        || changed_number(
            (*a).energy_excess_range,
            (*b).energy_excess_range,
            5 as ::core::ffi::c_int as ::core::ffi::c_double,
            0.001f64,
        ) as ::core::ffi::c_int != 0
        || (*a).has_force_vectors as ::core::ffi::c_int
            != (*b).has_force_vectors as ::core::ffi::c_int
        || (*a).has_force_vectors as ::core::ffi::c_int != 0
            && (changed_vector(
                &raw const (*a).lift_vector as *const ::core::ffi::c_double,
                &raw const (*b).lift_vector as *const ::core::ffi::c_double,
                50 as ::core::ffi::c_int as ::core::ffi::c_double,
                0.005f64,
            ) as ::core::ffi::c_int != 0
                || changed_vector(
                    &raw const (*a).drag_vector as *const ::core::ffi::c_double,
                    &raw const (*b).drag_vector as *const ::core::ffi::c_double,
                    50 as ::core::ffi::c_int as ::core::ffi::c_double,
                    0.005f64,
                ) as ::core::ffi::c_int != 0
                || changed_vector(
                    &raw const (*a).aero_acceleration as *const ::core::ffi::c_double,
                    &raw const (*b).aero_acceleration as *const ::core::ffi::c_double,
                    0.005f64,
                    0.005f64,
                ) as ::core::ffi::c_int != 0)
    {
        mask |= VEHICLE_LOG_GROUP_AERO as ::core::ffi::c_int as uint32_t;
    }
    if changed_number(
        (*a).physics_confidence,
        (*b).physics_confidence,
        0.002f64,
        0.01f64,
    ) as ::core::ffi::c_int != 0
        || changed_number(
            (*a).physics_model_residual,
            (*b).physics_model_residual,
            0.002f64,
            0.01f64,
        ) as ::core::ffi::c_int != 0
        || changed_number(
            (*a).physics_model_residual_confidence,
            (*b).physics_model_residual_confidence,
            0.002f64,
            0.01f64,
        ) as ::core::ffi::c_int != 0
        || changed_number(
            (*a).physics_certified_uncertainty,
            (*b).physics_certified_uncertainty,
            0.002f64,
            0.01f64,
        ) as ::core::ffi::c_int != 0
        || changed_vector(
            &raw const (*a).physics_force_residual_per_q as *const ::core::ffi::c_double,
            &raw const (*b).physics_force_residual_per_q as *const ::core::ffi::c_double,
            1e-5f64,
            0.01f64,
        ) as ::core::ffi::c_int != 0
        || changed_vector(
            &raw const (*a).physics_force_residual_sigma_per_q
                as *const ::core::ffi::c_double,
            &raw const (*b).physics_force_residual_sigma_per_q
                as *const ::core::ffi::c_double,
            1e-5f64,
            0.01f64,
        ) as ::core::ffi::c_int != 0
        || changed_number(
            (*a).physics_force_residual_confidence,
            (*b).physics_force_residual_confidence,
            0.002f64,
            0.01f64,
        ) as ::core::ffi::c_int != 0
        || (*a).physics_airbrake_model_available as ::core::ffi::c_int
            != (*b).physics_airbrake_model_available as ::core::ffi::c_int
        || changed_number(
            (*a).physics_airbrake_model_confidence,
            (*b).physics_airbrake_model_confidence,
            0.002f64,
            0.01f64,
        ) as ::core::ffi::c_int != 0
        || changed_number(
            (*a).physics_airbrake_drag_accel,
            (*b).physics_airbrake_drag_accel,
            0.02f64,
            0.01f64,
        ) as ::core::ffi::c_int != 0 || (*a).physics_samples != (*b).physics_samples
        || (*a).physics_live_samples != (*b).physics_live_samples
    {
        mask |= VEHICLE_LOG_GROUP_PHYSICS as ::core::ffi::c_int as uint32_t;
    }
    if changed_number((*a).mass, (*b).mass, 0.25f64, 1e-5f64) as ::core::ffi::c_int != 0
        || changed_number(
            (*a).available_thrust,
            (*b).available_thrust,
            100 as ::core::ffi::c_int as ::core::ffi::c_double,
            0.002f64,
        ) as ::core::ffi::c_int != 0
        || changed_number(
            (*a).current_thrust,
            (*b).current_thrust,
            100 as ::core::ffi::c_int as ::core::ffi::c_double,
            0.002f64,
        ) as ::core::ffi::c_int != 0
        || (*a).has_center_of_mass as ::core::ffi::c_int
            != (*b).has_center_of_mass as ::core::ffi::c_int
        || (*a).has_dry_mass as ::core::ffi::c_int
            != (*b).has_dry_mass as ::core::ffi::c_int
        || (*a).has_dry_mass as ::core::ffi::c_int != 0
            && changed_number((*a).dry_mass, (*b).dry_mass, 0.25f64, 1e-5f64)
                as ::core::ffi::c_int != 0
        || (*a).has_center_of_mass as ::core::ffi::c_int != 0
            && changed_vector(
                &raw const (*a).center_of_mass as *const ::core::ffi::c_double,
                &raw const (*b).center_of_mass as *const ::core::ffi::c_double,
                0.002f64,
                1e-4f64,
            ) as ::core::ffi::c_int != 0
        || (*a).has_center_of_mass_root as ::core::ffi::c_int
            != (*b).has_center_of_mass_root as ::core::ffi::c_int
        || (*a).has_center_of_mass_root as ::core::ffi::c_int != 0
            && changed_vector(
                &raw const (*a).center_of_mass_root as *const ::core::ffi::c_double,
                &raw const (*b).center_of_mass_root as *const ::core::ffi::c_double,
                0.002f64,
                1e-4f64,
            ) as ::core::ffi::c_int != 0
        || (*a).has_torque as ::core::ffi::c_int != (*b).has_torque as ::core::ffi::c_int
        || (*a).has_torque as ::core::ffi::c_int != 0
            && changed_vector(
                &raw const (*a).available_torque as *const ::core::ffi::c_double,
                &raw const (*b).available_torque as *const ::core::ffi::c_double,
                1.0f64,
                0.005f64,
            ) as ::core::ffi::c_int != 0
        || (*a).has_inertia as ::core::ffi::c_int
            != (*b).has_inertia as ::core::ffi::c_int
        || (*a).has_inertia as ::core::ffi::c_int != 0
            && changed_vector(
                &raw const (*a).moment_of_inertia as *const ::core::ffi::c_double,
                &raw const (*b).moment_of_inertia as *const ::core::ffi::c_double,
                0.01f64,
                0.005f64,
            ) as ::core::ffi::c_int != 0
    {
        mask |= VEHICLE_LOG_GROUP_VEHICLE as ::core::ffi::c_int as uint32_t;
    }
    if changed_number((*a).target_pitch, (*b).target_pitch, 0.04f64, 1e-4f64)
        as ::core::ffi::c_int != 0
        || changed_number((*a).target_heading, (*b).target_heading, 0.04f64, 1e-4f64)
            as ::core::ffi::c_int != 0
        || changed_number((*a).target_roll, (*b).target_roll, 0.04f64, 1e-4f64)
            as ::core::ffi::c_int != 0
        || changed_number((*a).target_throttle, (*b).target_throttle, 0.001f64, 0.001f64)
            as ::core::ffi::c_int != 0
        || (*a).has_target_aoa as ::core::ffi::c_int
            != (*b).has_target_aoa as ::core::ffi::c_int
        || (*a).has_target_aoa as ::core::ffi::c_int != 0
            && changed_number((*a).target_aoa, (*b).target_aoa, 0.04f64, 1e-4f64)
                as ::core::ffi::c_int != 0
        || (*a).target_gear as ::core::ffi::c_int
            != (*b).target_gear as ::core::ffi::c_int
        || (*a).target_brakes as ::core::ffi::c_int
            != (*b).target_brakes as ::core::ffi::c_int
        || (*a).target_airbrakes as ::core::ffi::c_int
            != (*b).target_airbrakes as ::core::ffi::c_int
    {
        mask |= VEHICLE_LOG_GROUP_COMMAND as ::core::ffi::c_int as uint32_t;
    }
    if (*a).has_control_state as ::core::ffi::c_int
        != (*b).has_control_state as ::core::ffi::c_int
        || (*a).has_control_state as ::core::ffi::c_int != 0
            && (changed_number(
                (*a).control_state_pitch,
                (*b).control_state_pitch,
                0.002f64,
                0.002f64,
            ) as ::core::ffi::c_int != 0
                || changed_number(
                    (*a).control_state_roll,
                    (*b).control_state_roll,
                    0.002f64,
                    0.002f64,
                ) as ::core::ffi::c_int != 0
                || changed_number(
                    (*a).control_state_yaw,
                    (*b).control_state_yaw,
                    0.002f64,
                    0.002f64,
                ) as ::core::ffi::c_int != 0
                || changed_number(
                    (*a).control_state_throttle,
                    (*b).control_state_throttle,
                    0.001f64,
                    0.001f64,
                ) as ::core::ffi::c_int != 0)
        || (*a).command_applied as ::core::ffi::c_int
            != (*b).command_applied as ::core::ffi::c_int
        || (*a).has_actuator_feedback as ::core::ffi::c_int
            != (*b).has_actuator_feedback as ::core::ffi::c_int
        || (*a).has_actuator_feedback as ::core::ffi::c_int != 0
            && (changed_number(
                (*a).control_pitch,
                (*b).control_pitch,
                0.002f64,
                0.002f64,
            ) as ::core::ffi::c_int != 0
                || changed_number(
                    (*a).control_roll,
                    (*b).control_roll,
                    0.002f64,
                    0.002f64,
                ) as ::core::ffi::c_int != 0
                || changed_number((*a).control_yaw, (*b).control_yaw, 0.002f64, 0.002f64)
                    as ::core::ffi::c_int != 0)
        || (*a).has_control_diagnostics as ::core::ffi::c_int
            != (*b).has_control_diagnostics as ::core::ffi::c_int
        || (*a).has_control_diagnostics as ::core::ffi::c_int != 0
            && (changed_number(
                (*a).control_pitch_error,
                (*b).control_pitch_error,
                0.02f64,
                0.002f64,
            ) as ::core::ffi::c_int != 0
                || changed_number(
                    (*a).control_pitch_trim,
                    (*b).control_pitch_trim,
                    0.002f64,
                    0.002f64,
                ) as ::core::ffi::c_int != 0
                || changed_number(
                    (*a).control_pitch_authority,
                    (*b).control_pitch_authority,
                    0.05f64,
                    0.002f64,
                ) as ::core::ffi::c_int != 0
                || changed_number(
                    (*a).control_pitch_aero_fraction,
                    (*b).control_pitch_aero_fraction,
                    0.002f64,
                    0.002f64,
                ) as ::core::ffi::c_int != 0)
    {
        mask |= VEHICLE_LOG_GROUP_CONTROL as ::core::ffi::c_int as uint32_t;
    }
    let mut surfaces_changed: bool = (*a).has_control_surface_state as ::core::ffi::c_int
        != (*b).has_control_surface_state as ::core::ffi::c_int
        || (*a).control_surface_count != (*b).control_surface_count;
    if !surfaces_changed && (*a).has_control_surface_state as ::core::ffi::c_int != 0 {
        surfaces_changed = changed_number(
            (*a).control_surface_state_ut,
            (*b).control_surface_state_ut,
            0.001f64,
            0 as ::core::ffi::c_int as ::core::ffi::c_double,
        );
        let mut i: size_t = 0 as size_t;
        while !surfaces_changed && i < (*a).control_surface_count
            && i < LANDING_CONTROL_SURFACE_MAX as size_t
        {
            let mut x: *const VehicleLogSurfaceSample = (&raw const (*a).control_surfaces
                as *const VehicleLogSurfaceSample)
                .offset(i as isize) as *const VehicleLogSurfaceSample;
            let mut y: *const VehicleLogSurfaceSample = (&raw const (*b).control_surfaces
                as *const VehicleLogSurfaceSample)
                .offset(i as isize) as *const VehicleLogSurfaceSample;
            surfaces_changed = strcmp(
                &raw const (*x).part_name as *const ::core::ffi::c_char,
                &raw const (*y).part_name as *const ::core::ffi::c_char,
            ) != 0
                || strcmp(
                    &raw const (*x).role as *const ::core::ffi::c_char,
                    &raw const (*y).role as *const ::core::ffi::c_char,
                ) != 0
                || (*x).deployed as ::core::ffi::c_int
                    != (*y).deployed as ::core::ffi::c_int
                || (*x).deflection_override as ::core::ffi::c_int
                    != (*y).deflection_override as ::core::ffi::c_int
                || (*x).deflection_override as ::core::ffi::c_int != 0
                    && changed_number(
                        (*x).override_deflection,
                        (*y).override_deflection,
                        0.002f64,
                        0.002f64,
                    ) as ::core::ffi::c_int != 0;
            i = i.wrapping_add(1);
        }
    }
    if surfaces_changed {
        mask |= VEHICLE_LOG_GROUP_SURFACES as ::core::ffi::c_int as uint32_t;
    }
    if (*a).has_loop_wall_delta as ::core::ffi::c_int
        != (*b).has_loop_wall_delta as ::core::ffi::c_int
        || (*a).has_telemetry_latency as ::core::ffi::c_int
            != (*b).has_telemetry_latency as ::core::ffi::c_int
        || (*a).has_guidance_compute as ::core::ffi::c_int
            != (*b).has_guidance_compute as ::core::ffi::c_int
        || (*a).has_apply_latency as ::core::ffi::c_int
            != (*b).has_apply_latency as ::core::ffi::c_int
        || (*a).has_control_loop as ::core::ffi::c_int
            != (*b).has_control_loop as ::core::ffi::c_int
        || (*a).has_loop_wall_delta as ::core::ffi::c_int != 0
            && changed_number(
                (*a).loop_wall_delta_ms,
                (*b).loop_wall_delta_ms,
                0.25f64,
                0.01f64,
            ) as ::core::ffi::c_int != 0
        || (*a).has_telemetry_latency as ::core::ffi::c_int != 0
            && changed_number(
                (*a).telemetry_latency_ms,
                (*b).telemetry_latency_ms,
                0.25f64,
                0.01f64,
            ) as ::core::ffi::c_int != 0
        || (*a).has_guidance_compute as ::core::ffi::c_int != 0
            && changed_number(
                (*a).guidance_compute_ms,
                (*b).guidance_compute_ms,
                0.25f64,
                0.01f64,
            ) as ::core::ffi::c_int != 0
        || (*a).has_apply_latency as ::core::ffi::c_int != 0
            && changed_number(
                (*a).apply_latency_ms,
                (*b).apply_latency_ms,
                0.25f64,
                0.01f64,
            ) as ::core::ffi::c_int != 0
        || (*a).has_control_loop as ::core::ffi::c_int != 0
            && changed_number(
                (*a).control_loop_ms,
                (*b).control_loop_ms,
                0.25f64,
                0.01f64,
            ) as ::core::ffi::c_int != 0
    {
        mask |= VEHICLE_LOG_GROUP_RUNTIME as ::core::ffi::c_int as uint32_t;
    }
    if changed_number(
        (*a).range_to_site,
        (*b).range_to_site,
        5 as ::core::ffi::c_int as ::core::ffi::c_double,
        0.0005f64,
    ) as ::core::ffi::c_int != 0
        || changed_number((*a).bearing_to_site, (*b).bearing_to_site, 0.05f64, 1e-4f64)
            as ::core::ffi::c_int != 0
        || changed_number(
            (*a).runway_along_track,
            (*b).runway_along_track,
            2 as ::core::ffi::c_int as ::core::ffi::c_double,
            0.0005f64,
        ) as ::core::ffi::c_int != 0
        || changed_number(
            (*a).runway_cross_track,
            (*b).runway_cross_track,
            2 as ::core::ffi::c_int as ::core::ffi::c_double,
            0.0005f64,
        ) as ::core::ffi::c_int != 0
        || changed_number(
            (*a).specific_mechanical_energy,
            (*b).specific_mechanical_energy,
            100 as ::core::ffi::c_int as ::core::ffi::c_double,
            1e-5f64,
        ) as ::core::ffi::c_int != 0
        || changed_number(
            (*a).entry_reversal_time_remaining,
            (*b).entry_reversal_time_remaining,
            0.10f64,
            0.002f64,
        ) as ::core::ffi::c_int != 0
        || changed_number(
            (*a).entry_reversal_ut,
            (*b).entry_reversal_ut,
            0.02f64,
            0 as ::core::ffi::c_int as ::core::ffi::c_double,
        ) as ::core::ffi::c_int != 0
        || changed_number(
            (*a).entry_reversal_range,
            (*b).entry_reversal_range,
            2 as ::core::ffi::c_int as ::core::ffi::c_double,
            0.0005f64,
        ) as ::core::ffi::c_int != 0
        || changed_number(
            (*a).entry_reversal_sign,
            (*b).entry_reversal_sign,
            0.01f64,
            0 as ::core::ffi::c_int as ::core::ffi::c_double,
        ) as ::core::ffi::c_int != 0
    {
        mask |= VEHICLE_LOG_GROUP_GUIDANCE as ::core::ffi::c_int as uint32_t;
    }
    return mask;
}
unsafe extern "C" fn minimum_interval(
    mut s: *const VehicleLogSample,
) -> ::core::ffi::c_double {
    let mut rapid: bool = fabs((*s).roll_rate)
        > 5 as ::core::ffi::c_int as ::core::ffi::c_double
        || fabs((*s).pitch_rate) > 4 as ::core::ffi::c_int as ::core::ffi::c_double
        || fabs((*s).yaw_rate) > 4 as ::core::ffi::c_int as ::core::ffi::c_double
        || (*s).has_actuator_feedback as ::core::ffi::c_int != 0
            && (fabs((*s).control_pitch) > 0.6f64 || fabs((*s).control_roll) > 0.6f64
                || fabs((*s).control_yaw) > 0.6f64);
    if rapid {
        return 0.05f64;
    }
    if (*s).dynamic_pressure > 25 as ::core::ffi::c_int as ::core::ffi::c_double
        || (*s).automation_engaged as ::core::ffi::c_int != 0
    {
        return 0.10f64;
    }
    return 0.20f64;
}
#[no_mangle]
pub unsafe extern "C" fn vehicle_log_encoder_init(mut encoder: *mut VehicleLogEncoder) {
    if !encoder.is_null() {
        memset(
            encoder as *mut ::core::ffi::c_void,
            0 as ::core::ffi::c_int,
            ::core::mem::size_of::<VehicleLogEncoder>() as size_t,
        );
    }
}
#[no_mangle]
pub unsafe extern "C" fn vehicle_log_decide(
    mut encoder: *const VehicleLogEncoder,
    mut sample: *const VehicleLogSample,
    mut force_event: bool,
    mut force_keyframe: bool,
) -> VehicleLogDecision {
    let mut d: VehicleLogDecision = VehicleLogDecision {
        emit: 0 as ::core::ffi::c_int != 0,
        keyframe: false,
        durable: false,
        groups: 0,
    };
    if encoder.is_null() || sample.is_null() {
        return d;
    }
    let mut first: bool = !(*encoder).has_last;
    let mut periodic: bool = (*encoder).has_last as ::core::ffi::c_int != 0
        && (*sample).wall_monotonic_seconds - (*encoder).last_keyframe_wall >= 5.0f64;
    if first as ::core::ffi::c_int != 0 || force_keyframe as ::core::ffi::c_int != 0
        || periodic as ::core::ffi::c_int != 0
    {
        d.emit = true_0 != 0;
        d.keyframe = true_0 != 0;
        d.groups = VEHICLE_LOG_GROUP_ALL as ::core::ffi::c_int as uint32_t;
        d.durable = force_event as ::core::ffi::c_int != 0
            || force_keyframe as ::core::ffi::c_int != 0;
        return d;
    }
    d.groups = changed_groups(sample, &raw const (*encoder).last);
    let mut semantic: bool = d.groups
        & VEHICLE_LOG_GROUP_STATE as ::core::ffi::c_int as uint32_t != 0 as uint32_t
        || (*sample).event[0 as ::core::ffi::c_int as usize] as ::core::ffi::c_int
            != 0 as ::core::ffi::c_int;
    if force_event as ::core::ffi::c_int != 0 || semantic as ::core::ffi::c_int != 0 {
        d.emit = true_0 != 0;
        d.durable = true_0 != 0;
        return d;
    }
    if d.groups == 0 {
        return d;
    }
    if (*sample).wall_monotonic_seconds - (*encoder).last_emit_wall + 1e-9f64
        < minimum_interval(sample)
    {
        return d;
    }
    d.emit = true_0 != 0;
    return d;
}
#[no_mangle]
pub unsafe extern "C" fn vehicle_log_commit(
    mut encoder: *mut VehicleLogEncoder,
    mut sample: *const VehicleLogSample,
    mut decision: *const VehicleLogDecision,
) {
    if encoder.is_null() || sample.is_null() || decision.is_null() || !(*decision).emit {
        return;
    }
    (*encoder).last = *sample;
    (*encoder).has_last = true_0 != 0;
    (*encoder).last_emit_wall = (*sample).wall_monotonic_seconds;
    if (*decision).keyframe {
        (*encoder).last_keyframe_wall = (*sample).wall_monotonic_seconds;
    }
}
unsafe extern "C" fn field_prefix(
    mut w: *mut JsonWriter,
    mut first: *mut bool,
    mut key: *const ::core::ffi::c_char,
) {
    if !*first {
        jw_char(w, ',' as i32 as ::core::ffi::c_char);
    }
    *first = false_0 != 0;
    jw_string(w, key);
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
}
unsafe extern "C" fn vec3_json(
    mut w: *mut JsonWriter,
    mut v: *const ::core::ffi::c_double,
) {
    jw_char(w, '[' as i32 as ::core::ffi::c_char);
    jw_number(w, *v.offset(0 as ::core::ffi::c_int as isize));
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_number(w, *v.offset(1 as ::core::ffi::c_int as isize));
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_number(w, *v.offset(2 as ::core::ffi::c_int as isize));
    jw_char(w, ']' as i32 as ::core::ffi::c_char);
}
#[no_mangle]
pub unsafe extern "C" fn vehicle_log_record_json(
    mut w: *mut JsonWriter,
    mut session_id: *const ::core::ffi::c_char,
    mut record_sequence: uint64_t,
    mut s: *const VehicleLogSample,
    mut d: *const VehicleLogDecision,
) {
    if w.is_null() || s.is_null() || d.is_null() {
        return;
    }
    jw_char(w, '{' as i32 as ::core::ffi::c_char);
    jw_raw(
        w,
        b"\"schemaVersion\":5,\"recordType\":\0" as *const u8
            as *const ::core::ffi::c_char,
    );
    jw_string(
        w,
        if (*d).keyframe as ::core::ffi::c_int != 0 {
            b"vehicleKeyframe\0" as *const u8 as *const ::core::ffi::c_char
        } else {
            b"vehicleDelta\0" as *const u8 as *const ::core::ffi::c_char
        },
    );
    jw_raw(w, b",\"sessionId\":\0" as *const u8 as *const ::core::ffi::c_char);
    jw_string(
        w,
        if !session_id.is_null() {
            session_id
        } else {
            b"\0" as *const u8 as *const ::core::ffi::c_char
        },
    );
    jw_raw(w, b",\"recordSequence\":\0" as *const u8 as *const ::core::ffi::c_char);
    jw_integer(w, record_sequence as ::core::ffi::c_longlong);
    jw_raw(w, b",\"tickSequence\":\0" as *const u8 as *const ::core::ffi::c_char);
    jw_integer(w, (*s).tick_sequence as ::core::ffi::c_longlong);
    jw_raw(
        w,
        b",\"wallMonotonicSeconds\":\0" as *const u8 as *const ::core::ffi::c_char,
    );
    jw_number(w, (*s).wall_monotonic_seconds);
    jw_raw(w, b",\"ut\":\0" as *const u8 as *const ::core::ffi::c_char);
    jw_number(w, (*s).ut);
    if (*s).event[0 as ::core::ffi::c_int as usize] != 0 {
        jw_raw(w, b",\"event\":\0" as *const u8 as *const ::core::ffi::c_char);
        jw_string(w, &raw const (*s).event as *const ::core::ffi::c_char);
    }
    jw_raw(w, b",\"fields\":{\0" as *const u8 as *const ::core::ffi::c_char);
    let mut first: bool = true_0 != 0;
    if (*d).groups & VEHICLE_LOG_GROUP_STATE as ::core::ffi::c_int as uint32_t != 0 {
        field_prefix(
            w,
            &raw mut first,
            b"state\0" as *const u8 as *const ::core::ffi::c_char,
        );
        jw_char(w, '{' as i32 as ::core::ffi::c_char);
        jw_raw(w, b"\"phase\":\0" as *const u8 as *const ::core::ffi::c_char);
        jw_string(w, &raw const (*s).phase as *const ::core::ffi::c_char);
        jw_raw(w, b",\"controlProfile\":\0" as *const u8 as *const ::core::ffi::c_char);
        jw_string(w, &raw const (*s).control_profile as *const ::core::ffi::c_char);
        jw_raw(w, b",\"vesselSituation\":\0" as *const u8 as *const ::core::ffi::c_char);
        jw_string(w, &raw const (*s).vessel_situation as *const ::core::ffi::c_char);
        jw_raw(w, b",\"automation\":\0" as *const u8 as *const ::core::ffi::c_char);
        jw_bool(w, (*s).automation_engaged);
        jw_raw(w, b",\"paused\":\0" as *const u8 as *const ::core::ffi::c_char);
        jw_bool(w, (*s).paused);
        jw_raw(
            w,
            b",\"constraintActive\":\0" as *const u8 as *const ::core::ffi::c_char,
        );
        jw_bool(w, (*s).constraint_active);
        jw_raw(w, b",\"gear\":\0" as *const u8 as *const ::core::ffi::c_char);
        jw_bool(w, (*s).gear);
        jw_raw(w, b",\"brakes\":\0" as *const u8 as *const ::core::ffi::c_char);
        jw_bool(w, (*s).brakes);
        jw_raw(
            w,
            b",\"airbrakesAvailable\":\0" as *const u8 as *const ::core::ffi::c_char,
        );
        jw_bool(w, (*s).has_airbrakes);
        jw_raw(w, b",\"airbrakes\":\0" as *const u8 as *const ::core::ffi::c_char);
        if (*s).has_airbrakes {
            jw_bool(w, (*s).airbrakes);
        } else {
            jw_raw(w, b"null\0" as *const u8 as *const ::core::ffi::c_char);
        }
        jw_raw(w, b",\"sasAvailable\":\0" as *const u8 as *const ::core::ffi::c_char);
        jw_bool(w, (*s).has_sas_state);
        jw_raw(w, b",\"sasEnabled\":\0" as *const u8 as *const ::core::ffi::c_char);
        if (*s).has_sas_state {
            jw_bool(w, (*s).sas_enabled);
        } else {
            jw_null(w);
        }
        jw_raw(w, b",\"entryPlanValid\":\0" as *const u8 as *const ::core::ffi::c_char);
        jw_bool(w, (*s).entry_plan_valid);
        jw_raw(
            w,
            b",\"entryReversalScheduled\":\0" as *const u8 as *const ::core::ffi::c_char,
        );
        jw_bool(w, (*s).entry_reversal_scheduled);
        jw_raw(w, b",\"planId\":\0" as *const u8 as *const ::core::ffi::c_char);
        if (*s).has_plan_identity {
            jw_integer(w, (*s).plan_id as ::core::ffi::c_longlong);
        } else {
            jw_null(w);
        }
        jw_raw(w, b",\"planVersion\":\0" as *const u8 as *const ::core::ffi::c_char);
        if (*s).has_plan_identity {
            jw_integer(w, (*s).plan_version as ::core::ffi::c_longlong);
        } else {
            jw_null(w);
        }
        jw_raw(w, b",\"parentPlanId\":\0" as *const u8 as *const ::core::ffi::c_char);
        if (*s).has_plan_identity {
            jw_integer(w, (*s).parent_plan_id as ::core::ffi::c_longlong);
        } else {
            jw_null(w);
        }
        jw_raw(
            w,
            b",\"parentPlanVersion\":\0" as *const u8 as *const ::core::ffi::c_char,
        );
        if (*s).has_plan_identity {
            jw_integer(w, (*s).parent_plan_version as ::core::ffi::c_longlong);
        } else {
            jw_null(w);
        }
        jw_char(w, '}' as i32 as ::core::ffi::c_char);
    }
    if (*d).groups & VEHICLE_LOG_GROUP_POSITION as ::core::ffi::c_int as uint32_t != 0 {
        field_prefix(
            w,
            &raw mut first,
            b"position\0" as *const u8 as *const ::core::ffi::c_char,
        );
        jw_char(w, '{' as i32 as ::core::ffi::c_char);
        jw_raw(w, b"\"latitude\":\0" as *const u8 as *const ::core::ffi::c_char);
        jw_number(w, (*s).latitude);
        jw_raw(w, b",\"longitude\":\0" as *const u8 as *const ::core::ffi::c_char);
        jw_number(w, (*s).longitude);
        jw_raw(w, b",\"altitude\":\0" as *const u8 as *const ::core::ffi::c_char);
        jw_number(w, (*s).altitude);
        jw_raw(w, b",\"radarAltitude\":\0" as *const u8 as *const ::core::ffi::c_char);
        jw_number(w, (*s).radar_altitude);
        jw_raw(w, b",\"inertial\":\0" as *const u8 as *const ::core::ffi::c_char);
        vec3_json(w, &raw const (*s).position as *const ::core::ffi::c_double);
        jw_char(w, '}' as i32 as ::core::ffi::c_char);
    }
    if (*d).groups & VEHICLE_LOG_GROUP_MOTION as ::core::ffi::c_int as uint32_t != 0 {
        field_prefix(
            w,
            &raw mut first,
            b"motion\0" as *const u8 as *const ::core::ffi::c_char,
        );
        jw_char(w, '{' as i32 as ::core::ffi::c_char);
        jw_raw(w, b"\"velocity\":\0" as *const u8 as *const ::core::ffi::c_char);
        vec3_json(w, &raw const (*s).velocity as *const ::core::ffi::c_double);
        jw_raw(w, b",\"trueAirSpeed\":\0" as *const u8 as *const ::core::ffi::c_char);
        jw_number(w, (*s).true_air_speed);
        jw_raw(w, b",\"horizontalSpeed\":\0" as *const u8 as *const ::core::ffi::c_char);
        jw_number(w, (*s).horizontal_speed);
        jw_raw(w, b",\"verticalSpeed\":\0" as *const u8 as *const ::core::ffi::c_char);
        jw_number(w, (*s).vertical_speed);
        jw_raw(w, b",\"flightPathAngle\":\0" as *const u8 as *const ::core::ffi::c_char);
        jw_number(w, (*s).flight_path_angle);
        jw_char(w, '}' as i32 as ::core::ffi::c_char);
    }
    if (*d).groups & VEHICLE_LOG_GROUP_ATTITUDE as ::core::ffi::c_int as uint32_t != 0 {
        field_prefix(
            w,
            &raw mut first,
            b"attitude\0" as *const u8 as *const ::core::ffi::c_char,
        );
        jw_char(w, '{' as i32 as ::core::ffi::c_char);
        jw_raw(w, b"\"pitch\":\0" as *const u8 as *const ::core::ffi::c_char);
        jw_number(w, (*s).pitch);
        jw_raw(w, b",\"roll\":\0" as *const u8 as *const ::core::ffi::c_char);
        jw_number(w, (*s).roll);
        jw_raw(w, b",\"heading\":\0" as *const u8 as *const ::core::ffi::c_char);
        jw_number(w, (*s).heading);
        jw_raw(w, b",\"angleOfAttack\":\0" as *const u8 as *const ::core::ffi::c_char);
        jw_number(w, (*s).angle_of_attack);
        jw_raw(w, b",\"sideslip\":\0" as *const u8 as *const ::core::ffi::c_char);
        jw_number(w, (*s).sideslip);
        jw_raw(w, b",\"pitchRate\":\0" as *const u8 as *const ::core::ffi::c_char);
        jw_number(w, (*s).pitch_rate);
        jw_raw(w, b",\"rollRate\":\0" as *const u8 as *const ::core::ffi::c_char);
        jw_number(w, (*s).roll_rate);
        jw_raw(w, b",\"yawRate\":\0" as *const u8 as *const ::core::ffi::c_char);
        jw_number(w, (*s).yaw_rate);
        jw_raw(w, b",\"courseRate\":\0" as *const u8 as *const ::core::ffi::c_char);
        jw_number(w, (*s).course_rate);
        jw_raw(
            w,
            b",\"coordinatePitchRate\":\0" as *const u8 as *const ::core::ffi::c_char,
        );
        jw_number(w, (*s).coordinate_pitch_rate);
        jw_raw(
            w,
            b",\"coordinateRollRate\":\0" as *const u8 as *const ::core::ffi::c_char,
        );
        jw_number(w, (*s).coordinate_roll_rate);
        jw_raw(w, b",\"headingRate\":\0" as *const u8 as *const ::core::ffi::c_char);
        jw_number(w, (*s).heading_rate);
        jw_raw(
            w,
            b",\"angleOfAttackRate\":\0" as *const u8 as *const ::core::ffi::c_char,
        );
        jw_number(w, (*s).angle_of_attack_rate);
        jw_raw(w, b",\"bodyPitchRate\":\0" as *const u8 as *const ::core::ffi::c_char);
        jw_number(w, (*s).body_pitch_rate);
        jw_raw(w, b",\"bodyRollRate\":\0" as *const u8 as *const ::core::ffi::c_char);
        jw_number(w, (*s).body_roll_rate);
        jw_raw(w, b",\"bodyYawRate\":\0" as *const u8 as *const ::core::ffi::c_char);
        jw_number(w, (*s).body_yaw_rate);
        jw_char(w, '}' as i32 as ::core::ffi::c_char);
    }
    if (*d).groups & VEHICLE_LOG_GROUP_AERO as ::core::ffi::c_int as uint32_t != 0 {
        field_prefix(
            w,
            &raw mut first,
            b"aero\0" as *const u8 as *const ::core::ffi::c_char,
        );
        jw_char(w, '{' as i32 as ::core::ffi::c_char);
        jw_raw(w, b"\"dynamicPressure\":\0" as *const u8 as *const ::core::ffi::c_char);
        jw_number(w, (*s).dynamic_pressure);
        jw_raw(w, b",\"staticPressure\":\0" as *const u8 as *const ::core::ffi::c_char);
        jw_number(w, (*s).static_pressure);
        jw_raw(w, b",\"density\":\0" as *const u8 as *const ::core::ffi::c_char);
        jw_number(w, (*s).atmospheric_density);
        jw_raw(w, b",\"mach\":\0" as *const u8 as *const ::core::ffi::c_char);
        jw_number(w, (*s).mach);
        jw_raw(w, b",\"stallFraction\":\0" as *const u8 as *const ::core::ffi::c_char);
        jw_number(w, (*s).stall_fraction);
        jw_raw(
            w,
            b",\"stallFractionMeasured\":\0" as *const u8 as *const ::core::ffi::c_char,
        );
        jw_bool(w, (*s).stall_fraction_is_measured);
        jw_raw(w, b",\"liftForce\":\0" as *const u8 as *const ::core::ffi::c_char);
        jw_number(w, (*s).lift_force);
        jw_raw(w, b",\"dragForce\":\0" as *const u8 as *const ::core::ffi::c_char);
        jw_number(w, (*s).drag_force);
        jw_raw(w, b",\"gForce\":\0" as *const u8 as *const ::core::ffi::c_char);
        jw_number(w, (*s).g_force);
        jw_raw(
            w,
            b",\"energyExcessRange\":\0" as *const u8 as *const ::core::ffi::c_char,
        );
        jw_number(w, (*s).energy_excess_range);
        jw_raw(w, b",\"liftVector\":\0" as *const u8 as *const ::core::ffi::c_char);
        if (*s).has_force_vectors {
            vec3_json(w, &raw const (*s).lift_vector as *const ::core::ffi::c_double);
        } else {
            jw_raw(w, b"null\0" as *const u8 as *const ::core::ffi::c_char);
        }
        jw_raw(w, b",\"dragVector\":\0" as *const u8 as *const ::core::ffi::c_char);
        if (*s).has_force_vectors {
            vec3_json(w, &raw const (*s).drag_vector as *const ::core::ffi::c_double);
        } else {
            jw_raw(w, b"null\0" as *const u8 as *const ::core::ffi::c_char);
        }
        jw_raw(
            w,
            b",\"aeroAcceleration\":\0" as *const u8 as *const ::core::ffi::c_char,
        );
        if (*s).has_force_vectors {
            vec3_json(
                w,
                &raw const (*s).aero_acceleration as *const ::core::ffi::c_double,
            );
        } else {
            jw_raw(w, b"null\0" as *const u8 as *const ::core::ffi::c_char);
        }
        jw_char(w, '}' as i32 as ::core::ffi::c_char);
    }
    if (*d).groups & VEHICLE_LOG_GROUP_PHYSICS as ::core::ffi::c_int as uint32_t != 0 {
        field_prefix(
            w,
            &raw mut first,
            b"physics\0" as *const u8 as *const ::core::ffi::c_char,
        );
        jw_char(w, '{' as i32 as ::core::ffi::c_char);
        jw_raw(w, b"\"confidence\":\0" as *const u8 as *const ::core::ffi::c_char);
        jw_number(w, (*s).physics_confidence);
        jw_raw(w, b",\"modelResidual\":\0" as *const u8 as *const ::core::ffi::c_char);
        jw_number(w, (*s).physics_model_residual);
        jw_raw(
            w,
            b",\"modelResidualConfidence\":\0" as *const u8 as *const ::core::ffi::c_char,
        );
        jw_number(w, (*s).physics_model_residual_confidence);
        jw_raw(
            w,
            b",\"certifiedUncertainty\":\0" as *const u8 as *const ::core::ffi::c_char,
        );
        jw_number(w, (*s).physics_certified_uncertainty);
        jw_raw(
            w,
            b",\"forceResidualPerQ\":\0" as *const u8 as *const ::core::ffi::c_char,
        );
        vec3_json(
            w,
            &raw const (*s).physics_force_residual_per_q as *const ::core::ffi::c_double,
        );
        jw_raw(
            w,
            b",\"forceResidualSigmaPerQ\":\0" as *const u8 as *const ::core::ffi::c_char,
        );
        vec3_json(
            w,
            &raw const (*s).physics_force_residual_sigma_per_q
                as *const ::core::ffi::c_double,
        );
        jw_raw(
            w,
            b",\"forceResidualConfidence\":\0" as *const u8 as *const ::core::ffi::c_char,
        );
        jw_number(w, (*s).physics_force_residual_confidence);
        jw_raw(
            w,
            b",\"airbrakeModelAvailable\":\0" as *const u8 as *const ::core::ffi::c_char,
        );
        jw_bool(w, (*s).physics_airbrake_model_available);
        jw_raw(
            w,
            b",\"airbrakeModelConfidence\":\0" as *const u8 as *const ::core::ffi::c_char,
        );
        jw_number(w, (*s).physics_airbrake_model_confidence);
        jw_raw(
            w,
            b",\"airbrakeDragAccel\":\0" as *const u8 as *const ::core::ffi::c_char,
        );
        jw_number(w, (*s).physics_airbrake_drag_accel);
        jw_raw(w, b",\"samples\":\0" as *const u8 as *const ::core::ffi::c_char);
        jw_integer(w, (*s).physics_samples as ::core::ffi::c_longlong);
        jw_raw(w, b",\"liveSamples\":\0" as *const u8 as *const ::core::ffi::c_char);
        jw_integer(w, (*s).physics_live_samples as ::core::ffi::c_longlong);
        jw_char(w, '}' as i32 as ::core::ffi::c_char);
    }
    if (*d).groups & VEHICLE_LOG_GROUP_VEHICLE as ::core::ffi::c_int as uint32_t != 0 {
        field_prefix(
            w,
            &raw mut first,
            b"vehicle\0" as *const u8 as *const ::core::ffi::c_char,
        );
        jw_char(w, '{' as i32 as ::core::ffi::c_char);
        jw_raw(w, b"\"mass\":\0" as *const u8 as *const ::core::ffi::c_char);
        jw_number(w, (*s).mass);
        jw_raw(w, b",\"dryMass\":\0" as *const u8 as *const ::core::ffi::c_char);
        if (*s).has_dry_mass {
            jw_number(w, (*s).dry_mass);
        } else {
            jw_null(w);
        }
        jw_raw(w, b",\"availableThrust\":\0" as *const u8 as *const ::core::ffi::c_char);
        jw_number(w, (*s).available_thrust);
        jw_raw(w, b",\"currentThrust\":\0" as *const u8 as *const ::core::ffi::c_char);
        jw_number(w, (*s).current_thrust);
        jw_raw(w, b",\"centerOfMass\":\0" as *const u8 as *const ::core::ffi::c_char);
        if (*s).has_center_of_mass {
            vec3_json(w, &raw const (*s).center_of_mass as *const ::core::ffi::c_double);
        } else {
            jw_raw(w, b"null\0" as *const u8 as *const ::core::ffi::c_char);
        }
        jw_raw(
            w,
            b",\"centerOfMassRoot\":\0" as *const u8 as *const ::core::ffi::c_char,
        );
        if (*s).has_center_of_mass_root {
            vec3_json(
                w,
                &raw const (*s).center_of_mass_root as *const ::core::ffi::c_double,
            );
        } else {
            jw_raw(w, b"null\0" as *const u8 as *const ::core::ffi::c_char);
        }
        jw_raw(w, b",\"availableTorque\":\0" as *const u8 as *const ::core::ffi::c_char);
        if (*s).has_torque {
            vec3_json(
                w,
                &raw const (*s).available_torque as *const ::core::ffi::c_double,
            );
        } else {
            jw_raw(w, b"null\0" as *const u8 as *const ::core::ffi::c_char);
        }
        jw_raw(w, b",\"momentOfInertia\":\0" as *const u8 as *const ::core::ffi::c_char);
        if (*s).has_inertia {
            vec3_json(
                w,
                &raw const (*s).moment_of_inertia as *const ::core::ffi::c_double,
            );
        } else {
            jw_raw(w, b"null\0" as *const u8 as *const ::core::ffi::c_char);
        }
        jw_char(w, '}' as i32 as ::core::ffi::c_char);
    }
    if (*d).groups & VEHICLE_LOG_GROUP_COMMAND as ::core::ffi::c_int as uint32_t != 0 {
        field_prefix(
            w,
            &raw mut first,
            b"command\0" as *const u8 as *const ::core::ffi::c_char,
        );
        jw_char(w, '{' as i32 as ::core::ffi::c_char);
        jw_raw(w, b"\"targetPitch\":\0" as *const u8 as *const ::core::ffi::c_char);
        jw_number(w, (*s).target_pitch);
        jw_raw(w, b",\"targetHeading\":\0" as *const u8 as *const ::core::ffi::c_char);
        jw_number(w, (*s).target_heading);
        jw_raw(w, b",\"targetRoll\":\0" as *const u8 as *const ::core::ffi::c_char);
        jw_number(w, (*s).target_roll);
        jw_raw(w, b",\"targetThrottle\":\0" as *const u8 as *const ::core::ffi::c_char);
        jw_number(w, (*s).target_throttle);
        jw_raw(w, b",\"targetAoA\":\0" as *const u8 as *const ::core::ffi::c_char);
        if (*s).has_target_aoa {
            jw_number(w, (*s).target_aoa);
        } else {
            jw_raw(w, b"null\0" as *const u8 as *const ::core::ffi::c_char);
        }
        jw_raw(w, b",\"gear\":\0" as *const u8 as *const ::core::ffi::c_char);
        jw_bool(w, (*s).target_gear);
        jw_raw(w, b",\"brakes\":\0" as *const u8 as *const ::core::ffi::c_char);
        jw_bool(w, (*s).target_brakes);
        jw_raw(w, b",\"airbrakes\":\0" as *const u8 as *const ::core::ffi::c_char);
        jw_bool(w, (*s).target_airbrakes);
        jw_char(w, '}' as i32 as ::core::ffi::c_char);
    }
    if (*d).groups & VEHICLE_LOG_GROUP_CONTROL as ::core::ffi::c_int as uint32_t != 0 {
        field_prefix(
            w,
            &raw mut first,
            b"appliedControl\0" as *const u8 as *const ::core::ffi::c_char,
        );
        jw_char(w, '{' as i32 as ::core::ffi::c_char);
        jw_raw(
            w,
            b"\"reportedAvailable\":\0" as *const u8 as *const ::core::ffi::c_char,
        );
        jw_bool(w, (*s).has_control_state);
        jw_raw(w, b",\"reportedPitch\":\0" as *const u8 as *const ::core::ffi::c_char);
        if (*s).has_control_state {
            jw_number(w, (*s).control_state_pitch);
        } else {
            jw_raw(w, b"null\0" as *const u8 as *const ::core::ffi::c_char);
        }
        jw_raw(w, b",\"reportedRoll\":\0" as *const u8 as *const ::core::ffi::c_char);
        if (*s).has_control_state {
            jw_number(w, (*s).control_state_roll);
        } else {
            jw_raw(w, b"null\0" as *const u8 as *const ::core::ffi::c_char);
        }
        jw_raw(w, b",\"reportedYaw\":\0" as *const u8 as *const ::core::ffi::c_char);
        if (*s).has_control_state {
            jw_number(w, (*s).control_state_yaw);
        } else {
            jw_raw(w, b"null\0" as *const u8 as *const ::core::ffi::c_char);
        }
        jw_raw(
            w,
            b",\"reportedThrottle\":\0" as *const u8 as *const ::core::ffi::c_char,
        );
        if (*s).has_control_state {
            jw_number(w, (*s).control_state_throttle);
        } else {
            jw_raw(w, b"null\0" as *const u8 as *const ::core::ffi::c_char);
        }
        jw_raw(w, b",\"applySucceeded\":\0" as *const u8 as *const ::core::ffi::c_char);
        jw_bool(w, (*s).command_applied);
        jw_raw(w, b",\"diagnosticPitch\":\0" as *const u8 as *const ::core::ffi::c_char);
        if (*s).has_actuator_feedback {
            jw_number(w, (*s).control_pitch);
        } else {
            jw_raw(w, b"null\0" as *const u8 as *const ::core::ffi::c_char);
        }
        jw_raw(w, b",\"diagnosticRoll\":\0" as *const u8 as *const ::core::ffi::c_char);
        if (*s).has_actuator_feedback {
            jw_number(w, (*s).control_roll);
        } else {
            jw_raw(w, b"null\0" as *const u8 as *const ::core::ffi::c_char);
        }
        jw_raw(w, b",\"diagnosticYaw\":\0" as *const u8 as *const ::core::ffi::c_char);
        if (*s).has_actuator_feedback {
            jw_number(w, (*s).control_yaw);
        } else {
            jw_raw(w, b"null\0" as *const u8 as *const ::core::ffi::c_char);
        }
        jw_raw(
            w,
            b",\"controlDiagnostics\":\0" as *const u8 as *const ::core::ffi::c_char,
        );
        if (*s).has_control_diagnostics {
            jw_char(w, '{' as i32 as ::core::ffi::c_char);
            jw_raw(w, b"\"pitchError\":\0" as *const u8 as *const ::core::ffi::c_char);
            jw_number(w, (*s).control_pitch_error);
            jw_raw(w, b",\"pitchTrim\":\0" as *const u8 as *const ::core::ffi::c_char);
            jw_number(w, (*s).control_pitch_trim);
            jw_raw(
                w,
                b",\"pitchAuthority\":\0" as *const u8 as *const ::core::ffi::c_char,
            );
            jw_number(w, (*s).control_pitch_authority);
            jw_raw(
                w,
                b",\"pitchAeroFraction\":\0" as *const u8 as *const ::core::ffi::c_char,
            );
            jw_number(w, (*s).control_pitch_aero_fraction);
            jw_char(w, '}' as i32 as ::core::ffi::c_char);
        } else {
            jw_raw(w, b"null\0" as *const u8 as *const ::core::ffi::c_char);
        }
        jw_char(w, '}' as i32 as ::core::ffi::c_char);
    }
    if (*d).groups & VEHICLE_LOG_GROUP_SURFACES as ::core::ffi::c_int as uint32_t != 0 {
        field_prefix(
            w,
            &raw mut first,
            b"controlSurfaces\0" as *const u8 as *const ::core::ffi::c_char,
        );
        jw_char(w, '{' as i32 as ::core::ffi::c_char);
        jw_raw(w, b"\"stateAvailable\":\0" as *const u8 as *const ::core::ffi::c_char);
        jw_bool(w, (*s).has_control_surface_state);
        jw_raw(w, b",\"sampleUT\":\0" as *const u8 as *const ::core::ffi::c_char);
        if (*s).has_control_surface_state {
            jw_number(w, (*s).control_surface_state_ut);
        } else {
            jw_null(w);
        }
        jw_raw(w, b",\"items\":[\0" as *const u8 as *const ::core::ffi::c_char);
        let mut i: size_t = 0 as size_t;
        while i < (*s).control_surface_count && i < LANDING_CONTROL_SURFACE_MAX as size_t
        {
            if i != 0 {
                jw_char(w, ',' as i32 as ::core::ffi::c_char);
            }
            let mut surface: *const VehicleLogSurfaceSample = (&raw const (*s)
                .control_surfaces as *const VehicleLogSurfaceSample)
                .offset(i as isize) as *const VehicleLogSurfaceSample;
            jw_char(w, '{' as i32 as ::core::ffi::c_char);
            jw_raw(w, b"\"partName\":\0" as *const u8 as *const ::core::ffi::c_char);
            jw_string(w, &raw const (*surface).part_name as *const ::core::ffi::c_char);
            jw_raw(w, b",\"role\":\0" as *const u8 as *const ::core::ffi::c_char);
            jw_string(w, &raw const (*surface).role as *const ::core::ffi::c_char);
            jw_raw(w, b",\"deployed\":\0" as *const u8 as *const ::core::ffi::c_char);
            jw_bool(w, (*surface).deployed);
            jw_raw(
                w,
                b",\"deflectionOverride\":\0" as *const u8 as *const ::core::ffi::c_char,
            );
            jw_bool(w, (*surface).deflection_override);
            jw_raw(
                w,
                b",\"overrideDeflectionNormalized\":\0" as *const u8
                    as *const ::core::ffi::c_char,
            );
            if (*surface).deflection_override {
                jw_number(w, (*surface).override_deflection);
            } else {
                jw_null(w);
            }
            jw_char(w, '}' as i32 as ::core::ffi::c_char);
            i = i.wrapping_add(1);
        }
        jw_raw(w, b"]}\0" as *const u8 as *const ::core::ffi::c_char);
    }
    if (*d).groups & VEHICLE_LOG_GROUP_RUNTIME as ::core::ffi::c_int as uint32_t != 0 {
        field_prefix(
            w,
            &raw mut first,
            b"runtime\0" as *const u8 as *const ::core::ffi::c_char,
        );
        jw_char(w, '{' as i32 as ::core::ffi::c_char);
        jw_raw(
            w,
            b"\"loopWallDeltaMilliseconds\":\0" as *const u8
                as *const ::core::ffi::c_char,
        );
        if (*s).has_loop_wall_delta {
            jw_number(w, (*s).loop_wall_delta_ms);
        } else {
            jw_raw(w, b"null\0" as *const u8 as *const ::core::ffi::c_char);
        }
        jw_raw(
            w,
            b",\"telemetryLatencyMilliseconds\":\0" as *const u8
                as *const ::core::ffi::c_char,
        );
        if (*s).has_telemetry_latency {
            jw_number(w, (*s).telemetry_latency_ms);
        } else {
            jw_raw(w, b"null\0" as *const u8 as *const ::core::ffi::c_char);
        }
        jw_raw(
            w,
            b",\"guidanceComputeMilliseconds\":\0" as *const u8
                as *const ::core::ffi::c_char,
        );
        if (*s).has_guidance_compute {
            jw_number(w, (*s).guidance_compute_ms);
        } else {
            jw_raw(w, b"null\0" as *const u8 as *const ::core::ffi::c_char);
        }
        jw_raw(
            w,
            b",\"applyLatencyMilliseconds\":\0" as *const u8
                as *const ::core::ffi::c_char,
        );
        if (*s).has_apply_latency {
            jw_number(w, (*s).apply_latency_ms);
        } else {
            jw_raw(w, b"null\0" as *const u8 as *const ::core::ffi::c_char);
        }
        jw_raw(
            w,
            b",\"controlLoopMilliseconds\":\0" as *const u8 as *const ::core::ffi::c_char,
        );
        if (*s).has_control_loop {
            jw_number(w, (*s).control_loop_ms);
        } else {
            jw_raw(w, b"null\0" as *const u8 as *const ::core::ffi::c_char);
        }
        jw_raw(w, b",\"warpRate\":\0" as *const u8 as *const ::core::ffi::c_char);
        if (*s).has_warp {
            jw_number(w, (*s).warp_rate);
        } else {
            jw_raw(w, b"null\0" as *const u8 as *const ::core::ffi::c_char);
        }
        jw_char(w, '}' as i32 as ::core::ffi::c_char);
    }
    if (*d).groups & VEHICLE_LOG_GROUP_GUIDANCE as ::core::ffi::c_int as uint32_t != 0 {
        field_prefix(
            w,
            &raw mut first,
            b"guidance\0" as *const u8 as *const ::core::ffi::c_char,
        );
        jw_char(w, '{' as i32 as ::core::ffi::c_char);
        jw_raw(w, b"\"rangeToSite\":\0" as *const u8 as *const ::core::ffi::c_char);
        jw_number(w, (*s).range_to_site);
        jw_raw(w, b",\"bearingToSite\":\0" as *const u8 as *const ::core::ffi::c_char);
        jw_number(w, (*s).bearing_to_site);
        jw_raw(
            w,
            b",\"runwayAlongTrack\":\0" as *const u8 as *const ::core::ffi::c_char,
        );
        jw_number(w, (*s).runway_along_track);
        jw_raw(
            w,
            b",\"runwayCrossTrack\":\0" as *const u8 as *const ::core::ffi::c_char,
        );
        jw_number(w, (*s).runway_cross_track);
        jw_raw(
            w,
            b",\"specificMechanicalEnergy\":\0" as *const u8
                as *const ::core::ffi::c_char,
        );
        jw_number(w, (*s).specific_mechanical_energy);
        jw_raw(
            w,
            b",\"entryReversalTimeRemaining\":\0" as *const u8
                as *const ::core::ffi::c_char,
        );
        jw_number(w, (*s).entry_reversal_time_remaining);
        jw_raw(w, b",\"entryReversalUT\":\0" as *const u8 as *const ::core::ffi::c_char);
        if (*s).entry_reversal_scheduled {
            jw_number(w, (*s).entry_reversal_ut);
        } else {
            jw_raw(w, b"null\0" as *const u8 as *const ::core::ffi::c_char);
        }
        jw_raw(
            w,
            b",\"entryReversalRange\":\0" as *const u8 as *const ::core::ffi::c_char,
        );
        if (*s).entry_reversal_scheduled {
            jw_number(w, (*s).entry_reversal_range);
        } else {
            jw_raw(w, b"null\0" as *const u8 as *const ::core::ffi::c_char);
        }
        jw_raw(
            w,
            b",\"entryReversalSign\":\0" as *const u8 as *const ::core::ffi::c_char,
        );
        if (*s).entry_reversal_scheduled {
            jw_number(w, (*s).entry_reversal_sign);
        } else {
            jw_raw(w, b"null\0" as *const u8 as *const ::core::ffi::c_char);
        }
        jw_char(w, '}' as i32 as ::core::ffi::c_char);
    }
    jw_raw(w, b"}}\0" as *const u8 as *const ::core::ffi::c_char);
}
