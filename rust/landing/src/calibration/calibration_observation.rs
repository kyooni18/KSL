use crate::sys::*;
extern "C" {
    fn expm1(__x: ::core::ffi::c_double) -> ::core::ffi::c_double;
    fn hypot(
        __x: ::core::ffi::c_double,
        __y: ::core::ffi::c_double,
    ) -> ::core::ffi::c_double;
    fn fabs(__x: ::core::ffi::c_double) -> ::core::ffi::c_double;
    fn fmax(
        __x: ::core::ffi::c_double,
        __y: ::core::ffi::c_double,
    ) -> ::core::ffi::c_double;
    fn fmin(
        __x: ::core::ffi::c_double,
        __y: ::core::ffi::c_double,
    ) -> ::core::ffi::c_double;
    fn decision_margin(
        available: ::core::ffi::c_double,
        required: ::core::ffi::c_double,
    ) -> DecisionMargin;
}
pub type size_t = usize;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct Vector3 {
    pub x: ::core::ffi::c_double,
    pub y: ::core::ffi::c_double,
    pub z: ::core::ffi::c_double,
}
pub type NavballSpeedMode = ::core::ffi::c_uint;
pub const SPEED_SURFACE: NavballSpeedMode = 2;
pub const SPEED_ORBIT: NavballSpeedMode = 1;
pub const SPEED_UNCHANGED: NavballSpeedMode = 0;
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
pub struct CalibrationSettings {
    pub enable_passive_calibration: bool,
    pub enable_trajectory_calibration: bool,
    pub auto_apply_in_flight: bool,
    pub minimum_dynamic_pressure: ::core::ffi::c_double,
    pub maximum_sample_dynamic_pressure: ::core::ffi::c_double,
    pub maximum_sideslip: ::core::ffi::c_double,
    pub minimum_calibration_radar_altitude: ::core::ffi::c_double,
    pub maximum_calibration_dynamic_pressure: ::core::ffi::c_double,
    pub maximum_calibration_g_load: ::core::ffi::c_double,
    pub maximum_calibration_sink_rate: ::core::ffi::c_double,
    pub abort_stall_fraction: ::core::ffi::c_double,
    pub minimum_angle_of_attack: ::core::ffi::c_double,
    pub maximum_angle_of_attack: ::core::ffi::c_double,
    pub angle_of_attack_step: ::core::ffi::c_double,
    pub settling_duration: ::core::ffi::c_double,
    pub sampling_duration: ::core::ffi::c_double,
    pub trajectory_learning_rate: ::core::ffi::c_double,
}
#[derive(Copy, Clone)]
#[repr(C)]
pub struct AttitudeResponseModel {
    pub pitch_valid: bool,
    pub roll_valid: bool,
    pub pitch_natural_frequency_s_inv: ::core::ffi::c_double,
    pub pitch_damping_ratio: ::core::ffi::c_double,
    pub roll_natural_frequency_s_inv: ::core::ffi::c_double,
    pub roll_damping_ratio: ::core::ffi::c_double,
    pub maximum_pitch_rate_deg_s: ::core::ffi::c_double,
    pub maximum_roll_rate_deg_s: ::core::ffi::c_double,
    pub maximum_pitch_accel_deg_s2: ::core::ffi::c_double,
    pub maximum_roll_accel_deg_s2: ::core::ffi::c_double,
}
#[derive(Copy, Clone)]
#[repr(C)]
pub struct TelemetryControlSurface {
    pub part_name: [::core::ffi::c_char; 64],
    pub role: [::core::ffi::c_char; 16],
    pub deployed: bool,
    pub deflection_override: bool,
    pub override_deflection: ::core::ffi::c_double,
}
#[derive(Copy, Clone)]
#[repr(C)]
pub struct Telemetry {
    pub ut: ::core::ffi::c_double,
    pub vessel_name: [::core::ffi::c_char; 128],
    pub latitude: ::core::ffi::c_double,
    pub longitude: ::core::ffi::c_double,
    pub mean_altitude: ::core::ffi::c_double,
    pub radar_altitude: ::core::ffi::c_double,
    pub vertical_speed: ::core::ffi::c_double,
    pub horizontal_speed: ::core::ffi::c_double,
    pub surface_speed: ::core::ffi::c_double,
    pub true_air_speed: ::core::ffi::c_double,
    pub atmospheric_density: ::core::ffi::c_double,
    pub speed_of_sound: ::core::ffi::c_double,
    pub mach: ::core::ffi::c_double,
    pub heading: ::core::ffi::c_double,
    pub ground_track_heading: ::core::ffi::c_double,
    pub course_to_site_error: ::core::ffi::c_double,
    pub pitch: ::core::ffi::c_double,
    pub roll: ::core::ffi::c_double,
    pub pitch_rate: ::core::ffi::c_double,
    pub roll_rate: ::core::ffi::c_double,
    pub heading_rate: ::core::ffi::c_double,
    pub angle_of_attack_rate: ::core::ffi::c_double,
    pub has_angle_of_attack_rate: bool,
    pub has_body_pitch_rate: bool,
    pub has_body_roll_rate: bool,
    pub has_body_yaw_rate: bool,
    pub has_course_rate: bool,
    pub body_pitch_rate: ::core::ffi::c_double,
    pub body_roll_rate: ::core::ffi::c_double,
    pub body_yaw_rate: ::core::ffi::c_double,
    pub course_rate: ::core::ffi::c_double,
    pub autopilot_error: ::core::ffi::c_double,
    pub has_attitude_quaternion: bool,
    pub attitude_quaternion: [::core::ffi::c_double; 4],
    pub attitude_reference_frame: [::core::ffi::c_char; 48],
    pub has_command_pitch_error: bool,
    pub has_command_roll_error: bool,
    pub has_command_heading_error: bool,
    pub command_pitch_error: ::core::ffi::c_double,
    pub command_roll_error: ::core::ffi::c_double,
    pub command_heading_error: ::core::ffi::c_double,
    pub has_pid_gains: bool,
    pub autopilot_pitch_pid_gains: [::core::ffi::c_double; 3],
    pub autopilot_roll_pid_gains: [::core::ffi::c_double; 3],
    pub autopilot_yaw_pid_gains: [::core::ffi::c_double; 3],
    pub navball_speed_mode: NavballSpeedMode,
    pub attitude_response: AttitudeResponseModel,
    pub angle_of_attack: ::core::ffi::c_double,
    pub sideslip: ::core::ffi::c_double,
    pub dynamic_pressure: ::core::ffi::c_double,
    pub static_pressure: ::core::ffi::c_double,
    pub g_force: ::core::ffi::c_double,
    pub stall_fraction: ::core::ffi::c_double,
    pub lift_force: ::core::ffi::c_double,
    pub drag_force: ::core::ffi::c_double,
    pub stall_fraction_is_measured: bool,
    pub has_force_vectors: bool,
    pub has_center_of_mass: bool,
    pub has_center_of_mass_root: bool,
    pub physics_sample_valid: bool,
    pub center_of_mass_root: Vector3,
    pub lift_vector: Vector3,
    pub drag_vector: Vector3,
    pub center_of_mass: Vector3,
    pub dry_mass: ::core::ffi::c_double,
    pub physics_confidence: ::core::ffi::c_double,
    pub physics_authority: [::core::ffi::c_double; 3],
    pub physics_authority_confidence: [::core::ffi::c_double; 3],
    pub physics_mass_flow: ::core::ffi::c_double,
    pub physics_model_residual: ::core::ffi::c_double,
    pub physics_model_residual_confidence: ::core::ffi::c_double,
    pub physics_certified_uncertainty: ::core::ffi::c_double,
    pub physics_force_residual_per_q: Vector3,
    pub physics_force_residual_sigma_per_q: Vector3,
    pub physics_force_residual_confidence: ::core::ffi::c_double,
    pub physics_airbrake_model_available: bool,
    pub physics_airbrake_model_confidence: ::core::ffi::c_double,
    pub physics_airbrake_drag_accel: ::core::ffi::c_double,
    pub physics_samples: ::core::ffi::c_uint,
    pub physics_live_samples: ::core::ffi::c_uint,
    pub predicted_physics_observed_seconds: ::core::ffi::c_double,
    pub predicted_physics_fallback_seconds: ::core::ffi::c_double,
    pub predicted_shadow_guidance: bool,
    pub predicted_terminal_policy_feasible: bool,
    pub predicted_terminal_survivability_stress_score: ::core::ffi::c_double,
    pub predicted_physics_relative_uncertainty: ::core::ffi::c_double,
    pub predicted_uncertainty_scenarios: ::core::ffi::c_uint,
    pub predicted_raw_published_position_delta_30s: ::core::ffi::c_double,
    pub predicted_raw_published_position_delta_60s: ::core::ffi::c_double,
    pub predicted_raw_published_position_delta_120s: ::core::ffi::c_double,
    pub mass: ::core::ffi::c_double,
    pub available_thrust: ::core::ffi::c_double,
    pub current_thrust: ::core::ffi::c_double,
    pub apoapsis_altitude: ::core::ffi::c_double,
    pub periapsis_altitude: ::core::ffi::c_double,
    pub orbit_period: ::core::ffi::c_double,
    pub throttle: ::core::ffi::c_double,
    pub has_controls: bool,
    pub control_pitch: ::core::ffi::c_double,
    pub control_roll: ::core::ffi::c_double,
    pub control_yaw: ::core::ffi::c_double,
    pub has_torque: bool,
    pub available_pitch_torque: ::core::ffi::c_double,
    pub available_roll_torque: ::core::ffi::c_double,
    pub available_yaw_torque: ::core::ffi::c_double,
    pub has_inertia: bool,
    pub pitch_moment_of_inertia: ::core::ffi::c_double,
    pub roll_moment_of_inertia: ::core::ffi::c_double,
    pub yaw_moment_of_inertia: ::core::ffi::c_double,
    pub has_loop_wall_delta: bool,
    pub has_telemetry_latency: bool,
    pub has_guidance_compute: bool,
    pub has_apply_latency: bool,
    pub has_control_loop: bool,
    pub has_warp: bool,
    pub warp_rails: bool,
    pub physics_warp_factor: ::core::ffi::c_int,
    pub warp_rate: ::core::ffi::c_double,
    pub loop_wall_delta_ms: ::core::ffi::c_double,
    pub telemetry_latency_ms: ::core::ffi::c_double,
    pub guidance_compute_ms: ::core::ffi::c_double,
    pub apply_latency_ms: ::core::ffi::c_double,
    pub control_loop_ms: ::core::ffi::c_double,
    pub has_rpc_budget: bool,
    pub rpc_read_calls: ::core::ffi::c_uint,
    pub rpc_read_wire_requests: ::core::ffi::c_uint,
    pub rpc_apply_calls: ::core::ffi::c_uint,
    pub rpc_apply_wire_requests: ::core::ffi::c_uint,
    pub rpc_total_calls: ::core::ffi::c_uint,
    pub rpc_total_wire_requests: ::core::ffi::c_uint,
    pub gear: bool,
    pub brakes: bool,
    pub has_airbrakes: bool,
    pub airbrakes: bool,
    pub has_sas_state: bool,
    pub sas_enabled: bool,
    pub has_control_surface_state: bool,
    pub control_surface_state_ut: ::core::ffi::c_double,
    pub control_surface_count: size_t,
    pub control_surfaces: [TelemetryControlSurface; 16],
    pub has_main_gear_grounded: bool,
    pub main_gear_grounded: bool,
    pub has_nose_gear_grounded: bool,
    pub nose_gear_grounded: bool,
    pub vessel_situation: [::core::ffi::c_char; 64],
    pub range_to_site: ::core::ffi::c_double,
    pub bearing_to_site: ::core::ffi::c_double,
    pub heading_error: ::core::ffi::c_double,
    pub runway_along_track: ::core::ffi::c_double,
    pub runway_cross_track: ::core::ffi::c_double,
    pub flight_path_angle: ::core::ffi::c_double,
    pub estimated_lift_to_drag: ::core::ffi::c_double,
    pub estimated_ballistic_coefficient: ::core::ffi::c_double,
    pub aerodynamic_confidence: ::core::ffi::c_double,
    pub calibrated_best_glide_angle_of_attack: ::core::ffi::c_double,
    pub calibrated_stall_speed: ::core::ffi::c_double,
    pub predicted_miss_distance: ::core::ffi::c_double,
    pub predicted_taem_distance: ::core::ffi::c_double,
    pub predicted_taem_range_error: ::core::ffi::c_double,
    pub predicted_entry_range: ::core::ffi::c_double,
    pub predicted_entry_flight_path_angle: ::core::ffi::c_double,
    pub predicted_taem_speed: ::core::ffi::c_double,
    pub predicted_taem_energy_error: ::core::ffi::c_double,
    pub predicted_peak_dynamic_pressure: ::core::ffi::c_double,
    pub predicted_peak_g_load: ::core::ffi::c_double,
    pub predicted_s_turn_reversals: ::core::ffi::c_double,
    pub trajectory_density_scale: ::core::ffi::c_double,
    pub trajectory_drag_scale: ::core::ffi::c_double,
    pub trajectory_lift_scale: ::core::ffi::c_double,
    pub bank_effectiveness: ::core::ffi::c_double,
    pub trajectory_calibration_confidence: ::core::ffi::c_double,
    pub trajectory_altitude_residual: ::core::ffi::c_double,
    pub trajectory_speed_residual: ::core::ffi::c_double,
    pub trajectory_range_residual: ::core::ffi::c_double,
    pub energy_excess_range: ::core::ffi::c_double,
}
#[derive(Copy, Clone)]
#[repr(C)]
pub struct DecisionMargin {
    pub valid: bool,
    pub available: ::core::ffi::c_double,
    pub required: ::core::ffi::c_double,
    pub margin: ::core::ffi::c_double,
    pub normalized_margin: ::core::ffi::c_double,
}
#[derive(Copy, Clone)]
#[repr(C)]
pub struct CalibrationObservationEnvelope {
    pub valid: bool,
    pub airborne: bool,
    pub clean_configuration: bool,
    pub actuator_domain_valid: bool,
    pub unpowered: bool,
    pub force_observable: bool,
    pub passive_aero_usable: bool,
    pub trajectory_usable: bool,
    pub dynamic_pressure_minimum: DecisionMargin,
    pub dynamic_pressure_maximum: DecisionMargin,
    pub incidence: DecisionMargin,
    pub sideslip: DecisionMargin,
    pub load: DecisionMargin,
    pub stall: DecisionMargin,
    pub thrust: DecisionMargin,
    pub incidence_deg: ::core::ffi::c_double,
    pub aerodynamic_accel_mps2: ::core::ffi::c_double,
    pub aerodynamic_response_time_s: ::core::ffi::c_double,
    pub information_increment: ::core::ffi::c_double,
    pub information_blend: ::core::ffi::c_double,
    pub worst_normalized_margin: ::core::ffi::c_double,
}
#[derive(Copy, Clone)]
#[repr(C)]
pub struct CalibrationSafetyEnvelope {
    pub valid: bool,
    pub safe: bool,
    pub altitude: DecisionMargin,
    pub dynamic_pressure: DecisionMargin,
    pub load: DecisionMargin,
    pub sink_rate: DecisionMargin,
    pub stall: DecisionMargin,
    pub speed: DecisionMargin,
    pub worst_normalized_margin: ::core::ffi::c_double,
}
unsafe extern "C" fn minimum_valid_normalized_margin(
    mut margins: *const DecisionMargin,
    mut count: size_t,
) -> ::core::ffi::c_double {
    let mut worst: ::core::ffi::c_double = ::core::f32::INFINITY
        as ::core::ffi::c_double;
    let mut any: bool = false_0 != 0;
    let mut i: size_t = 0 as size_t;
    while i < count {
        if (*margins.offset(i as isize)).valid {
            worst = fmin(worst, (*margins.offset(i as isize)).normalized_margin);
            any = true_0 != 0;
        }
        i = i.wrapping_add(1);
    }
    return if any as ::core::ffi::c_int != 0 {
        worst
    } else {
        -::core::f32::INFINITY as ::core::ffi::c_double
    };
}
unsafe extern "C" fn margin_nonnegative(mut margin: DecisionMargin) -> bool {
    return margin.valid as ::core::ffi::c_int != 0 && margin.margin >= 0.0f64;
}
unsafe extern "C" fn telemetry_airborne(mut t: *const Telemetry) -> bool {
    if t.is_null() {
        return false_0 != 0;
    }
    return strcasecmp(
        &raw const (*t).vessel_situation as *const ::core::ffi::c_char,
        b"landed\0" as *const u8 as *const ::core::ffi::c_char,
    ) != 0
        && strcasecmp(
            &raw const (*t).vessel_situation as *const ::core::ffi::c_char,
            b"splashed\0" as *const u8 as *const ::core::ffi::c_char,
        ) != 0
        && strcasecmp(
            &raw const (*t).vessel_situation as *const ::core::ffi::c_char,
            b"pre-launch\0" as *const u8 as *const ::core::ffi::c_char,
        ) != 0;
}
#[no_mangle]
pub unsafe extern "C" fn calibration_observation_envelope(
    mut t: *const Telemetry,
    mut v: *const VehicleProfile,
    mut s: *const CalibrationSettings,
    mut dt: ::core::ffi::c_double,
) -> CalibrationObservationEnvelope {
    let mut out: CalibrationObservationEnvelope = CalibrationObservationEnvelope {
        valid: 0 as ::core::ffi::c_int != 0,
        airborne: false,
        clean_configuration: false,
        actuator_domain_valid: false,
        unpowered: false,
        force_observable: false,
        passive_aero_usable: false,
        trajectory_usable: false,
        dynamic_pressure_minimum: DecisionMargin {
            valid: false,
            available: 0.,
            required: 0.,
            margin: 0.,
            normalized_margin: 0.,
        },
        dynamic_pressure_maximum: DecisionMargin {
            valid: false,
            available: 0.,
            required: 0.,
            margin: 0.,
            normalized_margin: 0.,
        },
        incidence: DecisionMargin {
            valid: false,
            available: 0.,
            required: 0.,
            margin: 0.,
            normalized_margin: 0.,
        },
        sideslip: DecisionMargin {
            valid: false,
            available: 0.,
            required: 0.,
            margin: 0.,
            normalized_margin: 0.,
        },
        load: DecisionMargin {
            valid: false,
            available: 0.,
            required: 0.,
            margin: 0.,
            normalized_margin: 0.,
        },
        stall: DecisionMargin {
            valid: false,
            available: 0.,
            required: 0.,
            margin: 0.,
            normalized_margin: 0.,
        },
        thrust: DecisionMargin {
            valid: false,
            available: 0.,
            required: 0.,
            margin: 0.,
            normalized_margin: 0.,
        },
        incidence_deg: 0.,
        aerodynamic_accel_mps2: 0.,
        aerodynamic_response_time_s: 0.,
        information_increment: 0.,
        information_blend: 0.,
        worst_normalized_margin: 0.,
    };
    if t.is_null() || v.is_null() || s.is_null() || dt.is_finite() as i32 == 0
        || dt < 0.0f64
    {
        return out;
    }
    out.airborne = telemetry_airborne(t);
    out.clean_configuration = (*t).has_airbrakes as ::core::ffi::c_int != 0
        && !(*t).airbrakes && !(*t).gear && !(*t).brakes;
    let mut maximum_control: ::core::ffi::c_double = 0.0f64;
    if (*t).has_controls {
        maximum_control = fmax(
            fabs((*t).control_pitch),
            fmax(fabs((*t).control_roll), fabs((*t).control_yaw)),
        );
    }
    out.actuator_domain_valid = !(*t).has_controls
        || maximum_control.is_finite() as i32 != 0 && maximum_control < 1.0f64;
    let mut thrust: ::core::ffi::c_double = if (*t).current_thrust.is_finite() as i32
        != 0
    {
        fmax(0.0f64, (*t).current_thrust)
    } else {
        ::core::f32::INFINITY as ::core::ffi::c_double
    };
    out.thrust = decision_margin(0.0f64, thrust);
    out.unpowered = margin_nonnegative(out.thrust);
    let mut q_limit: ::core::ffi::c_double = fmin(
        (*s).maximum_sample_dynamic_pressure,
        (*v).maximum_dynamic_pressure,
    );
    out.dynamic_pressure_minimum = decision_margin(
        (*t).dynamic_pressure,
        (*s).minimum_dynamic_pressure,
    );
    out.dynamic_pressure_maximum = decision_margin(q_limit, (*t).dynamic_pressure);
    out.incidence_deg = hypot((*t).angle_of_attack, (*t).sideslip);
    out.incidence = decision_margin((*v).maximum_angle_of_attack, out.incidence_deg);
    out.sideslip = decision_margin((*s).maximum_sideslip, fabs((*t).sideslip));
    out.load = decision_margin((*v).maximum_g_load, (*t).g_force);
    out.stall = decision_margin((*s).abort_stall_fraction, (*t).stall_fraction);
    let mut aerodynamic_force: ::core::ffi::c_double = hypot(
        (*t).lift_force,
        (*t).drag_force,
    );
    out.force_observable = (*t).has_force_vectors as ::core::ffi::c_int != 0
        && (*t).mass > 0.0f64 && (*t).mass.is_finite() as i32 != 0
        && (*t).true_air_speed > 0.0f64 && (*t).true_air_speed.is_finite() as i32 != 0
        && (*t).drag_force > 0.0f64 && (*t).drag_force.is_finite() as i32 != 0
        && (*t).lift_force.is_finite() as i32 != 0
        && aerodynamic_force.is_finite() as i32 != 0 && aerodynamic_force > 0.0f64;
    if out.force_observable {
        out.aerodynamic_accel_mps2 = aerodynamic_force / (*t).mass;
        if out.aerodynamic_accel_mps2 > DBL_MIN {
            out.aerodynamic_response_time_s = (*t).true_air_speed
                / out.aerodynamic_accel_mps2;
            if out.aerodynamic_response_time_s > DBL_MIN
                && out.aerodynamic_response_time_s.is_finite() as i32 != 0
            {
                out.information_increment = dt / out.aerodynamic_response_time_s;
                out.information_blend = -expm1(-out.information_increment);
            }
        }
    }
    let mut margins: [DecisionMargin; 7] = [
        out.dynamic_pressure_minimum,
        out.dynamic_pressure_maximum,
        out.incidence,
        out.sideslip,
        out.load,
        out.stall,
        out.thrust,
    ];
    out.worst_normalized_margin = minimum_valid_normalized_margin(
        &raw mut margins as *mut DecisionMargin,
        (::core::mem::size_of::<[DecisionMargin; 7]>() as size_t)
            .wrapping_div(::core::mem::size_of::<DecisionMargin>() as size_t),
    );
    let mut inside_model_domain: bool = margin_nonnegative(out.dynamic_pressure_minimum)
        as ::core::ffi::c_int != 0
        && margin_nonnegative(out.dynamic_pressure_maximum) as ::core::ffi::c_int != 0
        && margin_nonnegative(out.incidence) as ::core::ffi::c_int != 0
        && margin_nonnegative(out.sideslip) as ::core::ffi::c_int != 0
        && margin_nonnegative(out.load) as ::core::ffi::c_int != 0
        && margin_nonnegative(out.stall) as ::core::ffi::c_int != 0;
    out.valid = (*t).physics_sample_valid as ::core::ffi::c_int != 0
        && out.airborne as ::core::ffi::c_int != 0
        && out.actuator_domain_valid as ::core::ffi::c_int != 0
        && out.unpowered as ::core::ffi::c_int != 0
        && inside_model_domain as ::core::ffi::c_int != 0
        && out.information_increment.is_finite() as i32 != 0
        && out.information_blend.is_finite() as i32 != 0;
    out.passive_aero_usable = out.valid as ::core::ffi::c_int != 0
        && out.clean_configuration as ::core::ffi::c_int != 0
        && out.force_observable as ::core::ffi::c_int != 0;
    out.trajectory_usable = out.valid as ::core::ffi::c_int != 0
        && out.clean_configuration as ::core::ffi::c_int != 0
        && out.force_observable as ::core::ffi::c_int != 0;
    return out;
}
#[no_mangle]
pub unsafe extern "C" fn calibration_safety_envelope(
    mut t: *const Telemetry,
    mut v: *const VehicleProfile,
    mut s: *const CalibrationSettings,
) -> CalibrationSafetyEnvelope {
    let mut out: CalibrationSafetyEnvelope = CalibrationSafetyEnvelope {
        valid: 0 as ::core::ffi::c_int != 0,
        safe: false,
        altitude: DecisionMargin {
            valid: false,
            available: 0.,
            required: 0.,
            margin: 0.,
            normalized_margin: 0.,
        },
        dynamic_pressure: DecisionMargin {
            valid: false,
            available: 0.,
            required: 0.,
            margin: 0.,
            normalized_margin: 0.,
        },
        load: DecisionMargin {
            valid: false,
            available: 0.,
            required: 0.,
            margin: 0.,
            normalized_margin: 0.,
        },
        sink_rate: DecisionMargin {
            valid: false,
            available: 0.,
            required: 0.,
            margin: 0.,
            normalized_margin: 0.,
        },
        stall: DecisionMargin {
            valid: false,
            available: 0.,
            required: 0.,
            margin: 0.,
            normalized_margin: 0.,
        },
        speed: DecisionMargin {
            valid: false,
            available: 0.,
            required: 0.,
            margin: 0.,
            normalized_margin: 0.,
        },
        worst_normalized_margin: 0.,
    };
    if t.is_null() || v.is_null() || s.is_null() {
        return out;
    }
    let mut q_limit: ::core::ffi::c_double = fmin(
        (*s).maximum_calibration_dynamic_pressure,
        (*v).maximum_dynamic_pressure,
    );
    let mut load_limit: ::core::ffi::c_double = fmin(
        (*s).maximum_calibration_g_load,
        (*v).maximum_g_load,
    );
    let mut downward_speed: ::core::ffi::c_double = fmax(0.0f64, -(*t).vertical_speed);
    out.altitude = decision_margin(
        (*t).radar_altitude,
        (*s).minimum_calibration_radar_altitude,
    );
    out.dynamic_pressure = decision_margin(q_limit, (*t).dynamic_pressure);
    out.load = decision_margin(load_limit, (*t).g_force);
    out.sink_rate = decision_margin((*s).maximum_calibration_sink_rate, downward_speed);
    out.stall = decision_margin((*s).abort_stall_fraction, (*t).stall_fraction);
    out.speed = decision_margin((*t).true_air_speed, (*v).minimum_safe_speed);
    let mut margins: [DecisionMargin; 6] = [
        out.altitude,
        out.dynamic_pressure,
        out.load,
        out.sink_rate,
        out.stall,
        out.speed,
    ];
    out.worst_normalized_margin = minimum_valid_normalized_margin(
        &raw mut margins as *mut DecisionMargin,
        (::core::mem::size_of::<[DecisionMargin; 6]>() as size_t)
            .wrapping_div(::core::mem::size_of::<DecisionMargin>() as size_t),
    );
    out.valid = telemetry_airborne(t);
    out.safe = out.valid as ::core::ffi::c_int != 0
        && margin_nonnegative(out.altitude) as ::core::ffi::c_int != 0
        && margin_nonnegative(out.dynamic_pressure) as ::core::ffi::c_int != 0
        && margin_nonnegative(out.load) as ::core::ffi::c_int != 0
        && margin_nonnegative(out.sink_rate) as ::core::ffi::c_int != 0
        && margin_nonnegative(out.stall) as ::core::ffi::c_int != 0
        && margin_nonnegative(out.speed) as ::core::ffi::c_int != 0;
    return out;
}
pub const DBL_MIN: ::core::ffi::c_double = __DBL_MIN__;
pub const __DBL_MIN__: ::core::ffi::c_double = 2.2250738585072014e-308f64;
pub const true_0: ::core::ffi::c_int = 1 as ::core::ffi::c_int;
pub const false_0: ::core::ffi::c_int = 0 as ::core::ffi::c_int;
