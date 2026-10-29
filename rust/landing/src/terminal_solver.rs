use crate::sys::*;
extern "C" {
    fn world_inertial_to_fixed(
        w: *const KerbinWorld,
        inertial: Vec3,
        ut: ::core::ffi::c_double,
    ) -> Vec3;
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
    fn terminal_model_validate(
        model: *const TerminalModel,
        reason: *mut ::core::ffi::c_char,
        reason_size: size_t,
    ) -> bool;
    fn terminal_state_validate(
        state: *const TerminalDynamicState,
        reason: *mut ::core::ffi::c_char,
        reason_size: size_t,
    ) -> bool;
    fn taem_inertial_to_fixed(
        world: *const TaemFrameWorld,
        position_i: TaemVec3,
        velocity_i: TaemVec3,
        ut: ::core::ffi::c_double,
        position_b: *mut TaemVec3,
        velocity_b: *mut TaemVec3,
    ) -> bool;
    fn taem_energy_diagnostic(
        world: *const TaemFrameWorld,
        position_i: TaemVec3,
        velocity_i: TaemVec3,
        velocity_b: TaemVec3,
        air_velocity_b: TaemVec3,
        aerodynamic_force_i: TaemVec3,
        aerodynamic_force_b: TaemVec3,
        mass_kg: ::core::ffi::c_double,
        drag_n: ::core::ffi::c_double,
        diagnostic: *mut TaemEnergyDiagnostic,
    ) -> bool;
    fn taem_geometry_world(
        model: *const TerminalModel,
        world: *mut TaemFrameWorld,
    ) -> bool;
    fn taem_geometry_state(
        model: *const TerminalModel,
        state: *const TerminalDynamicState,
        geometry: *mut TaemGeometryState,
    ) -> bool;
    fn asin(__x: ::core::ffi::c_double) -> ::core::ffi::c_double;
    fn atan2(
        __y: ::core::ffi::c_double,
        __x: ::core::ffi::c_double,
    ) -> ::core::ffi::c_double;
    fn sin(__x: ::core::ffi::c_double) -> ::core::ffi::c_double;
    fn tan(__x: ::core::ffi::c_double) -> ::core::ffi::c_double;
    fn hypot(
        __x: ::core::ffi::c_double,
        __y: ::core::ffi::c_double,
    ) -> ::core::ffi::c_double;
    fn ceil(__x: ::core::ffi::c_double) -> ::core::ffi::c_double;
    fn fabs(__x: ::core::ffi::c_double) -> ::core::ffi::c_double;
    fn remainder(
        __x: ::core::ffi::c_double,
        __y: ::core::ffi::c_double,
    ) -> ::core::ffi::c_double;
    fn round(__x: ::core::ffi::c_double) -> ::core::ffi::c_double;
    fn fmax(
        __x: ::core::ffi::c_double,
        __y: ::core::ffi::c_double,
    ) -> ::core::ffi::c_double;
    fn fmin(
        __x: ::core::ffi::c_double,
        __y: ::core::ffi::c_double,
    ) -> ::core::ffi::c_double;
    fn terminal_propagator_step(
        model: *const TerminalModel,
        state: *mut TerminalDynamicState,
        control: *const TerminalControl,
    ) -> TerminalStepStatus;
    fn taem_tracker_update(
        model: *const TerminalModel,
        state: *const TerminalDynamicState,
        geometry: *const TaemGeometryState,
        reference: *const TaemPathReference,
        dt_s: ::core::ffi::c_double,
    ) -> TaemTrackerOutput;
    fn taem_tracker_lateral_demand(
        model: *const TerminalModel,
        geometry: *const TaemGeometryState,
        reference: *const TaemPathReference,
    ) -> ::core::ffi::c_double;
    fn taem_route_limit_initial_vertical_authority(
        model: *const TerminalModel,
        state: *const TerminalDynamicState,
        geometry: *const TaemGeometryState,
        route: *mut TaemRoute,
    ) -> bool;
    fn taem_route_planned_speed(
        route: *const TaemRoute,
        station_m: ::core::ffi::c_double,
    ) -> ::core::ffi::c_double;
    fn taem_route_reference(
        route: *const TaemRoute,
        state: *const TaemGeometryState,
        cursor: *mut size_t,
        reference: *mut TaemPathReference,
        point_index: *mut size_t,
    ) -> bool;
    fn v3_norm(a: Vec3) -> ::core::ffi::c_double;
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
pub struct TaemGeometryState {
    pub runway_along_m: ::core::ffi::c_double,
    pub runway_cross_m: ::core::ffi::c_double,
    pub altitude_above_runway_m: ::core::ffi::c_double,
    pub ground_speed_mps: ::core::ffi::c_double,
    pub airspeed_mps: ::core::ffi::c_double,
    pub course_deg: ::core::ffi::c_double,
    pub flight_path_angle_deg: ::core::ffi::c_double,
    pub heading_error_deg: ::core::ffi::c_double,
    pub curvature_right_per_m: ::core::ffi::c_double,
}
#[derive(Copy, Clone)]
#[repr(C)]
pub struct TaemPoint2 {
    pub x: ::core::ffi::c_double,
    pub y: ::core::ffi::c_double,
}
#[derive(Copy, Clone)]
#[repr(C)]
pub struct TaemFixedHacGeometry {
    pub entry: TaemPoint2,
    pub center: TaemPoint2,
    pub exit: TaemPoint2,
    pub radius_m: ::core::ffi::c_double,
    pub side: ::core::ffi::c_double,
    pub capture_course_deg: ::core::ffi::c_double,
    pub arc_sweep_rad: ::core::ffi::c_double,
    pub arc_length_m: ::core::ffi::c_double,
    pub final_length_m: ::core::ffi::c_double,
    pub total_length_m: ::core::ffi::c_double,
}
#[derive(Copy, Clone)]
#[repr(C)]
pub struct TaemRoute {
    pub valid: bool,
    pub count: size_t,
    pub lead_count: size_t,
    pub arc_count: size_t,
    pub rollout_count: size_t,
    pub length_m: ::core::ffi::c_double,
    pub side: ::core::ffi::c_double,
    pub hac: TaemFixedHacGeometry,
    pub p0_along_m: ::core::ffi::c_double,
    pub p0_cross_m: ::core::ffi::c_double,
    pub p1_along_m: ::core::ffi::c_double,
    pub p1_cross_m: ::core::ffi::c_double,
    pub p2_along_m: ::core::ffi::c_double,
    pub p2_cross_m: ::core::ffi::c_double,
    pub p3_along_m: ::core::ffi::c_double,
    pub p3_cross_m: ::core::ffi::c_double,
    pub capture_control: [TaemPoint2; 6],
    pub spiral_exit_radius_m: ::core::ffi::c_double,
    pub spiral_coefficient_m: ::core::ffi::c_double,
    pub spiral_exit_heading_rad: ::core::ffi::c_double,
    pub spiral_arc_lut: [::core::ffi::c_double; 129],
    pub unload_length_m: ::core::ffi::c_double,
    pub settle_length_m: ::core::ffi::c_double,
    pub lead_length_m: ::core::ffi::c_double,
    pub lead_arc_fraction_lut: [::core::ffi::c_float; 129],
    pub rollout_length_m: ::core::ffi::c_double,
    pub alignment_along_m: ::core::ffi::c_double,
    pub initial_course_deg: ::core::ffi::c_double,
    pub runway_heading_deg: ::core::ffi::c_double,
    pub profile_start_altitude_m: ::core::ffi::c_double,
    pub profile_start_fpa_deg: ::core::ffi::c_double,
    pub profile_final_altitude_m: ::core::ffi::c_double,
    pub profile_final_slope_deg: ::core::ffi::c_double,
    pub profile_total_length_m: ::core::ffi::c_double,
    pub profile_midpoint_offset_m: ::core::ffi::c_double,
    pub profile_local_offset_m: ::core::ffi::c_double,
    pub profile_local_start_fraction: ::core::ffi::c_double,
    pub profile_local_peak_fraction: ::core::ffi::c_double,
    pub profile_local_end_fraction: ::core::ffi::c_double,
    pub profile_initial_sag_m: ::core::ffi::c_double,
    pub profile_initial_sag_length_m: ::core::ffi::c_double,
    pub profile_tabulated: bool,
    pub profile_generation_aoa_deg: ::core::ffi::c_double,
    pub profile_altitude_lut: [::core::ffi::c_float; 257],
    pub profile_fpa_lut: [::core::ffi::c_float; 257],
    pub profile_speed_tabulated: bool,
    pub profile_speed_lut: [::core::ffi::c_float; 257],
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
pub struct TaemPathReference {
    pub runway_along_m: ::core::ffi::c_double,
    pub runway_cross_m: ::core::ffi::c_double,
    pub course_deg: ::core::ffi::c_double,
    pub curvature_right_per_m: ::core::ffi::c_double,
    pub altitude_m: ::core::ffi::c_double,
    pub flight_path_angle_deg: ::core::ffi::c_double,
    pub vertical_curvature_per_m: ::core::ffi::c_double,
    pub station_m: ::core::ffi::c_double,
}
#[derive(Copy, Clone)]
#[repr(C)]
pub struct TaemTrackerOutput {
    pub valid: bool,
    pub lateral_authority_ok: bool,
    pub vertical_authority_ok: bool,
    pub control: TerminalControl,
    pub cross_track_error_m: ::core::ffi::c_double,
    pub course_error_deg: ::core::ffi::c_double,
    pub required_lateral_accel_mps2: ::core::ffi::c_double,
    pub available_lateral_accel_mps2: ::core::ffi::c_double,
    pub required_vertical_lift_mps2: ::core::ffi::c_double,
    pub delivered_vertical_lift_mps2: ::core::ffi::c_double,
    pub response_time_s: ::core::ffi::c_double,
}
pub type TerminalSolverStatus = ::core::ffi::c_uint;
pub const TERMINAL_SOLVER_INVALID_INPUT: TerminalSolverStatus = 4;
pub const TERMINAL_SOLVER_SEARCH_EXHAUSTED: TerminalSolverStatus = 3;
pub const TERMINAL_SOLVER_UNQUALIFIED: TerminalSolverStatus = 2;
pub const TERMINAL_SOLVER_INFEASIBLE: TerminalSolverStatus = 1;
pub const TERMINAL_SOLVER_QUALIFIED: TerminalSolverStatus = 0;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct TerminalSolverResult {
    pub status: TerminalSolverStatus,
    pub path_constraints_ok: bool,
    pub final_interface_qualified: bool,
    pub final_state: TerminalDynamicState,
    pub final_geometry: TaemGeometryState,
    pub elapsed_s: ::core::ffi::c_double,
    pub minimum_speed_mps: ::core::ffi::c_double,
    pub maximum_dynamic_pressure_pa: ::core::ffi::c_double,
    pub maximum_load_g: ::core::ffi::c_double,
    pub maximum_cross_track_m: ::core::ffi::c_double,
    pub maximum_course_error_deg: ::core::ffi::c_double,
    pub final_route_course_error_deg: ::core::ffi::c_double,
    pub maximum_altitude_error_m: ::core::ffi::c_double,
    pub maximum_lateral_authority_shortfall_mps2: ::core::ffi::c_double,
    pub maximum_vertical_authority_shortfall_mps2: ::core::ffi::c_double,
    pub turn_burden_integral_s: ::core::ffi::c_double,
    pub maximum_turn_authority_fraction: ::core::ffi::c_double,
    pub maximum_commanded_aoa_deg: ::core::ffi::c_double,
    pub bank_target_reversals: ::core::ffi::c_uint,
    pub failure_cross_track_m: ::core::ffi::c_double,
    pub failure_course_error_deg: ::core::ffi::c_double,
    pub failure_required_lateral_accel_mps2: ::core::ffi::c_double,
    pub failure_available_lateral_accel_mps2: ::core::ffi::c_double,
    pub failure_required_vertical_lift_mps2: ::core::ffi::c_double,
    pub failure_delivered_vertical_lift_mps2: ::core::ffi::c_double,
    pub failure_target_altitude_m: ::core::ffi::c_double,
    pub failure_target_flight_path_angle_deg: ::core::ffi::c_double,
    pub failure_route_index: size_t,
    pub energy_start_j_kg: ::core::ffi::c_double,
    pub energy_end_j_kg: ::core::ffi::c_double,
    pub drag_work_j_kg: ::core::ffi::c_double,
    pub energy_closure_residual_j_kg: ::core::ffi::c_double,
    pub final_route_index: size_t,
    pub final_target_altitude_m: ::core::ffi::c_double,
    pub final_target_flight_path_angle_deg: ::core::ffi::c_double,
    pub reason: *const ::core::ffi::c_char,
}
#[derive(Copy, Clone)]
#[repr(C)]
pub struct TerminalProfileResult {
    pub valid: bool,
    pub aoa_deg: ::core::ffi::c_double,
    pub exit_speed_mps: ::core::ffi::c_double,
    pub exit_fpa_deg: ::core::ffi::c_double,
    pub reason: *const ::core::ffi::c_char,
}
#[derive(Copy, Clone)]
#[repr(C)]
pub struct ProfileFlight {
    pub end_altitude_m: ::core::ffi::c_double,
    pub capture_station_m: ::core::ffi::c_double,
    pub steepest_descent_deg: ::core::ffi::c_double,
    pub exit_speed_mps: ::core::ffi::c_double,
    pub exit_fpa_deg: ::core::ffi::c_double,
    pub exit_specific_energy_j_kg: ::core::ffi::c_double,
    pub reason: *const ::core::ffi::c_char,
    pub valid: bool,
}
#[derive(Copy, Clone)]
#[repr(C)]
pub struct ProfileSample {
    pub station_m: ::core::ffi::c_double,
    pub altitude_m: ::core::ffi::c_double,
    pub fpa_deg: ::core::ffi::c_double,
    pub speed_mps: ::core::ffi::c_double,
}
pub const SIZE_MAX: ::core::ffi::c_ulong = 18446744073709551615 as ::core::ffi::c_ulong;
pub const TAEM_ROUTE_PROFILE_LUT_POINTS: ::core::ffi::c_int = 257 as ::core::ffi::c_int;
#[inline]
unsafe extern "C" fn taem_route_station_at_index(
    mut route: *const TaemRoute,
    mut index: size_t,
) -> ::core::ffi::c_double {
    if route.is_null() || !(*route).valid || (*route).count < 2 as size_t
        || (*route).lead_count == 0 as size_t || (*route).arc_count == 0 as size_t
    {
        return ::core::f32::NAN as ::core::ffi::c_double;
    }
    if index >= (*route).count {
        index = (*route).count.wrapping_sub(1 as size_t);
    }
    if index <= (*route).lead_count {
        return (*route).lead_length_m * index as ::core::ffi::c_double
            / (*route).lead_count as ::core::ffi::c_double;
    }
    if index <= (*route).lead_count.wrapping_add((*route).arc_count) {
        return (*route).lead_length_m
            + (*route).hac.arc_length_m
                * index.wrapping_sub((*route).lead_count) as ::core::ffi::c_double
                / (*route).arc_count as ::core::ffi::c_double;
    }
    if (*route).rollout_count == 0 as size_t {
        return (*route).lead_length_m + (*route).hac.arc_length_m;
    }
    return (*route).lead_length_m + (*route).hac.arc_length_m
        + (*route).rollout_length_m
            * index.wrapping_sub((*route).lead_count).wrapping_sub((*route).arc_count)
                as ::core::ffi::c_double
            / (*route).rollout_count as ::core::ffi::c_double;
}
#[inline]
unsafe extern "C" fn taem_route_remaining_at_index(
    mut route: *const TaemRoute,
    mut index: size_t,
) -> ::core::ffi::c_double {
    let mut station: ::core::ffi::c_double = taem_route_station_at_index(route, index);
    if station.is_finite() as i32 == 0 || (*route).length_m.is_finite() as i32 == 0 {
        return ::core::f32::NAN as ::core::ffi::c_double;
    }
    return fmax(0.0f64, (*route).length_m - station);
}
unsafe extern "C" fn state_energy(
    mut m: *const TerminalModel,
    mut s: *const TerminalDynamicState,
    mut forces: AeroForces,
    mut energy: *mut TaemEnergyDiagnostic,
) -> bool {
    let mut world: TaemFrameWorld = TaemFrameWorld {
        radius_m: 0.,
        mu_m3_s2: 0.,
        rotation_axis_i: TaemVec3 { x: 0., y: 0., z: 0. },
        rotation_rate_rad_s: 0.,
        rotation_phase_rad_at_ut0: 0.,
    };
    if !taem_geometry_world(m, &raw mut world) {
        return false_0 != 0;
    }
    let mut pi: TaemVec3 = TaemVec3 {
        x: (*s).position_i_m.x,
        y: (*s).position_i_m.y,
        z: (*s).position_i_m.z,
    };
    let mut vi: TaemVec3 = TaemVec3 {
        x: (*s).velocity_i_mps.x,
        y: (*s).velocity_i_mps.y,
        z: (*s).velocity_i_mps.z,
    };
    let mut pb: TaemVec3 = TaemVec3 { x: 0., y: 0., z: 0. };
    let mut vb: TaemVec3 = TaemVec3 { x: 0., y: 0., z: 0. };
    if !taem_inertial_to_fixed(
        &raw mut world,
        pi,
        vi,
        (*s).ut_s,
        &raw mut pb,
        &raw mut vb,
    ) {
        return false_0 != 0;
    }
    let mut force_fixed_native: Vec3 = world_inertial_to_fixed(
        &raw const (*m).world,
        forces.force_i,
        (*s).ut_s,
    );
    let mut fi: TaemVec3 = TaemVec3 {
        x: forces.force_i.x,
        y: forces.force_i.y,
        z: forces.force_i.z,
    };
    let mut fb: TaemVec3 = TaemVec3 {
        x: force_fixed_native.x,
        y: force_fixed_native.y,
        z: force_fixed_native.z,
    };
    let mut air: TaemVec3 = vb;
    return taem_energy_diagnostic(
        &raw mut world,
        pi,
        vi,
        vb,
        air,
        fi,
        fb,
        (*s).mass_kg,
        forces.drag_n,
        energy,
    );
}
unsafe extern "C" fn record_turn_demand(
    mut m: *const TerminalModel,
    mut demand: *const TaemTrackerOutput,
    mut mach: ::core::ffi::c_double,
    mut dt: ::core::ffi::c_double,
    mut last_turn_sign: *mut ::core::ffi::c_int,
    mut out: *mut TerminalSolverResult,
) {
    let mut aoa_limit: ::core::ffi::c_double = fmin(
        (*m).vehicle.maximum_angle_of_attack,
        (*m).aero.alpha_deg[(*m).aero.alpha_count.wrapping_sub(1 as size_t) as usize],
    );
    if mach < 1.0f64 && (*m).vehicle.terminal_maximum_lift_angle_of_attack > 0.0f64 {
        aoa_limit = fmin(aoa_limit, (*m).vehicle.terminal_maximum_lift_angle_of_attack);
    }
    let mut bank_limit: ::core::ffi::c_double = fmin(
        (*m).vehicle.maximum_bank_angle,
        80.0f64,
    );
    let mut aoa_deg: ::core::ffi::c_double = fabs((*demand).control.angle_of_attack_rad)
        * 57.29577951308232f64;
    let mut bank_deg: ::core::ffi::c_double = fabs((*demand).control.bank_rad)
        * 57.29577951308232f64;
    let mut aoa_fraction: ::core::ffi::c_double = aoa_deg / fmax(aoa_limit, 1.0f64);
    let mut bank_fraction: ::core::ffi::c_double = bank_deg / fmax(bank_limit, 1.0f64);
    let mut lateral_fraction: ::core::ffi::c_double = fabs(
        (*demand).required_lateral_accel_mps2,
    ) / fmax((*demand).available_lateral_accel_mps2, 0.1f64);
    (*out).turn_burden_integral_s
        += dt
            * (lateral_fraction * lateral_fraction
                * (1.0f64 + aoa_fraction * aoa_fraction)
                + bank_fraction * bank_fraction * aoa_fraction * aoa_fraction);
    (*out).maximum_turn_authority_fraction = fmax(
        (*out).maximum_turn_authority_fraction,
        lateral_fraction,
    );
    (*out).maximum_commanded_aoa_deg = fmax((*out).maximum_commanded_aoa_deg, aoa_deg);
    let mut sign: ::core::ffi::c_int = if (*demand).control.bank_rad
        > 5.0f64 / 57.29577951308232f64
    {
        1 as ::core::ffi::c_int
    } else if (*demand).control.bank_rad < -5.0f64 / 57.29577951308232f64 {
        -(1 as ::core::ffi::c_int)
    } else {
        0 as ::core::ffi::c_int
    };
    if sign != 0 {
        if *last_turn_sign != 0 && sign != *last_turn_sign {
            (*out).bank_target_reversals = (*out).bank_target_reversals.wrapping_add(1);
        }
        *last_turn_sign = sign;
    }
}
pub const TERMINAL_PROFILE_MAX_DESCENT_EXCESS_DEG: ::core::ffi::c_double = 3.0f64;
pub const TERMINAL_PROFILE_DIVE_AOA_DEG: ::core::ffi::c_double = 3.0f64;
pub const TERMINAL_PROFILE_PULLOUT_M: ::core::ffi::c_double = 1000.0f64;
pub const TERMINAL_PROFILE_TURN_MARGIN: ::core::ffi::c_double = 1.3f64;
unsafe extern "C" fn max_profile_aoa(
    mut m: *const TerminalModel,
) -> ::core::ffi::c_double {
    let mut aoa: ::core::ffi::c_double = fmin(
        (*m).vehicle.maximum_angle_of_attack,
        (*m).aero.alpha_deg[(*m).aero.alpha_count.wrapping_sub(1 as size_t) as usize],
    );
    if (*m).vehicle.terminal_maximum_lift_angle_of_attack.is_finite() as i32 != 0
        && (*m).vehicle.terminal_maximum_lift_angle_of_attack > 0.0f64
    {
        aoa = fmin(aoa, (*m).vehicle.terminal_maximum_lift_angle_of_attack);
    }
    return aoa;
}
unsafe extern "C" fn fly_profile(
    mut m: *const TerminalModel,
    mut initial: *const TerminalDynamicState,
    mut route: *const TaemRoute,
    mut aoa_deg: ::core::ffi::c_double,
    mut dive_station_m: ::core::ffi::c_double,
    mut dive_aoa_deg: ::core::ffi::c_double,
    mut descent_limit_deg: ::core::ffi::c_double,
    mut capture_line: bool,
    mut dt: ::core::ffi::c_double,
    mut max_elapsed: ::core::ffi::c_double,
    mut samples: *mut ProfileSample,
    mut sample_capacity: size_t,
    mut sample_count: *mut size_t,
) -> ProfileFlight {
    let pi: ::core::ffi::c_double = 3.14159265358979323846f64;
    let mut out: ProfileFlight = ProfileFlight {
        end_altitude_m: ::core::f32::NAN as ::core::ffi::c_double,
        capture_station_m: ::core::f32::INFINITY as ::core::ffi::c_double,
        steepest_descent_deg: 0.,
        exit_speed_mps: ::core::f32::NAN as ::core::ffi::c_double,
        exit_fpa_deg: ::core::f32::NAN as ::core::ffi::c_double,
        exit_specific_energy_j_kg: ::core::f32::NAN as ::core::ffi::c_double,
        reason: b"profile propagation did not complete\0" as *const u8
            as *const ::core::ffi::c_char,
        valid: false,
    };
    let mut state: TerminalDynamicState = *initial;
    let mut geometry: TaemGeometryState = TaemGeometryState {
        runway_along_m: 0.,
        runway_cross_m: 0.,
        altitude_above_runway_m: 0.,
        ground_speed_mps: 0.,
        airspeed_mps: 0.,
        course_deg: 0.,
        flight_path_angle_deg: 0.,
        heading_error_deg: 0.,
        curvature_right_per_m: 0.,
    };
    if !taem_geometry_state(m, &raw mut state, &raw mut geometry) {
        out.reason = b"initial TAEM geometry invalid\0" as *const u8
            as *const ::core::ffi::c_char;
        return out;
    }
    let mut cursor: size_t = 0 as size_t;
    let mut count: size_t = 0 as size_t;
    let mut station: ::core::ffi::c_double = 0.0f64;
    let mut bank_max: ::core::ffi::c_double = fmin(
        (*m).vehicle.maximum_bank_angle,
        80.0f64,
    ) * pi / 180.0f64;
    let mut line_slope: ::core::ffi::c_double = tan(
        (*route).profile_final_slope_deg * pi / 180.0f64,
    );
    let mut on_line: bool = false_0 != 0;
    let mut limited: bool = false_0 != 0;
    let mut start_above_line: bool = true_0 != 0;
    let mut max_steps: size_t = ceil(max_elapsed / dt) as size_t;
    let mut step: size_t = 0 as size_t;
    while step < max_steps {
        let mut reference: TaemPathReference = TaemPathReference {
            runway_along_m: 0.,
            runway_cross_m: 0.,
            course_deg: 0.,
            curvature_right_per_m: 0.,
            altitude_m: 0.,
            flight_path_angle_deg: 0.,
            vertical_curvature_per_m: 0.,
            station_m: 0.,
        };
        let mut index: size_t = cursor;
        if !taem_route_reference(
            route,
            &raw mut geometry,
            &raw mut cursor,
            &raw mut reference,
            &raw mut index,
        ) {
            out.reason = b"route reference projection failed\0" as *const u8
                as *const ::core::ffi::c_char;
            return out;
        }
        station = reference.station_m;
        let mut altitude: ::core::ffi::c_double = (*m).site.altitude
            + geometry.altitude_above_runway_m;
        if !samples.is_null() && count < sample_capacity {
            let fresh0 = count;
            count = count.wrapping_add(1);
            *samples.offset(fresh0 as isize) = ProfileSample {
                station_m: station,
                altitude_m: altitude,
                fpa_deg: geometry.flight_path_angle_deg,
                speed_mps: geometry.airspeed_mps,
            };
        }
        if station >= (*route).length_m {
            let mut forces: AeroForces = aero_compute(
                &raw const (*m).world,
                &raw const (*m).aero,
                state.position_i_m,
                state.velocity_i_mps,
                state.ut_s,
                state.mass_kg,
                state.attitude.aoa_rad,
                state.attitude.bank_rad,
            );
            let mut energy: TaemEnergyDiagnostic = TaemEnergyDiagnostic {
                gravitational_potential_j_kg: 0.,
                inertial_specific_energy_j_kg: 0.,
                effective_specific_energy_j_kg: 0.,
                inertial_energy_rate_w_kg: 0.,
                effective_energy_rate_w_kg: 0.,
                airspeed_mps: 0.,
                drag_work_rate_w_kg: 0.,
            };
            if !state_energy(m, &raw mut state, forces, &raw mut energy) {
                out.reason = b"exit energy diagnostic failed\0" as *const u8
                    as *const ::core::ffi::c_char;
                return out;
            }
            out.exit_specific_energy_j_kg = energy.effective_specific_energy_j_kg;
            if !sample_count.is_null() {
                *sample_count = count;
            }
            out.end_altitude_m = altitude;
            out.exit_speed_mps = geometry.airspeed_mps;
            out.exit_fpa_deg = geometry.flight_path_angle_deg;
            out.valid = true_0 != 0;
            return out;
        }
        let mut line_altitude: ::core::ffi::c_double = (*route).profile_final_altitude_m
            + fmax(0.0f64, (*route).length_m - station) * line_slope;
        if step == 0 as size_t {
            start_above_line = altitude > line_altitude;
        }
        if capture_line as ::core::ffi::c_int != 0 && !on_line && step > 0 as size_t
            && (if start_above_line as ::core::ffi::c_int != 0 {
                (altitude <= line_altitude) as ::core::ffi::c_int
            } else {
                (altitude >= line_altitude) as ::core::ffi::c_int
            }) != 0
        {
            on_line = true_0 != 0;
            out.capture_station_m = station;
        }
        if !on_line && station >= dive_station_m
            && descent_limit_deg.is_finite() as i32 != 0
            && geometry.flight_path_angle_deg <= -descent_limit_deg
        {
            limited = true_0 != 0;
        }
        let mut control: TerminalControl = TerminalControl {
            angle_of_attack_rad: 0.,
            bank_rad: 0.,
            dt_s: 0.,
        };
        if on_line as ::core::ffi::c_int != 0 || limited as ::core::ffi::c_int != 0 {
            reference.altitude_m = if on_line as ::core::ffi::c_int != 0 {
                line_altitude
            } else {
                ::core::f32::NAN as ::core::ffi::c_double
            };
            reference.flight_path_angle_deg = if on_line as ::core::ffi::c_int != 0 {
                -(*route).profile_final_slope_deg
            } else {
                -descent_limit_deg
            };
            reference.vertical_curvature_per_m = 0.0f64;
            let mut demand: TaemTrackerOutput = taem_tracker_update(
                m,
                &raw mut state,
                &raw mut geometry,
                &raw mut reference,
                dt,
            );
            if !demand.valid {
                out.reason = b"TAEM tracker rejected vertical-line reference\0"
                    as *const u8 as *const ::core::ffi::c_char;
                return out;
            }
            control = demand.control;
        } else {
            let mut required_lateral: ::core::ffi::c_double = taem_tracker_lateral_demand(
                m,
                &raw mut geometry,
                &raw mut reference,
            );
            if required_lateral.is_finite() as i32 == 0 {
                out.reason = b"lateral demand became non-finite\0" as *const u8
                    as *const ::core::ffi::c_char;
                return out;
            }
            let mut aoa_cmd: ::core::ffi::c_double = if station < dive_station_m {
                aoa_deg
            } else {
                dive_aoa_deg
            };
            let mut aoa: ::core::ffi::c_double = aoa_cmd * pi / 180.0f64;
            let mut forces_0: AeroForces = aero_compute(
                &raw const (*m).world,
                &raw const (*m).aero,
                state.position_i_m,
                state.velocity_i_mps,
                state.ut_s,
                state.mass_kg,
                aoa,
                0.0f64,
            );
            let mut lift: ::core::ffi::c_double = fabs(forces_0.lift_n) / state.mass_kg;
            let mut needed: ::core::ffi::c_double = TERMINAL_PROFILE_TURN_MARGIN
                * fabs(required_lateral) / sin(bank_max);
            while lift < needed && aoa_cmd < max_profile_aoa(m) {
                aoa_cmd = fmin(max_profile_aoa(m), aoa_cmd + 0.5f64);
                aoa = aoa_cmd * pi / 180.0f64;
                forces_0 = aero_compute(
                    &raw const (*m).world,
                    &raw const (*m).aero,
                    state.position_i_m,
                    state.velocity_i_mps,
                    state.ut_s,
                    state.mass_kg,
                    aoa,
                    0.0f64,
                );
                lift = fabs(forces_0.lift_n) / state.mass_kg;
            }
            let mut bank: ::core::ffi::c_double = if lift > 1e-6f64 {
                asin(fmax(-1.0f64, fmin(1.0f64, required_lateral / lift)))
            } else {
                0.0f64
            };
            control = TerminalControl {
                angle_of_attack_rad: aoa,
                bank_rad: fmax(-bank_max, fmin(bank_max, bank)),
                dt_s: dt,
            };
        }
        let mut step_status: TerminalStepStatus = terminal_propagator_step(
            m,
            &raw mut state,
            &raw mut control,
        );
        if step_status as ::core::ffi::c_uint
            != TERMINAL_STEP_OK as ::core::ffi::c_int as ::core::ffi::c_uint
        {
            out.reason = if step_status as ::core::ffi::c_uint
                == TERMINAL_STEP_INVALID_INPUT as ::core::ffi::c_int
                    as ::core::ffi::c_uint
            {
                b"terminal propagator rejected input\0" as *const u8
                    as *const ::core::ffi::c_char
            } else {
                b"terminal propagator produced invalid result\0" as *const u8
                    as *const ::core::ffi::c_char
            };
            return out;
        }
        if !taem_geometry_state(m, &raw mut state, &raw mut geometry) {
            out.reason = b"TAEM geometry invalid after propagation\0" as *const u8
                as *const ::core::ffi::c_char;
            return out;
        }
        if geometry.altitude_above_runway_m <= 0.0f64 {
            if !sample_count.is_null() {
                *sample_count = count;
            }
            out.end_altitude_m = -::core::f32::INFINITY as ::core::ffi::c_double;
            out.valid = true_0 != 0;
            return out;
        }
        step = step.wrapping_add(1);
    }
    if !getenv(
            b"KSP_LANDER_TAEM_PROFILE_DIAGNOSTICS\0" as *const u8
                as *const ::core::ffi::c_char,
        )
        .is_null()
    {
        fprintf(
            stderr,
            b"TAEM constant profile timeout: length=%.3f station=%.3f elapsed=%.3f altitude=%.3f speed=%.3f fpa=%.3f aoa=%.3f\n\0"
                as *const u8 as *const ::core::ffi::c_char,
            (*route).length_m,
            station,
            max_elapsed,
            geometry.altitude_above_runway_m,
            geometry.airspeed_mps,
            geometry.flight_path_angle_deg,
            aoa_deg,
        );
    }
    out.reason = b"profile exceeded propagation time before route completion\0"
        as *const u8 as *const ::core::ffi::c_char;
    return out;
}
pub const TERMINAL_SMOOTH_TRACK_ERROR_M: ::core::ffi::c_double = 300.0f64;
pub const TERMINAL_SMOOTH_EXIT_ERROR_M: ::core::ffi::c_double = 150.0f64;
pub const TERMINAL_SMOOTH_SPEED_TOLERANCE: ::core::ffi::c_double = 0.12f64;
pub const TERMINAL_SMOOTH_BOW_FRACTION: ::core::ffi::c_double = 0.30f64;
pub const TERMINAL_SMOOTH_BOW_STEPS: ::core::ffi::c_int = 13 as ::core::ffi::c_int;
unsafe extern "C" fn alignment_speed(
    mut m: *const TerminalModel,
) -> ::core::ffi::c_double {
    return if (*m).guidance.final_alignment_speed.is_finite() as i32 != 0
        && (*m).guidance.final_alignment_speed > 0.0f64
    {
        (*m).guidance.final_alignment_speed
    } else {
        (*m).vehicle.final_approach_speed
    };
}
unsafe extern "C" fn taem_descent_limit(
    mut m: *const TerminalModel,
    mut route: *const TaemRoute,
) -> ::core::ffi::c_double {
    let mut final_limit: ::core::ffi::c_double = (*route).profile_final_slope_deg
        + TERMINAL_PROFILE_MAX_DESCENT_EXCESS_DEG;
    let mut taem_limit: ::core::ffi::c_double = if (*m)
        .guidance
        .taem_glide_slope
        .is_finite() as i32 != 0 && (*m).guidance.taem_glide_slope > 0.0f64
    {
        (*m).guidance.taem_glide_slope
    } else {
        final_limit
    };
    return fmax(final_limit, taem_limit);
}
unsafe extern "C" fn fly_smooth_profile(
    mut m: *const TerminalModel,
    mut initial: *const TerminalDynamicState,
    mut route: *const TaemRoute,
    mut dt: ::core::ffi::c_double,
    mut max_elapsed: ::core::ffi::c_double,
    mut max_error: *mut ::core::ffi::c_double,
    mut samples: *mut ProfileSample,
    mut sample_capacity: size_t,
    mut sample_count: *mut size_t,
) -> ProfileFlight {
    let mut out: ProfileFlight = ProfileFlight {
        end_altitude_m: ::core::f32::NAN as ::core::ffi::c_double,
        capture_station_m: ::core::f32::INFINITY as ::core::ffi::c_double,
        steepest_descent_deg: 0.,
        exit_speed_mps: ::core::f32::NAN as ::core::ffi::c_double,
        exit_fpa_deg: ::core::f32::NAN as ::core::ffi::c_double,
        exit_specific_energy_j_kg: ::core::f32::NAN as ::core::ffi::c_double,
        reason: b"not completed\0" as *const u8 as *const ::core::ffi::c_char,
        valid: false,
    };
    *max_error = ::core::f32::INFINITY as ::core::ffi::c_double;
    let mut state: TerminalDynamicState = *initial;
    let mut geometry: TaemGeometryState = TaemGeometryState {
        runway_along_m: 0.,
        runway_cross_m: 0.,
        altitude_above_runway_m: 0.,
        ground_speed_mps: 0.,
        airspeed_mps: 0.,
        course_deg: 0.,
        flight_path_angle_deg: 0.,
        heading_error_deg: 0.,
        curvature_right_per_m: 0.,
    };
    if !taem_geometry_state(m, &raw mut state, &raw mut geometry) {
        out.reason = b"initial geometry failed\0" as *const u8
            as *const ::core::ffi::c_char;
        return out;
    }
    let mut cursor: size_t = 0 as size_t;
    let mut count: size_t = 0 as size_t;
    let mut worst: ::core::ffi::c_double = 0.0f64;
    let mut steepest: ::core::ffi::c_double = -::core::f32::INFINITY
        as ::core::ffi::c_double;
    let mut max_steps: size_t = ceil(max_elapsed / dt) as size_t;
    let mut step: size_t = 0 as size_t;
    while step < max_steps {
        let mut reference: TaemPathReference = TaemPathReference {
            runway_along_m: 0.,
            runway_cross_m: 0.,
            course_deg: 0.,
            curvature_right_per_m: 0.,
            altitude_m: 0.,
            flight_path_angle_deg: 0.,
            vertical_curvature_per_m: 0.,
            station_m: 0.,
        };
        let mut index: size_t = cursor;
        if !taem_route_reference(
            route,
            &raw mut geometry,
            &raw mut cursor,
            &raw mut reference,
            &raw mut index,
        ) {
            out.reason = b"route reference failed\0" as *const u8
                as *const ::core::ffi::c_char;
            return out;
        }
        let mut altitude: ::core::ffi::c_double = (*m).site.altitude
            + geometry.altitude_above_runway_m;
        worst = fmax(worst, fabs(altitude - reference.altitude_m));
        steepest = fmax(steepest, -reference.flight_path_angle_deg);
        if !samples.is_null() && count < sample_capacity {
            let fresh1 = count;
            count = count.wrapping_add(1);
            *samples.offset(fresh1 as isize) = ProfileSample {
                station_m: reference.station_m,
                altitude_m: reference.altitude_m,
                fpa_deg: reference.flight_path_angle_deg,
                speed_mps: geometry.airspeed_mps,
            };
        }
        if reference.station_m >= (*route).length_m {
            if !sample_count.is_null() {
                *sample_count = count;
            }
            out.end_altitude_m = altitude;
            out.exit_speed_mps = geometry.airspeed_mps;
            out.exit_fpa_deg = geometry.flight_path_angle_deg;
            out.valid = true_0 != 0;
            *max_error = worst;
            out.steepest_descent_deg = steepest;
            out.reason = b"completed\0" as *const u8 as *const ::core::ffi::c_char;
            return out;
        }
        let mut demand: TaemTrackerOutput = taem_tracker_update(
            m,
            &raw mut state,
            &raw mut geometry,
            &raw mut reference,
            dt,
        );
        if !demand.valid {
            out.reason = b"tracker invalid\0" as *const u8 as *const ::core::ffi::c_char;
            return out;
        }
        if terminal_propagator_step(m, &raw mut state, &raw mut demand.control)
            as ::core::ffi::c_uint
            != TERMINAL_STEP_OK as ::core::ffi::c_int as ::core::ffi::c_uint
        {
            out.reason = b"propagator failed\0" as *const u8
                as *const ::core::ffi::c_char;
            return out;
        }
        if !taem_geometry_state(m, &raw mut state, &raw mut geometry) {
            out.reason = b"post-step geometry failed\0" as *const u8
                as *const ::core::ffi::c_char;
            return out;
        }
        if geometry.altitude_above_runway_m <= 0.0f64 {
            out.reason = b"ground before route end\0" as *const u8
                as *const ::core::ffi::c_char;
            return out;
        }
        step = step.wrapping_add(1);
    }
    out.reason = b"max elapsed before route end\0" as *const u8
        as *const ::core::ffi::c_char;
    return out;
}
unsafe extern "C" fn smooth_flight_ok(
    mut m: *const TerminalModel,
    mut route: *const TaemRoute,
    mut f: ProfileFlight,
    mut max_error: ::core::ffi::c_double,
) -> bool {
    return f.valid as ::core::ffi::c_int != 0 && f.end_altitude_m.is_finite() as i32 != 0
        && f.steepest_descent_deg <= taem_descent_limit(m, route)
        && max_error <= TERMINAL_SMOOTH_TRACK_ERROR_M
        && fabs(f.end_altitude_m - (*route).profile_final_altitude_m)
            <= TERMINAL_SMOOTH_EXIT_ERROR_M;
}
unsafe extern "C" fn tabulate_profile(
    mut route: *mut TaemRoute,
    mut samples: *const ProfileSample,
    mut count: size_t,
) {
    let intervals: size_t = (TAEM_ROUTE_PROFILE_LUT_POINTS - 1 as ::core::ffi::c_int)
        as size_t;
    let mut k: size_t = 0 as size_t;
    let mut j: size_t = 0 as size_t;
    while j <= intervals {
        let mut s: ::core::ffi::c_double = (*route).length_m * j as ::core::ffi::c_double
            / intervals as ::core::ffi::c_double;
        while k.wrapping_add(2 as size_t) < count
            && (*samples.offset(k.wrapping_add(1 as size_t) as isize)).station_m < s
        {
            k = k.wrapping_add(1);
        }
        let mut s0: ::core::ffi::c_double = (*samples.offset(k as isize)).station_m;
        let mut s1: ::core::ffi::c_double = (*samples
            .offset(k.wrapping_add(1 as size_t) as isize))
            .station_m;
        let mut f: ::core::ffi::c_double = if s1 > s0 {
            fmax(0.0f64, fmin(1.0f64, (s - s0) / (s1 - s0)))
        } else {
            0.0f64
        };
        (*route).profile_altitude_lut[j as usize] = ((1.0f64 - f)
            * (*samples.offset(k as isize)).altitude_m
            + f * (*samples.offset(k.wrapping_add(1 as size_t) as isize)).altitude_m)
            as ::core::ffi::c_float;
        (*route).profile_fpa_lut[j as usize] = ((1.0f64 - f)
            * (*samples.offset(k as isize)).fpa_deg
            + f * (*samples.offset(k.wrapping_add(1 as size_t) as isize)).fpa_deg)
            as ::core::ffi::c_float;
        (*route).profile_speed_lut[j as usize] = ((1.0f64 - f)
            * (*samples.offset(k as isize)).speed_mps
            + f * (*samples.offset(k.wrapping_add(1 as size_t) as isize)).speed_mps)
            as ::core::ffi::c_float;
        j = j.wrapping_add(1);
    }
    (*route).profile_total_length_m = (*route).length_m;
    (*route).profile_tabulated = true_0 != 0;
    (*route).profile_speed_tabulated = true_0 != 0;
}
unsafe extern "C" fn solve_smooth_profile(
    mut m: *const TerminalModel,
    mut initial: *const TerminalDynamicState,
    mut route: *mut TaemRoute,
    mut dt: ::core::ffi::c_double,
    mut max_elapsed: ::core::ffi::c_double,
    mut result: *mut TerminalProfileResult,
) -> bool {
    let mut target_speed: ::core::ffi::c_double = alignment_speed(m);
    let mut drop_0: ::core::ffi::c_double = (*route).profile_start_altitude_m
        - (*route).profile_final_altitude_m;
    if !(target_speed > 0.0f64) || !(drop_0 > 0.0f64) {
        return false_0 != 0;
    }
    let mut trial: TaemRoute = *route;
    trial.profile_tabulated = false_0 != 0;
    trial.profile_speed_tabulated = false_0 != 0;
    trial.profile_local_offset_m = 0.0f64;
    let mut start_geometry: TaemGeometryState = TaemGeometryState {
        runway_along_m: 0.,
        runway_cross_m: 0.,
        altitude_above_runway_m: 0.,
        ground_speed_mps: 0.,
        airspeed_mps: 0.,
        course_deg: 0.,
        flight_path_angle_deg: 0.,
        heading_error_deg: 0.,
        curvature_right_per_m: 0.,
    };
    if !taem_geometry_state(m, initial, &raw mut start_geometry) {
        return false_0 != 0;
    }
    let mut span: ::core::ffi::c_double = TERMINAL_SMOOTH_BOW_FRACTION * drop_0;
    let mut bows: [::core::ffi::c_double; 13] = [0.; 13];
    let mut errors: [::core::ffi::c_double; 13] = [0.; 13];
    let mut ok: [bool; 13] = [false; 13];
    let mut best: ::core::ffi::c_long = -(1 as ::core::ffi::c_int)
        as ::core::ffi::c_long;
    let mut raw_valid: ::core::ffi::c_int = 0 as ::core::ffi::c_int;
    let mut descent_ok: ::core::ffi::c_int = 0 as ::core::ffi::c_int;
    let mut track_ok: ::core::ffi::c_int = 0 as ::core::ffi::c_int;
    let mut endpoint_ok: ::core::ffi::c_int = 0 as ::core::ffi::c_int;
    let mut fail_tracker: ::core::ffi::c_int = 0 as ::core::ffi::c_int;
    let mut fail_propagator: ::core::ffi::c_int = 0 as ::core::ffi::c_int;
    let mut fail_ground: ::core::ffi::c_int = 0 as ::core::ffi::c_int;
    let mut fail_reference: ::core::ffi::c_int = 0 as ::core::ffi::c_int;
    let mut fail_geometry: ::core::ffi::c_int = 0 as ::core::ffi::c_int;
    let mut fail_timeout: ::core::ffi::c_int = 0 as ::core::ffi::c_int;
    let mut min_steep: ::core::ffi::c_double = ::core::f32::INFINITY
        as ::core::ffi::c_double;
    let mut min_track: ::core::ffi::c_double = ::core::f32::INFINITY
        as ::core::ffi::c_double;
    let mut min_endpoint: ::core::ffi::c_double = ::core::f32::INFINITY
        as ::core::ffi::c_double;
    let mut raw_min_speed: ::core::ffi::c_double = ::core::f32::INFINITY
        as ::core::ffi::c_double;
    let mut raw_max_speed: ::core::ffi::c_double = -::core::f32::INFINITY
        as ::core::ffi::c_double;
    let mut i: ::core::ffi::c_int = 0 as ::core::ffi::c_int;
    while i < TERMINAL_SMOOTH_BOW_STEPS {
        bows[i as usize] = -span
            + 2.0f64 * span * i as ::core::ffi::c_double
                / (TERMINAL_SMOOTH_BOW_STEPS - 1 as ::core::ffi::c_int)
                    as ::core::ffi::c_double;
        trial.profile_midpoint_offset_m = bows[i as usize];
        if !taem_route_limit_initial_vertical_authority(
            m,
            initial,
            &raw mut start_geometry,
            &raw mut trial,
        ) {
            ok[i as usize] = false_0 != 0;
            errors[i as usize] = ::core::f32::NAN as ::core::ffi::c_double;
        } else {
            let mut max_error: ::core::ffi::c_double = ::core::f32::INFINITY
                as ::core::ffi::c_double;
            let mut f: ProfileFlight = fly_smooth_profile(
                m,
                initial,
                &raw mut trial,
                dt,
                max_elapsed,
                &raw mut max_error,
                ::core::ptr::null_mut::<ProfileSample>(),
                0 as size_t,
                ::core::ptr::null_mut::<size_t>(),
            );
            if f.valid as ::core::ffi::c_int != 0
                && f.end_altitude_m.is_finite() as i32 != 0
            {
                raw_valid += 1;
                min_steep = fmin(min_steep, f.steepest_descent_deg);
                min_track = fmin(min_track, max_error);
                min_endpoint = fmin(
                    min_endpoint,
                    fabs(f.end_altitude_m - trial.profile_final_altitude_m),
                );
                raw_min_speed = fmin(raw_min_speed, f.exit_speed_mps);
                raw_max_speed = fmax(raw_max_speed, f.exit_speed_mps);
                if f.steepest_descent_deg <= taem_descent_limit(m, &raw mut trial) {
                    descent_ok += 1;
                }
                if max_error <= TERMINAL_SMOOTH_TRACK_ERROR_M {
                    track_ok += 1;
                }
                if fabs(f.end_altitude_m - trial.profile_final_altitude_m)
                    <= TERMINAL_SMOOTH_EXIT_ERROR_M
                {
                    endpoint_ok += 1;
                }
            } else if !f.reason.is_null() {
                if strcmp(
                    f.reason,
                    b"tracker invalid\0" as *const u8 as *const ::core::ffi::c_char,
                ) == 0 as ::core::ffi::c_int
                {
                    fail_tracker += 1;
                } else if strcmp(
                    f.reason,
                    b"propagator failed\0" as *const u8 as *const ::core::ffi::c_char,
                ) == 0 as ::core::ffi::c_int
                {
                    fail_propagator += 1;
                } else if strcmp(
                    f.reason,
                    b"ground before route end\0" as *const u8
                        as *const ::core::ffi::c_char,
                ) == 0 as ::core::ffi::c_int
                {
                    fail_ground += 1;
                } else if strcmp(
                    f.reason,
                    b"route reference failed\0" as *const u8
                        as *const ::core::ffi::c_char,
                ) == 0 as ::core::ffi::c_int
                {
                    fail_reference += 1;
                } else if strcmp(
                    f.reason,
                    b"initial geometry failed\0" as *const u8
                        as *const ::core::ffi::c_char,
                ) == 0 as ::core::ffi::c_int
                    || strcmp(
                        f.reason,
                        b"post-step geometry failed\0" as *const u8
                            as *const ::core::ffi::c_char,
                    ) == 0 as ::core::ffi::c_int
                {
                    fail_geometry += 1;
                } else if strcmp(
                    f.reason,
                    b"max elapsed before route end\0" as *const u8
                        as *const ::core::ffi::c_char,
                ) == 0 as ::core::ffi::c_int
                {
                    fail_timeout += 1;
                }
            }
            ok[i as usize] = smooth_flight_ok(m, &raw mut trial, f, max_error);
            errors[i as usize] = if ok[i as usize] as ::core::ffi::c_int != 0 {
                f.exit_speed_mps - target_speed
            } else {
                ::core::f32::NAN as ::core::ffi::c_double
            };
            if !getenv(
                    b"KSP_LANDER_TAEM_PROFILE_DIAGNOSTICS\0" as *const u8
                        as *const ::core::ffi::c_char,
                )
                .is_null()
            {
                fprintf(
                    stderr,
                    b"TAEM smooth trial: route=%.3f bow=%.3f completed=%d feasible=%d speed=%.3f steep=%.3f track=%.3f endpoint=%.3f slopeLimit=%.3f trackLimit=%.3f endpointLimit=%.3f reason=%s\n\0"
                        as *const u8 as *const ::core::ffi::c_char,
                    (*route).length_m,
                    bows[i as usize],
                    f.valid as ::core::ffi::c_int,
                    ok[i as usize] as ::core::ffi::c_int,
                    f.exit_speed_mps,
                    if f.valid as ::core::ffi::c_int != 0 {
                        f.steepest_descent_deg
                    } else {
                        ::core::f32::NAN as ::core::ffi::c_double
                    },
                    max_error,
                    if f.valid as ::core::ffi::c_int != 0 {
                        fabs(f.end_altitude_m - trial.profile_final_altitude_m)
                    } else {
                        ::core::f32::NAN as ::core::ffi::c_double
                    },
                    taem_descent_limit(m, &raw mut trial),
                    TERMINAL_SMOOTH_TRACK_ERROR_M,
                    TERMINAL_SMOOTH_EXIT_ERROR_M,
                    if !f.reason.is_null() {
                        f.reason
                    } else {
                        b"unknown\0" as *const u8 as *const ::core::ffi::c_char
                    },
                );
            }
            if ok[i as usize] as ::core::ffi::c_int != 0
                && (best < 0 as ::core::ffi::c_long
                    || fabs(errors[i as usize]) < fabs(errors[best as usize]))
            {
                best = i as ::core::ffi::c_long;
            }
        }
        i += 1;
    }
    if !getenv(
            b"KSP_LANDER_TAEM_DIAGNOSTICS\0" as *const u8 as *const ::core::ffi::c_char,
        )
        .is_null()
    {
        let mut min_speed: ::core::ffi::c_double = ::core::f32::INFINITY
            as ::core::ffi::c_double;
        let mut max_speed: ::core::ffi::c_double = -::core::f32::INFINITY
            as ::core::ffi::c_double;
        let mut feasible: ::core::ffi::c_int = 0 as ::core::ffi::c_int;
        let mut i_0: ::core::ffi::c_int = 0 as ::core::ffi::c_int;
        while i_0 < TERMINAL_SMOOTH_BOW_STEPS {
            if ok[i_0 as usize] {
                let mut speed: ::core::ffi::c_double = target_speed
                    + errors[i_0 as usize];
                min_speed = fmin(min_speed, speed);
                max_speed = fmax(max_speed, speed);
                feasible += 1;
            }
            i_0 += 1;
        }
        fprintf(
            stderr,
            b"TAEM smooth envelope: route=%.0f targetV=%.1f feasible=%d minV=%.1f maxV=%.1f bestV=%.1f bow=%.0f span=%.0f raw=%d descent=%d track=%d endpoint=%d fail[t=%d p=%d g=%d r=%d geom=%d timeout=%d] minSteep=%.1f minTrack=%.0f minEnd=%.0f rawV=%.1f..%.1f limit=%.1f\n\0"
                as *const u8 as *const ::core::ffi::c_char,
            (*route).length_m,
            target_speed,
            feasible,
            if feasible != 0 {
                min_speed
            } else {
                ::core::f32::NAN as ::core::ffi::c_double
            },
            if feasible != 0 {
                max_speed
            } else {
                ::core::f32::NAN as ::core::ffi::c_double
            },
            if best >= 0 as ::core::ffi::c_long {
                target_speed + errors[best as usize]
            } else {
                ::core::f32::NAN as ::core::ffi::c_double
            },
            if best >= 0 as ::core::ffi::c_long {
                bows[best as usize]
            } else {
                ::core::f32::NAN as ::core::ffi::c_double
            },
            span,
            raw_valid,
            descent_ok,
            track_ok,
            endpoint_ok,
            fail_tracker,
            fail_propagator,
            fail_ground,
            fail_reference,
            fail_geometry,
            fail_timeout,
            if raw_valid != 0 {
                min_steep
            } else {
                ::core::f32::NAN as ::core::ffi::c_double
            },
            if raw_valid != 0 {
                min_track
            } else {
                ::core::f32::NAN as ::core::ffi::c_double
            },
            if raw_valid != 0 {
                min_endpoint
            } else {
                ::core::f32::NAN as ::core::ffi::c_double
            },
            if raw_valid != 0 {
                raw_min_speed
            } else {
                ::core::f32::NAN as ::core::ffi::c_double
            },
            if raw_valid != 0 {
                raw_max_speed
            } else {
                ::core::f32::NAN as ::core::ffi::c_double
            },
            taem_descent_limit(m, route),
        );
    }
    if best < 0 as ::core::ffi::c_long {
        return false_0 != 0;
    }
    let mut bow: ::core::ffi::c_double = bows[best as usize];
    let mut error: ::core::ffi::c_double = errors[best as usize];
    let mut side: ::core::ffi::c_int = -(1 as ::core::ffi::c_int);
    while side <= 1 as ::core::ffi::c_int {
        let mut j: ::core::ffi::c_long = best + side as ::core::ffi::c_long;
        if j < 0 as ::core::ffi::c_long
            || j >= TERMINAL_SMOOTH_BOW_STEPS as ::core::ffi::c_long || !ok[j as usize]
            || (errors[j as usize] > 0.0f64) as ::core::ffi::c_int
                == (errors[best as usize] > 0.0f64) as ::core::ffi::c_int
        {
            side += 2 as ::core::ffi::c_int;
        } else {
            let mut a: ::core::ffi::c_double = bows[best as usize];
            let mut ea: ::core::ffi::c_double = errors[best as usize];
            let mut b: ::core::ffi::c_double = bows[j as usize];
            let mut iteration: ::core::ffi::c_int = 0 as ::core::ffi::c_int;
            while iteration < 8 as ::core::ffi::c_int {
                let mut c: ::core::ffi::c_double = 0.5f64 * (a + b);
                trial.profile_midpoint_offset_m = c;
                if !taem_route_limit_initial_vertical_authority(
                    m,
                    initial,
                    &raw mut start_geometry,
                    &raw mut trial,
                ) {
                    break;
                }
                let mut max_error_0: ::core::ffi::c_double = ::core::f32::INFINITY
                    as ::core::ffi::c_double;
                let mut f_0: ProfileFlight = fly_smooth_profile(
                    m,
                    initial,
                    &raw mut trial,
                    dt,
                    max_elapsed,
                    &raw mut max_error_0,
                    ::core::ptr::null_mut::<ProfileSample>(),
                    0 as size_t,
                    ::core::ptr::null_mut::<size_t>(),
                );
                if !smooth_flight_ok(m, &raw mut trial, f_0, max_error_0) {
                    break;
                }
                let mut ec: ::core::ffi::c_double = f_0.exit_speed_mps - target_speed;
                if fabs(ec) < fabs(error) {
                    bow = c;
                    error = ec;
                }
                if (ec > 0.0f64) as ::core::ffi::c_int
                    == (ea > 0.0f64) as ::core::ffi::c_int
                {
                    a = c;
                    ea = ec;
                } else {
                    b = c;
                }
                iteration += 1;
            }
            break;
        }
    }
    if fabs(error) > TERMINAL_SMOOTH_SPEED_TOLERANCE * target_speed {
        return false_0 != 0;
    }
    trial.profile_midpoint_offset_m = bow;
    if !taem_route_limit_initial_vertical_authority(
        m,
        initial,
        &raw mut start_geometry,
        &raw mut trial,
    ) {
        return false_0 != 0;
    }
    let mut capacity: size_t = (ceil(max_elapsed / dt) as size_t)
        .wrapping_add(1 as size_t);
    let mut samples: *mut ProfileSample = calloc(
        capacity,
        ::core::mem::size_of::<ProfileSample>() as size_t,
    ) as *mut ProfileSample;
    if samples.is_null() {
        return false_0 != 0;
    }
    let mut count: size_t = 0 as size_t;
    let mut max_error_1: ::core::ffi::c_double = ::core::f32::INFINITY
        as ::core::ffi::c_double;
    let mut flight: ProfileFlight = fly_smooth_profile(
        m,
        initial,
        &raw mut trial,
        dt,
        max_elapsed,
        &raw mut max_error_1,
        samples,
        capacity,
        &raw mut count,
    );
    if !smooth_flight_ok(m, &raw mut trial, flight, max_error_1) || count < 2 as size_t {
        free(samples as *mut ::core::ffi::c_void);
        return false_0 != 0;
    }
    (*route).profile_midpoint_offset_m = bow;
    (*route).profile_local_offset_m = 0.0f64;
    (*route).profile_initial_sag_m = trial.profile_initial_sag_m;
    (*route).profile_initial_sag_length_m = trial.profile_initial_sag_length_m;
    tabulate_profile(route, samples, count);
    free(samples as *mut ::core::ffi::c_void);
    (*route).profile_generation_aoa_deg = ::core::f32::NAN as ::core::ffi::c_double;
    (*result).valid = true_0 != 0;
    (*result).aoa_deg = ::core::f32::NAN as ::core::ffi::c_double;
    (*result).exit_speed_mps = flight.exit_speed_mps;
    (*result).exit_fpa_deg = flight.exit_fpa_deg;
    (*result).reason = b"smooth reference meets the Final alignment point altitude and speed\0"
        as *const u8 as *const ::core::ffi::c_char;
    return true_0 != 0;
}
#[no_mangle]
pub unsafe extern "C" fn terminal_solver_generate_profile(
    mut m: *const TerminalModel,
    mut initial: *const TerminalDynamicState,
    mut route: *mut TaemRoute,
    mut dt: ::core::ffi::c_double,
    mut max_elapsed: ::core::ffi::c_double,
) -> TerminalProfileResult {
    let mut out: TerminalProfileResult = TerminalProfileResult {
        valid: 0 as ::core::ffi::c_int != 0,
        aoa_deg: 0.,
        exit_speed_mps: 0.,
        exit_fpa_deg: 0.,
        reason: ::core::ptr::null::<::core::ffi::c_char>(),
    };
    out.reason = b"invalid profile input\0" as *const u8 as *const ::core::ffi::c_char;
    if m.is_null() || !(*m).replay_validated
        || !terminal_model_validate(
            m,
            ::core::ptr::null_mut::<::core::ffi::c_char>(),
            0 as size_t,
        )
        || !terminal_state_validate(
            initial,
            ::core::ptr::null_mut::<::core::ffi::c_char>(),
            0 as size_t,
        ) || route.is_null() || !(*route).valid || !(dt > 0.0f64)
        || !(max_elapsed > 0.0f64) || dt.is_finite() as i32 == 0
        || max_elapsed.is_finite() as i32 == 0
        || (max_elapsed / dt).is_finite() as i32 == 0
        || max_elapsed / dt
            >= SIZE_MAX as ::core::ffi::c_double
                / ::core::mem::size_of::<ProfileSample>() as ::core::ffi::c_double
                - 1.0f64
    {
        return out;
    }
    let mut start: TaemGeometryState = TaemGeometryState {
        runway_along_m: 0.,
        runway_cross_m: 0.,
        altitude_above_runway_m: 0.,
        ground_speed_mps: 0.,
        airspeed_mps: 0.,
        course_deg: 0.,
        flight_path_angle_deg: 0.,
        heading_error_deg: 0.,
        curvature_right_per_m: 0.,
    };
    if !taem_geometry_state(m, initial, &raw mut start) {
        return out;
    }
    if solve_smooth_profile(m, initial, route, dt, max_elapsed, &raw mut out) {
        return out;
    }
    let pi: ::core::ffi::c_double = 3.14159265358979323846f64;
    let mut target: ::core::ffi::c_double = (*route).profile_final_altitude_m;
    let mut start_altitude: ::core::ffi::c_double = (*m).site.altitude
        + start.altitude_above_runway_m;
    let mut above_line: bool = start_altitude
        > target
            + (*route).length_m * tan((*route).profile_final_slope_deg * pi / 180.0f64);
    let mut best_aoa: ::core::ffi::c_double = ::core::f32::NAN as ::core::ffi::c_double;
    let mut high_end: ::core::ffi::c_double = -::core::f32::INFINITY
        as ::core::ffi::c_double;
    let mut best_energy: ::core::ffi::c_double = -::core::f32::INFINITY
        as ::core::ffi::c_double;
    let mut last_propagation_reason: *const ::core::ffi::c_char = ::core::ptr::null::<
        ::core::ffi::c_char,
    >();
    let mut aoa: ::core::ffi::c_double = 2.0f64;
    while aoa <= max_profile_aoa(m) + 1e-9f64 {
        let mut f: ProfileFlight = fly_profile(
            m,
            initial,
            route,
            aoa,
            ::core::f32::INFINITY as ::core::ffi::c_double,
            aoa,
            ::core::f32::NAN as ::core::ffi::c_double,
            false_0 != 0,
            dt,
            max_elapsed,
            ::core::ptr::null_mut::<ProfileSample>(),
            0 as size_t,
            ::core::ptr::null_mut::<size_t>(),
        );
        if !f.valid {
            last_propagation_reason = f.reason;
        } else if f.end_altitude_m.is_finite() as i32 == 0 {
            if high_end.is_finite() as i32 == 0 {
                high_end = -::core::f32::INFINITY as ::core::ffi::c_double;
            }
        } else {
            high_end = fmax(high_end, f.end_altitude_m);
            let mut energy: ::core::ffi::c_double = f.exit_specific_energy_j_kg;
            if energy > best_energy {
                best_energy = energy;
                best_aoa = aoa;
            }
        }
        aoa += 2.0f64;
    }
    if best_aoa.is_finite() as i32 == 0 {
        out.reason = if !last_propagation_reason.is_null() {
            last_propagation_reason
        } else if high_end == -::core::f32::INFINITY as ::core::ffi::c_double {
            b"route is too long for the available energy\0" as *const u8
                as *const ::core::ffi::c_char
        } else {
            b"profile propagation failed\0" as *const u8 as *const ::core::ffi::c_char
        };
        return out;
    }
    if high_end < target {
        out.reason = b"route is too long for the available energy\0" as *const u8
            as *const ::core::ffi::c_char;
        return out;
    }
    let mut aoa_low: ::core::ffi::c_double = 0.5f64;
    let mut aoa_high: ::core::ffi::c_double = best_aoa;
    let mut capture: bool = false_0 != 0;
    let mut dive_station: ::core::ffi::c_double = ::core::f32::INFINITY
        as ::core::ffi::c_double;
    let mut dive_aoa: ::core::ffi::c_double = aoa_low;
    let mut descent_limit: ::core::ffi::c_double = ::core::f32::NAN
        as ::core::ffi::c_double;
    if above_line {
        let mut capture_target: ::core::ffi::c_double = (*route).length_m
            - TERMINAL_PROFILE_PULLOUT_M;
        dive_aoa = fmin(best_aoa, TERMINAL_PROFILE_DIVE_AOA_DEG);
        descent_limit = taem_descent_limit(m, route);
        let mut steep: ProfileFlight = fly_profile(
            m,
            initial,
            route,
            best_aoa,
            0.0f64,
            dive_aoa,
            descent_limit,
            true_0 != 0,
            dt,
            max_elapsed,
            ::core::ptr::null_mut::<ProfileSample>(),
            0 as size_t,
            ::core::ptr::null_mut::<size_t>(),
        );
        if !steep.valid || !(steep.capture_station_m <= capture_target) {
            out.reason = if steep.valid as ::core::ffi::c_int != 0 {
                b"route is too short to descend within the slope limit\0" as *const u8
                    as *const ::core::ffi::c_char
            } else if !steep.reason.is_null() {
                steep.reason
            } else {
                b"route is too short to descend within the slope limit\0" as *const u8
                    as *const ::core::ffi::c_char
            };
            return out;
        }
        let mut early: ::core::ffi::c_double = 0.0f64;
        let mut late: ::core::ffi::c_double = (*route).length_m;
        let mut iteration: ::core::ffi::c_int = 0 as ::core::ffi::c_int;
        while iteration < 12 as ::core::ffi::c_int {
            let mut s: ::core::ffi::c_double = 0.5f64 * (early + late);
            let mut f_0: ProfileFlight = fly_profile(
                m,
                initial,
                route,
                best_aoa,
                s,
                dive_aoa,
                descent_limit,
                true_0 != 0,
                dt,
                max_elapsed,
                ::core::ptr::null_mut::<ProfileSample>(),
                0 as size_t,
                ::core::ptr::null_mut::<size_t>(),
            );
            if !f_0.valid {
                out.reason = if !f_0.reason.is_null() {
                    f_0.reason
                } else {
                    b"profile propagation failed\0" as *const u8
                        as *const ::core::ffi::c_char
                };
                return out;
            }
            if f_0.capture_station_m <= capture_target {
                early = s;
            } else {
                late = s;
            }
            iteration += 1;
        }
        dive_station = early;
        aoa_high = best_aoa;
        capture = true_0 != 0;
    } else {
        let mut capture_target_0: ::core::ffi::c_double = (*route).length_m
            - TERMINAL_PROFILE_PULLOUT_M;
        let mut meet: ProfileFlight = fly_profile(
            m,
            initial,
            route,
            best_aoa,
            ::core::f32::INFINITY as ::core::ffi::c_double,
            best_aoa,
            ::core::f32::NAN as ::core::ffi::c_double,
            true_0 != 0,
            dt,
            max_elapsed,
            ::core::ptr::null_mut::<ProfileSample>(),
            0 as size_t,
            ::core::ptr::null_mut::<size_t>(),
        );
        if meet.valid as ::core::ffi::c_int != 0
            && meet.capture_station_m <= capture_target_0
        {
            let mut shallow: ::core::ffi::c_double = best_aoa;
            let mut iteration_0: ::core::ffi::c_int = 0 as ::core::ffi::c_int;
            while iteration_0 < 10 as ::core::ffi::c_int {
                let mut aoa_0: ::core::ffi::c_double = 0.5f64 * (aoa_low + shallow);
                let mut f_1: ProfileFlight = fly_profile(
                    m,
                    initial,
                    route,
                    aoa_0,
                    ::core::f32::INFINITY as ::core::ffi::c_double,
                    aoa_0,
                    ::core::f32::NAN as ::core::ffi::c_double,
                    true_0 != 0,
                    dt,
                    max_elapsed,
                    ::core::ptr::null_mut::<ProfileSample>(),
                    0 as size_t,
                    ::core::ptr::null_mut::<size_t>(),
                );
                if !f_1.valid {
                    out.reason = if !f_1.reason.is_null() {
                        f_1.reason
                    } else {
                        b"profile propagation failed\0" as *const u8
                            as *const ::core::ffi::c_char
                    };
                    return out;
                }
                if f_1.capture_station_m <= capture_target_0 {
                    shallow = aoa_0;
                } else {
                    aoa_low = aoa_0;
                }
                iteration_0 += 1;
            }
            aoa_high = shallow;
            dive_aoa = shallow;
            capture = true_0 != 0;
        }
    }
    if !above_line && !capture {
        let mut low: ProfileFlight = fly_profile(
            m,
            initial,
            route,
            aoa_low,
            ::core::f32::INFINITY as ::core::ffi::c_double,
            aoa_low,
            ::core::f32::NAN as ::core::ffi::c_double,
            false_0 != 0,
            dt,
            max_elapsed,
            ::core::ptr::null_mut::<ProfileSample>(),
            0 as size_t,
            ::core::ptr::null_mut::<size_t>(),
        );
        if !low.valid {
            out.reason = if !low.reason.is_null() {
                low.reason
            } else {
                b"profile propagation failed\0" as *const u8
                    as *const ::core::ffi::c_char
            };
            return out;
        }
        if low.end_altitude_m > target {
            out.reason = b"route is too short to dissipate the available energy\0"
                as *const u8 as *const ::core::ffi::c_char;
            return out;
        }
        let mut iteration_1: ::core::ffi::c_int = 0 as ::core::ffi::c_int;
        while iteration_1 < 10 as ::core::ffi::c_int {
            let mut aoa_1: ::core::ffi::c_double = 0.5f64 * (aoa_low + aoa_high);
            let mut f_2: ProfileFlight = fly_profile(
                m,
                initial,
                route,
                aoa_1,
                ::core::f32::INFINITY as ::core::ffi::c_double,
                aoa_1,
                ::core::f32::NAN as ::core::ffi::c_double,
                false_0 != 0,
                dt,
                max_elapsed,
                ::core::ptr::null_mut::<ProfileSample>(),
                0 as size_t,
                ::core::ptr::null_mut::<size_t>(),
            );
            if !f_2.valid {
                out.reason = if !f_2.reason.is_null() {
                    f_2.reason
                } else {
                    b"profile propagation failed\0" as *const u8
                        as *const ::core::ffi::c_char
                };
                return out;
            }
            if f_2.end_altitude_m < target {
                aoa_low = aoa_1;
            } else {
                aoa_high = aoa_1;
            }
            iteration_1 += 1;
        }
    }
    let mut capacity: size_t = (ceil(max_elapsed / dt) as size_t)
        .wrapping_add(1 as size_t);
    let mut samples: *mut ProfileSample = calloc(
        capacity,
        ::core::mem::size_of::<ProfileSample>() as size_t,
    ) as *mut ProfileSample;
    if samples.is_null() {
        out.reason = b"profile allocation failed\0" as *const u8
            as *const ::core::ffi::c_char;
        return out;
    }
    let mut count: size_t = 0 as size_t;
    let mut flight: ProfileFlight = fly_profile(
        m,
        initial,
        route,
        aoa_high,
        dive_station,
        dive_aoa,
        descent_limit,
        capture,
        dt,
        max_elapsed,
        samples,
        capacity,
        &raw mut count,
    );
    if !flight.valid || flight.end_altitude_m.is_finite() as i32 == 0
        || count < 2 as size_t
    {
        free(samples as *mut ::core::ffi::c_void);
        out.reason = if !flight.reason.is_null() {
            flight.reason
        } else {
            b"profile propagation failed\0" as *const u8 as *const ::core::ffi::c_char
        };
        return out;
    }
    let mut target_speed: ::core::ffi::c_double = alignment_speed(m);
    let mut speed_error: ::core::ffi::c_double = flight.exit_speed_mps - target_speed;
    let mut speed_tolerance: ::core::ffi::c_double = TERMINAL_SMOOTH_SPEED_TOLERANCE
        * target_speed;
    if flight.exit_speed_mps.is_finite() as i32 == 0
        || fabs(speed_error) > speed_tolerance
    {
        free(samples as *mut ::core::ffi::c_void);
        out.exit_speed_mps = flight.exit_speed_mps;
        out.exit_fpa_deg = flight.exit_fpa_deg;
        out.reason = if flight.exit_speed_mps < target_speed {
            b"alignment profile is too slow for Final handoff\0" as *const u8
                as *const ::core::ffi::c_char
        } else {
            b"alignment profile retains excess energy for Final handoff\0" as *const u8
                as *const ::core::ffi::c_char
        };
        return out;
    }
    tabulate_profile(route, samples, count);
    free(samples as *mut ::core::ffi::c_void);
    (*route).profile_generation_aoa_deg = aoa_high;
    (*route).profile_tabulated = true_0 != 0;
    out.valid = true_0 != 0;
    out.aoa_deg = aoa_high;
    out.exit_speed_mps = flight.exit_speed_mps;
    out.exit_fpa_deg = flight.exit_fpa_deg;
    out.reason = if capture as ::core::ffi::c_int != 0 {
        b"native slope-limited dive captured the exit glide line\0" as *const u8
            as *const ::core::ffi::c_char
    } else {
        b"native constant-incidence profile meets the exit altitude\0" as *const u8
            as *const ::core::ffi::c_char
    };
    return out;
}
#[no_mangle]
pub unsafe extern "C" fn terminal_solver_replay(
    mut m: *const TerminalModel,
    mut initial: *const TerminalDynamicState,
    mut route: *const TaemRoute,
    mut dt: ::core::ffi::c_double,
    mut max_elapsed: ::core::ffi::c_double,
) -> TerminalSolverResult {
    let mut out: TerminalSolverResult = TerminalSolverResult {
        status: TERMINAL_SOLVER_QUALIFIED,
        path_constraints_ok: false,
        final_interface_qualified: false,
        final_state: TerminalDynamicState {
            position_i_m: Vec3 { x: 0., y: 0., z: 0. },
            velocity_i_mps: Vec3 { x: 0., y: 0., z: 0. },
            attitude: AttitudeModel {
                aoa_rad: 0.,
                bank_rad: 0.,
                aoa_rate_rad_s: 0.,
                bank_rate_rad_s: 0.,
                requested_aoa_rad: 0.,
                requested_bank_rad: 0.,
                cmd_aoa_rad: 0.,
                cmd_bank_rad: 0.,
                pitch_wn: 0.,
                pitch_zeta: 0.,
                roll_wn: 0.,
                roll_zeta: 0.,
                max_pitch_rate_rad_s: 0.,
                max_roll_rate_rad_s: 0.,
                max_pitch_accel_rad_s2: 0.,
                max_roll_accel_rad_s2: 0.,
                pitch_full_authority_q_pa: 0.,
                roll_full_authority_q_pa: 0.,
                direct: false,
                direct_body_coordinates: false,
                body_rates_initialized: false,
                body_pitch_rate_rad_s: 0.,
                body_roll_rate_rad_s: 0.,
                body_yaw_rate_rad_s: 0.,
                input_pitch: 0.,
                input_roll: 0.,
                input_yaw: 0.,
                sideslip_rad: 0.,
                sideslip_rate_rad_s: 0.,
                direct_min_q_pa: 0.,
                direct_pitch_accel_per_kpa: 0.,
                direct_pitch_accel_max: 0.,
                direct_pitch_stiffness_per_kpa: 0.,
                direct_trim_aoa_rad: 0.,
                direct_pitch_damping_ratio: 0.,
                direct_roll_accel_per_kpa: 0.,
                direct_roll_accel_max: 0.,
                direct_roll_damping_s_inv: 0.,
                direct_yaw_accel_per_kpa: 0.,
                direct_yaw_accel_max: 0.,
                direct_yaw_stiffness_per_kpa: 0.,
            },
            mass_kg: 0.,
            ut_s: 0.,
            trim_aoa_ceiling_rad: 0.,
        },
        final_geometry: TaemGeometryState {
            runway_along_m: 0.,
            runway_cross_m: 0.,
            altitude_above_runway_m: 0.,
            ground_speed_mps: 0.,
            airspeed_mps: 0.,
            course_deg: 0.,
            flight_path_angle_deg: 0.,
            heading_error_deg: 0.,
            curvature_right_per_m: 0.,
        },
        elapsed_s: 0.,
        minimum_speed_mps: 0.,
        maximum_dynamic_pressure_pa: 0.,
        maximum_load_g: 0.,
        maximum_cross_track_m: 0.,
        maximum_course_error_deg: 0.,
        final_route_course_error_deg: 0.,
        maximum_altitude_error_m: 0.,
        maximum_lateral_authority_shortfall_mps2: 0.,
        maximum_vertical_authority_shortfall_mps2: 0.,
        turn_burden_integral_s: 0.,
        maximum_turn_authority_fraction: 0.,
        maximum_commanded_aoa_deg: 0.,
        bank_target_reversals: 0,
        failure_cross_track_m: 0.,
        failure_course_error_deg: 0.,
        failure_required_lateral_accel_mps2: 0.,
        failure_available_lateral_accel_mps2: 0.,
        failure_required_vertical_lift_mps2: 0.,
        failure_delivered_vertical_lift_mps2: 0.,
        failure_target_altitude_m: 0.,
        failure_target_flight_path_angle_deg: 0.,
        failure_route_index: 0,
        energy_start_j_kg: 0.,
        energy_end_j_kg: 0.,
        drag_work_j_kg: 0.,
        energy_closure_residual_j_kg: 0.,
        final_route_index: 0,
        final_target_altitude_m: 0.,
        final_target_flight_path_angle_deg: 0.,
        reason: ::core::ptr::null::<::core::ffi::c_char>(),
    };
    memset(
        &raw mut out as *mut ::core::ffi::c_void,
        0 as ::core::ffi::c_int,
        ::core::mem::size_of::<TerminalSolverResult>() as size_t,
    );
    out.status = TERMINAL_SOLVER_INVALID_INPUT;
    out.reason = b"invalid solver input\0" as *const u8 as *const ::core::ffi::c_char;
    if m.is_null() || !(*m).replay_validated
        || !terminal_model_validate(
            m,
            ::core::ptr::null_mut::<::core::ffi::c_char>(),
            0 as size_t,
        )
        || !terminal_state_validate(
            initial,
            ::core::ptr::null_mut::<::core::ffi::c_char>(),
            0 as size_t,
        ) || route.is_null() || !(*route).valid || (*route).count < 3 as size_t
        || !(dt > 0.0f64) || !(max_elapsed > 0.0f64) || dt.is_finite() as i32 == 0
        || max_elapsed.is_finite() as i32 == 0
        || (max_elapsed / dt).is_finite() as i32 == 0
        || max_elapsed / dt
            >= SIZE_MAX as ::core::ffi::c_double
                / ::core::mem::size_of::<ProfileSample>() as ::core::ffi::c_double
                - 1.0f64
    {
        return out;
    }
    let mut state: TerminalDynamicState = *initial;
    let mut geometry: TaemGeometryState = TaemGeometryState {
        runway_along_m: 0.,
        runway_cross_m: 0.,
        altitude_above_runway_m: 0.,
        ground_speed_mps: 0.,
        airspeed_mps: 0.,
        course_deg: 0.,
        flight_path_angle_deg: 0.,
        heading_error_deg: 0.,
        curvature_right_per_m: 0.,
    };
    if !taem_geometry_state(m, &raw mut state, &raw mut geometry) {
        out.reason = b"initial runway-frame projection failed\0" as *const u8
            as *const ::core::ffi::c_char;
        return out;
    }
    let mut cursor: size_t = 0 as size_t;
    let mut initial_speed: ::core::ffi::c_double = geometry.airspeed_mps;
    out.minimum_speed_mps = initial_speed;
    out.energy_start_j_kg = ::core::f32::NAN as ::core::ffi::c_double;
    out.energy_end_j_kg = ::core::f32::NAN as ::core::ffi::c_double;
    let mut initial_forces: AeroForces = aero_compute(
        &raw const (*m).world,
        &raw const (*m).aero,
        state.position_i_m,
        state.velocity_i_mps,
        state.ut_s,
        state.mass_kg,
        state.attitude.aoa_rad,
        state.attitude.bank_rad,
    );
    let mut energy: TaemEnergyDiagnostic = TaemEnergyDiagnostic {
        gravitational_potential_j_kg: 0.,
        inertial_specific_energy_j_kg: 0.,
        effective_specific_energy_j_kg: 0.,
        inertial_energy_rate_w_kg: 0.,
        effective_energy_rate_w_kg: 0.,
        airspeed_mps: 0.,
        drag_work_rate_w_kg: 0.,
    };
    if state_energy(m, &raw mut state, initial_forces, &raw mut energy) {
        out.energy_start_j_kg = energy.effective_specific_energy_j_kg;
    }
    let mut reached_gate: bool = false_0 != 0;
    let mut elapsed: ::core::ffi::c_double = 0.0f64;
    let mut max_steps: size_t = ceil(max_elapsed / dt) as size_t;
    let mut diagnostics: *const ::core::ffi::c_char = getenv(
        b"KSP_LANDER_TAEM_DIAGNOSTICS\0" as *const u8 as *const ::core::ffi::c_char,
    );
    let mut last_turn_sign: ::core::ffi::c_int = 0 as ::core::ffi::c_int;
    let mut step: size_t = 0 as size_t;
    while step < max_steps {
        let mut reference: TaemPathReference = TaemPathReference {
            runway_along_m: 0.,
            runway_cross_m: 0.,
            course_deg: 0.,
            curvature_right_per_m: 0.,
            altitude_m: 0.,
            flight_path_angle_deg: 0.,
            vertical_curvature_per_m: 0.,
            station_m: 0.,
        };
        let mut reference_index: size_t = cursor;
        if !taem_route_reference(
            route,
            &raw mut geometry,
            &raw mut cursor,
            &raw mut reference,
            &raw mut reference_index,
        ) {
            out.status = TERMINAL_SOLVER_INVALID_INPUT;
            out.reason = b"route reference lookup failed\0" as *const u8
                as *const ::core::ffi::c_char;
            break;
        } else {
            if !diagnostics.is_null()
                && strcmp(diagnostics, b"1\0" as *const u8 as *const ::core::ffi::c_char)
                    == 0 as ::core::ffi::c_int && step < 8 as size_t
            {
                fprintf(
                    stderr,
                    b"TAEM replay step: bow=%+.0f k=%zu idx=%zu along=%.0f refAlong=%.0f cross=%.0f refCross=%.0f course=%.1f refCourse=%.1f kappa=%g alt=%.0f refAlt=%.0f gamma=%.2f refGamma=%.2f\n\0"
                        as *const u8 as *const ::core::ffi::c_char,
                    (*route).profile_midpoint_offset_m,
                    step,
                    reference_index,
                    geometry.runway_along_m,
                    reference.runway_along_m,
                    geometry.runway_cross_m,
                    reference.runway_cross_m,
                    geometry.course_deg,
                    reference.course_deg,
                    reference.curvature_right_per_m,
                    geometry.altitude_above_runway_m,
                    reference.altitude_m,
                    geometry.flight_path_angle_deg,
                    reference.flight_path_angle_deg,
                );
            }
            let mut demand: TaemTrackerOutput = taem_tracker_update(
                m,
                &raw mut state,
                &raw mut geometry,
                &raw mut reference,
                dt,
            );
            if !diagnostics.is_null()
                && strcmp(diagnostics, b"2\0" as *const u8 as *const ::core::ffi::c_char)
                    == 0 as ::core::ffi::c_int
                && step.wrapping_rem(fmax(1.0f64, round(2.5f64 / dt)) as size_t)
                    == 0 as size_t
            {
                fprintf(
                    stderr,
                    b"TAEM replay trace: t=%.1f idx=%zu along=%.0f cross=%.0f h=%.0f V=%.1f course=%.1f ref=%.1f kappa=%.3g xt=%.0f crsErr=%.1f fpa=%.1f refFpa=%.1f bank=%.1f cmdBank=%.1f aoa=%.1f cmdAoa=%.1f\n\0"
                        as *const u8 as *const ::core::ffi::c_char,
                    elapsed,
                    reference_index,
                    geometry.runway_along_m,
                    geometry.runway_cross_m,
                    geometry.altitude_above_runway_m,
                    geometry.airspeed_mps,
                    geometry.course_deg,
                    reference.course_deg,
                    reference.curvature_right_per_m,
                    demand.cross_track_error_m,
                    demand.course_error_deg,
                    geometry.flight_path_angle_deg,
                    reference.flight_path_angle_deg,
                    state.attitude.bank_rad * 57.29577951308232f64,
                    demand.control.bank_rad * 57.29577951308232f64,
                    state.attitude.aoa_rad * 57.29577951308232f64,
                    demand.control.angle_of_attack_rad * 57.29577951308232f64,
                );
            }
            if !demand.valid {
                out.status = TERMINAL_SOLVER_INFEASIBLE;
                out.reason = b"tracker could not produce a bounded command\0"
                    as *const u8 as *const ::core::ffi::c_char;
                break;
            } else {
                out.failure_target_altitude_m = reference.altitude_m;
                out.failure_target_flight_path_angle_deg = reference
                    .flight_path_angle_deg;
                out.failure_route_index = reference_index;
                out.failure_cross_track_m = demand.cross_track_error_m;
                out.failure_course_error_deg = demand.course_error_deg;
                out.failure_required_lateral_accel_mps2 = demand
                    .required_lateral_accel_mps2;
                out.failure_available_lateral_accel_mps2 = demand
                    .available_lateral_accel_mps2;
                out.failure_required_vertical_lift_mps2 = demand
                    .required_vertical_lift_mps2;
                out.failure_delivered_vertical_lift_mps2 = demand
                    .delivered_vertical_lift_mps2;
                out.maximum_lateral_authority_shortfall_mps2 = fmax(
                    out.maximum_lateral_authority_shortfall_mps2,
                    fabs(demand.required_lateral_accel_mps2)
                        - demand.available_lateral_accel_mps2,
                );
                out.maximum_vertical_authority_shortfall_mps2 = fmax(
                    out.maximum_vertical_authority_shortfall_mps2,
                    fabs(
                        demand.required_vertical_lift_mps2
                            - demand.delivered_vertical_lift_mps2,
                    ),
                );
                out.maximum_cross_track_m = fmax(
                    out.maximum_cross_track_m,
                    fabs(demand.cross_track_error_m),
                );
                out.maximum_course_error_deg = fmax(
                    out.maximum_course_error_deg,
                    fabs(demand.course_error_deg),
                );
                let mut forces: AeroForces = aero_compute(
                    &raw const (*m).world,
                    &raw const (*m).aero,
                    state.position_i_m,
                    state.velocity_i_mps,
                    state.ut_s,
                    state.mass_kg,
                    state.attitude.aoa_rad,
                    state.attitude.bank_rad,
                );
                record_turn_demand(
                    m,
                    &raw mut demand,
                    forces.mach,
                    dt,
                    &raw mut last_turn_sign,
                    &raw mut out,
                );
                let mut altitude: ::core::ffi::c_double = v3_norm(state.position_i_m)
                    - (*m).world.radius_m;
                let mut local_radius: ::core::ffi::c_double = (*m).world.radius_m
                    + altitude;
                let mut gravity: ::core::ffi::c_double = (*m).world.mu_m3_s2
                    / (local_radius * local_radius);
                let mut load: ::core::ffi::c_double = fabs(forces.lift_n) / state.mass_kg
                    / 9.80665f64;
                out.maximum_load_g = fmax(out.maximum_load_g, load);
                out.maximum_dynamic_pressure_pa = fmax(
                    out.maximum_dynamic_pressure_pa,
                    forces.dynamic_pressure_pa,
                );
                out.minimum_speed_mps = fmin(out.minimum_speed_mps, forces.airspeed_mps);
                if forces.dynamic_pressure_pa
                    > (*m).vehicle.maximum_dynamic_pressure * (1.0f64 + 1e-9f64)
                    || load > (*m).vehicle.maximum_g_load * (1.0f64 + 1e-6f64)
                {
                    out.status = TERMINAL_SOLVER_INFEASIBLE;
                    out.reason = b"candidate exceeded dynamic-pressure or load limit\0"
                        as *const u8 as *const ::core::ffi::c_char;
                    break;
                } else {
                    let mut drag_work_rate: ::core::ffi::c_double = -forces.drag_n
                        * forces.airspeed_mps / state.mass_kg;
                    out.drag_work_j_kg += drag_work_rate * dt;
                    let mut status: TerminalStepStatus = terminal_propagator_step(
                        m,
                        &raw mut state,
                        &raw mut demand.control,
                    );
                    if status as ::core::ffi::c_uint
                        != TERMINAL_STEP_OK as ::core::ffi::c_int as ::core::ffi::c_uint
                    {
                        out.status = TERMINAL_SOLVER_INFEASIBLE;
                        out.reason = b"native airborne propagation failed\0" as *const u8
                            as *const ::core::ffi::c_char;
                        break;
                    } else {
                        elapsed += dt;
                        if !taem_geometry_state(m, &raw mut state, &raw mut geometry) {
                            out.status = TERMINAL_SOLVER_INFEASIBLE;
                            out.reason = b"runway-frame state became invalid\0"
                                as *const u8 as *const ::core::ffi::c_char;
                            break;
                        } else if geometry.altitude_above_runway_m <= 0.0f64 {
                            out.status = TERMINAL_SOLVER_INFEASIBLE;
                            out.reason = b"candidate crossed runway elevation before the HAC exit\0"
                                as *const u8 as *const ::core::ffi::c_char;
                            break;
                        } else {
                            let mut emergency_cross_track_m: ::core::ffi::c_double = fmax(
                                3000.0f64,
                                0.25f64 * (*route).hac.radius_m,
                            );
                            if fabs(demand.cross_track_error_m) > emergency_cross_track_m
                                || fabs(demand.course_error_deg) > 45.0f64
                            {
                                out.status = TERMINAL_SOLVER_INFEASIBLE;
                                out.reason = b"candidate diverged from the MM305 tracking corridor\0"
                                    as *const u8 as *const ::core::ffi::c_char;
                                break;
                            } else {
                                let mut current_altitude: ::core::ffi::c_double = (*m)
                                    .site
                                    .altitude + geometry.altitude_above_runway_m;
                                out.maximum_altitude_error_m = fmax(
                                    out.maximum_altitude_error_m,
                                    fabs(current_altitude - reference.altitude_m),
                                );
                                let mut end_forces: AeroForces = aero_compute(
                                    &raw const (*m).world,
                                    &raw const (*m).aero,
                                    state.position_i_m,
                                    state.velocity_i_mps,
                                    state.ut_s,
                                    state.mass_kg,
                                    state.attitude.aoa_rad,
                                    state.attitude.bank_rad,
                                );
                                if state_energy(
                                    m,
                                    &raw mut state,
                                    end_forces,
                                    &raw mut energy,
                                ) {
                                    out.energy_end_j_kg = energy.effective_specific_energy_j_kg;
                                }
                                let mut end_distance: ::core::ffi::c_double = hypot(
                                    geometry.runway_along_m - (*route).alignment_along_m,
                                    geometry.runway_cross_m,
                                );
                                let mut route_remaining: ::core::ffi::c_double = taem_route_remaining_at_index(
                                    route,
                                    cursor,
                                );
                                let mut alignment_capture: ::core::ffi::c_double = taem_alignment_capture_distance(
                                    m,
                                    &raw mut geometry,
                                );
                                let mut in_alignment_capture: bool = route_remaining
                                    .is_finite() as i32 != 0
                                    && route_remaining <= alignment_capture;
                                if in_alignment_capture as ::core::ffi::c_int != 0
                                    && end_distance <= alignment_capture + 500.0f64
                                    && fabs(demand.course_error_deg) <= 12.0f64
                                {
                                    reached_gate = true_0 != 0;
                                    out.final_route_index = reference_index;
                                    out.final_route_course_error_deg = demand.course_error_deg;
                                    out.final_target_altitude_m = reference.altitude_m;
                                    out.final_target_flight_path_angle_deg = reference
                                        .flight_path_angle_deg;
                                    if !diagnostics.is_null() {
                                        fprintf(
                                            stderr,
                                            b"TAEM alignment profile compare: t=%.1f station=%.1f remaining=%.1f plannedV=%.2f actualV=%.2f plannedExitV=%.2f referenceH=%.1f actualH=%.1f referenceFpa=%.2f actualFpa=%.2f\n\0"
                                                as *const u8 as *const ::core::ffi::c_char,
                                            elapsed,
                                            reference.station_m,
                                            route_remaining,
                                            taem_route_planned_speed(route, reference.station_m),
                                            geometry.airspeed_mps,
                                            taem_route_planned_speed(route, (*route).length_m),
                                            reference.altitude_m - (*m).site.altitude,
                                            geometry.altitude_above_runway_m,
                                            reference.flight_path_angle_deg,
                                            geometry.flight_path_angle_deg,
                                        );
                                    }
                                    if !diagnostics.is_null() {
                                        fprintf(
                                            stderr,
                                            b"TAEM alignment gate entry: t=%.1f along=%.1f cross=%.1f courseErr=%.2f bank=%.2f bankRate=%.2f h=%.1f V=%.1f fpa=%.2f\n\0"
                                                as *const u8 as *const ::core::ffi::c_char,
                                            elapsed,
                                            geometry.runway_along_m,
                                            geometry.runway_cross_m,
                                            geometry.heading_error_deg,
                                            state.attitude.bank_rad * 57.29577951308232f64,
                                            state.attitude.bank_rate_rad_s * 57.29577951308232f64,
                                            geometry.altitude_above_runway_m,
                                            geometry.airspeed_mps,
                                            geometry.flight_path_angle_deg,
                                        );
                                    }
                                    break;
                                } else {
                                    out.final_route_index = reference_index;
                                    step = step.wrapping_add(1);
                                }
                            }
                        }
                    }
                }
            }
        }
    }
    if reached_gate {
        reached_gate = false_0 != 0;
        while elapsed + dt <= max_elapsed + 1e-9f64 {
            if taem_alignment_exhausted(m, &raw mut geometry) {
                if !diagnostics.is_null() {
                    fprintf(
                        stderr,
                        b"TAEM alignment miss: t=%.1f along=%.1f targetAlong=%.1f cross=%.1f courseErr=%.2f bank=%.2f bankRate=%.2f h=%.1f targetH=%.1f V=%.1f targetV=%.1f fpa=%.2f targetFpa=%.2f\n\0"
                            as *const u8 as *const ::core::ffi::c_char,
                        elapsed,
                        geometry.runway_along_m,
                        -(*m).guidance.final_approach_distance,
                        geometry.runway_cross_m,
                        geometry.heading_error_deg,
                        state.attitude.bank_rad * 57.29577951308232f64,
                        state.attitude.bank_rate_rad_s * 57.29577951308232f64,
                        geometry.altitude_above_runway_m,
                        (*m).guidance.final_approach_distance
                            * tan(
                                (*m).guidance.final_glide_slope / 57.29577951308232f64,
                            ),
                        geometry.airspeed_mps,
                        fmax(
                            (*m).guidance.final_alignment_speed,
                            (*m).vehicle.touchdown_speed,
                        ),
                        geometry.flight_path_angle_deg,
                        -(*m).guidance.final_glide_slope,
                    );
                }
                out.status = TERMINAL_SOLVER_INFEASIBLE;
                out.reason = b"Final alignment point was missed\0" as *const u8
                    as *const ::core::ffi::c_char;
                break;
            } else {
                if !diagnostics.is_null()
                    && fabs(
                        geometry.runway_along_m + (*m).guidance.final_approach_distance,
                    ) <= 300.0f64
                {
                    let mut target_h: ::core::ffi::c_double = (*m)
                        .guidance
                        .final_approach_distance
                        * tan((*m).guidance.final_glide_slope / 57.29577951308232f64);
                    let mut target_v: ::core::ffi::c_double = fmax(
                        (*m).guidance.final_alignment_speed,
                        (*m).vehicle.touchdown_speed,
                    );
                    fprintf(
                        stderr,
                        b"TAEM alignment station sample: t=%.1f along=%.1f cross=%.1f hdgErr=%.2f bank=%.2f bankRate=%.2f h=%.1f dh=%+.1f V=%.1f dV=%+.1f fpa=%.2f dFpa=%+.2f ready=%d\n\0"
                            as *const u8 as *const ::core::ffi::c_char,
                        elapsed,
                        geometry.runway_along_m,
                        geometry.runway_cross_m,
                        geometry.heading_error_deg,
                        state.attitude.bank_rad * 57.29577951308232f64,
                        state.attitude.bank_rate_rad_s * 57.29577951308232f64,
                        geometry.altitude_above_runway_m,
                        geometry.altitude_above_runway_m - target_h,
                        geometry.airspeed_mps,
                        geometry.airspeed_mps - target_v,
                        geometry.flight_path_angle_deg,
                        geometry.flight_path_angle_deg + (*m).guidance.final_glide_slope,
                        if taem_alignment_ready(m, &raw mut state, &raw mut geometry)
                            as ::core::ffi::c_int != 0
                        {
                            1 as ::core::ffi::c_int
                        } else {
                            0 as ::core::ffi::c_int
                        },
                    );
                }
                if taem_alignment_ready(m, &raw mut state, &raw mut geometry) {
                    reached_gate = true_0 != 0;
                    let mut reference_0: TaemPathReference = taem_alignment_reference(
                        m,
                        &raw mut geometry,
                    );
                    out.final_route_course_error_deg = remainder(
                        geometry.course_deg - reference_0.course_deg,
                        360.0f64,
                    );
                    out.final_target_altitude_m = taem_alignment_target_altitude(m);
                    out.final_target_flight_path_angle_deg = reference_0
                        .flight_path_angle_deg;
                    break;
                } else {
                    let mut reference_1: TaemPathReference = taem_alignment_reference(
                        m,
                        &raw mut geometry,
                    );
                    let mut demand_0: TaemTrackerOutput = taem_tracker_update(
                        m,
                        &raw mut state,
                        &raw mut geometry,
                        &raw mut reference_1,
                        dt,
                    );
                    if !demand_0.valid {
                        out.status = TERMINAL_SOLVER_INFEASIBLE;
                        out.reason = b"runway-line tracker could not produce a command\0"
                            as *const u8 as *const ::core::ffi::c_char;
                        break;
                    } else {
                        let mut forces_0: AeroForces = aero_compute(
                            &raw const (*m).world,
                            &raw const (*m).aero,
                            state.position_i_m,
                            state.velocity_i_mps,
                            state.ut_s,
                            state.mass_kg,
                            state.attitude.aoa_rad,
                            state.attitude.bank_rad,
                        );
                        record_turn_demand(
                            m,
                            &raw mut demand_0,
                            forces_0.mach,
                            dt,
                            &raw mut last_turn_sign,
                            &raw mut out,
                        );
                        let mut load_0: ::core::ffi::c_double = fabs(forces_0.lift_n)
                            / state.mass_kg / 9.80665f64;
                        out.maximum_load_g = fmax(out.maximum_load_g, load_0);
                        out.maximum_dynamic_pressure_pa = fmax(
                            out.maximum_dynamic_pressure_pa,
                            forces_0.dynamic_pressure_pa,
                        );
                        out.minimum_speed_mps = fmin(
                            out.minimum_speed_mps,
                            forces_0.airspeed_mps,
                        );
                        if forces_0.dynamic_pressure_pa
                            > (*m).vehicle.maximum_dynamic_pressure
                            || load_0 > (*m).vehicle.maximum_g_load * (1.0f64 + 1e-6f64)
                        {
                            out.status = TERMINAL_SOLVER_INFEASIBLE;
                            out.reason = b"runway-line alignment exceeded pressure or load limit\0"
                                as *const u8 as *const ::core::ffi::c_char;
                            break;
                        } else {
                            out.drag_work_j_kg
                                -= forces_0.drag_n * forces_0.airspeed_mps / state.mass_kg
                                    * dt;
                            if terminal_propagator_step(
                                m,
                                &raw mut state,
                                &raw mut demand_0.control,
                            ) as ::core::ffi::c_uint
                                != TERMINAL_STEP_OK as ::core::ffi::c_int
                                    as ::core::ffi::c_uint
                                || !taem_geometry_state(
                                    m,
                                    &raw mut state,
                                    &raw mut geometry,
                                ) || geometry.altitude_above_runway_m <= 0.0f64
                            {
                                out.status = TERMINAL_SOLVER_INFEASIBLE;
                                out.reason = b"runway-line alignment propagation failed\0"
                                    as *const u8 as *const ::core::ffi::c_char;
                                break;
                            } else {
                                elapsed += dt;
                                out.maximum_cross_track_m = fmax(
                                    out.maximum_cross_track_m,
                                    fabs(geometry.runway_cross_m),
                                );
                                out.maximum_course_error_deg = fmax(
                                    out.maximum_course_error_deg,
                                    fabs(geometry.heading_error_deg),
                                );
                                out.maximum_altitude_error_m = fmax(
                                    out.maximum_altitude_error_m,
                                    fabs(
                                        (*m).site.altitude + geometry.altitude_above_runway_m
                                            - reference_1.altitude_m,
                                    ),
                                );
                                let mut end_forces_0: AeroForces = aero_compute(
                                    &raw const (*m).world,
                                    &raw const (*m).aero,
                                    state.position_i_m,
                                    state.velocity_i_mps,
                                    state.ut_s,
                                    state.mass_kg,
                                    state.attitude.aoa_rad,
                                    state.attitude.bank_rad,
                                );
                                if state_energy(
                                    m,
                                    &raw mut state,
                                    end_forces_0,
                                    &raw mut energy,
                                ) {
                                    out.energy_end_j_kg = energy.effective_specific_energy_j_kg;
                                }
                            }
                        }
                    }
                }
            }
        }
        if !reached_gate
            && out.status as ::core::ffi::c_uint
                != TERMINAL_SOLVER_INFEASIBLE as ::core::ffi::c_int
                    as ::core::ffi::c_uint
        {
            out.status = TERMINAL_SOLVER_SEARCH_EXHAUSTED;
            out.reason = b"runway-line alignment exceeded replay time\0" as *const u8
                as *const ::core::ffi::c_char;
        }
    }
    out.final_state = state;
    out.final_geometry = geometry;
    out.elapsed_s = elapsed;
    if out.energy_start_j_kg.is_finite() as i32 != 0
        && out.energy_end_j_kg.is_finite() as i32 != 0
    {
        out.energy_closure_residual_j_kg = out.energy_end_j_kg - out.energy_start_j_kg
            - out.drag_work_j_kg;
    }
    if reached_gate {
        let mut energy_scale: ::core::ffi::c_double = fmax(
            fabs(out.drag_work_j_kg),
            0.5f64 * initial_speed * initial_speed,
        );
        let mut energy_tolerance: ::core::ffi::c_double = fmax(
            250.0f64,
            0.02f64 * energy_scale,
        );
        let mut energy_closure_ok: bool = out.energy_closure_residual_j_kg.is_finite()
            as i32 != 0 && fabs(out.energy_closure_residual_j_kg) <= energy_tolerance;
        let mut path_geometry_ok: bool = out.maximum_cross_track_m
            <= fmax(1200.0f64, 0.1f64 * (*route).hac.radius_m)
            && out.maximum_course_error_deg <= 20.0f64
            && out.maximum_altitude_error_m <= 2000.0f64
            && fabs(
                out.final_geometry.altitude_above_runway_m + (*m).site.altitude
                    - out.final_target_altitude_m,
            ) <= 1200.0f64
            && fabs(
                out.final_geometry.flight_path_angle_deg
                    - out.final_target_flight_path_angle_deg,
            ) <= 6.0f64;
        let mut speed_ok: bool = out.minimum_speed_mps
            >= (*m).vehicle.minimum_safe_speed;
        out.path_constraints_ok = path_geometry_ok as ::core::ffi::c_int != 0
            && speed_ok as ::core::ffi::c_int != 0
            && energy_closure_ok as ::core::ffi::c_int != 0;
        if out.path_constraints_ok {
            out.status = TERMINAL_SOLVER_UNQUALIFIED;
            out.reason = b"runway-aligned MM305 exit reached; downstream Final tail is not qualified\0"
                as *const u8 as *const ::core::ffi::c_char;
        } else {
            out.status = TERMINAL_SOLVER_INFEASIBLE;
            out.reason = if !path_geometry_ok {
                b"candidate exceeded path tracking envelope\0" as *const u8
                    as *const ::core::ffi::c_char
            } else if !speed_ok {
                b"candidate fell below the vehicle minimum safe speed\0" as *const u8
                    as *const ::core::ffi::c_char
            } else {
                b"native energy and drag-work closure exceeded tolerance\0" as *const u8
                    as *const ::core::ffi::c_char
            };
        }
    } else if out.status as ::core::ffi::c_uint
        == TERMINAL_SOLVER_INVALID_INPUT as ::core::ffi::c_int as ::core::ffi::c_uint
        && elapsed > 0.0f64
    {
        out.status = TERMINAL_SOLVER_SEARCH_EXHAUSTED;
        out.reason = b"native replay reached its wall-simulation horizon before HAC exit\0"
            as *const u8 as *const ::core::ffi::c_char;
    }
    if !diagnostics.is_null()
        && strcmp(diagnostics, b"1\0" as *const u8 as *const ::core::ffi::c_char)
            == 0 as ::core::ffi::c_int
    {
        fprintf(
            stderr,
            b"TAEM replay result: status=%d elapsed=%.2f index=%zu reason=%s lat=%.2f/%.2f vert=%.2f/%.2f\n\0"
                as *const u8 as *const ::core::ffi::c_char,
            out.status as ::core::ffi::c_int,
            out.elapsed_s,
            out.failure_route_index,
            if !out.reason.is_null() {
                out.reason
            } else {
                b"none\0" as *const u8 as *const ::core::ffi::c_char
            },
            out.failure_required_lateral_accel_mps2,
            out.failure_available_lateral_accel_mps2,
            out.failure_required_vertical_lift_mps2,
            out.failure_delivered_vertical_lift_mps2,
        );
    }
    return out;
}
#[inline]
unsafe extern "C" fn taem_alignment_target_height(
    mut model: *const TerminalModel,
) -> ::core::ffi::c_double {
    return (*model).guidance.final_approach_distance
        * tan((*model).guidance.final_glide_slope / 57.29577951308232f64);
}
#[inline]
unsafe extern "C" fn taem_alignment_target_altitude(
    mut model: *const TerminalModel,
) -> ::core::ffi::c_double {
    return (*model).site.altitude + taem_alignment_target_height(model);
}
#[inline]
unsafe extern "C" fn taem_alignment_capture_distance(
    mut model: *const TerminalModel,
    mut geometry: *const TaemGeometryState,
) -> ::core::ffi::c_double {
    let mut speed: ::core::ffi::c_double = if !geometry.is_null() {
        fmax(0.0f64, (*geometry).ground_speed_mps)
    } else {
        0.0f64
    };
    let mut response_distance: ::core::ffi::c_double = 30.0f64 * speed;
    return fmax(3000.0f64, fmin(6000.0f64, response_distance));
}
#[inline]
unsafe extern "C" fn taem_alignment_reference(
    mut model: *const TerminalModel,
    mut geometry: *const TaemGeometryState,
) -> TaemPathReference {
    let radians_per_degree: ::core::ffi::c_double = 0.017453292519943295f64;
    let mut station: ::core::ffi::c_double = -(*model).guidance.final_approach_distance;
    let mut remaining: ::core::ffi::c_double = station - (*geometry).runway_along_m;
    let mut effective_remaining: ::core::ffi::c_double = fmax(2000.0f64, remaining);
    let mut course_offset: ::core::ffi::c_double = atan2(
        -2.0f64 * (*geometry).runway_cross_m,
        effective_remaining,
    ) / radians_per_degree;
    course_offset = fmax(-20.0f64, fmin(20.0f64, course_offset));
    let mut final_slope: ::core::ffi::c_double = (*model).guidance.final_glide_slope;
    let mut target_height: ::core::ffi::c_double = taem_alignment_target_height(model);
    let mut upstream: ::core::ffi::c_double = fmax(0.0f64, remaining);
    let mut glide_height: ::core::ffi::c_double = target_height
        + upstream * tan(final_slope * radians_per_degree);
    let mut height_error: ::core::ffi::c_double = glide_height
        - (*geometry).altitude_above_runway_m;
    let mut correction: ::core::ffi::c_double = atan2(height_error, effective_remaining)
        / radians_per_degree;
    correction = fmax(-6.0f64, fmin(6.0f64, correction));
    let mut fpa: ::core::ffi::c_double = -final_slope + correction;
    return TaemPathReference {
        runway_along_m: (*geometry).runway_along_m,
        runway_cross_m: 0.0f64,
        course_deg: (*model).site.runway_heading + course_offset,
        curvature_right_per_m: 0.0f64,
        altitude_m: ::core::f32::NAN as ::core::ffi::c_double,
        flight_path_angle_deg: fpa,
        vertical_curvature_per_m: 0.0f64,
        station_m: 0.0f64,
    };
}
#[inline]
unsafe extern "C" fn taem_alignment_ready(
    mut model: *const TerminalModel,
    mut state: *const TerminalDynamicState,
    mut geometry: *const TaemGeometryState,
) -> bool {
    let degrees_per_radian: ::core::ffi::c_double = 57.29577951308232f64;
    let mut station: ::core::ffi::c_double = -(*model).guidance.final_approach_distance;
    let mut target_height: ::core::ffi::c_double = taem_alignment_target_height(model);
    let mut target_speed: ::core::ffi::c_double = fmax(
        (*model).guidance.final_alignment_speed,
        (*model).vehicle.touchdown_speed,
    );
    return fabs((*geometry).runway_cross_m) < 0.5f64 * (*model).site.runway_width
        && fabs((*geometry).heading_error_deg) < 2.0f64
        && fabs((*state).attitude.bank_rad * degrees_per_radian) < 5.0f64
        && fabs((*state).attitude.bank_rate_rad_s * degrees_per_radian) < 3.0f64
        && fabs((*geometry).runway_along_m - station) <= 250.0f64
        && fabs((*geometry).altitude_above_runway_m - target_height) <= 150.0f64
        && fabs((*geometry).airspeed_mps - target_speed) <= 10.0f64
        && fabs((*geometry).flight_path_angle_deg + (*model).guidance.final_glide_slope)
            <= 3.0f64;
}
#[inline]
unsafe extern "C" fn taem_alignment_exhausted(
    mut model: *const TerminalModel,
    mut geometry: *const TaemGeometryState,
) -> bool {
    let mut station: ::core::ffi::c_double = -(*model).guidance.final_approach_distance;
    if (*geometry).flight_path_angle_deg < -(*model).guidance.final_glide_slope - 6.0f64
    {
        return (*geometry).runway_along_m > station;
    }
    return (*geometry).runway_along_m > station + 500.0f64
        || (*geometry).altitude_above_runway_m < 300.0f64;
}
pub const true_0: ::core::ffi::c_int = 1 as ::core::ffi::c_int;
pub const false_0: ::core::ffi::c_int = 0 as ::core::ffi::c_int;
pub const NULL: *mut ::core::ffi::c_void = ::core::ptr::null_mut::<
    ::core::ffi::c_void,
>();
