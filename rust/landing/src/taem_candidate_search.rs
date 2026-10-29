use crate::sys::*;
extern "C" {
    fn world_atmosphere_sample(
        w: *const KerbinWorld,
        altitude_m: ::core::ffi::c_double,
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
    fn taem_geometry_state(
        model: *const TerminalModel,
        state: *const TerminalDynamicState,
        geometry: *mut TaemGeometryState,
    ) -> bool;
    fn atan(__x: ::core::ffi::c_double) -> ::core::ffi::c_double;
    fn tan(__x: ::core::ffi::c_double) -> ::core::ffi::c_double;
    fn log1p(__x: ::core::ffi::c_double) -> ::core::ffi::c_double;
    fn fabs(__x: ::core::ffi::c_double) -> ::core::ffi::c_double;
    fn fmax(
        __x: ::core::ffi::c_double,
        __y: ::core::ffi::c_double,
    ) -> ::core::ffi::c_double;
    fn fmin(
        __x: ::core::ffi::c_double,
        __y: ::core::ffi::c_double,
    ) -> ::core::ffi::c_double;
    fn taem_fixed_hac_turn_reachability(
        model: *const TerminalModel,
        state: *const TerminalDynamicState,
        geometry: *const TaemGeometryState,
        radius_m: ::core::ffi::c_double,
        reachability: *mut TaemReachability,
    ) -> bool;
    fn taem_route_build_hac(
        model: *const TerminalModel,
        start: *const TaemGeometryState,
        hac_radius_m: ::core::ffi::c_double,
        side: ::core::ffi::c_double,
        sweep_abs_rad: ::core::ffi::c_double,
        point_spacing_m: ::core::ffi::c_double,
        maximum_lead_curvature_per_m: ::core::ffi::c_double,
        route: *mut TaemRoute,
        reason: *mut ::core::ffi::c_char,
        reason_size: size_t,
    ) -> bool;
    fn taem_route_build_hac_shortest(
        model: *const TerminalModel,
        start: *const TaemGeometryState,
        hac_radius_m: ::core::ffi::c_double,
        side: ::core::ffi::c_double,
        sweep_abs_rad: ::core::ffi::c_double,
        point_spacing_m: ::core::ffi::c_double,
        maximum_lead_curvature_per_m: ::core::ffi::c_double,
        peak_curvature_limit_per_m: ::core::ffi::c_double,
        route: *mut TaemRoute,
        reason: *mut ::core::ffi::c_char,
        reason_size: size_t,
    ) -> bool;
    fn taem_route_build_hac_length(
        model: *const TerminalModel,
        start: *const TaemGeometryState,
        hac_radius_m: ::core::ffi::c_double,
        side: ::core::ffi::c_double,
        sweep_abs_rad: ::core::ffi::c_double,
        point_spacing_m: ::core::ffi::c_double,
        maximum_lead_curvature_per_m: ::core::ffi::c_double,
        peak_curvature_limit_per_m: ::core::ffi::c_double,
        target_lead_length_m: ::core::ffi::c_double,
        route: *mut TaemRoute,
        reason: *mut ::core::ffi::c_char,
        reason_size: size_t,
    ) -> bool;
    fn taem_route_blend_hac_length(
        model: *const TerminalModel,
        start: *const TaemGeometryState,
        route_a: *const TaemRoute,
        route_b: *const TaemRoute,
        point_spacing_m: ::core::ffi::c_double,
        maximum_lead_curvature_per_m: ::core::ffi::c_double,
        peak_curvature_limit_per_m: ::core::ffi::c_double,
        target_lead_length_m: ::core::ffi::c_double,
        route: *mut TaemRoute,
        reason: *mut ::core::ffi::c_char,
        reason_size: size_t,
    ) -> bool;
    fn taem_route_lead_peak_curvature(route: *const TaemRoute) -> ::core::ffi::c_double;
    fn taem_route_planned_altitude(
        route: *const TaemRoute,
        station_m: ::core::ffi::c_double,
    ) -> ::core::ffi::c_double;
    fn taem_route_curvature_at_station(
        route: *const TaemRoute,
        station_m: ::core::ffi::c_double,
    ) -> ::core::ffi::c_double;
    fn terminal_solver_generate_profile(
        model: *const TerminalModel,
        initial: *const TerminalDynamicState,
        route: *mut TaemRoute,
        dt_s: ::core::ffi::c_double,
        maximum_elapsed_s: ::core::ffi::c_double,
    ) -> TerminalProfileResult;
    fn terminal_solver_replay(
        model: *const TerminalModel,
        initial: *const TerminalDynamicState,
        route: *const TaemRoute,
        dt_s: ::core::ffi::c_double,
        maximum_elapsed_s: ::core::ffi::c_double,
    ) -> TerminalSolverResult;
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
pub struct TaemReachability {
    pub valid: bool,
    pub lateral_authority_ok: bool,
    pub energy_qualified: bool,
    pub required_lateral_accel_mps2: ::core::ffi::c_double,
    pub available_lateral_accel_mps2: ::core::ffi::c_double,
    pub lateral_margin_mps2: ::core::ffi::c_double,
    pub minimum_turn_radius_m: ::core::ffi::c_double,
    pub roll_settling_time_s: ::core::ffi::c_double,
    pub roll_settling_distance_m: ::core::ffi::c_double,
}
pub type TaemPlanStatus = ::core::ffi::c_uint;
pub const TAEM_PLAN_QUALIFIED: TaemPlanStatus = 3;
pub const TAEM_PLAN_SEARCH_EXHAUSTED: TaemPlanStatus = 2;
pub const TAEM_PLAN_UNQUALIFIED: TaemPlanStatus = 1;
pub const TAEM_PLAN_INFEASIBLE: TaemPlanStatus = 0;
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
pub struct TaemFixedHacCandidate {
    pub status: TaemPlanStatus,
    pub side: ::core::ffi::c_double,
    pub runway_end: ::core::ffi::c_int,
    pub route_built: bool,
    pub final_tail_qualified: bool,
    pub required_lateral_accel_mps2: ::core::ffi::c_double,
    pub available_lateral_accel_mps2: ::core::ffi::c_double,
    pub quality_score: ::core::ffi::c_double,
    pub route: TaemRoute,
    pub replay: TerminalSolverResult,
    pub reason: *const ::core::ffi::c_char,
}
#[derive(Copy, Clone)]
#[repr(C)]
pub struct TaemFixedHacSearch {
    pub status: TaemPlanStatus,
    pub selected_candidate: ::core::ffi::c_int,
    pub candidate_count: ::core::ffi::c_int,
    pub candidates: [TaemFixedHacCandidate; 4],
    pub reason: *const ::core::ffi::c_char,
}
#[derive(Copy, Clone)]
#[repr(C)]
pub struct JoinStation {
    pub sweep_rad: ::core::ffi::c_double,
    pub score: ::core::ffi::c_double,
    pub route: TaemRoute,
}
pub const TAEM_JOIN_SWEEP_MAX_DEG: ::core::ffi::c_double = 540.0f64;
pub const TAEM_JOIN_SWEEP_MIN_DEG: ::core::ffi::c_double = 15.0f64;
pub const TAEM_HAC_RADIUS_STEPS: ::core::ffi::c_int = 7 as ::core::ffi::c_int;
pub const TAEM_JOIN_SWEEP_STEP_DEG: ::core::ffi::c_double = 30.0f64;
pub const TAEM_JOIN_MAX_STATIONS: ::core::ffi::c_int = 64 as ::core::ffi::c_int;
pub const TAEM_JOIN_MAX_REPLAYS: ::core::ffi::c_int = 2 as ::core::ffi::c_int;
pub const TAEM_JOIN_MAX_PROFILES: ::core::ffi::c_int = 6 as ::core::ffi::c_int;
pub const TAEM_PROFILE_DT_S: ::core::ffi::c_double = 0.5f64;
pub const TAEM_MIN_HAC_RADIUS_M: ::core::ffi::c_double = 3000.0f64;
unsafe extern "C" fn compare_station_length(
    mut a: *const ::core::ffi::c_void,
    mut b: *const ::core::ffi::c_void,
) -> ::core::ffi::c_int {
    let mut la: ::core::ffi::c_double = (*(a as *const JoinStation)).route.length_m;
    let mut lb: ::core::ffi::c_double = (*(b as *const JoinStation)).route.length_m;
    return (la > lb) as ::core::ffi::c_int - (la < lb) as ::core::ffi::c_int;
}
unsafe extern "C" fn compare_station(
    mut a: *const ::core::ffi::c_void,
    mut b: *const ::core::ffi::c_void,
) -> ::core::ffi::c_int {
    let mut x: ::core::ffi::c_double = (*(a as *const JoinStation)).score;
    let mut y: ::core::ffi::c_double = (*(b as *const JoinStation)).score;
    return (x > y) as ::core::ffi::c_int - (x < y) as ::core::ffi::c_int;
}
unsafe extern "C" fn diagnostics_enabled() -> bool {
    let mut diagnostics: *const ::core::ffi::c_char = getenv(
        b"KSP_LANDER_TAEM_DIAGNOSTICS\0" as *const u8 as *const ::core::ffi::c_char,
    );
    return !diagnostics.is_null()
        && (strcmp(diagnostics, b"1\0" as *const u8 as *const ::core::ffi::c_char)
            == 0 as ::core::ffi::c_int
            || strcmp(diagnostics, b"2\0" as *const u8 as *const ::core::ffi::c_char)
                == 0 as ::core::ffi::c_int);
}
unsafe extern "C" fn hermite_peak_descent_deg(
    mut route: *const TaemRoute,
    mut length_m: ::core::ffi::c_double,
) -> ::core::ffi::c_double {
    if route.is_null() || !(length_m > 0.0f64) {
        return ::core::f32::INFINITY as ::core::ffi::c_double;
    }
    let d2r: ::core::ffi::c_double = 3.14159265358979323846f64 / 180.0f64;
    let mut h0: ::core::ffi::c_double = (*route).profile_start_altitude_m;
    let mut h1: ::core::ffi::c_double = (*route).profile_final_altitude_m;
    let mut m0: ::core::ffi::c_double = tan((*route).profile_start_fpa_deg * d2r);
    let mut m1: ::core::ffi::c_double = -tan((*route).profile_final_slope_deg * d2r);
    let mut peak: ::core::ffi::c_double = 0.0f64;
    let mut i: ::core::ffi::c_int = 0 as ::core::ffi::c_int;
    while i <= 96 as ::core::ffi::c_int {
        let mut x: ::core::ffi::c_double = i as ::core::ffi::c_double / 96.0f64;
        let mut x2: ::core::ffi::c_double = x * x;
        let mut dx: ::core::ffi::c_double = (6.0f64 * x2 - 6.0f64 * x) * h0
            + (3.0f64 * x2 - 4.0f64 * x + 1.0f64) * length_m * m0
            + (-6.0f64 * x2 + 6.0f64 * x) * h1
            + (3.0f64 * x2 - 2.0f64 * x) * length_m * m1;
        let mut descent: ::core::ffi::c_double = -atan(dx / length_m) / d2r;
        peak = fmax(peak, descent);
        i += 1;
    }
    return peak;
}
unsafe extern "C" fn minimum_vertical_profile_length(
    mut model: *const TerminalModel,
    mut route: *const TaemRoute,
) -> ::core::ffi::c_double {
    if model.is_null() || route.is_null() {
        return ::core::f32::INFINITY as ::core::ffi::c_double;
    }
    let d2r: ::core::ffi::c_double = 3.14159265358979323846f64 / 180.0f64;
    let mut final_limit: ::core::ffi::c_double = (*route).profile_final_slope_deg
        + 3.0f64;
    let mut limit: ::core::ffi::c_double = if (*model)
        .guidance
        .taem_glide_slope
        .is_finite() as i32 != 0 && (*model).guidance.taem_glide_slope > 0.0f64
    {
        fmax(final_limit, (*model).guidance.taem_glide_slope)
    } else {
        final_limit
    };
    let mut drop_0: ::core::ffi::c_double = (*route).profile_start_altitude_m
        - (*route).profile_final_altitude_m;
    if !(drop_0 > 0.0f64) || !(limit > 0.0f64) || limit >= 89.0f64 {
        return ::core::f32::INFINITY as ::core::ffi::c_double;
    }
    let mut lo: ::core::ffi::c_double = drop_0 / tan(limit * d2r);
    let mut hi: ::core::ffi::c_double = fmax(lo, (*route).length_m);
    let mut i: ::core::ffi::c_int = 0 as ::core::ffi::c_int;
    while i < 8 as ::core::ffi::c_int && hermite_peak_descent_deg(route, hi) > limit {
        hi *= 1.25f64;
        i += 1;
    }
    if hermite_peak_descent_deg(route, hi) > limit {
        return ::core::f32::INFINITY as ::core::ffi::c_double;
    }
    let mut i_0: ::core::ffi::c_int = 0 as ::core::ffi::c_int;
    while i_0 < 24 as ::core::ffi::c_int {
        let mut mid: ::core::ffi::c_double = 0.5f64 * (lo + hi);
        if hermite_peak_descent_deg(route, mid) <= limit {
            hi = mid;
        } else {
            lo = mid;
        }
        i_0 += 1;
    }
    return hi;
}
unsafe extern "C" fn profile_lateral_authority_ratio(
    mut model: *const TerminalModel,
    mut route: *const TaemRoute,
    mut live_curvature: ::core::ffi::c_double,
    mut live_density: ::core::ffi::c_double,
) -> ::core::ffi::c_double {
    if model.is_null() || route.is_null() || !(*route).profile_tabulated
        || !((*route).length_m > 0.0f64) || !(live_curvature > 0.0f64)
        || !(live_density > 0.0f64)
    {
        return ::core::f32::INFINITY as ::core::ffi::c_double;
    }
    let mut worst: ::core::ffi::c_double = 0.0f64;
    let samples: ::core::ffi::c_int = 64 as ::core::ffi::c_int;
    let mut i: ::core::ffi::c_int = 0 as ::core::ffi::c_int;
    while i <= samples {
        let mut station: ::core::ffi::c_double = (*route).length_m
            * i as ::core::ffi::c_double / samples as ::core::ffi::c_double;
        let mut curvature: ::core::ffi::c_double = fabs(
            taem_route_curvature_at_station(route, station),
        );
        let mut altitude: ::core::ffi::c_double = taem_route_planned_altitude(
            route,
            station,
        );
        if curvature.is_finite() as i32 == 0 || altitude.is_finite() as i32 == 0 {
            return ::core::f32::INFINITY as ::core::ffi::c_double;
        }
        let mut atmosphere: AtmosphereSample = world_atmosphere_sample(
            &raw const (*model).world,
            altitude,
        );
        if !(atmosphere.density_kg_m3 > 0.0f64) {
            return ::core::f32::INFINITY as ::core::ffi::c_double;
        }
        let mut available_curvature: ::core::ffi::c_double = live_curvature
            * atmosphere.density_kg_m3 / live_density;
        if !(available_curvature > 0.0f64) || available_curvature.is_finite() as i32 == 0
        {
            return ::core::f32::INFINITY as ::core::ffi::c_double;
        }
        worst = fmax(worst, curvature / available_curvature);
        i += 1;
    }
    return worst;
}
unsafe extern "C" fn replay_reaches_aligned_exit(
    mut replay: *const TerminalSolverResult,
) -> bool {
    return (*replay).status as ::core::ffi::c_uint
        == TERMINAL_SOLVER_UNQUALIFIED as ::core::ffi::c_int as ::core::ffi::c_uint
        && (*replay).path_constraints_ok as ::core::ffi::c_int != 0;
}
unsafe extern "C" fn exit_speed_target(
    mut model: *const TerminalModel,
) -> ::core::ffi::c_double {
    return if (*model).guidance.final_alignment_speed.is_finite() as i32 != 0
        && (*model).guidance.final_alignment_speed > 0.0f64
    {
        (*model).guidance.final_alignment_speed
    } else {
        (*model).vehicle.final_approach_speed
    };
}
unsafe extern "C" fn hac_radius_floor(
    mut model: *const TerminalModel,
) -> ::core::ffi::c_double {
    let mut bank: ::core::ffi::c_double = fmin(
        (*model).vehicle.maximum_bank_angle,
        80.0f64,
    ) * 3.14159265358979323846f64 / 180.0f64;
    let mut speed: ::core::ffi::c_double = exit_speed_target(model);
    let mut radius: ::core::ffi::c_double = (*model).world.radius_m
        + (*model).site.altitude;
    let mut gravity: ::core::ffi::c_double = (*model).world.mu_m3_s2 / (radius * radius);
    return if bank > 0.0f64 && speed > 0.0f64 && gravity.is_finite() as i32 != 0
        && gravity > 0.0f64
    {
        fmax(TAEM_MIN_HAC_RADIUS_M, speed * speed / (gravity * tan(bank)))
    } else {
        ::core::f32::INFINITY as ::core::ffi::c_double
    };
}
unsafe extern "C" fn initial_kinetic(
    mut initial: *const TerminalDynamicState,
) -> ::core::ffi::c_double {
    let mut vx: ::core::ffi::c_double = (*initial).velocity_i_mps.x;
    let mut vy: ::core::ffi::c_double = (*initial).velocity_i_mps.y;
    let mut vz: ::core::ffi::c_double = (*initial).velocity_i_mps.z;
    return 0.5f64 * (vx * vx + vy * vy + vz * vz);
}
unsafe extern "C" fn estimated_energy_route_length(
    mut model: *const TerminalModel,
    mut initial: *const TerminalDynamicState,
    mut geometry: *const TaemGeometryState,
) -> ::core::ffi::c_double {
    if model.is_null() || initial.is_null() || geometry.is_null()
        || !((*initial).mass_kg > 1.0f64)
    {
        return ::core::f32::NAN as ::core::ffi::c_double;
    }
    let mut target_speed: ::core::ffi::c_double = exit_speed_target(model);
    let mut final_height: ::core::ffi::c_double = (*model)
        .guidance
        .final_approach_distance
        * tan(
            (*model).guidance.final_glide_slope * 3.14159265358979323846f64 / 180.0f64,
        );
    let mut radius: ::core::ffi::c_double = (*model).world.radius_m
        + (*model).site.altitude + fmax(0.0f64, (*geometry).altitude_above_runway_m);
    let mut gravity: ::core::ffi::c_double = (*model).world.mu_m3_s2
        / fmax(radius * radius, 1.0f64);
    let mut energy: ::core::ffi::c_double = 0.5f64
        * ((*geometry).airspeed_mps * (*geometry).airspeed_mps
            - target_speed * target_speed)
        + gravity * ((*geometry).altitude_above_runway_m - final_height);
    energy = fmax(0.0f64, energy);
    let mut forces: AeroForces = aero_compute(
        &raw const (*model).world,
        &raw const (*model).aero,
        (*initial).position_i_m,
        (*initial).velocity_i_mps,
        (*initial).ut_s,
        (*initial).mass_kg,
        (*initial).attitude.aoa_rad,
        (*initial).attitude.bank_rad,
    );
    let mut drag: ::core::ffi::c_double = if forces.drag_n > 0.0f64 {
        forces.drag_n / (*initial).mass_kg
    } else {
        ::core::f32::NAN as ::core::ffi::c_double
    };
    if !(drag > 0.0f64) || drag.is_finite() as i32 == 0 {
        return ::core::f32::NAN as ::core::ffi::c_double;
    }
    let mut planning_drag: ::core::ffi::c_double = fmax(0.75f64, drag);
    return energy / planning_drag;
}
unsafe extern "C" fn better_failure(
    mut trial: *const TaemFixedHacCandidate,
    mut best: *const TaemFixedHacCandidate,
    mut have_best: bool,
) -> bool {
    if !have_best {
        return true_0 != 0;
    }
    if (*trial).route_built as ::core::ffi::c_int
        != (*best).route_built as ::core::ffi::c_int
    {
        return (*trial).route_built;
    }
    return (*trial).replay.elapsed_s > (*best).replay.elapsed_s;
}
unsafe extern "C" fn candidate_circle_penalty(
    mut trial: *const TaemFixedHacCandidate,
) -> ::core::ffi::c_double {
    if trial.is_null() || !(*trial).route_built {
        return ::core::f32::INFINITY as ::core::ffi::c_double;
    }
    let mut arc: ::core::ffi::c_double = fmax((*trial).route.hac.arc_length_m, 1.0f64);
    let mut lead: ::core::ffi::c_double = fmax((*trial).route.lead_length_m, 0.0f64);
    return log1p(lead / arc);
}
#[no_mangle]
pub unsafe extern "C" fn taem_fixed_hac_candidate_preferred(
    mut model: *const TerminalModel,
    mut trial: *const TaemFixedHacCandidate,
    mut best: *const TaemFixedHacCandidate,
) -> bool {
    if (*trial).status as ::core::ffi::c_uint
        != TAEM_PLAN_UNQUALIFIED as ::core::ffi::c_int as ::core::ffi::c_uint
    {
        return false_0 != 0;
    }
    if (*best).status as ::core::ffi::c_uint
        != TAEM_PLAN_UNQUALIFIED as ::core::ffi::c_int as ::core::ffi::c_uint
    {
        return true_0 != 0;
    }
    let mut target: ::core::ffi::c_double = exit_speed_target(model);
    let mut trial_speed: ::core::ffi::c_double = (*trial)
        .replay
        .final_geometry
        .airspeed_mps;
    let mut best_speed: ::core::ffi::c_double = (*best)
        .replay
        .final_geometry
        .airspeed_mps;
    let mut trial_shortfall: ::core::ffi::c_double = fmax(0.0f64, target - trial_speed);
    let mut best_shortfall: ::core::ffi::c_double = fmax(0.0f64, target - best_speed);
    if fabs(trial_shortfall - best_shortfall) > 2.0f64 {
        return trial_shortfall < best_shortfall;
    }
    return (*trial).quality_score < (*best).quality_score;
}
unsafe extern "C" fn finish_candidate(
    mut model: *const TerminalModel,
    mut initial: *const TerminalDynamicState,
    mut trial: *mut TaemFixedHacCandidate,
) {
    (*trial).status = TAEM_PLAN_UNQUALIFIED;
    (*trial).reason = b"native HAC and runway alignment reach their exit; Final tail qualification is pending\0"
        as *const u8 as *const ::core::ffi::c_char;
    let mut target_speed: ::core::ffi::c_double = exit_speed_target(model);
    let mut deficit: ::core::ffi::c_double = fabs(
        target_speed - (*trial).replay.final_geometry.airspeed_mps,
    ) / target_speed;
    let mut cross_limit: ::core::ffi::c_double = fmax(
        1200.0f64,
        0.1f64 * (*trial).route.hac.radius_m,
    );
    (*trial).quality_score = (*trial).replay.maximum_cross_track_m / cross_limit
        + (*trial).replay.maximum_course_error_deg / 20.0f64
        + (*trial).replay.maximum_altitude_error_m / 2000.0f64
        + fabs((*trial).replay.energy_closure_residual_j_kg)
            / fmax(
                250.0f64,
                0.02f64
                    * fmax(
                        fabs((*trial).replay.drag_work_j_kg),
                        initial_kinetic(initial),
                    ),
            ) + 4.0f64 * deficit
        + 2.0f64 * (*trial).replay.turn_burden_integral_s
            / fmax((*trial).replay.elapsed_s, 1.0f64)
        + 2.0f64 * fmax(0.0f64, (*trial).replay.maximum_turn_authority_fraction - 0.8f64)
        + 0.5f64 * (*trial).replay.bank_target_reversals as ::core::ffi::c_double
        + 1.25f64 * candidate_circle_penalty(trial);
    if (*trial).quality_score.is_finite() as i32 == 0 {
        (*trial).quality_score = ::core::f32::INFINITY as ::core::ffi::c_double;
    }
}
unsafe extern "C" fn mark_failure(mut trial: *mut TaemFixedHacCandidate) {
    (*trial).status = (if (*trial).replay.status as ::core::ffi::c_uint
        == TERMINAL_SOLVER_SEARCH_EXHAUSTED as ::core::ffi::c_int as ::core::ffi::c_uint
    {
        TAEM_PLAN_SEARCH_EXHAUSTED as ::core::ffi::c_int
    } else {
        TAEM_PLAN_INFEASIBLE as ::core::ffi::c_int
    }) as TaemPlanStatus;
    (*trial).reason = (*trial).replay.reason;
}
#[no_mangle]
pub unsafe extern "C" fn taem_fixed_hac_evaluate_route(
    mut model: *const TerminalModel,
    mut initial: *const TerminalDynamicState,
    mut route: *const TaemRoute,
    mut runway_end: ::core::ffi::c_int,
    mut dt: ::core::ffi::c_double,
    mut maximum_elapsed: ::core::ffi::c_double,
) -> TaemFixedHacCandidate {
    let mut candidate: TaemFixedHacCandidate = TaemFixedHacCandidate {
        status: TAEM_PLAN_INFEASIBLE,
        side: 0.,
        runway_end: 0,
        route_built: false,
        final_tail_qualified: false,
        required_lateral_accel_mps2: 0.,
        available_lateral_accel_mps2: 0.,
        quality_score: 0.,
        route: TaemRoute {
            valid: false,
            count: 0,
            lead_count: 0,
            arc_count: 0,
            rollout_count: 0,
            length_m: 0.,
            side: 0.,
            hac: TaemFixedHacGeometry {
                entry: TaemPoint2 { x: 0., y: 0. },
                center: TaemPoint2 { x: 0., y: 0. },
                exit: TaemPoint2 { x: 0., y: 0. },
                radius_m: 0.,
                side: 0.,
                capture_course_deg: 0.,
                arc_sweep_rad: 0.,
                arc_length_m: 0.,
                final_length_m: 0.,
                total_length_m: 0.,
            },
            p0_along_m: 0.,
            p0_cross_m: 0.,
            p1_along_m: 0.,
            p1_cross_m: 0.,
            p2_along_m: 0.,
            p2_cross_m: 0.,
            p3_along_m: 0.,
            p3_cross_m: 0.,
            capture_control: [TaemPoint2 { x: 0., y: 0. }; 6],
            spiral_exit_radius_m: 0.,
            spiral_coefficient_m: 0.,
            spiral_exit_heading_rad: 0.,
            spiral_arc_lut: [0.; 129],
            unload_length_m: 0.,
            settle_length_m: 0.,
            lead_length_m: 0.,
            lead_arc_fraction_lut: [0.; 129],
            rollout_length_m: 0.,
            alignment_along_m: 0.,
            initial_course_deg: 0.,
            runway_heading_deg: 0.,
            profile_start_altitude_m: 0.,
            profile_start_fpa_deg: 0.,
            profile_final_altitude_m: 0.,
            profile_final_slope_deg: 0.,
            profile_total_length_m: 0.,
            profile_midpoint_offset_m: 0.,
            profile_local_offset_m: 0.,
            profile_local_start_fraction: 0.,
            profile_local_peak_fraction: 0.,
            profile_local_end_fraction: 0.,
            profile_initial_sag_m: 0.,
            profile_initial_sag_length_m: 0.,
            profile_tabulated: false,
            profile_generation_aoa_deg: 0.,
            profile_altitude_lut: [0.; 257],
            profile_fpa_lut: [0.; 257],
            profile_speed_tabulated: false,
            profile_speed_lut: [0.; 257],
        },
        replay: TerminalSolverResult {
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
        },
        reason: ::core::ptr::null::<::core::ffi::c_char>(),
    };
    memset(
        &raw mut candidate as *mut ::core::ffi::c_void,
        0 as ::core::ffi::c_int,
        ::core::mem::size_of::<TaemFixedHacCandidate>() as size_t,
    );
    candidate.status = TAEM_PLAN_INFEASIBLE;
    candidate.reason = b"provisional route qualification failed\0" as *const u8
        as *const ::core::ffi::c_char;
    candidate.runway_end = runway_end;
    if model.is_null() || initial.is_null() || route.is_null() || !(*route).valid
        || !(dt > 0.0f64) || !(maximum_elapsed > 0.0f64)
    {
        return candidate;
    }
    candidate.side = (*route).side;
    candidate.route = *route;
    candidate.route_built = true_0 != 0;
    let mut profile: TerminalProfileResult = terminal_solver_generate_profile(
        model,
        initial,
        &raw mut candidate.route,
        dt,
        maximum_elapsed,
    );
    if !profile.valid {
        candidate.reason = if !profile.reason.is_null() {
            profile.reason
        } else {
            b"provisional route has no feasible native vertical profile\0" as *const u8
                as *const ::core::ffi::c_char
        };
        return candidate;
    }
    candidate.replay = terminal_solver_replay(
        model,
        initial,
        &raw mut candidate.route,
        dt,
        maximum_elapsed,
    );
    if replay_reaches_aligned_exit(&raw mut candidate.replay) {
        finish_candidate(model, initial, &raw mut candidate);
        return candidate;
    }
    mark_failure(&raw mut candidate);
    return candidate;
}
unsafe extern "C" fn evaluate_side(
    mut model: *const TerminalModel,
    mut initial: *const TerminalDynamicState,
    mut geometry: *const TaemGeometryState,
    mut hac_radius: ::core::ffi::c_double,
    mut side: ::core::ffi::c_double,
    mut spacing: ::core::ffi::c_double,
    mut dt: ::core::ffi::c_double,
    mut maximum_elapsed: ::core::ffi::c_double,
    mut stations: *mut JoinStation,
) -> TaemFixedHacCandidate {
    let mut candidate: TaemFixedHacCandidate = TaemFixedHacCandidate {
        status: TAEM_PLAN_INFEASIBLE,
        side: 0.,
        runway_end: 0,
        route_built: false,
        final_tail_qualified: false,
        required_lateral_accel_mps2: 0.,
        available_lateral_accel_mps2: 0.,
        quality_score: 0.,
        route: TaemRoute {
            valid: false,
            count: 0,
            lead_count: 0,
            arc_count: 0,
            rollout_count: 0,
            length_m: 0.,
            side: 0.,
            hac: TaemFixedHacGeometry {
                entry: TaemPoint2 { x: 0., y: 0. },
                center: TaemPoint2 { x: 0., y: 0. },
                exit: TaemPoint2 { x: 0., y: 0. },
                radius_m: 0.,
                side: 0.,
                capture_course_deg: 0.,
                arc_sweep_rad: 0.,
                arc_length_m: 0.,
                final_length_m: 0.,
                total_length_m: 0.,
            },
            p0_along_m: 0.,
            p0_cross_m: 0.,
            p1_along_m: 0.,
            p1_cross_m: 0.,
            p2_along_m: 0.,
            p2_cross_m: 0.,
            p3_along_m: 0.,
            p3_cross_m: 0.,
            capture_control: [TaemPoint2 { x: 0., y: 0. }; 6],
            spiral_exit_radius_m: 0.,
            spiral_coefficient_m: 0.,
            spiral_exit_heading_rad: 0.,
            spiral_arc_lut: [0.; 129],
            unload_length_m: 0.,
            settle_length_m: 0.,
            lead_length_m: 0.,
            lead_arc_fraction_lut: [0.; 129],
            rollout_length_m: 0.,
            alignment_along_m: 0.,
            initial_course_deg: 0.,
            runway_heading_deg: 0.,
            profile_start_altitude_m: 0.,
            profile_start_fpa_deg: 0.,
            profile_final_altitude_m: 0.,
            profile_final_slope_deg: 0.,
            profile_total_length_m: 0.,
            profile_midpoint_offset_m: 0.,
            profile_local_offset_m: 0.,
            profile_local_start_fraction: 0.,
            profile_local_peak_fraction: 0.,
            profile_local_end_fraction: 0.,
            profile_initial_sag_m: 0.,
            profile_initial_sag_length_m: 0.,
            profile_tabulated: false,
            profile_generation_aoa_deg: 0.,
            profile_altitude_lut: [0.; 257],
            profile_fpa_lut: [0.; 257],
            profile_speed_tabulated: false,
            profile_speed_lut: [0.; 257],
        },
        replay: TerminalSolverResult {
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
        },
        reason: ::core::ptr::null::<::core::ffi::c_char>(),
    };
    memset(
        &raw mut candidate as *mut ::core::ffi::c_void,
        0 as ::core::ffi::c_int,
        ::core::mem::size_of::<TaemFixedHacCandidate>() as size_t,
    );
    candidate.side = side;
    candidate.status = TAEM_PLAN_INFEASIBLE;
    candidate.reason = b"fixed-HAC candidate is infeasible\0" as *const u8
        as *const ::core::ffi::c_char;
    let mut reachability: TaemReachability = TaemReachability {
        valid: false,
        lateral_authority_ok: false,
        energy_qualified: false,
        required_lateral_accel_mps2: 0.,
        available_lateral_accel_mps2: 0.,
        lateral_margin_mps2: 0.,
        minimum_turn_radius_m: 0.,
        roll_settling_time_s: 0.,
        roll_settling_distance_m: 0.,
    };
    memset(
        &raw mut reachability as *mut ::core::ffi::c_void,
        0 as ::core::ffi::c_int,
        ::core::mem::size_of::<TaemReachability>() as size_t,
    );
    if !taem_fixed_hac_turn_reachability(
        model,
        initial,
        geometry,
        hac_radius,
        &raw mut reachability,
    ) || !reachability.valid || !(reachability.available_lateral_accel_mps2 > 0.0f64)
    {
        candidate.required_lateral_accel_mps2 = reachability.required_lateral_accel_mps2;
        candidate.available_lateral_accel_mps2 = reachability
            .available_lateral_accel_mps2;
        candidate.reason = b"no measurable lateral authority for the finite lead\0"
            as *const u8 as *const ::core::ffi::c_char;
        return candidate;
    }
    candidate.required_lateral_accel_mps2 = reachability.required_lateral_accel_mps2;
    candidate.available_lateral_accel_mps2 = reachability.available_lateral_accel_mps2;
    let mut live_curvature: ::core::ffi::c_double = 0.95f64
        * reachability.available_lateral_accel_mps2
        / fmax((*geometry).ground_speed_mps * (*geometry).ground_speed_mps, 1.0f64);
    let mut rho_live: ::core::ffi::c_double = world_atmosphere_sample(
            &raw const (*model).world,
            (*model).site.altitude + (*geometry).altitude_above_runway_m,
        )
        .density_kg_m3;
    let mut best_failure: TaemFixedHacCandidate = candidate;
    let mut have_failure: bool = false_0 != 0;
    let mut count: size_t = 0 as size_t;
    let mut reason: [::core::ffi::c_char; 160] = [0; 160];
    let mut sweep_deg: ::core::ffi::c_double = TAEM_JOIN_SWEEP_MAX_DEG;
    while sweep_deg >= TAEM_JOIN_SWEEP_MIN_DEG {
        let mut sweep_rad: ::core::ffi::c_double = sweep_deg * 3.14159265358979323846f64
            / 180.0f64;
        let mut seed: TaemRoute = TaemRoute {
            valid: false,
            count: 0,
            lead_count: 0,
            arc_count: 0,
            rollout_count: 0,
            length_m: 0.,
            side: 0.,
            hac: TaemFixedHacGeometry {
                entry: TaemPoint2 { x: 0., y: 0. },
                center: TaemPoint2 { x: 0., y: 0. },
                exit: TaemPoint2 { x: 0., y: 0. },
                radius_m: 0.,
                side: 0.,
                capture_course_deg: 0.,
                arc_sweep_rad: 0.,
                arc_length_m: 0.,
                final_length_m: 0.,
                total_length_m: 0.,
            },
            p0_along_m: 0.,
            p0_cross_m: 0.,
            p1_along_m: 0.,
            p1_cross_m: 0.,
            p2_along_m: 0.,
            p2_cross_m: 0.,
            p3_along_m: 0.,
            p3_cross_m: 0.,
            capture_control: [TaemPoint2 { x: 0., y: 0. }; 6],
            spiral_exit_radius_m: 0.,
            spiral_coefficient_m: 0.,
            spiral_exit_heading_rad: 0.,
            spiral_arc_lut: [0.; 129],
            unload_length_m: 0.,
            settle_length_m: 0.,
            lead_length_m: 0.,
            lead_arc_fraction_lut: [0.; 129],
            rollout_length_m: 0.,
            alignment_along_m: 0.,
            initial_course_deg: 0.,
            runway_heading_deg: 0.,
            profile_start_altitude_m: 0.,
            profile_start_fpa_deg: 0.,
            profile_final_altitude_m: 0.,
            profile_final_slope_deg: 0.,
            profile_total_length_m: 0.,
            profile_midpoint_offset_m: 0.,
            profile_local_offset_m: 0.,
            profile_local_start_fraction: 0.,
            profile_local_peak_fraction: 0.,
            profile_local_end_fraction: 0.,
            profile_initial_sag_m: 0.,
            profile_initial_sag_length_m: 0.,
            profile_tabulated: false,
            profile_generation_aoa_deg: 0.,
            profile_altitude_lut: [0.; 257],
            profile_fpa_lut: [0.; 257],
            profile_speed_tabulated: false,
            profile_speed_lut: [0.; 257],
        };
        if taem_route_build_hac(
            model,
            geometry,
            hac_radius,
            side,
            sweep_rad,
            spacing,
            live_curvature,
            &raw mut seed,
            &raw mut reason as *mut ::core::ffi::c_char,
            ::core::mem::size_of::<[::core::ffi::c_char; 160]>() as size_t,
        ) {
            let mut terminal_height: ::core::ffi::c_double = seed
                .profile_final_altitude_m - (*model).site.altitude;
            let mut hac_height: ::core::ffi::c_double = terminal_height
                + (seed.hac.arc_length_m + seed.rollout_length_m)
                    * tan(
                        fmin((*model).guidance.taem_glide_slope, 12.0f64)
                            * 3.14159265358979323846f64 / 180.0f64,
                    );
            let mut rho_join: ::core::ffi::c_double = world_atmosphere_sample(
                    &raw const (*model).world,
                    (*model).site.altitude + fmax(0.0f64, hac_height),
                )
                .density_kg_m3;
            let mut density_gain: ::core::ffi::c_double = if rho_live > 0.0f64
                && rho_join > rho_live
            {
                rho_join / rho_live
            } else {
                1.0f64
            };
            let mut sweep_curvature_ceiling: ::core::ffi::c_double = live_curvature
                * density_gain;
            let mut energy_lead_length: ::core::ffi::c_double = ::core::f32::NAN
                as ::core::ffi::c_double;
            let mut variant: ::core::ffi::c_int = 0 as ::core::ffi::c_int;
            while variant < 3 as ::core::ffi::c_int
                && count < TAEM_JOIN_MAX_STATIONS as size_t
            {
                let mut station: *mut JoinStation = stations.offset(count as isize)
                    as *mut JoinStation;
                (*station).sweep_rad = sweep_rad;
                if !(variant == 2 as ::core::ffi::c_int
                    && energy_lead_length.is_finite() as i32 == 0)
                {
                    let mut built: bool = if variant == 0 as ::core::ffi::c_int {
                        (*station).route = seed;
                        true_0
                    } else if variant == 1 as ::core::ffi::c_int {
                        taem_route_build_hac_shortest(
                            model,
                            geometry,
                            hac_radius,
                            side,
                            (*station).sweep_rad,
                            spacing,
                            live_curvature,
                            sweep_curvature_ceiling,
                            &raw mut (*station).route,
                            &raw mut reason as *mut ::core::ffi::c_char,
                            ::core::mem::size_of::<[::core::ffi::c_char; 160]>()
                                as size_t,
                        ) as ::core::ffi::c_int
                    } else {
                        taem_route_build_hac_length(
                            model,
                            geometry,
                            hac_radius,
                            side,
                            (*station).sweep_rad,
                            spacing,
                            live_curvature,
                            sweep_curvature_ceiling,
                            energy_lead_length,
                            &raw mut (*station).route,
                            &raw mut reason as *mut ::core::ffi::c_char,
                            ::core::mem::size_of::<[::core::ffi::c_char; 160]>()
                                as size_t,
                        ) as ::core::ffi::c_int
                    } != 0;
                    if !built {
                        if !have_failure {
                            best_failure.reason = if strcmp(
                                &raw mut reason as *mut ::core::ffi::c_char,
                                b"lead-to-HAC distance is too short\0" as *const u8
                                    as *const ::core::ffi::c_char,
                            ) == 0 as ::core::ffi::c_int
                            {
                                b"live state is too close to every HAC join\0" as *const u8
                                    as *const ::core::ffi::c_char
                            } else {
                                b"finite lead exceeds live curvature authority at every HAC join\0"
                                    as *const u8 as *const ::core::ffi::c_char
                            };
                            have_failure = true_0 != 0;
                        }
                    } else if !(variant > 0 as ::core::ffi::c_int && count > 0 as size_t
                        && fabs(
                            (*stations.offset(count.wrapping_sub(1 as size_t) as isize))
                                .sweep_rad - (*station).sweep_rad,
                        ) < 1e-9f64
                        && fabs(
                            (*stations.offset(count.wrapping_sub(1 as size_t) as isize))
                                .route
                                .length_m - (*station).route.length_m,
                        ) < 200.0f64)
                    {
                        let mut peak: ::core::ffi::c_double = taem_route_lead_peak_curvature(
                            &raw mut (*station).route,
                        );
                        let mut hac_curvature: ::core::ffi::c_double = fabs(
                            taem_route_curvature_at_station(
                                &raw mut (*station).route,
                                (*station).route.lead_length_m,
                            ),
                        );
                        if !(peak.is_finite() as i32 == 0) {
                            let mut profile_length: ::core::ffi::c_double = minimum_vertical_profile_length(
                                model,
                                &raw mut (*station).route,
                            );
                            if !(profile_length.is_finite() as i32 == 0) {
                                if variant == 0 as ::core::ffi::c_int
                                    && profile_length > (*station).route.length_m + 1000.0f64
                                {
                                    energy_lead_length = profile_length
                                        - ((*station).route.length_m
                                            - (*station).route.lead_length_m);
                                }
                                (*station).score = 0.25f64
                                    * fmax(0.0f64, peak - hac_curvature) / hac_curvature
                                    + fabs((*station).route.length_m - profile_length)
                                        / fmax(profile_length, (*station).route.hac.radius_m);
                                count = count.wrapping_add(1);
                            }
                        }
                    }
                }
                variant += 1;
            }
        }
        sweep_deg = if sweep_deg > TAEM_JOIN_SWEEP_MIN_DEG {
            fmax(TAEM_JOIN_SWEEP_MIN_DEG, sweep_deg - TAEM_JOIN_SWEEP_STEP_DEG)
        } else {
            0.0f64
        };
    }
    if count == 0 as size_t {
        return best_failure;
    }
    qsort(
        stations as *mut ::core::ffi::c_void,
        count,
        ::core::mem::size_of::<JoinStation>() as size_t,
        Some(
            compare_station
                as unsafe extern "C" fn(
                    *const ::core::ffi::c_void,
                    *const ::core::ffi::c_void,
                ) -> ::core::ffi::c_int,
        ),
    );
    let mut target_speed: ::core::ffi::c_double = exit_speed_target(model);
    let mut profile_budget: size_t = if count < TAEM_JOIN_MAX_PROFILES as size_t {
        count
    } else {
        TAEM_JOIN_MAX_PROFILES as size_t
    };
    if diagnostics_enabled() as ::core::ffi::c_int != 0 && count > 0 as size_t {
        let mut n: size_t = if count < 8 as size_t { count } else { 8 as size_t };
        let mut i: size_t = 0 as size_t;
        while i < n {
            fprintf(
                stderr,
                b"TAEM geometry: side=%+.0f radius=%.0f sweep=%.1f total=%.0f lead=%.0f arc=%.0f rollout=%.0f peakK=%.3e score=%.3f\n\0"
                    as *const u8 as *const ::core::ffi::c_char,
                side,
                (*stations.offset(i as isize)).route.hac.radius_m,
                (*stations.offset(i as isize)).sweep_rad * 180.0f64
                    / 3.14159265358979323846f64,
                (*stations.offset(i as isize)).route.length_m,
                (*stations.offset(i as isize)).route.lead_length_m,
                (*stations.offset(i as isize)).route.hac.arc_length_m,
                (*stations.offset(i as isize)).route.rollout_length_m,
                taem_route_lead_peak_curvature(
                    &raw mut (*stations.offset(i as isize)).route,
                ),
                (*stations.offset(i as isize)).score,
            );
            i = i.wrapping_add(1);
        }
    }
    qsort(
        stations as *mut ::core::ffi::c_void,
        count,
        ::core::mem::size_of::<JoinStation>() as size_t,
        Some(
            compare_station_length
                as unsafe extern "C" fn(
                    *const ::core::ffi::c_void,
                    *const ::core::ffi::c_void,
                ) -> ::core::ffi::c_int,
        ),
    );
    let mut target_length: ::core::ffi::c_double = estimated_energy_route_length(
        model,
        initial,
        geometry,
    );
    if target_length.is_finite() as i32 == 0 {
        target_length = (*stations.offset(0 as ::core::ffi::c_int as isize))
            .route
            .length_m;
    }
    let mut center: size_t = 0 as size_t;
    let mut center_error: ::core::ffi::c_double = fabs(
        (*stations.offset(0 as ::core::ffi::c_int as isize)).route.length_m
            - target_length,
    );
    let mut i_0: size_t = 1 as size_t;
    while i_0 < count {
        let mut error: ::core::ffi::c_double = fabs(
            (*stations.offset(i_0 as isize)).route.length_m - target_length,
        );
        if error < center_error {
            center = i_0;
            center_error = error;
        }
        i_0 = i_0.wrapping_add(1);
    }
    if diagnostics_enabled() {
        let mut n_0: size_t = if count < 10 as size_t { count } else { 10 as size_t };
        fprintf(
            stderr,
            b"TAEM energy length target: side=%+.0f target=%.0f nearest=%zu/%.0f count=%zu\n\0"
                as *const u8 as *const ::core::ffi::c_char,
            side,
            target_length,
            center,
            (*stations.offset(center as isize)).route.length_m,
            count,
        );
        let mut i_1: size_t = 0 as size_t;
        while i_1 < n_0 {
            fprintf(
                stderr,
                b"TAEM shortest: side=%+.0f idx=%zu/%zu radius=%.0f sweep=%.1f total=%.0f lead=%.0f arc=%.0f rollout=%.0f score=%.3f\n\0"
                    as *const u8 as *const ::core::ffi::c_char,
                side,
                i_1,
                count,
                (*stations.offset(i_1 as isize)).route.hac.radius_m,
                (*stations.offset(i_1 as isize)).sweep_rad * 180.0f64
                    / 3.14159265358979323846f64,
                (*stations.offset(i_1 as isize)).route.length_m,
                (*stations.offset(i_1 as isize)).route.lead_length_m,
                (*stations.offset(i_1 as isize)).route.hac.arc_length_m,
                (*stations.offset(i_1 as isize)).route.rollout_length_m,
                (*stations.offset(i_1 as isize)).score,
            );
            i_1 = i_1.wrapping_add(1);
        }
    }
    let mut tried: *mut ::core::ffi::c_uchar = calloc(count, 1 as size_t)
        as *mut ::core::ffi::c_uchar;
    let mut results: *mut TerminalProfileResult = calloc(
        count,
        ::core::mem::size_of::<TerminalProfileResult>() as size_t,
    ) as *mut TerminalProfileResult;
    if tried.is_null() || results.is_null() {
        free(tried as *mut ::core::ffi::c_void);
        free(results as *mut ::core::ffi::c_void);
        best_failure.reason = b"profile workspace allocation failed\0" as *const u8
            as *const ::core::ffi::c_char;
        return best_failure;
    }
    let mut used: size_t = 0 as size_t;
    let mut anchor_sweeps: size_t = 0 as size_t;
    let mut anchored_sweep: [::core::ffi::c_double; 2] = [
        ::core::f32::NAN as ::core::ffi::c_double,
        ::core::f32::NAN as ::core::ffi::c_double,
    ];
    let mut i_2: size_t = 0 as size_t;
    while i_2 < count && used < profile_budget && anchor_sweeps < 2 as size_t {
        let mut seen: bool = false_0 != 0;
        let mut a: size_t = 0 as size_t;
        while a < anchor_sweeps {
            if fabs(
                (*stations.offset(i_2 as isize)).sweep_rad - anchored_sweep[a as usize],
            ) < 1e-9f64
            {
                seen = true_0 != 0;
            }
            a = a.wrapping_add(1);
        }
        if !seen {
            let fresh0 = anchor_sweeps;
            anchor_sweeps = anchor_sweeps.wrapping_add(1);
            anchored_sweep[fresh0 as usize] = (*stations.offset(i_2 as isize)).sweep_rad;
            if *tried.offset(i_2 as isize) == 0 && used < profile_budget {
                *tried.offset(i_2 as isize) = 1 as ::core::ffi::c_uchar;
                used = used.wrapping_add(1);
                *results.offset(i_2 as isize) = terminal_solver_generate_profile(
                    model,
                    initial,
                    &raw mut (*stations.offset(i_2 as isize)).route,
                    TAEM_PROFILE_DT_S,
                    maximum_elapsed,
                );
                if diagnostics_enabled() {
                    fprintf(
                        stderr,
                        b"TAEM profile: side=%+.0f radius=%.0f join_sweep=%.1f length=%.0f valid=%d aoa=%.2f exitV=%.1f exitFpa=%.1f reason=%s\n\0"
                            as *const u8 as *const ::core::ffi::c_char,
                        side,
                        (*stations.offset(i_2 as isize)).route.hac.radius_m,
                        (*stations.offset(i_2 as isize)).sweep_rad * 180.0f64
                            / 3.14159265358979323846f64,
                        (*stations.offset(i_2 as isize)).route.length_m,
                        if (*results.offset(i_2 as isize)).valid as ::core::ffi::c_int
                            != 0
                        {
                            1 as ::core::ffi::c_int
                        } else {
                            0 as ::core::ffi::c_int
                        },
                        (*results.offset(i_2 as isize)).aoa_deg,
                        (*results.offset(i_2 as isize)).exit_speed_mps,
                        (*results.offset(i_2 as isize)).exit_fpa_deg,
                        (*results.offset(i_2 as isize)).reason,
                    );
                }
            }
            let mut mate: size_t = i_2;
            let mut j: size_t = i_2.wrapping_add(1 as size_t);
            while j < count {
                if fabs(
                    (*stations.offset(j as isize)).sweep_rad
                        - (*stations.offset(i_2 as isize)).sweep_rad,
                ) < 1e-9f64
                    && (*stations.offset(j as isize)).route.lead_length_m
                        > (*stations.offset(mate as isize)).route.lead_length_m
                {
                    mate = j;
                }
                j = j.wrapping_add(1);
            }
            if mate != i_2 {
                if *tried.offset(mate as isize) == 0 && used < profile_budget {
                    *tried.offset(mate as isize) = 1 as ::core::ffi::c_uchar;
                    used = used.wrapping_add(1);
                    *results.offset(mate as isize) = terminal_solver_generate_profile(
                        model,
                        initial,
                        &raw mut (*stations.offset(mate as isize)).route,
                        TAEM_PROFILE_DT_S,
                        maximum_elapsed,
                    );
                    if diagnostics_enabled() {
                        fprintf(
                            stderr,
                            b"TAEM profile: side=%+.0f radius=%.0f join_sweep=%.1f length=%.0f valid=%d aoa=%.2f exitV=%.1f exitFpa=%.1f reason=%s\n\0"
                                as *const u8 as *const ::core::ffi::c_char,
                            side,
                            (*stations.offset(mate as isize)).route.hac.radius_m,
                            (*stations.offset(mate as isize)).sweep_rad * 180.0f64
                                / 3.14159265358979323846f64,
                            (*stations.offset(mate as isize)).route.length_m,
                            if (*results.offset(mate as isize)).valid
                                as ::core::ffi::c_int != 0
                            {
                                1 as ::core::ffi::c_int
                            } else {
                                0 as ::core::ffi::c_int
                            },
                            (*results.offset(mate as isize)).aoa_deg,
                            (*results.offset(mate as isize)).exit_speed_mps,
                            (*results.offset(mate as isize)).exit_fpa_deg,
                            (*results.offset(mate as isize)).reason,
                        );
                    }
                }
            }
        }
        i_2 = i_2.wrapping_add(1);
    }
    if *tried.offset(center as isize) == 0 && used < profile_budget {
        *tried.offset(center as isize) = 1 as ::core::ffi::c_uchar;
        used = used.wrapping_add(1);
        *results.offset(center as isize) = terminal_solver_generate_profile(
            model,
            initial,
            &raw mut (*stations.offset(center as isize)).route,
            TAEM_PROFILE_DT_S,
            maximum_elapsed,
        );
        if diagnostics_enabled() {
            fprintf(
                stderr,
                b"TAEM profile: side=%+.0f radius=%.0f join_sweep=%.1f length=%.0f valid=%d aoa=%.2f exitV=%.1f exitFpa=%.1f reason=%s\n\0"
                    as *const u8 as *const ::core::ffi::c_char,
                side,
                (*stations.offset(center as isize)).route.hac.radius_m,
                (*stations.offset(center as isize)).sweep_rad * 180.0f64
                    / 3.14159265358979323846f64,
                (*stations.offset(center as isize)).route.length_m,
                if (*results.offset(center as isize)).valid as ::core::ffi::c_int != 0 {
                    1 as ::core::ffi::c_int
                } else {
                    0 as ::core::ffi::c_int
                },
                (*results.offset(center as isize)).aoa_deg,
                (*results.offset(center as isize)).exit_speed_mps,
                (*results.offset(center as isize)).exit_fpa_deg,
                (*results.offset(center as isize)).reason,
            );
        }
    }
    let mut left: ::core::ffi::c_long = center as ::core::ffi::c_long
        - 1 as ::core::ffi::c_long;
    let mut right: size_t = center.wrapping_add(1 as size_t);
    while used < profile_budget && (left >= 0 as ::core::ffi::c_long || right < count) {
        let mut take_left: bool = false_0 != 0;
        if left >= 0 as ::core::ffi::c_long && right < count {
            let mut le: ::core::ffi::c_double = fabs(
                (*stations.offset(left as isize)).route.length_m - target_length,
            );
            let mut re: ::core::ffi::c_double = fabs(
                (*stations.offset(right as isize)).route.length_m - target_length,
            );
            take_left = le <= re;
        } else {
            take_left = left >= 0 as ::core::ffi::c_long;
        }
        if take_left {
            if *tried.offset(left as size_t as isize) == 0 && used < profile_budget {
                *tried.offset(left as size_t as isize) = 1 as ::core::ffi::c_uchar;
                used = used.wrapping_add(1);
                *results.offset(left as size_t as isize) = terminal_solver_generate_profile(
                    model,
                    initial,
                    &raw mut (*stations.offset(left as size_t as isize)).route,
                    TAEM_PROFILE_DT_S,
                    maximum_elapsed,
                );
                if diagnostics_enabled() {
                    fprintf(
                        stderr,
                        b"TAEM profile: side=%+.0f radius=%.0f join_sweep=%.1f length=%.0f valid=%d aoa=%.2f exitV=%.1f exitFpa=%.1f reason=%s\n\0"
                            as *const u8 as *const ::core::ffi::c_char,
                        side,
                        (*stations.offset(left as size_t as isize)).route.hac.radius_m,
                        (*stations.offset(left as size_t as isize)).sweep_rad * 180.0f64
                            / 3.14159265358979323846f64,
                        (*stations.offset(left as size_t as isize)).route.length_m,
                        if (*results.offset(left as size_t as isize)).valid
                            as ::core::ffi::c_int != 0
                        {
                            1 as ::core::ffi::c_int
                        } else {
                            0 as ::core::ffi::c_int
                        },
                        (*results.offset(left as size_t as isize)).aoa_deg,
                        (*results.offset(left as size_t as isize)).exit_speed_mps,
                        (*results.offset(left as size_t as isize)).exit_fpa_deg,
                        (*results.offset(left as size_t as isize)).reason,
                    );
                }
            }
            left -= 1;
        } else {
            if *tried.offset(right as isize) == 0 && used < profile_budget {
                *tried.offset(right as isize) = 1 as ::core::ffi::c_uchar;
                used = used.wrapping_add(1);
                *results.offset(right as isize) = terminal_solver_generate_profile(
                    model,
                    initial,
                    &raw mut (*stations.offset(right as isize)).route,
                    TAEM_PROFILE_DT_S,
                    maximum_elapsed,
                );
                if diagnostics_enabled() {
                    fprintf(
                        stderr,
                        b"TAEM profile: side=%+.0f radius=%.0f join_sweep=%.1f length=%.0f valid=%d aoa=%.2f exitV=%.1f exitFpa=%.1f reason=%s\n\0"
                            as *const u8 as *const ::core::ffi::c_char,
                        side,
                        (*stations.offset(right as isize)).route.hac.radius_m,
                        (*stations.offset(right as isize)).sweep_rad * 180.0f64
                            / 3.14159265358979323846f64,
                        (*stations.offset(right as isize)).route.length_m,
                        if (*results.offset(right as isize)).valid as ::core::ffi::c_int
                            != 0
                        {
                            1 as ::core::ffi::c_int
                        } else {
                            0 as ::core::ffi::c_int
                        },
                        (*results.offset(right as isize)).aoa_deg,
                        (*results.offset(right as isize)).exit_speed_mps,
                        (*results.offset(right as isize)).exit_fpa_deg,
                        (*results.offset(right as isize)).reason,
                    );
                }
            }
            right = right.wrapping_add(1);
        }
    }
    let mut refined_valid: bool = false_0 != 0;
    let mut i_3: size_t = 0 as size_t;
    while i_3 < count && !refined_valid {
        if !(*tried.offset(i_3 as isize) == 0
            || (*results.offset(i_3 as isize)).valid as ::core::ffi::c_int != 0)
        {
            let mut i_short: bool = !(*results.offset(i_3 as isize)).reason.is_null()
                && (!strstr(
                        (*results.offset(i_3 as isize)).reason,
                        b"too short\0" as *const u8 as *const ::core::ffi::c_char,
                    )
                    .is_null()
                    || !strstr(
                            (*results.offset(i_3 as isize)).reason,
                            b"retains excess energy\0" as *const u8
                                as *const ::core::ffi::c_char,
                        )
                        .is_null());
            let mut i_long: bool = !(*results.offset(i_3 as isize)).reason.is_null()
                && (!strstr(
                        (*results.offset(i_3 as isize)).reason,
                        b"too long\0" as *const u8 as *const ::core::ffi::c_char,
                    )
                    .is_null()
                    || strcmp(
                        (*results.offset(i_3 as isize)).reason,
                        b"ground before route end\0" as *const u8
                            as *const ::core::ffi::c_char,
                    ) == 0 as ::core::ffi::c_int
                    || strcmp(
                        (*results.offset(i_3 as isize)).reason,
                        b"max elapsed before route end\0" as *const u8
                            as *const ::core::ffi::c_char,
                    ) == 0 as ::core::ffi::c_int
                    || !strstr(
                            (*results.offset(i_3 as isize)).reason,
                            b"too slow for Final handoff\0" as *const u8
                                as *const ::core::ffi::c_char,
                        )
                        .is_null());
            if !(!i_short && !i_long) {
                let mut j_0: size_t = i_3.wrapping_add(1 as size_t);
                while j_0 < count && !refined_valid {
                    if !(*tried.offset(j_0 as isize) == 0
                        || (*results.offset(j_0 as isize)).valid as ::core::ffi::c_int
                            != 0)
                    {
                        let mut j_short: bool = !(*results.offset(j_0 as isize))
                            .reason
                            .is_null()
                            && (!strstr(
                                    (*results.offset(j_0 as isize)).reason,
                                    b"too short\0" as *const u8 as *const ::core::ffi::c_char,
                                )
                                .is_null()
                                || !strstr(
                                        (*results.offset(j_0 as isize)).reason,
                                        b"retains excess energy\0" as *const u8
                                            as *const ::core::ffi::c_char,
                                    )
                                    .is_null());
                        let mut j_long: bool = !(*results.offset(j_0 as isize))
                            .reason
                            .is_null()
                            && (!strstr(
                                    (*results.offset(j_0 as isize)).reason,
                                    b"too long\0" as *const u8 as *const ::core::ffi::c_char,
                                )
                                .is_null()
                                || strcmp(
                                    (*results.offset(j_0 as isize)).reason,
                                    b"ground before route end\0" as *const u8
                                        as *const ::core::ffi::c_char,
                                ) == 0 as ::core::ffi::c_int
                                || strcmp(
                                    (*results.offset(j_0 as isize)).reason,
                                    b"max elapsed before route end\0" as *const u8
                                        as *const ::core::ffi::c_char,
                                ) == 0 as ::core::ffi::c_int
                                || !strstr(
                                        (*results.offset(j_0 as isize)).reason,
                                        b"too slow for Final handoff\0" as *const u8
                                            as *const ::core::ffi::c_char,
                                    )
                                    .is_null());
                        if i_short as ::core::ffi::c_int != 0
                            && j_long as ::core::ffi::c_int != 0
                            || i_long as ::core::ffi::c_int != 0
                                && j_short as ::core::ffi::c_int != 0
                        {
                            let mut a_0: *mut JoinStation = stations.offset(i_3 as isize)
                                as *mut JoinStation;
                            let mut b: *mut JoinStation = stations.offset(j_0 as isize)
                                as *mut JoinStation;
                            if !(fabs((*a_0).sweep_rad - (*b).sweep_rad) > 1e-9f64
                                || fabs((*a_0).route.hac.radius_m - (*b).route.hac.radius_m)
                                    > 1e-6f64)
                            {
                                let mut short_lead: ::core::ffi::c_double = (*if i_short
                                    as ::core::ffi::c_int != 0
                                {
                                    a_0
                                } else {
                                    b
                                })
                                    .route
                                    .lead_length_m;
                                let mut long_lead: ::core::ffi::c_double = (*if i_long
                                    as ::core::ffi::c_int != 0
                                {
                                    a_0
                                } else {
                                    b
                                })
                                    .route
                                    .lead_length_m;
                                if short_lead > long_lead {
                                    let mut tmp: ::core::ffi::c_double = short_lead;
                                    short_lead = long_lead;
                                    long_lead = tmp;
                                }
                                if long_lead - short_lead > 5.0f64 {
                                    let mut profile_short_route: *const TaemRoute = if i_short
                                        as ::core::ffi::c_int != 0
                                    {
                                        &raw mut (*a_0).route
                                    } else {
                                        &raw mut (*b).route
                                    };
                                    let mut profile_long_route: *const TaemRoute = if i_long
                                        as ::core::ffi::c_int != 0
                                    {
                                        &raw mut (*a_0).route
                                    } else {
                                        &raw mut (*b).route
                                    };
                                    let mut terminal_height_0: ::core::ffi::c_double = (*a_0)
                                        .route
                                        .profile_final_altitude_m - (*model).site.altitude;
                                    let mut hac_height_0: ::core::ffi::c_double = terminal_height_0
                                        + ((*a_0).route.hac.arc_length_m
                                            + (*a_0).route.rollout_length_m)
                                            * tan(
                                                fmin((*model).guidance.taem_glide_slope, 12.0f64)
                                                    * 3.14159265358979323846f64 / 180.0f64,
                                            );
                                    let mut rho_join_0: ::core::ffi::c_double = world_atmosphere_sample(
                                            &raw const (*model).world,
                                            (*model).site.altitude + fmax(0.0f64, hac_height_0),
                                        )
                                        .density_kg_m3;
                                    let mut density_gain_0: ::core::ffi::c_double = if rho_live
                                        > 0.0f64 && rho_join_0 > rho_live
                                    {
                                        rho_join_0 / rho_live
                                    } else {
                                        1.0f64
                                    };
                                    let mut curvature_ceiling: ::core::ffi::c_double = live_curvature
                                        * density_gain_0;
                                    let mut refine: ::core::ffi::c_int = 0
                                        as ::core::ffi::c_int;
                                    while refine < 5 as ::core::ffi::c_int
                                        && long_lead - short_lead > 2.0f64
                                    {
                                        let mut lead: ::core::ffi::c_double = 0.5f64
                                            * (short_lead + long_lead);
                                        let mut candidate_station: JoinStation = JoinStation {
                                            sweep_rad: 0 as ::core::ffi::c_int as ::core::ffi::c_double,
                                            score: 0.,
                                            route: TaemRoute {
                                                valid: false,
                                                count: 0,
                                                lead_count: 0,
                                                arc_count: 0,
                                                rollout_count: 0,
                                                length_m: 0.,
                                                side: 0.,
                                                hac: TaemFixedHacGeometry {
                                                    entry: TaemPoint2 { x: 0., y: 0. },
                                                    center: TaemPoint2 { x: 0., y: 0. },
                                                    exit: TaemPoint2 { x: 0., y: 0. },
                                                    radius_m: 0.,
                                                    side: 0.,
                                                    capture_course_deg: 0.,
                                                    arc_sweep_rad: 0.,
                                                    arc_length_m: 0.,
                                                    final_length_m: 0.,
                                                    total_length_m: 0.,
                                                },
                                                p0_along_m: 0.,
                                                p0_cross_m: 0.,
                                                p1_along_m: 0.,
                                                p1_cross_m: 0.,
                                                p2_along_m: 0.,
                                                p2_cross_m: 0.,
                                                p3_along_m: 0.,
                                                p3_cross_m: 0.,
                                                capture_control: [TaemPoint2 { x: 0., y: 0. }; 6],
                                                spiral_exit_radius_m: 0.,
                                                spiral_coefficient_m: 0.,
                                                spiral_exit_heading_rad: 0.,
                                                spiral_arc_lut: [0.; 129],
                                                unload_length_m: 0.,
                                                settle_length_m: 0.,
                                                lead_length_m: 0.,
                                                lead_arc_fraction_lut: [0.; 129],
                                                rollout_length_m: 0.,
                                                alignment_along_m: 0.,
                                                initial_course_deg: 0.,
                                                runway_heading_deg: 0.,
                                                profile_start_altitude_m: 0.,
                                                profile_start_fpa_deg: 0.,
                                                profile_final_altitude_m: 0.,
                                                profile_final_slope_deg: 0.,
                                                profile_total_length_m: 0.,
                                                profile_midpoint_offset_m: 0.,
                                                profile_local_offset_m: 0.,
                                                profile_local_start_fraction: 0.,
                                                profile_local_peak_fraction: 0.,
                                                profile_local_end_fraction: 0.,
                                                profile_initial_sag_m: 0.,
                                                profile_initial_sag_length_m: 0.,
                                                profile_tabulated: false,
                                                profile_generation_aoa_deg: 0.,
                                                profile_altitude_lut: [0.; 257],
                                                profile_fpa_lut: [0.; 257],
                                                profile_speed_tabulated: false,
                                                profile_speed_lut: [0.; 257],
                                            },
                                        };
                                        candidate_station.sweep_rad = (*a_0).sweep_rad;
                                        let mut refine_reason: [::core::ffi::c_char; 160] = [
                                            0 as ::core::ffi::c_int as ::core::ffi::c_char,
                                            0,
                                            0,
                                            0,
                                            0,
                                            0,
                                            0,
                                            0,
                                            0,
                                            0,
                                            0,
                                            0,
                                            0,
                                            0,
                                            0,
                                            0,
                                            0,
                                            0,
                                            0,
                                            0,
                                            0,
                                            0,
                                            0,
                                            0,
                                            0,
                                            0,
                                            0,
                                            0,
                                            0,
                                            0,
                                            0,
                                            0,
                                            0,
                                            0,
                                            0,
                                            0,
                                            0,
                                            0,
                                            0,
                                            0,
                                            0,
                                            0,
                                            0,
                                            0,
                                            0,
                                            0,
                                            0,
                                            0,
                                            0,
                                            0,
                                            0,
                                            0,
                                            0,
                                            0,
                                            0,
                                            0,
                                            0,
                                            0,
                                            0,
                                            0,
                                            0,
                                            0,
                                            0,
                                            0,
                                            0,
                                            0,
                                            0,
                                            0,
                                            0,
                                            0,
                                            0,
                                            0,
                                            0,
                                            0,
                                            0,
                                            0,
                                            0,
                                            0,
                                            0,
                                            0,
                                            0,
                                            0,
                                            0,
                                            0,
                                            0,
                                            0,
                                            0,
                                            0,
                                            0,
                                            0,
                                            0,
                                            0,
                                            0,
                                            0,
                                            0,
                                            0,
                                            0,
                                            0,
                                            0,
                                            0,
                                            0,
                                            0,
                                            0,
                                            0,
                                            0,
                                            0,
                                            0,
                                            0,
                                            0,
                                            0,
                                            0,
                                            0,
                                            0,
                                            0,
                                            0,
                                            0,
                                            0,
                                            0,
                                            0,
                                            0,
                                            0,
                                            0,
                                            0,
                                            0,
                                            0,
                                            0,
                                            0,
                                            0,
                                            0,
                                            0,
                                            0,
                                            0,
                                            0,
                                            0,
                                            0,
                                            0,
                                            0,
                                            0,
                                            0,
                                            0,
                                            0,
                                            0,
                                            0,
                                            0,
                                            0,
                                            0,
                                            0,
                                            0,
                                            0,
                                            0,
                                            0,
                                            0,
                                            0,
                                            0,
                                            0,
                                            0,
                                            0,
                                            0,
                                            0,
                                            0,
                                        ];
                                        if !taem_route_blend_hac_length(
                                            model,
                                            geometry,
                                            profile_short_route,
                                            profile_long_route,
                                            spacing,
                                            live_curvature,
                                            curvature_ceiling,
                                            lead,
                                            &raw mut candidate_station.route,
                                            &raw mut refine_reason as *mut ::core::ffi::c_char,
                                            ::core::mem::size_of::<[::core::ffi::c_char; 160]>()
                                                as size_t,
                                        ) {
                                            break;
                                        }
                                        let mut pr: TerminalProfileResult = terminal_solver_generate_profile(
                                            model,
                                            initial,
                                            &raw mut candidate_station.route,
                                            TAEM_PROFILE_DT_S,
                                            maximum_elapsed,
                                        );
                                        if diagnostics_enabled() {
                                            fprintf(
                                                stderr,
                                                b"TAEM profile refine: side=%+.0f radius=%.0f sweep=%.1f lead=%.1f total=%.1f valid=%d exitV=%.1f exitFpa=%.1f reason=%s\n\0"
                                                    as *const u8 as *const ::core::ffi::c_char,
                                                side,
                                                candidate_station.route.hac.radius_m,
                                                candidate_station.sweep_rad * 180.0f64
                                                    / 3.14159265358979323846f64,
                                                candidate_station.route.lead_length_m,
                                                candidate_station.route.length_m,
                                                if pr.valid as ::core::ffi::c_int != 0 {
                                                    1 as ::core::ffi::c_int
                                                } else {
                                                    0 as ::core::ffi::c_int
                                                },
                                                pr.exit_speed_mps,
                                                pr.exit_fpa_deg,
                                                if !pr.reason.is_null() {
                                                    pr.reason
                                                } else {
                                                    b"none\0" as *const u8 as *const ::core::ffi::c_char
                                                },
                                            );
                                        }
                                        if pr.valid {
                                            candidate_station.score = 0.5f64
                                                * ((*a_0).score + (*b).score);
                                            *stations.offset(i_3 as isize) = candidate_station;
                                            *results.offset(i_3 as isize) = pr;
                                            *tried.offset(i_3 as isize) = 1 as ::core::ffi::c_uchar;
                                            refined_valid = true_0 != 0;
                                            break;
                                        } else {
                                            if !pr.reason.is_null()
                                                && (!strstr(
                                                        pr.reason,
                                                        b"too short\0" as *const u8 as *const ::core::ffi::c_char,
                                                    )
                                                    .is_null()
                                                    || !strstr(
                                                            pr.reason,
                                                            b"retains excess energy\0" as *const u8
                                                                as *const ::core::ffi::c_char,
                                                        )
                                                        .is_null())
                                            {
                                                short_lead = candidate_station.route.lead_length_m;
                                            } else {
                                                if !(!pr.reason.is_null()
                                                    && (!strstr(
                                                            pr.reason,
                                                            b"too long\0" as *const u8 as *const ::core::ffi::c_char,
                                                        )
                                                        .is_null()
                                                        || strcmp(
                                                            pr.reason,
                                                            b"ground before route end\0" as *const u8
                                                                as *const ::core::ffi::c_char,
                                                        ) == 0 as ::core::ffi::c_int
                                                        || strcmp(
                                                            pr.reason,
                                                            b"max elapsed before route end\0" as *const u8
                                                                as *const ::core::ffi::c_char,
                                                        ) == 0 as ::core::ffi::c_int
                                                        || !strstr(
                                                                pr.reason,
                                                                b"too slow for Final handoff\0" as *const u8
                                                                    as *const ::core::ffi::c_char,
                                                            )
                                                            .is_null()))
                                                {
                                                    break;
                                                }
                                                long_lead = candidate_station.route.lead_length_m;
                                            }
                                            refine += 1;
                                        }
                                    }
                                }
                            }
                        }
                    }
                    j_0 = j_0.wrapping_add(1);
                }
            }
        }
        i_3 = i_3.wrapping_add(1);
    }
    let mut profiled: size_t = 0 as size_t;
    let mut i_4: size_t = 0 as size_t;
    while i_4 < count {
        if !(*tried.offset(i_4 as isize) == 0 || !(*results.offset(i_4 as isize)).valid)
        {
            let mut authority_ratio: ::core::ffi::c_double = profile_lateral_authority_ratio(
                model,
                &raw mut (*stations.offset(i_4 as isize)).route,
                live_curvature,
                rho_live,
            );
            if !(authority_ratio.is_finite() as i32 == 0) {
                if diagnostics_enabled() {
                    fprintf(
                        stderr,
                        b"TAEM authority: side=%+.0f sweep=%.1f length=%.0f ratio=%.3f leadPeakK=%.3e liveK=%.3e\n\0"
                            as *const u8 as *const ::core::ffi::c_char,
                        side,
                        (*stations.offset(i_4 as isize)).sweep_rad * 180.0f64
                            / 3.14159265358979323846f64,
                        (*stations.offset(i_4 as isize)).route.length_m,
                        authority_ratio,
                        taem_route_lead_peak_curvature(
                            &raw mut (*stations.offset(i_4 as isize)).route,
                        ),
                        live_curvature,
                    );
                }
                (*stations.offset(i_4 as isize)).score
                    += fabs(
                        target_speed - (*results.offset(i_4 as isize)).exit_speed_mps,
                    ) / target_speed * 10.0f64
                        + 20.0f64 * fmax(0.0f64, authority_ratio - 0.8f64);
                if i_4 != profiled {
                    let mut swap: JoinStation = *stations.offset(profiled as isize);
                    *stations.offset(profiled as isize) = *stations.offset(i_4 as isize);
                    *stations.offset(i_4 as isize) = swap;
                }
                profiled = profiled.wrapping_add(1);
            }
        }
        i_4 = i_4.wrapping_add(1);
    }
    if profiled == 0 as size_t {
        let mut i_5: size_t = 0 as size_t;
        while i_5 < count {
            if !(*tried.offset(i_5 as isize) == 0) {
                let mut reason_0: *const ::core::ffi::c_char = if !(*results
                    .offset(i_5 as isize))
                    .reason
                    .is_null()
                {
                    (*results.offset(i_5 as isize)).reason
                } else {
                    b"unknown\0" as *const u8 as *const ::core::ffi::c_char
                };
                let mut seen_0: bool = false_0 != 0;
                let mut j_1: size_t = 0 as size_t;
                while j_1 < i_5 {
                    if *tried.offset(j_1 as isize) as ::core::ffi::c_int != 0
                        && !(*results.offset(j_1 as isize)).reason.is_null()
                        && strcmp(reason_0, (*results.offset(j_1 as isize)).reason)
                            == 0 as ::core::ffi::c_int
                    {
                        seen_0 = true_0 != 0;
                    }
                    j_1 = j_1.wrapping_add(1);
                }
                if !seen_0 {
                    let mut failures: size_t = 0 as size_t;
                    let mut j_2: size_t = i_5;
                    while j_2 < count {
                        if *tried.offset(j_2 as isize) as ::core::ffi::c_int != 0
                            && !(*results.offset(j_2 as isize)).reason.is_null()
                            && strcmp(reason_0, (*results.offset(j_2 as isize)).reason)
                                == 0 as ::core::ffi::c_int
                        {
                            failures = failures.wrapping_add(1);
                        }
                        j_2 = j_2.wrapping_add(1);
                    }
                    fprintf(
                        stderr,
                        b"TAEM profile rejection: side=%+.0f count=%zu/%zu reason=%s\n\0"
                            as *const u8 as *const ::core::ffi::c_char,
                        side,
                        failures,
                        used,
                        reason_0,
                    );
                }
            }
            i_5 = i_5.wrapping_add(1);
        }
    }
    free(tried as *mut ::core::ffi::c_void);
    free(results as *mut ::core::ffi::c_void);
    if profiled == 0 as size_t {
        best_failure.reason = b"no HAC join has an energy-feasible native vertical profile\0"
            as *const u8 as *const ::core::ffi::c_char;
        return best_failure;
    }
    qsort(
        stations as *mut ::core::ffi::c_void,
        profiled,
        ::core::mem::size_of::<JoinStation>() as size_t,
        Some(
            compare_station
                as unsafe extern "C" fn(
                    *const ::core::ffi::c_void,
                    *const ::core::ffi::c_void,
                ) -> ::core::ffi::c_int,
        ),
    );
    count = profiled;
    have_failure = false_0 != 0;
    let mut replays: size_t = if count < TAEM_JOIN_MAX_REPLAYS as size_t {
        count
    } else {
        TAEM_JOIN_MAX_REPLAYS as size_t
    };
    let mut i_6: size_t = 0 as size_t;
    while i_6 < replays {
        let mut trial: TaemFixedHacCandidate = candidate;
        trial.route = (*stations.offset(i_6 as isize)).route;
        trial.route_built = true_0 != 0;
        trial.replay = terminal_solver_replay(
            model,
            initial,
            &raw mut trial.route,
            dt,
            maximum_elapsed,
        );
        if diagnostics_enabled() {
            fprintf(
                stderr,
                b"TAEM candidate: side=%+.0f join_sweep=%.1f lead=%.0f arc=%.0f leadPeakK=%.3e liveK=%.3e status=%d elapsed=%.2f index=%zu/%zu reason=%s cross=%.1f course=%.1f altErr=%.1f exitV=%.1f latShort=%.2f turnMean=%.3f turnPeak=%.3f aoaMax=%.1f reversals=%u\n\0"
                    as *const u8 as *const ::core::ffi::c_char,
                side,
                (*stations.offset(i_6 as isize)).sweep_rad * 180.0f64
                    / 3.14159265358979323846f64,
                trial.route.lead_length_m,
                trial.route.hac.arc_length_m,
                taem_route_lead_peak_curvature(&raw mut trial.route),
                live_curvature,
                trial.replay.status as ::core::ffi::c_int,
                trial.replay.elapsed_s,
                trial.replay.failure_route_index,
                trial.route.count,
                if !trial.replay.reason.is_null() {
                    trial.replay.reason
                } else {
                    b"none\0" as *const u8 as *const ::core::ffi::c_char
                },
                trial.replay.maximum_cross_track_m,
                trial.replay.maximum_course_error_deg,
                trial.replay.maximum_altitude_error_m,
                trial.replay.final_geometry.airspeed_mps,
                trial.replay.maximum_lateral_authority_shortfall_mps2,
                trial.replay.turn_burden_integral_s
                    / fmax(trial.replay.elapsed_s, 1.0f64),
                trial.replay.maximum_turn_authority_fraction,
                trial.replay.maximum_commanded_aoa_deg,
                trial.replay.bank_target_reversals,
            );
        }
        if replay_reaches_aligned_exit(&raw mut trial.replay) {
            fprintf(
                stderr,
                b"MM305_REPLAY_EXIT side=%+.0f radius=%.0f sweep=%.1f lead=%.0f refErr=%.2f runwayCourseErr=%.2f along=%.1f cross=%.1f h=%.1f airV=%.1f fpa=%.2f bank=%.2f aoa=%.2f energy=%.1f reason=%s\n\0"
                    as *const u8 as *const ::core::ffi::c_char,
                trial.route.side,
                trial.route.hac.radius_m,
                trial.route.hac.arc_sweep_rad * 180.0f64 / 3.14159265358979323846f64,
                trial.route.lead_length_m,
                trial.replay.final_route_course_error_deg,
                trial.replay.final_geometry.heading_error_deg,
                trial.replay.final_geometry.runway_along_m,
                trial.replay.final_geometry.runway_cross_m,
                trial.replay.final_geometry.altitude_above_runway_m,
                trial.replay.final_geometry.airspeed_mps,
                trial.replay.final_geometry.flight_path_angle_deg,
                trial.replay.final_state.attitude.bank_rad * 180.0f64
                    / 3.14159265358979323846f64,
                trial.replay.final_state.attitude.aoa_rad * 180.0f64
                    / 3.14159265358979323846f64,
                trial.replay.energy_end_j_kg,
                if !trial.replay.reason.is_null() {
                    trial.replay.reason
                } else {
                    b"none\0" as *const u8 as *const ::core::ffi::c_char
                },
            );
            finish_candidate(model, initial, &raw mut trial);
            if diagnostics_enabled() {
                fprintf(
                    stderr,
                    b"TAEM rank: side=%+.0f radius=%.0f sweep=%.1f exitV=%.1f quality=%.3f reversals=%u\n\0"
                        as *const u8 as *const ::core::ffi::c_char,
                    side,
                    trial.route.hac.radius_m,
                    trial.route.hac.arc_sweep_rad * 57.29577951308232f64,
                    trial.replay.final_geometry.airspeed_mps,
                    trial.quality_score,
                    trial.replay.bank_target_reversals,
                );
            }
            if taem_fixed_hac_candidate_preferred(
                model,
                &raw mut trial,
                &raw mut best_failure,
            ) {
                best_failure = trial;
                have_failure = true_0 != 0;
            }
        } else {
            mark_failure(&raw mut trial);
            if best_failure.status as ::core::ffi::c_uint
                != TAEM_PLAN_UNQUALIFIED as ::core::ffi::c_int as ::core::ffi::c_uint
                && better_failure(&raw mut trial, &raw mut best_failure, have_failure)
                    as ::core::ffi::c_int != 0
            {
                best_failure = trial;
                have_failure = true_0 != 0;
            }
        }
        i_6 = i_6.wrapping_add(1);
    }
    return best_failure;
}
unsafe extern "C" fn search_inputs_valid(
    mut model: *const TerminalModel,
    mut initial: *const TerminalDynamicState,
    mut hac_radius: ::core::ffi::c_double,
    mut spacing: ::core::ffi::c_double,
    mut dt: ::core::ffi::c_double,
    mut maximum_elapsed: ::core::ffi::c_double,
) -> bool {
    return !model.is_null() && (*model).replay_validated as ::core::ffi::c_int != 0
        && terminal_model_validate(
            model,
            ::core::ptr::null_mut::<::core::ffi::c_char>(),
            0 as size_t,
        ) as ::core::ffi::c_int != 0
        && terminal_state_validate(
            initial,
            ::core::ptr::null_mut::<::core::ffi::c_char>(),
            0 as size_t,
        ) as ::core::ffi::c_int != 0 && hac_radius >= TAEM_MIN_HAC_RADIUS_M
        && hac_radius.is_finite() as i32 != 0 && spacing >= 100.0f64
        && spacing.is_finite() as i32 != 0 && dt > 0.0f64 && dt.is_finite() as i32 != 0
        && maximum_elapsed > 0.0f64 && maximum_elapsed.is_finite() as i32 != 0;
}
#[no_mangle]
pub unsafe extern "C" fn taem_fixed_hac_evaluate_exact(
    mut model: *const TerminalModel,
    mut initial: *const TerminalDynamicState,
    mut hac_radius: ::core::ffi::c_double,
    mut side: ::core::ffi::c_double,
    mut spacing: ::core::ffi::c_double,
    mut dt: ::core::ffi::c_double,
    mut maximum_elapsed: ::core::ffi::c_double,
) -> TaemFixedHacCandidate {
    let mut candidate: TaemFixedHacCandidate = TaemFixedHacCandidate {
        status: TAEM_PLAN_INFEASIBLE,
        side: 0.,
        runway_end: 0,
        route_built: false,
        final_tail_qualified: false,
        required_lateral_accel_mps2: 0.,
        available_lateral_accel_mps2: 0.,
        quality_score: 0.,
        route: TaemRoute {
            valid: false,
            count: 0,
            lead_count: 0,
            arc_count: 0,
            rollout_count: 0,
            length_m: 0.,
            side: 0.,
            hac: TaemFixedHacGeometry {
                entry: TaemPoint2 { x: 0., y: 0. },
                center: TaemPoint2 { x: 0., y: 0. },
                exit: TaemPoint2 { x: 0., y: 0. },
                radius_m: 0.,
                side: 0.,
                capture_course_deg: 0.,
                arc_sweep_rad: 0.,
                arc_length_m: 0.,
                final_length_m: 0.,
                total_length_m: 0.,
            },
            p0_along_m: 0.,
            p0_cross_m: 0.,
            p1_along_m: 0.,
            p1_cross_m: 0.,
            p2_along_m: 0.,
            p2_cross_m: 0.,
            p3_along_m: 0.,
            p3_cross_m: 0.,
            capture_control: [TaemPoint2 { x: 0., y: 0. }; 6],
            spiral_exit_radius_m: 0.,
            spiral_coefficient_m: 0.,
            spiral_exit_heading_rad: 0.,
            spiral_arc_lut: [0.; 129],
            unload_length_m: 0.,
            settle_length_m: 0.,
            lead_length_m: 0.,
            lead_arc_fraction_lut: [0.; 129],
            rollout_length_m: 0.,
            alignment_along_m: 0.,
            initial_course_deg: 0.,
            runway_heading_deg: 0.,
            profile_start_altitude_m: 0.,
            profile_start_fpa_deg: 0.,
            profile_final_altitude_m: 0.,
            profile_final_slope_deg: 0.,
            profile_total_length_m: 0.,
            profile_midpoint_offset_m: 0.,
            profile_local_offset_m: 0.,
            profile_local_start_fraction: 0.,
            profile_local_peak_fraction: 0.,
            profile_local_end_fraction: 0.,
            profile_initial_sag_m: 0.,
            profile_initial_sag_length_m: 0.,
            profile_tabulated: false,
            profile_generation_aoa_deg: 0.,
            profile_altitude_lut: [0.; 257],
            profile_fpa_lut: [0.; 257],
            profile_speed_tabulated: false,
            profile_speed_lut: [0.; 257],
        },
        replay: TerminalSolverResult {
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
        },
        reason: ::core::ptr::null::<::core::ffi::c_char>(),
    };
    memset(
        &raw mut candidate as *mut ::core::ffi::c_void,
        0 as ::core::ffi::c_int,
        ::core::mem::size_of::<TaemFixedHacCandidate>() as size_t,
    );
    candidate.status = TAEM_PLAN_INFEASIBLE;
    candidate.side = if side < 0.0f64 { -1.0f64 } else { 1.0f64 };
    candidate.reason = b"invalid model, state, or frozen-HAC contract\0" as *const u8
        as *const ::core::ffi::c_char;
    if !search_inputs_valid(model, initial, hac_radius, spacing, dt, maximum_elapsed)
        || side.is_finite() as i32 == 0 || fabs(side) < 0.5f64
    {
        return candidate;
    }
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
    if !taem_geometry_state(model, initial, &raw mut geometry) {
        candidate.reason = b"initial state cannot be projected into the runway frame\0"
            as *const u8 as *const ::core::ffi::c_char;
        return candidate;
    }
    let mut stations: *mut JoinStation = calloc(
        TAEM_JOIN_MAX_STATIONS as size_t,
        ::core::mem::size_of::<JoinStation>() as size_t,
    ) as *mut JoinStation;
    if stations.is_null() {
        candidate.reason = b"candidate search could not allocate join stations\0"
            as *const u8 as *const ::core::ffi::c_char;
        return candidate;
    }
    candidate = evaluate_side(
        model,
        initial,
        &raw mut geometry,
        hac_radius,
        if side < 0.0f64 { -1.0f64 } else { 1.0f64 },
        spacing,
        dt,
        maximum_elapsed,
        stations,
    );
    free(stations as *mut ::core::ffi::c_void);
    return candidate;
}
#[no_mangle]
pub unsafe extern "C" fn taem_fixed_hac_search_runway_ends(
    mut model: *const TerminalModel,
    mut reciprocal_model: *const TerminalModel,
    mut initial: *const TerminalDynamicState,
    mut hac_radius: ::core::ffi::c_double,
    mut spacing: ::core::ffi::c_double,
    mut dt: ::core::ffi::c_double,
    mut maximum_elapsed: ::core::ffi::c_double,
) -> TaemFixedHacSearch {
    let mut result: TaemFixedHacSearch = TaemFixedHacSearch {
        status: TAEM_PLAN_INFEASIBLE,
        selected_candidate: 0,
        candidate_count: 0,
        candidates: [TaemFixedHacCandidate {
            status: TAEM_PLAN_INFEASIBLE,
            side: 0.,
            runway_end: 0,
            route_built: false,
            final_tail_qualified: false,
            required_lateral_accel_mps2: 0.,
            available_lateral_accel_mps2: 0.,
            quality_score: 0.,
            route: TaemRoute {
                valid: false,
                count: 0,
                lead_count: 0,
                arc_count: 0,
                rollout_count: 0,
                length_m: 0.,
                side: 0.,
                hac: TaemFixedHacGeometry {
                    entry: TaemPoint2 { x: 0., y: 0. },
                    center: TaemPoint2 { x: 0., y: 0. },
                    exit: TaemPoint2 { x: 0., y: 0. },
                    radius_m: 0.,
                    side: 0.,
                    capture_course_deg: 0.,
                    arc_sweep_rad: 0.,
                    arc_length_m: 0.,
                    final_length_m: 0.,
                    total_length_m: 0.,
                },
                p0_along_m: 0.,
                p0_cross_m: 0.,
                p1_along_m: 0.,
                p1_cross_m: 0.,
                p2_along_m: 0.,
                p2_cross_m: 0.,
                p3_along_m: 0.,
                p3_cross_m: 0.,
                capture_control: [TaemPoint2 { x: 0., y: 0. }; 6],
                spiral_exit_radius_m: 0.,
                spiral_coefficient_m: 0.,
                spiral_exit_heading_rad: 0.,
                spiral_arc_lut: [0.; 129],
                unload_length_m: 0.,
                settle_length_m: 0.,
                lead_length_m: 0.,
                lead_arc_fraction_lut: [0.; 129],
                rollout_length_m: 0.,
                alignment_along_m: 0.,
                initial_course_deg: 0.,
                runway_heading_deg: 0.,
                profile_start_altitude_m: 0.,
                profile_start_fpa_deg: 0.,
                profile_final_altitude_m: 0.,
                profile_final_slope_deg: 0.,
                profile_total_length_m: 0.,
                profile_midpoint_offset_m: 0.,
                profile_local_offset_m: 0.,
                profile_local_start_fraction: 0.,
                profile_local_peak_fraction: 0.,
                profile_local_end_fraction: 0.,
                profile_initial_sag_m: 0.,
                profile_initial_sag_length_m: 0.,
                profile_tabulated: false,
                profile_generation_aoa_deg: 0.,
                profile_altitude_lut: [0.; 257],
                profile_fpa_lut: [0.; 257],
                profile_speed_tabulated: false,
                profile_speed_lut: [0.; 257],
            },
            replay: TerminalSolverResult {
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
            },
            reason: ::core::ptr::null::<::core::ffi::c_char>(),
        }; 4],
        reason: ::core::ptr::null::<::core::ffi::c_char>(),
    };
    memset(
        &raw mut result as *mut ::core::ffi::c_void,
        0 as ::core::ffi::c_int,
        ::core::mem::size_of::<TaemFixedHacSearch>() as size_t,
    );
    result.status = TAEM_PLAN_INFEASIBLE;
    result.selected_candidate = -(1 as ::core::ffi::c_int);
    result.reason = b"every fixed-HAC candidate is infeasible\0" as *const u8
        as *const ::core::ffi::c_char;
    if !search_inputs_valid(model, initial, hac_radius, spacing, dt, maximum_elapsed)
        || !reciprocal_model.is_null()
            && !search_inputs_valid(
                reciprocal_model,
                initial,
                hac_radius,
                spacing,
                dt,
                maximum_elapsed,
            )
    {
        result.reason = b"invalid model, state, or candidate-search bound\0" as *const u8
            as *const ::core::ffi::c_char;
        return result;
    }
    let mut stations: *mut JoinStation = calloc(
        TAEM_JOIN_MAX_STATIONS as size_t,
        ::core::mem::size_of::<JoinStation>() as size_t,
    ) as *mut JoinStation;
    if stations.is_null() {
        result.reason = b"candidate search could not allocate join stations\0"
            as *const u8 as *const ::core::ffi::c_char;
        return result;
    }
    let mut ends: [*const TerminalModel; 2] = [model, reciprocal_model];
    let mut end_count: ::core::ffi::c_int = if !reciprocal_model.is_null() {
        2 as ::core::ffi::c_int
    } else {
        1 as ::core::ffi::c_int
    };
    let mut end: ::core::ffi::c_int = 0 as ::core::ffi::c_int;
    while end < end_count {
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
        let mut projected: bool = taem_geometry_state(
            ends[end as usize],
            initial,
            &raw mut geometry,
        );
        let mut side_index: ::core::ffi::c_int = 0 as ::core::ffi::c_int;
        while side_index < 2 as ::core::ffi::c_int {
            let mut slot: *mut TaemFixedHacCandidate = (&raw mut result.candidates
                as *mut TaemFixedHacCandidate)
                .offset((end * 2 as ::core::ffi::c_int + side_index) as isize)
                as *mut TaemFixedHacCandidate;
            if !projected {
                memset(
                    slot as *mut ::core::ffi::c_void,
                    0 as ::core::ffi::c_int,
                    ::core::mem::size_of::<TaemFixedHacCandidate>() as size_t,
                );
                (*slot).status = TAEM_PLAN_INFEASIBLE;
                (*slot).side = if side_index == 0 as ::core::ffi::c_int {
                    -1.0f64
                } else {
                    1.0f64
                };
                (*slot).reason = b"initial state cannot be projected into the runway frame\0"
                    as *const u8 as *const ::core::ffi::c_char;
            } else {
                let mut side: ::core::ffi::c_double = if side_index
                    == 0 as ::core::ffi::c_int
                {
                    -1.0f64
                } else {
                    1.0f64
                };
                let mut floor_radius: ::core::ffi::c_double = fmin(
                    hac_radius,
                    hac_radius_floor(ends[end as usize]),
                );
                let mut radius: ::core::ffi::c_double = hac_radius;
                *slot = evaluate_side(
                    ends[end as usize],
                    initial,
                    &raw mut geometry,
                    radius,
                    side,
                    spacing,
                    dt,
                    maximum_elapsed,
                    stations,
                );
                let mut step: ::core::ffi::c_int = 1 as ::core::ffi::c_int;
                while step < TAEM_HAC_RADIUS_STEPS {
                    let mut fraction: ::core::ffi::c_double = step
                        as ::core::ffi::c_double
                        / (TAEM_HAC_RADIUS_STEPS - 1 as ::core::ffi::c_int)
                            as ::core::ffi::c_double;
                    let mut next_radius: ::core::ffi::c_double = hac_radius
                        + fraction * (floor_radius - hac_radius);
                    if next_radius < radius - 1.0f64 {
                        radius = next_radius;
                        let mut tighter: TaemFixedHacCandidate = evaluate_side(
                            ends[end as usize],
                            initial,
                            &raw mut geometry,
                            radius,
                            side,
                            spacing,
                            dt,
                            maximum_elapsed,
                            stations,
                        );
                        if taem_fixed_hac_candidate_preferred(
                            ends[end as usize],
                            &raw mut tighter,
                            slot,
                        ) as ::core::ffi::c_int != 0
                            || (*slot).status as ::core::ffi::c_uint
                                != TAEM_PLAN_UNQUALIFIED as ::core::ffi::c_int
                                    as ::core::ffi::c_uint
                                && tighter.route_built as ::core::ffi::c_int != 0
                                && !(*slot).route_built
                        {
                            *slot = tighter;
                        }
                    }
                    step += 1;
                }
            }
            (*slot).runway_end = end;
            side_index += 1;
        }
        end += 1;
    }
    free(stations as *mut ::core::ffi::c_void);
    result.candidate_count = end_count * 2 as ::core::ffi::c_int;
    let mut best: ::core::ffi::c_int = -(1 as ::core::ffi::c_int);
    let mut i: ::core::ffi::c_int = 0 as ::core::ffi::c_int;
    while i < result.candidate_count {
        if result.candidates[i as usize].status as ::core::ffi::c_uint
            == TAEM_PLAN_UNQUALIFIED as ::core::ffi::c_int as ::core::ffi::c_uint
            && (best < 0 as ::core::ffi::c_int
                || taem_fixed_hac_candidate_preferred(
                    ends[result.candidates[i as usize].runway_end as usize],
                    (&raw mut result.candidates as *mut TaemFixedHacCandidate)
                        .offset(i as isize) as *mut TaemFixedHacCandidate,
                    (&raw mut result.candidates as *mut TaemFixedHacCandidate)
                        .offset(best as isize) as *mut TaemFixedHacCandidate,
                ) as ::core::ffi::c_int != 0)
        {
            best = i;
        }
        i += 1;
    }
    if best >= 0 as ::core::ffi::c_int {
        result.status = TAEM_PLAN_UNQUALIFIED;
        result.selected_candidate = best;
        result.reason = result.candidates[best as usize].reason;
        return result;
    }
    let mut i_0: ::core::ffi::c_int = 0 as ::core::ffi::c_int;
    while i_0 < result.candidate_count {
        if result.candidates[i_0 as usize].status as ::core::ffi::c_uint
            == TAEM_PLAN_SEARCH_EXHAUSTED as ::core::ffi::c_int as ::core::ffi::c_uint
        {
            result.status = TAEM_PLAN_SEARCH_EXHAUSTED;
            result.reason = b"candidate search exhausted before a route reached the HAC exit\0"
                as *const u8 as *const ::core::ffi::c_char;
        }
        i_0 += 1;
    }
    return result;
}
#[no_mangle]
pub unsafe extern "C" fn taem_fixed_hac_search(
    mut model: *const TerminalModel,
    mut initial: *const TerminalDynamicState,
    mut hac_radius: ::core::ffi::c_double,
    mut spacing: ::core::ffi::c_double,
    mut dt: ::core::ffi::c_double,
    mut maximum_elapsed: ::core::ffi::c_double,
) -> TaemFixedHacSearch {
    return taem_fixed_hac_search_runway_ends(
        model,
        ::core::ptr::null::<TerminalModel>(),
        initial,
        hac_radius,
        spacing,
        dt,
        maximum_elapsed,
    );
}
pub const NULL: *mut ::core::ffi::c_void = ::core::ptr::null_mut::<
    ::core::ffi::c_void,
>();
pub const true_0: ::core::ffi::c_int = 1 as ::core::ffi::c_int;
pub const false_0: ::core::ffi::c_int = 0 as ::core::ffi::c_int;
