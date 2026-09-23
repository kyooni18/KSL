#ifndef KSP_LANDER_CORE_H
#define KSP_LANDER_CORE_H

#include "json.h"
#include "entry_exec.h"
#include "entry_lateral.h"
#include "taem_exec.h"

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>

#define LANDER_PI 3.14159265358979323846264338327950288
#define DEG2RAD (LANDER_PI / 180.0)
#define RAD2DEG (180.0 / LANDER_PI)
/* Live kRPC uses ~500 m atmosphere samples, while ShuttleSim's fitted
   atmosphere table is 250 m and extends beyond the 70 km boundary.  144 slots
   silently truncated the simulator table near 37.75 km, so every predictor
   lookup above that altitude reused dense lower-atmosphere air. */
#define LANDER_ATMOSPHERE_SAMPLE_MAX 512
#define TAEM_INTERFACE_FPA_DEBT_LIMIT_DEG 6.0

typedef struct { double x, y, z; } Vector3;
typedef struct { double latitude, longitude, altitude; } GeoPoint;

typedef enum { CONN_DISCONNECTED, CONN_CONNECTING, CONN_CONNECTED, CONN_FAILED } ConnectionStatus;
typedef enum {
    PHASE_IDLE, PHASE_PLANNING, PHASE_CALIBRATION, PHASE_COAST, PHASE_BURN_SETUP,
    PHASE_DEORBIT_BURN, PHASE_ENTRY_INTERFACE, PHASE_ENTRY_ENERGY, PHASE_TAEM,
    PHASE_HEADING_ALIGNMENT, PHASE_FINAL, PHASE_FLARE, PHASE_TOUCHDOWN,
    PHASE_ROLLOUT, PHASE_COMPLETE, PHASE_PAUSED, PHASE_ABORT, PHASE_FAULT, PHASE_ATTITUDE_RECOVERY
} GuidancePhase;
typedef enum { SPEED_UNCHANGED, SPEED_ORBIT, SPEED_SURFACE } NavballSpeedMode;
typedef enum { PROFILE_ORBITAL, PROFILE_ENTRY, PROFILE_TAEM, PROFILE_APPROACH, PROFILE_FLARE, PROFILE_ROLLOUT, PROFILE_RECOVERY } ControlProfile;
typedef enum {
    TERMINAL_TRAJECTORY_CAPTURE,
    TERMINAL_OUTER_FINAL,
    TERMINAL_PREFLARE,
    TERMINAL_INNER_FINAL,
    TERMINAL_TOUCHDOWN_FLARE,
    TERMINAL_GROUND
} TerminalVerticalStage;
typedef enum { CAL_IDLE, CAL_SETTLING, CAL_SAMPLING, CAL_COMPLETE, CAL_STOPPED, CAL_ABORTED } CalibrationRunState;
typedef enum { TRAJ_PLANNED, TRAJ_ACTUAL, TRAJ_REFERENCE } TrajectoryKind;

typedef struct {
    char serial_port[256];
    int baud_rate, timeout_ms;
    char client_name[128];
} ConnectionConfiguration;
typedef struct {
    char name[128]; double latitude, longitude, altitude, runway_heading, runway_length, runway_width; bool allow_reciprocal_runway;
} LandingSite;
typedef struct {
    char model_id[96];
    double estimated_lift_to_drag, estimated_ballistic_coefficient, entry_angle_of_attack;
    double maximum_angle_of_attack, maximum_bank_angle, maximum_dynamic_pressure, maximum_g_load;
    double minimum_safe_speed, final_approach_speed, touchdown_speed;
    bool allow_powered_approach; double maximum_approach_throttle; unsigned airbrake_action_group;
} VehicleProfile;
typedef struct {
    double target_deorbit_capture_radius, entry_interface_altitude_margin, target_entry_range;
    double target_entry_flight_path_angle, maximum_entry_flight_path_angle, target_post_burn_periapsis_altitude;
    double deorbit_maximum_throttle, deorbit_throttle_ramp_duration, taem_interface_altitude, taem_interface_range;
    double taem_force_handoff_speed;
    double mm304_handoff_radius, mm304_perpendicular_heading_half_width;
    double deorbit_timing_uncertainty, deorbit_thrust_uncertainty_fraction, deorbit_delta_v_uncertainty;
    double deorbit_mass_uncertainty_fraction, deorbit_position_uncertainty, deorbit_velocity_uncertainty;
    double deorbit_pointing_uncertainty, deorbit_atmosphere_uncertainty_fraction, deorbit_robust_minimum_pass_fraction;
    double hac_radius, hac_look_ahead_angle, final_approach_distance, final_glide_slope, taem_glide_slope;
    double gear_deployment_altitude, flare_altitude, touchdown_sink_rate, guidance_rate, prediction_interval;
    double s_turn_minimum_leg_duration, entry_roll_rate, entry_roll_acceleration;
    double taem_roll_rate, approach_roll_rate; bool use_time_warp; double minimum_planning_lead_time;
} GuidanceSettings;
typedef struct {
    bool enable_passive_calibration, enable_trajectory_calibration, auto_apply_in_flight;
    double minimum_dynamic_pressure, maximum_sample_dynamic_pressure, maximum_sideslip;
    double minimum_calibration_radar_altitude, maximum_calibration_dynamic_pressure, maximum_calibration_g_load;
    double maximum_calibration_sink_rate, abort_stall_fraction, minimum_angle_of_attack, maximum_angle_of_attack;
    double angle_of_attack_step, settling_duration, sampling_duration;
    /* Deprecated serialization compatibility only; adaptation is response-time-derived. */
    double trajectory_learning_rate;
} CalibrationSettings;
typedef struct {
    ConnectionConfiguration connection; LandingSite site; VehicleProfile vehicle;
    GuidanceSettings guidance; CalibrationSettings calibration;
} LandingConfiguration;

typedef struct {
    char name[96]; double radius, gravitational_parameter, rotational_speed, atmosphere_depth, surface_density;
    double atmosphere_adiabatic_index;
    size_t atmosphere_sample_count;
    double atmosphere_altitude[LANDER_ATMOSPHERE_SAMPLE_MAX];
    double atmosphere_pressure[LANDER_ATMOSPHERE_SAMPLE_MAX];
    double atmosphere_density[LANDER_ATMOSPHERE_SAMPLE_MAX];
    Vector3 north_axis, prime_meridian_at_epoch; double epoch_ut;
} PlanetModel;
typedef struct VesselPhysicsModel VesselPhysicsModel;
typedef struct { double lift_to_drag, ballistic_coefficient, confidence; } AerodynamicModel;
typedef struct { AerodynamicModel regimes[4]; } AerodynamicEnvelope;
typedef struct {
    const VesselPhysicsModel *physics; /* Borrowed, immutable during prediction; controller owns it. */
    double density_scale, drag_scale, lift_scale, bank_effectiveness, speed_of_sound;
    double speed_of_sound_scale;
    /* Explicit robustness multipliers.  Normal flight leaves these at unity;
       stress simulations vary them without mutating the certified data book. */
    double stress_drag_scale, stress_lift_scale;
    double altitude_residual, speed_residual, range_residual, confidence; int accepted_samples;
} TrajectoryCalibrationModel;
typedef struct { double ut; Vector3 position, velocity; double mass; } VehicleState;

typedef struct {
    double ut, latitude, longitude, altitude, speed; GuidancePhase phase; TrajectoryKind kind;
} TrajectoryPoint;
typedef struct { TrajectoryPoint *points; size_t count, capacity; } Trajectory;

typedef struct {
    double created_ut, burn_ut, delta_v, estimated_burn_duration, predicted_taem_distance, predicted_taem_range_error, predicted_closest_distance;
    double planning_mass, planning_available_thrust;
    double predicted_entry_range, predicted_entry_flight_path_angle, predicted_post_burn_periapsis_altitude;
    bool nominal_capture_achieved, robustness_qualified, target_capture_achieved;
    unsigned robustness_scenarios, robustness_passed, robustness_unsafe;
    double robustness_pass_fraction, worst_case_closest_distance, worst_case_taem_range_error;
    bool execution_qualified, execution_degraded;
    unsigned recovery_passed;
    double recovery_pass_fraction;
    double worst_case_entry_flight_path_angle, worst_case_post_burn_periapsis_altitude;
    double worst_case_peak_dynamic_pressure, worst_case_peak_g_load;
    bool live_cutoff_capture_qualified;
    double live_cutoff_periapsis_altitude, live_cutoff_closest_distance, live_cutoff_entry_flight_path_angle;
    bool achieved_state_verified, achieved_state_capture_qualified;
    double achieved_post_burn_periapsis_altitude;
    double confidence; Trajectory trajectory; char note[384];
} DeorbitPlan;

/* Closed-loop attitude response identified by a runtime adapter or calibrated
   vehicle model. Rates are degrees/s, angular accelerations are degrees/s^2,
   and natural frequencies are 1/s. A validity flag means all parameters for
   that axis describe the same response model; no synthetic defaults are
   implied when the flag is false. */
typedef struct {
    bool pitch_valid, roll_valid;
    double pitch_natural_frequency_s_inv, pitch_damping_ratio;
    double roll_natural_frequency_s_inv, roll_damping_ratio;
    double maximum_pitch_rate_deg_s, maximum_roll_rate_deg_s;
    double maximum_pitch_accel_deg_s2, maximum_roll_accel_deg_s2;
} AttitudeResponseModel;

typedef struct {
    double ut; char vessel_name[128];
    double latitude, longitude, mean_altitude, radar_altitude, vertical_speed, horizontal_speed, surface_speed, true_air_speed;
    double atmospheric_density, speed_of_sound, mach, heading, ground_track_heading, course_to_site_error, pitch, roll;
    double pitch_rate, roll_rate, heading_rate, angle_of_attack_rate;
    bool has_angle_of_attack_rate, has_body_pitch_rate, has_body_roll_rate, has_body_yaw_rate, has_course_rate;
    double body_pitch_rate, body_roll_rate, body_yaw_rate, course_rate, autopilot_error;
    bool has_attitude_quaternion; double attitude_quaternion[4]; char attitude_reference_frame[48];
    bool has_command_pitch_error, has_command_roll_error, has_command_heading_error;
    double command_pitch_error, command_roll_error, command_heading_error;
    bool has_pid_gains; double autopilot_pitch_pid_gains[3], autopilot_roll_pid_gains[3], autopilot_yaw_pid_gains[3];
    NavballSpeedMode navball_speed_mode;
    AttitudeResponseModel attitude_response;
    double angle_of_attack, sideslip, dynamic_pressure, static_pressure, g_force, stall_fraction, lift_force, drag_force;
    /* True only when stall_fraction came from an independent measured stall
       observable. False means it is the conservative q/speed/AoA proxy. */
    bool stall_fraction_is_measured;
    bool has_force_vectors, has_center_of_mass, has_center_of_mass_root;
    /* True only when the bridge obtained a coherent live physics frame rather
       than substituting a startup/scene-load stream fallback.  Guidance may
       still display a degraded frame, but calibration must never learn it. */
    bool physics_sample_valid;
    Vector3 center_of_mass_root; /* metres, native kRPC root-part axes */
    Vector3 lift_vector, drag_vector, center_of_mass; /* canonical body non-rotating, SI */
    double dry_mass;
    double physics_confidence, physics_authority[3], physics_authority_confidence[3], physics_mass_flow;
    double physics_model_residual, physics_model_residual_confidence, physics_certified_uncertainty;
    Vector3 physics_force_residual_per_q, physics_force_residual_sigma_per_q;
    double physics_force_residual_confidence;
    bool physics_airbrake_model_available;
    double physics_airbrake_model_confidence, physics_airbrake_drag_accel;
    unsigned physics_samples, physics_live_samples;
    double predicted_physics_observed_seconds, predicted_physics_fallback_seconds;
    bool predicted_shadow_guidance, predicted_terminal_policy_feasible;
    double predicted_terminal_survivability_stress_score, predicted_physics_relative_uncertainty;
    unsigned predicted_uncertainty_scenarios;
    double predicted_raw_published_position_delta_30s, predicted_raw_published_position_delta_60s, predicted_raw_published_position_delta_120s;
    double mass, available_thrust, current_thrust, apoapsis_altitude, periapsis_altitude, orbit_period, throttle;
    bool has_controls; double control_pitch, control_roll, control_yaw;
    bool has_torque; double available_pitch_torque, available_roll_torque, available_yaw_torque;
    bool has_inertia; double pitch_moment_of_inertia, roll_moment_of_inertia, yaw_moment_of_inertia;
    bool has_loop_wall_delta, has_telemetry_latency, has_guidance_compute, has_apply_latency, has_control_loop;
    double loop_wall_delta_ms, telemetry_latency_ms, guidance_compute_ms, apply_latency_ms, control_loop_ms;
    bool has_rpc_budget;
    unsigned rpc_read_calls, rpc_read_wire_requests, rpc_apply_calls, rpc_apply_wire_requests;
    unsigned rpc_total_calls, rpc_total_wire_requests;
    bool gear, brakes, has_airbrakes, airbrakes; char vessel_situation[64];
    double range_to_site, bearing_to_site, heading_error, runway_along_track, runway_cross_track, flight_path_angle;
    double estimated_lift_to_drag, estimated_ballistic_coefficient, aerodynamic_confidence;
    double calibrated_best_glide_angle_of_attack, calibrated_stall_speed;
    double predicted_miss_distance, predicted_taem_distance, predicted_taem_range_error, predicted_entry_range;
    double predicted_entry_flight_path_angle, predicted_taem_speed, predicted_taem_energy_error;
    double predicted_peak_dynamic_pressure, predicted_peak_g_load, predicted_s_turn_reversals;
    double trajectory_density_scale, trajectory_drag_scale, trajectory_lift_scale, bank_effectiveness;
    double trajectory_calibration_confidence, trajectory_altitude_residual, trajectory_speed_residual, trajectory_range_residual;
    double energy_excess_range;
} Telemetry;

typedef struct {
    double target_pitch, target_heading, target_roll, target_throttle, wheel_steering;
    bool has_target_aoa; double target_aoa;
    bool gear, brakes, airbrakes, use_inertial_direction; Vector3 inertial_direction;
    bool autopilot_engaged, heading_control_enabled, hac_control_tuning, terminal_pitch_tuning; NavballSpeedMode navball_speed_mode; ControlProfile control_profile;
} GuidanceCommand;

typedef struct {
    bool applied, autopilot_engaged, gear, brakes, airbrakes;
    double target_pitch, target_heading, target_roll, throttle, wheel_steering;
    bool has_actuator_feedback;
    double control_pitch, control_roll, control_yaw;
    bool has_control_diagnostics, control_body_pitch_rate_available, control_body_roll_rate_available, control_body_yaw_rate_available;
    double control_target_aoa, control_measured_aoa, control_aoa_rate, control_roll_rate;
    double control_body_pitch_rate, control_body_roll_rate, control_body_yaw_rate;
    double control_pitch_error, control_pitch_trim, control_pitch_authority, control_pitch_aero_fraction;
    double control_roll_authority, control_roll_raw_authority, control_roll_aero_fraction;
    double control_target_roll_rate;
    double control_beta_yaw_gain, control_beta_roll_coupling, control_beta_confidence, control_yaw_command;
    char control_revision[48];
    char reference_frame[24], speed_mode[24], control_profile[24];
} KRPCApplyResult;

typedef struct {
    CalibrationRunState state; bool active; double progress, target_angle_of_attack;
    int accepted_samples, rejected_samples; double confidence, planning_confidence;
    double best_glide_angle_of_attack, estimated_stall_speed; char status[256], warning[256]; bool has_warning;
} CalibrationSnapshot;

typedef struct {
    ConnectionStatus connection_status; GuidancePhase phase; Telemetry telemetry; GuidanceCommand command;
    DeorbitPlan *plan; Trajectory actual_trajectory, reference_trajectory, predicted_trajectory; char status_message[384];
    char warning_message[384], last_error[512]; bool has_warning, has_error, automation_engaged, paused;
    CalibrationSnapshot calibration; VehicleProfile adaptive_vehicle_profile;
    /* Forensic frame metadata: replay timing, inertial motion and state-machine decisions. */
    uint64_t tick_sequence, log_sequence; double wall_monotonic_seconds, simulation_dt;
    bool has_vehicle_state; VehicleState vehicle_state;
    bool has_attitude_quaternion; double attitude_quaternion[4]; char attitude_reference_frame[48];
    bool command_apply_attempted, command_applied;
    double command_computed_wall_seconds, command_apply_started_wall_seconds, command_apply_finished_wall_seconds;
    KRPCApplyResult applied_command;
    bool guidance_has_burn_started, guidance_burn_completed, guidance_atmosphere_crossed;
    bool guidance_final_captured, guidance_airbrakes_deployed, guidance_hac_side_selected;
    double guidance_delivered_delta_v, guidance_burn_active_elapsed, guidance_s_turn_sign;
    double guidance_hac_side, guidance_entry_leg_elapsed;
    bool guidance_entry_plan_valid, guidance_entry_plan_terminal_ready, guidance_entry_reversal_scheduled;
    bool guidance_entry_reversal_is_final;
    double guidance_entry_plan_ut, guidance_entry_plan_target_bank, guidance_entry_plan_target_aoa;
    double guidance_entry_plan_target_heading, guidance_entry_plan_segment_remaining, guidance_entry_plan_cost;
    double guidance_entry_plan_taem_range_error, guidance_entry_plan_taem_speed, guidance_entry_plan_taem_energy_error;
    double guidance_entry_reversal_time_remaining, guidance_entry_reversal_range, guidance_entry_reversal_sign;
    unsigned guidance_entry_plan_predicted_reversals;
    EntryExecTelemetry guidance_entry_exec;
    TaemExecTelemetry guidance_taem_exec;
    bool guidance_hac_captured, guidance_hac_completed, guidance_hac_progress_valid;
    bool guidance_hac_transition_active;
    bool guidance_terminal_prediction_valid, guidance_terminal_candidate_valid, guidance_terminal_committed;
    double guidance_terminal_candidate_radius, guidance_terminal_candidate_altitude, guidance_terminal_candidate_speed;
    int guidance_terminal_candidate_kind;
    bool guidance_terminal_candidate_geometry_degraded;
    bool guidance_terminal_candidate_energy_degraded;
    bool guidance_terminal_candidate_shell_degraded;
    double guidance_terminal_candidate_side;
    double guidance_terminal_candidate_slope;
    double guidance_terminal_candidate_curve_length;
    double guidance_terminal_candidate_lead_length;
    double guidance_terminal_candidate_arc_remaining;
    double guidance_terminal_candidate_final_distance;
    double guidance_terminal_candidate_quality;
    double guidance_terminal_candidate_arrival_ut;
    double guidance_terminal_candidate_course;
    double guidance_terminal_candidate_peak_lateral;
    double guidance_terminal_candidate_peak_rate_ratio;
    double guidance_terminal_candidate_exit_speed;
    bool guidance_terminal_candidate_path_degraded;
    bool guidance_terminal_candidate_control_degraded;
    bool guidance_terminal_candidate_rate_degraded;
    bool guidance_terminal_candidate_end_degraded;
    double guidance_terminal_reference_fpa, guidance_terminal_mix;
    double guidance_hac_remaining, guidance_hac_radius, guidance_minimum_turn_radius;
    double guidance_hac_transition_progress, guidance_commanded_course_rate_estimate;
    double guidance_hac_transition_points[8];
    unsigned guidance_hac_circuit_count;
    double guidance_hac_circuit_slope, guidance_terminal_reentry_after_ut;
    char decision_reason[384];
} LandingSnapshot;

typedef struct { double kp, ki, kd, integral_limit, derivative_tc, integral, previous_error, filtered_derivative; bool has_previous; } RobustPID;
typedef struct { double value; bool has_value; } SlewLimiter;
typedef struct { double value, rate; bool has_value; } JerkLimiter;
typedef struct { double tc, value; bool has_value; } LowPass;
typedef struct { double integral; bool deployed, initialized; } SpeedbrakeController;
typedef struct { double command_rate_scale, command_accel_scale, response_scale; } FlightControlRegime;
typedef struct {
    double remaining, early, projected_taem_range_error, range_norm, energy_norm, raw_score;
} EntryTerminalDemand;

typedef struct {
    double heading,bank,desired_altitude,distance_final,arc_remaining,radial_error,course_error,angle;
    double lateral_acceleration; /* Signed path-tracking acceleration from curvature and path error. */
} HACGuidance;

typedef struct KRPCSession KRPCSession;
typedef struct LandingController LandingController;

typedef struct {
    bool valid;
    /* Geometry may be useful for acquisition before the unpowered arrival
       projection can certify the downstream energy/speed contract.  This flag
       keeps that distinction explicit: valid is a guidance target geometry,
       energy_qualified is strict MM304 delivery evidence. */
    bool energy_qualified;
    double along_track, cross_track, course;
    double altitude, speed, flight_path_angle, specific_energy;
    double selected_ut, arrival_ut, quality_score;
    double acquisition_lead, remaining_path, response_time;
} TaemInterfaceTarget;

/* Shared by live ownership and entry prediction: one acquisition boundary. */
typedef struct {
    double horizontal_radius_m;
    double perpendicular_heading_half_width_deg;
} TaemHandoffContract;

TaemHandoffContract taem_handoff_contract(const GuidanceSettings *settings);

typedef struct {
    bool valid, ready;
    unsigned veto;
    double along, cross, course_error, altitude_error, energy_margin, turn_margin;
    double path_length_m, required_time_s, available_time_s, minimum_turn_radius_m;
    double energy_available, energy_drag_work, energy_uncertainty;
} TaemInterfaceCapture;
TaemInterfaceCapture entry_taem_interface_capture(const TaemInterfaceTarget *target,
    const Telemetry *telemetry,double course,const PlanetModel *planet,
    const LandingConfiguration *configuration);

typedef struct {
    Trajectory trajectory; double taem_distance, taem_range_error, closest_distance; bool entered_atmosphere, reached_taem, taem_ownership_boundary_missed;
    bool taem_dynamic_interface_captured, taem_dynamic_interface_target_valid, taem_terminal_candidate_valid;
    bool taem_terminal_candidate_geometry_clean;
    double entry_range, entry_flight_path_angle, entry_speed, entry_course_error, taem_speed, taem_flight_path_angle;
    double taem_altitude, taem_desired_altitude, taem_along_track, taem_cross_track, taem_course;
    bool mm304_gate_recorded;
    double mm304_gate_range, mm304_gate_along_track, mm304_gate_cross_track;
    double mm304_gate_course, mm304_gate_altitude, mm304_gate_speed, mm304_gate_flight_path_angle;
    double taem_hac_capture_score, taem_energy_error, taem_interface_error, peak_dynamic_pressure, peak_g_load;
    double best_terminal_capture_cost;
    double minimum_entry_speed, maximum_abs_angle_of_attack;
    double minimum_bank_control_margin, minimum_aoa_control_margin;
    bool control_horizon_recorded; double control_horizon_speed, control_horizon_altitude, control_horizon_range, control_horizon_specific_energy, control_horizon_alignment_distance;
    bool has_first_s_turn_reversal, first_s_turn_reversal_is_final;
    double first_s_turn_reversal_ut, first_s_turn_reversal_range, first_s_turn_reversal_sign;
    unsigned s_turn_reversals; double s_turn_max_bank; VehicleState final_state;
    double physics_observed_seconds, physics_fallback_seconds;
    bool shadow_guidance_used, shadow_terminal_prediction_valid, shadow_terminal_policy_feasible;
    bool shadow_terminal_path_committed, shadow_final_approach_captured, shadow_aborted;
    double terminal_survivability_stress_score, physics_relative_uncertainty;
    unsigned uncertainty_scenarios;
} EntryPrediction;

typedef struct {
    bool valid, terminal_ready;
    uint64_t plan_id, plan_version, parent_plan_id, parent_plan_version;
    double planned_ut, target_bank, target_aoa, target_heading, bank_cap, target_turn_radius, segment_duration, cost;
    double taem_range_error, taem_speed, taem_energy_error, closest_distance;
    bool has_planned_reversal, planned_reversal_is_final, final_heading_lock;
    double planned_reversal_ut, planned_reversal_range, planned_reversal_sign;
    unsigned predicted_reversals;
} EntryControlPlan;

/* One physically propagated Entry -> reversal -> perpendicular HAC inlet.
 * The first side, energy profile, deadline and inlet are selected together. */
typedef enum {
    ENTRY_TOPOLOGY_FAILURE_NONE=0,
    ENTRY_TOPOLOGY_FAILURE_REVERSAL_SETUP=1u<<0,
    ENTRY_TOPOLOGY_FAILURE_TERMINAL_TURN=1u<<1,
    ENTRY_TOPOLOGY_FAILURE_HEADING_LOCK=1u<<2,
    ENTRY_TOPOLOGY_FAILURE_NONFINITE_STATE=1u<<3,
    ENTRY_TOPOLOGY_FAILURE_ALTITUDE_FLOOR=1u<<4,
    ENTRY_TOPOLOGY_FAILURE_SPEED_FLOOR=1u<<5,
    ENTRY_TOPOLOGY_FAILURE_DYNAMIC_PRESSURE=1u<<6,
    ENTRY_TOPOLOGY_FAILURE_G_LOAD=1u<<7,
    ENTRY_TOPOLOGY_FAILURE_POSITION=1u<<8,
    ENTRY_TOPOLOGY_FAILURE_COURSE=1u<<9,
    ENTRY_TOPOLOGY_FAILURE_CAPTURE_BANK=1u<<10,
    ENTRY_TOPOLOGY_FAILURE_SPEED_PATH=1u<<11,
    ENTRY_TOPOLOGY_FAILURE_MANEUVER_PATH=1u<<12,
    ENTRY_TOPOLOGY_FAILURE_CAPTURE_SPEED=1u<<13,
    ENTRY_TOPOLOGY_FAILURE_CAPTURE_SPATIAL=1u<<14,
    ENTRY_TOPOLOGY_FAILURE_CAPTURE_COURSE=1u<<15,
    ENTRY_TOPOLOGY_FAILURE_CAPTURE_ALTITUDE=1u<<16,
    ENTRY_TOPOLOGY_FAILURE_CAPTURE_ENERGY=1u<<17,
    ENTRY_TOPOLOGY_FAILURE_CAPTURE_FPA=1u<<18,
    ENTRY_TOPOLOGY_FAILURE_CAPTURE_STRUCTURAL=1u<<19
} EntryTopologyFailureFlags;

typedef struct {
    bool valid;
    unsigned failure_flags;
    double planned_ut, first_sign, first_bank, turn_bank, outbound_course_offset, first_aoa, turn_aoa;
    double aoa_switch_speed, reversal_ut, terminal_turn_ut, capture_ut;
    double reversal_along, reversal_cross, reversal_altitude, reversal_speed, reversal_course;
    double reversal_turn_radius, reversal_geometry_error;
    double terminal_turn_along, terminal_turn_cross, terminal_turn_altitude, terminal_turn_speed, terminal_turn_course;
    double terminal_turn_radius, terminal_turn_geometry_error;
    double capture_along, capture_cross, capture_altitude, capture_speed;
    double capture_course, capture_course_error, capture_bank, capture_heading_debt, capture_endpoint_error;
    double heading_lock_ut, heading_lock_along, heading_lock_cross, heading_lock_altitude, heading_lock_speed, heading_lock_course, heading_lock_bank;
    double position_error, glide_reserve, terminal_energy_margin, minimum_alignment_altitude, cost;
    unsigned capture_veto;
    double capture_turn_margin;
    TaemInterfaceTarget inlet;
} EntryTopologyPlan;

typedef struct {
    bool valid, safe, terminal_feasible;
    bool dynamic_pressure_violation, g_load_violation, stall_risk, control_margin_violation;
    double dynamic_pressure_ratio, g_load_ratio, minimum_speed_ratio;
    double bank_margin, aoa_margin;
} EntryPredictionAssessment;

typedef enum {
    ENTRY_SUPERVISION_PASS_THROUGH = 0,
    ENTRY_SUPERVISION_BOUNDED_CORRECTION,
    ENTRY_SUPERVISION_FALLBACK,
    ENTRY_SUPERVISION_INFEASIBLE
} EntrySupervisionMode;

typedef struct {
    bool valid, taem_ownership_boundary_missed;
    EntrySupervisionMode mode;
    EntryControlPlan plan;
    EntryPredictionAssessment nominal_assessment, selected_assessment;
    double bank_correction, aoa_correction;
} EntrySupervisionResult;

#define PREDICTOR_PLANNER_TRACE_MAX_CANDIDATES 48
#define PREDICTOR_PLANNER_TRACE_MAX_PATH_POINTS 24
typedef enum {
    PREDICTOR_PLAN_CANDIDATE_NOMINAL = 0,
    PREDICTOR_PLAN_CANDIDATE_BOUNDED,
    PREDICTOR_PLAN_CANDIDATE_LEGACY
} PredictorPlanCandidateSource;
typedef enum {
    PREDICTOR_CANDIDATE_REJECT_NONE=0,
    PREDICTOR_CANDIDATE_REJECT_INVALID=1u<<0,
    PREDICTOR_CANDIDATE_REJECT_DYNAMIC_PRESSURE=1u<<1,
    PREDICTOR_CANDIDATE_REJECT_G_LOAD=1u<<2,
    PREDICTOR_CANDIDATE_REJECT_STALL=1u<<3,
    PREDICTOR_CANDIDATE_REJECT_CONTROL_MARGIN=1u<<4,
    PREDICTOR_CANDIDATE_REJECT_NO_TAEM=1u<<5,
    PREDICTOR_CANDIDATE_REJECT_TERMINAL_INFEASIBLE=1u<<6,
    PREDICTOR_CANDIDATE_REJECT_HIGHER_COST=1u<<7,
    PREDICTOR_CANDIDATE_REJECT_POLICY_PRIORITY=1u<<8
} PredictorPlanCandidateRejectionFlags;
typedef struct {
    double terminal_range, terminal_energy, hac_geometry, pending_terminal, altitude, interface_match;
    double penetration, short_horizon, transition, dynamic_pressure, g_load;
    double reversal_count, turn_tightness, vertical_lift, bank_change, aoa_change;
    double bounded_adjustment;
} PredictorPlannerCostTerms;
typedef struct {
    PredictorPlanCandidateSource source;
    EntryControlPlan plan;
    EntryPredictionAssessment assessment;
    double evaluation_cost;
    PredictorPlannerCostTerms cost_terms;
    double peak_dynamic_pressure, peak_g_load, minimum_entry_speed, terminal_capture_score;
    bool reached_taem, selected;
    unsigned evaluation_index;
    unsigned rejection_flags;
    size_t trajectory_total_points, trajectory_sample_count;
    bool trajectory_truncated;
    TrajectoryPoint trajectory_points[PREDICTOR_PLANNER_TRACE_MAX_PATH_POINTS];
} PredictorPlannerCandidateTrace;
typedef struct {
    bool valid;
    EntrySupervisionMode mode;
    double ut, altitude, airspeed, range;
    double current_bank, current_aoa, current_sign, current_leg_elapsed, current_commit_remaining;
    bool current_leg_established, side_locked, final_heading_lock;
    unsigned candidate_count, dropped_candidate_count, candidate_budget;
    int selected_index;
    double search_horizon_seconds, maximum_bank_correction, maximum_aoa_correction;
    bool selection_converged;
    uint64_t producing_tick_sequence, accepted_tick_sequence;
    uint64_t plan_id, plan_version, parent_plan_id, parent_plan_version;
    PredictorPlannerCandidateTrace candidates[PREDICTOR_PLANNER_TRACE_MAX_CANDIDATES];
} PredictorPlannerTrace;

typedef struct {
    GuidancePhase phase; GuidanceCommand command; char status[384], warning[384]; bool has_warning; Trajectory reference;
} GuidanceResult;

typedef struct { double e,n; } HACPoint2;
typedef struct {
    bool valid, degraded;
    bool degraded_path, degraded_control, degraded_rate, degraded_end, degraded_arc;
    bool heading_cone;
    HACPoint2 lead_start;
    HACPoint2 p0,p1,p2,p3;
    HACPoint2 cone_center;
    double lead_length, length, end_angle, arc_remaining, exit_speed, peak_lateral, opposite_lateral,
        peak_course_rate_ratio,
        violation_score,
        debug_min_control_ratio,debug_min_rate_ratio,debug_min_peak_lateral,
        debug_min_control_length,debug_min_control_arc,debug_min_control_advance;
    double cone_start_angle, cone_end_angle, cone_arc_length;
    int reject_arc, reject_opposite, reject_control, reject_rate, reject_end, reject_path;
} HACTransitionPlan;

typedef enum {
    TERMINAL_PATH_NONE = 0,
    TERMINAL_PATH_SPLINE = 1,
    TERMINAL_PATH_HAC = 2
} TerminalPathKind;

typedef struct {
    bool valid, degraded;
    /* Keep geometry feasibility separate from the preferred TAEM altitude
       shell.  A regular, dynamically qualified HAC a little above/below the
       nominal shell is still a valid path and must not be treated like an
       unflyable join. */
    bool geometry_degraded, energy_degraded, shell_degraded;
    TerminalPathKind kind;
    HACTransitionPlan join;
    double radius, side, slope, final_distance, response, energy_aoa;
    double altitude, speed, course, selected_ut, arrival_ut, quality_score;
    /* Physical acquisition cost from the live state to this candidate's
       response lead.  A path that is elegant downstream but already behind
       the vehicle must not outrank a path the shuttle can begin executing. */
    double execution_distance, execution_time, execution_course_error;
    double execution_radial_closure, execution_horizon, execution_margin;
    bool execution_evaluated;
} TerminalCandidate;

typedef struct {
    GuidancePhase phase; bool automation_engaged, paused, aborted;
    bool diagnostic_shadow; /* Prediction rollouts must not masquerade as live decisions. */
    double s_turn_sign; double s_turn_leg_started_ut; bool has_s_turn_leg_started;
    double s_turn_reversal_requested_ut; bool has_s_turn_reversal_requested;
    EntryExecutive entry_exec; EntryLateralState entry_lateral; EntryLateralState taem_s_turn_lateral; TaemExecutive taem_exec;
    bool entry_alpha_has_target, entry_alpha_has_modulation, entry_drag_ratio_valid;
    double entry_alpha_target, entry_alpha_modulation, entry_drag_velocity_ratio;
    bool entry_bank_authority_acquired;
    bool entry_supervision_valid, entry_supervision_boundary_missed; EntrySupervisionMode entry_supervision_mode;
    double entry_supervision_bank_correction, entry_supervision_aoa_correction;

    double entry_supervision_ut;

    /* Live control defers expensive searches to the prediction worker. */
    bool entry_planning_deferred, entry_planning_needed, entry_lateral_infeasible, entry_committed_infeasible;
    bool terminal_planning_deferred; /* Live controller offloads only terminal candidate search. */
    EntryControlPlan entry_s_turn_plan;
    EntryTopologyPlan entry_topology;
    double entry_topology_capture_good_duration;
    EntryControlPlan taem_s_turn_plan;
    bool entry_predictor_models_valid;
    AerodynamicEnvelope entry_predictor_envelope;
    TrajectoryCalibrationModel entry_predictor_calibration;
    TerminalCandidate terminal_candidate;
    TaemInterfaceTarget taem_interface_target;
    bool taem_interface_captured, taem_safety_handoff;
    double taem_interface_diagnostic_ut;
    TerminalPathKind terminal_path_kind;
    bool terminal_prediction_valid, terminal_path_committed;
    bool hac_plan_degraded, hac_plan_geometry_degraded, hac_plan_energy_degraded;
    double hac_plan_violation_score, hac_commit_blend;
    double terminal_prediction_ut, terminal_reference_fpa, terminal_reference_heading;
    double terminal_reference_bank, terminal_reference_aoa, terminal_mix;
    /* The path provider's actual preview demand, kept separate from the
       high-level reference so diagnostics can distinguish a neutral lead
       segment from a neutral vehicle command. */
    double terminal_reference_path_lateral_acceleration;
    double terminal_reference_path_bank;
    double terminal_reference_path_course_error;
    double terminal_reference_path_arc_remaining;
    bool terminal_reference_path_transition_active;
    double terminal_candidate_live_energy_margin;
    bool terminal_candidate_live_energy_valid;
    double terminal_prediction_altitude, terminal_prediction_speed, terminal_prediction_time;
    double hac_side; bool hac_side_selected; double delivered_delta_v, burn_command_started_ut, burn_active_elapsed;
    double burn_progress_watch_ut, burn_progress_watch_delta_v;
    bool has_burn_command_started, has_burn_progress_watch, deorbit_burn_completed, atmospheric_interface_crossed, has_previous_ut;
    double previous_ut; bool final_approach_captured, airbrakes_deployed;
    bool hac_progress_valid, hac_captured, hac_completed, terminal_region_entered, terminal_glide_mode, terminal_rehearsal_mode, terminal_test_capture_active;
    double hac_capture_lost_duration, terminal_energy_mismatch_duration;
    double terminal_reentry_after_ut, hac_circuit_slope;
    unsigned hac_circuit_count;
    bool hac_transition_active;
    bool hac_transition_heading_cone;
    bool terminal_test_spiral_active;
    double hac_previous_angle, hac_remaining, hac_radius, minimum_turn_radius, terminal_test_glide_slope, terminal_test_final_approach_distance, final_invalid_duration;
    double hac_transition_p0_e, hac_transition_p0_n, hac_transition_p1_e, hac_transition_p1_n;
    double hac_transition_p2_e, hac_transition_p2_n, hac_transition_p3_e, hac_transition_p3_n;
    double hac_transition_cone_center_e, hac_transition_cone_center_n;
    double hac_transition_cone_start_angle, hac_transition_cone_end_angle, hac_transition_cone_arc_length;
    double hac_transition_lead_start_e, hac_transition_lead_start_n, hac_transition_lead_length, hac_transition_lead_progress;
    double hac_transition_length, hac_transition_end_angle, hac_transition_exit_speed, hac_transition_response_time, hac_transition_progress;
    double terminal_test_revolution_remaining;
    double terminal_test_preflare_altitude, terminal_test_preflare_target_speed, terminal_test_preflare_min_speed;
    TerminalVerticalStage terminal_vertical_stage;
    bool terminal_vertical_stage_valid, gear_command_latched, ground_contact_latched, flare_sink_captured;
    bool terminal_response_sample_valid, terminal_preflare_plan_valid;
    bool terminal_energy_sample_valid;
    double terminal_stage_started_ut, terminal_stage_good_duration, ground_contact_duration;
    double terminal_previous_aoa, terminal_previous_vertical_speed, terminal_response_sample_ut;
    double terminal_previous_specific_energy, terminal_previous_speed, terminal_energy_sample_ut;
    double terminal_energy_loss_accel_ema, terminal_speed_loss_accel_ema;
    double terminal_positive_aoa_rate_ema, terminal_sink_accel_ema, terminal_pitch_response_delay_ema;
    double preflare_trigger_altitude, preflare_target_aoa, preflare_target_sink;
    double preflare_minimum_speed, preflare_reference_speed, preflare_predicted_height_loss;
    double preflare_predicted_kinetic_margin, preflare_effective_accel;
    bool attitude_recovery; double control_bad_duration, control_good_duration, recovery_duration, recovery_heading, recovery_aoa;
    /* Roll-limit-cycle monitor.  Unlike the ordinary departure detector this
       tracks repeated high-energy relative-rate reversals even when each
       instantaneous aileron command is correctly signed as braking. */
    double previous_relative_roll_rate, roll_oscillation_score, roll_rate_excess_duration;
    bool has_previous_relative_roll_rate;
    double previous_sideslip; bool has_previous_sideslip;
    double entry_predictive_score, entry_reference_speed, entry_reference_altitude, entry_reference_range; bool has_entry_predictive_score;
    bool entry_control_plan_valid, entry_control_terminal_ready;
    uint64_t control_plan_sequence;
    double entry_control_plan_ut, entry_control_bank, entry_control_aoa, entry_control_heading;
    double entry_control_turn_radius, entry_control_segment_until_ut;
    double entry_control_cost, entry_control_taem_range_error, entry_control_taem_speed, entry_control_taem_energy_error;
    bool entry_reversal_scheduled, entry_reversal_is_final, entry_final_reversal_pending, entry_final_reversal_completed;
    bool entry_topology_heading_locked, entry_continuation_bootstrap;
    /* The MM304 inlet side is a mission-geometry choice.  It must not change
       merely because the energy-management S-turn changes ownership side. */
    bool entry_target_side_latched;
    double entry_target_side;
    double entry_reversal_ut, entry_reversal_range, entry_reversal_sign, entry_reversal_bank;
    unsigned entry_control_reversals;
    /* Per-tick Entry allocation evidence.  These values expose the physical
       demand before it is sent to the controller; they do not alter the
       executable command or grant any extra authority. */
    double entry_lateral_bank_magnitude, entry_geometry_bank,
        entry_vertical_bank_magnitude, entry_demand_bank,
        entry_lateral_required_bank;
    char abort_reason[384];
    RobustPID entry_energy_pid, taem_altitude_pid, final_altitude_pid, speed_pid, flare_sink_pid;
    SpeedbrakeController speedbrake_controller;
    JerkLimiter pitch_limiter, roll_limiter, heading_limiter; SlewLimiter throttle_limiter;
    GuidancePhase last_stabilized_phase; bool has_last_stabilized_phase;
} GuidanceMachine;

typedef struct {
    bool active; CalibrationRunState state; double start_ut, state_start_ut; double initial_heading;
    double angles[16]; int angle_count, angle_index;
    char status[256];
} GlideCalibrationMachine;

typedef struct {
    double ld[4], beta[4], confidence[4]; int accepted, rejected; double best_glide_aoa, stall_speed;
    VehicleProfile recommended;
} AdaptiveFlightCalibrator;

typedef struct {
    Trajectory forecast; TrajectoryCalibrationModel model; double previous_ut; bool has_previous;
} InFlightTrajectoryCalibrator;

/* Unified vessel physics: wind axes are drag-opposed, lift-up at zero bank,
   and cross(air direction, lift-up). Angular arrays: pitch, roll, yaw; deg/s². */
#define VESSEL_AERO_SAMPLES 192
typedef struct {
    double q, mach, aoa, beta, mass;
    Vector3 force_per_q; /* signed wind-axis force / Pa, preserves side force */
    bool gear, brakes;
    int airbrakes; /* -1 unknown/legacy, 0 retracted, 1 deployed */
    /* Number of independent prior flight/timeline contributors represented by
       this certified cell. It is statistical support, not frame count. */
    unsigned observations;
    double trust; /* provenance/reliability weight in the normalized [0,1] domain */
    double last_observation_ut; /* current-flight observation timestamp */
} VesselAeroSample;
struct VesselPhysicsModel {
    VehicleState state;
    Vector3 center_of_mass, lift_vector, drag_vector, center_of_mass_root;
    bool has_center_of_mass_root;
    bool has_state, has_center_of_mass, gear, brakes;
    int airbrakes;
    double thrust, available_thrust, throttle, mass_flow, propellant_per_impulse;
    double authority[3], authority_confidence[3], previous_rate[3], previous_control[3];
    double torque[3], inertia[3];
    bool previous_rate_valid[3];
    double authority_q, sideslip, confidence;
    double model_residual, model_residual_confidence, certified_uncertainty, certified_confidence;
    double last_residual_ut; bool has_last_residual_ut;
    Vector3 force_residual_per_q, force_residual_variance_per_q;
    double force_residual_confidence;
    double dry_mass;
    unsigned count, next, accepted, rejected, live_observations;
    VesselAeroSample samples[VESSEL_AERO_SAMPLES];
};
void vessel_physics_init(VesselPhysicsModel *m);
void vessel_physics_import_sample(VesselPhysicsModel *m,const VesselAeroSample *sample);
void vessel_physics_observe(VesselPhysicsModel *m, Telemetry *t, const VehicleState *s, const PlanetModel *p);
bool vessel_physics_aero(const VesselPhysicsModel *m,double q,double mach,double aoa,double beta,double mass,Vector3 *specific,double *confidence);
bool vessel_physics_aero_config(const VesselPhysicsModel *m,double q,double mach,double aoa,double beta,double mass,bool gear,bool brakes,int airbrakes,Vector3 *specific,double *confidence);
Vector3 vessel_physics_gravity(Vector3 position,const PlanetModel *p);
Vector3 vessel_physics_force(const VesselPhysicsModel *m,double q,double mach,double aoa,double beta,double mass,AerodynamicModel fallback,const TrajectoryCalibrationModel *cal,const VehicleProfile *v,double *confidence,bool *observed);
Vector3 vessel_physics_force_config(const VesselPhysicsModel *m,double q,double mach,double aoa,double beta,double mass,bool gear,bool brakes,int airbrakes,AerodynamicModel fallback,const TrajectoryCalibrationModel *cal,const VehicleProfile *v,double *confidence,bool *observed);
Vector3 vessel_physics_force_best_estimate_config(const VesselPhysicsModel *m,double q,double mach,double aoa,double beta,double mass,bool gear,bool brakes,int airbrakes,AerodynamicModel fallback,const TrajectoryCalibrationModel *cal,const VehicleProfile *v,double *confidence,bool *observed,double *relative_uncertainty);
bool vessel_physics_derive_envelope(VesselPhysicsModel *m,const VehicleProfile *baseline,AerodynamicEnvelope *envelope,AerodynamicModel *planning);
Vector3 vessel_physics_acceleration(Vector3 position,Vector3 air_velocity,const PlanetModel *p,Vector3 specific_force,double bank);
void vessel_physics_environment(const PlanetModel *p,const TrajectoryCalibrationModel *cal,double altitude,double *density,double *sound);
double vessel_physics_conservative_stall_fraction(double true_air_speed,double angle_of_attack,double dynamic_pressure,const VehicleProfile *vehicle);
double vessel_physics_burn_mass(const VesselPhysicsModel *m,double mass,double thrust,double dt);
void vessel_physics_axis_step(const VesselPhysicsModel *m,int axis,double target,double q,double rate_limit,double dt,double *angle,double *rate);
void landing_snapshot_set_sample(LandingSnapshot *snapshot,const Telemetry *t,const VehicleState *state);

/* math/navigation */
double clampd(double v, double lo, double hi); double norm_deg(double a); double norm_signed_deg(double a);
double entry_guidance_start_altitude(const PlanetModel *planet,const GuidanceSettings *settings);
void entry_taem_handoff_altitude_bounds(const GuidanceSettings *settings,double *minimum,double *maximum);
bool entry_taem_handoff_geometry_ready(double altitude,double runway_along_track,double vertical_speed,double horizontal_speed,const GuidanceSettings *settings);
bool entry_s_turn_bank_authority_available(double dynamic_pressure,double true_air_speed,double stall_fraction,double g_force,const VehicleProfile *vehicle);
double entry_bank_authority_limit(double true_air_speed,double dynamic_pressure,double g_force,const VehicleProfile *vehicle,double maximum_bank);
double entry_taem_range_target(const PlanetModel *planet,const GuidanceSettings *settings);
double rotating_specific_energy(double latitude,double altitude,double air_relative_speed,const PlanetModel *planet);
double entry_remaining_specific_energy(double latitude,double altitude,double air_relative_speed,double target_latitude,double target_altitude,double target_speed,const PlanetModel *planet);
double entry_altitude_target_for_speed(double reference_altitude,double reference_speed,double true_air_speed,double target_altitude,double target_speed);
double entry_altitude_target_for_range(double reference_altitude,double reference_range,double range,double target_altitude,double target_range);
double entry_thermal_protection_aoa_floor(const VehicleProfile *vehicle);
double entry_low_q_protective_aoa_floor(double dynamic_pressure,const VehicleProfile *vehicle);
double entry_terminal_turn_aoa_floor(double true_air_speed,double dynamic_pressure,const VehicleProfile *vehicle);
double entry_final_s_turn_aoa_ceiling(const VehicleProfile *vehicle);
EntryTerminalDemand entry_terminal_demand(double latitude,double altitude,double range,double horizontal_speed,double course_error,double vertical_speed,double true_air_speed,double entry_reference_speed,const PlanetModel *planet,const VehicleProfile *vehicle,const LandingSite *site,const GuidanceSettings *settings);
Vector3 v3(double x,double y,double z); Vector3 vadd(Vector3 a,Vector3 b); Vector3 vsub(Vector3 a,Vector3 b);
Vector3 vscale(Vector3 a,double s); double vdot(Vector3 a,Vector3 b); Vector3 vcross(Vector3 a,Vector3 b);
double vmag(Vector3 a); Vector3 vnorm(Vector3 a, Vector3 fallback); Vector3 vproject_plane(Vector3 a, Vector3 normal);
Vector3 vrotate(Vector3 a, Vector3 axis, double radians);
double great_circle_distance(GeoPoint a, GeoPoint b, double radius); double initial_bearing(GeoPoint a, GeoPoint b);
GeoPoint destination_point(GeoPoint a,double bearing,double distance,double radius); void local_offsets(GeoPoint origin,GeoPoint p,double radius,double *east,double *north);
GeoPoint runway_approach_aimpoint(const LandingSite *site,double radius,double distance_before_threshold);
GeoPoint local_point(GeoPoint origin,double east,double north,double radius,double altitude); double surface_course(Vector3 pos,Vector3 vel,Vector3 omega,Vector3 north,double fallback);
void runway_coordinates(GeoPoint point, GeoPoint site, double heading, double radius, double *along, double *cross);
double taem_interface_line_heading(double runway_along,double runway_cross,double runway_heading,
    const TaemInterfaceTarget *target);
double taem_course_rate_bank_command(double horizontal_speed,double lift_accel,double bank_effectiveness,
    double bank_limit,double current_course,double desired_course,double response_time);
LandingSite runway_reciprocal_site(const LandingSite *primary,double radius);
void telemetry_reframe_runway(Telemetry *telemetry,const LandingSite *site,double radius);
double runway_end_acquisition_score(const Telemetry *telemetry,const LandingSite *site,const GuidanceSettings *settings,double radius);
double bank_for_point_capture(double speed,double lift_accel,double bank_effectiveness,double course_error,double distance,double maximum_bank);
HACGuidance hac_guidance_compute(GeoPoint current,double true_air_speed,double course,const LandingSite *site,const GuidanceSettings *settings,double radius,double side,double gravity);
double hac_guidance_score(GeoPoint current,double true_air_speed,double course,const LandingSite *site,const GuidanceSettings *settings,double radius,double side,double gravity);
HACGuidance hac_guidance_compute_radius(GeoPoint current,double true_air_speed,double course,const LandingSite *site,const GuidanceSettings *settings,double planet_radius,double side,double gravity,double hac_radius);
double hac_guidance_score_radius(GeoPoint current,double true_air_speed,double course,const LandingSite *site,const GuidanceSettings *settings,double planet_radius,double side,double gravity,double hac_radius);
bool hac_entry_capture_geometry_ready(const HACGuidance *guidance,double hac_radius);
bool hac_high_pass_circuit(double altitude, double speed, double hac_radius, double remaining,
    double minimum_turn_radius, double drag_accel, const PlanetModel *planet,
    const LandingSite *site, const VehicleProfile *vehicle, const GuidanceSettings *settings,
    double *new_remaining, double *glide_slope);
bool taem_alignment_maneuver_geometry(double altitude, double radius,
    double minimum_turn_radius, double alignment_turn_deg, const LandingSite *site,
    const GuidanceSettings *settings, double *remaining_path, double *glide_slope,
    double *final_altitude);
bool taem_alignment_maneuver_feasible(double altitude, double speed, double radius,
    double minimum_turn_radius, double drag_accel, double alignment_turn_deg, const PlanetModel *planet,
    const LandingSite *site, const VehicleProfile *vehicle, const GuidanceSettings *settings,
    double *remaining_path, double *glide_slope);
bool terminal_approach_valid(GeoPoint current,double range_to_site,double runway_along,double runway_cross,double course,double true_air_speed,double horizontal_speed,double vertical_speed,double flight_path_angle,const LandingSite *site,const VehicleProfile *vehicle,const GuidanceSettings *settings);
void robust_pid_init(RobustPID *p,double kp,double ki,double kd,double limit,double tc); void robust_pid_reset(RobustPID *p); double robust_pid_update(RobustPID *p,double error,double dt,double lo,double hi);
double lowpass_update(LowPass *f,double value,double dt); void lowpass_reset(LowPass *f);
double jerk_update(JerkLimiter *j,double target,double max_rate,double max_accel,double dt); double jerk_angle_update(JerkLimiter *j,double target,double max_rate,double max_accel,double dt);
double burn_fraction(double elapsed,double remaining,double max_accel,double ramp); double burn_estimated_duration(double dv,double max_accel,double ramp);
FlightControlRegime flight_control_regime(double mach,double true_air_speed,double dynamic_pressure,double minimum_safe_speed);
double entry_s_turn_effective_minimum_leg(double true_air_speed,double taem_speed,const GuidanceSettings *settings);
double entry_s_turn_fpa_delivery_debt(double flight_path_angle,const TaemInterfaceTarget *target);
double entry_s_turn_reversal_corridor(double base_corridor,double range_to_site,double terminal_join_range,double altitude_debt,double fpa_delivery_debt);
void speedbrake_controller_reset(SpeedbrakeController *c,bool deployed);
bool speedbrake_controller_update(SpeedbrakeController *c,double dynamic_pressure,double target_dynamic_pressure,double specific_energy_error,double energy_scale,double maximum_dynamic_pressure,bool inhibit,double dt);

/* models/serialization */
LandingConfiguration landing_configuration_default(void); void landing_configuration_normalize(LandingConfiguration *c);
bool landing_configuration_from_json(LandingConfiguration *c,const JsonDoc *doc,int index); void landing_configuration_json(JsonWriter *w,const LandingConfiguration *c); void planet_model_replay_json(JsonWriter *w,const PlanetModel *p);
void telemetry_init(Telemetry *t); void guidance_command_init(GuidanceCommand *c); void calibration_snapshot_init(CalibrationSnapshot *c); void landing_snapshot_init(LandingSnapshot *s,const VehicleProfile *p);
const char *connection_status_string(ConnectionStatus s); const char *phase_string(GuidancePhase p); const char *speed_mode_string(NavballSpeedMode s); const char *profile_string(ControlProfile p); const char *cal_state_string(CalibrationRunState s);
void trajectory_init(Trajectory *t); void trajectory_clear(Trajectory *t); bool trajectory_append(Trajectory *t,TrajectoryPoint p); bool trajectory_copy(Trajectory *dst,const Trajectory *src);
void deorbit_plan_clear(DeorbitPlan *p); void snapshot_json(JsonWriter *w,const LandingSnapshot *s); void snapshot_log_json(JsonWriter *w,const LandingSnapshot *s); void trajectory_json(JsonWriter *w,const Trajectory *t);

/* prediction/guidance */
Vector3 planet_rotation_vector(const PlanetModel *p); double planet_surface_gravity(const PlanetModel *p);
double planet_atmospheric_density(const PlanetModel *p,double altitude);
double planet_atmospheric_pressure(const PlanetModel *p,double altitude);
double planet_atmospheric_speed_of_sound(const PlanetModel *p,double altitude);
void aerodynamic_force_factors(double angle_of_attack,const VehicleProfile *vehicle,double *lift_factor,double *drag_factor);
void aerodynamic_force_factors_mach(double mach,double angle_of_attack,const VehicleProfile *vehicle,double *lift_factor,double *drag_factor);
GeoPoint predictor_geo_point(Vector3 position,const PlanetModel *planet,double ut); Vector3 predictor_inertial_position(GeoPoint point,const PlanetModel *planet,double ut);
VehicleState predictor_propagate_vacuum(VehicleState state,double target_ut,const PlanetModel *planet,double max_step);
double predictor_postburn_periapsis(VehicleState state,const PlanetModel *planet);
double predictor_directed_taem_range_error(double range,double closest_distance,double target_range);
double entry_taem_speed_target(const VehicleProfile *vehicle,const GuidanceSettings *settings,const PlanetModel *planet);
void entry_publish_taem_tangent_target(GuidanceMachine *guidance,const Telemetry *telemetry,double course,const PlanetModel *planet,AerodynamicModel aero,const LandingConfiguration *cfg);
bool entry_taem_shaping_crossrange_ready(const TaemInterfaceTarget *target,const Telemetry *telemetry,const PlanetModel *planet,const LandingConfiguration *cfg,double next_sign,double *built_cross,double *required_cross);
EntryPrediction predictor_simulate_entry(VehicleState state,const PlanetModel *planet,AerodynamicModel aero,const AerodynamicEnvelope *env,const TrajectoryCalibrationModel *cal,const VehicleProfile *vehicle,const LandingSite *site,const GuidanceSettings *settings,double max_bank,double initial_bank,double initial_bank_sign,double initial_leg_elapsed,double max_duration,bool include_trajectory);
EntryPrediction predictor_simulate_entry_with_rate(VehicleState state,const PlanetModel *planet,AerodynamicModel aero,const AerodynamicEnvelope *env,const TrajectoryCalibrationModel *cal,const VehicleProfile *vehicle,const LandingSite *site,const GuidanceSettings *settings,double max_bank,double initial_bank,double initial_bank_rate,double initial_bank_sign,double initial_leg_elapsed,double max_duration,bool include_trajectory);
EntryPrediction predictor_simulate_entry_with_attitude(VehicleState state,const PlanetModel *planet,AerodynamicModel aero,const AerodynamicEnvelope *env,const TrajectoryCalibrationModel *cal,const VehicleProfile *vehicle,const LandingSite *site,const GuidanceSettings *settings,double max_bank,double initial_bank,double initial_bank_rate,double initial_angle_of_attack,double initial_angle_of_attack_rate,double initial_bank_sign,double initial_leg_elapsed,double max_duration,bool include_trajectory);
EntryControlPlan predictor_plan_entry_control(VehicleState state,const PlanetModel *planet,AerodynamicModel aero,const AerodynamicEnvelope *env,const TrajectoryCalibrationModel *cal,const VehicleProfile *vehicle,const LandingSite *site,const GuidanceSettings *settings,double initial_bank,double initial_bank_rate,double initial_angle_of_attack,double initial_angle_of_attack_rate,double current_bank_sign,bool current_leg_established,double current_leg_elapsed,double current_commit_remaining,bool s_turn_side_locked,bool final_heading_lock,double max_duration);
EntryControlPlan predictor_plan_entry_control_to_interface(VehicleState state,const PlanetModel *planet,AerodynamicModel aero,const AerodynamicEnvelope *env,const TrajectoryCalibrationModel *cal,const VehicleProfile *vehicle,const LandingSite *site,const GuidanceSettings *settings,const TaemInterfaceTarget *interface_target,bool enforce_terminal_delivery_budget,double initial_bank,double initial_bank_rate,double initial_angle_of_attack,double initial_angle_of_attack_rate,double current_bank_sign,bool current_leg_established,double current_leg_elapsed,double current_commit_remaining,bool s_turn_side_locked,bool final_heading_lock,double max_duration);
EntryPrediction predictor_simulate_entry_control_plan(VehicleState state,const PlanetModel *planet,AerodynamicModel aero,const AerodynamicEnvelope *env,const TrajectoryCalibrationModel *cal,const VehicleProfile *vehicle,const LandingSite *site,const GuidanceSettings *settings,double initial_bank,double initial_bank_rate,double initial_angle_of_attack,double initial_angle_of_attack_rate,double current_bank_sign,double current_leg_elapsed,const EntryControlPlan *plan,double max_duration,bool include_trajectory);
EntryPrediction predictor_simulate_entry_control_plan_to_interface(VehicleState state,const PlanetModel *planet,AerodynamicModel aero,const AerodynamicEnvelope *env,const TrajectoryCalibrationModel *cal,const VehicleProfile *vehicle,const LandingSite *site,const GuidanceSettings *settings,const TaemInterfaceTarget *interface_target,bool enforce_terminal_delivery_budget,double initial_bank,double initial_bank_rate,double initial_angle_of_attack,double initial_angle_of_attack_rate,double current_bank_sign,double current_leg_elapsed,const EntryControlPlan *plan,double max_duration,bool include_trajectory);
EntryPrediction predictor_simulate_entry_guidance_shadow(VehicleState state,const Telemetry *telemetry,const GuidanceMachine *guidance,const DeorbitPlan *plan,const PlanetModel *planet,AerodynamicModel aero,const AerodynamicEnvelope *env,const TrajectoryCalibrationModel *cal,const LandingConfiguration *cfg,double max_duration,bool include_trajectory);
EntryPrediction predictor_simulate_terminal_shadow(VehicleState state,const Telemetry *telemetry,const GuidanceMachine *guidance,const DeorbitPlan *plan,const PlanetModel *planet,AerodynamicModel aero,const AerodynamicEnvelope *env,const TrajectoryCalibrationModel *cal,const LandingConfiguration *cfg,double max_duration,bool include_trajectory);
EntryPrediction predictor_simulate_terminal_shadow_ensemble(VehicleState state,const Telemetry *telemetry,const GuidanceMachine *guidance,const DeorbitPlan *plan,const PlanetModel *planet,AerodynamicModel aero,const AerodynamicEnvelope *env,const TrajectoryCalibrationModel *cal,const LandingConfiguration *cfg,double max_duration,bool include_trajectory);
EntryPredictionAssessment predictor_assess_entry_prediction(const EntryPrediction *prediction,const VehicleProfile *vehicle,double available_bank,const EntryControlPlan *plan);
EntrySupervisionResult predictor_supervise_entry_control_to_interface(VehicleState state,const PlanetModel *planet,AerodynamicModel aero,const AerodynamicEnvelope *env,const TrajectoryCalibrationModel *cal,const VehicleProfile *vehicle,const LandingSite *site,const GuidanceSettings *settings,const TaemInterfaceTarget *interface_target,bool enforce_terminal_delivery_budget,double initial_bank,double initial_bank_rate,double initial_angle_of_attack,double initial_angle_of_attack_rate,double current_bank_sign,bool current_leg_established,double current_leg_elapsed,double current_commit_remaining,bool s_turn_side_locked,bool final_heading_lock,const EntryControlPlan *nominal_plan,double maximum_bank_correction,double maximum_aoa_correction,double max_duration);
EntrySupervisionResult predictor_supervise_entry_control(VehicleState state,const PlanetModel *planet,AerodynamicModel aero,const AerodynamicEnvelope *env,const TrajectoryCalibrationModel *cal,const VehicleProfile *vehicle,const LandingSite *site,const GuidanceSettings *settings,double initial_bank,double initial_bank_rate,double initial_angle_of_attack,double initial_angle_of_attack_rate,double current_bank_sign,bool current_leg_established,double current_leg_elapsed,double current_commit_remaining,bool s_turn_side_locked,bool final_heading_lock,const EntryControlPlan *nominal_plan,double maximum_bank_correction,double maximum_aoa_correction,double max_duration);
bool predictor_last_planner_trace(PredictorPlannerTrace *out);
void entry_prediction_clear(EntryPrediction *p);
bool reentry_guidance_shadow_recovery_qualified(const EntryPrediction *prediction,const VehicleProfile *vehicle,const GuidanceSettings *settings);
bool deorbit_plan_create(DeorbitPlan *out,VehicleState state,double orbit_period,const PlanetModel *planet,AerodynamicModel aero,const AerodynamicEnvelope *env,const TrajectoryCalibrationModel *cal,const LandingSite *site,const VehicleProfile *vehicle,const GuidanceSettings *settings,double available_thrust);
bool deorbit_plan_preburn_state_compatible(const DeorbitPlan *plan,double live_mass,double live_available_thrust,const GuidanceSettings *settings);
bool deorbit_capture_qualified(const EntryPrediction *p,const LandingSite *site,const VehicleProfile *vehicle,const GuidanceSettings *settings,double postburn_periapsis,bool require_entry_corridor);
bool deorbit_recovery_qualified(const EntryPrediction *p,const LandingSite *site,const VehicleProfile *vehicle,const GuidanceSettings *settings,double postburn_periapsis);

bool guidance_plan_terminal_preview(GuidanceMachine *g,const Telemetry *t,const PlanetModel *p,
    AerodynamicModel aero,const LandingConfiguration *cfg);
bool guidance_accept_terminal_preview(GuidanceMachine *g,const GuidanceMachine *request,
    const GuidanceMachine *result,const Telemetry *t,const LandingConfiguration *cfg);
bool guidance_accept_entry_plan(GuidanceMachine *g,const GuidanceMachine *request,
    const GuidanceMachine *result,const Telemetry *t,const LandingConfiguration *cfg);
void guidance_update_entry_reversal(GuidanceMachine *machine,const EntryControlPlan *plan,double ut);
void guidance_machine_init(GuidanceMachine *g); void guidance_set_engaged(GuidanceMachine *g,bool engaged); void guidance_set_paused(GuidanceMachine *g,bool paused); void guidance_abort(GuidanceMachine *g); void guidance_reset_plan(GuidanceMachine *g); double guidance_entry_leg_elapsed(const GuidanceMachine *g,double ut);
void guidance_set_entry_predictor_models(GuidanceMachine *g,const AerodynamicEnvelope *env,const TrajectoryCalibrationModel *cal);
void guidance_initialize_reentry_continuation(GuidanceMachine *g,const Telemetry *t,const PlanetModel *planet,const LandingConfiguration *cfg,double initial_s_turn_sign,bool late_terminal_test,const AerodynamicEnvelope *env,const TrajectoryCalibrationModel *cal);
bool guidance_begin_hac_test(GuidanceMachine *g,const Telemetry *t,double course,const PlanetModel *planet,AerodynamicModel aero,const LandingConfiguration *cfg,char *message,size_t message_size);
GuidanceResult guidance_update(GuidanceMachine *g,const Telemetry *t,const VehicleState *state,const DeorbitPlan *plan,const PlanetModel *planet,AerodynamicModel aero,const LandingConfiguration *cfg);
EntryControlPlan guidance_terminal_control_plan(const GuidanceMachine *g,EntryControlPlan plan);
void guidance_result_clear(GuidanceResult *r); void reference_trajectory(Trajectory *out,const LandingSite *site,const GuidanceSettings *settings,double radius,double hac_side);

/* calibration */
void adaptive_calibrator_init(AdaptiveFlightCalibrator *c,const VehicleProfile *profile); void adaptive_calibrator_reset(AdaptiveFlightCalibrator *c,const VehicleProfile *profile);
void adaptive_calibrator_update(AdaptiveFlightCalibrator *c,const Telemetry *t,const CalibrationSettings *settings,const VehicleProfile *baseline,const GuidanceCommand *command,double dt,bool allow_adaptation,bool active,bool sampling,CalibrationRunState run_state,double progress,double target_aoa,const char *run_status,const char *warning,AerodynamicModel *current,AerodynamicModel *planning,AerodynamicEnvelope *envelope,VehicleProfile *recommended,CalibrationSnapshot *snapshot);
void glide_calibration_init(GlideCalibrationMachine *g); void glide_calibration_start(GlideCalibrationMachine *g,const Telemetry *t,const CalibrationSettings *settings); void glide_calibration_stop(GlideCalibrationMachine *g,const char *status); GuidanceResult glide_calibration_update(GlideCalibrationMachine *g,const Telemetry *t,const VehicleProfile *profile,const CalibrationSettings *settings,bool *sampling,bool *finished,double *progress,double *target_aoa);
void trajectory_calibrator_init(InFlightTrajectoryCalibrator *c); void trajectory_calibrator_clear(InFlightTrajectoryCalibrator *c); void trajectory_calibrator_set_forecast(InFlightTrajectoryCalibrator *c,const Trajectory *forecast); TrajectoryCalibrationModel trajectory_calibrator_update(InFlightTrajectoryCalibrator *c,const Telemetry *t,const VehicleState *state,const PlanetModel *planet,const LandingSite *site,const AerodynamicEnvelope *env,const VehicleProfile *vehicle,const GuidanceCommand *command,const CalibrationSettings *settings,bool allow_adaptation,double dt);

typedef struct {
    unsigned read_calls, read_wire_requests, apply_calls, apply_wire_requests;
    unsigned total_calls, total_wire_requests;
    double read_seconds, apply_seconds;
} KRPCTransportBudget;

/* native kRPC C-Nano runtime */
KRPCSession *krpc_session_open(const LandingConfiguration *cfg,char *error,size_t error_size); void krpc_session_close(KRPCSession *s); const PlanetModel *krpc_session_planet(const KRPCSession *s); const char *krpc_session_vessel(const KRPCSession *s);
const char *krpc_session_transport(const KRPCSession *s); const char *krpc_session_library_version(const KRPCSession *s); KRPCTransportBudget krpc_session_transport_budget(const KRPCSession *s); bool krpc_session_is_simulator(const KRPCSession *s);
const char *krpc_session_physics_structure_id(const KRPCSession *s); const char *krpc_session_physics_environment_id(const KRPCSession *s); const char *krpc_session_physics_storage(const KRPCSession *s); unsigned krpc_session_physics_history_count(const KRPCSession *s);
void krpc_session_seed_physics(const KRPCSession *s,VesselPhysicsModel *model);
bool krpc_orbital_up_reference(const GuidanceCommand *command,const VehicleState *state,Vector3 *out);
bool krpc_read_telemetry(KRPCSession *s,const LandingConfiguration *cfg,Telemetry *t,VehicleState *state,char *error,size_t error_size); bool krpc_apply(KRPCSession *s,const GuidanceCommand *command,unsigned airbrake_group,const char *phase,const char *status,const char *warning,const Trajectory *hud_predicted_trajectory,const Trajectory *hud_reference_trajectory,KRPCApplyResult *result,char *error,size_t error_size); void krpc_safe(KRPCSession *s); bool krpc_set_gear(KRPCSession *s,bool value,char *error,size_t error_size); bool krpc_set_brakes(KRPCSession *s,bool value,char *error,size_t error_size); bool krpc_save_game(KRPCSession *s,const char *name,char *error,size_t error_size); bool krpc_warp(KRPCSession *s,double ut,char *error,size_t error_size);

/* controller */
typedef void (*SnapshotCallback)(const LandingSnapshot *snapshot, void *context);
LandingController *landing_controller_create(const LandingConfiguration *cfg,SnapshotCallback callback,void *context); void landing_controller_destroy(LandingController *c); void landing_controller_update_configuration(LandingController *c,const LandingConfiguration *cfg); LandingConfiguration landing_controller_configuration(LandingController *c);
void landing_controller_connect(LandingController *c); void landing_controller_disconnect(LandingController *c); void landing_controller_create_plan(LandingController *c); void landing_controller_engage(LandingController *c); void landing_controller_engage_reentry(LandingController *c); void landing_controller_engage_hac_test(LandingController *c); void landing_controller_start_calibration(LandingController *c); void landing_controller_stop_calibration(LandingController *c,bool apply); void landing_controller_reset_calibration(LandingController *c); void landing_controller_set_paused(LandingController *c,bool paused); void landing_controller_abort(LandingController *c); void landing_controller_set_gear(LandingController *c,bool deployed); void landing_controller_set_brakes(LandingController *c,bool enabled); bool landing_controller_save_checkpoint(LandingController *c,const char *name,char *error,size_t error_size); void landing_controller_shutdown(LandingController *c);

/* Coupled one-reversal Entry topology. Search only on the prediction worker. */
EntryTopologyPlan predictor_plan_entry_topology(VehicleState state,const Telemetry *telemetry,const GuidanceMachine *guidance,
    const PlanetModel *planet,const AerodynamicEnvelope *envelope,
    const TrajectoryCalibrationModel *calibration,const LandingConfiguration *configuration);
bool guidance_install_entry_topology(GuidanceMachine *guidance,const EntryTopologyPlan *plan,double ut);
GuidanceCommand guidance_entry_reference_step(GuidanceMachine *guidance,const Telemetry *telemetry,
    const VehicleProfile *vehicle,const GuidanceSettings *settings,double bank,double aoa,double dt);
#endif
