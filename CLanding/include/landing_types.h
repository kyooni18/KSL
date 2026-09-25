#ifndef KSP_LANDER_TYPES_H
#define KSP_LANDER_TYPES_H

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
/* Keep the full ShuttleSim fitted atmosphere in the shared PlanetModel.  The
   current Kerbin table is sampled every 100 m through the explicit 70 km vacuum
   boundary (701 data rows); truncating it changes the density/drag model used by
   guidance prediction relative to the simulator. */
#define LANDER_ATMOSPHERE_SAMPLE_MAX 1024
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
    char rpc_host[256];
    int rpc_port;
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
    /* Subsonic CL-max incidence: terminal lift requests saturate here rather
       than on the post-stall side of maximum_angle_of_attack. */
    double terminal_maximum_lift_angle_of_attack;
    bool allow_powered_approach; double maximum_approach_throttle; unsigned airbrake_action_group;
} VehicleProfile;
typedef struct {
    double target_deorbit_capture_radius, entry_interface_altitude_margin, target_entry_range;
    double target_entry_flight_path_angle, maximum_entry_flight_path_angle, target_post_burn_periapsis_altitude;
    double deorbit_maximum_throttle, deorbit_throttle_ramp_duration, taem_interface_altitude, taem_interface_range;
    double taem_force_handoff_speed;
    double mm304_handoff_radius, mm304_perpendicular_heading_half_width;
    double mm304_handoff_along_track, mm304_handoff_cross_track;
    /* MM305 begins at the high-energy TAEM acquisition interface.
       HAC acquisition is a later downstream event inside MM305 ownership. */
    double mm305_min_altitude, mm305_max_altitude;
    double mm305_target_mach, mm305_mach_half_width;
    double hac_acquisition_altitude, hac_acquisition_mach;
    double deorbit_timing_uncertainty, deorbit_thrust_uncertainty_fraction, deorbit_delta_v_uncertainty;
    double deorbit_mass_uncertainty_fraction, deorbit_position_uncertainty, deorbit_velocity_uncertainty;
    double deorbit_pointing_uncertainty, deorbit_atmosphere_uncertainty_fraction, deorbit_robust_minimum_pass_fraction;
    double hac_radius, hac_look_ahead_angle, final_approach_distance, final_alignment_speed, final_glide_slope, taem_glide_slope;
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
    bool gear, brakes, has_airbrakes, airbrakes;
    /* Live kRPC wheel-contact telemetry. `main_gear_grounded` is true when
       either rear/main gear reports ModuleWheelBase.isGrounded. */
    bool has_main_gear_grounded, main_gear_grounded;
    char vessel_situation[64];
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

/* Authoritative live MM304 feedback state.  Prediction may estimate these
   quantities, but only the guidance cycle initialized from measured telemetry
   publishes this record. Distances are metres; accelerations are m/s^2. */
typedef struct {
    bool valid, lateral_valid, reversal_requested;
    double ut, confidence;
    double measured_drag_accel, reference_drag_accel, drag_error_accel;
    double predicted_achievable_range, required_range_to_go, energy_margin, range_error_scale;
    double required_vertical_lift_accel;
    double crossrange_error, crossrange_rate, projected_crossrange_error, crossrange_corridor;
    double energy_bank_magnitude_deg, commanded_bank_deg, bank_sign;
} EntryGuidanceFeedback;

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
    EntryGuidanceFeedback guidance_entry_feedback;
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
    double guidance_terminal_candidate_tracking_error;
    double guidance_terminal_candidate_tracking_course_error;
    double guidance_terminal_candidate_tracking_endpoint_error;
    double guidance_terminal_candidate_tracking_margin;
    bool guidance_terminal_candidate_tracking_evaluated;
    bool guidance_terminal_candidate_path_degraded;
    bool guidance_terminal_candidate_control_degraded;
    bool guidance_terminal_candidate_rate_degraded;
    bool guidance_terminal_candidate_end_degraded;
    double guidance_terminal_reference_fpa, guidance_terminal_mix;
    double guidance_hac_remaining, guidance_hac_radius, guidance_minimum_turn_radius;
    double guidance_hac_transition_progress, guidance_commanded_course_rate_estimate;
    double guidance_hac_transition_points[8];
    bool guidance_hac_transition_heading_cone, guidance_hac_transition_lead_curve;
    double guidance_hac_transition_lead_points[6];
    double guidance_hac_transition_cone_center[2];
    double guidance_hac_transition_cone_start_angle;
    double guidance_hac_transition_cone_end_angle;
    double guidance_hac_transition_cone_arc_length;
    unsigned guidance_hac_circuit_count;
    double guidance_hac_circuit_slope, guidance_terminal_reentry_after_ut;
    char decision_reason[384];
} LandingSnapshot;


#endif
