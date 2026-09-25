#ifndef KSP_LANDER_GUIDANCE_TYPES_H
#define KSP_LANDER_GUIDANCE_TYPES_H

#include "landing_types.h"
#include "taem_route.h"

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
       projection can certify MM304 delivery into the MM305 engagement band.
       This flag is the inlet seed only; propagated topology and live capture
       independently re-prove the complete downstream maneuver/energy contract. */
    bool energy_qualified;
    double along_track, cross_track, course;
    double altitude, speed, flight_path_angle, specific_energy;
    double selected_ut, arrival_ut, quality_score;
    double acquisition_lead, remaining_path, response_time;
    /* Frozen runway-anchored HAC radius owned by this inlet.  MM304 must aim
       at the inlet of this exact circle; MM305 must not silently replace it
       with the configured comfort radius after ownership transfers. */
    double hac_radius;
} TaemInterfaceTarget;

/* Shared by live ownership and entry prediction: one acquisition boundary. */
typedef struct {
    double horizontal_radius_m;
    double perpendicular_heading_half_width_deg;
} TaemHandoffContract;

TaemHandoffContract taem_handoff_contract(const GuidanceSettings *settings);
void mm304_handoff_station(const GuidanceSettings *settings,double *along_m,double *cross_m);

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
    bool taem_dynamic_interface_captured, taem_dynamic_interface_target_valid;
    double entry_range, entry_flight_path_angle, entry_speed, entry_course_error, taem_speed, taem_flight_path_angle;
    double taem_altitude, taem_desired_altitude, taem_along_track, taem_cross_track, taem_course;
    bool mm304_gate_recorded;
    double mm304_gate_range, mm304_gate_along_track, mm304_gate_cross_track;
    double mm304_gate_course, mm304_gate_altitude, mm304_gate_speed, mm304_gate_flight_path_angle;
    double taem_energy_error, taem_interface_error;
    double taem_interface_energy_margin;
    double peak_dynamic_pressure, peak_g_load;
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

typedef enum {
    ENTRY_SUPERVISION_PASS_THROUGH = 0,
    ENTRY_SUPERVISION_BOUNDED_CORRECTION = 1,
    ENTRY_SUPERVISION_CONSERVATIVE_FALLBACK = 2,
    ENTRY_SUPERVISION_INFEASIBLE = 3
} EntrySupervisionMode;

typedef struct {
    GuidancePhase phase; GuidanceCommand command; char status[384], warning[384]; bool has_warning; Trajectory reference;
} GuidanceResult;

typedef struct { double e,n; } HACPoint2;

typedef enum {
    TERMINAL_PATH_NONE = 0,
    /* Value 2 is retained for telemetry compatibility with earlier logs. */
    TERMINAL_PATH_HAC = 2
} TerminalPathKind;


typedef struct {
    GuidancePhase phase; bool automation_engaged, paused, aborted;
    bool diagnostic_shadow; /* Prediction rollouts must not masquerade as live decisions. */
    bool diagnostic_stop_at_taem; /* Boundary-only entry supervision stops before MM305 route search. */
    double s_turn_sign; double s_turn_leg_started_ut; bool has_s_turn_leg_started;
    double s_turn_reversal_requested_ut; bool has_s_turn_reversal_requested;
    EntryExecutive entry_exec; EntryLateralState entry_lateral; TaemExecutive taem_exec;
    bool entry_alpha_has_target, entry_alpha_has_modulation, entry_drag_ratio_valid;
    double entry_alpha_target, entry_alpha_modulation, entry_drag_velocity_ratio;
    bool entry_bank_authority_acquired;
    bool entry_supervision_valid, entry_supervision_boundary_missed; EntrySupervisionMode entry_supervision_mode;

    double entry_supervision_ut;

    /* Live control defers expensive searches to the prediction worker. */
    bool entry_planning_deferred, entry_planning_needed, entry_lateral_infeasible, entry_committed_infeasible;
    EntryControlPlan entry_s_turn_plan;
    bool entry_predictor_models_valid;
    AerodynamicEnvelope entry_predictor_envelope;
    TrajectoryCalibrationModel entry_predictor_calibration;
    /* The committed native MM305 route is a compact deterministic descriptor;
       it has value semantics and is regenerated from its frozen geometry when
       queried, so predictor/preview copies remain independent and inexpensive. */
    TaemRoute mm305_route;
    size_t mm305_route_cursor;
    bool mm305_route_committed, mm305_hac_exit_reached;
    uint64_t mm305_model_snapshot_id;
    TaemInterfaceTarget taem_interface_target;
    bool taem_interface_captured, taem_safety_handoff;
    double taem_interface_diagnostic_ut;
    TerminalPathKind terminal_path_kind;
    bool terminal_path_committed;
    /*
     * Reciprocal runway direction is a guidance/planning decision.  preview is
     * reversible while terminal geometry is disposable; committed is latched
     * with terminal_path_committed and may change only after path invalidation.
     */
    bool runway_end_preview_valid, runway_end_committed;
    int runway_end_index; /* 0 = configured end, 1 = reciprocal end */
    double runway_end_primary_path, runway_end_reciprocal_path;
    /* The MM305 -> Final ownership boundary is selected with the committed
       terminal path. It is immutable until that path is explicitly invalidated. */
    bool terminal_final_handoff_latched;
    double terminal_final_handoff_distance;
    double hac_side; bool hac_side_selected; double delivered_delta_v, burn_command_started_ut, burn_active_elapsed;
    double burn_progress_watch_ut, burn_progress_watch_delta_v;
    bool has_burn_command_started, has_burn_progress_watch, deorbit_burn_completed, atmospheric_interface_crossed, has_previous_ut;
    double previous_ut; bool final_approach_captured, airbrakes_deployed;
    bool hac_progress_valid, hac_captured, hac_completed, terminal_region_entered, terminal_glide_mode, terminal_final_test_mode, terminal_rehearsal_mode, terminal_test_capture_active;
    double hac_capture_lost_duration, terminal_energy_mismatch_duration;
    double terminal_reentry_after_ut;
    double hac_remaining, hac_radius, minimum_turn_radius, terminal_test_glide_slope, terminal_test_final_approach_distance, final_invalid_duration;
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
    /* Post-capture landing law: learned lift/drag reference areas (per q per
       unit aerodynamic factor).  Kept per machine so predictor copies of the
       guidance never share or pollute the live estimate. */
    double terminal_lift_area_ema, terminal_drag_area_ema;
    double terminal_speedbrake_cda; /* learned speedbrake drag area, m^2 (0 = unknown) */
    double terminal_vs_prev, terminal_vs_prev_ut, terminal_vs_rate_ema;
    /* Observed lift/q and drag/q (m^2) in 1 deg incidence bins 0..20 deg. */
    double terminal_lift_q[21], terminal_drag_q[21];
    unsigned char terminal_aero_seen[21];
    double terminal_aero_mach[21];
    double terminal_aoa_cmd_last,terminal_aoa_cmd_ut;
    bool terminal_pull_latch;
    double terminal_lift_ratio; /* Pull-up ramp done; holding the sink profile. */
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
    bool entry_continuation_bootstrap;
    /* The MM304 inlet side is a mission-geometry choice.  It must not change
       merely because the energy-management S-turn changes ownership side. */
    bool entry_target_side_latched;
    double entry_target_side;
    double entry_reversal_ut, entry_reversal_range, entry_reversal_sign, entry_reversal_bank;
    unsigned entry_control_reversals;
    EntryGuidanceFeedback entry_feedback;
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
TaemInterfaceCapture entry_mm305_acquisition_capture(const GuidanceMachine *guidance,
    const TaemInterfaceTarget *target,const Telemetry *telemetry,double course,
    const PlanetModel *planet,const LandingConfiguration *configuration);
TaemInterfaceCapture entry_mm305_admission_envelope(
    const GuidanceMachine *guidance,const Telemetry *telemetry,double course,
    const PlanetModel *planet,const LandingConfiguration *configuration);

#endif
