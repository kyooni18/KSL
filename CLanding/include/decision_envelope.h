#ifndef KSP_LANDER_DECISION_ENVELOPE_H
#define KSP_LANDER_DECISION_ENVELOPE_H

#include "landing.h"

typedef struct {
    bool valid;
    double available;
    double required;
    double margin;
    double normalized_margin;
} DecisionMargin;

/* Down is positive. The response interval holds at most the supplied downward
 * acceleration; recovery then supplies the stated net upward acceleration.
 * This is a local constant-acceleration envelope, not a trajectory forecast.
 * Height is measured above the caller's required delivery boundary. */
typedef struct {
    bool valid;
    bool reachable;
    DecisionMargin height;
    double response_height_m;
    double braking_height_m;
    double required_height_m;
    double sink_after_response_mps;
    double recovery_time_s;
} VerticalRecoveryEnvelope;

VerticalRecoveryEnvelope decision_vertical_recovery_envelope(
    double available_height_m, double sink_mps, double target_sink_mps,
    double response_time_s, double response_down_accel_mps2,
    double recovery_up_accel_mps2);
double decision_pitch_capture_time(const Telemetry *telemetry, double target_aoa_deg);

typedef struct {
    double rate_deg_s;
    double accel_deg_s2;
} GuidanceAttitudeLimits;

/* Executable reference limits shared by guidance and final recovery timing. */
GuidanceAttitudeLimits decision_guidance_attitude_limits(
    const Telemetry *telemetry, const GuidanceSettings *settings,
    GuidancePhase phase, bool pitch_axis);
double decision_final_approach_response_time(const GuidanceMachine *machine,
    const Telemetry *telemetry, const GuidanceSettings *settings,
    double target_aoa_deg);

typedef struct {
    bool valid;
    bool survivable;
    bool controllable;

    DecisionMargin speed;
    DecisionMargin dynamic_pressure;
    DecisionMargin load;
    DecisionMargin stall;
    bool stall_observed;

    double minimum_speed_mps;
    double maximum_lift_aoa_deg;
    double maximum_normal_accel_mps2;
    double maximum_lateral_accel_mps2;
    double control_response_time_s;
} ControlAuthorityEnvelope;

typedef struct {
    bool valid;
    bool position_ready;
    bool heading_ready;
    bool inside;

    ControlAuthorityEnvelope control;
    DecisionMargin position;
    DecisionMargin heading;

    double center_distance_m;
    double heading_error_deg;
    double minimum_turn_radius_m;
    double path_to_set_m;
} HandoffSetEnvelope;

typedef struct {
    bool valid;
    bool feasible;
    double minimum_speed_mps;
    double maximum_speed_mps;
    double density_kg_m3;
    double dynamic_pressure_limit_pa;
    double load_pressure_limit_pa;
    double minimum_turn_radius_m;
} TaemSpeedEnvelope;

typedef struct {
    bool valid;
    bool reachable;

    ControlAuthorityEnvelope control;
    DecisionMargin capture_time;

    double forward_m;
    double cross_m;
    double range_m;
    double minimum_turn_radius_m;
    double path_length_m;
    double time_available_s;
    double required_time_s;
    double lateral_capture_time_s;
    double heading_capture_time_s;
    double course_error_deg;
} TargetCaptureEnvelope;

typedef struct {
    bool valid;
    bool feasible;

    DecisionMargin energy;
    double available_specific_energy;
    double modeled_drag_work;
    double uncertainty_specific_energy;
} EnergyPathEnvelope;

typedef struct {
    bool valid;
    double ground_path_m;
    double air_path_m;
    double modeled_drag_work;
    double terminal_speed_mps;
    double travel_time_s;
    double target_aoa_deg;
} UnpoweredPathProjection;

typedef struct {
    bool valid;
    bool feasible;

    DecisionMargin path_minimum;
    DecisionMargin path_maximum;
    DecisionMargin slope_minimum;
    DecisionMargin slope_maximum;
    DecisionMargin lateral_authority;
    DecisionMargin course_rate;
    DecisionMargin endpoint_rate;

    /*
     * Minimum normalized margin across every physical constraint.  Positive
     * means the path is inside every envelope; negative gives a scale-free
     * distance to the nearest violated boundary.  This is intentionally a
     * minimax quantity, not a weighted tuning score.
     */
    double worst_normalized_margin;
} PathFeasibilityEnvelope;

typedef struct {
    bool valid;
    bool reachable;

    DecisionMargin runway_time;
    DecisionMargin vertical_recovery;
    DecisionMargin speed;
    DecisionMargin dynamic_pressure;
    DecisionMargin load;
    DecisionMargin runway_remaining;

    double ground_range_m;
    double time_available_s;
    double lateral_capture_time_s;
    double heading_capture_time_s;
    double control_response_time_s;

    double cross_rate_mps;
    double lateral_accel_mps2;
    double course_rate_deg_s;
    double maximum_lift_aoa_deg;
    double maximum_normal_accel_mps2;

    double sink_rate_mps;
    double vertical_recovery_accel_mps2;
    double response_down_accel_mps2;
    double vertical_recovery_height_m;
    double recoverable_sink_rate_mps;
    double recoverable_fpa_deg;
} RunwayCaptureEnvelope;

DecisionMargin decision_margin(double available, double required);
double decision_bounded_capture_time(double position, double velocity,
    double acceleration_limit, double response_time);
double decision_axis_capture_time(double error_deg, double rate_deg_s,
        double acceleration_limit_deg_s2, double rate_limit_deg_s);
ControlAuthorityEnvelope decision_control_authority_envelope(
    const GuidanceMachine *guidance,
    const Telemetry *telemetry,
    const PlanetModel *planet,
    const LandingConfiguration *configuration);

HandoffSetEnvelope decision_mm304_handoff_set_envelope(
    const GuidanceMachine *guidance,
    const Telemetry *telemetry,
    double course_deg,
    double center_along_m,
    double center_cross_m,
    const PlanetModel *planet,
    const LandingConfiguration *configuration);

TaemSpeedEnvelope decision_taem_speed_envelope(
    const VehicleProfile *vehicle,
    const PlanetModel *planet,
    double altitude_m);
TargetCaptureEnvelope decision_target_capture_envelope(
    const GuidanceMachine *guidance,
    const Telemetry *telemetry,
    double course_deg,
    double target_along_m,
    double target_cross_m,
    double target_course_deg,
    double available_time_s,
    const PlanetModel *planet,
    const LandingConfiguration *configuration);

double decision_target_path_length(
    const GuidanceMachine *guidance,
    const Telemetry *telemetry,
    double course_deg,
    double target_along_m,
    double target_cross_m,
    double target_course_deg,
    double lateral_accel_limit_mps2,
    const PlanetModel *planet,
    const LandingConfiguration *configuration,
    double *minimum_turn_radius_m);

TargetCaptureEnvelope decision_target_capture_envelope_with_lateral_accel(
    const GuidanceMachine *guidance,
    const Telemetry *telemetry,
    double course_deg,
    double target_along_m,
    double target_cross_m,
    double target_course_deg,
    double available_time_s,
    double lateral_accel_limit_mps2,
    const PlanetModel *planet,
    const LandingConfiguration *configuration);
RunwayCaptureEnvelope decision_runway_capture_envelope(
    const GuidanceMachine *guidance,
    const Telemetry *telemetry,
    double course_deg,
    const PlanetModel *planet,
    const LandingConfiguration *configuration);
EnergyPathEnvelope decision_energy_path_envelope(
    const Telemetry *telemetry,
    const PlanetModel *planet,
    double available_specific_energy,
    double modeled_drag_work,
    double path_length_m);

UnpoweredPathProjection decision_unpowered_path_projection(
    const GuidanceMachine *guidance,
    const Telemetry *telemetry,
    const PlanetModel *planet,
    const LandingConfiguration *configuration,
    double target_altitude_m,
    double ground_path_m);

PathFeasibilityEnvelope decision_path_feasibility_envelope(
    double path_length_m,
    double minimum_path_m,
    double maximum_path_m,
    double slope_deg,
    double minimum_slope_deg,
    double maximum_slope_deg,
    double required_lateral_accel_mps2,
    double available_lateral_accel_mps2,
    double peak_course_rate_ratio,
    double endpoint_course_rate_deg_s,
    double endpoint_course_rate_limit_deg_s);

#endif
