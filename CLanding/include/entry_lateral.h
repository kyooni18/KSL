#ifndef KSP_ENTRY_LATERAL_H
#define KSP_ENTRY_LATERAL_H

#include <stdbool.h>

typedef struct {
    bool valid;
    double confidence;
    double reference_drag_accel;
    double drag_error_accel;
    double predicted_range_error;
    double range_error_scale;
    double required_vertical_lift_accel;
} EntryLateralLongitudinalDemand;

/*
 * ENTRYLATERAL is an actuator allocator, not an S-turn policy owner.
 * The executive supplies all physical limits. No default course/crossrange
 * corridor or reversal threshold lives here.
 */
typedef struct {
    double maximum_bank_deg;
    double maximum_roll_rate_deg_s;
    double maximum_roll_accel_deg_s2;
    double minimum_leg_duration_s; /* retained for ABI/tests; executive owns dwell */
} EntryLateralLimits;

typedef struct {
    double ut;
    double dt;
    double relative_speed;
    double measured_bank_deg;
    double measured_bank_rate_deg_s;
    double bank_effectiveness;
    double lift_accel;
    /* Direct local force evidence; distinct from confidence in extrapolating
     * an aerodynamic/authority model to future states. */
    bool has_measured_lift;
    /* A closed-loop rollout's prospective sensor observation. This is modeled
       force, not evidence of a real measurement or model certification. */
    bool has_simulated_lift_observation;
    double authority_confidence;
    double course_to_site_error_deg;
    bool has_crossrange_error;
    double crossrange_error_m;
    bool has_crossrange_rate;
    double crossrange_error_rate_mps;
    bool has_crossrange_accel;
    double crossrange_error_accel_mps2;
    double crossrange_uncertainty_m;
    /* Cosine of current ground course relative to runway heading. This maps
       predicted target-current crossrange to the corrective bank sign. */
    /* Experimental measured-authority heading-arrest projection. */
    bool crossrange_arrest_prediction;
    bool has_crossrange_course_alignment;
    double crossrange_course_alignment;
    /* Measured coordination state. The planner uses the configured sideslip
       envelope to slow bank motion before a reversal outruns yaw coordination. */
    bool has_sideslip;
    double sideslip_deg;
    double maximum_sideslip_deg;
    EntryLateralLongitudinalDemand longitudinal;
    /* Bank magnitude owned by the energy law (entry_energy_control). When set,
       it replaces the legacy vertical-lift allocation above. */
    bool has_bank_magnitude;
    double bank_magnitude_deg;
    /* Target-relative azimuth error (bearing to target minus velocity azimuth,
       deg, positive = target to the right) and its reversal deadband.  This is
       heading-independent: a right bank always turns toward a target on the
       right, whatever the flight direction. */
    bool has_azimuth_error;
    double azimuth_error_deg;
    double azimuth_deadband_deg;
    /* Time remaining to the lateral capture point. With crossrange state it
       defines the lateral-lift demand that shares measured lift with the
       longitudinal vertical-lift demand. */
    bool has_time_to_go;
    double time_to_go_s;
    /* Zero-effort lateral miss of the capture point along the present ground
       track (target-current sign). When set it is the lateral-lift demand's
       miss; runway-line crossrange remains for terminal capture/reversal. */
    bool has_point_miss;
    double point_miss_m;
} EntryLateralInput;

typedef struct {
    bool initialized;
    double bank_sign;
    bool leg_captured;
    double leg_captured_ut;
    bool reversal_armed;
    double last_reversal_ut;
    double command_bank_deg;
    double command_bank_rate_deg_s;
    bool has_previous_sideslip;
    double previous_sideslip_deg;
} EntryLateralState;

typedef struct {
    bool valid;
    bool reversal_requested; /* physical crossrange feedback requests a bank-sign change */
    bool leg_captured;
    bool reversal_armed;
    bool degraded_authority;
    double bank_sign;
    double bank_magnitude_deg;
    /* Bank the vertical-lift demand alone would fly. Drag/incidence
       saturation logic must read this, not a magnitude raised for crossrange. */
    double energy_bank_magnitude_deg;
    double raw_target_bank_deg;
    double target_bank_deg;
    double target_bank_rate_deg_s;
    double corridor_metric;       /* NAN: no corridor policy in this allocator */
    double course_corridor_deg;   /* NAN */
    double crossrange_corridor_m; /* physical roll-response + navigation uncertainty */
    double projected_crossrange_error_m;
    double reversal_response_time_s;
    double required_lateral_lift_accel; /* NAN when no crossrange demand */
    double zero_effort_crossrange_m;    /* NAN when no crossrange demand */
    bool lateral_demand_binding;        /* combined demand exceeded measured lift */
    double expected_course_rate_deg_s;
    double sideslip_rate_deg_s;
    double coordination_roll_rate_limit_deg_s;
} EntryLateralOutput;

/*
 * Physical admission model for the MM304 terminal turn.
 *
 * Courses are runway-relative degrees; along/cross are runway-frame metres.
 * capture_radius_m is an explicit mission contract (currently the fixed
 * MM304 rear-alignment capture radius), not a tuning tolerance.
 */
typedef struct {
    double horizontal_speed_mps;
    double current_course_deg;
    double target_course_deg;
    double current_along_m;
    double current_cross_m;
    double target_along_m;
    double target_cross_m;
    double measured_bank_deg;
    double measured_bank_rate_deg_s;
    double lift_accel_mps2;
    double bank_effectiveness;
    double maximum_bank_deg;
    double maximum_roll_rate_deg_s;
    double maximum_roll_accel_deg_s2;
    double capture_radius_m;
    double position_uncertainty_m;
} EntryLateralTurnInput;

typedef struct {
    bool valid;
    double turn_sign;
    double response_time_s;
    double response_distance_m;
    double response_heading_change_deg;
    double projected_along_m;
    double projected_cross_m;
    double projected_course_deg;
    double remaining_course_change_deg;
    double minimum_turn_radius_m;
    double ideal_turn_radius_m;
    double radius_margin_m;
    double endpoint_error_m;
    double capture_margin_m;
    double feasibility_margin_m;
} EntryLateralTurnEnvelope;

/*
 * Select the locally shorter signed turn toward a runway-frame point. If the
 * point is numerically coincident with the vehicle, target_course_deg is the
 * geometric tie-break. A zero return means the geometry itself is symmetric.
 */
double entry_lateral_geometry_turn_sign(double current_course_deg,
    double current_along_m, double current_cross_m,
    double target_along_m, double target_cross_m,
    double target_course_deg);

double entry_lateral_crossrange_arrest_distance(double crossrange_rate,
    double course_alignment,double lift_accel,double effective_bank_deg);
EntryLateralLimits entry_lateral_default_limits(void);
void entry_lateral_state_init(EntryLateralState *state, double ut,
    double bank_sign_hint, double measured_bank_deg);
EntryLateralOutput entry_lateral_update(EntryLateralState *state,
    const EntryLateralInput *input, const EntryLateralLimits *limits);
EntryLateralTurnEnvelope entry_lateral_terminal_turn_envelope(
    const EntryLateralTurnInput *input);

#endif
