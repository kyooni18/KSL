#ifndef KSP_LANDER_FLIGHT_CONTROL_H
#define KSP_LANDER_FLIGHT_CONTROL_H

#include "landing.h"

#include <stdbool.h>

#define FLIGHT_CONTROL_REVISION "native-c-fcs-20260911-r2"

typedef enum {
    FLIGHT_CONTROL_AXIS_PITCH = 0,
    FLIGHT_CONTROL_AXIS_ROLL = 1,
    FLIGHT_CONTROL_AXIS_YAW = 2,
    FLIGHT_CONTROL_AXIS_COUNT = 3
} FlightControlAxis;

typedef struct {
    double torque_accel;
    double reported_torque_accel;
    double non_aero_confidence;
    /* Effective acceleration of the coordinate actually controlled by this
       axis (AoA for pitch, surface bank for roll). Rigid-body torque/inertia is
       retained separately because it is not, by itself, a stopping authority
       for those coordinates. */
    double controlled_accel;
    double controlled_confidence;
    double aero_per_q;
    double confidence;
    double drift_accel;
    double last_rate;
    bool reported_initialized;
    bool has_last_rate;
} FlightControlAxisAuthority;

typedef struct {
    double mean_yaw_rate;
    double mean_roll_rate;
    double mean_beta_rate;
    double yy;
    double rr;
    double yr;
    double by;
    double br;
    double yaw_gain;
    double roll_gain;
    double confidence;
    bool initialized;
} FlightControlBetaModel;

typedef struct {
    ControlProfile profile;
    double target_pitch_state;
    double measured_pitch_state;
    double pitch_error;
    double effective_pitch_rate;
    double body_pitch_rate;
    bool body_pitch_rate_available;

    double roll_error;
    double effective_roll_rate;
    double body_roll_rate;
    bool body_roll_rate_available;
    double target_roll_rate;
    double commanded_roll_rate;
    double relative_roll_rate;
    bool recovery_rate_first;

    double heading_error;
    double sideslip;
    double sideslip_rate;
    double body_yaw_rate;
    bool body_yaw_rate_available;
    double flight_path_angle;

    double pitch_trim;
    double steady_pitch_trim;
    double terminal_pitch_integral;
    double roll_trim;

    double pitch_authority;
    double pitch_raw_authority;
    double pitch_guard_authority;
    double pitch_aero_fraction;
    double pitch_fast_command_limit;
    double pitch_hold_seconds;
    double pitch_rate_impulse_budget;

    double roll_authority;
    double roll_raw_authority;
    double roll_guard_authority;
    double roll_aero_fraction;
    double roll_rate_limit;
    double roll_command_limit;
    double roll_hold_seconds;
    double roll_rate_impulse_budget;
    double roll_bandwidth_scale;

    double yaw_authority;
    double yaw_aero_fraction;
    double beta_yaw_gain;
    double beta_roll_coupling;
    double beta_confidence;

    double sample_dt;
    double attitude_error;
    bool rcs_transonic_cutoff;
} FlightControlDiagnostics;

typedef struct {
    bool valid;
    double pitch;
    double roll;
    double yaw;
    double throttle;
    double wheel_steering;
    bool gear;
    bool brakes;
    bool airbrakes;
    double rcs_assist;
    bool rcs_requested;
    FlightControlDiagnostics diagnostics;
} FlightControlOutput;

typedef struct {
    double nominal_dt;
    bool has_sample;
    double last_ut;
    double last_pitch;
    double last_aoa;
    double last_roll;
    double last_heading;
    double last_sideslip;

    double pitch_rate;
    double aoa_rate;
    double roll_rate;
    double heading_rate;
    double sideslip_rate;

    double last_control[FLIGHT_CONTROL_AXIS_COUNT];
    FlightControlAxisAuthority authority[FLIGHT_CONTROL_AXIS_COUNT];
    FlightControlBetaModel beta;

    double pitch_trim;
    double terminal_pitch_integral;
    double roll_trim;
    double terminal_pitch_authority;
    double terminal_roll_authority;
    bool has_terminal_pitch_authority;
    bool has_terminal_roll_authority;

    double last_target_pitch_state, target_pitch_rate;
    bool has_last_target_pitch;
    double last_target_roll;
    double target_roll_rate;
    bool has_last_target_roll;

    bool rcs_transonic_cutoff;
    ControlProfile last_profile;
    bool has_last_profile;
} FlightControlState;

void flight_control_init(FlightControlState *state, double nominal_dt);
void flight_control_reset_transients(FlightControlState *state);
void flight_control_seed_axis_authority(FlightControlState *state,
                                        FlightControlAxis axis,
                                        double non_aero_accel,
                                        double aero_accel_per_pascal,
                                        double confidence);

bool flight_control_step(FlightControlState *state,
                         const Telemetry *telemetry,
                         const GuidanceCommand *command,
                         double sample_dt,
                         FlightControlOutput *output);

const char *flight_control_revision(void);

#endif
