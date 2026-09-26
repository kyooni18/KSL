#ifndef KSP_LANDER_FLIGHT_CONTROL_H
#define KSP_LANDER_FLIGHT_CONTROL_H

#include "landing.h"

#include <stdbool.h>

#define FLIGHT_CONTROL_REVISION "native-c-fcs-20260927-r3"

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

/* Shortest-rotation attitude error resolved in the vessel body axes and
   expressed in the same pitch/roll/yaw sign convention as direct kRPC
   controls. The target frame itself is constructed from air-relative AoA
   and bank, so guidance remains aerodynamic rather than Euler-attitude based. */
typedef struct {
    bool valid;
    bool use_yaw_error;
    double pitch_error_deg;
    double roll_error_deg;
    double yaw_error_deg;
} FlightControlAttitudeError;

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
    double pitch_bandwidth_scale;

    double yaw_authority;
    double yaw_aero_fraction;
    double beta_yaw_gain;
    double beta_roll_coupling;
    double beta_confidence;

    double sample_dt;
    double attitude_error;
    bool quaternion_attitude_control;
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

    /* Quaternion tracking differentiates the target-relative rotation vector,
       not absolute body rate. This keeps damping in the moving air-relative
       reference frame while the authority observer still sees physical rates. */
    bool has_quaternion_error_sample;
    double last_quaternion_pitch_error;
    double last_quaternion_roll_error;
    double last_quaternion_yaw_error;
    double quaternion_pitch_error_rate;
    double quaternion_roll_error_rate;
    double quaternion_yaw_error_rate;

    /* Online pitch-response tuning.  The pitch loop adapts its closed-loop
       bandwidth from measured target-relative response; roll and yaw retain
       their validated fixed/scheduled laws because they must sustain lateral
       control against aerodynamic cross-coupling. */
    double pitch_response_scale;

    bool rcs_transonic_cutoff;
    ControlProfile last_profile;
    bool has_last_profile;
    /* Latched on the first real main-wheel contact. Pitch is guided through
       derotation until nose gear contact, then permanently released. */
    bool main_gear_contact_latched;
    bool nose_gear_contact_latched;
    double main_gear_contact_ut;
} FlightControlState;

void flight_control_init(FlightControlState *state, double nominal_dt);
void flight_control_reset_transients(FlightControlState *state);
void flight_control_seed_axis_authority(FlightControlState *state,
                                        FlightControlAxis axis,
                                        double non_aero_accel,
                                        double aero_accel_per_pascal,
                                        double confidence);

/* Construct an air-relative desired body frame in the same reference frame as
   attitude_quaternion. air_direction and radial_up must already be expressed
   in that frame. Positive bank follows the measured lift-vector convention. */
bool flight_control_airframe_error(const double attitude_quaternion[4],
                                   Vector3 air_direction,
                                   Vector3 radial_up,
                                   double target_aoa_deg,
                                   double target_bank_deg,
                                   bool use_yaw_error,
                                   FlightControlAttitudeError *error);

bool flight_control_step_attitude(FlightControlState *state,
                                  const Telemetry *telemetry,
                                  const GuidanceCommand *command,
                                  const FlightControlAttitudeError *attitude_error,
                                  double sample_dt,
                                  FlightControlOutput *output);

bool flight_control_step(FlightControlState *state,
                         const Telemetry *telemetry,
                         const GuidanceCommand *command,
                         double sample_dt,
                         FlightControlOutput *output);

const char *flight_control_revision(void);

#endif
