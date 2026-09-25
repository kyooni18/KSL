#ifndef SHUTTLESIM_ATTITUDE_H
#define SHUTTLESIM_ATTITUDE_H
#include "types.h"

typedef struct {
    double aoa_rad;
    double bank_rad;
    double aoa_rate_rad_s;
    double bank_rate_rad_s;
    /* Raw demand is retained for diagnostics; cmd_* is the bounded target
       actually exposed to the physics and telemetry consumers. */
    double requested_aoa_rad;
    double requested_bank_rad;
    double cmd_aoa_rad;
    double cmd_bank_rad;
    /* Closed-loop response identified from KSP target-vs-actual telemetry. */
    double pitch_wn;
    double pitch_zeta;
    double roll_wn;
    double roll_zeta;
    double max_pitch_rate_rad_s;
    double max_roll_rate_rad_s;
    /* Directly measured KSP rate and angular-acceleration envelope. */
    double max_pitch_accel_rad_s2;
    double max_roll_accel_rad_s2;
    /* Dynamic pressure at which the measured full-authority response is reached.
       Below this point aerodynamic moment/acceleration scales with q. */
    double pitch_full_authority_q_pa;
    double roll_full_authority_q_pa;

    /* Direct-control mode (surface moments).  When `direct` is set the axes
       are driven by normalized stick inputs in [-1, 1] instead of the ideal
       AoA/bank servo, so the backend's own flight-control system closes the
       loop exactly as on KSP.  Coefficients are a generic lifting-body model
       (control and restoring moments proportional to q, light aerodynamic
       damping), not a KSP identification.  Below direct_min_q_pa the servo
       stands in for RCS/reaction-wheel attitude hold. */
    bool direct;
    double input_pitch, input_roll, input_yaw;
    double sideslip_rad, sideslip_rate_rad_s;
    double direct_min_q_pa;
    double direct_pitch_accel_per_kpa;     /* rad/s^2 per kPa of q at full input */
    double direct_pitch_accel_max;         /* rad/s^2 */
    double direct_pitch_stiffness_per_kpa; /* 1/s^2 per kPa: restoring toward trim */
    double direct_trim_aoa_rad;            /* zero-input trim incidence */
    double direct_pitch_damping_ratio;     /* aerodynamic, of the stiffness mode */
    double direct_roll_accel_per_kpa;
    double direct_roll_accel_max;
    double direct_roll_damping_s_inv;      /* roll damping at 10 kPa, scales with sqrt(q) */
    double direct_yaw_accel_per_kpa;
    double direct_yaw_accel_max;
    double direct_yaw_stiffness_per_kpa;   /* weathercock stability on sideslip */
} AttitudeModel;

void attitude_seed(AttitudeModel *a);
bool attitude_load_ini(AttitudeModel *a, const char *path);
void attitude_set_command(AttitudeModel *a, double aoa_rad, double bank_rad);
void attitude_set_inputs(AttitudeModel *a, double pitch, double roll, double yaw);
void attitude_step(AttitudeModel *a, double dynamic_pressure_pa, double dt);
Quat attitude_body_quat(Vec3 position_i, Vec3 air_velocity_i, double aoa_rad, double bank_rad);
#endif
