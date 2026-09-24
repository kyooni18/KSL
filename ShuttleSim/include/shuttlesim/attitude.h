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
} AttitudeModel;

void attitude_seed(AttitudeModel *a);
bool attitude_load_ini(AttitudeModel *a, const char *path);
void attitude_set_command(AttitudeModel *a, double aoa_rad, double bank_rad);
void attitude_step(AttitudeModel *a, double dynamic_pressure_pa, double dt);
Quat attitude_body_quat(Vec3 position_i, Vec3 air_velocity_i, double aoa_rad, double bank_rad);
#endif
