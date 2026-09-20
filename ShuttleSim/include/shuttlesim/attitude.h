#ifndef SHUTTLESIM_ATTITUDE_H
#define SHUTTLESIM_ATTITUDE_H
#include "types.h"

typedef struct {
    double aoa_rad;
    double bank_rad;
    double aoa_rate_rad_s;
    double bank_rate_rad_s;
    double cmd_aoa_rad;
    double cmd_bank_rad;
    double pitch_wn;
    double pitch_zeta;
    double roll_wn;
    double roll_zeta;
    double max_pitch_rate_rad_s;
    double max_roll_rate_rad_s;
    double max_pitch_accel_rad_s2;
    double max_roll_accel_rad_s2;
} AttitudeModel;

void attitude_seed(AttitudeModel *a);
bool attitude_load_ini(AttitudeModel *a, const char *path);
void attitude_set_command(AttitudeModel *a, double aoa_rad, double bank_rad);
void attitude_step(AttitudeModel *a, double dt);
Quat attitude_body_quat(Vec3 position_i, Vec3 air_velocity_i, double aoa_rad, double bank_rad);
#endif
