#ifndef KSP_EXPERIMENTAL_FLYBRAIN_H
#define KSP_EXPERIMENTAL_FLYBRAIN_H

#include <stdbool.h>

#define FLYBRAIN_AXIS_COUNT 3
#define FLYBRAIN_PARAM_COUNT 9
#define FLYBRAIN_REVISION "flybrain-indi-cem-v1"

typedef enum {
    FLYBRAIN_PITCH = 0,
    FLYBRAIN_ROLL = 1,
    FLYBRAIN_YAW = 2
} FlyBrainAxis;

typedef struct {
    double attitude_gain;
    double rate_gain;
    double indi_gain;
    double reference_feedforward;
    double accel_filter_alpha;
    double authority_rise_tau;
    double authority_fall_tau;
    double control_slew_per_second;
    double max_rate_deg_s;
} FlyBrainParams;

typedef struct {
    double attitude_error_deg[FLYBRAIN_AXIS_COUNT];
    double body_rate_deg_s[FLYBRAIN_AXIS_COUNT];
    double body_accel_deg_s2[FLYBRAIN_AXIS_COUNT];
    double authority_hint_deg_s2[FLYBRAIN_AXIS_COUNT];
    double dt;
} FlyBrainInput;

typedef struct {
    double control[FLYBRAIN_AXIS_COUNT];
    double target_rate_deg_s[FLYBRAIN_AXIS_COUNT];
    double target_accel_deg_s2[FLYBRAIN_AXIS_COUNT];
    double authority_estimate_deg_s2[FLYBRAIN_AXIS_COUNT];
} FlyBrainOutput;

typedef struct {
    bool initialized;
    double control[FLYBRAIN_AXIS_COUNT];
    double filtered_accel[FLYBRAIN_AXIS_COUNT];
    double last_filtered_accel[FLYBRAIN_AXIS_COUNT];
    double authority_estimate[FLYBRAIN_AXIS_COUNT];
    double last_target_rate[FLYBRAIN_AXIS_COUNT];
    double last_control_delta[FLYBRAIN_AXIS_COUNT];
} FlyBrainState;

FlyBrainParams flybrain_default_params(void);
void flybrain_init(FlyBrainState *state, const double authority_hint_deg_s2[FLYBRAIN_AXIS_COUNT]);
bool flybrain_step(FlyBrainState *state,
                   const FlyBrainParams *params,
                   const FlyBrainInput *input,
                   FlyBrainOutput *output);

bool flybrain_params_save(const char *path, const FlyBrainParams *params);
bool flybrain_params_load(const char *path, FlyBrainParams *params);
void flybrain_params_to_array(const FlyBrainParams *params, double values[FLYBRAIN_PARAM_COUNT]);
FlyBrainParams flybrain_params_from_array(const double values[FLYBRAIN_PARAM_COUNT]);
void flybrain_params_clamp(FlyBrainParams *params);

#endif
