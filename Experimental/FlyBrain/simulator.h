#ifndef KSP_EXPERIMENTAL_FLYBRAIN_SIMULATOR_H
#define KSP_EXPERIMENTAL_FLYBRAIN_SIMULATOR_H

#include "flybrain.h"

#include <stdbool.h>
#include <stdint.h>

typedef struct {
    double cost;
    double rms_error_deg;
    double final_mean_abs_error_deg;
    double max_abs_error_deg;
    double max_abs_rate_deg_s;
    double mean_control_activity;
    double saturation_fraction;
    bool stable;
} FlyBrainEpisodeResult;

typedef struct {
    double mean_cost;
    double mean_rms_error_deg;
    double mean_final_error_deg;
    double worst_final_error_deg;
    double worst_rate_deg_s;
    double stable_fraction;
    double mean_control_activity;
    double saturation_fraction;
} FlyBrainAggregate;

FlyBrainEpisodeResult flybrain_simulate_episode(const FlyBrainParams *params,
                                                 uint64_t seed,
                                                 bool hard_mode);
FlyBrainAggregate flybrain_evaluate(const FlyBrainParams *params,
                                    uint64_t seed_base,
                                    int episode_count,
                                    bool hard_mode);

#endif
