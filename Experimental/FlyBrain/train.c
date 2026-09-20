#include "flybrain.h"
#include "simulator.h"

#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define CANDIDATES 30
#define ELITES 7
#define GENERATIONS 24
#define EPISODES_PER_CANDIDATE 72

typedef struct {
    uint64_t state;
} TrainRng;

typedef struct {
    FlyBrainParams params;
    double score;
} Candidate;

static uint64_t tr_u64(TrainRng *rng) {
    uint64_t x = rng->state;
    x ^= x >> 12;
    x ^= x << 25;
    x ^= x >> 27;
    rng->state = x;
    return x * UINT64_C(2685821657736338717);
}

static double tr_unit(TrainRng *rng) {
    return (double)(tr_u64(rng) >> 11) * (1.0 / 9007199254740992.0);
}

static double tr_normal(TrainRng *rng) {
    const double u1 = fmax(tr_unit(rng), 1e-12);
    const double u2 = tr_unit(rng);
    return sqrt(-2.0 * log(u1)) * cos(6.2831853071795864769 * u2);
}

static int compare_candidate(const void *a, const void *b) {
    const Candidate *ca = (const Candidate *)a;
    const Candidate *cb = (const Candidate *)b;
    return (ca->score > cb->score) - (ca->score < cb->score);
}

static void print_params(const FlyBrainParams *p) {
    printf("att=%.5f rate=%.5f indi=%.5f ff=%.5f filt=%.5f rise=%.5f fall=%.5f slew=%.5f maxRate=%.5f",
           p->attitude_gain, p->rate_gain, p->indi_gain, p->reference_feedforward,
           p->accel_filter_alpha, p->authority_rise_tau, p->authority_fall_tau,
           p->control_slew_per_second, p->max_rate_deg_s);
}

int main(int argc, char **argv) {
    const char *output_path = argc > 1 ? argv[1] : "trained_params.txt";
    TrainRng rng = { UINT64_C(0x6a09e667f3bcc909) };

    FlyBrainParams initial = flybrain_default_params();
    double mean[FLYBRAIN_PARAM_COUNT];
    flybrain_params_to_array(&initial, mean);
    double sigma[FLYBRAIN_PARAM_COUNT] = {0.28, 0.55, 0.16, 0.28, 0.16, 0.85, 0.16, 0.75, 1.00};
    Candidate candidates[CANDIDATES];
    FlyBrainParams best = initial;
    double best_score = INFINITY;

    printf("FlyBrain trainer %s: %d generations, %d candidates, %d episodes/candidate\n",
           FLYBRAIN_REVISION, GENERATIONS, CANDIDATES, EPISODES_PER_CANDIDATE);

    for (int generation = 0; generation < GENERATIONS; ++generation) {
        const uint64_t generation_seed = UINT64_C(0x243f6a8885a308d3) +
                                         UINT64_C(0x9e3779b97f4a7c15) * (uint64_t)generation;
        for (int i = 0; i < CANDIDATES; ++i) {
            double values[FLYBRAIN_PARAM_COUNT];
            for (int j = 0; j < FLYBRAIN_PARAM_COUNT; ++j) {
                values[j] = mean[j] + sigma[j] * tr_normal(&rng);
            }
            candidates[i].params = flybrain_params_from_array(values);
            const FlyBrainAggregate a = flybrain_evaluate(&candidates[i].params,
                                                           generation_seed,
                                                           EPISODES_PER_CANDIDATE,
                                                           false);
            candidates[i].score = a.mean_cost +
                                  6000.0 * (1.0 - a.stable_fraction) +
                                  18.0 * a.mean_final_error_deg;
        }

        qsort(candidates, CANDIDATES, sizeof(candidates[0]), compare_candidate);
        if (candidates[0].score < best_score) {
            best_score = candidates[0].score;
            best = candidates[0].params;
        }

        double next_mean[FLYBRAIN_PARAM_COUNT] = {0};
        double next_sigma[FLYBRAIN_PARAM_COUNT] = {0};
        for (int e = 0; e < ELITES; ++e) {
            double values[FLYBRAIN_PARAM_COUNT];
            flybrain_params_to_array(&candidates[e].params, values);
            for (int j = 0; j < FLYBRAIN_PARAM_COUNT; ++j) next_mean[j] += values[j] / ELITES;
        }
        for (int e = 0; e < ELITES; ++e) {
            double values[FLYBRAIN_PARAM_COUNT];
            flybrain_params_to_array(&candidates[e].params, values);
            for (int j = 0; j < FLYBRAIN_PARAM_COUNT; ++j) {
                const double d = values[j] - next_mean[j];
                next_sigma[j] += d * d / ELITES;
            }
        }
        for (int j = 0; j < FLYBRAIN_PARAM_COUNT; ++j) {
            next_sigma[j] = sqrt(next_sigma[j]);
            mean[j] = 0.25 * mean[j] + 0.75 * next_mean[j];
            sigma[j] = fmax(0.035 * (fabs(mean[j]) + 0.25),
                            0.25 * sigma[j] + 0.75 * next_sigma[j]);
        }

        printf("gen %02d score=%9.3f ", generation + 1, candidates[0].score);
        print_params(&candidates[0].params);
        printf("\n");
    }

    const FlyBrainAggregate final_normal = flybrain_evaluate(&best, UINT64_C(0xb7e151628aed2a6b), 600, false);
    const FlyBrainAggregate final_hard = flybrain_evaluate(&best, UINT64_C(0x8aed2a6abf715880), 600, true);

    printf("best score %.3f\n", best_score);
    printf("best params: ");
    print_params(&best);
    printf("\nnormal: stable=%.3f rms=%.3f final=%.3f worstFinal=%.3f worstRate=%.3f activity=%.3f sat=%.4f\n",
           final_normal.stable_fraction, final_normal.mean_rms_error_deg,
           final_normal.mean_final_error_deg, final_normal.worst_final_error_deg,
           final_normal.worst_rate_deg_s, final_normal.mean_control_activity,
           final_normal.saturation_fraction);
    printf("hard:   stable=%.3f rms=%.3f final=%.3f worstFinal=%.3f worstRate=%.3f activity=%.3f sat=%.4f\n",
           final_hard.stable_fraction, final_hard.mean_rms_error_deg,
           final_hard.mean_final_error_deg, final_hard.worst_final_error_deg,
           final_hard.worst_rate_deg_s, final_hard.mean_control_activity,
           final_hard.saturation_fraction);

    if (!flybrain_params_save(output_path, &best)) {
        fprintf(stderr, "failed to write %s\n", output_path);
        return 2;
    }
    printf("wrote %s\n", output_path);
    return final_hard.stable_fraction >= 0.98 ? 0 : 1;
}
