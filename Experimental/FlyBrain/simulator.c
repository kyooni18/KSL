#include "simulator.h"

#include <math.h>
#include <stddef.h>
#include <string.h>

#define SIM_PI 3.14159265358979323846264338327950288

typedef struct {
    uint64_t state;
} Rng;

typedef struct {
    double angle[FLYBRAIN_AXIS_COUNT];
    double rate[FLYBRAIN_AXIS_COUNT];
    double accel[FLYBRAIN_AXIS_COUNT];
    double actuator[FLYBRAIN_AXIS_COUNT];
    double authority[FLYBRAIN_AXIS_COUNT][FLYBRAIN_AXIS_COUNT];
    double damping[FLYBRAIN_AXIS_COUNT];
    double lag_tau[FLYBRAIN_AXIS_COUNT];
    double authority_phase[FLYBRAIN_AXIS_COUNT];
    double disturbance[FLYBRAIN_AXIS_COUNT];
} Plant;

static uint64_t rng_u64(Rng *rng) {
    uint64_t x = rng->state;
    x ^= x >> 12;
    x ^= x << 25;
    x ^= x >> 27;
    rng->state = x;
    return x * UINT64_C(2685821657736338717);
}

static double rng_unit(Rng *rng) {
    return (double)(rng_u64(rng) >> 11) * (1.0 / 9007199254740992.0);
}

static double rng_range(Rng *rng, double lower, double upper) {
    return lower + (upper - lower) * rng_unit(rng);
}

static double rng_signed(Rng *rng) {
    return rng_range(rng, -1.0, 1.0);
}

static double clampd(double v, double lo, double hi) {
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

static double wrap_error(double target, double actual) {
    double error = fmod(target - actual, 360.0);
    if (error > 180.0) error -= 360.0;
    if (error < -180.0) error += 360.0;
    return error;
}

static void plant_randomize(Plant *plant, Rng *rng, bool hard_mode) {
    memset(plant, 0, sizeof(*plant));
    const double authority_min = hard_mode ? 10.0 : 20.0;
    const double authority_max = hard_mode ? 260.0 : 200.0;
    const double coupling = hard_mode ? 0.24 : 0.16;

    for (int i = 0; i < FLYBRAIN_AXIS_COUNT; ++i) {
        const double diagonal = exp(rng_range(rng, log(authority_min), log(authority_max)));
        plant->authority[i][i] = diagonal;
        plant->damping[i] = rng_range(rng, hard_mode ? 0.03 : 0.08, hard_mode ? 1.8 : 1.25);
        plant->lag_tau[i] = rng_range(rng, hard_mode ? 0.035 : 0.045, hard_mode ? 0.42 : 0.28);
        plant->authority_phase[i] = rng_range(rng, 0.0, 2.0 * SIM_PI);
        plant->angle[i] = rng_range(rng, hard_mode ? -32.0 : -24.0, hard_mode ? 32.0 : 24.0);
        plant->rate[i] = rng_range(rng, hard_mode ? -7.0 : -4.0, hard_mode ? 7.0 : 4.0);
        plant->disturbance[i] = rng_range(rng, -1.0, 1.0);
    }

    for (int i = 0; i < FLYBRAIN_AXIS_COUNT; ++i) {
        for (int j = 0; j < FLYBRAIN_AXIS_COUNT; ++j) {
            if (i == j) continue;
            const double scale = sqrt(plant->authority[i][i] * plant->authority[j][j]);
            plant->authority[i][j] = coupling * scale * rng_signed(rng);
        }
    }
}

static void target_profile(double t, const double target_a[3], const double target_b[3],
                           const double target_c[3], const double slew[3], double target[3]) {
    for (int axis = 0; axis < 3; ++axis) {
        if (t < 2.0) {
            target[axis] = target_a[axis];
        } else if (t < 6.0) {
            const double delta = target_b[axis] - target_a[axis];
            const double max_move = slew[axis] * (t - 2.0);
            target[axis] = target_a[axis] + clampd(delta, -max_move, max_move);
        } else {
            const double at_six_delta = target_b[axis] - target_a[axis];
            const double at_six = target_a[axis] + clampd(at_six_delta, -slew[axis] * 4.0, slew[axis] * 4.0);
            const double delta = target_c[axis] - at_six;
            const double max_move = slew[axis] * (t - 6.0);
            target[axis] = at_six + clampd(delta, -max_move, max_move);
        }
    }
}

FlyBrainEpisodeResult flybrain_simulate_episode(const FlyBrainParams *params,
                                                 uint64_t seed,
                                                 bool hard_mode) {
    FlyBrainEpisodeResult result;
    memset(&result, 0, sizeof(result));

    Rng rng = { seed ? seed : UINT64_C(0x9e3779b97f4a7c15) };
    Plant plant;
    plant_randomize(&plant, &rng, hard_mode);

    const double dt = rng_range(&rng, hard_mode ? 0.025 : 0.04, hard_mode ? 0.14 : 0.10);
    const double duration = hard_mode ? 15.0 : 12.0;
    const int steps = (int)ceil(duration / dt);
    const double noise_accel = hard_mode ? rng_range(&rng, 0.0, 0.55) : rng_range(&rng, 0.0, 0.22);
    const double noise_rate = hard_mode ? rng_range(&rng, 0.0, 0.13) : rng_range(&rng, 0.0, 0.06);

    double target_a[3] = {0.0, 0.0, 0.0};
    double target_b[3];
    double target_c[3];
    double slew[3];
    for (int axis = 0; axis < 3; ++axis) {
        const double amplitude = hard_mode ? 32.0 : 24.0;
        target_b[axis] = rng_range(&rng, -amplitude, amplitude);
        target_c[axis] = -0.75 * target_b[axis] + rng_range(&rng, -8.0, 8.0);
        slew[axis] = rng_range(&rng, hard_mode ? 2.0 : 2.0, hard_mode ? 8.0 : 6.0);
    }

    double hints[3];
    for (int axis = 0; axis < 3; ++axis) {
        const double hint_error = hard_mode ? rng_range(&rng, 0.55, 1.55) : rng_range(&rng, 0.75, 1.25);
        hints[axis] = plant.authority[axis][axis] * hint_error;
    }

    FlyBrainState brain;
    flybrain_init(&brain, hints);

    double error_sq_sum = 0.0;
    double final_error_sum = 0.0;
    double activity_sum = 0.0;
    double last_control[3] = {0.0, 0.0, 0.0};
    long sample_count = 0;
    long final_sample_count = 0;
    long saturation_count = 0;
    result.stable = true;

    for (int step = 0; step < steps; ++step) {
        const double t = step * dt;
        double target[3];
        target_profile(t, target_a, target_b, target_c, slew, target);

        FlyBrainInput input;
        memset(&input, 0, sizeof(input));
        input.dt = dt;
        for (int axis = 0; axis < 3; ++axis) {
            input.attitude_error_deg[axis] = wrap_error(target[axis], plant.angle[axis]);
            input.body_rate_deg_s[axis] = plant.rate[axis] + noise_rate * rng_signed(&rng);
            input.body_accel_deg_s2[axis] = plant.accel[axis] + noise_accel * rng_signed(&rng);
            input.authority_hint_deg_s2[axis] = hints[axis];
        }

        FlyBrainOutput output;
        if (!flybrain_step(&brain, params, &input, &output)) {
            result.stable = false;
            result.cost = 1e12;
            return result;
        }

        double actuator_next[3];
        for (int axis = 0; axis < 3; ++axis) {
            const double alpha = 1.0 - exp(-dt / fmax(plant.lag_tau[axis], 1e-3));
            actuator_next[axis] = plant.actuator[axis] + alpha * (output.control[axis] - plant.actuator[axis]);
        }

        for (int axis = 0; axis < 3; ++axis) {
            plant.actuator[axis] = actuator_next[axis];
            const double authority_scale = 1.0 + (hard_mode ? 0.38 : 0.25) *
                sin(0.31 * t + plant.authority_phase[axis]);
            double accel = -plant.damping[axis] * plant.rate[axis] + plant.disturbance[axis];
            for (int control_axis = 0; control_axis < 3; ++control_axis) {
                accel += authority_scale * plant.authority[axis][control_axis] * plant.actuator[control_axis];
            }
            if (hard_mode && t > 7.0 && t < 9.0) accel += 2.5 * rng_signed(&rng);
            plant.accel[axis] = accel;
        }

        for (int axis = 0; axis < 3; ++axis) {
            plant.rate[axis] += plant.accel[axis] * dt;
            plant.angle[axis] += plant.rate[axis] * dt;

            const double error = wrap_error(target[axis], plant.angle[axis]);
            const double activity = fabs(output.control[axis] - last_control[axis]) / fmax(dt, 1e-6);
            const double saturation = fabs(output.control[axis]) > 0.985 ? 1.0 : 0.0;

            error_sq_sum += error * error;
            activity_sum += activity;
            sample_count++;
            if (saturation > 0.5) saturation_count++;
            if (t > duration - 2.0) {
                final_error_sum += fabs(error);
                final_sample_count++;
            }
            if (fabs(error) > result.max_abs_error_deg) result.max_abs_error_deg = fabs(error);
            if (fabs(plant.rate[axis]) > result.max_abs_rate_deg_s) result.max_abs_rate_deg_s = fabs(plant.rate[axis]);

            const double rate_excess = fmax(0.0, fabs(plant.rate[axis]) - 8.0);
            result.cost += dt * (0.024 * error * error +
                                 0.010 * plant.rate[axis] * plant.rate[axis] +
                                 0.045 * rate_excess * rate_excess +
                                 0.020 * output.control[axis] * output.control[axis] +
                                 0.004 * activity * activity +
                                 0.35 * saturation);
            last_control[axis] = output.control[axis];

            if (!isfinite(plant.angle[axis]) || !isfinite(plant.rate[axis]) ||
                fabs(plant.rate[axis]) > 90.0 || fabs(error) > 120.0) {
                result.stable = false;
            }
        }
        if (!result.stable) break;
    }

    if (sample_count > 0) {
        result.rms_error_deg = sqrt(error_sq_sum / sample_count);
        result.mean_control_activity = activity_sum / sample_count;
        result.saturation_fraction = (double)saturation_count / sample_count;
    }
    if (final_sample_count > 0) result.final_mean_abs_error_deg = final_error_sum / final_sample_count;

    if (!result.stable) result.cost += 2.0e5 + 500.0 * result.max_abs_rate_deg_s;
    result.cost += 30.0 * result.final_mean_abs_error_deg * result.final_mean_abs_error_deg;
    result.cost += 100.0 * result.saturation_fraction;
    return result;
}

FlyBrainAggregate flybrain_evaluate(const FlyBrainParams *params,
                                    uint64_t seed_base,
                                    int episode_count,
                                    bool hard_mode) {
    FlyBrainAggregate aggregate;
    memset(&aggregate, 0, sizeof(aggregate));
    if (episode_count <= 0) return aggregate;

    int stable_count = 0;
    for (int i = 0; i < episode_count; ++i) {
        const uint64_t seed = seed_base + UINT64_C(0x9e3779b97f4a7c15) * (uint64_t)(i + 1);
        const FlyBrainEpisodeResult r = flybrain_simulate_episode(params, seed, hard_mode);
        aggregate.mean_cost += r.cost;
        aggregate.mean_rms_error_deg += r.rms_error_deg;
        aggregate.mean_final_error_deg += r.final_mean_abs_error_deg;
        aggregate.mean_control_activity += r.mean_control_activity;
        aggregate.saturation_fraction += r.saturation_fraction;
        if (r.final_mean_abs_error_deg > aggregate.worst_final_error_deg) aggregate.worst_final_error_deg = r.final_mean_abs_error_deg;
        if (r.max_abs_rate_deg_s > aggregate.worst_rate_deg_s) aggregate.worst_rate_deg_s = r.max_abs_rate_deg_s;
        if (r.stable) stable_count++;
    }

    const double n = (double)episode_count;
    aggregate.mean_cost /= n;
    aggregate.mean_rms_error_deg /= n;
    aggregate.mean_final_error_deg /= n;
    aggregate.mean_control_activity /= n;
    aggregate.saturation_fraction /= n;
    aggregate.stable_fraction = (double)stable_count / n;
    return aggregate;
}
