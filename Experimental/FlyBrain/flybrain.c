#include "flybrain.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static double fb_clamp(double value, double lower, double upper) {
    if (value < lower) return lower;
    if (value > upper) return upper;
    return value;
}

static double fb_finite(double value, double fallback) {
    return isfinite(value) ? value : fallback;
}

FlyBrainParams flybrain_default_params(void) {
    FlyBrainParams p = {
        .attitude_gain = 0.90,
        .rate_gain = 1.55,
        .indi_gain = 0.34,
        .reference_feedforward = 0.70,
        .accel_filter_alpha = 0.62,
        .authority_rise_tau = 2.20,
        .authority_fall_tau = 0.32,
        .control_slew_per_second = 2.40,
        .max_rate_deg_s = 6.00,
    };
    return p;
}

void flybrain_params_to_array(const FlyBrainParams *p, double v[FLYBRAIN_PARAM_COUNT]) {
    v[0] = p->attitude_gain;
    v[1] = p->rate_gain;
    v[2] = p->indi_gain;
    v[3] = p->reference_feedforward;
    v[4] = p->accel_filter_alpha;
    v[5] = p->authority_rise_tau;
    v[6] = p->authority_fall_tau;
    v[7] = p->control_slew_per_second;
    v[8] = p->max_rate_deg_s;
}

FlyBrainParams flybrain_params_from_array(const double v[FLYBRAIN_PARAM_COUNT]) {
    FlyBrainParams p = {
        .attitude_gain = v[0],
        .rate_gain = v[1],
        .indi_gain = v[2],
        .reference_feedforward = v[3],
        .accel_filter_alpha = v[4],
        .authority_rise_tau = v[5],
        .authority_fall_tau = v[6],
        .control_slew_per_second = v[7],
        .max_rate_deg_s = v[8],
    };
    flybrain_params_clamp(&p);
    return p;
}

void flybrain_params_clamp(FlyBrainParams *p) {
    if (!p) return;
    p->attitude_gain = fb_clamp(fb_finite(p->attitude_gain, 0.90), 0.15, 2.50);
    p->rate_gain = fb_clamp(fb_finite(p->rate_gain, 1.55), 0.20, 6.00);
    p->indi_gain = fb_clamp(fb_finite(p->indi_gain, 0.34), 0.02, 1.50);
    p->reference_feedforward = fb_clamp(fb_finite(p->reference_feedforward, 0.70), 0.0, 1.50);
    p->accel_filter_alpha = fb_clamp(fb_finite(p->accel_filter_alpha, 0.62), 0.0, 0.97);
    p->authority_rise_tau = fb_clamp(fb_finite(p->authority_rise_tau, 2.20), 0.20, 8.00);
    p->authority_fall_tau = fb_clamp(fb_finite(p->authority_fall_tau, 0.32), 0.03, 3.00);
    p->control_slew_per_second = fb_clamp(fb_finite(p->control_slew_per_second, 2.40), 0.15, 8.00);
    p->max_rate_deg_s = fb_clamp(fb_finite(p->max_rate_deg_s, 6.00), 2.0, 8.0);
}

void flybrain_init(FlyBrainState *state, const double hint[FLYBRAIN_AXIS_COUNT]) {
    if (!state) return;
    memset(state, 0, sizeof(*state));
    for (int axis = 0; axis < FLYBRAIN_AXIS_COUNT; ++axis) {
        const double value = hint ? hint[axis] : 0.0;
        state->authority_estimate[axis] =
            isfinite(value) && value > 1.0 ? fb_clamp(value, 5.0, 400.0) : 40.0;
    }
}

static double fb_authority_update(double current,
                                  double observed,
                                  double dt,
                                  const FlyBrainParams *p) {
    if (!isfinite(observed) || observed < 5.0 || observed > 400.0) return current;
    const double tau = observed < current ? p->authority_fall_tau : p->authority_rise_tau;
    const double blend = 1.0 - exp(-dt / fmax(tau, 1e-3));
    return fb_clamp(current + blend * (observed - current), 5.0, 400.0);
}

bool flybrain_step(FlyBrainState *state,
                   const FlyBrainParams *params_in,
                   const FlyBrainInput *input,
                   FlyBrainOutput *output) {
    if (!state || !params_in || !input || !output) return false;
    if (!isfinite(input->dt) || input->dt <= 0.0 || input->dt > 0.5) return false;

    FlyBrainParams p = *params_in;
    flybrain_params_clamp(&p);
    memset(output, 0, sizeof(*output));

    for (int axis = 0; axis < FLYBRAIN_AXIS_COUNT; ++axis) {
        const double error = fb_finite(input->attitude_error_deg[axis], 0.0);
        const double rate = fb_finite(input->body_rate_deg_s[axis], 0.0);
        const double measured_accel = fb_finite(input->body_accel_deg_s2[axis], 0.0);
        const double hint = fb_finite(input->authority_hint_deg_s2[axis], 0.0);

        if (!state->initialized) {
            state->filtered_accel[axis] = measured_accel;
            state->last_filtered_accel[axis] = measured_accel;
            state->last_target_rate[axis] = 0.0;
            if (hint > 1.0) state->authority_estimate[axis] = fb_clamp(hint, 5.0, 400.0);
        }

        const double filtered = p.accel_filter_alpha * state->filtered_accel[axis] +
                                (1.0 - p.accel_filter_alpha) * measured_accel;

        double authority = state->authority_estimate[axis];
        if (state->initialized && fabs(state->last_control_delta[axis]) >= 0.012) {
            const double accel_delta = filtered - state->last_filtered_accel[axis];
            const double observed = fabs(accel_delta / state->last_control_delta[axis]);
            authority = fb_authority_update(authority, observed, input->dt, &p);
        }
        if (hint > 1.0) {
            const double hint_blend = 1.0 - exp(-input->dt / 3.0);
            authority += hint_blend * (fb_clamp(hint, 5.0, 400.0) - authority);
        }

        const double target_rate = p.max_rate_deg_s *
            tanh((p.attitude_gain * error) / fmax(p.max_rate_deg_s, 1e-6));
        const double reference_accel = state->initialized
            ? fb_clamp((target_rate - state->last_target_rate[axis]) / input->dt, -20.0, 20.0)
            : 0.0;
        const double target_accel = p.rate_gain * (target_rate - rate) +
                                    p.reference_feedforward * reference_accel;

        double delta_u = p.indi_gain * (target_accel - filtered) / fmax(authority, 5.0);
        const double max_delta = p.control_slew_per_second * input->dt;
        delta_u = fb_clamp(delta_u, -max_delta, max_delta);
        const double next_control = fb_clamp(state->control[axis] + delta_u, -1.0, 1.0);

        output->control[axis] = next_control;
        output->target_rate_deg_s[axis] = target_rate;
        output->target_accel_deg_s2[axis] = target_accel;
        output->authority_estimate_deg_s2[axis] = authority;

        state->last_control_delta[axis] = next_control - state->control[axis];
        state->control[axis] = next_control;
        state->last_filtered_accel[axis] = filtered;
        state->filtered_accel[axis] = filtered;
        state->authority_estimate[axis] = authority;
        state->last_target_rate[axis] = target_rate;
    }

    state->initialized = true;
    return true;
}

bool flybrain_params_save(const char *path, const FlyBrainParams *p) {
    if (!path || !p) return false;
    FILE *f = fopen(path, "w");
    if (!f) return false;
    fprintf(f, "# %s\n", FLYBRAIN_REVISION);
    fprintf(f, "attitude_gain=%.17g\n", p->attitude_gain);
    fprintf(f, "rate_gain=%.17g\n", p->rate_gain);
    fprintf(f, "indi_gain=%.17g\n", p->indi_gain);
    fprintf(f, "reference_feedforward=%.17g\n", p->reference_feedforward);
    fprintf(f, "accel_filter_alpha=%.17g\n", p->accel_filter_alpha);
    fprintf(f, "authority_rise_tau=%.17g\n", p->authority_rise_tau);
    fprintf(f, "authority_fall_tau=%.17g\n", p->authority_fall_tau);
    fprintf(f, "control_slew_per_second=%.17g\n", p->control_slew_per_second);
    fprintf(f, "max_rate_deg_s=%.17g\n", p->max_rate_deg_s);
    return fclose(f) == 0;
}

bool flybrain_params_load(const char *path, FlyBrainParams *p) {
    if (!path || !p) return false;
    FILE *f = fopen(path, "r");
    if (!f) return false;

    FlyBrainParams loaded = flybrain_default_params();
    char line[256];
    while (fgets(line, sizeof(line), f)) {
        char key[96];
        double value = 0.0;
        if (sscanf(line, " %95[^=]=%lf", key, &value) != 2) continue;
        if (strcmp(key, "attitude_gain") == 0) loaded.attitude_gain = value;
        else if (strcmp(key, "rate_gain") == 0) loaded.rate_gain = value;
        else if (strcmp(key, "indi_gain") == 0) loaded.indi_gain = value;
        else if (strcmp(key, "reference_feedforward") == 0) loaded.reference_feedforward = value;
        else if (strcmp(key, "accel_filter_alpha") == 0) loaded.accel_filter_alpha = value;
        else if (strcmp(key, "authority_rise_tau") == 0) loaded.authority_rise_tau = value;
        else if (strcmp(key, "authority_fall_tau") == 0) loaded.authority_fall_tau = value;
        else if (strcmp(key, "control_slew_per_second") == 0) loaded.control_slew_per_second = value;
        else if (strcmp(key, "max_rate_deg_s") == 0) loaded.max_rate_deg_s = value;
    }
    fclose(f);
    flybrain_params_clamp(&loaded);
    *p = loaded;
    return true;
}
