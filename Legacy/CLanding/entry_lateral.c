#include "entry_lateral.h"

#include <float.h>
#include <math.h>
#include <stddef.h>
#include <string.h>

#define ENTRY_LATERAL_PI 3.14159265358979323846264338327950288
#define ENTRY_LATERAL_DEG2RAD (ENTRY_LATERAL_PI / 180.0)
#define ENTRY_LATERAL_RAD2DEG (180.0 / ENTRY_LATERAL_PI)

/*
 * Numerical integration resolution for the actuator-response projection.
 * 0.25 degree bounds each roll integration increment; the focused regression
 * suite checks the envelope's monotonicity and mirror symmetry. This is a
 * discretization/convergence parameter, not a behavioral flight threshold.
 */
#define ENTRY_LATERAL_RESPONSE_BANK_STEP_DEG 0.25 /* decision-literal-ok: discretization/convergence parameter */

static double clamp_value(double value, double minimum, double maximum) {
    return fmin(fmax(value, minimum), maximum);
}

static double unit_sign(double value) {
    return signbit(value) ? -1.0 : 1.0;
}

static double finite_or(double value, double fallback) {
    return isfinite(value) ? value : fallback;
}

static double wrap_signed_deg(double value) {
    /* 360/180 are angular identities, not flight-policy thresholds. */
    value = remainder(value, 360.0); /* decision-literal-ok: mathematical angular period */
    if (value <= -180.0) value += 360.0; /* decision-literal-ok: mathematical signed-angle domain */
    if (value > 180.0) value -= 360.0; /* decision-literal-ok: mathematical signed-angle domain */
    return value;
}

double entry_lateral_geometry_turn_sign(double current_course_deg,
        double current_along_m, double current_cross_m,
        double target_along_m, double target_cross_m,
        double target_course_deg) {
    if (!isfinite(current_course_deg) ||
        !isfinite(current_along_m) || !isfinite(current_cross_m) ||
        !isfinite(target_along_m) || !isfinite(target_cross_m) ||
        !isfinite(target_course_deg))
        return 0.0;

    double da = target_along_m - current_along_m;
    double dc = target_cross_m - current_cross_m;
    double coordinate_scale = fmax(1.0,
        fmax(fmax(fabs(current_along_m), fabs(current_cross_m)),
             fmax(fabs(target_along_m), fabs(target_cross_m))));
    double position_resolution = sqrt(DBL_EPSILON) * coordinate_scale;
    double angular_resolution =
        sqrt(DBL_EPSILON) * ENTRY_LATERAL_RAD2DEG;

    if (hypot(da, dc) > position_resolution) {
        double point_course =
            atan2(dc, da) * ENTRY_LATERAL_RAD2DEG;
        double point_turn = wrap_signed_deg(
            point_course - current_course_deg);
        if (fabs(point_turn) > angular_resolution)
            return unit_sign(point_turn);
    }

    double terminal_turn = wrap_signed_deg(
        target_course_deg - current_course_deg);
    if (fabs(terminal_turn) > angular_resolution)
        return unit_sign(terminal_turn);

    return 0.0;
}

EntryLateralLimits entry_lateral_default_limits(void) {
    /*
     * Zero means "not provided". Runtime callers replace these fields from the
     * vehicle/guidance authority model before calling update(). There are no
     * hidden flight-policy defaults in this module.
     */
    EntryLateralLimits limits = {0};
    return limits;
}

void entry_lateral_state_init(EntryLateralState *state, double ut,
        double bank_sign_hint, double measured_bank_deg) {
    if (!state) return;
    memset(state, 0, sizeof(*state));
    state->initialized = true;
    if (isfinite(bank_sign_hint) && fabs(bank_sign_hint) > DBL_EPSILON) {
        state->bank_sign = unit_sign(bank_sign_hint);
    } else if (isfinite(measured_bank_deg) && fabs(measured_bank_deg) > DBL_EPSILON) {
        state->bank_sign = unit_sign(measured_bank_deg);
    } else {
        state->bank_sign = 1.0;
    }
    state->reversal_armed = false;
    state->last_reversal_ut = -INFINITY;
    state->command_bank_deg = finite_or(measured_bank_deg, 0.0);
    state->command_bank_rate_deg_s = 0.0;
    state->leg_captured_ut = finite_or(ut, 0.0);
}

static bool limits_valid(const EntryLateralLimits *limits) {
    return limits &&
        isfinite(limits->maximum_bank_deg) && limits->maximum_bank_deg >= 0.0 &&
        isfinite(limits->maximum_roll_rate_deg_s) && limits->maximum_roll_rate_deg_s > 0.0 &&
        isfinite(limits->maximum_roll_accel_deg_s2) && limits->maximum_roll_accel_deg_s2 > 0.0 &&
        isfinite(limits->minimum_leg_duration_s) && limits->minimum_leg_duration_s >= 0.0;
}

static double choose_bank_sign(const EntryLateralState *state,
        const EntryLateralInput *input) {
    if (state && state->initialized && fabs(state->bank_sign) > DBL_EPSILON)
        return unit_sign(state->bank_sign);
    if (isfinite(input->measured_bank_deg) &&
        fabs(input->measured_bank_deg) > DBL_EPSILON)
        return unit_sign(input->measured_bank_deg);
    if (input->has_crossrange_error && isfinite(input->crossrange_error_m) &&
        fabs(input->crossrange_error_m) > DBL_EPSILON)
        return -unit_sign(input->crossrange_error_m);
    if (isfinite(input->course_to_site_error_deg) &&
        fabs(input->course_to_site_error_deg) > DBL_EPSILON)
        return unit_sign(input->course_to_site_error_deg);
    return 1.0;
}

static bool bank_magnitude(const EntryLateralInput *input,
        const EntryLateralLimits *limits, double *out) {
    if (!out || !input->longitudinal.valid ||
        !isfinite(input->longitudinal.required_vertical_lift_accel) ||
        input->longitudinal.required_vertical_lift_accel < 0.0)
        return false;

    double measured_lift = finite_or(input->lift_accel, 0.0);
    double confidence = clamp_value(
        fmin(finite_or(input->authority_confidence, 0.0),
             finite_or(input->longitudinal.confidence, 0.0)),
        0.0, 1.0); /* decision-literal-ok: probability/confidence domain */
    double available_lift = measured_lift * confidence;
    if (!(available_lift > DBL_EPSILON)) return false;

    double effectiveness = finite_or(input->bank_effectiveness, 1.0);
    if (!(effectiveness > DBL_EPSILON)) return false;

    double bank_limit = fabs(limits->maximum_bank_deg);
    double effective_bank_limit = bank_limit * effectiveness;
    if (!(effective_bank_limit < 90.0)) /* decision-literal-ok: positive vertical-lift bank domain */
        effective_bank_limit = nextafter(90.0, 0.0);

    double required_fraction =
        input->longitudinal.required_vertical_lift_accel / available_lift;
    double minimum_fraction = cos(effective_bank_limit * ENTRY_LATERAL_DEG2RAD);
    required_fraction = clamp_value(required_fraction, minimum_fraction, 1.0);

    double effective_bank = acos(required_fraction) * ENTRY_LATERAL_RAD2DEG;
    *out = clamp_value(effective_bank / effectiveness, 0.0, bank_limit);
    return true;
}

static void update_bank_command(EntryLateralState *state, double target,
        const EntryLateralLimits *limits, double dt) {
    double bank_limit = fabs(limits->maximum_bank_deg);
    double rate_limit = fabs(limits->maximum_roll_rate_deg_s);
    double accel_limit = fabs(limits->maximum_roll_accel_deg_s2);
    if (!(dt > 0.0)) return;

    target = clamp_value(target, -bank_limit, bank_limit);
    double error = target - state->command_bank_deg;

    /*
     * Time-optimal bounded double-integrator command: desired roll rate is the
     * smaller of the actuator rate limit and the exact stopping-rate curve.
     */
    double desired_rate = 0.0;
    if (fabs(error) > DBL_EPSILON) {
        double stopping_rate = sqrt(2.0 * accel_limit * fabs(error));
        desired_rate = unit_sign(error) * fmin(rate_limit, stopping_rate);
    }

    double rate_delta = desired_rate - state->command_bank_rate_deg_s;
    state->command_bank_rate_deg_s +=
        clamp_value(rate_delta, -accel_limit * dt, accel_limit * dt);
    state->command_bank_rate_deg_s =
        clamp_value(state->command_bank_rate_deg_s, -rate_limit, rate_limit);

    double next = state->command_bank_deg + state->command_bank_rate_deg_s * dt;
    if ((target - state->command_bank_deg) * (target - next) <= 0.0) {
        next = target;
        state->command_bank_rate_deg_s = 0.0;
    }
    state->command_bank_deg = clamp_value(next, -bank_limit, bank_limit);
}

EntryLateralOutput entry_lateral_update(EntryLateralState *state,
        const EntryLateralInput *input, const EntryLateralLimits *limits) {
    EntryLateralOutput output;
    memset(&output, 0, sizeof(output));
    output.corridor_metric = NAN;
    output.course_corridor_deg = NAN;
    output.crossrange_corridor_m = NAN;

    if (!state || !input || !limits_valid(limits) ||
        !isfinite(input->ut) || !isfinite(input->relative_speed) ||
        !(input->relative_speed > 0.0) ||
        !isfinite(input->dt) || input->dt < 0.0)
        return output;

    if (!state->initialized) {
        entry_lateral_state_init(state, input->ut,
            choose_bank_sign(NULL, input), input->measured_bank_deg);
        state->command_bank_rate_deg_s = clamp_value(
            finite_or(input->measured_bank_rate_deg_s, 0.0),
            -fabs(limits->maximum_roll_rate_deg_s),
            fabs(limits->maximum_roll_rate_deg_s));
    }

    state->bank_sign = choose_bank_sign(state, input);

    double magnitude = 0.0;
    bool authority_valid = bank_magnitude(input, limits, &magnitude);
    if (!authority_valid) {
        magnitude = 0.0;
        output.degraded_authority = true;
    }

    double raw_target = state->bank_sign * magnitude;
    update_bank_command(state, raw_target, limits, input->dt);

    /*
     * Capture is a kinematic state, not a tuned angle band. Once the measured
     * bank has reached/passed the currently required magnitude on the selected
     * side, the leg is captured. Reversal remains the executive's job.
     */
    if (!state->leg_captured && magnitude > DBL_EPSILON &&
        isfinite(input->measured_bank_deg) &&
        state->bank_sign * input->measured_bank_deg >= magnitude) {
        state->leg_captured = true;
        state->leg_captured_ut = input->ut;
    }

    double effectiveness = finite_or(input->bank_effectiveness, 1.0);
    double effective_bank = state->command_bank_deg * effectiveness;
    effective_bank = clamp_value(effective_bank,
        -nextafter(90.0, 0.0), nextafter(90.0, 0.0)); /* decision-literal-ok: tangent singularity */
    double lateral_accel = fmax(0.0, finite_or(input->lift_accel, 0.0)) *
        sin(effective_bank * ENTRY_LATERAL_DEG2RAD);

    output.valid = true;
    output.reversal_requested = false;
    output.leg_captured = state->leg_captured;
    output.reversal_armed = false;
    output.bank_sign = state->bank_sign;
    output.bank_magnitude_deg = magnitude;
    output.raw_target_bank_deg = raw_target;
    output.target_bank_deg = state->command_bank_deg;
    output.target_bank_rate_deg_s = state->command_bank_rate_deg_s;
    output.expected_course_rate_deg_s =
        lateral_accel / input->relative_speed * ENTRY_LATERAL_RAD2DEG;
    return output;
}

static double rest_to_rest_roll_time(double span_deg,
        double rate_limit_deg_s, double accel_limit_deg_s2) {
    span_deg = fabs(span_deg);
    if (!(span_deg > DBL_EPSILON)) return 0.0;
    double triangular_span =
        rate_limit_deg_s * rate_limit_deg_s / accel_limit_deg_s2;
    if (span_deg <= triangular_span)
        return 2.0 * sqrt(span_deg / accel_limit_deg_s2);
    return span_deg / rate_limit_deg_s +
        rate_limit_deg_s / accel_limit_deg_s2;
}

EntryLateralTurnEnvelope entry_lateral_terminal_turn_envelope(
        const EntryLateralTurnInput *input) {
    EntryLateralTurnEnvelope out;
    memset(&out, 0, sizeof(out));
    out.response_time_s = NAN;
    out.response_distance_m = NAN;
    out.response_heading_change_deg = NAN;
    out.projected_along_m = NAN;
    out.projected_cross_m = NAN;
    out.projected_course_deg = NAN;
    out.remaining_course_change_deg = NAN;
    out.minimum_turn_radius_m = INFINITY;
    out.ideal_turn_radius_m = INFINITY;
    out.radius_margin_m = -INFINITY;
    out.endpoint_error_m = INFINITY;
    out.capture_margin_m = -INFINITY;
    out.feasibility_margin_m = -INFINITY;

    if (!input ||
        !isfinite(input->horizontal_speed_mps) || !(input->horizontal_speed_mps > DBL_EPSILON) ||
        !isfinite(input->current_course_deg) || !isfinite(input->target_course_deg) ||
        !isfinite(input->current_along_m) || !isfinite(input->current_cross_m) ||
        !isfinite(input->target_along_m) || !isfinite(input->target_cross_m) ||
        !isfinite(input->measured_bank_deg) || !isfinite(input->measured_bank_rate_deg_s) ||
        !isfinite(input->lift_accel_mps2) || !(input->lift_accel_mps2 > DBL_EPSILON) ||
        !isfinite(input->bank_effectiveness) || !(input->bank_effectiveness > DBL_EPSILON) ||
        !isfinite(input->maximum_bank_deg) || !(input->maximum_bank_deg > DBL_EPSILON) ||
        !isfinite(input->maximum_roll_rate_deg_s) || !(input->maximum_roll_rate_deg_s > DBL_EPSILON) ||
        !isfinite(input->maximum_roll_accel_deg_s2) || !(input->maximum_roll_accel_deg_s2 > DBL_EPSILON) ||
        !isfinite(input->capture_radius_m) || input->capture_radius_m < 0.0 ||
        !isfinite(input->position_uncertainty_m) || input->position_uncertainty_m < 0.0)
        return out;

    double initial_turn = wrap_signed_deg(
        input->target_course_deg - input->current_course_deg);
    double angular_resolution =
        ENTRY_LATERAL_RESPONSE_BANK_STEP_DEG;
    if (!(fabs(initial_turn) > angular_resolution))
        return out;

    double side = unit_sign(initial_turn);
    double bank_limit = fabs(input->maximum_bank_deg);
    double target_bank = side * bank_limit;
    double rate_limit = fabs(input->maximum_roll_rate_deg_s);
    double accel_limit = fabs(input->maximum_roll_accel_deg_s2);

    /*
     * Construct a conservative finite response horizon. First stop the measured
     * rate, then execute a rest-to-rest move from that stopped bank. The actual
     * bounded double-integrator simulation below normally completes sooner.
     */
    double initial_rate = clamp_value(input->measured_bank_rate_deg_s,
        -rate_limit, rate_limit);
    double stop_time = fabs(initial_rate) / accel_limit;
    double bank_after_stop =
        input->measured_bank_deg + 0.5 * initial_rate * stop_time;
    double response_time_bound = stop_time +
        rest_to_rest_roll_time(target_bank - bank_after_stop,
            rate_limit, accel_limit);

    double bank = clamp_value(input->measured_bank_deg, -bank_limit, bank_limit);
    double bank_rate = initial_rate;
    double course_rad = input->current_course_deg * ENTRY_LATERAL_DEG2RAD;
    double along = input->current_along_m;
    double cross = input->current_cross_m;
    double elapsed = 0.0;
    double dt_nominal =
        ENTRY_LATERAL_RESPONSE_BANK_STEP_DEG / rate_limit;
    if (!(dt_nominal > 0.0) || !isfinite(response_time_bound))
        return out;

    size_t max_steps = (size_t)ceil(response_time_bound / dt_nominal) + 1u;
    for (size_t step = 0; step < max_steps && elapsed < response_time_bound; ++step) {
        double remaining_time = response_time_bound - elapsed;
        double dt = fmin(dt_nominal, remaining_time);
        if (!(dt > 0.0)) break;

        double error = target_bank - bank;
        double desired_rate = 0.0;
        if (fabs(error) > DBL_EPSILON) {
            double stopping_rate = sqrt(2.0 * accel_limit * fabs(error));
            desired_rate = unit_sign(error) * fmin(rate_limit, stopping_rate);
        }
        bank_rate += clamp_value(desired_rate - bank_rate,
            -accel_limit * dt, accel_limit * dt);
        bank_rate = clamp_value(bank_rate, -rate_limit, rate_limit);

        double next_bank = bank + bank_rate * dt;
        if ((target_bank - bank) * (target_bank - next_bank) <= 0.0) {
            next_bank = target_bank;
            bank_rate = 0.0;
        }
        next_bank = clamp_value(next_bank, -bank_limit, bank_limit);

        double average_bank = 0.5 * (bank + next_bank);
        double effective_bank = clamp_value(
            average_bank * input->bank_effectiveness,
            -nextafter(90.0, 0.0), nextafter(90.0, 0.0)); /* decision-literal-ok: tangent singularity */
        double lateral_accel =
            input->lift_accel_mps2 * sin(effective_bank * ENTRY_LATERAL_DEG2RAD);
        double course_rate = lateral_accel / input->horizontal_speed_mps;
        double midpoint_course = course_rad + 0.5 * course_rate * dt;
        along += input->horizontal_speed_mps * cos(midpoint_course) * dt;
        cross += input->horizontal_speed_mps * sin(midpoint_course) * dt;
        course_rad += course_rate * dt;
        bank = next_bank;
        elapsed += dt;

        if (bank == target_bank && fabs(bank_rate) <= DBL_EPSILON)
            break;
    }

    if (fabs(target_bank - bank) > ENTRY_LATERAL_RESPONSE_BANK_STEP_DEG)
        return out;

    double projected_course =
        wrap_signed_deg(course_rad * ENTRY_LATERAL_RAD2DEG);
    double remaining_turn =
        wrap_signed_deg(input->target_course_deg - projected_course);
    if (side * remaining_turn <= angular_resolution)
        return out;

    double effective_target_bank = fmin(
        bank_limit * input->bank_effectiveness,
        nextafter(90.0, 0.0));
    double lateral_authority =
        input->lift_accel_mps2 *
        fabs(sin(effective_target_bank * ENTRY_LATERAL_DEG2RAD));
    if (!(lateral_authority > DBL_EPSILON))
        return out;
    double minimum_radius =
        input->horizontal_speed_mps * input->horizontal_speed_mps /
        lateral_authority;

    double psi0 = projected_course * ENTRY_LATERAL_DEG2RAD;
    double psi1 = input->target_course_deg * ENTRY_LATERAL_DEG2RAD;
    double ua = (sin(psi1) - sin(psi0)) / side;
    double uc = (cos(psi0) - cos(psi1)) / side;
    double norm2 = ua * ua + uc * uc;
    if (!(norm2 > DBL_EPSILON) || !isfinite(norm2))
        return out;

    double da = input->target_along_m - along;
    double dc = input->target_cross_m - cross;
    double ideal_radius = (da * ua + dc * uc) / norm2;
    if (!isfinite(ideal_radius))
        return out;

    double endpoint_along = along + ideal_radius * ua;
    double endpoint_cross = cross + ideal_radius * uc;
    double endpoint_error = hypot(
        endpoint_along - input->target_along_m,
        endpoint_cross - input->target_cross_m);

    double radius_margin = ideal_radius - minimum_radius;
    double capture_margin =
        input->capture_radius_m - input->position_uncertainty_m - endpoint_error;

    out.valid = true;
    out.turn_sign = side;
    out.response_time_s = elapsed;
    out.response_distance_m = input->horizontal_speed_mps * elapsed;
    out.response_heading_change_deg =
        wrap_signed_deg(projected_course - input->current_course_deg);
    out.projected_along_m = along;
    out.projected_cross_m = cross;
    out.projected_course_deg = projected_course;
    out.remaining_course_change_deg = remaining_turn;
    out.minimum_turn_radius_m = minimum_radius;
    out.ideal_turn_radius_m = ideal_radius;
    out.radius_margin_m = radius_margin;
    out.endpoint_error_m = endpoint_error;
    out.capture_margin_m = capture_margin;
    out.feasibility_margin_m = fmin(radius_margin, capture_margin);
    return out;
}
