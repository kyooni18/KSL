#include "flight_control.h"

#include <float.h>
#include <math.h>
#include <stddef.h>
#include <string.h>

static double fc_finite(double value, double fallback) {
    return isfinite(value) ? value : fallback;
}

static double fc_clamp(double value, double lower, double upper) {
    if (value < lower) return lower;
    if (value > upper) return upper;
    return value;
}

static double fc_signed_angle(double value) {
    double wrapped = fmod(value, 360.0);
    if (wrapped < 0.0) wrapped += 360.0;
    return wrapped > 180.0 ? wrapped - 360.0 : wrapped;
}

static bool fc_atmospheric_profile(ControlProfile profile) {
    return profile == PROFILE_ENTRY || profile == PROFILE_TAEM ||
           profile == PROFILE_APPROACH || profile == PROFILE_FLARE ||
           profile == PROFILE_RECOVERY;
}

static void fc_axis_set_reported(FlightControlAxisAuthority *axis, double value) {
    double reported = fmax(0.0, fc_finite(value, 0.0));
    if (reported <= 0.0) return;
    axis->reported_torque_accel = reported;
    if (!axis->reported_initialized || axis->torque_accel <= 0.0) {
        axis->torque_accel = reported;
        axis->reported_initialized = true;
    }
}

static double fc_axis_available(const FlightControlAxisAuthority *axis, double q) {
    if (axis->controlled_confidence > 0.0 &&
        isfinite(axis->controlled_accel) && axis->controlled_accel > 0.0)
        return axis->controlled_accel;
    double baseline = fmax(0.0, fc_finite(axis->torque_accel, 0.0));
    double aerodynamic = fmax(0.0, fc_finite(axis->aero_per_q, 0.0)) * fmax(0.0, q);
    return fmax(DBL_EPSILON, baseline + aerodynamic);
}

static bool fc_axis_control_authority_known(const FlightControlAxisAuthority *axis) {
    return axis && axis->controlled_confidence > 0.0 &&
        isfinite(axis->controlled_accel) && axis->controlled_accel > 0.0;
}

static void fc_axis_observe(FlightControlAxisAuthority *axis,
                            double rate,
                            double previous_input,
                            double dt,
                            double cross_axis_input) {
    if (!axis || !isfinite(rate) || !isfinite(dt) || !(dt > 0.0)) return;
    if (!axis->has_last_rate) {
        axis->last_rate = rate;
        axis->has_last_rate = true;
        return;
    }

    double acceleration = (rate - axis->last_rate) / dt;
    axis->last_rate = rate;
    if (!isfinite(acceleration)) return;

    double input = fc_finite(previous_input, 0.0);
    double magnitude = fabs(input);
    double interference = fabs(fc_finite(cross_axis_input, 0.0));
    if (!(magnitude > DBL_EPSILON)) {
        /* With no commanded excitation, the observed acceleration is the
           disturbance estimate. No arbitrary deadband or decay time is used. */
        axis->drift_accel = acceleration;
        return;
    }

    double excitation_weight = magnitude /
        (magnitude + interference + DBL_EPSILON);
    double controlled = (acceleration - axis->drift_accel) / input;
    if (!isfinite(controlled) || !(controlled > 0.0)) return;

    /*
     * This estimate is deliberately kept in the controlled coordinate.  A
     * torque/inertia report is body-axis actuator capability; subtracting it
     * from d(AoA)/dt or d(bank)/dt acceleration mixes coordinates and can make
     * a stopping-distance calculation optimistically wrong.
     */
    if (!(axis->controlled_accel > 0.0) || axis->controlled_confidence <= 0.0)
        axis->controlled_accel = controlled;
    else
        axis->controlled_accel += excitation_weight *
            (controlled - axis->controlled_accel);
    axis->controlled_confidence =
        1.0 - (1.0 - axis->controlled_confidence) *
        (1.0 - excitation_weight);
}

static double fc_reported_axis_accel(const Telemetry *t, FlightControlAxis axis) {
    if (!t->has_torque || !t->has_inertia) return 0.0;
    double torque = 0.0;
    double inertia = 0.0;
    switch (axis) {
        case FLIGHT_CONTROL_AXIS_PITCH:
            torque = t->available_pitch_torque;
            inertia = t->pitch_moment_of_inertia;
            break;
        case FLIGHT_CONTROL_AXIS_ROLL:
            torque = t->available_roll_torque;
            inertia = t->roll_moment_of_inertia;
            break;
        case FLIGHT_CONTROL_AXIS_YAW:
            torque = t->available_yaw_torque;
            inertia = t->yaw_moment_of_inertia;
            break;
        default:
            return 0.0;
    }
    if (!isfinite(torque) || !isfinite(inertia) || inertia <= 0.0) return 0.0;
    return fabs(torque) / inertia * RAD2DEG;
}

void flight_control_init(FlightControlState *state, double nominal_dt) {
    if (!state) return;
    memset(state, 0, sizeof(*state));
    state->nominal_dt = isfinite(nominal_dt) && nominal_dt > 0.0 ? nominal_dt : 0.1;
}

void flight_control_reset_transients(FlightControlState *state) {
    if (!state) return;
    state->has_sample = false;
    state->pitch_rate = 0.0;
    state->aoa_rate = 0.0;
    state->roll_rate = 0.0;
    state->heading_rate = 0.0;
    state->sideslip_rate = 0.0;
    state->pitch_trim = 0.0;
    state->terminal_pitch_integral = 0.0;
    state->roll_trim = 0.0;
    state->has_terminal_pitch_authority = false;
    state->has_terminal_roll_authority = false;
    state->has_last_target_roll = false;
    state->target_roll_rate = 0.0;
    state->rcs_transonic_cutoff = false;
    state->has_last_profile = false;
    memset(state->last_control, 0, sizeof(state->last_control));
    memset(&state->beta, 0, sizeof(state->beta));
    for (int i = 0; i < FLIGHT_CONTROL_AXIS_COUNT; ++i) {
        state->authority[i].has_last_rate = false;
    }
}

void flight_control_seed_axis_authority(FlightControlState *state,
                                        FlightControlAxis axis,
                                        double non_aero_accel,
                                        double aero_accel_per_pascal,
                                        double confidence) {
    if (!state || axis < 0 || axis >= FLIGHT_CONTROL_AXIS_COUNT) return;
    FlightControlAxisAuthority *estimate = &state->authority[axis];
    estimate->torque_accel = fmax(0.0, fc_finite(non_aero_accel, 0.0));
    estimate->aero_per_q = fmax(0.0, fc_finite(aero_accel_per_pascal, 0.0));
    estimate->confidence = fc_clamp(fc_finite(confidence, 0.0), 0.0, 1.0);
    estimate->non_aero_confidence = estimate->confidence;
    if (estimate->torque_accel > 0.0) estimate->reported_initialized = true;
    if (estimate->confidence > 0.0 && estimate->torque_accel > 0.0) {
        estimate->controlled_accel = estimate->torque_accel;
        estimate->controlled_confidence = estimate->confidence;
    }
}

const char *flight_control_revision(void) {
    return FLIGHT_CONTROL_REVISION;
}


static double fc_bounded_axis_command(double angle_error,
        double measured_rate,double target_rate,double authority,
        bool stopping_authority_known,double drift_accel,double hold_seconds) {
    double available=fabs(fc_finite(authority,0.0));
    if(!(available>DBL_EPSILON))return 0.0;

    double relative_rate=measured_rate-target_rate;
    double desired_accel=0.0;
    if(fabs(angle_error)<=DBL_EPSILON){
        if(fabs(relative_rate)>DBL_EPSILON)
            desired_accel=-copysign(available,relative_rate);
    }else{
        double direction=copysign(1.0,angle_error);
        double distance=fabs(angle_error);
        double toward_rate=direction*relative_rate;

        if(!stopping_authority_known&&toward_rate>0.0){
            desired_accel=-direction*available;
        }else{
            double stopping_distance=toward_rate>0.0?
                toward_rate*toward_rate/(2.0*available):0.0;
            if(stopping_distance>=distance){
                desired_accel=-direction*available;
            }else{
                desired_accel=direction*available;
                double hold=fc_finite(hold_seconds,0.0);
                if(hold>0.0){
                    double end_rate=toward_rate+available*hold;
                    double remaining=distance-toward_rate*hold-
                        0.5*available*hold*hold;
                    double end_stop=end_rate>0.0?
                        end_rate*end_rate/(2.0*available):0.0;
                    if(end_rate>0.0&&remaining<=end_stop){
                        double qa=hold*hold;
                        double qb=available*hold*hold+
                            2.0*toward_rate*hold;
                        double qc=toward_rate*toward_rate+
                            2.0*available*toward_rate*hold-
                            2.0*available*distance;
                        double disc=qb*qb-4.0*qa*qc;
                        if(disc>=0.0&&isfinite(disc)){
                            double toward_accel=(-qb+sqrt(disc))/(2.0*qa);
                            toward_accel=fc_clamp(toward_accel,-available,available);
                            desired_accel=direction*toward_accel;
                        }
                    }
                }
            }
        }
    }
    return fc_clamp((desired_accel-fc_finite(drift_accel,0.0))/available,-1.0,1.0);
}

bool flight_control_step(FlightControlState *state,
                         const Telemetry *telemetry,
                         const GuidanceCommand *command,
                         double sample_dt,
                         FlightControlOutput *output) {
    if (!state || !telemetry || !command || !output) return false;
    memset(output, 0, sizeof(*output));

    ControlProfile profile = command->control_profile;
    output->throttle = fc_clamp(fc_finite(command->target_throttle, 0.0), 0.0, 1.0);
    output->wheel_steering = fc_clamp(fc_finite(command->wheel_steering, 0.0), -1.0, 1.0);
    output->gear = command->gear;
    output->brakes = command->brakes;
    output->airbrakes = profile == PROFILE_ROLLOUT && command->airbrakes;
    output->diagnostics.profile = profile;

    if (profile == PROFILE_ROLLOUT) {
        output->valid = true;
        output->diagnostics.rcs_transonic_cutoff = true;
        return true;
    }
    if (!fc_atmospheric_profile(profile)) return false;

    double measured_dt = isfinite(sample_dt) && sample_dt > 0.0 ? sample_dt : 0.0;
    if (state->has_sample && isfinite(telemetry->ut) && telemetry->ut < state->last_ut) {
        flight_control_reset_transients(state);
    }
    if (!(measured_dt > 0.0) && state->has_sample && isfinite(telemetry->ut))
        measured_dt = telemetry->ut - state->last_ut;
    double control_dt = measured_dt > 0.0 ? measured_dt : state->nominal_dt;
    if (!(control_dt > 0.0) || !isfinite(control_dt)) return false;
    bool sample_valid = state->has_sample && measured_dt > 0.0;

    double pitch = fc_finite(telemetry->pitch, 0.0);
    double aoa = fc_finite(telemetry->angle_of_attack, 0.0);
    double roll = fc_signed_angle(fc_finite(telemetry->roll, 0.0));
    double heading = fc_signed_angle(fc_finite(telemetry->heading, 0.0));
    double sideslip = fc_signed_angle(fc_finite(telemetry->sideslip, 0.0));
    double q = fmax(0.0, fc_finite(telemetry->dynamic_pressure, 0.0));

    double vertical_speed = fc_finite(telemetry->vertical_speed, 0.0);
    double horizontal_speed = fabs(fc_finite(telemetry->horizontal_speed, 0.0));
    double flight_path_angle = isfinite(telemetry->flight_path_angle)
        ? telemetry->flight_path_angle
        : atan2(vertical_speed, fmax(horizontal_speed, DBL_EPSILON)) * RAD2DEG;

    if (sample_valid) {
        state->pitch_rate = (pitch - state->last_pitch) / measured_dt;
        state->aoa_rate = fc_signed_angle(aoa - state->last_aoa) / measured_dt;
        state->roll_rate = fc_signed_angle(roll - state->last_roll) / measured_dt;
        state->heading_rate = fc_signed_angle(heading - state->last_heading) / measured_dt;
        state->sideslip_rate =
            fc_signed_angle(sideslip - state->last_sideslip) / measured_dt;
    }

    for (int i = 0; i < FLIGHT_CONTROL_AXIS_COUNT; ++i)
        fc_axis_set_reported(&state->authority[i],
            fc_reported_axis_accel(telemetry, (FlightControlAxis)i));
    if(telemetry->attitude_response.pitch_valid){
        state->authority[FLIGHT_CONTROL_AXIS_PITCH].controlled_accel=
            telemetry->attitude_response.maximum_pitch_accel_deg_s2;
        state->authority[FLIGHT_CONTROL_AXIS_PITCH].controlled_confidence=1.0;
    }
    if(telemetry->attitude_response.roll_valid){
        state->authority[FLIGHT_CONTROL_AXIS_ROLL].controlled_accel=
            telemetry->attitude_response.maximum_roll_accel_deg_s2;
        state->authority[FLIGHT_CONTROL_AXIS_ROLL].controlled_confidence=1.0;
    }

    bool body_pitch_rate_valid =
        telemetry->has_body_pitch_rate && isfinite(telemetry->body_pitch_rate);
    bool body_roll_rate_valid =
        telemetry->has_body_roll_rate && isfinite(telemetry->body_roll_rate);
    bool body_yaw_rate_valid =
        telemetry->has_body_yaw_rate && isfinite(telemetry->body_yaw_rate);

    /*
     * A body-rate stream that is exactly stationary while the independently
     * differentiated controlled coordinate is moving is stale. This is a data
     * consistency test, not a rate threshold.
     */
    bool body_pitch_rate_stale = body_pitch_rate_valid &&
        fabs(telemetry->body_pitch_rate)<=DBL_EPSILON &&
        fabs(state->aoa_rate)>DBL_EPSILON;
    bool body_roll_rate_stale = body_roll_rate_valid &&
        fabs(telemetry->body_roll_rate)<=DBL_EPSILON &&
        fabs(state->roll_rate)>DBL_EPSILON;

    /*
     * Pitch and roll authority must be identified in the coordinates actually
     * controlled below. Body p/q remain diagnostics; they are not substituted
     * for d(bank)/dt or d(AoA)/dt.
     */
    double observed_pitch_rate=state->aoa_rate;
    double observed_roll_rate=state->roll_rate;
    double observed_yaw_rate=
        body_yaw_rate_valid?telemetry->body_yaw_rate:state->heading_rate;

    if (sample_valid) {
        fc_axis_observe(&state->authority[FLIGHT_CONTROL_AXIS_PITCH],
            observed_pitch_rate,state->last_control[FLIGHT_CONTROL_AXIS_PITCH],
            measured_dt,0.0);
        fc_axis_observe(&state->authority[FLIGHT_CONTROL_AXIS_ROLL],
            observed_roll_rate,state->last_control[FLIGHT_CONTROL_AXIS_ROLL],
            measured_dt,state->last_control[FLIGHT_CONTROL_AXIS_YAW]);
        fc_axis_observe(&state->authority[FLIGHT_CONTROL_AXIS_YAW],
            observed_yaw_rate,state->last_control[FLIGHT_CONTROL_AXIS_YAW],
            measured_dt,0.0);
    }


    double target_pitch_state = command->has_target_aoa && isfinite(command->target_aoa)
        ? command->target_aoa
        : fc_finite(command->target_pitch, 0.0) - flight_path_angle;
    double pitch_error = target_pitch_state - aoa;
    double roll_target = fc_signed_angle(fc_finite(command->target_roll, 0.0));
    double roll_error = fc_signed_angle(roll_target - roll);

    if (state->has_last_target_roll) {
        state->target_roll_rate =
            fc_signed_angle(roll_target - state->last_target_roll) / control_dt;
    } else {
        state->target_roll_rate = 0.0;
    }
    state->last_target_roll = roll_target;
    state->has_last_target_roll = true;

    double raw_pitch_authority =
        fc_axis_available(&state->authority[FLIGHT_CONTROL_AXIS_PITCH], q);
    double raw_roll_authority =
        fc_axis_available(&state->authority[FLIGHT_CONTROL_AXIS_ROLL], q);
    double yaw_authority =
        fc_axis_available(&state->authority[FLIGHT_CONTROL_AXIS_YAW], q);

    double pitch_authority = raw_pitch_authority;
    double roll_authority = raw_roll_authority;
    bool pitch_guard_known=fc_axis_control_authority_known(
        &state->authority[FLIGHT_CONTROL_AXIS_PITCH]);
    bool roll_guard_known=fc_axis_control_authority_known(
        &state->authority[FLIGHT_CONTROL_AXIS_ROLL]);
    double pitch_guard_authority=pitch_guard_known?pitch_authority:0.0;
    double roll_guard_authority=roll_guard_known?roll_authority:0.0;

    double pitch_aero_fraction = fc_clamp(
        fmax(0.0,state->authority[FLIGHT_CONTROL_AXIS_PITCH].aero_per_q)*q/
        raw_pitch_authority,0.0,1.0); /* decision-literal-ok: normalized fraction domain */
    double roll_aero_fraction = fc_clamp(
        fmax(0.0,state->authority[FLIGHT_CONTROL_AXIS_ROLL].aero_per_q)*q/
        raw_roll_authority,0.0,1.0); /* decision-literal-ok: normalized fraction domain */
    double yaw_aero_fraction = fc_clamp(
        fmax(0.0,state->authority[FLIGHT_CONTROL_AXIS_YAW].aero_per_q)*q/
        yaw_authority,0.0,1.0); /* decision-literal-ok: normalized fraction domain */

    double effective_pitch_rate = state->aoa_rate;
    double effective_roll_rate = state->roll_rate;

    /* No hidden trim/integral policy. Disturbance acceleration learned by the
       authority observer is compensated directly in the plant inversion. */
    state->pitch_trim = 0.0;
    state->terminal_pitch_integral = 0.0;
    state->roll_trim = 0.0;
    state->terminal_pitch_authority = pitch_authority;
    state->terminal_roll_authority = roll_authority;
    state->has_terminal_pitch_authority = true;
    state->has_terminal_roll_authority = true;

    double pitch_command = fc_bounded_axis_command(
        pitch_error,effective_pitch_rate,0.0,pitch_authority,
        pitch_guard_known,state->authority[FLIGHT_CONTROL_AXIS_PITCH].drift_accel,
        control_dt);

    double commanded_roll_rate = state->target_roll_rate;
    double relative_rate = effective_roll_rate-commanded_roll_rate;
    double roll_command = fc_bounded_axis_command(
        roll_error,effective_roll_rate,commanded_roll_rate,roll_authority,
        roll_guard_known,state->authority[FLIGHT_CONTROL_AXIS_ROLL].drift_accel,
        control_dt);

    double heading_error = 0.0;
    double yaw_error = -sideslip;
    if (command->heading_control_enabled) {
        heading_error =
            fc_signed_angle(fc_finite(command->target_heading, 0.0)-heading);
        yaw_error = heading_error;
    }
    double yaw_command = fc_bounded_axis_command(
        yaw_error,state->heading_rate,0.0,yaw_authority,true,
        state->authority[FLIGHT_CONTROL_AXIS_YAW].drift_accel,control_dt);

    /* Mission constraint: RCS is not used during atmospheric guidance. */
    double rcs_assist = 0.0;
    state->rcs_transonic_cutoff = true;

    state->last_control[FLIGHT_CONTROL_AXIS_PITCH] =
        fc_clamp(pitch_command,-1.0,1.0);
    state->last_control[FLIGHT_CONTROL_AXIS_ROLL] =
        fc_clamp(roll_command,-1.0,1.0);
    state->last_control[FLIGHT_CONTROL_AXIS_YAW] =
        fc_clamp(yaw_command,-1.0,1.0);

    output->pitch = state->last_control[FLIGHT_CONTROL_AXIS_PITCH];
    output->roll = state->last_control[FLIGHT_CONTROL_AXIS_ROLL];
    output->yaw = state->last_control[FLIGHT_CONTROL_AXIS_YAW];
    output->rcs_assist = rcs_assist;
    output->rcs_requested = false;
    output->valid = true;

    FlightControlDiagnostics *d = &output->diagnostics;
    d->target_pitch_state = target_pitch_state;
    d->measured_pitch_state = aoa;
    d->pitch_error = pitch_error;
    d->effective_pitch_rate = effective_pitch_rate;
    d->body_pitch_rate = body_pitch_rate_valid ? telemetry->body_pitch_rate : 0.0;
    d->body_pitch_rate_available = body_pitch_rate_valid && !body_pitch_rate_stale;
    d->roll_error = roll_error;
    d->effective_roll_rate = effective_roll_rate;
    d->body_roll_rate = body_roll_rate_valid ? telemetry->body_roll_rate : 0.0;
    d->body_roll_rate_available = body_roll_rate_valid && !body_roll_rate_stale;
    d->target_roll_rate = state->target_roll_rate;
    d->commanded_roll_rate = commanded_roll_rate;
    d->relative_roll_rate = relative_rate;
    d->recovery_rate_first = false;
    d->heading_error = heading_error;
    d->sideslip = sideslip;
    d->sideslip_rate = state->sideslip_rate;
    d->body_yaw_rate = body_yaw_rate_valid ? telemetry->body_yaw_rate : 0.0;
    d->body_yaw_rate_available = body_yaw_rate_valid;
    d->flight_path_angle = flight_path_angle;
    d->pitch_trim = 0.0;
    d->steady_pitch_trim = 0.0;
    d->terminal_pitch_integral = 0.0;
    d->roll_trim = 0.0;
    d->pitch_authority = pitch_authority;
    d->pitch_raw_authority = raw_pitch_authority;
    d->pitch_guard_authority = pitch_guard_authority;
    d->pitch_aero_fraction = pitch_aero_fraction;
    d->pitch_fast_command_limit = 1.0; /* decision-literal-ok: normalized actuator command domain */
    d->pitch_hold_seconds = control_dt;
    d->pitch_rate_impulse_budget = pitch_authority*control_dt;
    d->roll_authority = roll_authority;
    d->roll_raw_authority = raw_roll_authority;
    d->roll_guard_authority = roll_guard_authority;
    d->roll_aero_fraction = roll_aero_fraction;
    d->roll_rate_limit = sqrt(2.0*roll_authority*180.0); /* decision-literal-ok: maximum signed bank distance */
    d->roll_command_limit = 1.0; /* decision-literal-ok: normalized actuator command domain */
    d->roll_hold_seconds = control_dt;
    d->roll_rate_impulse_budget = roll_authority*control_dt;
    d->roll_bandwidth_scale = 1.0;
    d->yaw_authority = yaw_authority;
    d->yaw_aero_fraction = yaw_aero_fraction;
    d->beta_yaw_gain = state->beta.yaw_gain;
    d->beta_roll_coupling = state->beta.roll_gain;
    d->beta_confidence = state->beta.confidence;
    d->sample_dt = control_dt;
    d->attitude_error = fmax(fabs(pitch_error),
        fmax(fabs(roll_error),command->heading_control_enabled?
            fabs(heading_error):fabs(sideslip)));
    d->rcs_transonic_cutoff = true;

    state->last_ut = fc_finite(telemetry->ut, state->last_ut + control_dt);
    state->last_pitch = pitch;
    state->last_aoa = aoa;
    state->last_roll = roll;
    state->last_heading = heading;
    state->last_sideslip = sideslip;
    state->has_sample = true;
    state->last_profile = profile;
    state->has_last_profile = true;
    return true;
}
