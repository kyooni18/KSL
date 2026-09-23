#include "calibration_observation.h"

#include <float.h>
#include <math.h>
#include <strings.h>

static double minimum_valid_normalized_margin(
        const DecisionMargin *margins, size_t count) {
    double worst = INFINITY;
    bool any = false;
    for (size_t i = 0; i < count; ++i) {
        if (!margins[i].valid) continue;
        worst = fmin(worst, margins[i].normalized_margin);
        any = true;
    }
    return any ? worst : -INFINITY;
}

static bool margin_nonnegative(DecisionMargin margin) {
    return margin.valid && margin.margin >= 0.0;
}

static bool telemetry_airborne(const Telemetry *t) {
    if (!t) return false;
    return strcasecmp(t->vessel_situation, "landed") &&
           strcasecmp(t->vessel_situation, "splashed") &&
           strcasecmp(t->vessel_situation, "pre-launch");
}

CalibrationObservationEnvelope calibration_observation_envelope(
        const Telemetry *t,
        const VehicleProfile *v,
        const CalibrationSettings *s,
        double dt) {
    CalibrationObservationEnvelope out = {0};
    if (!t || !v || !s || !isfinite(dt) || dt < 0.0) return out;

    out.airborne = telemetry_airborne(t);
    out.clean_configuration =
        t->has_airbrakes && !t->airbrakes && !t->gear && !t->brakes;

    /*
     * Normalized actuator commands have the protocol domain [-1, 1].
     * Saturation is a domain boundary, not a tuned 0.9-style quality gate.
     */
    double maximum_control = 0.0;
    if (t->has_controls) {
        maximum_control = fmax(fabs(t->control_pitch),
            fmax(fabs(t->control_roll), fabs(t->control_yaw)));
    }
    out.actuator_domain_valid =
        !t->has_controls || (isfinite(maximum_control) && maximum_control < 1.0);

    /*
     * Atmospheric landing is explicitly unpowered. A thrust-contaminated force
     * sample belongs to a different dynamics model rather than this aero model.
     */
    double thrust =
        isfinite(t->current_thrust) ? fmax(0.0, t->current_thrust) : INFINITY;
    out.thrust = decision_margin(0.0, thrust);
    out.unpowered = margin_nonnegative(out.thrust);

    double q_limit =
        fmin(s->maximum_sample_dynamic_pressure, v->maximum_dynamic_pressure);
    out.dynamic_pressure_minimum =
        decision_margin(t->dynamic_pressure, s->minimum_dynamic_pressure);
    out.dynamic_pressure_maximum =
        decision_margin(q_limit, t->dynamic_pressure);

    out.incidence_deg = hypot(t->angle_of_attack, t->sideslip);
    out.incidence =
        decision_margin(v->maximum_angle_of_attack, out.incidence_deg);
    out.sideslip =
        decision_margin(s->maximum_sideslip, fabs(t->sideslip));
    out.load = decision_margin(v->maximum_g_load, t->g_force);
    out.stall =
        decision_margin(s->abort_stall_fraction, t->stall_fraction);

    double aerodynamic_force = hypot(t->lift_force, t->drag_force);
    out.force_observable =
        t->has_force_vectors &&
        t->mass > 0.0 && isfinite(t->mass) &&
        t->true_air_speed > 0.0 && isfinite(t->true_air_speed) &&
        t->drag_force > 0.0 && isfinite(t->drag_force) &&
        isfinite(t->lift_force) && isfinite(aerodynamic_force) &&
        aerodynamic_force > 0.0;

    if (out.force_observable) {
        out.aerodynamic_accel_mps2 = aerodynamic_force / t->mass;
        if (out.aerodynamic_accel_mps2 > DBL_MIN) {
            /*
             * tau = V/|a_aero| is the local aerodynamic response time.
             * dt/tau is therefore dimensionless independent dynamical
             * information. 1-exp(-dt/tau) is its exact exponential blend.
             */
            out.aerodynamic_response_time_s =
                t->true_air_speed / out.aerodynamic_accel_mps2;
            if (out.aerodynamic_response_time_s > DBL_MIN &&
                isfinite(out.aerodynamic_response_time_s)) {
                out.information_increment =
                    dt / out.aerodynamic_response_time_s;
                out.information_blend =
                    -expm1(-out.information_increment);
            }
        }
    }

    DecisionMargin margins[] = {
        out.dynamic_pressure_minimum,
        out.dynamic_pressure_maximum,
        out.incidence,
        out.sideslip,
        out.load,
        out.stall,
        out.thrust,
    };
    out.worst_normalized_margin =
        minimum_valid_normalized_margin(
            margins, sizeof(margins) / sizeof(margins[0]));

    bool inside_model_domain =
        margin_nonnegative(out.dynamic_pressure_minimum) &&
        margin_nonnegative(out.dynamic_pressure_maximum) &&
        margin_nonnegative(out.incidence) &&
        margin_nonnegative(out.sideslip) &&
        margin_nonnegative(out.load) &&
        margin_nonnegative(out.stall);

    out.valid =
        t->physics_sample_valid && out.airborne &&
        out.actuator_domain_valid && out.unpowered &&
        inside_model_domain &&
        isfinite(out.information_increment) &&
        isfinite(out.information_blend);

    out.passive_aero_usable =
        out.valid && out.clean_configuration && out.force_observable;
    out.trajectory_usable =
        out.valid && out.clean_configuration && out.force_observable;
    return out;
}

CalibrationSafetyEnvelope calibration_safety_envelope(
        const Telemetry *t,
        const VehicleProfile *v,
        const CalibrationSettings *s) {
    CalibrationSafetyEnvelope out = {0};
    if (!t || !v || !s) return out;

    double q_limit =
        fmin(s->maximum_calibration_dynamic_pressure,
             v->maximum_dynamic_pressure);
    double load_limit =
        fmin(s->maximum_calibration_g_load, v->maximum_g_load);
    double downward_speed = fmax(0.0, -t->vertical_speed);

    out.altitude = decision_margin(
        t->radar_altitude, s->minimum_calibration_radar_altitude);
    out.dynamic_pressure =
        decision_margin(q_limit, t->dynamic_pressure);
    out.load = decision_margin(load_limit, t->g_force);
    out.sink_rate =
        decision_margin(s->maximum_calibration_sink_rate, downward_speed);
    out.stall =
        decision_margin(s->abort_stall_fraction, t->stall_fraction);
    out.speed =
        decision_margin(t->true_air_speed, v->minimum_safe_speed);

    DecisionMargin margins[] = {
        out.altitude,
        out.dynamic_pressure,
        out.load,
        out.sink_rate,
        out.stall,
        out.speed,
    };
    out.worst_normalized_margin =
        minimum_valid_normalized_margin(
            margins, sizeof(margins) / sizeof(margins[0]));

    out.valid = telemetry_airborne(t);
    out.safe =
        out.valid &&
        margin_nonnegative(out.altitude) &&
        margin_nonnegative(out.dynamic_pressure) &&
        margin_nonnegative(out.load) &&
        margin_nonnegative(out.sink_rate) &&
        margin_nonnegative(out.stall) &&
        margin_nonnegative(out.speed);
    return out;
}
