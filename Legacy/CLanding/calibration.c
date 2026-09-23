#include "landing.h"
#include "calibration_observation.h"

#include <float.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

static int regime_for(double mach) {
    /* Predictor interpolation anchors; midpoint partition is a discretization. */
    static const double anchors[] = {0.35, 1.05, 2.6, 6.0};
    double x = fmax(0.0, mach);
    for (int i = 0; i < 3; ++i) {
        if (x < 0.5 * (anchors[i] + anchors[i + 1])) return i;
    }
    return 3;
}

static double positive_log_blend(double current, double sample, double blend) {
    if (!(current > 0.0) || !(sample > 0.0) ||
        !isfinite(current) || !isfinite(sample) || !isfinite(blend))
        return current;
    blend = clampd(blend, 0.0, 1.0);
    return exp(log(current) + blend * (log(sample) - log(current)));
}

void adaptive_calibrator_init(
        AdaptiveFlightCalibrator *c, const VehicleProfile *profile) {
    memset(c, 0, sizeof(*c));
    for (int i = 0; i < 4; ++i) {
        c->ld[i] = profile->estimated_lift_to_drag;
        c->beta[i] = profile->estimated_ballistic_coefficient;
        c->confidence[i] = 0.0;
    }
    c->recommended = *profile;
}

void adaptive_calibrator_reset(
        AdaptiveFlightCalibrator *c, const VehicleProfile *profile) {
    adaptive_calibrator_init(c, profile);
}

void adaptive_calibrator_update(
        AdaptiveFlightCalibrator *c,
        const Telemetry *t,
        const CalibrationSettings *s,
        const VehicleProfile *baseline,
        const GuidanceCommand *command,
        double dt,
        bool allow_adaptation,
        bool active,
        bool sampling,
        CalibrationRunState run_state,
        double progress,
        double target_aoa,
        const char *run_status,
        const char *warning,
        AerodynamicModel *current,
        AerodynamicModel *planning,
        AerodynamicEnvelope *envelope,
        VehicleProfile *recommended,
        CalibrationSnapshot *snap) {
    (void)command;

    int regime = regime_for(t->mach);
    bool enabled = s->enable_passive_calibration || active;
    CalibrationObservationEnvelope observation =
        calibration_observation_envelope(t, baseline, s, dt);

    if (allow_adaptation && enabled && observation.passive_aero_usable) {
        double lift_factor = 0.0;
        double drag_factor = 1.0;
        aerodynamic_force_factors_mach(
            t->mach, observation.incidence_deg, baseline,
            &lift_factor, &drag_factor);

        bool updated = false;
        if (drag_factor > DBL_MIN && t->drag_force > DBL_MIN) {
            double beta_sample =
                t->dynamic_pressure * t->mass / t->drag_force * drag_factor;
            if (isfinite(beta_sample) && beta_sample > 0.0) {
                c->beta[regime] = positive_log_blend(
                    c->beta[regime], beta_sample,
                    observation.information_blend);
                updated = true;
            }

            if (fabs(lift_factor) > DBL_MIN && t->lift_force >= 0.0) {
                double observed_ld = t->lift_force / t->drag_force;
                double ld_sample =
                    observed_ld * drag_factor / fabs(lift_factor);
                if (isfinite(ld_sample) && ld_sample > 0.0) {
                    c->ld[regime] = positive_log_blend(
                        c->ld[regime], ld_sample,
                        observation.information_blend);
                    updated = true;
                }
            }
        }

        if (updated) {
            c->accepted++;
            /*
             * Information confidence is the complement of unexplained
             * aerodynamic response exposure. It is cadence-independent because
             * information_increment = dt / (V / |a_aero|).
             */
            c->confidence[regime] =
                1.0 - (1.0 - c->confidence[regime]) *
                    exp(-observation.information_increment);
        } else {
            c->rejected++;
        }
    } else if (allow_adaptation && enabled && observation.airborne) {
        c->rejected++;
    }

    double current_conf = clampd(c->confidence[regime], 0.0, 1.0);
    double planning_ld = baseline->estimated_lift_to_drag;
    double planning_beta = baseline->estimated_ballistic_coefficient;
    double high_weight = c->confidence[2] + c->confidence[3];
    if (high_weight > DBL_MIN) {
        planning_ld =
            (c->ld[2] * c->confidence[2] +
             c->ld[3] * c->confidence[3]) / high_weight;
        planning_beta =
            (c->beta[2] * c->confidence[2] +
             c->beta[3] * c->confidence[3]) / high_weight;
    }
    double planning_conf =
        0.5 * (c->confidence[2] + c->confidence[3]);

    /*
     * Same-flight reconstruction remains advisory. In particular, do not infer
     * a stall speed from the conservative stall proxy and feed heuristic speed
     * multipliers back into the vehicle profile. Certification happens through
     * the prior-flight vessel-physics data book.
     */
    c->recommended = *baseline;

    current->lift_to_drag = c->ld[regime];
    current->ballistic_coefficient = c->beta[regime];
    current->confidence = current_conf;
    planning->lift_to_drag = planning_ld;
    planning->ballistic_coefficient = planning_beta;
    planning->confidence = planning_conf;
    for (int i = 0; i < 4; ++i) {
        envelope->regimes[i].lift_to_drag = c->ld[i];
        envelope->regimes[i].ballistic_coefficient = c->beta[i];
        envelope->regimes[i].confidence =
            clampd(c->confidence[i], 0.0, 1.0);
    }
    *recommended = c->recommended;

    calibration_snapshot_init(snap);
    snap->state = (active || run_state != CAL_IDLE) ? run_state : CAL_IDLE;
    snap->active = active;
    snap->progress = clampd(progress, 0.0, 1.0);
    snap->target_angle_of_attack = target_aoa;
    snap->accepted_samples = c->accepted;
    snap->rejected_samples = c->rejected;
    snap->confidence = current_conf;
    snap->planning_confidence = planning_conf;
    snap->best_glide_angle_of_attack = c->best_glide_aoa;
    snap->estimated_stall_speed = c->stall_speed;

    if (run_status && *run_status) {
        snprintf(snap->status, sizeof(snap->status), "%s%s",
            run_status,
            (active && !sampling && run_state == CAL_SETTLING)
                ? " Waiting for the configured settling interval."
                : "");
    } else if (c->accepted == 0) {
        snprintf(snap->status, sizeof(snap->status),
            "Waiting for an in-domain clean unpowered aerodynamic observation.");
    } else {
        snprintf(snap->status, sizeof(snap->status),
            "Shadow aerodynamic reconstruction: %d usable observations; "
            "current-regime information confidence %.3f.",
            c->accepted, current_conf);
    }

    if (warning && *warning) {
        snap->has_warning = true;
        snprintf(snap->warning, sizeof(snap->warning), "%s", warning);
    }
}

void glide_calibration_init(GlideCalibrationMachine *g) {
    memset(g, 0, sizeof(*g));
    g->state = CAL_IDLE;
    snprintf(g->status, sizeof(g->status), "Calibration is idle.");
}

static void make_angles(
        GlideCalibrationMachine *g, const CalibrationSettings *s) {
    g->angle_count = 0;
    size_t capacity = sizeof(g->angles) / sizeof(g->angles[0]);
    double angle = s->minimum_angle_of_attack;

    while ((size_t)g->angle_count + 1 < capacity &&
           angle < s->maximum_angle_of_attack) {
        g->angles[g->angle_count++] = angle;
        angle += s->angle_of_attack_step;
    }

    if ((size_t)g->angle_count < capacity) {
        double scale = fmax(1.0, fabs(s->maximum_angle_of_attack));
        bool already_at_end =
            g->angle_count > 0 &&
            fabs(g->angles[g->angle_count - 1] -
                 s->maximum_angle_of_attack) <= DBL_EPSILON * scale;
        if (!already_at_end)
            g->angles[g->angle_count++] = s->maximum_angle_of_attack;
    }
}

void glide_calibration_start(
        GlideCalibrationMachine *g,
        const Telemetry *t,
        const CalibrationSettings *s) {
    glide_calibration_init(g);
    g->active = true;
    g->state = CAL_SETTLING;
    g->start_ut = g->state_start_ut = t->ut;
    g->initial_heading = t->heading;
    make_angles(g, s);
    snprintf(g->status, sizeof(g->status),
        "Starting a wings-level glide calibration.");
}

void glide_calibration_stop(
        GlideCalibrationMachine *g, const char *status) {
    g->active = false;
    g->state = CAL_STOPPED;
    snprintf(g->status, sizeof(g->status), "%s",
        status ? status : "Calibration stopped. Manual control restored.");
}

static const char *safety_failure(
        const Telemetry *t,
        const VehicleProfile *v,
        const CalibrationSettings *s) {
    CalibrationSafetyEnvelope safety =
        calibration_safety_envelope(t, v, s);

    if (!safety.valid)
        return "Calibration requires an airborne craft.";
    if (safety.altitude.margin < 0.0)
        return "Calibration altitude floor reached.";
    if (safety.dynamic_pressure.margin < 0.0)
        return "Calibration stopped because dynamic pressure exceeded its safety limit.";
    if (safety.load.margin < 0.0)
        return "Calibration stopped because G-load exceeded its safety limit.";
    if (safety.sink_rate.margin < 0.0)
        return "Calibration stopped because sink rate exceeded its safety limit.";
    if (safety.stall.margin < 0.0)
        return "Calibration stopped at the configured stall boundary.";
    if (safety.speed.margin < 0.0)
        return "Calibration stopped because airspeed fell below the vehicle minimum safe speed.";
    return NULL;
}

GuidanceResult glide_calibration_update(
        GlideCalibrationMachine *g,
        const Telemetry *t,
        const VehicleProfile *v,
        const CalibrationSettings *s,
        bool *sampling,
        bool *finished,
        double *progress,
        double *target_aoa) {
    *sampling = false;
    *finished = false;
    *progress = 0.0;
    *target_aoa = 0.0;

    GuidanceCommand safe;
    guidance_command_init(&safe);

    if (!g->active) {
        *finished =
            g->state == CAL_COMPLETE ||
            g->state == CAL_STOPPED ||
            g->state == CAL_ABORTED;
        return (GuidanceResult){
            .phase = PHASE_CALIBRATION,
            .command = safe,
            .status = "Calibration is idle."
        };
    }

    const char *failure = safety_failure(t, v, s);
    if (failure) {
        g->active = false;
        g->state = CAL_ABORTED;
        snprintf(g->status, sizeof(g->status), "%s", failure);

        GuidanceResult result;
        memset(&result, 0, sizeof(result));
        result.phase = PHASE_CALIBRATION;
        result.command = safe;
        result.has_warning = true;
        snprintf(result.status, sizeof(result.status), "%s", failure);
        snprintf(result.warning, sizeof(result.warning), "%s", failure);
        trajectory_init(&result.reference);
        *finished = true;
        return result;
    }

    double total = s->settling_duration + s->sampling_duration;
    if (total <= 0.0) {
        g->angle_index = g->angle_count;
    } else {
        while (g->angle_index < g->angle_count &&
               t->ut - g->state_start_ut >= total) {
            g->angle_index++;
            g->state_start_ut += total;
        }
    }

    if (g->angle_index >= g->angle_count) {
        g->active = false;
        g->state = CAL_COMPLETE;
        snprintf(g->status, sizeof(g->status),
            "Calibration sweep completed. Review the shadow profile; "
            "recorded force data becomes certified on a later session.");

        GuidanceResult result;
        memset(&result, 0, sizeof(result));
        result.phase = PHASE_CALIBRATION;
        result.command = safe;
        snprintf(result.status, sizeof(result.status), "%s", g->status);
        trajectory_init(&result.reference);
        *finished = true;
        *progress = 1.0;
        return result;
    }

    double elapsed = fmax(0.0, t->ut - g->state_start_ut);
    *sampling = elapsed >= s->settling_duration;
    g->state = *sampling ? CAL_SAMPLING : CAL_SETTLING;

    double angle = g->angles[g->angle_index];
    *target_aoa = angle;
    double interval_progress =
        total > 0.0 ? clampd(elapsed / total, 0.0, 1.0) : 1.0;
    *progress =
        ((double)g->angle_index + interval_progress) /
        (double)g->angle_count;

    GuidanceCommand command_out;
    guidance_command_init(&command_out);
    command_out.autopilot_engaged = true;
    command_out.target_heading = g->initial_heading;
    command_out.target_roll = 0.0;
    command_out.has_target_aoa = true;
    command_out.target_aoa = angle;
    command_out.target_pitch = t->flight_path_angle + angle;
    command_out.gear = t->gear;
    command_out.airbrakes = false;
    command_out.navball_speed_mode = SPEED_SURFACE;
    command_out.control_profile = PROFILE_APPROACH;

    GuidanceResult result;
    memset(&result, 0, sizeof(result));
    result.phase = PHASE_CALIBRATION;
    result.command = command_out;
    trajectory_init(&result.reference);
    snprintf(result.status, sizeof(result.status),
        *sampling
            ? "Sampling the clean glide at %.1f deg AoA."
            : "Settling at %.1f deg AoA before sampling.",
        angle);
    snprintf(g->status, sizeof(g->status), "%s", result.status);
    return result;
}
