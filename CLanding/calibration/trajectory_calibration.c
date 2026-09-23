#include "landing.h"
#include "calibration_observation.h"

#include <float.h>
#include <math.h>
#include <string.h>


static double linear_blend(double current, double sample, double blend) {
    if (!isfinite(sample)) return current;
    if (!isfinite(current)) return sample;
    return current + clampd(blend, 0.0, 1.0) * (sample - current);
}

static double positive_log_blend(double current, double sample, double blend) {
    if (!(current > 0.0) || !(sample > 0.0) ||
        !isfinite(current) || !isfinite(sample))
        return current;
    blend = clampd(blend, 0.0, 1.0);
    return exp(log(current) + blend * (log(sample) - log(current)));
}

void trajectory_calibrator_init(InFlightTrajectoryCalibrator *c) {
    memset(c, 0, sizeof(*c));
    trajectory_init(&c->forecast);
    c->model.density_scale = 1.0;
    c->model.drag_scale = 1.0;
    c->model.lift_scale = 1.0;
    c->model.bank_effectiveness = 1.0;
    c->model.speed_of_sound_scale = 1.0;
    c->model.speed_of_sound = 340.0;
    c->model.confidence = 0.0;
}

void trajectory_calibrator_clear(InFlightTrajectoryCalibrator *c) {
    trajectory_clear(&c->forecast);
    trajectory_calibrator_init(c);
}

void trajectory_calibrator_set_forecast(
        InFlightTrajectoryCalibrator *c, const Trajectory *forecast) {
    trajectory_copy(&c->forecast, forecast);
}

static bool forecast_at(
        const Trajectory *forecast, double ut, TrajectoryPoint *out) {
    if (forecast->count < 2 ||
        ut < forecast->points[0].ut ||
        ut > forecast->points[forecast->count - 1].ut)
        return false;

    size_t lo = 0;
    size_t hi = forecast->count - 1;
    while (hi - lo > 1) {
        size_t mid = lo + (hi - lo) / 2;
        if (forecast->points[mid].ut >= ut)
            hi = mid;
        else
            lo = mid;
    }

    if (ut <= forecast->points[0].ut) {
        *out = forecast->points[0];
        return true;
    }
    if (ut >= forecast->points[forecast->count - 1].ut) {
        *out = forecast->points[forecast->count - 1];
        return true;
    }

    TrajectoryPoint a = forecast->points[lo];
    TrajectoryPoint b = forecast->points[hi];
    double denominator = b.ut - a.ut;
    if (!(denominator > DBL_MIN)) return false;

    double fraction = clampd((ut - a.ut) / denominator, 0.0, 1.0);
    *out = a;
    out->ut = ut;
    out->latitude = a.latitude + (b.latitude - a.latitude) * fraction;
    out->longitude = norm_signed_deg(
        a.longitude + norm_signed_deg(b.longitude - a.longitude) * fraction);
    out->altitude = a.altitude + (b.altitude - a.altitude) * fraction;
    out->speed = a.speed + (b.speed - a.speed) * fraction;
    return true;
}

TrajectoryCalibrationModel trajectory_calibrator_update(
        InFlightTrajectoryCalibrator *c,
        const Telemetry *t,
        const VehicleState *state,
        const PlanetModel *planet,
        const LandingSite *site,
        const AerodynamicEnvelope *env,
        const VehicleProfile *vehicle,
        const GuidanceCommand *command,
        const CalibrationSettings *settings,
        bool allow_adaptation,
        double dt) {
    (void)command;

    if (c->has_previous && state->ut < c->previous_ut) {
        /*
         * A quickload invalidates differentiated state and the abandoned
         * branch's receding-horizon forecast. Learned environment/response
         * observations remain physical evidence.
         */
        c->has_previous = false;
        trajectory_clear(&c->forecast);
        trajectory_init(&c->forecast);
    }

    bool learning_enabled =
        settings->enable_trajectory_calibration &&
        allow_adaptation &&
        t->physics_sample_valid;

    CalibrationObservationEnvelope observation =
        calibration_observation_envelope(t, vehicle, settings, dt);

    /*
     * Density and sound speed are direct environmental observations. They do
     * not require a quasi-steady vehicle or an aerodynamic force sample.
     * KSP supplies these values deterministically, so use the measurement
     * directly instead of a hand-tuned learning-rate filter.
     */
    double base_density =
        planet_atmospheric_density(planet, t->mean_altitude);
    if (learning_enabled &&
        t->mean_altitude >= 0.0 &&
        t->mean_altitude < planet->atmosphere_depth) {
        if (t->atmospheric_density > 0.0 &&
            isfinite(t->atmospheric_density) &&
            base_density > DBL_MIN) {
            double density_scale = t->atmospheric_density / base_density;
            if (isfinite(density_scale) && density_scale > 0.0)
                c->model.density_scale = density_scale;
        }

        double observed_sound = 0.0;
        if (t->speed_of_sound > 0.0 && isfinite(t->speed_of_sound)) {
            observed_sound = t->speed_of_sound;
        } else if (t->mach > DBL_MIN && t->true_air_speed > 0.0) {
            observed_sound = t->true_air_speed / t->mach;
        }

        if (observed_sound > 0.0 && isfinite(observed_sound)) {
            c->model.speed_of_sound = observed_sound;
            double profile_sound =
                planet_atmospheric_speed_of_sound(planet, t->mean_altitude);
            if (profile_sound > DBL_MIN) {
                double scale = observed_sound / profile_sound;
                if (isfinite(scale) && scale > 0.0)
                    c->model.speed_of_sound_scale = scale;
            }
        }
    }

    /*
     * Certified aerodynamic coefficients remain prior-flight products. Current
     * flight identifies environment, forecast residuals and bank response.
     */
    c->model.drag_scale = 1.0;
    c->model.lift_scale = 1.0;

    if (learning_enabled && observation.trajectory_usable) {
        c->model.accepted_samples++;
        c->model.confidence =
            1.0 - (1.0 - c->model.confidence) *
                exp(-observation.information_increment);

        TrajectoryPoint forecast_point;
        if (forecast_at(&c->forecast, t->ut, &forecast_point)) {
            GeoPoint predicted = {
                forecast_point.latitude,
                forecast_point.longitude,
                forecast_point.altitude
            };
            GeoPoint target = {
                site->latitude,
                site->longitude,
                site->altitude
            };

            double predicted_range =
                great_circle_distance(predicted, target, planet->radius);
            double altitude_error =
                t->mean_altitude - forecast_point.altitude;
            double speed_error =
                t->true_air_speed - forecast_point.speed;
            double range_error =
                t->range_to_site - predicted_range;

            c->model.altitude_residual =
                linear_blend(c->model.altitude_residual,
                    altitude_error, observation.information_blend);
            c->model.speed_residual =
                linear_blend(c->model.speed_residual,
                    speed_error, observation.information_blend);
            c->model.range_residual =
                linear_blend(c->model.range_residual,
                    range_error, observation.information_blend);
        }

        /*
         * Bank effectiveness is directly observable from
         *   course_rate = (lift/mass) * sin(bank) / V_horizontal.
         * sin^2(bank) is the excitation/observability weight: it tends
         * continuously to zero at wings-level, eliminating fixed bank/error
         * gates while naturally rejecting an unobservable division.
         */
        double horizontal_speed = t->horizontal_speed;
        double lift_accel =
            t->mass > 0.0 ? fmax(0.0, t->lift_force) / t->mass : 0.0;
        double bank = t->roll * DEG2RAD;
        double lateral_fraction = sin(bank);
        double expected_course_rate = 0.0;
        if (horizontal_speed > DBL_MIN && lift_accel > 0.0) {
            expected_course_rate =
                lift_accel * lateral_fraction / horizontal_speed * RAD2DEG;
        }

        double measured_course_rate =
            t->has_course_rate ? t->course_rate : t->heading_rate;
        if (fabs(expected_course_rate) > DBL_MIN &&
            isfinite(measured_course_rate)) {
            double sample =
                measured_course_rate / expected_course_rate;
            if (sample > 0.0 && isfinite(sample)) {
                double excitation =
                    lateral_fraction * lateral_fraction;
                double bank_blend =
                    -expm1(-observation.information_increment * excitation);
                c->model.bank_effectiveness =
                    positive_log_blend(
                        c->model.bank_effectiveness, sample, bank_blend);
            }
        }
    }

    c->previous_ut = state->ut;
    c->has_previous = true;

    if (!(c->model.density_scale > 0.0) ||
        !isfinite(c->model.density_scale))
        c->model.density_scale = 1.0;
    if (!(c->model.speed_of_sound > 0.0) ||
        !isfinite(c->model.speed_of_sound))
        c->model.speed_of_sound = 340.0;
    if (!(c->model.speed_of_sound_scale > 0.0) ||
        !isfinite(c->model.speed_of_sound_scale))
        c->model.speed_of_sound_scale = 1.0;
    if (!(c->model.bank_effectiveness > 0.0) ||
        !isfinite(c->model.bank_effectiveness))
        c->model.bank_effectiveness = 1.0;
    c->model.confidence = clampd(c->model.confidence, 0.0, 1.0);

    /*
     * Keep env referenced so compiler/test builds fail if the API stops
     * carrying the certified prior needed for future residual-model work.
     */
    (void)env;
    return c->model;
}
