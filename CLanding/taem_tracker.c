#include "taem_tracker.h"

#include <math.h>
#include <string.h>

#include "shuttlesim/aero.h"
#include "shuttlesim/math3.h"

static const double tracker_pi = 3.14159265358979323846264338327950288;

static double radians(double degrees) { return degrees * tracker_pi / 180.0; }
static double degrees(double radians_value) { return radians_value * 180.0 / tracker_pi; }
static double clamp_value(double x, double lo, double hi) {
    return fmax(lo, fmin(hi, x));
}

static double lateral_response_time(const TerminalModel *m) {
    double roll_wn = m->attitude.roll_wn;
    double roll_zeta = m->attitude.roll_zeta;
    return roll_wn > 0.0 && roll_zeta > 0.0 ?
        clamp_value(4.0 / (roll_wn * roll_zeta), 1.0, 20.0) : 8.0;
}

static double lateral_demand(const TaemGeometryState *g,
        const TaemPathReference *r, double cross_track, double course_error,
        double response) {
    double speed = g->ground_speed_mps;
    double heading_error = radians(course_error);
    return speed * speed * r->curvature_right_per_m -
        2.0 * speed * heading_error / response - cross_track / (response * response);
}

double taem_tracker_lateral_demand(const TerminalModel *m,
        const TaemGeometryState *g, const TaemPathReference *r) {
    if (!m || !g || !r || !isfinite(r->curvature_right_per_m)) return NAN;
    double runway_relative_course = radians(r->course_deg - m->site.runway_heading);
    double tx = cos(runway_relative_course);
    double ty = sin(runway_relative_course);
    double cross_track = (g->runway_along_m - r->runway_along_m) * -ty +
                         (g->runway_cross_m - r->runway_cross_m) * tx;
    double course_error = remainder(g->course_deg - r->course_deg, 360.0);
    return lateral_demand(g, r, cross_track, course_error, lateral_response_time(m));
}

TaemTrackerOutput taem_tracker_update(const TerminalModel *m,
        const TerminalDynamicState *s, const TaemGeometryState *g,
        const TaemPathReference *r, double dt) {
    TaemTrackerOutput out;
    memset(&out, 0, sizeof(out));
    if (!m || !s || !g || !r || !(dt > 0.0) ||
        !(g->ground_speed_mps > 0.0) || !(g->airspeed_mps > 0.0) ||
        !isfinite(r->curvature_right_per_m) ||
        !isfinite(r->flight_path_angle_deg) ||
        !isfinite(r->vertical_curvature_per_m)) return out;

    double runway_relative_course = radians(r->course_deg - m->site.runway_heading);
    double tx = cos(runway_relative_course);
    double ty = sin(runway_relative_course);
    double nx = -ty;
    double ny = tx;
    double dx = g->runway_along_m - r->runway_along_m;
    double dy = g->runway_cross_m - r->runway_cross_m;
    double cross_track = dx * nx + dy * ny;
    double course_error = remainder(g->course_deg - r->course_deg, 360.0);

    double response = lateral_response_time(m);
    double lateral = lateral_demand(g, r, cross_track, course_error, response);

    double altitude = v3_norm(s->position_i_m) - m->world.radius_m;
    double radius = m->world.radius_m + altitude;
    if (!(radius > 0.0)) return out;
    double gravity = m->world.mu_m3_s2 / (radius * radius);
    double gamma = radians(g->flight_path_angle_deg);
    double target_gamma = radians(r->flight_path_angle_deg);
    /* Hold the reference altitude, not only its slope: FPA tracking alone lets
     * altitude error integrate without bound.  Close it over ~15 s of flight,
     * bounded so the correction never dominates the reference slope. */
    if (isfinite(r->altitude_m) && g->ground_speed_mps > 1.0) {
        double altitude_error = r->altitude_m -
            (m->site.altitude + g->altitude_above_runway_m);
        target_gamma += clamp_value(atan(altitude_error /
            (15.0 * g->ground_speed_mps)), -radians(6.0), radians(6.0));
    }
    double pitch_wn = m->attitude.pitch_wn;
    double pitch_zeta = m->attitude.pitch_zeta;
    double pitch_response = pitch_wn > 0.0 && pitch_zeta > 0.0 ?
        clamp_value(4.0 / (pitch_wn * pitch_zeta), 1.0, 20.0) : 8.0;
    double desired_gamma_rate = g->ground_speed_mps * r->vertical_curvature_per_m +
        (target_gamma - gamma) / pitch_response;
    desired_gamma_rate = clamp_value(desired_gamma_rate,
        -radians(8.0), radians(8.0));
    double airspeed = g->airspeed_mps;
    double vertical_required = (gravity - airspeed * airspeed / radius) * cos(gamma) +
        airspeed * desired_gamma_rate;
    vertical_required = fmax(0.0, vertical_required);

    double aoa_max = fmin(m->vehicle.maximum_angle_of_attack,
                          m->aero.alpha_deg[m->aero.alpha_count - 1]);
    AeroForces flow = aero_compute(&m->world, &m->aero,
        s->position_i_m, s->velocity_i_mps, s->ut_s, s->mass_kg, 0.0, 0.0);
    if (flow.mach < 1.0 &&
        isfinite(m->vehicle.terminal_maximum_lift_angle_of_attack) &&
        m->vehicle.terminal_maximum_lift_angle_of_attack > 0.0)
        aoa_max = fmin(aoa_max,
            m->vehicle.terminal_maximum_lift_angle_of_attack);
    double bank_max = fmin(m->vehicle.maximum_bank_angle, 80.0);
    double g_limit = fmax(0.0, m->vehicle.maximum_g_load) * gravity;
    double best_error = INFINITY;
    double chosen_aoa = 0.0, chosen_bank = 0.0, chosen_vertical = 0.0;
    double available_lateral = 0.0;
    bool chosen_lateral_feasible = false;
    double best_glide_ratio = -INFINITY;
    double glide_aoa = 0.0, glide_bank = 0.0, glide_vertical = 0.0;
    bool have_glide_candidate = false;
    double refinement_low = 0.0, refinement_high = 0.0;
    for (int pass = 0; pass < 2; ++pass) {
      if (pass == 1) {
          refinement_low = fmax(0.0, chosen_aoa - 0.5);
          refinement_high = fmin(aoa_max, chosen_aoa + 0.5);
      }
      double first_aoa = pass == 0 ? 0.0 : refinement_low;
      double last_aoa = pass == 0 ? aoa_max : refinement_high;
      double step_aoa = pass == 0 ? 0.5 : 0.1;
      for (double aoa_deg = first_aoa;
           aoa_deg <= last_aoa + 1e-9; aoa_deg += step_aoa) {
        AeroForces forces = aero_compute(&m->world, &m->aero,
            s->position_i_m, s->velocity_i_mps, s->ut_s, s->mass_kg,
            radians(aoa_deg), 0.0);
        if (!isfinite(forces.lift_n)) continue;
        double lift = fmin(fabs(forces.lift_n) / s->mass_kg, g_limit);
        if (!(lift > 1e-8)) continue;

        double lateral_capacity = lift * sin(radians(bank_max));
        available_lateral = fmax(available_lateral, lateral_capacity);
        bool lateral_feasible = fabs(lateral) <= lateral_capacity + 1e-9;
        double bank;
        if (lateral_feasible) {
            bank = degrees(asin(clamp_value(lateral / lift, -1.0, 1.0)));
        } else {
            bank = lateral >= 0.0 ? bank_max : -bank_max;
        }
        double delivered_lateral = lift * sin(radians(bank));
        /* The gamma-rate equation requires lift normal to the flight path.
         * cos(gamma) belongs on the gravity/curvature term above, not here:
         * another projection would over-command lift in a steep descent. */
        double delivered_vertical = lift * cos(radians(bank));
        double vertical_error = fabs(delivered_vertical - vertical_required);
        double lateral_error = fabs(delivered_lateral - lateral);
        double score = vertical_error + 0.35 * lateral_error;

        if (pass == 0 && lateral_feasible && isfinite(forces.drag_n) &&
            forces.drag_n > 1e-6 && forces.lift_n > 0.0) {
            double glide_ratio = forces.lift_n / forces.drag_n;
            if (glide_ratio > best_glide_ratio) {
                best_glide_ratio = glide_ratio;
                glide_aoa = aoa_deg;
                glide_bank = bank;
                glide_vertical = delivered_vertical;
                have_glide_candidate = true;
            }
        }

        /* A turn that cannot be produced is never preferable to one that can.
         * The old weighted score could select a low-AoA solution with a smaller
         * vertical error even when it under-delivered the required lateral
         * acceleration, then falsely report that the route exceeded authority. */
        bool choose = false;
        if (lateral_feasible && !chosen_lateral_feasible)
            choose = true;
        else if (lateral_feasible == chosen_lateral_feasible && score < best_error)
            choose = true;
        if (choose) {
            best_error = score;
            chosen_aoa = aoa_deg;
            chosen_bank = bank;
            chosen_vertical = delivered_vertical;
            chosen_lateral_feasible = lateral_feasible;
        }
      }
    }
    if (!isfinite(best_error)) return out;

    /* An unpowered vehicle must not spend the rest of its kinetic energy trying
     * to hold a vertical path that the current dynamic pressure cannot support.
     * If even the best vertical-tracking solution under-delivers required lift,
     * fly the best-L/D incidence that can still generate the required lateral
     * acceleration.  The resulting steeper descent builds density and preserves
     * speed; altitude/FPA/energy closure remain checked by native replay. */
    double vertical_tolerance = fmax(0.25, 0.1 * fmax(1.0, vertical_required));
    if (chosen_lateral_feasible && have_glide_candidate &&
        chosen_vertical < vertical_required - vertical_tolerance) {
        chosen_aoa = glide_aoa;
        chosen_bank = glide_bank;
        chosen_vertical = glide_vertical;
    }

    out.valid = true;
    out.control = (TerminalControl){
        .angle_of_attack_rad = radians(chosen_aoa),
        .bank_rad = radians(chosen_bank),
        .dt_s = dt
    };
    out.cross_track_error_m = cross_track;
    out.course_error_deg = course_error;
    out.required_lateral_accel_mps2 = lateral;
    out.available_lateral_accel_mps2 = available_lateral;
    out.required_vertical_lift_mps2 = vertical_required;
    out.delivered_vertical_lift_mps2 = chosen_vertical;
    out.response_time_s = fmax(response, pitch_response);
    out.lateral_authority_ok = fabs(lateral) <= available_lateral + 1e-6;
    out.vertical_authority_ok = fabs(chosen_vertical - vertical_required) <=
        fmax(0.25, 0.1 * fmax(1.0, vertical_required));
    return out;
}
