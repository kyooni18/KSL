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

TaemTrackerOutput taem_tracker_update(const TerminalModel *m,
        const TerminalDynamicState *s, const TaemGeometryState *g,
        const TaemPathReference *r, double dt) {
    TaemTrackerOutput out;
    memset(&out, 0, sizeof(out));
    if (!m || !s || !g || !r || !(dt > 0.0) ||
        !(g->ground_speed_mps > 0.0) || !isfinite(r->curvature_right_per_m) ||
        !isfinite(r->flight_path_angle_deg)) return out;

    double runway_relative_course = radians(r->course_deg - m->site.runway_heading);
    double tx = cos(runway_relative_course);
    double ty = sin(runway_relative_course);
    double nx = -ty;
    double ny = tx;
    double dx = g->runway_along_m - r->runway_along_m;
    double dy = g->runway_cross_m - r->runway_cross_m;
    double cross_track = dx * nx + dy * ny;
    double course_error = remainder(g->course_deg - r->course_deg, 360.0);

    double roll_wn = m->attitude.roll_wn;
    double roll_zeta = m->attitude.roll_zeta;
    double response = roll_wn > 0.0 && roll_zeta > 0.0 ?
        clamp_value(4.0 / (roll_wn * roll_zeta), 1.0, 20.0) : 8.0;
    double speed = g->ground_speed_mps;
    double heading_error = radians(course_error);
    double lateral = speed * speed * r->curvature_right_per_m -
        2.0 * speed * heading_error / response - cross_track / (response * response);

    double altitude = v3_norm(s->position_i_m) - m->world.radius_m;
    double radius = m->world.radius_m + altitude;
    if (!(radius > 0.0)) return out;
    double gravity = m->world.mu_m3_s2 / (radius * radius);
    double gamma = radians(g->flight_path_angle_deg);
    double target_gamma = radians(r->flight_path_angle_deg);
    double pitch_wn = m->attitude.pitch_wn;
    double pitch_zeta = m->attitude.pitch_zeta;
    double pitch_response = pitch_wn > 0.0 && pitch_zeta > 0.0 ?
        clamp_value(4.0 / (pitch_wn * pitch_zeta), 1.0, 20.0) : 8.0;
    double desired_gamma_rate = clamp_value((target_gamma - gamma) / pitch_response,
        -radians(8.0), radians(8.0));
    double horizontal_curvature = speed * speed / radius;
    double vertical_required = gravity * cos(gamma) -
        horizontal_curvature * cos(gamma) + speed * desired_gamma_rate;
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
        double bank_needed = degrees(asin(clamp_value(lateral / lift, -1.0, 1.0)));
        double bank = clamp_value(bank_needed, -bank_max, bank_max);
        double delivered_lateral = lift * sin(radians(bank));
        double delivered_vertical = lift * cos(radians(bank)) * cos(gamma);
        double vertical_error = fabs(delivered_vertical - vertical_required);
        double lateral_error = fabs(delivered_lateral - lateral);
        double score = vertical_error + 0.35 * lateral_error;
        if (score < best_error) {
            best_error = score;
            chosen_aoa = aoa_deg;
            chosen_bank = bank;
            chosen_vertical = delivered_vertical;
            available_lateral = lift * sin(radians(bank_max));
        }
      }
    }
    if (!isfinite(best_error)) return out;

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
