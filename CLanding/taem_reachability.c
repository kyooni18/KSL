#include "taem_reachability.h"

#include <math.h>
#include <string.h>

#include "shuttlesim/aero.h"
#include "shuttlesim/math3.h"

static const double reachability_pi = 3.14159265358979323846264338327950288;

static double radians(double degrees) { return degrees * reachability_pi / 180.0; }
static double degrees(double radians_value) { return radians_value * 180.0 / reachability_pi; }

bool taem_hac_geometry_sweep(const TerminalModel *m, double radius, double side,
        double sweep_abs_rad, TaemFixedHacGeometry *out) {
    if (!m || !out || !(radius >= 3000.0) || !isfinite(radius) ||
        (side != -1.0 && side != 1.0) ||
        !(sweep_abs_rad > 0.0) || !isfinite(sweep_abs_rad) ||
        sweep_abs_rad > 1.5 * reachability_pi * (1.0 + 1e-9) ||
        !(m->guidance.final_approach_distance > 0.0)) return false;

    /* Coordinates use runway-forward and runway-right axes. Course is
     * clockwise from north and positive relative course points to the right.
     * The circle, radius and Final exit remain fixed; only the capture point
     * moves around that circle. */
    double final_distance = m->guidance.final_approach_distance;
    TaemPoint2 exit = {-final_distance, 0.0};
    TaemPoint2 center = {exit.x, side * radius};
    double end_angle = atan2(exit.y - center.y, exit.x - center.x);
    /* The exit must be flown in the runway-forward (+x) direction.  At the
     * exit the radius vector points from the center toward the centerline,
     * so a center on the right (side > 0) requires increasing polar angle
     * and a center on the left decreasing angle: the sweep sign is +side. */
    double sweep_sign = side;
    double sweep = sweep_sign * sweep_abs_rad;
    double start_angle = end_angle - sweep;
    TaemPoint2 entry = {center.x + radius * cos(start_angle),
                        center.y + radius * sin(start_angle)};
    double tangent_delta = remainder(start_angle + sweep_sign * reachability_pi / 2.0,
                                      2.0 * reachability_pi);
    double capture_course = m->site.runway_heading + degrees(tangent_delta);

    *out = (TaemFixedHacGeometry){
        .entry = entry,
        .center = center,
        .exit = exit,
        .radius_m = radius,
        .side = side,
        .capture_course_deg = capture_course,
        .arc_sweep_rad = sweep,
        .arc_length_m = fabs(sweep) * radius,
        .final_length_m = final_distance,
        .total_length_m = fabs(sweep) * radius + final_distance
    };
    return true;
}

bool taem_fixed_hac_geometry(const TerminalModel *m, double radius, double side,
        TaemFixedHacGeometry *out) {
    return taem_hac_geometry_sweep(m, radius, side,
        1.5 * reachability_pi, out);
}

bool taem_fixed_hac_turn_reachability(const TerminalModel *m,
        const TerminalDynamicState *s, const TaemGeometryState *g,
        double radius, TaemReachability *out) {
    if (!m || !s || !g || !out || !(radius > 0.0) ||
        !(g->ground_speed_mps > 0.0) || !(s->mass_kg > 0.0)) return false;
    memset(out, 0, sizeof(*out));

    double altitude = v3_norm(s->position_i_m) - m->world.radius_m;
    double local_radius = m->world.radius_m + altitude;
    if (!(local_radius > 0.0) || !isfinite(m->world.mu_m3_s2) ||
        !(s->mass_kg > 0.0) || m->aero.alpha_count < 2) return false;

    double gravity = m->world.mu_m3_s2 / (local_radius * local_radius);
    /* Lift curves are not monotonic in AoA.  Use the configured subsonic
     * terminal CL-max incidence as the shared control/reachability limit, so
     * neither side assumes authority on the post-stall side of the curve. */
    double max_aoa = fmin(m->vehicle.maximum_angle_of_attack,
                          m->aero.alpha_deg[m->aero.alpha_count - 1]);
    AeroForces flow = aero_compute(&m->world, &m->aero,
        s->position_i_m, s->velocity_i_mps, s->ut_s, s->mass_kg, 0.0, 0.0);
    if (flow.mach < 1.0 &&
        isfinite(m->vehicle.terminal_maximum_lift_angle_of_attack) &&
        m->vehicle.terminal_maximum_lift_angle_of_attack > 0.0)
        max_aoa = fmin(max_aoa,
            m->vehicle.terminal_maximum_lift_angle_of_attack);
    double lift_accel = 0.0;
    for (double aoa_deg = 0.0; aoa_deg <= max_aoa + 1e-9; aoa_deg += 0.5) {
        AeroForces forces = aero_compute(&m->world, &m->aero,
            s->position_i_m, s->velocity_i_mps, s->ut_s, s->mass_kg,
            radians(aoa_deg), 0.0);
        if (isfinite(forces.lift_n))
            lift_accel = fmax(lift_accel, fmax(0.0, forces.lift_n) / s->mass_kg);
    }
    double normal_accel = fmin(lift_accel,
        fmax(0.0, m->vehicle.maximum_g_load) * gravity);
    double bank = fmin(m->vehicle.maximum_bank_angle, 80.0) * reachability_pi / 180.0;
    double lateral = normal_accel * sin(bank);
    double required = g->ground_speed_mps * g->ground_speed_mps / radius;
    double wn = m->attitude.roll_wn;
    double zeta = m->attitude.roll_zeta;
    double settle = (wn > 0.0 && zeta > 0.0) ? 4.0 / (wn * zeta) : INFINITY;
    if (!isfinite(lateral) || !isfinite(required) || !isfinite(settle)) return false;

    out->valid = true;
    out->required_lateral_accel_mps2 = required;
    out->available_lateral_accel_mps2 = lateral;
    out->lateral_margin_mps2 = lateral - required;
    out->minimum_turn_radius_m = lateral > 1e-9 ?
        g->ground_speed_mps * g->ground_speed_mps / lateral : INFINITY;
    out->roll_settling_time_s = settle;
    out->roll_settling_distance_m = g->ground_speed_mps * settle;
    out->lateral_authority_ok = out->lateral_margin_mps2 >= 0.0;
    /* Geometry and instantaneous authority cannot certify path energy. */
    out->energy_qualified = false;
    return true;
}
