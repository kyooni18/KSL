#include "taem_planner.h"

#include <math.h>
#include <stdio.h>
#include <string.h>

static const double planner_pi = 3.14159265358979323846264338327950288;

typedef struct { double x, y; } Point;

static double radians(double degrees) { return degrees * planner_pi / 180.0; }
static double degrees(double radians_value) { return radians_value * 180.0 / planner_pi; }
static double distance(Point a, Point b) { return hypot(a.x - b.x, a.y - b.y); }
static double clamp(double x, double lo, double hi) { return fmax(lo, fmin(hi, x)); }

static void vertical_profile(const TerminalModel *m, double start_altitude,
        double start_fpa_deg, double total_length, double distance_from_start,
        double midpoint_offset, double local_offset,
        double local_start_fraction, double local_peak_fraction,
        double local_end_fraction,
        double initial_sag, double initial_sag_length,
        double *altitude, double *fpa_deg, double *vertical_curvature_per_m) {
    double x = clamp(distance_from_start / total_length, 0.0, 1.0);
    double x2 = x * x, x3 = x2 * x;
    double h0 = start_altitude;
    double h1 = m->site.altitude;
    double slope0 = tan(radians(start_fpa_deg));
    double slope1 = -tan(radians(m->guidance.taem_glide_slope));
    double h00 = 2.0*x3 - 3.0*x2 + 1.0;
    double h10 = x3 - 2.0*x2 + x;
    double h01 = -2.0*x3 + 3.0*x2;
    double h11 = x3 - x2;
    /* The smooth bow changes only the interior profile. Its value and first
     * derivative are zero at both ends, while its second derivative is exposed
     * to the tracker as vertical-path curvature feed-forward. */
    double bow = 16.0 * x2 * (1.0 - x) * (1.0 - x);
    double bow_derivative_x = 32.0 * x * (1.0 - x) * (1.0 - 2.0*x);
    double bow_second_derivative_x = 32.0 * (1.0 - 6.0*x + 6.0*x2);
    double initial_u = initial_sag_length > 0.0 ?
        clamp(distance_from_start / initial_sag_length, 0.0, 1.0) : 0.0;
    double initial_bow = 16.0 * initial_u * initial_u *
        (1.0 - initial_u) * (1.0 - initial_u);
    double initial_slope = initial_sag_length > 0.0 ?
        initial_sag * 32.0 * initial_u * (1.0 - initial_u) *
            (1.0 - 2.0 * initial_u) / initial_sag_length : 0.0;
    double initial_second_slope = 0.0;
    if (initial_sag_length > 0.0 && distance_from_start <= initial_sag_length) {
        initial_second_slope = initial_sag * 32.0 *
            (1.0 - 6.0*initial_u + 6.0*initial_u*initial_u) /
            (initial_sag_length * initial_sag_length);
    }
    double local_bow = 0.0, local_slope = 0.0, local_second_slope = 0.0;
    double local_fall_span = local_peak_fraction - local_start_fraction;
    double local_recovery_span = local_end_fraction - local_peak_fraction;
    if (local_fall_span > 1e-6 && local_recovery_span > 1e-6) {
        if (x >= local_start_fraction && x < local_peak_fraction) {
            double u = clamp((x - local_start_fraction) / local_fall_span,
                0.0, 1.0);
            double smooth = u * u * (3.0 - 2.0 * u);
            local_bow = smooth;
            local_slope = local_offset * 6.0 * u * (1.0 - u) /
                (local_fall_span * total_length);
            local_second_slope = local_offset * 6.0 * (1.0 - 2.0*u) /
                (local_fall_span * local_fall_span * total_length * total_length);
        } else if (x >= local_peak_fraction && x <= local_end_fraction) {
            double u = clamp((x - local_peak_fraction) /
                local_recovery_span, 0.0, 1.0);
            double smooth = u * u * (3.0 - 2.0 * u);
            local_bow = 1.0 - smooth;
            local_slope = -local_offset * 6.0 * u * (1.0 - u) /
                (local_recovery_span * total_length);
            local_second_slope = -local_offset * 6.0 * (1.0 - 2.0*u) /
                (local_recovery_span * local_recovery_span * total_length * total_length);
        }
    }
    double height = h00*h0 + h10*total_length*slope0 +
                    h01*h1 + h11*total_length*slope1 +
                    midpoint_offset * bow + local_offset * local_bow +
                    initial_sag * initial_bow;
    double derivative_x = (6.0*x2 - 6.0*x)*h0 +
        (3.0*x2 - 4.0*x + 1.0)*total_length*slope0 +
        (-6.0*x2 + 6.0*x)*h1 + (3.0*x2 - 2.0*x)*total_length*slope1;
    derivative_x += midpoint_offset * bow_derivative_x;
    double second_derivative_x = (12.0*x - 6.0)*h0 +
        (6.0*x - 4.0)*total_length*slope0 +
        (-12.0*x + 6.0)*h1 + (6.0*x - 2.0)*total_length*slope1;
    second_derivative_x += midpoint_offset * bow_second_derivative_x;
    double slope = derivative_x / total_length + initial_slope + local_slope;
    if (altitude) *altitude = height;
    if (fpa_deg) *fpa_deg = degrees(atan(slope));
    if (vertical_curvature_per_m) {
        double second_slope = second_derivative_x /
            (total_length * total_length) + initial_second_slope + local_second_slope;
        *vertical_curvature_per_m = second_slope / (1.0 + slope*slope);
    }
}
#include "spiral_route.inc"

bool taem_route_build_hac(const TerminalModel *m,
        const TaemGeometryState *start, double hac_radius_m, double side,
        double sweep_abs_rad, double spacing, double maximum_lead_curvature,
        TaemRoute *route, char *reason, size_t reason_size) {
    return build_hac(m, start, hac_radius_m, side, sweep_abs_rad, spacing,
        maximum_lead_curvature, INFINITY, NAN, route, reason, reason_size);
}

bool taem_route_build_hac_shortest(const TerminalModel *m,
        const TaemGeometryState *start, double hac_radius_m, double side,
        double sweep_abs_rad, double spacing, double maximum_lead_curvature,
        double peak_curvature_limit, TaemRoute *route, char *reason,
        size_t reason_size) {
    if (!(peak_curvature_limit > 0.0) || !isfinite(peak_curvature_limit)) {
        if (reason && reason_size) snprintf(reason, reason_size, "invalid route input");
        return false;
    }
    return build_hac(m, start, hac_radius_m, side, sweep_abs_rad, spacing,
        maximum_lead_curvature, peak_curvature_limit, NAN, route, reason,
        reason_size);
}

bool taem_route_build_hac_length(const TerminalModel *m,
        const TaemGeometryState *start, double hac_radius_m, double side,
        double sweep_abs_rad, double spacing, double maximum_lead_curvature,
        double peak_curvature_limit, double target_lead_length_m,
        TaemRoute *route, char *reason, size_t reason_size) {
    if (!(peak_curvature_limit > 0.0) || !isfinite(peak_curvature_limit) ||
        !(target_lead_length_m > 0.0) || !isfinite(target_lead_length_m)) {
        if (reason && reason_size) snprintf(reason, reason_size, "invalid route input");
        return false;
    }
    return build_hac(m, start, hac_radius_m, side, sweep_abs_rad, spacing,
        maximum_lead_curvature, peak_curvature_limit, target_lead_length_m,
        route, reason, reason_size);
}

bool taem_route_blend_hac_length(const TerminalModel *m,
        const TaemGeometryState *start, const TaemRoute *a,
        const TaemRoute *b, double spacing, double initial_limit,
        double peak_limit, double target, TaemRoute *route,
        char *reason, size_t reason_size) {
    if (!a || !b || !a->valid || !b->valid || a->side != b->side ||
        a->hac.radius_m != b->hac.radius_m || a->hac.arc_sweep_rad != b->hac.arc_sweep_rad)
        return false;
    return build_hac(m,start,a->hac.radius_m,a->side,fabs(a->hac.arc_sweep_rad),
        spacing,initial_limit,peak_limit,target,route,reason,reason_size);
}

double taem_route_lead_peak_curvature(const TaemRoute *route) {
    if (!route || !route->valid) return INFINITY;
    TaemRoute copy=*route;
    return capture_measure(&copy,false);
}

bool taem_route_build_fixed_hac(const TerminalModel *m,
        const TaemGeometryState *start, double hac_radius_m, double side,
        double spacing, double maximum_lead_curvature,
        TaemRoute *route, char *reason, size_t reason_size) {
    return taem_route_build_hac(m, start, hac_radius_m, side,
        1.5 * planner_pi, spacing, maximum_lead_curvature,
        route, reason, reason_size);
}

double taem_route_planned_speed(const TaemRoute *route, double station_m) {
    if (!route || !route->profile_speed_tabulated ||
        !(route->profile_total_length_m > 0.0) || !isfinite(station_m)) return NAN;
    const size_t intervals = TAEM_ROUTE_PROFILE_LUT_POINTS - 1;
    double u = clamp(station_m / route->profile_total_length_m, 0.0, 1.0) *
        (double)intervals;
    size_t i = (size_t)fmin(floor(u), (double)intervals - 1.0);
    double f = u - (double)i;
    return (1.0 - f) * route->profile_speed_lut[i] +
           f * route->profile_speed_lut[i + 1];
}

double taem_route_planned_altitude(const TaemRoute *route, double station_m) {
    if (!route || !route->profile_tabulated ||
        !(route->profile_total_length_m > 0.0) || !isfinite(station_m)) return NAN;
    const size_t intervals = TAEM_ROUTE_PROFILE_LUT_POINTS - 1;
    double u = clamp(station_m / route->profile_total_length_m, 0.0, 1.0) *
        (double)intervals;
    size_t i = (size_t)fmin(floor(u), (double)intervals - 1.0);
    double f = u - (double)i;
    return (1.0 - f) * route->profile_altitude_lut[i] +
           f * route->profile_altitude_lut[i + 1];
}

bool taem_route_limit_initial_vertical_authority(const TerminalModel *m,
        const TerminalDynamicState *state, const TaemGeometryState *geometry,
        TaemRoute *route) {
    if (!m || !state || !geometry || !route || !route->valid ||
        !(state->mass_kg > 1.0) || !(geometry->airspeed_mps > 1.0) ||
        !(geometry->ground_speed_mps > 1.0) ||
        !(route->profile_total_length_m > 100.0))
        return false;

    /* Evaluate the unmodified analytic profile at the live route origin.  The
       Hermite curve already matches the measured FPA there; only its second
       derivative may be physically impossible. */
    route->profile_initial_sag_m = 0.0;
    route->profile_initial_sag_length_m = 0.0;
    double base_altitude = NAN, base_fpa = NAN, base_curvature = NAN;
    vertical_profile(&(TerminalModel){
        .site.altitude = route->profile_final_altitude_m,
        .guidance.taem_glide_slope = route->profile_final_slope_deg
    }, route->profile_start_altitude_m, route->profile_start_fpa_deg,
        route->profile_total_length_m, 0.0,
        route->profile_midpoint_offset_m, route->profile_local_offset_m,
        route->profile_local_start_fraction, route->profile_local_peak_fraction,
        route->profile_local_end_fraction, 0.0, 0.0,
        &base_altitude, &base_fpa, &base_curvature);
    if (!isfinite(base_curvature) || !isfinite(base_fpa)) return false;

    double altitude = sqrt(
        state->position_i_m.x*state->position_i_m.x +
        state->position_i_m.y*state->position_i_m.y +
        state->position_i_m.z*state->position_i_m.z) - m->world.radius_m;
    double local_radius = m->world.radius_m + altitude;
    if (!(local_radius > 1.0)) return false;
    double gravity = m->world.mu_m3_s2 / (local_radius * local_radius);

    double aoa_max = fmin(m->vehicle.maximum_angle_of_attack,
        m->aero.alpha_deg[m->aero.alpha_count - 1]);
    AeroForces flow = aero_compute(&m->world, &m->aero,
        state->position_i_m, state->velocity_i_mps, state->ut_s,
        state->mass_kg, 0.0, 0.0);
    if (flow.mach < 1.0 &&
        isfinite(m->vehicle.terminal_maximum_lift_angle_of_attack) &&
        m->vehicle.terminal_maximum_lift_angle_of_attack > 0.0)
        aoa_max = fmin(aoa_max,
            m->vehicle.terminal_maximum_lift_angle_of_attack);
    if (state->trim_aoa_ceiling_rad > 0.0)
        aoa_max = fmin(aoa_max, degrees(state->trim_aoa_ceiling_rad));

    double g_limit = fmax(0.0, m->vehicle.maximum_g_load) * gravity;
    double lift_max = 0.0;
    for (double aoa = 0.0; aoa <= aoa_max + 1e-9; aoa += 0.5) {
        AeroForces f = aero_compute(&m->world, &m->aero,
            state->position_i_m, state->velocity_i_mps, state->ut_s,
            state->mass_kg, radians(aoa), 0.0);
        if (isfinite(f.lift_n))
            lift_max = fmax(lift_max,
                fmin(fabs(f.lift_n) / state->mass_kg, g_limit));
    }
    if (!(lift_max > 1e-6)) return false;

    double lateral_curvature = taem_route_curvature_at_station(route, 0.0);
    if (!isfinite(lateral_curvature)) return false;
    double lateral_required = fabs(geometry->ground_speed_mps *
        geometry->ground_speed_mps * lateral_curvature);
    double bank_limit = radians(fmin(m->vehicle.maximum_bank_angle, 80.0));
    if (lateral_required > lift_max * sin(bank_limit) + 1e-6)
        return false;

    double vertical_available = sqrt(fmax(0.0,
        lift_max * lift_max - lateral_required * lateral_required));
    /* Leave modest control margin so a route does not depend on sitting exactly
       on the identified trim/lift ceiling from its first sample. */
    vertical_available *= 0.95;

    double radial_required = taem_tracker_vertical_lift_demand(
        m, state, geometry, 0.0);
    if (!isfinite(radial_required)) return false;
    double gamma_rate_max =
        (vertical_available - radial_required) / geometry->airspeed_mps;
    double curvature_max = gamma_rate_max / geometry->ground_speed_mps;

    if (base_curvature <= curvature_max + 1e-9)
        return true;

    double pitch_response = m->attitude.pitch_wn > 0.0 &&
        m->attitude.pitch_zeta > 0.0 ?
        4.0 / (m->attitude.pitch_wn * m->attitude.pitch_zeta) : 4.0;
    double sag_length = geometry->ground_speed_mps *
        fmax(2.0, 2.0 * pitch_response);
    sag_length = fmax(1000.0, fmin(sag_length,
        fmin(6000.0, 0.30 * route->profile_total_length_m)));
    if (!(sag_length > 100.0)) return false;

    double slope0 = tan(radians(route->profile_start_fpa_deg));
    double delta_second_slope =
        (curvature_max - base_curvature) * (1.0 + slope0 * slope0);
    double sag = delta_second_slope * sag_length * sag_length / 32.0;
    double drop = route->profile_start_altitude_m -
        route->profile_final_altitude_m;
    double sag_limit = fmax(50.0, 0.20 * fmax(0.0, drop));
    sag = fmax(-sag_limit, fmin(sag_limit, sag));

    route->profile_initial_sag_m = sag;
    route->profile_initial_sag_length_m = sag_length;
    return true;
}

double taem_route_curvature_at_station(const TaemRoute *route, double station_m) {
    Point position;
    double course = NAN, curvature = NAN;
    if (!route_sample_at_station(route, station_m, &position, &course, &curvature))
        return NAN;
    return curvature;
}

bool taem_route_sample(const TaemRoute *route, double station, TaemRoutePoint *point) {
    Point p; double h,k;
    if (!point || !route_sample_at_station(route,station,&p,&h,&k)) return false;
    *point=(TaemRoutePoint){.along_m=p.x,.cross_m=p.y,.course_deg=h,
        .curvature_right_per_m=k,.altitude_m=taem_route_planned_altitude(route,station),
        .flight_path_angle_deg=NAN,.distance_from_start_m=station};
    return true;
}

bool taem_route_point_at_index(const TaemRoute *route, size_t index,
        TaemRoutePoint *point) {
    if (!route || !route->valid || route->count < 2 || !point ||
        index >= route->count) return false;
    double station = route_station(route, index);
    Point position;
    double course = NAN, curvature = NAN;
    if (!route_sample_at_station(route, station, &position, &course, &curvature))
        return false;
    double altitude = NAN;
    if (route->profile_tabulated)
        altitude = taem_route_planned_altitude(route, station);
    else if (route->profile_total_length_m > 0.0) {
        double f = clamp(station / route->profile_total_length_m, 0.0, 1.0);
        altitude = route->profile_start_altitude_m +
            f * (route->profile_final_altitude_m - route->profile_start_altitude_m);
    }
    *point = (TaemRoutePoint){
        .along_m = position.x,
        .cross_m = position.y,
        .course_deg = course,
        .curvature_right_per_m = curvature,
        .altitude_m = altitude,
        .flight_path_angle_deg = NAN,
        .distance_from_start_m = station
    };
    return true;
}

bool taem_route_reference(const TaemRoute *route, const TaemGeometryState *state,
        size_t *cursor, TaemPathReference *reference, size_t *point_index) {
    if (!route || !route->valid || route->count < 3 || !state || !cursor ||
        !reference || *cursor >= route->count) return false;

    size_t start = *cursor;
    size_t lookahead = 16;
    size_t end = start + lookahead < route->count ?
        start + lookahead : route->count - 1;
    double best_d2 = INFINITY;
    double best_station = route_station(route, start);
    size_t best_segment = start;

    if (start == route->count - 1) {
        Point p = route_position(route, start);
        double dx = state->runway_along_m - p.x;
        double dy = state->runway_cross_m - p.y;
        best_d2 = dx * dx + dy * dy;
    } else {
        for (size_t i = start; i < end; ++i) {
            Point a = route_position(route, i);
            Point b = route_position(route, i + 1);
            double vx = b.x - a.x;
            double vy = b.y - a.y;
            double vv = vx * vx + vy * vy;
            if (!(vv > 1e-12)) continue;
            double wx = state->runway_along_m - a.x;
            double wy = state->runway_cross_m - a.y;
            double u = clamp((wx * vx + wy * vy) / vv, 0.0, 1.0);
            Point q = {a.x + u * vx, a.y + u * vy};
            double dx = state->runway_along_m - q.x;
            double dy = state->runway_cross_m - q.y;
            double d2 = dx * dx + dy * dy;
            if (d2 < best_d2) {
                double s0 = route_station(route, i);
                double s1 = route_station(route, i + 1);
                best_d2 = d2;
                best_station = s0 + u * (s1 - s0);
                best_segment = i;
            }
        }
    }
    if (!isfinite(best_d2)) return false;

    size_t progress_index = best_segment;
    if (best_segment + 1 < route->count &&
        best_station >= route_station(route, best_segment + 1) - 1e-6)
        progress_index = best_segment + 1;
    *cursor = progress_index;

    Point center;
    double course = NAN, curvature = NAN;
    if (!route_sample_at_station(route, best_station, &center, &course, &curvature))
        return false;

    double altitude = NAN, fpa = NAN, vertical_curvature = NAN;
    if (route->profile_tabulated) {
        const size_t intervals = TAEM_ROUTE_PROFILE_LUT_POINTS - 1;
        double length = route->profile_total_length_m;
        double u = clamp(best_station / length, 0.0, 1.0) * (double)intervals;
        size_t i = (size_t)fmin(floor(u), (double)intervals - 1.0);
        double f = u - (double)i;
        altitude = (1.0 - f) * route->profile_altitude_lut[i] +
                   f * route->profile_altitude_lut[i + 1];
        fpa = (1.0 - f) * route->profile_fpa_lut[i] +
              f * route->profile_fpa_lut[i + 1];
        double ds = length / (double)intervals;
        vertical_curvature = radians(route->profile_fpa_lut[i + 1] -
                                     route->profile_fpa_lut[i]) / ds;
    } else vertical_profile(&(TerminalModel){
        .site.altitude = route->profile_final_altitude_m,
        .guidance.taem_glide_slope = route->profile_final_slope_deg
    }, route->profile_start_altitude_m, route->profile_start_fpa_deg,
        route->profile_total_length_m, best_station,
        route->profile_midpoint_offset_m, route->profile_local_offset_m,
        route->profile_local_start_fraction, route->profile_local_peak_fraction,
        route->profile_local_end_fraction,
        route->profile_initial_sag_m,
        route->profile_initial_sag_length_m, &altitude, &fpa, &vertical_curvature);
    *reference = (TaemPathReference){
        .runway_along_m = center.x,
        .runway_cross_m = center.y,
        .course_deg = course,
        .curvature_right_per_m = curvature,
        .altitude_m = altitude,
        .flight_path_angle_deg = fpa,
        .vertical_curvature_per_m = vertical_curvature,
        .station_m = best_station
    };
    if (point_index) *point_index = progress_index;
    return true;
}
