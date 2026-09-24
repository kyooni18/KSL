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
        double *altitude, double *fpa_deg) {
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
    double height = h00*h0 + h10*total_length*slope0 +
                    h01*h1 + h11*total_length*slope1;
    double derivative_x = (6.0*x2 - 6.0*x)*h0 +
        (3.0*x2 - 4.0*x + 1.0)*total_length*slope0 +
        (-6.0*x2 + 6.0*x)*h1 + (3.0*x2 - 2.0*x)*total_length*slope1;
    if (altitude) *altitude = height;
    if (fpa_deg) *fpa_deg = degrees(atan(derivative_x / total_length));
}
static Point cubic(Point p0, Point p1, Point p2, Point p3, double t) {
    double q = 1.0 - t;
    return (Point){
        q*q*q*p0.x + 3.0*q*q*t*p1.x + 3.0*q*t*t*p2.x + t*t*t*p3.x,
        q*q*q*p0.y + 3.0*q*q*t*p1.y + 3.0*q*t*t*p2.y + t*t*t*p3.y
    };
}

static Point cubic_derivative(Point p0, Point p1, Point p2, Point p3, double t) {
    double q = 1.0 - t;
    return (Point){
        3.0*q*q*(p1.x-p0.x) + 6.0*q*t*(p2.x-p1.x) + 3.0*t*t*(p3.x-p2.x),
        3.0*q*q*(p1.y-p0.y) + 6.0*q*t*(p2.y-p1.y) + 3.0*t*t*(p3.y-p2.y)
    };
}

static Point cubic_second_derivative(Point p0, Point p1, Point p2, Point p3,
                                     double t) {
    double q = 1.0 - t;
    return (Point){
        6.0*q*(p2.x - 2.0*p1.x + p0.x) +
            6.0*t*(p3.x - 2.0*p2.x + p1.x),
        6.0*q*(p2.y - 2.0*p1.y + p0.y) +
            6.0*t*(p3.y - 2.0*p2.y + p1.y)
    };
}

static double cubic_peak_curvature(Point p0, Point p1, Point p2, Point p3) {
    double peak = 0.0;
    Point previous = cubic_derivative(p0, p1, p2, p3, 0.0);
    for (int i = 0; i <= 128; ++i) {
        double t = (double)i / 128.0;
        Point d = cubic_derivative(p0, p1, p2, p3, t);
        Point dd = cubic_second_derivative(p0, p1, p2, p3, t);
        double speed = hypot(d.x, d.y);
        if (speed < 1.0 || d.x*previous.x + d.y*previous.y <= 0.0)
            return INFINITY;
        peak = fmax(peak, fabs(d.x*dd.y - d.y*dd.x) / (speed*speed*speed));
        previous = d;
    }
    return peak;
}

static Point route_position(const TaemRoute *route, size_t index) {
    Point p0 = {route->p0_along_m, route->p0_cross_m};
    Point p1 = {route->p1_along_m, route->p1_cross_m};
    Point p2 = {route->p2_along_m, route->p2_cross_m};
    Point p3 = {route->p3_along_m, route->p3_cross_m};
    if (index <= route->lead_count) {
        double t = (double)index / (double)route->lead_count;
        return cubic(p0, p1, p2, p3, t);
    }
    size_t arc_index = index - route->lead_count;
    double end_angle = atan2(route->hac.exit.y - route->hac.center.y,
                             route->hac.exit.x - route->hac.center.x);
    double start_angle = end_angle - route->hac.arc_sweep_rad;
    double t = (double)arc_index / (double)route->arc_count;
    double angle = start_angle + route->hac.arc_sweep_rad * t;
    return (Point){route->hac.center.x + route->hac.radius_m * cos(angle),
                   route->hac.center.y + route->hac.radius_m * sin(angle)};
}

static double route_station(const TaemRoute *route, size_t index) {
    double station = 0.0;
    Point previous = route_position(route, 0);
    for (size_t i = 1; i <= index; ++i) {
        Point current = route_position(route, i);
        station += distance(previous, current);
        previous = current;
    }
    return station;
}

static bool route_finish(const TerminalModel *m, const TaemGeometryState *start,
                         TaemRoute *route) {
    if (route->count < 3 || route->lead_count < 2 || route->arc_count < 3)
        return false;
    route->length_m = route_station(route, route->count - 1);
    if (!(route->length_m > 0.0) || !isfinite(route->length_m)) return false;
    /* The live MM305 tracker owns only the route through the HAC exit. Keep
     * that endpoint above runway elevation at the configured Final glide
     * height; Final's existing numeric admission contract decides whether the
     * actual state can transfer into the downstream approach. */
    route->profile_total_length_m = route->length_m;
    route->profile_start_altitude_m = m->site.altitude + start->altitude_above_runway_m;
    route->profile_start_fpa_deg = start->flight_path_angle_deg;
    route->profile_final_altitude_m = m->site.altitude +
        route->hac.final_length_m * tan(radians(m->guidance.final_glide_slope));
    route->profile_final_slope_deg = m->guidance.final_glide_slope;
    route->valid = true;
    return true;
}

bool taem_route_build_fixed_hac(const TerminalModel *m,
        const TaemGeometryState *start, double side, double spacing,
        double maximum_lead_curvature,
        TaemRoute *route, char *reason, size_t reason_size) {
    if (reason && reason_size) reason[0] = '\0';
    if (!m || !start || !route || !(spacing >= 100.0) || !isfinite(spacing) ||
        (side != -1.0 && side != 1.0) ||
        !(maximum_lead_curvature > 0.0) || !isfinite(maximum_lead_curvature)) {
        if (reason && reason_size) snprintf(reason, reason_size, "invalid route input");
        return false;
    }
    memset(route, 0, sizeof(*route));
    if (!taem_fixed_hac_geometry(m, m->guidance.hac_radius, side, &route->hac)) {
        if (reason && reason_size) snprintf(reason, reason_size, "fixed HAC geometry is invalid");
        return false;
    }
    route->side = side;

    Point p0 = {start->runway_along_m, start->runway_cross_m};
    Point p3 = {route->hac.entry.x, route->hac.entry.y};
    double initial_delta = radians(start->course_deg - m->site.runway_heading);
    double final_delta = radians(route->hac.capture_course_deg - m->site.runway_heading);
    Point initial_tangent = {cos(initial_delta), sin(initial_delta)};
    Point final_tangent = {cos(final_delta), sin(final_delta)};
    double direct = distance(p0, p3);
    if (!(direct > 500.0)) {
        if (reason && reason_size) snprintf(reason, reason_size, "lead-to-HAC distance is too short");
        return false;
    }
    double handle = fmax(1200.0, start->ground_speed_mps * 12.0);
    bool lead_feasible = false;
    for (int trial = 0; trial < 32; ++trial) {
        Point p1_try = {p0.x + handle * initial_tangent.x,
                        p0.y + handle * initial_tangent.y};
        Point p2_try = {p3.x - handle * final_tangent.x,
                        p3.y - handle * final_tangent.y};
        if (cubic_peak_curvature(p0, p1_try, p2_try, p3) <=
                maximum_lead_curvature) {
            lead_feasible = true;
            break;
        }
        handle *= 1.18;
    }
    if (!lead_feasible) {
        if (reason && reason_size)
            snprintf(reason, reason_size, "finite lead turn exceeds available curvature authority");
        memset(route, 0, sizeof(*route));
        return false;
    }
    Point p1 = {p0.x + handle * initial_tangent.x,
                p0.y + handle * initial_tangent.y};
    Point p2 = {p3.x - handle * final_tangent.x,
                p3.y - handle * final_tangent.y};
    double lead_count_d = ceil(direct / spacing);
    size_t lead_count = (size_t)fmax(2.0, fmin(lead_count_d, 500.0));
    size_t arc_count = (size_t)fmax(3.0, ceil(route->hac.arc_length_m / spacing));
    if (lead_count + arc_count + 1 > TAEM_ROUTE_MAX_POINTS) goto capacity_failure;
    route->lead_count = lead_count;
    route->arc_count = arc_count;
    route->count = lead_count + arc_count + 1;
    route->p0_along_m = p0.x; route->p0_cross_m = p0.y;
    route->p1_along_m = p1.x; route->p1_cross_m = p1.y;
    route->p2_along_m = p2.x; route->p2_cross_m = p2.y;
    route->p3_along_m = p3.x; route->p3_cross_m = p3.y;
    route->initial_course_deg = start->course_deg;
    route->runway_heading_deg = m->site.runway_heading;

    if (!route_finish(m, start, route)) {
        if (reason && reason_size) snprintf(reason, reason_size, "route sampling produced invalid geometry");
        return false;
    }
    return true;

capacity_failure:
    if (reason && reason_size) snprintf(reason, reason_size, "fixed HAC route exceeded point capacity");
    memset(route, 0, sizeof(*route));
    return false;
}

bool taem_route_reference(const TaemRoute *route, const TaemGeometryState *state,
        size_t *cursor, TaemPathReference *reference, size_t *point_index) {
    if (!route || !route->valid || route->count < 3 || !state || !cursor ||
        !reference || *cursor >= route->count) return false;
    size_t best = *cursor;
    double best_d2 = INFINITY;
    size_t lookahead = 16;
    size_t end = best + lookahead < route->count ? best + lookahead : route->count - 1;
    for (size_t i = best; i <= end; ++i) {
        Point p = route_position(route, i);
        double dx = state->runway_along_m - p.x;
        double dy = state->runway_cross_m - p.y;
        double d2 = dx * dx + dy * dy;
        if (d2 < best_d2) { best_d2 = d2; best = i; }
    }
    *cursor = best;
    Point center = route_position(route, best);
    size_t a = best ? best - 1 : best;
    size_t b = best + 1 < route->count ? best + 1 : best;
    size_t c = best + 2 < route->count ? best + 2 : b;
    Point pa = route_position(route, a);
    Point pc = route_position(route, c);
    double course = route->initial_course_deg;
    if (pc.x != pa.x || pc.y != pa.y)
        course = route->runway_heading_deg + degrees(atan2(pc.y - pa.y, pc.x - pa.x));
    double curvature = 0.0;
    if (best > 0 && best + 1 < route->count) {
        Point previous = route_position(route, best - 1);
        Point next = route_position(route, best + 1);
        double ax = center.x - previous.x, ay = center.y - previous.y;
        double bx = next.x - center.x, by = next.y - center.y;
        double chord = distance(previous, next);
        double cross = ax * by - ay * bx;
        double denom = distance(previous, center) * distance(center, next) * chord;
        if (denom > 1e-8) curvature = 2.0 * cross / denom;
    }
    double station = route_station(route, best);
    double altitude = NAN, fpa = NAN;
    vertical_profile(&(TerminalModel){
        .site.altitude = route->profile_final_altitude_m,
        .guidance.taem_glide_slope = route->profile_final_slope_deg
    }, route->profile_start_altitude_m, route->profile_start_fpa_deg,
        route->profile_total_length_m, station, &altitude, &fpa);
    *reference = (TaemPathReference){
        .runway_along_m = center.x,
        .runway_cross_m = center.y,
        .course_deg = best == 0 ? route->initial_course_deg : course,
        .curvature_right_per_m = curvature,
        .altitude_m = altitude,
        .flight_path_angle_deg = fpa
    };
    if (point_index) *point_index = best;
    return true;
}
