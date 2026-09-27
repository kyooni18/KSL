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


static double cubic_length_estimate(Point p0, Point p1, Point p2, Point p3) {
    Point previous = p0;
    double length = 0.0;
    for (int i = 1; i <= 64; ++i) {
        Point current = cubic(p0, p1, p2, p3, (double)i / 64.0);
        length += distance(previous, current);
        previous = current;
    }
    return length;
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
        peak = fmax(peak, fabs(d.x*dd.y - d.y*dd.x) /
            (speed*speed*speed));
        previous = d;
    }
    return peak;
}


static bool cubic_forward_regular(Point p0, Point p1, Point p2, Point p3) {
    Point previous = cubic_derivative(p0, p1, p2, p3, 0.0);
    if (hypot(previous.x, previous.y) < 1.0) return false;
    for (int i = 1; i <= 128; ++i) {
        double t = (double)i / 128.0;
        Point d = cubic_derivative(p0, p1, p2, p3, t);
        if (hypot(d.x, d.y) < 1.0 || d.x*previous.x + d.y*previous.y <= 0.0)
            return false;
        previous = d;
    }
    return true;
}

static double cubic_endpoint_curvature(Point p0, Point p1, Point p2, Point p3,
        double t) {
    Point d = cubic_derivative(p0, p1, p2, p3, t);
    Point dd = cubic_second_derivative(p0, p1, p2, p3, t);
    double speed = hypot(d.x, d.y);
    if (speed < 1.0) return INFINITY;
    return fabs(d.x*dd.y - d.y*dd.x) / (speed*speed*speed);
}

static bool cubic_arc_lookup(TaemRoute *route) {
    Point p0 = {route->p0_along_m, route->p0_cross_m};
    Point p1 = {route->p1_along_m, route->p1_cross_m};
    Point p2 = {route->p2_along_m, route->p2_cross_m};
    Point p3 = {route->p3_along_m, route->p3_cross_m};
    Point previous = cubic(p0, p1, p2, p3, 0.0);
    double cumulative[TAEM_ROUTE_LEAD_LUT_POINTS] = {0.0};
    const size_t intervals = TAEM_ROUTE_LEAD_LUT_POINTS - 1;
    for (size_t i = 1; i <= intervals; ++i) {
        Point current = cubic(p0, p1, p2, p3, (double)i / intervals);
        cumulative[i] = cumulative[i - 1] + distance(previous, current);
        previous = current;
    }
    route->lead_length_m = cumulative[intervals];
    if (!(route->lead_length_m > 0.0) || !isfinite(route->lead_length_m))
        return false;
    for (size_t i = 0; i <= intervals; ++i)
        route->lead_arc_fraction_lut[i] =
            (float)(cumulative[i] / route->lead_length_m);
    route->lead_arc_fraction_lut[0] = 0.0f;
    route->lead_arc_fraction_lut[intervals] = 1.0f;
    return true;
}

static double cubic_t_at_arc_fraction(const TaemRoute *route, double fraction) {
    const size_t intervals = TAEM_ROUTE_LEAD_LUT_POINTS - 1;
    fraction = clamp(fraction, 0.0, 1.0);
    size_t low = 0, high = intervals;
    while (low + 1 < high) {
        size_t middle = low + (high - low) / 2;
        if ((double)route->lead_arc_fraction_lut[middle] < fraction)
            low = middle;
        else
            high = middle;
    }
    double a = route->lead_arc_fraction_lut[low];
    double b = route->lead_arc_fraction_lut[high];
    double part = b > a ? (fraction - a) / (b - a) : 0.0;
    return ((double)low + clamp(part, 0.0, 1.0)) / intervals;
}

static Point route_position(const TaemRoute *route, size_t index) {
    Point p0 = {route->p0_along_m, route->p0_cross_m};
    Point p1 = {route->p1_along_m, route->p1_cross_m};
    Point p2 = {route->p2_along_m, route->p2_cross_m};
    Point p3 = {route->p3_along_m, route->p3_cross_m};
    if (index <= route->lead_count) {
        double t = cubic_t_at_arc_fraction(route,
            (double)index / (double)route->lead_count);
        return cubic(p0, p1, p2, p3, t);
    }
    if (index <= route->lead_count + route->arc_count) {
        size_t arc_index = index - route->lead_count;
        double end_angle = atan2(route->hac.exit.y - route->hac.center.y,
                                 route->hac.exit.x - route->hac.center.x);
        double start_angle = end_angle - route->hac.arc_sweep_rad;
        double t = (double)arc_index / (double)route->arc_count;
        double angle = start_angle + route->hac.arc_sweep_rad * t;
        return (Point){route->hac.center.x + route->hac.radius_m * cos(angle),
                       route->hac.center.y + route->hac.radius_m * sin(angle)};
    }
    size_t rollout_index = index - route->lead_count - route->arc_count;
    double t = route->rollout_count > 0 ?
        clamp((double)rollout_index / (double)route->rollout_count, 0.0, 1.0) : 1.0;
    return (Point){
        route->hac.exit.x + t * (route->alignment_along_m - route->hac.exit.x),
        0.0
    };
}

static double route_station(const TaemRoute *route, size_t index) {
    return taem_route_station_at_index(route,index);
}


static bool route_finish(const TerminalModel *m, const TaemGeometryState *start,
                         TaemRoute *route) {
    if (route->count < 3 || route->lead_count < 2 || route->arc_count < 3)
        return false;
    route->length_m = route->lead_length_m + route->hac.arc_length_m +
        route->rollout_length_m;
    if (!(route->length_m > 0.0) || !isfinite(route->length_m)) return false;

    /* MM305 owns the full path through the bank-unload rollout and ends at the
       fixed Final alignment station.  The vertical/speed profile therefore also
       terminates at that station rather than at the circular HAC exit. */
    route->profile_total_length_m = route->length_m;
    route->profile_start_altitude_m = m->site.altitude + start->altitude_above_runway_m;
    route->profile_start_fpa_deg = start->flight_path_angle_deg;
    route->profile_final_altitude_m = m->site.altitude +
        m->guidance.final_approach_distance * tan(radians(m->guidance.final_glide_slope));
    route->profile_final_slope_deg = m->guidance.final_glide_slope;
    route->profile_midpoint_offset_m = 0.0;
    route->profile_local_offset_m = 0.0;
    route->profile_local_start_fraction = 0.08;
    route->profile_local_peak_fraction = 0.24;
    route->profile_local_end_fraction = 0.40;
    route->profile_initial_sag_m = 0.0;
    route->profile_initial_sag_length_m = 0.0;
    route->valid = true;
    return true;
}

/* peak_curvature_limit == INFINITY selects the smoothest lead (minimum peak
 * curvature).  A finite limit selects the shortest lead whose peak curvature
 * stays within it: a compact acquisition that the smoothest-lead rule would
 * replace with a long loop. */
static bool build_hac(const TerminalModel *m,
        const TaemGeometryState *start, double hac_radius_m, double side,
        double sweep_abs_rad, double spacing, double maximum_lead_curvature,
        double peak_curvature_limit, double target_polygon_m,
        TaemRoute *route, char *reason, size_t reason_size) {
    if (reason && reason_size) reason[0] = '\0';
    if (!m || !start || !route || !(hac_radius_m > 0.0) || !isfinite(hac_radius_m) ||
        !(spacing >= 100.0) || !isfinite(spacing) ||
        !(hac_radius_m >= 3000.0) || !isfinite(hac_radius_m) ||
        (side != -1.0 && side != 1.0) ||
        !(sweep_abs_rad > 0.0) || !isfinite(sweep_abs_rad) ||
        !(maximum_lead_curvature > 0.0) || !isfinite(maximum_lead_curvature)) {
        if (reason && reason_size) snprintf(reason, reason_size, "invalid route input");
        return false;
    }
    memset(route, 0, sizeof(*route));
    if (!taem_hac_geometry_sweep(m, hac_radius_m, side, sweep_abs_rad, &route->hac)) {
        if (reason && reason_size) snprintf(reason, reason_size, "fixed HAC geometry is invalid");
        return false;
    }
    route->side = side;

    Point p0 = {start->runway_along_m, start->runway_cross_m};

    /* sweep_abs_rad selects the join station on the fixed analytic circle
     * (center, radius, exit and turn direction are unchanged).  The candidate
     * search owns that choice, so a state that has already passed part of the
     * canonical 270-degree HAC can join the remaining arc directly. */
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
    /* Current turn authority constrains the lead through the measured roll
       settling distance, not merely at one mathematical endpoint.  Later
       curvature may tighten as speed/q evolve, but native replay must prove it. */
    /* Handle length is a geometric search variable, not a time-of-flight
     * constraint.  Starting at speed*12s excluded short, physically smooth
     * captures when the vehicle was already adjacent to the HAC.  Curvature,
     * forward-regularity, and full native replay remain the authority gates. */
    double initial_handle_min = fmax(2.0 * spacing, 300.0);
    double final_handle_min = fmax(2.0 * spacing, 300.0);
    double maximum_handle = fmax(8.0 * direct,
        fmax(initial_handle_min, final_handle_min));
    bool lead_feasible = false;
    Point p1 = {0.0, 0.0}, p2 = {0.0, 0.0};
    double best_control_polygon = INFINITY;
    double best_peak_curvature = INFINITY;
    double best_target_error = INFINITY;
    /* Only the route's instantaneous starting curvature is constrained by
       the *current* thin-air lateral authority here.  Authority later on the
       lead changes rapidly with q and speed, so applying today's curvature
       ceiling to a finite prefix or the whole cubic is not a physical gate.
       Native profile propagation and replay evaluate lateral authority at every
       subsequent station and remain the authoritative rejection mechanism. */
    double initial_handle = initial_handle_min;
    for (int initial_trial = 0; initial_trial < 32 &&
            initial_handle <= maximum_handle * 1.000001; ++initial_trial) {
        double final_handle = final_handle_min;
        for (int final_trial = 0; final_trial < 32 &&
                final_handle <= maximum_handle * 1.000001; ++final_trial) {
            Point p1_try = {p0.x + initial_handle * initial_tangent.x,
                            p0.y + initial_handle * initial_tangent.y};
            Point p2_try = {p3.x - final_handle * final_tangent.x,
                            p3.y - final_handle * final_tangent.y};
            if (!cubic_forward_regular(p0, p1_try, p2_try, p3)) {
                final_handle *= 1.18;
                continue;
            }
            double initial_curvature = cubic_endpoint_curvature(
                p0, p1_try, p2_try, p3, 0.0);
            double peak_curvature = cubic_peak_curvature(p0, p1_try, p2_try, p3);
            if (isfinite(initial_curvature) && isfinite(peak_curvature) &&
                initial_curvature <= maximum_lead_curvature * 1.000001) {
                double control_polygon = initial_handle +
                    distance(p1_try, p2_try) + final_handle;
                double curvature_tolerance = 0.02 *
                    fmax(maximum_lead_curvature, 1e-12);
                bool smoother = !isfinite(best_peak_curvature) ||
                    peak_curvature < best_peak_curvature - curvature_tolerance;
                bool similarly_smooth = isfinite(best_peak_curvature) &&
                    fabs(peak_curvature-best_peak_curvature) <= curvature_tolerance;
                double target_error = isfinite(target_polygon_m) ?
                    fabs(cubic_length_estimate(p0, p1_try, p2_try, p3) -
                         target_polygon_m) : INFINITY;
                bool better = isfinite(target_polygon_m) ?
                    (peak_curvature <= peak_curvature_limit &&
                     target_error < best_target_error) :
                    isfinite(peak_curvature_limit) ?
                    (peak_curvature <= peak_curvature_limit &&
                     control_polygon < best_control_polygon) :
                    (smoother ||
                     (similarly_smooth && control_polygon < best_control_polygon));
                if (better) {
                    best_peak_curvature = peak_curvature;
                    best_control_polygon = control_polygon;
                    best_target_error = target_error;
                    p1 = p1_try;
                    p2 = p2_try;
                    lead_feasible = true;
                }
            }
            final_handle *= 1.18;
        }
        initial_handle *= 1.18;
    }
    if (!lead_feasible) {
        if (reason && reason_size)
            snprintf(reason, reason_size, "finite lead cannot satisfy live initial curvature and HAC join geometry");
        memset(route, 0, sizeof(*route));
        return false;
    }
    route->p0_along_m = p0.x; route->p0_cross_m = p0.y;
    route->p1_along_m = p1.x; route->p1_cross_m = p1.y;
    route->p2_along_m = p2.x; route->p2_cross_m = p2.y;
    route->p3_along_m = p3.x; route->p3_cross_m = p3.y;
    route->initial_course_deg = start->course_deg;
    route->runway_heading_deg = m->site.runway_heading;
    if (!cubic_arc_lookup(route)) {
        if (reason && reason_size)
            snprintf(reason, reason_size, "finite lead arc-length lookup failed");
        return false;
    }
    size_t lead_count = (size_t)fmax(2.0,
        ceil(route->lead_length_m / spacing));
    size_t arc_count = (size_t)fmax(3.0,
        ceil(route->hac.arc_length_m / spacing));
    route->alignment_along_m = -m->guidance.final_approach_distance;
    route->rollout_length_m = route->alignment_along_m - route->hac.exit.x;
    if (!(route->rollout_length_m >= 0.0) || !isfinite(route->rollout_length_m))
        goto capacity_failure;
    size_t rollout_count = route->rollout_length_m > 1.0 ?
        (size_t)fmax(1.0,ceil(route->rollout_length_m / spacing)) : 0;
    if (lead_count + arc_count + rollout_count + 1 > TAEM_ROUTE_MAX_POINTS)
        goto capacity_failure;
    route->lead_count = lead_count;
    route->arc_count = arc_count;
    route->rollout_count = rollout_count;
    route->count = lead_count + arc_count + rollout_count + 1;

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
        const TaemGeometryState *start, const TaemRoute *route_a,
        const TaemRoute *route_b, double spacing,
        double maximum_lead_curvature, double peak_curvature_limit,
        double target_lead_length_m, TaemRoute *route,
        char *reason, size_t reason_size) {
    if (reason && reason_size) reason[0] = '\0';
    if (!m || !start || !route_a || !route_b || !route ||
        !route_a->valid || !route_b->valid ||
        !(spacing >= 100.0) || !isfinite(spacing) ||
        !(maximum_lead_curvature > 0.0) || !isfinite(maximum_lead_curvature) ||
        !(peak_curvature_limit > 0.0) || !isfinite(peak_curvature_limit) ||
        !(target_lead_length_m > 0.0) || !isfinite(target_lead_length_m)) {
        if (reason && reason_size) snprintf(reason, reason_size, "invalid route input");
        return false;
    }

    const TaemRoute *short_route = route_a;
    const TaemRoute *long_route = route_b;
    if (short_route->lead_length_m > long_route->lead_length_m) {
        short_route = route_b;
        long_route = route_a;
    }
    if (fabs(short_route->hac.radius_m - long_route->hac.radius_m) > 1e-6 ||
        fabs(short_route->hac.arc_sweep_rad - long_route->hac.arc_sweep_rad) > 1e-9 ||
        short_route->side != long_route->side ||
        fabs(short_route->p0_along_m - long_route->p0_along_m) > 1e-6 ||
        fabs(short_route->p0_cross_m - long_route->p0_cross_m) > 1e-6 ||
        fabs(short_route->p3_along_m - long_route->p3_along_m) > 1e-6 ||
        fabs(short_route->p3_cross_m - long_route->p3_cross_m) > 1e-6 ||
        target_lead_length_m < short_route->lead_length_m - 1.0 ||
        target_lead_length_m > long_route->lead_length_m + 1.0) {
        if (reason && reason_size) snprintf(reason, reason_size, "routes do not bracket one HAC lead family");
        return false;
    }

    Point p0 = {short_route->p0_along_m, short_route->p0_cross_m};
    Point p3 = {short_route->p3_along_m, short_route->p3_cross_m};
    Point short_p1 = {short_route->p1_along_m, short_route->p1_cross_m};
    Point short_p2 = {short_route->p2_along_m, short_route->p2_cross_m};
    Point long_p1 = {long_route->p1_along_m, long_route->p1_cross_m};
    Point long_p2 = {long_route->p2_along_m, long_route->p2_cross_m};
    Point best_p1 = {0.0, 0.0}, best_p2 = {0.0, 0.0};
    double best_error = INFINITY;
    bool found = false;

    for (int i = 0; i <= 32; ++i) {
        double t1 = (double)i / 32.0;
        Point p1 = {short_p1.x + t1 * (long_p1.x - short_p1.x),
                    short_p1.y + t1 * (long_p1.y - short_p1.y)};
        for (int j = 0; j <= 32; ++j) {
            double t2 = (double)j / 32.0;
            Point p2 = {short_p2.x + t2 * (long_p2.x - short_p2.x),
                        short_p2.y + t2 * (long_p2.y - short_p2.y)};
            if (!cubic_forward_regular(p0, p1, p2, p3)) continue;
            double initial_curvature = cubic_endpoint_curvature(p0, p1, p2, p3, 0.0);
            double peak_curvature = cubic_peak_curvature(p0, p1, p2, p3);
            if (!isfinite(initial_curvature) || !isfinite(peak_curvature) ||
                initial_curvature > maximum_lead_curvature * 1.000001 ||
                peak_curvature > peak_curvature_limit * 1.000001)
                continue;
            double lead_length = cubic_length_estimate(p0, p1, p2, p3);
            double error = fabs(lead_length - target_lead_length_m);
            if (error < best_error) {
                best_error = error;
                best_p1 = p1;
                best_p2 = p2;
                found = true;
            }
        }
    }
    if (!found) {
        if (reason && reason_size) snprintf(reason, reason_size, "no curvature-feasible route between profile brackets");
        return false;
    }

    TaemRoute candidate = {0};
    candidate.side = short_route->side;
    candidate.hac = short_route->hac;
    candidate.p0_along_m = p0.x;
    candidate.p0_cross_m = p0.y;
    candidate.p1_along_m = best_p1.x;
    candidate.p1_cross_m = best_p1.y;
    candidate.p2_along_m = best_p2.x;
    candidate.p2_cross_m = best_p2.y;
    candidate.p3_along_m = p3.x;
    candidate.p3_cross_m = p3.y;
    candidate.initial_course_deg = start->course_deg;
    candidate.runway_heading_deg = m->site.runway_heading;
    if (!cubic_arc_lookup(&candidate)) {
        if (reason && reason_size) snprintf(reason, reason_size, "interpolated lead arc-length lookup failed");
        return false;
    }

    candidate.alignment_along_m = -m->guidance.final_approach_distance;
    candidate.rollout_length_m = candidate.alignment_along_m - candidate.hac.exit.x;
    if (!(candidate.rollout_length_m >= 0.0) || !isfinite(candidate.rollout_length_m)) {
        if (reason && reason_size) snprintf(reason, reason_size, "interpolated route has invalid rollout");
        return false;
    }
    candidate.lead_count = (size_t)fmax(2.0, ceil(candidate.lead_length_m / spacing));
    candidate.arc_count = (size_t)fmax(3.0, ceil(candidate.hac.arc_length_m / spacing));
    candidate.rollout_count = candidate.rollout_length_m > 1.0 ?
        (size_t)fmax(1.0, ceil(candidate.rollout_length_m / spacing)) : 0;
    if (candidate.lead_count + candidate.arc_count + candidate.rollout_count + 1 >
        TAEM_ROUTE_MAX_POINTS) {
        if (reason && reason_size) snprintf(reason, reason_size, "interpolated HAC route exceeded point capacity");
        return false;
    }
    candidate.count = candidate.lead_count + candidate.arc_count +
        candidate.rollout_count + 1;
    if (!route_finish(m, start, &candidate)) {
        if (reason && reason_size) snprintf(reason, reason_size, "interpolated route sampling produced invalid geometry");
        return false;
    }
    *route = candidate;
    return true;
}

double taem_route_lead_peak_curvature(const TaemRoute *route) {
    if (!route || !route->valid) return INFINITY;
    return cubic_peak_curvature(
        (Point){route->p0_along_m, route->p0_cross_m},
        (Point){route->p1_along_m, route->p1_cross_m},
        (Point){route->p2_along_m, route->p2_cross_m},
        (Point){route->p3_along_m, route->p3_cross_m});
}

bool taem_route_build_fixed_hac(const TerminalModel *m,
        const TaemGeometryState *start, double hac_radius_m, double side,
        double spacing, double maximum_lead_curvature,
        TaemRoute *route, char *reason, size_t reason_size) {
    return taem_route_build_hac(m, start, hac_radius_m, side,
        1.5 * planner_pi, spacing, maximum_lead_curvature,
        route, reason, reason_size);
}

static bool route_sample_at_station(const TaemRoute *route, double station,
        Point *position, double *course_deg, double *curvature_right_per_m) {
    if (!route || !route->valid || !position || !course_deg ||
        !curvature_right_per_m || !(route->length_m > 0.0)) return false;

    station = clamp(station, 0.0, route->length_m);
    if (station <= route->lead_length_m) {
        Point p0 = {route->p0_along_m, route->p0_cross_m};
        Point p1 = {route->p1_along_m, route->p1_cross_m};
        Point p2 = {route->p2_along_m, route->p2_cross_m};
        Point p3 = {route->p3_along_m, route->p3_cross_m};
        double fraction = route->lead_length_m > 0.0 ?
            station / route->lead_length_m : 0.0;
        double t = cubic_t_at_arc_fraction(route, fraction);
        Point d = cubic_derivative(p0, p1, p2, p3, t);
        Point dd = cubic_second_derivative(p0, p1, p2, p3, t);
        double tangent_norm = hypot(d.x, d.y);
        if (!(tangent_norm > 1e-9)) return false;
        *position = cubic(p0, p1, p2, p3, t);
        *course_deg = route->runway_heading_deg + degrees(atan2(d.y, d.x));
        *curvature_right_per_m =
            (d.x * dd.y - d.y * dd.x) /
            (tangent_norm * tangent_norm * tangent_norm);
        return isfinite(*course_deg) && isfinite(*curvature_right_per_m);
    }

    double arc_end_station=route->lead_length_m+route->hac.arc_length_m;
    if (station <= arc_end_station) {
        if (!(route->hac.arc_length_m > 0.0) || !(route->hac.radius_m > 0.0))
            return false;
        double arc_station = station - route->lead_length_m;
        double fraction = clamp(arc_station / route->hac.arc_length_m, 0.0, 1.0);
        double sweep_sign = route->hac.arc_sweep_rad >= 0.0 ? 1.0 : -1.0;
        double end_angle = atan2(route->hac.exit.y - route->hac.center.y,
                                 route->hac.exit.x - route->hac.center.x);
        double start_angle = end_angle - route->hac.arc_sweep_rad;
        double angle = start_angle + route->hac.arc_sweep_rad * fraction;
        *position = (Point){
            route->hac.center.x + route->hac.radius_m * cos(angle),
            route->hac.center.y + route->hac.radius_m * sin(angle)
        };
        double tangent_x = -sin(angle) * sweep_sign;
        double tangent_y = cos(angle) * sweep_sign;
        *course_deg = route->runway_heading_deg + degrees(atan2(tangent_y, tangent_x));
        *curvature_right_per_m = sweep_sign / route->hac.radius_m;
        return isfinite(*course_deg) && isfinite(*curvature_right_per_m);
    }

    double rollout_station=station-arc_end_station;
    double f=route->rollout_length_m>0.0?
        clamp(rollout_station/route->rollout_length_m,0.0,1.0):1.0;
    *position=(Point){
        route->hac.exit.x+f*(route->alignment_along_m-route->hac.exit.x),
        0.0
    };
    *course_deg=route->runway_heading_deg;
    *curvature_right_per_m=0.0;
    return true;
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
    double gamma = radians(geometry->flight_path_angle_deg);

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

    double radial_required = (gravity -
        geometry->airspeed_mps * geometry->airspeed_mps / local_radius) *
        cos(gamma);
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
