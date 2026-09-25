#ifndef CLANDING_TAEM_ROUTE_H
#define CLANDING_TAEM_ROUTE_H

#include <stdbool.h>
#include <stddef.h>
#include <math.h>

typedef struct { double x, y; } TaemPoint2;

typedef struct {
    TaemPoint2 entry;
    TaemPoint2 center;
    TaemPoint2 exit;
    double radius_m;
    double side;
    double capture_course_deg;
    double arc_sweep_rad;
    double arc_length_m;
    double final_length_m;
    double total_length_m;
} TaemFixedHacGeometry;

#define TAEM_ROUTE_MAX_POINTS 2048
#define TAEM_ROUTE_LEAD_LUT_POINTS 129
#define TAEM_ROUTE_PROFILE_LUT_POINTS 257

typedef struct {
    double along_m;
    double cross_m;
    double course_deg;
    double curvature_right_per_m;
    double altitude_m;
    double flight_path_angle_deg;
    double distance_from_start_m;
} TaemRoutePoint;

typedef struct {
    bool valid;
    size_t count;
    size_t lead_count;
    size_t arc_count;
    double length_m;
    double side;
    TaemFixedHacGeometry hac;
    double p0_along_m, p0_cross_m;
    double p1_along_m, p1_cross_m;
    double p2_along_m, p2_cross_m;
    double p3_along_m, p3_cross_m;
    double lead_length_m;
    float lead_arc_fraction_lut[TAEM_ROUTE_LEAD_LUT_POINTS];
    double initial_course_deg, runway_heading_deg;
    double profile_start_altitude_m, profile_start_fpa_deg;
    double profile_final_altitude_m, profile_final_slope_deg;
    double profile_total_length_m;
    /* Mid-route altitude bow; zero at both endpoints with zero endpoint slope. */
    double profile_midpoint_offset_m;
    /* Localized interior bow for independently shaping the lead descent. */
    double profile_local_offset_m;
    double profile_local_start_fraction, profile_local_peak_fraction;
    double profile_local_end_fraction;
    /* Localized initial descent sag; also returns to zero with zero slope. */
    double profile_initial_sag_m;
    double profile_initial_sag_length_m;
    /* Dynamically generated vertical reference: altitude (MSL) and FPA at
     * uniform stations over [0, profile_total_length_m], produced by native
     * propagation of the vehicle along this route.  When set it replaces the
     * analytic profile above. */
    bool profile_tabulated;
    double profile_generation_aoa_deg;
    float profile_altitude_lut[TAEM_ROUTE_PROFILE_LUT_POINTS];
    float profile_fpa_lut[TAEM_ROUTE_PROFILE_LUT_POINTS];
} TaemRoute;


static inline double taem_route_station_at_index(const TaemRoute *route, size_t index) {
    if (!route || !route->valid || route->count < 2 ||
        route->lead_count == 0 || route->arc_count == 0)
        return NAN;
    if (index >= route->count) index = route->count - 1;
    if (index <= route->lead_count)
        return route->lead_length_m * (double)index / (double)route->lead_count;
    return route->lead_length_m + route->hac.arc_length_m *
        (double)(index - route->lead_count) / (double)route->arc_count;
}

static inline double taem_route_remaining_at_index(const TaemRoute *route, size_t index) {
    double station = taem_route_station_at_index(route, index);
    if (!isfinite(station) || !isfinite(route->length_m)) return NAN;
    return fmax(0.0, route->length_m - station);
}
#endif
