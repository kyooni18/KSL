#ifndef CLANDING_TAEM_TRACKER_H
#define CLANDING_TAEM_TRACKER_H

#include <stdbool.h>

#include "taem_reachability.h"
#include "terminal_propagator.h"

typedef struct {
    double runway_along_m;
    double runway_cross_m;
    double course_deg;
    double curvature_right_per_m;
    double altitude_m;
    double flight_path_angle_deg;
    /* d(flight-path angle)/d(horizontal route station), radians per metre. */
    double vertical_curvature_per_m;
    /* Continuous projection onto the owned route, not accumulated flight distance. */
    double station_m;
} TaemPathReference;

typedef struct {
    bool valid;
    bool lateral_authority_ok;
    bool vertical_authority_ok;
    TerminalControl control;
    double cross_track_error_m;
    double course_error_deg;
    double required_lateral_accel_mps2;
    double available_lateral_accel_mps2;
    double required_vertical_lift_mps2;
    double delivered_vertical_lift_mps2;
    double response_time_s;
} TaemTrackerOutput;

/* Bounded Frenet tracker. The caller owns path planning and full-horizon
 * candidate qualification; this function only supplies the next control. */
TaemTrackerOutput taem_tracker_update(const TerminalModel *model,
        const TerminalDynamicState *state, const TaemGeometryState *geometry,
        const TaemPathReference *reference, double dt_s);

/* The lateral acceleration the tracker demands at this state/reference
 * (curvature feed-forward plus heading and cross-track feedback). */
double taem_tracker_lateral_demand(const TerminalModel *model,
        const TaemGeometryState *geometry, const TaemPathReference *reference);

#endif
