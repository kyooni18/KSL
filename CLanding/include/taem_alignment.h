#ifndef CLANDING_TAEM_ALIGNMENT_H
#define CLANDING_TAEM_ALIGNMENT_H

#include <math.h>

#include "taem_tracker.h"

/* The circular HAC is followed by a runway-line roll-out.  Keep the reference
 * and measured completion contract identical in planning replay and flight. */
static inline TaemPathReference taem_alignment_reference(
        const TerminalModel *model, const TaemGeometryState *geometry) {
    double slope=model->guidance.final_glide_slope;
    return (TaemPathReference){
        .runway_along_m=geometry->runway_along_m,
        .runway_cross_m=0.0,
        .course_deg=model->site.runway_heading,
        .curvature_right_per_m=0.0,
        .altitude_m=model->site.altitude+
            fmax(0.0,-geometry->runway_along_m)*tan(slope*0.017453292519943295),
        .flight_path_angle_deg=-slope,
        .vertical_curvature_per_m=0.0,
        .station_m=0.0
    };
}

static inline bool taem_alignment_ready(const TerminalModel *model,
        const TerminalDynamicState *state, const TaemGeometryState *geometry) {
    const double degrees_per_radian=57.29577951308232;
    double station=-model->guidance.final_approach_distance;
    double target_height=model->guidance.final_approach_distance*
        tan(model->guidance.final_glide_slope/degrees_per_radian);
    double target_speed=fmax(model->guidance.final_alignment_speed,
        model->vehicle.touchdown_speed);
    return fabs(geometry->runway_cross_m)<0.5*model->site.runway_width &&
        fabs(geometry->heading_error_deg)<2.0 &&
        fabs(state->attitude.bank_rad*degrees_per_radian)<5.0 &&
        fabs(state->attitude.bank_rate_rad_s*degrees_per_radian)<3.0 &&
        fabs(geometry->runway_along_m-station)<=250.0 &&
        fabs(geometry->altitude_above_runway_m-target_height)<=150.0 &&
        fabs(geometry->airspeed_mps-target_speed)<=10.0 &&
        fabs(geometry->flight_path_angle_deg+model->guidance.final_glide_slope)<=3.0;
}

static inline bool taem_alignment_exhausted(const TaemGeometryState *geometry) {
    return geometry->runway_along_m>-500.0 ||
        geometry->altitude_above_runway_m<300.0;
}

#endif
