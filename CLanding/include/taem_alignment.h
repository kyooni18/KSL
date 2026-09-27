#ifndef CLANDING_TAEM_ALIGNMENT_H
#define CLANDING_TAEM_ALIGNMENT_H

#include <math.h>

#include "taem_tracker.h"

static inline double taem_alignment_target_height(
        const TerminalModel *model) {
    return model->guidance.final_approach_distance *
        tan(model->guidance.final_glide_slope / 57.29577951308232);
}

static inline double taem_alignment_target_altitude(
        const TerminalModel *model) {
    return model->site.altitude + taem_alignment_target_height(model);
}

/* Begin exact alignment-point capture far enough upstream for a real
 * unpowered shuttle to settle bank/rate and trim residual energy.  This is not
 * the Final handoff gate; it only changes which MM305 reference law is flown. */
static inline double taem_alignment_capture_distance(
        const TerminalModel *model, const TaemGeometryState *geometry) {
    (void)model;
    double speed = geometry ? fmax(0.0, geometry->ground_speed_mps) : 0.0;
    double response_distance = 15.0 * speed;
    return fmax(2000.0, fmin(3500.0, response_distance));
}

/* MM305 owns delivery to the Final alignment station.  The endpoint has
 * two simultaneous constraints: centerline position and runway-tangent course.
 * Use a quadratic terminal intercept: y(s) -> 0 as s^2, so its tangent also
 * tends to runway heading as the station is approached.  This cannot command a
 * 180-degree reversal if the station is crossed; the miss gate handles that. */
static inline TaemPathReference taem_alignment_reference(
        const TerminalModel *model, const TaemGeometryState *geometry) {
    const double radians_per_degree=0.017453292519943295;
    double station=-model->guidance.final_approach_distance;
    double remaining=station-geometry->runway_along_m;
    /* Inside the last couple of kilometres, a pure endpoint intercept turns a
       small residual cross-track error into a very large course command.  That
       was observed in replay as a settled ~14 deg bank being driven back toward
       ~40 deg just before the alignment point.  Keep a terminal lookahead floor:
       if the route has not converged by then, the strict alignment gate rejects
       it instead of asking the real shuttle for a last-second turn. */
    double effective_remaining=fmax(2000.0,remaining);

    /* For y(s)=y0*(s/s0)^2, dy/dx at the current state is 2*y/s.
       runway_cross is positive to the right, so the corrective course is
       negative when the vehicle is right of centerline. */
    double course_offset=atan2(-2.0*geometry->runway_cross_m,
        effective_remaining)/radians_per_degree;
    course_offset=fmax(-20.0,fmin(20.0,course_offset));

    /* The alignment station is a position + tangent + glide-state endpoint.
       Track the Final glide line itself, not a point-height intercept.  The old
       atan2(height_error, remaining) law collapsed toward -2 deg near the station,
       causing a pull-up, speed loss and an otherwise good replay to miss Final. */
    double final_slope=model->guidance.final_glide_slope;
    double target_height=taem_alignment_target_height(model);
    double upstream=fmax(0.0,remaining);
    double glide_height=target_height+
        upstream*tan(final_slope*radians_per_degree);
    double height_error=glide_height-geometry->altitude_above_runway_m;
    double correction=atan2(height_error,effective_remaining)/radians_per_degree;
    correction=fmax(-6.0,fmin(6.0,correction));
    double fpa=-final_slope+correction;

    return (TaemPathReference){
        /* Anchor lateral error at the current along-track station; the desired
           cross-track is zero while course supplies the terminal intercept. */
        .runway_along_m=geometry->runway_along_m,
        .runway_cross_m=0.0,
        .course_deg=model->site.runway_heading+course_offset,
        .curvature_right_per_m=0.0,
        .altitude_m=NAN,
        .flight_path_angle_deg=fpa,
        .vertical_curvature_per_m=0.0,
        .station_m=0.0
    };
}

static inline bool taem_alignment_ready(const TerminalModel *model,
        const TerminalDynamicState *state, const TaemGeometryState *geometry) {
    const double degrees_per_radian=57.29577951308232;
    double station=-model->guidance.final_approach_distance;
    double target_height=taem_alignment_target_height(model);
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

static inline bool taem_alignment_exhausted(const TerminalModel *model,
        const TaemGeometryState *geometry) {
    double station=-model->guidance.final_approach_distance;
    return geometry->runway_along_m>station+500.0 ||
        geometry->altitude_above_runway_m<300.0;
}

#endif
