#include "taem_geometry.h"

#include <math.h>

static const double taem_geometry_pi = 3.14159265358979323846264338327950288;

static double degrees(double radians) { return radians * 180.0 / taem_geometry_pi; }

static double wrap_degrees(double degrees_in) {
    return remainder(degrees_in, 360.0);
}

bool taem_geometry_world(const TerminalModel *m, TaemFrameWorld *w) {
    if (!m || !w || !(m->world.radius_m > 0.0) || !(m->world.mu_m3_s2 > 0.0))
        return false;
    *w = (TaemFrameWorld){
        .radius_m = m->world.radius_m,
        .mu_m3_s2 = m->world.mu_m3_s2,
        .rotation_axis_i = {0.0, 0.0, 1.0},
        .rotation_rate_rad_s = m->world.rotation_rate_rad_s,
        .rotation_phase_rad_at_ut0 = m->world.rotation_phase_rad_at_ut0
    };
    return true;
}

bool taem_geometry_runway(const TerminalModel *m, TaemFrameWorld *w,
                          TaemRunwayFrame *f) {
    if (!m || !w || !f || !taem_geometry_world(m, w)) return false;
    TaemRunway runway = {
        .latitude_rad = m->site.latitude * taem_geometry_pi / 180.0,
        .longitude_rad = m->site.longitude * taem_geometry_pi / 180.0,
        .elevation_m = m->site.altitude,
        .heading_rad = m->site.runway_heading * taem_geometry_pi / 180.0,
        .length_m = m->site.runway_length,
        .width_m = m->site.runway_width
    };
    return taem_runway_frame_create(w, &runway, f);
}

bool taem_geometry_state(const TerminalModel *m,
                         const TerminalDynamicState *s,
                         TaemGeometryState *out) {
    if (!m || !s || !out) return false;
    TaemFrameWorld w;
    TaemRunwayFrame runway;
    if (!taem_geometry_runway(m, &w, &runway)) return false;
    TaemVec3 pi = {s->position_i_m.x, s->position_i_m.y, s->position_i_m.z};
    TaemVec3 vi = {s->velocity_i_mps.x, s->velocity_i_mps.y, s->velocity_i_mps.z};
    TaemVec3 pb, vb;
    if (!taem_inertial_to_fixed(&w, pi, vi, s->ut_s, &pb, &vb)) return false;
    TaemRunwayCoordinates rc;
    if (!taem_runway_project(&w, &runway, pb, &rc)) return false;

    TaemVec3 north, east, up;
    if (!taem_local_north_east_up(pb, &north, &east, &up)) return false;
    double vn = taem_vec3_dot(vb, north);
    double ve = taem_vec3_dot(vb, east);
    double vu = taem_vec3_dot(vb, up);
    double horizontal = hypot(vn, ve);
    if (!(horizontal > 1e-6) || !isfinite(vu)) return false;
    double course = degrees(atan2(ve, vn));
    double airspeed = taem_vec3_norm(vb);
    if (!isfinite(airspeed)) return false;

    *out = (TaemGeometryState){
        .runway_along_m = rc.runway_along_m,
        .runway_cross_m = rc.runway_cross_m,
        .altitude_above_runway_m = rc.altitude_above_runway_m,
        .ground_speed_mps = horizontal,
        .airspeed_mps = airspeed,
        .course_deg = wrap_degrees(course),
        .flight_path_angle_deg = degrees(atan2(vu, horizontal)),
        .heading_error_deg = wrap_degrees(course - m->site.runway_heading)
    };
    return true;
}
