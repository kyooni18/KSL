#include "taem_frames_energy.h"

#include <math.h>
#include <stddef.h>

#define TAEM_PI 3.14159265358979323846264338327950288
#define TAEM_EPS 1e-12

static bool finite3(TaemVec3 v) {
    return isfinite(v.x) && isfinite(v.y) && isfinite(v.z);
}

static bool valid_world(const TaemFrameWorld *w) {
    return w && isfinite(w->radius_m) && w->radius_m > 0.0 &&
           isfinite(w->mu_m3_s2) && w->mu_m3_s2 > 0.0 &&
           finite3(w->rotation_axis_i) && taem_vec3_norm(w->rotation_axis_i) > TAEM_EPS &&
           isfinite(w->rotation_rate_rad_s) && isfinite(w->rotation_phase_rad_at_ut0);
}

static TaemVec3 unit(TaemVec3 a) {
    double n = taem_vec3_norm(a);
    return n > TAEM_EPS ? taem_vec3_scale(a, 1.0 / n) : (TaemVec3){0.0, 0.0, 0.0};
}

static TaemVec3 rotate_axis(TaemVec3 v, TaemVec3 axis, double angle) {
    axis = unit(axis);
    double c = cos(angle), s = sin(angle);
    return taem_vec3_add(taem_vec3_add(taem_vec3_scale(v, c),
                                       taem_vec3_scale(taem_vec3_cross(axis, v), s)),
                         taem_vec3_scale(axis, taem_vec3_dot(axis, v) * (1.0 - c)));
}

static TaemVec3 omega_cross(const TaemFrameWorld *w, TaemVec3 v) {
    TaemVec3 omega = taem_vec3_scale(unit(w->rotation_axis_i), w->rotation_rate_rad_s);
    return taem_vec3_cross(omega, v);
}

static bool valid_runway_frame(const TaemFrameWorld *w, const TaemRunwayFrame *f) {
    if (!valid_world(w) || !f || !finite3(f->threshold_unit_b) || !finite3(f->forward_b) ||
        !finite3(f->right_b) || !isfinite(f->elevation_m) || !isfinite(f->length_m) ||
        !isfinite(f->width_m) || f->length_m <= 0.0 || f->width_m <= 0.0 ||
        w->radius_m + f->elevation_m <= 0.0) return false;
    double u=taem_vec3_norm(f->threshold_unit_b), a=taem_vec3_norm(f->forward_b), b=taem_vec3_norm(f->right_b);
    return fabs(u-1.0)<1e-7 && fabs(a-1.0)<1e-7 && fabs(b-1.0)<1e-7 &&
           fabs(taem_vec3_dot(f->threshold_unit_b,f->forward_b))<1e-7 &&
           fabs(taem_vec3_dot(f->threshold_unit_b,f->right_b))<1e-7 &&
           fabs(taem_vec3_dot(f->forward_b,f->right_b))<1e-7 &&
           taem_vec3_dot(taem_vec3_cross(f->forward_b,f->right_b),f->threshold_unit_b)<-0.999999;
}

TaemVec3 taem_vec3_add(TaemVec3 a, TaemVec3 b) { return (TaemVec3){a.x+b.x,a.y+b.y,a.z+b.z}; }
TaemVec3 taem_vec3_sub(TaemVec3 a, TaemVec3 b) { return (TaemVec3){a.x-b.x,a.y-b.y,a.z-b.z}; }
TaemVec3 taem_vec3_scale(TaemVec3 a, double s) { return (TaemVec3){a.x*s,a.y*s,a.z*s}; }
double taem_vec3_dot(TaemVec3 a, TaemVec3 b) { return a.x*b.x+a.y*b.y+a.z*b.z; }
TaemVec3 taem_vec3_cross(TaemVec3 a, TaemVec3 b) {
    return (TaemVec3){a.y*b.z-a.z*b.y,a.z*b.x-a.x*b.z,a.x*b.y-a.y*b.x};
}
double taem_vec3_norm(TaemVec3 a) { return hypot(hypot(a.x,a.y),a.z); }
double taem_wrap_pi(double a) {
    if (!isfinite(a)) return NAN;
    a = remainder(a, 2.0 * TAEM_PI);
    return a <= -TAEM_PI ? a + 2.0 * TAEM_PI : a;
}

bool taem_inertial_to_fixed(const TaemFrameWorld *w, TaemVec3 p_i, TaemVec3 v_i,
                            double ut, TaemVec3 *p_b, TaemVec3 *v_b) {
    if (!valid_world(w) || !finite3(p_i) || !finite3(v_i) || !isfinite(ut) || !p_b || !v_b) return false;
    double angle = w->rotation_phase_rad_at_ut0 + w->rotation_rate_rad_s * ut;
    *p_b = rotate_axis(p_i, w->rotation_axis_i, -angle);
    *v_b = rotate_axis(taem_vec3_sub(v_i, omega_cross(w, p_i)), w->rotation_axis_i, -angle);
    return finite3(*p_b) && finite3(*v_b);
}

bool taem_fixed_to_inertial(const TaemFrameWorld *w, TaemVec3 p_b, TaemVec3 v_b,
                            double ut, TaemVec3 *p_i, TaemVec3 *v_i) {
    if (!valid_world(w) || !finite3(p_b) || !finite3(v_b) || !isfinite(ut) || !p_i || !v_i) return false;
    double angle = w->rotation_phase_rad_at_ut0 + w->rotation_rate_rad_s * ut;
    *p_i = rotate_axis(p_b, w->rotation_axis_i, angle);
    *v_i = rotate_axis(taem_vec3_add(v_b, omega_cross(w, p_b)), w->rotation_axis_i, angle);
    return finite3(*p_i) && finite3(*v_i);
}

bool taem_local_north_east_up(TaemVec3 p, TaemVec3 *north, TaemVec3 *east, TaemVec3 *up) {
    if (!finite3(p) || !north || !east || !up || taem_vec3_norm(p) <= TAEM_EPS) return false;
    *up = unit(p);
    TaemVec3 z = {0.0,0.0,1.0};
    *east = unit(taem_vec3_cross(z, *up));
    if (taem_vec3_norm(*east) <= TAEM_EPS) *east = (TaemVec3){0.0,1.0,0.0};
    *north = unit(taem_vec3_cross(*up, *east));
    return true;
}

bool taem_runway_frame_create(const TaemFrameWorld *w, const TaemRunway *r, TaemRunwayFrame *f) {
    if (!valid_world(w) || !r || !f || !isfinite(r->latitude_rad) || !isfinite(r->longitude_rad) ||
        !isfinite(r->elevation_m) || !isfinite(r->heading_rad) || !isfinite(r->length_m) ||
        !isfinite(r->width_m) || fabs(r->latitude_rad) > TAEM_PI/2.0 ||
        r->length_m <= 0.0 || r->width_m <= 0.0 || w->radius_m+r->elevation_m <= 0.0) return false;
    double cl = cos(r->latitude_rad), sl = sin(r->latitude_rad);
    double co = cos(r->longitude_rad), so = sin(r->longitude_rad);
    TaemVec3 up = {cl*co,cl*so,sl};
    TaemVec3 north = {-sl*co,-sl*so,cl};
    TaemVec3 east = {-so,co,0.0};
    double ch=cos(r->heading_rad), sh=sin(r->heading_rad);
    f->threshold_unit_b = up;
    f->forward_b = taem_vec3_add(taem_vec3_scale(north,ch),taem_vec3_scale(east,sh));
    f->right_b = taem_vec3_sub(taem_vec3_scale(east,ch),taem_vec3_scale(north,sh));
    f->elevation_m=r->elevation_m; f->length_m=r->length_m; f->width_m=r->width_m;
    return true;
}

/* Exponential map on a spherical planet, with a stable zero-distance limit. */
static TaemVec3 sphere_exp(TaemVec3 origin, TaemVec3 tangent, double angle) {
    return unit(taem_vec3_add(taem_vec3_scale(origin, cos(angle)), taem_vec3_scale(tangent, sin(angle))));
}

static bool sphere_log(TaemVec3 origin, TaemVec3 target, TaemVec3 *tangent, double *angle) {
    double c = fmax(-1.0, fmin(1.0, taem_vec3_dot(origin,target)));
    double s = taem_vec3_norm(taem_vec3_cross(origin,target));
    double a = atan2(s,c);
    if (a >= TAEM_PI-1e-9) return false; /* antipodal log-map is not unique */
    *angle = a;
    *tangent = unit(taem_vec3_sub(target,taem_vec3_scale(origin,c)));
    return a < 1e-10 || s > TAEM_EPS;
}

bool taem_runway_frame_reciprocal(const TaemFrameWorld *w, const TaemRunwayFrame *s, TaemRunwayFrame *r) {
    if (!valid_runway_frame(w,s) || !r) return false;
    TaemVec3 u=unit(s->threshold_unit_b), f=unit(s->forward_b), right=unit(s->right_b);
    TaemVec3 end=sphere_exp(u,f,s->length_m/(w->radius_m+s->elevation_m));
    /* Parallel-transport the runway direction along its great-circle centerline. */
    TaemVec3 axis=unit(taem_vec3_cross(u,f));
    TaemVec3 end_forward=rotate_axis(f,axis,s->length_m/(w->radius_m+s->elevation_m));
    r->threshold_unit_b=end;
    r->forward_b=taem_vec3_scale(end_forward,-1.0);
    r->right_b=taem_vec3_scale(rotate_axis(right,axis,s->length_m/(w->radius_m+s->elevation_m)),-1.0);
    r->elevation_m=s->elevation_m; r->length_m=s->length_m; r->width_m=s->width_m;
    return finite3(r->threshold_unit_b) && finite3(r->forward_b) && finite3(r->right_b);
}

bool taem_runway_project(const TaemFrameWorld *w, const TaemRunwayFrame *f, TaemVec3 p,
                         TaemRunwayCoordinates *c) {
    if (!valid_runway_frame(w,f) || !c || !finite3(p) || taem_vec3_norm(p)<=TAEM_EPS) return false;
    double radius=taem_vec3_norm(p); TaemVec3 u=taem_vec3_scale(p,1.0/radius), tangent;
    double angle;
    if (!sphere_log(unit(f->threshold_unit_b),u,&tangent,&angle)) return false;
    double base=w->radius_m+f->elevation_m;
    if (base <= 0.0) return false;
    TaemVec3 logv=taem_vec3_scale(tangent,angle*base);
    c->position_b_m=p; c->radius_from_center_m=radius; c->altitude_above_datum_m=radius-w->radius_m;
    c->altitude_above_runway_m=radius-base;
    c->runway_along_m=taem_vec3_dot(logv,unit(f->forward_b));
    c->runway_cross_m=taem_vec3_dot(logv,unit(f->right_b));
    c->runway_vertical_m=c->altitude_above_runway_m;
    return true;
}

bool taem_runway_unproject(const TaemFrameWorld *w, const TaemRunwayFrame *f, double along, double cross,
                           double above, TaemVec3 *p) {
    if (!valid_runway_frame(w,f) || !p || !isfinite(along) || !isfinite(cross) || !isfinite(above)) return false;
    double base=w->radius_m+f->elevation_m, radius=base+above;
    if (base<=0.0 || radius<=0.0) return false;
    TaemVec3 u=unit(f->threshold_unit_b), e=unit(taem_vec3_add(taem_vec3_scale(f->forward_b,along),taem_vec3_scale(f->right_b,cross)));
    double distance=hypot(along,cross), angle=distance/base;
    *p=taem_vec3_scale(sphere_exp(u,e,angle),radius);
    return finite3(*p);
}

bool taem_gravity_diagnostic(const TaemFrameWorld *w, TaemVec3 p, TaemGravityDiagnostic *d) {
    if (!valid_world(w) || !finite3(p) || !d) return false;
    double r=taem_vec3_norm(p);
    if (r<=TAEM_EPS) return false;
    TaemVec3 omegaxr=omega_cross(w,p);
    d->gravity_mps2=w->mu_m3_s2/(r*r);
    d->surface_speed_mps=taem_vec3_norm(omegaxr);
    d->centrifugal_accel_mps2=taem_vec3_norm(omega_cross(w,omegaxr));
    return isfinite(d->gravity_mps2) && isfinite(d->surface_speed_mps) && isfinite(d->centrifugal_accel_mps2);
}

bool taem_energy_diagnostic(const TaemFrameWorld *w, TaemVec3 p_i, TaemVec3 v_i, TaemVec3 v_b,
                            TaemVec3 air_b, TaemVec3 force_i, TaemVec3 force_b, double mass,
                            double drag, TaemEnergyDiagnostic *d) {
    if (!valid_world(w) || !finite3(p_i) || !finite3(v_i) || !finite3(v_b) || !finite3(air_b) ||
        !finite3(force_i) || !finite3(force_b) || !d || !isfinite(mass) || mass<=0.0 || !isfinite(drag) || drag<0.0) return false;
    double r=taem_vec3_norm(p_i);
    if (r<=TAEM_EPS) return false;
    /* The centrifugal potential magnitude is rotation invariant about the axis. */
    double grav=-w->mu_m3_s2/r;
    d->gravitational_potential_j_kg=grav;
    d->inertial_specific_energy_j_kg=0.5*taem_vec3_dot(v_i,v_i)+grav;
    TaemVec3 omega_cross_position=omega_cross(w,p_i);
    d->effective_specific_energy_j_kg=0.5*taem_vec3_dot(v_b,v_b)+grav-0.5*taem_vec3_dot(omega_cross_position,omega_cross_position);
    d->inertial_energy_rate_w_kg=taem_vec3_dot(force_i,v_i)/mass;
    d->effective_energy_rate_w_kg=taem_vec3_dot(force_b,v_b)/mass;
    d->airspeed_mps=taem_vec3_norm(air_b);
    d->drag_work_rate_w_kg=-drag*d->airspeed_mps/mass;
    return isfinite(d->gravitational_potential_j_kg) && isfinite(d->inertial_specific_energy_j_kg) &&
           isfinite(d->effective_specific_energy_j_kg) && isfinite(d->inertial_energy_rate_w_kg) &&
           isfinite(d->effective_energy_rate_w_kg) && isfinite(d->airspeed_mps) && isfinite(d->drag_work_rate_w_kg);
}

bool taem_no_wind_energy_work_residual(const TaemEnergyDiagnostic *d, double *residual) {
    if (!d || !residual || !isfinite(d->effective_energy_rate_w_kg) || !isfinite(d->drag_work_rate_w_kg)) return false;
    *residual=d->effective_energy_rate_w_kg-d->drag_work_rate_w_kg;
    return isfinite(*residual);
}

bool taem_ground_arc_work_rate(double drag, double mass, double speed, double gamma,
                               double radius, double reference_radius, double *work_rate) {
    if (!work_rate || !isfinite(drag) || drag<0.0 || !isfinite(mass) || mass<=0.0 ||
        !isfinite(speed) || speed<0.0 || !isfinite(gamma) ||
        !isfinite(radius) || radius<=0.0 || !isfinite(reference_radius) || reference_radius<=0.0) return false;
    /* (dE/ds_g)(ds_g/dt); the spherical and flight-path factors cancel. */
    *work_rate=-(drag/mass)*speed;
    return isfinite(*work_rate);
}
