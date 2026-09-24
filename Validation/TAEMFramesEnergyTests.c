#include "taem_frames_energy.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>

#define PI 3.14159265358979323846264338327950288
#define CHECK(x) do { if (!(x)) { fprintf(stderr, "FAIL %s:%d: %s\n", __FILE__, __LINE__, #x); return 1; } } while (0)

static int near(double a, double b, double tol) { return fabs(a-b) <= tol; }

static unsigned rng = 305u;
static double random_unit(void) {
    rng = 1664525u*rng + 1013904223u;
    return (double)(rng >> 8) / 16777216.0;
}

static TaemVec3 rotate_axis(TaemVec3 v, TaemVec3 axis, double angle) {
    double n=taem_vec3_norm(axis); axis=taem_vec3_scale(axis,1.0/n);
    return taem_vec3_add(taem_vec3_add(taem_vec3_scale(v,cos(angle)),
             taem_vec3_scale(taem_vec3_cross(axis,v),sin(angle))),
             taem_vec3_scale(axis,taem_vec3_dot(axis,v)*(1.0-cos(angle))));
}

static TaemVec3 unit(TaemVec3 v) { return taem_vec3_scale(v,1.0/taem_vec3_norm(v)); }

int main(void) {
    TaemFrameWorld w = {600000.0, 3.5316e12, {0.0,0.0,1.0}, 0.0002915709030370688, 0.37};
    TaemGravityDiagnostic gd;
    TaemVec3 equator = {w.radius_m,0.0,0.0};
    CHECK(taem_gravity_diagnostic(&w,equator,&gd));
    CHECK(near(gd.gravity_mps2,9.81,1e-12));
    CHECK(near(gd.surface_speed_mps,174.94254182224128,1e-10));
    CHECK(near(gd.centrifugal_accel_mps2,0.051008154898711064,1e-12));

    double g10=w.mu_m3_s2/610000.0/610000.0;
    double g20=w.mu_m3_s2/620000.0/620000.0;
    double dphi10=w.mu_m3_s2*(1.0/w.radius_m-1.0/610000.0);
    double dphi20=w.mu_m3_s2*(1.0/w.radius_m-1.0/620000.0);
    CHECK(near(g10,9.490997043805429,1e-12));
    CHECK(near(g20,9.187304890738814,1e-12));
    CHECK(near(dphi10,96491.80327868853,1e-8));
    CHECK(near(dphi20,189870.96774193548,1e-8));
    CHECK(near((9.81*10000.0-dphi10)/dphi10,0.016666666666666607,1e-14));
    CHECK(near((9.81*20000.0-dphi20)/dphi20,0.03333333333333344,1e-14));
    double work_rate;
    CHECK(taem_ground_arc_work_rate(2000.0,1000.0,240.0,PI/2.0,620000.0,600000.0,&work_rate));
    CHECK(near(work_rate,-480.0,1e-12));

    TaemRunway runway={0.44,-1.21,250.0,1.7,3000.0,45.0};
    TaemRunwayFrame rf, rr;
    CHECK(taem_runway_frame_create(&w,&runway,&rf));
    CHECK(taem_runway_frame_reciprocal(&w,&rf,&rr));
    CHECK(near(taem_vec3_dot(rf.forward_b,rf.right_b),0.0,1e-12));
    TaemVec3 gc_axis=taem_vec3_cross(rf.threshold_unit_b,rf.forward_b);
    double gc_angle=rf.length_m/(w.radius_m+rf.elevation_m);
    TaemVec3 transported_forward=rotate_axis(rf.forward_b,gc_axis,gc_angle);
    TaemVec3 transported_right=rotate_axis(rf.right_b,gc_axis,gc_angle);
    CHECK(taem_vec3_norm(taem_vec3_add(transported_forward,rr.forward_b))<1e-10);
    CHECK(taem_vec3_norm(taem_vec3_add(transported_right,rr.right_b))<1e-10);
    CHECK(near(taem_vec3_norm(rr.threshold_unit_b),1.0,1e-12));
    TaemVec3 reciprocal_origin;
    CHECK(taem_runway_unproject(&w,&rr,runway.length_m,0.0,0.0,&reciprocal_origin));
    CHECK(taem_vec3_norm(taem_vec3_sub(reciprocal_origin,taem_vec3_scale(rf.threshold_unit_b,w.radius_m+runway.elevation_m)))<1e-7);

    double roundtrip_max=0.0;
    for (int i=0;i<256;i++) {
        double along=-50000.0+100000.0*random_unit();
        double cross=-30000.0+60000.0*random_unit();
        double alt=-1000.0+25000.0*random_unit();
        TaemVec3 p; TaemRunwayCoordinates c;
        CHECK(taem_runway_unproject(&w,&rf,along,cross,alt,&p));
        CHECK(taem_runway_project(&w,&rf,p,&c));
        double e=fmax(fabs(c.runway_along_m-along),fmax(fabs(c.runway_cross_m-cross),fabs(c.altitude_above_runway_m-alt)));
        if (e>roundtrip_max) roundtrip_max=e;
    }
    CHECK(roundtrip_max < 2e-8);

    TaemVec3 p_i={w.radius_m+18000.0,2000.0,-500.0};
    TaemVec3 v_i={-30.0,250.0,40.0}, p_b,v_b,p_i2,v_i2;
    CHECK(taem_inertial_to_fixed(&w,p_i,v_i,1200.0,&p_b,&v_b));
    CHECK(taem_fixed_to_inertial(&w,p_b,v_b,1200.0,&p_i2,&v_i2));
    CHECK(taem_vec3_norm(taem_vec3_sub(p_i2,p_i))<1e-9);
    CHECK(taem_vec3_norm(taem_vec3_sub(v_i2,v_i))<1e-10);

    double energy_residual_max=0.0, inertial_rate_residual_max=0.0;
    for (int i=0;i<256;i++) {
        double speed=100.0+300.0*random_unit(), drag=1000.0+9000.0*random_unit(), mass=50000.0+150000.0*random_unit();
        TaemVec3 vb={speed,0.0,0.0}, air={speed,0.0,0.0};
        TaemVec3 normal={0.0,0.0,1.0};
        TaemVec3 fb=taem_vec3_add(taem_vec3_scale(unit(vb),-drag),taem_vec3_scale(normal,5000.0));
        TaemVec3 pb={w.radius_m+20000.0,0.0,1000.0}, pi,vi;
        double ut=500.0*random_unit();
        CHECK(taem_fixed_to_inertial(&w,pb,vb,ut,&pi,&vi));
        TaemVec3 fi=rotate_axis(fb,w.rotation_axis_i,w.rotation_phase_rad_at_ut0+w.rotation_rate_rad_s*ut);
        TaemEnergyDiagnostic ed;
        CHECK(taem_energy_diagnostic(&w,pi,vi,vb,air,fi,fb,mass,drag,&ed));
        double residual;
        CHECK(taem_no_wind_energy_work_residual(&ed,&residual));
        if (fabs(residual)>energy_residual_max) energy_residual_max=fabs(residual);
        double inertial_expected=taem_vec3_dot(fi,vi)/mass;
        if (fabs(ed.inertial_energy_rate_w_kg-inertial_expected)>inertial_rate_residual_max)
            inertial_rate_residual_max=fabs(ed.inertial_energy_rate_w_kg-inertial_expected);
    }
    CHECK(energy_residual_max < 1e-12);
    CHECK(inertial_rate_residual_max < 1e-12);

    TaemVec3 north,east,up;
    CHECK(taem_local_north_east_up((TaemVec3){0,0,1},&north,&east,&up));
    CHECK(near(north.x,-1.0,1e-12) && near(east.y,1.0,1e-12) && near(up.z,1.0,1e-12));
    CHECK(near(taem_wrap_pi(3.0*PI),PI,1e-12));
    CHECK(!taem_gravity_diagnostic(&w,(TaemVec3){0,0,0},&gd));
    CHECK(!taem_ground_arc_work_rate(-1.0,1000.0,240.0,0.0,620000.0,600000.0,&work_rate));

    printf("TAEM frame/energy algebraic checks passed: 256 spherical round trips max %.3g m; 256 no-wind energy residuals max %.3g W/kg; inertial energy-rate residual max %.3g W/kg\n",roundtrip_max,energy_residual_max,inertial_rate_residual_max);
    return 0;
}
