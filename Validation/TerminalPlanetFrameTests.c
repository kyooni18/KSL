/* Fast, deterministic guard: native MM305 plant must use the live KSP
   inertial/rotating frame before it may certify an orbital deorbit path.
   This test does not open kRPC or modify the game save. */
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include "landing.h"
#include "terminal_model.h"
#include "sim_telemetry.h"

static double wrap_degrees(double x){
    x=fmod(x+180.0,360.0);
    if(x<0)x+=360.0;
    return x-180.0;
}
int main(void){
    LandingConfiguration cfg=landing_configuration_default();
    char paths[4][512],reason[192];
    TerminalModelSourceFiles files={
        shuttle_sim_model_path(NULL,SHUTTLE_SIM_MODEL_ATMOSPHERE,paths[0],512),
        shuttle_sim_model_path(NULL,SHUTTLE_SIM_MODEL_AERO,paths[1],512),
        shuttle_sim_model_path(NULL,SHUTTLE_SIM_MODEL_FORCE_BOOK,paths[2],512),
        shuttle_sim_model_path(NULL,SHUTTLE_SIM_MODEL_ATTITUDE,paths[3],512)};
    TerminalModel *model=calloc(1,sizeof(*model));
    assert(model);
    assert(terminal_model_capture(model,&files,&cfg,314,reason,sizeof(reason)));
    assert(model->replay_validated&&!model->planet_frame_aligned);
    PlanetModel p={.radius=model->world.radius_m,
        .gravitational_parameter=model->world.mu_m3_s2,
        .rotational_speed=model->world.rotation_rate_rad_s,
        .epoch_ut=146949660.647894,
        .north_axis={0,0,1},
        .prime_meridian_at_epoch={-.933582298018746,.358363074026938,0}};
    assert(terminal_model_align_planet_frame(model,&p,reason,sizeof(reason)));
    assert(model->planet_frame_aligned);
    assert(terminal_model_policy_matches(model,&cfg.site,&cfg.vehicle,&cfg.guidance));
    VehicleProfile changed=cfg.vehicle;
    changed.minimum_safe_speed+=1.0;
    assert(!terminal_model_policy_matches(model,&cfg.site,&changed,&cfg.guidance));
    LandingSite other=cfg.site;
    other.runway_heading+=1.0;
    assert(!terminal_model_policy_matches(model,&other,&cfg.vehicle,&cfg.guidance));
    GuidanceSettings different=cfg.guidance;
    different.hac_radius+=100.0;
    assert(!terminal_model_policy_matches(model,&cfg.site,&cfg.vehicle,&different));

    const double pi=acos(-1.0),rad=180.0/pi;
    Vec3 prime=world_fixed_to_inertial(&model->world,(Vec3){1,0,0},p.epoch_ut);
    double longitude=atan2(p.prime_meridian_at_epoch.y,p.prime_meridian_at_epoch.x);
    assert(fabs(prime.x-cos(longitude))<1e-10);
    assert(fabs(prime.y-sin(longitude))<1e-10);
    const Vector3 orbiter_position={265913.976923641,266230.434410646,590362.583042728};
    for(double dt=0;dt<=1200;dt+=300){
        double ut=p.epoch_ut+dt;
        double prime_angle=longitude+p.rotational_speed*dt;
        double x=orbiter_position.x*cos(prime_angle)+orbiter_position.y*sin(prime_angle);
        double y=-orbiter_position.x*sin(prime_angle)+orbiter_position.y*cos(prime_angle);
        double r=hypot(hypot(x,y),orbiter_position.z);
        double expected_lat=asin(orbiter_position.z/r)*rad;
        double expected_lon=atan2(y,x)*rad;
        LLA pos=world_lla(&model->world,
            (Vec3){orbiter_position.x,orbiter_position.y,orbiter_position.z},ut);
        assert(fabs(pos.lat_rad*rad-expected_lat)<1e-7);
        assert(fabs(wrap_degrees(pos.lon_rad*rad-expected_lon))<1e-7);
        assert(fabs(pos.altitude_m-(r-p.radius))<1e-5);
    }

    /* Reject a tilted axis or dynamic mismatch, clearing any former binding. */
    p.north_axis=(Vector3){.1,0,.995};
    assert(!terminal_model_align_planet_frame(model,&p,reason,sizeof(reason)));
    assert(!model->planet_frame_aligned);
    p.north_axis=(Vector3){0,0,1};
    p.gravitational_parameter*=1.1;
    assert(!terminal_model_align_planet_frame(model,&p,reason,sizeof(reason)));
    assert(!model->planet_frame_aligned);
    free(model);
    puts("Terminal planet-frame alignment tests passed.");
    return 0;
}
