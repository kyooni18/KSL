#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "shuttlesim/sim.h"
#include "shuttlesim/math3.h"
#include "shuttlesim/quat.h"

typedef struct { Vec3 origin, forward, right, normal; } Plane;
static Plane plane_at(const Simulation *sim,double ut){
    Plane p;
    p.origin=world_lla_to_inertial(&sim->world,sim->runway.lat_rad,sim->runway.lon_rad,sim->runway.elevation_m,ut);
    LocalFrame lf=world_local_frame_i(&sim->world,p.origin,ut);
    p.forward=v3_normalized(v3_add(v3_scale(lf.north,cos(sim->runway.heading_rad)),v3_scale(lf.east,sin(sim->runway.heading_rad))));
    p.normal=lf.up;p.right=v3_normalized(v3_cross(p.normal,p.forward));return p;
}
static double gap(const Plane *p,Vec3 q){return v3_dot(v3_sub(q,p->origin),p->normal);}
static void seed(Scenario *sc){
    memset(sc,0,sizeof(*sc));strcpy(sc->name,"collision-regression");
    sc->ut0=0;sc->latitude_deg=-0.0486;sc->longitude_deg=-74.7240;sc->altitude_m=77;
    sc->heading_deg=90;sc->has_surface_flight_state=true;sc->surface_speed_mps=70;sc->flight_path_angle_deg=-4;
    sc->mass_kg=70000;sc->initial_aoa_deg=4;sc->initial_bank_deg=0;
}
int main(void){
    Scenario sc;seed(&sc);Simulation sim;sim_init(&sim,&sc);sim.state.gear_down=true;
    for(int i=0;i<300&&!sim.state.touchdown_seen;i++)sim_step(&sim,0.02);
    Plane p=plane_at(&sim,sim.state.ut);
    Vec3 wheel=v3_add(sim.state.position_i_m,quat_rotate(sim.state.body_q_i,ss_v3(-3,0,5.5)));
    LLA l=world_lla(&sim.world,sim.state.position_i_m,sim.state.ut);
    printf("nominal touch=%d gap=%.9f cg_alt=%.3f\n",sim.state.touchdown_seen,gap(&p,wheel),l.altitude_m);
    if(!sim.state.touchdown_seen||fabs(gap(&p,wheel))>1e-5||l.altitude_m<74.0)return 1;

    seed(&sc);sim_init(&sim,&sc);sim.state.gear_down=true;p=plane_at(&sim,sim.state.ut);
    sim.state.position_i_m=v3_add(v3_add(p.origin,v3_scale(p.forward,2400.0)),v3_scale(p.normal,7.0));
    Vec3 air=v3_add(v3_scale(p.forward,70.0),v3_scale(p.normal,-4.0));
    sim.state.velocity_i_mps=v3_add(air,world_atmosphere_velocity_i(&sim.world,sim.state.position_i_m));
    sim.state.body_q_i=attitude_body_quat(sim.state.position_i_m,air,sim.state.attitude.aoa_rad,sim.state.attitude.bank_rad);
    for(int i=0;i<300&&!sim.state.touchdown_seen;i++)sim_step(&sim,0.02);
    p=plane_at(&sim,sim.state.ut);wheel=v3_add(sim.state.position_i_m,quat_rotate(sim.state.body_q_i,ss_v3(-3,0,5.5)));
    l=world_lla(&sim.world,sim.state.position_i_m,sim.state.ut);
    printf("far touch=%d gap=%.9f cg_alt=%.3f along=%.3f\n",sim.state.touchdown_seen,gap(&p,wheel),l.altitude_m,sim.state.touchdown_along_m);
    if(!sim.state.touchdown_seen||fabs(gap(&p,wheel))>1e-5||l.altitude_m<78.0)return 2;

    seed(&sc);sim_init(&sim,&sc);sim.state.gear_down=false;p=plane_at(&sim,sim.state.ut);
    sim.state.position_i_m=v3_add(v3_add(p.origin,v3_scale(p.forward,-1.0)),v3_scale(p.normal,1.0));
    air=v3_scale(p.forward,100.0);
    sim.state.velocity_i_mps=v3_add(air,world_atmosphere_velocity_i(&sim.world,sim.state.position_i_m));
    sim.state.attitude.aoa_rad=sim.state.attitude.cmd_aoa_rad=sim.state.attitude.requested_aoa_rad=0.0;
    sim.state.body_q_i=attitude_body_quat(sim.state.position_i_m,air,0.0,0.0);
    sim_step(&sim,0.02);
    printf("edge touch=%d on_ground=%d\n",sim.state.touchdown_seen,sim.state.on_ground);
    if(sim.state.touchdown_seen||sim.state.on_ground)return 3;
    puts("runway_collision: PASS");
    return 0;
}
