/* Contact-model regression: gear spring-damper landing, mains-first, released
   pitch settling to the measured stance, runway-edge terrain continuity,
   departure detection, and stopping. */
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
static double pitch_deg(const Simulation *sim){
    Plane p=plane_at(sim,sim->state.ut);
    Vec3 fwd=quat_rotate(sim->state.body_q_i,ss_v3(1,0,0));
    return rad2deg(asin(v3_dot(fwd,p.normal)));
}
static void place(Simulation *sim,double along,double cross,double height,double speed,double sink,double aoa_deg){
    Plane p=plane_at(sim,sim->state.ut);
    sim->state.position_i_m=v3_add(v3_add(v3_add(p.origin,v3_scale(p.forward,along)),v3_scale(p.right,cross)),v3_scale(p.normal,height));
    Vec3 air=v3_add(v3_scale(p.forward,speed),v3_scale(p.normal,-sink));
    sim->state.velocity_i_mps=v3_add(air,world_atmosphere_velocity_i(&sim->world,sim->state.position_i_m));
    AttitudeModel *a=&sim->state.attitude;
    a->aoa_rad=a->cmd_aoa_rad=a->requested_aoa_rad=deg2rad(aoa_deg);
    sim->state.body_q_i=attitude_body_quat(sim->state.position_i_m,air,a->aoa_rad,0.0);
}
static void seed(Scenario *sc){
    memset(sc,0,sizeof(*sc));strcpy(sc->name,"collision-regression");
    sc->ut0=0;sc->latitude_deg=-0.0486;sc->longitude_deg=-74.7240;sc->altitude_m=77;
    sc->heading_deg=90;sc->has_surface_flight_state=true;sc->surface_speed_mps=70;sc->flight_path_angle_deg=-4;
    sc->mass_kg=40000;sc->initial_aoa_deg=8;sc->initial_bank_deg=0;
}
static bool initial_state_is_preserved(void){
    Scenario sc={.has_cartesian_state=true,.mass_kg=40000,
        .position_i_m={605000,12,-23},.velocity_i_mps={-10,210,70}};
    Simulation sim;sim_init(&sim,&sc);
    if(v3_norm(v3_sub(sim.state.position_i_m,sc.position_i_m))!=0.0||
       v3_norm(v3_sub(sim.state.velocity_i_mps,sc.velocity_i_mps))!=0.0)return false;
    sc=(Scenario){.has_surface_flight_state=true,.surface_speed_mps=0,
        .altitude_m=5000,.mass_kg=40000};
    sim_init(&sim,&sc);
    Vec3 atmosphere=world_atmosphere_velocity_i(&sim.world,sim.state.position_i_m);
    return v3_norm(v3_sub(sim.state.velocity_i_mps,atmosphere))==0.0;
}

int main(void){
    if(!initial_state_is_preserved()){fputs("Authored initial state changed\n",stderr);return 4;}
    static Scenario sc;static Simulation sim;
    AeroTable *aero=&sim.aero;(void)aero;

    /* 1. Gentle gear-down landing at 8 deg: mains first, sink recorded, the
          released nose comes down to the measured stance, brakes stop it on
          the runway with no departure and a bounded gear load. */
    seed(&sc);sim_init(&sim,&sc);sim.state.gear_down=true;
    place(&sim,300.0,0.0,3.40,70.0,1.5,8.0);
    for(int i=0;i<200&&!sim.state.touchdown_seen;i++)sim_step(&sim,0.02);
    printf("touch=%d gear=%d runway=%d sink=%.2f pitch=%.2f\n",sim.state.touchdown_seen,
           sim.state.touchdown_gear,sim.state.on_runway_at_touchdown,sim.state.touchdown_sink_mps,
           sim.state.touchdown_pitch_deg);
    if(!sim.state.touchdown_seen||!sim.state.touchdown_gear||!sim.state.on_runway_at_touchdown||
       fabs(sim.state.touchdown_sink_mps-1.5)>0.6)return 1;
    sim.state.brakes=true;
    for(int i=0;i<6000&&!sim.state.stopped;i++)sim_step(&sim,0.02);
    printf("stopped=%d along=%.1f nose=%d pitch=%.2f bounces=%d max_g=%.2f departure=%d\n",
           sim.state.stopped,sim.state.stop_along_m,sim.state.nose_touchdown_seen,pitch_deg(&sim),
           sim.state.bounce_count,sim.state.max_gear_load_g,sim.state.runway_departure);
    if(!sim.state.stopped||!sim.state.nose_touchdown_seen||fabs(pitch_deg(&sim)+4.3)>1.0||
       sim.state.runway_departure||sim.state.max_gear_load_g>4.0||sim.state.stop_along_m>2500.0)return 2;

    /* 2. Rolling off the far end is a runway departure on the plateau, not a
          fall off a 70 m cliff. */
    seed(&sc);sim_init(&sim,&sc);sim.state.gear_down=true;
    place(&sim,2300.0,0.0,4.2,60.0,1.0,8.0);
    for(int i=0;i<3000&&!sim.state.stopped;i++)sim_step(&sim,0.02);
    Plane p=plane_at(&sim,sim.state.ut);
    double h=v3_dot(v3_sub(sim.state.position_i_m,p.origin),p.normal);
    printf("departure=%d along=%.1f speed=%.1f final_height=%.2f\n",sim.state.runway_departure,
           sim.state.departure_along_m,sim.state.departure_speed_mps,h);
    if(!sim.state.runway_departure||sim.state.departure_along_m<2400.0||h<1.0)return 3;

    /* 3. Gear-up contact is a belly contact, never a gear touchdown. */
    seed(&sc);sim_init(&sim,&sc);sim.state.gear_down=false;
    place(&sim,300.0,0.0,2.5,70.0,1.0,4.0);
    for(int i=0;i<200&&!sim.state.touchdown_seen;i++)sim_step(&sim,0.02);
    printf("belly touch=%d gear=%d\n",sim.state.touchdown_seen,sim.state.touchdown_gear);
    if(!sim.state.touchdown_seen||sim.state.touchdown_gear)return 5;
    puts("runway_collision: PASS");
    return 0;
}
