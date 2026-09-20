#include <stdio.h>
#include <string.h>
#include <math.h>
#include <float.h>
#include "shuttlesim/sim.h"
#include "shuttlesim/math3.h"
#include "shuttlesim/quat.h"

static Vec3 flight_accel(const Simulation *sim,Vec3 p,Vec3 v,double ut,AeroForces *af){
    Vec3 g=world_gravity_accel(&sim->world,p);
    AeroForces a=aero_compute(&sim->world,&sim->aero,p,v,ut,sim->state.mass_kg,sim->state.attitude.aoa_rad,sim->state.attitude.bank_rad);
    if(af)*af=a;
    Vec3 out=v3_add(g,v3_scale(a.force_i,1.0/sim->state.mass_kg));
    if(sim->deorbit_burn_remaining_s>0 && sim->deorbit_burn_accel_mps2>0){
        Vec3 retro=v3_scale(v3_normalized(v),-sim->deorbit_burn_accel_mps2);
        out=v3_add(out,retro);
    }
    return out;
}

static void derive_initial_orbit(Simulation *sim){
    const Scenario *s=&sim->scenario; KerbinWorld *w=&sim->world;
    if(s->has_cartesian_state){sim->state.position_i_m=s->position_i_m;sim->state.velocity_i_mps=s->velocity_i_mps;return;}
    Vec3 p=world_lla_to_inertial(w,deg2rad(s->latitude_deg),deg2rad(s->longitude_deg),s->altitude_m,s->ut0);
    LocalFrame lf=world_local_frame_i(w,p,s->ut0); double hdg=deg2rad(s->heading_deg);
    Vec3 dir=v3_normalized(v3_add(v3_scale(lf.north,cos(hdg)),v3_scale(lf.east,sin(hdg))));
    double vc=sqrt(w->mu_m3_s2/v3_norm(p));
    sim->state.position_i_m=p; sim->state.velocity_i_mps=v3_scale(dir,vc);
}
void sim_init(Simulation *sim,const Scenario *scenario){
    memset(sim,0,sizeof(*sim));
    world_seed_kerbin(&sim->world);
    runway_seed_ksp09(&sim->runway);
    aero_seed_stsn(&sim->aero);
    sim->scenario=*scenario;
    if(scenario->runway_override){
        sim->runway.lat_rad=deg2rad(scenario->runway_latitude_deg);
        sim->runway.lon_rad=deg2rad(scenario->runway_longitude_deg);
        sim->runway.elevation_m=scenario->runway_elevation_m;
        sim->runway.heading_rad=deg2rad(scenario->runway_heading_deg);
        sim->runway.length_m=scenario->runway_length_m;
        sim->runway.width_m=scenario->runway_width_m;
    }sim->physics_dt_s=0.02;sim->rolling_mu=0.025;sim->brake_mu=0.18;
    sim->state.ut=scenario->ut0;sim->state.mass_kg=scenario->mass_kg;attitude_seed(&sim->state.attitude);
    sim->state.attitude.aoa_rad=sim->state.attitude.cmd_aoa_rad=deg2rad(scenario->initial_aoa_deg);
    sim->state.attitude.bank_rad=sim->state.attitude.cmd_bank_rad=deg2rad(scenario->initial_bank_deg);
    derive_initial_orbit(sim);sim_apply_deorbit_initial(sim);
    Vec3 vair=v3_sub(sim->state.velocity_i_mps,world_atmosphere_velocity_i(&sim->world,sim->state.position_i_m));
    sim->state.body_q_i=attitude_body_quat(sim->state.position_i_m,vair,sim->state.attitude.aoa_rad,sim->state.attitude.bank_rad);
}
void sim_apply_deorbit_initial(Simulation *sim){
    double dv=sim->scenario.deorbit_delta_v_mps; if(dv<=0)return;
    sim->deorbit_burn_delay_remaining_s=fmax(0.0,sim->scenario.deorbit_delay_s);
    if(sim->scenario.deorbit_duration_s<=0){
        if(sim->deorbit_burn_delay_remaining_s<=0){
            sim->state.velocity_i_mps=v3_add(sim->state.velocity_i_mps,v3_scale(v3_normalized(sim->state.velocity_i_mps),-dv));
        } else {
            sim->deorbit_impulse_pending_mps=dv;
        }
    } else {
        sim->deorbit_burn_accel_mps2=dv/sim->scenario.deorbit_duration_s;
        if(sim->deorbit_burn_delay_remaining_s<=0) sim->deorbit_burn_remaining_s=sim->scenario.deorbit_duration_s;
        else sim->deorbit_burn_pending_duration_s=sim->scenario.deorbit_duration_s;
    }
}
void sim_set_attitude(Simulation *sim,double aoa_deg,double bank_deg){attitude_set_command(&sim->state.attitude,deg2rad(aoa_deg),deg2rad(bank_deg));}
void sim_apply_command(Simulation *sim,const SimCommand *c){
    if(c->has_attitude) sim_set_attitude(sim,c->aoa_deg,c->bank_deg);
    if(c->has_gear) sim->state.gear_down=c->gear_down;
    if(c->has_brakes) sim->state.brakes=c->brakes;
    if(c->pause) sim->paused=true;
    if(c->resume) sim->paused=false;
}

static void ground_step(Simulation *sim,double dt){
    SimState *s=&sim->state;KerbinWorld *w=&sim->world;Vec3 up=v3_normalized(s->position_i_m);
    Vec3 surface=v3_sub(s->velocity_i_mps,world_atmosphere_velocity_i(w,s->position_i_m));
    surface=v3_sub(surface,v3_scale(up,v3_dot(surface,up)));double speed=v3_norm(surface);
    double mu=s->brakes?sim->brake_mu:sim->rolling_mu; double g=w->mu_m3_s2/(v3_norm(s->position_i_m)*v3_norm(s->position_i_m));
    double ns=fmax(0.0,speed-mu*g*dt);
    if(speed>0.0)surface=v3_scale(surface,ns/speed);
    else surface=v3_scale(surface,0.0);
    s->velocity_i_mps=v3_add(surface,world_atmosphere_velocity_i(w,s->position_i_m));
    s->position_i_m=v3_add(s->position_i_m,v3_scale(s->velocity_i_mps,dt));
    double along,cross,vert;runway_coordinates(w,&sim->runway,s->position_i_m,s->ut,&along,&cross,&vert);
    bool runway=runway_contains(&sim->runway,along,cross);
    double ground=runway?sim->runway.elevation_m:0.0;
    double r=w->radius_m+ground;
    s->position_i_m=v3_scale(v3_normalized(s->position_i_m),r);
    /* Preserve the ground-relative speed while transporting it to the new
       tangent plane.  At ns == 0 this leaves the vehicle exactly co-rotating
       with the surface instead of manufacturing a small residual velocity. */
    Vec3 new_up=v3_normalized(s->position_i_m);
    surface=v3_sub(surface,v3_scale(new_up,v3_dot(surface,new_up)));
    double tangent_speed=v3_norm(surface);
    if(ns<=0.0)surface=v3_scale(surface,0.0);
    else if(tangent_speed>0.0)surface=v3_scale(surface,ns/tangent_speed);
    s->velocity_i_mps=v3_add(surface,world_atmosphere_velocity_i(w,s->position_i_m));
    s->aero=aero_compute(w,&sim->aero,s->position_i_m,s->velocity_i_mps,s->ut,s->mass_kg,s->attitude.aoa_rad,s->attitude.bank_rad);
}

void sim_step(Simulation *sim,double dt){
    if(sim->paused) return;
    SimState *s=&sim->state;
    if(sim->deorbit_burn_delay_remaining_s>0){
        sim->deorbit_burn_delay_remaining_s=fmax(0.0,sim->deorbit_burn_delay_remaining_s-dt);
        if(sim->deorbit_burn_delay_remaining_s<=0){
            if(sim->deorbit_impulse_pending_mps>0){
                double dv=sim->deorbit_impulse_pending_mps;
                s->velocity_i_mps=v3_add(s->velocity_i_mps,v3_scale(v3_normalized(s->velocity_i_mps),-dv));
                sim->deorbit_impulse_pending_mps=0;
            }
            if(sim->deorbit_burn_pending_duration_s>0){
                sim->deorbit_burn_remaining_s=sim->deorbit_burn_pending_duration_s;
                sim->deorbit_burn_pending_duration_s=0;
            }
        }
    }
    attitude_step(&s->attitude,dt);
    if(s->on_ground){ground_step(sim,dt);s->ut+=dt;s->sim_elapsed_s+=dt;return;}
    Vec3 p=s->position_i_m,v=s->velocity_i_mps;double ut=s->ut;AeroForces tmp;
    Vec3 a1=flight_accel(sim,p,v,ut,&tmp);Vec3 k1p=v,k1v=a1;
    Vec3 p2=v3_add(p,v3_scale(k1p,dt*0.5));Vec3 v2=v3_add(v,v3_scale(k1v,dt*0.5));
    Vec3 a2=flight_accel(sim,p2,v2,ut+dt*0.5,NULL);Vec3 k2p=v2,k2v=a2;
    Vec3 p3=v3_add(p,v3_scale(k2p,dt*0.5));Vec3 v3v=v3_add(v,v3_scale(k2v,dt*0.5));
    Vec3 a3=flight_accel(sim,p3,v3v,ut+dt*0.5,NULL);Vec3 k3p=v3v,k3v=a3;
    Vec3 p4=v3_add(p,v3_scale(k3p,dt));Vec3 v4=v3_add(v,v3_scale(k3v,dt));
    Vec3 a4=flight_accel(sim,p4,v4,ut+dt,NULL);Vec3 k4p=v4,k4v=a4;
    s->position_i_m=v3_add(p,v3_scale(v3_add(v3_add(k1p,v3_scale(k2p,2)),v3_add(v3_scale(k3p,2),k4p)),dt/6.0));
    s->velocity_i_mps=v3_add(v,v3_scale(v3_add(v3_add(k1v,v3_scale(k2v,2)),v3_add(v3_scale(k3v,2),k4v)),dt/6.0));
    if(sim->deorbit_burn_remaining_s>0){sim->deorbit_burn_remaining_s=fmax(0,sim->deorbit_burn_remaining_s-dt);}
    s->ut+=dt;s->sim_elapsed_s+=dt;
    s->aero=aero_compute(&sim->world,&sim->aero,s->position_i_m,s->velocity_i_mps,s->ut,s->mass_kg,s->attitude.aoa_rad,s->attitude.bank_rad);
    Vec3 vair=v3_sub(s->velocity_i_mps,world_atmosphere_velocity_i(&sim->world,s->position_i_m));
    s->body_q_i=attitude_body_quat(s->position_i_m,vair,s->attitude.aoa_rad,s->attitude.bank_rad);

    LLA l=world_lla(&sim->world,s->position_i_m,s->ut);double along,cross,vertical;
    runway_coordinates(&sim->world,&sim->runway,s->position_i_m,s->ut,&along,&cross,&vertical);
    bool near_runway=runway_contains(&sim->runway,along,cross);
    double ground=near_runway?sim->runway.elevation_m:0.0;
    if(l.altitude_m<=ground){
        LocalFrame lf=world_local_frame_i(&sim->world,s->position_i_m,s->ut);Vec3 surf=v3_sub(s->velocity_i_mps,world_atmosphere_velocity_i(&sim->world,s->position_i_m));
        double vv=v3_dot(surf,lf.up);Vec3 horiz=v3_sub(surf,v3_scale(lf.up,vv));
        if(!s->touchdown_seen){s->touchdown_seen=true;s->on_runway_at_touchdown=near_runway;s->touchdown_sink_mps=-vv;s->touchdown_speed_mps=v3_norm(horiz);s->touchdown_along_m=along;s->touchdown_cross_m=cross;}
        s->on_ground=true;s->position_i_m=v3_scale(v3_normalized(s->position_i_m),sim->world.radius_m+ground);s->velocity_i_mps=v3_add(horiz,world_atmosphere_velocity_i(&sim->world,s->position_i_m));
    }
}

void sim_build_telemetry_json(const Simulation *sim,double rate,char *out,size_t n){
    const SimState *s=&sim->state;LLA l=world_lla(&sim->world,s->position_i_m,s->ut);LocalFrame lf=world_local_frame_i(&sim->world,s->position_i_m,s->ut);
    Vec3 surf=v3_sub(s->velocity_i_mps,world_atmosphere_velocity_i(&sim->world,s->position_i_m));Vec3 air=surf;
    double vn=v3_dot(surf,lf.north),ve=v3_dot(surf,lf.east),vu=v3_dot(surf,lf.up);
    double horizontal=hypot(vn,ve),surface_speed=v3_norm(surf),heading=atan2(ve,vn);if(heading<0)heading+=2*3.14159265358979323846;
    double stop_resolution=sqrt(DBL_EPSILON)*fmax(1.0,v3_norm(s->velocity_i_mps));
    bool stopped=s->on_ground&&surface_speed<=stop_resolution;
    double fpa=atan2(vu,fmax(horizontal,1e-9));
    AtmosphereSample atm=world_atmosphere_sample(&sim->world,l.altitude_m);
    double along,cross,vertical;runway_coordinates(&sim->world,&sim->runway,s->position_i_m,s->ut,&along,&cross,&vertical);
    snprintf(out,n,"{\"schema\":1,\"type\":\"telemetry\",\"source\":\"sim\",\"scenario\":\"%s\",\"ut\":%.6f,\"sim_time\":%.6f,\"sim_rate\":%.2f,"
             "\"world\":{\"radius_m\":%.9g,\"mu_m3_s2\":%.12g,\"rotation_rate_rad_s\":%.12g,\"atmosphere_top_m\":%.9g,\"rotation_phase_rad_at_ut0\":%.12g},"
             "\"position\":{\"x\":%.6f,\"y\":%.6f,\"z\":%.6f,\"lat_deg\":%.9f,\"lon_deg\":%.9f,\"altitude_m\":%.3f},"
             "\"velocity\":{\"x_mps\":%.6f,\"y_mps\":%.6f,\"z_mps\":%.6f,\"inertial_mps\":%.3f,\"surface_mps\":%.3f,\"air_mps\":%.3f,\"horizontal_mps\":%.3f,\"vertical_mps\":%.3f,\"flight_path_angle_deg\":%.6f},"
             "\"attitude\":{\"aoa_deg\":%.4f,\"bank_deg\":%.4f,\"heading_deg\":%.4f,\"cmd_aoa_deg\":%.4f,\"cmd_bank_deg\":%.4f,"
             "\"aoa_rate_deg_s\":%.6f,\"bank_rate_deg_s\":%.6f,"
             "\"pitch_wn_s_inv\":%.9g,\"pitch_zeta\":%.9g,\"roll_wn_s_inv\":%.9g,\"roll_zeta\":%.9g,"
             "\"max_pitch_rate_deg_s\":%.9g,\"max_roll_rate_deg_s\":%.9g,"
             "\"max_pitch_accel_deg_s2\":%.9g,\"max_roll_accel_deg_s2\":%.9g,"
             "\"q_w\":%.9f,\"q_x\":%.9f,\"q_y\":%.9f,\"q_z\":%.9f},"
             "\"atmosphere\":{\"density_kg_m3\":%.12g,\"pressure_pa\":%.9g,\"temperature_k\":%.6f,\"speed_of_sound_mps\":%.6f},"
             "\"aero\":{\"mach\":%.6f,\"q_pa\":%.6f,\"lift_n\":%.6f,\"drag_n\":%.6f},"
             "\"vehicle\":{\"mass_kg\":%.6f},"
             "\"runway\":{\"along_m\":%.3f,\"cross_m\":%.3f,\"vertical_m\":%.3f,\"length_m\":%.3f,\"width_m\":%.3f,\"heading_deg\":%.6f},"
             "\"ground\":{\"gear_down\":%s,\"brakes\":%s,\"on_ground\":%s,\"stopped\":%s,\"touchdown_seen\":%s,\"on_runway_touchdown\":%s}}",
             sim->scenario.name,s->ut,s->sim_elapsed_s,rate,sim->world.radius_m,sim->world.mu_m3_s2,sim->world.rotation_rate_rad_s,sim->world.atmosphere_top_m,sim->world.rotation_phase_rad_at_ut0,s->position_i_m.x,s->position_i_m.y,s->position_i_m.z,rad2deg(l.lat_rad),rad2deg(l.lon_rad),l.altitude_m,
             s->velocity_i_mps.x,s->velocity_i_mps.y,s->velocity_i_mps.z,v3_norm(s->velocity_i_mps),v3_norm(surf),v3_norm(air),horizontal,vu,rad2deg(fpa),
             rad2deg(s->attitude.aoa_rad),rad2deg(s->attitude.bank_rad),rad2deg(heading),rad2deg(s->attitude.cmd_aoa_rad),rad2deg(s->attitude.cmd_bank_rad),
             rad2deg(s->attitude.aoa_rate_rad_s),rad2deg(s->attitude.bank_rate_rad_s),
             s->attitude.pitch_wn,s->attitude.pitch_zeta,s->attitude.roll_wn,s->attitude.roll_zeta,
             rad2deg(s->attitude.max_pitch_rate_rad_s),rad2deg(s->attitude.max_roll_rate_rad_s),
             rad2deg(s->attitude.max_pitch_accel_rad_s2),rad2deg(s->attitude.max_roll_accel_rad_s2),
             s->body_q_i.w,s->body_q_i.x,s->body_q_i.y,s->body_q_i.z,
             atm.density_kg_m3,atm.pressure_pa,atm.temperature_k,atm.speed_of_sound_mps,
             s->aero.mach,s->aero.dynamic_pressure_pa,s->aero.lift_n,s->aero.drag_n,s->mass_kg,
             along,cross,vertical,sim->runway.length_m,sim->runway.width_m,rad2deg(sim->runway.heading_rad),
             s->gear_down?"true":"false",s->brakes?"true":"false",s->on_ground?"true":"false",stopped?"true":"false",
             s->touchdown_seen?"true":"false",s->on_runway_at_touchdown?"true":"false");
}
void sim_build_summary_json(const Simulation *sim,char *out,size_t n){
    const SimState *s=&sim->state;LLA l=world_lla(&sim->world,s->position_i_m,s->ut);double along,cross,vertical;runway_coordinates(&sim->world,&sim->runway,s->position_i_m,s->ut,&along,&cross,&vertical);
    snprintf(out,n,"{\"scenario\":\"%s\",\"sim_time_s\":%.3f,\"final_altitude_m\":%.3f,\"touchdown\":%s,\"on_runway\":%s,\"touchdown_sink_mps\":%.3f,\"touchdown_speed_mps\":%.3f,\"touchdown_along_m\":%.3f,\"touchdown_cross_m\":%.3f,\"final_along_m\":%.3f,\"final_cross_m\":%.3f}",
             sim->scenario.name,s->sim_elapsed_s,l.altitude_m,s->touchdown_seen?"true":"false",s->on_runway_at_touchdown?"true":"false",s->touchdown_sink_mps,s->touchdown_speed_mps,s->touchdown_along_m,s->touchdown_cross_m,along,cross);
}
