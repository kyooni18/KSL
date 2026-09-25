#include <stdio.h>
#include <string.h>
#include <math.h>
#include <float.h>
#include <stdlib.h>
#include "shuttlesim/sim.h"
#include "shuttlesim/flight_physics.h"
#include "shuttlesim/math3.h"
#include "shuttlesim/quat.h"

typedef struct {
    Vec3 origin_i, forward_i, right_i, normal_i;
} RunwayPlane;

static RunwayPlane runway_plane_at(const Simulation *sim,double ut){
    const Runway *r=&sim->runway;const KerbinWorld *w=&sim->world;
    RunwayPlane p;
    p.origin_i=world_lla_to_inertial(w,r->lat_rad,r->lon_rad,r->elevation_m,ut);
    LocalFrame lf=world_local_frame_i(w,p.origin_i,ut);
    p.forward_i=v3_normalized(v3_add(v3_scale(lf.north,cos(r->heading_rad)),
                                     v3_scale(lf.east,sin(r->heading_rad))));
    p.normal_i=lf.up;
    p.right_i=v3_normalized(v3_cross(p.normal_i,p.forward_i));
    return p;
}

static void runway_plane_coords(const RunwayPlane *p,Vec3 point,
                                double *along,double *cross,double *height){
    Vec3 d=v3_sub(point,p->origin_i);
    if(along)*along=v3_dot(d,p->forward_i);
    if(cross)*cross=v3_dot(d,p->right_i);
    if(height)*height=v3_dot(d,p->normal_i);
}

static Vec3 primary_contact_b(const SimState *s){
    return s->gear_down?ss_v3(-3.0,0.0,5.5):ss_v3(-3.0,0.0,2.0);
}

static bool runway_contact_crossing(const Simulation *sim,
                                    RunwayPlane prev_plane,RunwayPlane cur_plane,
                                    Vec3 prev_point,Vec3 cur_point,
                                    double *toi,double *along,double *cross,
                                    double *cur_height){
    double a0,c0,h0,a1,c1,h1;
    runway_plane_coords(&prev_plane,prev_point,&a0,&c0,&h0);
    runway_plane_coords(&cur_plane,cur_point,&a1,&c1,&h1);
    if(cur_height)*cur_height=h1;
    if(h0>0.0&&h1<=0.0){
        double d=h0-h1;if(!(d>DBL_EPSILON))return false;
        double t=ss_clampd(h0/d,0.0,1.0);
        double a=a0+(a1-a0)*t,c=c0+(c1-c0)*t;
        if(!runway_contains(&sim->runway,a,c))return false;
        if(toi)*toi=t;if(along)*along=a;if(cross)*cross=c;return true;
    }
    if(h0<=0.0&&h1<=0.0&&
       runway_contains(&sim->runway,a0,c0)&&runway_contains(&sim->runway,a1,c1)){
        if(toi)*toi=0.0;if(along)*along=a1;if(cross)*cross=c1;return true;
    }
    return false;
}

static void derive_initial_orbit(Simulation *sim){
    const Scenario *s=&sim->scenario; KerbinWorld *w=&sim->world;
    if(s->has_cartesian_state){sim->state.position_i_m=s->position_i_m;sim->state.velocity_i_mps=s->velocity_i_mps;return;}
    Vec3 p=world_lla_to_inertial(w,deg2rad(s->latitude_deg),deg2rad(s->longitude_deg),s->altitude_m,s->ut0);
    LocalFrame lf=world_local_frame_i(w,p,s->ut0); double hdg=deg2rad(s->heading_deg);
    Vec3 horizontal_dir=v3_normalized(v3_add(v3_scale(lf.north,cos(hdg)),v3_scale(lf.east,sin(hdg))));
    sim->state.position_i_m=p;
    if(s->has_surface_flight_state){
        double fpa=deg2rad(s->flight_path_angle_deg);
        Vec3 air=v3_add(v3_scale(horizontal_dir,s->surface_speed_mps*cos(fpa)),
                       v3_scale(lf.up,s->surface_speed_mps*sin(fpa)));
        sim->state.velocity_i_mps=v3_add(air,world_atmosphere_velocity_i(w,p));
    }else{
        double vc=sqrt(w->mu_m3_s2/v3_norm(p));
        sim->state.velocity_i_mps=v3_scale(horizontal_dir,vc);
    }
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
    /* Conservative terminal-ground model.  The split vertical fin is represented
       as incremental drag area, while steering is bounded by lateral acceleration
       and yaw rate so the simulator cannot hide a rollover-prone rollout behind
       an unrealistically fast heading correction. */
    sim->ground_airbrake_cda_m2=8.0;
    sim->ground_max_lateral_accel_g=0.12;
    sim->ground_max_yaw_rate_rad_s=deg2rad(10.0);
    sim->state.ut=scenario->ut0;sim->state.mass_kg=scenario->mass_kg;attitude_seed(&sim->state.attitude);
    sim->state.attitude.aoa_rad=sim->state.attitude.cmd_aoa_rad=
        sim->state.attitude.requested_aoa_rad=deg2rad(scenario->initial_aoa_deg);
    sim->state.attitude.bank_rad=sim->state.attitude.cmd_bank_rad=
        sim->state.attitude.requested_bank_rad=deg2rad(scenario->initial_bank_deg);
    derive_initial_orbit(sim);
    /* The scenario is authoritative. In particular, Cartesian checkpoints
     * must retain their recorded velocity instead of being redirected to KSC. */
    sim_apply_deorbit_initial(sim);
    Vec3 vair=v3_sub(sim->state.velocity_i_mps,world_atmosphere_velocity_i(&sim->world,sim->state.position_i_m));
    sim->state.body_q_i=attitude_body_quat(sim->state.position_i_m,vair,sim->state.attitude.aoa_rad,sim->state.attitude.bank_rad);
}
void sim_apply_deorbit_initial(Simulation *sim){
    /* Recorded delta-v metadata is a legacy open-loop fallback. When a scenario
       exposes an orbital engine, production Guidance owns the burn through the
       normal throttle command path; pre-applying the recorded impulse here would
       double-count deorbit before createPlan/engage can execute it. */
    if(sim->scenario.orbital_engine_available_thrust_n>0.0)return;
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
    if(getenv("SHUTTLESIM_COMMAND_DIAG")){
        fprintf(stderr,"SHUTTLESIM_CMD t=%.3f attitude=%d aoa=%.3f bank=%.3f gear=%d brakes=%d airbrakes=%d wheel=%.3f throttle=%.4f step=%d pause=%d resume=%d\n",
            sim?sim->state.sim_elapsed_s:0.0,c->has_attitude,c->aoa_deg,c->bank_deg,
            c->has_gear?c->gear_down:-1,c->has_brakes?c->brakes:-1,
            c->has_airbrakes?c->airbrakes:-1,c->has_wheel_steering?c->wheel_steering:(sim?sim->state.wheel_steering:0.0),
            c->has_throttle?c->throttle:(sim?sim->commanded_throttle:0.0),
            c->step,c->pause,c->resume);
    }
    if(c->has_attitude) sim_set_attitude(sim,c->aoa_deg,c->bank_deg);
    if(c->has_inputs) attitude_set_inputs(&sim->state.attitude,c->pitch_input,c->roll_input,c->yaw_input);
    if(c->has_gear) sim->state.gear_down=c->gear_down;
    if(c->has_brakes) sim->state.brakes=c->brakes;
    if(c->has_airbrakes) sim->state.airbrakes=c->airbrakes;
    if(c->has_wheel_steering) sim->state.wheel_steering=fmin(1.0,fmax(-1.0,c->wheel_steering));
    if(c->has_throttle) sim->commanded_throttle=fmin(1.0,fmax(0.0,c->throttle));
    if(c->pause) sim->paused=true;
    if(c->resume) sim->paused=false;
}

static void ground_step(Simulation *sim,double dt){
    SimState *s=&sim->state;KerbinWorld *w=&sim->world;Vec3 up=v3_normalized(s->position_i_m);
    LocalFrame lf=world_local_frame_i(w,s->position_i_m,s->ut);
    Vec3 atmosphere=world_atmosphere_velocity_i(w,s->position_i_m);
    Vec3 surface=v3_sub(s->velocity_i_mps,atmosphere);
    surface=v3_sub(surface,v3_scale(up,v3_dot(surface,up)));double speed=v3_norm(surface);
    double radius=v3_norm(s->position_i_m);
    double g=w->mu_m3_s2/(radius*radius);
    s->ground_lateral_accel_mps2=0.0;
    if(speed>0.5&&fabs(s->wheel_steering)>1e-6){
        double max_lat=sim->ground_max_lateral_accel_g*g;
        double max_yaw=fmin(sim->ground_max_yaw_rate_rad_s,max_lat/fmax(speed,1.0));
        double yaw=s->wheel_steering*max_yaw*dt;
        double vn=v3_dot(surface,lf.north),ve=v3_dot(surface,lf.east);
        double rn=vn*cos(yaw)-ve*sin(yaw),re=vn*sin(yaw)+ve*cos(yaw);
        surface=v3_add(v3_scale(lf.north,rn),v3_scale(lf.east,re));
        s->ground_lateral_accel_mps2=fabs(speed*s->wheel_steering*max_yaw);
    }
    Vec3 steered_velocity=v3_add(surface,atmosphere);
    AeroForces ground_aero=aero_compute(w,&sim->aero,s->position_i_m,steered_velocity,s->ut,s->mass_kg,s->attitude.aoa_rad,s->attitude.bank_rad);
    double airbrake_decel=(s->airbrakes&&s->mass_kg>0.0)?ground_aero.dynamic_pressure_pa*sim->ground_airbrake_cda_m2/s->mass_kg:0.0;
    if(!isfinite(airbrake_decel)||airbrake_decel<0.0)airbrake_decel=0.0;
    s->ground_airbrake_decel_mps2=airbrake_decel;
    double mu=s->brakes?sim->brake_mu:sim->rolling_mu;
    double ns=fmax(0.0,speed-(mu*g+airbrake_decel)*dt);
    if(speed>0.0)surface=v3_scale(surface,ns/speed);else surface=v3_scale(surface,0.0);
    s->velocity_i_mps=v3_add(surface,atmosphere);
    s->position_i_m=v3_add(s->position_i_m,v3_scale(s->velocity_i_mps,dt));
    /* Constrain the actual body contact point, not the CG, to the runway.
       The KSC runway is a finite flat collider; using a constant radial
       altitude makes it curve by metres over its length and sinks the CG
       into the deck at touchdown. */
    RunwayPlane plane=runway_plane_at(sim,s->ut+dt);
    Vec3 primary_b=primary_contact_b(s),tail_b=ss_v3(-18.0,0.0,1.48);
    Vec3 primary=v3_add(s->position_i_m,quat_rotate(s->body_q_i,primary_b));
    Vec3 tail=v3_add(s->position_i_m,quat_rotate(s->body_q_i,tail_b));
    double pa,pc,ph,ta,tc,th;
    runway_plane_coords(&plane,primary,&pa,&pc,&ph);
    runway_plane_coords(&plane,tail,&ta,&tc,&th);
    bool primary_runway=runway_contains(&sim->runway,pa,pc);
    bool tail_runway=runway_contains(&sim->runway,ta,tc);
    Vec3 support_normal;
    double support_gap;
    if(primary_runway||tail_runway){
        if(primary_runway&&(!tail_runway||ph<=th)){support_gap=ph;support_normal=plane.normal_i;}
        else {support_gap=th;support_normal=plane.normal_i;}
    }else{
        double primary_gap=v3_norm(primary)-w->radius_m;
        double tail_gap=v3_norm(tail)-w->radius_m;
        Vec3 support_point=primary_gap<=tail_gap?primary:tail;
        support_gap=fmin(primary_gap,tail_gap);
        support_normal=v3_normalized(support_point);
    }
    if(support_gap>0.5){
        s->on_ground=false;
        return;
    }
    s->position_i_m=v3_sub(s->position_i_m,v3_scale(support_normal,support_gap));
    /* Preserve ground-relative speed while transporting it to the corrected
       contact plane. */
    surface=v3_sub(surface,v3_scale(support_normal,v3_dot(surface,support_normal)));
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
    if(s->on_ground){ground_step(sim,dt);s->ut+=dt;s->sim_elapsed_s+=dt;return;}

    Vec3 previous_position_i=s->position_i_m;
    Quat previous_body_q=s->body_q_i;
    double previous_ut=s->ut;

    /* The shared plant preserves KSP's force-at-tick-start, velocity, position,
       then attitude ordering. In particular, do not RK4 this force field. */
    double engine_thrust=sim->scenario.orbital_engine_available_thrust_n*
        sim->commanded_throttle;
    FlightAirborneState airborne={
        .position_i_m=s->position_i_m,
        .velocity_i_mps=s->velocity_i_mps,
        .attitude=s->attitude,
        .body_q_i=s->body_q_i,
        .aero=s->aero
    };
    FlightAirborneStepInput step_input={
        .mass_kg=s->mass_kg,
        .ut=s->ut,
        .dt_s=dt,
        .burn_accel_mps2=(sim->deorbit_burn_remaining_s>0)
            ? sim->deorbit_burn_accel_mps2 : 0.0,
        .engine_thrust_n=engine_thrust
    };
    flight_physics_airborne_step(&sim->world,&sim->aero,&step_input,&airborne);
    s->position_i_m=airborne.position_i_m;
    s->velocity_i_mps=airborne.velocity_i_mps;
    s->attitude=airborne.attitude;
    s->body_q_i=airborne.body_q_i;
    s->aero=airborne.aero;
    if(s->gear_down&&s->aero.airspeed_mps>1.0){
        /* Extended landing gear: parasite drag along the air-relative velocity. */
        const double gear_cda_m2=5.0;
        Vec3 vair=v3_sub(s->velocity_i_mps,world_atmosphere_velocity_i(&sim->world,s->position_i_m));
        double dv=s->aero.dynamic_pressure_pa*gear_cda_m2/s->mass_kg*dt;
        double va=v3_norm(vair);
        if(va>1e-6)s->velocity_i_mps=v3_sub(s->velocity_i_mps,v3_scale(vair,fmin(dv,va)/va));
    }
    if(s->airbrakes&&s->aero.airspeed_mps>1.0){
        /* Deployed speedbrake: parasite drag along the air-relative velocity. */
        Vec3 vair=v3_sub(s->velocity_i_mps,world_atmosphere_velocity_i(&sim->world,s->position_i_m));
        double dv=s->aero.dynamic_pressure_pa*sim->ground_airbrake_cda_m2/s->mass_kg*dt;
        double va=v3_norm(vair);
        if(va>1e-6)s->velocity_i_mps=v3_sub(s->velocity_i_mps,v3_scale(vair,fmin(dv,va)/va));
        s->aero.drag_n+=s->aero.dynamic_pressure_pa*sim->ground_airbrake_cda_m2;
    }

    if(sim->deorbit_burn_remaining_s>0){
        sim->deorbit_burn_remaining_s=fmax(0.0,sim->deorbit_burn_remaining_s-dt);
    }
    s->ut+=dt;
    s->sim_elapsed_s+=dt;
    /* Continuous collision against the finite flat KSC runway top surface.
       Evaluate body contact points at both ends of the fixed tick so entering
       the runway footprint below deck height does not behave like a 70 m wall. */
    RunwayPlane prev_plane=runway_plane_at(sim,previous_ut);
    RunwayPlane cur_plane=runway_plane_at(sim,s->ut);
    Vec3 primary_b=primary_contact_b(s),tail_b=ss_v3(-18.0,0.0,1.48);
    Vec3 prev_primary=v3_add(previous_position_i,quat_rotate(previous_body_q,primary_b));
    Vec3 cur_primary=v3_add(s->position_i_m,quat_rotate(s->body_q_i,primary_b));
    Vec3 prev_tail=v3_add(previous_position_i,quat_rotate(previous_body_q,tail_b));
    Vec3 cur_tail=v3_add(s->position_i_m,quat_rotate(s->body_q_i,tail_b));

    double pt=DBL_MAX,pa=0.0,pc=0.0,ph=0.0;
    double tt=DBL_MAX,ta=0.0,tc=0.0,th=0.0;
    bool primary_runway=runway_contact_crossing(sim,prev_plane,cur_plane,
                                                prev_primary,cur_primary,
                                                &pt,&pa,&pc,&ph);
    bool tail_runway=runway_contact_crossing(sim,prev_plane,cur_plane,
                                             prev_tail,cur_tail,
                                             &tt,&ta,&tc,&th);

    bool collision=false,on_runway=false,hit_primary=false;
    double hit_gap=0.0,hit_along=0.0,hit_cross=0.0;
    Vec3 hit_normal=cur_plane.normal_i;
    if(primary_runway||tail_runway){
        hit_primary=primary_runway&&(!tail_runway||pt<=tt);
        collision=true;on_runway=true;
        hit_gap=hit_primary?ph:th;
        hit_along=hit_primary?pa:ta;
        hit_cross=hit_primary?pc:tc;
    }else{
        double pg0=v3_norm(prev_primary)-sim->world.radius_m;
        double pg1=v3_norm(cur_primary)-sim->world.radius_m;
        double tg0=v3_norm(prev_tail)-sim->world.radius_m;
        double tg1=v3_norm(cur_tail)-sim->world.radius_m;
        double ptoi=DBL_MAX,ttoi=DBL_MAX;
        bool primary_terrain=false,tail_terrain=false;
        if(pg0>0.0&&pg1<=0.0){primary_terrain=true;ptoi=pg0/(pg0-pg1);}
        else if(pg0<=0.0&&pg1<=0.0){primary_terrain=true;ptoi=0.0;}
        if(tg0>0.0&&tg1<=0.0){tail_terrain=true;ttoi=tg0/(tg0-tg1);}
        else if(tg0<=0.0&&tg1<=0.0){tail_terrain=true;ttoi=0.0;}
        if(primary_terrain||tail_terrain){
            hit_primary=primary_terrain&&(!tail_terrain||ptoi<=ttoi);
            double toi=hit_primary?ptoi:ttoi;
            Vec3 p0=hit_primary?prev_primary:prev_tail;
            Vec3 p1=hit_primary?cur_primary:cur_tail;
            Vec3 impact=v3_add(p0,v3_scale(v3_sub(p1,p0),toi));
            collision=true;
            hit_gap=hit_primary?pg1:tg1;
            hit_normal=v3_normalized(p1);
            double vertical_unused=0.0;
            ss_runway_coordinates(&sim->world,&sim->runway,impact,
                                  previous_ut+toi*dt,
                                  &hit_along,&hit_cross,&vertical_unused);
        }
    }

    if(collision){
        Vec3 surf=v3_sub(s->velocity_i_mps,
                         world_atmosphere_velocity_i(&sim->world,s->position_i_m));
        double vv=v3_dot(surf,hit_normal);
        Vec3 horiz=v3_sub(surf,v3_scale(hit_normal,vv));
        if(!s->touchdown_seen){
            s->touchdown_seen=true;
            s->on_runway_at_touchdown=on_runway;
            s->touchdown_sink_mps=fmax(0.0,-vv);
            s->touchdown_speed_mps=v3_norm(horiz);
            s->touchdown_along_m=hit_along;
            s->touchdown_cross_m=hit_cross;
            s->touchdown_gear=s->gear_down&&hit_primary;
            s->tail_strike=!hit_primary;
            Vec3 fwd=quat_rotate(s->body_q_i,ss_v3(1,0,0));
            s->touchdown_pitch_deg=rad2deg(asin(ss_clampd(v3_dot(fwd,hit_normal),
                                                         -1.0,1.0)));
        }
        if(hit_gap<0.0)
            s->position_i_m=v3_sub(s->position_i_m,
                                   v3_scale(hit_normal,hit_gap));
        Vec3 atmosphere=world_atmosphere_velocity_i(&sim->world,s->position_i_m);
        Vec3 relative=v3_sub(s->velocity_i_mps,atmosphere);
        double vn=v3_dot(relative,hit_normal);
        if(vn<0.0)relative=v3_sub(relative,v3_scale(hit_normal,vn));
        s->velocity_i_mps=v3_add(relative,atmosphere);
        s->on_ground=true;
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
    AtmosphereSample atm=world_atmosphere_sample_state(&sim->world,s->position_i_m,s->ut);
    double along,cross,vertical;ss_runway_coordinates(&sim->world,&sim->runway,s->position_i_m,s->ut,&along,&cross,&vertical);
    snprintf(out,n,"{\"schema\":1,\"type\":\"telemetry\",\"source\":\"sim\",\"scenario\":\"%s\",\"ut\":%.6f,\"sim_time\":%.6f,\"sim_rate\":%.2f,"
             "\"world\":{\"radius_m\":%.9g,\"mu_m3_s2\":%.12g,\"rotation_rate_rad_s\":%.12g,\"atmosphere_top_m\":%.9g,\"rotation_phase_rad_at_ut0\":%.12g},"
             "\"position\":{\"x\":%.6f,\"y\":%.6f,\"z\":%.6f,\"lat_deg\":%.9f,\"lon_deg\":%.9f,\"altitude_m\":%.3f},"
             "\"velocity\":{\"x_mps\":%.6f,\"y_mps\":%.6f,\"z_mps\":%.6f,\"inertial_mps\":%.3f,\"surface_mps\":%.3f,\"air_mps\":%.3f,\"horizontal_mps\":%.3f,\"vertical_mps\":%.3f,\"flight_path_angle_deg\":%.6f},"
             "\"attitude\":{\"aoa_deg\":%.4f,\"bank_deg\":%.4f,\"heading_deg\":%.4f,\"cmd_aoa_deg\":%.4f,\"cmd_bank_deg\":%.4f,\"requested_aoa_deg\":%.4f,\"requested_bank_deg\":%.4f,"
             "\"aoa_rate_deg_s\":%.6f,\"bank_rate_deg_s\":%.6f,\"sideslip_deg\":%.6f,\"direct_control\":%s,"
             "\"pitch_wn_s_inv\":%.9g,\"pitch_zeta\":%.9g,\"roll_wn_s_inv\":%.9g,\"roll_zeta\":%.9g,"
             "\"max_pitch_rate_deg_s\":%.9g,\"max_roll_rate_deg_s\":%.9g,"
             "\"max_pitch_accel_deg_s2\":%.9g,\"max_roll_accel_deg_s2\":%.9g,"
             "\"q_w\":%.9f,\"q_x\":%.9f,\"q_y\":%.9f,\"q_z\":%.9f},"
             "\"atmosphere\":{\"density_kg_m3\":%.12g,\"pressure_pa\":%.9g,\"temperature_k\":%.6f,\"speed_of_sound_mps\":%.6f},"
             "\"aero\":{\"mach\":%.6f,\"q_pa\":%.6f,\"lift_n\":%.6f,\"drag_n\":%.6f},"
             "\"vehicle\":{\"mass_kg\":%.6f,\"available_thrust_n\":%.6f,\"current_thrust_n\":%.6f,\"throttle\":%.9f},"
             "\"runway\":{\"along_m\":%.3f,\"cross_m\":%.3f,\"vertical_m\":%.3f,\"length_m\":%.3f,\"width_m\":%.3f,\"heading_deg\":%.6f},"
             "\"ground\":{\"gear_down\":%s,\"brakes\":%s,\"airbrakes\":%s,\"wheel_steering\":%.6f,\"steering_lateral_accel_mps2\":%.6f,\"airbrake_decel_mps2\":%.6f,\"on_ground\":%s,\"stopped\":%s,\"touchdown_seen\":%s,\"on_runway_touchdown\":%s}}",
             sim->scenario.name,s->ut,s->sim_elapsed_s,rate,sim->world.radius_m,sim->world.mu_m3_s2,sim->world.rotation_rate_rad_s,sim->world.atmosphere_top_m,sim->world.rotation_phase_rad_at_ut0,s->position_i_m.x,s->position_i_m.y,s->position_i_m.z,rad2deg(l.lat_rad),rad2deg(l.lon_rad),l.altitude_m,
             s->velocity_i_mps.x,s->velocity_i_mps.y,s->velocity_i_mps.z,v3_norm(s->velocity_i_mps),v3_norm(surf),v3_norm(air),horizontal,vu,rad2deg(fpa),
             rad2deg(s->attitude.aoa_rad),rad2deg(s->attitude.bank_rad),rad2deg(heading),rad2deg(s->attitude.cmd_aoa_rad),rad2deg(s->attitude.cmd_bank_rad),rad2deg(s->attitude.requested_aoa_rad),rad2deg(s->attitude.requested_bank_rad),
             rad2deg(s->attitude.aoa_rate_rad_s),rad2deg(s->attitude.bank_rate_rad_s),
             rad2deg(s->attitude.sideslip_rad),s->attitude.direct?"true":"false",
             s->attitude.pitch_wn,s->attitude.pitch_zeta,s->attitude.roll_wn,s->attitude.roll_zeta,
             rad2deg(s->attitude.max_pitch_rate_rad_s),rad2deg(s->attitude.max_roll_rate_rad_s),
             rad2deg(s->attitude.max_pitch_accel_rad_s2),rad2deg(s->attitude.max_roll_accel_rad_s2),
             s->body_q_i.w,s->body_q_i.x,s->body_q_i.y,s->body_q_i.z,
             atm.density_kg_m3,atm.pressure_pa,atm.temperature_k,atm.speed_of_sound_mps,
             s->aero.mach,s->aero.dynamic_pressure_pa,s->aero.lift_n,s->aero.drag_n,s->mass_kg,
             sim->scenario.orbital_engine_available_thrust_n,
             sim->scenario.orbital_engine_available_thrust_n*sim->commanded_throttle,
             sim->commanded_throttle,
             along,cross,vertical,sim->runway.length_m,sim->runway.width_m,rad2deg(sim->runway.heading_rad),
             s->gear_down?"true":"false",s->brakes?"true":"false",s->airbrakes?"true":"false",s->wheel_steering,
             s->ground_lateral_accel_mps2,s->ground_airbrake_decel_mps2,s->on_ground?"true":"false",stopped?"true":"false",
             s->touchdown_seen?"true":"false",s->on_runway_at_touchdown?"true":"false");
}
void sim_build_summary_json(const Simulation *sim,char *out,size_t n){
    const SimState *s=&sim->state;LLA l=world_lla(&sim->world,s->position_i_m,s->ut);double along,cross,vertical;ss_runway_coordinates(&sim->world,&sim->runway,s->position_i_m,s->ut,&along,&cross,&vertical);
    snprintf(out,n,"{\"scenario\":\"%s\",\"sim_time_s\":%.3f,\"final_altitude_m\":%.3f,\"touchdown\":%s,\"on_runway\":%s,\"touchdown_sink_mps\":%.3f,\"touchdown_speed_mps\":%.3f,\"touchdown_along_m\":%.3f,\"touchdown_cross_m\":%.3f,\"touchdown_gear\":%s,\"tail_strike\":%s,\"touchdown_pitch_deg\":%.3f,\"final_along_m\":%.3f,\"final_cross_m\":%.3f}",
             sim->scenario.name,s->sim_elapsed_s,l.altitude_m,s->touchdown_seen?"true":"false",s->on_runway_at_touchdown?"true":"false",s->touchdown_sink_mps,s->touchdown_speed_mps,s->touchdown_along_m,s->touchdown_cross_m,s->touchdown_gear?"true":"false",s->tail_strike?"true":"false",s->touchdown_pitch_deg,along,cross);
}
