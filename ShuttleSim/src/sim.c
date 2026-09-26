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

/* ---------------------------------------------------------------------------
   Landing-gear contact geometry (aerospace body frame: +X fwd, +Y right, +Z
   down, origin at the CoM).
   - Main wheels: MEASURED in KSP (quickload): contact patch 3.1535 m below and
     0.9058 m aft of the CoM (both main wheels lumped on the centreline).
   - Nose wheel: its height is set so the vehicle rests at the MEASURED stance
     (pitch -4.3 deg, CoM ~3.06 m above the deck; 22 live stops 2026-09-24/25).
     Its x-station (10 m forward) is a SURROGATE; it only sets the nose/main load
     split and the nose-slam lever arm.
   - Tail point: SURROGATE placed for a 15 deg mains-on-deck tail-strike pitch.
   - Gear retracted: belly points 2 m shallower (SURROGATE).
   Struts are spring-dampers sized for ~0.15 m static compression and ~0.6 of
   critical heave damping; they are a SURROGATE for KSP wheel colliders.
   --------------------------------------------------------------------------- */
enum { CP_MAIN=0, CP_NOSE=1, CP_TAIL=2, CP_COUNT=3 };
#define GEAR_MAIN_X (-0.9058)
#define GEAR_MAIN_Z (3.1535)
#define GEAR_NOSE_X (10.0)
#define GEAR_STANCE_PITCH_RAD (-4.3*3.14159265358979323846/180.0)
#define GEAR_STATIC_COMPRESSION_M 0.15
#define GEAR_HEAVE_DAMPING_RATIO 0.6
#define GROUND_PLATEAU_HALF_SIZE_M 5000.0

static double gear_nose_z(void){
    double c=cos(GEAR_STANCE_PITCH_RAD),sn=sin(GEAR_STANCE_PITCH_RAD);
    double rest_height=GEAR_MAIN_Z*c-GEAR_MAIN_X*sn;
    return (rest_height+GEAR_NOSE_X*sn)/c;
}

static Vec3 contact_point_b(const SimState *s,int which){
    double belly=s->gear_down?0.0:2.0;
    switch(which){
    case CP_MAIN: return ss_v3(GEAR_MAIN_X,0.0,GEAR_MAIN_Z-belly);
    case CP_NOSE: return ss_v3(GEAR_NOSE_X,0.0,gear_nose_z()-belly);
    default: {
        double tail_x=-18.0;
        double tail_z=GEAR_MAIN_Z-(GEAR_MAIN_X-tail_x)*tan(15.0*3.14159265358979323846/180.0);
        return ss_v3(tail_x,0.0,tail_z);
    }
    }
}

/* Ground surface: the KSC runway is a finite flat deck on a plateau at runway
   elevation (SURROGATE: the plateau is the runway plane extended to
   +/-GROUND_PLATEAU_HALF_SIZE_M); beyond it the terrain is sea level.  Leaving
   the paved rectangle is a runway departure, not a fall. */
static double ground_height(const Simulation *sim,const RunwayPlane *plane,Vec3 point,
                            Vec3 *normal,bool *on_runway,double *along,double *cross){
    double a,c,h;runway_plane_coords(plane,point,&a,&c,&h);
    if(along)*along=a;if(cross)*cross=c;
    double centre=0.5*sim->runway.length_m;
    /* |h| bound: points elsewhere on the planet can project into the box. */
    if(fabs(a-centre)<=GROUND_PLATEAU_HALF_SIZE_M&&fabs(c)<=GROUND_PLATEAU_HALF_SIZE_M&&fabs(h)<2000.0){
        if(normal)*normal=plane->normal_i;
        if(on_runway)*on_runway=runway_contains(&sim->runway,a,c);
        return h;
    }
    if(normal)*normal=v3_normalized(point);
    if(on_runway)*on_runway=false;
    return v3_norm(point)-sim->world.radius_m;
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
    sim->gear_cda_m2=0.0;
    sim->pitch_inertia_kg_m2=40000.0*5.5*5.5;
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

/* Rigid-body pitch after the first main-wheel contact.  Aerodynamic pitch
   acceleration uses the identified direct-control coefficients (already per
   unit inertia); gear moments use the surrogate pitch inertia. */
static double released_pitch_aero_accel(const AttitudeModel *a,double q_pa,double aoa_rad,
                                        double rate_rad_s){
    double kpa=fmax(0.0,q_pa)/1000.0;
    double ctrl=fmin(a->direct_pitch_accel_max,a->direct_pitch_accel_per_kpa*kpa);
    double stiffness=a->direct_pitch_stiffness_per_kpa*kpa;
    double damping=2.0*a->direct_pitch_damping_ratio*sqrt(fmax(stiffness,0.0));
    /* In ideal-servo mode the stick is released by the post-mains contract;
       in direct mode the FCS stick input is honoured (it must also be zero). */
    double input=a->direct?a->input_pitch:0.0;
    return ctrl*input-stiffness*(aoa_rad-a->direct_trim_aoa_rad)-damping*rate_rad_s;
}

static double surface_fpa(const Simulation *sim,Vec3 up){
    const SimState *s=&sim->state;
    Vec3 surf=v3_sub(s->velocity_i_mps,world_atmosphere_velocity_i(&sim->world,s->position_i_m));
    double sp=v3_norm(surf);
    return sp>1e-6?asin(ss_clampd(v3_dot(surf,up)/sp,-1.0,1.0)):0.0;
}

static double body_pitch(const SimState *s,Vec3 up){
    Vec3 fwd=quat_rotate(s->body_q_i,ss_v3(1,0,0));
    return asin(ss_clampd(v3_dot(fwd,up),-1.0,1.0));
}

/* After release the body attitude is the rigid-body pitch about the retained
   ground heading, not a velocity-relative construction: the velocity direction
   is meaningless as the vehicle stops.  The heading follows the ground track
   while it is well defined (the wheels steer the track). */
static void released_update_body(Simulation *sim,Vec3 up){
    SimState *s=&sim->state;
    Vec3 st=v3_sub(s->velocity_i_mps,world_atmosphere_velocity_i(&sim->world,s->position_i_m));
    Vec3 horizontal=v3_sub(st,v3_scale(up,v3_dot(st,up)));
    if(v3_norm(horizontal)>2.0)s->released_heading_i=v3_normalized(horizontal);
    Vec3 hdg=s->released_heading_i;
    Vec3 body_fwd=v3_normalized(v3_add(v3_scale(hdg,cos(s->released_pitch_rad)),
                                       v3_scale(up,sin(s->released_pitch_rad))));
    Vec3 right=v3_normalized(v3_cross(hdg,up));
    Vec3 body_up=v3_normalized(v3_cross(right,body_fwd));
    body_up=v3_rotate_axis(body_up,body_fwd,s->attitude.bank_rad);
    Vec3 body_right=v3_normalized(v3_cross(body_fwd,body_up));
    s->body_q_i=quat_from_basis(body_fwd,body_right,v3_scale(body_up,-1.0));
}

/* Gear/structure contact forces, friction, steering and touchdown/rollout
   classification.  Called once per physics tick after the airborne
   integration, so aerodynamic lift unloads the gear naturally. */
static void contact_step(Simulation *sim,double dt){
    SimState *s=&sim->state;KerbinWorld *w=&sim->world;
    RunwayPlane plane=runway_plane_at(sim,s->ut);
    if(s->pitch_released)released_update_body(sim,plane.normal_i);
    double radius=v3_norm(s->position_i_m);
    double g=w->mu_m3_s2/(radius*radius);
    double weight=s->mass_kg*g;
    double k_main=weight/GEAR_STATIC_COMPRESSION_M;
    double k_nose=0.25*k_main;
    double c_main=2.0*GEAR_HEAVE_DAMPING_RATIO*sqrt(k_main*s->mass_kg);
    double c_nose=2.0*GEAR_HEAVE_DAMPING_RATIO*sqrt(k_nose*s->mass_kg);
    double k_tail=2.0*k_main,c_tail=c_main;

    Vec3 atmosphere=world_atmosphere_velocity_i(w,s->position_i_m);
    Vec3 surf=v3_sub(s->velocity_i_mps,atmosphere);
    Vec3 total=ss_v3(0,0,0);double pitch_moment=0.0;
    Vec3 body_right=quat_rotate(s->body_q_i,ss_v3(0,1,0));
    bool loaded[CP_COUNT]={false,false,false};bool wheel_off_runway=false;
    Vec3 contact_normal=plane.normal_i;
    double deepest=0.0;
    for(int i=0;i<CP_COUNT;i++){
        Vec3 rb=contact_point_b(s,i);
        Vec3 r=quat_rotate(s->body_q_i,rb);
        Vec3 point=v3_add(s->position_i_m,r);
        Vec3 n;bool on_rw=false;double along=0,cross=0;
        double h=ground_height(sim,&plane,point,&n,&on_rw,&along,&cross);
        double hdot=s->contact_prev_valid?(h-s->contact_prev_height_m[i])/dt:0.0;
        s->contact_prev_height_m[i]=h;
        s->contact_load_n[i]=0.0;
        if(h>=0.0)continue;
        double pen=-h;deepest=fmax(deepest,pen);
        double k=i==CP_MAIN?k_main:i==CP_NOSE?k_nose:k_tail;
        double c=i==CP_MAIN?c_main:i==CP_NOSE?c_nose:c_tail;
        double normal_force=fmax(0.0,k*pen-c*hdot);
        if(!(normal_force>0.0))continue;
        loaded[i]=true;contact_normal=n;
        s->contact_load_n[i]=normal_force;
        if(!on_rw&&(i==CP_MAIN||i==CP_NOSE))wheel_off_runway=true;
        /* Tangential force: rolling/brake friction on wheels (brakes act on
           the mains), sliding friction for belly/tail/retracted gear. */
        Vec3 tangential=v3_sub(surf,v3_scale(n,v3_dot(surf,n)));
        double ts=v3_norm(tangential);
        double mu=sim->rolling_mu;
        bool wheel=s->gear_down&&i!=CP_TAIL;
        if(!wheel)mu=0.5;
        else if(i==CP_MAIN&&s->brakes)mu=sim->brake_mu;
        Vec3 friction=ts>1e-6?v3_scale(tangential,-mu*normal_force/ts):ss_v3(0,0,0);
        Vec3 f=v3_add(v3_scale(n,normal_force),friction);
        total=v3_add(total,f);
        pitch_moment+=v3_dot(v3_cross(r,f),body_right);
        if(!s->touchdown_seen){
            double vv=v3_dot(surf,n);
            Vec3 horiz=v3_sub(surf,v3_scale(n,vv));
            s->touchdown_seen=true;
            s->on_runway_at_touchdown=on_rw;
            s->touchdown_sink_mps=fmax(0.0,-vv);
            s->touchdown_speed_mps=v3_norm(horiz);
            s->touchdown_along_m=along;
            s->touchdown_cross_m=cross;
            s->touchdown_gear=s->gear_down&&i==CP_MAIN;
            s->tail_strike=i==CP_TAIL;
            s->touchdown_pitch_deg=rad2deg(body_pitch(s,n));
        }
        if(i==CP_MAIN&&!s->main_touchdown_seen){
            s->main_touchdown_seen=true;s->main_touchdown_ut=s->ut;
        }
        if(i==CP_NOSE&&!s->nose_touchdown_seen){
            s->nose_touchdown_seen=true;
            s->nose_touchdown_sink_mps=fmax(0.0,-hdot);
            s->nose_touchdown_delay_s=s->main_touchdown_seen?s->ut-s->main_touchdown_ut:-1.0;
        }
        if(i==CP_TAIL)s->tail_strike=true;
        /* Crash: a contact normal speed or ground speed far outside anything a
           gear or airframe survives.  Thresholds are classification limits,
           not tuned values: 25 m/s is ~8x the qualification sink limit and
           200 m/s is ~2.7x touchdown speed. */
        double vn=-v3_dot(surf,n);
        if(!s->crashed&&(vn>25.0||v3_norm(v3_sub(surf,v3_scale(n,v3_dot(surf,n))))>200.0)){
            s->crashed=true;
        }
    }
    if(s->crashed){
        s->velocity_i_mps=world_atmosphere_velocity_i(w,s->position_i_m);
        s->on_ground=true;s->stopped=true;
        s->contact_prev_valid=false;
        return;
    }
    s->contact_prev_valid=true;
    double load=total.x*contact_normal.x+total.y*contact_normal.y+total.z*contact_normal.z;
    if(weight>0.0)s->max_gear_load_g=fmax(s->max_gear_load_g,load/weight);
    bool any=loaded[CP_MAIN]||loaded[CP_NOSE]||loaded[CP_TAIL];
    s->main_contact=loaded[CP_MAIN];
    s->nose_contact=loaded[CP_NOSE];
    if(s->touchdown_seen&&!any){
        s->airborne_after_contact_s+=dt;
    }else{
        if(any&&s->airborne_after_contact_s>0.1)s->bounce_count++;
        s->airborne_after_contact_s=0.0;
    }
    s->on_ground=any;
    if(any){
        /* A collider cannot be penetrated far: remove numerical blow-through
           (only reachable in crash-grade impacts). */
        if(deepest>0.5)s->position_i_m=v3_add(s->position_i_m,v3_scale(contact_normal,deepest-0.5));
        s->velocity_i_mps=v3_add(s->velocity_i_mps,v3_scale(total,dt/s->mass_kg));
        if(wheel_off_runway&&!s->runway_departure){
            double a,c,h;runway_plane_coords(&plane,s->position_i_m,&a,&c,&h);(void)h;
            s->runway_departure=true;s->departure_along_m=a;s->departure_cross_m=c;
            Vec3 sv=v3_sub(s->velocity_i_mps,atmosphere);
            s->departure_speed_mps=v3_norm(v3_sub(sv,v3_scale(contact_normal,v3_dot(sv,contact_normal))));
        }
        /* Nose-wheel steering rotates the ground track, bounded by lateral
           acceleration, yaw rate and the load actually on the wheels. */
        surf=v3_sub(s->velocity_i_mps,atmosphere);
        Vec3 tangential=v3_sub(surf,v3_scale(contact_normal,v3_dot(surf,contact_normal)));
        double speed=v3_norm(tangential);
        s->ground_lateral_accel_mps2=0.0;
        if(s->gear_down&&loaded[CP_NOSE]&&speed>0.5&&fabs(s->wheel_steering)>1e-6){
            double load_fraction=ss_clampd(load/fmax(weight,1.0),0.0,1.0);
            double max_lat=sim->ground_max_lateral_accel_g*g*load_fraction;
            double max_yaw=fmin(sim->ground_max_yaw_rate_rad_s,max_lat/fmax(speed,1.0));
            double yaw=s->wheel_steering*max_yaw*dt;
            Vec3 t=v3_rotate_axis(tangential,contact_normal,-yaw);
            s->velocity_i_mps=v3_add(v3_add(t,v3_scale(contact_normal,v3_dot(surf,contact_normal))),atmosphere);
            s->ground_lateral_accel_mps2=fabs(speed*s->wheel_steering*max_yaw);
        }
        if(s->brakes&&speed<0.3&&s->gear_down){
            s->velocity_i_mps=v3_add(v3_scale(contact_normal,v3_dot(surf,contact_normal)),atmosphere);
            speed=0.0;
        }
        if(speed<0.3&&!s->stopped){
            double a,c,h;runway_plane_coords(&plane,s->position_i_m,&a,&c,&h);(void)h;
            s->stopped=true;s->stop_along_m=a;s->stop_cross_m=c;
        }
    }
    /* Post-mains contract: first main-wheel contact releases pitch for good. */
    Vec3 up=plane.normal_i;
    if(loaded[CP_MAIN]&&s->gear_down&&!s->pitch_released){
        s->pitch_released=true;
        s->released_pitch_rad=body_pitch(s,up);
        s->released_prev_fpa_rad=surface_fpa(sim,up);
        s->released_pitch_rate_rad_s=s->attitude.aoa_rate_rad_s;
        Vec3 fwd=quat_rotate(s->body_q_i,ss_v3(1,0,0));
        s->released_heading_i=v3_normalized(v3_sub(fwd,v3_scale(up,v3_dot(fwd,up))));
    }
    if(s->pitch_released){
        double fpa=surface_fpa(sim,up);
        double fpa_rate=(fpa-s->released_prev_fpa_rad)/dt;
        s->released_prev_fpa_rad=fpa;
        double aoa=s->released_pitch_rad-fpa;
        double aero_q=s->aero.dynamic_pressure_pa;
        double acc=released_pitch_aero_accel(&s->attitude,aero_q,aoa,s->released_pitch_rate_rad_s)+
            pitch_moment/sim->pitch_inertia_kg_m2;
        s->released_pitch_rate_rad_s+=acc*dt;
        s->released_pitch_rad+=s->released_pitch_rate_rad_s*dt;
        s->attitude.aoa_rad=s->released_pitch_rad-fpa;
        s->attitude.aoa_rate_rad_s=s->released_pitch_rate_rad_s-fpa_rate;
        s->attitude.cmd_aoa_rad=s->attitude.aoa_rad;
        s->attitude.requested_aoa_rad=s->attitude.aoa_rad;
        if(s->on_ground){
            /* Wheels on the deck level the wings. */
            s->attitude.bank_rad*=fmax(0.0,1.0-2.0*dt);
            s->attitude.bank_rate_rad_s=0.0;
            s->attitude.cmd_bank_rad=s->attitude.bank_rad;
        }
        released_update_body(sim,up);
        Vec3 st=v3_sub(s->velocity_i_mps,world_atmosphere_velocity_i(w,s->position_i_m));
        if(v3_norm(st)<2.0){
            /* Incidence is undefined when stopped; report pitch. */
            s->attitude.aoa_rad=s->released_pitch_rad;
            s->attitude.aoa_rate_rad_s=s->released_pitch_rate_rad_s;
            s->attitude.cmd_aoa_rad=s->attitude.requested_aoa_rad=s->attitude.aoa_rad;
        }
    }
    s->aero=aero_compute(w,&sim->aero,s->position_i_m,s->velocity_i_mps,s->ut,s->mass_kg,
                         s->attitude.aoa_rad,s->attitude.bank_rad);
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
    if(s->stopped&&s->on_ground){s->ut+=dt;s->sim_elapsed_s+=dt;return;}

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
    if(s->gear_down&&sim->gear_cda_m2>0.0&&s->aero.airspeed_mps>1.0){
        Vec3 vair=v3_sub(s->velocity_i_mps,world_atmosphere_velocity_i(&sim->world,s->position_i_m));
        double dv=s->aero.dynamic_pressure_pa*sim->gear_cda_m2/s->mass_kg*dt;
        double va=v3_norm(vair);
        if(va>1e-6)s->velocity_i_mps=v3_sub(s->velocity_i_mps,v3_scale(vair,fmin(dv,va)/va));
    }
    s->ground_airbrake_decel_mps2=0.0;
    if(s->airbrakes&&s->aero.airspeed_mps>1.0){
        /* Deployed speedbrake: parasite drag along the air-relative velocity. */
        Vec3 vair=v3_sub(s->velocity_i_mps,world_atmosphere_velocity_i(&sim->world,s->position_i_m));
        double dv=s->aero.dynamic_pressure_pa*sim->ground_airbrake_cda_m2/s->mass_kg*dt;
        double va=v3_norm(vair);
        if(va>1e-6)s->velocity_i_mps=v3_sub(s->velocity_i_mps,v3_scale(vair,fmin(dv,va)/va));
        s->aero.drag_n+=s->aero.dynamic_pressure_pa*sim->ground_airbrake_cda_m2;
        s->ground_airbrake_decel_mps2=dv/dt;
    }

    if(sim->deorbit_burn_remaining_s>0){
        sim->deorbit_burn_remaining_s=fmax(0.0,sim->deorbit_burn_remaining_s-dt);
    }
    s->ut+=dt;
    s->sim_elapsed_s+=dt;
    contact_step(sim,dt);
}

void sim_build_telemetry_json(const Simulation *sim,double rate,char *out,size_t n){
    const SimState *s=&sim->state;LLA l=world_lla(&sim->world,s->position_i_m,s->ut);LocalFrame lf=world_local_frame_i(&sim->world,s->position_i_m,s->ut);
    Vec3 surf=v3_sub(s->velocity_i_mps,world_atmosphere_velocity_i(&sim->world,s->position_i_m));Vec3 air=surf;
    double vn=v3_dot(surf,lf.north),ve=v3_dot(surf,lf.east),vu=v3_dot(surf,lf.up);
    double horizontal=hypot(vn,ve),surface_speed=v3_norm(surf),heading=atan2(ve,vn);if(heading<0)heading+=2*3.14159265358979323846;
    bool stopped=s->stopped;(void)surface_speed;
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
             "\"ground\":{\"gear_down\":%s,\"brakes\":%s,\"airbrakes\":%s,\"wheel_steering\":%.6f,\"steering_lateral_accel_mps2\":%.6f,\"airbrake_decel_mps2\":%.6f,\"on_ground\":%s,\"stopped\":%s,\"touchdown_seen\":%s,\"on_runway_touchdown\":%s,"
             "\"main_contact\":%s,\"nose_contact\":%s,\"pitch_released\":%s,\"main_load_n\":%.1f,\"nose_load_n\":%.1f,\"tail_load_n\":%.1f,\"bounce_count\":%d,\"runway_departure\":%s,\"pitch_deg\":%.4f}}",
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
             s->touchdown_seen?"true":"false",s->on_runway_at_touchdown?"true":"false",
             s->main_contact?"true":"false",s->nose_contact?"true":"false",s->pitch_released?"true":"false",
             s->contact_load_n[0],s->contact_load_n[1],s->contact_load_n[2],s->bounce_count,
             s->runway_departure?"true":"false",rad2deg(asin(ss_clampd(v3_dot(quat_rotate(s->body_q_i,ss_v3(1,0,0)),lf.up),-1.0,1.0))));
}
void sim_build_summary_json(const Simulation *sim,char *out,size_t n){
    const SimState *s=&sim->state;LLA l=world_lla(&sim->world,s->position_i_m,s->ut);double along,cross,vertical;ss_runway_coordinates(&sim->world,&sim->runway,s->position_i_m,s->ut,&along,&cross,&vertical);
    snprintf(out,n,"{\"scenario\":\"%s\",\"sim_time_s\":%.3f,\"final_altitude_m\":%.3f,\"touchdown\":%s,\"on_runway\":%s,\"touchdown_sink_mps\":%.3f,\"touchdown_speed_mps\":%.3f,\"touchdown_along_m\":%.3f,\"touchdown_cross_m\":%.3f,\"touchdown_gear\":%s,\"tail_strike\":%s,\"touchdown_pitch_deg\":%.3f,\"final_along_m\":%.3f,\"final_cross_m\":%.3f,"
             "\"main_touchdown\":%s,\"nose_touchdown\":%s,\"nose_touchdown_sink_mps\":%.3f,\"nose_touchdown_delay_s\":%.3f,\"bounce_count\":%d,\"max_gear_load_g\":%.3f,"
             "\"runway_departure\":%s,\"departure_along_m\":%.3f,\"departure_cross_m\":%.3f,\"departure_speed_mps\":%.3f,\"stopped\":%s,\"stop_along_m\":%.3f,\"stop_cross_m\":%.3f,\"crashed\":%s,"
             "\"ground_model\":\"gear-spring-damper-v2 (main geometry measured; nose x, tail, strut, pitch inertia surrogate)\"}",
             sim->scenario.name,s->sim_elapsed_s,l.altitude_m,s->touchdown_seen?"true":"false",s->on_runway_at_touchdown?"true":"false",s->touchdown_sink_mps,s->touchdown_speed_mps,s->touchdown_along_m,s->touchdown_cross_m,s->touchdown_gear?"true":"false",s->tail_strike?"true":"false",s->touchdown_pitch_deg,along,cross,
             s->main_touchdown_seen?"true":"false",s->nose_touchdown_seen?"true":"false",s->nose_touchdown_sink_mps,s->nose_touchdown_delay_s,s->bounce_count,s->max_gear_load_g,
             s->runway_departure?"true":"false",s->departure_along_m,s->departure_cross_m,s->departure_speed_mps,s->stopped?"true":"false",s->stop_along_m,s->stop_cross_m,s->crashed?"true":"false");
}
