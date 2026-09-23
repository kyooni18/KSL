#include "landing.h"
#include "decision_envelope.h"

#include <float.h>
#include <math.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>

typedef struct {
    double density; Vector3 air_velocity;
    double speed,speed_of_sound,mach,q,drag_accel,lift_accel,non_gravity;
    Vector3 specific_force;double physics_confidence,physics_uncertainty;bool direct_aero;
} AtmosState;
typedef struct {
    double sign,leg_elapsed; bool established,entry_loaded,final_captured,configured_leg_dwell_satisfied;
    bool hac_progress_valid,hac_completed; double hac_previous_angle,hac_remaining,hac_radius,hac_circuit_slope;
    bool hac_return_to_entry, hac_failed;
    double hac_lost_duration, terminal_reentry_after_ut;
    unsigned reversals; JerkLimiter limiter;
    bool has_first_reversal, first_reversal_final, final_heading_lock;
    double first_reversal_ut, first_reversal_range, first_reversal_sign;
    bool replay_event, replay_event_final; double replay_event_ut, replay_event_sign;
    double entry_score; bool has_entry_score;
    double actual_bank,actual_bank_rate,actual_aoa,actual_aoa_rate,entry_speed;
    bool has_interface_target,enforce_terminal_delivery_budget; TaemInterfaceTarget interface_target;
    bool force_control; double forced_until_ut,control_horizon_ut,forced_bank,forced_aoa;
    SpeedbrakeController speedbrake;
} PredictorGuidance;
typedef struct {
    const PlanetModel*p; const AerodynamicEnvelope*e; const TrajectoryCalibrationModel*c; const VehicleProfile*v;
    double bank,aoa,ut,mass;
    bool gear,brakes; int airbrakes;
} AeroContext;

static AerodynamicModel aero_at(const AerodynamicEnvelope*e,double mach){double a[]={.35,1.05,2.6,6};double x=fmax(0,mach);if(x<=a[0])return e->regimes[0];if(x>=a[3])return e->regimes[3];for(int i=0;i<3;i++)if(x<=a[i+1]){double f=(x-a[i])/(a[i+1]-a[i]);AerodynamicModel r={e->regimes[i].lift_to_drag+(e->regimes[i+1].lift_to_drag-e->regimes[i].lift_to_drag)*f,e->regimes[i].ballistic_coefficient+(e->regimes[i+1].ballistic_coefficient-e->regimes[i].ballistic_coefficient)*f,e->regimes[i].confidence+(e->regimes[i+1].confidence-e->regimes[i].confidence)*f};return r;}return e->regimes[3];}
static double scheduled_aoa(double altitude,const PlanetModel*p,const VehicleProfile*v){double lower=fmax(5,fmin(v->entry_angle_of_attack,10)),f=clampd(altitude/fmax(p->atmosphere_depth,1),0,1);return lower+(v->entry_angle_of_attack-lower)*f;}
static Vector3 gravity(Vector3 pos,const PlanetModel*p){return vessel_physics_gravity(pos,p);}
static double context_aoa(const AeroContext*ctx,double altitude){return isfinite(ctx->aoa)?ctx->aoa:scheduled_aoa(altitude,ctx->p,ctx->v);}
static AtmosState atmosphere(Vector3 pos,Vector3 vel,const AeroContext*ctx){
    double alt=vmag(pos)-ctx->p->radius;
    double d,sos;vessel_physics_environment(ctx->p,ctx->c,alt,&d,&sos);
    Vector3 air=vsub(vel,vcross(planet_rotation_vector(ctx->p),pos));
    double sp=vmag(air);
    double mach=sp/fmax(sos,1),q=.5*fmax(0,d)*sp*sp;
    AtmosState s={.density=d,.air_velocity=air,.speed=sp,.speed_of_sound=sos,.mach=mach,.q=q};
    s.specific_force=vessel_physics_force_best_estimate_config(ctx->c->physics,q,mach,context_aoa(ctx,alt),0,ctx->mass,ctx->gear,ctx->brakes,ctx->airbrakes,aero_at(ctx->e,mach),ctx->c,ctx->v,&s.physics_confidence,&s.direct_aero,&s.physics_uncertainty);
    s.drag_accel=s.specific_force.x;s.lift_accel=s.specific_force.y;
    s.non_gravity=vmag(s.specific_force);
    return s;
}
static Vector3 total_accel(Vector3 pos,Vector3 vel,const AeroContext*ctx){
    AtmosState s=atmosphere(pos,vel,ctx);
    /* A physics model pointer does not guarantee local direct-force support.
       In uncovered Mach/q/incidence regions vessel_physics_force() falls back
       to the legacy envelope, and that fallback still needs the learned turn
       effectiveness.  Only an actually observed local force sample gets the
       ideal geometric bank rotation. */
    double effectiveness=s.direct_aero?1:clampd(ctx->c->bank_effectiveness,.55,1.35);
    return vessel_physics_acceleration(pos,s.air_velocity,ctx->p,s.specific_force,ctx->bank*effectiveness);
}
static VehicleState rk4_gravity(VehicleState s,double dt,const PlanetModel*p){Vector3 p1=s.velocity,v1=gravity(s.position,p),p2=vadd(s.position,vscale(p1,dt*.5)),u2=vadd(s.velocity,vscale(v1,dt*.5)),v2=gravity(p2,p),p3=vadd(s.position,vscale(u2,dt*.5)),u3=vadd(s.velocity,vscale(v2,dt*.5)),v3a=gravity(p3,p),p4=vadd(s.position,vscale(u3,dt)),u4=vadd(s.velocity,vscale(v3a,dt)),v4=gravity(p4,p);s.position=vadd(s.position,vscale(vadd(vadd(p1,vscale(u2,2)),vadd(vscale(u3,2),u4)),dt/6));s.velocity=vadd(s.velocity,vscale(vadd(vadd(v1,vscale(v2,2)),vadd(vscale(v3a,2),v4)),dt/6));s.ut+=dt;return s;}
static VehicleState rk4_aero(VehicleState s,double dt,AeroContext*ctx){ctx->ut=s.ut;ctx->mass=s.mass;Vector3 k1p=s.velocity,k1v=total_accel(s.position,s.velocity,ctx),p2=vadd(s.position,vscale(k1p,dt*.5)),u2=vadd(s.velocity,vscale(k1v,dt*.5)),k2p=u2,k2v=total_accel(p2,u2,ctx),p3=vadd(s.position,vscale(k2p,dt*.5)),u3=vadd(s.velocity,vscale(k2v,dt*.5)),k3p=u3,k3v=total_accel(p3,u3,ctx),p4=vadd(s.position,vscale(k3p,dt)),u4=vadd(s.velocity,vscale(k3v,dt)),k4p=u4,k4v=total_accel(p4,u4,ctx);s.position=vadd(s.position,vscale(vadd(vadd(k1p,vscale(k2p,2)),vadd(vscale(k3p,2),k4p)),dt/6));s.velocity=vadd(s.velocity,vscale(vadd(vadd(k1v,vscale(k2v,2)),vadd(vscale(k3v,2),k4v)),dt/6));s.ut+=dt;return s;}

static void body_basis(const PlanetModel*p,double ut,Vector3*prime,Vector3*east,Vector3*north){*north=vnorm(p->north_axis,v3(0,0,1));*prime=vnorm(vproject_plane(vrotate(p->prime_meridian_at_epoch,*north,p->rotational_speed*(ut-p->epoch_ut)),*north),v3(1,0,0));*east=vnorm(vcross(*north,*prime),v3(0,1,0));}
GeoPoint predictor_geo_point(Vector3 pos,const PlanetModel*p,double ut){Vector3 prime,east,north;body_basis(p,ut,&prime,&east,&north);Vector3 up=vnorm(pos,prime);double sl=clampd(vdot(up,north),-1,1),lat=asin(sl);Vector3 eq=vnorm(vsub(up,vscale(north,sl)),prime);GeoPoint g={lat*RAD2DEG,norm_signed_deg(atan2(vdot(eq,east),vdot(eq,prime))*RAD2DEG),vmag(pos)-p->radius};return g;}
Vector3 predictor_inertial_position(GeoPoint g,const PlanetModel*p,double ut){Vector3 prime,east,north;body_basis(p,ut,&prime,&east,&north);double lat=g.latitude*DEG2RAD,lon=g.longitude*DEG2RAD,h=cos(lat);return vscale(vadd(vadd(vscale(prime,h*cos(lon)),vscale(east,h*sin(lon))),vscale(north,sin(lat))),p->radius+g.altitude);}
VehicleState predictor_propagate_vacuum(VehicleState s,double target,const PlanetModel*p,double step){if(!p||!isfinite(target)||!isfinite(s.ut)||target<=s.ut)return s;step=clampd(step,.01,120);while(s.ut<target){double dt=fmin(step,target-s.ut);s=rk4_gravity(s,dt,p);}return s;}
double predictor_postburn_periapsis(VehicleState s,const PlanetModel*p){if(!p||!isfinite(s.mass)||!isfinite(s.position.x)||!isfinite(s.position.y)||!isfinite(s.position.z)||!isfinite(s.velocity.x)||!isfinite(s.velocity.y)||!isfinite(s.velocity.z))return -DBL_MAX;double r=vmag(s.position);if(!isfinite(r)||r<=1||!isfinite(p->gravitational_parameter)||p->gravitational_parameter<=0)return -DBL_MAX;Vector3 h=vcross(s.position,s.velocity);double h2=vdot(h,h),e=.5*vdot(s.velocity,s.velocity)-p->gravitational_parameter/r;if(!isfinite(h2)||!isfinite(e))return -DBL_MAX;double ecc=sqrt(fmax(0,1+2*e*h2/(p->gravitational_parameter*p->gravitational_parameter))),pr=h2/fmax(p->gravitational_parameter*(1+ecc),1);return isfinite(pr)?pr-p->radius:-DBL_MAX;}
double predictor_directed_taem_range_error(double range,double closest_distance,double target_range){
    if(!isfinite(range)||!isfinite(closest_distance)||!isfinite(target_range))return NAN;
    range=fmax(0,range);target_range=fmax(0,target_range);closest_distance=clampd(closest_distance,0,range);
    if(closest_distance<target_range){
        /* The trajectory entered the desired TAEM shell before reaching TAEM
           altitude. Preserve that penetration even when the TAEM point is
           still numerically inside the shell: an aircraft that passed close
           to KSC and is now 20 km away on the far side is not equivalent to
           an aircraft approaching the 35 km interface from outside. */
        double passed_distance=range+target_range-2.0*closest_distance;
        return -fmax(fabs(range-target_range),passed_distance);
    }
    return range-target_range;
}

static _Thread_local PredictorPlannerTrace g_planner_trace;

static void planner_trace_begin(VehicleState state,const PlanetModel*p,const LandingSite*site,
        double current_bank,double current_aoa,double current_sign,bool current_leg_established,
        double current_leg_elapsed,double current_commit_remaining,bool side_locked,bool final_heading_lock){
    memset(&g_planner_trace,0,sizeof(g_planner_trace));
    g_planner_trace.selected_index=-1;
    g_planner_trace.mode=ENTRY_SUPERVISION_INFEASIBLE;
    g_planner_trace.candidate_budget=PREDICTOR_PLANNER_TRACE_MAX_CANDIDATES;
    if(!p||!site)return;
    GeoPoint geo=predictor_geo_point(state.position,p,state.ut),target={site->latitude,site->longitude,site->altitude};
    Vector3 air=vsub(state.velocity,vcross(planet_rotation_vector(p),state.position));
    g_planner_trace.valid=true;g_planner_trace.ut=state.ut;
    g_planner_trace.altitude=geo.altitude;g_planner_trace.airspeed=vmag(air);
    g_planner_trace.range=great_circle_distance(geo,target,p->radius);
    g_planner_trace.current_bank=current_bank;g_planner_trace.current_aoa=current_aoa;
    g_planner_trace.current_sign=current_sign;g_planner_trace.current_leg_established=current_leg_established;
    g_planner_trace.current_leg_elapsed=current_leg_elapsed;g_planner_trace.current_commit_remaining=current_commit_remaining;
    g_planner_trace.side_locked=side_locked;g_planner_trace.final_heading_lock=final_heading_lock;
}

static void planner_trace_capture_path(PredictorPlannerCandidateTrace*c,const EntryPrediction*prediction){
    if(!c||!prediction||!prediction->trajectory.points||prediction->trajectory.count==0)return;
    size_t total=prediction->trajectory.count;
    size_t samples=total<PREDICTOR_PLANNER_TRACE_MAX_PATH_POINTS?total:PREDICTOR_PLANNER_TRACE_MAX_PATH_POINTS;
    c->trajectory_total_points=total;c->trajectory_sample_count=samples;c->trajectory_truncated=total>samples;
    for(size_t i=0;i<samples;i++){
        size_t index=samples<=1?0:(i*(total-1))/(samples-1);
        c->trajectory_points[i]=prediction->trajectory.points[index];
    }
}

static int planner_trace_add(PredictorPlanCandidateSource source,const EntryControlPlan*plan,
        const EntryPredictionAssessment*assessment,const EntryPrediction*prediction,double evaluation_cost,
        const PredictorPlannerCostTerms*cost_terms){
    if(!g_planner_trace.valid||!plan)return -1;
    if(g_planner_trace.candidate_count>=PREDICTOR_PLANNER_TRACE_MAX_CANDIDATES){g_planner_trace.dropped_candidate_count++;return -1;}
    unsigned i=g_planner_trace.candidate_count++;
    PredictorPlannerCandidateTrace*c=&g_planner_trace.candidates[i];memset(c,0,sizeof(*c));
    c->source=source;c->plan=*plan;c->evaluation_index=i;c->evaluation_cost=evaluation_cost;
    if(cost_terms)c->cost_terms=*cost_terms;
    if(assessment)c->assessment=*assessment;
    if(assessment){
        if(!assessment->valid)c->rejection_flags|=PREDICTOR_CANDIDATE_REJECT_INVALID;
        if(assessment->dynamic_pressure_violation)c->rejection_flags|=PREDICTOR_CANDIDATE_REJECT_DYNAMIC_PRESSURE;
        if(assessment->g_load_violation)c->rejection_flags|=PREDICTOR_CANDIDATE_REJECT_G_LOAD;
        if(assessment->stall_risk)c->rejection_flags|=PREDICTOR_CANDIDATE_REJECT_STALL;
        if(assessment->control_margin_violation)c->rejection_flags|=PREDICTOR_CANDIDATE_REJECT_CONTROL_MARGIN;
        if(assessment->valid&&!assessment->terminal_feasible)c->rejection_flags|=PREDICTOR_CANDIDATE_REJECT_TERMINAL_INFEASIBLE;
    }
    if(prediction){
        c->peak_dynamic_pressure=prediction->peak_dynamic_pressure;c->peak_g_load=prediction->peak_g_load;
        c->minimum_entry_speed=prediction->minimum_entry_speed;c->terminal_capture_score=prediction->taem_hac_capture_score;
        c->reached_taem=prediction->reached_taem;
        if(!prediction->reached_taem)c->rejection_flags|=PREDICTOR_CANDIDATE_REJECT_NO_TAEM;
        planner_trace_capture_path(c,prediction);
    }
    return (int)i;
}

static void planner_trace_select(PredictorPlanCandidateSource source,const EntryControlPlan*plan,EntrySupervisionMode mode){
    g_planner_trace.mode=mode;g_planner_trace.selected_index=-1;g_planner_trace.selection_converged=false;
    for(unsigned i=0;i<g_planner_trace.candidate_count;i++)g_planner_trace.candidates[i].selected=false;
    if(!plan)return;
    double best=INFINITY;
    for(unsigned i=0;i<g_planner_trace.candidate_count;i++){
        PredictorPlannerCandidateTrace*c=&g_planner_trace.candidates[i];if(c->source!=source)continue;
        double d=fabs(c->plan.target_bank-plan->target_bank)+fabs(c->plan.target_aoa-plan->target_aoa)+
            .01*fabs(c->plan.segment_duration-plan->segment_duration);
        if(d<best){best=d;g_planner_trace.selected_index=(int)i;}
    }
    if(g_planner_trace.selected_index>=0){
        g_planner_trace.selection_converged=true;
        for(unsigned i=0;i<g_planner_trace.candidate_count;i++){
            PredictorPlannerCandidateTrace*c=&g_planner_trace.candidates[i];
            if((int)i==g_planner_trace.selected_index){c->selected=true;c->rejection_flags=PREDICTOR_CANDIDATE_REJECT_NONE;continue;}
            if(c->rejection_flags==PREDICTOR_CANDIDATE_REJECT_NONE)
                c->rejection_flags|=c->source==source?PREDICTOR_CANDIDATE_REJECT_HIGHER_COST:PREDICTOR_CANDIDATE_REJECT_POLICY_PRIORITY;
        }
    }
}

bool predictor_last_planner_trace(PredictorPlannerTrace*out){if(!out||!g_planner_trace.valid)return false;*out=g_planner_trace;return true;}

static Vector3 burn_direction(Vector3 pos,Vector3 vel,double radial_error_deg,double normal_error_deg){
    Vector3 retro=vnorm(vscale(vel,-1),vnorm(vscale(pos,-1),v3(-1,0,0)));
    Vector3 normal=vnorm(vcross(pos,vel),v3(0,0,1));
    Vector3 radial=vnorm(vproject_plane(vnorm(pos,v3(1,0,0)),retro),vnorm(vcross(normal,retro),v3(0,1,0)));
    double radial_error=tan(radial_error_deg*DEG2RAD),normal_error=tan(normal_error_deg*DEG2RAD);
    return vnorm(vadd(retro,vadd(vscale(radial,radial_error),vscale(normal,normal_error))),retro);
}

static Vector3 burn_accel(Vector3 pos,Vector3 vel,const PlanetModel*p,double thrust_accel,double radial_error_deg,double normal_error_deg){
    return vadd(gravity(pos,p),vscale(burn_direction(pos,vel,radial_error_deg,normal_error_deg),thrust_accel));
}

static VehicleState retro_burn_scenario(VehicleState s,double dv,double thrust,double throttle,double ramp,const PlanetModel*p,double radial_error_deg,double normal_error_deg,double *duration,double *projected_delta_v,const VesselPhysicsModel *physics){
    double maxa=thrust*clampd(throttle,0,1)/fmax(s.mass,1);
    if(!isfinite(maxa)||!isfinite(dv)||!isfinite(ramp))maxa=0;
    ramp=clampd(isfinite(ramp)?ramp:0,0,60);
    if(duration)*duration=0;
    if(projected_delta_v)*projected_delta_v=0;
    if(dv<=0||maxa<=.01)return s;
    double delivered=0,elapsed=0,step=.25;
    while(delivered<dv-.08&&elapsed<600){
        if(physics&&physics->dry_mass>0&&s.mass<=physics->dry_mass)break;
        maxa=thrust*clampd(throttle,0,1)/fmax(s.mass,1);
        double rem=fmax(0,dv-delivered),f=burn_fraction(elapsed,rem,maxa,ramp);
        if(f<=.0001){elapsed+=step;continue;}
        double dt=fmin(step,fmax(.02,(dv-delivered)/fmax(maxa*f,.01))),ta=maxa*f;
        Vector3 retro=vnorm(vscale(s.velocity,-1),v3(-1,0,0));
        Vector3 dir=burn_direction(s.position,s.velocity,radial_error_deg,normal_error_deg);
        double projection=clampd(vdot(dir,retro),0,1);
        Vector3 p1=s.velocity,v1=burn_accel(s.position,s.velocity,p,ta,radial_error_deg,normal_error_deg);
        Vector3 p2=vadd(s.position,vscale(p1,dt*.5)),u2=vadd(s.velocity,vscale(v1,dt*.5)),v2=burn_accel(p2,u2,p,ta,radial_error_deg,normal_error_deg);
        Vector3 p3=vadd(s.position,vscale(u2,dt*.5)),u3=vadd(s.velocity,vscale(v2,dt*.5)),v3a=burn_accel(p3,u3,p,ta,radial_error_deg,normal_error_deg);
        Vector3 p4=vadd(s.position,vscale(u3,dt)),u4=vadd(s.velocity,vscale(v3a,dt)),v4=burn_accel(p4,u4,p,ta,radial_error_deg,normal_error_deg);
        s.position=vadd(s.position,vscale(vadd(vadd(p1,vscale(u2,2)),vadd(vscale(u3,2),u4)),dt/6));
        s.velocity=vadd(s.velocity,vscale(vadd(vadd(v1,vscale(v2,2)),vadd(vscale(v3a,2),v4)),dt/6));
        s.ut+=dt;
        s.mass=vessel_physics_burn_mass(physics,s.mass,thrust*throttle*f,dt);
        delivered+=ta*projection*dt;
        elapsed+=dt;
        if(!isfinite(s.ut)||!isfinite(vmag(s.position))||!isfinite(vmag(s.velocity)))break;
    }
    if(duration)*duration=elapsed;
    if(projected_delta_v)*projected_delta_v=delivered;
    return s;
}

static double bank_limit(const AtmosState*a,const PlanetModel*p,const VehicleProfile*v,double max_bank){
    double g_load=a->non_gravity/fmax(planet_surface_gravity(p),.1);
    return entry_bank_authority_limit(a->speed,a->q,g_load,v,max_bank);
}
static double turn_radius_for_bank(const AtmosState*a,const TrajectoryCalibrationModel*c,double bank){
    if(!a||!c||a->speed<=1||a->lift_accel<=.005||fabs(bank)<.05)return INFINITY;
    double effective=fabs(bank)*clampd(c->bank_effectiveness,.35,1.8)*DEG2RAD;
    effective=clampd(effective,0,89*DEG2RAD);
    double lateral=a->lift_accel*fabs(sin(effective));
    return lateral>.005?a->speed*a->speed/lateral:INFINITY;
}
static double bank_for_turn_radius(const AtmosState*a,const TrajectoryCalibrationModel*c,double radius,double max_bank){
    if(!a||!c||!isfinite(radius)||radius<=1||a->speed<=1||a->lift_accel<=.005)return 0;
    double needed=a->speed*a->speed/radius,ratio=needed/fmax(a->lift_accel,.005);
    if(ratio<=0)return 0;
    double eff=clampd(c->bank_effectiveness,.35,1.8),limit=fabs(max_bank)*eff*DEG2RAD;
    limit=clampd(limit,0,89*DEG2RAD);
    double angle=asin(clampd(ratio,0,sin(limit)))/eff*RAD2DEG;
    return clampd(angle,0,fabs(max_bank));
}
double entry_taem_speed_target(const VehicleProfile*v,const GuidanceSettings*s,const PlanetModel*p){
    if(!v||!s||!p)return NAN;
    TaemSpeedEnvelope envelope=decision_taem_speed_envelope(
        v,p,s->taem_interface_altitude);
    if(!envelope.valid)
        return fmax(v->minimum_safe_speed,v->final_approach_speed);
    /*
     * This scalar is retained only as a compatibility/reference value for
     * predictor code that has not yet been migrated to the full envelope.
     * It means "maximum structurally usable TAEM speed at the configured
     * interface altitude", not a tuned nominal. Live MM304 publication further
     * constrains it by the actual energy available to reach the fixed point.
     */
    return envelope.feasible?envelope.maximum_speed_mps:
        envelope.minimum_speed_mps;
}
static int predictor_speedbrake_state(PredictorGuidance*g,GeoPoint geo,const AtmosState*a,const PlanetModel*p,const TrajectoryCalibrationModel*c,const VehicleProfile*v,const LandingSite*site,const GuidanceSettings*s,double mass,bool reached_taem,double dt){
    if(!reached_taem){speedbrake_controller_reset(&g->speedbrake,false);return 0;}
    bool predicted_gear=geo.altitude-site->altitude<=s->gear_deployment_altitude;
    Vector3 deployed_force;double deployed_confidence=0;
    bool speedbrake_model_ready=c&&c->physics&&
        vessel_physics_aero_config(c->physics,a->q,a->mach,g->actual_aoa,0,mass,predicted_gear,false,1,&deployed_force,&deployed_confidence)&&
        deployed_confidence>=.12;
    if(!speedbrake_model_ready){speedbrake_controller_reset(&g->speedbrake,false);return 0;}
    GeoPoint target={site->latitude,site->longitude,site->altitude};
    double range=great_circle_distance(geo,target,p->radius),taem_range=fmax(s->taem_interface_range,s->final_approach_distance+1000);
    double blend=clampd((range-s->final_approach_distance)/fmax(taem_range-s->final_approach_distance,1),0,1);
    blend=sqrt(blend);
    double taem_speed=entry_taem_speed_target(v,s,p);
    double final_speed=fmax(v->minimum_safe_speed*1.08,v->final_approach_speed);
    double target_speed=final_speed+(taem_speed-final_speed)*blend;
    double altitude_blend=clampd(range/fmax(taem_range,1),0,1);
    double target_altitude=site->altitude+(s->taem_interface_altitude-site->altitude)*altitude_blend;
    double density=planet_atmospheric_density(p,target_altitude)*clampd(c->density_scale,.35,2.8);
    double target_q=.5*fmax(0,density)*target_speed*target_speed;
    double current_energy=rotating_specific_energy(geo.latitude,geo.altitude,a->speed,p);
    double target_energy=rotating_specific_energy(geo.latitude,target_altitude,target_speed,p);
    double energy_scale=fmax(.5*target_speed*target_speed,planet_surface_gravity(p)*fmax(1000,target_altitude-site->altitude));
    return speedbrake_controller_update(&g->speedbrake,a->q,target_q,current_energy-target_energy,energy_scale,v->maximum_dynamic_pressure,false,dt)?1:0;
}
static double entry_energy_to_taem(double latitude,double altitude,double speed,const PlanetModel*p,const VehicleProfile*v,const LandingSite*site,const GuidanceSettings*s){
    double target_speed=entry_taem_speed_target(v,s,p);
    return fmax(0,entry_remaining_specific_energy(latitude,altitude,speed,site->latitude,s->taem_interface_altitude,target_speed,p));
}
static double entry_required_average_drag(GeoPoint geo,double speed,double range,const PlanetModel*p,
        const VehicleProfile*v,const LandingSite*site,const GuidanceSettings*s,
        const TaemInterfaceTarget*interface_target){
    if(interface_target&&interface_target->valid&&isfinite(interface_target->specific_energy)&&
       isfinite(interface_target->along_track)&&isfinite(interface_target->cross_track)){
        double rh=site->runway_heading*DEG2RAD;
        double e=interface_target->along_track*sin(rh)+interface_target->cross_track*cos(rh);
        double n=interface_target->along_track*cos(rh)-interface_target->cross_track*sin(rh);
        GeoPoint origin={site->latitude,site->longitude,site->altitude};
        GeoPoint gate=local_point(origin,e,n,p->radius,interface_target->altitude);
        double distance=great_circle_distance(geo,gate,p->radius);
        double current=rotating_specific_energy(geo.latitude,geo.altitude,speed,p);
        if(isfinite(distance)&&isfinite(current))
            return fmax(0.0,current-interface_target->specific_energy)/fmax(5000.0,distance);
    }
    double reserve=fmax(entry_taem_range_target(p,s),s->hac_radius*3.0);
    double usable=fmax(5000,range-reserve);
    return entry_energy_to_taem(geo.latitude,geo.altitude,speed,p,v,site,s)/usable;
}
static void predictor_change_sign(PredictorGuidance*g,double sign){sign=sign>=0?1:-1;if(g->sign==sign)return;if(g->sign!=0)g->reversals++;g->sign=sign;g->leg_elapsed=0;g->established=false;g->configured_leg_dwell_satisfied=false;}
static void attitude_step(PredictorGuidance*g,double target_bank,double target_aoa,const AtmosState*a,const VehicleProfile*v,const GuidanceSettings*set,double dt,const VesselPhysicsModel *physics){
    bool measured_roll=physics&&physics->authority_confidence[1]>0&&isfinite(physics->authority[1])&&physics->authority[1]>1e-6;
    bool measured_pitch=physics&&physics->authority_confidence[0]>0&&isfinite(physics->authority[0])&&physics->authority[0]>1e-6;
    if(measured_roll)
        vessel_physics_axis_step(physics,1,target_bank,a->q,set->entry_roll_rate,dt,&g->actual_bank,&g->actual_bank_rate);
    else{
        FlightControlRegime regime=flight_control_regime(a->mach,a->speed,a->q,v->minimum_safe_speed);
        double bank_error=norm_signed_deg(target_bank-g->actual_bank),max_rr=fmax(.8,set->entry_roll_rate*1.12*regime.response_scale),max_ra=fmax(.45,set->entry_roll_acceleration*1.15*regime.response_scale);
        double desired_rr=clampd(bank_error*.72,-max_rr,max_rr),rr_delta=clampd(desired_rr-g->actual_bank_rate,-max_ra*dt,max_ra*dt);
        g->actual_bank_rate=clampd(g->actual_bank_rate+rr_delta,-max_rr*1.35,max_rr*1.35);g->actual_bank=norm_signed_deg(g->actual_bank+g->actual_bank_rate*dt);
    }
    if(measured_pitch)
        vessel_physics_axis_step(physics,0,target_aoa,a->q,set->entry_roll_rate,dt,&g->actual_aoa,&g->actual_aoa_rate);
    else{
        double aoa_error=target_aoa-g->actual_aoa,max_ar=4.2,max_aa=2.4,desired_ar=clampd(aoa_error*.62,-max_ar,max_ar),ar_delta=clampd(desired_ar-g->actual_aoa_rate,-max_aa*dt,max_aa*dt);
        g->actual_aoa_rate=clampd(g->actual_aoa_rate+ar_delta,-max_ar*1.25,max_ar*1.25);g->actual_aoa=clampd(g->actual_aoa+g->actual_aoa_rate*dt,-v->maximum_angle_of_attack*1.5,v->maximum_angle_of_attack*1.5);
    }
}
static double predictor_terminal_hac_radius(const AtmosState*a,const PlanetModel*p,const TrajectoryCalibrationModel*c,const VehicleProfile*v,const GuidanceSettings*set,double maxbank);
static double predictor_interface_heading(const PredictorGuidance*g,GeoPoint geo,const PlanetModel*p,
        const LandingSite*site,const GuidanceSettings*set,double fallback_bearing){
    if(!g||!g->has_interface_target||!g->interface_target.valid)return fallback_bearing;
    (void)set;
    GeoPoint site_geo={site->latitude,site->longitude,site->altitude};
    double along=0.0,cross=0.0;
    runway_coordinates(geo,site_geo,site->runway_heading,p->radius,&along,&cross);
    return taem_interface_line_heading(along,cross,site->runway_heading,&g->interface_target);
}

static bool predictor_terminal_arc_ready(GeoPoint geo,double course,const PlanetModel*p,
        const LandingSite*site,const TaemInterfaceTarget*q,double final_sign){
    if(!p||!site||!q||!q->valid||!isfinite(course))return true;
    GeoPoint site_geo={site->latitude,site->longitude,site->altitude};
    double along=0.0,cross=0.0;
    runway_coordinates(geo,site_geo,site->runway_heading,p->radius,&along,&cross);
    double da=q->along_track-along,dc=q->cross_track-cross;
    if(hypot(da,dc)<1000.0)return true;
    double bearing=norm_deg(site->runway_heading+atan2(dc,da)*RAD2DEG);
    double side=final_sign>=0.0?1.0:-1.0;
    double gate_turn=norm_signed_deg(bearing-course);
    double tangent_turn=norm_signed_deg(q->course-course);
    return side*gate_turn>=1.0&&side*tangent_turn>=-.5;
}
static double entry_bank(PredictorGuidance*g,VehicleState s,GeoPoint geo,const AtmosState*a,const PlanetModel*p,AerodynamicModel aero,const TrajectoryCalibrationModel*c,const VehicleProfile*v,const LandingSite*site,const GuidanceSettings*set,double maxbank,double dt,double*aoa_out){
    (void)aero;
    Vector3 up=vnorm(s.position,v3(0,1,0)),air=vsub(s.velocity,vcross(planet_rotation_vector(p),s.position)),hv=vproject_plane(air,up),north=vnorm(vsub(p->north_axis,vscale(up,vdot(p->north_axis,up))),v3(0,0,1)),east=vnorm(vcross(north,up),v3(1,0,0));
    GeoPoint target={site->latitude,site->longitude,site->altitude};
    bool tangent_lock=g->final_heading_lock&&g->has_interface_target&&g->interface_target.valid;
    GeoPoint heading_target=g->final_heading_lock&&!tangent_lock?
        runway_approach_aimpoint(site,p->radius,500.0):target;
    double course=vmag(hv)>1?norm_deg(atan2(vdot(hv,east),vdot(hv,north))*RAD2DEG):initial_bearing(geo,heading_target);
    double bearing=initial_bearing(geo,heading_target);
    if(g->has_interface_target&&g->interface_target.valid)
        bearing=predictor_interface_heading(g,geo,p,site,set,bearing);
    double he=norm_signed_deg(bearing-course),range=great_circle_distance(geo,target,p->radius);
    double grav=p->gravitational_parameter/pow(p->radius+fmax(0,geo.altitude),2);
    if(g->entry_speed<=0&&a->speed>100)g->entry_speed=a->speed;
    const TaemInterfaceTarget*delivery_target=g->has_interface_target&&g->interface_target.valid?
        &g->interface_target:NULL;
    LandingConfiguration shaping_cfg={.site=*site,.vehicle=*v,.guidance=*set};
    double required_drag=entry_required_average_drag(geo,a->speed,range,p,v,site,set,delivery_target);
    double drag_ratio=required_drag>1e-6?a->drag_accel/required_drag:1.0;
    double drag_deficit=clampd(1.0-drag_ratio,0,1);
    /* Autonomous predictor policy keeps a conservative 60 deg planning ceiling,
       but a committed MM304 plan may already be legal above that private ceiling.
       Replay the committed command against the same shared authority limit as live
       guidance instead of silently clipping it and manufacturing a margin failure. */
    double shared_bank_limit=bank_limit(a,p,v,maxbank);
    double env=fmin(60.0,shared_bank_limit),minimum_radius=turn_radius_for_bank(a,c,env),mag=0;
    bool forced_policy=g->force_control&&s.ut<g->forced_until_ut;
    if(forced_policy)env=shared_bank_limit;
    if(forced_policy){
        /* MPC candidates are policies, not one short control pulse followed by
           an optimistic fallback. Keep the selected bank magnitude and AoA as
           the entry-energy policy for the whole forecast, while still letting
           the S-turn side reverse when the predicted ground track crosses its
           heading corridor. This removes receding-horizon procrastination: a
           wings-level candidate is scored as wings-level all the way to TAEM
           instead of assuming it will suddenly bank hard after 60-70 seconds. */
        mag=clampd(fabs(g->forced_bank),0,env);
    }else if(drag_deficit>.01&&isfinite(minimum_radius)){
        /* Bank is an energy-path actuator: reducing the vertical component of
           lift lets a hot trajectory descend into denser air. Express the
           command as a broad turn-radius family so energy correction never
           degenerates into the old 70 deg knife-edge S-turn. */
        double radius_scale=8.0-(8.0-1.20)*drag_deficit;
        mag=bank_for_turn_radius(a,c,minimum_radius*radius_scale,env);
    }
    if(g->sign==0)predictor_change_sign(g,he>=0?1:-1);
    double effective_min_leg=entry_s_turn_effective_minimum_leg(a->speed,entry_taem_speed_target(v,set,p),set);
    /* The longer dwell governs creation of new S-turn topology. A reversal that
       was already committed remains an executable event and keeps the original
       configured actuator dwell; otherwise a model update could move the goal
       posts after the aircraft has already committed to the roll-through. */
    double executable_min_leg=g->replay_event?set->s_turn_minimum_leg_duration:effective_min_leg;
    if(g->established&&g->leg_elapsed>=set->s_turn_minimum_leg_duration)g->configured_leg_dwell_satisfied=true;
    bool configured_minleg=g->configured_leg_dwell_satisfied;
    bool minleg=g->established&&g->leg_elapsed>=executable_min_leg;
    /* Tighten the bank-reversal corridor as terminal range closes. The old
       geometry widened near KSC because the fixed HAC-width term was divided
       by a shrinking range, so predicted reversals accumulated in the last
       few dozen kilometres. Use the configured HAC look-ahead angle for the
       broad early-entry corridor and the apparent angular width of one HAC
       diameter from the nominal entry range for the terminal corridor.

       Evaluate the corridor at the range where the shuttle will be after a
       full sign reversal has had time to establish. This starts the roll soon
       enough that the new bank, rather than wings-level roll-through, reaches
       the terminal region. */
    FlightControlRegime reversal_regime=flight_control_regime(a->mach,a->speed,a->q,v->minimum_safe_speed);
    double reversal_rate=fmax(.5,set->entry_roll_rate*fmax(.42,reversal_regime.command_rate_scale));
    double reversal_accel=fmax(.3,set->entry_roll_acceleration*fmax(.30,reversal_regime.command_accel_scale));
    double reversal_span=fabs(g->actual_bank)+fmax(4.0,mag);
    double reversal_time=reversal_span/reversal_rate+reversal_rate/reversal_accel;
    double range_after_roll=fmax(0,range-a->speed*reversal_time);
    double terminal_join_range=entry_taem_range_target(p,set);
    double corridor_fraction=clampd((range_after_roll-terminal_join_range)/
        fmax(set->target_entry_range-terminal_join_range,1),0,1);
    corridor_fraction=corridor_fraction*corridor_fraction*(3.0-2.0*corridor_fraction);
    double mission_entry_range=fmax(set->target_entry_range,terminal_join_range+1000.0);
    double near_corridor=atan2(2.0*set->hac_radius,fmax(mission_entry_range,1))*RAD2DEG;
    double far_corridor=fmax(near_corridor,set->hac_look_ahead_angle);
    double heading_corridor=near_corridor+(far_corridor-near_corridor)*corridor_fraction;
    /* Timed S-turn reversals are only useful after the current bank has created
       meaningful lateral/vertical work. When the propagated shuttle is high
       for the mission altitude-vs-range corridor, widen the signed course
       corridor toward 22 deg. This keeps one bank side long enough to reduce
       vertical lift and build real crossrange instead of alternating around
       the site bearing every few tens of seconds. The extra corridor tapers to
       zero as TAEM range is consumed. */
    double mission_entry_altitude=entry_guidance_start_altitude(p,set);
    double desired_range_altitude=entry_altitude_target_for_range(mission_entry_altitude,
        mission_entry_range,range,set->taem_interface_altitude,terminal_join_range);
    double altitude_debt=fmax(0.0,geo.altitude-desired_range_altitude);
    double debt_scale=fmax(4500.0,(mission_entry_altitude-set->taem_interface_altitude)*.18);
    double altitude_debt_fraction=clampd(altitude_debt/debt_scale,0.0,1.0);
    double predicted_fpa=atan2(vdot(air,up),fmax(1.0,vmag(hv)))*RAD2DEG;
    double fpa_delivery_debt=entry_s_turn_fpa_delivery_debt(predicted_fpa,delivery_target);
    double interface_distance=INFINITY;
    if(delivery_target){
        double ih=site->runway_heading*DEG2RAD;
        double ie=delivery_target->along_track*sin(ih)+delivery_target->cross_track*cos(ih);
        double in=delivery_target->along_track*cos(ih)-delivery_target->cross_track*sin(ih);
        GeoPoint origin={site->latitude,site->longitude,site->altitude};
        GeoPoint interface_geo=local_point(origin,ie,in,p->radius,delivery_target->altitude);
        interface_distance=great_circle_distance(geo,interface_geo,p->radius);
    }
    heading_corridor=entry_s_turn_reversal_corridor(heading_corridor,range,
        terminal_join_range,altitude_debt_fraction,fpa_delivery_debt);
    /* With an explicit MM304 tangent state the last reversal is scheduled against
       distance to that gate, not radial distance to KSC.  The old KSC-range deadline
       could fire repeatedly while the vehicle was still hundreds of kilometres from
       the lateral tangent, producing alternating +/-60 deg legs that erased the very
       crossrange MM304 needed to deliver.  Reserve one roll-through plus one established
       final leg, then let that reversal become a one-way tangent capture. */
    double terminal_reversal_reserve;
    double terminal_reversal_metric;
    if(delivery_target){
        terminal_reversal_reserve=fmax(delivery_target->acquisition_lead,
            a->speed*(reversal_time+1.35*effective_min_leg));
        terminal_reversal_metric=fmax(0.0,interface_distance-a->speed*reversal_time);
    }else{
        terminal_reversal_reserve=terminal_join_range+
            a->speed*(reversal_time+1.35*effective_min_leg);
        terminal_reversal_metric=range_after_roll;
    }
    /* A high/FPA-indebted trajectory should keep the established energy-management
       leg rather than spend its remaining sink authority in a premature roll-through. */
    double remaining_taem_energy;
    if(delivery_target&&isfinite(delivery_target->specific_energy)){
        double current_energy=rotating_specific_energy(geo.latitude,geo.altitude,a->speed,p);
        remaining_taem_energy=isfinite(current_energy)?
            fmax(0.0,current_energy-delivery_target->specific_energy):INFINITY;
    }else{
        remaining_taem_energy=entry_energy_to_taem(geo.latitude,geo.altitude,a->speed,p,v,site,set);
    }
    double terminal_removable_energy=fmax(.03,a->drag_accel)*terminal_reversal_reserve*1.35;
    bool terminal_energy_ready=isfinite(remaining_taem_energy)&&remaining_taem_energy<=terminal_removable_energy;
    bool terminal_altitude_ready=altitude_debt<=600.0||
        a->speed<=set->taem_force_handoff_speed;
    bool terminal_reversal_deadline=terminal_reversal_metric<=terminal_reversal_reserve;
    bool terminal_roll_has_delivery_time=true;
    if(g->enforce_terminal_delivery_budget&&delivery_target&&fpa_delivery_debt>1e-9){
        /* Compare the remaining speed clock with the actual work a reversal would
           consume. Gravity partly offsets drag on a descending trajectory, so use
           the along-path net deceleration rather than drag alone. Then estimate the
           vertical-speed change still required to reach the latest legal TAEM FPA
           using the sink authority of the *currently established* bank. */
        double net_deceleration=fmax(.5,a->drag_accel+grav*sin(predicted_fpa*DEG2RAD));
        double handoff_eta=a->speed>set->taem_force_handoff_speed?
            (a->speed-set->taem_force_handoff_speed)/net_deceleration:0.0;
        double latest_delivery_fpa=clampd(delivery_target->flight_path_angle+
            TAEM_INTERFACE_FPA_DEBT_LIMIT_DEG,-24.0,-2.5);
        double required_sink_delta=a->speed*fmax(0.0,
            sin(predicted_fpa*DEG2RAD)-sin(latest_delivery_fpa*DEG2RAD));
        double hold_bank=clampd(fabs(g->actual_bank),0.0,89.0)*DEG2RAD;
        double sink_authority=grav-a->lift_accel*cos(hold_bank);
        double fpa_recovery_eta=sink_authority>.25?required_sink_delta/sink_authority:INFINITY;
        terminal_roll_has_delivery_time=handoff_eta>reversal_time+fpa_recovery_eta;
    }
    bool terminal_deadline_roll_ready=terminal_reversal_deadline&&terminal_roll_has_delivery_time;
    double terminal_final_sign=fabs(g->sign)>.1?-g->sign:(he>=0.0?1.0:-1.0);
    bool terminal_arc_ready=!delivery_target||predictor_terminal_arc_ready(geo,course,p,site,
        delivery_target,terminal_final_sign);
    bool terminal_reversal_due=terminal_deadline_roll_ready&&terminal_energy_ready&&
        terminal_altitude_ready&&terminal_arc_ready;
    bool measured_reversal_due=g->sign*he<-heading_corridor;
    /* Predictor and live MM304 must agree on when a nonfinal sign change is executable.
       The failed live run published an early reversal time while the vehicle had only a
       fraction of the setup-side crossrange that live guidance requires, so replay and
       execution diverged by tens of seconds. Apply the same chord-aware setup contract
       here before exporting or replaying that event. */
    Telemetry reversal_geometry={0};
    runway_coordinates(geo,target,site->runway_heading,p->radius,
        &reversal_geometry.runway_along_track,&reversal_geometry.runway_cross_track);
    double candidate_sign=fabs(g->sign)>.1?-g->sign:0.0;
    bool shaping_ready=!delivery_target||entry_taem_shaping_crossrange_ready(delivery_target,
        &reversal_geometry,p,&shaping_cfg,candidate_sign,NULL,NULL);
    bool shaping_endpoint_ready=!delivery_target||predictor_terminal_arc_ready(geo,course,p,site,
        delivery_target,candidate_sign);
    /* Once the terminal deadline is active, keep building the current S-turn until
       the opposite-side arc can actually reach the precomputed tangent without a
       second bank reversal. The same rule now also protects an ordinary corridor
       reversal that would switch onto the selected terminal side too early. */
    if(delivery_target&&measured_reversal_due&&!shaping_ready&&!shaping_endpoint_ready)
        measured_reversal_due=false;
    if(delivery_target&&terminal_reversal_deadline&&!terminal_arc_ready)measured_reversal_due=false;
    bool event_due=g->replay_event&&s.ut>=g->replay_event_ut;
    bool replay_arc_ready=!delivery_target||
        predictor_terminal_arc_ready(geo,course,p,site,delivery_target,g->replay_event_sign);
    bool replay_shaping_ready=!delivery_target||entry_taem_shaping_crossrange_ready(delivery_target,
        &reversal_geometry,p,&shaping_cfg,g->replay_event_sign,NULL,NULL);
    /* Final events require the actual tangent arc. Nonfinal committed events use the
       same live OR-contract: sufficient setup crossrange or a directly reachable
       endpoint may release the sign change. */
    bool replay_geometry_ready=g->replay_event_final?replay_arc_ready:
        (replay_shaping_ready||replay_arc_ready);
    bool replay_due=event_due&&replay_geometry_ready;
    bool deadline_action=delivery_target?terminal_reversal_due:terminal_deadline_roll_ready;
    bool reverse_due=g->replay_event?replay_due:(measured_reversal_due||deadline_action);
    bool reversal_dwell_ready=minleg||(!g->replay_event&&deadline_action&&configured_minleg);
    if(!g->final_heading_lock&&mag>=4&&reversal_dwell_ready&&reverse_due){
        double old_sign=g->sign;
        bool tangent_final_available=delivery_target!=NULL;
        bool legacy_final_available=isfinite(predictor_terminal_hac_radius(a,p,c,v,set,maxbank));
        bool final_reversal=g->replay_event?
            (g->replay_event_final&&(tangent_final_available||legacy_final_available)):
            (terminal_reversal_due&&(tangent_final_available||legacy_final_available));
        double event_sign;
        if(g->replay_event)event_sign=g->replay_event_sign;
        else if(final_reversal&&tangent_final_available){
            /* Nominal MM304 terminal topology contains an actual final reversal.
               Its timing is delayed until the opposite-side arc can join the tangent
               line monotonically, so exporting that opposite side is now consistent
               with the trajectory the predictor continues to fly. */
            event_sign=terminal_final_sign;
        }else event_sign=-g->sign;
        predictor_change_sign(g,event_sign);
        g->replay_event=false;
        if(final_reversal)g->final_heading_lock=true;
        bool side_changed=old_sign!=0&&g->sign!=old_sign;
        if(!g->has_first_reversal&&(side_changed||final_reversal)){
            g->has_first_reversal=true;
            g->first_reversal_final=final_reversal;
            g->first_reversal_ut=s.ut;
            g->first_reversal_range=range;
            g->first_reversal_sign=g->sign;
        }
    }

    double qr=a->q/fmax(v->maximum_dynamic_pressure,1),gr=(a->non_gravity/fmax(grav,.1))/fmax(v->maximum_g_load,.1);
    double protective_aoa=entry_low_q_protective_aoa_floor(a->q,v);
    double aoa=forced_policy?clampd(g->forced_aoa,protective_aoa,v->maximum_angle_of_attack):
        v->entry_angle_of_attack+(v->maximum_angle_of_attack-v->entry_angle_of_attack)*drag_deficit;
    /* Vertical debt is paid with bank/path geometry. The predictor must mirror
       live guidance and never buy sink by exposing the orbiter at low incidence. */
    aoa=fmax(aoa,protective_aoa);
    if(!forced_policy&&a->mach>4&&qr<.45&&geo.altitude<=desired_range_altitude+300.0)
        aoa=fmax(aoa,v->maximum_angle_of_attack-1.0);
    if(qr>.92){aoa=fmin(aoa+1.0,v->maximum_angle_of_attack);mag=fmin(v->maximum_bank_angle,mag+5);}
    if(gr>.92){aoa-=clampd((gr-.9)*18,2,7);mag*=.6;}
    aoa=clampd(aoa,0,v->maximum_angle_of_attack);if(aoa_out)*aoa_out=aoa;
    mag=fmin(mag,env);
    FlightControlRegime regime=flight_control_regime(a->mach,a->speed,a->q,v->minimum_safe_speed);double rs=fmax(.42,regime.command_rate_scale),as=fmax(.30,regime.command_accel_scale);
    double command_mag=forced_policy?mag:mag/clampd(c->bank_effectiveness,.55,1.45);
    double targetbank;
    if(g->final_heading_lock&&g->has_interface_target&&g->interface_target.valid){
        /* Final heading lock is a line-capture controller, not a second S-turn.
           Use the shared course-rate inversion and permit only a small opposite-bank
           correction after the perpendicular course has been acquired. */
        double response=fmax(4.0,g->interface_target.response_time);
        targetbank=taem_course_rate_bank_command(a->speed,a->lift_accel,c->bank_effectiveness,
            env,course,bearing,response);
        if(targetbank*g->sign<0.0&&fabs(targetbank)>18.0)targetbank=-g->sign*18.0;
    }else if(g->final_heading_lock){
        /* Legacy no-interface recovery only. Nominal MM304 always has a tangent
           target and therefore never previews or owns a HAC here. */
        double hac_radius=predictor_terminal_hac_radius(a,p,c,v,set,maxbank);
        if(isfinite(hac_radius)){
            double side=g->sign>=0?1:-1;
            HACGuidance h=hac_guidance_compute_radius(geo,a->speed,course,site,set,p->radius,side,grav,hac_radius);
            double lateral=h.lateral_acceleration,eff=clampd(c->bank_effectiveness,.35,1.8);
            double ratio=fabs(lateral)/fmax(a->lift_accel,.01);
            targetbank=copysign(asin(clampd(ratio,0,sin(clampd(env*eff,0,89)*DEG2RAD)))/eff*RAD2DEG,lateral);
            targetbank=clampd(targetbank,-env,env);
        }else targetbank=g->sign*fmin(env,command_mag);
    }else targetbank=g->sign*fmin(env,command_mag);
    double limited=norm_signed_deg(jerk_angle_update(&g->limiter,targetbank,set->entry_roll_rate*rs,set->entry_roll_acceleration*as,dt));
    double predicted_g_force=a->non_gravity/fmax(planet_surface_gravity(p),.1);
    double predicted_stall_fraction=vessel_physics_conservative_stall_fraction(a->speed,g->actual_aoa,a->q,v);
    if(!g->entry_loaded&&a->non_gravity>=1.5)g->entry_loaded=true;
    bool aerodynamic_authority=entry_s_turn_bank_authority_available(a->q,a->speed,
        predicted_stall_fraction,predicted_g_force,v);
    bool loaded=g->entry_loaded||aerodynamic_authority;
    double magnitude=fabs(limited),threshold=fmin(12,fmax(4,magnitude*.35));
    bool meaningful=loaded&&magnitude>=threshold;
    bool captured=meaningful&&g->sign*g->actual_bank>0&&fabs(g->actual_bank)>=threshold;
    if(!captured){g->established=false;g->leg_elapsed=0;}
    else if(!g->established){g->established=true;g->leg_elapsed=0;}
    else g->leg_elapsed+=dt;
    return limited;
}

static double predictor_local_log_density_gradient(const PlanetModel*p,double altitude){
    if(!p||p->atmosphere_sample_count<2||!isfinite(altitude))return 0.0; /* decision-literal-ok: interpolation requires two samples */
    size_t hi=1;
    while(hi<p->atmosphere_sample_count&&p->atmosphere_altitude[hi]<altitude)hi++;
    if(hi>=p->atmosphere_sample_count)hi=p->atmosphere_sample_count-1;
    size_t lo=hi-1;
    double h0=p->atmosphere_altitude[lo],h1=p->atmosphere_altitude[hi];
    double r0=p->atmosphere_density[lo],r1=p->atmosphere_density[hi];
    if(!(h1>h0)||!(r0>0.0)||!(r1>0.0))return 0.0;
    return fabs(log(r1/r0)/(h1-h0));
}

static double predictor_step_size(const AtmosState*a,const PlanetModel*p,
        const VehicleProfile*v,const GuidanceSettings*set,double altitude,
        double vertical_speed,double entry_alt,bool entered,bool reached_taem,
        double roll_rate,double base){
    (void)v;(void)set;(void)reached_taem;
    if(!(base>0.0)||!isfinite(base))return 0.0;

    double dt=base;
    if(a&&a->density>0.0){
        if(a->non_gravity>DBL_EPSILON&&a->speed>DBL_EPSILON){
            double acceleration_timescale=a->speed/a->non_gravity;
            if(isfinite(acceleration_timescale)&&acceleration_timescale>0.0)
                dt=fmin(dt,acceleration_timescale);
        }

        double gradient=predictor_local_log_density_gradient(p,altitude);
        if(gradient>DBL_EPSILON&&fabs(vertical_speed)>DBL_EPSILON){
            double density_timescale=1.0/(gradient*fabs(vertical_speed));
            if(isfinite(density_timescale)&&density_timescale>0.0)
                dt=fmin(dt,density_timescale);
        }
    }

    if(fabs(roll_rate)>DBL_EPSILON){
        /* One radian is the natural angular state scale; resolve any faster
           roll motion rather than switching on arbitrary deg/s bands. */
        double roll_timescale=1.0/(fabs(roll_rate)*DEG2RAD);
        if(isfinite(roll_timescale)&&roll_timescale>0.0)
            dt=fmin(dt,roll_timescale);
    }

    if(!entered&&vertical_speed<0.0&&altitude>entry_alt){
        double event_time=(altitude-entry_alt)/(-vertical_speed);
        if(isfinite(event_time)&&event_time>0.0)
            dt=fmin(dt,event_time);
    }

    /* Lower bound is purely floating-point/numerical, not flight policy. */
    double numerical_floor=base*sqrt(DBL_EPSILON); /* decision-literal-ok: numerical precision floor */
    return fmax(numerical_floor,fmin(dt,base));
}

static bool predictor_terminal_hac_radius_bounds(const AtmosState*a,
        const PlanetModel*p,const TrajectoryCalibrationModel*c,
        const VehicleProfile*v,const GuidanceSettings*set,double maxbank,
        double*minimum_out,double*maximum_out){
    if(!a||!p||!c||!v||!set||!(a->speed>0.0))return false;

    double usable_bank=bank_limit(a,p,v,maxbank);
    if(!(usable_bank>0.0))return false;
    double live_turn=turn_radius_for_bank(a,c,usable_bank);
    if(!isfinite(live_turn)||!(live_turn>0.0))return false;

    double design_speed=entry_taem_speed_target(v,set,p);
    double minimum_speed=fmax(v->minimum_safe_speed,v->final_approach_speed);
    design_speed=fmax(design_speed,minimum_speed);
    if(!(design_speed>0.0))return false;

    /*
     * At unchanged aerodynamic state turn radius scales with V^2.  This is the
     * only scaling needed here; the actual candidate must still pass the full
     * maneuver-feasibility test below.
     */
    double scale=design_speed/a->speed;
    double minimum_radius=live_turn*scale*scale;
    double maximum_radius=p->radius; /* geometric limit of the local body */
    if(!isfinite(minimum_radius)||!(minimum_radius>0.0)||
       !isfinite(maximum_radius)||maximum_radius<minimum_radius)
        return false;

    if(minimum_out)*minimum_out=minimum_radius;
    if(maximum_out)*maximum_out=maximum_radius;
    return true;
}

static double predictor_terminal_hac_radius(const AtmosState*a,
        const PlanetModel*p,const TrajectoryCalibrationModel*c,
        const VehicleProfile*v,const GuidanceSettings*set,double maxbank){
    double minimum_radius=0.0,maximum_radius=0.0;
    if(!predictor_terminal_hac_radius_bounds(
            a,p,c,v,set,maxbank,&minimum_radius,&maximum_radius))
        return INFINITY;
    /*
     * Legacy callers need one representative radius only to keep a continuous
     * path law alive.  The nominal configured radius is an objective, not a
     * feasibility threshold; clamp it only to the physical authority interval.
     */
    return clampd(set->hac_radius,minimum_radius,maximum_radius);
}

static double predictor_hac_candidate_residual(const PredictorGuidance*g,
        GeoPoint geo,const AtmosState*a,double course,const PlanetModel*p,
        const VehicleProfile*v,const LandingSite*site,const GuidanceSettings*set,
        double hac_side,double hac_radius,double minimum_turn_radius,
        double*path_out){
    HACGuidance h=hac_guidance_compute_radius(
        geo,a->speed,course,site,set,p->radius,hac_side,
        planet_surface_gravity(p),hac_radius);
    double path=0.0,slope=0.0;
    double offset=fabs(norm_signed_deg(course-site->runway_heading));
    bool maneuver=taem_alignment_maneuver_feasible(
        geo.altitude,a->speed,hac_radius,minimum_turn_radius,
        a->drag_accel,offset,p,site,v,set,&path,&slope);
    if(path_out)*path_out=path;

    double minimum_speed=fmax(v->minimum_safe_speed,v->final_approach_speed);
    double target_speed=entry_taem_speed_target(v,set,p);
    double remaining=g->hac_progress_valid?g->hac_remaining:h.arc_remaining;
    double desired=h.desired_altitude+
        (remaining-h.arc_remaining)*tan(set->taem_glide_slope*DEG2RAD);
    double minimum_energy=entry_remaining_specific_energy(
        geo.latitude,geo.altitude,a->speed,site->latitude,desired,
        minimum_speed,p);

    double radius_residual=fabs(h.radial_error)/fmax(hac_radius,DBL_MIN);
    double course_residual=fabs(h.course_error)/180.0;
    double speed_residual=a->speed>=minimum_speed?0.0:
        (minimum_speed-a->speed)/fmax(minimum_speed,DBL_MIN);
    double energy_scale=fmax(
        planet_surface_gravity(p)*fmax(geo.altitude-site->altitude,0.0),
        .5*fmax(a->speed*a->speed,DBL_MIN));
    double energy_residual=isfinite(minimum_energy)?
        fmax(0.0,-minimum_energy)/fmax(energy_scale,DBL_MIN):INFINITY;
    double maneuver_residual=maneuver?0.0:1.0;

    (void)target_speed;(void)slope;
    return fmax(radius_residual,
        fmax(course_residual,
            fmax(speed_residual,
                fmax(energy_residual,maneuver_residual))));
}

static bool predictor_hac_ready(const PredictorGuidance*g,VehicleState state,
        GeoPoint geo,const AtmosState*a,const PlanetModel*p,
        const TrajectoryCalibrationModel*c,const VehicleProfile*v,
        const LandingSite*site,const GuidanceSettings*set,double maxbank,
        double hac_side,double*radius_out,double*score_out){
    GeoPoint sp={site->latitude,site->longitude,site->altitude};
    double course=surface_course(state.position,state.velocity,
        planet_rotation_vector(p),p->north_axis,initial_bearing(geo,sp));

    double minimum_radius=0.0,maximum_radius=0.0;
    if(!predictor_terminal_hac_radius_bounds(
            a,p,c,v,set,maxbank,&minimum_radius,&maximum_radius))
        return false;
    if(radius_out)*radius_out=INFINITY;
    if(score_out)*score_out=INFINITY;

    double minimum_speed=fmax(v->minimum_safe_speed,v->final_approach_speed);
    if(a->speed<minimum_speed)return false;

    double usable_bank=bank_limit(a,p,v,maxbank);
    double minimum_turn=turn_radius_for_bank(a,c,usable_bank);
    if(!isfinite(minimum_turn))return false;

    bool found=false;
    double best_radius=INFINITY,best_delivery=INFINITY;
    const int samples=25; /* numerical resolution only */
    double log_min=log(minimum_radius);
    double log_max=log(maximum_radius);

    for(int i=0;i<samples;i++){
        double f=(double)i/(double)(samples-1);
        double hac_radius=exp(log_min+(log_max-log_min)*f);
        double path=0.0;
        double residual=predictor_hac_candidate_residual(
            g,geo,a,course,p,v,site,set,hac_side,hac_radius,
            minimum_turn,&path);
        bool ready=isfinite(residual)&&residual<=sqrt(DBL_EPSILON);
        double delivery=isfinite(path)&&a->speed>DBL_MIN?
            path/a->speed:INFINITY;
        if(ready&&(!found||delivery<best_delivery)){
            found=true;
            best_radius=hac_radius;
            best_delivery=delivery;
        }

        if(getenv("KSP_LANDER_TRACE_TAEM")&&i==samples-1){
            fprintf(stderr,
                "taem-hac feasibility: alt=%.0f speed=%.0f R=[%.0f,%.0f]km side=%+.0f best=%s residual=%.4f\n",
                geo.altitude,a->speed,minimum_radius/1000.0,
                maximum_radius/1000.0,hac_side,
                found?"yes":"no",residual);
        }
    }

    if(found){
        if(radius_out)*radius_out=best_radius;
        if(score_out)*score_out=best_delivery;
    }
    return found;
}

static double predictor_hac_soft_cost(const PredictorGuidance*g,
        VehicleState state,GeoPoint geo,const AtmosState*a,
        const PlanetModel*p,const TrajectoryCalibrationModel*c,
        const VehicleProfile*v,const LandingSite*site,
        const GuidanceSettings*set,double maxbank,double hac_side,
        double hac_radius){
    GeoPoint sp={site->latitude,site->longitude,site->altitude};
    double course=surface_course(state.position,state.velocity,
        planet_rotation_vector(p),p->north_axis,initial_bearing(geo,sp));
    double usable_bank=bank_limit(a,p,v,maxbank);
    double minimum_turn=turn_radius_for_bank(a,c,usable_bank);
    if(!isfinite(minimum_turn))return INFINITY;
    return predictor_hac_candidate_residual(
        g,geo,a,course,p,v,site,set,hac_side,hac_radius,
        minimum_turn,NULL);
}

static double post_taem_bank(PredictorGuidance*g,VehicleState state,GeoPoint geo,const AtmosState*a,const PlanetModel*p,const TrajectoryCalibrationModel*c,const VehicleProfile*v,const LandingSite*site,const GuidanceSettings*set,double maxbank,double hac_side,double dt){
    GeoPoint sp={site->latitude,site->longitude,site->altitude};
    double course=surface_course(state.position,state.velocity,planet_rotation_vector(p),p->north_axis,initial_bearing(geo,sp));
    double along=0,cross=0;runway_coordinates(geo,sp,site->runway_heading,p->radius,&along,&cross);
    Vector3 up=vnorm(state.position,v3(0,1,0));
    double vertical=vdot(a->air_velocity,up),horizontal=vmag(vproject_plane(a->air_velocity,up));
    double fpa=atan2(vertical,fmax(horizontal,.1))*RAD2DEG,range=great_circle_distance(geo,sp,p->radius);
    double gravity=planet_surface_gravity(p);
    double hac_radius=g->hac_radius>0?g->hac_radius:set->hac_radius;
    HACGuidance h=hac_guidance_compute_radius(geo,a->speed,course,site,set,p->radius,hac_side,gravity,hac_radius);

    bool fresh_hac=!g->hac_progress_valid;
    if(fresh_hac){
        g->hac_remaining=h.arc_remaining;
        g->hac_previous_angle=h.angle;
        g->hac_progress_valid=fabs(h.radial_error)<=fmax(250,fmin(1500.0,hac_radius*.10))&&fabs(h.course_error)<24;
    }else if(!g->hac_completed){
        double progress=-hac_side*norm_signed_deg((h.angle-g->hac_previous_angle)*RAD2DEG)*DEG2RAD;
        g->hac_remaining-=progress*hac_radius;
        g->hac_previous_angle=h.angle;
    }

    g->hac_previous_angle=h.angle;
    double lead=clampd(horizontal*2.0,180.0,2400.0),station=-set->final_approach_distance;
    double min_turn=turn_radius_for_bank(a,c,fmin(55.0,bank_limit(a,p,v,maxbank)));
    bool approach=terminal_approach_valid(geo,range,along,cross,course,a->speed,horizontal,vertical,fpa,site,v,set);
    if(fresh_hac&&along>=station-lead&&along<=station+fmax(200,hac_radius*.012)&&
       g->hac_remaining>LANDER_PI*hac_radius)g->hac_remaining-=2*LANDER_PI*hac_radius;
    if(!g->hac_completed&&!approach&&g->hac_remaining<=lead+fmax(200,hac_radius*.012)&&
       along>=station-lead&&along<=station+fmax(200,hac_radius*.012)&&
       fabs(cross)<fmax(250,fmin(1000,hac_radius*.10))&&fabs(norm_signed_deg(course-site->runway_heading))<12){
        double arc=0,slope=0;
        if(hac_high_pass_circuit(geo.altitude,a->speed,hac_radius,g->hac_remaining,min_turn,a->drag_accel,
            p,site,v,set,&arc,&slope)){
            g->hac_remaining=arc;g->hac_circuit_slope=slope;g->hac_progress_valid=true;
        }
    }
    double slope=g->hac_circuit_slope>0?g->hac_circuit_slope:set->taem_glide_slope;
    double desired_altitude=site->altitude+set->final_approach_distance*tan(set->final_glide_slope*DEG2RAD)+
        (g->hac_remaining+fabs(h.radial_error)*.65)*tan(slope*DEG2RAD);
    double path=fmax(set->final_approach_distance,g->hac_remaining+set->final_approach_distance);
    double target_speed=fmax(v->final_approach_speed*1.12,v->minimum_safe_speed*1.08);
    double energy_error=entry_remaining_specific_energy(geo.latitude,geo.altitude,a->speed,site->latitude,site->altitude,target_speed,p);
    double removable=fmax(.03,a->drag_accel)*path*1.25;
    bool energy_ok=fabs(geo.altitude-desired_altitude)<=8000&&a->speed>=v->minimum_safe_speed&&isfinite(energy_error)&&energy_error>=-.04*target_speed*target_speed&&energy_error<=removable;
    bool hac_captured=min_turn*1.15<=hac_radius&&energy_ok&&fabs(h.radial_error)<=fmax(250,fmin(1500.0,hac_radius*.10))&&fabs(h.course_error)<24;
    double exit_window=fmax(200,hac_radius*.012);
    bool lost=!isfinite(min_turn)||min_turn*1.15>hac_radius||
        fabs(h.radial_error)>fmax(750.0,fmin(5000.0,hac_radius*.25))||fabs(h.course_error)>60;
    g->hac_lost_duration=lost?g->hac_lost_duration+dt:fmax(0,g->hac_lost_duration-dt*2);
    bool missed=!approach&&along>station+exit_window&&fabs(norm_signed_deg(course-site->runway_heading))<45&&
        (!g->hac_progress_valid||g->hac_remaining<=exit_window);
    if(!g->final_captured&&(g->hac_lost_duration>=5||missed)){
        /* Terrain is unavailable in the reduced model: site-relative height
           approximates the live radar-height guard. Arrival metrics remain
           historical; only the active guidance ownership is reset. */
        if(geo.altitude>=fmax(12000.0,set->taem_interface_altitude*.5)&&
           geo.altitude-site->altitude>=8000&&a->speed>=v->minimum_safe_speed*2){
            g->hac_return_to_entry=true;g->terminal_reentry_after_ut=state.ut+15;
            g->hac_progress_valid=false;g->hac_completed=false;g->hac_remaining=0;g->hac_radius=0;
            g->hac_circuit_slope=0;g->hac_lost_duration=0;g->final_heading_lock=false;
            g->replay_event=false;g->force_control=false;g->established=false;
            if(fabs(g->actual_bank)>=4)g->sign=g->actual_bank>=0?1:-1;
            return norm_signed_deg(jerk_angle_update(&g->limiter,0,set->entry_roll_rate,set->entry_roll_acceleration,dt));
        }
        if(missed)g->hac_failed=true;
    }
    if(!g->hac_completed&&g->hac_progress_valid&&g->hac_remaining<=exit_window){
        if(g->hac_remaining>=-exit_window&&hac_captured&&approach&&along<0){g->hac_completed=true;g->hac_remaining=0;}
        /* Preserve signed overshoot; never recycle the exit window. */
    }

    if(g->hac_completed&&approach&&along<0)g->final_captured=true;

    double raw,rate,acc,floor;
    if(g->final_captured){
        double dist=fmax(0,-along),look=clampd(a->speed*5.5,450,2800);
        double inter=clampd(atan2(-cross,look)*RAD2DEG,-28,28),target=norm_deg(site->runway_heading+inter);
        double err=norm_signed_deg(target-course),lat=2*horizontal*horizontal/fmax(look,1)*sin(err*DEG2RAD);
        double frac=clampd(dist/fmax(set->final_approach_distance,1),0,1),lim=fmin(30,bank_limit(a,p,v,maxbank));
        lim=fmin(lim,8+22*sqrt(frac));
        double radar=fmax(0,geo.altitude-site->altitude);
        if(radar<set->flare_altitude*4)lim=fmin(lim,6+10*clampd(radar/fmax(set->flare_altitude*4,1),0,1));
        raw=clampd(atan(lat/fmax(gravity,.1))*RAD2DEG,-lim,lim);
        rate=set->approach_roll_rate;acc=fmax(1.4,set->entry_roll_acceleration*.55);floor=.60;
    }else{
        double lateral=h.lateral_acceleration;
        double eff=clampd(c->bank_effectiveness,.35,1.8);
        double ratio=fabs(lateral)/fmax(a->lift_accel,.01);
        raw=copysign(asin(clampd(ratio,0,sin(clampd(fmin(maxbank,75)*eff,0,89)*DEG2RAD)))/eff*RAD2DEG,lateral);
        raw=clampd(raw,-maxbank,maxbank);
        rate=set->taem_roll_rate;acc=fmax(2,set->entry_roll_acceleration*.85);floor=.50;
    }
    FlightControlRegime regime=flight_control_regime(a->mach,a->speed,a->q,v->minimum_safe_speed);
    rate*=fmax(floor,regime.command_rate_scale);acc*=fmax(floor*.72,regime.command_accel_scale);
    return norm_signed_deg(jerk_angle_update(&g->limiter,raw,rate,acc,dt));
}

void entry_prediction_clear(EntryPrediction*p){if(!p)return;trajectory_clear(&p->trajectory);memset(p,0,sizeof(*p));}
static EntryPrediction predictor_simulate_entry_core(VehicleState state,const PlanetModel*p,AerodynamicModel aero,const AerodynamicEnvelope*env0,const TrajectoryCalibrationModel*cal0,const VehicleProfile*v,const LandingSite*site,const GuidanceSettings*set,double maxbank,double initial_bank,double initial_bank_rate,double initial_aoa,double initial_aoa_rate,double initial_sign,double initial_leg,bool initial_final_heading_lock,double maxdur,bool include,bool force_control,double forced_bank,double forced_aoa,double forced_duration,double control_horizon_duration,bool stop_at_taem,bool enforce_terminal_delivery_budget,const EntryControlPlan*replay,const TaemInterfaceTarget*interface_target){
    EntryPrediction out;memset(&out,0,sizeof(out));trajectory_init(&out.trajectory);out.closest_distance=DBL_MAX;out.taem_distance=DBL_MAX;out.taem_hac_capture_score=DBL_MAX;out.best_terminal_capture_cost=DBL_MAX;out.taem_altitude=NAN;out.taem_desired_altitude=NAN;out.minimum_entry_speed=DBL_MAX;out.minimum_bank_control_margin=INFINITY;out.minimum_aoa_control_margin=INFINITY;
    AerodynamicEnvelope uniform;if(!env0){for(int i=0;i<4;i++)uniform.regimes[i]=aero;env0=&uniform;}
    TrajectoryCalibrationModel dc={
        .density_scale=1,.drag_scale=1,.lift_scale=1,.bank_effectiveness=1,.speed_of_sound=340,
        .speed_of_sound_scale=1,
        .altitude_residual=0,.speed_residual=0,.range_residual=0,.confidence=.05,.accepted_samples=0
    };if(!cal0)cal0=&dc;
    PredictorGuidance g;memset(&g,0,sizeof(g));g.sign=fabs(initial_sign)>.1?(initial_sign>=0?1:-1):0;g.leg_elapsed=fmax(0,initial_leg);g.established=initial_leg>0;g.final_heading_lock=initial_final_heading_lock;g.limiter.has_value=true;g.limiter.value=norm_deg(initial_bank);g.limiter.rate=0;g.actual_bank=norm_signed_deg(initial_bank);g.actual_bank_rate=clampd(initial_bank_rate,-60,60);g.actual_aoa=isfinite(initial_aoa)?clampd(initial_aoa,-v->maximum_angle_of_attack*1.5,v->maximum_angle_of_attack*1.5):scheduled_aoa(vmag(state.position)-p->radius,p,v);g.actual_aoa_rate=isfinite(initial_aoa_rate)?clampd(initial_aoa_rate,-15,15):0;g.force_control=force_control&&forced_duration>0;g.forced_until_ut=state.ut+fmax(0,forced_duration);g.control_horizon_ut=state.ut+fmax(0,control_horizon_duration);g.forced_bank=forced_bank;g.forced_aoa=forced_aoa;g.enforce_terminal_delivery_budget=enforce_terminal_delivery_budget;
    g.entry_loaded=initial_leg>0;
    g.configured_leg_dwell_satisfied=initial_leg+1e-6>=set->s_turn_minimum_leg_duration;
    if(interface_target&&interface_target->valid){g.has_interface_target=true;g.interface_target=*interface_target;}
    LandingConfiguration interface_cfg={.site=*site,.vehicle=*v,.guidance=*set};
    /* Never seed an established leg on the opposite measured bank. */
    if(g.sign*g.actual_bank<=0||fabs(g.actual_bank)<4){g.established=false;g.leg_elapsed=0;}
    if(replay&&replay->has_planned_reversal&&isfinite(replay->planned_reversal_ut)&&
            fabs(replay->planned_reversal_sign)>.1){
        g.replay_event=true;g.replay_event_ut=replay->planned_reversal_ut;
        g.replay_event_sign=replay->planned_reversal_sign;
        g.replay_event_final=replay->planned_reversal_is_final;
    }
    GeoPoint target={site->latitude,site->longitude,site->altitude};double entry_alt=entry_guidance_start_altitude(p,set),elapsed=0,lastsample=-DBL_MAX,hac_side=1;bool hac_set=false;
    double mm304_gate_altitude=fmin(50000.0,entry_alt-5000.0);
    bool initial_gear=cal0->physics?cal0->physics->gear:false,initial_brakes=cal0->physics?cal0->physics->brakes:false;
    int initial_airbrakes=cal0->physics&&cal0->physics->airbrakes>=0?cal0->physics->airbrakes:0;
    speedbrake_controller_reset(&g.speedbrake,initial_airbrakes!=0);
    AeroContext ctx={.p=p,.e=env0,.c=cal0,.v=v,.bank=g.actual_bank,.aoa=g.actual_aoa,.ut=state.ut,.mass=state.mass,.gear=initial_gear,.brakes=initial_brakes,.airbrakes=initial_airbrakes};
    while(elapsed<maxdur){
        GeoPoint geo=predictor_geo_point(state.position,p,state.ut);double range=great_circle_distance(geo,target,p->radius);
        AtmosState at=atmosphere(state.position,state.velocity,&ctx);Vector3 up=vnorm(state.position,v3(0,1,0));double vs=vdot(at.air_velocity,up),hs=vmag(vproject_plane(at.air_velocity,up)),fpa=atan2(vs,fmax(hs,.1))*RAD2DEG;
        if(force_control&&!out.control_horizon_recorded&&state.ut>=g.control_horizon_ut){
            out.control_horizon_recorded=true;out.control_horizon_speed=at.speed;out.control_horizon_altitude=geo.altitude;out.control_horizon_range=range;
            out.control_horizon_specific_energy=rotating_specific_energy(geo.latitude,geo.altitude,at.speed,p);
            double along=0,cross=0;runway_coordinates(geo,target,site->runway_heading,p->radius,&along,&cross);
            out.control_horizon_alignment_distance=-along-set->final_approach_distance;
        }
        if(!out.entered_atmosphere&&geo.altitude<=entry_alt&&vs<0){out.entered_atmosphere=true;out.entry_range=range;out.entry_flight_path_angle=fpa;out.entry_speed=at.speed;double course=surface_course(state.position,state.velocity,planet_rotation_vector(p),p->north_axis,initial_bearing(geo,target));out.entry_course_error=norm_signed_deg(initial_bearing(geo,target)-course);}
        if(out.entered_atmosphere&&!out.mm304_gate_recorded&&geo.altitude<=mm304_gate_altitude&&vs<0.0){
            double gate_along=0.0,gate_cross=0.0;
            runway_coordinates(geo,target,site->runway_heading,p->radius,&gate_along,&gate_cross);
            double gate_course=surface_course(state.position,state.velocity,planet_rotation_vector(p),p->north_axis,initial_bearing(geo,target));
            out.mm304_gate_recorded=true;out.mm304_gate_range=range;
            out.mm304_gate_along_track=gate_along;out.mm304_gate_cross_track=gate_cross;
            out.mm304_gate_course=gate_course;out.mm304_gate_altitude=geo.altitude;
            out.mm304_gate_speed=at.speed;out.mm304_gate_flight_path_angle=fpa;
        }
        /* Closest approach is an entry-trajectory quantity. Counting orbital
           ground-track passes before atmospheric interface can falsely mark
           the TAEM shell as already crossed and can also make a bad deorbit
           candidate look like a close KSC capture. */
        if(out.entered_atmosphere)out.closest_distance=fmin(out.closest_distance,range);
        double ownership_along=0.0,ownership_cross=0.0;
        bool ownership_ready=false,ownership_boundary_missed=false;
        if(stop_at_taem&&out.entered_atmosphere){
            runway_coordinates(geo,target,site->runway_heading,p->radius,&ownership_along,&ownership_cross);
            if(g.has_interface_target){
                Telemetry capture_state={0};
                capture_state.latitude=geo.latitude;capture_state.mean_altitude=geo.altitude;
                capture_state.true_air_speed=at.speed;capture_state.horizontal_speed=hs;
                capture_state.vertical_speed=vs;capture_state.flight_path_angle=fpa;
                capture_state.runway_along_track=ownership_along;capture_state.runway_cross_track=ownership_cross;
                capture_state.dynamic_pressure=at.q;capture_state.g_force=at.non_gravity/fmax(.1,planet_surface_gravity(p));
                capture_state.bank_effectiveness=cal0->bank_effectiveness;
                double capture_course=surface_course(state.position,state.velocity,planet_rotation_vector(p),p->north_axis,initial_bearing(geo,target));
                TaemInterfaceCapture capture=entry_taem_interface_capture(
                    &g.interface_target,&capture_state,capture_course,p,&interface_cfg);
                ownership_ready=capture.ready;
                /*
                 * The fixed MM304 point is a one-way ownership surface.  Once
                 * the propagated state has passed it without satisfying the
                 * mission contract, that forecast is a miss.  Energy or minimum
                 * safe-speed deficits are independently irreversible in an
                 * unpowered vehicle.  No downstream/crossrange grace box is
                 * invented here.
                 */
                ownership_boundary_missed=capture.valid&&!capture.ready&&
                    (capture.along<0.0||capture.energy_margin<0.0||
                     at.speed<v->minimum_safe_speed);
            }else{
                ownership_ready=entry_taem_handoff_geometry_ready(geo.altitude,ownership_along,vs,hs,set);
                if(!ownership_ready){
                    double handoff_low=0.0,handoff_high=0.0;
                    entry_taem_handoff_altitude_bounds(set,&handoff_low,&handoff_high);
                    ownership_boundary_missed=
                        ownership_along>=-set->final_approach_distance||
                        geo.altitude<handoff_low;
                }
            }
        }
        if(out.entered_atmosphere&&!out.reached_taem){
            out.minimum_entry_speed=fmin(out.minimum_entry_speed,at.speed);
            out.maximum_abs_angle_of_attack=fmax(out.maximum_abs_angle_of_attack,fabs(g.actual_aoa));
            if(force_control&&state.ut<g.forced_until_ut){
                /* Forced replay evaluates an already-committed live command. Do not
                   apply the autonomous 60 deg candidate-search ceiling here. */
                double available_bank=bank_limit(&at,p,v,maxbank);
                out.minimum_bank_control_margin=fmin(out.minimum_bank_control_margin,available_bank-fabs(g.forced_bank));
                out.minimum_aoa_control_margin=fmin(out.minimum_aoa_control_margin,
                    fmin(g.forced_aoa,v->maximum_angle_of_attack-g.forced_aoa));
            }
        }
        out.peak_dynamic_pressure=fmax(out.peak_dynamic_pressure,at.q);out.peak_g_load=fmax(out.peak_g_load,at.non_gravity/fmax(planet_surface_gravity(p),.1));
        double terminal_radius=INFINITY;
        bool terminal_ready=false;
        if(out.entered_atmosphere&&!hac_set&&state.ut>=g.terminal_reentry_after_ut){
            double grav=planet_surface_gravity(p),candidate_radius=predictor_terminal_hac_radius(&at,p,cal0,v,set,maxbank);
            if(isfinite(candidate_radius)){
                double course=surface_course(state.position,state.velocity,planet_rotation_vector(p),p->north_axis,initial_bearing(geo,target));
                double positive_radius=INFINITY,negative_radius=INFINITY;
                double ps=INFINITY,ns=INFINITY;
                bool positive_ready=predictor_hac_ready(&g,state,geo,&at,p,cal0,v,site,set,maxbank,1,&positive_radius,&ps);
                bool negative_ready=predictor_hac_ready(&g,state,geo,&at,p,cal0,v,site,set,maxbank,-1,&negative_radius,&ns);
                terminal_ready=positive_ready||negative_ready;
                /* Match live acquisition: compare scores only among feasible
                   circles. The geometrically closer side may fail energy. */
                hac_side=terminal_ready?((positive_ready&&(!negative_ready||ps<=ns))?1:-1):(ps<=ns?1:-1);
                terminal_radius=hac_side>0?positive_radius:negative_radius;
                hac_set=terminal_ready;
                double positive_soft=predictor_hac_soft_cost(&g,state,geo,&at,p,cal0,v,site,set,maxbank,1,candidate_radius);
                double negative_soft=predictor_hac_soft_cost(&g,state,geo,&at,p,cal0,v,site,set,maxbank,-1,candidate_radius);
                double soft=fmin(positive_soft,negative_soft);
                if(isfinite(soft))out.best_terminal_capture_cost=fmin(out.best_terminal_capture_cost,soft);
                if(terminal_ready&&!stop_at_taem){
                    g.hac_radius=terminal_radius;
                    HACGuidance h=hac_guidance_compute_radius(geo,at.speed,course,site,set,p->radius,hac_side,grav,terminal_radius);
                    if(!out.reached_taem){
                    double target_speed=entry_taem_speed_target(v,set,p);
                    out.reached_taem=true;out.taem_distance=range;out.taem_range_error=h.radial_error;out.taem_speed=at.speed;out.taem_flight_path_angle=fpa;
                    out.taem_altitude=geo.altitude;out.taem_desired_altitude=set->taem_interface_altitude;
                    runway_coordinates(geo,target,site->runway_heading,p->radius,&out.taem_along_track,&out.taem_cross_track);
                    out.taem_course=course;
                    /* Entry owns energy management only down to the TAEM
                       interface. The previous residual still targeted final-
                       approach speed here, so a perfectly valid 500-700 m/s
                       TAEM handoff was scored as huge excess energy and MPC
                       kept dragging the shuttle long after it should have
                       handed control to HAC. TAEM/HAC owns the subsequent
                       deceleration toward final approach. */
                    out.taem_hac_capture_score=hac_side>0?ps:ns;out.taem_energy_error=entry_remaining_specific_energy(geo.latitude,geo.altitude,at.speed,site->latitude,h.desired_altitude,target_speed,p);
                    }
                }
            }
        }
        if(ownership_ready&&!out.reached_taem){
            double target_speed=g.has_interface_target?g.interface_target.speed:entry_taem_speed_target(v,set,p);
            double target_altitude=g.has_interface_target?g.interface_target.altitude:set->taem_interface_altitude;
            out.reached_taem=true;out.taem_distance=range;
            out.taem_range_error=range-entry_taem_range_target(p,set);out.taem_speed=at.speed;out.taem_flight_path_angle=fpa;
            out.taem_altitude=geo.altitude;out.taem_desired_altitude=target_altitude;
            out.taem_along_track=ownership_along;out.taem_cross_track=ownership_cross;
            out.taem_course=surface_course(state.position,state.velocity,planet_rotation_vector(p),p->north_axis,initial_bearing(geo,target));
            out.taem_energy_error=entry_remaining_specific_energy(geo.latitude,geo.altitude,at.speed,site->latitude,
                target_altitude,target_speed,p);
        }
        if(stop_at_taem&&(out.reached_taem||ownership_boundary_missed)){
            out.taem_ownership_boundary_missed=ownership_boundary_missed;
            /* Preserve the exact accepted/missed interface state even when the
               prediction terminates before the normal six-second trace cadence. */
            if(include&&(out.trajectory.count==0||
               fabs(out.trajectory.points[out.trajectory.count-1].ut-state.ut)>1e-6)){
                GuidancePhase stop_phase=out.reached_taem?PHASE_TAEM:PHASE_ENTRY_ENERGY;
                TrajectoryPoint tp={state.ut,geo.latitude,geo.longitude,geo.altitude,at.speed,stop_phase,TRAJ_PLANNED};
                trajectory_append(&out.trajectory,tp);lastsample=state.ut;
            }
            /* A forced Entry-plan forecast is an executable shadow, not permission
               to invent another Entry segment after live MM304 crosses its one-way
               V_TAEM ownership boundary. A miss stays reached_taem=false so
               supervision cannot discover a later synthetic recovery. */
            break;
        }
        GuidancePhase phase=geo.altitude>entry_alt?PHASE_COAST:g.final_captured?PHASE_FINAL:(g.hac_progress_valid||hac_set)?PHASE_TAEM:PHASE_ENTRY_ENERGY;
        if(include&&state.ut-lastsample>=6){TrajectoryPoint tp={state.ut,geo.latitude,geo.longitude,geo.altitude,at.speed,phase,TRAJ_PLANNED};trajectory_append(&out.trajectory,tp);lastsample=state.ut;}
        if(geo.altitude<=fmax(site->altitude,0)&&vs<0)break;if(out.reached_taem&&range<500&&geo.altitude<1000)break;if(vmag(state.position)<p->radius*.8||!isfinite(vmag(state.position)))break;
        double frac=clampd(geo.altitude/fmax(1,p->atmosphere_depth),0,1),base_dt=geo.altitude>p->atmosphere_depth?8:clampd(.25+frac*1.55,.25,1.8);
        double dt=predictor_step_size(&at,p,v,set,geo.altitude,vs,entry_alt,out.entered_atmosphere,out.reached_taem,g.limiter.rate,base_dt),aoa=NAN;AerodynamicModel local=aero_at(env0,at.mach);
        double bank=0,target_aoa=0;
        if(!out.entered_atmosphere){
            /* Match live Entry Interface: wings level and maximum entry AoA,
               including the thin-air interval before S-turn engagement. */
            FlightControlRegime regime=flight_control_regime(at.mach,at.speed,at.q,v->minimum_safe_speed);
            double rate=set->entry_roll_rate*fmax(.55,regime.command_rate_scale),acc=set->entry_roll_acceleration*fmax(.45,regime.command_accel_scale);
            bank=norm_signed_deg(jerk_angle_update(&g.limiter,0,rate,acc,dt));
            target_aoa=v->maximum_angle_of_attack;
        }else{
            if(out.reached_taem&&hac_set){
                bank=post_taem_bank(&g,state,geo,&at,p,cal0,v,site,set,maxbank,hac_side,dt);
                if(g.hac_return_to_entry){hac_set=false;g.hac_return_to_entry=false;}
                if(g.hac_failed||(!g.final_captured&&g.hac_remaining < -fmax(200,g.hac_radius*.012)))break;
                target_aoa=clampd(scheduled_aoa(geo.altitude,p,v),4,14);
                if(g.hac_circuit_slope>0&&!g.final_captured)
                    target_aoa=clampd(-g.hac_circuit_slope+7.0-fpa,3.0,14.0);
            }else{
                bank=entry_bank(&g,state,geo,&at,p,local,cal0,v,site,set,maxbank,dt,&aoa);
                target_aoa=aoa;
            }
        }
        attitude_step(&g,bank,target_aoa,&at,v,set,dt,cal0->physics);
        if(out.entered_atmosphere&&!out.reached_taem)out.maximum_abs_angle_of_attack=fmax(out.maximum_abs_angle_of_attack,fabs(g.actual_aoa));
        ctx.bank=g.actual_bank;ctx.aoa=g.actual_aoa;ctx.ut=state.ut;
        ctx.airbrakes=predictor_speedbrake_state(&g,geo,&at,p,cal0,v,site,set,state.mass,out.reached_taem,dt);
        /* Wheel brakes belong only to rollout; gear follows the same terminal
           altitude gate as live guidance so direct-force lookup does not use
           clean-airframe cells after predicted deployment. */
        ctx.brakes=false;ctx.gear=out.reached_taem&&geo.altitude-site->altitude<=set->gear_deployment_altitude;
        out.s_turn_max_bank=fmax(out.s_turn_max_bank,fabs(g.actual_bank));out.s_turn_reversals=g.reversals;
        if(at.q>1){if(at.direct_aero)out.physics_observed_seconds+=dt;else out.physics_fallback_seconds+=dt;}
        state=rk4_aero(state,dt,&ctx);elapsed+=dt;
    }
    if(force_control&&!out.control_horizon_recorded){GeoPoint geo=predictor_geo_point(state.position,p,state.ut);AtmosState at=atmosphere(state.position,state.velocity,&ctx);out.control_horizon_recorded=true;out.control_horizon_speed=at.speed;out.control_horizon_altitude=geo.altitude;out.control_horizon_range=great_circle_distance(geo,target,p->radius);out.control_horizon_specific_energy=rotating_specific_energy(geo.latitude,geo.altitude,at.speed,p);
            double along=0,cross=0;runway_coordinates(geo,target,site->runway_heading,p->radius,&along,&cross);
            out.control_horizon_alignment_distance=-along-set->final_approach_distance;}
    out.has_first_s_turn_reversal=g.has_first_reversal;
    out.first_s_turn_reversal_is_final=g.first_reversal_final;
    out.first_s_turn_reversal_ut=g.first_reversal_ut;
    out.first_s_turn_reversal_range=g.first_reversal_range;
    out.first_s_turn_reversal_sign=g.first_reversal_sign;
    if(out.minimum_entry_speed==DBL_MAX)out.minimum_entry_speed=NAN;
    if(!out.reached_taem){GeoPoint geo=predictor_geo_point(state.position,p,state.ut);out.taem_distance=great_circle_distance(geo,target,p->radius);out.taem_range_error=isfinite(out.closest_distance)?out.taem_distance-out.closest_distance:out.taem_distance;out.taem_speed=atmosphere(state.position,state.velocity,&ctx).speed;out.taem_energy_error=INFINITY;}out.final_state=state;return out;
}

EntryPrediction predictor_simulate_entry_with_attitude(VehicleState state,const PlanetModel*p,AerodynamicModel aero,const AerodynamicEnvelope*env,const TrajectoryCalibrationModel*cal,const VehicleProfile*v,const LandingSite*site,const GuidanceSettings*set,double maxbank,double initial_bank,double initial_bank_rate,double initial_aoa,double initial_aoa_rate,double initial_sign,double initial_leg,double maxdur,bool include){
    return predictor_simulate_entry_core(state,p,aero,env,cal,v,site,set,maxbank,initial_bank,initial_bank_rate,initial_aoa,initial_aoa_rate,initial_sign,initial_leg,false,maxdur,include,false,0,0,0,0,false,false,NULL,NULL);
}

EntryPrediction predictor_simulate_entry_with_rate(VehicleState state,const PlanetModel*p,AerodynamicModel aero,const AerodynamicEnvelope*env,const TrajectoryCalibrationModel*cal,const VehicleProfile*v,const LandingSite*site,const GuidanceSettings*set,double maxbank,double initial_bank,double initial_bank_rate,double initial_sign,double initial_leg,double maxdur,bool include){
    return predictor_simulate_entry_with_attitude(state,p,aero,env,cal,v,site,set,maxbank,initial_bank,initial_bank_rate,NAN,NAN,initial_sign,initial_leg,maxdur,include);
}

EntryPrediction predictor_simulate_entry(VehicleState state,const PlanetModel*p,AerodynamicModel aero,const AerodynamicEnvelope*env,const TrajectoryCalibrationModel*cal,const VehicleProfile*v,const LandingSite*site,const GuidanceSettings*set,double maxbank,double initial_bank,double initial_sign,double initial_leg,double maxdur,bool include){
    return predictor_simulate_entry_with_rate(state,p,aero,env,cal,v,site,set,maxbank,initial_bank,0,initial_sign,initial_leg,maxdur,include);
}

static bool entry_interface_target_compatible(const EntryPrediction*pr,
        const TaemInterfaceTarget*target){
    if(!target||!target->valid)return true;
    return pr&&pr->reached_taem&&!pr->taem_ownership_boundary_missed;
}

typedef struct {
    bool valid;
    bool safe;
    bool terminal_feasible;
    bool ownership_missed;
    double safety_violation;
    double energy_deficit;
    double energy_pressure;
    double alignment_miss;
    double closest_distance;
    double handoff_energy_excess;
    double control_effort;
    unsigned reversals;
} EntryCandidateRank;

static double predictor_normalized_upper_violation(double value,double limit){
    if(!isfinite(value)||!isfinite(limit)||!(limit>0.0))return INFINITY;
    return fmax(0.0,value/limit-1.0);
}

static EntryCandidateRank entry_candidate_rank(const EntryPrediction*pr,
        const VehicleProfile*v,const TaemInterfaceTarget*interface_target,
        double target_bank,double target_aoa,double initial_bank,double initial_aoa,
        double target_specific_energy){
    EntryCandidateRank rank={0};
    rank.safety_violation=INFINITY;
    rank.energy_deficit=INFINITY;
    rank.energy_pressure=INFINITY;
    rank.alignment_miss=INFINITY;
    rank.closest_distance=INFINITY;
    rank.handoff_energy_excess=INFINITY;
    rank.control_effort=INFINITY;
    if(!pr||!v||!pr->entered_atmosphere||!isfinite(pr->closest_distance)||
       !(v->maximum_dynamic_pressure>0.0)||!(v->maximum_g_load>0.0)||
       !(v->minimum_safe_speed>0.0)||!(v->maximum_bank_angle>0.0)||
       !(v->maximum_angle_of_attack>0.0))
        return rank;

    double q_violation=predictor_normalized_upper_violation(
        pr->peak_dynamic_pressure,v->maximum_dynamic_pressure);
    double g_violation=predictor_normalized_upper_violation(
        pr->peak_g_load,v->maximum_g_load);
    double speed_violation=0.0;
    if(isfinite(pr->minimum_entry_speed)&&pr->minimum_entry_speed>0.0)
        speed_violation=fmax(0.0,
            v->minimum_safe_speed/pr->minimum_entry_speed-1.0);
    else
        speed_violation=INFINITY;
    double bank_violation=predictor_normalized_upper_violation(
        fabs(target_bank),v->maximum_bank_angle);
    double aoa_violation=predictor_normalized_upper_violation(
        fmax(fabs(target_aoa),pr->maximum_abs_angle_of_attack),
        v->maximum_angle_of_attack);

    rank.safety_violation=fmax(q_violation,
        fmax(g_violation,fmax(speed_violation,
            fmax(bank_violation,aoa_violation))));
    rank.safe=isfinite(rank.safety_violation)&&rank.safety_violation<=0.0;
    rank.ownership_missed=pr->taem_ownership_boundary_missed;

    bool target_compatible=entry_interface_target_compatible(pr,interface_target);
    rank.terminal_feasible=rank.safe&&target_compatible&&pr->reached_taem&&
        !rank.ownership_missed&&isfinite(pr->taem_energy_error)&&
        pr->taem_energy_error>=0.0;

    if(rank.terminal_feasible)
        rank.handoff_energy_excess=fmax(0.0,pr->taem_energy_error);

    if(pr->control_horizon_recorded&&
       isfinite(pr->control_horizon_specific_energy)&&
       isfinite(pr->control_horizon_range)){
        double energy_error=pr->control_horizon_specific_energy-
            target_specific_energy;
        rank.energy_deficit=fmax(0.0,-energy_error);
        double remaining_path=fmax(pr->control_horizon_range,DBL_MIN);
        rank.energy_pressure=fmax(0.0,energy_error)/remaining_path;
        rank.alignment_miss=isfinite(pr->control_horizon_alignment_distance)?
            fmax(0.0,-pr->control_horizon_alignment_distance):INFINITY;
    }

    rank.closest_distance=pr->closest_distance;
    double bank_use=fabs(target_bank)/v->maximum_bank_angle;
    double aoa_use=fabs(target_aoa)/v->maximum_angle_of_attack;
    double bank_change=fabs(norm_signed_deg(target_bank-initial_bank))/
        v->maximum_bank_angle;
    double aoa_change=fabs(target_aoa-initial_aoa)/
        v->maximum_angle_of_attack;
    rank.control_effort=fmax(fmax(bank_use,aoa_use),
        fmax(bank_change,aoa_change));
    rank.reversals=pr->s_turn_reversals;
    rank.valid=true;
    return rank;
}

static int entry_candidate_rank_compare(const EntryCandidateRank*a,
        const EntryCandidateRank*b){
    if(!a||!a->valid)return (!b||!b->valid)?0:1;
    if(!b||!b->valid)return -1;

    if(a->safe!=b->safe)return a->safe?-1:1;
    if(a->safety_violation<b->safety_violation)return -1;
    if(a->safety_violation>b->safety_violation)return 1;

    if(a->ownership_missed!=b->ownership_missed)
        return a->ownership_missed?1:-1;
    if(a->terminal_feasible!=b->terminal_feasible)
        return a->terminal_feasible?-1:1;

    if(a->terminal_feasible){
        if(a->handoff_energy_excess<b->handoff_energy_excess)return -1;
        if(a->handoff_energy_excess>b->handoff_energy_excess)return 1;
        if(a->control_effort<b->control_effort)return -1;
        if(a->control_effort>b->control_effort)return 1;
        if(a->reversals<b->reversals)return -1;
        if(a->reversals>b->reversals)return 1;
        return 0;
    }

    /*
     * Before the handoff exists, preserve unpowered feasibility first:
     * never prefer an energy-deficit forecast; then avoid crossing the fixed
     * alignment station; then minimize the aerodynamic work still required per
     * metre of useful range.  Geometry and control effort are tie-breakers.
     */
    if(a->energy_deficit<b->energy_deficit)return -1;
    if(a->energy_deficit>b->energy_deficit)return 1;
    if(a->alignment_miss<b->alignment_miss)return -1;
    if(a->alignment_miss>b->alignment_miss)return 1;
    if(a->energy_pressure<b->energy_pressure)return -1;
    if(a->energy_pressure>b->energy_pressure)return 1;
    if(a->closest_distance<b->closest_distance)return -1;
    if(a->closest_distance>b->closest_distance)return 1;
    if(a->control_effort<b->control_effort)return -1;
    if(a->control_effort>b->control_effort)return 1;
    if(a->reversals<b->reversals)return -1;
    if(a->reversals>b->reversals)return 1;
    return 0;
}

static double entry_candidate_trace_metric(const EntryCandidateRank*r){
    if(!r||!r->valid)return INFINITY;
    if(!r->safe)return r->safety_violation;
    if(r->terminal_feasible)return r->control_effort;
    if(r->energy_deficit>0.0)return r->energy_deficit;
    return r->energy_pressure;
}

static PredictorPlannerCostTerms entry_candidate_trace_terms(
        const EntryCandidateRank*r){
    PredictorPlannerCostTerms terms={0};
    if(!r||!r->valid)return terms;
    terms.terminal_range=r->closest_distance;
    terms.terminal_energy=r->energy_pressure;
    terms.pending_terminal=r->terminal_feasible?0.0:1.0;
    terms.interface_match=r->alignment_miss;
    terms.dynamic_pressure=r->safety_violation;
    terms.transition=r->control_effort;
    terms.reversal_count=(double)r->reversals;
    return terms;
}

static double predicted_course_after_control(VehicleState state,const PlanetModel*p,const AerodynamicEnvelope*env0,const TrajectoryCalibrationModel*cal0,const VehicleProfile*v,const GuidanceSettings*s,double initial_bank,double initial_bank_rate,double initial_aoa,double initial_aoa_rate,double target_bank,double target_aoa,double horizon){
    AerodynamicEnvelope uniform;if(!env0){AerodynamicModel a={v->estimated_lift_to_drag,v->estimated_ballistic_coefficient,.2};for(int i=0;i<4;i++)uniform.regimes[i]=a;env0=&uniform;}
    TrajectoryCalibrationModel dc={
        .density_scale=1,.drag_scale=1,.lift_scale=1,.bank_effectiveness=1,.speed_of_sound=340,
        .speed_of_sound_scale=1,
        .altitude_residual=0,.speed_residual=0,.range_residual=0,.confidence=.05,.accepted_samples=0
    };if(!cal0)cal0=&dc;
    PredictorGuidance g;memset(&g,0,sizeof(g));g.actual_bank=norm_signed_deg(initial_bank);g.actual_bank_rate=initial_bank_rate;g.actual_aoa=initial_aoa;g.actual_aoa_rate=initial_aoa_rate;
    int airbrakes=cal0->physics&&cal0->physics->airbrakes>=0?cal0->physics->airbrakes:0;
    AeroContext ctx={.p=p,.e=env0,.c=cal0,.v=v,.bank=g.actual_bank,.aoa=g.actual_aoa,.ut=state.ut,.mass=state.mass,.gear=cal0->physics?cal0->physics->gear:false,.brakes=cal0->physics?cal0->physics->brakes:false,.airbrakes=airbrakes};
    double elapsed=0;
    while(elapsed<horizon){
        AtmosState at=atmosphere(state.position,state.velocity,&ctx);double dt=fmin(.35,horizon-elapsed);
        attitude_step(&g,target_bank,target_aoa,&at,v,s,dt,cal0->physics);
        ctx.bank=g.actual_bank;ctx.aoa=g.actual_aoa;ctx.ut=state.ut;state=rk4_aero(state,dt,&ctx);elapsed+=dt;
    }
    GeoPoint geo=predictor_geo_point(state.position,p,state.ut),target={0,0,0};
    return surface_course(state.position,state.velocity,planet_rotation_vector(p),p->north_axis,initial_bearing(geo,target));
}

static EntryControlPlan predictor_plan_entry_control_legacy(VehicleState state,const PlanetModel*p,AerodynamicModel aero,const AerodynamicEnvelope*env,const TrajectoryCalibrationModel*cal,const VehicleProfile*v,const LandingSite*site,const GuidanceSettings*s,const TaemInterfaceTarget*interface_target,bool enforce_terminal_delivery_budget,double initial_bank,double initial_bank_rate,double initial_aoa,double initial_aoa_rate,double current_sign,bool current_leg_established,double current_leg_elapsed,double current_commit_remaining,bool s_turn_side_locked,bool final_heading_lock,double maxdur){
    EntryControlPlan out,best;
    memset(&out,0,sizeof(out));memset(&best,0,sizeof(best));
    out.cost=best.cost=INFINITY;out.planned_ut=best.planned_ut=state.ut;
    EntryCandidateRank best_rank={0};
    AerodynamicEnvelope uniform;if(!env){for(int i=0;i<4;i++)uniform.regimes[i]=aero;env=&uniform;}
    TrajectoryCalibrationModel fallback={.density_scale=1,.drag_scale=1,.lift_scale=1,.bank_effectiveness=1,.speed_of_sound=340,.speed_of_sound_scale=1,.confidence=.05};
    if(!cal)cal=&fallback;
    if(!(v->maximum_bank_angle>0.0)||!(v->maximum_angle_of_attack>0.0))
        return out;
    double maxbank=v->maximum_bank_angle;
    int current_airbrakes=cal->physics&&cal->physics->airbrakes>=0?cal->physics->airbrakes:0;
    AeroContext current_ctx={.p=p,.e=env,.c=cal,.v=v,.bank=initial_bank,.aoa=initial_aoa,.ut=state.ut,.mass=state.mass,.gear=cal->physics?cal->physics->gear:false,.brakes=cal->physics?cal->physics->brakes:false,.airbrakes=current_airbrakes};
    AtmosState current_at=atmosphere(state.position,state.velocity,&current_ctx);
    double current_limit=bank_limit(&current_at,p,v,maxbank);
    if(!(current_limit>0.0)||!isfinite(current_limit))return out;

    const int bank_samples=6; /* numerical resolution only */
    double banks[bank_samples];
    double effectiveness=isfinite(cal->bank_effectiveness)&&
        cal->bank_effectiveness>0.0?cal->bank_effectiveness:1.0;
    double lift=fmax(0.0,current_at.lift_accel);
    double max_effective_bank=fmin(current_limit*effectiveness,
        nextafter(90.0,0.0));
    double max_lateral=lift*fabs(sin(max_effective_bank*DEG2RAD));
    for(int i=0;i<bank_samples;i++){
        double fraction=(double)i/(double)(bank_samples-1);
        double requested=max_lateral*fraction;
        double sine=lift>DBL_MIN?requested/lift:0.0;
        banks[i]=asin(clampd(sine,0.0,1.0))/effectiveness*RAD2DEG;
    }

    const int aoa_samples=5; /* numerical resolution only */
    double aoas[aoa_samples];
    for(int i=0;i<aoa_samples;i++)
        aoas[i]=v->maximum_angle_of_attack*
            (double)i/(double)(aoa_samples-1);
    double target_speed=entry_taem_speed_target(v,s,p);
    double target_specific_energy=rotating_specific_energy(
        site->latitude,s->taem_interface_altitude,target_speed,p);
    double effective_min_leg=entry_s_turn_effective_minimum_leg(
        current_at.speed,target_speed,s);
    bool have_side=fabs(current_sign)>DBL_EPSILON;
    bool lock_side=final_heading_lock||s_turn_side_locked||
        (have_side&&(current_commit_remaining>0.0||
         (current_leg_established&&current_leg_elapsed<effective_min_leg)));

    for(int bi=0;bi<bank_samples;bi++)for(int ai=0;ai<aoa_samples;ai++){
        bool wings_level=fabs(banks[bi])<=DBL_EPSILON;
        int side_count=wings_level?1:(lock_side?1:2);
        for(int si=0;si<side_count;si++){
            double sign=wings_level?
                (have_side?(current_sign>=0.0?1.0:-1.0):1.0):
                (lock_side?(current_sign>=0.0?1.0:-1.0):
                 (si==0?1.0:-1.0));
            double bank=sign*banks[bi];
            double leg=have_side&&sign*current_sign>0.0?
                current_leg_elapsed:0.0;
            double radius=wings_level?INFINITY:
                turn_radius_for_bank(&current_at,cal,fabs(bank));

            /*
             * Forecast one physically executable command segment: long enough
             * to satisfy the S-turn dwell and to achieve the commanded roll
             * under the configured rate/acceleration limits.
             */
            double bank_delta=fabs(norm_signed_deg(bank-initial_bank));
            double rate_time=s->entry_roll_rate>DBL_MIN?
                bank_delta/s->entry_roll_rate:0.0;
            double accel_time=s->entry_roll_acceleration>DBL_MIN?
                sqrt(2.0*bank_delta/s->entry_roll_acceleration):0.0;
            double segment=fmax(effective_min_leg,
                fmax(rate_time,accel_time));
            if(current_commit_remaining>0.0)
                segment=fmax(segment,current_commit_remaining);
            if(maxdur>0.0)segment=fmin(segment,maxdur);

            EntryPrediction pr=predictor_simulate_entry_core(
                state,p,aero,env,cal,v,site,s,maxbank,
                initial_bank,initial_bank_rate,initial_aoa,initial_aoa_rate,
                sign,leg,final_heading_lock,maxdur,true,true,
                bank,aoas[ai],segment,segment,true,
                enforce_terminal_delivery_budget,NULL,interface_target);

            EntryCandidateRank rank=entry_candidate_rank(&pr,v,
                interface_target,bank,aoas[ai],initial_bank,initial_aoa,
                target_specific_energy);
            double trace_metric=entry_candidate_trace_metric(&rank);
            PredictorPlannerCostTerms cost_terms=
                entry_candidate_trace_terms(&rank);

            EntryControlPlan trace_plan={0};
            trace_plan.valid=rank.valid;
            trace_plan.planned_ut=state.ut;
            trace_plan.terminal_ready=rank.terminal_feasible;
            trace_plan.target_bank=bank;
            trace_plan.target_aoa=aoas[ai];
            trace_plan.bank_cap=current_limit;
            trace_plan.target_turn_radius=radius;
            trace_plan.segment_duration=segment;
            trace_plan.cost=trace_metric;
            trace_plan.taem_range_error=pr.reached_taem?
                pr.taem_range_error:pr.closest_distance;
            trace_plan.taem_speed=pr.reached_taem?pr.taem_speed:NAN;
            trace_plan.taem_energy_error=pr.taem_energy_error;
            trace_plan.closest_distance=pr.closest_distance;
            trace_plan.predicted_reversals=pr.s_turn_reversals;
            trace_plan.has_planned_reversal=pr.has_first_s_turn_reversal;
            trace_plan.planned_reversal_is_final=
                pr.first_s_turn_reversal_is_final;
            trace_plan.final_heading_lock=final_heading_lock;
            trace_plan.planned_reversal_ut=pr.first_s_turn_reversal_ut;
            trace_plan.planned_reversal_range=pr.first_s_turn_reversal_range;
            trace_plan.planned_reversal_sign=pr.has_first_s_turn_reversal?
                (fabs(pr.first_s_turn_reversal_sign)>DBL_EPSILON?
                 pr.first_s_turn_reversal_sign:-sign):0.0;

            EntryPredictionAssessment trace_assessment=
                predictor_assess_entry_prediction(
                    &pr,v,current_limit,&trace_plan);
            planner_trace_add(PREDICTOR_PLAN_CANDIDATE_LEGACY,
                &trace_plan,&trace_assessment,&pr,trace_metric,&cost_terms);

            if(rank.valid&&(!best.valid||
               entry_candidate_rank_compare(&rank,&best_rank)<0)){
                best=trace_plan;
                best_rank=rank;
            }
            entry_prediction_clear(&pr);
        }
    }

    out=best;
    if(out.valid){
        GeoPoint geo=predictor_geo_point(state.position,p,state.ut),target=final_heading_lock?runway_approach_aimpoint(site,p->radius,500.0):(GeoPoint){site->latitude,site->longitude,site->altitude};
        double fallback=initial_bearing(geo,target);
        if(interface_target&&interface_target->valid){
            PredictorGuidance reference={0};reference.has_interface_target=true;reference.interface_target=*interface_target;
            fallback=predictor_interface_heading(&reference,geo,p,site,s,fallback);
        }
        double predicted=predicted_course_after_control(state,p,env,cal,v,s,
            initial_bank,initial_bank_rate,initial_aoa,initial_aoa_rate,
            out.target_bank,out.target_aoa,out.segment_duration);
        out.target_heading=final_heading_lock?fallback:(interface_target&&interface_target->valid?fallback:(isfinite(predicted)?predicted:fallback));
    }
    return out;
}

EntryPrediction predictor_simulate_entry_control_plan_to_interface(VehicleState state,const PlanetModel*p,AerodynamicModel aero,const AerodynamicEnvelope*env,const TrajectoryCalibrationModel*cal,const VehicleProfile*v,const LandingSite*site,const GuidanceSettings*s,const TaemInterfaceTarget*interface_target,bool enforce_terminal_delivery_budget,double initial_bank,double initial_bank_rate,double initial_aoa,double initial_aoa_rate,double current_sign,double current_leg_elapsed,const EntryControlPlan*plan,double maxdur,bool include){
    if(!plan||!plan->valid)
        return predictor_simulate_entry_with_attitude(state,p,aero,env,cal,v,site,s,v->maximum_bank_angle,initial_bank,initial_bank_rate,initial_aoa,initial_aoa_rate,current_sign,current_leg_elapsed,maxdur,include);
    double horizon=plan->segment_duration>0.0?
        plan->segment_duration:s->s_turn_minimum_leg_duration;
    if(maxdur>0.0)horizon=fmin(horizon,maxdur);
    double sign=fabs(plan->target_bank)>=1?(plan->target_bank>=0?1:-1):current_sign;
    /* The live controller commits only the current control segment.  Holding
       that bank/AoA for the entire 2500 s display rollout made every small
       replan move the whole predicted vertical profile, producing the visible
       upward/downward jumps in the HUD.  Apply the retained command only for
       its committed horizon, then let the predictor resume its own S-turn
       policy (including any latched reversal event) for the remaining path. */
    return predictor_simulate_entry_core(state,p,aero,env,cal,v,site,s,v->maximum_bank_angle,initial_bank,initial_bank_rate,initial_aoa,initial_aoa_rate,sign,current_leg_elapsed,plan->final_heading_lock,maxdur,include,true,plan->target_bank,plan->target_aoa,horizon,horizon,true,enforce_terminal_delivery_budget,plan,interface_target);
}

EntryPrediction predictor_simulate_entry_control_plan(VehicleState state,const PlanetModel*p,AerodynamicModel aero,const AerodynamicEnvelope*env,const TrajectoryCalibrationModel*cal,const VehicleProfile*v,const LandingSite*site,const GuidanceSettings*s,double initial_bank,double initial_bank_rate,double initial_aoa,double initial_aoa_rate,double current_sign,double current_leg_elapsed,const EntryControlPlan*plan,double maxdur,bool include){
    return predictor_simulate_entry_control_plan_to_interface(state,p,aero,env,cal,v,site,s,NULL,false,
        initial_bank,initial_bank_rate,initial_aoa,initial_aoa_rate,current_sign,current_leg_elapsed,plan,maxdur,include);
}

typedef struct {
    bool terminal_prediction_valid;
    bool terminal_policy_feasible;
    bool terminal_path_committed;
    bool final_approach_captured;
    bool aborted;
} TerminalShadowOutcome;

static Telemetry guidance_shadow_telemetry(const Telemetry*base,VehicleState state,
        PredictorGuidance*response,AeroContext*ctx,const PlanetModel*p,
        const LandingConfiguration*cfg,AtmosState*out_at){
    Telemetry t;memset(&t,0,sizeof(t));if(base)t=*base;
    ctx->ut=state.ut;ctx->mass=state.mass;ctx->bank=response->actual_bank;ctx->aoa=response->actual_aoa;
    AtmosState at=atmosphere(state.position,state.velocity,ctx);if(out_at)*out_at=at;
    GeoPoint geo=predictor_geo_point(state.position,p,state.ut),target={cfg->site.latitude,cfg->site.longitude,cfg->site.altitude};
    Vector3 up=vnorm(state.position,v3(0,1,0));double vs=vdot(at.air_velocity,up);
    double hs=vmag(vproject_plane(at.air_velocity,up));double fpa=atan2(vs,fmax(hs,.1))*RAD2DEG;
    double fallback=initial_bearing(geo,target);
    double course=surface_course(state.position,state.velocity,planet_rotation_vector(p),p->north_axis,fallback);
    double along=0,cross=0;runway_coordinates(geo,target,cfg->site.runway_heading,p->radius,&along,&cross);
    double turn_rate=at.speed>1?at.lift_accel*sin(response->actual_bank*DEG2RAD)/at.speed*RAD2DEG:0;
    t.ut=state.ut;t.latitude=geo.latitude;t.longitude=geo.longitude;t.mean_altitude=geo.altitude;
    t.radar_altitude=fmax(0,geo.altitude-cfg->site.altitude);t.vertical_speed=vs;t.horizontal_speed=hs;
    t.surface_speed=hs;t.true_air_speed=at.speed;t.atmospheric_density=at.density;t.speed_of_sound=at.speed_of_sound;t.mach=at.mach;
    t.heading=course;t.ground_track_heading=course;t.course_to_site_error=norm_signed_deg(fallback-course);
    t.pitch=fpa+response->actual_aoa;t.roll=response->actual_bank;
    t.pitch_rate=response->actual_aoa_rate;t.roll_rate=response->actual_bank_rate;t.heading_rate=turn_rate;
    t.has_angle_of_attack_rate=true;t.angle_of_attack_rate=response->actual_aoa_rate;
    /* The reduced shadow integrates bank/AoA coordinate rates, not body-axis p/q.
       Do not masquerade these state derivatives as rigid-body angular rates. */
    t.has_body_pitch_rate=false;t.has_body_roll_rate=false;t.has_body_yaw_rate=false;t.has_course_rate=true;
    t.body_pitch_rate=0;t.body_roll_rate=0;t.body_yaw_rate=0;t.course_rate=turn_rate;t.autopilot_error=0;
    t.has_command_pitch_error=false;t.has_command_roll_error=false;t.has_command_heading_error=false;
    t.angle_of_attack=response->actual_aoa;t.sideslip=0;t.dynamic_pressure=at.q;
    t.static_pressure=planet_atmospheric_pressure(p,geo.altitude);
    t.g_force=at.non_gravity/fmax(planet_surface_gravity(p),.1);
    t.stall_fraction=vessel_physics_conservative_stall_fraction(t.true_air_speed,t.angle_of_attack,t.dynamic_pressure,&cfg->vehicle);
    t.stall_fraction_is_measured=false;
    /* A reduced attitude model cannot declare recovery from a measured stall
       while it still carries the observed incidence. Retain that safety evidence. */
    if(base&&base->stall_fraction_is_measured&&response->actual_aoa>=base->angle_of_attack-1.0){
        t.stall_fraction=fmax(t.stall_fraction,base->stall_fraction);
        t.stall_fraction_is_measured=true;
    }
    t.lift_force=fabs(at.lift_accel)*fmax(state.mass,1);t.drag_force=fabs(at.drag_accel)*fmax(state.mass,1);
    t.has_force_vectors=false;t.physics_sample_valid=false;t.mass=state.mass;t.available_thrust=0;t.current_thrust=0;t.throttle=0;
    t.has_controls=false;t.control_pitch=0;t.control_roll=0;t.control_yaw=0;
    t.gear=ctx->gear;t.brakes=ctx->brakes;t.has_airbrakes=true;t.airbrakes=ctx->airbrakes!=0;
    snprintf(t.vessel_situation,sizeof(t.vessel_situation),"flying");
    t.range_to_site=great_circle_distance(geo,target,p->radius);t.bearing_to_site=fallback;t.heading_error=norm_signed_deg(fallback-course);
    t.runway_along_track=along;t.runway_cross_track=cross;t.flight_path_angle=fpa;
    t.aerodynamic_confidence=clampd(fmax(at.physics_confidence,base?base->aerodynamic_confidence:0),0,1);
    t.physics_confidence=at.physics_confidence;t.physics_certified_uncertainty=at.physics_uncertainty;
    if(ctx->c){
        t.trajectory_density_scale=ctx->c->density_scale;t.trajectory_drag_scale=ctx->c->drag_scale;
        t.trajectory_lift_scale=ctx->c->lift_scale;t.bank_effectiveness=ctx->c->bank_effectiveness;
        t.trajectory_calibration_confidence=ctx->c->confidence;t.trajectory_altitude_residual=ctx->c->altitude_residual;
        t.trajectory_speed_residual=ctx->c->speed_residual;t.trajectory_range_residual=ctx->c->range_residual;
    }
    double entry_ref=base&&base->true_air_speed>0?base->true_air_speed:at.speed;
    EntryTerminalDemand demand=entry_terminal_demand(t.latitude,t.mean_altitude,t.range_to_site,t.horizontal_speed,
        t.course_to_site_error,t.vertical_speed,t.true_air_speed,entry_ref,p,&cfg->vehicle,&cfg->site,&cfg->guidance);
    t.energy_excess_range=-demand.projected_taem_range_error;
    return t;
}

static bool terminal_shadow_survives(const EntryPrediction*pr,const TerminalShadowOutcome*outcome,
        const VehicleProfile*v){
    if(!pr||!outcome||!v||outcome->aborted)return false;
    if(pr->peak_dynamic_pressure>v->maximum_dynamic_pressure||
       pr->peak_g_load>v->maximum_g_load)return false;
    if(isfinite(pr->minimum_entry_speed)&&
       pr->minimum_entry_speed<v->minimum_safe_speed)return false;
    return outcome->final_approach_captured||outcome->terminal_path_committed||outcome->terminal_policy_feasible;
}

static EntryPrediction predictor_simulate_guidance_shadow_core(VehicleState state,const Telemetry*telemetry,
        const GuidanceMachine*guidance,const DeorbitPlan*plan,const PlanetModel*p,AerodynamicModel aero,
        const AerodynamicEnvelope*env0,const TrajectoryCalibrationModel*cal0,const LandingConfiguration*cfg,
        double maxdur,bool include,bool require_terminal_start,bool stop_at_taem,TerminalShadowOutcome*outcome){
    EntryPrediction out;memset(&out,0,sizeof(out));trajectory_init(&out.trajectory);
    out.closest_distance=DBL_MAX;out.taem_distance=DBL_MAX;out.taem_hac_capture_score=DBL_MAX;
    out.best_terminal_capture_cost=DBL_MAX;out.taem_altitude=NAN;out.taem_desired_altitude=NAN;
    out.minimum_entry_speed=DBL_MAX;out.minimum_bank_control_margin=INFINITY;out.minimum_aoa_control_margin=INFINITY;
    if(outcome)memset(outcome,0,sizeof(*outcome));
    if(!telemetry||!guidance||!p||!cfg||!guidance->automation_engaged)return out;
    if(require_terminal_start&&!taem_exec_owns_vehicle(&guidance->taem_exec))return out;

    AerodynamicEnvelope uniform;if(!env0){for(int i=0;i<4;i++)uniform.regimes[i]=aero;env0=&uniform;}
    TrajectoryCalibrationModel dc={.density_scale=1,.drag_scale=1,.lift_scale=1,.bank_effectiveness=1,
        .speed_of_sound=340,.speed_of_sound_scale=1,.stress_drag_scale=1,.stress_lift_scale=1,.confidence=.05};
    if(!cal0)cal0=&dc;
    GuidanceMachine shadow=*guidance;
    PredictorPlannerTrace saved_planner_trace=g_planner_trace;
    shadow.diagnostic_shadow=true;
    shadow.terminal_planning_deferred=false; /* Replay the identical terminal search without a wall-clock worker. */
    /* The full-flight shadow runs on the prediction worker, not the control
       thread. Let it execute the same rolling MM304 replans after each segment
       expires; freezing the worker's deferred mode made the forecast hold one
       stale high-altitude command all the way to the 15 km recovery floor. */
    shadow.entry_planning_deferred=false;
    shadow.has_previous_ut=true;shadow.previous_ut=state.ut-.1;
    guidance_set_entry_predictor_models(&shadow,env0,cal0);
    PredictorGuidance response;memset(&response,0,sizeof(response));
    response.actual_bank=norm_signed_deg(telemetry->roll);
    response.actual_bank_rate=isfinite(telemetry->roll_rate)?telemetry->roll_rate:0.0;
    response.actual_aoa=isfinite(telemetry->angle_of_attack)?telemetry->angle_of_attack:cfg->vehicle.entry_angle_of_attack;
    response.actual_aoa_rate=telemetry->has_angle_of_attack_rate&&isfinite(telemetry->angle_of_attack_rate)?
        telemetry->angle_of_attack_rate:0.0;
    response.limiter.has_value=true;response.limiter.value=norm_deg(response.actual_bank);
    bool initial_gear=telemetry->gear,initial_brakes=telemetry->brakes;
    int initial_airbrakes=telemetry->has_airbrakes&&telemetry->airbrakes?1:0;
    AeroContext ctx={.p=p,.e=env0,.c=cal0,.v=&cfg->vehicle,.bank=response.actual_bank,.aoa=response.actual_aoa,
        .ut=state.ut,.mass=state.mass,.gear=initial_gear,.brakes=initial_brakes,.airbrakes=initial_airbrakes};
    GeoPoint target={cfg->site.latitude,cfg->site.longitude,cfg->site.altitude};
    double elapsed=0,lastsample=-DBL_MAX;
    double entry_alt=p->atmosphere_depth-cfg->guidance.entry_interface_altitude_margin;
    out.entered_atmosphere=telemetry->mean_altitude<=entry_alt&&telemetry->vertical_speed<0.0;
    bool initial_terminal_owned=taem_exec_owns_vehicle(&shadow.taem_exec);
    bool initial_degraded_safety_handoff=initial_terminal_owned&&shadow.taem_safety_handoff&&
        !shadow.taem_interface_captured;
    /* A replay can begin after live guidance already latched degraded MM305 safety
       ownership. For stop-at-TAEM qualification that state is evidence that the
       fixed MM304 ownership boundary was missed, not a successful TAEM capture.
       The in-loop transition path already applies this rule; apply it at t=0 too. */
    out.reached_taem=initial_terminal_owned&&
        !(stop_at_taem&&initial_degraded_safety_handoff);
    out.taem_ownership_boundary_missed=stop_at_taem&&initial_degraded_safety_handoff;
    out.taem_dynamic_interface_captured=shadow.taem_interface_captured;
    out.taem_dynamic_interface_target_valid=shadow.taem_interface_target.valid;
    out.taem_terminal_candidate_valid=shadow.terminal_candidate.valid;
    out.taem_terminal_candidate_geometry_clean=shadow.terminal_candidate.valid&&
        !shadow.terminal_candidate.geometry_degraded;
    if(out.entered_atmosphere){
        GeoPoint initial_geo=predictor_geo_point(state.position,p,state.ut);
        GeoPoint initial_target={cfg->site.latitude,cfg->site.longitude,cfg->site.altitude};
        out.entry_range=great_circle_distance(initial_geo,initial_target,p->radius);
        out.entry_flight_path_angle=telemetry->flight_path_angle;
        out.entry_speed=telemetry->true_air_speed;
        out.entry_course_error=telemetry->course_to_site_error;
    }
    out.shadow_guidance_used=true;
    while(elapsed<maxdur){
        AtmosState at;Telemetry synthetic=guidance_shadow_telemetry(telemetry,state,&response,&ctx,p,cfg,&at);
        GeoPoint geo={synthetic.latitude,synthetic.longitude,synthetic.mean_altitude};
        double range=synthetic.range_to_site;
        if(!out.entered_atmosphere&&geo.altitude<=entry_alt&&synthetic.vertical_speed<0.0){
            out.entered_atmosphere=true;out.entry_range=range;out.entry_flight_path_angle=synthetic.flight_path_angle;
            out.entry_speed=at.speed;out.entry_course_error=synthetic.course_to_site_error;
        }
        /* 50 km is the explicit mission validation checkpoint.  Record the same
           runway-relative state from the executable-policy shadow so reduced
           prediction, shadow execution, and live KSP can be compared on one
           physical crossing instead of inferred from different terminal events. */
        if(out.entered_atmosphere&&!out.mm304_gate_recorded&&
           geo.altitude<=50000.0&&synthetic.vertical_speed<0.0){
            out.mm304_gate_recorded=true;out.mm304_gate_range=range;
            out.mm304_gate_along_track=synthetic.runway_along_track;
            out.mm304_gate_cross_track=synthetic.runway_cross_track;
            out.mm304_gate_course=synthetic.ground_track_heading;
            out.mm304_gate_altitude=geo.altitude;out.mm304_gate_speed=at.speed;
            out.mm304_gate_flight_path_angle=synthetic.flight_path_angle;
        }
        if(out.entered_atmosphere)out.closest_distance=fmin(out.closest_distance,range);
        if(out.entered_atmosphere&&!out.reached_taem)out.minimum_entry_speed=fmin(out.minimum_entry_speed,at.speed);
        out.maximum_abs_angle_of_attack=fmax(out.maximum_abs_angle_of_attack,fabs(response.actual_aoa));
        out.peak_dynamic_pressure=fmax(out.peak_dynamic_pressure,at.q);out.peak_g_load=fmax(out.peak_g_load,synthetic.g_force);
        if(at.direct_aero)out.physics_observed_seconds+=fmin(1.0,maxdur-elapsed);else out.physics_fallback_seconds+=fmin(1.0,maxdur-elapsed);
        if(!out.reached_taem){
            out.taem_distance=range;out.taem_range_error=range-entry_taem_range_target(p,&cfg->guidance);
            out.taem_speed=at.speed;out.taem_flight_path_angle=synthetic.flight_path_angle;out.taem_altitude=geo.altitude;
            out.taem_energy_error=entry_remaining_specific_energy(geo.latitude,geo.altitude,at.speed,cfg->site.latitude,
                cfg->guidance.taem_interface_altitude,entry_taem_speed_target(&cfg->vehicle,&cfg->guidance,p),p);
        }else if(elapsed<=1e-9){
            out.taem_distance=range;out.taem_range_error=range-entry_taem_range_target(p,&cfg->guidance);
            out.taem_speed=at.speed;out.taem_flight_path_angle=synthetic.flight_path_angle;out.taem_altitude=geo.altitude;
            out.taem_energy_error=entry_remaining_specific_energy(geo.latitude,geo.altitude,at.speed,cfg->site.latitude,
                cfg->guidance.taem_interface_altitude,entry_taem_speed_target(&cfg->vehicle,&cfg->guidance,p),p);
        }
        unsigned previous_reversals=shadow.entry_control_reversals;
        bool previous_final=shadow.entry_final_reversal_pending||shadow.entry_final_reversal_completed;
        GuidanceResult r=guidance_update(&shadow,&synthetic,&state,plan,p,aero,cfg);
        bool current_final=shadow.entry_final_reversal_pending||shadow.entry_final_reversal_completed;
        if(!out.has_first_s_turn_reversal&&
           (shadow.entry_control_reversals!=previous_reversals||(!previous_final&&current_final))){
            out.has_first_s_turn_reversal=true;out.first_s_turn_reversal_ut=state.ut;
            out.first_s_turn_reversal_range=range;out.first_s_turn_reversal_sign=shadow.s_turn_sign;
            out.first_s_turn_reversal_is_final=current_final;
            if(shadow.diagnostic_shadow)
                fprintf(stderr,"DEBUG_ENTRY_REVERSAL ut=%.1f V=%.1f h=%.0f live=(%.0f,%.0f,%.1f) side=%.0f final=%d target=(%.0f,%.0f,%.1f)\n",
                    state.ut,at.speed,geo.altitude,synthetic.runway_along_track,synthetic.runway_cross_track,
                    synthetic.ground_track_heading,shadow.s_turn_sign,current_final,
                    shadow.taem_interface_target.along_track,shadow.taem_interface_target.cross_track,
                    shadow.taem_interface_target.course);
        }
        out.s_turn_reversals=shadow.entry_control_reversals-guidance->entry_control_reversals;
        bool terminal_owned_now=taem_exec_owns_vehicle(&shadow.taem_exec);
        bool noninterface_terminal_ownership=terminal_owned_now&&!shadow.taem_interface_captured;
        /* TAEM recovery ownership is useful live, but a stop-at-TAEM shadow is
           qualifying the MM304 fixed-interface contract. Any ownership transfer
           that did not capture that interface is a miss, not "TAEM reached". */
        if(stop_at_taem&&noninterface_terminal_ownership)
            out.taem_ownership_boundary_missed=true;
        if(!out.reached_taem&&terminal_owned_now&&!(stop_at_taem&&noninterface_terminal_ownership)){
            out.reached_taem=true;out.taem_distance=range;
            out.taem_dynamic_interface_captured=shadow.taem_interface_captured;
            out.taem_dynamic_interface_target_valid=shadow.taem_interface_target.valid;
            out.taem_terminal_candidate_valid=shadow.terminal_candidate.valid;
            out.taem_terminal_candidate_geometry_clean=shadow.terminal_candidate.valid&&
                !shadow.terminal_candidate.geometry_degraded;
            out.taem_range_error=range-entry_taem_range_target(p,&cfg->guidance);
            out.taem_speed=at.speed;out.taem_flight_path_angle=synthetic.flight_path_angle;
            out.taem_altitude=geo.altitude;out.taem_desired_altitude=cfg->guidance.taem_interface_altitude;
            out.taem_along_track=synthetic.runway_along_track;out.taem_cross_track=synthetic.runway_cross_track;
            out.taem_course=synthetic.ground_track_heading;
            out.taem_energy_error=entry_remaining_specific_energy(geo.latitude,geo.altitude,at.speed,cfg->site.latitude,
                cfg->guidance.taem_interface_altitude,entry_taem_speed_target(&cfg->vehicle,&cfg->guidance,p),p);
        }
        if(stop_at_taem&&out.entered_atmosphere&&!out.reached_taem&&
           at.speed<=cfg->guidance.taem_force_handoff_speed){
            /* The ownership boundary is the dynamic MM304 tangent state, not the
               historical 20 km runway-along shell.  Crossing the speed ceiling is
               only eligibility; keep flying the S-turn until the tangent tube is
               captured or its state has been physically passed/lost. */
            if(shadow.taem_interface_target.valid){
                TaemInterfaceCapture capture=entry_taem_interface_capture(
                    &shadow.taem_interface_target,&synthetic,synthetic.ground_track_heading,
                    p,cfg);
                double low_speed=fmax(cfg->vehicle.minimum_safe_speed*1.6,
                    shadow.taem_interface_target.speed*.72);
                double pass_lead=fmax(0.0,shadow.taem_interface_target.acquisition_lead);
                double pass_downstream=fmax(1500.0,fmin(4000.0,pass_lead*.35));
                double pass_cross=fmax(3000.0,fmin(9000.0,pass_lead*.50));
                bool passed=capture.valid&&capture.along<-pass_downstream&&
                    fabs(capture.cross)<=pass_cross;
                bool energy_lost=capture.valid&&at.speed<low_speed&&capture.energy_margin<0.0;
                TaemSpeedEnvelope speed_envelope=
                    decision_taem_speed_envelope(&cfg->vehicle,p,geo.altitude);
                bool near_alignment=capture.valid&&speed_envelope.valid&&
                    hypot(capture.along,capture.cross)<=12000.0&&
                    geo.altitude>=cfg->site.altitude+15000.0&&
                    at.speed>=speed_envelope.minimum_speed_mps&&
                    at.speed<=speed_envelope.maximum_speed_mps;
                /* Near the fixed station, a slow state is still recoverable if
                   it can continue turning toward the explicit heading/position
                   contract. Do not terminate the shadow before that contract is
                   either met or the vehicle leaves the 15 km recovery shell. */
                out.taem_ownership_boundary_missed=passed||(energy_lost&&!near_alignment);
                if(out.taem_ownership_boundary_missed)
                    fprintf(stderr,"DEBUG_TAEM_MISS ut=%.1f V=%.1f h=%.0f fpa=%.1f live=(%.0f,%.0f,%.1f) target=(%.0f,%.0f,%.1f,%.0f,%.1f,%.1f) cap=(along %.0f cross %.0f course %.1f h %.0f E %.0f turn %.0f veto %u) prog=(side %.0f bank %.1f roll %.1f sched %d final %d rsign %.0f rut %.1f pending %d done %d infeasible %d) passed=%d energy=%d\n",
                        state.ut,at.speed,geo.altitude,synthetic.flight_path_angle,
                        synthetic.runway_along_track,synthetic.runway_cross_track,synthetic.ground_track_heading,
                        shadow.taem_interface_target.along_track,shadow.taem_interface_target.cross_track,
                        shadow.taem_interface_target.course,shadow.taem_interface_target.altitude,
                        shadow.taem_interface_target.speed,shadow.taem_interface_target.flight_path_angle,
                        capture.along,capture.cross,capture.course_error,capture.altitude_error,
                        capture.energy_margin,capture.turn_margin,capture.veto,
                        shadow.s_turn_sign,shadow.entry_control_bank,synthetic.roll,
                        shadow.entry_reversal_scheduled,shadow.entry_reversal_is_final,
                        shadow.entry_reversal_sign,shadow.entry_reversal_ut,
                        shadow.entry_final_reversal_pending,shadow.entry_final_reversal_completed,
                        shadow.entry_lateral_infeasible,passed,energy_lost);
            }else{
                /* No dynamic target is fail-closed.  Retain the old shell only as a
                   last-resort miss detector for malformed/legacy shadow state. */
                bool geometry_ready=entry_taem_handoff_geometry_ready(geo.altitude,
                    synthetic.runway_along_track,synthetic.vertical_speed,
                    synthetic.horizontal_speed,&cfg->guidance);
                if(!geometry_ready){
                    double handoff_low=0.0,handoff_high=0.0;
                    entry_taem_handoff_altitude_bounds(&cfg->guidance,&handoff_low,&handoff_high);
                    double station_lead=clampd(synthetic.horizontal_speed*2.0,180.0,2400.0);
                    out.taem_ownership_boundary_missed=
                        synthetic.runway_along_track>=-cfg->guidance.final_approach_distance-station_lead||
                        geo.altitude<handoff_low;
                    (void)handoff_high;
                }
            }
        }
        bool policy_feasible=shadow.taem_exec.terminal_evaluation.valid&&shadow.taem_exec.terminal_evaluation.feasible;
        out.shadow_terminal_prediction_valid=shadow.terminal_prediction_valid;
        out.shadow_terminal_policy_feasible=policy_feasible;
        out.shadow_terminal_path_committed=shadow.terminal_path_committed;
        out.shadow_final_approach_captured=shadow.final_approach_captured;
        out.shadow_aborted=shadow.aborted||r.phase==PHASE_ABORT;
        bool terminal_public_phase=out.reached_taem&&r.phase==PHASE_TAEM;
        if(include&&(state.ut-lastsample>=2.0||terminal_public_phase)){
            /* A degraded safety takeover means MM304 missed the fixed TAEM
               interface. For stop-at-TAEM qualification/presentation, do not
               serialize recovery ownership as a successful TAEM trajectory point. */
            GuidancePhase published_phase=(stop_at_taem&&noninterface_terminal_ownership)?
                PHASE_ENTRY_ENERGY:r.phase;
            TrajectoryPoint tp={state.ut,geo.latitude,geo.longitude,geo.altitude,at.speed,published_phase,TRAJ_PLANNED};
            trajectory_append(&out.trajectory,tp);lastsample=state.ut;
        }
        /* The ownership latch can become true inside the Entry guidance call that
           still returns its Entry-phase result for that tick. Continue one reduced
           integration step so qualification observes the same public PHASE_TAEM
           frame the live controller would publish next, then stop immediately. */
        if(out.shadow_aborted||
           (stop_at_taem&&(out.taem_ownership_boundary_missed||terminal_public_phase))){
            guidance_result_clear(&r);break;
        }
        if(geo.altitude<=cfg->site.altitude-20||at.speed<fmax(25.0,cfg->vehicle.touchdown_speed*.55)){
            guidance_result_clear(&r);break;
        }
        double target_bank=isfinite(r.command.target_roll)?norm_signed_deg(r.command.target_roll):response.actual_bank;
        double target_aoa=response.actual_aoa;
        if(r.command.use_inertial_direction&&vmag(r.command.inertial_direction)>1e-9){
            /* Live q<0.5 Entry Interface capture ignores the dormant Euler/AoA
               fields and uses a quaternion/RCS law to align body-forward with the
               commanded inertial prograde vector while keeping radial-up as the
               roll reference.  The reduced shadow has only bank/AoA states, so
               project that same attitude objective into aerodynamic coordinates:
               wings level, with incidence equal to the vertical-plane angle
               between inertial forward and the atmosphere-relative velocity. */
            Vector3 local_up=vnorm(state.position,v3(1,0,0));
            Vector3 desired_forward=vnorm(r.command.inertial_direction,state.velocity);
            Vector3 air_velocity=vsub(state.velocity,vcross(planet_rotation_vector(p),state.position));
            Vector3 air_forward=vnorm(air_velocity,desired_forward);
            double desired_fpa=atan2(vdot(desired_forward,local_up),
                fmax(vmag(vproject_plane(desired_forward,local_up)),1e-9))*RAD2DEG;
            double air_fpa=atan2(vdot(air_forward,local_up),
                fmax(vmag(vproject_plane(air_forward,local_up)),1e-9))*RAD2DEG;
            target_bank=0.0;
            target_aoa=norm_signed_deg(desired_fpa-air_fpa);
        }else if(r.command.has_target_aoa&&isfinite(r.command.target_aoa))target_aoa=r.command.target_aoa;
        else if(isfinite(r.command.target_pitch))target_aoa=r.command.target_pitch-synthetic.flight_path_angle;
        target_aoa=clampd(target_aoa,-cfg->vehicle.maximum_angle_of_attack,cfg->vehicle.maximum_angle_of_attack);
        ctx.gear=r.command.gear;ctx.brakes=r.command.brakes;ctx.airbrakes=r.command.airbrakes?1:0;
        double dt=taem_exec_owns_vehicle(&shadow.taem_exec)?
            (geo.altitude>25000?.8:(geo.altitude>8000?.45:.25)):
            1.0/clampd(cfg->guidance.guidance_rate,2.0,30.0);
        dt=fmin(dt,maxdur-elapsed);
        attitude_step(&response,target_bank,target_aoa,&at,&cfg->vehicle,&cfg->guidance,dt,cal0->physics);
        ctx.bank=response.actual_bank;ctx.aoa=response.actual_aoa;ctx.ut=state.ut;ctx.mass=state.mass;
        state=rk4_aero(state,dt,&ctx);elapsed+=dt;
        guidance_result_clear(&r);
    }
    out.final_state=state;
    if(out.closest_distance==DBL_MAX)out.closest_distance=great_circle_distance(predictor_geo_point(state.position,p,state.ut),target,p->radius);
    out.physics_relative_uncertainty=telemetry->physics_certified_uncertainty>0?clampd(telemetry->physics_certified_uncertainty,.06,.90):.45;
    if(outcome){
        outcome->terminal_prediction_valid=out.shadow_terminal_prediction_valid;
        outcome->terminal_policy_feasible=out.shadow_terminal_policy_feasible;
        outcome->terminal_path_committed=out.shadow_terminal_path_committed;
        outcome->final_approach_captured=out.shadow_final_approach_captured;
        outcome->aborted=out.shadow_aborted;
    }
    g_planner_trace=saved_planner_trace;
    return out;
}

EntryPrediction predictor_simulate_entry_guidance_shadow(VehicleState state,const Telemetry*telemetry,
        const GuidanceMachine*guidance,const DeorbitPlan*plan,const PlanetModel*p,AerodynamicModel aero,
        const AerodynamicEnvelope*env,const TrajectoryCalibrationModel*cal,const LandingConfiguration*cfg,
        double maxdur,bool include){
    EntryPrediction empty;memset(&empty,0,sizeof(empty));trajectory_init(&empty.trajectory);
    if(!plan)return empty;
    TerminalShadowOutcome outcome;
    EntryPrediction out=predictor_simulate_guidance_shadow_core(state,telemetry,guidance,plan,p,aero,env,cal,cfg,
        maxdur,include,false,true,&outcome);
    out.uncertainty_scenarios=out.shadow_guidance_used?1u:0u;
    return out;
}


EntryPrediction predictor_simulate_terminal_shadow(VehicleState state,const Telemetry*telemetry,
        const GuidanceMachine*guidance,const DeorbitPlan*plan,const PlanetModel*p,AerodynamicModel aero,
        const AerodynamicEnvelope*env,const TrajectoryCalibrationModel*cal,const LandingConfiguration*cfg,
        double maxdur,bool include){
    TerminalShadowOutcome outcome;
    EntryPrediction out=predictor_simulate_guidance_shadow_core(state,telemetry,guidance,plan,p,aero,env,cal,cfg,
        maxdur,include,true,false,&outcome);
    out.uncertainty_scenarios=out.shadow_guidance_used?1u:0u;
    out.terminal_survivability_stress_score=out.shadow_guidance_used&&terminal_shadow_survives(&out,&outcome,&cfg->vehicle)?1.0:0.0;
    return out;
}

EntryPrediction predictor_simulate_terminal_shadow_ensemble(VehicleState state,const Telemetry*telemetry,
        const GuidanceMachine*guidance,const DeorbitPlan*plan,const PlanetModel*p,AerodynamicModel aero,
        const AerodynamicEnvelope*env,const TrajectoryCalibrationModel*cal,const LandingConfiguration*cfg,
        double maxdur,bool include){
    TerminalShadowOutcome nominal_outcome;
    EntryPrediction nominal=predictor_simulate_guidance_shadow_core(state,telemetry,guidance,plan,p,aero,env,cal,cfg,maxdur,include,true,false,&nominal_outcome);
    if(!nominal.shadow_guidance_used)return nominal;
    AeroContext base_ctx={.p=p,.e=env,.c=cal,.v=&cfg->vehicle,.bank=telemetry->roll,.aoa=telemetry->angle_of_attack,
        .ut=state.ut,.mass=state.mass,.gear=telemetry->gear,.brakes=telemetry->brakes,.airbrakes=telemetry->has_airbrakes&&telemetry->airbrakes};
    AtmosState initial=atmosphere(state.position,state.velocity,&base_ctx);
    double sigma=clampd(initial.physics_uncertainty,.08,.40);nominal.physics_relative_uncertainty=sigma;
    /* Three deterministic sigma cases are a robustness stress score, not a
       calibrated survival probability. Keep the name explicit until held-out
       terminal outcomes support reliability/Brier calibration. */
    double stress_score=terminal_shadow_survives(&nominal,&nominal_outcome,&cfg->vehicle)?.50:0.0;
    unsigned scenarios=1;
    for(int direction=-1;direction<=1;direction+=2){
        TrajectoryCalibrationModel stress=cal?*cal:(TrajectoryCalibrationModel){.density_scale=1,.drag_scale=1,.lift_scale=1,.bank_effectiveness=1,.speed_of_sound=340,.speed_of_sound_scale=1,.stress_drag_scale=1,.stress_lift_scale=1,.confidence=.05};
        double signed_sigma=direction*sigma;
        stress.stress_drag_scale=clampd(1.0+signed_sigma,.60,1.40);
        stress.stress_lift_scale=clampd(1.0-signed_sigma,.60,1.40);
        stress.density_scale=clampd(fmax(.1,stress.density_scale)*(1.0+.20*signed_sigma),.70,1.30);
        TerminalShadowOutcome scenario_outcome;
        EntryPrediction scenario=predictor_simulate_guidance_shadow_core(state,telemetry,guidance,plan,p,aero,env,&stress,cfg,fmin(maxdur,360.0),false,true,false,&scenario_outcome);
        if(terminal_shadow_survives(&scenario,&scenario_outcome,&cfg->vehicle))stress_score+=.25;
        scenarios++;entry_prediction_clear(&scenario);
    }
    nominal.terminal_survivability_stress_score=clampd(stress_score,0,1);nominal.uncertainty_scenarios=scenarios;
    return nominal;
}

EntryPredictionAssessment predictor_assess_entry_prediction(const EntryPrediction*pr,const VehicleProfile*v,double available_bank,const EntryControlPlan*plan){
    EntryPredictionAssessment a;memset(&a,0,sizeof(a));
    a.dynamic_pressure_ratio=NAN;a.g_load_ratio=NAN;a.minimum_speed_ratio=NAN;a.bank_margin=NAN;a.aoa_margin=NAN;
    if(!pr||!v||!plan||!plan->valid||v->maximum_dynamic_pressure<=0||v->maximum_g_load<=0||v->minimum_safe_speed<=0||v->maximum_angle_of_attack<=0)return a;
    a.dynamic_pressure_ratio=pr->peak_dynamic_pressure/v->maximum_dynamic_pressure;
    a.g_load_ratio=pr->peak_g_load/v->maximum_g_load;
    if(isfinite(pr->minimum_entry_speed))a.minimum_speed_ratio=pr->minimum_entry_speed/v->minimum_safe_speed;
    a.bank_margin=isfinite(pr->minimum_bank_control_margin)?pr->minimum_bank_control_margin:available_bank-fabs(plan->target_bank);
    a.aoa_margin=isfinite(pr->minimum_aoa_control_margin)?pr->minimum_aoa_control_margin:
        fmin(plan->target_aoa,v->maximum_angle_of_attack-plan->target_aoa);
    a.valid=pr->entered_atmosphere&&isfinite(a.dynamic_pressure_ratio)&&isfinite(a.g_load_ratio)&&
        isfinite(a.minimum_speed_ratio)&&isfinite(a.bank_margin)&&isfinite(a.aoa_margin)&&
        isfinite(plan->target_bank)&&isfinite(plan->target_aoa);
    if(!a.valid)return a;
    a.dynamic_pressure_violation=a.dynamic_pressure_ratio>1.0+1e-9;
    a.g_load_violation=a.g_load_ratio>1.0+1e-9;
    /* The reduced Entry predictor has no separate stall-fraction state. Minimum
       safe speed is therefore the conservative aerodynamic-margin proxy; the live
       controller's measured stall_fraction remains authoritative when available. */
    a.stall_risk=a.minimum_speed_ratio<1.0;
    a.control_margin_violation=a.bank_margin<0.0||a.aoa_margin<0.0||
        pr->maximum_abs_angle_of_attack>v->maximum_angle_of_attack;
    a.safe=!a.dynamic_pressure_violation&&!a.g_load_violation&&
        !a.stall_risk&&!a.control_margin_violation;
    /*
     * A TAEM prediction is terminal-feasible only if the same strict MM304
     * ownership event was actually reached.  Do not add a second speed/range
     * compatibility envelope here.
     */
    a.terminal_feasible=a.safe&&pr->reached_taem&&
        !pr->taem_ownership_boundary_missed&&
        isfinite(pr->taem_energy_error)&&pr->taem_energy_error>=0.0;
    return a;
}

static void entry_supervision_apply_forecast(EntryControlPlan*plan,const EntryPrediction*pr,const GuidanceSettings*s){
    if(!plan||!pr||!s)return;
    double capture_hint=fmax(s->hac_radius*8.0,s->taem_interface_range*2.5);
    plan->terminal_ready=pr->reached_taem;
    plan->taem_range_error=pr->reached_taem?pr->taem_range_error:
        (isfinite(pr->closest_distance)?pr->closest_distance-capture_hint:NAN);
    plan->taem_speed=pr->reached_taem?pr->taem_speed:NAN;
    plan->taem_energy_error=pr->reached_taem?pr->taem_energy_error:NAN;
    plan->closest_distance=pr->closest_distance;
    plan->predicted_reversals=pr->s_turn_reversals;
}

static double entry_supervision_adjustment_cost(const EntryControlPlan*c,const EntryControlPlan*n,const VehicleProfile*v,double bank_bound,double aoa_bound){
    double db=fabs(c->target_bank-n->target_bank)/fmax(bank_bound,1.0);
    double da=fabs(c->target_aoa-n->target_aoa)/fmax(aoa_bound,1.0);
    double vehicle_db=fabs(c->target_bank-n->target_bank)/fmax(v->maximum_bank_angle,1.0);
    double vehicle_da=fabs(c->target_aoa-n->target_aoa)/fmax(v->maximum_angle_of_attack,1.0);
    return db*db+da*da+.05*(vehicle_db*vehicle_db+vehicle_da*vehicle_da);
}

static bool entry_prediction_committed_boundary_miss(const EntryPrediction*pr,
        const EntryControlPlan*plan,double origin_ut){
    if(!pr||!plan||!pr->taem_ownership_boundary_missed||
       !isfinite(pr->final_state.ut)||!isfinite(origin_ut)||
       !isfinite(plan->segment_duration)||plan->segment_duration<0)return false;
    /* A forecast may deliberately hold one candidate policy beyond this plan's
       receding-horizon segment so future terminal geometry can be scored.  A
       V_TAEM miss after that segment expires is not proof the current command is
       wrong; MM304 is expected to solve again first.  Only a miss reached while
       this segment is still executable is an immediate replan condition. */
    return pr->final_state.ut<=origin_ut+plan->segment_duration+1e-6;
}

EntrySupervisionResult predictor_supervise_entry_control_to_interface(VehicleState state,const PlanetModel*p,AerodynamicModel aero,const AerodynamicEnvelope*env0,const TrajectoryCalibrationModel*cal0,const VehicleProfile*v,const LandingSite*site,const GuidanceSettings*s,const TaemInterfaceTarget*interface_target,bool enforce_terminal_delivery_budget,double initial_bank,double initial_bank_rate,double initial_aoa,double initial_aoa_rate,double current_sign,bool current_leg_established,double current_leg_elapsed,double current_commit_remaining,bool s_turn_side_locked,bool final_heading_lock,const EntryControlPlan*nominal_plan,double maximum_bank_correction,double maximum_aoa_correction,double maxdur){
    EntrySupervisionResult out;memset(&out,0,sizeof(out));out.mode=ENTRY_SUPERVISION_INFEASIBLE;
    planner_trace_begin(state,p,site,initial_bank,initial_aoa,current_sign,current_leg_established,
        current_leg_elapsed,current_commit_remaining,s_turn_side_locked,final_heading_lock);
    if(!p||!v||!site||!s||!isfinite(maxdur)||maxdur<=0)return out;
    double bank_bound=clampd(fabs(maximum_bank_correction),0,v->maximum_bank_angle);
    double aoa_bound=clampd(fabs(maximum_aoa_correction),0,v->maximum_angle_of_attack);
    g_planner_trace.search_horizon_seconds=maxdur;
    g_planner_trace.maximum_bank_correction=bank_bound;g_planner_trace.maximum_aoa_correction=aoa_bound;
    AerodynamicEnvelope uniform;if(!env0){for(int i=0;i<4;i++)uniform.regimes[i]=aero;env0=&uniform;}
    TrajectoryCalibrationModel fallback={.density_scale=1,.drag_scale=1,.lift_scale=1,.bank_effectiveness=1,.speed_of_sound=340,.speed_of_sound_scale=1,.confidence=.05};
    if(!cal0)cal0=&fallback;
    int airbrakes=cal0->physics&&cal0->physics->airbrakes>=0?cal0->physics->airbrakes:0;
    AeroContext current_ctx={.p=p,.e=env0,.c=cal0,.v=v,.bank=initial_bank,.aoa=initial_aoa,.ut=state.ut,.mass=state.mass,.gear=cal0->physics?cal0->physics->gear:false,.brakes=cal0->physics?cal0->physics->brakes:false,.airbrakes=airbrakes};
    AtmosState current_at=atmosphere(state.position,state.velocity,&current_ctx);
    /* Supervision judges the executable nominal MM304 plan against live shared
       authority; the 60 deg ceiling belongs only to autonomous candidate search. */
    double available_bank=bank_limit(&current_at,p,v,v->maximum_bank_angle);

    EntryControlPlan nominal;memset(&nominal,0,sizeof(nominal));
    bool nominal_safe=false;
    EntryControlPlan nominal_forecast={0};
    bool nominal_boundary_missed=false;
    if(nominal_plan&&nominal_plan->valid){
        nominal=*nominal_plan;
        EntryControlPlan forecast_plan=nominal;

        EntryPrediction pr=predictor_simulate_entry_control_plan_to_interface(state,p,aero,env0,cal0,v,site,s,
            interface_target,enforce_terminal_delivery_budget,initial_bank,initial_bank_rate,initial_aoa,
            initial_aoa_rate,current_sign,current_leg_elapsed,&forecast_plan,maxdur,true);
        out.nominal_assessment=predictor_assess_entry_prediction(&pr,v,available_bank,&forecast_plan);
        nominal_boundary_missed=entry_prediction_committed_boundary_miss(&pr,&forecast_plan,state.ut);
        /* Preserve a raw long-horizon warning for telemetry, but do not let a
           post-segment miss invalidate the current command. After at most 120 s
           this replay resumes predictor-private S-turn policy; authoritative
           long-horizon ownership proof requires the guidance-shadow path (P5). */
        out.taem_ownership_boundary_missed=pr.taem_ownership_boundary_missed;
        out.plan=nominal;entry_supervision_apply_forecast(&out.plan,&pr,s);
        nominal_forecast=out.plan;
        planner_trace_add(PREDICTOR_PLAN_CANDIDATE_NOMINAL,&nominal_forecast,&out.nominal_assessment,&pr,0.0,NULL);
        nominal_safe=out.nominal_assessment.valid&&out.nominal_assessment.safe;
        if(out.nominal_assessment.valid&&out.nominal_assessment.terminal_feasible){
            out.valid=true;out.mode=ENTRY_SUPERVISION_PASS_THROUGH;out.selected_assessment=out.nominal_assessment;
            planner_trace_select(PREDICTOR_PLAN_CANDIDATE_NOMINAL,&nominal_forecast,out.mode);
            entry_prediction_clear(&pr);return out;
        }
        entry_prediction_clear(&pr);

        double bank_offsets[3]={0,-bank_bound,bank_bound},aoa_offsets[3]={0,-aoa_bound,aoa_bound};
        double nominal_mag=fabs(nominal.target_bank);
        double nominal_sign=fabs(nominal.target_bank)>1e-6?(nominal.target_bank>=0?1:-1):
            (fabs(current_sign)>.1?(current_sign>=0?1:-1):1);
        double best_cost=INFINITY;EntryControlPlan best_plan;EntryPredictionAssessment best_assessment;memset(&best_plan,0,sizeof(best_plan));memset(&best_assessment,0,sizeof(best_assessment));
        for(size_t bi=0;bi<3;bi++)for(size_t ai=0;ai<3;ai++){
            if(bi==0&&ai==0)continue;
            if((bi>0&&bank_bound<=0)||(ai>0&&aoa_bound<=0))continue;
            double bank_mag=nominal_mag+bank_offsets[bi],aoa=nominal.target_aoa+aoa_offsets[ai];
            if(bank_mag<-1e-9||bank_mag>v->maximum_bank_angle+1e-9||aoa<-1e-9||aoa>v->maximum_angle_of_attack+1e-9)continue;
            EntryControlPlan candidate=nominal;
            candidate.target_bank=bank_mag<=1e-6?0:nominal_sign*bank_mag;
            candidate.target_aoa=aoa;

            candidate.target_turn_radius=fabs(candidate.target_bank)<1?INFINITY:turn_radius_for_bank(&current_at,cal0,fabs(candidate.target_bank));
            if(candidate.segment_duration<=0)candidate.segment_duration=s->s_turn_minimum_leg_duration;
            EntryPrediction cp=predictor_simulate_entry_control_plan_to_interface(state,p,aero,env0,cal0,v,site,s,
                interface_target,enforce_terminal_delivery_budget,initial_bank,initial_bank_rate,initial_aoa,
                initial_aoa_rate,current_sign,current_leg_elapsed,&candidate,maxdur,true);
            EntryPredictionAssessment ca=predictor_assess_entry_prediction(&cp,v,available_bank,&candidate);
            entry_supervision_apply_forecast(&candidate,&cp,s);
            double cost=entry_supervision_adjustment_cost(&candidate,&nominal,v,bank_bound,aoa_bound);
            PredictorPlannerCostTerms bounded_terms={.bounded_adjustment=cost};
            planner_trace_add(PREDICTOR_PLAN_CANDIDATE_BOUNDED,&candidate,&ca,&cp,cost,&bounded_terms);
            if(ca.valid&&ca.terminal_feasible&&cost<best_cost){best_cost=cost;best_plan=candidate;best_assessment=ca;}
            entry_prediction_clear(&cp);
        }
        if(best_plan.valid){
            out.valid=true;out.mode=ENTRY_SUPERVISION_BOUNDED_CORRECTION;out.plan=best_plan;out.selected_assessment=best_assessment;
            out.taem_ownership_boundary_missed=false;
            out.bank_correction=best_plan.target_bank-nominal.target_bank;
            out.aoa_correction=best_plan.target_aoa-nominal.target_aoa;
            planner_trace_select(PREDICTOR_PLAN_CANDIDATE_BOUNDED,&best_plan,out.mode);
            return out;
        }
    }

    /* Compatibility/recovery shim only. This is the pre-refactor broad search,
       including free alpha candidates. Nominal MM304 integration must call this
       supervisor with @ALPHA/@ENTRYLATERAL commands and reach this path only when
       those commands are unsafe or cannot produce a TAEM/HAC-feasible trajectory. */
    EntryControlPlan fallback_plan=predictor_plan_entry_control_legacy(state,p,aero,env0,cal0,v,site,s,
        interface_target,enforce_terminal_delivery_budget,initial_bank,initial_bank_rate,initial_aoa,initial_aoa_rate,
        current_sign,current_leg_established,current_leg_elapsed,current_commit_remaining,s_turn_side_locked,final_heading_lock,maxdur);
    if(fallback_plan.valid){
        EntryPrediction fp=predictor_simulate_entry_control_plan_to_interface(state,p,aero,env0,cal0,v,site,s,
            interface_target,enforce_terminal_delivery_budget,initial_bank,initial_bank_rate,initial_aoa,
            initial_aoa_rate,current_sign,current_leg_elapsed,&fallback_plan,maxdur,false);
        bool fallback_boundary_missed=entry_prediction_committed_boundary_miss(&fp,&fallback_plan,state.ut);
        EntryPredictionAssessment fa=predictor_assess_entry_prediction(&fp,v,available_bank,&fallback_plan);
        entry_supervision_apply_forecast(&fallback_plan,&fp,s);
        /* Legacy broad search is advisory while MM304 still owns an executable
           nominal plan. It may not escape the supervisor's explicit correction
           envelope or silently command the opposite S-turn side; doing so turns a
           supposedly bounded correction into a second private guidance policy. */
        bool fallback_local=true;
        if(nominal_plan&&nominal_plan->valid){
            double db=fallback_plan.target_bank-nominal.target_bank;
            double da=fallback_plan.target_aoa-nominal.target_aoa;
            double nominal_side=fabs(nominal.target_bank)>1e-6?(nominal.target_bank>=0?1:-1):
                (fabs(current_sign)>.1?(current_sign>=0?1:-1):1);
            bool same_side=fabs(fallback_plan.target_bank)<1e-6||fallback_plan.target_bank*nominal_side>=-1e-9;
            fallback_local=same_side&&fabs(db)<=bank_bound+1e-9&&fabs(da)<=aoa_bound+1e-9;
        }
        /* Legacy broad search may replace MM304 only when it stays inside that
           local contract and solves a terminal-feasibility problem MM304 could not,
           or when the nominal itself is unsafe. Otherwise let MM304 replan rather
           than injecting a discontinuous broad-search command. */
        if(fallback_local&&fa.valid&&fa.safe&&
           (fa.terminal_feasible||(!nominal_safe&&!fallback_boundary_missed))){
            out.plan=fallback_plan;out.selected_assessment=fa;
            out.valid=true;out.mode=ENTRY_SUPERVISION_FALLBACK;
            out.taem_ownership_boundary_missed=fp.taem_ownership_boundary_missed;
            out.bank_correction=fallback_plan.target_bank-nominal.target_bank;
            out.aoa_correction=fallback_plan.target_aoa-nominal.target_aoa;
            planner_trace_select(PREDICTOR_PLAN_CANDIDATE_LEGACY,&fallback_plan,out.mode);
            entry_prediction_clear(&fp);return out;
        }
        entry_prediction_clear(&fp);
    }
    if(nominal_safe&&!nominal_boundary_missed){
        out.valid=true;out.mode=ENTRY_SUPERVISION_PASS_THROUGH;
        out.plan=nominal_forecast;out.selected_assessment=out.nominal_assessment;
        planner_trace_select(PREDICTOR_PLAN_CANDIDATE_NOMINAL,&nominal_forecast,out.mode);
    }else g_planner_trace.mode=out.mode;
    return out;
}


EntrySupervisionResult predictor_supervise_entry_control(VehicleState state,const PlanetModel*p,AerodynamicModel aero,const AerodynamicEnvelope*env0,const TrajectoryCalibrationModel*cal0,const VehicleProfile*v,const LandingSite*site,const GuidanceSettings*s,double initial_bank,double initial_bank_rate,double initial_aoa,double initial_aoa_rate,double current_sign,bool current_leg_established,double current_leg_elapsed,double current_commit_remaining,bool s_turn_side_locked,bool final_heading_lock,const EntryControlPlan*nominal_plan,double maximum_bank_correction,double maximum_aoa_correction,double maxdur){
    return predictor_supervise_entry_control_to_interface(state,p,aero,env0,cal0,v,site,s,NULL,false,
        initial_bank,initial_bank_rate,initial_aoa,initial_aoa_rate,current_sign,current_leg_established,
        current_leg_elapsed,current_commit_remaining,s_turn_side_locked,final_heading_lock,nominal_plan,
        maximum_bank_correction,maximum_aoa_correction,maxdur);
}

EntryControlPlan predictor_plan_entry_control(VehicleState state,const PlanetModel*p,AerodynamicModel aero,const AerodynamicEnvelope*env,const TrajectoryCalibrationModel*cal,const VehicleProfile*v,const LandingSite*site,const GuidanceSettings*s,double initial_bank,double initial_bank_rate,double initial_aoa,double initial_aoa_rate,double current_sign,bool current_leg_established,double current_leg_elapsed,double current_commit_remaining,bool s_turn_side_locked,bool final_heading_lock,double maxdur){
    /* Backward-compatible staging API. Existing callers keep the old behavior until
       @INTEGRATE wires the nominal drag/alpha/lateral modules into the supervisor. */
    planner_trace_begin(state,p,site,initial_bank,initial_aoa,current_sign,current_leg_established,
        current_leg_elapsed,current_commit_remaining,s_turn_side_locked,final_heading_lock);
    EntryControlPlan out=predictor_plan_entry_control_legacy(state,p,aero,env,cal,v,site,s,NULL,false,initial_bank,initial_bank_rate,
        initial_aoa,initial_aoa_rate,current_sign,current_leg_established,current_leg_elapsed,
        current_commit_remaining,s_turn_side_locked,final_heading_lock,maxdur);
    if(out.valid)planner_trace_select(PREDICTOR_PLAN_CANDIDATE_LEGACY,&out,ENTRY_SUPERVISION_FALLBACK);
    return out;
}
EntryControlPlan predictor_plan_entry_control_to_interface(VehicleState state,const PlanetModel*p,AerodynamicModel aero,const AerodynamicEnvelope*env,const TrajectoryCalibrationModel*cal,const VehicleProfile*v,const LandingSite*site,const GuidanceSettings*s,const TaemInterfaceTarget*interface_target,bool enforce_terminal_delivery_budget,double initial_bank,double initial_bank_rate,double initial_aoa,double initial_aoa_rate,double current_sign,bool current_leg_established,double current_leg_elapsed,double current_commit_remaining,bool s_turn_side_locked,bool final_heading_lock,double maxdur){
    planner_trace_begin(state,p,site,initial_bank,initial_aoa,current_sign,current_leg_established,
        current_leg_elapsed,current_commit_remaining,s_turn_side_locked,final_heading_lock);
    EntryControlPlan out=predictor_plan_entry_control_legacy(state,p,aero,env,cal,v,site,s,interface_target,enforce_terminal_delivery_budget,
        initial_bank,initial_bank_rate,initial_aoa,initial_aoa_rate,current_sign,current_leg_established,
        current_leg_elapsed,current_commit_remaining,s_turn_side_locked,final_heading_lock,maxdur);
    if(out.valid)planner_trace_select(PREDICTOR_PLAN_CANDIDATE_LEGACY,&out,ENTRY_SUPERVISION_FALLBACK);
    return out;
}
bool deorbit_plan_preburn_state_compatible(const DeorbitPlan*plan,double live_mass,double live_thrust,const GuidanceSettings*s){
    if(!plan||!s||!isfinite(live_mass)||live_mass<=0||!isfinite(live_thrust)||live_thrust<0)return false;
    if(plan->planning_mass>1){
        double limit=fmax(0.0,s->deorbit_mass_uncertainty_fraction);
        if(fabs(live_mass-plan->planning_mass)>plan->planning_mass*(limit+1e-9))return false;
    }
    if(plan->planning_available_thrust>1){
        double limit=fmax(0.0,s->deorbit_thrust_uncertainty_fraction);
        if(fabs(live_thrust-plan->planning_available_thrust)>plan->planning_available_thrust*(limit+1e-9))return false;
    }
    return true;
}
static double deorbit_alignment_position_error(const EntryPrediction*p,const GuidanceSettings*s){
    if(!p||!s||!isfinite(p->taem_along_track)||!isfinite(p->taem_cross_track))return INFINITY;
    return hypot(p->taem_along_track+s->final_approach_distance,p->taem_cross_track);
}

static double deorbit_alignment_course_error(const EntryPrediction*p,const LandingSite*site){
    if(!p||!site||!isfinite(p->taem_course))return INFINITY;
    double runway_offset=fabs(norm_signed_deg(p->taem_course-site->runway_heading));
    return fabs(runway_offset-90.0);
}

static double deorbit_alignment_contract_utilization(const EntryPrediction*p,
        const LandingSite*site,const GuidanceSettings*s){
    double position=deorbit_alignment_position_error(p,s);
    double course=deorbit_alignment_course_error(p,site);
    TaemHandoffContract contract=taem_handoff_contract(s);
    if(!isfinite(position)||!isfinite(course)||
       !(contract.horizontal_radius_m>0.0)||
       !(contract.perpendicular_heading_half_width_deg>0.0))
        return INFINITY;
    return fmax(position/contract.horizontal_radius_m,
        course/contract.perpendicular_heading_half_width_deg);
}

bool deorbit_capture_qualified(const EntryPrediction*p,const LandingSite*site,
        const VehicleProfile*v,const GuidanceSettings*s,double peri,bool require_entry_target){
    (void)site;(void)peri;
    if(!p||!v||!s||!p->entered_atmosphere||!p->mm304_gate_recorded||
       !isfinite(p->entry_range)||!isfinite(p->entry_flight_path_angle)||
       !isfinite(p->entry_course_error)||!isfinite(p->peak_dynamic_pressure)||
       !isfinite(p->peak_g_load))
        return false;

    bool safe=p->entry_flight_path_angle>=s->maximum_entry_flight_path_angle&&
        p->peak_dynamic_pressure<=v->maximum_dynamic_pressure&&
        p->peak_g_load<=v->maximum_g_load;
    if(!safe)return false;

    /*
     * target_entry_range/FPA/course are optimization objectives, not a second
     * safety corridor.  When a caller requests the entry target, require only
     * finite target data; candidate ordering below minimizes those errors.
     */
    if(require_entry_target)
        return isfinite(s->target_entry_range)&&
            isfinite(s->target_entry_flight_path_angle);
    return true;
}

bool deorbit_recovery_qualified(const EntryPrediction*p,const LandingSite*site,
        const VehicleProfile*v,const GuidanceSettings*s,double peri){
    return deorbit_capture_qualified(p,site,v,s,peri,false);
}

bool reentry_guidance_shadow_recovery_qualified(const EntryPrediction*p,
        const VehicleProfile*v,const GuidanceSettings*s){
    if(!p||!v||!s||!p->shadow_guidance_used||!p->entered_atmosphere||
       !p->reached_taem||p->shadow_aborted||
       p->taem_ownership_boundary_missed)
        return false;
    if(!isfinite(p->closest_distance)||!isfinite(p->taem_distance)||
       !isfinite(p->taem_range_error)||!isfinite(p->taem_speed)||
       !isfinite(p->peak_dynamic_pressure)||!isfinite(p->peak_g_load)||
       !isfinite(p->minimum_entry_speed))
        return false;

    bool capture=p->taem_dynamic_interface_target_valid?
        p->taem_dynamic_interface_captured:
        (p->closest_distance>=0.0&&
         p->closest_distance<=s->target_deorbit_capture_radius);
    bool load=p->peak_dynamic_pressure<=v->maximum_dynamic_pressure&&
        p->peak_g_load<=v->maximum_g_load;
    bool speed=p->minimum_entry_speed>=v->minimum_safe_speed&&
        p->taem_speed>=v->minimum_safe_speed;
    return capture&&load&&speed;
}

typedef struct {
    bool valid;
    double impact_along_track,impact_cross_track,impact_range,impact_altitude,impact_speed;
} DeorbitEnergyMargin;
typedef struct {
    double burn_ut,dv,duration,peri,score;
    VehicleState after;
    EntryPrediction pred;
    DeorbitEnergyMargin energy_margin;
    int tier;
    double safety_violation;
    double energy_deficit;
    double energy_excess;
    double entry_fpa_error;
    double entry_course_error;
    double entry_range_error;
} Candidate;

static DeorbitEnergyMargin deorbit_energy_margin_probe(VehicleState after,const PlanetModel*p,
        AerodynamicModel aero,const AerodynamicEnvelope*env,const TrajectoryCalibrationModel*cal,
        const LandingSite*site,const VehicleProfile*v,const GuidanceSettings*s){
    DeorbitEnergyMargin m={0};
    /* Zero maximum bank means no S-turn/lateral energy-management maneuver. The
       normal safe entry AoA schedule still flies the vehicle, so this asks the
       physically useful question: with wings level, does the post-burn state carry
       just beyond KSC? Any normal MM304 bank then has positive disposable energy to
       spend on geometry instead of stretching an undershooting trajectory. */
    EntryPrediction passive=predictor_simulate_entry(after,p,aero,env,cal,v,site,s,0.0,0,0,0,4500,false);
    VehicleState final=passive.final_state;
    GeoPoint geo=predictor_geo_point(final.position,p,final.ut);
    Vector3 up=vnorm(final.position,v3(0,1,0));
    Vector3 surface_velocity=vsub(final.velocity,vcross(planet_rotation_vector(p),final.position));
    double vertical=vdot(surface_velocity,up);
    GeoPoint site_geo={site->latitude,site->longitude,site->altitude};
    double along=0.0,cross=0.0;
    runway_coordinates(geo,site_geo,site->runway_heading,p->radius,&along,&cross);
    m.valid=passive.entered_atmosphere&&isfinite(geo.altitude)&&isfinite(along)&&isfinite(cross)&&
        isfinite(vertical)&&geo.altitude<=site->altitude+750.0&&vertical<0.0;
    m.impact_along_track=along;m.impact_cross_track=cross;
    m.impact_range=great_circle_distance(geo,site_geo,p->radius);
    m.impact_altitude=geo.altitude;m.impact_speed=vmag(surface_velocity);
    entry_prediction_clear(&passive);
    return m;
}
static bool deorbit_energy_margin_qualified(const DeorbitEnergyMargin*m,
        const GuidanceSettings*s,bool strict){
    (void)s;(void)strict;
    return m&&m->valid&&isfinite(m->impact_along_track)&&
        m->impact_along_track>=0.0;
}

static void candidate_clear(Candidate*c){entry_prediction_clear(&c->pred);}
static void candidate_move(Candidate*dst,Candidate*src){*dst=*src;memset(src,0,sizeof(*src));trajectory_init(&src->pred.trajectory);}
static double deorbit_candidate_safety_violation(const EntryPrediction*p,
        const VehicleProfile*v,const GuidanceSettings*s){
    if(!p||!v||!s||!p->entered_atmosphere)return INFINITY;
    double q=predictor_normalized_upper_violation(
        p->peak_dynamic_pressure,v->maximum_dynamic_pressure);
    double g=predictor_normalized_upper_violation(
        p->peak_g_load,v->maximum_g_load);
    double fpa=fmax(0.0,s->maximum_entry_flight_path_angle-
        p->entry_flight_path_angle);
    return fmax(q,fmax(g,fpa));
}

static int tier(Candidate*c,const LandingSite*site,const VehicleProfile*v,
        const GuidanceSettings*s){
    if(!c)return 0;
    bool safe=deorbit_capture_qualified(&c->pred,site,v,s,c->peri,false);
    bool energy=deorbit_energy_margin_qualified(&c->energy_margin,s,false);
    if(safe&&energy&&c->pred.mm304_gate_recorded)return 3;
    if(safe&&energy)return 2;
    if(safe)return 1;
    return 0;
}

static bool impulsive_deorbit_periapsis_plausible(VehicleState coast,double dv,
        const PlanetModel*p,const GuidanceSettings*s){
    (void)s;
    VehicleState impulse=coast;
    Vector3 direction=burn_direction(coast.position,coast.velocity,0,0);
    impulse.velocity=vadd(impulse.velocity,vscale(direction,dv));
    double peri=predictor_postburn_periapsis(impulse,p);
    return isfinite(peri)&&peri<p->atmosphere_depth&&peri>-p->radius;
}

static bool eval_candidate(Candidate*out,VehicleState coast,double dv,
        const PlanetModel*p,AerodynamicModel aero,const AerodynamicEnvelope*env,
        const TrajectoryCalibrationModel*cal,const LandingSite*site,
        const VehicleProfile*v,const GuidanceSettings*s,double thrust){
    memset(out,0,sizeof(*out));
    if(!impulsive_deorbit_periapsis_plausible(coast,dv,p,s))return false;

    double dur=0.0,delivered=0.0;
    VehicleState after=retro_burn_scenario(coast,dv,thrust,
        s->deorbit_maximum_throttle,s->deorbit_throttle_ramp_duration,
        p,0,0,&dur,&delivered,cal?cal->physics:NULL);
    double dv_tol=sqrt(DBL_EPSILON)*fmax(1.0,fabs(dv));
    if(!isfinite(dur)||!isfinite(delivered)||delivered+dv_tol<dv)
        return false;

    double peri=predictor_postburn_periapsis(after,p);
    if(!isfinite(peri)||peri>=p->atmosphere_depth||peri<=-p->radius)
        return false;

    EntryPrediction pr=predictor_simulate_entry(after,p,aero,env,cal,v,site,s,
        v->maximum_bank_angle,0,0,0,4500,false);
    if(!pr.entered_atmosphere||!isfinite(pr.entry_range)||
       !isfinite(pr.entry_flight_path_angle)||!isfinite(pr.entry_course_error)||
       !isfinite(pr.peak_dynamic_pressure)||!isfinite(pr.peak_g_load)){
        entry_prediction_clear(&pr);
        return false;
    }

    DeorbitEnergyMargin energy_margin=
        deorbit_energy_margin_probe(after,p,aero,env,cal,site,v,s);

    out->burn_ut=coast.ut+dur*.5;
    out->dv=dv;
    out->duration=dur;
    out->after=after;
    out->peri=peri;
    out->energy_margin=energy_margin;
    out->pred=pr;

    out->safety_violation=deorbit_candidate_safety_violation(&pr,v,s);
    out->energy_deficit=energy_margin.valid?
        fmax(0.0,-energy_margin.impact_along_track):INFINITY;
    out->energy_excess=energy_margin.valid?
        fmax(0.0,energy_margin.impact_along_track):INFINITY;
    out->entry_fpa_error=fabs(
        pr.entry_flight_path_angle-s->target_entry_flight_path_angle);
    out->entry_course_error=fabs(pr.entry_course_error);
    out->entry_range_error=fabs(pr.entry_range-s->target_entry_range);
    out->tier=tier(out,site,v,s);

    /* Kept only for telemetry/backward-compatible traces; not used to decide. */
    out->score=out->tier>=2?out->entry_fpa_error:out->safety_violation;
    return true;
}

static bool better(const Candidate*a,const Candidate*b,bool hasb){
    if(!a)return false;
    if(!hasb||!b)return true;
    if(a->tier!=b->tier)return a->tier>b->tier;

    if(a->safety_violation<b->safety_violation)return true;
    if(a->safety_violation>b->safety_violation)return false;
    if(a->energy_deficit<b->energy_deficit)return true;
    if(a->energy_deficit>b->energy_deficit)return false;

    /*
     * Once safety and positive unpowered reach are equal, follow the configured
     * entry target without mixing units: FPA, course, then range.  Finally
     * minimize excess energy and burn effort.
     */
    if(a->entry_fpa_error<b->entry_fpa_error)return true;
    if(a->entry_fpa_error>b->entry_fpa_error)return false;
    if(a->entry_course_error<b->entry_course_error)return true;
    if(a->entry_course_error>b->entry_course_error)return false;
    if(a->entry_range_error<b->entry_range_error)return true;
    if(a->entry_range_error>b->entry_range_error)return false;
    if(a->energy_excess<b->energy_excess)return true;
    if(a->energy_excess>b->energy_excess)return false;
    if(a->dv<b->dv)return true;
    if(a->dv>b->dv)return false;
    return a->burn_ut<b->burn_ut;
}

typedef struct {
    double timing_offset,thrust_scale,dv_offset,mass_scale,position_offset,velocity_offset;
    double radial_error,normal_error,atmosphere_fraction;
    double aero_drag_scale,aero_lift_scale;
} BurnStressCase;

typedef struct {
    unsigned scenarios,passed,unsafe;
    unsigned recovery_passed;
    double pass_fraction,recovery_pass_fraction,worst_closest,worst_taem_error,worst_alignment_error,worst_entry_angle,min_periapsis,max_dynamic_pressure,max_g_load;
    bool qualified,recovery_qualified;
} RobustAssessment;

static bool hard_unsafe(const EntryPrediction*pr,double peri,const VehicleProfile*v,const GuidanceSettings*s,bool burn_completed){
    if(!burn_completed||!pr->entered_atmosphere)return true;
    (void)peri;
    if(pr->entry_flight_path_angle<s->maximum_entry_flight_path_angle)return true;
    if(pr->peak_dynamic_pressure>v->maximum_dynamic_pressure)return true;
    if(pr->peak_g_load>v->maximum_g_load)return true;
    return false;
}

static RobustAssessment assess_candidate_robustness(const Candidate*c,VehicleState initial,const PlanetModel*p,AerodynamicModel aero,const AerodynamicEnvelope*env,const TrajectoryCalibrationModel*cal,const LandingSite*site,const VehicleProfile*v,const GuidanceSettings*s,double thrust,bool trace){
    double timing=s->deorbit_timing_uncertainty,thrust_unc=s->deorbit_thrust_uncertainty_fraction,dv_unc=s->deorbit_delta_v_uncertainty;
    double mass_unc=s->deorbit_mass_uncertainty_fraction,pos_unc=s->deorbit_position_uncertainty,vel_unc=s->deorbit_velocity_uncertainty;
    double pointing=s->deorbit_pointing_uncertainty,atmos=s->deorbit_atmosphere_uncertainty_fraction;
    double aero_unc=.30;
    if(cal&&cal->physics){
        const VesselPhysicsModel*m=cal->physics;
        if(m->certified_uncertainty>0)aero_unc=m->certified_uncertainty;
        if(m->model_residual_confidence>.20)aero_unc=fmax(aero_unc,m->model_residual*clampd(m->model_residual_confidence,.25,1));
    }
    /* A globally credible Mach envelope can still leave a particular trajectory
       outside the local q/Mach/AoA/configuration cells represented by prior-flight
       force data. Treat analytic-fallback dwell time as epistemic uncertainty
       instead of granting it the same local evidence as a direct cell. */
    double aero_seconds=c->pred.physics_observed_seconds+c->pred.physics_fallback_seconds;
    if(aero_seconds>1.0&&c->pred.physics_fallback_seconds>0&&aero_unc<.30){
        double fallback_fraction=clampd(c->pred.physics_fallback_seconds/aero_seconds,0,1);
        aero_unc+=fallback_fraction*(.30-aero_unc);
    }
    aero_unc=clampd(aero_unc,.08,.40);
    BurnStressCase cases[]={
        {0,1,0,1,0,0,0,0,0,0,0},
        { timing,1-thrust_unc,-dv_unc,1+mass_unc, pos_unc,-vel_unc, pointing,0,0,0,0},
        {-timing,1+thrust_unc, dv_unc,fmax(.5,1-mass_unc),-pos_unc, vel_unc,-pointing,0,0,0,0},
        { timing,1-thrust_unc,0,1+mass_unc,0,-vel_unc,0, pointing,0,0,0},
        {-timing,1+thrust_unc,0,fmax(.5,1-mass_unc),0, vel_unc,0,-pointing,0,0,0},
        {0,1,-dv_unc,1+mass_unc, pos_unc,0, pointing,0, atmos,0,0},
        {0,1, dv_unc,fmax(.5,1-mass_unc),-pos_unc,0,-pointing,0,-atmos,0,0},
        {0,1,0,1, pos_unc, vel_unc,0, pointing, atmos,0,0},
        {0,1,0,1,-pos_unc,-vel_unc,0,-pointing,-atmos,0,0},
        { timing,1,0,1+mass_unc, pos_unc, vel_unc,-pointing, pointing, atmos,0,0},
        {-timing,1,0,fmax(.5,1-mass_unc),-pos_unc,-vel_unc, pointing,-pointing,-atmos,0,0},
        {0,1-thrust_unc,-dv_unc,1+mass_unc,0,0,0,0, atmos,0,0},
        {0,1+thrust_unc, dv_unc,fmax(.5,1-mass_unc),0,0,0,0,-atmos,0,0},
        /* Shuttle certification explicitly stressed the aerodynamic data-book
           uncertainty rather than letting the flight computer relearn the
           vehicle during entry.  Worst-case drag-high/lift-low and the
           converse expose both energy and crossrange/control sensitivity. */
        {0,1,0,1,0,0,0,0,0,1+aero_unc,fmax(.50,1-aero_unc)},
        {0,1,0,1,0,0,0,0,0,fmax(.50,1-aero_unc),1+aero_unc},
    };
    RobustAssessment r;memset(&r,0,sizeof(r));
    r.worst_closest=0;r.worst_taem_error=0;r.worst_alignment_error=0;r.worst_entry_angle=DBL_MAX;r.min_periapsis=DBL_MAX;
    double nominal_start=c->burn_ut-c->duration*.5;
    for(size_t i=0;i<sizeof(cases)/sizeof(cases[0]);i++){
        BurnStressCase sc=cases[i];
        VehicleState scenario_state=initial;
        Vector3 radial=vnorm(initial.position,v3(1,0,0));
        Vector3 along=vnorm(vproject_plane(initial.velocity,radial),vnorm(initial.velocity,v3(0,1,0)));
        scenario_state.position=vadd(scenario_state.position,vscale(radial,sc.position_offset));
        scenario_state.velocity=vadd(scenario_state.velocity,vscale(along,sc.velocity_offset));
        scenario_state.mass=fmax(1,scenario_state.mass*sc.mass_scale);
        double start=fmax(scenario_state.ut+5,nominal_start+sc.timing_offset);
        VehicleState coast=predictor_propagate_vacuum(scenario_state,start,p,3);
        double scenario_dv=fmax(.2,c->dv+sc.dv_offset),duration=0,delivered=0;
        VehicleState after=retro_burn_scenario(coast,scenario_dv,thrust*fmax(.05,sc.thrust_scale),s->deorbit_maximum_throttle,s->deorbit_throttle_ramp_duration,p,sc.radial_error,sc.normal_error,&duration,&delivered,cal?cal->physics:NULL);
        bool burn_completed=delivered>=scenario_dv-.25&&duration<599;
        double peri=predictor_postburn_periapsis(after,p);
        TrajectoryCalibrationModel scenario_cal=*cal;
        scenario_cal.density_scale=clampd(cal->density_scale*(1+sc.atmosphere_fraction),.35,2.8);
        scenario_cal.stress_drag_scale=sc.aero_drag_scale>0?sc.aero_drag_scale:1;
        scenario_cal.stress_lift_scale=sc.aero_lift_scale>0?sc.aero_lift_scale:1;
        EntryPrediction pr=predictor_simulate_entry(after,p,aero,env,&scenario_cal,v,site,s,v->maximum_bank_angle,0,0,0,4500,false);
        bool base_pass=burn_completed&&deorbit_capture_qualified(&pr,site,v,s,peri,true);
        bool base_recovery=burn_completed&&deorbit_recovery_qualified(&pr,site,v,s,peri);
        DeorbitEnergyMargin energy_margin={0};
        if(base_pass||base_recovery)
            energy_margin=deorbit_energy_margin_probe(after,p,aero,env,&scenario_cal,site,v,s);
        bool pass=base_pass&&deorbit_energy_margin_qualified(&energy_margin,s,true);
        bool recovery=base_recovery&&deorbit_energy_margin_qualified(&energy_margin,s,false);
        bool unsafe=hard_unsafe(&pr,peri,v,s,burn_completed);
        double align_metric=deorbit_alignment_contract_utilization(&pr,site,s);
        double align_pos=deorbit_alignment_position_error(&pr,s);
        double align_course=deorbit_alignment_course_error(&pr,site);
        if(trace){
            fprintf(stderr,"stress[%zu] pass=%d recovery=%d unsafe=%d timing=%+.1f dv=%+.1f atm=%+.2f aeroD=%.2f aeroL=%.2f entry=%.1fkm/%.2fdeg gate=%d %.1fkm x=%.1f y=%.1f course=%.1f FPA=%.2f V=%.0f align=%.1fkm/%.1fdeg h=%.1fkm V=%.0f reached=%d min=%.0f rev=%u closest=%.1fkm Pe=%.1fkm q=%.1fkPa g=%.2f\n",
                i,pass?1:0,recovery?1:0,unsafe?1:0,sc.timing_offset,sc.dv_offset,sc.atmosphere_fraction,scenario_cal.stress_drag_scale,scenario_cal.stress_lift_scale,
                pr.entry_range/1000,pr.entry_flight_path_angle,pr.mm304_gate_recorded?1:0,pr.mm304_gate_range/1000,pr.mm304_gate_along_track/1000,pr.mm304_gate_cross_track/1000,pr.mm304_gate_course,pr.mm304_gate_flight_path_angle,pr.mm304_gate_speed,
                align_pos/1000,align_course,pr.taem_altitude/1000,pr.taem_speed,pr.reached_taem?1:0,pr.minimum_entry_speed,pr.s_turn_reversals,pr.closest_distance/1000,peri/1000,pr.peak_dynamic_pressure/1000,pr.peak_g_load);
            fprintf(stderr,"  energy-margin impact valid=%d along=%+.1fkm cross=%+.1fkm range=%.1fkm V=%.0f\n",
                energy_margin.valid?1:0,energy_margin.impact_along_track/1000.0,energy_margin.impact_cross_track/1000.0,
                energy_margin.impact_range/1000.0,energy_margin.impact_speed);
        }
        r.scenarios++;
        if(pass)r.passed++;
        if(recovery)r.recovery_passed++;
        if(unsafe)r.unsafe++;
        if(isfinite(pr.closest_distance))r.worst_closest=fmax(r.worst_closest,pr.closest_distance);
        if(isfinite(pr.taem_range_error))r.worst_taem_error=fmax(r.worst_taem_error,fabs(pr.taem_range_error));
        if(isfinite(align_metric))r.worst_alignment_error=fmax(r.worst_alignment_error,align_metric);
        if(pr.entered_atmosphere&&isfinite(pr.entry_flight_path_angle))r.worst_entry_angle=fmin(r.worst_entry_angle,pr.entry_flight_path_angle);
        if(isfinite(peri))r.min_periapsis=fmin(r.min_periapsis,peri);
        r.max_dynamic_pressure=fmax(r.max_dynamic_pressure,pr.peak_dynamic_pressure);
        r.max_g_load=fmax(r.max_g_load,pr.peak_g_load);
        entry_prediction_clear(&pr);
    }
    if(r.worst_entry_angle==DBL_MAX)r.worst_entry_angle=0;
    if(r.min_periapsis==DBL_MAX)r.min_periapsis=-DBL_MAX;
    r.pass_fraction=r.scenarios?(double)r.passed/r.scenarios:0;
    r.recovery_pass_fraction=r.scenarios?(double)r.recovery_passed/r.scenarios:0;
    r.qualified=c->tier==3&&r.pass_fraction+1e-9>=s->deorbit_robust_minimum_pass_fraction&&r.unsafe==0;
    r.recovery_qualified=deorbit_recovery_qualified(&c->pred,site,v,s,c->peri)&&
        deorbit_energy_margin_qualified(&c->energy_margin,s,false)&&
        r.recovery_pass_fraction+1e-9>=s->deorbit_robust_minimum_pass_fraction&&r.unsafe==0;
    return r;
}

static bool robust_better(const Candidate*a,const RobustAssessment*ra,const Candidate*b,const RobustAssessment*rb,bool hasb){
    if(!hasb)return true;
    if(ra->qualified!=rb->qualified)return ra->qualified;
    if(ra->recovery_qualified!=rb->recovery_qualified)return ra->recovery_qualified;
    if(ra->unsafe!=rb->unsafe)return ra->unsafe<rb->unsafe;
    if(ra->passed!=rb->passed)return ra->passed>rb->passed;
    if(ra->recovery_passed!=rb->recovery_passed)return ra->recovery_passed>rb->recovery_passed;
    /* Once candidates are in the same qualified stress class, let the
       configured entry-angle/periapsis targets in Candidate.score decide the
       trajectory. Independent "shallower is always better" tie-breaks defeat
       those targets and can select a grazing entry even when a safe deeper
       solution is available. */
    if(ra->worst_alignment_error<rb->worst_alignment_error)return true;
    if(ra->worst_alignment_error>rb->worst_alignment_error)return false;
    if(ra->worst_closest<rb->worst_closest)return true;
    if(ra->worst_closest>rb->worst_closest)return false;
    return better(a,b,true);
}

static double deorbit_finite_burn_periapsis(VehicleState coast,double dv,
        const PlanetModel*p,const GuidanceSettings*s,double thrust,
        const TrajectoryCalibrationModel*cal){
    double duration=0.0,delivered=0.0;
    VehicleState after=retro_burn_scenario(coast,dv,thrust,
        s->deorbit_maximum_throttle,s->deorbit_throttle_ramp_duration,
        p,0,0,&duration,&delivered,cal?cal->physics:NULL);
    double tolerance=sqrt(DBL_EPSILON)*fmax(1.0,fabs(dv));
    if(!isfinite(duration)||!isfinite(delivered)||
       delivered+tolerance<dv)
        return NAN;
    return predictor_postburn_periapsis(after,p);
}

static bool deorbit_solve_target_periapsis(VehicleState coast,
        const PlanetModel*p,const GuidanceSettings*s,double thrust,
        const TrajectoryCalibrationModel*cal,double*out_dv){
    if(!out_dv||!p||!s||!(thrust>0.0)||
       !isfinite(s->target_post_burn_periapsis_altitude))
        return false;

    double low=0.0;
    double high=vmag(coast.velocity);
    if(!(high>0.0)||!isfinite(high))return false;

    double low_peri=deorbit_finite_burn_periapsis(
        coast,low,p,s,thrust,cal);
    double high_peri=deorbit_finite_burn_periapsis(
        coast,high,p,s,thrust,cal);
    double target=s->target_post_burn_periapsis_altitude;
    if(!isfinite(low_peri)||!isfinite(high_peri))return false;

    if(low_peri<=target){
        *out_dv=0.0;
        return true;
    }
    if(high_peri>target)return false;

    /* Numerical bisection only; the decision target is configured periapsis. */
    for(int i=0;i<48;i++){
        double mid=.5*(low+high);
        double peri=deorbit_finite_burn_periapsis(
            coast,mid,p,s,thrust,cal);
        if(!isfinite(peri))return false;
        if(peri>target)low=mid;
        else high=mid;
    }
    *out_dv=.5*(low+high);
    return isfinite(*out_dv)&&*out_dv>=0.0;
}

static bool deorbit_evaluate_start(Candidate*out,VehicleState initial,
        double burn_start,double dv_offset,const PlanetModel*p,
        AerodynamicModel aero,const AerodynamicEnvelope*env,
        const TrajectoryCalibrationModel*cal,const LandingSite*site,
        const VehicleProfile*v,const GuidanceSettings*s,double thrust){
    if(!out||!isfinite(burn_start)||burn_start<initial.ut)return false;
    VehicleState coast=predictor_propagate_vacuum(
        initial,burn_start,p,12);
    double dv=0.0;
    if(!deorbit_solve_target_periapsis(
            coast,p,s,thrust,cal,&dv))
        return false;
    dv=fmax(0.0,dv+dv_offset);
    return eval_candidate(out,coast,dv,p,aero,env,cal,site,v,s,thrust);
}

static EntryPrediction deorbit_executable_policy_witness(const Candidate*c,
        const PlanetModel*p,AerodynamicModel aero,const AerodynamicEnvelope*env,
        const TrajectoryCalibrationModel*cal,const LandingSite*site,
        const VehicleProfile*v,const GuidanceSettings*s){
    EntryPrediction empty;memset(&empty,0,sizeof(empty));trajectory_init(&empty.trajectory);
    if(!c||!p||!env||!cal||!site||!v||!s)return empty;

    /* The reduced predictor remains a search model.  Final certification is a
       separate witness that executes the production MM304 guidance law from the
       selected post-burn state.  This prevents a biased reduced policy from both
       proposing and certifying the same deorbit. */
    LandingConfiguration cfg=landing_configuration_default();
    cfg.site=*site;cfg.vehicle=*v;cfg.guidance=*s;

    /* Above meaningful aerodynamic load, Entry Interface owns an inertial
       capture rather than a high-incidence atmospheric trim.  Seed the witness
       wings-level and zero-incidence; production guidance then selects the first
       S-turn side from runway/TAEM geometry when aerodynamic authority arrives. */
    PredictorGuidance response;memset(&response,0,sizeof(response));
    response.actual_bank=0.0;response.actual_aoa=0.0;
    AeroContext ctx={.p=p,.e=env,.c=cal,.v=&cfg.vehicle,.bank=0.0,.aoa=0.0,
        .ut=c->after.ut,.mass=c->after.mass,
        .gear=cal->physics?cal->physics->gear:false,
        .brakes=cal->physics?cal->physics->brakes:false,
        .airbrakes=0};
    Telemetry seed=guidance_shadow_telemetry(NULL,c->after,&response,&ctx,p,&cfg,NULL);

    GuidanceMachine guidance;
    guidance_initialize_reentry_continuation(&guidance,&seed,p,&cfg,1.0,false,env,cal);

    DeorbitPlan executed;memset(&executed,0,sizeof(executed));trajectory_init(&executed.trajectory);
    executed.created_ut=c->after.ut;executed.burn_ut=c->burn_ut;executed.delta_v=c->dv;
    executed.estimated_burn_duration=c->duration;
    executed.predicted_post_burn_periapsis_altitude=c->peri;
    executed.live_cutoff_capture_qualified=true;executed.execution_qualified=true;
    executed.achieved_state_verified=true;executed.achieved_state_capture_qualified=true;

    EntryPrediction witness=predictor_simulate_entry_guidance_shadow(c->after,&seed,&guidance,
        &executed,p,aero,env,cal,&cfg,4500.0,true);
    trajectory_clear(&executed.trajectory);
    return witness;
}

bool deorbit_plan_create(DeorbitPlan*out,VehicleState state,double period,
        const PlanetModel*p,AerodynamicModel aero,
        const AerodynamicEnvelope*env,const TrajectoryCalibrationModel*cal,
        const LandingSite*site,const VehicleProfile*v,
        const GuidanceSettings*s,double thrust){
    if(!out||!p||!env||!cal||!site||!v||!s||
       !isfinite(period)||!(period>0.0)||!(thrust>0.0))
        return false;

    memset(out,0,sizeof(*out));
    trajectory_init(&out->trajectory);

    double lead=fmax(0.0,s->minimum_planning_lead_time);
    const int time_samples=49; /* numerical search resolution only */
    double coarse_step=period/(double)(time_samples-1);

    Candidate nominal;
    memset(&nominal,0,sizeof(nominal));
    trajectory_init(&nominal.pred.trajectory);
    bool nominal_valid=false;

    /*
     * One orbital period contains the next burn opportunity in orbital phase.
     * Planet rotation is already present in the propagated state/site geometry,
     * so no arbitrary multi-orbit count is needed for the next executable plan.
     */
    for(int i=0;i<time_samples;i++){
        double offset=lead+period*(double)i/(double)(time_samples-1);
        Candidate c;
        if(!deorbit_evaluate_start(&c,state,state.ut+offset,0.0,
                p,aero,env,cal,site,v,s,thrust))
            continue;
        if(better(&c,&nominal,nominal_valid)){
            if(nominal_valid)candidate_clear(&nominal);
            candidate_move(&nominal,&c);
            nominal_valid=true;
        }else candidate_clear(&c);
    }
    if(!nominal_valid)return false;

    /*
     * Refine burn time around the best orbital-phase sample.  Bisection of the
     * search interval is numerical convergence, not a flight-policy threshold;
     * delta-v is re-solved from the configured periapsis at every time sample.
     */
    double time_step=coarse_step;
    for(int iteration=0;iteration<7;iteration++){
        double center=nominal.burn_ut-nominal.duration*.5;
        Candidate iteration_best;
        memset(&iteration_best,0,sizeof(iteration_best));
        trajectory_init(&iteration_best.pred.trajectory);
        bool found=false;

        for(int direction=-1;direction<=1;direction++){
            double start=center+direction*time_step;
            if(start<state.ut+lead)continue;
            Candidate c;
            if(!deorbit_evaluate_start(&c,state,start,0.0,
                    p,aero,env,cal,site,v,s,thrust))
                continue;
            if(better(&c,&iteration_best,found)){
                if(found)candidate_clear(&iteration_best);
                candidate_move(&iteration_best,&c);
                found=true;
            }else candidate_clear(&c);
        }
        if(found&&better(&iteration_best,&nominal,true)){
            candidate_clear(&nominal);
            candidate_move(&nominal,&iteration_best);
        }else if(found)candidate_clear(&iteration_best);
        time_step*=.5; /* numerical interval refinement */
    }

    Candidate selected;
    memset(&selected,0,sizeof(selected));
    trajectory_init(&selected.pred.trajectory);
    RobustAssessment selected_robust=assess_candidate_robustness(
        &nominal,state,p,aero,env,cal,site,v,s,thrust,false);
    candidate_move(&selected,&nominal);
    bool selected_any=true;

    /*
     * Robust refinement uses the configured uncertainty scales themselves as
     * the initial neighborhood.  No rescue corridor or hand-authored interior
     * gate is involved.
     */
    double robust_time_step=fmax(coarse_step,
        fmax(0.0,s->deorbit_timing_uncertainty));
    double robust_dv_step=fmax(0.0,s->deorbit_delta_v_uncertainty);
    for(int iteration=0;iteration<6;iteration++){
        double center_start=selected.burn_ut-selected.duration*.5;
        Candidate iteration_best;
        RobustAssessment iteration_robust={0};
        memset(&iteration_best,0,sizeof(iteration_best));
        trajectory_init(&iteration_best.pred.trajectory);
        bool found=false;

        for(int ti=-1;ti<=1;ti++)for(int di=-1;di<=1;di++){
            if(ti==0&&di==0)continue;
            double start=center_start+ti*robust_time_step;
            if(start<state.ut+lead)continue;

            Candidate c;
            double dv_offset=di*robust_dv_step;
            if(!deorbit_evaluate_start(&c,state,start,dv_offset,
                    p,aero,env,cal,site,v,s,thrust))
                continue;
            RobustAssessment rr=assess_candidate_robustness(
                &c,state,p,aero,env,cal,site,v,s,thrust,false);
            if(robust_better(&c,&rr,&iteration_best,&iteration_robust,found)){
                if(found)candidate_clear(&iteration_best);
                candidate_move(&iteration_best,&c);
                iteration_robust=rr;
                found=true;
            }else candidate_clear(&c);
        }

        if(found&&robust_better(&iteration_best,&iteration_robust,
                &selected,&selected_robust,selected_any)){
            candidate_clear(&selected);
            candidate_move(&selected,&iteration_best);
            selected_robust=iteration_robust;
            selected_any=true;
        }else if(found)candidate_clear(&iteration_best);

        robust_time_step*=.5; /* numerical interval refinement */
        robust_dv_step*=.5;
    }

    if(getenv("KSP_LANDER_TRACE_STRESS")){
        RobustAssessment traced=assess_candidate_robustness(
            &selected,state,p,aero,env,cal,site,v,s,thrust,true);
        (void)traced;
    }

    /* Search/ranking above intentionally stays on the reduced model.  The selected
       candidate is certified once through the executable production guidance
       policy so a self-consistent reduced-model bias cannot create a false
       capture.  Failure to produce the explicit 50 km checkpoint fails closed. */
    EntryPrediction detailed=deorbit_executable_policy_witness(
        &selected,p,aero,env,cal,site,v,s);
    bool executable_witness=detailed.shadow_guidance_used&&detailed.entered_atmosphere&&
        detailed.mm304_gate_recorded&&!detailed.shadow_aborted;
    bool nominal_capture=executable_witness&&deorbit_capture_qualified(
            &detailed,site,v,s,selected.peri,true)&&
        deorbit_energy_margin_qualified(&selected.energy_margin,s,true);
    bool recovery_capture=executable_witness&&deorbit_recovery_qualified(
            &detailed,site,v,s,selected.peri)&&
        deorbit_energy_margin_qualified(&selected.energy_margin,s,false);

    out->created_ut=state.ut;
    out->burn_ut=selected.burn_ut;
    out->delta_v=selected.dv;
    out->estimated_burn_duration=selected.duration;
    out->planning_mass=state.mass;
    out->planning_available_thrust=thrust;
    out->predicted_taem_distance=detailed.taem_distance;
    out->predicted_taem_range_error=detailed.taem_range_error;
    out->predicted_closest_distance=detailed.closest_distance;
    out->predicted_entry_range=detailed.entry_range;
    out->predicted_entry_flight_path_angle=detailed.entry_flight_path_angle;
    out->predicted_post_burn_periapsis_altitude=selected.peri;
    out->nominal_capture_achieved=nominal_capture;
    out->robustness_qualified=selected_robust.qualified&&nominal_capture;
    out->target_capture_achieved=out->robustness_qualified;
    out->robustness_scenarios=selected_robust.scenarios;
    out->robustness_passed=selected_robust.passed;
    out->robustness_unsafe=selected_robust.unsafe;
    out->robustness_pass_fraction=selected_robust.pass_fraction;
    out->recovery_passed=selected_robust.recovery_passed;
    out->recovery_pass_fraction=selected_robust.recovery_pass_fraction;
    out->execution_qualified=out->target_capture_achieved||
        (recovery_capture&&selected_robust.recovery_qualified);
    out->execution_degraded=out->execution_qualified&&
        !out->target_capture_achieved;
    out->worst_case_closest_distance=selected_robust.worst_closest;
    out->worst_case_taem_range_error=selected_robust.worst_taem_error;
    out->worst_case_entry_flight_path_angle=selected_robust.worst_entry_angle;
    out->worst_case_post_burn_periapsis_altitude=selected_robust.min_periapsis;
    out->worst_case_peak_dynamic_pressure=selected_robust.max_dynamic_pressure;
    out->worst_case_peak_g_load=selected_robust.max_g_load;

    double model_confidence=0.0;
    for(int i=0;i<4;i++)model_confidence+=env->regimes[i].confidence;
    model_confidence/=4.0;
    double evidence_confidence=fmin(model_confidence,
        fmin(cal->confidence,selected_robust.pass_fraction));
    if(selected_robust.unsafe||!nominal_capture)evidence_confidence=0.0;
    out->confidence=clampd(evidence_confidence,0.0,1.0);

    out->trajectory=detailed.trajectory;
    double selected_align_pos=deorbit_alignment_position_error(&detailed,s);
    double selected_align_course=deorbit_alignment_course_error(&detailed,site);
    snprintf(out->note,sizeof(out->note),
        "Nominal entry %.0f km at %.2f°, wings-level impact %+.1f/%+.1f km along/cross, alignment %.1f km / %.1f° at %.1f km / %.0f m/s, Pe %.1f km. Strict %u/%u (%.0f%%), recovery %u/%u (%.0f%%)%s%s",
        detailed.entry_range/1000,detailed.entry_flight_path_angle,
        selected.energy_margin.impact_along_track/1000.0,
        selected.energy_margin.impact_cross_track/1000.0,
        selected_align_pos/1000,selected_align_course,
        detailed.taem_altitude/1000,detailed.taem_speed,selected.peri/1000,
        selected_robust.passed,selected_robust.scenarios,
        selected_robust.pass_fraction*100,
        selected_robust.recovery_passed,selected_robust.scenarios,
        selected_robust.recovery_pass_fraction*100,
        selected_robust.unsafe?"; unsafe stress case detected":"",
        out->target_capture_achieved?".":out->execution_qualified?
            "; guarded recovery execution accepted.":
            "; execution acceptance not met.");

    if(selected.pred.mm304_gate_recorded&&detailed.mm304_gate_recorded){
        fprintf(stderr,
            "Deorbit 50km policy witness: reduced along/cross %+.1f/%+.1fkm V %.1f; executable %+.1f/%+.1fkm V %.1f; delta %+.1f/%+.1fkm dV %+.1f.\n",
            selected.pred.mm304_gate_along_track/1000.0,selected.pred.mm304_gate_cross_track/1000.0,
            selected.pred.mm304_gate_speed,detailed.mm304_gate_along_track/1000.0,
            detailed.mm304_gate_cross_track/1000.0,detailed.mm304_gate_speed,
            (detailed.mm304_gate_along_track-selected.pred.mm304_gate_along_track)/1000.0,
            (detailed.mm304_gate_cross_track-selected.pred.mm304_gate_cross_track)/1000.0,
            detailed.mm304_gate_speed-selected.pred.mm304_gate_speed);
    }else{
        fprintf(stderr,"Deorbit 50km policy witness incomplete: reduced=%d executable=%d shadow=%d abort=%d.\n",
            selected.pred.mm304_gate_recorded?1:0,detailed.mm304_gate_recorded?1:0,
            detailed.shadow_guidance_used?1:0,detailed.shadow_aborted?1:0);
    }

    fprintf(stderr,
        "Deorbit selected: tier %d burn %.1f dv %.1f Pe %.1fkm entry %.1fkm FPA %.2fdeg marginImpact %+.1f/%+.1fkm align %.1fkm courseDebt %.1fdeg h %.1fkm V %.0f nominal %d robust %u/%u recovery %u/%u exec %d.\n",
        selected.tier,selected.burn_ut,selected.dv,selected.peri/1000.0,
        detailed.entry_range/1000.0,detailed.entry_flight_path_angle,
        selected.energy_margin.impact_along_track/1000.0,
        selected.energy_margin.impact_cross_track/1000.0,
        selected_align_pos/1000.0,selected_align_course,
        detailed.taem_altitude/1000.0,detailed.taem_speed,
        nominal_capture?1:0,selected_robust.passed,
        selected_robust.scenarios,selected_robust.recovery_passed,
        selected_robust.scenarios,out->execution_qualified?1:0);

    candidate_clear(&selected);
    return true;
}

/* A HAC point is not a reachable Entry program. Search the complete decelerating
 * first leg and one opposite turn, including roll response and the same live
 * inlet contract. This runs exclusively on the existing prediction worker. */
static double entry_topology_energy_target_speed(const PlanetModel*p,
        const LandingConfiguration*cfg){
    if(!p||!cfg)return NAN;
    TaemSpeedEnvelope envelope=decision_taem_speed_envelope(
        &cfg->vehicle,p,cfg->guidance.taem_interface_altitude);
    if(!envelope.valid)return NAN;
    return envelope.feasible?envelope.maximum_speed_mps:
        envelope.minimum_speed_mps;
}

static double entry_topology_interface_eta(const Telemetry*t,
        const LandingConfiguration*cfg,double target_speed){
    if(!t||!cfg||!isfinite(target_speed)||!(target_speed>0.0))return NAN;
    const GuidanceSettings*s=&cfg->guidance;
    double station=-s->final_approach_distance;
    double distance=(isfinite(t->runway_along_track)&&isfinite(t->runway_cross_track))?
        hypot(station-t->runway_along_track,t->runway_cross_track):
        fmax(0.0,t->range_to_site);
    if(!(t->horizontal_speed>DBL_EPSILON))return NAN;

    double slope=s->taem_glide_slope*DEG2RAD;
    double target_horizontal=target_speed*cos(slope);
    if(!(target_horizontal>DBL_EPSILON))return NAN;

    /* Constant-deceleration first-order transit estimate. The factor 1/2 is
       the trapezoidal mean of endpoint velocities, not a tuned policy value. */
    double average_horizontal=.5*(t->horizontal_speed+target_horizontal); /* decision-literal-ok: trapezoidal mean */
    return average_horizontal>DBL_EPSILON?distance/average_horizontal:NAN;
}

static bool entry_topology_terminal_endpoint_fit(const GuidanceSettings*s,const VehicleProfile*v,
        const PlanetModel*p,double along,double cross,double course,double speed,double horizontal,
        double lift_accel,double bank_effectiveness,double bank_limit_deg,double station,
        double runway_course,double target_course,double target_speed,double final_sign,
        double*endpoint_error_out,double*radius_out){
    (void)s;(void)p;(void)speed;(void)target_speed;
    if(endpoint_error_out)*endpoint_error_out=INFINITY;
    if(radius_out)*radius_out=INFINITY;
    if(!v||!isfinite(along)||!isfinite(cross)||!isfinite(course)||
       !isfinite(horizontal)||!(horizontal>0.0)||!isfinite(lift_accel)||
       !(lift_accel>0.0)||!isfinite(bank_effectiveness)||
       !(bank_effectiveness>0.0)||!isfinite(bank_limit_deg)||
       !(bank_limit_deg>0.0)||!isfinite(target_course))return false;

    double side=final_sign>=0.0?1.0:-1.0;
    double tangent_turn=norm_signed_deg(target_course-course);
    if(side*tangent_turn<=0.0)return false;

    double usable_bank=fmin(fabs(bank_limit_deg),v->maximum_bank_angle);
    double effective_bank=fmin(usable_bank*bank_effectiveness,nextafter(90.0,0.0)); /* decision-literal-ok: tangent singularity */
    double lateral=lift_accel*fabs(sin(effective_bank*DEG2RAD));
    if(!(lateral>DBL_EPSILON))return false;
    double minimum_radius=horizontal*horizontal/lateral;

    double psi0=norm_signed_deg(course-runway_course)*DEG2RAD;
    double psi1=norm_signed_deg(target_course-runway_course)*DEG2RAD;
    double ua=(sin(psi1)-sin(psi0))/side;
    double uc=(cos(psi0)-cos(psi1))/side;
    double norm2=ua*ua+uc*uc;
    if(!(norm2>DBL_EPSILON))return false;

    double ba=station-along,bc=-cross;
    double ideal_radius=(ba*ua+bc*uc)/norm2;
    if(!(ideal_radius>0.0))return false;

    /* If the geometrically exact circle is tighter than available lateral
       authority, fly the minimum physically reachable radius and judge the
       resulting endpoint against the same MM304 handoff point contract. */
    double radius=fmax(ideal_radius,minimum_radius);
    double endpoint_along=along+radius*ua;
    double endpoint_cross=cross+radius*uc;
    double endpoint_error=hypot(endpoint_along-station,endpoint_cross);

    if(endpoint_error_out)*endpoint_error_out=endpoint_error;
    if(radius_out)*radius_out=radius;
    return endpoint_error<=1000.0; /* decision-literal-ok: explicit MM304 1 km mission contract */
}


/* Mirror live MM304's bounded actuator roll-through proof. The instantaneous
 * fixed-pose circle may still be outside its tube while the orbiter rolls onto
 * final-turn bank. Project only that short transient, then apply the same
 * endpoint-fit gate; this never certifies TAEM handoff. */
static bool entry_topology_rollthrough_endpoint_fit(const GuidanceSettings*s,
        const VehicleProfile*v,const PlanetModel*p,double along,double cross,
        double course,double speed,double horizontal,double altitude,double vertical_speed,
        double lift_accel,double drag_accel,double bank_effectiveness,double bank_limit_deg,
        double current_bank,double current_bank_rate,double current_course_rate,
        double station,double runway_course,double target_course,double target_speed,
        double final_sign,double*endpoint_error_out,double*radius_out,double*lead_time_out){
    (void)current_course_rate;
    if(endpoint_error_out)*endpoint_error_out=INFINITY;
    if(radius_out)*radius_out=INFINITY;
    if(lead_time_out)*lead_time_out=NAN;
    if(!s||!v||!p||!isfinite(along)||!isfinite(cross)||!isfinite(course)||
       !isfinite(speed)||!(speed>0.0)||!isfinite(horizontal)||!(horizontal>0.0)||
       !isfinite(altitude)||!isfinite(vertical_speed)||!isfinite(lift_accel)||
       !(lift_accel>0.0)||!isfinite(drag_accel)||drag_accel<0.0||
       !isfinite(bank_effectiveness)||!(bank_effectiveness>0.0)||
       !isfinite(current_bank)||!isfinite(bank_limit_deg))return false;

    double side=final_sign>=0.0?1.0:-1.0;
    double bank_limit=fmin(fabs(bank_limit_deg),v->maximum_bank_angle);
    if(!(bank_limit>0.0)||!(s->entry_roll_rate>0.0)||
       !(s->entry_roll_acceleration>0.0)||!(s->guidance_rate>0.0))
        return false;

    double target_bank=side*bank_limit;
    double bank_error=norm_signed_deg(current_bank-target_bank);
    double bank_rate=isfinite(current_bank_rate)?current_bank_rate:0.0;

    double lead=decision_bounded_capture_time(bank_error,bank_rate,
        s->entry_roll_acceleration,0.0);
    double rate_bound=fabs(bank_error)/s->entry_roll_rate;
    lead=fmax(lead,rate_bound);
    if(!isfinite(lead)||lead<0.0)return false;
    if(lead_time_out)*lead_time_out=lead;

    double control_dt=1.0/s->guidance_rate;
    size_t steps=lead>0.0?(size_t)ceil(lead/control_dt):1u;
    if(steps==0u)steps=1u;
    double dt=lead/(double)steps;

    double bank=current_bank;
    double tas0=speed,tas=tas0;
    double hs0=horizontal,hs=hs0;
    double rho0=planet_atmospheric_density(p,altitude);

    for(size_t i=0;i<steps;i++){
        double error=norm_signed_deg(target_bank-bank);
        double stopping_rate=sqrt(2.0*s->entry_roll_acceleration*fabs(error)); /* decision-literal-ok: double-integrator stopping law */
        double desired_rate=fabs(error)>DBL_EPSILON?
            copysign(fmin(s->entry_roll_rate,stopping_rate),error):0.0;
        bank_rate+=clampd(desired_rate-bank_rate,
            -s->entry_roll_acceleration*dt,s->entry_roll_acceleration*dt);
        bank_rate=clampd(bank_rate,-s->entry_roll_rate,s->entry_roll_rate);
        double next_bank=bank+bank_rate*dt;
        if(error*norm_signed_deg(target_bank-next_bank)<=0.0){
            next_bank=target_bank;
            bank_rate=0.0;
        }
        bank=clampd(next_bank,-v->maximum_bank_angle,v->maximum_bank_angle);

        double projected_altitude=altitude+vertical_speed*((double)i+0.5)*dt;
        double rho=planet_atmospheric_density(p,projected_altitude);
        double density_scale=rho0>DBL_EPSILON?rho/rho0:1.0;
        double speed_scale=tas0>DBL_EPSILON?(tas/tas0)*(tas/tas0):1.0;
        double force_scale=fmax(0.0,density_scale*speed_scale);

        double effective_bank=clampd(bank*bank_effectiveness,
            -nextafter(90.0,0.0),nextafter(90.0,0.0)); /* decision-literal-ok: tangent singularity */
        double lateral=lift_accel*force_scale*sin(effective_bank*DEG2RAD);
        double course_rate=lateral/fmax(hs,DBL_EPSILON)*RAD2DEG;
        double mid_course=course+course_rate*dt*0.5; /* decision-literal-ok: midpoint integration */
        double relative=norm_signed_deg(mid_course-runway_course)*DEG2RAD;
        double surface_horizontal=hs*p->radius/(p->radius+projected_altitude);
        along+=surface_horizontal*cos(relative)*dt;
        cross+=surface_horizontal*sin(relative)*dt;
        course=norm_deg(course+course_rate*dt);

        tas-=drag_accel*force_scale*dt;
        if(tas<v->minimum_safe_speed)return false;
        hs=hs0*(tas/tas0);
        if(!(hs>0.0))return false;
    }

    double final_rho=planet_atmospheric_density(p,altitude+vertical_speed*lead);
    double final_density_scale=rho0>DBL_EPSILON?final_rho/rho0:1.0;
    double final_speed_scale=(tas/tas0)*(tas/tas0);
    double final_force_scale=fmax(0.0,final_density_scale*final_speed_scale);
    return entry_topology_terminal_endpoint_fit(s,v,p,along,cross,course,tas,hs,
        lift_accel*final_force_scale,bank_effectiveness,bank_limit,station,
        runway_course,target_course,target_speed,final_sign,
        endpoint_error_out,radius_out);
}


static unsigned entry_topology_capture_failure_flags(unsigned veto){
    unsigned flags=0;
    if(veto&1u)flags|=ENTRY_TOPOLOGY_FAILURE_CAPTURE_SPEED;
    if(veto&2u)flags|=ENTRY_TOPOLOGY_FAILURE_CAPTURE_SPATIAL;
    if(veto&4u)flags|=ENTRY_TOPOLOGY_FAILURE_CAPTURE_COURSE;
    if(veto&8u)flags|=ENTRY_TOPOLOGY_FAILURE_CAPTURE_ALTITUDE;
    if(veto&16u)flags|=ENTRY_TOPOLOGY_FAILURE_CAPTURE_ENERGY;
    if(veto&32u)flags|=ENTRY_TOPOLOGY_FAILURE_CAPTURE_FPA;
    if(veto&64u)flags|=ENTRY_TOPOLOGY_FAILURE_CAPTURE_STRUCTURAL;
    return flags;
}


static EntryTopologyPlan entry_topology_trial(VehicleState initial,const Telemetry*t,const GuidanceMachine*base,
        const PlanetModel*p,const AerodynamicEnvelope*env,const TrajectoryCalibrationModel*cal,
        const LandingConfiguration*cfg,double side,double first_bank,double turn_bank,double outbound_offset,
        double first_aoa,double turn_aoa,double reversal_delay,double forced_terminal_delay){
    EntryTopologyPlan best={.cost=INFINITY};
    const VehicleProfile*v=&cfg->vehicle;const GuidanceSettings*settings=&cfg->guidance;
    /* Every topology candidate must obey the same configured bank envelope that
       live guidance will fly. This prevents a nominal solve from depending on a
       60/70 deg turn when the mission authorizes only +/-45 deg. */
    first_bank=clampd(first_bank,0.0,v->maximum_bank_angle);
    turn_bank=clampd(turn_bank,0.0,v->maximum_bank_angle);
    TaemSpeedEnvelope taem_speed_envelope=decision_taem_speed_envelope(
        v,p,settings->taem_interface_altitude);
    double taem_speed_min=taem_speed_envelope.valid?
        taem_speed_envelope.minimum_speed_mps:fmax(v->minimum_safe_speed,v->final_approach_speed);
    double taem_speed_max=taem_speed_envelope.valid?
        taem_speed_envelope.maximum_speed_mps:taem_speed_min;
    double taem_speed_nominal=base&&base->taem_interface_target.valid&&
        isfinite(base->taem_interface_target.speed)&&base->taem_interface_target.speed>0.0?
        base->taem_interface_target.speed:entry_topology_energy_target_speed(p,cfg);
    double station=-settings->final_approach_distance;
    double runway_course=norm_deg(cfg->site.runway_heading);
    double requested_course=base&&base->taem_interface_target.valid&&
        isfinite(base->taem_interface_target.course)?
            norm_deg(base->taem_interface_target.course):
            (isfinite(outbound_offset)&&fabs(outbound_offset)>DBL_EPSILON?
                norm_deg(runway_course+outbound_offset):
                norm_deg(runway_course-side*90.0));
    /* Preserve MM304's published target verbatim. Without a published target,
       search offsets select the HAC side of the perpendicular nominal inlet. */
    double requested_offset=norm_signed_deg(requested_course-runway_course);
    double target_course=base&&base->taem_interface_target.valid&&
        isfinite(base->taem_interface_target.course)?requested_course:
        norm_deg(runway_course+
            (fabs(requested_offset)>DBL_EPSILON?
                copysign(90.0,requested_offset):-side*90.0));
    double selected_outbound_offset=norm_signed_deg(target_course-runway_course);
    if(fabs(selected_outbound_offset)>DBL_EPSILON&&
       (-side)*selected_outbound_offset<=0.0)
        return best;
    VehicleState state=initial;
    GuidanceMachine reference;if(base)reference=*base;else guidance_machine_init(&reference);
    reference.diagnostic_shadow=true;
    PredictorGuidance response={0};response.actual_bank=norm_signed_deg(t->roll);
    response.actual_bank_rate=isfinite(t->roll_rate)?t->roll_rate:0.0;
    response.actual_aoa=t->angle_of_attack;
    response.actual_aoa_rate=t->has_angle_of_attack_rate?t->angle_of_attack_rate:0.0;
    AeroContext ctx={.p=p,.e=env,.c=cal,.v=v,.bank=response.actual_bank,
        .aoa=response.actual_aoa,.mass=initial.mass,.ut=initial.ut};
    double reversal_ut=NAN,reversal_along=NAN,reversal_cross=NAN,reversal_altitude=NAN,reversal_speed=NAN,reversal_course=NAN;
    double terminal_turn_ut=NAN,terminal_turn_along=NAN,terminal_turn_cross=NAN;
    double terminal_turn_altitude=NAN,terminal_turn_speed=NAN,terminal_turn_course=NAN;
    double terminal_turn_radius=INFINITY,terminal_turn_geometry_error=INFINITY;
    double gravity0=planet_surface_gravity(p),peak_q=0,peak_g=0;
    bool reversal_recorded=false,terminal_turn_started=false,heading_locked=false;
    unsigned propagation_failure_flags=ENTRY_TOPOLOGY_FAILURE_NONE;
    double heading_lock_ut=NAN,heading_lock_along=NAN,heading_lock_cross=NAN;
    double heading_lock_altitude=NAN,heading_lock_speed=NAN,heading_lock_course=NAN,heading_lock_bank=NAN;
    /* Preserve the closest terminal-turn setup even when a coarse trial does not
       enter the capture tube.  Without a finite near-miss seed, coarse reversal
       timing errors are discarded as INFINITY and the refinement stage has nothing
       to optimize around.  These snapshots are never installable by themselves. */
    double setup_seed_metric=INFINITY,setup_seed_ut=NAN,setup_seed_along=NAN,setup_seed_cross=NAN;
    double setup_seed_altitude=NAN,setup_seed_speed=NAN,setup_seed_course=NAN;
    double setup_seed_radius=INFINITY,setup_seed_geom=INFINITY;
    const double dt=.5; /* numerical integration step */

    GeoPoint runway={cfg->site.latitude,cfg->site.longitude,cfg->site.altitude};
    double rh=cfg->site.runway_heading*DEG2RAD;
    TaemInterfaceTarget lock_target=base&&base->taem_interface_target.valid?
        base->taem_interface_target:(TaemInterfaceTarget){0};
    lock_target.valid=true;lock_target.along_track=station;lock_target.cross_track=0.0;
    lock_target.course=target_course;
    if(!isfinite(lock_target.response_time)||lock_target.response_time<=0.0){
        double full_bank=fabs(v->maximum_bank_angle);
        double rate_time=settings->entry_roll_rate>DBL_MIN?
            full_bank/settings->entry_roll_rate:0.0;
        double accel_time=settings->entry_roll_acceleration>DBL_MIN?
            sqrt(2.0*full_bank/settings->entry_roll_acceleration):0.0;
        lock_target.response_time=fmax(rate_time,accel_time);
    }
    if(!isfinite(lock_target.acquisition_lead)||lock_target.acquisition_lead<=0.0)
        lock_target.acquisition_lead=
            fmax(0.0,t->horizontal_speed)*lock_target.response_time;

    double trial_eta=entry_topology_interface_eta(t,cfg,taem_speed_nominal);
    if(!isfinite(trial_eta)){
        if(!(t->horizontal_speed>DBL_EPSILON))return best;
        trial_eta=fmax(0.0,t->range_to_site)/t->horizontal_speed;
    }
    double latest_event=fmax(reversal_delay,
        isfinite(forced_terminal_delay)?forced_terminal_delay:0.0);
    double trial_horizon=fmax(trial_eta,latest_event)+
        lock_target.response_time+
        fmax(0.0,settings->s_turn_minimum_leg_duration);

    for(double elapsed=0;elapsed<trial_horizon;elapsed+=dt){
        AtmosState a;Telemetry synthetic=guidance_shadow_telemetry(t,state,&response,&ctx,p,cfg,&a);
        GeoPoint geo={synthetic.latitude,synthetic.longitude,synthetic.mean_altitude};
        double along,cross;runway_coordinates(geo,runway,cfg->site.runway_heading,p->radius,&along,&cross);
        double course=surface_course(state.position,state.velocity,planet_rotation_vector(p),p->north_axis,t->ground_track_heading);
        double target_course_error=norm_signed_deg(target_course-course);
        double requested_bank=side*first_bank;
        if(!reversal_recorded&&elapsed>=reversal_delay){
            /* The scheduled first sign change is the energy-management reversal,
               not automatically the final perpendicular turn.  Live MM304 executes
               this committed deadline even when terminal geometry is not ready, then
               carries the acquired crossrange on the opposite/runway-parallel side
               until the fixed-pose arc becomes flyable.  Preserve the same two-event
               topology in the shadow so a valid plan can publish distinct reversal
               and terminal-turn milestones. */
            reversal_recorded=true;
            reversal_ut=state.ut;
            reversal_along=along;reversal_cross=cross;reversal_altitude=geo.altitude;
            reversal_speed=a.speed;reversal_course=course;
        }
        if(reversal_recorded&&!terminal_turn_started){
            double final_sign=-side;
            double horizontal=fmax(80.0,synthetic.horizontal_speed);
            double limit=fmin(turn_bank,bank_limit(&a,p,v,v->maximum_bank_angle));
            double endpoint_error=INFINITY,endpoint_radius=INFINITY;
            bool endpoint_ready=entry_topology_terminal_endpoint_fit(settings,v,p,
                along,cross,course,a.speed,horizontal,a.lift_accel,cal->bank_effectiveness,
                limit,station,runway_course,target_course,taem_speed_nominal,final_sign,
                &endpoint_error,&endpoint_radius);
            double setup_endpoint_error=endpoint_error,setup_endpoint_radius=endpoint_radius;

            /* Live MM304 can release the later final event on a bounded actuator
               roll-through proof. Mirror that proof here so preflight does not reject
               a path merely because the instantaneous circle becomes admissible a few
               seconds after roll begins. The strict capture contract is unchanged. */
            if(!endpoint_ready&&isfinite(endpoint_error)){
                double rollout_error=INFINITY,rollout_radius=INFINITY,rollout_lead=NAN;
                bool rollout_ready=entry_topology_rollthrough_endpoint_fit(settings,v,p,
                    along,cross,course,a.speed,horizontal,geo.altitude,synthetic.vertical_speed,
                    a.lift_accel,a.drag_accel,cal->bank_effectiveness,limit,
                    response.actual_bank,response.actual_bank_rate,synthetic.course_rate,
                    station,runway_course,target_course,taem_speed_nominal,final_sign,
                    &rollout_error,&rollout_radius,&rollout_lead);
                if(isfinite(rollout_error)&&rollout_error<setup_endpoint_error){
                    setup_endpoint_error=rollout_error;setup_endpoint_radius=rollout_radius;
                }
                if(rollout_ready){
                    endpoint_ready=true;endpoint_error=rollout_error;endpoint_radius=rollout_radius;
                }
            }

            /* Keep the best usable instantaneous or projected roll-through setup as
               a refinement seed after the first shaping reversal.  A projected near
               miss is still not executable: only endpoint_ready may start the final turn. */
            if(isfinite(setup_endpoint_error)&&
               geo.altitude>cfg->site.altitude&&
               a.speed>=v->minimum_safe_speed){
                double setup_metric=setup_endpoint_error;
                if(setup_metric<setup_seed_metric){
                    setup_seed_metric=setup_metric;setup_seed_ut=state.ut;
                    setup_seed_along=along;setup_seed_cross=cross;
                    setup_seed_altitude=geo.altitude;
                    setup_seed_speed=a.speed;setup_seed_course=course;
                    setup_seed_radius=setup_endpoint_radius;
                    setup_seed_geom=setup_endpoint_error;
                }
            }

            /* Mirror entry_topology_guidance's post-shaping parallel stage.  The
               first reversal brings course back toward the runway heading and may
               then unload while the vehicle advances downstream.  The later final
               turn stays on this same side and begins only when the fixed-pose
               endpoint gate is actually satisfied. */
            double efficiency=fmax(cal->bank_effectiveness,DBL_MIN);
            requested_bank=taem_course_rate_bank_command(
                horizontal,a.lift_accel,efficiency,limit,
                course,runway_course,lock_target.response_time);

            /* Search-only dynamic timing candidates may deliberately begin the
               second/final event before the frozen-current-radius endpoint tube is
               satisfied.  This does not certify anything: the remainder of this same
               high-fidelity shadow must still fly the turn, settle heading, and pass
               the ordinary strict TAEM/HAC capture contract.  The first shaping
               reversal remains mandatory and distinct. */
            bool forced_terminal_due=isfinite(forced_terminal_delay)&&
                forced_terminal_delay>=reversal_delay&&elapsed>=forced_terminal_delay;
            if(endpoint_ready||forced_terminal_due){
                terminal_turn_started=true;terminal_turn_ut=state.ut;
                terminal_turn_along=along;terminal_turn_cross=cross;
                terminal_turn_altitude=geo.altitude;terminal_turn_speed=a.speed;
                terminal_turn_course=course;terminal_turn_radius=endpoint_radius;
                terminal_turn_geometry_error=endpoint_error;
            }
        }
        if(terminal_turn_started&&!heading_locked&&fabs(target_course_error)<=8.0){
            heading_locked=true;
            heading_lock_ut=state.ut;heading_lock_along=along;heading_lock_cross=cross;
            heading_lock_altitude=geo.altitude;heading_lock_speed=a.speed;heading_lock_course=course;
            heading_lock_bank=ctx.bank;
        }

        if(terminal_turn_started&&!heading_locked){
            /* Propagate the same dynamic fixed-pose circle used by live MM304.
               Recompute the required radius as speed/course evolve, then invert
               current lift for the bank that preserves that radius. */
            double limit=fmin(turn_bank,bank_limit(&a,p,v,v->maximum_bank_angle));
            double pose_error=INFINITY,pose_radius=INFINITY;
            (void)entry_topology_terminal_endpoint_fit(settings,v,p,
                along,cross,course,a.speed,fmax(80.0,synthetic.horizontal_speed),
                a.lift_accel,cal->bank_effectiveness,limit,station,runway_course,
                target_course,taem_speed_nominal,-side,&pose_error,&pose_radius);
            double efficiency=clampd(cal->bank_effectiveness,.35,1.8);
            double horizontal=fmax(80.0,synthetic.horizontal_speed);
            double bank_mag=0.0;
            if(isfinite(pose_radius)&&pose_radius>=1000.0&&a.lift_accel>.01){
                double lateral=horizontal*horizontal/pose_radius;
                double max_ratio=sin(clampd(limit*efficiency,0.0,89.0)*DEG2RAD);
                bank_mag=asin(clampd(lateral/a.lift_accel,0.0,max_ratio))/efficiency*RAD2DEG;
            }else{
                bank_mag=fabs(taem_course_rate_bank_command(horizontal,a.lift_accel,
                    efficiency,limit,course,target_course,lock_target.response_time));
            }
            requested_bank=-side*clampd(bank_mag,0.0,limit);
            double lateral=a.lift_accel*sin(clampd(fabs(requested_bank)*efficiency,0.0,89.0)*DEG2RAD);
            double radius=lateral>.05?horizontal*horizontal/lateral:INFINITY;
            if(!isfinite(terminal_turn_radius)&&isfinite(radius))terminal_turn_radius=radius;
        }else if(terminal_turn_started){
            double desired_heading=taem_interface_line_heading(along,cross,cfg->site.runway_heading,
                &lock_target);
            double correction_limit=fmin(18.0,bank_limit(&a,p,v,v->maximum_bank_angle));
            requested_bank=taem_course_rate_bank_command(synthetic.horizontal_speed,a.lift_accel,
                cal->bank_effectiveness,correction_limit,course,desired_heading,
                lock_target.response_time);
        }
        double terminal_floor=entry_terminal_turn_aoa_floor(a.speed,a.q,v);
        /* Match entry_topology_guidance(): the high-drag final-S-turn AoA
           is a low-speed regime, not something that begins at the first shaping
           reversal hundreds of kilometres upstream. */
        double requested_aoa=fmax(a.speed>1100.0?first_aoa:turn_aoa,terminal_floor);
        /* Match the executable MM304 energy-preservation policy. The topology search
           used to propagate 18-26 deg incidence all the way through the terminal
           geometry, so every otherwise-good -8 km capture arrived hundreds of m/s
           slow even though live guidance unloads once thermal/q authority permits. */
        bool preserve_terminal_energy=reversal_recorded&&
            a.speed<settings->taem_force_handoff_speed+350.0&&
            a.q>=700.0&&a.q<v->maximum_dynamic_pressure*.85&&
            a.non_gravity/fmax(.1,gravity0)<v->maximum_g_load*.85;
        double extended_aoa_speed_floor=fmax(taem_speed_max*1.08,taem_speed_nominal*1.35);
        bool extended_final_turn=reversal_recorded&&turn_aoa>v->maximum_angle_of_attack+1e-6&&
            !heading_locked&&a.speed>extended_aoa_speed_floor&&
            a.q<v->maximum_dynamic_pressure*.88&&
            a.non_gravity/fmax(.1,gravity0)<v->maximum_g_load*.90;
        if(preserve_terminal_energy&&!extended_final_turn)
            requested_aoa=fmin(requested_aoa,fmax(15.0,terminal_floor));
        requested_aoa=clampd(requested_aoa,0.0,extended_final_turn?
            entry_final_s_turn_aoa_ceiling(v):v->maximum_angle_of_attack);

        for(unsigned step=0;step<5;step++){
            synthetic.roll=response.actual_bank;synthetic.roll_rate=response.actual_bank_rate;
            synthetic.angle_of_attack=response.actual_aoa;synthetic.pitch=synthetic.flight_path_angle+response.actual_aoa;
            GuidanceCommand command=guidance_entry_reference_step(&reference,&synthetic,v,settings,
                requested_bank,requested_aoa,dt/5.0);
            attitude_step(&response,norm_signed_deg(command.target_roll),command.target_aoa,&a,v,settings,dt/5.0,cal->physics);
        }
        ctx.bank=response.actual_bank;ctx.aoa=response.actual_aoa;ctx.ut=state.ut;
        peak_q=fmax(peak_q,a.q);peak_g=fmax(peak_g,a.non_gravity/fmax(.1,gravity0));
        propagation_failure_flags=ENTRY_TOPOLOGY_FAILURE_NONE;
        if(!isfinite(geo.altitude)||!isfinite(a.speed))
            propagation_failure_flags|=ENTRY_TOPOLOGY_FAILURE_NONFINITE_STATE;
        if(isfinite(geo.altitude)&&geo.altitude<cfg->site.altitude)
            propagation_failure_flags|=ENTRY_TOPOLOGY_FAILURE_ALTITUDE_FLOOR;
        if(isfinite(a.speed)&&a.speed<v->minimum_safe_speed)
            propagation_failure_flags|=ENTRY_TOPOLOGY_FAILURE_SPEED_FLOOR;
        if(peak_q>v->maximum_dynamic_pressure)
            propagation_failure_flags|=ENTRY_TOPOLOGY_FAILURE_DYNAMIC_PRESSURE;
        if(peak_g>v->maximum_g_load)
            propagation_failure_flags|=ENTRY_TOPOLOGY_FAILURE_G_LOAD;
        if(propagation_failure_flags!=ENTRY_TOPOLOGY_FAILURE_NONE)break;

        double position_error=hypot(along-station,cross);
        bool candidate_window=heading_locked&&terminal_turn_started&&
            state.ut>=terminal_turn_ut;
        if(candidate_window){
            Vector3 up=vnorm(state.position,v3(0,0,1));
            double vertical=vdot(a.air_velocity,up);
            double horizontal=sqrt(fmax(0.0,a.speed*a.speed-vertical*vertical));
            double fpa=asin(clampd(vertical/fmax(1.0,a.speed),-1,1))*RAD2DEG;

            double usable_bank=bank_limit(&a,p,v,v->maximum_bank_angle);
            double effective_bank=clampd(
                usable_bank*cal->bank_effectiveness,
                -nextafter(90.0,0.0),nextafter(90.0,0.0));
            double lateral=a.lift_accel*fabs(sin(effective_bank*DEG2RAD));
            double minimum_turn_radius=lateral>DBL_MIN?
                horizontal*horizontal/lateral:INFINITY;
            double endpoint_error=position_error;
            double heading_debt=isfinite(minimum_turn_radius)?
                minimum_turn_radius*fabs(target_course_error)*DEG2RAD:
                INFINITY;

            /*
             * Radius is determined by demonstrated turn authority; configured
             * HAC radius is only a preferred geometry.  Feasibility is proved
             * by the same TAEM alignment model used by the live handoff.
             */
            double terminal_radius=isfinite(minimum_turn_radius)?
                fmax(settings->hac_radius,minimum_turn_radius):INFINITY;
            double maneuver_path=0.0,maneuver_slope=0.0;
            bool maneuver_ok=isfinite(terminal_radius)&&
                taem_alignment_maneuver_feasible(
                    geo.altitude,a.speed,terminal_radius,
                    minimum_turn_radius,a.drag_accel,
                    fabs(selected_outbound_offset),p,&cfg->site,v,settings,
                    &maneuver_path,&maneuver_slope);

            double final_altitude=cfg->site.altitude+
                settings->final_approach_distance*
                tan(settings->final_glide_slope*DEG2RAD);
            double terminal_path=settings->final_approach_distance+
                (maneuver_ok?maneuver_path:0.0);
            double minimum_alignment_altitude=final_altitude+
                fmax(0.0,terminal_path-settings->final_approach_distance)*
                tan(settings->taem_glide_slope*DEG2RAD);
            double kinetic_height=fmax(0.0,
                a.speed*a.speed-v->final_approach_speed*
                v->final_approach_speed)/(2.0*gravity0);
            double reserve=geo.altitude+kinetic_height-
                minimum_alignment_altitude;

            double target_bank=-side*turn_bank;
            double bank_error=norm_signed_deg(target_bank-response.actual_bank);
            double response_time=decision_bounded_capture_time(
                bank_error,response.actual_bank_rate,
                settings->entry_roll_acceleration,0.0);
            if(settings->entry_roll_rate>DBL_MIN)
                response_time=fmax(response_time,
                    fabs(bank_error)/settings->entry_roll_rate);
            if(!isfinite(response_time)||response_time<0.0)
                response_time=0.0;
            double acquisition_lead=horizontal*response_time;
            double remaining_path=maneuver_ok?maneuver_path:
                settings->final_approach_distance;

            GeoPoint alignment=local_point(runway,
                station*sin(rh),station*cos(rh),p->radius,geo.altitude);
            TaemInterfaceTarget inlet={
                .valid=true,.along_track=station,.cross_track=0.0,
                .course=target_course,.altitude=geo.altitude,.speed=a.speed,
                .flight_path_angle=fpa,
                .specific_energy=rotating_specific_energy(
                    alignment.latitude,geo.altitude,a.speed,p),
                .selected_ut=initial.ut,.arrival_ut=state.ut,
                .quality_score=INFINITY,
                .acquisition_lead=acquisition_lead,
                .remaining_path=remaining_path,
                .response_time=response_time
            };

            Telemetry capture_state=synthetic;
            capture_state.latitude=geo.latitude;
            capture_state.longitude=geo.longitude;
            capture_state.mean_altitude=geo.altitude;
            capture_state.range_to_site=hypot(along,cross);
            capture_state.runway_along_track=along;
            capture_state.runway_cross_track=cross;
            capture_state.true_air_speed=a.speed;
            capture_state.horizontal_speed=horizontal;
            capture_state.vertical_speed=vertical;
            capture_state.flight_path_angle=fpa;
            capture_state.dynamic_pressure=a.q;
            capture_state.g_force=a.non_gravity/gravity0;
            capture_state.stall_fraction_is_measured=false;
            capture_state.bank_effectiveness=cal->bank_effectiveness;

            TaemInterfaceCapture capture=entry_taem_interface_capture(
                &inlet,&capture_state,course,p,cfg);

            unsigned failure_flags=ENTRY_TOPOLOGY_FAILURE_NONE;
            TaemHandoffContract handoff_contract=
                taem_handoff_contract(&cfg->guidance);
            if(position_error>handoff_contract.horizontal_radius_m)
                failure_flags|=ENTRY_TOPOLOGY_FAILURE_POSITION;
            if(fabs(target_course_error)>
               handoff_contract.perpendicular_heading_half_width_deg)
                failure_flags|=ENTRY_TOPOLOGY_FAILURE_COURSE;
            if(!maneuver_ok)
                failure_flags|=ENTRY_TOPOLOGY_FAILURE_MANEUVER_PATH;
            failure_flags|=entry_topology_capture_failure_flags(capture.veto);
            bool valid=capture.ready&&
                failure_flags==ENTRY_TOPOLOGY_FAILURE_NONE;

            /*
             * Cost is telemetry only.  Keep it dimensionless as the worst
             * normalized mission-contract residual; candidate ordering below is
             * lexicographic and never sums these terms.
             */
            double position_residual=position_error/
                handoff_contract.horizontal_radius_m;
            double course_residual=fabs(target_course_error)/
                handoff_contract.perpendicular_heading_half_width_deg;
            double energy_scale=fmax(
                planet_surface_gravity(p)*fmax(geo.altitude-cfg->site.altitude,0.0),
                .5*fmax(a.speed*a.speed,DBL_MIN));
            double energy_residual=fmax(0.0,-capture.energy_margin)/
                fmax(energy_scale,DBL_MIN);
            double turn_residual=fmax(0.0,-capture.turn_margin)/
                fmax(energy_scale,DBL_MIN);
            double residual=fmax(position_residual,
                fmax(course_residual,
                    fmax(energy_residual,turn_residual)));
            inlet.quality_score=residual;

            bool better=false;
            if(!isfinite(best.cost))better=true;
            else if(valid!=best.valid)better=valid;
            else{
                unsigned trial_veto=0,best_veto=0;
                for(unsigned bits=capture.veto;bits;bits>>=1)
                    trial_veto+=bits&1u;
                for(unsigned bits=best.capture_veto;bits;bits>>=1)
                    best_veto+=bits&1u;
                if(trial_veto!=best_veto)better=trial_veto<best_veto;
                else if(position_error!=best.position_error)
                    better=position_error<best.position_error;
                else if(fabs(target_course_error)!=fabs(best.capture_course_error))
                    better=fabs(target_course_error)<
                        fabs(best.capture_course_error);
                else if(fmax(0.0,-capture.energy_margin)!=
                        fmax(0.0,-best.terminal_energy_margin))
                    better=fmax(0.0,-capture.energy_margin)<
                        fmax(0.0,-best.terminal_energy_margin);
                else
                    better=state.ut<best.capture_ut;
            }
            if(better){
                best=(EntryTopologyPlan){
                    .valid=valid,.failure_flags=failure_flags,.planned_ut=initial.ut,
                    .first_sign=side,.first_bank=first_bank,.turn_bank=turn_bank,
                    .outbound_course_offset=selected_outbound_offset,.capture_veto=capture.veto,.capture_turn_margin=capture.turn_margin,
                    .first_aoa=first_aoa,.turn_aoa=turn_aoa,.aoa_switch_speed=1100.0,
                    .reversal_ut=reversal_ut,.terminal_turn_ut=terminal_turn_ut,.capture_ut=state.ut,
                    .reversal_along=reversal_along,.reversal_cross=reversal_cross,.reversal_altitude=reversal_altitude,
                    .reversal_speed=reversal_speed,.reversal_course=reversal_course,
                    .reversal_turn_radius=terminal_turn_radius,.reversal_geometry_error=terminal_turn_geometry_error,
                    .terminal_turn_along=terminal_turn_along,.terminal_turn_cross=terminal_turn_cross,
                    .terminal_turn_altitude=terminal_turn_altitude,.terminal_turn_speed=terminal_turn_speed,
                    .terminal_turn_course=terminal_turn_course,.terminal_turn_radius=terminal_turn_radius,
                    .terminal_turn_geometry_error=terminal_turn_geometry_error,
                    .capture_along=along,.capture_cross=cross,.capture_altitude=geo.altitude,.capture_speed=a.speed,
                    .capture_course=course,.capture_course_error=target_course_error,.capture_bank=ctx.bank,
                    .capture_heading_debt=heading_debt,.capture_endpoint_error=endpoint_error,
                    .heading_lock_ut=heading_lock_ut,.heading_lock_along=heading_lock_along,.heading_lock_cross=heading_lock_cross,
                    .heading_lock_altitude=heading_lock_altitude,.heading_lock_speed=heading_lock_speed,
                    .heading_lock_course=heading_lock_course,.heading_lock_bank=heading_lock_bank,
                    .position_error=position_error,.glide_reserve=reserve,.terminal_energy_margin=capture.energy_margin,
                    .minimum_alignment_altitude=minimum_alignment_altitude,.cost=residual,.inlet=inlet};
            }
        }
        state=rk4_aero(state,dt,&ctx);
    }
    if(!isfinite(best.cost)&&isfinite(setup_seed_metric)){
        double setup_endpoint=hypot(setup_seed_along-station,setup_seed_cross);
        double setup_course_error=norm_signed_deg(target_course-setup_seed_course);
        unsigned failure_flags=propagation_failure_flags;
        if(!reversal_recorded)failure_flags|=ENTRY_TOPOLOGY_FAILURE_REVERSAL_SETUP;
        if(!terminal_turn_started)failure_flags|=ENTRY_TOPOLOGY_FAILURE_TERMINAL_TURN;
        else if(!heading_locked)failure_flags|=ENTRY_TOPOLOGY_FAILURE_HEADING_LOCK;
        if(setup_endpoint>1000.0)failure_flags|=ENTRY_TOPOLOGY_FAILURE_POSITION;
        if(fabs(setup_course_error)>30.0)failure_flags|=ENTRY_TOPOLOGY_FAILURE_COURSE;
        failure_flags|=entry_topology_capture_failure_flags(2u|4u|16u);
        best=(EntryTopologyPlan){
            .valid=false,.failure_flags=failure_flags,.planned_ut=initial.ut,
            .first_sign=side,.first_bank=first_bank,.turn_bank=turn_bank,
            .outbound_course_offset=selected_outbound_offset,
            .first_aoa=first_aoa,.turn_aoa=turn_aoa,.aoa_switch_speed=1100.0,
            .reversal_ut=isfinite(reversal_ut)?reversal_ut:setup_seed_ut,
            .reversal_along=reversal_along,.reversal_cross=reversal_cross,.reversal_altitude=reversal_altitude,
            .reversal_speed=reversal_speed,.reversal_course=reversal_course,
            .reversal_turn_radius=setup_seed_radius,.reversal_geometry_error=setup_seed_geom,
            .terminal_turn_ut=setup_seed_ut,.terminal_turn_along=setup_seed_along,.terminal_turn_cross=setup_seed_cross,
            .terminal_turn_altitude=setup_seed_altitude,.terminal_turn_speed=setup_seed_speed,
            .terminal_turn_course=setup_seed_course,.terminal_turn_radius=setup_seed_radius,
            .terminal_turn_geometry_error=setup_seed_geom,
            .capture_ut=setup_seed_ut,.capture_along=setup_seed_along,.capture_cross=setup_seed_cross,
            .capture_altitude=setup_seed_altitude,.capture_speed=setup_seed_speed,.capture_course=setup_seed_course,
            .capture_course_error=setup_course_error,.capture_bank=0.0,
            .capture_heading_debt=isfinite(setup_seed_radius)?
                setup_seed_radius*fabs(setup_course_error)*DEG2RAD:INFINITY,
            .capture_endpoint_error=setup_endpoint,.position_error=setup_endpoint,
            .capture_veto=2u|4u|16u,.minimum_alignment_altitude=NAN,.terminal_energy_margin=NAN,
            .glide_reserve=-INFINITY,.cost=setup_seed_metric};
    }
    return best;
}

static unsigned predictor_popcount_unsigned(unsigned value){
    unsigned count=0u;
    while(value){count+=value&1u;value>>=1;}
    return count;
}

static bool entry_topology_non_geometric_qualified(
        const EntryTopologyPlan*p){
    if(!p||!isfinite(p->cost))return false;
    unsigned blocking=ENTRY_TOPOLOGY_FAILURE_SPEED_PATH|
        ENTRY_TOPOLOGY_FAILURE_MANEUVER_PATH|
        ENTRY_TOPOLOGY_FAILURE_CAPTURE_SPEED|
        ENTRY_TOPOLOGY_FAILURE_CAPTURE_ALTITUDE|
        ENTRY_TOPOLOGY_FAILURE_CAPTURE_ENERGY|
        ENTRY_TOPOLOGY_FAILURE_CAPTURE_STRUCTURAL;
    return (p->failure_flags&blocking)==0u;
}

static unsigned entry_topology_progress_rank(
        const EntryTopologyPlan*p){
    if(!p||!isfinite(p->cost))return 0u;
    if(p->failure_flags&ENTRY_TOPOLOGY_FAILURE_REVERSAL_SETUP)return 1u;
    if(p->failure_flags&ENTRY_TOPOLOGY_FAILURE_TERMINAL_TURN)return 2u;
    if(p->failure_flags&ENTRY_TOPOLOGY_FAILURE_HEADING_LOCK)return 3u;
    return p->valid?5u:4u;
}

static bool entry_topology_search_better(
        const EntryTopologyPlan*trial,const EntryTopologyPlan*best){
    if(!trial||!isfinite(trial->cost))return false;
    if(!best||!isfinite(best->cost))return true;
    if(trial->valid!=best->valid)return trial->valid;

    unsigned tp=entry_topology_progress_rank(trial);
    unsigned bp=entry_topology_progress_rank(best);
    if(tp!=bp)return tp>bp;

    bool tq=entry_topology_non_geometric_qualified(trial);
    bool bq=entry_topology_non_geometric_qualified(best);
    if(tq!=bq)return tq;

    unsigned tf=predictor_popcount_unsigned(trial->failure_flags);
    unsigned bf=predictor_popcount_unsigned(best->failure_flags);
    if(tf!=bf)return tf<bf;

    unsigned tv=predictor_popcount_unsigned(trial->capture_veto);
    unsigned bv=predictor_popcount_unsigned(best->capture_veto);
    if(tv!=bv)return tv<bv;

    if(isfinite(trial->position_error)&&isfinite(best->position_error)&&
       trial->position_error!=best->position_error)
        return trial->position_error<best->position_error;

    double tc=fabs(trial->capture_course_error);
    double bc=fabs(best->capture_course_error);
    if(isfinite(tc)&&isfinite(bc)&&tc!=bc)return tc<bc;

    double te=fmax(0.0,-trial->terminal_energy_margin);
    double be=fmax(0.0,-best->terminal_energy_margin);
    if(isfinite(te)&&isfinite(be)&&te!=be)return te<be;

    double tt=fmax(0.0,-trial->capture_turn_margin);
    double bt=fmax(0.0,-best->capture_turn_margin);
    if(isfinite(tt)&&isfinite(bt)&&tt!=bt)return tt<bt;

    return trial->capture_ut<best->capture_ut;
}

EntryTopologyPlan predictor_plan_entry_topology(VehicleState state,
        const Telemetry*t,const GuidanceMachine*base,const PlanetModel*p,
        const AerodynamicEnvelope*env,const TrajectoryCalibrationModel*cal,
        const LandingConfiguration*cfg){
    EntryTopologyPlan best={.cost=INFINITY};
    if(!t||!p||!env||!cal||!cfg||!isfinite(state.ut)||
       !(t->true_air_speed>=cfg->vehicle.minimum_safe_speed)||
       !(t->horizontal_speed>DBL_EPSILON))
        return best;

    double energy_target_speed=
        base&&base->taem_interface_target.valid&&
        isfinite(base->taem_interface_target.speed)&&
        base->taem_interface_target.speed>0.0?
            base->taem_interface_target.speed:
            entry_topology_energy_target_speed(p,cfg);
    double eta=entry_topology_interface_eta(
        t,cfg,energy_target_speed);
    if(!isfinite(eta))
        eta=fmax(0.0,t->range_to_site)/t->horizontal_speed;

    double max_bank=cfg->vehicle.maximum_bank_angle;
    if(!(max_bank>0.0))return best;
    double alpha_floor=entry_thermal_protection_aoa_floor(
        &cfg->vehicle);
    double alpha_ceiling=entry_final_s_turn_aoa_ceiling(
        &cfg->vehicle);
    alpha_ceiling=fmin(alpha_ceiling,
        cfg->vehicle.maximum_angle_of_attack);
    if(alpha_ceiling<alpha_floor)alpha_ceiling=alpha_floor;
    const int bank_samples=5;   /* numerical control-space resolution */
    const int aoa_samples=4;
    const int delay_samples=11;

    double earliest_delay=fmax(
        0.0,cfg->guidance.s_turn_minimum_leg_duration);
    double latest_delay=fmax(earliest_delay,eta);

    for(int side=-1;side<=1;side+=2)
    for(int bi=0;bi<bank_samples;bi++)
    for(int ti=0;ti<bank_samples;ti++)
    for(int ai=0;ai<aoa_samples;ai++)
    for(int di=0;di<delay_samples;di++){
        double first_bank=max_bank*
            (double)(bi+1)/(double)bank_samples;
        double turn_bank=max_bank*
            (double)(ti+1)/(double)bank_samples;
        double turn_aoa=alpha_floor+
            (alpha_ceiling-alpha_floor)*
            (double)ai/(double)(aoa_samples-1);
        double delay=earliest_delay+
            (latest_delay-earliest_delay)*
            (double)di/(double)(delay_samples-1);

        EntryTopologyPlan trial=entry_topology_trial(
            state,t,base,p,env,cal,cfg,side,
            first_bank,turn_bank,0.0,
            alpha_floor,turn_aoa,delay,NAN);
        if(getenv("KSP_LANDER_TRACE_TOPOLOGY_GRID")&&side>0.0&&bi==0&&ai==0){
            fprintf(stderr,
                "Entry topology grid: turnBank=%.1f turnAoA=%.1f delay=%.1f valid=%d flags=0x%x along=%+.1f cross=%+.1f pos=%.1f h=%.1f V=%.1f course=%.2f geom=%.1f veto=%u cost=%.1f\n",
                turn_bank,turn_aoa,delay,trial.valid,trial.failure_flags,
                trial.capture_along,trial.capture_cross,trial.position_error,
                trial.capture_altitude,trial.capture_speed,trial.capture_course,
                trial.terminal_turn_geometry_error,trial.capture_veto,trial.cost);
        }
        if(entry_topology_search_better(&trial,&best))
            best=trial;
    }

    /*
     * Refine the continuous controls around the current best by halving the
     * search interval.  Step sizes come from the original sampled domain;
     * there are no hand-authored degree/second nudges.
     */
    if(isfinite(best.cost)){
        double bank_step=max_bank/(double)bank_samples;
        double aoa_step=(alpha_ceiling-alpha_floor)/
            fmax((double)(aoa_samples-1),1.0);
        double delay_step=(latest_delay-earliest_delay)/
            fmax((double)(delay_samples-1),1.0);

        for(int iteration=0;iteration<6;iteration++){
            EntryTopologyPlan seed=best;
            for(int db=-1;db<=1;db++)
            for(int dtb=-1;dtb<=1;dtb++)
            for(int da=-1;da<=1;da++)
            for(int dd=-1;dd<=1;dd++){
                double first_bank=clampd(
                    seed.first_bank+db*bank_step,
                    DBL_EPSILON,max_bank);
                double turn_bank=clampd(
                    seed.turn_bank+dtb*bank_step,
                    DBL_EPSILON,max_bank);
                double turn_aoa=clampd(
                    seed.turn_aoa+da*aoa_step,
                    alpha_floor,alpha_ceiling);
                double delay=clampd(
                    seed.reversal_ut-state.ut+dd*delay_step,
                    earliest_delay,latest_delay);

                EntryTopologyPlan trial=entry_topology_trial(
                    state,t,base,p,env,cal,cfg,seed.first_sign,
                    first_bank,turn_bank,0.0,
                    seed.first_aoa,turn_aoa,delay,NAN);
                if(entry_topology_search_better(&trial,&best))
                    best=trial;
            }
            bank_step*=.5;
            aoa_step*=.5;
            delay_step*=.5;
        }
    }

    /*
     * If the physically propagated trial did not autonomously detect the final
     * turn, refine that event time directly between the shaping reversal and
     * the predicted MM304 interface ETA.  The forced event is only a search
     * coordinate; the propagated strict capture contract still decides validity.
     */
    if(isfinite(best.cost)&&!best.valid&&isfinite(best.reversal_ut)){
        double reversal_delay=best.reversal_ut-state.ut;
        double lo=fmax(reversal_delay,earliest_delay);
        double hi=fmax(lo,latest_delay);
        for(int iteration=0;iteration<7;iteration++){
            double mid=.5*(lo+hi);
            EntryTopologyPlan left=entry_topology_trial(
                state,t,base,p,env,cal,cfg,best.first_sign,
                best.first_bank,best.turn_bank,0.0,
                best.first_aoa,best.turn_aoa,
                reversal_delay,.5*(lo+mid));
            EntryTopologyPlan right=entry_topology_trial(
                state,t,base,p,env,cal,cfg,best.first_sign,
                best.first_bank,best.turn_bank,0.0,
                best.first_aoa,best.turn_aoa,
                reversal_delay,.5*(mid+hi));

            bool left_better=entry_topology_search_better(
                &left,&right);
            EntryTopologyPlan chosen=left_better?left:right;
            if(entry_topology_search_better(&chosen,&best))
                best=chosen;
            if(left_better)hi=mid;
            else lo=mid;
        }
    }

    if(getenv("KSP_LANDER_TRACE_TOPOLOGY_TIMING")){
        fprintf(stderr,
            "Entry topology constrained solve: valid=%d flags=0x%x side=%+.0f bank=%.2f/%.2f aoa=%.2f/%.2f rev=%.2fs term=%.2fs cap=%.2fs pos=%.1f course=%.2f veto=%u residual=%.5f\n",
            best.valid,best.failure_flags,best.first_sign,
            best.first_bank,best.turn_bank,best.first_aoa,best.turn_aoa,
            best.reversal_ut-state.ut,best.terminal_turn_ut-state.ut,
            best.capture_ut-state.ut,best.position_error,
            best.capture_course_error,best.capture_veto,best.cost);
    }
    if(getenv("KSP_LANDER_TRACE_TOPOLOGY_DETAIL")){
        fprintf(stderr,
            "Entry topology detail: targetCourse=%.2f reversal=(along=%+.1f cross=%+.1f h=%.1f V=%.1f course=%.2f) terminal=(along=%+.1f cross=%+.1f h=%.1f V=%.1f course=%.2f radius=%.1f geom=%.1f) capture=(along=%+.1f cross=%+.1f h=%.1f V=%.1f course=%.2f) reserve=%.1f energy=%+.1f turn=%+.1f\n",
            best.inlet.valid?best.inlet.course:NAN,
            best.reversal_along,best.reversal_cross,best.reversal_altitude,
            best.reversal_speed,best.reversal_course,
            best.terminal_turn_along,best.terminal_turn_cross,
            best.terminal_turn_altitude,best.terminal_turn_speed,
            best.terminal_turn_course,best.terminal_turn_radius,
            best.terminal_turn_geometry_error,
            best.capture_along,best.capture_cross,best.capture_altitude,
            best.capture_speed,best.capture_course,
            best.glide_reserve,best.terminal_energy_margin,
            best.capture_turn_margin);
    }
    return best;
}
