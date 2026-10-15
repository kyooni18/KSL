/* Frozen-state causal experiments using the production native integrators,
 * force model and guidance. White-box, like DeorbitWindowTests. No transport
 * function is called. The observation archive is opened READ_ONLY only.
 *
 * Usage: polar_reentry_replay FIXTURE OUTPUT_PREFIX MODE [ARG...]
 * MODE: execute-burn [DT] | vacuum | forces CSV | recorded CSV CLOCK FORCE | shadow LABEL [prior]
 *       candidate OFFSET_S NORMAL_DEG PERIAPSIS_M [prior] | plan [prior]
 * CLOCK: raw | kinematic; FORCE: prior | envelope | measured
 * "measured" is a dynamics reconstruction, never a predictive certificate.
 */
#include "landing.h"
#include "physics_store.h"
#include "sim_telemetry.h"
#include "terminal_model.h"
#include <sqlite3.h>
#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "../CLanding/prediction/predictor.c"

#define TOKEN_CAPACITY 65536
#define MAX_ROWS 30000
static JsonDoc doc;
static PlanetModel planet;
static LandingConfiguration config;
static AerodynamicEnvelope envelope;
static TrajectoryCalibrationModel calibration;
static VesselPhysicsModel physics;
static TerminalModel *terminal;
static const char *output;
static double number(int o,const char *key){return json_number(&doc,json_object_get(&doc,o,key),NAN);}
static Vector3 vector(int o,const char *key){int a=json_object_get(&doc,o,key);return v3(json_number(&doc,json_array_get(&doc,a,0),NAN),json_number(&doc,json_array_get(&doc,a,1),NAN),json_number(&doc,json_array_get(&doc,a,2),NAN));}
static int object(int o,const char *key){return json_object_get(&doc,o,key);}
static void fail(const char *s){fprintf(stderr,"%s\n",s);exit(2);}
static FILE *create_output(const char *suffix){char path[2048];snprintf(path,sizeof(path),"%s%s",output,suffix);FILE*f=fopen(path,"w");if(!f)fail("cannot open output");return f;}
static void print_v(FILE*f,Vector3 a){fprintf(f,"%.12g,%.12g,%.12g",a.x,a.y,a.z);}
static void load_planet(void){
    int p=object(0,"planet");
    planet.radius=number(p,"radius");planet.gravitational_parameter=number(p,"gravitationalParameter");
    planet.rotational_speed=number(p,"rotationalSpeed");planet.atmosphere_depth=number(p,"atmosphereDepth");
    planet.surface_density=number(p,"surfaceDensity");planet.atmosphere_adiabatic_index=number(p,"atmosphereAdiabaticIndex");
    planet.epoch_ut=number(p,"epochUT");planet.north_axis=vector(p,"northAxis");planet.prime_meridian_at_epoch=vector(p,"primeMeridianAtEpoch");
    int a=object(p,"atmosphereSamples");
    for(int i=0;;i++){int r=json_array_get(&doc,a,i);if(r<0)break;if(planet.atmosphere_sample_count>=LANDER_ATMOSPHERE_SAMPLE_MAX)fail("too many atmosphere samples");size_t n=planet.atmosphere_sample_count++;planet.atmosphere_altitude[n]=number(r,"altitude");planet.atmosphere_density[n]=number(r,"density");planet.atmosphere_pressure[n]=number(r,"pressure");}
    if(!(planet.radius>0)||!(planet.gravitational_parameter>0)||!planet.atmosphere_sample_count)fail("invalid planet fixture");
    int env=object(0,"aeroEnvelope");
    for(int i=0;i<4;i++){int e=json_array_get(&doc,env,i);envelope.regimes[i]=(AerodynamicModel){number(e,"liftToDrag"),number(e,"ballisticCoefficient"),number(e,"confidence")};if(!isfinite(envelope.regimes[i].lift_to_drag))fail("invalid aerodynamic envelope");}
    calibration=(TrajectoryCalibrationModel){.density_scale=1,.drag_scale=1,.lift_scale=1,.bank_effectiveness=1,.speed_of_sound=340,.speed_of_sound_scale=1,.stress_drag_scale=1,.stress_lift_scale=1};
}
static void load_history(void){
    int context=object(0,"physicsContext");char database[1024];
    if(!json_string(&doc,object(context,"storage"),database,sizeof(database)))fail("physics archive path missing");
    sqlite3*db=NULL;sqlite3_stmt*statement=NULL;
    if(sqlite3_open_v2(database,&db,SQLITE_OPEN_READONLY,NULL)!=SQLITE_OK)fail("cannot read physics archive");
    /* Select exactly this recorded flight, not the latest mutable session. */
    const char*sql="SELECT s.flight_key,c.structure_json,c.environment_json FROM physics_sessions s JOIN physics_contexts c ON c.structure_id=s.structure_id AND c.environment_id=s.environment_id WHERE s.session_id=?";
    if(sqlite3_prepare_v2(db,sql,-1,&statement,NULL)!=SQLITE_OK)fail(sqlite3_errmsg(db));
    char session[256];if(!json_string(&doc,object(0,"physicsSessionId"),session,sizeof(session)))fail("missing exact physicsSessionId; regenerate fixture with --physics-session");
    sqlite3_bind_text(statement,1,session,-1,SQLITE_STATIC);
    if(sqlite3_step(statement)!=SQLITE_ROW)fail("frozen physics session missing");
    const char*flight=(const char*)sqlite3_column_text(statement,0);
    const char*structure=(const char*)sqlite3_column_text(statement,1);
    const char*environment=(const char*)sqlite3_column_text(statement,2);
    PhysicsStoreOptions options={.path=database,.model_id="STS",.vessel_name="STS",.structure_manifest_json=structure,.environment_manifest_json=environment,.session_id=session,.flight_key=flight,.mode=PHYSICS_STORE_READ_ONLY,.structure_variant=getenv("KSP_LANDER_PHYSICS_VARIANT")};
    char error[1024]={0};PhysicsStore*store=physics_store_open(&options,error,sizeof(error));if(!store)fail(error);
    VesselAeroSample samples[VESSEL_AERO_SAMPLES];size_t n=physics_store_load_history(store,&planet,samples,VESSEL_AERO_SAMPLES,error,sizeof(error));if(error[0])fail(error);
    vessel_physics_init(&physics);for(size_t i=0;i<n;i++)vessel_physics_import_sample(&physics,&samples[i]);
    fprintf(stderr,"prior_cells=%zu recorded_initial_cells=%.0f excluded_flight=%s\n",n,number(context,"historicalSamplesLoaded"),flight);
    if(n!=(size_t)number(context,"historicalSamplesLoaded")&&!getenv("REPLAY_ALLOW_PRIOR_CHANGE"))fail("certified prior changed: freeze compatible original history before comparing");
    FILE*f=create_output("-prior.csv");fprintf(f,"q,mach,aoa,beta,mass,fx_per_q,fy_per_q,fz_per_q,gear,brakes,airbrakes,observations,trust\n");
    for(size_t i=0;i<n;i++){VesselAeroSample*s=&samples[i];fprintf(f,"%.17g,%.17g,%.17g,%.17g,%.17g,",s->q,s->mach,s->aoa,s->beta,s->mass);print_v(f,s->force_per_q);fprintf(f,",%d,%d,%d,%u,%.17g\n",s->gear,s->brakes,s->airbrakes,s->observations,s->trust);}fclose(f);
    int identity=object(0,"modelIdentity");physics.certified_uncertainty=number(identity,"physicsCertifiedUncertainty");physics.certified_confidence=number(identity,"physicsCertifiedConfidence");
    calibration.physics=&physics;physics_store_close(store);sqlite3_finalize(statement);sqlite3_close(db);
}
static void load_terminal(void){
    char paths[4][512],reason[512],profile[128];
    if(!json_string(&doc,object(0,"modelProfile"),profile,sizeof(profile)))fail("missing recorded model profile");
    if(!getenv("KSP_LANDER_MODEL_PROFILE"))setenv("KSP_LANDER_MODEL_PROFILE",profile,1);
    if(!shuttle_sim_model_matches_vehicle(config.vehicle.model_id,reason,sizeof(reason)))fail(reason);
    TerminalModelSourceFiles files={
        shuttle_sim_model_path(NULL,SHUTTLE_SIM_MODEL_ATMOSPHERE,paths[0],512),
        shuttle_sim_model_path(NULL,SHUTTLE_SIM_MODEL_AERO,paths[1],512),
        shuttle_sim_model_path(NULL,SHUTTLE_SIM_MODEL_FORCE_BOOK,paths[2],512),
        shuttle_sim_model_path(NULL,SHUTTLE_SIM_MODEL_ATTITUDE,paths[3],512)};
    terminal=calloc(1,sizeof(*terminal));if(!terminal||!terminal_model_capture(terminal,&files,&config,20261013,reason,sizeof(reason))||!terminal_model_align_planet_frame(terminal,&planet,reason,sizeof(reason)))fail(reason);
    fprintf(stderr,"terminal_profile=%s aero=%s force_book=%s frame_aligned=%d\n",shuttle_sim_model_profile(),files.aero_csv,files.aero_book_csv,terminal->planet_frame_aligned);
}
static int sample_named(const char*label){int samples=object(0,"samples");for(int i=0;;i++){int r=json_array_get(&doc,samples,i);if(r<0)break;if(json_token_eq(&doc,object(r,"label"),label))return r;}fail("sample label not found");return -1;}
static VehicleState load_state(int row,Telemetry*t){
    int f=object(row,"fields"),po=object(f,"position"),mo=object(f,"motion"),at=object(f,"attitude"),ae=object(f,"aero"),ve=object(f,"vehicle"),gu=object(f,"guidance"),ph=object(f,"physics");
    telemetry_init(t);t->ut=number(row,"ut");t->latitude=number(po,"latitude");t->longitude=number(po,"longitude");t->mean_altitude=number(po,"altitude");t->radar_altitude=number(po,"radarAltitude");
    t->true_air_speed=number(mo,"trueAirSpeed");t->horizontal_speed=number(mo,"horizontalSpeed");t->vertical_speed=number(mo,"verticalSpeed");t->flight_path_angle=number(mo,"flightPathAngle");
    t->pitch=number(at,"pitch");t->heading=number(at,"heading");t->roll=number(at,"roll");t->angle_of_attack=number(at,"angleOfAttack");t->sideslip=number(at,"sideslip");t->roll_rate=number(at,"coordinateRollRate");if(!isfinite(t->roll_rate))t->roll_rate=0;
    t->angle_of_attack_rate=number(at,"angleOfAttackRate");t->has_angle_of_attack_rate=isfinite(t->angle_of_attack_rate);
    t->dynamic_pressure=number(ae,"dynamicPressure");t->mach=number(ae,"mach");t->lift_force=number(ae,"liftForce");t->drag_force=number(ae,"dragForce");t->g_force=number(ae,"gForce");t->mass=number(ve,"mass");t->available_thrust=number(ve,"availableThrust");t->current_thrust=number(ve,"currentThrust");t->has_airbrakes=true;t->airbrakes=false;
    t->atmospheric_density=number(ae,"atmosphericDensity");t->speed_of_sound=t->true_air_speed/fmax(t->mach,.001);t->stall_fraction=number(ae,"stallFraction");t->stall_fraction_is_measured=false;
    t->range_to_site=number(gu,"rangeToSite");t->bearing_to_site=number(gu,"bearingToSite");t->runway_along_track=number(gu,"runwayAlongTrack");t->runway_cross_track=number(gu,"runwayCrossTrack");
    t->physics_certified_uncertainty=number(ph,"certifiedUncertainty");t->physics_model_residual=number(ph,"modelResidual");t->physics_model_residual_confidence=number(ph,"modelResidualConfidence");t->aerodynamic_confidence=number(ph,"confidence");
    t->estimated_lift_to_drag=config.vehicle.estimated_lift_to_drag;t->estimated_ballistic_coefficient=config.vehicle.estimated_ballistic_coefficient;t->bank_effectiveness=1;
    Vector3 torque=vector(ve,"availableTorque"),inertia=vector(ve,"momentOfInertia");
    t->has_torque=true;t->has_inertia=true;t->available_pitch_torque=torque.x;t->available_roll_torque=torque.y;t->available_yaw_torque=torque.z;t->pitch_moment_of_inertia=inertia.x;t->roll_moment_of_inertia=inertia.y;t->yaw_moment_of_inertia=inertia.z;
    VehicleState state={t->ut,vector(po,"inertial"),vector(mo,"velocity"),t->mass};
    t->ground_track_heading=surface_course(state.position,state.velocity,planet_rotation_vector(&planet),planet.north_axis,t->heading);t->course_to_site_error=norm_signed_deg(t->bearing_to_site-t->ground_track_heading);
    t->periapsis_altitude=predictor_postburn_periapsis(state,&planet);
    return state;
}
static VehicleState planned_state(void){int p=object(0,"plannedPostburn");return (VehicleState){number(p,"ut"),vector(p,"position"),vector(p,"velocity"),number(p,"mass")};}
static void trajectory_csv(const EntryPrediction*prediction){FILE*f=create_output("-trajectory.csv");fprintf(f,"ut,altitude,speed,latitude,longitude,phase,range\n");GeoPoint site={config.site.latitude,config.site.longitude,config.site.altitude};for(size_t i=0;i<prediction->trajectory.count;i++){const TrajectoryPoint*r=&prediction->trajectory.points[i];GeoPoint g={r->latitude,r->longitude,r->altitude};fprintf(f,"%.9f,%.9f,%.9f,%.12f,%.12f,%d,%.9f\n",r->ut,r->altitude,r->speed,r->latitude,r->longitude,r->phase,great_circle_distance(g,site,planet.radius));}fclose(f);}
static void prediction_summary(EntryPrediction*pr,VehicleState initial){
    printf("startUT=%.9f startAlt=%.6f startPe=%.6f endUT=%.9f endAlt=%.6f closest=%.6f entryRange=%.6f entryFPA=%.6f gate50range=%.6f gate50speed=%.6f taem=%d taemRange=%.6f taemAlt=%.6f taemSpeed=%.6f taemCourse=%.6f inlet=%d inletMargin=%.6f route=%d final=%d aborted=%d peakQ=%.6f peakG=%.6f maxAoA=%.6f samples=%zu\n",initial.ut,vmag(initial.position)-planet.radius,predictor_postburn_periapsis(initial,&planet),pr->final_state.ut,vmag(pr->final_state.position)-planet.radius,pr->closest_distance,pr->entry_range,pr->entry_flight_path_angle,pr->mm304_gate_range,pr->mm304_gate_speed,pr->reached_taem,pr->taem_distance,pr->taem_altitude,pr->taem_speed,pr->taem_course,pr->taem_dynamic_interface_captured,pr->taem_interface_energy_margin,pr->shadow_terminal_path_committed,pr->shadow_final_approach_captured,pr->shadow_aborted,pr->peak_dynamic_pressure,pr->peak_g_load,pr->maximum_abs_angle_of_attack,pr->trajectory.count);
    trajectory_csv(pr);
}
static void burn_study(void){
    Telemetry t,live;VehicleState initial=load_state(object(0,"orbitalSample"),&t),ignition=load_state(object(0,"ignitionSample"),&live),planned=planned_state();
    int p=object(0,"planParameters");double start=number(p,"burnUT")-.5*number(p,"estimatedBurnDuration");
    VehicleState predicted_ignition=predictor_propagate_vacuum(initial,ignition.ut,&planet,.5);
    printf("ignition UT=%.9f sourceUT=%.9f preburn_position_error=%.9f preburn_velocity_error=%.9f actual_mass=%.9f initial_mass=%.9f\n",ignition.ut,initial.ut,vmag(vsub(predicted_ignition.position,ignition.position)),vmag(vsub(predicted_ignition.velocity,ignition.velocity)),ignition.mass,initial.mass);
    Telemetry pt;VehicleState ps=load_state(object(0,"planningSample"),&pt),pc=predictor_propagate_vacuum(initial,ps.ut,&planet,.01);Vector3 pv=vscale(vadd(pc.velocity,ps.velocity),.5);
    printf("planning_seed UT=%.9f dt=%.9f position_error=%.9f velocity_error=%.9f along_time_lag=%.9f\n",ps.ut,ps.ut-initial.ut,vmag(vsub(pc.position,ps.position)),vmag(vsub(pc.velocity,ps.velocity)),vdot(vsub(pc.position,ps.position),pv)/fmax(1.,vdot(pv,pv)));
    for(int i=0;i<3;i++){Telemetry st;VehicleState origin=i==2?load_state(object(0,"planningSample"),&st):initial;VehicleState coast=i==1?ignition:predictor_propagate_vacuum(origin,start,&planet,.5);double duration,delivered;
        VehicleState after=retro_burn_scenario(coast,number(p,"deltaV"),t.available_thrust,config.guidance.deorbit_maximum_throttle,config.guidance.deorbit_throttle_ramp_duration,&planet,0,number(p,"burnNormalAngle"),&duration,&delivered,NULL);
        VehicleState aligned=predictor_propagate_vacuum(planned,after.ut,&planet,.25);
        printf("finite_burn case=%s duration=%.9f state_elapsed=%.9f afterUT=%.9f peri=%.9f delivered=%.9f original_prediction_position_error=%.9f original_prediction_velocity_error=%.9f\n",i==2?"planning_seed":(i==1?"measured_ignition":"orbital_source"),duration,after.ut-coast.ut,after.ut,predictor_postburn_periapsis(after,&planet),delivered,vmag(vsub(aligned.position,after.position)),vmag(vsub(aligned.velocity,after.velocity)));
    }
}
static void vacuum(void){
    Telemetry t;VehicleState states[2]={planned_state(),load_state(sample_named("postburn"),&t)};
    FILE*f=create_output("-vacuum.csv");fprintf(f,"case,ut,altitude,speed,vertical,range,x,y,z,vx,vy,vz\n");
    for(int j=0;j<2;j++){VehicleState s=states[j];printf("case=%s startPe=%.9f startUT=%.9f\n",j?"measured":"planned",predictor_postburn_periapsis(s,&planet),s.ut);for(unsigned i=0;i<6000&&vmag(s.position)-planet.radius>65000;i++){GeoPoint g=predictor_geo_point(s.position,&planet,s.ut),site={config.site.latitude,config.site.longitude,config.site.altitude};Vector3 air=vsub(s.velocity,vcross(planet_rotation_vector(&planet),s.position));fprintf(f,"%s,%.9f,%.9f,%.9f,%.9f,%.9f,",j?"measured":"planned",s.ut,g.altitude,vmag(air),vdot(air,vnorm(s.position,v3(1,0,0))),great_circle_distance(g,site,planet.radius));print_v(f,s.position);fputc(',',f);print_v(f,s.velocity);fputc('\n',f);s=rk4_gravity(s,.25,&planet);}}fclose(f);
}
typedef struct {double ut,dt,mass,aoa,beta,bank,q,mach,cmd_aoa,cmd_bank,thrust;Vector3 p,v,lift,drag;} Recorded;
static Recorded*load_csv(const char*path,size_t*count){FILE*f=fopen(path,"r");if(!f)fail("cannot open recorded CSV");Recorded*rows=calloc(MAX_ROWS,sizeof(*rows));if(!rows)fail("allocation");char line[8192];if(!fgets(line,sizeof(line),f))fail("empty CSV");*count=0;while(fgets(line,sizeof(line),f)){if(*count>=MAX_ROWS)fail("too many CSV rows");double x[23];char*cursor=line;for(int i=0;i<23;i++){char*end;errno=0;x[i]=strtod(cursor,&end);if(end==cursor||errno)fail("invalid CSV number");cursor=end;if(i<22){if(*cursor!=',')fail("invalid CSV separator");cursor++;}}rows[(*count)++]=(Recorded){x[0],x[1],x[2],x[3],x[4],x[5],x[6],x[7],x[8],x[9],x[10],{x[11],x[12],x[13]},{x[14],x[15],x[16]},{x[17],x[18],x[19]},{x[20],x[21],x[22]}};}fclose(f);return rows;}
static Vector3 measured_specific(Recorded*r){Vector3 air=vsub(r->v,vcross(planet_rotation_vector(&planet),r->p));Vector3 dir=vnorm(air,v3(1,0,0)),up=vnorm(vproject_plane(r->p,dir),v3(0,0,1)),side=vnorm(vcross(dir,up),v3(0,1,0));double b=r->bank*DEG2RAD;Vector3 lift=vadd(vscale(up,cos(b)),vscale(side,sin(b))),lateral=vsub(vscale(side,cos(b)),vscale(up,sin(b))),force=vscale(vadd(r->lift,r->drag),1/r->mass);return v3(-vdot(force,dir),vdot(force,lift),vdot(force,lateral));}
static void forces(const char*path){size_t n;Recorded*r=load_csv(path,&n);load_history();FILE*f=create_output("-forces.csv");fprintf(f,"ut,altitude,q,mach,aoa,bank,measured_D,measured_L,measured_side,model_D,model_L,model_side,direct,confidence,uncertainty,environment_q,environment_mach,actual_vertical_lift,model_vertical_aero,physics_dt,raw_dt,measured_force_dv_error,model_force_dv_error,model_raw_dt_dv_error\n");
    for(size_t i=0;i+1<n;i++){Recorded*a=&r[i],*b=&r[i+1];if(a->q<1)continue;double conf,unc;bool direct;Vector3 measured=measured_specific(a);Vector3 specific=vessel_physics_force_best_estimate_config(&physics,a->q,a->mach,a->aoa,a->beta,a->mass,false,false,0,aero_at(&envelope,a->mach),&calibration,&config.vehicle,&conf,&direct,&unc);Vector3 air=vsub(a->v,vcross(planet_rotation_vector(&planet),a->p));Vector3 grav=gravity(a->p,&planet),ma=vadd(grav,vscale(vadd(a->lift,a->drag),1/a->mass)),pa=vessel_physics_acceleration(a->p,air,&planet,specific,a->bank);AeroContext ctx={.p=&planet,.e=&envelope,.c=&calibration,.v=&config.vehicle,.bank=a->bank,.aoa=a->aoa,.ut=a->ut,.mass=a->mass};AtmosState env=atmosphere(a->p,a->v,&ctx);double dt=b->dt,raw=b->ut-a->ut;Vector3 dv=vsub(b->v,a->v),up=vnorm(a->p,v3(1,0,0));fprintf(f,"%.9f,%.6f,%.9f,%.9f,%.9f,%.9f,%.9f,%.9f,%.9f,%.9f,%.9f,%.9f,%d,%.9f,%.9f,%.9f,%.9f,%.9f,%.9f,%.9f,%.9f,%.9f,%.9f,%.9f\n",a->ut,vmag(a->p)-planet.radius,a->q,a->mach,a->aoa,a->bank,measured.x,measured.y,measured.z,specific.x,specific.y,specific.z,direct,conf,unc,env.q,env.mach,vdot(a->lift,up)/a->mass,vdot(vsub(pa,grav),up),dt,raw,vmag(vsub(dv,vscale(ma,dt))),vmag(vsub(dv,vscale(pa,dt))),vmag(vsub(dv,vscale(pa,raw))));}fclose(f);free(r);
}
static VehicleState measured_force_step(VehicleState s,double dt,Vector3 force){VehicleState g=rk4_gravity(s,dt,&planet);g.position=vadd(g.position,vscale(force,.5*dt*dt));g.velocity=vadd(g.velocity,vscale(force,dt));return g;}
static void recorded(const char*path,const char*clock,const char*force){size_t n;Recorded*r=load_csv(path,&n);bool raw=!strcmp(clock,"raw"),measured=!strcmp(force,"measured");if(!strcmp(force,"prior"))load_history();VehicleState s={r[0].ut,r[0].p,r[0].v,r[0].mass};FILE*f=create_output("-recorded.csv");fprintf(f,"ut,actual_altitude,predicted_altitude,actual_speed,predicted_speed,position_error,velocity_error,actual_range,predicted_range,actual_q,predicted_q,aoa,bank,physics_dt,raw_dt\n");double max_step=.1;
    for(size_t i=1;i<n;i++){Recorded*a=&r[i-1],*b=&r[i];double interval=raw?b->ut-a->ut:b->dt;if(interval<0||interval>30||!isfinite(interval))fail("recorded interval outside replay bound");double done=0;s.mass=.5*(a->mass+b->mass);while(done<interval-1e-9){double dt=fmin(max_step,interval-done);double fraction=interval>0?(done+.5*dt)/interval:0;AeroContext ctx={.p=&planet,.e=&envelope,.c=&calibration,.v=&config.vehicle,.bank=a->bank+fraction*norm_signed_deg(b->bank-a->bank),.aoa=a->aoa+fraction*(b->aoa-a->aoa),.ut=s.ut,.mass=s.mass};if(measured){Vector3 specific=vscale(vadd(vscale(vadd(a->lift,a->drag),1-fraction),vscale(vadd(b->lift,b->drag),fraction)),1/s.mass);s=measured_force_step(s,dt,specific);}else s=rk4_aero(s,dt,&ctx);done+=dt;}s.ut=b->ut;s.mass=b->mass;
        GeoPoint actual=predictor_geo_point(b->p,&planet,b->ut),pred=predictor_geo_point(s.position,&planet,s.ut),site={config.site.latitude,config.site.longitude,config.site.altitude};AeroContext ctx={.p=&planet,.e=&envelope,.c=&calibration,.v=&config.vehicle,.bank=b->bank,.aoa=b->aoa,.ut=s.ut,.mass=s.mass};AtmosState atm=atmosphere(s.position,s.velocity,&ctx);Vector3 actualair=vsub(b->v,vcross(planet_rotation_vector(&planet),b->p));fprintf(f,"%.9f,%.9f,%.9f,%.9f,%.9f,%.9f,%.9f,%.9f,%.9f,%.9f,%.9f,%.9f,%.9f,%.9f,%.9f\n",s.ut,actual.altitude,pred.altitude,vmag(actualair),atm.speed,vmag(vsub(s.position,b->p)),vmag(vsub(s.velocity,b->v)),great_circle_distance(actual,site,planet.radius),great_circle_distance(pred,site,planet.radius),b->q,atm.q,b->aoa,b->bank,b->dt,b->ut-a->ut);if(!isfinite(pred.altitude)||pred.altitude<0||pred.altitude>200000){fprintf(stderr,"recorded open-loop diverged at row %zu, altitude %.1f; not a landing\n",i,pred.altitude);break;}}
    fclose(f);free(r);
}
static void close_burn(double dt){
    load_history();load_terminal();Telemetry t;VehicleState s=load_state(sample_named("postburn"),&t),start=s;
    int pp=object(0,"planParameters");DeorbitPlan plan={0};plan.burn_ut=number(pp,"burnUT");plan.estimated_burn_duration=number(pp,"estimatedBurnDuration");plan.delta_v=number(pp,"deltaV");plan.burn_normal_angle=number(pp,"burnNormalAngle");plan.predicted_post_burn_periapsis_altitude=number(pp,"predictedPostBurnPeriapsisAltitude");plan.execution_qualified=true;plan.live_cutoff_capture_qualified=true;
    GuidanceMachine g;guidance_machine_init(&g);guidance_set_engaged(&g,true);g.has_burn_command_started=true;g.burn_active_elapsed=117.75;g.delivered_delta_v=42.05813674171725;g.has_previous_ut=true;g.previous_ut=s.ut-dt;
    t.autopilot_error=0.;double impulse=0.,last_throttle=0.;FILE*f=create_output("-closure.csv");fprintf(f,"ut,periapsis,phase,throttle,delivered_delta_v\n");
    for(int i=0;i<3000;i++){
        t.ut=s.ut;t.mass=s.mass;t.mean_altitude=vmag(s.position)-planet.radius;t.periapsis_altitude=predictor_postburn_periapsis(s,&planet);t.current_thrust=t.available_thrust*last_throttle;
        GuidanceResult r=guidance_update(&g,&t,&s,&plan,&planet,envelope.regimes[2],&config);
        fprintf(f,"%.9f,%.9f,%d,%.9f,%.9f\n",s.ut,t.periapsis_altitude,r.phase,r.command.target_throttle,g.delivered_delta_v);
        if(r.phase!=PHASE_DEORBIT_BURN){guidance_result_clear(&r);break;}
        last_throttle=r.command.target_throttle;Vector3 force=vscale(r.command.inertial_direction,t.available_thrust*last_throttle/s.mass);impulse+=vmag(force)*dt;s=measured_force_step(s,dt,force);guidance_result_clear(&r);
    }
    fclose(f);printf("burn_closure step=%.6f duration=%.9f initialPe=%.9f finalPe=%.9f additionalImpulsePerMass=%.9f completed=%d fixed_mass_assumption=1\n",dt,s.ut-start.ut,predictor_postburn_periapsis(start,&planet),predictor_postburn_periapsis(s,&planet),impulse,g.deorbit_burn_completed);
    if(!g.deorbit_burn_completed)fail("burn closure did not finish within diagnostic bound");
    PredictorGuidance response={0};AeroContext ctx={.p=&planet,.e=&envelope,.c=&calibration,.v=&config.vehicle,.ut=s.ut,.mass=s.mass};Telemetry seed=guidance_shadow_telemetry(&t,s,&response,&ctx,&planet,&config,NULL);copy_control_authority_evidence(&seed,&t);GuidanceMachine entry;guidance_initialize_reentry_continuation(&entry,&seed,&planet,&config,1.,false,&envelope,&calibration);
    TerminalShadowOutcome outcome;EntryPrediction prediction=predictor_simulate_guidance_shadow_core(s,&seed,&entry,&plan,&planet,envelope.regimes[2],&envelope,&calibration,&config,4500,true,false,false,terminal,&outcome);prediction_summary(&prediction,s);entry_prediction_clear(&prediction);free(terminal);
}
static void shadow(const char*label,bool prior){if(prior)load_history();load_terminal();Telemetry t;VehicleState s;if(!strcmp(label,"planned")){s=planned_state();PredictorGuidance response={0};AeroContext ctx={.p=&planet,.e=&envelope,.c=&calibration,.v=&config.vehicle,.ut=s.ut,.mass=s.mass};t=guidance_shadow_telemetry(NULL,s,&response,&ctx,&planet,&config,NULL);Telemetry authority;load_state(sample_named("postburn"),&authority);copy_control_authority_evidence(&t,&authority);}else s=load_state(sample_named(label),&t);
    GuidanceMachine g;guidance_initialize_reentry_continuation(&g,&t,&planet,&config,t.roll<0?-1:1,false,&envelope,&calibration);
    DeorbitPlan plan={0};plan.predicted_post_burn_periapsis_altitude=predictor_postburn_periapsis(s,&planet);plan.execution_qualified=true;
    TerminalShadowOutcome outcome;EntryPrediction result=predictor_simulate_guidance_shadow_core(s,&t,&g,&plan,&planet,envelope.regimes[2],&envelope,&calibration,&config,4500,true,false,false,terminal,&outcome);prediction_summary(&result,s);entry_prediction_clear(&result);free(terminal);
}
static void candidate(double offset,double normal,double peri,bool prior){if(prior)load_history();config.guidance.target_post_burn_periapsis_altitude=peri;load_terminal();Telemetry t;VehicleState initial=load_state(object(0,"orbitalSample"),&t);int p=object(0,"planParameters");double start=number(p,"burnUT")-.5*number(p,"estimatedBurnDuration")+offset;config.guidance.target_post_burn_periapsis_altitude=peri;/* Rebind policy after a scenario changes the orbital target. */char reason[512];if(!terminal_model_align_planet_frame(terminal,&planet,reason,sizeof(reason)))fail(reason);
    VehicleState coast=predictor_propagate_vacuum(initial,start,&planet,2.);double dv;DeorbitRejectReason reject;if(!deorbit_solve_target_periapsis(coast,normal,&planet,&config.guidance,t.available_thrust,&calibration,&dv,&reject)){printf("candidate offset=%.1f normal=%.4f peri=%.1f solved=0 reject=%d\n",offset,normal,peri,reject);free(terminal);return;}
    Candidate c={0};double delivered;c.after=retro_burn_scenario(coast,dv,t.available_thrust,config.guidance.deorbit_maximum_throttle,config.guidance.deorbit_throttle_ramp_duration,&planet,0,normal,&c.duration,&delivered,calibration.physics);c.dv=dv;c.burn_ut=start+.5*c.duration;c.peri=predictor_postburn_periapsis(c.after,&planet);c.burn_normal_angle=normal;
    printf("candidate offset=%.1f normal=%.4f peri=%.1f solved=1 dv=%.9f duration=%.9f state_elapsed=%.9f delivered=%.9f\n",offset,normal,peri,dv,c.duration,c.after.ut-coast.ut,delivered);EntryPrediction result=deorbit_executable_policy_witness(&c,&planet,envelope.regimes[2],&envelope,&calibration,&config.site,&config.vehicle,&config.guidance,&t,terminal,true,true);prediction_summary(&result,c.after);entry_prediction_clear(&result);free(terminal);
}
/* Execute the recorded plan with fresh measured-state periapsis feedback.
   This isolates the burn controller from the atmospheric forecast's wall time. */
static void execute_burn(double dt){
    if(!(dt>0.0)||dt>1.0)fail("burn timestep must be in (0, 1] seconds");
    load_history();load_terminal();Telemetry t;
    VehicleState initial=load_state(object(0,"planningSample"),&t);
    int pp=object(0,"planParameters");DeorbitPlan plan={0};
    plan.burn_ut=number(pp,"burnUT");plan.estimated_burn_duration=number(pp,"estimatedBurnDuration");
    plan.delta_v=number(pp,"deltaV");plan.burn_normal_angle=number(pp,"burnNormalAngle");
    plan.predicted_post_burn_periapsis_altitude=number(pp,"predictedPostBurnPeriapsisAltitude");
    plan.execution_qualified=true;
    double burnstart=plan.burn_ut-.5*plan.estimated_burn_duration;
    VehicleState state=predictor_propagate_vacuum(initial,burnstart,&planet,dt);
    GuidanceMachine g;guidance_machine_init(&g);guidance_set_engaged(&g,true);
    g.has_previous_ut=true;g.previous_ut=state.ut-dt;t.autopilot_error=0.0;
    double last_throttle=0.0,impulse=0.0;
    FILE*f=create_output("-burn.csv");fprintf(f,"ut,periapsis,throttle,delivered_delta_v\n");
    for(double elapsed=0;elapsed<2.0*plan.estimated_burn_duration;elapsed+=dt){
        t.ut=state.ut;t.mass=state.mass;t.mean_altitude=vmag(state.position)-planet.radius;
        t.periapsis_altitude=predictor_postburn_periapsis(state,&planet);
        t.current_thrust=t.available_thrust*last_throttle;
        GuidanceResult r=guidance_update(&g,&t,&state,&plan,&planet,envelope.regimes[2],&config);
        fprintf(f,"%.9f,%.9f,%.9f,%.9f\n",state.ut,t.periapsis_altitude,r.command.target_throttle,g.delivered_delta_v);
        if(g.deorbit_burn_completed){guidance_result_clear(&r);break;}
        last_throttle=r.command.target_throttle;
        double thrust=t.available_thrust*last_throttle;
        Vector3 force=vscale(r.command.inertial_direction,thrust/state.mass);
        impulse+=vmag(force)*dt;
        state=measured_force_step(state,dt,force);
        state.mass=vessel_physics_burn_mass(&physics,state.mass,thrust,dt);
        guidance_result_clear(&r);
    }
    fclose(f);
    printf("feedback_burn dt=%.3f duration=%.3f completed=%d peri=%.3f target=%.3f impulse=%.6f delivered=%.6f\n",
        dt,state.ut-burnstart,g.deorbit_burn_completed,predictor_postburn_periapsis(state,&planet),
        plan.predicted_post_burn_periapsis_altitude,impulse,g.delivered_delta_v);
    if(!g.deorbit_burn_completed)fail("feedback burn did not finish");
    /* Freeze an entry checkpoint in ShuttleSim's canonical longitude frame. */
    double entry_alt=planet.atmosphere_depth+config.guidance.entry_interface_altitude_margin;
    double coast_limit=state.ut+2.0*LANDER_PI*sqrt(pow(vmag(initial.position),3)/planet.gravitational_parameter);
    while(vmag(state.position)-planet.radius>entry_alt||vdot(state.position,state.velocity)>=0){
        if(state.ut>=coast_limit)fail("feedback orbit did not reach entry within one period");
        state=rk4_gravity(state,dt,&planet);
    }
    double phase=atan2(planet.prime_meridian_at_epoch.y,planet.prime_meridian_at_epoch.x)-
        planet.rotational_speed*planet.epoch_ut;
    KerbinWorld simulator_world;world_seed_kerbin(&simulator_world);
    double rotation=simulator_world.rotation_phase_rad_at_ut0-phase;
    Vector3 position=vrotate(state.position,planet.north_axis,rotation);
    Vector3 velocity=vrotate(state.velocity,planet.north_axis,rotation);
    f=create_output("-entry.ini");
    fprintf(f,"name=recorded-plan-feedback-entry\nut0=%.17g\nposition_x_m=%.17g\nposition_y_m=%.17g\nposition_z_m=%.17g\nvelocity_x_mps=%.17g\nvelocity_y_mps=%.17g\nvelocity_z_mps=%.17g\nmass_kg=%.17g\ninitial_aoa_deg=0\ninitial_bank_deg=0\ndeorbit_delta_v_mps=0\ndeorbit_duration_s=0\ndeorbit_delay_s=0\nrunway_latitude_deg=%.17g\nrunway_longitude_deg=%.17g\nrunway_elevation_m=%.17g\nrunway_heading_deg=%.17g\nrunway_length_m=%.17g\nrunway_width_m=%.17g\n",state.ut,position.x,position.y,position.z,velocity.x,velocity.y,velocity.z,state.mass,config.site.latitude,config.site.longitude,config.site.altitude,config.site.runway_heading,config.site.runway_length,config.site.runway_width);
    fclose(f);free(terminal);
}

/* Full native deorbit planner from the recorded pre-plan orbit, then the
   closed-loop executable witness for the selected burn (offline, read-only). */
static void plan_mode(bool prior){if(prior)load_history();load_terminal();Telemetry t;
    /* Replay the actual createPlan sample, not an earlier connect sample. */
    int initial_sample=object(0,"planningSample");
    if(initial_sample<0)initial_sample=object(0,"orbitalSample");
    VehicleState initial=load_state(initial_sample,&t);
    double r=vmag(initial.position),a=1/(2/r-vdot(initial.velocity,initial.velocity)/planet.gravitational_parameter),period=2*M_PI*sqrt(a*a*a/planet.gravitational_parameter);
    DeorbitPlan out;memset(&out,0,sizeof(out));bool ok=deorbit_plan_create_with_terminal_model(&out,initial,period,&planet,envelope.regimes[2],&envelope,&calibration,&config.site,&config.vehicle,&config.guidance,t.available_thrust,&t,terminal);
    printf("plan ok=%d burnUT=%.3f dv=%.4f normal=%.4f duration=%.3f taemDist=%.1f taemErr=%.1f closest=%.1f peri=%.1f nominal=%d robust=%d/%d inlet=%d route=%d final=%d exec=%d degraded=%d note=%s\n",ok,out.burn_ut,out.delta_v,out.burn_normal_angle,out.estimated_burn_duration,out.predicted_taem_distance,out.predicted_taem_range_error,out.predicted_closest_distance,out.predicted_post_burn_periapsis_altitude,out.nominal_capture_achieved,out.robustness_passed,out.robustness_scenarios,out.terminal_inlet_qualified,out.terminal_native_path_committed,out.terminal_final_approach_captured,out.execution_qualified,out.execution_degraded,out.note);
    if(!ok){free(terminal);return;}
    LandingSnapshot snapshot;landing_snapshot_init(&snapshot,&config.vehicle);
    snapshot.plan=&out;
    JsonWriter writer;jw_init(&writer);snapshot_json(&writer,&snapshot);
    if(writer.failed)fail("could not encode selected plan");
    FILE *plan_file=create_output("-plan-snapshot.json");
    fputs(writer.data,plan_file);fputc('\n',plan_file);fclose(plan_file);jw_free(&writer);
    /* REPLAY_TRUTH_ENVELOPE="ld0,bc0,ld1,bc1,ld2,bc2,ld3,bc3": fly the selected
       burn's witness on a different (e.g. flight-calibrated) plant. */
    const char*truth=getenv("REPLAY_TRUTH_ENVELOPE");if(truth){double x[8];if(sscanf(truth,"%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf",&x[0],&x[1],&x[2],&x[3],&x[4],&x[5],&x[6],&x[7])!=8)fail("bad REPLAY_TRUTH_ENVELOPE");for(int i=0;i<4;i++){envelope.regimes[i].lift_to_drag=x[2*i];envelope.regimes[i].ballistic_coefficient=x[2*i+1];}}
    double start=out.burn_ut-.5*out.estimated_burn_duration;VehicleState coast=predictor_propagate_vacuum(initial,start,&planet,2.);
    Candidate c={0};double delivered;c.after=retro_burn_scenario(coast,out.delta_v,t.available_thrust,config.guidance.deorbit_maximum_throttle,config.guidance.deorbit_throttle_ramp_duration,&planet,0,out.burn_normal_angle,&c.duration,&delivered,calibration.physics);c.dv=out.delta_v;c.burn_ut=start+.5*c.duration;c.peri=predictor_postburn_periapsis(c.after,&planet);c.burn_normal_angle=out.burn_normal_angle;
    EntryPrediction result=deorbit_executable_policy_witness(&c,&planet,envelope.regimes[2],&envelope,&calibration,&config.site,&config.vehicle,&config.guidance,&t,terminal,true,true);prediction_summary(&result,c.after);entry_prediction_clear(&result);trajectory_clear(&out.trajectory);free(terminal);
}
int main(int argc,char**argv){if(argc<4)fail("usage: polar_reentry_replay FIXTURE OUTPUT_PREFIX MODE [ARGS]");output=argv[2];FILE*f=fopen(argv[1],"rb");if(!f)fail("fixture not found");if(fseek(f,0,SEEK_END))fail("fixture seek");long n=ftell(f);if(n<0||n>10000000)fail("fixture size");rewind(f);char*text=calloc((size_t)n+1,1);JsonToken*tokens=calloc(TOKEN_CAPACITY,sizeof(*tokens));if(!text||!tokens||fread(text,1,(size_t)n,f)!=(size_t)n)fail("fixture read");fclose(f);if(json_parse(text,tokens,TOKEN_CAPACITY,&doc)<0)fail("fixture JSON parse");config=landing_configuration_default();if(!landing_configuration_from_json(&config,&doc,object(0,"configuration")))fail("configuration parse");load_planet();setvbuf(stdout,NULL,_IOLBF,0);
    if(!strcmp(argv[3],"vacuum"))vacuum();
    else if(!strcmp(argv[3],"burn"))burn_study();
    else if(!strcmp(argv[3],"execute-burn"))execute_burn(argc>4?strtod(argv[4],NULL):.1);
    else if(!strcmp(argv[3],"close-burn"))close_burn(argc>4?strtod(argv[4],NULL):.1);
    else if(!strcmp(argv[3],"forces")&&argc==5)forces(argv[4]);
    else if(!strcmp(argv[3],"recorded")&&argc==7)recorded(argv[4],argv[5],argv[6]);
    else if(!strcmp(argv[3],"shadow")&&argc>=5)shadow(argv[4],argc>5&&!strcmp(argv[5],"prior"));
    else if(!strcmp(argv[3],"plan"))plan_mode(argc>4&&!strcmp(argv[4],"prior"));
    else if(!strcmp(argv[3],"candidate")&&argc>=7)candidate(strtod(argv[4],NULL),strtod(argv[5],NULL),strtod(argv[6],NULL),argc>7&&!strcmp(argv[7],"prior"));
    else fail("invalid mode/arguments");free(tokens);free(text);return 0;
}
