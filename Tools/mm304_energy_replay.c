/* Offline recorded-state audit. Reads one frozen JSON fixture; no transport. */
#include "landing.h"
#include "../CLanding/guidance/guidance_internal.h"
#include "decision_envelope.h"
#include "terminal_model.h"
#include "mm305_planning.h"
#include "taem_geometry.h"
#include "sim_telemetry.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
static double number(const JsonDoc*d,int o,const char*k){return json_number(d,json_object_get(d,o,k),NAN);}
static Vector3 vector(const JsonDoc*d,int o,const char*k){int a=json_object_get(d,o,k);return v3(json_number(d,json_array_get(d,a,0),NAN),json_number(d,json_array_get(d,a,1),NAN),json_number(d,json_array_get(d,a,2),NAN));}
int main(int argc,char**argv){
    if(argc<2){fprintf(stderr,"usage: mm304_energy_replay FIXTURE.json [--routes]\n");return 2;}
    setvbuf(stdout,NULL,_IOLBF,0);
    bool routes=argc>2&&!strcmp(argv[2],"--routes");
    FILE*f=fopen(argv[1],"rb");if(!f)return 2;
    if(fseek(f,0,SEEK_END)){fclose(f);return 2;}long length=ftell(f);rewind(f);
    char*buf=calloc((size_t)length+1,1);JsonToken*tokens=calloc(32768,sizeof(*tokens));
    if(!buf||!tokens||fread(buf,1,(size_t)length,f)!=(size_t)length)return 2;fclose(f);
    JsonDoc doc;if(json_parse(buf,tokens,32768,&doc)<0)return 2;
    LandingConfiguration cfg=landing_configuration_default();
    if(!landing_configuration_from_json(&cfg,&doc,json_object_get(&doc,0,"configuration")))return 2;
    PlanetModel p={0};int pl=json_object_get(&doc,0,"planet");
    p.radius=number(&doc,pl,"radius");p.gravitational_parameter=number(&doc,pl,"gravitationalParameter");
    p.rotational_speed=number(&doc,pl,"rotationalSpeed");p.atmosphere_depth=number(&doc,pl,"atmosphereDepth");
    p.atmosphere_adiabatic_index=number(&doc,pl,"atmosphereAdiabaticIndex");
    p.north_axis=vector(&doc,pl,"northAxis");p.prime_meridian_at_epoch=vector(&doc,pl,"primeMeridianAtEpoch");p.epoch_ut=number(&doc,pl,"epochUT");
    int a=json_object_get(&doc,pl,"atmosphereSamples");
    for(int i=0;;i++){int row=json_array_get(&doc,a,i);if(row<0)break;if(p.atmosphere_sample_count>=LANDER_ATMOSPHERE_SAMPLE_MAX)return 2;
        size_t n=p.atmosphere_sample_count++;p.atmosphere_altitude[n]=number(&doc,row,"altitude");p.atmosphere_density[n]=number(&doc,row,"density");p.atmosphere_pressure[n]=number(&doc,row,"pressure");}
    char paths[4][512],reason[256];TerminalModelSourceFiles files={
        shuttle_sim_model_path(NULL,SHUTTLE_SIM_MODEL_ATMOSPHERE,paths[0],512),
        shuttle_sim_model_path(NULL,SHUTTLE_SIM_MODEL_AERO,paths[1],512),
        shuttle_sim_model_path(NULL,SHUTTLE_SIM_MODEL_FORCE_BOOK,paths[2],512),
        shuttle_sim_model_path(NULL,SHUTTLE_SIM_MODEL_ATTITUDE,paths[3],512)};
    TerminalModel*model=malloc(sizeof(*model));if(!model||!terminal_model_capture(model,&files,&cfg,100,reason,sizeof(reason)))return 2;
    model->world.rotation_phase_rad_at_ut0=atan2(p.prime_meridian_at_epoch.y,p.prime_meridian_at_epoch.x)-p.rotational_speed*p.epoch_ut;
    printf("model=%s aero=%s book=%s routes=%d\n",shuttle_sim_model_profile(),files.aero_csv,files.aero_book_csv,routes);
    int samples=json_object_get(&doc,0,"samples");
    for(int i=0;;i++){
        int row=json_array_get(&doc,samples,i);if(row<0)break;int fields=json_object_get(&doc,row,"fields");
        int pos=json_object_get(&doc,fields,"position"),motion=json_object_get(&doc,fields,"motion"),att=json_object_get(&doc,fields,"attitude"),aero=json_object_get(&doc,fields,"aero"),veh=json_object_get(&doc,fields,"vehicle"),guid=json_object_get(&doc,fields,"guidance"),phys=json_object_get(&doc,fields,"physics");
        Telemetry t={0};t.ut=number(&doc,row,"ut");t.latitude=number(&doc,pos,"latitude");t.longitude=number(&doc,pos,"longitude");t.mean_altitude=number(&doc,pos,"altitude");t.radar_altitude=number(&doc,pos,"radarAltitude");
        t.true_air_speed=number(&doc,motion,"trueAirSpeed");t.horizontal_speed=number(&doc,motion,"horizontalSpeed");t.vertical_speed=number(&doc,motion,"verticalSpeed");t.flight_path_angle=number(&doc,motion,"flightPathAngle");
        t.angle_of_attack=number(&doc,att,"angleOfAttack");t.sideslip=number(&doc,att,"sideslip");t.roll=number(&doc,att,"roll");t.roll_rate=number(&doc,att,"rollRate");t.angle_of_attack_rate=number(&doc,att,"angleOfAttackRate");t.has_angle_of_attack_rate=isfinite(t.angle_of_attack_rate);
        t.dynamic_pressure=number(&doc,aero,"dynamicPressure");t.mach=number(&doc,aero,"mach");t.lift_force=number(&doc,aero,"liftForce");t.drag_force=number(&doc,aero,"dragForce");t.g_force=number(&doc,aero,"gForce");t.mass=number(&doc,veh,"mass");t.physics_sample_valid=true;
        t.runway_along_track=number(&doc,guid,"runwayAlongTrack");t.runway_cross_track=number(&doc,guid,"runwayCrossTrack");t.range_to_site=number(&doc,guid,"rangeToSite");
        t.physics_certified_uncertainty=number(&doc,phys,"certifiedUncertainty");
        t.aerodynamic_confidence=number(&doc,phys,"confidence");
        t.physics_model_residual=number(&doc,phys,"modelResidual");t.physics_model_residual_confidence=number(&doc,phys,"modelResidualConfidence");
        t.has_torque=true;t.has_inertia=true;Vector3 torque=vector(&doc,veh,"availableTorque"),inertia=vector(&doc,veh,"momentOfInertia");t.available_pitch_torque=torque.x;t.pitch_moment_of_inertia=inertia.x;
        t.estimated_ballistic_coefficient=cfg.vehicle.estimated_ballistic_coefficient;t.estimated_lift_to_drag=cfg.vehicle.estimated_lift_to_drag;t.bank_effectiveness=1.0;
        VehicleState state={t.ut,vector(&doc,pos,"inertial"),vector(&doc,motion,"velocity"),t.mass};
        Vector3 air=vsub(state.velocity,vcross(planet_rotation_vector(&p),state.position));
        Vector3 up=vnorm(state.position,v3(0,0,1)),north=vnorm(vproject_plane(p.north_axis,up),v3(1,0,0)),east=vnorm(vcross(north,up),v3(0,1,0));
        t.ground_track_heading=norm_deg(atan2(vdot(air,east),vdot(air,north))*RAD2DEG);
        GuidanceMachine g={0};g.entry_terminal_model=model;
        double final_distance=final_alignment_distance_for_state(&g,t.mean_altitude,t.true_air_speed,&cfg.site,&cfg.vehicle,&cfg.guidance);
        FinalAlignmentContract alignment=final_alignment_contract(final_distance,&cfg.site,&cfg.vehicle,&cfg.guidance);
        double path=hypot(t.runway_along_track+alignment.distance_m,t.runway_cross_track);
        double height=alignment.altitude_m;
        UnpoweredPathProjection u=decision_unpowered_path_projection(&g,&t,&p,&cfg,height,path);
        TaemInterfaceCapture capture=entry_mm305_admission_envelope(&g,&t,t.ground_track_heading,&p,&cfg);
        double ml,md,raw_lq,raw_dq;double incidence=hypot(t.angle_of_attack,t.sideslip);
        aero_force_per_q(&model->aero,t.dynamic_pressure,t.mach,incidence,&raw_lq,&raw_dq);
        bool forces=decision_unpowered_aerodynamic_accelerations(&g,&t,&p,&cfg,t.mean_altitude,t.true_air_speed,incidence,&ml,&md);
        printf("sample=%d ut=%.6f h=%.3f M=%.6f along=%.3f cross=%.3f measured_D=%.6f measured_LD=%.6f raw_D=%.6f raw_LD=%.6f corrected_D=%.6f corrected_LD=%.6f valid=%d reference_reachable=%d range=%.3f required=%.3f work=%.3f Vterminal=%.6f numerical_error=%.6f Eavail=%.3f margin=%.3f uncertainty=%.3f admission=%d veto=%u\n",i,t.ut,t.mean_altitude,t.mach,t.runway_along_track,t.runway_cross_track,t.drag_force/t.mass,t.lift_force/t.drag_force,t.dynamic_pressure*raw_dq/t.mass,raw_lq/raw_dq,forces?md:NAN,forces?ml/md:NAN,u.valid,u.reference_reachable,u.projected_ground_path_m,path,u.modeled_drag_work,u.terminal_speed_mps,u.integration_error_j_kg,capture.energy_available,capture.energy_margin,capture.energy_uncertainty,capture.ready,capture.veto);
        GuidanceMachine entry=g;
        entry.entry_exec.initialized=true;entry.entry_exec.phase=ENTRY_PHASE_TRANSITION;
        entry.s_turn_sign=t.roll>=0.0?1.0:-1.0;
        t.pitch=number(&doc,att,"pitch");t.heading=number(&doc,att,"heading");
        t.bearing_to_site=number(&doc,guid,"bearingToSite");
        t.course_to_site_error=norm_signed_deg(t.bearing_to_site-t.ground_track_heading);
        AerodynamicModel prior={cfg.vehicle.estimated_lift_to_drag,cfg.vehicle.estimated_ballistic_coefficient,0.0};
        GuidanceResult command=entry_program_guidance(&entry,&t,&state,t.ground_track_heading,&p,prior,&cfg,0.1);
        printf("entry sample=%d valid=%d Rrequired=%.3f Rpredicted=%.3f Dref=%.6f bank=%.6f targetAoA=%.6f energy_budget=%.3f complete=%d\n",
            i,entry.entry_feedback.valid,entry.entry_feedback.required_range_to_go,
            entry.entry_feedback.predicted_achievable_range,entry.entry_feedback.reference_drag_accel,
            command.command.target_roll,command.command.target_aoa,entry.entry_control_taem_energy_error,entry.entry_exec.entry_complete);
        guidance_result_clear(&command);
#ifdef BASELINE_COMPARISON
        extern GuidanceResult baseline_entry_program_guidance(GuidanceMachine*,const Telemetry*,const VehicleState*,double,const PlanetModel*,AerodynamicModel,const LandingConfiguration*,double);
        GuidanceMachine old=g;old.entry_exec.initialized=true;old.entry_exec.phase=ENTRY_PHASE_TRANSITION;old.s_turn_sign=entry.s_turn_sign;
        GuidanceResult previous=baseline_entry_program_guidance(&old,&t,&state,t.ground_track_heading,&p,prior,&cfg,0.1);
        printf("baseline_entry sample=%d valid=%d Rrequired=%.3f Rpredicted=%.3f Dref=%.6f energy_budget=%.3f\n",i,old.entry_feedback.valid,old.entry_feedback.required_range_to_go,old.entry_feedback.predicted_achievable_range,old.entry_feedback.reference_drag_accel,old.entry_control_taem_energy_error);
        guidance_result_clear(&previous);
#endif
        if(routes){
            TerminalDynamicState dynamic={0};dynamic.ut_s=t.ut;dynamic.mass_kg=t.mass;dynamic.position_i_m=(Vec3){state.position.x,state.position.y,state.position.z};dynamic.velocity_i_mps=(Vec3){state.velocity.x,state.velocity.y,state.velocity.z};dynamic.attitude=model->attitude;
            dynamic.attitude.aoa_rad=dynamic.attitude.cmd_aoa_rad=dynamic.attitude.requested_aoa_rad=t.angle_of_attack*DEG2RAD;
            dynamic.attitude.bank_rad=dynamic.attitude.cmd_bank_rad=dynamic.attitude.requested_bank_rad=t.roll*DEG2RAD;
            Mm305PlanRequest request={.valid=true,.model_snapshot_id=model->snapshot_id,.state=dynamic,.request_ut=t.ut,.hac_radius_m=cfg.guidance.hac_radius,.scale_mach=t.mach,.lift_scale=t.lift_force/(t.dynamic_pressure*raw_lq),.drag_scale=t.drag_force/(t.dynamic_pressure*raw_dq)};
            Mm305PlanResult result=mm305_plan(model,&request);printf("route sample=%d valid=%d found=%d diagnostic=%s\n",i,result.valid,result.found,result.diagnostic);
        }
    }
    free(model);free(tokens);free(buf);return 0;
}
