#include "../CLanding/planner_log.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

static EntryControlPlan test_plan(double bank){
    EntryControlPlan p;memset(&p,0,sizeof(p));p.valid=true;p.plan_id=42;p.plan_version=3;p.parent_plan_id=41;p.parent_plan_version=2;
    p.planned_ut=100;p.target_bank=bank;p.target_aoa=23;
    p.target_heading=90;p.bank_cap=fabs(bank);p.target_turn_radius=50000;p.segment_duration=40;p.cost=2;
    p.taem_range_error=1000;p.taem_speed=500;p.taem_energy_error=5000;p.closest_distance=30000;
    p.has_planned_reversal=true;p.planned_reversal_ut=140;p.planned_reversal_range=150000;p.planned_reversal_sign=-1;p.predicted_reversals=1;
    return p;
}

static void test_policy_rate_forcing_and_lineage(void){
    PlannerLogPolicyState state;planner_log_policy_init(&state);EntryControlPlan p=test_plan(20);
    PlannerLogDecision d=planner_log_policy_step(&state,100,PHASE_ENTRY_ENERGY,&p,false,false,false);
    assert(d.emit_scalar&&d.emit_trajectory&&d.forced&&d.durable&&d.executable_lineage);
    assert(d.plan_id==42&&d.plan_version==3&&d.parent_plan_id==41&&d.parent_plan_version==2);
    d=planner_log_policy_step(&state,100.5,PHASE_ENTRY_ENERGY,&p,false,false,false);
    assert(!d.emit_scalar&&!d.emit_trajectory&&!d.forced&&!d.durable&&d.executable_lineage&&d.plan_id==42&&d.plan_version==3);
    p.target_bank=21;p.parent_plan_id=42;p.parent_plan_version=3;p.plan_version=4;
    d=planner_log_policy_step(&state,100.6,PHASE_ENTRY_ENERGY,&p,false,false,false);
    assert(d.plan_changed&&d.emit_scalar&&d.emit_trajectory&&d.durable&&d.executable_lineage);
    assert(d.plan_id==42&&d.plan_version==4&&d.parent_plan_id==42&&d.parent_plan_version==3);
    d=planner_log_policy_step(&state,102.7,PHASE_ENTRY_ENERGY,&p,false,false,false);
    assert(d.emit_scalar&&!d.emit_trajectory&&!d.forced&&!d.durable);
    d=planner_log_policy_step(&state,103.0,PHASE_TAEM,&p,false,false,false);
    assert(d.phase_changed&&d.emit_scalar&&d.emit_trajectory&&d.durable&&d.plan_id==42&&d.plan_version==4);
    d=planner_log_policy_step(&state,103.2,PHASE_TAEM,&p,true,false,false);
    assert(d.constraint_changed&&d.emit_scalar&&d.emit_trajectory&&!d.durable);
    d=planner_log_policy_step(&state,103.3,PHASE_TAEM,&p,true,false,true);
    assert(d.forced&&d.durable&&d.emit_scalar&&d.emit_trajectory&&(d.event_flags&PLANNER_LOG_EVENT_EXPLICIT));
    d=planner_log_policy_step(&state,103.4,PHASE_TAEM,&p,true,true,false);
    assert(d.predictor_discontinuity&&d.emit_scalar&&d.emit_trajectory&&!d.durable);
    d=planner_log_policy_step(&state,90.0,PHASE_TAEM,&p,true,false,false);
    assert(d.time_rewind&&d.emit_scalar&&d.emit_trajectory&&d.durable&&d.plan_id==42&&d.plan_version==4);
}

static void test_legacy_lineage_fallback_and_no_plan(void){
    PlannerLogPolicyState state;planner_log_policy_init(&state);EntryControlPlan p=test_plan(18);
    p.plan_id=p.plan_version=p.parent_plan_id=p.parent_plan_version=0;
    PlannerLogDecision d=planner_log_policy_step(&state,300,PHASE_ENTRY_ENERGY,&p,false,false,false);
    assert(!d.executable_lineage&&d.plan_id==1&&d.plan_version==1);
    p.target_bank=19;d=planner_log_policy_step(&state,300.5,PHASE_ENTRY_ENERGY,&p,false,false,false);
    assert(d.plan_changed&&d.plan_id==1&&d.plan_version==2);
    d=planner_log_policy_step(&state,301,PHASE_ENTRY_ENERGY,NULL,false,false,false);
    assert(d.plan_changed&&d.plan_id==0&&d.plan_version==0&&!d.executable_lineage);
}

static void test_nan_fields_do_not_create_fake_replans(void){
    PlannerLogPolicyState state;planner_log_policy_init(&state);EntryControlPlan p=test_plan(18);
    p.taem_range_error=NAN;p.taem_speed=NAN;p.taem_energy_error=NAN;p.target_turn_radius=INFINITY;
    (void)planner_log_policy_step(&state,200,PHASE_ENTRY_ENERGY,&p,false,false,false);
    PlannerLogDecision d=planner_log_policy_step(&state,200.4,PHASE_ENTRY_ENERGY,&p,false,false,false);
    assert(!d.plan_changed&&!d.emit_scalar&&!d.forced);
}

static void test_writer_rotation_and_correlation(void){
    char directory[]="/tmp/ksp-planner-log-test-XXXXXX";assert(mkdtemp(directory));
    LandingConfiguration cfg=landing_configuration_default();landing_configuration_normalize(&cfg);
    char vehicle_peer[1800];snprintf(vehicle_peer,sizeof(vehicle_peer),"%s/test-STS-N-vehicle.jsonl",directory);
    assert(setenv("KSP_LANDER_CAMPAIGN_IDENTITY","campaign-test-identity",1)==0);
    PlannerLog *log=planner_log_open(directory,"test","STS-N","session-test","native-build-test",vehicle_peer,true,&cfg,65536);assert(log);
    EntryControlPlan p=test_plan(20);PredictorPlannerTrace trace;memset(&trace,0,sizeof(trace));
    trace.valid=true;trace.mode=ENTRY_SUPERVISION_PASS_THROUGH;trace.candidate_count=1;trace.candidate_budget=PREDICTOR_PLANNER_TRACE_MAX_CANDIDATES;
    trace.producing_tick_sequence=17;trace.accepted_tick_sequence=19;
    trace.search_horizon_seconds=500;trace.maximum_bank_correction=8;trace.maximum_aoa_correction=3;trace.selection_converged=true;trace.selected_index=0;
    trace.candidates[0].selected=true;trace.candidates[0].plan=p;trace.candidates[0].trajectory_total_points=10;trace.candidates[0].trajectory_sample_count=2;trace.candidates[0].trajectory_truncated=true;
    trace.candidates[0].trajectory_points[0]=(TrajectoryPoint){100,0,0,40000,1200,PHASE_ENTRY_ENERGY,TRAJ_PLANNED};
    trace.candidates[0].trajectory_points[1]=(TrajectoryPoint){160,0.1,0.2,20000,700,PHASE_TAEM,TRAJ_PLANNED};
    TrajectoryCalibrationModel calibration={.density_scale=1,.drag_scale=1.02,.lift_scale=.98,.bank_effectiveness=.97,.speed_of_sound=340,.speed_of_sound_scale=1,.confidence=.8,.accepted_samples=17};
    AerodynamicEnvelope envelope={0};for(int i=0;i<4;i++)envelope.regimes[i]=(AerodynamicModel){1.2+i*.1,420+i*10,.7};
    PlannerLogSample sample;memset(&sample,0,sizeof(sample));sample.phase=PHASE_ENTRY_ENERGY;sample.plan=&p;sample.trace=&trace;sample.calibration=&calibration;sample.envelope=&envelope;
    for(unsigned i=0;i<220;i++){sample.tick_sequence=i+1;sample.ut=100+i*2.1;PlannerLogDecision d=planner_log_record(log,&sample);assert(d.emit_scalar);}
    assert(planner_log_segment_index(log)>0);assert(planner_log_segment_bytes(log)<=65536);
    char first[1800];snprintf(first,sizeof(first),"%s/test-STS-N-planner.jsonl",directory);FILE*f=fopen(first,"r");assert(f);
    char *buf=malloc(65537);assert(buf);size_t n=fread(buf,1,65536,f);buf[n]=0;fclose(f);
    assert(strstr(buf,"\"schemaVersion\":5"));assert(strstr(buf,"\"sessionSchemaVersion\":1"));assert(strstr(buf,"\"startedUTC\":\"test\""));assert(strstr(buf,"\"sessionId\":\"session-test\""));assert(strstr(buf,"\"vessel\":\"STS-N\""));
    assert(strstr(buf,"\"peer\":{\"stream\":\"vehicle\",\"available\":true"));assert(strstr(buf,vehicle_peer));assert(strstr(buf,"\"nativeBuild\":\"native-build-test\""));assert(strstr(buf,"\"vehicleModelId\""));
    assert(strstr(buf,"\"campaignIdentity\":\"campaign-test-identity\""));
    assert(strstr(buf,"\"tickSequence\":1"));assert(strstr(buf,"\"durable\":true"));
    assert(strstr(buf,"\"planId\":42"));assert(strstr(buf,"\"lineageSource\":\"executablePlan\""));
    assert(strstr(buf,"\"producingTickSequence\":17"));assert(strstr(buf,"\"acceptedTickSequence\":19"));
    assert(strstr(buf,"\"candidatePath\""));assert(strstr(buf,"\"candidateBudget\":48"));assert(strstr(buf,"\"selectionConverged\":true"));
    assert(strstr(buf,"\"physicsSampleCount\":0"));assert(strstr(buf,"\"calibrationAcceptedSamples\":17"));free(buf);
    unsigned last=planner_log_segment_index(log);planner_log_close(log);
    assert(unsetenv("KSP_LANDER_CAMPAIGN_IDENTITY")==0);
    for(unsigned i=0;i<=last;i++){char path[1800];if(i==0)snprintf(path,sizeof(path),"%s/test-STS-N-planner.jsonl",directory);else snprintf(path,sizeof(path),"%s/test-STS-N-planner-%03u.jsonl",directory,i);(void)unlink(path);}assert(rmdir(directory)==0);
}

static void test_missing_vehicle_peer_is_declared(void){
    char directory[]="/tmp/ksp-planner-peer-test-XXXXXX";assert(mkdtemp(directory));
    LandingConfiguration cfg=landing_configuration_default();landing_configuration_normalize(&cfg);
    char vehicle_peer[1800];snprintf(vehicle_peer,sizeof(vehicle_peer),"%s/test-STS-N-vehicle.jsonl",directory);
    PlannerLog *log=planner_log_open(directory,"test","STS-N","session-missing-peer","native-build-test",vehicle_peer,false,&cfg,65536);assert(log);
    char planner_path[1800];snprintf(planner_path,sizeof(planner_path),"%s/test-STS-N-planner.jsonl",directory);
    FILE*f=fopen(planner_path,"r");assert(f);char buf[8192];size_t n=fread(buf,1,sizeof(buf)-1,f);buf[n]=0;fclose(f);
    assert(strstr(buf,"\"sessionId\":\"session-missing-peer\""));
    assert(strstr(buf,"\"peer\":{\"stream\":\"vehicle\",\"available\":false"));
    assert(strstr(buf,vehicle_peer));
    planner_log_close(log);assert(unlink(planner_path)==0);assert(rmdir(directory)==0);
}


int main(void){test_policy_rate_forcing_and_lineage();test_legacy_lineage_fallback_and_no_plan();test_nan_fields_do_not_create_fake_replans();test_writer_rotation_and_correlation();test_missing_vehicle_peer_is_declared();puts("Planner log tests passed.");return 0;}
