#include "flight_log_codec.h"

#include <assert.h>
#include <stddef.h>
#include <stdio.h>
#include <string.h>

static VehicleLogSample sample_at(double wall,uint64_t tick){
    VehicleLogSample s={0};
    s.tick_sequence=tick;
    s.wall_monotonic_seconds=wall;
    s.ut=67000.0+wall;
    s.latitude=-.1;
    s.longitude=-74.6;
    s.altitude=75000.0;
    s.radar_altitude=74900.0;
    s.position[0]=1000.0;s.position[1]=2000.0;s.position[2]=3000.0;
    s.velocity[0]=-200.0;s.velocity[1]=50.0;s.velocity[2]=-20.0;
    s.true_air_speed=2200.0;
    s.horizontal_speed=2195.0;
    s.vertical_speed=-120.0;
    s.flight_path_angle=-3.1;
    s.pitch=34.0;s.roll=-25.0;s.heading=88.0;s.angle_of_attack=18.0;s.sideslip=.2;
    s.pitch_rate=3.0;s.roll_rate=-4.0;s.yaw_rate=.5;s.course_rate=.3;
    s.coordinate_pitch_rate=1.2;s.coordinate_roll_rate=-2.3;s.heading_rate=.4;s.angle_of_attack_rate=-1.1;
    s.body_pitch_rate=3.0;s.body_roll_rate=-4.0;s.body_yaw_rate=.5;
    s.dynamic_pressure=30.0;s.static_pressure=8.0;s.atmospheric_density=.00002;s.mach=6.8;s.stall_fraction=.06;s.stall_fraction_is_measured=false;
    s.lift_force=120000.0;s.drag_force=45000.0;s.g_force=.7;s.energy_excess_range=120000.0;
    s.has_force_vectors=true;s.lift_vector[0]=120000.0;s.drag_vector[1]=-45000.0;s.aero_acceleration[0]=120000.0/51000.0;s.aero_acceleration[1]=-45000.0/51000.0;
    s.physics_confidence=.72;s.physics_model_residual=.18;s.physics_model_residual_confidence=.61;s.physics_certified_uncertainty=.34;
    s.physics_force_residual_per_q[0]=-.0012;s.physics_force_residual_per_q[1]=.0004;s.physics_force_residual_per_q[2]=.0001;
    s.physics_force_residual_sigma_per_q[0]=.0002;s.physics_force_residual_sigma_per_q[1]=.0001;s.physics_force_residual_sigma_per_q[2]=.00005;
    s.physics_force_residual_confidence=.55;s.physics_airbrake_model_available=true;s.physics_airbrake_model_confidence=.8;s.physics_airbrake_drag_accel=4.75;s.physics_samples=192;s.physics_live_samples=12;
    s.mass=51000.0;s.available_thrust=0;s.current_thrust=0;
    s.has_center_of_mass=true;s.center_of_mass[0]=.1;s.center_of_mass[1]=-.2;s.center_of_mass[2]=.3;
    s.target_pitch=18.0;s.target_heading=90.0;s.target_roll=-30.0;s.has_target_aoa=true;s.target_aoa=18.0;
    s.target_gear=false;s.target_brakes=false;s.target_airbrakes=false;
    s.gear=false;s.brakes=false;s.has_airbrakes=true;s.airbrakes=false;
    s.has_control_state=true;s.control_state_pitch=.11;s.control_state_roll=-.24;s.control_state_yaw=.01;
    s.command_applied=true;s.has_actuator_feedback=true;
    s.control_pitch=.12;s.control_roll=-.25;s.control_yaw=.01;
    s.has_loop_wall_delta=true;s.has_telemetry_latency=true;s.has_guidance_compute=true;s.has_apply_latency=true;s.has_control_loop=true;
    s.loop_wall_delta_ms=100.0;s.telemetry_latency_ms=18.0;s.guidance_compute_ms=12.5;s.apply_latency_ms=6.0;s.control_loop_ms=38.0;
    s.automation_engaged=true;s.constraint_active=false;
    snprintf(s.phase,sizeof(s.phase),"Entry Energy");
    snprintf(s.control_profile,sizeof(s.control_profile),"Entry");
    snprintf(s.vessel_situation,sizeof(s.vessel_situation),"FLYING");
    s.range_to_site=400000.0;s.bearing_to_site=92.0;s.runway_along_track=390000.0;s.runway_cross_track=20000.0;s.specific_mechanical_energy=-1500000.0;
    s.entry_plan_valid=true;s.entry_reversal_scheduled=true;s.entry_reversal_time_remaining=30.0;s.entry_reversal_ut=630.0;s.entry_reversal_range=180000.0;s.entry_reversal_sign=1.0;
    s.has_plan_identity=true;s.plan_id=7;s.plan_version=3;s.parent_plan_id=6;s.parent_plan_version=2;
    return s;
}

static void test_first_and_unchanged(void){
    VehicleLogEncoder e;vehicle_log_encoder_init(&e);
    VehicleLogSample s=sample_at(100.0,1);
    VehicleLogDecision d=vehicle_log_decide(&e,&s,false,false);
    assert(d.emit&&d.keyframe&&d.groups==VEHICLE_LOG_GROUP_ALL);
    vehicle_log_commit(&e,&s,&d);
    s.wall_monotonic_seconds=100.04;s.ut+=.04;s.tick_sequence=2;
    d=vehicle_log_decide(&e,&s,false,false);
    assert(!d.emit&&d.groups==0);
}

static void test_deadband_and_delta(void){
    VehicleLogEncoder e;vehicle_log_encoder_init(&e);
    VehicleLogSample s=sample_at(200.0,1);
    VehicleLogDecision d=vehicle_log_decide(&e,&s,false,false);vehicle_log_commit(&e,&s,&d);
    s.wall_monotonic_seconds=200.11;s.ut+=.11;s.tick_sequence=2;s.altitude+=.05;
    d=vehicle_log_decide(&e,&s,false,false);assert(!d.emit);
    s.altitude+=2.0;s.position[0]+=2.0;
    d=vehicle_log_decide(&e,&s,false,false);
    assert(d.emit&&!d.keyframe&&(d.groups&VEHICLE_LOG_GROUP_POSITION));
}

static void test_semantic_event_forcing(void){
    VehicleLogEncoder e;vehicle_log_encoder_init(&e);
    VehicleLogSample s=sample_at(300.0,1);
    VehicleLogDecision d=vehicle_log_decide(&e,&s,false,false);vehicle_log_commit(&e,&s,&d);
    s.wall_monotonic_seconds=300.01;s.ut+=.01;s.tick_sequence=2;
    snprintf(s.phase,sizeof(s.phase),"TAEM");
    snprintf(s.event,sizeof(s.event),"event=phase-change");
    d=vehicle_log_decide(&e,&s,true,false);
    assert(d.emit&&!d.keyframe&&d.durable&&(d.groups&VEHICLE_LOG_GROUP_STATE));
}

static void test_plan_identity_forcing(void){
    VehicleLogEncoder e;vehicle_log_encoder_init(&e);
    VehicleLogSample s=sample_at(325.0,1);
    VehicleLogDecision d=vehicle_log_decide(&e,&s,false,false);vehicle_log_commit(&e,&s,&d);
    s.wall_monotonic_seconds=325.01;s.ut+=.01;s.tick_sequence=2;s.plan_id=8;s.plan_version=1;s.parent_plan_id=7;s.parent_plan_version=3;
    d=vehicle_log_decide(&e,&s,false,false);
    assert(d.emit&&!d.keyframe&&d.durable&&(d.groups&VEHICLE_LOG_GROUP_STATE));
    vehicle_log_commit(&e,&s,&d);
    s.wall_monotonic_seconds=325.02;s.ut+=.01;s.tick_sequence=3;s.parent_plan_version=4;
    d=vehicle_log_decide(&e,&s,false,false);
    assert(d.emit&&!d.keyframe&&d.durable&&(d.groups&VEHICLE_LOG_GROUP_STATE));
}

static void test_constraint_and_touchdown_forcing(void){
    VehicleLogEncoder e;vehicle_log_encoder_init(&e);
    VehicleLogSample s=sample_at(350.0,1);
    VehicleLogDecision d=vehicle_log_decide(&e,&s,false,false);vehicle_log_commit(&e,&s,&d);
    s.wall_monotonic_seconds=350.01;s.ut+=.01;s.tick_sequence=2;s.constraint_active=true;
    d=vehicle_log_decide(&e,&s,false,false);
    assert(d.emit&&d.durable&&(d.groups&VEHICLE_LOG_GROUP_STATE));vehicle_log_commit(&e,&s,&d);
    s.wall_monotonic_seconds=350.02;s.ut+=.01;s.tick_sequence=3;snprintf(s.vessel_situation,sizeof(s.vessel_situation),"LANDED");
    d=vehicle_log_decide(&e,&s,false,false);
    assert(d.emit&&d.durable&&(d.groups&VEHICLE_LOG_GROUP_STATE));
}

static void test_periodic_keyframe(void){
    VehicleLogEncoder e;vehicle_log_encoder_init(&e);
    VehicleLogSample s=sample_at(400.0,1);
    VehicleLogDecision d=vehicle_log_decide(&e,&s,false,false);vehicle_log_commit(&e,&s,&d);
    s.wall_monotonic_seconds=405.001;s.ut+=5.001;s.tick_sequence=2;
    d=vehicle_log_decide(&e,&s,false,false);
    assert(d.emit&&d.keyframe&&d.groups==VEHICLE_LOG_GROUP_ALL);
}

static void test_rapid_transient_rate(void){
    VehicleLogEncoder e;vehicle_log_encoder_init(&e);
    VehicleLogSample s=sample_at(500.0,1);
    VehicleLogDecision d=vehicle_log_decide(&e,&s,false,false);vehicle_log_commit(&e,&s,&d);
    s.wall_monotonic_seconds=500.051;s.ut+=.051;s.tick_sequence=2;
    s.roll_rate=8.0;s.roll+=1.0;
    d=vehicle_log_decide(&e,&s,false,false);
    assert(d.emit&&!d.keyframe&&(d.groups&VEHICLE_LOG_GROUP_ATTITUDE));
}

static void test_json_separation_and_identity(void){
    VehicleLogEncoder e;vehicle_log_encoder_init(&e);
    VehicleLogSample s=sample_at(600.0,42);
    VehicleLogDecision d=vehicle_log_decide(&e,&s,false,false);
    JsonWriter w;jw_init(&w);vehicle_log_record_json(&w,"session-abc",7,&s,&d);
    assert(!w.failed&&w.data);
    assert(strstr(w.data,"\"schemaVersion\":4"));
    assert(strstr(w.data,"\"recordType\":\"vehicleKeyframe\""));
    assert(strstr(w.data,"\"sessionId\":\"session-abc\""));
    assert(strstr(w.data,"\"recordSequence\":7"));
    assert(strstr(w.data,"\"tickSequence\":42"));
    assert(strstr(w.data,"\"aeroAcceleration\""));
    assert(strstr(w.data,"\"reportedPitch\""));
    assert(strstr(w.data,"\"stallFraction\""));
    assert(strstr(w.data,"\"stallFractionMeasured\":false"));
    assert(strstr(w.data,"\"physics\""));
    assert(strstr(w.data,"\"forceResidualConfidence\":0.55"));
    assert(strstr(w.data,"\"airbrakeModelAvailable\":true"));
    assert(strstr(w.data,"\"airbrakeModelConfidence\":0.8"));
    assert(strstr(w.data,"\"airbrakeDragAccel\":4.75"));
    assert(strstr(w.data,"\"liveSamples\":12"));
    assert(strstr(w.data,"\"specificMechanicalEnergy\""));
    assert(strstr(w.data,"\"entryReversalUT\""));
    assert(strstr(w.data,"\"entryReversalSign\""));
    assert(strstr(w.data,"\"coordinateRollRate\":-2.3"));
    assert(strstr(w.data,"\"angleOfAttackRate\":-1.1"));
    assert(strstr(w.data,"\"bodyRollRate\":-4"));
    assert(strstr(w.data,"\"planId\":7"));
    assert(strstr(w.data,"\"planVersion\":3"));
    assert(strstr(w.data,"\"parentPlanId\":6"));
    assert(strstr(w.data,"\"parentPlanVersion\":2"));
    assert(strstr(w.data,"\"runtime\""));
    assert(strstr(w.data,"\"loopWallDeltaMilliseconds\":100"));
    assert(strstr(w.data,"\"guidanceComputeMilliseconds\":12.5"));
    assert(strstr(w.data,"\"applyLatencyMilliseconds\":6"));
    assert(strstr(w.data,"\"controlLoopMilliseconds\":38"));
    assert(!strstr(w.data,"trajectory"));
    assert(!strstr(w.data,"predictedTrajectory"));
    jw_free(&w);
}

static void replay_vehicle_record(VehicleLogSample *dst,const VehicleLogSample *src,const VehicleLogDecision *d){
    assert(dst&&src&&d&&d->emit);
    if(d->keyframe){*dst=*src;return;}
    dst->tick_sequence=src->tick_sequence;dst->wall_monotonic_seconds=src->wall_monotonic_seconds;dst->ut=src->ut;memcpy(dst->event,src->event,sizeof(dst->event));
#define COPY_RANGE(first,last) memcpy((char*)dst+offsetof(VehicleLogSample,first),(const char*)src+offsetof(VehicleLogSample,first),offsetof(VehicleLogSample,last)+sizeof(src->last)-offsetof(VehicleLogSample,first))
    if(d->groups&VEHICLE_LOG_GROUP_STATE){COPY_RANGE(gear,airbrakes);COPY_RANGE(automation_engaged,vessel_situation);dst->has_plan_identity=src->has_plan_identity;dst->plan_id=src->plan_id;dst->plan_version=src->plan_version;dst->parent_plan_id=src->parent_plan_id;dst->parent_plan_version=src->parent_plan_version;dst->entry_plan_valid=src->entry_plan_valid;dst->entry_reversal_scheduled=src->entry_reversal_scheduled;}
    if(d->groups&VEHICLE_LOG_GROUP_POSITION)COPY_RANGE(latitude,position);
    if(d->groups&VEHICLE_LOG_GROUP_MOTION)COPY_RANGE(velocity,flight_path_angle);
    if(d->groups&VEHICLE_LOG_GROUP_ATTITUDE)COPY_RANGE(pitch,course_rate);
    if(d->groups&VEHICLE_LOG_GROUP_AERO)COPY_RANGE(dynamic_pressure,aero_acceleration);
    if(d->groups&VEHICLE_LOG_GROUP_PHYSICS)COPY_RANGE(physics_confidence,physics_live_samples);
    if(d->groups&VEHICLE_LOG_GROUP_VEHICLE)COPY_RANGE(mass,center_of_mass);
    if(d->groups&VEHICLE_LOG_GROUP_COMMAND)COPY_RANGE(target_pitch,target_airbrakes);
    if(d->groups&VEHICLE_LOG_GROUP_CONTROL)COPY_RANGE(has_control_state,control_yaw);
    if(d->groups&VEHICLE_LOG_GROUP_RUNTIME)COPY_RANGE(has_loop_wall_delta,control_loop_ms);
    if(d->groups&VEHICLE_LOG_GROUP_GUIDANCE)COPY_RANGE(range_to_site,entry_reversal_sign);
#undef COPY_RANGE
}

static void test_keyframe_delta_reconstruction(void){
    VehicleLogEncoder e;vehicle_log_encoder_init(&e);VehicleLogSample reconstructed={0};
    VehicleLogSample base=sample_at(650.0,1);VehicleLogDecision d=vehicle_log_decide(&e,&base,false,false);
    assert(d.emit&&d.keyframe);replay_vehicle_record(&reconstructed,&base,&d);vehicle_log_commit(&e,&base,&d);assert(memcmp(&reconstructed,&base,sizeof(base))==0);

    VehicleLogSample next=base;next.wall_monotonic_seconds+=.11;next.ut+=.11;next.tick_sequence=2;next.altitude-=3.0;next.position[0]+=3.0;next.roll+=2.0;next.control_state_roll+=.04;next.specific_mechanical_energy-=2500.0;next.entry_reversal_time_remaining-=.11;next.entry_reversal_ut+=.0;next.entry_reversal_sign=-1.0;next.guidance_compute_ms+=2.0;next.control_loop_ms+=3.0;next.physics_force_residual_confidence+=.02;next.physics_live_samples++;
    d=vehicle_log_decide(&e,&next,false,false);assert(d.emit&&!d.keyframe);assert(d.groups&VEHICLE_LOG_GROUP_POSITION);assert(d.groups&VEHICLE_LOG_GROUP_ATTITUDE);assert(d.groups&VEHICLE_LOG_GROUP_CONTROL);assert(d.groups&VEHICLE_LOG_GROUP_RUNTIME);assert(d.groups&VEHICLE_LOG_GROUP_GUIDANCE);assert(d.groups&VEHICLE_LOG_GROUP_PHYSICS);
    replay_vehicle_record(&reconstructed,&next,&d);vehicle_log_commit(&e,&next,&d);
    assert(reconstructed.tick_sequence==2);assert(reconstructed.altitude==next.altitude);assert(reconstructed.roll==next.roll);assert(reconstructed.control_state_roll==next.control_state_roll);assert(reconstructed.guidance_compute_ms==next.guidance_compute_ms);assert(reconstructed.control_loop_ms==next.control_loop_ms);assert(reconstructed.specific_mechanical_energy==next.specific_mechanical_energy);assert(reconstructed.entry_reversal_sign==next.entry_reversal_sign);assert(reconstructed.physics_force_residual_confidence==next.physics_force_residual_confidence);assert(reconstructed.physics_live_samples==next.physics_live_samples);assert(reconstructed.mass==base.mass);

    VehicleLogSample recovery=next;recovery.wall_monotonic_seconds+=5.01;recovery.ut+=5.01;recovery.tick_sequence=3;recovery.stall_fraction=.4;recovery.target_roll=-42.0;
    d=vehicle_log_decide(&e,&recovery,false,false);assert(d.emit&&d.keyframe);replay_vehicle_record(&reconstructed,&recovery,&d);assert(memcmp(&reconstructed,&recovery,sizeof(recovery))==0);
}

static void test_physics_diagnostics_delta(void){
    VehicleLogEncoder e;vehicle_log_encoder_init(&e);
    VehicleLogSample s=sample_at(675.0,1);VehicleLogDecision d=vehicle_log_decide(&e,&s,false,false);vehicle_log_commit(&e,&s,&d);
    s.wall_monotonic_seconds+=.51;s.ut+=.51;s.tick_sequence=2;s.physics_live_samples++;s.physics_force_residual_confidence+=.03;
    d=vehicle_log_decide(&e,&s,false,false);
    assert(d.emit&&!d.keyframe&&(d.groups&VEHICLE_LOG_GROUP_PHYSICS));
}

static void test_representative_bound(void){
    VehicleLogEncoder e;vehicle_log_encoder_init(&e);
    size_t bytes=0,records=0,keyframes=0;
    for(unsigned i=0;i<1200;i++){
        double wall=700.0+i*.05;
        VehicleLogSample s=sample_at(wall,(uint64_t)i+1);
        s.altitude-=i*.5;
        s.position[0]+=i*.5;
        s.true_air_speed-=i*.02;
        s.entry_reversal_time_remaining-=i*.05;
        s.physics_live_samples=12+i/10;
        s.physics_force_residual_confidence=.55+(((double)i/12000.0)<.10?((double)i/12000.0):.10);
        VehicleLogDecision d=vehicle_log_decide(&e,&s,false,false);
        if(!d.emit)continue;
        JsonWriter w;jw_init(&w);vehicle_log_record_json(&w,"bound-session",records+1,&s,&d);
        assert(!w.failed);bytes+=w.length+1;records++;if(d.keyframe)keyframes++;
        vehicle_log_commit(&e,&s,&d);jw_free(&w);
    }
    assert(records<=620);
    assert(keyframes>=12&&keyframes<=14);
    assert(bytes<1024u*1024u);
}

int main(void){
    test_first_and_unchanged();
    test_deadband_and_delta();
    test_semantic_event_forcing();
    test_plan_identity_forcing();
    test_constraint_and_touchdown_forcing();
    test_periodic_keyframe();
    test_rapid_transient_rate();
    test_json_separation_and_identity();
    test_keyframe_delta_reconstruction();
    test_physics_diagnostics_delta();
    test_representative_bound();
    puts("Vehicle flight-log codec tests passed.");
    return 0;
}
