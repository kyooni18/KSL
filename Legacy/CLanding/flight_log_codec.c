#include "flight_log_codec.h"

#include <math.h>
#include <string.h>

static bool changed_number(double a,double b,double absolute_deadband,double relative_deadband){
    if(isnan(a)&&isnan(b))return false;
    if(!isfinite(a)||!isfinite(b))return a!=b;
    double delta=fabs(a-b);
    double scale=fmax(fabs(a),fabs(b));
    return delta>absolute_deadband+relative_deadband*scale;
}

static bool changed_vector(const double a[3],const double b[3],double absolute_deadband,double relative_deadband){
    for(int i=0;i<3;i++)if(changed_number(a[i],b[i],absolute_deadband,relative_deadband))return true;
    return false;
}

static uint32_t changed_groups(const VehicleLogSample *a,const VehicleLogSample *b){
    uint32_t mask=0;
    if(strcmp(a->phase,b->phase)||strcmp(a->control_profile,b->control_profile)||strcmp(a->vessel_situation,b->vessel_situation)||
       a->automation_engaged!=b->automation_engaged||a->paused!=b->paused||a->constraint_active!=b->constraint_active||
       a->gear!=b->gear||a->brakes!=b->brakes||a->has_airbrakes!=b->has_airbrakes||a->airbrakes!=b->airbrakes||
       a->entry_plan_valid!=b->entry_plan_valid||a->entry_reversal_scheduled!=b->entry_reversal_scheduled||
       a->has_plan_identity!=b->has_plan_identity||a->plan_id!=b->plan_id||a->plan_version!=b->plan_version||
       a->parent_plan_id!=b->parent_plan_id||a->parent_plan_version!=b->parent_plan_version)
        mask|=VEHICLE_LOG_GROUP_STATE;

    if(changed_number(a->latitude,b->latitude,1e-6,0)||changed_number(a->longitude,b->longitude,1e-6,0)||
       changed_number(a->altitude,b->altitude,.25,1e-6)||changed_number(a->radar_altitude,b->radar_altitude,.25,1e-6)||
       changed_vector(a->position,b->position,.25,1e-8))
        mask|=VEHICLE_LOG_GROUP_POSITION;

    if(changed_vector(a->velocity,b->velocity,.15,1e-5)||changed_number(a->true_air_speed,b->true_air_speed,.20,1e-4)||
       changed_number(a->horizontal_speed,b->horizontal_speed,.15,1e-4)||changed_number(a->vertical_speed,b->vertical_speed,.10,1e-4)||
       changed_number(a->flight_path_angle,b->flight_path_angle,.03,1e-4))
        mask|=VEHICLE_LOG_GROUP_MOTION;

    if(changed_number(a->pitch,b->pitch,.04,1e-4)||changed_number(a->roll,b->roll,.04,1e-4)||
       changed_number(a->heading,b->heading,.04,1e-4)||changed_number(a->angle_of_attack,b->angle_of_attack,.04,1e-4)||
       changed_number(a->sideslip,b->sideslip,.04,1e-4)||changed_number(a->pitch_rate,b->pitch_rate,.02,1e-4)||
       changed_number(a->roll_rate,b->roll_rate,.02,1e-4)||changed_number(a->yaw_rate,b->yaw_rate,.02,1e-4)||
       changed_number(a->course_rate,b->course_rate,.02,1e-4)||changed_number(a->coordinate_pitch_rate,b->coordinate_pitch_rate,.02,1e-4)||
       changed_number(a->coordinate_roll_rate,b->coordinate_roll_rate,.02,1e-4)||changed_number(a->heading_rate,b->heading_rate,.02,1e-4)||
       changed_number(a->angle_of_attack_rate,b->angle_of_attack_rate,.02,1e-4)||changed_number(a->body_pitch_rate,b->body_pitch_rate,.02,1e-4)||
       changed_number(a->body_roll_rate,b->body_roll_rate,.02,1e-4)||changed_number(a->body_yaw_rate,b->body_yaw_rate,.02,1e-4))
        mask|=VEHICLE_LOG_GROUP_ATTITUDE;

    if(changed_number(a->dynamic_pressure,b->dynamic_pressure,2.0,.005)||changed_number(a->static_pressure,b->static_pressure,2.0,.005)||
       changed_number(a->atmospheric_density,b->atmospheric_density,1e-5,.01)||changed_number(a->mach,b->mach,.002,.001)||changed_number(a->stall_fraction,b->stall_fraction,.002,.01)||
       a->stall_fraction_is_measured!=b->stall_fraction_is_measured||changed_number(a->lift_force,b->lift_force,100,.005)||changed_number(a->drag_force,b->drag_force,100,.005)||
       changed_number(a->g_force,b->g_force,.005,.002)||changed_number(a->energy_excess_range,b->energy_excess_range,5,.001)||
       a->has_force_vectors!=b->has_force_vectors||(a->has_force_vectors&&(changed_vector(a->lift_vector,b->lift_vector,50,.005)||changed_vector(a->drag_vector,b->drag_vector,50,.005)||changed_vector(a->aero_acceleration,b->aero_acceleration,.005,.005))))
        mask|=VEHICLE_LOG_GROUP_AERO;

    if(changed_number(a->physics_confidence,b->physics_confidence,.002,.01)||
       changed_number(a->physics_model_residual,b->physics_model_residual,.002,.01)||
       changed_number(a->physics_model_residual_confidence,b->physics_model_residual_confidence,.002,.01)||
       changed_number(a->physics_certified_uncertainty,b->physics_certified_uncertainty,.002,.01)||
       changed_vector(a->physics_force_residual_per_q,b->physics_force_residual_per_q,1e-5,.01)||
       changed_vector(a->physics_force_residual_sigma_per_q,b->physics_force_residual_sigma_per_q,1e-5,.01)||
       changed_number(a->physics_force_residual_confidence,b->physics_force_residual_confidence,.002,.01)||
       a->physics_airbrake_model_available!=b->physics_airbrake_model_available||
       changed_number(a->physics_airbrake_model_confidence,b->physics_airbrake_model_confidence,.002,.01)||
       changed_number(a->physics_airbrake_drag_accel,b->physics_airbrake_drag_accel,.02,.01)||
       a->physics_samples!=b->physics_samples||a->physics_live_samples!=b->physics_live_samples)
        mask|=VEHICLE_LOG_GROUP_PHYSICS;

    if(changed_number(a->mass,b->mass,.25,1e-5)||changed_number(a->available_thrust,b->available_thrust,100,.002)||
       changed_number(a->current_thrust,b->current_thrust,100,.002)||a->has_center_of_mass!=b->has_center_of_mass||
       (a->has_center_of_mass&&changed_vector(a->center_of_mass,b->center_of_mass,.002,1e-4)))
        mask|=VEHICLE_LOG_GROUP_VEHICLE;

    if(changed_number(a->target_pitch,b->target_pitch,.04,1e-4)||changed_number(a->target_heading,b->target_heading,.04,1e-4)||
       changed_number(a->target_roll,b->target_roll,.04,1e-4)||changed_number(a->target_throttle,b->target_throttle,.001,.001)||
       a->has_target_aoa!=b->has_target_aoa||(a->has_target_aoa&&changed_number(a->target_aoa,b->target_aoa,.04,1e-4))||
       a->target_gear!=b->target_gear||a->target_brakes!=b->target_brakes||a->target_airbrakes!=b->target_airbrakes)
        mask|=VEHICLE_LOG_GROUP_COMMAND;

    if(a->has_control_state!=b->has_control_state||
       (a->has_control_state&&(changed_number(a->control_state_pitch,b->control_state_pitch,.002,.002)||changed_number(a->control_state_roll,b->control_state_roll,.002,.002)||changed_number(a->control_state_yaw,b->control_state_yaw,.002,.002)))||
       a->command_applied!=b->command_applied||a->has_actuator_feedback!=b->has_actuator_feedback||
       (a->has_actuator_feedback&&(changed_number(a->control_pitch,b->control_pitch,.002,.002)||changed_number(a->control_roll,b->control_roll,.002,.002)||changed_number(a->control_yaw,b->control_yaw,.002,.002))))
        mask|=VEHICLE_LOG_GROUP_CONTROL;

    if(a->has_loop_wall_delta!=b->has_loop_wall_delta||a->has_telemetry_latency!=b->has_telemetry_latency||
       a->has_guidance_compute!=b->has_guidance_compute||a->has_apply_latency!=b->has_apply_latency||a->has_control_loop!=b->has_control_loop||
       (a->has_loop_wall_delta&&changed_number(a->loop_wall_delta_ms,b->loop_wall_delta_ms,.25,.01))||
       (a->has_telemetry_latency&&changed_number(a->telemetry_latency_ms,b->telemetry_latency_ms,.25,.01))||
       (a->has_guidance_compute&&changed_number(a->guidance_compute_ms,b->guidance_compute_ms,.25,.01))||
       (a->has_apply_latency&&changed_number(a->apply_latency_ms,b->apply_latency_ms,.25,.01))||
       (a->has_control_loop&&changed_number(a->control_loop_ms,b->control_loop_ms,.25,.01)))
        mask|=VEHICLE_LOG_GROUP_RUNTIME;

    if(changed_number(a->range_to_site,b->range_to_site,5,.0005)||changed_number(a->bearing_to_site,b->bearing_to_site,.05,1e-4)||
       changed_number(a->runway_along_track,b->runway_along_track,2,.0005)||changed_number(a->runway_cross_track,b->runway_cross_track,2,.0005)||
       changed_number(a->specific_mechanical_energy,b->specific_mechanical_energy,100,1e-5)||changed_number(a->entry_reversal_time_remaining,b->entry_reversal_time_remaining,.10,.002)||
       changed_number(a->entry_reversal_ut,b->entry_reversal_ut,.02,0)||changed_number(a->entry_reversal_range,b->entry_reversal_range,2,.0005)||changed_number(a->entry_reversal_sign,b->entry_reversal_sign,.01,0))
        mask|=VEHICLE_LOG_GROUP_GUIDANCE;
    return mask;
}

static double minimum_interval(const VehicleLogSample *s){
    bool rapid=fabs(s->roll_rate)>5||fabs(s->pitch_rate)>4||fabs(s->yaw_rate)>4||
        (s->has_actuator_feedback&&(fabs(s->control_pitch)>.6||fabs(s->control_roll)>.6||fabs(s->control_yaw)>.6));
    if(rapid)return .05;
    if(s->dynamic_pressure>25||s->automation_engaged)return .10;
    return .20;
}

void vehicle_log_encoder_init(VehicleLogEncoder *encoder){if(encoder)memset(encoder,0,sizeof(*encoder));}

VehicleLogDecision vehicle_log_decide(const VehicleLogEncoder *encoder,const VehicleLogSample *sample,bool force_event,bool force_keyframe){
    VehicleLogDecision d={0};
    if(!encoder||!sample)return d;
    bool first=!encoder->has_last;
    bool periodic=encoder->has_last&&sample->wall_monotonic_seconds-encoder->last_keyframe_wall>=5.0;
    if(first||force_keyframe||periodic){d.emit=true;d.keyframe=true;d.groups=VEHICLE_LOG_GROUP_ALL;d.durable=force_event||force_keyframe;return d;}
    d.groups=changed_groups(sample,&encoder->last);
    bool semantic=(d.groups&VEHICLE_LOG_GROUP_STATE)!=0||sample->event[0]!=0;
    if(force_event||semantic){d.emit=true;d.durable=true;return d;}
    if(!d.groups)return d;
    if(sample->wall_monotonic_seconds-encoder->last_emit_wall+1e-9<minimum_interval(sample))return d;
    d.emit=true;
    return d;
}

void vehicle_log_commit(VehicleLogEncoder *encoder,const VehicleLogSample *sample,const VehicleLogDecision *decision){
    if(!encoder||!sample||!decision||!decision->emit)return;
    encoder->last=*sample;
    encoder->has_last=true;
    encoder->last_emit_wall=sample->wall_monotonic_seconds;
    if(decision->keyframe)encoder->last_keyframe_wall=sample->wall_monotonic_seconds;
}

static void field_prefix(JsonWriter *w,bool *first,const char *key){if(!*first)jw_char(w,',');*first=false;jw_string(w,key);jw_char(w,':');}
static void vec3_json(JsonWriter *w,const double v[3]){jw_char(w,'[');jw_number(w,v[0]);jw_char(w,',');jw_number(w,v[1]);jw_char(w,',');jw_number(w,v[2]);jw_char(w,']');}

void vehicle_log_record_json(JsonWriter *w,const char *session_id,uint64_t record_sequence,const VehicleLogSample *s,const VehicleLogDecision *d){
    if(!w||!s||!d)return;
    jw_char(w,'{');
    jw_raw(w,"\"schemaVersion\":4,\"recordType\":");jw_string(w,d->keyframe?"vehicleKeyframe":"vehicleDelta");
    jw_raw(w,",\"sessionId\":");jw_string(w,session_id?session_id:"");
    jw_raw(w,",\"recordSequence\":");jw_integer(w,(long long)record_sequence);
    jw_raw(w,",\"tickSequence\":");jw_integer(w,(long long)s->tick_sequence);
    jw_raw(w,",\"wallMonotonicSeconds\":");jw_number(w,s->wall_monotonic_seconds);
    jw_raw(w,",\"ut\":");jw_number(w,s->ut);
    if(s->event[0]){jw_raw(w,",\"event\":");jw_string(w,s->event);}
    jw_raw(w,",\"fields\":{");bool first=true;
    if(d->groups&VEHICLE_LOG_GROUP_STATE){
        field_prefix(w,&first,"state");jw_char(w,'{');jw_raw(w,"\"phase\":");jw_string(w,s->phase);jw_raw(w,",\"controlProfile\":");jw_string(w,s->control_profile);jw_raw(w,",\"vesselSituation\":");jw_string(w,s->vessel_situation);jw_raw(w,",\"automation\":");jw_bool(w,s->automation_engaged);jw_raw(w,",\"paused\":");jw_bool(w,s->paused);jw_raw(w,",\"constraintActive\":");jw_bool(w,s->constraint_active);jw_raw(w,",\"gear\":");jw_bool(w,s->gear);jw_raw(w,",\"brakes\":");jw_bool(w,s->brakes);jw_raw(w,",\"airbrakesAvailable\":");jw_bool(w,s->has_airbrakes);jw_raw(w,",\"airbrakes\":");if(s->has_airbrakes)jw_bool(w,s->airbrakes);else jw_raw(w,"null");jw_raw(w,",\"entryPlanValid\":");jw_bool(w,s->entry_plan_valid);jw_raw(w,",\"entryReversalScheduled\":");jw_bool(w,s->entry_reversal_scheduled);jw_raw(w,",\"planId\":");if(s->has_plan_identity)jw_integer(w,(long long)s->plan_id);else jw_null(w);jw_raw(w,",\"planVersion\":");if(s->has_plan_identity)jw_integer(w,(long long)s->plan_version);else jw_null(w);jw_raw(w,",\"parentPlanId\":");if(s->has_plan_identity)jw_integer(w,(long long)s->parent_plan_id);else jw_null(w);jw_raw(w,",\"parentPlanVersion\":");if(s->has_plan_identity)jw_integer(w,(long long)s->parent_plan_version);else jw_null(w);jw_char(w,'}');
    }
    if(d->groups&VEHICLE_LOG_GROUP_POSITION){
        field_prefix(w,&first,"position");jw_char(w,'{');jw_raw(w,"\"latitude\":");jw_number(w,s->latitude);jw_raw(w,",\"longitude\":");jw_number(w,s->longitude);jw_raw(w,",\"altitude\":");jw_number(w,s->altitude);jw_raw(w,",\"radarAltitude\":");jw_number(w,s->radar_altitude);jw_raw(w,",\"inertial\":");vec3_json(w,s->position);jw_char(w,'}');
    }
    if(d->groups&VEHICLE_LOG_GROUP_MOTION){
        field_prefix(w,&first,"motion");jw_char(w,'{');jw_raw(w,"\"velocity\":");vec3_json(w,s->velocity);jw_raw(w,",\"trueAirSpeed\":");jw_number(w,s->true_air_speed);jw_raw(w,",\"horizontalSpeed\":");jw_number(w,s->horizontal_speed);jw_raw(w,",\"verticalSpeed\":");jw_number(w,s->vertical_speed);jw_raw(w,",\"flightPathAngle\":");jw_number(w,s->flight_path_angle);jw_char(w,'}');
    }
    if(d->groups&VEHICLE_LOG_GROUP_ATTITUDE){
        field_prefix(w,&first,"attitude");jw_char(w,'{');jw_raw(w,"\"pitch\":");jw_number(w,s->pitch);jw_raw(w,",\"roll\":");jw_number(w,s->roll);jw_raw(w,",\"heading\":");jw_number(w,s->heading);jw_raw(w,",\"angleOfAttack\":");jw_number(w,s->angle_of_attack);jw_raw(w,",\"sideslip\":");jw_number(w,s->sideslip);jw_raw(w,",\"pitchRate\":");jw_number(w,s->pitch_rate);jw_raw(w,",\"rollRate\":");jw_number(w,s->roll_rate);jw_raw(w,",\"yawRate\":");jw_number(w,s->yaw_rate);jw_raw(w,",\"courseRate\":");jw_number(w,s->course_rate);jw_raw(w,",\"coordinatePitchRate\":");jw_number(w,s->coordinate_pitch_rate);jw_raw(w,",\"coordinateRollRate\":");jw_number(w,s->coordinate_roll_rate);jw_raw(w,",\"headingRate\":");jw_number(w,s->heading_rate);jw_raw(w,",\"angleOfAttackRate\":");jw_number(w,s->angle_of_attack_rate);jw_raw(w,",\"bodyPitchRate\":");jw_number(w,s->body_pitch_rate);jw_raw(w,",\"bodyRollRate\":");jw_number(w,s->body_roll_rate);jw_raw(w,",\"bodyYawRate\":");jw_number(w,s->body_yaw_rate);jw_char(w,'}');
    }
    if(d->groups&VEHICLE_LOG_GROUP_AERO){
        field_prefix(w,&first,"aero");jw_char(w,'{');jw_raw(w,"\"dynamicPressure\":");jw_number(w,s->dynamic_pressure);jw_raw(w,",\"staticPressure\":");jw_number(w,s->static_pressure);jw_raw(w,",\"density\":");jw_number(w,s->atmospheric_density);jw_raw(w,",\"mach\":");jw_number(w,s->mach);jw_raw(w,",\"stallFraction\":");jw_number(w,s->stall_fraction);jw_raw(w,",\"stallFractionMeasured\":");jw_bool(w,s->stall_fraction_is_measured);jw_raw(w,",\"liftForce\":");jw_number(w,s->lift_force);jw_raw(w,",\"dragForce\":");jw_number(w,s->drag_force);jw_raw(w,",\"gForce\":");jw_number(w,s->g_force);jw_raw(w,",\"energyExcessRange\":");jw_number(w,s->energy_excess_range);jw_raw(w,",\"liftVector\":");if(s->has_force_vectors)vec3_json(w,s->lift_vector);else jw_raw(w,"null");jw_raw(w,",\"dragVector\":");if(s->has_force_vectors)vec3_json(w,s->drag_vector);else jw_raw(w,"null");jw_raw(w,",\"aeroAcceleration\":");if(s->has_force_vectors)vec3_json(w,s->aero_acceleration);else jw_raw(w,"null");jw_char(w,'}');
    }
    if(d->groups&VEHICLE_LOG_GROUP_PHYSICS){
        field_prefix(w,&first,"physics");jw_char(w,'{');jw_raw(w,"\"confidence\":");jw_number(w,s->physics_confidence);jw_raw(w,",\"modelResidual\":");jw_number(w,s->physics_model_residual);jw_raw(w,",\"modelResidualConfidence\":");jw_number(w,s->physics_model_residual_confidence);jw_raw(w,",\"certifiedUncertainty\":");jw_number(w,s->physics_certified_uncertainty);jw_raw(w,",\"forceResidualPerQ\":");vec3_json(w,s->physics_force_residual_per_q);jw_raw(w,",\"forceResidualSigmaPerQ\":");vec3_json(w,s->physics_force_residual_sigma_per_q);jw_raw(w,",\"forceResidualConfidence\":");jw_number(w,s->physics_force_residual_confidence);jw_raw(w,",\"airbrakeModelAvailable\":");jw_bool(w,s->physics_airbrake_model_available);jw_raw(w,",\"airbrakeModelConfidence\":");jw_number(w,s->physics_airbrake_model_confidence);jw_raw(w,",\"airbrakeDragAccel\":");jw_number(w,s->physics_airbrake_drag_accel);jw_raw(w,",\"samples\":");jw_integer(w,(long long)s->physics_samples);jw_raw(w,",\"liveSamples\":");jw_integer(w,(long long)s->physics_live_samples);jw_char(w,'}');
    }
    if(d->groups&VEHICLE_LOG_GROUP_VEHICLE){
        field_prefix(w,&first,"vehicle");jw_char(w,'{');jw_raw(w,"\"mass\":");jw_number(w,s->mass);jw_raw(w,",\"availableThrust\":");jw_number(w,s->available_thrust);jw_raw(w,",\"currentThrust\":");jw_number(w,s->current_thrust);jw_raw(w,",\"centerOfMass\":");if(s->has_center_of_mass)vec3_json(w,s->center_of_mass);else jw_raw(w,"null");jw_char(w,'}');
    }
    if(d->groups&VEHICLE_LOG_GROUP_COMMAND){
        field_prefix(w,&first,"command");jw_char(w,'{');jw_raw(w,"\"targetPitch\":");jw_number(w,s->target_pitch);jw_raw(w,",\"targetHeading\":");jw_number(w,s->target_heading);jw_raw(w,",\"targetRoll\":");jw_number(w,s->target_roll);jw_raw(w,",\"targetThrottle\":");jw_number(w,s->target_throttle);jw_raw(w,",\"targetAoA\":");if(s->has_target_aoa)jw_number(w,s->target_aoa);else jw_raw(w,"null");jw_raw(w,",\"gear\":");jw_bool(w,s->target_gear);jw_raw(w,",\"brakes\":");jw_bool(w,s->target_brakes);jw_raw(w,",\"airbrakes\":");jw_bool(w,s->target_airbrakes);jw_char(w,'}');
    }
    if(d->groups&VEHICLE_LOG_GROUP_CONTROL){
        field_prefix(w,&first,"appliedControl");jw_char(w,'{');jw_raw(w,"\"reportedAvailable\":");jw_bool(w,s->has_control_state);jw_raw(w,",\"reportedPitch\":");if(s->has_control_state)jw_number(w,s->control_state_pitch);else jw_raw(w,"null");jw_raw(w,",\"reportedRoll\":");if(s->has_control_state)jw_number(w,s->control_state_roll);else jw_raw(w,"null");jw_raw(w,",\"reportedYaw\":");if(s->has_control_state)jw_number(w,s->control_state_yaw);else jw_raw(w,"null");jw_raw(w,",\"applySucceeded\":");jw_bool(w,s->command_applied);jw_raw(w,",\"diagnosticPitch\":");if(s->has_actuator_feedback)jw_number(w,s->control_pitch);else jw_raw(w,"null");jw_raw(w,",\"diagnosticRoll\":");if(s->has_actuator_feedback)jw_number(w,s->control_roll);else jw_raw(w,"null");jw_raw(w,",\"diagnosticYaw\":");if(s->has_actuator_feedback)jw_number(w,s->control_yaw);else jw_raw(w,"null");jw_char(w,'}');
    }
    if(d->groups&VEHICLE_LOG_GROUP_RUNTIME){
        field_prefix(w,&first,"runtime");jw_char(w,'{');jw_raw(w,"\"loopWallDeltaMilliseconds\":");if(s->has_loop_wall_delta)jw_number(w,s->loop_wall_delta_ms);else jw_raw(w,"null");jw_raw(w,",\"telemetryLatencyMilliseconds\":");if(s->has_telemetry_latency)jw_number(w,s->telemetry_latency_ms);else jw_raw(w,"null");jw_raw(w,",\"guidanceComputeMilliseconds\":");if(s->has_guidance_compute)jw_number(w,s->guidance_compute_ms);else jw_raw(w,"null");jw_raw(w,",\"applyLatencyMilliseconds\":");if(s->has_apply_latency)jw_number(w,s->apply_latency_ms);else jw_raw(w,"null");jw_raw(w,",\"controlLoopMilliseconds\":");if(s->has_control_loop)jw_number(w,s->control_loop_ms);else jw_raw(w,"null");jw_char(w,'}');
    }
    if(d->groups&VEHICLE_LOG_GROUP_GUIDANCE){
        field_prefix(w,&first,"guidance");jw_char(w,'{');jw_raw(w,"\"rangeToSite\":");jw_number(w,s->range_to_site);jw_raw(w,",\"bearingToSite\":");jw_number(w,s->bearing_to_site);jw_raw(w,",\"runwayAlongTrack\":");jw_number(w,s->runway_along_track);jw_raw(w,",\"runwayCrossTrack\":");jw_number(w,s->runway_cross_track);jw_raw(w,",\"specificMechanicalEnergy\":");jw_number(w,s->specific_mechanical_energy);jw_raw(w,",\"entryReversalTimeRemaining\":");jw_number(w,s->entry_reversal_time_remaining);jw_raw(w,",\"entryReversalUT\":");if(s->entry_reversal_scheduled)jw_number(w,s->entry_reversal_ut);else jw_raw(w,"null");jw_raw(w,",\"entryReversalRange\":");if(s->entry_reversal_scheduled)jw_number(w,s->entry_reversal_range);else jw_raw(w,"null");jw_raw(w,",\"entryReversalSign\":");if(s->entry_reversal_scheduled)jw_number(w,s->entry_reversal_sign);else jw_raw(w,"null");jw_char(w,'}');
    }
    jw_raw(w,"}}");
}
