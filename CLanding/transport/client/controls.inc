static bool set_engine_throttle(KrpcCNanoClient *client, double throttle,
                                char *error, size_t error_size) {
    bool powered = throttle > 1e-4;
    for (size_t i = 0; i < client->engine_count; ++i) {
        CommandableEngine *engine = &client->engines[i];
        if (powered) {
            bool active = false, fuel = true;
            krpc_error_t e = krpc_SpaceCenter_Engine_Active(client->connection, &active, engine->object); client->total_rpc_calls++; client->total_wire_requests++;
            if (e != KRPC_OK) continue;
            e = krpc_SpaceCenter_Engine_HasFuel(client->connection, &fuel, engine->object); client->total_rpc_calls++; client->total_wire_requests++;
            if (e != KRPC_OK || !fuel) continue;
            if (!active) {
                if (rpc_failed(client, krpc_SpaceCenter_Engine_set_Active(client->connection, engine->object, true),
                               "Engine.Active", error, error_size)) return false;
                engine->activated_by_client = true;
            }
            if (rpc_failed(client, krpc_SpaceCenter_Engine_set_IndependentThrottle(client->connection, engine->object, true),
                           "Engine.IndependentThrottle", error, error_size) ||
                rpc_failed(client, krpc_SpaceCenter_Engine_set_Throttle(client->connection, engine->object, (float)throttle),
                           "Engine.Throttle", error, error_size)) return false;
            engine->independent_owned = true;
        } else if (engine->independent_owned) {
            RPC_OPTIONAL(client, krpc_SpaceCenter_Engine_set_Throttle(client->connection, engine->object, 0.0f));
            RPC_OPTIONAL(client, krpc_SpaceCenter_Engine_set_IndependentThrottle(client->connection, engine->object, false));
            engine->independent_owned = false;
        }
    }
    return true;
}

bool krpc_cnano_client_set_direct_controls(KrpcCNanoClient *client,
                                           double pitch,double roll,double yaw,
                                           double throttle,double wheel_steering,
                                           bool rcs,
                                           char *error,size_t error_size){
    if(!client||!client->connected){set_error(error,error_size,"C-Nano client is not connected");return false;}
    client->pause_triplet_sent=false;
    double started=monotonic_seconds();unsigned before=client->total_rpc_calls;unsigned wire_before=client->total_wire_requests;
    pitch=clampd(pitch,-1,1);roll=clampd(roll,-1,1);yaw=clampd(yaw,-1,1);
    throttle=clampd(throttle,0,1);wheel_steering=clampd(wheel_steering,-1,1);

    bool reset_orbital_profile=client->orbital_autotune_profile_active;
    if(reset_orbital_profile){
        /* Direct atmospheric control must not inherit orbital AutoPilot tuning.
           Reset is a single deterministic handoff: it disengages AutoPilot and
           restores all of its controller configuration/oscillation state. */
        RPC_REQUIRED(client,krpc_SpaceCenter_AutoPilot_Reset(client->connection,client->autopilot),error,error_size);
        client->cached_autopilot_engaged=false;client->has_autopilot_engaged=true;
        client->orbital_autotune_profile_active=false;
    }
    bool disengage_autopilot=!reset_orbital_profile&&(!client->has_autopilot_engaged||client->cached_autopilot_engaged);
    bool disable_sas=!client->has_sas||client->cached_sas;
    bool change_throttle=!client->has_command_throttle||fabs(client->cached_command_throttle-throttle)>1e-4;
    bool change_wheel=!client->has_command_wheel||fabs(client->cached_command_wheel-wheel_steering)>1e-3;
    bool change_rcs=!client->has_rcs||client->cached_rcs!=rcs;

    ClientBatch batch={0};
#define WRITE(expr) do{if(!(expr)){set_error(error,error_size,"Could not construct direct-control C-Nano batch");return false;}}while(0)
    if(disengage_autopilot)WRITE(batch_add_object_bool(&batch,PROC_AUTOPILOT_SET_ENGAGED,client->autopilot,false,true));
    if(disable_sas)WRITE(batch_add_object_bool(&batch,PROC_CONTROL_SET_SAS,client->control,false,true));
    WRITE(batch_add_object_float(&batch,PROC_CONTROL_SET_PITCH,client->control,(float)pitch,true));
    WRITE(batch_add_object_float(&batch,PROC_CONTROL_SET_ROLL,client->control,(float)roll,true));
    WRITE(batch_add_object_float(&batch,PROC_CONTROL_SET_YAW,client->control,(float)yaw,true));
    if(change_throttle)WRITE(batch_add_object_float(&batch,PROC_CONTROL_SET_THROTTLE,client->control,(float)throttle,true));
    if(change_wheel)WRITE(batch_add_object_float(&batch,PROC_CONTROL_SET_WHEEL_STEERING,client->control,(float)wheel_steering,true));
    if(change_rcs)WRITE(batch_add_object_bool(&batch,PROC_CONTROL_SET_RCS,client->control,rcs,true));
#undef WRITE
    if(!batch_run(client,&batch,"direct control",error,error_size))return false;

    if(disengage_autopilot){client->cached_autopilot_engaged=false;client->has_autopilot_engaged=true;}
    if(disable_sas){client->cached_sas=false;client->has_sas=true;}
    if(change_throttle){
        if(!set_engine_throttle(client,throttle,error,error_size))return false;
        client->cached_command_throttle=throttle;client->has_command_throttle=true;
    }
    if(change_wheel){client->cached_command_wheel=wheel_steering;client->has_command_wheel=true;}
    if(change_rcs){client->cached_rcs=rcs;client->has_rcs=true;}

    client->last_apply_calls=client->total_rpc_calls-before;
    client->last_apply_wire_requests=client->total_wire_requests-wire_before;
    client->last_apply_seconds=monotonic_seconds()-started;
    return true;
}

bool krpc_cnano_client_set_autopilot(KrpcCNanoClient *client,
                                     const GuidanceCommand *command,
                                     const Vector3 *inertial_up_reference,
                                     char *error, size_t error_size) {
    if (!client || !client->connected || !command) { set_error(error, error_size, "C-Nano autopilot state is invalid"); return false; }
    client->pause_triplet_sent=false;
    double started=monotonic_seconds(); unsigned before=client->total_rpc_calls; unsigned wire_before=client->total_wire_requests;
    if (!client->has_sas || client->cached_sas) {
        RPC_REQUIRED(client, krpc_SpaceCenter_Control_set_SAS(client->connection, client->control, false), error, error_size);
        client->cached_sas = false; client->has_sas = true;
    }
    /* Orbital/inertial attitude capture has essentially no aerodynamic control
       authority. Own RCS explicitly there, then release it as soon as guidance
       returns to surface-frame atmospheric control. SAS remains off. */
    bool want_rcs = command->control_profile == PROFILE_ORBITAL;
    if (!client->has_rcs || client->cached_rcs != want_rcs) {
        RPC_REQUIRED(client, krpc_SpaceCenter_Control_set_RCS(client->connection, client->control, want_rcs), error, error_size);
        client->cached_rcs = want_rcs; client->has_rcs = true;
    }
    krpc_SpaceCenter_ReferenceFrame_t frame = command->use_inertial_direction ? client->body_nonrotating : client->surface_frame;
    bool want_orbital_tuning = command->control_profile == PROFILE_ORBITAL;
    bool engaging_orbital = want_orbital_tuning && command->autopilot_engaged &&
        (!client->has_autopilot_engaged || !client->cached_autopilot_engaged);
    if (engaging_orbital && !client->orbital_autotune_profile_active) {
        /* Keep kRPC AutoTune live: it recomputes gains from the current torque and
           inertia every physics tick. STS-N's roll axis has much greater angular
           authority than pitch/yaw, so a lower roll bandwidth prevents the roll
           rate damper from railing together with pitch during the antipodal
           retrograde slew. The pitch/yaw overshoot allowance recovers capture-time
           margin without manually freezing any PID gains. Repeated live checkpoint
           trials: 27.44-28.58 s to <=10 deg, 0.26-0.36 s multi-axis saturation. */
        krpc_tuple_double_double_double_t time_to_peak = {0.8, 5.0, 0.8};
        krpc_tuple_double_double_double_t overshoot = {0.05, 0.01, 0.05};
        RPC_REQUIRED(client, krpc_SpaceCenter_AutoPilot_set_AutoTune(client->connection, client->autopilot, true), error, error_size);
        RPC_REQUIRED(client, krpc_SpaceCenter_AutoPilot_set_TimeToPeak(client->connection, client->autopilot, &time_to_peak), error, error_size);
        RPC_REQUIRED(client, krpc_SpaceCenter_AutoPilot_set_Overshoot(client->connection, client->autopilot, &overshoot), error, error_size);
        RPC_REQUIRED(client, krpc_SpaceCenter_AutoPilot_set_SoftStartTime(client->connection, client->autopilot, 0.5), error, error_size);
        client->orbital_autotune_profile_active = true;
    } else if (!want_orbital_tuning && client->orbital_autotune_profile_active) {
        /* Restore kRPC's complete default controller contract before any later
           surface-frame AutoPilot engagement in this session. */
        RPC_REQUIRED(client, krpc_SpaceCenter_AutoPilot_Reset(client->connection, client->autopilot), error, error_size);
        client->cached_autopilot_engaged = false; client->has_autopilot_engaged = true;
        client->orbital_autotune_profile_active = false;
    }
    RPC_REQUIRED(client, krpc_SpaceCenter_AutoPilot_set_ReferenceFrame(client->connection, client->autopilot, frame), error, error_size);
    if (command->use_inertial_direction) {
        krpc_tuple_double_double_double_t direction = canonical_to_tuple(command->inertial_direction);
        if (inertial_up_reference) {
            Vector3 normalized_up = vnorm(*inertial_up_reference, v3(1, 0, 0));
            krpc_tuple_double_double_double_t up = canonical_to_tuple(normalized_up);
            RPC_REQUIRED(client, krpc_SpaceCenter_AutoPilot_SetDirectionAndUp(client->connection, client->autopilot, &direction, &up, (float)command->target_roll), error, error_size);
        } else {
            RPC_REQUIRED(client, krpc_SpaceCenter_AutoPilot_set_TargetDirection(client->connection, client->autopilot, &direction), error, error_size);
            RPC_REQUIRED(client, krpc_SpaceCenter_AutoPilot_set_TargetRoll(client->connection, client->autopilot, (float)command->target_roll), error, error_size);
        }
    } else {
        /* Vessel.SurfaceReferenceFrame is x=up, y=north, z=east.  Sending
           target pitch and heading as two independent RPCs makes kRPC rotate
           through an intermediate attitude every control tick; during the
           post-burn 180-degree RCS flip that produced a large roll/yaw limit
           cycle.  Build the desired local direction once and update it
           atomically as a quaternion target instead. */
        double pitch = clampd(command->target_pitch, -89.9, 89.9) * DEG2RAD;
        double heading = norm_deg(command->target_heading) * DEG2RAD;
        double cp = cos(pitch);
        krpc_tuple_double_double_double_t direction = {
            sin(pitch),
            cp * cos(heading),
            cp * sin(heading)
        };
        RPC_REQUIRED(client, krpc_SpaceCenter_AutoPilot_set_TargetDirection(client->connection, client->autopilot, &direction), error, error_size);
        RPC_REQUIRED(client, krpc_SpaceCenter_AutoPilot_set_TargetRoll(client->connection, client->autopilot, (float)command->target_roll), error, error_size);
    }
    if (!client->has_autopilot_engaged || client->cached_autopilot_engaged != command->autopilot_engaged) {
        RPC_REQUIRED(client, krpc_SpaceCenter_AutoPilot_set_Engaged(client->connection, client->autopilot, command->autopilot_engaged), error, error_size);
        client->cached_autopilot_engaged = command->autopilot_engaged; client->has_autopilot_engaged = true;
    }
    double throttle = clampd(command->target_throttle, 0, 1);
    if (!client->has_command_throttle || fabs(client->cached_command_throttle - throttle) > 1e-4) {
        RPC_REQUIRED(client, krpc_SpaceCenter_Control_set_Throttle(client->connection, client->control, (float)throttle), error, error_size);
        if (!set_engine_throttle(client, throttle, error, error_size)) return false;
        client->cached_command_throttle = throttle; client->has_command_throttle = true;
    }
    client->cached_sas = false; client->has_sas = true;
    client->last_apply_calls=client->total_rpc_calls-before;
    client->last_apply_wire_requests=client->total_wire_requests-wire_before;
    client->last_apply_seconds=monotonic_seconds()-started;
    return true;
}

bool krpc_cnano_client_set_gear(KrpcCNanoClient *client, bool value, char *error, size_t error_size) {
    if (!client || !client->connected) return false;
    if (client->has_gear && client->cached_gear == value) return true;
    RPC_REQUIRED(client, krpc_SpaceCenter_Control_set_Gear(client->connection, client->control, value), error, error_size);
    client->cached_gear = value; client->has_gear = true; return true;
}
bool krpc_cnano_client_set_brakes(KrpcCNanoClient *client, bool value, char *error, size_t error_size) {
    if (!client || !client->connected) return false;
    if (client->has_brakes && client->cached_brakes == value) return true;
    RPC_REQUIRED(client, krpc_SpaceCenter_Control_set_Brakes(client->connection, client->control, value), error, error_size);
    client->cached_brakes = value; client->has_brakes = true; return true;
}
bool krpc_cnano_client_set_airbrakes(KrpcCNanoClient *client, unsigned group, bool value, char *error, size_t error_size) {
    if (!client || !client->connected) return false;
    if (client->has_airbrakes && client->cached_airbrakes == value) return true;
    if (group < 1) group = 1; if (group > 10) group = 10;
    RPC_REQUIRED(client, krpc_SpaceCenter_Control_SetActionGroup(client->connection, client->control, group, value), error, error_size);
    client->cached_airbrakes = value; client->has_airbrakes = true; return true;
}
bool krpc_cnano_client_set_speed_mode(KrpcCNanoClient *client, NavballSpeedMode mode, char *error, size_t error_size) {
    if (!client || !client->connected || mode == SPEED_UNCHANGED) return true;
    if (client->has_speed_mode && client->cached_speed_mode == mode) return true;
    krpc_SpaceCenter_SpeedMode_t native = mode == SPEED_ORBIT ? KRPC_SPACECENTER_SPEEDMODE_ORBIT : KRPC_SPACECENTER_SPEEDMODE_SURFACE;
    RPC_REQUIRED(client, krpc_SpaceCenter_Control_set_SpeedMode(client->connection, client->control, native), error, error_size);
    client->cached_speed_mode = mode; client->has_speed_mode = true; return true;
}
/* decision-literal: protocol-domain-requirement | Stock KSP rails-warp factor indices map to these on-rails rates; live SpaceCenter.WarpRate replaces each prior once observed. */
static const double k_stock_rails_rate_by_factor[] = {
    1.0, 5.0, 10.0, 50.0, 100.0, 1000.0, 10000.0, 100000.0
};

