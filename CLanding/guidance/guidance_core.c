#include "guidance_internal.h"

#include <float.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* Translation-unit private helpers. */
static void reset_controllers(GuidanceMachine*g);
static void reset_entry_s_turn_program(GuidanceMachine*g);
static double entry_model_best_glide_aoa(const Telemetry*t,const VehicleProfile*v);

       GuidanceResult result_make(GuidancePhase phase,GuidanceCommand c,const char*status,const char*warning){GuidanceResult r;memset(&r,0,sizeof(r));r.phase=phase;r.command=c;snprintf(r.status,sizeof(r.status),"%s",status?status:"");if(warning&&*warning){r.has_warning=true;snprintf(r.warning,sizeof(r.warning),"%s",warning);}trajectory_init(&r.reference);return r;}
void guidance_result_clear(GuidanceResult*r){trajectory_clear(&r->reference);memset(r,0,sizeof(*r));}
       void reset_limiters(GuidanceMachine*g){memset(&g->pitch_limiter,0,sizeof(g->pitch_limiter));memset(&g->roll_limiter,0,sizeof(g->roll_limiter));memset(&g->heading_limiter,0,sizeof(g->heading_limiter));memset(&g->throttle_limiter,0,sizeof(g->throttle_limiter));g->has_last_stabilized_phase=false;}

static void reset_controllers(GuidanceMachine*g){
    if(!g)return;

    g->taem_interface_target.valid=false;
    g->terminal_path_kind=TERMINAL_PATH_NONE;
    g->terminal_path_committed=false;
    g->mm305_route=(TaemRoute){0};
    g->mm305_route_cursor=0;
    g->mm305_route_committed=false;
    g->mm305_hac_exit_reached=false;
    g->mm305_model_snapshot_id=0;
    g->mm305_planning_needed=false;
    g->mm305_plan_request_ut=NAN;
    g->mm305_last_plan_attempt_ut=-INFINITY;
    g->mm305_last_route_ut=-INFINITY;
    g->mm305_plan_failures=0;
    g->mm305_replans=0;
    g->mm305_lift_scale=1.0;
    g->mm305_drag_scale=1.0;
    g->runway_end_preview_valid=false;
    g->runway_end_committed=false;
    g->runway_end_index=0;
    g->runway_end_primary_path=NAN;
    g->runway_end_reciprocal_path=NAN;
    g->terminal_final_handoff_latched=false;
    g->terminal_final_handoff_distance=NAN;

    robust_pid_reset(&g->entry_energy_pid);
    robust_pid_reset(&g->taem_altitude_pid);
    robust_pid_reset(&g->final_altitude_pid);
    robust_pid_reset(&g->speed_pid);
    robust_pid_reset(&g->flare_sink_pid);
    reset_limiters(g);
    speedbrake_controller_reset(&g->speedbrake_controller,false);

    g->airbrakes_deployed=false;
    g->final_approach_captured=false;
    g->hac_side_selected=false;
    g->hac_progress_valid=false;
    g->hac_captured=false;
    g->hac_completed=false;
    g->hac_remaining=0.0;
    g->hac_radius=0.0;
    g->minimum_turn_radius=0.0;

    g->terminal_region_entered=false;
    g->hac_capture_lost_duration=0.0;
    g->terminal_energy_mismatch_duration=0.0;
    g->terminal_reentry_after_ut=0.0;

    g->terminal_glide_mode=false;
    g->terminal_rehearsal_mode=false;
    g->terminal_test_capture_active=false;
    g->terminal_test_glide_slope=0.0;
    g->terminal_test_final_approach_distance=0.0;
    g->terminal_test_revolution_remaining=0.0;
    g->terminal_test_preflare_altitude=0.0;
    g->terminal_test_preflare_target_speed=0.0;
    g->terminal_test_preflare_min_speed=0.0;

    g->terminal_vertical_stage=TERMINAL_TRAJECTORY_CAPTURE;
    g->terminal_vertical_stage_valid=false;
    g->gear_command_latched=false;
    g->ground_contact_latched=false;
    g->flare_sink_captured=false;
    g->terminal_response_sample_valid=false;
    g->terminal_preflare_plan_valid=false;
    g->terminal_energy_sample_valid=false;
    g->terminal_stage_started_ut=0.0;
    g->terminal_stage_good_duration=0.0;
    g->ground_contact_duration=0.0;
    g->terminal_previous_aoa=0.0;
    g->terminal_previous_vertical_speed=0.0;
    g->terminal_response_sample_ut=0.0;
    g->terminal_previous_specific_energy=0.0;
    g->terminal_previous_speed=0.0;
    g->terminal_energy_sample_ut=0.0;
    g->terminal_energy_loss_accel_ema=0.0;
    g->terminal_speed_loss_accel_ema=0.0;

    /* Unknown response stays unknown.  It is learned from telemetry or derived
       from live torque/inertia; reset must not manufacture actuator authority. */
    g->terminal_positive_aoa_rate_ema=0.0;
    g->terminal_sink_accel_ema=0.0;
    g->terminal_pitch_response_delay_ema=0.0;

    g->preflare_trigger_altitude=0.0;
    g->preflare_target_aoa=0.0;
    g->preflare_target_sink=0.0;
    g->preflare_minimum_speed=0.0;
    g->preflare_reference_speed=0.0;
    g->preflare_predicted_height_loss=0.0;
    g->preflare_predicted_kinetic_margin=0.0;
    g->preflare_effective_accel=0.0;

    g->final_invalid_duration=0.0;
    g->attitude_recovery=false;
    g->control_bad_duration=0.0;
    g->control_good_duration=0.0;
    g->recovery_duration=0.0;
    g->recovery_heading=0.0;
    g->recovery_aoa=0.0;
    g->previous_relative_roll_rate=0.0;
    g->roll_oscillation_score=0.0;
    g->roll_rate_excess_duration=0.0;
    g->has_previous_relative_roll_rate=false;
    g->previous_sideslip=0.0;
    g->has_previous_sideslip=false;

    g->entry_predictive_score=0.0;
    g->entry_reference_speed=0.0;
    g->entry_reference_altitude=0.0;
    g->entry_reference_range=0.0;
    g->has_entry_predictive_score=false;
    g->entry_control_plan_valid=false;
    g->entry_control_terminal_ready=false;
    g->entry_control_plan_ut=0.0;
    g->entry_control_bank=0.0;
    g->entry_control_aoa=0.0;
    g->entry_control_heading=0.0;
    g->entry_control_turn_radius=0.0;
    g->entry_control_segment_until_ut=0.0;
    g->entry_control_cost=0.0;
    g->entry_control_taem_range_error=0.0;
    g->entry_control_taem_speed=0.0;
    g->entry_control_taem_energy_error=0.0;
    g->entry_reversal_scheduled=false;
    g->entry_reversal_is_final=false;
    g->entry_final_reversal_pending=false;
    g->entry_final_reversal_completed=false;
    g->entry_continuation_bootstrap=false;
    g->entry_target_side_latched=false;
    g->entry_target_side=0.0;
    g->entry_reversal_ut=0.0;
    g->entry_reversal_range=0.0;
    g->entry_reversal_sign=0.0;
    g->entry_reversal_bank=0.0;
    g->entry_control_reversals=0u;
    memset(&g->entry_feedback,0,sizeof(g->entry_feedback));
    g->entry_lateral_bank_magnitude=0.0;
    g->entry_geometry_bank=0.0;
    g->entry_vertical_bank_magnitude=0.0;
    g->entry_demand_bank=0.0;
    g->entry_lateral_required_bank=0.0;

    g->entry_planning_deferred=false;
    g->entry_planning_needed=false;
    g->entry_lateral_infeasible=false;
    g->entry_committed_infeasible=false;
    g->entry_supervision_boundary_missed=false;

    entry_exec_reset(&g->entry_exec);
    memset(&g->entry_lateral,0,sizeof(g->entry_lateral));
    memset(&g->entry_energy,0,sizeof(g->entry_energy));
    taem_exec_reset(&g->taem_exec);

    g->entry_alpha_has_target=false;
    g->entry_alpha_has_modulation=false;
    g->entry_alpha_target=0.0;
    g->entry_alpha_modulation=0.0;
    g->entry_alpha_drag_boost=0.0;
    g->entry_drag_ratio_valid=false;
    g->entry_drag_velocity_ratio=0.0;
    g->entry_bank_authority_acquired=false;
    g->entry_supervision_valid=false;
    g->entry_supervision_mode=ENTRY_SUPERVISION_PASS_THROUGH;
}

static void reset_entry_s_turn_program(GuidanceMachine*g){
    if(!g)return;
    memset(&g->entry_s_turn_plan,0,sizeof(g->entry_s_turn_plan));
    g->entry_bank_authority_acquired=false;
    g->entry_supervision_ut=-INFINITY;
    g->entry_target_side_latched=false;
    g->entry_target_side=0.0;
}

       void reset_taem_handoff_state(GuidanceMachine*g){
    if(!g)return;
    g->taem_interface_captured=false;
    g->taem_safety_handoff=false;
}
void guidance_machine_init(GuidanceMachine*g){memset(g,0,sizeof(*g));entry_exec_reset(&g->entry_exec);taem_exec_reset(&g->taem_exec);g->entry_supervision_ut=-INFINITY;g->phase=PHASE_IDLE;g->s_turn_sign=1;g->hac_side=1;g->terminal_vertical_stage=TERMINAL_TRAJECTORY_CAPTURE;g->terminal_positive_aoa_rate_ema=0.0;g->terminal_pitch_response_delay_ema=0.0;robust_pid_init(&g->entry_energy_pid,.00048,.0000035,.00012,180000,2.5);robust_pid_init(&g->taem_altitude_pid,.0026,.000035,.005,8000,.8);robust_pid_init(&g->final_altitude_pid,.005,.00006,.012,2500,.55);robust_pid_init(&g->speed_pid,.02,.0018,.006,60,.7);robust_pid_init(&g->flare_sink_pid,.75,.08,.22,12,.35);speedbrake_controller_reset(&g->speedbrake_controller,false);}

/* Executable-plan identity is guidance state, not a logging-side guess. A new
   persistent segment receives a monotonic plan id only after all plan shaping is
   complete; the previous executable segment is retained as explicit lineage. */
       void control_plan_assign_lineage(GuidanceMachine*g,EntryControlPlan*plan,
        const EntryControlPlan*parent){
    if(!g||!plan||!plan->valid||plan->plan_id)return;
    uint64_t next=++g->control_plan_sequence;
    if(next==0)next=++g->control_plan_sequence;
    plan->plan_id=next;
    plan->plan_version=1;
    plan->parent_plan_id=parent?parent->plan_id:0;
    plan->parent_plan_version=parent?parent->plan_version:0;
}
void guidance_set_entry_predictor_models(GuidanceMachine*g,const AerodynamicEnvelope*env,const TrajectoryCalibrationModel*cal){
    if(!g)return;
    if(!env||!cal){g->entry_predictor_models_valid=false;return;}
    g->entry_predictor_envelope=*env;
    g->entry_predictor_calibration=*cal;
    g->entry_predictor_models_valid=true;
}
void guidance_initialize_reentry_continuation(GuidanceMachine*g,const Telemetry*t,
        const PlanetModel*p,const LandingConfiguration*cfg,double initial_s_turn_sign,
        bool late_terminal_test,const AerodynamicEnvelope*env,const TrajectoryCalibrationModel*cal){
    if(!g||!t||!p||!cfg)return;
    guidance_machine_init(g);
    guidance_set_engaged(g,true);
    if(env&&cal)guidance_set_entry_predictor_models(g,env,cal);
    g->s_turn_sign=initial_s_turn_sign<0.0?-1.0:1.0;
    g->has_burn_command_started=true;
    g->deorbit_burn_completed=true;
    double entry_alt=entry_guidance_start_altitude(p,&cfg->guidance);
    g->atmospheric_interface_crossed=(late_terminal_test||t->mean_altitude<=entry_alt-500.0)&&
        t->vertical_speed<0.0;
    g->phase=late_terminal_test?PHASE_TAEM:
        (g->atmospheric_interface_crossed?PHASE_ENTRY_ENERGY:PHASE_ENTRY_INTERFACE);
    /* A saved atmospheric checkpoint has no durable MM304 leg history.  Mark only
       that restart bootstrap so its first fresh reversal cannot execute tens of
       seconds earlier than the uninterrupted leg solely because the old committed
       schedule was not serialized with the KSP vessel state. */
    g->entry_continuation_bootstrap=g->atmospheric_interface_crossed&&!late_terminal_test;
}
void guidance_set_engaged(GuidanceMachine*g,bool e){g->automation_engaged=e;if(e){g->aborted=false;g->abort_reason[0]=0;g->paused=false;if(g->phase==PHASE_IDLE||g->phase==PHASE_PAUSED||g->phase==PHASE_ABORT)g->phase=PHASE_COAST;}else{g->phase=PHASE_IDLE;reset_controllers(g);reset_taem_handoff_state(g);reset_entry_s_turn_program(g);speedbrake_controller_reset(&g->speedbrake_controller,false);}}
void guidance_set_paused(GuidanceMachine*g,bool p){g->paused=p;if(p){g->phase=PHASE_PAUSED;reset_limiters(g);}}
void guidance_abort(GuidanceMachine*g){g->aborted=true;g->paused=false;g->automation_engaged=false;g->phase=PHASE_ABORT;reset_controllers(g);reset_taem_handoff_state(g);reset_entry_s_turn_program(g);speedbrake_controller_reset(&g->speedbrake_controller,false);}
void guidance_reset_plan(GuidanceMachine*g){g->delivered_delta_v=0;g->has_burn_command_started=false;g->has_burn_progress_watch=false;g->burn_active_elapsed=0;g->burn_progress_watch_ut=0;g->burn_progress_watch_delta_v=0;g->deorbit_burn_completed=false;g->atmospheric_interface_crossed=false;g->has_previous_ut=false;g->hac_side_selected=false;g->final_approach_captured=false;g->has_s_turn_leg_started=false;g->has_s_turn_reversal_requested=false;g->airbrakes_deployed=false;reset_controllers(g);reset_taem_handoff_state(g);reset_entry_s_turn_program(g);speedbrake_controller_reset(&g->speedbrake_controller,false);if(g->automation_engaged)g->phase=PHASE_COAST;}
double guidance_entry_leg_elapsed(const GuidanceMachine*g,double ut){return g->phase==PHASE_ENTRY_ENERGY&&g->has_s_turn_leg_started?fmax(0,ut-g->s_turn_leg_started_ut):0;}

double dynamic_bank_limit(const Telemetry*t,const VehicleProfile*v){
    return entry_bank_authority_limit(t->true_air_speed,t->dynamic_pressure,t->g_force,
        v,v->maximum_bank_angle);
}

double terminal_lateral_bank_limit(const Telemetry*t,const VehicleProfile*v){
    double limit=dynamic_bank_limit(t,v);
    /* Applies at any height once in Final: a vehicle below minimum-safe speed
       while still high needs lateral authority most, not least. */
    if(limit>0.0||!t||!v||!isfinite(t->radar_altitude)||
       !isfinite(t->true_air_speed)||!isfinite(t->dynamic_pressure)||
       !isfinite(t->g_force)||t->dynamic_pressure<=DBL_MIN||
       t->dynamic_pressure>v->maximum_dynamic_pressure||t->g_force>v->maximum_g_load||
       !(v->minimum_safe_speed>v->touchdown_speed)||!(v->touchdown_speed>0.0))
        return limit;

    /* Entry protects stall margin by removing bank below minimum-safe speed.
       Final still needs a small lateral correction on the way to the lower
       touchdown target. Fade that authority smoothly with speed, while keeping
       the same measured pressure and load-factor guards. */
    double floor=0.70*v->touchdown_speed;
    double u=clampd((t->true_air_speed-floor)/(v->minimum_safe_speed-floor),0.0,1.0);
    double fade=u*u*(3.0-2.0*u);
    return fmin(fmin(12.0,v->maximum_bank_angle)*fade,v->maximum_bank_angle);
}

                                                                                          
                                                                                 
                               

static double entry_model_best_glide_aoa(const Telemetry*t,const VehicleProfile*v){
    if(t&&isfinite(t->calibrated_best_glide_angle_of_attack)&&
       t->calibrated_best_glide_angle_of_attack>=0.0)
        return clampd(t->calibrated_best_glide_angle_of_attack,0.0,
            v->maximum_angle_of_attack);

    const int samples=2*DBL_MANT_DIG;
    double best=0.0,best_ld=-INFINITY;
    for(int i=0;i<=samples;i++){
        double aoa=v->maximum_angle_of_attack*(double)i/(double)samples;
        double lf=0.0,df=0.0;
        aerodynamic_force_factors_mach(t?t->mach:0.0,aoa,v,&lf,&df);
        if(!(df>DBL_MIN)||!isfinite(lf)||!isfinite(df))continue;
        double ld=fabs(lf)/df;
        if(ld>best_ld){best_ld=ld;best=aoa;}
    }
    return best;
}

       double entry_survivability_recovery_aoa(const GuidanceMachine*g,
        const Telemetry*t,const PlanetModel*p,const LandingConfiguration*cfg,
        const ControlAuthorityEnvelope*authority){
    (void)g;
    const VehicleProfile*v=&cfg->vehicle;
    double best_glide=entry_model_best_glide_aoa(t,v);
    bool speed_or_stall=authority->speed.margin<0.0||
        (authority->stall_observed&&authority->stall.margin<0.0);
    bool load_exceeded=authority->load.margin<0.0;
    if(speed_or_stall||load_exceeded)return best_glide;

    if(authority->dynamic_pressure.margin<0.0){
        /*
         * Above q-bar limit, choose the drag-maximizing incidence that still
         * fits the configured normal-load envelope.  Candidate lift is scaled
         * from the measured current lift when available, so this is an
         * optimization against current vehicle physics rather than a fixed
         * high-AoA override.
         */
        double current_lf=0.0,current_df=0.0;
        aerodynamic_force_factors_mach(t->mach,fabs(t->angle_of_attack),v,
            &current_lf,&current_df);
        double measured_lift=t->mass>DBL_MIN&&isfinite(t->lift_force)?
            fmax(0.0,t->lift_force/t->mass):0.0;
        double gravity=planet_surface_gravity(p);
        double load_limit=v->maximum_g_load*gravity;
        const int samples=2*DBL_MANT_DIG;
        double best=best_glide,best_drag=-INFINITY;
        for(int i=0;i<=samples;i++){
            double aoa=v->maximum_angle_of_attack*(double)i/(double)samples;
            double lf=0.0,df=0.0;
            aerodynamic_force_factors_mach(t->mach,aoa,v,&lf,&df);
            double predicted_lift=(measured_lift>0.0&&fabs(current_lf)>DBL_MIN)?
                measured_lift*fabs(lf/current_lf):0.0;
            if(predicted_lift>load_limit)continue;
            if(isfinite(df)&&df>best_drag){best_drag=df;best=aoa;}
        }
        return best;
    }
    return best_glide;
}
       void terminal_speedbrake_closed(GuidanceMachine*g){
    /*
     * Mission contract: STS-N atmospheric recovery and landing are flown
     * without speedbrakes/airbrakes.  Keep both the command latch and its
     * controller state closed so no later energy branch can resurrect them.
     */
    if(!g)return;
    g->airbrakes_deployed=false;
    g->speedbrake_controller.deployed=false;
    g->speedbrake_controller.initialized=true;
}
       GuidanceCommand atmospheric(const Telemetry*t,double heading,double roll,const VehicleProfile*v,double throttle,bool air,ControlProfile profile){GuidanceCommand c;guidance_command_init(&c);c.autopilot_engaged=true;c.target_heading=norm_deg(heading);c.target_roll=clampd(roll,-v->maximum_bank_angle,v->maximum_bank_angle);c.target_throttle=clampd(throttle,0,1);c.gear=t->gear;c.airbrakes=air;c.navball_speed_mode=SPEED_SURFACE;c.control_profile=profile;return c;}
       void aerodynamic_pitch_target(GuidanceCommand*c,const Telemetry*t,const VehicleProfile*v,double surface_pitch){
    c->has_target_aoa=true;
    c->target_aoa=clampd(surface_pitch-t->flight_path_angle,0,v->maximum_angle_of_attack);
    c->target_pitch=t->flight_path_angle+c->target_aoa;
}
       GuidanceResult stabilized(GuidanceMachine*g,GuidanceResult r,const Telemetry*t,const VehicleProfile*v,const GuidanceSettings*s,double dt){
    if(!r.command.autopilot_engaged||r.command.use_inertial_direction){
        reset_limiters(g);
        return r;
    }
    /*
     * Guidance owns the executable attitude reference as well as geometry and
     * safety envelopes. FlightControl still closes the surface/torque loop,
     * but it must not receive a discontinuous AoA or bank target that assumes
     * an instantaneous shuttle. The same bounded command is consumed by the
     * native FCS and by ShuttleSim, so simulator-only behavior cannot diverge
     * from the real guidance contract.
     */
    if(r.phase==PHASE_ATTITUDE_RECOVERY){
        reset_limiters(g);
        return r;
    }

    double aoa_ceiling=v->maximum_angle_of_attack;
    if(r.phase==PHASE_ENTRY_ENERGY&&r.command.has_target_aoa&&
       isfinite(r.command.target_aoa)&&r.command.target_aoa>aoa_ceiling)
        aoa_ceiling=entry_final_s_turn_aoa_ceiling(v);

    bool aerodynamic_phase=r.phase==PHASE_ENTRY_ENERGY||
        r.phase==PHASE_TAEM||r.phase==PHASE_HEADING_ALIGNMENT||
        r.phase==PHASE_FINAL||r.phase==PHASE_FLARE;

    if(aerodynamic_phase){
        double target_aoa=r.command.has_target_aoa?
            r.command.target_aoa:r.command.target_pitch-t->flight_path_angle;
        target_aoa=clampd(target_aoa,0.0,aoa_ceiling);
        r.command.has_target_aoa=true;
        r.command.target_aoa=target_aoa;
        r.command.target_pitch=t->flight_path_angle+target_aoa;
    }else{
        double target_aoa=r.command.target_pitch-t->flight_path_angle;
        target_aoa=clampd(target_aoa,0.0,aoa_ceiling);
        r.command.target_pitch=t->flight_path_angle+target_aoa;
    }

    double bank_limit=(r.phase==PHASE_FINAL||r.phase==PHASE_FLARE)?
        terminal_lateral_bank_limit(t,v):dynamic_bank_limit(t,v);
    r.command.target_roll=clampd(norm_signed_deg(r.command.target_roll),
        -bank_limit,bank_limit);
    r.command.target_heading=norm_deg(r.command.target_heading);
    r.command.target_throttle=clampd(r.command.target_throttle,0.0,1.0);

    double pitch_reference=r.command.has_target_aoa?
        r.command.target_aoa:r.command.target_pitch;
    bool pitch_reference_is_aoa=r.command.has_target_aoa;
    GuidanceAttitudeLimits pitch_limits=stabilized_attitude_limits(
        t,s,r.phase,true);
    /* A committed MM305 route was replay-qualified by applying the native
     * tracker's commands directly to the vehicle's attitude dynamics, which
     * already bound rate and acceleration.  A second guidance-side limiter in
     * the loop adds lag the qualified route never had and drives a lateral
     * limit cycle, so the committed route bypasses it (state stays seeded). */
    bool mm305_native_window=r.phase==PHASE_TAEM&&g->mm305_route_committed&&
        g->mm305_route.valid&&!g->mm305_hac_exit_reached;
    seed_stabilized_limiter(&g->pitch_limiter,
        pitch_reference_is_aoa
            ? clampd(t->angle_of_attack,0.0,aoa_ceiling)
            : t->pitch,
        clampd(controlled_aoa_rate(t),-pitch_limits.rate_deg_s,
            pitch_limits.rate_deg_s),false);
    double limited_pitch=jerk_update(&g->pitch_limiter,pitch_reference,
        pitch_limits.rate_deg_s,pitch_limits.accel_deg_s2,dt);
    if(mm305_native_window&&isfinite(pitch_reference))limited_pitch=pitch_reference;
    if(!isfinite(limited_pitch))
        limited_pitch=clampd(t->angle_of_attack,0.0,aoa_ceiling);
    if(pitch_reference_is_aoa)
        limited_pitch=clampd(limited_pitch,0.0,aoa_ceiling);
    g->pitch_limiter.value=limited_pitch;
    if(pitch_reference_is_aoa) {
        r.command.target_pitch=t->flight_path_angle+limited_pitch;
        r.command.target_aoa=limited_pitch;
    } else {
        r.command.target_pitch=limited_pitch;
    }

    double roll_reference=norm_signed_deg(r.command.target_roll);
    GuidanceAttitudeLimits roll_limits=stabilized_attitude_limits(
        t,s,r.phase,false);
    seed_stabilized_limiter(&g->roll_limiter,
        norm_signed_deg(t->roll),
        clampd(controlled_roll_rate(t),-roll_limits.rate_deg_s,
            roll_limits.rate_deg_s),true);
    double limited_roll=jerk_angle_update(&g->roll_limiter,roll_reference,
        roll_limits.rate_deg_s,roll_limits.accel_deg_s2,dt);
    if(mm305_native_window&&isfinite(roll_reference))limited_roll=roll_reference;
    if(!isfinite(limited_roll))
        limited_roll=norm_signed_deg(t->roll);
    r.command.target_roll=clampd(norm_signed_deg(limited_roll),
        -bank_limit,bank_limit);
    g->roll_limiter.value=norm_deg(r.command.target_roll);

    double heading_reference=norm_deg(r.command.target_heading);
    if(g->heading_limiter.has_value&&dt>0.0&&isfinite(dt))
        g->heading_limiter.rate=norm_signed_deg(heading_reference-
            norm_deg(g->heading_limiter.value))/dt;
    else g->heading_limiter.rate=0.0;
    g->heading_limiter.value=heading_reference;
    g->heading_limiter.has_value=true;
    /* While the committed MM305 route bypasses the limiters their internal
       rate state goes stale; leave them unseeded so the first limited tick
       (normally Final) starts from the measured attitude and rate instead of
       resuming a stale jerk state and overshooting by tens of degrees. */
    if(mm305_native_window){
        g->roll_limiter.has_value=false;
        g->pitch_limiter.has_value=false;
    }

    g->throttle_limiter.value=r.command.target_throttle;
    g->throttle_limiter.has_value=true;
    g->last_stabilized_phase=r.phase;
    g->has_last_stabilized_phase=true;
    return r;
}
       GuidanceCommand entry_capture(const Telemetry*t,const VehicleState*state,const VehicleProfile*v){GuidanceCommand c;guidance_command_init(&c);c.autopilot_engaged=true;c.heading_control_enabled=true;double capture_aoa=entry_low_q_protective_aoa_floor(t->dynamic_pressure,v);c.has_target_aoa=true;c.target_aoa=capture_aoa;c.target_pitch=t->flight_path_angle+capture_aoa;c.target_heading=norm_deg(t->ground_track_heading);c.target_roll=0;c.control_profile=PROFILE_ENTRY;/* In vacuum, AoA/heading Euler errors are not a stable attitude target for a 180-degree retrograde-to-prograde flip. Request an inertial prograde vector instead; krpc_apply routes this particular ENTRY command through the bounded native quaternion/RCS capture law. Once q is measurable, return to the normal aerodynamic entry controller. */if(state&&fmax(0,t->dynamic_pressure)<0.5){c.use_inertial_direction=true;c.inertial_direction=vnorm(state->velocity,v3(1,0,0));c.navball_speed_mode=SPEED_ORBIT;}else{c.use_inertial_direction=false;c.navball_speed_mode=SPEED_SURFACE;}return c;}
       void request_side(GuidanceMachine*g,double sign,double ut){if(sign==g->s_turn_sign)return;g->s_turn_sign=sign;g->has_s_turn_leg_started=false;g->s_turn_reversal_requested_ut=ut;g->has_s_turn_reversal_requested=true;}

/* The reversal dwell belongs to a real aerodynamic S-turn leg, not to side
   bookkeeping. Near-vacuum roll overshoot can otherwise pre-satisfy the dwell
   hundreds of seconds before useful bank authority exists. A leg is therefore
   established only after either the executive load transition or the same physical
   aerodynamic-authority gate that permits authoritative PREENTRY bank work, while a
   meaningful same-side bank command is measurably captured. Losing that capture or
   transient PREENTRY authority restarts the dwell without changing the planned side/event. */
       void entry_update_s_turn_leg_capture(GuidanceMachine*g,const Telemetry*t,double bank,const VehicleProfile*v){
    if(!g||!t||!v)return;
    double magnitude=fabs(bank);
    double planned_sign=bank>=0?1.0:-1.0;

    double threshold=fmin(12.0,fmax(4.0,magnitude*.35));
    bool executive_loaded=g->entry_exec.initialized&&g->entry_exec.phase!=ENTRY_PHASE_PREENTRY;
    bool aerodynamic_authority=entry_s_turn_bank_authority_available(t->dynamic_pressure,
        t->true_air_speed,t->stall_fraction,t->g_force,v);
    bool loaded=executive_loaded||aerodynamic_authority;
    bool meaningful=loaded&&magnitude>=threshold;
    double actual=norm_signed_deg(t->roll);
    bool captured=meaningful&&planned_sign*actual>0.0&&fabs(actual)>=threshold;
    {   /* KSP_LANDER_SETUP_REVERSAL_TRACE: why the S-turn leg is (not) captured. */
        static int enabled=-1;static _Thread_local int last=-1;
        if(enabled<0){const char*e=getenv("KSP_LANDER_SETUP_REVERSAL_TRACE");enabled=e&&*e&&strcmp(e,"0");}
        int code=!loaded?1:!meaningful?2:!(planned_sign*actual>0.0&&fabs(actual)>=threshold)?3:
            planned_sign!=g->s_turn_sign?4:0;
        if(enabled&&code!=last){last=code;
            static const char*names[]={"captured","unloaded","small_cmd","roll_not_captured","cmd_side_mismatch"};
            fprintf(stderr,"LEG_CAPTURE_TRACE shadow=%d ut=%.1f V=%.0f reason=%s cmd=%.1f roll=%.1f thr=%.1f side=%+.0f sched=%d sut=%.1f\n",
                g->diagnostic_shadow,t->ut,t->surface_speed,names[code],bank,actual,threshold,g->s_turn_sign,
                g->entry_reversal_scheduled,g->entry_reversal_ut);}
    }
    /* A near-zero bank command (energy trim near the corridor centre) is not
       loss of capture: the owned side is still physically held. Resetting here
       made the scheduled reversal permanently not-due until the command grew
       again at V~220 m/s. Only an unloaded vehicle or an off-side roll restarts. */
    if(!captured&&g->has_s_turn_leg_started&&loaded&&magnitude<threshold&&
       g->s_turn_sign*actual>0.0)return;
    if(!captured){
        g->has_s_turn_leg_started=false;
        g->s_turn_leg_started_ut=0.0;
        return;
    }
    /* Measured roll capture confirms that an already-owned side is physically
       established; it does not authorize a new side. Side ownership changes only
       through the explicit initial-side selector or a committed reversal event.
       Otherwise a stray opposite bank command can become self-authorizing as soon
       as the airframe rolls through zero, bypassing the final-arc geometry gate. */
    if(planned_sign!=g->s_turn_sign){
        g->has_s_turn_leg_started=false;
        g->s_turn_leg_started_ut=0.0;
        return;
    }
    if(!g->has_s_turn_leg_started){
        g->s_turn_leg_started_ut=t->ut;
        g->has_s_turn_leg_started=true;
        g->has_s_turn_reversal_requested=false;
        if(g->entry_final_reversal_pending){
            g->entry_final_reversal_pending=false;
            g->entry_final_reversal_completed=true;
        }
    }
}
void guidance_update_entry_reversal(GuidanceMachine*g,const EntryControlPlan*plan,double ut){
    /* Reversal metadata is part of the accepted policy, not an independent
       timer. Preserve a deadline only while its bank/AoA policy is unchanged. */
    bool usable=plan&&plan->valid&&!g->entry_final_reversal_pending&&
        !g->entry_final_reversal_completed&&plan->has_planned_reversal&&
        fabs(plan->target_bank)>=4&&isfinite(plan->planned_reversal_ut)&&
        plan->planned_reversal_ut>=ut-1e-6&&plan->planned_reversal_sign*plan->target_bank<0;
    double event_ut=usable?fmax(ut,plan->planned_reversal_ut):0;
    if(usable&&g->entry_reversal_scheduled&&g->entry_reversal_sign*plan->planned_reversal_sign>0&&
            fabs(g->entry_control_bank-plan->target_bank)<1e-6&&
            fabs(g->entry_control_aoa-plan->target_aoa)<1e-6&&
            g->entry_reversal_is_final==plan->planned_reversal_is_final)
        event_ut=fmin(event_ut,g->entry_reversal_ut);
    g->entry_reversal_scheduled=usable;
    g->entry_reversal_is_final=usable&&plan->planned_reversal_is_final;
    if(usable){
        g->entry_reversal_ut=event_ut;g->entry_reversal_range=plan->planned_reversal_range;
        g->entry_reversal_sign=plan->planned_reversal_sign;
        g->entry_reversal_bank=fabs(plan->target_bank);
    }
}
