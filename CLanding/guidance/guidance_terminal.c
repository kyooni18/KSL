#include "guidance_internal.h"

#include <float.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* Translation-unit private helpers. */
static GuidanceResult terminal_final_test_guidance(GuidanceMachine*g,const Telemetry*t,double course,
        const PlanetModel*p,AerodynamicModel aero,const LandingConfiguration*cfg,double dt);
static GuidanceResult terminal_guidance(GuidanceMachine *g, const Telemetry *t, const VehicleState *state, double course,
        const PlanetModel *p, AerodynamicModel aero, const LandingConfiguration *cfg, double dt,
        const TerminalModel *terminal_model);

static GuidanceResult terminal_final_test_guidance(GuidanceMachine*g,const Telemetry*t,double course,
        const PlanetModel*p,AerodynamicModel aero,const LandingConfiguration*cfg,double dt){
    const GuidanceSettings*s=&cfg->guidance;
    const VehicleProfile*v=&cfg->vehicle;

    /* TAEM -> Final ownership is one-way.  Once final intercept has been
       delivered, do not re-run TAEM admission while Preflare/Inner Final/Flare
       are actively changing sink and speed according to their own certified
       envelopes. */
    if(g->final_approach_captured&&g->taem_exec.taem_complete){
        Trajectory ref;
        trajectory_init(&ref);
        GuidanceResult result=terminal_approach_sequence(g,t,course,p,aero,cfg,&ref,dt);
        trajectory_clear(&ref);
        return result;
    }

    TerminalPreflarePlan approach_plan={0};
    bool approach=terminal_outer_gate(g,t,course,p,aero,cfg,&approach_plan);
    TaemTerminalContract contract=terminal_delivery_contract(g,t,course,p,cfg,&approach_plan);
    TaemTerminalEvaluation evaluation=taem_exec_evaluate_terminal_contract(&contract);
    if(!approach||!evaluation.valid||!evaluation.feasible){
        EnergyPathEnvelope conditioning_energy={0};
        double projected_end_speed=NAN,conditioning_slope=NAN;
        if(!terminal_final_conditioning_reachable(g,t,course,p,aero,cfg,
                &conditioning_energy,&projected_end_speed,&conditioning_slope)){
            char reason[320];
            snprintf(reason,sizeof(reason),
                "Post-HAC final alignment exhausted its physical conditioning envelope: approach=%d preflare=%s terminal=%s.",
                approach?1:0,approach_plan.reject_reason?approach_plan.reject_reason:"none",
                taem_terminal_block_reason_string(evaluation.block_reason));
            return terminal_abort(g,reason);
        }
        /* Execute the same runway-line/FPA solution that was just certified.
           Calling final_guidance() here used the normal speed-dissipation trim,
           which is intentionally more draggy than the conditioning proof and
           could consume the preflare reserve before Final was admitted. */
        HACGuidance line=terminal_runway_line_path_guidance(t,&cfg->site,s,
            planet_surface_gravity(p),course);
        double conditioning_aoa=terminal_fpa_force_aoa(t,p,aero,v,-conditioning_slope);
        GuidanceCommand command=atmospheric(t,line.heading,line.bank,v,0.0,false,PROFILE_TAEM);
        command.heading_control_enabled=true;
        command.has_target_aoa=true;
        command.target_aoa=conditioning_aoa;
        command.target_pitch=t->flight_path_angle+conditioning_aoa;
        g->gear_command_latched=g->gear_command_latched||t->gear||
            t->radar_altitude<=s->gear_deployment_altitude;
        command.gear=t->gear||g->gear_command_latched;
        terminal_speedbrake_closed(g);
        GuidanceResult result=stabilized(g,result_make(PHASE_TAEM,command,NULL,NULL),
            t,v,s,dt);
        g->phase=PHASE_TAEM;
        snprintf(result.status,sizeof(result.status),
            "Post-HAC final alignment energy conditioning: V %.1f m/s, projected %.1f m/s at flare boundary, minimum %.1f m/s, slope %.1f deg, AoA %.1f deg, energy margin %+.0f J/kg.",
            t->true_air_speed,projected_end_speed,v->minimum_safe_speed,conditioning_slope,
            conditioning_aoa,conditioning_energy.energy.margin);
        return result;
    }
    (void)taem_exec_sync_with_contract(g,t,s,p,aero,cfg,&contract,false,false);
    evaluation=g->taem_exec.terminal_evaluation;
    if(!g->final_approach_captured&&t->runway_along_track<0.0&&
       g->taem_exec.phase==TAEM_PHASE_FINAL_INTERCEPT){
        (void)taem_exec_sync_with_contract(g,t,s,p,aero,cfg,&contract,true,true);
        if(g->taem_exec.taem_complete){
            g->final_approach_captured=true;
            terminal_store_preflare_plan(g,&approach_plan);
            /* engageFinalTest admits only a runway-aligned post-HAC state and TAEM
               completion has already proved the final-intercept contract.  Do not
               burn another full control-response interval in a duplicate capture
               dwell while a valid preflare energy window is closing. */
            terminal_set_stage(g,TERMINAL_OUTER_FINAL,t->ut);
            robust_pid_reset(&g->final_altitude_pid);
            robust_pid_reset(&g->speed_pid);
            robust_pid_reset(&g->flare_sink_pid);
        }
    }
    Trajectory ref;
    trajectory_init(&ref);
    GuidanceResult result;
    if(g->final_approach_captured&&g->taem_exec.taem_complete){
        result=terminal_approach_sequence(g,t,course,p,aero,cfg,&ref,dt);
    }else{
        HACGuidance line=terminal_runway_line_path_guidance(t,&cfg->site,s,planet_surface_gravity(p),course);
        GuidanceCommand command=atmospheric(t,line.heading,line.bank,v,0.0,true,PROFILE_TAEM);
        command.heading_control_enabled=true;
        g->phase=PHASE_TAEM;
        result=stabilized(g,result_make(PHASE_TAEM,command,
            "Post-HAC final alignment: runway-line capture and TAEM final-intercept admission.",
            NULL),t,v,s,dt);
    }
    trajectory_clear(&ref);
    return result;
}
static GuidanceResult terminal_guidance(GuidanceMachine *g, const Telemetry *t,
        const VehicleState *state, double course, const PlanetModel *p,
        AerodynamicModel aero, const LandingConfiguration *cfg, double dt,
        const TerminalModel *terminal_model) {
    const GuidanceSettings *s=&cfg->guidance;
    const VehicleProfile *v=&cfg->vehicle;
    if (g->terminal_final_test_mode) {
        g->terminal_glide_mode=false;
        GuidanceResult result=terminal_final_test_guidance(g,t,course,p,aero,cfg,dt);
        g->terminal_glide_mode=true;
        return result;
    }

    bool reported_landed=!strcasecmp(t->vessel_situation,"landed");
    if (g->final_approach_captured &&
        (g->phase==PHASE_ROLLOUT || g->phase==PHASE_COMPLETE || reported_landed)) {
        if (reported_landed && !terminal_runway_contact_position(t,cfg))
            return terminal_abort(g,"Touchdown occurred outside the runway 09 contact envelope.");
        if (reported_landed && !t->gear)
            return terminal_abort(g,"Touchdown occurred without confirmed landing-gear deployment.");
        if (reported_landed) {
            g->ground_contact_latched=true;
            g->gear_command_latched=true;
            terminal_set_stage(g,TERMINAL_GROUND,t->ut);
        }
        Trajectory reference; trajectory_init(&reference);
        GuidanceResult result=touchdown(g,t,cfg,&reference);
        trajectory_clear(&reference);
        return result;
    }

    bool recovery_was_active=g->attitude_recovery;
    if (control_recovery_needed(g,t,s,v,dt)) {
        if (!recovery_was_active && (g->terminal_path_committed || g->hac_side_selected))
            terminal_invalidate_frozen_path(g);
        if (g->recovery_duration>=15.0)
            return terminal_abort(g,"Control departure: attitude recovery exceeded 15 s without stabilizing.");
        if (t->radar_altitude<fmax(300.0,-t->vertical_speed*8.0))
            return terminal_abort(g,"Control departure: insufficient height remains to complete attitude recovery.");
        g->phase=PHASE_ATTITUDE_RECOVERY;
        GuidanceCommand command=atmospheric(t,g->recovery_heading,0.0,v,0.0,
            false,PROFILE_RECOVERY);
        command.heading_control_enabled=false;
        double aoa=g->recovery_aoa>0.0?g->recovery_aoa:
            clampd(v->entry_angle_of_attack,10.0,18.0);
        command.has_target_aoa=true;
        command.target_aoa=aoa;
        command.target_pitch=clampd(t->flight_path_angle+aoa,-12.0,25.0);
        return stabilized(g,result_make(g->phase,command,
            "Attitude recovery: unloading and leveling wings; terminal progression inhibited.",
            "Sustained saturation/attitude error or excessive roll rate."),t,v,s,dt);
    }
    if (recovery_was_active && !g->attitude_recovery) {
        if (terminal_entry_recovery_available(g,t,v,s))
            return terminal_return_to_entry(g,t,state,course,p,aero,cfg,dt);
        terminal_invalidate_frozen_path(g);
    }

    terminal_energy_observe(g,t,p);
    if (!g->terminal_region_entered && !g->final_approach_captured) {
        g->phase=PHASE_ENTRY_ENERGY;
        GuidanceResult entry=entry_program_guidance(g,t,state,course,p,aero,cfg,dt);
        bool strict_handoff=mm305_acquisition_ready(t,p,cfg)&&
            g->entry_exec.entry_complete;
        g->taem_interface_captured=strict_handoff;
        if (!strict_handoff) {
            double low=0.0,high=0.0;
            entry_taem_handoff_altitude_bounds(s,&low,&high);
            double sound=planet_atmospheric_speed_of_sound(p,t->mean_altitude);
            double mach=isfinite(t->mach)&&t->mach>0.0?t->mach:
                (isfinite(sound)&&sound>DBL_MIN?t->true_air_speed/sound:NAN);
            double minimum_mach=s->mm305_target_mach-s->mm305_mach_half_width;
            bool handoff_lost=t->true_air_speed<v->minimum_safe_speed||
                t->mean_altitude<cfg->site.altitude+100.0;
            (void)low;(void)high;(void)mach;(void)minimum_mach;
            if (handoff_lost)
                return terminal_abort(g,
                    "MM304 flight recovery margin was exhausted before TAEM acquisition.");
            return entry;
        }

        double loss=g->terminal_energy_loss_accel_ema;
        double speed_loss=g->terminal_speed_loss_accel_ema;
        bool rehearsal=g->terminal_rehearsal_mode;
        terminal_glide_initialize(g,v,s);
        g->terminal_energy_loss_accel_ema=loss;
        g->terminal_speed_loss_accel_ema=speed_loss;
        g->terminal_rehearsal_mode=rehearsal;
        g->terminal_region_entered=true;
        g->terminal_test_capture_active=false;
        g->taem_safety_handoff=false;
        g->terminal_path_kind=TERMINAL_PATH_NONE;
        g->terminal_path_committed=false;
        g->mm305_route=(TaemRoute){0};
        g->mm305_route_cursor=0;
        g->mm305_route_committed=false;
        g->mm305_hac_exit_reached=false;
        g->mm305_model_snapshot_id=0;
        g->hac_side_selected=false;
        g->hac_captured=false;
        g->hac_completed=false;
        g->hac_progress_valid=false;
        g->hac_remaining=0.0;
        g->hac_radius=s->hac_radius;
        g->hac_side=terminal_default_hac_side(t,&cfg->site,course);
        g->terminal_reference_heading=course;
        g->terminal_reference_bank=0.0;
        g->terminal_reference_fpa=t->flight_path_angle;
        g->terminal_reference_aoa=clampd(t->angle_of_attack,0.0,
            v->maximum_angle_of_attack);
        g->phase=PHASE_TAEM;
        if (!taem_exec_enter(g,t,s,p,aero,cfg,false))
            return terminal_abort(g,
                "MM304 ownership handoff was qualified, but the TAEM executive could not enter Path Acquisition.");
        guidance_result_clear(&entry);
    }

    if (g->hac_completed || g->final_approach_captured)
        return terminal_final_test_guidance(g,t,course,p,aero,cfg,dt);
    if (!taem_exec_owns_vehicle(&g->taem_exec))
        return terminal_abort(g,
            "MM305 lost TAEM executive ownership before the HAC exit.");
    GuidanceResult result=taem_guidance_native(g,t,state,course,p,aero,cfg,
        terminal_model,dt);
    if (g->mm305_hac_exit_reached && result.phase!=PHASE_ABORT) {
        guidance_result_clear(&result);
        return terminal_final_test_guidance(g,t,course,p,aero,cfg,dt);
    }
    return result;
}

/*
 * Execute terminal guidance in the same runway-end frame selected by the
 * terminal planner.  Preview selection already evaluates RW09/RW27 as peers;
 * the control path must not silently fall back to the configured primary frame.
 */
static GuidanceResult terminal_guidance_selected(GuidanceMachine*g,
        const Telemetry*t,const VehicleState*state,double course,
        const PlanetModel*p,AerodynamicModel aero,
        const LandingConfiguration*cfg,double dt,const TerminalModel *terminal_model){
    /*
     * A qualified MM304 outlet is runway-end-specific.  Resolve that physical
     * boundary before the generic RW09/RW27 terminal preview: otherwise preview
     * can reframe a vehicle sitting in RW09's handoff disk into RW27 coordinates
     * and make the same state fail MM305 ownership.  The guidance, not the fixture,
     * chooses the end by testing the live pose against both configured outlets.
     */
    bool handoff_end_selected=false;
    if(cfg->site.allow_reciprocal_runway&&!g->runway_end_committed&&
       !g->terminal_region_entered){
        LandingSite primary=cfg->site;
        LandingSite reciprocal=runway_reciprocal_site(&primary,p->radius);
        double primary_path=decision_runway_end_path_score(g,t,p,cfg,&primary);
        double reciprocal_path=decision_runway_end_path_score(g,t,p,cfg,&reciprocal);
        if(isfinite(primary_path)||isfinite(reciprocal_path)){
            int best_end=(!isfinite(primary_path)||(isfinite(reciprocal_path)&&reciprocal_path<primary_path))?1:0;
            g->runway_end_index=best_end;
            g->runway_end_preview_valid=true;
            handoff_end_selected=true;
        }
    }

    LandingConfiguration selected=*cfg;
    Telemetry framed=*t;
    if(cfg->site.allow_reciprocal_runway&&g->runway_end_preview_valid&&
       g->runway_end_index==1)
        selected.site=runway_reciprocal_site(&cfg->site,p->radius);

    telemetry_reframe_runway(&framed,&selected.site,p->radius);
    double entry_ref=g->entry_reference_speed>0.0?
        g->entry_reference_speed:framed.true_air_speed;
    EntryTerminalDemand demand=entry_terminal_demand(framed.latitude,
        framed.mean_altitude,framed.range_to_site,framed.horizontal_speed,
        framed.course_to_site_error,framed.vertical_speed,framed.true_air_speed,
        entry_ref,p,&selected.vehicle,&selected.site,&selected.guidance);
    framed.energy_excess_range=-demand.projected_taem_range_error;

    GuidanceResult result=terminal_guidance(g,&framed,state,course,p,aero,
        &selected,dt,terminal_model);
    if(handoff_end_selected&&g->terminal_region_entered)
        g->runway_end_committed=true;
    if(g->terminal_path_committed)
        g->runway_end_committed=true;
    return result;
}

#define guidance_update guidance_update_impl
GuidanceResult guidance_update(GuidanceMachine*g,const Telemetry*t,const VehicleState*state,const DeorbitPlan*plan,const PlanetModel*p,AerodynamicModel aero,const LandingConfiguration*cfg,const TerminalModel *terminal_model){double dt=g->has_previous_ut?clampd(t->ut-g->previous_ut,0,1):.1;g->previous_ut=t->ut;g->has_previous_ut=true;if(g->aborted){GuidanceCommand c;guidance_command_init(&c);return result_make(PHASE_ABORT,c,g->abort_reason[0]?g->abort_reason:"Automation aborted. Manual control restored.",NULL);}if(!g->automation_engaged){g->phase=PHASE_IDLE;reset_limiters(g);GuidanceCommand c;guidance_command_init(&c);if(plan){if(plan->execution_qualified)return result_make(PHASE_IDLE,c,plan->execution_degraded?"Recoverable deorbit plan is ready. Engage guidance to begin execution.":"Robust deorbit plan is ready. Engage guidance to begin execution.",plan->execution_degraded?"The plan misses the preferred strict corridor but passed the guarded recovery envelope.":NULL);return result_make(PHASE_IDLE,c,"The current deorbit plan is preview-only and cannot be executed safely.","Replan for a later orbital opportunity or revise the vehicle/site model before engagement.");}return result_make(PHASE_IDLE,c,"Guidance is not engaged.",NULL);}if(g->paused){g->phase=PHASE_PAUSED;GuidanceCommand c;guidance_command_init(&c);return result_make(PHASE_PAUSED,c,"Guidance paused. Attitude hold released.",NULL);}const VehicleProfile*v=&cfg->vehicle;const GuidanceSettings*s=&cfg->guidance;double course=surface_course(state->position,state->velocity,planet_rotation_vector(p),p->north_axis,t->heading);if(g->terminal_glide_mode)return terminal_guidance_selected(g,t,state,course,p,aero,cfg,dt,terminal_model);if(!plan){g->phase=PHASE_PLANNING;GuidanceCommand c;guidance_command_init(&c);return result_make(PHASE_PLANNING,c,"A deorbit plan is required before engagement.",NULL);}double entry_alt=entry_guidance_start_altitude(p,s);if(!g->deorbit_burn_completed&&t->mean_altitude<=entry_alt&&t->vertical_speed<0)g->deorbit_burn_completed=true;double peri_rem=t->periapsis_altitude-plan->predicted_post_burn_periapsis_altitude;bool has_peri=plan->predicted_post_burn_periapsis_altitude>10000&&plan->predicted_post_burn_periapsis_altitude<p->atmosphere_depth&&isfinite(t->periapsis_altitude)&&t->periapsis_altitude>-p->radius*.5,peri_hit=g->has_burn_command_started&&has_peri&&peri_rem<=300,state_hit=g->has_burn_command_started&&plan->live_cutoff_capture_qualified;double dvtarget=plan->delta_v+(has_peri?8:0);if(g->has_burn_command_started&&(g->delivered_delta_v>=dvtarget||peri_hit||state_hit))g->deorbit_burn_completed=true;bool still=g->has_burn_command_started&&!g->deorbit_burn_completed&&g->delivered_delta_v<dvtarget&&!peri_hit&&!state_hit;double burnstart=plan->burn_ut-plan->estimated_burn_duration*.5,open=t->ut>=burnstart;if(!g->deorbit_burn_completed&&t->mean_altitude>entry_alt-4500&&(t->vertical_speed>=-20||still||open)){GuidanceCommand c;guidance_command_init(&c);c.autopilot_engaged=true;c.use_inertial_direction=true;c.inertial_direction=vnorm(vscale(state->velocity,-1),v3(-1,0,0));c.navball_speed_mode=SPEED_ORBIT;c.control_profile=PROFILE_ORBITAL;if(t->ut<burnstart-12){g->phase=PHASE_COAST;char st[128];int sec=(int)fmax(0,round(burnstart-t->ut));snprintf(st,sizeof(st),"Coasting while acquiring retrograde. T−%02d:%02d.",sec/60,sec%60);return result_make(g->phase,c,st,NULL);}if(t->ut<burnstart){g->phase=PHASE_BURN_SETUP;return result_make(g->phase,c,"Aligning retrograde for the deorbit burn.",NULL);}if(g->delivered_delta_v<dvtarget&&!peri_hit&&!state_hit){g->phase=PHASE_DEORBIT_BURN;double maxthr=s->deorbit_maximum_throttle,avail=fmax(.05,t->available_thrust*maxthr/fmax(t->mass,1)),meas=fmax(0,t->current_thrust/fmax(t->mass,1)),align=fabs(t->autopilot_error);bool aligned=align<=10;if(!aligned&&!g->has_burn_command_started&&t->ut>burnstart+fmax(30,plan->estimated_burn_duration)){guidance_abort(g);GuidanceCommand safe;guidance_command_init(&safe);return result_make(PHASE_ABORT,safe,"Deorbit burn aborted because retrograde alignment missed the burn window.","The vehicle did not achieve the required retrograde alignment in time. Manual control restored; replan from the current orbit.");}if(aligned&&!g->has_burn_command_started){g->burn_command_started_ut=t->ut;g->has_burn_command_started=true;g->burn_active_elapsed=0;g->burn_progress_watch_ut=t->ut;g->burn_progress_watch_delta_v=g->delivered_delta_v;g->has_burn_progress_watch=true;}if(aligned&&g->has_burn_command_started){g->delivered_delta_v+=meas*fmax(0,cos(align*DEG2RAD))*dt;g->burn_active_elapsed+=dt;if(!g->has_burn_progress_watch||g->delivered_delta_v-g->burn_progress_watch_delta_v>=.25){g->burn_progress_watch_ut=t->ut;g->burn_progress_watch_delta_v=g->delivered_delta_v;g->has_burn_progress_watch=true;}}double rem=fmax(0,dvtarget-g->delivered_delta_v);double fraction=burn_fraction(g->burn_active_elapsed,rem,avail,s->deorbit_throttle_ramp_duration),authority=clampd((10-align)/6,0,1),pa=has_peri?clampd((peri_rem-250)/8000,0,1):1;c.target_throttle=rem<.08?0:(aligned?maxthr*fmin(fraction,pa)*authority:0);double expected_accel=fmax(0,t->available_thrust*c.target_throttle/fmax(t->mass,1));bool stalled=aligned&&g->has_burn_command_started&&g->has_burn_progress_watch&&rem>2&&t->ut-g->burn_progress_watch_ut>6&&expected_accel>.02&&meas<fmax(.01,expected_accel*.10);if(stalled){guidance_abort(g);GuidanceCommand safe;guidance_command_init(&safe);return result_make(PHASE_ABORT,safe,"Deorbit burn aborted after sustained loss of thrust/delta-v progress.","The burn stopped making measurable progress despite a meaningful commanded thrust level. Manual control restored; do not continue entry on the stale plan.");}char st[320];snprintf(st,sizeof(st),"Deorbit burn: %.1f / %.1f m/s, throttle %.0f%%.%s%s",g->delivered_delta_v,plan->delta_v,c.target_throttle*100,has_peri?" Periapsis closure active.":"",plan->live_cutoff_capture_qualified?" Cutoff-now trajectory is inside the entry corridor.":"");return result_make(g->phase,c,st,t->available_thrust<1?"No usable thrust is available.":align>10?"Holding throttle until retrograde alignment is stable.":NULL);}}
    if(!g->deorbit_burn_completed){g->phase=PHASE_COAST;GuidanceCommand c;guidance_command_init(&c);c.autopilot_engaged=true;c.use_inertial_direction=true;c.inertial_direction=vnorm(vscale(state->velocity,-1),v3(-1,0,0));c.navball_speed_mode=SPEED_ORBIT;c.control_profile=PROFILE_ORBITAL;return result_make(g->phase,c,"Holding retrograde until the deorbit burn completes.",NULL);}if(!g->atmospheric_interface_crossed){double signed_entry_roll=norm_signed_deg(t->roll),entry_roll=fabs(signed_entry_roll),capture_aoa=entry_low_q_protective_aoa_floor(t->dynamic_pressure,v),entry_pitch_error=fabs(t->angle_of_attack-capture_aoa),entry_heading_error=fabs(norm_signed_deg(t->ground_track_heading-t->heading));bool entry_attitude_ready=entry_roll<=18&&entry_pitch_error<=4&&entry_heading_error<=15&&fabs(t->roll_rate)<=8&&fabs(t->pitch_rate)<=8&&fabs(t->heading_rate)<=8;bool entry_reached=t->mean_altitude<=entry_alt&&t->vertical_speed<0;if(entry_reached&&entry_attitude_ready){g->atmospheric_interface_crossed=true;if(entry_roll>=5)g->s_turn_sign=signed_entry_roll<0?-1:1;}else{g->phase=PHASE_ENTRY_INTERFACE;const char*st=entry_reached?"Atmospheric interface reached. Holding prograde heading, entry AoA and wings-level attitude before MM304 guidance.":"Burn complete. Using RCS/direct control to capture prograde heading, entry AoA and wings-level attitude.";return result_make(g->phase,entry_capture(t,state,v),st,entry_reached&&!entry_attitude_ready?"MM304 guidance is inhibited until entry heading/AoA/roll attitude is stabilized.":NULL);}}
    return terminal_guidance_selected(g,t,state,course,p,aero,cfg,dt,terminal_model);
}
#undef guidance_update
