       GuidanceResult terminal_approach_sequence(GuidanceMachine*g,const Telemetry*t,double course,
        const PlanetModel*p,AerodynamicModel aero,const LandingConfiguration*cfg,const Trajectory*ref,double dt){
    const GuidanceSettings*s=&cfg->guidance;const VehicleProfile*v=&cfg->vehicle;const LandingSite*site=&cfg->site;
    if(!strcasecmp(t->vessel_situation,"landed")&&!terminal_runway_contact_position(t,cfg))
        return terminal_abort(g,"Touchdown occurred outside the runway 09 contact envelope.");
    if(!strcasecmp(t->vessel_situation,"landed")&&!t->gear)
        return terminal_abort(g,"Touchdown occurred without confirmed landing-gear deployment.");
    if(!g->terminal_vertical_stage_valid)terminal_set_stage(g,TERMINAL_TRAJECTORY_CAPTURE,t->ut);
    terminal_observe_response(g,t,dt);
    if(terminal_update_ground_latch(g,t,cfg,dt))return touchdown(g,t,cfg,ref);
    RunwayCaptureEnvelope live_envelope=
        decision_runway_capture_envelope(g,t,course,p,cfg);
    bool corridor_valid=terminal_airborne_corridor_valid(g,t,course,p,cfg);
    if(corridor_valid)g->final_invalid_duration=fmax(0.0,
        g->final_invalid_duration-fmax(0.0,dt));
    else g->final_invalid_duration+=fmax(0.0,dt);

    /*
     * Invalid-state hysteresis is bounded by physics, not a stage-tuned dwell:
     * allow at most one measured control-response interval, and never longer
     * than the remaining ballistic time-to-ground at the current sink rate.
     */
    double radar=isfinite(t->radar_altitude)&&t->radar_altitude>=0.0?
        t->radar_altitude:fmax(0.0,t->mean_altitude-site->altitude);
    double sink=fmax(0.0,-t->vertical_speed);
    double time_to_ground=sink>DBL_MIN?radar/sink:INFINITY;
    double corridor_grace=live_envelope.valid?
        fmin(live_envelope.control_response_time_s,time_to_ground):0.0;
    if(!(corridor_grace>0.0)||!isfinite(corridor_grace))
        corridor_grace=fmax(0.0,dt);
    if(!corridor_valid&&g->final_invalid_duration>=corridor_grace)
        return terminal_abort(g,"Final approach lost: the physics-derived runway capture envelope remained infeasible for a full available response interval.");

    bool preflare_due=false;
    if(g->terminal_vertical_stage<=TERMINAL_OUTER_FINAL){
        TerminalPreflarePlan live=terminal_preflare_plan(g,t,p,aero,cfg);
        g->terminal_preflare_plan_valid=false;
        /* The speed projection assumes the pull-up starts now at the live
           drag.  Above the pull-up gate, with airspeed still above the stall
           boundary, the outer glide can still rebuild speed before the gate,
           so a speed-only rejection is not yet terminal. */
        bool speed_recoverable=!live.feasible&&live.reject_reason&&
            strcmp(live.reject_reason,"preflare-speed-aoa-or-stall-gate")==0&&
            isfinite(live.trigger_altitude)&&radar>live.trigger_altitude&&
            t->true_air_speed>live.minimum_speed&&
            !(t->stall_fraction_is_measured&&t->stall_fraction>=1.0);
        if((!live.feasible||!live.height.valid)&&!speed_recoverable)
            return terminal_abort(g,"No feasible preflare pull-up remains during final capture.");
        if(live.feasible&&live.height.valid){
        if(live.height.margin<0.0)
            return terminal_abort(g,"Preflare initiation missed: remaining height cannot preserve the pull-up reserve.");
        terminal_store_preflare_plan(g,&live);
        /* Start before a control sample can consume the remaining recovery
           margin. This uses the actual integration interval, not an altitude
           window or a grace timer. Capture cannot skip the outer-final state. */
        double sample_time=fmax(0.0,dt);
        /* Same linear-lift-ramp response model as terminal_preflare_plan. */
        double down=fmax(0.0,0.5*(live_envelope.response_down_accel_mps2-
            live_envelope.vertical_recovery_accel_mps2));
        // decision-literal: physical-law-constant | Constant acceleration height loss over one controller sample.
        double next_height=t->radar_altitude-s->flare_altitude-
            sink*sample_time-0.5*down*sample_time*sample_time;
        VerticalRecoveryEnvelope next=decision_vertical_recovery_envelope(
            next_height,sink+down*sample_time,-live.target_sink,
            live.response_time,down,live.effective_accel);
        if(!next.valid)
            return terminal_abort(g,"Preflare response forecast is numerically invalid.");
        preflare_due=next.height.margin<=0.0;
        if(preflare_due&&(g->terminal_vertical_stage!=TERMINAL_OUTER_FINAL||
           !terminal_preflare_alignment_valid(g,t,course,cfg)))
            return terminal_abort(g,"Preflare recovery boundary reached before final alignment was delivered.");
        }
    }
    terminal_update_gear_latch(g,t,s);

    if(g->terminal_vertical_stage==TERMINAL_TRAJECTORY_CAPTURE){
        bool good=terminal_outer_gate(g,t,course,p,aero,cfg,NULL);
        g->terminal_stage_good_duration=good?
            g->terminal_stage_good_duration+fmax(0.0,dt):0.0;
        double stable_for=live_envelope.valid?
            live_envelope.control_response_time_s:INFINITY;
        if(good&&isfinite(stable_for)&&
           g->terminal_stage_good_duration>=stable_for)
            terminal_set_stage(g,TERMINAL_OUTER_FINAL,t->ut);
        g->phase=PHASE_FINAL;
        return final_guidance(g,t,course,p,cfg,ref,dt);
    }

    if(g->terminal_vertical_stage==TERMINAL_OUTER_FINAL){
        bool aligned=terminal_preflare_alignment_valid(g,t,course,cfg);
        if(g->terminal_preflare_plan_valid&&aligned&&preflare_due){
            terminal_set_stage(g,TERMINAL_PREFLARE,t->ut);
            robust_pid_reset(&g->flare_sink_pid);
            terminal_speedbrake_closed(g);
            g->phase=PHASE_FINAL;
            return preflare_guidance(g,t,course,cfg,ref,dt);
        }
        g->phase=PHASE_FINAL;
        return final_guidance(g,t,course,p,cfg,ref,dt);
    }

    if(g->terminal_vertical_stage==TERMINAL_PREFLARE){
        bool stable=t->gear&&fabs(t->vertical_speed-g->preflare_target_sink)<=2.0&&
            g->terminal_sink_accel_ema>=-.2&&t->true_air_speed>=g->preflare_minimum_speed&&
            t->angle_of_attack<=v->maximum_angle_of_attack-2.0&&t->stall_fraction<.15&&
            fabs(norm_signed_deg(t->roll))<=8.0&&fabs(norm_signed_deg(site->runway_heading-course))<=8.0&&
            fabs(t->runway_cross_track)<=fmax(site->runway_width,70.0);
        g->terminal_stage_good_duration=stable?g->terminal_stage_good_duration+fmax(0,dt):0;
        if(g->terminal_stage_good_duration>=.75){
            terminal_set_stage(g,TERMINAL_INNER_FINAL,t->ut);robust_pid_reset(&g->flare_sink_pid);
            g->phase=PHASE_FINAL;return inner_final_guidance(g,t,course,cfg,ref,dt);
        }
        if(t->radar_altitude<fmax(80.0,s->flare_altitude*1.2)&&t->vertical_speed<-12.0)
            return terminal_abort(g,"Preflare failed to arrest sink before the inner-final height floor.");
        g->phase=PHASE_FINAL;return preflare_guidance(g,t,course,cfg,ref,dt);
    }

    if(g->terminal_vertical_stage==TERMINAL_INNER_FINAL){
        double flare_height=terminal_touchdown_flare_altitude(g,t,p,cfg);
        if(!isfinite(flare_height))
            return terminal_abort(g,"Touchdown flare response envelope is unavailable.");
        double flare_sink=clampd(fmax(4.0,.04*t->horizontal_speed),4.0,7.0);
        double flare_min=fmax(v->touchdown_speed*1.10,v->minimum_safe_speed*.95);
        if(t->calibrated_stall_speed>0)flare_min=fmax(flare_min,t->calibrated_stall_speed*1.15);
        bool aligned=t->runway_along_track>=-fmax(1000.0,t->horizontal_speed*10.0)&&
            t->runway_along_track<site->runway_length&&fabs(t->runway_cross_track)<fmax(site->runway_width,70.0)&&
            fabs(norm_signed_deg(site->runway_heading-course))<=8.0;
        if(t->radar_altitude<=flare_height&&t->vertical_speed>=-(flare_sink+2.0)&&t->true_air_speed>=flare_min&&aligned){
            terminal_set_stage(g,TERMINAL_TOUCHDOWN_FLARE,t->ut);robust_pid_reset(&g->flare_sink_pid);g->phase=PHASE_FLARE;
            return flare_guidance(g,t,course,cfg,ref,dt);
        }
        if(t->radar_altitude<=fmax(12.0,flare_height*.45)&&t->vertical_speed<-(flare_sink+4.0))
            return terminal_abort(g,"Inner-final sink rate is too high for the remaining flare height.");
        g->phase=PHASE_FINAL;return inner_final_guidance(g,t,course,cfg,ref,dt);
    }

    if(g->terminal_vertical_stage==TERMINAL_TOUCHDOWN_FLARE){
        bool sink_good=fabs(t->vertical_speed-s->touchdown_sink_rate)<=2.0&&t->stall_fraction<.2&&
            t->angle_of_attack<=v->maximum_angle_of_attack-1.0;
        g->terminal_stage_good_duration=sink_good?g->terminal_stage_good_duration+fmax(0,dt):0;
        if(g->terminal_stage_good_duration>=.50)g->flare_sink_captured=true;
        g->phase=PHASE_FLARE;return flare_guidance(g,t,course,cfg,ref,dt);
    }

    return touchdown(g,t,cfg,ref);
}

