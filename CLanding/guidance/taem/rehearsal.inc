/* HAC/MM305 rehearsal initialization only.
   Production path selection and control are intentionally owned by other files. */

bool guidance_begin_hac_test(GuidanceMachine*g,const Telemetry*t,double course,
        const PlanetModel*p,AerodynamicModel aero,const LandingConfiguration*cfg,
        char*message,size_t message_size){
    if(!g||!t||!p||!cfg){
        if(message&&message_size)
            snprintf(message,message_size,
                "HAC test initialization received an invalid argument.");
        return false;
    }
    const GuidanceSettings*s=&cfg->guidance;
    const VehicleProfile*v=&cfg->vehicle;
    double altitude_floor=fmax(1000.0,s->flare_altitude*8.0);
    double speed_floor=fmax(v->minimum_safe_speed*1.20,
        v->final_approach_speed*1.25);
    if(t->radar_altitude<=altitude_floor){
        if(message&&message_size)
            snprintf(message,message_size,
                "HAC-only test requires at least %.0f m radar altitude.",
                altitude_floor);
        return false;
    }
    if(t->true_air_speed<speed_floor){
        if(message&&message_size)
            snprintf(message,message_size,
                "HAC-only test requires at least %.0f m/s airspeed for recovery margin.",
                speed_floor);
        fprintf(stderr,
            "HAC_INIT_REJECT reason=speed-floor speed=%.9f minimum=%.9f\n",
            t->true_air_speed,speed_floor);
        return false;
    }
    if(t->dynamic_pressure>v->maximum_dynamic_pressure*1.05){
        if(message&&message_size)
            snprintf(message,message_size,
                "HAC-only test blocked because dynamic pressure is above the vehicle limit.");
        return false;
    }

    guidance_machine_init(g);
    guidance_set_engaged(g,true);
    g->terminal_glide_mode=true;
    g->terminal_rehearsal_mode=true;
    g->terminal_test_capture_active=true;
    g->terminal_fixed_hac_fixture_latched=false;

    g->has_burn_command_started=true;
    g->deorbit_burn_completed=true;
    g->atmospheric_interface_crossed=true;
    g->terminal_region_entered=true;

    /* Start uncommitted. MM305 must acquire a future executable path from the
       measured state; the rehearsal may not manufacture an energy-dump loop. */
    g->hac_side=terminal_default_hac_side(t,&cfg->site,course);
    g->hac_side_selected=false;
    g->hac_radius=s->hac_radius;
    g->hac_remaining=0.0;
    g->hac_previous_angle=0.0;
    g->hac_progress_valid=false;
    g->minimum_turn_radius=INFINITY;

    terminal_glide_initialize(g,v,s);

    g->terminal_test_upstream_staging=hac_variant_b_requested();
    g->terminal_test_upstream_staging_line_active=false;
    g->terminal_test_upstream_staging_y=NAN;
    g->terminal_test_upstream_staging_course=NAN;
    g->terminal_test_upstream_staging_epoch_ut=-INFINITY;
    g->terminal_test_upstream_capture_ready=false;
    g->terminal_test_upstream_energy_rejected=false;
    g->terminal_test_upstream_plan_valid=false;
    g->terminal_test_upstream_provisional_plan_valid=false;
    g->terminal_test_upstream_planned_radius=0.0;
    g->terminal_test_upstream_speed_target_valid=false;
    g->terminal_test_upstream_speed_target=0.0;
    g->terminal_test_upstream_tuple_valid=false;
    g->terminal_test_upstream_p0_e=0.0;
    g->terminal_test_upstream_p0_n=0.0;
    g->terminal_test_upstream_p3_e=0.0;
    g->terminal_test_upstream_p3_n=0.0;
    g->terminal_test_upstream_center_e=0.0;
    g->terminal_test_upstream_center_n=0.0;
    g->terminal_test_upstream_cone_start_angle=0.0;
    g->terminal_test_upstream_cone_end_angle=0.0;
    g->terminal_test_upstream_cone_arc_length=0.0;
    g->terminal_test_upstream_lead_curve=false;
    g->terminal_test_upstream_lead_start_e=0.0;
    g->terminal_test_upstream_lead_start_n=0.0;
    g->terminal_test_upstream_lead_p1_e=0.0;
    g->terminal_test_upstream_lead_p1_n=0.0;
    g->terminal_test_upstream_lead_p2_e=0.0;
    g->terminal_test_upstream_lead_p2_n=0.0;
    g->terminal_test_upstream_lead_start_course=0.0;
    g->terminal_test_upstream_lead_end_course=0.0;
    g->terminal_test_upstream_lead_length=0.0;
    g->terminal_test_upstream_vertical_profile_valid=false;
    g->terminal_test_upstream_profile_start_altitude=0.0;
    g->terminal_test_upstream_profile_final_altitude=0.0;
    g->terminal_test_upstream_profile_slope_deg=0.0;
    g->terminal_test_upstream_profile_initial_slope_deg=0.0;
    g->terminal_test_upstream_profile_settled_slope_deg=0.0;
    g->terminal_test_upstream_profile_final_slope_deg=0.0;
    g->terminal_test_upstream_profile_transition_distance=0.0;
    g->terminal_test_upstream_profile_exit_transition_distance=0.0;
    g->terminal_test_upstream_avoidable_aoa_energy_loss=0.0;
    g->terminal_test_upstream_last_avoidable_aoa_loss=0.0;
    g->terminal_test_upstream_last_plan_ut=-INFINITY;
    g->terminal_test_upstream_last_log_ut=-INFINITY;
    g->terminal_test_upstream_future_ref_valid=false;
    g->terminal_test_upstream_future_ref_ut=-INFINITY;
    g->terminal_test_upstream_future_ref_stage_time=0.0;
    g->terminal_test_upstream_future_ref_stage_distance=0.0;
    g->terminal_test_upstream_future_ref_radius=0.0;
    g->terminal_test_upstream_future_ref_p0_e=0.0;
    g->terminal_test_upstream_future_ref_p0_n=0.0;
    g->terminal_test_upstream_future_ref_speed=0.0;
    g->terminal_test_upstream_future_ref_altitude=0.0;
    g->terminal_test_upstream_future_ref_margin=NAN;

    if(!taem_exec_enter(g,t,s,p,aero,cfg,true)){
        if(message&&message_size)
            snprintf(message,message_size,
                "HAC-only landing test could not reconstruct MM305 executive ownership.");
        return false;
    }

    g->phase=PHASE_TAEM;
    if(message&&message_size)
        snprintf(message,message_size,
            "HAC-only landing test armed: preflare gate %.0f m / %.0f m/s "
            "(minimum %.0f m/s); acquiring an executable terminal path from live authority and energy.",
            g->terminal_test_preflare_altitude,
            g->terminal_test_preflare_target_speed,
            g->terminal_test_preflare_min_speed);
    return true;
}
