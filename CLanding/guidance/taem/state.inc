       double terminal_hac_radius_live(const Telemetry*t,const PlanetModel*p,AerodynamicModel aero,const VehicleProfile*v,const GuidanceSettings*s){
    double usable_bank=fmin(55.0,dynamic_bank_limit(t,v));
    double min_radius=live_turn_radius(t,aero,v,usable_bank);
    const double minimum_hac_radius=3000.0;
    double max_terminal_radius=fmax(minimum_hac_radius,
        fmin(p->radius*.20,fmax(s->hac_radius*8.0,s->taem_interface_range*2.5)));
    double comfort_radius=t->true_air_speed/(hac_course_rate_cap(t)*DEG2RAD);
    if(!isfinite(min_radius)||min_radius*1.20>max_terminal_radius||comfort_radius>max_terminal_radius)return INFINITY;
    /* The configured value is a nominal planning scale, not an immutable
       terminal circle.  Select the smallest radius that the measured lift,
       bank and response envelope can sustain, while preserving a hard 3 km
       lower bound for the analytic HAC.  A smaller circle is rejected later
       by the same authority gate rather than being commanded optimistically. */
    return clampd(fmax(comfort_radius,fmax(minimum_hac_radius,min_radius*1.30)),
        minimum_hac_radius,max_terminal_radius);
}
       void terminal_glide_initialize(GuidanceMachine*g,const VehicleProfile*v,const GuidanceSettings*s){
    g->terminal_glide_mode=true;
    g->terminal_path_kind=TERMINAL_PATH_NONE;
    g->hac_transition_lead_curve=false;
    g->hac_transition_handoff_lateral_valid=false;
    g->hac_transition_handoff_lateral_acceleration=0.0;
    g->hac_transition_handoff_blend=0.0;
    g->hac_transition_handoff_aoa_valid=false;
    g->hac_transition_handoff_aoa=0.0;
    g->fixed_hac_lead_speed_target_valid=false;
    g->fixed_hac_lead_target_speed=0.0;
    g->fixed_hac_reference_valid=false;
    g->fixed_hac_reference_start_altitude=0.0;
    g->fixed_hac_reference_final_altitude=0.0;
    g->fixed_hac_reference_slope_deg=0.0;
    g->fixed_hac_reference_initial_slope_deg=0.0;
    g->fixed_hac_reference_settled_slope_deg=0.0;
    g->fixed_hac_reference_final_slope_deg=0.0;
    g->fixed_hac_reference_transition_distance=0.0;
    g->fixed_hac_reference_exit_transition_distance=0.0;
    g->hac_plan_degraded=false;
    g->hac_plan_geometry_degraded=false;
    g->hac_plan_energy_degraded=false;
    g->hac_plan_violation_score=0;
    g->hac_commit_blend=0;
    g->terminal_energy_sample_valid=false;
    g->terminal_previous_specific_energy=0;
    g->terminal_previous_speed=0;
    g->terminal_energy_sample_ut=0;
    g->terminal_energy_loss_accel_ema=0;
    g->terminal_speed_loss_accel_ema=0;
    g->terminal_test_preflare_altitude=clampd(fmax(500.0,s->flare_altitude*8.0),450.0,750.0);
    g->terminal_test_preflare_target_speed=fmax(v->final_approach_speed*1.12,v->minimum_safe_speed*1.40);
    g->terminal_test_preflare_min_speed=fmax(v->final_approach_speed*1.03,v->minimum_safe_speed*1.25);
    g->terminal_test_glide_slope=22.0;
    /* Diagnostic-only contract probe.  Production keeps the existing HAC
       test slope unless the caller explicitly asks for a bounded ShuttleSim
       comparison; this must never silently alter normal missions. */
    const char*test_slope_env=getenv("KSP_LANDER_HAC_TEST_GLIDE_SLOPE_DEG");
    const char*diag_env=getenv("KSP_LANDER_HAC_DIAGNOSTICS");
    if(diag_env&&strcmp(diag_env,"1")==0&&test_slope_env){
        char*end=NULL;
        double requested=strtod(test_slope_env,&end);
        if(end!=test_slope_env&&*end=='\0'&&isfinite(requested))
            g->terminal_test_glide_slope=clampd(requested,5.0,35.0);
    }
    /* MM305 ShuttleSim fixtures use the explicit runway-alignment station;
       production/live configuration keeps its configured station. */
    g->terminal_test_final_approach_distance=hac_mm305_dynamic_fixture_requested()?
        3500.0:s->final_approach_distance;
}

       void terminal_energy_observe(GuidanceMachine*g,const Telemetry*t,const PlanetModel*p){
    if(!g||!t||!p||!g->terminal_glide_mode)return;
    double specific=rotating_specific_energy(t->latitude,t->mean_altitude,t->true_air_speed,p);
    if(!isfinite(specific)||!isfinite(t->ut)||!isfinite(t->true_air_speed))return;
    if(g->terminal_energy_sample_valid){
        double dt=t->ut-g->terminal_energy_sample_ut;
        if(dt>=.08&&dt<=2.5){
            double mean_speed=fmax(30.0,.5*(g->terminal_previous_speed+t->true_air_speed));
            double energy_equiv=(g->terminal_previous_specific_energy-specific)/dt/mean_speed;
            double speed_loss=(g->terminal_previous_speed-t->true_air_speed)/dt;
            if(isfinite(energy_equiv)&&energy_equiv>=0&&energy_equiv<60){
                double a=clampd(dt/(2.2+dt),.03,.45);
                g->terminal_energy_loss_accel_ema=g->terminal_energy_loss_accel_ema>0?
                    g->terminal_energy_loss_accel_ema+(energy_equiv-g->terminal_energy_loss_accel_ema)*a:energy_equiv;
            }
            if(isfinite(speed_loss)&&speed_loss>=0&&speed_loss<60){
                double a=clampd(dt/(1.8+dt),.03,.50);
                g->terminal_speed_loss_accel_ema=g->terminal_speed_loss_accel_ema>0?
                    g->terminal_speed_loss_accel_ema+(speed_loss-g->terminal_speed_loss_accel_ema)*a:speed_loss;
            }
        }
    }
    g->terminal_previous_specific_energy=specific;
    g->terminal_previous_speed=t->true_air_speed;
    g->terminal_energy_sample_ut=t->ut;
    g->terminal_energy_sample_valid=true;
}

       double terminal_expected_energy_loss_accel(const GuidanceMachine*g,const Telemetry*t,
        AerodynamicModel aero,const VehicleProfile*v){
    /*
     * Specific mechanical energy lost per metre of air path is D/m. Prefer the
     * larger of the instantaneous measured/modelled drag acceleration and the
     * observed mechanical-energy-loss EMA when both exist. No confidence
     * percentage or safety multiplier is translated into extra drag here;
     * uncertainty is carried separately by EnergyPathEnvelope.
     */
    double live=live_drag_accel(t,aero,v);
    if(!isfinite(live)||live<0.0)return NAN;
    double incidence=hypot(t->angle_of_attack,t->sideslip),df=1.0;
    aerodynamic_force_factors_mach(t->mach,incidence,v,NULL,&df);
    double modeled=fmax(0.0,t->dynamic_pressure)/
        fmax(20.0,aero.ballistic_coefficient)*fmax(0.0,df);
    if(!isfinite(modeled)||modeled<0.0)return NAN;
    /* live_drag_accel intentionally prefers a measured force when available,
       but the admission contract above promises the larger measured/modelled
       instantaneous loss.  Keep that promise here so a stale or low measured
       force cannot make the lead reachability ledger disagree with the energy
       admission ledger on the same telemetry frame. */
    live=fmax(live,modeled);
    if(g&&isfinite(g->terminal_energy_loss_accel_ema)&&
       g->terminal_energy_loss_accel_ema>=0.0)
        return fmax(live,g->terminal_energy_loss_accel_ema);
    return live;
}

/* Diagnostic-only live closure for the committed fixed-HAC lead.  This is
 * deliberately kept outside all admission and command decisions: it measures
 * what the simulator actually flew so the forecast ledger can be audited. */
       void hac_energy_audit_observe(GuidanceMachine*g,const Telemetry*t,
        const PlanetModel*p,AerodynamicModel aero,const VehicleProfile*v,
        double dt){
    if(!g||!t||!p||!v||!g->hac_energy_audit_active||
       g->hac_energy_audit_logged)return;
    double step=dt;
    if(isfinite(g->hac_energy_audit_last_ut)&&isfinite(t->ut))
        step=t->ut-g->hac_energy_audit_last_ut;
    if(!isfinite(step)||step<0.0||step>5.0)step=0.0;
    double drag=terminal_expected_energy_loss_accel(g,t,aero,v);
    if(isfinite(drag)&&drag>=0.0&&isfinite(t->true_air_speed))
        g->hac_energy_audit_actual_drag_work+=drag*t->true_air_speed*step;
    if(isfinite(t->true_air_speed))
        g->hac_energy_audit_actual_distance+=t->true_air_speed*step;
    g->hac_energy_audit_last_ut=t->ut;
}

       void hac_energy_audit_close(GuidanceMachine*g,const Telemetry*t,
        const PlanetModel*p){
    if(!g||!t||!p||!g->hac_energy_audit_active||
       g->hac_energy_audit_logged)return;
    double end_energy=rotating_specific_energy(t->latitude,t->mean_altitude,
        t->true_air_speed,p);
    double realized_loss=isfinite(g->hac_energy_audit_start_specific_energy)&&
        isfinite(end_energy)?g->hac_energy_audit_start_specific_energy-end_energy:NAN;
    double mechanical_residual=isfinite(realized_loss)?
        realized_loss-g->hac_energy_audit_actual_drag_work:NAN;
    double total_path=g->hac_energy_audit_forecast_lead_length+
        fmax(0.0,g->hac_transition_cone_arc_length);
    double forecast_work_per_m=g->hac_energy_audit_forecast_loss;
    double forecast_work=isfinite(forecast_work_per_m)&&isfinite(total_path)?
        forecast_work_per_m*total_path:NAN;
    double forecast_residual=isfinite(forecast_work)&&isfinite(realized_loss)?
        forecast_work-realized_loss:NAN;
    const char*diag=getenv("KSP_LANDER_HAC_DIAGNOSTICS");
    if(diag&&strcmp(diag,"1")==0)
        fprintf(stderr,"HAC_ENERGY_LIVE_CLOSE startUt=%.6f endUt=%.6f "
            "leadForecast=%.6f leadActual=%.6f startE=%.6f endE=%.6f "
            "realizedLoss=%.6f actualDragWork=%.6f mechanicalResidual=%.6f "
            "forecastLossPerM=%.9f forecastWork=%.6f forecastResidual=%.6f "
            "forecastAvailable=%.6f forecastRequired=%.6f forecastMargin=%.6f "
            "p0SpeedForecast=%.6f p0AltForecast=%.6f p0FpaForecast=%.6f "
            "p0AoaForecast=%.6f actualSpeed=%.6f actualAlt=%.6f actualFpa=%.6f actualAoa=%.6f\n",
            g->hac_energy_audit_start_ut,t->ut,
            g->hac_energy_audit_forecast_lead_length,
            g->hac_energy_audit_actual_distance,
            g->hac_energy_audit_start_specific_energy,end_energy,
            realized_loss,g->hac_energy_audit_actual_drag_work,
            mechanical_residual,forecast_work_per_m,forecast_work,
            forecast_residual,g->hac_energy_audit_forecast_available,
            g->hac_energy_audit_forecast_required,
            g->hac_energy_audit_forecast_margin,
            g->hac_energy_audit_forecast_p0_speed,
            g->hac_energy_audit_forecast_p0_altitude,
            g->hac_energy_audit_forecast_p0_fpa,
            g->hac_energy_audit_forecast_p0_aoa,t->true_air_speed,
            t->mean_altitude,t->flight_path_angle,t->angle_of_attack);
    g->hac_energy_audit_logged=true;
    g->hac_energy_audit_active=false;
}

/* A dimensionless polar correction belongs to the measured source state.
   Carry this coefficient, not a live absolute drag acceleration, to p0. */
       double terminal_drag_projection_anchor(const GuidanceMachine*g,
        const Telemetry*t,const PlanetModel*p,AerodynamicModel aero,
        const VehicleProfile*v){
    if(!g||!t||!p||!v||!isfinite(aero.ballistic_coefficient)||
       !(aero.ballistic_coefficient>0.0))return NAN;
    double df=1.0;
    aerodynamic_force_factors_mach(t->mach,hypot(t->angle_of_attack,t->sideslip),v,NULL,&df);
    double density=planet_atmospheric_density(p,t->mean_altitude);
    double modeled=.5*density*t->true_air_speed*t->true_air_speed/
        aero.ballistic_coefficient*df;
    double observed=terminal_expected_energy_loss_accel(g,t,aero,v);
    if(!isfinite(modeled)||modeled<0.0||!isfinite(observed)||observed<0.0)return NAN;
    double anchor=modeled>DBL_MIN?observed/modeled:1.0;
    const char*diag=getenv("KSP_LANDER_HAC_DIAGNOSTICS");
    static int anchor_diag_count=0;
    if(diag&&strcmp(diag,"1")==0&&anchor_diag_count++<12)
        fprintf(stderr,"HAC_DRAG_ANCHOR speed=%.6f aoa=%.6f mach=%.6f density=%.9f q=%.6f polarDf=%.9f dragForce=%.6f liftForce=%.6f mass=%.6f forceVectors=%d physicsValid=%d modeled=%.9f observed=%.9f anchor=%.9f ema=%.9f\n",
            t->true_air_speed,t->angle_of_attack,t->mach,density,
            t->dynamic_pressure,df,t->drag_force,t->lift_force,t->mass,
            t->has_force_vectors?1:0,t->physics_sample_valid?1:0,
            modeled,observed,anchor,
            g->terminal_energy_loss_accel_ema);
    return anchor;
}

static double terminal_expected_speed_loss_accel(const GuidanceMachine*g,const Telemetry*t,
        const PlanetModel*p,AerodynamicModel aero,const VehicleProfile*v){
    (void)g;
    /* Aerodynamic energy loss is not speed loss. During the recorded F14
       -32 degree glide, D/m was about 5.2 m/s2 while TAS stayed near 200 m/s:
       descending potential energy paid the drag work. The former 0.85*D/m
       floor predicted 133 m/s twenty seconds later; live KSP retained 198.
       Keep signed acceleration rather than inventing deceleration in a dive. */
    double radius=fmax(p->radius+t->mean_altitude,1.0);
    double latitude=t->latitude*DEG2RAD;
    double gravity=p->gravitational_parameter/(radius*radius)-
        p->rotational_speed*p->rotational_speed*radius*cos(latitude)*cos(latitude);
    return clampd(live_drag_accel(t,aero,v)+gravity*sin(t->flight_path_angle*DEG2RAD),-40.0,40.0);
}

/* MM305 cannot use the Entry altitude-crossing range proxy once it owns the
   vehicle at or below the TAEM shell.  Define high terminal energy against a
   compact, physically flyable path to the preflare gate instead.  The returned
   distance is only an upstream S-turn planning signal; it never changes HAC
   geometry or relaxes the terminal energy/authority certificates. */
static bool terminal_mm305_energy_excess_range(const GuidanceMachine*g,
        const Telemetry*t,const PlanetModel*p,AerodynamicModel aero,
        const LandingConfiguration*cfg,double*excess_range_out){
    if(excess_range_out)*excess_range_out=NAN;
    if(!g||!t||!p||!cfg||!excess_range_out||
       !isfinite(t->latitude)||!isfinite(t->mean_altitude)||
       !isfinite(t->true_air_speed)||!(t->true_air_speed>0.0)||
       !isfinite(t->runway_along_track)||!isfinite(t->runway_cross_track))
        return false;

    const GuidanceSettings*s=&cfg->guidance;
    const VehicleProfile*v=&cfg->vehicle;
    double gate_height=g->terminal_test_preflare_altitude>0.0?
        g->terminal_test_preflare_altitude:
        clampd(fmax(500.0,s->flare_altitude*8.0),450.0,750.0);
    double gate_altitude=cfg->site.altitude+gate_height;
    double gate_speed=fmax(
        g->terminal_test_preflare_target_speed>0.0?
            g->terminal_test_preflare_target_speed:
            fmax(v->final_approach_speed*1.12,v->minimum_safe_speed*1.40),
        g->terminal_test_preflare_min_speed>0.0?
            g->terminal_test_preflare_min_speed:
            fmax(v->final_approach_speed*1.03,v->minimum_safe_speed*1.25));

    double final_distance=g->terminal_test_final_approach_distance>0.0?
        g->terminal_test_final_approach_distance:s->final_approach_distance;
    double final_glide=g->terminal_test_glide_slope>0.0?
        g->terminal_test_glide_slope:fmax(5.0,s->final_glide_slope);
    double final_tangent=tan(clampd(final_glide,5.0,35.0)*DEG2RAD);
    if(!(final_distance>0.0)||!(final_tangent>0.0)||!isfinite(gate_speed))
        return false;

    double to_alignment=hypot(t->runway_along_track+final_distance,
        t->runway_cross_track);
    double gate_ground=gate_height/final_tangent;
    double final_to_gate=fmax(0.0,final_distance-gate_ground);
    double direct_ground=to_alignment+final_to_gate;
    double height_to_gate=fmax(0.0,t->mean_altitude-gate_altitude);

    double minimum_slope=0.0,maximum_slope=0.0;
    terminal_path_slope_bounds(t,v,s,&minimum_slope,&maximum_slope);
    if(!(minimum_slope>0.0)||!(maximum_slope>=minimum_slope))return false;
    double maximum_tangent=tan(maximum_slope*DEG2RAD);
    if(!(maximum_tangent>0.0))return false;

    /* The shortest usable terminal route must satisfy both the endpoint
       geometry and the steep-side vertical corridor.  Any additional distance
       needed to dissipate energy belongs upstream in the S-turn phase. */
    double compact_ground=fmax(direct_ground,
        height_to_gate/maximum_tangent);
    if(!(compact_ground>DBL_MIN)||!isfinite(compact_ground))return false;
    double compact_slope=atan2(height_to_gate,compact_ground)*RAD2DEG;
    compact_slope=clampd(compact_slope,minimum_slope,maximum_slope);

    double available=entry_remaining_specific_energy(
        t->latitude,t->mean_altitude,t->true_air_speed,
        cfg->site.latitude,gate_altitude,gate_speed,p);
    if(!isfinite(available)||available<0.0)return false;

    /* Price the compact route at the incidence the terminal path is allowed to
       rely on.  Higher-alpha energy disposal belongs to the propagated S-turn
       control search, not to the frozen HAC/spline certificate. */
    double terminal_aoa=terminal_hac_energy_aoa(v);
    double compact_work=terminal_projected_drag_work(g,t,p,aero,v,s,
        terminal_aoa,compact_ground,compact_slope);
    if(!isfinite(compact_work)||compact_work<0.0)return false;
    EnergyPathEnvelope compact_energy=decision_energy_path_envelope(
        t,p,available,compact_work,compact_ground);
    if(!compact_energy.valid)return false;

    /* For a high-energy decision the adverse uncertainty direction is *less*
       drag than nominal.  Subtract the same certified uncertainty that the
       low-energy gate adds to required work. */
    double credible_work=fmax(0.0,
        compact_work-compact_energy.uncertainty_specific_energy);
    double excess_specific=fmax(0.0,available-credible_work);

    double current_incidence=fmax(.25,hypot(t->angle_of_attack,t->sideslip));
    double current_df=1.0,staging_df=1.0;
    aerodynamic_force_factors_mach(t->mach,current_incidence,v,NULL,&current_df);
    double staging_aoa=clampd(v->entry_angle_of_attack,terminal_aoa,
        v->maximum_angle_of_attack);
    aerodynamic_force_factors_mach(t->mach,staging_aoa,v,NULL,&staging_df);
    double live_drag=live_drag_accel(t,aero,v);
    double staging_drag=live_drag*fmax(.02,staging_df)/fmax(.02,current_df);
    if(!isfinite(staging_drag)||!(staging_drag>DBL_MIN))return false;

    double energy_distance=excess_specific/staging_drag;
    double vertical_extra=fmax(0.0,compact_ground-direct_ground);
    *excess_range_out=excess_specific>0.0?
        fmax(energy_distance,vertical_extra):0.0;
    return isfinite(*excess_range_out)&&*excess_range_out>=0.0;
}

/* MM305 owns terminal energy management as well as terminal-path
   acquisition.  A checkpoint/rehearsal may bypass MM304, but a high-energy
   state must first execute the physically propagated S-turn control search;
   otherwise the path provider is forced to manufacture an enormous HAC solely
   to absorb energy.  The executive still enters S-turn only when the live
   energy signal is valid, positive, and no nominal terminal path is feasible. */
static TaemExecProfile taem_exec_profile_production(const GuidanceSettings*s,
        bool terminal_hac_rehearsal){
    (void)s;
    (void)terminal_hac_rehearsal;
    TaemExecProfile profile={0};
    profile.s_turn_enabled=true;
    return profile;
}

static TaemExecInputs taem_exec_inputs_live_with_contract(const GuidanceMachine*g,const Telemetry*t,
        const GuidanceSettings*s,const PlanetModel*p,AerodynamicModel aero,
        const LandingConfiguration*cfg,const TaemTerminalContract*terminal_contract,
        bool has_final_approach_override,bool final_approach_override){
    TaemExecInputs inputs={0};
    inputs.mm304_complete=g->entry_exec.entry_complete;

    /* Preserve the Entry range proxy while it is physically meaningful, but
       once MM305 is being updated use a terminal mechanical-energy staging
       signal that remains valid at/below the handoff altitude. */
    bool entry_energy_valid=isfinite(t->energy_excess_range)&&
        isfinite(t->mean_altitude)&&t->mean_altitude>s->taem_interface_altitude;
    double terminal_excess=NAN;
    bool terminal_energy_valid=p&&cfg&&g->terminal_glide_mode&&
        terminal_mm305_energy_excess_range(g,t,p,aero,cfg,&terminal_excess);
    inputs.energy_valid=terminal_energy_valid||entry_energy_valid;
    inputs.energy_excess=terminal_energy_valid?terminal_excess:
        (entry_energy_valid?t->energy_excess_range:0.0);

    double shell_min=0.0,shell_max=0.0;
    entry_taem_handoff_altitude_bounds(s,&shell_min,&shell_max);
    bool candidate_nominal=g->terminal_candidate.valid&&
        !g->terminal_candidate.geometry_degraded&&!g->terminal_candidate.energy_degraded;
    bool entry_high_energy_staging=entry_energy_valid&&
        t->energy_excess_range>0.0&&t->mean_altitude>shell_max;
    bool terminal_high_energy_staging=terminal_energy_valid&&
        terminal_excess>0.0;
    bool high_energy_staging=entry_high_energy_staging||
        terminal_high_energy_staging;

    /* Above the shell the Entry range surplus remains a one-way staging fact.
       At/below the shell, excess energy is defined against the compact terminal
       route itself.  A giant HAC may not waive that surplus by manufacturing
       distance; only the propagated S-turn may spend it before path acquisition. */
    TaemTerminalEvaluation direct_eval={0};
    bool direct_contract_valid=false;
    bool direct_contract_feasible=false;
    if(g->terminal_final_test_mode&&terminal_contract){
        direct_eval=taem_exec_evaluate_terminal_contract(terminal_contract);
        direct_contract_valid=direct_eval.valid;
        direct_contract_feasible=direct_eval.feasible;
    }
    inputs.terminal_feasibility_valid=g->terminal_final_test_mode?
        direct_contract_valid:(g->terminal_candidate.valid||high_energy_staging);
    inputs.nominal_terminal_path_feasible=g->terminal_final_test_mode?
        direct_contract_feasible:(candidate_nominal&&!high_energy_staging);

    /* Generic post-HAC final testing owns only the runway line.  It must never
       need a HAC side/radius/capture record to satisfy TAEM path ownership. */
    inputs.terminal_path_selected=g->terminal_final_test_mode||
        g->terminal_path_committed||g->hac_side_selected;
    bool committed_fixed_hac=g->fixed_alignment_hac_latched&&
        g->terminal_path_committed&&g->hac_transition_heading_cone&&
        g->hac_transition_active;
    inputs.terminal_path_captured=g->terminal_final_test_mode||
        (g->hac_captured&&!g->hac_transition_active)||committed_fixed_hac;
    inputs.final_intercept_ready=g->terminal_final_test_mode||g->hac_completed;
    inputs.final_approach_ready=has_final_approach_override?
        final_approach_override:g->final_approach_captured;
    if(terminal_contract)inputs.terminal_contract=*terminal_contract;
    inputs.off_nominal_recovery_active=g->attitude_recovery;
    inputs.off_nominal_recovery_reason=g->attitude_recovery?
        TAEM_RECOVERY_ATTITUDE:TAEM_RECOVERY_NONE;
    return inputs;
}


       bool taem_exec_enter(GuidanceMachine*g,const Telemetry*t,const GuidanceSettings*s,
        const PlanetModel*p,AerodynamicModel aero,const LandingConfiguration*cfg,
        bool checkpoint_resume){
    if(g->taem_exec.initialized&&g->taem_exec.ownership_latched)return true;
    TaemExecObservation observation={
        .ut=t->ut,.relative_velocity=t->true_air_speed,.checkpoint_restart=checkpoint_resume
    };
    /* Ownership classification must use the same live MM305 mechanical-energy
       signal as subsequent updates.  Passing a null planet/config here made a
       20 km, 500+ m/s restart enter Path Acquisition for one critical frame and
       allowed an enormous fallback HAC to freeze before S-turn staging existed. */
    TaemExecInputs inputs=taem_exec_inputs_live_with_contract(g,t,s,p,aero,cfg,
        NULL,false,false);
    inputs.resume_mm305=checkpoint_resume;
    if(checkpoint_resume)inputs.mm304_complete=false;
    TaemExecProfile profile=taem_exec_profile_production(s,
        g->terminal_rehearsal_mode);
    return taem_exec_initialize(&g->taem_exec,&observation,&inputs,&profile);
}

       bool taem_exec_sync_with_contract(GuidanceMachine*g,const Telemetry*t,const GuidanceSettings*s,
        const PlanetModel*p,AerodynamicModel aero,const LandingConfiguration*cfg,
        const TaemTerminalContract*terminal_contract,bool has_final_approach_override,
        bool final_approach_override){
    if(!g->taem_exec.initialized||!g->taem_exec.ownership_latched)return false;
    TaemExecObservation observation={.ut=t->ut,.relative_velocity=t->true_air_speed,.checkpoint_restart=false};
    TaemExecInputs inputs=taem_exec_inputs_live_with_contract(g,t,s,p,aero,cfg,
        terminal_contract,has_final_approach_override,final_approach_override);
    TaemExecProfile profile=taem_exec_profile_production(s,
        g->terminal_rehearsal_mode);
    bool updated=taem_exec_update(&g->taem_exec,&observation,&inputs,&profile);
    if(g->taem_exec.phase!=TAEM_PHASE_S_TURN)g->taem_s_turn_plan.valid=false;
    return updated;
}

       void taem_exec_sync(GuidanceMachine*g,const Telemetry*t,const GuidanceSettings*s,
        const PlanetModel*p,AerodynamicModel aero,const LandingConfiguration*cfg){
    (void)taem_exec_sync_with_contract(g,t,s,p,aero,cfg,NULL,false,false);
}


                
               
                                                                              
                                                                      
                                                     
                      

