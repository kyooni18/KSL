       double terminal_outer_glide_slope(const GuidanceMachine*g,const GuidanceSettings*s){
    return g->terminal_glide_mode&&g->terminal_test_glide_slope>0?g->terminal_test_glide_slope:s->final_glide_slope;
}

static double terminal_preflare_altitude(const GuidanceMachine*g,const Telemetry*t,const GuidanceSettings*s){
    double slope=terminal_outer_glide_slope(g,s);
    double nominal_sink=t->horizontal_speed*tan(slope*DEG2RAD);
    double sink=fmax(nominal_sink,fmax(0,-t->vertical_speed));
    if(g->terminal_glide_mode){
        double minimum=fmax(g->terminal_test_preflare_altitude,fmax(450.0,s->flare_altitude*8.0));
        return clampd(fmax(minimum,sink*7.0),minimum,900.0);
    }
    return clampd(sink*7.0,fmax(180.0,s->flare_altitude*3.0),700.0);
}

                
                  
                              
                            
                      
                       
                         
                           
                                 
                                    
                           
                         
                          
                       

static double terminal_final_distance(const GuidanceMachine*g,const GuidanceSettings*s){
    if(g->terminal_glide_mode&&g->terminal_test_final_approach_distance>0)
        return g->terminal_test_final_approach_distance;
    return s->final_approach_distance;
}

/* The outer-final contract is distinct from the low-speed touchdown contract.
   The shuttle reaches the final-alignment station at the configured outer
   speed, then spends the remaining station-to-runway distance decelerating to
   the configured touchdown speed. Keep this schedule monotone and bounded;
   do not turn the 160 m/s station value into a flat final-approach hold. */
static double terminal_final_alignment_speed(const GuidanceSettings*s,
        const VehicleProfile*v){
    double fallback=fmax(v->final_approach_speed,v->minimum_safe_speed);
    double alignment=s&&isfinite(s->final_alignment_speed)&&s->final_alignment_speed>0.0?
        s->final_alignment_speed:fallback;
    return fmax(alignment,fmax(v->touchdown_speed,1.0));
}

static double terminal_final_speed_target(const GuidanceMachine*g,
        const VehicleProfile*v,const GuidanceSettings*s,double distance){
    double final_distance=fmax(1.0,terminal_final_distance(g,s));
    double alignment=terminal_final_alignment_speed(s,v);
    double touchdown=clampd(v->touchdown_speed,1.0,alignment);
    double fraction=clampd(distance/final_distance,0.0,1.0);
    return touchdown+(alignment-touchdown)*fraction;
}

static void terminal_observe_response(GuidanceMachine*g,const Telemetry*t,double dt){
    if(!g->terminal_response_sample_valid){
        g->terminal_response_sample_valid=true;
        g->terminal_previous_aoa=t->angle_of_attack;
        g->terminal_previous_vertical_speed=t->vertical_speed;
        g->terminal_response_sample_ut=t->ut;
        return;
    }
    double sample_dt=t->ut-g->terminal_response_sample_ut;
    if(!(sample_dt>=.04&&sample_dt<=.75)){
        g->terminal_previous_aoa=t->angle_of_attack;
        g->terminal_previous_vertical_speed=t->vertical_speed;
        g->terminal_response_sample_ut=t->ut;
        return;
    }
    double aoa_rate=(t->angle_of_attack-g->terminal_previous_aoa)/sample_dt;
    double sink_accel=(t->vertical_speed-g->terminal_previous_vertical_speed)/sample_dt;
    if(isfinite(aoa_rate)&&aoa_rate>.15){
        double observed=clampd(aoa_rate,.25,4.0);
        g->terminal_positive_aoa_rate_ema=.88*g->terminal_positive_aoa_rate_ema+.12*observed;
        if(isfinite(sink_accel)&&sink_accel>.1){
            double accel=clampd(sink_accel,.1,6.0);
            g->terminal_sink_accel_ema=g->terminal_sink_accel_ema>0?
                .90*g->terminal_sink_accel_ema+.10*accel:accel;
        }
    }
    (void)dt;
    g->terminal_previous_aoa=t->angle_of_attack;
    g->terminal_previous_vertical_speed=t->vertical_speed;
    g->terminal_response_sample_ut=t->ut;
}

static TerminalPreflarePlan terminal_preflare_plan(const GuidanceMachine*g,const Telemetry*t,
        const PlanetModel*p,AerodynamicModel aero,const LandingConfiguration*cfg){
    const VehicleProfile*v=&cfg->vehicle;
    const GuidanceSettings*s=&cfg->guidance;
    TerminalPreflarePlan plan={0};
    plan.reject_reason="uninitialized";
    double slope=terminal_outer_glide_slope(g,s);

    double gravity=planet_surface_gravity(p);
    if(!(gravity>0.0)||!isfinite(t->vertical_speed)||!isfinite(t->true_air_speed)){
        plan.reject_reason="gravity-or-vertical-speed-invalid";
        return plan;
    }

    double sink=fmax(0.0,-t->vertical_speed);
    /* Preflare delivers the vehicle to Inner Final, not directly to touchdown.
       Use the same shallow inner-final entry sink that the controller already
       uses when no stored plan is available; the touchdown sink belongs to the
       later flare phase. */
    double target_sink=clampd(t->horizontal_speed*tan(8.0*DEG2RAD),6.0,14.0);
    RunwayCaptureEnvelope recovery=decision_runway_capture_envelope(
        g,t,cfg->site.runway_heading,p,cfg);
    if(!recovery.valid){plan.reject_reason="capture-envelope-invalid";return plan;}
    /* Preflare is an energy/height trade, not a minimum-height maximum-lift
       maneuver.  Search the certified incidence envelope and use the lowest
       AoA that can capture the inner-final sink while preserving the
       aerodynamic speed floor.  Extra altitude is therefore spent on a gentler
       pull-up instead of being thrown away and then paid for as induced drag. */
    double minimum_speed=v->minimum_safe_speed;
    if(isfinite(t->calibrated_stall_speed)&&t->calibrated_stall_speed>0.0)
        minimum_speed=fmax(minimum_speed,t->calibrated_stall_speed);

    double current_lf=0.0,current_df=0.0;
    aerodynamic_force_factors_mach(t->mach,t->angle_of_attack,v,&current_lf,&current_df);
    if(!(current_df>DBL_EPSILON)){
        plan.reject_reason="aero-force-factors-invalid";
        return plan;
    }

    double measured_lift=live_lift_accel(t,aero,v);
    double measured_drag=live_drag_accel(t,aero,v);
    if(!(measured_lift>0.0)||!(measured_drag>=0.0)){
        plan.reject_reason="measured-aero-invalid";
        return plan;
    }

    double vertical_projection=cos(t->flight_path_angle*DEG2RAD);
    double target_aoa=NAN,modeled_up=NAN,response_time=NAN;
    double required_height=NAN,trigger=NAN;
    double predicted_speed=NAN,best_trigger=INFINITY;
    VerticalRecoveryEnvelope vertical={0};

    /* Keep a maximum-authority fallback solely to publish a physical gate when
       no energy-feasible candidate exists.  Callers use that gate to distinguish
       "recoverable while still above preflare" from a truly exhausted state. */
    double fallback_trigger=NAN,fallback_required_height=NAN,fallback_pull_time=NAN;
    double fallback_target_drag=NAN,fallback_air_path=NAN,fallback_modeled_up=NAN;
    double fallback_response_time=NAN,fallback_predicted_speed=NAN;

    double maximum_aoa=clampd(recovery.maximum_lift_aoa_deg,0.0,v->maximum_angle_of_attack);
    for(double candidate=maximum_aoa;candidate>=0.5;candidate-=0.5){
        double target_df=0.0;
        aerodynamic_force_factors_mach(t->mach,candidate,v,NULL,&target_df);
        if(!(target_df>=0.0)||!isfinite(target_df))continue;

        double projected_lift=terminal_projected_lift_accel_at_aoa(t,aero,v,candidate);
        if(isfinite(recovery.maximum_normal_accel_mps2)&&recovery.maximum_normal_accel_mps2>0.0)
            projected_lift=fmin(projected_lift,recovery.maximum_normal_accel_mps2);
        double candidate_up=fmax(0.0,projected_lift*vertical_projection-gravity);
        if(!(candidate_up>0.0)||!isfinite(candidate_up))continue;

        double candidate_response=decision_final_approach_response_time(g,t,s,candidate);
        if(!isfinite(candidate_response))continue;
        double response_down=fmax(0.0,
            0.5*(recovery.response_down_accel_mps2-candidate_up));
        VerticalRecoveryEnvelope candidate_vertical=decision_vertical_recovery_envelope(
            t->radar_altitude-s->flare_altitude,sink,target_sink,candidate_response,
            response_down,candidate_up);
        if(!candidate_vertical.valid)continue;

        double candidate_required=candidate_vertical.required_height_m;
        double candidate_trigger=s->flare_altitude+candidate_required;
        double candidate_pull=candidate_vertical.recovery_time_s;
        double candidate_drag=measured_drag*(target_df/current_df);
        double candidate_air_path=fmax(0.0,t->true_air_speed*candidate_pull);
        double response_path=fmax(0.0,t->true_air_speed*candidate_response);
        double response_speed=NAN,response_work=NAN,candidate_speed=NAN,candidate_work=NAN;
        if(!terminal_quadratic_drag_energy_step(t->true_air_speed,measured_drag,
                response_path,gravity*candidate_vertical.response_height_m,
                &response_speed,&response_work)||
           !terminal_quadratic_drag_energy_step(response_speed,candidate_drag,
                fmax(0.0,response_speed*(candidate_pull-candidate_response)),
                gravity*candidate_vertical.braking_height_m,&candidate_speed,&candidate_work))
            continue;
        candidate_work+=response_work;

        if(candidate>=maximum_aoa-0.25){
            fallback_trigger=candidate_trigger;
            fallback_required_height=candidate_required;
            fallback_pull_time=candidate_pull;
            fallback_target_drag=candidate_drag;
            fallback_air_path=candidate_air_path;
            fallback_modeled_up=candidate_up;
            fallback_response_time=candidate_response;
            fallback_predicted_speed=candidate_speed;
        }

        if(candidate_vertical.height.margin<0.0||candidate_speed<minimum_speed)continue;

        /* Height and aerodynamic energy are independent boundaries.  Outer Final
           is intentionally a steep energy-dissipation glide; do not force a
           preflare merely because speed has fallen to the later 115 m/s nominal
           reference.  Start only early enough that the certified pull-up still
           exits above the aerodynamic safety floor. */
        double candidate_reference=minimum_speed;
        double net_kinetic_loss_rate=fmax(0.0,
            measured_drag*t->true_air_speed-gravity*sink);
        double energy_trigger=s->flare_altitude;
        if(candidate_speed>candidate_reference&&
           net_kinetic_loss_rate>DBL_EPSILON&&sink>0.5){
            double excess=.5*(candidate_speed*candidate_speed-
                candidate_reference*candidate_reference);
            double delay=excess/net_kinetic_loss_rate;
            energy_trigger=fmax(s->flare_altitude,t->radar_altitude-sink*delay);
        }
        candidate_trigger=fmax(candidate_trigger,energy_trigger);

        /* Account for the energy shed while waiting for the selected trigger so
           the plan's kinetic margin describes the state at recovery initiation,
           not the current sample.  The plan is recomputed every guidance tick. */
        double delay_to_trigger=sink>0.5?
            fmax(0.0,(t->radar_altitude-candidate_trigger)/sink):0.0;
        double adjusted_v2=candidate_speed*candidate_speed-
            2.0*net_kinetic_loss_rate*delay_to_trigger;
        if(!isfinite(adjusted_v2)||adjusted_v2<minimum_speed*minimum_speed)continue;
        double adjusted_speed=sqrt(fmax(0.0,adjusted_v2));

        /* Pick the candidate that permits the latest safe initiation.  This
           balances lift authority against induced drag instead of blindly
           preferring either minimum AoA or maximum lift. */
        if(candidate_trigger>=best_trigger-1e-6)continue;
        best_trigger=candidate_trigger;
        target_aoa=candidate;
        modeled_up=candidate_up;
        response_time=candidate_response;
        vertical=candidate_vertical;
        required_height=candidate_required;
        trigger=candidate_trigger;
        predicted_speed=adjusted_speed;
    }

    if(!isfinite(target_aoa)){
        plan.trigger_altitude=fallback_trigger;
        plan.minimum_speed=minimum_speed;
        if(getenv("KSP_LANDER_FINAL_APPROACH_TEST")) fprintf(stderr,
            "FINAL_TEST_PREFLARE speedGate predictedSpeed=%.6f minimumSpeed=%.6f currentSpeed=%.6f aoa=%.6f maxAoA=%.6f stallMeasured=%d stallFraction=%.6f trigger=%.6f requiredHeight=%.6f pullTime=%.6f targetDrag=%.6f airPath=%.6f modeledUp=%.6f responseTime=%.6f targetAoA=%.6f sink=%.6f targetSink=%.6f\n",
            fallback_predicted_speed,minimum_speed,t->true_air_speed,t->angle_of_attack,
            v->maximum_angle_of_attack,t->stall_fraction_is_measured?1:0,t->stall_fraction,
            fallback_trigger,fallback_required_height,fallback_pull_time,fallback_target_drag,
            fallback_air_path,fallback_modeled_up,fallback_response_time,maximum_aoa,sink,target_sink);
        plan.reject_reason="preflare-speed-aoa-or-stall-gate";
        return plan;
    }

    plan.height=vertical.height;
    double predicted_v2=predicted_speed*predicted_speed;

    double preflare_ground=trigger/fmax(tan(slope*DEG2RAD),1e-3);
    double reference_speed=g->terminal_glide_mode?
        fmax(terminal_final_speed_target(g,v,s,preflare_ground),minimum_speed):
        fmax(v->final_approach_speed,minimum_speed);

    double margin_v2=predicted_v2-minimum_speed*minimum_speed;
    if(!isfinite(trigger)||!isfinite(predicted_v2)||
       t->true_air_speed<minimum_speed||
       t->angle_of_attack>v->maximum_angle_of_attack||
       (t->stall_fraction_is_measured&&t->stall_fraction>=1.0)){
        plan.trigger_altitude=trigger;
        plan.minimum_speed=minimum_speed;
        plan.reject_reason="preflare-speed-aoa-or-stall-gate";
        return plan;
    }

    plan.feasible=true;
    plan.trigger_altitude=trigger;
    plan.target_aoa=target_aoa;
    plan.target_sink=-target_sink;
    plan.minimum_speed=minimum_speed;
    plan.reference_speed=reference_speed;
    plan.predicted_height_loss=required_height;
    plan.predicted_kinetic_margin=.5*margin_v2;
    plan.effective_accel=modeled_up;
    plan.response_time=response_time;
    plan.reject_reason="none";
    return plan;
}

       void terminal_store_preflare_plan(GuidanceMachine*g,const TerminalPreflarePlan*p){
    g->terminal_preflare_plan_valid=p&&p->feasible;
    if(!p||!p->feasible)return;
    g->preflare_trigger_altitude=p->trigger_altitude;
    g->preflare_target_aoa=p->target_aoa;
    g->preflare_target_sink=p->target_sink;
    g->preflare_minimum_speed=p->minimum_speed;
    g->preflare_reference_speed=p->reference_speed;
    g->preflare_predicted_height_loss=p->predicted_height_loss;
    g->preflare_predicted_kinetic_margin=p->predicted_kinetic_margin;
    g->preflare_effective_accel=p->effective_accel;
}

       void terminal_set_stage(GuidanceMachine*g,TerminalVerticalStage stage,double ut){
    if(g->terminal_vertical_stage_valid&&g->terminal_vertical_stage==stage)return;
    /* Terminal vertical progression is one-way; once pull-up has begun it can
       never fall back to the steep outer-glide controller. */
    if(g->terminal_vertical_stage_valid&&stage<g->terminal_vertical_stage)return;
    g->terminal_vertical_stage=stage;
    g->terminal_vertical_stage_valid=true;
    g->terminal_stage_started_ut=ut;
    g->terminal_stage_good_duration=0;
    g->flare_sink_captured=false;
}

       bool terminal_outer_gate(GuidanceMachine*g,const Telemetry*t,double course,
        const PlanetModel*p,AerodynamicModel aero,const LandingConfiguration*cfg,
        TerminalPreflarePlan*out_plan){
    const GuidanceSettings*s=&cfg->guidance;
    double slope=terminal_outer_glide_slope(g,s);
    double distance=fmax(0.0,-t->runway_along_track);
    TerminalPreflarePlan plan=terminal_preflare_plan(g,t,p,aero,cfg);
    if(out_plan)*out_plan=plan;
    if(!plan.feasible||!plan.height.valid||plan.height.margin<0.0)return false;

    RunwayCaptureEnvelope envelope=decision_runway_capture_envelope(g,t,course,p,cfg);
    if(!envelope.valid)return false;

    double tangent=tan(slope*DEG2RAD);
    if(!(tangent>0.0)||!(t->horizontal_speed>0.0))return false;
    double preflare_ground=plan.trigger_altitude/tangent;
    double ground_to_preflare=distance-preflare_ground;
    if(!(ground_to_preflare>0.0))return false;

    double time_to_preflare=ground_to_preflare/t->horizontal_speed;
    /* A state whose current velocity vector already intersects the runway has no
       remaining lateral-capture maneuver to reserve.  The old envelope response
       time is still useful when alignment is incomplete, but charging it here for
       an already aligned post-HAC state can consume the entire energy-trigger lead. */
    bool runway_aligned=terminal_preflare_alignment_valid(g,t,course,cfg);
    double lateral_time=runway_aligned?0.0:fmax(envelope.lateral_capture_time_s,
        envelope.heading_capture_time_s);
    DecisionMargin capture_time=decision_margin(time_to_preflare,lateral_time);

    return t->runway_along_track<0.0&&
        envelope.runway_remaining.margin>=0.0&&
        envelope.vertical_recovery.margin>=0.0&&
        envelope.speed.margin>=0.0&&
        envelope.dynamic_pressure.margin>=0.0&&
        envelope.load.margin>=0.0&&
        capture_time.valid&&capture_time.margin>=0.0&&
        plan.predicted_kinetic_margin>=0.0;
}


/* Every terminal path provider uses the same final delivery envelope. */
       bool terminal_outer_capture_admissible(GuidanceMachine*g,const Telemetry*t,double course,
        const PlanetModel*p,AerodynamicModel aero,const LandingConfiguration*cfg,
        TerminalPreflarePlan*out_plan){
    /*
     * There is no second, wider magic-number envelope.  Spline and HAC delivery
     * use the same physically-derived runway-capture proof.
     */
    return terminal_outer_gate(g,t,course,p,aero,cfg,out_plan);
}


/* Reduce the currently committed terminal path and measured vehicle response to
   TAEMEXEC's shape-independent delivery contract.  This is deliberately derived
   from the same preflare, q-bar and energy quantities that Final will actually
   use; no optimistic candidate-only forecast is allowed to authorize delivery. */
       TaemTerminalContract terminal_delivery_contract(const GuidanceMachine*g,const Telemetry*t,
        double course,const PlanetModel*p,const LandingConfiguration*cfg,
        const TerminalPreflarePlan*plan){
    TaemTerminalContract contract={0};
    if(!g||!t||!p||!cfg||!plan||!plan->feasible)return contract;

    const LandingSite*site=&cfg->site;
    const VehicleProfile*v=&cfg->vehicle;
    const GuidanceSettings*s=&cfg->guidance;
    RunwayCaptureEnvelope envelope=decision_runway_capture_envelope(g,t,course,p,cfg);
    if(!envelope.valid)return contract;

    double slope=terminal_outer_glide_slope(g,s);
    double tangent=tan(slope*DEG2RAD);
    double distance=fmax(0.0,-t->runway_along_track);
    if(!(tangent>0.0)||!(t->horizontal_speed>0.0))return contract;

    double preflare_ground=plan->trigger_altitude/tangent;
    double ground_to_gate=distance-preflare_ground;
    if(!(ground_to_gate>0.0))return contract;

    double response_available=ground_to_gate/t->horizontal_speed;
    if(!isfinite(plan->response_time))return contract;
    /* plan->response_time is the pull-up actuator response after the preflare
       trigger and is already charged by terminal_preflare_plan().  TAEM delivery
       only needs reserve for unfinished runway-vector capture before that trigger. */
    bool runway_aligned=terminal_preflare_alignment_valid(g,t,course,cfg);
    double response_required=runway_aligned?0.0:
        fmax(envelope.lateral_capture_time_s,envelope.heading_capture_time_s);

    double gravity=planet_surface_gravity(p);
    if(!(gravity>0.0))return contract;
    double height_to_gate=fmax(0.0,t->radar_altitude-plan->trigger_altitude);
    double air_path=hypot(ground_to_gate,height_to_gate);

    double minimum_speed=plan->minimum_speed;
    double gate_speed=fmax(plan->reference_speed,minimum_speed);
    double live_loss=t->mass>0.0&&isfinite(t->drag_force)&&t->drag_force>=0.0?
        t->drag_force/t->mass:NAN;
    if(!isfinite(live_loss))return contract;

    /* Drag acceleration scales with V^2; extending the instantaneous value as
       a constant acceleration to the gate can drive predicted V^2 negative and
       reject an otherwise feasible unpowered path.  Use the same closed-form
       quadratic-drag energy step as preflare planning. */
    double predicted_gate_speed=NAN,predicted_drag_work=NAN;
    if(!terminal_quadratic_drag_energy_step(t->true_air_speed,live_loss,air_path,
            gravity*height_to_gate,&predicted_gate_speed,&predicted_drag_work))
        return contract;
    (void)predicted_drag_work;
    double predicted_gate_v2=predicted_gate_speed*predicted_gate_speed;

    /* The preflare plan already models the pull-up response, induced drag and
       height exchange.  Judge terminal energy at the end of that planned
       recovery rather than at its high-speed entry boundary. */
    double planned_exit_v2=minimum_speed*minimum_speed+
        2.0*plan->predicted_kinetic_margin;
    if(!isfinite(planned_exit_v2)||planned_exit_v2<0.0)return contract;
    double low_energy_margin=plan->predicted_kinetic_margin;
    double gate_energy=.5*gate_speed*gate_speed;
    double planned_exit_energy=.5*planned_exit_v2;
    double high_energy_margin=gate_energy-planned_exit_energy;
    /* The trigger is solved analytically to the reference-energy boundary; do
       not reject an exact solution because of sub-millijoule floating-point
       roundoff.  This tolerance is numerical only, not a flight-energy allowance. */
    double energy_roundoff=1e-9*fmax(gate_energy,1.0);
    if(high_energy_margin<0.0&&high_energy_margin>=-energy_roundoff)
        high_energy_margin=0.0;
    if(getenv("KSP_LANDER_FINAL_APPROACH_TEST")&&g->terminal_final_test_mode&&!g->terminal_glide_mode){
        static bool final_energy_diag_printed=false;
        if(!final_energy_diag_printed){
            fprintf(stderr,
                "FINAL_TEST_ENERGY preTriggerSpeed=%.6f plannedExitSpeed=%.6f minimumSpeed=%.6f referenceGateSpeed=%.6f lowMargin=%.6f highMargin=%.6f liveDragAccel=%.6f airPath=%.6f heightToGate=%.6f groundToGate=%.6f\n",
                predicted_gate_v2>=0.0?sqrt(predicted_gate_v2):NAN,sqrt(planned_exit_v2),
                minimum_speed,gate_speed,low_energy_margin,high_energy_margin,live_loss,
                air_path,height_to_gate,ground_to_gate);
            final_energy_diag_printed=true;
        }
    }

    double down_fpa=fmax(0.0,-t->flight_path_angle);
    double fpa_margin=envelope.recoverable_fpa_deg-down_fpa;

    double current_specific=gravity*(t->mean_altitude-site->altitude)+
        .5*t->true_air_speed*t->true_air_speed;

    if(!isfinite(response_available)||!isfinite(response_required)||
       !isfinite(low_energy_margin)||!isfinite(high_energy_margin)||
       !isfinite(current_specific)||!isfinite(fpa_margin))return contract;

    contract.valid=true;
    contract.path_committed=g->terminal_path_committed&&!g->hac_transition_active;
    contract.range_to_go=distance;
    contract.range_margin=(response_available-response_required)*t->horizontal_speed;
    contract.dynamic_pressure=t->dynamic_pressure;
    contract.dynamic_pressure_margin=v->maximum_dynamic_pressure-t->dynamic_pressure;
    contract.speedbrake_required=false;
    contract.speedbrake_available=false;
    contract.speedbrake_dynamic_pressure_margin=v->maximum_dynamic_pressure-t->dynamic_pressure;
    contract.altitude=t->mean_altitude;
    /* Delivery reserves the configured flare boundary, not merely ground
       clearance. A plan can be numerically valid yet physically unreachable. */
    contract.altitude_margin=fmin(envelope.vertical_recovery.margin,
        plan->height.valid?plan->height.margin:-INFINITY);
    contract.flight_path_angle=t->flight_path_angle;
    contract.flight_path_angle_margin=fpa_margin;
    contract.specific_energy=current_specific;
    contract.specific_energy_margin=fmin(low_energy_margin,high_energy_margin);
    contract.response_time_available=response_available;
    contract.response_time_required=response_required;
    contract.attitude_response_qualified=plan->effective_accel>0.0&&
        envelope.vertical_recovery_accel_mps2>0.0;
    return contract;
}

bool terminal_final_conditioning_reachable(const GuidanceMachine*g,
        const Telemetry*t,double course,const PlanetModel*p,AerodynamicModel aero,
        const LandingConfiguration*cfg,EnergyPathEnvelope*energy_out,double*end_speed_out,
        double*slope_out){
    if(energy_out)*energy_out=(EnergyPathEnvelope){0};
    if(end_speed_out)*end_speed_out=NAN;
    if(slope_out)*slope_out=NAN;
    if(!g||!t||!p||!cfg)return false;
    const VehicleProfile*v=&cfg->vehicle;
    const GuidanceSettings*s=&cfg->guidance;
    RunwayCaptureEnvelope capture=decision_runway_capture_envelope(g,t,course,p,cfg);
    /* Conditioning is prospective: it proves that the runway-aligned glide can
       still reach the flare boundary, not that the vehicle can arrest all sink
       immediately at this sample.  Keep the unchanged Final gate responsible
       for instantaneous vertical-recovery authority; here retain every other
       live capture margin plus finite lateral/heading timing, then prove the
       future glide below with projected speed and certified energy. */
    bool capture_valid_except_vertical=capture.runway_time.valid&&
        capture.speed.valid&&capture.dynamic_pressure.valid&&capture.load.valid&&
        capture.runway_remaining.valid&&isfinite(capture.lateral_capture_time_s)&&
        isfinite(capture.heading_capture_time_s);
    bool capture_reachable_except_vertical=capture_valid_except_vertical&&
        capture.runway_time.margin>=0.0&&capture.speed.margin>=0.0&&
        capture.dynamic_pressure.margin>=0.0&&capture.load.margin>=0.0&&
        capture.runway_remaining.margin>=0.0;
    if(!capture_reachable_except_vertical)return false;

    double radar=isfinite(t->radar_altitude)?t->radar_altitude:
        t->mean_altitude-cfg->site.altitude;
    double height=fmax(0.0,radar-s->flare_altitude);
    double ground=fmax(0.0,-t->runway_along_track);
    if(!(height>0.0)||!(ground>1.0))return false;
    double slope=atan2(height,ground)*RAD2DEG;
    /* The final controller is already bounded to a 35 degree recovery dive.
       A state requiring a steeper zero-incidence path merely to reach the flare
       boundary is not a physically admissible final-only starting point. */
    if(!isfinite(slope)||slope>35.0)return false;
    slope=fmax(0.0,slope);

    /* Certify the same force-derived incidence that the conditioning controller
       can actually command.  The old zero-AoA projection admitted a low-drag
       path, then execution used the normal Final drag trim and spent the energy
       reserve before preflare. */
    double conditioning_aoa=terminal_fpa_force_aoa(t,p,aero,v,-slope);
    double end_speed=NAN,end_altitude=NAN;
    double work=terminal_projected_drag_work_state(g,t,p,aero,v,s,conditioning_aoa,ground,
        slope,&end_speed,&end_altitude);
    double gate_altitude=cfg->site.altitude+s->flare_altitude;
    double minimum_speed=fmax(v->minimum_safe_speed,1.0);
    double available=entry_remaining_specific_energy(t->latitude,t->mean_altitude,
        t->true_air_speed,cfg->site.latitude,gate_altitude,minimum_speed,p);
    if(!isfinite(work)||work<0.0||!isfinite(available)||available<0.0||
       !isfinite(end_speed)||end_speed<minimum_speed)return false;
    EnergyPathEnvelope energy=decision_energy_path_envelope(t,p,available,work,ground);
    if(energy_out)*energy_out=energy;
    if(end_speed_out)*end_speed_out=end_speed;
    if(slope_out)*slope_out=slope;
    return energy.valid&&energy.feasible;
}

bool guidance_begin_final_test(GuidanceMachine*g,const Telemetry*t,double course,const PlanetModel*p,AerodynamicModel aero,const LandingConfiguration*cfg,char*message,size_t message_size){
    if(!g||!t||!p||!cfg){
        if(message&&message_size)snprintf(message,message_size,"Final-only test initialization received an invalid argument.");
        return false;
    }
    if(!(t->radar_altitude>0.0)||!(t->true_air_speed>0.0)||t->runway_along_track>=0.0){
        if(message&&message_size)snprintf(message,message_size,"Final-only test requires an airborne pre-threshold runway-referenced state.");
        return false;
    }
    guidance_machine_init(g);
    guidance_set_engaged(g,true);
    g->terminal_glide_mode=false; /* keep production final gates during snapshot admission */
    g->terminal_final_test_mode=true;
    g->terminal_rehearsal_mode=false;
    g->terminal_test_capture_active=false;
    g->terminal_test_glide_slope=0.0;
    g->terminal_test_final_approach_distance=0.0;
    g->has_burn_command_started=true;
    g->deorbit_burn_completed=true;
    g->atmospheric_interface_crossed=true;
    g->terminal_region_entered=true;
    g->terminal_path_kind=TERMINAL_PATH_NONE;
    g->terminal_path_committed=true;
    g->hac_completed=true;
    g->hac_captured=false;
    g->hac_progress_valid=false;
    g->hac_side_selected=false;
    g->hac_radius=0.0;
    g->hac_transition_active=false;
    g->fixed_alignment_hac_latched=false;
    g->fixed_hac_reference_valid=false;
    if(!taem_exec_enter(g,t,&cfg->guidance,p,aero,cfg,true)){
        guidance_machine_init(g);
        if(message&&message_size)snprintf(message,message_size,"Final-only test could not reconstruct TAEM final-intercept ownership.");
        return false;
    }
    if(getenv("KSP_LANDER_FINAL_APPROACH_TEST")){
        RunwayCaptureEnvelope e=decision_runway_capture_envelope(g,t,course,p,cfg);
        fprintf(stderr,
            "FINAL_TEST_CAPTURE valid=%d reachable=%d runwayTime=%.6f/%.6f/%.6f vertical=%.6f/%.6f/%.6f speed=%.6f/%.6f/%.6f q=%.6f/%.6f/%.6f load=%.6f/%.6f/%.6f runwayRemaining=%.6f/%.6f/%.6f groundRange=%.6f timeAvailable=%.6f lateralTime=%.6f headingTime=%.6f responseTime=%.6f crossRate=%.6f lateralAccel=%.6f courseRate=%.6f maxLiftAoA=%.6f maxNormalAccel=%.6f sink=%.6f verticalRecoveryAccel=%.6f responseDownAccel=%.6f recoveryHeight=%.6f recoverableSink=%.6f recoverableFPA=%.6f\n",
            e.valid?1:0,e.reachable?1:0,
            e.runway_time.available,e.runway_time.required,e.runway_time.margin,
            e.vertical_recovery.available,e.vertical_recovery.required,e.vertical_recovery.margin,
            e.speed.available,e.speed.required,e.speed.margin,
            e.dynamic_pressure.available,e.dynamic_pressure.required,e.dynamic_pressure.margin,
            e.load.available,e.load.required,e.load.margin,
            e.runway_remaining.available,e.runway_remaining.required,e.runway_remaining.margin,
            e.ground_range_m,e.time_available_s,e.lateral_capture_time_s,e.heading_capture_time_s,
            e.control_response_time_s,e.cross_rate_mps,e.lateral_accel_mps2,e.course_rate_deg_s,
            e.maximum_lift_aoa_deg,e.maximum_normal_accel_mps2,e.sink_rate_mps,
            e.vertical_recovery_accel_mps2,e.response_down_accel_mps2,e.vertical_recovery_height_m,
            e.recoverable_sink_rate_mps,e.recoverable_fpa_deg);
    }
    TerminalPreflarePlan plan={0};
    bool approach=terminal_outer_gate(g,t,course,p,aero,cfg,&plan);
    TaemTerminalContract contract=terminal_delivery_contract(g,t,course,p,cfg,&plan);
    TaemTerminalEvaluation evaluation=taem_exec_evaluate_terminal_contract(&contract);
    EnergyPathEnvelope conditioning_energy={0};
    double conditioning_end_speed=NAN,conditioning_slope=NAN;
    bool conditioning=terminal_final_conditioning_reachable(g,t,course,p,aero,cfg,
        &conditioning_energy,&conditioning_end_speed,&conditioning_slope);
    bool delivered=approach&&evaluation.valid&&evaluation.feasible;
    if(!delivered&&!conditioning){
        const char*reject=plan.reject_reason?plan.reject_reason:"none";
        const char*block=taem_terminal_block_reason_string(evaluation.block_reason);
        if(getenv("KSP_LANDER_FINAL_APPROACH_TEST")) fprintf(stderr,
            "FINAL_TEST_ADMISSION result=reject approach=%d preflare=%s terminal=%s conditioning=0 rangeMargin=%.6f qMargin=%.6f altitudeMargin=%.6f fpaMargin=%.6f energyMargin=%.6f responseAvailable=%.6f responseRequired=%.6f\n",
            approach?1:0,reject,block,contract.range_margin,contract.dynamic_pressure_margin,
            contract.altitude_margin,contract.flight_path_angle_margin,contract.specific_energy_margin,
            contract.response_time_available,contract.response_time_required);
        if(message&&message_size)snprintf(message,message_size,
            "Final-only snapshot rejected: no physically reachable runway-line conditioning path remains (approach=%d preflare=%s terminal=%s).",
            approach?1:0,reject,block);
        guidance_machine_init(g);
        return false;
    }
    g->terminal_glide_mode=true; /* routing flag only; final guidance clears it while evaluating/controlling */
    if(getenv("KSP_LANDER_FINAL_APPROACH_TEST")){
        if(delivered)fprintf(stderr,
            "FINAL_TEST_ADMISSION result=accept approach=1 preflare=ok terminal=ok conditioning=1 rangeMargin=%.6f qMargin=%.6f altitudeMargin=%.6f fpaMargin=%.6f energyMargin=%.6f responseAvailable=%.6f responseRequired=%.6f\n",
            contract.range_margin,contract.dynamic_pressure_margin,contract.altitude_margin,
            contract.flight_path_angle_margin,contract.specific_energy_margin,
            contract.response_time_available,contract.response_time_required);
        else fprintf(stderr,
            "FINAL_TEST_ADMISSION result=condition approach=%d preflare=%s terminal=%s conditioning=1 slope=%.6f projectedEndSpeed=%.6f energyMargin=%.6f\n",
            approach?1:0,plan.reject_reason?plan.reject_reason:"none",
            taem_terminal_block_reason_string(evaluation.block_reason),conditioning_slope,
            conditioning_end_speed,conditioning_energy.energy.margin);
    }
    g->phase=PHASE_TAEM;
    if(message&&message_size)snprintf(message,message_size,delivered?
        "Final-only post-HAC test armed from runway-aligned recorded state; TAEM final-intercept contract is feasible.":
        "Final-only post-HAC test armed from runway-aligned recorded state; TAEM runway alignment will condition energy until the unchanged final-intercept contract becomes feasible.");
    return true;
}


bool terminal_preflare_alignment_valid(const GuidanceMachine*g,const Telemetry*t,double course,
        const LandingConfiguration*cfg){
    (void)g;
    const LandingSite*site=&cfg->site;
    const VehicleProfile*v=&cfg->vehicle;
    if(!isfinite(t->runway_along_track)||!isfinite(t->runway_cross_track)||
       !isfinite(course)||!isfinite(t->horizontal_speed)||!(t->horizontal_speed>0.0))
        return false;

    /*
     * Preflare alignment uses the runway geometry itself.  At this stage there
     * is no remaining terminal turn: lateral velocity must be converging toward
     * centerline and the projected intercept must lie on the physical runway.
     */
    double error=norm_signed_deg(course-site->runway_heading)*DEG2RAD;
    double cross_rate=t->horizontal_speed*sin(error);
    double along_rate=t->horizontal_speed*cos(error);
    if(!(along_rate>0.0))return false;

    double time_to_threshold=fmax(0.0,-t->runway_along_track)/along_rate;
    double projected_cross=t->runway_cross_track+cross_rate*time_to_threshold;
    double minimum=g&&g->terminal_preflare_plan_valid?
        g->preflare_minimum_speed:v->minimum_safe_speed;

    return t->runway_along_track<site->runway_length&&
        fabs(projected_cross)<=site->runway_width*.5&&
        t->true_air_speed>=minimum;
}


static void terminal_update_gear_latch(GuidanceMachine*g,const Telemetry*t,const GuidanceSettings*s){
    if(g->gear_command_latched)return;
    if(t->gear||t->radar_altitude<=s->gear_deployment_altitude){g->gear_command_latched=true;return;}
    if(!g->terminal_preflare_plan_valid)return;
    double sink=fmax(.5,-t->vertical_speed);
    double seconds=(t->radar_altitude-g->preflare_trigger_altitude)/sink;
    if(seconds<=12.0&&g->preflare_predicted_kinetic_margin>=0)g->gear_command_latched=true;
}

       bool terminal_runway_contact_position(const Telemetry*t,const LandingConfiguration*cfg){
    const LandingSite*site=&cfg->site;
    return t->runway_along_track>=-40.0&&t->runway_along_track<site->runway_length+80.0&&
        fabs(t->runway_cross_track)<fmax(site->runway_width*.75,45.0);
}

static bool terminal_update_ground_latch(GuidanceMachine*g,const Telemetry*t,const LandingConfiguration*cfg,double dt){
    const VehicleProfile*v=&cfg->vehicle;
    if(g->ground_contact_latched)return true;
    bool over_runway=terminal_runway_contact_position(t,cfg);
    bool landed_on_runway=!strcasecmp(t->vessel_situation,"landed")&&over_runway&&t->gear;
    bool flare_stage=g->terminal_vertical_stage_valid&&g->terminal_vertical_stage>=TERMINAL_TOUCHDOWN_FLARE;
    bool near_contact=flare_stage&&t->gear&&over_runway&&
        t->radar_altitude<.6&&t->vertical_speed>-3.0&&fabs(t->vertical_speed)<3.0&&
        t->surface_speed<fmax(190.0,v->final_approach_speed*1.65);
    if(near_contact)g->ground_contact_duration+=fmax(0,dt);
    else g->ground_contact_duration=fmax(0,g->ground_contact_duration-fmax(0,dt)*2.0);
    if(landed_on_runway||g->ground_contact_duration>=.25){
        g->ground_contact_latched=true;g->gear_command_latched=true;
        terminal_set_stage(g,TERMINAL_GROUND,t->ut);
        return true;
    }
    return false;
}
