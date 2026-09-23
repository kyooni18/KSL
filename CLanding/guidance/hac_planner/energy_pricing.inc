/* Price the exact fixed-HAC candidate for admission only.  The analytic
   fixed HAC remains at its configured, frozen radius; the energy ledger uses
   the atmosphere/Mach-dependent drag work over the already-frozen lead, the
   270-degree arc, and the final-to-preflare segment exactly once. A caller
   supplying a predicted p0 state passes zero lead length because that state
   already begins after the lead. Specific mechanical energy already includes
   gravity, so no additional gravity credit belongs in this calculation. */
static bool terminal_fixed_hac_energy_radius_impl(const GuidanceMachine*g,
        const Telemetry*t,const PlanetModel*p,AerodynamicModel aero,
        const VehicleProfile*v,const GuidanceSettings*s,const LandingSite*site,
        const Telemetry*drag_reference,double requested_radius,double lead_length,
        double arc_angle,double*radius_out,
        double*margin_out,double*loss_out,double*available_out,
        double*required_out,FixedHacEnergyAudit*audit,
        double fixed_profile_slope,double fixed_target_aoa){
    if(audit)*audit=(FixedHacEnergyAudit){0};
    if(radius_out)*radius_out=INFINITY;
    if(margin_out)*margin_out=NAN;
    if(loss_out)*loss_out=NAN;
    if(available_out)*available_out=NAN;
    if(required_out)*required_out=NAN;
    if(!g||!t||!p||!v||!s||!site||!drag_reference||!(requested_radius>=3000.0)||!(arc_angle>0.0)||
       !isfinite(lead_length)||lead_length<0.0)
        return false;
    /* Non-rehearsal production paths have their own candidate energy
       envelope.  Keep this fixed-HAC test scoped to the terminal test where
       the preflare gate contract is explicit and measured in this fixture. */
    if(!g->terminal_glide_mode)return true;

    double gate_height=fmax(0.0,g->terminal_test_preflare_altitude);
    double gate_altitude=site->altitude+gate_height;
    double gate_speed=g->terminal_test_preflare_min_speed>0.0?
        g->terminal_test_preflare_min_speed:
        (g->terminal_test_preflare_target_speed>0.0?
            g->terminal_test_preflare_target_speed:
            fmax(v->final_approach_speed,v->minimum_safe_speed));
    double available=entry_remaining_specific_energy(t->latitude,
        t->mean_altitude,t->true_air_speed,site->latitude,gate_altitude,
        gate_speed,p);
    double p0_specific_energy=rotating_specific_energy(t->latitude,
        t->mean_altitude,t->true_air_speed,p);
    double gate_specific_energy=rotating_specific_energy(site->latitude,
        gate_altitude,gate_speed,p);
    double final_distance=g->terminal_test_final_approach_distance>0.0?
        g->terminal_test_final_approach_distance:s->final_approach_distance;
    double slope=g->terminal_test_glide_slope>0.0?
        g->terminal_test_glide_slope:s->taem_glide_slope;
    double gate_ground=gate_height/
        fmax(tan(clampd(slope,1.0,75.0)*DEG2RAD),1e-3);
    double final_to_gate=fmax(0.0,final_distance-gate_ground);
    double requested_arc_length=requested_radius*arc_angle;
    double final_altitude=site->altitude+final_distance*
        tan(clampd(slope,1.0,75.0)*DEG2RAD);
    double vertical_path=lead_length+requested_radius*arc_angle;
    double profile_slope=atan2(fmax(0.0,t->mean_altitude-final_altitude),
        fmax(1.0,vertical_path))*RAD2DEG;
    profile_slope=clampd(profile_slope,1.0,75.0);
    if(isfinite(fixed_profile_slope))
        profile_slope=clampd(fixed_profile_slope,1.0,75.0);

    /* Match the admission drag model to the committed command path.  The HAC
       controller does not fly a fixed 10 deg incidence: it starts from the
       force-derived AoA required to hold the selected FPA and raises incidence
       only when the frozen curvature needs more lateral lift.  Certifying a
       constant 10 deg path priced drag that the executable controller never
       commanded and rejected otherwise flyable candidates. */
    double target_aoa=terminal_hac_energy_aoa(v);
    bool target_aoa_overridden=isfinite(fixed_target_aoa);
    if(target_aoa_overridden)
        target_aoa=clampd(fixed_target_aoa,0.0,v->maximum_angle_of_attack);
    if(isfinite(t->true_air_speed)&&!target_aoa_overridden){
        double mid_altitude=0.5*(t->mean_altitude+final_altitude);
        Telemetry mid_state=*t;
        mid_state.mean_altitude=mid_altitude;
        mid_state.flight_path_angle=-profile_slope;
        mid_state.horizontal_speed=t->true_air_speed*cos(profile_slope*DEG2RAD);
        mid_state.vertical_speed=-t->true_air_speed*sin(profile_slope*DEG2RAD);
        mid_state.surface_speed=t->true_air_speed;
        mid_state.lift_force=0.0;
        mid_state.drag_force=0.0;
        mid_state.atmospheric_density=planet_atmospheric_density(p,mid_altitude);
        double sound=planet_atmospheric_speed_of_sound(p,mid_altitude);
        mid_state.speed_of_sound=sound;
        mid_state.mach=sound>DBL_MIN?t->true_air_speed/sound:t->mach;
        mid_state.dynamic_pressure=.5*mid_state.atmospheric_density*
            t->true_air_speed*t->true_air_speed;
        double vertical_aoa=terminal_fpa_force_aoa(&mid_state,p,aero,v,
            -profile_slope);
        if(isfinite(vertical_aoa))target_aoa=vertical_aoa;
        double bank_limit=fmin(v->maximum_bank_angle,dynamic_bank_limit(&mid_state,v));
        double required_lateral=mid_state.horizontal_speed*mid_state.horizontal_speed/
            requested_radius;
        double required_aoa=terminal_required_aoa_for_lateral_limit(&mid_state,aero,v,
            required_lateral,bank_limit,v->maximum_angle_of_attack);
        /* Reachability, not dissipation, is what this certificate proves.
           The geometric slope above spends *all* remaining altitude over the
           path, and the FPA-hold AoA for a steep dive is low-L/D; the drag
           work of that dive equals the altitude it burns, so modeled work was
           ~= available by construction and the uncertainty term rejected
           every candidate (0 of 12,670 passed in the MM305 campaign, even from
           states with >25 km of best-glide range).  When the geometric slope
           is steeper than the best-L/D glide, price the path at the best-L/D
           incidence and glide slope; the excess becomes positive margin that
           the executed profile dissipates.  A slope shallower than best glide
           is left unchanged and still fails honestly. */
        double best_aoa=NAN,best_ratio=-INFINITY;
        for(double aoa=2.0;aoa<=v->maximum_angle_of_attack-1.0+1e-9;aoa+=.25){
            double lf=0.0,df=1.0;
            aerodynamic_force_factors_mach(mid_state.mach,aoa,v,&lf,&df);
            double ratio=lf/fmax(df,.02);
            if(isfinite(ratio)&&ratio>best_ratio){best_ratio=ratio;best_aoa=aoa;}
        }
        double best_ld=isfinite(aero.lift_to_drag)&&aero.lift_to_drag>0.0&&
            isfinite(best_ratio)?aero.lift_to_drag*best_ratio:NAN;
        if(isfinite(best_ld)&&best_ld>DBL_MIN&&isfinite(best_aoa)){
            double glide_slope=atan(1.0/best_ld)*RAD2DEG;
            if(!isfinite(fixed_profile_slope)&&profile_slope>glide_slope){
                profile_slope=clampd(glide_slope,1.0,75.0);
                target_aoa=best_aoa;
            }
        }
        if(isfinite(required_aoa))
            target_aoa=clampd(fmax(target_aoa,required_aoa),0.0,
                v->maximum_angle_of_attack);
    }
    /* Integrate the altitude/Mach/AoA-dependent drag over the exact candidate
       path.  The old point-loss approximation multiplied one sampled D/m by
       lead+arc+final, which made a p0 candidate pay live-state drag and could
       reject a path by several MJ/kg.  A caller that supplies a predicted p0
       state must pass lead_length=0; a live-origin caller includes the lead
       exactly once. */
    double non_arc_path=lead_length+final_to_gate;
    double path_length=non_arc_path+requested_arc_length;
    double arc_end_speed=NAN,arc_end_altitude=NAN;
    /* Freeze one coefficient correction at the real source state. The old
       p0 and final projections re-anchored an absolute live D/m average at
       their lower speed, artificially multiplying future drag a second time. */
    double drag_anchor=terminal_drag_projection_anchor(g,drag_reference,p,aero,v);
    double arc_work=terminal_projected_drag_work_state_anchored(g,t,p,aero,v,s,target_aoa,
        vertical_path,profile_slope,&arc_end_speed,&arc_end_altitude,drag_anchor);
    double end_speed=arc_end_speed,end_altitude=arc_end_altitude;
    double final_work=0.0;
    if(isfinite(arc_work)&&arc_work>=0.0&&final_to_gate>DBL_MIN&&
       isfinite(arc_end_speed)&&isfinite(arc_end_altitude)){
        Telemetry after_arc=*t;
        after_arc.mean_altitude=arc_end_altitude;
        after_arc.true_air_speed=arc_end_speed;
        after_arc.horizontal_speed=arc_end_speed*cos(slope*DEG2RAD);
        after_arc.vertical_speed=-arc_end_speed*sin(slope*DEG2RAD);
        after_arc.flight_path_angle=-slope;
        after_arc.angle_of_attack=target_aoa;
        after_arc.roll=0.0;
        after_arc.lift_force=0.0;
        after_arc.drag_force=0.0;
        double sound=planet_atmospheric_speed_of_sound(p,arc_end_altitude);
        after_arc.mach=sound>DBL_MIN?arc_end_speed/sound:t->mach;
        after_arc.atmospheric_density=planet_atmospheric_density(p,arc_end_altitude);
        after_arc.dynamic_pressure=.5*after_arc.atmospheric_density*arc_end_speed*arc_end_speed;
        double final_target_aoa=terminal_fpa_force_aoa(&after_arc,p,aero,v,-slope);
        if(!isfinite(final_target_aoa))final_target_aoa=terminal_hac_energy_aoa(v);
        final_target_aoa=clampd(final_target_aoa,0.0,v->maximum_angle_of_attack);
        final_work=terminal_projected_drag_work_state_anchored(g,&after_arc,p,aero,v,s,
            final_target_aoa,final_to_gate,slope,&end_speed,&end_altitude,drag_anchor);
    }
    double work=(isfinite(arc_work)&&arc_work>=0.0&&
                 (final_to_gate<=DBL_MIN||(isfinite(final_work)&&final_work>=0.0)))?
        arc_work+final_work:INFINITY;
    bool work_valid=isfinite(work)&&work>=0.0&&path_length>DBL_MIN;
    double loss=work_valid?work/path_length:INFINITY;
    EnergyPathEnvelope energy={0};
    if(work_valid)
        energy=decision_energy_path_envelope(t,p,available,work,path_length);
    double required=energy.valid?
        energy.modeled_drag_work+energy.uncertainty_specific_energy:INFINITY;
    double margin=isfinite(available)?available-required:-INFINITY;
    if(audit){
        audit->valid=work_valid&&energy.valid;
        audit->drag_anchor=drag_anchor;
        audit->arc_end_altitude=arc_end_altitude;audit->arc_end_speed=arc_end_speed;
        audit->radius=requested_radius;
        audit->lead_length=lead_length;
        audit->arc_length=requested_arc_length;
        audit->final_path_length=final_to_gate;
        audit->total_path_length=path_length;
        audit->profile_slope=profile_slope;
        audit->target_aoa=target_aoa;
        audit->altitude=t->mean_altitude;
        audit->speed=t->true_air_speed;
        audit->fpa=t->flight_path_angle;
        audit->specific_energy=rotating_specific_energy(t->latitude,
            t->mean_altitude,t->true_air_speed,p);
        audit->gate_specific_energy=gate_specific_energy;
        audit->available=available;
        audit->available_expected=isfinite(p0_specific_energy)&&
            isfinite(gate_specific_energy)?p0_specific_energy-gate_specific_energy:NAN;
        audit->available_residual=isfinite(audit->available)&&
            isfinite(audit->available_expected)?
            audit->available-audit->available_expected:NAN;
        audit->modeled_work=work_valid?energy.modeled_drag_work:NAN;
        audit->nominal_margin=isfinite(audit->available)&&
            isfinite(audit->modeled_work)?
            audit->available-audit->modeled_work:NAN;
        audit->uncertainty=energy.valid?energy.uncertainty_specific_energy:NAN;
        audit->required=required;
        audit->margin=margin;
        audit->altitude_residual=t->trajectory_altitude_residual;
        audit->speed_residual=t->trajectory_speed_residual;
        audit->range_residual=t->trajectory_range_residual;
        double gravity=planet_surface_gravity(p);
        audit->altitude_uncertainty=isfinite(audit->altitude_residual)?
            gravity*fabs(audit->altitude_residual):0.0;
        double dv=isfinite(audit->speed_residual)?
            fabs(audit->speed_residual):0.0;
        audit->speed_uncertainty=isfinite(t->true_air_speed)?
            fabs(t->true_air_speed)*dv+.5*dv*dv:0.0;
        double mean_loss=path_length>0.0&&work_valid?
            work/path_length:0.0;
        audit->range_uncertainty=isfinite(audit->range_residual)?
            fabs(mean_loss*audit->range_residual):0.0;
        audit->physics_relative_uncertainty=0.0;
        audit->physics_relative_uncertainty_raw=0.0;
        if(isfinite(t->physics_certified_uncertainty)&&
           t->physics_certified_uncertainty>0.0)
            audit->physics_relative_uncertainty_raw=t->physics_certified_uncertainty;
        if(isfinite(t->predicted_physics_relative_uncertainty)&&
           t->predicted_physics_relative_uncertainty>0.0)
            audit->physics_relative_uncertainty_raw=fmax(
                audit->physics_relative_uncertainty_raw,
                t->predicted_physics_relative_uncertainty);
        audit->physics_relative_uncertainty=audit->physics_relative_uncertainty_raw;
        /* Keep the audit ledger identical to decision_energy_path_envelope().
           Active-regime evidence may reduce the global multi-regime uncertainty,
           but it must retain its measured magnitude instead of being capped to
           an artificially optimistic 8%. */
        if(isfinite(t->mach) && t->mach < 1.0 &&
           audit->physics_relative_uncertainty > 0.08) {
            double active_unc=0.0;
            bool active_evidence=false;
            if(isfinite(t->aerodynamic_confidence) &&
               t->aerodynamic_confidence > 0.0) {
                active_unc=fmax(active_unc,1.0-t->aerodynamic_confidence);
                active_evidence=true;
            }
            if(isfinite(t->physics_model_residual) &&
               t->physics_model_residual >= 0.0 &&
               isfinite(t->physics_model_residual_confidence) &&
               t->physics_model_residual_confidence > 0.0) {
                active_unc=fmax(active_unc,t->physics_model_residual);
                active_evidence=true;
            }
            audit->physics_relative_uncertainty=fmin(
                audit->physics_relative_uncertainty,
                active_evidence?clampd(active_unc,0.0,0.35):0.08);
        }
        audit->physics_uncertainty=work_valid?
            work*audit->physics_relative_uncertainty:0.0;
        audit->uncertainty_reconstructed=audit->altitude_uncertainty+
            audit->speed_uncertainty+audit->range_uncertainty+
            audit->physics_uncertainty;
        audit->uncertainty_residual=isfinite(audit->uncertainty)?
            audit->uncertainty-audit->uncertainty_reconstructed:NAN;
        audit->exit_altitude=end_altitude;
        audit->exit_speed=end_speed;
        audit->exit_specific_energy=rotating_specific_energy(t->latitude,
            end_altitude,end_speed,p);
        audit->closure=isfinite(audit->specific_energy)&&
            isfinite(audit->exit_specific_energy)&&work_valid?
            (audit->specific_energy-audit->exit_specific_energy)-work:NAN;
        audit->arc_work_shadow=arc_work;
        audit->final_work_shadow=final_work;
        audit->arc_length_expected=requested_arc_length;
        audit->arc_mean_drag_accel=isfinite(arc_work)&&requested_arc_length>0.0?
            arc_work/requested_arc_length:NAN;
        audit->partition_residual=isfinite(arc_work)&&isfinite(final_work)?
            work-arc_work-final_work:0.0;
        audit->modeled_work_residual=isfinite(audit->modeled_work)&&isfinite(work)?
            audit->modeled_work-work:NAN;
        audit->required_work_residual=isfinite(audit->required)&&
            isfinite(arc_work)&&isfinite(final_work)?
            audit->required-(arc_work+final_work):NAN;
        double arc_end_specific_energy=rotating_specific_energy(t->latitude,
            arc_end_altitude,arc_end_speed,p);
        audit->arc_energy_closure=isfinite(audit->specific_energy)&&
            isfinite(arc_end_specific_energy)&&isfinite(arc_work)?
            (audit->specific_energy-arc_end_specific_energy)-arc_work:NAN;
        audit->final_energy_closure=isfinite(arc_end_specific_energy)&&
            isfinite(audit->exit_specific_energy)&&isfinite(final_work)?
            (arc_end_specific_energy-audit->exit_specific_energy)-final_work:NAN;
    }
    /* This helper is called per candidate.  Report the candidate itself as
       sustainable only when its integrated path passes; do not manufacture a
       larger virtual radius from a point-loss algebraic shortcut. */
    double energy_radius=isfinite(margin)&&margin>=0.0?
        requested_radius:0.0;
    if(radius_out)*radius_out=energy_radius;
    if(margin_out)*margin_out=margin;
    if(loss_out)*loss_out=loss;
    if(available_out)*available_out=available;
    if(required_out)*required_out=required;
    return isfinite(available)&&available>0.0&&isfinite(loss)&&loss>=0.0&&
        isfinite(energy_radius)&&energy_radius>=requested_radius&&margin>=0.0;
}

       bool terminal_hac_debug_force_requested(void){
    const char*force_debug_env=getenv("KSP_LANDER_HAC_DEBUG_FORCE_CAPTURE");
    const char*diagnostics_env=getenv("KSP_LANDER_HAC_DIAGNOSTICS");
    const char*command_trace_env=getenv("KSP_LANDER_HAC_COMMAND_TRACE");
    return force_debug_env&&strcmp(force_debug_env,"1")==0&&
        diagnostics_env&&strcmp(diagnostics_env,"1")==0&&
        command_trace_env&&strcmp(command_trace_env,"1")==0;
}

       bool terminal_hac_debug_fixture_state(const Telemetry*t,double course,
        const LandingConfiguration*cfg,double*capture_course_out){
    if(capture_course_out)*capture_course_out=NAN;
    if(!t||!cfg||!isfinite(course)||!isfinite(t->mean_altitude)||
       !isfinite(t->runway_along_track)||!isfinite(t->runway_cross_track)||
       !(t->true_air_speed>0.0)||!isfinite(t->lift_force)||!(t->lift_force>0.0))
        return false;
    const GuidanceSettings*s=&cfg->guidance;
    if(t->mean_altitude<s->mm305_min_altitude||
       t->mean_altitude>s->mm305_max_altitude)return false;

    TaemHandoffContract contract=taem_handoff_contract(s);
    double positive=norm_deg(cfg->site.runway_heading+90.0);
    double negative=norm_deg(cfg->site.runway_heading-90.0);
    double positive_error=fabs(norm_signed_deg(positive-course));
    double negative_error=fabs(norm_signed_deg(negative-course));
    double capture_course=positive_error<=negative_error?positive:negative;
    double course_error=fmin(positive_error,negative_error);
    double station=-s->final_approach_distance;
    double position_error=hypot(t->runway_along_track-station,
        t->runway_cross_track);
    double position_limit=fmax(1000.0,contract.horizontal_radius_m);

    /* The focused MM305 diagnostic starts at the original 8 km fixture
       station, while the executable final-alignment station may be moved
       inward to 3.2 km.  Permit that finite bridge only when all three
       command-path diagnostic gates are explicitly enabled.  This does not
       alter normal MM304 acceptance or the measured-authority gate: it only
       lets the forced diagnostic reach the fixed, runway-anchored HAC so its
       lead and analytic arc can be inspected. */
    const char*force_debug_env=getenv("KSP_LANDER_HAC_DEBUG_FORCE_CAPTURE");
    const char*diagnostics_env=getenv("KSP_LANDER_HAC_DIAGNOSTICS");
    const char*command_trace_env=getenv("KSP_LANDER_HAC_COMMAND_TRACE");
    bool command_path_diagnostic=force_debug_env&&
        strcmp(force_debug_env,"1")==0&&diagnostics_env&&
        strcmp(diagnostics_env,"1")==0&&command_trace_env&&
        strcmp(command_trace_env,"1")==0;
    if(command_path_diagnostic)
        position_limit=fmax(position_limit,0.5*s->hac_radius);

    if(course_error>contract.perpendicular_heading_half_width_deg||
       position_error>position_limit)return false;
    if(capture_course_out)*capture_course_out=capture_course;
    return true;
}

