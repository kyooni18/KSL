/* The entry range that lets a drag-limited vehicle shed its entry energy by the TAEM interface grows
   with that energy: retrograde and high-inclination orbits arrive with higher air-relative speed than
   the prograde orbit the configured range was tuned for. */
static double deorbit_effective_entry_range(const GuidanceSettings*s,const PlanetModel*p,double speed){
    double ref=s->entry_range_reference_speed;
    if(!(ref>0.0)||!isfinite(speed)||!(speed>0.0))return s->target_entry_range;
    double vt=fmax(1.0,s->mm305_target_mach)*320.0;
    double g=p->gravitational_parameter/((p->radius+60000.0)*(p->radius+60000.0));
    double dh=fmax(0.0,p->atmosphere_depth-s->entry_interface_altitude_margin-s->taem_interface_altitude);
    double pe=2.0*g*dh;
    double scale=(speed*speed-vt*vt+pe)/fmax(1.0,ref*ref-vt*vt+pe);
    return s->target_entry_range*clampd(scale,0.7,2.0);
}

static void deorbit_passive_runway_margin(GeoPoint geo,const LandingSite*site,
        double radius,DeorbitEnergyMargin*m){
    GeoPoint site_geo={site->latitude,site->longitude,site->altitude};
    double along=0.0,cross=0.0;
    runway_coordinates(geo,site_geo,site->runway_heading,radius,&along,&cross);
    m->impact_runway_end=0;
    if(site->allow_reciprocal_runway){
        LandingSite reciprocal=runway_reciprocal_site(site,radius);
        GeoPoint reciprocal_geo={reciprocal.latitude,reciprocal.longitude,
            reciprocal.altitude};
        double reciprocal_along=0.0,reciprocal_cross=0.0;
        runway_coordinates(geo,reciprocal_geo,reciprocal.runway_heading,radius,
            &reciprocal_along,&reciprocal_cross);
        /* Passive overflight reserve belongs to either usable runway end.
           A western approach need not pass the RW09 threshold to reach RW27. */
        if(isfinite(reciprocal_along)&&reciprocal_along>along){
            along=reciprocal_along;
            cross=reciprocal_cross;
            m->impact_runway_end=1;
        }
    }
    m->impact_along_track=along;
    m->impact_cross_track=cross;
}

static DeorbitEnergyMargin deorbit_energy_margin_probe(VehicleState after,const PlanetModel*p,
        AerodynamicModel aero,const AerodynamicEnvelope*env,const TrajectoryCalibrationModel*cal,
        const LandingSite*site,const VehicleProfile*v,const GuidanceSettings*s){
    DeorbitEnergyMargin m={0};
    /* Zero maximum bank means no S-turn/lateral energy-management maneuver. The
       normal safe entry AoA schedule still flies the vehicle, so this asks the
       physically useful question: with wings level, does the post-burn state carry
       just beyond KSC? Any normal MM304 bank then has positive disposable energy to
       spend on geometry instead of stretching an undershooting trajectory. */
    EntryPrediction passive=predictor_simulate_entry(after,p,aero,env,cal,v,site,s,0.0,0,0,0,4500,false);
    VehicleState final=passive.final_state;
    GeoPoint geo=predictor_geo_point(final.position,p,final.ut);
    if(getenv("KSP_LANDER_TRACE_PLAN_TIMING"))
        fprintf(stderr,"PLAN_TIMING probe passive_s=%.0f entered=%d alt_km=%.2f vs=%.1f\n",
            final.ut-after.ut,passive.entered_atmosphere?1:0,
            predictor_geo_point(final.position,p,final.ut).altitude/1000.0,
            vdot(final.velocity,vnorm(final.position,v3(0,1,0))));
    Vector3 up=vnorm(final.position,v3(0,1,0));
    Vector3 surface_velocity=vsub(final.velocity,vcross(planet_rotation_vector(p),final.position));
    double vertical=vdot(surface_velocity,up);
    GeoPoint site_geo={site->latitude,site->longitude,site->altitude};
    deorbit_passive_runway_margin(geo,site,p->radius,&m);
    m.valid=passive.entered_atmosphere&&isfinite(geo.altitude)&&
        isfinite(m.impact_along_track)&&isfinite(m.impact_cross_track)&&
        isfinite(vertical)&&geo.altitude<=site->altitude+750.0&&vertical<0.0;
    m.impact_range=great_circle_distance(geo,site_geo,p->radius);
    m.impact_altitude=geo.altitude;m.impact_speed=vmag(surface_velocity);
    entry_prediction_clear(&passive);
    return m;
}
static bool deorbit_energy_margin_qualified(const DeorbitEnergyMargin*m,
        const GuidanceSettings*s,bool strict){
    (void)s;(void)strict;
    return m&&m->valid&&isfinite(m->impact_along_track)&&
        m->impact_along_track>=0.0;
}

static void candidate_clear(Candidate*c){entry_prediction_clear(&c->pred);}
static double deorbit_candidate_safety_violation(const EntryPrediction*p,
        const VehicleProfile*v,const GuidanceSettings*s){
    if(!p||!v||!s||!p->entered_atmosphere)return INFINITY;
    double q=predictor_normalized_upper_violation(
        p->peak_dynamic_pressure,v->maximum_dynamic_pressure);
    double g=predictor_normalized_upper_violation(
        p->peak_g_load,v->maximum_g_load);
    double fpa=fmax(0.0,s->maximum_entry_flight_path_angle-
        p->entry_flight_path_angle);
    return fmax(q,fmax(g,fpa));
}

static int tier(Candidate*c,const LandingSite*site,const VehicleProfile*v,
        const GuidanceSettings*s){
    if(!c)return 0;
    bool safe=deorbit_capture_qualified(&c->pred,site,v,s,c->peri,false);
    bool energy=deorbit_energy_margin_qualified(&c->energy_margin,s,false);
    bool landing=deorbit_runway_capture_qualified(
        &c->pred,site,v,s,c->peri);
    if(safe&&energy&&c->pred.mm304_gate_recorded&&landing)return 4;
    if(safe&&energy&&c->pred.mm304_gate_recorded)return 3;
    if(safe&&energy)return 2;
    if(safe)return 1;
    return 0;
}

static bool impulsive_deorbit_periapsis_plausible(VehicleState coast,double dv,double normal_angle,
        const PlanetModel*p,const GuidanceSettings*s){
    (void)s;
    VehicleState impulse=coast;
    Vector3 direction=burn_direction(coast.position,coast.velocity,0,normal_angle);
    impulse.velocity=vadd(impulse.velocity,vscale(direction,dv));
    double peri=predictor_postburn_periapsis(impulse,p);
    return isfinite(peri)&&peri<p->atmosphere_depth&&peri>-p->radius;
}

static bool eval_candidate(Candidate*out,VehicleState coast,double dv,double normal_angle,
        const PlanetModel*p,AerodynamicModel aero,const AerodynamicEnvelope*env,
        const TrajectoryCalibrationModel*cal,const LandingSite*site,
        const VehicleProfile*v,const GuidanceSettings*s,double thrust,
        DeorbitRejectReason*reject){
    memset(out,0,sizeof(*out));
    if(reject)*reject=DEORBIT_REJECT_IMPULSIVE_PERIAPSIS;
    if(!impulsive_deorbit_periapsis_plausible(coast,dv,normal_angle,p,s))return false;

    double dur=0.0,delivered=0.0;
    VehicleState after=retro_burn_scenario(coast,dv,thrust,
        s->deorbit_maximum_throttle,s->deorbit_throttle_ramp_duration,
        p,0,normal_angle,&dur,&delivered,cal?cal->physics:NULL);
    double dv_tol=fmax(GUIDANCE_BURN_TERMINAL_DV_TOLERANCE_MPS,
        sqrt(DBL_EPSILON)*fmax(1.0,fabs(dv)));
    if(reject)*reject=DEORBIT_REJECT_FINITE_BURN;
    if(!isfinite(dur)||!isfinite(delivered)||delivered+dv_tol<dv)
        return false;

    double peri=predictor_postburn_periapsis(after,p);
    if(reject)*reject=DEORBIT_REJECT_POSTBURN_PERIAPSIS;
    if(!isfinite(peri)||peri>=p->atmosphere_depth||peri<=-p->radius)
        return false;

    if(reject)*reject=DEORBIT_REJECT_ENTRY_PREDICTION;
    EntryPrediction pr=predictor_simulate_entry(after,p,aero,env,cal,v,site,s,
        v->maximum_bank_angle,0,0,0,4500,false);
    if(!pr.entered_atmosphere||!isfinite(pr.entry_range)||
       !isfinite(pr.entry_flight_path_angle)||!isfinite(pr.entry_course_error)||
       !isfinite(pr.peak_dynamic_pressure)||!isfinite(pr.peak_g_load)){
        entry_prediction_clear(&pr);
        return false;
    }

    DeorbitEnergyMargin energy_margin=
        deorbit_energy_margin_probe(after,p,aero,env,cal,site,v,s);

    out->burn_ut=coast.ut+dur*.5;
    out->dv=dv;
    out->duration=dur;
    out->burn_normal_angle=normal_angle;
    out->after=after;
    out->peri=peri;
    out->requested_periapsis_altitude=s->target_post_burn_periapsis_altitude;
    out->energy_margin=energy_margin;
    out->pred=pr;

    out->safety_violation=deorbit_candidate_safety_violation(&pr,v,s);
    out->energy_deficit=energy_margin.valid?
        fmax(0.0,-energy_margin.impact_along_track):INFINITY;
    out->energy_excess=energy_margin.valid?
        fmax(0.0,energy_margin.impact_along_track):INFINITY;
    out->entry_fpa_error=fabs(
        pr.entry_flight_path_angle-s->target_entry_flight_path_angle);
    out->entry_course_error=fabs(pr.entry_course_error);
    double entry_central_angle=pr.entry_range/fmax(p->radius,DBL_MIN);
    double entry_cross_sine=sin(entry_central_angle)*sin(pr.entry_course_error*DEG2RAD);
    out->entry_groundtrack_crossrange_error=fabs(
        asin(clampd(entry_cross_sine,-1.0,1.0))*p->radius);
    out->entry_range_error=fabs(pr.entry_range-s->target_entry_range);
    out->handoff_alignment_utilization=INFINITY;
    out->handoff_range_error=INFINITY;
    if(pr.reached_taem&&!pr.taem_ownership_boundary_missed){
        out->handoff_alignment_utilization=
            deorbit_alignment_contract_utilization(&pr,site,s);
        if(isfinite(pr.taem_range_error))
            out->handoff_range_error=fabs(pr.taem_range_error);
    }
    out->tier=tier(out,site,v,s);

    /* Kept only for telemetry/backward-compatible traces; not used to decide. */
    out->score=out->tier>=2?out->entry_fpa_error:out->safety_violation;
    if(reject)*reject=DEORBIT_REJECT_NONE;
    return true;
}


typedef struct {
    double timing_offset,thrust_scale,dv_offset,mass_scale,position_offset,velocity_offset;
    double radial_error,normal_error,atmosphere_fraction;
    double aero_drag_scale,aero_lift_scale;
} BurnStressCase;

typedef struct {
    unsigned scenarios,passed,unsafe;
    unsigned recovery_passed;
    unsigned entry_stage_passed;
    double pass_fraction,recovery_pass_fraction,worst_closest,worst_taem_error,worst_alignment_error,worst_entry_angle,min_periapsis,max_dynamic_pressure,max_g_load;
    double entry_stage_pass_fraction;
    bool qualified,recovery_qualified,entry_stage_qualified;
} RobustAssessment;

static bool hard_unsafe(const EntryPrediction*pr,double peri,const VehicleProfile*v,const GuidanceSettings*s,bool burn_completed){
    if(!burn_completed||!pr->entered_atmosphere)return true;
    (void)peri;
    if(pr->entry_flight_path_angle<s->maximum_entry_flight_path_angle)return true;
    if(pr->peak_dynamic_pressure>v->maximum_dynamic_pressure)return true;
    if(pr->peak_g_load>v->maximum_g_load)return true;
    return false;
}

static EntryPrediction deorbit_executable_policy_witness(const Candidate*c,
        const PlanetModel*p,AerodynamicModel aero,const AerodynamicEnvelope*env,
        const TrajectoryCalibrationModel*cal,const LandingSite*site,
        const VehicleProfile*v,const GuidanceSettings*s,
        const Telemetry*authority_source,const TerminalModel*terminal_model,
        bool run_terminal,bool include_trajectory);

static RobustAssessment assess_candidate_robustness(const Candidate*c,VehicleState initial,const PlanetModel*p,AerodynamicModel aero,const AerodynamicEnvelope*env,const TrajectoryCalibrationModel*cal,const LandingSite*site,const VehicleProfile*v,const GuidanceSettings*s,double thrust,const Telemetry*authority_source,const TerminalModel*terminal_model,bool full_terminal_stress,bool trace){
    double timing=s->deorbit_timing_uncertainty,thrust_unc=s->deorbit_thrust_uncertainty_fraction,dv_unc=s->deorbit_delta_v_uncertainty;
    double mass_unc=s->deorbit_mass_uncertainty_fraction,pos_unc=s->deorbit_position_uncertainty,vel_unc=s->deorbit_velocity_uncertainty;
    double pointing=s->deorbit_pointing_uncertainty,atmos=s->deorbit_atmosphere_uncertainty_fraction;
    double aero_unc=.30;
    if(cal&&cal->physics){
        const VesselPhysicsModel*m=cal->physics;
        if(m->certified_uncertainty>0)aero_unc=m->certified_uncertainty;
        if(m->model_residual_confidence>.20)aero_unc=fmax(aero_unc,m->model_residual*clampd(m->model_residual_confidence,.25,1));
    }
    /* A globally credible Mach envelope can still leave a particular trajectory
       outside the local q/Mach/AoA/configuration cells represented by prior-flight
       force data. Treat analytic-fallback dwell time as epistemic uncertainty
       instead of granting it the same local evidence as a direct cell. */
    double aero_seconds=c->pred.physics_observed_seconds+c->pred.physics_fallback_seconds;
    if(aero_seconds>1.0&&c->pred.physics_fallback_seconds>0&&aero_unc<.30){
        double fallback_fraction=clampd(c->pred.physics_fallback_seconds/aero_seconds,0,1);
        aero_unc+=fallback_fraction*(.30-aero_unc);
    }
    aero_unc=clampd(aero_unc,.08,.40);
    BurnStressCase cases[]={
        {0,1,0,1,0,0,0,0,0,0,0},
        { timing,1-thrust_unc,-dv_unc,1+mass_unc, pos_unc,-vel_unc, pointing,0,0,0,0},
        {-timing,1+thrust_unc, dv_unc,fmax(.5,1-mass_unc),-pos_unc, vel_unc,-pointing,0,0,0,0},
        { timing,1-thrust_unc,0,1+mass_unc,0,-vel_unc,0, pointing,0,0,0},
        {-timing,1+thrust_unc,0,fmax(.5,1-mass_unc),0, vel_unc,0,-pointing,0,0,0},
        {0,1,-dv_unc,1+mass_unc, pos_unc,0, pointing,0, atmos,0,0},
        {0,1, dv_unc,fmax(.5,1-mass_unc),-pos_unc,0,-pointing,0,-atmos,0,0},
        {0,1,0,1, pos_unc, vel_unc,0, pointing, atmos,0,0},
        {0,1,0,1,-pos_unc,-vel_unc,0,-pointing,-atmos,0,0},
        { timing,1,0,1+mass_unc, pos_unc, vel_unc,-pointing, pointing, atmos,0,0},
        {-timing,1,0,fmax(.5,1-mass_unc),-pos_unc,-vel_unc, pointing,-pointing,-atmos,0,0},
        {0,1-thrust_unc,-dv_unc,1+mass_unc,0,0,0,0, atmos,0,0},
        {0,1+thrust_unc, dv_unc,fmax(.5,1-mass_unc),0,0,0,0,-atmos,0,0},
        /* Shuttle certification explicitly stressed the aerodynamic data-book
           uncertainty rather than letting the flight computer relearn the
           vehicle during entry.  Worst-case drag-high/lift-low and the
           converse expose both energy and crossrange/control sensitivity. */
        {0,1,0,1,0,0,0,0,0,1+aero_unc,fmax(.50,1-aero_unc)},
        {0,1,0,1,0,0,0,0,0,fmax(.50,1-aero_unc),1+aero_unc},
    };
    RobustAssessment r;memset(&r,0,sizeof(r));
    r.worst_closest=0;r.worst_taem_error=0;r.worst_alignment_error=0;r.worst_entry_angle=DBL_MAX;r.min_periapsis=DBL_MAX;
    double nominal_start=c->burn_ut-c->duration*.5;
    for(size_t i=0;i<sizeof(cases)/sizeof(cases[0]);i++){
        BurnStressCase sc=cases[i];
        VehicleState scenario_state=initial;
        Vector3 radial=vnorm(initial.position,v3(1,0,0));
        Vector3 along=vnorm(vproject_plane(initial.velocity,radial),vnorm(initial.velocity,v3(0,1,0)));
        scenario_state.position=vadd(scenario_state.position,vscale(radial,sc.position_offset));
        scenario_state.velocity=vadd(scenario_state.velocity,vscale(along,sc.velocity_offset));
        scenario_state.mass=fmax(1,scenario_state.mass*sc.mass_scale);
        double start=fmax(scenario_state.ut+5,nominal_start+sc.timing_offset);
        VehicleState coast=predictor_propagate_vacuum(scenario_state,start,p,3);
        double scenario_dv=fmax(.2,c->dv+sc.dv_offset),duration=0,delivered=0;
        VehicleState after=retro_burn_scenario(coast,scenario_dv,thrust*fmax(.05,sc.thrust_scale),s->deorbit_maximum_throttle,s->deorbit_throttle_ramp_duration,p,sc.radial_error,c->burn_normal_angle+sc.normal_error,&duration,&delivered,cal?cal->physics:NULL);
        bool burn_completed=isfinite(duration)&&isfinite(delivered)&&
            delivered>=scenario_dv-.25;
        double peri=predictor_postburn_periapsis(after,p);
        TrajectoryCalibrationModel scenario_cal=*cal;
        scenario_cal.density_scale=clampd(cal->density_scale*(1+sc.atmosphere_fraction),.35,2.8);
        scenario_cal.stress_drag_scale=sc.aero_drag_scale>0?sc.aero_drag_scale:1;
        scenario_cal.stress_lift_scale=sc.aero_lift_scale>0?sc.aero_lift_scale:1;
        /* The reduced trajectory can screen entry and approximate drag work,
           but has no native MM305/Final commitment. Only actual full-route
           continuation can pass strict robustness when nominal route exists. */
        EntryPrediction pr;
        if(full_terminal_stress&&burn_completed){
            Candidate stressed={0};
            stressed.after=after;stressed.peri=peri;stressed.dv=scenario_dv;
            stressed.duration=duration;stressed.burn_ut=start+duration*.5;
            pr=deorbit_executable_policy_witness(&stressed,p,aero,env,
                &scenario_cal,site,v,s,authority_source,terminal_model,true,false);
        }else{
            pr=predictor_simulate_entry(after,p,aero,env,&scenario_cal,v,site,s,
                v->maximum_bank_angle,0,0,0,4500,false);
        }
        bool base_pass=burn_completed&&
            deorbit_runway_capture_qualified(&pr,site,v,s,peri);
        bool base_recovery=base_pass;
        DeorbitEnergyMargin energy_margin={0};
        if(base_pass||base_recovery)
            energy_margin=deorbit_energy_margin_probe(after,p,aero,env,&scenario_cal,site,v,s);
        bool pass=base_pass&&deorbit_energy_margin_qualified(&energy_margin,s,true);
        bool recovery=base_recovery&&deorbit_energy_margin_qualified(&energy_margin,s,false);
        bool entry_stage=deorbit_entry_stage_qualified(&pr,v,s,peri,
            site->altitude,p->atmosphere_depth,burn_completed);
        bool unsafe=hard_unsafe(&pr,peri,v,s,burn_completed);
        double align_metric=deorbit_alignment_contract_utilization(&pr,site,s);
        double align_pos=deorbit_alignment_position_error(&pr,s);
        double align_course=deorbit_alignment_course_error(&pr,site);
        if(trace){
            fprintf(stderr,"stress[%zu] pass=%d recovery=%d unsafe=%d timing=%+.1f dv=%+.1f atm=%+.2f aeroD=%.2f aeroL=%.2f entry=%.1fkm/%.2fdeg gate=%d %.1fkm x=%.1f y=%.1f course=%.1f FPA=%.2f V=%.0f align=%.1fkm/%.1fdeg h=%.1fkm V=%.0f reached=%d min=%.0f rev=%u closest=%.1fkm Pe=%.1fkm q=%.1fkPa g=%.2f\n",
                i,pass?1:0,recovery?1:0,unsafe?1:0,sc.timing_offset,sc.dv_offset,sc.atmosphere_fraction,scenario_cal.stress_drag_scale,scenario_cal.stress_lift_scale,
                pr.entry_range/1000,pr.entry_flight_path_angle,pr.mm304_gate_recorded?1:0,pr.mm304_gate_range/1000,pr.mm304_gate_along_track/1000,pr.mm304_gate_cross_track/1000,pr.mm304_gate_course,pr.mm304_gate_flight_path_angle,pr.mm304_gate_speed,
                align_pos/1000,align_course,pr.taem_altitude/1000,pr.taem_speed,pr.reached_taem?1:0,pr.minimum_entry_speed,pr.s_turn_reversals,pr.closest_distance/1000,peri/1000,pr.peak_dynamic_pressure/1000,pr.peak_g_load);
            fprintf(stderr,"  energy-margin impact valid=%d along=%+.1fkm cross=%+.1fkm range=%.1fkm V=%.0f\n",
                energy_margin.valid?1:0,energy_margin.impact_along_track/1000.0,energy_margin.impact_cross_track/1000.0,
                energy_margin.impact_range/1000.0,energy_margin.impact_speed);
        }
        r.scenarios++;
        if(pass)r.passed++;
        if(recovery)r.recovery_passed++;
        if(entry_stage)r.entry_stage_passed++;
        if(unsafe)r.unsafe++;
        if(isfinite(pr.closest_distance))r.worst_closest=fmax(r.worst_closest,pr.closest_distance);
        if(isfinite(pr.taem_range_error))r.worst_taem_error=fmax(r.worst_taem_error,fabs(pr.taem_range_error));
        if(isfinite(align_metric))r.worst_alignment_error=fmax(r.worst_alignment_error,align_metric);
        if(pr.entered_atmosphere&&isfinite(pr.entry_flight_path_angle))r.worst_entry_angle=fmin(r.worst_entry_angle,pr.entry_flight_path_angle);
        if(isfinite(peri))r.min_periapsis=fmin(r.min_periapsis,peri);
        r.max_dynamic_pressure=fmax(r.max_dynamic_pressure,pr.peak_dynamic_pressure);
        r.max_g_load=fmax(r.max_g_load,pr.peak_g_load);
        entry_prediction_clear(&pr);
    }
    if(r.worst_entry_angle==DBL_MAX)r.worst_entry_angle=0;
    if(r.min_periapsis==DBL_MAX)r.min_periapsis=-DBL_MAX;
    r.pass_fraction=r.scenarios?(double)r.passed/r.scenarios:0;
    r.recovery_pass_fraction=r.scenarios?(double)r.recovery_passed/r.scenarios:0;
    r.entry_stage_pass_fraction=r.scenarios?
        (double)r.entry_stage_passed/r.scenarios:0;
    r.qualified=c->executable_witness_capture&&
        r.pass_fraction+1e-9>=s->deorbit_robust_minimum_pass_fraction&&r.unsafe==0;
    r.recovery_qualified=c->executable_witness_capture&&
        deorbit_energy_margin_qualified(&c->energy_margin,s,false)&&
        r.recovery_pass_fraction+1e-9>=s->deorbit_robust_minimum_pass_fraction&&r.unsafe==0;
    r.entry_stage_qualified=deorbit_entry_stage_robustness_qualified(
        r.entry_stage_passed,r.scenarios,r.unsafe,
        s->deorbit_robust_minimum_pass_fraction);
    return r;
}


static double deorbit_finite_burn_periapsis(VehicleState coast,double dv,double normal_angle,
        const PlanetModel*p,const GuidanceSettings*s,double thrust,
        const TrajectoryCalibrationModel*cal){
    double duration=0.0,delivered=0.0;
    VehicleState after=retro_burn_scenario(coast,dv,thrust,
        s->deorbit_maximum_throttle,s->deorbit_throttle_ramp_duration,
        p,0,normal_angle,&duration,&delivered,cal?cal->physics:NULL);
    double tolerance=fmax(GUIDANCE_BURN_TERMINAL_DV_TOLERANCE_MPS,
        sqrt(DBL_EPSILON)*fmax(1.0,fabs(dv)));
    if(!isfinite(duration)||!isfinite(delivered)||
       delivered+tolerance<dv)
        return NAN;
    return predictor_postburn_periapsis(after,p);
}

static bool deorbit_solve_target_periapsis(VehicleState coast,double normal_angle,
        const PlanetModel*p,const GuidanceSettings*s,double thrust,
        const TrajectoryCalibrationModel*cal,double*out_dv,
        DeorbitRejectReason*reject){
    if(reject)*reject=DEORBIT_REJECT_TARGET_INPUT;
    if(!out_dv||!p||!s||!(thrust>0.0)||
       !isfinite(s->target_post_burn_periapsis_altitude))
        return false;

    double low=0.0;
    double speed=vmag(coast.velocity);
    if(!(speed>0.0)||!isfinite(speed)){
        if(reject)*reject=DEORBIT_REJECT_TARGET_ORBIT;
        return false;
    }

    double low_peri=deorbit_finite_burn_periapsis(
        coast,low,normal_angle,p,s,thrust,cal);
    double target=s->target_post_burn_periapsis_altitude;
    if(!isfinite(low_peri)){
        if(reject)*reject=DEORBIT_REJECT_TARGET_LOW_STATE;
        return false;
    }

    if(low_peri<=target){
        *out_dv=0.0;
        if(reject)*reject=DEORBIT_REJECT_NONE;
        return true;
    }

    /* Use the impulsive two-body solution to seed a finite-burn bracket. The
       full orbital speed is only a geometric bound and can demand far more
       propellant than the requested periapsis change. Expand the numerical
       bracket only when the modeled finite burn still misses the configured
       periapsis, using live thrust and mass flow. */
    double impulse_low=0.0,impulse_high=speed;
    Vector3 direction=burn_direction(coast.position,coast.velocity,0.0,normal_angle);
    VehicleState impulse=coast;
    impulse.velocity=vadd(impulse.velocity,vscale(direction,impulse_high));
    double impulse_high_peri=predictor_postburn_periapsis(impulse,p);
    if(!isfinite(impulse_high_peri)||impulse_high_peri>target){
        if(reject)*reject=DEORBIT_REJECT_TARGET_IMPULSE_BOUND;
        return false;
    }
    for(int i=0;i<48;i++){
        double mid=.5*(impulse_low+impulse_high);
        impulse=coast;
        impulse.velocity=vadd(coast.velocity,vscale(direction,mid));
        double peri=predictor_postburn_periapsis(impulse,p);
        if(!isfinite(peri)){
            if(reject)*reject=DEORBIT_REJECT_TARGET_IMPULSE_ROOT;
            return false;
        }
        if(peri>target)impulse_low=mid;
        else impulse_high=mid;
    }

    double high=impulse_high;
    double high_peri=deorbit_finite_burn_periapsis(
        coast,high,normal_angle,p,s,thrust,cal);
    if(!isfinite(high_peri)){
        if(reject)*reject=DEORBIT_REJECT_TARGET_FINITE_SEED;
        return false;
    }

    while(high_peri>target){
        low=high;
        if(high>=speed){
            if(reject)*reject=DEORBIT_REJECT_TARGET_FINITE_BRACKET;
            return false;
        }
        high=fmin(speed,high*2.0);
        if(!(high>low)){
            if(reject)*reject=DEORBIT_REJECT_TARGET_FINITE_BRACKET;
            return false;
        }
        high_peri=deorbit_finite_burn_periapsis(
            coast,high,normal_angle,p,s,thrust,cal);
        if(!isfinite(high_peri)){
            if(reject)*reject=DEORBIT_REJECT_TARGET_FINITE_BRACKET;
            return false;
        }
    }

    /* Stop when the delta-v bracket is well below the actual finite-burn
       cutoff accuracy (0.08 m/s). Forty-eight rounds unnecessarily integrate
       thousands of identical quarter-second burns to sub-nanometer delta-v
       precision. Retain bisection and both bracket signs, but resolve to
       <=0.01 m/s; the last accepted midpoint differs by <=0.005 m/s. */
    const double dv_bracket_tolerance=fmin(.01,
        .125*GUIDANCE_BURN_TERMINAL_DV_TOLERANCE_MPS);
    for(int i=0;i<48&&high-low>dv_bracket_tolerance;i++){
        double mid=.5*(low+high);
        double peri=deorbit_finite_burn_periapsis(
            coast,mid,normal_angle,p,s,thrust,cal);
        if(!isfinite(peri)){
            if(reject)*reject=DEORBIT_REJECT_TARGET_FINITE_ROOT;
            return false;
        }
        if(peri>target)low=mid;
        else high=mid;
    }
    *out_dv=.5*(low+high);
    bool solved=isfinite(*out_dv)&&*out_dv>=0.0;
    if(solved&&reject)*reject=DEORBIT_REJECT_NONE;
    return solved;
}

static bool deorbit_evaluate_start(Candidate*out,VehicleState initial,
        double burn_start,double dv_offset,double normal_angle,const PlanetModel*p,
        AerodynamicModel aero,const AerodynamicEnvelope*env,
        const TrajectoryCalibrationModel*cal,const LandingSite*site,
        const VehicleProfile*v,const GuidanceSettings*s,double thrust,
        DeorbitRejectReason*reject){
    if(reject)*reject=DEORBIT_REJECT_START;
    if(!out||!isfinite(burn_start)||burn_start<initial.ut)return false;
    VehicleState coast=predictor_propagate_vacuum(
        initial,burn_start,p,12);
    double dv=0.0;
    if(!deorbit_solve_target_periapsis(
            coast,normal_angle,p,s,thrust,cal,&dv,reject))
        return false;
    dv=fmax(0.0,dv+dv_offset);
    return eval_candidate(out,coast,dv,normal_angle,p,aero,env,cal,site,v,s,thrust,reject);
}

static void copy_control_authority_evidence(Telemetry *destination,
        const Telemetry *source){
    if(!destination||!source)return;
    destination->attitude_response=source->attitude_response;
    memcpy(destination->physics_authority,source->physics_authority,
        sizeof(destination->physics_authority));
    memcpy(destination->physics_authority_confidence,
        source->physics_authority_confidence,
        sizeof(destination->physics_authority_confidence));
    destination->has_torque=source->has_torque;
    destination->available_pitch_torque=source->available_pitch_torque;
    destination->available_roll_torque=source->available_roll_torque;
    destination->available_yaw_torque=source->available_yaw_torque;
    destination->has_inertia=source->has_inertia;
    destination->pitch_moment_of_inertia=source->pitch_moment_of_inertia;
    destination->roll_moment_of_inertia=source->roll_moment_of_inertia;
    destination->yaw_moment_of_inertia=source->yaw_moment_of_inertia;
}

static EntryPrediction deorbit_executable_policy_witness(const Candidate*c,
        const PlanetModel*p,AerodynamicModel aero,const AerodynamicEnvelope*env,
        const TrajectoryCalibrationModel*cal,const LandingSite*site,
        const VehicleProfile*v,const GuidanceSettings*s,
        const Telemetry*authority_source,const TerminalModel*terminal_model,
        bool run_terminal,bool include_trajectory){
    EntryPrediction empty;memset(&empty,0,sizeof(empty));trajectory_init(&empty.trajectory);
    if(!c||!p||!env||!cal||!site||!v||!s)return empty;

    /* The reduced predictor remains a search model.  Final certification is a
       separate witness that executes the production MM304 guidance law from the
       selected post-burn state.  This prevents a biased reduced policy from both
       proposing and certifying the same deorbit. */
    LandingConfiguration cfg=landing_configuration_default();
    cfg.site=*site;cfg.vehicle=*v;cfg.guidance=*s;

    /* Above meaningful aerodynamic load, Entry Interface owns an inertial
       capture rather than a high-incidence atmospheric trim.  Seed the witness
       wings-level and zero-incidence; production guidance then selects the first
       S-turn side from runway/TAEM geometry when aerodynamic authority arrives. */
    PredictorGuidance response;memset(&response,0,sizeof(response));
    response.actual_bank=0.0;response.actual_aoa=0.0;
    AeroContext ctx={.p=p,.e=env,.c=cal,.v=&cfg.vehicle,.bank=0.0,.aoa=0.0,
        .ut=c->after.ut,.mass=c->after.mass,
        .gear=cal->physics?cal->physics->gear:false,
        .brakes=cal->physics?cal->physics->brakes:false,
        .airbrakes=0};
    Telemetry seed=guidance_shadow_telemetry(NULL,c->after,&response,&ctx,p,&cfg,NULL);
    /* The witness state is synthetic, but its control envelope must come from
       an observed adapter/model. Preserve only measured authority evidence;
       orbital state, airspeed, and other stale telemetry are not copied. */
    copy_control_authority_evidence(&seed,authority_source);

    GuidanceMachine guidance;
    guidance_initialize_reentry_continuation(&guidance,&seed,p,&cfg,1.0,false,env,cal);

    DeorbitPlan executed;memset(&executed,0,sizeof(executed));trajectory_init(&executed.trajectory);
    executed.created_ut=c->after.ut;executed.burn_ut=c->burn_ut;executed.delta_v=c->dv;
    executed.estimated_burn_duration=c->duration;
    executed.predicted_post_burn_periapsis_altitude=c->peri;
    executed.live_cutoff_capture_qualified=true;executed.execution_qualified=true;
    executed.achieved_state_verified=true;executed.achieved_state_capture_qualified=true;

    EntryPrediction witness;
    bool sts_table_witness=s->taem_energy_law&&!strcmp(v->model_id,"STS");
    if((run_terminal||sts_table_witness)&&terminal_model&&terminal_model->replay_validated&&
       terminal_model->planet_frame_aligned){
        TerminalShadowOutcome outcome;
        witness=predictor_simulate_guidance_shadow_core(c->after,&seed,&guidance,
            &executed,p,aero,env,cal,&cfg,4500.0,include_trajectory,
            false,!run_terminal,terminal_model,&outcome);
        witness.uncertainty_scenarios=witness.shadow_guidance_used?1u:0u;
    }else{
        witness=predictor_simulate_entry_guidance_shadow(c->after,&seed,&guidance,
            &executed,p,aero,env,cal,&cfg,4500.0,include_trajectory);
    }
    trajectory_clear(&executed.trajectory);
    return witness;
}

static void deorbit_record_execution_witness(Candidate*c,
        const EntryPrediction*witness,const LandingSite*site,
        const VehicleProfile*v,
        const GuidanceSettings*s){
    if(!c||!witness||!site||!v||!s)return;
    bool executable=witness->shadow_guidance_used&&witness->entered_atmosphere&&
        witness->mm304_gate_recorded&&!witness->shadow_aborted;
    c->executable_witness_evaluated=true;
    c->executable_witness_present=witness->shadow_guidance_used;
    c->executable_witness_capture=executable&&
        deorbit_runway_capture_qualified(witness,site,v,s,c->peri);
    c->executable_witness_handoff_valid=witness->reached_taem&&
        !witness->taem_ownership_boundary_missed;
    c->executable_witness_boundary_missed=
        witness->taem_ownership_boundary_missed;
    c->executable_witness_aborted=witness->shadow_aborted;
    c->executable_witness_taem_error=isfinite(witness->taem_interface_error)?
        witness->taem_interface_error:
        (isfinite(witness->taem_range_error)?
            fabs(witness->taem_range_error):INFINITY);
    c->executable_witness_range_error=isfinite(witness->taem_range_error)?
        witness->taem_range_error:NAN;
    c->executable_witness_alignment_error=
        deorbit_alignment_contract_utilization(witness,site,s);
    c->executable_witness_closest_distance=
        isfinite(witness->closest_distance)?witness->closest_distance:INFINITY;
}

/* Search refinement must compare the actual MM304 guidance witness, not
   choose a deorbit purely because its ballistic Entry-Interface miss is low.
   These tests describe an *acquisition opportunity*, not proof that the native
   MM305 solver can finish a runway approach. */
static bool deorbit_witness_closing(const EntryPrediction*w,const LandingSite*site){
    if(!w||!site||!isfinite(w->taem_along_track)||
       !isfinite(w->taem_cross_track)||!isfinite(w->taem_course))return false;
    double relative=(w->taem_course-site->runway_heading)*DEG2RAD;
    return w->taem_along_track*cos(relative)+
        w->taem_cross_track*sin(relative)<0.0;
}
static double deorbit_witness_range_debt(const EntryPrediction*w,
        const PlanetModel*p,const GuidanceSettings*s){
    if(!w||!isfinite(w->taem_distance)||!(w->taem_distance>=0.0))
        return INFINITY;
    double minimum=fmax(s->final_approach_distance*1.5,
        s->final_approach_distance+fmax(0.0,s->hac_radius));
    double maximum=entry_taem_range_target(p,s);
    return fmax(0.0,fmax(minimum-w->taem_distance,
        w->taem_distance-maximum));
}
static bool deorbit_witness_inlet_geometry(const EntryPrediction*w,
        const PlanetModel*p,const LandingSite*site,const GuidanceSettings*s){
    if(!w||!w->reached_taem||!isfinite(w->taem_altitude)||
       !isfinite(w->taem_speed)||w->taem_speed<=0.0||
       w->taem_altitude<s->mm305_min_altitude||
       w->taem_altitude>s->mm305_max_altitude||
       !isfinite(w->taem_course))return false;
    double heading_offset=fabs(norm_signed_deg(w->taem_course-site->runway_heading));
    return deorbit_witness_range_debt(w,p,s)<=0.0&&
        fabs(heading_offset-90.0)<=s->mm304_perpendicular_heading_half_width&&
        deorbit_witness_closing(w,site);
}
/* A negative analytic inlet reserve or a non-perpendicular course is not
   proof that the production MM305 law cannot acquire a runway route.  Spend
   the bounded native continuation on safe, closing, nearby physical inlets;
   only that continuation can prove Final. */
static bool deorbit_witness_terminal_trial(const Candidate *candidate,
        const EntryPrediction *w,const PlanetModel *p,const LandingSite *site,
        const VehicleProfile *v,const GuidanceSettings *s) {
    return candidate&&w&&deorbit_entry_stage_qualified(w,v,s,candidate->peri,
        site->altitude,p->atmosphere_depth,true)&&w->reached_taem&&
        isfinite(w->taem_altitude)&&w->taem_altitude>=s->mm305_min_altitude&&
        w->taem_altitude<=s->mm305_max_altitude&&
        deorbit_witness_range_debt(w,p,s)<=0.0&&deorbit_witness_closing(w,site);
}

/* Ground endpoint of a completed native attempt, not its closest high-altitude
   overflight.  This ranks uncertified recovery attempts only; it never grants
   a capture certificate or weakens Final's contract. */
static double deorbit_guided_terminal_miss(const Candidate *candidate,
        const EntryPrediction *w,const PlanetModel *p,const LandingSite *site) {
    if(!candidate->executable_witness_evaluated||!w->shadow_guidance_used||
       w->shadow_aborted||!isfinite(w->final_state.ut))return INFINITY;
    GeoPoint end=predictor_geo_point(w->final_state.position,p,w->final_state.ut);
    if(!isfinite(end.altitude)||end.altitude>site->altitude+750.0)return INFINITY;
    GeoPoint target={site->latitude,site->longitude,site->altitude};
    double miss=great_circle_distance(end,target,p->radius);
    return isfinite(miss)?miss:INFINITY;
}

double mm305_energy_law_handoff_reach(const struct TerminalModel *model,
        double altitude_above_site_m,double speed_mps,double mass_kg);

/* Site distance at which live experimental-STS MM304 aims to hand over:
   the TAEM energy-law reach from the band-centre Mach at the interface
   altitude, plus the Final alignment distance (guidance/entry/contract.inc).
   NAN when the native terminal model is unavailable. */
static double deorbit_sts_handoff_target(const TerminalModel*model,const PlanetModel*p,
        const LandingSite*site,const VehicleProfile*v,const GuidanceSettings*s,double mass){
    if(!model||!p||!site||!v||!s)return NAN;
    if(s->taem_hac_law){
        /* HAC law: the site distance live MM304 steers for (contract.inc),
           MM305's own reference point at the handoff Mach plus the Final leg.
           Ranking toward the 96.5 km interface range instead chose a burn
           that handed over 133 km out against a 44.6 km MM304 target
           (live 2026-10-29), short of the HAC. */
        TaemHacLiveAero live={0};
        double h=NAN,speed=NAN,station=NAN;
        if(!mm305_hac_handoff_state(model,&live,mass,s->mm305_target_mach,&h,&speed,&station))
            return NAN;
        return station+final_alignment_distance_for_state(NULL,h+site->altitude,speed,site,v,s);
    }
    double sound=planet_atmospheric_speed_of_sound(p,s->taem_interface_altitude);
    double reach=mm305_energy_law_handoff_reach(model,
        s->taem_interface_altitude-site->altitude,s->mm305_target_mach*sound,mass);
    return isfinite(reach)?reach+s->final_approach_distance:NAN;
}

static bool deorbit_guided_candidate_better(const Candidate*a,const EntryPrediction*aw,
        const Candidate*b,const EntryPrediction*bw,const PlanetModel*p,
        const LandingSite*site,const VehicleProfile*v,const GuidanceSettings*s,
        double sts_handoff_target){
    bool a_safe=deorbit_entry_stage_qualified(aw,v,s,a->peri,
        site->altitude,p->atmosphere_depth,true);
    bool b_safe=deorbit_entry_stage_qualified(bw,v,s,b->peri,
        site->altitude,p->atmosphere_depth,true);
    if(a_safe!=b_safe)return a_safe;
    bool a_capture=a_safe&&deorbit_runway_capture_qualified(aw,site,v,s,a->peri)&&
        deorbit_energy_margin_qualified(&a->energy_margin,s,true);
    bool b_capture=b_safe&&deorbit_runway_capture_qualified(bw,site,v,s,b->peri)&&
        deorbit_energy_margin_qualified(&b->energy_margin,s,true);
    if(a_capture!=b_capture)return a_capture;
    /* A small ground miss after an unqualified overflight is not a usable
       MM305 inlet. Rank the ownership boundary before endpoint proximity. */
    bool a_inlet=aw->taem_dynamic_interface_captured&&
        !aw->taem_ownership_boundary_missed;
    bool b_inlet=bw->taem_dynamic_interface_captured&&
        !bw->taem_ownership_boundary_missed;
    if(a_inlet!=b_inlet)return a_inlet;
    /* Experimental STS: rank by the handoff distance live MM304 will steer
       for, so the burn and MM304 agree.  The 60 deg flight's ranking ignored
       it (chose a 28 km handoff by heading test); ranking toward the 96.5 km
       configured interface range instead chose a burn MM304 could not reach
       its 39 km target from.  The analytic inlet margin ranked next is
       uniformly -200..-280 kJ/kg across these candidates. */
    if((s->sts_trajectory_delivery||s->taem_hac_law)&&!strcmp(v->model_id,"STS")&&isfinite(sts_handoff_target)&&
       aw->reached_taem&&bw->reached_taem&&
       isfinite(aw->taem_distance)&&isfinite(bw->taem_distance)){
        double ae=fabs(aw->taem_distance-sts_handoff_target);
        double be=fabs(bw->taem_distance-sts_handoff_target);
        if(fabs(ae-be)>1.0)return ae<be;
    }
    bool a_geometry=deorbit_witness_inlet_geometry(aw,p,site,s);
    bool b_geometry=deorbit_witness_inlet_geometry(bw,p,site,s);
    if(a_geometry!=b_geometry)return a_geometry;
    if(!a_geometry){
        double ad=deorbit_witness_range_debt(aw,p,s);
        double bd=deorbit_witness_range_debt(bw,p,s);
        if(ad<bd)return true;
        if(ad>bd)return false;
        bool ac=deorbit_witness_closing(aw,site);
        bool bc=deorbit_witness_closing(bw,site);
        if(ac!=bc)return ac;
    }
    /* A certified positive inlet reserve wins within the geometric capture
       set. When neither candidate has one, prefer the smaller energy deficit. */
    bool ae=isfinite(aw->taem_interface_energy_margin)&&
        aw->taem_interface_energy_margin>=0.0;
    bool be=isfinite(bw->taem_interface_energy_margin)&&
        bw->taem_interface_energy_margin>=0.0;
    if(ae!=be)return ae;
    if(!ae&&isfinite(aw->taem_interface_energy_margin)&&
       isfinite(bw->taem_interface_energy_margin)&&
       fabs(aw->taem_interface_energy_margin-
            bw->taem_interface_energy_margin)>1.0)
        return aw->taem_interface_energy_margin>
            bw->taem_interface_energy_margin;
    if(a_safe&&b_safe&&a->executable_witness_evaluated&&
       b->executable_witness_evaluated){
        double am=deorbit_guided_terminal_miss(a,aw,p,site);
        double bm=deorbit_guided_terminal_miss(b,bw,p,site);
        if(isfinite(am)&&isfinite(bm)&&fabs(am-bm)>1.0)
            return am<bm;
    }
    double nominal=entry_taem_range_target(p,s);
    double ad=fabs(aw->taem_distance-nominal);
    double bd=fabs(bw->taem_distance-nominal);
    if(ad<bd)return true;
    if(ad>bd)return false;
    return a->burn_ut<b->burn_ut;
}

typedef struct {
    bool valid,closing;
    double burn_start,burn_ut,dv,duration,peri,normal_angle;
    double entry_range,entry_fpa,entry_speed,entry_course_error;
    double entry_crossrange,entry_miss;
} DeorbitTimingProbe;

static bool deorbit_timing_probe_entry(DeorbitTimingProbe*out,VehicleState state,
        const PlanetModel*p,const LandingSite*site,const GuidanceSettings*s){
    if(!out||!p||!site||!s)return false;
    GeoPoint target={site->latitude,site->longitude,site->altitude};
    double entry_alt=entry_guidance_start_altitude(p,s),elapsed=0.0;
    while(elapsed<4500.0){
        GeoPoint geo=predictor_geo_point(state.position,p,state.ut);
        Vector3 up=vnorm(state.position,v3(0,1,0));
        Vector3 air=vsub(state.velocity,vcross(planet_rotation_vector(p),state.position));
        double speed=vmag(air),vertical=vdot(air,up);
        if(geo.altitude<=entry_alt&&vertical<0.0){
            double horizontal=vmag(vproject_plane(air,up));
            double bearing=initial_bearing(geo,target);
            double course=surface_course(state.position,state.velocity,
                planet_rotation_vector(p),p->north_axis,bearing);
            double signed_error=norm_signed_deg(bearing-course);
            double range=great_circle_distance(geo,target,p->radius);
            double central=range/fmax(p->radius,DBL_MIN);
            double cross_sine=sin(central)*sin(signed_error*DEG2RAD);
            out->entry_range=range;
            out->entry_fpa=atan2(vertical,fmax(horizontal,.1))*RAD2DEG;
            out->entry_speed=speed;
            out->entry_course_error=fabs(signed_error);
            out->entry_crossrange=fabs(asin(clampd(cross_sine,-1.0,1.0))*p->radius);
            out->entry_miss=hypot(out->entry_crossrange,
                range-deorbit_effective_entry_range(s,p,out->entry_speed));
            out->closing=out->entry_course_error<90.0;
            out->valid=isfinite(out->entry_range)&&isfinite(out->entry_fpa)&&
                isfinite(out->entry_speed)&&isfinite(out->entry_course_error)&&
                isfinite(out->entry_crossrange)&&isfinite(out->entry_miss);
            return out->valid;
        }
        if(!isfinite(geo.altitude)||geo.altitude<-1000.0)return false;
        double excess=geo.altitude-entry_alt;
        double dt=excess>10000.0?8.0:(excess>2000.0?2.0:.25);
        state=rk4_gravity(state,dt,p);
        elapsed+=dt;
    }
    return false;
}

static bool deorbit_timing_probe_start(DeorbitTimingProbe*out,VehicleState initial,
        double burn_start,double normal_angle,const PlanetModel*p,const TrajectoryCalibrationModel*cal,
        const LandingSite*site,const GuidanceSettings*s,double thrust,
        DeorbitRejectReason*reject){
    if(reject)*reject=DEORBIT_REJECT_START;
    if(!out||!isfinite(burn_start)||burn_start<initial.ut)return false;
    memset(out,0,sizeof(*out));
    VehicleState coast=predictor_propagate_vacuum(initial,burn_start,p,12.0);
    double dv=0.0;
    if(!deorbit_solve_target_periapsis(coast,normal_angle,p,s,thrust,cal,&dv,reject))return false;
    double duration=0.0,delivered=0.0;
    VehicleState after=retro_burn_scenario(coast,dv,thrust,
        s->deorbit_maximum_throttle,s->deorbit_throttle_ramp_duration,
        p,0.0,normal_angle,&duration,&delivered,cal?cal->physics:NULL);
    double tolerance=fmax(GUIDANCE_BURN_TERMINAL_DV_TOLERANCE_MPS,
        sqrt(DBL_EPSILON)*fmax(1.0,fabs(dv)));
    if(reject)*reject=DEORBIT_REJECT_FINITE_BURN;
    if(!isfinite(duration)||!isfinite(delivered)||delivered+tolerance<dv)return false;
    double peri=predictor_postburn_periapsis(after,p);
    if(reject)*reject=DEORBIT_REJECT_POSTBURN_PERIAPSIS;
    if(!isfinite(peri)||peri>=p->atmosphere_depth||peri<=-p->radius)return false;
    out->burn_start=burn_start;
    out->normal_angle=normal_angle;
    out->burn_ut=coast.ut+duration*.5;
    out->dv=dv;out->duration=duration;out->peri=peri;
    if(reject)*reject=DEORBIT_REJECT_ENTRY_PREDICTION;
    if(!deorbit_timing_probe_entry(out,after,p,site,s))return false;
    if(reject)*reject=DEORBIT_REJECT_NONE;
    return true;
}

static bool deorbit_timing_probe_better(const DeorbitTimingProbe*a,
        const DeorbitTimingProbe*b,bool hasb,const GuidanceSettings*s){
    if(!a||!a->valid)return false;
    if(!hasb||!b||!b->valid)return true;
    if(a->closing!=b->closing)return a->closing;
    bool a_capture=a->entry_miss<=s->target_deorbit_capture_radius;
    bool b_capture=b->entry_miss<=s->target_deorbit_capture_radius;
    if(a_capture!=b_capture)return a_capture;
    /* Inside one ground-track pass the entry range varies continuously with burn time, so the earliest
       in-radius burn only lands at the near edge of the capture radius, which is an energy error of up
       to the radius itself.  Within a pass prefer the smaller miss; across passes keep the earlier one. */
    if(a_capture&&a->burn_start!=b->burn_start&&fabs(a->burn_start-b->burn_start)>1200.0)
        return a->burn_start<b->burn_start;
    if(a->entry_miss<b->entry_miss)return true;
    if(a->entry_miss>b->entry_miss)return false;
    if(a->entry_crossrange<b->entry_crossrange)return true;
    if(a->entry_crossrange>b->entry_crossrange)return false;
    double af=fabs(a->entry_fpa-s->target_entry_flight_path_angle);
    double bf=fabs(b->entry_fpa-s->target_entry_flight_path_angle);
    if(af<bf)return true;
    if(af>bf)return false;
    if(a->entry_course_error<b->entry_course_error)return true;
    if(a->entry_course_error>b->entry_course_error)return false;
    return a->burn_start<b->burn_start;
}

static double deorbit_search_horizon(VehicleState state,double period,const PlanetModel*p,const GuidanceSettings*s){
    double search_horizon=period;
    Vector3 angular_momentum=vcross(state.position,state.velocity);
    double angular_momentum_magnitude=vmag(angular_momentum);
    if(isfinite(angular_momentum_magnitude)&&
       angular_momentum_magnitude>DBL_EPSILON&&
       isfinite(p->rotational_speed)){
        Vector3 orbit_normal=vscale(angular_momentum,
            1.0/angular_momentum_magnitude);
        Vector3 north=vnorm(p->north_axis,v3(0.0,0.0,1.0));
        double orbital_rate=2.0*acos(-1.0)/period;
        double ground_relative_rate=orbital_rate-
            p->rotational_speed*vdot(orbit_normal,north);
        if(isfinite(ground_relative_rate)&&
           fabs(ground_relative_rate)>DBL_EPSILON*fmax(1.0,orbital_rate)){
            double target_phase_period=2.0*acos(-1.0)/
                fabs(ground_relative_rate);
            if(isfinite(target_phase_period)&&target_phase_period>0.0)
                search_horizon=fmax(period,target_phase_period);
        }

        /* Longer waits can matter when plane or apsis geometry varies by
           more than the configured capture radius. A circular equatorial
           orbit has neither reason to search a whole planetary day. */
        double radius=vmag(state.position);
        double energy=.5*vdot(state.velocity,state.velocity)-p->gravitational_parameter/fmax(radius,DBL_MIN);
        Vector3 eccentricity=vsub(vscale(vcross(state.velocity,angular_momentum),1/p->gravitational_parameter),vnorm(state.position,v3(1,0,0)));
        double semimajor=energy<0?-p->gravitational_parameter/(2*energy):INFINITY;
        double apsis_span=2*semimajor*vmag(eccentricity);
        if(fabs(p->rotational_speed)>DBL_EPSILON &&
           (p->radius*vmag(vcross(orbit_normal,north))>s->target_deorbit_capture_radius||
            apsis_span>s->target_deorbit_capture_radius)){
            double rotation_period=2.0*acos(-1.0)/fabs(p->rotational_speed);
            if(isfinite(rotation_period)&&rotation_period>0.0)
                search_horizon=fmax(search_horizon,rotation_period);
        }
    }
    return search_horizon;
}

#include <time.h>
/* Per-stage CPU timing for planner stalls; set KSP_LANDER_TRACE_PLAN_TIMING. */
#define PLAN_TIMING(stage) do{ if(getenv("KSP_LANDER_TRACE_PLAN_TIMING")) \
    fprintf(stderr,"PLAN_TIMING stage=%s cpu=%.3fs\n",(stage),(double)clock()/CLOCKS_PER_SEC); }while(0)

bool deorbit_plan_create_with_terminal_model(DeorbitPlan*out,VehicleState state,
        double period,
        const PlanetModel*p,AerodynamicModel aero,
        const AerodynamicEnvelope*env,const TrajectoryCalibrationModel*cal,
        const LandingSite*site,const VehicleProfile*v,
        const GuidanceSettings*s,double thrust,
        const Telemetry*authority_source,const TerminalModel*terminal_model){
    if(!out||!p||!env||!cal||!site||!v||!s||
       !isfinite(period)||!(period>0.0)||!(thrust>0.0))
        return false;

    /* A loaded plant table with the default ShuttleSim planet frame is not
       a valid model of an actual KSP polar orbital state. Never use it to
       certify an orbit-to-runway route before the frames agree. */
    bool native_terminal=terminal_model_policy_matches(
        terminal_model,site,v,s)&&
        terminal_model_planet_matches(terminal_model,p);
    const TerminalModel *entry_model=native_terminal&&s->taem_energy_law&&
        !strcmp(v->model_id,"STS")?terminal_model:NULL;
    memset(out,0,sizeof(*out));
    trajectory_init(&out->trajectory);
    double sts_handoff_target=entry_model&&(s->sts_trajectory_delivery||s->taem_hac_law)?
        deorbit_sts_handoff_target(entry_model,p,site,v,s,state.mass):NAN;
    if(getenv("KSP_LANDER_TRACE_DEORBIT_SEARCH"))
        fprintf(stderr,"DEORBIT_STS_HANDOFF_TARGET %.1fkm\n",sts_handoff_target/1000.0);

    double lead=fmax(0.0,s->minimum_planning_lead_time);
    double search_horizon=deorbit_search_horizon(state,period,p,s);
    /* One-degree body-rotation spacing is only the coarse numerical grid.
       The cheap orbital probe is refined below to sub-second burn timing. */
    const int time_samples=361;
    double coarse_step=search_horizon/(double)(time_samples-1);

    unsigned rejection_counts[DEORBIT_REJECT_COUNT]={0};

    /*
     * Solve orbital ground-track geometry first.  This probe includes the real
     * finite retro burn and target-periapsis solve, then propagates ballistically
     * only to Entry Interface.  MM304/TAEM behavior is intentionally absent:
     * large crossrange belongs to deorbit timing, not atmospheric recovery.
     */
    DeorbitTimingProbe timing_best={0};
    bool timing_valid=false;
    for(int i=0;i<time_samples;i++){
        double offset=lead+search_horizon*(double)i/(double)(time_samples-1);
        DeorbitTimingProbe probe={0};
        DeorbitRejectReason rejection=DEORBIT_REJECT_NONE;
        if(!deorbit_timing_probe_start(&probe,state,state.ut+offset,0.0,p,cal,
                site,s,thrust,&rejection)){
            if(rejection>DEORBIT_REJECT_NONE&&rejection<DEORBIT_REJECT_COUNT)
                rejection_counts[rejection]++;
            continue;
        }
        if(getenv("KSP_LANDER_TRACE_DEORBIT_CANDIDATES"))
            fprintf(stderr,
                "DEORBIT_TIMING sample=%d start=%.2f burn=%.2f dv=%.2f entry=%.1fkm cross=%.1fkm miss=%.1fkm courseErr=%.2f FPA=%.2f closing=%d\n",
                i,probe.burn_start,probe.burn_ut,probe.dv,
                probe.entry_range/1000.0,probe.entry_crossrange/1000.0,
                probe.entry_miss/1000.0,probe.entry_course_error,
                probe.entry_fpa,probe.closing?1:0);
        if(deorbit_timing_probe_better(&probe,&timing_best,timing_valid,s)){
            timing_best=probe;
            timing_valid=true;
        }
    }
    if(!timing_valid){
        if(getenv("KSP_LANDER_TRACE_DEORBIT_SEARCH"))
            fprintf(stderr,
                "Deorbit timing search rejected all burn starts: start=%u target-input=%u target-orbit=%u target-low-state=%u target-impulse-bound=%u target-impulse-root=%u target-finite-seed=%u target-finite-bracket=%u target-finite-root=%u impulsive-periapsis=%u finite-burn=%u postburn-periapsis=%u entry-prediction=%u.\n",
                rejection_counts[DEORBIT_REJECT_START],
                rejection_counts[DEORBIT_REJECT_TARGET_INPUT],
                rejection_counts[DEORBIT_REJECT_TARGET_ORBIT],
                rejection_counts[DEORBIT_REJECT_TARGET_LOW_STATE],
                rejection_counts[DEORBIT_REJECT_TARGET_IMPULSE_BOUND],
                rejection_counts[DEORBIT_REJECT_TARGET_IMPULSE_ROOT],
                rejection_counts[DEORBIT_REJECT_TARGET_FINITE_SEED],
                rejection_counts[DEORBIT_REJECT_TARGET_FINITE_BRACKET],
                rejection_counts[DEORBIT_REJECT_TARGET_FINITE_ROOT],
                rejection_counts[DEORBIT_REJECT_IMPULSIVE_PERIAPSIS],
                rejection_counts[DEORBIT_REJECT_FINITE_BURN],
                rejection_counts[DEORBIT_REJECT_POSTBURN_PERIAPSIS],
                rejection_counts[DEORBIT_REJECT_ENTRY_PREDICTION]);
        return false;
    }

    PLAN_TIMING("coarse-search");
    /* Refine burn time using only orbital geometry.  Delta-v is re-solved from
       the configured periapsis at every probe; downstream guidance cannot move
       the selected ground-track pass. */
    double time_step=coarse_step;
    for(int iteration=0;iteration<9;iteration++){
        DeorbitTimingProbe iteration_best=timing_best;
        for(int direction=-1;direction<=1;direction+=2){
            double start=timing_best.burn_start+direction*time_step;
            if(start<state.ut+lead||start>state.ut+lead+search_horizon)continue;
            DeorbitTimingProbe probe={0};
            if(!deorbit_timing_probe_start(&probe,state,start,0.0,p,cal,site,s,
                    thrust,NULL))
                continue;
            if(deorbit_timing_probe_better(&probe,&iteration_best,true,s))
                iteration_best=probe;
        }
        if(deorbit_timing_probe_better(&iteration_best,&timing_best,true,s))
            timing_best=iteration_best;
        time_step*=.5;
    }

    if(getenv("KSP_LANDER_TRACE_DEORBIT_SEARCH"))
        fprintf(stderr,
            "DEORBIT_TIMING_SELECTED start=%.3f burn=%.3f dv=%.2f entry=%.1fkm cross=%.1fkm miss=%.1fkm courseErr=%.2f FPA=%.2f\n",
            timing_best.burn_start,timing_best.burn_ut,timing_best.dv,
            timing_best.entry_range/1000.0,timing_best.entry_crossrange/1000.0,
            timing_best.entry_miss/1000.0,timing_best.entry_course_error,
            timing_best.entry_fpa);

    PLAN_TIMING("timing-refine");
    /* Only now pay for atmospheric prediction.  It may reject the correctly
       targeted orbital pass, but it is not allowed to retime that pass. */
    /* At the selected nearby pass, spend a lateral component only when the
       finite-burn prediction improves entry geometry. Re-solve delta-v for
       the same periapsis at each angle; do not disguise a plane correction
       as attitude error or ask atmospheric bank to absorb it. */
    double angle_step=45.0; /* equal normal/retro components, then refine */
    for(int iteration=0;iteration<12;iteration++){
        DeorbitTimingProbe improved=timing_best;
        for(int sign=-1;sign<=1;sign+=2){
            double angle=clampd(timing_best.normal_angle+sign*angle_step,-45.0,45.0);
            DeorbitTimingProbe probe={0};
            if(deorbit_timing_probe_start(&probe,state,timing_best.burn_start,angle,p,cal,site,s,thrust,NULL)&&
               probe.closing&&probe.entry_miss<improved.entry_miss)
                improved=probe;
        }
        timing_best=improved;
        angle_step*=.5;
    }
    Candidate selected;
    memset(&selected,0,sizeof(selected));
    trajectory_init(&selected.pred.trajectory);
    if(!deorbit_evaluate_start(&selected,state,timing_best.burn_start,0.0,timing_best.normal_angle,
            p,aero,env,cal,site,v,s,thrust,NULL)){
        candidate_clear(&selected);
        return false;
    }
    PLAN_TIMING("angle-search");
    /* The cheap orbital probe is only a seed. One 100 km geometric capture
       radius corresponds to about 45 s of Entry-Interface travel for this
       shuttle; evaluate two such increments on either side using the actual
       MM304 executive before committing an irreversible deorbit. The scanned
       alternatives stay in the same ground-track pass and solve Pe separately. */
    EntryPrediction selected_inlet=deorbit_executable_policy_witness(
        &selected,p,aero,env,cal,site,v,s,authority_source,entry_model,false,false);
    /* Evaluate nearby safe, closing inlets with the production terminal law.
       Analytic inlet energy/course heuristics do not certify or veto the
       downstream route; the complete native attempt participates in ranking. */
    if(native_terminal&&deorbit_witness_terminal_trial(&selected,&selected_inlet,
            p,site,v,s)){
        EntryPrediction full=deorbit_executable_policy_witness(
            &selected,p,aero,env,cal,site,v,s,authority_source,
            terminal_model,true,false);
        deorbit_record_execution_witness(&selected,&full,site,v,s);
        entry_prediction_clear(&selected_inlet);
        selected_inlet=full;
    }
    unsigned inlet_candidates_evaluated=1;
    /* (burn-start offset, witness inlet range) of every scanned candidate. */
    double scan_dt[16],scan_range[16];int scan_count=0;
    if(isfinite(selected_inlet.taem_distance)&&selected_inlet.reached_taem){
        scan_dt[scan_count]=0.0;scan_range[scan_count++]=selected_inlet.taem_distance;}
    double scan_origin=timing_best.burn_start;
    double inlet_step=s->target_deorbit_capture_radius/
        fmax(1.0,timing_best.entry_speed);
    if(!isfinite(inlet_step)||inlet_step<=0.0)inlet_step=0.0;
    /* A later burn hands over closer.  While the latest scanned handoff is
       still beyond MM304's target, keep stepping later so the refinement
       below has a bracket: live 2026-10-29 the +2 step stopped at 95 km
       against a 44.6 km HAC target and the burn delivered MM305 short. */
    int last_offset=2;
    for(int offset=-2;offset<=last_offset&&inlet_step>0.0;offset++){
        if(offset==0)continue;
        double start=timing_best.burn_start+offset*inlet_step;
        if(start<state.ut+lead||start>state.ut+lead+search_horizon)continue;
        Candidate nearby;
        memset(&nearby,0,sizeof(nearby));
        trajectory_init(&nearby.pred.trajectory);
        if(!deorbit_evaluate_start(&nearby,state,start,0.0,timing_best.normal_angle,
                p,aero,env,cal,site,v,s,thrust,NULL)){
            candidate_clear(&nearby);
            continue;
        }
        EntryPrediction nearby_inlet=deorbit_executable_policy_witness(
            &nearby,p,aero,env,cal,site,v,s,authority_source,entry_model,false,false);
        if(native_terminal&&deorbit_witness_terminal_trial(&nearby,&nearby_inlet,
            p,site,v,s)){
            EntryPrediction full=deorbit_executable_policy_witness(
                &nearby,p,aero,env,cal,site,v,s,authority_source,
                terminal_model,true,false);
            deorbit_record_execution_witness(&nearby,&full,site,v,s);
            entry_prediction_clear(&nearby_inlet);
            nearby_inlet=full;
        }
        inlet_candidates_evaluated++;
        if(scan_count<16&&isfinite(nearby_inlet.taem_distance)&&nearby_inlet.reached_taem){
            scan_dt[scan_count]=offset*inlet_step;scan_range[scan_count++]=nearby_inlet.taem_distance;}
        if(offset==last_offset&&last_offset<8&&isfinite(sts_handoff_target)&&
           nearby_inlet.reached_taem&&isfinite(nearby_inlet.taem_distance)&&
           nearby_inlet.taem_distance>sts_handoff_target)
            last_offset++;
        bool better=deorbit_guided_candidate_better(&nearby,&nearby_inlet,
            &selected,&selected_inlet,p,site,v,s,sts_handoff_target);
        if(getenv("KSP_LANDER_TRACE_DEORBIT_SEARCH"))
            fprintf(stderr,
                "DEORBIT_INLET_CANDIDATE dt=%+.1fs burn=%.1f tier=%d safe=%d range=%.1fkm course=%.1fdeg closing=%d geometry=%d energy=%+.0fJ/kg boundaryMiss=%d chosen=%d\n",
                offset*inlet_step,nearby.burn_ut,nearby.tier,
                deorbit_entry_stage_qualified(&nearby_inlet,v,s,nearby.peri,
                    site->altitude,p->atmosphere_depth,true)?1:0,
                nearby_inlet.taem_distance/1000.0,nearby_inlet.taem_course,
                deorbit_witness_closing(&nearby_inlet,site)?1:0,
                deorbit_witness_inlet_geometry(&nearby_inlet,p,site,s)?1:0,
                nearby_inlet.taem_interface_energy_margin,
                nearby_inlet.taem_ownership_boundary_missed?1:0,better?1:0);
        if(better){
            candidate_clear(&selected);
            entry_prediction_clear(&selected_inlet);
            selected=nearby;
            selected_inlet=nearby_inlet;
        }else{
            candidate_clear(&nearby);
            entry_prediction_clear(&nearby_inlet);
        }
    }
    /* The scan step (~43 s) moves the polar MM305 inlet ~75 km, so nothing
       near the configured inlet range may be sampled (2026-10-13: 132/57/26 km;
       the 57 km pick overshot live once the bay-closed vehicle glided further
       than the learned envelope).  Secant-refine inside the adjacent pair that
       brackets the nominal range; candidates are ranked by the same rule. */
    {
        /* Experimental STS ranks toward MM304's handoff target, so refine
           toward it too; refining toward the configured inlet range found
           92.8 km, which that ranking rejected, leaving the coarse 28 km
           sample that delivered MM305 short (60 deg live, 2026-10-16). */
        double nominal=isfinite(sts_handoff_target)?sts_handoff_target:
            entry_taem_range_target(p,s);
        for(int i=0;i<scan_count;i++)for(int j=i+1;j<scan_count;j++)
            if(scan_dt[j]<scan_dt[i]){double t=scan_dt[i];scan_dt[i]=scan_dt[j];scan_dt[j]=t;
                t=scan_range[i];scan_range[i]=scan_range[j];scan_range[j]=t;}
        int bracket=-1;
        for(int i=0;i+1<scan_count;i++)
            if((scan_range[i]-nominal)*(scan_range[i+1]-nominal)<0.0&&
               scan_dt[i+1]-scan_dt[i]<=1.5*inlet_step){bracket=i;break;}
        double lo_dt=bracket>=0?scan_dt[bracket]:0.0,lo_r=bracket>=0?scan_range[bracket]:0.0;
        double hi_dt=bracket>=0?scan_dt[bracket+1]:0.0,hi_r=bracket>=0?scan_range[bracket+1]:0.0;
        for(int it=0;bracket>=0&&it<3;it++){
            double dt=lo_dt+(nominal-lo_r)*(hi_dt-lo_dt)/(hi_r-lo_r);
            dt=clampd(dt,lo_dt+.1*(hi_dt-lo_dt),hi_dt-.1*(hi_dt-lo_dt));
            double start=scan_origin+dt;
            if(start<state.ut+lead||start>state.ut+lead+search_horizon)break;
            Candidate refined;memset(&refined,0,sizeof(refined));
            trajectory_init(&refined.pred.trajectory);
            if(!deorbit_evaluate_start(&refined,state,start,0.0,timing_best.normal_angle,
                    p,aero,env,cal,site,v,s,thrust,NULL)){candidate_clear(&refined);break;}
            EntryPrediction refined_inlet=deorbit_executable_policy_witness(
                &refined,p,aero,env,cal,site,v,s,authority_source,entry_model,false,false);
            if(native_terminal&&deorbit_witness_terminal_trial(&refined,&refined_inlet,
                    p,site,v,s)){
                EntryPrediction full=deorbit_executable_policy_witness(
                    &refined,p,aero,env,cal,site,v,s,authority_source,
                    terminal_model,true,false);
                deorbit_record_execution_witness(&refined,&full,site,v,s);
                entry_prediction_clear(&refined_inlet);
                refined_inlet=full;
            }
            inlet_candidates_evaluated++;
            double r=refined_inlet.reached_taem?refined_inlet.taem_distance:NAN;
            bool better=deorbit_guided_candidate_better(&refined,&refined_inlet,
                &selected,&selected_inlet,p,site,v,s,sts_handoff_target);
            if(getenv("KSP_LANDER_TRACE_DEORBIT_SEARCH"))
                fprintf(stderr,"DEORBIT_INLET_REFINE dt=%+.1fs burn=%.1f range=%.1fkm geometry=%d chosen=%d\n",
                    dt,refined.burn_ut,r/1000.0,
                    deorbit_witness_inlet_geometry(&refined_inlet,p,site,s)?1:0,better?1:0);
            if(better){
                candidate_clear(&selected);entry_prediction_clear(&selected_inlet);
                selected=refined;selected_inlet=refined_inlet;
            }else{candidate_clear(&refined);entry_prediction_clear(&refined_inlet);}
            if(!isfinite(r))break;
            if((r-nominal)*(lo_r-nominal)>0.0){lo_dt=dt;lo_r=r;}else{hi_dt=dt;hi_r=r;}
            if(fabs(r-nominal)<5000.0)break;
        }
    }
    if(getenv("KSP_LANDER_TRACE_DEORBIT_SEARCH"))
        fprintf(stderr,"DEORBIT_INLET_SELECTED samples=%u burn=%.1f range=%.1fkm energy=%+.0fJ/kg geometry=%d\n",
            inlet_candidates_evaluated,selected.burn_ut,
            selected_inlet.taem_distance/1000.0,
            selected_inlet.taem_interface_energy_margin,
            deorbit_witness_inlet_geometry(&selected_inlet,p,site,s)?1:0);
    entry_prediction_clear(&selected_inlet);
    PLAN_TIMING("inlet-retime");
    /* This *full* witness owns the selected finite-burn state and executes
       native MM304 -> MM305/HAC -> Final against the immutable live-aligned
       TerminalModel, rather than certifying a handoff alone. Without such a
       model it remains an entry-only diagnostic and cannot certify landing. */
    PLAN_TIMING("detailed-witness");
    EntryPrediction detailed=deorbit_executable_policy_witness(
        &selected,p,aero,env,cal,site,v,s,authority_source,
        terminal_model,native_terminal,true);
    deorbit_record_execution_witness(&selected,&detailed,site,v,s);
    bool nominal_capture=native_terminal&&selected.executable_witness_capture&&
        deorbit_energy_margin_qualified(&selected.energy_margin,s,true);
    /* Apply the same native downstream route proof to the burn and aero
       stress cases *if* the nominal candidate survives. Invalid/uncertified
       nominal approaches retain reduced entry-only stress diagnostics. */
    RobustAssessment selected_robust=assess_candidate_robustness(
        &selected,state,p,aero,env,cal,site,v,s,thrust,authority_source,
        terminal_model,nominal_capture,false);
    PLAN_TIMING("robustness");

    if(getenv("KSP_LANDER_TRACE_STRESS")){
        RobustAssessment traced=assess_candidate_robustness(
            &selected,state,p,aero,env,cal,site,v,s,thrust,authority_source,
            terminal_model,nominal_capture,true);
        (void)traced;
    }
    selected_robust.qualified=selected.executable_witness_capture&&
        selected_robust.pass_fraction+1e-9>=s->deorbit_robust_minimum_pass_fraction&&
        selected_robust.unsafe==0;
    selected_robust.recovery_qualified=selected.executable_witness_capture&&
        deorbit_energy_margin_qualified(&selected.energy_margin,s,false)&&
        selected_robust.recovery_pass_fraction+1e-9>=
            s->deorbit_robust_minimum_pass_fraction&&selected_robust.unsafe==0;

    out->created_ut=state.ut;
    out->burn_ut=selected.burn_ut;
    out->delta_v=selected.dv;
    out->burn_normal_angle=selected.burn_normal_angle;
    out->estimated_burn_duration=selected.duration;
    out->planning_mass=state.mass;
    out->planning_available_thrust=thrust;
    out->predicted_taem_distance=detailed.taem_distance;
    out->predicted_taem_range_error=detailed.taem_range_error;
    out->predicted_closest_distance=detailed.closest_distance;
    out->predicted_entry_range=detailed.entry_range;
    out->predicted_entry_flight_path_angle=detailed.entry_flight_path_angle;
    out->predicted_post_burn_periapsis_altitude=selected.peri;
    out->terminal_model_snapshot_id=native_terminal?terminal_model->snapshot_id:0;
    out->terminal_replay_evaluated=native_terminal&&detailed.shadow_guidance_used;
    out->terminal_inlet_qualified=out->terminal_replay_evaluated&&
        detailed.reached_taem&&detailed.taem_dynamic_interface_captured&&
        !detailed.taem_ownership_boundary_missed;
    out->terminal_native_path_committed=out->terminal_replay_evaluated&&
        detailed.shadow_terminal_path_committed;
    out->terminal_final_approach_captured=out->terminal_replay_evaluated&&
        detailed.shadow_final_approach_captured;
    out->terminal_guidance_aborted=out->terminal_replay_evaluated&&
        detailed.shadow_aborted;
    out->terminal_inlet_energy_margin=out->terminal_replay_evaluated?
        detailed.taem_interface_energy_margin:NAN;
    GeoPoint terminal_end=predictor_geo_point(detailed.final_state.position,p,
        detailed.final_state.ut);
    GeoPoint runway_geo={site->latitude,site->longitude,site->altitude};
    double terminal_end_range=great_circle_distance(terminal_end,runway_geo,p->radius);
    if(getenv("KSP_LANDER_TRACE_DEORBIT_SEARCH"))
        fprintf(stderr,
            "DEORBIT_NATIVE_TERMINAL snapshot=%llu replay=%d inlet=%d committed=%d final=%d aborted=%d energy=%+.0f J/kg strict=%d endUT=%.1f endH=%.0f endSpeed=%.1f endRange=%.1fkm\n",
            (unsigned long long)out->terminal_model_snapshot_id,
            out->terminal_replay_evaluated?1:0,out->terminal_inlet_qualified?1:0,
            out->terminal_native_path_committed?1:0,
            out->terminal_final_approach_captured?1:0,
            out->terminal_guidance_aborted?1:0,
            out->terminal_inlet_energy_margin,nominal_capture?1:0,
            detailed.final_state.ut,terminal_end.altitude,
            vmag(detailed.final_state.velocity),terminal_end_range/1000.0);
    out->nominal_capture_achieved=nominal_capture;
    out->robustness_qualified=selected_robust.qualified&&nominal_capture;
    out->target_capture_achieved=out->robustness_qualified;
    out->robustness_scenarios=selected_robust.scenarios;
    out->robustness_passed=selected_robust.passed;
    out->robustness_unsafe=selected_robust.unsafe;
    out->robustness_pass_fraction=selected_robust.pass_fraction;
    out->recovery_passed=selected_robust.recovery_passed;
    out->recovery_pass_fraction=selected_robust.recovery_pass_fraction;
    out->entry_stage_passed=selected_robust.entry_stage_passed;
    out->entry_stage_scenarios=selected_robust.scenarios;
    out->entry_stage_pass_fraction=selected_robust.entry_stage_pass_fraction;
    bool nominal_entry_stage=deorbit_entry_stage_qualified(&detailed,v,s,
        selected.peri,site->altitude,p->atmosphere_depth,true);
    out->entry_stage_qualified=nominal_entry_stage&&
        selected_robust.entry_stage_qualified;
    /* Orbital-stage admission certifies the finite burn and safe atmospheric
       entry envelope. MM304/TAEM shadow capture remains separately represented
       by target_capture_achieved and is advisory at this stage boundary. */
    out->execution_qualified=out->entry_stage_qualified;
    out->execution_degraded=out->execution_qualified&&
        !out->target_capture_achieved;
    out->worst_case_closest_distance=selected_robust.worst_closest;
    out->worst_case_taem_range_error=selected_robust.worst_taem_error;
    out->worst_case_entry_flight_path_angle=selected_robust.worst_entry_angle;
    out->worst_case_post_burn_periapsis_altitude=selected_robust.min_periapsis;
    out->worst_case_peak_dynamic_pressure=selected_robust.max_dynamic_pressure;
    out->worst_case_peak_g_load=selected_robust.max_g_load;

    double model_confidence=0.0;
    for(int i=0;i<4;i++)model_confidence+=env->regimes[i].confidence;
    model_confidence/=4.0;
    double evidence_confidence=fmin(model_confidence,
        fmin(cal->confidence,selected_robust.pass_fraction));
    if(selected_robust.unsafe||!nominal_entry_stage)evidence_confidence=0.0;
    out->confidence=clampd(evidence_confidence,0.0,1.0);

    out->trajectory=detailed.trajectory;
    double selected_align_pos=deorbit_alignment_position_error(&detailed,s);
    double selected_align_course=deorbit_alignment_course_error(&detailed,site);
    snprintf(out->note,sizeof(out->note),
        "Entry %.0fkm FPA %.2fdeg, impact %+.1f/%+.1fkm, alignment %.1fkm/%.1fdeg at %.1fkm/%.0fmps, inlet %.1fkm margin %+.0fkJ/kg, Pe %.1fkm; %u guided candidates. Strict %u/%u (%.0f%%), recovery %u/%u (%.0f%%)%s%s",
        detailed.entry_range/1000,detailed.entry_flight_path_angle,
        selected.energy_margin.impact_along_track/1000.0,
        selected.energy_margin.impact_cross_track/1000.0,
        selected_align_pos/1000,selected_align_course,
        detailed.taem_altitude/1000,detailed.taem_speed,
        detailed.taem_distance/1000,detailed.taem_interface_energy_margin/1000,
        selected.peri/1000,inlet_candidates_evaluated,
        selected_robust.passed,selected_robust.scenarios,
        selected_robust.pass_fraction*100,
        selected_robust.recovery_passed,selected_robust.scenarios,
        selected_robust.recovery_pass_fraction*100,
        selected_robust.unsafe?"; unsafe stress case detected":"",
        out->target_capture_achieved?".":out->execution_qualified?
            "; entry-stage qualified; downstream MM304/TAEM shadow capture not achieved.":
            "; entry-stage execution acceptance not met.");

    if(selected.pred.mm304_gate_recorded&&detailed.mm304_gate_recorded){
        fprintf(stderr,
            "Deorbit 50km policy witness: reduced along/cross %+.1f/%+.1fkm V %.1f; executable %+.1f/%+.1fkm V %.1f; delta %+.1f/%+.1fkm dV %+.1f.\n",
            selected.pred.mm304_gate_along_track/1000.0,selected.pred.mm304_gate_cross_track/1000.0,
            selected.pred.mm304_gate_speed,detailed.mm304_gate_along_track/1000.0,
            detailed.mm304_gate_cross_track/1000.0,detailed.mm304_gate_speed,
            (detailed.mm304_gate_along_track-selected.pred.mm304_gate_along_track)/1000.0,
            (detailed.mm304_gate_cross_track-selected.pred.mm304_gate_cross_track)/1000.0,
            detailed.mm304_gate_speed-selected.pred.mm304_gate_speed);
    }else{
        fprintf(stderr,"Deorbit 50km policy witness incomplete: reduced=%d executable=%d shadow=%d abort=%d.\n",
            selected.pred.mm304_gate_recorded?1:0,detailed.mm304_gate_recorded?1:0,
            detailed.shadow_guidance_used?1:0,detailed.shadow_aborted?1:0);
    }

    fprintf(stderr,
        "Deorbit selected: tier %d burn %.1f dv %.1f Pe %.1fkm entry %.1fkm FPA %.2fdeg marginImpact %+.1f/%+.1fkm align %.1fkm courseDebt %.1fdeg h %.1fkm V %.0f miss %.1fkm TAEMrange %.1fkm rangeError %+.1fkm capture=%d shadow=%d entered=%d gate=%d taem=%d abort=%d boundaryMiss=%d minEntryV=%.0f safeV=%.0f energyValid=%d nominal %d robust %u/%u recovery %u/%u exec %d.\n",
        selected.tier,selected.burn_ut,selected.dv,selected.peri/1000.0,
        detailed.entry_range/1000.0,detailed.entry_flight_path_angle,
        selected.energy_margin.impact_along_track/1000.0,
        selected.energy_margin.impact_cross_track/1000.0,
        selected_align_pos/1000.0,selected_align_course,
        detailed.taem_altitude/1000.0,detailed.taem_speed,
        detailed.closest_distance/1000.0,detailed.taem_distance/1000.0,
        detailed.taem_range_error/1000.0,
        deorbit_runway_capture_qualified(&detailed,site,v,s,selected.peri)?1:0,
        detailed.shadow_guidance_used?1:0,detailed.entered_atmosphere?1:0,
        detailed.mm304_gate_recorded?1:0,detailed.reached_taem?1:0,
        detailed.shadow_aborted?1:0,detailed.taem_ownership_boundary_missed?1:0,
        detailed.minimum_entry_speed,v->minimum_safe_speed,
        selected.energy_margin.valid?1:0,
        nominal_capture?1:0,selected_robust.passed,
        selected_robust.scenarios,selected_robust.recovery_passed,
        selected_robust.scenarios,out->execution_qualified?1:0);

    PLAN_TIMING("done");
    candidate_clear(&selected);
    return true;
}

bool deorbit_plan_create_with_telemetry(DeorbitPlan*out,VehicleState state,
        double period,const PlanetModel*p,AerodynamicModel aero,
        const AerodynamicEnvelope*env,const TrajectoryCalibrationModel*cal,
        const LandingSite*site,const VehicleProfile*v,
        const GuidanceSettings*s,double thrust,const Telemetry*authority_source){
    return deorbit_plan_create_with_terminal_model(out,state,period,p,aero,
        env,cal,site,v,s,thrust,authority_source,NULL);
}

bool deorbit_plan_create(DeorbitPlan*out,VehicleState state,double period,
        const PlanetModel*p,AerodynamicModel aero,
        const AerodynamicEnvelope*env,const TrajectoryCalibrationModel*cal,
        const LandingSite*site,const VehicleProfile*v,
        const GuidanceSettings*s,double thrust){
    return deorbit_plan_create_with_telemetry(out,state,period,p,aero,env,cal,
        site,v,s,thrust,NULL);
}
