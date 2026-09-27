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
    Vector3 up=vnorm(final.position,v3(0,1,0));
    Vector3 surface_velocity=vsub(final.velocity,vcross(planet_rotation_vector(p),final.position));
    double vertical=vdot(surface_velocity,up);
    GeoPoint site_geo={site->latitude,site->longitude,site->altitude};
    double along=0.0,cross=0.0;
    runway_coordinates(geo,site_geo,site->runway_heading,p->radius,&along,&cross);
    m.valid=passive.entered_atmosphere&&isfinite(geo.altitude)&&isfinite(along)&&isfinite(cross)&&
        isfinite(vertical)&&geo.altitude<=site->altitude+750.0&&vertical<0.0;
    m.impact_along_track=along;m.impact_cross_track=cross;
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

static bool impulsive_deorbit_periapsis_plausible(VehicleState coast,double dv,
        const PlanetModel*p,const GuidanceSettings*s){
    (void)s;
    VehicleState impulse=coast;
    Vector3 direction=burn_direction(coast.position,coast.velocity,0,0);
    impulse.velocity=vadd(impulse.velocity,vscale(direction,dv));
    double peri=predictor_postburn_periapsis(impulse,p);
    return isfinite(peri)&&peri<p->atmosphere_depth&&peri>-p->radius;
}

static bool eval_candidate(Candidate*out,VehicleState coast,double dv,
        const PlanetModel*p,AerodynamicModel aero,const AerodynamicEnvelope*env,
        const TrajectoryCalibrationModel*cal,const LandingSite*site,
        const VehicleProfile*v,const GuidanceSettings*s,double thrust,
        DeorbitRejectReason*reject){
    memset(out,0,sizeof(*out));
    if(reject)*reject=DEORBIT_REJECT_IMPULSIVE_PERIAPSIS;
    if(!impulsive_deorbit_periapsis_plausible(coast,dv,p,s))return false;

    double dur=0.0,delivered=0.0;
    VehicleState after=retro_burn_scenario(coast,dv,thrust,
        s->deorbit_maximum_throttle,s->deorbit_throttle_ramp_duration,
        p,0,0,&dur,&delivered,cal?cal->physics:NULL);
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

static RobustAssessment assess_candidate_robustness(const Candidate*c,VehicleState initial,const PlanetModel*p,AerodynamicModel aero,const AerodynamicEnvelope*env,const TrajectoryCalibrationModel*cal,const LandingSite*site,const VehicleProfile*v,const GuidanceSettings*s,double thrust,bool trace){
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
        VehicleState after=retro_burn_scenario(coast,scenario_dv,thrust*fmax(.05,sc.thrust_scale),s->deorbit_maximum_throttle,s->deorbit_throttle_ramp_duration,p,sc.radial_error,sc.normal_error,&duration,&delivered,cal?cal->physics:NULL);
        bool burn_completed=isfinite(duration)&&isfinite(delivered)&&
            delivered>=scenario_dv-.25;
        double peri=predictor_postburn_periapsis(after,p);
        TrajectoryCalibrationModel scenario_cal=*cal;
        scenario_cal.density_scale=clampd(cal->density_scale*(1+sc.atmosphere_fraction),.35,2.8);
        scenario_cal.stress_drag_scale=sc.aero_drag_scale>0?sc.aero_drag_scale:1;
        scenario_cal.stress_lift_scale=sc.aero_lift_scale>0?sc.aero_lift_scale:1;
        EntryPrediction pr=predictor_simulate_entry(after,p,aero,env,&scenario_cal,v,site,s,v->maximum_bank_angle,0,0,0,4500,false);
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


static double deorbit_finite_burn_periapsis(VehicleState coast,double dv,
        const PlanetModel*p,const GuidanceSettings*s,double thrust,
        const TrajectoryCalibrationModel*cal){
    double duration=0.0,delivered=0.0;
    VehicleState after=retro_burn_scenario(coast,dv,thrust,
        s->deorbit_maximum_throttle,s->deorbit_throttle_ramp_duration,
        p,0,0,&duration,&delivered,cal?cal->physics:NULL);
    double tolerance=fmax(GUIDANCE_BURN_TERMINAL_DV_TOLERANCE_MPS,
        sqrt(DBL_EPSILON)*fmax(1.0,fabs(dv)));
    if(!isfinite(duration)||!isfinite(delivered)||
       delivered+tolerance<dv)
        return NAN;
    return predictor_postburn_periapsis(after,p);
}

static bool deorbit_solve_target_periapsis(VehicleState coast,
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
        coast,low,p,s,thrust,cal);
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
    Vector3 direction=burn_direction(coast.position,coast.velocity,0.0,0.0);
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
        coast,high,p,s,thrust,cal);
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
            coast,high,p,s,thrust,cal);
        if(!isfinite(high_peri)){
            if(reject)*reject=DEORBIT_REJECT_TARGET_FINITE_BRACKET;
            return false;
        }
    }

    /* Numerical bisection only; the decision target is configured periapsis. */
    for(int i=0;i<48;i++){
        double mid=.5*(low+high);
        double peri=deorbit_finite_burn_periapsis(
            coast,mid,p,s,thrust,cal);
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
        double burn_start,double dv_offset,const PlanetModel*p,
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
            coast,p,s,thrust,cal,&dv,reject))
        return false;
    dv=fmax(0.0,dv+dv_offset);
    return eval_candidate(out,coast,dv,p,aero,env,cal,site,v,s,thrust,reject);
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
        const Telemetry*authority_source,bool include_trajectory){
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

    EntryPrediction witness=predictor_simulate_entry_guidance_shadow(c->after,&seed,&guidance,
        &executed,p,aero,env,cal,&cfg,4500.0,include_trajectory);
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

typedef struct {
    bool valid,closing;
    double burn_start,burn_ut,dv,duration,peri;
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
                range-s->target_entry_range);
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
        double burn_start,const PlanetModel*p,const TrajectoryCalibrationModel*cal,
        const LandingSite*site,const GuidanceSettings*s,double thrust,
        DeorbitRejectReason*reject){
    if(reject)*reject=DEORBIT_REJECT_START;
    if(!out||!isfinite(burn_start)||burn_start<initial.ut)return false;
    memset(out,0,sizeof(*out));
    VehicleState coast=predictor_propagate_vacuum(initial,burn_start,p,12.0);
    double dv=0.0;
    if(!deorbit_solve_target_periapsis(coast,p,s,thrust,cal,&dv,reject))return false;
    double duration=0.0,delivered=0.0;
    VehicleState after=retro_burn_scenario(coast,dv,thrust,
        s->deorbit_maximum_throttle,s->deorbit_throttle_ramp_duration,
        p,0.0,0.0,&duration,&delivered,cal?cal->physics:NULL);
    double tolerance=fmax(GUIDANCE_BURN_TERMINAL_DV_TOLERANCE_MPS,
        sqrt(DBL_EPSILON)*fmax(1.0,fabs(dv)));
    if(reject)*reject=DEORBIT_REJECT_FINITE_BURN;
    if(!isfinite(duration)||!isfinite(delivered)||delivered+tolerance<dv)return false;
    double peri=predictor_postburn_periapsis(after,p);
    if(reject)*reject=DEORBIT_REJECT_POSTBURN_PERIAPSIS;
    if(!isfinite(peri)||peri>=p->atmosphere_depth||peri<=-p->radius)return false;
    out->burn_start=burn_start;
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

bool deorbit_plan_create_with_telemetry(DeorbitPlan*out,VehicleState state,
        double period,
        const PlanetModel*p,AerodynamicModel aero,
        const AerodynamicEnvelope*env,const TrajectoryCalibrationModel*cal,
        const LandingSite*site,const VehicleProfile*v,
        const GuidanceSettings*s,double thrust,
        const Telemetry*authority_source){
    if(!out||!p||!env||!cal||!site||!v||!s||
       !isfinite(period)||!(period>0.0)||!(thrust>0.0))
        return false;

    memset(out,0,sizeof(*out));
    trajectory_init(&out->trajectory);

    double lead=fmax(0.0,s->minimum_planning_lead_time);
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

        /* Inclined inertial planes do not repeat their body-fixed node longitude
           after one spacecraft orbit.  Let the rotating runway sweep once under
           the plane so deorbit timing, rather than atmospheric bank, owns the
           large-scale crossrange. */
        if(fabs(p->rotational_speed)>DBL_EPSILON){
            double rotation_period=2.0*acos(-1.0)/fabs(p->rotational_speed);
            if(isfinite(rotation_period)&&rotation_period>0.0)
                search_horizon=fmax(search_horizon,rotation_period);
        }
    }
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
        if(!deorbit_timing_probe_start(&probe,state,state.ut+offset,p,cal,
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
            if(!deorbit_timing_probe_start(&probe,state,start,p,cal,site,s,
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

    /* Only now pay for atmospheric prediction.  It may reject the correctly
       targeted orbital pass, but it is not allowed to retime that pass. */
    Candidate selected;
    memset(&selected,0,sizeof(selected));
    trajectory_init(&selected.pred.trajectory);
    if(!deorbit_evaluate_start(&selected,state,timing_best.burn_start,0.0,
            p,aero,env,cal,site,v,s,thrust,NULL)){
        candidate_clear(&selected);
        return false;
    }
    RobustAssessment selected_robust=assess_candidate_robustness(
        &selected,state,p,aero,env,cal,site,v,s,thrust,false);

    if(getenv("KSP_LANDER_TRACE_STRESS")){
        RobustAssessment traced=assess_candidate_robustness(
            &selected,state,p,aero,env,cal,site,v,s,thrust,true);
        (void)traced;
    }

    /* Atmospheric guidance certifies the orbital solution; it does not
       retime the burn.  If MM304 cannot accept this correctly targeted pass,
       that is an Entry problem rather than permission to move the ground track. */
    EntryPrediction detailed=deorbit_executable_policy_witness(
        &selected,p,aero,env,cal,site,v,s,authority_source,true);
    deorbit_record_execution_witness(&selected,&detailed,site,v,s);
    bool nominal_capture=selected.executable_witness_capture&&
        deorbit_energy_margin_qualified(&selected.energy_margin,s,true);
    /* Stress cases are evaluated before the expensive production-guidance
       witness. Join their unchanged results with that witness now, instead of
       leaving the base-capture prerequisite unset on the selected candidate. */
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
    out->estimated_burn_duration=selected.duration;
    out->planning_mass=state.mass;
    out->planning_available_thrust=thrust;
    out->predicted_taem_distance=detailed.taem_distance;
    out->predicted_taem_range_error=detailed.taem_range_error;
    out->predicted_closest_distance=detailed.closest_distance;
    out->predicted_entry_range=detailed.entry_range;
    out->predicted_entry_flight_path_angle=detailed.entry_flight_path_angle;
    out->predicted_post_burn_periapsis_altitude=selected.peri;
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
        "Nominal entry %.0f km at %.2f°, wings-level impact %+.1f/%+.1f km along/cross, alignment %.1f km / %.1f° at %.1f km / %.0f m/s, Pe %.1f km. Strict %u/%u (%.0f%%), recovery %u/%u (%.0f%%)%s%s",
        detailed.entry_range/1000,detailed.entry_flight_path_angle,
        selected.energy_margin.impact_along_track/1000.0,
        selected.energy_margin.impact_cross_track/1000.0,
        selected_align_pos/1000,selected_align_course,
        detailed.taem_altitude/1000,detailed.taem_speed,selected.peri/1000,
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

    candidate_clear(&selected);
    return true;
}

bool deorbit_plan_create(DeorbitPlan*out,VehicleState state,double period,
        const PlanetModel*p,AerodynamicModel aero,
        const AerodynamicEnvelope*env,const TrajectoryCalibrationModel*cal,
        const LandingSite*site,const VehicleProfile*v,
        const GuidanceSettings*s,double thrust){
    return deorbit_plan_create_with_telemetry(out,state,period,p,aero,env,cal,
        site,v,s,thrust,NULL);
}
