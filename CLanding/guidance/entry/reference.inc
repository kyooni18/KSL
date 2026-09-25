static EntryAlphaPhase entry_alpha_phase_for_exec(EntryPhase phase){
    switch(phase){
        case ENTRY_PHASE_PREENTRY:return ENTRY_ALPHA_PRE_ENTRY;
        case ENTRY_PHASE_TEMPERATURE_CONTROL:return ENTRY_ALPHA_TEMPERATURE_CONTROL;
        case ENTRY_PHASE_EQUILIBRIUM_GLIDE:return ENTRY_ALPHA_EQUILIBRIUM_GLIDE;
        case ENTRY_PHASE_CONSTANT_DRAG:return ENTRY_ALPHA_CONSTANT_DRAG;
        case ENTRY_PHASE_TRANSITION:return ENTRY_ALPHA_TRANSITION;
        default:return ENTRY_ALPHA_PRE_ENTRY;
    }
}

/* MM304 still owns the vertical state before TAEM. DRAGREF can make its
   longitudinal range residual small while the vehicle remains physically too high.
   Preserve the Shuttle-derived speed/range altitude corridor as an independent
   state objective: when the vehicle is high and shallow with useful aerodynamic
   authority, spend vertical lift with bank while preserving belly-first Entry AoA.
   Local q/g/stall and dynamic-bank protections remain authoritative. */


static double entry_program_taem_path_fpa(const GuidanceMachine*g,const Telemetry*t,
        const LandingConfiguration*cfg,double*distance_out){
    if(distance_out)*distance_out=NAN;
    if(!g||!t||!cfg||!g->taem_interface_target.valid||
       !isfinite(t->runway_along_track)||!isfinite(t->runway_cross_track))return NAN;
    const TaemInterfaceTarget*q=&g->taem_interface_target;
    double da=q->along_track-t->runway_along_track;
    double dc=q->cross_track-t->runway_cross_track;
    double distance=hypot(da,dc);
    if(distance_out)*distance_out=distance;
    if(!isfinite(distance)||distance<=DBL_MIN)return q->flight_path_angle;

    /* The vertical target is geometric outside the response/acquisition distance.
       Inside that physically reserved distance, smoothly converge to the exact
       MM304 inlet FPA. The length scale is therefore predicted acquisition lead
       plus measured response travel, not a tuned kilometre window. */
    double los=atan2(q->altitude-t->mean_altitude,distance)*RAD2DEG;
    double response_distance=fmax(0.0,q->speed)*fmax(0.0,q->response_time);
    double lead=fmax(q->acquisition_lead,response_distance);
    if(!(lead>DBL_MIN)||!isfinite(lead))return los;
    double blend=1.0-clampd(distance/lead,0.0,1.0);
    blend=blend*blend*(3.0-2.0*blend); /* decision-literal-ok: cubic smoothstep basis */
    return los+blend*(q->flight_path_angle-los);
}

/* Defined with the terminal path-provider helpers below.  MM304 uses the same
   distance-domain atmosphere/response integration as MM305 when it prices a
   candidate incidence; it must not replace that path integral with a current
   drag value multiplied by the whole future route. */
                                                                                     
                                                                         
                                                                      
                          
                                                                         
                                                                    
                                                                          
                                                                 
                                 
                                                                                
                                                                            
                                                              

static double entry_program_final_s_turn_drag_aoa(const GuidanceMachine*g,
        const Telemetry*t,const PlanetModel*p,AerodynamicModel aero,
        const LandingConfiguration*cfg,const EntryDragReferenceOutput*drag,
        bool final_s_turn_active){
    if(!g||!t||!p||!cfg||!drag||!drag->valid||!final_s_turn_active||
       !g->taem_interface_target.valid)return NAN;

    const VehicleProfile*v=&cfg->vehicle;
    ControlAuthorityEnvelope authority=
        decision_control_authority_envelope(g,t,p,cfg);
    if(!authority.valid||!authority.survivable)return NAN;

    double path_distance=NAN;
    double path_fpa=entry_program_taem_path_fpa(g,t,cfg,&path_distance);
    if(!isfinite(path_fpa)||!isfinite(path_distance)||path_distance<=DBL_MIN||
       !isfinite(t->flight_path_angle))return NAN;

    /* If the vehicle is already steeper than the geometric delivery path,
       additional drag would spend energy needed to recover that vertical debt. */
    if(t->flight_path_angle<path_fpa)return NAN;

    const TaemInterfaceTarget*q=&g->taem_interface_target;
    double available=entry_remaining_specific_energy(t->latitude,t->mean_altitude,
        t->true_air_speed,cfg->site.latitude,q->altitude,q->speed,p);
    if(!isfinite(available)||available<=0.0)return NAN;
    double speed_excess=t->true_air_speed-q->speed;
    double speed_excess_resolution=fmax(25.0,fabs(q->speed)*0.10);
    if(!isfinite(speed_excess)||speed_excess<=speed_excess_resolution)return NAN;

    double slope=-path_fpa;
    if(!(slope>0.0)||!(slope<90.0)||!isfinite(slope))return NAN;

    /* Price both the present incidence and each candidate over the same
       atmosphere-resolved path.  The old current-drag*air-path approximation
       implicitly held density, speed, and drag coefficient constant all the
       way to the inlet and therefore over-commanded alpha immediately after
       the shaping reversal. */
    double current_aoa=clampd(hypot(t->angle_of_attack,t->sideslip),0.0,
        v->maximum_angle_of_attack);
    double current_work=terminal_projected_drag_work(g,t,p,aero,v,&cfg->guidance,
        current_aoa,path_distance,slope);
    if(!isfinite(current_work)||current_work<0.0)return NAN;
    EnergyPathEnvelope current_energy=decision_energy_path_envelope(
        t,p,available,current_work,path_distance);
    if(!current_energy.valid)return NAN;

    double target_work=fmax(0.0,
        available-current_energy.uncertainty_specific_energy);
    /* The atmosphere-resolved path model is nominal; live Entry energy/range
       excess plus measured drag deficit is stronger evidence that the final
       S-turn must spend additional energy even when the local drag reference
       predicts a nominal undershoot.  Convert that range debt through the
       observed drag deficit so finalaoa-style cases can use the explicitly
       permitted final-S-turn AoA extension. */
    /* Use the same achievable-range margin that owns MM304 longitudinal
       guidance.  Do not mix in Telemetry.energy_excess_range: that field was
       historically produced by a separate terminal-demand approximation and
       can disagree in both magnitude and timing with DRAGREF. */
    double live_range_excess=isfinite(drag->predicted_range_error)?
        fmax(0.0,drag->predicted_range_error):0.0;
    double drag_deficit=isfinite(drag->drag_error_accel)?
        fmax(0.0,drag->drag_error_accel):0.0;
    if(live_range_excess>0.0&&drag_deficit>0.0){
        double evidence_weight=isfinite(drag->confidence)&&drag->confidence>0.0?
            clampd(drag->confidence,0.0,1.0):1.0;
        double live_debt=live_range_excess*drag_deficit*evidence_weight;
        if(isfinite(live_debt)&&live_debt>0.0)
            target_work=fmax(target_work,current_work+live_debt);
    }
    if(current_work>=target_work)return NAN;

    double thermal=entry_thermal_protection_aoa_floor(v);
    double base=fmax(thermal,
        entry_terminal_turn_aoa_floor(t->true_air_speed,t->dynamic_pressure,v));
    double ceiling=entry_final_s_turn_aoa_ceiling(v);
    if(!(ceiling>base))return NAN;

    double current_lf=0.0,current_df=0.0;
    aerodynamic_force_factors_mach(t->mach,fabs(t->angle_of_attack),v,
        &current_lf,&current_df);
    double measured_lift=t->mass>DBL_MIN&&isfinite(t->lift_force)?
        fmax(0.0,t->lift_force/t->mass):0.0;
    double gravity=planet_surface_gravity(p);
    double load_limit=v->maximum_g_load*gravity;

    /* Restrict the search to the portion of the incidence envelope that is
       compatible with current measured lift/load authority.  The bound is
       derived from the live polar and load limit, not from a flight-phase
       alpha constant. */
    double safe_ceiling=base;
    double model_aoa_resolution=cfg->calibration.angle_of_attack_step;
    if(!isfinite(model_aoa_resolution)||!(model_aoa_resolution>0.0))
        model_aoa_resolution=sqrt(DBL_EPSILON)*fmax(1.0,ceiling-base);
    int samples=(int)ceil((ceiling-base)/model_aoa_resolution);
    if(samples<1)samples=1;
    for(int i=0;i<=samples;i++){
        double aoa=base+(ceiling-base)*(double)i/(double)samples;
        double lf=0.0,df=0.0;
        aerodynamic_force_factors_mach(t->mach,aoa,v,&lf,&df);
        if(!isfinite(lf)||!isfinite(df)||!(df>DBL_MIN))continue;

        double predicted_lift=(measured_lift>0.0&&fabs(current_lf)>DBL_MIN)?
            measured_lift*fabs(lf/current_lf):measured_lift;
        if(predicted_lift>load_limit)continue;
        safe_ceiling=aoa;
    }

    /* The polar is calibrated in configured AoA increments.  Solving below
       that model resolution only repeats the full atmosphere/path integral
       without adding physical information, so use the model resolution as
       the numerical solve resolution and retain machine precision as a floor. */
    double resolution=fmax(model_aoa_resolution,
        sqrt(DBL_EPSILON)*fmax(1.0,safe_ceiling));
    if(!(safe_ceiling>base+resolution))return NAN;

    double low=base,high=safe_ceiling;
    double low_work=terminal_projected_drag_work(g,t,p,aero,v,&cfg->guidance,
        low,path_distance,slope);
    double high_work=terminal_projected_drag_work(g,t,p,aero,v,&cfg->guidance,
        high,path_distance,slope);
    if(!isfinite(low_work)||low_work<0.0)return NAN;
    if(!isfinite(high_work)||high_work<target_work)return safe_ceiling;
    if(low_work>=target_work)return base;

    /* Drag work is evaluated from the same live atmosphere model at every
       candidate.  Solve its first feasible crossing rather than sampling a
       preferred alpha table; this preserves the minimum drag that satisfies
       the current energy contract and avoids spending the reserve early. */
    for(unsigned i=0;i<(unsigned)(2*DBL_MANT_DIG);i++){
        if(high-low<=resolution)break;
        double mid=.5*(low+high);
        double work=terminal_projected_drag_work(g,t,p,aero,v,&cfg->guidance,
            mid,path_distance,slope);
        if(!isfinite(work)||work>=target_work)high=mid;
        else low=mid;
    }
    return high>base+resolution?high:NAN;
}

static double entry_program_vertical_bank_ceiling(const GuidanceMachine*g,const Telemetry*t,
        const PlanetModel*p,AerodynamicModel aero,const LandingConfiguration*cfg){
    if(!g||!t||!p||!cfg||!g->taem_interface_target.valid)return NAN;
    double bank_limit=dynamic_bank_limit(t,&cfg->vehicle);
    if(!isfinite(bank_limit)||bank_limit<=0.0)return 0.0;

    double distance=NAN;
    double desired_fpa=entry_program_taem_path_fpa(g,t,cfg,&distance);
    if(!isfinite(desired_fpa)||!isfinite(t->flight_path_angle)||
       !isfinite(distance)||distance<=DBL_MIN)return bank_limit;

    /*
     * Bank feasibility must use the lift the vehicle can command now, not only
     * the lift produced by its current AoA.  Using live lift here created a
     * deadlock: a shallow/high-energy shuttle at 18 deg AoA was declared unable
     * to bank, so the controller never commanded the extra incidence that would
     * have made the turn supportable.
     */
    ControlAuthorityEnvelope authority=
        decision_control_authority_envelope(g,t,p,cfg);
    double lift=authority.valid?
        authority.maximum_normal_accel_mps2:live_lift_accel(t,aero,&cfg->vehicle);
    if(!isfinite(lift)||lift<=DBL_MIN)return 0.0;

    double radius=p->radius+fmax(0.0,t->mean_altitude);
    if(!(radius>DBL_MIN)||!isfinite(radius))return 0.0;
    double gravity=p->gravitational_parameter/(radius*radius);
    double horizontal=fmax(0.0,t->horizontal_speed);

    double latitude=isfinite(t->latitude)?clampd(t->latitude,-90.0,90.0):0.0;
    double course=isfinite(t->ground_track_heading)?
        t->ground_track_heading:t->heading;
    double rotation=p->rotational_speed*radius*cos(latitude*DEG2RAD);
    double inertial_horizontal_sq=horizontal*horizontal+rotation*rotation+
        2.0*horizontal*rotation*sin(course*DEG2RAD);
    double equilibrium=gravity-fmax(0.0,inertial_horizontal_sq)/radius;


    const TaemInterfaceTarget*q=&g->taem_interface_target;
    double response=fmax(fmax(0.0,q->response_time),
        authority.valid?authority.control_response_time_s:0.0);
    response=clampd(response,2.0,12.0);
    if(!(response>DBL_MIN)||!isfinite(response))return bank_limit;

    double gamma=t->flight_path_angle*DEG2RAD;
    double cos_gamma=cos(gamma);
    if(fabs(cos_gamma)<=DBL_EPSILON)return bank_limit;

    /* In thin air / hypersonic entry, available aerodynamic lift cannot support
       level flight (lift < equilibrium * cos_gamma). The vehicle is descending
       along an entry arc where dynamic pressure builds with depth; it must not be
       forced wings-level (bank=0) while high above the TAEM altitude. */
    if(lift<equilibrium*cos_gamma&&t->mean_altitude>q->altitude&&
       t->flight_path_angle>=desired_fpa-3.0)
        return bank_limit;

    /*
     * If MM304 is already steeper than the geometric inlet slope, do not spend
     * all available lift trying to erase that FPA error inside one actuator
     * response time.  Preserve the measured glide first; the altitude budget can
     * still reduce bank as the actual interface height is approached.
     */
    double vertical_reference_fpa=fmin(desired_fpa,t->flight_path_angle);
    double fpa_accel=t->true_air_speed*
        (vertical_reference_fpa-t->flight_path_angle)*DEG2RAD/response;
    double altitude_accel=2.0*(q->altitude-t->mean_altitude-
        t->vertical_speed*response)/(response*response);
    double radial_lift=(equilibrium+altitude_accel+
        live_drag_accel(t,aero,&cfg->vehicle)*sin(gamma))/cos_gamma;
    double effectiveness=isfinite(t->bank_effectiveness)&&
        t->bank_effectiveness>0.0?t->bank_effectiveness:1.0;
    double max_effective=fmin(bank_limit*effectiveness,
        nextafter(90.0,0.0))*DEG2RAD;
    double minimum_vertical=lift*cos(max_effective);
    double required_vertical=fmax(0.0,
        fmax(radial_lift,equilibrium*cos_gamma+fpa_accel));
    if(required_vertical<=minimum_vertical)return bank_limit;
    if(required_vertical>=lift){
        /* If MM304 is already descending more steeply than the downstream
         * tangent, the required vertical acceleration is not a wings-level
         * constraint: banking is how the vehicle trades lift for the planned
         * descent while acquiring the fixed station.  The previous branch
         * returned zero bank for every state below the tangent altitude,
         * which made the high-energy +crossrange case fly straight past the
         * recoverable handoff. */
        if(t->mean_altitude>q->altitude&&
           t->flight_path_angle>=desired_fpa-2.5)
            return bank_limit;
        if(desired_fpa<t->flight_path_angle-1.0)
            return bank_limit;
        return 0.0;
    }

    double effective=acos(clampd(required_vertical/lift,0.0,1.0))*RAD2DEG;
    return clampd(effective/effectiveness,0.0,bank_limit);
}

static bool entry_program_altitude_capture(GuidanceMachine*g,const Telemetry*t,
        const PlanetModel*p,AerodynamicModel aero,const LandingConfiguration*cfg,
        double*bank,double*aoa){
    if(!g||!t||!p||!cfg||!bank||!aoa||!g->taem_interface_target.valid)
        return false;

    /* This is a feasibility ceiling, never a new bank demand. In particular,
       a zero-degree ceiling is a valid wings-level constraint, not a reason
       to skip the limiter. The planner owns any increase in bank magnitude. */
    double ceiling=entry_program_vertical_bank_ceiling(g,t,p,aero,cfg);
    if(!isfinite(ceiling)||ceiling<0.0||!isfinite(*bank)||fabs(*bank)<=ceiling)
        return false;
    *bank=copysign(ceiling,*bank);
    *aoa=fmax(*aoa,entry_thermal_protection_aoa_floor(&cfg->vehicle));
    return true;
}


/* Shared executable vertical shaping: both live ticks and guidance replay use
   these altitude/FPA budgets. Legacy topology proposals also receive this local
   seed, but only the full guidance replay evaluates its evolution over time. */
static bool entry_program_shape_vertical_capture_plan(GuidanceMachine*g,const Telemetry*t,
        const PlanetModel*p,AerodynamicModel aero,const LandingConfiguration*cfg,
        EntryControlPlan*plan){
    if(!plan||!plan->valid)return false;
    const VehicleProfile*v=&cfg->vehicle;
    double bank=plan->target_bank,aoa=plan->target_aoa;
    bool changed=entry_program_altitude_capture(g,t,p,aero,cfg,&bank,&aoa);

    /*
     * If the vehicle is already steeper than the geometric path to the owned
     * interface target, do not add drag merely because an old plan requested a
     * higher AoA.  Use the measured best-glide incidence when available, but
     * never below the configured thermal-protection floor.
     * Only apply relief during gentle glides; in steep descents, full AoA lift is needed.
     */
    double path_distance=NAN;
    double path_fpa=entry_program_taem_path_fpa(g,t,cfg,&path_distance);
    ControlAuthorityEnvelope authority=
        decision_control_authority_envelope(g,t,p,cfg);
    if(authority.valid&&authority.survivable&&isfinite(path_fpa)&&
       isfinite(path_distance)&&path_distance>DBL_MIN&&
       isfinite(t->flight_path_angle)&&t->flight_path_angle<path_fpa&&
       t->flight_path_angle>=-6.0&&t->vertical_speed>=-50.0){
        double thermal=entry_thermal_protection_aoa_floor(v);
        double best_glide=isfinite(t->calibrated_best_glide_angle_of_attack)&&
            t->calibrated_best_glide_angle_of_attack>0.0?
            t->calibrated_best_glide_angle_of_attack:thermal;
        double relief_target=clampd(fmax(thermal,best_glide),
            thermal,v->maximum_angle_of_attack);
        if(aoa>relief_target+sqrt(DBL_EPSILON)){
            aoa=relief_target;
            changed=true;
        }
    }

    /*
     * Execute the same lift allocation used by the bank ceiling.  Preserve the
     * current glide angle when it is already steeper than the geometric path
     * rather than demanding an energy-expensive pull-up, but add enough AoA to
     * support the selected bank whenever the measured polar can do so.
     */
    if(isfinite(path_fpa)&&isfinite(t->flight_path_angle)&&isfinite(bank)){
        double support_fpa=fmin(path_fpa,t->flight_path_angle);
        Telemetry banked=*t;
        banked.roll=bank;
        double support_aoa=terminal_fpa_force_aoa(&banked,p,aero,v,support_fpa);
        if(isfinite(support_aoa)&&support_aoa>aoa+sqrt(DBL_EPSILON)){
            aoa=clampd(support_aoa,0.0,v->maximum_angle_of_attack);
            changed=true;
        }
    }


    if(!changed)return false;
    plan->target_bank=bank;
    plan->target_aoa=aoa;
    plan->target_turn_radius=live_turn_radius(t,aero,v,bank);
    return true;
}

                                                                                 
                                                                           
                                                                          
                                                                                
                                                                            
                                                                            
                                                          

                
               
                          
                         
                          
                            

