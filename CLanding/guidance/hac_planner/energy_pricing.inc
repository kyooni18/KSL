typedef struct {
    bool valid;
    Telemetry end;
    double work;
    double mean_drag;
    double mean_aoa;
    double max_aoa;
    double minimum_lateral_margin;
} FixedHacArcIntegration;

/*
 * Propagate the analytic HAC at the actual guidance command cadence.  This is
 * the energy certificate's authoritative arc model:
 *
 *   reference FPA -> requested AoA
 *   lateral curvature -> minimum AoA/bank requirement
 *   measured pitch rate/acceleration -> applied AoA
 *   stock-KSP / calibrated whole-vessel force -> lift and drag
 *   lift + bank + local Kerbin gravity -> FPA and altitude
 *   drag work + exact rotating potential difference -> next speed
 *
 * The radius is therefore judged at the speed/FPA/AoA that exists at each
 * point on the arc, not at the high-energy MM305 state that existed before the
 * lead.
 */
static FixedHacArcIntegration terminal_fixed_hac_integrate_arc(
        const GuidanceMachine*g,const Telemetry*initial,const PlanetModel*p,
        AerodynamicModel aero,const VehicleProfile*v,const GuidanceSettings*s,
        const LandingSite*site,const HACTransitionPlan*plan,
        const GuidanceMachine*vertical_profile,double radius,double arc_angle,
        double drag_anchor){
    FixedHacArcIntegration out={0};
    out.minimum_lateral_margin=INFINITY;
    if(!g||!initial||!p||!v||!s||!site||!plan||!plan->valid||
       !vertical_profile||!vertical_profile->fixed_hac_reference_valid||
       !(radius>DBL_MIN)||!(arc_angle>DBL_MIN)||
       !(s->guidance_rate>DBL_MIN)||!isfinite(drag_anchor)||drag_anchor<0.0)
        return out;

    double arc_length=radius*arc_angle;
    double plan_arc=fmax(0.0,plan->cone_arc_length);
    double arc_tolerance=sqrt(DBL_EPSILON)*fmax(1.0,arc_length);
    if(!(plan_arc>0.0)||fabs(plan_arc-arc_length)>arc_tolerance)return out;

    double signed_angle=plan->cone_end_angle-plan->cone_start_angle;
    if(!isfinite(signed_angle)||fabs(signed_angle)<=DBL_MIN)return out;
    double side=signed_angle<0.0?1.0:-1.0;

    Telemetry sim=*initial;
    sim.physics_sample_valid=false;
    sim.has_force_vectors=false;
    sim.has_attitude_quaternion=false;
    sim.has_command_pitch_error=false;
    sim.has_command_roll_error=false;
    sim.has_command_heading_error=false;

    double aoa=clampd(fabs(initial->angle_of_attack),0.0,
        v->maximum_angle_of_attack);
    JerkLimiter aoa_limiter=g->pitch_limiter;
    seed_stabilized_limiter(&aoa_limiter,aoa,
        controlled_aoa_rate(initial),false);

    GeoPoint origin={site->latitude,site->longitude,site->altitude};
    double speed=initial->true_air_speed;
    double altitude=initial->mean_altitude;
    double gamma=initial->flight_path_angle*DEG2RAD;
    double latitude=initial->latitude,longitude=initial->longitude;
    double travelled=0.0,elapsed=0.0,work=0.0;
    double drag_distance_integral=0.0,aoa_distance_integral=0.0;
    double max_aoa=aoa;
    const double command_dt=1.0/s->guidance_rate;

    while(travelled<arc_length-arc_tolerance){
        if(!(speed>DBL_MIN)||!isfinite(speed)||!isfinite(altitude)||
           altitude<0.0||!isfinite(gamma))return out;

        sim.mean_altitude=altitude;
        sim.true_air_speed=speed;
        sim.horizontal_speed=speed*fabs(cos(gamma));
        sim.surface_speed=speed;
        sim.flight_path_angle=gamma*RAD2DEG;
        sim.vertical_speed=speed*sin(gamma);
        sim.ut=initial->ut+elapsed;
        sim.latitude=latitude;sim.longitude=longitude;
        sim.angle_of_attack=aoa;
        sim.angle_of_attack_rate=aoa_limiter.rate;
        sim.has_angle_of_attack_rate=true;
        sim.atmospheric_density=planet_atmospheric_density(p,altitude);
        sim.speed_of_sound=planet_atmospheric_speed_of_sound(p,altitude);
        sim.mach=sim.speed_of_sound>DBL_MIN?speed/sim.speed_of_sound:NAN;
        sim.dynamic_pressure=.5*sim.atmospheric_density*speed*speed;
        if(!isfinite(sim.atmospheric_density)||sim.atmospheric_density<0.0||
           !isfinite(sim.speed_of_sound)||sim.speed_of_sound<=0.0||
           !isfinite(sim.dynamic_pressure))return out;

        double horizontal=sim.horizontal_speed;
        if(!(horizontal>DBL_MIN))return out;
        double remaining=arc_length-travelled;
        double ground_ds=fmin(remaining,horizontal*command_dt);
        double dt=ground_ds/horizontal;
        if(!(dt>0.0)||!isfinite(dt))return out;

        double reference_slope=fixed_hac_reference_slope(vertical_profile,
            remaining);
        if(!isfinite(reference_slope)||reference_slope<0.0||
           reference_slope>=75.0)return out;

        double upper=v->maximum_angle_of_attack;
        if(sim.mach<1.0&&
           isfinite(v->terminal_maximum_lift_angle_of_attack)&&
           v->terminal_maximum_lift_angle_of_attack>0.0)
            upper=fmin(upper,v->terminal_maximum_lift_angle_of_attack);
        upper=fmax(0.0,upper);

        double fpa_aoa=terminal_fpa_force_aoa(&sim,p,aero,v,
            -reference_slope);
        double bank_limit=fmin(v->maximum_bank_angle,
            dynamic_bank_limit(&sim,v));
        double required_lateral=horizontal*horizontal/radius;
        double lateral_aoa=terminal_required_aoa_for_lateral_limit(
            &sim,aero,v,required_lateral,bank_limit,upper);
        double requested=0.0;
        if(isfinite(fpa_aoa)&&isfinite(lateral_aoa))
            requested=hypot(fmax(0.0,fpa_aoa),fmax(0.0,lateral_aoa));
        else if(isfinite(fpa_aoa))requested=fmax(0.0,fpa_aoa);
        else if(isfinite(lateral_aoa))requested=fmax(0.0,lateral_aoa);
        requested=clampd(requested,0.0,upper);

        GuidanceAttitudeLimits pitch_limits=stabilized_attitude_limits(
            &sim,s,PHASE_TAEM,true);
        aoa=clampd(jerk_update(&aoa_limiter,requested,
            pitch_limits.rate_deg_s,pitch_limits.accel_deg_s2,dt),
            0.0,upper);
        aoa_limiter.value=aoa;
        sim.angle_of_attack=aoa;
        sim.angle_of_attack_rate=aoa_limiter.rate;

        double lift=terminal_fixed_hac_projected_lift(g,&sim,aero,v,aoa);
        double drag=terminal_fixed_hac_projected_drag(g,&sim,aero,v,aoa,
            drag_anchor);
        if(!isfinite(lift)||lift<0.0||!isfinite(drag)||drag<0.0)return out;

        double local_radius=p->radius+altitude;
        if(!(local_radius>DBL_MIN))return out;
        double gravity=p->gravitational_parameter/(local_radius*local_radius);
        double normal=fmin(lift,gravity*fmax(0.0,v->maximum_g_load));
        double effectiveness=isfinite(sim.bank_effectiveness)&&
            sim.bank_effectiveness>0.0?sim.bank_effectiveness:1.0;
        effectiveness=clampd(effectiveness,.35,1.8);
        double effective_limit=clampd(bank_limit*effectiveness,0.0,
            nextafter(89.0,0.0))*DEG2RAD;
        double available_lateral=normal*sin(effective_limit);
        double lateral_margin=available_lateral-required_lateral;
        out.minimum_lateral_margin=fmin(out.minimum_lateral_margin,
            lateral_margin);
        if(!isfinite(lateral_margin)||lateral_margin<
           -sqrt(DBL_EPSILON)*fmax(1.0,available_lateral))
            return out;

        double required_effective_bank=normal>DBL_MIN?
            asin(clampd(required_lateral/normal,0.0,1.0)):0.0;
        double actual_bank=required_effective_bank/effectiveness;
        sim.roll=copysign(actual_bank*RAD2DEG,side);
        sim.lift_force=lift*fmax(1.0,sim.mass);
        sim.drag_force=drag*fmax(1.0,sim.mass);
        sim.g_force=gravity>DBL_MIN?normal/gravity:sim.g_force;

        /*
         * Flight-path dynamics for an unpowered banked vehicle on a spherical
         * planet.  The same lift vector supplies both horizontal curvature and
         * vertical FPA control.
         */
        double vertical_lift=normal*cos(actual_bank);
        double gamma_rate=(vertical_lift-gravity*cos(gamma))/speed+
            speed*cos(gamma)/local_radius;
        double next_gamma=gamma+gamma_rate*dt;
        double mid_gamma=.5*(gamma+next_gamma);
        double next_altitude=altitude+speed*sin(mid_gamma)*dt;
        if(!isfinite(next_gamma)||!isfinite(next_altitude)||next_altitude<0.0||
           next_gamma<-60.0*DEG2RAD||next_gamma>20.0*DEG2RAD)return out;

        double next_travelled=travelled+ground_ds;
        double angle=plan->cone_start_angle-side*(next_travelled/radius);
        HACPoint2 next_local=hac_point(
            plan->cone_center.e+radius*cos(angle),
            plan->cone_center.n+radius*sin(angle));
        GeoPoint next_point=local_point(origin,next_local.e,next_local.n,
            p->radius,next_altitude);
        double potential_drop=rotating_specific_energy(latitude,altitude,0.0,p)-
            rotating_specific_energy(next_point.latitude,next_altitude,0.0,p);
        double air_ds=speed*dt;
        double next_speed=NAN,segment_work=NAN;
        if(!terminal_quadratic_drag_energy_step(speed,drag,air_ds,
                potential_drop,&next_speed,&segment_work))
            return out;

        work+=segment_work;
        drag_distance_integral+=drag*ground_ds;
        aoa_distance_integral+=aoa*ground_ds;
        max_aoa=fmax(max_aoa,aoa);
        elapsed+=dt;
        travelled=next_travelled;
        speed=next_speed;
        altitude=next_altitude;
        gamma=next_gamma;
        latitude=next_point.latitude;
        longitude=next_point.longitude;
    }

    sim.mean_altitude=altitude;
    sim.true_air_speed=speed;
    sim.horizontal_speed=speed*fabs(cos(gamma));
    sim.surface_speed=speed;
    sim.flight_path_angle=gamma*RAD2DEG;
    sim.vertical_speed=speed*sin(gamma);
    sim.ut=initial->ut+elapsed;
    sim.latitude=latitude;sim.longitude=longitude;
    sim.angle_of_attack=aoa;sim.angle_of_attack_rate=aoa_limiter.rate;
    sim.has_angle_of_attack_rate=true;
    sim.radar_altitude=fmax(0.0,altitude-site->altitude);
    sim.speed_of_sound=planet_atmospheric_speed_of_sound(p,altitude);
    sim.mach=sim.speed_of_sound>DBL_MIN?speed/sim.speed_of_sound:NAN;
    sim.atmospheric_density=planet_atmospheric_density(p,altitude);
    sim.dynamic_pressure=.5*sim.atmospheric_density*speed*speed;
    GeoPoint end_point={latitude,longitude,altitude};
    runway_coordinates(end_point,origin,site->runway_heading,p->radius,
        &sim.runway_along_track,&sim.runway_cross_track);
    sim.range_to_site=great_circle_distance(end_point,origin,p->radius);

    out.valid=true;
    out.end=sim;
    out.work=work;
    out.mean_drag=arc_length>DBL_MIN?
        drag_distance_integral/arc_length:NAN;
    out.mean_aoa=arc_length>DBL_MIN?
        aoa_distance_integral/arc_length:NAN;
    out.max_aoa=max_aoa;
    return out;
}

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
        const HACTransitionPlan*plan,const GuidanceMachine*vertical_profile,
        double fixed_profile_slope,double fixed_target_aoa){
    if(audit)*audit=(FixedHacEnergyAudit){0};
    if(radius_out)*radius_out=INFINITY;
    if(margin_out)*margin_out=NAN;
    if(loss_out)*loss_out=NAN;
    if(available_out)*available_out=NAN;
    if(required_out)*required_out=NAN;
    if(!g||!t||!p||!v||!s||!site||!drag_reference||!(requested_radius>0.0)||!(arc_angle>0.0)||
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
    double target_aoa=terminal_hac_energy_aoa(t,v);
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
            if(profile_slope>glide_slope){
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
    FixedHacArcIntegration integrated_arc={0};
    bool use_integrated_arc=plan&&vertical_profile&&
        vertical_profile->fixed_hac_reference_valid&&lead_length<=DBL_MIN;
    double arc_work=NAN;
    Telemetry arc_end_state={0};
    if(use_integrated_arc){
        integrated_arc=terminal_fixed_hac_integrate_arc(g,t,p,aero,v,s,site,
            plan,vertical_profile,requested_radius,arc_angle,drag_anchor);
        if(integrated_arc.valid){
            arc_work=integrated_arc.work;
            arc_end_state=integrated_arc.end;
            arc_end_speed=arc_end_state.true_air_speed;
            arc_end_altitude=arc_end_state.mean_altitude;
            target_aoa=integrated_arc.mean_aoa;
        }
    }
    if(!isfinite(arc_work)){
        arc_work=terminal_projected_drag_work_state_anchored(g,t,p,aero,v,s,
            target_aoa,vertical_path,profile_slope,&arc_end_speed,
            &arc_end_altitude,drag_anchor);
    }
    double end_speed=arc_end_speed,end_altitude=arc_end_altitude;
    double final_work=0.0;
    if(isfinite(arc_work)&&arc_work>=0.0&&final_to_gate>DBL_MIN&&
       isfinite(arc_end_speed)&&isfinite(arc_end_altitude)){
        Telemetry after_arc=integrated_arc.valid?arc_end_state:*t;
        after_arc.mean_altitude=arc_end_altitude;
        after_arc.true_air_speed=arc_end_speed;
        if(!integrated_arc.valid){
            after_arc.horizontal_speed=arc_end_speed*cos(slope*DEG2RAD);
            after_arc.vertical_speed=-arc_end_speed*sin(slope*DEG2RAD);
            after_arc.flight_path_angle=-slope;
            after_arc.angle_of_attack=target_aoa;
        }
        after_arc.roll=0.0;
        after_arc.lift_force=0.0;
        after_arc.drag_force=0.0;
        double sound=planet_atmospheric_speed_of_sound(p,arc_end_altitude);
        after_arc.mach=sound>DBL_MIN?arc_end_speed/sound:t->mach;
        after_arc.atmospheric_density=planet_atmospheric_density(p,arc_end_altitude);
        after_arc.dynamic_pressure=.5*after_arc.atmospheric_density*arc_end_speed*arc_end_speed;
        double final_target_aoa=terminal_fpa_force_aoa(&after_arc,p,aero,v,-slope);
        if(!isfinite(final_target_aoa))final_target_aoa=terminal_hac_energy_aoa(&after_arc,v);
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
        audit->arc_mean_aoa=integrated_arc.valid?integrated_arc.mean_aoa:target_aoa;
        audit->arc_max_aoa=integrated_arc.valid?integrated_arc.max_aoa:target_aoa;
        audit->arc_min_lateral_margin=integrated_arc.valid?
            integrated_arc.minimum_lateral_margin:NAN;
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
    const double perpendicular=90.0; /* decision-literal: mathematical-numerical-requirement | analytic HAC inlet tangent */
    double positive=norm_deg(cfg->site.runway_heading+perpendicular);
    double negative=norm_deg(cfg->site.runway_heading-perpendicular);
    double positive_error=fabs(norm_signed_deg(positive-course));
    double negative_error=fabs(norm_signed_deg(negative-course));
    double capture_course=positive_error<=negative_error?positive:negative;
    double course_error=fmin(positive_error,negative_error);
    double relative=norm_signed_deg(capture_course-cfg->site.runway_heading);
    double angular_resolution=sqrt(DBL_EPSILON)*RAD2DEG;
    if(fabs(relative)<=angular_resolution)return false;
    double side=relative>0.0?1.0:-1.0;

    /* This is an MM305/p0 detector, so compare against the actual analytic HAC
       inlet.  The old implementation compared against the Final station at
       (-finalApproachDistance,0), which made a true p0 state look tens of
       kilometres away and sent the HAC-only test into generic topology search. */
    HACPoint2 entry={0},center={0},exit={0};
    if(!hac_fixed_alignment_geometry(&entry,&center,&exit,&cfg->site,s,
            s->hac_radius,capture_course,side))return false;
    double runway=cfg->site.runway_heading*DEG2RAD;
    double inlet_along=entry.e*sin(runway)+entry.n*cos(runway);
    double inlet_cross=entry.e*cos(runway)-entry.n*sin(runway);
    double position_error=hypot(t->runway_along_track-inlet_along,
        t->runway_cross_track-inlet_cross);
    double position_limit=fmax(1000.0,contract.horizontal_radius_m);

    if(course_error>contract.perpendicular_heading_half_width_deg||
       position_error>position_limit)return false;
    if(capture_course_out)*capture_course_out=capture_course;
    return true;
}
