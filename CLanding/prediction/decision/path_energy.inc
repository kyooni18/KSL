UnpoweredPathProjection decision_unpowered_path_projection(
        const GuidanceMachine *g,const Telemetry *t,const PlanetModel *p,
        const LandingConfiguration *cfg,double target_altitude_m,
        double ground_path_m){
    UnpoweredPathProjection out={0};
    if(!t||!p||!cfg||!isfinite(target_altitude_m)||!isfinite(ground_path_m)||
       ground_path_m<0.0||!(t->true_air_speed>0.0)||!isfinite(t->true_air_speed))
        return out;

    const VehicleProfile*v=&cfg->vehicle;
    double beta=isfinite(t->estimated_ballistic_coefficient)&&
        t->estimated_ballistic_coefficient>0.0?
        t->estimated_ballistic_coefficient:v->estimated_ballistic_coefficient;
    if(!(beta>DBL_MIN)||!isfinite(beta))return out;

    double height_delta=target_altitude_m-t->mean_altitude;
    double air_path=hypot(ground_path_m,height_delta);
    if(!(air_path>DBL_MIN)||!isfinite(air_path)){
        out.valid=true;out.ground_path_m=ground_path_m;out.air_path_m=0.0;
        out.terminal_speed_mps=t->true_air_speed;out.travel_time_s=0.0;
        out.target_aoa_deg=hypot(t->angle_of_attack,t->sideslip);
        return out;
    }

    ControlAuthorityEnvelope control=decision_control_authority_envelope(g,t,p,cfg);
    double response_distance=control.valid&&isfinite(control.control_response_time_s)?
        t->true_air_speed*fmax(0.0,control.control_response_time_s):0.0;
    double initial_aoa=hypot(t->angle_of_attack,t->sideslip);
    double target_aoa=best_glide_aoa(t->mach,v);
    double density_scale=isfinite(t->trajectory_density_scale)&&t->trajectory_density_scale>0.0?
        t->trajectory_density_scale:1.0;
    double drag_scale=isfinite(t->trajectory_drag_scale)&&t->trajectory_drag_scale>0.0?
        t->trajectory_drag_scale:1.0;

    /* Use the atmosphere table itself as the integration resolution. Each
       crossed density interval gets one exact-in-v^2 constant-coefficient
       energy step. This avoids a tuned time/distance step count. */
    unsigned segments=1;
    if(p->atmosphere_sample_count>=2&&height_delta!=0.0){
        double lo=fmin(t->mean_altitude,target_altitude_m);
        double hi=fmax(t->mean_altitude,target_altitude_m);
        for(size_t i=1;i<p->atmosphere_sample_count;i++){
            double a=p->atmosphere_altitude[i];
            if(a>lo&&a<hi)segments++;
        }
    }
    if(response_distance>DBL_MIN&&response_distance<air_path)segments++;

    double ds=air_path/(double)segments;
    double vertical_per_air=height_delta/air_path;
    double speed2=t->true_air_speed*t->true_air_speed;
    double work=0.0,time=0.0;

    for(unsigned i=0;i<segments;i++){
        double s0=(double)i*ds,s1=(double)(i+1)*ds,sm=.5*(s0+s1);
        double f=sm/air_path;
        double altitude=t->mean_altitude+height_delta*f;
        double density=planet_atmospheric_density(p,altitude)*density_scale;
        double speed0=sqrt(fmax(0.0,speed2));
        double sound=planet_atmospheric_speed_of_sound(p,altitude);
        double mach=sound>DBL_EPSILON?speed0/sound:t->mach;
        double response_progress=response_distance>DBL_MIN?
            clampd(sm/response_distance,0.0,1.0):1.0;
        /* Best-L/D incidence is Mach dependent. Holding the source-state optimum
           through the entire hypersonic-to-terminal descent can price enormous
           drag that the executable vehicle would never command. Re-evaluate the
           aerodynamic optimum at each atmosphere/Mach integration cell while the
           initial control-response transient still ramps from measured incidence. */
        double local_target_aoa=best_glide_aoa(mach,v);
        if(!isfinite(local_target_aoa))return out;
        target_aoa=local_target_aoa;
        double aoa=initial_aoa+(local_target_aoa-initial_aoa)*response_progress;
        double df=1.0;
        aerodynamic_force_factors_mach(mach,aoa,v,NULL,&df);
        double drag_coefficient=fmax(0.0,density)*fmax(0.0,df)*drag_scale/beta;
        double radius=p->radius+altitude;
        if(!(radius>DBL_MIN)||!(p->gravitational_parameter>0.0))return out;
        double gravity=p->gravitational_parameter/(radius*radius);
        double drive=-2.0*gravity*vertical_per_air;
        double next2;
        if(drag_coefficient>DBL_EPSILON){
            double equilibrium=drive/drag_coefficient;
            next2=equilibrium+(speed2-equilibrium)*exp(-drag_coefficient*ds);
        }else{
            next2=speed2+drive*ds;
        }
        next2=fmax(0.0,next2);
        double dh=vertical_per_air*ds;
        double segment_work=.5*(speed2-next2)-gravity*dh;
        work+=fmax(0.0,segment_work);
        double speed1=sqrt(next2);
        double mean_speed=.5*(speed0+speed1); /* decision-literal-ok: trapezoidal time integration */
        if(!(mean_speed>DBL_MIN))return out;
        time+=ds/mean_speed;
        speed2=next2;
    }

    out.valid=isfinite(work)&&isfinite(speed2)&&isfinite(time);
    out.ground_path_m=ground_path_m;
    out.air_path_m=air_path;
    out.modeled_drag_work=work;
    out.terminal_speed_mps=sqrt(fmax(0.0,speed2));
    out.travel_time_s=time;
    out.target_aoa_deg=target_aoa;
    return out;
}

EnergyPathEnvelope decision_energy_path_envelope(
        const Telemetry *t,
        const PlanetModel *p,
        double available_specific_energy,
        double modeled_drag_work,
        double path_length_m) {
    EnergyPathEnvelope out = {0};
    if (!t || !p || !isfinite(available_specific_energy) ||
        !isfinite(modeled_drag_work) || !isfinite(path_length_m) ||
        path_length_m < 0.0 || modeled_drag_work < 0.0)
        return out;

    double gravity = planet_surface_gravity(p);
    if (!(gravity > 0.0) || !isfinite(gravity)) return out;

    /*
     * Convert observed model residuals into the same specific-energy unit as
     * the path budget.  No confidence percentage is translated into a policy
     * margin: only measured residual magnitude and a certified relative
     * physics uncertainty contribute.
     */
    double uncertainty = 0.0;
    if (isfinite(t->trajectory_altitude_residual))
        uncertainty += gravity * fabs(t->trajectory_altitude_residual);
    if (isfinite(t->trajectory_speed_residual)) {
        double dv = fabs(t->trajectory_speed_residual);
        uncertainty += fabs(t->true_air_speed) * dv + 0.5 * dv * dv;
    }
    if (isfinite(t->trajectory_range_residual) && path_length_m > 0.0) {
        double mean_drag_loss = modeled_drag_work / path_length_m;
        uncertainty += fabs(mean_drag_loss * t->trajectory_range_residual);
    }

    double relative_uncertainty = 0.0;
    if (isfinite(t->physics_certified_uncertainty) &&
        t->physics_certified_uncertainty > 0.0)
        relative_uncertainty = t->physics_certified_uncertainty;
    if (isfinite(t->predicted_physics_relative_uncertainty) &&
        t->predicted_physics_relative_uncertainty > 0.0)
        relative_uncertainty = fmax(relative_uncertainty,
            t->predicted_physics_relative_uncertainty);
    /* Subsonic/transonic terminal glide should use uncertainty from the active
       aerodynamic regime rather than missing supersonic/hypersonic calibration.
       Do not, however, make that active evidence artificially more certain than
       it actually is.  The previous 8% upper clamp converted ~0.80 measured
       aerodynamic confidence into only 8% uncertainty, which systematically
       overestimated remaining HAC energy.  Active evidence now replaces the
       global uncertainty at its measured magnitude (bounded only to a sane
       relative domain); the legacy 8% fallback is retained only when no
       active-regime evidence exists at all. */
    if (isfinite(t->mach) && t->mach < 1.0 && relative_uncertainty > 0.08) {
        double active_unc = 0.0;
        bool active_evidence = false;
        if (isfinite(t->aerodynamic_confidence) && t->aerodynamic_confidence > 0.0) {
            active_unc = fmax(active_unc, 1.0 - t->aerodynamic_confidence);
            active_evidence = true;
        }
        if (isfinite(t->physics_model_residual) && t->physics_model_residual >= 0.0 &&
            isfinite(t->physics_model_residual_confidence) &&
            t->physics_model_residual_confidence > 0.0) {
            active_unc = fmax(active_unc, t->physics_model_residual);
            active_evidence = true;
        }
        relative_uncertainty = fmin(relative_uncertainty,
            active_evidence ? clampd(active_unc, 0.0, 0.35) : 0.08);
    }
    uncertainty += modeled_drag_work * relative_uncertainty;

    out.available_specific_energy = available_specific_energy;
    out.modeled_drag_work = modeled_drag_work;
    out.uncertainty_specific_energy = uncertainty;
    out.energy = decision_margin(available_specific_energy,
        modeled_drag_work + uncertainty);
    out.valid = out.energy.valid;
    out.feasible = out.valid && out.energy.margin >= 0.0;
    return out;
}

PathFeasibilityEnvelope decision_path_feasibility_envelope(
        double path_length_m,
        double minimum_path_m,
        double maximum_path_m,
        double slope_deg,
        double minimum_slope_deg,
        double maximum_slope_deg,
        double required_lateral_accel_mps2,
        double available_lateral_accel_mps2,
        double peak_course_rate_ratio,
        double endpoint_course_rate_deg_s,
        double endpoint_course_rate_limit_deg_s) {
    PathFeasibilityEnvelope out = {0};
    if (!isfinite(path_length_m) || !isfinite(minimum_path_m) ||
        !isfinite(maximum_path_m) || !isfinite(slope_deg) ||
        !isfinite(minimum_slope_deg) || !isfinite(maximum_slope_deg) ||
        !isfinite(required_lateral_accel_mps2) ||
        !isfinite(available_lateral_accel_mps2) ||
        !isfinite(peak_course_rate_ratio) ||
        !isfinite(endpoint_course_rate_deg_s) ||
        !isfinite(endpoint_course_rate_limit_deg_s) ||
        minimum_path_m < 0.0 || maximum_path_m < minimum_path_m ||
        maximum_slope_deg < minimum_slope_deg ||
        available_lateral_accel_mps2 < 0.0 ||
        endpoint_course_rate_limit_deg_s < 0.0)
        return out;

    out.path_minimum = decision_margin(path_length_m, minimum_path_m);
    out.path_maximum = decision_margin(maximum_path_m, path_length_m);
    out.slope_minimum = decision_margin(slope_deg, minimum_slope_deg);
    out.slope_maximum = decision_margin(maximum_slope_deg, slope_deg);
    out.lateral_authority = decision_margin(
        available_lateral_accel_mps2, required_lateral_accel_mps2);
    out.course_rate = decision_margin(1.0, peak_course_rate_ratio);
    out.endpoint_rate = decision_margin(
        endpoint_course_rate_limit_deg_s, endpoint_course_rate_deg_s);

    const DecisionMargin *margins[] = {
        &out.path_minimum, &out.path_maximum,
        &out.slope_minimum, &out.slope_maximum,
        &out.lateral_authority, &out.course_rate, &out.endpoint_rate,
    };
    out.worst_normalized_margin = INFINITY;
    for (size_t i = 0; i < sizeof(margins) / sizeof(margins[0]); ++i) {
        if (!margins[i]->valid) return out;
        out.worst_normalized_margin = fmin(
            out.worst_normalized_margin, margins[i]->normalized_margin);
    }

    out.valid = isfinite(out.worst_normalized_margin);
    out.feasible = out.valid && out.worst_normalized_margin >= 0.0;
    return out;
}

RunwayCaptureEnvelope decision_runway_capture_envelope(
        const GuidanceMachine *g,
        const Telemetry *t,
        double course_deg,
        const PlanetModel *p,
        const LandingConfiguration *cfg) {
    RunwayCaptureEnvelope out = {0};
    if (!t || !p || !cfg || !isfinite(course_deg)) return out;

    const LandingSite *site = &cfg->site;

    double gravity = planet_surface_gravity(p);
    double horizontal = finite_positive(t->horizontal_speed);
    if (!(gravity > 0.0) || !(horizontal > 0.0) ||
        !isfinite(t->runway_along_track) || !isfinite(t->runway_cross_track))
        return out;

    ControlAuthorityEnvelope authority =
        decision_control_authority_envelope(g, t, p, cfg);
    if (!authority.valid) return out;

    double course_error_rad =
        norm_signed_deg(course_deg - site->runway_heading) * DEG2RAD;
    double cross_rate = horizontal * sin(course_error_rad);

    double max_lift_aoa = authority.maximum_lift_aoa_deg;
    double normal_accel = authority.maximum_normal_accel_mps2;
    double lateral_accel = authority.maximum_lateral_accel_mps2;
    double response_time = authority.control_response_time_s;
    double lateral_time = decision_bounded_capture_time(
        t->runway_cross_track, cross_rate, lateral_accel, response_time);

    double course_rate_rad = horizontal > 0.0
        ? lateral_accel / horizontal : 0.0;
    double heading_time = course_rate_rad > 0.0
        ? response_time + fabs(course_error_rad) / course_rate_rad
        : INFINITY;

    double runway_forward = site->runway_length - t->runway_along_track;
    double direct_ground_range = hypot(fmax(0.0, runway_forward),
        t->runway_cross_track);
    /* A committed fixed HAC is not flying the direct runway chord.  Use the
       remaining finite lead, frozen arc and configured outer-final leg for the
       time/vertical envelope; otherwise the direct chord can report no time
       left while the vehicle is intentionally spending that time on the HAC.
       The fixed geometry itself remains owned by guidance and is never
       replaced or resized here. */
    bool fixed_hac=g&&g->fixed_alignment_hac_latched&&
        g->hac_transition_heading_cone&&!g->hac_completed&&
        !g->final_approach_captured;
    double committed_path=0.0;
    if(fixed_hac){
        double lead_remaining=0.0;
        double arc_remaining=0.0;
        if(g->hac_transition_active){
            if(isfinite(g->hac_transition_lead_length)&&g->hac_transition_lead_length>0.0)
                lead_remaining=fmax(0.0,(1.0-clampd(g->hac_transition_lead_progress,0.0,1.0))*
                    g->hac_transition_lead_length);
            if(g->hac_transition_heading_cone&&
               isfinite(g->hac_transition_cone_arc_length)&&g->hac_transition_cone_arc_length>0.0)
                arc_remaining=fmax(0.0,(1.0-clampd(g->hac_transition_progress,0.0,1.0))*
                    g->hac_transition_cone_arc_length);
            else if(isfinite(g->hac_transition_length))
                arc_remaining=fmax(0.0,(1.0-clampd(g->hac_transition_progress,0.0,1.0))*
                    g->hac_transition_length);
        }else{
            arc_remaining=fmax(0.0,g->hac_remaining);
        }
        /* Acquisition and the frozen circle are disjoint path segments.  Do not
           add hac_remaining while the stored transition is active: it is legacy
           circle state, not a third copy of the committed route. */
        committed_path=lead_remaining+arc_remaining+
            fmax(0.0,cfg->guidance.final_approach_distance);
    }
    double ground_range=fixed_hac&&isfinite(committed_path)&&committed_path>0.0?
        committed_path:direct_ground_range;
    double time_available=ground_range/horizontal;
    double time_required = fmax(lateral_time, heading_time);

    double sink = fmax(0.0, -t->vertical_speed);
    if (!isfinite(t->flight_path_angle) || !isfinite(t->roll) ||
        !isfinite(t->vertical_speed)) return out;
    double vertical_projection = cos(t->flight_path_angle * DEG2RAD);
    double vertical_accel = fmax(0.0, normal_accel * vertical_projection - gravity);
    double current_lift = measured_or_modeled_lift_accel(t, &cfg->vehicle);
    /* During response we credit current banked lift, but no favorable upward
       acceleration or drag support. After response the wings are level.
       Holding these accelerations is a local model, re-evaluated each sample. */
    double response_down = fmax(0.0, gravity - current_lift *
        cos(t->roll * DEG2RAD) * vertical_projection);
    double radar = isfinite(t->radar_altitude) && t->radar_altitude >= 0.0
        ? t->radar_altitude : t->mean_altitude - site->altitude;
    VerticalRecoveryEnvelope recovery = decision_vertical_recovery_envelope(
        radar, sink, 0.0, response_time, response_down, vertical_accel);
    double recovery_height = recovery.valid ? recovery.required_height_m : INFINITY;
    double recoverable_sink = 0.0;
    if (vertical_accel > 0.0 && radar >= 0.0) {
        /* Positive root of H = v*t + a*t^2/2 + (v+a*t)^2/(2*u).
           a is downward response acceleration; u is net upward authority. */
        // decision-literal: physical-law-constant | Inverse constant acceleration stopping height, including response drift.
        double radicand = vertical_accel * (vertical_accel + response_down) *
            response_time * response_time + 2.0 * vertical_accel * radar;
        recoverable_sink = fmax(0.0, sqrt(radicand) -
            (vertical_accel + response_down) * response_time);
    }

    out.runway_time = decision_margin(time_available, time_required);
    out.vertical_recovery = recovery.height;
    out.speed = authority.speed;
    out.dynamic_pressure = authority.dynamic_pressure;
    out.load = authority.load;
    out.runway_remaining = decision_margin(runway_forward, 0.0);

    out.ground_range_m = ground_range;
    out.time_available_s = time_available;
    out.lateral_capture_time_s = lateral_time;
    out.heading_capture_time_s = heading_time;
    out.control_response_time_s = response_time;
    out.cross_rate_mps = cross_rate;
    out.lateral_accel_mps2 = lateral_accel;
    out.course_rate_deg_s = course_rate_rad * RAD2DEG;
    out.maximum_lift_aoa_deg = max_lift_aoa;
    out.maximum_normal_accel_mps2 = normal_accel;
    out.sink_rate_mps = sink;
    out.vertical_recovery_accel_mps2 = vertical_accel;
    out.response_down_accel_mps2 = response_down;
    out.vertical_recovery_height_m = recovery_height;
    out.recoverable_sink_rate_mps = recoverable_sink;
    out.recoverable_fpa_deg = horizontal > 0.0
        ? atan2(recoverable_sink, horizontal) * RAD2DEG : 0.0;

    out.valid = out.runway_time.valid && out.vertical_recovery.valid &&
        out.speed.valid && out.dynamic_pressure.valid && out.load.valid &&
        out.runway_remaining.valid && isfinite(lateral_time) &&
        isfinite(heading_time);
    out.reachable = out.valid && authority.controllable &&
        out.runway_time.margin >= 0.0 &&
        out.vertical_recovery.margin >= 0.0 &&
        out.speed.margin >= 0.0 &&
        out.dynamic_pressure.margin >= 0.0 &&
        out.load.margin >= 0.0 &&
        out.runway_remaining.margin >= 0.0;
    return out;
}

/* Acquisition authority is separate from terminal-path commitment. The same
   state/energy/control envelope is evaluated by prediction and live guidance. */
