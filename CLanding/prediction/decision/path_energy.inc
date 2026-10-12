/* Prescribed descent reference. This is deliberately NOT a physical lower
 * bound: a shorter path can require more lift/drag than a longer trajectory.
 * It may screen an inlet only when lift and response support are demonstrated;
 * MM305 still independently replays its real bounded-curvature route. */
static bool path_reference_forces(const GuidanceMachine *g,const Telemetry *t,
        const PlanetModel *p,const LandingConfiguration *cfg,double fraction,
        double altitude,double speed,double aoa,double *lift,double *drag){
    (void)fraction;
    const VehicleProfile *v=&cfg->vehicle;
    double rho=planet_atmospheric_density(p,altitude);
    double sound=planet_atmospheric_speed_of_sound(p,altitude);
    if(!isfinite(rho)||rho<0.0||!(sound>0.0)||!isfinite(sound))return false;
    double q=.5*rho*speed*speed;
    const TerminalModel *model=g?g->entry_terminal_model:NULL;
    if(model){
        double lq=0.0,dq=0.0;
        aero_force_per_q(&model->aero,q,speed/sound,aoa,&lq,&dq);
        *lift=q*lq/t->mass;*drag=q*dq/t->mass;
    }else{
        /* Profile beta is anchored at Mach 5 / entry trim. Live q/D is NOT
           that beta. Normalize measured forces at their measured incidence
           before applying any Mach/incidence variation. */
        double lf=0.0,df=0.0,l0=0.0,d0=0.0;
        aerodynamic_force_factors_mach(speed/sound,aoa,v,&lf,&df);
        aerodynamic_force_factors_mach(t->mach,hypot(t->angle_of_attack,t->sideslip),v,&l0,&d0);
        double base=1.0/v->estimated_ballistic_coefficient;
        double lbase=base*v->estimated_lift_to_drag;
        if(isfinite(t->estimated_ballistic_coefficient)&&t->estimated_ballistic_coefficient>0.0&&d0>DBL_MIN){
            base=1.0/(t->estimated_ballistic_coefficient*d0);
            double ld=t->estimated_lift_to_drag>0.0?t->estimated_lift_to_drag:v->estimated_lift_to_drag;
            if(fabs(l0)>DBL_MIN)lbase=ld/(t->estimated_ballistic_coefficient*fabs(l0));
        }
        if(t->mass>0.0&&t->dynamic_pressure>0.0&&
           isfinite(t->lift_force)&&t->lift_force>0.0&&isfinite(t->drag_force)&&t->drag_force>0.0){
            if(t->drag_force>=0.0&&isfinite(t->drag_force)&&d0>DBL_MIN)
                base=t->drag_force/(t->mass*t->dynamic_pressure*d0);
            if(t->lift_force>=0.0&&isfinite(t->lift_force)&&fabs(l0)>DBL_MIN)
                lbase=t->lift_force/(t->mass*t->dynamic_pressure*fabs(l0));
        }
        *lift=q*lbase*fabs(lf);*drag=q*base*df;
    }
    return isfinite(*lift)&&*lift>=0.0&&isfinite(*drag)&&*drag>=0.0;
}

static void path_axis_step(double target,double dt,GuidanceAttitudeLimits limits,
        double *angle,double *rate){
    double error=target-*angle;
    double wanted=copysign(fmin(limits.rate_deg_s,
        sqrt(2.0*limits.accel_deg_s2*fabs(error))),error);
    double next=*rate+clampd(wanted-*rate,-limits.accel_deg_s2*dt,limits.accel_deg_s2*dt);
    double delta=.5*(*rate+next)*dt;
    if(delta*error>=0.0&&fabs(delta)>fabs(error)){*angle=target;*rate=0.0;}
    else{*angle+=delta;*rate=next;}
}

static bool path_acceleration(const GuidanceMachine *g,const Telemetry *t,
        const PlanetModel *p,const LandingConfiguration *cfg,Vec3 position,
        Vec3 velocity,double aoa,double bank,Vec3 *accel,double *drag_power){
    double r=v3_norm(position),speed=v3_norm(velocity);
    if(!(r>0.0)||!(speed>0.0))return false;
    Vec3 up=v3_scale(position,1.0/r),flow=v3_scale(velocity,1.0/speed);
    double lift=0.0,drag=0.0;
    if(!path_reference_forces(g,t,p,cfg,0.0,r-p->radius,speed,aoa,&lift,&drag))return false;
    Vec3 right=v3_normalized(v3_cross(flow,up));
    Vec3 lift_axis=v3_normalized(v3_cross(right,flow));
    lift_axis=v3_rotate_axis(lift_axis,flow,bank*DEG2RAD);
    Vec3 omega={0.0,0.0,p->rotational_speed};
    *accel=v3_add(v3_scale(position,-p->gravitational_parameter/(r*r*r)),
        v3_add(v3_scale(v3_cross(omega,v3_cross(omega,position)),-1.0),
        v3_add(v3_scale(v3_cross(omega,velocity),-2.0),
        v3_add(v3_scale(flow,-drag),v3_scale(lift_axis,lift)))));
    *drag_power=drag*speed;
    return isfinite(accel->x)&&isfinite(accel->y)&&isfinite(accel->z);
}

static UnpoweredPathProjection path_reference_integrate(const GuidanceMachine *g,
        const Telemetry *t,const PlanetModel *p,const LandingConfiguration *cfg,
        double target_altitude,double path,unsigned resolution){
    UnpoweredPathProjection out={0};
    double lat=t->latitude*DEG2RAD,r=p->radius+t->mean_altitude;
    Vec3 position={r*cos(lat),0.0,r*sin(lat)},up=v3_normalized(position);
    Vec3 north={-sin(lat),0.0,cos(lat)},east={0.0,1.0,0.0};
    double course=isfinite(t->ground_track_heading)?t->ground_track_heading*DEG2RAD:0.0;
    Vec3 horizontal=v3_add(v3_scale(north,cos(course)),v3_scale(east,sin(course)));
    double gamma=t->flight_path_angle*DEG2RAD;
    Vec3 velocity=v3_scale(v3_add(v3_scale(horizontal,cos(gamma)),v3_scale(up,sin(gamma))),t->true_air_speed);
    double aoa=t->angle_of_attack,aoa_rate=t->has_angle_of_attack_rate?t->angle_of_attack_rate:0.0;
    double bank=t->roll,bank_rate=t->roll_rate;
    GuidanceAttitudeLimits pitch=decision_guidance_attitude_limits(t,&cfg->guidance,PHASE_ENTRY_ENERGY,true);
    GuidanceAttitudeLimits roll=decision_guidance_attitude_limits(t,&cfg->guidance,PHASE_ENTRY_ENERGY,false);
    double step=8.0/resolution; /* seconds; refinement is independent of the atmosphere grid */
    double time=0.0,ground=0.0,air=0.0,work=0.0;
    double initial_energy=rotating_specific_energy(t->latitude,t->mean_altitude,t->true_air_speed,p);
    double target_aoa=aoa;
    /* This is a flyable range-retention reference, not a proof of maximum
       range or impossibility. Initial bank and pitch cannot disappear at once.
       Curvature, normal lift, centrifugal gravity and Coriolis are integrated
       in the rotating frame. MM305 must still qualify the alignment route. */
    for(unsigned n=0;n<resolution*256u;n++){
        double altitude=v3_norm(position)-p->radius,speed=v3_norm(velocity);
        if(altitude<target_altitude||!(speed>0.0))return out;
        double sound=planet_atmospheric_speed_of_sound(p,altitude);
        if(!(sound>0.0))return out;
        double upper=cfg->vehicle.maximum_angle_of_attack;
        if(speed/sound<1.0&&cfg->vehicle.terminal_maximum_lift_angle_of_attack>0.0)
            upper=fmin(upper,cfg->vehicle.terminal_maximum_lift_angle_of_attack);
        up=v3_normalized(position);
        double live_gamma=asin(clampd(v3_dot(velocity,up)/speed,-1.0,1.0));
        double desired_gamma=atan2(target_altitude-altitude,fmax(0.0,path-ground));
        double response=decision_pitch_capture_time(t,upper);
        if(!isfinite(response))return out;
        response=fmax(response,step);
        double lat_now=asin(up.z);
        double gravity=p->gravitational_parameter/pow(p->radius+altitude,2.0)-
            p->rotational_speed*p->rotational_speed*(p->radius+altitude)*pow(cos(lat_now),2.0);
        double required=fmax(0.0,((gravity-speed*speed/(p->radius+altitude))*cos(live_gamma)+
            speed*(desired_gamma-live_gamma)/response)/fmax(DBL_EPSILON,cos(bank*DEG2RAD)));
        double best_drag=INFINITY,max_lift=-INFINITY,max_lift_aoa=upper;
        /* Globally search all incidence cells, then refine every lift crossing.
           This follows a descending reference with physically available lift;
           insufficient lift steepens the actual trajectory rather than keeping
           the aircraft on a fictitious geometric line. */
        double previous_lift=0.0;
        for(unsigned j=0;j<=(unsigned)ceil(upper);j++){
            double candidate=fmin(j,upper),lift=0.0,drag=0.0;
            if(!path_reference_forces(g,t,p,cfg,0.0,altitude,speed,candidate,&lift,&drag))return out;
            if(lift>max_lift){max_lift=lift;max_lift_aoa=candidate;}
            if(j&&previous_lift<required&&lift>=required){
                double lo=candidate-1.0,hi=candidate;
                for(unsigned k=0;k<16;k++){
                    double mid=.5*(lo+hi),ml=0.0,md=0.0;
                    if(!path_reference_forces(g,t,p,cfg,0.0,altitude,speed,mid,&ml,&md))return out;
                    if(ml<required)lo=mid;else hi=mid;
                }
                candidate=hi;
                if(!path_reference_forces(g,t,p,cfg,0.0,altitude,speed,candidate,&lift,&drag))return out;
            }
            if(lift>=required&&drag<best_drag){best_drag=drag;target_aoa=candidate;}
            previous_lift=lift;
        }
        if(!isfinite(best_drag))target_aoa=max_lift_aoa;
        path_axis_step(target_aoa,step,pitch,&aoa,&aoa_rate);
        path_axis_step(0.0,step,roll,&bank,&bank_rate);
        Vec3 accel,mid_accel;double power,mid_power;
        if(!path_acceleration(g,t,p,cfg,position,velocity,aoa,bank,&accel,&power))return out;
        Vec3 mid_position=v3_add(position,v3_scale(velocity,.5*step));
        Vec3 mid_velocity=v3_add(velocity,v3_scale(accel,.5*step));
        if(!path_acceleration(g,t,p,cfg,mid_position,mid_velocity,aoa,bank,&mid_accel,&mid_power))return out;
        Vec3 next_position=v3_add(position,v3_scale(mid_velocity,step));
        Vec3 next_velocity=v3_add(velocity,v3_scale(mid_accel,step));
        double next_altitude=v3_norm(next_position)-p->radius;
        double dt=step;
        bool arrived=next_altitude<=target_altitude;
        if(arrived){
            double fraction=(altitude-target_altitude)/(altitude-next_altitude);
            dt*=fraction;
            next_position=v3_add(position,v3_scale(v3_sub(next_position,position),fraction));
            next_velocity=v3_add(velocity,v3_scale(v3_sub(next_velocity,velocity),fraction));
            next_position=v3_scale(v3_normalized(next_position),p->radius+target_altitude);
        }
        up=v3_normalized(mid_position);
        double radial=v3_dot(mid_velocity,up);
        ground+=sqrt(fmax(0.0,v3_dot(mid_velocity,mid_velocity)-radial*radial))*
            p->radius/v3_norm(mid_position)*dt;
        air+=v3_norm(mid_velocity)*dt;work+=mid_power*dt;time+=dt;
        position=next_position;velocity=next_velocity;
        if(arrived){
            double final_lat=asin(position.z/v3_norm(position))*RAD2DEG;
            double energy=rotating_specific_energy(final_lat,target_altitude,v3_norm(velocity),p);
            out.valid=isfinite(work)&&isfinite(energy)&&work>=0.0;
            out.reference_reachable=ground>=path; /* reference range, not a route proof */
            out.projected_ground_path_m=ground;
            out.terminal_latitude_deg=final_lat;
            out.integration_error_j_kg=fabs(initial_energy-energy-work);
            out.ground_path_m=path;out.air_path_m=air;out.modeled_drag_work=work;
            out.terminal_speed_mps=v3_norm(velocity);out.travel_time_s=time;out.target_aoa_deg=target_aoa;
            return out;
        }
    }
    return out; /* no arrival in the finite 2048 s reference horizon */
}

UnpoweredPathProjection decision_unpowered_path_projection(const GuidanceMachine *g,
        const Telemetry *t,const PlanetModel *p,const LandingConfiguration *cfg,
        double target_altitude,double path){
    UnpoweredPathProjection out={0};
    if(!t||!p||!cfg||!isfinite(target_altitude)||!isfinite(path)||!(path>0.0)||
       !isfinite(t->mean_altitude)||!isfinite(t->latitude)||fabs(t->latitude)>90.0||
       !isfinite(t->true_air_speed)||!(t->true_air_speed>0.0)||
       !isfinite(t->angle_of_attack)||!isfinite(t->sideslip)||!isfinite(t->roll)||
       !isfinite(t->flight_path_angle)||!(p->radius+t->mean_altitude>0.0)||
       !(p->radius+target_altitude>0.0)||!(p->gravitational_parameter>0.0)||
       !isfinite(p->rotational_speed)||!(cfg->vehicle.estimated_ballistic_coefficient>0.0)||
       (g&&g->entry_terminal_model&&(!(t->mass>0.0)||
        !g->entry_terminal_model->replay_validated)))return out;
    /* Refine independently of atmosphere spacing. Relative energy tolerance
       is a quadrature budget, charged against admission, never credited. */
    double tolerance=1e-4*fmax(1.0,fabs(entry_remaining_specific_energy(
        t->latitude,t->mean_altitude,t->true_air_speed,cfg->site.latitude,
        target_altitude,0.0,p)));
    unsigned n=32;
    out=path_reference_integrate(g,t,p,cfg,target_altitude,path,n);
    while(out.valid&&n<4096){
        n*=2;
        UnpoweredPathProjection finer=path_reference_integrate(g,t,p,cfg,target_altitude,path,n);
        if(!finer.valid)return finer;
        finer.integration_error_j_kg=fmax(finer.integration_error_j_kg,
            fabs(finer.modeled_drag_work-out.modeled_drag_work));
        out=finer;
        if(out.integration_error_j_kg<=tolerance)return out;
    }
    out.valid=false; /* unconverged results cannot admit an inlet */
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
    /* While native MM305 owns a committed fixed-HAC route, use its monotone
       analytic station for time/vertical reserve.  This keeps prediction tied to
       exactly the geometry the tracker is flying and removes the legacy lead/cone
       transition caches from the decision path. */
    bool fixed_hac=g&&g->mm305_route_committed&&g->mm305_route.valid&&
        !g->mm305_hac_exit_reached&&!g->final_approach_captured;
    double committed_path=0.0;
    if(fixed_hac){
        double route_remaining=taem_route_remaining_at_index(
            &g->mm305_route,g->mm305_route_cursor);
        if(isfinite(route_remaining))
            committed_path=route_remaining+
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
