/* A HAC point is not a reachable Entry program. Search the complete decelerating
 * first leg and one opposite turn, including roll response and the same live
 * inlet contract. This runs exclusively on the existing prediction worker. */
static double entry_topology_energy_target_speed(const PlanetModel*p,
        const LandingConfiguration*cfg){
    if(!p||!cfg)return NAN;
    return entry_taem_speed_target(&cfg->vehicle,&cfg->guidance,p);
}

static double entry_topology_interface_eta(const Telemetry*t,
        const TaemInterfaceTarget*inlet,const LandingConfiguration*cfg,
        double target_speed){
    if(!t||!inlet||!inlet->valid||!cfg||!isfinite(target_speed)||
       !(target_speed>0.0)||!isfinite(inlet->along_track)||
       !isfinite(inlet->cross_track)||!isfinite(t->runway_along_track)||
       !isfinite(t->runway_cross_track))return NAN;
    const GuidanceSettings*s=&cfg->guidance;
    double distance=hypot(inlet->along_track-t->runway_along_track,
        inlet->cross_track-t->runway_cross_track);
    if(!(t->horizontal_speed>DBL_EPSILON))return NAN;

    double slope=s->taem_glide_slope*DEG2RAD;
    double target_horizontal=target_speed*cos(slope);
    if(!(target_horizontal>DBL_EPSILON))return NAN;

    /* Constant-deceleration first-order transit estimate. The factor 1/2 is
       the trapezoidal mean of endpoint velocities, not a tuned policy value. */
    double average_horizontal=.5*(t->horizontal_speed+target_horizontal); /* decision-literal-ok: trapezoidal mean */
    return average_horizontal>DBL_EPSILON?distance/average_horizontal:NAN;
}

static double entry_topology_shadow_refresh_interval(const Telemetry*t,
        const GuidanceMachine*g,const PlanetModel*p,
        const LandingConfiguration*cfg){
    double interval=async_prediction_cooldown_seconds(
        cfg?cfg->guidance.prediction_interval:NAN,0.0);
    if(!t||!g||!p||!cfg||!g->taem_interface_target.valid)return interval;

    double target_speed=isfinite(g->taem_interface_target.speed)&&
        g->taem_interface_target.speed>0.0?
            g->taem_interface_target.speed:
            entry_topology_energy_target_speed(p,cfg);
    double eta=entry_topology_interface_eta(t,&g->taem_interface_target,
        cfg,target_speed);
    if(!isfinite(eta)&&t->horizontal_speed>DBL_EPSILON)
        eta=fmax(0.0,t->range_to_site)/t->horizontal_speed;
    if(!isfinite(eta))return interval;

    /* The shadow need not repeat the full route solve until its delay search
       has advanced by one of that search's own temporal cells.  This only
       spaces redundant previews; the live prediction worker and all route and
       capture contracts retain their configured behavior. */
    double earliest=fmax(0.0,cfg->guidance.s_turn_minimum_leg_duration);
    double latest=fmax(earliest,eta);
    double cell=(latest-earliest)/
        (double)(ENTRY_TOPOLOGY_DELAY_SAMPLE_COUNT-1);
    if(isfinite(cell)&&cell>0.0)interval=fmax(interval,cell);
    return interval;
}

static bool entry_topology_terminal_endpoint_fit(const GuidanceSettings*s,const VehicleProfile*v,
        const PlanetModel*p,double along,double cross,double course,double speed,double horizontal,
        double lift_accel,double bank_effectiveness,double bank_limit_deg,double station,
        double runway_course,double target_course,double target_speed,double final_sign,
        double*endpoint_error_out,double*radius_out){
    (void)s;(void)p;(void)speed;(void)target_speed;
    if(endpoint_error_out)*endpoint_error_out=INFINITY;
    if(radius_out)*radius_out=INFINITY;
    if(!v||!isfinite(along)||!isfinite(cross)||!isfinite(course)||
       !isfinite(horizontal)||!(horizontal>0.0)||!isfinite(lift_accel)||
       !(lift_accel>0.0)||!isfinite(bank_effectiveness)||
       !(bank_effectiveness>0.0)||!isfinite(bank_limit_deg)||
       !(bank_limit_deg>0.0)||!isfinite(target_course))return false;

    double side=final_sign>=0.0?1.0:-1.0;
    double tangent_turn=norm_signed_deg(target_course-course);
    if(side*tangent_turn<=0.0)return false;

    double usable_bank=fmin(fabs(bank_limit_deg),v->maximum_bank_angle);
    double effective_bank=fmin(usable_bank*bank_effectiveness,nextafter(90.0,0.0)); /* decision-literal-ok: tangent singularity */
    double lateral=lift_accel*fabs(sin(effective_bank*DEG2RAD));
    if(!(lateral>DBL_EPSILON))return false;
    double minimum_radius=horizontal*horizontal/lateral;

    double psi0=norm_signed_deg(course-runway_course)*DEG2RAD;
    double psi1=norm_signed_deg(target_course-runway_course)*DEG2RAD;
    double ua=(sin(psi1)-sin(psi0))/side;
    double uc=(cos(psi0)-cos(psi1))/side;
    double norm2=ua*ua+uc*uc;
    if(!(norm2>DBL_EPSILON))return false;

    double ba=station-along,bc=-cross;
    double ideal_radius=(ba*ua+bc*uc)/norm2;
    if(!(ideal_radius>0.0))return false;

    /* If the geometrically exact circle is tighter than available lateral
       authority, fly the minimum physically reachable radius and judge the
       resulting endpoint against the same MM304 handoff point contract. */
    double radius=fmax(ideal_radius,minimum_radius);
    double endpoint_along=along+radius*ua;
    double endpoint_cross=cross+radius*uc;
    double endpoint_error=hypot(endpoint_along-station,endpoint_cross);

    if(endpoint_error_out)*endpoint_error_out=endpoint_error;
    if(radius_out)*radius_out=radius;
    return endpoint_error<=1000.0; /* decision-literal-ok: explicit MM304 1 km mission contract */
}


/* Mirror live MM304's bounded actuator roll-through proof. The instantaneous
 * fixed-pose circle may still be outside its tube while the orbiter rolls onto
 * final-turn bank. Project only that short transient, then apply the same
 * endpoint-fit gate; this never certifies TAEM handoff. */
static bool entry_topology_rollthrough_endpoint_fit(const GuidanceSettings*s,
        const VehicleProfile*v,const PlanetModel*p,double along,double cross,
        double course,double speed,double horizontal,double altitude,double vertical_speed,
        double lift_accel,double drag_accel,double bank_effectiveness,double bank_limit_deg,
        double current_bank,double current_bank_rate,double current_course_rate,
        double station,double runway_course,double target_course,double target_speed,
        double final_sign,double*endpoint_error_out,double*radius_out,double*lead_time_out){
    (void)current_course_rate;
    if(endpoint_error_out)*endpoint_error_out=INFINITY;
    if(radius_out)*radius_out=INFINITY;
    if(lead_time_out)*lead_time_out=NAN;
    if(!s||!v||!p||!isfinite(along)||!isfinite(cross)||!isfinite(course)||
       !isfinite(speed)||!(speed>0.0)||!isfinite(horizontal)||!(horizontal>0.0)||
       !isfinite(altitude)||!isfinite(vertical_speed)||!isfinite(lift_accel)||
       !(lift_accel>0.0)||!isfinite(drag_accel)||drag_accel<0.0||
       !isfinite(bank_effectiveness)||!(bank_effectiveness>0.0)||
       !isfinite(current_bank)||!isfinite(bank_limit_deg))return false;

    double side=final_sign>=0.0?1.0:-1.0;
    double bank_limit=fmin(fabs(bank_limit_deg),v->maximum_bank_angle);
    if(!(bank_limit>0.0)||!(s->entry_roll_rate>0.0)||
       !(s->entry_roll_acceleration>0.0)||!(s->guidance_rate>0.0))
        return false;

    double target_bank=side*bank_limit;
    double bank_error=norm_signed_deg(current_bank-target_bank);
    double bank_rate=isfinite(current_bank_rate)?current_bank_rate:0.0;

    double lead=decision_bounded_capture_time(bank_error,bank_rate,
        s->entry_roll_acceleration,0.0);
    double rate_bound=fabs(bank_error)/s->entry_roll_rate;
    lead=fmax(lead,rate_bound);
    if(!isfinite(lead)||lead<0.0)return false;
    if(lead_time_out)*lead_time_out=lead;

    double control_dt=1.0/s->guidance_rate;
    size_t steps=lead>0.0?(size_t)ceil(lead/control_dt):1u;
    if(steps==0u)steps=1u;
    double dt=lead/(double)steps;

    double bank=current_bank;
    double tas0=speed,tas=tas0;
    double hs0=horizontal,hs=hs0;
    double rho0=planet_atmospheric_density(p,altitude);

    for(size_t i=0;i<steps;i++){
        double error=norm_signed_deg(target_bank-bank);
        double stopping_rate=sqrt(2.0*s->entry_roll_acceleration*fabs(error)); /* decision-literal-ok: double-integrator stopping law */
        double desired_rate=fabs(error)>DBL_EPSILON?
            copysign(fmin(s->entry_roll_rate,stopping_rate),error):0.0;
        bank_rate+=clampd(desired_rate-bank_rate,
            -s->entry_roll_acceleration*dt,s->entry_roll_acceleration*dt);
        bank_rate=clampd(bank_rate,-s->entry_roll_rate,s->entry_roll_rate);
        double next_bank=bank+bank_rate*dt;
        if(error*norm_signed_deg(target_bank-next_bank)<=0.0){
            next_bank=target_bank;
            bank_rate=0.0;
        }
        bank=clampd(next_bank,-v->maximum_bank_angle,v->maximum_bank_angle);

        double projected_altitude=altitude+vertical_speed*((double)i+0.5)*dt;
        double rho=planet_atmospheric_density(p,projected_altitude);
        double density_scale=rho0>DBL_EPSILON?rho/rho0:1.0;
        double speed_scale=tas0>DBL_EPSILON?(tas/tas0)*(tas/tas0):1.0;
        double force_scale=fmax(0.0,density_scale*speed_scale);

        double effective_bank=clampd(bank*bank_effectiveness,
            -nextafter(90.0,0.0),nextafter(90.0,0.0)); /* decision-literal-ok: tangent singularity */
        double lateral=lift_accel*force_scale*sin(effective_bank*DEG2RAD);
        double course_rate=lateral/fmax(hs,DBL_EPSILON)*RAD2DEG;
        double mid_course=course+course_rate*dt*0.5; /* decision-literal-ok: midpoint integration */
        double relative=norm_signed_deg(mid_course-runway_course)*DEG2RAD;
        double surface_horizontal=hs*p->radius/(p->radius+projected_altitude);
        along+=surface_horizontal*cos(relative)*dt;
        cross+=surface_horizontal*sin(relative)*dt;
        course=norm_deg(course+course_rate*dt);

        tas-=drag_accel*force_scale*dt;
        if(tas<v->minimum_safe_speed)return false;
        hs=hs0*(tas/tas0);
        if(!(hs>0.0))return false;
    }

    double final_rho=planet_atmospheric_density(p,altitude+vertical_speed*lead);
    double final_density_scale=rho0>DBL_EPSILON?final_rho/rho0:1.0;
    double final_speed_scale=(tas/tas0)*(tas/tas0);
    double final_force_scale=fmax(0.0,final_density_scale*final_speed_scale);
    return entry_topology_terminal_endpoint_fit(s,v,p,along,cross,course,tas,hs,
        lift_accel*final_force_scale,bank_effectiveness,bank_limit,station,
        runway_course,target_course,target_speed,final_sign,
        endpoint_error_out,radius_out);
}


static unsigned entry_topology_capture_failure_flags(unsigned veto){
    unsigned flags=0;
    if(veto&1u)flags|=ENTRY_TOPOLOGY_FAILURE_CAPTURE_SPEED;
    if(veto&2u)flags|=ENTRY_TOPOLOGY_FAILURE_CAPTURE_SPATIAL;
    if(veto&4u)flags|=ENTRY_TOPOLOGY_FAILURE_CAPTURE_COURSE;
    if(veto&8u)flags|=ENTRY_TOPOLOGY_FAILURE_CAPTURE_ALTITUDE;
    if(veto&16u)flags|=ENTRY_TOPOLOGY_FAILURE_CAPTURE_ENERGY;
    if(veto&32u)flags|=ENTRY_TOPOLOGY_FAILURE_CAPTURE_FPA;
    if(veto&64u)flags|=ENTRY_TOPOLOGY_FAILURE_CAPTURE_STRUCTURAL;
    return flags;
}


static EntryTopologyPlan entry_topology_trial(VehicleState initial,const Telemetry*t,const GuidanceMachine*base,
        const PlanetModel*p,const AerodynamicEnvelope*env,const TrajectoryCalibrationModel*cal,
        const LandingConfiguration*cfg,double side,double first_bank,double turn_bank,double outbound_offset,
        double first_aoa,double turn_aoa,double reversal_delay,double forced_terminal_delay){
    EntryTopologyPlan best={.cost=INFINITY};
    (void)outbound_offset;
    if(!base||!base->taem_interface_target.valid||
       !isfinite(base->taem_interface_target.along_track)||
       !isfinite(base->taem_interface_target.cross_track)||
       !isfinite(base->taem_interface_target.course))return best;
    const VehicleProfile*v=&cfg->vehicle;const GuidanceSettings*settings=&cfg->guidance;
    const TaemInterfaceTarget*published_inlet=&base->taem_interface_target;
    double inlet_along=published_inlet->along_track;
    double inlet_cross=published_inlet->cross_track;
    /* Every topology candidate must obey the same configured bank envelope that
       live guidance will fly. This prevents a nominal solve from depending on a
       60/70 deg turn when the mission authorizes only +/-45 deg. */
    first_bank=clampd(first_bank,0.0,v->maximum_bank_angle);
    turn_bank=clampd(turn_bank,0.0,v->maximum_bank_angle);
    TaemSpeedEnvelope taem_speed_envelope=decision_taem_speed_envelope(
        v,p,settings->taem_interface_altitude);
    double taem_speed_min=taem_speed_envelope.valid?
        taem_speed_envelope.minimum_speed_mps:fmax(v->minimum_safe_speed,v->final_approach_speed);
    double taem_speed_max=taem_speed_envelope.valid?
        taem_speed_envelope.maximum_speed_mps:taem_speed_min;
    double taem_speed_nominal=base&&base->taem_interface_target.valid&&
        isfinite(base->taem_interface_target.speed)&&base->taem_interface_target.speed>0.0?
        base->taem_interface_target.speed:entry_topology_energy_target_speed(p,cfg);
    double aoa_switch_speed=fmax(taem_speed_max*1.08,
        taem_speed_nominal*1.35);
    double runway_course=norm_deg(cfg->site.runway_heading);
    double requested_course=norm_deg(published_inlet->course);
    /* Topology candidates are judged against the exact MM304 inlet currently
       published by live guidance.  Search offsets never replace its geometry. */
    double target_course=requested_course;
    double selected_outbound_offset=norm_signed_deg(target_course-runway_course);
    if(fabs(selected_outbound_offset)>DBL_EPSILON&&
       (-side)*selected_outbound_offset<=0.0)
        return best;
    VehicleState state=initial;
    GuidanceMachine reference;if(base)reference=*base;else guidance_machine_init(&reference);
    reference.diagnostic_shadow=true;
    PredictorGuidance response={0};response.actual_bank=norm_signed_deg(t->roll);
    response.actual_bank_rate=isfinite(t->roll_rate)?t->roll_rate:0.0;
    response.actual_aoa=t->angle_of_attack;
    response.actual_aoa_rate=t->has_angle_of_attack_rate?t->angle_of_attack_rate:0.0;
    AeroContext ctx={.p=p,.e=env,.c=cal,.v=v,.bank=response.actual_bank,
        .aoa=response.actual_aoa,.mass=initial.mass,.ut=initial.ut};
    double reversal_ut=NAN,reversal_along=NAN,reversal_cross=NAN,reversal_altitude=NAN,reversal_speed=NAN,reversal_course=NAN;
    double terminal_turn_ut=NAN,terminal_turn_along=NAN,terminal_turn_cross=NAN;
    double terminal_turn_altitude=NAN,terminal_turn_speed=NAN,terminal_turn_course=NAN;
    double terminal_turn_radius=INFINITY,terminal_turn_geometry_error=INFINITY;
    double gravity0=planet_surface_gravity(p),peak_q=0,peak_g=0;
    bool reversal_recorded=false,terminal_turn_started=false,heading_locked=false;
    unsigned propagation_failure_flags=ENTRY_TOPOLOGY_FAILURE_NONE;
    double heading_lock_ut=NAN,heading_lock_along=NAN,heading_lock_cross=NAN;
    double heading_lock_altitude=NAN,heading_lock_speed=NAN,heading_lock_course=NAN,heading_lock_bank=NAN;
    /* Preserve the closest terminal-turn setup even when a coarse trial does not
       enter the capture tube.  Without a finite near-miss seed, coarse reversal
       timing errors are discarded as INFINITY and the refinement stage has nothing
       to optimize around.  These snapshots are never installable by themselves. */
    double setup_seed_metric=INFINITY,setup_seed_ut=NAN,setup_seed_along=NAN,setup_seed_cross=NAN;
    double setup_seed_altitude=NAN,setup_seed_speed=NAN,setup_seed_course=NAN;
    double setup_seed_radius=INFINITY,setup_seed_geom=INFINITY;
    const double dt=.5; /* numerical integration step */

    GeoPoint runway={cfg->site.latitude,cfg->site.longitude,cfg->site.altitude};
    double rh=cfg->site.runway_heading*DEG2RAD;
    TaemInterfaceTarget lock_target=*published_inlet;
    if(!isfinite(lock_target.response_time)||lock_target.response_time<=0.0){
        double full_bank=fabs(v->maximum_bank_angle);
        double rate_time=settings->entry_roll_rate>DBL_MIN?
            full_bank/settings->entry_roll_rate:0.0;
        double accel_time=settings->entry_roll_acceleration>DBL_MIN?
            sqrt(2.0*full_bank/settings->entry_roll_acceleration):0.0;
        lock_target.response_time=fmax(rate_time,accel_time);
    }
    if(!isfinite(lock_target.acquisition_lead)||lock_target.acquisition_lead<=0.0)
        lock_target.acquisition_lead=
            fmax(0.0,t->horizontal_speed)*lock_target.response_time;

    double trial_eta=entry_topology_interface_eta(t,published_inlet,cfg,
        taem_speed_nominal);
    if(!isfinite(trial_eta)){
        if(!(t->horizontal_speed>DBL_EPSILON))return best;
        trial_eta=fmax(0.0,t->range_to_site)/t->horizontal_speed;
    }
    double latest_event=fmax(reversal_delay,
        isfinite(forced_terminal_delay)?forced_terminal_delay:0.0);
    double trial_horizon=fmax(trial_eta,latest_event)+
        lock_target.response_time+
        fmax(0.0,settings->s_turn_minimum_leg_duration);

    for(double elapsed=0;elapsed<trial_horizon;elapsed+=dt){
        AtmosState a;Telemetry synthetic=guidance_shadow_telemetry(t,state,&response,&ctx,p,cfg,&a);
        GeoPoint geo={synthetic.latitude,synthetic.longitude,synthetic.mean_altitude};
        double along,cross;runway_coordinates(geo,runway,cfg->site.runway_heading,p->radius,&along,&cross);
        double course=surface_course(state.position,state.velocity,planet_rotation_vector(p),p->north_axis,t->ground_track_heading);
        double target_course_error=norm_signed_deg(target_course-course);
        double requested_bank=side*first_bank;
        if(!reversal_recorded&&elapsed>=reversal_delay){
            /* The scheduled first sign change is the energy-management reversal,
               not automatically the final perpendicular turn.  Live MM304 executes
               this committed deadline even when terminal geometry is not ready, then
               carries the acquired crossrange on the opposite/runway-parallel side
               until the fixed-pose arc becomes flyable.  Preserve the same two-event
               topology in the shadow so a valid plan can publish distinct reversal
               and terminal-turn milestones. */
            reversal_recorded=true;
            reversal_ut=state.ut;
            reversal_along=along;reversal_cross=cross;reversal_altitude=geo.altitude;
            reversal_speed=a.speed;reversal_course=course;
        }
        if(reversal_recorded&&!terminal_turn_started){
            double final_sign=-side;
            double horizontal=fmax(80.0,synthetic.horizontal_speed);
            double limit=fmin(turn_bank,bank_limit(&a,p,v,v->maximum_bank_angle));
            double endpoint_error=INFINITY,endpoint_radius=INFINITY;
            bool endpoint_ready=entry_topology_terminal_endpoint_fit(settings,v,p,
                along,cross,course,a.speed,horizontal,a.lift_accel,cal->bank_effectiveness,
                limit,inlet_along,runway_course,target_course,taem_speed_nominal,final_sign,
                &endpoint_error,&endpoint_radius);
            double setup_endpoint_error=endpoint_error,setup_endpoint_radius=endpoint_radius;

            /* Live MM304 can release the later final event on a bounded actuator
               roll-through proof. Mirror that proof here so preflight does not reject
               a path merely because the instantaneous circle becomes admissible a few
               seconds after roll begins. The strict capture contract is unchanged. */
            if(!endpoint_ready&&isfinite(endpoint_error)){
                double rollout_error=INFINITY,rollout_radius=INFINITY,rollout_lead=NAN;
                bool rollout_ready=entry_topology_rollthrough_endpoint_fit(settings,v,p,
                    along,cross,course,a.speed,horizontal,geo.altitude,synthetic.vertical_speed,
                    a.lift_accel,a.drag_accel,cal->bank_effectiveness,limit,
                    response.actual_bank,response.actual_bank_rate,synthetic.course_rate,
                    inlet_along,runway_course,target_course,taem_speed_nominal,final_sign,
                    &rollout_error,&rollout_radius,&rollout_lead);
                if(isfinite(rollout_error)&&rollout_error<setup_endpoint_error){
                    setup_endpoint_error=rollout_error;setup_endpoint_radius=rollout_radius;
                }
                if(rollout_ready){
                    endpoint_ready=true;endpoint_error=rollout_error;endpoint_radius=rollout_radius;
                }
            }

            /* Keep the best usable instantaneous or projected roll-through setup as
               a refinement seed after the first shaping reversal.  A projected near
               miss is still not executable: only endpoint_ready may start the final turn. */
            if(isfinite(setup_endpoint_error)&&
               geo.altitude>cfg->site.altitude&&
               a.speed>=v->minimum_safe_speed){
                double setup_metric=setup_endpoint_error;
                if(setup_metric<setup_seed_metric){
                    setup_seed_metric=setup_metric;setup_seed_ut=state.ut;
                    setup_seed_along=along;setup_seed_cross=cross;
                    setup_seed_altitude=geo.altitude;
                    setup_seed_speed=a.speed;setup_seed_course=course;
                    setup_seed_radius=setup_endpoint_radius;
                    setup_seed_geom=setup_endpoint_error;
                }
            }

            /* Mirror entry_topology_guidance's post-shaping parallel stage.  The
               first reversal brings course back toward the runway heading and may
               then unload while the vehicle advances downstream.  The later final
               turn stays on this same side and begins only when the fixed-pose
               endpoint gate is actually satisfied. */
            double efficiency=fmax(cal->bank_effectiveness,DBL_MIN);
            requested_bank=taem_course_rate_bank_command(
                horizontal,a.lift_accel,efficiency,limit,
                course,runway_course,lock_target.response_time);

            /* Search-only dynamic timing candidates may deliberately begin the
               second/final event before the frozen-current-radius endpoint tube is
               satisfied.  This does not certify anything: the remainder of this same
               high-fidelity shadow must still fly the turn, settle heading, and pass
               the ordinary strict TAEM/HAC capture contract.  The first shaping
               reversal remains mandatory and distinct. */
            bool forced_terminal_due=isfinite(forced_terminal_delay)&&
                forced_terminal_delay>=reversal_delay&&elapsed>=forced_terminal_delay;
            if(endpoint_ready||forced_terminal_due){
                terminal_turn_started=true;terminal_turn_ut=state.ut;
                terminal_turn_along=along;terminal_turn_cross=cross;
                terminal_turn_altitude=geo.altitude;terminal_turn_speed=a.speed;
                terminal_turn_course=course;terminal_turn_radius=endpoint_radius;
                terminal_turn_geometry_error=endpoint_error;
            }
        }
        if(terminal_turn_started&&!heading_locked&&fabs(target_course_error)<=8.0){
            heading_locked=true;
            heading_lock_ut=state.ut;heading_lock_along=along;heading_lock_cross=cross;
            heading_lock_altitude=geo.altitude;heading_lock_speed=a.speed;heading_lock_course=course;
            heading_lock_bank=ctx.bank;
        }

        if(terminal_turn_started&&!heading_locked){
            /* Propagate the same dynamic fixed-pose circle used by live MM304.
               Recompute the required radius as speed/course evolve, then invert
               current lift for the bank that preserves that radius. */
            double limit=fmin(turn_bank,bank_limit(&a,p,v,v->maximum_bank_angle));
            double pose_error=INFINITY,pose_radius=INFINITY;
            (void)entry_topology_terminal_endpoint_fit(settings,v,p,
                along,cross,course,a.speed,fmax(80.0,synthetic.horizontal_speed),
                a.lift_accel,cal->bank_effectiveness,limit,inlet_along,runway_course,
                target_course,taem_speed_nominal,-side,&pose_error,&pose_radius);
            double efficiency=clampd(cal->bank_effectiveness,.35,1.8);
            double horizontal=fmax(80.0,synthetic.horizontal_speed);
            double bank_mag=0.0;
            if(isfinite(pose_radius)&&pose_radius>=1000.0&&a.lift_accel>.01){
                double lateral=horizontal*horizontal/pose_radius;
                double max_ratio=sin(clampd(limit*efficiency,0.0,89.0)*DEG2RAD);
                bank_mag=asin(clampd(lateral/a.lift_accel,0.0,max_ratio))/efficiency*RAD2DEG;
            }else{
                bank_mag=fabs(taem_course_rate_bank_command(horizontal,a.lift_accel,
                    efficiency,limit,course,target_course,lock_target.response_time));
            }
            requested_bank=-side*clampd(bank_mag,0.0,limit);
            double lateral=a.lift_accel*sin(clampd(fabs(requested_bank)*efficiency,0.0,89.0)*DEG2RAD);
            double radius=lateral>.05?horizontal*horizontal/lateral:INFINITY;
            if(!isfinite(terminal_turn_radius)&&isfinite(radius))terminal_turn_radius=radius;
        }else if(terminal_turn_started){
            double desired_heading=taem_interface_line_heading(along,cross,cfg->site.runway_heading,
                &lock_target);
            double correction_limit=fmin(18.0,bank_limit(&a,p,v,v->maximum_bank_angle));
            requested_bank=taem_course_rate_bank_command(synthetic.horizontal_speed,a.lift_accel,
                cal->bank_effectiveness,correction_limit,course,desired_heading,
                lock_target.response_time);
        }
        double terminal_floor=entry_terminal_turn_aoa_floor(a.speed,a.q,v);
        /* Match entry_topology_guidance(): the high-drag final-S-turn AoA
           is a low-speed regime, not something that begins at the first shaping
           reversal hundreds of kilometres upstream. */
        double requested_aoa=fmax(a.speed>aoa_switch_speed?first_aoa:turn_aoa,terminal_floor);
        /* Match the executable MM304 energy-preservation policy. The topology search
           used to propagate 18-26 deg incidence all the way through the terminal
           geometry, so every otherwise-good -8 km capture arrived hundreds of m/s
           slow even though live guidance unloads once thermal/q authority permits. */
        bool preserve_terminal_energy=reversal_recorded&&
            a.speed<settings->taem_force_handoff_speed+350.0&&
            a.q>=700.0&&a.q<v->maximum_dynamic_pressure*.85&&
            a.non_gravity/fmax(.1,gravity0)<v->maximum_g_load*.85;
        double extended_aoa_speed_floor=aoa_switch_speed;
        bool extended_final_turn=reversal_recorded&&turn_aoa>v->maximum_angle_of_attack+1e-6&&
            !heading_locked&&a.speed>extended_aoa_speed_floor&&
            a.q<v->maximum_dynamic_pressure*.88&&
            a.non_gravity/fmax(.1,gravity0)<v->maximum_g_load*.90;
        if(preserve_terminal_energy&&!extended_final_turn)
            requested_aoa=fmin(requested_aoa,fmax(15.0,terminal_floor));
        requested_aoa=clampd(requested_aoa,0.0,extended_final_turn?
            entry_final_s_turn_aoa_ceiling(v):v->maximum_angle_of_attack);

        for(unsigned step=0;step<5;step++){
            synthetic.roll=response.actual_bank;synthetic.roll_rate=response.actual_bank_rate;
            synthetic.angle_of_attack=response.actual_aoa;synthetic.pitch=synthetic.flight_path_angle+response.actual_aoa;
            GuidanceCommand command=guidance_entry_reference_step(&reference,&synthetic,v,settings,
                requested_bank,requested_aoa,dt/5.0);
            attitude_step(&response,norm_signed_deg(command.target_roll),command.target_aoa,&a,v,settings,dt/5.0,cal->physics);
        }
        ctx.bank=response.actual_bank;ctx.aoa=response.actual_aoa;ctx.ut=state.ut;
        peak_q=fmax(peak_q,a.q);peak_g=fmax(peak_g,a.non_gravity/fmax(.1,gravity0));
        propagation_failure_flags=ENTRY_TOPOLOGY_FAILURE_NONE;
        if(!isfinite(geo.altitude)||!isfinite(a.speed))
            propagation_failure_flags|=ENTRY_TOPOLOGY_FAILURE_NONFINITE_STATE;
        if(isfinite(geo.altitude)&&geo.altitude<cfg->site.altitude)
            propagation_failure_flags|=ENTRY_TOPOLOGY_FAILURE_ALTITUDE_FLOOR;
        if(isfinite(a.speed)&&a.speed<v->minimum_safe_speed)
            propagation_failure_flags|=ENTRY_TOPOLOGY_FAILURE_SPEED_FLOOR;
        if(peak_q>v->maximum_dynamic_pressure)
            propagation_failure_flags|=ENTRY_TOPOLOGY_FAILURE_DYNAMIC_PRESSURE;
        if(peak_g>v->maximum_g_load)
            propagation_failure_flags|=ENTRY_TOPOLOGY_FAILURE_G_LOAD;
        if(propagation_failure_flags!=ENTRY_TOPOLOGY_FAILURE_NONE)break;

        double position_error=hypot(along-inlet_along,cross-inlet_cross);
        bool candidate_window=heading_locked&&terminal_turn_started&&
            state.ut>=terminal_turn_ut;
        if(candidate_window){
            Vector3 up=vnorm(state.position,v3(0,0,1));
            double vertical=vdot(a.air_velocity,up);
            double horizontal=sqrt(fmax(0.0,a.speed*a.speed-vertical*vertical));
            double fpa=asin(clampd(vertical/fmax(1.0,a.speed),-1,1))*RAD2DEG;

            double usable_bank=bank_limit(&a,p,v,v->maximum_bank_angle);
            double effective_bank=clampd(
                usable_bank*cal->bank_effectiveness,
                -nextafter(90.0,0.0),nextafter(90.0,0.0));
            double lateral=a.lift_accel*fabs(sin(effective_bank*DEG2RAD));
            double minimum_turn_radius=lateral>DBL_MIN?
                horizontal*horizontal/lateral:INFINITY;
            double endpoint_error=position_error;
            double heading_debt=isfinite(minimum_turn_radius)?
                minimum_turn_radius*fabs(target_course_error)*DEG2RAD:
                INFINITY;

            /*
             * Radius is determined by demonstrated turn authority; configured
             * HAC radius is only a preferred geometry.  Feasibility is proved
             * by the same TAEM alignment model used by the live handoff.
             */
            double terminal_radius=isfinite(minimum_turn_radius)?
                fmax(settings->hac_radius,minimum_turn_radius):INFINITY;
            double maneuver_path=0.0,maneuver_slope=0.0;
            bool maneuver_ok=isfinite(terminal_radius)&&
                taem_alignment_maneuver_feasible(
                    geo.altitude,a.speed,terminal_radius,
                    minimum_turn_radius,a.drag_accel,
                    fabs(selected_outbound_offset),p,&cfg->site,v,settings,
                    &maneuver_path,&maneuver_slope);

            double final_altitude=cfg->site.altitude+
                settings->final_approach_distance*
                tan(settings->final_glide_slope*DEG2RAD);
            double terminal_path=settings->final_approach_distance+
                (maneuver_ok?maneuver_path:0.0);
            double minimum_alignment_altitude=final_altitude+
                fmax(0.0,terminal_path-settings->final_approach_distance)*
                tan(settings->taem_glide_slope*DEG2RAD);
            double kinetic_height=fmax(0.0,
                a.speed*a.speed-v->final_approach_speed*
                v->final_approach_speed)/(2.0*gravity0);
            double reserve=geo.altitude+kinetic_height-
                minimum_alignment_altitude;

            double target_bank=-side*turn_bank;
            double bank_error=norm_signed_deg(target_bank-response.actual_bank);
            double response_time=decision_bounded_capture_time(
                bank_error,response.actual_bank_rate,
                settings->entry_roll_acceleration,0.0);
            if(settings->entry_roll_rate>DBL_MIN)
                response_time=fmax(response_time,
                    fabs(bank_error)/settings->entry_roll_rate);
            if(!isfinite(response_time)||response_time<0.0)
                response_time=0.0;
            double acquisition_lead=horizontal*response_time;
            double remaining_path=maneuver_ok?maneuver_path:
                settings->final_approach_distance;

            GeoPoint alignment=local_point(runway,
                inlet_along*sin(rh)+inlet_cross*cos(rh),
                inlet_along*cos(rh)-inlet_cross*sin(rh),p->radius,geo.altitude);
            TaemInterfaceTarget inlet=*published_inlet;
            inlet.valid=true;
            inlet.energy_qualified=false;
            inlet.selected_ut=initial.ut;
            inlet.arrival_ut=state.ut+response_time+
                remaining_path/fmax(horizontal,DBL_EPSILON);
            inlet.quality_score=INFINITY;
            inlet.acquisition_lead=acquisition_lead;
            inlet.remaining_path=remaining_path;
            inlet.response_time=response_time;
            inlet.specific_energy=rotating_specific_energy(
                alignment.latitude,inlet.altitude,inlet.speed,p);

            Telemetry capture_state=synthetic;
            capture_state.latitude=geo.latitude;
            capture_state.longitude=geo.longitude;
            capture_state.mean_altitude=geo.altitude;
            capture_state.range_to_site=hypot(along,cross);
            capture_state.runway_along_track=along;
            capture_state.runway_cross_track=cross;
            capture_state.true_air_speed=a.speed;
            capture_state.horizontal_speed=horizontal;
            capture_state.vertical_speed=vertical;
            capture_state.flight_path_angle=fpa;
            capture_state.dynamic_pressure=a.q;
            capture_state.g_force=a.non_gravity/gravity0;
            capture_state.stall_fraction_is_measured=false;
            capture_state.bank_effectiveness=cal->bank_effectiveness;

            TaemInterfaceCapture capture=entry_taem_interface_capture(
                &inlet,&capture_state,course,p,cfg);

            unsigned failure_flags=ENTRY_TOPOLOGY_FAILURE_NONE;
            TaemHandoffContract handoff_contract=
                taem_handoff_contract(&cfg->guidance);
            if(position_error>handoff_contract.horizontal_radius_m)
                failure_flags|=ENTRY_TOPOLOGY_FAILURE_POSITION;
            if(fabs(target_course_error)>
               handoff_contract.perpendicular_heading_half_width_deg)
                failure_flags|=ENTRY_TOPOLOGY_FAILURE_COURSE;
            if(!maneuver_ok)
                failure_flags|=ENTRY_TOPOLOGY_FAILURE_MANEUVER_PATH;
            failure_flags|=entry_topology_capture_failure_flags(capture.veto);
            bool valid=capture.ready&&
                failure_flags==ENTRY_TOPOLOGY_FAILURE_NONE;
            inlet.energy_qualified=valid;

            /*
             * Cost is telemetry only.  Keep it dimensionless as the worst
             * normalized mission-contract residual; candidate ordering below is
             * lexicographic and never sums these terms.
             */
            double position_residual=position_error/
                handoff_contract.horizontal_radius_m;
            double course_residual=fabs(target_course_error)/
                handoff_contract.perpendicular_heading_half_width_deg;
            double energy_scale=fmax(
                planet_surface_gravity(p)*fmax(geo.altitude-cfg->site.altitude,0.0),
                .5*fmax(a.speed*a.speed,DBL_MIN));
            double energy_residual=fmax(0.0,-capture.energy_margin)/
                fmax(energy_scale,DBL_MIN);
            double turn_residual=fmax(0.0,-capture.turn_margin)/
                fmax(energy_scale,DBL_MIN);
            double altitude_span=fmax(settings->mm305_max_altitude-
                settings->mm305_min_altitude,DBL_EPSILON);
            double altitude_residual=fmax(0.0,fmax(
                settings->mm305_min_altitude-geo.altitude,
                geo.altitude-settings->mm305_max_altitude))/altitude_span;
            double sound=planet_atmospheric_speed_of_sound(p,geo.altitude);
            double mach=isfinite(sound)&&sound>DBL_EPSILON?
                a.speed/sound:NAN;
            double mach_half_width=fmax(settings->mm305_mach_half_width,
                DBL_EPSILON);
            double mach_residual=isfinite(mach)?fmax(0.0,fmax(
                settings->mm305_target_mach-settings->mm305_mach_half_width-mach,
                mach-(settings->mm305_target_mach+
                    settings->mm305_mach_half_width)))/mach_half_width:
                INFINITY;
            double residual=fmax(position_residual,
                fmax(course_residual,
                    fmax(mach_residual,
                        fmax(altitude_residual,
                            fmax(energy_residual,turn_residual)))));
            inlet.quality_score=residual;

            bool better=false;
            if(!isfinite(best.cost))better=true;
            else if(valid!=best.valid)better=valid;
            else{
                unsigned trial_veto=0,best_veto=0;
                for(unsigned bits=capture.veto;bits;bits>>=1)
                    trial_veto+=bits&1u;
                for(unsigned bits=best.capture_veto;bits;bits>>=1)
                    best_veto+=bits&1u;
                if(trial_veto!=best_veto)better=trial_veto<best_veto;
                else if(residual!=best.cost)better=residual<best.cost;
                else if(position_error!=best.position_error)
                    better=position_error<best.position_error;
                else if(fabs(target_course_error)!=fabs(best.capture_course_error))
                    better=fabs(target_course_error)<
                        fabs(best.capture_course_error);
                else if(fmax(0.0,-capture.energy_margin)!=
                        fmax(0.0,-best.terminal_energy_margin))
                    better=fmax(0.0,-capture.energy_margin)<
                        fmax(0.0,-best.terminal_energy_margin);
                else
                    better=state.ut<best.capture_ut;
            }
            if(better){
                best=(EntryTopologyPlan){
                    .valid=valid,.failure_flags=failure_flags,.planned_ut=initial.ut,
                    .first_sign=side,.first_bank=first_bank,.turn_bank=turn_bank,
                    .outbound_course_offset=selected_outbound_offset,.capture_veto=capture.veto,.capture_turn_margin=capture.turn_margin,
                    .first_aoa=first_aoa,.turn_aoa=turn_aoa,.aoa_switch_speed=aoa_switch_speed,
                    .reversal_ut=reversal_ut,.terminal_turn_ut=terminal_turn_ut,.capture_ut=state.ut,
                    .reversal_along=reversal_along,.reversal_cross=reversal_cross,.reversal_altitude=reversal_altitude,
                    .reversal_speed=reversal_speed,.reversal_course=reversal_course,
                    .reversal_turn_radius=terminal_turn_radius,.reversal_geometry_error=terminal_turn_geometry_error,
                    .terminal_turn_along=terminal_turn_along,.terminal_turn_cross=terminal_turn_cross,
                    .terminal_turn_altitude=terminal_turn_altitude,.terminal_turn_speed=terminal_turn_speed,
                    .terminal_turn_course=terminal_turn_course,.terminal_turn_radius=terminal_turn_radius,
                    .terminal_turn_geometry_error=terminal_turn_geometry_error,
                    .capture_along=along,.capture_cross=cross,.capture_altitude=geo.altitude,.capture_speed=a.speed,
                    .capture_course=course,.capture_course_error=target_course_error,.capture_bank=ctx.bank,
                    .capture_heading_debt=heading_debt,.capture_endpoint_error=endpoint_error,
                    .heading_lock_ut=heading_lock_ut,.heading_lock_along=heading_lock_along,.heading_lock_cross=heading_lock_cross,
                    .heading_lock_altitude=heading_lock_altitude,.heading_lock_speed=heading_lock_speed,
                    .heading_lock_course=heading_lock_course,.heading_lock_bank=heading_lock_bank,
                    .position_error=position_error,.glide_reserve=reserve,.terminal_energy_margin=capture.energy_margin,
                    .minimum_alignment_altitude=minimum_alignment_altitude,.cost=residual,.inlet=inlet};
            }
        }
        state=rk4_aero(state,dt,&ctx);
    }
    if(!isfinite(best.cost)&&isfinite(setup_seed_metric)){
        double setup_endpoint=hypot(setup_seed_along-inlet_along,
            setup_seed_cross-inlet_cross);
        double setup_course_error=norm_signed_deg(target_course-setup_seed_course);
        unsigned failure_flags=propagation_failure_flags;
        if(!reversal_recorded)failure_flags|=ENTRY_TOPOLOGY_FAILURE_REVERSAL_SETUP;
        if(!terminal_turn_started)failure_flags|=ENTRY_TOPOLOGY_FAILURE_TERMINAL_TURN;
        else if(!heading_locked)failure_flags|=ENTRY_TOPOLOGY_FAILURE_HEADING_LOCK;
        if(setup_endpoint>taem_handoff_contract(settings).horizontal_radius_m)
            failure_flags|=ENTRY_TOPOLOGY_FAILURE_POSITION;
        if(fabs(setup_course_error)>30.0)failure_flags|=ENTRY_TOPOLOGY_FAILURE_COURSE;
        failure_flags|=entry_topology_capture_failure_flags(2u|4u|16u);
        best=(EntryTopologyPlan){
            .valid=false,.failure_flags=failure_flags,.planned_ut=initial.ut,
            .first_sign=side,.first_bank=first_bank,.turn_bank=turn_bank,
            .outbound_course_offset=selected_outbound_offset,
            .first_aoa=first_aoa,.turn_aoa=turn_aoa,.aoa_switch_speed=aoa_switch_speed,
            .reversal_ut=isfinite(reversal_ut)?reversal_ut:setup_seed_ut,
            .reversal_along=reversal_along,.reversal_cross=reversal_cross,.reversal_altitude=reversal_altitude,
            .reversal_speed=reversal_speed,.reversal_course=reversal_course,
            .reversal_turn_radius=setup_seed_radius,.reversal_geometry_error=setup_seed_geom,
            .terminal_turn_ut=setup_seed_ut,.terminal_turn_along=setup_seed_along,.terminal_turn_cross=setup_seed_cross,
            .terminal_turn_altitude=setup_seed_altitude,.terminal_turn_speed=setup_seed_speed,
            .terminal_turn_course=setup_seed_course,.terminal_turn_radius=setup_seed_radius,
            .terminal_turn_geometry_error=setup_seed_geom,
            .capture_ut=setup_seed_ut,.capture_along=setup_seed_along,.capture_cross=setup_seed_cross,
            .capture_altitude=setup_seed_altitude,.capture_speed=setup_seed_speed,.capture_course=setup_seed_course,
            .capture_course_error=setup_course_error,.capture_bank=0.0,
            .capture_heading_debt=isfinite(setup_seed_radius)?
                setup_seed_radius*fabs(setup_course_error)*DEG2RAD:INFINITY,
            .capture_endpoint_error=setup_endpoint,.position_error=setup_endpoint,
            .capture_veto=2u|4u|16u,.minimum_alignment_altitude=NAN,.terminal_energy_margin=NAN,
            .glide_reserve=-INFINITY,.cost=setup_seed_metric};
    }
    return best;
}

static unsigned predictor_popcount_unsigned(unsigned value){
    unsigned count=0u;
    while(value){count+=value&1u;value>>=1;}
    return count;
}

static bool entry_topology_non_geometric_qualified(
        const EntryTopologyPlan*p){
    if(!p||!isfinite(p->cost))return false;
    unsigned blocking=ENTRY_TOPOLOGY_FAILURE_SPEED_PATH|
        ENTRY_TOPOLOGY_FAILURE_MANEUVER_PATH|
        ENTRY_TOPOLOGY_FAILURE_CAPTURE_SPEED|
        ENTRY_TOPOLOGY_FAILURE_CAPTURE_ALTITUDE|
        ENTRY_TOPOLOGY_FAILURE_CAPTURE_ENERGY|
        ENTRY_TOPOLOGY_FAILURE_CAPTURE_STRUCTURAL;
    return (p->failure_flags&blocking)==0u;
}

static unsigned entry_topology_progress_rank(
        const EntryTopologyPlan*p){
    if(!p||!isfinite(p->cost))return 0u;
    if(p->failure_flags&ENTRY_TOPOLOGY_FAILURE_REVERSAL_SETUP)return 1u;
    if(p->failure_flags&ENTRY_TOPOLOGY_FAILURE_TERMINAL_TURN)return 2u;
    if(p->failure_flags&ENTRY_TOPOLOGY_FAILURE_HEADING_LOCK)return 3u;
    return p->valid?5u:4u;
}

static bool entry_topology_search_better(
        const EntryTopologyPlan*trial,const EntryTopologyPlan*best){
    if(!trial||!isfinite(trial->cost))return false;
    if(!best||!isfinite(best->cost))return true;
    if(trial->valid!=best->valid)return trial->valid;

    unsigned tp=entry_topology_progress_rank(trial);
    unsigned bp=entry_topology_progress_rank(best);
    if(tp!=bp)return tp>bp;

    bool tq=entry_topology_non_geometric_qualified(trial);
    bool bq=entry_topology_non_geometric_qualified(best);
    if(tq!=bq)return tq;

    unsigned tf=predictor_popcount_unsigned(trial->failure_flags);
    unsigned bf=predictor_popcount_unsigned(best->failure_flags);
    if(tf!=bf)return tf<bf;

    unsigned tv=predictor_popcount_unsigned(trial->capture_veto);
    unsigned bv=predictor_popcount_unsigned(best->capture_veto);
    if(tv!=bv)return tv<bv;

    /* Compare the normalized full-contract residual before individual axes.
       This prevents a small positional improvement from outweighing much
       larger speed or energy errors in an otherwise similar candidate. */
    if(isfinite(trial->cost)&&isfinite(best->cost)&&trial->cost!=best->cost)
        return trial->cost<best->cost;

    if(isfinite(trial->position_error)&&isfinite(best->position_error)&&
       trial->position_error!=best->position_error)
        return trial->position_error<best->position_error;

    double tc=fabs(trial->capture_course_error);
    double bc=fabs(best->capture_course_error);
    if(isfinite(tc)&&isfinite(bc)&&tc!=bc)return tc<bc;

    double te=fmax(0.0,-trial->terminal_energy_margin);
    double be=fmax(0.0,-best->terminal_energy_margin);
    if(isfinite(te)&&isfinite(be)&&te!=be)return te<be;

    double tt=fmax(0.0,-trial->capture_turn_margin);
    double bt=fmax(0.0,-best->capture_turn_margin);
    if(isfinite(tt)&&isfinite(bt)&&tt!=bt)return tt<bt;

    return trial->capture_ut<best->capture_ut;
}

static EntryTopologyPlan predictor_plan_entry_topology_for_inlet(VehicleState state,
        const Telemetry*t,const GuidanceMachine*base,const PlanetModel*p,
        const AerodynamicEnvelope*env,const TrajectoryCalibrationModel*cal,
        const LandingConfiguration*cfg){
    EntryTopologyPlan best={.cost=INFINITY};
    if(!t||!base||!base->taem_interface_target.valid||!p||!env||!cal||!cfg||!isfinite(state.ut)||
       !(t->true_air_speed>=cfg->vehicle.minimum_safe_speed)||
       !(t->horizontal_speed>DBL_EPSILON))
        return best;

    double energy_target_speed=
        base&&base->taem_interface_target.valid&&
        isfinite(base->taem_interface_target.speed)&&
        base->taem_interface_target.speed>0.0?
            base->taem_interface_target.speed:
            entry_topology_energy_target_speed(p,cfg);
    double eta=entry_topology_interface_eta(
        t,&base->taem_interface_target,cfg,energy_target_speed);
    if(!isfinite(eta))
        eta=fmax(0.0,t->range_to_site)/t->horizontal_speed;

    double max_bank=cfg->vehicle.maximum_bank_angle;
    if(!(max_bank>0.0))return best;
    double alpha_floor=entry_thermal_protection_aoa_floor(
        &cfg->vehicle);
    double alpha_ceiling=entry_final_s_turn_aoa_ceiling(
        &cfg->vehicle);
    alpha_ceiling=fmin(alpha_ceiling,
        cfg->vehicle.maximum_angle_of_attack);
    if(alpha_ceiling<alpha_floor)alpha_ceiling=alpha_floor;
    double first_aoa_seed=base&&isfinite(base->entry_s_turn_plan.target_aoa)?
        clampd(base->entry_s_turn_plan.target_aoa,alpha_floor,alpha_ceiling):
        alpha_floor;
    /*
     * Live MM304 already owns the executable first-leg bank and AoA through
     * DRAGREF/ENTRYLATERAL/@ALPHA.  Searching those controls again here creates
     * routes that the live controller will never fly and multiplies the topology
     * solve by hundreds of full atmospheric propagations.  Anchor the first leg
     * to the currently executable plan and search only the future terminal-turn
     * authority/incidence plus the reversal timing.  If no persistent plan exists
     * yet, the measured bank (or the demonstrated maximum as a final fallback)
     * supplies the same executable seed.
     */
    double first_bank_seed=max_bank;
    if(base->entry_s_turn_plan.valid&&isfinite(base->entry_s_turn_plan.target_bank)&&
       fabs(base->entry_s_turn_plan.target_bank)>DBL_EPSILON)
        first_bank_seed=clampd(fabs(base->entry_s_turn_plan.target_bank),DBL_EPSILON,max_bank);
    else if(isfinite(t->roll)&&fabs(norm_signed_deg(t->roll))>DBL_EPSILON)
        first_bank_seed=clampd(fabs(norm_signed_deg(t->roll)),DBL_EPSILON,max_bank);

    const int delay_samples=ENTRY_TOPOLOGY_DELAY_SAMPLE_COUNT;
    const double turn_bank_candidates[]={first_bank_seed,max_bank};
    const double turn_aoa_candidates[]={alpha_floor,first_aoa_seed,alpha_ceiling};
    const int turn_bank_samples=(int)(sizeof(turn_bank_candidates)/sizeof(turn_bank_candidates[0]));
    const int turn_aoa_samples=(int)(sizeof(turn_aoa_candidates)/sizeof(turn_aoa_candidates[0]));

    double earliest_delay=fmax(
        0.0,cfg->guidance.s_turn_minimum_leg_duration);
    double latest_delay=fmax(earliest_delay,eta);

    bool feasible_topology_found=false;
    for(int side=-1;side<=1;side+=2)
    for(int ti=0;ti<turn_bank_samples;ti++)
    for(int ai=0;ai<turn_aoa_samples;ai++)
    for(int di=0;di<delay_samples;di++){
        double turn_bank=clampd(turn_bank_candidates[ti],DBL_EPSILON,max_bank);
        double turn_aoa=clampd(turn_aoa_candidates[ai],alpha_floor,alpha_ceiling);
        double delay=earliest_delay+
            (latest_delay-earliest_delay)*
            (double)di/(double)(delay_samples-1);

        EntryTopologyPlan trial=entry_topology_trial(
            state,t,base,p,env,cal,cfg,side,
            first_bank_seed,turn_bank,0.0,
            first_aoa_seed,turn_aoa,delay,NAN);
        if(getenv("KSP_LANDER_TRACE_TOPOLOGY_GRID")&&side>0.0&&ti==0&&ai==0){
            fprintf(stderr,
                "Entry topology grid: firstBank=%.1f turnBank=%.1f firstAoA=%.1f turnAoA=%.1f delay=%.1f valid=%d flags=0x%x along=%+.1f cross=%+.1f pos=%.1f h=%.1f V=%.1f course=%.2f geom=%.1f veto=%u cost=%.1f\n",
                first_bank_seed,turn_bank,first_aoa_seed,turn_aoa,delay,
                trial.valid,trial.failure_flags,trial.capture_along,trial.capture_cross,
                trial.position_error,trial.capture_altitude,trial.capture_speed,
                trial.capture_course,trial.terminal_turn_geometry_error,
                trial.capture_veto,trial.cost);
        }
        if(entry_topology_search_better(&trial,&best))best=trial;
        /* A propagated route that satisfies the complete capture contract is
           already executable. Stop immediately; extra optimizer work cannot
           improve ownership safety and only delays the live reversal. */
        if(trial.valid){best=trial;feasible_topology_found=true;goto coarse_search_complete;}
    }
coarse_search_complete:

    /*
     * Refine only the three coordinates that topology actually owns. Coordinate
     * descent avoids the old 3^5 Cartesian neighborhood (972 extra propagated
     * routes) while retaining successively halved continuous resolution around
     * the best physically propagated candidate.
     */
    if(isfinite(best.cost)&&!feasible_topology_found){
        double bank_step=max_bank/(double)3.0;
        double aoa_step=(alpha_ceiling-alpha_floor)/2.0;
        double delay_step=(latest_delay-earliest_delay)/
            fmax((double)(delay_samples-1),1.0);

        for(int iteration=0;iteration<4;iteration++){
            EntryTopologyPlan seed=best;
            double seed_delay=isfinite(seed.reversal_ut)?
                clampd(seed.reversal_ut-state.ut,earliest_delay,latest_delay):
                .5*(earliest_delay+latest_delay);
            for(int direction=-1;direction<=1;direction+=2){
                EntryTopologyPlan trial=entry_topology_trial(
                    state,t,base,p,env,cal,cfg,seed.first_sign,
                    first_bank_seed,
                    clampd(seed.turn_bank+direction*bank_step,DBL_EPSILON,max_bank),
                    0.0,first_aoa_seed,seed.turn_aoa,seed_delay,NAN);
                if(entry_topology_search_better(&trial,&best))best=trial;
                if(trial.valid){best=trial;feasible_topology_found=true;goto refinement_complete;}
            }
            seed=best;
            seed_delay=isfinite(seed.reversal_ut)?
                clampd(seed.reversal_ut-state.ut,earliest_delay,latest_delay):
                seed_delay;
            for(int direction=-1;direction<=1;direction+=2){
                EntryTopologyPlan trial=entry_topology_trial(
                    state,t,base,p,env,cal,cfg,seed.first_sign,
                    first_bank_seed,seed.turn_bank,0.0,first_aoa_seed,
                    clampd(seed.turn_aoa+direction*aoa_step,alpha_floor,alpha_ceiling),
                    seed_delay,NAN);
                if(entry_topology_search_better(&trial,&best))best=trial;
                if(trial.valid){best=trial;feasible_topology_found=true;goto refinement_complete;}
            }
            seed=best;
            seed_delay=isfinite(seed.reversal_ut)?
                clampd(seed.reversal_ut-state.ut,earliest_delay,latest_delay):
                seed_delay;
            for(int direction=-1;direction<=1;direction+=2){
                EntryTopologyPlan trial=entry_topology_trial(
                    state,t,base,p,env,cal,cfg,seed.first_sign,
                    first_bank_seed,seed.turn_bank,0.0,first_aoa_seed,seed.turn_aoa,
                    clampd(seed_delay+direction*delay_step,earliest_delay,latest_delay),NAN);
                if(entry_topology_search_better(&trial,&best))best=trial;
                if(trial.valid){best=trial;feasible_topology_found=true;goto refinement_complete;}
            }
            bank_step*=.5;
            aoa_step*=.5;
            delay_step*=.5;
        }
    }
refinement_complete:

    /*
     * If the physically propagated trial did not autonomously detect the final
     * turn, refine that event time directly between the shaping reversal and
     * the predicted MM304 interface ETA.  The forced event is only a search
     * coordinate; the propagated strict capture contract still decides validity.
     */
    if(isfinite(best.cost)&&!best.valid&&isfinite(best.reversal_ut)){
        double reversal_delay=best.reversal_ut-state.ut;
        double lo=fmax(reversal_delay,earliest_delay);
        double hi=fmax(lo,latest_delay);
        for(int iteration=0;iteration<7;iteration++){
            double mid=.5*(lo+hi);
            EntryTopologyPlan left=entry_topology_trial(
                state,t,base,p,env,cal,cfg,best.first_sign,
                first_bank_seed,best.turn_bank,0.0,
                first_aoa_seed,best.turn_aoa,
                reversal_delay,.5*(lo+mid));
            EntryTopologyPlan right=entry_topology_trial(
                state,t,base,p,env,cal,cfg,best.first_sign,
                first_bank_seed,best.turn_bank,0.0,
                first_aoa_seed,best.turn_aoa,
                reversal_delay,.5*(mid+hi));

            bool left_better=entry_topology_search_better(&left,&right);
            EntryTopologyPlan chosen=left_better?left:right;
            if(entry_topology_search_better(&chosen,&best))best=chosen;
            if(chosen.valid){best=chosen;break;}
            if(left_better)hi=mid;
            else lo=mid;
        }
    }

    if(getenv("KSP_LANDER_TRACE_TOPOLOGY_TIMING")){
        fprintf(stderr,
            "Entry topology constrained solve: valid=%d flags=0x%x side=%+.0f bank=%.2f/%.2f aoa=%.2f/%.2f rev=%.2fs term=%.2fs cap=%.2fs pos=%.1f course=%.2f veto=%u residual=%.5f\n",
            best.valid,best.failure_flags,best.first_sign,
            best.first_bank,best.turn_bank,best.first_aoa,best.turn_aoa,
            best.reversal_ut-state.ut,best.terminal_turn_ut-state.ut,
            best.capture_ut-state.ut,best.position_error,
            best.capture_course_error,best.capture_veto,best.cost);
    }
    if(getenv("KSP_LANDER_TRACE_TOPOLOGY_DETAIL")){
        fprintf(stderr,
            "Entry topology detail: targetCourse=%.2f reversal=(along=%+.1f cross=%+.1f h=%.1f V=%.1f course=%.2f) terminal=(along=%+.1f cross=%+.1f h=%.1f V=%.1f course=%.2f radius=%.1f geom=%.1f) capture=(along=%+.1f cross=%+.1f h=%.1f V=%.1f course=%.2f) reserve=%.1f energy=%+.1f turn=%+.1f\n",
            best.inlet.valid?best.inlet.course:NAN,
            best.reversal_along,best.reversal_cross,best.reversal_altitude,
            best.reversal_speed,best.reversal_course,
            best.terminal_turn_along,best.terminal_turn_cross,
            best.terminal_turn_altitude,best.terminal_turn_speed,
            best.terminal_turn_course,best.terminal_turn_radius,
            best.terminal_turn_geometry_error,
            best.capture_along,best.capture_cross,best.capture_altitude,
            best.capture_speed,best.capture_course,
            best.glide_reserve,best.terminal_energy_margin,
            best.capture_turn_margin);
    }
    return best;
}

static bool entry_topology_reciprocal_inlet(const TaemInterfaceTarget*source,
        const Telemetry*t,const PlanetModel*p,const LandingConfiguration*cfg,
        TaemInterfaceTarget*reciprocal){
    if(!source||!source->valid||!t||!p||!cfg||!reciprocal||
       !isfinite(source->along_track)||!isfinite(source->cross_track)||
       !isfinite(source->course)||!isfinite(source->altitude)||
       !isfinite(source->speed)||!(source->speed>0.0)||
       !isfinite(cfg->site.runway_heading)||
       !isfinite(t->runway_along_track)||!isfinite(t->runway_cross_track))
        return false;

    TaemHandoffContract handoff=taem_handoff_contract(&cfg->guidance);
    double relative=norm_signed_deg(source->course-cfg->site.runway_heading);
    double perpendicular_error=fabs(fabs(relative)-90.0);
    double resolution=sqrt(DBL_EPSILON)*90.0;
    if(!isfinite(relative)||
       perpendicular_error>handoff.perpendicular_heading_half_width_deg+resolution)
        return false;

    *reciprocal=*source;
    reciprocal->course=norm_deg(2.0*cfg->site.runway_heading-source->course);
    reciprocal->cross_track=-source->cross_track;
    if(!isfinite(reciprocal->course)||
       (reciprocal->course==source->course&&
        reciprocal->cross_track==source->cross_track))
        return false;

    /* Reflect the existing HAC inlet across the runway centerline. Both
       perpendicular outlets keep the same runway geometry and acceptance
       envelope; each propagated candidate must still pass the full capture,
       energy, turn-authority, and speed contracts below. */
    GeoPoint runway={cfg->site.latitude,cfg->site.longitude,cfg->site.altitude};
    double rh=cfg->site.runway_heading*DEG2RAD;
    double east=reciprocal->along_track*sin(rh)+reciprocal->cross_track*cos(rh);
    double north=reciprocal->along_track*cos(rh)-reciprocal->cross_track*sin(rh);
    GeoPoint target=local_point(runway,east,north,p->radius,reciprocal->altitude);
    reciprocal->specific_energy=rotating_specific_energy(target.latitude,
        reciprocal->altitude,reciprocal->speed,p);
    reciprocal->energy_qualified=false;

    double target_range=hypot(reciprocal->along_track-t->runway_along_track,
        reciprocal->cross_track-t->runway_cross_track);
    double reference_course=isfinite(t->ground_track_heading)?
        t->ground_track_heading:t->heading;
    double eta=entry_topology_interface_eta(t,reciprocal,cfg,reciprocal->speed);
    reciprocal->arrival_ut=isfinite(eta)?t->ut+eta:NAN;
    reciprocal->quality_score=
        fabs(norm_signed_deg(reciprocal->course-reference_course))/
            fmax(handoff.perpendicular_heading_half_width_deg,DBL_EPSILON)+
        target_range/fmax(handoff.horizontal_radius_m,DBL_EPSILON);
    return isfinite(reciprocal->specific_energy)&&isfinite(target_range);
}

EntryTopologyPlan predictor_plan_entry_topology(VehicleState state,
        const Telemetry*t,const GuidanceMachine*base,const PlanetModel*p,
        const AerodynamicEnvelope*env,const TrajectoryCalibrationModel*cal,
        const LandingConfiguration*cfg){
    EntryTopologyPlan best={.cost=INFINITY};
    if(!t||!base||!base->taem_interface_target.valid||!p||!env||!cal||!cfg||
       !isfinite(state.ut)||!(t->true_air_speed>=cfg->vehicle.minimum_safe_speed)||
       !(t->horizontal_speed>DBL_EPSILON))
        return best;

    GuidanceMachine primary_base=*base;
    EntryTopologyPlan primary=predictor_plan_entry_topology_for_inlet(
        state,t,&primary_base,p,env,cal,cfg);
    best=primary;

    TaemInterfaceTarget reciprocal_inlet={0};
    if(entry_topology_reciprocal_inlet(&base->taem_interface_target,t,p,cfg,
            &reciprocal_inlet)){
        GuidanceMachine reciprocal_base=*base;
        reciprocal_base.taem_interface_target=reciprocal_inlet;
        double relative=norm_signed_deg(reciprocal_inlet.course-
            cfg->site.runway_heading);
        reciprocal_base.entry_target_side=relative>=0.0?1.0:-1.0;
        reciprocal_base.entry_target_side_latched=true;
        EntryTopologyPlan reciprocal=predictor_plan_entry_topology_for_inlet(
            state,t,&reciprocal_base,p,env,cal,cfg);
        if(getenv("KSP_LANDER_TRACE_TOPOLOGY_BRANCHES"))
            fprintf(stderr,
                "Entry topology outlet comparison: current=%.2f valid=%d flags=0x%x residual=%.3f position=%.1f reciprocal=%.2f valid=%d flags=0x%x residual=%.3f position=%.1f selected=%.2f\n",
                primary.inlet.valid?primary.inlet.course:base->taem_interface_target.course,
                primary.valid,primary.failure_flags,primary.cost,primary.position_error,
                reciprocal.inlet.valid?reciprocal.inlet.course:reciprocal_inlet.course,
                reciprocal.valid,reciprocal.failure_flags,reciprocal.cost,
                reciprocal.position_error,
                entry_topology_search_better(&reciprocal,&best)?
                    (reciprocal.inlet.valid?reciprocal.inlet.course:
                     reciprocal_inlet.course):
                    (best.inlet.valid?best.inlet.course:
                     base->taem_interface_target.course));
        if(entry_topology_search_better(&reciprocal,&best))best=reciprocal;
    }
    return best;
}
