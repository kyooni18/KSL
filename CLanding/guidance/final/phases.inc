static double final_lift_aoa_limit(const Telemetry *t, const VehicleProfile *v) {
    double limit = v->maximum_angle_of_attack;
    if (t->mach < 1.0 && v->terminal_maximum_lift_angle_of_attack > 0.0)
        limit = fmin(limit, v->terminal_maximum_lift_angle_of_attack);
    return limit;
}

typedef struct { double steering, lateral_rate; } RolloutSteeringDemand;

static RolloutSteeringDemand runway_rollout_steering(const Telemetry*t,
        const LandingConfiguration*cfg){
    double heading_error=norm_signed_deg(cfg->site.runway_heading-t->heading);
    double course_error=norm_signed_deg(t->ground_track_heading-cfg->site.runway_heading);
    double lateral_rate=t->surface_speed*sin(course_error*DEG2RAD);
    if(!isfinite(lateral_rate))lateral_rate=0.0;
    double rate_weight=clampd((t->surface_speed-8.0)/35.0,0.0,1.0);
    double authority=clampd(55.0/fmax(t->surface_speed,15.0),.25,1.0);
    /* Feedback must oppose both cross-track error and lateral velocity.  The
       stronger gains preserve enough authority to arrest drift during rollout. */
    double correction=(heading_error*.09-t->runway_cross_track*.0033-
        lateral_rate*.036*rate_weight)*authority;
    double steer=clampd(correction,-.40,.40);
    return (RolloutSteeringDemand){.steering=steer,.lateral_rate=lateral_rate};
}

GuidanceResult touchdown(GuidanceMachine*g,const Telemetry*t,
        const LandingConfiguration*cfg,const Trajectory*ref){
    RolloutSteeringDemand steering=runway_rollout_steering(t,cfg);
    double steer=steering.steering,lateral_rate=steering.lateral_rate;
    GuidanceCommand c;guidance_command_init(&c);
    c.gear=true;
    c.brakes=t->surface_speed<fmax(120.0,cfg->vehicle.touchdown_speed*1.35);
    c.airbrakes=false; /* Wheel braking and runway steering own rollout. */
    c.wheel_steering=steer;
    c.heading_control_enabled=true;
    c.target_heading=cfg->site.runway_heading;
    c.navball_speed_mode=SPEED_SURFACE;
    c.control_profile=PROFILE_ROLLOUT;
    char status[160];GuidancePhase phase;
    if(t->surface_speed<1.5){
        if(!terminal_runway_contact_position(t,cfg))
            return terminal_abort(g,"Rollout stopped outside the selected runway envelope.");
        phase=PHASE_COMPLETE;c.wheel_steering=0;c.brakes=true;reset_limiters(g);
        snprintf(status,sizeof(status),"Landing complete.");
    }else if(g->phase!=PHASE_TOUCHDOWN&&g->phase!=PHASE_ROLLOUT){
        phase=PHASE_TOUCHDOWN;
        snprintf(status,sizeof(status),"Touchdown: %.1f m/s, sink %.1f m/s.",t->surface_speed,t->vertical_speed);
    }else{
        phase=PHASE_ROLLOUT;
        snprintf(status,sizeof(status),"Rollout: %.1f m/s, steering %+.2f, lateral %.1f m/s.",
            t->surface_speed,steer,lateral_rate);
    }
    g->phase=phase;
    GuidanceResult r=result_make(phase,c,status,NULL);trajectory_copy(&r.reference,ref);return r;
}

/* A terminal abort is latched. No unvalidated recovery orbit is synthesized. */
GuidanceResult terminal_abort(GuidanceMachine *g, const char *reason) {
    guidance_abort(g);
    snprintf(g->abort_reason, sizeof(g->abort_reason), "%s", reason);
    GuidanceCommand c;
    guidance_command_init(&c);
    return result_make(PHASE_ABORT, c, reason, "Nominal landing terminated. Manual control restored.");
}
