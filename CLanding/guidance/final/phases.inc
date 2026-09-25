static double final_lift_aoa_limit(const Telemetry *t, const VehicleProfile *v) {
    double limit = v->maximum_angle_of_attack;
    if (t->mach < 1.0 && v->terminal_maximum_lift_angle_of_attack > 0.0)
        limit = fmin(limit, v->terminal_maximum_lift_angle_of_attack);
    return limit;
}

typedef struct { double steering, lateral_rate; } RolloutSteeringDemand;

/*
 * Wheel-Steering Sign Convention and Runway Guidance Law
 * -------------------------------------------------------
 * Command coordinate:
 * - Normalized wheel steering output command in [-1.0, +1.0].
 * - Positive (> 0): Steers nose wheel to the RIGHT (clockwise yaw / +heading,
 *   turning towards +East when runway heading is North).
 * - Negative (< 0): Steers nose wheel to the LEFT (counter-clockwise yaw / -heading,
 *   turning towards -East when runway heading is North).
 *
 * Feedback error signs:
 * 1. Heading error:
 *    heading_error = norm_signed_deg(runway_heading - t->heading)
 *    If t->heading < runway_heading (vehicle pointed LEFT of centerline),
 *    heading_error > 0, calling for positive steering (RIGHT turn).
 * 2. Cross-track displacement:
 *    runway_cross_track > 0 indicates vehicle is displaced to the RIGHT of centerline.
 *    Subtracted from steering demand (-cross_track * 0.0033) to steer LEFT back to center.
 * 3. Lateral velocity / drift rate:
 *    course_error = norm_signed_deg(t->ground_track_heading - runway_heading)
 *    lateral_rate = surface_speed * sin(course_error * DEG2RAD)
 *    Positive lateral_rate means drifting RIGHT across the runway.
 *    Subtracted from steering demand (-lateral_rate * 0.036 * rate_weight) to arrest drift.
 */
static RolloutSteeringDemand runway_rollout_steering(const Telemetry*t,
        const LandingConfiguration*cfg){
    double heading_error=norm_signed_deg(cfg->site.runway_heading-t->heading);
    double course_error=norm_signed_deg(t->ground_track_heading-cfg->site.runway_heading);
    double lateral_rate=t->surface_speed*sin(course_error*DEG2RAD);
    if(!isfinite(lateral_rate))lateral_rate=0.0;
    double rate_weight=clampd((t->surface_speed-8.0)/35.0,0.0,1.0);
    double authority=clampd(55.0/fmax(t->surface_speed,15.0),.25,1.0);
    /* Feedback opposes heading misalignment, cross-track error, and lateral velocity. */
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
    c.airbrakes=false; /* Wheel braking and runway steering own rollout. */
    c.wheel_steering=steer;
    c.heading_control_enabled=true;
    c.target_heading=cfg->site.runway_heading;
    c.navball_speed_mode=SPEED_SURFACE;
    c.control_profile=PROFILE_ROLLOUT;

    bool nose_gear_contact=(t->has_nose_gear_grounded&&t->nose_gear_grounded)||
                           (t->pitch<=0.5);

    char status[160];GuidancePhase phase;
    if(t->surface_speed<1.5){
        if(!terminal_runway_contact_position(t,cfg))
            return terminal_abort(g,"Rollout stopped outside the selected runway envelope.");
        phase=PHASE_COMPLETE;c.wheel_steering=0;c.brakes=true;reset_limiters(g);
        snprintf(status,sizeof(status),"Landing complete.");
    }else if(!nose_gear_contact){
        /* Main gear contact established; active derotation until nose-gear contact.
           Braking is strictly inhibited while the nose is in the air to prevent
           severe pitch-down moments and nose-gear slamming. */
        phase=PHASE_TOUCHDOWN;
        c.brakes=false;
        c.has_target_aoa=true;
        c.target_aoa=0.0;
        /* Derotate toward 0 deg pitch at a controlled rate (~1.8 deg/s) */
        double dt=(g->has_previous_ut&&t->ut>g->previous_ut)?(t->ut-g->previous_ut):0.1;
        double target_pitch=fmax(0.0,t->pitch-1.8*dt);
        c.target_pitch=target_pitch;
        snprintf(status,sizeof(status),"Touchdown derotation: pitch %.1f deg, sink %.1f m/s.",
            t->pitch,t->vertical_speed);
    }else{
        /* Nose gear is firmly on the runway: rollout phase. Full wheel steering
           and progressive braking. */
        phase=PHASE_ROLLOUT;
        c.has_target_aoa=false;
        c.target_pitch=0.0;
        c.brakes=t->surface_speed<fmax(120.0,cfg->vehicle.touchdown_speed*1.35);
        snprintf(status,sizeof(status),"Rollout: %.1f m/s, steering %+.2f, lateral %.1f m/s.",
            t->surface_speed,steer,lateral_rate);
    }
    g->phase=phase;
    g->previous_ut=t->ut;
    g->has_previous_ut=true;
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
