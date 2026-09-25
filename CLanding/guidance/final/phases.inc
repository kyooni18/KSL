static double final_lift_aoa_limit(const Telemetry *t, const VehicleProfile *v) {
    double limit = v->maximum_angle_of_attack;
    if (t->mach < 1.0 && v->terminal_maximum_lift_angle_of_attack > 0.0)
        limit = fmin(limit, v->terminal_maximum_lift_angle_of_attack);
    return limit;
}

static double terminal_approach_drag_aoa(const Telemetry*t,const VehicleProfile*v,
        double measured_loss,double required_loss,double nominal_aoa){
    double current_df=1.0;
    aerodynamic_force_factors_mach(t->mach,
        fmax(1.0,hypot(t->angle_of_attack,t->sideslip)),v,NULL,&current_df);
    double upper=fmin(22.0,v->maximum_angle_of_attack-1.5);
    upper=fmax(2.0,upper);
    double best=clampd(nominal_aoa,0.0,upper),best_cost=INFINITY;
    for(double aoa=0.0;aoa<=upper+.01;aoa+=.5){
        double df=1.0;
        aerodynamic_force_factors_mach(t->mach,aoa,v,NULL,&df);
        double projected=fmax(.01,measured_loss)*fmax(.05,df)/fmax(.05,current_df);
        double normalized=fabs(projected-required_loss)/fmax(.35,required_loss);
        double trim_cost=fabs(aoa-nominal_aoa)*.012;
        double cost=normalized+trim_cost;
        if(cost<best_cost){best_cost=cost;best=aoa;}
    }
    return best;
}

       GuidanceResult final_guidance(GuidanceMachine*g,const Telemetry*t,double course,const PlanetModel*p,const LandingConfiguration*cfg,const Trajectory*ref,double dt){
    const LandingSite*site=&cfg->site;const VehicleProfile*v=&cfg->vehicle;const GuidanceSettings*s=&cfg->guidance;
    double slope=terminal_outer_glide_slope(g,s),dist=fmax(0,-t->runway_along_track);
    double desired=site->altitude+dist*tan(slope*DEG2RAD),ae=desired-t->mean_altitude;
    double sink=-t->horizontal_speed*tan(slope*DEG2RAD),se=sink-t->vertical_speed;
    double path_limit=g->terminal_glide_mode?1.25:6.5;
    double sink_limit=g->terminal_glide_mode?1.8:2.5;
    double path=robust_pid_update(&g->final_altitude_pid,ae,dt,-path_limit,path_limit)+clampd(se*.18,-sink_limit,sink_limit);
    double preflare_alt=g->terminal_preflare_plan_valid?g->preflare_trigger_altitude:terminal_preflare_altitude(g,t,s);
    double gate_ground=preflare_alt/fmax(tan(slope*DEG2RAD),1e-3);
    double ground_to_gate=fmax(120.0,dist-gate_ground);
    double height_to_gate=fmax(0.0,t->radar_altitude-preflare_alt);
    double time_to_gate=ground_to_gate/fmax(t->horizontal_speed,1.0);

    /* Lateral final is a damped cross-track state controller.  The previous
       fixed lookahead reacted mostly to heading and could either understeer
       far out or become twitchy near the runway.  Cross-track rate gives the
       controller the missing state: it can remove lateral velocity before
       asking for zero position error. */
    double runway_error=norm_signed_deg(course-site->runway_heading)*DEG2RAD;
    double cross_rate=t->horizontal_speed*sin(runway_error);
    double omega=clampd(2.4/fmax(time_to_gate,7.0),.045,.28);
    double lat=-2.0*.92*omega*cross_rate-omega*omega*t->runway_cross_track;
    double look=clampd(t->horizontal_speed*fmax(3.0,fmin(7.0,time_to_gate*.32)),350.0,3200.0);
    double inter=clampd(atan2(-t->runway_cross_track,look)*RAD2DEG,-24.0,24.0);
    double target_course=norm_deg(site->runway_heading+inter),ce=norm_signed_deg(target_course-course);
    /* Add only a small tangent-tracking term; the state feedback above owns
       convergence and therefore remains well damped through shrinking look. */
    lat+=t->horizontal_speed*clampd(ce*DEG2RAD,-.35,.35)*omega*.55;
    double frac=clampd(dist/fmax(s->final_approach_distance,1),0,1);
    double banklim=fmin(30,dynamic_bank_limit(t,v));banklim=fmin(banklim,8+22*sqrt(frac));
    if(t->radar_altitude<preflare_alt*1.25)banklim=fmin(banklim,6+10*clampd(t->radar_altitude/fmax(preflare_alt*1.25,1),0,1));
    double bank=clampd(atan(lat/fmax(planet_surface_gravity(p),.1))*RAD2DEG,-banklim,banklim);
    double target;
    if(g->terminal_glide_mode)
        target=terminal_final_speed_target(g,v,s,dist);
    else if(g->terminal_preflare_plan_valid)
        target=g->preflare_reference_speed+clampd((dist-terminal_final_distance(g,s)*.45)/180.0,0,35.0);
    else target=v->final_approach_speed+clampd(dist/420.0,0,24.0);
    double speedE=.5*(target*target-t->true_air_speed*t->true_air_speed),altE=planet_surface_gravity(p)*ae,total=altE+speedE;
    double balance=clampd((altE-speedE)/fmax(planet_surface_gravity(p)*500,1),-4.0,4.0);
    double minimum=g->terminal_preflare_plan_valid?g->preflare_minimum_speed:
        (g->terminal_glide_mode?fmax(terminal_test_speed_floor(v),v->minimum_safe_speed):
        fmax(v->final_approach_speed*1.03,v->minimum_safe_speed*1.25));
    double speed_shortfall=fmax(0,target-t->true_air_speed-3.0);
    double speed_margin=t->true_air_speed-minimum;
    double altitude_weight=clampd(speed_margin/fmax(12.0,.12*target),0.0,1.0);
    double gate_distance=fmax(0.0,preflare_alt)/fmax(tan(slope*DEG2RAD),1e-3);
    double gate_speed=g->terminal_glide_mode?
        terminal_final_speed_target(g,v,s,gate_distance):
        (g->terminal_preflare_plan_valid?g->preflare_reference_speed:
        fmax(v->final_approach_speed*1.12,v->minimum_safe_speed*1.40));
    double air_path=fmax(1.0,hypot(ground_to_gate,height_to_gate));
    double energy_to_gate=planet_surface_gravity(p)*height_to_gate+
        .5*(t->true_air_speed*t->true_air_speed-gate_speed*gate_speed);
    double required_loss=clampd(energy_to_gate/fmax(air_path,1.0),.02,40.0);
    double live_loss=t->mass>1.0&&isfinite(t->drag_force)&&t->drag_force>0.0?
        t->drag_force/t->mass:0.0;
    double measured_loss=g->terminal_energy_loss_accel_ema>.05?
        fmax(live_loss*1.02,g->terminal_energy_loss_accel_ema):fmax(.05,live_loss*1.15);
    double predicted_gate_speed=NAN,predicted_drag_work=NAN;
    if(!terminal_quadratic_drag_energy_step(t->true_air_speed,measured_loss,air_path,
            planet_surface_gravity(p)*height_to_gate,&predicted_gate_speed,&predicted_drag_work))
        predicted_gate_speed=0.0;
    (void)predicted_drag_work;
    double predicted_deficit=fmax(0.0,gate_speed-predicted_gate_speed);
    double predicted_excess=fmax(0.0,predicted_gate_speed-gate_speed);
    if(g->terminal_glide_mode){
        /* The glide-mode schedule starts at the outer alignment speed, which
           the steep unpowered glide cannot hold; judging energy against it
           reads a permanent shortfall and dives the vehicle below the path.
           Judge speed energy by the physics-predicted gate speed instead. */
        speed_shortfall=0.0;
        speedE=.5*(gate_speed*gate_speed-predicted_gate_speed*predicted_gate_speed);
        total=altE+speedE;
        balance=clampd((altE-speedE)/fmax(planet_surface_gravity(p)*500,1),-4.0,4.0);
    }
    bool low_energy=total>planet_surface_gravity(p)*120||speed_shortfall>2.0||predicted_deficit>4.0;
    double runway_intercept_slope=atan2(fmax(0.0,t->radar_altitude),fmax(dist,1.0))*RAD2DEG;
    double maximum_slope=clampd(runway_intercept_slope,8.0,35.0);
    double effective_slope=clampd(slope+fmax(speed_shortfall*.30,predicted_deficit*.22)-
        fmin(4.0,predicted_excess*.055),8.0,maximum_slope);
    if(path>0)path*=altitude_weight;
    if(balance>0)balance*=altitude_weight;
    if(low_energy){path=fmin(path,0.0);balance=fmin(balance,0.0);}
    terminal_speedbrake_closed(g);
    double gamma=fmax(-35.0,-effective_slope+path+balance);
    double learned=t->calibrated_best_glide_angle_of_attack>0?t->calibrated_best_glide_angle_of_attack:0;
    double glide=learned>0?learned:clampd(v->entry_angle_of_attack,8.0,20.0);
    double trim=terminal_approach_drag_aoa(t,v,measured_loss,required_loss,glide);
    trim=clampd(trim-clampd(fmax(speed_shortfall,predicted_deficit)*.18,0,8.0),0.0,22.0);
    robust_pid_reset(&g->speed_pid);
    GuidanceCommand c=atmospheric(t,target_course,bank,v,0.0,false,PROFILE_APPROACH);
    /* The target AoA is surface pitch minus the live FPA, so the pitch floor
       must admit the steep outer glide; a -12 deg floor on a 30 deg glide
       demands a 17 deg AoA pull-up and floats the vehicle above the path. */
    double pitch_floor=g->terminal_glide_mode?-35.0:-12.0;
    aerodynamic_pitch_target(&c,t,v,clampd(gamma+trim,pitch_floor,18));
    if(t->true_air_speed<minimum)c.target_aoa=fmin(c.target_aoa,trim);
    c.target_pitch=t->flight_path_angle+c.target_aoa;
    c.gear=t->gear||g->gear_command_latched;
    char status[320],warn[256]={0};
    snprintf(status,sizeof(status),"Outer final %.1f deg%s: %.1f km, cross %+.0f m, FPA %.1f / %.1f deg, V %.0f -> %.0f/%.0f m/s, loss %.2f / %.2f m/s2, preflare %.0f m.",effective_slope,low_energy?" energy recovery":"",dist/1000,t->runway_cross_track,t->flight_path_angle,gamma,t->true_air_speed,predicted_gate_speed,gate_speed,measured_loss,required_loss,preflare_alt);
    bool sl=fabs(t->runway_cross_track)<fmax(120,dist*.08)&&fabs(ce)<12;
    bool sv=fabs(ae)<fmax(120,dist*.08)&&fabs(se)<fmax(g->terminal_glide_mode?10.0:5.0,fabs(sink)*.20);
    double speed_floor=g->terminal_glide_mode?fmax(terminal_test_speed_floor(v),g->terminal_test_preflare_min_speed):v->minimum_safe_speed;
    bool ss=t->true_air_speed>speed_floor&&fabs(t->true_air_speed-target)<(g->terminal_glide_mode?45:18);
    if(t->true_air_speed<speed_floor)snprintf(warn,sizeof(warn),"Airspeed is below the terminal aerodynamic floor.");
    else if(dist<5000&&!(sl&&sv&&ss))snprintf(warn,sizeof(warn),"Final approach is not stabilized. Preserve the outer glide and recapture the centerline.");
    else if(total>planet_surface_gravity(p)*500&&!v->allow_powered_approach)snprintf(warn,sizeof(warn),"The unpowered approach is below the preferred energy corridor.");
    GuidanceResult r=result_make(PHASE_FINAL,c,status,warn[0]?warn:NULL);trajectory_copy(&r.reference,ref);return stabilized(g,r,t,v,s,dt);
}

static GuidanceResult preflare_guidance(GuidanceMachine*g,const Telemetry*t,double course,const LandingConfiguration*cfg,const Trajectory*ref,double dt){
    const VehicleProfile*v=&cfg->vehicle;const LandingSite*site=&cfg->site;const GuidanceSettings*s=&cfg->guidance;
    double trigger=fmax(g->preflare_trigger_altitude,fmax(250.0,s->flare_altitude*4.0));
    double desired=g->terminal_preflare_plan_valid?g->preflare_target_sink:
        -clampd(t->horizontal_speed*tan(4.5*DEG2RAD),6,14);
    double target_aoa=g->terminal_preflare_plan_valid?g->preflare_target_aoa:
        clampd(v->entry_angle_of_attack+3,4,v->maximum_angle_of_attack-2);
    double minimum=g->terminal_preflare_plan_valid?g->preflare_minimum_speed:v->minimum_safe_speed;

    /* Execute the maneuver terminal_preflare_plan() actually certified.  The old
       altitude interpolation kept changing the sink target after initiation,
       causing a full pull-up/level-off/dive oscillation and spending the entire
       kinetic reserve.  The certified AoA is the maximum command for recovery;
       once the target sink is reached, only unload it to avoid overshoot. */
    double sink_error=desired-t->vertical_speed;
    double unload=0.0;
    if(sink_error<0.0)
        unload=clampd((-sink_error)*0.30,0.0,target_aoa);
    double commanded_aoa=fmax(0.0,target_aoa-unload);
    if(t->true_air_speed<minimum){
        double relief=clampd((minimum-t->true_air_speed)*.55,0,8.0);
        commanded_aoa=fmax(0.0,commanded_aoa-relief);
    }

    double look=clampd(t->true_air_speed*4.5,320,1500),inter=clampd(atan2(-t->runway_cross_track,look)*RAD2DEG,-14,14);
    double target_course=norm_deg(site->runway_heading+inter),ce=norm_signed_deg(target_course-course);
    double bank=clampd(ce*.55,-8.0,8.0);
    double target=g->terminal_glide_mode?
        terminal_final_speed_target(g,v,s,fmax(0.0,-t->runway_along_track)):
        (g->terminal_preflare_plan_valid?g->preflare_reference_speed:
        fmax(v->final_approach_speed,v->minimum_safe_speed*1.08));
    terminal_speedbrake_closed(g);
    robust_pid_reset(&g->flare_sink_pid);
    robust_pid_reset(&g->speed_pid);
    GuidanceCommand c=atmospheric(t,target_course,bank,v,0.0,false,PROFILE_APPROACH);
    c.has_target_aoa=true;
    c.target_aoa=commanded_aoa;
    c.target_pitch=t->flight_path_angle+commanded_aoa;
    c.gear=t->gear||g->gear_command_latched;
    char status[320];
    snprintf(status,sizeof(status),"Preflare recovery: %.0f / %.0f m, sink %.1f / %.1f m/s, AoA %.1f / %.1f deg, speed %.1f / %.1f m/s.",t->radar_altitude,trigger,t->vertical_speed,desired,t->angle_of_attack,c.target_aoa,t->true_air_speed,target);
    GuidanceResult r=result_make(PHASE_FINAL,c,status,t->true_air_speed<minimum?"Preflare speed margin is low; incidence is being unloaded to preserve the aerodynamic floor.":NULL);
    trajectory_copy(&r.reference,ref);return stabilized(g,r,t,v,s,dt);
}

static GuidanceResult inner_final_guidance(GuidanceMachine*g,const Telemetry*t,double course,const LandingConfiguration*cfg,const Trajectory*ref,double dt){
    const VehicleProfile*v=&cfg->vehicle;const LandingSite*site=&cfg->site;const GuidanceSettings*s=&cfg->guidance;
    double top=fmax(g->preflare_trigger_altitude,s->flare_altitude*4.0),bottom=fmax(s->flare_altitude,1.0);
    double hf=clampd((t->radar_altitude-bottom)/fmax(top-bottom,1.0),0,1);
    double entry_sink=g->terminal_preflare_plan_valid?g->preflare_target_sink:-10.0;
    double flare_entry=-fmax(3.5,fabs(s->touchdown_sink_rate)*2.5);
    double desired=flare_entry+(entry_sink-flare_entry)*sqrt(hf);
    double se=desired-t->vertical_speed,corr=robust_pid_update(&g->flare_sink_pid,se,dt,-2.5,3.5);
    double look=clampd(t->true_air_speed*4.0,260,1000),inter=clampd(atan2(-t->runway_cross_track,look)*RAD2DEG,-10,10);
    double target_course=norm_deg(site->runway_heading+inter),ce=norm_signed_deg(target_course-course);
    double roll=clampd(ce*.55,-6.0,6.0);
    double high=g->terminal_preflare_plan_valid?g->preflare_reference_speed:v->final_approach_speed;
    double low=fmax(v->minimum_safe_speed*1.03,v->touchdown_speed*1.22);
    double target=low+(fmax(low,high)-low)*sqrt(hf);
    if(g->terminal_glide_mode)
        target=terminal_final_speed_target(g,v,s,fmax(0.0,-t->runway_along_track));
    double base=t->calibrated_best_glide_angle_of_attack>0?t->calibrated_best_glide_angle_of_attack:fmin(12.0,v->entry_angle_of_attack);
    double gamma=atan2(desired,fmax(t->horizontal_speed,1))*RAD2DEG;
    terminal_speedbrake_closed(g);
    GuidanceCommand c=atmospheric(t,target_course,roll,v,0.0,false,PROFILE_APPROACH);
    double aoa_limit=final_lift_aoa_limit(t,v);
    c.has_target_aoa=true;
    c.target_aoa=clampd(gamma+clampd(base+2.0+corr,3.0,aoa_limit)-
        t->flight_path_angle,0.0,aoa_limit);
    c.target_pitch=t->flight_path_angle+c.target_aoa;
    c.gear=true;
    char status[256];snprintf(status,sizeof(status),"Inner final: %.0f m, sink %.1f / %.1f m/s, speed %.1f / %.1f m/s, cross %+.0f m.",t->radar_altitude,t->vertical_speed,desired,t->true_air_speed,target,t->runway_cross_track);
    GuidanceResult r=result_make(PHASE_FINAL,c,status,t->vertical_speed<desired-5?"Inner-final sink rate remains high for the available flare height.":NULL);trajectory_copy(&r.reference,ref);return stabilized(g,r,t,v,s,dt);
}

static GuidanceResult flare_guidance(GuidanceMachine*g,const Telemetry*t,double course,
        const LandingConfiguration*cfg,const Trajectory*ref,double dt){
    const VehicleProfile*v=&cfg->vehicle;
    const LandingSite*site=&cfg->site;
    const GuidanceSettings*s=&cfg->guidance;
    double hf=clampd(t->radar_altitude/fmax(s->flare_altitude,1),0,1);
    double approach=-fmax(3.5,fabs(s->touchdown_sink_rate)*2.5);
    double desired=s->touchdown_sink_rate+(approach-s->touchdown_sink_rate)*sqrt(hf);
    double se=desired-t->vertical_speed;
    double corr=robust_pid_update(&g->flare_sink_pid,se,dt,-3,5);
    double aoa=t->calibrated_best_glide_angle_of_attack>0?
        t->calibrated_best_glide_angle_of_attack:5;
    double top=fmax(v->minimum_safe_speed*1.03,v->touchdown_speed*1.22);
    double touch=fmax(v->touchdown_speed,10);
    double blend=pow(hf,.65);
    double target=touch+(top-touch)*blend;
    if(g->terminal_glide_mode)
        target=terminal_final_speed_target(g,v,s,fmax(0.0,-t->runway_along_track));
    double floor=touch+(fmax(v->touchdown_speed*.95,touch)-touch)*fmin(1,hf*2);
    double red=clampd((floor-t->true_air_speed)*.12,0,4);
    double flare=clampd(aoa+(1-hf)*4-red,2,14);
    double gamma=atan2(desired,fmax(t->horizontal_speed,1))*RAD2DEG;
    double look=clampd(t->true_air_speed*3.5,220,900);
    double inter=clampd(atan2(-t->runway_cross_track,look)*RAD2DEG,-10,10);
    double tc=norm_deg(site->runway_heading+inter);
    double ce=norm_signed_deg(tc-course);
    double banklim=t->radar_altitude<=15.0?2.0:4.0;
    double roll=clampd(ce*.6,-banklim,banklim);
    terminal_speedbrake_closed(g);
    GuidanceCommand c=atmospheric(t,tc,roll,v,0.0,false,PROFILE_FLARE);
    double aoa_limit=final_lift_aoa_limit(t,v);
    c.has_target_aoa=true;
    c.target_aoa=clampd(gamma+clampd(flare+corr,0,aoa_limit)-
        t->flight_path_angle,0.0,aoa_limit);
    c.target_pitch=t->flight_path_angle+c.target_aoa;
    c.gear=true;
    char status[256];
    snprintf(status,sizeof(status),
        "Touchdown flare: %.1f m, sink %.1f / %.1f m/s, speed %.1f / %.1f m/s.",
        t->radar_altitude,t->vertical_speed,desired,t->true_air_speed,target);
    GuidanceResult r=result_make(PHASE_FLARE,c,status,
        t->vertical_speed<fmin(-5,desired-3)?
            "Sink rate is high for the remaining touchdown-flare height.":NULL);
    trajectory_copy(&r.reference,ref);
    return stabilized(g,r,t,v,s,dt);
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
    /* Live KSP steering is opposite runway heading/cross-track error: k71's
       positive command saturated while the measured track kept turning left. */
    double steer=-clampd((heading_error*.06-t->runway_cross_track*.0022-
        lateral_rate*.024*rate_weight)*authority,-.25,.25);
    return (RolloutSteeringDemand){.steering=steer,.lateral_rate=lateral_rate};
}

GuidanceResult touchdown(GuidanceMachine*g,const Telemetry*t,
        const LandingConfiguration*cfg,const Trajectory*ref){
    RolloutSteeringDemand steering=runway_rollout_steering(t,cfg);
    double steer=steering.steering,lateral_rate=steering.lateral_rate;
    GuidanceCommand c;guidance_command_init(&c);
    c.gear=true;
    c.brakes=t->surface_speed<fmax(120.0,cfg->vehicle.touchdown_speed*1.35);
    c.airbrakes=true;
    c.wheel_steering=steer;
    if(t->surface_speed>25.0){
        c.has_target_aoa=true;
        c.target_aoa=2.0;
    }
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


static double terminal_touchdown_flare_altitude(const GuidanceMachine*g,const Telemetry*t,
        const PlanetModel*p,const LandingConfiguration*cfg){
    if(!g||!t||!p||!cfg||!g->terminal_preflare_plan_valid)return NAN;
    const GuidanceSettings*s=&cfg->guidance;
    double sink=fmax(0.0,-t->vertical_speed);
    double touchdown=fmax(0.0,-s->touchdown_sink_rate);
    RunwayCaptureEnvelope authority=decision_runway_capture_envelope(
        g,t,cfg->site.runway_heading,p,cfg);
    if(!authority.valid)return NAN;
    VerticalRecoveryEnvelope recovery=decision_vertical_recovery_envelope(
        t->radar_altitude,sink,touchdown,authority.control_response_time_s,
        authority.response_down_accel_mps2,authority.vertical_recovery_accel_mps2);
    return recovery.valid?fmax(s->flare_altitude,recovery.required_height_m):NAN;
}

static bool terminal_airborne_corridor_valid(const GuidanceMachine*g,const Telemetry*t,double course,
        const PlanetModel*p,const LandingConfiguration*cfg){
    if(!g||!t||!p||!cfg)return false;
    TerminalVerticalStage stage=g->terminal_vertical_stage_valid?
        g->terminal_vertical_stage:TERMINAL_TRAJECTORY_CAPTURE;
    if(stage>=TERMINAL_GROUND)return true;

    /*
     * Final survivability has one source of truth.  The runway envelope derives
     * capture time from live lateral acceleration, heading rate and measured
     * control response; vertical reserve from live sink and lift authority; and
     * speed/q/load/runway margins from the vehicle and site.  Do not rebuild a
     * second set of stage-specific angle/cross-track/speed thresholds here.
     */
    RunwayCaptureEnvelope envelope=decision_runway_capture_envelope(g,t,course,p,cfg);
    if(!envelope.valid)return false;
    bool survivable=envelope.speed.margin>=0.0&&
        envelope.dynamic_pressure.margin>=0.0&&
        envelope.load.margin>=0.0&&
        envelope.vertical_recovery.margin>=0.0&&
        envelope.runway_remaining.margin>=0.0;
    if(!survivable)return false;

    if(stage<=TERMINAL_OUTER_FINAL)
        return envelope.reachable;

    /*
     * Once preflare starts there is no remaining terminal-area turn to invent.
     * Require the current velocity vector to intersect the physical runway and
     * retain the same physics-derived survivability margins.
     */
    return terminal_preflare_alignment_valid(g,t,course,cfg);
}
