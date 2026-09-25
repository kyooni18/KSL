/*
 * Post-capture approach, flare and touchdown.
 *
 * Final guidance follows a runway-geometry glide, then makes a one-way capture
 * into a response-led low-sink corridor.  A learned lift model maps the vertical
 * acceleration demand to incidence; damped lateral feedback captures the runway
 * centerline.  Measured speedbrake drag is removed before learning clean-aircraft
 * aerodynamics.
 */

static void terminal_aero_q(const GuidanceMachine*g,const Telemetry*t,const VehicleProfile*v,double aoa,double*L,double*D);

/* Live pull response to touchdown needs about ten seconds; shorter lead
   crossed the threshold too high and entered flare downrange. */
static double terminal_pull_response_seconds(void){return 10.0;}
static double terminal_pull_lead_height(double glide_sink,double measured_sink){
    return terminal_pull_response_seconds()*fmax(glide_sink,fmax(0.0,measured_sink));
}

static double terminal_geometry_fpa(double height,double distance,double float_height){
    return clampd(-atan2(fmax(0.0,height-float_height),fmax(120.0,distance)),
        -25.0*DEG2RAD,-4.0*DEG2RAD);
}

static double terminal_sink_profile_fpa(double height,double speed,double float_height,
        double float_sink,double arrest_accel,double height_rate){
    double profile_height=fmax(0.0,height-float_height);
    double sink=float_sink+fmin(sqrt(2.0*arrest_accel*profile_height),height_rate*profile_height);
    return -asin(fmin(.95,sink/speed));
}

static void terminal_learn_aero(GuidanceMachine*g,const Telemetry*t,const VehicleProfile*v){
    double q=t->dynamic_pressure;
    if(!(q>200.0)||!isfinite(t->lift_force)||!isfinite(t->drag_force))return;
    double lf=0.0,df=0.0;
    aerodynamic_force_factors_mach(t->mach,t->angle_of_attack,v,&lf,&df);
    Telemetry tc=*t;
    if(t->has_airbrakes&&t->airbrakes){
        /* Speedbrake drag area: from the vessel physics model when it has one,
           else the excess over the clean table at this incidence. */
        double k=NAN;
        if(t->physics_airbrake_model_available&&t->physics_airbrake_drag_accel>0.0&&t->mass>1.0)
            k=t->physics_airbrake_drag_accel*t->mass/q;
        else{double L=0.0,D=0.0;terminal_aero_q(g,t,v,t->angle_of_attack,&L,&D);if(D>0.0)k=t->drag_force/q-D;}
        if(isfinite(k)&&k>0.0)g->terminal_speedbrake_cda=g->terminal_speedbrake_cda>0.0?g->terminal_speedbrake_cda*.9+k*.1:k;
        if(!(g->terminal_speedbrake_cda>0.0))return;
        tc.drag_force=t->drag_force-g->terminal_speedbrake_cda*q;
        t=&tc;
    }
    if(lf>.05&&t->lift_force>0.0){
        double k=t->lift_force/(q*lf);
        g->terminal_lift_area_ema=g->terminal_lift_area_ema>0.0?g->terminal_lift_area_ema*.9+k*.1:k;
    }
    if(df>.05&&t->drag_force>0.0){
        double k=t->drag_force/(q*df);
        g->terminal_drag_area_ema=g->terminal_drag_area_ema>0.0?g->terminal_drag_area_ema*.9+k*.1:k;
    }
    /* Subsonic incidence table: the real lift curve can be far from the
       model shape, so the landing law inverts what the vehicle has shown. */
    double a=t->angle_of_attack;
    if(t->mach>.8||!(a>=0.0&&a<=20.0))return;
    int i=(int)floor(a);double w=a-i;
    for(int k=0;k<2;k++){
        int j=i+k;if(j<0||j>20)continue;
        double wt=k?w:1.0-w;if(wt<.2)continue;
        double L=t->lift_force/q,D=t->drag_force/q,e=.25*wt;
        /* Low-speed lift is strongly Mach dependent, so a bin learned at a
           different Mach is replaced rather than blended. */
        if(!g->terminal_aero_seen[j]||fabs(t->mach-g->terminal_aero_mach[j])>.04){
            g->terminal_lift_q[j]=L;g->terminal_drag_q[j]=D;g->terminal_aero_seen[j]=1;
        }else{g->terminal_lift_q[j]+=e*(L-g->terminal_lift_q[j]);g->terminal_drag_q[j]+=e*(D-g->terminal_drag_q[j]);}
        g->terminal_aero_mach[j]=t->mach;
    }
}

/* Lift and drag per dynamic pressure at an incidence: observed bins first,
   otherwise the model shape scaled by the learned reference areas. */
static void terminal_aero_q(const GuidanceMachine*g,const Telemetry*t,const VehicleProfile*v,double aoa,double*L,double*D){
    int i=(int)floor(aoa);double w=aoa-i;
    if(t->mach<=.8&&i>=0&&i<20&&g->terminal_aero_seen[i]&&g->terminal_aero_seen[i+1]
       &&fabs(t->mach-g->terminal_aero_mach[i])<=.06&&fabs(t->mach-g->terminal_aero_mach[i+1])<=.06){
        *L=g->terminal_lift_q[i]*(1.0-w)+g->terminal_lift_q[i+1]*w;
        *D=g->terminal_drag_q[i]*(1.0-w)+g->terminal_drag_q[i+1]*w;
        return;
    }
    double lf=0.0,df=0.0;
    aerodynamic_force_factors_mach(t->mach,aoa,v,&lf,&df);
    *L=(g->terminal_lift_area_ema>0.0?g->terminal_lift_area_ema:100.0)*lf;
    *D=(g->terminal_drag_area_ema>0.0?g->terminal_drag_area_ema:50.0)*df;
}

/* Smallest incidence producing the lift; the maximum-lift incidence if none. */
static double terminal_aoa_for_lift(const GuidanceMachine*g,const Telemetry*t,const VehicleProfile*v,double lift,double aoa_max){
    double q=fmax(t->dynamic_pressure,1.0),best=aoa_max,best_l=-INFINITY;
    for(double a=0.0;a<=aoa_max+1e-6;a+=.1){
        double L=0.0,D=0.0;terminal_aero_q(g,t,v,a,&L,&D);
        if(L*q>=lift)return a;
        if(L>best_l){best_l=L;best=a;}
    }
    return best;
}

static GuidanceResult terminal_landing_guidance(GuidanceMachine*g,const Telemetry*t,double course,
        const PlanetModel*p,const LandingConfiguration*cfg,const Trajectory*ref,double dt){
    const GuidanceSettings*s=&cfg->guidance;
    const VehicleProfile*v=&cfg->vehicle;
    const LandingSite*site=&cfg->site;
    const double g0=planet_surface_gravity(p);
    const double H_FLOAT=3.5;
    const double S_FLOAT=.5;
    const double TOUCHDOWN_ALONG=-150.0;
    const double ARREST_ACCEL=.35*g0;
    const double SINK_HEIGHT_RATE=.14;
    const double TOUCHDOWN_PITCH=5.0;

    double height=t->mean_altitude-site->altitude;
    if(isfinite(t->radar_altitude)&&t->radar_altitude>=0.0&&terminal_runway_contact_position(t,cfg))
        height=fmin(height,t->radar_altitude);
    {   /* Fly the deployed main-wheel contact patch, not the CG. On the intact
           STS-N the main-gear part origin is 2.8035 m below and 0.9058 m aft of
           the COM; the 0.35 m wheel radius puts the contact patch 3.1535 m down. */
        const double MAIN_GEAR_DOWN=3.1535;
        const double MAIN_GEAR_AFT=.9058;
        double th=(t->flight_path_angle+t->angle_of_attack)*DEG2RAD;
        height-=MAIN_GEAR_DOWN*cos(th)+MAIN_GEAR_AFT*sin(th);
    }
    height=fmax(0.0,height);
    double V=fmax(1.0,t->true_air_speed),Vh=fmax(1.0,t->horizontal_speed);
    double along=t->runway_along_track,gamma=t->flight_path_angle*DEG2RAD;
    double aoa_max=final_lift_aoa_limit(t,v);
    double v_td=v->touchdown_speed>1.0?v->touchdown_speed:65.0;
    terminal_learn_aero(g,t,v);

    TerminalVerticalStage stage=g->terminal_vertical_stage;
    if(stage<TERMINAL_OUTER_FINAL)stage=TERMINAL_OUTER_FINAL;

    if(g->terminal_vs_prev_ut>0.0&&t->ut>g->terminal_vs_prev_ut+1e-3&&
       t->ut<g->terminal_vs_prev_ut+2.0){
        double rate=(t->vertical_speed-g->terminal_vs_prev)/
            (t->ut-g->terminal_vs_prev_ut);
        if(isfinite(rate))g->terminal_vs_rate_ema+=.3*(clampd(rate,-30.0,30.0)-g->terminal_vs_rate_ema);
    }
    g->terminal_vs_prev=t->vertical_speed;
    g->terminal_vs_prev_ut=t->ut;
    double sink_lead=t->vertical_speed+g->terminal_vs_rate_ema;

    /* Lateral alignment is a damped cross-track/track-rate capture aimed at a
       pre-threshold station.  Low altitude progressively limits bank. */
    double lateral_scale=clampd(height/60.0,0.0,1.0);
    double track_error=norm_signed_deg(course-site->runway_heading)*DEG2RAD;
    double cross_rate=Vh*sin(track_error);
    const double ALIGN_STATION=-2800.0;
    double align_remaining=fmax(0.0,ALIGN_STATION-along);
    double align_time=fmax(6.0,align_remaining/Vh);
    double omega=clampd(2.2/align_time,.06,.18);
    double lateral_accel=-2.0*.95*omega*cross_rate-
        omega*omega*clampd(t->runway_cross_track,-400.0,400.0);
    double bank_limit=fmin(fmin(30.0,terminal_lateral_bank_limit(t,v)),
        3.0+27.0*lateral_scale);
    double bank=clampd(atan(lateral_accel/g0)*RAD2DEG,-bank_limit,bank_limit);
    double lift_div=fmax(.5,cos(gamma)*cos(bank*DEG2RAD));

    /* Learn the lift scale in flight and target a geometric runway intercept.
       Pull capture is one-way: after it begins, the sink profile can only
       flatten relative to the geometric glide. */
    if(g->terminal_lift_ratio<=0.0)g->terminal_lift_ratio=1.0;
    double lift_per_q=0.0,drag_per_q=0.0;
    terminal_aero_q(g,t,v,t->angle_of_attack,&lift_per_q,&drag_per_q);
    (void)drag_per_q;
    double modeled_lift=lift_per_q*fmax(t->dynamic_pressure,1.0);
    if(modeled_lift>.2*t->mass*g0&&isfinite(t->lift_force)&&t->lift_force>0.0)
        g->terminal_lift_ratio+=fmin(1.0,dt/2.0)*
            (clampd(t->lift_force/modeled_lift,.6,1.6)-g->terminal_lift_ratio);

    double touchdown_distance=fmax(120.0,TOUCHDOWN_ALONG-along);
    double glide_fpa=terminal_geometry_fpa(height,touchdown_distance,H_FLOAT);
    double glide_sink=fmax(S_FLOAT,-V*sin(glide_fpa));
    double response_height=terminal_pull_lead_height(glide_sink,-sink_lead);
    double future_height=fmax(0.0,height-response_height);
    double future_along=along+Vh*terminal_pull_response_seconds();
    double future_distance=fmax(120.0,TOUCHDOWN_ALONG-future_along);
    double future_glide_fpa=terminal_geometry_fpa(future_height,future_distance,H_FLOAT);
    double future_sink_fpa=terminal_sink_profile_fpa(
        future_height,V,H_FLOAT,S_FLOAT,ARREST_ACCEL,SINK_HEIGHT_RATE);
    if(future_sink_fpa>future_glide_fpa)g->terminal_pull_latch=true;

    bool flaring=g->terminal_pull_latch;
    double sink_fpa=terminal_sink_profile_fpa(
        height,V,H_FLOAT,S_FLOAT,ARREST_ACCEL,SINK_HEIGHT_RATE);
    double reference_fpa=flaring?fmax(glide_fpa,sink_fpa):glide_fpa;
    double reference_sink=V*sin(reference_fpa);
    double sink=-t->vertical_speed;
    double sink_rate_ff=0.0;
    double height_above_float=fmax(0.0,height-H_FLOAT);
    if(flaring&&height_above_float>0.0){
        double root_limit=sqrt(2.0*ARREST_ACCEL*height_above_float);
        double profile_rate=root_limit<SINK_HEIGHT_RATE*height_above_float?
            ARREST_ACCEL/(root_limit+1e-3):SINK_HEIGHT_RATE;
        sink_rate_ff=profile_rate*sink/(V*fmax(.3,cos(reference_fpa)));
    }
    /* Lead the path loop with motion the vehicle has actually achieved.  The
       lift-force reconstruction was over-optimistic in k93: it predicted roughly
       twice the observed FPA recovery and unloaded the flare while sink was still
       excessive.  Vertical-speed acceleration is already filtered above and gives
       a conservative direct observation of the real pull response. */
    double observed_gamma_rate=clampd(g->terminal_vs_rate_ema/
        (V*fmax(.3,cos(gamma))),-6.0*DEG2RAD,6.0*DEG2RAD);
    double lead_gamma=gamma+1.5*observed_gamma_rate;
    double vertical_accel=clampd(V*(sink_rate_ff+.3*(reference_fpa-lead_gamma)),
        -.3*g0,1.5*g0);
    double lift_required=t->mass*(vertical_accel+g0*cos(gamma))/lift_div/
        g->terminal_lift_ratio;
    double aoa=clampd(terminal_aoa_for_lift(g,t,v,fmax(0.0,lift_required),aoa_max),
        0.0,aoa_max);
    /* Keep a positive body attitude through wheel capture. The two main-gear
       origins are about 11.6 m aft of the nose gear on STS-N, so a five-degree
       touchdown attitude leaves a clear nose-gear height margin without changing
       the vertical sink schedule. Apply it before the existing flare slew limiter. */
    if(flaring&&height<=12.0){
        double touchdown_aoa=clampd(TOUCHDOWN_PITCH-t->flight_path_angle,0.0,aoa_max);
        aoa=fmax(aoa,touchdown_aoa);
    }

    if(stage==TERMINAL_TOUCHDOWN_FLARE||(flaring&&height<=15.0))
        stage=TERMINAL_TOUCHDOWN_FLARE;
    else stage=flaring?TERMINAL_PREFLARE:TERMINAL_OUTER_FINAL;
    if(g->terminal_vertical_stage!=stage)terminal_set_stage(g,stage,t->ut);
    if(g->terminal_vertical_stage==TERMINAL_TOUCHDOWN_FLARE)g->flare_sink_captured=true;
    flaring=g->terminal_vertical_stage!=TERMINAL_TRAJECTORY_CAPTURE&&
        g->terminal_vertical_stage!=TERMINAL_OUTER_FINAL;

    /* Follow incidence references at measured-safe slew rates; the flare slew
       is slower to avoid chasing force-response noise. */
    if(g->terminal_aoa_cmd_ut>0.0&&t->ut>g->terminal_aoa_cmd_ut&&
       t->ut<g->terminal_aoa_cmd_ut+2.0){
        double rate=flaring?1.5:6.0;
        double allowance=rate*(t->ut-g->terminal_aoa_cmd_ut);
        aoa=clampd(aoa,g->terminal_aoa_cmd_last-allowance,
            g->terminal_aoa_cmd_last+allowance);
    }else aoa=t->angle_of_attack;
    g->terminal_aoa_cmd_last=aoa;
    g->terminal_aoa_cmd_ut=t->ut;

    double intercept=clampd(atan2(-t->runway_cross_track,fmax(400.0,V*6.0))*RAD2DEG,
        -20.0,20.0);
    double target_course=norm_deg(site->runway_heading+intercept);
    if(height<90.0)g->gear_command_latched=true;
    if(t->has_airbrakes&&!flaring&&height>60.0){
        if(V>100.0)g->airbrakes_deployed=true;
        else if(V<95.0)g->airbrakes_deployed=false;
    }else g->airbrakes_deployed=false;

    GuidanceCommand c=atmospheric(t,target_course,bank,v,0.0,
        g->airbrakes_deployed,flaring?PROFILE_FLARE:PROFILE_APPROACH);
    c.has_target_aoa=true;
    c.target_aoa=aoa;
    c.target_pitch=t->flight_path_angle+aoa;
    c.gear=t->gear||g->gear_command_latched;
    GuidancePhase phase=g->terminal_vertical_stage==TERMINAL_TOUCHDOWN_FLARE?
        PHASE_FLARE:PHASE_FINAL;
    g->phase=phase;
    char status[256];
    snprintf(status,sizeof(status),"%s: wheel height %.0f m, along %.0f m, cross %+.0f m, "
        "speed %.0f m/s, sink %.1f / %.1f m/s, AoA %.1f deg.",
        phase==PHASE_FLARE?"Touchdown flare":flaring?"Preflare":"Final glide",
        height,along,t->runway_cross_track,V,t->vertical_speed,reference_sink,aoa);
    const char*warn=NULL;
    if(V<v_td*.9)warn="Airspeed is below the touchdown target; the flare will be firm.";
    else if(along<0.0&&flaring&&V<v_td*1.1)warn="Energy is low for the runway threshold.";
    GuidanceResult r=result_make(phase,c,status,warn);
    trajectory_copy(&r.reference,ref);
    r=stabilized(g,r,t,v,s,dt);
    const char*lat_diag=getenv("KSP_LANDER_FINAL_LATERAL_DIAGNOSTICS");
    if(lat_diag&&strcmp(lat_diag,"1")==0&&fmod(t->ut,1.0)<dt*.999)
        fprintf(stderr,"FINAL_LAT ut=%.3f x=%.1f cross=%.1f course=%.2f track=%.2f runway=%.2f trackErr=%.2f crossRate=%.2f omega=%.3f lat=%.3f bankLimit=%.2f rawBank=%.2f targetHeading=%.2f targetRoll=%.2f actualRoll=%.2f rollRate=%.2f\n",
            t->ut,along,t->runway_cross_track,course,t->ground_track_heading,
            site->runway_heading,track_error*RAD2DEG,cross_rate,omega,lateral_accel,
            bank_limit,bank,target_course,r.command.target_roll,t->roll,t->roll_rate);
    return r;
}

       GuidanceResult terminal_approach_sequence(GuidanceMachine*g,const Telemetry*t,double course,
        const PlanetModel*p,AerodynamicModel aero,const LandingConfiguration*cfg,const Trajectory*ref,double dt){
    (void)aero;
    if(!strcasecmp(t->vessel_situation,"landed")&&!terminal_runway_contact_position(t,cfg))
        return terminal_abort(g,"Touchdown occurred outside the runway 09 contact envelope.");
    if(!strcasecmp(t->vessel_situation,"landed")&&!t->gear)
        return terminal_abort(g,"Touchdown occurred without confirmed landing-gear deployment.");
    if(!g->terminal_vertical_stage_valid)terminal_set_stage(g,TERMINAL_OUTER_FINAL,t->ut);
    terminal_observe_response(g,t,dt);
    if(terminal_update_ground_latch(g,t,cfg,dt))return touchdown(g,t,cfg,ref);
    return terminal_landing_guidance(g,t,course,p,cfg,ref,dt);
}
