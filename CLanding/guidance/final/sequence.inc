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

/* Best lift-to-drag the learned (or model-shaped) subsonic table offers; the
   flare/float is flown near it. */
static double terminal_learned_max_lift_to_drag(const GuidanceMachine*g,const Telemetry*t,
        const VehicleProfile*v,double aoa_max);

static double terminal_geometry_fpa(double height,double distance,double float_height){
    return clampd(-atan2(fmax(0.0,height-float_height),fmax(120.0,distance)),
        -25.0*DEG2RAD,-4.0*DEG2RAD);
}

/*
 * Shuttle-style flare profile as sink versus wheel height:
 *  - above the shallow-glide height: a constant-deceleration preflare arc,
 *    sink^2 = s_sgs^2 + 2 a (h - h_sgs) (the same kinematics the latch height
 *    is sized with; a profile of the form s_sgs + sqrt(2 a dh) demands a
 *    deceleration a*s/(s - s_sgs) that grows without bound as the sink nears
 *    the shallow-glide sink, a hard late pull), so the pull-up starts only as high
 *    as the present sink and the available pull require (energy is kept for
 *    the float instead of being bled at high incidence from a kilometre up);
 *  - then the shallow glide at its constant sink;
 *  - then a gentle constant-deceleration final flare to the float sink at
 *    the float height, and the float sink below it.
 * Time spent near the ground costs speed at about g/(L/D), so the segments
 * below the arc are sized to be short: a linear sink blend from the
 * shallow-glide height, or a very low float sink, spends many seconds there
 * and delivers the vehicle to the runway well below touchdown speed.
 */
static double terminal_final_flare_height(double float_height,double float_sink,
        double final_accel,double sgs_sink){
    return float_height+fmax(0.0,sgs_sink*sgs_sink-float_sink*float_sink)/(2.0*final_accel);
}
static double terminal_sink_profile(double height,double float_height,double float_sink,
        double arrest_accel,double sgs_height,double sgs_sink,double final_accel){
    if(height>sgs_height)
        return sqrt(sgs_sink*sgs_sink+2.0*arrest_accel*(height-sgs_height));
    if(height>terminal_final_flare_height(float_height,float_sink,final_accel,sgs_sink))
        return sgs_sink;
    if(height>float_height)
        return sqrt(float_sink*float_sink+2.0*final_accel*(height-float_height));
    return float_sink;
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

static double terminal_learned_max_lift_to_drag(const GuidanceMachine*g,const Telemetry*t,
        const VehicleProfile*v,double aoa_max){
    double best=0.0;
    for(double a=1.0;a<=aoa_max+1e-6;a+=.5){
        double L=0.0,D=0.0;terminal_aero_q(g,t,v,a,&L,&D);
        if(D>1e-6&&L/D>best)best=L/D;
    }
    return clampd(best,1.5,10.0);
}

typedef struct {
    double arrest_accel,sgs_height,sgs_sink,final_accel,float_height,float_sink,lift_ratio,aoa_max,g0;
    const PlanetModel*planet;double site_altitude;
} TerminalFlarePlan;

/*
 * Touchdown speed the planned vertical profile delivers from the present
 * state: hold the present flight-path angle until the preflare latch height,
 * then track the sink profile (arc, shallow glide, final flare, float) with
 * the learned lift and drag, integrating speed as a point mass.  The time
 * spent in the arc and near the ground is where an unpowered vehicle's speed
 * goes, and it depends on the vehicle's L/D at the incidences actually
 * flown, so a fixed multiple of touchdown speed at the outer glide cannot
 * tell whether the flare will arrive at touchdown speed; this can.
 */
static double terminal_predict_touchdown_speed(const GuidanceMachine*g,const Telemetry*t,
        const VehicleProfile*v,double height,double V,double sink,bool latched,
        const TerminalFlarePlan*p){
    Telemetry tc=*t;
    double m=fmax(t->mass,1.0);
    double sound=isfinite(t->mach)&&t->mach>.05&&isfinite(t->true_air_speed)?
        t->true_air_speed/t->mach:340.0;
    double rho_now=isfinite(t->atmospheric_density)&&t->atmospheric_density>0.0?
        t->atmospheric_density:2.0*fmax(t->dynamic_pressure,1.0)/fmax(V*V,1.0);
    /* Anchor on the measured density and speed of sound and carry them down
       with the planet's atmosphere profile: from a few kilometres up the
       air near the runway is tens of percent denser, and so draggier. */
    double rho_model_now=p->planet?planet_atmospheric_density(p->planet,p->site_altitude+height):NAN;
    double sound_model_now=p->planet?planet_atmospheric_speed_of_sound(p->planet,p->site_altitude+height):NAN;
    double sin_glide=clampd(sink/fmax(V,1.0),-.95,.95);
    const double DT=.25,TAU=1.5;
    for(int i=0;i<1200&&height>0.0&&V>5.0;++i){
        if(!latched){
            double pull=fmax(0.0,sink*sink-p->sgs_sink*p->sgs_sink)/(2.0*p->arrest_accel);
            if(height<=pull+p->sgs_height+sink*1.5)latched=true;
        }
        double accel_up=0.0;
        if(latched){
            double ref=terminal_sink_profile(height,p->float_height,p->float_sink,
                p->arrest_accel,p->sgs_height,p->sgs_sink,p->final_accel);
            accel_up=clampd((sink-ref)/TAU+(sink>p->sgs_sink&&height>p->sgs_height?p->arrest_accel:0.0),
                -.3*p->g0,1.5*p->g0);
        }else sink=V*sin_glide;
        double gamma=-asin(clampd(sink/V,-.95,.95));
        double rho=rho_now,a=sound;
        if(rho_model_now>0.0&&sound_model_now>0.0){
            rho*=planet_atmospheric_density(p->planet,p->site_altitude+height)/rho_model_now;
            a*=planet_atmospheric_speed_of_sound(p->planet,p->site_altitude+height)/sound_model_now;
        }
        tc.mach=V/a;tc.dynamic_pressure=.5*rho*V*V;
        double lift=m*(accel_up+p->g0*cos(gamma))/fmax(p->lift_ratio,.3);
        double aoa=terminal_aoa_for_lift(g,&tc,v,fmax(0.0,lift),p->aoa_max);
        double L=0.0,D=0.0;terminal_aero_q(g,&tc,v,aoa,&L,&D);
        V+=(-D*tc.dynamic_pressure/m-p->g0*sin(gamma))*DT;
        if(latched)sink-=accel_up*DT;
        height-=sink*DT;
    }
    return V;
}

static GuidanceResult terminal_landing_guidance(GuidanceMachine*g,const Telemetry*t,double course,
        const PlanetModel*p,const LandingConfiguration*cfg,const Trajectory*ref,double dt){
    const GuidanceSettings*s=&cfg->guidance;
    const VehicleProfile*v=&cfg->vehicle;
    const LandingSite*site=&cfg->site;
    const double g0=planet_surface_gravity(p);
    const double H_FLOAT=1.5;
    const double S_FLOAT=1.0;       /* a third of the 3 m/s touchdown sink gate */
    const double FINAL_FLARE_ACCEL=.1*g0;
    const double TOUCHDOWN_ALONG=75.0;
    const double ARREST_ACCEL=.5*g0;   /* preflare design pull; see the arc energy note below */
    const double SGS_HEIGHT=25.0;   /* shallow-glide segment below this wheel height */
    const double SGS_SINK=3.5;      /* ~ -2..-2.5 deg at touchdown speeds */
    const double TOUCHDOWN_PITCH=8.0;
    const double TOUCHDOWN_ATTITUDE_START=26.0;
    const double TOUCHDOWN_ATTITUDE_FULL=5.0;

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

    /* Lateral alignment: an authority-aware cross-track capture (below).  The
       capture rate is no faster than the pre-threshold alignment station
       allows; low altitude progressively limits bank. */
    double lateral_scale=clampd(height/60.0,0.0,1.0);
    double track_error=norm_signed_deg(course-site->runway_heading)*DEG2RAD;
    double cross_rate=Vh*sin(track_error);
    const double ALIGN_STATION=-2800.0;
    double align_remaining=fmax(0.0,ALIGN_STATION-along);
    double align_time=fmax(6.0,align_remaining/Vh);
    double omega=clampd(2.2/align_time,.06,.18);
    double bank_limit=fmin(fmin(30.0,terminal_lateral_bank_limit(t,v)),
        3.0+27.0*lateral_scale);
    /* Authority-aware capture.  Command the cross-track rate the vehicle can
       still stop with the bank it will have later in the approach (bank
       authority fades with speed and height), then close on that rate.  A
       fixed-gain capture overshoots when it spends high-speed bank and the
       low-speed limit cannot take the overshoot back out. */
    /* Lateral acceleration is lift times sin(bank).  In the preflare pull
       lift is 2-3 g, so g*tan(bank) (a 1-g relation) understates it 2-3x and
       the capture overshoots; use the measured lift. */
    double lift_accel=isfinite(t->lift_force)&&t->lift_force>0.0&&t->mass>1.0?
        t->lift_force/t->mass:g0;
    lift_accel=fmax(.3*g0,lift_accel);
    double later_bank=fmin(bank_limit,10.0);
    double stop_accel=.6*g0*tan(later_bank*DEG2RAD);
    double y=clampd(t->runway_cross_track,-2000.0,2000.0);
    /* The bank command passes through the Final roll limiter (rate and
       acceleration limited).  A lateral acceleration demand therefore arrives
       only after the minimum-time swing of that limiter from its present
       state, and the cross-track rate keeps integrating the present lateral
       acceleration (fading roughly linearly) meanwhile.  Both the prediction
       horizon and the position bandwidth come from that swing time, so the
       capture is consistent with whatever roll limits are configured.  */
    GuidanceAttitudeLimits roll_limits=decision_guidance_attitude_limits(t,s,PHASE_FINAL,false);
    double roll_now=g->roll_limiter.has_value?norm_signed_deg(g->roll_limiter.value):
        norm_signed_deg(t->roll);
    double roll_rate_now=g->roll_limiter.has_value?g->roll_limiter.rate:
        (isfinite(t->roll_rate)?t->roll_rate:0.0);
    double reversal_time=decision_axis_capture_time(2.0*later_bank,0.0,
        roll_limits.accel_deg_s2,roll_limits.rate_deg_s);
    const double RATE_CLOSURE_TAU=2.0;
    /* Position loop an order slower than the rate loop plus one bank reversal. */
    double omega_limit=.25/(RATE_CLOSURE_TAU+(isfinite(reversal_time)?reversal_time:30.0));
    double rate_gain=fmin(fmin(omega,omega_limit),sqrt(2.0*stop_accel/fmax(fabs(y),1.0)));
    double desired_rate=-y*rate_gain;
    double present_accel=lift_accel*sin(clampd(roll_now,-60.0,60.0)*DEG2RAD);
    double lateral_limit=lift_accel*sin(bank_limit*DEG2RAD);
    double lateral_accel=0.0;
    for(int it=0;it<3;++it){
        double bank_goal=asin(clampd(lateral_accel/lift_accel,-1.0,1.0))*RAD2DEG;
        double swing=decision_axis_capture_time(bank_goal-roll_now,roll_rate_now,
            roll_limits.accel_deg_s2,roll_limits.rate_deg_s);
        if(!isfinite(swing))swing=reversal_time;
        if(!isfinite(swing))swing=10.0;
        /* rate at swing end = cross_rate + (a0 + a_cmd)/2 * swing; close the
           remaining rate error with time constant RATE_CLOSURE_TAU. */
        lateral_accel=clampd((desired_rate-cross_rate-.5*present_accel*swing)/
            (.5*swing+RATE_CLOSURE_TAU),-lateral_limit,lateral_limit);
    }
    double bank=clampd(asin(clampd(lateral_accel/lift_accel,-1.0,1.0))*RAD2DEG,
        -bank_limit,bank_limit);
    /* Flight-path normal acceleration resolves lift through bank only.
     * Gravity already has its cos(gamma) projection in the gamma equation. */
    double lift_div=cos(bank*DEG2RAD);

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

    /*
     * Outer glide aim point.  The Shuttle flew a steep outer glide to an aim
     * point short of the runway and used the speedbrake to arrive at preflare
     * with the energy that floats it to the touchdown target.  This vehicle
     * has no usable speedbrake, so the aim point moves instead: from preflare
     * (height h_pf, speed V) to touchdown at v_td the float covers about
     * LD_f * (h_pf + (V^2 - v_td^2)/(2g)), so the glide is aimed that far short
     * of the target (plus the glide distance below h_pf).  LD_f comes from the
     * learned lift/drag, so a low-L/D vehicle aims long and a high-L/D one aims
     * short instead of both aiming at a fixed point.
     */
    double runway_length=site->runway_length>100.0?site->runway_length:2500.0;
    double touchdown_target=clampd(.15*runway_length,TOUCHDOWN_ALONG,600.0);
    double ld_float=.8*terminal_learned_max_lift_to_drag(g,t,v,aoa_max);
    double pull_capability=NAN;
    {   double L=0.0,D=0.0;terminal_aero_q(g,t,v,aoa_max,&L,&D);
        pull_capability=L*fmax(t->dynamic_pressure,1.0)*g->terminal_lift_ratio/
            fmax(t->mass,1.0)-g0*cos(gamma); }
    double arrest_accel=clampd(.4*pull_capability,.12*g0,ARREST_ACCEL);
    double lift_q_max=0.0,drag_q_max=0.0;
    terminal_aero_q(g,t,v,aoa_max,&lift_q_max,&drag_q_max);
    double rho=isfinite(t->atmospheric_density)&&t->atmospheric_density>0.0?
        t->atmospheric_density:2.0*fmax(t->dynamic_pressure,1.0)/(V*V);
    double speed_1g=sqrt(2.0*t->mass*g0/fmax(1e-6,rho*lift_q_max*g->terminal_lift_ratio));
    double sink_now=fmax(0.0,-t->vertical_speed);
    double pull_height=fmax(0.0,sink_now*sink_now-SGS_SINK*SGS_SINK)/(2.0*arrest_accel);
    double energy_height=fmax(0.0,V*V-v_td*v_td)/(2.0*g0);
    double glide_tan=fmax(tan(4.0*DEG2RAD),fabs(tan(g->terminal_aim_valid?
        terminal_geometry_fpa(height,fmax(120.0,g->terminal_aim_along-along),0.0):gamma)));
    /* Horizontal distance flown from preflare start to touchdown: the
       constant-deceleration arc (time sink/a) plus the float, which dissipates
       the speed above touchdown speed and the shallow-glide height at LD_f. */
    double arc_distance=Vh*sink_now/arrest_accel*.6;
    double float_distance=ld_float*(energy_height+SGS_HEIGHT);
    double preflare_height=pull_height+SGS_HEIGHT;
    double aim=touchdown_target-arc_distance-float_distance+preflare_height/glide_tan;
    aim=clampd(aim,touchdown_target-6000.0,touchdown_target);
    if(!g->terminal_pull_latch){
        if(!g->terminal_aim_valid){g->terminal_aim_along=aim;g->terminal_aim_valid=true;}
        else{
            double tau=4.0,a=clampd(dt/(tau+dt),0.0,1.0);
            g->terminal_aim_along+=a*(aim-g->terminal_aim_along);
        }
        g->terminal_aim_ut=t->ut;
    }
    double aim_distance=fmax(120.0,g->terminal_aim_along-along);
    double glide_fpa=terminal_geometry_fpa(height,aim_distance,0.0);
    /* Outer-glide speed closure.  The flare needs speed in hand: the arc is
       flown near maximum incidence and bleeds speed, and below the 1-g speed
       at that incidence it cannot hold the shallow glide.  The Shuttle held
       its outer-glide reference speed with the speedbrake; here the glide
       angle does it (steeper is faster on the front side of the polar).
       V_1g comes from the learned lift at the incidence limit. */
    /* The learned table tends to over-predict lift at the incidence limit
       until that incidence has been flown, so never trust a 1-g speed below
       the configured touchdown speed. */
    /* Energy-consistent outer-glide speed: predict the touchdown speed the
       planned profile delivers and ask for the speed whose extra kinetic
       energy makes up the shortfall (or sheds the excess).  The floor keeps
       the stall margin the arc needs at its incidence limit. */
    double speed_ref=1.6*fmax(speed_1g,v_td);
    double predicted_touchdown_speed=NAN;
    if(!g->terminal_pull_latch){
        TerminalFlarePlan plan={arrest_accel,SGS_HEIGHT,SGS_SINK,FINAL_FLARE_ACCEL,H_FLOAT,S_FLOAT,
            g->terminal_lift_ratio,aoa_max,g0,p,site->altitude};
        predicted_touchdown_speed=terminal_predict_touchdown_speed(g,t,v,height,V,sink_now,false,&plan);
        speed_ref=fmax(1.3*fmax(speed_1g,v_td),
            sqrt(fmax(0.0,V*V+v_td*v_td-predicted_touchdown_speed*predicted_touchdown_speed)));
    }
    if(!g->terminal_pull_latch&&isfinite(speed_ref))
        glide_fpa=clampd(glide_fpa+clampd(.8*(V-speed_ref),-12.0,6.0)*DEG2RAD,
            -35.0*DEG2RAD,-3.0*DEG2RAD);

    /* Preflare latch: the height a constant design pull needs to take the
       present sink down to the shallow-glide sink, plus the height lost while
       the incidence swings to the pull.  That swing is sized from the lift
       slope applied to the lift increment over the *measured* lift (the table
       level can be tens of percent off at incidences not yet flown; its slope
       is the better-known quantity), at the flare slew rate. */
    double aoa_pull=aoa_max;
    {   double La=0.0,Da=0.0,Lb=0.0,Db=0.0,a0=clampd(t->angle_of_attack,0.0,aoa_max);
        terminal_aero_q(g,t,v,fmax(0.0,a0-1.0),&La,&Da);
        terminal_aero_q(g,t,v,fmin(aoa_max+1.0,a0+1.0),&Lb,&Db);
        double slope=(Lb-La)/(fmin(aoa_max+1.0,a0+1.0)-fmax(0.0,a0-1.0));
        double needed=t->mass*(g0*cos(gamma)+arrest_accel)/fmax(.3,lift_div);
        if(slope>1e-3&&isfinite(t->lift_force)&&t->dynamic_pressure>100.0)
            aoa_pull=clampd(a0+(needed-t->lift_force)/(t->dynamic_pressure*slope),0.0,aoa_max);
    }
    const double FLARE_SLEW=5.0; /* deg/s incidence command slew in the flare */
    double response=clampd(1.5+fmax(0.0,aoa_pull-t->angle_of_attack)/FLARE_SLEW,1.5,8.0);
    double response_height=sink_now*response;
    if(height<=preflare_height+response_height)
        g->terminal_pull_latch=true;

    /* Long landing: an unpowered vehicle cannot go around.  Past the point
       where the float would leave too little runway, stop floating and put
       the mains down firmly. */
    bool long_landing=along>touchdown_target+.35*runway_length;
    double float_height=long_landing?0.0:H_FLOAT;
    double float_sink=long_landing?1.8:S_FLOAT;
    bool flaring=g->terminal_pull_latch;
    double sink=-t->vertical_speed;
    double reference_fpa=glide_fpa;
    double sink_rate_ff=0.0;
    double arc_accel=NAN;
    if(flaring&&height>SGS_HEIGHT){
        /* Preflare arc: the deceleration that takes the present sink to the
           shallow-glide sink at the shallow-glide height, less the height
           lost while the incidence responds, re-solved every cycle so it is
           itself the closed loop.  The latch is sized so this starts at the
           design pull, and it fades to nothing as the sink reaches its goal
           instead of carrying a full design pull into the last metres (a
           balloon).  There is no hold-then-pull switch: holding until the
           required pull reaches the design value leaves the incidence lag to
           turn it into a late hard pull.  An overshoot below the shallow-glide
           sink, or a climb, is flown back to it rather than held while the
           float speed bleeds away. */
        const double PREFLARE_TAU=1.5;
        double room=fmax(3.0,height-SGS_HEIGHT-fmax(0.0,sink)*.5*response);
        arc_accel=sink>SGS_SINK?(sink*sink-SGS_SINK*SGS_SINK)/(2.0*room):
            (sink-SGS_SINK)/PREFLARE_TAU;
        double arc_goal=sink>SGS_SINK?fmax(SGS_SINK,sink-arc_accel*PREFLARE_TAU):SGS_SINK;
        reference_fpa=fmax(glide_fpa,-asin(clampd(arc_goal/V,0.0,.95)));
    }else if(flaring){
        /* Shallow glide blending into the float.  It is tracked both ways
           (its sink never exceeds the shallow-glide sink): a vehicle that
           arrested high and holds level at the shallow-glide height only
           bleeds the float speed until it stalls onto the runway.  Near the
           ground it is never a reason to push over. */
        /* Sink is tracked directly (feedforward plus sink error over a 1 s
           time constant), with the profile read one second of sink ahead:
           the final flare lasts only a few seconds, so a gamma loop with a
           2 s time constant behind a 1.5 s incidence lag starts the arrest
           after the flare is already over and the mains hit at the
           shallow-glide sink. */
        const double NO_PUSH_HEIGHT=3.0*H_FLOAT,FLARE_TAU=1.0,FLARE_LEAD=1.0;
        double lead_height=fmax(0.0,height-fmax(0.0,sink)*FLARE_LEAD);
        double profile_sink=terminal_sink_profile(lead_height,float_height,float_sink,
            arrest_accel,SGS_HEIGHT,SGS_SINK,FINAL_FLARE_ACCEL);
        double reference_sink=height>NO_PUSH_HEIGHT?profile_sink:
            fmin(profile_sink,fmax(float_sink,sink));
        double flare_top=terminal_final_flare_height(float_height,float_sink,FINAL_FLARE_ACCEL,SGS_SINK);
        double ff=lead_height>float_height&&lead_height<=flare_top&&sink>float_sink?FINAL_FLARE_ACCEL:0.0;
        arc_accel=ff+(sink-reference_sink)/FLARE_TAU;
        reference_fpa=fmax(glide_fpa,-asin(fmin(.95,reference_sink/V)));
    }
    double reference_sink=V*sin(reference_fpa);
    double measured_gamma_rate=isfinite(t->lift_force)?
        (t->lift_force*cos(bank*DEG2RAD)/fmax(t->mass,1.0)-g0*cos(gamma))/V:0.0;
    double lead_gamma=gamma+1.5*measured_gamma_rate;
    /* Above the shallow glide the re-solved arc is itself the closed loop
       (falling behind raises the required deceleration), so its deceleration
       is commanded directly; a gamma-rate lead there fights the slow incidence
       response and produces a push-over halfway down the arc. */
    double vertical_accel=isfinite(arc_accel)?clampd(arc_accel,-.3*g0,1.5*g0):
        clampd(V*(sink_rate_ff+.5*(reference_fpa-lead_gamma)),-.3*g0,1.5*g0);
    double lift_required=t->mass*(vertical_accel+g0*cos(gamma))/lift_div/
        g->terminal_lift_ratio;
    double aoa=clampd(terminal_aoa_for_lift(g,t,v,fmax(0.0,lift_required),aoa_max),
        0.0,aoa_max);
    /* In the flare a lift shortfall is not recoverable, and the learned table
       can be biased by tens of percent at incidences not yet flown.  Take a
       local Newton step on the measured lift instead: only the table's slope
       is trusted, not its level. */
    if(flaring&&isfinite(t->lift_force)&&t->dynamic_pressure>100.0){
        double q=t->dynamic_pressure;
        double La=0.0,Da=0.0,Lb=0.0,Db=0.0;
        double a0=clampd(t->angle_of_attack,0.0,aoa_max);
        terminal_aero_q(g,t,v,fmax(0.0,a0-1.0),&La,&Da);
        terminal_aero_q(g,t,v,fmin(aoa_max+1.0,a0+1.0),&Lb,&Db);
        double slope=(Lb-La)/(fmin(aoa_max+1.0,a0+1.0)-fmax(0.0,a0-1.0)); /* m^2/deg */
        if(slope>1e-3){
            double needed=t->mass*(vertical_accel+g0*cos(gamma))/lift_div;
            aoa=clampd(a0+(needed-t->lift_force)/(q*slope),0.0,aoa_max);
        }
    }
    /* Establish the mains-first attitude before contact without a low-altitude
       pitch step.  The smoothstep begins gently at 26 m main-wheel height and
       reaches the full 8 deg only near 5 m, so the incidence floor assists sink
       arrest without exciting the live pitch loop. */
    if(flaring&&height<=TOUCHDOWN_ATTITUDE_START){
        double u=clampd((TOUCHDOWN_ATTITUDE_START-height)/
            (TOUCHDOWN_ATTITUDE_START-TOUCHDOWN_ATTITUDE_FULL),0.0,1.0);
        u=u*u*(3.0-2.0*u);
        double attitude_floor=TOUCHDOWN_PITCH*u;
        double touchdown_aoa=clampd(attitude_floor-t->flight_path_angle,0.0,aoa_max);
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
        double rate=flaring?FLARE_SLEW:6.0;
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
    terminal_speedbrake_closed(g); /* Unpowered landing contract: no airbrakes. */

    GuidanceCommand c=atmospheric(t,target_course,bank,v,0.0,
        g->airbrakes_deployed,flaring?PROFILE_FLARE:PROFILE_APPROACH);
    c.has_target_aoa=true;
    c.target_aoa=aoa;
    c.target_pitch=t->flight_path_angle+aoa;
    c.gear=t->gear||g->gear_command_latched;
    /* KSP can still report `flying` for a short interval after the mains touch.
       Carry runway-heading yaw hold through the terminal flare so first-main
       contact cannot create a yaw-control gap before the coarse landed latch. */
    if(flaring&&height<=15.0){
        c.heading_control_enabled=true;
        c.target_heading=site->runway_heading;
    }
    GuidancePhase phase=g->terminal_vertical_stage==TERMINAL_TOUCHDOWN_FLARE?
        PHASE_FLARE:PHASE_FINAL;
    g->phase=phase;
    char status[256];
    snprintf(status,sizeof(status),"%s: wheel height %.0f m, along %.0f m, cross %+.0f m, "
        "speed %.0f / %.0f m/s, sink %.1f / %.1f m/s, AoA %.1f deg, aim %.0f m.",
        phase==PHASE_FLARE?"Touchdown flare":flaring?"Preflare":"Final glide",
        height,along,t->runway_cross_track,V,speed_ref,t->vertical_speed,reference_sink,aoa,
        g->terminal_aim_along);
    const char*warn=NULL;
    if(long_landing)warn="Long landing: floating would leave too little runway; landing firmly now.";
    else if(V<v_td*.9)warn="Airspeed is below the touchdown target; the flare will be firm.";
    else if(along<0.0&&flaring&&V<v_td*1.1)warn="Energy is low for the runway threshold.";
    GuidanceResult r=result_make(phase,c,status,warn);
    trajectory_copy(&r.reference,ref);
    r=stabilized(g,r,t,v,s,dt);
    if(getenv("KSP_LANDER_FINAL_TRACE"))
        fprintf(stderr,"FINAL_TRACE g=%p ut=%.2f h=%.1f x=%.0f V=%.1f Vref=%.1f sink=%.1f latch=%d "
            "pfh=%.0f resp=%.1f arr=%.2f aim=%.0f glide=%.1f ref=%.1f acc=%.2f aoa=%.1f ld=%.2f long=%d vtd=%.1f\n",
            (void*)g,t->ut,height,along,V,speed_ref,sink_now,g->terminal_pull_latch,preflare_height,
            response,arrest_accel,g->terminal_aim_along,glide_fpa*RAD2DEG,reference_fpa*RAD2DEG,
            vertical_accel,aoa,ld_float,long_landing,predicted_touchdown_speed);
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
