/*
 * Post-capture approach, flare and touchdown.
 *
 * One continuous law replaces the stage-by-stage abort ladder.  Every cycle it
 * chooses a desired sink rate from the live vehicle/runway state:
 *
 *   energy   : reach runway height as airspeed decays to the touchdown speed,
 *              using the flare deceleration implied by the measured L/D;
 *   runway   : never so shallow that touchdown falls beyond the stopping
 *              margin, never so steep that it falls short of the threshold;
 *   flare    : never faster than the sink that can still be arrested with a
 *              bounded vertical acceleration before contact.
 *
 * The sink error becomes a vertical-acceleration demand, which is converted to
 * a lift demand and inverted through the learned lift slope to an AoA command.
 * Energy is managed by path shape (float versus dive) and incidence, and by
 * the speedbrake when the vessel has one: its drag is measured in flight and
 * removed from the clean-aircraft model before the incidence tables learn.
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
    /* Separate scales from high-incidence flight, where the flare is flown:
       the model lift-curve shape does not match across incidence. */
    if(t->angle_of_attack>=11.0&&lf>.05&&df>.05&&t->lift_force>0.0&&t->drag_force>0.0){
        double kl=t->lift_force/(q*lf),kd=t->drag_force/(q*df);
        g->terminal_lift_k_hi=g->terminal_lift_k_hi>0.0?g->terminal_lift_k_hi*.9+kl*.1:kl;
        g->terminal_drag_k_hi=g->terminal_drag_k_hi>0.0?g->terminal_drag_k_hi*.9+kd*.1:kd;
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

/* Forward-simulate a maximum-rate pull from the present state with the
   learned lift and drag scales and the model Mach trend.  Returns the height
   at which the sink is arrested to sink_td (negative when the ground comes
   first, scaled by the excess sink at contact) and the airspeed there. */
__attribute__((unused)) static double terminal_flare_predict(const GuidanceMachine*g,const Telemetry*t,const VehicleProfile*v,
                                     double g0,double height,double aoa_max,double aoa_rate,double sink_td,double*v_arr){
    double V=fmax(1.0,t->true_air_speed),m=t->mass;
    double sound=t->mach>.02?V/t->mach:340.0,rho=2.0*fmax(t->dynamic_pressure,1.0)/(V*V);
    double kl=g->terminal_lift_k_hi>0.0?g->terminal_lift_k_hi:g->terminal_lift_area_ema>0.0?g->terminal_lift_area_ema:100.0;
    double kd=g->terminal_drag_k_hi>0.0?g->terminal_drag_k_hi:g->terminal_drag_area_ema>0.0?g->terminal_drag_area_ema:50.0;
    double vx=fmax(1.0,t->horizontal_speed),vz=t->vertical_speed,h=height,a=t->angle_of_attack,dt=.05;
    if(v_arr)*v_arr=V;
    if(!(m>1.0))return height;
    for(double s=0.0;s<25.0;s+=dt){
        if(-vz<=sink_td){if(v_arr)*v_arr=hypot(vx,vz);return h;}
        if(h<=0.0){if(v_arr)*v_arr=hypot(vx,vz);return -2.0*(-vz-sink_td);}
        if(s>=.5)a=fmin(aoa_max,a+aoa_rate*dt);
        double sp=hypot(vx,vz),q=.5*rho*sp*sp,lf=0.0,df=0.0;
        aerodynamic_force_factors_mach(sp/sound,a,v,&lf,&df);
        double L=.9*kl*lf*q,D=kd*df*q,c=vx/sp,sn=vz/sp;
        vx+=(-D*c-L*sn)/m*dt;vz+=((-D*sn+L*c)/m-g0)*dt;h+=vz*dt;vx=fmax(vx,1.0);
    }
    if(v_arr)*v_arr=hypot(vx,vz);
    return h;
}

static double q0(const Telemetry*t){return fmax(t->dynamic_pressure,1.0);}
/* Fly a pull-up forward: incidence slews toward a_cmd at the rate the
   vehicle can actually pitch, lift scaled by what the wing is producing.
   Returns the height at which the path has come up to gam_end. */
__attribute__((unused)) static double terminal_pull_predict(const GuidanceMachine*g,const Telemetry*t,const VehicleProfile*v,
        double g0,double height,double a_cmd,double rate,double delay,double gam_end,double*v_end){
    double V=fmax(1.0,t->true_air_speed),m=fmax(t->mass,1.0),gam=t->flight_path_angle*DEG2RAD;
    double rho=2.0*fmax(t->dynamic_pressure,1.0)/(V*V),lr=1.0;(void)g;
    double gprev=-9.0,a=t->angle_of_attack,h=height,dt=.05,sound=t->mach>.02?V/t->mach:340.0;
    double kl=g->terminal_lift_area_ema>0.0?g->terminal_lift_area_ema:100.0,kd=g->terminal_drag_area_ema>0.0?g->terminal_drag_area_ema:50.0;
    for(double s=0.0;s<30.0;s+=dt){
        if(-V*sin(gam)<=gam_end)break;
        if(s>3.0&&gam<gprev-1e-5){double e=-V*sin(gam)-gam_end;h-=e*e/(2.0*.3*g0);break;}
        gprev=gam;
        if(s>=delay)a+=clampd(a_cmd-a,-rate*dt,rate*dt);
        double lf=0.0,df=0.0,q=.5*rho*V*V;aerodynamic_force_factors_mach(V/sound,a,v,&lf,&df);
        double L=kl*lf,D=kd*df;
        V+=(-D*q/m-g0*sin(gam))*dt;
        if(V<5.0)break;
        gam+=(lr*L*q/m-g0*cos(gam))/V*dt;
        h+=V*sin(gam)*dt;
    }
    if(v_end)*v_end=V;
    return h;
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

static double KP(const char*n,double d){const char*e=getenv(n);return e?atof(e):d;}
static GuidanceResult terminal_landing_guidance(GuidanceMachine*g,const Telemetry*t,double course,
        const PlanetModel*p,const LandingConfiguration*cfg,const Trajectory*ref,double dt){
    const GuidanceSettings*s=&cfg->guidance;const VehicleProfile*v=&cfg->vehicle;const LandingSite*site=&cfg->site;
    double g0=planet_surface_gravity(p);
    double height=t->mean_altitude-site->altitude;
    if(isfinite(t->radar_altitude)&&t->radar_altitude>=0.0&&terminal_runway_contact_position(t,cfg))
        height=fmin(height,t->radar_altitude);
    {   /* Fly the main wheels, not the CG: they hang 5.5 m below and 3 m aft. */
        double th=(t->flight_path_angle+t->angle_of_attack)*DEG2RAD;
        height-=5.5*cos(th)+3.0*sin(th);
    }
    height=fmax(0.0,height);
    double V=fmax(1.0,t->true_air_speed),Vh=fmax(1.0,t->horizontal_speed);
    double along=t->runway_along_track;
    double gamma=t->flight_path_angle*DEG2RAD;
    terminal_learn_aero(g,t,v);

    double aoa_max=final_lift_aoa_limit(t,v);
    double v_td=v->touchdown_speed>1.0?v->touchdown_speed:65.0;
    double sink_td=fmax(.6,fabs(s->touchdown_sink_rate));
    double sink=fmax(0.0,-t->vertical_speed);

    /* Lift-to-drag in steady 1 g flight at the present airspeed, from the
       learned aero scale factors: at high dynamic pressure the needed
       incidence is small and the glide is draggy. */
    double ld_1g=2.0;
    if(t->mass>1.0){
        double a1=terminal_aoa_for_lift(g,t,v,t->mass*g0,aoa_max),L=0.0,D=0.0;
        terminal_aero_q(g,t,v,a1,&L,&D);
        if(D>1e-6)ld_1g=L/D;
    }
    ld_1g=clampd(ld_1g,.8,4.0);
    /* Pull-up authority: the vehicle just holds 1 g at maximum incidence
       near its touchdown speed, so the available load factor scales with
       (V / v_td)^2. This is smooth, unlike instantaneous lift estimates. */
    double v_1g=g->terminal_v1g_ema>0.0?g->terminal_v1g_ema:v_td;
    if(t->mass>1.0&&t->angle_of_attack>=10.0&&isfinite(t->lift_force)&&t->lift_force>0.0&&t->mach<=.8){
        /* Speed at which maximum-incidence lift carries 1 g, from the lift
           flown now corrected along the model lift curve to aoa_max. */
        double la=0.0,lm=0.0,d=0.0;
        aerodynamic_force_factors_mach(t->mach,t->angle_of_attack,v,&la,&d);
        aerodynamic_force_factors_mach(t->mach,aoa_max,v,&lm,&d);
        if(la>.05&&lm>=la){
            double s1=V*sqrt(t->mass*g0/(t->lift_force*lm/la));
            if(isfinite(s1)&&s1>20.0)v_1g=g->terminal_v1g_ema=g->terminal_v1g_ema>0.0?g->terminal_v1g_ema+fmin(1.0,dt/3.0)*(s1-g->terminal_v1g_ema):s1;
        }
    }
    double n_max=fmax(1.0,(V/v_1g)*(V/v_1g));
    double a_pull=clampd(.5*g0*(n_max-1.0),.05*g0,1.2*g0);
    /* Smooth the authority estimate so the sink limit does not chatter. */
    if(g->terminal_apull_ema>0.0)a_pull=g->terminal_apull_ema+=fmin(1.0,dt/2.0)*(a_pull-g->terminal_apull_ema);
    else g->terminal_apull_ema=a_pull;
    if(getenv("KSP_LANDER_FINAL_DEBUG")&&fmod(t->ut,5.0)<dt*0.999){
        fprintf(stderr,"FINALTAB %p v1g=%.1f nmax=%.2f q=%.0f",(void*)g,v_1g,n_max,t->dynamic_pressure);
        for(int i=0;i<=20;++i)if(g->terminal_aero_seen[i])fprintf(stderr," %d:%.0f@%.2f",i,g->terminal_lift_q[i],g->terminal_aero_mach[i]);
        fprintf(stderr,"\n");
    }
    TerminalVerticalStage st=g->terminal_vertical_stage;
    if(st<TERMINAL_OUTER_FINAL)st=TERMINAL_OUTER_FINAL;
    /* No touchdown plan.  Three smooth phases, each flown from the present
       state:
       glide     hold an approach airspeed on incidence (speed on pitch), so
                 the steep glide is steady and the pull has authority in hand;
       preflare  start pulling when a forward simulation of the pull says it
                 only just arrests the sink above the ground;
       flare     sink proportional to wheel height (exponential flare), the
                 rate constant relaxing from the pull-up entry to the float.
       Where the wheels touch is a consequence, not a target. */
    double drag=t->mass>1.0&&isfinite(t->drag_force)?t->drag_force:0.0;
    double v_dot=-drag/fmax(t->mass,1.0)-g0*sin(gamma);
    double pull_rate=KP("KP_R",1.0);
    if(g->terminal_vs_prev_ut>0.0&&t->ut>g->terminal_vs_prev_ut+1e-3&&t->ut<g->terminal_vs_prev_ut+2.0){
        double r=(t->vertical_speed-g->terminal_vs_prev)/(t->ut-g->terminal_vs_prev_ut);
        if(isfinite(r))g->terminal_vs_rate_ema+=.3*(clampd(r,-30.0,30.0)-g->terminal_vs_rate_ema);
    }
    g->terminal_vs_prev=t->vertical_speed;g->terminal_vs_prev_ut=t->ut;
    double vs_lead=t->vertical_speed+KP("KP_LEAD",1.0)*g->terminal_vs_rate_ema;

    const double V_GLIDE=KP("KP_VG",115.0),FLARE_C=KP("KP_C",.2),H_MARGIN=KP("KP_HM",20.0);
    double h_arrest=INFINITY,v_arr=V;
    double pull_response_height=terminal_pull_lead_height(fmax(0.0,-V*sin(gamma)),fmax(0.0,-vs_lead));
    if(!getenv("KP_FPA")&&!g->terminal_pull_latch){
        h_arrest=terminal_flare_predict(g,t,v,g0,height,aoa_max,pull_rate,sink_td+FLARE_C*H_MARGIN,&v_arr);
        if(h_arrest<=H_MARGIN+pull_response_height||height<=15.0){
            g->terminal_pull_latch=true;
            g->terminal_flare_k=fmax(0.0,sink-sink_td)/fmax(height,5.0);
            g->terminal_flare_ut=t->ut;g->terminal_flare_aoa0=t->angle_of_attack;
            g->terminal_flare_captured=false;
            if(g->terminal_aoa_trim<0.0)g->terminal_aoa_trim=0.0;
        }
    }
    bool flaring=g->terminal_pull_latch;
    if(flaring&&g->terminal_flare_k>FLARE_C)g->terminal_flare_k+=fmin(1.0,dt/KP("KP_TK",3.0))*(FLARE_C-g->terminal_flare_k);
    double kf=flaring?g->terminal_flare_k:FLARE_C;
    if(st==TERMINAL_TOUCHDOWN_FLARE||(flaring&&height<=15.0))st=TERMINAL_TOUCHDOWN_FLARE;
    else st=flaring?TERMINAL_PREFLARE:TERMINAL_OUTER_FINAL;
    double vs_des=flaring?-(sink_td+kf*height):t->vertical_speed;
    double s_energy=V-V_GLIDE,s_long=h_arrest;

    double lateral_scale=clampd(height/60.0,0.0,1.0);
    double track_err=norm_signed_deg(course-site->runway_heading)*DEG2RAD;
    double cross_rate=Vh*sin(track_err);
    /* Capture the centerline before the threshold instead of merely converging
       toward it at touchdown.  The second-order law remains state based, but its
       bandwidth is set by time remaining to a pre-threshold alignment station. */
    const double align_station=-2800.0;
    double align_remaining=fmax(0.0,align_station-along);
    double align_time=fmax(6.0,align_remaining/Vh);
    double omega=clampd(2.2/align_time,.06,.18);
    double lat=-2.0*.95*omega*cross_rate-
        omega*omega*clampd(t->runway_cross_track,-400.0,400.0);
    double banklim=fmin(fmin(30.0,terminal_lateral_bank_limit(t,v)),3.0+27.0*lateral_scale);
    double bank=clampd(atan(lat/g0)*RAD2DEG,-banklim,banklim);
    double lift_div=fmax(.5,cos(gamma)*cos(bank*DEG2RAD));

    double aoa,az=0.0;
    if(getenv("KP_FPA")){
        /* Glide: hold a steep flight-path angle, speed finds its own level.
           Pull: when a firm pull-up would only just level the wheels off at
           the float height, start it, re-solving each cycle for the
           incidence that levels off right there.  Float: settle slowly. */
        if(g->terminal_lift_ratio<=0.0)g->terminal_lift_ratio=1.0;
        {   double Lm=0.0,Dm=0.0;terminal_aero_q(g,t,v,t->angle_of_attack,&Lm,&Dm);
            double Lq=Lm*fmax(t->dynamic_pressure,1.0);
            if(Lq>.2*t->mass*g0&&isfinite(t->lift_force)&&t->lift_force>0.0)
                g->terminal_lift_ratio+=fmin(1.0,dt/KP("KP_LRT",2.0))*(clampd(t->lift_force/Lq,.6,1.6)-g->terminal_lift_ratio);
        }
        const double H_FLOAT=KP("KP_HT",2.0),S_FLOAT=KP("KP_STD",.5);
        /* Aim the outer glide at a real touchdown point on the runway instead of
           switching from an energy gate to a fixed steep slope.  This keeps the
           reference geometric and continuous from final capture through preflare. */
        /* Live gear contact trails this reference point; bias upstream so
           the low-sink corridor reaches the runway with rollout distance left. */
        /* k70's -200 m aim hit terrain just short; k67's -100 m aim
           contacted at +162 m and overran by 83 m. Keep most of the stopping
           margin while moving wheel contact onto the runway. */
        double touchdown_along=-220.0;
        double touchdown_distance=fmax(120.0,touchdown_along-along);
        double gref=terminal_geometry_fpa(height,touchdown_distance,H_FLOAT);
        double gdot_ff=0.0,kg=KP("KP_KG",.3);
        double lift_req=-1.0;
        if(getenv("KP_PD")){double lf=0,df=0;aerodynamic_force_factors_mach(t->mach,t->angle_of_attack,v,&lf,&df);
            fprintf(stderr,"AERO %p h=%.0f V=%.1f aoa=%.1f Lr=%.2f Dr=%.2f\n",(const void*)g,height,V,t->angle_of_attack,
            t->lift_force/(g->terminal_lift_area_ema*lf*q0(t)),t->drag_force/(g->terminal_drag_area_ema*df*q0(t)));}

        /* The flare corridor is a one-way transition.  Its sink envelope is
           height-only, so once capture begins the reference can only become
           shallower than the geometric runway glide; it never unloads back into
           a dive because the instantaneous sink happened to improve. */
        double a_arr=KP("KP_AF",.35)*g0;
        /* Start the sink-arrest profile before the geometric intercept by the
           height lost during the measured pull response.  k17 needed about
           seven seconds to reduce sink from 32 m/s to below 8 m/s. */
        double glide_sink=fmax(S_FLOAT,-V*sin(gref));
        double response_height=terminal_pull_lead_height(glide_sink,-vs_lead);
        double hf=fmax(0.0,height-H_FLOAT),kx=KP("KP_FK",.14);
        double g_prof=terminal_sink_profile_fpa(height,V,H_FLOAT,S_FLOAT,a_arr,kx);
        double future_height=fmax(0.0,height-response_height);
        double future_along=along+Vh*terminal_pull_response_seconds();
        double future_distance=fmax(120.0,touchdown_along-future_along);
        double future_gref=terminal_geometry_fpa(future_height,future_distance,H_FLOAT);
        double future_gprof=terminal_sink_profile_fpa(
            future_height,V,H_FLOAT,S_FLOAT,a_arr,kx);
        if(future_gprof>future_gref)g->terminal_pull_latch=true;
        bool onprof=g->terminal_pull_latch;
        if(onprof){gref=fmax(gref,g_prof);kg=KP("KP_KGF",.55);}
        double sink=-t->vertical_speed;
        if(onprof&&hf>0.0)
            gdot_ff=(sqrt(2.0*a_arr*hf)<kx*hf?
                a_arr/(sqrt(2.0*a_arr*hf)+1e-3):kx)*sink/
                (V*fmax(.3,cos(gref)));
        {
            double gdot=isfinite(t->lift_force)?(t->lift_force*cos(bank*DEG2RAD)/fmax(t->mass,1.0)-g0*cos(gamma))/V:0.0;
            double g_lead=gamma+KP("KP_LEADG",1.5)*gdot;
            double an=clampd(V*(gdot_ff+kg*(gref-g_lead)),-KP("KP_ANMIN",.3)*g0,1.5*g0);
            az=an;
            lift_req=t->mass*(an+g0*cos(gamma))/lift_div/g->terminal_lift_ratio;
            aoa=clampd(terminal_aoa_for_lift(g,t,v,fmax(0.0,lift_req),aoa_max),0.0,aoa_max);
        }
        vs_des=V*sin(gref);
        flaring=g->terminal_pull_latch;g->terminal_flare_captured=g->terminal_flare_captured;kf=0.0;
    }else if(getenv("KP_UNI")){
        /* One law: sink target from speed error, capped by the flare profile. */
        double c=KP("KP_C",.2);
        double s_glide=clampd(KP("KP_SG0",45.0)-KP("KP_KS",1.0)*(V-V_GLIDE),KP("KP_SMIN",20.0),90.0);
        double s_prof=KP("KP_STD",.5)+c*fmax(0.0,height-KP("KP_HF",2.0));
        bool onprof=s_prof<s_glide;
        vs_des=-fmin(s_glide,s_prof);
        az=clampd((onprof?c*sink:0.0)+(onprof?KP("KP_KPF",.4):KP("KP_KP",.15))*(vs_des-vs_lead),-.3*g0,1.5*g0);
        double lift_req=(t->mass*(az+g0)+drag*sin(gamma))/lift_div;
        aoa=terminal_aoa_for_lift(g,t,v,fmax(0.0,lift_req),aoa_max);
        double vs_err=vs_des-vs_lead;
        double trim=clampd(g->terminal_aoa_trim+KP("KP_KI",.05)*vs_err*dt,-3.0,4.0);
        bool saturated=(aoa>=aoa_max-.05&&vs_err>0.0)||(aoa<=.05&&vs_err<0.0);
        if(isfinite(trim)&&!saturated)g->terminal_aoa_trim=trim;
        aoa=clampd(aoa+g->terminal_aoa_trim,0.0,aoa_max);
        flaring=onprof;kf=c;
    }else if(!flaring){
        /* Speed on pitch: fast -> more incidence, accelerating -> more. */
        double a_eq=terminal_aoa_for_lift(g,t,v,t->mass*g0*cos(gamma)/lift_div,aoa_max);
        aoa=a_eq+KP("KP_KV",.12)*(V-V_GLIDE)+KP("KP_KD",.8)*v_dot;
        aoa=clampd(aoa,1.0,fmin(aoa_max,14.0));
    }else if(!g->terminal_flare_captured&&!getenv("KP_CL")){
        /* Pull-up: the same incidence ramp the trigger simulated, so the
           arrest happens where it was predicted.  Hand over to the sink
           profile once the sink is down to it. */
        aoa=fmin(aoa_max,g->terminal_flare_aoa0+pull_rate*fmax(0.0,t->ut-g->terminal_flare_ut-.5));
        if(sink<=sink_td+FLARE_C*height+1.0){
            g->terminal_flare_captured=true;g->terminal_flare_k=FLARE_C;g->terminal_aoa_trim=0.0;
        }
    }else{
        /* Track the exponential sink profile: feed forward its own curvature
           (d/dt of k*h is -k*sink) plus proportional feedback. */
        az=clampd(kf*sink+KP("KP_KP",.8)*(vs_des-vs_lead),-.1*g0,1.5*g0);
        double lift_req=(t->mass*(az+g0)+drag*sin(gamma))/lift_div;
        aoa=terminal_aoa_for_lift(g,t,v,fmax(0.0,lift_req),aoa_max);
        double vs_err=vs_des-vs_lead;
        double trim=clampd(g->terminal_aoa_trim+KP("KP_KI",.1)*vs_err*dt,-2.0,4.0);
        bool saturated=(aoa>=aoa_max-.05&&vs_err>0.0)||(aoa<=.05&&vs_err<0.0);
        if(isfinite(trim)&&!saturated)g->terminal_aoa_trim=trim;
        aoa=clampd(aoa+g->terminal_aoa_trim,0.0,aoa_max);
    }
    /* Slew the flare incidence more slowly than outer-final corrections.  The
       flare corridor is one-way, so this produces one continuous pitch pull rather
       than making the inner loop chase force-inversion noise. */
    if(g->terminal_aoa_cmd_ut>0.0&&t->ut>g->terminal_aoa_cmd_ut&&t->ut<g->terminal_aoa_cmd_ut+2.0){
        double rate=getenv("KP_FPA")?
            (flaring?KP("KP_SR_FLARE",1.5):KP("KP_SR",6.0)):
            (flaring?(g->terminal_flare_captured?KP("KP_SF",5.0):20.0):KP("KP_SG",2.0));
        double r=rate*(t->ut-g->terminal_aoa_cmd_ut);
        aoa=clampd(aoa,g->terminal_aoa_cmd_last-r,g->terminal_aoa_cmd_last+r);
    }else aoa=t->angle_of_attack;
    g->terminal_aoa_cmd_last=aoa;g->terminal_aoa_cmd_ut=t->ut;
    if(getenv("KSP_LANDER_FINAL_DEBUG")&&fmod(t->ut,1.0)<dt*0.999)
        fprintf(stderr,"FINALDBG %p st=%d alt=%.1f site=%.1f h=%.1f x=%.0f V=%.1f vs=%.1f vsd=%.1f k=%.3f harr=%.1f varr=%.1f az=%.2f lr=%.2f aoa=%.1f cmd=%.1f\n",
            (void*)g,(int)st,t->mean_altitude,site->altitude,height,along,V,t->vertical_speed,vs_des,kf,h_arrest,v_arr,az,g->terminal_lift_ratio,t->angle_of_attack,aoa);

    double inter=clampd(atan2(-t->runway_cross_track,fmax(400.0,V*6.0))*RAD2DEG,-20.0,20.0);
    double target_course=norm_deg(site->runway_heading+inter);

    if(height<KP("KP_GEAR",90.0))g->gear_command_latched=true;
    TerminalVerticalStage stage=st;
    if(g->terminal_vertical_stage!=stage)terminal_set_stage(g,stage,t->ut);
    if(g->terminal_vertical_stage==TERMINAL_TOUCHDOWN_FLARE)g->flare_sink_captured=true;

    /* Speedbrake: shed speed above the approach cap while there is height to
       spare, retract with hysteresis and always for the flare. */
    if(t->has_airbrakes&&!flaring&&height>KP("KP_SBH",60.0)){
        double v_cap=KP("KP_SBV",125.0);
        if(V>v_cap)g->airbrakes_deployed=true;
        else if(V<v_cap-5.0)g->airbrakes_deployed=false;
    }else g->airbrakes_deployed=false;
    GuidanceCommand c=atmospheric(t,target_course,bank,v,0.0,g->airbrakes_deployed,
        flaring?PROFILE_FLARE:PROFILE_APPROACH);
    c.has_target_aoa=true;
    c.target_aoa=aoa;
    c.target_pitch=t->flight_path_angle+aoa;
    c.gear=t->gear||g->gear_command_latched;
    GuidancePhase phase=g->terminal_vertical_stage==TERMINAL_TOUCHDOWN_FLARE?PHASE_FLARE:PHASE_FINAL;
    g->phase=phase;
    char status[320];
    snprintf(status,sizeof(status),
        "%s: h %.0f m, along %.0f m, cross %+.0f m, V %.0f m/s, sink %.1f / %.1f m/s "
        "(energy %.1f, long %.1f), AoA %.1f deg.",
        stage==TERMINAL_TOUCHDOWN_FLARE?"Touchdown flare":stage==TERMINAL_INNER_FINAL?"Shallow glide":
        (flaring?"Preflare":"Final glide"),
        height,along,t->runway_cross_track,V,t->vertical_speed,vs_des,-s_energy,-s_long,aoa);
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
            site->runway_heading,track_err*RAD2DEG,cross_rate,omega,lat,
            banklim,bank,target_course,r.command.target_roll,t->roll,
            t->roll_rate);
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
