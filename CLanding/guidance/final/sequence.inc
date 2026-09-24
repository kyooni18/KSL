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
 * The unpowered vehicle has no in-flight speedbrake authority, so energy is
 * managed only by path shape (float versus dive) and incidence.
 */

static void terminal_learn_aero(GuidanceMachine*g,const Telemetry*t,const VehicleProfile*v){
    double q=t->dynamic_pressure;
    if(!(q>200.0)||!isfinite(t->lift_force)||!isfinite(t->drag_force))return;
    double lf=0.0,df=0.0;
    aerodynamic_force_factors_mach(t->mach,t->angle_of_attack,v,&lf,&df);
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
    const GuidanceSettings*s=&cfg->guidance;const VehicleProfile*v=&cfg->vehicle;const LandingSite*site=&cfg->site;
    double g0=planet_surface_gravity(p);
    double height=t->mean_altitude-site->altitude;
    if(isfinite(t->radar_altitude)&&t->radar_altitude>=0.0&&terminal_runway_contact_position(t,cfg))
        height=fmin(height,t->radar_altitude);
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
    double n_max=fmax(1.0,(V/v_td)*(V/v_td));
    double a_pull=clampd(.7*g0*(n_max-1.0),.05*g0,1.2*g0);
    /* Smooth the authority estimate so the sink limit does not chatter. */
    if(g->terminal_apull_ema>0.0)a_pull=g->terminal_apull_ema+=fmin(1.0,dt/2.0)*(a_pull-g->terminal_apull_ema);
    else g->terminal_apull_ema=a_pull;
    if(getenv("KSP_LANDER_FINAL_DEBUG")&&fmod(t->ut,5.0)<dt*0.999){
        fprintf(stderr,"FINALTAB %p nmax=%.2f q=%.0f",(void*)g,n_max,t->dynamic_pressure);
        for(int i=0;i<=20;++i)if(g->terminal_aero_seen[i])fprintf(stderr," %d:%.0f@%.2f",i,g->terminal_lift_q[i],g->terminal_aero_mach[i]);
        fprintf(stderr,"\n");
    }
    TerminalVerticalStage st=g->terminal_vertical_stage;
    if(st<TERMINAL_OUTER_FINAL)st=TERMINAL_OUTER_FINAL;

    /* Touchdown window: past the threshold, early enough to stop with margin. */
    double stop=v_td*v_td/(2.0*.15*g0);
    double x_td=clampd(.25*(site->runway_length-stop),150.0,600.0);
    double x_long=clampd(site->runway_length-stop-150.0,x_td+200.0,site->runway_length*.7);

    /* Speed-hold glide: the unpowered glide is flown at the flare-entry
       speed, sinking at the equilibrium rate for the observed 1 g L/D and
       trading height for airspeed when slow.  Holding airspeed, not a fixed
       path, is what keeps pull-up authority for the flare. */
    double v_ref=1.5*v_td;
    /* Speed-on-pitch: sink more when slow or decelerating, less when fast.
       Built on the measured sink and the along-path acceleration from the
       actual drag, so it needs no lift-to-drag model. */
    double drag_acc=t->mass>1.0&&isfinite(t->drag_force)?t->drag_force/t->mass:0.0;
    double v_dot=-drag_acc-g0*sin(gamma);
    double s_energy=sink+.3*(v_ref-V)-1.5*v_dot;
    double s_long=along<x_long?height*Vh/fmax(50.0,x_long-along):Vh*sin(8.0*DEG2RAD);
    /* Range protection: never steeper than the straight line that still
       clears the threshold, while there is airspeed to spare for it. */
    /* The flare turns the dive level over ground it covers itself, so the
       straight-line aim may fall short of the runway by that distance. */
    double sink_now=fmax(0.0,-t->vertical_speed);
    double d_flare=clampd(Vh*sink_now/fmax(a_pull,.5)*.5,0.0,300.0);
    double s_short=along<x_td-100.0-d_flare?height*Vh/fmax(1.0,x_td-100.0-d_flare-along):INFINITY;
    /* With surplus speed, do not float past the long end of the window. */
    double s_glide=V>v_ref?fmax(s_energy,s_long):s_energy;
    s_glide=clampd(s_glide,V*sin(1.0*DEG2RAD),V*sin(35.0*DEG2RAD));
    if(V>1.15*v_td)s_glide=fmin(s_glide,s_short);
    /* Pull-up limit (with incidence lag) and the exponential final flare. */
    double h_eff=fmax(0.0,height-sink*2.5);
    double s_cap=sqrt(sink_td*sink_td+2.0*a_pull*h_eff);
    double s_ff=sink_td+height/3.0;
    double s_des=fmin(s_glide,fmin(s_cap,s_ff));
    if(st==TERMINAL_TOUCHDOWN_FLARE||height<=15.0){st=TERMINAL_TOUCHDOWN_FLARE;s_des=fmin(s_des,s_ff);}
    else if(s_cap<=s_glide||s_ff<=s_glide)st=TERMINAL_PREFLARE;
    else if(st<TERMINAL_PREFLARE)st=TERMINAL_OUTER_FINAL;
    bool flaring=st>=TERMINAL_PREFLARE;
    double vs_des=-s_des;
    if(getenv("KSP_LANDER_FINAL_DEBUG")&&fmod(t->ut,1.0)<dt*0.999)
        fprintf(stderr,"FINALDBG %p st=%d h=%.0f x=%.0f V=%.1f vs=%.1f sdes=%.1f sen=%.1f slong=%.1f sshort=%.1f scap=%.1f ld=%.2f apull=%.2f aoa=%.1f\n",
            (void*)g,(int)st,height,along,V,t->vertical_speed,s_des,s_energy,s_long,s_short,s_cap,ld_1g,a_pull,t->angle_of_attack);
    /* Vertical acceleration demand -> lift -> AoA. */
    double lateral_scale=clampd(height/60.0,0.0,1.0);
    /* Vertical acceleration estimate, used to lead the incidence response. */
    if(g->terminal_vs_prev_ut>0.0&&t->ut>g->terminal_vs_prev_ut+1e-3&&t->ut<g->terminal_vs_prev_ut+2.0){
        double r=(t->vertical_speed-g->terminal_vs_prev)/(t->ut-g->terminal_vs_prev_ut);
        if(isfinite(r))g->terminal_vs_rate_ema+=.3*(clampd(r,-30.0,30.0)-g->terminal_vs_rate_ema);
    }
    g->terminal_vs_prev=t->vertical_speed;g->terminal_vs_prev_ut=t->ut;
    double vs_lead=t->vertical_speed+.8*g->terminal_vs_rate_ema;
    /* Incidence changes take ~3 s, so never build more sink than the pull-up
       cap can absorb once the rotation completes. */
    double az_floor=fmax(-.30*g0,fmin(0.0,(-s_cap-t->vertical_speed)/3.0));
    double az=clampd(.45*(vs_des-vs_lead),az_floor,.6*g0);
    double drag=t->mass>1.0&&isfinite(t->drag_force)?t->drag_force:0.0;
    double lift_req=t->mass*(az+g0)+drag*sin(gamma);

    /* Lateral: damped cross-track state feedback, bank authority reduced near
       the ground so the wings are level at contact. */
    double track_err=norm_signed_deg(course-site->runway_heading)*DEG2RAD;
    double cross_rate=Vh*sin(track_err);
    double omega=clampd(.9*V/fmax(fabs(along)+800.0,800.0),.05,.16);
    double lat=-2.0*.9*omega*cross_rate-omega*omega*clampd(t->runway_cross_track,-400.0,400.0);
    double banklim=fmin(fmin(30.0,dynamic_bank_limit(t,v)),3.0+27.0*lateral_scale);
    double bank=clampd(atan(lat/g0)*RAD2DEG,-banklim,banklim);
    lift_req/=fmax(.5,cos(gamma)*cos(bank*DEG2RAD));
    double aoa=terminal_aoa_for_lift(g,t,v,fmax(0.0,lift_req),aoa_max);
    /* Close the loop on sink rate: the model lift curve is only an estimate,
       so a proportional-integral incidence trim removes its error. */
    double vs_err=vs_des-vs_lead;
    double trim=clampd(g->terminal_aoa_trim+.04*vs_err*dt,-2.0,4.0);
    bool saturated=(aoa>=aoa_max-.05&&vs_err>0.0)||(aoa<=.05&&vs_err<0.0);
    if(isfinite(trim)&&!saturated)g->terminal_aoa_trim=trim;
    /* Never unload below 0.7 g once sink approaches the pull-up cap: the incidence needed to recover from
       an unloaded dive takes seconds to rebuild, which the ground does not
       allow. */
    double floor_frac=.7*clampd((sink-.5*s_cap)/fmax(1.0,.3*s_cap),0.0,1.0);
    double aoa_floor=terminal_aoa_for_lift(g,t,v,floor_frac*t->mass*g0,aoa_max);
    aoa=clampd(fmax(aoa+g->terminal_aoa_trim,aoa_floor),0.0,aoa_max);
    if(getenv("KSP_LANDER_FINAL_DEBUG")&&fmod(t->ut,1.0)<dt*0.999)
        fprintf(stderr,"FINALCMD %p aoa_cmd=%.1f trim=%.1f bank=%.1f\n",(void*)g,aoa,g->terminal_aoa_trim,bank);

    double inter=clampd(atan2(-t->runway_cross_track,fmax(600.0,V*6.0))*RAD2DEG,-20.0,20.0);
    double target_course=norm_deg(site->runway_heading+inter);

    if(height<fmax(300.0,s->gear_deployment_altitude))g->gear_command_latched=true;
    TerminalVerticalStage stage=st;
    if(g->terminal_vertical_stage!=stage)terminal_set_stage(g,stage,t->ut);
    if(g->terminal_vertical_stage==TERMINAL_TOUCHDOWN_FLARE)g->flare_sink_captured=true;

    terminal_speedbrake_closed(g);
    GuidanceCommand c=atmospheric(t,target_course,bank,v,0.0,false,
        g->terminal_vertical_stage==TERMINAL_TOUCHDOWN_FLARE?PROFILE_FLARE:PROFILE_APPROACH);
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
    return stabilized(g,r,t,v,s,dt);
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
