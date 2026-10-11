/* Experimental native-stage forecast: reproduces the Final latch, onset, arc
   and shallow-glide commands, the flare incidence slew and an optional
   first-order incidence lag, with learned force model frozen at the call. */
static double experiment_native_stage_forecast(const GuidanceMachine*g,const Telemetry*t,
        const VehicleProfile*v,const TerminalFlarePlan*p,double height,double V,double sink,
        double lift_div,double cmd0,double aoa_lag,bool latched,double*sink_out){
    const double SGS_H=12.0,SGS_S=3.5,SLEW=5.0,DT=.1,PTAU=1.5,FTAU=1.0;
    const double ARC_LAG=1.0,H_FLOAT=1.5,S_FLOAT=1.0,NOPUSH=4.5;
    double m=fmax(t->mass,1.0),g0=p->g0;
    double sound=isfinite(t->mach)&&t->mach>.05&&isfinite(t->true_air_speed)?t->true_air_speed/t->mach:340.0;
    double rho_now=isfinite(t->atmospheric_density)&&t->atmospheric_density>0.0?
        t->atmospheric_density:2.0*fmax(t->dynamic_pressure,1.0)/fmax(V*V,1.0);
    double rho_m=planet_atmospheric_density(p->planet,t->mean_altitude);
    double snd_m=planet_atmospheric_speed_of_sound(p->planet,t->mean_altitude);
    double glide_gamma=-asin(clampd(sink/fmax(V,1.0),-.95,.95));
    double aoa=t->angle_of_attack,cmd=isfinite(cmd0)?cmd0:aoa;
    Telemetry tc=*t;
    const char*dump_env=getenv("KSL_NSF_DUMP_UT");
    bool dump=dump_env&&aoa_lag>0.0&&fabs(t->ut-strtod(dump_env,NULL))<.015;
    double sim_t=0.0;
    for(int i=0;i<6000&&height>0.0&&V>5.0;++i){
        double rho=rho_now,a=sound;
        if(rho_m>0.0&&snd_m>0.0){
            rho*=planet_atmospheric_density(p->planet,p->site_altitude+height)/rho_m;
            a*=planet_atmospheric_speed_of_sound(p->planet,p->site_altitude+height)/snd_m;
        }
        tc.mach=V/a;tc.dynamic_pressure=.5*rho*V*V;tc.angle_of_attack=aoa;
        double q=tc.dynamic_pressure;
        double gamma=-asin(clampd(sink/V,-.95,.95));
        double L=0.0,D=0.0;terminal_aero_q(g,&tc,v,aoa,&L,&D);
        double lift_acc=p->lift_ratio*L*q*lift_div/m;
        /* Native incidence-pull size and response time. */
        double La=0.0,Da=0.0,Lb=0.0,Db=0.0,a0=clampd(aoa,0.0,p->aoa_max);
        double lo=fmax(0.0,a0-1.0),hi=fmin(p->aoa_max+1.0,a0+1.0);
        terminal_aero_q(g,&tc,v,lo,&La,&Da);terminal_aero_q(g,&tc,v,hi,&Lb,&Db);
        double slope=(Lb-La)/(hi-lo),aoa_pull=p->aoa_max;
        double needed=m*(g0*cos(gamma)+p->arrest_accel)/fmax(.3,lift_div);
        if(slope>1e-3&&q>100.0)aoa_pull=clampd(a0+(needed-p->lift_ratio*L*q)/(q*slope*p->lift_ratio),0.0,p->aoa_max);
        double response=clampd(1.5+fmax(0.0,aoa_pull-aoa)/SLEW,1.5,8.0);
        if(isfinite(g->preflare_response_time)&&g->preflare_response_time>0.0&&g->terminal_preflare_plan_valid)
            response=fmax(response,g->preflare_response_time);
        double pull_h=fmax(0.0,sink*sink-SGS_S*SGS_S)/(2.0*p->arrest_accel);
        if(!latched){
            double lh=fmax(pull_h+SGS_H+sink*response,
                isfinite(p->preflare_trigger_altitude)?p->preflare_trigger_altitude:0.0);
            if(height<=lh)latched=true;
        }
        double vertical_accel,target,dbg_lr=0.0;
        if(!latched){
            sink=V*sin(-glide_gamma);gamma=glide_gamma;
            vertical_accel=0.0;target=aoa;
            double lr=m*g0*cos(gamma)/fmax(.3,lift_div)/p->lift_ratio;
            target=terminal_aoa_for_lift(g,&tc,v,lr,p->aoa_max);
        }else{
            if(height>SGS_H){
                double arc_lag=fmin(response,fmax(ARC_LAG,fabs(aoa_pull-aoa)/SLEW));
                double room=fmax(0.0,height-SGS_H-fmax(0.0,sink)*arc_lag);
                double design=sqrt(SGS_S*SGS_S+2.0*p->arrest_accel*room);
                double glide_sink=fmax(SGS_S,V*sin(-glide_gamma));
                double onset_room=fmax(0.0,height-SGS_H-fmax(0.0,sink)*(response+1.0));
                double onset_sink=sqrt(SGS_S*SGS_S+2.0*p->arrest_accel*onset_room);
                bool on_arc=sink<.92*glide_sink||onset_sink<sink;
                if(sink>SGS_S&&!on_arc){
                    double lead=gamma+1.5*((lift_acc-g0*cos(gamma))/V);
                    vertical_accel=clampd(V*.5*(glide_gamma-lead),-.3*g0,1.5*g0);
                }else{
                    double arc_sink=fmax(0.0,sink-ARC_LAG*fmax(0.0,lift_acc-g0*cos(gamma)));
                    double acc=sink>SGS_S?fmax(0.0,p->arrest_accel+(arc_sink-design)/PTAU):(sink-SGS_S)/PTAU;
                    vertical_accel=clampd(acc,-.3*g0,1.5*g0);
                }
            }else{
                double lead_h=fmax(0.0,height-fmax(0.0,sink)*1.0);
                double prof=terminal_sink_profile(lead_h,H_FLOAT,S_FLOAT,p->arrest_accel,SGS_H,SGS_S,p->final_accel);
                double ref=height>NOPUSH?prof:fmin(prof,fmax(S_FLOAT,sink));
                double top=terminal_final_flare_height(H_FLOAT,S_FLOAT,p->final_accel,SGS_S);
                double ff=lead_h>H_FLOAT&&lead_h<=top&&sink>S_FLOAT?p->final_accel:0.0;
                vertical_accel=clampd(ff+(sink-ref)/FTAU,-.3*g0,1.5*g0);
            }
            double lr=m*(vertical_accel+g0*cos(gamma))/fmax(.3,lift_div)/p->lift_ratio;
            dbg_lr=lr;target=terminal_aoa_for_lift(g,&tc,v,fmax(0.0,lr),p->aoa_max);
            /* Native flare: local Newton step on measured lift, trusting only the table slope. */
            if(q>100.0&&slope>1e-3){
                double need=m*(vertical_accel+g0*cos(gamma))/fmax(.3,lift_div);
                target=clampd(a0+(need-p->lift_ratio*L*q)/(q*slope),0.0,p->aoa_max);
            }
        }
        double rate=latched?SLEW:6.0;
        cmd=clampd(target,cmd-rate*DT,cmd+rate*DT);
        aoa=aoa_lag>0.0?aoa+(cmd-aoa)*DT/aoa_lag:cmd;
        if(dump&&i%5==0)fprintf(stderr,"NSFT t=%.2f h=%.3f V=%.3f sink=%.3f aoa=%.3f cmd=%.3f target=%.3f liftAcc=%.3f vAcc=%.3f latch=%d lr=%.0f Lq=%.0f lq45=%.0f lq5=%.0f lq55=%.0f\n",sim_t,height,V,sink,aoa,cmd,target,lift_acc,vertical_accel,latched,dbg_lr,L*q,({double l2=0,d2=0;terminal_aero_q(g,&tc,v,4.5,&l2,&d2);l2*q;}),({double l2=0,d2=0;terminal_aero_q(g,&tc,v,5.0,&l2,&d2);l2*q;}),({double l2=0,d2=0;terminal_aero_q(g,&tc,v,5.5,&l2,&d2);l2*q;}));
        sim_t+=DT;
        V+=(-D*q/m-g0*sin(gamma))*DT;
        if(latched)sink-=(lift_acc-g0*cos(gamma))*DT;
        height-=sink*DT;
    }
    if(sink_out)*sink_out=sink;
    return height<=0.0&&isfinite(V)&&V>0.0?V:NAN;
}
