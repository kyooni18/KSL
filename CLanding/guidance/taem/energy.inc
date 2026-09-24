       double terminal_fpa_force_aoa(const Telemetry*t,const PlanetModel*p,
        AerodynamicModel aero,const VehicleProfile*v,double reference_fpa){
    if(!t||!p||!v||!isfinite(reference_fpa)||!(t->true_air_speed>DBL_MIN))
        return NAN;
    double upper=v->maximum_angle_of_attack;
    if(t->mach<1.0&&isfinite(v->terminal_maximum_lift_angle_of_attack)&&
       v->terminal_maximum_lift_angle_of_attack>0.0)
        upper=fmin(upper,v->terminal_maximum_lift_angle_of_attack);
    upper=fmax(0.0,upper);

    double radius=p->radius+t->mean_altitude;
    if(!(radius>DBL_MIN)||!(p->gravitational_parameter>0.0))return NAN;
    double speed=t->true_air_speed;
    double gravity=p->gravitational_parameter/(radius*radius);
    double curvature=t->horizontal_speed*t->horizontal_speed/radius;
    double gamma=t->flight_path_angle*DEG2RAD;
    double bank=norm_signed_deg(t->roll)*DEG2RAD;
    double vertical_lift_factor=cos(bank);
    double base=(gravity-curvature)*cos(gamma);

    double lift0=terminal_projected_lift_accel_at_aoa(t,aero,v,0.0);
    double lift1=terminal_projected_lift_accel_at_aoa(t,aero,v,upper);
    if(!isfinite(lift0)||!isfinite(lift1))return NAN;
    double rate0=(lift0*vertical_lift_factor-base)/speed;
    double rate1=(lift1*vertical_lift_factor-base)/speed;

    double error=(reference_fpa-t->flight_path_angle)*DEG2RAD;
    double response=decision_pitch_capture_time(t,upper);
    double desired_rate;
    if(isfinite(response)&&response>DBL_MIN)
        desired_rate=error/response;
    else
        desired_rate=error>=0.0?fmax(rate0,rate1):fmin(rate0,rate1);

    double minimum_rate=fmin(rate0,rate1),maximum_rate=fmax(rate0,rate1);
    desired_rate=clampd(desired_rate,minimum_rate,maximum_rate);
    if(fabs(rate1-rate0)<=DBL_EPSILON)
        return fabs(desired_rate-rate0)<=sqrt(DBL_EPSILON)?0.0:upper;

    bool increasing=rate1>rate0;
    if((increasing&&desired_rate<=rate0)||(!increasing&&desired_rate>=rate0))
        return 0.0;
    if((increasing&&desired_rate>=rate1)||(!increasing&&desired_rate<=rate1))
        return upper;

    double lo=0.0,hi=upper;
    double resolution=sqrt(DBL_EPSILON)*fmax(1.0,upper);
    unsigned guard=0;
    while(hi-lo>resolution&&guard++<(unsigned)(4*DBL_MANT_DIG)){
        double mid=.5*(lo+hi);
        double lift=terminal_projected_lift_accel_at_aoa(t,aero,v,mid);
        if(!isfinite(lift))return NAN;
        double rate=(lift*vertical_lift_factor-base)/speed;
        if((increasing&&rate<desired_rate)||(!increasing&&rate>desired_rate))
            lo=mid;
        else
            hi=mid;
    }
    return clampd(.5*(lo+hi),0.0,upper);
}

/* The fixed circle's normal gate is intentionally strict.  When a diagnostic
   force-captures a circle that is too tight for the current speed, however,
   the finite vector lead is still a real piece of executable trajectory. Use
   that already-committed distance to remove as much excess kinetic energy as
   the measured polar can reach before p0.  The returned speed is latched at
   capture; recomputing it from the changing AoA would make the lead chase its
   own authority estimate and produce the same late-brake spiral this path was
   designed to avoid. */
       double fixed_hac_lead_target_speed(const Telemetry*t,const PlanetModel*p,
        AerodynamicModel aero,const VehicleProfile*v,double hac_radius){
    if(!t||!p||!v||!(hac_radius>0.0)||!(t->true_air_speed>DBL_MIN))
        return NAN;
    double lateral_speed=guidance_lateral_speed(t);
    if(!(lateral_speed>DBL_MIN))return NAN;
    double lift_aoa=v->maximum_angle_of_attack;
    if(t->mach<1.0&&isfinite(v->terminal_maximum_lift_angle_of_attack)&&
       v->terminal_maximum_lift_angle_of_attack>0.0)
        lift_aoa=fmin(lift_aoa,v->terminal_maximum_lift_angle_of_attack);
    double maximum_lift=terminal_projected_lift_accel_at_aoa(t,aero,v,lift_aoa);
    double local_radius=p->radius+t->mean_altitude;
    if(!isfinite(maximum_lift)||!(maximum_lift>0.0)||!(local_radius>DBL_MIN))
        return NAN;
    double gravity=p->gravitational_parameter/(local_radius*local_radius);
    double normal_limit=fmax(0.0,v->maximum_g_load)*gravity;
    double bank_limit=fmin(v->maximum_bank_angle,dynamic_bank_limit(t,v));
    double effectiveness=isfinite(t->bank_effectiveness)&&t->bank_effectiveness>0.0?
        t->bank_effectiveness:1.0;
    double effective_bank=fmin(fabs(bank_limit*effectiveness),
        nextafter(90.0,0.0))*DEG2RAD;
    double available_lateral=fmin(maximum_lift,normal_limit)*sin(effective_bank);
    if(!(available_lateral>0.0)||!isfinite(available_lateral))return NAN;

    double maximum_lateral_speed=sqrt(available_lateral*hac_radius);
    double lateral_fraction=lateral_speed/t->true_air_speed;
    if(!(lateral_fraction>DBL_MIN))return NAN;
    double target_tas=maximum_lateral_speed/lateral_fraction;
    double floor_speed=fmax(v->minimum_safe_speed,v->final_approach_speed);
    double selected=clampd(target_tas,floor_speed,t->true_air_speed);
    const char*diag=getenv("KSP_LANDER_HAC_DIAGNOSTICS");
    static int target_diag_count=0;
    if(diag&&strcmp(diag,"1")==0&&target_diag_count++<12)
        fprintf(stderr,"HAC_LEAD_TARGET tas=%.6f lateral=%.6f liftAoA=%.6f maxLift=%.9f normalLimit=%.9f bank=%.6f effectiveness=%.6f availableLat=%.9f maxLateralSpeed=%.6f targetTas=%.6f floor=%.6f selected=%.6f\n",
            t->true_air_speed,lateral_speed,lift_aoa,maximum_lift,normal_limit,
            effective_bank*RAD2DEG,effectiveness,available_lateral,
            maximum_lateral_speed,target_tas,floor_speed,selected);
    return selected;
}

       double fixed_hac_lead_drag_aoa(const Telemetry*t,const PlanetModel*p,
        AerodynamicModel aero,const VehicleProfile*v,double target_speed,
        double remaining_distance,double reference_slope_deg){
    if(!t||!p||!v||!(remaining_distance>DBL_MIN)||!(target_speed>0.0)||
       !(t->true_air_speed>target_speed)||!isfinite(reference_slope_deg)||
       fabs(reference_slope_deg)>=90.0)
        return NAN;
    double radius=p->radius+t->mean_altitude;
    if(!(radius>DBL_MIN))return NAN;
    double gravity=p->gravitational_parameter/(radius*radius);
    double speed=t->true_air_speed;
    double required=(speed*speed-target_speed*target_speed)/
        (2.0*remaining_distance);
    required+=gravity*sin(reference_slope_deg*DEG2RAD);
    if(!(required>0.0)||!isfinite(required))return NAN;

    double upper=v->maximum_angle_of_attack;
    double drag0=fixed_hac_projected_drag_accel(t,aero,v,0.0);
    double drag1=fixed_hac_projected_drag_accel(t,aero,v,upper);
    if(!isfinite(drag0)||!isfinite(drag1))return NAN;
    if(drag0>=required)return 0.0;
    if(drag1<=required)return upper;

    double lo=0.0,hi=upper;
    double resolution=sqrt(DBL_EPSILON)*fmax(1.0,upper);
    unsigned guard=0;
    while(hi-lo>resolution&&guard++<(unsigned)(4*DBL_MANT_DIG)){
        double mid=.5*(lo+hi);
        double drag=fixed_hac_projected_drag_accel(t,aero,v,mid);
        if(!isfinite(drag))return NAN;
        if(drag<required)lo=mid;else hi=mid;
    }
    return clampd(.5*(lo+hi),0.0,upper);
}

/* Use the same measured/modelled drag projection as the fixed-HAC path
   integrator to audit, without changing, the upstream AoA arbitration.  This
   is intentionally a diagnostic quantity: it estimates the instantaneous
   specific-energy loss rate for a candidate incidence at the current state;
   it is not an additional admission gate or command owner. */
       double fixed_hac_projected_drag_accel(const Telemetry*t,
        AerodynamicModel aero,const VehicleProfile*v,double aoa){
    if(!t||!v||!(aero.ballistic_coefficient>0.0)||
       !isfinite(t->true_air_speed)||!(t->true_air_speed>0.0)||
       !isfinite(t->atmospheric_density)||t->atmospheric_density<0.0||
       !isfinite(aoa))return NAN;
    double current_incidence=fmax(0.0,hypot(t->angle_of_attack,t->sideslip));
    double current_df=1.0,target_df=1.0;
    aerodynamic_force_factors_mach(t->mach,current_incidence,v,NULL,
        &current_df);
    aerodynamic_force_factors_mach(t->mach,clampd(aoa,0.0,
        v->maximum_angle_of_attack),v,NULL,&target_df);
    double modeled_current=.5*t->atmospheric_density*t->true_air_speed*
        t->true_air_speed/aero.ballistic_coefficient*fmax(0.0,current_df);
    double modeled_target=.5*t->atmospheric_density*t->true_air_speed*
        t->true_air_speed/aero.ballistic_coefficient*fmax(0.0,target_df);
    if(!isfinite(modeled_current)||!isfinite(modeled_target)||
       modeled_current<0.0||modeled_target<0.0)return NAN;
    double measured=live_drag_accel(t,aero,v);
    double anchor=modeled_current>DBL_MIN&&isfinite(measured)&&measured>=0.0?
        measured/modeled_current:1.0;
    return isfinite(anchor)&&anchor>=0.0?modeled_target*anchor:NAN;
}
       double terminal_required_aoa_for_lateral_limit(const Telemetry*t,AerodynamicModel aero,
        const VehicleProfile*v,double lateral_accel,double bank_deg,double maximum_aoa){
    if(!t||!v||!isfinite(lateral_accel)||!isfinite(bank_deg))return NAN;
    maximum_aoa=clampd(maximum_aoa,0.0,v->maximum_angle_of_attack);
    double required_lateral=fabs(lateral_accel);
    if(required_lateral<=DBL_EPSILON)return 0.0;
    double effectiveness=isfinite(t->bank_effectiveness)&&t->bank_effectiveness>0.0?
        t->bank_effectiveness:1.0;
    double effective_bank=fmin(fabs(bank_deg*effectiveness),
        nextafter(90.0,0.0))*DEG2RAD;
    double bank_component=fabs(sin(effective_bank));
    if(bank_component<=DBL_MIN)return maximum_aoa;
    double required_lift=required_lateral/bank_component;
    double lift0=terminal_projected_lift_accel_at_aoa(t,aero,v,0.0);
    double lift1=terminal_projected_lift_accel_at_aoa(t,aero,v,maximum_aoa);
    if(!isfinite(lift0)||!isfinite(lift1))return NAN;
    if(lift0>=required_lift)return 0.0;
    if(lift1<=required_lift)return maximum_aoa;
    double lo=0.0,hi=maximum_aoa;
    double resolution=sqrt(DBL_EPSILON)*fmax(1.0,maximum_aoa);
    unsigned guard=0;
    while(hi-lo>resolution&&guard++<(unsigned)(4*DBL_MANT_DIG)){
        double mid=.5*(lo+hi);
        double lift=terminal_projected_lift_accel_at_aoa(t,aero,v,mid);
        if(!isfinite(lift))return NAN;
        if(lift<required_lift)lo=mid;else hi=mid;
    }
    return clampd(.5*(lo+hi),0.0,maximum_aoa);
}


static double terminal_atmosphere_vertical_resolution(const PlanetModel*p,
        double lower_altitude,double upper_altitude){
    if(!p||p->atmosphere_sample_count<2)return INFINITY;
    lower_altitude=fmax(0.0,lower_altitude);
    upper_altitude=fmax(lower_altitude,upper_altitude);
    double resolution=INFINITY;
    for(size_t i=1;i<p->atmosphere_sample_count;i++){
        double a=p->atmosphere_altitude[i-1],b=p->atmosphere_altitude[i];
        if(!isfinite(a)||!isfinite(b)||!(b>a))return NAN;
        if(b<lower_altitude||a>upper_altitude)continue;
        resolution=fmin(resolution,b-a);
    }
    return resolution;
}

/* Integrate a frozen local quadratic-drag coefficient without Euler's false
   negative kinetic energy. For y=v^2, dy/ds=2*dPhi/ds-2*k*y. The exponential
   solution also integrates drag work; exact rotating-potential differences
   keep propagation and the admission certificate in the same energy frame. */
       bool terminal_quadratic_drag_energy_step(double speed,double drag,
        double air_distance,double potential_drop,double*next_speed,double*work){
    if(!next_speed||!work||!isfinite(speed)||!(speed>0.0)||
       !isfinite(drag)||drag<0.0||!isfinite(air_distance)||air_distance<0.0||
       !isfinite(potential_drop))return false;
    double y=speed*speed;
    double x=2.0*drag*air_distance/y;
    if(!isfinite(x)||x<0.0)return false;
    double attenuation=exp(-x);
    double response=x>DBL_EPSILON?-expm1(-x)/x:1.0;
    double next_y=y*attenuation+2.0*potential_drop*response;
    if(!isfinite(next_y)||!(next_y>0.0))return false;
    double integrated_work=.5*y*(-expm1(-x))+potential_drop*(1.0-response);
    if(!isfinite(integrated_work)||integrated_work<0.0)return false;
    *next_speed=sqrt(next_y);*work=integrated_work;
    return true;
}

       double terminal_projected_drag_work_state_anchored(const GuidanceMachine*g,const Telemetry*t,const PlanetModel*p,
        AerodynamicModel aero,const VehicleProfile*v,const GuidanceSettings*s,double target_aoa,
        double ground_path,double slope_deg,double*end_speed_out,
        double*end_altitude_out,double inherited_anchor){
    if(end_speed_out)*end_speed_out=NAN;
    if(end_altitude_out)*end_altitude_out=NAN;
    if(!g||!t||!p||!v||!s||!isfinite(ground_path)||ground_path<0.0||
       !isfinite(slope_deg)||!isfinite(target_aoa)||
       !isfinite(aero.ballistic_coefficient)||!(aero.ballistic_coefficient>0.0)||
       !isfinite(t->true_air_speed)||!(t->true_air_speed>0.0))
        return NAN;
    if(ground_path==0.0){
        if(end_speed_out)*end_speed_out=t->true_air_speed;
        if(end_altitude_out)*end_altitude_out=t->mean_altitude;
        return 0.0;
    }

    double slope=slope_deg*DEG2RAD;
    double cosine=cos(slope);
    if(!(slope>=0.0&&slope<.5*M_PI)||!(cosine>0.0))return NAN;
    double tangent=tan(slope);
    double path_factor=1.0/cosine;

    double anchor=isnan(inherited_anchor)?
        terminal_drag_projection_anchor(g,t,p,aero,v):inherited_anchor;
    if(!isfinite(anchor)||anchor<0.0)return NAN;

    /*
     * AoA cannot jump to its planned terminal value. Resolve the integration at
     * least as finely as one measured/modelled control-response distance and,
     * when a live atmosphere table exists, one atmosphere interpolation
     * interval in altitude. These are model resolutions, not flight gates.
     */
    double response_time=hac_response_lead_time(t,s,0.0);
    if(!isfinite(response_time)||response_time<0.0)return NAN;
    double response_distance=t->true_air_speed*response_time;
    double end_altitude=fmax(0.0,t->mean_altitude-ground_path*tangent);
    double vertical_resolution=terminal_atmosphere_vertical_resolution(
        p,end_altitude,t->mean_altitude);
    if(isnan(vertical_resolution))return NAN;
    double atmosphere_ground_resolution=
        isfinite(vertical_resolution)&&tangent>DBL_MIN?
            vertical_resolution/tangent:INFINITY;
    /* A response below one control frame cannot be represented by the
       command stream. Do not use its floating-point residual as a global
       quadrature spacing: tiny settled-roll errors otherwise create millions
       of drag-work steps on every MM304 shadow tick. */
    double response_resolution=fmax(
        t->true_air_speed/fmax(1.0,s->guidance_rate),
        sqrt(DBL_EPSILON)*fmax(1.0,ground_path));
    bool response_resolved=response_distance>response_resolution;
    double segment_length=ground_path;
    if(response_resolved)
        segment_length=fmin(segment_length,response_distance);
    if(isfinite(atmosphere_ground_resolution)&&atmosphere_ground_resolution>DBL_MIN)
        segment_length=fmin(segment_length,atmosphere_ground_resolution);
    if(!(segment_length>0.0)||!isfinite(segment_length))segment_length=ground_path;
    size_t steps=(size_t)ceil(ground_path/segment_length);
    if(steps<1)steps=1;

    double initial_aoa=clampd(fabs(t->angle_of_attack),0.0,v->maximum_angle_of_attack);
    double ds=ground_path/(double)steps;
    double speed=t->true_air_speed,altitude=t->mean_altitude;
    double work=0.0;
    for(size_t i=0;i<steps;i++){
        double sample_altitude=fmax(0.0,altitude-.5*ds*tangent);
        double density=planet_atmospheric_density(p,sample_altitude);
        if(!isfinite(density)||density<0.0)return NAN;
        double sound=planet_atmospheric_speed_of_sound(p,sample_altitude);
        double mach=isfinite(sound)&&sound>DBL_MIN?speed/sound:t->mach;

        double distance_mid=((double)i+.5)*ds;
        double response_progress=response_resolved?
            clampd(distance_mid/response_distance,0.0,1.0):1.0;
        response_progress=response_progress*response_progress*(3.0-2.0*response_progress);
        double projected_aoa=initial_aoa+(target_aoa-initial_aoa)*response_progress;
        double df=1.0;
        aerodynamic_force_factors_mach(mach,projected_aoa,v,NULL,&df);
        if(!isfinite(df)||df<0.0)return NAN;

        double drag=.5*density*speed*speed/aero.ballistic_coefficient*df*anchor;
        if(!isfinite(drag)||drag<0.0)return NAN;
        double air_ds=ds*path_factor;
        double next_altitude=fmax(0.0,altitude-ds*tangent);
        double potential_drop=rotating_specific_energy(t->latitude,altitude,0.0,p)-
            rotating_specific_energy(t->latitude,next_altitude,0.0,p);
        double next_speed=NAN,segment_work=NAN;
        if(!terminal_quadratic_drag_energy_step(speed,drag,air_ds,potential_drop,
                &next_speed,&segment_work)){
            if(end_speed_out)*end_speed_out=0.0;
            if(end_altitude_out)*end_altitude_out=altitude;
            return INFINITY;
        }
        work+=segment_work;
        speed=next_speed;
        altitude=next_altitude;
    }
    if(end_speed_out)*end_speed_out=speed;
    if(end_altitude_out)*end_altitude_out=altitude;
    return work;
}

       double terminal_projected_drag_work_state(const GuidanceMachine*g,const Telemetry*t,
        const PlanetModel*p,AerodynamicModel aero,const VehicleProfile*v,
        const GuidanceSettings*s,double target_aoa,double ground_path,double slope_deg,
        double*end_speed_out,double*end_altitude_out){
    return terminal_projected_drag_work_state_anchored(g,t,p,aero,v,s,target_aoa,
        ground_path,slope_deg,end_speed_out,end_altitude_out,NAN);
}

       double terminal_projected_drag_work(const GuidanceMachine*g,const Telemetry*t,
        const PlanetModel*p,AerodynamicModel aero,const VehicleProfile*v,
        const GuidanceSettings*s,double target_aoa,double ground_path,
        double slope_deg){
    return terminal_projected_drag_work_state(g,t,p,aero,v,s,target_aoa,
        ground_path,slope_deg,NULL,NULL);
}

       void terminal_path_slope_bounds(const Telemetry*t,const VehicleProfile*v,
        const GuidanceSettings*s,double*minimum,double*maximum){
    double taem=clampd(s?s->taem_glide_slope:12.0,6.0,30.0);
    double final=clampd(s?s->final_glide_slope:20.0,taem,35.0);
    /* Cool/nominal arrivals should retain the compact terminal corridor;
       only genuine excess speed earns a long shallow energy-management leg.
       This replaces the former universal 18 degree floor with a vehicle- and
       state-scaled envelope rather than simply lowering the constant. */
    double speed_reference=v?
        fmax(v->minimum_safe_speed,v->final_approach_speed):0.0;
    double speed_scale=fmax(speed_reference,DBL_MIN);
    double hot=t?clampd((t->true_air_speed-speed_reference)/speed_scale,
        0.0,1.0):0.0;
    double cool_lo=fmax(taem,final*.90);
    double hot_lo=clampd(taem-4.0,5.0,cool_lo);
    double lo=cool_lo+(hot_lo-cool_lo)*hot;
    /* No slope shallower than the vehicle's live best glide is sustainable:
       pricing a 12-20 deg path for an L/D ~1.5-2.5 orbiter overstates reach
       by 2-3x and the executed path bleeds speed into departure. */
    double ld=t&&isfinite(t->estimated_lift_to_drag)&&t->estimated_lift_to_drag>0.0?
        t->estimated_lift_to_drag:(v?v->estimated_lift_to_drag:NAN);
    /* The HAC is flown at glide alpha, so bound it by the achievable glide
       ratio rather than the (possibly high-drag) current one. */
    if(isfinite(ld)&&ld>0.0)lo=fmax(lo,clampd(atan(1.0/fmax(ld,2.0))*RAD2DEG,5.0,45.0));
    double hi=clampd(fmax(fmax(final+12.0,taem+12.0),lo+8.0),lo+4.0,50.0);
    if(minimum)*minimum=lo;
    if(maximum)*maximum=hi;
}
