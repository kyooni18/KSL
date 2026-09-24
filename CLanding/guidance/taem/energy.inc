/* Specific mechanical energy along an unpowered path obeys
       dE/ds = -D/m.
   That makes drag acceleration, not FPA by itself, the primary TAEM energy
   actuator.  Estimate what the current measured vehicle would produce at a
   different incidence by scaling the measured drag with the calibrated Mach
   polar, then choose the largest AoA that stays inside the remaining drag-work
   budget.  Choosing the largest admissible AoA preserves as much lift/control
   authority as possible without spending the kinetic reserve needed later. */
static double terminal_drag_budget_aoa(const Telemetry*t,AerodynamicModel aero,
        const VehicleProfile*v,double allowed_drag,double preferred_aoa){
    if(!t||!v||!isfinite(allowed_drag)||allowed_drag<=0.0)return NAN;
    double upper=clampd(fmax(preferred_aoa,terminal_hac_energy_aoa(t,v)),
        0.0,v->maximum_angle_of_attack);
    double zero_drag=fixed_hac_projected_drag_accel(t,aero,v,0.0);
    double upper_drag=fixed_hac_projected_drag_accel(t,aero,v,upper);
    if(!isfinite(zero_drag)||!isfinite(upper_drag))return NAN;
    if(zero_drag>allowed_drag)return NAN;
    if(upper_drag<=allowed_drag)return upper;
    double lo=0.0,hi=upper;
    double resolution=sqrt(DBL_EPSILON)*fmax(1.0,upper);
    unsigned guard=0;
    while(hi-lo>resolution&&guard++<(unsigned)(4*DBL_MANT_DIG)){
        double mid=.5*(lo+hi);
        double drag=fixed_hac_projected_drag_accel(t,aero,v,mid);
        if(!isfinite(drag))return NAN;
        if(drag<=allowed_drag)lo=mid;else hi=mid;
    }
    return clampd(lo,0.0,upper);
}

/* A drag-work budget may unload a shallow approach, but it cannot own normal
   force after the commanded dive has been captured.  In the recorded 27 km
   flight the zero-AoA drag itself exceeded the budget.  Treating that empty
   feasible set as a zero-AoA command overrode the FPA feedback all the way from
   -23 to -52 degrees and consumed the remaining maneuver height.

   Release the energy cap continuously across the existing three-degree path
   tracking band.  At/above the shallow edge the energy law retains authority;
   below the steep edge the existing bounded FPA-feedback command owns lift.
   An infeasible budget requests minimum drag only while the vehicle is still
   too shallow, never an indefinitely ballistic descent.  Command slew and
   dynamic-pressure/g-load protections remain downstream in stabilized(). */
/* Close the flight-path angle through normal acceleration, not a fixed
   alpha trim. The old 14 deg + 0.72*FPA-error law converged near -18 deg for
   a -32 deg request as dynamic pressure rose in the live flight: its small
   positive incidence already supplied almost all of weight. Invert the
   current measured lift/polar instead, retaining bounded incidence and the
   downstream load/attitude protections. */
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

static double terminal_preview_recovery_aoa(double preferred_aoa,double budget_aoa,
        double reference_fpa,double actual_fpa){
    if(!isfinite(reference_fpa)||!isfinite(actual_fpa)||!isfinite(budget_aoa))
        return preferred_aoa;
    if(actual_fpa<reference_fpa)
        return preferred_aoa;
    return fmin(preferred_aoa,budget_aoa);
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

static double terminal_required_aoa_for_lateral(const Telemetry*t,AerodynamicModel aero,
        const VehicleProfile*v,double lateral_accel,double bank_deg){
    return terminal_required_aoa_for_lateral_limit(t,aero,v,lateral_accel,
        bank_deg,terminal_hac_energy_aoa(t,v));
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

static double terminal_integrated_energy_path(const GuidanceMachine*g,const Telemetry*t,const PlanetModel*p,
        AerodynamicModel aero,const VehicleProfile*v,const GuidanceSettings*s,double target_aoa,
        double available_specific_energy,double slope_deg,double min_path,double max_path,
        EnergyPathEnvelope*out_envelope){
    if(out_envelope)*out_envelope=(EnergyPathEnvelope){0};
    if(!isfinite(available_specific_energy)||!isfinite(min_path)||!isfinite(max_path)||
       max_path<min_path)return NAN;
    min_path=fmax(0.0,min_path);max_path=fmax(min_path,max_path);

    double min_work=terminal_projected_drag_work(g,t,p,aero,v,s,target_aoa,min_path,slope_deg);
    if(!isfinite(min_work)||min_work<0.0)return NAN;
    EnergyPathEnvelope minimum=decision_energy_path_envelope(
        t,p,available_specific_energy,min_work,min_path);
    if(!minimum.valid)return NAN;
    if(!minimum.feasible){
        if(out_envelope)*out_envelope=minimum;
        return min_path;
    }

    double max_work=terminal_projected_drag_work(g,t,p,aero,v,s,target_aoa,max_path,slope_deg);
    if(isnan(max_work)||max_work<0.0)return NAN;
    if(isfinite(max_work)){
        EnergyPathEnvelope maximum=decision_energy_path_envelope(
            t,p,available_specific_energy,max_work,max_path);
        if(!maximum.valid)return NAN;
        if(maximum.feasible){
            if(out_envelope)*out_envelope=maximum;
            return max_path;
        }
    }
    /*
     * +INFINITY is a meaningful result from the forward integrator: the vehicle
     * would exhaust kinetic energy before completing this path. Treat it as a
     * physically infeasible upper bracket, not as a broken model.
     */

    /*
     * Solve the longest path whose modeled drag work plus measured/model
     * uncertainty fits the available unpowered specific-energy budget.
     * Convergence is numerical: stop at sqrt(machine epsilon) relative bracket
     * width rather than using a tuned flight-behavior iteration count.
     */
    double lo=min_path,hi=max_path;
    double slope_rad=slope_deg*DEG2RAD;
    double path_resolution=sqrt(DBL_EPSILON)*fmax(1.0,max_path);
    if(isfinite(slope_rad)&&slope_rad>0.0&&slope_rad<.5*M_PI){
        double vertical_resolution=terminal_atmosphere_vertical_resolution(
            p,fmax(0.0,t->mean_altitude-max_path*tan(slope_rad)),
            t->mean_altitude);
        if(isfinite(vertical_resolution)&&vertical_resolution>0.0)
            path_resolution=fmax(path_resolution,
                vertical_resolution/tan(slope_rad));
    }
    EnergyPathEnvelope best=minimum;
    for(int i=0;i<DBL_MANT_DIG;i++){
        double scale=fmax(1.0,fmax(fabs(lo),fabs(hi)));
        if(hi-lo<=fmax(path_resolution,sqrt(DBL_EPSILON)*scale))break;
        double mid=.5*(lo+hi);
        double work=terminal_projected_drag_work(g,t,p,aero,v,s,target_aoa,mid,slope_deg);
        if(isnan(work)||work<0.0)return NAN;
        if(isinf(work)){
            hi=mid;
            continue;
        }
        EnergyPathEnvelope probe=decision_energy_path_envelope(
            t,p,available_specific_energy,work,mid);
        if(!probe.valid)return NAN;
        if(probe.feasible){lo=mid;best=probe;}else hi=mid;
    }
    if(out_envelope)*out_envelope=best;
    return lo;
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

/* Until a lateral route is committed, do not spend inbound distance on an
   imagined energy-management loop. Close height against the real preflare
   gate. A qualified, committed HAC has its own path law and never calls this
   preview helper. Drag work is budgeted over air-path metres, not horizontal
   metres. This preserves the same bounded descent envelope. */
static double terminal_uncommitted_alignment_heading(const Telemetry*t,
        const LandingConfiguration*cfg){
    /* The terminal alignment station is a fixed point, not an always-positive
       look-ahead distance. The old max(4000, staging_along-along) moved the
       target east with the aircraft and commanded 35 degrees even after the
       vehicle passed east of the runway. Preserve the signed along delta. */
    double target_along=-fmax(1500.0,cfg->guidance.final_approach_distance);
    double along_delta=target_along-t->runway_along_track;
    return norm_deg(cfg->site.runway_heading+
        atan2(-t->runway_cross_track,along_delta)*RAD2DEG);
}

static double terminal_uncommitted_gate_geometry(const GuidanceMachine*g,
        const Telemetry*t,const LandingConfiguration*cfg,double*fpa){
    double gate_height=fmax(0.0,g->terminal_test_preflare_altitude);
    double gate_ground=gate_height/fmax(tan(clampd(cfg->guidance.final_glide_slope,
        5.0,35.0)*DEG2RAD),1e-3);
    double ground=hypot(-t->runway_along_track-gate_ground,
        t->runway_cross_track);
    double height=fmax(0.0,t->mean_altitude-cfg->site.altitude-gate_height);
    double minimum=0.0,maximum=0.0;
    terminal_path_slope_bounds(t,&cfg->vehicle,&cfg->guidance,&minimum,&maximum);
    if(fpa)*fpa=-clampd(atan2(height,fmax(1000.0,ground))*RAD2DEG,minimum,maximum);
    return fmax(1000.0,hypot(ground,height));
}

static double terminal_test_energy_selected_slope(const GuidanceMachine*g,const Telemetry*t,
        const PlanetModel*p,AerodynamicModel aero,const LandingConfiguration*cfg,
        double*preferred_total_path,EnergyPathEnvelope*out_envelope){
    const VehicleProfile*v=&cfg->vehicle;
    if(out_envelope)*out_envelope=(EnergyPathEnvelope){0};
    double gate_altitude=cfg->site.altitude+fmax(0,g->terminal_test_preflare_altitude);
    double target_speed=fmax(g->terminal_test_preflare_target_speed,g->terminal_test_preflare_min_speed);
    double available_energy=entry_remaining_specific_energy(
        t->latitude,t->mean_altitude,t->true_air_speed,
        cfg->site.latitude,gate_altitude,target_speed,p);
    if(!isfinite(available_energy))return NAN;

    /*
     * The path budget is solved in specific-energy units. Uncertainty comes
     * from measured trajectory residuals and certified physics uncertainty in
     * decision_energy_path_envelope(); calibration confidence is not converted
     * into an arbitrary percentage reserve here.
     */
    double gate_height=fmax(0,t->mean_altitude-gate_altitude);
    double minimum_slope=0,maximum_slope=0;
    terminal_path_slope_bounds(t,v,&cfg->guidance,&minimum_slope,&maximum_slope);
    double min_total=gate_height/fmax(tan(maximum_slope*DEG2RAD),DBL_MIN);
    double max_total=gate_height/fmax(tan(minimum_slope*DEG2RAD),DBL_MIN);
    double baseline_aoa=clampd(fmax(3.0,t->angle_of_attack),3.0,terminal_hac_energy_aoa(t,v));
    double nominal_slope=clampd(
        atan2(gate_height,fmax(.5*(min_total+max_total),DBL_MIN))*RAD2DEG,
        minimum_slope,maximum_slope);

    EnergyPathEnvelope envelope={0};
    double selected_total=terminal_integrated_energy_path(
        g,t,p,aero,v,&cfg->guidance,baseline_aoa,available_energy,
        nominal_slope,min_total,max_total,&envelope);
    if(!isfinite(selected_total)||!envelope.valid)return NAN;

    double slope=atan2(gate_height,fmax(selected_total,DBL_MIN))*RAD2DEG;
    slope=clampd(slope,minimum_slope,maximum_slope);
    if(preferred_total_path)*preferred_total_path=selected_total;
    if(out_envelope)*out_envelope=envelope;
    return slope;
}

static double terminal_test_projected_turn_radius(const Telemetry*t,const PlanetModel*p,
        AerodynamicModel aero,const VehicleProfile*v,double bank_deg){
    double current_lf=0,target_lf=0;
    double current_incidence=fmax(1.0,fabs(t->angle_of_attack));
    double target_incidence=terminal_hac_energy_aoa(t,v);
    aerodynamic_force_factors_mach(t->mach,current_incidence,v,&current_lf,NULL);
    aerodynamic_force_factors_mach(t->mach,target_incidence,v,&target_lf,NULL);
    double measured=t->mass>1&&isfinite(t->lift_force)&&t->lift_force>0?t->lift_force/t->mass:0;
    double projected=measured>0?measured*fabs(target_lf)/fmax(.05,fabs(current_lf)):0;
    double modeled=fmax(0,t->dynamic_pressure)/fmax(20,aero.ballistic_coefficient)*
        fmax(0,aero.lift_to_drag)*fabs(target_lf);
    projected=fmax(modeled,clampd(projected,0,planet_surface_gravity(p)*2.0));
    double eff=clampd(t->bank_effectiveness,.35,1.8);
    double lateral=projected*fabs(sin(clampd(fabs(bank_deg)*eff,0,89)*DEG2RAD));
    return lateral>.005?t->true_air_speed*t->true_air_speed/lateral:INFINITY;
}

static double hac_oriented_progress_rate(const Telemetry*t,const LandingSite*site,
        const GuidanceSettings*s,double planet_radius,double side,double hac_radius,double course){
    GeoPoint current={t->latitude,t->longitude,t->mean_altitude};
    GeoPoint threshold={site->latitude,site->longitude,site->altitude};
    double e=0,n=0;local_offsets(threshold,current,planet_radius,&e,&n);
    double h=site->runway_heading*DEG2RAD,ae=sin(h),an=cos(h),re=cos(h),rn=-sin(h);
    double fe=-ae*s->final_approach_distance,fn=-an*s->final_approach_distance;
    double ce=fe+re*side*hac_radius,cn=fn+rn*side*hac_radius;
    double x=e-ce,y=n-cn,r2=x*x+y*y;
    if(r2<1)return 0;
    double cr=course*DEG2RAD,ve=t->horizontal_speed*sin(cr),vn=t->horizontal_speed*cos(cr);
    double angular=(x*vn-y*ve)/r2;
    double oriented=-side*angular;
    double geometric_limit=t->horizontal_speed/fmax(sqrt(r2),1)*1.5;
    return clampd(oriented,0,fmax(0,geometric_limit));
}

/* Publication invariant shared by every HAC selection outcome. A distant
   circle is legal only with a continuous, qualified join and a real arc. */
static bool hac_handoff_geometry_valid(const HACTransitionPlan*join,double e,double n,
        double course,const LandingSite*site,const GuidanceSettings*s,double radius,double side){
    (void)course;
    if(!join||!join->valid||!isfinite(radius)||radius<1000.0||
       fabs(side)!=1.0||!isfinite(join->length)||join->length<200.0)return false;
    double start_error=hypot(join->lead_start.e-e,join->lead_start.n-n);
    double lead_chord=hypot(join->p0.e-e,join->p0.n-n);
    if(!isfinite(start_error)||start_error>=.01||!isfinite(lead_chord))return false;
    if(join->lead_curve){
        if(!(join->lead_length>=lead_chord-0.05)||
           !(join->lead_length<=lead_chord*2.5+0.05)||
           !isfinite(join->lead_start_course)||
           !isfinite(join->lead_end_course))return false;
    }else{
        double lead_error=fabs(join->lead_length-lead_chord);
        if(!isfinite(lead_error)||lead_error>=.05)return false;
    }
    if(join->heading_cone){
        if(!(join->cone_arc_length>0.0)||!isfinite(join->cone_start_angle)||
           !isfinite(join->cone_end_angle)||!isfinite(join->cone_center.e)||
           !isfinite(join->cone_center.n))return false;
        double r0=hypot(join->p0.e-join->cone_center.e,join->p0.n-join->cone_center.n);
        double r3=hypot(join->p3.e-join->cone_center.e,join->p3.n-join->cone_center.n);
        double p0e=join->cone_center.e+radius*cos(join->cone_start_angle);
        double p0n=join->cone_center.n+radius*sin(join->cone_start_angle);
        double p3e=join->cone_center.e+radius*cos(join->cone_end_angle);
        double p3n=join->cone_center.n+radius*sin(join->cone_end_angle);
        double directed=fmod(-side*(join->cone_end_angle-join->cone_start_angle),2.0*LANDER_PI);
        if(directed<0.0)directed+=2.0*LANDER_PI;
        double end_heading=norm_deg(atan2(side*sin(join->cone_end_angle),
            -side*cos(join->cone_end_angle))*RAD2DEG);
        return fabs(r0-radius)<.05&&fabs(r3-radius)<.05&&
            hypot(join->p0.e-p0e,join->p0.n-p0n)<.05&&
            hypot(join->p3.e-p3e,join->p3.n-p3n)<.05&&
            fabs(join->cone_arc_length-radius*directed)<fmax(.05,radius*1e-8)&&
            fabs(norm_signed_deg(end_heading-site->runway_heading))<1e-6&&
            fabs(norm_signed_deg((join->end_angle-join->cone_end_angle)*RAD2DEG))<1e-6&&
            fabs(join->arc_remaining)<.05;
    }
    if(!isfinite(join->arc_remaining)||join->arc_remaining<700.0)return false;
    double h=site->runway_heading*DEG2RAD;
    double fe=-sin(h)*s->final_approach_distance,fn=-cos(h)*s->final_approach_distance;
    double ce=fe+cos(h)*side*radius,cn=fn-sin(h)*side*radius;
    double end=atan2(fn-cn,fe-ce)+side*join->arc_remaining/radius;
    double end_error=hypot(join->p3.e-(ce+radius*cos(end)),join->p3.n-(cn+radius*sin(end)));
    double angle_error=fabs(norm_signed_deg((join->end_angle-end)*RAD2DEG));
    return isfinite(end_error)&&end_error<.01&&isfinite(angle_error)&&angle_error<1e-6;
}

/* Acquisition is measured at the live aircraft, independently of downstream
   path fit. Positive radial closure means toward the circle; in this local
   frame +1 is south and -1 is north for an eastbound runway. */
static void terminal_candidate_execution_cost(TerminalCandidate*c,
        const GuidanceMachine*g,const Telemetry*t,double course,
        const PlanetModel*p,AerodynamicModel aero,const LandingConfiguration*cfg){
    if(!c)return;
    c->execution_evaluated=true;
    c->execution_distance=c->execution_time=c->execution_course_error=INFINITY;
    c->execution_margin=-INFINITY;
    c->execution_horizon=0.0;
    if(!t||!c->valid||!c->join.valid||!(t->horizontal_speed>0.0))return;
    GeoPoint origin={cfg->site.latitude,cfg->site.longitude,cfg->site.altitude};
    GeoPoint current={t->latitude,t->longitude,t->mean_altitude};
    double e,n;local_offsets(origin,current,p->radius,&e,&n);
    double de=c->join.lead_start.e-e,dn=c->join.lead_start.n-n;
    double distance=hypot(de,dn);
    HACPoint2 d={c->join.p0.e-c->join.lead_start.e,
        c->join.p0.n-c->join.lead_start.n};
    if(hypot(d.e,d.n)<=sqrt(DBL_EPSILON))
        d=(HACPoint2){c->join.p1.e-c->join.p0.e,c->join.p1.n-c->join.p0.n};
    if(hypot(d.e,d.n)<=sqrt(DBL_EPSILON))return;
    double tangent=atan2(d.e,d.n)*RAD2DEG;
    double error=fabs(norm_signed_deg(tangent-course));
    double acquisition_error=error;
    if(distance>t->horizontal_speed*fmax(0.0,c->response))
        acquisition_error=fmax(error,fabs(norm_signed_deg(atan2(de,dn)*RAD2DEG-course)));
    double rate=hac_course_rate_cap_for_speed(t->true_air_speed);
    double bank=fmin(fabs(cfg->vehicle.maximum_bank_angle),dynamic_bank_limit(t,&cfg->vehicle));
    double lift=live_lift_accel(t,aero,&cfg->vehicle);
    double effectiveness=t->bank_effectiveness>0.0?t->bank_effectiveness:1.0;
    rate=fmin(rate,lift*fabs(sin(fmin(bank*effectiveness,nextafter(90.0,0.0))*DEG2RAD))/t->horizontal_speed*RAD2DEG);
    double radial_turn=0.0;
    c->execution_radial_closure=0.0;
    if(c->kind==TERMINAL_PATH_HAC){
        double h=cfg->site.runway_heading*DEG2RAD;
        double ce=c->join.heading_cone?c->join.cone_center.e:
            -sin(h)*c->final_distance+cos(h)*c->side*c->radius;
        double cn=c->join.heading_cone?c->join.cone_center.n:
            -cos(h)*c->final_distance-sin(h)*c->side*c->radius;
        double r=hypot(ce-e,cn-n);
        if(r>sqrt(DBL_EPSILON)){
            c->execution_radial_closure=t->horizontal_speed*
                ((ce-e)*sin(course*DEG2RAD)+(cn-n)*cos(course*DEG2RAD))/r;
            double inward=fabs(norm_signed_deg(atan2(ce-e,cn-n)*RAD2DEG-course));
            radial_turn=fmax(0.0,inward-90.0);
        }
    }
    double turn=fmax(acquisition_error,radial_turn)/fmax(rate,DBL_MIN);
    double time=distance/t->horizontal_speed+turn+fmax(0.0,c->response);
    double gate=cfg->site.altitude+fmax(0.0,g->terminal_test_preflare_altitude);
    double height=fmax(0.0,t->mean_altitude-gate);
    double sink=fmax(fmax(0.0,-t->vertical_speed),t->horizontal_speed*fabs(tan(c->slope*DEG2RAD)));
    double drag=terminal_expected_speed_loss_accel(g,t,p,aero,&cfg->vehicle);
    double minimum_speed=fmax(g->terminal_test_preflare_min_speed,cfg->vehicle.minimum_safe_speed);
    double decel=fmax(0.0,drag-planet_surface_gravity(p)*fmax(0.0,-sin(t->flight_path_angle*DEG2RAD)));
    double horizon=sink>0.0?height/sink:INFINITY;
    if(decel>0.0)horizon=fmin(horizon,fmax(0.0,t->true_air_speed-minimum_speed)/decel);
    double reserve=planet_surface_gravity(p)*height+
        (t->true_air_speed*t->true_air_speed-minimum_speed*minimum_speed)/2.0;
    if(drag>0.0)horizon=fmin(horizon,fmax(0.0,reserve)/(drag*t->true_air_speed));
    c->execution_distance=distance;c->execution_course_error=error;
    c->execution_time=time;c->execution_horizon=horizon;
    c->execution_margin=horizon-time;
    if(!(c->execution_margin>=0.0)){c->degraded=true;c->energy_degraded=true;}
}
