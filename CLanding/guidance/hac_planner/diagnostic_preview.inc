/* Diagnostic two-horizon preview.  This deliberately does not create a live
   HAC tuple.  It advances a copy of the measured state along the current
   course with the existing terminal energy incidence and measured/modelled
   drag, then asks the ordinary dynamic planner whether a future measured
   state could produce a real candidate.  The preview is evidence for a
   future-staging implementation; it is never used as admission, ownership,
   or executable lead geometry. */
static bool terminal_variant_b_project_staging_state(const Telemetry*initial,
        const PlanetModel*p,AerodynamicModel aero,const VehicleProfile*v,
        const GuidanceSettings*s,double course,double horizon,
        double initial_e,double initial_n,Telemetry*out,double*e_out,
        double*n_out,double*distance_out){
    if(out)memset(out,0,sizeof(*out));
    if(e_out)*e_out=initial_e;
    if(n_out)*n_out=initial_n;
    if(distance_out)*distance_out=0.0;
    if(!initial||!p||!v||!s||!isfinite(horizon)||horizon<=0.0||
       !isfinite(initial->true_air_speed)||initial->true_air_speed<=1.0||
       !isfinite(initial->mean_altitude)||initial->mean_altitude<=1000.0)
        return false;
    Telemetry sim=*initial;
    double speed=initial->true_air_speed;
    double altitude=initial->mean_altitude;
    double aoa=clampd(fabs(initial->angle_of_attack),0.0,
        v->maximum_angle_of_attack);
    double aoa_rate=controlled_aoa_rate(initial);
    GuidanceAttitudeLimits limits=stabilized_attitude_limits(initial,s,
        PHASE_TAEM,true);
    double slope=clampd(fabs(isfinite(initial->flight_path_angle)?
        initial->flight_path_angle:s->taem_glide_slope),2.0,22.0);
    double e=initial_e,n=initial_n,distance=0.0;
    double dt=0.5;
    size_t steps=(size_t)ceil(horizon/dt);
    for(size_t i=0;i<steps;i++){
        double step=fmin(dt,horizon-(double)i*dt);
        if(!(step>0.0))break;
        sim.mean_altitude=altitude;
        sim.true_air_speed=speed;
        sim.horizontal_speed=speed*cos(slope*DEG2RAD);
        sim.surface_speed=sim.horizontal_speed;
        sim.flight_path_angle=-slope;
        sim.angle_of_attack=aoa;
        sim.sideslip=0.0;
        sim.ground_track_heading=course;
        sim.heading=course;
        sim.atmospheric_density=planet_atmospheric_density(p,altitude);
        sim.speed_of_sound=planet_atmospheric_speed_of_sound(p,altitude);
        sim.mach=sim.speed_of_sound>DBL_MIN?
            speed/sim.speed_of_sound:NAN;
        sim.dynamic_pressure=.5*sim.atmospheric_density*speed*speed;
        if(!isfinite(sim.atmospheric_density)||sim.atmospheric_density<0.0||
           !isfinite(sim.dynamic_pressure))return false;

        double drag=fixed_hac_projected_drag_accel(&sim,aero,v,aoa);
        if(!isfinite(drag)||drag<0.0)
            drag=terminal_expected_energy_loss_accel(NULL,&sim,aero,v);
        if(!isfinite(drag)||drag<0.0)return false;
        double ds=speed*step;
        double air_ds=ds/fmax(cos(slope*DEG2RAD),DBL_MIN);
        double next_altitude=fmax(0.0,altitude-ds*tan(slope*DEG2RAD));
        double gravity=p->gravitational_parameter/
            fmax(1.0,p->radius+.5*(altitude+next_altitude));
        double next_v2=speed*speed+2.0*gravity*(altitude-next_altitude)-
            2.0*drag*air_ds;
        if(!isfinite(next_v2)||next_v2<=0.0)return false;
        e+=ds*sin(course*DEG2RAD);
        n+=ds*cos(course*DEG2RAD);
        distance+=ds;
        speed=sqrt(next_v2);
        altitude=next_altitude;

        double target=terminal_hac_energy_aoa(&sim,v);
        double requested_rate=clampd((target-aoa)/step,
            -limits.rate_deg_s,limits.rate_deg_s);
        double rate_delta=clampd(requested_rate-aoa_rate,
            -limits.accel_deg_s2*step,limits.accel_deg_s2*step);
        aoa_rate=clampd(aoa_rate+rate_delta,-limits.rate_deg_s,
            limits.rate_deg_s);
        aoa=clampd(aoa+aoa_rate*step,0.0,v->maximum_angle_of_attack);
        if(altitude<=1000.0||speed<=1.0)break;
    }
    sim.ut=initial->ut+horizon;
    sim.mean_altitude=altitude;
    sim.true_air_speed=speed;
    sim.horizontal_speed=speed*cos(slope*DEG2RAD);
    sim.surface_speed=sim.horizontal_speed;
    sim.flight_path_angle=-slope;
    sim.angle_of_attack=aoa;
    sim.angle_of_attack_rate=aoa_rate;
    sim.has_angle_of_attack_rate=true;
    sim.ground_track_heading=course;
    sim.heading=course;
    sim.speed_of_sound=planet_atmospheric_speed_of_sound(p,altitude);
    sim.mach=sim.speed_of_sound>DBL_MIN?speed/sim.speed_of_sound:NAN;
    sim.atmospheric_density=planet_atmospheric_density(p,altitude);
    sim.dynamic_pressure=.5*sim.atmospheric_density*speed*speed;
    if(out)*out=sim;
    if(e_out)*e_out=e;
    if(n_out)*n_out=n;
    if(distance_out)*distance_out=distance;
    return isfinite(speed)&&isfinite(altitude)&&isfinite(e)&&isfinite(n);
}

static bool terminal_variant_b_preview_future_hac(GuidanceMachine*g,
        const Telemetry*t,double course,const PlanetModel*p,AerodynamicModel aero,
        const LandingConfiguration*cfg,double current_e,double current_n,
        double capture_course){
    const char*diag=getenv("KSP_LANDER_HAC_DIAGNOSTICS");
    const char*preview=getenv("KSP_LANDER_HAC_FUTURE_PREVIEW");
    if(!g||!t||!p||!cfg||!diag||strcmp(diag,"1")!=0||!preview||
       strcmp(preview,"1")!=0||g->diagnostic_shadow)return false;
    const VehicleProfile*v=&cfg->vehicle;
    const GuidanceSettings*s=&cfg->guidance;
    static const double horizons[]={10.0,20.0,30.0,40.0};
    for(size_t i=0;i<sizeof(horizons)/sizeof(horizons[0]);i++){
        Telemetry forecast={0};
        double e=current_e,n=current_n,stage_distance=0.0;
        if(!terminal_variant_b_project_staging_state(t,p,aero,
                v,s,course,horizons[i],current_e,current_n,&forecast,&e,&n,
                &stage_distance))continue;
        HACTransitionPlan future={0};
        GuidanceMachine future_profile={0};
        double radius=NAN,authority_min=INFINITY,energy_radius=0.0,
            entry_speed=NAN;
        bool provisional=false;
        bool dynamic=terminal_variant_b_find_dynamic_plan(g,&forecast,course,p,
            aero,cfg,e,n,capture_course,&future,&radius,&authority_min,
            &energy_radius,&entry_speed,&provisional,&future_profile);
        if(!dynamic&&!provisional)continue;
        g->terminal_test_upstream_future_ref_valid=true;
        g->terminal_test_upstream_future_ref_ut=forecast.ut;
        g->terminal_test_upstream_future_ref_stage_time=horizons[i];
        g->terminal_test_upstream_future_ref_stage_distance=stage_distance;
        g->terminal_test_upstream_future_ref_radius=radius;
        g->terminal_test_upstream_future_ref_p0_e=future.p0.e;
        g->terminal_test_upstream_future_ref_p0_n=future.p0.n;
        g->terminal_test_upstream_future_ref_speed=forecast.true_air_speed;
        g->terminal_test_upstream_future_ref_altitude=forecast.mean_altitude;
        g->terminal_test_upstream_future_ref_margin=dynamic?
            energy_radius-radius:NAN;
        fprintf(stderr,"HAC_FUTURE_PREVIEW ut=%.3f horizon=%.1f stageDistance=%.1f forecastAlt=%.1f forecastSpeed=%.1f radius=%.1f p0=(%.1f,%.1f) dynamic=%d provisional=%d energyRadius=%.1f authorityMinR=%.1f margin=%.1f accounting=stage-separate\n",
            t->ut,horizons[i],stage_distance,forecast.mean_altitude,
            forecast.true_air_speed,radius,future.p0.e,future.p0.n,dynamic?1:0,
            provisional?1:0,energy_radius,authority_min,
            dynamic?energy_radius-radius:NAN);
        return true;
    }
    g->terminal_test_upstream_future_ref_valid=false;
    return false;
}

