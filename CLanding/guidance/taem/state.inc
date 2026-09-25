       void terminal_glide_initialize(GuidanceMachine*g,const VehicleProfile*v,const GuidanceSettings*s){
    g->terminal_glide_mode=true;
    g->terminal_path_kind=TERMINAL_PATH_NONE;
    g->terminal_energy_sample_valid=false;
    g->terminal_previous_specific_energy=0;
    g->terminal_previous_speed=0;
    g->terminal_energy_sample_ut=0;
    g->terminal_energy_loss_accel_ema=0;
    g->terminal_speed_loss_accel_ema=0;
    g->terminal_test_preflare_altitude=clampd(fmax(500.0,s->flare_altitude*8.0),450.0,750.0);
    g->terminal_test_preflare_target_speed=fmax(v->final_approach_speed*1.12,v->minimum_safe_speed*1.40);
    g->terminal_test_preflare_min_speed=fmax(v->final_approach_speed*1.03,v->minimum_safe_speed*1.25);
    g->terminal_test_glide_slope=s->taem_glide_slope;
    g->terminal_test_final_approach_distance=s->final_approach_distance;
}

       void terminal_energy_observe(GuidanceMachine*g,const Telemetry*t,const PlanetModel*p){
    if(!g||!t||!p||!g->terminal_glide_mode)return;
    double specific=rotating_specific_energy(t->latitude,t->mean_altitude,t->true_air_speed,p);
    if(!isfinite(specific)||!isfinite(t->ut)||!isfinite(t->true_air_speed))return;
    if(g->terminal_energy_sample_valid){
        double dt=t->ut-g->terminal_energy_sample_ut;
        if(dt>=.08&&dt<=2.5){
            double mean_speed=fmax(30.0,.5*(g->terminal_previous_speed+t->true_air_speed));
            double energy_equiv=(g->terminal_previous_specific_energy-specific)/dt/mean_speed;
            double speed_loss=(g->terminal_previous_speed-t->true_air_speed)/dt;
            if(isfinite(energy_equiv)&&energy_equiv>=0&&energy_equiv<60){
                double a=clampd(dt/(2.2+dt),.03,.45);
                g->terminal_energy_loss_accel_ema=g->terminal_energy_loss_accel_ema>0?
                    g->terminal_energy_loss_accel_ema+(energy_equiv-g->terminal_energy_loss_accel_ema)*a:energy_equiv;
            }
            if(isfinite(speed_loss)&&speed_loss>=0&&speed_loss<60){
                double a=clampd(dt/(1.8+dt),.03,.50);
                g->terminal_speed_loss_accel_ema=g->terminal_speed_loss_accel_ema>0?
                    g->terminal_speed_loss_accel_ema+(speed_loss-g->terminal_speed_loss_accel_ema)*a:speed_loss;
            }
        }
    }
    g->terminal_previous_specific_energy=specific;
    g->terminal_previous_speed=t->true_air_speed;
    g->terminal_energy_sample_ut=t->ut;
    g->terminal_energy_sample_valid=true;
}

       double terminal_expected_energy_loss_accel(const GuidanceMachine*g,const Telemetry*t,
        AerodynamicModel aero,const VehicleProfile*v){
    /*
     * Specific mechanical energy lost per metre of air path is D/m. Prefer the
     * larger of the instantaneous measured/modelled drag acceleration and the
     * observed mechanical-energy-loss EMA when both exist. No confidence
     * percentage or safety multiplier is translated into extra drag here;
     * uncertainty is carried separately by EnergyPathEnvelope.
     */
    double live=live_drag_accel(t,aero,v);
    if(!isfinite(live)||live<0.0)return NAN;
    double incidence=hypot(t->angle_of_attack,t->sideslip),df=1.0;
    aerodynamic_force_factors_mach(t->mach,incidence,v,NULL,&df);
    double modeled=fmax(0.0,t->dynamic_pressure)/
        fmax(20.0,aero.ballistic_coefficient)*fmax(0.0,df);
    if(!isfinite(modeled)||modeled<0.0)return NAN;
    /* live_drag_accel intentionally prefers a measured force when available,
       but the admission contract above promises the larger measured/modelled
       instantaneous loss.  Keep that promise here so a stale or low measured
       force cannot make the lead reachability ledger disagree with the energy
       admission ledger on the same telemetry frame. */
    live=fmax(live,modeled);
    if(g&&isfinite(g->terminal_energy_loss_accel_ema)&&
       g->terminal_energy_loss_accel_ema>=0.0)
        return fmax(live,g->terminal_energy_loss_accel_ema);
    return live;
}


/* A dimensionless polar correction belongs to the measured source state.
   Carry this coefficient, not a live absolute drag acceleration, to p0. */
       double terminal_drag_projection_anchor(const GuidanceMachine*g,
        const Telemetry*t,const PlanetModel*p,AerodynamicModel aero,
        const VehicleProfile*v){
    if(!g||!t||!p||!v||!isfinite(aero.ballistic_coefficient)||
       !(aero.ballistic_coefficient>0.0))return NAN;
    double df=1.0;
    aerodynamic_force_factors_mach(t->mach,hypot(t->angle_of_attack,t->sideslip),v,NULL,&df);
    double density=planet_atmospheric_density(p,t->mean_altitude);
    double modeled=.5*density*t->true_air_speed*t->true_air_speed/
        aero.ballistic_coefficient*df;
    double observed=terminal_expected_energy_loss_accel(g,t,aero,v);
    if(!isfinite(modeled)||modeled<0.0||!isfinite(observed)||observed<0.0)return NAN;
    double anchor=modeled>DBL_MIN?observed/modeled:1.0;
    const char*diag=getenv("KSP_LANDER_HAC_DIAGNOSTICS");
    static int anchor_diag_count=0;
    if(diag&&strcmp(diag,"1")==0&&anchor_diag_count++<12)
        fprintf(stderr,"HAC_DRAG_ANCHOR speed=%.6f aoa=%.6f mach=%.6f density=%.9f q=%.6f polarDf=%.9f dragForce=%.6f liftForce=%.6f mass=%.6f forceVectors=%d physicsValid=%d modeled=%.9f observed=%.9f anchor=%.9f ema=%.9f\n",
            t->true_air_speed,t->angle_of_attack,t->mach,density,
            t->dynamic_pressure,df,t->drag_force,t->lift_force,t->mass,
            t->has_force_vectors?1:0,t->physics_sample_valid?1:0,
            modeled,observed,anchor,
            g->terminal_energy_loss_accel_ema);
    return anchor;
}
