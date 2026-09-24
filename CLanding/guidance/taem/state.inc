       double terminal_hac_radius_live(const Telemetry*t,const PlanetModel*p,AerodynamicModel aero,const VehicleProfile*v,const GuidanceSettings*s){
    (void)p;(void)s;
    if(!t||!v)return INFINITY;
    /*
     * Radius is a consequence of stock-KSP lateral force, not a mission magic
     * number.  live_turn_radius() uses the current calibrated lift model and
     * the vehicle/dynamic bank envelope; no extra bank cap, safety multiplier,
     * or absolute radius floor is inserted here.
     */
    double usable_bank=dynamic_bank_limit(t,v);
    double radius=live_turn_radius(t,aero,v,usable_bank);
    return isfinite(radius)&&radius>0.0?radius:INFINITY;
}
       void terminal_glide_initialize(GuidanceMachine*g,const VehicleProfile*v,const GuidanceSettings*s){
    g->terminal_glide_mode=true;
    g->terminal_path_kind=TERMINAL_PATH_NONE;
    g->hac_transition_lead_curve=false;
    g->hac_transition_lead_acquisition=false;
    g->hac_transition_handoff_lateral_valid=false;
    g->hac_transition_handoff_lateral_acceleration=0.0;
    g->hac_transition_handoff_blend=0.0;
    g->hac_transition_handoff_aoa_valid=false;
    g->hac_transition_handoff_aoa=0.0;
    g->fixed_hac_lead_speed_target_valid=false;
    g->fixed_hac_lead_target_speed=0.0;
    g->fixed_hac_reference_valid=false;
    g->fixed_hac_reference_start_altitude=0.0;
    g->fixed_hac_reference_final_altitude=0.0;
    g->fixed_hac_reference_slope_deg=0.0;
    g->fixed_hac_reference_initial_slope_deg=0.0;
    g->fixed_hac_reference_settled_slope_deg=0.0;
    g->fixed_hac_reference_final_slope_deg=0.0;
    g->fixed_hac_reference_transition_distance=0.0;
    g->fixed_hac_reference_exit_transition_distance=0.0;
    g->hac_plan_degraded=false;
    g->hac_plan_geometry_degraded=false;
    g->hac_plan_energy_degraded=false;
    g->hac_plan_violation_score=0;
    g->hac_commit_blend=0;
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

/* Diagnostic-only live closure for the committed fixed-HAC lead.  This is
 * deliberately kept outside all admission and command decisions: it measures
 * what the simulator actually flew so the forecast ledger can be audited. */
       void hac_energy_audit_observe(GuidanceMachine*g,const Telemetry*t,
        const PlanetModel*p,AerodynamicModel aero,const VehicleProfile*v,
        double dt){
    if(!g||!t||!p||!v||!g->hac_energy_audit_active||
       g->hac_energy_audit_logged)return;
    double step=dt;
    if(isfinite(g->hac_energy_audit_last_ut)&&isfinite(t->ut))
        step=t->ut-g->hac_energy_audit_last_ut;
    if(!isfinite(step)||step<0.0||step>5.0)step=0.0;
    double drag=terminal_expected_energy_loss_accel(g,t,aero,v);
    if(isfinite(drag)&&drag>=0.0&&isfinite(t->true_air_speed))
        g->hac_energy_audit_actual_drag_work+=drag*t->true_air_speed*step;
    if(isfinite(t->true_air_speed))
        g->hac_energy_audit_actual_distance+=t->true_air_speed*step;
    g->hac_energy_audit_last_ut=t->ut;
}

       void hac_energy_audit_close(GuidanceMachine*g,const Telemetry*t,
        const PlanetModel*p){
    if(!g||!t||!p||!g->hac_energy_audit_active||
       g->hac_energy_audit_logged)return;
    double end_energy=rotating_specific_energy(t->latitude,t->mean_altitude,
        t->true_air_speed,p);
    double realized_loss=isfinite(g->hac_energy_audit_start_specific_energy)&&
        isfinite(end_energy)?g->hac_energy_audit_start_specific_energy-end_energy:NAN;
    double mechanical_residual=isfinite(realized_loss)?
        realized_loss-g->hac_energy_audit_actual_drag_work:NAN;
    double total_path=g->hac_energy_audit_forecast_lead_length+
        fmax(0.0,g->hac_transition_cone_arc_length);
    double forecast_work_per_m=g->hac_energy_audit_forecast_loss;
    double forecast_work=isfinite(forecast_work_per_m)&&isfinite(total_path)?
        forecast_work_per_m*total_path:NAN;
    double forecast_residual=isfinite(forecast_work)&&isfinite(realized_loss)?
        forecast_work-realized_loss:NAN;
    const char*diag=getenv("KSP_LANDER_HAC_DIAGNOSTICS");
    if(diag&&strcmp(diag,"1")==0)
        fprintf(stderr,"HAC_ENERGY_LIVE_CLOSE startUt=%.6f endUt=%.6f "
            "leadForecast=%.6f leadActual=%.6f startE=%.6f endE=%.6f "
            "realizedLoss=%.6f actualDragWork=%.6f mechanicalResidual=%.6f "
            "forecastLossPerM=%.9f forecastWork=%.6f forecastResidual=%.6f "
            "forecastAvailable=%.6f forecastRequired=%.6f forecastMargin=%.6f "
            "p0SpeedForecast=%.6f p0AltForecast=%.6f p0FpaForecast=%.6f "
            "p0AoaForecast=%.6f actualSpeed=%.6f actualAlt=%.6f actualFpa=%.6f actualAoa=%.6f\n",
            g->hac_energy_audit_start_ut,t->ut,
            g->hac_energy_audit_forecast_lead_length,
            g->hac_energy_audit_actual_distance,
            g->hac_energy_audit_start_specific_energy,end_energy,
            realized_loss,g->hac_energy_audit_actual_drag_work,
            mechanical_residual,forecast_work_per_m,forecast_work,
            forecast_residual,g->hac_energy_audit_forecast_available,
            g->hac_energy_audit_forecast_required,
            g->hac_energy_audit_forecast_margin,
            g->hac_energy_audit_forecast_p0_speed,
            g->hac_energy_audit_forecast_p0_altitude,
            g->hac_energy_audit_forecast_p0_fpa,
            g->hac_energy_audit_forecast_p0_aoa,t->true_air_speed,
            t->mean_altitude,t->flight_path_angle,t->angle_of_attack);
    g->hac_energy_audit_logged=true;
    g->hac_energy_audit_active=false;
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

static double terminal_expected_speed_loss_accel(const GuidanceMachine*g,const Telemetry*t,
        const PlanetModel*p,AerodynamicModel aero,const VehicleProfile*v){
    (void)g;
    /* Aerodynamic energy loss is not speed loss. During the recorded F14
       -32 degree glide, D/m was about 5.2 m/s2 while TAS stayed near 200 m/s:
       descending potential energy paid the drag work. The former 0.85*D/m
       floor predicted 133 m/s twenty seconds later; live KSP retained 198.
       Keep signed acceleration rather than inventing deceleration in a dive. */
    double radius=fmax(p->radius+t->mean_altitude,1.0);
    double latitude=t->latitude*DEG2RAD;
    double gravity=p->gravitational_parameter/(radius*radius)-
        p->rotational_speed*p->rotational_speed*radius*cos(latitude)*cos(latitude);
    return clampd(live_drag_accel(t,aero,v)+gravity*sin(t->flight_path_angle*DEG2RAD),-40.0,40.0);
}

