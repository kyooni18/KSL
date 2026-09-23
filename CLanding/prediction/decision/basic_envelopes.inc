static double finite_positive(double value) {
    return isfinite(value) && value > 0.0 ? value : 0.0;
}

DecisionMargin decision_margin(double available, double required) {
    DecisionMargin out = {.margin = NAN, .normalized_margin = NAN};
    if (!isfinite(available) || !isfinite(required)) return out;
    out.available = available;
    out.required = required;
    out.margin = available - required;
    if (!isfinite(out.margin)) return out;
    double scale = fmax(fabs(available), fabs(required));
    out.normalized_margin = scale > DBL_MIN ? out.margin / scale : out.margin;
    out.valid = isfinite(out.normalized_margin);
    return out;
}

VerticalRecoveryEnvelope decision_vertical_recovery_envelope(
        double available_height_m, double sink_mps, double target_sink_mps,
        double response_time_s, double response_down_accel_mps2,
        double recovery_up_accel_mps2) {
    VerticalRecoveryEnvelope out = {0};
    out.height = decision_margin(NAN, NAN);
    if (!isfinite(available_height_m) || !isfinite(sink_mps) || sink_mps < 0.0 ||
        !isfinite(target_sink_mps) || target_sink_mps < 0.0 ||
        !isfinite(response_time_s) || response_time_s < 0.0 ||
        !isfinite(response_down_accel_mps2) || response_down_accel_mps2 < 0.0 ||
        !isfinite(recovery_up_accel_mps2) || recovery_up_accel_mps2 < 0.0)
        return out;

    double sink = sink_mps + response_down_accel_mps2 * response_time_s;
    // decision-literal: physical-law-constant | Constant acceleration displacement is v*t + a*t*t/2.
    double response_height = sink_mps * response_time_s +
        0.5 * response_down_accel_mps2 * response_time_s * response_time_s;
    double braking_height = 0.0, braking_time = 0.0;
    if (sink > target_sink_mps) {
        if (!(recovery_up_accel_mps2 > 0.0)) return out;
        // decision-literal: physical-law-constant | Work-energy identity v_final^2 = v_initial^2 + 2*a*distance.
        braking_height = (sink - target_sink_mps) * (sink + target_sink_mps) /
            (2.0 * recovery_up_accel_mps2);
        braking_time = (sink - target_sink_mps) / recovery_up_accel_mps2;
    }
    out.response_height_m = response_height;
    out.braking_height_m = braking_height;
    out.required_height_m = response_height + braking_height;
    out.sink_after_response_mps = sink;
    out.recovery_time_s = response_time_s + braking_time;
    out.height = decision_margin(available_height_m, out.required_height_m);
    out.valid = out.height.valid && isfinite(sink) && isfinite(out.recovery_time_s);
    out.reachable = out.valid && out.height.margin >= 0.0;
    return out;
}

HandoffSetEnvelope decision_mm304_handoff_set_envelope(
        const GuidanceMachine *g,const Telemetry *t,double course_deg,
        double center_along_m,double center_cross_m,const PlanetModel *p,
        const LandingConfiguration *cfg){
    HandoffSetEnvelope out={0};
    if(!t||!p||!cfg||!isfinite(course_deg)||
       !isfinite(center_along_m)||!isfinite(center_cross_m)||
       !isfinite(t->runway_along_track)||!isfinite(t->runway_cross_track))
        return out;

    TaemHandoffContract contract=taem_handoff_contract(&cfg->guidance);
    if(!(contract.horizontal_radius_m>0.0)||
       !(contract.perpendicular_heading_half_width_deg>=0.0))
        return out;

    out.control=decision_control_authority_envelope(g,t,p,cfg);
    double da=center_along_m-t->runway_along_track;
    double dc=center_cross_m-t->runway_cross_track;
    out.center_distance_m=hypot(da,dc);
    out.position=decision_margin(contract.horizontal_radius_m,
        out.center_distance_m);
    out.position_ready=out.position.valid&&out.position.margin>=0.0;

    const double perpendicular=90.0; /* decision-literal-ok: perpendicular runway geometry */
    double center_a=norm_deg(cfg->site.runway_heading-perpendicular);
    double center_b=norm_deg(cfg->site.runway_heading+perpendicular);
    double error_a=fabs(norm_signed_deg(course_deg-center_a));
    double error_b=fabs(norm_signed_deg(course_deg-center_b));
    double center_error=fmin(error_a,error_b);
    out.heading_error_deg=fmax(0.0,
        center_error-contract.perpendicular_heading_half_width_deg);
    out.heading=decision_margin(
        contract.perpendicular_heading_half_width_deg,center_error);
    out.heading_ready=out.heading.valid&&out.heading.margin>=0.0;

    double lateral=out.control.maximum_lateral_accel_mps2;
    out.minimum_turn_radius_m=out.control.valid&&lateral>DBL_MIN&&
        t->horizontal_speed>DBL_MIN?
        t->horizontal_speed*t->horizontal_speed/lateral:INFINITY;
    double position_excess=fmax(0.0,
        out.center_distance_m-contract.horizontal_radius_m);
    double turn_path=isfinite(out.minimum_turn_radius_m)?
        out.minimum_turn_radius_m*out.heading_error_deg*DEG2RAD:INFINITY;

    /* Translation and heading correction can occur simultaneously, but their
       sum is a conservative executable upper bound on path-to-set. It has no
       tuned capture tube: both terms come directly from the explicit mission
       set and live turn authority. */
    out.path_to_set_m=isfinite(turn_path)?position_excess+turn_path:INFINITY;
    out.inside=out.position_ready&&out.heading_ready;
    out.valid=out.position.valid&&out.heading.valid&&out.control.valid&&
        isfinite(out.path_to_set_m);
    return out;
}

TaemSpeedEnvelope decision_taem_speed_envelope(
        const VehicleProfile *v,const PlanetModel *p,double altitude_m){
    TaemSpeedEnvelope out={0};
    if(!v||!p||!isfinite(altitude_m)||
       !(v->minimum_safe_speed>0.0)||!(v->final_approach_speed>0.0)||
       !(v->maximum_dynamic_pressure>0.0)||!(v->maximum_g_load>0.0)||
       !(v->estimated_ballistic_coefficient>0.0)||
       !(v->estimated_lift_to_drag>0.0))return out;

    double density=planet_atmospheric_density(p,altitude_m);
    double radius=p->radius+altitude_m;
    if(!(density>DBL_MIN)||!(radius>DBL_MIN)||
       !(p->gravitational_parameter>0.0))return out;

    double gravity=p->gravitational_parameter/(radius*radius);
    double ld=fabs(v->estimated_lift_to_drag);
    double beta=v->estimated_ballistic_coefficient;
    double aerodynamic_ratio=sqrt(1.0+ld*ld);
    double load_q=v->maximum_g_load*gravity*beta/aerodynamic_ratio;
    double q_limit=fmin(v->maximum_dynamic_pressure,load_q);
    if(!(q_limit>0.0))return out;

    double minimum=fmax(v->minimum_safe_speed,v->final_approach_speed);
    double maximum=sqrt(2.0*q_limit/density);

    double effective_bank=fmin(fabs(v->maximum_bank_angle),nextafter(90.0,0.0));
    double lateral_per_speed2=.5*density/beta*ld*
        fabs(sin(effective_bank*DEG2RAD));
    double minimum_turn=lateral_per_speed2>DBL_MIN?
        1.0/lateral_per_speed2:INFINITY;

    out.valid=isfinite(minimum)&&isfinite(maximum)&&
        isfinite(minimum_turn);
    out.feasible=out.valid&&maximum>=minimum;
    out.minimum_speed_mps=minimum;
    out.maximum_speed_mps=maximum;
    out.density_kg_m3=density;
    out.dynamic_pressure_limit_pa=v->maximum_dynamic_pressure;
    out.load_pressure_limit_pa=load_q;
    out.minimum_turn_radius_m=minimum_turn;
    return out;
}

