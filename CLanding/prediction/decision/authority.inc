static double measured_or_modeled_lift_accel(const Telemetry *t,
        const VehicleProfile *v) {
    if (!t || !v) return 0.0;
    if (t->mass > 0.0 && isfinite(t->lift_force) && t->lift_force > 0.0)
        return t->lift_force / t->mass;

    double ballistic = isfinite(t->estimated_ballistic_coefficient) &&
        t->estimated_ballistic_coefficient > 0.0
        ? t->estimated_ballistic_coefficient : v->estimated_ballistic_coefficient;
    double lift_to_drag = isfinite(t->estimated_lift_to_drag) &&
        t->estimated_lift_to_drag > 0.0
        ? t->estimated_lift_to_drag : v->estimated_lift_to_drag;
    if (!(ballistic > 0.0) || !(lift_to_drag > 0.0) ||
        !(t->dynamic_pressure > 0.0) || !isfinite(t->dynamic_pressure))
        return 0.0;

    double factor = lift_factor(t->mach, hypot(t->angle_of_attack, t->sideslip), v);
    return t->dynamic_pressure / ballistic * lift_to_drag * factor;
}

static double lift_factor(double mach, double aoa, const VehicleProfile *v) {
    double lift = 0.0;
    aerodynamic_force_factors_mach(mach, aoa, v, &lift, NULL);
    return fabs(lift);
}

static double lift_to_drag_factor(double mach,double aoa,const VehicleProfile*v){
    double lf=0.0,df=0.0;
    aerodynamic_force_factors_mach(mach,aoa,v,&lf,&df);
    if(!(df>DBL_MIN)||!isfinite(lf)||!isfinite(df))return 0.0;
    return fabs(lf)/df;
}

static double best_glide_aoa(double mach,const VehicleProfile*v){
    if(!v||!(v->maximum_angle_of_attack>0.0)||!isfinite(mach))return 0.0;
    double lo=0.0,hi=v->maximum_angle_of_attack;
    const double phi=(sqrt(5.0)-1.0)*0.5; /* decision-literal-ok: golden-section ratio */
    double x1=hi-phi*(hi-lo),x2=lo+phi*(hi-lo);
    double f1=lift_to_drag_factor(mach,x1,v),f2=lift_to_drag_factor(mach,x2,v);
    double resolution=sqrt(DBL_EPSILON)*fmax(1.0,hi);
    unsigned guard=0;
    while(hi-lo>resolution&&guard++<(unsigned)(4*DBL_MANT_DIG)){
        if(f1<f2){
            lo=x1;x1=x2;f1=f2;x2=lo+phi*(hi-lo);
            f2=lift_to_drag_factor(mach,x2,v);
        }else{
            hi=x2;x2=x1;f2=f1;x1=hi-phi*(hi-lo);
            f1=lift_to_drag_factor(mach,x1,v);
        }
    }
    return .5*(lo+hi); /* decision-literal-ok: interval midpoint */
}

static double maximum_lift_aoa(const Telemetry *t, const VehicleProfile *v) {
    if (!t || !v || !(v->maximum_angle_of_attack > 0.0)) return 0.0;
    double lo = 0.0;
    double hi = v->maximum_angle_of_attack;
    const double phi = (sqrt(5.0) - 1.0) * 0.5;
    double x1 = hi - phi * (hi - lo);
    double x2 = lo + phi * (hi - lo);
    double f1 = lift_factor(t->mach, x1, v);
    double f2 = lift_factor(t->mach, x2, v);
    double resolution = sqrt(DBL_EPSILON) * fmax(1.0, hi);
    unsigned guard = 0;
    while (hi - lo > resolution && guard++ < (unsigned)(4 * DBL_MANT_DIG)) {
        if (f1 < f2) {
            lo = x1;
            x1 = x2;
            f1 = f2;
            x2 = lo + phi * (hi - lo);
            f2 = lift_factor(t->mach, x2, v);
        } else {
            hi = x2;
            x2 = x1;
            f2 = f1;
            x1 = hi - phi * (hi - lo);
            f1 = lift_factor(t->mach, x1, v);
        }
    }
    double aoa = 0.5 * (lo + hi);
    /* The analytic polar has no subsonic stall break; the measured vehicle
       loses lift beyond the configured terminal CL-max AoA. */
    if (t->mach < 1.0 && v->terminal_maximum_lift_angle_of_attack > 0.0)
        aoa = fmin(aoa, v->terminal_maximum_lift_angle_of_attack);
    return aoa;
}

static double maximum_normal_accel(const Telemetry *t, const VehicleProfile *v,
        double gravity, double target_aoa) {
    double current = measured_or_modeled_lift_accel(t, v);
    if (!t || !v || !(current > 0.0)) return current;

    double current_factor = lift_factor(t->mach,
        hypot(t->angle_of_attack, t->sideslip), v);
    double target_factor = lift_factor(t->mach, target_aoa, v);
    double predicted = current;
    /* Scaling the measurement by the polar ratio is ill-conditioned near the
       zero-lift attitude (a push-over to shed energy): the tiny denominator
       would certify only the momentary lift.  Below a quarter of the target
       lift factor, use the stall-speed lift bound below. */
    // decision-literal: numerical-conditioning | Ratio scaling kept only while the current lift factor is a usable fraction of the target.
    if (current_factor > 0.25 * target_factor && isfinite(target_factor))
        predicted = current * target_factor / current_factor;
    else {
        /* The stall speed is the 1-g speed at maximum lift, so the lift
           available at the current airspeed is g*(V/Vs)^2.  The minimum safe
           speed bounds the stall speed from above and is the conservative
           fallback before calibration converges. */
        double stall = isfinite(t->calibrated_stall_speed) &&
            t->calibrated_stall_speed > 0.0
            ? t->calibrated_stall_speed : v->minimum_safe_speed;
        if (stall > 0.0 && isfinite(t->true_air_speed) && t->true_air_speed > 0.0)
            predicted = gravity * (t->true_air_speed / stall) *
                (t->true_air_speed / stall);
    }

    double structural = v->maximum_g_load > 0.0
        ? v->maximum_g_load * gravity : predicted;
    return fmin(fmax(current, predicted), structural);
}

GuidanceAttitudeLimits decision_guidance_attitude_limits(
        const Telemetry *t,const GuidanceSettings *s,GuidancePhase phase,
        bool pitch_axis) {
    GuidanceAttitudeLimits limits = {
        fabs(s ? s->entry_roll_rate : 0.0),
        fabs(s ? s->entry_roll_acceleration : 0.0)
    };
    if(pitch_axis){
        if(t&&t->attitude_response.pitch_valid){
            limits.rate_deg_s=fmin(limits.rate_deg_s,
                fabs(t->attitude_response.maximum_pitch_rate_deg_s));
            limits.accel_deg_s2=fmin(limits.accel_deg_s2,
                fabs(t->attitude_response.maximum_pitch_accel_deg_s2));
        }
    }else{
        if(phase==PHASE_TAEM||phase==PHASE_HEADING_ALIGNMENT){
            limits.rate_deg_s=fabs(s?s->taem_roll_rate:0.0);
            limits.accel_deg_s2=fabs(s?s->entry_roll_acceleration*.85:0.0);
        }else if(phase==PHASE_FINAL||phase==PHASE_FLARE){
            limits.rate_deg_s=fabs(s?s->approach_roll_rate:0.0);
            limits.accel_deg_s2=fabs(s?s->entry_roll_acceleration*.55:0.0);
        }
        if(t&&t->attitude_response.roll_valid){
            limits.rate_deg_s=fmin(limits.rate_deg_s,
                fabs(t->attitude_response.maximum_roll_rate_deg_s));
            limits.accel_deg_s2=fmin(limits.accel_deg_s2,
                fabs(t->attitude_response.maximum_roll_accel_deg_s2));
        }
    }
    /* Match the executable reference's cold-start fallback. This does not
       supply missing physical authority to a recovery envelope. */
    limits.rate_deg_s=isfinite(limits.rate_deg_s)&&limits.rate_deg_s>0.0?
        limits.rate_deg_s:1.0;
    limits.accel_deg_s2=isfinite(limits.accel_deg_s2)&&limits.accel_deg_s2>0.0?
        limits.accel_deg_s2:1.0;
    return limits;
}

static double control_response_time(const GuidanceMachine *g,
        const Telemetry *t, const GuidanceSettings *s, double target_aoa,
        bool final) {
    if (!t || !s || !isfinite(t->roll) || !isfinite(t->roll_rate))
        return INFINITY;
    double roll_rate = s->approach_roll_rate;
    double roll_accel = s->entry_roll_acceleration;
    if (t->attitude_response.roll_valid) {
        roll_rate = t->attitude_response.maximum_roll_rate_deg_s;
        roll_accel = t->attitude_response.maximum_roll_accel_deg_s2;
    }
    double roll_time = decision_axis_capture_time(-norm_signed_deg(t->roll),
        t->roll_rate, roll_accel, roll_rate);
    double pitch_time = decision_pitch_capture_time(t, target_aoa);
    if(final){
        /* A capable plant cannot follow a reference before guidance delivers
           it. Include both the measured aircraft and the in-flight command
           reference, whose position/rate can differ while a pull-up begins. */
        GuidanceAttitudeLimits pitch=decision_guidance_attitude_limits(t,s,PHASE_FINAL,true);
        GuidanceAttitudeLimits roll=decision_guidance_attitude_limits(t,s,PHASE_FINAL,false);
        double aoa_rate=t->has_angle_of_attack_rate?t->angle_of_attack_rate:0.0;
        double pitch_reference=decision_axis_capture_time(target_aoa-t->angle_of_attack,
            aoa_rate,pitch.accel_deg_s2,pitch.rate_deg_s);
        double roll_reference=decision_axis_capture_time(-norm_signed_deg(t->roll),
            t->roll_rate,roll.accel_deg_s2,roll.rate_deg_s);
        if(g&&g->pitch_limiter.has_value)
            pitch_reference=fmax(pitch_reference,decision_axis_capture_time(
                target_aoa-g->pitch_limiter.value,g->pitch_limiter.rate,
                pitch.accel_deg_s2,pitch.rate_deg_s));
        if(g&&g->roll_limiter.has_value)
            roll_reference=fmax(roll_reference,decision_axis_capture_time(
                -norm_signed_deg(g->roll_limiter.value),g->roll_limiter.rate,
                roll.accel_deg_s2,roll.rate_deg_s));
        pitch_time=fmax(pitch_time,pitch_reference);
        roll_time=fmax(roll_time,roll_reference);
    }
    /* Both axes act concurrently. Unknown axis authority cannot certify an
       instantaneous response; no historical phase delay overrides this model. */
    return fmax(pitch_time, roll_time);
}

double decision_final_approach_response_time(const GuidanceMachine *g,
        const Telemetry *t,const GuidanceSettings *s,double target_aoa) {
    return control_response_time(g,t,s,target_aoa,true);
}

ControlAuthorityEnvelope decision_control_authority_envelope(
        const GuidanceMachine *g,
        const Telemetry *t,
        const PlanetModel *p,
        const LandingConfiguration *cfg) {
    ControlAuthorityEnvelope out = {0};
    if (!t || !p || !cfg) return out;

    const VehicleProfile *v = &cfg->vehicle;
    const GuidanceSettings *s = &cfg->guidance;
    double gravity = planet_surface_gravity(p);
    if (!(gravity > 0.0)) return out;

    double minimum_speed = v->minimum_safe_speed;
    if (isfinite(t->calibrated_stall_speed) && t->calibrated_stall_speed > 0.0)
        minimum_speed = fmax(minimum_speed, t->calibrated_stall_speed);

    out.speed = decision_margin(t->true_air_speed, minimum_speed);
    out.dynamic_pressure = decision_margin(v->maximum_dynamic_pressure,
        t->dynamic_pressure);
    out.load = decision_margin(v->maximum_g_load, t->g_force);
    out.stall_observed = t->stall_fraction_is_measured &&
        isfinite(t->stall_fraction);
    if (out.stall_observed)
        out.stall = decision_margin(1.0, t->stall_fraction);

    out.minimum_speed_mps = minimum_speed;
    out.maximum_lift_aoa_deg = maximum_lift_aoa(t, v);
    out.maximum_normal_accel_mps2 = maximum_normal_accel(t, v, gravity,
        out.maximum_lift_aoa_deg);

    double bank_effectiveness = isfinite(t->bank_effectiveness) &&
        t->bank_effectiveness > 0.0 ? t->bank_effectiveness : 1.0;
    double bank_limit_rad = fabs(v->maximum_bank_angle) *
        bank_effectiveness * DEG2RAD;
    bank_limit_rad = fmin(bank_limit_rad, nextafter(LANDER_PI * 0.5, 0.0));
    out.maximum_lateral_accel_mps2 =
        out.maximum_normal_accel_mps2 * fabs(sin(bank_limit_rad));
    bool final=g&&(g->final_approach_captured||g->phase==PHASE_FINAL||
        g->phase==PHASE_FLARE||g->terminal_vertical_stage_valid);
    out.control_response_time_s = control_response_time(g, t, s,
        out.maximum_lift_aoa_deg,final);

    out.valid = out.speed.valid && out.dynamic_pressure.valid && out.load.valid &&
        isfinite(out.maximum_lift_aoa_deg) &&
        isfinite(out.maximum_normal_accel_mps2) &&
        isfinite(out.maximum_lateral_accel_mps2) &&
        isfinite(out.control_response_time_s);
    out.survivable = out.valid &&
        out.speed.margin >= 0.0 &&
        out.dynamic_pressure.margin >= 0.0 &&
        out.load.margin >= 0.0 &&
        (!out.stall_observed || (out.stall.valid && out.stall.margin >= 0.0));
    out.controllable = out.survivable &&
        out.maximum_normal_accel_mps2 > 0.0 &&
        out.maximum_lateral_accel_mps2 > 0.0;
    return out;
}

