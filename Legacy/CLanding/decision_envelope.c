#include "decision_envelope.h"

#include <float.h>
#include <math.h>

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

double decision_bounded_capture_time(double position, double velocity,
        double acceleration_limit, double response_time) {
    if (!isfinite(position) || !isfinite(velocity) ||
        !isfinite(acceleration_limit) || !isfinite(response_time) ||
        acceleration_limit <= 0.0 || response_time < 0.0)
        return INFINITY;

    /*
     * Conservative bang-bang capture bound:
     *  1. wait through the measured actuator response,
     *  2. arrest existing lateral velocity at |a|max,
     *  3. translate the residual displacement rest-to-rest at |a|max.
     *
     * No corridor width or hand-tuned angle is involved.  If this bound fits
     * inside the physical time/range left, the state is dynamically capturable.
     */
    double stop_time = fabs(velocity) / acceleration_limit;
    double stop_displacement = copysign(
        velocity * velocity / (2.0 * acceleration_limit), velocity);
    double residual = position + velocity * response_time + stop_displacement;
    double translate_time = 2.0 * sqrt(fabs(residual) / acceleration_limit);
    return response_time + stop_time + translate_time;
}

double decision_axis_capture_time(double error_deg, double rate_deg_s,
        double acceleration_limit_deg_s2, double rate_limit_deg_s) {
    if (!isfinite(error_deg) || !isfinite(rate_deg_s) ||
        !isfinite(acceleration_limit_deg_s2) ||
        isnan(rate_limit_deg_s) ||
        acceleration_limit_deg_s2 <= 0.0 || rate_limit_deg_s <= 0.0)
        return INFINITY;

    /*
     * Minimum-time rest-to-rest capture for a symmetric acceleration-limited,
     * rate-limited attitude axis. Transform the signed axis into a coordinate
     * whose positive direction points from the current attitude to the target.
     * The result then follows directly from constant-acceleration kinematics:
     * either accelerate/brake triangularly, or accelerate/cruise/brake at the
     * measured/configured angular-rate limit.
     *
     * If the current rate is already too large to stop before the target, the
     * optimal trajectory necessarily overshoots and reverses. That case is
     * solved analytically instead of being hidden behind a fixed lead time.
     */
    double distance = fabs(error_deg);
    double direction;
    if (distance > 0.0)
        direction = copysign(1.0, error_deg);
    else if (rate_deg_s != 0.0)
        direction = -copysign(1.0, rate_deg_s);
    else
        return 0.0;

    double initial_rate = direction * rate_deg_s;
    double acceleration = acceleration_limit_deg_s2;
    /*
     * A measured state can transiently exceed a finite nominal rate cap. Treat
     * that current rate as the initial condition rather than pretending the
     * state is impossible. Positive infinity is a meaningful capability model:
     * it denotes an acceleration-bounded axis for which no independent angular
     * rate limit has been identified.
     */
    double rate_limit = isinf(rate_limit_deg_s)
        ? INFINITY : fmax(rate_limit_deg_s, fabs(initial_rate));
    double stopping_distance = initial_rate > 0.0
        ? initial_rate * initial_rate / (2.0 * acceleration) : 0.0;

    if (stopping_distance > distance) {
        double reverse_rate_sq =
            initial_rate * initial_rate * 0.5 - acceleration * distance;
        double reverse_rate = sqrt(fmax(0.0, reverse_rate_sq));
        return (initial_rate + 2.0 * reverse_rate) / acceleration;
    }

    double peak_rate_sq = acceleration * distance +
        initial_rate * initial_rate * 0.5;
    double peak_rate = sqrt(fmax(0.0, peak_rate_sq));

    if (peak_rate <= rate_limit) {
        return (2.0 * peak_rate - initial_rate) / acceleration;
    }

    double accelerate_distance =
        (rate_limit * rate_limit - initial_rate * initial_rate) /
        (2.0 * acceleration);
    double brake_distance =
        rate_limit * rate_limit / (2.0 * acceleration);
    double cruise_distance =
        fmax(0.0, distance - accelerate_distance - brake_distance);
    return (rate_limit - initial_rate) / acceleration +
        cruise_distance / rate_limit +
        rate_limit / acceleration;
}

double decision_pitch_capture_time(const Telemetry*t,double target_aoa){
    if(!t||!isfinite(t->angle_of_attack)||!isfinite(target_aoa))return INFINITY;

    double accel_limit=0.0;
    double rate_limit=INFINITY;

    /*
     * ShuttleSim can publish the exact closed-loop plant limits.  In the live
     * vehicle, the native FCS itself is an acceleration-bounded bang-bang axis
     * controller and has no independent pitch-rate cap, so the learned/live
     * angular acceleration authority is the matching reachability model.
     */
    if(t->attitude_response.pitch_valid){
        accel_limit=t->attitude_response.maximum_pitch_accel_deg_s2;
        rate_limit=t->attitude_response.maximum_pitch_rate_deg_s;
    }else if(isfinite(t->physics_authority[0])&&t->physics_authority[0]>0.0&&
             isfinite(t->physics_authority_confidence[0])&&
             t->physics_authority_confidence[0]>0.0){
        accel_limit=t->physics_authority[0];
    }else if(t->has_torque&&t->has_inertia&&
             isfinite(t->available_pitch_torque)&&t->available_pitch_torque>0.0&&
             isfinite(t->pitch_moment_of_inertia)&&t->pitch_moment_of_inertia>0.0){
        accel_limit=t->available_pitch_torque/t->pitch_moment_of_inertia*RAD2DEG;
    }

    if(!(accel_limit>DBL_MIN)||!isfinite(accel_limit))return INFINITY;
    return decision_axis_capture_time(target_aoa-t->angle_of_attack,
        (t->has_angle_of_attack_rate ? t->angle_of_attack_rate : 0.0),accel_limit,rate_limit);
}

static double lift_factor(double mach, double aoa, const VehicleProfile *v);

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
    return 0.5 * (lo + hi);
}

static double maximum_normal_accel(const Telemetry *t, const VehicleProfile *v,
        double gravity, double target_aoa) {
    double current = measured_or_modeled_lift_accel(t, v);
    if (!t || !v || !(current > 0.0)) return current;

    double current_factor = lift_factor(t->mach,
        hypot(t->angle_of_attack, t->sideslip), v);
    double target_factor = lift_factor(t->mach, target_aoa, v);
    double predicted = current;
    if (current_factor > DBL_EPSILON && isfinite(target_factor))
        predicted = current * target_factor / current_factor;

    double structural = v->maximum_g_load > 0.0
        ? v->maximum_g_load * gravity : predicted;
    return fmin(fmax(current, predicted), structural);
}

static double control_response_time(const GuidanceMachine *g,
        const Telemetry *t, const GuidanceSettings *s, double target_aoa) {
    (void)g;
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
    /* Both axes act concurrently. Unknown axis authority cannot certify an
       instantaneous response; no historical phase delay overrides this model. */
    return fmax(pitch_time, roll_time);
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
    out.control_response_time_s = control_response_time(g, t, s, out.maximum_lift_aoa_deg);

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

static double mod_two_pi(double x) {
    double two_pi = 2.0 * LANDER_PI;
    double y = fmod(x, two_pi);
    return y < 0.0 ? y + two_pi : y;
}

static bool dubins_lsl(double a,double b,double d,double*out) {
    double tmp=d+sin(a)-sin(b);
    double p2=2.0+d*d-2.0*cos(a-b)+2.0*d*(sin(a)-sin(b));
    if(p2<0.0)return false;
    double p=sqrt(fmax(0.0,p2));
    double x=atan2(cos(b)-cos(a),tmp);
    *out=mod_two_pi(-a+x)+p+mod_two_pi(b-x);
    return true;
}
static bool dubins_rsr(double a,double b,double d,double*out) {
    double tmp=d-sin(a)+sin(b);
    double p2=2.0+d*d-2.0*cos(a-b)+2.0*d*(-sin(a)+sin(b));
    if(p2<0.0)return false;
    double p=sqrt(fmax(0.0,p2));
    double x=atan2(cos(a)-cos(b),tmp);
    *out=mod_two_pi(a-x)+p+mod_two_pi(-b+x);
    return true;
}
static bool dubins_lsr(double a,double b,double d,double*out) {
    double p2=-2.0+d*d+2.0*cos(a-b)+2.0*d*(sin(a)+sin(b));
    if(p2<0.0)return false;
    double p=sqrt(fmax(0.0,p2));
    double x=atan2(-cos(a)-cos(b),d+sin(a)+sin(b))-atan2(-2.0,p);
    *out=mod_two_pi(-a+x)+p+mod_two_pi(-b+x);
    return true;
}
static bool dubins_rsl(double a,double b,double d,double*out) {
    double p2=d*d-2.0+2.0*cos(a-b)-2.0*d*(sin(a)+sin(b));
    if(p2<0.0)return false;
    double p=sqrt(fmax(0.0,p2));
    double x=atan2(cos(a)+cos(b),d-sin(a)-sin(b))-atan2(2.0,p);
    *out=mod_two_pi(a-x)+p+mod_two_pi(b-x);
    return true;
}
static bool dubins_rlr(double a,double b,double d,double*out) {
    double x=(6.0-d*d+2.0*cos(a-b)+2.0*d*(sin(a)-sin(b)))/8.0;
    if(x<-1.0||x>1.0)return false;
    double p=mod_two_pi(2.0*LANDER_PI-acos(clampd(x,-1.0,1.0)));
    double t=mod_two_pi(a-atan2(cos(a)-cos(b),d-sin(a)+sin(b))+p*.5);
    double q=mod_two_pi(a-b-t+p);
    *out=t+p+q;
    return true;
}
static bool dubins_lrl(double a,double b,double d,double*out) {
    double x=(6.0-d*d+2.0*cos(a-b)+2.0*d*(-sin(a)+sin(b)))/8.0;
    if(x<-1.0||x>1.0)return false;
    double p=mod_two_pi(2.0*LANDER_PI-acos(clampd(x,-1.0,1.0)));
    double t=mod_two_pi(-a-atan2(cos(a)-cos(b),d+sin(a)-sin(b))+p*.5);
    double q=mod_two_pi(b-a-t+p);
    *out=t+p+q;
    return true;
}

static double shortest_dubins_path(double de,double dn,double start_course,
        double end_course,double radius) {
    if(!isfinite(de)||!isfinite(dn)||!isfinite(start_course)||
       !isfinite(end_course)||!(radius>0.0)||!isfinite(radius))
        return INFINITY;
    double distance=hypot(de,dn);
    if(distance<=DBL_EPSILON&&
       fabs(norm_signed_deg((end_course-start_course)*RAD2DEG))<=
           sqrt(DBL_EPSILON)*RAD2DEG)
        return 0.0;

    double d=distance/radius;
    double theta=atan2(de,dn);
    double a=mod_two_pi(start_course-theta);
    double b=mod_two_pi(end_course-theta);
    double best=INFINITY,candidate=INFINITY;
    if(dubins_lsl(a,b,d,&candidate))best=fmin(best,candidate);
    if(dubins_rsr(a,b,d,&candidate))best=fmin(best,candidate);
    if(dubins_lsr(a,b,d,&candidate))best=fmin(best,candidate);
    if(dubins_rsl(a,b,d,&candidate))best=fmin(best,candidate);
    if(dubins_rlr(a,b,d,&candidate))best=fmin(best,candidate);
    if(dubins_lrl(a,b,d,&candidate))best=fmin(best,candidate);
    return isfinite(best)?best*radius:INFINITY;
}

static bool decision_target_path_geometry(
        const GuidanceMachine *g,const Telemetry *t,double course_deg,
        double target_along_m,double target_cross_m,double target_course_deg,
        double lateral_accel_limit_mps2,const PlanetModel *p,
        const LandingConfiguration *cfg,ControlAuthorityEnvelope *control_out,
        double *range_out,double *forward_out,double *cross_out,
        double *course_error_out,double *minimum_turn_radius_out,
        double *path_length_out) {
    if (!t || !p || !cfg || !isfinite(course_deg) ||
        !isfinite(target_along_m) || !isfinite(target_cross_m) ||
        !isfinite(target_course_deg) ||
        !isfinite(t->runway_along_track) || !isfinite(t->runway_cross_track))
        return false;

    ControlAuthorityEnvelope control =
        decision_control_authority_envelope(g, t, p, cfg);
    if (!control.valid || !(t->horizontal_speed > DBL_MIN) ||
        !isfinite(t->horizontal_speed))
        return false;

    double lateral=control.maximum_lateral_accel_mps2;
    if(isfinite(lateral_accel_limit_mps2))
        lateral=fmin(lateral,fmax(0.0,lateral_accel_limit_mps2));
    if(!(lateral>DBL_MIN)||!isfinite(lateral))return false;

    double da=target_along_m-t->runway_along_track;
    double dc=target_cross_m-t->runway_cross_track;
    double runway=cfg->site.runway_heading*DEG2RAD;
    double de=da*sin(runway)+dc*cos(runway);
    double dn=da*cos(runway)-dc*sin(runway);
    double target_course=target_course_deg*DEG2RAD;

    double minimum_turn=t->horizontal_speed*t->horizontal_speed/lateral;
    double path=shortest_dubins_path(de,dn,course_deg*DEG2RAD,target_course,
        minimum_turn);
    if(!isfinite(path)||!isfinite(minimum_turn))return false;

    if(control_out)*control_out=control;
    if(range_out)*range_out=hypot(de,dn);
    if(forward_out)*forward_out=de*sin(target_course)+dn*cos(target_course);
    if(cross_out)*cross_out=-de*cos(target_course)+dn*sin(target_course);
    if(course_error_out)*course_error_out=norm_signed_deg(target_course_deg-course_deg);
    if(minimum_turn_radius_out)*minimum_turn_radius_out=minimum_turn;
    if(path_length_out)*path_length_out=path;
    return true;
}

double decision_target_path_length(
        const GuidanceMachine *g,const Telemetry *t,double course_deg,
        double target_along_m,double target_cross_m,double target_course_deg,
        double lateral_accel_limit_mps2,const PlanetModel *p,
        const LandingConfiguration *cfg,double *minimum_turn_radius_m) {
    double path=INFINITY;
    if(!decision_target_path_geometry(g,t,course_deg,target_along_m,target_cross_m,
        target_course_deg,lateral_accel_limit_mps2,p,cfg,NULL,NULL,NULL,NULL,NULL,
        minimum_turn_radius_m,&path))
        return INFINITY;
    return path;
}

static TargetCaptureEnvelope decision_target_capture_envelope_impl(
        const GuidanceMachine *g,
        const Telemetry *t,
        double course_deg,
        double target_along_m,
        double target_cross_m,
        double target_course_deg,
        double available_time_s,
        double lateral_accel_limit_mps2,
        const PlanetModel *p,
        const LandingConfiguration *cfg) {
    TargetCaptureEnvelope out = {0};
    if (!t || !p || !cfg || !isfinite(course_deg) ||
        !isfinite(target_along_m) || !isfinite(target_cross_m) ||
        !isfinite(target_course_deg) ||
        !isfinite(t->runway_along_track) || !isfinite(t->runway_cross_track))
        return out;

    if(!decision_target_path_geometry(g,t,course_deg,target_along_m,target_cross_m,
        target_course_deg,lateral_accel_limit_mps2,p,cfg,&out.control,
        &out.range_m,&out.forward_m,&out.cross_m,&out.course_error_deg,
        &out.minimum_turn_radius_m,&out.path_length_m))
        return out;

    double lateral=t->horizontal_speed*t->horizontal_speed/
        out.minimum_turn_radius_m;
    out.required_time_s=isfinite(out.path_length_m)?
        out.control.control_response_time_s+
            out.path_length_m/t->horizontal_speed:INFINITY;
    out.time_available_s=available_time_s;
    out.capture_time=decision_margin(available_time_s,out.required_time_s);

    double course_rate=lateral/t->horizontal_speed;
    out.heading_capture_time_s=course_rate>0.0?
        out.control.control_response_time_s+
            fabs(out.course_error_deg*DEG2RAD)/course_rate:INFINITY;
    double cross_rate=t->horizontal_speed*
        sin(norm_signed_deg(course_deg-target_course_deg)*DEG2RAD);
    out.lateral_capture_time_s=decision_bounded_capture_time(
        out.cross_m,cross_rate,lateral,out.control.control_response_time_s);

    out.valid=out.control.valid&&out.capture_time.valid&&
        isfinite(out.path_length_m)&&isfinite(out.required_time_s);
    out.reachable=out.valid&&out.control.controllable&&
        out.capture_time.margin>=0.0;
    return out;
}

TargetCaptureEnvelope decision_target_capture_envelope(
        const GuidanceMachine *g,
        const Telemetry *t,
        double course_deg,
        double target_along_m,
        double target_cross_m,
        double target_course_deg,
        double available_time_s,
        const PlanetModel *p,
        const LandingConfiguration *cfg) {
    return decision_target_capture_envelope_impl(g,t,course_deg,target_along_m,
        target_cross_m,target_course_deg,available_time_s,NAN,p,cfg);
}

TargetCaptureEnvelope decision_target_capture_envelope_with_lateral_accel(
        const GuidanceMachine *g,
        const Telemetry *t,
        double course_deg,
        double target_along_m,
        double target_cross_m,
        double target_course_deg,
        double available_time_s,
        double lateral_accel_limit_mps2,
        const PlanetModel *p,
        const LandingConfiguration *cfg) {
    return decision_target_capture_envelope_impl(g,t,course_deg,target_along_m,
        target_cross_m,target_course_deg,available_time_s,
        lateral_accel_limit_mps2,p,cfg);
}

UnpoweredPathProjection decision_unpowered_path_projection(
        const GuidanceMachine *g,const Telemetry *t,const PlanetModel *p,
        const LandingConfiguration *cfg,double target_altitude_m,
        double ground_path_m){
    UnpoweredPathProjection out={0};
    if(!t||!p||!cfg||!isfinite(target_altitude_m)||!isfinite(ground_path_m)||
       ground_path_m<0.0||!(t->true_air_speed>0.0)||!isfinite(t->true_air_speed))
        return out;

    const VehicleProfile*v=&cfg->vehicle;
    double beta=isfinite(t->estimated_ballistic_coefficient)&&
        t->estimated_ballistic_coefficient>0.0?
        t->estimated_ballistic_coefficient:v->estimated_ballistic_coefficient;
    if(!(beta>DBL_MIN)||!isfinite(beta))return out;

    double height_delta=target_altitude_m-t->mean_altitude;
    double air_path=hypot(ground_path_m,height_delta);
    if(!(air_path>DBL_MIN)||!isfinite(air_path)){
        out.valid=true;out.ground_path_m=ground_path_m;out.air_path_m=0.0;
        out.terminal_speed_mps=t->true_air_speed;out.travel_time_s=0.0;
        out.target_aoa_deg=hypot(t->angle_of_attack,t->sideslip);
        return out;
    }

    ControlAuthorityEnvelope control=decision_control_authority_envelope(g,t,p,cfg);
    double response_distance=control.valid&&isfinite(control.control_response_time_s)?
        t->true_air_speed*fmax(0.0,control.control_response_time_s):0.0;
    double initial_aoa=hypot(t->angle_of_attack,t->sideslip);
    double target_aoa=best_glide_aoa(t->mach,v);
    double density_scale=isfinite(t->trajectory_density_scale)&&t->trajectory_density_scale>0.0?
        t->trajectory_density_scale:1.0;
    double drag_scale=isfinite(t->trajectory_drag_scale)&&t->trajectory_drag_scale>0.0?
        t->trajectory_drag_scale:1.0;

    /* Use the atmosphere table itself as the integration resolution. Each
       crossed density interval gets one exact-in-v^2 constant-coefficient
       energy step. This avoids a tuned time/distance step count. */
    unsigned segments=1;
    if(p->atmosphere_sample_count>=2&&height_delta!=0.0){
        double lo=fmin(t->mean_altitude,target_altitude_m);
        double hi=fmax(t->mean_altitude,target_altitude_m);
        for(size_t i=1;i<p->atmosphere_sample_count;i++){
            double a=p->atmosphere_altitude[i];
            if(a>lo&&a<hi)segments++;
        }
    }
    if(response_distance>DBL_MIN&&response_distance<air_path)segments++;

    double ds=air_path/(double)segments;
    double vertical_per_air=height_delta/air_path;
    double speed2=t->true_air_speed*t->true_air_speed;
    double work=0.0,time=0.0;

    for(unsigned i=0;i<segments;i++){
        double s0=(double)i*ds,s1=(double)(i+1)*ds,sm=.5*(s0+s1);
        double f=sm/air_path;
        double altitude=t->mean_altitude+height_delta*f;
        double density=planet_atmospheric_density(p,altitude)*density_scale;
        double speed0=sqrt(fmax(0.0,speed2));
        double sound=planet_atmospheric_speed_of_sound(p,altitude);
        double mach=sound>DBL_EPSILON?speed0/sound:t->mach;
        double response_progress=response_distance>DBL_MIN?
            clampd(sm/response_distance,0.0,1.0):1.0;
        double aoa=initial_aoa+(target_aoa-initial_aoa)*response_progress;
        double df=1.0;
        aerodynamic_force_factors_mach(mach,aoa,v,NULL,&df);
        double drag_coefficient=fmax(0.0,density)*fmax(0.0,df)*drag_scale/beta;
        double radius=p->radius+altitude;
        if(!(radius>DBL_MIN)||!(p->gravitational_parameter>0.0))return out;
        double gravity=p->gravitational_parameter/(radius*radius);
        double drive=-2.0*gravity*vertical_per_air;
        double next2;
        if(drag_coefficient>DBL_EPSILON){
            double equilibrium=drive/drag_coefficient;
            next2=equilibrium+(speed2-equilibrium)*exp(-drag_coefficient*ds);
        }else{
            next2=speed2+drive*ds;
        }
        next2=fmax(0.0,next2);
        double dh=vertical_per_air*ds;
        double segment_work=.5*(speed2-next2)-gravity*dh;
        work+=fmax(0.0,segment_work);
        double speed1=sqrt(next2);
        double mean_speed=.5*(speed0+speed1); /* decision-literal-ok: trapezoidal time integration */
        if(!(mean_speed>DBL_MIN))return out;
        time+=ds/mean_speed;
        speed2=next2;
    }

    out.valid=isfinite(work)&&isfinite(speed2)&&isfinite(time);
    out.ground_path_m=ground_path_m;
    out.air_path_m=air_path;
    out.modeled_drag_work=work;
    out.terminal_speed_mps=sqrt(fmax(0.0,speed2));
    out.travel_time_s=time;
    out.target_aoa_deg=target_aoa;
    return out;
}

EnergyPathEnvelope decision_energy_path_envelope(
        const Telemetry *t,
        const PlanetModel *p,
        double available_specific_energy,
        double modeled_drag_work,
        double path_length_m) {
    EnergyPathEnvelope out = {0};
    if (!t || !p || !isfinite(available_specific_energy) ||
        !isfinite(modeled_drag_work) || !isfinite(path_length_m) ||
        path_length_m < 0.0 || modeled_drag_work < 0.0)
        return out;

    double gravity = planet_surface_gravity(p);
    if (!(gravity > 0.0) || !isfinite(gravity)) return out;

    /*
     * Convert observed model residuals into the same specific-energy unit as
     * the path budget.  No confidence percentage is translated into a policy
     * margin: only measured residual magnitude and a certified relative
     * physics uncertainty contribute.
     */
    double uncertainty = 0.0;
    if (isfinite(t->trajectory_altitude_residual))
        uncertainty += gravity * fabs(t->trajectory_altitude_residual);
    if (isfinite(t->trajectory_speed_residual)) {
        double dv = fabs(t->trajectory_speed_residual);
        uncertainty += fabs(t->true_air_speed) * dv + 0.5 * dv * dv;
    }
    if (isfinite(t->trajectory_range_residual) && path_length_m > 0.0) {
        double mean_drag_loss = modeled_drag_work / path_length_m;
        uncertainty += fabs(mean_drag_loss * t->trajectory_range_residual);
    }

    double relative_uncertainty = 0.0;
    if (isfinite(t->physics_certified_uncertainty) &&
        t->physics_certified_uncertainty > 0.0)
        relative_uncertainty = t->physics_certified_uncertainty;
    if (isfinite(t->predicted_physics_relative_uncertainty) &&
        t->predicted_physics_relative_uncertainty > 0.0)
        relative_uncertainty = fmax(relative_uncertainty,
            t->predicted_physics_relative_uncertainty);
    uncertainty += modeled_drag_work * relative_uncertainty;

    out.available_specific_energy = available_specific_energy;
    out.modeled_drag_work = modeled_drag_work;
    out.uncertainty_specific_energy = uncertainty;
    out.energy = decision_margin(available_specific_energy,
        modeled_drag_work + uncertainty);
    out.valid = out.energy.valid;
    out.feasible = out.valid && out.energy.margin >= 0.0;
    return out;
}

PathFeasibilityEnvelope decision_path_feasibility_envelope(
        double path_length_m,
        double minimum_path_m,
        double maximum_path_m,
        double slope_deg,
        double minimum_slope_deg,
        double maximum_slope_deg,
        double required_lateral_accel_mps2,
        double available_lateral_accel_mps2,
        double peak_course_rate_ratio,
        double endpoint_course_rate_deg_s,
        double endpoint_course_rate_limit_deg_s) {
    PathFeasibilityEnvelope out = {0};
    if (!isfinite(path_length_m) || !isfinite(minimum_path_m) ||
        !isfinite(maximum_path_m) || !isfinite(slope_deg) ||
        !isfinite(minimum_slope_deg) || !isfinite(maximum_slope_deg) ||
        !isfinite(required_lateral_accel_mps2) ||
        !isfinite(available_lateral_accel_mps2) ||
        !isfinite(peak_course_rate_ratio) ||
        !isfinite(endpoint_course_rate_deg_s) ||
        !isfinite(endpoint_course_rate_limit_deg_s) ||
        minimum_path_m < 0.0 || maximum_path_m < minimum_path_m ||
        maximum_slope_deg < minimum_slope_deg ||
        available_lateral_accel_mps2 < 0.0 ||
        endpoint_course_rate_limit_deg_s < 0.0)
        return out;

    out.path_minimum = decision_margin(path_length_m, minimum_path_m);
    out.path_maximum = decision_margin(maximum_path_m, path_length_m);
    out.slope_minimum = decision_margin(slope_deg, minimum_slope_deg);
    out.slope_maximum = decision_margin(maximum_slope_deg, slope_deg);
    out.lateral_authority = decision_margin(
        available_lateral_accel_mps2, required_lateral_accel_mps2);
    out.course_rate = decision_margin(1.0, peak_course_rate_ratio);
    out.endpoint_rate = decision_margin(
        endpoint_course_rate_limit_deg_s, endpoint_course_rate_deg_s);

    const DecisionMargin *margins[] = {
        &out.path_minimum, &out.path_maximum,
        &out.slope_minimum, &out.slope_maximum,
        &out.lateral_authority, &out.course_rate, &out.endpoint_rate,
    };
    out.worst_normalized_margin = INFINITY;
    for (size_t i = 0; i < sizeof(margins) / sizeof(margins[0]); ++i) {
        if (!margins[i]->valid) return out;
        out.worst_normalized_margin = fmin(
            out.worst_normalized_margin, margins[i]->normalized_margin);
    }

    out.valid = isfinite(out.worst_normalized_margin);
    out.feasible = out.valid && out.worst_normalized_margin >= 0.0;
    return out;
}

RunwayCaptureEnvelope decision_runway_capture_envelope(
        const GuidanceMachine *g,
        const Telemetry *t,
        double course_deg,
        const PlanetModel *p,
        const LandingConfiguration *cfg) {
    RunwayCaptureEnvelope out = {0};
    if (!t || !p || !cfg || !isfinite(course_deg)) return out;

    const LandingSite *site = &cfg->site;

    double gravity = planet_surface_gravity(p);
    double horizontal = finite_positive(t->horizontal_speed);
    if (!(gravity > 0.0) || !(horizontal > 0.0) ||
        !isfinite(t->runway_along_track) || !isfinite(t->runway_cross_track))
        return out;

    ControlAuthorityEnvelope authority =
        decision_control_authority_envelope(g, t, p, cfg);
    if (!authority.valid) return out;

    double course_error_rad =
        norm_signed_deg(course_deg - site->runway_heading) * DEG2RAD;
    double cross_rate = horizontal * sin(course_error_rad);

    double max_lift_aoa = authority.maximum_lift_aoa_deg;
    double normal_accel = authority.maximum_normal_accel_mps2;
    double lateral_accel = authority.maximum_lateral_accel_mps2;
    double response_time = authority.control_response_time_s;
    double lateral_time = decision_bounded_capture_time(
        t->runway_cross_track, cross_rate, lateral_accel, response_time);

    double course_rate_rad = horizontal > 0.0
        ? lateral_accel / horizontal : 0.0;
    double heading_time = course_rate_rad > 0.0
        ? response_time + fabs(course_error_rad) / course_rate_rad
        : INFINITY;

    double runway_forward = site->runway_length - t->runway_along_track;
    double ground_range = hypot(fmax(0.0, runway_forward),
        t->runway_cross_track);
    double time_available = ground_range / horizontal;
    double time_required = fmax(lateral_time, heading_time);

    double sink = fmax(0.0, -t->vertical_speed);
    if (!isfinite(t->flight_path_angle) || !isfinite(t->roll) ||
        !isfinite(t->vertical_speed)) return out;
    double vertical_projection = cos(t->flight_path_angle * DEG2RAD);
    double vertical_accel = fmax(0.0, normal_accel * vertical_projection - gravity);
    double current_lift = measured_or_modeled_lift_accel(t, &cfg->vehicle);
    /* During response we credit current banked lift, but no favorable upward
       acceleration or drag support. After response the wings are level.
       Holding these accelerations is a local model, re-evaluated each sample. */
    double response_down = fmax(0.0, gravity - current_lift *
        cos(t->roll * DEG2RAD) * vertical_projection);
    double radar = isfinite(t->radar_altitude) && t->radar_altitude >= 0.0
        ? t->radar_altitude : t->mean_altitude - site->altitude;
    VerticalRecoveryEnvelope recovery = decision_vertical_recovery_envelope(
        radar, sink, 0.0, response_time, response_down, vertical_accel);
    double recovery_height = recovery.valid ? recovery.required_height_m : INFINITY;
    double recoverable_sink = 0.0;
    if (vertical_accel > 0.0 && radar >= 0.0) {
        /* Positive root of H = v*t + a*t^2/2 + (v+a*t)^2/(2*u).
           a is downward response acceleration; u is net upward authority. */
        // decision-literal: physical-law-constant | Inverse constant acceleration stopping height, including response drift.
        double radicand = vertical_accel * (vertical_accel + response_down) *
            response_time * response_time + 2.0 * vertical_accel * radar;
        recoverable_sink = fmax(0.0, sqrt(radicand) -
            (vertical_accel + response_down) * response_time);
    }

    out.runway_time = decision_margin(time_available, time_required);
    out.vertical_recovery = recovery.height;
    out.speed = authority.speed;
    out.dynamic_pressure = authority.dynamic_pressure;
    out.load = authority.load;
    out.runway_remaining = decision_margin(runway_forward, 0.0);

    out.ground_range_m = ground_range;
    out.time_available_s = time_available;
    out.lateral_capture_time_s = lateral_time;
    out.heading_capture_time_s = heading_time;
    out.control_response_time_s = response_time;
    out.cross_rate_mps = cross_rate;
    out.lateral_accel_mps2 = lateral_accel;
    out.course_rate_deg_s = course_rate_rad * RAD2DEG;
    out.maximum_lift_aoa_deg = max_lift_aoa;
    out.maximum_normal_accel_mps2 = normal_accel;
    out.sink_rate_mps = sink;
    out.vertical_recovery_accel_mps2 = vertical_accel;
    out.response_down_accel_mps2 = response_down;
    out.vertical_recovery_height_m = recovery_height;
    out.recoverable_sink_rate_mps = recoverable_sink;
    out.recoverable_fpa_deg = horizontal > 0.0
        ? atan2(recoverable_sink, horizontal) * RAD2DEG : 0.0;

    out.valid = out.runway_time.valid && out.vertical_recovery.valid &&
        out.speed.valid && out.dynamic_pressure.valid && out.load.valid &&
        out.runway_remaining.valid && isfinite(lateral_time) &&
        isfinite(heading_time);
    out.reachable = out.valid && authority.controllable &&
        out.runway_time.margin >= 0.0 &&
        out.vertical_recovery.margin >= 0.0 &&
        out.speed.margin >= 0.0 &&
        out.dynamic_pressure.margin >= 0.0 &&
        out.load.margin >= 0.0 &&
        out.runway_remaining.margin >= 0.0;
    return out;
}

/* Acquisition authority is separate from terminal-path commitment. The same
   state/energy/control envelope is evaluated by prediction and live guidance. */
TaemInterfaceCapture entry_taem_interface_capture(const TaemInterfaceTarget*q,
        const Telemetry*t,double course,const PlanetModel*p,const LandingConfiguration*cfg){
    TaemInterfaceCapture out={0};
    if(!q||!t||!p||!cfg)return out;
    const GuidanceSettings*s=&cfg->guidance;const VehicleProfile*v=&cfg->vehicle;
    if(!q->valid||!isfinite(q->along_track)||!isfinite(q->cross_track)||!isfinite(q->course)||
       !isfinite(q->altitude)||!isfinite(q->speed)||q->speed<=0||
       !isfinite(q->flight_path_angle)||!isfinite(q->specific_energy)||
       !isfinite(q->acquisition_lead)||q->acquisition_lead<=0||
       !isfinite(q->remaining_path)||q->remaining_path<q->acquisition_lead||
       !isfinite(q->response_time)||q->response_time<=0||
       !isfinite(t->latitude)||!isfinite(t->runway_along_track)||!isfinite(t->runway_cross_track)||
       !isfinite(t->true_air_speed)||!isfinite(t->mean_altitude)||
       !isfinite(t->flight_path_angle)||!isfinite(course)||
       !isfinite(t->horizontal_speed)||!isfinite(t->vertical_speed)||
       !isfinite(t->dynamic_pressure)||!isfinite(t->g_force)||
       (t->stall_fraction_is_measured&&!isfinite(t->stall_fraction)))return out;
    double h=norm_signed_deg(q->course-cfg->site.runway_heading)*DEG2RAD;
    double da=q->along_track-t->runway_along_track,dc=q->cross_track-t->runway_cross_track;
    out.along=da*cos(h)+dc*sin(h);out.cross=-da*sin(h)+dc*cos(h);
    out.course_error=norm_signed_deg(q->course-course);

    /* The handoff evaluator uses the same bounded-curvature geometry as the
       rest of the decision layer.  No minimum speed/time or pre-gate distance
       is manufactured here. */
    double horizontal=t->horizontal_speed;
    double response=q->response_time;
    ControlAuthorityEnvelope control=
        decision_control_authority_envelope(NULL,t,p,cfg);
    if(!control.valid||!control.controllable)return out;
    double minimum_turn_radius=INFINITY;
    double path_length=decision_target_path_length(NULL,t,course,
        q->along_track,q->cross_track,q->course,NAN,p,cfg,
        &minimum_turn_radius);

    double slope=-q->flight_path_angle*DEG2RAD;
    bool slope_valid=q->flight_path_angle<0.0&&q->flight_path_angle>-90.0&&
        isfinite(slope)&&cos(slope)>DBL_EPSILON;
    double corridor_altitude=slope_valid?
        q->altitude+out.along*tan(slope):NAN;
    out.altitude_error=isfinite(corridor_altitude)?
        t->mean_altitude-corridor_altitude:NAN;

    /* Energy ownership is evaluated against the downstream unpowered
       alignment/final maneuver below.  q->speed remains a planning prediction,
       not a nominal speed that the live vehicle must chase. */
    out.energy_margin=NAN;

    /* The MM304 boundary is a capture set, not an exact nonholonomic
       waypoint.  Asking the exact-point Dubins solver to end at the same
       coordinates with a sub-degree residual heading error produces a full
       2*pi turn, even though the mission contract already accepts the vehicle
       anywhere inside the 1 km tube and within the perpendicular heading
       sector.  Charge the bounded distance to that explicit set instead. */
    HandoffSetEnvelope handoff_set=decision_mm304_handoff_set_envelope(
        NULL,t,course,q->along_track,q->cross_track,p,cfg);
    double capture_path=handoff_set.valid?handoff_set.path_to_set_m:path_length;
    double required_time=isfinite(capture_path)&&horizontal>DBL_MIN?
        response+capture_path/horizontal:INFINITY;
    double available_time=isfinite(q->arrival_ut)?
        q->arrival_ut-t->ut:NAN;
    out.path_length_m=capture_path;
    out.required_time_s=required_time;
    out.available_time_s=available_time;
    out.minimum_turn_radius_m=minimum_turn_radius;
    out.turn_margin=isfinite(required_time)&&isfinite(available_time)?
        (available_time-required_time)*horizontal:NAN;

    out.valid=isfinite(out.along)&&isfinite(out.cross)&&
        isfinite(out.turn_margin)&&isfinite(minimum_turn_radius)&&slope_valid;
    if(!out.valid)return out;

    /*
     * Mission ownership contract:
     * - horizontal distance <= 1 km at the fixed rear-alignment point;
     * - ground track within 30 deg of either runway-perpendicular direction;
     * - positive usable mechanical-energy reserve;
     * - live lift/turn/height state can execute the remaining HAC/alignment.
     *
     * The first two numbers are explicit user mission requirements, not tuned
     * controller thresholds. Everything else is computed from live physics.
     */
    TaemHandoffContract handoff=taem_handoff_contract(s);
    double position_error=hypot(out.along,out.cross);
    bool inside_spatial=isfinite(position_error)&&
        position_error<=handoff.horizontal_radius_m;
    if(!inside_spatial)out.veto|=2u;

    double runway_offset=fabs(norm_signed_deg(course-cfg->site.runway_heading));
    bool heading_contract_ready=fabs(runway_offset-90.0)<=
        handoff.perpendicular_heading_half_width_deg;
    if(!heading_contract_ready)out.veto|=4u;

    if(t->true_air_speed<v->minimum_safe_speed)out.veto|=1u;
    if(!(t->vertical_speed<0.0))out.veto|=32u;

    /* Reuse the same measured turn authority that generated the bounded-curvature
       path above; handoff geometry and downstream maneuver proof must not carry
       separate turn-radius models. */
    double terminal_radius=fmax(s->hac_radius,minimum_turn_radius);

    double maneuver_path=0.0,maneuver_slope=0.0,final_altitude=0.0;
    bool maneuver_geometry=taem_alignment_maneuver_geometry(
        t->mean_altitude,terminal_radius,minimum_turn_radius,
        fabs(norm_signed_deg(q->course-cfg->site.runway_heading)),
        &cfg->site,s,&maneuver_path,&maneuver_slope,&final_altitude)&&
        t->true_air_speed>=v->minimum_safe_speed;

    double final_speed=fmax(v->final_approach_speed,v->minimum_safe_speed);
    double available_energy=entry_remaining_specific_energy(
        t->latitude,t->mean_altitude,t->true_air_speed,
        cfg->site.latitude,final_altitude,final_speed,p);
    UnpoweredPathProjection maneuver_projection={0};
    if(maneuver_geometry)
        maneuver_projection=decision_unpowered_path_projection(
            NULL,t,p,cfg,final_altitude,maneuver_path);
    EnergyPathEnvelope maneuver_energy={0};
    if(maneuver_projection.valid)
        maneuver_energy=decision_energy_path_envelope(t,p,
            available_energy,maneuver_projection.modeled_drag_work,
            maneuver_projection.air_path_m);
    out.energy_margin=maneuver_energy.valid?maneuver_energy.energy.margin:0.0;
    out.energy_available=maneuver_energy.valid?
        maneuver_energy.available_specific_energy:0.0;
    out.energy_drag_work=maneuver_energy.valid?
        maneuver_energy.modeled_drag_work:0.0;
    out.energy_uncertainty=maneuver_energy.valid?
        maneuver_energy.uncertainty_specific_energy:0.0;

    bool maneuver_ready=maneuver_geometry&&maneuver_energy.feasible;
    if(!isfinite(minimum_turn_radius)||!maneuver_geometry)out.veto|=128u;
    if(maneuver_geometry&&(!maneuver_energy.valid||out.energy_margin<0.0||
                           out.turn_margin<0.0))out.veto|=16u;

    /* Altitude/speed suitability is exactly the downstream maneuver proof.
       Do not add a second absolute altitude shell on top of it. */
    if(!maneuver_ready)out.veto|=8u;

    /* Structural margins use the configured physical limits directly. */
    if(t->dynamic_pressure>v->maximum_dynamic_pressure||
       t->g_force>v->maximum_g_load||
       (t->stall_fraction_is_measured&&t->stall_fraction>=1.0)) /* decision-literal-ok: stall fraction normalized domain */
        out.veto|=64u;

    (void)maneuver_path;
    (void)maneuver_slope;
    out.ready=out.veto==0;
    return out;
}
