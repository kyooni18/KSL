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

