//! Native ascent guidance. Angles are in degrees and time in simulation seconds.

#[no_mangle]
pub extern "C" fn ksl_roll_settled(
    roll: f64,
    rate: f64,
    nose: f64,
    angle_limit: f64,
    rate_limit: f64,
    nose_limit: f64,
) -> bool {
    roll.abs() <= angle_limit && rate.abs() <= rate_limit && nose.abs() <= nose_limit
}

#[no_mangle]
pub extern "C" fn ksl_launch_roll_handoff(
    commanded: f64,
    actual: f64,
    target: f64,
    roll: f64,
    rate: f64,
    nose: f64,
) -> bool {
    (commanded - target).abs() <= 2.5
        && (actual - target).abs() <= 8.0
        && ksl_roll_settled(roll, rate, nose, 7.0, 3.0, 8.0)
}

#[no_mangle]
pub extern "C" fn ksl_maneuver_roll_rate(error: f64, duration: f64, limit: f64) -> f64 {
    limit.min((1.4 * error.abs() / duration).min(30.0).max(3.0))
}

#[no_mangle]
pub extern "C" fn ksl_roll_allowed(
    liftoff: bool,
    gain: f64,
    start: f64,
    speed: f64,
    minimum: f64,
) -> bool {
    liftoff && gain >= start && speed.is_finite() && speed >= minimum
}

const TURN_PROFILE: [(f64, f64); 8] = [
    (500., 78.),
    (5000., 75.),
    (10000., 68.),
    (20000., 55.),
    (30000., 42.),
    (40000., 28.),
    (55000., 12.),
    (65000., 5.),
];

#[no_mangle]
pub extern "C" fn ksl_turn_reference(
    altitude: f64,
    target: f64,
    initial: f64,
    minimum: f64,
) -> f64 {
    let h = altitude.max(0.) * 90000. / target.max(1000.);
    if h <= TURN_PROFILE[0].0 {
        return initial;
    }
    if h >= TURN_PROFILE[7].0 {
        return minimum;
    }
    let mut slopes = [0.; 7];
    for i in 0..7 {
        slopes[i] = (TURN_PROFILE[i + 1].1 - TURN_PROFILE[i].1)
            / (TURN_PROFILE[i + 1].0 - TURN_PROFILE[i].0);
    }
    let mut tangents = [0.; 8];
    for i in 1..7 {
        tangents[i] = 2. * slopes[i - 1] * slopes[i] / (slopes[i - 1] + slopes[i]);
    }
    for i in 0..7 {
        let (x0, y0) = TURN_PROFILE[i];
        let (x1, y1) = TURN_PROFILE[i + 1];
        if h <= x1 {
            let u = (h - x0) / (x1 - x0);
            let pitch = (2. * u.powi(3) - 3. * u.powi(2) + 1.) * y0
                + (u.powi(3) - 2. * u.powi(2) + u) * (x1 - x0) * tangents[i]
                + (-2. * u.powi(3) + 3. * u.powi(2)) * y1
                + (u.powi(3) - u.powi(2)) * (x1 - x0) * tangents[i + 1];
            return minimum + (initial - minimum) * (pitch - 5.) / 73.;
        }
    }
    minimum
}

#[no_mangle]
pub extern "C" fn ksl_constrain_turn(
    pitch: f64,
    altitude: f64,
    target: f64,
    prograde: f64,
    q: f64,
    initial: f64,
    minimum: f64,
    q_alpha: f64,
    alpha_floor: f64,
    alpha_ceiling: f64,
) -> f64 {
    let reference = ksl_turn_reference(altitude, target, initial, minimum);
    let h = altitude.max(0.) * 90000. / target.max(1000.);
    let t = ((h - 10000.) / 35000.).max(0.).min(1.);
    let margin = 3. + 12. * t * t * (3. - 2. * t);
    let desired = pitch.min(reference + margin).max(reference - margin);
    let alpha = (q_alpha / q.max(1.)).min(alpha_ceiling).max(alpha_floor);
    desired
        .min(prograde + alpha)
        .max(prograde - alpha)
        .max(minimum)
        .min(initial)
}

#[no_mangle]
pub extern "C" fn ksl_smoothstep(value: f64) -> f64 {
    let x = value.max(0.).min(1.);
    x * x * (3. - 2. * x)
}

#[no_mangle]
pub extern "C" fn ksl_slew_value(current: f64, target: f64, rate: f64, dt: f64) -> f64 {
    if dt <= 0. || rate <= 0. {
        return current;
    }
    let step = rate * dt;
    current + (target - current).min(step).max(-step)
}

#[no_mangle]
pub extern "C" fn ksl_smooth_value(current: f64, target: f64, response: f64, dt: f64) -> f64 {
    if dt <= 0. {
        return current;
    }
    if response <= 0. {
        return target;
    }
    current + (target - current) * (-(-dt / response).exp_m1())
}

mod attitude;

mod allocation;

mod booster;
