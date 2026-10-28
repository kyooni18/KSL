//! Booster-stage trajectory integration, in SI units.
use crate::{
    ksl_constrain_turn, ksl_slew_value, ksl_smooth_value, ksl_smoothstep, ksl_turn_reference,
};

#[repr(C)]
pub struct BoosterState {
    radius: f64,
    radial_speed: f64,
    horizontal_speed: f64,
    surface_horizontal_speed: f64,
    mass: f64,
    commanded_pitch: f64,
    flown_pitch: f64,
    core_thrust: f64,
    core_mass_flow: f64,
    booster_thrust: f64,
    booster_mass_flow: f64,
    booster_time: f64,
    booster_dry_mass: f64,
    dynamic_pressure: f64,
    drag_acceleration: f64,
    lift_radial: f64,
    lift_horizontal: f64,
}
#[repr(C)]
#[derive(Clone, Copy, Default)]
pub struct Row {
    time: f64,
    radius: f64,
    radial_speed: f64,
    horizontal_speed: f64,
    mass: f64,
    raw_pitch: f64,
    commanded_pitch: f64,
    flown_pitch: f64,
    thrust: f64,
    core_thrust: f64,
    booster_thrust: f64,
    mass_flow: f64,
    dynamic_pressure: f64,
}
fn reference_apoapsis(altitude: f64, target_apoapsis: f64) -> f64 {
    let table = [
        (0., 0.),
        (0.12, 0.25),
        (0.22, 0.42),
        (0.34, 0.62),
        (0.45, 0.78),
        (0.57, 0.90),
        (0.70, 0.98),
        (0.82, 1.03),
        (1., 1.03),
    ];
    let target = target_apoapsis.max(1000.);
    let x = altitude.max(0.) / target;
    for pair in table.windows(2) {
        let (x0, y0) = pair[0];
        let (x1, y1) = pair[1];
        if x <= x1 {
            return (y0 + (y1 - y0) * (x - x0) / (x1 - x0)) * target;
        }
    }
    1.03 * target
}
fn apoapsis(mu: f64, r: f64, vr: f64, vh: f64) -> f64 {
    let energy = 0.5 * (vr * vr + vh * vh) - mu / r;
    let h = r * vh;
    if energy >= 0. {
        return f64::INFINITY;
    }
    let a = -mu / (2. * energy);
    let e = (1. - h * h / (mu * a)).max(0.).sqrt();
    a * (1. + e)
}
fn limit(pitch: f64, altitude: f64, target: f64, prograde: f64, q: f64) -> f64 {
    ksl_constrain_turn(
        pitch, altitude, target, prograde, q, 78., 5., 50000., 1.5, 20.,
    )
}
#[no_mangle]
pub unsafe extern "C" fn ksl_predict_booster(
    state: *const BoosterState,
    mu: f64,
    body_radius: f64,
    target: f64,
    bias: f64,
    end_bias: f64,
    has_end_bias: bool,
    response: f64,
    attitude: f64,
    dt: f64,
    pitch_slew: f64,
    rows: *mut Row,
    capacity: usize,
) -> isize {
    let s = &*state;
    let out = std::slice::from_raw_parts_mut(rows, capacity);
    let (mut r, mut vr, mut vh, mut mass) = (s.radius, s.radial_speed, s.horizontal_speed, s.mass);
    let (mut command, mut flown, mut t) = (s.commanded_pitch, s.flown_pitch, 0.);
    let rotation = vh - s.surface_horizontal_speed;
    let seed_speed = vr.hypot(s.surface_horizontal_speed);
    let thrust = s.core_thrust + s.booster_thrust;
    let flow = s.core_mass_flow + s.booster_mass_flow;
    let mut count = 0;
    let mut last_q = s.dynamic_pressure;
    while t < s.booster_time {
        if count >= capacity {
            return -2;
        }
        let step = dt.min(s.booster_time - t);
        let surface_h = vh - rotation;
        let speed = vr.hypot(surface_h);
        let density = ((s.radius - r) / 6000.).max(-20.).min(20.).exp();
        let ratio = density * (speed / seed_speed.max(1.)).powi(2);
        let q = s.dynamic_pressure * ratio;
        last_q = q;
        let drag = s.drag_acceleration * ratio * s.mass / mass;
        let altitude = r - body_radius;
        let prograde = vr.max(0.).atan2(surface_h.max(0.5)).to_degrees();
        let reference = ksl_turn_reference(altitude, target, 78., 5.);
        let mut base = limit(reference, altitude, target, prograde, q);
        if altitude >= 6000. {
            let error =
                (apoapsis(mu, r, vr, vh) - body_radius - reference_apoapsis(altitude, target))
                    / 1000.;
            let correction = (-0.35 * error).max(-10.).min(10.);
            base = limit(base + correction, altitude, target, prograde, q);
        }
        let bias_now = if has_end_bias {
            bias + (end_bias - bias) * ksl_smoothstep(t / s.booster_time.max(1e-6))
        } else {
            bias
        };
        let raw = limit(base + bias_now, altitude, target, prograde, q);
        out[count] = Row {
            time: t,
            radius: r,
            radial_speed: vr,
            horizontal_speed: vh,
            mass,
            raw_pitch: raw,
            commanded_pitch: command,
            flown_pitch: flown,
            thrust,
            core_thrust: s.core_thrust,
            booster_thrust: s.booster_thrust,
            mass_flow: flow,
            dynamic_pressure: q,
        };
        count += 1;
        command = ksl_slew_value(
            command,
            ksl_smooth_value(command, raw, response, step),
            pitch_slew,
            step,
        );
        flown = ksl_smooth_value(flown, command, attitude, step);
        let angle = flown.to_radians();
        let accel = thrust / mass;
        let ar = accel * angle.sin() - mu / r.powi(2) + vh.powi(2) / r - drag * vr / speed.max(1.)
            + s.lift_radial * ratio * s.mass / mass;
        let ah = accel * angle.cos() - vr * vh / r - drag * surface_h / speed.max(1.)
            + s.lift_horizontal * ratio * s.mass / mass;
        r += vr * step + 0.5 * ar * step.powi(2);
        vr += ar * step;
        vh += ah * step;
        mass -= flow * step;
        if mass <= s.booster_dry_mass {
            return -1;
        }
        t += step;
    }
    if count >= capacity {
        return -2;
    }
    out[count] = Row {
        time: t,
        radius: r,
        radial_speed: vr,
        horizontal_speed: vh,
        mass,
        raw_pitch: command,
        commanded_pitch: command,
        flown_pitch: flown,
        thrust,
        core_thrust: s.core_thrust,
        booster_thrust: s.booster_thrust,
        mass_flow: flow,
        dynamic_pressure: last_q,
    };
    (count + 1) as isize
}
