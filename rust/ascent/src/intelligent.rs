//! Intelligent ascent planner and measured-force trajectory model.
use crate::peg::PegState;
#[repr(C)]
#[derive(Clone, Copy)]
pub struct State {
    altitude: f64,
    radial: f64,
    horizontal: f64,
    mass: f64,
    thrust: f64,
    isp: f64,
    q: f64,
    drag: f64,
    fixed_thrust: f64,
    surface_horizontal: f64,
    lift_radial: f64,
    lift_horizontal: f64,
    mass_flow: f64,
    fixed_mass_flow: f64,
}
#[repr(C)]
#[derive(Clone, Copy)]
pub struct Target {
    radius: f64,
    mu: f64,
    atmosphere: f64,
    apoapsis: f64,
    periapsis: f64,
    inclination: f64,
    altitude_tolerance: f64,
    inclination_tolerance: f64,
}
#[repr(C)]
pub struct Planner {
    pub max_q: f64,
    pub max_g: f64,
    pub slew: f64,
    pub pitch: f64,
    pub clock: f64,
    pub peg: PegState,
    pub has_peg: bool,
    pub peg_active: bool,
    pub has_plan: bool,
    pub plan_rt: f64,
    pub plan_rp: f64,
}
#[repr(C)]
pub struct Command {
    pitch: f64,
    throttle: f64,
    predicted_apoapsis: f64,
    horizon: f64,
    status: i32,
}
fn clamp(x: f64, lo: f64, hi: f64) -> f64 {
    x.min(hi).max(lo)
}
fn apsides(s: State, t: Target) -> (f64, f64) {
    let r = t.radius + s.altitude;
    let h = r * s.horizontal;
    let energy = 0.5 * (s.radial * s.radial + s.horizontal * s.horizontal) - t.mu / r;
    if energy >= 0. {
        return (f64::INFINITY, -t.radius);
    }
    let a = -t.mu / (2. * energy);
    let e = (1. - h * h / (t.mu * a)).max(0.).sqrt();
    (a * (1. + e) - t.radius, a * (1. - e) - t.radius)
}
fn flow(s: State, throttle: f64) -> (f64, f64) {
    let force = s.fixed_thrust + (s.thrust - s.fixed_thrust) * throttle;
    let flow = if s.mass_flow >= 0. {
        s.fixed_mass_flow + (s.mass_flow - s.fixed_mass_flow) * throttle
    } else {
        force / (s.isp.max(1.) * 9.80665)
    };
    (force, flow)
}
fn step(mut s: State, t: Target, pitch: f64, throttle: f64, dt: f64) -> State {
    let p = pitch.to_radians();
    let (force, flow) = flow(s, throttle);
    let deriv = |h: f64, vr: f64, vt: f64, mass: f64| {
        let r = t.radius + h;
        let speed = vr.hypot(vt).max(1.);
        let density = clamp((s.altitude - h) / 6000., -20., 20.).exp();
        let drag = s.drag * density;
        (
            vr,
            force / mass.max(1.) * p.sin() - t.mu / r.powi(2) + vt * vt / r - drag * vr / speed
                + s.lift_radial * density,
            force / mass.max(1.) * p.cos() - vr * vt / r - drag * vt / speed
                + s.lift_horizontal * density,
        )
    };
    let d = deriv(s.altitude, s.radial, s.horizontal, s.mass);
    let mid = deriv(
        s.altitude + d.0 * dt / 2.,
        s.radial + d.1 * dt / 2.,
        s.horizontal + d.2 * dt / 2.,
        s.mass - flow * dt / 2.,
    );
    s.altitude += mid.0 * dt;
    s.radial += mid.1 * dt;
    s.horizontal += mid.2 * dt;
    s.mass = (s.mass - flow * dt).max(1.);
    s
}
fn coast_apoapsis(mut s: State, t: Target) -> f64 {
    s.fixed_thrust = 0.;
    let initial = s;
    for _ in 0..240 {
        if s.radial <= 0. || s.altitude >= t.atmosphere {
            break;
        }
        let ratio = (s.radial * s.radial + s.horizontal * s.horizontal)
            / (initial.radial.powi(2) + initial.horizontal.powi(2)).max(1.);
        let density = clamp((initial.altitude - s.altitude) / 5600., -30., 20.).exp();
        s.drag = initial.drag * density * ratio;
        s.lift_radial = initial.lift_radial * density;
        s.lift_horizontal = initial.lift_horizontal * density;
        s = step(s, t, 0., 0., 1.);
    }
    apsides(s, t).0
}
fn target_state(mu: f64, ra: f64, rp: f64, r: f64) -> (f64, f64) {
    let r = clamp(r, rp, ra);
    let vh = (2. * mu * ra * rp / (ra + rp)).sqrt() / r;
    let vr = (2. * mu * (1. / r - 1. / (ra + rp)) - vh * vh)
        .max(0.)
        .sqrt();
    (vr, vh)
}
fn solve(
    peg: &mut PegState,
    clock: f64,
    s: State,
    t: Target,
    force: f64,
    ve: f64,
    constant: bool,
    rt: f64,
    rp: f64,
) -> (Option<f64>, f64) {
    let (vrt, vht) = target_state(t.mu, t.radius + t.apoapsis, rp, rt);
    peg.update(
        clock,
        t.radius + s.altitude,
        s.radial,
        s.horizontal,
        force / s.mass,
        ve,
        rt,
        vrt,
        vht,
        constant,
    )
}
fn find_plan(planner: &mut Planner, s: State, t: Target, force: f64, ve: f64, constant: bool) {
    let ra = t.radius + t.apoapsis;
    let low = t.apoapsis.min(s.altitude.max(0.75 * t.atmosphere) + 2000.);
    let mut best: Option<(f64, f64, f64, PegState)> = None;
    for i in 0..8 {
        let rt = t.radius + low + (t.apoapsis - low) * i as f64 / 7.;
        for k in 0..9 {
            let rp = t.radius + t.periapsis - (t.periapsis + 0.3 * t.radius) * k as f64 / 8.;
            if rp > rt {
                continue;
            }
            let mut trial = PegState::new(t.mu, 8.);
            solve(&mut trial, planner.clock, s, t, force, ve, constant, rt, rp);
            if !trial.converged || trial.tgo <= 1. {
                continue;
            }
            let burn = if constant {
                force / s.mass * trial.tgo
            } else {
                -ve * (1. - trial.tgo * (force / s.mass) / ve).ln()
            };
            let vap = (2. * t.mu * rp / (ra * (ra + rp))).sqrt();
            let goal = t.radius + t.periapsis;
            let vgoal = (2. * t.mu * goal / (ra * (ra + goal))).sqrt();
            let cost = burn + (vgoal - vap).abs();
            if best.as_ref().map_or(true, |b| cost < b.0) {
                best = Some((cost, rt, rp, trial));
            }
        }
    }
    if let Some((_, rt, rp, peg)) = best {
        planner.has_plan = true;
        planner.plan_rt = rt;
        planner.plan_rp = rp;
        planner.peg = peg;
    } else {
        planner.has_plan = false;
    }
}
fn command(p: &mut Planner, s: State, t: Target, dt: f64) -> Command {
    p.clock += dt;
    let accel = s.thrust / s.mass;
    let mut throttle = clamp(
        (p.max_g * 9.80665 * s.mass - s.fixed_thrust) / (s.thrust - s.fixed_thrust).max(0.01),
        0.,
        1.,
    );
    if s.q > p.max_q {
        throttle *= (p.max_q / s.q).sqrt();
    }
    let aero = if s.surface_horizontal < 0. {
        s.horizontal
    } else {
        s.surface_horizontal
    };
    let speed = s.radial.hypot(aero);
    let launch = clamp(90. - (speed - 35.).max(0.) * 0.08, 0., 90.);
    let mut ap = apsides(s, t).0;
    let mut desired = launch;
    if ap > t.apoapsis * 0.8 {
        desired = clamp(launch - (ap - t.apoapsis * 0.8) / 1500., -60., launch);
    }
    let prograde = s.radial.atan2(aero.max(0.1)).to_degrees();
    let alpha = (50000. / s.q.max(1.)).max(1.5);
    let (mut lower, mut upper) = (-75.0_f64, 90.0_f64);
    if s.q > 100. {
        lower = lower.max(prograde - alpha);
        upper = upper.min(prograde + alpha);
    }
    lower = lower.max(p.pitch - p.slew * dt);
    upper = upper.min(p.pitch + p.slew * dt);
    if lower > upper {
        lower = clamp(prograde, p.pitch - p.slew * dt, p.pitch + p.slew * dt);
        upper = lower;
    }
    let ready = p.has_peg
        && s.altitude > 0.3 * t.atmosphere
        && s.q < 0.05 * p.max_q
        && s.radial > 0.
        && s.fixed_thrust < 1.;
    if p.peg_active || ready {
        let (force, flow) = flow(s, throttle);
        let ve = force / flow.max(1e-6);
        let constant = throttle < 0.999;
        let ending = p.has_plan && p.peg.tgo < 2. * p.peg.freeze_tgo;
        if !ending
            && (!p.has_plan || !(p.peg.tgo != 0. && p.peg.tgo.is_finite() && p.peg.peak < 1.2))
        {
            find_plan(p, s, t, force, ve, constant);
        }
        let (mut peg_pitch, tgo) = if p.has_plan {
            solve(
                &mut p.peg, p.clock, s, t, force, ve, constant, p.plan_rt, p.plan_rp,
            )
        } else {
            (None, 0.)
        };
        ap = apsides(s, t).0;
        if peg_pitch.is_some()
            && tgo <= 0.2_f64.max(1.5 * dt)
            && ap < t.apoapsis - t.altitude_tolerance
        {
            p.has_plan = false;
            peg_pitch = None;
            p.peg = PegState::new(t.mu, 8.);
        }
        p.peg_active = peg_pitch.is_some();
        if let Some(pitch) = peg_pitch {
            p.pitch = clamp(pitch, lower, upper);
            return Command {
                pitch: p.pitch,
                throttle,
                predicted_apoapsis: ap,
                horizon: tgo,
                status: if tgo <= 0.2_f64.max(1.5 * dt) { 2 } else { 1 },
            };
        }
    }
    let nominal = clamp(desired, lower, upper);
    if s.q > p.max_q * 0.6 && speed > 1. {
        let near = step(s, t, nominal, throttle, 2.);
        let surface_next = aero + (near.horizontal - s.horizontal);
        let future = s.q * (near.radial.powi(2) + surface_next.powi(2)) / (speed * speed)
            * clamp((s.altitude - near.altitude) / 5600., -10., 10.).exp();
        if future > p.max_q {
            throttle *= (p.max_q / future).sqrt();
        }
    }
    let rp = t.radius + t.periapsis;
    let ra = t.radius + t.apoapsis;
    let insertion_speed = (2. * t.mu * ra / (rp * (ra + rp))).sqrt();
    let horizon = clamp(
        (insertion_speed - s.horizontal) / accel.max(0.1) * 0.12,
        3.,
        15.,
    );
    let mut choices: Vec<_> = [-6., -3., 0., 3., 6.]
        .iter()
        .map(|d| clamp(nominal + d, lower, upper))
        .collect();
    choices.sort_by(|a, b| a.total_cmp(b));
    choices.dedup();
    let mut best: Option<(f64, f64, f64)> = None;
    for pitch in choices {
        let mut pred = s;
        for _ in 0..10 {
            pred = step(pred, t, pitch, throttle, horizon / 10.);
        }
        let ap = apsides(pred, t).0;
        let excess = if ap.is_finite() {
            (ap - t.apoapsis).max(0.)
        } else {
            1e7
        };
        let mut score = ((pitch - nominal) / 6.).powi(2) + (excess / 10000.).powi(2);
        if pred.altitude < 0. {
            score += 1e6;
        }
        if best.as_ref().map_or(true, |b| score < b.0) {
            best = Some((score, pitch, ap));
        }
    }
    let (_, pitch, predicted_apoapsis) = best.unwrap();
    p.pitch = pitch;
    Command {
        pitch,
        throttle,
        predicted_apoapsis,
        horizon,
        status: 0,
    }
}
#[no_mangle]
pub unsafe extern "C" fn ksl_intelligent_command(
    p: *mut Planner,
    s: *const State,
    t: *const Target,
    dt: f64,
    out: *mut Command,
) {
    *out = command(&mut *p, *s, *t, dt);
}
#[no_mangle]
pub unsafe extern "C" fn ksl_intelligent_step(
    s: *const State,
    t: *const Target,
    pitch: f64,
    throttle: f64,
    dt: f64,
    out: *mut State,
) {
    *out = step(*s, *t, pitch, throttle, dt);
}
#[no_mangle]
pub unsafe extern "C" fn ksl_intelligent_apsides(s: *const State, t: *const Target, out: *mut f64) {
    let (ap, pe) = apsides(*s, *t);
    *out = ap;
    *out.add(1) = pe;
}
#[no_mangle]
pub unsafe extern "C" fn ksl_intelligent_coast(s: *const State, t: *const Target) -> f64 {
    coast_apoapsis(*s, *t)
}
