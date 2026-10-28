//! Attitude-aware powered insertion forecast.
use crate::peg::PegState;
use crate::{ksl_smooth_value, ksl_smoothstep};

#[repr(C)]
pub struct Input {
    pub mu: f64,
    pub ra: f64,
    pub rp: f64,
    pub ap_tol: f64,
    pub pe_tol: f64,
    pub slew: f64,
    pub hold_slew: f64,
    pub lag: f64,
    pub lo: f64,
    pub hi: f64,
    pub response: f64,
    pub has_response: bool,
    pub rt: f64,
    pub r: f64,
    pub vr: f64,
    pub vh: f64,
    pub mass: f64,
    pub thrust: f64,
    pub ve: f64,
    pub g_limit: f64,
    pub pitch: f64,
    pub hold_left: f64,
    pub ramp: f64,
    pub cap_now: f64,
    pub dt: f64,
    pub t_max: f64,
    pub flown: f64,
    pub attitude_hold: f64,
    pub anchor: f64,
    pub has_anchor: bool,
    pub since_separation: f64,
    pub dry_mass: f64,
}
#[repr(C)]
#[derive(Clone, Copy, Default)]
pub struct Row {
    pub time: f64,
    pub radius: f64,
    pub radial_speed: f64,
    pub horizontal_speed: f64,
    pub mass: f64,
    pub raw_pitch: f64,
    pub commanded_pitch: f64,
    pub flown_pitch: f64,
    pub thrust: f64,
    pub mass_flow: f64,
    pub throttle: f64,
}
#[repr(C)]
pub struct Output {
    pub score: f64,
    pub apoapsis_r: f64,
    pub periapsis_r: f64,
    pub peak_apoapsis_r: f64,
    pub time: f64,
    pub dv: f64,
    pub saturated: f64,
    pub radius: f64,
    pub radial_speed: f64,
    pub horizontal_speed: f64,
    pub mass: f64,
    pub propellant_used: f64,
    pub fuel_feasible: bool,
    pub target_feasible: bool,
}
fn apsides(mu: f64, r: f64, vr: f64, vh: f64) -> (f64, f64) {
    let energy = 0.5 * (vr * vr + vh * vh) - mu / r;
    let h = r * vh;
    if energy >= 0. {
        return (f64::INFINITY, h * h / mu / 2.);
    }
    let a = -mu / (2. * energy);
    let e = (1. - h * h / (mu * a)).max(0.).sqrt();
    (a * (1. + e), a * (1. - e))
}
fn target_state(mu: f64, ra: f64, rp: f64, r: f64) -> (f64, f64) {
    let r = r.max(rp).min(ra);
    let vh = (2. * mu * ra * rp / (ra + rp)).sqrt() / r;
    let vr = (2. * mu * (1. / r - 1. / (ra + rp)) - vh * vh)
        .max(0.)
        .sqrt();
    (vr, vh)
}
fn derivative(mu: f64, r: f64, vr: f64, vh: f64, a: f64, p: f64) -> (f64, f64, f64) {
    (
        vr,
        a * p.sin() - mu / (r * r) + vh * vh / r,
        a * p.cos() - vr * vh / r,
    )
}
#[no_mangle]
pub unsafe extern "C" fn ksl_insertion_fly(
    input: *const Input,
    output: *mut Output,
    rows: *mut Row,
    capacity: usize,
) -> isize {
    let s = &*input;
    let out = std::slice::from_raw_parts_mut(rows, capacity);
    let (mut r, mut vr, mut vh, mut mass, mut pitch, mut flown) =
        (s.r, s.vr, s.vh, s.mass, s.pitch, s.flown);
    let m0 = mass;
    let mut t = 0.;
    let mut peak_ra: f64 = 0.;
    let mut saturated = 0.;
    let mut guide = PegState::new(s.mu, 8.);
    let mut count = 0;
    while t < s.t_max {
        if count >= capacity {
            return -2;
        }
        let cap = if t < s.hold_left {
            s.cap_now
        } else if s.ramp <= 0. {
            1.
        } else {
            (s.cap_now + (1. - s.cap_now) * (t - s.hold_left) / s.ramp).min(1.)
        };
        let avail = s.thrust * cap / mass;
        let accel = avail.min(s.g_limit);
        let (vrt, vht) = target_state(s.mu, s.ra, s.rp, s.rt);
        let (raw, tgo) = guide.update(
            t,
            r,
            vr,
            vh,
            accel,
            s.ve,
            s.rt,
            vrt,
            vht,
            accel < 0.99 * avail,
        );
        let mut cmd = if raw.is_none() || tgo < guide.freeze_tgo || r >= s.rt - 500. {
            let (vrh, vhh) = target_state(s.mu, s.ra, s.rp, r);
            let dvr = vrh - vr;
            let dvh = vhh - vh;
            let go = (dvr.hypot(dvh) / accel.max(0.1)).max(1.);
            let g = s.mu / (r * r) - vh * vh / r;
            (dvr / go + g).atan2((dvh / go).max(1e-3)).to_degrees()
        } else {
            raw.unwrap()
        };
        if cmd < s.lo || cmd > s.hi {
            saturated += s.dt;
        }
        cmd = cmd.min(s.hi).max(s.lo);
        if s.has_anchor {
            cmd = s.anchor + ksl_smoothstep((s.since_separation + t - 5.) / 30.) * (cmd - s.anchor);
        }
        out[count] = Row {
            time: t,
            radius: r,
            radial_speed: vr,
            horizontal_speed: vh,
            mass,
            raw_pitch: if t < s.attitude_hold { pitch } else { cmd },
            commanded_pitch: pitch,
            flown_pitch: flown,
            thrust: accel * mass,
            mass_flow: accel * mass / s.ve,
            throttle: accel * mass / s.thrust,
        };
        count += 1;
        if t >= s.attitude_hold {
            if s.has_response {
                pitch = ksl_smooth_value(pitch, cmd, s.response, s.dt);
            } else {
                let rate = if t < s.hold_left { s.hold_slew } else { s.slew };
                pitch += (cmd - pitch).max(-rate * s.dt).min(rate * s.dt);
            }
            flown += (pitch - flown) * (s.dt / s.lag.max(1e-3)).min(1.);
        }
        let p = flown.to_radians();
        let (k0, k1, k2) = derivative(s.mu, r, vr, vh, accel, p);
        let (k0, k1, k2) = derivative(
            s.mu,
            r + k0 * s.dt / 2.,
            vr + k1 * s.dt / 2.,
            vh + k2 * s.dt / 2.,
            accel,
            p,
        );
        r += k0 * s.dt;
        vr += k1 * s.dt;
        vh += k2 * s.dt;
        mass -= accel * mass / s.ve * s.dt;
        t += s.dt;
        let (ra, rp) = apsides(s.mu, r, vr, vh);
        peak_ra = peak_ra.max(ra.min(10. * s.ra));
        if (ra >= s.ra - s.ap_tol && rp >= s.rp - s.pe_tol) || mass < s.dry_mass.max(0.2 * m0) {
            break;
        }
    }
    let (ra, rp) = apsides(s.mu, r, vr, vh);
    let dv = s.ve * (m0 / mass.max(1.)).ln();
    let score = (ra.min(10. * s.ra) - s.ra).abs()
        + (rp - s.rp).abs()
        + 2. * (peak_ra - s.ra - s.ap_tol).max(0.)
        + 20. * dv
        + 500. * saturated;
    *output = Output {
        score,
        apoapsis_r: ra,
        periapsis_r: rp,
        peak_apoapsis_r: peak_ra,
        time: t,
        dv,
        saturated,
        radius: r,
        radial_speed: vr,
        horizontal_speed: vh,
        mass,
        propellant_used: m0 - mass,
        fuel_feasible: mass >= s.dry_mass,
        target_feasible: ra >= s.ra - s.ap_tol
            && ra <= s.ra + s.ap_tol
            && rp >= s.rp - s.pe_tol
            && peak_ra <= s.ra + s.ap_tol,
    };
    if count > 0 {
        if count >= capacity {
            return -2;
        }
        let mut final_row = out[count - 1];
        final_row.time = t;
        final_row.radius = r;
        final_row.radial_speed = vr;
        final_row.horizontal_speed = vh;
        final_row.mass = mass;
        final_row.commanded_pitch = pitch;
        final_row.flown_pitch = flown;
        out[count] = final_row;
        count += 1;
    }
    count as isize
}
