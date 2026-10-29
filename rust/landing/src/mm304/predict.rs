//! Closed-loop MM304 forecast: 3-DOF over the rotating planet with the
//! predictor's measured-calibrated aerodynamics, the MM304 law in the loop on a
//! copy of its state, and a rate- and acceleration-limited roll response.
//! A forecast that does not reach the MM305 boundary, leaves the load envelope
//! or cannot be scored by MM305 is reported as such, never as a success.

use core::ffi::c_void;

use super::law::{self, LawInput};
use super::state::*;
use super::terminal::{self, Handoff, Site};
use crate::prediction::predictor::{
    atmosphere, predictor_geo_point, rk4_aero, AeroContext, AerodynamicEnvelope, GeoPoint, PlanetModel,
    TerminalModel, TrajectoryCalibrationModel, Vector3, VehicleProfile, VehicleState,
};

extern "C" {
    fn entry_bank_authority_limit(speed: f64, q: f64, g_force: f64, v: *const VehicleProfile, maximum_bank: f64) -> f64;
    fn aero_force_per_q(table: *const c_void, q: f64, mach: f64, alpha_deg: f64, lift_per_q: *mut f64, drag_per_q: *mut f64);
}

const G0: f64 = 9.80665;
/// Bank the MM304 plan may use, the same cap as live guidance.
pub const MM304_PLANNED_MAX_BANK_DEG: f64 = 40.0;

/// Bank authority at a flight condition: the vehicle's q/g/speed authority, capped.
pub unsafe fn bank_limit(speed: f64, q: f64, g_force: f64, v: *const VehicleProfile) -> f64 {
    entry_bank_authority_limit(speed, q, g_force, v, (*v).maximum_bank_angle).min(MM304_PLANNED_MAX_BANK_DEG)
}

pub struct PredictInput {
    pub state: VehicleState,
    pub planet: *const PlanetModel,
    pub envelope: *const AerodynamicEnvelope,
    pub calibration: *const TrajectoryCalibrationModel,
    pub vehicle: *const VehicleProfile,
    pub model: *const TerminalModel,
    pub live_aero: *const c_void,
    pub site: Site,
    pub schedule: Mm304Schedule,
    pub law: Mm304Law,
    pub target: GeoPoint,
    pub station_range: f64,
    pub handoff_mach: f64,
    pub min_altitude: f64,
    pub max_altitude: f64,
    /// Live coordination reduction of the bank authority (1 when none).
    pub bank_limit_scale: f64,
    pub roll_rate: f64,
    pub roll_accel: f64,
    pub cross_uncertainty: f64,
    pub bank: f64,
    pub bank_rate: f64,
    pub aoa: f64,
    pub max_duration: f64,
    pub surface_gravity: f64,
}

pub struct Forecast {
    pub converged: bool,
    pub in_envelope: bool,
    pub terminal: Mm304Terminal,
    pub reversals: u32,
    /// The plan's commands at the first step: AoA, the schedule there, energy bank.
    pub first_aoa: f64,
    pub first_nominal: f64,
    pub first_energy_bank: f64,
    pub sample_count: usize,
    pub samples: [Mm304Sample; MM304_SAMPLES],
}

impl Forecast {
    pub fn energy(&self) -> f64 {
        if self.converged && self.terminal.valid { self.terminal.energy_error } else { f64::NAN }
    }
}

fn dot(a: Vector3, b: Vector3) -> f64 {
    a.x * b.x + a.y * b.y + a.z * b.z
}
fn cross(a: Vector3, b: Vector3) -> Vector3 {
    Vector3 { x: a.y * b.z - a.z * b.y, y: a.z * b.x - a.x * b.z, z: a.x * b.y - a.y * b.x }
}
fn unit(a: Vector3) -> Vector3 {
    let m = dot(a, a).sqrt();
    if m > 0.0 { Vector3 { x: a.x / m, y: a.y / m, z: a.z / m } } else { a }
}

/// Great-circle range and initial bearing on a sphere of radius `r`.
pub fn range_bearing(from: GeoPoint, to: GeoPoint, r: f64) -> (f64, f64) {
    let (p1, p2) = (from.latitude.to_radians(), to.latitude.to_radians());
    let dl = (to.longitude - from.longitude).to_radians();
    let a = ((p2 - p1) * 0.5).sin().powi(2) + p1.cos() * p2.cos() * (dl * 0.5).sin().powi(2);
    let range = 2.0 * r * a.sqrt().min(1.0).asin();
    let bearing = (dl.sin() * p2.cos()).atan2(p1.cos() * p2.sin() - p1.sin() * p2.cos() * dl.cos());
    (range, bearing.to_degrees().rem_euclid(360.0))
}

/// The AoA of best lift-to-drag from MM305's aero table, at or below `ceiling`.
pub unsafe fn max_lift_to_drag_aoa(model: *const TerminalModel, q: f64, mach: f64, ceiling: f64) -> f64 {
    if model.is_null() || !mach.is_finite() || !(ceiling > 0.0) {
        return f64::NAN;
    }
    let table = &raw const (*model).aero as *const c_void;
    let (mut best, mut best_ld) = (f64::NAN, f64::NEG_INFINITY);
    let mut a = 4.0;
    while a <= ceiling + 1e-9 {
        let (mut l, mut d) = (0.0, 0.0);
        aero_force_per_q(table, q.max(1.0), mach, a, &mut l, &mut d);
        if d > f64::MIN_POSITIVE && l / d > best_ld {
            best_ld = l / d;
            best = a;
        }
        a += 0.5;
    }
    best
}

/// Energy height above the site, m.
pub fn energy_height(altitude: f64, site_altitude: f64, speed: f64, g: f64) -> f64 {
    altitude - site_altitude + speed * speed / (2.0 * g)
}

fn track_roll(bank: &mut f64, rate: &mut f64, target: f64, max_rate: f64, max_accel: f64, dt: f64) {
    let err = target - *bank;
    let stop = (2.0 * max_accel * err.abs()).sqrt();
    let want = err.signum() * max_rate.min(stop).min(err.abs() / dt.max(1e-6));
    *rate += (want - *rate).clamp(-max_accel * dt, max_accel * dt);
    *bank += *rate * dt;
}

pub unsafe fn forecast(x: &PredictInput, sigma: f64) -> Forecast {
    let p = x.planet;
    let mut out = Forecast {
        converged: false,
        in_envelope: true,
        terminal: Mm304Terminal { valid: false, admissible: false, ..core::mem::zeroed() },
        reversals: 0,
        first_aoa: f64::NAN,
        first_nominal: f64::NAN,
        first_energy_bank: f64::NAN,
        sample_count: 0,
        samples: [core::mem::zeroed(); MM304_SAMPLES],
    };
    let mut st = x.state;
    let mut law = x.law;
    let reversals0 = law.reversals;
    let mut ctx = AeroContext {
        p,
        e: x.envelope,
        c: x.calibration,
        v: x.vehicle,
        entry_table: x.model,
        bank: x.bank,
        aoa: x.aoa,
        ut: st.ut,
        mass: st.mass,
        gear: false,
        brakes: false,
        airbrakes: 0,
    };
    let (mut bank, mut bank_rate) = (x.bank, x.bank_rate);
    let mut max_ld = f64::NAN;
    let mut next_ld_ut = f64::NEG_INFINITY;
    let mut spacing = 15.0;
    let mut next_sample_ut = st.ut;
    let mut prev: Option<(f64, GeoPoint, f64, f64, f64, f64, f64, f64)> = None;
    let max_q = (*x.vehicle).maximum_dynamic_pressure;
    let max_g = (*x.vehicle).maximum_g_load;
    let t_end = st.ut + x.max_duration;
    let r = (*p).radius;
    while st.ut < t_end {
        let atm = atmosphere(st.position, st.velocity, &ctx);
        let geo = predictor_geo_point(st.position, p, st.ut);
        let up = unit(st.position);
        let east = unit(cross((*p).north_axis, up));
        let north = cross(up, east);
        let (ve, vn, vu) = (dot(atm.air_velocity, east), dot(atm.air_velocity, north), dot(atm.air_velocity, up));
        let ground = ve.hypot(vn);
        let course = ve.atan2(vn).to_degrees().rem_euclid(360.0);
        let fpa = vu.atan2(ground).to_degrees();
        let excess = atm.speed - x.handoff_mach * atm.speed_of_sound;
        let (range, bearing) = range_bearing(geo, x.target, r);
        let to_go = range - x.station_range.max(0.0);
        // The handoff is the first of: the handoff Mach inside the MM305 band, the
        // HAC station (to-go reaches zero), or closest approach to it. Ending at the
        // station makes an overshoot a surplus, so terminal energy stays monotone.
        let mach_cross = geo.altitude <= x.max_altitude && excess <= 0.0;
        let station = to_go <= 0.0 || prev.is_some_and(|q| to_go > q.7);
        if mach_cross || station {
            let (ut, point, speed, gs, crs, gamma) = match prev {
                Some((pu, pg, pex, ps, pgs, pc, pf, ptg)) if (mach_cross && pex > 0.0) || (to_go <= 0.0 && ptg > 0.0) => {
                    let k = if mach_cross && pex > 0.0 { pex / (pex - excess) } else { ptg / (ptg - to_go) };
                    let lerp = |a: f64, b: f64| a + k * (b - a);
                    let pt = GeoPoint {
                        latitude: lerp(pg.latitude, geo.latitude),
                        longitude: pg.longitude + k * terminal::norm_signed(geo.longitude - pg.longitude),
                        altitude: lerp(pg.altitude, geo.altitude),
                    };
                    (lerp(pu, st.ut), pt, lerp(ps, atm.speed), lerp(pgs, ground), pc + k * terminal::norm_signed(course - pc), lerp(pf, fpa))
                }
                _ => (st.ut, geo, atm.speed, ground, course, fpa),
            };
            out.terminal = terminal::evaluate(
                &Handoff { ut, point, airspeed: speed, ground_speed: gs, course: crs, flight_path_angle: gamma, mass: st.mass },
                &x.site,
                x.model as *const c_void,
                x.live_aero,
                x.min_altitude,
                x.max_altitude,
            );
            out.terminal.admissible &= mach_cross;
            out.converged = true;
            break;
        }
        if geo.altitude < x.site.altitude + 1000.0 {
            break;
        }
        if atm.q > max_q || atm.non_gravity / G0 > max_g {
            out.in_envelope = false;
        }
        if st.ut >= next_ld_ut {
            max_ld = max_lift_to_drag_aoa(x.model, atm.q, atm.mach, x.schedule.max_aoa);
            next_ld_ut = st.ut + 5.0;
        }
        let input = LawInput {
            ut: st.ut,
            mach: atm.mach,
            speed: atm.speed,
            ground_speed: ground,
            lift_accel: atm.lift_accel,
            range,
            bearing_error: terminal::norm_signed(bearing - course),
            station_range: x.station_range,
            handoff_speed: x.handoff_mach * atm.speed_of_sound,
            max_ld_aoa: max_ld,
            bank_limit: bank_limit(atm.speed, atm.q, atm.non_gravity / G0, x.vehicle) * x.bank_limit_scale,
            roll_rate: x.roll_rate,
            cross_uncertainty: x.cross_uncertainty,
            measured_bank: bank,
        };
        let cmd = law::step(&mut law, &x.schedule, sigma, &input);
        if out.first_aoa.is_nan() {
            out.first_aoa = cmd.aoa;
            out.first_nominal = cmd.nominal_aoa;
            out.first_energy_bank = law::plan_attitude(sigma, cmd.nominal_aoa, max_ld, input.bank_limit).1;
        }
        if st.ut >= next_sample_ut {
            if out.sample_count == MM304_SAMPLES {
                for i in 0..MM304_SAMPLES / 2 {
                    out.samples[i] = out.samples[2 * i];
                }
                out.sample_count = MM304_SAMPLES / 2;
                spacing *= 2.0;
            }
            out.samples[out.sample_count] = Mm304Sample {
                ut: st.ut,
                latitude: geo.latitude,
                longitude: geo.longitude,
                altitude: geo.altitude,
                speed: atm.speed,
                energy: energy_height(geo.altitude, x.site.altitude, atm.speed, x.surface_gravity),
                aoa: cmd.aoa,
                bank: cmd.bank,
                miss: law.miss,
            };
            out.sample_count += 1;
            next_sample_ut = st.ut + spacing;
        }
        let dt = if atm.mach > 6.0 { 2.0 } else { 1.0 };
        track_roll(&mut bank, &mut bank_rate, cmd.bank, x.roll_rate, x.roll_accel, dt);
        ctx.bank = bank;
        ctx.aoa = cmd.aoa;
        prev = Some((st.ut, geo, excess, atm.speed, ground, course, fpa, to_go));
        st = rk4_aero(st, dt, &mut ctx);
    }
    out.reversals = law.reversals - reversals0;
    out
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Bearing error stays continuous across the 0/360 wrap and an antimeridian crossing.
    #[test]
    fn bearing_error_is_continuous_across_wraps() {
        let r = 600_000.0;
        let at = |lon: f64| GeoPoint { latitude: 0.0, longitude: lon, altitude: 0.0 };
        let (d1, b1) = range_bearing(at(179.9), at(-179.9), r);
        let (d2, b2) = range_bearing(at(-179.9), at(179.9), r);
        assert!((d1 - d2).abs() < 1e-6 && d1 < 3000.0);
        assert!((b1 - 90.0).abs() < 1e-6 && (b2 - 270.0).abs() < 1e-6);
        let e = |bearing: f64, course: f64| terminal::norm_signed(bearing - course);
        assert!((e(1.0, 359.0) - 2.0).abs() < 1e-9);
        assert!((e(359.0, 1.0) + 2.0).abs() < 1e-9);
    }
}
