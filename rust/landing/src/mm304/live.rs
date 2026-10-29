//! The live MM304 tick: residuals of the active plan, then the law.

use super::law::{self, LawInput, LawOutput};
use super::predict::{energy_height, range_bearing};
use super::state::*;
use super::terminal::norm_signed;
use crate::prediction::predictor::GeoPoint;

#[derive(Copy, Clone, Debug)]
pub struct LiveInput {
    pub ut: f64,
    pub mach: f64,
    pub speed: f64,
    pub ground_speed: f64,
    pub altitude: f64,
    pub latitude: f64,
    pub longitude: f64,
    pub course: f64,
    pub site_altitude: f64,
    pub surface_gravity: f64,
    pub radius: f64,
    pub lift_accel: f64,
    pub target_latitude: f64,
    pub target_longitude: f64,
    pub station_range: f64,
    pub handoff_speed: f64,
    pub max_ld_aoa: f64,
    pub bank_limit: f64,
    pub roll_rate: f64,
    pub cross_uncertainty: f64,
    pub measured_bank: f64,
}

/// Copy the AoA schedule and the optional test staircase into the state.
pub fn set_schedule(m: &mut Mm304State, velocity: &[f64], alpha: &[f64], rate_limit: f64, min_aoa: f64, max_aoa: f64, staircase: Option<&str>) {
    let s = &mut m.schedule;
    let n = velocity.len().min(alpha.len()).min(MM304_SCHEDULE_POINTS);
    s.count = n as u32;
    s.velocity[..n].copy_from_slice(&velocity[..n]);
    s.alpha[..n].copy_from_slice(&alpha[..n]);
    s.stair_count = 0;
    if let Some(spec) = staircase {
        for (i, item) in spec.split(',').take(MM304_SCHEDULE_POINTS).enumerate() {
            let mut parts = item.split(':');
            match (parts.next().and_then(|a| a.trim().parse().ok()), parts.next().and_then(|b| b.trim().parse().ok())) {
                (Some(mach), Some(aoa)) => {
                    s.stair_mach[i] = mach;
                    s.stair_alpha[i] = aoa;
                    s.stair_count = i as u32 + 1;
                }
                _ => {
                    s.stair_count = 0;
                    break;
                }
            }
        }
    }
    s.rate_limit = rate_limit;
    s.min_aoa = min_aoa;
    s.max_aoa = max_aoa;
    s.valid = n > 0 || s.stair_count > 0;
}

fn interpolate(plan: &Mm304Plan, ut: f64) -> Option<Mm304Sample> {
    let n = (plan.sample_count as usize).min(MM304_SAMPLES);
    let s = &plan.samples[..n];
    if n < 2 || ut < s[0].ut || ut > s[n - 1].ut {
        return None;
    }
    let i = s.windows(2).position(|w| ut <= w[1].ut)?;
    let (a, b) = (s[i], s[i + 1]);
    let k = if b.ut > a.ut { (ut - a.ut) / (b.ut - a.ut) } else { 0.0 };
    let l = |x: f64, y: f64| x + k * (y - x);
    Some(Mm304Sample {
        ut,
        latitude: l(a.latitude, b.latitude),
        longitude: a.longitude + k * norm_signed(b.longitude - a.longitude),
        altitude: l(a.altitude, b.altitude),
        speed: l(a.speed, b.speed),
        energy: l(a.energy, b.energy),
        aoa: l(a.aoa, b.aoa),
        bank: l(a.bank, b.bank),
        miss: l(a.miss, b.miss),
    })
}

fn update_residual(m: &mut Mm304State, x: &LiveInput, cmd: &LawOutput) {
    if !m.reference.valid || x.ut - m.residual.ut < 2.0 {
        return;
    }
    let Some(p) = interpolate(&m.reference, x.ut) else { return };
    let horizon = x.ut - m.reference.ut;
    if horizon < 5.0 {
        return;
    }
    let r = &mut m.residual;
    r.ut = x.ut;
    r.plan_sequence = m.reference.sequence;
    r.horizon = horizon;
    r.energy = energy_height(x.altitude, x.site_altitude, x.speed, x.surface_gravity) - p.energy;
    r.altitude = x.altitude - p.altitude;
    r.speed = x.speed - p.speed;
    r.miss = m.law.miss - p.miss;
    r.aoa = cmd.aoa - p.aoa;
    r.bank = cmd.bank - p.bank;
    // Exponentially weighted mean squares, one weight per 2 s update.
    let w = if r.count == 0 { 1.0 } else { 0.2 };
    let rate = r.energy / horizon;
    r.energy_rate_rms = ((1.0 - w) * r.energy_rate_rms.powi(2) + w * rate * rate).sqrt();
    r.aoa_rms = ((1.0 - w) * r.aoa_rms.powi(2) + w * r.aoa * r.aoa).sqrt();
    r.bank_rms = ((1.0 - w) * r.bank_rms.powi(2) + w * r.bank * r.bank).sqrt();
    r.count += 1;
}

pub fn tick(m: &mut Mm304State, x: &LiveInput) -> LawOutput {
    m.target_valid = x.target_latitude.is_finite() && x.target_longitude.is_finite();
    m.target_latitude = x.target_latitude;
    m.target_longitude = x.target_longitude;
    m.station_range = x.station_range;
    m.handoff_speed = x.handoff_speed;
    m.bank_limit = x.bank_limit;
    m.cross_uncertainty = x.cross_uncertainty;
    let here = GeoPoint { latitude: x.latitude, longitude: x.longitude, altitude: x.altitude };
    let target = GeoPoint { latitude: x.target_latitude, longitude: x.target_longitude, altitude: 0.0 };
    let (range, bearing) = range_bearing(here, target, x.radius);
    let sigma = if m.plan.valid { m.plan.sigma } else { 0.0 };
    let input = LawInput {
        ut: x.ut,
        mach: x.mach,
        speed: x.speed,
        ground_speed: x.ground_speed,
        lift_accel: x.lift_accel,
        range,
        bearing_error: norm_signed(bearing - x.course),
        station_range: x.station_range,
        handoff_speed: x.handoff_speed,
        max_ld_aoa: x.max_ld_aoa,
        bank_limit: x.bank_limit,
        roll_rate: x.roll_rate,
        cross_uncertainty: x.cross_uncertainty,
        measured_bank: x.measured_bank,
    };
    let schedule = m.schedule;
    let cmd = law::step(&mut m.law, &schedule, sigma, &input);
    update_residual(m, x, &cmd);
    cmd
}

/// Adopt a worker plan into the live state; the law and residual state stay live.
pub fn accept(m: &mut Mm304State, result: &Mm304State) -> bool {
    if !result.plan.valid || result.plan.sequence == m.plan.sequence {
        return false;
    }
    m.plan = result.plan;
    if !m.reference.valid || m.plan.ut - m.reference.ut >= MM304_RESIDUAL_HORIZON_S {
        m.reference = m.plan;
    }
    true
}
