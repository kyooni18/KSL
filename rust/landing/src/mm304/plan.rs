//! Receding-horizon MM304 planner, run on the prediction worker each interval.
//!
//! Decision variable: the continuous energy control `sigma` (see `law.rs`), which
//! couples AoA (toward max L/D) and bank magnitude. Terminal energy at the MM305
//! boundary, scored by MM305's HAC, decreases monotonically with `sigma`. The
//! objective is asymmetric about the middle of MM305's acceptance window
//! (`[0, band]`): a deficit cannot be recovered downstream, a surplus can be shed.
//!
//! Hysteresis is physical: the active plan is re-forecast every interval and kept
//! while its terminal energy is inside the prediction uncertainty measured from
//! its own predicted-vs-actual residuals. Otherwise it is replaced by the solve.
//! If no `sigma` reaches the window, the planner flies the energy-preserving
//! extreme and reports the limiting constraint.

use core::ffi::c_void;

use super::predict::{self, Forecast, PredictInput};
use super::state::*;
use super::terminal::Site;
use crate::prediction::predictor::{
    AerodynamicEnvelope, GeoPoint, GuidanceSettings, LandingSite, PlanetModel, TerminalModel,
    TrajectoryCalibrationModel, VehicleProfile, VehicleState,
};

extern "C" {
    fn planet_surface_gravity(p: *const PlanetModel) -> f64;
    fn planet_atmospheric_speed_of_sound(p: *const PlanetModel, altitude: f64) -> f64;
}

/// Cost of a terminal energy error relative to the target. A deficit is scaled by
/// the deficit side of the window (target - 0), a surplus by the surplus side
/// (band - target), with the deficit weighted four times harder.
pub fn energy_cost(error: f64, target: f64, band: f64) -> f64 {
    if !error.is_finite() {
        return f64::INFINITY;
    }
    let side = (band - target).max(target).max(1.0);
    let d = (error - target) / side;
    if error < target { 4.0 * d * d } else { d * d }
}

fn adopt(plan: &mut Mm304Plan, f: &Forecast, sigma: f64) {
    plan.sigma = sigma;
    plan.converged = f.converged;
    plan.in_envelope = f.in_envelope;
    plan.terminal = f.terminal;
    plan.predicted_energy = f.energy();
    plan.reversals_predicted = f.reversals;
    plan.sample_count = f.sample_count as u32;
    plan.samples = f.samples;
    plan.aoa_bias = f.first_aoa - f.first_nominal;
    plan.bank = f.first_energy_bank;
}

fn limiting_of(f: &Forecast, min_altitude: f64) -> u32 {
    if !f.converged {
        MM304_LIMIT_NOT_CONVERGED
    } else if !f.terminal.valid {
        MM304_LIMIT_HAC_GEOMETRY
    } else if !f.in_envelope {
        MM304_LIMIT_LOAD_ENVELOPE
    } else if f.terminal.altitude < min_altitude {
        MM304_LIMIT_HANDOFF_ALTITUDE
    } else {
        MM304_LIMIT_NONE
    }
}

#[no_mangle]
pub unsafe extern "C" fn mm304_plan_update(
    m: *mut Mm304State,
    state: VehicleState,
    p: *const PlanetModel,
    env: *const AerodynamicEnvelope,
    cal: *const TrajectoryCalibrationModel,
    v: *const VehicleProfile,
    site: *const LandingSite,
    set: *const GuidanceSettings,
    model: *const TerminalModel,
    live_aero: *const c_void,
    measured_bank: f64,
    measured_bank_rate: f64,
    measured_aoa: f64,
    range_to_site: f64,
) -> bool {
    if m.is_null() || model.is_null() || !(*m).target_valid || !(*m).schedule.valid {
        return false;
    }
    let m = &mut *m;
    let s = &*set;
    let sound_floor = planet_atmospheric_speed_of_sound(p, s.mm305_max_altitude).max(1.0);
    let input = PredictInput {
        state,
        planet: p,
        envelope: env,
        calibration: cal,
        vehicle: v,
        model,
        live_aero,
        site: Site {
            latitude: (*site).latitude,
            longitude: (*site).longitude,
            altitude: (*site).altitude,
            runway_heading: (*site).runway_heading,
            radius: (*p).radius,
        },
        schedule: m.schedule,
        law: m.law,
        target: GeoPoint { latitude: m.target_latitude, longitude: m.target_longitude, altitude: 0.0 },
        station_range: m.station_range,
        handoff_mach: s.mm305_target_mach,
        min_altitude: s.mm305_min_altitude,
        max_altitude: s.mm305_max_altitude,
        bank_limit: m.bank_limit,
        roll_rate: s.entry_roll_rate,
        roll_accel: s.entry_roll_acceleration,
        cross_uncertainty: m.cross_uncertainty,
        bank: measured_bank,
        bank_rate: if measured_bank_rate.is_finite() { measured_bank_rate } else { 0.0 },
        aoa: measured_aoa,
        max_duration: 2.0 * range_to_site.max(0.0) / (s.mm305_target_mach * sound_floor).max(1.0) + 120.0,
        surface_gravity: planet_surface_gravity(p),
    };
    let previous = m.plan;
    let mut plan = previous;
    plan.forecasts = 0;
    let mut run = |sigma: f64, plan: &mut Mm304Plan| -> Forecast {
        plan.forecasts += 1;
        predict::forecast(&input, sigma)
    };

    // Re-forecast the active plan; keep it while inside the measured uncertainty.
    let active_sigma = if previous.valid { previous.sigma } else { 0.0 };
    let active = run(active_sigma, &mut plan);
    let band = if active.terminal.band.is_finite() { active.terminal.band.max(0.0) } else { previous.terminal.band.max(0.0) };
    let target = 0.5 * band;
    let t_go = (active.terminal.ut - state.ut).max(0.0);
    let uncertainty = if m.residual.count > 0 { m.residual.energy_rate_rms * t_go } else { f64::INFINITY };
    // Resolution floor: what one degree of bank does to terminal energy.
    let mut slope = f64::NAN;
    if previous.valid && previous.energy_at_min.is_finite() && previous.energy_at_max.is_finite() {
        slope = (previous.energy_at_min - previous.energy_at_max).abs() / 2.0;
    }
    let resolution = if slope.is_finite() && m.bank_limit > 0.0 { slope / m.bank_limit } else { 0.0 };
    let tolerance = if uncertainty.is_finite() { uncertainty.max(resolution) } else { resolution };
    let e_active = active.energy();
    plan.uncertainty = tolerance;
    plan.target_energy = target;
    if previous.valid
        && e_active.is_finite()
        && active.in_envelope
        && (e_active - target).abs() <= tolerance
        && e_active >= 0.0
        && e_active <= band
    {
        adopt(&mut plan, &active, active_sigma);
        plan.decision = MM304_DECISION_KEPT;
        plan.limiting = MM304_LIMIT_NONE;
    } else {
        // Bracket and solve e(sigma) = target (Illinois false position).
        let lo_f = run(-1.0, &mut plan);
        let hi_f = run(1.0, &mut plan);
        let (e_lo, e_hi) = (lo_f.energy(), hi_f.energy());
        plan.energy_at_min = e_lo;
        plan.energy_at_max = e_hi;
        if !e_lo.is_finite() && !e_hi.is_finite() {
            // Nothing can be scored; keep flying the active plan, flagged uncertain.
            adopt(&mut plan, &active, active_sigma);
            plan.decision = MM304_DECISION_UNCERTAIN;
            plan.limiting = limiting_of(&lo_f, s.mm305_min_altitude);
        } else if e_lo.is_finite() && e_lo < target {
            adopt(&mut plan, &lo_f, -1.0);
            plan.decision = MM304_DECISION_INFEASIBLE_DEFICIT;
            plan.limiting = MM304_LIMIT_ENERGY_DEFICIT;
        } else if e_hi.is_finite() && e_hi > target {
            adopt(&mut plan, &hi_f, 1.0);
            let shed = e_hi <= band;
            plan.decision = if shed { MM304_DECISION_REPLACED } else { MM304_DECISION_INFEASIBLE_SURPLUS };
            plan.limiting = if shed { MM304_LIMIT_NONE } else { MM304_LIMIT_ENERGY_SURPLUS };
        } else {
            let (mut a, mut fa) = (-1.0, e_lo - target);
            let (mut b, mut fb) = if e_hi.is_finite() { (1.0, e_hi - target) } else { (active_sigma, e_active - target) };
            let mut best = (f64::INFINITY, active_sigma, None::<Forecast>);
            let mut side = 0;
            for _ in 0..7 {
                if !(fa.is_finite() && fb.is_finite()) || fa == fb {
                    break;
                }
                let c = (b - (fb * (b - a) / (fb - fa))).clamp(-1.0, 1.0);
                let f = run(c, &mut plan);
                let fc = f.energy() - target;
                if !fc.is_finite() {
                    break;
                }
                let cost = energy_cost(f.energy(), target, band);
                let done = fc.abs() <= tolerance.max(0.02 * band.max(1.0));
                if cost < best.0 {
                    best = (cost, c, Some(f));
                }
                if done {
                    break;
                }
                if fc * fb > 0.0 {
                    b = c;
                    fb = fc;
                    if side == 1 {
                        fa *= 0.5;
                    }
                    side = 1;
                } else {
                    a = c;
                    fa = fc;
                    if side == -1 {
                        fb *= 0.5;
                    }
                    side = -1;
                }
            }
            match best.2 {
                Some(f) => {
                    adopt(&mut plan, &f, best.1);
                    plan.decision = if previous.valid { MM304_DECISION_REPLACED } else { MM304_DECISION_INITIAL };
                    plan.limiting = limiting_of(&f, s.mm305_min_altitude);
                }
                None => {
                    adopt(&mut plan, &active, active_sigma);
                    plan.decision = MM304_DECISION_UNCERTAIN;
                    plan.limiting = limiting_of(&active, s.mm305_min_altitude);
                }
            }
        }
    }
    plan.cost = energy_cost(plan.predicted_energy, target, band);
    plan.ut = state.ut;
    plan.valid = true;
    plan.sequence = previous.sequence.wrapping_add(1);
    m.plan = plan;
    true
}

#[cfg(test)]
mod tests {
    use super::energy_cost;

    #[test]
    fn deficit_costs_more_than_equal_surplus() {
        let (target, band) = (1500.0, 3000.0);
        assert!(energy_cost(target - 500.0, target, band) > energy_cost(target + 500.0, target, band));
        assert_eq!(energy_cost(target, target, band), 0.0);
        assert!(energy_cost(f64::NAN, target, band).is_infinite());
    }
}
