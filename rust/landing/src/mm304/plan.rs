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

fn adopt(plan: &mut Mm304Plan, f: &Forecast, sigma: f64, route: u32) {
    plan.sigma = sigma;
    plan.route = route;
    plan.converged = f.converged;
    plan.in_envelope = f.in_envelope;
    plan.terminal = f.terminal;
    plan.predicted_energy = f.energy(route);
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
        bank_limit_scale: 1.0,
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
    let mut route = if previous.valid { previous.route } else { MM304_ROUTE_SPIRAL };
    let route_band = |f: &Forecast, route: u32| {
        let b = f.band(route);
        if b.is_finite() { b.max(0.0) } else { previous.terminal.route_energy(route).1.max(0.0) }
    };
    let mut band = route_band(&active, route);
    let mut target = 0.5 * band;
    let t_go = (active.terminal.ut - state.ut).max(0.0);
    let uncertainty = if m.residual.count > 0 { m.residual.energy_rate_rms * t_go } else { f64::INFINITY };
    // Resolution floor: what one degree of bank does to terminal energy.
    let mut slope = f64::NAN;
    if previous.valid && previous.energy_at_min.is_finite() && previous.energy_at_max.is_finite() {
        slope = (previous.energy_at_min - previous.energy_at_max).abs() / 2.0;
    }
    let resolution = if slope.is_finite() { slope / predict::MM304_PLANNED_MAX_BANK_DEG } else { 0.0 };
    let tolerance = if uncertainty.is_finite() { uncertainty.max(resolution) } else { resolution };
    let e_active = active.energy(route);
    plan.uncertainty = tolerance;
    plan.target_energy = target;
    // On the short route, test each cycle whether maximum range now reaches the
    // spiral's target energy; if so the spiral becomes the route again.
    let mut spiral_probe: Option<Forecast> = None;
    if previous.valid && route == MM304_ROUTE_SHORT {
        let f = run(-1.0, &mut plan);
        plan.spiral_at_max_range = f.energy(MM304_ROUTE_SPIRAL);
        plan.spiral_turn_at_max_range = f.terminal.spiral_turn;
        let spiral_band = route_band(&f, MM304_ROUTE_SPIRAL);
        if f.energy(MM304_ROUTE_SPIRAL) >= 0.5 * spiral_band {
            spiral_probe = Some(f);
        }
    }
    if previous.valid
        && spiral_probe.is_none()
        && e_active.is_finite()
        && active.in_envelope
        && (e_active - target).abs() <= tolerance
        && e_active >= 0.0
        && e_active <= band
    {
        adopt(&mut plan, &active, active_sigma, route);
        plan.decision = MM304_DECISION_KEPT;
        plan.limiting = MM304_LIMIT_NONE;
    } else {
        // Bracket and solve e(sigma) = target (Illinois false position). The full
        // spiral is the route whenever maximum range reaches its target; otherwise
        // MM305's least-energy HAC route, so an unreachable spiral is never chased
        // past the site.
        let lo_f = match spiral_probe {
            Some(f) => f,
            None => run(-1.0, &mut plan),
        };
        let hi_f = run(1.0, &mut plan);
        plan.spiral_at_max_range = lo_f.energy(MM304_ROUTE_SPIRAL);
        plan.spiral_turn_at_max_range = lo_f.terminal.spiral_turn;
        // Joining the spiral needs maximum range to reach its target (mid-window);
        // a plan already on it keeps it while maximum range stays inside MM305's
        // window, so the route does not flip at the threshold.
        let join = if route == MM304_ROUTE_SPIRAL && previous.valid { 0.0 } else { 0.5 };
        route = if lo_f.energy(MM304_ROUTE_SPIRAL) >= join * route_band(&lo_f, MM304_ROUTE_SPIRAL) {
            MM304_ROUTE_SPIRAL
        } else {
            MM304_ROUTE_SHORT
        };
        band = route_band(&lo_f, route);
        target = 0.5 * band;
        plan.target_energy = target;
        let (e_lo, e_hi) = (lo_f.energy(route), hi_f.energy(route));
        plan.energy_at_min = e_lo;
        plan.energy_at_max = e_hi;
        if !e_lo.is_finite() && !e_hi.is_finite() {
            // Nothing can be scored; keep flying the active plan, flagged uncertain.
            adopt(&mut plan, &active, active_sigma, route);
            plan.decision = MM304_DECISION_UNCERTAIN;
            plan.limiting = limiting_of(&lo_f, s.mm305_min_altitude);
        } else if e_lo.is_finite() && e_lo < target {
            adopt(&mut plan, &lo_f, -1.0, route);
            plan.decision = MM304_DECISION_INFEASIBLE_DEFICIT;
            plan.limiting = MM304_LIMIT_ENERGY_DEFICIT;
        } else if e_hi.is_finite() && e_hi > target {
            adopt(&mut plan, &hi_f, 1.0, route);
            let shed = e_hi <= band;
            plan.decision = if shed { MM304_DECISION_REPLACED } else { MM304_DECISION_INFEASIBLE_SURPLUS };
            plan.limiting = if shed { MM304_LIMIT_NONE } else { MM304_LIMIT_ENERGY_SURPLUS };
        } else {
            let (mut a, mut fa) = (-1.0, e_lo - target);
            let (mut b, mut fb) = if e_hi.is_finite() { (1.0, e_hi - target) } else { (active_sigma, active.energy(route) - target) };
            let mut best = (f64::INFINITY, active_sigma, None::<Forecast>);
            let mut side = 0;
            for _ in 0..8 {
                if !(fa.is_finite() && fb.is_finite()) || fa == fb {
                    break;
                }
                let mut c = (b - (fb * (b - a) / (fb - fa))).clamp(-1.0, 1.0);
                let mut f = run(c, &mut plan);
                if !f.energy(route).is_finite() {
                    // An unscorable forecast inside the bracket: fall back to bisection.
                    c = 0.5 * (a + b);
                    f = run(c, &mut plan);
                }
                let fc = f.energy(route) - target;
                if !fc.is_finite() {
                    break;
                }
                let cost = energy_cost(f.energy(route), target, band);
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
                    adopt(&mut plan, &f, best.1, route);
                    plan.decision = if previous.valid { MM304_DECISION_REPLACED } else { MM304_DECISION_INITIAL };
                    plan.limiting = limiting_of(&f, s.mm305_min_altitude);
                }
                None => {
                    adopt(&mut plan, &active, active_sigma, route);
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
