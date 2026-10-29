//! The MM304 guidance law. One function, run every live tick and every
//! forecast step, so a prediction flies exactly what guidance will command.
//!
//! The plan is one continuous energy-control value `sigma`:
//! - `sigma` in [0, 1]: the AoA schedule with bank magnitude `sigma * bank_limit`
//!   (more bank, steeper descent, less range).
//! - `sigma` in [-1, 0]: wings level with AoA moved from the schedule toward
//!   max L/D by `-sigma` (more range).
//! Both branches meet at the schedule with zero energy bank, so terminal energy
//! is continuous and monotone in `sigma` and there is no 0/max bank switch.

use super::lateral::{self, LateralInput};
use super::state::*;

#[derive(Copy, Clone, Debug)]
pub struct LawInput {
    pub ut: f64,
    pub mach: f64,
    pub speed: f64,
    pub ground_speed: f64,
    pub lift_accel: f64,
    /// Ground range and bearing error (bearing to target minus course) to the
    /// final-alignment aimpoint.
    pub range: f64,
    pub bearing_error: f64,
    pub station_range: f64,
    pub handoff_speed: f64,
    pub max_ld_aoa: f64,
    pub bank_limit: f64,
    pub roll_rate: f64,
    pub cross_uncertainty: f64,
    pub measured_bank: f64,
}

#[derive(Copy, Clone, Debug)]
pub struct LawOutput {
    pub aoa: f64,
    pub bank: f64,
    pub nominal_aoa: f64,
    pub reversal: bool,
}

pub fn schedule_aoa(s: &Mm304Schedule, mach: f64, speed: f64) -> f64 {
    let stairs = (s.stair_count as usize).min(MM304_SCHEDULE_POINTS);
    if stairs > 0 && mach.is_finite() {
        for i in 0..stairs {
            if mach >= s.stair_mach[i] {
                return s.stair_alpha[i];
            }
        }
        return s.stair_alpha[stairs - 1];
    }
    let n = (s.count as usize).min(MM304_SCHEDULE_POINTS);
    if n == 0 || !speed.is_finite() {
        return f64::NAN;
    }
    if speed <= s.velocity[0] {
        return s.alpha[0];
    }
    for i in 0..n - 1 {
        if speed <= s.velocity[i + 1] {
            let u = (speed - s.velocity[i]) / (s.velocity[i + 1] - s.velocity[i]);
            return s.alpha[i] + (s.alpha[i + 1] - s.alpha[i]) * u;
        }
    }
    s.alpha[n - 1]
}

/// AoA target and energy bank magnitude for a plan value.
pub fn plan_attitude(sigma: f64, nominal: f64, max_ld: f64, bank_limit: f64) -> (f64, f64) {
    let sigma = if sigma.is_finite() { sigma.clamp(-1.0, 1.0) } else { 0.0 };
    let ld = if max_ld.is_finite() { max_ld.min(nominal) } else { nominal };
    let aoa = nominal + (-sigma).max(0.0) * (ld - nominal);
    (aoa, sigma.max(0.0) * bank_limit.max(0.0))
}

pub fn time_to_go(x: &LawInput) -> f64 {
    let mean = 0.5 * (x.ground_speed.max(0.0) + x.handoff_speed.max(0.0));
    (x.range - x.station_range.max(0.0)).max(0.0) / mean.max(1.0)
}

pub fn step(law: &mut Mm304Law, schedule: &Mm304Schedule, sigma: f64, x: &LawInput) -> LawOutput {
    let nominal = schedule_aoa(schedule, x.mach, x.speed);
    let (target, energy_bank) = plan_attitude(sigma, nominal, x.max_ld_aoa, x.bank_limit);
    let mut aoa = target.clamp(schedule.min_aoa, schedule.max_aoa.max(schedule.min_aoa));
    let dt = if law.initialized { (x.ut - law.last_ut).max(0.0) } else { 0.0 };
    if law.initialized && law.aoa_cmd.is_finite() && schedule.rate_limit > 0.0 {
        let step = schedule.rate_limit * dt;
        aoa = aoa.clamp(law.aoa_cmd - step, law.aoa_cmd + step);
    }
    let t_go = time_to_go(x);
    let miss = x.range * x.bearing_error.to_radians().sin();
    if !law.initialized {
        law.bank_sign = if x.measured_bank.abs() > 1.0 {
            x.measured_bank.signum()
        } else if miss < 0.0 {
            -1.0
        } else {
            1.0
        };
        law.last_reversal_ut = f64::NAN;
    }
    let steer = lateral::steer(
        law.bank_sign,
        &LateralInput {
            miss,
            t_go,
            lift_accel: x.lift_accel,
            energy_bank,
            bank_limit: x.bank_limit,
            roll_rate: x.roll_rate,
            uncertainty: x.cross_uncertainty,
        },
    );
    if steer.reverse {
        law.reversals += 1;
        law.last_reversal_ut = x.ut;
    }
    law.bank_sign = steer.sign;
    law.regime = if steer.lateral_regime { MM304_REGIME_LATERAL } else { MM304_REGIME_ENERGY };
    law.t_go = t_go;
    law.miss = miss;
    law.miss_hold = steer.miss_hold;
    law.miss_reverse = steer.miss_reverse;
    law.accel_capacity = steer.capacity;
    law.accel_required = steer.required;
    law.aoa_cmd = aoa;
    law.bank_cmd = steer.sign * steer.bank;
    law.last_ut = x.ut;
    law.initialized = true;
    LawOutput { aoa, bank: law.bank_cmd, nominal_aoa: nominal, reversal: steer.reverse }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn plan_attitude_is_continuous_through_the_schedule() {
        let (a0, b0) = plan_attitude(0.0, 40.0, 22.0, 70.0);
        let (am, bm) = plan_attitude(-1e-9, 40.0, 22.0, 70.0);
        let (ap, bp) = plan_attitude(1e-9, 40.0, 22.0, 70.0);
        assert!((a0 - am).abs() < 1e-6 && (a0 - ap).abs() < 1e-6);
        assert!(b0 == 0.0 && bm == 0.0 && bp < 1e-6);
        assert_eq!(plan_attitude(-1.0, 40.0, 22.0, 70.0), (22.0, 0.0));
        assert_eq!(plan_attitude(1.0, 40.0, 22.0, 70.0), (40.0, 70.0));
    }
}
