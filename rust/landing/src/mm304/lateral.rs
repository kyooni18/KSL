//! Lateral steering: bank sign and the bank floor geometry needs.
//!
//! The target's lateral miss off the projected track is `y = D sin(e)` (positive
//! right). Over the time to go `T` a rightward lift component `a` removes
//! `∫(T-t) a dt`. Holding the current sign `s` with capacity `A = L sin(bank)`
//! ends at `y - ½ s A T²`; reversing now (bank swings through zero over `Tr`)
//! ends at `y + ½ s A T² - s A (T Tr - Tr²/3)`. The bank is reversed at the first
//! moment that even reversing leaves the target behind the turn (`s·y_rev <= 0`),
//! so one reversal nulls the miss, and only when holding would miss by more than
//! the measured cross-range uncertainty. If the miss needs more lateral
//! acceleration than the energy bank gives, the bank is raised to the bank that
//! nulls it at constant acceleration (`2y/T²`).

#[derive(Copy, Clone, Debug)]
pub struct LateralInput {
    pub miss: f64,
    pub t_go: f64,
    pub lift_accel: f64,
    pub energy_bank: f64,
    pub bank_limit: f64,
    pub roll_rate: f64,
    pub uncertainty: f64,
}

#[derive(Copy, Clone, Debug)]
pub struct LateralOutput {
    pub sign: f64,
    pub bank: f64,
    pub reverse: bool,
    pub lateral_regime: bool,
    pub miss_hold: f64,
    pub miss_reverse: f64,
    pub capacity: f64,
    pub required: f64,
}

pub fn steer(sign: f64, x: &LateralInput) -> LateralOutput {
    let s = if sign < 0.0 { -1.0 } else { 1.0 };
    let t = x.t_go.max(1.0);
    let lift = x.lift_accel.max(0.0);
    let limit = x.bank_limit.clamp(0.0, 89.0);
    let energy_bank = x.energy_bank.clamp(0.0, limit);
    let capacity = lift * energy_bank.to_radians().sin();
    let required = 2.0 * x.miss / (t * t);
    // A required acceleration known no better than the miss itself.
    let required_noise = 2.0 * x.uncertainty.max(0.0) / (t * t);
    let mut out = LateralOutput {
        sign: s,
        bank: energy_bank,
        reverse: false,
        lateral_regime: false,
        miss_hold: x.miss - 0.5 * s * capacity * t * t,
        miss_reverse: x.miss - 0.5 * s * capacity * t * t,
        capacity,
        required,
    };
    if lift > 0.0 && required.abs() - capacity > required_noise {
        let need = (required.abs() / lift).min(limit.to_radians().sin());
        out.bank = need.asin().to_degrees().max(energy_bank);
        out.lateral_regime = true;
        let want = if required < 0.0 { -1.0 } else { 1.0 };
        out.reverse = want != s;
        out.sign = want;
        return out;
    }
    let roll_time = if x.roll_rate > 0.0 { 2.0 * energy_bank / x.roll_rate } else { f64::INFINITY };
    if capacity > 0.0 && roll_time < t {
        out.miss_reverse = x.miss + 0.5 * s * capacity * t * t
            - s * capacity * (t * roll_time - roll_time * roll_time / 3.0);
        if s * out.miss_reverse <= 0.0 && out.miss_hold.abs() > x.uncertainty.max(0.0) {
            out.reverse = true;
            out.sign = -s;
        }
    }
    out
}

#[cfg(test)]
mod tests {
    use super::*;

    fn input(miss: f64, t_go: f64) -> LateralInput {
        LateralInput {
            miss,
            t_go,
            lift_accel: 6.0,
            energy_bank: 45.0,
            bank_limit: 70.0,
            roll_rate: 5.0,
            uncertainty: 2000.0,
        }
    }

    /// Integrate the lateral miss under the steering law: reversals must stay few
    /// and the final miss must be inside the uncertainty.
    #[test]
    fn reversal_dynamics_null_the_miss_with_few_reversals() {
        let dt = 1.0;
        let mut t_go = 900.0;
        // Target offset right of the vehicle and its rate; the projected miss is x + ẋT.
        let mut x = 60_000.0;
        let mut rate = 0.0;
        let mut sign = 1.0;
        let mut bank: f64 = 0.0;
        let mut reversals = 0;
        while t_go > 1.0 {
            let o = steer(sign, &input(x + rate * t_go, t_go));
            if o.reverse {
                reversals += 1;
            }
            sign = o.sign;
            let target = sign * o.bank;
            bank += (target - bank).clamp(-5.0 * dt, 5.0 * dt);
            rate -= 6.0 * bank.to_radians().sin() * dt;
            x += rate * dt;
            t_go -= dt;
        }
        assert!(reversals <= 4, "reversals {reversals}");
        assert!(x.abs() < 6000.0, "final miss {x}");
    }

    #[test]
    fn no_reversal_inside_uncertainty() {
        let o = steer(1.0, &input(500.0, 20.0));
        assert!(!o.reverse);
    }
}
