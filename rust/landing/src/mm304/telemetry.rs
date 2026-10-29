//! MM304 telemetry: the live guidance publishes its state each tick (never the
//! shadow or a worker copy), and the snapshot writer emits it as `guidanceState.mm304`.

use std::fmt::Write;
use std::sync::Mutex;

use super::state::*;

#[derive(Copy, Clone)]
pub struct Published {
    pub state: Mm304State,
    pub ut: f64,
    pub aoa_cmd: f64,
    pub nominal_aoa: f64,
    pub bank_cmd: f64,
    pub max_ld_aoa: f64,
    pub range: f64,
}

static LATEST: Mutex<Option<Published>> = Mutex::new(None);

pub fn publish(p: Published) {
    if let Ok(mut slot) = LATEST.lock() {
        *slot = Some(p);
    }
}

pub fn clear() {
    if let Ok(mut slot) = LATEST.lock() {
        *slot = None;
    }
}

pub fn decision_name(d: u32) -> &'static str {
    match d {
        MM304_DECISION_INITIAL => "initial",
        MM304_DECISION_KEPT => "kept",
        MM304_DECISION_REPLACED => "replaced",
        MM304_DECISION_INFEASIBLE_DEFICIT => "infeasible-deficit",
        MM304_DECISION_INFEASIBLE_SURPLUS => "infeasible-surplus",
        MM304_DECISION_UNCERTAIN => "uncertain",
        _ => "none",
    }
}

/// NUL-terminated `decision_name` for C formatting.
pub fn decision_cname(d: u32) -> *const core::ffi::c_char {
    match d {
        MM304_DECISION_INITIAL => c"initial".as_ptr(),
        MM304_DECISION_KEPT => c"kept".as_ptr(),
        MM304_DECISION_REPLACED => c"replaced".as_ptr(),
        MM304_DECISION_INFEASIBLE_DEFICIT => c"infeasible-deficit".as_ptr(),
        MM304_DECISION_INFEASIBLE_SURPLUS => c"infeasible-surplus".as_ptr(),
        MM304_DECISION_UNCERTAIN => c"uncertain".as_ptr(),
        _ => c"none".as_ptr(),
    }
}

pub fn limiting_name(l: u32) -> &'static str {
    match l {
        MM304_LIMIT_ENERGY_DEFICIT => "energy deficit at max L/D",
        MM304_LIMIT_ENERGY_SURPLUS => "energy surplus beyond MM305 band at max bank",
        MM304_LIMIT_NOT_CONVERGED => "forecast did not reach the MM305 boundary",
        MM304_LIMIT_LOAD_ENVELOPE => "dynamic pressure or g-load envelope",
        MM304_LIMIT_HANDOFF_ALTITUDE => "handoff below the MM305 altitude band",
        MM304_LIMIT_HAC_GEOMETRY => "MM305 HAC cannot score the handoff geometry",
        _ => "none",
    }
}

/// NUL-terminated short limiting-constraint code for C formatting.
pub fn limiting_cname(l: u32) -> *const core::ffi::c_char {
    match l {
        MM304_LIMIT_ENERGY_DEFICIT => c"deficit".as_ptr(),
        MM304_LIMIT_ENERGY_SURPLUS => c"surplus".as_ptr(),
        MM304_LIMIT_NOT_CONVERGED => c"not-converged".as_ptr(),
        MM304_LIMIT_LOAD_ENVELOPE => c"load-envelope".as_ptr(),
        MM304_LIMIT_HANDOFF_ALTITUDE => c"handoff-altitude".as_ptr(),
        MM304_LIMIT_HAC_GEOMETRY => c"hac-geometry".as_ptr(),
        _ => c"none".as_ptr(),
    }
}

fn num(out: &mut String, key: &str, v: f64) {
    if v.is_finite() {
        let _ = write!(out, "\"{key}\":{},", (v * 1e4).round() / 1e4);
    } else {
        let _ = write!(out, "\"{key}\":null,");
    }
}

fn close(out: &mut String) {
    if out.ends_with(',') {
        out.pop();
    }
}

pub fn json(p: &Published) -> String {
    let m = &p.state;
    let mut o = String::with_capacity(4096);
    o.push('{');
    num(&mut o, "ut", p.ut);
    o.push_str("\"command\":{");
    num(&mut o, "aoa", p.aoa_cmd);
    num(&mut o, "nominalAoa", p.nominal_aoa);
    num(&mut o, "maxLiftToDragAoa", p.max_ld_aoa);
    num(&mut o, "bank", p.bank_cmd);
    num(&mut o, "bankLimit", m.bank_limit);
    let _ = write!(o, "\"regime\":\"{}\",", if m.law.regime == MM304_REGIME_LATERAL { "lateral" } else { "energy" });
    let _ = write!(o, "\"reversals\":{},", m.law.reversals);
    num(&mut o, "lastReversalUt", m.law.last_reversal_ut);
    num(&mut o, "timeToGo", m.law.t_go);
    num(&mut o, "rangeToTarget", p.range);
    num(&mut o, "miss", m.law.miss);
    num(&mut o, "missHold", m.law.miss_hold);
    num(&mut o, "missReverse", m.law.miss_reverse);
    num(&mut o, "lateralCapacity", m.law.accel_capacity);
    num(&mut o, "lateralRequired", m.law.accel_required);
    close(&mut o);
    o.push_str("},\"plan\":");
    let pl = &m.plan;
    if pl.valid {
        o.push('{');
        let _ = write!(o, "\"sequence\":{},\"decision\":\"{}\",\"limiting\":\"{}\",", pl.sequence, decision_name(pl.decision), limiting_name(pl.limiting));
        let _ = write!(o, "\"converged\":{},\"inEnvelope\":{},\"forecasts\":{},\"reversalsPredicted\":{},", pl.converged, pl.in_envelope, pl.forecasts, pl.reversals_predicted);
        num(&mut o, "ut", pl.ut);
        num(&mut o, "sigma", pl.sigma);
        num(&mut o, "aoaBias", pl.aoa_bias);
        num(&mut o, "bank", pl.bank);
        num(&mut o, "targetEnergy", pl.target_energy);
        num(&mut o, "predictedEnergy", pl.predicted_energy);
        num(&mut o, "uncertainty", pl.uncertainty);
        num(&mut o, "cost", pl.cost);
        num(&mut o, "energyAtMaxRange", pl.energy_at_min);
        num(&mut o, "energyAtMaxBank", pl.energy_at_max);
        let t = &pl.terminal;
        let _ = write!(o, "\"terminal\":{{\"valid\":{},\"admissible\":{},", t.valid, t.admissible);
        num(&mut o, "ut", t.ut);
        num(&mut o, "along", t.along);
        num(&mut o, "cross", t.cross);
        num(&mut o, "headingError", t.heading_error);
        num(&mut o, "altitude", t.altitude);
        num(&mut o, "speed", t.speed);
        num(&mut o, "flightPathAngle", t.flight_path_angle);
        num(&mut o, "energyError", t.energy_error);
        num(&mut o, "band", t.band);
        close(&mut o);
        o.push_str("},\"trajectory\":[");
        for s in &pl.samples[..(pl.sample_count as usize).min(MM304_SAMPLES)] {
            let _ = write!(
                o,
                "[{:.1},{:.5},{:.5},{:.0},{:.1},{:.0},{:.2},{:.2},{:.0}],",
                s.ut, s.latitude, s.longitude, s.altitude, s.speed, s.energy, s.aoa, s.bank, s.miss
            );
        }
        close(&mut o);
        o.push_str("]}");
    } else {
        o.push_str("null");
    }
    let r = &m.residual;
    let _ = write!(o, ",\"residual\":{{\"count\":{},\"planSequence\":{},", r.count, r.plan_sequence);
    num(&mut o, "ut", r.ut);
    num(&mut o, "horizon", r.horizon);
    num(&mut o, "energy", r.energy);
    num(&mut o, "altitude", r.altitude);
    num(&mut o, "speed", r.speed);
    num(&mut o, "miss", r.miss);
    num(&mut o, "aoa", r.aoa);
    num(&mut o, "bank", r.bank);
    num(&mut o, "energyRateRms", r.energy_rate_rms);
    num(&mut o, "aoaRms", r.aoa_rms);
    num(&mut o, "bankRms", r.bank_rms);
    close(&mut o);
    o.push_str("},\"target\":{");
    num(&mut o, "latitude", m.target_latitude);
    num(&mut o, "longitude", m.target_longitude);
    num(&mut o, "stationRange", m.station_range);
    num(&mut o, "handoffSpeed", m.handoff_speed);
    close(&mut o);
    o.push_str("}}");
    o
}

/// The latest published MM304 block, or `None` outside MM304.
pub fn latest_json() -> Option<String> {
    LATEST.lock().ok().and_then(|s| s.as_ref().map(json))
}
