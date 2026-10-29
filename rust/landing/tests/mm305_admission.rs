//! MM305 admission: a physically admissible inlet is ready, each violated
//! constraint raises its own veto bit, and the cheap kinematic prefilter never
//! vetoes what only the full energy projection can judge.

use landing::prediction::decision_envelope::{
    GuidanceMachine, GuidanceSettings, LandingConfiguration, PlanetModel, TaemInterfaceCapture, Telemetry, PHASE_ENTRY_ENERGY,
};

#[allow(clashing_extern_declarations)]
extern "C" {
    fn landing_configuration_default() -> LandingConfiguration;
    fn landing_configuration_normalize(c: *mut LandingConfiguration);
    fn planet_atmospheric_speed_of_sound(p: *const PlanetModel, altitude: f64) -> f64;
    fn entry_taem_range_target(p: *const PlanetModel, s: *const GuidanceSettings) -> f64;
    fn entry_mm305_admission_envelope(
        g: *const GuidanceMachine,
        t: *const Telemetry,
        course: f64,
        p: *const PlanetModel,
        cfg: *const LandingConfiguration,
    ) -> TaemInterfaceCapture;
    fn entry_mm305_admission_prefilter_veto(t: *const Telemetry, p: *const PlanetModel, cfg: *const LandingConfiguration) -> u32;
}

const VETO_SPEED: u32 = 1;
const VETO_RANGE: u32 = 2;
const VETO_ALTITUDE: u32 = 8;
const VETO_ENERGY: u32 = 16;
const VETO_DESCENT: u32 = 32;
const VETO_SAFETY: u32 = 64;

fn kerbin() -> Box<PlanetModel> {
    // SAFETY: plain C data; all-zero is valid.
    let mut p: Box<PlanetModel> = unsafe { Box::new_zeroed().assume_init() };
    p.radius = 600000.0;
    p.gravitational_parameter = 3531600000000.0;
    p.rotational_speed = 0.000291570900559802;
    p.atmosphere_depth = 70000.0;
    p.surface_density = 1.225;
    p.atmosphere_adiabatic_index = 1.4;
    let rows = [(0.0, 101325.0, 1.225), (15000.0, 12000.0, 0.18), (25000.0, 3600.0, 0.05), (50000.0, 120.0, 0.002), (70000.0, 1.0, 1e-6)];
    p.atmosphere_sample_count = rows.len();
    for (i, (alt, pressure, density)) in rows.into_iter().enumerate() {
        p.atmosphere_altitude[i] = alt;
        p.atmosphere_pressure[i] = pressure;
        p.atmosphere_density[i] = density;
    }
    p
}

fn at_mach(t: &mut Telemetry, p: &PlanetModel, mach: f64) {
    t.mach = mach;
    t.true_air_speed = unsafe { planet_atmospheric_speed_of_sound(p, t.mean_altitude) } * mach;
    t.horizontal_speed = t.true_air_speed * 8f64.to_radians().cos();
    t.vertical_speed = -t.true_air_speed * 8f64.to_radians().sin();
}

fn admissible(p: &PlanetModel, cfg: &LandingConfiguration) -> Telemetry {
    // SAFETY: plain C data; all-zero is valid.
    let mut t: Telemetry = unsafe { core::mem::zeroed() };
    let g = &cfg.guidance;
    t.latitude = cfg.site.latitude;
    t.mean_altitude = 0.5 * (g.mm305_min_altitude + g.mm305_max_altitude);
    t.radar_altitude = t.mean_altitude;
    at_mach(&mut t, p, g.mm305_target_mach);
    t.flight_path_angle = -8.0;
    t.runway_along_track = -30000.0;
    t.runway_cross_track = 8000.0;
    t.dynamic_pressure = 7000f64.min(0.35 * cfg.vehicle.maximum_dynamic_pressure);
    t.g_force = 1.2f64.min(0.35 * cfg.vehicle.maximum_g_load);
    t.mass = 40000.0;
    t.angle_of_attack = 18.0;
    t.lift_force = t.mass * 8.0;
    t.drag_force = t.mass;
    t.bank_effectiveness = 1.0;
    t.estimated_ballistic_coefficient = cfg.vehicle.estimated_ballistic_coefficient;
    t.aerodynamic_confidence = 0.9;
    let r = &mut t.attitude_response;
    r.roll_valid = true;
    r.pitch_valid = true;
    r.maximum_roll_rate_deg_s = 18.0;
    r.maximum_roll_accel_deg_s2 = 12.0;
    r.maximum_pitch_rate_deg_s = 8.0;
    r.maximum_pitch_accel_deg_s2 = 5.0;
    t.stall_fraction_is_measured = true;
    t
}

#[test]
fn admission_vetoes_each_violated_constraint() {
    let mut cfg = unsafe { landing_configuration_default() };
    unsafe { landing_configuration_normalize(&mut cfg) };
    cfg.site.latitude = -0.0486111111;
    cfg.site.longitude = -74.7283333333;
    cfg.site.altitude = 70.0;
    cfg.site.runway_heading = 90.0;
    let p = kerbin();
    let mut g: Box<GuidanceMachine> = unsafe { Box::new_zeroed().assume_init() };
    g.phase = PHASE_ENTRY_ENERGY;
    g.automation_engaged = true;
    let envelope = |t: &Telemetry| unsafe { entry_mm305_admission_envelope(&*g, t, 90.0, &*p, &cfg) };
    let prefilter = |t: &Telemetry| unsafe { entry_mm305_admission_prefilter_veto(t, &*p, &cfg) };

    let nominal = admissible(&p, &cfg);
    let ready = envelope(&nominal);
    assert!(ready.valid && ready.ready && ready.veto == 0);
    assert!(ready.path_length_m > 0.0 && ready.energy_margin > 0.0);
    assert_eq!(prefilter(&nominal), 0, "the prefilter must not skip the energy proof of an admissible inlet");

    let mut cases: Vec<(&str, Telemetry, u32)> = Vec::new();
    let mut low = nominal;
    low.mean_altitude = cfg.guidance.mm305_min_altitude - 1.0;
    cases.push(("altitude", low, VETO_ALTITUDE));
    let mut slow = nominal;
    at_mach(&mut slow, &p, cfg.guidance.mm305_target_mach - cfg.guidance.mm305_mach_half_width - 0.01);
    cases.push(("speed", slow, VETO_SPEED));
    let mut far = nominal;
    far.runway_along_track = -(unsafe { entry_taem_range_target(&*p, &cfg.guidance) } + 1000.0);
    far.runway_cross_track = 0.0;
    cases.push(("range", far, VETO_RANGE));
    let mut climbing = nominal;
    climbing.vertical_speed = climbing.vertical_speed.abs();
    cases.push(("descent", climbing, VETO_DESCENT));
    let mut overloaded = nominal;
    overloaded.dynamic_pressure = cfg.vehicle.maximum_dynamic_pressure + 1.0;
    cases.push(("safety", overloaded, VETO_SAFETY));
    for (name, t, bit) in cases {
        let c = envelope(&t);
        assert!(c.valid && c.veto & bit != 0 && !c.ready, "{name} veto");
        assert!(prefilter(&t) & bit != 0, "{name} prefilter");
    }

    // An energy shortfall is not kinematic: only the full projection may veto it.
    let mut draggy = nominal;
    draggy.runway_along_track = -85000.0;
    draggy.runway_cross_track = 0.0;
    draggy.estimated_ballistic_coefficient = 30.0;
    let energy = envelope(&draggy);
    assert!(energy.valid && energy.veto & VETO_ENERGY != 0 && !energy.ready);
    assert!(energy.energy_margin < ready.energy_margin);
    assert_eq!(prefilter(&draggy), 0);
}
