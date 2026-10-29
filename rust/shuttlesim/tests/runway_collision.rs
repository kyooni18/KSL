//! Contact-model regression: gear spring-damper landing, mains first, released
//! pitch settling to the measured stance, runway-edge terrain continuity,
//! departure detection, and stopping.

use shuttlesim::ffi::*;
use shuttlesim::scenario::Scenario;
use shuttlesim::sim::Simulation;

struct Plane {
    origin: Vec3,
    forward: Vec3,
    right: Vec3,
    normal: Vec3,
}

fn plane(sim: &Simulation) -> Plane {
    let r = sim.runway;
    let w = &sim.world;
    let origin = unsafe { world_lla_to_inertial(w, r.lat_rad, r.lon_rad, r.elevation_m, sim.state.ut) };
    let lf = unsafe { world_local_frame_i(w, origin, sim.state.ut) };
    let mut forward = normalized(add(scale(lf.north, r.heading_rad.cos()), scale(lf.east, r.heading_rad.sin())));
    let mut normal = lf.up;
    if sim.scenario.runway_slope_deg != 0.0 {
        let slope = deg2rad(sim.scenario.runway_slope_deg);
        let f = forward;
        forward = add(scale(f, slope.cos()), scale(lf.up, slope.sin()));
        normal = sub(scale(lf.up, slope.cos()), scale(f, slope.sin()));
    }
    Plane { origin, forward, right: normalized(cross(normal, forward)), normal }
}

fn pitch_deg(sim: &Simulation) -> f64 {
    let fwd = rotate(sim.state.body_q_i, v3(1.0, 0.0, 0.0));
    rad2deg(dot(fwd, plane(sim).normal).asin())
}

fn place(sim: &mut Simulation, along: f64, cross_m: f64, height: f64, speed: f64, sink: f64, aoa_deg: f64) {
    let p = plane(sim);
    let s = &mut sim.state;
    s.position_i_m = add(add(add(p.origin, scale(p.forward, along)), scale(p.right, cross_m)), scale(p.normal, height));
    let air = add(scale(p.forward, speed), scale(p.normal, -sink));
    s.velocity_i_mps = add(air, unsafe { world_atmosphere_velocity_i(&sim.world, s.position_i_m) });
    let a = &mut s.attitude;
    a.aoa_rad = deg2rad(aoa_deg);
    a.cmd_aoa_rad = a.aoa_rad;
    a.requested_aoa_rad = a.aoa_rad;
    s.body_q_i = unsafe { attitude_body_quat(s.position_i_m, air, a.aoa_rad, 0.0) };
}

fn blank() -> Scenario {
    // SAFETY: the scenario is plain numeric data; all-zero is valid.
    unsafe { core::mem::zeroed() }
}

fn seed() -> Scenario {
    let mut sc = blank();
    sc.name[..20].copy_from_slice(b"collision-regression");
    sc.latitude_deg = -0.0486;
    sc.longitude_deg = -74.7240;
    sc.altitude_m = 77.0;
    sc.heading_deg = 90.0;
    sc.has_surface_flight_state = true;
    sc.surface_speed_mps = 70.0;
    sc.flight_path_angle_deg = -4.0;
    sc.mass_kg = 40000.0;
    sc.initial_aoa_deg = 8.0;
    sc
}

fn vertical_m(sim: &Simulation) -> f64 {
    let json = sim.telemetry_json(1.0);
    let at = json.find("\"vertical_m\":").unwrap() + 13;
    json[at..].split(',').next().unwrap().parse().unwrap()
}

fn run_until(sim: &mut Simulation, ticks: usize, done: impl Fn(&Simulation) -> bool) {
    for _ in 0..ticks {
        if done(sim) {
            break;
        }
        sim.step(0.02);
    }
}

#[test]
fn authored_initial_state_is_preserved() {
    let mut sc = blank();
    sc.has_cartesian_state = true;
    sc.mass_kg = 40000.0;
    sc.position_i_m = v3(605000.0, 12.0, -23.0);
    sc.velocity_i_mps = v3(-10.0, 210.0, 70.0);
    let sim = Simulation::new(&sc);
    assert_eq!(norm(sub(sim.state.position_i_m, sc.position_i_m)), 0.0);
    assert_eq!(norm(sub(sim.state.velocity_i_mps, sc.velocity_i_mps)), 0.0);
    let mut sc = blank();
    sc.has_surface_flight_state = true;
    sc.altitude_m = 5000.0;
    sc.mass_kg = 40000.0;
    let sim = Simulation::new(&sc);
    let atmosphere = unsafe { world_atmosphere_velocity_i(&sim.world, sim.state.position_i_m) };
    assert_eq!(norm(sub(sim.state.velocity_i_mps, atmosphere)), 0.0);
}

#[test]
fn radar_height_uses_the_contact_deck() {
    // Measured KSC deck: radar height and physical contact use the same tilted plane.
    let mut sc = seed();
    sc.runway_slope_deg = -0.1274200834706079;
    let mut sim = Simulation::new(&sc);
    sim.state.gear_down = true;
    place(&mut sim, 1400.0, 0.0, 3.40, 70.0, 1.5, 8.0);
    assert!((vertical_m(&sim) - 3.40).abs() <= 0.002);
    run_until(&mut sim, 200, |s| s.state.touchdown_seen);
    assert!(sim.state.touchdown_seen && sim.state.on_runway_at_touchdown);
    assert!((sim.state.touchdown_sink_mps - 1.5).abs() <= 0.6);
    // The zero-slope far threshold uses the same planar clearance too.
    let mut sim = Simulation::new(&seed());
    let length = sim.runway.length_m;
    place(&mut sim, length + 30.0, 0.0, 3.40, 70.0, 1.5, 8.0);
    assert!((vertical_m(&sim) - 3.40).abs() <= 0.002);
}

#[test]
fn gentle_landing_stops_on_the_runway_at_the_measured_stance() {
    let mut sim = Simulation::new(&seed());
    sim.state.gear_down = true;
    place(&mut sim, 300.0, 0.0, 3.40, 70.0, 1.5, 8.0);
    run_until(&mut sim, 200, |s| s.state.touchdown_seen);
    let s = sim.state;
    assert!(s.touchdown_seen && s.touchdown_gear && s.on_runway_at_touchdown);
    assert!((s.touchdown_sink_mps - 1.5).abs() <= 0.6);
    sim.state.brakes = true;
    run_until(&mut sim, 6000, |s| s.state.stopped);
    let s = sim.state;
    assert!(s.stopped && s.nose_touchdown_seen && !s.runway_departure);
    assert!((pitch_deg(&sim) + 4.3).abs() <= 1.0, "stance pitch {}", pitch_deg(&sim));
    assert!(s.max_gear_load_g <= 4.0 && s.stop_along_m <= 2500.0);
}

#[test]
fn rolling_off_the_far_end_is_a_departure_not_a_fall() {
    let mut sim = Simulation::new(&seed());
    sim.state.gear_down = true;
    place(&mut sim, 2300.0, 0.0, 4.2, 60.0, 1.0, 8.0);
    run_until(&mut sim, 3000, |s| s.state.stopped);
    let p = plane(&sim);
    let height = dot(sub(sim.state.position_i_m, p.origin), p.normal);
    assert!(sim.state.runway_departure && sim.state.departure_along_m >= 2400.0 && height >= 1.0);
}

#[test]
fn gear_up_contact_is_a_belly_contact() {
    let mut sim = Simulation::new(&seed());
    sim.state.gear_down = false;
    place(&mut sim, 300.0, 0.0, 2.5, 70.0, 1.0, 4.0);
    run_until(&mut sim, 200, |s| s.state.touchdown_seen);
    assert!(sim.state.touchdown_seen && !sim.state.touchdown_gear);
}
