//! ShuttleSim command line: scenario setup, optional replay, and the
//! (optionally lockstep) physics loop with UDP command/telemetry.

use shuttlesim::ffi::*;
use shuttlesim::protocol::Protocol;
use shuttlesim::replay::AttitudeReplay;
use shuttlesim::scenario::Scenario;
use shuttlesim::sim::Simulation;
use std::ffi::CString;
use std::io::Write;
use std::sync::atomic::{AtomicBool, Ordering};
use std::time::{Duration, Instant};

/// Backend Abort releases lockstep control. Termination must still preserve
/// the measured touchdown/rollout summary instead of losing flight evidence.
static STOP: AtomicBool = AtomicBool::new(false);

extern "C" fn request_stop(_: libc::c_int) {
    STOP.store(true, Ordering::SeqCst);
}

const USAGE: &str = "  --scenario FILE          Scenario INI (default scenarios/orbit86km.ini)
  --atmosphere FILE        Override Kerbin atmosphere CSV
  --aero FILE              Override fallback STS-N Mach/AoA aero CSV
  --aero-book FILE         Direct KSP force/q data book (log(q), Mach, AoA)
  --attitude FILE          Override vehicle attitude-response INI
  --dt SEC                 Physics step, default 0.02 (KSP fixed-update cadence)
  --rate max|N             Wall-clock pacing; N=sim seconds / real second
  --telemetry-hz N         JSONL publication rate in simulated time, default 10
  --max-sim-time SEC       Stop limit, default 2400
  --fixed-aoa DEG          Test driver; hold AoA command
  --plant-lift-scale X     Model-error campaigns: multiply plant lift (default 1)
  --plant-drag-scale X     Model-error campaigns: multiply plant drag (default 1)
  --fixed-bank DEG         Test driver; hold bank command
  --attitude-replay FILE   CSV: time_s,aoa_deg,bank_deg,gear_down
  --replay-mode MODE       command (servo) or actual (open-loop physics), default command
  --command-port PORT      UDP JSON command input, default 8795 (0 disables)
  --telemetry-host HOST    UDP telemetry destination, default 127.0.0.1
  --telemetry-port PORT    Guidance telemetry UDP destination, default 8796 (0 disables)
  --web-telemetry-port PORT  Telemetry Web UDP destination, default 8797 (0 disables)
  --record FILE            Also record telemetry JSONL
  --quiet                  Suppress stdout telemetry; summary still printed
  --gear-down              Start with gear down
  --start-paused           Bind I/O but do not advance until resume command
  --lockstep               Wait for a command with step=true after each telemetry frame
";

fn usage(program: &str) {
    eprint!("Usage: {program} [options]\n{USAGE}");
}

fn number(text: &str) -> f64 {
    text.parse::<f64>().ok().filter(|v| v.is_finite()).unwrap_or(f64::NAN)
}

fn port(text: &str) -> i64 {
    text.parse::<i64>().ok().filter(|v| (0..=65535).contains(v)).unwrap_or(-1)
}

fn cpath(path: &str) -> CString {
    CString::new(path).unwrap_or_default()
}

struct Output {
    quiet: bool,
    record: Option<std::fs::File>,
}

impl Output {
    fn publish(&mut self, proto: &Protocol, json: &str) {
        if !self.quiet {
            let mut out = std::io::stdout().lock();
            let _ = writeln!(out, "{json}");
            let _ = out.flush();
        }
        if let Some(f) = &mut self.record {
            let _ = writeln!(f, "{json}");
            let _ = f.flush();
        }
        proto.send_telemetry(json);
    }
}

/// Hold the frame until a step (or resume) command arrives, applying every
/// command received meanwhile; optionally resend the frame every 20 ms.
fn await_step(sim: &mut Simulation, proto: &Protocol, json: &str, resend: bool) {
    let mut resend_at = Instant::now() + Duration::from_millis(20);
    while !STOP.load(Ordering::SeqCst) {
        let mut advance = false;
        while let Some(c) = proto.poll_command() {
            sim.apply_command(&c);
            advance |= c.step || c.resume;
        }
        if advance {
            return;
        }
        let now = Instant::now();
        if resend && now >= resend_at {
            proto.send_telemetry(json);
            resend_at = now + Duration::from_millis(20);
        }
        std::thread::sleep(Duration::from_micros(200));
    }
}

fn main() {
    std::process::exit(run());
}

fn run() -> i32 {
    unsafe {
        libc::signal(libc::SIGTERM, request_stop as extern "C" fn(libc::c_int) as libc::sighandler_t);
        libc::signal(libc::SIGINT, request_stop as extern "C" fn(libc::c_int) as libc::sighandler_t);
    }
    let args: Vec<String> = std::env::args().collect();
    let program = args.first().map(String::as_str).unwrap_or("shuttlesim");
    let (mut scenario_path, mut atm_path, mut aero_path, mut book_path, mut attitude_path) = (None, None, None, None, None);
    let (mut record_path, mut replay_path) = (None, None);
    let mut thost = "127.0.0.1".to_string();
    let mut replay_mode = "command".to_string();
    let (mut dt, mut rate, mut telemetry_hz, mut max_time) = (0.02, 0.0, 10.0, 2400.0);
    let (mut lift_scale, mut drag_scale) = (1.0, 1.0);
    let (mut fixed_aoa, mut fixed_bank) = (None::<f64>, None::<f64>);
    let (mut quiet, mut start_gear, mut start_paused, mut lockstep) = (false, false, false, false);
    let (mut command_port, mut telemetry_port, mut web_port) = (8795i64, 8796i64, 8797i64);
    let mut i = 1;
    while i < args.len() {
        let flag = args[i].as_str();
        let value = args.get(i + 1).cloned();
        let takes = matches!(
            flag,
            "--scenario"
                | "--atmosphere"
                | "--aero"
                | "--aero-book"
                | "--attitude"
                | "--dt"
                | "--rate"
                | "--telemetry-hz"
                | "--max-sim-time"
                | "--plant-lift-scale"
                | "--plant-drag-scale"
                | "--fixed-aoa"
                | "--fixed-bank"
                | "--attitude-replay"
                | "--replay-mode"
                | "--command-port"
                | "--telemetry-host"
                | "--telemetry-port"
                | "--web-telemetry-port"
                | "--record"
        );
        if takes && value.is_none() {
            eprintln!("Unknown/incomplete option: {flag}");
            usage(program);
            return 2;
        }
        let v = value.unwrap_or_default();
        match flag {
            "--help" => {
                usage(program);
                return 0;
            }
            "--scenario" => scenario_path = Some(v),
            "--atmosphere" => atm_path = Some(v),
            "--aero" => aero_path = Some(v),
            "--aero-book" => book_path = Some(v),
            "--attitude" => attitude_path = Some(v),
            "--dt" => dt = number(&v),
            "--rate" => rate = if v == "max" { 0.0 } else { number(&v) },
            "--telemetry-hz" => telemetry_hz = number(&v),
            "--max-sim-time" => max_time = number(&v),
            "--plant-lift-scale" => lift_scale = number(&v),
            "--plant-drag-scale" => drag_scale = number(&v),
            "--fixed-aoa" => fixed_aoa = Some(number(&v)),
            "--fixed-bank" => fixed_bank = Some(number(&v)),
            "--attitude-replay" => replay_path = Some(v),
            "--replay-mode" => replay_mode = v,
            "--command-port" => command_port = port(&v),
            "--telemetry-host" => thost = v,
            "--telemetry-port" => telemetry_port = port(&v),
            "--web-telemetry-port" => web_port = port(&v),
            "--record" => record_path = Some(v),
            "--quiet" => quiet = true,
            "--gear-down" => start_gear = true,
            "--start-paused" => start_paused = true,
            "--lockstep" => lockstep = true,
            _ => {
                eprintln!("Unknown/incomplete option: {flag}");
                usage(program);
                return 2;
            }
        }
        i += if takes { 2 } else { 1 };
    }
    let finite = [dt, telemetry_hz, max_time, rate, fixed_aoa.unwrap_or(0.0), fixed_bank.unwrap_or(0.0)].iter().all(|v| v.is_finite());
    if !finite || dt <= 0.0 || dt > 1.0 || telemetry_hz <= 0.0 || max_time <= 0.0 || rate < 0.0 || command_port < 0 || telemetry_port < 0 || web_port < 0 {
        eprintln!("Invalid numeric option.");
        return 2;
    }
    if ((lockstep || start_paused) && command_port == 0) || (command_port > 0 && (command_port == telemetry_port || command_port == web_port)) {
        eprintln!("Interactive simulation requires a distinct command endpoint.");
        return 2;
    }
    let scenario = match &scenario_path {
        Some(p) => match Scenario::load(p) {
            Some(s) => s,
            None => {
                eprintln!("Failed to load scenario: {p}");
                return 3;
            }
        },
        None => Scenario::seed_86km(),
    };
    let mut sim = Simulation::new(&scenario);
    sim.physics_dt_s = dt;
    sim.state.gear_down = start_gear;
    sim.paused = start_paused;
    unsafe {
        if let Some(p) = &atm_path {
            if !world_load_atmosphere_csv(&mut sim.world, cpath(p).as_ptr()) {
                eprintln!("Failed to load atmosphere: {p}");
                return 3;
            }
        }
        if let Some(p) = &aero_path {
            if !aero_load_csv(&mut sim.aero, cpath(p).as_ptr()) {
                eprintln!("Failed to load aero table: {p}");
                return 3;
            }
        }
        if !(lift_scale > 0.0) || !(drag_scale > 0.0) {
            eprintln!("plant scales must be positive");
            return 2;
        }
        sim.aero.lift_scale = lift_scale;
        sim.aero.drag_scale = drag_scale;
        if let Some(p) = &book_path {
            if !aero_load_book_csv(&mut sim.aero, cpath(p).as_ptr()) {
                eprintln!("Failed to load aero data book: {p}");
                return 3;
            }
        }
        if let Some(p) = &attitude_path {
            if !attitude_load_ini(&mut sim.state.attitude, cpath(p).as_ptr()) {
                eprintln!("Failed to load attitude model: {p}");
                return 3;
            }
        }
    }
    if fixed_aoa.is_some() || fixed_bank.is_some() {
        let a = fixed_aoa.unwrap_or(rad2deg(sim.state.attitude.cmd_aoa_rad));
        let b = fixed_bank.unwrap_or(rad2deg(sim.state.attitude.cmd_bank_rad));
        sim.set_attitude(a, b);
    }
    let s = &mut sim.state;
    if s.attitude.direct_body_coordinates && !scenario.has_body_attitude {
        let vair = sub(s.velocity_i_mps, unsafe { world_atmosphere_velocity_i(&sim.world, s.position_i_m) });
        s.body_q_i = unsafe { attitude_body_quat_wind(s.position_i_m, vair, s.attitude.aoa_rad, s.attitude.bank_rad) };
    }
    // The first lockstep packet is a real state, not an uninitialized force
    // sample: refresh after loading the models without advancing anything.
    s.aero = unsafe {
        aero_compute(&sim.world, &sim.aero, s.position_i_m, s.velocity_i_mps, s.ut, s.mass_kg, s.attitude.aoa_rad, s.attitude.bank_rad)
    };
    let mut replay = None;
    if let Some(p) = &replay_path {
        match AttitudeReplay::load_csv(p) {
            Some(r) => replay = Some(r),
            None => {
                eprintln!("Failed to load attitude replay: {p}");
                return 3;
            }
        }
    }
    if replay_mode != "command" && replay_mode != "actual" {
        eprintln!("--replay-mode must be command or actual");
        return 2;
    }
    let proto = match Protocol::open(command_port as u16, &thost, telemetry_port as u16, web_port as u16) {
        Ok(p) => p,
        Err(e) => {
            eprintln!("{e}");
            eprintln!("Failed to open requested UDP endpoints; refusing an uncontrolled run.");
            return 3;
        }
    };
    let mut out = Output { quiet, record: None };
    if let Some(p) = &record_path {
        match std::fs::File::create(p) {
            Ok(f) => out.record = Some(f),
            Err(_) => {
                eprintln!("Failed to open record file: {p}");
                return 3;
            }
        }
    }
    let wall0 = Instant::now();
    let observed = |sim: &Simulation| {
        let wall = wall0.elapsed().as_secs_f64();
        if wall > 1e-6 { sim.state.sim_elapsed_s / wall } else { 0.0 }
    };
    let pub_period = 1.0 / telemetry_hz;
    let mut next_pub = 0.0;
    let resend = std::env::var("SHUTTLESIM_LOCKSTEP_RESEND").is_ok_and(|v| v == "1");
    if lockstep && !sim.paused {
        let json = sim.telemetry_json(observed(&sim));
        out.publish(&proto, &json);
        next_pub += pub_period;
        await_step(&mut sim, &proto, &json, resend);
    }
    while sim.state.sim_elapsed_s < max_time && !STOP.load(Ordering::SeqCst) {
        while let Some(c) = proto.poll_command() {
            sim.apply_command(&c);
        }
        if sim.paused {
            std::thread::sleep(Duration::from_millis(5));
            continue;
        }
        if let Some(r) = &mut replay {
            let rp = r.sample(sim.state.sim_elapsed_s);
            sim.state.gear_down = rp.gear_down;
            if replay_mode == "actual" {
                let a = &mut sim.state.attitude;
                a.aoa_rad = deg2rad(rp.aoa_deg);
                a.bank_rad = deg2rad(rp.bank_deg);
                a.cmd_aoa_rad = a.aoa_rad;
                a.requested_aoa_rad = a.aoa_rad;
                a.cmd_bank_rad = a.bank_rad;
                a.requested_bank_rad = a.bank_rad;
                a.aoa_rate_rad_s = 0.0;
                a.bank_rate_rad_s = 0.0;
            } else {
                sim.set_attitude(rp.aoa_deg, rp.bank_deg);
            }
        }
        sim.step(dt);
        if sim.state.sim_elapsed_s + 1e-12 >= next_pub {
            let json = sim.telemetry_json(observed(&sim));
            out.publish(&proto, &json);
            next_pub += pub_period;
            if lockstep {
                await_step(&mut sim, &proto, &json, resend);
            }
        }
        if rate > 0.0 {
            let target = sim.state.sim_elapsed_s / rate;
            let wall = wall0.elapsed().as_secs_f64();
            if target > wall {
                std::thread::sleep(Duration::from_secs_f64(target - wall));
            }
        }
        if sim.state.on_ground && sim.state.stopped {
            break;
        }
    }
    // The stop condition can occur between telemetry periods: publish the
    // terminal state, with no further lockstep acknowledgement.
    let final_wall = wall0.elapsed().as_secs_f64();
    let json = sim.telemetry_json(if final_wall > 0.0 { sim.state.sim_elapsed_s / final_wall } else { 0.0 });
    out.publish(&proto, &json);
    eprintln!("SHUTTLESIM_SUMMARY {}", sim.summary_json());
    0
}
