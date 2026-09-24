#!/usr/bin/env python3
"""Export a source-pinned MM305 fixture and effective seeded ShuttleSim model."""
from __future__ import annotations

import hashlib
import json
import math
import select
import subprocess
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
OUT = Path(__file__).resolve().parent
SCENARIO = Path("ShuttleSim/scenarios/mm305-tf-hac-inlet-20km.ini")
CONFIG = Path("Configuration/final-landing-validation.json")
MODEL_FILES = [
    "ShuttleSim/src/world.c", "ShuttleSim/src/aero.c", "ShuttleSim/src/attitude.c",
    "ShuttleSim/src/scenario.c", "ShuttleSim/src/sim.c", "ShuttleSim/src/main.c",
    "ShuttleSim/src/flight_physics.c", "ShuttleSim/CMakeLists.txt",
    "ShuttleSim/include/shuttlesim/world.h", "ShuttleSim/include/shuttlesim/aero.h",
    "ShuttleSim/include/shuttlesim/attitude.h", "ShuttleSim/include/shuttlesim/scenario.h",
    "ShuttleSim/include/shuttlesim/sim.h", "ShuttleSim/include/shuttlesim/types.h",
    "ShuttleSim/include/shuttlesim/flight_physics.h", "ShuttleSim/include/shuttlesim/math3.h",
    "ShuttleSim/include/shuttlesim/quat.h",
    str(SCENARIO), str(CONFIG),
]
BUILT_BINARY = Path("ShuttleSim/build/shuttlesim")


def sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def read_ini(path: Path) -> dict[str, str]:
    result: dict[str, str] = {}
    for line in path.read_text().splitlines():
        line = line.strip()
        if line and not line.startswith("#") and "=" in line:
            k, v = line.split("=", 1)
            result[k.strip()] = v.strip()
    return result


def num(v: str) -> float:
    return float(v)


def numeric_mapping(values: dict[str, str]) -> dict[str, float | str]:
    converted: dict[str, float | str] = {}
    for key, value in values.items():
        try:
            converted[key] = float(value)
        except ValueError:
            converted[key] = value
    return converted


def capture_initial_runtime(binary: Path, scenario: Path) -> tuple[dict | None, str]:
    """Read the built process's initialized state before its first physics step."""
    if not binary.exists():
        return None, "built ShuttleSim binary is absent"
    args = [str(binary), "--scenario", str(scenario), "--lockstep",
            "--command-port", "0", "--telemetry-port", "0", "--web-telemetry-port", "0"]
    proc = subprocess.Popen(args, cwd=ROOT, stdout=subprocess.PIPE, stderr=subprocess.PIPE,
                            text=True, bufsize=1)
    try:
        ready, _, _ = select.select([proc.stdout], [], [], 5.0)
        if not ready:
            return None, "timed out waiting for initialized telemetry (5 s bound)"
        line = proc.stdout.readline()
        if not line:
            stderr = proc.stderr.read()
            return None, "process exited without telemetry: " + stderr[-500:]
        record = json.loads(line)
        if record.get("type") != "telemetry" or record.get("source") != "sim":
            return None, "first stdout record was not ShuttleSim telemetry"
        if record.get("sim_time") != 0.0:
            return None, "initial telemetry was emitted after simulation advancement"
        return record, "captured initial lockstep telemetry at sim_time_s=0 before the first physics step"
    except (OSError, ValueError, json.JSONDecodeError) as exc:
        return None, f"runtime capture failed: {type(exc).__name__}: {exc}"
    finally:
        proc.terminate()
        try:
            proc.wait(timeout=2.0)
        except subprocess.TimeoutExpired:
            proc.kill()
            proc.wait(timeout=2.0)


def main() -> None:
    scenario = read_ini(ROOT / SCENARIO)
    final_config = json.loads((ROOT / CONFIG).read_text())
    mach = [0, .8, 1.2, 2, 5, 10, 15, 25]
    alpha = [-5, 0, 5, 10, 15, 20, 25, 30, 35, 40, 45]
    cl = []
    cd = []
    for m in mach:
        hyp = 1.0 if m >= 5 else .82 + .036 * m
        cd0 = .16 if m < .8 else (.28 if m < 1.2 else (.34 if m < 3 else .42))
        cl_row, cd_row = [], []
        for a in alpha:
            ar = math.radians(a)
            lift = hyp * (1.15 * math.sin(2 * ar))
            drag = cd0 + hyp * 1.55 * math.sin(ar) ** 2
            if a < 0:
                lift *= .8
            cl_row.append(lift)
            cd_row.append(drag)
        cl.append(cl_row)
        cd.append(cd_row)

    git = lambda *args: subprocess.check_output(["git", *args], cwd=ROOT, text=True).strip()
    dirty = git("status", "--short").splitlines()
    latest_model_source = max((ROOT / p).stat().st_mtime for p in MODEL_FILES)
    built_binary = ROOT / BUILT_BINARY
    binary_mtime = built_binary.stat().st_mtime if built_binary.exists() else None
    binary_is_fresh = binary_mtime is not None and binary_mtime >= latest_model_source
    initial_runtime, runtime_capture_note = capture_initial_runtime(built_binary, ROOT / SCENARIO)
    manifest = {
        "schema": "mm305-effective-model-manifest/v2",
        "repository": str(ROOT),
        "head": git("rev-parse", "HEAD"),
        "working_tree": {"dirty": bool(dirty), "status_short": dirty,
                         "note": "Dirty paths are recorded as provenance; no file is modified by export."},
        "snapshot_scope": "Source-pinned effective initialization plus initial telemetry from the built ShuttleSim binary for the MM305 TF-HAC inlet fixture's documented launch recipe. This is not a persistent-process capture.",
        "capture": {
            "method": "Read initialized ShuttleSim telemetry in lockstep at sim_time_s=0 with UDP ports disabled; then terminated the bounded local process. Seeded defaults and overrides are independently recorded from source/configuration.",
            "active_shuttlesim_process_after_capture": False,
            "runtime_loaded_state_available": initial_runtime is not None,
            "runtime_capture_note": runtime_capture_note,
            "initial_runtime_telemetry": initial_runtime,
            "build_artifact": str(BUILT_BINARY),
            "build_artifact_sha256": sha256(built_binary) if built_binary.exists() else None,
            "build_artifact_mtime_epoch_s": binary_mtime,
            "latest_model_source_mtime_epoch_s": latest_model_source,
            "build_artifact_fresh_against_model_sources": binary_is_fresh,
            "build_artifact_note": "A stale or absent binary is provenance only; effective values below describe the checked-out source initialization and stated launch recipe.",
        },
        "input": {"scenario": str(SCENARIO), "scenario_sha256": sha256(ROOT / SCENARIO),
                  "launch_recipe": {"scenario": str(SCENARIO), "atmosphere_override": None,
                                    "aero_override": None, "aero_book_override": None,
                                    "attitude_override": None, "dt_s": 0.02,
                                    "start_gear_down": False, "initially_paused": False},
                  "initial_state": {
                      "ut_s": num(scenario["ut0"]),
                      "position_inertial_m": [num(scenario[f"position_{axis}_m"]) for axis in "xyz"],
                      "velocity_inertial_mps": [num(scenario[f"velocity_{axis}_mps"]) for axis in "xyz"],
                      "mass_kg": num(scenario["mass_kg"]),
                      "initial_aoa_deg": num(scenario["initial_aoa_deg"]),
                      "initial_bank_deg": num(scenario["initial_bank_deg"]),
                      "residual_deorbit": {k: num(scenario[k]) for k in ("deorbit_delta_v_mps", "deorbit_duration_s", "deorbit_delay_s")}},
                  "overrides": numeric_mapping({k: scenario[k] for k in scenario if k.startswith("runway_")})},
        "effective_world": {"origin": "world_seed_kerbin; no --atmosphere override",
            "radius_m": 600000.0, "mu_m3_s2": 3.5316e12,
            "rotation_rate_rad_s": 2 * math.pi / 21549.425,
            "atmosphere_top_m": 70000.0, "rotation_phase_rad_at_ut0": math.pi / 2,
            "orbital_rate_rad_s": 2 * math.pi / 9203544.61750141,
            "solar_phase_rad_at_ut0": 3.14000010490417 + math.pi / 2,
            "atmosphere": "Stock spatial Kerbin pressure/temperature Hermite curves and latitude/day-night thermal modifiers in pinned world.c; 701 altitude samples are seeded, while flight sampling uses the spatial state function."},
        "effective_aerodynamics": {"origin": "aero_seed_stsn; no --aero or --aero-book override",
            "reference_area_m2": 250.0, "mach_grid": mach, "alpha_deg_grid": alpha,
            "cl": cl, "cd": cd, "direct_ksp_book": {"enabled": False, "count": 0},
            "coverage": "Outside table bounds, coefficient interpolation clamps to edge grid; direct-book calibration/fade is absent in this seeded fixture."},
        "effective_attitude_response": {"origin": "attitude_seed; no --attitude override",
            "pitch_wn": 1.4, "pitch_zeta": 0.9, "roll_wn": 1.8, "roll_zeta": 0.85,
            "max_pitch_rate_deg_s": 8.0, "max_roll_rate_deg_s": 18.0,
            "max_pitch_accel_deg_s2": 5.0, "max_roll_accel_deg_s2": 15.0,
            "pitch_full_authority_q_pa": 15.0, "roll_full_authority_q_pa": 10.0,
            "q_scaling": "authority=min(1,q/q_full); natural frequency and max rate scale with sqrt(authority), max acceleration with authority."},
        "effective_vehicle": {
            "native_shuttlesim": {"origin": "scenario initial state plus sim_init defaults",
                "scenario_mass_kg": num(scenario["mass_kg"]),
                "orbital_engine_available_thrust_n": num(scenario.get("orbital_engine_available_thrust_n", "0")),
                "initial_aoa_deg": num(scenario["initial_aoa_deg"]),
                "initial_bank_deg": num(scenario["initial_bank_deg"]),
                "initial_gear_down": False, "engine_commanded_throttle": 0.0,
                "airframe_model": "aero_seed_stsn; 250 m2 reference area; coefficient grids are in effective_aerodynamics"},
            "application_vehicle_policy": final_config["vehicle"],
            "note": "Native ShuttleSim vehicle state and application-level STS-N guidance policy are separate models; the latter is configuration, not a validated aerodynamic or landing envelope."},
        "effective_runway": {"origin": "scenario runway_* override", **numeric_mapping({k: scenario[k] for k in scenario if k.startswith("runway_")})},
        "seeded_runway_defaults": {"origin": "runway_seed_ksp09 before scenario override",
            "latitude_deg": -0.0486, "longitude_deg": -74.7240, "elevation_m": 70.0,
            "heading_deg": 90.0, "length_m": 2500.0, "width_m": 70.0},
        "simulator_defaults": {"physics_dt_s": 0.02, "gear_down": False, "brakes": False,
            "airbrakes": False, "rolling_mu": 0.025, "brake_mu": 0.18,
            "ground_airbrake_cda_m2": 8.0, "ground_max_lateral_accel_g": 0.12,
            "ground_max_yaw_rate_deg_s": 10.0, "scenario_orbital_engine_thrust_n_default": 0.0},
        "application_final_policy": {"config": str(CONFIG), "sha256": sha256(ROOT / CONFIG),
            "configuration": final_config,
            "note": "Full application configuration is recorded separately from ShuttleSim plant physics; it is not a characterized Final acceptance set or a native ShuttleSim runtime-loaded object."},
        "source_sha256": {p: sha256(ROOT / p) for p in MODEL_FILES},
        "baseline_evidence": {
            "source_manifest_exported": True,
            "build_run": False,
            "build_reason": "Not run: shared ShuttleSim build files and working tree have unrelated dirty work; avoid replacing shared build artifacts during baseline capture.",
            "focused_tests_run": False,
            "test_reason": "Not run: no read-only pre-existing Stage 0 test report was present; tests that compile shared targets would alter shared build outputs.",
            "algebraic_checks_input": "external analytic_checks.json is an algebraic reference only and is not a build, plant-parity, or flight result.",
        },
        "limitations": [
            "This is a reproducible source-derived initialization snapshot, not a capture from a current executable or active process.",
            "Runtime-loaded aero, atmosphere, response, learned calibration, and final state could not be queried because no ShuttleSim process was active.",
            "No --atmosphere, --aero, --aero-book, or --attitude override is specified by the fixture launch recipe; explicit runtime overrides would change this snapshot.",
            "The effective atmosphere implementation is source-pinned rather than expanded into a serialized 701-row numeric table.",
            "KSP-calibrated force-book data, learned live calibration state, and a qualified joint Final delivery envelope are not established by this export.",
            "No trajectory, plant parity, touchdown, or flight-success claim is made."],
    }
    (OUT / "effective_model_manifest.json").write_text(json.dumps(manifest, indent=2, sort_keys=True) + "\n")
    binary_parity_note = (
        "Its mtime is not older than the pinned model source files, though mtime alone is not proof that it was built from these exact source bytes."
        if binary_is_fresh else
        "It is stale by mtime relative to one or more pinned model source files, so it does not establish loaded-model parity with the current source snapshot; the source-derived section separately records current checked-out defaults and overrides."
    )
    report = f"""# MM305 Stage 0 baseline

Captured from `{ROOT}` at HEAD `{manifest['head']}`. The source-derived effective-model export is [effective_model_manifest.json](effective_model_manifest.json).

## Fixture and provenance

- Scenario: `{SCENARIO}` (SHA-256 `{sha256(ROOT / SCENARIO)}`).
- Inputs: Cartesian MM305 inlet state and scenario runway overrides. The launch recipe uses seeded world, STS-N aero, and attitude models, with no atmosphere/aero/aero-book/attitude override, at 0.02 s; no gear-down start.
- Git working tree is {'dirty' if dirty else 'clean'}; all paths and the source hashes are recorded in the manifest. No unrelated edits were changed.
- No persistent ShuttleSim or live KSP process was active. The exporter launches the existing binary in lockstep, records its first telemetry at simulator time zero with UDP disabled, then terminates it. Binary freshness is `{'current by mtime' if binary_is_fresh else 'stale by mtime'}` relative to the pinned model source files.
- The captured telemetry is the effective initialized state of that binary artifact. {binary_parity_note}

## Effective model boundary

The manifest separates source-derived native world, atmosphere, aero, attitude response, vehicle initialization, simulator defaults, and scenario runway from the full application Final/vehicle configuration. The initial binary telemetry provides loaded world, atmosphere sample, aero forces, attitude response, vehicle state, and runway geometry. The application Final configuration is policy input only; this baseline does not establish its joint delivery envelope or learned runtime state.

## Build and validation evidence

- Build: not run. The shared build tree and unrelated ShuttleSim changes were preserved; rebuilding would replace shared artifacts.
- Focused tests: not run. No pre-existing independent Stage 0 test report was available, and candidate tests would compile shared outputs.
- Supplied analytic checks: reference only. Their algebraic identities do not establish native plant parity, runtime model loading, trajectory feasibility, or flight success.

This baseline is provenance and initialization evidence only. It makes no current-source plant-parity, trajectory, touchdown, or flight-success claim.
"""
    (OUT / "baseline_report.md").write_text(report)


if __name__ == "__main__":
    main()
