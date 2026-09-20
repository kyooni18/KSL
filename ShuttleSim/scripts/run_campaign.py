#!/usr/bin/env python3
"""Queued ShuttleSim campaign: broad 1-hour-simulated mission exploration.

Every case has a 3600 s simulated horizon. The campaign mixes:
- real Guidance-in-the-loop parameter sweeps,
- dynamics / aerodynamic / atmosphere / actuator perturbations,
- combined deterministic Monte Carlo cases,
- alternate aerodynamic algorithms,
- open-loop fixed/reversal/profile controls,
- historical replay controls.

No live KSP connection is used.
"""
from __future__ import annotations

import argparse
import concurrent.futures
import copy
import csv
import fcntl
import json
import math
import os
import pathlib
import random
import re
import shutil
import subprocess
import sys
import threading
import time
from dataclasses import dataclass
from typing import Any

ROOT = pathlib.Path(__file__).resolve().parents[2]
SIM = ROOT / "ShuttleSim"
GUIDANCE_RUNNER = SIM / "scripts" / "run_guidance.py"
BIN = SIM / "build" / "shuttlesim"
BASE_CONFIG = ROOT / "Configuration" / "default.json"
BASE_SCENARIO = SIM / "scenarios" / "ksp86km-postburn.ini"
HELDOUT_SCENARIO = SIM / "scenarios" / "ksp86km-heldout-20260915.ini"
BASE_ATMOS = SIM / "data" / "fitted" / "kerbin_atmosphere_ksp.csv"
BASE_AERO = SIM / "data" / "fitted" / "stsn_aero_ksp_robust.csv"
SEED_AERO = SIM / "data" / "stsn_aero_seed.csv"
BASE_BOOK = SIM / "data" / "fitted" / "stsn_force_book.csv"
BASE_ATT = SIM / "data" / "fitted" / "stsn_attitude_ksp.ini"
HIST_REPLAY = SIM / "data" / "fitted" / "replay-heldout-full.csv"
HORIZON = 3600.0
PRINT_LOCK = threading.Lock()


def log(msg: str) -> None:
    with PRINT_LOCK:
        print(msg, flush=True)


def load_ini(path: pathlib.Path) -> dict[str, str]:
    out: dict[str, str] = {}
    for raw in path.read_text().splitlines():
        line = raw.strip()
        if not line or line.startswith("#") or "=" not in line:
            continue
        k, v = line.split("=", 1)
        out[k.strip()] = v.strip()
    return out


def write_ini(path: pathlib.Path, values: dict[str, Any], comment: str = "") -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    lines = []
    if comment:
        lines.append(f"# {comment}")
    for k, v in values.items():
        if isinstance(v, bool):
            v = "true" if v else "false"
        lines.append(f"{k}={v}")
    path.write_text("\n".join(lines) + "\n")


def set_nested(obj: dict[str, Any], dotted: str, value: Any) -> None:
    parts = dotted.split(".")
    cur = obj
    for p in parts[:-1]:
        cur = cur[p]
    cur[parts[-1]] = value


def scaled_atmos(src: pathlib.Path, dst: pathlib.Path, scale: float) -> None:
    with src.open(newline="") as f, dst.open("w", newline="") as g:
        r = csv.DictReader(f)
        w = csv.DictWriter(g, fieldnames=r.fieldnames)
        w.writeheader()
        for row in r:
            row["density_kg_m3"] = f"{float(row['density_kg_m3']) * scale:.12g}"
            row["pressure_pa"] = f"{float(row['pressure_pa']) * scale:.12g}"
            w.writerow(row)


def scaled_book(src: pathlib.Path, dst: pathlib.Path, lift_scale: float, drag_scale: float) -> None:
    with src.open(newline="") as f, dst.open("w", newline="") as g:
        r = csv.DictReader(f)
        w = csv.DictWriter(g, fieldnames=r.fieldnames)
        w.writeheader()
        for row in r:
            row["lift_per_q_m2"] = f"{float(row['lift_per_q_m2']) * lift_scale:.12g}"
            row["drag_per_q_m2"] = f"{float(row['drag_per_q_m2']) * drag_scale:.12g}"
            w.writerow(row)


def scaled_attitude(src: pathlib.Path, dst: pathlib.Path, scale: float) -> None:
    vals = load_ini(src)
    for k in ("pitch_wn", "roll_wn", "max_pitch_rate_deg_s", "max_roll_rate_deg_s",
              "max_pitch_accel_deg_s2", "max_roll_accel_deg_s2"):
        if k in vals:
            vals[k] = f"{float(vals[k]) * scale:.9g}"
    write_ini(dst, vals, f"attitude response x{scale:.3f}")


def scenario_variant(src: pathlib.Path, dst: pathlib.Path, *,
                     mass_scale: float = 1.0,
                     speed_delta_mps: float = 0.0,
                     cross_velocity_delta_mps: float = 0.0) -> None:
    v = load_ini(src)
    v["mass_kg"] = f"{float(v['mass_kg']) * mass_scale:.12g}"
    vx, vy, vz = (float(v["velocity_x_mps"]), float(v["velocity_y_mps"]), float(v["velocity_z_mps"]))
    speed = math.sqrt(vx * vx + vy * vy + vz * vz)
    if speed_delta_mps:
        vx += vx / speed * speed_delta_mps
        vy += vy / speed * speed_delta_mps
        vz += vz / speed * speed_delta_mps
    vz += cross_velocity_delta_mps
    v["velocity_x_mps"] = f"{vx:.12g}"
    v["velocity_y_mps"] = f"{vy:.12g}"
    v["velocity_z_mps"] = f"{vz:.12g}"
    write_ini(dst, v, f"mass x{mass_scale:.5f}, speed delta {speed_delta_mps:+.3f}, cross-v {cross_velocity_delta_mps:+.3f}")


def write_reversal_replay(path: pathlib.Path, aoa: float, bank: float, period: float) -> None:
    rows = [(0.0, aoa, bank, 0)]
    t = period
    sign = -1.0
    while t <= HORIZON:
        rows.append((max(0.0, t - 0.02), aoa, -sign * bank, 0))
        rows.append((t, aoa, sign * bank, 0))
        sign *= -1.0
        t += period
    rows.append((HORIZON, max(12.0, aoa - 8.0), 0.0, 1))
    with path.open("w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["time_s", "aoa_deg", "bank_deg", "gear_down"])
        w.writerows(rows)


def write_profile_replay(path: pathlib.Path, kind: str) -> None:
    profiles: dict[str, list[tuple[float, float, float, int]]] = {
        "energy_hold": [(0,25,0,0),(450,25,55,0),(750,27,-55,0),(1050,30,45,0),(1250,22,0,0),(1500,14,0,1),(3600,10,0,1)],
        "high_aoa_final": [(0,25,0,0),(500,25,65,0),(850,25,-65,0),(1080,35,-55,0),(1250,35,35,0),(1400,18,0,0),(1600,12,0,1),(3600,10,0,1)],
        "late_reversal": [(0,25,0,0),(650,25,65,0),(1050,28,-70,0),(1300,30,55,0),(1500,16,0,1),(3600,10,0,1)],
        "early_reversal": [(0,25,60,0),(300,25,-60,0),(600,25,55,0),(850,28,-50,0),(1100,30,35,0),(1400,15,0,1),(3600,10,0,1)],
        "shallow_bank": [(0,24,35,0),(500,25,-35,0),(900,28,35,0),(1200,25,-30,0),(1450,14,0,1),(3600,10,0,1)],
        "aggressive_bank": [(0,26,75,0),(420,28,-75,0),(760,30,70,0),(1050,35,-65,0),(1300,20,35,0),(1500,12,0,1),(3600,10,0,1)],
    }
    with path.open("w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["time_s", "aoa_deg", "bank_deg", "gear_down"])
        w.writerows(profiles[kind])


def parse_summary(stderr_path: pathlib.Path) -> dict[str, Any]:
    if not stderr_path.exists():
        return {}
    out: dict[str, Any] = {}
    for line in stderr_path.read_text(errors="replace").splitlines():
        marker = "SHUTTLESIM_SUMMARY "
        if marker in line:
            try:
                out = json.loads(line.split(marker, 1)[1])
            except Exception:
                pass
    return out


def parse_sim_metrics(path: pathlib.Path) -> dict[str, Any]:
    m: dict[str, Any] = {
        "frames": 0, "max_q_pa": 0.0, "max_mach": 0.0,
        "terminal_metric_m": float("inf"), "closest_horizontal_m": float("inf"),
        "closest_altitude_m": None, "closest_speed_mps": None,
        "final_altitude_m": None, "final_speed_mps": None,
        "final_along_m": None, "final_cross_m": None,
        "sim_time_s": 0.0, "touchdown": False, "on_runway": False,
    }
    if not path.exists():
        return m
    with path.open(errors="replace") as f:
        for line in f:
            try:
                r = json.loads(line)
            except Exception:
                continue
            pos = r.get("position") or {}
            vel = r.get("velocity") or {}
            aero = r.get("aero") or {}
            rw = r.get("runway") or {}
            ground = r.get("ground") or {}
            try:
                alt = float(pos.get("altitude_m", 1e12))
                along = float(rw.get("along_m", 1e12))
                cross = float(rw.get("cross_m", 1e12))
                vertical = float(rw.get("vertical_m", alt - 70.0))
                speed = float(vel.get("air_mps", 0.0))
                horiz = math.hypot(along, cross)
                # runway-seeking 3D objective; vertical error weighted because a
                # high overflight is not a useful terminal solution.
                terminal = math.sqrt(along * along + cross * cross + (2.0 * vertical) ** 2)
                if terminal < m["terminal_metric_m"]:
                    m["terminal_metric_m"] = terminal
                    m["closest_horizontal_m"] = horiz
                    m["closest_altitude_m"] = alt
                    m["closest_speed_mps"] = speed
                m["max_q_pa"] = max(m["max_q_pa"], float(aero.get("q_pa", 0.0) or 0.0))
                m["max_mach"] = max(m["max_mach"], float(aero.get("mach", 0.0) or 0.0))
                m["final_altitude_m"] = alt
                m["final_speed_mps"] = speed
                m["final_along_m"] = along
                m["final_cross_m"] = cross
                m["sim_time_s"] = float(r.get("sim_time", m["sim_time_s"]) or m["sim_time_s"])
                m["touchdown"] = bool(ground.get("touchdown_seen", m["touchdown"]))
                m["on_runway"] = bool(ground.get("on_runway_touchdown", m["on_runway"]))
                m["frames"] += 1
            except Exception:
                continue
    if not math.isfinite(m["terminal_metric_m"]):
        m["terminal_metric_m"] = 1e12
    return m


def parse_guidance(path: pathlib.Path) -> dict[str, Any]:
    phases: list[str] = []
    final_phase = ""
    final_status = ""
    if path.exists():
        with path.open(errors="replace") as f:
            for line in f:
                try:
                    obj = json.loads(line)
                except Exception:
                    continue
                if obj.get("type") != "snapshot" or not isinstance(obj.get("snapshot"), dict):
                    continue
                snap = obj["snapshot"]
                phase = str(snap.get("phase") or "")
                if phase and (not phases or phases[-1] != phase):
                    phases.append(phase)
                final_phase = phase
                final_status = str(snap.get("statusMessage") or "")
    return {"phases": phases, "final_phase": final_phase, "final_status": final_status}


def rank_key(r: dict[str, Any]) -> tuple:
    return (
        0 if r.get("on_runway") else 1,
        0 if r.get("touchdown") else 1,
        float(r.get("terminal_metric_m", 1e12)),
        abs(float(r.get("final_cross_m") or 1e12)),
    )


def write_result_csv(path: pathlib.Path, rows: list[dict[str, Any]]) -> None:
    fields = [
        "rank","id","family","mode","description","success","on_runway","touchdown",
        "terminal_metric_m","closest_horizontal_m","closest_altitude_m","closest_speed_mps",
        "final_altitude_m","final_speed_mps","final_along_m","final_cross_m",
        "max_q_pa","max_mach","sim_time_s","wall_s","returncode","final_phase","params_json"
    ]
    with path.open("w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=fields, extrasaction="ignore")
        w.writeheader()
        for i, r in enumerate(rows, 1):
            x = dict(r)
            x["rank"] = i
            x["params_json"] = json.dumps(r.get("params", {}), sort_keys=True, separators=(",", ":"))
            w.writerow(x)


class Factory:
    def __init__(self, cdir: pathlib.Path):
        self.cdir = cdir
        self.inputs = cdir / "inputs"
        for sub in ("configs","scenarios","atmospheres","aero_books","attitudes","replays"):
            (self.inputs / sub).mkdir(parents=True, exist_ok=True)
        self.base_config = json.loads(BASE_CONFIG.read_text())
        self.cache: dict[tuple, pathlib.Path] = {}

    def config(self, case_id: str, changes: dict[str, Any]) -> pathlib.Path:
        p = self.inputs / "configs" / f"{case_id}.json"
        cfg = copy.deepcopy(self.base_config)
        for k, v in changes.items():
            set_nested(cfg, k, v)
        p.write_text(json.dumps(cfg, indent=2, sort_keys=True) + "\n")
        return p

    def atmosphere(self, scale: float) -> pathlib.Path:
        if abs(scale - 1.0) < 1e-12:
            return BASE_ATMOS
        key = ("atmos", round(scale, 6))
        if key not in self.cache:
            p = self.inputs / "atmospheres" / f"density-{scale:.4f}.csv"
            scaled_atmos(BASE_ATMOS, p, scale)
            self.cache[key] = p
        return self.cache[key]

    def book(self, lift: float, drag: float) -> pathlib.Path:
        if abs(lift - 1.0) < 1e-12 and abs(drag - 1.0) < 1e-12:
            return BASE_BOOK
        key = ("book", round(lift, 6), round(drag, 6))
        if key not in self.cache:
            p = self.inputs / "aero_books" / f"book-L{lift:.4f}-D{drag:.4f}.csv"
            scaled_book(BASE_BOOK, p, lift, drag)
            self.cache[key] = p
        return self.cache[key]

    def attitude(self, scale: float) -> pathlib.Path:
        if abs(scale - 1.0) < 1e-12:
            return BASE_ATT
        key = ("att", round(scale, 6))
        if key not in self.cache:
            p = self.inputs / "attitudes" / f"response-{scale:.4f}.ini"
            scaled_attitude(BASE_ATT, p, scale)
            self.cache[key] = p
        return self.cache[key]

    def scenario(self, case_id: str, mass: float = 1.0, speed_delta: float = 0.0, cross_v: float = 0.0) -> pathlib.Path:
        if abs(mass-1)<1e-12 and abs(speed_delta)<1e-12 and abs(cross_v)<1e-12:
            return BASE_SCENARIO
        p = self.inputs / "scenarios" / f"{case_id}.ini"
        scenario_variant(BASE_SCENARIO, p, mass_scale=mass, speed_delta_mps=speed_delta, cross_velocity_delta_mps=cross_v)
        return p

    def reversal(self, case_id: str, aoa: float, bank: float, period: float) -> pathlib.Path:
        p = self.inputs / "replays" / f"{case_id}.csv"
        write_reversal_replay(p, aoa, bank, period)
        return p

    def profile(self, case_id: str, kind: str) -> pathlib.Path:
        p = self.inputs / "replays" / f"{case_id}.csv"
        write_profile_replay(p, kind)
        return p


def make_guidance_case(idx: int, factory: Factory, family: str, desc: str,
                       config_changes: dict[str, Any] | None = None,
                       *, scenario: pathlib.Path | None = None,
                       atmosphere: pathlib.Path | None = None,
                       aero: pathlib.Path | None = None,
                       aero_book: pathlib.Path | None = None,
                       attitude: pathlib.Path | None = None,
                       dt: float = 0.02,
                       params: dict[str, Any] | None = None) -> dict[str, Any]:
    cid = f"G{idx:03d}"
    config_changes = config_changes or {}
    return {
        "id": cid, "family": family, "mode": "guidance", "description": desc,
        "configuration": str(factory.config(cid, config_changes)),
        "scenario": str(scenario or BASE_SCENARIO),
        "atmosphere": str(atmosphere or BASE_ATMOS),
        "aero": str(aero or BASE_AERO),
        "aero_book": str(aero_book or BASE_BOOK),
        "attitude": str(attitude or BASE_ATT),
        "dt": dt, "telemetry_hz": 10.0, "max_sim_time": HORIZON,
        "params": params or config_changes,
    }


def make_open_case(idx: int, family: str, desc: str, *,
                   scenario: pathlib.Path = BASE_SCENARIO,
                   atmosphere: pathlib.Path = BASE_ATMOS,
                   aero: pathlib.Path = BASE_AERO,
                   aero_book: pathlib.Path | None = BASE_BOOK,
                   attitude: pathlib.Path = BASE_ATT,
                   dt: float = 0.02,
                   fixed_aoa: float | None = None,
                   fixed_bank: float | None = None,
                   replay: pathlib.Path | None = None,
                   replay_mode: str = "command",
                   params: dict[str, Any] | None = None) -> dict[str, Any]:
    return {
        "id": f"O{idx:03d}", "family": family, "mode": "open_loop", "description": desc,
        "scenario": str(scenario), "atmosphere": str(atmosphere), "aero": str(aero),
        "aero_book": str(aero_book) if aero_book else "none", "attitude": str(attitude),
        "dt": dt, "telemetry_hz": 5.0, "max_sim_time": HORIZON,
        "fixed_aoa": fixed_aoa, "fixed_bank": fixed_bank,
        "replay": str(replay) if replay else None, "replay_mode": replay_mode,
        "params": params or {},
    }


def build_queue(factory: Factory, seed: int) -> list[dict[str, Any]]:
    q: list[dict[str, Any]] = []
    gi = 0
    oi = 0

    def G(*args, **kwargs):
        nonlocal gi
        q.append(make_guidance_case(gi, factory, *args, **kwargs)); gi += 1

    def O(*args, **kwargs):
        nonlocal oi
        q.append(make_open_case(oi, *args, **kwargs)); oi += 1

    # 1 nominal Guidance baseline.
    G("baseline", "current Guidance + nominal direct-force physics")

    # 69 one-factor Guidance policy/numerics sweeps.
    specs = [
        ("vehicle.entryAngleOfAttack",[16,20,22,24]),
        ("vehicle.maximumAngleOfAttack",[30,32,35,38]),
        ("vehicle.maximumBankAngle",[55,60,65,75,80]),
        ("guidance.entryRollRate",[3.0,4.0,5.5,6.5]),
        ("guidance.entryRollAcceleration",[1.2,2.4,3.0]),
        ("guidance.sTurnMinimumLegDuration",[16,20,28,32,40]),
        ("guidance.taemInterfaceAltitude",[14000,15000,17500,18500,20000]),
        ("guidance.taemInterfaceRange",[25000,30000,40000,45000]),
        ("guidance.hacRadius",[9000,10500,13500,15000]),
        ("guidance.hacLookAheadAngle",[10,12,20,24]),
        ("guidance.taemGlideSlope",[9,10,14,16]),
        ("guidance.taemForceHandoffSpeed",[1100,1200,1400,1500]),
        ("guidance.finalGlideSlope",[16,18,22,24]),
        ("guidance.finalApproachDistance",[6000,10000,12000]),
        ("guidance.flareAltitude",[40,70,90]),
        ("guidance.gearDeploymentAltitude",[1000,2000,2500]),
        ("guidance.predictionInterval",[0.75,1.0,2.0,2.5]),
        ("guidance.guidanceRate",[5,20]),
    ]
    for path, values in specs:
        for value in values:
            G("guidance_sweep", f"{path}={value}", {path:value}, params={path:value})

    # Algorithm/learning toggles.
    for changes, desc in [
        ({"calibration.enableTrajectoryCalibration":False},"trajectory calibration disabled"),
        ({"calibration.enablePassiveCalibration":False},"passive calibration disabled"),
        ({"calibration.autoApplyInFlight":False},"in-flight calibration application disabled"),
        ({"calibration.trajectoryLearningRate":0.05},"trajectory learning rate 0.05"),
        ({"calibration.trajectoryLearningRate":0.10},"trajectory learning rate 0.10"),
        ({"calibration.trajectoryLearningRate":0.30},"trajectory learning rate 0.30"),
        ({"calibration.trajectoryLearningRate":0.40},"trajectory learning rate 0.40"),
    ]:
        G("algorithm_toggle", desc, changes, params=changes)

    # Dynamics/model/numerics one-factor sweeps (37).
    for scale in (0.90,0.95,1.05,1.10):
        cid = f"tmp-mass-{scale}"
        sc = factory.scenario(cid, mass=scale)
        G("dynamics", f"vehicle mass x{scale:.2f}", scenario=sc, params={"mass_scale":scale})
    for dv in (-20,-10,10,20):
        sc = factory.scenario(f"tmp-speed-{dv}", speed_delta=dv)
        G("dynamics", f"postburn inertial speed {dv:+.0f} m/s", scenario=sc, params={"speed_delta_mps":dv})
    for cv in (-15,-5,5,15):
        sc = factory.scenario(f"tmp-cross-{cv}", cross_v=cv)
        G("dynamics", f"cross-plane velocity {cv:+.0f} m/s", scenario=sc, params={"cross_velocity_delta_mps":cv})
    for scale in (0.90,0.95,1.05,1.10):
        G("atmosphere", f"Kerbin density x{scale:.2f}", atmosphere=factory.atmosphere(scale), params={"density_scale":scale})
    for scale in (0.90,0.95,1.05,1.10):
        G("aero", f"lift/q x{scale:.2f}", aero_book=factory.book(scale,1.0), params={"lift_scale":scale})
    for scale in (0.90,0.95,1.05,1.10):
        G("aero", f"drag/q x{scale:.2f}", aero_book=factory.book(1.0,scale), params={"drag_scale":scale})
    for ls, ds in ((0.90,1.10),(1.10,0.90),(0.95,0.90),(1.05,1.10)):
        G("aero", f"coupled lift x{ls:.2f} drag x{ds:.2f}", aero_book=factory.book(ls,ds),
          params={"lift_scale":ls,"drag_scale":ds})
    for scale in (0.70,0.85,1.15,1.30):
        G("actuator", f"attitude response x{scale:.2f}", attitude=factory.attitude(scale), params={"attitude_scale":scale})
    for dt in (0.01,0.04,0.08):
        G("numerics", f"physics dt={dt}", dt=dt, params={"dt":dt})
    G("aero_algorithm", "robust Mach/AoA fallback only", aero_book=None, params={"aero_algorithm":"fallback_robust"})
    # Explicit string "none" is handled after construction.
    q[-1]["aero_book"] = "none"
    G("aero_algorithm", "seed Mach/AoA model only", aero=SEED_AERO, aero_book=None, params={"aero_algorithm":"seed"})
    q[-1]["aero_book"] = "none"

    # 40 deterministic combined Monte Carlo / policy co-variation cases.
    rng = random.Random(seed)
    for j in range(40):
        mass = rng.uniform(0.95,1.05)
        dv = rng.uniform(-12,12)
        cv = rng.uniform(-5,5)
        density = rng.uniform(0.93,1.07)
        lift = rng.uniform(0.92,1.08)
        drag = rng.uniform(0.90,1.10)
        att = rng.uniform(0.80,1.20)
        cfg = {
            "vehicle.entryAngleOfAttack": round(rng.uniform(16,24),3),
            "vehicle.maximumAngleOfAttack": round(rng.uniform(30,36),3),
            "vehicle.maximumBankAngle": round(rng.uniform(60,80),3),
            "guidance.entryRollRate": round(rng.uniform(3.5,6.5),3),
            "guidance.sTurnMinimumLegDuration": round(rng.uniform(18,36),3),
            "guidance.taemInterfaceAltitude": round(rng.uniform(14000,19000),1),
            "guidance.taemInterfaceRange": round(rng.uniform(28000,43000),1),
            "guidance.hacRadius": round(rng.uniform(9000,15000),1),
            "guidance.finalGlideSlope": round(rng.uniform(17,23),3),
            "guidance.flareAltitude": round(rng.uniform(40,80),2),
        }
        sc = factory.scenario(f"mc-{j:02d}", mass=mass, speed_delta=dv, cross_v=cv)
        params = {
            **cfg, "mass_scale":mass, "speed_delta_mps":dv, "cross_velocity_delta_mps":cv,
            "density_scale":density, "lift_scale":lift, "drag_scale":drag, "attitude_scale":att,
            "seed":seed, "mc_index":j,
        }
        G("monte_carlo", f"combined deterministic Monte Carlo {j:02d}", cfg,
          scenario=sc, atmosphere=factory.atmosphere(density), aero_book=factory.book(lift,drag),
          attitude=factory.attitude(att), params=params)

    assert gi == 154, gi

    # 46 open-loop / alternate strategy controls.
    for aoa in (15,20,25,30,35):
        for bank in (0,-60,60):
            O("fixed_strategy", f"fixed AoA {aoa} bank {bank}", fixed_aoa=aoa, fixed_bank=bank,
              params={"algorithm":"fixed","aoa":aoa,"bank":bank})
    for aoa in (20,25,30,35):
        for bank in (-75,75):
            O("fixed_strategy", f"fixed AoA {aoa} bank {bank}", fixed_aoa=aoa, fixed_bank=bank,
              params={"algorithm":"fixed","aoa":aoa,"bank":bank})
    for period in (80,120,180,240):
        for bank in (55,70):
            cid=f"reversal-{period}-{bank}"
            rp=factory.reversal(cid,25,bank,period)
            O("reversal_strategy", f"periodic reversal {period}s bank ±{bank}", replay=rp, replay_mode="command",
              params={"algorithm":"periodic_reversal","aoa":25,"bank":bank,"period_s":period})
    for kind in ("energy_hold","high_aoa_final","late_reversal","early_reversal","shallow_bank","aggressive_bank"):
        rp=factory.profile(f"profile-{kind}",kind)
        O("profile_strategy", kind.replace("_"," "), replay=rp, replay_mode="command",
          params={"algorithm":"profile","profile":kind})
    for mode in ("actual","command"):
        for model in ("direct","fallback","seed"):
            aero = BASE_AERO if model!="seed" else SEED_AERO
            book = BASE_BOOK if model=="direct" else None
            O("historical_replay", f"held-out historical {mode} replay using {model} aero",
              scenario=HELDOUT_SCENARIO, aero=aero, aero_book=book, replay=HIST_REPLAY,
              replay_mode=mode, params={"algorithm":"historical_replay","replay_mode":mode,"aero_algorithm":model})
    for dt in (0.01,0.04,0.08):
        O("numerics_control", f"fixed AoA25 bank0 dt={dt}", fixed_aoa=25, fixed_bank=0, dt=dt,
          params={"algorithm":"fixed","aoa":25,"bank":0,"dt":dt})

    assert oi == 46, oi
    assert len(q) == 200, len(q)
    return q


def execute_guidance(case: dict[str, Any], campaign: pathlib.Path, retain: bool = False) -> dict[str, Any]:
    idx = int(case["id"][1:])
    base_port = 23000 + idx * 3
    label = f"campaign-{campaign.name}-{case['id']}"
    cmd = [
        sys.executable, str(GUIDANCE_RUNNER),
        "--scenario", case["scenario"],
        "--configuration", case["configuration"],
        "--dt", str(case["dt"]),
        "--atmosphere", case["atmosphere"],
        "--aero", case["aero"],
        "--aero-book", case["aero_book"],
        "--attitude", case["attitude"],
        "--telemetry-hz", str(case["telemetry_hz"]),
        "--max-sim-time", str(case["max_sim_time"]),
        "--label", label,
        "--command-port", str(base_port),
        "--telemetry-port", str(base_port + 1),
        "--web-telemetry-port", "0",
        "--skip-build", "--no-mirror", "--quiet-progress",
    ]
    started = time.monotonic()
    p = subprocess.run(cmd, cwd=ROOT, text=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    wall = time.monotonic() - started
    result_obj: dict[str, Any] = {}
    for line in reversed(p.stdout.splitlines()):
        try:
            x = json.loads(line)
        except Exception:
            continue
        if isinstance(x, dict) and "sim" in x:
            result_obj = x
            break
    sim_log = pathlib.Path(result_obj.get("sim",""))
    guide_log = pathlib.Path(result_obj.get("guidance",""))
    be_err = pathlib.Path(result_obj.get("backendStderr",""))
    sim_err = pathlib.Path(result_obj.get("simStderr",""))
    metrics = parse_sim_metrics(sim_log) if sim_log else {}
    metrics.update(parse_summary(sim_err) if sim_err else {})
    g = parse_guidance(guide_log) if guide_log else {}
    out = {
        **case, **metrics, **g,
        "wall_s": wall, "returncode": p.returncode,
        "success": bool(result_obj.get("success", False)),
        "runner_stdout_tail": p.stdout.splitlines()[-3:],
        "runner_stderr_tail": p.stderr.splitlines()[-3:],
    }
    if retain:
        tdir = campaign / "top" / case["id"]
        tdir.mkdir(parents=True, exist_ok=True)
        for src in (sim_log, guide_log, be_err, sim_err):
            if src and src.exists():
                shutil.copy2(src, tdir / src.name)
        out["retained_dir"] = str(tdir)
    for src in (sim_log, guide_log, be_err, sim_err):
        if src and src.exists():
            try: src.unlink()
            except OSError: pass
    return out


def execute_open(case: dict[str, Any], campaign: pathlib.Path, retain: bool = False) -> dict[str, Any]:
    run_dir = campaign / ("top" if retain else "scratch")
    run_dir.mkdir(parents=True, exist_ok=True)
    record = run_dir / f"{case['id']}.jsonl"
    cmd = [
        str(BIN), "--scenario", case["scenario"], "--atmosphere", case["atmosphere"],
        "--aero", case["aero"], "--attitude", case["attitude"],
        "--dt", str(case["dt"]), "--rate", "max", "--telemetry-hz", str(case["telemetry_hz"]),
        "--max-sim-time", str(case["max_sim_time"]),
        "--command-port", "0", "--telemetry-port", "0", "--web-telemetry-port", "0",
        "--record", str(record), "--quiet",
    ]
    if case["aero_book"].lower() != "none":
        cmd += ["--aero-book", case["aero_book"]]
    if case.get("replay"):
        cmd += ["--attitude-replay", case["replay"], "--replay-mode", case["replay_mode"]]
    else:
        if case.get("fixed_aoa") is not None:
            cmd += ["--fixed-aoa", str(case["fixed_aoa"])]
        if case.get("fixed_bank") is not None:
            cmd += ["--fixed-bank", str(case["fixed_bank"])]
    started = time.monotonic()
    p = subprocess.run(cmd, cwd=ROOT, text=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    wall = time.monotonic() - started
    metrics = parse_sim_metrics(record)
    summary: dict[str, Any] = {}
    for line in p.stderr.splitlines():
        if "SHUTTLESIM_SUMMARY " in line:
            try: summary = json.loads(line.split("SHUTTLESIM_SUMMARY ",1)[1])
            except Exception: pass
    metrics.update(summary)
    out = {
        **case, **metrics, "wall_s":wall, "returncode":p.returncode,
        "success": bool(summary.get("on_runway",False)),
        "final_phase":"open-loop",
        "runner_stderr_tail":p.stderr.splitlines()[-3:],
    }
    if not retain and record.exists():
        record.unlink()
    elif retain:
        out["retained_dir"] = str(run_dir)
    return out


def execute_case(case: dict[str, Any], campaign: pathlib.Path, retain: bool = False) -> dict[str, Any]:
    try:
        if case["mode"] == "guidance":
            return execute_guidance(case, campaign, retain)
        return execute_open(case, campaign, retain)
    except Exception as e:
        return {**case, "success":False, "on_runway":False, "touchdown":False,
                "terminal_metric_m":1e12, "wall_s":0.0, "returncode":999,
                "error":repr(e), "final_phase":"runner-error"}


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--workers", type=int, default=4)
    ap.add_argument("--seed", type=int, default=20260918)
    ap.add_argument("--campaign-dir")
    ap.add_argument("--generate-only", action="store_true")
    ap.add_argument("--resume", action="store_true")
    ap.add_argument("--rerun-top", type=int, default=8)
    args = ap.parse_args()

    if args.workers < 1 or args.workers > 8:
        raise SystemExit("--workers must be 1..8")

    stamp = time.strftime("%Y%m%dT%H%M%SZ", time.gmtime())
    campaign = pathlib.Path(args.campaign_dir) if args.campaign_dir else SIM / "campaigns" / f"{stamp}-one-hour"
    campaign.mkdir(parents=True, exist_ok=True)
    runner_lock = (campaign / ".runner.lock").open("a+")
    try:
        fcntl.flock(runner_lock.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
    except BlockingIOError:
        raise SystemExit(f"campaign already active: {campaign}")
    runner_lock.seek(0)
    runner_lock.truncate()
    runner_lock.write(f"pid={os.getpid()} started_utc={stamp}\n")
    runner_lock.flush()
    factory = Factory(campaign)
    queue_cases = build_queue(factory, args.seed)
    qpath = campaign / "queue.jsonl"
    qpath.write_text("".join(json.dumps(c,sort_keys=True,separators=(",",":"))+"\n" for c in queue_cases))

    manifest = {
        "created_utc": stamp, "seed": args.seed, "simulated_horizon_s_per_case": HORIZON,
        "case_count": len(queue_cases), "guidance_cases": sum(c["mode"]=="guidance" for c in queue_cases),
        "open_loop_cases": sum(c["mode"]=="open_loop" for c in queue_cases),
        "workers": args.workers, "live_ksp_used": False,
        "description": "Broad queued ShuttleSim one-hour-simulated mission campaign",
    }
    (campaign/"manifest.json").write_text(json.dumps(manifest,indent=2,sort_keys=True)+"\n")
    log(f"CAMPAIGN {campaign}")
    log(f"QUEUE {len(queue_cases)} cases: {manifest['guidance_cases']} Guidance, {manifest['open_loop_cases']} open-loop; horizon={HORIZON:.0f}s/case")

    if args.generate_only:
        return 0

    # Build once before the worker pool.
    subprocess.run(["make","-C",str(ROOT/"CLanding"),"-j","4"],check=True,stdout=subprocess.DEVNULL)
    subprocess.run(["cmake","--build",str(SIM/"build"),"-j","4"],check=True,stdout=subprocess.DEVNULL)

    results_path = campaign / "results.jsonl"
    completed: dict[str, dict[str, Any]] = {}
    raw_result_count = 0
    if args.resume and results_path.exists():
        for line in results_path.read_text().splitlines():
            try:
                r=json.loads(line)
                cid=r["id"]
                raw_result_count += 1
                old=completed.get(cid)
                score=lambda x: ((1 if int(x.get("returncode",999)) == 999 else 0), *rank_key(x))
                if old is None or score(r) < score(old):
                    completed[cid]=r
            except Exception:
                pass
        if raw_result_count != len(completed):
            ordered=[completed[c["id"]] for c in queue_cases if c["id"] in completed]
            tmp=results_path.with_suffix(".jsonl.tmp")
            tmp.write_text("".join(json.dumps(r,sort_keys=True,separators=(",",":"))+"\n" for r in ordered))
            tmp.replace(results_path)
            log(f"RESUME normalized duplicate ledger {raw_result_count} -> {len(completed)} unique cases")

    pending = [c for c in queue_cases if c["id"] not in completed]
    total = len(queue_cases)
    start = time.monotonic()
    lock = threading.Lock()
    results = list(completed.values())

    mode = "a" if completed else "w"
    with results_path.open(mode, buffering=1) as out:
        with concurrent.futures.ThreadPoolExecutor(max_workers=args.workers) as ex:
            future_map = {ex.submit(execute_case,c,campaign,False):c for c in pending}
            for fut in concurrent.futures.as_completed(future_map):
                r = fut.result()
                with lock:
                    results.append(r)
                    out.write(json.dumps(r,sort_keys=True,separators=(",",":"))+"\n")
                    done=len(results)
                    elapsed=time.monotonic()-start
                    best=min(results,key=rank_key)
                    progress={
                        "done":done,"total":total,"elapsed_wall_s":elapsed,
                        "best_id":best["id"],"best_terminal_metric_m":best.get("terminal_metric_m"),
                        "best_on_runway":best.get("on_runway",False),"best_touchdown":best.get("touchdown",False),
                    }
                    (campaign/"progress.json").write_text(json.dumps(progress,indent=2)+"\n")
                    if done == 1 or done % 10 == 0 or r.get("on_runway"):
                        log(f"PROGRESS {done}/{total} elapsed={elapsed:.1f}s latest={r['id']} family={r['family']} terminal={float(r.get('terminal_metric_m',1e12)):.0f}m best={best['id']}:{float(best.get('terminal_metric_m',1e12)):.0f}m runway={bool(best.get('on_runway'))}")

    ranked = sorted(results,key=rank_key)
    write_result_csv(campaign/"results.csv",ranked)

    family_best={}
    for r in ranked:
        family_best.setdefault(r["family"],r)
    summary = {
        **manifest,
        "elapsed_wall_s":time.monotonic()-start,
        "completed":len(ranked),
        "on_runway_count":sum(bool(r.get("on_runway")) for r in ranked),
        "touchdown_count":sum(bool(r.get("touchdown")) for r in ranked),
        "best":ranked[:20],
        "family_best":family_best,
    }
    (campaign/"summary.json").write_text(json.dumps(summary,indent=2,sort_keys=True)+"\n")

    # Re-run the strongest cases and retain their full traces for inspection.
    top_cases=[]
    seen=set()
    for r in ranked:
        if r["id"] in seen: continue
        top_cases.append(next(c for c in queue_cases if c["id"]==r["id"]))
        seen.add(r["id"])
        if len(top_cases)>=args.rerun_top: break
    if top_cases:
        log(f"RERUN retaining full traces for top {len(top_cases)} cases")
        with concurrent.futures.ThreadPoolExecutor(max_workers=min(args.workers,len(top_cases))) as ex:
            rr=list(ex.map(lambda c: execute_case(c,campaign,True),top_cases))
        (campaign/"top_reruns.json").write_text(json.dumps(rr,indent=2,sort_keys=True)+"\n")

    log(f"DONE elapsed={summary['elapsed_wall_s']:.1f}s on_runway={summary['on_runway_count']} touchdowns={summary['touchdown_count']}")
    for i,r in enumerate(ranked[:10],1):
        log(f"TOP {i:02d} {r['id']} {r['family']} terminal={float(r.get('terminal_metric_m',1e12)):.1f}m touchdown={bool(r.get('touchdown'))} runway={bool(r.get('on_runway'))} phase={r.get('final_phase','')} :: {r['description']}")
    log(f"RESULTS {campaign/'results.csv'}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
