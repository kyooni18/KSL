#!/usr/bin/env python3
"""Reconstruct a recorded polar return without transport or mutable game state.

UT is preserved.  The kinematic clock is a diagnostic, not a replacement UT:
server clock steps and non-coincident batches can advance UT without advancing
vessel physics.  Original planned vectors are reconstructed from its recorded
geographic trajectory; the plan did not record original forces or controls.
"""
from __future__ import annotations

import argparse
import bisect
import copy
import csv
import hashlib
import json
import math
from pathlib import Path
from typing import Any, Iterable


def merge(target: dict, patch: dict) -> None:
    for key, value in patch.items():
        if isinstance(value, dict):
            if not isinstance(target.get(key), dict):
                target[key] = {}
            merge(target[key], value)
        else:
            target[key] = copy.deepcopy(value)


def load_vehicle(path: Path) -> tuple[dict, list[dict]]:
    header = None
    fields: dict = {}
    samples: list[dict] = []
    previous_sequence = -1
    session = None
    for number, line in enumerate(path.open(), 1):
        record = json.loads(line)
        kind = record.get("recordType")
        if kind == "sessionStart":
            if header is not None:
                raise ValueError("multiple sessions require separate audits")
            header = record
            session = record.get("sessionId")
            continue
        if kind not in ("vehicleKeyframe", "vehicleDelta"):
            continue
        if header is None or record.get("sessionId") != session:
            raise ValueError(f"missing/mismatched session at line {number}")
        sequence = record["recordSequence"]
        if sequence <= previous_sequence:
            raise ValueError(f"non-increasing sequence at line {number}")
        previous_sequence = sequence
        if kind == "vehicleKeyframe":
            fields = copy.deepcopy(record["fields"])
        else:
            if not fields:
                raise ValueError("delta before keyframe")
            merge(fields, record.get("fields", {}))
        samples.append({"ut": record["ut"], "sequence": sequence,
                        "wall": record["wallMonotonicSeconds"],
                        "fields": copy.deepcopy(fields)})
    if header is None or not samples:
        raise ValueError("no complete vehicle stream")
    return header, samples


def add(a, b): return [x + y for x, y in zip(a, b)]
def sub(a, b): return [x - y for x, y in zip(a, b)]
def scale(a, k): return [x * k for x in a]
def dot(a, b): return sum(x * y for x, y in zip(a, b))
def norm(a): return math.sqrt(dot(a, a))
def unit(a): return scale(a, 1.0 / norm(a))
def cross(a, b):
    return [a[1]*b[2]-a[2]*b[1], a[2]*b[0]-a[0]*b[2], a[0]*b[1]-a[1]*b[0]]
def wrap(x): return (x + 180.0) % 360.0 - 180.0


def basis(planet: dict, ut: float):
    north = unit(planet["northAxis"])
    p = planet["primeMeridianAtEpoch"]
    p = unit(sub(p, scale(north, dot(p, north))))
    a = planet["rotationalSpeed"] * (ut - planet["epochUT"])
    prime = add(scale(p, math.cos(a)), scale(cross(north, p), math.sin(a)))
    return prime, cross(north, prime), north


def inertial(planet, latitude, longitude, altitude, ut):
    lat, lon = math.radians(latitude), math.radians(longitude)
    b = basis(planet, ut)
    coeff = (math.cos(lat)*math.cos(lon), math.cos(lat)*math.sin(lon), math.sin(lat))
    return scale([sum(b[j][i]*coeff[j] for j in range(3)) for i in range(3)], planet["radius"]+altitude)


def geometry(planet, site, p, v, ut):
    b = basis(planet, ut)
    fixed = [dot(p, x) for x in b]
    up = unit(p)
    air = sub(v, cross(scale(b[2], planet["rotationalSpeed"]), p))
    north = unit(sub(b[2], scale(up, dot(b[2], up))))
    east = cross(north, up)
    lat = math.degrees(math.asin(max(-1., min(1., dot(up, b[2])))))
    lon = math.degrees(math.atan2(fixed[1], fixed[0]))
    vup = dot(air, up)
    vh = norm(sub(air, scale(up, vup)))
    sitep = inertial(planet, site["latitude"], site["longitude"], 0, ut)
    angle = math.atan2(norm(cross(up, unit(sitep))), dot(up, unit(sitep)))
    return dict(latitude=lat, longitude=lon, altitude=norm(p)-planet["radius"],
                range=planet["radius"]*angle, speed=norm(air), vertical=vup,
                fpa=math.degrees(math.atan2(vup, vh)),
                course=math.degrees(math.atan2(dot(air, east), dot(air, north))) % 360,
                fixed=fixed, fixed_velocity=[dot(air, x) for x in b])


def kinematic_dt(p0, v0, p1, v1):
    velocity = scale(add(v0, v1), .5)
    return dot(sub(p1, p0), velocity) / max(dot(velocity, velocity), 1e-30)


def periapsis(p, v, planet):
    mu = planet["gravitationalParameter"]
    h2 = dot(cross(p, v), cross(p, v))
    energy = .5*dot(v, v)-mu/norm(p)
    e = math.sqrt(max(0., 1.+2.*energy*h2/(mu*mu)))
    return h2/(mu*(1.+e))-planet["radius"]


def derivative(times, positions, index):
    """Five-point local Lagrange derivative, including nonuniform steps."""
    start = max(0, min(index-2, len(times)-5))
    ids = list(range(start, min(start+5, len(times))))
    x = [times[i]-times[index] for i in ids]
    result = [0., 0., 0.]
    for j, i in enumerate(ids):
        denominator = math.prod(x[j]-x[k] for k in range(len(ids)) if k != j)
        numerator = sum(math.prod(-x[k] for k in range(len(ids)) if k not in (j, m))
                        for m in range(len(ids)) if m != j)
        result = add(result, scale(positions[i], numerator/denominator))
    return result


def interpolate(a: dict, b: dict, fraction: float):
    out = {}
    for key in a.keys() & b.keys():
        x, y = a[key], b[key]
        if isinstance(x, (int, float)) and not isinstance(x, bool) and isinstance(y, (int, float)):
            d = wrap(y-x) if key in ("longitude", "course", "heading", "bank", "cmd_bank", "cmd_heading") else y-x
            out[key] = x+fraction*d
        elif isinstance(x, list) and isinstance(y, list) and len(x) == len(y) and all(isinstance(z, (int, float)) for z in x+y):
            out[key] = add(scale(x, 1.-fraction), scale(y, fraction))
        else:
            out[key] = x if fraction < .5 else y
    return out


def crossing(rows: list[dict], altitude: float):
    for a, b in zip(rows, rows[1:]):
        if a["altitude"] >= altitude > b["altitude"]:
            f = (a["altitude"]-altitude)/(a["altitude"]-b["altitude"])
            out = interpolate(a, b, f)
            out["bracket_dt"] = b["ut"]-a["ut"]
            out["bracket_clock_error"] = b.get("clock_error_step", 0.)
            return out
    return None


def at_time(rows, times, ut):
    i = bisect.bisect_right(times, ut)-1
    if i < 0 or i >= len(rows)-1 or times[i+1] <= times[i]:
        return None
    return interpolate(rows[i], rows[i+1], (ut-times[i])/(times[i+1]-times[i]))


def csv_write(path: Path, rows: Iterable[dict]):
    rows = list(rows)
    if not rows:
        return
    keys = list(dict.fromkeys(k for row in rows for k in row))
    with path.open("w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=keys)
        writer.writeheader()
        for row in rows:
            writer.writerow({k: json.dumps(v, separators=(",", ":")) if isinstance(v, (dict, list)) else v for k, v in row.items()})


def sha(path: Path): return hashlib.sha256(path.read_bytes()).hexdigest()


def classify_observed_outcome(observed: list[dict], run_status: dict) -> dict:
    """Classify only witnessed milestones and durable mission state, never a forecast."""
    witnessed = {r["phase"] for r in observed}
    last_phase = observed[-1]["phase"] if observed else None
    state = run_status.get("state")
    milestones = {"mm305Observed": "TAEM" in witnessed,
                  "finalObserved": "Final Approach" in witnessed,
                  "touchdownPhaseObserved": "Touchdown" in witnessed,
                  "rolloutPhaseObserved": "Rollout" in witnessed,
                  "completePhaseObserved": "Complete" in witnessed}
    if state == "aborted":
        description = f"mission aborted at {last_phase}; no verified touchdown/rollout"
    elif state == "failed":
        description = f"mission failed at {last_phase}; no verified touchdown/rollout"
    elif state == "completed" and all(milestones[k] for k in
                                    ("touchdownPhaseObserved", "rolloutPhaseObserved", "completePhaseObserved")):
        description = "mission completed; touchdown, rollout and Complete phases observed"
    else:
        description = (f"incomplete/unverified recording ending at {last_phase}; mission status "
                       f"{state or 'unavailable'}; no verified terminal landing")
    return dict(description=description, statusState=state, lastObservedPhase=last_phase,
                lastObservedUT=observed[-1]["ut"] if observed else None,
                **milestones)


def audit(vehicle: Path, planner: Path, plan_path: Path, output: Path, physics_session: str | None = None):
    header, samples = load_vehicle(vehicle)
    planet, cfg = header["planet"], header["configuration"]
    site = cfg["site"]
    plan = json.loads(plan_path.read_text())
    run_config_path = plan_path.parent/"config.json"
    run_config = json.loads(run_config_path.read_text())
    model_profile = run_config["configuration"]["landing"]["modelProfile"]
    ignition = plan["burnUT"]-plan["estimatedBurnDuration"]*.5
    planning_sample = copy.deepcopy(min(samples, key=lambda r: abs(r["ut"]-plan["createdUT"])))
    orbital_sample = copy.deepcopy(samples[0])
    ignition_sample = copy.deepcopy(min(samples, key=lambda r: abs(r["ut"]-ignition)))
    samples = [r for r in samples if r["ut"] >= ignition-1.]
    if any(b["ut"] < a["ut"] for a, b in zip(samples, samples[1:])):
        raise ValueError("UT rewind: split flight at rewind rather than sorting it away")
    observed = []
    clock = 0.
    previous = None
    for raw in samples:
        f = raw["fields"]
        p, v = f["position"]["inertial"], f["motion"]["velocity"]
        row = geometry(planet, site, p, v, raw["ut"])
        att, aero, command = f["attitude"], f["aero"], f["command"]
        row.update(ut=raw["ut"], sequence=raw["sequence"], phase=f["state"]["phase"],
                   p=p, v=v, wall=raw["wall"], recorded_speed=f["motion"]["trueAirSpeed"],
                   aoa=att["angleOfAttack"], bank=att["roll"], beta=att["sideslip"], heading=att["heading"],
                   cmd_aoa=command["targetAoA"], cmd_bank=command["targetRoll"], cmd_heading=command["targetHeading"],
                   cmd_throttle=command["targetThrottle"], q=aero["dynamicPressure"], mach=aero["mach"],
                   mass=f["vehicle"]["mass"], thrust=f["vehicle"]["currentThrust"],
                   lift=aero["liftForce"], drag=aero["dragForce"], lift_vector=aero["liftVector"], drag_vector=aero["dragVector"],
                   applied=f.get("appliedControl", {}),
                   vertical_lift=dot(aero["liftVector"], unit(p))/f["vehicle"]["mass"],
                   vertical_drag=dot(aero["dragVector"], unit(p))/f["vehicle"]["mass"],
                   physics_dt=0., clock_error_step=0.)
        if previous is not None:
            rawdt = row["ut"]-previous["ut"]
            kin = kinematic_dt(previous["p"], previous["v"], p, v)
            row["physics_dt"] = kin
            row["clock_error_step"] = rawdt-kin
            clock += rawdt-kin
        row["clock_excess"] = clock
        row["kinematic_ut"] = row["ut"]-clock
        row["frame_closure_m"] = norm(sub(p, inertial(planet, f["position"]["latitude"], f["position"]["longitude"], f["position"]["altitude"], raw["ut"])))
        observed.append(row)
        previous = row
    original = plan["trajectory"]
    times = [r["ut"] for r in original]
    positions = [inertial(planet, r["latitude"], r["longitude"], r["altitude"], r["ut"]) for r in original]
    predicted = []
    for i, r in enumerate(original):
        v = derivative(times, positions, i)
        g = geometry(planet, site, positions[i], v, r["ut"])
        g.update(ut=r["ut"], p=positions[i], v=v, recorded_speed=r["speed"], phase=r["phase"])
        predicted.append(g)
    output.mkdir(parents=True, exist_ok=True)
    csv_write(output/"observed.csv", observed)
    csv_write(output/"predicted-reconstructed.csv", predicted)
    csv_write(output/"clock-discontinuities.csv", [r for r in observed if abs(r["clock_error_step"]) > .5])
    altitudes = [99000, 95000, 90000, 80000, 71000, 70000, 69500, 69000, 68000, 65000, 60000, 55000, 50000, 45000, 42000, 40000, 35000, 30000, 28000, 27500, 25000, 22000, 18000, 15000]
    matches = []
    for h in altitudes:
        a, p = crossing(observed, h), crossing(predicted, h)
        if a is None or p is None:
            continue
        r = {"altitude": h, "delay_ut_s": a["ut"]-p["ut"], "clock_excess_s": a["clock_excess"],
             "delay_minus_clock_s": a["ut"]-p["ut"]-a["clock_excess"],
             "position_difference_m": norm(sub(a["p"], p["p"]))}
        for prefix, value in (("actual", a), ("predicted", p)):
            for key in ("ut", "range", "recorded_speed", "vertical", "fpa", "course", "p", "v", "fixed", "fixed_velocity", "bracket_dt", "bracket_clock_error", "q", "aoa", "cmd_aoa", "bank", "cmd_bank", "lift", "drag", "vertical_lift", "vertical_drag"):
                if key in value:
                    r[prefix+"_"+key] = value[key]
        matches.append(r)
    csv_write(output/"same-altitude.csv", matches)
    observed_times = [r["ut"] for r in observed]
    equal_ut = []
    for p in predicted:
        a = at_time(observed, observed_times, p["ut"])
        if a:
            equal_ut.append(dict(ut=p["ut"], actual_altitude=a["altitude"], predicted_altitude=p["altitude"],
                                 actual_range=a["range"], predicted_range=p["range"],
                                 position_error_m=norm(sub(a["p"], p["p"])), velocity_error_mps=norm(sub(a["v"], p["v"])),
                                 actual_p=a["p"], predicted_p=p["p"], actual_fixed=a["fixed"], predicted_fixed=p["fixed"],
                                 clock_excess_s=a["clock_excess"]))
    csv_write(output/"same-ut.csv", equal_ut)
    phases = []
    previous_phase = None
    for row in observed:
        if row["phase"] != previous_phase:
            phases.append({k: row[k] for k in ("phase", "ut", "sequence", "altitude", "range", "recorded_speed", "course", "clock_excess")})
            previous_phase = row["phase"]
    cutoff_i = next(i for i, r in enumerate(observed) if r["phase"] == "Entry Interface" and r["thrust"] < 1.)
    cutoff = observed[cutoff_i]
    cutoff_raw = samples[cutoff_i]
    onset = {str(threshold): next(r["ut"] for r in observed if r["phase"] == "Deorbit Burn" and r["thrust"] > threshold) for threshold in (1, 100, 1000)}
    pulses = []
    group = []
    for row in observed[cutoff_i+1:]:
        if row["altitude"] < 70000 and row["thrust"] > 100 and row["cmd_throttle"] == 0:
            group.append(row)
        elif group:
            pulses.append(group)
            group = []
    if group:
        pulses.append(group)
    pulse_stats = []
    for group in pulses:
        pulse_stats.append(dict(start_ut=group[0]["ut"], end_ut=group[-1]["ut"], samples=len(group),
                                altitude=group[0]["altitude"], maximum_thrust_n=max(r["thrust"] for r in group),
                                physical_time_s=sum(max(0., r["physics_dt"]) for r in group),
                                impulse_over_mass_mps=sum(r["thrust"]/r["mass"]*max(0., r["physics_dt"]) for r in group),
                                observed_mass_change_kg=group[0]["mass"]-group[-1]["mass"]))
    planner_initial = next(r for r in map(json.loads, planner.open()) if r.get("recordType") == "plannerSample")
    selected = {cutoff_raw["sequence"]: dict(label="postburn", **cutoff_raw)}
    for h in (70000, 69000, 65000, 60000, 50000, 45000, 40000, 35000, 31000, 30000, 29000, 28000, 27500, 26000, 25000, 22000, 18000, 15000):
        if crossing(observed[cutoff_i:], h) is None:
            continue  # Never label the last sample as a gate the flight did not cross.
        i = min(range(cutoff_i, len(observed)), key=lambda j: abs(observed[j]["altitude"]-h))
        selected[samples[i]["sequence"]] = dict(label=f"h{h}", **samples[i])
    i = next((i for i, r in enumerate(observed) if r["phase"] == "TAEM"), None)
    if i is not None:
        selected[samples[i]["sequence"]] = dict(label="mm305-first", **samples[i])
    frozen = []
    for raw in sorted(selected.values(), key=lambda r: r["ut"]):
        frozen.append({"label": raw["label"], "ut": raw["ut"], "sequence": raw["sequence"],
                       "fields": {k: copy.deepcopy(raw["fields"][k]) for k in ("position", "motion", "attitude", "aero", "vehicle", "guidance", "physics", "command", "state")}})
    manifest = {"runConfiguration": {"path": str(run_config_path), "sha256": sha(run_config_path)}, "vehicle": {"path": str(vehicle), "sha256": sha(vehicle)},
                "planner": {"path": str(planner), "sha256": sha(planner)},
                "plan": {"path": str(plan_path), "sha256": sha(plan_path)}}
    fixture = {"schemaVersion": 1, "sources": manifest, "planet": planet, "configuration": cfg,
               "aeroEnvelope": planner_initial["aeroEnvelope"], "calibration": planner_initial["calibration"],
               "modelProfile": model_profile, "modelIdentity": planner_initial["modelIdentity"], "physicsContext": header["physicsContext"], "orbitalSample": orbital_sample, "planningSample": planning_sample, "ignitionSample": ignition_sample, "physicsSessionId": physics_session,
               "planParameters": {k: v for k, v in plan.items() if k != "trajectory"},
               "plannedPostburn": {"ut": predicted[0]["ut"], "position": predicted[0]["p"], "velocity": predicted[0]["v"],
                                   "mass": cutoff["mass"], "velocityProvenance": "five-point derivative of original planned geographic positions", "massProvenance": "measured cutoff; original planned mass not recorded"},
               "samples": frozen}
    (output/"latest-fixture.json").write_text(json.dumps(fixture, indent=2)+"\n")
    columns = ["ut", "physics_dt", "mass", "aoa", "beta", "bank", "q", "mach", "cmd_aoa", "cmd_bank", "thrust"]
    with (output/"recorded-dynamics.csv").open("w", newline="") as f:
        w = csv.writer(f)
        w.writerow(columns+[a+str(i) for a in ("p", "v", "lift", "drag") for i in range(3)])
        for r in observed[cutoff_i:]:
            w.writerow([r[c] if r[c] is not None else math.nan for c in columns]+r["p"]+r["v"]+r["lift_vector"]+r["drag_vector"])
    status_path = plan_path.parent/"status.json"
    run_status = json.loads(status_path.read_text()) if status_path.is_file() else {}
    outcome = classify_observed_outcome(observed, run_status)
    summary = {"sources": manifest, "planet_frame_closure_max_m": max(r["frame_closure_m"] for r in observed),
               "sample_count": len(observed), "phases": phases, "outcome": outcome["description"],
               "observed_outcome": outcome,
               "burn": {"planned_ignition_ut": ignition, "planned_midpoint_ut": plan["burnUT"],
                        "planned_duration_s": plan["estimatedBurnDuration"], "thrust_onset_ut_by_threshold_n": onset,
                        "first_zero_thrust_after_entry_interface_ut": cutoff["ut"],
                        "cutoff_late_s": cutoff["ut"]-ignition-plan["estimatedBurnDuration"],
                        "planned_periapsis_m": plan["predictedPostBurnPeriapsisAltitude"],
                        "measured_cutoff_osculating_periapsis_m": periapsis(cutoff["p"], cutoff["v"], planet)},
               "atmospheric_zero_command_thrust_episodes": pulse_stats,
               "same_altitude": matches,
               "clock": {"positive_steps_over_half_second": sum(r["clock_error_step"]>.5 for r in observed),
                         "negative_steps_over_half_second": sum(r["clock_error_step"]<-.5 for r in observed),
                         "cumulative_excess_at_last_sample_s": observed[-1]["clock_excess"],
                         "definition": "sum(raw UT delta - displacement projected on mean inertial velocity / mean velocity squared); diagnostic, raw UT unchanged"},
               "limitations": ["Original plan omitted force/control samples; reconstructed derivatives are not original telemetry.",
                               "Interpolated UT crossings can span non-coincident state batches; bracket discrepancy is retained.",
                               "Frame closure checks the recorded frame transform, not independently measured planet rotation.",
                               "Thrust actor is unknown: the original stream did not record measured global or per-engine throttle."]}
    (output/"audit-summary.json").write_text(json.dumps(summary, indent=2, allow_nan=False)+"\n")
    print(json.dumps({k: summary[k] for k in ("sample_count", "burn", "clock", "atmospheric_zero_command_thrust_episodes", "planet_frame_closure_max_m")}, indent=2))
    print("altitude_m delay_UT_s clock_excess_s residual_s actual_range_km predicted_range_km")
    for r in matches:
        print(f"{r['altitude']:8.0f} {r['delay_ut_s']:9.3f} {r['clock_excess_s']:9.3f} {r['delay_minus_clock_s']:9.3f} {r['actual_range']/1000:10.3f} {r['predicted_range']/1000:10.3f}")


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--vehicle", type=Path, required=True)
    p.add_argument("--planner", type=Path, required=True)
    p.add_argument("--plan", type=Path, required=True)
    p.add_argument("--output", type=Path, required=True)
    p.add_argument("--physics-session", help="Exact historical read-only archive session identity")
    a = p.parse_args()
    audit(a.vehicle, a.planner, a.plan, a.output, a.physics_session)


if __name__ == "__main__":
    main()
