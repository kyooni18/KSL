#!/usr/bin/env python3
"""Offline predictor-fidelity laboratory for KSPShuttleLander.

Scores persisted closed-loop shadow-guidance trajectories against later vehicle
telemetry, runs a production-prior recorded-control open-loop physics replay,
and performs leakage-safe leave-one-flight-out validation of the aero store.
"""
from __future__ import annotations

import argparse
import bisect
import json
import math
import sqlite3
import statistics
import subprocess
from collections import defaultdict
from pathlib import Path
from typing import Any, Iterable

from clanding_layout import build_artifact as clanding_build_artifact, source_root as clanding_source_root

DEFAULT_HORIZONS = (10.0, 30.0, 60.0, 120.0, 180.0, 300.0)


def finite(value: Any) -> bool:
    return isinstance(value, (int, float)) and math.isfinite(float(value))


def percentile(values: Iterable[float], p: float) -> float | None:
    data = sorted(float(v) for v in values if finite(v))
    if not data:
        return None
    if len(data) == 1:
        return data[0]
    x = max(0.0, min(1.0, p)) * (len(data) - 1)
    lo, hi = int(math.floor(x)), int(math.ceil(x))
    if lo == hi:
        return data[lo]
    f = x - lo
    return data[lo] * (1.0 - f) + data[hi] * f


def summarize(values: list[float]) -> dict[str, Any]:
    vals = [float(v) for v in values if finite(v)]
    return {
        "count": len(vals),
        "p50": percentile(vals, 0.50),
        "p95": percentile(vals, 0.95),
        "max": max(vals) if vals else None,
        "mean": statistics.fmean(vals) if vals else None,
    }


def interpolate_track(samples: list[tuple[float, float, float, float, float]], uts: list[float], ut: float):
    if len(samples) < 2 or ut < samples[0][0] or ut > samples[-1][0]:
        return None
    ix = bisect.bisect_left(uts, ut)
    if ix < len(samples) and abs(samples[ix][0] - ut) < 1e-9:
        return samples[ix]
    if ix <= 0 or ix >= len(samples):
        return None
    a, b = samples[ix - 1], samples[ix]
    dt = b[0] - a[0]
    if dt <= 0 or dt > 30.0:
        return None
    f = (ut - a[0]) / dt
    dl = (b[2] - a[2] + 180.0) % 360.0 - 180.0
    return (ut, a[1] + (b[1] - a[1]) * f, a[2] + dl * f, a[3] + (b[3] - a[3]) * f, a[4] + (b[4] - a[4]) * f)


def great_circle_m(a_lat: float, a_lon: float, b_lat: float, b_lon: float, radius: float) -> float:
    p1, p2 = math.radians(a_lat), math.radians(b_lat)
    dp = p2 - p1
    dl = math.radians((b_lon - a_lon + 180.0) % 360.0 - 180.0)
    h = math.sin(dp / 2.0) ** 2 + math.cos(p1) * math.cos(p2) * math.sin(dl / 2.0) ** 2
    return 2.0 * radius * math.asin(min(1.0, math.sqrt(max(0.0, h))))


def trajectory_array_points(points: Any) -> list[tuple[float, float, float, float, float]]:
    out = []
    for p in points or []:
        if not isinstance(p, dict):
            continue
        values = (p.get("ut"), p.get("latitude"), p.get("longitude"), p.get("altitude"), p.get("speed"))
        if all(finite(v) for v in values):
            out.append(tuple(float(v) for v in values))
    out.sort(key=lambda x: x[0])
    return out


def trajectory_points(record: dict[str, Any]) -> list[tuple[float, float, float, float, float]]:
    return trajectory_array_points(record.get("points") or [])


def dedup_track(actual: list[tuple[float, float, float, float, float]]):
    actual.sort(key=lambda x: x[0])
    dedup = []
    for sample in actual:
        if dedup and abs(sample[0] - dedup[-1][0]) < 1e-9:
            dedup[-1] = sample
        else:
            dedup.append(sample)
    return dedup


def merge_fields(base: dict[str, Any], patch: dict[str, Any]) -> None:
    for key, value in patch.items():
        if isinstance(value, dict) and isinstance(base.get(key), dict):
            merge_fields(base[key], value)
        elif isinstance(value, dict):
            base[key] = dict(value)
        else:
            base[key] = value


def load_vehicle_stream(path: Path) -> dict[str, Any]:
    actual: list[tuple[float, float, float, float, float]] = []
    planet_radius = 600000.0
    session_id = None
    fields: dict[str, Any] = {}
    with path.open("r", encoding="utf-8") as handle:
        for raw in handle:
            try:
                record = json.loads(raw)
            except json.JSONDecodeError:
                continue
            kind = record.get("recordType")
            if kind == "sessionStart":
                session_id = record.get("sessionId") or session_id
                radius = (record.get("planet") or {}).get("radius")
                if finite(radius):
                    planet_radius = float(radius)
                continue
            if kind == "vehicleKeyframe":
                fields = dict(record.get("fields") or {})
            elif kind == "vehicleDelta":
                merge_fields(fields, record.get("fields") or {})
            else:
                continue
            position, motion = fields.get("position") or {}, fields.get("motion") or {}
            values = (record.get("ut"), position.get("latitude"), position.get("longitude"),
                      position.get("altitude"), motion.get("trueAirSpeed"))
            if all(finite(v) for v in values):
                actual.append(tuple(float(v) for v in values))
    return {"radius": planet_radius, "actual": dedup_track(actual), "sessionId": session_id}


def load_planner_stream(path: Path) -> dict[str, Any]:
    forecasts: dict[str, list[dict[str, Any]]] = defaultdict(list)
    session_id = None
    with path.open("r", encoding="utf-8") as handle:
        for raw in handle:
            try:
                record = json.loads(raw)
            except json.JSONDecodeError:
                continue
            if record.get("recordType") == "sessionStart":
                session_id = record.get("sessionId") or session_id
                continue
            if record.get("recordType") != "plannerSample" or not finite(record.get("ut")):
                continue
            origin_ut = float(record["ut"])
            for field, stream in (("publishedPrediction", "plan"), ("rawPrediction", "planRaw")):
                points = trajectory_array_points(record.get(field) or [])
                if len(points) >= 2:
                    forecasts[stream].append({"originUT": origin_ut, "points": points})
    return {"forecasts": forecasts, "sessionId": session_id}


def load_legacy_flight_log(path: Path) -> dict[str, Any]:
    snapshots: dict[int, dict[str, Any]] = {}
    actual: list[tuple[float, float, float, float, float]] = []
    forecasts: dict[str, list[dict[str, Any]]] = defaultdict(list)
    planet_radius = 600000.0
    pending_keyframes: list[dict[str, Any]] = []
    with path.open("r", encoding="utf-8") as handle:
        for raw in handle:
            try:
                record = json.loads(raw)
            except json.JSONDecodeError:
                continue
            kind = record.get("recordType")
            if kind == "sessionStart":
                radius = (record.get("planet") or {}).get("radius")
                if finite(radius):
                    planet_radius = float(radius)
            elif kind == "snapshot":
                seq = record.get("logSequence")
                telemetry = record.get("telemetry") or {}
                ut, lat, lon = telemetry.get("ut"), telemetry.get("latitude"), telemetry.get("longitude")
                alt, speed = telemetry.get("meanAltitude"), telemetry.get("trueAirSpeed")
                if isinstance(seq, int) and finite(ut):
                    snapshots[seq] = record
                if all(finite(v) for v in (ut, lat, lon, alt, speed)):
                    actual.append((float(ut), float(lat), float(lon), float(alt), float(speed)))
            elif kind == "trajectoryKeyframe":
                pending_keyframes.append(record)

    for record in pending_keyframes:
        stream = str(record.get("stream") or "")
        if stream not in {"plan", "planRaw", "rawPlan", "plan-raw"}:
            continue
        seq = record.get("logSequence")
        origin = snapshots.get(seq) if isinstance(seq, int) else None
        origin_ut = ((origin or {}).get("telemetry") or {}).get("ut")
        points = trajectory_points(record)
        if finite(origin_ut) and len(points) >= 2:
            forecasts[stream].append({"originUT": float(origin_ut), "points": points})
    return {"radius": planet_radius, "actual": dedup_track(actual), "forecasts": forecasts,
            "format": "legacy", "vehiclePath": str(path), "plannerPath": str(path), "sessionId": None}


def load_flight_log(path: Path) -> dict[str, Any]:
    name = path.name
    if name.endswith("-planner.jsonl") or name.endswith("-vehicle.jsonl"):
        planner_path = path if name.endswith("-planner.jsonl") else path.with_name(name.removesuffix("-vehicle.jsonl") + "-planner.jsonl")
        vehicle_path = path if name.endswith("-vehicle.jsonl") else path.with_name(name.removesuffix("-planner.jsonl") + "-vehicle.jsonl")
        if planner_path.exists() and vehicle_path.exists():
            vehicle = load_vehicle_stream(vehicle_path)
            planner = load_planner_stream(planner_path)
            session_match = not vehicle["sessionId"] or not planner["sessionId"] or vehicle["sessionId"] == planner["sessionId"]
            return {"radius": vehicle["radius"], "actual": vehicle["actual"], "forecasts": planner["forecasts"],
                    "format": "split", "vehiclePath": str(vehicle_path), "plannerPath": str(planner_path),
                    "sessionId": vehicle["sessionId"] or planner["sessionId"], "sessionMatch": session_match}
    return load_legacy_flight_log(path)


def score_forecast_stream(records: list[dict[str, Any]], actual: list[tuple[float, float, float, float, float]], radius: float, horizons: tuple[float, ...]) -> dict[str, Any]:
    actual_uts = [s[0] for s in actual]
    metrics = {h: {"horizontal_m": [], "altitude_m": [], "speed_mps": [], "position3d_m": []} for h in horizons}
    last_origin = -math.inf
    used_records = 0
    for record in sorted(records, key=lambda r: r["originUT"]):
        origin = record["originUT"]
        if origin - last_origin < 1.0:
            continue
        last_origin = origin
        points = record["points"]
        point_uts = [p[0] for p in points]
        used = False
        for horizon in horizons:
            target_ut = origin + horizon
            pred = interpolate_track(points, point_uts, target_ut)
            truth = interpolate_track(actual, actual_uts, target_ut)
            if pred is None or truth is None:
                continue
            horizontal = great_circle_m(pred[1], pred[2], truth[1], truth[2], radius)
            altitude, speed = abs(pred[3] - truth[3]), abs(pred[4] - truth[4])
            metrics[horizon]["horizontal_m"].append(horizontal)
            metrics[horizon]["altitude_m"].append(altitude)
            metrics[horizon]["speed_mps"].append(speed)
            metrics[horizon]["position3d_m"].append(math.hypot(horizontal, altitude))
            used = True
        used_records += int(used)
    return {
        "forecastOrigins": len(records),
        "independentOriginsUsed": used_records,
        "horizons": {str(int(h) if h.is_integer() else h): {name: summarize(vals) for name, vals in metric.items()} for h, metric in metrics.items()},
    }


def score_log(path: Path, horizons: tuple[float, ...]) -> dict[str, Any]:
    data = load_flight_log(path)
    streams = {name: score_forecast_stream(records, data["actual"], data["radius"], horizons) for name, records in data["forecasts"].items()}
    raw_names = [name for name in streams if name != "plan"]
    return {
        "path": str(path), "format": data.get("format", "legacy"),
        "vehiclePath": data.get("vehiclePath"), "plannerPath": data.get("plannerPath"),
        "sessionId": data.get("sessionId"), "sessionMatch": data.get("sessionMatch", True),
        "actualSamples": len(data["actual"]),
        "publishedStreamPresent": "plan" in streams,
        "rawStreamPresent": bool(raw_names),
        "rawStreams": raw_names,
        "streams": streams,
        "instrumentationGap": None if raw_names else "No raw predictor trajectory stream is persisted; raw-vs-stabilized forecast error cannot yet be separated.",
    }


def aero_rows(db_path: Path) -> list[dict[str, Any]]:
    con = sqlite3.connect(f"file:{db_path.resolve()}?mode=ro", uri=True)
    con.row_factory = sqlite3.Row
    try:
        rows = con.execute("""
            SELECT a.id,a.structure_id,a.environment_id,a.session_id,a.timeline_epoch,
                   COALESCE(s.flight_key,a.session_id) AS flight_key,
                   a.q,a.mach,a.aoa,a.beta,a.gear,a.brakes,a.airbrakes,
                   a.lift_x,a.lift_y,a.lift_z,a.drag_x,a.drag_y,a.drag_z,
                   COALESCE(q.quality,0.5) AS quality,COALESCE(q.eligible,1) AS eligible
              FROM aero_observations a
              LEFT JOIN physics_sessions s ON s.session_id=a.session_id
              LEFT JOIN aero_observation_quality q ON q.observation_id=a.id
             WHERE a.q>1.0 ORDER BY a.id
        """).fetchall()
    finally:
        con.close()
    out = []
    for r in rows:
        if not r["eligible"]:
            continue
        q = float(r["q"])
        lift = math.sqrt(float(r["lift_x"]) ** 2 + float(r["lift_y"]) ** 2 + float(r["lift_z"]) ** 2) / q
        drag = math.sqrt(float(r["drag_x"]) ** 2 + float(r["drag_y"]) ** 2 + float(r["drag_z"]) ** 2) / q
        if not (finite(lift) and finite(drag) and lift > 1e-12 and drag > 1e-12):
            continue
        item = dict(r)
        item.update(lift_per_q=lift, drag_per_q=drag,
                    flight_key=str(r["flight_key"]), source=f"{r['flight_key']}#{r['timeline_epoch']}")
        out.append(item)
    return out


def neighbor_distance(target: dict[str, Any], sample: dict[str, Any], policy: str) -> float | None:
    if (target["structure_id"], target["environment_id"], target["gear"], target["brakes"], target["airbrakes"]) != (sample["structure_id"], sample["environment_id"], sample["gear"], sample["brakes"], sample["airbrakes"]):
        return None
    dm = (float(target["mach"]) - float(sample["mach"])) / 0.5
    da = (float(target["aoa"]) - float(sample["aoa"])) / 10.0
    db = (float(target["beta"]) - float(sample["beta"])) / 10.0
    base = dm * dm + da * da + db * db
    if base >= 1.0:
        return None
    dq = math.log(max(float(target["q"]), 1e-9) / max(float(sample["q"]), 1e-9), 2.0)
    if policy == "hard_q":
        d2 = base + dq * dq
        return d2 if d2 < 1.0 else None
    if policy == "soft_q":
        if abs(dq) > 4.0:
            return None
        return base + (0.25 * dq) ** 2
    if policy == "no_q":
        return base
    raise ValueError(policy)


def predict_aero(target: dict[str, Any], pool: list[dict[str, Any]], policy: str, k: int = 16):
    neighbors = []
    for sample in pool:
        # Match the certified runtime prior: never validate against another
        # quickload/session from the same stable KSP launch identity.
        if sample["flight_key"] == target["flight_key"]:
            continue
        distance = neighbor_distance(target, sample, policy)
        if distance is not None:
            neighbors.append((distance, sample))
    neighbors.sort(key=lambda x: x[0])
    neighbors = neighbors[:k]
    sources = {row["flight_key"] for _, row in neighbors}
    if len(neighbors) < 4 or len(sources) < 2:
        return None
    lift_sum = drag_sum = weight_sum = 0.0
    for distance, row in neighbors:
        weight = max(0.05, min(1.0, float(row["quality"]))) / (0.04 + distance)
        lift_sum += weight * math.log(float(row["lift_per_q"]))
        drag_sum += weight * math.log(float(row["drag_per_q"]))
        weight_sum += weight
    return math.exp(lift_sum / weight_sum), math.exp(drag_sum / weight_sum), len(sources)


def cross_validate_aero(db_path: Path, max_eval: int) -> dict[str, Any]:
    rows = aero_rows(db_path)
    pools: dict[tuple[Any, ...], list[dict[str, Any]]] = defaultdict(list)
    for row in rows:
        pools[(row["structure_id"], row["environment_id"], row["gear"], row["brakes"], row["airbrakes"])].append(row)
    if len(rows) > max_eval:
        stride = len(rows) / max_eval
        eval_rows = [rows[min(len(rows) - 1, int(i * stride))] for i in range(max_eval)]
    else:
        eval_rows = rows
    results = {policy: {"lift": [], "drag": [], "combined": [], "sources": []} for policy in ("hard_q", "soft_q", "no_q")}
    for target in eval_rows:
        pool = pools[(target["structure_id"], target["environment_id"], target["gear"], target["brakes"], target["airbrakes"])]
        for policy, result in results.items():
            pred = predict_aero(target, pool, policy)
            if pred is None:
                continue
            lift, drag, source_count = pred
            le = abs(lift - target["lift_per_q"]) / target["lift_per_q"]
            de = abs(drag - target["drag_per_q"]) / target["drag_per_q"]
            result["lift"].append(le); result["drag"].append(de)
            result["combined"].append(math.hypot(le, de) / math.sqrt(2.0)); result["sources"].append(float(source_count))
    report = {}
    for policy, result in results.items():
        n = len(result["combined"]); attempts = len(eval_rows)
        report[policy] = {
            "attempts": attempts, "predictions": n, "coverage": n / attempts if attempts else 0.0,
            "liftRelativeError": summarize(result["lift"]), "dragRelativeError": summarize(result["drag"]),
            "combinedRelativeError": summarize(result["combined"]), "independentSources": summarize(result["sources"]),
        }
    def objective(policy: str) -> float:
        p95 = report[policy]["combinedRelativeError"]["p95"]
        return math.inf if p95 is None else float(p95) + 0.20 * (1.0 - float(report[policy]["coverage"]))
    winner = min(report, key=objective) if report else None
    recommendation = {"hard_q": "keep_hard_q_locality", "soft_q": "replace_hard_q_gate_with_soft_q_weight", "no_q": "remove_q_from_hard_locality_kernel"}.get(winner)
    qv, mv, av, bv = ([float(r[k]) for r in rows] for k in ("q", "mach", "aoa", "beta"))
    return {
        "database": str(db_path), "eligibleRows": len(rows),
        "independentFlightKeys": len({r["flight_key"] for r in rows}),
        "independentFlightEpochs": len({r["source"] for r in rows}),
        "ranges": {"q": [min(qv), max(qv)] if qv else None, "mach": [min(mv), max(mv)] if mv else None, "aoaDeg": [min(av), max(av)] if av else None, "betaDeg": [min(bv), max(bv)] if bv else None},
        "policies": report, "selectionObjective": "p95 combined normalized-force relative error + 0.20*(1-coverage)",
        "recommendedQLocality": recommendation, "recommendedPolicyEvidence": report.get(winner or "", {}),
        "leakageGuard": "Each validation target excludes every observation sharing its stable flight_key, including other sessions/quickloads from that launch.",
    }


def parse_horizons(text: str) -> tuple[float, ...]:
    values = tuple(sorted({float(x.strip()) for x in text.split(",") if x.strip()}))
    if not values or any(v <= 0 for v in values):
        raise argparse.ArgumentTypeError("horizons must be positive comma-separated seconds")
    return values


def recorded_control_physics_replay(root: Path, db: Path, session: str) -> dict[str, Any]:
    target = clanding_build_artifact(root, "predictor_recorded_control_replay")
    build = subprocess.run(
        ["make", "-C", str(clanding_source_root(root)), "build/predictor_recorded_control_replay"],
        text=True, capture_output=True,
    )
    if build.returncode != 0:
        return {"error": "replay helper build failed", "stderr": build.stderr[-4000:]}
    run = subprocess.run([str(target), str(db), session], cwd=root, text=True, capture_output=True)
    if run.returncode != 0:
        return {"error": "recorded-control replay failed", "stderr": run.stderr[-4000:], "physicsSession": session}
    try:
        return json.loads(run.stdout)
    except json.JSONDecodeError as exc:
        return {"error": f"invalid replay JSON: {exc}", "stdout": run.stdout[-4000:], "physicsSession": session}


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--root", type=Path, default=Path(__file__).resolve().parents[1])
    parser.add_argument("--log", type=Path, action="append")
    parser.add_argument("--db", type=Path)
    parser.add_argument("--horizons", type=parse_horizons, default=DEFAULT_HORIZONS)
    parser.add_argument("--max-aero-eval", type=int, default=1800)
    parser.add_argument("--output", type=Path)
    parser.add_argument("--physics-session", help="Physics-store session to replay with recorded states while excluding its whole flight_key from the certified prior.")
    args = parser.parse_args()
    root = args.root.resolve()
    logs = args.log or sorted((root / "FlightLogs").glob("*.jsonl"), key=lambda p: p.stat().st_mtime, reverse=True)[:1]
    logs = [p if p.is_absolute() else root / p for p in logs]
    db = args.db or root / "Runtime/Physics/observations.sqlite3"
    if not db.is_absolute(): db = root / db
    closed_loop = [score_log(p, args.horizons) for p in logs if p.exists()]
    result = {
        "schemaVersion": 2, "horizonsSeconds": list(args.horizons),
        "closedLoopShadowGuidance": closed_loop,
        "flightLogs": closed_loop,
        "aeroCrossValidation": cross_validate_aero(db, max(1, args.max_aero_eval)) if db.exists() else {"error": f"missing DB: {db}"},
    }
    if args.physics_session and db.exists():
        result["recordedControlOpenLoopPhysics"] = recorded_control_physics_replay(root, db, args.physics_session)
    encoded = json.dumps(result, indent=2, sort_keys=True)
    if args.output:
        output = args.output if args.output.is_absolute() else root / args.output
        output.parent.mkdir(parents=True, exist_ok=True); output.write_text(encoded + "\n", encoding="utf-8")
    print(encoded)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
