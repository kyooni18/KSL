#!/usr/bin/env python3
"""Deterministic one-pass acceptance analysis for KSPShuttleLander split flight logs."""
from __future__ import annotations

import argparse
import copy
import json
import math
import statistics
from collections import defaultdict
from pathlib import Path
from typing import Any, Iterable

RUNTIME_FIELDS = {
    "loopWallDeltaMilliseconds": "loopWallDeltaMilliseconds",
    "controlLoopMilliseconds": "controlLoopMilliseconds",
    "telemetryLatencyMilliseconds": "telemetryLatencyMilliseconds",
    "guidanceComputeMilliseconds": "guidanceComputeMilliseconds",
    "applyLatencyMilliseconds": "applyLatencyMilliseconds",
}


def finite(value: Any) -> bool:
    return isinstance(value, (int, float)) and math.isfinite(float(value))


def percentile(values: Iterable[float], q: float) -> float | None:
    data = sorted(float(v) for v in values if finite(v))
    if not data:
        return None
    if len(data) == 1:
        return data[0]
    pos = max(0.0, min(1.0, q)) * (len(data) - 1)
    lo, hi = int(math.floor(pos)), int(math.ceil(pos))
    if lo == hi:
        return data[lo]
    frac = pos - lo
    return data[lo] * (1.0 - frac) + data[hi] * frac


def stats(values: Iterable[float]) -> dict[str, Any]:
    vals = [float(v) for v in values if finite(v)]
    return {
        "count": len(vals),
        "p50": percentile(vals, 0.50),
        "p95": percentile(vals, 0.95),
        "p99": percentile(vals, 0.99),
        "max": max(vals) if vals else None,
        "mean": statistics.fmean(vals) if vals else None,
    }


def merge_fields(base: dict[str, Any], patch: dict[str, Any]) -> None:
    for key, value in patch.items():
        if isinstance(value, dict) and isinstance(base.get(key), dict):
            merge_fields(base[key], value)
        elif isinstance(value, dict):
            base[key] = copy.deepcopy(value)
        else:
            base[key] = value


def jsonl(path: Path):
    with path.open("r", encoding="utf-8") as handle:
        for line_number, raw in enumerate(handle, 1):
            if not raw.strip():
                continue
            try:
                yield line_number, json.loads(raw)
            except json.JSONDecodeError:
                yield line_number, {"_decodeError": True}


def flatten_vehicle_record(record: dict[str, Any], state: dict[str, Any]) -> dict[str, Any]:
    status = state.get("state") or {}
    position = state.get("position") or {}
    motion = state.get("motion") or {}
    attitude = state.get("attitude") or {}
    command = state.get("command") or {}
    applied = state.get("appliedControl") or {}
    guidance = state.get("guidance") or {}
    aero = state.get("aero") or {}
    runtime = state.get("runtime") or {}
    vehicle_state = state.get("vehicle") or {}
    physics = state.get("physics") or {}
    return {
        "tick": record.get("tickSequence"),
        "ut": record.get("ut"),
        "wall": record.get("wallMonotonicSeconds"),
        "recordType": record.get("recordType"),
        "phase": status.get("phase"),
        "controlProfile": status.get("controlProfile"),
        "situation": status.get("vesselSituation"),
        "automation": status.get("automation"),
        "paused": status.get("paused"),
        "constraintActive": status.get("constraintActive"),
        "entryPlanValid": status.get("entryPlanValid"),
        "entryReversalScheduled": status.get("entryReversalScheduled"),
        "planId": status.get("planId"),
        "planVersion": status.get("planVersion"),
        "parentPlanId": status.get("parentPlanId"),
        "parentPlanVersion": status.get("parentPlanVersion"),
        "altitude": position.get("altitude"),
        "radarAltitude": position.get("radarAltitude"),
        "tas": motion.get("trueAirSpeed"),
        "verticalSpeed": motion.get("verticalSpeed"),
        "flightPathAngle": motion.get("flightPathAngle"),
        "heading": attitude.get("heading"),
        "roll": attitude.get("roll"),
        "coordinateRollRate": attitude.get("coordinateRollRate"),
        "angleOfAttack": attitude.get("angleOfAttack"),
        "angleOfAttackRate": attitude.get("angleOfAttackRate"),
        "gForce": aero.get("gForce"),
        "stallFraction": aero.get("stallFraction"),
        "stallFractionMeasured": aero.get("stallFractionMeasured"),
        "dynamicPressure": aero.get("dynamicPressure"),
        "mach": aero.get("mach"),
        "liftForce": aero.get("liftForce"),
        "dragForce": aero.get("dragForce"),
        "energyExcessRange": aero.get("energyExcessRange"),
        "mass": vehicle_state.get("mass"),
        "physicsConfidence": physics.get("confidence"),
        "physicsModelResidual": physics.get("modelResidual"),
        "physicsModelResidualConfidence": physics.get("modelResidualConfidence"),
        "physicsCertifiedUncertainty": physics.get("certifiedUncertainty"),
        "physicsForceResidualPerQ": copy.deepcopy(physics.get("forceResidualPerQ")),
        "physicsForceResidualSigmaPerQ": copy.deepcopy(physics.get("forceResidualSigmaPerQ")),
        "physicsForceResidualConfidence": physics.get("forceResidualConfidence"),
        "physicsSamples": physics.get("samples"),
        "physicsLiveSamples": physics.get("liveSamples"),
        "targetPitch": command.get("targetPitch"),
        "targetHeading": command.get("targetHeading"),
        "targetRoll": command.get("targetRoll"),
        "targetAoA": command.get("targetAoA"),
        "appliedPitch": applied.get("reportedPitch"),
        "appliedRoll": applied.get("reportedRoll"),
        "appliedYaw": applied.get("reportedYaw"),
        "range": guidance.get("rangeToSite"),
        "along": guidance.get("runwayAlongTrack"),
        "cross": guidance.get("runwayCrossTrack"),
        "reversalUT": guidance.get("entryReversalUT"),
        "reversalRange": guidance.get("entryReversalRange"),
        "reversalSign": guidance.get("entryReversalSign"),
        "runtime": copy.deepcopy(runtime),
    }


def load_vehicle(path: Path) -> dict[str, Any]:
    state: dict[str, Any] = {}
    rows: list[dict[str, Any]] = []
    session_id = None
    schema_version = None
    decode_errors: list[int] = []
    session_start: dict[str, Any] | None = None
    for line_number, record in jsonl(path):
        if record.get("_decodeError"):
            decode_errors.append(line_number)
            continue
        kind = record.get("recordType")
        if kind == "sessionStart":
            session_start = record
            session_id = record.get("sessionId") or session_id
            schema_version = record.get("schemaVersion") or schema_version
            continue
        if kind == "vehicleKeyframe":
            state = copy.deepcopy(record.get("fields") or {})
        elif kind == "vehicleDelta":
            merge_fields(state, record.get("fields") or {})
        else:
            continue
        rows.append(flatten_vehicle_record(record, state))
    return {
        "path": str(path),
        "sessionId": session_id,
        "schemaVersion": schema_version,
        "sessionStart": session_start,
        "rows": rows,
        "decodeErrors": decode_errors,
    }


def load_planner(path: Path | None) -> dict[str, Any]:
    if path is None:
        return {"path": None, "paths": [], "sessionId": None, "records": [], "decodeErrors": [], "missing": True}
    # Long live campaigns rotate the high-volume planner stream into
    # *-planner-001.jsonl, *-planner-002.jsonl, ... while retaining the base
    # *-planner.jsonl as the first segment. Treat that family as one logical stream.
    base_name = path.name.removesuffix(".jsonl")
    parts = ([path] if path.exists() else []) + sorted(path.parent.glob(base_name + "-[0-9][0-9][0-9].jsonl"))
    if not parts:
        return {"path": str(path), "paths": [], "sessionId": None, "records": [], "decodeErrors": [], "missing": True}
    records: list[dict[str, Any]] = []
    session_id = None
    decode_errors: list[dict[str, Any]] = []
    for part in parts:
        for line_number, record in jsonl(part):
            if record.get("_decodeError"):
                decode_errors.append({"path": str(part), "line": line_number})
                continue
            if record.get("recordType") == "sessionStart":
                session_id = record.get("sessionId") or session_id
            elif record.get("recordType") == "plannerSample":
                records.append(record)
    records.sort(key=lambda r: (float(r.get("ut")) if finite(r.get("ut")) else math.inf,
                                int(r.get("tickSequence")) if isinstance(r.get("tickSequence"), int) else 2**63 - 1))
    return {"path": str(path), "paths": [str(part) for part in parts], "sessionId": session_id,
            "records": records, "decodeErrors": decode_errors, "missing": False}


def infer_pair(path: Path, explicit_planner: Path | None) -> tuple[Path, Path | None]:
    if path.name.endswith("-planner.jsonl"):
        planner = path
        vehicle = path.with_name(path.name.removesuffix("-planner.jsonl") + "-vehicle.jsonl")
        return vehicle, planner
    vehicle = path
    if explicit_planner:
        return vehicle, explicit_planner
    if path.name.endswith("-vehicle.jsonl"):
        candidate = path.with_name(path.name.removesuffix("-vehicle.jsonl") + "-planner.jsonl")
        return vehicle, candidate
    return vehicle, None


def phase_summary(rows: list[dict[str, Any]]) -> dict[str, Any]:
    grouped: dict[str, list[dict[str, Any]]] = defaultdict(list)
    for row in rows:
        grouped[str(row.get("phase") or "Unknown")].append(row)
    result = {}
    for phase, phase_rows in grouped.items():
        uts = [r["ut"] for r in phase_rows if finite(r.get("ut"))]
        ticks = [int(r["tick"]) for r in phase_rows if isinstance(r.get("tick"), int)]
        result[phase] = {
            "updates": len(phase_rows),
            "firstUT": min(uts) if uts else None,
            "lastUT": max(uts) if uts else None,
            "durationUTSeconds": max(uts) - min(uts) if len(uts) >= 2 else 0.0 if uts else None,
            "firstTick": min(ticks) if ticks else None,
            "lastTick": max(ticks) if ticks else None,
        }
    return result


COMMON_RANGE_TARGETS_M = {
    400000.0: 34500.0,
    300000.0: 29000.0,
    200000.0: 23500.0,
    100000.0: 18000.0,
    72000.0: 16500.0,
}


def common_range_summary(rows: list[dict[str, Any]]) -> dict[str, Any]:
    """Freeze the first inward MM304 crossing of each live-acceptance range station."""
    result: dict[str, Any] = {}
    previous_range: float | None = None
    for row in rows:
        if row.get("phase") != "MM304 Entry" or not finite(row.get("range")):
            continue
        current_range = float(row["range"])
        for station, target_altitude in COMMON_RANGE_TARGETS_M.items():
            key = f"{int(station / 1000)}km"
            if key in result:
                continue
            crossed = previous_range is not None and previous_range > station >= current_range
            if not crossed:
                continue
            altitude = float(row["altitude"]) if finite(row.get("altitude")) else None
            result[key] = {
                "stationRangeM": station,
                "ut": row.get("ut"),
                "observedRangeM": current_range,
                "altitudeM": altitude,
                "targetAltitudeM": target_altitude,
                "altitudeDebtM": altitude - target_altitude if altitude is not None else None,
                "trueAirSpeedMps": row.get("tas"),
                "verticalSpeedMps": row.get("verticalSpeed"),
                "dynamicPressurePa": row.get("dynamicPressure"),
                "gForce": row.get("gForce"),
                "energyExcessRangeM": row.get("energyExcessRange"),
                "flightPathAngleDeg": row.get("flightPathAngle"),
                "angleOfAttackDeg": row.get("angleOfAttack"),
                "targetAoADeg": row.get("targetAoA"),
                "rollDeg": row.get("roll"),
                "targetRollDeg": row.get("targetRoll"),
            }
        previous_range = current_range
    previous_debt: float | None = None
    for station in COMMON_RANGE_TARGETS_M:
        point = result.get(f"{int(station / 1000)}km")
        if not point:
            continue
        debt = point.get("altitudeDebtM")
        point["altitudeDebtDeltaFromPreviousStationM"] = (
            float(debt) - previous_debt if finite(debt) and previous_debt is not None else None
        )
        if finite(debt):
            previous_debt = float(debt)
    return result


def entry_corridor_flags(common_ranges: dict[str, Any]) -> list[dict[str, Any]]:
    """Apply the published deterministic MM304 common-range release gates."""
    flags: list[dict[str, Any]] = []
    c400 = common_ranges.get("400km") or {}
    c300 = common_ranges.get("300km") or {}
    c200 = common_ranges.get("200km") or {}
    c100 = common_ranges.get("100km") or {}
    if finite(c300.get("altitudeDebtM")) and float(c300["altitudeDebtM"]) > 10000.0 and \
            finite(c400.get("altitudeDebtM")) and float(c300["altitudeDebtM"]) >= float(c400["altitudeDebtM"]):
        flags.append({"code": "ENTRY_ALTITUDE_DEBT_NOT_CONVERGING_BY_300KM", "severity": "error",
                      "debt300M": c300["altitudeDebtM"], "debt400M": c400["altitudeDebtM"]})
    if finite(c200.get("altitudeDebtM")) and float(c200["altitudeDebtM"]) > 7000.0:
        flags.append({"code": "ENTRY_ALTITUDE_DEBT_ABOVE_7KM_AT_200KM", "severity": "error",
                      "debtM": c200["altitudeDebtM"]})
    if finite(c100.get("altitudeM")) and float(c100["altitudeM"]) > 21000.0:
        flags.append({"code": "ENTRY_ALTITUDE_ABOVE_21KM_AT_100KM", "severity": "error",
                      "altitudeM": c100["altitudeM"]})
    if finite(c100.get("verticalSpeedMps")) and float(c100["verticalSpeedMps"]) >= 0.0:
        flags.append({"code": "ENTRY_NOT_DESCENDING_AT_100KM", "severity": "error",
                      "verticalSpeedMps": c100["verticalSpeedMps"]})
    return flags


def thermal_aoa_summary(rows: list[dict[str, Any]], floor_deg: float,
                        maximum_g_load: float, minimum_safe_speed: float) -> dict[str, Any]:
    samples = [row for row in rows if row.get("phase") == "MM304 Entry" and finite(row.get("targetAoA"))]
    below = [row for row in samples if float(row["targetAoA"]) < floor_deg - 0.05]
    explicit_safety: list[dict[str, Any]] = []
    recovery_transient: list[dict[str, Any]] = []
    unexplained: list[dict[str, Any]] = []
    recovery_armed = False
    recovery_seen_below = False
    last_safety_ut: float | None = None
    recovery_grace_s = 0.0  # Below-floor AoA is legal only while explicit safety is active.
    for row in samples:
        target = float(row["targetAoA"])
        g_recovery = finite(row.get("gForce")) and float(row["gForce"]) / max(maximum_g_load, 0.1) > 0.92
        measured_stall = row.get("stallFractionMeasured") is True
        stall_recovery = (measured_stall and finite(row.get("stallFraction")) and float(row["stallFraction"]) > 0.12) or \
            (finite(row.get("tas")) and float(row["tas"]) < minimum_safe_speed * 1.10)
        safety_now = g_recovery or stall_recovery
        if safety_now:
            recovery_armed = True
            if finite(row.get("ut")):
                last_safety_ut = float(row["ut"])
        if target >= floor_deg - 0.05:
            if recovery_armed and recovery_seen_below:
                recovery_armed = False
                recovery_seen_below = False
            elif recovery_armed and last_safety_ut is not None and finite(row.get("ut")) and \
                    float(row["ut"]) - last_safety_ut > recovery_grace_s:
                recovery_armed = False
            continue
        if safety_now:
            explicit_safety.append(row)
            recovery_seen_below = True
        elif recovery_armed and last_safety_ut is not None and finite(row.get("ut")) and \
                float(row["ut"]) - last_safety_ut <= recovery_grace_s:
            recovery_transient.append(row)
            recovery_seen_below = True
        else:
            unexplained.append(row)
    return {
        "floorDeg": floor_deg,
        "sampleCount": len(samples),
        "minimumTargetAoADeg": min((float(row["targetAoA"]) for row in samples), default=None),
        "belowFloorCount": len(below),
        "explicitSafetyBelowFloorCount": len(explicit_safety),
        "safetyRecoveryTransientBelowFloorCount": len(recovery_transient),
        "unexplainedBelowFloorCount": len(unexplained),
        "firstUnexplainedBelowFloor": ({
            "tick": unexplained[0].get("tick"), "ut": unexplained[0].get("ut"),
            "rangeToSiteM": unexplained[0].get("range"), "altitudeM": unexplained[0].get("altitude"),
            "targetAoADeg": unexplained[0].get("targetAoA"), "gForce": unexplained[0].get("gForce"),
            "stallFraction": unexplained[0].get("stallFraction"),
            "stallFractionMeasured": unexplained[0].get("stallFractionMeasured"),
        } if unexplained else None),
    }


def first_handoff(rows: list[dict[str, Any]]) -> dict[str, Any] | None:
    seen_mm304 = False
    for row in rows:
        phase = row.get("phase")
        if phase == "MM304 Entry":
            seen_mm304 = True
        elif seen_mm304 and phase == "TAEM":
            return {
                "tick": row.get("tick"), "ut": row.get("ut"), "altitudeM": row.get("altitude"),
                "trueAirSpeedMps": row.get("tas"), "verticalSpeedMps": row.get("verticalSpeed"),
                "flightPathAngleDeg": row.get("flightPathAngle"), "headingDeg": row.get("heading"),
                "rangeToSiteM": row.get("range"), "runwayAlongTrackM": row.get("along"),
                "runwayCrossTrackM": row.get("cross"),
            }
    return None


def taem_kinematic_prerequisite_summary(
        rows: list[dict[str, Any]], altitude_min_m: float = 15000.0,
        altitude_max_m: float = 18000.0, speed_max_mps: float = 1300.5) -> dict[str, Any]:
    """Report basic MM304 altitude/speed/upstream coexistence without claiming full TAEM eligibility."""
    samples = [row for row in rows if row.get("phase") == "MM304 Entry"]

    def snapshot(row: dict[str, Any] | None) -> dict[str, Any] | None:
        if row is None:
            return None
        return {
            "tick": row.get("tick"), "ut": row.get("ut"),
            "altitudeM": row.get("altitude"), "trueAirSpeedMps": row.get("tas"),
            "rangeToSiteM": row.get("range"), "runwayAlongTrackM": row.get("along"),
            "runwayCrossTrackM": row.get("cross"), "flightPathAngleDeg": row.get("flightPathAngle"),
            "dynamicPressurePa": row.get("dynamicPressure"), "gForce": row.get("gForce"),
        }

    first_speed = next((row for row in samples if finite(row.get("tas")) and float(row["tas"]) <= speed_max_mps), None)
    first_altitude = next((row for row in samples if finite(row.get("altitude")) and float(row["altitude"]) <= altitude_max_m), None)
    first_basic = next((row for row in samples
                        if finite(row.get("altitude")) and altitude_min_m <= float(row["altitude"]) <= altitude_max_m
                        and finite(row.get("tas")) and float(row["tas"]) <= speed_max_mps
                        and finite(row.get("along")) and float(row["along"]) < 0.0), None)
    return {
        "altitudeMinM": altitude_min_m,
        "altitudeMaxM": altitude_max_m,
        "speedMaxMps": speed_max_mps,
        "basicUpstreamWindowSeen": first_basic is not None,
        "firstSpeedAtOrBelowMax": snapshot(first_speed),
        "firstAltitudeAtOrBelowMax": snapshot(first_altitude),
        "firstBasicUpstreamWindow": snapshot(first_basic),
    }


def observed_aero_force_summary(
        rows: list[dict[str, Any]], milestones_pa: tuple[float, ...] = (300.0, 360.0, 500.0, 700.0, 1000.0, 1300.0)) -> dict[str, Any]:
    """Freeze live force evidence at the first rising MM304 crossing of fixed q milestones."""
    result: dict[str, Any] = {}
    previous_q: float | None = None
    for row in rows:
        if row.get("phase") != "MM304 Entry" or not finite(row.get("dynamicPressure")):
            continue
        q = float(row["dynamicPressure"])
        for milestone in milestones_pa:
            key = f"q{int(milestone)}"
            if key in result:
                continue
            crossed = q >= milestone and (previous_q is None or previous_q < milestone)
            if not crossed:
                continue
            mass = float(row["mass"]) if finite(row.get("mass")) and float(row["mass"]) > 0.0 else None
            drag_force = float(row["dragForce"]) if finite(row.get("dragForce")) else None
            lift_force = float(row["liftForce"]) if finite(row.get("liftForce")) else None
            drag_accel = abs(drag_force) / mass if drag_force is not None and mass is not None else None
            lift_accel = abs(lift_force) / mass if lift_force is not None and mass is not None else None
            result[key] = {
                "targetDynamicPressurePa": milestone,
                "observedDynamicPressurePa": q,
                "ut": row.get("ut"),
                "rangeToSiteM": row.get("range"),
                "altitudeM": row.get("altitude"),
                "mach": row.get("mach"),
                "angleOfAttackDeg": row.get("angleOfAttack"),
                "massKg": mass,
                "dragForceN": drag_force,
                "liftForceN": lift_force,
                "dragAccelerationMps2": drag_accel,
                "liftAccelerationMps2": lift_accel,
                "liftToDrag": (abs(lift_force) / abs(drag_force)
                               if lift_force is not None and drag_force is not None and abs(drag_force) > 1e-9 else None),
                "physicsConfidence": row.get("physicsConfidence"),
                "physicsModelResidual": row.get("physicsModelResidual"),
                "physicsModelResidualConfidence": row.get("physicsModelResidualConfidence"),
                "physicsCertifiedUncertainty": row.get("physicsCertifiedUncertainty"),
                "physicsForceResidualPerQ": copy.deepcopy(row.get("physicsForceResidualPerQ")),
                "physicsForceResidualSigmaPerQ": copy.deepcopy(row.get("physicsForceResidualSigmaPerQ")),
                "physicsForceResidualConfidence": row.get("physicsForceResidualConfidence"),
                "physicsSamples": row.get("physicsSamples"),
                "physicsLiveSamples": row.get("physicsLiveSamples"),
            }
        previous_q = q
    return result


def reversal_summary(rows: list[dict[str, Any]], planner_records: list[dict[str, Any]],
                     minimum_leg_duration_s: float = 24.0) -> dict[str, Any]:
    events_by_key: dict[tuple[float, float], dict[str, Any]] = {}
    for row in rows:
        if not row.get("entryReversalScheduled") or not finite(row.get("reversalUT")) or not finite(row.get("reversalSign")):
            continue
        key = (round(float(row["reversalUT"]), 3), round(float(row["reversalSign"]), 3))
        event = events_by_key.get(key)
        if event is None:
            event = {
                "observedTick": row.get("tick"), "observedUT": row.get("ut"),
                "lastObservedTick": row.get("tick"), "lastObservedUT": row.get("ut"),
                "plannedUT": row.get("reversalUT"), "plannedRangeM": row.get("reversalRange"),
                "sign": row.get("reversalSign"),
            }
            events_by_key[key] = event
        else:
            event["lastObservedTick"] = row.get("tick")
            event["lastObservedUT"] = row.get("ut")
    events = list(events_by_key.values())

    rows_by_tick = {int(r["tick"]): r for r in rows if isinstance(r.get("tick"), int)}
    predicted = []
    comparable = []
    mismatches = []
    unpaired = 0
    physical_deadlines: dict[tuple[float, float], float] = {}
    for record in planner_records:
        plan = record.get("currentPlan") or {}
        if not plan.get("hasPlannedReversal"):
            continue
        predicted.append((plan.get("plannedReversalUT"), plan.get("plannedReversalSign")))
        if finite(plan.get("plannedReversalUT")) and finite(plan.get("plannedReversalSign")):
            key = (round(float(plan["plannedReversalUT"]), 3), round(float(plan["plannedReversalSign"]), 3))
            trace = record.get("plannerTrace") or {}
            if trace.get("legEstablished") is True and finite(trace.get("legElapsed")) and finite(record.get("ut")):
                physical_due = max(float(plan["plannedReversalUT"]),
                                   float(record["ut"]) + max(0.0, minimum_leg_duration_s - float(trace["legElapsed"])))
                prior = physical_deadlines.get(key)
                physical_deadlines[key] = physical_due if prior is None else min(prior, physical_due)
        tick = record.get("tickSequence")
        row = rows_by_tick.get(tick) if isinstance(tick, int) else None
        if row is None or not row.get("entryReversalScheduled"):
            unpaired += 1
            continue
        if not all(finite(v) for v in (plan.get("plannedReversalUT"), plan.get("plannedReversalSign"), row.get("reversalUT"), row.get("reversalSign"))):
            continue
        delta = abs(float(plan["plannedReversalUT"]) - float(row["reversalUT"]))
        sign_match = float(plan["plannedReversalSign"]) * float(row["reversalSign"]) > 0.0
        sample = {
            "tick": tick, "ut": record.get("ut"),
            "plannerReversalUT": plan.get("plannedReversalUT"), "vehicleReversalUT": row.get("reversalUT"),
            "utDeltaSeconds": delta, "plannerSign": plan.get("plannedReversalSign"),
            "vehicleSign": row.get("reversalSign"), "signMatch": sign_match,
        }
        comparable.append(sample)
        if delta > 0.15 or not sign_match:
            mismatches.append(sample)

    last_ut = max((float(r["ut"]) for r in rows if finite(r.get("ut"))), default=None)
    effective_captures = []
    missing_capture_count = 0
    past_threshold_count = 0
    low_reserve_count = 0
    late_execution_count = 0
    unexecutable_published_count = 0

    def capture_point(candidates: list[dict[str, Any]], field: str, sign: float, due_ut: float) -> dict[str, Any] | None:
        for row in candidates:
            value = row.get(field)
            if not finite(value) or sign * float(value) <= 0.0:
                continue
            return {
                "tick": row.get("tick"), "ut": row.get("ut"),
                "utDeltaSeconds": float(row["ut"]) - due_ut,
                "valueDeg": value, "runwayAlongTrackM": row.get("along"), "rangeToSiteM": row.get("range"),
            }
        return None

    for event in events:
        planned_ut = float(event["plannedUT"])
        sign = 1.0 if float(event["sign"]) >= 0.0 else -1.0
        key = (round(planned_ut, 3), round(float(event["sign"]), 3))
        physical_due = physical_deadlines.get(key)
        executable_ut = physical_due if physical_due is not None else planned_ut
        due = last_ut is not None and executable_ut <= last_ut + 1e-6
        last_observed = event.get("lastObservedUT")
        durable = finite(last_observed) and float(last_observed) >= min(planned_ut, executable_ut) - 2.0
        dwell_deferral = max(0.0, executable_ut - planned_ut)
        capture: dict[str, Any] = {
            "plannedUT": planned_ut, "sign": sign, "durable": durable, "due": due,
            "earliestExecutableUT": executable_ut,
            "physicalDwellDeadlineAvailable": physical_due is not None,
            "physicalDwellDeferralSeconds": dwell_deferral,
        }
        if physical_due is not None and dwell_deferral > 2.0 + 1e-6:
            unexecutable_published_count += 1
        if not due:
            capture["status"] = "not-physically-due-in-observed-log" if physical_due is not None else "not-due-in-observed-log"
            effective_captures.append(capture)
            continue
        if not durable:
            capture["status"] = "superseded-before-due"
            effective_captures.append(capture)
            continue
        candidates = [r for r in rows if finite(r.get("ut")) and float(r["ut"]) >= executable_ut - 0.15]
        def same_durable_event(row: dict[str, Any]) -> bool:
            return bool(row.get("entryReversalScheduled")) and finite(row.get("reversalUT")) and finite(row.get("reversalSign")) and \
                abs(float(row["reversalUT"]) - planned_ut) <= 0.15 and float(row["reversalSign"]) * sign > 0.0
        execution = next((r for r in candidates if not same_durable_event(r)), None)
        if execution is not None:
            capture["executionStateTransition"] = {
                "tick": execution.get("tick"), "ut": execution.get("ut"),
                "utDeltaSeconds": float(execution["ut"]) - executable_ut,
                "runwayAlongTrackM": execution.get("along"), "rangeToSiteM": execution.get("range"),
            }
            capture["executionDelaySeconds"] = float(execution["ut"]) - executable_ut
            if capture["executionDelaySeconds"] > 2.0 + 1e-6:
                late_execution_count += 1
        else:
            capture["executionStateTransition"] = None
            capture["executionDelaySeconds"] = None
        target = capture_point(candidates, "targetRoll", sign, executable_ut)
        measured = capture_point(candidates, "roll", sign, executable_ut)
        capture["targetSignCapture"] = target
        capture["measuredSignCapture"] = measured
        capture["targetResponseDelaySeconds"] = target["utDeltaSeconds"] if target is not None else None
        capture["measuredResponseDelaySeconds"] = measured["utDeltaSeconds"] if measured is not None else None
        missing = target is None or measured is None
        if missing:
            missing_capture_count += 1
        capture_past = False
        capture_low = False
        for point in (target, measured):
            if not point or not finite(point.get("runwayAlongTrackM")):
                continue
            along = float(point["runwayAlongTrackM"])
            capture_past = capture_past or along >= 0.0
            capture_low = capture_low or (-10000.0 < along < 0.0)
        if capture_past:
            past_threshold_count += 1
        elif capture_low:
            low_reserve_count += 1
        capture["status"] = "missing-capture" if missing else "late-execution" if execution is not None and float(capture["executionDelaySeconds"]) > 2.0 + 1e-6 else "past-threshold" if capture_past else "low-upstream-reserve" if capture_low else "captured"
        effective_captures.append(capture)

    return {
        "scheduledEventCount": len(events), "scheduledEvents": events,
        "plannerSamplesWithPlannedReversal": len(predicted),
        "comparablePlannerVehicleSamples": len(comparable),
        "plannerVehicleMismatchCount": len(mismatches), "plannerVehicleMismatches": mismatches,
        "unpairedPlannerReversalSamples": unpaired,
        "effectiveCaptures": effective_captures,
        "effectiveCaptureMissingCount": missing_capture_count,
        "effectiveCapturePastThresholdCount": past_threshold_count,
        "effectiveCaptureLowReserveCount": low_reserve_count,
        "lateExecutableReversalCount": late_execution_count,
        "publishedBeforePhysicalDwellCount": unexecutable_published_count,
        "minimumLegDurationSeconds": minimum_leg_duration_s,
    }


def lineage_summary(records: list[dict[str, Any]]) -> dict[str, Any]:
    keys: list[tuple[int, int]] = []
    breaks = []
    last: tuple[int, int] | None = None
    for record in records:
        lineage = record.get("lineage") or {}
        pid, version = lineage.get("planId"), lineage.get("version")
        if not isinstance(pid, int) or not isinstance(version, int) or pid <= 0 or version <= 0:
            continue
        key = (pid, version)
        if keys and key == keys[-1]:
            continue
        parent = (lineage.get("parentPlanId"), lineage.get("parentVersion"))
        if last is not None and parent != last:
            breaks.append({"ut": record.get("ut"), "tick": record.get("tickSequence"), "previous": list(last), "current": list(key), "reportedParent": list(parent)})
        keys.append(key)
        last = key
    return {
        "available": bool(keys), "distinctPlanVersions": len(keys), "first": list(keys[0]) if keys else None,
        "last": list(keys[-1]) if keys else None, "continuityBreakCount": len(breaks), "continuityBreaks": breaks,
    }


def plan_identity_summary(rows: list[dict[str, Any]], records: list[dict[str, Any]]) -> dict[str, Any]:
    rows_by_tick = {int(row["tick"]): row for row in rows if isinstance(row.get("tick"), int)}
    vehicle_available = any(isinstance(row.get("planId"), int) and int(row["planId"]) > 0 for row in rows)
    transitions: list[dict[str, Any]] = []
    last_key: tuple[int, int] | None = None
    for record in records:
        lineage = record.get("lineage") or {}
        pid, version = lineage.get("planId"), lineage.get("version")
        if not isinstance(pid, int) or not isinstance(version, int) or pid <= 0 or version <= 0:
            continue
        key = (pid, version)
        if key == last_key:
            continue
        last_key = key
        transitions.append({
            "tick": record.get("tickSequence"), "ut": record.get("ut"),
            "planId": pid, "planVersion": version,
            "parentPlanId": lineage.get("parentPlanId"), "parentPlanVersion": lineage.get("parentVersion"),
        })

    comparable: list[dict[str, Any]] = []
    missing: list[dict[str, Any]] = []
    mismatches: list[dict[str, Any]] = []
    for transition in transitions:
        tick = transition.get("tick")
        row = rows_by_tick.get(tick) if isinstance(tick, int) else None
        if row is None or not isinstance(row.get("planId"), int):
            missing.append(transition)
            continue
        observed = {
            "planId": row.get("planId"), "planVersion": row.get("planVersion"),
            "parentPlanId": row.get("parentPlanId"), "parentPlanVersion": row.get("parentPlanVersion"),
        }
        sample = {**transition, "vehicle": observed}
        comparable.append(sample)
        expected_parent = (transition.get("parentPlanId"), transition.get("parentPlanVersion"))
        observed_parent = (observed.get("parentPlanId"), observed.get("parentPlanVersion"))
        if ((observed.get("planId"), observed.get("planVersion")) != (transition["planId"], transition["planVersion"]) or
                observed_parent != expected_parent):
            mismatches.append(sample)
    return {
        "vehicleIdentityAvailable": vehicle_available,
        "plannerPlanTransitionCount": len(transitions),
        "comparableTransitionCount": len(comparable),
        "missingTransitionCount": len(missing), "missingTransitions": missing,
        "mismatchCount": len(mismatches), "mismatches": mismatches,
    }


def entry_plan_churn_summary(records: list[dict[str, Any]], rapid_interval_s: float = 0.5,
                             frequent_interval_s: float = 6.0) -> dict[str, Any]:
    """Detect MM304 plan regeneration at control-loop or sustained retry cadence."""
    transitions: list[dict[str, Any]] = []
    last_key: tuple[int, int] | None = None
    for record in records:
        if str(record.get("phase") or "").lower() != "entryenergy":
            continue
        lineage = record.get("lineage") or {}
        pid, version = lineage.get("planId"), lineage.get("version")
        if not isinstance(pid, int) or not isinstance(version, int) or pid <= 0 or version <= 0:
            continue
        key = (pid, version)
        if key == last_key:
            continue
        last_key = key
        transitions.append({"tick": record.get("tickSequence"), "ut": record.get("ut"),
                            "planId": pid, "planVersion": version})

    rapid: list[dict[str, Any]] = []
    frequent: list[dict[str, Any]] = []
    minimum_interval: float | None = None
    rapid_consecutive = 0
    maximum_rapid_consecutive = 0
    frequent_consecutive = 0
    maximum_frequent_consecutive = 0
    for previous, current in zip(transitions, transitions[1:]):
        if not finite(previous.get("ut")) or not finite(current.get("ut")):
            rapid_consecutive = 0
            frequent_consecutive = 0
            continue
        interval = float(current["ut"]) - float(previous["ut"])
        if interval < 0.0:
            rapid_consecutive = 0
            frequent_consecutive = 0
            continue
        minimum_interval = interval if minimum_interval is None else min(minimum_interval, interval)
        transition = {"intervalSeconds": interval, "previous": previous, "current": current}
        if interval <= rapid_interval_s + 1e-9:
            rapid_consecutive += 1
            maximum_rapid_consecutive = max(maximum_rapid_consecutive, rapid_consecutive)
            rapid.append(transition)
        else:
            rapid_consecutive = 0
        if interval <= frequent_interval_s + 1e-9:
            frequent_consecutive += 1
            maximum_frequent_consecutive = max(maximum_frequent_consecutive, frequent_consecutive)
            frequent.append(transition)
        else:
            frequent_consecutive = 0

    valid_uts = [float(item["ut"]) for item in transitions if finite(item.get("ut"))]
    duration = max(0.0, valid_uts[-1] - valid_uts[0]) if len(valid_uts) >= 2 else 0.0
    rate = (len(valid_uts) - 1) / duration if duration > 1e-9 else 0.0
    return {
        "transitionCount": len(transitions),
        "rapidIntervalThresholdSeconds": rapid_interval_s,
        "rapidTransitionCount": len(rapid),
        "maximumConsecutiveRapidTransitions": maximum_rapid_consecutive,
        "frequentIntervalThresholdSeconds": frequent_interval_s,
        "frequentTransitionCount": len(frequent),
        "maximumConsecutiveFrequentTransitions": maximum_frequent_consecutive,
        "minimumTransitionIntervalSeconds": minimum_interval,
        "transitionRateHz": rate,
        "firstRapidTransition": rapid[0] if rapid else None,
        "firstFrequentTransition": frequent[0] if frequent else None,
    }


def terminal_summary(records: list[dict[str, Any]]) -> dict[str, Any]:
    taem = [r for r in records if str(r.get("phase") or "").lower() == "taem"]
    true_counts = {k: 0 for k in ("terminalPredictionValid", "terminalCandidateValid", "terminalCommitted")}
    first_commit = None
    transitions, circuits, remaining, mixes = [], [], [], []
    for record in taem:
        geom = record.get("terminalGeometry") or {}
        for key in true_counts:
            if geom.get(key) is True:
                true_counts[key] += 1
        if first_commit is None and geom.get("terminalCommitted") is True:
            first_commit = {"ut": record.get("ut"), "tick": record.get("tickSequence"), "geometry": geom}
        for key, target in (("hacTransitionProgress", transitions), ("hacCircuitCount", circuits), ("hacRemaining", remaining), ("mix", mixes)):
            if finite(geom.get(key)):
                target.append(float(geom[key]))
    return {
        "plannerTAEMSamples": len(taem), **true_counts, "firstTerminalCommit": first_commit,
        "maximumHACTransitionProgress": max(transitions) if transitions else None,
        "maximumHACCircuitCount": max(circuits) if circuits else None,
        "hacRemainingRange": [min(remaining), max(remaining)] if remaining else None,
        "mixRange": [min(mixes), max(mixes)] if mixes else None,
    }


def terminal_smoothness_summary(rows: list[dict[str, Any]], control_deadband: float = 0.25,
                                rapid_reversal_interval_s: float = 1.0,
                                large_target_step_deg: float = 10.0) -> dict[str, Any]:
    """Quantify terminal target continuity and control sign flashing without imposing acceptance gates."""
    terminal_tokens = ("taem", "hac", "final", "flare", "touchdown")
    terminal_rows = [
        row for row in rows
        if any(token in str(row.get("phase") or "").lower() for token in terminal_tokens)
    ]

    def signed_delta(current: float, previous: float, circular: bool = False) -> float:
        delta = current - previous
        if circular:
            delta = (delta + 180.0) % 360.0 - 180.0
        return delta

    target_metrics: dict[str, Any] = {}
    for field, circular in (("targetPitch", False), ("targetRoll", False),
                            ("targetHeading", True), ("targetAoA", False)):
        steps: list[float] = []
        large_steps: list[dict[str, Any]] = []
        previous: dict[str, Any] | None = None
        for row in terminal_rows:
            value = row.get(field)
            if not finite(value):
                continue
            if previous is not None and finite(previous.get(field)):
                delta = abs(signed_delta(float(value), float(previous[field]), circular))
                steps.append(delta)
                if delta > large_target_step_deg + 1e-9:
                    large_steps.append({
                        "field": field,
                        "fromTick": previous.get("tick"), "toTick": row.get("tick"),
                        "fromUT": previous.get("ut"), "toUT": row.get("ut"),
                        "fromPhase": previous.get("phase"), "toPhase": row.get("phase"),
                        "stepDeg": delta,
                    })
            previous = row
        target_metrics[field] = {
            "stepCount": len(steps),
            "maximumStepDeg": max(steps) if steps else None,
            "p95StepDeg": percentile(steps, 0.95),
            "largeStepThresholdDeg": large_target_step_deg,
            "largeStepCount": len(large_steps),
            "firstLargeStep": large_steps[0] if large_steps else None,
        }

    control_metrics: dict[str, Any] = {}
    for field in ("appliedPitch", "appliedRoll", "appliedYaw"):
        reversals: list[dict[str, Any]] = []
        last_active: dict[str, Any] | None = None
        for row in terminal_rows:
            value = row.get(field)
            if not finite(value) or abs(float(value)) < control_deadband:
                continue
            sign = 1 if float(value) > 0.0 else -1
            if last_active is not None and sign != last_active["sign"]:
                interval = None
                if finite(last_active.get("ut")) and finite(row.get("ut")):
                    interval = float(row["ut"]) - float(last_active["ut"])
                reversals.append({
                    "field": field,
                    "fromTick": last_active.get("tick"), "toTick": row.get("tick"),
                    "fromUT": last_active.get("ut"), "toUT": row.get("ut"),
                    "fromPhase": last_active.get("phase"), "toPhase": row.get("phase"),
                    "intervalSeconds": interval,
                    "fromValue": last_active.get("value"), "toValue": float(value),
                })
            last_active = {
                "sign": sign, "value": float(value), "tick": row.get("tick"),
                "ut": row.get("ut"), "phase": row.get("phase"),
            }
        intervals = [float(item["intervalSeconds"]) for item in reversals if finite(item.get("intervalSeconds"))]
        rapid = [item for item in reversals if finite(item.get("intervalSeconds"))
                 and 0.0 <= float(item["intervalSeconds"]) <= rapid_reversal_interval_s + 1e-9]
        control_metrics[field] = {
            "deadband": control_deadband,
            "reversalCount": len(reversals),
            "rapidReversalIntervalSeconds": rapid_reversal_interval_s,
            "rapidReversalCount": len(rapid),
            "minimumReversalIntervalSeconds": min(intervals) if intervals else None,
            "firstRapidReversal": rapid[0] if rapid else None,
        }

    return {
        "terminalSampleCount": len(terminal_rows),
        "phasesPresent": sorted({str(row.get("phase")) for row in terminal_rows if row.get("phase")}),
        "targetContinuity": target_metrics,
        "appliedControlReversals": control_metrics,
    }


def direct_runtime_latencies(rows: list[dict[str, Any]]) -> tuple[bool, dict[str, Any]]:
    has_runtime = any(any(finite((r.get("runtime") or {}).get(key)) for key in RUNTIME_FIELDS) for r in rows)
    if not has_runtime:
        return False, {}
    grouped: dict[str, dict[str, list[float]]] = defaultdict(lambda: defaultdict(list))
    for row in rows:
        phase = str(row.get("phase") or "Unknown")
        runtime = row.get("runtime") or {}
        for output_key, source_key in RUNTIME_FIELDS.items():
            if finite(runtime.get(source_key)):
                grouped[phase][output_key].append(float(runtime[source_key]))
    result = {}
    for phase, metrics in grouped.items():
        result[phase] = {name: stats(values) for name, values in metrics.items()}
    return True, result


def direct_runtime_gap_events(rows: list[dict[str, Any]], threshold_ms: float = 2000.0) -> list[dict[str, Any]]:
    events: list[dict[str, Any]] = []
    for row in rows:
        runtime = row.get("runtime") or {}
        loop = runtime.get("loopWallDeltaMilliseconds")
        control = runtime.get("controlLoopMilliseconds")
        if not ((finite(loop) and float(loop) > threshold_ms) or (finite(control) and float(control) > threshold_ms)):
            continue
        events.append({
            "phase": str(row.get("phase") or "Unknown"), "tick": row.get("tick"), "ut": row.get("ut"),
            "loopWallDeltaMilliseconds": float(loop) if finite(loop) else None,
            "controlLoopMilliseconds": float(control) if finite(control) else None,
            "telemetryLatencyMilliseconds": runtime.get("telemetryLatencyMilliseconds"),
            "guidanceComputeMilliseconds": runtime.get("guidanceComputeMilliseconds"),
            "applyLatencyMilliseconds": runtime.get("applyLatencyMilliseconds"),
        })
    return events


def inferred_wall_latencies(rows: list[dict[str, Any]]) -> tuple[dict[str, Any], list[dict[str, Any]]]:
    grouped: dict[str, list[float]] = defaultdict(list)
    gaps: list[dict[str, Any]] = []
    for before, after in zip(rows, rows[1:]):
        if not finite(before.get("wall")) or not finite(after.get("wall")):
            continue
        delta_ms = (float(after["wall"]) - float(before["wall"])) * 1000.0
        if delta_ms < 0:
            continue
        phase = str(before.get("phase") or "Unknown")
        grouped[phase].append(delta_ms)
        if delta_ms > 2000.0:
            gaps.append({
                "sourcePhase": phase, "fromTick": before.get("tick"), "toTick": after.get("tick"),
                "fromUT": before.get("ut"), "toUT": after.get("ut"), "wallGapMilliseconds": delta_ms,
                "kspUTAdvanceSeconds": float(after["ut"]) - float(before["ut"]) if finite(before.get("ut")) and finite(after.get("ut")) else None,
            })
    return {phase: {"loopWallDeltaMilliseconds": stats(values)} for phase, values in grouped.items()}, gaps


def outcome_summary(rows: list[dict[str, Any]], planner_records: list[dict[str, Any]],
                    runway_length_m: float, runway_width_m: float) -> dict[str, Any]:
    phases = [str(r.get("phase") or "") for r in rows]
    lower = [p.lower() for p in phases]
    situations = [str(r.get("situation") or "").lower() for r in rows]
    landed_indices = [index for index, situation in enumerate(situations) if situation == "landed"]
    splashed_indices = [index for index, situation in enumerate(situations) if situation == "splashed"]
    landed_seen = bool(landed_indices)
    splashed_seen = bool(splashed_indices)
    contact_index = min(landed_indices + splashed_indices) if landed_indices or splashed_indices else None
    contact = rows[contact_index] if contact_index is not None else None
    precontact_lower = lower[:contact_index + 1] if contact_index is not None else lower
    touchdown_phase_seen = any("touchdown" in p for p in lower)
    abort = any(p == "abort" for p in lower)
    fault = any(p == "fault" for p in lower)
    final_indices = [index for index, p in enumerate(precontact_lower) if "final" in p]
    flare_or_touchdown_indices = [index for index, p in enumerate(precontact_lower)
                                  if "flare" in p or "touchdown" in p]
    final_seen = any("final" in p for p in lower)
    flare_seen = any("flare" in p for p in lower)
    landing_sequence_valid = bool(
        final_indices
        and flare_or_touchdown_indices
        and min(final_indices) < max(flare_or_touchdown_indices)
    )
    runway_cross_limit_m = max(runway_width_m * 0.75, 45.0)
    runway_contact_valid = bool(
        contact is not None
        and finite(contact.get("along"))
        and finite(contact.get("cross"))
        and -40.0 <= float(contact["along"]) < runway_length_m + 80.0
        and abs(float(contact["cross"])) < runway_cross_limit_m
    )
    touchdown = landed_seen and not splashed_seen and not fault and landing_sequence_valid and runway_contact_valid
    disconnect = any("disconnect" in str(r.get("eventReason") or "").lower() for r in planner_records)
    last = rows[-1] if rows else {}
    if splashed_seen:
        outcome = "splashdown"
    elif fault:
        outcome = "fault"
    elif touchdown:
        outcome = "touchdown"
    elif landed_seen:
        outcome = "landed-invalid"
    elif abort:
        outcome = "abort"
    elif disconnect:
        outcome = "incomplete-disconnected"
    else:
        outcome = "incomplete"
    return {
        "outcome": outcome, "touchdownSeen": touchdown, "touchdownPhaseSeen": touchdown_phase_seen,
        "landedSeen": landed_seen, "splashedSeen": splashed_seen,
        "finalSeen": final_seen, "flareSeen": flare_seen, "landingSequenceValid": landing_sequence_valid,
        "runwayContactValid": runway_contact_valid,
        "contact": ({
            "tick": contact.get("tick"), "ut": contact.get("ut"), "situation": contact.get("situation"),
            "runwayAlongTrackM": contact.get("along"), "runwayCrossTrackM": contact.get("cross"),
        } if contact else None),
        "runwayEnvelope": {"alongMinM": -40.0, "alongMaxExclusiveM": runway_length_m + 80.0,
                           "crossAbsMaxExclusiveM": runway_cross_limit_m},
        "abortSeen": abort, "faultSeen": fault, "disconnectSeen": disconnect,
        "lastPhase": last.get("phase"), "lastSituation": last.get("situation"),
        "lastUT": last.get("ut"), "lastTick": last.get("tick"),
    }

def analyze(vehicle_path: Path, planner_path: Path | None = None, overshoot_range_m: float = 30000.0, overshoot_cross_m: float = 5000.0) -> dict[str, Any]:
    vehicle_path, inferred_planner = infer_pair(vehicle_path, planner_path)
    planner_path = inferred_planner
    vehicle = load_vehicle(vehicle_path)
    planner = load_planner(planner_path)
    rows = vehicle["rows"]
    planner_records = planner["records"]
    configuration = ((vehicle.get("sessionStart") or {}).get("configuration") or {})
    guidance_cfg = configuration.get("guidance") or {}
    vehicle_cfg = configuration.get("vehicle") or {}
    site_cfg = configuration.get("site") or {}
    minimum_leg_duration_s = float(guidance_cfg.get("sTurnMinimumLegDuration", 24.0))
    thermal_floor_deg = float(vehicle_cfg.get("entryAngleOfAttack", 18.0))
    maximum_g_load = float(vehicle_cfg.get("maximumGLoad", 3.5))
    minimum_safe_speed = float(vehicle_cfg.get("minimumSafeSpeed", 85.0))
    runway_length_m = float(site_cfg.get("runwayLength", 2500.0))
    runway_width_m = float(site_cfg.get("runwayWidth", 70.0))
    pairing_valid = bool(vehicle.get("sessionId") and planner.get("sessionId") and vehicle["sessionId"] == planner["sessionId"])
    handoff = first_handoff(rows)
    taem_kinematics = taem_kinematic_prerequisite_summary(rows)
    session_start = vehicle.get("sessionStart") or {}
    physics_context = copy.deepcopy(session_start.get("physicsContext") or {})
    observed_aero = observed_aero_force_summary(rows)
    phases = phase_summary(rows)
    common_ranges = common_range_summary(rows)
    thermal_aoa = thermal_aoa_summary(rows, thermal_floor_deg, maximum_g_load, minimum_safe_speed)
    reversals = reversal_summary(rows, planner_records, minimum_leg_duration_s)
    lineage = lineage_summary(planner_records)
    plan_identity = plan_identity_summary(rows, planner_records)
    plan_churn = entry_plan_churn_summary(planner_records)
    terminal = terminal_summary(planner_records)
    terminal_smoothness = terminal_smoothness_summary(rows)
    has_runtime, runtime_by_phase = direct_runtime_latencies(rows)
    runtime_gap_events = direct_runtime_gap_events(rows) if has_runtime else []
    inferred_by_phase, wall_gaps = inferred_wall_latencies(rows)
    latency_source = "fields.runtime" if has_runtime else "inferred tick-to-tick wallMonotonicSeconds"
    latency_by_phase = runtime_by_phase if has_runtime else inferred_by_phase
    taem_runtime_gaps = [gap for gap in runtime_gap_events if gap["phase"] == "TAEM"]
    taem_inferred_gaps = [gap for gap in wall_gaps if gap["sourcePhase"] == "TAEM"]
    taem_gaps = taem_runtime_gaps if has_runtime else taem_inferred_gaps
    outcome = outcome_summary(rows, planner_records, runway_length_m, runway_width_m)
    last_row = rows[-1] if rows else {}
    current_state = {k: last_row.get(k) for k in ("tick", "ut", "phase", "altitude", "tas", "range", "along", "cross", "heading", "flightPathAngle")}

    flags: list[dict[str, Any]] = []
    if planner.get("missing"):
        flags.append({"code": "MISSING_PLANNER_STREAM", "severity": "error"})
    elif not pairing_valid:
        flags.append({"code": "SESSION_PAIR_MISMATCH", "severity": "error", "vehicleSession": vehicle.get("sessionId"), "plannerSession": planner.get("sessionId")})
    if vehicle["decodeErrors"] or planner["decodeErrors"]:
        flags.append({"code": "JSONL_DECODE_ERRORS", "severity": "error", "vehicleLines": vehicle["decodeErrors"], "plannerLines": planner["decodeErrors"]})
    if handoff is None:
        incomplete = outcome["outcome"].startswith("incomplete")
        flags.append({"code": "NO_MM304_TO_TAEM_HANDOFF_YET" if incomplete else "NO_MM304_TO_TAEM_HANDOFF", "severity": "warning" if incomplete else "error"})
    else:
        altitude = handoff.get("altitudeM")
        speed = handoff.get("trueAirSpeedMps")
        along = handoff.get("runwayAlongTrackM")
        near = finite(handoff.get("rangeToSiteM")) and float(handoff["rangeToSiteM"]) <= overshoot_range_m
        cross = finite(handoff.get("runwayCrossTrackM")) and abs(float(handoff["runwayCrossTrackM"])) >= overshoot_cross_m
        if finite(altitude) and not (15000.0 <= float(altitude) <= 18000.0):
            flags.append({"code": "TAEM_HANDOFF_ALTITUDE_OUTSIDE_15_18KM", "severity": "error", "altitudeM": altitude})
        if finite(speed) and float(speed) > 1300.5:
            flags.append({"code": "TAEM_HANDOFF_SPEED_ABOVE_1300", "severity": "error", "trueAirSpeedMps": speed})
        vertical_speed = handoff.get("verticalSpeedMps")
        if finite(vertical_speed) and float(vertical_speed) >= 0.0:
            flags.append({"code": "TAEM_HANDOFF_NOT_DESCENDING", "severity": "error",
                          "verticalSpeedMps": vertical_speed})
        if finite(along) and float(along) >= 0.0:
            flags.append({"code": "TAEM_HANDOFF_PAST_RUNWAY_THRESHOLD", "severity": "error", "runwayAlongTrackM": along})
        elif finite(along) and float(along) > -10000.0:
            flags.append({"code": "TAEM_HANDOFF_LOW_UPSTREAM_RESERVE", "severity": "warning", "runwayAlongTrackM": along})
        if near and cross:
            flags.append({"code": "HISTORIC_1605_OVERSHOOT_SIGNATURE", "severity": "error", "rangeThresholdM": overshoot_range_m, "crossTrackThresholdM": overshoot_cross_m})
    if len(taem_gaps) >= 2:
        flags.append({"code": "REPEATED_TAEM_LOOP_GAPS_OVER_2S", "severity": "error", "count": len(taem_gaps), "source": latency_source})
    elif taem_gaps:
        flags.append({"code": "TAEM_LOOP_GAP_OVER_2S", "severity": "warning", "count": len(taem_gaps), "source": latency_source})
    if thermal_aoa["unexplainedBelowFloorCount"]:
        flags.append({"code": "MM304_TARGET_AOA_BELOW_THERMAL_FLOOR", "severity": "error",
                      "count": thermal_aoa["unexplainedBelowFloorCount"],
                      "floorDeg": thermal_aoa["floorDeg"],
                      "first": thermal_aoa["firstUnexplainedBelowFloor"]})
    flags.extend(entry_corridor_flags(common_ranges))
    if reversals["plannerVehicleMismatchCount"]:
        flags.append({"code": "EXECUTABLE_REVERSAL_METADATA_MISMATCH", "severity": "error", "count": reversals["plannerVehicleMismatchCount"]})
    if reversals["effectiveCapturePastThresholdCount"]:
        flags.append({"code": "REVERSAL_CAPTURE_PAST_RUNWAY_THRESHOLD", "severity": "error", "count": reversals["effectiveCapturePastThresholdCount"]})
    if reversals["effectiveCaptureMissingCount"]:
        incomplete = outcome["outcome"].startswith("incomplete")
        flags.append({"code": "REVERSAL_CAPTURE_MISSING", "severity": "warning" if incomplete else "error", "count": reversals["effectiveCaptureMissingCount"]})
    if reversals["effectiveCaptureLowReserveCount"]:
        flags.append({"code": "REVERSAL_CAPTURE_LOW_UPSTREAM_RESERVE", "severity": "warning", "count": reversals["effectiveCaptureLowReserveCount"]})
    if reversals["lateExecutableReversalCount"]:
        flags.append({"code": "REVERSAL_EXECUTION_MORE_THAN_2S_AFTER_PHYSICAL_DEADLINE", "severity": "error",
                      "count": reversals["lateExecutableReversalCount"]})
    if reversals["publishedBeforePhysicalDwellCount"]:
        flags.append({"code": "REVERSAL_PUBLISHED_BEFORE_PHYSICAL_DWELL", "severity": "error",
                      "count": reversals["publishedBeforePhysicalDwellCount"]})
    if lineage["available"] and lineage["continuityBreakCount"]:
        flags.append({"code": "PLANNER_LINEAGE_BREAK", "severity": "error", "count": lineage["continuityBreakCount"]})
    if lineage["available"] and not plan_identity["vehicleIdentityAvailable"]:
        flags.append({"code": "VEHICLE_PLAN_IDENTITY_UNAVAILABLE", "severity": "warning" if outcome["outcome"].startswith("incomplete") else "error"})
    elif plan_identity["missingTransitionCount"]:
        flags.append({"code": "PLAN_IDENTITY_TRANSITION_SAMPLE_MISSING", "severity": "warning" if outcome["outcome"].startswith("incomplete") else "error", "count": plan_identity["missingTransitionCount"]})
    if plan_identity["mismatchCount"]:
        flags.append({"code": "PLAN_IDENTITY_MISMATCH", "severity": "error", "count": plan_identity["mismatchCount"]})
    if plan_churn["rapidTransitionCount"] >= 10:
        flags.append({"code": "MM304_PLAN_REGENERATION_CHURN", "severity": "error",
                      "rapidTransitionCount": plan_churn["rapidTransitionCount"],
                      "maximumConsecutiveRapidTransitions": plan_churn["maximumConsecutiveRapidTransitions"],
                      "minimumTransitionIntervalSeconds": plan_churn["minimumTransitionIntervalSeconds"],
                      "transitionRateHz": plan_churn["transitionRateHz"],
                      "firstRapidTransition": plan_churn["firstRapidTransition"]})
    elif plan_churn["maximumConsecutiveFrequentTransitions"] >= 10:
        flags.append({"code": "MM304_SUSTAINED_REPLAN_LOOP", "severity": "error",
                      "frequentIntervalThresholdSeconds": plan_churn["frequentIntervalThresholdSeconds"],
                      "frequentTransitionCount": plan_churn["frequentTransitionCount"],
                      "maximumConsecutiveFrequentTransitions": plan_churn["maximumConsecutiveFrequentTransitions"],
                      "minimumTransitionIntervalSeconds": plan_churn["minimumTransitionIntervalSeconds"],
                      "transitionRateHz": plan_churn["transitionRateHz"],
                      "firstFrequentTransition": plan_churn["firstFrequentTransition"]})
    if outcome["splashedSeen"]:
        flags.append({"code": "SPLASHDOWN_OUTCOME", "severity": "error", "contact": outcome["contact"]})
    if outcome["landedSeen"] and not outcome["landingSequenceValid"]:
        flags.append({"code": "LANDING_SEQUENCE_INCOMPLETE", "severity": "error",
                      "finalSeen": outcome["finalSeen"], "flareSeen": outcome["flareSeen"],
                      "touchdownPhaseSeen": outcome["touchdownPhaseSeen"]})
    if outcome["landedSeen"] and not outcome["runwayContactValid"]:
        flags.append({"code": "LANDED_OUTSIDE_RUNWAY", "severity": "error", "contact": outcome["contact"],
                      "runwayEnvelope": outcome["runwayEnvelope"]})
    if outcome["touchdownPhaseSeen"] and not outcome["landedSeen"]:
        flags.append({"code": "TOUCHDOWN_NOT_CONFIRMED_LANDED", "severity": "error"})
    if outcome["faultSeen"]:
        flags.append({"code": "FAULT_OUTCOME", "severity": "error"})
    if outcome["abortSeen"]:
        flags.append({"code": "ABORT_OUTCOME", "severity": "error"})
    elif outcome["outcome"].startswith("incomplete"):
        flags.append({"code": "INCOMPLETE_RUN", "severity": "warning", "disconnect": outcome["disconnectSeen"]})

    severity = {"error": 2, "warning": 1}
    maximum = max((severity.get(f["severity"], 0) for f in flags), default=0)
    acceptance = "FAIL" if maximum >= 2 else "WARN" if maximum == 1 else "PASS"
    return {
        "schemaVersion": 2,
        "acceptance": acceptance,
        "vehicle": {k: vehicle[k] for k in ("path", "sessionId", "schemaVersion")},
        "planner": {"path": planner.get("path"), "paths": planner.get("paths", []), "sessionId": planner.get("sessionId"), "missing": planner.get("missing")},
        "sessionPairingValid": pairing_valid,
        "phaseSummary": phases,
        "ownershipHandoff": handoff,
        "taemKinematicPrerequisites": taem_kinematics,
        "physicsContext": physics_context,
        "observedAeroForceMilestones": observed_aero,
        "currentState": current_state,
        "entryCommonRange": common_ranges,
        "thermalAoA": thermal_aoa,
        "entryReversals": reversals,
        "plannerLineage": lineage,
        "vehiclePlannerPlanIdentity": plan_identity,
        "entryPlanChurn": plan_churn,
        "terminalProgress": terminal,
        "terminalSmoothness": terminal_smoothness,
        "runtimeLatency": {
            "source": latency_source,
            "byPhase": latency_by_phase,
            "directGapEventsOver2Seconds": runtime_gap_events,
            "wallGapEventsOver2Seconds": wall_gaps,
            "taemSourcePhaseGapCountOver2Seconds": len(taem_gaps),
        },
        "outcome": outcome,
        "thresholds": {
            "taemHandoffAltitudeMinM": 15000.0, "taemHandoffAltitudeMaxM": 18000.0,
            "taemHandoffSpeedMaxMps": 1300.5, "taemUpstreamReserveWarningM": 10000.0,
            "entryThermalAoAFloorDeg": thermal_floor_deg, "sTurnMinimumLegDurationSeconds": minimum_leg_duration_s,
            "entryDebt300RejectM": 10000.0, "entryDebt200RejectM": 7000.0, "entryAltitude100RejectM": 21000.0,
            "reversalExecutionDelayRejectSeconds": 2.0, "reversalPhysicalDwellDeferralRejectSeconds": 2.0,
            "historicOvershootRangeM": overshoot_range_m, "historicOvershootCrossTrackM": overshoot_cross_m,
            "loopGapWarningMsec": 2000.0, "mm304PlanRapidTransitionSeconds": 0.5,
            "mm304PlanRapidTransitionRejectCount": 10,
            "runwayContactAlongMinM": -40.0, "runwayContactAlongMaxExclusiveM": runway_length_m + 80.0,
            "runwayContactCrossAbsMaxExclusiveM": max(runway_width_m * 0.75, 45.0),
        },
        "flags": flags,
    }


def fmt(value: Any, digits: int = 1) -> str:
    return "n/a" if not finite(value) else f"{float(value):.{digits}f}"


def km(value: Any) -> float | None:
    return float(value) / 1000.0 if finite(value) else None


def human(report: dict[str, Any]) -> str:
    lines = [f"75 km postflight acceptance: {report['acceptance']}"]
    lines.append(f"session: {report['vehicle'].get('sessionId') or 'unknown'} | paired={report['sessionPairingValid']}")
    physics_context = report.get("physicsContext") or {}
    if physics_context:
        lines.append(
            "physics context: samples=%s structure=%s environment=%s" % (
                physics_context.get("historicalSamplesLoaded", "n/a"),
                physics_context.get("structureId") or "n/a",
                physics_context.get("environmentId") or "n/a",
            )
        )
    aero_milestones = report.get("observedAeroForceMilestones") or {}
    if aero_milestones:
        pieces = []
        for key in ("q300", "q360", "q500", "q700", "q1000", "q1300"):
            point = aero_milestones.get(key)
            if point:
                pieces.append(
                    f"{key}:q={fmt(point.get('observedDynamicPressurePa'),1)}Pa "
                    f"D={fmt(point.get('dragAccelerationMps2'),3)} L={fmt(point.get('liftAccelerationMps2'),3)}m/s2 "
                    f"L/D={fmt(point.get('liftToDrag'),3)} resConf={fmt(point.get('physicsForceResidualConfidence'),3)} "
                    f"unc={fmt(point.get('physicsCertifiedUncertainty'),3)} liveN={point.get('physicsLiveSamples') if point.get('physicsLiveSamples') is not None else 'n/a'}"
                )
        lines.append("MM304 observed aero (evidence only): " + " | ".join(pieces))
    h = report.get("ownershipHandoff")
    if h:
        lines.append(
            "MM304->TAEM: UT %s | alt %s km | TAS %s m/s | range %s km | along %s km | cross %s km | hdg %s deg | FPA %s deg" % (
                fmt(h.get("ut"), 3), fmt(km(h.get("altitudeM")), 2), fmt(h.get("trueAirSpeedMps"), 1),
                fmt(km(h.get("rangeToSiteM")), 2), fmt(km(h.get("runwayAlongTrackM")), 2),
                fmt(km(h.get("runwayCrossTrackM")), 2), fmt(h.get("headingDeg"), 1), fmt(h.get("flightPathAngleDeg"), 1),
            )
        )
    else:
        c = report.get("currentState") or {}
        lines.append("MM304->TAEM: not observed | current %s UT %s alt %s km TAS %s m/s range %s km along %s km cross %s km" % (
            c.get("phase") or "unknown", fmt(c.get("ut"), 3), fmt(km(c.get("altitude")), 2), fmt(c.get("tas"), 1),
            fmt(km(c.get("range")), 2), fmt(km(c.get("along")), 2), fmt(km(c.get("cross")), 2)))
    kin = report.get("taemKinematicPrerequisites") or {}
    first_speed = kin.get("firstSpeedAtOrBelowMax") or {}
    first_altitude = kin.get("firstAltitudeAtOrBelowMax") or {}
    lines.append(
        "TAEM basic kinematics (evidence only): upstreamWindow=%s | speed<=%s first alt=%skm range=%skm along=%skm | alt<=%skm first TAS=%sm/s range=%skm along=%skm" % (
            kin.get("basicUpstreamWindowSeen", False), fmt(kin.get("speedMaxMps"), 1),
            fmt(km(first_speed.get("altitudeM")), 2), fmt(km(first_speed.get("rangeToSiteM")), 2),
            fmt(km(first_speed.get("runwayAlongTrackM")), 2), fmt(km(kin.get("altitudeMaxM")), 1),
            fmt(first_altitude.get("trueAirSpeedMps"), 1), fmt(km(first_altitude.get("rangeToSiteM")), 2),
            fmt(km(first_altitude.get("runwayAlongTrackM")), 2),
        )
    )
    thermal = report.get("thermalAoA") or {}
    lines.append(f"MM304 thermal AoA: floor={fmt(thermal.get('floorDeg'), 1)} deg minTarget={fmt(thermal.get('minimumTargetAoADeg'), 2)} deg samples={thermal.get('sampleCount', 0)} below={thermal.get('belowFloorCount', 0)} safety={thermal.get('explicitSafetyBelowFloorCount', 0)} recovery={thermal.get('safetyRecoveryTransientBelowFloorCount', 0)} unexplained={thermal.get('unexplainedBelowFloorCount', 0)}")
    checkpoints = report.get("entryCommonRange") or {}
    if checkpoints:
        pieces = []
        for key in ("400km", "300km", "200km", "100km", "72km"):
            point = checkpoints.get(key)
            if point:
                debt_delta = point.get("altitudeDebtDeltaFromPreviousStationM")
                debt_trend = ""
                if finite(debt_delta):
                    trend = "worse" if float(debt_delta) > 0.0 else "better" if float(debt_delta) < 0.0 else "flat"
                    debt_trend = f" dDebt={fmt(km(debt_delta),2)}km({trend})"
                pieces.append(
                    f"{key}:alt={fmt(km(point.get('altitudeM')),2)}km debt={fmt(km(point.get('altitudeDebtM')),2)}km{debt_trend} "
                    f"FPA={fmt(point.get('flightPathAngleDeg'),2)} VS={fmt(point.get('verticalSpeedMps'),1)}m/s "
                    f"q={fmt(point.get('dynamicPressurePa'),1)}Pa g={fmt(point.get('gForce'),3)} "
                    f"excess={fmt(km(point.get('energyExcessRangeM')),1)}km"
                )
        lines.append("MM304 common-range: " + " | ".join(pieces))
    t = report["terminalProgress"]
    lines.append(f"TAEM planner: samples={t['plannerTAEMSamples']} committed={t['terminalCommitted']} maxTransition={fmt(t['maximumHACTransitionProgress'], 3)} circuits={fmt(t['maximumHACCircuitCount'], 0)}")
    smooth = report.get("terminalSmoothness") or {}
    targets = smooth.get("targetContinuity") or {}
    controls = smooth.get("appliedControlReversals") or {}
    lines.append(
        "terminal smoothness (evidence only): samples=%s phases=%s maxTargetStep[pitch=%s roll=%s heading=%s aoa=%s] deg" % (
            smooth.get("terminalSampleCount", 0), ",".join(smooth.get("phasesPresent") or []) or "none",
            fmt((targets.get("targetPitch") or {}).get("maximumStepDeg"), 2),
            fmt((targets.get("targetRoll") or {}).get("maximumStepDeg"), 2),
            fmt((targets.get("targetHeading") or {}).get("maximumStepDeg"), 2),
            fmt((targets.get("targetAoA") or {}).get("maximumStepDeg"), 2),
        )
    )
    lines.append(
        "terminal control reversals (evidence only): pitch=%s/%s rapid roll=%s/%s rapid yaw=%s/%s rapid" % (
            (controls.get("appliedPitch") or {}).get("reversalCount", 0),
            (controls.get("appliedPitch") or {}).get("rapidReversalCount", 0),
            (controls.get("appliedRoll") or {}).get("reversalCount", 0),
            (controls.get("appliedRoll") or {}).get("rapidReversalCount", 0),
            (controls.get("appliedYaw") or {}).get("reversalCount", 0),
            (controls.get("appliedYaw") or {}).get("rapidReversalCount", 0),
        )
    )
    latency = report["runtimeLatency"]
    taem = (latency.get("byPhase") or {}).get("TAEM", {})
    loop = taem.get("loopWallDeltaMilliseconds", {})
    lines.append(f"TAEM latency ({latency['source']}): n={loop.get('count', 0)} p95={fmt(loop.get('p95'), 1)} ms p99={fmt(loop.get('p99'), 1)} ms max={fmt(loop.get('max'), 1)} ms >2s(source-phase)={latency['taemSourcePhaseGapCountOver2Seconds']}")

    r = report["entryReversals"]
    lines.append(f"reversal parity: comparable={r['comparablePlannerVehicleSamples']} mismatches={r['plannerVehicleMismatchCount']} unpairedPlanner={r['unpairedPlannerReversalSamples']} publishedBeforeDwell={r['publishedBeforePhysicalDwellCount']} effectiveMissing={r['effectiveCaptureMissingCount']} latePhysical={r['lateExecutableReversalCount']} effectivePastThreshold={r['effectiveCapturePastThresholdCount']} lowReserve={r['effectiveCaptureLowReserveCount']}")
    pi = report["vehiclePlannerPlanIdentity"]
    lines.append(f"plan identity: vehicle={pi['vehicleIdentityAvailable']} transitions={pi['plannerPlanTransitionCount']} comparable={pi['comparableTransitionCount']} missing={pi['missingTransitionCount']} mismatches={pi['mismatchCount']}")
    churn = report.get("entryPlanChurn") or {}
    lines.append(f"plan churn: rapid<={fmt(churn.get('rapidIntervalThresholdSeconds'),2)}s count={churn.get('rapidTransitionCount',0)} maxBurst={churn.get('maximumConsecutiveRapidTransitions',0)} minDt={fmt(churn.get('minimumTransitionIntervalSeconds'),3)}s rate={fmt(churn.get('transitionRateHz'),2)}Hz")
    o = report["outcome"]
    lines.append(f"outcome: {o['outcome']} | final={o['finalSeen']} flare={o['flareSeen']} touchdown={o['touchdownSeen']} abort={o['abortSeen']} disconnect={o['disconnectSeen']}")
    if report["flags"]:
        lines.append("flags: " + ", ".join(f["code"] for f in report["flags"]))
    else:
        lines.append("flags: none")
    return "\n".join(lines)


def newest_vehicle(root: Path) -> Path | None:
    candidates = list((root / "FlightLogs").glob("*-vehicle.jsonl"))
    return max(candidates, key=lambda p: p.stat().st_mtime) if candidates else None


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("path", nargs="?", type=Path, help="vehicle or planner split JSONL; defaults to newest *-vehicle.jsonl")
    parser.add_argument("--planner", type=Path, help="explicit paired planner stream")
    parser.add_argument("--root", type=Path, default=Path(__file__).resolve().parents[1])
    parser.add_argument("--json-output", type=Path)
    parser.add_argument("--json-only", action="store_true")
    parser.add_argument("--overshoot-range-km", type=float, default=30.0)
    parser.add_argument("--overshoot-cross-km", type=float, default=5.0)
    args = parser.parse_args()
    root = args.root.resolve()
    path = args.path
    if path is None:
        path = newest_vehicle(root)
        if path is None:
            parser.error("no split vehicle logs found")
    elif not path.is_absolute():
        path = root / path
    planner = args.planner
    if planner is not None and not planner.is_absolute():
        planner = root / planner
    report = analyze(path, planner, args.overshoot_range_km * 1000.0, args.overshoot_cross_km * 1000.0)
    encoded = json.dumps(report, indent=2, sort_keys=True)
    if args.json_output:
        output = args.json_output if args.json_output.is_absolute() else root / args.json_output
        output.parent.mkdir(parents=True, exist_ok=True)
        output.write_text(encoded + "\n", encoding="utf-8")
    print(encoded if args.json_only else human(report))
    return 0 if report["acceptance"] != "FAIL" else 1


if __name__ == "__main__":
    raise SystemExit(main())
