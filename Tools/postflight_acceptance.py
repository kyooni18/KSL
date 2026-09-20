#!/usr/bin/env python3
"""Summarize a KSPShuttleLander split flight-log session for 75 km acceptance."""
from __future__ import annotations

import argparse
import copy
import json
import math
from pathlib import Path
from typing import Any

RUNTIME_KEYS = (
    "loopWallDeltaMilliseconds",
    "controlLoopMilliseconds",
    "telemetryLatencyMilliseconds",
    "guidanceComputeMilliseconds",
    "applyLatencyMilliseconds",
)


def finite(value: Any) -> float | None:
    try:
        number = float(value)
    except (TypeError, ValueError):
        return None
    return number if math.isfinite(number) else None


def percentile(values: list[float], fraction: float) -> float | None:
    if not values:
        return None
    ordered = sorted(values)
    pos = (len(ordered) - 1) * fraction
    lo = int(pos)
    hi = min(lo + 1, len(ordered) - 1)
    blend = pos - lo
    return ordered[lo] * (1.0 - blend) + ordered[hi] * blend


def metric_stats(values: list[float]) -> dict[str, Any]:
    return {
        "count": len(values),
        "p50": percentile(values, .50),
        "p95": percentile(values, .95),
        "p99": percentile(values, .99),
        "max": max(values) if values else None,
    }


def merge_fields(base: dict[str, Any], patch: dict[str, Any]) -> None:
    for key, value in patch.items():
        if isinstance(value, dict) and isinstance(base.get(key), dict):
            merge_fields(base[key], value)
        elif isinstance(value, dict):
            base[key] = copy.deepcopy(value)
        else:
            base[key] = value


def infer_pair(path: Path) -> tuple[Path, Path]:
    name = path.name
    if name.endswith("-vehicle.jsonl"):
        return path, path.with_name(name.removesuffix("-vehicle.jsonl") + "-planner.jsonl")
    if name.endswith("-planner.jsonl"):
        return path.with_name(name.removesuffix("-planner.jsonl") + "-vehicle.jsonl"), path
    raise ValueError("input must be a split *-vehicle.jsonl or *-planner.jsonl log")


def load_vehicle(path: Path) -> dict[str, Any]:
    fields: dict[str, Any] = {}
    rows: list[dict[str, Any]] = []
    session_id = None
    native_build = None
    configuration: dict[str, Any] = {}
    with path.open("r", encoding="utf-8") as handle:
        for raw in handle:
            try:
                record = json.loads(raw)
            except json.JSONDecodeError:
                continue
            kind = record.get("recordType")
            if kind == "sessionStart":
                session_id = record.get("sessionId") or session_id
                native_build = (record.get("runtime") or {}).get("nativeBuild") or native_build
                configuration = record.get("configuration") or configuration
                continue
            patch = record.get("fields") or {}
            if kind == "vehicleKeyframe":
                fields = copy.deepcopy(patch)
            elif kind == "vehicleDelta":
                merge_fields(fields, patch)
            else:
                continue
            rows.append({
                "recordSequence": record.get("recordSequence"),
                "tickSequence": record.get("tickSequence"),
                "ut": finite(record.get("ut")),
                "wall": finite(record.get("wallMonotonicSeconds")),
                "event": str(record.get("event") or ""),
                "runtime": copy.deepcopy(patch.get("runtime")) if isinstance(patch.get("runtime"), dict) else None,
                "fields": copy.deepcopy(fields),
            })
    return {"path": str(path), "sessionId": session_id, "nativeBuild": native_build, "configuration": configuration, "rows": rows}


def load_planner(path: Path) -> dict[str, Any]:
    rows: list[dict[str, Any]] = []
    session_id = None
    if not path.exists():
        return {"path": str(path), "sessionId": None, "rows": [], "missing": True}
    with path.open("r", encoding="utf-8") as handle:
        for raw in handle:
            try:
                record = json.loads(raw)
            except json.JSONDecodeError:
                continue
            if record.get("recordType") == "sessionStart":
                session_id = record.get("sessionId") or session_id
            elif record.get("recordType") == "plannerSample":
                rows.append(record)
    return {"path": str(path), "sessionId": session_id, "rows": rows, "missing": False}


def field(row: dict[str, Any], group: str, key: str) -> Any:
    return ((row.get("fields") or {}).get(group) or {}).get(key)


def phase(row: dict[str, Any]) -> str:
    return str(field(row, "state", "phase") or "unknown")


def phase_transitions(rows: list[dict[str, Any]]) -> list[dict[str, Any]]:
    out: list[dict[str, Any]] = []
    previous = None
    for row in rows:
        current = phase(row)
        if current != previous:
            out.append({"phase": current, "ut": row.get("ut"), "tickSequence": row.get("tickSequence")})
            previous = current
    return out



def handoff_summary(rows: list[dict[str, Any]]) -> dict[str, Any]:
    row = next((item for item in rows if phase(item).lower() == "taem"), None)
    if row is None:
        return {"found": False}
    altitude = finite(field(row, "position", "altitude"))
    speed = finite(field(row, "motion", "trueAirSpeed"))
    distance = finite(field(row, "guidance", "rangeToSite"))
    cross = finite(field(row, "guidance", "runwayCrossTrack"))
    bad_geometry = distance is not None and cross is not None and distance <= 20000.0 and abs(cross) >= 5000.0
    return {
        "found": True,
        "ut": row.get("ut"),
        "tickSequence": row.get("tickSequence"),
        "altitudeMeters": altitude,
        "trueAirSpeedMps": speed,
        "rangeToSiteMeters": distance,
        "runwayCrossTrackMeters": cross,
        "altitudeIn15To18KmCorridor": altitude is not None and 15000.0 <= altitude <= 18000.0,
        "atOrBelow1300Mps": speed is not None and speed <= 1300.0 + 1e-6,
        "nearSiteLargeCrossTrack": bad_geometry,
    }



def runtime_samples(rows: list[dict[str, Any]]) -> tuple[dict[str, list[float]], str]:
    direct = {key: [] for key in RUNTIME_KEYS}
    found = False
    for row in rows:
        runtime = row.get("runtime") or {}
        for key in RUNTIME_KEYS:
            value = finite(runtime.get(key))
            if value is not None:
                direct[key].append(value)
                found = True
    if found:
        return direct, "runtimeFields"
    inferred = {key: [] for key in RUNTIME_KEYS}
    previous = None
    for row in rows:
        if previous is not None:
            tick = row.get("tickSequence")
            prev_tick = previous.get("tickSequence")
            wall = row.get("wall")
            prev_wall = previous.get("wall")
            if isinstance(tick, int) and isinstance(prev_tick, int) and tick == prev_tick + 1 and wall is not None and prev_wall is not None:
                inferred["loopWallDeltaMilliseconds"].append(max(0.0, (wall - prev_wall) * 1000.0))
        previous = row
    return inferred, "inferredConsecutiveTickWallGaps"



def runtime_outliers(rows: list[dict[str, Any]], threshold_ms: float = 500.0) -> list[dict[str, Any]]:
    direct = any(row.get("runtime") for row in rows)
    out = []
    previous = None
    for row in rows:
        current_runtime = row.get("runtime") or {}
        loop = finite(current_runtime.get("loopWallDeltaMilliseconds")) if direct else None
        if not direct and previous is not None:
            tick, prev_tick = row.get("tickSequence"), previous.get("tickSequence")
            wall, prev_wall = row.get("wall"), previous.get("wall")
            if isinstance(tick, int) and isinstance(prev_tick, int) and tick == prev_tick + 1 and wall is not None and prev_wall is not None:
                loop = max(0.0, (wall - prev_wall) * 1000.0)
        if loop is not None and loop >= threshold_ms:
            cause = (previous or {}).get("runtime") or {}
            out.append({
                "ut": row.get("ut"), "tickSequence": row.get("tickSequence"), "phase": phase(row),
                "loopWallDeltaMilliseconds": loop,
                "precedingTickSequence": (previous or {}).get("tickSequence"),
                "precedingControlLoopMilliseconds": finite(cause.get("controlLoopMilliseconds")),
                "precedingTelemetryLatencyMilliseconds": finite(cause.get("telemetryLatencyMilliseconds")),
                "precedingGuidanceComputeMilliseconds": finite(cause.get("guidanceComputeMilliseconds")),
                "precedingApplyLatencyMilliseconds": finite(cause.get("applyLatencyMilliseconds")),
                "source": "runtimeFields" if direct else "inferredConsecutiveTickWallGaps",
            })
        previous = row
    return sorted(out, key=lambda item: item["loopWallDeltaMilliseconds"], reverse=True)[:20]

def runtime_summary(rows: list[dict[str, Any]]) -> dict[str, Any]:
    all_values, source = runtime_samples(rows)
    phase_names = []
    for row in rows:
        name = phase(row)
        if name not in phase_names:
            phase_names.append(name)
    by_phase = {}
    for name in phase_names:
        values, phase_source = runtime_samples([row for row in rows if phase(row) == name])
        by_phase[name] = {
            "source": phase_source,
            "metrics": {key: metric_stats(samples) for key, samples in values.items()},
        }
    taem_rows = [row for row in rows if phase(row).lower() == "taem"]
    taem_values, taem_source = runtime_samples(taem_rows)
    taem_long = [value for value in taem_values["loopWallDeltaMilliseconds"] if value > 2000.0]
    handoff_index = next((index for index, row in enumerate(rows) if phase(row).lower() == "taem"), None)
    post_rows = rows[handoff_index:] if handoff_index is not None else []
    post_values, post_source = runtime_samples(post_rows)
    post_long = [value for value in post_values["loopWallDeltaMilliseconds"] if value > 2000.0]
    return {
        "source": source,
        "allPhases": {key: metric_stats(values) for key, values in all_values.items()},
        "byPhase": by_phase,
        "taemSource": taem_source,
        "taem": {key: metric_stats(values) for key, values in taem_values.items()},
        "taemLoopGapsOver2Seconds": len(taem_long),
        "taemLoopGapOver2SecondsValues": taem_long[:20],
        "repeatedTaemMultiSecondLoopGaps": len(taem_long) >= 2,
        "postHandoffSource": post_source,
        "postHandoff": {key: metric_stats(values) for key, values in post_values.items()},
        "postHandoffLoopGapsOver2Seconds": len(post_long),
        "postHandoffLoopGapOver2SecondsValues": post_long[:20],
        "repeatedPostHandoffMultiSecondLoopGaps": len(post_long) >= 2,
        "outliersOver500Milliseconds": runtime_outliers(rows),
    }

def reversal_summary(rows: list[dict[str, Any]]) -> dict[str, Any]:
    event_rows = []
    starts = 0
    last = False
    for row in rows:
        text = row.get("event") or ""
        if "reversal" in text.lower():
            event_rows.append({"ut": row.get("ut"), "event": text})
        scheduled = bool(field(row, "state", "entryReversalScheduled"))
        if scheduled and not last:
            starts += 1
        last = scheduled
    return {"eventCount": len(event_rows), "events": event_rows[:20], "scheduleActivationCount": starts}



def planner_summary(rows: list[dict[str, Any]]) -> dict[str, Any]:
    candidate_ut = None
    committed_ut = None
    max_progress = None
    max_circuits = None
    min_remaining = None
    transitions = []
    previous_phase = None
    lineage_changes = 0
    previous_lineage = None
    first_entry_program = None
    for row in rows:
        ut = finite(row.get("ut"))
        current = str(row.get("phase") or "unknown")
        if current != previous_phase:
            transitions.append({"phase": current, "ut": ut})
            previous_phase = current
        planner_trace = row.get("plannerTrace") or {}
        row_plan = row.get("currentPlan") or {}
        if first_entry_program is None and planner_trace.get("inputSign") is not None and row_plan.get("valid"):
            first_entry_program = {
                "ut": ut,
                "plannedUT": finite(row_plan.get("plannedUT")),
                "inputSign": finite(planner_trace.get("inputSign")),
                "mode": planner_trace.get("mode"),
                "selectedSource": planner_trace.get("selectedSource"),
                "targetBank": finite(row_plan.get("targetBank")),
                "targetAoA": finite(row_plan.get("targetAoA")),
                "terminalReady": row_plan.get("terminalReady"),
                "planId": row_plan.get("planId"),
                "planVersion": row_plan.get("planVersion"),
                "plannedReversalUT": finite(row_plan.get("plannedReversalUT")),
                "plannedReversalRangeMeters": finite(row_plan.get("plannedReversalRange")),
                "plannedReversalSign": finite(row_plan.get("plannedReversalSign")),
            }
        geometry = row.get("terminalGeometry") or {}
        if geometry.get("terminalCandidateValid") and candidate_ut is None:
            candidate_ut = ut
        if geometry.get("terminalCommitted") and committed_ut is None:
            committed_ut = ut
        progress = finite(geometry.get("hacTransitionProgress"))
        circuits = finite(geometry.get("hacCircuitCount"))
        remaining = finite(geometry.get("hacRemaining"))
        if progress is not None:
            max_progress = progress if max_progress is None else max(max_progress, progress)
        if circuits is not None:
            max_circuits = circuits if max_circuits is None else max(max_circuits, circuits)
        if remaining is not None:
            min_remaining = remaining if min_remaining is None else min(min_remaining, remaining)
        lineage = row.get("lineage") or {}
        identity = (lineage.get("planId"), lineage.get("version"))
        if identity != (None, None):
            if previous_lineage is not None and identity != previous_lineage:
                lineage_changes += 1
            previous_lineage = identity
    latest = rows[-1] if rows else {}
    current_plan = latest.get("currentPlan") or {}
    trace = latest.get("plannerTrace") or {}
    latest_plan = {
        "ut": finite(latest.get("ut")),
        "phase": latest.get("phase"),
        "traceMode": trace.get("mode"),
        "planId": current_plan.get("planId"),
        "planVersion": current_plan.get("planVersion"),
        "terminalReady": current_plan.get("terminalReady"),
        "targetBank": finite(current_plan.get("targetBank")),
        "targetAoA": finite(current_plan.get("targetAoA")),
        "taemRangeErrorMeters": finite(current_plan.get("taemRangeError")),
        "taemSpeedMps": finite(current_plan.get("taemSpeed")),
        "taemEnergyError": finite(current_plan.get("taemEnergyError")),
        "closestDistanceMeters": finite(current_plan.get("closestDistance")),
    }
    return {
        "samples": len(rows),
        "terminalGeometryAvailable": any(bool(row.get("terminalGeometry")) for row in rows),
        "latestPlan": latest_plan,
        "firstEntryProgram": first_entry_program,
        "firstTerminalCandidateUT": candidate_ut,
        "firstTerminalCommittedUT": committed_ut,
        "maximumHACTransitionProgress": max_progress,
        "maximumHACCircuitCount": max_circuits,
        "minimumHACRemaining": min_remaining,
        "plannerLineageChanges": lineage_changes,
        "phaseTransitions": transitions,
    }



def latest_summary(rows: list[dict[str, Any]]) -> dict[str, Any]:
    if not rows:
        return {"available": False}
    row = rows[-1]
    return {
        "available": True,
        "ut": row.get("ut"),
        "tickSequence": row.get("tickSequence"),
        "phase": phase(row),
        "altitudeMeters": finite(field(row, "position", "altitude")),
        "trueAirSpeedMps": finite(field(row, "motion", "trueAirSpeed")),
        "rangeToSiteMeters": finite(field(row, "guidance", "rangeToSite")),
        "runwayCrossTrackMeters": finite(field(row, "guidance", "runwayCrossTrack")),
        "vesselSituation": field(row, "state", "vesselSituation"),
    }


def outcome_summary(rows: list[dict[str, Any]]) -> dict[str, Any]:
    if not rows:
        return {"complete": False, "reason": "no vehicle samples"}
    landed = next((row for row in rows if str(field(row, "state", "vesselSituation") or "").lower() == "landed"), None)
    aborted = next((row for row in rows if "abort" in phase(row).lower() or "abort" in (row.get("event") or "").lower()), None)
    faulted = next((row for row in rows if "fault" in phase(row).lower() or "fault" in (row.get("event") or "").lower()), None)
    disconnected = next((row for row in reversed(rows) if "disconnect" in (row.get("event") or "").lower()), None)
    final = rows[-1]
    return {
        "touchdownDetected": landed is not None,
        "touchdownUT": landed.get("ut") if landed else None,
        "abortDetected": aborted is not None,
        "abortUT": aborted.get("ut") if aborted else None,
        "faultDetected": faulted is not None,
        "faultUT": faulted.get("ut") if faulted else None,
        "disconnectDetected": disconnected is not None,
        "finalUT": final.get("ut"),
        "finalPhase": phase(final),
        "finalVesselSituation": field(final, "state", "vesselSituation"),
        "complete": landed is not None or aborted is not None or faulted is not None,
    }



def control_quality_summary(vehicle_rows: list[dict[str, Any]], planner_rows: list[dict[str, Any]], configuration: dict[str, Any]) -> dict[str, Any]:
    vehicle_cfg = configuration.get("vehicle") or {}
    maximum_aoa = finite(vehicle_cfg.get("maximumAngleOfAttack")) or 28.0
    stall_alpha_cap = max(4.0, maximum_aoa * .40)
    wrong_yaw = []
    maximum_abs_beta = 0.0
    for row in vehicle_rows:
        beta = finite(field(row, "attitude", "sideslip"))
        applied = (row.get("fields") or {}).get("appliedControl") or {}
        yaw = finite(applied.get("diagnosticYaw"))
        if yaw is None:
            yaw = finite(applied.get("reportedYaw"))
        if beta is not None:
            maximum_abs_beta = max(maximum_abs_beta, abs(beta))
        if beta is None or yaw is None or abs(beta) < 12.0 or abs(yaw) < .08:
            continue
        if beta * yaw < 0.0:
            wrong_yaw.append({
                "ut": row.get("ut"),
                "tickSequence": row.get("tickSequence"),
                "phase": phase(row),
                "sideslipDegrees": beta,
                "yawCommand": yaw,
            })

    alpha_violations = []
    bank_violations = []
    for row in planner_rows:
        planner_phase = str(row.get("phase") or "").lower()
        if "entry" not in planner_phase or "recovery" in planner_phase:
            continue
        constraints = row.get("constraints") or {}
        stall = finite(constraints.get("stallFraction"))
        stall_measured = constraints.get("stallFractionMeasured") is True
        plan = row.get("currentPlan") or {}
        if stall is None or not plan.get("valid"):
            continue
        target_aoa = finite(plan.get("targetAoA"))
        target_bank = finite(plan.get("targetBank"))
        if stall > .12 and target_aoa is not None and target_aoa > stall_alpha_cap + .25:
            alpha_violations.append({
                "ut": finite(row.get("ut")), "stallFraction": stall,
                "targetAoA": target_aoa, "allowedAoA": stall_alpha_cap,
                "planId": plan.get("planId"), "planVersion": plan.get("planVersion"),
            })
        if stall_measured and stall > .10 and target_bank is not None and abs(target_bank) > 24.25:
            bank_violations.append({
                "ut": finite(row.get("ut")), "stallFraction": stall,
                "targetBank": target_bank, "allowedAbsBank": 24.0,
                "planId": plan.get("planId"), "planVersion": plan.get("planVersion"),
            })
    return {
        "maximumAbsoluteSideslipDegrees": maximum_abs_beta,
        "wrongSignedBetaYawFrames": len(wrong_yaw),
        "wrongSignedBetaYawExamples": wrong_yaw[:20],
        "stallAlphaCapDegrees": stall_alpha_cap,
        "plannerStallAlphaSafetyViolations": len(alpha_violations),
        "plannerStallAlphaSafetyExamples": alpha_violations[:20],
        "plannerStallBankSafetyViolations": len(bank_violations),
        "plannerStallBankSafetyExamples": bank_violations[:20],
    }


def analyze(vehicle_path: Path, planner_path: Path | None = None) -> dict[str, Any]:
    if planner_path is None:
        vehicle_path, planner_path = infer_pair(vehicle_path)
    vehicle = load_vehicle(vehicle_path)
    planner = load_planner(planner_path)
    rows = vehicle["rows"]
    handoff = handoff_summary(rows)
    runtime = runtime_summary(rows)
    outcome = outcome_summary(rows)
    control_quality = control_quality_summary(rows, planner["rows"], vehicle.get("configuration") or {})
    session_match = bool(vehicle.get("sessionId") and planner.get("sessionId") and vehicle["sessionId"] == planner["sessionId"])
    result = {
        "schemaVersion": 1,
        "vehiclePath": vehicle["path"],
        "plannerPath": planner["path"],
        "sessionId": vehicle.get("sessionId") or planner.get("sessionId"),
        "sessionPairValid": session_match,
        "nativeBuild": vehicle.get("nativeBuild"),
        "vehicleSamples": len(rows),
        "plannerSamples": len(planner["rows"]),
        "phaseTransitions": phase_transitions(rows),
        "latestState": latest_summary(rows),
        "handoff": handoff,
        "entryReversals": reversal_summary(rows),
        "runtime": runtime,
        "controlQuality": control_quality,
        "terminalPlanner": planner_summary(planner["rows"]),
        "outcome": outcome,
    }
    return add_acceptance(result)


def add_acceptance(result: dict[str, Any]) -> dict[str, Any]:
    handoff = result["handoff"]
    runtime = result["runtime"]
    outcome = result["outcome"]
    control_quality = result["controlQuality"]
    first_program = (result["terminalPlanner"].get("firstEntryProgram") or {})
    flags = []
    if not result["sessionPairValid"]:
        flags.append("session-pair-missing-or-mismatched")
    if not handoff.get("found"):
        flags.append("no-mm305-taem-handoff-observed")
    elif handoff.get("nearSiteLargeCrossTrack"):
        flags.append("handoff-near-site-with-large-cross-track")
    if runtime["repeatedPostHandoffMultiSecondLoopGaps"]:
        flags.append("repeated-post-handoff-loop-gaps-over-2s")
    if control_quality["wrongSignedBetaYawFrames"]:
        flags.append("wrong-signed-beta-yaw-control")
    if control_quality["plannerStallAlphaSafetyViolations"]:
        flags.append("planner-stall-alpha-envelope-violation")
    if control_quality["plannerStallBankSafetyViolations"]:
        flags.append("planner-stall-bank-envelope-violation")
    first_sign = finite(first_program.get("inputSign"))
    if first_sign is not None and first_sign < 0:
        flags.append("first-entry-program-negative-side")
    if first_program.get("mode") == "infeasible":
        flags.append("first-entry-program-infeasible")
    if outcome.get("abortDetected"):
        flags.append("abort-observed")
    if outcome.get("faultDetected"):
        flags.append("fault-observed")
    if not outcome.get("complete"):
        flags.append("run-incomplete-no-touchdown-abort-or-fault")
    result["flags"] = flags
    result["acceptance"] = {
        "sessionPairValid": result["sessionPairValid"],
        "handoffObserved": bool(handoff.get("found")),
        "handoffAltitude15To18Km": bool(handoff.get("altitudeIn15To18KmCorridor")),
        "handoffAtOrBelow1300Mps": bool(handoff.get("atOrBelow1300Mps")),
        "handoffNotNearSiteLargeCrossTrack": bool(handoff.get("found")) and not bool(handoff.get("nearSiteLargeCrossTrack")),
        "noRepeatedTaemLoopGapsOver2Seconds": not runtime["repeatedTaemMultiSecondLoopGaps"],
        "noRepeatedPostHandoffLoopGapsOver2Seconds": not runtime["repeatedPostHandoffMultiSecondLoopGaps"],
        "noWrongSignedBetaYawControl": control_quality["wrongSignedBetaYawFrames"] == 0,
        "noPlannerStallAlphaSafetyViolation": control_quality["plannerStallAlphaSafetyViolations"] == 0,
        "firstEntryProgramSignPositive": first_sign is None or first_sign > 0,
        "firstEntryProgramNotInfeasible": first_program.get("mode") != "infeasible",
        "noPlannerStallBankSafetyViolation": control_quality["plannerStallBankSafetyViolations"] == 0,
        "touchdownDetected": bool(outcome.get("touchdownDetected")),
    }
    return result


def fmt(value: Any, digits: int = 2) -> str:
    number = finite(value)
    return "n/a" if number is None else f"{number:.{digits}f}"


def human_report(result: dict[str, Any]) -> str:
    handoff = result["handoff"]
    runtime = result["runtime"]
    outcome = result["outcome"]
    taem_loop = runtime["taem"]["loopWallDeltaMilliseconds"]
    lines = [
        f"session: {result.get('sessionId') or 'unknown'} pair={'OK' if result['sessionPairValid'] else 'BAD'}",
        f"samples: vehicle={result['vehicleSamples']} planner={result['plannerSamples']}",
    ]
    latest = result["latestState"]
    if latest.get("available"):
        lalt = finite(latest.get("altitudeMeters")); lrange = finite(latest.get("rangeToSiteMeters")); lcross = finite(latest.get("runwayCrossTrackMeters"))
        lines.append(
            f"latest: {latest.get('phase')} UT {fmt(latest.get('ut'),3)} alt {fmt(None if lalt is None else lalt/1000)} km "
            f"TAS {fmt(latest.get('trueAirSpeedMps'))} m/s range {fmt(None if lrange is None else lrange/1000)} km "
            f"cross {fmt(None if lcross is None else lcross/1000)} km"
        )
    if handoff.get("found"):
        altitude = finite(handoff.get("altitudeMeters"))
        speed = finite(handoff.get("trueAirSpeedMps"))
        distance = finite(handoff.get("rangeToSiteMeters"))
        cross = finite(handoff.get("runwayCrossTrackMeters"))
        lines.append(
            "TAEM handoff: "
            f"UT {fmt(handoff.get('ut'),3)} alt {fmt(None if altitude is None else altitude/1000)} km "
            f"TAS {fmt(speed)} m/s range {fmt(None if distance is None else distance/1000)} km "
            f"cross {fmt(None if cross is None else cross/1000)} km"
        )
    else:
        lines.append("TAEM handoff: not observed")
    lines.append(
        f"TAEM loop ({runtime['taemSource']}): n={taem_loop['count']} p95={fmt(taem_loop['p95'])} ms "
        f"p99={fmt(taem_loop['p99'])} ms max={fmt(taem_loop['max'])} ms >2s={runtime['taemLoopGapsOver2Seconds']}"
    )
    post = runtime["postHandoff"]["loopWallDeltaMilliseconds"]
    lines.append(
        f"post-handoff loop ({runtime['postHandoffSource']}): n={post['count']} p95={fmt(post['p95'])} ms "
        f"p99={fmt(post['p99'])} ms max={fmt(post['max'])} ms >2s={runtime['postHandoffLoopGapsOver2Seconds']}"
    )
    quality = result["controlQuality"]
    lines.append(
        f"control quality: max|beta|={fmt(quality['maximumAbsoluteSideslipDegrees'])} deg "
        f"wrong-beta/yaw={quality['wrongSignedBetaYawFrames']} "
        f"stall-alpha-plan={quality['plannerStallAlphaSafetyViolations']} "
        f"stall-bank-plan={quality['plannerStallBankSafetyViolations']}"
    )
    terminal = result["terminalPlanner"]
    first_program = terminal.get("firstEntryProgram") or {}
    if first_program:
        lines.append(
            f"first MM304 program: sign={fmt(first_program.get('inputSign'),0)} mode={first_program.get('mode')} "
            f"bank={fmt(first_program.get('targetBank'))} AoA={fmt(first_program.get('targetAoA'))} ready={first_program.get('terminalReady')}"
        )
    latest_plan = terminal.get("latestPlan") or {}
    if latest_plan.get("ut") is not None:
        lines.append(
            f"latest plan: {latest_plan.get('phase')} mode={latest_plan.get('traceMode')} ready={latest_plan.get('terminalReady')} "
            f"TAEM dR={fmt(None if latest_plan.get('taemRangeErrorMeters') is None else latest_plan.get('taemRangeErrorMeters')/1000)} km "
            f"TAEM v={fmt(latest_plan.get('taemSpeedMps'))} m/s closest={fmt(None if latest_plan.get('closestDistanceMeters') is None else latest_plan.get('closestDistanceMeters')/1000)} km"
        )
    lines.append(
        "terminal planner: "
        f"candidateUT={fmt(terminal.get('firstTerminalCandidateUT'),3)} "
        f"commitUT={fmt(terminal.get('firstTerminalCommittedUT'),3)} "
        f"HAC progress max={fmt(terminal.get('maximumHACTransitionProgress'),3)}"
    )
    lines.append(
        f"outcome: touchdown={outcome.get('touchdownDetected')} abort={outcome.get('abortDetected')} "
        f"fault={outcome.get('faultDetected')} final={outcome.get('finalPhase')}/{outcome.get('finalVesselSituation')}"
    )
    lines.append("flags: " + (", ".join(result["flags"]) if result["flags"] else "none"))
    return "\n".join(lines)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("log", type=Path, help="split *-vehicle.jsonl or *-planner.jsonl path")
    parser.add_argument("--planner", type=Path, help="explicit planner stream path")
    parser.add_argument("--json", action="store_true", help="emit machine-readable JSON")
    args = parser.parse_args()
    result = analyze(args.log, args.planner)
    print(json.dumps(result, indent=2, sort_keys=True) if args.json else human_report(result))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
