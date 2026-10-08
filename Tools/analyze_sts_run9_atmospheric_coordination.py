#!/usr/bin/env python3
"""Summarize archived original-STS atmospheric roll/yaw response.

The archived run bypassed the shadow gate. Its telemetry is physical evidence
for diagnosis only and cannot qualify any campaign gate or landing.
"""

from __future__ import annotations

import hashlib
import json
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
INPUT = ROOT / "FlightLogs/2026-10-07T21-34-08Z-STS-vehicle.jsonl"
OUTPUT = ROOT / "Docs/Experiments/STS-Entry-20261009/live-atmospheric-coordination-crosscheck.json"
RUNWAY_PASS_UT = 10515.3518261821


def deep_merge(target: dict, update: dict) -> None:
    for key, value in update.items():
        if isinstance(value, dict) and isinstance(target.get(key), dict):
            deep_merge(target[key], value)
        else:
            target[key] = value


def selected_state(row: tuple[float, dict, dict], label: str) -> dict:
    ut, source_record, state = row
    aero = state["aero"]
    attitude = state["attitude"]
    motion = state["motion"]
    guidance = state["guidance"]
    command = state["command"]
    applied = state["appliedControl"]
    vehicle = state.get("vehicle", {})
    return {
        "label": label,
        "recordSequence": source_record.get("recordSequence"),
        "tickSequence": source_record.get("tickSequence"),
        "ut": ut,
        "secondsFromRunwayLongitudePass": ut - RUNWAY_PASS_UT,
        "meanAltitudeM": state.get("position", {}).get("altitude"),
        "dynamicPressurePa": aero.get("dynamicPressure"),
        "mach": aero.get("mach"),
        "trueAirSpeedMps": motion.get("trueAirSpeed"),
        "angleOfAttackDeg": attitude.get("angleOfAttack"),
        "sideslipDeg": attitude.get("sideslip"),
        "bankDeg": attitude.get("roll"),
        "headingDeg": attitude.get("heading"),
        "bodyRatesDegPerSecond_pitchRollYaw": [
            attitude.get("bodyPitchRate"),
            attitude.get("bodyRollRate"),
            attitude.get("bodyYawRate"),
        ],
        "runwayAlongTrackM": guidance.get("runwayAlongTrack"),
        "runwayCrossTrackM": guidance.get("runwayCrossTrack"),
        "liftForceN": aero.get("liftForce"),
        "dragForceN": aero.get("dragForce"),
        "targetRollDeg": command.get("targetRoll"),
        "reportedControlXYZ_pitchRollYaw": [
            applied.get("reportedPitch"),
            applied.get("reportedRoll"),
            applied.get("reportedYaw"),
        ],
        "targetThrottle": command.get("targetThrottle"),
        "speedbrakeCommanded": command.get("airbrakes"),
        "speedbrakeState": state.get("state", {}).get("airbrakes"),
        "currentEngineThrustN": vehicle.get("currentThrust"),
        "reportedControlApplied": applied.get("applySucceeded"),
    }


def main() -> int:
    source_hash = hashlib.sha256(INPUT.read_bytes()).hexdigest()
    header = None
    state: dict = {}
    rows: list[tuple[float, dict, dict]] = []
    gate_bypass_events = []
    for line in INPUT.read_text().splitlines():
        record = json.loads(line)
        if record.get("recordType") == "sessionStart":
            header = record
        event = record.get("event", "")
        if "shadow-gate-bypassed" in event:
            gate_bypass_events.append({
                "recordSequence": record.get("recordSequence"),
                "tickSequence": record.get("tickSequence"),
                "ut": record.get("ut"),
                "event": event,
            })
        fields = record.get("fields")
        if not fields:
            continue
        if record.get("recordType") == "vehicleKeyframe":
            state = json.loads(json.dumps(fields))
        else:
            deep_merge(state, fields)
        if all(key in state for key in ("aero", "attitude", "motion", "guidance", "command", "appliedControl")):
            rows.append((float(record.get("ut", 0.0)), record, json.loads(json.dumps(state))))

    if not header or header.get("vessel") != "STS" or not rows:
        raise RuntimeError("The archived telemetry is missing its original-STS header or complete samples")
    if not gate_bypass_events:
        raise RuntimeError("The archived run lacks the known shadow-gate bypass marker")

    atmospheric = [row for row in rows if float(row[2]["aero"].get("dynamicPressure", 0.0)) >= 200.0]
    if not atmospheric:
        raise RuntimeError("No atmospheric samples were found")
    powered = [row for row in atmospheric if abs(float(row[2].get("vehicle", {}).get("currentThrust", 0.0) or 0.0)) > 1.0]
    braking = [row for row in atmospheric if bool(row[2].get("state", {}).get("airbrakes")) or bool(row[2]["command"].get("airbrakes"))]

    runway_window = [
        row for row in atmospheric
        if abs(float(row[2]["guidance"].get("runwayAlongTrack", 1e99))) <= 10000.0
        and float(row[2]["aero"].get("dynamicPressure", 0.0)) >= 1000.0
    ]
    if not runway_window:
        raise RuntimeError("No high-q runway-crossing samples were found")
    crossing = min(rows, key=lambda row: abs(row[0] - RUNWAY_PASS_UT))

    selected = []
    for target, label in (
        (10513.871826182, "two_point_five_km_before_runway_longitude"),
        (10515.351826182, "runway_longitude_crossing"),
        (10515.991826182, "near_site_yaw_roll_transition"),
        (10516.711826182, "cross_track_turn_building"),
        (10517.431826182, "two_seconds_after_runway_longitude"),
    ):
        row = min(rows, key=lambda item: abs(item[0] - target))
        selected.append(selected_state(row, label))

    acceleration_windows = []
    for center in (10515.1, 10515.7, 10516.3, 10516.9):
        before = min(rows, key=lambda row: abs(row[0] - (center - 0.3)))
        after = min(rows, key=lambda row: abs(row[0] - (center + 0.3)))
        dt = after[0] - before[0]
        if dt <= 0.0:
            continue
        a0, a1 = before[2]["attitude"], after[2]["attitude"]
        c0, c1 = before[2]["appliedControl"], after[2]["appliedControl"]
        acceleration_windows.append({
            "startUT": before[0],
            "endUT": after[0],
            "durationS": dt,
            "rollAngularAccelerationDegPerSecondSquared": (
                float(a1["bodyRollRate"]) - float(a0["bodyRollRate"])
            ) / dt,
            "yawAngularAccelerationDegPerSecondSquared": (
                float(a1["bodyYawRate"]) - float(a0["bodyYawRate"])
            ) / dt,
            "meanReportedRollCommand": (float(c0["reportedRoll"]) + float(c1["reportedRoll"])) / 2.0,
            "meanReportedYawCommand": (float(c0["reportedYaw"]) + float(c1["reportedYaw"])) / 2.0,
            "note": "Finite difference over unqualified live telemetry. Roll/yaw commands covary; this is observational and not a causal actuator derivative.",
        })

    q_values = [float(row[2]["aero"].get("dynamicPressure", 0.0)) for row in runway_window]
    beta_values = [float(row[2]["attitude"].get("sideslip", 0.0)) for row in runway_window]
    report = {
        "status": "complete",
        "classification": "UNQUALIFIED_LIVE_TELEMETRY_ANALYSIS",
        "qualification": {
            "shadowGatePassed": False,
            "shadowGateBypassed": True,
            "measuredFlightData": True,
            "qualifiedAerodynamicEvidence": False,
            "landingOrRollout": False,
            "note": "This original-STS atmospheric telemetry is from an archived run with an explicit KSP_LANDER_BYPASS_SHADOW_GATE event. It is useful for physical diagnosis only and cannot qualify entry, TAEM, touchdown, or rollout.",
        },
        "provenance": {
            "sourceFile": str(INPUT.relative_to(ROOT)),
            "sourceSHA256": source_hash,
            "analysisScript": str(Path(__file__).resolve().relative_to(ROOT)),
            "analysisScriptSHA256": hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
            "sessionId": header.get("sessionId"),
            "startedUTC": header.get("startedUTC"),
            "vessel": header.get("vessel"),
            "gateBypassEvents": gate_bypass_events,
            "bodyRateUnits": "degrees/second; CLanding/transport/client/telemetry.inc converts KSP body angular velocity to deg/s with the recorded sign convention.",
            "reportedControlUnits": "normalized direct-control axis values, not measured surface deflections.",
            "noPropulsionAndBrakeAudit": {
                "samplesAtQAtLeast200Pa": len(atmospheric),
                "samplesWithEngineThrustOver1N": len(powered),
                "samplesWithSpeedbrakeCommandOrState": len(braking),
                "basis": "vehicle.currentThrust, state.airbrakes, and command.airbrakes in reconstructed telemetry deltas; this audit does not claim RCS state because the log does not persist it per sample.",
            },
        },
        "runwayCrossingWindow": {
            "referenceUT": RUNWAY_PASS_UT,
            "selectedCrossingSample": selected_state(crossing, "closest_logged_sample_to_first_runway_longitude_passage"),
            "sampleCountWithinPlusMinus10kmAlongTrackAndQAtLeast1kPa": len(runway_window),
            "dynamicPressureRangePa": [min(q_values), max(q_values)],
            "sideslipRangeDeg": [min(beta_values), max(beta_values)],
            "samples": selected,
            "rollRateFiniteDifferenceWindows": acceleration_windows,
        },
        "interpretation": "Near the runway-longitude passage, measured sideslip changes from about +0.4 to below -1 degree while commanded yaw grows and measured roll rate reverses from negative to positive. Roll command also grows, so these live data do not isolate yaw-to-roll causality. The trace does show that the real STS can develop substantial body-rate and sideslip dynamics near the site that the point-mass shadow does not represent. Keep the normal gate and bank ramp unchanged; obtain gate-qualified in-flight coordination evidence before using this trace to retune guidance.",
    }
    OUTPUT.parent.mkdir(parents=True, exist_ok=True)
    OUTPUT.write_text(json.dumps(report, indent=2, allow_nan=False) + "\n")
    print(json.dumps({
        "status": report["status"],
        "output": str(OUTPUT),
        "shadowGateBypassed": report["qualification"]["shadowGateBypassed"],
        "runwayWindowSamples": len(runway_window),
        "atmosphericEngineThrustViolations": len(powered),
        "speedbrakeViolations": len(braking),
    }, separators=(",", ":")))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
