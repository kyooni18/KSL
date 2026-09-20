#!/usr/bin/env python3
"""Attach the AR HUD to a headless test that was already running before HUD startup.

This is an observer only. It reads KSP flight state from a separate kRPC client and
uses the current planner log for the active target/path. It never writes controls.
"""

from __future__ import annotations

import argparse
import json
import math
import os
from pathlib import Path
import time
from typing import Any

import krpc  # type: ignore

from PyQtApp.ingame_hud import InGameARHUD


def _number(value: Any, default: float = 0.0) -> float:
    try:
        result = float(value)
    except (TypeError, ValueError):
        return default
    return result if math.isfinite(result) else default


def _live_exact_owner(path: Path) -> bool:
    try:
        pid = int(path.read_text(encoding="utf-8").strip())
    except (OSError, ValueError):
        return False
    try:
        os.kill(pid, 0)
        return True
    except OSError:
        try:
            path.unlink()
        except OSError:
            pass
        return False


def _newest_planner_log(root: Path) -> Path | None:
    logs = list((root / "FlightLogs").glob("*-planner.jsonl"))
    return max(logs, key=lambda path: path.stat().st_mtime, default=None)


def _candidate_path(record: dict[str, Any]) -> list[dict[str, Any]]:
    current_value = record.get("currentPlan")
    if not isinstance(current_value, dict) or not current_value or current_value.get("valid") is False:
        # A planner trace can outlive executable-plan lineage during safe release.
        # Keep that diagnostic candidate off the HUD instead of labelling it PLAN.
        return []
    current = current_value
    trace = record.get("plannerTrace")
    if not isinstance(trace, dict):
        return []
    candidates = trace.get("candidates")
    if not isinstance(candidates, list):
        return []
    usable = [item for item in candidates if isinstance(item, dict) and isinstance(item.get("candidatePath"), list)]
    if not usable:
        return []
    selected = next((item for item in usable if item.get("selected")), None)
    if selected is None:
        target_bank = _number(current.get("targetBank"))
        target_aoa = _number(current.get("targetAoA"))
        target_heading = _number(current.get("targetHeading"))

        def score(item: dict[str, Any]) -> float:
            command = item.get("command") if isinstance(item.get("command"), dict) else item
            return (
                abs(_number(command.get("bank", command.get("targetBank")), target_bank) - target_bank)
                + 0.5 * abs(_number(command.get("aoa", command.get("targetAoA")), target_aoa) - target_aoa)
                + 0.05 * abs(_number(command.get("heading", command.get("targetHeading")), target_heading) - target_heading)
            )

        selected = min(usable, key=score)
    return [item for item in selected.get("candidatePath", []) if isinstance(item, dict)]


def _load_latest_plan(path: Path) -> tuple[dict[str, Any], list[dict[str, Any]], list[dict[str, Any]]]:
    latest: dict[str, Any] = {}
    latest_prediction: list[dict[str, Any]] = []
    latest_plan: list[dict[str, Any]] = []
    try:
        with path.open("r", encoding="utf-8") as handle:
            for line in handle:
                try:
                    record = json.loads(line)
                except json.JSONDecodeError:
                    continue
                if not isinstance(record, dict):
                    continue
                latest = record
                if record.get("trajectoryIncluded"):
                    published = record.get("publishedPrediction")
                    if isinstance(published, list):
                        # This is the backend's lineage-checked, continuity-preserving forecast.
                        # Preserve an explicitly empty publication too: it represents a deliberate
                        # predictor discontinuity and must clear, rather than resurrect, old geometry.
                        latest_prediction = [item for item in published if isinstance(item, dict)]
                    # Keep PLAN lineage snapshot semantics identical to the web observer:
                    # an explicit trajectory record with no usable candidate clears the old
                    # planner geometry instead of resurrecting it after the current plan ends.
                    latest_plan = _candidate_path(record)
    except OSError:
        pass
    return latest, latest_prediction, latest_plan


def main() -> int:
    parser = argparse.ArgumentParser(description="Attach the in-game AR HUD to an already-running live test.")
    parser.add_argument(
        "--stale-timeout",
        type=float,
        default=0.0,
        help="Exit after planner inactivity; 0 keeps the observer attached indefinitely.",
    )
    args = parser.parse_args()

    root = Path(__file__).resolve().parents[1]
    config_path = root / "Configuration" / "live-cnano.json"
    with config_path.open("r", encoding="utf-8") as handle:
        configuration = json.load(handle)

    connection = krpc.connect(name="KSP Shuttle Lander HUD Observer")
    hud = InGameARHUD()
    actual: list[dict[str, float]] = []
    planner_path: Path | None = None
    planner_mtime = -1.0
    planner_record: dict[str, Any] = {}
    prediction: list[dict[str, Any]] = []
    planned: list[dict[str, Any]] = []
    last_planner_change = time.monotonic()
    exact_owner = root / "Runtime" / "Headless" / "ingame_hud_exact.pid"
    exact_suppressed = False
    last_actual_sample = 0.0

    try:
        while True:
            if _live_exact_owner(exact_owner):
                if not exact_suppressed:
                    hud.suspend()
                    exact_suppressed = True
                time.sleep(0.20)
                continue
            exact_suppressed = False
            current_path = _newest_planner_log(root)
            if current_path is not None:
                current_mtime = current_path.stat().st_mtime
                if current_path != planner_path or current_mtime > planner_mtime:
                    planner_path = current_path
                    planner_mtime = current_mtime
                    planner_record, prediction, planned = _load_latest_plan(current_path)
                    last_planner_change = time.monotonic()

            if (
                args.stale_timeout > 0.0
                and planner_path is not None
                and time.monotonic() - last_planner_change > args.stale_timeout
            ):
                break

            vessel = connection.space_center.active_vessel
            if vessel is None:
                time.sleep(0.25)
                continue
            flight = vessel.flight()
            ut = _number(connection.space_center.ut, float("nan"))
            latitude = _number(flight.latitude)
            longitude = _number(flight.longitude)
            altitude = _number(flight.mean_altitude)
            radar_altitude = max(0.0, altitude - _number(flight.surface_altitude))
            horizontal_speed = max(0.0, _number(flight.horizontal_speed))
            vertical_speed = _number(flight.vertical_speed)
            fpa = math.degrees(math.atan2(vertical_speed, max(1e-6, horizontal_speed)))
            pitch = _number(flight.pitch)
            roll = _number(flight.roll)
            heading = _number(flight.heading)
            aoa = math.degrees(_number(flight.angle_of_attack))

            now = time.monotonic()
            if now - last_actual_sample >= 0.4:
                actual.append({"latitude": latitude, "longitude": longitude, "altitude": altitude})
                actual = actual[-64:]
                last_actual_sample = now

            current = planner_record.get("currentPlan") if isinstance(planner_record.get("currentPlan"), dict) else {}
            terminal = planner_record.get("terminalGeometry") if isinstance(planner_record.get("terminalGeometry"), dict) else {}
            target_aoa = _number(current.get("targetAoA"), aoa)
            target_pitch = fpa + target_aoa
            phase = str(planner_record.get("phase") or "LIVE TEST")
            reversal_ut = _number(current.get("plannedReversalUT"), float("nan"))
            guidance_state = {
                "entryPlanValid": bool(current.get("valid")),
                "entryPlanTerminalReady": bool(current.get("terminalReady")),
                "entryPlanTargetBank": current.get("targetBank"),
                "entryPlanTargetAoA": current.get("targetAoA"),
                "entryPlanTargetHeading": current.get("targetHeading"),
                "entryPlanSegmentRemaining": max(0.0, _number(current.get("plannedUT")) + _number(current.get("segmentDuration")) - ut),
                "entryPlanCost": current.get("plannerCost"),
                "entryPlanTAEMRangeError": current.get("taemRangeError"),
                "entryPlanTAEMSpeed": current.get("taemSpeed"),
                "entryPlanTAEMEnergyError": current.get("taemEnergyError"),
                "entryPlanPredictedReversals": current.get("predictedReversals", 0),
                "entryReversalScheduled": bool(current.get("hasPlannedReversal")),
                "entryReversalIsFinal": bool(current.get("plannedReversalFinal")),
                "entryReversalTimeRemaining": max(0.0, reversal_ut - ut) if math.isfinite(reversal_ut) else None,
                "entryReversalRange": current.get("plannedReversalRange"),
                "entryReversalSign": current.get("plannedReversalSign"),
                "terminalPredictionValid": bool(terminal.get("terminalPredictionValid")),
                "terminalCandidateValid": bool(terminal.get("terminalCandidateValid")),
                "terminalPathCommitted": bool(terminal.get("terminalCommitted")),
                "terminalCandidateRadius": terminal.get("candidateRadius"),
                "terminalCandidateAltitude": terminal.get("candidateAltitude"),
                "terminalCandidateSpeed": terminal.get("candidateSpeed"),
                "terminalReferenceFPA": terminal.get("referenceFPA"),
                "terminalBlend": terminal.get("mix"),
                "terminalPathRemaining": terminal.get("hacRemaining"),
                "hacRadius": terminal.get("hacRadius"),
                "hacTransitionProgress": terminal.get("hacTransitionProgress"),
                "commandedCourseRateEstimate": terminal.get("commandedCourseRate"),
            }
            snapshot = {
                "connectionStatus": "connected",
                "phase": phase,
                "automationEngaged": True,
                "telemetry": {
                    "ut": ut,
                    "latitude": latitude,
                    "longitude": longitude,
                    "meanAltitude": altitude,
                    "radarAltitude": radar_altitude,
                    "verticalSpeed": vertical_speed,
                    "horizontalSpeed": horizontal_speed,
                    "trueAirSpeed": _number(flight.true_air_speed),
                    "pitch": pitch,
                    "roll": roll,
                    "heading": heading,
                    "groundTrackHeading": heading,
                    "flightPathAngle": fpa,
                    "angleOfAttack": aoa,
                    "predictedTAEMRangeError": current.get("taemRangeError"),
                    "predictedTAEMEnergyError": current.get("taemEnergyError"),
                },
                "command": {
                    "autopilotEngaged": True,
                    "targetPitch": target_pitch,
                    "targetAoA": target_aoa,
                    "targetRoll": _number(current.get("targetBank"), roll),
                    "targetHeading": _number(current.get("targetHeading"), heading),
                },
                "guidanceState": guidance_state,
                "predictedTrajectory": prediction,
                # The log observer cannot reconstruct the backend's terminal reference trajectory.
                # Keep selected planner intent distinct so the renderer can label it PLAN rather
                # than misrepresenting it as either a propagated prediction or a guidance reference.
                "referenceTrajectory": [],
                "plannedTrajectory": planned,
                "actualTrajectory": actual,
            }
            hud.update(snapshot, configuration)
            time.sleep(0.12)
    finally:
        hud.close()
        connection.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
