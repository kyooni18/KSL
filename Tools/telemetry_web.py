#!/usr/bin/env python3
"""Read-only KSP Shuttle Lander telemetry web server.

The HTTP/SSE server stays alive when KSP disappears. A collector thread owns a
separate display-only kRPC connection and reconnects forever with bounded
backoff. Guidance intent is read from the active planner JSONL log; the flight
backend remains the sole owner of controls.
"""

from __future__ import annotations

import argparse
import copy
import json
import math
import mimetypes
import os
import socket
from pathlib import Path
import sys
import threading
import time
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from typing import Any
from urllib.parse import parse_qs, urlparse

try:
    import krpc  # type: ignore
except ImportError:  # Simulator/replay modes intentionally do not require kRPC.
    krpc = None  # type: ignore

ROOT = Path(__file__).resolve().parents[1]
WEB_ROOT = ROOT / "WebTelemetry"
DEFAULT_CONFIG = ROOT / "Configuration" / "live-cnano.json"
STSN_MODEL_ROOT = ROOT / "Runtime" / "WebTelemetry"
RL_RESULTS_ROOT = ROOT / "ShuttleSim" / "rl" / "results"
RL_SLOWRUNS_ROOT = ROOT / "Runtime" / "SlowRuns"
RL_STATUS_POLL_SECONDS = 1.0



def number(value: Any, default: float = 0.0) -> float:
    try:
        result = float(value)
    except (TypeError, ValueError):
        return default
    return result if math.isfinite(result) else default


def optional_number(value: Any) -> float | None:
    try:
        result = float(value)
    except (TypeError, ValueError):
        return None
    return result if math.isfinite(result) else None


def signed_degrees(value: float) -> float:
    wrapped = value % 360.0
    return wrapped - 360.0 if wrapped > 180.0 else wrapped


def bearing_degrees(a: dict[str, float], b: dict[str, float]) -> float:
    lat1 = math.radians(a["latitude"])
    lat2 = math.radians(b["latitude"])
    dlon = math.radians(b["longitude"] - a["longitude"])
    y = math.sin(dlon) * math.cos(lat2)
    x = math.cos(lat1) * math.sin(lat2) - math.sin(lat1) * math.cos(lat2) * math.cos(dlon)
    return math.degrees(math.atan2(y, x)) % 360.0


KERBIN_RADIUS = 600000.0


def great_circle_distance(a: dict[str, Any], b: dict[str, Any], radius: float = KERBIN_RADIUS) -> float:
    lat1 = math.radians(number(a.get("latitude")))
    lat2 = math.radians(number(b.get("latitude")))
    dlat = lat2 - lat1
    dlon = math.radians(signed_degrees(number(b.get("longitude")) - number(a.get("longitude"))))
    h = math.sin(dlat * 0.5) ** 2 + math.cos(lat1) * math.cos(lat2) * math.sin(dlon * 0.5) ** 2
    return radius * 2.0 * math.atan2(math.sqrt(max(0.0, h)), math.sqrt(max(0.0, 1.0 - h)))


ACTUAL_TRAJECTORY_MAX_POINTS = 2400
ACTUAL_TRAJECTORY_MAX_UT_AGE = 900.0
ACTUAL_TRAJECTORY_RESUME_MAX_UT_GAP = 90.0
ORBIT_TRAJECTORY_SAMPLE_COUNT = 73
ORBIT_TRAJECTORY_REFRESH_SECONDS = 2.0
ORBIT_TRAJECTORY_THRUST_EPSILON = 1.0


def actual_trajectory_history_path(root: Path) -> Path:
    return root / "Runtime" / "WebTelemetry" / "actual-trajectory.json"


def normalize_actual_trajectory_point(value: Any) -> dict[str, float] | None:
    if not isinstance(value, dict):
        return None
    latitude = optional_number(value.get("latitude"))
    longitude = optional_number(value.get("longitude"))
    altitude = optional_number(value.get("altitude"))
    ut = optional_number(value.get("ut"))
    if latitude is None or longitude is None or altitude is None or ut is None:
        return None
    return {"latitude": latitude, "longitude": longitude, "altitude": altitude, "ut": ut}


def prune_actual_trajectory(points: list[dict[str, Any]], current_ut: float) -> list[dict[str, float]]:
    cutoff = current_ut - ACTUAL_TRAJECTORY_MAX_UT_AGE
    cleaned: list[dict[str, float]] = []
    for value in points:
        point = normalize_actual_trajectory_point(value)
        if point is None or point["ut"] < cutoff or point["ut"] > current_ut + 1.0:
            continue
        cleaned.append(point)
    return cleaned[-ACTUAL_TRAJECTORY_MAX_POINTS:]


def load_actual_trajectory_history(root: Path) -> list[dict[str, float]]:
    path = actual_trajectory_history_path(root)
    try:
        value = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError):
        return []
    if not isinstance(value, list):
        return []
    points = [point for item in value if (point := normalize_actual_trajectory_point(item)) is not None]
    return points[-ACTUAL_TRAJECTORY_MAX_POINTS:]


def save_actual_trajectory_history(root: Path, points: list[dict[str, Any]]) -> None:
    path = actual_trajectory_history_path(root)
    try:
        path.parent.mkdir(parents=True, exist_ok=True)
        temporary = path.with_suffix(".tmp")
        temporary.write_text(json.dumps(points, separators=(",", ":"), allow_nan=False), encoding="utf-8")
        os.replace(temporary, path)
    except (OSError, TypeError, ValueError):
        pass


def actual_trajectory_can_resume(points: list[dict[str, Any]], current: dict[str, Any]) -> bool:
    if not points:
        return True
    previous = normalize_actual_trajectory_point(points[-1])
    current_point = normalize_actual_trajectory_point(current)
    if previous is None or current_point is None:
        return False
    dt = current_point["ut"] - previous["ut"]
    if dt < -0.5 or dt > ACTUAL_TRAJECTORY_RESUME_MAX_UT_GAP:
        return False
    # Permit normal hypersonic motion across a short observer restart, but reject
    # quickloads/teleports so an old trail is never joined to a new vessel state.
    maximum_distance = max(30000.0, 5000.0 + 3000.0 * max(0.0, dt))
    return great_circle_distance(previous, current_point) <= maximum_distance


def merge_vehicle_fields(base: dict[str, Any], patch: dict[str, Any]) -> None:
    for key, value in patch.items():
        if isinstance(value, dict) and isinstance(base.get(key), dict):
            merge_vehicle_fields(base[key], value)
        elif isinstance(value, dict):
            base[key] = copy.deepcopy(value)
        else:
            base[key] = value


def load_actual_trajectory_from_vehicle_log(root: Path, current: dict[str, Any]) -> list[dict[str, float]]:
    current_point = normalize_actual_trajectory_point(current)
    if current_point is None:
        return []
    candidates = sorted((root / "FlightLogs").glob("*-vehicle.jsonl"), key=lambda path: path.stat().st_mtime, reverse=True)
    for path in candidates[:8]:
        fields: dict[str, Any] = {}
        points: list[dict[str, float]] = []
        try:
            with path.open("r", encoding="utf-8") as handle:
                for raw in handle:
                    try:
                        record = json.loads(raw)
                    except json.JSONDecodeError:
                        continue
                    kind = record.get("recordType")
                    patch = record.get("fields")
                    if not isinstance(patch, dict):
                        continue
                    if kind == "vehicleKeyframe":
                        fields = copy.deepcopy(patch)
                    elif kind == "vehicleDelta":
                        merge_vehicle_fields(fields, patch)
                    else:
                        continue
                    ut = optional_number(record.get("ut"))
                    position = fields.get("position") if isinstance(fields.get("position"), dict) else {}
                    point = normalize_actual_trajectory_point({
                        "ut": ut,
                        "latitude": position.get("latitude"),
                        "longitude": position.get("longitude"),
                        "altitude": position.get("altitude"),
                    })
                    if point is None or point["ut"] > current_point["ut"] + 1.0:
                        continue
                    if point["ut"] >= current_point["ut"] - ACTUAL_TRAJECTORY_MAX_UT_AGE:
                        points.append(point)
        except OSError:
            continue
        points = prune_actual_trajectory(points, current_point["ut"])
        if points and actual_trajectory_can_resume(points, current_point):
            return points
    return []


def local_offsets(origin: dict[str, Any], point: dict[str, Any], radius: float = KERBIN_RADIUS) -> tuple[float, float]:
    distance = great_circle_distance(origin, point, radius)
    bearing = math.radians(bearing_degrees(origin, point))
    return distance * math.sin(bearing), distance * math.cos(bearing)


def local_point(origin: dict[str, Any], east: float, north: float, altitude: float, radius: float = KERBIN_RADIUS) -> dict[str, float]:
    distance = math.hypot(east, north)
    if distance <= 1e-9:
        return {"latitude": number(origin.get("latitude")), "longitude": number(origin.get("longitude")), "altitude": altitude}
    bearing = math.atan2(east, north)
    angular = distance / radius
    lat1 = math.radians(number(origin.get("latitude")))
    lon1 = math.radians(number(origin.get("longitude")))
    lat2 = math.asin(math.sin(lat1) * math.cos(angular) + math.cos(lat1) * math.sin(angular) * math.cos(bearing))
    lon2 = lon1 + math.atan2(
        math.sin(bearing) * math.sin(angular) * math.cos(lat1),
        math.cos(angular) - math.sin(lat1) * math.sin(lat2),
    )
    return {"latitude": math.degrees(lat2), "longitude": signed_degrees(math.degrees(lon2)), "altitude": altitude}


def projected_taem_trajectory(
    prediction: list[dict[str, Any]], terminal: dict[str, Any], configuration: dict[str, Any],
    guidance_state_payload: dict[str, Any] | None = None,
) -> list[dict[str, Any]]:
    """Build a display-only TAEM/HAC continuation from a forecasted TAEM handoff.

    This does not feed guidance. It uses the propagated TAEM endpoint plus the
    terminal candidate geometry when available, otherwise the configured nominal
    HAC, and keeps that provenance separate from the physical predictor stream.
    """
    usable = [
        point for point in prediction
        if isinstance(point, dict) and optional_number(point.get("latitude")) is not None
        and optional_number(point.get("longitude")) is not None
    ]
    if not usable:
        return []
    endpoint = usable[-1]
    if "TAEM" not in str(endpoint.get("phase") or "").upper():
        return []

    site = configuration.get("site") if isinstance(configuration.get("site"), dict) else {}
    guidance_cfg = configuration.get("guidance") if isinstance(configuration.get("guidance"), dict) else {}
    if optional_number(site.get("latitude")) is None or optional_number(site.get("longitude")) is None:
        return []

    runway_heading = math.radians(number(site.get("runwayHeading"), 90.0))
    candidate_valid = bool(terminal.get("terminalCandidateValid"))
    final_distance = number(terminal.get("candidateFinalDistance"), 0.0) if candidate_valid else 0.0
    if final_distance < 500.0:
        final_distance = max(500.0, number(guidance_cfg.get("finalApproachDistance"), 8000.0))
    hac_radius = number(terminal.get("candidateRadius"), 0.0) if candidate_valid else 0.0
    if hac_radius < 1000.0:
        hac_radius = max(1000.0, number(guidance_cfg.get("hacRadius"), 12000.0))
    taem_slope = math.radians(max(0.1, number(guidance_cfg.get("taemGlideSlope"), 12.0)))
    final_slope = math.radians(max(0.1, number(guidance_cfg.get("finalGlideSlope"), 20.0)))
    site_altitude = number(site.get("altitude"), 0.0)
    endpoint_altitude = max(site_altitude, number(endpoint.get("altitude"), site_altitude))
    endpoint_speed = max(0.0, number(endpoint.get("speed"), 0.0))
    endpoint_ut = number(endpoint.get("ut"), 0.0)

    east, north = local_offsets(site, endpoint)
    axis_e, axis_n = math.sin(runway_heading), math.cos(runway_heading)
    right_e, right_n = math.cos(runway_heading), -math.sin(runway_heading)
    cross = east * right_e + north * right_n
    candidate_side = optional_number(terminal.get("candidateSide")) if candidate_valid else None
    side = (1.0 if candidate_side >= 0.0 else -1.0) if candidate_side is not None and abs(candidate_side) >= 0.1 else (1.0 if cross >= 0.0 else -1.0)

    final_e, final_n = -axis_e * final_distance, -axis_n * final_distance
    candidate_kind = int(round(number(terminal.get("candidateKind"), 2.0))) if candidate_valid else 2
    entry_side = optional_number(guidance_state_payload.get("sTurnSign")) if isinstance(guidance_state_payload, dict) else None
    if entry_side is not None and abs(entry_side) >= 0.1:
        # MM304's fixed Runway-09 outlet is a pose contract: ground track must
        # be perpendicular to the runway at the alignment station. Do not infer
        # this tangent from the preceding Entry segment, which is normally still
        # runway-parallel and made the UI projection appear rotated by ~90 deg.
        start_course = (math.degrees(runway_heading) - (1.0 if entry_side >= 0.0 else -1.0) * 90.0) % 360.0
    else:
        start_course = bearing_degrees(usable[-2], endpoint) if len(usable) >= 2 else bearing_degrees(endpoint, site)
    start_tangent = (math.sin(math.radians(start_course)), math.cos(math.radians(start_course)))
    projected: list[dict[str, Any]] = []

    def append_local(e: float, n: float, altitude: float, fraction: float, phase: str = "TAEM PROJ") -> None:
        point = local_point(site, e, n, max(site_altitude, altitude))
        point.update({
            "ut": endpoint_ut + max(0.0, fraction) * 240.0,
            "speed": endpoint_speed,
            "phase": phase,
            "kind": "projected",
        })
        projected.append(point)

    append_local(east, north, endpoint_altitude, 0.0)
    final_fix_altitude = site_altitude + final_distance * math.tan(final_slope)

    if candidate_kind == 1:
        # Terminal spline candidate: use the predicted course at TAEM and the
        # runway tangent to form a smooth geometric continuation to the final fix.
        span = math.hypot(final_e - east, final_n - north)
        c1_e = east + start_tangent[0] * span * 0.35
        c1_n = north + start_tangent[1] * span * 0.35
        c2_e = final_e - axis_e * span * 0.28
        c2_n = final_n - axis_n * span * 0.28
        for index in range(1, 19):
            t = index / 18.0
            q = 1.0 - t
            e = q**3 * east + 3.0 * q*q*t * c1_e + 3.0 * q*t*t * c2_e + t**3 * final_e
            n = q**3 * north + 3.0 * q*q*t * c1_n + 3.0 * q*t*t * c2_n + t**3 * final_n
            altitude = endpoint_altitude + (final_fix_altitude - endpoint_altitude) * (t*t * (3.0 - 2.0*t))
            append_local(e, n, altitude, t * 0.65)
    else:
        center_e = final_e + right_e * side * hac_radius
        center_n = final_n + right_n * side * hac_radius
        final_angle = math.atan2(final_n - center_n, final_e - center_e)
        orientation = -side
        arc_remaining = optional_number(terminal.get("candidateArcRemaining")) if candidate_valid else None
        if arc_remaining is not None and arc_remaining > hac_radius * 0.05:
            arc_angle = max(0.05, min(math.tau, arc_remaining / hac_radius))
            join_angle = final_angle - orientation * arc_angle
        else:
            join_angle = math.atan2(north - center_n, east - center_e)
            arc_angle = (orientation * (final_angle - join_angle)) % math.tau
        join_e = center_e + math.cos(join_angle) * hac_radius
        join_n = center_n + math.sin(join_angle) * hac_radius
        join_arc = hac_radius * arc_angle
        join_altitude = min(endpoint_altitude, max(final_fix_altitude, final_fix_altitude + join_arc * math.tan(taem_slope)))

        acquire = math.hypot(join_e - east, join_n - north)
        end_tangent = (orientation * -math.sin(join_angle), orientation * math.cos(join_angle))
        c1_e = east + start_tangent[0] * acquire * 0.34
        c1_n = north + start_tangent[1] * acquire * 0.34
        c2_e = join_e - end_tangent[0] * acquire * 0.26
        c2_n = join_n - end_tangent[1] * acquire * 0.26
        for index in range(1, 13):
            t = index / 12.0
            q = 1.0 - t
            e = q**3 * east + 3.0 * q*q*t * c1_e + 3.0 * q*t*t * c2_e + t**3 * join_e
            n = q**3 * north + 3.0 * q*q*t * c1_n + 3.0 * q*t*t * c2_n + t**3 * join_n
            altitude = endpoint_altitude + (join_altitude - endpoint_altitude) * (t*t * (3.0 - 2.0*t))
            append_local(e, n, altitude, t * 0.22)

        arc_steps = max(10, min(72, int(math.ceil(math.degrees(arc_angle) / 4.0))))
        for index in range(1, arc_steps + 1):
            f = index / arc_steps
            angle = join_angle + orientation * arc_angle * f
            e = center_e + math.cos(angle) * hac_radius
            n = center_n + math.sin(angle) * hac_radius
            remaining = hac_radius * arc_angle * (1.0 - f)
            altitude = final_fix_altitude + remaining * math.tan(taem_slope)
            append_local(e, n, min(join_altitude, altitude), 0.22 + f * 0.58)

    for index in range(1, 21):
        f = index / 20.0
        distance = final_distance * (1.0 - f)
        e = -axis_e * distance
        n = -axis_n * distance
        altitude = site_altitude + distance * math.tan(final_slope)
        append_local(e, n, altitude, 0.80 + f * 0.20, "FINAL PROJ")
    return projected



def _trajectory_points(value: Any) -> list[dict[str, Any]] | None:
    if not isinstance(value, list):
        return None
    return [copy.deepcopy(item) for item in value if isinstance(item, dict)]


TAEM_CAPTURE_VETO_BITS = (
    (1, "speed"),
    (2, "spatial"),
    (4, "course"),
    (8, "altitude"),
    (16, "maneuver-energy"),
    (32, "fpa"),
    (64, "structural"),
    (128, "hac-radius"),
    (256, "minimum-altitude"),
)


def taem_capture_veto_reasons(value: Any) -> list[str]:
    veto = optional_number(value)
    if veto is None:
        return []
    mask = int(veto)
    return [name for bit, name in TAEM_CAPTURE_VETO_BITS if mask & bit] or (["unknown"] if mask else [])


def _first_present(mapping: dict[str, Any], names: tuple[str, ...]) -> Any:
    for name in names:
        if name in mapping:
            return mapping.get(name)
    return None


def _capture_ready_from_mapping(mapping: dict[str, Any], ready_names: tuple[str, ...], veto_names: tuple[str, ...]) -> bool:
    ready = _first_present(mapping, ready_names)
    if ready is not None:
        return ready is True
    veto = _first_present(mapping, veto_names)
    if veto is not None:
        parsed = optional_number(veto)
        return parsed is not None and int(parsed) == 0
    # Backward-compatible for old archives that only had terminalReady.
    return True


def _entry_plan_displayable(value: Any) -> bool:
    """Only expose entry PLAN geometry once its terminal handoff is proven."""
    return (
        isinstance(value, dict)
        and value.get("valid") is True
        and value.get("terminalReady") is True
        and _capture_ready_from_mapping(
            value,
            ("taemCaptureReady", "captureReady", "ready"),
            ("taemCaptureVeto", "captureVeto", "veto"),
        )
    )


def _entry_guidance_plan_ready(guidance: Any) -> bool:
    return (
        isinstance(guidance, dict)
        and guidance.get("entryPlanValid") is True
        and guidance.get("entryPlanTerminalReady") is True
        and _capture_ready_from_mapping(
            guidance,
            ("entryPlanTAEMCaptureReady", "entryPlanCaptureReady"),
            ("entryPlanTAEMCaptureVeto", "entryPlanCaptureVeto"),
        )
    )


def _terminal_reference_displayable(exact: dict[str, Any]) -> bool:
    """A terminal reference is a plan only after guidance has committed/captured it."""
    guidance = exact.get("guidanceState")
    if not isinstance(guidance, dict):
        return False
    return any(
        guidance.get(key) is True
        for key in ("terminalPathCommitted", "terminalPathCaptured", "terminalPathComplete")
    )


def _terminal_phase(value: Any) -> bool:
    phase = str(value or "").upper()
    return any(token in phase for token in ("TAEM", "HAC", "FINAL", "PREFLARE", "FLARE", "TOUCHDOWN", "ROLLOUT"))


def _displayable_entry_prediction(exact: dict[str, Any],
                                  predicted: list[dict[str, Any]]) -> list[dict[str, Any]]:
    """Expose only trajectory geometry backed by the current executable MM304 plan.

    A non-terminal-ready Entry plan proves only its committed segment. Older
    archives could contain a much longer shadow that later fell through degraded
    TAEM recovery; drawing that whole path made an invalid far-away endpoint look
    like the actual TAEM handoff. Keep the near-term segment, and reveal the full
    forecast only once the controller certifies terminal delivery.
    """
    if _terminal_phase(exact.get("phase")):
        return copy.deepcopy(predicted)

    guidance = exact.get("guidanceState")
    if not isinstance(guidance, dict) or guidance.get("entryPlanValid") is not True:
        return []
    if guidance.get("entryPlanTerminalReady") is True:
        return copy.deepcopy(predicted)

    telemetry = exact.get("telemetry")
    current_ut = optional_number(telemetry.get("ut")) if isinstance(telemetry, dict) else None
    remaining = optional_number(guidance.get("entryPlanSegmentRemaining"))
    if current_ut is None or remaining is None or remaining <= 0.0:
        return []

    cutoff_ut = current_ut + remaining + 1e-6
    bounded: list[dict[str, Any]] = []
    for point in predicted:
        point_ut = optional_number(point.get("ut"))
        if point_ut is None or point_ut <= cutoff_ut:
            clean = copy.deepcopy(point)
            # A degraded/recovery phase inside an otherwise unqualified Entry
            # segment is never a certified TAEM handoff on the replay surface.
            if _terminal_phase(clean.get("phase")):
                clean["phase"] = "MM304 Entry"
            bounded.append(clean)
    return bounded


def apply_exact_trajectory_geometry(snapshot: dict[str, Any], exact: dict[str, Any], terminal: dict[str, Any], configuration: dict[str, Any]) -> bool:
    predicted = _trajectory_points(exact.get("predictedTrajectory"))
    reference = _trajectory_points(exact.get("referenceTrajectory"))
    if predicted is None and reference is None:
        return False
    if predicted is not None:
        exact_guidance = exact.get("guidanceState")
        exact_entry_plan_ready = (
            _entry_guidance_plan_ready(exact_guidance)
            and not _terminal_phase(exact.get("phase"))
        )
        displayed_prediction = _displayable_entry_prediction(exact, predicted)
        snapshot["predictedTrajectory"] = displayed_prediction
        if len(displayed_prediction) < 2 and len(predicted) >= 2:
            # Keep rejected/stale forecast geometry available to the archive
            # inspector. Live surfaces continue to use only the qualified path.
            snapshot["replayCandidatePrediction"] = predicted
        snapshot["projectedTAEMTrajectory"] = (
            projected_taem_trajectory(displayed_prediction, terminal, configuration, exact_guidance)
            if exact_entry_plan_ready
            else []
        )
    if reference is not None:
        if _terminal_reference_displayable(exact):
            snapshot["referenceTrajectory"] = reference
            if len(reference) >= 2:
                snapshot["plannedTrajectory"] = copy.deepcopy(reference)
            else:
                snapshot["plannedTrajectory"] = []
        else:
            # The controller can publish candidate/reference geometry before MM305
            # commits it. Keep that forensic geometry out of the operator PLAN/REF
            # surfaces; otherwise an unproven candidate looks like a fixed path.
            if len(reference) >= 2:
                snapshot["replayCandidateTrajectory"] = reference
            snapshot["referenceTrajectory"] = []
            if reference or _terminal_phase(exact.get("phase")):
                snapshot["plannedTrajectory"] = []
    tick_sequence = optional_number(exact.get("tickSequence"))
    if tick_sequence is not None:
        snapshot["trajectoryRevision"] = int(tick_sequence)
    return True


def newest_planner_log(root: Path) -> Path | None:
    logs = list((root / "FlightLogs").glob("*-planner.jsonl"))
    return max(logs, key=lambda path: path.stat().st_mtime, default=None)


def _committed_plan_path(record: dict[str, Any], current: dict[str, Any]) -> list[dict[str, Any]]:
    """Return the canonical forecast prefix covered by the committed Entry segment."""
    published = record.get("publishedPrediction")
    if not isinstance(published, list):
        return []
    planned_ut = optional_number(current.get("plannedUT"))
    duration = optional_number(current.get("segmentDuration"))
    if planned_ut is None or duration is None or duration <= 0.0:
        return []
    end_ut = planned_ut + duration
    path: list[dict[str, Any]] = []
    for item in published:
        if not isinstance(item, dict):
            continue
        point_ut = optional_number(item.get("ut"))
        if point_ut is None:
            continue
        if point_ut <= end_ut + 1e-6:
            path.append(item)
        elif path:
            break
    return path if len(path) >= 2 else []


def candidate_path(record: dict[str, Any]) -> list[dict[str, Any]]:
    current_value = record.get("currentPlan")
    if not _entry_plan_displayable(current_value):
        # Historical planner traces may contain local/diagnostic candidates even
        # when terminal delivery was not proven. Never render those as PLAN.
        return []
    current = current_value
    trace = record.get("plannerTrace")
    if isinstance(trace, dict):
        candidates = trace.get("candidates")
        if isinstance(candidates, list):
            usable = [
                item for item in candidates
                if isinstance(item, dict) and isinstance(item.get("candidatePath"), list)
            ]
            if usable:
                selected = next((item for item in usable if item.get("selected")), None)
                if selected is None:
                    target_bank = number(current.get("targetBank"))
                    target_aoa = number(current.get("targetAoA"))
                    target_heading = number(current.get("targetHeading"))

                    def score(item: dict[str, Any]) -> float:
                        command = item.get("command") if isinstance(item.get("command"), dict) else item
                        return (
                            abs(number(command.get("bank", command.get("targetBank")), target_bank) - target_bank)
                            + 0.5 * abs(number(command.get("aoa", command.get("targetAoA")), target_aoa) - target_aoa)
                            + 0.05 * abs(number(command.get("heading", command.get("targetHeading")), target_heading) - target_heading)
                        )

                    selected = min(usable, key=score)
                historical_path = [
                    item for item in selected.get("candidatePath", []) if isinstance(item, dict)
                ]
                if historical_path:
                    return historical_path

    # The current MM304 architecture has no second candidate-search policy. Use
    # only the portion of the canonical published forecast covered by currentPlan;
    # this keeps PLAN distinct from the longer PRED continuation.
    return _committed_plan_path(record, current)


class PlannerFollower:
    def __init__(self, root: Path) -> None:
        self.root = root
        self.path: Path | None = None
        self.offset = 0
        self.latest: dict[str, Any] = {}
        self.prediction: list[dict[str, Any]] = []
        self.planned: list[dict[str, Any]] = []
        self.trajectory_revision = 0
        self.last_record_wall = 0.0

    def poll(self) -> None:
        path = newest_planner_log(self.root)
        if path is None:
            return
        if path != self.path:
            self.path = path
            self.offset = 0
            self.latest = {}
            self.prediction = []
            self.planned = []
            self.trajectory_revision = 0
            self.last_record_wall = 0.0
        try:
            size = path.stat().st_size
            if size < self.offset:
                # Treat in-place log replacement/truncation like a new lineage epoch.
                self.offset = 0
                self.latest = {}
                self.prediction = []
                self.planned = []
                self.trajectory_revision = 0
            with path.open("r", encoding="utf-8") as handle:
                handle.seek(self.offset)
                for raw in handle:
                    try:
                        record = json.loads(raw)
                    except json.JSONDecodeError:
                        continue
                    if not isinstance(record, dict):
                        continue
                    self.latest = record
                    self.last_record_wall = time.time()
                    if record.get("trajectoryIncluded"):
                        published = record.get("publishedPrediction")
                        if isinstance(published, list):
                            self.prediction = [item for item in published if isinstance(item, dict)]
                        # A trajectory-bearing planner record is an authoritative snapshot of
                        # both propagated prediction and selected-plan geometry. An explicit
                        # empty candidate set therefore clears stale PLAN geometry just like an
                        # explicit empty publishedPrediction clears PRED.
                        self.planned = candidate_path(record)
                        self.trajectory_revision += 1
                self.offset = handle.tell()
        except OSError:
            return

    @property
    def fresh(self) -> bool:
        if self.path is None:
            return False
        try:
            modified = self.path.stat().st_mtime
        except OSError:
            return False
        return time.time() - max(modified, self.last_record_wall) < 8.0


class SnapshotStore:
    def __init__(self) -> None:
        self._condition = threading.Condition()
        self._version = 0
        self._snapshot: dict[str, Any] = {
            "connectionStatus": "connecting",
            "phase": "OFFLINE",
            "statusMessage": "Waiting for KSP",
            "telemetry": {},
            "command": {},
            "guidanceState": {},
            "actualTrajectory": [],
            "orbitalTrajectory": [],
            "orbitalTrajectoryMeta": {},
            "predictedTrajectory": [],
            "projectedTAEMTrajectory": [],
            "plannedTrajectory": [],
            "referenceTrajectory": [],
            "deorbitPlan": None,
            "rlTraining": empty_rl_training_status(),
            "server": {"generatedAt": time.time(), "source": "kRPC observer"},
        }

    def publish(self, snapshot: dict[str, Any]) -> None:
        with self._condition:
            self._snapshot = snapshot
            self._version += 1
            self._condition.notify_all()

    def current(self) -> tuple[int, dict[str, Any]]:
        with self._condition:
            return self._version, self._snapshot

    def wait_after(self, version: int, timeout: float) -> tuple[int, dict[str, Any]]:
        with self._condition:
            if self._version == version:
                self._condition.wait(timeout)
            return self._version, self._snapshot


def read_optional(obj: Any, name: str, default: Any = None) -> Any:
    try:
        return getattr(obj, name)
    except Exception:
        return default


def build_orbital_trajectory(
    orbit: Any,
    body: Any,
    current_ut: float,
    current_position: dict[str, Any],
    thrust: float = 0.0,
) -> tuple[list[dict[str, Any]], dict[str, Any]]:
    """Sample a display-only osculating vacuum path.

    kRPC's Orbit.position_at(..., body.reference_frame) resolves future orbital
    positions against the body's axes at the current instant. That is ideal for
    the 3D osculating orbit, but a 2D ground track must additionally remove the
    body's future rotation. Both coordinate sets are therefore published per
    point: latitude/longitude for the rotating-surface map and inertial* for the
    3D orbit. During powered flight this remains an explicitly instantaneous
    osculating orbit, not a prediction of the continuing burn.
    """
    if orbit is None or body is None:
        return [], {}

    period = optional_number(read_optional(orbit, "period"))
    reference_frame = read_optional(body, "reference_frame")
    equatorial_radius = optional_number(read_optional(body, "equatorial_radius"))
    rotation_period = optional_number(read_optional(body, "rotational_period"))
    atmosphere_depth = max(0.0, number(read_optional(body, "atmosphere_depth", 0.0)))
    current_altitude = optional_number(current_position.get("altitude"))
    if (
        period is None or period <= 1.0 or period > 7 * 86400.0
        or reference_frame is None or equatorial_radius is None or equatorial_radius <= 0.0
        or rotation_period is None or rotation_period <= 0.0
        or current_altitude is None
        or (atmosphere_depth > 0.0 and current_altitude <= atmosphere_depth)
    ):
        return [], {}

    rotation_rate = 360.0 / rotation_period

    def sample_at(sample_ut: float) -> dict[str, Any] | None:
        try:
            position = orbit.position_at(sample_ut, reference_frame)
            x, y, z = (float(position[0]), float(position[1]), float(position[2]))
        except Exception:
            return None
        radius = math.sqrt(x * x + y * y + z * z)
        if not math.isfinite(radius) or radius <= 1.0:
            return None
        inertial_latitude = math.degrees(math.atan2(y, math.hypot(x, z)))
        inertial_longitude = signed_degrees(math.degrees(math.atan2(z, x)))
        altitude = radius - equatorial_radius
        ground_longitude = signed_degrees(inertial_longitude - rotation_rate * (sample_ut - current_ut))
        return {
            "ut": sample_ut,
            "latitude": inertial_latitude,
            "longitude": ground_longitude,
            "altitude": altitude,
            "inertialLatitude": inertial_latitude,
            "inertialLongitude": inertial_longitude,
            "kind": "orbit",
        }

    points: list[dict[str, Any]] = []
    end_reason = "one-orbit"
    sample_count = max(17, ORBIT_TRAJECTORY_SAMPLE_COUNT)
    previous: dict[str, Any] | None = None
    for index in range(sample_count):
        sample_ut = current_ut + period * index / (sample_count - 1)
        point = sample_at(sample_ut)
        if point is None:
            break
        if index == 0:
            live_latitude = optional_number(current_position.get("latitude"))
            live_longitude = optional_number(current_position.get("longitude"))
            if live_latitude is not None and live_longitude is not None:
                point["latitude"] = live_latitude
                point["longitude"] = live_longitude
                point["inertialLatitude"] = live_latitude
                point["inertialLongitude"] = live_longitude
            point["altitude"] = current_altitude

        if atmosphere_depth > 0.0 and point["altitude"] <= atmosphere_depth:
            end_reason = "atmosphere-interface"
            if previous is not None and previous["altitude"] > atmosphere_depth:
                low_ut = float(previous["ut"]); high_ut = sample_ut
                crossing = point
                for _ in range(10):
                    mid_ut = 0.5 * (low_ut + high_ut)
                    mid = sample_at(mid_ut)
                    if mid is None:
                        break
                    crossing = mid
                    if mid["altitude"] > atmosphere_depth:
                        low_ut = mid_ut
                    else:
                        high_ut = mid_ut
                crossing["altitude"] = atmosphere_depth
                crossing["phase"] = "ENTRY INTERFACE"
                points.append(crossing)
            break

        points.append(point)
        previous = point

    if len(points) < 2:
        return [], {}
    if end_reason == "one-orbit":
        points[-1]["phase"] = "1 ORBIT"

    return points, {
        "kind": "vacuum-arc" if end_reason == "atmosphere-interface" else "osculating-orbit",
        "endReason": end_reason,
        "generatedAtUT": current_ut,
        "period": period,
        "rotationPeriod": rotation_period,
        "atmosphereDepth": atmosphere_depth,
        "sampleCount": len(points),
        "underThrust": thrust > ORBIT_TRAJECTORY_THRUST_EPSILON,
    }


def read_controller_mirror(root: Path, max_age: float | None = 1.5) -> dict[str, Any] | None:
    """Read the display-only exact backend snapshot mirror.

    Live mode requires freshness. Simulator/replay mode may intentionally retain
    the final frame after the producer exits, so max_age=None disables freshness
    rejection while keeping the exact same snapshot schema.
    """
    path = root / "Runtime" / "WebTelemetry" / "controller-snapshot.json"
    try:
        if max_age is not None and time.time() - path.stat().st_mtime > max_age:
            return None
        value = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError):
        return None
    if not isinstance(value, dict):
        return None
    if max_age is not None:
        generated_at = optional_number(value.get("generatedAt"))
        if generated_at is None or time.time() - generated_at > max_age:
            return None
    return value


def read_simulator_run(root: Path) -> dict[str, Any] | None:
    path = root / "Runtime" / "WebTelemetry" / "simulator-run.json"
    try:
        value = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError):
        return None
    return value if isinstance(value, dict) else None


def simulation_status(run: dict[str, Any] | None, snapshot: dict[str, Any] | None,
                      mode: str, *, replay_index: int | None = None,
                      replay_count: int | None = None, replay_speed: float | None = None) -> dict[str, Any]:
    run = run or {}
    snapshot = snapshot or {}
    telemetry = snapshot.get("telemetry") if isinstance(snapshot.get("telemetry"), dict) else {}
    state = str(run.get("state") or ("replay" if mode == "replay" else "idle"))
    started = optional_number(run.get("startedAt"))
    finished = optional_number(run.get("finishedAt"))
    wall_seconds = optional_number(run.get("wallSeconds"))
    if wall_seconds is None and started is not None:
        wall_seconds = max(0.0, (finished if finished is not None else time.time()) - started)
    sim_time = optional_number(telemetry.get("ut"))
    base_ut = optional_number((run.get("prerollFinal") or {}).get("ut")) if isinstance(run.get("prerollFinal"), dict) else None
    elapsed_sim = optional_number(run.get("simElapsedSeconds"))
    if elapsed_sim is not None and elapsed_sim <= 1e-9:
        elapsed_sim = None
    if elapsed_sim is None and sim_time is not None and base_ut is not None:
        elapsed_sim = max(0.0, sim_time - base_ut)
    effective_rate = None
    if elapsed_sim is not None and wall_seconds is not None and wall_seconds > 1e-6:
        effective_rate = elapsed_sim / wall_seconds
    result = {
        "active": mode in ("simulator", "replay"),
        "sourceMode": mode,
        "runId": run.get("runId"),
        "state": state,
        "mode": run.get("mode") or ("replay" if mode == "replay" else "closed-loop"),
        "scenario": run.get("scenario"),
        "rateMode": run.get("rateMode") or ("replay" if mode == "replay" else "max"),
        "lockstep": bool(run.get("lockstep", mode == "simulator")),
        "physicsDt": run.get("physicsDt"),
        "guidanceRateHz": run.get("guidanceRateHz"),
        "wallSeconds": wall_seconds,
        "simUT": sim_time,
        "simElapsedSeconds": elapsed_sim,
        "effectiveRate": effective_rate,
        "terminalPhase": run.get("terminalPhase") or snapshot.get("phase"),
        "final": run.get("final"),
        "replayAvailable": bool(
            run.get("guidanceSnapshots") or run.get("simulatorTelemetry") or replay_count
        ),
        "guidanceSnapshots": run.get("guidanceSnapshots"),
        "simulatorTelemetry": run.get("simulatorTelemetry"),
        "runDirectory": run.get("runDirectory"),
    }
    if replay_index is not None:
        result["replayIndex"] = replay_index
    if replay_count is not None:
        result["replayCount"] = replay_count
    if replay_speed is not None:
        result["replaySpeed"] = replay_speed
    return result


def empty_rl_queue_status() -> dict[str, Any]:
    return {
        "available": False,
        "state": "idle",
        "pid": None,
        "pidRunning": False,
        "session": None,
        "sessionName": None,
        "masterLog": None,
        "currentRun": None,
        "currentRunId": None,
        "currentRunLog": None,
        "deadlineEpoch": None,
        "startEpoch": None,
        "remainingSeconds": None,
        "runsCompleted": 0,
        "runsDiscovered": 0,
        "artifactAgeSeconds": None,
    }


def empty_rl_training_status() -> dict[str, Any]:
    return {
        "available": False,
        "state": "idle",
        "status": "NO RUN",
        "runId": None,
        "output": None,
        "fileAgeSeconds": None,
        "episodes": 0,
        "resets": 0,
        "eligibleSteps": 0,
        "activeSteps": 0,
        "fallbacks": 0,
        "touchdowns": 0,
        "successes": 0,
        "validRollouts": 0,
        "meanReturn": None,
        "latestPolicy": None,
        "latestSeed": None,
        "latestEvent": None,
        "latestCondition": None,
        "latestEpisode": None,
        "curriculum": None,
        "blockers": [],
        "queue": empty_rl_queue_status(),
    }


class RLTrainingMonitor:
    """Read-only monitor for the simulator-only RL JSONL output.

    This intentionally observes the training artifact and never imports or
    launches the learner.  The live telemetry server can therefore show RL
    progress without giving the RL process any path to the KSP controller.
    """

    def __init__(self, root: Path, output: Path | None = None) -> None:
        self.root = root
        requested = output.expanduser() if output is not None else None
        self.requested_output = (
            requested if requested is None or requested.is_absolute() else root / requested
        )
        self.output: Path | None = None
        self.train_path: Path | None = None
        self.train_offset = 0
        self.latest_train_signature: tuple[int, int] | None = None
        self.latest_report_signature: tuple[int, int] | None = None
        self.report: dict[str, Any] = {}
        self.resets = 0
        self.episodes = 0
        self.eligible_steps = 0
        self.active_steps = 0
        self.fallbacks = 0
        self.touchdowns = 0
        self.successes = 0
        self.valid_rollouts = 0
        self.return_total = 0.0
        self.latest_policy: str | None = None
        self.latest_seed: int | None = None
        self.latest_event: str | None = None
        self.latest_condition: dict[str, Any] | None = None
        self.latest_episode: dict[str, Any] | None = None
        self.queue_status = empty_rl_queue_status()
        self.queue_output: Path | None = None
        self.cached = empty_rl_training_status()

    def _reset_output_state(self, output: Path) -> None:
        self.output = output
        self.train_path = None
        self.train_offset = 0
        self.latest_train_signature = None
        self.latest_report_signature = None
        self.report = {}
        self.resets = self.episodes = 0
        self.eligible_steps = self.active_steps = self.fallbacks = 0
        self.touchdowns = self.successes = self.valid_rollouts = 0
        self.return_total = 0.0
        self.latest_policy = self.latest_seed = self.latest_event = None
        self.latest_condition = self.latest_episode = None

    def _relative_path(self, path: Path | None) -> str | None:
        if path is None:
            return None
        try:
            return str(path.resolve().relative_to(self.root.resolve()))
        except ValueError:
            return str(path)

    @staticmethod
    def _read_int(path: Path) -> int | None:
        try:
            raw = path.read_text(encoding="utf-8").strip()
            return int(raw) if raw else None
        except (OSError, ValueError):
            return None

    @staticmethod
    def _process_running(pid: int | None) -> bool:
        if pid is None or pid <= 0:
            return False
        try:
            os.kill(pid, 0)
        except OSError:
            return False
        return True

    @staticmethod
    def _activity_mtime(path: Path) -> float:
        candidates = [path, path / "train.jsonl", path / "validation.jsonl",
                      path / "training-report.json", Path(str(path) + ".log")]
        stamps = []
        for candidate in candidates:
            try:
                stamps.append(candidate.stat().st_mtime)
            except OSError:
                pass
        return max(stamps, default=0.0)

    def _slow_queue_status(self) -> dict[str, Any]:
        status = empty_rl_queue_status()
        marker = RL_SLOWRUNS_ROOT / "rl-sixhour.session"
        pid = self._read_int(RL_SLOWRUNS_ROOT / "rl-sixhour.pid")
        pid_running = self._process_running(pid)
        status["pid"] = pid
        status["pidRunning"] = pid_running
        try:
            raw_session = marker.read_text(encoding="utf-8").strip()
        except OSError:
            raw_session = ""
        if not raw_session:
            status["state"] = "running" if pid_running else "idle"
            return status
        session = Path(raw_session).expanduser()
        if not session.is_absolute():
            session = self.root / session
        if not session.is_dir():
            status["state"] = "stale" if raw_session else "idle"
            status["session"] = raw_session
            return status
        self.queue_output = None
        start_epoch = self._read_int(session / "start.epoch")
        deadline_epoch = self._read_int(session / "deadline.epoch")
        now = time.time()
        remaining = (max(0.0, float(deadline_epoch) - now)
                     if deadline_epoch is not None else None)
        try:
            runs = [path for path in session.iterdir()
                    if path.is_dir() and path.name.startswith("run-")]
        except OSError:
            runs = []
        runs.sort(key=self._activity_mtime, reverse=True)
        current = runs[0] if runs else None
        if current and ((current / "train.jsonl").is_file() or
                        (current / "training-report.json").is_file()):
            self.queue_output = current
        completed = sum(1 for run in runs if (run / "training-report.json").is_file())
        master_log = session / "master.log"
        activity_age = None
        latest_activity = max(
            [self._activity_mtime(run) for run in runs] +
            ([master_log.stat().st_mtime] if master_log.is_file() else []),
            default=0.0,
        )
        if latest_activity > 0.0:
            activity_age = max(0.0, now - latest_activity)
        state = "running" if pid_running and (remaining is None or remaining > 0.0) else (
            "finished" if completed or (deadline_epoch is not None and remaining == 0.0) else "stale"
        )
        status.update({
            "available": True,
            "state": state,
            "session": self._relative_path(session),
            "sessionName": session.name,
            "masterLog": self._relative_path(master_log) if master_log.is_file() else None,
            "currentRun": self._relative_path(current),
            "currentRunId": current.name if current else None,
            "currentRunLog": self._relative_path(Path(str(current) + ".log"))
                if current and Path(str(current) + ".log").is_file() else None,
            "deadlineEpoch": deadline_epoch,
            "startEpoch": start_epoch,
            "remainingSeconds": remaining,
            "runsCompleted": completed,
            "runsDiscovered": len(runs),
            "artifactAgeSeconds": activity_age,
        })
        return status

    def _select_output(self) -> Path | None:
        if self.requested_output is not None:
            return self.requested_output if self.requested_output.is_dir() else None
        if self.queue_output is not None:
            return self.queue_output
        try:
            candidates = [
                path for path in RL_RESULTS_ROOT.iterdir()
                if path.is_dir() and (
                    (path / "train.jsonl").is_file() or
                    (path / "training-report.json").is_file()
                )
            ]
        except OSError:
            return None
        if not candidates:
            return None
        return max(
            candidates,
            key=lambda path: max(
                (candidate.stat().st_mtime for candidate in (
                    path / "train.jsonl", path / "training-report.json"
                ) if candidate.is_file()),
                default=0.0,
            ),
        )

    def _consume_train_records(self, path: Path) -> None:
        try:
            signature = (path.stat().st_size, path.stat().st_mtime_ns)
        except OSError:
            return
        output_changed = path != self.train_path
        truncated = signature[0] < self.train_offset
        if output_changed or truncated:
            self.train_path = path
            self.train_offset = 0
            self.latest_train_signature = None
            if truncated:
                self.resets = self.episodes = 0
                self.eligible_steps = self.active_steps = self.fallbacks = 0
                self.touchdowns = self.successes = self.valid_rollouts = 0
                self.return_total = 0.0
                self.latest_policy = self.latest_seed = self.latest_event = None
                self.latest_condition = self.latest_episode = None
        if signature == self.latest_train_signature:
            return
        try:
            with path.open("r", encoding="utf-8") as handle:
                handle.seek(self.train_offset)
                for raw in handle:
                    try:
                        record = json.loads(raw)
                    except json.JSONDecodeError:
                        continue
                    if not isinstance(record, dict):
                        continue
                    kind = record.get("type")
                    self.latest_policy = str(record.get("policy")) if record.get("policy") is not None else self.latest_policy
                    if kind == "reset":
                        self.resets += 1
                        seed = record.get("seed")
                        self.latest_seed = int(seed) if isinstance(seed, int) else self.latest_seed
                        condition = record.get("initial_condition")
                        self.latest_condition = copy.deepcopy(condition) if isinstance(condition, dict) else None
                        self.latest_event = "reset"
                    elif kind == "episode":
                        self.episodes += 1
                        self.eligible_steps += int(number(record.get("eligible_steps"), 0.0))
                        self.active_steps += int(number(record.get("active_steps"), 0.0))
                        self.fallbacks += int(number(record.get("fallbacks"), 0.0))
                        metrics = record.get("metrics") if isinstance(record.get("metrics"), dict) else {}
                        self.touchdowns += int(bool(metrics.get("touchdown")))
                        self.successes += int(bool(record.get("success")))
                        self.valid_rollouts += int(bool(record.get("rollout_valid")))
                        self.return_total += number(record.get("return"), 0.0)
                        self.latest_episode = {
                            "outcome": record.get("outcome"),
                            "phase": record.get("phase"),
                            "elapsed": record.get("elapsed"),
                            "return": record.get("return"),
                            "activeSteps": record.get("active_steps"),
                            "eligibleSteps": record.get("eligible_steps"),
                            "touchdown": bool(metrics.get("touchdown")),
                            "success": bool(record.get("success")),
                            "rolloutValid": bool(record.get("rollout_valid")),
                        }
                        self.latest_event = "episode"
                self.train_offset = handle.tell()
        except OSError:
            return
        self.latest_train_signature = signature

    def _read_report(self, path: Path) -> None:
        try:
            signature = (path.stat().st_size, path.stat().st_mtime_ns)
        except OSError:
            return
        if signature == self.latest_report_signature:
            return
        try:
            value = json.loads(path.read_text(encoding="utf-8"))
        except (OSError, json.JSONDecodeError):
            return
        self.report = value if isinstance(value, dict) else {}
        self.latest_report_signature = signature

    def poll(self) -> dict[str, Any]:
        self.queue_output = None
        self.queue_status = self._slow_queue_status()
        output = self._select_output()
        if output is None:
            self.cached = empty_rl_training_status()
            self.cached["queue"] = copy.deepcopy(self.queue_status)
            if self.queue_status.get("available"):
                self.cached.update({
                    "available": True,
                    "state": self.queue_status.get("state") or "idle",
                    "status": "SIX_HOUR_QUEUE",
                    "runId": self.queue_status.get("sessionName"),
                    "output": self.queue_status.get("session"),
                    "fileAgeSeconds": self.queue_status.get("artifactAgeSeconds"),
                })
            return copy.deepcopy(self.cached)
        if output != self.output:
            self._reset_output_state(output)
        train_path = output / "train.jsonl"
        report_path = output / "training-report.json"
        if train_path.is_file():
            self._consume_train_records(train_path)
        if report_path.is_file():
            self._read_report(report_path)
        try:
            latest_mtime = max(
                candidate.stat().st_mtime for candidate in (train_path, report_path)
                if candidate.is_file()
            )
            age = max(0.0, time.time() - latest_mtime)
        except (OSError, ValueError):
            age = None
        finished = bool(self.report)
        queue_running = bool(self.queue_status.get("pidRunning")) and self.requested_output is None
        state = "running" if queue_running else "finished" if finished else "running" if age is not None and age < 8.0 else "stale"
        report_status = self.report.get("status") if isinstance(self.report, dict) else None
        relative_output = str(output)
        try:
            relative_output = str(output.resolve().relative_to(self.root.resolve()))
        except ValueError:
            pass
        self.cached = {
            "available": True,
            "state": state,
            "status": str(report_status or ("SIX_HOUR_QUEUE" if self.queue_status.get("available") else ("TRAINING" if state == "running" else "LAST RUN"))),
            "runId": output.name,
            "output": relative_output,
            "fileAgeSeconds": age,
            "episodes": self.episodes,
            "resets": self.resets,
            "eligibleSteps": self.eligible_steps,
            "activeSteps": self.active_steps,
            "fallbacks": self.fallbacks,
            "touchdowns": self.touchdowns,
            "successes": self.successes,
            "validRollouts": self.valid_rollouts,
            "meanReturn": self.return_total / self.episodes if self.episodes else None,
            "latestPolicy": self.latest_policy,
            "latestSeed": self.latest_seed,
            "latestEvent": self.latest_event,
            "latestCondition": copy.deepcopy(self.latest_condition),
            "latestEpisode": copy.deepcopy(self.latest_episode),
            "curriculum": self.report.get("curriculum") if isinstance(self.report, dict) else None,
            "blockers": (self.report.get("promotion", {}).get("blockers")
                         if isinstance(self.report.get("promotion"), dict) else []),
            "queue": copy.deepcopy(self.queue_status),
        }
        return copy.deepcopy(self.cached)



def _downsample_replay_points(points: Any, limit: int = 120) -> list[dict[str, Any]]:
    if not isinstance(points, list):
        return []
    clean = [point for point in points if isinstance(point, dict)]
    if len(clean) <= limit:
        return copy.deepcopy(clean)
    if limit <= 2:
        return [copy.deepcopy(clean[0]), copy.deepcopy(clean[-1])]
    scale = (len(clean) - 1) / float(limit - 1)
    indices = sorted({min(len(clean) - 1, int(round(i * scale))) for i in range(limit)})
    return [copy.deepcopy(clean[i]) for i in indices]


def _first_simulator_packet(path: Path) -> dict[str, Any] | None:
    try:
        with path.open("r", encoding="utf-8") as handle:
            for line in handle:
                try:
                    value = json.loads(line)
                except json.JSONDecodeError:
                    continue
                if isinstance(value, dict) and value.get("source") == "sim" and value.get("type") == "telemetry":
                    return value
    except OSError:
        return None
    return None


def _last_simulator_packet(path: Path) -> dict[str, Any] | None:
    try:
        with path.open("rb") as handle:
            handle.seek(0, 2)
            position = handle.tell()
            carry = b""
            while position > 0:
                size = min(65536, position)
                position -= size
                handle.seek(position)
                data = handle.read(size) + carry
                lines = data.splitlines()
                if position > 0 and lines:
                    carry = lines.pop(0)
                else:
                    carry = b""
                for raw in reversed(lines):
                    if not raw.strip():
                        continue
                    try:
                        value = json.loads(raw.decode("utf-8"))
                    except (UnicodeDecodeError, json.JSONDecodeError):
                        continue
                    if (isinstance(value, dict) and value.get("source") == "sim" and
                            value.get("type") == "telemetry"):
                        return value
    except OSError:
        return None
    return None


def _simulator_elapsed_seconds(manifest: dict[str, Any], simulator_file: Path) -> float | None:
    elapsed = optional_number(manifest.get("simElapsedSeconds"))
    if elapsed is not None and elapsed >= 1.0:
        return elapsed
    packet = _last_simulator_packet(simulator_file) if simulator_file.is_file() else None
    elapsed = optional_number(packet.get("sim_time")) if packet else None
    return elapsed if elapsed is not None and elapsed >= 1.0 else None


def _legacy_run_manifest(root: Path, run_id: str) -> tuple[Path, dict[str, Any]] | None:
    runs_root = root / "ShuttleSim" / "runs"
    guidance_path = runs_root / f"{run_id}-guidance.jsonl"
    simulator_path = runs_root / f"{run_id}-sim.jsonl"
    exact_path = runs_root / f"{run_id}.jsonl"
    if not simulator_path.is_file() and exact_path.is_file() and _first_simulator_packet(exact_path):
        simulator_path = exact_path
    if not guidance_path.is_file() and not simulator_path.is_file():
        return None

    timestamps = [path.stat().st_mtime for path in (guidance_path, simulator_path) if path.is_file()]
    first_packet = _first_simulator_packet(simulator_path) if simulator_path.is_file() else None
    manifest: dict[str, Any] = {
        "schema": 1,
        "runId": run_id,
        "state": "finished",
        "mode": "legacy-replay",
        "scenario": first_packet.get("scenario") if first_packet else None,
        "startedAt": min(timestamps) if timestamps else None,
        "finishedAt": max(timestamps) if timestamps else None,
        "guidanceSnapshots": str(guidance_path) if guidance_path.is_file() else None,
        "simulatorTelemetry": str(simulator_path) if simulator_path.is_file() else None,
        "runDirectory": str(runs_root),
    }
    return runs_root, manifest


def _discover_simulation_run(root: Path, run_id: str) -> tuple[Path, dict[str, Any]] | None:
    if not run_id or "/" in run_id or "\\" in run_id or run_id in {".", ".."}:
        return None
    runs_root = (root / "ShuttleSim" / "runs").resolve()
    run_dir = (runs_root / run_id).resolve()
    if run_dir.is_relative_to(runs_root):
        manifest_path = run_dir / "manifest.json"
        try:
            manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
        except (OSError, json.JSONDecodeError):
            manifest = None
        if isinstance(manifest, dict):
            return run_dir, manifest
    return _legacy_run_manifest(root, run_id)


def _legacy_run_ids(root: Path) -> set[str]:
    runs_root = root / "ShuttleSim" / "runs"
    ids: set[str] = set()
    try:
        paths = list(runs_root.glob("*.jsonl"))
    except OSError:
        return ids
    for path in paths:
        name = path.name
        if name.endswith("-guidance.jsonl"):
            ids.add(name[:-len("-guidance.jsonl")])
        elif name.endswith("-sim.jsonl"):
            ids.add(name[:-len("-sim.jsonl")])
        elif _first_simulator_packet(path):
            ids.add(path.stem)
    return ids



def _simulation_run_summary(run_dir: Path, manifest: dict[str, Any]) -> dict[str, Any]:
    run_id = str(manifest.get("runId") or run_dir.name)
    replay_file = _simulation_run_artifact(
        run_dir, manifest.get("guidanceSnapshots"), "guidance-snapshots.jsonl"
    )
    simulator_file = _simulation_run_artifact(
        run_dir, manifest.get("simulatorTelemetry"), "simulator-telemetry.jsonl"
    )
    sim_elapsed = _simulator_elapsed_seconds(manifest, simulator_file)
    final = manifest.get("final") if isinstance(manifest.get("final"), dict) else {}
    terminal = str(manifest.get("terminalPhase") or "")
    landed = str(final.get("situation") or "").lower() == "landed"
    along = optional_number(final.get("runwayAlongTrack"))
    cross = optional_number(final.get("runwayCrossTrack"))
    success = terminal == "Complete" or (
        landed and along is not None and cross is not None and abs(along) <= 1800.0 and abs(cross) <= 100.0
    )
    try:
        guidance_bytes = replay_file.stat().st_size if replay_file.is_file() else 0
        simulator_bytes = simulator_file.stat().st_size if simulator_file.is_file() else 0
    except OSError:
        guidance_bytes = 0
        simulator_bytes = 0
    replay_bytes = guidance_bytes + simulator_bytes
    has_replay = replay_bytes > 0
    return {
        "runId": run_id,
        "state": manifest.get("state"),
        "mode": manifest.get("mode"),
        "scenario": manifest.get("scenario"),
        "startedAt": manifest.get("startedAt"),
        "finishedAt": manifest.get("finishedAt"),
        "wallSeconds": manifest.get("wallSeconds"),
        "simElapsedSeconds": sim_elapsed,
        "terminalPhase": terminal or None,
        "final": final,
        "success": success,
        "hasReplay": has_replay,
        "replayBytes": replay_bytes,
        "guidanceReplayBytes": guidance_bytes,
        "simulatorReplayBytes": simulator_bytes,
        "runDirectory": str(manifest.get("runDirectory") or run_dir),
    }


def list_simulation_runs(root: Path) -> list[dict[str, Any]]:
    runs_root = root / "ShuttleSim" / "runs"
    result: list[dict[str, Any]] = []
    seen: set[str] = set()
    try:
        candidates = list(runs_root.glob("*/manifest.json"))
    except OSError:
        candidates = []
    for manifest_path in candidates:
        try:
            manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
        except (OSError, json.JSONDecodeError):
            continue
        if not isinstance(manifest, dict):
            continue
        summary = _simulation_run_summary(manifest_path.parent, manifest)
        result.append(summary)
        seen.add(summary["runId"])

    for run_id in sorted(_legacy_run_ids(root)):
        if run_id in seen:
            continue
        discovered = _legacy_run_manifest(root, run_id)
        if discovered is None:
            continue
        run_dir, manifest = discovered
        result.append(_simulation_run_summary(run_dir, manifest))

    result = [
        row for row in result
        if row.get("hasReplay") and optional_number(row.get("simElapsedSeconds")) is not None
        and number(row.get("simElapsedSeconds")) >= 1.0
    ]
    result.sort(key=lambda row: number(row.get("startedAt"), 0.0), reverse=True)
    return result


def _replay_configuration(root: Path, run: dict[str, Any] | None,
                          fallback: dict[str, Any] | None = None) -> dict[str, Any]:
    run = run or {}
    candidate = run.get("config")
    if isinstance(candidate, str) and candidate:
        path = Path(candidate).expanduser()
        if not path.is_absolute():
            path = root / path
        loaded = load_configuration(path)
        if loaded:
            return loaded
    if isinstance(fallback, dict) and fallback:
        return copy.deepcopy(fallback)
    return load_configuration(DEFAULT_CONFIG)


def _replay_terminal_geometry(snapshot: dict[str, Any]) -> dict[str, Any]:
    guidance = snapshot.get("guidanceState") if isinstance(snapshot.get("guidanceState"), dict) else {}
    return {
        "terminalCandidateValid": bool(guidance.get("terminalCandidateValid")),
        "candidateRadius": guidance.get("terminalCandidateRadius"),
        "candidateAltitude": guidance.get("terminalCandidateAltitude"),
        "candidateSpeed": guidance.get("terminalCandidateSpeed"),
        "referenceFPA": guidance.get("terminalReferenceFPA"),
        "mix": guidance.get("terminalBlend"),
        "hacRemaining": guidance.get("terminalPathRemaining"),
        "hacRadius": guidance.get("hacRadius"),
        "hacTransitionProgress": guidance.get("hacTransitionProgress"),
        "commandedCourseRate": guidance.get("commandedCourseRateEstimate"),
        "hacCircuitCount": guidance.get("hacCircuitCount"),
        "terminalCommitted": bool(
            guidance.get("terminalPathCommitted")
            or guidance.get("terminalPathCaptured")
            or guidance.get("terminalPathComplete")
        ),
    }


def normalize_replay_snapshot(snapshot: dict[str, Any],
                              configuration: dict[str, Any]) -> dict[str, Any]:
    """Convert archived exact-controller frames to the live WebTelemetry schema."""
    out = copy.deepcopy(snapshot)

    if not isinstance(out.get("deorbitPlan"), dict):
        plan = out.get("plan")
        if isinstance(plan, dict):
            out["deorbitPlan"] = copy.deepcopy(plan)

    configured_site = configuration.get("site") if isinstance(configuration.get("site"), dict) else {}
    if not isinstance(out.get("site"), dict) or not out.get("site"):
        out["site"] = copy.deepcopy(configured_site)

    telemetry = out.get("telemetry")
    if isinstance(telemetry, dict) and not telemetry.get("vesselName"):
        telemetry["vesselName"] = "STS-N · ShuttleSim"

    if "plannerFresh" not in out:
        out["plannerFresh"] = bool(
            out.get("tickSequence") is not None
            or out.get("predictedTrajectory")
            or out.get("referenceTrajectory")
        )

    apply_exact_trajectory_geometry(
        out,
        snapshot,
        _replay_terminal_geometry(snapshot),
        configuration,
    )

    for key in (
        "predictedTrajectory", "projectedTAEMTrajectory", "plannedTrajectory",
        "referenceTrajectory", "orbitalTrajectory", "replayCandidatePrediction",
        "replayCandidateTrajectory",
    ):
        if not isinstance(out.get(key), list):
            out[key] = []
    if not isinstance(out.get("orbitalTrajectoryMeta"), dict):
        out["orbitalTrajectoryMeta"] = {}
    return out


def compact_replay_snapshot(snapshot: dict[str, Any], run: dict[str, Any],
                            index: int, count: int,
                            configuration: dict[str, Any]) -> dict[str, Any]:
    snapshot = normalize_replay_snapshot(snapshot, configuration)
    keep = (
        "connectionStatus", "phase", "statusMessage", "warningMessage", "lastError",
        "automationEngaged", "plannerFresh", "tickSequence", "trajectoryRevision",
        "trajectoryLineage", "telemetry", "command", "guidanceState", "deorbitPlan",
        "site", "orbitalTrajectoryMeta",
    )
    out: dict[str, Any] = {}
    for key in keep:
        if key in snapshot:
            out[key] = copy.deepcopy(snapshot[key])
    for key in (
        "predictedTrajectory", "projectedTAEMTrajectory", "plannedTrajectory",
        "referenceTrajectory", "orbitalTrajectory", "replayCandidatePrediction",
        "replayCandidateTrajectory",
    ):
        out[key] = _downsample_replay_points(snapshot.get(key), 120)
    out["actualTrajectory"] = []
    out["simulation"] = simulation_status(
        {**run, "state": "replay"}, out, "replay",
        replay_index=index, replay_count=count, replay_speed=1.0,
    )
    out["server"] = {
        "generatedAt": time.time(),
        "source": "ShuttleSim archive replay",
        "simulation": True,
    }
    return out


def _simulation_run_artifact(run_dir: Path, value: Any, fallback_name: str) -> Path:
    if isinstance(value, str) and value:
        path = Path(value).expanduser()
        return path if path.is_absolute() else run_dir / path
    return run_dir / fallback_name


def load_simulator_replay_frames(run_dir: Path, run: dict[str, Any],
                                 max_frames: int) -> list[dict[str, Any]]:
    """Load a bounded dense physics timeline for archive replay.

    Guidance snapshots are intentionally sparse because trajectory generation is
    expensive. The simulator telemetry stream is ~10 Hz and is the authoritative
    source for the moving vehicle pose between those guidance updates.
    """
    path = _simulation_run_artifact(
        run_dir, run.get("simulatorTelemetry"), "simulator-telemetry.jsonl"
    )
    packets: list[dict[str, Any]] = []
    base = run.get("prerollFinal") if isinstance(run.get("prerollFinal"), dict) else {}
    final = run.get("final") if isinstance(run.get("final"), dict) else {}
    base_ut = optional_number(base.get("ut"))
    final_ut = optional_number(final.get("ut"))
    try:
        with path.open("r", encoding="utf-8") as handle:
            for line in handle:
                try:
                    packet = json.loads(line)
                except json.JSONDecodeError:
                    continue
                if not isinstance(packet, dict) or packet.get("source") != "sim":
                    continue
                ut = optional_number(packet.get("ut"))
                if base_ut is not None and ut is not None and ut + 1e-6 < base_ut:
                    continue
                if final_ut is not None and ut is not None and ut - 1e-6 > final_ut:
                    continue
                packets.append(packet)
    except OSError:
        return []

    if not packets:
        return []
    limit = max(20, min(2000, max_frames))
    if len(packets) > limit:
        scale = (len(packets) - 1) / float(limit - 1)
        indices = sorted({min(len(packets) - 1, int(round(i * scale))) for i in range(limit)})
        packets = [packets[i] for i in indices]

    frames: list[dict[str, Any]] = []
    for packet in packets:
        ut = optional_number(packet.get("ut"))
        sim_time = optional_number(packet.get("sim_time"))
        elapsed = (
            max(0.0, ut - base_ut)
            if ut is not None and base_ut is not None
            else sim_time
        )
        attitude = packet.get("attitude") if isinstance(packet.get("attitude"), dict) else {}
        ground = packet.get("ground") if isinstance(packet.get("ground"), dict) else {}
        frames.append({
            "ut": ut,
            "simTime": sim_time,
            "simElapsedSeconds": elapsed,
            "simRate": optional_number(packet.get("sim_rate")),
            "telemetry": simulator_packet_telemetry(packet),
            "command": {
                "targetAoA": optional_number(attitude.get("cmd_aoa_deg")),
                "targetRoll": optional_number(attitude.get("cmd_bank_deg")),
                "gear": bool(ground.get("gear_down")),
            },
        })
    return frames


def _replay_prediction_has_future(snapshot: dict[str, Any], current_ut: float | None) -> bool:
    points = snapshot.get("predictedTrajectory")
    if not isinstance(points, list) or not points:
        return False
    if current_ut is None:
        return True
    for point in points:
        if not isinstance(point, dict):
            continue
        point_ut = optional_number(point.get("ut"))
        if point_ut is None or point_ut > current_ut + 1e-6:
            return True
    return False


def hydrate_archive_replay_frames(guidance_frames: list[dict[str, Any]],
                                  simulator_frames: list[dict[str, Any]],
                                  configuration: dict[str, Any]) -> list[dict[str, Any]]:
    """Overlay sparse controller snapshots on the dense archived physics stream."""
    if not simulator_frames:
        return [normalize_replay_snapshot(frame, configuration) for frame in guidance_frames]

    guidance = [frame for frame in guidance_frames if isinstance(frame, dict)]
    timed_guidance: list[dict[str, Any]] = []
    for frame in guidance:
        telemetry = frame.get("telemetry") if isinstance(frame.get("telemetry"), dict) else {}
        frame_ut = optional_number(telemetry.get("ut"))
        if frame_ut is not None and frame_ut > 1.0:
            timed_guidance.append(frame)

    current: dict[str, Any] = guidance[0] if guidance else {}
    current_normalized = normalize_replay_snapshot(current, configuration)
    cursor = -1
    held_prediction: list[dict[str, Any]] = []
    held_prediction_revision: int | None = None
    actual: list[dict[str, Any]] = []
    result: list[dict[str, Any]] = []

    for simulator_frame in simulator_frames:
        sim_ut = optional_number(simulator_frame.get("ut"))
        while cursor + 1 < len(timed_guidance):
            candidate = timed_guidance[cursor + 1]
            candidate_telemetry = candidate.get("telemetry") if isinstance(candidate.get("telemetry"), dict) else {}
            candidate_ut = optional_number(candidate_telemetry.get("ut"))
            if sim_ut is None or candidate_ut is None or candidate_ut > sim_ut + 0.05:
                break
            cursor += 1
            current = timed_guidance[cursor]
            current_normalized = normalize_replay_snapshot(current, configuration)
            prediction = current.get("predictedTrajectory")
            if isinstance(prediction, list) and prediction and _replay_prediction_has_future(current, sim_ut):
                held_prediction = copy.deepcopy(prediction)
                revision = optional_number(current.get("trajectoryRevision", current.get("tickSequence")))
                held_prediction_revision = int(revision) if revision is not None else cursor
            else:
                guidance_state = current.get("guidanceState") if isinstance(current.get("guidanceState"), dict) else {}
                if (
                    guidance_state.get("terminalPredictionValid") is False
                    or not _replay_prediction_has_future({"predictedTrajectory": held_prediction}, sim_ut)
                ):
                    held_prediction = []
                    held_prediction_revision = None

        base = copy.deepcopy(current_normalized)
        if not _replay_prediction_has_future(base, sim_ut) and _replay_prediction_has_future(
            {"predictedTrajectory": held_prediction}, sim_ut
        ):
            base["predictedTrajectory"] = copy.deepcopy(held_prediction)
            if held_prediction_revision is not None:
                base["trajectoryRevision"] = held_prediction_revision

        sim_telemetry = simulator_frame.get("telemetry")
        if isinstance(sim_telemetry, dict):
            existing = base.get("telemetry") if isinstance(base.get("telemetry"), dict) else {}
            base["telemetry"] = {**existing, **sim_telemetry}
        if not isinstance(base.get("command"), dict) or not base["command"]:
            base["command"] = copy.deepcopy(simulator_frame.get("command") or {})

        telemetry = base.get("telemetry") if isinstance(base.get("telemetry"), dict) else {}
        latitude = optional_number(telemetry.get("latitude"))
        longitude = optional_number(telemetry.get("longitude"))
        altitude = optional_number(telemetry.get("meanAltitude"))
        if latitude is not None and longitude is not None and altitude is not None:
            actual.append({
                "ut": optional_number(telemetry.get("ut")),
                "latitude": latitude,
                "longitude": longitude,
                "altitude": altitude,
                "phase": base.get("phase") or "REPLAY",
            })
            if len(actual) > 900:
                actual = actual[::2]
        base["actualTrajectory"] = copy.deepcopy(actual)
        base["connectionStatus"] = "connected"
        result.append(base)

    return result


def load_simulation_replay(root: Path, run_id: str, max_frames: int = 600) -> dict[str, Any] | None:
    discovered = _discover_simulation_run(root, run_id)
    if discovered is None:
        return None
    run_dir, manifest = discovered

    max_frames = max(20, min(2000, max_frames))
    candidate = manifest.get("guidanceSnapshots")
    replay_path = _simulation_run_artifact(run_dir, candidate, "guidance-snapshots.jsonl")
    frames = load_replay_frames(replay_path)
    simulator_frames = load_simulator_replay_frames(run_dir, manifest, max_frames)
    if not frames:
        return {"run": manifest, "frames": [], "simulatorFrames": simulator_frames}

    if len(frames) > max_frames:
        scale = (len(frames) - 1) / float(max_frames - 1)
        indices = sorted({min(len(frames) - 1, int(round(i * scale))) for i in range(max_frames)})
        frames = [frames[i] for i in indices]
    configuration = _replay_configuration(root, manifest)
    compact = [
        compact_replay_snapshot(frame, manifest, i, len(frames), configuration)
        for i, frame in enumerate(frames)
    ]
    return {"run": manifest, "frames": compact, "simulatorFrames": simulator_frames}


def simulator_packet_telemetry(packet: dict[str, Any]) -> dict[str, Any]:
    pos = packet.get("position") if isinstance(packet.get("position"), dict) else {}
    vel = packet.get("velocity") if isinstance(packet.get("velocity"), dict) else {}
    att = packet.get("attitude") if isinstance(packet.get("attitude"), dict) else {}
    aero = packet.get("aero") if isinstance(packet.get("aero"), dict) else {}
    runway = packet.get("runway") if isinstance(packet.get("runway"), dict) else {}
    ground = packet.get("ground") if isinstance(packet.get("ground"), dict) else {}
    airspeed = optional_number(vel.get("air_mps"))
    vertical = optional_number(vel.get("vertical_mps"))
    fpa = None
    if airspeed is not None and airspeed > 1e-6 and vertical is not None:
        fpa = math.degrees(math.asin(max(-1.0, min(1.0, vertical / airspeed))))
    aoa = optional_number(att.get("aoa_deg"))
    pitch = fpa + aoa if fpa is not None and aoa is not None else None
    along = optional_number(runway.get("along_m"))
    cross = optional_number(runway.get("cross_m"))
    range_to_site = math.hypot(along, cross) if along is not None and cross is not None else None
    return {
        "ut": optional_number(packet.get("ut")),
        "meanAltitude": optional_number(pos.get("altitude_m")),
        "radarAltitude": optional_number(pos.get("altitude_m")),
        "latitude": optional_number(pos.get("lat_deg")),
        "longitude": optional_number(pos.get("lon_deg")),
        "surfaceSpeed": optional_number(vel.get("surface_mps")),
        "trueAirSpeed": airspeed,
        "verticalSpeed": vertical,
        "flightPathAngle": fpa,
        "pitch": pitch,
        "roll": optional_number(att.get("bank_deg")),
        "heading": optional_number(att.get("heading_deg")),
        "angleOfAttack": aoa,
        "mach": optional_number(aero.get("mach")),
        "dynamicPressure": optional_number(aero.get("q_pa")),
        "liftForce": optional_number(aero.get("lift_n")),
        "dragForce": optional_number(aero.get("drag_n")),
        "runwayAlongTrack": along,
        "runwayCrossTrack": cross,
        "rangeToSite": range_to_site,
        "gear": bool(ground.get("gear_down")),
        "vesselSituation": "landed" if ground.get("on_ground") else "flying",
        "vesselName": "STS-N · ShuttleSim",
    }


def merge_simulator_packet(snapshot: dict[str, Any] | None, packet: dict[str, Any],
                           run: dict[str, Any] | None, trail: list[dict[str, Any]]) -> dict[str, Any]:
    out = copy.deepcopy(snapshot) if isinstance(snapshot, dict) else {}
    sim_tel = simulator_packet_telemetry(packet)
    existing_tel = out.get("telemetry") if isinstance(out.get("telemetry"), dict) else {}
    out["telemetry"] = {**existing_tel, **{k: v for k, v in sim_tel.items() if v is not None}}
    att = packet.get("attitude") if isinstance(packet.get("attitude"), dict) else {}
    ground = packet.get("ground") if isinstance(packet.get("ground"), dict) else {}
    existing_cmd = out.get("command") if isinstance(out.get("command"), dict) else {}
    if not existing_cmd:
        out["command"] = {
            "targetAoA": optional_number(att.get("cmd_aoa_deg")),
            "targetRoll": optional_number(att.get("cmd_bank_deg")),
            "gear": bool(ground.get("gear_down")),
        }
    else:
        out["command"] = existing_cmd
    out.setdefault("guidanceState", {})
    out.setdefault("predictedTrajectory", [])
    out.setdefault("projectedTAEMTrajectory", [])
    out.setdefault("plannedTrajectory", [])
    out.setdefault("referenceTrajectory", [])
    out.setdefault("orbitalTrajectory", [])
    out.setdefault("orbitalTrajectoryMeta", {})
    out.setdefault("site", {})
    out["actualTrajectory"] = copy.deepcopy(trail)
    out["connectionStatus"] = "connected"
    if not out.get("phase") or str(out.get("phase")).upper() == "OFFLINE":
        out["phase"] = "Simulation"
    out["statusMessage"] = f"ShuttleSim live · {packet.get('scenario') or 'scenario'}"
    sim = simulation_status(run, out, "simulator")
    sim["active"] = True
    sim["state"] = (run or {}).get("state") or ("landed" if ground.get("on_ground") else "running")
    sim["simTime"] = optional_number(packet.get("sim_time"))
    sim["simElapsedSeconds"] = optional_number(packet.get("sim_time"))
    sim["simUT"] = optional_number(packet.get("ut"))
    sim["effectiveRate"] = optional_number(packet.get("sim_rate"))
    sim["scenario"] = packet.get("scenario") or sim.get("scenario")
    out["simulation"] = sim
    out["server"] = {
        **(out.get("server") if isinstance(out.get("server"), dict) else {}),
        "generatedAt": time.time(),
        "source": "ShuttleSim UDP + Guidance",
        "simulation": True,
    }
    return out


def simulator_collector_loop(store: "SnapshotStore", root: Path, stop: threading.Event,
                             sim_telemetry_port: int = 8797,
                             rl_output: Path | None = None) -> None:
    rl_monitor = RLTrainingMonitor(root, rl_output)
    rl_status = rl_monitor.poll()
    last_rl_poll = 0.0
    last_signature: tuple[Any, ...] | None = None
    last_packet: dict[str, Any] | None = None
    last_packet_wall = 0.0
    trail: list[dict[str, Any]] = []
    last_trail_time = -1e30
    sock: socket.socket | None = None
    if sim_telemetry_port > 0:
        try:
            sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            sock.bind(("127.0.0.1", sim_telemetry_port))
            sock.setblocking(False)
        except OSError as exc:
            if sock is not None:
                sock.close()
            sock = None
            store.publish({
                "connectionStatus": "waiting",
                "phase": "Offline",
                "statusMessage": f"Simulator Web UDP {sim_telemetry_port} unavailable: {exc}",
                "server": {"generatedAt": time.time(), "source": "ShuttleSim observer", "simulation": True},
            })
    try:
        while not stop.is_set():
            now = time.monotonic()
            if now - last_rl_poll >= RL_STATUS_POLL_SECONDS:
                rl_status = rl_monitor.poll()
                last_rl_poll = now
            if sock is not None:
                while True:
                    try:
                        payload, _ = sock.recvfrom(65535)
                    except BlockingIOError:
                        break
                    except OSError:
                        break
                    try:
                        packet = json.loads(payload.decode("utf-8"))
                    except (UnicodeDecodeError, json.JSONDecodeError):
                        continue
                    if not isinstance(packet, dict) or packet.get("source") != "sim":
                        continue
                    last_packet = packet
                    last_packet_wall = time.monotonic()
                    pos = packet.get("position") if isinstance(packet.get("position"), dict) else {}
                    sim_time = optional_number(packet.get("sim_time"))
                    lat = optional_number(pos.get("lat_deg"))
                    lon = optional_number(pos.get("lon_deg"))
                    alt = optional_number(pos.get("altitude_m"))
                    if (
                        sim_time is not None and lat is not None and lon is not None and alt is not None
                        and (sim_time - last_trail_time >= 1.0 or not trail)
                    ):
                        trail.append({
                            "ut": optional_number(packet.get("ut")),
                            "latitude": lat,
                            "longitude": lon,
                            "altitude": alt,
                            "phase": "SIM",
                        })
                        last_trail_time = sim_time
                        if len(trail) > 900:
                            trail = trail[::2]
            mirror = read_controller_mirror(root, max_age=None)
            run = read_simulator_run(root)
            packet_recent = last_packet is not None and time.monotonic() - last_packet_wall < 3.0
            if last_packet is not None:
                snapshot = merge_simulator_packet(mirror, last_packet, run, trail)
                snapshot["rlTraining"] = rl_status
                if not packet_recent and (run or {}).get("state") == "running":
                    snapshot["connectionStatus"] = "reconnecting"
                    snapshot["statusMessage"] = "ShuttleSim telemetry paused; retaining last frame"
            elif mirror is None:
                snapshot = {
                    "connectionStatus": "waiting",
                    "phase": "Offline",
                    "statusMessage": f"Waiting for ShuttleSim UDP {sim_telemetry_port} / Guidance snapshot",
                    "telemetry": {}, "command": {}, "guidanceState": {},
                    "actualTrajectory": [], "predictedTrajectory": [], "plannedTrajectory": [],
                    "projectedTAEMTrajectory": [], "referenceTrajectory": [], "orbitalTrajectory": [],
                    "rlTraining": rl_status,
                    "server": {"generatedAt": time.time(), "source": "ShuttleSim observer", "simulation": True},
                }
                snapshot["simulation"] = simulation_status(run, snapshot, "simulator")
            else:
                snapshot = copy.deepcopy(mirror)
                snapshot["rlTraining"] = rl_status
                snapshot["simulation"] = simulation_status(run, snapshot, "simulator")
                snapshot["server"] = {
                    **(snapshot.get("server") if isinstance(snapshot.get("server"), dict) else {}),
                    "generatedAt": time.time(),
                    "source": "ShuttleSim Guidance mirror",
                    "simulation": True,
                }
            sim = snapshot.get("simulation") if isinstance(snapshot.get("simulation"), dict) else {}
            signature = (
                snapshot.get("tickSequence"),
                snapshot.get("phase"),
                sim.get("state"),
                sim.get("runId"),
                sim.get("simTime"),
                sim.get("wallSeconds"),
            )
            if signature != last_signature:
                store.publish(snapshot)
                last_signature = signature
            stop.wait(0.025)
    finally:
        if sock is not None:
            sock.close()


def load_replay_frames(path: Path) -> list[dict[str, Any]]:
    frames: list[dict[str, Any]] = []
    try:
        with path.open("r", encoding="utf-8") as handle:
            for line in handle:
                try:
                    value = json.loads(line)
                except json.JSONDecodeError:
                    continue
                if not isinstance(value, dict):
                    continue
                if value.get("source") == "sim" and value.get("type") == "telemetry":
                    continue
                snapshot = value.get("snapshot") if isinstance(value.get("snapshot"), dict) else value
                if isinstance(snapshot, dict):
                    frames.append(snapshot)
    except OSError:
        return []
    return frames


def replay_collector_loop(store: "SnapshotStore", root: Path, stop: threading.Event,
                          replay_path: Path, replay_speed: float, replay_loop: bool,
                          fallback_configuration: dict[str, Any] | None = None,
                          rl_output: Path | None = None) -> None:
    run: dict[str, Any] | None = None
    manifest_path = replay_path.parent / "manifest.json"
    try:
        candidate_run = json.loads(manifest_path.read_text(encoding="utf-8"))
        if isinstance(candidate_run, dict):
            run = candidate_run
    except (OSError, json.JSONDecodeError):
        pass
    if run is None:
        run = read_simulator_run(root) or {
            "runId": replay_path.parent.name,
            "state": "replay",
            "mode": "replay",
            "guidanceSnapshots": str(replay_path),
        }
    configuration = _replay_configuration(root, run, fallback_configuration)
    guidance_frames = load_replay_frames(replay_path)
    simulator_frames = load_simulator_replay_frames(replay_path.parent, run, 2000)
    frames = hydrate_archive_replay_frames(guidance_frames, simulator_frames, configuration)
    if not frames:
        store.publish({
            "connectionStatus": "failed",
            "phase": "Offline",
            "statusMessage": f"Replay has no readable frames: {replay_path}",
            "simulation": {"active": True, "sourceMode": "replay", "state": "error",
                           "mode": "replay", "replayCount": 0},
            "server": {"generatedAt": time.time(), "source": "ShuttleSim replay", "simulation": True},
        })
        return
    speed = max(0.05, replay_speed)
    rl_monitor = RLTrainingMonitor(root, rl_output)
    base_ut = optional_number((run.get("prerollFinal") or {}).get("ut")) if isinstance(run.get("prerollFinal"), dict) else None

    while not stop.is_set():
        previous_time: float | None = None
        for index, raw in enumerate(frames):
            if stop.is_set():
                return
            snapshot = copy.deepcopy(raw)
            snapshot["rlTraining"] = rl_monitor.poll()
            telemetry = snapshot.get("telemetry") if isinstance(snapshot.get("telemetry"), dict) else {}
            ut = optional_number(telemetry.get("ut"))
            frame_time = max(0.0, ut - base_ut) if ut is not None and base_ut is not None else ut
            if previous_time is not None and frame_time is not None:
                stop.wait(min(1.0, max(0.0, frame_time - previous_time) / speed))
            previous_time = frame_time if frame_time is not None else previous_time
            snapshot["simulation"] = simulation_status(
                {**run, "state": "replay"}, snapshot, "replay",
                replay_index=index, replay_count=len(frames), replay_speed=speed,
            )
            snapshot["server"] = {
                **(snapshot.get("server") if isinstance(snapshot.get("server"), dict) else {}),
                "generatedAt": time.time(),
                "source": "ShuttleSim replay",
                "simulation": True,
            }
            store.publish(snapshot)
        if not replay_loop:
            final = copy.deepcopy(frames[-1])
            final["rlTraining"] = rl_monitor.poll()
            final["simulation"] = simulation_status(
                {**run, "state": "replay-finished"}, final, "replay",
                replay_index=len(frames) - 1, replay_count=len(frames), replay_speed=speed,
            )
            final["server"] = {"generatedAt": time.time(), "source": "ShuttleSim replay", "simulation": True}
            store.publish(final)
            return
            return


def guidance_state(current: dict[str, Any], terminal: dict[str, Any], ut: float) -> dict[str, Any]:
    reversal_ut = optional_number(current.get("plannedReversalUT"))
    planned_ut = number(current.get("plannedUT"), ut)
    duration = max(0.0, number(current.get("segmentDuration")))
    return {
        "entryPlanValid": bool(current.get("valid")),
        "entryPlanTerminalReady": bool(current.get("terminalReady")),
        "entryPlanTargetBank": current.get("targetBank"),
        "entryPlanTargetAoA": current.get("targetAoA"),
        "entryPlanTargetHeading": current.get("targetHeading"),
        "entryPlanSegmentRemaining": max(0.0, planned_ut + duration - ut) if current else None,
        "entryPlanCost": current.get("plannerCost"),
        "entryPlanTAEMRangeError": current.get("taemRangeError"),
        "entryPlanTAEMSpeed": current.get("taemSpeed"),
        "entryPlanTAEMEnergyError": current.get("taemEnergyError"),
        "entryPlanTAEMCaptureReady": current.get("taemCaptureReady"),
        "entryPlanTAEMCaptureVeto": current.get("taemCaptureVeto"),
        "entryPlanTAEMCaptureVetoReasons": taem_capture_veto_reasons(current.get("taemCaptureVeto")),
        "entryPlanPredictedReversals": current.get("predictedReversals"),
        "entryReversalScheduled": bool(current.get("hasPlannedReversal")),
        "entryReversalIsFinal": bool(current.get("plannedReversalFinal")),
        "entryReversalTimeRemaining": max(0.0, reversal_ut - ut) if reversal_ut is not None else None,
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
        "hacCircuitCount": terminal.get("hacCircuitCount"),
    }


def collector_loop(store: SnapshotStore, root: Path, configuration: dict[str, Any], stop: threading.Event,
                   rl_output: Path | None = None) -> None:
    follower = PlannerFollower(root)
    rl_monitor = RLTrainingMonitor(root, rl_output)
    rl_status = rl_monitor.poll()
    last_rl_poll = 0.0
    actual: list[dict[str, float]] = load_actual_trajectory_history(root)
    connection: Any = None
    last_actual_sample = 0.0
    last_actual_persist = 0.0
    actual_backfill_checked = False
    orbital_trajectory: list[dict[str, Any]] = []
    orbital_trajectory_meta: dict[str, Any] = {}
    last_orbit_sample = 0.0
    last_orbital_speed: float | None = None
    last_orbital_speed_ut: float | None = None
    orbital_speed_rate = 0.0
    retry_delay = 0.5
    last_error = ""

    while not stop.is_set():
        now = time.monotonic()
        if now - last_rl_poll >= RL_STATUS_POLL_SECONDS:
            rl_status = rl_monitor.poll()
            last_rl_poll = now
        if connection is None:
            store.publish({
                **store.current()[1],
                "rlTraining": rl_status,
                "connectionStatus": "connecting",
                "statusMessage": "Connecting to KSP",
                "lastError": last_error,
                "server": {"generatedAt": time.time(), "source": "kRPC observer", "reconnectDelay": retry_delay},
            })
            try:
                connection = krpc.connect(name="KSP Shuttle Lander Telemetry Web")
                retry_delay = 0.5
                last_error = ""
            except Exception as exc:
                last_error = str(exc)
                store.publish({
                    **store.current()[1],
                    "connectionStatus": "reconnecting",
                    "statusMessage": "KSP unavailable; retrying",
                    "lastError": last_error,
                    "server": {"generatedAt": time.time(), "source": "kRPC observer", "reconnectDelay": retry_delay},
                })
                stop.wait(retry_delay)
                retry_delay = min(5.0, retry_delay * 1.6)
                continue

        try:
            follower.poll()
            vessel = connection.space_center.active_vessel
            if vessel is None:
                raise RuntimeError("No active vessel")
            # kRPC's default vessel Flight uses the vessel-local surface frame. It is
            # the right frame for pitch/roll/heading, but its origin follows the craft,
            # so speed components read as zero. Use Kerbin's rotating body frame for
            # surface-relative velocity while keeping attitude in the local frame.
            flight = vessel.flight()
            body = vessel.orbit.body
            motion = vessel.flight(body.reference_frame)
            ut = number(connection.space_center.ut)
            latitude = number(flight.latitude)
            longitude = number(flight.longitude)
            altitude = number(flight.mean_altitude)
            radar_altitude = max(0.0, number(flight.surface_altitude))
            horizontal_speed = max(0.0, number(motion.horizontal_speed))
            vertical_speed = number(motion.vertical_speed)
            fpa = math.degrees(math.atan2(vertical_speed, max(1e-6, horizontal_speed)))
            pitch = number(flight.pitch)
            roll = number(flight.roll)
            heading = number(flight.heading) % 360.0
            # kRPC exposes angle_of_attack and sideslip_angle in degrees.
            aoa = number(flight.angle_of_attack)
            true_air_speed = max(0.0, number(flight.true_air_speed))

            # Orbital-mode telemetry is intentionally measured in Kerbin's
            # non-rotating frame. The normal surface Flight frame under-reports
            # orbital speed by the body's rotation and cannot drive a correct
            # prograde/retrograde navball.
            orbital_speed: float | None = None
            orbital_heading: float | None = None
            orbital_fpa: float | None = None
            inertial_frame = read_optional(body, "non_rotating_reference_frame")
            if inertial_frame is not None:
                try:
                    position_i = tuple(float(x) for x in vessel.position(inertial_frame))
                    velocity_i = tuple(float(x) for x in vessel.velocity(inertial_frame))
                    radius_i = math.sqrt(sum(x*x for x in position_i))
                    orbital_speed = math.sqrt(sum(x*x for x in velocity_i))
                    if radius_i > 1.0 and orbital_speed > 1e-6:
                        up_i = tuple(x / radius_i for x in position_i)
                        east_raw = (-up_i[2], 0.0, up_i[0])
                        east_norm = math.sqrt(sum(x*x for x in east_raw))
                        if east_norm > 1e-8:
                            east_i = tuple(x / east_norm for x in east_raw)
                            north_i = (
                                east_i[1]*up_i[2] - east_i[2]*up_i[1],
                                east_i[2]*up_i[0] - east_i[0]*up_i[2],
                                east_i[0]*up_i[1] - east_i[1]*up_i[0],
                            )
                            ve = sum(velocity_i[i]*east_i[i] for i in range(3))
                            vn = sum(velocity_i[i]*north_i[i] for i in range(3))
                            vu = sum(velocity_i[i]*up_i[i] for i in range(3))
                            orbital_heading = math.degrees(math.atan2(ve, vn)) % 360.0
                            orbital_fpa = math.degrees(math.atan2(vu, max(1e-9, math.hypot(ve, vn))))
                except Exception:
                    orbital_speed = orbital_heading = orbital_fpa = None

            # Resolve the physical subsolar point in Kerbin's rotating frame. The
            # browser can then use the same latitude/longitude basis as the planet
            # mesh, avoiding any kRPC/Unity axis-order guess for lighting.
            sun_latitude: float | None = None
            sun_longitude: float | None = None
            try:
                sun = connection.space_center.bodies.get("Sun")
                if sun is not None:
                    sun_position = sun.position(body.reference_frame)
                    sun_latitude = optional_number(body.latitude_at_position(sun_position, body.reference_frame))
                    sun_longitude = optional_number(body.longitude_at_position(sun_position, body.reference_frame))
            except Exception:
                pass

            now = time.monotonic()
            if now - last_actual_sample >= 0.45:
                point = {"latitude": latitude, "longitude": longitude, "altitude": altitude, "ut": ut}
                if not actual_backfill_checked:
                    logged = load_actual_trajectory_from_vehicle_log(root, point)
                    if logged:
                        trailing = [item for item in actual if item["ut"] > logged[-1]["ut"] + 1e-6]
                        actual = prune_actual_trajectory(logged + trailing, ut)
                    actual_backfill_checked = True
                if actual and not actual_trajectory_can_resume(actual, point):
                    actual = []
                actual.append(point)
                actual = prune_actual_trajectory(actual, ut)
                last_actual_sample = now
                if now - last_actual_persist >= 2.0:
                    save_actual_trajectory_history(root, actual)
                    last_actual_persist = now
            ground_track = heading
            if len(actual) >= 2:
                a = actual[-2]
                b = actual[-1]
                if abs(a["latitude"] - b["latitude"]) + abs(a["longitude"] - b["longitude"]) > 1e-7:
                    ground_track = bearing_degrees(a, b)

            record = follower.latest if follower.fresh else {}
            current = record.get("currentPlan") if isinstance(record.get("currentPlan"), dict) else {}
            terminal = record.get("terminalGeometry") if isinstance(record.get("terminalGeometry"), dict) else {}
            phase = str(record.get("phase") or "OBSERVE").upper()
            has_current_command = isinstance(current, dict) and current.get("valid") is True
            has_display_plan = _entry_plan_displayable(current)
            target_aoa = optional_number(current.get("targetAoA")) if has_current_command else None
            target_pitch = fpa + target_aoa if target_aoa is not None else None
            target_roll = optional_number(current.get("targetBank")) if has_current_command else None
            target_heading = optional_number(current.get("targetHeading")) if has_current_command else None
            if target_heading is not None:
                target_heading %= 360.0

            rails_warp_factor = int(number(read_optional(connection.space_center, "rails_warp_factor", 0)))
            physics_warp_factor = int(number(read_optional(connection.space_center, "physics_warp_factor", 0)))
            rails_rates = (1.0, 5.0, 10.0, 50.0, 100.0, 1000.0, 10000.0, 100000.0)
            physics_rates = (1.0, 2.0, 3.0, 4.0)
            if rails_warp_factor > 0:
                time_warp_rate = rails_rates[min(rails_warp_factor, len(rails_rates) - 1)]
                time_warp_mode = "rails"
            elif physics_warp_factor > 0:
                time_warp_rate = physics_rates[min(physics_warp_factor, len(physics_rates) - 1)]
                time_warp_mode = "physics"
            else:
                time_warp_rate = 1.0
                time_warp_mode = "none"

            if orbital_speed is not None:
                if last_orbital_speed is not None and last_orbital_speed_ut is not None:
                    speed_dt = ut - last_orbital_speed_ut
                    if 0.02 <= speed_dt <= 2.0:
                        raw_rate = (orbital_speed - last_orbital_speed) / speed_dt
                        orbital_speed_rate = orbital_speed_rate * 0.68 + raw_rate * 0.32
                    elif speed_dt > 2.0 or speed_dt < -0.01:
                        orbital_speed_rate = 0.0
                last_orbital_speed = orbital_speed
                last_orbital_speed_ut = ut

            telemetry = {
                "ut": ut,
                "vesselName": str(read_optional(vessel, "name", "—")),
                "latitude": latitude,
                "longitude": longitude,
                "meanAltitude": altitude,
                "radarAltitude": radar_altitude,
                "verticalSpeed": vertical_speed,
                "horizontalSpeed": horizontal_speed,
                "trueAirSpeed": true_air_speed,
                "mach": optional_number(read_optional(flight, "mach")),
                "dynamicPressure": optional_number(read_optional(flight, "dynamic_pressure")),
                "gForce": optional_number(read_optional(flight, "g_force")),
                "pitch": pitch,
                "roll": roll,
                "heading": heading,
                "groundTrackHeading": ground_track,
                "sunLatitude": sun_latitude,
                "sunLongitude": sun_longitude,
                "flightPathAngle": fpa,
                "angleOfAttack": aoa,
                "sideslipAngle": number(read_optional(flight, "sideslip_angle", 0.0)),
                "mass": optional_number(read_optional(vessel, "mass")),
                "orbitalSpeed": orbital_speed,
                "orbitalHeading": orbital_heading,
                "orbitalFlightPathAngle": orbital_fpa,
                "orbitalSpeedRate": orbital_speed_rate if orbital_speed is not None else None,
                "deceleration": max(0.0, -orbital_speed_rate) if orbital_speed is not None else None,
                "railsWarpFactor": rails_warp_factor,
                "physicsWarpFactor": physics_warp_factor,
                "timeWarpRate": time_warp_rate,
                "timeWarpMode": time_warp_mode,
                "predictedTAEMRangeError": current.get("taemRangeError"),
                "predictedTAEMEnergyError": current.get("taemEnergyError"),
            }
            orbit = read_optional(vessel, "orbit")
            thrust = max(0.0, number(read_optional(vessel, "thrust", 0.0)))
            available_thrust = max(0.0, number(read_optional(vessel, "available_thrust", 0.0)))
            mass = max(0.0, number(read_optional(vessel, "mass", 0.0)))
            control = read_optional(vessel, "control")
            throttle = optional_number(read_optional(control, "throttle")) if control is not None else None
            telemetry["thrust"] = thrust
            telemetry["currentThrust"] = thrust
            telemetry["availableThrust"] = available_thrust
            telemetry["throttle"] = throttle
            telemetry["specificImpulse"] = optional_number(read_optional(vessel, "specific_impulse"))
            telemetry["propulsiveAcceleration"] = thrust / mass if mass > 1.0 else None
            if orbit is not None:
                telemetry["apoapsisAltitude"] = optional_number(read_optional(orbit, "apoapsis_altitude"))
                telemetry["periapsisAltitude"] = optional_number(read_optional(orbit, "periapsis_altitude"))
                telemetry["orbitPeriod"] = optional_number(read_optional(orbit, "period"))
                telemetry["timeToApoapsis"] = optional_number(read_optional(orbit, "time_to_apoapsis"))
                telemetry["timeToPeriapsis"] = optional_number(read_optional(orbit, "time_to_periapsis"))
                telemetry["eccentricity"] = optional_number(read_optional(orbit, "eccentricity"))
                inclination = optional_number(read_optional(orbit, "inclination"))
                telemetry["inclination"] = math.degrees(inclination) if inclination is not None else None

            atmosphere_depth = max(0.0, number(read_optional(body, "atmosphere_depth", 0.0)))
            orbit_eligible = (
                orbit is not None
                and altitude > atmosphere_depth
            )
            if not orbit_eligible:
                orbital_trajectory = []
                orbital_trajectory_meta = {}
            elif now - last_orbit_sample >= ORBIT_TRAJECTORY_REFRESH_SECONDS or not orbital_trajectory:
                orbital_trajectory, orbital_trajectory_meta = build_orbital_trajectory(
                    orbit,
                    body,
                    ut,
                    {"latitude": latitude, "longitude": longitude, "altitude": altitude},
                    thrust,
                )
                last_orbit_sample = now

            snapshot = {
                "connectionStatus": "connected",
                "phase": phase,
                "statusMessage": (
                    "Live guidance plan" if follower.fresh and has_display_plan
                    else "Live guidance command; terminal path unproven" if follower.fresh and has_current_command
                    else "Guidance log active; no current plan" if follower.fresh
                    else "Flight live; waiting for fresh guidance plan"
                ),
                "warningMessage": record.get("warningMessage") if isinstance(record, dict) else None,
                "lastError": "",
                # Guidance may legitimately execute a short current-leg command while
                # the end-to-end landing path is still unproven. Keep command activity
                # separate from PLAN visibility.
                "automationEngaged": has_current_command,
                "plannerFresh": follower.fresh,
                "plannerLog": follower.path.name if follower.path else None,
                "trajectoryRevision": follower.trajectory_revision if follower.fresh else 0,
                "trajectoryLineage": record.get("lineage") if isinstance(record.get("lineage"), dict) else {},
                "telemetry": telemetry,
                "command": {
                    "autopilotEngaged": has_current_command,
                    "targetPitch": target_pitch,
                    "targetAoA": target_aoa,
                    "targetRoll": target_roll,
                    "targetHeading": target_heading,
                },
                "guidanceState": guidance_state(current, terminal, ut),
                "predictedTrajectory": follower.prediction if follower.fresh else [],
                "projectedTAEMTrajectory": (
                    projected_taem_trajectory(follower.prediction, terminal, configuration, guidance_state(current, terminal, ut))
                    if follower.fresh and _entry_plan_displayable(current) and not _terminal_phase(phase)
                    else []
                ),
                "plannedTrajectory": follower.planned if follower.fresh else [],
                "referenceTrajectory": [],
                "deorbitPlan": None,
                "actualTrajectory": actual,
                "orbitalTrajectory": orbital_trajectory,
                "orbitalTrajectoryMeta": orbital_trajectory_meta,
                "site": configuration.get("site", {}),
                "rlTraining": rl_status,
                "server": {"generatedAt": time.time(), "source": "kRPC observer", "reconnectDelay": 0.0},
            }
            exact = read_controller_mirror(root)
            if exact is not None:
                exact_telemetry = exact.get("telemetry")
                if isinstance(exact_telemetry, dict):
                    # Keep observer-only convenience fields such as vesselName if the
                    # backend schema does not carry them, but let the controller's
                    # telemetry and command-error fields win.
                    snapshot["telemetry"] = {**telemetry, **exact_telemetry}
                exact_command = exact.get("command")
                if isinstance(exact_command, dict) and exact_command:
                    snapshot["command"] = exact_command
                exact_guidance = exact.get("guidanceState")
                if isinstance(exact_guidance, dict) and exact_guidance:
                    snapshot["guidanceState"] = exact_guidance
                exact_plan = exact.get("plan")
                snapshot["deorbitPlan"] = copy.deepcopy(exact_plan) if isinstance(exact_plan, dict) else None
                apply_exact_trajectory_geometry(snapshot, exact, terminal, configuration)
                for key in ("tickSequence", "connectionStatus", "phase", "statusMessage", "warningMessage", "automationEngaged"):
                    if exact.get(key) is not None:
                        snapshot[key] = exact[key]
                snapshot["server"] = {
                    "generatedAt": time.time(),
                    "source": "exact-controller-snapshot",
                    "reconnectDelay": 0.0,
                }
            store.publish(snapshot)
            retry_delay = 0.5
            stop.wait(0.12)
        except Exception as exc:
            last_error = str(exc)
            try:
                if connection is not None:
                    connection.close()
            except Exception:
                pass
            connection = None
            orbital_trajectory = []
            orbital_trajectory_meta = {}
            last_orbit_sample = 0.0
            last_orbital_speed = None
            last_orbital_speed_ut = None
            orbital_speed_rate = 0.0
            store.publish({
                **store.current()[1],
                "connectionStatus": "reconnecting",
                "statusMessage": "Telemetry source lost; reconnecting",
                "lastError": last_error,
                "server": {"generatedAt": time.time(), "source": "kRPC observer", "reconnectDelay": retry_delay},
            })
            stop.wait(retry_delay)
            retry_delay = min(5.0, retry_delay * 1.6)

    try:
        if connection is not None:
            connection.close()
    except Exception:
        pass



def kerbin_texture_candidates() -> list[Path]:
    candidates: list[Path] = []
    configured = os.environ.get("KSP_LANDER_KERBIN_TEXTURE")
    if configured:
        candidates.append(Path(configured).expanduser())
    # Stock KSP scaled-space diffuse (_MainTex from NewKerbinScaledSpace),
    # extracted from sharedassets2.assets and compressed for the web viewer.
    candidates.append(STSN_MODEL_ROOT / "kerbin-stock.webp")
    return candidates


def kerbin_texture() -> Path | None:
    return next((path for path in kerbin_texture_candidates() if path.is_file()), None)


class TelemetryHTTPServer(ThreadingHTTPServer):
    daemon_threads = True
    allow_reuse_address = True

    def __init__(self, address: tuple[str, int], store: SnapshotStore) -> None:
        super().__init__(address, TelemetryHandler)
        self.store = store
        self.web_root = WEB_ROOT
        self.texture = kerbin_texture()
        self.model_root = STSN_MODEL_ROOT
        self.rl_monitor: RLTrainingMonitor | None = None


class TelemetryHandler(BaseHTTPRequestHandler):
    server: TelemetryHTTPServer
    protocol_version = "HTTP/1.1"

    def log_message(self, fmt: str, *args: Any) -> None:
        if os.environ.get("KSP_LANDER_WEB_ACCESS_LOG") == "1":
            super().log_message(fmt, *args)

    def send_bytes(self, body: bytes, content_type: str, status: int = 200, cache: str = "no-store") -> None:
        self.send_response(status)
        self.send_header("Content-Type", content_type)
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", cache)
        self.send_header("Connection", "keep-alive")
        self.end_headers()
        self.wfile.write(body)

    def send_json(self, value: Any, status: int = 200) -> None:
        body = json.dumps(value, separators=(",", ":"), allow_nan=False).encode("utf-8")
        self.send_bytes(body, "application/json; charset=utf-8", status)

    def do_GET(self) -> None:  # noqa: N802
        parsed = urlparse(self.path)
        path = parsed.path
        query = parse_qs(parsed.query)
        if path == "/api/sim-runs":
            self.send_json({"runs": list_simulation_runs(ROOT)})
            return
        if path == "/api/sim-replay":
            run_id = (query.get("id") or [""])[0]
            try:
                max_frames = int((query.get("maxFrames") or ["600"])[0])
            except ValueError:
                max_frames = 600
            replay = load_simulation_replay(ROOT, run_id, max_frames)
            if replay is None:
                self.send_json({"error": "simulation run not found"}, HTTPStatus.NOT_FOUND)
            else:
                self.send_json(replay)
            return
        if path == "/events":
            self.handle_events()
            return
        if path == "/api/snapshot":
            self.send_json(self.server.store.current()[1])
            return
        if path == "/api/rl-training":
            if self.server.rl_monitor is not None:
                self.send_json(self.server.rl_monitor.poll())
            else:
                snapshot = self.server.store.current()[1]
                status = snapshot.get("rlTraining") if isinstance(snapshot.get("rlTraining"), dict) else empty_rl_training_status()
                self.send_json(status)
            return
        if path == "/api/health":
            version, snapshot = self.server.store.current()
            self.send_json({
                "ok": True,
                "version": version,
                "connectionStatus": snapshot.get("connectionStatus"),
                "plannerFresh": snapshot.get("plannerFresh", False),
                "kerbinTexture": str(self.server.texture) if self.server.texture else None,
            })
            return
        if path == "/asset/kerbin-map":
            texture = self.server.texture
            if texture is None or not texture.is_file():
                self.send_json({"error": "Kerbin texture not found"}, HTTPStatus.NOT_FOUND)
                return
            try:
                data = texture.read_bytes()
            except OSError as exc:
                self.send_json({"error": str(exc)}, HTTPStatus.INTERNAL_SERVER_ERROR)
                return
            mime = mimetypes.guess_type(texture.name)[0] or "image/png"
            self.send_bytes(data, mime, cache="public, max-age=3600")
            return

        if path in {"/asset/sts-n-model.json", "/asset/sts-n-model.bin"} or path.startswith("/asset/sts-n-textures/"):
            relative = path.removeprefix("/asset/")
            root = self.server.model_root.resolve()
            asset = (root / relative).resolve()
            if not asset.is_relative_to(root):
                self.send_json({"error": "invalid asset path"}, HTTPStatus.BAD_REQUEST)
                return
            try:
                data = asset.read_bytes()
            except OSError as exc:
                self.send_json({"error": f"STS-N asset unavailable: {exc}"}, HTTPStatus.NOT_FOUND)
                return
            mime = mimetypes.guess_type(asset.name)[0] or "application/octet-stream"
            if asset.suffix == ".json":
                mime = "application/json; charset=utf-8"
            self.send_bytes(data, mime, cache="no-store" if asset.suffix == ".json" else "public, max-age=3600")
            return

        static_map = {
            "/": "index.html",
            "/index.html": "index.html",
            "/app.js": "app.js",
            "/scene3d.js": "scene3d.js",
            "/styles.css": "styles.css",
        }
        name = static_map.get(path)
        if name is None:
            self.send_json({"error": "not found"}, HTTPStatus.NOT_FOUND)
            return
        file_path = self.server.web_root / name
        try:
            data = file_path.read_bytes()
        except OSError:
            self.send_json({"error": f"missing static asset: {name}"}, HTTPStatus.NOT_FOUND)
            return
        mime = mimetypes.guess_type(file_path.name)[0] or "application/octet-stream"
        if mime.startswith("text/") or mime in {"application/javascript", "application/json"}:
            mime += "; charset=utf-8"
        self.send_bytes(data, mime)

    def handle_events(self) -> None:
        self.send_response(HTTPStatus.OK)
        self.send_header("Content-Type", "text/event-stream; charset=utf-8")
        self.send_header("Cache-Control", "no-cache, no-transform")
        self.send_header("Connection", "keep-alive")
        self.send_header("X-Accel-Buffering", "no")
        self.end_headers()
        version = -1
        try:
            while True:
                next_version, snapshot = self.server.store.wait_after(version, 10.0)
                if next_version != version:
                    payload = json.dumps(snapshot, separators=(",", ":"), allow_nan=False)
                    self.wfile.write(f"event: snapshot\ndata: {payload}\n\n".encode("utf-8"))
                    version = next_version
                else:
                    self.wfile.write(b": keepalive\n\n")
                self.wfile.flush()
        except (BrokenPipeError, ConnectionResetError, ConnectionAbortedError, OSError):
            return


def load_configuration(path: Path) -> dict[str, Any]:
    try:
        with path.open("r", encoding="utf-8") as handle:
            value = json.load(handle)
        return value if isinstance(value, dict) else {}
    except (OSError, json.JSONDecodeError):
        return {}


def main() -> int:
    parser = argparse.ArgumentParser(description="KSP Shuttle Lander telemetry web server")
    parser.add_argument("--host", default=os.environ.get("KSP_LANDER_WEB_HOST", "127.0.0.1"))
    parser.add_argument("--port", type=int, default=int(os.environ.get("KSP_LANDER_WEB_PORT", "8790")))
    parser.add_argument("--config", type=Path, default=DEFAULT_CONFIG)
    parser.add_argument("--mode", choices=("live", "simulator", "replay"), default=os.environ.get("KSP_LANDER_WEB_MODE", "live"))
    parser.add_argument("--replay", type=Path, default=None,
                        help="Archived guidance or simulator JSONL for replay mode; defaults to simulator-run.json")
    parser.add_argument("--replay-speed", type=float, default=20.0)
    parser.add_argument("--replay-loop", action="store_true")
    parser.add_argument("--rl-output", type=Path, default=None,
                        help="RL results directory to monitor; defaults to newest ShuttleSim/rl/results run")
    parser.add_argument("--sim-telemetry-port", type=int,
                        default=int(os.environ.get("KSP_LANDER_SIM_WEB_TELEMETRY_PORT", "8797")))
    args = parser.parse_args()

    configuration = load_configuration(args.config.expanduser())
    store = SnapshotStore()
    stop = threading.Event()
    if args.mode == "live":
        if krpc is None:
            raise SystemExit("Python kRPC is required in live mode. Use --mode simulator or --mode replay without KSP.")
        target = collector_loop
        collector_args = (store, ROOT, configuration, stop, args.rl_output)
    elif args.mode == "simulator":
        target = simulator_collector_loop
        collector_args = (store, ROOT, stop, args.sim_telemetry_port, args.rl_output)
    else:
        replay_path = args.replay
        if replay_path is None:
            run = read_simulator_run(ROOT)
            candidate = None
            if isinstance(run, dict):
                candidate = run.get("guidanceSnapshots") or run.get("simulatorTelemetry")
            replay_path = Path(candidate) if candidate else None
        if replay_path is None:
            raise SystemExit("Replay mode needs --replay FILE or simulator-run.json with archived telemetry.")
        target = replay_collector_loop
        collector_args = (
            store, ROOT, stop, replay_path.expanduser(), args.replay_speed,
            args.replay_loop, configuration, args.rl_output,
        )
    collector = threading.Thread(
        target=target,
        args=collector_args,
        name=f"telemetry-{args.mode}-collector",
        daemon=True,
    )
    collector.start()

    server = TelemetryHTTPServer((args.host, args.port), store)
    server.rl_monitor = RLTrainingMonitor(ROOT, args.rl_output)
    texture = str(server.texture) if server.texture else "not found (map falls back to vector grid)"
    print(f"telemetry web: http://{args.host}:{args.port}  mode={args.mode}  kerbin={texture}", flush=True)
    try:
        server.serve_forever(poll_interval=0.25)
    except KeyboardInterrupt:
        pass
    finally:
        stop.set()
        server.shutdown()
        server.server_close()
        collector.join(timeout=2.0)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
