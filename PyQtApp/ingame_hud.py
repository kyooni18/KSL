from __future__ import annotations

import math
import os
import sys
import threading
import time
from typing import Any, Iterable, Mapping, Sequence


Vec3 = tuple[float, float, float]
GeoPoint = tuple[float, float, float]

# High-contrast phosphor-green palette for the in-game AR HUD. KSP's
# world-space Drawing API does not provide a dependable outline/alpha pass,
# so visibility comes from saturated green hues and deliberately heavier cues.
_HUD_GREEN_PRIMARY = (0.10, 1.0, 0.24)
_HUD_GREEN_SECONDARY = (0.08, 0.82, 0.18)
_HUD_GREEN_DIM = (0.06, 0.64, 0.14)
_HUD_WARNING = (1.0, 0.38, 0.10)


def _finite(value: Any, default: float = 0.0) -> float:
    try:
        number = float(value)
    except (TypeError, ValueError):
        return default
    return number if math.isfinite(number) else default


def _clamp(value: float, lower: float, upper: float) -> float:
    return max(lower, min(upper, value))




def _add(a: Vec3, b: Vec3) -> Vec3:
    return (a[0] + b[0], a[1] + b[1], a[2] + b[2])


def _mul(v: Vec3, scale: float) -> Vec3:
    return (v[0] * scale, v[1] * scale, v[2] * scale)



def _dot(a: Vec3, b: Vec3) -> float:
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]

def _cross(a: Vec3, b: Vec3) -> Vec3:
    return (
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    )


def _length(v: Vec3) -> float:
    return math.sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2])


def _unit(value: Iterable[Any]) -> Vec3:
    values = list(value)
    if len(values) < 3:
        return (0.0, 0.0, 0.0)
    vector = (_finite(values[0]), _finite(values[1]), _finite(values[2]))
    magnitude = _length(vector)
    if magnitude <= 1e-9:
        return (0.0, 0.0, 0.0)
    return _mul(vector, 1.0 / magnitude)


def _mix3(a: Vec3, sa: float, b: Vec3, sb: float, c: Vec3, sc: float) -> Vec3:
    return (
        a[0] * sa + b[0] * sb + c[0] * sc,
        a[1] * sa + b[1] * sb + c[1] * sc,
        a[2] * sa + b[2] * sb + c[2] * sc,
    )


def _local_basis(latitude_deg: float, longitude_deg: float, prime: Vec3, quarter: Vec3, pole: Vec3) -> tuple[Vec3, Vec3, Vec3]:
    latitude = math.radians(latitude_deg)
    longitude = math.radians(longitude_deg)
    cos_lat = math.cos(latitude)
    sin_lat = math.sin(latitude)
    cos_lon = math.cos(longitude)
    sin_lon = math.sin(longitude)

    up = _unit(_mix3(prime, cos_lat * cos_lon, quarter, cos_lat * sin_lon, pole, sin_lat))
    east = _unit(_add(_mul(prime, -sin_lon), _mul(quarter, cos_lon)))
    north = _unit(
        _mix3(
            prime,
            -sin_lat * cos_lon,
            quarter,
            -sin_lat * sin_lon,
            pole,
            cos_lat,
        )
    )
    return up, east, north


def _geo_position(point: GeoPoint, radius: float, prime: Vec3, quarter: Vec3, pole: Vec3, lift: float = 18.0) -> Vec3:
    latitude, longitude, altitude = point
    up, _, _ = _local_basis(latitude, longitude, prime, quarter, pole)
    return _mul(up, max(1.0, radius + altitude + lift))


def _geo_points(value: Any) -> list[GeoPoint]:
    if not isinstance(value, list):
        return []
    points: list[GeoPoint] = []
    for item in value:
        if isinstance(item, Mapping):
            latitude = item.get("latitude")
            longitude = item.get("longitude")
            altitude = item.get("altitude")
        elif isinstance(item, Sequence) and not isinstance(item, (str, bytes)) and len(item) >= 3:
            latitude, longitude, altitude = item[0], item[1], item[2]
        else:
            continue
        lat = _finite(latitude, float("nan"))
        lon = _finite(longitude, float("nan"))
        alt = _finite(altitude, float("nan"))
        if math.isfinite(lat) and math.isfinite(lon) and math.isfinite(alt):
            points.append((lat, lon, alt))
    return points


def _downsample(points: Sequence[GeoPoint], maximum: int) -> list[GeoPoint]:
    if maximum <= 0 or not points:
        return []
    if len(points) <= maximum:
        return list(points)
    if maximum == 1:
        return [points[-1]]
    scale = (len(points) - 1) / float(maximum - 1)
    indices: list[int] = []
    for index in range(maximum):
        source = min(len(points) - 1, int(round(index * scale)))
        if not indices or source != indices[-1]:
            indices.append(source)
    if indices[-1] != len(points) - 1:
        indices[-1] = len(points) - 1
    return [points[index] for index in indices]


def _polyline(points: Sequence[Vec3]) -> list[tuple[Vec3, Vec3]]:
    return [(points[index], points[index + 1]) for index in range(max(0, len(points) - 1))]


def _trajectory_segment_layers(
    points: Sequence[Vec3], near_distance: float
) -> tuple[list[tuple[Vec3, Vec3]], list[tuple[Vec3, Vec3]]]:
    """Split a trajectory into a bright near spine and a quieter far continuation.

    This keeps the trajectory itself as the dominant guidance cue without adding another
    reticle. The split is by actual world-space distance, so it remains stable when the
    predictor changes sample density.
    """
    segments = _polyline(points)
    if not segments:
        return [], []
    limit = max(1.0, near_distance)
    near: list[tuple[Vec3, Vec3]] = []
    far: list[tuple[Vec3, Vec3]] = []
    travelled = 0.0
    for segment in segments:
        length = _length(_add(segment[1], _mul(segment[0], -1.0)))
        if travelled < limit or not near:
            near.append(segment)
        else:
            far.append(segment)
        travelled += length
    return near, far




def _planned_back_trajectory_points(
    value: Any,
    current_ut: float,
    latitude: float,
    longitude: float,
    altitude: float,
    phase: str,
    radius: float,
    maximum: int,
    max_distance: float,
) -> list[GeoPoint]:
    """Return the selected guidance path behind the shuttle, never flown history.

    Timed predictions/references use samples before current UT. Static terminal paths
    use samples before the point nearest the live shuttle. The result is capped by
    distance and point count so the rear continuation stays useful without drawing
    the entire mission behind the camera.
    """
    samples = _reference_samples(value)
    if not samples or maximum <= 0:
        return []

    phase_upper = phase.upper()
    if any(token in phase_upper for token in ("FINAL", "FLARE", "TOUCHDOWN", "ROLLOUT")):
        filtered = [sample for sample in samples if "FINAL" in sample[3].upper()]
        if filtered:
            samples = filtered
    elif any(token in phase_upper for token in ("TAEM", "HAC", "ACQUISITION", "ALIGNMENT")):
        filtered = [
            sample
            for sample in samples
            if any(token in sample[3].upper() for token in ("TAEM", "FINAL", "HAC"))
        ]
        if filtered:
            samples = filtered

    timed = [sample for sample in samples if math.isfinite(sample[0]) and sample[0] > 1.0]
    if timed and math.isfinite(current_ut):
        candidates = [sample[1] for sample in timed if sample[0] < current_ut - 1e-6]
    elif math.isfinite(latitude) and math.isfinite(longitude):
        nearest = min(
            range(len(samples)),
            key=lambda i: (
                _surface_distance_m(latitude, longitude, samples[i][1][0], samples[i][1][1], radius)
                + (0.30 * abs(samples[i][1][2] - altitude) if math.isfinite(altitude) else 0.0)
            ),
        )
        candidates = [sample[1] for sample in samples[:nearest]]
    else:
        return []

    if not candidates:
        return []

    if math.isfinite(latitude) and math.isfinite(longitude) and math.isfinite(altitude):
        cursor: GeoPoint = (latitude, longitude, altitude)
    else:
        cursor = candidates[-1]
    selected: list[GeoPoint] = []
    travelled = 0.0
    distance_limit = max(1.0, max_distance)
    for point in reversed(candidates):
        surface = _surface_distance_m(cursor[0], cursor[1], point[0], point[1], radius)
        step = math.hypot(surface, point[2] - cursor[2])
        selected.append(point)
        travelled += step
        cursor = point
        if travelled >= distance_limit:
            break
    selected.reverse()
    return _downsample(selected, maximum)


def _back_extrapolation_positions(
    live_position: Vec3,
    future_positions: Sequence[Vec3],
    distance: float,
    count: int = 4,
) -> list[Vec3]:
    """Visually continue a future-only guidance path behind the shuttle.

    Some predictors publish only future samples. In that case a short linear
    continuation of the path's local tangent is preferable to substituting actual
    flown history. This is display geometry only; it never feeds guidance.
    """
    if not future_positions or count <= 0 or distance <= 0.0:
        return []
    tangent = _unit(_add(future_positions[0], _mul(live_position, -1.0)))
    if _length(tangent) <= 1e-6 and len(future_positions) >= 2:
        tangent = _unit(_add(future_positions[1], _mul(future_positions[0], -1.0)))
    if _length(tangent) <= 1e-6:
        return []
    count = max(2, count)
    return [
        _add(live_position, _mul(tangent, -distance * index / float(count)))
        for index in range(count, 0, -1)
    ]
def _norm_signed_deg(value: float) -> float:
    return (value + 180.0) % 360.0 - 180.0


def _surface_attitude_basis(heading_deg: float, pitch_deg: float, roll_deg: float) -> tuple[Vec3, Vec3, Vec3]:
    """Aircraft forward/right/up in kRPC surface axes (+X up, +Y north, +Z east)."""
    heading = math.radians(heading_deg)
    pitch = math.radians(pitch_deg)
    roll = math.radians(roll_deg)
    cos_pitch = math.cos(pitch)
    forward = _unit((math.sin(pitch), cos_pitch * math.cos(heading), cos_pitch * math.sin(heading)))
    unrolled_right = _unit((0.0, -math.sin(heading), math.cos(heading)))
    unrolled_up = _unit(_cross(forward, unrolled_right))
    # Positive aircraft roll lowers the right wing. Rotate the horizon-aligned right/up
    # pair about forward with the same sign convention used by guidance targetRoll.
    right = _unit(_add(_mul(unrolled_right, math.cos(roll)), _mul(unrolled_up, -math.sin(roll))))
    up = _unit(_add(_mul(unrolled_up, math.cos(roll)), _mul(unrolled_right, math.sin(roll))))
    return forward, right, up


def _target_attitude_basis_body(
    telemetry: Mapping[str, Any], command: Mapping[str, Any]
) -> tuple[Vec3, Vec3, Vec3] | None:
    """Guidance target expressed in current vessel body axes (+X right,+Y forward,+Z down)."""
    heading = _finite(telemetry.get("heading"), float("nan"))
    pitch = _finite(telemetry.get("pitch"), float("nan"))
    roll = _finite(telemetry.get("roll"), float("nan"))
    if not all(math.isfinite(v) for v in (heading, pitch, roll)):
        return None

    target_roll = _finite(command.get("targetRoll"), roll)
    target_aoa = _finite(command.get("targetAoA"), float("nan"))
    actual_aoa = _finite(telemetry.get("angleOfAttack"), float("nan"))

    if math.isfinite(target_aoa) and math.isfinite(actual_aoa):
        # Aerodynamic guidance does not command a surface-Euler pitch attitude. The flight
        # controller tracks angle of attack directly and uses bank to steer the trajectory;
        # targetHeading is a trajectory/course reference whose direct yaw contribution is
        # strongly authority-weighted. Combining targetHeading/targetPitch/targetRoll into one
        # Euler pose therefore draws a false director, especially at high bank. Mirror the
        # controller-native attitude errors instead: AoA about body-right and bank about the
        # nose. Heading/trajectory intent is already represented by the world-space path HUD.
        pitch_error = _finite(telemetry.get("commandPitchError"), target_aoa - actual_aoa)
        roll_error = _finite(
            telemetry.get("commandRollError"),
            _norm_signed_deg(target_roll - roll),
        )
        pitch_error = _clamp(pitch_error, -45.0, 45.0)
        roll_error = _norm_signed_deg(roll_error)
        relative_forward, relative_right, relative_up = _surface_attitude_basis(
            0.0, pitch_error, roll_error
        )

        # At the zero-error reference, surface axes map to body as
        # +Z(surface-east)->+X(body-right), +Y->+Y, +X(surface-up)->-Z(body-down).
        def relative_to_body(vector: Vec3) -> Vec3:
            return (vector[2], vector[1], -vector[0])

        return (
            _unit(relative_to_body(relative_forward)),
            _unit(relative_to_body(relative_right)),
            _unit(relative_to_body(relative_up)),
        )

    # Non-aerodynamic phases still carry a genuine surface Euler attitude target.
    target_heading = _finite(command.get("targetHeading"), heading)
    target_pitch = _finite(command.get("targetPitch"), pitch)
    current_forward, current_right, current_up = _surface_attitude_basis(heading, pitch, roll)
    target_forward, target_right, target_up = _surface_attitude_basis(target_heading, target_pitch, target_roll)

    def to_body(vector: Vec3) -> Vec3:
        return (
            _dot(vector, current_right),
            _dot(vector, current_forward),
            -_dot(vector, current_up),
        )

    return _unit(to_body(target_forward)), _unit(to_body(target_right)), _unit(to_body(target_up))


def _body_attitude_director_segments(
    telemetry: Mapping[str, Any], command: Mapping[str, Any], distance: float
) -> tuple[list[tuple[Vec3, Vec3]], list[tuple[Vec3, Vec3]]]:
    """Guidance director in the vessel body frame (+Y forward, +X right, -Z up)."""
    basis = _target_attitude_basis_body(telemetry, command)
    if basis is None:
        return [], []

    forward: Vec3 = (0.0, 1.0, 0.0)
    right: Vec3 = (1.0, 0.0, 0.0)
    up: Vec3 = (0.0, 0.0, -1.0)
    center = _mul(forward, distance)
    half = distance * 0.13
    gap = distance * 0.032
    datum = [
        (_add(center, _mul(right, -half)), _add(center, _mul(right, -gap))),
        (_add(center, _mul(right, gap)), _add(center, _mul(right, half))),
        (_add(center, _mul(up, -distance * 0.035)), _add(center, _mul(up, distance * 0.035))),
    ]

    target_forward, target_right, target_up = basis
    # Project the real 3D target direction onto the fixed-depth AR plane. Only the visual
    # excursion is clipped; current-roll coupling remains in target_forward itself.
    forward_depth = max(0.20, target_forward[1])
    max_right = distance * math.tan(math.radians(22.0))
    max_vertical = distance * math.tan(math.radians(16.0))
    target_center = (
        _clamp(distance * target_forward[0] / forward_depth, -max_right, max_right),
        distance,
        _clamp(distance * target_forward[2] / forward_depth, -max_vertical, max_vertical),
    )

    director_right = _unit((target_right[0], 0.0, target_right[2]))
    director_up = _unit((target_up[0], 0.0, target_up[2]))
    if _length(director_right) <= 1e-9:
        director_right = right
    director_up = _unit(_add(director_up, _mul(director_right, -_dot(director_up, director_right))))
    if _length(director_up) <= 1e-9:
        director_up = up
    if _dot(director_up, target_up) < 0.0:
        director_up = _mul(director_up, -1.0)

    span = distance * 0.105
    inner = distance * 0.040
    rise = distance * 0.040
    director = [
        (_add(_add(target_center, _mul(director_right, -span)), _mul(director_up, rise)),
         _add(target_center, _mul(director_right, -inner))),
        (_add(target_center, _mul(director_right, inner)),
         _add(_add(target_center, _mul(director_right, span)), _mul(director_up, rise))),
        (_add(target_center, _mul(director_up, -distance * 0.065)),
         _add(target_center, _mul(director_up, -distance * 0.025))),
    ]
    return datum, director


def _attitude_director_distance(radar_altitude: float, camera_distance: float = float("nan")) -> float:
    """Keep body-fixed attitude cues close to the shuttle while limiting extreme camera parallax."""
    radar = max(0.0, _finite(radar_altitude, 0.0))
    distance = _clamp(500.0 + radar * 0.004, 500.0, 1800.0)
    camera = _finite(camera_distance, float("nan"))
    if math.isfinite(camera) and camera > 0.0:
        distance = max(distance, camera * 10.0)
    return _clamp(distance, 500.0, 5000.0)


def _select_guidance_trajectory(snapshot: Mapping[str, Any]) -> tuple[Any, str]:
    phase = str(snapshot.get("phase") or "").upper()
    predicted = snapshot.get("predictedTrajectory")
    reference = snapshot.get("referenceTrajectory")
    planned = snapshot.get("plannedTrajectory")
    terminal = any(token in phase for token in ("TAEM", "HAC", "FINAL", "FLARE", "TOUCHDOWN", "ROLLOUT"))
    if not terminal:
        if isinstance(predicted, list) and len(predicted) >= 2:
            return predicted, "PRED"
        # An entry plan transition may intentionally suppress a stale/mismatched forecast
        # for one control interval. Never substitute planner/reference geometry and present
        # it as the active MM304 future; absence is safer and the forced next predictor tick
        # refills it.
        return [], "PRED WAIT"
    if isinstance(reference, list) and len(reference) >= 2:
        return reference, "REF"
    if isinstance(planned, list) and len(planned) >= 2:
        # Read-only fallback observers only know the selected planner candidate. Showing it
        # is useful, but provenance must remain explicit: it is PLAN, not the live terminal
        # reference path owned by guidance.
        return planned, "PLAN"
    return [], "REF WAIT"


def _active_trajectory_samples(
    value: Any,
    current_ut: float,
    latitude: float,
    longitude: float,
    altitude: float,
    phase: str,
    radius: float,
) -> list[ReferenceSample]:
    samples = _reference_samples(value)
    if not samples:
        return []

    phase_upper = phase.upper()
    timed = [sample for sample in samples if math.isfinite(sample[0]) and sample[0] > 1.0]
    if timed and math.isfinite(current_ut):
        # Keep the trajectory visually attached to the shuttle. The previous version
        # began the rendered path a fraction of a second in the future to avoid a
        # camera-facing ribbon crossing the chase camera, which left an obvious gap.
        # Instead, anchor the first point at the live vessel position and join it to
        # a short interpolated future point. We still never retain an already-passed
        # predictor sample, so the first segment cannot run backward through camera.
        if any(token in phase_upper for token in ("FINAL", "FLARE", "TOUCHDOWN", "ROLLOUT")):
            join_seconds = 0.20
        elif any(token in phase_upper for token in ("TAEM", "HAC", "ACQUISITION", "ALIGNMENT")):
            join_seconds = 0.35
        else:
            join_seconds = 0.50
        target_ut = current_ut + join_seconds
        index = next((i for i, sample in enumerate(timed) if sample[0] >= target_ut), len(timed))
        if index >= len(timed):
            if all(math.isfinite(value) for value in (latitude, longitude, altitude)):
                return [(current_ut, (latitude, longitude, altitude), float("nan"), phase)]
            return []

        active = list(timed[index:])
        join_sample: ReferenceSample | None = None
        if index > 0:
            before = timed[index - 1]
            after = timed[index]
            interval = after[0] - before[0]
            if before[0] <= target_ut <= after[0] and math.isfinite(interval) and interval > 1e-6:
                fraction = _clamp((target_ut - before[0]) / interval, 0.0, 1.0)
                before_geo = before[1]
                after_geo = after[1]
                longitude_delta = _norm_signed_deg(after_geo[1] - before_geo[1])
                join_geo = (
                    before_geo[0] + (after_geo[0] - before_geo[0]) * fraction,
                    before_geo[1] + longitude_delta * fraction,
                    before_geo[2] + (after_geo[2] - before_geo[2]) * fraction,
                )
                if math.isfinite(before[2]) and math.isfinite(after[2]):
                    join_speed = before[2] + (after[2] - before[2]) * fraction
                else:
                    join_speed = after[2] if math.isfinite(after[2]) else before[2]
                join_sample = (target_ut, join_geo, join_speed, after[3] or before[3])
                active.insert(0, join_sample)

        if all(math.isfinite(value) for value in (latitude, longitude, altitude)):
            first = join_sample if join_sample is not None else active[0]
            anchor_speed = first[2]
            anchor_phase = first[3] or phase
            active.insert(0, (current_ut, (latitude, longitude, altitude), anchor_speed, anchor_phase))
        return active

    if any(token in phase_upper for token in ("FINAL", "FLARE", "TOUCHDOWN", "ROLLOUT")):
        finals = [sample for sample in samples if "FINAL" in sample[3].upper()]
        if finals:
            samples = finals
    elif any(token in phase_upper for token in ("TAEM", "HAC", "ACQUISITION", "ALIGNMENT")):
        terminal = [sample for sample in samples if any(token in sample[3].upper() for token in ("TAEM", "FINAL", "HAC"))]
        if terminal:
            samples = terminal

    if not (math.isfinite(latitude) and math.isfinite(longitude)):
        return samples
    nearest = min(
        range(len(samples)),
        key=lambda i: (
            _surface_distance_m(latitude, longitude, samples[i][1][0], samples[i][1][1], radius)
            + (0.30 * abs(samples[i][1][2] - altitude) if math.isfinite(altitude) else 0.0)
        ),
    )
    # Static/reference paths have no reliable time axis. Skip the nearest path sample
    # (which may already be behind the shuttle), but prepend the live vessel position
    # so the visible trajectory still originates at the vehicle.
    start = min(len(samples), nearest + 1)
    active = list(samples[start:])
    if math.isfinite(altitude):
        anchor_speed = active[0][2] if active else samples[nearest][2]
        anchor_phase = (active[0][3] if active else samples[nearest][3]) or phase
        active.insert(0, (current_ut, (latitude, longitude, altitude), anchor_speed, anchor_phase))
    return active


def _compact_signed(value: float) -> str:
    magnitude = abs(value)
    if magnitude >= 1.0e9:
        return f"{value / 1.0e9:+.1f}G"
    if magnitude >= 1.0e6:
        return f"{value / 1.0e6:+.1f}M"
    if magnitude >= 1.0e3:
        return f"{value / 1.0e3:+.1f}k"
    return f"{value:+.0f}"


def _decision_label(snapshot: Mapping[str, Any]) -> str:
    phase = str(snapshot.get("phase") or "").upper()
    guidance = snapshot.get("guidanceState") if isinstance(snapshot.get("guidanceState"), Mapping) else {}
    parts: list[str] = []

    if any(token in phase for token in ("ENTRY", "MM304", "REENTRY")):
        if bool(guidance.get("entryReversalScheduled")):
            remaining = _finite(guidance.get("entryReversalTimeRemaining"), float("nan"))
            final = " FINAL" if bool(guidance.get("entryReversalIsFinal")) else ""
            parts.append(f"REV{final} T−{max(0.0, remaining):.0f}s" if math.isfinite(remaining) else f"REV{final}")
    elif any(token in phase for token in ("TAEM", "HAC", "ACQUISITION", "ALIGNMENT")):
        if bool(guidance.get("terminalPathCaptured")):
            parts.append("PATH CAPTURED")
        elif bool(guidance.get("terminalPathCommitted")):
            parts.append("PATH COMMITTED")
        elif bool(guidance.get("terminalCandidateValid")):
            parts.append("CANDIDATE")
        if bool(guidance.get("hacTransitionActive")):
            progress = _finite(guidance.get("hacTransitionProgress"), 0.0)
            parts.append(f"JOIN {progress * 100.0:.0f}%")

    if not parts:
        warning = str(snapshot.get("warningMessage") or snapshot.get("lastError") or "").strip()
        if warning:
            parts.append(warning.split(".", 1)[0][:72])
    return "  ·  ".join(parts[:2])


ReferenceSample = tuple[float, GeoPoint, float, str]


def _reference_samples(value: Any) -> list[ReferenceSample]:
    if not isinstance(value, list):
        return []
    samples: list[ReferenceSample] = []
    for item in value:
        if not isinstance(item, Mapping):
            continue
        latitude = _finite(item.get("latitude"), float("nan"))
        longitude = _finite(item.get("longitude"), float("nan"))
        altitude = _finite(item.get("altitude"), float("nan"))
        if not all(math.isfinite(v) for v in (latitude, longitude, altitude)):
            continue
        samples.append(
            (
                _finite(item.get("ut"), float("nan")),
                (latitude, longitude, altitude),
                _finite(item.get("speed"), float("nan")),
                str(item.get("phase") or ""),
            )
        )
    return samples


def _future_reference_points(value: Any, current_ut: float) -> list[GeoPoint]:
    samples = _reference_samples(value)
    if not samples:
        return _geo_points(value)
    if not math.isfinite(current_ut) or not any(math.isfinite(sample[0]) for sample in samples):
        return [sample[1] for sample in samples]
    future = [sample[1] for sample in samples if not math.isfinite(sample[0]) or sample[0] >= current_ut - 1.0]
    return future if len(future) >= 2 else [sample[1] for sample in samples[-2:]]


def _guidance_gate_samples(
    value: Any,
    current_ut: float,
    phase: str,
    maximum: int = 6,
    samples_override: Sequence[ReferenceSample] | None = None,
) -> list[ReferenceSample]:
    """Pick stable, existing trajectory samples as AR gates ahead of the shuttle.

    Gates do not slide continuously with a moving look-ahead point. They remain fixed
    in the KSP world until the shuttle passes them, which makes the tunnel visually
    fluid at any game frame rate and avoids RPC-driven reticle jitter.
    """
    samples = list(samples_override) if samples_override is not None else _reference_samples(value)
    if not samples or maximum <= 0:
        return []

    phase_upper = phase.upper()
    if any(token in phase_upper for token in ("FLARE", "FINAL", "TOUCHDOWN", "ROLLOUT")):
        offsets = (2.0, 4.0, 7.0, 10.0, 14.0, 19.0)
    elif any(token in phase_upper for token in ("TAEM", "HAC", "ACQUISITION", "ALIGNMENT")):
        offsets = (4.0, 8.0, 14.0, 21.0, 30.0, 42.0)
    elif any(token in phase_upper for token in ("ENTRY", "S-TURN", "MM304", "REENTRY")):
        offsets = (6.0, 12.0, 20.0, 30.0, 42.0, 56.0)
    else:
        offsets = (5.0, 10.0, 18.0, 28.0, 40.0, 55.0)

    timed = [sample for sample in samples if math.isfinite(sample[0]) and sample[0] > 1.0]
    if not math.isfinite(current_ut) or not timed:
        # Static terminal/fallback paths start at (or just behind) the shuttle after
        # active-segment clipping. Do not place the highlighted capture gate on top
        # of the vehicle; begin one or two path samples ahead.
        skip = 2 if len(samples) >= 4 else (1 if len(samples) >= 2 else 0)
        usable = samples[skip:]
        if not usable:
            return []
        if len(usable) <= maximum:
            return usable
        scale = (len(usable) - 1) / float(maximum - 1)
        return [usable[min(len(usable) - 1, int(round(i * scale)))] for i in range(maximum)]

    selected: list[ReferenceSample] = []
    for offset in offsets[:maximum]:
        target_ut = current_ut + offset
        candidate = next((sample for sample in timed if sample[0] >= target_ut), None)
        if candidate is None:
            break
        if not selected or candidate[0] != selected[-1][0]:
            selected.append(candidate)

    if len(selected) < min(3, maximum):
        first_future = next((i for i, sample in enumerate(timed) if sample[0] >= current_ut + 1.0), len(timed) - 1)
        for sample in timed[first_future:]:
            if sample not in selected:
                selected.append(sample)
            if len(selected) >= maximum:
                break
    return selected[:maximum]


def _gate_dimensions(phase: str, altitude: float) -> tuple[float, float]:
    phase_upper = phase.upper()
    altitude = max(0.0, altitude)
    if any(token in phase_upper for token in ("FLARE", "FINAL", "TOUCHDOWN", "ROLLOUT")):
        half_width = _clamp(42.0 + altitude * 0.020, 48.0, 180.0)
        half_height = _clamp(24.0 + altitude * 0.010, 28.0, 95.0)
    elif any(token in phase_upper for token in ("TAEM", "HAC", "ACQUISITION", "ALIGNMENT")):
        half_width = _clamp(180.0 + altitude * 0.014, 230.0, 620.0)
        half_height = _clamp(95.0 + altitude * 0.006, 120.0, 300.0)
    else:
        half_width = _clamp(320.0 + altitude * 0.010, 420.0, 980.0)
        half_height = _clamp(150.0 + altitude * 0.004, 190.0, 390.0)
    return half_width, half_height


def _gate_brackets(center: Vec3, lateral: Vec3, vertical: Vec3, half_width: float, half_height: float, fraction: float) -> list[tuple[Vec3, Vec3]]:
    fraction = _clamp(fraction, 0.12, 0.48)
    horizontal_leg = half_width * fraction
    vertical_leg = half_height * fraction
    segments: list[tuple[Vec3, Vec3]] = []
    for side in (-1.0, 1.0):
        for level in (-1.0, 1.0):
            corner = _add(_add(center, _mul(lateral, side * half_width)), _mul(vertical, level * half_height))
            inward_h = _add(corner, _mul(lateral, -side * horizontal_leg))
            inward_v = _add(corner, _mul(vertical, -level * vertical_leg))
            segments.append((corner, inward_h))
            segments.append((corner, inward_v))
    return segments


def _guidance_corridor_geometry(
    gates: Sequence[ReferenceSample],
    radius: float,
    prime: Vec3,
    quarter: Vec3,
    pole: Vec3,
    fallback_phase: str = "",
) -> tuple[list[tuple[Vec3, Vec3]], list[tuple[Vec3, Vec3]]]:
    """Return a sparse trajectory ladder and the highlighted next capture gate."""
    if not gates:
        return [], []
    positions = [_geo_position(sample[1], radius, prime, quarter, pole, lift=24.0) for sample in gates]
    corridor: list[tuple[Vec3, Vec3]] = []
    active: list[tuple[Vec3, Vec3]] = []
    tangents: list[Vec3] = []
    axes: list[tuple[Vec3, Vec3]] = []

    for index, center in enumerate(positions):
        if len(positions) == 1:
            _, east, north = _local_basis(gates[index][1][0], gates[index][1][1], prime, quarter, pole)
            tangent = north
        elif index == 0:
            tangent = _unit(_add(positions[1], _mul(positions[0], -1.0)))
        elif index == len(positions) - 1:
            tangent = _unit(_add(positions[-1], _mul(positions[-2], -1.0)))
        else:
            tangent = _unit(_add(positions[index + 1], _mul(positions[index - 1], -1.0)))

        radial_up = _unit(center)
        lateral = _unit(_cross(tangent, radial_up))
        if _length(lateral) <= 1e-6:
            _, east, _ = _local_basis(gates[index][1][0], gates[index][1][1], prime, quarter, pole)
            lateral = east
        vertical = _unit(_cross(lateral, tangent))
        if _dot(vertical, radial_up) < 0.0:
            vertical = _mul(vertical, -1.0)

        gate_phase = gates[index][3] or fallback_phase
        half_width, half_height = _gate_dimensions(gate_phase, gates[index][1][2])
        if index == 0:
            # The next capture point is the only full gate. Keep it open and modest so
            # it frames the path instead of becoming another dominant HUD reticle.
            active.extend(_gate_brackets(center, lateral, vertical, half_width * 0.72, half_height * 0.72, 0.24))
        else:
            # Future depth is conveyed by one short rung per gate. The centerline remains
            # unobstructed and therefore reads as the primary trajectory cue.
            rung_half = half_width * (0.34 if index == 1 else 0.26)
            corridor.append((_add(center, _mul(lateral, -rung_half)), _add(center, _mul(lateral, rung_half))))

        tangents.append(tangent)
        axes.append((lateral, vertical))

    # A tiny curvature chevron on the active gate communicates upcoming turn sense
    # without a separate roll/prograde instrument.
    if len(tangents) >= 2:
        center = positions[0]
        lateral, vertical = axes[0]
        radial_up = _unit(center)
        signed_turn = _dot(_cross(tangents[0], tangents[1]), radial_up)
        if abs(signed_turn) > 2e-4:
            phase0 = gates[0][3] or fallback_phase
            half_width, half_height = _gate_dimensions(phase0, gates[0][1][2])
            side = 1.0 if signed_turn > 0.0 else -1.0
            tip = _add(center, _mul(lateral, side * half_width * 0.42))
            tail = _add(tip, _mul(lateral, -side * half_width * 0.15))
            active.append((_add(tail, _mul(vertical, half_height * 0.08)), tip))
            active.append((_add(tail, _mul(vertical, -half_height * 0.08)), tip))

    return corridor, active


def _dashed_axis_segments(
    origin: Vec3,
    direction: Vec3,
    start_distance: float,
    end_distance: float,
    count: int,
    duty: float = 0.5,
) -> list[tuple[Vec3, Vec3]]:
    if count <= 0 or not math.isfinite(start_distance) or not math.isfinite(end_distance):
        return []
    span = end_distance - start_distance
    if abs(span) <= 1e-6:
        return []
    cell = span / float(count)
    duty = _clamp(duty, 0.05, 0.95)
    segments: list[tuple[Vec3, Vec3]] = []
    for index in range(count):
        dash_start = start_distance + cell * index
        dash_end = dash_start + cell * duty
        segments.append(
            (
                _add(origin, _mul(direction, dash_start)),
                _add(origin, _mul(direction, dash_end)),
            )
        )
    return segments


def _runway_segments(site: Mapping[str, Any], radius: float, prime: Vec3, quarter: Vec3, pole: Vec3) -> list[tuple[Vec3, Vec3]]:
    latitude = _finite(site.get("latitude"), float("nan"))
    longitude = _finite(site.get("longitude"), float("nan"))
    altitude = _finite(site.get("altitude"), 0.0)
    if not math.isfinite(latitude) or not math.isfinite(longitude):
        return []
    heading = _finite(site.get("runwayHeading"), 90.0)
    length = _clamp(_finite(site.get("runwayLength"), 2500.0), 300.0, 10000.0)
    width = _clamp(_finite(site.get("runwayWidth"), 70.0), 10.0, 500.0)
    threshold = _geo_position((latitude, longitude, altitude), radius, prime, quarter, pole, lift=10.0)
    _, east, north = _local_basis(latitude, longitude, prime, quarter, pole)
    heading_rad = math.radians(heading)
    along = _unit(_add(_mul(north, math.cos(heading_rad)), _mul(east, math.sin(heading_rad))))
    right = _unit(_add(_mul(east, math.cos(heading_rad)), _mul(north, -math.sin(heading_rad))))
    # LandingSite latitude/longitude is the Runway 09 threshold, not the runway midpoint.
    # Anchor the outline at that threshold; centering it here shifts the HUD west by half
    # the runway length (~1.25 km at KSC) and makes the display disagree with guidance.
    start = threshold
    end = _add(threshold, _mul(along, length))
    half_width = 0.5 * width
    start_left = _add(start, _mul(right, -half_width))
    start_right = _add(start, _mul(right, half_width))
    end_left = _add(end, _mul(right, -half_width))
    end_right = _add(end, _mul(right, half_width))

    outline = [
        (start_left, end_left),
        (start_right, end_right),
        (start_left, start_right),
        (end_left, end_right),
    ]
    approach_centerline = _dashed_axis_segments(start, along, -8000.0, -180.0, 9, duty=0.46)
    runway_centerline = _dashed_axis_segments(start, along, 120.0, length - 120.0, 8, duty=0.42)
    return outline + approach_centerline + runway_centerline


def _mode_label(snapshot: Mapping[str, Any]) -> str:
    phase = str(snapshot.get("phase") or "IDLE").upper()
    guidance = snapshot.get("guidanceState")
    detail = ""
    if isinstance(guidance, Mapping):
        if "TAEM" in phase:
            executive = guidance.get("taemExecutive")
        elif "ENTRY" in phase or "MM304" in phase:
            executive = guidance.get("entryExecutive")
        else:
            executive = None
        if isinstance(executive, Mapping):
            candidate = str(executive.get("phase") or "").strip()
            if candidate and candidate.upper() not in (phase, "IDLE", "NONE"):
                detail = candidate.upper()
    suffix = "AUTO" if bool(snapshot.get("automationEngaged")) else "FD"
    label = phase if not detail else f"{phase} · {detail}"
    return f"{label} · {suffix}"


def _surface_distance_m(latitude_a: float, longitude_a: float, latitude_b: float, longitude_b: float, radius: float) -> float:
    lat_a = math.radians(latitude_a)
    lat_b = math.radians(latitude_b)
    d_lat = lat_b - lat_a
    d_lon = math.radians(longitude_b - longitude_a)
    hav = math.sin(d_lat * 0.5) ** 2 + math.cos(lat_a) * math.cos(lat_b) * math.sin(d_lon * 0.5) ** 2
    return radius * 2.0 * math.asin(min(1.0, math.sqrt(max(0.0, hav))))


def _status_label(snapshot: Mapping[str, Any], configuration: Mapping[str, Any], radius: float) -> str:
    telemetry = snapshot.get("telemetry") if isinstance(snapshot.get("telemetry"), Mapping) else {}
    altitude = _finite(telemetry.get("meanAltitude"), float("nan"))
    speed = _finite(telemetry.get("trueAirSpeed"), float("nan"))
    parts: list[str] = []
    if math.isfinite(altitude):
        parts.append(f"ALT {altitude / 1000.0:.1f} km" if altitude >= 1000.0 else f"ALT {altitude:.0f} m")
    if math.isfinite(speed):
        parts.append(f"SPD {speed:.0f} m/s")

    site = configuration.get("site") if isinstance(configuration.get("site"), Mapping) else {}
    latitude = _finite(telemetry.get("latitude"), float("nan"))
    longitude = _finite(telemetry.get("longitude"), float("nan"))
    site_latitude = _finite(site.get("latitude"), float("nan"))
    site_longitude = _finite(site.get("longitude"), float("nan"))
    if all(math.isfinite(value) for value in (latitude, longitude, site_latitude, site_longitude)):
        distance = _surface_distance_m(latitude, longitude, site_latitude, site_longitude, radius)
        parts.append(f"RWY {distance / 1000.0:.1f} km" if distance >= 1000.0 else f"RWY {distance:.0f} m")
    return "  ·  ".join(parts)


def _performance_label(snapshot: Mapping[str, Any]) -> str:
    telemetry = snapshot.get("telemetry") if isinstance(snapshot.get("telemetry"), Mapping) else {}
    guidance = snapshot.get("guidanceState") if isinstance(snapshot.get("guidanceState"), Mapping) else {}
    primary: list[str] = []
    tactical: list[str] = []

    mach = _finite(telemetry.get("mach"), float("nan"))
    fpa = _finite(telemetry.get("flightPathAngle"), float("nan"))
    dynamic_pressure = _finite(telemetry.get("dynamicPressure"), float("nan"))
    g_force = _finite(telemetry.get("gForce"), float("nan"))
    if math.isfinite(mach):
        primary.append(f"M {mach:.2f}")
    if math.isfinite(fpa):
        primary.append(f"FPA {fpa:+.1f}°")
    if math.isfinite(dynamic_pressure):
        primary.append(f"Q {dynamic_pressure / 1000.0:.1f} kPa")
    if math.isfinite(g_force):
        primary.append(f"G {g_force:.2f}")

    energy_error = _finite(
        guidance.get("entryPlanTAEMEnergyError"),
        _finite(telemetry.get("predictedTAEMEnergyError"), float("nan")),
    )
    range_error = _finite(
        guidance.get("entryPlanTAEMRangeError"),
        _finite(telemetry.get("predictedTAEMRangeError"), float("nan")),
    )
    cross_track = _finite(telemetry.get("runwayCrossTrack"), float("nan"))
    if math.isfinite(energy_error):
        tactical.append(f"ΔE {_compact_signed(energy_error)}")
    if math.isfinite(range_error):
        tactical.append(f"ΔR {range_error / 1000.0:+.1f} km")
    if math.isfinite(cross_track):
        tactical.append(f"XTK {cross_track / 1000.0:+.1f} km" if abs(cross_track) >= 1000.0 else f"XTK {cross_track:+.0f} m")

    line_one = "  ·  ".join(primary)
    line_two = "  ·  ".join(tactical)
    return line_one if not line_two else (line_two if not line_one else f"{line_one}\n{line_two}")


class _LinePool:
    def __init__(self, drawing: Any, frame: Any, color: tuple[float, float, float], thickness: float) -> None:
        self.drawing = drawing
        self.frame = frame
        self.color = color
        self.thickness = thickness
        self.lines: list[Any] = []

    def render(self, segments: Sequence[tuple[Vec3, Vec3]]) -> None:
        while len(self.lines) < len(segments):
            start, end = segments[len(self.lines)]
            line = self.drawing.add_line(start, end, self.frame)
            line.color = self.color
            line.thickness = self.thickness
            self.lines.append(line)
        for index, line in enumerate(self.lines):
            if index < len(segments):
                start, end = segments[index]
                line.start = start
                line.end = end
                line.visible = True
            else:
                line.visible = False

    def hide(self) -> None:
        for line in self.lines:
            try:
                line.visible = False
            except Exception:
                pass




class InGameARHUD:
    """Visual-first in-game flight director rendered through a separate kRPC client."""

    def __init__(self) -> None:
        self.enabled = os.environ.get("KSP_LANDER_AR_HUD", "1").strip().lower() not in ("0", "false", "off", "no")
        self.address = os.environ.get("KSP_LANDER_AR_HUD_ADDRESS", "127.0.0.1")
        self.rpc_port = int(_finite(os.environ.get("KSP_LANDER_AR_HUD_RPC_PORT"), 50000.0))
        self.stream_port = int(_finite(os.environ.get("KSP_LANDER_AR_HUD_STREAM_PORT"), 50001.0))
        self.update_hz = _clamp(_finite(os.environ.get("KSP_LANDER_AR_HUD_HZ"), 18.0), 6.0, 30.0)
        self.trajectory_hz = _clamp(_finite(os.environ.get("KSP_LANDER_AR_HUD_TRAJECTORY_HZ"), 4.0), 1.0, 8.0)
        self.max_planned_points = int(_clamp(_finite(os.environ.get("KSP_LANDER_AR_HUD_PATH_POINTS"), 48.0), 12.0, 96.0))
        self.max_back_points = int(_clamp(_finite(os.environ.get("KSP_LANDER_AR_HUD_BACK_POINTS"), 24.0), 8.0, 64.0))

        self._lock = threading.Lock()
        self._latest_snapshot: dict[str, Any] = {}
        self._latest_configuration: dict[str, Any] = {}
        self._event = threading.Event()
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None
        if self.enabled:
            self._thread = threading.Thread(target=self._worker, name="ksp-ar-hud", daemon=True)
            self._thread.start()

    def update(self, snapshot: Mapping[str, Any], configuration: Mapping[str, Any] | None = None) -> None:
        if not self.enabled:
            return
        with self._lock:
            self._latest_snapshot = dict(snapshot)
            if configuration is not None:
                self._latest_configuration = dict(configuration)
        self._event.set()

    def suspend(self) -> None:
        if not self.enabled:
            return
        with self._lock:
            snapshot = dict(self._latest_snapshot)
            snapshot["connectionStatus"] = "disconnected"
            self._latest_snapshot = snapshot
        self._event.set()

    def close(self) -> None:
        self._stop.set()
        self._event.set()
        thread = self._thread
        self._thread = None
        if thread is not None and thread is not threading.current_thread():
            thread.join(timeout=2.0)

    def _snapshot_copy(self) -> tuple[dict[str, Any], dict[str, Any]]:
        with self._lock:
            return dict(self._latest_snapshot), dict(self._latest_configuration)

    def _worker(self) -> None:
        connection: Any = None
        vessel: Any = None
        frame: Any = None
        cue_frame: Any = None
        drawing: Any = None
        mode_text: Any = None
        decision_text: Any = None
        hud_width = 0.0
        hud_height = 0.0
        planned_pool: _LinePool | None = None
        attitude_datum_pool: _LinePool | None = None
        attitude_director_pool: _LinePool | None = None
        runway_pool: _LinePool | None = None
        radius = 600000.0
        prime: Vec3 = (1.0, 0.0, 0.0)
        quarter: Vec3 = (0.0, 1.0, 0.0)
        pole: Vec3 = (0.0, 0.0, 1.0)
        next_connect = 0.0
        next_fast = 0.0
        next_path = 0.0
        next_ui = 0.0
        last_path_signature: tuple[Any, ...] | None = None
        last_runway_signature: tuple[Any, ...] | None = None
        last_mode = ""
        last_decision = ""
        last_failure = ""
        last_failure_time = 0.0

        def report_failure(exc: Exception) -> None:
            nonlocal last_failure, last_failure_time
            text = f"{type(exc).__name__}: {exc}"
            now = time.monotonic()
            if text != last_failure or now - last_failure_time >= 10.0:
                print(f"KSP AR HUD reconnecting after failure: {text}", file=sys.stderr, flush=True)
                last_failure = text
                last_failure_time = now

        def clear_hud() -> None:
            nonlocal mode_text, decision_text, hud_width, hud_height, last_mode, last_decision
            for item in (mode_text, decision_text):
                if item is not None:
                    try:
                        item.remove()
                    except Exception:
                        pass
            mode_text = decision_text = None
            hud_width = hud_height = 0.0
            last_mode = ""
            last_decision = ""

        def create_hud() -> bool:
            nonlocal mode_text, decision_text, hud_width, hud_height
            if connection is None:
                return False
            try:
                canvas = connection.ui.stock_canvas
                screen_width, screen_height = canvas.rect_transform.size
                hud_width = float(screen_width)
                hud_height = float(screen_height)
                column_width = _clamp(hud_width * 0.24, 190.0, 320.0)
                column_height = _clamp(hud_height * 0.22, 90.0, 150.0)
                column_y = hud_height * 0.08

                # KSP canvas coordinates are centered. +/- width/6 are exactly the
                # 1/3 and 2/3 screen-width vertical guide lines.
                mode_text = canvas.add_text("")
                mode_text.rect_transform.size = (column_width, column_height)
                mode_text.rect_transform.position = (-hud_width / 6.0, column_y)
                mode_text.size = 14
                mode_text.style = connection.ui.FontStyle.bold
                mode_text.color = _HUD_GREEN_PRIMARY
                mode_text.alignment = connection.ui.TextAnchor.middle_center

                decision_text = canvas.add_text("")
                decision_text.rect_transform.size = (column_width, column_height)
                decision_text.rect_transform.position = (hud_width / 6.0, column_y)
                decision_text.size = 13
                decision_text.style = connection.ui.FontStyle.bold
                decision_text.color = _HUD_GREEN_SECONDARY
                decision_text.alignment = connection.ui.TextAnchor.middle_center

                return True
            except Exception:
                clear_hud()
                return False

        def disconnect() -> None:
            nonlocal connection, vessel, frame, cue_frame, drawing
            nonlocal planned_pool, attitude_datum_pool, attitude_director_pool, runway_pool
            nonlocal last_path_signature, last_runway_signature
            clear_hud()
            if drawing is not None:
                try:
                    drawing.clear(client_only=True)
                except Exception:
                    pass
            if connection is not None:
                try:
                    connection.close()
                except Exception:
                    pass
            connection = vessel = frame = cue_frame = drawing = None
            planned_pool = attitude_datum_pool = attitude_director_pool = runway_pool = None
            last_path_signature = None
            last_runway_signature = None

        try:
            while not self._stop.is_set():
                self._event.wait(1.0 / self.update_hz)
                self._event.clear()
                if self._stop.is_set():
                    break
                snapshot, configuration = self._snapshot_copy()
                if str(snapshot.get("connectionStatus", "disconnected")).lower() != "connected":
                    if connection is not None:
                        disconnect()
                    continue

                now = time.monotonic()
                if connection is None:
                    if now < next_connect:
                        continue
                    try:
                        import krpc  # type: ignore

                        connection = krpc.connect(
                            name="KSP Shuttle Lander AR HUD",
                            address=self.address,
                            rpc_port=self.rpc_port,
                            stream_port=self.stream_port,
                        )
                        vessel = connection.space_center.active_vessel
                        if vessel is None:
                            raise RuntimeError("no active vessel")
                        body = vessel.orbit.body
                        frame = body.reference_frame
                        cue_frame = vessel.reference_frame
                        radius = max(1.0, _finite(body.equatorial_radius, 600000.0))
                        prime = _unit(body.msl_position(0.0, 0.0, frame))
                        quarter = _unit(body.msl_position(0.0, 90.0, frame))
                        pole = _unit(body.msl_position(90.0, 0.0, frame))
                        drawing = connection.drawing
                        # One persistent world-space centerline. No layered halos, gates,
                        # tethers, or history overlays: independently updated line layers caused
                        # visible flicker around the shuttle.
                        planned_pool = _LinePool(drawing, frame, _HUD_GREEN_PRIMARY, 2.0)
                        attitude_datum_pool = _LinePool(drawing, cue_frame, _HUD_GREEN_SECONDARY, 2.6)
                        attitude_director_pool = _LinePool(drawing, cue_frame, _HUD_GREEN_PRIMARY, 3.8)
                        runway_pool = _LinePool(drawing, frame, _HUD_GREEN_SECONDARY, 3.0)

                        next_ui = 0.0
                        if not create_hud():
                            next_ui = now + 1.0

                        next_fast = 0.0
                        next_path = 0.0
                        last_path_signature = None
                        last_runway_signature = None
                        last_mode = ""
                        last_decision = ""
                        last_failure = ""
                    except Exception as exc:
                        report_failure(exc)
                        disconnect()
                        next_connect = now + 2.0
                        continue

                try:
                    assert planned_pool is not None and runway_pool is not None
                    assert attitude_datum_pool is not None and attitude_director_pool is not None
                    if now >= next_fast:
                        telemetry = snapshot.get("telemetry") if isinstance(snapshot.get("telemetry"), Mapping) else {}
                        command = snapshot.get("command") if isinstance(snapshot.get("command"), Mapping) else {}

                        if mode_text is None and now >= next_ui:
                            if create_hud():
                                next_ui = 0.0
                            else:
                                next_ui = now + 2.0

                        # World-space AR cues do not depend on KSP's stock text canvas. Keep
                        # them alive through UI canvas/scene transitions whenever Drawing itself
                        # is still valid.
                        radar_altitude = max(0.0, _finite(telemetry.get("radarAltitude"), _finite(telemetry.get("meanAltitude"))))
                        camera_distance = float("nan")
                        try:
                            camera_distance = _finite(connection.space_center.camera.distance, float("nan"))
                        except Exception:
                            # Camera distance is display-only metadata; a scene/mode that does
                            # not expose it falls back to the conservative far-depth projection.
                            pass
                        director_distance = _attitude_director_distance(radar_altitude, camera_distance)
                        datum_segments, director_segments = _body_attitude_director_segments(
                            telemetry, command, director_distance
                        )
                        attitude_datum_pool.render(datum_segments)
                        surface_guidance = not bool(command.get("useInertialDirection"))
                        engaged = bool(snapshot.get("automationEngaged")) or bool(command.get("autopilotEngaged"))
                        if surface_guidance and engaged:
                            attitude_director_pool.render(director_segments)
                        else:
                            attitude_director_pool.hide()
                        if mode_text is not None:
                            try:
                                mode = _mode_label(snapshot)
                                decision = _decision_label(snapshot)
                                mode_column = "\n".join(part.strip() for part in mode.split(" · ") if part.strip())
                                decision_column = "\n".join(part.strip() for part in decision.split(" · ") if part.strip())
                                if mode_column != last_mode:
                                    mode_text.content = mode_column
                                    last_mode = mode_column
                                warning = bool(snapshot.get("warningMessage") or snapshot.get("lastError"))
                                mode_text.color = _HUD_WARNING if warning else _HUD_GREEN_PRIMARY
                                if decision_text is not None and decision_column != last_decision:
                                    decision_text.content = decision_column
                                    last_decision = decision_column
                            except Exception:
                                # Scene transitions can invalidate stock-canvas text independently
                                # of kRPC Drawing. Recreate only the text overlay on the next tick.
                                clear_hud()
                                next_ui = now + 1.0
                        next_fast = now + 1.0 / self.update_hz

                    site = configuration.get("site") if isinstance(configuration.get("site"), Mapping) else {}
                    runway_signature = tuple(
                        round(_finite(site.get(key)), 6)
                        for key in ("latitude", "longitude", "altitude", "runwayHeading", "runwayLength", "runwayWidth")
                    )
                    if runway_signature != last_runway_signature:
                        runway_pool.render(_runway_segments(site, radius, prime, quarter, pole))
                        last_runway_signature = runway_signature

                    if now >= next_path:
                        telemetry = snapshot.get("telemetry") if isinstance(snapshot.get("telemetry"), Mapping) else {}
                        current_ut = _finite(telemetry.get("ut"), float("nan"))
                        latitude = _finite(telemetry.get("latitude"), float("nan"))
                        longitude = _finite(telemetry.get("longitude"), float("nan"))
                        phase = str(snapshot.get("phase") or "")
                        raw_path, source_tag = _select_guidance_trajectory(snapshot)
                        altitude = _finite(telemetry.get("meanAltitude"), float("nan"))
                        active_samples = _active_trajectory_samples(
                            raw_path, current_ut, latitude, longitude, altitude, phase, radius
                        )

                        # The visible route is plan geometry only. Do not splice in the live
                        # shuttle position: that made the centerline twitch with telemetry.
                        future_samples = active_samples
                        if active_samples and all(math.isfinite(value) for value in (latitude, longitude, altitude)):
                            first_geo = active_samples[0][1]
                            if (
                                abs(first_geo[0] - latitude) < 1e-7
                                and abs(_norm_signed_deg(first_geo[1] - longitude)) < 1e-7
                                and abs(first_geo[2] - altitude) < 0.5
                            ):
                                future_samples = active_samples[1:]
                        future_geo = _downsample(
                            [sample[1] for sample in future_samples], self.max_planned_points
                        )

                        phase_upper = phase.upper()
                        if any(token in phase_upper for token in ("FINAL", "FLARE", "TOUCHDOWN", "ROLLOUT")):
                            back_distance = 6000.0
                        elif any(token in phase_upper for token in ("TAEM", "HAC", "ACQUISITION", "ALIGNMENT")):
                            back_distance = 18000.0
                        else:
                            back_distance = 35000.0
                        back_geo = _planned_back_trajectory_points(
                            raw_path, current_ut, latitude, longitude, altitude, phase,
                            radius, self.max_back_points, back_distance
                        )

                        planned_positions = [
                            _geo_position(point, radius, prime, quarter, pole, lift=24.0)
                            for point in future_geo
                        ]
                        back_positions = [
                            _geo_position(point, radius, prime, quarter, pole, lift=24.0)
                            for point in back_geo
                        ]
                        if not back_positions and len(planned_positions) >= 2:
                            # Future-only paths get the same line extended backward from their
                            # own tangent. This remains fixed to the plan, not live vessel motion.
                            back_positions = _back_extrapolation_positions(
                                planned_positions[0],
                                planned_positions[1:],
                                min(back_distance, 12000.0),
                                count=min(self.max_back_points, 6),
                            )

                        centerline_positions = back_positions + planned_positions
                        signature = (
                            source_tag,
                            tuple(
                                (round(point[0], 1), round(point[1], 1), round(point[2], 1))
                                for point in centerline_positions
                            ),
                        )
                        if signature != last_path_signature:
                            planned_pool.render(_polyline(centerline_positions))
                            last_path_signature = signature
                        next_path = now + 1.0 / self.trajectory_hz
                except Exception as exc:
                    report_failure(exc)
                    disconnect()
                    next_connect = time.monotonic() + 1.0
        finally:
            disconnect()
