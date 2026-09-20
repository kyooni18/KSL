"""Offline guidance-level experiment. Standard library only; no network API."""
import ctypes as C
import json
import math
from pathlib import Path
import random
import sys

ROOT = Path(__file__).resolve().parents[2]
SIM = ROOT / "ShuttleSim"
SCHEMA = "shuttlesim-guidance-v2"
# Explicit experiment coverage only; never used as a flight-feasibility gate.
DEORBIT_COVERAGE_MAX_ALTITUDE_M = 400000.0
MAX_INITIAL_INCLINATION_DEG = 90.0  # prograde-hemisphere sampling domain

# Every continuous feature below is already dimensionless when observation()
# returns it.  Normalizers come from planet physics, active vehicle/config limits,
# modeled actuator rates, or mathematical angle domains rather than training-tuned
# scales.  Path-provider internals (HAC radius/progress/violation score) are
# intentionally absent: an RL residual must not learn to compensate for planner
# implementation details.
FEATURES = (
    "phase_fraction",
    "altitude_radius_fraction", "radar_altitude_radius_fraction",
    "airspeed_orbital_fraction", "horizontal_speed_orbital_fraction",
    "vertical_speed_orbital_fraction", "mach", "dynamic_pressure_fraction",
    "flight_path_angle_fraction", "aoa_authority_fraction", "bank_authority_fraction",
    "aoa_rate_fraction", "bank_rate_fraction", "course_rate_authority_fraction",
    "runway_along_radius_fraction", "runway_cross_radius_fraction",
    "range_to_site_radius_fraction", "bearing_fraction", "heading_error_fraction",
    "taem_course_error_fraction", "projected_taem_range_error_radius_fraction",
    "gear_down", "brakes", "expert_aoa_fraction", "expert_bank_fraction",
    "residual_enabled",
    "authority_speed_margin", "authority_q_margin", "authority_load_margin",
    "authority_stall_margin",
    "capture_along_radius_fraction", "capture_cross_radius_fraction",
    "capture_altitude_error_radius_fraction", "capture_energy_margin_fraction",
    "capture_turn_margin_radius_fraction", "capture_valid", "capture_ready",
)



def _signed_degrees(angle):
    return (float(angle) + 180.0) % 360.0 - 180.0


def taem_course_signed_error_deg(telemetry):
    """Signed heading error to the nearer runway-perpendicular inlet."""
    if not isinstance(telemetry, dict):
        raise ValueError("telemetry is required")
    attitude = telemetry.get("attitude", {})
    runway = telemetry.get("runway", {})
    heading = _finite(attitude.get("heading_deg"), float("nan"))
    runway_heading = _finite(runway.get("heading_deg"), float("nan"))
    if not (math.isfinite(heading) and math.isfinite(runway_heading)):
        raise ValueError("runway and vehicle heading are required")
    first = _signed_degrees(heading - (runway_heading - 90.0))
    reciprocal = _signed_degrees(heading - (runway_heading + 90.0))
    return first if abs(first) <= abs(reciprocal) else reciprocal


def taem_course_error_deg(telemetry):
    return abs(taem_course_signed_error_deg(telemetry))


def _finite(value, default=0.0):
    try:
        value = float(value)
    except (TypeError, ValueError):
        return default
    return value if math.isfinite(value) else default


def _ratio(value, scale, name):
    value = _finite(value, float("nan"))
    scale = _finite(scale, float("nan"))
    if not (math.isfinite(value) and math.isfinite(scale) and scale > 0.0):
        if value == 0.0 and scale == 0.0:
            return 0.0
        # A zero authority limit is a valid physical state at the edge of the
        # envelope (for example, before dynamic pressure makes bank authority
        # available).  Preserve the fact that the measured value is outside
        # that unavailable domain without producing a non-finite observation.
        if math.isfinite(value) and scale == 0.0:
            return math.copysign(5.0, value)
        raise ValueError(f"invalid physical normalizer for {name}")
    return max(-5.0, min(5.0, value / scale))


def taem_capture_veto_reasons(veto):
    try:
        veto = int(veto)
    except (TypeError, ValueError):
        return []
    reasons = []
    for bit, name in (
        (1, "speed"), (2, "spatial"), (4, "course"), (8, "altitude"),
        (16, "maneuver-energy"), (32, "fpa"), (64, "structural"),
        (128, "hac-radius"), (256, "minimum-altitude")):
        if veto & bit:
            reasons.append(name)
    return reasons


def taem_interface_debt(telemetry, expert):
    """Diagnostic debt reported directly from the production handoff contract."""
    capture_valid = bool(expert.get("taem_capture_valid"))
    capture_ready = bool(expert.get("taem_capture_ready"))
    capture_veto = int(_finite(expert.get("taem_capture_veto"), 0.0))
    reasons = taem_capture_veto_reasons(capture_veto)
    if capture_veto and not reasons:
        reasons = ["unknown-veto"]
    if not capture_valid:
        reasons = ["capture-invalid"]
        return {
            "position_error_m": None,
            "altitude_error_m": None,
            "course_error_deg": taem_course_error_deg(telemetry),
            "energy_error": None,
            "turn_error_m": None,
            "capture_valid": False,
            "capture_ready": False,
            "capture_veto": capture_veto,
            "capture_veto_reasons": reasons,
            "constraint_violations": None,
        }
    along_error = _finite(expert.get("taem_capture_along_m"))
    cross_error = _finite(expert.get("taem_capture_cross_m"))
    energy_margin = _finite(expert.get("taem_capture_energy_margin"))
    turn_margin = _finite(expert.get("taem_capture_turn_margin"))
    return {
        "position_error_m": math.hypot(along_error, cross_error),
        "altitude_error_m": max(0.0, -_finite(expert.get("taem_capture_altitude_error_m"))),
        "course_error_deg": taem_course_error_deg(telemetry),
        "energy_error": max(0.0, -energy_margin),
        "turn_error_m": max(0.0, -turn_margin),
        "capture_valid": True,
        "capture_ready": capture_ready,
        "capture_veto": capture_veto,
        "capture_veto_reasons": reasons,
        "constraint_violations": len(reasons),
    }


def terminal_path_contract(expert):
    """Return the production MM305 path contract, separate from MM304 debt.

    A local TAEM handoff can be ready while the downstream terminal candidate
    is geometrically degraded, energy-infeasible, or not yet committed.  The
    simulator/RL adapter must expose those states explicitly instead of
    treating ``initial_handoff`` or ``curriculum_handoff`` as a runway-path
    event.
    """
    diagnostics = expert.get("diagnostics", {}) if isinstance(expert, dict) else {}
    if not isinstance(diagnostics, dict):
        diagnostics = {}
    candidate_valid = bool(diagnostics.get("candidate_valid"))
    candidate_degraded = bool(diagnostics.get("candidate_degraded"))
    geometry_degraded = bool(diagnostics.get("candidate_geometry_degraded"))
    energy_degraded = bool(diagnostics.get("candidate_energy_degraded"))
    energy_valid = bool(diagnostics.get("candidate_live_energy_valid"))
    margin = _finite(diagnostics.get("candidate_live_energy_margin_j_kg"), float("nan"))
    energy_ready = energy_valid and math.isfinite(margin) and margin >= 0.0
    candidate_ready = (candidate_valid and not candidate_degraded and
                       not geometry_degraded and not energy_degraded and energy_ready)
    path_committed = bool(diagnostics.get("path_committed"))
    blockers = []
    if not candidate_valid:
        blockers.append("candidate-invalid")
    if candidate_degraded:
        blockers.append("candidate-degraded")
    if geometry_degraded:
        blockers.append("geometry-degraded")
    if energy_degraded:
        blockers.append("energy-degraded")
    if not energy_valid:
        blockers.append("energy-invalid")
    elif not math.isfinite(margin):
        blockers.append("energy-margin-invalid")
    elif margin < 0.0:
        blockers.append("negative-live-energy")
    if not path_committed:
        blockers.append("path-not-committed")
    return {
        "candidate_valid": candidate_valid,
        "candidate_degraded": candidate_degraded,
        "candidate_geometry_degraded": geometry_degraded,
        "candidate_energy_degraded": energy_degraded,
        "candidate_live_energy_valid": energy_valid,
        "candidate_live_energy_margin_j_kg": margin if math.isfinite(margin) else None,
        "candidate_ready": candidate_ready,
        "path_committed": path_committed,
        "path_ready": path_committed,
        "blockers": blockers,
    }


def _world_model(world):
    required = ("radius_m", "mu_m3_s2", "rotation_rate_rad_s",
                "atmosphere_top_m", "rotation_phase_rad_at_ut0")
    if not isinstance(world, dict) or not all(k in world for k in required):
        raise ValueError("missing simulator world model")
    values = {k: _finite(world[k], float("nan")) for k in required}
    if (not all(math.isfinite(v) for v in values.values())
            or values["radius_m"] <= 0.0 or values["mu_m3_s2"] <= 0.0
            or values["atmosphere_top_m"] < 0.0):
        raise ValueError("invalid simulator world model")
    return values


def orbital_elements(telemetry):
    """Diagnostic two-body elements using the simulator's authoritative world."""
    world = _world_model(telemetry.get("world"))
    mu, body_radius = world["mu_m3_s2"], world["radius_m"]
    p = telemetry["position"]
    v = telemetry["velocity"]
    position = [p[k] for k in ("x", "y", "z")]
    velocity = [v[k] for k in ("x_mps", "y_mps", "z_mps")]
    radius = math.sqrt(sum(x * x for x in position))
    speed2 = sum(x * x for x in velocity)
    h = [position[1] * velocity[2] - position[2] * velocity[1],
         position[2] * velocity[0] - position[0] * velocity[2],
         position[0] * velocity[1] - position[1] * velocity[0]]
    hnorm = math.sqrt(sum(x * x for x in h))
    specific_energy = speed2 / 2 - mu / radius
    radial_dot = sum(position[j] * velocity[j] for j in range(3))
    evec = [((speed2 - mu / radius) * position[i] - radial_dot * velocity[i]) / mu
            for i in range(3)]
    eccentricity = math.sqrt(sum(x * x for x in evec))
    semimajor_axis = -mu / (2 * specific_energy) if specific_energy < 0 else None
    return {
        "specific_energy_m2_s2": specific_energy,
        "inclination_deg": (math.degrees(math.acos(max(-1.0, min(1.0, h[2] / hnorm))))
                            if hnorm > 0.0 else 0.0),
        "eccentricity": eccentricity,
        "semimajor_axis_m": semimajor_axis,
        "periapsis_altitude_m": (semimajor_axis * (1 - eccentricity) - body_radius
                                  if semimajor_axis is not None else None),
        "apoapsis_altitude_m": (semimajor_axis * (1 + eccentricity) - body_radius
                                 if semimajor_axis is not None else None),
    }


def deorbit_delta_v(apoapsis_altitude_m, periapsis_altitude_m,
                    target_periapsis_altitude_m, world):
    """Retrograde apoapsis impulse from the simulator's two-body world model."""
    model = _world_model(world)
    radius, mu = model["radius_m"], model["mu_m3_s2"]
    ra = radius + apoapsis_altitude_m
    rp = radius + periapsis_altitude_m
    target_rp = radius + target_periapsis_altitude_m
    semimajor = (ra + rp) / 2
    target_semimajor = (ra + target_rp) / 2
    pre = math.sqrt(mu * (2 / ra - 1 / semimajor))
    post = math.sqrt(mu * (2 / ra - 1 / target_semimajor))
    return pre - post


def deorbit_entry_true_anomaly(apoapsis_altitude_m, post_periapsis_altitude_m, world):
    model = _world_model(world)
    radius = model["radius_m"]
    ra = radius + apoapsis_altitude_m
    rp = radius + post_periapsis_altitude_m
    semimajor = (ra + rp) / 2
    eccentricity = (ra - rp) / (ra + rp)
    parameter = semimajor * (1 - eccentricity * eccentricity)
    cosine = (parameter / (radius + model["atmosphere_top_m"]) - 1) / eccentricity
    return -math.acos(max(-1.0, min(1.0, cosine)))


def _norm(vector):
    return math.sqrt(sum(x * x for x in vector))


def _cross(a, b):
    return [a[1] * b[2] - a[2] * b[1],
            a[2] * b[0] - a[0] * b[2],
            a[0] * b[1] - a[1] * b[0]]


def _scale(vector, factor):
    return [factor * x for x in vector]


def _add(a, b):
    return [x + y for x, y in zip(a, b)]


def _rotate_z(vector, angle):
    c, s = math.cos(angle), math.sin(angle)
    return [c * vector[0] - s * vector[1],
            s * vector[0] + c * vector[1], vector[2]]


def _wrap_degrees(angle):
    return (angle + 180) % 360 - 180


def runway_offset_position(along_m, cross_m, runway, world):
    """Invert the runway along/cross convention used by deorbit_entry_screen."""
    distance = math.hypot(along_m, cross_m)
    if distance < 1e-9:
        return runway["latitude_deg"], runway["longitude_deg"]
    bearing = math.radians(runway["heading_deg"] + math.degrees(math.atan2(cross_m, along_m)))
    angular = distance / _world_model(world)["radius_m"]
    lat1 = math.radians(runway["latitude_deg"])
    lon1 = math.radians(runway["longitude_deg"])
    sin_lat2 = (math.sin(lat1) * math.cos(angular)
                + math.cos(lat1) * math.sin(angular) * math.cos(bearing))
    lat2 = math.asin(max(-1.0, min(1.0, sin_lat2)))
    lon2 = lon1 + math.atan2(
        math.sin(bearing) * math.sin(angular) * math.cos(lat1),
        math.cos(angular) - math.sin(lat1) * math.sin(lat2),
    )
    return math.degrees(lat2), _wrap_degrees(math.degrees(lon2))


def curriculum_initial_condition(kind, rng, seed, runway, nominal, world):
    """Sample a simulator-only short-horizon state near the TAEM interface.

    These states are a curriculum for learning entry residuals.  They are not
    hidden policy inputs and do not alter live guidance; they simply avoid
    spending every smoke rollout hundreds of kilometers away from the handoff
    contract.
    """
    kind = kind or "full"
    if kind == "entry-corridor":
        # Proximal MM304 shaping curriculum: far enough upstream to need a real
        # energy/lateral correction, but not so far that short smoke runs are
        # dominated by unrelated entry-screen geometry.
        along = rng.uniform(-150000.0, -70000.0)
        cross = rng.uniform(-8000.0, 8000.0)
        altitude = rng.uniform(46000.0, 58000.0)
        speed = rng.uniform(1200.0, 1650.0)
        fpa = rng.uniform(-4.4, -2.0)
        heading = rng.uniform(-8.0, 8.0)
    elif kind == "interface-corridor":
        # Short-horizon acquisition curriculum for the strict 1 km handoff tube.
        # The state is screened by the production handoff contract at runtime;
        # downstream path feasibility remains an observed outcome, not a
        # hidden sampling guarantee.
        along = -8000.0
        cross = 0.0
        altitude = rng.uniform(21200.0, 21700.0)
        speed = rng.uniform(560.0, 630.0)
        fpa = rng.uniform(-8.0, -4.0)
        # At this low-density inlet a one-degree course correction can consume
        # tens of kilometres of turn radius.  The production target is the
        # runway-perpendicular course, so the curriculum starts on that
        # demonstrated tangent rather than hiding an infeasible heading debt.
        heading = 0.0
    elif kind == "terminal-corridor":
        # Seed directly inside the real C-side ownership tube. These bounds
        # are deliberately narrower than the interface curriculum and keep the
        # initial state near the 1 km point contract. The native expert still
        # owns the downstream terminal feasibility decision; being in the
        # handoff set does not certify a flyable runway path.
        # The offline executive currently initializes the positive S-turn side;
        # keep the inlet near that perpendicular course instead of fabricating
        # a reciprocal-side case it cannot own.
        along = -8000.0
        cross = 0.0
        altitude = rng.uniform(21200.0, 21700.0)
        speed = rng.uniform(560.0, 630.0)
        fpa = rng.uniform(-8.0, -4.0)
        heading = 0.0
    else:
        raise ValueError(f"unknown curriculum kind: {kind}")
    # Keep deterministic seed strata visible while allowing stochastic spread.
    if seed % 2:
        cross = -cross
    latitude, longitude = runway_offset_position(along, cross, runway, world)
    return {
        "stage": kind,
        "altitude_m": altitude,
        "speed_mps": speed,
        "flight_path_angle_deg": fpa,
        "heading_deg": _wrap_degrees(heading),
        "latitude_deg": latitude,
        "longitude_deg": longitude,
        "runway_along_m": along,
        "runway_cross_m": cross,
        "base_ut": nominal["ut"],
        "energy_case": "curriculum",
    }


def deorbit_entry_screen(apoapsis_altitude_m, post_periapsis_altitude_m,
                         inclination_deg, raan_deg, argument_of_periapsis_deg, ut,
                         runway, world):
    """Screen a post-deorbit ballistic crossing for MM304/TAEM plausibility.

    This is deliberately a conservative pre-sampler filter, not a replacement
    for the atmospheric simulation.  It rejects only obviously unusable
    combinations: non-entry flight-path energy, extreme entry speed, a ground
    track that cannot approach the runway corridor, or a crossing that is not
    upstream of the site for the configured runway heading.
    """
    model = _world_model(world)
    radius, mu = model["radius_m"], model["mu_m3_s2"]
    rotation_rate = model["rotation_rate_rad_s"]
    rotation_phase = model["rotation_phase_rad_at_ut0"]
    atmosphere_top = model["atmosphere_top_m"]
    ra = radius + apoapsis_altitude_m
    rp = radius + post_periapsis_altitude_m
    semimajor = (ra + rp) / 2
    eccentricity = (ra - rp) / (ra + rp)
    if not 0 < eccentricity < 1:
        return None
    phase = rotation_phase + rotation_rate * ut
    raan = math.radians(raan_deg) + phase
    argument = math.radians(argument_of_periapsis_deg)
    ci, si = math.cos(math.radians(inclination_deg)), math.sin(math.radians(inclination_deg))
    co, so = math.cos(raan), math.sin(raan)
    cw, sw = math.cos(argument), math.sin(argument)
    periapsis_axis = [co * cw - so * sw * ci,
                      so * cw + co * sw * ci, sw * si]
    transverse_axis = [-co * sw - so * cw * ci,
                       -so * sw + co * cw * ci, cw * si]
    post_speed = math.sqrt(mu * (2 / ra - 1 / semimajor))
    position = _scale(periapsis_axis, -ra)
    velocity = _scale(transverse_axis, -post_speed)
    h = _cross(position, velocity)
    h_norm = _norm(h)
    parameter = h_norm * h_norm / mu
    eccentricity_vector = _scale(
        _add(_scale(position, post_speed * post_speed - mu / ra),
             _scale(velocity, -sum(x * y for x, y in zip(position, velocity)))),
        1 / mu)
    e_norm = _norm(eccentricity_vector)
    if e_norm < 1e-9:
        return None
    periapsis_axis = _scale(eccentricity_vector, 1 / e_norm)
    transverse_axis = _scale(_cross(h, periapsis_axis), 1 / h_norm)
    entry_radius = radius + atmosphere_top
    cosine_true_anomaly = (parameter / entry_radius - 1) / e_norm
    if not -1 <= cosine_true_anomaly <= 1:
        return None
    true_anomaly = -math.acos(max(-1.0, min(1.0, cosine_true_anomaly)))
    entry_position = _scale(
        _add(_scale(periapsis_axis, math.cos(true_anomaly)),
             _scale(transverse_axis, math.sin(true_anomaly))), entry_radius)
    entry_velocity = _scale(
        _add(_scale(periapsis_axis, -math.sin(true_anomaly)),
             _scale(transverse_axis, e_norm + math.cos(true_anomaly))),
        math.sqrt(mu / parameter))
    up = _scale(entry_position, 1 / _norm(entry_position))
    radial_speed = sum(x * y for x, y in zip(entry_velocity, up))
    horizontal_speed = math.sqrt(max(0.0, _norm(entry_velocity) ** 2 - radial_speed ** 2))
    entry_fpa = math.degrees(math.atan2(radial_speed, horizontal_speed))
    entry_speed = _norm(entry_velocity)
    eccentric_anomaly = 2 * math.atan2(
        math.sqrt(1 - eccentricity) * math.sin(true_anomaly / 2),
        math.sqrt(1 + eccentricity) * math.cos(true_anomaly / 2))
    mean_anomaly = eccentric_anomaly - eccentricity * math.sin(eccentric_anomaly)
    delta_mean_anomaly = (mean_anomaly - math.pi) % (2 * math.pi)
    time_to_entry = delta_mean_anomaly / math.sqrt(mu / semimajor ** 3)
    entry_ut = ut + time_to_entry
    entry_fixed = _rotate_z(
        entry_position, -(rotation_phase + rotation_rate * entry_ut))
    entry_latitude = math.degrees(math.asin(entry_fixed[2] / entry_radius))
    entry_longitude = math.degrees(math.atan2(entry_fixed[1], entry_fixed[0]))
    lat1, lat2 = math.radians(runway["latitude_deg"]), math.radians(entry_latitude)
    dlon = math.radians(_wrap_degrees(entry_longitude - runway["longitude_deg"]))
    cos_delta = math.sin(lat1) * math.sin(lat2) + math.cos(lat1) * math.cos(lat2) * math.cos(dlon)
    delta = math.acos(max(-1.0, min(1.0, cos_delta)))
    bearing = math.atan2(math.sin(dlon) * math.cos(lat2),
                         math.cos(lat1) * math.sin(lat2) -
                         math.sin(lat1) * math.cos(lat2) * math.cos(dlon))
    relative_bearing = bearing - math.radians(runway["heading_deg"])
    cross = math.asin(max(-1.0, min(1.0, math.sin(delta) * math.sin(relative_bearing)))) * radius
    along = math.atan2(math.sin(delta) * math.cos(relative_bearing), math.cos(delta)) * radius
    return {
        "entry_altitude_m": atmosphere_top,
        "entry_fpa_deg": entry_fpa,
        "entry_speed_mps": entry_speed,
        "time_to_entry_s": time_to_entry,
        "entry_along_m": along,
        "entry_cross_m": cross,
        "entry_range_m": delta * radius,
        "entry_latitude_deg": entry_latitude,
        "entry_longitude_deg": entry_longitude,
    }


def deorbit_condition_feasible(condition, ut, runway, world):
    """Accept an orbit when it physically produces a future descending entry.

    Reachability to TAEM is intentionally not guessed here.  The atmospheric
    simulator and production handoff/path envelopes own that decision.
    """
    prediction = deorbit_entry_screen(
        condition["apoapsis_altitude_m"], condition["target_post_deorbit_periapsis_altitude_m"],
        condition["inclination_deg"], condition["raan_deg"],
        condition["argument_of_periapsis_deg"], ut, runway, world)
    if prediction is None:
        return False, None
    required = ("entry_fpa_deg", "entry_speed_mps", "time_to_entry_s",
                "entry_along_m", "entry_cross_m", "entry_range_m")
    if not all(math.isfinite(prediction[k]) for k in required):
        return False, prediction
    dv = _finite(condition.get("deorbit_delta_v_mps"), float("nan"))
    physical_entry = (prediction["entry_fpa_deg"] < 0.0
                      and prediction["entry_speed_mps"] > 0.0
                      and prediction["time_to_entry_s"] > 0.0
                      and math.isfinite(dv) and dv > 0.0)
    return physical_entry, prediction


def _project_axis(action, previous, lower, upper, response_time):
    """Map a normalized residual into currently available command headroom.

    The maximum one-policy-step change is the full admissible command span
    divided by the production control-response time.  This keeps the learner's
    target motion tied to modeled vehicle response rather than a second set of
    hand-tuned residual amplitudes or slew rates.
    """
    if not all(math.isfinite(x) for x in (action, previous, lower, upper, response_time)):
        raise ValueError("invalid authority envelope")
    if response_time <= 0.0 or lower > upper:
        raise ValueError("invalid authority envelope")
    unit = max(-1.0, min(1.0, action))
    # The expert command is allowed to arrive outside the learned residual
    # interval (for example, an AoA reference can be below a newly active
    # low-Q floor).  Zero residual must then move to the nearest admissible
    # command instead of rejecting the whole policy step.  The old
    # lower<=0<=upper assumption turned this ordinary one-sided state into a
    # deterministic gate, making the simulator RL path unable to respond.
    anchor = max(lower, min(upper, 0.0))
    desired = (anchor + unit * (upper - anchor) if unit >= 0.0 else
               anchor + unit * (anchor - lower))
    max_step = (upper - lower) / response_time
    if max_step <= 0.0:
        return anchor
    return max(lower, min(upper, max(previous - max_step, min(previous + max_step, desired))))


def project(action, expert, previous):
    """Project policy residuals through the production authority envelope."""
    reason = "projected"
    try:
        valid = len(action) == 2 and all(math.isfinite(x) for x in action)
    except (TypeError, ValueError):
        valid = False
    response_time = _finite(expert.get("authority_response_time_s"), float("nan"))
    authority_ready = (bool(expert.get("authority_valid"))
                       and bool(expert.get("authority_survivable"))
                       and bool(expert.get("authority_controllable"))
                       and math.isfinite(response_time) and response_time > 0.0)
    if not valid:
        residual, reason = [0.0, 0.0], "invalid_policy"
    elif not expert["enabled"] or expert["abort"] or not authority_ready:
        residual, reason = [0.0, 0.0], "deterministic_gate"
    else:
        # The deterministic expert may intentionally publish an emergency
        # reference outside the ordinary residual envelope (for example, a
        # terminal energy-preserving AoA below the entry thermal floor). Keep
        # that approved baseline available to the environment; the residual
        # may move only into the physical interval, never farther outside it.
        aoa_lower = min(0.0, expert["aoa_min"] - expert["aoa"])
        aoa_upper = max(0.0, expert["aoa_max"] - expert["aoa"])
        if expert["bank"] > 0.0:
            bank_lower = -expert["bank"]
            bank_upper = expert["bank_max"] - expert["bank"]
        elif expert["bank"] < 0.0:
            bank_lower = -expert["bank_max"] - expert["bank"]
            bank_upper = -expert["bank"]
        else:
            bank_lower = bank_upper = 0.0
        try:
            residual = [
                _project_axis(action[0], previous[0], aoa_lower, aoa_upper, response_time),
                _project_axis(action[1], previous[1], bank_lower, bank_upper, response_time),
            ]
        except (TypeError, ValueError):
            residual, reason = [0.0, 0.0], "deterministic_gate"
    aoa, bank = expert["aoa"] + residual[0], expert["bank"] + residual[1]
    if reason == "projected":
        if expert["aoa"] < expert["aoa_min"]:
            aoa = max(expert["aoa"], min(expert["aoa_max"], aoa))
        elif expert["aoa"] > expert["aoa_max"]:
            aoa = min(expert["aoa"], max(expert["aoa_min"], aoa))
        else:
            aoa = max(expert["aoa_min"], min(expert["aoa_max"], aoa))
        if expert["bank"] > expert["bank_max"]:
            bank = min(expert["bank"], expert["bank_max"])
        elif expert["bank"] < -expert["bank_max"]:
            bank = max(expert["bank"], -expert["bank_max"])
        else:
            bank = max(-expert["bank_max"], min(expert["bank_max"], bank))
        if bank * expert["bank"] < 0.0:
            bank = expert["bank"]  # No learned reversal/commitment override.
    return [aoa, bank], [aoa - expert["aoa"], bank - expert["bank"]], reason


class Environment:
    def __init__(self, *, simulator_only=False, scenario="ksp86km-postburn.ini",
                 horizon=3600, log=None, randomized=True, stress=None,
                 curriculum=None):
        if not simulator_only:
            raise ValueError("explicit simulator_only=True required")
        if horizon <= 0:
            raise ValueError("positive horizon required")
        if stress not in (None, "density-low", "lag-slow", "mass-high"):
            raise ValueError("unknown stress case")
        if curriculum not in (None, "full", "entry-corridor", "interface-corridor", "terminal-corridor"):
            raise ValueError("unknown curriculum case")
        self.stress = stress
        self.curriculum = None if curriculum in (None, "full") else curriculum
        self.scenario = SIM / "scenarios" / scenario
        self.runway = {"latitude_deg": -0.0486111111,
                       "longitude_deg": -74.7283333333,
                       "heading_deg": 90.0}
        self.horizon, self.log, self.randomized = horizon, Path(log) if log else None, randomized
        suffix = ".dylib" if sys.platform == "darwin" else ".so"
        self.lib = C.CDLL(str(SIM / "build-offline" / ("libshuttlesim_offline" + suffix)))
        self.teacher = C.CDLL(str(SIM / "build-offline" / ("libshuttlesim_expert" + suffix)))
        signatures = [
            (self.lib, "offline_create", [C.c_char_p] * 4, C.c_void_p),
            (self.lib, "offline_destroy", [C.c_void_p], None),
            (self.lib, "offline_step", [C.c_void_p, C.c_double, C.c_double, C.c_int, C.c_int, C.c_int], C.c_int),
            (self.lib, "offline_randomize", [C.c_void_p] + [C.c_double] * 6, C.c_int),
            (self.lib, "offline_set_initial_conditions",
             [C.c_void_p] + [C.c_double] * 6, C.c_int),
            (self.lib, "offline_set_deorbit_orbit",
             [C.c_void_p] + [C.c_double] * 6, C.c_int),
            (self.lib, "offline_load_book", [C.c_void_p, C.c_char_p], C.c_int),
            (self.lib, "offline_telemetry", [C.c_void_p], C.c_char_p),
            (self.lib, "offline_summary", [C.c_void_p], C.c_char_p),
            (self.teacher, "expert_create", [C.c_char_p, C.c_char_p], C.c_void_p),
            (self.teacher, "expert_destroy", [C.c_void_p], None),
            (self.teacher, "expert_command", [C.c_void_p, C.c_char_p], C.c_char_p),
        ]
        for lib, name, args, result in signatures:
            fn = getattr(lib, name)
            fn.argtypes, fn.restype = args, result
        self.handle = self.expert_handle = None
        self.done = True

    def close(self):
        if self.handle:
            self.lib.offline_destroy(self.handle)
        if self.expert_handle:
            self.teacher.expert_destroy(self.expert_handle)
        self.handle = self.expert_handle = None
        self.done = True

    def _log(self, record):
        if self.log:
            self.log.parent.mkdir(parents=True, exist_ok=True)
            with self.log.open("a") as f:
                f.write(json.dumps(record, allow_nan=False) + "\n")

    def reset(self, seed=0, policy_version="deterministic"):
        self.close()
        self.seed, self.policy_version = seed, policy_version
        rng = random.Random(seed)
        paths = [self.scenario, SIM / "data/fitted/kerbin_atmosphere_ksp.csv",
                 SIM / "data/fitted/stsn_aero_ksp_robust.csv", SIM / "data/fitted/stsn_attitude_ksp.ini"]
        self.handle = self.lib.offline_create(*(str(p).encode() for p in paths))
        self.expert_handle = self.teacher.expert_create(
            str(ROOT / "Configuration/default.json").encode(), str(paths[1]).encode())
        if not self.handle or not self.expert_handle:
            self.close()
            raise RuntimeError("model/scenario/config load failed")
        if not self.lib.offline_load_book(self.handle, str(SIM / "data/fitted/stsn_force_book.csv").encode()):
            self.close()
            raise RuntimeError("force book load failed")
        nominal = json.loads(self.lib.offline_telemetry(self.handle))
        world = _world_model(nominal.get("world"))
        self.world = world
        if self.curriculum:
            self.initial_condition = curriculum_initial_condition(
                self.curriculum, rng, seed, self.runway, nominal, world)
            if not self.lib.offline_set_initial_conditions(
                    self.handle, self.initial_condition["altitude_m"],
                    self.initial_condition["speed_mps"],
                    self.initial_condition["flight_path_angle_deg"],
                    self.initial_condition["heading_deg"],
                    self.initial_condition["latitude_deg"],
                    self.initial_condition["longitude_deg"]):
                self.close()
                raise RuntimeError("invalid curriculum initial condition")
        elif self.randomized:
            # Sampling coverage is separate from feasibility.  Construct a valid
            # vacuum pre-orbit and a post-burn periapsis that intersects the
            # simulator's own atmosphere; downstream guidance decides reachability.
            atmosphere_top = world["atmosphere_top_m"]
            coverage_top = max(DEORBIT_COVERAGE_MAX_ALTITUDE_M,
                               math.nextafter(atmosphere_top, math.inf))
            apoapsis = rng.uniform(math.nextafter(atmosphere_top, math.inf), coverage_top)
            periapsis = rng.uniform(atmosphere_top, apoapsis)
            target_periapsis = rng.uniform(
                0.0, math.nextafter(atmosphere_top, 0.0))
            inclination_band = seed % 4
            inclination = ((inclination_band + rng.random()) / 4.0) * MAX_INITIAL_INCLINATION_DEG
            entry_true_anomaly = deorbit_entry_true_anomaly(
                apoapsis, target_periapsis, world)
            # These perturbations define experiment coverage, not a validity gate.
            argument = -math.degrees(entry_true_anomaly) + rng.uniform(-5.0, 5.0)
            anchor = deorbit_entry_screen(
                apoapsis, target_periapsis, inclination, 0.0, argument,
                nominal["ut"], self.runway, world)
            if anchor is None:
                self.close()
                raise RuntimeError("constructed deorbit orbit has no atmospheric crossing")
            target_entry_offset = rng.uniform(-18.0, -6.0)
            longitude = _wrap_degrees(
                self.runway["longitude_deg"] + target_entry_offset -
                anchor["entry_longitude_deg"])
            self.initial_condition = {
                "stage": "pre_deorbit",
                "apoapsis_altitude_m": apoapsis,
                "periapsis_altitude_m": periapsis,
                "inclination_deg": inclination,
                "raan_deg": longitude,
                "argument_of_periapsis_deg": argument,
                "target_post_deorbit_periapsis_altitude_m": target_periapsis,
                "deorbit_delta_v_mps": deorbit_delta_v(
                    apoapsis, periapsis, target_periapsis, world),
            }
            feasible, screen_prediction = deorbit_condition_feasible(
                self.initial_condition, nominal["ut"], self.runway, world)
            if not feasible:
                self.close()
                raise RuntimeError("constructed deorbit condition violates orbital entry domain")
            self.initial_condition["mm304_taem_screen"] = screen_prediction
            if not self.lib.offline_set_deorbit_orbit(
                    self.handle, self.initial_condition["apoapsis_altitude_m"],
                    self.initial_condition["periapsis_altitude_m"],
                    self.initial_condition["inclination_deg"],
                    self.initial_condition["raan_deg"],
                    self.initial_condition["argument_of_periapsis_deg"],
                    self.initial_condition["target_post_deorbit_periapsis_altitude_m"]):
                self.close()
                raise RuntimeError("invalid randomized pre-deorbit condition")
        else:
            self.initial_condition = {
                "stage": "post_deorbit",
                "altitude_m": nominal["position"]["altitude_m"],
                "speed_mps": nominal["velocity"]["surface_mps"],
                "flight_path_angle_deg": nominal["velocity"]["flight_path_angle_deg"],
                "heading_deg": nominal["attitude"]["heading_deg"],
                "latitude_deg": nominal["position"]["lat_deg"],
                "longitude_deg": nominal["position"]["lon_deg"],
                "energy_case": "nominal",
            }
        scales = [rng.uniform(1-r, 1+r) for r in (.02, .05, .05, .05, .05)] + [rng.uniform(-1, 1)]
        self.scales = scales if self.randomized else [1, 1, 1, 1, 1, 0]
        if self.stress:
            index, value = {"density-low": (1, .9), "lag-slow": (4, .9),
                            "mass-high": (0, 1.05)}[self.stress]
            self.scales[index] = value
        if not self.lib.offline_randomize(self.handle, *self.scales):
            raise RuntimeError("invalid randomization")
        self.done, self.elapsed, self.total_reward = False, 0, 0.0
        self.handoff_seen, self.first_handoff_elapsed = False, None
        self.handoff_event_seen = False
        self.terminal_path_candidate_seen = False
        self.terminal_path_committed_seen = False
        self.residual = [0.0, 0.0]
        self.phase, self.phase_time = None, 0
        self.active_steps = self.fallbacks = self.eligible_steps = 0
        self.max_q = self.max_g = 0.0
        self.phases = {}
        self.previous_potential = None
        self._sample()
        initial_terminal_path = self._record_terminal_path_contract()
        self.initial_terminal_path = initial_terminal_path
        initial_debt = taem_interface_debt(self.telemetry, self.expert)
        initial_unsafe = bool(
            self.expert["abort"] or
            not self.expert.get("authority_valid", False) or
            not self.expert.get("authority_survivable", False))
        self.initial_handoff = bool(
            not initial_unsafe and
            self.expert["phase"] in ("ENTRY_ENERGY", "TAEM") and
            initial_debt.get("capture_ready") and
            int(initial_debt.get("capture_veto", 1)) == 0)
        # A terminal curriculum may intentionally begin after MM304 has
        # already met its interface contract. Do not wait for a physics tick
        # to move the vehicle out of the 1 km set and then call that a missed
        # handoff. Interface-only episodes retain a pending event so their
        # first step can terminate without advancing the simulator.
        self.initial_handoff_pending = bool(
            self.initial_handoff and self.curriculum == "interface-corridor")
        if self.initial_handoff and not self.initial_handoff_pending:
            self.handoff_seen = True
            self.first_handoff_elapsed = 0
        self.initial_condition["realized_post_deorbit"] = {
            "altitude_m": self.telemetry["position"]["altitude_m"],
            "speed_mps": self.telemetry["velocity"]["surface_mps"],
            "flight_path_angle_deg": self.telemetry["velocity"]["flight_path_angle_deg"],
            "heading_deg": self.telemetry["attitude"]["heading_deg"],
            "latitude_deg": self.telemetry["position"]["lat_deg"],
            "longitude_deg": self.telemetry["position"]["lon_deg"],
        }
        self.initial_condition["post_deorbit_orbital_elements"] = orbital_elements(self.telemetry)
        self.previous_potential = self._potential()
        self._log({"type": "reset", "schema": SCHEMA, "seed": seed, "scenario": str(self.scenario),
                   "policy": policy_version, "randomization": self.scales,
                   "initial_condition": self.initial_condition, "stress": self.stress,
                   "curriculum": self.curriculum or "full",
                   "initial_handoff": self.initial_handoff,
                   "initial_handoff_debt": initial_debt,
                   "initial_terminal_path": initial_terminal_path})
        return self.observation(), {"schema": SCHEMA, "seed": seed,
                                    "initial_condition": self.initial_condition,
                                    "initial_handoff": self.initial_handoff,
                                    "initial_handoff_debt": initial_debt,
                                    "initial_terminal_path": initial_terminal_path,
                                    "initial_terminal_path_ready": initial_terminal_path["path_ready"],
                                    "initial_terminal_candidate_ready": initial_terminal_path["candidate_ready"]}

    def _sample(self):
        packet = self.lib.offline_telemetry(self.handle)
        self.telemetry = json.loads(packet)
        result = self.teacher.expert_command(self.expert_handle, packet)
        if not result:
            raise RuntimeError("invalid teacher telemetry")
        self.expert = json.loads(result)
        if self.expert["phase"] != self.phase:
            self.phase, self.phase_time = self.expert["phase"], self.elapsed

    def _record_terminal_path_contract(self):
        contract = terminal_path_contract(self.expert)
        self.terminal_path_candidate_seen |= contract["candidate_ready"]
        self.terminal_path_committed_seen |= contract["path_committed"]
        return contract

    def observation(self):
        t, e = self.telemetry, self.expert
        p, v, a, rw, ground = (t[k] for k in ("position", "velocity", "attitude", "runway", "ground"))
        world = _world_model(t.get("world"))
        body_radius, mu = world["radius_m"], world["mu_m3_s2"]
        radial_distance = body_radius + p["altitude_m"]
        orbital_speed = math.sqrt(mu / radial_distance)
        local_g = mu / (radial_distance * radial_distance)

        phase_max = _finite(e.get("phase_domain_max"), float("nan"))
        aoa_scale = max(abs(_finite(e["aoa_min"])), abs(_finite(e["aoa_max"])))
        bank_scale = abs(_finite(e["bank_max"]))
        horizontal_speed = abs(_finite(v["horizontal_mps"]))
        bank_for_turn = min(bank_scale, math.nextafter(90.0, 0.0))
        course_rate_scale = (math.degrees(local_g * math.tan(math.radians(bank_for_turn)) / horizontal_speed)
                             if horizontal_speed > 0.0 and bank_for_turn > 0.0 else 0.0)
        course_rate = _finite(e["course_rate"])
        course_rate_fraction = _ratio(course_rate, course_rate_scale, "course rate")

        capture_valid = bool(e.get("taem_capture_valid"))
        if capture_valid:
            capture_along = _finite(e["taem_capture_along_m"], float("nan"))
            capture_cross = _finite(e["taem_capture_cross_m"], float("nan"))
            capture_altitude = _finite(e["taem_capture_altitude_error_m"], float("nan"))
            capture_energy = _finite(e["taem_capture_energy_margin"], float("nan"))
            capture_turn = _finite(e["taem_capture_turn_margin"], float("nan"))
            energy_scale = max(abs(_finite(e.get("taem_energy_available"))),
                               abs(_finite(e.get("taem_energy_drag_work"))),
                               abs(_finite(e.get("taem_energy_uncertainty"))))
            capture_energy_fraction = (_ratio(capture_energy, energy_scale, "capture energy")
                                       if energy_scale > 0.0 else 0.0 if capture_energy == 0.0
                                       else (_ for _ in ()).throw(ValueError("energy margin without energy scale")))
        else:
            capture_along = capture_cross = capture_altitude = capture_turn = 0.0
            capture_energy_fraction = 0.0

        values = [
            _ratio(e["phase_id"], phase_max, "phase domain"),
            p["altitude_m"] / body_radius, max(0.0, rw["vertical_m"]) / body_radius,
            v["air_mps"] / orbital_speed, v["horizontal_mps"] / orbital_speed,
            v["vertical_mps"] / orbital_speed, t["aero"]["mach"],
            _ratio(t["aero"]["q_pa"], e["q_max"], "dynamic pressure"),
            v["flight_path_angle_deg"] / 90.0,
            _ratio(a["aoa_deg"], aoa_scale, "angle of attack"),
            _ratio(a["bank_deg"], bank_scale, "bank angle"),
            _ratio(a["aoa_rate_deg_s"], a["max_pitch_rate_deg_s"], "AoA rate"),
            _ratio(a["bank_rate_deg_s"], a["max_roll_rate_deg_s"], "bank rate"),
            course_rate_fraction, rw["along_m"] / body_radius, rw["cross_m"] / body_radius,
            e["range"] / body_radius, e["bearing"] / 180.0, e["heading_error"] / 180.0,
            taem_course_signed_error_deg(t) / 180.0, e["energy_error"] / body_radius,
            float(bool(ground["gear_down"])), float(bool(ground["brakes"])),
            _ratio(e["aoa"], aoa_scale, "expert AoA"),
            _ratio(e["bank"], bank_scale, "expert bank"), float(bool(e["enabled"])),
            _finite(e["authority_speed_normalized_margin"], float("nan")),
            _finite(e["authority_q_normalized_margin"], float("nan")),
            _finite(e["authority_load_normalized_margin"], float("nan")),
            _finite(e["authority_stall_normalized_margin"], float("nan")),
            capture_along / body_radius, capture_cross / body_radius,
            capture_altitude / body_radius, capture_energy_fraction, capture_turn / body_radius,
            float(capture_valid), float(bool(e.get("taem_capture_ready"))),
        ]
        if len(values) != len(FEATURES) or not all(math.isfinite(x) for x in values):
            raise ValueError("invalid dimensionless observation")
        return values

    def _potential(self):
        """Contract potential measured only by satisfied production constraints."""
        violations = taem_interface_debt(self.telemetry, self.expert).get("constraint_violations")
        return None if violations is None else -float(violations)

    def step(self, action):
        if self.done:
            raise RuntimeError("reset required before step")
        try:
            policy_requested = len(action) == 2 and all(math.isfinite(x) for x in action) and any(abs(x) > 1e-9 for x in action)
        except (TypeError, ValueError):
            policy_requested = False
        command, residual, reason = project(action, self.expert, self.residual)
        prior = self.residual
        self.residual = residual
        self.eligible_steps += int(self.expert["enabled"])
        # Count learned policy authority, not deterministic safety projection
        # clamps such as AoA floor enforcement on an all-zero policy.
        self.active_steps += int(reason == "projected" and policy_requested)
        self.fallbacks += int(reason == "invalid_policy")
        e = self.expert
        prior_expert_command = [e["aoa"], e["bank"]]
        initial_handoff_event = bool(getattr(self, "initial_handoff_pending", False))
        if initial_handoff_event:
            self.initial_handoff_pending = False
        # Abort is authoritative: do not integrate another physics tick.
        if not e["abort"] and not initial_handoff_event:
            if not self.lib.offline_step(self.handle, *command, e["gear"], e["brakes"], 50):
                raise RuntimeError("physics step rejected")
            self.elapsed += 1
            self._sample()
        t, e = self.telemetry, self.expert
        terminal_path = self._record_terminal_path_contract()
        summary = json.loads(self.lib.offline_summary(self.handle))
        ground, velocity = t["ground"], t["velocity"]
        # Additional conservative guards; cannot turn an abort into a success.
        g_proxy = math.hypot(t["aero"]["lift_n"], t["aero"]["drag_n"]) / (t["vehicle"]["mass_kg"] * 9.81)
        self.max_q = max(self.max_q, t["aero"]["q_pa"])
        self.max_g = max(self.max_g, g_proxy)
        self.phases[e["phase"]] = self.phases.get(e["phase"], 0) + 1
        unsafe_reasons = []
        if e["abort"]:
            unsafe_reasons.append("guidance_abort")
        if not e.get("authority_valid", False):
            unsafe_reasons.append("authority_invalid")
        else:
            for name, key in (("speed", "authority_speed_margin"),
                              ("dynamic_pressure", "authority_q_margin"),
                              ("g_load", "authority_load_margin"),
                              ("stall", "authority_stall_margin")):
                margin = _finite(e.get(key), float("nan"))
                if not math.isfinite(margin) or margin < 0.0:
                    unsafe_reasons.append(name)
            if not e.get("authority_survivable", False) and not unsafe_reasons:
                unsafe_reasons.append("authority_unsurvivable")
        unsafe = bool(unsafe_reasons)
        potential = self._potential()
        progress = (potential - self.previous_potential
                    if potential is not None and self.previous_potential is not None else 0.0)
        if potential is not None:
            self.previous_potential = potential
        taem_debt = taem_interface_debt(t, e)
        handoff_contract = (
            not unsafe
            and e["phase"] in ("ENTRY_ENERGY", "TAEM")
            and bool(taem_debt.get("capture_ready"))
            and int(taem_debt.get("capture_veto", 1)) == 0
        )
        handoff_event = bool(initial_handoff_event or
                             (handoff_contract and not self.handoff_seen))
        if handoff_event:
            self.handoff_seen = True
            self.first_handoff_elapsed = self.elapsed
            self.handoff_event_seen = True
        # Interface cases may terminate at the handoff to keep an isolated signal.
        # Terminal cases continue past handoff so the learner can see TAEM/final
        # consequences instead of always ending with a fake success.
        handoff_terminal = bool(handoff_event and self.curriculum == "interface-corridor")
        success, miss = touchdown_outcome(summary, t, e, unsafe)
        terminal = bool(unsafe or success or miss or handoff_terminal)
        truncated = self.elapsed >= self.horizon and not terminal
        # Reward is deliberately expressed in contract events/counts only.  The
        # learner is not given a second altitude, speed, energy or timing model
        # that could teach it to compensate for a broken deterministic expert.
        terms = {
            "contract_progress": progress if not self.handoff_seen or handoff_event else 0.0,
            "handoff": 1.0 if handoff_event else 0.0,
            "terminal": 1.0 if success else -1.0 if (unsafe or miss) else 0.0,
        }
        reward = sum(terms.values())
        self.total_reward += reward
        self.done = terminal or truncated
        outcome = "success" if success else "handoff_ready" if handoff_terminal else "unsafe_or_abort" if unsafe else "runway_miss" if miss else "timeout" if truncated else "running"
        info = {"type": "step", "schema": SCHEMA, "policy": self.policy_version, "seed": self.seed,
                "elapsed": self.elapsed, "phase": e["phase"], "command": command,
                "residual": residual, "projection": reason, "reward_terms": terms,
                "taem_interface_debt": taem_debt,
                "terminal_path": terminal_path,
                "terminal_path_candidate_ready": terminal_path["candidate_ready"],
                "terminal_path_committed": terminal_path["path_committed"],
                "curriculum_handoff": self.handoff_seen,
                "handoff_event": handoff_event,
                "handoff_event_seen": self.handoff_event_seen,
                "first_handoff_elapsed": self.first_handoff_elapsed,
                "expert_command": prior_expert_command}
        # Nonfinite requested actions cannot be written as JSON numbers.
        info["action"] = action if reason != "invalid_policy" else "invalid"
        self._log(info)
        if self.done:
            info.update({"type": "episode", "outcome": outcome, "success": success,
                         "return": self.total_reward, "metrics": summary, "active_steps": self.active_steps,
                         "fallbacks": self.fallbacks, "eligible_steps": self.eligible_steps,
                         "max_q_pa": self.max_q, "max_g": self.max_g, "phases": self.phases,
                         "stress": self.stress, "randomization": self.scales,
                         "curriculum": self.curriculum or "full",
                         "stage": self.initial_condition.get("stage"),
                         "unsafe_reasons": unsafe_reasons,
                         "curriculum_handoff": self.handoff_seen,
                         "handoff_event": handoff_event,
                         "handoff_event_seen": self.handoff_event_seen,
                         "first_handoff_elapsed": self.first_handoff_elapsed,
                         "initial_condition": self.initial_condition,
                         "scenario": self.scenario.name,
                         "final_phase": e["phase"],
                         "final_altitude_m": t["position"]["altitude_m"],
                         "final_along_m": t["runway"]["along_m"],
                         "final_cross_m": t["runway"]["cross_m"],
                         "final_air_speed_mps": velocity["air_mps"],
                         "final_speed": velocity["surface_mps"],
                         "final_course_error_deg": taem_debt["course_error_deg"],
                         "final_guidance_course_error_deg": e["course_error"],
                         "final_energy_error": e["energy_error"],
                         "taem_interface_debt": taem_debt,
                         "initial_terminal_path": self.initial_terminal_path,
                         "final_terminal_path": terminal_path,
                         "terminal_path_candidate_seen": self.terminal_path_candidate_seen,
                         "terminal_path_committed": self.terminal_path_committed_seen,
                         "terminal_path_ready": self.terminal_path_committed_seen,
                         "terminal_path_blockers": terminal_path["blockers"],
                         "rollout_valid": success})
            self._log(info)
        return self.observation(), reward, terminal, truncated, info


def touchdown_outcome(summary, telemetry, expert, unsafe):
    """Validate touchdown/rollout against shared geometry and configured targets."""
    ground, rw = telemetry["ground"], telemetry["runway"]
    length = _finite(rw.get("length_m"), float("nan"))
    width = _finite(rw.get("width_m"), float("nan"))
    touchdown_speed = _finite(expert.get("touchdown_speed_mps"), float("nan"))
    touchdown_sink = _finite(expert.get("touchdown_sink_rate_mps"), float("nan"))
    contract_valid = (math.isfinite(length) and length > 0.0
                      and math.isfinite(width) and width > 0.0
                      and math.isfinite(touchdown_speed) and touchdown_speed > 0.0
                      and math.isfinite(touchdown_sink) and touchdown_sink < 0.0)
    if not contract_valid:
        return False, bool(summary.get("touchdown"))
    half_width = width * 0.5
    maximum_sink = -touchdown_sink
    contact = bool(summary["touchdown"])
    valid_contact = (contact and summary["on_runway"] and ground["gear_down"]
                     and abs(summary["touchdown_cross_m"]) <= half_width
                     and 0.0 <= summary["touchdown_along_m"] <= length
                     and 0.0 <= summary["touchdown_sink_mps"] <= maximum_sink
                     and 0.0 <= summary["touchdown_speed_mps"] <= touchdown_speed)
    on_strip = (abs(rw["cross_m"]) <= half_width
                and 0.0 <= rw["along_m"] <= length)
    success = bool(not unsafe and valid_contact and on_strip and ground["on_ground"]
                   and ground.get("stopped", False)
                   and expert["phase"] == "COMPLETE")
    miss = bool(contact and (not valid_contact or not on_strip))
    return success, miss
