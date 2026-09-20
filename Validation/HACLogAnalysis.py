#!/usr/bin/env python3
"""Print integration metrics from HAC flight-log snapshots."""

from __future__ import annotations

import argparse
import json
import math
import re
import statistics
import sys
from pathlib import Path


RADIAL_RE = re.compile(r"radial\s+([+-]?\d+(?:\.\d+)?)\s*km")
PATH_RE = re.compile(r"(?:join\+arc|arc)\s+([0-9.]+)\s*km")


def finite(value):
    try:
        number = float(value)
    except (TypeError, ValueError):
        return None
    return number if math.isfinite(number) else None


def signed_angle(value):
    value %= 360.0
    return value - 360.0 if value > 180.0 else value


def angle_difference(a, b):
    return signed_angle(a - b)


def percentile(values, fraction):
    values = sorted(values)
    if not values:
        return math.nan
    pos = (len(values) - 1) * fraction
    lo = int(pos)
    hi = min(lo + 1, len(values) - 1)
    blend = pos - lo
    return values[lo] * (1.0 - blend) + values[hi] * blend


def summary(values):
    if not values:
        return "n/a"
    absolute = [abs(value) for value in values]
    return (
        f"min={min(values):.3f} med={statistics.median(values):.3f} max={max(values):.3f} "
        f"|abs|med={statistics.median(absolute):.3f} p95={percentile(absolute, .95):.3f} "
        f"max={max(absolute):.3f}"
    )


def sign_changes(values, deadband):
    signs = []
    for value in values:
        if abs(value) <= deadband:
            continue
        sign = 1 if value > 0 else -1
        if not signs or signs[-1] != sign:
            signs.append(sign)
    return max(0, len(signs) - 1)


def increasing_steps(values, tolerance):
    return sum(
        1 for previous, current in zip(values, values[1:])
        if current > previous + tolerance
    )


def decreasing_steps(values, tolerance):
    return sum(
        1 for previous, current in zip(values, values[1:])
        if current < previous - tolerance
    )


def iter_snapshots(path):
    with path.open("r", encoding="utf-8") as handle:
        for line in handle:
            try:
                snapshot = json.loads(line)
            except json.JSONDecodeError:
                continue
            if snapshot.get("recordType") == "snapshot" and snapshot.get("automationEngaged"):
                yield snapshot


def _longest_saturation_run(rows, minimum_axes=2, threshold=.95, maximum_gap=.5):
    longest = 0.0
    started = previous_ut = None
    for row in rows:
        ut = row["ut"]
        controls = row["controls"]
        saturated = sum(value is not None and abs(value) >= threshold for value in controls)
        if saturated >= minimum_axes:
            if started is None or previous_ut is None or ut - previous_ut > maximum_gap:
                started = ut
            longest = max(longest, ut - started)
        else:
            started = None
        previous_ut = ut
    return longest


def orbital_capture_metrics(path):
    rows = []
    phases = []
    native_build = None
    try:
        with path.open("r", encoding="utf-8") as handle:
            header = json.loads(handle.readline())
        native_build = (header.get("runtime") or {}).get("nativeBuild")
    except (OSError, json.JSONDecodeError, AttributeError):
        pass

    for snapshot in iter_snapshots(path):
        command = snapshot.get("command") or {}
        if command.get("controlProfile") != "orbital" or not command.get("useInertialDirection"):
            continue
        telemetry = snapshot.get("telemetry") or {}
        ut = finite(telemetry.get("ut"))
        if ut is None:
            continue
        phase = str(snapshot.get("phase") or "unknown")
        if not phases or phases[-1] != phase:
            phases.append(phase)
        torque = tuple(finite(telemetry.get(key)) for key in (
            "availablePitchTorque", "availableRollTorque", "availableYawTorque"))
        inertia = tuple(finite(telemetry.get(key)) for key in (
            "pitchMomentOfInertia", "rollMomentOfInertia", "yawMomentOfInertia"))
        authority = tuple(
            None if t is None or i is None or i <= 0 else t / i * 180.0 / math.pi
            for t, i in zip(torque, inertia)
        )
        rows.append({
            "ut": ut,
            "phase": phase,
            "error": finite(telemetry.get("autopilotError")),
            "controls": (
                finite(telemetry.get("controlPitch")),
                finite(telemetry.get("controlRoll")),
                finite(telemetry.get("controlYaw")),
            ),
            "body_rates": (
                finite(telemetry.get("bodyPitchRate")),
                finite(telemetry.get("bodyRollRate")),
                finite(telemetry.get("bodyYawRate")),
            ),
            "authority": authority,
            "throttle": finite(command.get("targetThrottle")) or 0.0,
        })

    valid = [row for row in rows if row["error"] is not None and row["error"] < 179.5]
    if not valid:
        return {
            "rows": len(rows), "valid_rows": 0, "phases": phases, "native_build": native_build,
            "capture_start_ut": None, "capture_start_error": None, "final_error": None,
            "aligned_ut": None, "capture_seconds": math.inf,
            "max_multi_axis_saturation_seconds": 0.0, "zero_actuation_stall_seconds": 0.0,
            "actuation_delay_seconds": math.inf, "max_control_abs": 0.0, "max_body_rate_abs": 0.0,
            "median_authority_deg_s2": (math.nan, math.nan, math.nan),
            "unsafe_throttle_rows": 0, "largest_error_regression": math.nan,
            "post_align_max_error": math.nan, "throttle_started_ut": None,
        }

    first = valid[0]
    aligned = next((row for row in valid if row["ut"] >= first["ut"] and row["error"] <= 10.0), None)
    capture_rows = [
        row for row in valid
        if row["ut"] >= first["ut"] and (aligned is None or row["ut"] <= aligned["ut"])
    ]
    running_min = math.inf
    largest_regression = 0.0
    for row in capture_rows:
        running_min = min(running_min, row["error"])
        largest_regression = max(largest_regression, row["error"] - running_min)

    throttle_started = next((row for row in rows if row["throttle"] > 1e-4), None)
    unsafe_throttle = sum(
        1 for row in rows
        if row["throttle"] > 1e-4 and row["error"] is not None and row["error"] > 10.0
    )
    post_align_rows = []
    if aligned is not None:
        post_align_rows = [
            row for row in valid
            if row["ut"] >= aligned["ut"]
            and (throttle_started is None or row["ut"] <= throttle_started["ut"])
        ]

    actuation = next((
        row for row in valid
        if any(value is not None and abs(value) > .05 for value in row["controls"])
    ), None)
    actuation_delay = math.inf if actuation is None else max(0.0, actuation["ut"] - first["ut"])

    zero_actuation_stall = 0.0
    stall_started = previous_ut = None
    for row in valid:
        controls = [value for value in row["controls"] if value is not None]
        rates = [value for value in row["body_rates"] if value is not None]
        stalled = (
            row["error"] >= 30.0 and len(controls) == 3 and len(rates) == 3
            and max(abs(value) for value in controls) <= .05
            and max(abs(value) for value in rates) <= .10
        )
        if stalled:
            if stall_started is None or previous_ut is None or row["ut"] - previous_ut > .5:
                stall_started = row["ut"]
            zero_actuation_stall = max(zero_actuation_stall, row["ut"] - stall_started)
        else:
            stall_started = None
        previous_ut = row["ut"]

    controls = [abs(value) for row in valid for value in row["controls"] if value is not None]
    rates = [abs(value) for row in valid for value in row["body_rates"] if value is not None]
    axis_authority = [
        [row["authority"][axis] for row in valid if row["authority"][axis] is not None]
        for axis in range(3)
    ]
    median_authority = tuple(
        statistics.median(values) if values else math.nan for values in axis_authority
    )

    return {
        "rows": len(rows),
        "valid_rows": len(valid),
        "phases": phases,
        "native_build": native_build,
        "capture_start_ut": first["ut"],
        "capture_start_error": first["error"],
        "final_error": valid[-1]["error"],
        "aligned_ut": None if aligned is None else aligned["ut"],
        "capture_seconds": math.inf if aligned is None else aligned["ut"] - first["ut"],
        "max_multi_axis_saturation_seconds": _longest_saturation_run(capture_rows),
        "zero_actuation_stall_seconds": zero_actuation_stall,
        "actuation_delay_seconds": actuation_delay,
        "max_control_abs": max(controls, default=0.0),
        "max_body_rate_abs": max(rates, default=0.0),
        "median_authority_deg_s2": median_authority,
        "unsafe_throttle_rows": unsafe_throttle,
        "largest_error_regression": largest_regression,
        "post_align_max_error": max((row["error"] for row in post_align_rows), default=math.nan),
        "throttle_started_ut": None if throttle_started is None else throttle_started["ut"],
    }


def orbital_capture_failures(metrics):
    failures = []
    if metrics["rows"] == 0:
        failures.append("no engaged inertial orbital samples")
        return failures
    if not metrics.get("valid_rows", 0):
        failures.append("no usable orbital attitude-error samples below 179.5 deg")
        return failures
    if metrics["aligned_ut"] is None:
        failures.append("retrograde never reached the <=10 deg burn alignment gate")
    elif metrics["capture_seconds"] > 30.0:
        failures.append(f"retrograde acquisition took {metrics['capture_seconds']:.2f}s (>30s)")
    if metrics["max_multi_axis_saturation_seconds"] > .75:
        failures.append(
            "multi-axis |control|>=0.95 saturation lasted "
            f"{metrics['max_multi_axis_saturation_seconds']:.2f}s (>0.75s)"
        )
    if metrics["zero_actuation_stall_seconds"] > 5.0:
        failures.append(
            "high-error zero-actuation stall lasted "
            f"{metrics['zero_actuation_stall_seconds']:.2f}s (>5.00s)"
        )
    if metrics["unsafe_throttle_rows"]:
        failures.append(
            f"{metrics['unsafe_throttle_rows']} commanded-throttle samples occurred above 10 deg error"
        )
    if metrics["largest_error_regression"] > 15.0:
        failures.append(
            f"attitude error re-diverged by {metrics['largest_error_regression']:.2f} deg during acquisition"
        )
    post_align = metrics["post_align_max_error"]
    if math.isfinite(post_align) and post_align > 12.0:
        failures.append(f"attitude error rose back to {post_align:.2f} deg after capture")
    return failures


def print_orbital_capture(path):
    metrics = orbital_capture_metrics(path)
    failures = orbital_capture_failures(metrics)
    print(path)
    print("orbital_phases", " -> ".join(metrics["phases"]) if metrics["phases"] else "n/a")
    print("orbital_samples", metrics["rows"])
    print("capture_start_ut", metrics["capture_start_ut"] if metrics["capture_start_ut"] is not None else "n/a")
    print("capture_start_error_deg", metrics["capture_start_error"] if metrics["capture_start_error"] is not None else "n/a")
    print("aligned_ut", metrics["aligned_ut"] if metrics["aligned_ut"] is not None else "n/a")
    print("capture_to_10deg_s", f"{metrics['capture_seconds']:.2f}" if math.isfinite(metrics["capture_seconds"]) else "n/a")
    print("max_multi_axis_saturation_s", f"{metrics['max_multi_axis_saturation_seconds']:.2f}")
    print("native_build", metrics["native_build"] or "n/a")
    print("final_error_deg", f"{metrics['final_error']:.3f}" if metrics["final_error"] is not None else "n/a")
    print("zero_actuation_stall_s", f"{metrics['zero_actuation_stall_seconds']:.2f}")
    print("actuation_delay_s", f"{metrics['actuation_delay_seconds']:.2f}" if math.isfinite(metrics["actuation_delay_seconds"]) else "n/a")
    print("max_control_abs", f"{metrics['max_control_abs']:.3f}")
    print("max_body_rate_deg_s", f"{metrics['max_body_rate_abs']:.3f}")
    print("median_authority_deg_s2", " ".join(
        f"{value:.3f}" if math.isfinite(value) else "n/a"
        for value in metrics["median_authority_deg_s2"]
    ))
    print("largest_error_regression_deg", f"{metrics['largest_error_regression']:.2f}" if math.isfinite(metrics["largest_error_regression"]) else "n/a")
    print("post_align_max_error_deg", f"{metrics['post_align_max_error']:.2f}" if math.isfinite(metrics["post_align_max_error"]) else "n/a")
    print("throttle_started_ut", metrics["throttle_started_ut"] if metrics["throttle_started_ut"] is not None else "n/a")
    print("unsafe_throttle_rows", metrics["unsafe_throttle_rows"])
    print("orbital_acceptance", "PASS" if not failures else "FAIL")
    for failure in failures:
        print("  -", failure)
    return not failures


# Replay acceptance for TAEM/HAC transition behavior.
def terminal_transition_metrics(path):
    phases = []
    first_taem = None

    first_taem_speed = None
    first_taem_altitude = None
    first_taem_energy = None
    first_taem_exec_phase = None
    high_energy_acquisition_rows = 0
    taem_overspeed_rows = 0

    s_turn_airbrake_rows = 0
    s_turn_min_altitude = math.inf
    selected_ut = None
    selected_last_ut = None
    transition_seen = False
    transition_progress = []
    hac_captured_seen = False
    hac_completed_seen = False
    final_captured_seen = False

    for snapshot in iter_snapshots(path):
        telemetry = snapshot.get("telemetry") or {}
        guidance = snapshot.get("guidanceState") or {}
        ut = finite(telemetry.get("ut"))

        if ut is None:
            continue
        phase = str(snapshot.get("phase") or "unknown")
        if not phases or phases[-1] != phase:
            phases.append(phase)

        if phase == "TAEM":
            altitude = finite(telemetry.get("meanAltitude"))
            speed = finite(telemetry.get("trueAirSpeed"))
            energy = finite(telemetry.get("energyExcessRange"))
            taem_exec = guidance.get("taemExecutive") or {}
            exec_phase = str(taem_exec.get("phase") or "")

            if first_taem is None:
                first_taem = ut
                first_taem_speed = speed
                first_taem_altitude = altitude
                first_taem_energy = energy
                first_taem_exec_phase = exec_phase or None

            if speed is not None and speed > 1325.0:
                taem_overspeed_rows += 1
            if altitude is not None and altitude > 20000.0 and energy is not None and energy > 50000.0 and exec_phase and exec_phase != "S-turn":
                high_energy_acquisition_rows += 1

            if exec_phase == "S-turn":
                if altitude is not None:
                    s_turn_min_altitude = min(s_turn_min_altitude, altitude)
                command = snapshot.get("command") or {}
                if bool(command.get("airbrakes")):
                    s_turn_airbrake_rows += 1

        selected = bool(guidance.get("hacSideSelected"))
        if selected:
            if selected_ut is None:
                selected_ut = ut
            selected_last_ut = ut
        transition_seen |= bool(guidance.get("hacTransitionActive"))

        progress = finite(guidance.get("hacTransitionProgress"))
        if selected and progress is not None:
            transition_progress.append(progress)
        hac_captured_seen |= bool(guidance.get("hacCaptured"))
        hac_completed_seen |= bool(guidance.get("hacCompleted"))
        final_captured_seen |= bool(guidance.get("finalCaptured"))

    post_selected_seconds = 0.0
    if selected_ut is not None and selected_last_ut is not None:
        post_selected_seconds = max(0.0, selected_last_ut - selected_ut)
    min_s_turn_altitude = s_turn_min_altitude if math.isfinite(s_turn_min_altitude) else math.nan
    max_progress = max(transition_progress, default=math.nan)

    return {
        "phases": phases,
        "first_taem_ut": first_taem,
        "first_taem_speed": first_taem_speed,
        "first_taem_altitude": first_taem_altitude,
        "first_taem_energy": first_taem_energy,
        "first_taem_exec_phase": first_taem_exec_phase,
        "high_energy_acquisition_rows": high_energy_acquisition_rows,

        "taem_overspeed_rows": taem_overspeed_rows,
        "s_turn_airbrake_rows": s_turn_airbrake_rows,
        "s_turn_min_altitude": min_s_turn_altitude,
        "hac_selected_ut": selected_ut,
        "post_selected_seconds": post_selected_seconds,
        "transition_seen": transition_seen,
        "transition_max_progress": max_progress,
        "transition_backsteps": decreasing_steps(transition_progress, .02),

        "hac_captured_seen": hac_captured_seen,
        "hac_completed_seen": hac_completed_seen,
        "final_captured_seen": final_captured_seen,
    }


def terminal_transition_failures(metrics):
    failures = []
    if metrics["first_taem_ut"] is None:
        failures.append("no engaged TAEM snapshots")
        return failures

    if metrics["taem_overspeed_rows"]:
        failures.append(f"{metrics['taem_overspeed_rows']} TAEM samples exceeded the 1325 m/s handoff guard")
    if metrics["high_energy_acquisition_rows"]:
        failures.append(f"{metrics['high_energy_acquisition_rows']} high-altitude/high-energy TAEM samples bypassed S-turn ownership")
    if metrics["s_turn_airbrake_rows"]:
        failures.append(f"airbrakes were commanded during {metrics['s_turn_airbrake_rows']} TAEM S-turn samples")

    min_altitude = metrics["s_turn_min_altitude"]
    if math.isfinite(min_altitude) and min_altitude < 14000.0:
        failures.append(f"TAEM S-turn persisted down to {min_altitude:.0f} m (<14 km guard)")
    max_progress = metrics["transition_max_progress"]
    if metrics["hac_selected_ut"] is not None and metrics["transition_seen"] and metrics["post_selected_seconds"] > 10.0 and (not math.isfinite(max_progress) or max_progress < .05):
        failures.append(f"HAC transition was active for {metrics['post_selected_seconds']:.1f}s after selection but never exceeded 5% progress")

    if metrics["hac_selected_ut"] is not None and metrics["post_selected_seconds"] > 30.0 and not metrics["hac_captured_seen"]:
        failures.append(f"HAC remained selected for {metrics['post_selected_seconds']:.1f}s without capture")
    if metrics["transition_backsteps"] > 3:
        failures.append(f"HAC transition progress stepped backward {metrics['transition_backsteps']} times")
    if metrics["hac_completed_seen"] and not metrics["hac_captured_seen"]:
        failures.append("HAC completion was logged without prior/observed HAC capture")
    return failures

def print_terminal_transition(path):
    metrics = terminal_transition_metrics(path)
    failures = terminal_transition_failures(metrics)
    print(path)
    print("terminal_phases", " -> ".join(metrics["phases"]) if metrics["phases"] else "n/a")
    print("first_taem_ut", metrics["first_taem_ut"] if metrics["first_taem_ut"] is not None else "n/a")

    print("first_taem_altitude_m", metrics["first_taem_altitude"])
    print("first_taem_speed_mps", metrics["first_taem_speed"])
    print("first_taem_energy_excess_m", metrics["first_taem_energy"])
    print("first_taem_exec_phase", metrics["first_taem_exec_phase"] or "n/a")
    print("high_energy_acquisition_rows", metrics["high_energy_acquisition_rows"])
    print("taem_overspeed_rows", metrics["taem_overspeed_rows"])
    print("s_turn_airbrake_rows", metrics["s_turn_airbrake_rows"])
    print("s_turn_min_altitude_m", metrics["s_turn_min_altitude"])
    print("hac_selected_ut", metrics["hac_selected_ut"] if metrics["hac_selected_ut"] is not None else "n/a")
    print("post_selected_s", metrics["post_selected_seconds"])
    print("transition_seen", metrics["transition_seen"])
    print("transition_max_progress", metrics["transition_max_progress"])
    print("transition_backsteps", metrics["transition_backsteps"])
    print("hac_captured_seen", metrics["hac_captured_seen"])
    print("hac_completed_seen", metrics["hac_completed_seen"])
    print("final_captured_seen", metrics["final_captured_seen"])
    print("terminal_acceptance", "PASS" if not failures else "FAIL")
    for failure in failures:
        print("  -", failure)
    return not failures


def requested_course_rate(snapshot):
    guidance = snapshot.get("guidanceState") or {}
    logged = finite(guidance.get("commandedCourseRateEstimate"))
    if logged is not None:
        return logged
    telemetry = snapshot.get("telemetry") or {}
    command = snapshot.get("command") or {}
    bank = finite(command.get("targetRoll"))
    lift = finite(telemetry.get("liftForce"))
    mass = finite(telemetry.get("mass"))
    speed = finite(telemetry.get("trueAirSpeed"))
    effectiveness = finite(telemetry.get("bankEffectiveness"))
    if bank is None or lift is None or mass is None or speed is None or mass <= 1 or speed <= 1:
        return None
    effectiveness = 1.0 if effectiveness is None else max(.35, min(1.8, effectiveness))
    bank = max(-89.0, min(89.0, signed_angle(bank) * effectiveness))
    return math.degrees((lift / mass) * math.sin(math.radians(bank)) / speed)


def analyze(path):
    series = {name: [] for name in (
        "target_roll", "measured_roll", "roll_error", "body_roll_rate",
        "target_heading_rate", "requested_course_rate", "measured_course_rate",
        "course_rate_error", "radial_error", "cross_track", "speed",
        "vertical_speed", "aoa", "sideslip", "transition_progress",
        "hac_arc_remaining", "path_remaining", "roll_authority",
        "roll_raw_authority", "controller_target_roll_rate", "beta_confidence",
        "beta_yaw_gain", "beta_roll_coupling", "yaw_command",
    )}
    phases = []
    first_ut = last_ut = selected_ut = None
    previous = None
    transition_seen = final_seen = False

    for line in path.read_text(encoding="utf-8").splitlines():
        try:
            snapshot = json.loads(line)
        except json.JSONDecodeError:
            continue
        if snapshot.get("recordType") != "snapshot" or not snapshot.get("automationEngaged"):
            continue
        telemetry = snapshot.get("telemetry") or {}
        command = snapshot.get("command") or {}
        guidance = snapshot.get("guidanceState") or {}
        ut = finite(telemetry.get("ut"))
        if ut is None:
            continue
        if first_ut is None:
            first_ut = ut
        last_ut = ut
        phase = str(snapshot.get("phase") or "unknown")
        if not phases or phases[-1] != phase:
            phases.append(phase)
        selected = bool(guidance.get("hacSideSelected"))
        if selected and selected_ut is None:
            selected_ut = ut
        transition_seen |= bool(guidance.get("hacTransitionActive"))
        final_seen |= bool(guidance.get("finalCaptured"))
        if not selected or selected_ut is None or ut - selected_ut < .5:
            previous = snapshot
            continue

        target_roll = finite(command.get("targetRoll"))
        measured_roll = finite(telemetry.get("roll"))
        if target_roll is not None:
            target_roll = signed_angle(target_roll)
            series["target_roll"].append(target_roll)
        if measured_roll is not None:
            measured_roll = signed_angle(measured_roll)
            series["measured_roll"].append(measured_roll)
        if target_roll is not None and measured_roll is not None:
            series["roll_error"].append(angle_difference(target_roll, measured_roll))

        for name, key in (
            ("body_roll_rate", "bodyRollRate"),
            ("measured_course_rate", "courseRate"),
            ("cross_track", "runwayCrossTrack"),
            ("speed", "trueAirSpeed"),
            ("vertical_speed", "verticalSpeed"),
            ("aoa", "angleOfAttack"),
            ("sideslip", "sideslip"),
        ):
            value = finite(telemetry.get(key))
            if value is not None:
                series[name].append(value)
        for name, key in (
            ("transition_progress", "hacTransitionProgress"),
            ("hac_arc_remaining", "hacArcRemaining"),
        ):
            value = finite(guidance.get(key))
            if value is not None:
                series[name].append(value)

        lifecycle = snapshot.get("commandLifecycle") or {}
        applied = lifecycle.get("appliedCommand") or {}
        diagnostics = applied.get("controlDiagnostics") or {}
        for name, key in (
            ("roll_authority", "rollAuthority"),
            ("roll_raw_authority", "rollRawAuthority"),
            ("controller_target_roll_rate", "targetRollRate"),
            ("beta_confidence", "betaConfidence"),
            ("beta_yaw_gain", "betaYawGain"),
            ("beta_roll_coupling", "betaRollCoupling"),
            ("yaw_command", "yawCommand"),
        ):
            value = finite(diagnostics.get(key))
            if value is not None:
                series[name].append(value)

        status = str(snapshot.get("statusMessage") or "")
        transition_seen |= "join+arc" in status
        match = RADIAL_RE.search(status)
        if match:
            series["radial_error"].append(float(match.group(1)) * 1000.0)
        match = PATH_RE.search(status)
        if match:
            series["path_remaining"].append(float(match.group(1)) * 1000.0)

        requested = requested_course_rate(snapshot)
        measured = finite(telemetry.get("courseRate"))
        if requested is not None:
            series["requested_course_rate"].append(requested)
            if measured is not None:
                series["course_rate_error"].append(requested - measured)

        if previous is not None:
            prev_t = previous.get("telemetry") or {}
            prev_c = previous.get("command") or {}
            prev_ut = finite(prev_t.get("ut"))
            heading = finite(command.get("targetHeading"))
            prev_heading = finite(prev_c.get("targetHeading"))
            if prev_ut is not None and heading is not None and prev_heading is not None:
                dt = ut - prev_ut
                if .03 <= dt <= .5:
                    series["target_heading_rate"].append(angle_difference(heading, prev_heading) / dt)
        previous = snapshot

    duration = last_ut - first_ut if first_ut is not None and last_ut is not None else math.nan
    print(path)
    print(f"duration_s {duration:.2f}")
    print("phases", " -> ".join(phases) if phases else "n/a")
    print("hac_selected_ut", selected_ut if selected_ut is not None else "n/a")
    print("transition_active_seen", transition_seen, "final_captured_seen", final_seen)
    print(
        "path_events",
        f"target_roll_reversals={sign_changes(series['target_roll'], 3.0)}",
        f"requested_course_rate_reversals={sign_changes(series['requested_course_rate'], .15)}",
        f"radial_crossings={sign_changes(series['radial_error'], 100.0)}",
        f"transition_progress_backsteps={decreasing_steps(series['transition_progress'], 1e-4)}",
        f"hac_arc_increases={increasing_steps(series['hac_arc_remaining'], 1.0)}",
        f"reported_path_increases={increasing_steps(series['path_remaining'], 10.0)}",
    )
    for name, values in series.items():
        endpoints = ""
        if name in ("transition_progress", "hac_arc_remaining", "path_remaining") and values:
            endpoints = f" endpoints={values[0]:.3f}->{values[-1]:.3f}"
        print(name, summary(values) + endpoints)


def main():
    parser = argparse.ArgumentParser(
        description="Analyze HAC/terminal behavior or assert inertial orbital-capture acceptance from flight logs."
    )
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument("--orbital-capture", action="store_true",
                      help="print retrograde/inertial capture metrics and acceptance result")
    mode.add_argument("--assert-orbital-clean", action="store_true",
                      help="exit nonzero if any orbital-capture acceptance invariant fails")
    mode.add_argument("--expect-orbital-failure", action="store_true",
                      help="exit nonzero unless the detector finds at least one orbital-capture failure")
    mode.add_argument("--terminal-transition", action="store_true")
    mode.add_argument("--assert-terminal-clean", action="store_true")
    mode.add_argument("--expect-terminal-failure", action="store_true")
    parser.add_argument("logs", nargs="+", metavar="LOG.jsonl")
    args = parser.parse_args()

    orbital_mode = args.orbital_capture or args.assert_orbital_clean or args.expect_orbital_failure
    terminal_mode = args.terminal_transition or args.assert_terminal_clean or args.expect_terminal_failure
    overall_ok = True
    saw_expected_failure = False
    for index, name in enumerate(args.logs):
        if index:
            print()
        path = Path(name)
        if orbital_mode:
            clean = print_orbital_capture(path)
            overall_ok &= clean
            saw_expected_failure |= not clean
        elif terminal_mode:
            clean = print_terminal_transition(path)
            overall_ok &= clean
            saw_expected_failure |= not clean
        else:
            analyze(path)

    if (args.assert_orbital_clean or args.assert_terminal_clean) and not overall_ok:
        raise SystemExit(1)
    if (args.expect_orbital_failure or args.expect_terminal_failure) and not saw_expected_failure:
        raise SystemExit("expected a replay acceptance failure signature, but all supplied logs passed")


if __name__ == "__main__":
    main()
