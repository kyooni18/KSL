#!/usr/bin/env python3
"""Probe original-STS control-surface aero wrench at archived measured states.

Small bounded control inputs are applied only at the restored 72 km vacuum
checkpoint so KSP can position the surfaces. The vessel is paused during every
hypothetical atmospheric wrench call and restored afterward. Source telemetry
explicitly bypassed the shadow gate; results are unqualified diagnostics.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import os
from pathlib import Path
import subprocess
import sys
import time

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT / "Tools") not in sys.path:
    sys.path.insert(0, str(ROOT / "Tools"))

from headless_flight import restore_quicksave, secure_failed_test_state
from ksp_test_guard import connect
from probe_sts_aero_wrench_grid import cross, inverse_rotate, norm, rotate, state_snapshot
from probe_sts_measured_run9_wrench_grid import (
    INPUT_PATH,
    measured_attitude,
    read_measured_states,
)
from probe_sts_shadow_witness_wrench import pids_named_ksp, read_actuator_snapshot

PROFILE_PATH = ROOT / "Configuration/checkpoints/sts-campaign-72km-20261008.physics.json"
CONFIG_PATH = ROOT / "Configuration/sts.json"
OUTPUT_PATH = ROOT / "Docs/Experiments/STS-Entry-20261009/live-measured-run9-surface-wrench-20261009.json"
CONTROL_AXES = (("pitch", 0), ("roll", 1), ("yaw", 2))
BASIS_AMPLITUDE = 0.15
PULSE_SETTLE_SECONDS = 0.10
MAX_UNPAUSED_SECONDS = 2.5
WATCHDOG_MAX_WALL_SECONDS = 90.0


def sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def deep_merge(target: dict, update: dict) -> None:
    for key, value in update.items():
        if isinstance(value, dict) and isinstance(target.get(key), dict):
            deep_merge(target[key], value)
        else:
            target[key] = value


def read_recorded_controls(selected: list[dict]) -> list[dict]:
    wanted = {int(anchor["recordSequence"]): index for index, anchor in enumerate(selected)}
    state: dict = {}
    controls: list[dict | None] = [None] * len(selected)
    for line in INPUT_PATH.read_text().splitlines():
        record = json.loads(line)
        fields = record.get("fields") or {}
        if record.get("recordType") == "vehicleKeyframe":
            state = json.loads(json.dumps(fields))
        else:
            deep_merge(state, fields)
        sequence = int(record.get("recordSequence", -1))
        if sequence not in wanted:
            continue
        applied = state.get("appliedControl") or {}
        if not applied.get("reportedAvailable"):
            raise RuntimeError(f"Archived anchor {sequence} has no actual control readback")
        values = [float(applied[name]) for name in ("reportedPitch", "reportedRoll", "reportedYaw")]
        if any(not math.isfinite(value) or abs(value) > 0.5 for value in values):
            raise RuntimeError(f"Archived anchor {sequence} controls exceed the bounded playback envelope")
        controls[wanted[sequence]] = {
            "anchorIndex": wanted[sequence],
            "recordSequence": sequence,
            "ut": float(record["ut"]),
            "pitchRollYaw": values,
            "readbackSource": "appliedControl.reportedPitch/reportedRoll/reportedYaw",
        }
    if any(item is None for item in controls):
        raise RuntimeError("Could not recover actual control readbacks for every selected anchor")
    return [item for item in controls if item is not None]


def read_probe_actuator_snapshot(vessel, config: dict) -> dict:
    """Record enabled axes and override semantics for every KSP control surface."""
    snapshot = read_actuator_snapshot(vessel, config)
    for record, surface in zip(snapshot["controlSurfaces"], vessel.parts.control_surfaces):
        record.update({
            "deflectionOverrideEnabled": bool(surface.deflection_override),
            "deflectionOverrideCommandNormalized": float(surface.deflection),
            "pitchEnabled": bool(surface.pitch_enabled),
            "rollEnabled": bool(surface.roll_enabled),
            "yawEnabled": bool(surface.yaw_enabled),
            "inverted": bool(surface.inverted),
            "surfaceAreaM2": float(surface.surface_area),
        })
    snapshot["deflectionReadbackSemantics"] = (
        "kRPC ControlSurface.deflection is a normalized override command, not an actual surface-angle readback; "
        "it is applied only while deflectionOverrideEnabled is true."
    )
    return snapshot


def parent_alive(pid: int) -> bool:
    try:
        os.kill(pid, 0)
        return True
    except ProcessLookupError:
        return False


def watchdog(start_ut: float, parent_pid: int, ready_path: Path, save_name: str) -> int:
    """Freeze and restore the checkpoint if the bounded bench owner stalls."""
    conn = connect("STS surface-wrench probe independent freeze guard", timeout=8)
    try:
        sc = conn.space_center
        vessel = sc.active_vessel
        if vessel is None or vessel.name != "STS" or not conn.krpc.paused:
            raise RuntimeError("Independent guard requires paused original STS")
        ready_path.write_text("ready\n")
        # A 40-call hypothetical wrench grid can outlast a short fixed timeout
        # on a busy KSP host. Do not race a healthy parent by reloading the
        # checkpoint merely because the diagnostic is still computing.
        deadline = time.monotonic() + WATCHDOG_MAX_WALL_SECONDS
        while time.monotonic() < deadline and parent_alive(parent_pid):
            if float(sc.ut) - start_ut > MAX_UNPAUSED_SECONDS:
                break
            time.sleep(0.02)
        conn.krpc.paused = True
        for axis_name, _ in CONTROL_AXES:
            setattr(vessel.control, axis_name, 0.0)
    finally:
        conn.close()
    try:
        secure_failed_test_state(ROOT, save_name, True)
        subprocess.run(
            [sys.executable, str(ROOT / "Tools/settle_known_checkpoint.py"),
             "--profile", str(PROFILE_PATH), "--save", save_name],
            cwd=ROOT, capture_output=True, text=True, timeout=40, check=True,
        )
    finally:
        ready_path.unlink(missing_ok=True)
    return 0


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, default=OUTPUT_PATH)
    args = parser.parse_args()

    config = json.loads(CONFIG_PATH.read_text())
    profile = json.loads(PROFILE_PATH.read_text())
    measured_states, input_hash, bypass_events = read_measured_states(INPUT_PATH)
    recorded_controls = read_recorded_controls(measured_states)
    save_path = Path(profile["savePath"])
    checkpoint_hash = sha256(save_path)
    if checkpoint_hash != profile["saveSHA256"]:
        raise RuntimeError("Named campaign checkpoint SHA does not match its physics profile")
    if profile["saveName"] != "STS-campaign-72km-20261008" or config["vehicle"]["modelId"] != "STS":
        raise RuntimeError("Probe requires the named checkpoint and original STS")

    report = {
        "status": "running",
        "classification": "UNQUALIFIED_COUNTERFACTUAL_SIMULATION_FROM_BYPASSED_MEASUREMENTS",
        "purpose": "Evaluate current original-STS control-surface aero wrench at five archived measured run-9 states, using isolated bounded surface commands and the logged command vector for each matching anchor.",
        "qualification": {
            "qualified": False,
            "inputRunShadowGateBypassed": True,
            "measuredFlightStatesUsedAsInputs": True,
            "simulatedWrenchIsMeasuredFlightData": False,
            "shadowGateEvidence": False,
            "flightControlOrActuation": False,
            "landingOrRolloutEvidence": False,
            "note": "Every atmospheric wrench is a KSP simulation at archived measured kinematics. The source run explicitly bypassed the shadow gate, and historical surface deflections were not recorded.",
        },
        "provenance": {
            "repository": str(ROOT),
            "gitHead": subprocess.run(["git", "rev-parse", "HEAD"], cwd=ROOT, capture_output=True, text=True, check=True).stdout.strip(),
            "inputTelemetry": str(INPUT_PATH.relative_to(ROOT)),
            "inputTelemetrySHA256": input_hash,
            "inputGateBypassEvents": bypass_events,
            "inputMeasuredStates": measured_states,
            "loggedControlReadbacks": recorded_controls,
            "configuration": str(CONFIG_PATH.relative_to(ROOT)),
            "configurationSHA256": sha256(CONFIG_PATH),
            "checkpointProfile": str(PROFILE_PATH.relative_to(ROOT)),
            "checkpointName": profile["saveName"],
            "checkpointSHA256Before": checkpoint_hash,
            "api": "SpaceCenter.Flight.simulate_aerodynamic_wrench_at",
            "referenceFrame": "CelestialBody.non_rotating_reference_frame",
            "bodyAxisOrder": ["local +X pitch", "local +Y roll", "local +Z yaw"],
            "pulsePolicy": "Apply controls only at the paused 72 km vacuum checkpoint, briefly advance under watchdog for surface positioning, then pause before each simulated atmospheric wrench call. Pitch/roll/yaw only; SAS/RCS/throttle/speedbrake action group remain off.",
            "surfaceStateCaveat": "Each case sets the corresponding normalized vessel pitch/roll/yaw inputs and records the simulated aerodynamic wrench. kRPC ControlSurface.deflection is only an override command, not actual surface angle; its override was disabled on the six checkpoint surfaces. The API result is a current-model command response, not a measured historical surface deflection. Historical run-9 deflections and per-sample RCS state are unavailable, so this is not an exact actuator-state replay.",
            "controlSurfaceAPIReference": "https://krpc.github.io/krpc/0.6.0/python/api/space-center/parts.html",
            "noEntryPolicy": "No atmospheric flight or entry guidance is engaged. The checkpoint is restored and settled after the vacuum control-surface bench.",
            "probeSource": str(Path(__file__).relative_to(ROOT)),
            "probeSourceSHA256": sha256(Path(__file__)),
            "helperSourceSHA256": {
                "probe_sts_measured_run9_wrench_grid.py": sha256(Path(__file__).with_name("probe_sts_measured_run9_wrench_grid.py")),
                "probe_sts_aero_wrench_grid.py": sha256(Path(__file__).with_name("probe_sts_aero_wrench_grid.py")),
                "probe_sts_shadow_witness_wrench.py": sha256(Path(__file__).with_name("probe_sts_shadow_witness_wrench.py")),
                "headless_flight.py": sha256(Path(__file__).with_name("headless_flight.py")),
                "ksp_test_guard.py": sha256(Path(__file__).with_name("ksp_test_guard.py")),
            },
        },
        "gridDefinition": {
            "anchorCount": len(measured_states),
            "isolatedControlAmplitude": BASIS_AMPLITUDE,
            "controlAxes": {"pitch": "local +X", "roll": "local +Y", "yaw": "local +Z"},
            "basisCasesPerAxis": ["negative", "positive"],
            "recordedControlCases": "One actual archived pitch/roll/yaw readback vector per corresponding measured-state anchor.",
            "derivativeMatrixRows": "body torque XYZ (N m)",
            "derivativeMatrixColumns": "normalized kRPC pitch/roll/yaw control input",
            "derivativeUnits": "d(body torque N m)/d(normalized control input)",
            "callCountExpected": len(measured_states) * (1 + 2 * len(CONTROL_AXES)) + len(recorded_controls),
        },
        "startup": None,
        "controlCases": [],
        "samples": [],
        "anchorControlMatrices": [],
        "checkpointSHA256Before": checkpoint_hash,
        "checkpointSHA256After": None,
        "checkpointUnchanged": False,
        "postRunState": None,
        "postRunLiveReadback": None,
        "liveStateRestored": False,
        "kspProcessPIDsBefore": [],
        "kspProcessPIDsAfter": [],
        "singleKSPProcessPreserved": False,
        "errors": [],
    }

    conn = None
    guard = None
    ready_path = args.output.with_suffix(".guard-ready")
    args.output.parent.mkdir(parents=True, exist_ok=True)
    started = time.monotonic()
    failure: Exception | None = None
    try:
        pids_before = pids_named_ksp()
        if len(pids_before) != 1:
            raise RuntimeError(f"Expected exactly one live KSP process, found {pids_before}")
        report["kspProcessPIDsBefore"] = pids_before

        # Re-load the named checkpoint into the existing KSP process, then wait
        # for its ordinary physics-unpack placeholder to settle before pulsing.
        restore_quicksave(ROOT, profile["saveName"])
        settled = subprocess.run(
            [sys.executable, str(ROOT / "Tools/settle_known_checkpoint.py"),
             "--profile", str(PROFILE_PATH), "--save", profile["saveName"]],
            cwd=ROOT, capture_output=True, text=True, timeout=40, check=True,
        )
        startup = json.loads(settled.stdout.strip().splitlines()[-1])
        report["startup"] = startup

        conn = connect("STS measured-run9 surface-wrench pulse bench", timeout=8)
        sc = conn.space_center
        vessel = sc.active_vessel
        if vessel is None or vessel.name != "STS":
            raise RuntimeError("Active live vessel is not the original STS")
        body = vessel.orbit.body
        if body.name != "Kerbin":
            raise RuntimeError("Active original STS is not orbiting Kerbin")
        frame = body.non_rotating_reference_frame
        before = state_snapshot(conn, sc, vessel, body, frame)
        actuator_before = read_probe_actuator_snapshot(vessel, config)
        action_groups_before = [bool(vessel.control.get_action_group(i)) for i in range(10)]
        report["prePulseState"] = before
        report["prePulseActuators"] = {**actuator_before, "actionGroups0Through9": action_groups_before}
        if not before["paused"] or abs(before["ut"] - float(profile["savedUT"])) > 2.0:
            raise RuntimeError("Probe requires paused KSP at the named checkpoint epoch")
        if abs(before["altitude_m"] - 72000.0) > 500.0 or before["part_count"] != int(profile["partCount"]):
            raise RuntimeError("Live original STS is not at the settled checkpoint state")
        if abs(before["mass_kg"] - float(profile["mass"])) > 1.0:
            raise RuntimeError("Live STS mass does not match the full-mass checkpoint")
        if before["sas"] or before["rcs"] or before["wheel_brakes"]:
            raise RuntimeError("SAS/RCS/wheel brakes must be off")
        if abs(before["thrust_n"]) > 1.0 or before["dynamic_pressure_pa"] > 1e-6:
            raise RuntimeError("Vacuum bench requires zero thrust and dynamic pressure")
        if any(abs(float(value)) > 1e-9 for value in before["controls"].values()):
            raise RuntimeError("Control axes must start neutral")
        if any(action_groups_before) or actuator_before["speedbrakeActionGroupEnabled"]:
            raise RuntimeError("Action groups and speedbrake group must start off")
        if any(surface["deployed"] or surface["deflectionOverrideEnabled"]
               for surface in actuator_before["controlSurfaces"]):
            raise RuntimeError("Control surfaces must start stowed with normal flight-control allocation")

        initial_ut = before["ut"]
        ready_path.unlink(missing_ok=True)
        guard = subprocess.Popen(
            [sys.executable, str(Path(__file__).resolve()), "--watchdog",
             str(initial_ut), str(os.getpid()), str(ready_path), profile["saveName"]],
            cwd=ROOT, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
        )
        ready_deadline = time.monotonic() + 15.0
        while not ready_path.exists():
            if guard.poll() is not None or time.monotonic() > ready_deadline:
                raise RuntimeError("Independent surface-bench freeze guard did not become ready")
            time.sleep(0.02)

        body_omega = tuple(float(value) for value in body.angular_velocity(frame))
        flight = vessel.flight(frame)
        neutral_rates: list[list[float]] = []
        for anchor in measured_states:
            neutral_rates.append([math.radians(float(x)) for x in anchor["bodyRatesDegPerSecond_pitchRollYaw"]])

        case_requests: list[dict] = [{"caseId": "neutral", "controls": [0.0, 0.0, 0.0], "anchorIndices": list(range(len(measured_states))), "kind": "neutral_reference"}]
        for axis_name, axis in CONTROL_AXES:
            for sign, polarity in ((-1.0, "negative"), (1.0, "positive")):
                controls = [0.0, 0.0, 0.0]
                controls[axis] = sign * BASIS_AMPLITUDE
                case_requests.append({
                    "caseId": f"{axis_name}_{polarity}_0p15",
                    "controls": controls,
                    "anchorIndices": list(range(len(measured_states))),
                    "kind": "isolated_control_basis",
                    "axis": axis_name,
                })
        for command in recorded_controls:
            case_requests.append({
                "caseId": f"run9_anchor_{command['anchorIndex']}_control_replay",
                "controls": list(command["pitchRollYaw"]),
                "anchorIndices": [command["anchorIndex"]],
                "kind": "archived_actual_control_readback_replay",
                "axis": None,
            })
        report["gridDefinition"]["controlStateCount"] = len(case_requests)

        def pulse_and_pause(desired: list[float]) -> tuple[dict, dict]:
            if len(desired) != 3 or any(not math.isfinite(float(x)) or abs(float(x)) > 0.5 for x in desired):
                raise RuntimeError("Refusing an invalid or over-limit pitch/roll/yaw command")
            if not conn.krpc.paused:
                raise RuntimeError("Each new control state must start paused")
            for axis_name, axis in CONTROL_AXES:
                setattr(vessel.control, axis_name, float(desired[axis]))
            start = float(sc.ut)
            try:
                conn.krpc.paused = False
                deadline = time.monotonic() + 1.0
                while float(sc.ut) - start < PULSE_SETTLE_SECONDS:
                    if time.monotonic() > deadline:
                        raise RuntimeError("KSP did not advance during the bounded surface-position pulse")
                    if float(sc.ut) - initial_ut > MAX_UNPAUSED_SECONDS:
                        raise RuntimeError("Surface-position pulses exceeded the independent 2.5 s UT bound")
                    if abs(float(vessel.thrust)) > 1.0 or float(vessel.flight(frame).dynamic_pressure) > 1e-6:
                        raise RuntimeError("Unexpected thrust or atmosphere during the vacuum surface bench")
                    if vessel.control.sas or vessel.control.rcs:
                        raise RuntimeError("SAS or RCS became active during the surface bench")
                    time.sleep(0.002)
            finally:
                conn.krpc.paused = True
            state = state_snapshot(conn, sc, vessel, body, frame)
            actuators = read_probe_actuator_snapshot(vessel, config)
            actual_controls = [float(getattr(vessel.control, axis)) for axis, _ in CONTROL_AXES]
            if max(abs(actual_controls[i] - float(desired[i])) for i in range(3)) > 1e-4:
                raise RuntimeError(f"KSP control readback differs from requested pulse: {actual_controls} vs {desired}")
            if not state["paused"] or state["sas"] or state["rcs"] or abs(state["thrust_n"]) > 1.0:
                raise RuntimeError("Unsafe live state after control-surface pulse")
            if actuators["speedbrakeActionGroupEnabled"] or any(
                item["deployed"] or item["deflectionOverrideEnabled"] for item in actuators["controlSurfaces"]
            ):
                raise RuntimeError("Speedbrakes, deployed surfaces, or surface overrides detected during the pulse")
            angular_body = inverse_rotate(tuple(state["rotation_xyzw"]), tuple(state["angular_velocity_rad_s"]))
            max_rate_deg_s = max(abs(float(x)) for x in angular_body) * 180.0 / math.pi
            if max_rate_deg_s > float(config["guidance"]["entryRollRate"]):
                raise RuntimeError(f"Surface pulse exceeded the configured body-rate limit: {max_rate_deg_s:.4g} deg/s")
            return state, {**actuators, "actualControlsPitchRollYaw": actual_controls, "maximumBodyRateDegPerSecond": max_rate_deg_s}

        def evaluate_anchor(anchor_index: int, case_id: str, controls: list[float]) -> dict:
            anchor = measured_states[anchor_index]
            if not conn.krpc.paused or abs(float(sc.ut) - float(report["prePulseState"]["ut"])) > MAX_UNPAUSED_SECONDS + 0.1:
                raise RuntimeError("KSP must remain paused at the bounded checkpoint bench UT")
            position = tuple(anchor["positionM_nonrotatingKrpcXYZ"])
            inertial_velocity = tuple(anchor["velocityMps_nonrotatingKrpcXYZ"])
            atmo_velocity = cross(body_omega, position)
            air_velocity = tuple(inertial_velocity[i] - atmo_velocity[i] for i in range(3))
            rotation, achieved = measured_attitude(
                position, air_velocity, anchor["angleOfAttackDeg"],
                anchor["sideslipDeg"], anchor["bankDeg"],
            )
            rate_rad = neutral_rates[anchor_index]
            angular_ref = tuple(rotate(rotation, rate_rad))
            force_raw, torque_raw = flight.simulate_aerodynamic_wrench_at(
                body, position, inertial_velocity, rotation, angular_ref, anchor["ut"]
            )
            return {
                "caseId": case_id,
                "anchorIndex": anchor_index,
                "anchorRecordSequence": anchor["recordSequence"],
                "anchorUT": anchor["ut"],
                "controlsPitchRollYaw": list(controls),
                "measuredBodyRateDegPerSecondXYZ": list(anchor["bodyRatesDegPerSecond_pitchRollYaw"]),
                "achievedAnglesDeg": {"aoa": achieved[0], "sideslip": achieved[1], "bank": achieved[2]},
                "simulatedForceN_bodyXYZ": tuple(inverse_rotate(rotation, tuple(float(x) for x in force_raw))),
                "simulatedTorqueNm_bodyXYZ": tuple(inverse_rotate(rotation, tuple(float(x) for x in torque_raw))),
            }

        case_lookup: dict[tuple[str, int], dict] = {}
        for request in case_requests:
            controls = request["controls"]
            if request["kind"] == "neutral_reference":
                state = state_snapshot(conn, sc, vessel, body, frame)
                actuators = read_probe_actuator_snapshot(vessel, config)
                if any(abs(float(state["controls"][axis])) > 1e-9 for axis in ("pitch", "roll", "yaw")):
                    raise RuntimeError("Neutral reference is not actually at zero controls")
            else:
                state, actuators = pulse_and_pause(controls)
            case_record = {
                "caseId": request["caseId"],
                "kind": request["kind"],
                "axis": request.get("axis"),
                "requestedControlsPitchRollYaw": controls,
                "actualControlsPitchRollYaw": [float(getattr(vessel.control, axis)) for axis, _ in CONTROL_AXES],
                "liveUT": float(sc.ut),
                "livePaused": bool(conn.krpc.paused),
                "liveState": state,
                "actuatorState": actuators,
                "anchorIndices": request["anchorIndices"],
            }
            report["controlCases"].append(case_record)
            for anchor_index in request["anchorIndices"]:
                sample = evaluate_anchor(anchor_index, request["caseId"], controls)
                report["samples"].append(sample)
                case_lookup[(request["caseId"], anchor_index)] = sample

        for anchor_index, anchor in enumerate(measured_states):
            neutral = case_lookup[("neutral", anchor_index)]["simulatedTorqueNm_bodyXYZ"]
            matrix = [[0.0 for _ in range(3)] for _ in range(3)]
            for axis_name, axis in CONTROL_AXES:
                plus = case_lookup[(f"{axis_name}_positive_0p15", anchor_index)]["simulatedTorqueNm_bodyXYZ"]
                minus = case_lookup[(f"{axis_name}_negative_0p15", anchor_index)]["simulatedTorqueNm_bodyXYZ"]
                for torque_axis in range(3):
                    matrix[torque_axis][axis] = (plus[torque_axis] - minus[torque_axis]) / (2.0 * BASIS_AMPLITUDE)
            recorded_case = f"run9_anchor_{anchor_index}_control_replay"
            replayed_torque = case_lookup[(recorded_case, anchor_index)]["simulatedTorqueNm_bodyXYZ"]
            report["anchorControlMatrices"].append({
                "anchorIndex": anchor_index,
                "anchorRecordSequence": anchor["recordSequence"],
                "ut": anchor["ut"],
                "mach": anchor["machReported"],
                "dynamicPressurePa": anchor["dynamicPressurePaReported"],
                "angleOfAttackDeg": anchor["angleOfAttackDeg"],
                "sideslipDeg": anchor["sideslipDeg"],
                "bankDeg": anchor["bankDeg"],
                "measuredBodyRateDegPerSecondXYZ": anchor["bodyRatesDegPerSecond_pitchRollYaw"],
                "neutralSurfaceTorqueNm_bodyXYZ": neutral,
                "controlEffectivenessMatrixNmPerControl_bodyTorqueXYZ_by_pitchRollYaw": matrix,
                "recordedControlReadbackPitchRollYaw": recorded_controls[anchor_index]["pitchRollYaw"],
                "replayedRecordedControlTorqueNm_bodyXYZ": replayed_torque,
                "replayedMinusNeutralTorqueNm_bodyXYZ": [replayed_torque[i] - neutral[i] for i in range(3)],
            })

        # Return the temporary vacuum pulse state to neutral before restoring.
        pulse_and_pause([0.0, 0.0, 0.0])
        for axis_name, _ in CONTROL_AXES:
            setattr(vessel.control, axis_name, 0.0)
        report["status"] = "complete"
    except Exception as exc:
        failure = exc
        report["status"] = "error"
        report["errors"].append(str(exc))
    finally:
        if conn is not None:
            try:
                conn.krpc.paused = True
                if sc.active_vessel is not None:
                    for axis_name, _ in CONTROL_AXES:
                        setattr(sc.active_vessel.control, axis_name, 0.0)
            except Exception as exc:
                report["errors"].append(f"final freeze/control reset failed: {exc}")
            conn.close()
        if guard is not None and guard.poll() is None:
            guard.terminate()
            guard.wait(timeout=5)
        ready_path.unlink(missing_ok=True)
        try:
            report["cleanup"] = secure_failed_test_state(ROOT, profile["saveName"], True)
            settled = subprocess.run(
                [sys.executable, str(ROOT / "Tools/settle_known_checkpoint.py"),
                 "--profile", str(PROFILE_PATH), "--save", profile["saveName"]],
                cwd=ROOT, capture_output=True, text=True, timeout=40, check=True,
            )
            report["postRunState"] = json.loads(settled.stdout.strip().splitlines()[-1])
            # The settle helper proves mass and pause state but omits controls
            # and action groups. Read those back from the paused live vessel so
            # cleanup verification reflects the actual restored state.
            verify_conn = connect("STS surface-wrench probe restored-state verification", timeout=8)
            try:
                verify_sc = verify_conn.space_center
                verify_vessel = verify_sc.active_vessel
                if verify_vessel is None:
                    raise RuntimeError("No active vessel after checkpoint settle")
                verify_body = verify_vessel.orbit.body
                verify_frame = verify_body.non_rotating_reference_frame
                post = state_snapshot(verify_conn, verify_sc, verify_vessel, verify_body, verify_frame)
                post["actionGroups0Through9"] = [bool(verify_vessel.control.get_action_group(i)) for i in range(10)]
                post["actuators"] = read_probe_actuator_snapshot(verify_vessel, config)
                report["postRunLiveReadback"] = post
            finally:
                verify_conn.close()
            report["checkpointSHA256After"] = sha256(save_path)
            report["checkpointUnchanged"] = report["checkpointSHA256After"] == checkpoint_hash
            report["kspProcessPIDsAfter"] = pids_named_ksp()
            report["singleKSPProcessPreserved"] = report["kspProcessPIDsAfter"] == report["kspProcessPIDsBefore"] and len(report["kspProcessPIDsAfter"]) == 1
            report["liveStateRestored"] = (
                post.get("vessel") == "STS" and post.get("paused") is True
                and abs(float(post.get("mass_kg", 0.0)) - float(profile["mass"])) <= 1.0
                and post.get("sas") is False and post.get("rcs") is False
                and post.get("wheel_brakes") is False
                and all(abs(float(post["controls"][axis])) <= 1e-9 for axis in ("pitch", "roll", "yaw", "throttle"))
                and abs(float(post.get("thrust_n", math.inf))) <= 1.0
                and post.get("actuators", {}).get("speedbrakeActionGroupEnabled") is False
                and not any(post.get("actionGroups0Through9", []))
                and not any(surface["deployed"] or surface["deflectionOverrideEnabled"]
                            for surface in post.get("actuators", {}).get("controlSurfaces", []))
                and report["checkpointUnchanged"] and report["singleKSPProcessPreserved"]
            )
            if not report["liveStateRestored"]:
                report["status"] = "error"
                report["errors"].append("Post-run settle did not verify paused full-mass original STS and unchanged checkpoint")
        except Exception as exc:
            report["status"] = "error"
            report["errors"].append(f"checkpoint restore/verification failed: {exc}")
        report["elapsedWallSeconds"] = time.monotonic() - started
        report["sourceSHA256"] = sha256(Path(__file__))
        report["output"] = str(args.output)
        args.output.write_text(json.dumps(report, indent=2, allow_nan=False) + "\n")

    if failure is not None:
        raise failure
    print(json.dumps({
        "status": report["status"],
        "classification": report["classification"],
        "output": report["output"],
        "sampleCount": len(report["samples"]),
        "controlStateCount": len(report["controlCases"]),
        "liveStateRestored": report["liveStateRestored"],
        "checkpointUnchanged": report["checkpointUnchanged"],
        "singleKSPProcessPreserved": report["singleKSPProcessPreserved"],
        "elapsedWallSeconds": report["elapsedWallSeconds"],
    }, separators=(",", ":")))
    return 0


if __name__ == "__main__":
    if len(sys.argv) > 1 and sys.argv[1] == "--watchdog":
        raise SystemExit(watchdog(float(sys.argv[2]), int(sys.argv[3]), Path(sys.argv[4]), sys.argv[5]))
    raise SystemExit(main())
