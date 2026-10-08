#!/usr/bin/env python3
"""Read-only KSP aero-rate damping probe for the original STS.

All rate cases are hypothetical inputs to Flight.simulate_aerodynamic_wrench_at.
The probe never changes vessel controls or advances KSP physics.
"""

from __future__ import annotations

import hashlib
import json
import math
from pathlib import Path
import subprocess
import sys
import time

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT / "Tools") not in sys.path:
    sys.path.insert(0, str(ROOT / "Tools"))

from ksp_test_guard import connect
from probe_sts_aero_wrench_grid import (
    angles_degrees,
    bank_degrees,
    cross,
    dot,
    inverse_rotate,
    norm,
    quaternion_multiply,
    rotate,
    state_snapshot,
    unit,
    wrap_degrees,
)
from probe_sts_shadow_witness_wrench import (
    pids_named_ksp,
    quaternion_from_basis,
    read_actuator_snapshot,
    read_shadow_witness,
    rotate_by_body_spin,
    sts_flight_roll,
)


YAW_PULSE_PATH = ROOT / "Docs/Experiments/STS-Entry-20261008/live-preentry-yaw-repeat.json"
OUTPUT_PATH = ROOT / "Docs/Experiments/STS-Entry-20261009/live-aero-rate-damping.json"
RATE_AXES = (("bodyX", 0), ("bodyY_roll", 1), ("bodyZ_yaw", 2))


def measured_rate_envelope() -> tuple[dict, str]:
    data = json.loads(YAW_PULSE_PATH.read_text())
    samples = data.get("samples") or []
    pulse_samples = [sample for sample in samples if abs(float(sample.get("control_yaw", 0.0))) > 0.01]
    if not pulse_samples:
        raise RuntimeError("The retained yaw pulse has no commanded-yaw samples")
    body_rates = [sample.get("body_rates") for sample in pulse_samples]
    if any(not isinstance(rates, list) or len(rates) != 3 for rates in body_rates):
        raise RuntimeError("The retained yaw pulse is missing three-axis body rates")
    maxima = [max(abs(float(rates[axis])) for rates in body_rates) for axis in range(3)]
    summary = {
        "source": str(YAW_PULSE_PATH.relative_to(ROOT)),
        "sourceSHA256": hashlib.sha256(YAW_PULSE_PATH.read_bytes()).hexdigest(),
        "sampleCount": len(pulse_samples),
        "commandedYawRange": [
            min(float(sample["control_yaw"]) for sample in pulse_samples),
            max(float(sample["control_yaw"]) for sample in pulse_samples),
        ],
        "maximumAbsoluteBodyRatesDegPerSecondXYZ": maxima,
        "units": "The retained pulse stores body rates in degrees/second. These maxima set small hypothetical rate magnitudes; they are not measured atmospheric rates.",
    }
    return summary, hashlib.sha256(YAW_PULSE_PATH.read_bytes()).hexdigest()


def build_target_state(vessel, body, frame, current_ut: float, trace: dict,
                       witness: dict, bank_offset_deg: float) -> dict:
    config = json.loads((ROOT / "Configuration/sts.json").read_text())
    site = config["site"]
    latitude = float(site["latitude"])
    longitude = float(site["longitude"])
    target_ut = float(trace["ut"])
    altitude = float(trace["h"])
    elapsed = target_ut - current_ut
    body_omega = tuple(float(value) for value in body.angular_velocity(frame))

    base_position = tuple(body.position_at_altitude(latitude, longitude, altitude, frame))
    surface = tuple(body.position_at_altitude(latitude, longitude, 0.0, frame))
    north_sample = tuple(body.position_at_altitude(latitude + 0.01, longitude, 0.0, frame))
    east_sample = tuple(body.position_at_altitude(latitude, longitude + 0.01, 0.0, frame))
    radial_up = unit(base_position)
    north_raw = tuple(north_sample[i] - surface[i] for i in range(3))
    east_raw = tuple(east_sample[i] - surface[i] for i in range(3))
    north = unit(tuple(north_raw[i] - dot(north_raw, radial_up) * radial_up[i] for i in range(3)))
    east = unit(tuple(east_raw[i] - dot(east_raw, radial_up) * radial_up[i] for i in range(3)))
    heading = math.radians(float(site["runwayHeading"]))
    forward = unit(tuple(north[i] * math.cos(heading) + east[i] * math.sin(heading) for i in range(3)))
    vessel_z_sign = -1.0 if abs(bank_offset_deg) > 90.0 else 1.0
    vessel_z = unit(tuple(vessel_z_sign * (radial_up[i] - dot(radial_up, forward) * forward[i]) for i in range(3)))
    right = unit(cross(forward, vessel_z))
    local_rotation = quaternion_from_basis(right, forward, vessel_z)
    target_position, spin_rotation = rotate_by_body_spin(base_position, body_omega, elapsed)
    base_rotation = quaternion_multiply(spin_rotation, local_rotation)
    base_bank = sts_flight_roll(base_rotation, target_position, bank_offset_deg)
    if abs(wrap_degrees(base_bank)) > 0.2:
        raise RuntimeError(f"Constructed runway-aligned orientation has bank {base_bank:.3f} degrees")

    requested_bank = float(trace["roll"])
    roll_delta_rad = math.radians(wrap_degrees(base_bank - requested_bank))
    roll_rotation = (0.0, math.sin(roll_delta_rad / 2.0), 0.0, math.cos(roll_delta_rad / 2.0))
    rotation = quaternion_multiply(base_rotation, roll_rotation)
    alpha = math.radians(float(trace["aoa"]))
    beta = 0.0  # The retained ENTRY_TRACE does not record sideslip.
    speed = float(trace["V"])
    body_air_velocity = (
        speed * math.sin(beta),
        speed * math.cos(beta) * math.cos(alpha),
        speed * math.cos(beta) * math.sin(alpha),
    )
    air_velocity = tuple(rotate(rotation, body_air_velocity))
    inertial_velocity = tuple(
        air_velocity[i] + cross(body_omega, target_position)[i] for i in range(3)
    )
    sound_speed = float(witness["V"]) / float(witness["M"])
    achieved_aoa, achieved_beta, _ = angles_degrees(rotation, air_velocity)
    achieved_bank = sts_flight_roll(rotation, target_position, bank_offset_deg)
    if max(abs(achieved_aoa - float(trace["aoa"])), abs(achieved_beta),
           abs(wrap_degrees(achieved_bank - requested_bank))) > 0.05:
        raise RuntimeError("Hypothetical orientation and velocity failed the requested shadow-state angle check")
    return {
        "position": target_position,
        "rotation": rotation,
        "velocity": inertial_velocity,
        "airVelocity": air_velocity,
        "speedMps": speed,
        "nominalMachFromWitnessSoundSpeed": speed / sound_speed,
        "angleOfAttackDeg": float(trace["aoa"]),
        "sideslipDeg": 0.0,
        "bankDeg": requested_bank,
        "altitudeM": altitude,
        "ut": target_ut,
        "bodyAngularVelocityReferenceFrameRadPerS": body_omega,
        "spinExtrapolationSeconds": elapsed,
        "achieved": {
            "angleOfAttackDeg": achieved_aoa,
            "sideslipDeg": achieved_beta,
            "bankDeg": achieved_bank,
        },
    }


def main() -> int:
    config_path = ROOT / "Configuration/sts.json"
    profile_path = ROOT / "Configuration/checkpoints/sts-campaign-72km-20261008.physics.json"
    profile = json.loads(profile_path.read_text())
    config = json.loads(config_path.read_text())
    checkpoint_path = Path(profile["savePath"])
    checkpoint_hash_before = hashlib.sha256(checkpoint_path.read_bytes()).hexdigest()
    if checkpoint_hash_before != profile["saveSHA256"]:
        raise RuntimeError("The original campaign checkpoint hash does not match its physics profile")
    if config["vehicle"]["modelId"] != "STS":
        raise RuntimeError("The aerodynamic rate probe requires the original STS configuration")

    trace, witness, trace_hash = read_shadow_witness()
    yaw_rate_summary, yaw_hash = measured_rate_envelope()
    report = {
        "status": "running",
        "classification": "UNQUALIFIED_DIAGNOSTIC_SIMULATION",
        "purpose": "Measure the original STS KSP aerodynamic torque response to hypothetical angular rates at the retained 38 km shadow witness state, extending the static wrench grid with rate-dependent damping.",
        "qualification": {
            "qualified": False,
            "shadowGateEvidence": False,
            "measuredAerodynamicEvidence": False,
            "flightControlOrActuation": False,
            "landingOrRolloutEvidence": False,
            "note": "Every wrench is an instantaneous hypothetical KSP simulation. The witness state is a shadow-model trace at the KSC runway-center proxy, not measured atmospheric flight telemetry.",
        },
        "provenance": {
            "repository": str(ROOT),
            "gitHead": subprocess.run(["git", "rev-parse", "HEAD"], cwd=ROOT, capture_output=True, text=True, check=True).stdout.strip(),
            "sourceScript": "Tools/probe_sts_aero_rate_damping.py",
            "configuration": str(config_path.relative_to(ROOT)),
            "configurationSHA256": hashlib.sha256(config_path.read_bytes()).hexdigest(),
            "vehicleModelId": config["vehicle"]["modelId"],
            "checkpointProfile": str(profile_path.relative_to(ROOT)),
            "checkpointName": profile["saveName"],
            "checkpointSHA256Before": checkpoint_hash_before,
            "checkpointUsedAsLoadedVesselSource": True,
            "shadowTrace": str((ROOT / "Runtime/STS-Campaign-20261009/entry-trace/headless-live.log").relative_to(ROOT)),
            "shadowTraceSHA256": trace_hash,
            "selectedENTRYTraceRecord": trace,
            "nearestShadowWitnessRecord": witness,
            "yawRateAmplitudeSource": yaw_rate_summary,
            "api": "SpaceCenter.Flight.simulate_aerodynamic_wrench_at",
            "referenceFrame": "CelestialBody.non_rotating_reference_frame",
            "stateOrigin": "Shadow ENTRY_TRACE state projected to the configured KSC runway-center latitude/longitude at trace altitude and UT; Mach is reconstructed from the adjacent shadow-witness speed of sound.",
            "sideslipPolicy": "Zero sideslip because the retained ENTRY_TRACE has no measured or modeled sideslip field.",
            "angularVelocityPolicy": "Hypothetical body-axis rates are rotated into the non-rotating reference frame. The API reads the loaded STS part/control-surface state; no controls or state setters are called.",
            "controlAndFlightPolicy": "Read-only wrench calls only. No propulsion, RCS, speedbrakes, SAS, control axes, position, velocity, rotation, UT, physics, or save state is changed.",
        },
        "gridDefinition": {
            "angleOfAttackDeg": float(trace["aoa"]),
            "sideslipDeg": 0.0,
            "bankDeg": float(trace["roll"]),
            "nominalMachFromWitnessSoundSpeed": None,
            "bodyRateAxes": {"bodyX": "vessel local X", "bodyY_roll": "vessel local Y / roll axis", "bodyZ_yaw": "vessel local Z / yaw axis"},
            "requestCases": [],
            "tensorUnits": "d(body-axis torque N m)/d(body-axis angular velocity rad/s)",
        },
        "samples": [],
        "errors": [],
    }

    conn = None
    started = time.monotonic()
    try:
        pids_before = pids_named_ksp()
        if len(pids_before) != 1:
            raise RuntimeError(f"Expected one live KSP process; found {pids_before}")
        conn = connect("STS read-only aerodynamic rate damping", timeout=8)
        sc = conn.space_center
        vessel = sc.active_vessel
        if vessel is None or vessel.name != "STS":
            raise RuntimeError("The active live vessel must be the original STS")
        body = vessel.orbit.body
        if body.name != "Kerbin":
            raise RuntimeError(f"Expected Kerbin, found {body.name}")
        frame = body.non_rotating_reference_frame
        before = state_snapshot(conn, sc, vessel, body, frame)
        actuators_before = read_actuator_snapshot(vessel, config)
        saved_ut = float(profile["savedUT"])
        if not before["paused"]:
            raise RuntimeError("Refusing aerodynamic simulation unless KSP is paused")
        if abs(before["ut"] - saved_ut) > 5.0 or abs(before["altitude_m"] - 72000.0) > 500.0:
            raise RuntimeError("The live STS is not settled near the named 72 km checkpoint")
        if abs(before["mass_kg"] - float(profile["mass"])) > 1.0:
            raise RuntimeError("The live STS mass does not match the settled full-mass checkpoint profile")
        if before["sas"] or any(abs(float(value)) > 1e-9 for value in before["controls"].values()):
            raise RuntimeError("Refusing the probe while SAS or direct controls are active")
        if abs(before["thrust_n"]) > 1.0 or actuators_before["speedbrakeActionGroupEnabled"]:
            raise RuntimeError("Refusing the probe with thrust or speedbrakes active")
        if any(surface["deployed"] or abs(surface["deflection"]) > 1e-9 for surface in actuators_before["controlSurfaces"]):
            raise RuntimeError("Refusing the probe with a deployed or deflected control surface")
        if len(vessel.parts.all) != int(profile["partCount"]):
            raise RuntimeError("Loaded original-STS part count does not match the checkpoint profile")

        checkpoint_geometric_bank = bank_degrees(tuple(before["rotation_xyzw"]), tuple(before["position_m"]))
        bank_offset = wrap_degrees(before["roll_deg"] - checkpoint_geometric_bank)
        if min(abs(bank_offset), abs(abs(bank_offset) - 180.0)) > 0.2:
            raise RuntimeError("The checkpoint STS bank convention is not aligned with local +Z or -Z")
        target = build_target_state(vessel, body, frame, before["ut"], trace, witness, bank_offset)
        pulse_rates = yaw_rate_summary["maximumAbsoluteBodyRatesDegPerSecondXYZ"]
        request_cases = [{"case": "zero", "axis": None, "rateDegPerSecond": 0.0, "bodyRateRadPerSecondXYZ": [0.0, 0.0, 0.0]}]
        for name, axis in RATE_AXES:
            amplitude = max(float(pulse_rates[axis]), 0.01)
            amplitude_rad = math.radians(amplitude)
            for sign, label in ((-1.0, "negative"), (1.0, "positive")):
                vector = [0.0, 0.0, 0.0]
                vector[axis] = sign * amplitude_rad
                request_cases.append({
                    "case": f"{name}_{label}_measured-envelope",
                    "axis": name,
                    "rateDegPerSecond": sign * amplitude,
                    "bodyRateRadPerSecondXYZ": vector,
                })
        for axis_name, axis in (("bodyY_roll", 1), ("bodyZ_yaw", 2)):
            for sign, label in ((-1.0, "negative"), (1.0, "positive")):
                vector = [0.0, 0.0, 0.0]
                vector[axis] = sign * math.radians(1.0)
                request_cases.append({
                    "case": f"{axis_name}_{label}_1deg_s_linearity_check",
                    "axis": axis_name,
                    "rateDegPerSecond": sign,
                    "bodyRateRadPerSecondXYZ": vector,
                })
        report["gridDefinition"]["nominalMachFromWitnessSoundSpeed"] = target["nominalMachFromWitnessSoundSpeed"]
        report["gridDefinition"]["requestCases"] = request_cases
        report["liveStateBefore"] = before
        report["liveActuatorStateBefore"] = actuators_before
        report["kspProcessPIDsBefore"] = pids_before
        flight = vessel.flight(frame)
        baseline_torque = None
        for request in request_cases:
            if not conn.krpc.paused or abs(float(sc.ut) - before["ut"]) > 1e-9:
                raise RuntimeError("KSP pause or UT changed during hypothetical rate cases")
            omega_reference = tuple(rotate(tuple(target["rotation"]), request["bodyRateRadPerSecondXYZ"]))
            force_raw, torque_raw = flight.simulate_aerodynamic_wrench_at(
                body,
                target["position"],
                target["velocity"],
                target["rotation"],
                omega_reference,
                target["ut"],
            )
            force = tuple(float(value) for value in force_raw)
            torque_reference = tuple(float(value) for value in torque_raw)
            torque_body = tuple(inverse_rotate(tuple(target["rotation"]), torque_reference))
            sample = {
                "case": request["case"],
                "axis": request["axis"],
                "rateDegPerSecond": request["rateDegPerSecond"],
                "bodyRateRadPerSecondXYZ": request["bodyRateRadPerSecondXYZ"],
                "angularVelocityReferenceFrameRadPerSecond": omega_reference,
                "forceN_bodyXYZ": tuple(inverse_rotate(tuple(target["rotation"]), force)),
                "torqueNm_bodyXYZ": torque_body,
            }
            report["samples"].append(sample)
            if request["case"] == "zero":
                baseline_torque = torque_body

        if baseline_torque is None:
            raise RuntimeError("The zero-rate baseline was not evaluated")
        derivatives = [[None for _ in range(3)] for _ in range(3)]
        derivative_details = []
        sample_by_case = {sample["case"]: sample for sample in report["samples"]}
        for name, axis in RATE_AXES:
            amplitude_deg = max(float(pulse_rates[axis]), 0.01)
            plus = sample_by_case[f"{name}_positive_measured-envelope"]["torqueNm_bodyXYZ"]
            minus = sample_by_case[f"{name}_negative_measured-envelope"]["torqueNm_bodyXYZ"]
            amplitude_rad = math.radians(amplitude_deg)
            for torque_axis in range(3):
                derivatives[torque_axis][axis] = (plus[torque_axis] - minus[torque_axis]) / (2.0 * amplitude_rad)
            derivative_details.append({
                "rateAxis": name,
                "centralDifferenceRateDegPerSecond": amplitude_deg,
                "bodyTorqueDerivativeNmPerRadPerSecondXYZ": [derivatives[row][axis] for row in range(3)],
            })
        roll_from_yaw = derivatives[1][2]
        yaw_from_yaw = derivatives[2][2]
        yaw_from_roll = derivatives[2][1]
        roll_from_roll = derivatives[1][1]
        ratio = lambda numerator, denominator: abs(numerator / denominator) if abs(denominator) > 1e-9 else None
        report["staticZeroRateTorqueNmBodyXYZ"] = baseline_torque
        report["rateDampingDerivativeMatrixNmPerRadPerSecond"] = derivatives
        report["rateDampingDerivativeColumns"] = derivative_details
        report["crossAxisRatiosAtThisOneHypotheticalState"] = {
            "rollTorqueFromYawRateToYawTorqueFromYawRate": ratio(roll_from_yaw, yaw_from_yaw),
            "yawTorqueFromRollRateToRollTorqueFromRollRate": ratio(yaw_from_roll, roll_from_roll),
            "note": "Ratios describe simulated aerodynamic damping only; they do not include wheel/RCS actuator torque or the measured live yaw-pulse torque.",
        }
        linearity = {}
        for axis_name, axis in (("bodyY_roll", 1), ("bodyZ_yaw", 2)):
            plus_small = sample_by_case[f"{axis_name}_positive_measured-envelope"]["torqueNm_bodyXYZ"]
            minus_small = sample_by_case[f"{axis_name}_negative_measured-envelope"]["torqueNm_bodyXYZ"]
            plus_large = sample_by_case[f"{axis_name}_positive_1deg_s_linearity_check"]["torqueNm_bodyXYZ"]
            minus_large = sample_by_case[f"{axis_name}_negative_1deg_s_linearity_check"]["torqueNm_bodyXYZ"]
            small_amp = math.radians(max(float(pulse_rates[axis]), 0.01))
            large_amp = math.radians(1.0)
            small_derivative = [(plus_small[i] - minus_small[i]) / (2.0 * small_amp) for i in range(3)]
            large_derivative = [(plus_large[i] - minus_large[i]) / (2.0 * large_amp) for i in range(3)]
            linearity[axis_name] = {
                "measuredEnvelopeDerivativeNmPerRadPerSecondXYZ": small_derivative,
                "oneDegPerSecondDerivativeNmPerRadPerSecondXYZ": large_derivative,
                "relativeDifferenceXYZ": [
                    abs(large_derivative[i] - small_derivative[i]) / max(abs(small_derivative[i]), 1e-9)
                    for i in range(3)
                ],
            }
        report["rateLinearityCheck"] = linearity
        report["hypotheticalState"] = {key: value for key, value in target.items() if key not in {"position", "rotation", "velocity", "airVelocity"}}
        report["loadedMassAssessment"] = {
            "pausedLoadedMassKg": before["mass_kg"],
            "checkpointProfileMassKg": float(profile["mass"]),
            "massMatchesSettledProfile": abs(before["mass_kg"] - float(profile["mass"])) < 1.0,
            "note": "A full-mass checkpoint match was required before wrench simulation. Torque derivatives remain ideal rigid-body KSP aero outputs and are not live angular acceleration measurements.",
        }
        report["gridSummary"] = {
            "sampleCount": len(report["samples"]),
            "apiCallCount": len(report["samples"]),
            "elapsedWallSeconds": time.monotonic() - started,
        }
        report["status"] = "complete"
    except BaseException as exc:
        report["status"] = "failed"
        report["error"] = f"{type(exc).__name__}: {exc}"
        raise
    finally:
        if conn is not None:
            try:
                after = state_snapshot(conn, sc, vessel, body, frame)
                actuator_after = read_actuator_snapshot(vessel, config)
                report["liveStateAfter"] = after
                report["liveActuatorStateAfter"] = actuator_after
                report["liveStateUnchanged"] = report.get("liveStateBefore") == after
                report["liveActuatorStateUnchanged"] = report.get("liveActuatorStateBefore") == actuator_after
                report["checkpointSHA256After"] = hashlib.sha256(checkpoint_path.read_bytes()).hexdigest()
                report["checkpointUnchanged"] = report["checkpointSHA256After"] == checkpoint_hash_before == profile["saveSHA256"]
                report["kspProcessPIDsAfter"] = pids_named_ksp()
                report["singleKspProcessPreserved"] = report.get("kspProcessPIDsBefore") == report["kspProcessPIDsAfter"] and len(report["kspProcessPIDsAfter"]) == 1
                if not report["liveStateUnchanged"] or not report["liveActuatorStateUnchanged"] or not report["checkpointUnchanged"] or not report["singleKspProcessPreserved"]:
                    report["status"] = "failed"
                    report["invariantFailure"] = "The live state, controls, checkpoint, or one-KSP-process invariant changed."
            except Exception as post_error:
                report["postProbeCaptureError"] = f"{type(post_error).__name__}: {post_error}"
                report["status"] = "failed"
            try:
                conn.close()
            except Exception:
                pass
        report["sourceSHA256"] = hashlib.sha256(Path(__file__).read_bytes()).hexdigest()
        OUTPUT_PATH.parent.mkdir(parents=True, exist_ok=True)
        OUTPUT_PATH.write_text(json.dumps(report, indent=2, allow_nan=False) + "\n")
    print(json.dumps({
        "status": report["status"],
        "output": str(OUTPUT_PATH),
        "samples": len(report["samples"]),
        "liveStateUnchanged": report.get("liveStateUnchanged"),
        "checkpointUnchanged": report.get("checkpointUnchanged"),
    }, separators=(",", ":")))
    return 0 if report["status"] == "complete" else 1


if __name__ == "__main__":
    raise SystemExit(main())
