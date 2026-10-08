#!/usr/bin/env python3
"""Read-only KSP wrench grid around archived measured original-STS states.

The input run explicitly bypassed the shadow gate. Results are diagnostic
counterfactuals only and cannot qualify the campaign.
"""

from __future__ import annotations

import argparse
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
    read_actuator_snapshot,
)

INPUT_PATH = ROOT / "FlightLogs/2026-10-07T21-34-08Z-STS-vehicle.jsonl"
OUTPUT_PATH = ROOT / "Docs/Experiments/STS-Entry-20261009/live-measured-run9-wrench-grid-20261009.json"
PROFILE_PATH = ROOT / "Configuration/checkpoints/sts-campaign-72km-20261008.physics.json"
CONFIG_PATH = ROOT / "Configuration/sts.json"
TARGET_UTS = (10513.551826182, 10515.5918261821, 10517.6718261821, 10519.7118261822, 10521.7118261822)
MACH_FACTORS = (0.95, 1.0, 1.05)
AOA_OFFSETS_DEG = (-2.0, 0.0, 2.0)
BETA_OFFSETS_DEG = (-2.0, 0.0, 2.0)
BANK_OFFSETS_DEG = (-10.0, 0.0, 10.0)


def canonical_to_krpc(vector):
    """Invert the C bridge's kRPC -> canonical [x,z,y] axis mapping."""
    return [float(vector[0]), float(vector[2]), float(vector[1])]


def deep_merge(target: dict, update: dict) -> None:
    for key, value in update.items():
        if isinstance(value, dict) and isinstance(target.get(key), dict):
            deep_merge(target[key], value)
        else:
            target[key] = value


def read_measured_states(path: Path) -> tuple[list[dict], str, list[dict]]:
    digest = hashlib.sha256(path.read_bytes()).hexdigest()
    state: dict = {}
    rows: list[dict] = []
    bypass_events: list[dict] = []
    session = None
    for line in path.read_text().splitlines():
        record = json.loads(line)
        if record.get("recordType") == "sessionStart":
            session = record
        if "shadow-gate-bypassed" in str(record.get("event", "")):
            bypass_events.append({
                "recordSequence": record.get("recordSequence"),
                "tickSequence": record.get("tickSequence"),
                "ut": record.get("ut"),
                "event": record.get("event"),
            })
        fields = record.get("fields")
        if not fields:
            continue
        if record.get("recordType") == "vehicleKeyframe":
            state = json.loads(json.dumps(fields))
        else:
            deep_merge(state, fields)
        if not all(key in state for key in ("position", "motion", "attitude", "aero", "state", "vehicle")):
            continue
        aero = state["aero"]
        motion = state["motion"]
        attitude = state["attitude"]
        position = state["position"]
        if float(aero.get("dynamicPressure", 0.0) or 0.0) < 1000.0:
            continue
        if float(aero.get("mach", 0.0) or 0.0) < 3.0:
            continue
        if abs(float(state.get("vehicle", {}).get("currentThrust", 0.0) or 0.0)) > 1.0:
            continue
        if bool(state.get("state", {}).get("airbrakes", False)) or bool(state.get("command", {}).get("airbrakes", False)):
            continue
        required = (
            position.get("inertial"), motion.get("velocity"), motion.get("trueAirSpeed"),
            attitude.get("angleOfAttack"), attitude.get("sideslip"), attitude.get("roll"),
            attitude.get("bodyPitchRate"), attitude.get("bodyRollRate"), attitude.get("bodyYawRate"),
            aero.get("mach"), aero.get("dynamicPressure"), aero.get("liftForce"), aero.get("dragForce"),
        )
        if any(value is None for value in required):
            continue
        rows.append({
            "recordSequence": record.get("recordSequence"),
            "tickSequence": record.get("tickSequence"),
            "ut": float(record.get("ut", 0.0)),
            "positionM_canonicalXYZ": [float(value) for value in position["inertial"]],
            "velocityMps_canonicalXYZ": [float(value) for value in motion["velocity"]],
            "positionM_nonrotatingKrpcXYZ": canonical_to_krpc(position["inertial"]),
            "velocityMps_nonrotatingKrpcXYZ": canonical_to_krpc(motion["velocity"]),
            "trueAirSpeedMps_reported": float(motion["trueAirSpeed"]),
            "altitudeM": float(position["altitude"]),
            "machReported": float(aero["mach"]),
            "dynamicPressurePaReported": float(aero["dynamicPressure"]),
            "angleOfAttackDeg": float(attitude["angleOfAttack"]),
            "sideslipDeg": float(attitude["sideslip"]),
            "bankDeg": float(attitude["roll"]),
            "bodyRatesDegPerSecond_pitchRollYaw": [
                float(attitude["bodyPitchRate"]),
                float(attitude["bodyRollRate"]),
                float(attitude["bodyYawRate"]),
            ],
            "liftForceN_reported": float(aero["liftForce"]),
            "dragForceN_reported": float(aero["dragForce"]),
            "engineThrustN": float(state.get("vehicle", {}).get("currentThrust", 0.0) or 0.0),
            "speedbrakeCommanded": bool(state.get("command", {}).get("airbrakes", False)),
            "speedbrakeState": bool(state.get("state", {}).get("airbrakes", False)),
            "rcsStateRecorded": False,
        })
    if not session or session.get("vessel") != "STS":
        raise RuntimeError("Archived telemetry is not confirmed as the original STS")
    if not bypass_events:
        raise RuntimeError("Expected explicit shadow-gate bypass provenance in archived telemetry")
    selected = []
    for target_ut in TARGET_UTS:
        match = min(rows, key=lambda row: abs(row["ut"] - target_ut), default=None)
        if match is None or abs(match["ut"] - target_ut) > 0.6:
            raise RuntimeError(f"No eligible measured row within 0.6 s of UT {target_ut}")
        selected.append(match)
    if len({row["recordSequence"] for row in selected}) != len(selected):
        raise RuntimeError("Selected measurement anchors are not distinct")
    return selected, digest, bypass_events


def quaternion_from_vectors(source, target):
    """Shortest unit quaternion mapping one 3-vector to another."""
    a, b = unit(source), unit(target)
    cosine = max(-1.0, min(1.0, dot(a, b)))
    if cosine < -0.999999:
        basis = (1.0, 0.0, 0.0) if abs(a[0]) < 0.8 else (0.0, 1.0, 0.0)
        axis = unit(cross(a, basis))
        return (axis[0], axis[1], axis[2], 0.0)
    axis = cross(a, b)
    q = (axis[0], axis[1], axis[2], 1.0 + cosine)
    length = norm(q)
    return tuple(value / length for value in q)


def axis_angle(axis, angle_rad):
    direction = unit(axis)
    half = 0.5 * angle_rad
    return (direction[0] * math.sin(half), direction[1] * math.sin(half), direction[2] * math.sin(half), math.cos(half))


def measured_attitude(position, air_velocity, aoa_deg, beta_deg, bank_deg):
    """Reconstruct orientation from logged air-relative angles and KSP-sign bank."""
    alpha = math.radians(aoa_deg)
    beta = math.radians(beta_deg)
    body_air = unit((math.sin(beta), math.cos(beta) * math.cos(alpha), math.cos(beta) * math.sin(alpha)))
    air_direction = unit(air_velocity)
    base = quaternion_from_vectors(body_air, air_direction)
    theta = 0.0
    for _ in range(24):
        rotation = quaternion_multiply(axis_angle(air_direction, math.radians(theta)), base)
        error = wrap_degrees(bank_degrees(rotation, position) - bank_deg)
        if abs(error) < 1e-7:
            break
        step = 0.01
        plus = quaternion_multiply(axis_angle(air_direction, math.radians(theta + step)), base)
        minus = quaternion_multiply(axis_angle(air_direction, math.radians(theta - step)), base)
        slope = wrap_degrees(bank_degrees(plus, position) - bank_degrees(minus, position)) / (2.0 * step)
        if not math.isfinite(slope) or abs(slope) < 1e-6:
            raise RuntimeError("Could not solve measured bank orientation")
        theta -= error / slope
    rotation = quaternion_multiply(axis_angle(air_direction, math.radians(theta)), base)
    achieved_aoa, achieved_beta, _ = angles_degrees(rotation, air_velocity)
    achieved_bank = bank_degrees(rotation, position)
    if max(abs(achieved_aoa - aoa_deg), abs(achieved_beta - beta_deg), abs(wrap_degrees(achieved_bank - bank_deg))) > 1e-5:
        raise RuntimeError("Reconstructed quaternion does not match logged AoA, sideslip, and bank")
    return rotation, (achieved_aoa, achieved_beta, achieved_bank)


def drag_lift(force, air_velocity):
    direction = unit(air_velocity)
    axial = dot(force, direction)
    lift_vector = tuple(force[i] - axial * direction[i] for i in range(3))
    return -axial, norm(lift_vector)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, default=OUTPUT_PATH)
    args = parser.parse_args()

    config = json.loads(CONFIG_PATH.read_text())
    profile = json.loads(PROFILE_PATH.read_text())
    measured_states, log_hash, bypass_events = read_measured_states(INPUT_PATH)
    save_path = Path(profile["savePath"])
    checkpoint_hash_before = hashlib.sha256(save_path.read_bytes()).hexdigest()
    if checkpoint_hash_before != profile["saveSHA256"]:
        raise RuntimeError("Named campaign checkpoint SHA does not match its physics profile")

    report = {
        "status": "running",
        "classification": "UNQUALIFIED_COUNTERFACTUAL_SIMULATION_FROM_BYPASSED_MEASUREMENTS",
        "purpose": "Evaluate whether KSP's read-only simulated aerodynamic wrench API can replay measured original-STS atmospheric kinematics and provide a local Mach/AoA/sideslip/bank grid.",
        "qualification": {
            "qualified": False,
            "inputRunShadowGateBypassed": True,
            "measuredFlightStatesUsedAsInputs": True,
            "simulatedWrenchIsMeasuredFlightData": False,
            "flightControlOrActuation": False,
            "landingOrRolloutEvidence": False,
            "note": "All wrench outputs are counterfactual API simulations. The archived source run explicitly bypassed the shadow gate and cannot qualify the campaign.",
        },
        "provenance": {
            "repository": str(ROOT),
            "gitHead": subprocess.run(["git", "rev-parse", "HEAD"], cwd=ROOT, capture_output=True, text=True, check=True).stdout.strip(),
            "inputTelemetry": str(INPUT_PATH.relative_to(ROOT)),
            "inputTelemetrySHA256": log_hash,
            "inputSessionId": "2026-10-07T21-34-08Z-29000-108766028242",
            "inputGateBypassEvents": bypass_events,
            "configuration": str(CONFIG_PATH.relative_to(ROOT)),
            "configurationSHA256": hashlib.sha256(CONFIG_PATH.read_bytes()).hexdigest(),
            "probeSourceSHA256": hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
            "helperSourceSHA256": {
                "probe_sts_aero_wrench_grid.py": hashlib.sha256(Path(__file__).with_name("probe_sts_aero_wrench_grid.py").read_bytes()).hexdigest(),
                "probe_sts_shadow_witness_wrench.py": hashlib.sha256(Path(__file__).with_name("probe_sts_shadow_witness_wrench.py").read_bytes()).hexdigest(),
                "ksp_test_guard.py": hashlib.sha256(Path(__file__).with_name("ksp_test_guard.py").read_bytes()).hexdigest(),
            },
            "coordinateBoundarySourceSHA256": {
                "CLanding/transport/client/session.inc": hashlib.sha256((ROOT / "CLanding/transport/client/session.inc").read_bytes()).hexdigest(),
                "CLanding/transport/client/telemetry.inc": hashlib.sha256((ROOT / "CLanding/transport/client/telemetry.inc").read_bytes()).hexdigest(),
            },
            "vehicleModelId": config["vehicle"]["modelId"],
            "checkpointProfile": str(PROFILE_PATH.relative_to(ROOT)),
            "checkpointName": profile["saveName"],
            "checkpointSHA256Before": checkpoint_hash_before,
            "api": "SpaceCenter.Flight.simulate_aerodynamic_wrench_at",
            "apiReferenceFrame": "CelestialBody.non_rotating_reference_frame",
            "historicalInputPositionVelocity": "The archived JSON contains internal canonical XYZ vectors. Before the API call, the probe reverses CLanding/transport/client/session.inc tuple_to_canonical's [x,z,y] axis map to recover raw kRPC body-nonrotating XYZ.",
            "attitudeReconstruction": "Quaternion is solved from each measured air-relative AoA/sideslip, recorded inertial air-velocity direction, and KSP-sign bank. Reconstructed angles are numerically checked before each query.",
            "machPolicy": "Recorded KSP Mach is scaled by the same factor as the measured air-velocity vector at the same historical position and UT.",
            "angularVelocityPolicy": "Zero hypothetical angular velocity to isolate static aerodynamic loads; measured rate reconstruction and aerodynamic damping are excluded.",
            "surfaceStatePolicy": "The loaded checkpoint STS has neutral control surfaces. Historical run-9 surface deflections were not recorded, so simulated wrench is not a matched replay of actual actuator state. RCS state was not recorded per sample.",
            "noFlightMutation": "No setters for UT, pause, position, velocity, rotation, controls, action groups, throttle, SAS, RCS, or save state are called.",
            "inputMeasuredStates": measured_states,
        },
        "gridDefinition": {
            "anchorCount": len(measured_states),
            "machFactors": list(MACH_FACTORS),
            "angleOfAttackOffsetsDeg": list(AOA_OFFSETS_DEG),
            "sideslipOffsetsDeg": list(BETA_OFFSETS_DEG),
            "bankOffsetsDeg": list(BANK_OFFSETS_DEG),
            "angleOfAttackBoundDeg": [0.0, float(config["vehicle"]["maximumAngleOfAttack"])],
            "sideslipBoundDeg": [-5.0, 5.0],
            "bankBoundDeg": [-40.0, 40.0],
            "pressureBoundPa": float(config["vehicle"]["maximumDynamicPressure"]),
        },
        "liveKspBefore": None,
        "liveActuatorsBefore": None,
        "liveKspAfter": None,
        "liveActuatorsAfter": None,
        "liveStateUnchanged": False,
        "checkpointSHA256After": None,
        "checkpointUnchanged": False,
        "kspProcessPIDsBefore": [],
        "kspProcessPIDsAfter": [],
        "singleKSPProcessPreserved": False,
        "gridSummary": {},
        "samples": [],
        "errors": [],
    }
    conn = None
    started = time.monotonic()
    try:
        pids_before = pids_named_ksp()
        if len(pids_before) != 1:
            raise RuntimeError(f"Expected exactly one live KSP process, found {pids_before}")
        report["kspProcessPIDsBefore"] = pids_before
        conn = connect("STS measured-run9 read-only wrench grid", timeout=8)
        sc = conn.space_center
        vessel = sc.active_vessel
        if vessel is None or vessel.name != "STS" or config["vehicle"]["modelId"] != "STS":
            raise RuntimeError("Active vessel and configured vehicle must both be the original STS")
        body = vessel.orbit.body
        if body.name != "Kerbin":
            raise RuntimeError(f"Expected Kerbin, found {body.name}")
        frame = body.non_rotating_reference_frame
        flight = vessel.flight(frame)
        before = state_snapshot(conn, sc, vessel, body, frame)
        actuators_before = read_actuator_snapshot(vessel, config)
        action_groups_before = [bool(vessel.control.get_action_group(i)) for i in range(10)]
        report["liveKspBefore"] = before
        report["liveActuatorsBefore"] = {**actuators_before, "actionGroups0Through9": action_groups_before}
        if not before["paused"]:
            raise RuntimeError("Refusing diagnostic calls while live KSP is unpaused")
        if before["ut"] < 0.0 or abs(before["ut"] - float(profile["savedUT"])) > 2.0:
            raise RuntimeError("Live KSP is outside the named checkpoint UT tolerance")
        if abs(before["altitude_m"] - 72000.0) > 500.0:
            raise RuntimeError("Live STS is not at the settled 72 km checkpoint altitude")
        if before["part_count"] != int(profile["partCount"]):
            raise RuntimeError("Live STS part count does not match the checkpoint profile")
        if abs(before["mass_kg"] - float(profile["mass"])) > 1.0:
            raise RuntimeError("Live STS mass does not match the checkpoint profile")
        if before["sas"] or before["rcs"] or before["wheel_brakes"]:
            raise RuntimeError("Refusing diagnostic calls with SAS, RCS, or wheel brakes enabled")
        if any(abs(value) > 1e-9 for value in before["controls"].values()) or abs(before["thrust_n"]) > 1.0:
            raise RuntimeError("Refusing diagnostic calls with nonzero input or thrust")
        if any(action_groups_before):
            raise RuntimeError("Refusing diagnostic calls while an action group is enabled")
        if actuators_before["speedbrakeActionGroupEnabled"]:
            raise RuntimeError("Refusing diagnostic calls with the configured speedbrake action group enabled")
        if any(item["deployed"] or abs(item["deflection"]) > 1e-9 for item in actuators_before["controlSurfaces"]):
            raise RuntimeError("Refusing diagnostic calls with deployed or deflected control surfaces")

        atmosphere_omega = tuple(float(value) for value in body.angular_velocity(frame))
        maximum_aoa = float(config["vehicle"]["maximumAngleOfAttack"])
        maximum_q = float(config["vehicle"]["maximumDynamicPressure"])
        max_angle_error = 0.0
        max_bank_error = 0.0
        call_count = 0
        airspeed_frame_checks = []
        for anchor_index, anchor in enumerate(measured_states):
            position = tuple(anchor["positionM_nonrotatingKrpcXYZ"])
            measured_velocity = tuple(anchor["velocityMps_nonrotatingKrpcXYZ"])
            atmo_velocity = cross(atmosphere_omega, position)
            measured_air_velocity = tuple(measured_velocity[i] - atmo_velocity[i] for i in range(3))
            if norm(measured_air_velocity) < 1.0:
                raise RuntimeError("Archived measured velocity produced an invalid air-relative vector")
            computed_air_speed = norm(measured_air_velocity)
            reported_air_speed = anchor["trueAirSpeedMps_reported"]
            speed_error_fraction = (computed_air_speed - reported_air_speed) / max(reported_air_speed, 1.0)
            airspeed_frame_checks.append({
                "anchorIndex": anchor_index,
                "ut": anchor["ut"],
                "computedFromNonrotatingVectorAndBodySpinMps": computed_air_speed,
                "reportedTrueAirSpeedMps": reported_air_speed,
                "relativeDifference": speed_error_fraction,
            })
            if abs(speed_error_fraction) > 0.02:
                raise RuntimeError("Corrected frame conversion does not reconcile logged inertial velocity with reported true airspeed")
            factors_and_values = []
            for factor in MACH_FACTORS:
                if anchor["dynamicPressurePaReported"] * factor * factor > maximum_q:
                    raise RuntimeError("Hypothetical pressure estimate exceeds configured bound")
                for aoa_offset in AOA_OFFSETS_DEG:
                    aoa = max(0.0, min(maximum_aoa, anchor["angleOfAttackDeg"] + aoa_offset))
                    for beta_offset in BETA_OFFSETS_DEG:
                        beta = max(-5.0, min(5.0, anchor["sideslipDeg"] + beta_offset))
                        for bank_offset in BANK_OFFSETS_DEG:
                            bank = max(-40.0, min(40.0, anchor["bankDeg"] + bank_offset))
                            factors_and_values.append((factor, aoa, beta, bank, aoa_offset, beta_offset, bank_offset))
            unique = {}
            for values in factors_and_values:
                unique[tuple(round(value, 8) for value in values[:4])] = values
            for factor, aoa, beta, bank, aoa_offset, beta_offset, bank_offset in unique.values():
                air_velocity = tuple(value * factor for value in measured_air_velocity)
                rotation, achieved = measured_attitude(position, air_velocity, aoa, beta, bank)
                angle_error = max(abs(achieved[0] - aoa), abs(achieved[1] - beta))
                bank_error = abs(wrap_degrees(achieved[2] - bank))
                max_angle_error = max(max_angle_error, angle_error)
                max_bank_error = max(max_bank_error, bank_error)
                if angle_error > 1e-5 or bank_error > 1e-5:
                    raise RuntimeError("Hypothetical quaternion failed its AoA/sideslip/bank reconstruction check")
                inertial_velocity = tuple(air_velocity[i] + atmo_velocity[i] for i in range(3))
                wrench = flight.simulate_aerodynamic_wrench_at(
                    body, position, inertial_velocity, rotation, (0.0, 0.0, 0.0), anchor["ut"]
                )
                force = tuple(float(value) for value in wrench[0])
                torque = tuple(float(value) for value in wrench[1])
                drag_n, lift_n = drag_lift(force, air_velocity)
                call_count += 1
                report["samples"].append({
                    "anchorIndex": anchor_index,
                    "anchorRecordSequence": anchor["recordSequence"],
                    "anchorUT": anchor["ut"],
                    "anchorAltitudeM": anchor["altitudeM"],
                    "targetMachEstimate": anchor["machReported"] * factor,
                    "machScaleFromMeasuredState": factor,
                    "targetAngleOfAttackDeg": aoa,
                    "targetSideslipDeg": beta,
                    "targetBankDeg": bank,
                    "requestedOffsetsDeg": {"aoa": aoa_offset, "sideslip": beta_offset, "bank": bank_offset},
                    "achievedAnglesDeg": {"aoa": achieved[0], "sideslip": achieved[1], "bank": achieved[2]},
                    "airSpeedVectorNormMps": norm(air_velocity),
                    "simulatedForceN_nonrotating": force,
                    "simulatedTorqueNm_nonrotating": torque,
                    "simulatedForceN_bodyXYZ": inverse_rotate(rotation, force),
                    "simulatedTorqueNm_bodyXYZ": inverse_rotate(rotation, torque),
                    "simulatedDragN_alongRelativeAirflow": drag_n,
                    "simulatedLiftN_perpendicularToRelativeAirflow": lift_n,
                    "hypotheticalAngularVelocityRadPerS": [0.0, 0.0, 0.0],
                })
                if not conn.krpc.paused or abs(float(sc.ut) - before["ut"]) > 1e-9:
                    raise RuntimeError("Live KSP pause or UT changed during the read-only wrench grid")

        report["provenance"]["airVelocityFrameChecks"] = airspeed_frame_checks
        report["gridSummary"] = {
            "sampleCount": call_count,
            "anchorCount": len(measured_states),
            "elapsedWallSeconds": time.monotonic() - started,
            "maximumAchievedAoAOrSideslipErrorDeg": max_angle_error,
            "maximumAchievedBankErrorDeg": max_bank_error,
            "bodyAngularVelocityRadPerS_nonrotating": atmosphere_omega,
        }
        after = state_snapshot(conn, sc, vessel, body, frame)
        actuators_after = read_actuator_snapshot(vessel, config)
        action_groups_after = [bool(vessel.control.get_action_group(i)) for i in range(10)]
        report["liveKspAfter"] = after
        report["liveActuatorsAfter"] = {**actuators_after, "actionGroups0Through9": action_groups_after}
        report["liveStateUnchanged"] = (
            before == after and report["liveActuatorsBefore"] == report["liveActuatorsAfter"]
        )
        pids_after = pids_named_ksp()
        report["kspProcessPIDsAfter"] = pids_after
        report["singleKSPProcessPreserved"] = len(pids_after) == 1 and pids_after == pids_before
        report["checkpointSHA256After"] = hashlib.sha256(save_path.read_bytes()).hexdigest()
        report["checkpointUnchanged"] = report["checkpointSHA256After"] == checkpoint_hash_before
        if not report["liveStateUnchanged"] or not report["singleKSPProcessPreserved"] or not report["checkpointUnchanged"]:
            raise RuntimeError("Read-only API probe changed live state, process count, actuators, or checkpoint hash")

        centers = [sample for sample in report["samples"] if
            abs(sample["machScaleFromMeasuredState"] - 1.0) < 1e-9 and
            abs(sample["requestedOffsetsDeg"]["aoa"]) < 1e-9 and
            abs(sample["requestedOffsetsDeg"]["sideslip"]) < 1e-9 and
            abs(sample["requestedOffsetsDeg"]["bank"]) < 1e-9]
        report["measuredCenterComparisons"] = []
        for anchor_index, center in enumerate(centers):
            anchor = measured_states[anchor_index]
            force_delta_lift = center["simulatedLiftN_perpendicularToRelativeAirflow"] - anchor["liftForceN_reported"]
            force_delta_drag = center["simulatedDragN_alongRelativeAirflow"] - anchor["dragForceN_reported"]
            report["measuredCenterComparisons"].append({
                "anchorIndex": anchor_index,
                "recordSequence": anchor["recordSequence"],
                "ut": anchor["ut"],
                "reportedDynamicPressurePa": anchor["dynamicPressurePaReported"],
                "reportedMach": anchor["machReported"],
                "reportedTrueAirSpeedMps": anchor["trueAirSpeedMps_reported"],
                "nonrotatingVelocityAirSpeedNormMps": norm(tuple(
                    anchor["velocityMps_nonrotatingKrpcXYZ"][i] - cross(atmosphere_omega, tuple(anchor["positionM_nonrotatingKrpcXYZ"]))[i]
                    for i in range(3)
                )),
                "reportedLiftN": anchor["liftForceN_reported"],
                "simulatedLiftN": center["simulatedLiftN_perpendicularToRelativeAirflow"],
                "liftDeltaN": force_delta_lift,
                "reportedDragN": anchor["dragForceN_reported"],
                "simulatedDragN": center["simulatedDragN_alongRelativeAirflow"],
                "dragDeltaN": force_delta_drag,
                "note": "Comparison is diagnostic only: recorded control-surface deflections and per-sample RCS state are unavailable; wrench uses the current checkpoint's neutral surfaces and zero hypothetical angular rate.",
            })
        report["status"] = "complete"
        report["output"] = str(args.output)
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(json.dumps(report, indent=2, allow_nan=False) + "\n")
        print(json.dumps({
            "status": report["status"],
            "classification": report["classification"],
            "output": report["output"],
            "sampleCount": call_count,
            "anchorCount": len(measured_states),
            "checkpointUnchanged": report["checkpointUnchanged"],
            "liveStateUnchanged": report["liveStateUnchanged"],
            "singleKSPProcessPreserved": report["singleKSPProcessPreserved"],
            "elapsedWallSeconds": report["gridSummary"]["elapsedWallSeconds"],
        }, separators=(",", ":")))
        return 0
    except Exception as exc:
        report["status"] = "error"
        report["errors"].append(str(exc))
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(json.dumps(report, indent=2, allow_nan=False) + "\n")
        raise
    finally:
        if conn is not None:
            conn.close()


if __name__ == "__main__":
    raise SystemExit(main())
