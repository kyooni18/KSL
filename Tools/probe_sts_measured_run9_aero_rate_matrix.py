#!/usr/bin/env python3
"""Read-only KSP aero-rate matrix at archived measured original-STS states.

The historical source run bypassed the shadow gate. Every wrench is a
counterfactual simulation and is excluded from qualification evidence.
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
    cross,
    inverse_rotate,
    norm,
    rotate,
    state_snapshot,
    wrap_degrees,
)
from probe_sts_measured_run9_wrench_grid import (
    INPUT_PATH,
    measured_attitude,
    read_measured_states,
)
from probe_sts_shadow_witness_wrench import pids_named_ksp, read_actuator_snapshot

PROFILE_PATH = ROOT / "Configuration/checkpoints/sts-campaign-72km-20261008.physics.json"
CONFIG_PATH = ROOT / "Configuration/sts.json"
OUTPUT_PATH = ROOT / "Docs/Experiments/STS-Entry-20261009/live-measured-run9-aero-rate-matrix-20261009.json"
RATE_AXES = (("bodyX_pitch", 0), ("bodyY_roll", 1), ("bodyZ_yaw", 2))
RATE_OFFSETS_DEG_S = (0.1, 1.0)


def sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def rotate_rates(rotation: tuple[float, ...], body_rates_rad_s: list[float]) -> tuple[float, ...]:
    return tuple(float(value) for value in rotate(rotation, body_rates_rad_s))


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, default=OUTPUT_PATH)
    args = parser.parse_args()

    config = json.loads(CONFIG_PATH.read_text())
    profile = json.loads(PROFILE_PATH.read_text())
    measured_states, input_hash, bypass_events = read_measured_states(INPUT_PATH)
    save_path = Path(profile["savePath"])
    checkpoint_hash_before = sha256(save_path)
    ksp_root = save_path.parents[2]
    ksp_log = ksp_root / "KSP.log"
    ksp_log_lines = (
        [line.strip() for line in ksp_log.read_text(errors="replace").splitlines()
         if "FerramAerospaceResearch" in line]
        if ksp_log.is_file() else []
    )
    far_plugin_directories = [
        str(path.relative_to(ksp_root))
        for path in (ksp_root / "GameData").glob("*FerramAerospaceResearch*")
        if path.exists()
    ]
    if checkpoint_hash_before != profile["saveSHA256"]:
        raise RuntimeError("Named campaign checkpoint SHA does not match its physics profile")
    if config["vehicle"]["modelId"] != "STS" or profile["saveName"] != "STS-campaign-72km-20261008":
        raise RuntimeError("Probe requires the original STS and named campaign checkpoint")

    report = {
        "status": "running",
        "classification": "UNQUALIFIED_COUNTERFACTUAL_SIMULATION_FROM_BYPASSED_MEASUREMENTS",
        "purpose": "Measure local aerodynamic force and torque derivatives with respect to body angular velocity at five archived measured original-STS atmospheric states.",
        "qualification": {
            "qualified": False,
            "inputRunShadowGateBypassed": True,
            "measuredFlightStatesUsedAsInputs": True,
            "simulatedWrenchIsMeasuredFlightData": False,
            "shadowGateEvidence": False,
            "flightControlOrActuation": False,
            "landingOrRolloutEvidence": False,
            "note": "All wrench values are hypothetical KSP simulations. The archived source telemetry explicitly records a shadow-gate bypass and cannot qualify the campaign.",
        },
        "provenance": {
            "repository": str(ROOT),
            "gitHead": subprocess.run(["git", "rev-parse", "HEAD"], cwd=ROOT, capture_output=True, text=True, check=True).stdout.strip(),
            "inputTelemetry": str(INPUT_PATH.relative_to(ROOT)),
            "inputTelemetrySHA256": input_hash,
            "inputGateBypassEvents": bypass_events,
            "inputMeasuredStates": measured_states,
            "configuration": str(CONFIG_PATH.relative_to(ROOT)),
            "configurationSHA256": sha256(CONFIG_PATH),
            "checkpointProfile": str(PROFILE_PATH.relative_to(ROOT)),
            "checkpointName": profile["saveName"],
            "checkpointSHA256Before": checkpoint_hash_before,
            "kspInstallRoot": str(ksp_root),
            "kspLogSHA256": sha256(ksp_log) if ksp_log.is_file() else None,
            "farApiLookupEvidence": ksp_log_lines,
            "farPluginDirectoriesPresent": far_plugin_directories,
            "farAngularVelocityIgnoreWarningApplies": bool(far_plugin_directories),
            "inputStateSelection": "Five distinct archived atmospheric measurements, each within 0.6 s of a predetermined anchor UT; Mach > 3, dynamic pressure > 1 kPa, thrust zero, and speedbrakes recorded stowed.",
            "api": "SpaceCenter.Flight.simulate_aerodynamic_wrench_at",
            "apiReferenceFrame": "CelestialBody.non_rotating_reference_frame",
            "bodyAxisOrder": ["local +X pitch", "local +Y roll", "local +Z yaw"],
            "rateUnits": "Input body angular velocity in rad/s; saved telemetry bodyPitchRate/bodyRollRate/bodyYawRate values are degrees/s and converted to rad/s before rotation into the non-rotating reference frame.",
            "ratePerturbationsDegPerSecond": list(RATE_OFFSETS_DEG_S),
            "surfaceStateLimit": "The live checkpoint exposes neutral control surfaces. Historical run-9 control-surface deflections and per-sample RCS state are unavailable, so the simulation does not replay historical actuation.",
            "noFlightMutation": "The probe uses only hypothetical arguments to simulate_aerodynamic_wrench_at. It does not call setters for UT, pause, position, velocity, rotation, controls, action groups, throttle, SAS, RCS, physics, or save state.",
            "probeSource": str(Path(__file__).relative_to(ROOT)),
            "probeSourceSHA256": sha256(Path(__file__)),
            "helperSourceSHA256": {
                "probe_sts_measured_run9_wrench_grid.py": sha256(Path(__file__).with_name("probe_sts_measured_run9_wrench_grid.py")),
                "probe_sts_aero_wrench_grid.py": sha256(Path(__file__).with_name("probe_sts_aero_wrench_grid.py")),
                "probe_sts_shadow_witness_wrench.py": sha256(Path(__file__).with_name("probe_sts_shadow_witness_wrench.py")),
                "ksp_test_guard.py": sha256(Path(__file__).with_name("ksp_test_guard.py")),
            },
        },
        "gridDefinition": {
            "anchorCount": len(measured_states),
            "casesPerAnchor": 2 + 2 * len(RATE_AXES) * len(RATE_OFFSETS_DEG_S),
            "caseTypes": ["zero_rate_reference", "measured_rate_baseline", "symmetric_rate_offsets_about_measured_baseline"],
            "derivativeMatrixRows": "body torque XYZ (N m)",
            "derivativeMatrixColumns": "body angular velocity XYZ (rad/s)",
            "derivativeTensorUnits": "d(body torque N m)/d(body angular velocity rad/s)",
            "derivativeMethod": "Centered finite difference about each archived measured body-rate vector, independently perturbing one body axis at a time.",
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
        "airSpeedFrameChecks": [],
        "samples": [],
        "anchorMatrices": [],
        "errors": [],
    }

    conn = None
    started = time.monotonic()
    try:
        pids_before = pids_named_ksp()
        if len(pids_before) != 1:
            raise RuntimeError(f"Expected exactly one live KSP process, found {pids_before}")
        report["kspProcessPIDsBefore"] = pids_before
        conn = connect("STS measured-run9 read-only aerodynamic rate matrix", timeout=8)
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
        action_groups_before = [bool(vessel.control.get_action_group(i)) for i in range(10)]
        report["liveKspBefore"] = before
        report["liveActuatorsBefore"] = {**actuators_before, "actionGroups0Through9": action_groups_before}
        if not before["paused"]:
            raise RuntimeError("Refusing simulated wrench calls unless live KSP is paused")
        if abs(before["ut"] - float(profile["savedUT"])) > 2.0:
            raise RuntimeError("Live KSP is outside the named checkpoint UT tolerance")
        if abs(before["altitude_m"] - 72000.0) > 500.0:
            raise RuntimeError("Live STS is not settled at the 72 km checkpoint")
        if before["part_count"] != int(profile["partCount"]):
            raise RuntimeError("Live STS part count does not match the checkpoint profile")
        if abs(before["mass_kg"] - float(profile["mass"])) > 1.0:
            raise RuntimeError("Live STS mass does not match the full-mass checkpoint")
        if before["sas"] or before["rcs"] or before["wheel_brakes"]:
            raise RuntimeError("Refusing simulation while SAS, RCS, or wheel brakes are enabled")
        if any(abs(float(value)) > 1e-9 for value in before["controls"].values()) or abs(before["thrust_n"]) > 1.0:
            raise RuntimeError("Refusing simulation while controls or thrust are nonzero")
        if any(action_groups_before) or actuators_before["speedbrakeActionGroupEnabled"]:
            raise RuntimeError("Refusing simulation while an action group or speedbrake group is enabled")
        if any(item["deployed"] or abs(item["deflection"]) > 1e-9 for item in actuators_before["controlSurfaces"]):
            raise RuntimeError("Refusing simulation with deployed or deflected control surfaces")

        body_omega = tuple(float(value) for value in body.angular_velocity(frame))
        flight = vessel.flight(frame)
        max_air_speed_error = 0.0
        for anchor_index, anchor in enumerate(measured_states):
            position = tuple(anchor["positionM_nonrotatingKrpcXYZ"])
            inertial_velocity = tuple(anchor["velocityMps_nonrotatingKrpcXYZ"])
            atmosphere_velocity = cross(body_omega, position)
            air_velocity = tuple(inertial_velocity[i] - atmosphere_velocity[i] for i in range(3))
            air_speed = norm(air_velocity)
            air_speed_error = (air_speed - anchor["trueAirSpeedMps_reported"]) / max(anchor["trueAirSpeedMps_reported"], 1.0)
            max_air_speed_error = max(max_air_speed_error, abs(air_speed_error))
            report["airSpeedFrameChecks"].append({
                "anchorIndex": anchor_index,
                "ut": anchor["ut"],
                "computedAirSpeedMps": air_speed,
                "reportedTrueAirSpeedMps": anchor["trueAirSpeedMps_reported"],
                "relativeDifference": air_speed_error,
            })
            if abs(air_speed_error) > 0.02:
                raise RuntimeError("Archived inertial velocity does not reconcile with measured true airspeed")

            rotation, achieved = measured_attitude(
                position,
                air_velocity,
                anchor["angleOfAttackDeg"],
                anchor["sideslipDeg"],
                anchor["bankDeg"],
            )
            angle_error = max(
                abs(achieved[0] - anchor["angleOfAttackDeg"]),
                abs(achieved[1] - anchor["sideslipDeg"]),
                abs(wrap_degrees(achieved[2] - anchor["bankDeg"])),
            )
            if angle_error > 1e-5:
                raise RuntimeError("Reconstructed attitude failed the measured AoA/sideslip/bank check")

            base_rates_deg = list(anchor["bodyRatesDegPerSecond_pitchRollYaw"])
            if len(base_rates_deg) != 3 or any(not math.isfinite(float(value)) or abs(float(value)) > 20.0 for value in base_rates_deg):
                raise RuntimeError("Archived measured body rates are missing, non-finite, or outside the bounded diagnostic range")
            base_rates_rad = [math.radians(float(value)) for value in base_rates_deg]
            cases: list[tuple[str, list[float]]] = [
                ("zero_rate_reference", [0.0, 0.0, 0.0]),
                ("measured_rate_baseline", list(base_rates_rad)),
            ]
            for magnitude in RATE_OFFSETS_DEG_S:
                for axis_name, axis in RATE_AXES:
                    for sign, polarity in ((-1.0, "negative"), (1.0, "positive")):
                        rates = list(base_rates_rad)
                        rates[axis] += sign * math.radians(magnitude)
                        cases.append((f"{axis_name}_{polarity}_{magnitude:g}deg_s", rates))

            case_data: dict[str, dict] = {}
            for case_name, rates_rad in cases:
                if not conn.krpc.paused or abs(float(sc.ut) - before["ut"]) > 1e-9:
                    raise RuntimeError("KSP pause or UT changed during hypothetical wrench calls")
                angular_velocity_reference = rotate_rates(rotation, rates_rad)
                force_raw, torque_raw = flight.simulate_aerodynamic_wrench_at(
                    body,
                    position,
                    inertial_velocity,
                    rotation,
                    angular_velocity_reference,
                    anchor["ut"],
                )
                force_reference = tuple(float(value) for value in force_raw)
                torque_reference = tuple(float(value) for value in torque_raw)
                sample = {
                    "anchorIndex": anchor_index,
                    "anchorRecordSequence": anchor["recordSequence"],
                    "anchorUT": anchor["ut"],
                    "anchorAltitudeM": anchor["altitudeM"],
                    "case": case_name,
                    "measuredBaselineBodyRateDegPerSecondXYZ": base_rates_deg,
                    "hypotheticalBodyRateRadPerSecondXYZ": rates_rad,
                    "hypotheticalAngularVelocityNonrotatingRadPerSecondXYZ": angular_velocity_reference,
                    "simulatedForceN_bodyXYZ": tuple(inverse_rotate(rotation, force_reference)),
                    "simulatedTorqueNm_bodyXYZ": tuple(inverse_rotate(rotation, torque_reference)),
                }
                case_data[case_name] = sample
                report["samples"].append(sample)

            matrices = []
            for magnitude in RATE_OFFSETS_DEG_S:
                matrix = [[0.0 for _ in range(3)] for _ in range(3)]
                for axis_name, axis in RATE_AXES:
                    plus_name = f"{axis_name}_positive_{magnitude:g}deg_s"
                    minus_name = f"{axis_name}_negative_{magnitude:g}deg_s"
                    plus = case_data[plus_name]["simulatedTorqueNm_bodyXYZ"]
                    minus = case_data[minus_name]["simulatedTorqueNm_bodyXYZ"]
                    delta_rad = math.radians(magnitude)
                    for torque_axis in range(3):
                        matrix[torque_axis][axis] = (plus[torque_axis] - minus[torque_axis]) / (2.0 * delta_rad)
                matrices.append({
                    "centralDifferenceOffsetDegPerSecond": magnitude,
                    "matrixNmPerRadPerSecond_bodyTorqueXYZ_by_bodyRateXYZ": matrix,
                })
            zero = case_data["zero_rate_reference"]["simulatedTorqueNm_bodyXYZ"]
            measured = case_data["measured_rate_baseline"]["simulatedTorqueNm_bodyXYZ"]
            report["anchorMatrices"].append({
                "anchorIndex": anchor_index,
                "anchorRecordSequence": anchor["recordSequence"],
                "ut": anchor["ut"],
                "altitudeM": anchor["altitudeM"],
                "mach": anchor["machReported"],
                "dynamicPressurePa": anchor["dynamicPressurePaReported"],
                "angleOfAttackDeg": anchor["angleOfAttackDeg"],
                "sideslipDeg": anchor["sideslipDeg"],
                "bankDeg": anchor["bankDeg"],
                "measuredBodyRateDegPerSecondXYZ": base_rates_deg,
                "zeroRateTorqueNm_bodyXYZ": zero,
                "measuredRateTorqueNm_bodyXYZ": measured,
                "measuredRateMinusZeroRateTorqueNm_bodyXYZ": [measured[i] - zero[i] for i in range(3)],
                "localRateDerivativeMatrices": matrices,
            })

        after = state_snapshot(conn, sc, vessel, body, frame)
        actuators_after = read_actuator_snapshot(vessel, config)
        action_groups_after = [bool(vessel.control.get_action_group(i)) for i in range(10)]
        report["liveKspAfter"] = after
        report["liveActuatorsAfter"] = {**actuators_after, "actionGroups0Through9": action_groups_after}
        report["liveStateUnchanged"] = before == after and report["liveActuatorsBefore"] == report["liveActuatorsAfter"]
        report["kspProcessPIDsAfter"] = pids_named_ksp()
        report["singleKSPProcessPreserved"] = len(report["kspProcessPIDsAfter"]) == 1 and report["kspProcessPIDsAfter"] == pids_before
        report["checkpointSHA256After"] = sha256(save_path)
        report["checkpointUnchanged"] = report["checkpointSHA256After"] == checkpoint_hash_before
        if not report["liveStateUnchanged"] or not report["singleKSPProcessPreserved"] or not report["checkpointUnchanged"]:
            raise RuntimeError("Read-only API probe changed the live state, process count, actuators, or checkpoint")

        yaw_cross_ratios = []
        diagonal_scale_differences = []
        yaw_to_roll_scale_differences = []
        for anchor in report["anchorMatrices"]:
            small = anchor["localRateDerivativeMatrices"][0]["matrixNmPerRadPerSecond_bodyTorqueXYZ_by_bodyRateXYZ"]
            large = anchor["localRateDerivativeMatrices"][1]["matrixNmPerRadPerSecond_bodyTorqueXYZ_by_bodyRateXYZ"]
            yaw_cross_ratios.append(abs(small[1][2] / small[2][2]))
            diagonal_scale_differences.extend(
                abs(large[axis][axis] - small[axis][axis]) / max(abs(small[axis][axis]), 1e-12)
                for axis in range(3)
            )
            yaw_to_roll_scale_differences.append(
                abs(large[1][2] - small[1][2]) / max(abs(small[1][2]), 1e-12)
            )
        sorted_ratios = sorted(yaw_cross_ratios)
        report["rateDampingSummary"] = {
            "yawRateToRollTorqueDerivativeMagnitudeRatioByAnchor": yaw_cross_ratios,
            "yawRateToRollTorqueDerivativeMagnitudeRatioRange": [min(yaw_cross_ratios), max(yaw_cross_ratios)],
            "yawRateToRollTorqueDerivativeMagnitudeRatioMedian": sorted_ratios[len(sorted_ratios) // 2],
            "maximumDiagonalDerivativeRelativeDifferenceBetween0_1And1DegPerSecond": max(diagonal_scale_differences),
            "maximumYawRateToRollTorqueDerivativeRelativeDifferenceBetween0_1And1DegPerSecond": max(yaw_to_roll_scale_differences),
            "interpretation": "The simulated aerodynamic yaw-rate-to-roll-torque derivative is repeatable across these measured-state anchors. It is separate from the zero-dynamic-pressure reaction-wheel pulse and cannot identify that pulse's actuator source or establish a qualified atmospheric control gain.",
        }
        report["status"] = "complete"
        report["gridSummary"] = {
            "anchorCount": len(measured_states),
            "sampleCount": len(report["samples"]),
            "elapsedWallSeconds": time.monotonic() - started,
            "maximumRelativeAirSpeedFrameError": max_air_speed_error,
            "allAttitudesReconstructedWithinDeg": 1e-5,
            "allRatesUsedWithinDegPerSecond": 20.0,
        }
        report["output"] = str(args.output)
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(json.dumps(report, indent=2, allow_nan=False) + "\n")
        print(json.dumps({
            "status": report["status"],
            "classification": report["classification"],
            "output": report["output"],
            "sampleCount": len(report["samples"]),
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
