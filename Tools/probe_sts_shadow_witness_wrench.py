#!/usr/bin/env python3
"""Read-only KSP wrench grid at the latest STS shadow witness state."""

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


TRACE_PATH = ROOT / "Runtime/STS-Campaign-20261009/entry-trace/headless-live.log"
OUTPUT_PATH = ROOT / "Docs/Experiments/STS-Entry-20261009/shadow-witness-wrench.json"


def pids_named_ksp() -> list[int]:
    result = subprocess.run(["pgrep", "-x", "KSP"], capture_output=True, text=True)
    if result.returncode not in (0, 1):
        raise RuntimeError(f"pgrep failed while checking the single-KSP invariant: {result.stderr.strip()}")
    return [int(value) for value in result.stdout.split()]


def key_values(line: str) -> dict[str, str]:
    result: dict[str, str] = {}
    for token in line.split():
        if "=" in token:
            key, value = token.split("=", 1)
            result[key] = value.strip('"')
    return result


def read_shadow_witness() -> tuple[dict[str, float], dict[str, float], str]:
    lines = TRACE_PATH.read_text().splitlines()
    trace_rows = []
    witness_rows = []
    for line in lines:
        if line.startswith("ENTRY_TRACE ut="):
            row = key_values(line)
            if all(key in row for key in ("ut", "h", "V", "q", "D", "Dref", "along", "xtk", "Bcmd", "roll", "aoa")):
                trace_rows.append({key: float(row[key]) for key in ("ut", "h", "V", "q", "D", "Dref", "along", "xtk", "Bcmd", "roll", "aoa")})
        elif line.startswith("ENTRY_WITNESS state ut="):
            row = key_values(line)
            if all(key in row for key in ("ut", "h", "V", "M")):
                witness_rows.append({key: float(row[key]) for key in ("ut", "h", "V", "M")})
    if not trace_rows or not witness_rows:
        raise RuntimeError("The retained shadow trace lacks a usable ENTRY_TRACE or ENTRY_WITNESS state")
    trace = min(trace_rows, key=lambda row: abs(row["h"] - 38000.0))
    witness = min(witness_rows, key=lambda row: abs(row["ut"] - trace["ut"]))
    if abs(trace["h"] - 38000.0) > 2500 or abs(trace["aoa"] - 28.0) > 0.2:
        raise RuntimeError("The nearest retained witness is outside the intended high-AoA transition sample")
    return trace, witness, hashlib.sha256(TRACE_PATH.read_bytes()).hexdigest()


def quaternion_from_basis(x_axis, y_axis, z_axis):
    """Build an (x,y,z,w) quaternion from world-space basis columns."""
    m00, m10, m20 = x_axis
    m01, m11, m21 = y_axis
    m02, m12, m22 = z_axis
    trace = m00 + m11 + m22
    if trace > 0.0:
        scale = math.sqrt(trace + 1.0) * 2.0
        q = ((m21 - m12) / scale, (m02 - m20) / scale, (m10 - m01) / scale, 0.25 * scale)
    elif m00 > m11 and m00 > m22:
        scale = math.sqrt(1.0 + m00 - m11 - m22) * 2.0
        q = (0.25 * scale, (m01 + m10) / scale, (m02 + m20) / scale, (m21 - m12) / scale)
    elif m11 > m22:
        scale = math.sqrt(1.0 + m11 - m00 - m22) * 2.0
        q = ((m01 + m10) / scale, 0.25 * scale, (m12 + m21) / scale, (m02 - m20) / scale)
    else:
        scale = math.sqrt(1.0 + m22 - m00 - m11) * 2.0
        q = ((m02 + m20) / scale, (m12 + m21) / scale, 0.25 * scale, (m10 - m01) / scale)
    length = norm(q)
    return tuple(value / length for value in q)


def rotate_by_body_spin(value, omega, elapsed_s):
    angle = norm(omega) * elapsed_s
    if angle == 0.0:
        return tuple(value), (0.0, 0.0, 0.0, 1.0)
    axis = unit(omega)
    half = angle / 2.0
    q_spin = (axis[0] * math.sin(half), axis[1] * math.sin(half), axis[2] * math.sin(half), math.cos(half))
    return rotate(q_spin, value), q_spin


def read_actuator_snapshot(vessel, config) -> dict:
    controls = vessel.control
    action_group = int(config["vehicle"].get("airbrakeActionGroup", 1))
    surfaces = []
    for surface in vessel.parts.control_surfaces:
        surfaces.append({
            "part": str(surface.part.name),
            "deployed": bool(surface.deployed),
            "deflection": float(surface.deflection),
            "authorityLimiter": float(surface.authority_limiter),
        })
    return {
        "speedbrakeActionGroup": action_group,
        "speedbrakeActionGroupEnabled": bool(controls.get_action_group(action_group)),
        "controlSurfaces": surfaces,
    }


def sts_flight_roll(rotation, position, geometric_offset_deg: float) -> float:
    """Map geometric local-+Z bank to this STS checkpoint's KSP Flight.roll."""
    return wrap_degrees(bank_degrees(rotation, position) + geometric_offset_deg)


def main() -> int:
    config_path = ROOT / "Configuration/sts.json"
    profile_path = ROOT / "Configuration/checkpoints/sts-campaign-72km-20261008.physics.json"
    config = json.loads(config_path.read_text())
    profile = json.loads(profile_path.read_text())
    checkpoint_path = Path(profile["savePath"])
    checkpoint_hash_before = hashlib.sha256(checkpoint_path.read_bytes()).hexdigest()
    if checkpoint_hash_before != profile["saveSHA256"]:
        raise RuntimeError("The original campaign checkpoint hash does not match its physics profile")

    trace, nearest_witness, trace_hash = read_shadow_witness()
    report = {
        "status": "running",
        "classification": "UNQUALIFIED_DIAGNOSTIC_SIMULATION",
        "purpose": "Compare the current original-STS KSP aerodynamic model at a hypothetical KSC 38 km witness state with the retained entry-shadow drag estimate, and sample nearby Mach/AoA/sideslip/bank conditions.",
        "qualification": {
            "qualified": False,
            "shadowGateEvidence": False,
            "measuredAerodynamicEvidence": False,
            "flightControlOrActuation": False,
            "landingOrRolloutEvidence": False,
            "note": "Every returned wrench is a counterfactual KSP simulation. It is not measured flight force, a shadow-gate pass, or landing qualification.",
        },
        "provenance": {
            "repository": str(ROOT),
            "gitHead": subprocess.run(["git", "rev-parse", "HEAD"], cwd=ROOT, capture_output=True, text=True, check=True).stdout.strip(),
            "configuration": str(config_path.relative_to(ROOT)),
            "configurationSHA256": hashlib.sha256(config_path.read_bytes()).hexdigest(),
            "vehicleModelId": config["vehicle"]["modelId"],
            "checkpointProfile": str(profile_path.relative_to(ROOT)),
            "checkpointName": profile["saveName"],
            "checkpointSHA256Before": checkpoint_hash_before,
            "checkpointUsedAsLoadedVesselSource": True,
            "trace": str(TRACE_PATH.relative_to(ROOT)),
            "traceSHA256": trace_hash,
            "inputTraceRecord": trace,
            "nearestShadowWitnessRecord": nearest_witness,
            "witnessSoundSpeedApproxMps": nearest_witness["V"] / nearest_witness["M"],
            "witnessSoundSpeedTimeOffsetS": nearest_witness["ut"] - trace["ut"],
            "api": "SpaceCenter.Flight.simulate_aerodynamic_wrench_at",
            "referenceFrame": "CelestialBody.non_rotating_reference_frame",
            "apiUt": "The shadow trace UT is passed explicitly. The KSC site position and local orientation are rotated from current paused UT using the live body's measured angular velocity so they refer to the same inertial epoch.",
            "positionOrigin": "Configuration site latitude/longitude (KSC Runway 09 center) at the shadow trace altitude; the shadow point is about 4.55 km from the runway center, so this is a nearby uniform-atmosphere location proxy.",
            "velocityOrigin": "Requested local-body airflow for the selected AoA and sideslip plus the planet's local rotational velocity at the target inertial position.",
            "angularVelocityPolicy": "Zero hypothetical vessel angular velocity to isolate static aerodynamic force and moment; no damping moment is included.",
            "surfaceStatePolicy": "Use the currently loaded original-STS part and control-surface state. No control, action group, throttle, RCS, SAS, save, position, velocity, orientation, or time setters are called.",
        },
        "gridDefinition": {
            "nominalMachValues": [],
            "angleOfAttackDeg": [18.0, 28.0],
            "sideslipDeg": [-5.0, 0.0, 5.0],
            "bankDeg": [-40.0, 0.0, 40.0],
            "bankConvention": "KSP Flight.roll for the original STS, calibrated from its loaded 72 km checkpoint attitude; the checkpoint's local +Z axis is opposite radial up.",
            "nominalMachPolicy": "Mach slices use the closest recorded shadow-witness speed of sound (V/M, 1.50 s from the trace point). The trace-speed slice is exact input TAS; the API accepts velocity rather than Mach, so nominal Mach is provenance metadata, not a separate API output.",
            "dynamicPressureScaling": "The trace q is scaled by TAS squared at fixed density only for comparison metadata; KSP computes each wrench internally from the hypothetical position, velocity and explicit UT.",
            "pressureBoundPa": float(config["vehicle"].get("maximumDynamicPressure", math.inf)),
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
        conn = connect("STS read-only shadow-witness aerodynamic wrench", timeout=8)
        sc = conn.space_center
        vessel = sc.active_vessel
        if vessel is None or vessel.name != "STS" or config["vehicle"]["modelId"] != "STS":
            raise RuntimeError("The active vessel and configured vehicle must both be the original STS")
        body = vessel.orbit.body
        if body.name != "Kerbin":
            raise RuntimeError(f"Expected Kerbin, found {body.name}")
        frame = body.non_rotating_reference_frame
        before = state_snapshot(conn, sc, vessel, body, frame)
        actuators_before = read_actuator_snapshot(vessel, config)
        current_ut = before["ut"]
        checkpoint_geometric_bank = bank_degrees(
            tuple(before["rotation_xyzw"]), tuple(before["position_m"])
        )
        bank_offset = wrap_degrees(before["roll_deg"] - checkpoint_geometric_bank)
        if min(abs(bank_offset), abs(abs(bank_offset) - 180.0)) > 0.2:
            raise RuntimeError("The checkpoint STS bank convention is not aligned with local +Z or -Z")
        checkpoint_saved_ut = float(profile["savedUT"])
        if not before["paused"]:
            raise RuntimeError("Refusing to probe while KSP is unpaused")
        if abs(current_ut - checkpoint_saved_ut) > 1.0 or abs(before["altitude_m"] - 72000.0) > 500.0:
            raise RuntimeError("The live STS is not at the saved 72 km checkpoint state")
        if before["sas"]:
            raise RuntimeError("Refusing to probe with SAS enabled")
        if any(abs(value) > 1e-9 for value in before["controls"].values()):
            raise RuntimeError("Refusing to probe with a direct control or throttle input")
        if abs(before["thrust_n"]) > 1.0:
            raise RuntimeError("Refusing to probe with actual engine thrust")
        if actuators_before["speedbrakeActionGroupEnabled"]:
            raise RuntimeError("Refusing to probe with the configured speedbrake action group enabled")
        if any(surface["deployed"] or abs(surface["deflection"]) > 1e-9 for surface in actuators_before["controlSurfaces"]):
            raise RuntimeError("Refusing to probe with a deployed or deflected control surface")
        if len(vessel.parts.all) != int(profile["partCount"]):
            raise RuntimeError("Loaded original-STS part count does not match the checkpoint profile")

        site = config["site"]
        latitude = float(site["latitude"])
        longitude = float(site["longitude"])
        altitude = trace["h"]
        target_ut = trace["ut"]
        elapsed = target_ut - current_ut
        omega = tuple(float(x) for x in body.angular_velocity(frame))
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
        vessel_z_sign = -1.0 if abs(bank_offset) > 90.0 else 1.0
        vessel_z = unit(tuple(vessel_z_sign * (radial_up[i] - dot(radial_up, forward) * forward[i]) for i in range(3)))
        right = unit(cross(forward, vessel_z))
        local_rotation = quaternion_from_basis(right, forward, vessel_z)
        future_position_result = rotate_by_body_spin(base_position, omega, elapsed)
        target_position, spin_rotation = future_position_result
        base_rotation = quaternion_multiply(spin_rotation, local_rotation)
        base_geometric_bank = bank_degrees(base_rotation, target_position)
        base_bank = sts_flight_roll(base_rotation, target_position, bank_offset)
        if abs(wrap_degrees(base_bank)) > 0.2:
            raise RuntimeError(f"Constructed runway-aligned orientation has bank {base_bank:.3f} degrees")
        if not conn.krpc.paused or abs(float(sc.ut) - current_ut) > 1e-9:
            raise RuntimeError("KSP pause or UT changed during hypothetical position construction")

        witness_sound_speed = nearest_witness["V"] / nearest_witness["M"]
        trace_speed = trace["V"]
        speed_plan = [
            {"nominalMach": 4.0, "speedMps": 4.0 * witness_sound_speed, "source": "adjacent-witness sound-speed scaling"},
            {"nominalMach": nearest_witness["M"], "speedMps": nearest_witness["V"], "source": "nearest recorded witness TAS"},
            {"nominalMach": trace_speed / witness_sound_speed, "speedMps": trace_speed, "source": "exact selected ENTRY_TRACE TAS"},
            {"nominalMach": 6.0, "speedMps": 6.0 * witness_sound_speed, "source": "adjacent-witness sound-speed scaling"},
        ]
        report["gridDefinition"]["nominalMachValues"] = [item["nominalMach"] for item in speed_plan]
        maximum_angle_error = 0.0
        maximum_bank_error = 0.0
        for speed_item in speed_plan:
            speed = speed_item["speedMps"]
            q_scaled = trace["q"] * (speed / trace_speed) ** 2
            if q_scaled > report["gridDefinition"]["pressureBoundPa"]:
                raise RuntimeError(f"Diagnostic q estimate {q_scaled:.1f} Pa exceeds the configured pressure bound")
            for aoa in (18.0, 28.0):
                alpha = math.radians(aoa)
                for sideslip in (-5.0, 0.0, 5.0):
                    beta = math.radians(sideslip)
                    for bank in (-40.0, 0.0, 40.0):
                        delta = math.radians(wrap_degrees(base_bank - bank))
                        roll_delta = (0.0, math.sin(delta / 2.0), 0.0, math.cos(delta / 2.0))
                        rotation = quaternion_multiply(base_rotation, roll_delta)
                        achieved_bank = sts_flight_roll(rotation, target_position, bank_offset)
                        bank_error = abs(wrap_degrees(achieved_bank - bank))
                        maximum_bank_error = max(maximum_bank_error, bank_error)
                        if bank_error > 0.2:
                            raise RuntimeError("Counterfactual orientation did not reach the requested runway-relative bank")
                        body_air_velocity = (
                            speed * math.sin(beta),
                            speed * math.cos(beta) * math.cos(alpha),
                            speed * math.cos(beta) * math.sin(alpha),
                        )
                        air_velocity = tuple(rotate(rotation, body_air_velocity))
                        atmosphere_velocity = cross(omega, target_position)
                        inertial_velocity = tuple(air_velocity[i] + atmosphere_velocity[i] for i in range(3))
                        achieved_aoa, achieved_sideslip, _ = angles_degrees(rotation, air_velocity)
                        angle_error = max(abs(achieved_aoa - aoa), abs(achieved_sideslip - sideslip))
                        maximum_angle_error = max(maximum_angle_error, angle_error)
                        if angle_error > 0.05:
                            raise RuntimeError("Counterfactual velocity did not reach the requested AoA and sideslip")
                        if not conn.krpc.paused or abs(float(sc.ut) - current_ut) > 1e-9:
                            raise RuntimeError("KSP pause or UT changed during counterfactual wrench calls")
                        force_raw, torque_raw = vessel.flight(frame).simulate_aerodynamic_wrench_at(
                            body,
                            target_position,
                            inertial_velocity,
                            rotation,
                            (0.0, 0.0, 0.0),
                            target_ut,
                        )
                        force = tuple(float(x) for x in force_raw)
                        torque = tuple(float(x) for x in torque_raw)
                        air_direction = unit(air_velocity)
                        drag_n = -dot(force, air_direction)
                        report["samples"].append({
                            "nominalMachFromAdjacentWitness": speed_item["nominalMach"],
                            "speedSource": speed_item["source"],
                            "trueAirSpeedMps": speed,
                            "angleOfAttackDeg": aoa,
                            "sideslipDeg": sideslip,
                            "bankDeg": bank,
                            "achieved": {
                                "angleOfAttackDeg": achieved_aoa,
                                "sideslipDeg": achieved_sideslip,
                                "bankDeg": achieved_bank,
                            },
                            "dynamicPressureEstimatePa": q_scaled,
                            "forceN_nonrotating": force,
                            "forceN_bodyXYZ": tuple(inverse_rotate(rotation, force)),
                            "dragN_alongRelativeAirflow": drag_n,
                            "dragAccelerationMps2_usingCheckpointMass": drag_n / float(profile["mass"]),
                            "effectiveDragAreaM2_usingTraceQ": drag_n / q_scaled if q_scaled > 0.0 else None,
                            "torqueNm_nonrotating": torque,
                            "torqueNm_bodyXYZ": tuple(inverse_rotate(rotation, torque)),
                        })

        matched = min(
            report["samples"],
            key=lambda sample: abs(sample["trueAirSpeedMps"] - trace_speed)
            + 100.0 * abs(sample["angleOfAttackDeg"] - trace["aoa"])
            + 100.0 * abs(sample["sideslipDeg"])
            + 100.0 * abs(sample["bankDeg"] - trace["roll"]),
        )
        simulated_drag_accel = matched["dragAccelerationMps2_usingCheckpointMass"]
        report["matchedWitnessComparison"] = {
            "sampleIndex": report["samples"].index(matched),
            "simulatedDragAccelerationMps2": simulated_drag_accel,
            "shadowModelDragAccelerationMps2": trace["D"],
            "shadowDRAGREFAccelerationMps2": trace["Dref"],
            "simulationMinusShadowModelMps2": simulated_drag_accel - trace["D"],
            "simulationToShadowModelRatio": simulated_drag_accel / trace["D"] if trace["D"] else None,
            "simulationToDRAGREFRatio": simulated_drag_accel / trace["Dref"] if trace["Dref"] else None,
            "note": "The comparison uses current original-STS parts, a simulated KSC runway-center position, zero angular velocity, checkpoint-profile mass, and the shadow trace's q-scaled density proxy. It is a model discrepancy diagnostic, not measured flight force or qualification.",
        }
        report["gridSummary"] = {
            "sampleCount": len(report["samples"]),
            "speedSliceCount": len(speed_plan),
            "maximumAchievedAngleErrorDeg": maximum_angle_error,
            "maximumAchievedBankErrorDeg": maximum_bank_error,
            "elapsedWallSeconds": time.monotonic() - started,
        }
        report["liveStateBefore"] = before
        report["liveActuatorStateBefore"] = actuators_before
        report["kspProcessPIDsBefore"] = pids_before
        future_latitude_at_current_frame = float(body.latitude_at_position(target_position, frame))
        future_longitude_at_current_frame = float(body.longitude_at_position(target_position, frame))
        report["loadedMassAssessment"] = {
            "pausedLoadedMassKg": before["mass_kg"],
            "checkpointProfileMassKg": float(profile["mass"]),
            "pausedLoadedDryMassKg": float(vessel.dry_mass),
            "checkpointProfileDryMassKg": float(profile["dryMass"]),
            "massStateMatchesSettledProfile": abs(before["mass_kg"] - float(profile["mass"])) < 1.0,
            "massUsedForDragAcceleration": "checkpoint profile mass; the paused loaded vessel reports a placeholder mass, so no angular-acceleration inference is made from the simulated torque.",
            "torqueLimitation": "The wrench torque is returned about the current loaded vessel center of mass. Because the paused loaded mass differs from the settled checkpoint profile, torque is retained as descriptive output only and is not used to infer angular acceleration or calibrate control coupling.",
        }
        report["positionGeometry"] = {
            "siteLatitudeDeg": latitude,
            "siteLongitudeDeg": longitude,
            "runwayHeadingDeg": float(site["runwayHeading"]),
            "targetAltitudeM": altitude,
            "targetUT": target_ut,
            "currentPausedUT": current_ut,
            "bodyAngularVelocityRadPerS_nonrotating": omega,
            "spinExtrapolationSeconds": elapsed,
            "inertialSpinAngleRad": norm(omega) * elapsed,
            "positionNonrotatingM": target_position,
            "futureSiteLatitudeSeenInCurrentFrameDeg": future_latitude_at_current_frame,
            "futureSiteLongitudeSeenInCurrentFrameDeg": future_longitude_at_current_frame,
            "siteLongitudeAtCurrentUTDeg": float(body.longitude_at_position(base_position, frame)),
            "checkpointFlightRollDeg": before["roll_deg"],
            "checkpointGeometricLocalZBankDeg": checkpoint_geometric_bank,
            "stsFlightRollGeometricOffsetDeg": bank_offset,
            "vesselLocalZSignRelativeToRadialUp": vessel_z_sign,
            "baseGeometricLocalZBankDeg": base_geometric_bank,
            "baseBankBeforeCounterfactualRollDeg": base_bank,
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
                    report["invariantFailure"] = "Live state, control surfaces, checkpoint hash, or single-KSP process invariant changed."
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
        "matchedDragAccelerationMps2": report.get("matchedWitnessComparison", {}).get("simulatedDragAccelerationMps2"),
        "liveStateUnchanged": report.get("liveStateUnchanged"),
        "checkpointUnchanged": report.get("checkpointUnchanged"),
        "singleKspProcessPreserved": report.get("singleKspProcessPreserved"),
    }, separators=(",", ":")))
    return 0 if report["status"] == "complete" else 1


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except Exception as exc:
        print(json.dumps({"status": "failed", "error": f"{type(exc).__name__}: {exc}"}, separators=(",", ":")), file=sys.stderr)
        raise SystemExit(1)
