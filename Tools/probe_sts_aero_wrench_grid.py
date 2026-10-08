#!/usr/bin/env python3
"""Read-only STS aerodynamic wrench grid; outputs unqualified diagnostics only."""

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


def dot(a, b):
    return sum(x * y for x, y in zip(a, b))


def norm(v):
    return math.sqrt(dot(v, v))


def cross(a, b):
    return (
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    )


def unit(v):
    length = norm(v)
    if not math.isfinite(length) or length <= 1e-12:
        raise ValueError("Cannot normalize a zero or non-finite vector")
    return tuple(value / length for value in v)


def rotate(q, v):
    """Rotate a vector by an (x,y,z,w) quaternion, local vessel -> frame."""
    x, y, z, w = q
    tx = 2.0 * (y * v[2] - z * v[1])
    ty = 2.0 * (z * v[0] - x * v[2])
    tz = 2.0 * (x * v[1] - y * v[0])
    return (
        v[0] + w * tx + y * tz - z * ty,
        v[1] + w * ty + z * tx - x * tz,
        v[2] + w * tz + x * ty - y * tx,
    )


def inverse_rotate(q, v):
    return rotate((-q[0], -q[1], -q[2], q[3]), v)


def quaternion_multiply(a, b):
    x, y, z, w = a
    X, Y, Z, W = b
    return (
        w * X + x * W + y * Z - z * Y,
        w * Y - x * Z + y * W + z * X,
        w * Z + x * Y - y * X + z * W,
        w * W - x * X - y * Y - z * Z,
    )


def wrap_degrees(value):
    return (value + 180.0) % 360.0 - 180.0


def bank_degrees(rotation, position):
    """KSP-sign bank from radial up to vessel +Z around vessel +Y (forward)."""
    forward = unit(rotate(rotation, (0.0, 1.0, 0.0)))
    body_up = rotate(rotation, (0.0, 0.0, 1.0))
    radial_up = unit(position)
    up_plane = unit(tuple(radial_up[i] - dot(radial_up, forward) * forward[i] for i in range(3)))
    body_up_plane = unit(tuple(body_up[i] - dot(body_up, forward) * forward[i] for i in range(3)))
    signed = math.degrees(math.atan2(dot(cross(up_plane, body_up_plane), forward), dot(up_plane, body_up_plane)))
    return wrap_degrees(-signed)


def angles_degrees(rotation, air_velocity):
    body_velocity = inverse_rotate(rotation, air_velocity)
    speed = norm(body_velocity)
    alpha = math.degrees(math.atan2(body_velocity[2], body_velocity[1]))
    beta = math.degrees(math.asin(max(-1.0, min(1.0, body_velocity[0] / speed))))
    return alpha, beta, body_velocity


def state_snapshot(conn, sc, vessel, body, frame):
    flight = vessel.flight(frame)
    return {
        "ut": float(sc.ut),
        "paused": bool(conn.krpc.paused),
        "vessel": str(vessel.name),
        "situation": str(vessel.situation),
        "altitude_m": float(flight.mean_altitude),
        "mach": float(flight.mach),
        "true_air_speed_mps": float(flight.true_air_speed),
        "speed_of_sound_mps": float(flight.speed_of_sound),
        "dynamic_pressure_pa": float(flight.dynamic_pressure),
        "mass_kg": float(vessel.mass),
        "part_count": len(vessel.parts.all),
        "position_m": list(vessel.position(frame)),
        "velocity_mps": list(vessel.velocity(frame)),
        "rotation_xyzw": list(vessel.rotation(frame)),
        "angular_velocity_rad_s": list(vessel.angular_velocity(frame)),
        "controls": {
            key: float(getattr(vessel.control, key))
            for key in ("pitch", "roll", "yaw", "throttle")
        },
        "sas": bool(vessel.control.sas),
        "rcs": bool(vessel.control.rcs),
        "wheel_brakes": bool(vessel.control.brakes),
        "thrust_n": float(vessel.thrust),
        "angle_of_attack_deg": float(flight.angle_of_attack),
        "sideslip_deg": float(flight.sideslip_angle),
        "roll_deg": float(flight.roll),
    }


def relative_error(predicted, measured):
    denominator = norm(measured)
    return norm(tuple(predicted[i] - measured[i] for i in range(3))) / denominator if denominator else None


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--output",
        type=Path,
        default=ROOT / "Docs/Experiments/STS-Entry-20261009/live-aerodynamic-wrench-grid.json",
    )
    args = parser.parse_args()

    config_path = ROOT / "Configuration/sts.json"
    checkpoint_profile_path = ROOT / "Configuration/checkpoints/sts-campaign-72km-20261008.physics.json"
    config = json.loads(config_path.read_text())
    checkpoint_profile = json.loads(checkpoint_profile_path.read_text())
    checkpoint_path = Path(checkpoint_profile["savePath"])
    checkpoint_hash_before = hashlib.sha256(checkpoint_path.read_bytes()).hexdigest()
    if checkpoint_hash_before != checkpoint_profile["saveSHA256"]:
        raise RuntimeError("Reference campaign checkpoint hash does not match its profile")

    report = {
        "status": "running",
        "classification": "UNQUALIFIED_DIAGNOSTIC_SIMULATION",
        "purpose": "Check whether the live KSP Flight aerodynamic wrench API can evaluate an original-STS Mach/AoA/sideslip/bank grid without advancing or changing the live vessel.",
        "qualification": {
            "qualified": False,
            "shadow_gate_evidence": False,
            "flight_control_or_actuation": False,
            "landing_or_rollout_evidence": False,
            "note": "Counterfactual Flight.simulate_aerodynamic_wrench_at results are simulations, not measured aerodynamic-force cells, a shadow-gate pass, or flight qualification.",
        },
        "provenance": {
            "repository": str(ROOT),
            "gitHead": subprocess.run(
                ["git", "rev-parse", "HEAD"], cwd=ROOT, capture_output=True, text=True, check=True
            ).stdout.strip(),
            "configuration": str(config_path.relative_to(ROOT)),
            "configurationSHA256": hashlib.sha256(config_path.read_bytes()).hexdigest(),
            "vehicleModelId": config["vehicle"]["modelId"],
            "checkpointProfile": str(checkpoint_profile_path.relative_to(ROOT)),
            "checkpointName": checkpoint_profile["saveName"],
            "checkpointSHA256": checkpoint_hash_before,
            "checkpointUsedForThisProbe": False,
            "inputStateOrigin": "Direct read-only snapshot of the already-loaded live STS at the time recorded below; this state is not the 72 km checkpoint.",
            "api": "SpaceCenter.Flight.simulate_aerodynamic_wrench_at",
            "referenceFrame": "CelestialBody.non_rotating_reference_frame",
            "utPolicy": "Pass the measured current UT explicitly to every wrench call.",
            "angularVelocityPolicy": "Zero angular velocity for the grid to isolate static aerodynamic wrench; same-state parity check also uses measured angular velocity.",
            "controlSurfacePolicy": "No control-surface, action-group, RCS, SAS, throttle, or vessel-state setters are called. The API uses the currently loaded vessel parts and control-surface state.",
        },
        "gridDefinition": {
            "basis": "One directly measured live position/UT/STS configuration; position and atmosphere are fixed while velocity and orientation are counterfactually varied.",
            "machValues": [2.0, 4.0, 6.0],
            "machIncludesMeasuredCurrentValue": True,
            "angleOfAttackDeg": [0.0, 5.0, 10.0, 18.0, 28.0],
            "sideslipDeg": [-5.0, 0.0, 5.0],
            "bankDeg": [-180.0, -90.0, 0.0, 90.0],
            "machVelocityConstruction": "Mach times the live measured local speed of sound; body rotation velocity is added so the API receives inertial velocity in the non-rotating frame.",
            "angleConvention": "Calibrated against live Flight.angle_of_attack, Flight.sideslip_angle, Flight.roll, and Vessel.direction at the captured state.",
            "angleOfAttack28Deg": "Diagnostic boundary sample matching the configured cap; no live command or limit change.",
            "pressureLimitPa": float(config["vehicle"].get("maximumDynamicPressure", math.inf)),
        },
        "samples": [],
        "errors": [],
    }

    conn = None
    started = time.monotonic()
    try:
        conn = connect("STS read-only aerodynamic wrench grid", timeout=8)
        sc = conn.space_center
        vessel = sc.active_vessel
        if vessel is None or vessel.name != "STS":
            raise RuntimeError("Read-only aero grid requires the original STS as the active vessel")
        body = vessel.orbit.body
        if body.name != "Kerbin":
            raise RuntimeError(f"Expected Kerbin, found {body.name}")
        frame = body.non_rotating_reference_frame
        flight = vessel.flight(frame)
        before = state_snapshot(conn, sc, vessel, body, frame)
        if not before["paused"]:
            raise RuntimeError("Refusing to run an offline grid while live KSP is unpaused")
        if before["sas"] or any(abs(value) > 1e-9 for key, value in before["controls"].items()):
            raise RuntimeError("Refusing to run unless SAS and all direct controls are off")
        if abs(before["thrust_n"]) > 1.0:
            raise RuntimeError("Refusing to run with non-zero actual thrust")

        position = tuple(before["position_m"])
        rotation = tuple(before["rotation_xyzw"])
        inertial_velocity = tuple(before["velocity_mps"])
        measured_angular_velocity = tuple(before["angular_velocity_rad_s"])
        ut = before["ut"]
        omega_body = tuple(body.angular_velocity(frame))
        atmosphere_velocity = cross(omega_body, position)
        air_velocity = tuple(inertial_velocity[i] - atmosphere_velocity[i] for i in range(3))
        alpha_actual, beta_actual, body_air_velocity = angles_degrees(rotation, air_velocity)
        bank_actual = bank_degrees(rotation, position)
        if abs(alpha_actual - before["angle_of_attack_deg"]) > 0.1:
            raise RuntimeError("Quaternion-derived AoA did not match live Flight.angle_of_attack")
        if abs(beta_actual - before["sideslip_deg"]) > 0.1:
            raise RuntimeError("Quaternion-derived sideslip did not match live Flight.sideslip_angle")
        if abs(wrap_degrees(bank_actual - before["roll_deg"])) > 0.2:
            raise RuntimeError("Quaternion-derived bank did not match live Flight.roll")

        body_frame = body.reference_frame
        surface_velocity = tuple(vessel.velocity(body_frame))
        surface_velocity_inertial = tuple(sc.transform_direction(surface_velocity, body_frame, frame))
        frame_air_velocity_error = norm(tuple(
            air_velocity[i] - surface_velocity_inertial[i] for i in range(3)
        ))
        if frame_air_velocity_error > 0.1:
            raise RuntimeError("Atmosphere-rotation reconstruction did not match body-fixed velocity")

        predicted_force = tuple(flight.simulate_aerodynamic_force_at(body, position, inertial_velocity, rotation))
        predicted_torque = tuple(
            flight.simulate_aerodynamic_torque_at(
                body, position, inertial_velocity, rotation, measured_angular_velocity
            )
        )
        predicted_wrench = flight.simulate_aerodynamic_wrench_at(
            body, position, inertial_velocity, rotation, measured_angular_velocity, ut
        )
        live_force = tuple(flight.aerodynamic_force)
        live_torque = tuple(flight.aerodynamic_torque)
        report["sameStateParity"] = {
            "measuredForceN": live_force,
            "simulatedForceN": predicted_force,
            "wrenchForceN": tuple(predicted_wrench[0]),
            "measuredTorqueNm": live_torque,
            "simulatedTorqueNm": predicted_torque,
            "wrenchTorqueNm": tuple(predicted_wrench[1]),
            "forceRelativeError": relative_error(predicted_force, live_force),
            "torqueRelativeError": relative_error(predicted_torque, live_torque),
            "measuredMach": before["mach"],
            "derivedAoADeg": alpha_actual,
            "derivedSideslipDeg": beta_actual,
            "derivedBankDeg": bank_actual,
            "airVelocityFrameReconstructionErrorMps": frame_air_velocity_error,
        }

        current_mach = before["mach"]
        sound_speed = before["speed_of_sound_mps"]
        max_q = float(config["vehicle"].get("maximumDynamicPressure", math.inf))
        target_machs = sorted({2.0, 4.0, 6.0, round(current_mach, 6)})
        report["gridDefinition"]["machValues"] = target_machs
        target_aoas = (0.0, 5.0, 10.0, 18.0, 28.0)
        target_betas = (-5.0, 0.0, 5.0)
        target_banks = (-180.0, -90.0, 0.0, 90.0)
        maximum_bank_error = 0.0
        for target_mach in target_machs:
            speed = target_mach * sound_speed
            q_estimate = before["dynamic_pressure_pa"] * (speed / before["true_air_speed_mps"]) ** 2
            if q_estimate > max_q:
                report["errors"].append({
                    "skippedMach": target_mach,
                    "reason": "counterfactual dynamic pressure exceeds the configured diagnostic bound",
                    "estimatedDynamicPressurePa": q_estimate,
                })
                continue
            for target_aoa in target_aoas:
                alpha = math.radians(target_aoa)
                for target_beta in target_betas:
                    beta = math.radians(target_beta)
                    body_velocity = (
                        speed * math.sin(beta),
                        speed * math.cos(beta) * math.cos(alpha),
                        speed * math.cos(beta) * math.sin(alpha),
                    )
                    for target_bank in target_banks:
                        if not conn.krpc.paused or abs(float(sc.ut) - ut) > 1e-9:
                            raise RuntimeError("Live KSP pause/UT changed during the counterfactual grid")
                        delta = math.radians(wrap_degrees(bank_actual - target_bank))
                        roll_delta = (0.0, math.sin(delta / 2.0), 0.0, math.cos(delta / 2.0))
                        target_rotation = quaternion_multiply(rotation, roll_delta)
                        achieved_bank = bank_degrees(target_rotation, position)
                        bank_error = abs(wrap_degrees(achieved_bank - target_bank))
                        maximum_bank_error = max(maximum_bank_error, bank_error)
                        if bank_error > 0.2:
                            raise RuntimeError("Counterfactual bank quaternion did not reach its requested bank")
                        target_air_velocity = rotate(target_rotation, body_velocity)
                        target_velocity = tuple(target_air_velocity[i] + atmosphere_velocity[i] for i in range(3))
                        achieved_aoa, achieved_beta, _ = angles_degrees(target_rotation, target_air_velocity)
                        wrench = flight.simulate_aerodynamic_wrench_at(
                            body, position, target_velocity, target_rotation, (0.0, 0.0, 0.0), ut
                        )
                        force = tuple(wrench[0])
                        torque = tuple(wrench[1])
                        report["samples"].append({
                            "mach": target_mach,
                            "angleOfAttackDeg": target_aoa,
                            "sideslipDeg": target_beta,
                            "bankDeg": target_bank,
                            "achieved": {
                                "mach": norm(target_air_velocity) / sound_speed,
                                "angleOfAttackDeg": achieved_aoa,
                                "sideslipDeg": achieved_beta,
                                "bankDeg": achieved_bank,
                            },
                            "estimatedDynamicPressurePa": q_estimate,
                            "forceN_nonrotating": force,
                            "forceMagnitudeN": norm(force),
                            "torqueNm_nonrotating": torque,
                            "torqueMagnitudeNm": norm(torque),
                        })

        report["gridSummary"] = {
            "sampleCount": len(report["samples"]),
            "maximumAchievedBankErrorDeg": maximum_bank_error,
            "elapsedWallSeconds": time.monotonic() - started,
        }
        report["status"] = "complete"
    except BaseException as exc:
        report["status"] = "failed"
        report["error"] = str(exc)
        raise
    finally:
        if conn is not None:
            if "before" in locals():
                try:
                    after = state_snapshot(conn, sc, vessel, body, frame)
                    checkpoint_hash_after = hashlib.sha256(checkpoint_path.read_bytes()).hexdigest()
                    report["liveStateBefore"] = before
                    report["liveStateAfter"] = after
                    report["liveStateUnchanged"] = before == after
                    report["checkpointUnchanged"] = checkpoint_hash_before == checkpoint_hash_after
                    report["checkpointSHA256After"] = checkpoint_hash_after
                    if not report["liveStateUnchanged"] or not report["checkpointUnchanged"]:
                        report["status"] = "failed"
                        report["invariantFailure"] = "Live state or checkpoint changed during the read-only probe."
                except Exception as post_error:
                    report["postStateCaptureError"] = str(post_error)
            try:
                conn.close()
            except Exception:
                pass
        args.output.parent.mkdir(parents=True, exist_ok=True)
        report["sourceSHA256"] = hashlib.sha256(Path(__file__).read_bytes()).hexdigest()
        args.output.write_text(json.dumps(report, indent=2, allow_nan=False) + "\n")

    print(json.dumps({
        "status": report["status"],
        "output": str(args.output),
        "samples": len(report["samples"]),
        "liveStateUnchanged": report.get("liveStateUnchanged"),
        "checkpointUnchanged": report.get("checkpointUnchanged"),
    }, separators=(",", ":")))


if __name__ == "__main__":
    main()
