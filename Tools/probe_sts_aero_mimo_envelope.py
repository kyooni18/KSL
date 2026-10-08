#!/usr/bin/env python3
"""Read-only original-STS aerodynamic rate-coupling envelope probe.

The grid uses hypothetical states supplied to kRPC's aerodynamic wrench API.
It does not advance KSP or alter the loaded vessel, controls, or checkpoint.
All outputs are unqualified simulations, not flight measurements.
"""

from __future__ import annotations

import hashlib
import json
import math
from pathlib import Path
import subprocess
import sys
import tempfile
import time

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT / "Tools") not in sys.path:
    sys.path.insert(0, str(ROOT / "Tools"))

from ksp_test_guard import connect
from probe_sts_aero_envelope_grid import read_shadow_witness, runway_offset_lat_lon
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
    rotate_by_body_spin,
    sts_flight_roll,
)


OUTPUT = ROOT / "Docs/Experiments/STS-Entry-20261009/live-aero-mimo-envelope-grid.json"
ALTITUDES_M = (48000.0, 38000.0, 36000.0)
MACH_PLAN = (4.0, 5.0, "trace", 6.0)
AOA_DEG = (18.0, 28.0)
BETA_DEG = (-5.0, 0.0, 5.0)
BANK_DEG = (-40.0, 0.0, 40.0)
RATE_DEG_PER_S = 1.0
RATE_AXES = (("bodyX", 0), ("bodyY_roll", 1), ("bodyZ_yaw", 2))


def atomic_report(path: Path, report: dict) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with tempfile.NamedTemporaryFile("w", encoding="utf-8", dir=path.parent,
                                     prefix=path.name + ".", suffix=".tmp",
                                     delete=False) as handle:
        json.dump(report, handle, indent=2)
        handle.write("\n")
        temporary = Path(handle.name)
    temporary.replace(path)


def target_state(body, frame, site, trace, current_ut, omega, vessel_z_sign,
                 bank_offset, speed, aoa_deg, beta_deg, bank_deg):
    latitude, longitude = runway_offset_lat_lon(
        float(body.equatorial_radius), float(site["latitude"]),
        float(site["longitude"]), float(site["runwayHeading"]),
        float(trace["along"]), float(trace["xtk"]),
    )
    altitude = float(trace["h"])
    position_base = tuple(body.position_at_altitude(latitude, longitude, altitude, frame))
    surface = tuple(body.position_at_altitude(latitude, longitude, 0.0, frame))
    north_sample = tuple(body.position_at_altitude(latitude + 0.01, longitude, 0.0, frame))
    east_sample = tuple(body.position_at_altitude(latitude, longitude + 0.01, 0.0, frame))
    radial_up = unit(position_base)
    north_raw = tuple(north_sample[i] - surface[i] for i in range(3))
    east_raw = tuple(east_sample[i] - surface[i] for i in range(3))
    north = unit(tuple(north_raw[i] - dot(north_raw, radial_up) * radial_up[i] for i in range(3)))
    east = unit(tuple(east_raw[i] - dot(east_raw, radial_up) * radial_up[i] for i in range(3)))
    heading = math.radians(float(site["runwayHeading"]))
    forward = unit(tuple(north[i] * math.cos(heading) + east[i] * math.sin(heading) for i in range(3)))
    vessel_z = unit(tuple(vessel_z_sign * (radial_up[i] - dot(radial_up, forward) * forward[i]) for i in range(3)))
    right = unit(cross(forward, vessel_z))
    local_rotation = quaternion_from_basis(right, forward, vessel_z)
    target_ut = float(trace["ut"])
    target_position, spin_rotation = rotate_by_body_spin(
        position_base, omega, target_ut - current_ut
    )
    base_rotation = quaternion_multiply(spin_rotation, local_rotation)
    base_bank = sts_flight_roll(base_rotation, target_position, bank_offset)
    bank_delta = math.radians(wrap_degrees(base_bank - bank_deg))
    roll_rotation = (0.0, math.sin(bank_delta / 2.0), 0.0, math.cos(bank_delta / 2.0))
    rotation = quaternion_multiply(base_rotation, roll_rotation)

    alpha = math.radians(aoa_deg)
    beta = math.radians(beta_deg)
    body_air_velocity = (
        speed * math.sin(beta),
        speed * math.cos(beta) * math.cos(alpha),
        speed * math.cos(beta) * math.sin(alpha),
    )
    air_velocity = tuple(rotate(rotation, body_air_velocity))
    spin_velocity = cross(omega, target_position)
    inertial_velocity = tuple(air_velocity[i] + spin_velocity[i] for i in range(3))
    achieved_aoa, achieved_beta, _ = angles_degrees(rotation, air_velocity)
    achieved_bank = sts_flight_roll(rotation, target_position, bank_offset)
    error = max(
        abs(achieved_aoa - aoa_deg),
        abs(achieved_beta - beta_deg),
        abs(wrap_degrees(achieved_bank - bank_deg)),
    )
    if error > 1e-4:
        raise RuntimeError(f"Hypothetical state misses requested attitude by {error:.6g} deg")
    return {
        "position": target_position,
        "rotation": rotation,
        "velocity": inertial_velocity,
        "targetUT": target_ut,
        "speedMps": speed,
        "achievedMach": None,
        "achievedAoADeg": achieved_aoa,
        "achievedBetaDeg": achieved_beta,
        "achievedBankDeg": achieved_bank,
    }


def rate_requests():
    requests = [{"case": "zero", "axis": None, "rateDegPerSecond": 0.0,
                 "bodyRateRadPerSecondXYZ": [0.0, 0.0, 0.0]}]
    for name, axis in RATE_AXES:
        for sign, polarity in ((-1.0, "negative"), (1.0, "positive")):
            values = [0.0, 0.0, 0.0]
            values[axis] = sign * math.radians(RATE_DEG_PER_S)
            requests.append({
                "case": f"{name}_{polarity}_1deg_s",
                "axis": name,
                "rateDegPerSecond": sign * RATE_DEG_PER_S,
                "bodyRateRadPerSecondXYZ": values,
            })
    return requests


def evaluate_cell(flight, body, state, requests):
    samples = {}
    for request in requests:
        body_rate = tuple(request["bodyRateRadPerSecondXYZ"])
        rate_reference = tuple(rotate(tuple(state["rotation"]), body_rate))
        force_reference, torque_reference = flight.simulate_aerodynamic_wrench_at(
            body,
            state["position"],
            state["velocity"],
            state["rotation"],
            rate_reference,
            state["targetUT"],
        )
        samples[request["case"]] = {
            "bodyRateRadPerSecondXYZ": list(body_rate),
            "forceNBodyXYZ": list(inverse_rotate(tuple(state["rotation"]), tuple(force_reference))),
            "torqueNmBodyXYZ": list(inverse_rotate(tuple(state["rotation"]), tuple(torque_reference))),
        }
    denominator = 2.0 * math.radians(RATE_DEG_PER_S)
    derivative = [[None for _ in range(3)] for _ in range(3)]
    for _, rate_axis in RATE_AXES:
        plus = samples[f"{RATE_AXES[rate_axis][0]}_positive_1deg_s"]["torqueNmBodyXYZ"]
        minus = samples[f"{RATE_AXES[rate_axis][0]}_negative_1deg_s"]["torqueNmBodyXYZ"]
        for torque_axis in range(3):
            derivative[torque_axis][rate_axis] = (plus[torque_axis] - minus[torque_axis]) / denominator
    return samples, derivative


def main() -> int:
    config_path = ROOT / "Configuration/sts.json"
    profile_path = ROOT / "Configuration/checkpoints/sts-campaign-72km-20261008.physics.json"
    trace_path = ROOT / "Runtime/STS-Campaign-20261009/entry-trace/headless-live.log"
    config = json.loads(config_path.read_text())
    profile = json.loads(profile_path.read_text())
    checkpoint_path = Path(profile["savePath"])
    checkpoint_hash = hashlib.sha256(checkpoint_path.read_bytes()).hexdigest()
    if checkpoint_hash != profile["saveSHA256"]:
        raise RuntimeError("Named-checkpoint hash does not match its certified physics profile")
    if config["vehicle"]["modelId"] != "STS" or profile["vessel"] != "STS":
        raise RuntimeError("This probe requires the original STS")

    slices = [read_shadow_witness(altitude) for altitude in ALTITUDES_M]
    trace_hash = hashlib.sha256(trace_path.read_bytes()).hexdigest()
    requests = rate_requests()
    report = {
        "status": "running",
        "classification": "UNQUALIFIED_DIAGNOSTIC_SIMULATION",
        "purpose": "Measure the original-STS KSP aerodynamic torque derivative matrix with respect to hypothetical body rates over a shadow-trace Mach/AoA/sideslip/bank envelope.",
        "qualification": {
            "qualified": False,
            "shadowGateEvidence": False,
            "measuredAerodynamicEvidence": False,
            "flightControlOrActuation": False,
            "landingOrRolloutEvidence": False,
            "note": "All cells are hypothetical Flight.simulate_aerodynamic_wrench_at results. Their reference kinematics come from an ENTRY_TRACE shadow prediction, not measured atmospheric flight.",
        },
        "provenance": {
            "repository": str(ROOT),
            "gitHead": subprocess.run(["git", "rev-parse", "HEAD"], cwd=ROOT, capture_output=True, text=True, check=True).stdout.strip(),
            "sourceScript": "Tools/probe_sts_aero_mimo_envelope.py",
            "sourceSHA256": hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
            "configuration": str(config_path.relative_to(ROOT)),
            "configurationSHA256": hashlib.sha256(config_path.read_bytes()).hexdigest(),
            "vehicleModelId": config["vehicle"]["modelId"],
            "checkpointName": profile["saveName"],
            "checkpointSHA256Before": checkpoint_hash,
            "checkpointUsedAsLoadedVesselSource": True,
            "shadowTrace": str(trace_path.relative_to(ROOT)),
            "shadowTraceSHA256": trace_hash,
            "shadowTraceSlices": [
                {"requestedAltitudeM": altitude, "entryTrace": item[0], "nearestWitness": item[1], "traceSHA256": item[2]}
                for altitude, item in zip(ALTITUDES_M, slices)
            ],
            "api": "SpaceCenter.Flight.simulate_aerodynamic_wrench_at",
            "apiReferenceFrame": "CelestialBody.non_rotating_reference_frame",
            "hypotheticalPosition": "The retained shadow trace's runway-relative along/cross coordinates are projected onto the configured KSC site and rotated by measured Kerbin body spin to the trace UT.",
            "hypotheticalMach": "Mach 4, Mach 5, the selected trace TAS divided by temperature-scaled local sound speed, and Mach 6. The sound-speed scale is calibrated from the nearest retained ENTRY_WITNESS KSP Mach record.",
            "hypotheticalAngles": "AoA 18/28 degrees, sideslip -5/0/+5 degrees, and KSP STS Flight.roll bank -40/0/+40 degrees.",
            "hypotheticalRates": "Zero plus independent +/-1 degree/second rotations around local body X, Y (roll), and Z (yaw); symmetric differences produce d(body torque)/d(body rate). These are not measured flight rates.",
            "surfacePolicy": "Use the loaded original STS and the currently neutral control surfaces. No control, throttle, RCS, SAS, action-group, save, position, velocity, orientation, UT, or physics setters are called.",
            "flightPolicy": "KSP remains paused; only instantaneous hypothetical wrench simulations are requested.",
        },
        "gridDefinition": {
            "targetAltitudesM": list(ALTITUDES_M),
            "machPlan": list(MACH_PLAN),
            "angleOfAttackDeg": list(AOA_DEG),
            "sideslipDeg": list(BETA_DEG),
            "bankDeg": list(BANK_DEG),
            "rateCases": requests,
            "rateDerivativeUnits": "N m per (rad/s)",
            "derivativeMatrixOrientation": "rows=body torque XYZ; columns=body angular velocity XYZ",
        },
        "cells": [],
        "progress": {"completedCells": 0, "totalCells": len(ALTITUDES_M) * len(MACH_PLAN) * len(AOA_DEG) * len(BETA_DEG) * len(BANK_DEG)},
    }
    atomic_report(OUTPUT, report)
    conn = None
    vessel = body = sc = frame = None
    started = time.monotonic()
    try:
        pids_before = pids_named_ksp()
        if len(pids_before) != 1:
            raise RuntimeError(f"Expected one live KSP process, found {pids_before}")
        report["kspProcessPIDsBefore"] = pids_before
        conn = connect("STS read-only aero MIMO envelope", timeout=8)
        sc = conn.space_center
        vessel = sc.active_vessel
        if vessel is None or vessel.name != "STS" or len(sc.vessels) != 1:
            raise RuntimeError("Expected one loaded original STS vessel")
        body = vessel.orbit.body
        if body.name != "Kerbin":
            raise RuntimeError(f"Expected Kerbin, found {body.name}")
        frame = body.non_rotating_reference_frame
        before = state_snapshot(conn, sc, vessel, body, frame)
        actuators_before = read_actuator_snapshot(vessel, config)
        report["liveStateBefore"] = before
        report["liveDryMassKgBefore"] = float(vessel.dry_mass)
        report["liveActuatorStateBefore"] = actuators_before
        if not before["paused"]:
            raise RuntimeError("Refusing the probe unless KSP is paused")
        if before["sas"] or before["rcs"] or before["wheel_brakes"]:
            raise RuntimeError("Refusing the probe while SAS, RCS, or wheel brakes are active")
        if any(abs(float(value)) > 1e-9 for value in before["controls"].values()) or abs(before["thrust_n"]) > 1.0:
            raise RuntimeError("Refusing the probe with nonzero controls or thrust")
        if actuators_before["speedbrakeActionGroupEnabled"] or any(
            surface["deployed"] or abs(surface["deflection"]) > 1e-9
            for surface in actuators_before["controlSurfaces"]
        ):
            raise RuntimeError("Refusing the probe with deployed/deflected surfaces or speedbrakes")
        if before["part_count"] != int(profile["partCount"]):
            raise RuntimeError("Loaded STS part count differs from checkpoint profile")
        if abs(before["mass_kg"] - float(profile["mass"])) > 1.0 or abs(float(vessel.dry_mass) - float(profile["dryMass"])) > 1.0:
            raise RuntimeError("Loaded STS mass differs from the settled checkpoint profile")
        if abs(before["altitude_m"] - 72000.0) > 500.0 or abs(before["ut"] - float(profile["savedUT"])) > 15.0:
            raise RuntimeError("Loaded STS is outside the bounded checkpoint vicinity for this read-only diagnostic")

        checkpoint_bank = bank_degrees(tuple(before["rotation_xyzw"]), tuple(before["position_m"]))
        bank_offset = wrap_degrees(before["roll_deg"] - checkpoint_bank)
        if min(abs(bank_offset), abs(abs(bank_offset) - 180.0)) > 0.2:
            raise RuntimeError("Checkpoint STS bank convention is not aligned to local +Z or -Z")
        vessel_z_sign = -1.0 if abs(bank_offset) > 90.0 else 1.0
        omega = tuple(float(value) for value in body.angular_velocity(frame))

        reference_trace, reference_witness, _ = read_shadow_witness(38000.0)
        reference_lat, reference_lon = runway_offset_lat_lon(
            float(body.equatorial_radius), float(config["site"]["latitude"]),
            float(config["site"]["longitude"]), float(config["site"]["runwayHeading"]),
            float(reference_trace["along"]), float(reference_trace["xtk"]),
        )
        reference_position = tuple(body.position_at_altitude(
            reference_lat, reference_lon, float(reference_trace["h"]), frame
        ))
        reference_position, _ = rotate_by_body_spin(
            reference_position, omega, float(reference_trace["ut"]) - before["ut"]
        )
        reference_temperature = float(body.temperature_at(reference_position, frame))
        reference_sound_speed = float(reference_witness["V"]) / float(reference_witness["M"])
        if not math.isfinite(reference_temperature) or reference_temperature <= 0.0 or reference_sound_speed <= 0.0:
            raise RuntimeError("Invalid KSP sound-speed calibration from retained shadow witness")
        gamma_r = reference_sound_speed * reference_sound_speed / reference_temperature
        report["provenance"]["soundSpeedCalibration"] = {
            "referenceTrace": reference_trace,
            "referenceWitness": reference_witness,
            "referenceTemperatureK": reference_temperature,
            "referenceSoundSpeedMps": reference_sound_speed,
            "gammaRDerived": gamma_r,
            "note": "Sound speed at each hypothetical altitude uses sqrt(gammaR * KSP temperature); gammaR is calibrated to the retained KSP Mach witness.",
        }
        report["loadedCheckpointAssessment"] = {
            "loadedMassKg": before["mass_kg"],
            "profileMassKg": float(profile["mass"]),
            "loadedDryMassKg": float(vessel.dry_mass),
            "profileDryMassKg": float(profile["dryMass"]),
            "partCount": before["part_count"],
            "utOffsetFromSavedEpochSeconds": before["ut"] - float(profile["savedUT"]),
            "note": "This is a paused read-only diagnostic sourced from the loaded named checkpoint. The live state is reported verbatim; no flight attempt is started.",
        }

        flight = vessel.flight(frame)
        for altitude, (trace, witness, _) in zip(ALTITUDES_M, slices):
            target_position = target_state(
                body, frame, config["site"], trace, before["ut"], omega,
                vessel_z_sign, bank_offset, float(trace["V"]), 28.0, 0.0, 0.0,
            )["position"]
            density = float(body.atmospheric_density_at_position(target_position, frame))
            temperature = float(body.temperature_at(target_position, frame))
            if not math.isfinite(density) or density <= 0.0 or not math.isfinite(temperature) or temperature <= 0.0:
                raise RuntimeError(f"Invalid atmosphere at selected {trace['h']:.1f}m shadow trace")
            sound_speed = math.sqrt(gamma_r * temperature)
            trace_mach = float(trace["V"]) / sound_speed
            speed_plan = []
            for mach in MACH_PLAN:
                speed = float(trace["V"]) if mach == "trace" else float(mach) * sound_speed
                speed_plan.append((mach, speed, speed / sound_speed))

            for mach_label, speed, mach_value in speed_plan:
                q_estimate = 0.5 * density * speed * speed
                if q_estimate > float(config["vehicle"].get("maximumDynamicPressure", math.inf)):
                    raise RuntimeError(f"Hypothetical q={q_estimate:.1f}Pa exceeds configured envelope")
                for aoa in AOA_DEG:
                    for beta in BETA_DEG:
                        for bank in BANK_DEG:
                            if not conn.krpc.paused or abs(float(sc.ut) - before["ut"]) > 1e-9:
                                raise RuntimeError("KSP pause or UT changed during hypothetical wrench evaluation")
                            state = target_state(
                                body, frame, config["site"], trace, before["ut"], omega,
                                vessel_z_sign, bank_offset, speed, float(aoa), float(beta), float(bank),
                            )
                            state["achievedMach"] = mach_value
                            samples, derivative = evaluate_cell(flight, body, state, requests)
                            report["cells"].append({
                                "requestedAltitudeM": altitude,
                                "traceAltitudeM": float(trace["h"]),
                                "traceUT": float(trace["ut"]),
                                "machLabel": mach_label,
                                "mach": mach_value,
                                "speedMps": speed,
                                "estimatedDynamicPressurePa": q_estimate,
                                "angleOfAttackDeg": aoa,
                                "sideslipDeg": beta,
                                "bankDeg": bank,
                                "achieved": {
                                    "mach": mach_value,
                                    "angleOfAttackDeg": state["achievedAoADeg"],
                                    "sideslipDeg": state["achievedBetaDeg"],
                                    "bankDeg": state["achievedBankDeg"],
                                },
                                "wrenches": samples,
                                "rateDampingDerivativeMatrixNmPerRadPerSecond": derivative,
                            })
                            report["progress"]["completedCells"] = len(report["cells"])
                            if len(report["cells"]) % 8 == 0:
                                atomic_report(OUTPUT, report)

        report["status"] = "complete"
        report["gridSummary"] = {
            "cellCount": len(report["cells"]),
            "apiCallCount": len(report["cells"]) * len(requests),
            "elapsedWallSeconds": time.monotonic() - started,
        }
    except BaseException as exc:
        report["status"] = "failed"
        report["error"] = f"{type(exc).__name__}: {exc}"
        raise
    finally:
        try:
            if conn is not None:
                if vessel is not None:
                    after = state_snapshot(conn, sc, vessel, body, frame)
                    actuator_after = read_actuator_snapshot(vessel, config)
                    report["liveStateAfter"] = after
                    report["liveDryMassKgAfter"] = float(vessel.dry_mass)
                    report["liveActuatorStateAfter"] = actuator_after
                    report["liveStateUnchanged"] = after == report.get("liveStateBefore")
                    report["liveActuatorStateUnchanged"] = actuator_after == report.get("liveActuatorStateBefore")
                conn.close()
        finally:
            report["checkpointSHA256After"] = hashlib.sha256(checkpoint_path.read_bytes()).hexdigest()
            report["checkpointUnchanged"] = report["checkpointSHA256After"] == checkpoint_hash
            report["kspProcessPIDsAfter"] = pids_named_ksp()
            report["singleKspProcessPreserved"] = (
                report.get("kspProcessPIDsBefore") == report["kspProcessPIDsAfter"]
                and len(report["kspProcessPIDsAfter"]) == 1
            )
            report["probeElapsedWallSeconds"] = time.monotonic() - started
            if report.get("checkpointUnchanged") is False or report.get("singleKspProcessPreserved") is False:
                report["status"] = "failed"
                report["invariantError"] = "Checkpoint or single-KSP invariant changed during the read-only probe"
            atomic_report(OUTPUT, report)
    print(json.dumps({"status": report["status"], "cells": len(report["cells"]),
                      "output": str(OUTPUT), "apiCalls": len(report["cells"]) * len(requests)}))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
