#!/usr/bin/env python3
"""Read-only, multi-altitude original-STS KSP aerodynamic wrench grid."""

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
from probe_sts_aero_wrench_grid import angles_degrees, bank_degrees, cross, dot, inverse_rotate, norm, quaternion_multiply, rotate, state_snapshot, unit, wrap_degrees
from probe_sts_shadow_witness_wrench import pids_named_ksp, read_actuator_snapshot, rotate_by_body_spin, quaternion_from_basis, sts_flight_roll

TRACE_PATH = ROOT / "Runtime/STS-Campaign-20261009/entry-trace/headless-live.log"
OUTPUT_PATH = ROOT / "Docs/Experiments/STS-Entry-20261009/live-aero-envelope-grid-20261009.json"


def read_shadow_witness(target_altitude: float) -> tuple[dict[str, float], dict[str, float], str]:
    traces = []
    witnesses = []
    for line in TRACE_PATH.read_text().splitlines():
        if line.startswith("ENTRY_TRACE ut="):
            row = {}
            for token in line.split():
                if "=" in token:
                    key, value = token.split("=", 1)
                    try:
                        row[key] = float(value.strip('"'))
                    except ValueError:
                        pass
            keys = ("ut", "h", "V", "q", "D", "Dref", "along", "xtk", "Bcmd", "roll", "aoa")
            if all(key in row for key in keys):
                traces.append({key: row[key] for key in keys})
        elif line.startswith("ENTRY_WITNESS state ut="):
            row = {}
            for token in line.split():
                if "=" in token:
                    key, value = token.split("=", 1)
                    try:
                        row[key] = float(value.strip('"'))
                    except ValueError:
                        pass
            if all(key in row for key in ("ut", "h", "V", "M")):
                witnesses.append({key: row[key] for key in ("ut", "h", "V", "M")})
    if not traces or not witnesses:
        raise RuntimeError("The retained shadow trace lacks usable ENTRY_TRACE or ENTRY_WITNESS rows")
    trace = min(traces, key=lambda row: abs(row["h"] - target_altitude))
    witness = min(witnesses, key=lambda row: abs(row["ut"] - trace["ut"]))
    if abs(trace["h"] - target_altitude) > 1500.0 or abs(trace["aoa"] - 28.0) > 0.2:
        raise RuntimeError(f"No AoA=28 shadow trace is within 1.5 km of requested {target_altitude:.0f} m")
    return trace, witness, hashlib.sha256(TRACE_PATH.read_bytes()).hexdigest()


def runway_offset_lat_lon(radius_m: float, latitude_deg: float, longitude_deg: float,
                          heading_deg: float, along_m: float, cross_m: float) -> tuple[float, float]:
    """Reconstruct a trace surface point from its runway along/cross offsets."""
    latitude = math.radians(latitude_deg)
    longitude = math.radians(longitude_deg)
    heading = math.radians(heading_deg)
    north_m = along_m * math.cos(heading) - cross_m * math.sin(heading)
    east_m = along_m * math.sin(heading) + cross_m * math.cos(heading)
    distance = math.hypot(north_m, east_m)
    if distance <= 1e-9:
        return latitude_deg, longitude_deg
    bearing = math.atan2(east_m, north_m)
    angular_distance = distance / radius_m
    sin_latitude = math.sin(latitude)
    cos_latitude = math.cos(latitude)
    sin_delta = math.sin(angular_distance)
    cos_delta = math.cos(angular_distance)
    destination_latitude = math.asin(sin_latitude * cos_delta + cos_latitude * sin_delta * math.cos(bearing))
    destination_longitude = longitude + math.atan2(
        math.sin(bearing) * sin_delta * cos_latitude,
        cos_delta - sin_latitude * math.sin(destination_latitude),
    )
    return math.degrees(destination_latitude), (math.degrees(destination_longitude) + 180.0) % 360.0 - 180.0


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, default=OUTPUT_PATH)
    parser.add_argument("--altitudes-m", type=float, nargs="+", default=[48000.0, 44000.0, 40000.0, 38000.0, 36000.0])
    parser.add_argument("--checkpoint-ut-tolerance-s", type=float, default=2.0)
    args = parser.parse_args()
    if not args.altitudes_m or any(not math.isfinite(value) or value <= 0.0 for value in args.altitudes_m):
        raise ValueError("All target altitudes must be finite and positive")
    if len(set(args.altitudes_m)) != len(args.altitudes_m):
        raise ValueError("Target altitudes must be unique")
    if not math.isfinite(args.checkpoint_ut_tolerance_s) or args.checkpoint_ut_tolerance_s < 0.0:
        raise ValueError("Checkpoint UT tolerance must be finite and non-negative")

    config_path = ROOT / "Configuration/sts.json"
    profile_path = ROOT / "Configuration/checkpoints/sts-campaign-72km-20261008.physics.json"
    config = json.loads(config_path.read_text())
    profile = json.loads(profile_path.read_text())
    checkpoint_path = Path(profile["savePath"])
    checkpoint_hash = hashlib.sha256(checkpoint_path.read_bytes()).hexdigest()
    if checkpoint_hash != profile["saveSHA256"]:
        raise RuntimeError("The original campaign checkpoint hash does not match its physics profile")
    slices = [read_shadow_witness(altitude) for altitude in args.altitudes_m]

    report = {
        "status": "running",
        "classification": "UNQUALIFIED_DIAGNOSTIC_SIMULATION",
        "purpose": "Sample the original-STS KSP aerodynamic wrench model across high-AoA shadow-trace altitude slices, Mach estimates, sideslip, and bank.",
        "qualification": {
            "qualified": False,
            "shadowGateEvidence": False,
            "measuredAerodynamicEvidence": False,
            "flightControlOrActuation": False,
            "landingOrRolloutEvidence": False,
            "note": "All returned forces and torques are counterfactual KSP simulations. They are not flight measurements, a shadow-gate pass, or landing qualification.",
        },
        "provenance": {
            "repository": str(ROOT),
            "gitHead": subprocess.run(["git", "rev-parse", "HEAD"], cwd=ROOT, capture_output=True, text=True, check=True).stdout.strip(),
            "configuration": str(config_path.relative_to(ROOT)),
            "configurationSHA256": hashlib.sha256(config_path.read_bytes()).hexdigest(),
            "vehicleModelId": config["vehicle"]["modelId"],
            "checkpointProfile": str(profile_path.relative_to(ROOT)),
            "checkpointName": profile["saveName"],
            "checkpointSHA256Before": checkpoint_hash,
            "checkpointUsedAsLoadedVesselSource": True,
            "trace": str(TRACE_PATH.relative_to(ROOT)),
            "traceSHA256PerSlice": [item[2] for item in slices],
            "inputTraceRecords": [item[0] for item in slices],
            "nearestShadowWitnessRecords": [item[1] for item in slices],
            "aeroHelperSourceSHA256": hashlib.sha256(Path(__file__).with_name("probe_sts_aero_wrench_grid.py").read_bytes()).hexdigest(),
            "geometryHelperSourceSHA256": hashlib.sha256(Path(__file__).with_name("probe_sts_shadow_witness_wrench.py").read_bytes()).hexdigest(),
            "api": "SpaceCenter.Flight.simulate_aerodynamic_wrench_at",
            "referenceFrame": "CelestialBody.non_rotating_reference_frame",
            "apiUt": "Each retained shadow trace UT is passed to the wrench call. The target latitude/longitude is reconstructed from runway-relative along/cross offsets, then its KSC local frame is rotated from current paused UT using measured body angular velocity.",
            "machPolicy": "The logged KSP Mach witness near 38 km calibrates speed of sound at its measured altitude. Other altitude sound speeds are scaled by sqrt(KSP temperature ratio) using temperature_at at each target point. This is a derived Mach estimate, not a direct hypothetical Mach output; exact trace TAS is also sampled.",
            "densityAndDynamicPressurePolicy": "KSP atmospheric_density_at_position and temperature_at are queried at each reconstructed future inertial point using live paused UT; wrench calculations use the shadow trace UT explicitly. Both ephemeris times are recorded; q is computed as 0.5*rho*V^2.",
            "angularVelocityPolicy": "Zero hypothetical vessel angular velocity to isolate static aerodynamic force and moment; no damping moment is included.",
            "surfaceStatePolicy": "Use current original-STS parts and neutral control surfaces. No control, action group, throttle, RCS, SAS, save, position, velocity, orientation, or time setters are called.",
        },
        "gridDefinition": {
            "targetAltitudesM": args.altitudes_m,
            "nominalMachValuesByAltitude": [],
            "angleOfAttackDeg": [18.0, 28.0],
            "sideslipDeg": [-5.0, 0.0, 5.0],
            "bankDeg": [-40.0, 0.0, 40.0],
            "bankConvention": "KSP Flight.roll for the original STS, calibrated from its loaded 72 km checkpoint attitude; the loaded checkpoint local +Z axis is opposite radial up.",
            "pressureBoundPa": float(config["vehicle"].get("maximumDynamicPressure", math.inf)),
        },
        "samples": [],
        "altitudeSummaries": [],
        "positionGeometryBySlice": [],
    }

    conn = None
    started = time.monotonic()
    try:
        pids_before = pids_named_ksp()
        if len(pids_before) != 1:
            raise RuntimeError(f"Expected one live KSP process; found {pids_before}")
        report["kspProcessPIDsBefore"] = pids_before
        conn = connect("STS read-only multi-altitude aerodynamic envelope", timeout=8)
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
        report["liveStateBefore"] = before
        report["liveActuatorStateBefore"] = actuators_before
        current_ut = before["ut"]
        checkpoint_geometric_bank = bank_degrees(tuple(before["rotation_xyzw"]), tuple(before["position_m"]))
        bank_offset = wrap_degrees(before["roll_deg"] - checkpoint_geometric_bank)
        if min(abs(bank_offset), abs(abs(bank_offset) - 180.0)) > 0.2:
            raise RuntimeError("The checkpoint STS bank convention is not aligned with local +Z or -Z")
        if not before["paused"]:
            raise RuntimeError("Refusing to probe while KSP is unpaused")
        if abs(current_ut - float(profile["savedUT"])) > args.checkpoint_ut_tolerance_s or abs(before["altitude_m"] - 72000.0) > 500.0:
            raise RuntimeError("The live STS is not at the saved 72 km checkpoint state")
        if before["sas"] or before["rcs"]:
            raise RuntimeError("Refusing to probe with SAS or RCS enabled")
        if any(abs(value) > 1e-9 for value in before["controls"].values()) or abs(before["thrust_n"]) > 1.0:
            raise RuntimeError("Refusing to probe with direct input or actual engine thrust")
        if actuators_before["speedbrakeActionGroupEnabled"]:
            raise RuntimeError("Refusing to probe with the configured speedbrake action group enabled")
        if any(surface["deployed"] or abs(surface["deflection"]) > 1e-9 for surface in actuators_before["controlSurfaces"]):
            raise RuntimeError("Refusing to probe with a deployed or deflected control surface")
        if len(vessel.parts.all) != int(profile["partCount"]):
            raise RuntimeError("Loaded original-STS part count does not match the checkpoint profile")
        if abs(before["mass_kg"] - float(profile["mass"])) > 1.0 or abs(float(vessel.dry_mass) - float(profile["dryMass"])) > 1.0:
            raise RuntimeError("Loaded vessel mass does not match the settled checkpoint profile")

        site = config["site"]
        site_lat = float(site["latitude"])
        site_lon = float(site["longitude"])
        heading_deg = float(site["runwayHeading"])
        omega = tuple(float(x) for x in body.angular_velocity(frame))
        vessel_z_sign = -1.0 if abs(bank_offset) > 90.0 else 1.0
        reference_trace, reference_witness, _ = read_shadow_witness(38000.0)
        reference_lat, reference_lon = runway_offset_lat_lon(
            float(body.equatorial_radius), site_lat, site_lon, heading_deg,
            reference_trace["along"], reference_trace["xtk"],
        )
        reference_base_position = tuple(body.position_at_altitude(
            reference_lat, reference_lon, reference_trace["h"], frame
        ))
        reference_target_position, _ = rotate_by_body_spin(
            reference_base_position, omega, reference_trace["ut"] - current_ut
        )
        reference_temperature = float(body.temperature_at(reference_target_position, frame))
        reference_sound_speed = reference_witness["V"] / reference_witness["M"]
        if not math.isfinite(reference_temperature) or reference_temperature <= 0.0:
            raise RuntimeError("KSP returned invalid atmospheric temperature at the Mach calibration point")
        if not math.isfinite(reference_sound_speed) or reference_sound_speed <= 0.0:
            raise RuntimeError("The retained witness cannot calibrate a positive speed of sound")
        report["provenance"]["soundSpeedCalibration"] = {
            "referenceTrace": reference_trace,
            "referenceWitness": reference_witness,
            "referenceTemperatureK": reference_temperature,
            "referenceSoundSpeedMps": reference_sound_speed,
            "referenceTemperatureQueryUT": current_ut,
            "referenceWitnessUT": reference_witness["ut"],
            "referenceWrenchAtmosphericEphemerisUT": reference_trace["ut"],
            "assumption": "For a fixed atmospheric gas composition, sound speed scales with the square root of temperature; the effective gamma-R constant is calibrated to the recorded KSP V/M witness.",
        }
        gamma_r = reference_sound_speed * reference_sound_speed / reference_temperature
        max_angle_error = 0.0
        max_bank_error = 0.0
        for requested_altitude, (trace, witness, _) in zip(args.altitudes_m, slices):
            altitude = trace["h"]
            target_ut = trace["ut"]
            elapsed = target_ut - current_ut
            target_lat, target_lon = runway_offset_lat_lon(
                float(body.equatorial_radius), site_lat, site_lon, heading_deg, trace["along"], trace["xtk"]
            )
            base_position = tuple(body.position_at_altitude(target_lat, target_lon, altitude, frame))
            surface = tuple(body.position_at_altitude(target_lat, target_lon, 0.0, frame))
            north_sample = tuple(body.position_at_altitude(target_lat + 0.01, target_lon, 0.0, frame))
            east_sample = tuple(body.position_at_altitude(target_lat, target_lon + 0.01, 0.0, frame))
            radial_up = unit(base_position)
            north_raw = tuple(north_sample[i] - surface[i] for i in range(3))
            east_raw = tuple(east_sample[i] - surface[i] for i in range(3))
            north = unit(tuple(north_raw[i] - dot(north_raw, radial_up) * radial_up[i] for i in range(3)))
            east = unit(tuple(east_raw[i] - dot(east_raw, radial_up) * radial_up[i] for i in range(3)))
            heading = math.radians(heading_deg)
            forward = unit(tuple(north[i] * math.cos(heading) + east[i] * math.sin(heading) for i in range(3)))
            vessel_z = unit(tuple(vessel_z_sign * (radial_up[i] - dot(radial_up, forward) * forward[i]) for i in range(3)))
            right = unit(cross(forward, vessel_z))
            local_rotation = quaternion_from_basis(right, forward, vessel_z)
            target_position, spin_rotation = rotate_by_body_spin(base_position, omega, elapsed)
            base_rotation = quaternion_multiply(spin_rotation, local_rotation)
            base_geometric_bank = bank_degrees(base_rotation, target_position)
            base_bank = sts_flight_roll(base_rotation, target_position, bank_offset)
            if abs(wrap_degrees(base_bank)) > 0.2:
                raise RuntimeError(f"Constructed runway-aligned orientation has bank {base_bank:.3f} degrees")
            if not conn.krpc.paused or abs(float(sc.ut) - current_ut) > 1e-9:
                raise RuntimeError("KSP pause or UT changed during hypothetical position construction")

            density = float(body.atmospheric_density_at_position(target_position, frame))
            if not math.isfinite(density) or density <= 0.0:
                raise RuntimeError(f"KSP returned invalid atmosphere density at {altitude:.1f} m")
            temperature = float(body.temperature_at(target_position, frame))
            if not math.isfinite(temperature) or temperature <= 0.0:
                raise RuntimeError(f"KSP returned invalid atmospheric temperature at {altitude:.1f} m")
            sound_speed = math.sqrt(gamma_r * temperature)
            trace_speed = trace["V"]
            speed_plan = [
                {"nominalMachEstimate": 4.0, "speedMps": 4.0 * sound_speed, "source": "adjacent-witness sound-speed scaling"},
                {"nominalMachEstimate": 5.0, "speedMps": 5.0 * sound_speed, "source": "temperature-scaled sound-speed estimate"},
                {"nominalMachEstimate": trace_speed / sound_speed, "speedMps": trace_speed, "source": "exact selected ENTRY_TRACE TAS"},
                {"nominalMachEstimate": 6.0, "speedMps": 6.0 * sound_speed, "source": "adjacent-witness sound-speed scaling"},
            ]
            report["gridDefinition"]["nominalMachValuesByAltitude"].append({
                "traceAltitudeM": altitude,
                "traceUT": target_ut,
                "values": [item["nominalMachEstimate"] for item in speed_plan],
            })
            slice_start = len(report["samples"])
            for speed_item in speed_plan:
                speed = speed_item["speedMps"]
                q_estimate = 0.5 * density * speed * speed
                if q_estimate > report["gridDefinition"]["pressureBoundPa"]:
                    raise RuntimeError(f"Diagnostic q estimate {q_estimate:.1f} Pa exceeds the configured pressure bound")
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
                            max_bank_error = max(max_bank_error, bank_error)
                            if bank_error > 0.2:
                                raise RuntimeError("Counterfactual orientation missed requested runway-relative bank")
                            body_air_velocity = (
                                speed * math.sin(beta),
                                speed * math.cos(beta) * math.cos(alpha),
                                speed * math.cos(beta) * math.sin(alpha),
                            )
                            air_velocity = tuple(rotate(rotation, body_air_velocity))
                            atmosphere_velocity = cross(omega, target_position)
                            inertial_velocity = tuple(air_velocity[i] + atmosphere_velocity[i] for i in range(3))
                            achieved_aoa, achieved_beta, _ = angles_degrees(rotation, air_velocity)
                            angle_error = max(abs(achieved_aoa - aoa), abs(achieved_beta - sideslip))
                            max_angle_error = max(max_angle_error, angle_error)
                            if angle_error > 0.05:
                                raise RuntimeError("Counterfactual velocity missed requested AoA/sideslip")
                            if not conn.krpc.paused or abs(float(sc.ut) - current_ut) > 1e-9:
                                raise RuntimeError("KSP pause or UT changed during counterfactual wrench calls")
                            force_raw, torque_raw = vessel.flight(frame).simulate_aerodynamic_wrench_at(
                                body, target_position, inertial_velocity, rotation, (0.0, 0.0, 0.0), target_ut
                            )
                            force = tuple(float(x) for x in force_raw)
                            torque = tuple(float(x) for x in torque_raw)
                            drag_n = -dot(force, unit(air_velocity))
                            report["samples"].append({
                                "targetAltitudeM": altitude,
                                "traceUT": target_ut,
                                "densityQueryUT": current_ut,
                                "temperatureK": temperature,
                                "soundSpeedEstimateMps": sound_speed,
                                "nominalMachEstimate": speed_item["nominalMachEstimate"],
                                "speedSource": speed_item["source"],
                                "trueAirSpeedMps": speed,
                                "angleOfAttackDeg": aoa,
                                "sideslipDeg": sideslip,
                                "bankDeg": bank,
                                "achieved": {"angleOfAttackDeg": achieved_aoa, "sideslipDeg": achieved_beta, "bankDeg": achieved_bank},
                                "densityKgM3": density,
                                "dynamicPressureEstimatePa": q_estimate,
                                "forceN_nonrotating": force,
                                "forceN_bodyXYZ": tuple(inverse_rotate(rotation, force)),
                                "dragN_alongRelativeAirflow": drag_n,
                                "dragAccelerationMps2_usingCheckpointMass": drag_n / float(profile["mass"]),
                                "effectiveDragAreaM2_usingKspDensityEstimate": drag_n / q_estimate,
                                "torqueNm_nonrotating": torque,
                                "torqueNm_bodyXYZ": tuple(inverse_rotate(rotation, torque)),
                            })
            slice_samples = report["samples"][slice_start:]
            matched = min(slice_samples, key=lambda sample:
                abs(sample["trueAirSpeedMps"] - trace_speed)
                + 100.0 * abs(sample["angleOfAttackDeg"] - trace["aoa"])
                + 100.0 * abs(sample["sideslipDeg"])
                + 100.0 * abs(sample["bankDeg"] - trace["roll"]))
            beta_samples = {
                sample["sideslipDeg"]: sample for sample in slice_samples
                if sample["speedSource"] == "exact selected ENTRY_TRACE TAS"
                and abs(sample["angleOfAttackDeg"] - trace["aoa"]) < 0.01
                and abs(sample["bankDeg"] - trace["roll"]) < 1.0
            }
            roll_slope = yaw_slope = None
            if -5.0 in beta_samples and 5.0 in beta_samples:
                roll_slope = (beta_samples[5.0]["torqueNm_bodyXYZ"][1] - beta_samples[-5.0]["torqueNm_bodyXYZ"][1]) / 10.0
                yaw_slope = (beta_samples[5.0]["torqueNm_bodyXYZ"][2] - beta_samples[-5.0]["torqueNm_bodyXYZ"][2]) / 10.0
            q_trace_tas = 0.5 * density * trace_speed * trace_speed
            report["altitudeSummaries"].append({
                "requestedAltitudeM": requested_altitude,
                "traceAltitudeM": altitude,
                "traceUT": target_ut,
                "traceState": trace,
                "nearestShadowWitness": witness,
                "soundSpeedEstimateMps": sound_speed,
                "witnessTimeOffsetS": witness["ut"] - target_ut,
                "densityQueryUT": current_ut,
                "densityEphemerisOffsetS": current_ut - target_ut,
                "densityKgM3": density,
                "temperatureK": temperature,
                "soundSpeedEstimateMps": sound_speed,
                "estimatedMachAtTraceTAS": trace_speed / sound_speed,
                "estimatedDynamicPressureAtTraceTASPa": q_trace_tas,
                "shadowTraceDynamicPressurePa": trace["q"],
                "estimatedQToShadowQRatio": q_trace_tas / trace["q"] if trace["q"] else None,
                "matchedStaticWrenchSampleIndex": report["samples"].index(matched),
                "simulatedDragAccelerationMps2": matched["dragAccelerationMps2_usingCheckpointMass"],
                "shadowModelDragAccelerationMps2": trace["D"],
                "shadowDRAGREFAccelerationMps2": trace["Dref"],
                "simulationToShadowModelRatio": matched["dragAccelerationMps2_usingCheckpointMass"] / trace["D"] if trace["D"] else None,
                "simulationToDRAGREFRatio": matched["dragAccelerationMps2_usingCheckpointMass"] / trace["Dref"] if trace["Dref"] else None,
                "staticRollMomentSlopeNmPerDegSideslip": roll_slope,
                "staticYawMomentSlopeNmPerDegSideslip": yaw_slope,
                "qualification": "UNQUALIFIED_DIAGNOSTIC_SIMULATION",
            })
            report["positionGeometryBySlice"].append({
                "targetAltitudeM": altitude,
                "traceUT": target_ut,
                "currentPausedUT": current_ut,
                "traceAlongTrackM": trace["along"],
                "traceCrossTrackM": trace["xtk"],
                "reconstructedTraceLatitudeDeg": target_lat,
                "reconstructedTraceLongitudeDeg": target_lon,
                "densityQueryUT": current_ut,
                "wrenchAtmosphericEphemerisUT": target_ut,
                "bodyAngularVelocityRadPerS_nonrotating": omega,
                "spinExtrapolationSeconds": elapsed,
                "targetPositionNonrotatingM": target_position,
                "checkpointFlightRollDeg": before["roll_deg"],
                "checkpointGeometricLocalZBankDeg": checkpoint_geometric_bank,
                "stsFlightRollGeometricOffsetDeg": bank_offset,
                "vesselLocalZSignRelativeToRadialUp": vessel_z_sign,
                "baseGeometricLocalZBankDeg": base_geometric_bank,
                "baseBankBeforeCounterfactualRollDeg": base_bank,
            })

        report["gridSummary"] = {
            "sampleCount": len(report["samples"]),
            "sliceCount": len(args.altitudes_m),
            "nominalMachSliceCountPerAltitude": 4,
            "maximumAchievedAngleErrorDeg": max_angle_error,
            "maximumAchievedBankErrorDeg": max_bank_error,
            "elapsedWallSeconds": time.monotonic() - started,
        }
        report["liveStateBefore"] = before
        report["liveActuatorStateBefore"] = actuators_before
        report["kspProcessPIDsBefore"] = pids_before
        report["loadedMassAssessment"] = {
            "pausedLoadedMassKg": before["mass_kg"],
            "checkpointProfileMassKg": float(profile["mass"]),
            "pausedLoadedDryMassKg": float(vessel.dry_mass),
            "checkpointProfileDryMassKg": float(profile["dryMass"]),
            "massStateMatchesSettledProfile": abs(before["mass_kg"] - float(profile["mass"])) < 1.0,
            "massUsedForDragAcceleration": "Checkpoint profile mass; it agrees within 1 kg with the settled live vessel mass.",
            "torqueLimitation": "The returned torque is a hypothetical static KSP aerodynamic result about the settled vessel center of mass, not measured flight torque or actuator response.",
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
                report["checkpointUnchanged"] = report["checkpointSHA256After"] == checkpoint_hash == profile["saveSHA256"]
                report["kspProcessPIDsAfter"] = pids_named_ksp()
                report["singleKspProcessPreserved"] = report.get("kspProcessPIDsBefore") == report["kspProcessPIDsAfter"] and len(report["kspProcessPIDsAfter"]) == 1
                if not report["liveStateUnchanged"] or not report["liveActuatorStateUnchanged"] or not report["checkpointUnchanged"] or not report["singleKspProcessPreserved"]:
                    report["status"] = "failed"
                    report["invariantFailure"] = "Live state, actuators, checkpoint hash, or single-KSP invariant changed."
            except Exception as post_error:
                report["postProbeCaptureError"] = f"{type(post_error).__name__}: {post_error}"
                report["status"] = "failed"
            try:
                conn.close()
            except Exception:
                pass
        report["sourceSHA256"] = hashlib.sha256(Path(__file__).read_bytes()).hexdigest()
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(json.dumps(report, indent=2, allow_nan=False) + "\n")

    print(json.dumps({
        "status": report["status"],
        "output": str(args.output),
        "samples": len(report["samples"]),
        "altitudeSummaries": report.get("altitudeSummaries", []),
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
