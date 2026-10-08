#!/usr/bin/env python3
"""Compare a few read-only STS KSP wrench simulations with prior live samples."""

from __future__ import annotations

import hashlib
import json
import math
from pathlib import Path
import sqlite3
import sys

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT / "Tools") not in sys.path:
    sys.path.insert(0, str(ROOT / "Tools"))

from ksp_test_guard import connect
from probe_sts_aero_wrench_grid import (
    bank_degrees,
    cross,
    dot,
    norm,
    quaternion_multiply,
    rotate,
    state_snapshot,
    unit,
    wrap_degrees,
)


def percentile(values: list[float], fraction: float) -> float:
    ordered = sorted(values)
    if not ordered:
        raise ValueError("Cannot compute a percentile from an empty sample")
    index = (len(ordered) - 1) * fraction
    lower = math.floor(index)
    upper = math.ceil(index)
    return ordered[lower] + (ordered[upper] - ordered[lower]) * (index - lower)


def query_live_samples(database: Path) -> tuple[str, list[dict[str, float | str | int]]]:
    uri = f"file:{database.resolve()}?mode=ro"
    connection = sqlite3.connect(uri, uri=True)
    try:
        context = connection.execute(
            "SELECT environment_id FROM physics_contexts "
            "WHERE structure_id='model:STS' ORDER BY last_seen DESC LIMIT 1"
        ).fetchone()
        if not context:
            raise RuntimeError("No original-STS physics context exists in the observation store")
        rows = connection.execute(
            """SELECT a.session_id,a.ut,a.q,a.mach,a.aoa,a.beta,a.roll,a.mass,
                      a.gear,a.brakes,a.airbrakes,a.rcs,
                      a.pos_x,a.pos_y,a.pos_z,
                      a.lift_x,a.lift_y,a.lift_z,a.drag_x,a.drag_y,a.drag_z,
                      quality.control_profile,quality.quality
               FROM aero_observations AS a
               JOIN aero_observation_quality AS quality ON quality.observation_id=a.id
               WHERE a.structure_id='model:STS' AND a.environment_id=?
                 AND quality.eligible=1
                 AND a.mach BETWEEN 7.05 AND 7.16
                 AND a.q BETWEEN 40 AND 55
                 AND a.aoa BETWEEN 22 AND 26
                 AND abs(a.beta)<0.1
                 AND a.gear=0 AND a.brakes=0 AND a.airbrakes=0
               ORDER BY a.id""",
            (context[0],),
        ).fetchall()
    finally:
        connection.close()

    result = []
    for row in rows:
        result.append({
            "sessionId": row[0], "ut": row[1], "qPa": row[2], "mach": row[3],
            "aoaDeg": row[4], "betaDeg": row[5], "bankDeg": row[6],
            "massKg": row[7], "gear": row[8], "brakes": row[9],
            "airbrakes": row[10], "rcs": row[11],
            "positionM": row[12:15],
            "liftN": norm(row[15:18]), "dragN": norm(row[18:21]),
            "controlProfile": row[21], "quality": row[22],
        })
    return str(context[0]), result


def summarize_live_samples(samples: list[dict[str, float | str | int]]) -> dict:
    if not samples:
        raise RuntimeError("No prior original-STS samples match the requested comparison envelope")
    drag_per_q = [float(s["dragN"]) / float(s["qPa"]) for s in samples]
    lift_per_q = [float(s["liftN"]) / float(s["qPa"]) for s in samples]
    lift_drag = [float(s["liftN"]) / float(s["dragN"]) for s in samples]
    altitudes = [norm(s["positionM"]) - 600000.0 for s in samples]

    def span(key: str) -> list[float]:
        values = [float(s[key]) for s in samples]
        return [min(values), max(values)]

    def quantiles(values: list[float]) -> dict[str, float]:
        return {
            "median": percentile(values, 0.5),
            "q25": percentile(values, 0.25),
            "q75": percentile(values, 0.75),
        }

    return {
        "sampleCount": len(samples),
        "sessionIds": sorted({str(s["sessionId"]) for s in samples}),
        "controlProfiles": sorted({str(s["controlProfile"]) for s in samples}),
        "conditions": {
            "mach": span("mach"), "dynamicPressurePa": span("qPa"),
            "angleOfAttackDeg": span("aoaDeg"), "sideslipDeg": span("betaDeg"),
            "bankDeg": span("bankDeg"), "altitudeM": [min(altitudes), max(altitudes)],
            "massKg": span("massKg"),
            "gearStates": sorted({int(s["gear"]) for s in samples}),
            "brakesStates": sorted({int(s["brakes"]) for s in samples}),
            "airbrakeStates": sorted({int(s["airbrakes"]) for s in samples}),
            "rcsStates": sorted({int(s["rcs"]) for s in samples}),
        },
        "measuredForcePerDynamicPressure": {
            "dragM2": quantiles(drag_per_q),
            "liftM2": quantiles(lift_per_q),
            "liftToDrag": quantiles(lift_drag),
            "effectiveBallisticCoefficientKgM2": quantiles([
                float(s["massKg"]) / value for s, value in zip(samples, drag_per_q)
            ]),
        },
        "warning": "These are earlier quality-eligible live telemetry observations. They are not samples from the current admission attempt and do not certify a flight envelope.",
    }


def main() -> None:
    output = ROOT / "Docs/Experiments/STS-Entry-20261009/live-aero-measured-crosscheck.json"
    grid_path = ROOT / "Docs/Experiments/STS-Entry-20261009/live-aerodynamic-wrench-grid.json"
    database = ROOT / "Runtime/Physics/observations.sqlite3"
    config_path = ROOT / "Configuration/sts.json"
    checkpoint_profile_path = ROOT / "Configuration/checkpoints/sts-campaign-72km-20261008.physics.json"

    grid = json.loads(grid_path.read_text())
    config = json.loads(config_path.read_text())
    checkpoint_profile = json.loads(checkpoint_profile_path.read_text())
    checkpoint_path = Path(checkpoint_profile["savePath"])
    checkpoint_sha = hashlib.sha256(checkpoint_path.read_bytes()).hexdigest()
    if checkpoint_sha != checkpoint_profile["saveSHA256"]:
        raise RuntimeError("Reference campaign checkpoint hash does not match its profile")
    database_sha_before = hashlib.sha256(database.read_bytes()).hexdigest()
    environment_id, live_samples = query_live_samples(database)
    live_summary = summarize_live_samples(live_samples)

    measured_state = grid["liveStateBefore"]
    position = tuple(measured_state["position_m"])
    base_rotation = tuple(measured_state["rotation_xyzw"])
    measured_bank = bank_degrees(base_rotation, position)
    target_mach = percentile([float(s["mach"]) for s in live_samples], 0.5)
    target_aoa = percentile([float(s["aoaDeg"]) for s in live_samples], 0.5)
    target_beta = percentile([float(s["betaDeg"]) for s in live_samples], 0.5)
    target_bank = percentile([float(s["bankDeg"]) for s in live_samples], 0.5)
    target_speed = target_mach * float(measured_state["speed_of_sound_mps"])
    body_air_velocity = (
        target_speed * math.sin(math.radians(target_beta)),
        target_speed * math.cos(math.radians(target_beta)) * math.cos(math.radians(target_aoa)),
        target_speed * math.cos(math.radians(target_beta)) * math.sin(math.radians(target_aoa)),
    )
    bank_delta = math.radians(wrap_degrees(measured_bank - target_bank))
    roll_delta = (0.0, math.sin(bank_delta / 2.0), 0.0, math.cos(bank_delta / 2.0))
    target_rotation = quaternion_multiply(base_rotation, roll_delta)
    target_air_velocity = rotate(target_rotation, body_air_velocity)
    target_speed_mps = norm(target_air_velocity)
    source_q = float(measured_state["dynamic_pressure_pa"])
    source_speed = float(measured_state["true_air_speed_mps"])
    estimated_q = source_q * (target_speed_mps / source_speed) ** 2

    report = {
        "status": "running",
        "classification": "UNQUALIFIED_DIAGNOSTIC_CROSSCHECK",
        "qualification": {
            "qualified": False, "shadowGateEvidence": False,
            "flightQualification": False, "landingOrRolloutEvidence": False,
        },
        "purpose": "Check a same-state kRPC aerodynamic wrench simulation against prior original-STS live samples at closely matched Mach, AoA, sideslip, bank, altitude, and dynamic pressure.",
        "provenance": {
            "gitHeadAtCapture": __import__("subprocess").run(
                ["git", "rev-parse", "HEAD"], cwd=ROOT, capture_output=True,
                text=True, check=True,
            ).stdout.strip(),
            "vehicleModelId": config["vehicle"]["modelId"],
            "api": "SpaceCenter.Flight.simulate_aerodynamic_wrench_at",
            "apiInputsFrom": "Previously captured measured-state grid input at 62.8 km; only Mach/AoA/sideslip/bank are changed to the median matching live sample.",
            "gridInput": str(grid_path.relative_to(ROOT)),
            "gridInputSHA256": hashlib.sha256(grid_path.read_bytes()).hexdigest(),
            "physicsObservationStore": str(database.relative_to(ROOT)),
            "physicsObservationStoreSHA256BeforeAndAfter": database_sha_before,
            "physicsContext": environment_id,
            "configurationSHA256": hashlib.sha256(config_path.read_bytes()).hexdigest(),
            "checkpointName": checkpoint_profile["saveName"],
            "checkpointSHA256BeforeAndAfter": checkpoint_sha,
            "checkpointUnchanged": checkpoint_sha == checkpoint_profile["saveSHA256"],
            "checkpointUsedForInputState": False,
            "currentAdmissionAttemptUsedAsMeasuredInput": False,
        },
        "currentKspPreflight": None,
        "matchedLiveInput": {
            "mach": target_mach, "angleOfAttackDeg": target_aoa,
            "sideslipDeg": target_beta, "bankDeg": target_bank,
            "estimatedDynamicPressurePa": estimated_q,
            "speedOfSoundMpsFromMeasuredInput": measured_state["speed_of_sound_mps"],
            "positionM": position, "universalTime": measured_state["ut"],
            "angularVelocityRadS": [0.0, 0.0, 0.0],
            "note": "The simulator query is a counterfactual static wrench; zero angular velocity is used to match the grid convention.",
        },
        "priorLiveSamples": live_summary,
        "simulatedWrench": None,
        "errors": [],
    }

    connection = None
    try:
        connection = connect("STS matched measured-state wrench crosscheck", timeout=8)
        sc = connection.space_center
        vessel = sc.active_vessel
        if vessel is None or vessel.name != "STS":
            raise RuntimeError("Requires the original STS as the active vessel")
        body = vessel.orbit.body
        frame = body.non_rotating_reference_frame
        before = state_snapshot(connection, sc, vessel, body, frame)
        action_group = int(config["vehicle"].get("airbrakeActionGroup", 0))
        airbrake_group_on = bool(vessel.control.get_action_group(action_group))
        report["currentKspPreflight"] = {
            "state": before, "airbrakeActionGroup": action_group,
            "airbrakeActionGroupOn": airbrake_group_on,
        }
        if not before["paused"]:
            raise RuntimeError("Refusing counterfactual call while live KSP is unpaused")
        if before["sas"] or any(abs(value) > 1e-9 for value in before["controls"].values()):
            raise RuntimeError("Refusing unless SAS and direct controls are off")
        if abs(before["thrust_n"]) > 1.0 or airbrake_group_on:
            raise RuntimeError("Refusing unless thrust and the configured airbrake group are off")

        flight = vessel.flight(frame)
        body_omega = tuple(body.angular_velocity(frame))
        atmosphere_velocity = cross(body_omega, position)
        inertial_velocity = tuple(
            target_air_velocity[i] + atmosphere_velocity[i] for i in range(3)
        )
        wrench = flight.simulate_aerodynamic_wrench_at(
            body, position, inertial_velocity, target_rotation, (0.0, 0.0, 0.0),
            float(measured_state["ut"]),
        )
        force = tuple(wrench[0])
        torque = tuple(wrench[1])
        velocity_direction = unit(target_air_velocity)
        drag_n = -dot(force, velocity_direction)
        lift_vector = tuple(force[i] + drag_n * velocity_direction[i] for i in range(3))
        lift_n = norm(lift_vector)
        q = estimated_q
        simulated = {
            "forceN_nonrotating": force, "torqueNm_nonrotating": torque,
            "dragN": drag_n, "liftMagnitudeN": lift_n,
            "dragPerDynamicPressureM2": drag_n / q,
            "liftPerDynamicPressureM2": lift_n / q,
            "liftToDrag": lift_n / drag_n,
            "effectiveBallisticCoefficientKgM2": float(measured_state["mass_kg"]) * q / drag_n,
            "pressureLimitPa": float(config["vehicle"].get("maximumDynamicPressure", math.inf)),
        }
        report["simulatedWrench"] = simulated
        report["comparison"] = {
            "dragPerQRatioSimToLiveMedian": simulated["dragPerDynamicPressureM2"] /
                live_summary["measuredForcePerDynamicPressure"]["dragM2"]["median"],
            "liftPerQRatioSimToLiveMedian": simulated["liftPerDynamicPressureM2"] /
                live_summary["measuredForcePerDynamicPressure"]["liftM2"]["median"],
            "liftToDragDifference": simulated["liftToDrag"] -
                live_summary["measuredForcePerDynamicPressure"]["liftToDrag"]["median"],
            "note": "Comparison is descriptive: the matching telemetry spans a measured-state envelope rather than an identical single timestamp. The simulated wrench remains unqualified.",
        }
        after = state_snapshot(connection, sc, vessel, body, frame)
        report["liveStateAfter"] = after
        report["airbrakeActionGroupOnAfter"] = bool(vessel.control.get_action_group(action_group))
        report["liveStateUnchanged"] = before == after
        if not report["liveStateUnchanged"] or report["airbrakeActionGroupOnAfter"] != airbrake_group_on:
            raise RuntimeError("Live KSP state changed during the read-only wrench query")
        report["status"] = "complete"
    except BaseException as exc:
        report["status"] = "failed"
        report["errors"].append(str(exc))
        raise
    finally:
        if connection is not None:
            connection.close()
        database_sha_after = hashlib.sha256(database.read_bytes()).hexdigest()
        checkpoint_sha_after = hashlib.sha256(checkpoint_path.read_bytes()).hexdigest()
        report["physicsObservationStoreUnchanged"] = database_sha_before == database_sha_after
        report["checkpointUnchanged"] = checkpoint_sha == checkpoint_sha_after == checkpoint_profile["saveSHA256"]
        report["physicsObservationStoreSHA256After"] = database_sha_after
        report["checkpointSHA256After"] = checkpoint_sha_after
        report["sourceSHA256"] = hashlib.sha256(Path(__file__).read_bytes()).hexdigest()
        output.parent.mkdir(parents=True, exist_ok=True)
        output.write_text(json.dumps(report, indent=2, allow_nan=False) + "\n")

    print(json.dumps({
        "status": report["status"], "output": str(output),
        "matchedSamples": live_summary["sampleCount"],
        "liveStateUnchanged": report.get("liveStateUnchanged"),
        "checkpointUnchanged": report.get("checkpointUnchanged"),
        "storeUnchanged": report.get("physicsObservationStoreUnchanged"),
        "comparison": report.get("comparison"),
    }, separators=(",", ":")))


if __name__ == "__main__":
    main()
