#!/usr/bin/env python3
"""Recompute STS vacuum yaw-pulse slopes and inertia cross-axis response."""

from __future__ import annotations

import hashlib
import json
import math
from pathlib import Path
import subprocess

ROOT = Path(__file__).resolve().parents[1]
RAW_PATH = ROOT / "Docs/Experiments/STS-Entry-20261008/live-preentry-yaw-repeat.json"
COORDINATE_PATH = ROOT / "Docs/Experiments/STS-Entry-20261008/live-body-coordinate-crosscheck.json"
PROFILE_PATH = ROOT / "Configuration/checkpoints/sts-campaign-72km-20261008.physics.json"
OUTPUT_PATH = ROOT / "Docs/Experiments/STS-Entry-20261009/live-yaw-inertia-analysis.json"


def ols_slope(samples: list[dict], axis_index: int) -> float:
    """Return deg/s² from the probe's body-rate (deg/s) field."""
    xs = [float(sample["elapsed"]) for sample in samples]
    ys = [float(sample["body_rates"][axis_index]) for sample in samples]
    x_mean = sum(xs) / len(xs)
    y_mean = sum(ys) / len(ys)
    denominator = sum((x - x_mean) ** 2 for x in xs)
    if denominator <= 0.0:
        raise ValueError("Cannot fit a body-rate slope over a zero-duration interval")
    return sum((x - x_mean) * (y - y_mean) for x, y in zip(xs, ys)) / denominator


def symmetric_matrix(flat: list[float]) -> tuple[list[list[float]], float]:
    raw = [flat[index:index + 3] for index in range(0, 9, 3)]
    error = max(abs(raw[i][j] - raw[j][i]) for i in range(3) for j in range(3))
    symmetric = [[0.5 * (raw[i][j] + raw[j][i]) for j in range(3)] for i in range(3)]
    return symmetric, error


def solve3(matrix: list[list[float]], vector: list[float]) -> list[float]:
    augmented = [list(matrix[row]) + [vector[row]] for row in range(3)]
    for column in range(3):
        pivot = max(range(column, 3), key=lambda row: abs(augmented[row][column]))
        if abs(augmented[pivot][column]) <= 1e-15:
            raise ValueError("Inertia tensor is singular")
        augmented[column], augmented[pivot] = augmented[pivot], augmented[column]
        divisor = augmented[column][column]
        augmented[column] = [value / divisor for value in augmented[column]]
        for row in range(3):
            if row == column:
                continue
            factor = augmented[row][column]
            augmented[row] = [augmented[row][i] - factor * augmented[column][i] for i in range(4)]
    return [augmented[row][3] for row in range(3)]


def matvec3(matrix: list[list[float]], vector: list[float]) -> list[float]:
    return [sum(matrix[row][column] * vector[column] for column in range(3)) for row in range(3)]


def cross3(a: list[float], b: list[float]) -> list[float]:
    return [
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    ]


def inferred_body_torque(flat: list[float], angular_acceleration: list[float],
        angular_velocity: list[float]) -> dict:
    """Estimate body torque from the rigid-body Euler equation: τ = Iα + ω×Iω."""
    inertia, asymmetry = symmetric_matrix(flat)
    inertia_acceleration = matvec3(inertia, angular_acceleration)
    inertia_rate = matvec3(inertia, angular_velocity)
    gyroscopic = cross3(angular_velocity, inertia_rate)
    total = [inertia_acceleration[index] + gyroscopic[index] for index in range(3)]
    return {
        "inertiaTimesAngularAccelerationNmXYZ": inertia_acceleration,
        "gyroscopicCorrectionNmXYZ": gyroscopic,
        "totalInferredTorqueNmXYZ": total,
        "maximumCapturedTensorAsymmetryKgM2": asymmetry,
    }


def fit_yaw_polarity(samples: list[dict], positive: bool) -> dict:
    selected = [
        sample for sample in samples
        if (float(sample["control_yaw"]) > 0.14 if positive else float(sample["control_yaw"]) < -0.14)
        and abs(float(sample["control_pitch"])) <= 1e-9
        and abs(float(sample["control_roll"])) <= 1e-9
    ]
    if len(selected) < 3:
        raise ValueError("Yaw pulse does not contain enough uncontaminated samples")
    acceleration_deg = [ols_slope(selected, axis) for axis in range(3)]
    acceleration_rad = [value * math.pi / 180.0 for value in acceleration_deg]
    mean_rate_rad = [
        sum(float(sample["body_rates"][axis]) for sample in selected) / len(selected) * math.pi / 180.0
        for axis in range(3)
    ]
    input_axes = {
        key: [min(float(sample[key]) for sample in selected), max(float(sample[key]) for sample in selected)]
        for key in ("control_pitch", "control_roll", "control_yaw")
    }
    control_yaw = float(selected[0]["control_yaw"])
    return {
        "sampleCount": len(selected),
        "elapsedIntervalS": [min(float(x["elapsed"]) for x in selected), max(float(x["elapsed"]) for x in selected)],
        "yawCommand": control_yaw,
        "controlInputs": input_axes,
        "bodyAccelerationDegPerSecondSquaredXYZ": acceleration_deg,
        "bodyAccelerationRadPerSecondSquaredXYZ": acceleration_rad,
        "meanBodyRateRadPerSecondXYZ": mean_rate_rad,
        "rollRadPerSecondSquared": acceleration_rad[1],
        "yawRadPerSecondSquared": acceleration_rad[2],
        "rollDegPerSecondSquared": acceleration_deg[1],
        "yawDegPerSecondSquared": acceleration_deg[2],
        "rollToYawAccelerationRatio": abs(acceleration_rad[1] / acceleration_rad[2]) if acceleration_rad[2] else None,
    }


def inertia_counterfactual(flat: list[float]) -> dict:
    inertia, asymmetry = symmetric_matrix(flat)
    # STS vessel body rates use local +Y for roll and +Z for yaw. A pure yaw
    # actuator torque is therefore a unit moment about body +Z.
    angular_accel = solve3(inertia, [0.0, 0.0, 1.0])
    ratio = abs(angular_accel[1] / angular_accel[2]) if angular_accel[2] else None
    return {
        "unitTorqueBodyVectorNm": [0.0, 0.0, 1.0],
        "axisMapping": "Body-rate component 1 (local +Y) is roll; component 2 (local +Z) is yaw.",
        "angularAccelerationRadPerSecondSquaredPerNmXYZ": angular_accel,
        "rollToYawAngularAccelerationRatio": ratio,
        "maximumCapturedTensorAsymmetryKgM2": asymmetry,
        "calculation": "Symmetrize the captured tensor at floating-point precision, then solve I * alpha = [0, 0, 1] Nm.",
    }


def main() -> None:
    raw = json.loads(RAW_PATH.read_text())
    profile = json.loads(PROFILE_PATH.read_text())
    coordinate_hash = hashlib.sha256(COORDINATE_PATH.read_bytes()).hexdigest()
    raw_hash = hashlib.sha256(RAW_PATH.read_bytes()).hexdigest()
    checkpoint_hash = hashlib.sha256(Path(profile["savePath"]).read_bytes()).hexdigest()
    if checkpoint_hash != profile["saveSHA256"] or checkpoint_hash != raw["profile"]["saveSHA256"]:
        raise RuntimeError("The preserved 72 km checkpoint hash changed")
    samples = raw["samples"]
    if not samples:
        raise RuntimeError("The retained yaw capture has no samples")
    if any(sample.get("SAS") or sample.get("RCS") for sample in samples):
        raise RuntimeError("The retained yaw pulse was not performed with SAS/RCS off")
    if any(abs(float(sample.get("q", 0.0))) > 1e-9 or abs(float(sample.get("thrust", 0.0))) > 1e-6 for sample in samples):
        raise RuntimeError("The retained yaw pulse contains nonzero q or thrust")
    if max(abs(float(sample["control_pitch"])) + abs(float(sample["control_roll"])) for sample in samples) > 1e-9:
        raise RuntimeError("The retained yaw pulse contains pitch or roll command contamination")

    positive = fit_yaw_polarity(samples, True)
    negative = fit_yaw_polarity(samples, False)
    vessel_inertia = inertia_counterfactual(raw["inertia_tensor"])
    root_inertia = inertia_counterfactual(raw["root_inertia_tensor"])
    positive_torque = inferred_body_torque(
        raw["inertia_tensor"], positive["bodyAccelerationRadPerSecondSquaredXYZ"],
        positive["meanBodyRateRadPerSecondXYZ"],
    )
    negative_torque = inferred_body_torque(
        raw["inertia_tensor"], negative["bodyAccelerationRadPerSecondSquaredXYZ"],
        negative["meanBodyRateRadPerSecondXYZ"],
    )
    for fit, torque in ((positive, positive_torque), (negative, negative_torque)):
        torque["yawCommand"] = fit["yawCommand"]
        torque["controlInputs"] = fit["controlInputs"]
        torque["measuredAngularAccelerationRadPerSecondSquaredXYZ"] = fit[
            "bodyAccelerationRadPerSecondSquaredXYZ"
        ]
        torque["meanAngularVelocityRadPerSecondXYZ"] = fit["meanBodyRateRadPerSecondXYZ"]
        torque["inferredTorquePerUnitYawInputNmXYZ"] = [
            value / fit["yawCommand"] for value in torque["totalInferredTorqueNmXYZ"]
        ]
        roll_torque, yaw_torque = torque["totalInferredTorqueNmXYZ"][1:3]
        torque["rollToYawTorqueRatio"] = abs(roll_torque / yaw_torque) if yaw_torque else None
    measured_ratio = 0.5 * (positive["rollToYawAccelerationRatio"] + negative["rollToYawAccelerationRatio"])
    predicted_ratio = vessel_inertia["rollToYawAngularAccelerationRatio"]
    prior_report = json.loads(OUTPUT_PATH.read_text()) if OUTPUT_PATH.exists() else {}
    report = {
        "status": "complete",
        "classification": "UNQUALIFIED_DIAGNOSTIC_ANALYSIS",
        "purpose": "Audit yaw-to-roll response from the repeated live vacuum pulse against the measured STS inertia tensors, with explicit angular-rate units and axis mapping.",
        "qualification": {
            "qualified": False,
            "shadowGateEvidence": False,
            "atmosphericCoordinationEvidence": False,
            "landingOrRolloutEvidence": False,
        },
        "provenance": {
            "gitHeadAtCapture": raw.get("gitHeadAtCapture") or prior_report.get("provenance", {}).get("gitHeadAtCapture"),
            "gitHeadAtAnalysis": subprocess.run(["git", "rev-parse", "HEAD"], cwd=ROOT, capture_output=True, text=True, check=True).stdout.strip(),
            "rawYawPulse": str(RAW_PATH.relative_to(ROOT)),
            "rawYawPulseSHA256": raw_hash,
            "probeSource": "Tools/probe_sts_preentry_roll.py",
            "probeSourceSHA256": hashlib.sha256((ROOT / "Tools/probe_sts_preentry_roll.py").read_bytes()).hexdigest(),
            "coordinateCrosscheck": str(COORDINATE_PATH.relative_to(ROOT)),
            "coordinateCrosscheckSHA256": coordinate_hash,
            "checkpointName": profile["saveName"],
            "checkpointSHA256": checkpoint_hash,
            "analysisScript": str(Path(__file__).relative_to(ROOT)),
        },
        "benchConditions": {
            "vessel": raw["profile"]["vessel"],
            "altitudeM": [min(float(x["altitude"]) for x in samples), max(float(x["altitude"]) for x in samples)],
            "dynamicPressurePa": [min(float(x["q"]) for x in samples), max(float(x["q"]) for x in samples)],
            "maximumAbsoluteThrustN": max(abs(float(x["thrust"])) for x in samples),
            "sasOnAnySample": any(bool(x["SAS"]) for x in samples),
            "rcsOnAnySample": any(bool(x["RCS"]) for x in samples),
            "pitchAndRollInputsZeroDuringYawPulses": True,
            "yawCommands": {
                "positive": positive["yawCommand"],
                "negative": negative["yawCommand"],
            },
            "sampleCount": len(samples),
            "method": "OLS of body-rate components versus elapsed seconds over each constant-yaw-command interval. The probe converts KRPC angular velocity from rad/s to degrees/s when writing body_rates; the report includes the resulting degree/s² slopes and their rad/s² equivalents.",
        },
        "measuredAccelerationRadPerSecondSquared": {
            "positiveYaw": {"roll": positive["rollRadPerSecondSquared"], "yaw": positive["yawRadPerSecondSquared"]},
            "negativeYaw": {"roll": negative["rollRadPerSecondSquared"], "yaw": negative["yawRadPerSecondSquared"]},
        },
        "measuredAccelerationDegPerSecondSquared": {
            "positiveYaw": {"roll": positive["rollDegPerSecondSquared"], "yaw": positive["yawDegPerSecondSquared"]},
            "negativeYaw": {"roll": negative["rollDegPerSecondSquared"], "yaw": negative["yawDegPerSecondSquared"]},
        },
        "measuredBodyAccelerationDegPerSecondSquaredXYZ": {
            "axisOrder": ["local +X pitch", "local +Y roll", "local +Z yaw"],
            "positiveYaw": positive["bodyAccelerationDegPerSecondSquaredXYZ"],
            "negativeYaw": negative["bodyAccelerationDegPerSecondSquaredXYZ"],
        },
        "measuredRollToYawAccelerationRatio": {
            "positiveYaw": positive["rollToYawAccelerationRatio"],
            "negativeYaw": negative["rollToYawAccelerationRatio"],
            "meanAbsolute": measured_ratio,
        },
        "measuredVesselInertiaTensorRowMajorKgM2": raw["inertia_tensor"],
        "measuredRootInertiaTensorRowMajorKgM2": raw["root_inertia_tensor"],
        "inferredBodyTorqueFromMeasuredAcceleration": {
            "axisOrder": ["local +X pitch", "local +Y roll", "local +Z yaw"],
            "method": "Symmetrize the measured full-vessel tensor and apply Euler's rigid-body relation τ=Iα+ω×Iω to each polarity's three OLS body-rate slopes. Rates and slopes are converted from the probe's degrees to radians first. This is an inferred net torque, not a directly measured wheel torque.",
            "positiveYaw": positive_torque,
            "negativeYaw": negative_torque,
            "meanAbsoluteRollToYawTorqueRatio": 0.5 * (
                positive_torque["rollToYawTorqueRatio"] + negative_torque["rollToYawTorqueRatio"]
            ),
            "interpretationLimit": "The vacuum bench rules out aerodynamic torque and has SAS/RCS, thrust, pitch input, and roll input off. Residual external or measurement effects are not independently instrumented; do not treat the inferred torque vector as a direct actuator telemetry measurement.",
        },
        "pureBodyYawTorqueCounterfactual": {
            "unitTorqueBodyVectorNm": [0.0, 0.0, 1.0],
            "vesselAngularAccelerationVectorPerUnitTorqueRadPerSecondSquaredPerNmXYZ": vessel_inertia["angularAccelerationRadPerSecondSquaredPerNmXYZ"],
            "vesselRollToYawAngularAccelerationRatio": vessel_inertia["rollToYawAngularAccelerationRatio"],
            "rootAngularAccelerationVectorPerUnitTorqueRadPerSecondSquaredPerNmXYZ": root_inertia["angularAccelerationRadPerSecondSquaredPerNmXYZ"],
            "rootRollToYawAngularAccelerationRatio": root_inertia["rollToYawAngularAccelerationRatio"],
            "measuredResponseRatioDividedByVesselInertiaPrediction": measured_ratio / predicted_ratio if predicted_ratio else None,
            "vessel": vessel_inertia,
            "rootPart": root_inertia,
        },
        "interpretation": (
            f"The repeated vacuum yaw pulses show a mean absolute roll/yaw acceleration ratio of {measured_ratio:.3f}. "
            f"The full-vessel inertia predicts {predicted_ratio:.3f} for a pure body-z yaw torque, about "
            f"{measured_ratio / predicted_ratio:.1f} times smaller. The captured off-diagonal inertia alone does not explain the cross-axis response. "
            f"Applying the full three-axis Euler relation to the measured slopes implies an input-correlated body-roll torque of about "
            f"{100.0 * 0.5 * (positive_torque['rollToYawTorqueRatio'] + negative_torque['rollToYawTorqueRatio']):.1f}% of the yaw torque in both pulse directions; this is inferred net torque, not a direct wheel measurement, and its actuator source remains unresolved. "
            "The recorded body_rates values are already degrees/s, as specified by the captured probe source; radians/s² equivalents are provided separately. "
            "This does not identify the control-frame or actuator cause and is not atmospheric coordination or flight qualification."
        ),
    }
    report["sourceSHA256"] = hashlib.sha256(Path(__file__).read_bytes()).hexdigest()
    OUTPUT_PATH.write_text(json.dumps(report, indent=2, allow_nan=False) + "\n")
    print(json.dumps({
        "status": report["status"],
        "output": str(OUTPUT_PATH),
        "positiveYawRollDegS2": positive["rollDegPerSecondSquared"],
        "positiveYawYawDegS2": positive["yawDegPerSecondSquared"],
        "negativeYawRollDegS2": negative["rollDegPerSecondSquared"],
        "negativeYawYawDegS2": negative["yawDegPerSecondSquared"],
        "measuredRollToYawRatio": measured_ratio,
        "inertiaRollToYawRatio": predicted_ratio,
    }, separators=(",", ":")))


if __name__ == "__main__":
    main()
