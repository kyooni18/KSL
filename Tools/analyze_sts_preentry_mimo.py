#!/usr/bin/env python3
"""Compare original-STS vacuum roll/yaw pulse responses in body axes."""

from __future__ import annotations

import hashlib
import json
import math
from pathlib import Path
import subprocess

from analyze_sts_yaw_inertia import inferred_body_torque, solve3, symmetric_matrix


ROOT = Path(__file__).resolve().parents[1]
OUTPUT = ROOT / "Docs/Experiments/STS-Entry-20261009/live-preentry-mimo-analysis.json"
TRACE_SPECS = [
    {
        "label": "roll-pulse-1",
        "path": "Docs/Experiments/STS-Entry-20261009/live-preentry-roll-crosscheck-20261009.json",
        "controlAxis": "roll",
        "ratesField": "body_rates",
        "rateSignToRightHanded": -1.0,
    },
    {
        "label": "roll-pulse-2",
        "path": "Docs/Experiments/STS-Entry-20261009/live-preentry-roll-crosscheck-repeat-20261009.json",
        "controlAxis": "roll",
        "ratesField": "body_rates",
        "rateSignToRightHanded": -1.0,
    },
    {
        "label": "yaw-pulse-1",
        "path": "Docs/Experiments/STS-Entry-20261008/live-preentry-yaw-repeat.json",
        "controlAxis": "yaw",
        "ratesField": "body_rates",
        "rateSignToRightHanded": -1.0,
    },
    {
        "label": "yaw-pulse-2-native-frame",
        "path": "Docs/Experiments/STS-Entry-20261009/live-yaw-frame-crosscheck-20261009.json",
        "controlAxis": "yaw",
        "ratesField": "vesselFrameRatesDegPerSecond",
        "rateSignToRightHanded": 1.0,
    },
]
AXIS_INDEX = {"pitch": 0, "roll": 1, "yaw": 2}
RATE_TO_RAD = math.pi / 180.0


def field(sample: dict, stem: str) -> float:
    """Read camelCase or snake_case telemetry keys without changing units."""
    candidates = (stem, stem[0].lower() + stem[1:])
    snake = "".join(("_" + c.lower()) if c.isupper() else c for c in stem).lstrip("_")
    candidates += (snake,)
    if stem in AXIS_INDEX:
        candidates += ("control_" + stem, "control" + stem.capitalize())
    for key in candidates:
        if key in sample:
            return float(sample[key])
    raise KeyError(f"Missing {stem!r}; checked {candidates!r}")


def rate_vector(sample: dict, spec: dict) -> list[float]:
    return [float(value) for value in sample[spec["ratesField"]]]


def fit_slope(samples: list[dict], spec: dict, axis: int) -> float:
    xs = [float(sample["elapsed"]) for sample in samples]
    ys = [rate_vector(sample, spec)[axis] for sample in samples]
    x_mean = sum(xs) / len(xs)
    y_mean = sum(ys) / len(ys)
    denominator = sum((x - x_mean) ** 2 for x in xs)
    if denominator <= 0.0:
        raise ValueError("Cannot fit rates over a zero-duration pulse interval")
    return sum((x - x_mean) * (y - y_mean) for x, y in zip(xs, ys)) / denominator


def pure_axis_prediction(flat: list[float], commanded_axis: int) -> dict:
    matrix, asymmetry = symmetric_matrix(flat)
    unit = [0.0, 0.0, 0.0]
    unit[commanded_axis] = 1.0
    alpha = solve3(matrix, unit)
    cross_axis = 3 - commanded_axis  # roll↔yaw; this report only analyzes those inputs.
    return {
        "unitTorqueAxis": ["pitch", "roll", "yaw"][commanded_axis],
        "alphaPerUnitTorqueRadPerSecondSquaredPerNmXYZ": alpha,
        "crossToPrimaryAngularAccelerationRatio": abs(alpha[cross_axis] / alpha[commanded_axis]),
        "maximumTensorAsymmetryKgM2": asymmetry,
    }


def analyze_polarity(raw: dict, spec: dict, positive: bool) -> dict:
    axis = spec["controlAxis"]
    control_field = "control_" + axis
    if "samples" in raw and raw["samples"] and "controlPitch" in raw["samples"][0]:
        control_field = {"pitch": "controlPitch", "roll": "controlRoll", "yaw": "controlYaw"}[axis]
    selected = [
        sample for sample in raw["samples"]
        if (field(sample, control_field) > 0.14 if positive else field(sample, control_field) < -0.14)
    ]
    if len(selected) < 3:
        raise ValueError(f"{spec['label']} has too few samples at {'positive' if positive else 'negative'} {axis} control")
    other_controls = [name for name in ("pitch", "roll", "yaw") if name != axis]
    control_ranges = {
        name: [min(field(sample, name) for sample in selected), max(field(sample, name) for sample in selected)]
        for name in (axis, *other_controls)
    }
    if any(abs(value) > 1e-8 for name in other_controls for value in control_ranges[name]):
        raise ValueError(f"{spec['label']} has a nonselected control input during the fitted interval")

    rate_sign = float(spec["rateSignToRightHanded"])
    acceleration = [fit_slope(selected, spec, i) * rate_sign * RATE_TO_RAD for i in range(3)]
    mean_rate = [
        sum(rate_vector(sample, spec)[i] for sample in selected) / len(selected) * rate_sign * RATE_TO_RAD
        for i in range(3)
    ]
    tensor = raw.get("inertia_tensor")
    if tensor is None:
        tensor = raw["preflight"]["inertiaTensor"]
    torque = inferred_body_torque(tensor, acceleration, mean_rate)
    command = sum(field(sample, control_field) for sample in selected) / len(selected)
    return {
        "sampleCount": len(selected),
        "elapsedSeconds": [min(float(s["elapsed"]) for s in selected), max(float(s["elapsed"]) for s in selected)],
        "meanActualControl": command,
        "actualControlRanges": control_ranges,
        "bodyAngularAccelerationRadPerSecondSquaredXYZ": acceleration,
        "meanRightHandedBodyRateRadPerSecondXYZ": mean_rate,
        "inferredNetTorqueNmXYZ": torque["totalInferredTorqueNmXYZ"],
        "inertiaTimesAlphaNmXYZ": torque["inertiaTimesAngularAccelerationNmXYZ"],
        "gyroscopicCorrectionNmXYZ": torque["gyroscopicCorrectionNmXYZ"],
        "inferredNetTorquePerUnitControlNmXYZ": [value / command for value in torque["totalInferredTorqueNmXYZ"]],
    }


def summarize_run(raw: dict, spec: dict) -> dict:
    pos = analyze_polarity(raw, spec, True)
    neg = analyze_polarity(raw, spec, False)
    control_axis = spec["controlAxis"]
    primary = AXIS_INDEX[control_axis]
    cross = 2 if control_axis == "roll" else 1
    tensor = raw.get("inertia_tensor")
    if tensor is None:
        tensor = raw["preflight"]["inertiaTensor"]
    predicted = pure_axis_prediction(tensor, primary)
    normalized_alpha = [
        (pos["bodyAngularAccelerationRadPerSecondSquaredXYZ"][i] / pos["meanActualControl"]
         + neg["bodyAngularAccelerationRadPerSecondSquaredXYZ"][i] / neg["meanActualControl"]) / 2
        for i in range(3)
    ]
    normalized_torque = [
        (pos["inferredNetTorquePerUnitControlNmXYZ"][i]
         + neg["inferredNetTorquePerUnitControlNmXYZ"][i]) / 2
        for i in range(3)
    ]
    alpha_ratio = abs(normalized_alpha[cross] / normalized_alpha[primary])
    torque_ratio = abs(normalized_torque[cross] / normalized_torque[primary])
    frame_label = "right-handed vessel axes: +X pitch, +Y roll, +Z yaw"
    return {
        "label": spec["label"],
        "controlAxis": control_axis,
        "bodyAxisOrder": ["+X pitch", "+Y roll", "+Z yaw"],
        "rateSourceField": spec["ratesField"],
        "rateSignCorrectionToRightHanded": spec["rateSignToRightHanded"],
        "rightHandedRateFrame": frame_label,
        "positiveControl": pos,
        "negativeControl": neg,
        "symmetricMeanAngularAccelerationPerUnitControlRadPerSecondSquaredXYZ": normalized_alpha,
        "symmetricMeanInferredNetTorquePerUnitControlNmXYZ": normalized_torque,
        "observedAbsCrossToPrimaryAccelerationRatio": alpha_ratio,
        "pureAxisInertiaPredictedAbsCrossToPrimaryAccelerationRatio": predicted["crossToPrimaryAngularAccelerationRatio"],
        "observedAccelerationRatioDividedByInertiaPrediction": alpha_ratio / predicted["crossToPrimaryAngularAccelerationRatio"],
        "inferredAbsCrossToPrimaryNetTorqueRatio": torque_ratio,
        "pureAxisInertiaCounterfactual": predicted,
        "physicalConditions": {
            "vessel": raw["profile"]["vessel"] if "profile" in raw else raw["preflight"].get("vessel"),
            "checkpointName": raw.get("profile", {}).get("saveName"),
            "checkpointSHA256": raw.get("profile", {}).get("saveSHA256"),
            "massKgRange": [min(float(s["mass"]) for s in raw["samples"]), max(float(s["mass"]) for s in raw["samples"])],
            "altitudeMRange": [min(float(s["altitude"]) for s in raw["samples"]), max(float(s["altitude"]) for s in raw["samples"])],
            "dynamicPressurePaRange": [min(float(s["q"]) for s in raw["samples"]), max(float(s["q"]) for s in raw["samples"])],
            "maximumAbsoluteThrustN": max(abs(float(s["thrust"])) for s in raw["samples"]),
            "sasOnAnySample": any(bool(s.get("SAS", s.get("sas", False))) for s in raw["samples"]),
            "rcsOnAnySample": any(bool(s.get("RCS", s.get("rcs", False))) for s in raw["samples"]),
        },
    }


def main() -> None:
    runs = []
    inputs = []
    for spec in TRACE_SPECS:
        path = ROOT / spec["path"]
        raw_bytes = path.read_bytes()
        raw = json.loads(raw_bytes)
        if raw.get("profile", {}).get("saveSHA256") != "bb27c762add826731df3ad57332efd77d3d3a5d0928caa2ca361907ae0b1703f":
            raise ValueError(f"{spec['label']} does not cite the preserved 72 km checkpoint hash")
        if raw.get("profile", {}).get("vessel") != "STS" or raw.get("completed") is not True:
            raise ValueError(f"{spec['label']} is not a completed original-STS bench")
        if any(float(sample["q"]) != 0.0 or abs(float(sample["thrust"])) > 1.0
                or bool(sample.get("SAS", sample.get("sas", False)))
                or bool(sample.get("RCS", sample.get("rcs", False)))
                for sample in raw["samples"]):
            raise ValueError(f"{spec['label']} contains a non-vacuum or assisted sample")
        runs.append(summarize_run(raw, spec))
        inputs.append({
            "label": spec["label"],
            "path": spec["path"],
            "sha256": hashlib.sha256(raw_bytes).hexdigest(),
            "probeSourceSHA256": raw.get("sourceSHA256"),
            "sampleCount": len(raw["samples"]),
        })

    by_axis = {}
    for axis in ("roll", "yaw"):
        axis_runs = [run for run in runs if run["controlAxis"] == axis]
        primary = AXIS_INDEX[axis]
        cross = 2 if axis == "roll" else 1
        by_axis[axis] = {
            "repeatCount": len(axis_runs),
            "meanObservedAbsCrossToPrimaryAccelerationRatio": sum(r["observedAbsCrossToPrimaryAccelerationRatio"] for r in axis_runs) / len(axis_runs),
            "observedAbsCrossToPrimaryAccelerationRatioByRun": {r["label"]: r["observedAbsCrossToPrimaryAccelerationRatio"] for r in axis_runs},
            "meanPureAxisInertiaPrediction": sum(r["pureAxisInertiaPredictedAbsCrossToPrimaryAccelerationRatio"] for r in axis_runs) / len(axis_runs),
            "meanInferredAbsCrossToPrimaryNetTorqueRatio": sum(r["inferredAbsCrossToPrimaryNetTorqueRatio"] for r in axis_runs) / len(axis_runs),
            "symmetricMeanInferredNetTorquePerUnitControlNmXYZ": [
                sum(r["symmetricMeanInferredNetTorquePerUnitControlNmXYZ"][i] for r in axis_runs) / len(axis_runs)
                for i in range(3)
            ],
            "axisIndices": {"primary": primary, "cross": cross},
        }

    report = {
        "status": "complete",
        "classification": "UNQUALIFIED_LIVE_VACUUM_DIAGNOSTIC",
        "purpose": "Compare the measured original-STS vacuum roll/yaw response with full-vessel inertia using repeated three-axis rate pulses and actual control readbacks.",
        "qualification": {
            "qualified": False,
            "shadowGateEvidence": False,
            "atmosphericCoordinationEvidence": False,
            "landingOrRolloutEvidence": False,
        },
        "method": {
            "fit": "OLS of all three body-rate components against elapsed seconds for each constant actual-control polarity; actual control readback selects the window, not the issued command.",
            "rateConvention": "Legacy body_rates are negated relative to right-handed transformed angular velocity and are sign-corrected before Euler analysis. The independent yaw frame crosscheck already stores right-handed vessel-frame rates.",
            "torqueInference": "Apply the measured symmetric full-vessel inertia tensor to each three-axis angular-acceleration fit and add omega cross I-omega. Reported torque is an inferred net vessel moment, not direct reaction-wheel telemetry.",
            "inertiaCounterfactual": "Invert the measured full-vessel tensor for a unit torque aligned only with the commanded body axis; compare its cross/primary angular-acceleration ratio with the measured ratio.",
            "axisOrder": ["local +X pitch", "local +Y roll", "local +Z yaw"],
        },
        "provenance": {
            "gitHeadAtAnalysis": subprocess.run(["git", "rev-parse", "HEAD"], cwd=ROOT, capture_output=True, text=True, check=True).stdout.strip(),
            "checkpointName": "STS-campaign-72km-20261008",
            "checkpointSHA256": "bb27c762add826731df3ad57332efd77d3d3a5d0928caa2ca361907ae0b1703f",
            "postRunCheckpointVerification": {
                "method": "Separate bounded read-only kRPC query after settle_known_checkpoint.py restored the saved checkpoint.",
                "vessel": "STS",
                "ut": 9931.251826169331,
                "paused": True,
                "altitudeM": 71919.67588308488,
                "dynamicPressurePa": 0.0,
                "massKg": 25816.10546875,
                "dryMassKg": 20348.0,
                "partCount": 22,
                "thrustN": 0.0,
                "sas": False,
                "rcs": False,
                "controls": [0.0, 0.0, 0.0],
                "kspPid": 69877,
                "singleKspProcess": True,
                "checkpointSHA256": "bb27c762add826731df3ad57332efd77d3d3a5d0928caa2ca361907ae0b1703f",
            },
            "inputTraces": inputs,
            "controlAndWheelFrameEvidence": [
                {
                    "path": path,
                    "sha256": hashlib.sha256((ROOT / path).read_bytes()).hexdigest(),
                }
                for path in (
                    "Docs/Experiments/STS-Entry-20261008/live-control-frames.json",
                    "Docs/Experiments/STS-Entry-20261008/live-wheel-frames.json",
                )
            ],
            "inertiaTensorDocumentation": "kRPC Vessel API documents the full 3x3 tensor in row-major order in Vessel.reference_frame.",
            "analyzer": str(Path(__file__).relative_to(ROOT)),
        },
        "runAnalyses": runs,
        "summaryByControlAxis": by_axis,
        "interpretation": (
            "Across independent pulses, the right-handed rate response is repeatable and substantially exceeds the cross-axis acceleration predicted by a pure-axis torque passed through the measured inertia tensor. "
            "The inferred net torque vector therefore contains input-correlated cross-axis components under this vacuum bench. The available wheel/control-frame snapshots show a single wheel and control frame aligned with the root, but do not identify the physical source of the inferred cross-axis moment. "
            "The source remains unresolved. These zero-q measurements do not establish atmospheric control-surface coordination or justify changing entry guidance."
        ),
    }
    report["sourceSHA256"] = hashlib.sha256(Path(__file__).read_bytes()).hexdigest()
    OUTPUT.parent.mkdir(parents=True, exist_ok=True)
    OUTPUT.write_text(json.dumps(report, indent=2, allow_nan=False) + "\n")
    print(json.dumps({
        "status": report["status"],
        "output": str(OUTPUT),
        "summaryByControlAxis": by_axis,
    }, separators=(",", ":")))


if __name__ == "__main__":
    main()
