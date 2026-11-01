#!/usr/bin/env python3
"""Audit the zero-bias pitch plant against measured rates; identify no moment slope."""
import argparse
import bisect
import hashlib
import json
import math
from pathlib import Path
import statistics


def cross(a, b):
    return [a[1]*b[2]-a[2]*b[1], a[2]*b[0]-a[0]*b[2], a[0]*b[1]-a[1]*b[0]]


def add(a, b):
    return [x+y for x, y in zip(a, b)]


def inverse_rotate(q, v):
    u = [-q[k] for k in ("x", "y", "z")]
    t = [2*x for x in cross(u, v)]
    return add(v, add([q["w"]*x for x in t], cross(u, t)))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("input", type=Path)
    parser.add_argument("configuration", type=Path)
    parser.add_argument("output", type=Path)
    parser.add_argument("--minimum-mach", type=float, default=3)
    parser.add_argument("--maximum-mach", type=float, default=5)
    parser.add_argument("--derivative-window", type=float, default=1)
    args = parser.parse_args()
    if args.derivative_window <= 0 or args.minimum_mach > args.maximum_mach:
        parser.error("positive derivative window and ordered Mach bounds are required")
    config = json.loads(args.configuration.read_text())
    records = [json.loads(line) for line in args.input.open()]
    records = [r for r in records if "state" in r]
    times = [r["state"]["time"] for r in records]
    if any(b < a for a, b in zip(times, times[1:])):
        raise ValueError("Telemetry epochs are not ordered; split clock-reset segments before auditing")
    rows = []
    for i, record in enumerate(records):
        s = record["state"]; a = s["aerodynamics"]; t = s["time"]
        if not args.minimum_mach <= a["mach"] <= args.maximum_mach or a["dynamicPressure"] <= 200:
            continue
        j = bisect.bisect_left(times, t-args.derivative_window/2)
        k = bisect.bisect_left(times, t+args.derivative_window/2)
        if j >= i or k >= len(records):
            continue
        previous = records[j]["state"]; following = records[k]["state"]
        dt = following["time"]-previous["time"]
        if not 0 < dt <= 2 * args.derivative_window:
            continue  # Do not form a derivative across a missing-data interval.
        authority = s["controlAuthority"]
        inertia = [authority["inertia"][n] for n in ("x", "y", "z")]
        w = [s["angularVelocity"][n] for n in ("x", "y", "z")]
        frame_rate = inverse_rotate(s["attitude"], [0, 0, config["planet"]["rotationRate"]])
        inertial = add(w, frame_rate)
        gyro = cross(inertial, [inertia[n]*inertial[n] for n in range(3)])
        frame = cross(w, frame_rate)
        applied = s["appliedControls"]["pitch"]
        available = min(abs(authority["positiveTorque"]["y"]), abs(authority["negativeTorque"]["y"]))/inertia[1]
        model = available*applied-gyro[1]/inertia[1]+frame[1]
        actual = (following["angularVelocity"]["y"]-previous["angularVelocity"]["y"])/dt
        rows.append({"time": t, "mach": a["mach"], "q": a["dynamicPressure"],
            "alpha_deg": math.degrees(a["angleOfAttack"]),
            "target_alpha_deg": math.degrees(record["guidance"]["target"]["angleOfAttack"]),
            "input": applied, "model_pitch_accel": model, "observed_pitch_accel": actual,
            "unmodeled_accel": actual-model,
            "local_balancing_input": applied-(actual+gyro[1]/inertia[1]-frame[1])/available,
            "saturated": record["control"]["pitchSaturated"], "rcs": s["systems"]["rcs"]})
    if not rows:
        raise ValueError("No matching measured samples")
    def stats(key):
        values = sorted(r[key] for r in rows)
        return {"median": statistics.median(values), "p10": values[int(.1*(len(values)-1))],
            "p90": values[int(.9*(len(values)-1))]}
    result = {"samples": len(rows), "duration_seconds": rows[-1]["time"]-rows[0]["time"],
        "saturated_samples": sum(r["saturated"] for r in rows), "rcs_samples": sum(r["rcs"] for r in rows),
        "input_sha256": hashlib.sha256(args.input.read_bytes()).hexdigest(),
        "configuration_sha256": hashlib.sha256(args.configuration.read_bytes()).hexdigest(),
        "statistics": {key: stats(key) for key in ["alpha_deg", "target_alpha_deg", "input",
            "model_pitch_accel", "observed_pitch_accel", "unmodeled_accel", "local_balancing_input"]},
        "method": "Centered angular-rate derivative with measured input, conservative minimum directional torque, measured inertia, gyroscopic coupling and rotating-frame terms.",
        "limitations": ["A residual audit, not a fitted aerodynamic moment law.",
            "Applied control is not individual surface-deflection telemetry.", "The centered derivative uses future samples and is diagnostic only."]}
    args.output.write_text(json.dumps(result, indent=2)+"\n")
    print(json.dumps(result, indent=2))


if __name__ == "__main__":
    main()
