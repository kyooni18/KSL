#!/usr/bin/env python3
"""Refit L4 pitch response; cross-check L5 and the new L7 without adopting it."""
import bisect
import hashlib
import importlib.util
import json
from pathlib import Path

ROOT = Path(__file__).resolve().parents[4]
SOURCE = ROOT / "Docs/Experiments/STS-Landing-20261010/L6-MM305-Final/fit-pitch-response.py"
spec = importlib.util.spec_from_file_location("pitch_fit", SOURCE)
fit = importlib.util.module_from_spec(spec)
spec.loader.exec_module(fit)
LOGS = {
    "L4": "FlightLogs/2026-10-10T06-29-47Z-STS-vehicle.jsonl",
    "L5": "FlightLogs/2026-10-10T07-54-07Z-STS-vehicle.jsonl",
    "L7": "FlightLogs/2026-10-11T07-55-25Z-STS-vehicle.jsonl",
}


def delayed_control(rows, delay):
    """Use only a preceding selected observation; reject gaps over 0.15 s."""
    if delay == 0:
        return rows
    times = [r["ut"] for r in rows]
    result = []
    for row in rows:
        target = row["ut"] - delay
        i = bisect.bisect_right(times, target) - 1
        if i >= 0 and target - times[i] <= .15:
            result.append(dict(row, dp=rows[i]["dp"]))
    return result


def main():
    report = {
        "logs": {k: {"path": v, "sha256": hashlib.sha256((ROOT / v).read_bytes()).hexdigest()}
                 for k, v in LOGS.items()},
        "selection": "Existing L4/L5 fitter: unpowered flying, radar >=15m, q>=500Pa, "
                     "Mach<=.9, |bank|<5deg, |beta|<2deg, adjacent dt .05-.15s",
        "model": "acceleration = min(Q,cap)*control_gain*applied_pitch + rate_damping*rate "
                 "- Q*stiffness*(AoA-trim); Q in kPa, angles in degrees",
        "selectionPolicy": "All cap choices fit L4 only; L5/L7 are held out. Delay sweep is "
                           "diagnostic, not a selected production model.",
        "limitations": "Closed-loop passive regression, finite-difference noise and limited "
                       "excitation. Body pitch acceleration and AoA acceleration differ through "
                       "flight-path dynamics. Simulator direct_step integrates AoA acceleration; "
                       "body-fit coefficients cannot be substituted without that kinematic term. "
                       "The INI damping ratio scales damping with sqrt(Q), unlike this fitted "
                       "constant rate coefficient. No plant or golden changed.",
        "fits": {},
    }
    for coordinate in ("body", "aoa"):
        rows = {k: fit.samples(ROOT / v, coordinate) for k, v in LOGS.items()}
        diagnostics = []
        for delay in (0.0, .04, .08, .12, .20, .30, .40):
            selected = {k: delayed_control(v, delay) for k, v in rows.items()}
            candidate = fit.fit_axis(selected["L4"], selected["L5"], "pitch", "qd", "wp",
                                     "dp", "aoa", [x*.5 for x in range(2, 41)])
            l7 = fit.fit_axis(selected["L4"], selected["L7"], "pitch", "qd", "wp",
                             "dp", "aoa", [x*.5 for x in range(2, 41)])
            diagnostics.append({"controlDelaySeconds": delay, "fit": candidate,
                                "L7HeldoutR2": l7["heldout_r2"],
                                "L7HeldoutN": l7["heldout_n"],
                                "L7HeldoutByQ": l7["heldout_by_q"]})
        report["fits"][coordinate] = diagnostics
    output = Path(__file__).with_name("result.json")
    output.write_text(json.dumps(report, indent=2) + "\n")
    for coordinate, candidates in report["fits"].items():
        for c in candidates:
            print(coordinate, c["controlDelaySeconds"], "train", round(c["fit"]["train_r2"], 3),
                  "L5", round(c["fit"]["heldout_r2"], 3), "L7", round(c["L7HeldoutR2"], 3))


if __name__ == "__main__":
    main()
