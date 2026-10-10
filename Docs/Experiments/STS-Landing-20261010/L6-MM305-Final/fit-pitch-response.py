#!/usr/bin/env python3
"""Compare body/AoA response on L4 training data and held-out L5 data."""
import json
from pathlib import Path
import sys
from types import SimpleNamespace

ROOT = Path(__file__).resolve().parents[4]
sys.path.insert(0, str(ROOT / "ShuttleSim/tools"))
from identify_stsn_attitude import _log_samples, pairs, fit_axis
from identify_stsn_aero import _merge


def samples(path, coordinate):
    rows = _log_samples(str(path), "STS")
    if coordinate == "aoa":
        rates = {}
        state = {}
        for line in path.open():
            record = json.loads(line)
            kind = record.get("recordType")
            if kind == "vehicleKeyframe":
                state = {}
            if kind not in ("vehicleKeyframe", "vehicleDelta"):
                continue
            _merge(state, record.get("fields", {}))
            rates[record["ut"]] = state.get("attitude", {}).get("angleOfAttackRate")
        for row in rows:
            row["wp"] = rates.get(row["ut"])
    args = SimpleNamespace(min_radar_alt=15, min_q=500, max_mach=.9,
                           max_accel=80, max_body_rate=30, max_sideslip=2)
    return [row for row in pairs(rows, args) if abs(row["roll"]) < 5]


def main():
    train = ROOT / "FlightLogs/2026-10-10T06-29-47Z-STS-vehicle.jsonl"
    heldout = ROOT / "FlightLogs/2026-10-10T07-54-07Z-STS-vehicle.jsonl"
    fits = {}
    for coordinate in ("body", "aoa"):
        fits[coordinate] = fit_axis(samples(train, coordinate), samples(heldout, coordinate),
                                    "pitch", "qd", "wp", "dp", "aoa",
                                    [x*.5 for x in range(2, 41)])
    report = dict(trainLog=str(train.relative_to(ROOT)),
                  heldoutLog=str(heldout.relative_to(ROOT)), vehicleModel="STS",
                  selection="unpowered flying; radar>=15m, q>=500Pa, Mach<=.9, |bank|<5deg, |beta|<2deg; adjacent dt .05-.15s",
                  fits=fits,
                  limitations="Passive closed-loop regression with finite-difference noise and actuator/kinematic lag; body and AoA coordinates fitted separately. No coefficient adopted without replay and held-out checks.")
    path = Path(__file__).with_name("pitch-response-L4-L5.json")
    path.write_text(json.dumps(report, indent=2) + "\n")
    print(json.dumps(report, indent=2))


if __name__ == "__main__":
    main()
