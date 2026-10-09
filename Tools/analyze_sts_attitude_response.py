#!/usr/bin/env python3
"""Offline temporal-block audit of borrowed STS attitude servo dynamics."""
import argparse
import hashlib
import json
import math
from pathlib import Path
import statistics
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "ShuttleSim/tools"))
from normalize_ksp_vehicle_log import decode_log
from fit_attitude_from_telemetry import simulate_score, percentile, wrap_deg


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--log", type=Path, default=ROOT / "FlightLogs/2026-10-07T21-34-08Z-STS-vehicle.jsonl")
    ap.add_argument("--report", type=Path, required=True)
    args = ap.parse_args()
    with args.log.open() as stream:
        header = json.loads(next(stream))
    if header.get("vessel") != "STS":
        raise ValueError("requires exact original-STS archived source")
    keys = ["ut", "dynamic_pressure_pa", "aoa_deg", "bank_deg", "target_aoa_deg", "target_roll_deg"]
    raw = [r for r in decode_log(args.log, 0) if
           all(isinstance(r.get(k), (int, float)) and math.isfinite(r[k]) for k in keys)]
    retained = []
    for r in raw:
        if not retained or r["ut"] - retained[-1]["ut"] >= .099:
            retained.append(r)
    rows = []
    for previous, r in zip(retained, retained[1:]):
        dt = r["ut"] - previous["ut"]
        if not .015 <= dt <= .3 or r["dynamic_pressure_pa"] < 500:
            continue
        rows.append(dict(t=r["ut"], q=r["dynamic_pressure_pa"], aoa=r["aoa_deg"],
            bank=r["bank_deg"], target_aoa=r["target_aoa_deg"], target_bank=r["target_roll_deg"],
            pitch_rate=(r["aoa_deg"]-previous["aoa_deg"])/dt,
            roll_rate=wrap_deg(r["bank_deg"]-previous["bank_deg"])/dt))
    if not rows:
        raise ValueError("no usable high-q response samples")
    blocks = {}
    for r in rows:
        block = int((r["t"]-rows[0]["t"])/20)
        blocks.setdefault(block, []).append(r)
    train = [(str(k), v) for k, v in blocks.items() if k % 2 == 0 and len(v) >= 3]
    held = [(str(k), v) for k, v in blocks.items() if k % 2 == 1 and len(v) >= 3]
    if not train or not held:
        raise ValueError("requires disjoint temporal blocks for fit and evaluation")
    path = ROOT / "ShuttleSim/reference-model/stsn_attitude_identified.ini"
    params = dict(line.split("=", 1) for line in path.read_text().splitlines() if "=" in line and not line.startswith("#"))
    report = dict(classification="unqualified offline response-model audit; source run bypassed shadow gate",
        sourceSHA256=hashlib.sha256(args.log.read_bytes()).hexdigest(),
        baselineSHA256=hashlib.sha256(path.read_bytes()).hexdigest(),
        sourceVessel=header["vessel"], sampleCount=len(rows), qMinimumPa=500,
        integrationDtS=.02, trainBlocks=[k for k, _ in train], holdoutBlocks=[k for k, _ in held],
        ratePolicy="Backward difference of observed AoA/bank coordinates, not body angular rates; roll difference wrapped.",
        limitations=["Temporal blocks of the same historically bypassed flight, not held-out vehicle flights or certification.",
            "Independent axis servo omits pitch/roll/yaw aerodynamic coupling and actual actuator/surface state.",
            "No rate/acceleration or flight limits changed; q authority onset is not fitted from saturated high-q data."], axes={})
    for axis in ("pitch", "roll"):
        vmax = float(params[f"max_{axis}_rate_deg_s"])
        amax = float(params[f"max_{axis}_accel_deg_s2"])
        wn = float(params[f"{axis}_wn"])
        zeta = float(params[f"{axis}_zeta"])
        q_full = float(params[f"{axis}_full_authority_q_pa"])
        def score(data, w, z):
            return simulate_score(data, axis, w, z, vmax, amax, 500, .02, q_full=q_full)
        baseline_train, baseline_held = score(train, wn, zeta), score(held, wn, zeta)
        ranked = [(score(train, w, z), w, z) for w in (.4, .7, 1., 1.5, 2.5)
                  for z in (.5, .85, 1.3, 2.)]
        ranked.sort()
        best, bw, bz = ranked[0]
        fitted_held = score(held, bw, bz)
        report["axes"][axis] = dict(baselineParameters=dict(wn=wn, zeta=zeta, rate=vmax, acceleration=amax, qFull=q_full),
            baselineTrainScore=baseline_train, baselineHoldoutScore=baseline_held,
            trainSelectedParameters=dict(wn=bw, zeta=bz), fittedTrainScore=best, fittedHoldoutScore=fitted_held,
            scoreFields=["objective", "meanErrorDeg", "p95ErrorDeg", "p99ErrorDeg", "transientP95ErrorDeg"],
            observedCoordinateRateP995=percentile([abs(r[f"{axis}_rate"]) for r in rows], .995),
            fitOnGridBoundary=bw in (.4, 2.5) or bz in (.5, 2.))
        print(axis, "baseline holdout", baseline_held, "train-selected", bw, bz, "holdout", fitted_held, flush=True)
    args.report.write_text(json.dumps(report, indent=2, allow_nan=False) + "\n")


if __name__ == "__main__":
    main()
