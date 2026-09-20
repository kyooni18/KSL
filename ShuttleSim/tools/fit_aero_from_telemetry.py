#!/usr/bin/env python3
"""Fit a robust Mach x AoA CL/CD grid from normalized KSP telemetry CSV.

Required columns:
  mach, aoa_deg, dynamic_pressure_pa, lift_accel_mps2,
  drag_accel_mps2, mass_kg

When present, sideslip/thrust/airbrake/gear columns are used as quality gates so
the clean-airframe table is not contaminated by lateral-force projection,
powered flight, airbrakes, or landing-gear drag.
"""
import argparse
import csv
import math
import statistics


def number(row, key, default=0.0):
    try:
        v = row.get(key, "")
        return float(v) if v not in ("", None) else default
    except (TypeError, ValueError):
        return default


def flag(row, key):
    v = str(row.get(key, "")).strip().lower()
    return v in ("1", "true", "yes", "on")


def robust_weighted(neighbors, value_index):
    values = [x[value_index] for x in neighbors]
    med = statistics.median(values)
    deviations = [abs(v - med) for v in values]
    mad = statistics.median(deviations) if deviations else 0.0
    floor = 0.015 if value_index == 2 else 0.025
    limit = max(3.5 * mad, floor)
    kept = [x for x in neighbors if abs(x[value_index] - med) <= limit]
    if len(kept) < max(8, len(neighbors) // 4):
        kept = neighbors

    sw = sv = 0.0
    for item in kept:
        distance2 = item[4]
        w = 1.0 / (0.025 + distance2)
        sw += w
        sv += w * item[value_index]
    return sv / sw


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("input")
    ap.add_argument("output")
    ap.add_argument("--area", type=float, default=250.0)
    ap.add_argument("--min-q", type=float, default=25.0)
    ap.add_argument("--max-sideslip", type=float, default=6.0)
    ap.add_argument("--max-thrust", type=float, default=500.0)
    ap.add_argument("--neighbors", type=int, default=128)
    ap.add_argument(
        "--mach-grid",
        default="0,0.3,0.5,0.8,1.0,1.2,1.5,2,3,4,5,6,7,8,10,12,16,20,25",
    )
    ap.add_argument(
        "--aoa-grid",
        default="-5,0,5,10,15,20,25,30,35,40,45",
    )
    args = ap.parse_args()

    raw = []
    rejected = {
        "q": 0, "sideslip": 0, "thrust": 0, "airbrake": 0,
        "gear": 0, "coefficient": 0,
    }
    with open(args.input, newline="") as f:
        for row in csv.DictReader(f):
            q = number(row, "dynamic_pressure_pa", float("nan"))
            mass = number(row, "mass_kg", float("nan"))
            if not math.isfinite(q) or not math.isfinite(mass) or q < args.min_q or mass <= 0:
                rejected["q"] += 1
                continue
            if "sideslip_deg" in row and abs(number(row, "sideslip_deg")) > args.max_sideslip:
                rejected["sideslip"] += 1
                continue
            if "current_thrust_n" in row and number(row, "current_thrust_n") > args.max_thrust:
                rejected["thrust"] += 1
                continue
            if "airbrakes" in row and flag(row, "airbrakes"):
                rejected["airbrake"] += 1
                continue
            if "gear_down" in row and flag(row, "gear_down"):
                rejected["gear"] += 1
                continue

            try:
                mach = float(row["mach"])
                aoa = float(row["aoa_deg"])
                cl = float(row["lift_accel_mps2"]) * mass / (q * args.area)
                cd = float(row["drag_accel_mps2"]) * mass / (q * args.area)
            except (KeyError, TypeError, ValueError):
                rejected["coefficient"] += 1
                continue
            if not all(math.isfinite(v) for v in (mach, aoa, cl, cd)):
                rejected["coefficient"] += 1
                continue
            if abs(cl) > 2.5 or cd < 0.0 or cd > 2.5:
                rejected["coefficient"] += 1
                continue
            raw.append((mach, aoa, cl, cd))

    if not raw:
        raise SystemExit("no usable samples")

    mg = [float(x) for x in args.mach_grid.split(",")]
    ag = [float(x) for x in args.aoa_grid.split(",")]

    def estimate(m, a):
        ranked = []
        for sm, sa, cl, cd in raw:
            d2 = ((sm - m) / 1.5) ** 2 + ((sa - a) / 7.5) ** 2
            ranked.append((sm, sa, cl, cd, d2))
        ranked.sort(key=lambda x: x[4])
        nearest = ranked[: min(args.neighbors, len(ranked))]
        return robust_weighted(nearest, 2), robust_weighted(nearest, 3)

    with open(args.output, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["mach", "aoa_deg", "cl", "cd"])
        for m in mg:
            for a in ag:
                cl, cd = estimate(m, a)
                w.writerow([m, a, f"{cl:.9f}", f"{cd:.9f}"])

    print(
        f"wrote {len(mg)*len(ag)} cells from {len(raw)} clean telemetry samples "
        f"to {args.output}; rejected={rejected}"
    )


if __name__ == "__main__":
    main()
