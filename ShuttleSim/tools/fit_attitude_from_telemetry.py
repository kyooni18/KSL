#!/usr/bin/env python3
"""Fit ShuttleSim pitch/roll response from recorded KSP target-vs-actual telemetry.

The fit is intentionally split:
  * rate/acceleration limits come from measured KSP motion;
  * wn/zeta are fitted by forward-replaying high-q training segments where
    aerodynamic authority is saturated;
  * low-q full-authority thresholds are fitted from the same command-vs-actual
    telemetry using the exact q-scaling law implemented by ShuttleSim.

Required columns:
  source_log,time_s,dynamic_pressure_pa,aoa_deg,bank_deg,
  pitch_rate_deg_s,roll_rate_deg_s,target_aoa_deg,target_roll_deg
"""
import argparse
import csv
import math
import statistics
from collections import defaultdict


def wrap_deg(x):
    return (x + 180.0) % 360.0 - 180.0


def clamp(x, lo, hi):
    return max(lo, min(hi, x))


def percentile(values, p):
    xs = sorted(x for x in values if math.isfinite(x))
    if not xs:
        return 0.0
    k = (len(xs) - 1) * p
    i = int(k)
    f = k - i
    return xs[i] * (1.0 - f) + xs[min(i + 1, len(xs) - 1)] * f


def load_flights(path, thin_s):
    groups = defaultdict(dict)
    with open(path, newline="") as f:
        for r in csv.DictReader(f):
            try:
                row = {
                    "t": float(r["time_s"]),
                    "q": float(r["dynamic_pressure_pa"]),
                    "aoa": float(r["aoa_deg"]),
                    "bank": float(r["bank_deg"]),
                    "pitch_rate": float(r["pitch_rate_deg_s"]),
                    "roll_rate": float(r["roll_rate_deg_s"]),
                    "target_aoa": float(r["target_aoa_deg"]),
                    "target_bank": float(r["target_roll_deg"]),
                }
            except (KeyError, TypeError, ValueError):
                continue
            if not all(math.isfinite(v) for v in row.values()):
                continue
            groups[r.get("source_log", "unknown")][round(row["t"], 6)] = row

    flights = []
    for source, keyed in groups.items():
        raw = sorted(keyed.values(), key=lambda r: r["t"])
        thinned = []
        last_t = -1e30
        for row in raw:
            if not thinned or row["t"] - last_t >= thin_s - 1e-9:
                thinned.append(row)
                last_t = row["t"]
        if len(thinned) >= 3:
            flights.append((source, thinned))
    return flights


def measured_limits(flights):
    pitch_rates = []
    roll_rates = []
    pitch_accels = []
    roll_accels = []
    for _, rows in flights:
        for r in rows:
            pitch_rates.append(abs(r["pitch_rate"]))
            roll_rates.append(abs(r["roll_rate"]))
        for a, b in zip(rows, rows[1:]):
            dt = b["t"] - a["t"]
            if 0.015 <= dt <= 0.3:
                pitch_accels.append(abs((b["pitch_rate"] - a["pitch_rate"]) / dt))
                roll_accels.append(abs((b["roll_rate"] - a["roll_rate"]) / dt))
    return {
        "max_pitch_rate_deg_s": percentile(pitch_rates, 0.995),
        "max_roll_rate_deg_s": percentile(roll_rates, 0.995),
        "max_pitch_accel_deg_s2": percentile(pitch_accels, 0.995),
        "max_roll_accel_deg_s2": percentile(roll_accels, 0.995),
    }


def axis_delta(axis, target, actual):
    return wrap_deg(target - actual) if axis == "roll" else target - actual


def simulate_score(flights, axis, wn, zeta, vmax, amax, q_fit_min, physics_dt, q_full=None):
    value_key = "bank" if axis == "roll" else "aoa"
    rate_key = "roll_rate" if axis == "roll" else "pitch_rate"
    target_key = "target_bank" if axis == "roll" else "target_aoa"
    errors = []
    transients = []

    for _, rows in flights:
        start = next((i for i, r in enumerate(rows) if r["q"] >= q_fit_min), None)
        if start is None or start + 1 >= len(rows):
            continue
        rows = rows[start:]
        x = rows[0][value_key]
        v = rows[0][rate_key]
        prev = rows[0]

        for cur in rows[1:]:
            dt = cur["t"] - prev["t"]
            if not (0.0 < dt <= 0.3):
                x = cur[value_key]
                v = cur[rate_key]
                prev = cur
                continue

            elapsed = 0.0
            while elapsed < dt - 1e-12:
                h = min(physics_dt, dt - elapsed)
                # Guidance commands are zero-order held between recorded command
                # updates.  Interpolating target attitude would invent commands
                # that never existed and systematically make reversals easier.
                target = prev[target_key]
                if axis == "roll":
                    target = wrap_deg(target)

                # q is an observed plant state rather than a command.  When a log
                # interval spans more than one physics tick, interpolate q only so
                # the fitted authority follows the measured atmosphere/airspeed
                # evolution without fabricating intermediate control demand.
                u = (elapsed + 0.5 * h) / dt
                if q_full is None:
                    authority = 1.0
                else:
                    q = prev["q"] + u * (cur["q"] - prev["q"])
                    authority = clamp(q / max(q_full, 1e-9), 0.0, 1.0)

                err = axis_delta(axis, target, x)
                if authority <= 1e-12:
                    v = 0.0
                else:
                    root = math.sqrt(authority)
                    effective_wn = wn * root
                    effective_vmax = vmax * root
                    effective_amax = amax * authority
                    accel = effective_wn * effective_wn * err - 2.0 * zeta * effective_wn * v
                    accel = clamp(accel, -effective_amax, effective_amax)
                    v = clamp(v + accel * h, -effective_vmax, effective_vmax)
                    x += v * h
                    if axis == "roll":
                        x = wrap_deg(x)
                elapsed += h

            error = abs(axis_delta(axis, x, cur[value_key]))
            errors.append(error)
            target_error = abs(axis_delta(axis, cur[target_key], cur[value_key]))
            if target_error >= (5.0 if axis == "roll" else 1.0) or abs(cur[rate_key]) >= (3.0 if axis == "roll" else 1.0):
                transients.append(error)
            prev = cur

    if not errors:
        return (1e30,) * 5
    es = sorted(errors)
    ts = sorted(transients) if transients else [0.0]
    mean = statistics.mean(errors)
    p95 = es[int(0.95 * (len(es) - 1))]
    p99 = es[int(0.99 * (len(es) - 1))]
    transient_mean = statistics.mean(transients) if transients else 0.0
    transient_p95 = ts[int(0.95 * (len(ts) - 1))]
    objective = mean + 0.35 * transient_mean + 0.15 * transient_p95
    return objective, mean, p95, p99, transient_p95


def fit_axis(flights, axis, vmax, amax, q_fit_min, physics_dt):
    if axis == "roll":
        coarse_wn = [0.8, 1.0, 1.2, 1.4, 1.6, 1.8, 2.1]
        coarse_zeta = [0.20, 0.35, 0.50, 0.65, 0.85, 1.05]
    else:
        coarse_wn = [0.8, 1.0, 1.2, 1.4, 1.6, 1.9, 2.2]
        coarse_zeta = [0.50, 0.75, 1.00, 1.25, 1.50]

    ranked = []
    for wn in coarse_wn:
        for zeta in coarse_zeta:
            score = simulate_score(flights, axis, wn, zeta, vmax, amax, q_fit_min, physics_dt)
            ranked.append((score[0], wn, zeta, score))
    ranked.sort()
    _, best_wn, best_zeta, _ = ranked[0]

    refined = []
    for di in range(-4, 5):
        wn = max(0.1, best_wn + 0.05 * di)
        for dj in range(-5, 6):
            zeta = max(0.05, best_zeta + 0.05 * dj)
            score = simulate_score(flights, axis, wn, zeta, vmax, amax, q_fit_min, physics_dt)
            refined.append((score[0], wn, zeta, score))
    refined.sort()
    return refined[0], ranked[:5]


def fit_full_authority_q(flights, axis, wn, zeta, vmax, amax, q_fit_min, physics_dt):
    # Log-spaced enough to resolve the low-q onset without spending fit time on
    # regions where the response is already saturated.
    coarse_q = [1.0, 2.0, 3.0, 5.0, 7.5, 10.0, 15.0, 20.0, 30.0, 45.0,
                70.0, 100.0, 150.0, 220.0, 330.0, 500.0]
    ranked = []
    for q_full in coarse_q:
        score = simulate_score(
            flights, axis, wn, zeta, vmax, amax, q_fit_min, physics_dt, q_full=q_full
        )
        ranked.append((score[0], q_full, score))
    ranked.sort()
    best_q = ranked[0][1]

    refined_q = sorted({
        max(0.25, best_q * factor)
        for factor in (0.50, 0.65, 0.80, 0.90, 1.00, 1.10, 1.25, 1.50, 1.75)
    })
    refined = []
    for q_full in refined_q:
        score = simulate_score(
            flights, axis, wn, zeta, vmax, amax, q_fit_min, physics_dt, q_full=q_full
        )
        refined.append((score[0], q_full, score))
    refined.sort()
    return refined[0], ranked[:5]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("input")
    ap.add_argument("output")
    ap.add_argument("--thin-s", type=float, default=0.02)
    ap.add_argument("--fit-min-q", type=float, default=500.0,
                    help="minimum q for saturated-response wn/zeta fitting")
    ap.add_argument("--authority-fit-min-q", type=float, default=1.0,
                    help="minimum q for fitting the low-q authority transition")
    ap.add_argument("--physics-dt", type=float, default=0.02,
                    help="fixed plant integration step; use 0.02 for KSP parity")
    ap.add_argument("--pitch-full-authority-q-pa", type=float, default=None,
                    help="diagnostic override; default is fit from telemetry")
    ap.add_argument("--roll-full-authority-q-pa", type=float, default=None,
                    help="diagnostic override; default is fit from telemetry")
    args = ap.parse_args()
    if not (0.0 < args.physics_dt <= 0.1):
        raise SystemExit("--physics-dt must be in (0, 0.1]")

    flights = load_flights(args.input, args.thin_s)
    if not flights:
        raise SystemExit("no usable flights")
    limits = measured_limits(flights)

    pitch = fit_axis(
        flights, "pitch",
        limits["max_pitch_rate_deg_s"],
        limits["max_pitch_accel_deg_s2"],
        args.fit_min_q,
        args.physics_dt,
    )
    roll = fit_axis(
        flights, "roll",
        limits["max_roll_rate_deg_s"],
        limits["max_roll_accel_deg_s2"],
        args.fit_min_q,
        args.physics_dt,
    )

    _, pitch_wn, pitch_zeta, pitch_score = pitch[0]
    _, roll_wn, roll_zeta, roll_score = roll[0]

    if args.pitch_full_authority_q_pa is None:
        pitch_authority = fit_full_authority_q(
            flights, "pitch", pitch_wn, pitch_zeta,
            limits["max_pitch_rate_deg_s"], limits["max_pitch_accel_deg_s2"],
            args.authority_fit_min_q, args.physics_dt,
        )
        pitch_q = pitch_authority[0][1]
    else:
        pitch_q = args.pitch_full_authority_q_pa
        pitch_authority = (
            (simulate_score(
                flights, "pitch", pitch_wn, pitch_zeta,
                limits["max_pitch_rate_deg_s"], limits["max_pitch_accel_deg_s2"],
                args.authority_fit_min_q, args.physics_dt, q_full=pitch_q
            )[0], pitch_q, None),
            [],
        )

    if args.roll_full_authority_q_pa is None:
        roll_authority = fit_full_authority_q(
            flights, "roll", roll_wn, roll_zeta,
            limits["max_roll_rate_deg_s"], limits["max_roll_accel_deg_s2"],
            args.authority_fit_min_q, args.physics_dt,
        )
        roll_q = roll_authority[0][1]
    else:
        roll_q = args.roll_full_authority_q_pa
        roll_authority = (
            (simulate_score(
                flights, "roll", roll_wn, roll_zeta,
                limits["max_roll_rate_deg_s"], limits["max_roll_accel_deg_s2"],
                args.authority_fit_min_q, args.physics_dt, q_full=roll_q
            )[0], roll_q, None),
            [],
        )

    with open(args.output, "w") as f:
        f.write("# KSP training-telemetry fitted closed-loop response, authority, and measured limits.\n")
        f.write(f"pitch_wn={pitch_wn:.6f}\n")
        f.write(f"pitch_zeta={pitch_zeta:.6f}\n")
        f.write(f"roll_wn={roll_wn:.6f}\n")
        f.write(f"roll_zeta={roll_zeta:.6f}\n")
        for key in (
            "max_pitch_rate_deg_s", "max_roll_rate_deg_s",
            "max_pitch_accel_deg_s2", "max_roll_accel_deg_s2",
        ):
            f.write(f"{key}={limits[key]:.6f}\n")
        f.write(f"pitch_full_authority_q_pa={pitch_q:.6f}\n")
        f.write(f"roll_full_authority_q_pa={roll_q:.6f}\n")

    print(
        f"flights={len(flights)} thin_s={args.thin_s} physics_dt={args.physics_dt} "
        f"fit_min_q={args.fit_min_q} authority_fit_min_q={args.authority_fit_min_q}"
    )
    print("pitch", {"wn": pitch_wn, "zeta": pitch_zeta, "score": pitch_score,
                    "coarse": pitch[1], "authority": pitch_authority})
    print("roll", {"wn": roll_wn, "zeta": roll_zeta, "score": roll_score,
                   "coarse": roll[1], "authority": roll_authority})
    print("limits", limits)


if __name__ == "__main__":
    main()
