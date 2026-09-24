#!/usr/bin/env python3
"""Build a stock-Kerbin atmosphere table using KSP telemetry for temperature.

Stock KSP pressure is an altitude-only AnimationCurve, so fitting pressure or
density independently from rounded telemetry bins only adds error. This tool
evaluates Kerbin's stock pressure curve exactly, derives density from the ideal
gas relation used by KSP, and derives speed of sound from the stock molar mass
and adiabatic index. Telemetry is used only to recover the temperature profile
actually seen by the calibrated flights (including KSP's environmental thermal
modifiers). Below the observed telemetry floor, only temperature is linearly
extrapolated; pressure remains the stock curve all the way to sea level.
"""
import argparse
import csv
import math
import statistics

KERBIN_MOLAR_MASS_KG_MOL = 0.0289644002914429
KERBIN_ADIABATIC_INDEX = 1.39999997615814
UNIVERSAL_GAS_CONSTANT = 8.31446261815324

# altitude_m, pressure_kPa, in_tangent_kPa_per_m, out_tangent_kPa_per_m
KERBIN_PRESSURE_CURVE = [
    (0.0, 101.325, 0.0, -0.01501631),
    (1241.025, 84.02916, -0.01289846, -0.01289826),
    (2439.593, 69.68138, -0.01107876, -0.01107859),
    (3597.11, 57.78001, -0.009515483, -0.009515338),
    (4714.942, 47.90862, -0.00817254, -0.008172415),
    (5794.409, 39.72148, -0.00701892, -0.007018813),
    (6836.791, 32.93169, -0.006027969, -0.006027877),
    (7843.328, 27.30109, -0.005176778, -0.0051767),
    (8815.22, 22.63206, -0.004445662, -0.004445578),
    (10786.42, 15.3684, -0.003016528, -0.00301646),
    (12101.4, 11.87313, -0.002329273, -0.00232922),
    (13417.05, 9.172798, -0.001798594, -0.001798554),
    (16678.47, 4.842261, -0.0009448537, -0.0009448319),
    (21143.1, 2.050097, -0.0003894095, -0.0003894005),
    (26977.92, 0.6905929, -0.0001252565, -0.0001252534),
    (33593.82, 0.2201734, -3.626878e-05, -3.626788e-05),
    (42081.87, 0.05768469, -9.063159e-06, -9.062975e-06),
    (49312.13, 0.01753794, -3.029397e-06, -3.029335e-06),
    (56669.95, 0.004591824, -8.827175e-07, -8.826996e-07),
    (62300.84, 0.001497072, -3.077091e-07, -3.077031e-07),
    (70000.0, 0.0, 0.0, 0.0),
]


def animation_curve(curve, x):
    if x <= curve[0][0]:
        return curve[0][1]
    if x >= curve[-1][0]:
        return curve[-1][1]
    for i in range(1, len(curve)):
        if x <= curve[i][0]:
            x0, y0, _, m0 = curve[i - 1]
            x1, y1, m1, _ = curve[i]
            dx = x1 - x0
            t = (x - x0) / dx
            h00 = 2.0 * t**3 - 3.0 * t**2 + 1.0
            h10 = t**3 - 2.0 * t**2 + t
            h01 = -2.0 * t**3 + 3.0 * t**2
            h11 = t**3 - t**2
            return h00 * y0 + h10 * dx * m0 + h01 * y1 + h11 * dx * m1
    raise AssertionError("unreachable")


def linear_fit(points):
    n = len(points)
    sx = sum(x for x, _ in points)
    sy = sum(y for _, y in points)
    sxx = sum(x * x for x, _ in points)
    sxy = sum(x * y for x, y in points)
    d = n * sxx - sx * sx
    if n < 2 or abs(d) < 1e-12:
        raise ValueError("insufficient span for extrapolation")
    return (sy * sxx - sx * sxy) / d, (n * sxy - sx * sy) / d


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("input")
    ap.add_argument("output")
    ap.add_argument("--bin-m", type=float, default=100.0,
                    help="output altitude spacing and temperature aggregation bin")
    ap.add_argument("--atmosphere-top-m", type=float, default=70000.0)
    ap.add_argument("--low-extrapolation-span-m", type=float, default=1000.0)
    ap.add_argument("--max-rows", type=int, default=2048)
    a = ap.parse_args()
    if a.bin_m <= 0.0 or a.atmosphere_top_m <= 0.0:
        raise SystemExit("invalid atmosphere fit bounds")
    if abs(a.atmosphere_top_m - 70000.0) > 1e-6:
        raise SystemExit("stock Kerbin pressure curve requires atmosphere-top-m=70000")

    bins = {}
    seen = set()
    with open(a.input, newline="") as f:
        for r in csv.DictReader(f):
            try:
                h = float(r["altitude_m"])
                temp = float(r["temperature_k"])
                t = round(float(r.get("time_s", "nan")), 6)
                source = r.get("source_log", "")
            except (KeyError, TypeError, ValueError):
                continue
            if not math.isfinite(h) or not math.isfinite(temp) or not (0.0 < h < a.atmosphere_top_m) or temp <= 0.0:
                continue
            key = (source, t)
            if math.isfinite(t) and key in seen:
                continue
            if math.isfinite(t):
                seen.add(key)
            bins.setdefault(round(h / a.bin_m) * a.bin_m, []).append((h, temp))

    profile = []
    for samples in bins.values():
        hs, ts = zip(*samples)
        profile.append((statistics.median(hs), statistics.median(ts)))
    profile.sort()
    if len(profile) < 3:
        raise SystemExit("not enough temperature samples")

    floor = profile[0][0]
    low = [x for x in profile if x[0] <= floor + a.low_extrapolation_span_m]
    low_c0, low_c1 = linear_fit(low)

    def temperature_at(h):
        if h <= profile[0][0]:
            return max(100.0, min(400.0, low_c0 + low_c1 * h))
        if h >= profile[-1][0]:
            return profile[-1][1]
        lo = 0
        hi = len(profile) - 1
        while hi - lo > 1:
            mid = (lo + hi) // 2
            if profile[mid][0] <= h:
                lo = mid
            else:
                hi = mid
        h0, t0 = profile[lo]
        h1, t1 = profile[hi]
        u = (h - h0) / (h1 - h0)
        return t0 + (t1 - t0) * u

    rows = []
    count = int(round(a.atmosphere_top_m / a.bin_m))
    for i in range(count + 1):
        h = min(a.atmosphere_top_m, i * a.bin_m)
        temp = temperature_at(h)
        pressure_pa = max(0.0, animation_curve(KERBIN_PRESSURE_CURVE, h) * 1000.0)
        if pressure_pa > 0.0:
            density = pressure_pa * KERBIN_MOLAR_MASS_KG_MOL / (UNIVERSAL_GAS_CONSTANT * temp)
            sound = math.sqrt(KERBIN_ADIABATIC_INDEX * (UNIVERSAL_GAS_CONSTANT / KERBIN_MOLAR_MASS_KG_MOL) * temp)
        else:
            density = 0.0
            sound = math.sqrt(KERBIN_ADIABATIC_INDEX * (UNIVERSAL_GAS_CONSTANT / KERBIN_MOLAR_MASS_KG_MOL) * temp)
        rows.append((h, density, pressure_pa, temp, sound))
    if rows[-1][0] != a.atmosphere_top_m:
        h = a.atmosphere_top_m
        temp = temperature_at(h)
        rows.append((h, 0.0, 0.0, temp,
                     math.sqrt(KERBIN_ADIABATIC_INDEX * (UNIVERSAL_GAS_CONSTANT / KERBIN_MOLAR_MASS_KG_MOL) * temp)))
    if len(rows) > a.max_rows:
        raise SystemExit(f"atmosphere fit has {len(rows)} rows; exceeds runtime capacity {a.max_rows}")

    with open(a.output, "w", newline="") as f:
        f.write("# model=kerbin_stock_spatial\n")
        w = csv.writer(f)
        w.writerow(["altitude_m", "density_kg_m3", "pressure_pa", "temperature_k", "speed_of_sound_mps"])
        w.writerows(rows)
    print(f"wrote {len(rows)} stock-pressure rows covering 0..{a.atmosphere_top_m:g} m; temperature observed floor={floor:.1f} m")


if __name__ == "__main__":
    main()
