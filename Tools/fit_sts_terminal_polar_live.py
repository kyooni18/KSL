#!/usr/bin/env python3
"""Fit a live STS subsonic/transonic polar and merge it into the provisional table.

Per Mach bin (0.1..1.5): CL = cla*(alpha - a0) over alpha 0..18 deg and
CD = cd0 + k*CL^2, least squares on steady flying samples (q > 800 Pa,
|beta| < 2 deg, |pitch rate| < 3 deg/s) from the given live vehicle logs.
Outside the observed alpha span the base table is scaled by the fit/table
ratio at the nearest observed edge.  Mach rows above 1.5 come from the base
table unchanged.  Output is a provisional model, not a certified table.
"""
import argparse, collections, glob, hashlib, json, math, os, statistics, sys
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from sts_terminal_bench import Aero

ALPHAS = [-10, -5, -2, 0, 2, 4, 6, 8, 10, 12, 14, 16, 18, 20, 22, 25, 28, 30, 35, 40, 45]


def merge(a, b):
    for k, v in b.items():
        if isinstance(v, dict) and isinstance(a.get(k), dict):
            merge(a[k], v)
        else:
            a[k] = v


def samples(files, mach_min, mach_max, alpha_max):
    for fn in files:
        st = {}
        for line in open(fn):
            try:
                d = json.loads(line)
            except ValueError:
                continue
            rt = d.get("recordType")
            if rt in ("vehicleKeyframe", "keyframe"):
                st = {}
                merge(st, d.get("fields", d))
            elif rt == "vehicleDelta":
                merge(st, d.get("fields", {}))
            else:
                continue
            try:
                ae, at, s = st["aero"], st["attitude"], st["state"]
            except KeyError:
                continue
            q = ae.get("dynamicPressure") or 0.0
            if s.get("vesselSituation") != "flying" or q < 800 or st["state"].get("gear"):
                continue
            if abs(at.get("sideslip") or 0) > 2 or abs(at.get("bodyPitchRate") or 0) > 3:
                continue
            m, a = ae["mach"], at["angleOfAttack"]
            if not (mach_min < m < mach_max and -1 <= a <= alpha_max):
                continue
            yield m, a, ae["liftForce"] / (q * 250.0), ae["dragForce"] / (q * 250.0)


def lsq(xs, ys):
    n = len(xs)
    mx, my = sum(xs) / n, sum(ys) / n
    sxx = sum((x - mx) ** 2 for x in xs)
    if sxx <= 0:
        return None
    b = sum((x - mx) * (y - my) for x, y in zip(xs, ys)) / sxx
    return my - b * mx, b


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--base", default=os.path.join(os.path.dirname(__file__), "..", "ShuttleSim", "reference-model", "sts_aero_live20261009.csv"))
    ap.add_argument("--logs", nargs="+", required=True)
    ap.add_argument("--output", required=True)
    ap.add_argument("--report", required=True)
    # Defaults are the original subsonic/transonic fit; entry fits widen these.
    ap.add_argument("--mach-min", type=float, default=0.05)
    ap.add_argument("--mach-max", type=float, default=1.55)
    ap.add_argument("--alpha-max", type=float, default=19.0)
    ap.add_argument("--mach-bin", type=float, default=0.1)
    ap.add_argument("--min-alpha-span", type=float, default=6.0)
    args = ap.parse_args()
    base = Aero(args.base)
    files = sorted(set(f for g in args.logs for f in glob.glob(g)))
    bins = collections.defaultdict(list)
    for m, a, cl, cd in samples(files, args.mach_min, args.mach_max, args.alpha_max):
        bins[round(m / args.mach_bin) * args.mach_bin].append((a, cl, cd))
    fits = {}
    for mb in sorted(bins):
        pts = bins[mb]
        alphas = [p[0] for p in pts]
        if len(pts) < 60:
            continue
        if max(alphas) - min(alphas) < args.min_alpha_span:
            # Too narrow for a slope (entry holds one incidence): keep the base
            # polar's shape and scale it by the measured/table median ratio.
            rl = statistics.median(cl / base.coef(mb, a)[0] for a, cl, cd in pts if abs(base.coef(mb, a)[0]) > 1e-3)
            rd = statistics.median(cd / base.coef(mb, a)[1] for a, cl, cd in pts if base.coef(mb, a)[1] > 1e-4)
            fits[mb] = dict(ratio_lift=rl, ratio_drag=rd, n=len(pts), amin=min(alphas), amax=max(alphas))
            continue
        # weight AoA cells equally: median per 1-degree cell
        cells = collections.defaultdict(list)
        for a, cl, cd in pts:
            cells[round(a)].append((cl, cd))
        ca = [(a, statistics.median(v[0] for v in c), statistics.median(v[1] for v in c))
              for a, c in cells.items() if len(c) >= 3]
        if len(ca) < 3:
            continue
        lf = lsq([c[0] for c in ca], [c[1] for c in ca])
        df = lsq([c[1] ** 2 for c in ca], [c[2] for c in ca])
        if not lf or not df or lf[1] <= 0 or df[0] <= 0 or df[1] < 0:
            continue
        fits[mb] = dict(cl0=lf[0], cla=lf[1], cd0=df[0], k=df[1], n=len(pts),
                        amin=min(c[0] for c in ca), amax=max(c[0] for c in ca), cells=len(ca))
    rows = []
    fit_machs = sorted(fits)
    for mb in fit_machs:
        f = fits[mb]
        if "ratio_drag" in f:
            for a, cl, cd in base.tab[min(base.mach, key=lambda m: abs(m - mb))]:
                rows.append((mb, a, cl * f["ratio_lift"], cd * f["ratio_drag"]))
            continue
        for a in ALPHAS:
            ae = max(f["amin"], min(f["amax"], abs(a)))
            clf = f["cl0"] + f["cla"] * ae
            cdf = f["cd0"] + f["k"] * clf * clf
            bcl, bcd = base.coef(mb, ae)
            rl = clf / bcl if abs(bcl) > 1e-3 else 1.0
            rd = cdf / bcd if bcd > 1e-4 else 1.0
            if f["amin"] <= abs(a) <= f["amax"]:
                cl, cd = clf, cdf
            else:
                tcl, tcd = base.coef(mb, abs(a))
                cl, cd = tcl * rl, tcd * rd
            if a < 0:
                cl = -cl
            rows.append((mb, a, cl, cd))
    for m in base.mach:
        if m > (fit_machs[-1] if fit_machs else 0) + 0.05 or (m < (fit_machs[0] if fit_machs else 0) - 0.05):
            for a, cl, cd in base.tab[m]:
                rows.append((m, a, cl, cd))
    rows.sort()
    with open(args.output, "w") as out:
        out.write(f"# PROVISIONAL STS live polar: Mach {args.mach_min}-{args.mach_max} rows fitted to "
                  f"{', '.join(os.path.basename(f) for f in files)} (CL linear in alpha, CD parabolic in CL) "
                  f"inside observed alpha span; base table scaled outside; other Mach rows from "
                  f"{os.path.basename(args.base)}. Not certified. Area 250 m2 is normalization only.\n")
        out.write("mach,alpha_deg,cl,cd\n")
        for r in rows:
            out.write("%.8f,%g,%.8f,%.8f\n" % r)
    rep = dict(status="provisional; not certified", base=os.path.basename(args.base),
               logs=[dict(path=os.path.relpath(f, os.path.dirname(args.output)),
                          sha256=hashlib.sha256(open(f, "rb").read()).hexdigest()) for f in files],
               fits=fits)
    json.dump(rep, open(args.report, "w"), indent=1)
    for mb, f in fits.items():
        if "ratio_drag" in f:
            print(f"M{mb:.1f} n={f['n']:5d} a[{f['amin']:.0f},{f['amax']:.0f}] ratio CL x{f['ratio_lift']:.3f} CD x{f['ratio_drag']:.3f}")
            continue
        print(f"M{mb:.1f} n={f['n']:5d} a[{f['amin']},{f['amax']}] CL={f['cl0']:.3f}+{f['cla']:.4f}a  CD={f['cd0']:.4f}+{f['k']:.3f}CL^2  LDmax~{0.5/math.sqrt(f['cd0']*f['k']) if f['k']>0 else 0:.2f}")


if __name__ == "__main__":
    main()
