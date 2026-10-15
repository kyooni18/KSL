#!/usr/bin/env python3
"""Replace STS aero table cells with coefficients measured in live flight.

Every (Mach row, alpha node) of the base table is a cell.  Steady flying
samples (unpowered, gear up, airbrakes stowed, small sideslip and pitch rate, enough
dynamic pressure) are assigned to the nearest cell when they lie within a
fraction of the local grid spacing.  A cell's measured CL/CD is the median
of per-flight medians, so a long log cannot outvote other flights, and the
cell is replaced only with enough samples and flights.  Unmeasured alpha
nodes in a measured Mach row take the measured/table ratio of the nearest
measured node; rows with no measured cell keep the base table.  Grids come
from the base table.  Output is provisional, not certified.
"""
import argparse, bisect, collections, csv, hashlib, json, os, statistics


def merge(a, b):
    for k, v in b.items():
        if isinstance(v, dict):
            if not isinstance(a.get(k), dict):
                a[k] = {}
            merge(a[k], v)
        else:
            a[k] = v


def read_table(path):
    rows = collections.defaultdict(dict)
    with open(path) as f:
        lines = [l for l in f if not l.startswith("#")]
    for r in csv.DictReader(lines):
        rows[float(r["mach"])][float(r["alpha_deg"])] = (float(r["cl"]), float(r["cd"]))
    return rows


def nearest(grid, x, window):
    """Nearest grid node, or None when x is outside `window` of the local spacing."""
    i = bisect.bisect_left(grid, x)
    best = min((j for j in (i - 1, i) if 0 <= j < len(grid)), key=lambda j: abs(grid[j] - x))
    lo = grid[best] - grid[best - 1] if best > 0 else grid[1] - grid[0]
    hi = grid[best + 1] - grid[best] if best + 1 < len(grid) else lo
    spacing = lo if x < grid[best] else hi
    return grid[best] if abs(x - grid[best]) <= window * spacing else None


def samples(path, a):
    st = {}
    for line in open(path):
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
        ae, at, s = st.get("aero") or {}, st.get("attitude") or {}, st.get("state") or {}
        # Thrust (ascent stack, jet assist) would be read as negative drag.
        if (st.get("vehicle") or {}).get("currentThrust"):
            continue
        q = ae.get("dynamicPressure") or 0.0
        if s.get("vesselSituation") != "flying" or s.get("gear") or s.get("airbrakes") or q < a.q_min:
            continue
        if abs(at.get("sideslip") or 0) > a.max_sideslip or abs(at.get("bodyPitchRate") or 0) > a.max_pitch_rate:
            continue
        m, al, lf, df = ae.get("mach"), at.get("angleOfAttack"), ae.get("liftForce"), ae.get("dragForce")
        if None in (m, al, lf, df):
            continue
        yield m, al, lf / (q * a.area), df / (q * a.area)


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--base", required=True)
    ap.add_argument("--logs", nargs="+", required=True)
    ap.add_argument("--output", required=True)
    ap.add_argument("--report", required=True)
    ap.add_argument("--area", type=float, default=250.0, help="reference area of the base table (m^2)")
    ap.add_argument("--q-min", type=float, default=300.0, help="minimum dynamic pressure (Pa)")
    ap.add_argument("--max-sideslip", type=float, default=2.0, help="deg")
    ap.add_argument("--max-pitch-rate", type=float, default=3.0, help="deg/s")
    ap.add_argument("--window", type=float, default=0.25, help="assignment radius as a fraction of grid spacing")
    ap.add_argument("--min-samples", type=int, default=200, help="per cell")
    ap.add_argument("--min-flights", type=int, default=2, help="per cell")
    a = ap.parse_args()

    base = read_table(a.base)
    machs = sorted(base)
    cells = collections.defaultdict(lambda: collections.defaultdict(list))
    for path in a.logs:
        for m, al, cl, cd in samples(path, a):
            mr = nearest(machs, m, a.window)
            if mr is None:
                continue
            ar = nearest(sorted(base[mr]), al, a.window)
            if ar is not None:
                cells[(mr, ar)][path].append((cl, cd))

    measured, report = {}, {}
    for (mr, ar), by_flight in cells.items():
        n = sum(len(v) for v in by_flight.values())
        if n < a.min_samples or len(by_flight) < a.min_flights:
            continue
        cl = statistics.median(statistics.median(x[0] for x in v) for v in by_flight.values())
        cd = statistics.median(statistics.median(x[1] for x in v) for v in by_flight.values())
        bcl, bcd = base[mr][ar]
        measured[(mr, ar)] = (cl, cd)
        report[f"M{mr:.3f} a{ar:g}"] = dict(n=n, flights=len(by_flight), cl=cl, cd=cd,
                                           cl_ratio=cl / bcl if bcl else None, cd_ratio=cd / bcd)

    out = []
    for mr in machs:
        alphas = sorted(base[mr])
        known = [ar for ar in alphas if (mr, ar) in measured]
        for ar in alphas:
            bcl, bcd = base[mr][ar]
            if (mr, ar) in measured:
                cl, cd = measured[(mr, ar)]
            elif known:
                k = min(known, key=lambda x: abs(x - ar))
                kcl, kcd = base[mr][k]
                mcl, mcd = measured[(mr, k)]
                cl = bcl * (mcl / kcl if kcl else 1.0)
                cd = bcd * mcd / kcd
            else:
                cl, cd = bcl, bcd
            out.append((mr, ar, cl, cd))

    with open(a.output, "w") as f:
        f.write(f"# PROVISIONAL STS live aero: {len(measured)} cells measured from {len(a.logs)} logs "
                f"(q>={a.q_min:g} Pa, |beta|<={a.max_sideslip:g} deg, unpowered, gear up, airbrakes stowed); "
                f"other cells from {os.path.basename(a.base)} scaled by the nearest measured cell in their Mach row. "
                f"Not certified. Area {a.area:g} m2 is normalization only. Logs and per-cell evidence: {os.path.basename(a.report)}\n")
        f.write("mach,alpha_deg,cl,cd\n")
        for mr, ar, cl, cd in out:
            f.write(f"{mr:.8f},{ar:g},{cl:.8f},{cd:.8f}\n")
    logs = [dict(path=os.path.relpath(p), sha256=hashlib.sha256(open(p, "rb").read()).hexdigest()) for p in a.logs]
    json.dump(dict(status="provisional; not certified", base=os.path.basename(a.base), logs=logs,
                   settings={k: v for k, v in vars(a).items() if k not in ("logs", "base", "output", "report")},
                   cells=report), open(a.report, "w"), indent=1)
    for k, v in sorted(report.items(), key=lambda kv: (float(kv[0].split()[0][1:]), float(kv[0].split()[1][1:]))):
        print(f"{k:14s} n={v['n']:6d} flights={v['flights']:2d} CL x{v['cl_ratio'] or 0:.3f} CD x{v['cd_ratio']:.3f}")


if __name__ == "__main__":
    main()
