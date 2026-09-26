#!/usr/bin/env python3
"""Identify the STS-N terminal-regime aerodynamics from live KSP telemetry.

Model structure (per Mach knot, fitted by kernel-weighted least squares):

    CL(a, M) = CL0(M) + K(M) * sin(a) * cos(a)
    CD(a, M) = CD0(M) + KD(M) * sin(a)^2 + dCD_gear * gear + dCD_airbrake * airbrake

i.e. a normal-force coefficient K*sin(a) resolved into lift and induced drag,
plus an axial/parasite term.  Live KSP data from both the September 15-18
databook flights and the September 22-25 live landing attempts follow this
form with K/KD ~ 1 (the flat-plate signature of KSP lifting surfaces), so the
structure is a measured property, not an assumption.

Only the terminal regime (M <= --max-mach) is identified here.  Above it the
table blends to the existing KSP-fitted supersonic table (entry regime), which
this tool validates but does not refit.

Validation is by *flight*, not by sample: whole logs are held out (the
held-out databook flight plus every live log whose name hashes to bucket 0 of
4), so no neighbouring sample of a held-out flight informs the fit.

Outputs:
  --table   Mach x AoA CL/CD table readable by ShuttleSim aero_load_csv()
  --report  JSON: data provenance, per-knot parameters, held-out residuals,
            and an uncertainty band (held-out 5-95 % ratio measured/predicted)
"""
from __future__ import annotations

import argparse
import csv
import glob
import hashlib
import json
import math
import os
import zlib
from multiprocessing import Pool

AREA = 250.0
MACH_KNOTS = [0.0, 0.05, 0.10, 0.15, 0.20, 0.25, 0.30, 0.35, 0.40, 0.45, 0.50,
              0.60, 0.70, 0.80, 0.90]
ALPHA_KNOTS = [-10, -5, -2, 0, 2, 4, 6, 8, 10, 12, 14, 16, 18, 20, 22, 25, 28,
               30, 35, 40, 45]


def _f(x):
    try:
        v = float(x)
    except (TypeError, ValueError):
        return None
    return v if math.isfinite(v) else None


# ---------------------------------------------------------------- data ----

def _merge(dst, src):
    for k, v in src.items():
        if isinstance(v, dict) and isinstance(dst.get(k), dict):
            _merge(dst[k], v)
        else:
            dst[k] = v


def _g(d, *path):
    for k in path:
        if not isinstance(d, dict):
            return None
        d = d.get(k)
    return d


def _live_log(path):
    """Decode one live (non-ShuttleSim) vehicle log into aero samples."""
    out = []
    with open(path, encoding="utf-8") as h:
        try:
            first = json.loads(h.readline())
        except json.JSONDecodeError:
            return out
        transport = str(_g(first, "runtime", "transport"))
        if "shuttlesim" in transport or transport == "None":
            return out
        state = {}
        last_ut = None
        for line in h:
            try:
                r = json.loads(line)
            except json.JSONDecodeError:
                continue
            t = r.get("recordType")
            fields = r.get("fields")
            if not isinstance(fields, dict):
                continue
            if t == "vehicleKeyframe":
                state = json.loads(json.dumps(fields))
            elif t == "vehicleDelta":
                _merge(state, fields)
            else:
                continue
            ut = _f(r.get("ut"))
            if ut is None or ut == last_ut:
                continue
            last_ut = ut
            if _g(state, "state", "vesselSituation") != "flying":
                continue
            ralt = _f(_g(state, "position", "radarAltitude"))
            if ralt is None or ralt < 8.0:
                continue
            out.append(dict(
                log=os.path.basename(path), source="live",
                mach=_f(_g(state, "aero", "mach")),
                q=_f(_g(state, "aero", "dynamicPressure")),
                aoa=_f(_g(state, "attitude", "angleOfAttack")),
                beta=_f(_g(state, "attitude", "sideslip")),
                lift=_f(_g(state, "aero", "liftForce")),
                drag=_f(_g(state, "aero", "dragForce")),
                thrust=_f(_g(state, "vehicle", "currentThrust")),
                gear=1 if _g(state, "state", "gear") else 0,
                airbrake=1 if _g(state, "state", "airbrakes") else 0))
    return out


def _normalized_csv(path):
    out = []
    last = None
    with open(path, encoding="utf-8") as h:
        for row in csv.DictReader(h):
            key = (row.get("source_log"), row.get("ut"))
            if key == last:
                continue
            last = key
            alt = _f(row.get("altitude_m"))
            if alt is None or alt < 120.0:
                continue
            out.append(dict(
                log=row.get("source_log"), source=os.path.basename(path),
                mach=_f(row.get("mach")), q=_f(row.get("dynamic_pressure_pa")),
                aoa=_f(row.get("aoa_deg")), beta=_f(row.get("sideslip_deg")),
                lift=_f(row.get("lift_force_n")), drag=_f(row.get("drag_force_n")),
                thrust=_f(row.get("current_thrust_n")),
                gear=int(_f(row.get("gear_down")) or 0),
                airbrake=int(_f(row.get("airbrakes")) or 0)))
    return out


def load(args):
    samples = []
    for p in args.normalized:
        samples.extend(_normalized_csv(p))
    logs = sorted(glob.glob(os.path.join(args.live_logs, "*vehicle.jsonl")))
    with Pool(args.jobs) as pool:
        for rows in pool.imap_unordered(_live_log, logs, chunksize=8):
            samples.extend(rows)
    seen = set()
    clean = []
    for s in samples:
        if None in (s["mach"], s["q"], s["aoa"], s["lift"], s["drag"]):
            continue
        if s["q"] < args.min_q or s["lift"] == 0.0:
            continue
        if abs(s["beta"] or 0.0) > args.max_sideslip:
            continue
        if (s["thrust"] or 0.0) > 1.0:
            continue
        key = (s["log"], round(s["mach"], 5), round(s["aoa"], 4), round(s["q"], 1))
        if key in seen:
            continue
        seen.add(key)
        s["cl"] = s["lift"] / (s["q"] * AREA)
        s["cd"] = s["drag"] / (s["q"] * AREA)
        clean.append(s)
    return clean


def is_heldout(log, heldout_logs):
    if log in heldout_logs:
        return True
    return zlib.crc32(log.encode()) % 4 == 0


# ----------------------------------------------------------------- fit ----

def _wls(rows, basis, target, weight):
    """Weighted least squares for len(basis(row)) unknowns (normal equations)."""
    n = len(basis(rows[0]))
    ata = [[0.0] * n for _ in range(n)]
    atb = [0.0] * n
    for r in rows:
        w = weight(r)
        if w <= 0:
            continue
        x = basis(r)
        y = target(r)
        for i in range(n):
            atb[i] += w * x[i] * y
            for j in range(n):
                ata[i][j] += w * x[i] * x[j]
    # Gaussian elimination with partial pivoting.
    m = [ata[i] + [atb[i]] for i in range(n)]
    for c in range(n):
        p = max(range(c, n), key=lambda i: abs(m[i][c]))
        if abs(m[p][c]) < 1e-12:
            return None
        m[c], m[p] = m[p], m[c]
        for i in range(n):
            if i != c:
                f = m[i][c] / m[c][c]
                for j in range(c, n + 1):
                    m[i][j] -= f * m[c][j]
    return [m[i][n] / m[i][i] for i in range(n)]


def sc(a):
    r = math.radians(a)
    return math.sin(r) * math.cos(r)


def s2(a):
    r = math.radians(a)
    return math.sin(r) ** 2


def fit(train, args):
    sub = [s for s in train if s["mach"] <= args.max_mach + 0.1
           and args.alpha_min <= s["aoa"] <= args.alpha_max]
    # Configuration increments first, from subsonic samples with known polar
    # shape: fit CD = d0 + KD s^2 + dg gear + dab airbrake over M in [0.1, 0.35]
    # with a Mach-slope term so the Mach trend does not alias into dg/dab.
    band = [s for s in sub if 0.1 <= s["mach"] <= 0.35]
    inc = _wls(band,
               lambda r: [1.0, s2(r["aoa"]), s2(r["aoa"]) * r["mach"], r["mach"],
                          r["gear"], r["airbrake"]],
               lambda r: r["cd"], lambda r: 1.0)
    d_gear, d_ab = inc[4], inc[5]
    shared = None
    if args.constrained:
        # Shared structure from the band where the flown AoA span is widest
        # (M 0.25-0.5, >10 deg): a common zero-AoA lift CL0 and a common
        # induced-drag ratio rho = KD/K.  Per knot only K and CD0 remain, which
        # stays well-posed where the flown AoA span is narrow (M<0.2, M>0.6).
        wide = [s for s in sub if 0.25 <= s["mach"] <= 0.5]
        g = _wls(wide, lambda r: [1.0, sc(r["aoa"]), sc(r["aoa"]) * (r["mach"] - 0.35)],
                 lambda r: r["cl"], lambda r: 1.0)
        cl0 = g[0]
        rows_k = []
        for mk in (0.25, 0.3, 0.35, 0.4, 0.45, 0.5):
            rs = [s for s in wide if abs(s["mach"] - mk) <= 0.025]
            k = _wls(rs, lambda r: [sc(r["aoa"])], lambda r: r["cl"] - cl0, lambda r: 1.0)
            kd = _wls(rs, lambda r: [1.0, s2(r["aoa"])],
                      lambda r: r["cd"] - d_gear * r["gear"] - d_ab * r["airbrake"], lambda r: 1.0)
            rows_k.append(kd[1] / k[0])
        shared = dict(cl0=cl0, kd_over_k=sum(rows_k) / len(rows_k), kd_over_k_by_mach=rows_k)
    knots = []
    for mk in MACH_KNOTS:
        h = max(0.03, 0.12 * mk)
        rows = [s for s in sub if abs(s["mach"] - mk) <= 3 * h]

        def w(r, mk=mk, h=h):
            return math.exp(-0.5 * ((r["mach"] - mk) / h) ** 2)
        eff_n = sum(w(r) for r in rows)
        alphas = sorted(r["aoa"] for r in rows)
        span = (alphas[int(0.95 * (len(alphas) - 1))] - alphas[int(0.05 * (len(alphas) - 1))]) if rows else 0.0
        if shared:
            ok = eff_n >= args.min_effective_samples and span >= 3.0
            kk = _wls(rows, lambda r: [sc(r["aoa"])], lambda r: r["cl"] - shared["cl0"], w) if ok else None
            cl = [shared["cl0"], kk[0]] if kk else None
            rho = shared["kd_over_k"]
            c0 = _wls(rows, lambda r: [1.0],
                      lambda r: r["cd"] - d_gear * r["gear"] - d_ab * r["airbrake"] - rho * cl[1] * s2(r["aoa"]),
                      w) if cl else None
            cd = [c0[0], rho * cl[1]] if c0 else None
        else:
            ok = eff_n >= args.min_effective_samples and span >= 4.0
            cl = _wls(rows, lambda r: [1.0, sc(r["aoa"])], lambda r: r["cl"], w) if ok else None
            cd = _wls(rows, lambda r: [1.0, s2(r["aoa"])],
                      lambda r: r["cd"] - d_gear * r["gear"] - d_ab * r["airbrake"], w) if ok else None
        knots.append(dict(mach=mk, effective_samples=round(eff_n, 1), alpha_span_deg=round(span, 1),
                          alpha_p05=alphas[int(0.05 * (len(alphas) - 1))] if alphas else None,
                          alpha_p95=alphas[int(0.95 * (len(alphas) - 1))] if alphas else None,
                          supported=ok, cl0=cl[0] if cl else None, k=cl[1] if cl else None,
                          cd0=cd[0] if cd else None, kd=cd[1] if cd else None))
    # Unsupported knots (M=0 and M=0.05 have no flight data) hold the nearest
    # supported knot's coefficients: KSP lift does not keep growing below the
    # lowest flown Mach without evidence, so we do not extrapolate the slope.
    sup = [k for k in knots if k["supported"]]
    for k in knots:
        if not k["supported"]:
            near = min(sup, key=lambda s: abs(s["mach"] - k["mach"]))
            for key in ("cl0", "k", "cd0", "kd"):
                k[key] = near[key]
            k["held_from_mach"] = near["mach"]
    return knots, dict(gear=d_gear, airbrake=d_ab, shared=shared)


def _interp_knots(knots, mach):
    if mach <= knots[0]["mach"]:
        return knots[0]
    for a, b in zip(knots, knots[1:]):
        if mach <= b["mach"]:
            t = (mach - a["mach"]) / (b["mach"] - a["mach"])
            return {k: a[k] + t * (b[k] - a[k]) for k in ("cl0", "k", "cd0", "kd")}
    return knots[-1]


def stall_factor(alpha, alpha_stall):
    """No stall is observed through 18-20 deg subsonic.  Above the observed
    ceiling, hold normal-force growth (factor on K) to avoid extrapolating
    unmeasured lift; it is flagged in the report as extrapolated."""
    if alpha <= alpha_stall:
        return 1.0
    return sc(alpha_stall) / sc(alpha) if sc(alpha) > 1e-9 else 1.0


def predict(knots, mach, alpha, alpha_stall):
    c = _interp_knots(knots, mach)
    f = stall_factor(alpha, alpha_stall)
    cl = c["cl0"] + c["k"] * sc(alpha) * f
    cd = c["cd0"] + c["kd"] * s2(alpha) * (f if alpha > alpha_stall else 1.0)
    return cl, cd


def load_table(path):
    rows = {}
    with open(path) as h:
        for line in h:
            if line.startswith("#") or line.startswith("mach"):
                continue
            p = line.strip().split(",")
            if len(p) < 4:
                continue
            rows[(float(p[0]), float(p[1]))] = (float(p[2]), float(p[3]))
    ms = sorted({k[0] for k in rows})
    al = sorted({k[1] for k in rows})
    return ms, al, rows


def table_eval(tab, mach, alpha):
    ms, al, rows = tab

    def lo(v, xs):
        for i in range(len(xs) - 1):
            if v < xs[i + 1]:
                return i
        return len(xs) - 2
    i, j = lo(mach, ms), lo(alpha, al)
    tm = min(1, max(0, (mach - ms[i]) / (ms[i + 1] - ms[i])))
    ta = min(1, max(0, (alpha - al[j]) / (al[j + 1] - al[j])))
    out = []
    for c in (0, 1):
        v0 = rows[(ms[i], al[j])][c] * (1 - ta) + rows[(ms[i], al[j + 1])][c] * ta
        v1 = rows[(ms[i + 1], al[j])][c] * (1 - ta) + rows[(ms[i + 1], al[j + 1])][c] * ta
        out.append(v0 * (1 - tm) + v1 * tm)
    return out


def residual_stats(samples, pred):
    bins = {}
    for s in samples:
        cl, cd = pred(s)
        if cl is None:
            continue
        mb = 0.1 if s["mach"] < 0.125 else round(s["mach"] / 0.05) * 0.05 if s["mach"] < 0.5 else round(s["mach"], 1)
        ab = 4 * round(s["aoa"] / 4)
        bins.setdefault((round(mb, 2), ab), []).append((s["cl"] - cl, s["cd"] - cd, s["cl"] / cl if abs(cl) > 0.05 else None))
    out = []
    for k in sorted(bins):
        v = bins[k]
        if len(v) < 20:
            continue
        rat = sorted(x[2] for x in v if x[2] is not None)
        out.append(dict(mach=k[0], alpha=k[1], n=len(v),
                        cl_bias=sum(x[0] for x in v) / len(v),
                        cl_rms=math.sqrt(sum(x[0] ** 2 for x in v) / len(v)),
                        cd_bias=sum(x[1] for x in v) / len(v),
                        cd_rms=math.sqrt(sum(x[1] ** 2 for x in v) / len(v)),
                        cl_ratio_p05=rat[int(0.05 * (len(rat) - 1))] if rat else None,
                        cl_ratio_p95=rat[int(0.95 * (len(rat) - 1))] if rat else None))
    return out


def sha256(path):
    h = hashlib.sha256()
    with open(path, "rb") as f:
        for chunk in iter(lambda: f.read(1 << 20), b""):
            h.update(chunk)
    return h.hexdigest()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--normalized", nargs="*", default=[])
    ap.add_argument("--live-logs", required=True)
    ap.add_argument("--heldout-log", action="append", default=[])
    ap.add_argument("--supersonic-table", required=True,
                    help="existing KSP-fitted table used above --max-mach")
    ap.add_argument("--table", required=True)
    ap.add_argument("--report", required=True)
    ap.add_argument("--prior-book", default=None,
                    help="guidance certified-prior force book (<=192 cells) sampled from the identified model")
    ap.add_argument("--max-mach", type=float, default=0.9)
    ap.add_argument("--blend-mach", type=float, default=1.1)
    ap.add_argument("--alpha-min", type=float, default=-4.0)
    ap.add_argument("--alpha-max", type=float, default=22.0)
    ap.add_argument("--alpha-stall", type=float, default=20.0)
    ap.add_argument("--min-q", type=float, default=300.0)
    ap.add_argument("--max-sideslip", type=float, default=5.0)
    ap.add_argument("--min-effective-samples", type=float, default=150.0)
    ap.add_argument("--jobs", type=int, default=8)
    ap.add_argument("--constrained", action="store_true",
                    help="shared CL0 and KD/K across Mach (see fit())")
    args = ap.parse_args()

    data = load(args)
    held = set(args.heldout_log)
    train = [s for s in data if not is_heldout(s["log"], held)]
    test = [s for s in data if is_heldout(s["log"], held)]
    knots, inc = fit(train, args)
    sup = load_table(args.supersonic_table)

    def blended(mach, alpha):
        cl_s, cd_s = table_eval(sup, mach, alpha)
        if mach <= args.max_mach:
            return predict(knots, mach, alpha, args.alpha_stall)
        cl_i, cd_i = predict(knots, args.max_mach, alpha, args.alpha_stall)
        t = min(1.0, (mach - args.max_mach) / (args.blend_mach - args.max_mach))
        return cl_i + t * (cl_s - cl_i), cd_i + t * (cd_s - cd_i)

    ms = sorted(set(MACH_KNOTS + [1.0, args.blend_mach] + [m for m in sup[0] if m > args.blend_mach]))
    with open(args.table, "w") as f:
        f.write("# STS-N aerodynamics identified from live KSP telemetry "
                "(ShuttleSim/tools/identify_stsn_aero.py). M<=%.2f: CL=CL0+K sin a cos a, "
                "CD=CD0+KD sin^2 a (clean configuration; gear/airbrake increments in the report). "
                "M>=%.2f: KSP-fitted table %s. See the report for held-out validation.\n"
                % (args.max_mach, args.blend_mach, os.path.basename(args.supersonic_table)))
        f.write("mach,alpha_deg,cl,cd\n")
        for m in ms:
            for a in ALPHA_KNOTS:
                cl, cd = blended(m, a)
                f.write("%g,%g,%.6f,%.6f\n" % (m, a, cl, cd))

    if args.prior_book:
        # Guidance's certified vessel-physics prior (VESSEL_AERO_SAMPLES=192).
        # Cells sit on flown (Mach, AoA) support only; q is the median flown q
        # at that Mach (the prior's force/q does not depend on it).
        prior_mach = [0.15, 0.2, 0.25, 0.3, 0.35, 0.4, 0.5, 0.6, 0.8, 0.95, 1.2, 1.5, 2.0, 2.5,
                      3.0, 4.0, 5.0, 6.0, 7.0, 8.0]
        cells = []
        for m in prior_mach:
            near = [x for x in train if abs(x["mach"] - m) <= max(0.025, 0.08 * m)]
            if len(near) < 50:
                continue
            al = sorted(x["aoa"] for x in near)
            qs = sorted(x["q"] for x in near)
            lo_a, hi_a = al[int(0.05 * (len(al) - 1))], al[int(0.95 * (len(al) - 1))]
            n_alpha = 9
            for i in range(n_alpha):
                a = lo_a + (hi_a - lo_a) * i / (n_alpha - 1)
                cl, cd = blended(m, a)
                if cd <= 0:
                    continue
                cells.append((qs[len(qs) // 2], m, a, AREA * cl, AREA * cd, max(1, min(12, len(near) // 400))))
        cells = cells[:192]
        with open(args.prior_book, "w") as f:
            f.write("# Guidance certified aero prior sampled from %s (identified model, flown support only).\n"
                    % os.path.basename(args.table))
            f.write("q_pa,mach,aoa_deg,lift_per_q_m2,drag_per_q_m2,support\n")
            for c in cells:
                f.write("%.1f,%.3f,%.3f,%.6f,%.6f,%d\n" % c)
        print("prior book: %d cells" % len(cells))

    def pred_id(s):
        cl, cd = blended(s["mach"], s["aoa"])
        return cl, cd + inc["gear"] * s["gear"] + inc["airbrake"] * s["airbrake"]

    def pred_sup(s):
        return table_eval(sup, s["mach"], s["aoa"])
    term = [s for s in test if s["mach"] <= args.max_mach and args.alpha_min <= s["aoa"] <= args.alpha_max]
    report = dict(
        tool="ShuttleSim/tools/identify_stsn_aero.py",
        model="CL=CL0(M)+K(M) sin a cos a; CD=CD0(M)+KD(M) sin^2 a + dCD_gear*gear + dCD_airbrake*airbrake",
        reference_area_m2=AREA,
        inputs=dict(normalized={p: sha256(p) for p in args.normalized},
                    live_logs_dir=args.live_logs,
                    live_logs=sorted({s["log"] for s in data if s["source"] == "live"}),
                    supersonic_table=dict(path=args.supersonic_table, sha256=sha256(args.supersonic_table))),
        split=dict(rule="held-out flights: --heldout-log plus crc32(log)%4==0",
                   train_samples=len(train), heldout_samples=len(test),
                   train_logs=len({s['log'] for s in train}), heldout_logs=len({s['log'] for s in test})),
        settings={k: v for k, v in vars(args).items() if k not in ("normalized",)},
        increments=inc,
        knots=knots,
        heldout_terminal_identified=residual_stats(term, pred_id),
        heldout_terminal_previous_fitted=residual_stats(term, pred_sup),
        heldout_supersonic_table=residual_stats([s for s in test if s["mach"] > args.blend_mach], pred_sup),
        table_sha256=sha256(args.table))
    with open(args.report, "w") as f:
        json.dump(report, f, indent=1)
    print("train %d samples/%d logs, held-out %d/%d; dCD gear %.4f airbrake %.4f shared %s"
          % (len(train), report["split"]["train_logs"], len(test), report["split"]["heldout_logs"],
             inc["gear"], inc["airbrake"], inc["shared"]))
    for k in knots:
        print("M%.2f n=%7.1f span=%4.1f %s CL0=%.3f K=%.3f CD0=%.3f KD=%.3f" % (
            k["mach"], k["effective_samples"], k["alpha_span_deg"], "fit " if k["supported"] else "hold",
            k["cl0"], k["k"], k["cd0"], k["kd"]))

    def summarize(name, rows):
        n = sum(r["n"] for r in rows)
        if not n:
            return
        clr = math.sqrt(sum(r["n"] * r["cl_rms"] ** 2 for r in rows) / n)
        cdr = math.sqrt(sum(r["n"] * r["cd_rms"] ** 2 for r in rows) / n)
        print("%-34s n=%6d CL rms %.3f  CD rms %.3f" % (name, n, clr, cdr))
    summarize("held-out terminal, identified", report["heldout_terminal_identified"])
    summarize("held-out terminal, previous fitted", report["heldout_terminal_previous_fitted"])
    summarize("held-out supersonic, fitted table", report["heldout_supersonic_table"])


if __name__ == "__main__":
    main()
