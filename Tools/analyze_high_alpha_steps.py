#!/usr/bin/env python3
"""Summarize a high-alpha aero-cell flight (KSP_LANDER_ENTRY_AOA_STAIRCASE_TEST).

For each steady incidence step in the MM304 part of an STS vehicle log, report
Mach span, held AoA, CL/CD and L/D (same normalization as fit_sts_aero_cells),
and the yaw-authority evidence the Shuttle-style 40 deg hold depends on:
sideslip, yaw command use, saturation, and RCS state.
"""
import argparse, gzip, json, statistics


def open_log(path):
    return gzip.open(path, "rt") if str(path).endswith(".gz") else open(path)


def merge(a, b):
    for k, v in b.items():
        if isinstance(v, dict):
            if not isinstance(a.get(k), dict):
                a[k] = {}
            merge(a[k], v)
        else:
            a[k] = v


def rows(path, area, q_min):
    st = {}
    for line in open_log(path):
        try:
            d = json.loads(line)
        except ValueError:
            continue
        if d.get("recordType") in ("vehicleKeyframe", "keyframe"):
            st = {}
            merge(st, d.get("fields", d))
        elif d.get("recordType") == "vehicleDelta":
            merge(st, d.get("fields", {}))
        else:
            continue
        ae, at, s = st.get("aero") or {}, st.get("attitude") or {}, st.get("state") or {}
        ac = st.get("appliedControl") or {}
        q = ae.get("dynamicPressure") or 0.0
        if s.get("vesselSituation") != "flying" or q < q_min or (st.get("vehicle") or {}).get("currentThrust"):
            continue
        m, al, lf, df = ae.get("mach"), at.get("angleOfAttack"), ae.get("liftForce"), ae.get("dragForce")
        if None in (m, al, lf, df):
            continue
        diag = ac.get("controlDiagnostics") or {}
        yield dict(ut=d.get("ut"), mach=m, q=q, aoa=al, beta=at.get("sideslip") or 0.0,
                   pitch_rate=at.get("bodyPitchRate") or 0.0,
                   cmd_aoa=(st.get("command") or {}).get("targetAoA"),
                   yaw=ac.get("reportedYaw") or 0.0, roll=at.get("roll") or 0.0,
                   rcs=bool(diag.get("rcsRequested") if isinstance(diag, dict) else False),
                   cl=lf / (q * area), cd=df / (q * area))


def steps(samples, tol):
    """Group consecutive samples whose commanded AoA stays within tol."""
    group = []
    for r in samples:
        c = r["cmd_aoa"]
        if group and (c is None or group[0]["cmd_aoa"] is None or abs(c - group[0]["cmd_aoa"]) > tol):
            yield group
            group = []
        group.append(r)
    if group:
        yield group


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("log")
    ap.add_argument("--area", type=float, default=250.0)
    ap.add_argument("--q-min", type=float, default=300.0)
    ap.add_argument("--min-mach", type=float, default=4.0)
    a = ap.parse_args()
    data = [r for r in rows(a.log, a.area, a.q_min) if r["mach"] >= a.min_mach]
    print(f"{'Mach':>11} {'cmd':>5} {'aoa':>5} {'n':>4} {'CL':>6} {'CD':>6} {'L/D':>5} "
          f"{'|b|med':>6} {'|b|max':>6} {'|yaw|med':>8} {'sat%':>5} {'rcs%':>5}")
    for g in steps(data, 0.5):
        if len(g) < 20:
            continue
        steady = [r for r in g if abs(r["aoa"] - g[0]["cmd_aoa"]) <= 1.25] if g[0]["cmd_aoa"] is not None else g
        if not steady:
            steady = g
        cl = statistics.median(r["cl"] for r in steady)
        cd = statistics.median(r["cd"] for r in steady)
        b = [abs(r["beta"]) for r in g]
        y = [abs(r["yaw"]) for r in g]
        print(f"{g[0]['mach']:5.2f}-{g[-1]['mach']:5.2f} {g[0]['cmd_aoa'] or float('nan'):5.1f} "
              f"{statistics.median(r['aoa'] for r in g):5.1f} {len(steady):4d} {cl:6.3f} {cd:6.3f} {cl / cd:5.2f} "
              f"{statistics.median(b):6.2f} {max(b):6.2f} {statistics.median(y):8.2f} "
              f"{100 * sum(v >= 0.99 for v in y) / len(y):5.0f} {100 * sum(r['rcs'] for r in g) / len(g):5.0f}")


if __name__ == "__main__":
    main()
