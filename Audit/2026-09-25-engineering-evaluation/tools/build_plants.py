#!/usr/bin/env python3
"""Rebuild the audit plant variants and plant-consistent force books.

Plant A = ShuttleSim built-in seed table (aero_seed_stsn), dumped by dump_seed.c.
Plant B = KSP-anchored: supersonic CL x0.30 / CD x0.65, subsonic parasite CD halved.
          Reproduces the four recorded KSP force samples in
          Docs/LatestFlight-2026-09-07-analysis.json to ~10-20%.
Plant C = higher-L/D dispersion: supersonic CL x0.45 / CD x0.65,
          subsonic CD0=0.05 + 0.7*induced (L/D max ~4.3).
These are audit constructs, NOT claims of KSP parity (the repo's fitted
ShuttleSim/data/fitted/* files are git-ignored and absent from the checkout).

usage: build_plants.py PLANT_DIR   (PLANT_DIR must contain stsn_aero_seed.csv)
"""
import bisect
import csv
import os
import sys


def cd0_of(m):
    return 0.16 if m < 0.8 else (0.28 if m < 1.2 else (0.34 if m < 3 else 0.42))


def build(src, dst, cl_super, sub_cd):
    rows = list(csv.DictReader(open(src)))
    with open(dst, "w") as out:
        out.write("mach,alpha_deg,cl,cd\n")
        for r in rows:
            m, a = float(r["mach"]), float(r["alpha_deg"])
            cl, cd = float(r["cl"]), float(r["cd"])
            ind = cd - cd0_of(m)
            s = min(1, max(0, (m - 0.8) / 0.4))
            clk = 1.0 + (cl_super - 1.0) * s
            sc = sub_cd(cd0_of(m), ind)
            sup = 0.65 * cd
            out.write(f"{m:g},{a:g},{cl*clk:.9g},{sc+(sup-sc)*s:.9g}\n")


def load(fn):
    rows = list(csv.DictReader(open(fn)))
    M = sorted({float(r["mach"]) for r in rows})
    A = sorted({float(r["alpha_deg"]) for r in rows})
    T = {(float(r["mach"]), float(r["alpha_deg"])): (float(r["cl"]), float(r["cd"])) for r in rows}
    return M, A, T


def interp(M, A, T, m, a):
    lo = lambda arr, x: max(0, min(len(arr) - 2, bisect.bisect_right(arr, x) - 1))
    i, j = lo(M, m), lo(A, a)
    tm = min(1, max(0, (m - M[i]) / (M[i + 1] - M[i])))
    ta = min(1, max(0, (a - A[j]) / (A[j + 1] - A[j])))
    res = []
    for k in range(2):
        v0 = T[(M[i], A[j])][k] * (1 - ta) + T[(M[i], A[j + 1])][k] * ta
        v1 = T[(M[i + 1], A[j])][k] * (1 - ta) + T[(M[i + 1], A[j + 1])][k] * ta
        res.append(v0 * (1 - tm) + v1 * tm)
    return res


def book(table, dst):
    M, A, T = load(table)
    with open(dst, "w") as out:
        out.write("q_pa,mach,alpha_deg,lift_per_q_m2,drag_per_q_m2,support\n")
        for m in [0.3, 0.5, 0.7, 0.9, 1.1, 1.4, 1.8, 2.3, 3, 4, 5, 6, 7]:
            qs = [3000, 9000] if m < 1 else ([1500, 6000] if m < 3 else [150, 1200])
            for a in [5, 10, 15, 20, 25, 28]:
                for q in qs:
                    cl, cd = interp(M, A, T, m, a)
                    out.write(f"{q},{m},{a},{250*cl:.6f},{250*cd:.6f},4\n")


if __name__ == "__main__":
    d = sys.argv[1]
    seed = os.path.join(d, "stsn_aero_seed.csv")
    build(seed, os.path.join(d, "stsn_aero_kspB.csv"), 0.30, lambda c0, ind: 0.5 * c0 + ind)
    build(seed, os.path.join(d, "stsn_aero_kspC.csv"), 0.45, lambda c0, ind: 0.05 + 0.7 * ind)
    for tag, fn in [("A", "stsn_aero_seed.csv"), ("B", "stsn_aero_kspB.csv"), ("C", "stsn_aero_kspC.csv")]:
        book(os.path.join(d, fn), os.path.join(d, f"force_book_{tag}.csv"))
