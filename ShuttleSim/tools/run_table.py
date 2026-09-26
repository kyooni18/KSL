#!/usr/bin/env python3
"""Tabulate a ShuttleSim run: simulator truth plus the guidance phase.

usage: run_table.py RUN_DIR [--every SECONDS] [--from T] [--to T] [--tail-alt M]

--tail-alt prints every sample once the runway-relative height drops below M.
"""
import argparse
import gzip
import json
import pathlib


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("run")
    ap.add_argument("--every", type=float, default=2.0)
    ap.add_argument("--from", dest="t0", type=float, default=0.0)
    ap.add_argument("--to", dest="t1", type=float, default=1e9)
    ap.add_argument("--tail-alt", type=float, default=0.0)
    a = ap.parse_args()
    run = pathlib.Path(a.run)
    phases = []
    g = run / "guidance-events.jsonl.gz"
    if g.exists():
        with gzip.open(g, "rt") as f:
            for line in f:
                try:
                    r = json.loads(line)
                except json.JSONDecodeError:
                    continue
                snap = r.get("snapshot") or r.get("payload") or r
                ph = snap.get("phase") if isinstance(snap, dict) else None
                ut = snap.get("ut") if isinstance(snap, dict) else None
                if ph and isinstance(ut, (int, float)):
                    phases.append((ut, ph))
    phases.sort()
    print("%6s %8s %6s %7s %6s %6s %6s %5s %5s %6s %6s %6s %5s %s" % (
        "t", "along", "cross", "h", "V", "vs", "fpa", "aoa", "cmdA", "bank", "q", "n_z", "gear", "phase"))
    last = -1e9
    pi = 0
    phase = ""
    with gzip.open(run / "simulator-telemetry.jsonl.gz", "rt") as f:
        for line in f:
            try:
                r = json.loads(line)
            except json.JSONDecodeError:
                continue
            if r.get("type") != "telemetry":
                continue
            t = r["sim_time"]
            while pi < len(phases) and phases[pi][0] <= r["ut"]:
                phase = phases[pi][1]
                pi += 1
            h = r["runway"]["vertical_m"]
            tail = a.tail_alt > 0 and h < a.tail_alt and not r["ground"]["on_ground"]
            if not (a.t0 <= t <= a.t1):
                continue
            if t - last < a.every and not tail and not r["ground"]["touchdown_seen"] == True:
                continue
            last = t
            m = r["vehicle"]["mass_kg"]
            nz = r["aero"]["lift_n"] / (m * 9.81) if m else 0
            print("%6.1f %8.0f %6.1f %7.1f %6.1f %6.2f %6.1f %5.1f %5.1f %6.1f %6.0f %6.2f %5d %s" % (
                t, r["runway"]["along_m"], r["runway"]["cross_m"], h, r["velocity"]["air_mps"],
                r["velocity"]["vertical_mps"], r["velocity"]["flight_path_angle_deg"],
                r["attitude"]["aoa_deg"], r["attitude"]["cmd_aoa_deg"], r["attitude"]["bank_deg"],
                r["aero"]["q_pa"], nz, r["ground"]["gear_down"], phase))
            if r["ground"]["touchdown_seen"] and r["ground"]["on_ground"]:
                break
    s = run / "simulator-summary.json"
    for cand in (s, run / "summary.json"):
        if cand.exists():
            print(cand.read_text()[:1500])


if __name__ == "__main__":
    main()
