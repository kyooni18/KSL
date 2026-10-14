#!/usr/bin/env python3
"""touchdown_summary.py RESULT_JSON... -- first-contact (or abort) summary per run_guidance.py result."""
import gzip, json, sys, pathlib
for arg in sys.argv[1:]:
    try:
        r = json.load(open(arg))
    except Exception as e:
        print(arg, "unreadable", e); continue
    run = pathlib.Path(r["sim"]).parent
    prev = None; td = None; maxh = None
    with gzip.open(run / "simulator-telemetry.jsonl.gz", "rt") as f:
        for line in f:
            d = json.loads(line)
            if d.get("type") != "telemetry": continue
            g = d.get("ground", {})
            if g.get("on_ground") and td is None:
                td = prev or d
            prev = d
    name = pathlib.Path(arg).stem
    ab = (r.get("abortReason") or "")[:90]
    if td:
        v = td["velocity"]; rw = td["runway"]
        print(f"{name:22s} ok={r.get('success')} td along={rw['along_m']:7.0f} cross={rw['cross_m']:6.1f} V={v['air_mps']:5.1f} sink={-v['vertical_mps']:5.2f} t={td['sim_time']:.1f} | {ab}")
    else:
        print(f"{name:22s} ok={r.get('success')} no touchdown t={prev['sim_time'] if prev else 0:.1f} | {ab}")
