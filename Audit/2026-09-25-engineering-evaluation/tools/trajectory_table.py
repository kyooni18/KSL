#!/usr/bin/env python3
"""trajectory_table.py RUN_DIR_OR_RESULT_JSON [step_s] -- compact trajectory table from simulator telemetry."""
import gzip, json, sys, pathlib

arg = sys.argv[1]
step = float(sys.argv[2]) if len(sys.argv) > 2 else 2.0
p = pathlib.Path(arg)
if p.suffix == ".json":
    p = pathlib.Path(json.load(open(p))["sim"]).parent
rows = []
with gzip.open(p / "simulator-telemetry.jsonl.gz", "rt") as f:
    for line in f:
        d = json.loads(line)
        if d.get("type") != "telemetry":
            continue
        rows.append(d)
last = -1e9
print(" t     along   cross    h_agl   V    sink   fpa   aoa  bank  cmdaoa")
for d in rows:
    t = d["sim_time"]
    rw = d.get("runway", {})
    if t - last < step and d is not rows[-1]:
        continue
    last = t
    v = d["velocity"]; a = d["attitude"]
    h = d["position"]["altitude_m"] - 70.0
    print(f"{t:6.1f} {rw.get('along_m',0):8.0f} {rw.get('cross_m',0):6.0f} {h:7.1f} {v['air_mps']:5.1f} "
          f"{-v['vertical_mps']:5.1f} {v['flight_path_angle_deg']:5.1f} {a['aoa_deg']:5.1f} {a['bank_deg']:5.1f} {a['cmd_aoa_deg']:5.1f}")
# contact summary
for d in rows:
    c = d.get("contact") or d.get("gear") or {}
    if isinstance(c, dict) and (c.get("main") or c.get("mains") or c.get("any")):
        print("first contact record:", json.dumps(c)[:300], "t", d["sim_time"])
        break
