#!/usr/bin/env python3
"""Final-phase closed-loop campaign on ShuttleSim.

Generates runway-relative Final checkpoints (distance before the threshold,
glide angle, speed, cross-track with mirrored pairs, heading error) and plant
dispersions (lift/drag scale within the identified model's held-out band),
runs each through run_guidance.py (engageFinalTest) in parallel, and writes a
JSONL of classified results plus a summary.

Classification is by simulator truth first (first match wins); a guidance
abort latched after the physical outcome (e.g. the runway-envelope abort when
the vehicle rolls off the far end) is recorded alongside, not instead:
  abort:<reason>        guidance terminated before any touchdown
  no-touchdown          sim ended airborne
  crash                 non-survivable impact
  belly / tail-strike   structure contact before/without gear
  off-runway-touchdown  first contact outside the paved rectangle
  hard-touchdown        sink > gate
  speed                 touchdown speed outside the runner gate
  long-touchdown        first contact beyond the touchdown zone (--long-along)
  runway-departure      rolled off the pavement
  not-stopped           rollout did not end stopped on the runway
  pass
"""
from __future__ import annotations

import argparse
import concurrent.futures as cf
import hashlib
import json
import math
import pathlib
import random
import subprocess
import sys
import time

SIM = pathlib.Path(__file__).resolve().parents[1]
ROOT = SIM.parent
R_KERBIN = 600000.0
RUNWAY = dict(lat=-0.0486111111, lon=-74.7283333333, elev=70.0, heading=90.0, length=2500.0, width=70.0)
UT0 = 66768.169814
MASS = 40252.917969


def scenario_text(name, along, cross, height, speed, fpa_deg, heading_err_deg, aoa_deg):
    # Runway 09 points east: along -> longitude, cross (right of centreline)
    # -> south.
    r = R_KERBIN + RUNWAY["elev"]
    lat = RUNWAY["lat"] - math.degrees(cross / r)
    lon = RUNWAY["lon"] + math.degrees(along / (r * math.cos(math.radians(RUNWAY["lat"]))))
    return "\n".join([
        f"# Final campaign checkpoint (run_final_campaign.py): along={along:.0f} cross={cross:.0f} "
        f"height={height:.0f} speed={speed:.0f} fpa={fpa_deg:.1f} heading_err={heading_err_deg:.1f}",
        f"name={name}", f"ut0={UT0}", f"latitude_deg={lat:.10f}", f"longitude_deg={lon:.10f}",
        f"altitude_m={RUNWAY['elev'] + height:.3f}", f"heading_deg={RUNWAY['heading'] + heading_err_deg:.4f}",
        f"surface_speed_mps={speed:.3f}", f"flight_path_angle_deg={fpa_deg:.4f}",
        f"mass_kg={MASS}", f"initial_aoa_deg={aoa_deg:.3f}", "initial_bank_deg=0",
        "deorbit_delta_v_mps=0", "deorbit_duration_s=0", "deorbit_delay_s=0",
        f"runway_latitude_deg={RUNWAY['lat']}", f"runway_longitude_deg={RUNWAY['lon']}",
        f"runway_elevation_m={RUNWAY['elev']}", f"runway_heading_deg={RUNWAY['heading']}",
        f"runway_length_m={RUNWAY['length']}", f"runway_width_m={RUNWAY['width']}", ""])


def required_ld(along, height, speed, v_td=75.0, target=375.0, g=9.81):
    """Mean lift-to-drag the vehicle must fly from the checkpoint to arrive
    at the touchdown target at touchdown speed (energy height over range).
    A straight-in Final cannot fly much below its drag-polar L/D without a
    speedbrake, so a low value marks an energy-infeasible (too high or too
    fast) Final state rather than a guidance failure."""
    energy = height + (speed * speed - v_td * v_td) / (2.0 * g)
    return (target - along) / max(energy, 1.0)


def make_cases(n, seed, mirror=True, ld_band=None, speed=(140.0, 220.0), along_range=(2500.0, 6000.0),
               glide_range=(12.0, 26.0)):
    rng = random.Random(seed)
    cases = []
    while len(cases) < n:
        along = -rng.uniform(*along_range)
        glide = rng.uniform(*glide_range)
        height = -along * math.tan(math.radians(glide))
        speed_ = rng.uniform(*speed)
        ld = required_ld(along, height, speed_)
        if ld_band and not (ld_band[0] <= ld <= ld_band[1]):
            continue
        cross = rng.uniform(0.0, 250.0)
        herr = rng.uniform(-6.0, 6.0)
        lift = rng.uniform(0.88, 1.12)
        drag = rng.uniform(0.9, 1.1)
        aoa = rng.uniform(2.0, 6.0)
        base = dict(along=along, height=height, speed=speed_, fpa=-glide, aoa=aoa,
                    lift_scale=lift, drag_scale=drag, ld_required=ld)
        cases.append(dict(base, cross=cross, heading_err=herr))
        if mirror and len(cases) < n:
            cases.append(dict(base, cross=-cross, heading_err=-herr, mirror_of=len(cases) - 1))
    for i, c in enumerate(cases):
        c["id"] = i
    return cases


def classify(res, args):
    sim = res.get("simulatorSummary") or {}
    if not sim.get("touchdown"):
        if res.get("abortReason"):
            return "abort:" + res["abortReason"][:60]
        return "no-touchdown"
    if sim.get("crashed"):
        return "crash"
    if not sim.get("touchdown_gear"):
        return "tail-strike" if sim.get("tail_strike") else "belly"
    if not sim.get("on_runway"):
        return "off-runway-touchdown"
    if sim.get("touchdown_sink_mps", 99) > args.sink_gate:
        return "hard-touchdown"
    spd = sim.get("touchdown_speed_mps", 0)
    if not (args.speed_lo <= spd <= args.speed_hi):
        return "speed"
    if sim.get("touchdown_along_m", 0) > args.long_along:
        return "long-touchdown"
    if sim.get("runway_departure"):
        return "runway-departure"
    if not sim.get("stopped"):
        return "not-stopped"
    return "pass"


def run_case(c, args, scen_dir):
    name = f"{args.label}-{c['id']:03d}"
    scen = scen_dir / f"{name}.ini"
    scen.write_text(scenario_text(name, c["along"], c["cross"], c["height"], c["speed"], c["fpa"],
                                  c["heading_err"], c["aoa"]))
    cmd = [sys.executable, str(SIM / "scripts/run_guidance.py"), "--scenario", str(scen),
           "--engage", "engageFinalTest", "--label", name, "--no-mirror", "--quiet-progress",
           "--skip-build", "--max-sim-time", str(args.max_sim_time),
           "--plant-lift-scale", repr(c["lift_scale"]), "--plant-drag-scale", repr(c["drag_scale"])]
    if args.direct_control:
        cmd.append("--direct-control")
    t0 = time.time()
    p = subprocess.run(cmd, capture_output=True, text=True, cwd=ROOT)
    wall = time.time() - t0
    res = {}
    for line in reversed(p.stdout.splitlines()):
        if line.startswith("{"):
            try:
                res = json.loads(line)
                break
            except json.JSONDecodeError:
                pass
    # The runner's summary may be empty; read the simulator's own summary line.
    sim = res.get("simulatorSummary") or {}
    err = pathlib.Path(res.get("simStderr", "")) if res.get("simStderr") else None
    if err and err.exists():
        for line in err.read_text().splitlines():
            if line.startswith("SHUTTLESIM_SUMMARY "):
                sim = json.loads(line.split(" ", 1)[1])
    res["simulatorSummary"] = sim
    last = None
    tel = pathlib.Path(res.get("sim", "")) if res.get("sim") else None
    if tel and tel.exists():
        import gzip
        with gzip.open(tel, "rt") as fh:
            for line in fh:
                last = line
    end_state = {}
    if last:
        try:
            r = json.loads(last)
            end_state = dict(t=r["sim_time"], along=r["runway"]["along_m"], cross=r["runway"]["cross_m"],
                             height=r["runway"]["vertical_m"], speed=r["velocity"]["surface_mps"],
                             vs=r["velocity"]["vertical_mps"], bank=r["attitude"]["bank_deg"])
        except (KeyError, json.JSONDecodeError):
            pass
    out = dict(case=c, runId=res.get("runId"), wall_s=round(wall, 1), returncode=p.returncode,
               abortReason=res.get("abortReason"), runnerSuccess=res.get("success"),
               summary={k: sim.get(k) for k in (
                   "touchdown", "on_runway", "touchdown_gear", "touchdown_sink_mps", "touchdown_speed_mps",
                   "touchdown_along_m", "touchdown_cross_m", "touchdown_pitch_deg", "nose_touchdown_sink_mps",
                   "bounce_count", "max_gear_load_g", "runway_departure", "stopped", "stop_along_m",
                   "stop_cross_m", "crashed", "tail_strike", "sim_time_s")})
    out["end_state"] = end_state
    out["class"] = classify(res, args)
    if p.returncode != 0 and not res:
        out["class"] = "harness-error"
        out["stderr_tail"] = p.stderr[-600:]
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--label", default="finalcamp")
    ap.add_argument("--cases", type=int, default=24)
    ap.add_argument("--seed", type=int, default=1)
    ap.add_argument("--jobs", type=int, default=6)
    ap.add_argument("--direct-control", action="store_true")
    ap.add_argument("--nominal-plant", action="store_true", help="lift/drag scale 1 (model = plant)")
    ap.add_argument("--max-sim-time", type=float, default=240.0)
    ap.add_argument("--sink-gate", type=float, default=3.0)
    ap.add_argument("--speed-lo", type=float, default=0.85 * 75.0)
    ap.add_argument("--speed-hi", type=float, default=1.15 * 75.0)
    ap.add_argument("--ld-band", type=float, nargs=2, default=None,
                    help="only keep checkpoints whose required mean L/D is inside this band")
    ap.add_argument("--speed-range", type=float, nargs=2, default=(140.0, 220.0))
    ap.add_argument("--along-range", type=float, nargs=2, default=(2500.0, 6000.0),
                    help="checkpoint distance before the threshold, m")
    ap.add_argument("--glide-range", type=float, nargs=2, default=(12.0, 26.0))
    ap.add_argument("--long-along", type=float, default=1000.0,
                    help="first contact beyond this along-track distance is a long landing")
    ap.add_argument("--out", type=pathlib.Path, default=None)
    args = ap.parse_args()

    stamp = time.strftime("%Y%m%dT%H%M%SZ", time.gmtime())
    out_dir = args.out or SIM / "campaigns" / f"{stamp}-{args.label}"
    out_dir.mkdir(parents=True, exist_ok=True)
    scen_dir = out_dir / "scenarios"
    scen_dir.mkdir(exist_ok=True)
    subprocess.run(["make", "-C", str(ROOT / "CLanding"), "-j4"], check=True, stdout=subprocess.DEVNULL)
    subprocess.run(["cmake", "--build", str(SIM / "build"), "-j4"], check=True, stdout=subprocess.DEVNULL)
    cases = make_cases(args.cases, args.seed, ld_band=args.ld_band, speed=args.speed_range,
                       along_range=args.along_range, glide_range=args.glide_range)
    if args.nominal_plant:
        for c in cases:
            c["lift_scale"] = c["drag_scale"] = 1.0
    head = subprocess.run(["git", "-C", str(ROOT), "rev-parse", "HEAD"], capture_output=True, text=True).stdout.strip()
    dirty = subprocess.run(["git", "-C", str(ROOT), "status", "--porcelain", "--untracked-files=no"],
                           capture_output=True, text=True).stdout.strip()
    manifest = dict(tool="ShuttleSim/scripts/run_final_campaign.py", args={k: str(v) for k, v in vars(args).items()},
                    gitHead=head, gitDirty=bool(dirty),
                    gitDiffDigest=hashlib.sha256(subprocess.run(["git", "-C", str(ROOT), "diff", "HEAD"],
                                                                capture_output=True, text=True).stdout.encode()).hexdigest(),
                    started=stamp, cases=len(cases),
                    note="per-run manifests (runs/<runId>/manifest.json) carry model profile and file digests")
    (out_dir / "manifest.json").write_text(json.dumps(manifest, indent=1))
    results = []
    with cf.ThreadPoolExecutor(args.jobs) as ex, open(out_dir / "results.jsonl", "w") as f:
        for r in ex.map(lambda c: run_case(c, args, scen_dir), cases):
            results.append(r)
            f.write(json.dumps(r) + "\n")
            f.flush()
            s = r["summary"]
            c = r["case"]
            e = r.get("end_state") or {}
            print("%3d %-26s along %6.0f h %5.0f V %4.0f LD %.2f x %+5.0f L%.2f D%.2f | td %s sink %s V %s at %s stop %s | end x %s a %s" % (
                c["id"], r["class"][:26], c["along"], c["height"], c["speed"], c["ld_required"], c["cross"], c["lift_scale"],
                c["drag_scale"], s.get("on_runway"),
                "%.2f" % s["touchdown_sink_mps"] if s.get("touchdown_sink_mps") is not None else "-",
                "%.1f" % s["touchdown_speed_mps"] if s.get("touchdown_speed_mps") is not None else "-",
                "%.0f" % s["touchdown_along_m"] if s.get("touchdown_along_m") is not None else "-",
                "%.0f" % s["stop_along_m"] if s.get("stop_along_m") else "-",
                "%.0f" % e["cross"] if "cross" in e else "-", "%.0f" % e["along"] if "along" in e else "-"), flush=True)
    counts = {}
    for r in results:
        counts[r["class"]] = counts.get(r["class"], 0) + 1
    summary = dict(counts=counts, passes=counts.get("pass", 0), total=len(results))
    (out_dir / "summary.json").write_text(json.dumps(summary, indent=1))
    print(json.dumps(summary))
    print("campaign:", out_dir)


if __name__ == "__main__":
    main()
