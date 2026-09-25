#!/usr/bin/env python3
"""Print a trajectory table from a sweep result JSON file.

Usage:
  trajectory_table.py <result_json> [step_seconds]
"""

import gzip
import json
import pathlib
import sys


def open_maybe_gzip(path):
    p = pathlib.Path(path)
    if not p.is_file():
        raise FileNotFoundError(f"File not found: {p}")
    if p.suffix == ".gz":
        return gzip.open(p, "rt", encoding="utf-8")
    return open(p, "rt", encoding="utf-8")


def main():
    if len(sys.argv) < 2:
        print("Usage: trajectory_table.py <result_json> [step_seconds]", file=sys.stderr)
        sys.exit(1)

    json_path = pathlib.Path(sys.argv[1])
    step_s = float(sys.argv[2]) if len(sys.argv) > 2 else 2.0

    with open(json_path, "r", encoding="utf-8") as f:
        meta = json.load(f)

    sim_path = meta.get("sim")
    if not sim_path:
        print(f"No sim telemetry path in {json_path}", file=sys.stderr)
        sys.exit(1)

    # Optional: read guidance events to map phases by time
    phases_by_ut = []
    guidance_path = meta.get("guidance")
    if guidance_path and pathlib.Path(guidance_path).is_file():
        try:
            with open_maybe_gzip(guidance_path) as gf:
                for line in gf:
                    ev = json.loads(line)
                    if ev.get("type") == "event" and "phase" in ev.get("event", {}):
                        phases_by_ut.append((ev.get("ut", 0.0), ev["event"]["phase"]))
        except Exception:
            pass

    def get_phase(ut):
        if not phases_by_ut:
            return ""
        cur = ""
        for u, ph in phases_by_ut:
            if u <= ut:
                cur = ph
            else:
                break
        return cur

    header = (
        f"{'t (s)':>7}  {'alt (m)':>8}  {'v (m/s)':>8}  {'mach':>5}  "
        f"{'q (kPa)':>7}  {'aoa (deg)':>9}  {'bank':>6}  {'fpa':>6}  "
        f"{'sink':>6}  {'along (m)':>10}  {'cross (m)':>9}  {'phase':<12}"
    )
    print(header)
    print("-" * len(header))

    last_time = -1e9
    was_on_ground = False
    with open_maybe_gzip(sim_path) as f:
        for line in f:
            rec = json.loads(line)
            t = rec.get("sim_time", 0.0)
            ground = rec.get("ground", {})
            on_ground = ground.get("on_ground", False)
            # Print row if step_s elapsed, or at transition to on_ground, or at stopped
            is_touchdown_event = (not was_on_ground) and on_ground
            is_stopped_event = on_ground and ground.get("stopped", False)
            was_on_ground = on_ground

            if (t - last_time < step_s) and not is_touchdown_event and not is_stopped_event:
                continue
            last_time = t

            pos = rec.get("position", {})
            vel = rec.get("velocity", {})
            att = rec.get("attitude", {})
            aero = rec.get("aero", {})
            rwy = rec.get("runway", {})

            alt = pos.get("altitude_m", 0.0)
            v = vel.get("surface_mps", vel.get("air_mps", 0.0))
            mach = aero.get("mach", 0.0)
            q = aero.get("q_pa", 0.0) / 1000.0
            aoa = att.get("aoa_deg", 0.0)
            bank = att.get("bank_deg", 0.0)
            fpa = vel.get("flight_path_angle_deg", 0.0)
            sink = -vel.get("vertical_mps", 0.0)
            along = rwy.get("along_m", 0.0)
            cross = rwy.get("cross_m", 0.0)
            ut = rec.get("ut", 0.0)
            phase = get_phase(ut)

            print(
                f"{t:7.1f}  {alt:8.1f}  {v:8.1f}  {mach:5.2f}  "
                f"{q:7.2f}  {aoa:9.2f}  {bank:6.1f}  {fpa:6.1f}  "
                f"{sink:6.1f}  {along:10.1f}  {cross:9.1f}  {phase:<12}"
            )

            if on_ground and ground.get("stopped", False):
                break


if __name__ == "__main__":
    main()
