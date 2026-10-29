#!/usr/bin/env python3
"""Read-only: compare the Swift engine's decoded state with kRPC's own flight values.

Run with KSP in the flight scene. Never writes to the vessel. Large disagreements in
speed, angle of attack, sideslip, bank, or altitude mean a frame/sign bug in the Swift adapter.
"""
import argparse, json, math, subprocess, sys
from pathlib import Path

import krpc

ROOT = Path(__file__).resolve().parents[1]
SWIFT = ROOT / "swift/KSL-Landing/.build/release/ksl-landing"

ap = argparse.ArgumentParser()
ap.add_argument("--host", default="127.0.0.1"); ap.add_argument("--port", type=int, default=50000)
ap.add_argument("--stream-port", type=int, default=50001); ap.add_argument("--vessel", default="STS")
a = ap.parse_args()

conn = krpc.connect(name="swift-compare", address=a.host, rpc_port=a.port, stream_port=a.stream_port)
v = conn.space_center.active_vessel
body = v.orbit.body.reference_frame
hybrid = conn.space_center.ReferenceFrame.create_hybrid(position=body, rotation=v.surface_reference_frame)
f, fh = v.flight(v.surface_reference_frame), v.flight(hybrid)
ref = {"ut": conn.space_center.ut, "speed": fh.speed, "aoa": f.angle_of_attack, "sideslip": f.sideslip_angle,
       "roll": f.roll, "pitch": f.pitch, "heading": f.heading, "q": f.dynamic_pressure,
       "surfaceAltitude": f.surface_altitude, "velocity(up,north,east)": fh.velocity}
out = subprocess.run([str(SWIFT), "inspect", "--host", a.host, "--port", str(a.port), "--vessel", a.vessel],
                     capture_output=True, text=True)
if out.returncode:
    sys.exit(out.stderr)
s = json.loads(out.stdout)["state"]; ae = s["aerodynamics"]
swift = {"ut": s["time"], "speed": ae["airspeed"], "aoa": math.degrees(ae["angleOfAttack"]),
         "sideslip": math.degrees(ae["sideslip"]), "bank": math.degrees(ae["bank"]), "q": ae["dynamicPressure"],
         "surfaceAltitude": s["radarAltitude"], "velocity(up,north,east)": s["velocity"]}
print("kRPC :", json.dumps(ref, indent=1)); print("Swift:", json.dumps(swift, indent=1))
print("\nNote: kRPC roll is body-axis roll; Swift bank is the velocity-axis bank. Compare AoA, sideslip, speed, altitude, velocity.")
