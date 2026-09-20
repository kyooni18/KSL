#!/usr/bin/env python3
"""Apply a short KSP pitch pulse and report the measured body-rate response."""

from __future__ import annotations

import argparse
import json
import time

import krpc


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--input", type=float, required=True)
    parser.add_argument("--duration", type=float, default=0.8)
    args = parser.parse_args()

    conn = krpc.connect(
        name="STS-N Control Sign Probe",
        address="127.0.0.1",
        rpc_port=50000,
        stream_port=50001,
    )
    try:
        sc = conn.space_center
        vessel = sc.active_vessel
        control = vessel.control
        flight = vessel.flight()
        frame = vessel.reference_frame

        control.sas = False
        control.pitch = 0.0
        control.roll = 0.0
        control.yaw = 0.0

        def sample() -> dict[str, float]:
            raw_rate = vessel.angular_velocity(frame)
            return {
                "ut": float(sc.ut),
                "pitch": float(flight.pitch),
                "angleOfAttack": float(flight.angle_of_attack),
                "bodyPitchRate": -float(raw_rate[0]),
            }

        time.sleep(0.25)
        before = sample()
        control.pitch = max(-1.0, min(1.0, args.input))
        time.sleep(max(0.1, args.duration))
        after = sample()
        control.pitch = 0.0
        print(json.dumps({"input": args.input, "before": before, "after": after}))
        return 0
    finally:
        try:
            vessel.control.pitch = 0.0
            vessel.control.roll = 0.0
            vessel.control.yaw = 0.0
        except Exception:
            pass
        conn.close()


if __name__ == "__main__":
    raise SystemExit(main())
