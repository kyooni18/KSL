#!/usr/bin/env python3
"""Bounded STS vacuum yaw pulse with an inertial-to-vessel rate transform check."""
from __future__ import annotations

import hashlib
import json
import math
import os
from pathlib import Path
import subprocess
import sys
import time

from headless_flight import restore_quicksave, secure_failed_test_state
from ksp_test_guard import connect

ROOT = Path(__file__).resolve().parents[1]


def watchdog(start_ut: float, parent_pid: int, ready_path: Path) -> None:
    conn = connect("STS yaw frame crosscheck independent clock guard", timeout=8)
    try:
        sc = conn.space_center
        vessel = sc.active_vessel
        if vessel.name != "STS" or not conn.krpc.paused:
            raise RuntimeError("Guard requires frozen original STS")
        ready_path.write_text("ready")
        deadline = time.monotonic() + 15
        while time.monotonic() < deadline:
            try:
                os.kill(parent_pid, 0)
            except ProcessLookupError:
                break
            if sc.ut > start_ut + 3:
                break
            time.sleep(0.02)
        conn.krpc.paused = True
        vessel.control.yaw = 0
    finally:
        conn.close()


def main() -> None:
    import argparse

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--profile", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    profile = json.loads(args.profile.read_text())
    config = json.loads((ROOT / "Configuration/sts.json").read_text())
    save = Path(profile["savePath"])
    if profile.get("vessel") != "STS" or profile.get("saveName") != "STS-campaign-72km-20261008":
        raise RuntimeError("The named original-STS campaign checkpoint is required")
    if hashlib.sha256(save.read_bytes()).hexdigest() != profile.get("saveSHA256"):
        raise RuntimeError("Checkpoint save hash does not match its certified profile")

    report = {
        "scope": "Vacuum yaw reaction-wheel pulse for vessel-frame rate crosscheck; not entry qualification or aerodynamic coordination evidence.",
        "profile": profile,
        "sourceSHA256": hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
        "samples": [],
    }
    conn = None
    guard = None
    ready_path = None
    args.output.parent.mkdir(parents=True, exist_ok=True)
    try:
        restore_quicksave(ROOT, profile["saveName"])
        prepared = subprocess.run(
            [sys.executable, str(ROOT / "Tools/settle_known_checkpoint.py"), "--profile", str(args.profile), "--save", profile["saveName"]],
            capture_output=True,
            text=True,
            timeout=30,
            check=True,
        )
        report["startup"] = json.loads(prepared.stdout.strip().splitlines()[-1])
        conn = connect("Original STS yaw frame crosscheck", timeout=8)
        sc = conn.space_center
        vessel = sc.active_vessel
        if vessel.name != "STS" or not conn.krpc.paused or abs(float(vessel.mass) - float(profile["mass"])) > 1:
            raise RuntimeError("Probe requires the frozen, full-mass original STS checkpoint")
        if abs(float(vessel.thrust)) > 1 or bool(vessel.control.sas) or bool(vessel.control.rcs):
            raise RuntimeError("Probe requires zero thrust with SAS and RCS off")
        flight = vessel.flight()
        if float(flight.mean_altitude) <= 70000 or float(flight.dynamic_pressure) > 1e-6:
            raise RuntimeError("Probe requires the checkpoint's vacuum state above 70 km")
        speed_brakes = getattr(vessel.control, "speed_brakes", None)
        if speed_brakes is not None and bool(speed_brakes):
            raise RuntimeError("Probe requires speedbrakes stowed")

        vessel.control.throttle = 0.0
        vessel.control.sas = False
        vessel.control.rcs = False
        vessel.control.yaw = 0.0
        frame = vessel.orbit.body.non_rotating_reference_frame
        vessel_frame = vessel.reference_frame
        report["preflight"] = {
            "ut": float(sc.ut),
            "vessel": vessel.name,
            "paused": bool(conn.krpc.paused),
            "meanAltitude": float(flight.mean_altitude),
            "dynamicPressure": float(flight.dynamic_pressure),
            "mass": float(vessel.mass),
            "dryMass": float(vessel.dry_mass),
            "partCount": len(vessel.parts.all),
            "thrust": float(vessel.thrust),
            "sas": bool(vessel.control.sas),
            "rcs": bool(vessel.control.rcs),
            "speedBrakes": speed_brakes,
            "controls": [float(vessel.control.pitch), float(vessel.control.roll), float(vessel.control.yaw)],
            "inertiaTensor": list(vessel.inertia_tensor),
            "momentOfInertia": list(vessel.moment_of_inertia),
            "reactionWheelTorque": vessel.available_reaction_wheel_torque,
            "rootInertiaTensor": list(vessel.parts.root.inertia_tensor),
            "rootRotationInVessel": list(vessel.parts.root.rotation(vessel_frame)),
            "rateFrameTransform": "SpaceCenter.transform_direction(angular_velocity_nonrotating, non_rotating_frame, vessel.reference_frame)",
        }

        definitions = {
            "ut": (getattr, sc, "ut"),
            "angularNonRotating": (vessel.angular_velocity, frame),
            "rotationNonRotating": (vessel.rotation, frame),
            "altitude": (getattr, flight, "mean_altitude"),
            "q": (getattr, flight, "dynamic_pressure"),
            "thrust": (getattr, vessel, "thrust"),
            "mass": (getattr, vessel, "mass"),
            "sas": (getattr, vessel.control, "sas"),
            "rcs": (getattr, vessel.control, "rcs"),
            "controlPitch": (getattr, vessel.control, "pitch"),
            "controlRoll": (getattr, vessel.control, "roll"),
            "controlYaw": (getattr, vessel.control, "yaw"),
        }
        streams = {key: conn.add_stream(*definition) for key, definition in definitions.items()}
        for stream in streams.values():
            stream.rate = 50
            stream.start()

        def read() -> dict:
            sample = {key: getter() for key, getter in streams.items()}
            sample["wallTime"] = time.monotonic()
            # Vessel.angular_velocity(vessel.reference_frame) is relative to a
            # frame that co-rotates with the vessel, so it collapses to zero.
            # Transforming the inertial vector changes its components without
            # subtracting that frame's angular velocity.
            transformed = sc.transform_direction(
                tuple(float(x) for x in sample["angularNonRotating"]), frame, vessel_frame
            )
            sample["angularVesselFrameTransformed"] = list(transformed)
            sample["vesselFrameRatesDegPerSecond"] = [math.degrees(float(x)) for x in transformed]
            sample["nonRotatingRatesDegPerSecond"] = [math.degrees(float(x)) for x in sample["angularNonRotating"]]
            if max(abs(x) for x in sample["vesselFrameRatesDegPerSecond"]) > float(config["guidance"]["entryRollRate"]):
                raise RuntimeError("Body angular-rate limit exceeded")
            if sample["altitude"] <= 70000 or sample["q"] > 1e-6 or abs(sample["thrust"]) > 1:
                raise RuntimeError("Probe left its vacuum, unpowered envelope")
            if sample["sas"] or sample["rcs"]:
                raise RuntimeError("SAS or RCS became active")
            return sample

        initial = read()
        start_ut = float(initial["ut"])
        if max(abs(x) for x in initial["vesselFrameRatesDegPerSecond"]) > 0.1:
            raise RuntimeError("Initial vessel-frame rate too large for a small pulse bench")
        ready_path = args.output.with_suffix(".guard-ready")
        ready_path.unlink(missing_ok=True)
        guard = subprocess.Popen(
            [sys.executable, str(Path(__file__).resolve()), "--watchdog", str(start_ut), str(os.getpid()), str(ready_path)],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
        )
        ready_deadline = time.monotonic() + 8
        while not ready_path.exists():
            if guard.poll() is not None or time.monotonic() > ready_deadline:
                raise RuntimeError("Independent probe guard did not become ready")
            time.sleep(0.02)

        previous = start_ut
        last_command = None
        deadline = time.monotonic() + 8
        conn.krpc.paused = False
        while time.monotonic() < deadline:
            sample = read()
            ut = float(sample["ut"])
            if ut <= previous:
                time.sleep(0.005)
                continue
            if ut - previous > 0.16:
                raise RuntimeError("Probe missed its observation interval")
            elapsed = ut - start_ut
            command = 0.0 if elapsed < 0.3 else 0.15 if elapsed < 0.7 else -0.15 if elapsed < 1.1 else 0.0
            sample["elapsed"] = elapsed
            sample["issuedYaw"] = command
            report["samples"].append(sample)
            if command != last_command:
                vessel.control.yaw = command
                last_command = command
            previous = ut
            if elapsed >= 1.5:
                report["completed"] = True
                break
        else:
            raise RuntimeError("Probe did not finish within its wall-clock deadline")
    except BaseException as exc:
        report["error"] = f"{type(exc).__name__}: {exc}"
        raise
    finally:
        try:
            if conn is not None:
                try:
                    conn.krpc.paused = True
                    if conn.space_center.active_vessel is not None:
                        conn.space_center.active_vessel.control.yaw = 0.0
                    report["finalState"] = {
                        "ut": float(conn.space_center.ut),
                        "paused": bool(conn.krpc.paused),
                        "vessel": conn.space_center.active_vessel.name if conn.space_center.active_vessel else None,
                        "mass": float(conn.space_center.active_vessel.mass) if conn.space_center.active_vessel else None,
                        "thrust": float(conn.space_center.active_vessel.thrust) if conn.space_center.active_vessel else None,
                        "sas": bool(conn.space_center.active_vessel.control.sas) if conn.space_center.active_vessel else None,
                        "rcs": bool(conn.space_center.active_vessel.control.rcs) if conn.space_center.active_vessel else None,
                        "controls": [float(conn.space_center.active_vessel.control.pitch), float(conn.space_center.active_vessel.control.roll), float(conn.space_center.active_vessel.control.yaw)] if conn.space_center.active_vessel else None,
                    }
                finally:
                    conn.close()
        finally:
            if guard is not None and guard.poll() is None:
                guard.terminate()
                guard.wait(timeout=5)
            if ready_path is not None:
                ready_path.unlink(missing_ok=True)
            try:
                report["cleanup"] = secure_failed_test_state(ROOT, profile["saveName"], True)
            finally:
                report["checkpointSHA256After"] = hashlib.sha256(save.read_bytes()).hexdigest()
                args.output.write_text(json.dumps(report, indent=2) + "\n")
    print(json.dumps({"completed": report.get("completed", False), "samples": len(report["samples"]), "output": str(args.output)}))


if __name__ == "__main__":
    if len(sys.argv) > 1 and sys.argv[1] == "--watchdog":
        watchdog(float(sys.argv[2]), int(sys.argv[3]), Path(sys.argv[4]))
    else:
        main()
