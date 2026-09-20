#!/usr/bin/env python3
"""Decode delta-compressed KSPShuttleLander vehicle JSONL into ShuttleSim CSV.

The first four columns intentionally match ShuttleSim attitude replay:
  time_s,aoa_deg,bank_deg,gear_down

Remaining columns feed the aero/atmosphere/attitude fitters and preserve enough
state for checkpoint extraction and parity analysis.
"""
from __future__ import annotations
import argparse
import csv
import json
import math
from pathlib import Path
from typing import Any

R_AIR = 287.05


def deep_merge(dst: dict[str, Any], src: dict[str, Any]) -> None:
    for key, value in src.items():
        if isinstance(value, dict) and isinstance(dst.get(key), dict):
            deep_merge(dst[key], value)
        else:
            dst[key] = value


def get(d: dict[str, Any], *path: str, default=None):
    cur: Any = d
    for key in path:
        if not isinstance(cur, dict) or key not in cur:
            return default
        cur = cur[key]
    return cur


def vec_component(v: Any, i: int):
    return v[i] if isinstance(v, list) and len(v) > i else None


FIELDS = [
    "time_s", "aoa_deg", "bank_deg", "gear_down",
    "source_log", "ut", "record_type",
    "altitude_m", "latitude_deg", "longitude_deg",
    "position_x_m", "position_y_m", "position_z_m",
    "velocity_x_mps", "velocity_y_mps", "velocity_z_mps",
    "true_air_speed_mps", "horizontal_speed_mps", "vertical_speed_mps",
    "flight_path_angle_deg",
    "mach", "dynamic_pressure_pa", "density_kg_m3", "pressure_pa",
    "temperature_k", "speed_of_sound_mps",
    "pitch_deg", "heading_deg", "sideslip_deg",
    "pitch_rate_deg_s", "roll_rate_deg_s", "yaw_rate_deg_s",
    "lift_force_n", "drag_force_n", "lift_accel_mps2", "drag_accel_mps2",
    "mass_kg", "current_thrust_n", "available_thrust_n",
    "target_aoa_deg", "target_roll_deg", "target_pitch_deg",
    "target_heading_deg", "target_throttle", "brakes", "airbrakes",
]


def decode_log(path: Path, timeline_offset: float):
    state: dict[str, Any] = {}
    first_ut = None
    last_rel = 0.0
    with path.open("r", encoding="utf-8") as f:
        for line in f:
            try:
                rec = json.loads(line)
            except json.JSONDecodeError:
                continue
            fields = rec.get("fields")
            if not isinstance(fields, dict):
                continue
            rtype = str(rec.get("recordType", ""))
            if rtype == "vehicleKeyframe":
                state = json.loads(json.dumps(fields))
            elif rtype == "vehicleDelta":
                deep_merge(state, fields)
            else:
                continue

            ut = rec.get("ut")
            if not isinstance(ut, (int, float)) or not math.isfinite(float(ut)):
                continue
            if first_ut is None:
                first_ut = float(ut)
            rel = float(ut) - first_ut
            if rel < 0:
                continue
            last_rel = max(last_rel, rel)

            aero = state.get("aero", {})
            attitude = state.get("attitude", {})
            position = state.get("position", {})
            motion = state.get("motion", {})
            vehicle = state.get("vehicle", {})
            command = state.get("command", {})

            mass = vehicle.get("mass")
            lift = aero.get("liftForce")
            drag = aero.get("dragForce")
            rho = aero.get("density")
            pressure = aero.get("staticPressure")
            tas = motion.get("trueAirSpeed")
            mach = aero.get("mach")

            lift_acc = lift / mass if isinstance(lift, (int, float)) and isinstance(mass, (int, float)) and mass > 0 else None
            drag_acc = drag / mass if isinstance(drag, (int, float)) and isinstance(mass, (int, float)) and mass > 0 else None
            sound = tas / mach if isinstance(tas, (int, float)) and isinstance(mach, (int, float)) and mach > 1e-6 and isinstance(rho, (int, float)) and rho > 1e-12 else 0.0
            temp = pressure / (rho * R_AIR) if isinstance(pressure, (int, float)) and isinstance(rho, (int, float)) and rho > 1e-12 else 0.0

            pos = position.get("inertial")
            vel = motion.get("velocity")
            row = {
                "time_s": timeline_offset + rel,
                "aoa_deg": attitude.get("angleOfAttack"),
                "bank_deg": attitude.get("roll"),
                "gear_down": int(bool(command.get("gear", False))),
                "source_log": path.name,
                "ut": ut,
                "record_type": rtype,
                "altitude_m": position.get("altitude"),
                "latitude_deg": position.get("latitude"),
                "longitude_deg": position.get("longitude"),
                "position_x_m": vec_component(pos, 0),
                "position_y_m": vec_component(pos, 1),
                "position_z_m": vec_component(pos, 2),
                "velocity_x_mps": vec_component(vel, 0),
                "velocity_y_mps": vec_component(vel, 1),
                "velocity_z_mps": vec_component(vel, 2),
                "true_air_speed_mps": tas,
                "horizontal_speed_mps": motion.get("horizontalSpeed"),
                "vertical_speed_mps": motion.get("verticalSpeed"),
                "flight_path_angle_deg": motion.get("flightPathAngle"),
                "mach": mach,
                "dynamic_pressure_pa": aero.get("dynamicPressure"),
                "density_kg_m3": rho,
                "pressure_pa": pressure,
                "temperature_k": temp,
                "speed_of_sound_mps": sound,
                "pitch_deg": attitude.get("pitch"),
                "heading_deg": attitude.get("heading"),
                "sideslip_deg": attitude.get("sideslip"),
                "pitch_rate_deg_s": attitude.get("pitchRate"),
                "roll_rate_deg_s": attitude.get("rollRate"),
                "yaw_rate_deg_s": attitude.get("yawRate"),
                "lift_force_n": lift,
                "drag_force_n": drag,
                "lift_accel_mps2": lift_acc,
                "drag_accel_mps2": drag_acc,
                "mass_kg": mass,
                "current_thrust_n": vehicle.get("currentThrust"),
                "available_thrust_n": vehicle.get("availableThrust"),
                "target_aoa_deg": command.get("targetAoA"),
                "target_roll_deg": command.get("targetRoll"),
                "target_pitch_deg": command.get("targetPitch"),
                "target_heading_deg": command.get("targetHeading"),
                "target_throttle": command.get("targetThrottle"),
                "brakes": int(bool(command.get("brakes", False))),
                "airbrakes": int(bool(command.get("airbrakes", False))),
            }
            # Emit only after the first full state exists.
            if row["altitude_m"] is not None and row["aoa_deg"] is not None and row["bank_deg"] is not None:
                yield row
    return last_rel


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("inputs", nargs="+", help="vehicle JSONL logs")
    ap.add_argument("-o", "--output", required=True)
    args = ap.parse_args()

    paths = [Path(p) for p in args.inputs]
    missing = [str(p) for p in paths if not p.is_file()]
    if missing:
        raise SystemExit("missing input(s): " + ", ".join(missing))

    total = 0
    offset = 0.0
    with Path(args.output).open("w", newline="", encoding="utf-8") as out:
        writer = csv.DictWriter(out, fieldnames=FIELDS)
        writer.writeheader()
        for path in paths:
            rows = list(decode_log(path, offset))
            if not rows:
                continue
            for row in rows:
                writer.writerow(row)
            total += len(rows)
            offset = float(rows[-1]["time_s"]) + 5.0

    print(f"wrote {total} decoded samples from {len(paths)} log(s) to {args.output}")


if __name__ == "__main__":
    main()
