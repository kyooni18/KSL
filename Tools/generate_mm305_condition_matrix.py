#!/usr/bin/env python3
"""Compile reproducible MM305 ShuttleSim initial-condition matrices.

The MM305 Cartesian fixtures are authoritative: once position_* and
velocity_* are present, ShuttleSim ignores the nominal altitude/heading
fields.  This tool therefore derives both vectors from one explicit local
state description and records the derivation in a manifest.

Heading is the air-relative local-frame course in ShuttleSim's convention:
0 north, 90 east, 180 south, 270 west.  Speed is airspeed, not inertial
speed.  The generated inertial velocity includes Kerbin's atmosphere motion.
"""
from __future__ import annotations

import argparse
import itertools
import json
import math
from pathlib import Path
from typing import Any

RADIUS_M = 600_000.0
ROTATION_RATE_RAD_S = 2.0 * math.pi / 21549.425
ROTATION_PHASE_RAD_AT_UT0 = math.pi / 2.0


def load_ini(path: Path) -> dict[str, str]:
    values: dict[str, str] = {}
    for raw in path.read_text(encoding="utf-8").splitlines():
        line = raw.strip()
        if line and not line.startswith("#") and "=" in line:
            key, value = line.split("=", 1)
            values[key.strip()] = value.strip()
    return values


def f(values: dict[str, str], key: str) -> float:
    return float(values[key])


def norm(v: tuple[float, float, float]) -> float:
    return math.sqrt(sum(x * x for x in v))


def dot(a: tuple[float, float, float], b: tuple[float, float, float]) -> float:
    return sum(x * y for x, y in zip(a, b))


def scale(v: tuple[float, float, float], k: float) -> tuple[float, float, float]:
    return tuple(k * x for x in v)  # type: ignore[return-value]


def add(a: tuple[float, float, float], b: tuple[float, float, float]) -> tuple[float, float, float]:
    return tuple(x + y for x, y in zip(a, b))  # type: ignore[return-value]


def fixed_to_inertial(v: tuple[float, float, float], ut: float) -> tuple[float, float, float]:
    angle = ROTATION_PHASE_RAD_AT_UT0 + ROTATION_RATE_RAD_S * ut
    c, s = math.cos(angle), math.sin(angle)
    return (c * v[0] - s * v[1], s * v[0] + c * v[1], v[2])


def inertial_to_fixed(v: tuple[float, float, float], ut: float) -> tuple[float, float, float]:
    angle = -(ROTATION_PHASE_RAD_AT_UT0 + ROTATION_RATE_RAD_S * ut)
    c, s = math.cos(angle), math.sin(angle)
    return (c * v[0] - s * v[1], s * v[0] + c * v[1], v[2])


def lla_from_position(p_i: tuple[float, float, float], ut: float) -> tuple[float, float]:
    p = inertial_to_fixed(p_i, ut)
    r = norm(p)
    return math.asin(max(-1.0, min(1.0, p[2] / r))), math.atan2(p[1], p[0])


def position_from_lla(lat: float, lon: float, altitude: float, ut: float) -> tuple[float, float, float]:
    r = RADIUS_M + altitude
    fixed = (r * math.cos(lat) * math.cos(lon),
             r * math.cos(lat) * math.sin(lon),
             r * math.sin(lat))
    return fixed_to_inertial(fixed, ut)


def local_frame(p_i: tuple[float, float, float]) -> tuple[tuple[float, float, float], ...]:
    up = scale(p_i, 1.0 / norm(p_i))
    horizontal = math.hypot(up[0], up[1])
    east = (-up[1] / horizontal, up[0] / horizontal, 0.0)
    north = (-up[2] * east[1], up[2] * east[0], up[0] * east[1] - up[1] * east[0])
    return north, east, up


def inertial_velocity(p_i: tuple[float, float, float], ut: float, heading_deg: float,
                      airspeed_mps: float, fpa_deg: float) -> tuple[float, float, float]:
    north, east, up = local_frame(p_i)
    heading = math.radians(heading_deg)
    fpa = math.radians(fpa_deg)
    horizontal = airspeed_mps * math.cos(fpa)
    vertical = airspeed_mps * math.sin(fpa)
    air = add(add(scale(north, horizontal * math.cos(heading)),
                  scale(east, horizontal * math.sin(heading))), scale(up, vertical))
    atmosphere = (-ROTATION_RATE_RAD_S * p_i[1], ROTATION_RATE_RAD_S * p_i[0], 0.0)
    return add(air, atmosphere)


def fmt(x: float) -> str:
    return f"{x:.12g}"


def compile_case(base: dict[str, str], out: Path, heading: float, altitude: float,
                 speed: float, fpa: float, east_offset_m: float = 0.0,
                 north_offset_m: float = 0.0) -> dict[str, Any]:
    ut0 = f(base, "ut0")
    base_p = tuple(f(base, key) for key in ("position_x_m", "position_y_m", "position_z_m"))
    lat, lon = lla_from_position(base_p, ut0)
    p = position_from_lla(lat, lon, altitude, ut0)
    # Offsets are local tangent-plane displacements from the existing seed's
    # subpoint.  The runway/HAC geometry remains owned by the scenario's
    # runway fields; these offsets only vary the vehicle's initial condition.
    if east_offset_m or north_offset_m:
        north, east, _ = local_frame(p)
        p = add(p, add(scale(east, east_offset_m), scale(north, north_offset_m)))
        p = scale(p, (RADIUS_M + altitude) / norm(p))
    runway_lat = math.radians(f(base, "runway_latitude_deg"))
    runway_lon = math.radians(f(base, "runway_longitude_deg"))
    runway_p = position_from_lla(runway_lat, runway_lon, f(base, "runway_elevation_m"), ut0)
    runway_north, runway_east, runway_up = local_frame(runway_p)
    delta = tuple(a - b for a, b in zip(p, runway_p))
    runway_local_east_m = dot(delta, runway_east)
    runway_local_north_m = dot(delta, runway_north)
    runway_local_up_m = dot(delta, runway_up)
    v = inertial_velocity(p, ut0, heading, speed, fpa)
    slug = f"h{heading:g}-a{altitude:g}-v{speed:g}"
    if east_offset_m or north_offset_m:
        slug += f"-e{east_offset_m:g}-n{north_offset_m:g}"
    slug = slug.replace(".", "p")
    # Keep the existing MM305 fixture namespace.  simulator_campaign.py uses
    # this prefix to select the HAC-only terminal test harness; changing it
    # would silently turn a reproducibility case into a different experiment.
    name = f"mm305-hac-matrix-{slug}"
    values = dict(base)
    values.update({
        "name": name,
        "position_x_m": fmt(p[0]), "position_y_m": fmt(p[1]), "position_z_m": fmt(p[2]),
        "velocity_x_mps": fmt(v[0]), "velocity_y_mps": fmt(v[1]), "velocity_z_mps": fmt(v[2]),
    })
    order = list(base.keys())
    lines = [
        "# Generated by Tools/generate_mm305_condition_matrix.py.",
        f"# Condition: air-relative heading={heading:g} deg altitude={altitude:g} m "
        f"airspeed={speed:g} m/s FPA={fpa:g} deg "
        f"east_offset={east_offset_m:g} m north_offset={north_offset_m:g} m.",
        "# Cartesian position/velocity are authoritative in ShuttleSim.",
    ]
    for key in order:
        lines.append(f"{key}={values[key]}")
    out.write_text("\n".join(lines) + "\n", encoding="utf-8")
    return {
        "name": name, "scenario": str(out), "heading_deg": heading,
        "altitude_m": altitude, "airspeed_mps": speed, "fpa_deg": fpa,
        "east_offset_m": east_offset_m, "north_offset_m": north_offset_m,
        "runway_local_east_m": runway_local_east_m,
        "runway_local_north_m": runway_local_north_m,
        "runway_local_up_m": runway_local_up_m,
        "ut0": ut0, "position_i_m": list(p), "velocity_i_mps": list(v),
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--base", type=Path,
                        default=Path("ShuttleSim/scenarios/mm305-hac-staging-ideal-seed-10km-180mps.ini"))
    parser.add_argument("--output-dir", type=Path, default=Path("ShuttleSim/scenarios/mm305-condition-matrix"))
    parser.add_argument("--manifest", type=Path, default=None)
    parser.add_argument("--headings", default="175,180,185")
    parser.add_argument("--altitudes", default="9000,10000,11000")
    parser.add_argument("--speeds", default="170,180,190")
    parser.add_argument("--east-offsets-m", default="0",
                        help="comma-separated local east offsets from the seed subpoint")
    parser.add_argument("--north-offsets-m", default="0",
                        help="comma-separated local north offsets from the seed subpoint")
    parser.add_argument("--fpa-deg", type=float, default=-8.0)
    args = parser.parse_args()
    base = load_ini(args.base)
    headings = [float(x) for x in args.headings.split(",") if x.strip()]
    altitudes = [float(x) for x in args.altitudes.split(",") if x.strip()]
    speeds = [float(x) for x in args.speeds.split(",") if x.strip()]
    east_offsets = [float(x) for x in args.east_offsets_m.split(",") if x.strip()]
    north_offsets = [float(x) for x in args.north_offsets_m.split(",") if x.strip()]
    args.output_dir.mkdir(parents=True, exist_ok=True)
    cases = []
    for heading, altitude, speed, east_offset, north_offset in itertools.product(
            headings, altitudes, speeds, east_offsets, north_offsets):
        slug = f"h{heading:g}-a{altitude:g}-v{speed:g}"
        if east_offset or north_offset:
            slug += f"-e{east_offset:g}-n{north_offset:g}"
        slug = slug.replace(".", "p")
        # The campaign's existing MM305 opt-in is keyed to the scenario
        # filename, not the INI's name field.  Keep that established contract
        # explicit in generated artifacts so a matrix case cannot silently
        # run through the generic planner.
        cases.append(compile_case(base, args.output_dir / f"mm305-hac-repro-{slug}.ini",
                                  heading, altitude, speed, args.fpa_deg,
                                  east_offset, north_offset))
    manifest = {
        "schema": 1,
        "generator": "Tools/generate_mm305_condition_matrix.py",
        "base": str(args.base), "speed_definition": "total_air_speed_mps",
        "heading_definition": "air_relative_local_course_deg",
        "side": 1, "fpa_deg": args.fpa_deg,
        "headings_deg": headings, "altitudes_m": altitudes, "airspeeds_mps": speeds,
        "east_offsets_m": east_offsets, "north_offsets_m": north_offsets,
        "cases": cases,
    }
    manifest_path = args.manifest or args.output_dir / "manifest.json"
    manifest_path.write_text(json.dumps(manifest, indent=2) + "\n", encoding="utf-8")
    print(json.dumps({"manifest": str(manifest_path), "cases": len(cases)}, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
