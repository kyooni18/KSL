#!/usr/bin/env python3
"""Generate MM305 HAC fixtures on a ring around RW09 at fixed bearing steps.

Each state sits RANGE metres from the runway threshold at the given bearing,
flying straight at the threshold, with the same altitude/speed/FPA, so only the
approach geometry changes between runs.
"""
from __future__ import annotations
import argparse, math
from pathlib import Path

RADIUS = 600000.0
OMEGA = 2.0 * math.pi / 21549.425
LAT, LON, UT = -0.0486111111, -74.7283333333, 66564.2698140202


def write(path: Path, bearing: float, rng: float, alt: float, speed: float, fpa: float, aoa: float) -> None:
    lat0, lon0 = math.radians(LAT), math.radians(LON)
    d = rng / RADIUS; b = math.radians(bearing)
    lat = math.asin(math.sin(lat0)*math.cos(d) + math.cos(lat0)*math.sin(d)*math.cos(b))
    lon = lon0 + math.atan2(math.sin(b)*math.sin(d)*math.cos(lat0), math.cos(d)-math.sin(lat0)*math.sin(lat))
    theta = lon + math.pi/2 + OMEGA*UT
    up = (math.cos(lat)*math.cos(theta), math.cos(lat)*math.sin(theta), math.sin(lat))
    north = (-math.sin(lat)*math.cos(theta), -math.sin(lat)*math.sin(theta), math.cos(lat))
    east = (-math.sin(theta), math.cos(theta), 0.0)
    course = math.radians((bearing + 180.0) % 360.0)
    g = math.radians(fpa)
    pos = tuple((RADIUS+alt)*u for u in up)
    surf = tuple(speed*(math.cos(g)*(math.sin(course)*e + math.cos(course)*n) + math.sin(g)*u)
                 for e, n, u in zip(east, north, up))
    vel = tuple(s+r for s, r in zip(surf, (-OMEGA*pos[1], OMEGA*pos[0], 0.0)))
    v = dict(name=path.stem, ut0=UT)
    v.update({f'position_{a}_m': x for a, x in zip('xyz', pos)})
    v.update({f'velocity_{a}_mps': x for a, x in zip('xyz', vel)})
    v.update(mass_kg=40252.91796875, initial_aoa_deg=aoa, initial_bank_deg=0,
             deorbit_delta_v_mps=0, deorbit_duration_s=0, deorbit_delay_s=0,
             runway_latitude_deg=LAT, runway_longitude_deg=LON, runway_elevation_m=70,
             runway_heading_deg=90, runway_length_m=2500, runway_width_m=70)
    path.write_text(f'# HAC bearing sweep: bearing={bearing} range={rng} alt={alt} speed={speed} fpa={fpa}; course toward threshold.\n'
                    + ''.join(f'{k}={x:.17g}\n' if isinstance(x, float) else f'{k}={x}\n' for k, x in v.items()))


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument('--out', default=str(Path(__file__).resolve().parents[1]/'ShuttleSim/scenarios/hac-sweep'))
    ap.add_argument('--bearings', default='220,240,260,280')
    ap.add_argument('--range', type=float, default=18000.0)
    ap.add_argument('--altitude', type=float, default=12000.0)
    ap.add_argument('--speed', type=float, default=220.0)
    ap.add_argument('--fpa', type=float, default=-5.0)
    ap.add_argument('--aoa', type=float, default=10.0)
    a = ap.parse_args()
    out = Path(a.out); out.mkdir(parents=True, exist_ok=True)
    for b in (float(x) for x in a.bearings.split(',')):
        p = out/f'hac-sweep-b{int(b):03d}.ini'
        write(p, b, a.range, a.altitude, a.speed, a.fpa, a.aoa); print(p)


if __name__ == '__main__':
    main()
