#!/usr/bin/env python3
"""Compare schedule-forecast force/q with plant samples matched in Mach and AoA.

PRED_AERO D/L are accelerations; simulator drag_n/lift_n are forces. Atmosphere
comparisons match altitude separately; latitude/UT may differ between paths.
"""
import argparse
import gzip
import json
import math
import re
import statistics
from collections import defaultdict
from pathlib import Path


def telemetry(path):
    opener = gzip.open if path.suffix == '.gz' else open
    with opener(path, 'rt') as stream:
        for line in stream:
            row = json.loads(line)
            if row.get('type') == 'telemetry':
                yield row


def compare(trace, plant):
    rows = list(telemetry(plant))
    cells = defaultdict(list)
    for row in rows:
        mach = row['aero']['mach']
        alpha = row['attitude']['aoa_deg']
        if 2.4 <= mach <= 5.1:
            cells[(round(mach * 10), round(alpha * 2))].append(row)
    matches = defaultdict(list)
    density = defaultdict(list)
    by_height = {}
    for height in (24000, 28000, 32000, 40000, 48000, 52000):
        by_height[height] = min(rows, key=lambda r: abs(r['position']['altitude_m'] - height))
    with trace.open() as stream:
        for line in stream:
            if not line.startswith('PRED_AERO '):
                continue
            p = {k: float(v) for k, v in re.findall(r'(\w+)=([-+\deE.]+)', line)}
            height = round(p['h'] / 4000) * 4000
            if height in by_height and abs(p['h'] - height) < 100:
                density[height].append(p['rho'] / by_height[height]['atmosphere']['density_kg_m3'])
            if not 2.5 <= p['mach'] <= 5 or p['q'] <= 10:
                continue
            m, a = round(p['mach'] * 10), round(p['aoa'] * 2)
            nearby = [r for dm in (-1, 0, 1) for da in (-1, 0, 1)
                      for r in cells[(m + dm, a + da)]
                      if abs(r['aero']['mach'] - p['mach']) <= .05
                      and abs(r['attitude']['aoa_deg'] - p['aoa']) <= .25]
            if not nearby:
                continue
            r = min(nearby, key=lambda r: ((r['aero']['mach'] - p['mach']) / .05)**2
                    + ((r['attitude']['aoa_deg'] - p['aoa']) / .25)**2)
            aero = r['aero']
            mass = r['vehicle']['mass_kg']
            key = (math.floor(p['mach'] * 2) / 2, round(p['aoa']))
            matches[key].append((p['D'] * mass / p['q'] / (aero['drag_n'] / aero['q_pa']),
                                 p['L'] * mass / p['q'] / (aero['lift_n'] / aero['q_pa'])))
    med = statistics.median
    return {
        'trace': str(trace), 'plant': str(plant),
        'matching_tolerance': {'mach': .05, 'aoa_deg': .25},
        'force_ratios': [dict(mach_lo=m, aoa_deg=a, matches=len(values),
                              drag_ratio=med(x[0] for x in values),
                              lift_ratio=med(x[1] for x in values))
                         for (m, a), values in sorted(matches.items())],
        'density_ratios_at_altitude': {str(h): med(values) for h, values in sorted(density.items())},
        'plant_schedule': [dict(mach=target, ut=r['ut'], altitude_m=r['position']['altitude_m'],
                                aoa_deg=r['attitude']['aoa_deg'], command_aoa_deg=r['attitude']['cmd_aoa_deg'])
                           for target in (5, 4.5, 4, 3.5, 3, 2.5)
                           for r in [min(rows, key=lambda r: abs(r['aero']['mach'] - target))]],
    }


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('trace', type=Path)
    parser.add_argument('plant', type=Path)
    args = parser.parse_args()
    print(json.dumps(compare(args.trace, args.plant), indent=2))
