#!/usr/bin/env python3
"""Inspect simulator MM305 physical/closure windows; never certify admission."""
import argparse
import gzip
import hashlib
import json
import math
from pathlib import Path


def analyze(path, configuration):
    guidance = configuration['guidance']
    vehicle = configuration['vehicle']
    site = configuration['site']
    states = []
    count = 0
    with gzip.open(path, 'rt') as stream:
        for line in stream:
            row = json.loads(line)
            if row.get('type') != 'telemetry':
                continue
            count += 1
            position, velocity, aero = row['position'], row['velocity'], row['aero']
            if not (guidance['mm305MinimumAltitude'] <= position['altitude_m'] <= guidance['mm305MaximumAltitude']
                    and abs(aero['mach'] - guidance['mm305TargetMach']) <= guidance['mm305MachHalfWidth']
                    and velocity['air_mps'] >= vehicle['minimumSafeSpeed']
                    and velocity['vertical_mps'] < 0):
                continue
            g = math.hypot(aero['lift_n'], aero['drag_n']) / (row['vehicle']['mass_kg'] * 9.80665)
            if aero['q_pa'] > vehicle['maximumDynamicPressure'] or g > vehicle['maximumGLoad']:
                continue
            lat, target_lat = map(math.radians, (position['lat_deg'], site['latitude']))
            dlon = math.radians(site['longitude'] - position['lon_deg'])
            bearing = math.degrees(math.atan2(math.sin(dlon) * math.cos(target_lat),
                math.cos(lat) * math.sin(target_lat) - math.sin(lat) * math.cos(target_lat) * math.cos(dlon)))
            error = math.remainder(bearing - row['attitude']['heading_deg'], 360)
            cosine = math.sin(lat) * math.sin(target_lat) + math.cos(lat) * math.cos(target_lat) * math.cos(dlon)
            distance = row['world']['radius_m'] * math.acos(max(-1, min(1, cosine)))
            states.append(dict(ut=row['ut'], altitude_m=position['altitude_m'], mach=aero['mach'],
                range_m=distance, along_m=row['runway']['along_m'], cross_m=row['runway']['cross_m'],
                heading_deg=row['attitude']['heading_deg'], bearing_deg=bearing % 360,
                course_error_deg=error, closure_margin_deg=90-abs(error),
                range_margin_m=guidance['taemInterfaceRange']-distance))
    in_range = [s for s in states if s['range_margin_m'] >= 0]
    return dict(log=str(path), sha256=hashlib.sha256(path.read_bytes()).hexdigest(), telemetry_count=count,
        physical_window_frames=len(states), in_range_window_frames=len(in_range),
        closing_in_range_frames=sum(s['closure_margin_deg'] >= 0 for s in in_range),
        best_closure_in_range=max(in_range, key=lambda s:s['closure_margin_deg'], default=None),
        first_physical_window=states[0] if states else None)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('logs', nargs='+', type=Path)
    parser.add_argument('--configuration', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    config = json.loads(args.configuration.read_text())
    if config['vehicle'].get('modelId') != 'STS':
        raise ValueError('This diagnostic requires original STS configuration')
    report = dict(scope='Partial simulator physical/closure diagnostic. Uses simulator heading as course for these zero-sideslip, windless probes. Does not evaluate energy, stall, control authority, executive state, or native qualification. Finite cases are not a reachability proof.',
        configuration=str(args.configuration), configuration_sha256=hashlib.sha256(args.configuration.read_bytes()).hexdigest(),
        results=[analyze(p,config) for p in args.logs])
    args.output.write_text(json.dumps(report,indent=2)+'\n')
    for r in report['results']:
        best = r['best_closure_in_range']
        print(Path(r['log']).name, r['physical_window_frames'], r['closing_in_range_frames'],
              'best closure margin', best['closure_margin_deg'] if best else None)


if __name__ == '__main__':
    main()
