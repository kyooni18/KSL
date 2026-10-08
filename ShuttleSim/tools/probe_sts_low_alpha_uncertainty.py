#!/usr/bin/env python3
"""Finite STS polar sensitivity experiment, never a flight qualification."""
import argparse
import csv
import gzip
import hashlib
import json
import math
from pathlib import Path
import subprocess


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def snapshot(row):
    aero = row['aero']
    return dict(time=row['sim_time'], h=row['position']['altitude_m'],
                v=row['velocity']['air_mps'],
                fpa=row['velocity']['flight_path_angle_deg'],
                along=row['runway']['along_m'], cross=row['runway']['cross_m'],
                q=aero['q_pa'],
                g=math.hypot(aero['lift_n'], aero['drag_n']) /
                  (row['vehicle']['mass_kg'] * 9.80665))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--bank-authority', action='store_true',
                        help='Probe 18/28 deg AoA and 40/55/70 deg bank with original polar')
    args = parser.parse_args()
    root = Path(__file__).resolve().parents[2]
    out = args.output.resolve()
    out.mkdir(parents=True, exist_ok=True)
    source = root / 'ShuttleSim/reference-model/sts_aero_provisional.csv'
    inputs = [source, root / 'ShuttleSim/build/shuttlesim',
              root / 'ShuttleSim/scenarios/sts72km-run9-entry-interface.ini',
              root / 'ShuttleSim/reference-model/kerbin_atmosphere_identified.csv',
              root / 'ShuttleSim/reference-model/stsn_attitude_identified.ini']
    with source.open() as stream:
        rows = list(csv.DictReader(line for line in stream if not line.startswith('#')))
    results = []
    # Deliberately broad scenario brackets, not measured uncertainty bounds.
    variants = [('baseline', 1, 1)] if args.bank_authority else [
        ('baseline', 1, 1), ('lift-half', .5, 1), ('lift-double', 2, 1),
        ('drag-half', 1, .5), ('drag-double', 1, 2)]
    controls = [(a, b) for a in [18, 28] for b in [40, 55, 70]] if args.bank_authority else [(0, 40), (8, 40)]
    for name, lift, drag in variants:
        polar = out / f'{name}.csv'
        with polar.open('w') as stream:
            stream.write('# UNVALIDATED sensitivity polar; not for flight qualification.\n')
            writer = csv.DictWriter(stream, fieldnames=['mach', 'alpha_deg', 'cl', 'cd'])
            writer.writeheader()
            for row in rows:
                changed = dict(row)
                # Exact original strings above 14 deg, below all reported support.
                weight = max(0, 1 - abs(float(row['alpha_deg'])) / 14) ** 2
                for field, scale in [('cl', lift), ('cd', drag)]:
                    if weight and scale != 1:
                        changed[field] = f'{float(row[field]) * (1 + (scale - 1) * weight):.8f}'
                writer.writerow(changed)
        for alpha, bank in controls:
            raw = out / f'{name}-aoa{alpha}-bank{bank}.jsonl'
            command = [str(inputs[1]), '--scenario', str(inputs[2]),
                       '--atmosphere', str(inputs[3]), '--aero', str(polar),
                       '--attitude', str(inputs[4]), '--fixed-aoa', str(alpha),
                       '--fixed-bank', str(bank), '--max-sim-time', '1400', '--rate', 'max',
                       '--command-port', '0', '--telemetry-port', '0',
                       '--web-telemetry-port', '0', '--record', str(raw), '--quiet']
            process = subprocess.run(command, capture_output=True, text=True, check=True)
            result = dict(polar=name, aoa=alpha, bank=bank, command=command,
                          polarSha256=digest(polar), rawSha256=digest(raw),
                          returncode=process.returncode, peakQ=0, peakG=0)
            with raw.open() as stream, gzip.open(str(raw) + '.gz', 'wt') as archive:
                for line in stream:
                    archive.write(line)
                    row = json.loads(line)
                    if row.get('type') != 'telemetry':
                        continue
                    state = snapshot(row)
                    result['peakQ'] = max(result['peakQ'], state['q'])
                    result['peakG'] = max(result['peakG'], state['g'])
                    for key, condition in [('at25km', state['h'] <= 25000),
                                           ('at860mps', state['v'] <= 860)]:
                        if condition and key not in result:
                            result[key] = state
                    result['final'] = state
            raw.unlink()
            result['retainedLog'] = str(raw) + '.gz'
            results.append(result)
            print(name, alpha, bank, json.dumps(result.get('at25km')), flush=True)
    report = dict(scope='Unvalidated low-alpha model sensitivity, fixed controls; '
                        'not guidance, gate admission, or evidence for relaxing AoA limits. '
                        'Constant bank may exceed low-q guidance authority. '
                        'Borrowed STS-N attitude dynamics remain provisional.',
                  perturbation='CL or CD scaled below |alpha|=14 deg with '
                               'weight=(1-|alpha|/14)^2; measured support unchanged. '
                               'Multipliers 0.5 and 2 are scenario brackets, not confidence bounds.',
                  inputs=[dict(path=str(p), sha256=digest(p)) for p in inputs],
                  bankAuthorityProbe=args.bank_authority, results=results)
    (out / 'summary.json').write_text(json.dumps(report, indent=2) + '\n')


if __name__ == '__main__':
    main()
