#!/usr/bin/env python3
"""Recorded kinematic convention and independent-source coverage audit only."""
import hashlib
import json
import math
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
PRIMARY = ROOT/'runs/20261102T152517Z-dessert-150ms-live/live.jsonl'


def multiply(a, b):
    w, x, y, z = a
    W, X, Y, Z = b
    return [w*W-x*X-y*Y-z*Z, w*X+x*W+y*Z-z*Y,
            w*Y-x*Z+y*W+z*X, w*Z+x*Y-y*X+z*W]


def quaternion(s):
    return [s['attitude'][k] for k in ('w', 'x', 'y', 'z')]


def primary_interval():
    rows = []
    seed = None
    for line in PRIMARY.open():
        r = json.loads(line)
        if r.get('orbitalCommand') is not None:
            continue
        if seed is None and r['guidance']['phase'] == 'acquisition':
            seed = r
            config = json.loads(PRIMARY.with_suffix('.jsonl.configuration.json').read_text())
            horizon = max(config['guidance']['pathLookaheadTime'],
                          r['control']['rollResponseSeconds'], r['control']['pitchResponseSeconds'])
        if seed is not None:
            rows.append(r)
            if r['state']['time']-seed['state']['time'] >= horizon:
                break
    return rows


def conventions(rows):
    errors, reversed_errors = [[], [], []], [[], [], []]
    for a, b in zip(rows, rows[1:]):
        a, b = a['state'], b['state']
        dt = b['time']-a['time']
        if not 0 < dt <= 1:
            raise ValueError('incoherent sample interval')
        qa, qb = quaternion(a), quaternion(b)
        delta = multiply([qa[0]]+[-x for x in qa[1:]], qb)
        if delta[0] < 0:
            delta = [-x for x in delta]
        norm = math.sqrt(sum(x*x for x in delta[1:]))
        theta = 2*math.atan2(norm, delta[0])
        rate = [x*theta/(norm*dt) for x in delta[1:]] if norm else [0, 0, 0]
        for i, axis in enumerate(('x', 'y', 'z')):
            observed = (a['angularVelocity'][axis]+b['angularVelocity'][axis])/2
            errors[i].append(rate[i]-observed)
            reversed_errors[i].append(rate[i]+observed)
    rms = lambda x: math.sqrt(sum(v*v for v in x)/len(x))
    return {'intervals': len(rows)-1, 'rateResidualRMS_rad_per_second_FRD': list(map(rms, errors)),
            'oppositeSignResidualRMS_rad_per_second_FRD': list(map(rms, reversed_errors)),
            'method': 'Shortest quaternion log increment compared to adjacent endpoint-mean body rates.',
            'limitation': 'Right-endpoint versus midpoint body-coordinate transport and snapshot timing add local residual; no moment law is inferred.'}


def independent(path):
    digest = hashlib.sha256()
    n, first = 0, None
    ranges = {key: [math.inf, -math.inf] for key in ('mach', 'dynamicPressure', 'angleOfAttack')}
    for line in path.open('rb'):
        digest.update(line)
        r = json.loads(line)
        s = r.get('state')
        if not s or r.get('orbitalCommand') is not None or s['systems']['rcs']:
            continue
        if r['guidance']['phase'] not in ('acquisition', 'headingAlignment'):
            continue
        n += 1
        if first is None:
            first = {'time': s['time'], 'mass': s['mass'], 'vesselConfiguration': s.get('vesselConfiguration'),
                     'mach': s['aerodynamics']['mach'], 'q': s['aerodynamics']['dynamicPressure'],
                     'alpha': s['aerodynamics']['angleOfAttack']}
        for key in ranges:
            value = s['aerodynamics'][key]
            ranges[key] = [min(ranges[key][0], value), max(ranges[key][1], value)]
    return {'path': str(path.relative_to(ROOT)), 'sha256': digest.hexdigest(),
            'RCSOffTAEMSamples': n, 'first': first, 'marginalRanges': ranges if n else None,
            'limitation': 'Marginal ranges do not prove joint coverage or compatible control-surface history.'}


if __name__ == '__main__':
    rows = primary_interval()
    sources = [('20261102T122809Z-dessert-normal-oms-restart', 'live.jsonl'),
               ('20261102T144457Z-dessert-handoff-live', 'live.jsonl'),
               ('20261102T140105Z-dessert-recovery-fix-live', 'live.jsonl'),
               ('20261102T031333-taem-live-72km', 'live-sts-flight.jsonl')]
    grid_path = ROOT/'Docs/Experiments/STS-Entry-20261009/live-aero-mimo-envelope-grid.json'
    grid = json.loads(grid_path.read_text())
    print(json.dumps({'evidence': 'recorded-state-convention-and-existing-source-coverage-audit',
          'primary': str(PRIMARY.relative_to(ROOT)), 'seedTime': rows[0]['state']['time'],
          'primaryPartCount': rows[0]['state']['vesselConfiguration']['partCount'],
          'conventions': conventions(rows),
          'independentSources': [independent(ROOT/'runs'/run/name) for run, name in sources],
          'oldStaticGrid': {'path': str(grid_path.relative_to(ROOT)),
              'partCount': grid['liveStateBefore']['part_count'], 'qualification': grid['qualification'],
              'decision': 'Different topology and hypothetical shadow kinematics; no substitution as current measured moment model.'},
          'decision': 'Rate signs agree with measured rotation. Selected independent RCS-off flights do not cover seed Mach 2.85. Broader archives are not declared exhausted.'},
          indent=2, sort_keys=True, allow_nan=False))
