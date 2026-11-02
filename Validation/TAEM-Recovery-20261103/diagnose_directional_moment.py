#!/usr/bin/env python3
"""Recorded-state, fixed-anchor pitch-moment hypothesis check; no simulation.

Available torque is not a measured control derivative. Neither hypothesis is
validated by this diagnostic, which uses actual postbranch rates/axis readbacks.
"""
import argparse
import hashlib
import json
import math
from pathlib import Path
from identify_pitch_support import vector, cross, rotate_conjugate


def run(log, configuration, baseline):
    c = json.loads(Path(configuration).read_text())
    reference = json.loads(Path(baseline).read_text())
    t0 = reference['seedTime']
    t1 = t0+reference['duration']
    digest = hashlib.sha256()
    rows = []
    for line in Path(log).open('rb'):
        digest.update(line)
        row = json.loads(line)
        if row.get('orbitalCommand') is not None:
            continue
        time = row['state']['time']
        if time >= t0:
            rows.append(row)
        if time >= t1:
            break
    if not rows or rows[0]['state']['time'] != t0 or rows[-1]['state']['time'] != t1:
        raise ValueError('Exact baseline interval not present')
    seed = rows[0]['state']
    if seed['systems']['rcs']:
        raise ValueError('Requires the recorded RCS-off supported seed')
    anchor = seed['controlAuthority']
    inertia = vector(anchor['inertia'])
    positive = abs(anchor['positiveTorque']['y'])/inertia[1]
    negative = abs(anchor['negativeTorque']['y'])/inertia[1]
    conservative = min(positive, negative)
    balance = reference['seedPitchBalance']
    factor = c['vehicle'].get('bodyFlapTrimInputAtFullDeflection', 0)

    def control(s):
        a = s['appliedControls']
        return a['pitch']+a.get('bodyFlapTrimFraction', 0)*factor

    def actuator(u):
        return (positive if u >= 0 else negative)*u

    def prediction(s, directional):
        w = vector(s['angularVelocity'])
        frame = rotate_conjugate(s['attitude'], [0, 0, c['planet']['rotationRate']])
        inertial = [a+b for a, b in zip(w, frame)]
        gyro = cross(inertial, [a*b for a, b in zip(inertia, inertial)])
        # Fixed seed torque/inertia and original prebranch balance, no fitting.
        command = actuator(control(s))-actuator(balance) if directional else conservative*(control(s)-balance)
        return command-gyro[1]/inertia[1]+cross(w, frame)[1]

    samples = []
    for a, b in zip(rows, rows[1:]):
        x, y = a['state'], b['state']
        dt = y['time']-x['time']
        if not 0 < dt <= 1 or b['sequence'] != a['sequence']+1:
            raise ValueError('Missing/reordered interval')
        if x['systems']['rcs'] or y['systems']['rcs']:
            raise ValueError('RCS mode changes inside diagnostic interval')
        observed = (y['angularVelocity']['y']-x['angularVelocity']['y'])/dt
        samples.append({'duration': dt, 'end': y['time']-t0,
                        'symmetric': (prediction(x, False)+prediction(y, False))/2-observed,
                        'directional': (prediction(x, True)+prediction(y, True))/2-observed})

    def summary(part):
        total = sum(s['duration'] for s in part)
        def stats(name):
            return {'timeWeightedRMSRadPerSec2': math.sqrt(sum(s[name]**2*s['duration'] for s in part)/total),
                    'timeWeightedBiasRadPerSec2': sum(s[name]*s['duration'] for s in part)/total}
        return {'intervals': len(part), 'duration': total,
                'symmetricMinimum': stats('symmetric'), 'directionalAvailableTorque': stats('directional')}
    cutoff = reference['firstMomentSupportExit']
    return {'source': str(Path(log)), 'consumedPrefixSHA256': digest.hexdigest(),
            'configurationSHA256': hashlib.sha256(Path(configuration).read_bytes()).hexdigest(),
            'baseline': str(Path(baseline)),
            'baselineSHA256': hashlib.sha256(Path(baseline).read_bytes()).hexdigest(),
            'firstSequence': rows[0]['sequence'], 'lastSequence': rows[-1]['sequence'], 'seedTime': t0, 'prebranchBalance': balance,
            'positivePitchAcceleration': positive, 'negativePitchAcceleration': negative,
            'positiveNegativeRatio': positive/negative,
            'appliedEquivalentPitchRange': [min(control(r['state']) for r in rows), max(control(r['state']) for r in rows)],
            'beforeRecordedSupportExit': summary([s for s in samples if s['end'] < cutoff]),
            'fullDiagnosticInterval': summary(samples),
            'limitations': ['One-step recorded-state closure, not an open-loop trajectory prediction.',
                           'Actual postbranch axis readbacks and body rates are used by both hypotheses; no actual individual surface angle is available.',
                           'Available torque is not a measured control derivative; directional hypothesis is not deployed.',
                           'Seed torque, inertia and prebranch balance are frozen, with no fitted postbranch coefficients.',
                           'Full diagnostic interval extends beyond original measured moment support; no validation or flight-readiness claim.']}


if __name__ == '__main__':
    p = argparse.ArgumentParser()
    p.add_argument('log'); p.add_argument('configuration'); p.add_argument('baseline')
    args = p.parse_args()
    print(json.dumps(run(args.log, args.configuration, args.baseline), indent=2, allow_nan=False))
