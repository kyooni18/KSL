#!/usr/bin/env python3
"""One prehandoff local moment identifiability check; no trajectory simulation.

The measured-axis command is not individual surface deflection, so the fitted
residual includes unresolved actuator error. Never use this as a flight model.
Only the first TAEM response interval is held out; no coefficient is fit to it.
"""
import argparse
import json
import math
import sys
from pathlib import Path


def vector(v):
    return [v[k] for k in ('x', 'y', 'z')]


def cross(a, b):
    return [a[1]*b[2]-a[2]*b[1], a[2]*b[0]-a[0]*b[2], a[0]*b[1]-a[1]*b[0]]


def dot(a, b):
    return sum(x*y for x, y in zip(a, b))


def rotate_conjugate(q, v):
    axis = [-q[k] for k in ('x', 'y', 'z')]
    t = [2*x for x in cross(axis, v)]
    return [v[i]+q['w']*t[i]+cross(axis, t)[i] for i in range(3)]


def known_pitch(s, c):
    a = s['controlAuthority']
    inertia = vector(a['inertia'])
    w = vector(s['angularVelocity'])
    frame_rate = rotate_conjugate(s['attitude'], [0, 0, c['planet']['rotationRate']])
    inertial = [x+y for x, y in zip(w, frame_rate)]
    gyro = cross(inertial, [x*y for x, y in zip(inertia, inertial)])
    authority = min(abs(a['positiveTorque']['y']), abs(a['negativeTorque']['y']))/inertia[1]
    controls = s['appliedControls']
    control = controls['pitch']+controls.get('bodyFlapTrimFraction', 0)*c['vehicle'].get('bodyFlapTrimInputAtFullDeflection', 0)
    return authority*control-gyro[1]/inertia[1]+cross(w, frame_rate)[1]


def fit(rows, y):
    # Twice-reorthogonalized QR on normalized columns. Rank tolerance is
    # floating-point resolution only; it is not a physical acceptance gate.
    columns = list(map(list, zip(*rows)))
    scales = [math.sqrt(dot(x, x)) for x in columns]
    if any(x == 0 for x in scales):
        raise ValueError('unexcited feature')
    columns = [[v/s for v in col] for col, s in zip(columns, scales)]
    basis, r = [], [[0.0]*len(columns) for _ in columns]
    for j, col in enumerate(columns):
        v = col[:]
        for _ in range(2):
            for i, q in enumerate(basis):
                projection = dot(q, v)
                r[i][j] += projection
                v = [a-projection*b for a, b in zip(v, q)]
        r[j][j] = math.sqrt(dot(v, v))
        if r[j][j] <= sys.float_info.epsilon*max(len(rows), len(columns)):
            raise ValueError('numerically rank-deficient local design')
        basis.append([x/r[j][j] for x in v])
    b = [dot(q, y) for q in basis]
    coefficients = [0.0]*len(columns)
    for i in reversed(range(len(columns))):
        coefficients[i] = (b[i]-sum(r[i][j]*coefficients[j] for j in range(i+1, len(columns))))/r[i][i]
    return [x/s for x, s in zip(coefficients, scales)], [r[i][i] for i in range(len(columns))]


def run(log, configuration):
    c = json.loads(Path(configuration).read_text())
    records = []
    seed = None
    end = None
    for line in Path(log).open():
        r = json.loads(line)
        if r.get('orbitalCommand') is not None:
            continue
        records.append(r)
        if seed is None and r['guidance']['phase'] in ('acquisition', 'headingAlignment'):
            seed = r
            response = max(c['guidance']['pathLookaheadTime'],
                           r['control']['rollResponseSeconds'], r['control']['pitchResponseSeconds'])
        if seed is not None and r['state']['time']-seed['state']['time'] >= response:
            end = r
            break
    if end is None:
        raise ValueError('no complete first-TAEM interval')
    t0 = seed['state']['time']
    a0 = seed['state']['aerodynamics']
    rate0 = seed['state']['angularVelocity']['y']/a0['airspeed']
    def features(s):
        a = s['aerodynamics']
        q = a['dynamicPressure']/a0['dynamicPressure']
        return [q, q*(a['angleOfAttack']-a0['angleOfAttack']), q*(a['mach']-a0['mach']),
                q*(s['angularVelocity']['y']/a['airspeed']-rate0)]
    train, held = [], []
    for previous, r in zip(records, records[1:]):
        p, s = previous['state'], r['state']
        dt = s['time']-p['time']
        if not 0 < dt <= 1:
            raise ValueError('missing history')
        if s['time'] < t0-response:
            continue
        # Right-endpoint local derivative, consistent for train/holdout;
        # no future sample is needed to form the seed residual.
        value = (s['angularVelocity']['y']-p['angularVelocity']['y'])/dt-known_pitch(s, c)
        sample = (features(s), value, r)
        (train if s['time'] <= t0 else held).append(sample)
    coefficients, qr_diagonal = fit([x for x, _, _ in train], [y for _, y, _ in train])
    rms = lambda values: math.sqrt(sum(x*x for x in values)/len(values))
    constant = train[-1][1]
    bounds = {}
    for name in ('angleOfAttack', 'mach', 'dynamicPressure'):
        ts = [r['state']['aerodynamics'][name] for _, _, r in train]
        hs = [r['state']['aerodynamics'][name] for _, _, r in held]
        bounds[name] = {'training': [min(ts), max(ts)], 'heldOut': [min(hs), max(hs)],
                        'heldOutOutsideTraining': sum(x < min(ts) or x > max(ts) for x in hs)}
    return {'evidence': 'single-prehandoff-local-moment-regression-and-held-out-measured-state-residual-check',
            'seedTime': t0, 'responseInterval': response, 'trainingSamples': len(train),
            'heldOutSamples': len(held), 'features': ['q/q_seed', 'q/q_seed * delta_alpha_rad',
              'q/q_seed * delta_mach', 'q/q_seed * delta_pitch_rate_over_airspeed'],
            'coefficients': coefficients, 'normalizedQRDiagonal': qr_diagonal,
            'trainingResidualRMS': rms([dot(x, coefficients)-y for x, y, _ in train]),
            'heldOutResidualRMS': rms([dot(x, coefficients)-y for x, y, _ in held]),
            'constantSeedResidualHeldOutRMS': rms([constant-y for _, y, _ in held]),
            'support': bounds, 'limitations': [
              'Residual target is rad/s^2; regression is a diagnostic, not a validated restoring-moment model.',
              'Correlated alpha/Mach/time and finite-difference measurement noise limit identifiability.',
              'Held-out measured states are comparison inputs, not simulated future states; this is not trajectory closure.',
              'RCS changes across handoff; surface/wheel/RCS actual deflections and input response remain unresolved.',
              'No fitted coefficient is enabled in production guidance or used for an alternative trajectory.']}


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--input', required=True)
    parser.add_argument('--configuration', required=True)
    args = parser.parse_args()
    print(json.dumps(run(args.input, args.configuration), indent=2, sort_keys=True))
