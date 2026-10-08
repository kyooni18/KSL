#!/usr/bin/env python3
"""Build an explicitly provisional STS coefficient basis from archived live logs.

One flight cannot identify a two-dimensional polar. Assume normal-force
CL=K*sin(alpha)*cos(alpha), CD=CD0+K*sin(alpha)**2 in each Mach bin.
Robust medians estimate K and the residual axial drag. Alpha and Mach outside
observed support are extrapolation, never certified cells. No STS-N aero is used.
"""
import argparse
import csv
import hashlib
import json
import math
from pathlib import Path
import statistics
from normalize_ksp_vehicle_log import deep_merge

BINS = [(0, .5), (.5, 1), (1, 1.5), (1.5, 2), (2, 3),
        (3, 4), (4, 5), (5, 6), (6, 7), (7, 8)]
ALPHAS = [-10, -5, -2, 0, 2, 4, 6, 8, 10, 12, 14, 16, 18, 20,
          22, 25, 28, 30, 35, 40, 45]


def samples(path):
    state = {}
    seen = set()
    with path.open() as f:
        header = json.loads(next(f))
        if header.get('vessel') != 'STS':
            raise ValueError(f'{path}: requires exact STS vessel identity')
        if 'shuttlesim' in str(header.get('runtime', {})).lower():
            raise ValueError(f'{path}: requires live source')
        for line in f:
            rec = json.loads(line)
            kind = rec.get('recordType')
            if kind == 'vehicleKeyframe':
                state = rec['fields']
            elif kind == 'vehicleDelta':
                deep_merge(state, rec['fields'])
            else:
                continue
            ut = rec['ut']
            if ut in seen:
                continue
            seen.add(ut)
            aero, att = state['aero'], state['attitude']
            flags = state['state']
            q = aero['dynamicPressure']
            alpha = att['angleOfAttack']
            if (q < 100 or not 5 <= alpha <= 30 or
                abs(att['sideslip']) > 3 or flags['gear'] or flags['airbrakes'] or
                state['vehicle']['currentThrust'] > 500 or
                state['position']['radarAltitude'] < 120):
                continue
            a = math.radians(alpha)
            cl, cd = aero['liftForce']/(q*250), aero['dragForce']/(q*250)
            if not all(math.isfinite(x) for x in (cl, cd, a)) or cl <= 0 or cd <= 0:
                continue
            yield dict(mach=aero['mach'], alpha=alpha, cl=cl, cd=cd,
                       k=cl/(math.sin(a)*math.cos(a)), cd0=cd-cl*math.tan(a))


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('logs', nargs='+', type=Path)
    ap.add_argument('--table', required=True, type=Path)
    ap.add_argument('--report', required=True, type=Path)
    ap.add_argument('--mach-bin-width', type=float,
                    help='optional uniform Mach bins on [0, 8), for resolution experiments')
    args = ap.parse_args()
    rows = [r for p in args.logs for r in samples(p)]
    knots = []
    bins = BINS
    if args.mach_bin_width is not None:
        width = args.mach_bin_width
        if not math.isfinite(width) or width < .25 or width > 2:
            ap.error('Mach bin width must be finite and between .25 and 2')
        bins = [(i*width, min(8, (i+1)*width)) for i in range(math.ceil(8/width))]
    for lo, hi in bins:
        rs = [r for r in rows if lo <= r['mach'] < hi]
        if len(rs) < 30:
            continue
        k = statistics.median(r['k'] for r in rs)
        cd0 = statistics.median(r['cd0'] for r in rs)
        if cd0 <= 0:
            raise ValueError('nonpositive axial drag: polar assumption invalid')
        errors = []
        for r in rs:
            a = math.radians(r['alpha'])
            errors.append(max(abs(k*math.sin(a)*math.cos(a)/r['cl']-1),
                              abs((cd0+k*math.sin(a)**2)/r['cd']-1)))
        knots.append(dict(mach=statistics.median(r['mach'] for r in rs),
                          mach_bin=[lo, hi], alpha_support=[min(r['alpha'] for r in rs),
                          max(r['alpha'] for r in rs)], count=len(rs), k=k, cd0=cd0,
                          in_sample_median_relative_error=statistics.median(errors),
                          in_sample_p95_relative_error=sorted(errors)[int(.95*(len(errors)-1))]))
    if len(knots) < 2:
        raise ValueError('insufficient Mach coverage')
    def params(m):
        if m <= knots[0]['mach']: return knots[0]['k'], knots[0]['cd0']
        for left, right in zip(knots, knots[1:]):
            if m <= right['mach']:
                t=(m-left['mach'])/(right['mach']-left['mach'])
                return tuple(left[x]+t*(right[x]-left[x]) for x in ('k', 'cd0'))
        return knots[-1]['k'], knots[-1]['cd0']
    with args.table.open('w') as f:
        f.write('# PROVISIONAL STS: single-flight normal-force polar; extrapolation outside reported support; no certification. Area 250 m2 is coefficient normalization only.\n')
        writer=csv.writer(f, lineterminator='\n')
        writer.writerow(['mach', 'alpha_deg', 'cl', 'cd'])
        for m in [0]+[x['mach'] for x in knots]+[10, 16, 25]:
            k, cd0 = params(m)
            for alpha in ALPHAS:
                a=math.radians(alpha)
                writer.writerow([f'{m:.8f}', alpha, f'{k*math.sin(a)*math.cos(a):.8f}',
                                 f'{cd0+k*math.sin(a)**2:.8f}'])
    report=dict(status='provisional; not STS validation', certified_cells=0,
                assumptions=['normal-force polar; no independent alpha sweep',
                             'constant endpoint parameters outside measured Mach support',
                             'single flight; residuals are in-sample, not held-out validation'],
                source=[dict(path=str(p), sha256=hashlib.sha256(p.read_bytes()).hexdigest()) for p in args.logs],
                accepted_samples=len(rows), knots=knots)
    if args.mach_bin_width is not None:
        report['mach_bin_width'] = args.mach_bin_width
    args.report.write_text(json.dumps(report, indent=2)+'\n')
    print(f'{len(rows)} accepted STS samples; {len(knots)} provisional Mach knots')

if __name__ == '__main__':
    main()
