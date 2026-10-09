#!/usr/bin/env python3
"""Recalibrate the provisional STS polar against the most recent live KSP flights.

The 2026-10-07 table was fitted to one flight.  Archived flights from 2026-10-09
show a repeatable vehicle-state change: at Mach 5-7 and alpha 24-28 deg the same
lift (CL ~0.15) comes with ~0.05-0.06 more drag (CD 0.32-0.35 vs 0.27-0.30) in
06:18/06:37 versus 07:22/07:53/08:33.  This tool

  * groups live STS flights into an "earlier" and a "latest" vehicle state,
  * fits the same normal-force polar (CL=K sin a cos a, CD=CD0+K sin^2 a) per Mach bin,
  * uses latest-state data where it has support, and otherwise applies the measured
    CD0 increment (tapered to zero by Mach 1.5) to earlier-state fits,
  * validates on a held-out latest-state flight and reports error against the old table,
  * checks the model atmosphere via q_measured / (0.5 rho_model v^2).

Output is still PROVISIONAL (cells are not independently certified).
"""
import argparse
import bisect
import csv
import hashlib
import json
import math
import statistics
from pathlib import Path

from normalize_ksp_vehicle_log import deep_merge
from build_sts_provisional_aero import BINS, ALPHAS

AREA = 250.0


def load_atmosphere(path):
    alts, rho = [], []
    for line in open(path):
        if line.startswith('#') or line.startswith('altitude'):
            continue
        p = line.split(',')
        alts.append(float(p[0])); rho.append(float(p[1]))
    return alts, rho


def rho_at(atm, h):
    alts, rho = atm
    if h <= alts[0]:
        return rho[0]
    if h >= alts[-1]:
        return rho[-1] * math.exp(-(h - alts[-1]) / 5600.0)
    i = bisect.bisect_right(alts, h) - 1
    t = (h - alts[i]) / (alts[i + 1] - alts[i])
    return math.exp(math.log(rho[i]) * (1 - t) + math.log(rho[i + 1]) * t)


def samples(path, atm):
    state, seen = {}, set()
    with path.open() as f:
        header = json.loads(next(f))
        if header.get('vessel') != 'STS' or 'shuttlesim' in str(header.get('runtime', {})).lower():
            raise ValueError('not a live STS log')
        for line in f:
            try:
                rec = json.loads(line)
            except ValueError:
                continue
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
            try:
                aero, att, flags = state['aero'], state['attitude'], state['state']
                q, alpha = aero['dynamicPressure'], att['angleOfAttack']
                if (q < 100 or not 5 <= alpha <= 32 or abs(att['sideslip']) > 3 or flags['gear'] or
                        flags['airbrakes'] or state['vehicle']['currentThrust'] > 500 or
                        state['position']['radarAltitude'] < 120):
                    continue
                a = math.radians(alpha)
                cl, cd = aero['liftForce'] / (q * AREA), aero['dragForce'] / (q * AREA)
                if not all(math.isfinite(x) for x in (cl, cd)) or cl <= 0 or cd <= 0:
                    continue
                tas = state['motion']['trueAirSpeed']
                rho = rho_at(atm, state['position']['altitude'])
                yield dict(mach=aero['mach'], alpha=alpha, cl=cl, cd=cd, k=cl / (math.sin(a) * math.cos(a)),
                           cd0=cd - cl * math.tan(a), qratio=q / (0.5 * rho * tas * tas) if rho > 0 and tas > 50 else None)
            except KeyError:
                continue


def fit_bins(rows, min_n):
    knots = []
    for lo, hi in BINS:
        rs = [r for r in rows if lo <= r['mach'] < hi]
        if len(rs) < min_n:
            continue
        knots.append(dict(mach=statistics.median(r['mach'] for r in rs), mach_bin=[lo, hi], count=len(rs),
                          alpha_support=[min(r['alpha'] for r in rs), max(r['alpha'] for r in rs)],
                          k=statistics.median(r['k'] for r in rs), cd0=statistics.median(r['cd0'] for r in rs)))
    return knots


def interp(knots, m, key):
    if m <= knots[0]['mach']:
        return knots[0][key]
    for l, r in zip(knots, knots[1:]):
        if m <= r['mach']:
            t = (m - l['mach']) / (r['mach'] - l['mach'])
            return l[key] + t * (r[key] - l[key])
    return knots[-1][key]


def rel_errors(knots, rows):
    ecl, ecd = [], []
    for r in rows:
        k, cd0 = interp(knots, r['mach'], 'k'), interp(knots, r['mach'], 'cd0')
        a = math.radians(r['alpha'])
        ecl.append(abs(k * math.sin(a) * math.cos(a) / r['cl'] - 1))
        ecd.append(abs((cd0 + k * math.sin(a) ** 2) / r['cd'] - 1))
    def q(v, p): return sorted(v)[int(p * (len(v) - 1))]
    return dict(n=len(rows), cl_median=statistics.median(ecl), cl_p95=q(ecl, .95),
                cd_median=statistics.median(ecd), cd_p95=q(ecd, .95))


def combine(early, late):
    """Latest-state fit where supported; otherwise earlier fit + measured CD0 step."""
    overlap = []
    for l in late:
        for e in early:
            if e['mach_bin'] == l['mach_bin']:
                overlap.append((l['count'], l['cd0'] - e['cd0'], l['k'] / e['k']))
    if not overlap:
        raise ValueError('no overlapping Mach bins between vehicle states')
    w = sum(o[0] for o in overlap)
    d_cd0 = sum(o[0] * o[1] for o in overlap) / w
    k_ratio = sum(o[0] * o[2] for o in overlap) / w
    out = []
    for e in early:
        l = next((x for x in late if x['mach_bin'] == e['mach_bin']), None)
        if l:
            out.append(dict(l, source='latest-state live fit'))
        else:
            taper = min(1.0, max(0.0, (e['mach'] - 1.5) / 1.5))
            out.append(dict(e, cd0=e['cd0'] + taper * d_cd0, k=e['k'] * (1 + taper * (k_ratio - 1)),
                            source=f'earlier-state fit + CD0 step {taper*d_cd0:+.3f} (assumed)'))
    # Latest-state bins the earlier state never covered.
    for l in late:
        if not any(x['mach_bin'] == l['mach_bin'] for x in early):
            out.append(dict(l, source='latest-state live fit'))
    out.sort(key=lambda x: x['mach'])
    return out, d_cd0, k_ratio


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--early', nargs='+', type=Path, required=True)
    ap.add_argument('--late', nargs='+', type=Path, required=True)
    ap.add_argument('--holdout', type=Path, required=True, help='latest-state flight excluded from the fit')
    ap.add_argument('--atmosphere', type=Path, required=True)
    ap.add_argument('--old-table-report', type=Path, required=True)
    ap.add_argument('--table', type=Path, required=True)
    ap.add_argument('--report', type=Path, required=True)
    a = ap.parse_args()
    atm = load_atmosphere(a.atmosphere)
    early = [r for p in a.early for r in samples(p, atm)]
    late_fit = [r for p in a.late for r in samples(p, atm)]
    held = list(samples(a.holdout, atm))
    e_knots, l_knots_fit = fit_bins(early, 100), fit_bins(late_fit, 100)
    knots_fit, dcd0, kratio = combine(e_knots, l_knots_fit)
    old = json.loads(a.old_table_report.read_text())['knots']
    val = dict(heldout_flight=a.holdout.name, new=rel_errors(knots_fit, [r for r in held if r['mach'] >= 3.0]),
               old_provisional=rel_errors(old, [r for r in held if r['mach'] >= 3.0]),
               note='held-out flight validates Mach>=3 only; lower-Mach latest-state coverage is absent')
    # Final fit adds the held-out flight.
    late_all = late_fit + held
    knots, dcd0, kratio = combine(e_knots, fit_bins(late_all, 100))
    qr = [r['qratio'] for r in early + late_all if r['qratio']]
    with a.table.open('w') as f:
        f.write('# PROVISIONAL STS (live recalibration 2026-10-09): normal-force polar per Mach bin; latest-state flights where '
                'available, earlier-state + assumed CD0 step elsewhere; not certified. Area 250 m2 is normalization only.\n')
        w = csv.writer(f, lineterminator='\n')
        w.writerow(['mach', 'alpha_deg', 'cl', 'cd'])
        for m in [0] + [x['mach'] for x in knots] + [10, 16, 25]:
            k, cd0 = interp(knots, m, 'k'), interp(knots, m, 'cd0')
            for al in ALPHAS:
                r = math.radians(al)
                w.writerow([f'{m:.8f}', al, f'{k*math.sin(r)*math.cos(r):.8f}', f'{cd0+k*math.sin(r)**2:.8f}'])
    rep = dict(status='provisional; recalibrated to latest live vehicle state; not certified', certified_cells=0,
               vehicle_state_step=dict(delta_cd0=dcd0, k_ratio=kratio),
               validation=val, atmosphere_check=dict(q_over_half_rho_v2_median=statistics.median(qr),
                                                      p05=sorted(qr)[int(.05 * (len(qr) - 1))],
                                                      p95=sorted(qr)[int(.95 * (len(qr) - 1))], n=len(qr)),
               early_flights=[p.name for p in a.early], late_flights=[p.name for p in a.late] + [a.holdout.name],
               source=[dict(path=str(p), sha256=hashlib.sha256(p.read_bytes()).hexdigest())
                       for p in a.early + a.late + [a.holdout]],
               knots=knots)
    a.report.write_text(json.dumps(rep, indent=2) + '\n')
    print(json.dumps(dict(step=rep['vehicle_state_step'], validation=val, atm=rep['atmosphere_check']), indent=1))


if __name__ == '__main__':
    main()
