#!/usr/bin/env python3
"""Build an offline STS incidence-shape experiment from the archived API grid.

Neutral controls, beta=bank=0. Coefficients use force/q/250 m2. Only Mach
4..6 and alpha18..28 nodes are replaced; surrounding original nodes remain.
Interpolation across the enclosing cells is an explicitly provisional blend.
No live KSP calls, certified cells, or production profile changes.
"""
import argparse
import bisect
import csv
import hashlib
import json
import math
from pathlib import Path
import statistics

ROOT=Path(__file__).resolve().parents[1]


def bracket(values,x):
    i=max(0,min(len(values)-2,bisect.bisect_right(values,x)-1))
    return values[i],values[i+1]


def linear(x,x0,x1,y0,y1):
    return y0+(x-x0)/(x1-x0)*(y1-y0)


def bilinear(table,ms,alphas,m,a):
    m0,m1=bracket(ms,m);a0,a1=bracket(alphas,a)
    return tuple(linear(m,m0,m1,
        linear(a,a0,a1,table[m0,a0][k],table[m0,a1][k]),
        linear(a,a0,a1,table[m1,a0][k],table[m1,a1][k])) for k in range(2))


def identity(p):
    return dict(path=str(p.relative_to(ROOT)),sha256=hashlib.sha256(p.read_bytes()).hexdigest())


def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--grid',type=Path,default=ROOT/'Docs/Experiments/STS-Entry-20261009/live-aero-envelope-grid-20261009.json')
    ap.add_argument('--baseline',type=Path,default=ROOT/'ShuttleSim/reference-model/sts_aero_provisional.csv')
    ap.add_argument('--table',type=Path,required=True)
    ap.add_argument('--report',type=Path,required=True)
    args=ap.parse_args()
    source=json.loads(args.grid.read_text())
    if source['provenance'].get('vehicleModelId')!='STS' or source['liveStateBefore'].get('vessel')!='STS':
        raise ValueError('requires exact original STS grid identity')
    if source['status']!='complete' or not source['loadedMassAssessment']['massStateMatchesSettledProfile']:
        raise ValueError('requires completed settled-mass grid')
    with args.baseline.open() as f:
        rows=list(csv.DictReader(l for l in f if not l.startswith('#')))
    baseline={(float(r['mach']),float(r['alpha_deg'])):(float(r['cl']),float(r['cd'])) for r in rows}
    ms=sorted({m for m,a in baseline});alphas=sorted({a for m,a in baseline})
    grid={};anchors=[]
    for m in (4.,5.,6.):
        for a in (18.,28.):
            selected=[s for s in source['samples'] if s['nominalMachEstimate']==m and
                      s['angleOfAttackDeg']==a and s['sideslipDeg']==0 and s['bankDeg']==0]
            if len(selected)!=5:
                raise ValueError(f'expected five altitude samples for {m}/{a}')
            coefficients=[]
            for s in selected:
                drag=s['dragN_alongRelativeAirflow'];q=s['dynamicPressureEstimatePa']
                lift=math.sqrt(max(0,sum(v*v for v in s['forceN_bodyXYZ'])-drag*drag))
                if q<=0 or drag<=0 or lift<=0:raise ValueError('nonpositive force/flow')
                coefficients.append((lift/q/250,drag/q/250))
            cl,cd=(statistics.median(v[k] for v in coefficients) for k in range(2))
            grid[m,a]=(cl,cd)
            anchors.append(dict(mach_estimate=m,alpha_deg=a,count=len(selected),cl=cl,cd=cd,
                                coefficient_ranges=[[min(v[k] for v in coefficients),max(v[k] for v in coefficients)] for k in range(2)],
                                baseline_cl_cd=bilinear(baseline,ms,alphas,m,a)))
    new_ms=sorted(set(ms)|{4.,5.,6.})
    output={};replaced=0
    for m in new_ms:
        for a in alphas:
            v=bilinear(baseline,ms,alphas,m,a)
            if 4<=m<=6 and 18<=a<=28:
                v=bilinear(grid,[4.,5.,6.],[18.,28.],m,a);replaced+=1
            output[m,a]=v
    # The modification must leave every original node outside the supported band untouched.
    for (m,a),v in baseline.items():
        if not(4<=m<=6 and 18<=a<=28):
            if any(abs(output[m,a][k]-v[k])>1e-12 for k in range(2)):
                raise ValueError('out-of-band original node changed')
    with args.table.open('w') as f:
        f.write('# OFFLINE STS API incidence-shape experiment; hypothetical neutral-surface grid; zero certified cells.\n')
        w=csv.writer(f,lineterminator='\n');w.writerow(['mach','alpha_deg','cl','cd'])
        for (m,a),(cl,cd) in sorted(output.items()):
            w.writerow([f'{m:.8f}',f'{a:g}',f'{cl:.8f}',f'{cd:.8f}'])
    report=dict(classification='provisional offline API incidence-shape experiment; not STS validation',
        sources=[identity(args.grid),identity(args.baseline)],certified_cells=0,
        source_qualification=source['qualification'],
        modified_node_band=dict(mach=[4,6],alpha_deg=[18,28]),replaced_nodes=replaced,
        adjacent_interpolation_blend_extent=dict(
            mach=[max(m for m in ms if m<4),min(m for m in ms if m>6)],
            alpha_deg=[max(a for a in alphas if a<18),min(a for a in alphas if a>28)]),
        assumptions=['neutral controls, zero beta and bank; API forces are hypothetical',
                     'median force/q over five altitudes; q and Mach estimates inherit source epoch/temperature caveats',
                     'linear interpolation between two measured-in-API incidence points',
                     'original nodes outside the band remain; interpolation into adjacent cells is a provisional blend',
                     'sub-Mach4 aero and borrowed STS-N attitude dynamics remain unvalidated'],anchors=anchors)
    args.report.write_text(json.dumps(report,indent=2)+'\n')
    print(f'{replaced} nodes replaced from six API anchors; all out-of-band original nodes preserved')


if __name__=='__main__':main()
