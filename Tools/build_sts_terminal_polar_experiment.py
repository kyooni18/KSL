#!/usr/bin/env python3
"""Build an opt-in STS terminal polar using explicitly provisional Mach mapping."""
import argparse
import csv
import hashlib
import json
from pathlib import Path
from build_sts_api_incidence_experiment import bilinear, linear

ROOT = Path(__file__).resolve().parents[1]
def sha(p): return hashlib.sha256(p.read_bytes()).hexdigest()
def rows(p):
    with p.open() as f: return list(csv.DictReader(l for l in f if not l.startswith('#')))

def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--source', type=Path, default=ROOT/'Docs/Experiments/STS-Entry-20261009/live-terminal-speed-polar-20261009.json')
    ap.add_argument('--baseline', type=Path, default=ROOT/'ShuttleSim/reference-model/experiments/sts_api_mach2_polar.csv')
    ap.add_argument('--atmosphere', type=Path, default=ROOT/'ShuttleSim/reference-model/kerbin_atmosphere_identified.csv')
    ap.add_argument('--table', type=Path, required=True)
    ap.add_argument('--report', type=Path, required=True)
    args=ap.parse_args(); source=json.loads(args.source.read_text())
    assert source['status']=='complete' and source['liveStateBefore']['vessel']=='STS'
    assert all(source[k] for k in ('liveStateUnchanged','actuatorsUnchanged','savesUnchanged'))
    # One altitude avoids silently averaging different atmospheric/part effects.
    sound=float(next(r for r in rows(args.atmosphere) if float(r['altitude_m'])==20000)['speed_of_sound_mps'])
    selected=[c for c in source['samples'] if c['altitudeM']==20000 and c['airspeedMps']<=400]
    polar={(c['airspeedMps']/sound,c['alphaDeg']):(c['clPerpendicularMagnitude'],c['cd']) for c in selected}
    pm=sorted({m for m,a in polar}); pa=sorted({a for m,a in polar})
    assert len(polar)==28 and len(pm)==4 and len(pa)==7
    base={(float(r['mach']),float(r['alpha_deg'])):(float(r['cl']),float(r['cd'])) for r in rows(args.baseline)}
    bm=sorted({m for m,a in base}); ba=sorted({a for m,a in base})
    out={}; changed=0
    for m in sorted(set(bm+pm)):
        for a in ba:
            value=bilinear(base,bm,ba,m,a)
            if pm[0]<=m<=pm[-1] and pa[0]<=a<=pa[-1]:
                value=bilinear(polar,pm,pa,m,a); changed+=1
            out[m,a]=value
    assert all(out[k]==v for k,v in base.items() if not(pm[0]<=k[0]<=pm[-1] and pa[0]<=k[1]<=pa[-1]))
    with args.table.open('w') as f:
        f.write('# UNQUALIFIED STS terminal polar experiment; nominal atmosphere Mach mapping; zero certified cells.\n')
        w=csv.writer(f,lineterminator='\n');w.writerow(['mach','alpha_deg','cl','cd'])
        for (m,a),(cl,cd) in sorted(out.items()): w.writerow([f'{m:.8f}',f'{a:g}',f'{cl:.8f}',f'{cd:.8f}'])
    report=dict(classification='unqualified offline terminal polar sensitivity; provisional Mach mapping',certifiedCells=0,sourceSHA256=sha(args.source),baselineSHA256=sha(args.baseline),atmosphereSHA256=sha(args.atmosphere),tableSHA256=sha(args.table),sourceAltitudeM=20000,nominalSoundSpeedMps=sound,insertedMachAnchors=pm,modifiedMachBand=[pm[0],pm[-1]],modifiedAlphaBand=[0,28],replacedOrInsertedCells=changed,limitations=['Mach is exact TAS divided by nominal identified-table sound speed, not live hypothetical Mach; mapping sensitivity remains unvalidated.','Single 20 km static neutral-surface polar; other altitudes are holdout comparisons, not averaged.','Perpendicular force magnitude includes any side force and positive residual lift at alpha zero.','Original nodes outside selected band preserved; neighboring interpolation blends change.','No flight qualification, normal gate admission, touchdown or rollout evidence.'])
    args.report.write_text(json.dumps(report,indent=2)+'\n');print(json.dumps(report['modifiedMachBand']))
if __name__=='__main__': main()
