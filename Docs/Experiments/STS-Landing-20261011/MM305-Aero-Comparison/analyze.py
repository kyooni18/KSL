#!/usr/bin/env python3
"""Compare existing provisional polars without changing gates or coefficients."""
import collections,csv,hashlib,json,math,re
from pathlib import Path
HERE=Path(__file__).resolve().parent
ROOT=HERE.parents[3]
result={'scope':'focused sample 0 only, not a complete qualification batch',
 'sourceCommit':'7eb7bdd','massKg':25123.15,
 'environment':{'KSP_LANDER_MODEL_PROFILE':'sts-live-20261010','KSP_LANDER_TAEM_PROFILE_DIAGNOSTICS':'1'},
 'command':['CLanding/build/mm305_feasibility_tests','--configuration','Configuration/sts-live.json','--mass-kg','25123.15','--sample-index','0'],
 'alternativeArguments':['--aero','ShuttleSim/reference-model/sts_aero_live20261011.csv'],
 'runs':{},'polars':{},'hashes':{}}
for name in ['baseline.log','aero11.log']:
 p=HERE/name; rows=[]
 for line in p.read_text().splitlines():
  if 'TAEM smooth trial:' in line:
   rows.append({k:v if k=='reason' else float(v) for k,v in re.findall(r'(\w+)=([^\s]+)',line)})
 completed=[r for r in rows if r['completed']]
 result['runs'][name]={'sha256':hashlib.sha256(p.read_bytes()).hexdigest(),
  'trials':len(rows),'completed':len(completed),'jointlyFeasible':sum(r['feasible'] for r in rows),
  'closestAltitudeCompletedTrial':min(completed,key=lambda r:r['endpoint']),
  'qualificationExitCode':1}
 assert len(rows)==2184
for name in ['sts_aero_live20261010.csv','sts_aero_live20261011.csv']:
 p=ROOT/'ShuttleSim/reference-model'/name
 groups=collections.defaultdict(list)
 for row in csv.DictReader(x for x in p.read_text().splitlines() if not x.startswith('#')):
  r={k:float(v) for k,v in row.items()};groups[r['mach']].append(r)
 summary=[]
 for mach,rows in sorted(groups.items()):
  eligible=[r for r in rows if 0<r['alpha_deg']<=30 and r['cd']>0]
  best=max(eligible,key=lambda r:r['cl']/r['cd'])
  summary.append({'mach':mach,'alphaDeg':best['alpha_deg'],'maximumTabulatedLD_0to30deg':best['cl']/best['cd'],
   'unbankedSteadyGlideAngleDeg':math.degrees(math.atan2(best['cd'],best['cl']))})
 result['polars'][name]=summary
 result['hashes'][str(p.relative_to(ROOT))]=hashlib.sha256(p.read_bytes()).hexdigest()
for name in ['Configuration/sts-live.json','CLanding/build/mm305_feasibility_tests']:
 result['hashes'][name]=hashlib.sha256((ROOT/name).read_bytes()).hexdigest()
(HERE/'result.json').write_text(json.dumps(result,indent=2)+'\n')
