#!/usr/bin/env python3
"""Passive held-out test of measured control authority, not a plant qualification."""
import hashlib,importlib.util,json,math
from pathlib import Path
HERE=Path(__file__).resolve().parent
ROOT=HERE.parents[3]
spec=importlib.util.spec_from_file_location('integral',HERE.parent/'Pitch-Integral-Crosscheck/analyze.py')
m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m)
logs=dict(m.LOGS,L6='FlightLogs/2026-10-10T08-28-20Z-STS-vehicle.jsonl',L7c='FlightLogs/2026-10-11T09-29-11Z-STS-vehicle.jsonl')
rows={};sources={}
for name,relative in logs.items():
 p=ROOT/relative;data=m.decode(p);state={};extras={}
 for line in p.read_text().splitlines():
  rec=json.loads(line)
  if rec.get('recordType')=='vehicleKeyframe':state=rec['fields']
  elif rec.get('recordType')=='vehicleDelta':m._merge(state,rec['fields'])
  else:continue
  v=state['vehicle'];extras[rec['ut']]=(v.get('availableTorque'),v.get('momentOfInertia'))
 for row in data:
  torque,inertia=extras[row['ut']]
  row['authority']=math.degrees(torque[0]/inertia[0]) if torque and inertia and inertia[0]>0 else None
  row['inertiaScale']=1e6/inertia[0] if inertia and inertia[0]>0 else None
 rows[name]=m.windows([r for r in data if r['authority'] is not None],.5,0.,'wp')
 sources[name]={'path':relative,'sha256':hashlib.sha256(p.read_bytes()).hexdigest(),'windows':len(rows[name])}
print('windows',{k:len(v) for k,v in rows.items()},flush=True)
def basis(row,kind):
 vals=[]
 for r in row['samples']:
  Q=r['q']/1000
  if kind=='authority':val=[r['authority']*r['dp'],math.sqrt(Q)*r['wp'],Q*r['aoa'],Q]
  elif kind=='authority-inertia':val=[r['authority']*r['dp'],math.sqrt(Q)*r['wp']*r['inertiaScale'],Q*r['aoa']*r['inertiaScale'],Q*r['inertiaScale']]
  else:raise ValueError(kind)
  vals.append(val)
 return [sum((a[k]+b[k])*.5*(rb['ut']-ra['ut']) for a,b,ra,rb in zip(vals,vals[1:],row['samples'],row['samples'][1:]))/row['dt'] for k in range(4)]
def summary(data,sol,kind):
 pred=lambda r:sum(a*b for a,b in zip(basis(r,kind),sol))
 return {'n':len(data),'r2':m.r2(data,lambda r:r['deltaRatePerSecond'],pred),'rmseDegPerSecond2':math.sqrt(sum((r['deltaRatePerSecond']-pred(r))**2 for r in data)/len(data))}
report={'sources':sources,'policy':'L4 training only; .5s zero-delay nonoverlap wings-level windows. L5/L6/all L7 continuations held out. No coefficients adopted.',
        'model':'Pitch angular acceleration from native available pitch torque/inertia * input, sqrt(Q) damping, Q AoA stiffness and Q trim. Optional 1e6/I scaling of passive terms. AvailableTorque is maximum authority, not actual net moment.', 'fits':[]}
for kind in ('authority','authority-inertia'):
 train=rows['L4'];sol=m._wls(train,lambda r:basis(r,kind),lambda r:r['deltaRatePerSecond'],lambda r:1.)
 result={'model':kind,'coefficients':sol,'physicalSigns':sol[0]>0 and sol[1]<=0 and sol[2]<0,'train':summary(train,sol,kind),'heldout':{k:summary(v,sol,kind) for k,v in rows.items() if k!='L4'}}
 report['fits'].append(result);print(json.dumps(result),flush=True)
# Repeat the previous Q/cap model on the expanded held-out set for like-for-like comparison.
report['previousQModel']=m.fit(rows['L4'],{k:v for k,v in rows.items() if k!='L4'})
(HERE/'result.json').write_text(json.dumps(report,indent=2)+'\n')
