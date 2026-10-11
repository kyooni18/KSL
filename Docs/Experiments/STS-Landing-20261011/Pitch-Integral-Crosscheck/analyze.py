#!/usr/bin/env python3
"""Integral identification diagnostic; no production coefficients are installed."""
import bisect
import hashlib
import json
import math
import sys
from pathlib import Path
ROOT = Path(__file__).resolve().parents[4]
sys.path.insert(0, str(ROOT / 'ShuttleSim/tools'))
from identify_stsn_attitude import _log_samples, r2
from identify_stsn_aero import _merge, _wls
LOGS = {
 'L4': 'FlightLogs/2026-10-10T06-29-47Z-STS-vehicle.jsonl',
 'L5': 'FlightLogs/2026-10-10T07-54-07Z-STS-vehicle.jsonl',
 'L7a': 'FlightLogs/2026-10-11T07-55-25Z-STS-vehicle.jsonl',
 'L7b': 'FlightLogs/2026-10-11T08-34-41Z-STS-vehicle.jsonl',
}

def decode(path):
 rows = _log_samples(str(path), 'STS')
 state, extras = {}, {}
 for line in path.read_text().splitlines():
  rec = json.loads(line)
  if rec.get('recordType') == 'vehicleKeyframe': state = rec['fields']
  elif rec.get('recordType') == 'vehicleDelta': _merge(state, rec['fields'])
  else: continue
  extras[rec['ut']] = (state['attitude']['angleOfAttackRate'], state['state']['phase'], state['motion']['flightPathAngle'])
 for row in rows:
  row['wa'], row['phase'], row['fpa'] = extras[row['ut']]
 return rows

def acceptable(row):
 keys = ('q', 'mach', 'aoa', 'beta', 'wp', 'wa', 'roll', 'dp', 'thrust', 'ralt')
 return (all(row[k] is not None and math.isfinite(row[k]) for k in keys)
         and row['sit'] == 'flying' and row['ralt'] >= 15 and row['q'] >= 500
         and row['mach'] <= .9 and abs(row['roll']) < 5 and abs(row['beta']) < 2
         and abs(row['wp']) < 30 and abs(row['wa']) < 30 and row['thrust'] <= 1)

def windows(rows, duration, delay, rate):
 times = [r['ut'] for r in rows]
 out = []; i = 0
 while i < len(rows)-1:
  if not acceptable(rows[i]): i += 1; continue
  j = i+1
  while j < len(rows) and rows[j]['ut']-rows[i]['ut'] < duration: j += 1
  if j >= len(rows): break
  selected = rows[i:j+1]
  if any(not acceptable(r) for r in selected) or any(b['ut']-a['ut'] > .2 for a,b in zip(selected, selected[1:])):
   i += 1; continue
  duration_actual = rows[j]['ut']-rows[i]['ut']
  basis = []
  for r in selected:
   k = bisect.bisect_right(times, r['ut']-delay)-1
   if k < 0 or r['ut']-delay-times[k] > .2: break
   Q = r['q']/1000
   basis.append((Q, rows[k]['dp'], math.sqrt(Q)*r[rate], Q*r['aoa'], Q))
  if len(basis) != len(selected): i += 1; continue
  out.append({'deltaRatePerSecond': (rows[j][rate]-rows[i][rate])/duration_actual,
              'dt': duration_actual, 'samples': selected, 'basis': basis,
              'ut': rows[i]['ut'], 'phase': rows[i]['phase']})
  i = j # non-overlapping intervals, shared boundary only
 return out

def averaged_basis(row, cap, constant_control=False):
 values = [[min(Q,cap)*ctrl, damp, stiff, trim] + ([ctrl] if constant_control else []) for Q,ctrl,damp,stiff,trim in row['basis']]
 return [sum((a[k]+b[k])*.5*(rb['ut']-ra['ut']) for a,b,ra,rb in zip(values,values[1:],row['samples'],row['samples'][1:]))/row['dt'] for k in range(5 if constant_control else 4)]

def fit(train, tests, constant_control=False):
 candidates = []
 for cap in [x*.5 for x in range(2,41)]:
  basis = lambda r: averaged_basis(r,cap,constant_control)
  sol = _wls(train,basis,lambda r:r['deltaRatePerSecond'],lambda r:1.)
  if sol is None: continue
  pred = lambda r:sum(x*c for x,c in zip(basis(r),sol))
  score = r2(train,lambda r:r['deltaRatePerSecond'],pred)
  candidates.append((score,cap,sol))
 score,cap,sol = max(candidates,key=lambda c:c[0])
 pred = lambda r:sum(x*c for x,c in zip(averaged_basis(r,cap,constant_control),sol))
 def report(rows):
  return {'n':len(rows),'r2':r2(rows,lambda r:r['deltaRatePerSecond'],pred),
          'rmseDegPerSecond2':math.sqrt(sum((r['deltaRatePerSecond']-pred(r))**2 for r in rows)/len(rows)) if rows else None}
 return {'capKPa':cap,'controlPerKPa':sol[0],'sqrtQDamping':sol[1],
         'stiffnessPerKPa':-sol[2],'trimDeg':-sol[3]/sol[2] if sol[2] else None,
         'physicalSigns':sol[0]>0 and sol[1]<=0 and sol[2]<0 and (not constant_control or sol[4]>=0),
         'constantControlTorque': constant_control, 'controlConstant': sol[4] if constant_control else 0.,
         'train':report(train),'heldout':{k:report(v) for k,v in tests.items()}}

def main():
 rows = {k:decode(ROOT/v) for k,v in LOGS.items()}
 report = {'logs':{k:{'path':v,'sha256':hashlib.sha256((ROOT/v).read_bytes()).hexdigest()} for k,v in LOGS.items()},
           'selection':'flying, radar>=15m, Q>=.5kPa, Mach<=.9, |bank|<5deg, |beta|<2deg, unpowered; contiguous gaps<=.2s',
           'model':'delta rate / interval = time average of min(Q,cap)*controlGain*delayedControl + sqrt(Q)*damping*rate - Q*stiffness*(AoA-trim)',
           'policy':'L4 training only. Cap chosen by training R2. All durations/delays diagnostic; L5 and both L7 flights held out. No production model or golden changes.',
           'fits':[]}
 for coordinate,rate in [('body','wp'),('aoa','wa')]:
  for duration in (.2,.5,1.,2.):
   for delay in (0.,.1,.2):
    grouped = {k:windows(v,duration,delay,rate) for k,v in rows.items()}
    result = fit(grouped['L4'],{k:v for k,v in grouped.items() if k!='L4'})
    report['fits'].append({'coordinate':coordinate,'intervalSeconds':duration,'delaySeconds':delay,**result})
    print(coordinate,duration,delay,'signs',result['physicalSigns'],'train',round(result['train']['r2'],3),
          {k:round(v['r2'],3) if v['r2'] is not None else None for k,v in result['heldout'].items()},flush=True)
 report['kinematics'] = {}
 for name,seq in rows.items():
  grouped = windows(seq,.5,0.,'wp')
  values = []
  for w in grouped:
   samples = w['samples']; a,b=samples[0],samples[-1]
   rotation = sum((ra['wp']+rb['wp'])*.5*(rb['ut']-ra['ut']) for ra,rb in zip(samples,samples[1:]))
   values.append({'actual':b['aoa']-a['aoa'],'bodyOnly':rotation,'withFPA':rotation-(b['fpa']-a['fpa'])})
  report['kinematics'][name] = {'n':len(values), **{key:{'r2':r2(values,lambda r:r['actual'],lambda r:r[key]),'rmseDeg':math.sqrt(sum((r['actual']-r[key])**2 for r in values)/len(values))} for key in ('bodyOnly','withFPA')}}
 print('kinematics',report['kinematics'],flush=True)
 report['crossTraining'] = []
 for coordinate,rate in [('body','wp'),('aoa','wa')]:
  grouped = {k:windows(v,.5,0.,rate) for k,v in rows.items()}
  for constant_control in (False,True):
   for train in LOGS:
    result = fit(grouped[train], {k:v for k,v in grouped.items() if k!=train},constant_control)
    report['crossTraining'].append({'coordinate':coordinate,'trainingFlight':train,'intervalSeconds':.5,'delaySeconds':0.,**result})
    print('cross',coordinate,train,constant_control,result,flush=True)
 Path(__file__).with_name('result.json').write_text(json.dumps(report,indent=2)+'\n')
if __name__=='__main__':main()
