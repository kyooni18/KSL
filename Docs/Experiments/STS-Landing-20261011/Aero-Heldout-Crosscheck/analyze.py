#!/usr/bin/env python3
"""Held-out native force comparison; no fitting or guidance changes."""
import bisect,csv,hashlib,json,math,sys
from pathlib import Path
HERE=Path(__file__).resolve().parent;ROOT=HERE.parents[3]
sys.path.insert(0,str(ROOT/'ShuttleSim/tools'))
from identify_stsn_aero import _merge
LOGS={'L5':'2026-10-10T07-54-07Z','L6':'2026-10-10T08-28-20Z','L7':'2026-10-11T06-40-54Z'}
TABLES=['sts_aero_live20261010.csv','sts_aero_live20261011.csv']
BINS=[(.5,1),(1,1.5),(1.5,2),(2,2.5),(2.5,3.5)]
def table(path):
 rows=[{k:float(v) for k,v in r.items()} for r in csv.DictReader(x for x in path.read_text().splitlines() if not x.startswith('#'))]
 return sorted({r['mach'] for r in rows}),sorted({r['alpha_deg'] for r in rows}),{(r['mach'],r['alpha_deg']):(r['cl'],r['cd']) for r in rows}
def coefficients(t,m,a):
 ms,aa,grid=t
 i=max(0,min(len(ms)-2,bisect.bisect_left(ms,m)-1));j=max(0,min(len(aa)-2,bisect.bisect_left(aa,a)-1))
 fm=max(0,min(1,(m-ms[i])/(ms[i+1]-ms[i])));fa=max(0,min(1,(a-aa[j])/(aa[j+1]-aa[j])))
 return [sum(grid[ms[i+di],aa[j+dj]][k]*wm*wa for di,wm in [(0,1-fm),(1,fm)] for dj,wa in [(0,1-fa),(1,fa)]) for k in range(2)]
def quantile(rows,key,q):
 ordered=sorted(rows,key=lambda r:r[key]);goal=q*sum(r['dt'] for r in ordered);acc=0
 for r in ordered:
  acc+=r['dt']
  if acc>=goal:return r[key]
 return ordered[-1][key]
def summarize(rows):
 if not rows:return {'samples':0,'durationSeconds':0}
 duration=sum(r['dt'] for r in rows)
 out={'samples':len(rows),'durationSeconds':duration,'aoaDegRange':[min(r['aoa'] for r in rows),max(r['aoa'] for r in rows)],'qPaRange':[min(r['q'] for r in rows),max(r['q'] for r in rows)],'medianMeasuredLD':quantile(rows,'measuredLD',.5)}
 for component in ['lift','drag']:
  errors=[r[component+'Pred']-r[component] for r in rows]
  out[component]={'medianMeasuredOverPredicted':quantile(rows,component+'Ratio',.5),'p05MeasuredOverPredicted':quantile(rows,component+'Ratio',.05),'p95MeasuredOverPredicted':quantile(rows,component+'Ratio',.95),'biasN':sum(e*r['dt'] for e,r in zip(errors,rows))/duration,'rmseN':math.sqrt(sum(e*e*r['dt'] for e,r in zip(errors,rows))/duration),'relativeRmsError':math.sqrt(sum(e*e*r['dt'] for e,r in zip(errors,rows))/sum(r[component]**2*r['dt'] for r in rows))}
 return out
report={'policy':'No fitting; Oct10 L5/L6 and Oct11 L7 held out from Oct9 table sources. Uses measured q/Mach/AoA to isolate polar error from atmosphere or state propagation. 250m2 normalization; no force-book/scales. Bilinear interpolation matches aero_coefficients. Duration weighted on unique UT, excludes gaps >0.5s.','filters':'MM304 Entry or TAEM; q>=300Pa; Mach[0.5,3.5); AoA[0,30]; |sideslip|<=5deg; positive native lift/drag; no airbrakes; thrust<=1N. Native logged forces, not derived from trajectory.','sources':{},'tables':{},'results':{}}
tables={name:table(ROOT/'ShuttleSim/reference-model'/name) for name in TABLES}
for name in TABLES:report['tables'][name]={'sha256':hashlib.sha256((ROOT/'ShuttleSim/reference-model'/name).read_bytes()).hexdigest()}
for flight,stamp in LOGS.items():
 path=ROOT/'FlightLogs'/f'{stamp}-STS-vehicle.jsonl';state={};allrows=[];previous=None;skipped=0
 for line in path.read_text().splitlines():
  rec=json.loads(line);kind=rec.get('recordType')
  if kind=='vehicleKeyframe':state=rec['fields']
  elif kind=='vehicleDelta':_merge(state,rec['fields'])
  else:continue
  ut=rec['ut']
  if previous is None:previous=ut;continue
  dt=ut-previous
  if dt<=0:skipped+=1;continue
  previous=ut
  if dt>.5:continue
  a=state['aero'];att=state['attitude'];st=state['state'];v=state['vehicle']
  q=a['dynamicPressure'];mach=a['mach'];aoa=att['angleOfAttack'];lift=a['liftForce'];drag=a['dragForce']
  if st['phase'] not in ['MM304 Entry','TAEM'] or st['airbrakes'] or v['currentThrust']>1:continue
  if not all(math.isfinite(x) for x in [q,mach,aoa,lift,drag,att['sideslip']]):continue
  if not(q>=300 and .5<=mach<3.5 and 0<=aoa<=30 and abs(att['sideslip'])<=5 and lift>0 and drag>0):continue
  allrows.append({'ut':ut,'dt':dt,'q':q,'mach':mach,'aoa':aoa,'lift':lift,'drag':drag,'measuredLD':lift/drag})
 report['sources'][flight]={'path':str(path.relative_to(ROOT)),'sha256':hashlib.sha256(path.read_bytes()).hexdigest(),'repeatedOrBackwardUTExcluded':skipped}
 report['results'][flight]={}
 for name,t in tables.items():
  rows=[]
  for raw in allrows:
   r=dict(raw);cl,cd=coefficients(t,r['mach'],r['aoa']);r['liftPred']=r['q']*250*cl;r['dragPred']=r['q']*250*cd
   if r['liftPred']<=0 or r['dragPred']<=0:continue
   for key in ['lift','drag']:r[key+'Ratio']=r[key]/r[key+'Pred']
   rows.append(r)
  report['results'][flight][name]={f'{lo}-{hi}':summarize([r for r in rows if lo<=r['mach']<hi]) for lo,hi in BINS}
  print(flight,name,[(b,s['samples'],round(s.get('drag',{}).get('medianMeasuredOverPredicted',0),3),round(s.get('lift',{}).get('medianMeasuredOverPredicted',0),3)) for b,s in report['results'][flight][name].items()])
(HERE/'result.json').write_text(json.dumps(report,indent=2)+'\n')
