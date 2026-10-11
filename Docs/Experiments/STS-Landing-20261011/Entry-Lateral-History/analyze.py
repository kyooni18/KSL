#!/usr/bin/env python3
"""Decode measured entry lateral chronology; no changes to control or flight."""
import csv,hashlib,json,math,sys
from pathlib import Path
HERE=Path(__file__).resolve().parent;ROOT=HERE.parents[3]
sys.path.insert(0,str(ROOT/'ShuttleSim/tools'))
from identify_stsn_aero import _merge

def cross(a,b):return [a[1]*b[2]-a[2]*b[1],a[2]*b[0]-a[0]*b[2],a[0]*b[1]-a[1]*b[0]]
def dot(a,b):return sum(x*y for x,y in zip(a,b))
def norm(v):return [x/math.sqrt(dot(v,v)) for x in v]
report={}
for name,stamp in [('L5','2026-10-10T07-54-07Z'),('L7','2026-10-11T06-40-54Z')]:
 path=ROOT/'FlightLogs'/f'{stamp}-STS-vehicle.jsonl';state={};session={};rows=[]
 for line in path.open():
  r=json.loads(line);kind=r.get('recordType')
  if kind=='sessionStart':session=r;continue
  if kind=='vehicleKeyframe':state=r['fields']
  elif kind=='vehicleDelta':_merge(state,r['fields'])
  else:continue
  if state['state']['phase'] not in ('MM304 Entry','TAEM'):continue
  p=state['position']['inertial'];axis=session['planet']['northAxis'];omega=session['planet']['rotationalSpeed']
  rotate=cross([x*omega for x in axis],p);surf=[a-b for a,b in zip(state['motion']['velocity'],rotate)]
  up=norm(p);east=norm(cross(axis,up));north=cross(up,east)
  course=math.degrees(math.atan2(dot(surf,east),dot(surf,north)))%360
  g=state['guidance'];a=state['attitude'];c=state['command']
  rows.append(dict(ut=r['ut'],phase=state['state']['phase'],altitude=state['position']['altitude'],speed=state['motion']['trueAirSpeed'],along=g['runwayAlongTrack'],cross=g['runwayCrossTrack'],range=g['rangeToSite'],course=course,bearing=g['bearingToSite'],azimuthError=(g['bearingToSite']-course+180)%360-180,roll=a['roll'],bankCommand=c['targetRoll'],beta=a['sideslip']))
  if state['state']['phase']=='TAEM':break
 reversals=[];crossings=[];previous_sign=0
 for a,b in zip(rows,rows[1:]):
  sign=1 if b['bankCommand']>5 else -1 if b['bankCommand']<-5 else 0
  if sign and previous_sign and sign!=previous_sign:reversals.append(b)
  if sign:previous_sign=sign
  if a['cross']*b['cross']<0:crossings.append(dict(before=a,after=b))
 selected=[];last=-math.inf
 for r in rows:
  if r['ut']-last>=10 or r is rows[-1]:selected.append(r);last=r['ut']
 with (HERE/f'{name.lower()}-history.csv').open('w') as f:
  w=csv.DictWriter(f,fieldnames=rows[0].keys());w.writeheader();w.writerows(selected)
 report[name]=dict(source=str(path.relative_to(ROOT)),sha256=hashlib.sha256(path.read_bytes()).hexdigest(),samples=len(rows),inlet=rows[0],handoff=rows[-1],commandSignReversals=reversals,crossings=crossings,minimumCross=min(rows,key=lambda r:r['cross']),maximumCross=max(rows,key=lambda r:r['cross']))
 print(name,'inlet',rows[0]['cross'],'crossings',[(x['after']['ut'],x['after']['course'],x['after']['bankCommand']) for x in crossings],'reversals',[(x['ut'],x['bankCommand']) for x in reversals],'handoff',rows[-1]['cross'],flush=True)
(HERE/'result.json').write_text(json.dumps(report,indent=2)+'\n')
