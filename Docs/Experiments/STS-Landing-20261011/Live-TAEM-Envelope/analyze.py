#!/usr/bin/env python3
"""Compare measured TAEM history with planning assumptions; no gate changes."""
import csv,hashlib,json,math,sys
from pathlib import Path
HERE=Path(__file__).resolve().parent;ROOT=HERE.parents[3]
sys.path.insert(0,str(ROOT/'ShuttleSim/tools'))
from identify_stsn_aero import _merge
report={'policy':'Native L5/L6 successful landings and original L7 failed landing. TAEM phase to first Final; repeated UT excluded. Duration weighting excludes gaps over1s. Actual FPA is logged flight path, not planned reference slope. No certification or relaxation of constraints.', 'planningAssumptions':{'taemReferenceSlopeLimitDeg':24,'finalAlignmentSpeedMps':160},'flights':{}}
for name,stamp in [('L5','2026-10-10T07-54-07Z'),('L6','2026-10-10T08-28-20Z'),('L7','2026-10-11T06-40-54Z')]:
 p=ROOT/'FlightLogs'/f'{stamp}-STS-vehicle.jsonl';state={};rows=[];final=None;previous=None
 for line in p.read_text().splitlines():
  r=json.loads(line);kind=r.get('recordType')
  if kind=='vehicleKeyframe':state=r['fields']
  elif kind=='vehicleDelta':_merge(state,r['fields'])
  else:continue
  phase=state['state']['phase'];ut=r['ut']
  if previous is not None and ut<=previous:continue
  dt=0 if previous is None or ut-previous>1 else ut-previous;previous=ut
  a=state['aero'];m=state['motion'];g=state['guidance'];att=state['attitude']
  if state['vehicle']['mass']<=0:continue
  row={'ut':ut,'dt':dt,'altitude':state['position']['altitude'],'mach':a['mach'],'speed':m['trueAirSpeed'],'fpa':m['flightPathAngle'],'bank':att['roll'],'aoa':att['angleOfAttack'],'q':a['dynamicPressure'],'liftAccel':a['liftForce']/state['vehicle']['mass'],'dragAccel':a['dragForce']/state['vehicle']['mass'],'along':g['runwayAlongTrack'],'cross':g['runwayCrossTrack'],'range':g['rangeToSite']}
  if phase=='TAEM':rows.append(row)
  elif phase.startswith('Final') and rows:final=row;break
 assert rows
 rows[0]['dt']=0.0
 duration=sum(r['dt'] for r in rows)
 bands={}
 for lo,hi in [(0,.5),(.5,1),(1,1.5),(1.5,2),(2,2.5),(2.5,3.5)]:
  rs=[r for r in rows if lo<=r['mach']<hi]
  if not rs:continue
  bands[f'{lo}-{hi}']={'samples':len(rs),'durationSeconds':sum(r['dt'] for r in rs),'timeBelowMinus24DegSeconds':sum(r['dt'] for r in rs if r['fpa'] < -24),'steepestDescent':min(rs,key=lambda r:r['fpa']),'maximumAbsBank':max(rs,key=lambda r:abs(r['bank']))}
 aligned=[r for r in rows if abs(r['cross'])<=300 and r['altitude']<5000]
 near=min(aligned,key=lambda r:abs(r['along']+8000)) if aligned else None
 result={'maximumQPa':max(r['q'] for r in rows),'maximumLiftLoadG':max(r['liftAccel']/9.80665 for r in rows),'nearestAlignedEightKmSample':near,'source':str(p.relative_to(ROOT)),'sha256':hashlib.sha256(p.read_bytes()).hexdigest(),'samples':len(rows),'durationSeconds':duration,'firstTAEM':rows[0],'firstFinal':final,'lastTAEM':rows[-1],'steepestDescent':min(rows,key=lambda r:r['fpa']),'timeBelowMinus24DegSeconds':sum(r['dt'] for r in rows if r['fpa'] < -24),'machBands':bands}
 report['flights'][name]=result
 selected=[];last=-math.inf
 for r in rows:
  if r['ut']-last>=5 or r is result['steepestDescent'] or r is rows[-1]:selected.append(r);last=r['ut']
 with (HERE/f'{name.lower()}-history.csv').open('w') as f:
  w=csv.DictWriter(f,fieldnames=rows[0].keys());w.writeheader();w.writerows(selected)
 print(name,'duration',duration,'under-24',result['timeBelowMinus24DegSeconds'],'steepest',result['steepestDescent'],'Final',final,flush=True)
report['configurationSHA256']=hashlib.sha256((ROOT/'Configuration/sts-live.json').read_bytes()).hexdigest()
(HERE/'result.json').write_text(json.dumps(report,indent=2)+'\n')
