#!/usr/bin/env python3
"""Locate path departure and joint lift/energy state in passive solver traces."""
import collections,csv,json,math,re
from pathlib import Path
HERE=Path(__file__).resolve().parent
rows=collections.defaultdict(list)
for line in (HERE/'trace.log').read_text().splitlines():
 if not line.startswith('PROFILE_AUTHORITY'):continue
 r={k:float(v) for k,v in re.findall(r'(\w+)=([^\s]+)',line)}
 r['altitudeError']=r['altitude']-r['refAltitude']
 r['requiredVector']=math.hypot(r['requiredVertical'],r['requiredLateral'])
 r['actualVertical']=r['liftAccel']*math.cos(math.radians(r['bank']))
 r['optimisticLiftShortfall']=max(0,r['requiredVector']-r['maximumLift'])
 rows[int(r['id'])].append(r)
assert rows
report={'policy':'Instrumented smooth trial, route62326.028m midpoint bow-1150.990m. Existing model and constraints. Maximum table lift is restricted to permitted commanded incidence; excludes g cap and roll/servo limits. Actual incidence may exceed that cap transiently. Demand exceeding it is a command-envelope shortfall, not global physical impossibility.', 'traces':[]}
for identity,rs in rows.items():
 first=next((r for r in rs if abs(r['altitudeError'])>300),None)
 worst=max(rs,key=lambda r:abs(r['altitudeError']))
 short=[r for r in rs if r['optimisticLiftShortfall']>.25]
 crossing=next((i for i,r in enumerate(rs) if r['mach']<1),None)
 report['traces'].append({'machOneCrossing':rs[max(0,crossing-2):crossing+3] if crossing is not None else [],'id':identity,'samples':len(rs),'firstTrackingLimitCrossing':first,'largestTrackingDeparture':worst,'start':rs[0],'lastPreExit':rs[-1], 'optimisticLiftShortfallDurationSeconds':sum(r['dt'] for r in short), 'maximumOptimisticLiftShortfall':max(rs,key=lambda r:r['optimisticLiftShortfall']),'maximumAoACommandErrorDeg':max(abs(r['cmdAoa']-r['aoa']) for r in rs)})
 selected=[];last=-math.inf
 for r in rs:
  if r['elapsed']-last>=5 or r is first or r is worst or r is rs[-1]:selected.append(r);last=r['elapsed']
 with (HERE/f'trace-{identity}.csv').open('w') as f:
  writer=csv.DictWriter(f,fieldnames=rs[0].keys());writer.writeheader();writer.writerows(selected)
 print(json.dumps(report['traces'][-1]),flush=True)
(HERE/'result.json').write_text(json.dumps(report,indent=2)+'\n')
