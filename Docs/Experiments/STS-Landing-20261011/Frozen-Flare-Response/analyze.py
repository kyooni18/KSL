#!/usr/bin/env python3
"""Retain frozen histories and actual same-flight post-seed drag work."""
import bisect,csv,gzip,json,math,re
from pathlib import Path
HERE=Path(__file__).resolve().parent
r=json.loads((HERE/'result.json').read_text())
for run in r['runs']:
 result=run['runResult'];cases=[];current=[]
 for line in Path(result['backendStderr']).read_text().splitlines():
  if line.startswith('FROZEN_STEP'):current.append({k:float(v) for k,v in re.findall(r'(\w+)=([^\s]+)',line)})
  elif line.startswith('FROZEN_RESULT'):cases.append(current);current=[]
 assert len(cases)==len(run['forecasts'])
 rows=[]
 for i,(forecast,history) in enumerate(zip(run['forecasts'],cases)):
  rows.extend({'caseIndex':i,'trigger':forecast['trigger'],**p} for p in history)
 with (HERE/f"{run['name']}-frozen-history.csv").open('w') as f:
  w=csv.DictWriter(f,fieldnames=rows[0].keys());w.writeheader();w.writerows(rows)
 with gzip.open(result['sim'],'rt') as f:packets=[json.loads(s) for s in f if s.strip()]
 contact=next(p for p in packets if p.get('ground',{}).get('touchdown_seen'))
 seed=min(packets,key=lambda p:abs(p['ut']-run['targetUT']))
 native=[p for p in packets if seed['ut']<=p['ut']<=contact['ut']]
 def rate(p):return p['aero']['drag_n']*p['velocity']['air_mps']/p['vehicle']['mass_kg']
 work=sum(.5*(rate(a)+rate(b))*(b['ut']-a['ut']) for a,b in zip(native,native[1:]))
 actual=result['simulatorSummary']['touchdown_speed_mps']
 run['actualTail']={'seedUT':seed['ut'],'contactUT':contact['ut'],'elapsedSeconds':contact['ut']-seed['ut'],'dragWorkJPerKg':work,'maximumMeasuredAoADeg':max(p['attitude']['aoa_deg'] for p in native),'touchdownSpeedMps':actual,'touchdownSinkMps':result['simulatorSummary']['touchdown_sink_mps'],'samples':len(native)}
 for forecast in run['forecasts']:forecast['errorAgainstActualSpeedMps']=forecast['speed']-actual
 print(run['name'],run['actualTail'])
(HERE/'result.json').write_text(json.dumps(r,indent=2,allow_nan=False)+'\n')
