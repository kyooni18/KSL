#!/usr/bin/env python3
"""Compare existing native Final trace forecasts with their same replay touchdown."""
import bisect,csv,gzip,hashlib,json,math,re,subprocess,sys
from pathlib import Path
HERE=Path(__file__).resolve().parent;ROOT=HERE.parents[3]
assert len(sys.argv)==3,'supply L5 and L7 simulation-result.json paths'
report={'sourceCommit':subprocess.check_output(['git','rev-parse','HEAD'],cwd=ROOT,text=True).strip(),'policy':'Unmodified closed-loop l5-mm305/l7-mm305 native FCS runs with FINAL_TRACE. Same actual propagated state and learned history; no reanchoring/force reset. Conditional point-mass forecast compared to eventual same-flight contact, not frozen-plan or full-controller forecast accuracy certification. Legacy surrogate attitude plants are unqualified for live tuning. Trace values rounded0.1m/s.', 'runs':[]}
for path in map(Path,sys.argv[1:]):
 result=json.loads(path.read_text());summary=result['simulatorSummary'];label='l5' if 'L5' in summary['scenario'] else 'l7'
 stderr=Path(result['backendStderr']);sim=Path(result['sim']);manifest=Path(result['manifest']);provenance=json.loads(manifest.read_text())
 traces=[]
 for line in stderr.read_text().splitlines():
  if line.startswith('FINAL_TRACE'):traces.append({k:v if k=='g' else float(v) for k,v in re.findall(r'(\w+)=([^\s]+)',line)})
 assert traces and len(set(t['g'] for t in traces))==1,'shadow/multiple guidance streams require explicit selection'
 assert all(b['ut']>=a['ut'] for a,b in zip(traces,traces[1:]))
 with gzip.open(sim,'rt') if sim.suffix=='.gz' else sim.open() as f:packets=[json.loads(s) for s in f if s.strip()]
 packets.sort(key=lambda s:s['ut']);times=[s['ut'] for s in packets]
 contact=next(s for s in packets if s.get('ground',{}).get('touchdown_seen'))
 precontact=[t for t in traces if t['ut']<contact['ut']]
 finite=[t for t in precontact if not t['latch'] and math.isfinite(t['vtd'])]
 assert finite
 maximum_pair_error=0
 for t in finite:
  i=bisect.bisect_left(times,t['ut']);candidates=packets[max(0,i-1):i+1];p=min(candidates,key=lambda s:abs(s['ut']-t['ut']))
  assert abs(p['ut']-t['ut'])<=.06
  error=abs(p['velocity']['air_mps']-t['V']);maximum_pair_error=max(maximum_pair_error,error);assert error<=.15
  t['forecastErrorMps']=t['vtd']-summary['touchdown_speed_mps'];t['secondsToContact']=contact['ut']-t['ut']
 first=finite[0];last=finite[-1];latch=next((t for t in precontact if t['latch']),None)
 run={'label':label,'resultPath':str(path.resolve()),'sourceRunId':result['runId'],'summary':summary,'success':result['success'],'contactUT':contact['ut'],'traceSamples':len(traces),'finitePreLatchSamples':len(finite),'maximumTraceTelemetrySpeedErrorMps':maximum_pair_error,'firstFinalForecast':first,'lastPreLatchForecast':last,'firstPullLatch':latch,'firstForecastErrorMps':first['forecastErrorMps'],'lastForecastErrorMps':last['forecastErrorMps'],'traceMatching':'one monotonically advancing guidance stream; every finite pre-latch trace speed paired to actual simulator sample within0.06s and0.15m/s','inputProvenance':{k:provenance[k] for k in ['physicsSources','guidanceModelSources','provenance','config','scenario','physicsDt'] if k in provenance},'hashes':{str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in [path,stderr,sim,manifest]}}
 if label=='l5':
  for key,expected,tol in [('touchdown_speed_mps',66.129,.005),('touchdown_sink_mps',1.844,.005),('stop_along_m',1101.616,.05)]:assert abs(summary[key]-expected)<=tol
  assert result['success']
 else:assert 'Flare' in result['observedPhases'] and summary['touchdown_gear'] and not summary['crashed'] and not result['success']
 selected=[];previous=-math.inf
 for t in finite:
  if t['ut']-previous>=5 or t is first or t is last:selected.append(t);previous=t['ut']
 with (HERE/f'{label}-forecasts.csv').open('w') as f:
  writer=csv.DictWriter(f,fieldnames=first.keys());writer.writeheader();writer.writerows(selected)
 report['runs'].append(run)
 print(json.dumps({'label':label,'actualTouchdownSpeedMps':summary['touchdown_speed_mps'],'firstForecast':first['vtd'],'lastForecast':last['vtd'],'firstError':first['forecastErrorMps'],'lastError':last['forecastErrorMps'],'finiteSamples':len(finite),'actualSink':summary['touchdown_sink_mps'],'actualAlong':summary['touchdown_along_m'],'success':result['success']}),flush=True)
def finite_json(value):
 if isinstance(value,dict):return {k:finite_json(v) for k,v in value.items()}
 if isinstance(value,list):return [finite_json(v) for v in value]
 return None if isinstance(value,float) and not math.isfinite(value) else value
(HERE/'result.json').write_text(json.dumps(finite_json(report),indent=2,allow_nan=False)+'\n')
