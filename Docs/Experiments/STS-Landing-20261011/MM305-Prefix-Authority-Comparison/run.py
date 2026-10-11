#!/usr/bin/env python3
"""Passively compare the existing baseline and longest-prefix rejected trial."""
import collections,csv,hashlib,json,math,os,re,shlex,subprocess,tempfile
from pathlib import Path
HERE=Path(__file__).resolve().parent;ROOT=HERE.parents[3]
trace_runner=HERE.parent/'MM305-Profile-Authority-Trace/run.py'
prefix_runner=HERE.parent/'MM305-Unbanked-Prefix/run.py'
# Reuse the exact prior temporary source builders, stopping before their build/run.
trace_ns={'__file__':str(trace_runner)}
exec(compile(trace_runner.read_text().split("report={'sourceCommit':",1)[0],str(trace_runner),'exec'),trace_ns)
solver=trace_ns['modified'];needle='fabs(route->length_m-62326.028)<0.002 &&\n        fabs(route->profile_midpoint_offset_m+1150.990)<0.002'
assert solver.count(needle)==1
solver=solver.replace(needle,'fabs(route->length_m-strtod(getenv("KSL_TRACE_ROUTE"),NULL))<0.002 &&\n        fabs(route->profile_midpoint_offset_m-strtod(getenv("KSL_TRACE_BOW"),NULL))<0.002')
prefix_ns={'__file__':str(prefix_runner)}
exec(compile(prefix_runner.read_text().split("report={'sourceCommit':",1)[0],str(prefix_runner),'exec'),prefix_ns)
fixture=prefix_ns['modified']
report={'sourceCommit':subprocess.check_output(['git','rev-parse','HEAD'],cwd=ROOT,text=True).strip(),'policy':'Passive authority instrumentation, same temporary staged prefix fixture. No production command, propagation or gate changes. Compare best-tracking completed smooth candidates, not approved routes. Maximum table lift is permitted-incidence envelope ignoring g cap and servo; actual incidence may exceed cap transiently. Prefix counts and all smooth trial rows must match the prior uninstrumented runs.', 'hashes':{'instrumentedSolver':hashlib.sha256(solver.encode()).hexdigest(),'fixture':hashlib.sha256(fixture.encode()).hexdigest(),**{str(p.relative_to(ROOT)):hashlib.sha256(p.read_bytes()).hexdigest() for p in [trace_runner,prefix_runner,ROOT/'CLanding/terminal_solver.c',ROOT/'Configuration/sts-live.json',ROOT/'ShuttleSim/reference-model/sts_aero_live20261010.csv',ROOT/'ShuttleSim/reference-model/stsn_attitude_identified.ini',ROOT/'ShuttleSim/reference-model/kerbin_atmosphere_identified.csv']}},'runs':[]}
subprocess.run(['make','-C','CLanding'],cwd=ROOT,check=True,stdout=subprocess.DEVNULL)
dry=subprocess.check_output(['make','-n','-C','CLanding','mm305-feasibility-test'],cwd=ROOT,text=True).replace('\\\n',' ')
link=next(shlex.split(s) for s in dry.splitlines() if s.startswith('clang ') and 'MM305FeasibilityPropertyTests.c' in s)
with tempfile.TemporaryDirectory(prefix='ksl-prefix-authority-') as temp:
 temp=Path(temp);s=temp/'solver.c';s.write_text(solver);obj=temp/'solver.o';f=temp/'fixture.c';f.write_text(fixture);binary=temp/'fixture'
 flags=link[1:link.index('../Validation/MM305FeasibilityPropertyTests.c')]
 subprocess.run(['clang',*flags,'-c',str(s),'-o',str(obj)],cwd=ROOT/'CLanding',check=True)
 link=[str(obj) if x=='build/terminal_solver.o' else str(f) if x=='../Validation/MM305FeasibilityPropertyTests.c' else x for x in link];link[link.index('-o')+1]=str(binary)
 subprocess.run(link,cwd=ROOT/'CLanding',check=True);report['binarySHA256']=hashlib.sha256(binary.read_bytes()).hexdigest()
 for name,distance,aoa,route,bow in [('baseline',0,18,62326.028,-1150.990),('prefix12k-aoa10',12000,10,53174.425,-1041.155)]:
  env=dict(os.environ,KSP_LANDER_MODEL_PROFILE='sts-live-20261010',KSP_LANDER_TAEM_PROFILE_DIAGNOSTICS='1',KSP_LANDER_TAEM_DIAGNOSTICS='1',KSL_EXPERIMENT_PREFIX_M=str(distance),KSL_EXPERIMENT_PREFIX_AOA=str(aoa),KSL_TRACE_ROUTE=str(route),KSL_TRACE_BOW=str(bow))
  log=HERE/f'{name}.log';print('running',name,flush=True)
  with log.open('w') as out:result=subprocess.run([str(binary),'--configuration','Configuration/sts-live.json','--mass-kg','25123.15','--sample-index','0'],cwd=ROOT,env=env,stdout=out,stderr=subprocess.STDOUT)
  lines=log.read_text().splitlines();prior=(HERE.parent/'MM305-Unbanked-Prefix'/f'{name}.log').read_text().splitlines()
  trials=[s for s in lines if s.startswith('TAEM smooth trial:')]
  assert result.returncode==1 and len(trials)==2184
  assert trials==[s for s in prior if s.startswith('TAEM smooth trial:')]
  assert next(s for s in lines if s.startswith('PREFIX_EXIT'))==next(s for s in prior if s.startswith('PREFIX_EXIT'))
  rows=collections.defaultdict(list)
  for line in lines:
   if not line.startswith('PROFILE_AUTHORITY'):continue
   r={k:float(v) for k,v in re.findall(r'(\w+)=([^\s]+)',line)}
   r['altitudeError']=r['altitude']-r['refAltitude'];r['requiredVector']=math.hypot(r['requiredVertical'],r['requiredLateral']);r['actualVertical']=r['liftAccel']*math.cos(math.radians(r['bank']));r['optimisticLiftShortfall']=max(0,r['requiredVector']-r['maximumLift']);rows[int(r['id'])].append(r)
  assert rows
  run={'name':name,'routeM':route,'bowM':bow,'exitCode':result.returncode,'logSHA256':hashlib.sha256(log.read_bytes()).hexdigest(),'unchangedTrialRows':len(trials),'prefixExit':next(s for s in lines if s.startswith('PREFIX_EXIT')),'traces':[]}
  for identity,rs in rows.items():
   first=next((r for r in rs if abs(r['altitudeError'])>300),None);negative=next((r for r in rs if r['altitudeError'] < -300),None);short=next((r for r in rs if r['optimisticLiftShortfall']>.25),None);worst=max(rs,key=lambda r:abs(r['altitudeError']));maximum=max(rs,key=lambda r:r['optimisticLiftShortfall']);crossing=next((i for i,r in enumerate(rs) if r['mach']<1),None)
   summary={'id':identity,'samples':len(rs),'start':rs[0],'firstTrackingLimitCrossing':first,'firstBelowReferenceLimit':negative,'firstOptimisticLiftShortfall':short,'largestTrackingDeparture':worst,'maximumOptimisticLiftShortfall':maximum,'shortfallDurationSeconds':sum(r['dt'] for r in rs if r['optimisticLiftShortfall']>.25),'machOneCrossing':rs[max(0,crossing-2):crossing+3] if crossing is not None else [],'lastPreExit':rs[-1]};run['traces'].append(summary)
   selected=[];last=-math.inf
   for r in rs:
    if r['elapsed']-last>=5 or any(r is e for e in [first,negative,short,worst,maximum,rs[-1]]):selected.append(r);last=r['elapsed']
   with (HERE/f'{name}-trace-{identity}.csv').open('w') as out:
    writer=csv.DictWriter(out,fieldnames=rs[0].keys());writer.writeheader();writer.writerows(selected)
   print(json.dumps({'name':name,'id':identity,'firstTrackingLimitCrossing':first,'firstOptimisticLiftShortfall':short,'largestTrackingDeparture':worst,'maximumOptimisticLiftShortfall':maximum,'shortfallDurationSeconds':summary['shortfallDurationSeconds']}),flush=True)
  report['runs'].append(run);(HERE/'result.json').write_text(json.dumps(report,indent=2)+'\n')
