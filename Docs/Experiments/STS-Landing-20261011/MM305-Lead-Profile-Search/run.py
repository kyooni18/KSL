#!/usr/bin/env python3
"""Compile a temporary search experiment without changing production files."""
import hashlib,json,os,re,shlex,subprocess,tempfile
from pathlib import Path
HERE=Path(__file__).resolve().parent;ROOT=HERE.parents[3]
solver=ROOT/'CLanding/terminal_solver.c'
text=solver.read_text()
old='    trial.profile_local_offset_m = 0.0;'
assert text.count(old)==1
text=text.replace(old,'    trial.profile_local_offset_m = drop * strtod(getenv("KSL_EXPERIMENT_LEAD_BOW_FRACTION"), NULL);')
# Tabulated profile samples preserve the reference; retain analytic metadata too.
text=text.replace('    route->profile_local_offset_m=0.0;','    route->profile_local_offset_m=trial.profile_local_offset_m;')
report={'sourceCommit':subprocess.check_output(['git','rev-parse','HEAD'],cwd=ROOT,text=True).strip(),
 'policy':'Temporary solver object, unchanged production backend and gates. Focused sample0, all midpoint bows per native search, local lead shape start.08 peak.24 end.40. Signed offsets are fractions of initial-to-Final altitude drop. Not production adoption or full qualification.',
 'sourceSHA256':hashlib.sha256(solver.read_bytes()).hexdigest(),'experimentSourceSHA256':hashlib.sha256(text.encode()).hexdigest(),'runs':[]}
report['inputHashes']={name:hashlib.sha256((ROOT/name).read_bytes()).hexdigest() for name in ['Configuration/sts-live.json','ShuttleSim/reference-model/sts_aero_live20261010.csv','ShuttleSim/reference-model/stsn_attitude_identified.ini','ShuttleSim/reference-model/kerbin_atmosphere_identified.csv','Validation/MM305FeasibilityPropertyTests.c']}
subprocess.run(['make','-C','CLanding'],cwd=ROOT,check=True,stdout=subprocess.DEVNULL)
dry=subprocess.check_output(['make','-n','-C','CLanding','mm305-feasibility-test'],cwd=ROOT,text=True).replace('\\\n',' ')
link=next(shlex.split(line) for line in dry.splitlines() if line.startswith('clang ') and 'MM305FeasibilityPropertyTests.c' in line)
with tempfile.TemporaryDirectory(prefix='ksl-lead-search-') as temp:
 temp=Path(temp);src=temp/'solver.c';src.write_text(text);obj=temp/'solver.o';binary=temp/'fixture'
 flags=link[1:link.index('../Validation/MM305FeasibilityPropertyTests.c')]
 subprocess.run(['clang',*flags,'-c',str(src),'-o',str(obj)],cwd=ROOT/'CLanding',check=True)
 link=[str(obj) if x=='build/terminal_solver.o' else x for x in link]
 link[link.index('-o')+1]=str(binary)
 subprocess.run(link,cwd=ROOT/'CLanding',check=True)
 report['binarySHA256']=hashlib.sha256(binary.read_bytes()).hexdigest()
 for fraction in [0,-.1,-.2,.1,.2]:
  env=dict(os.environ,KSP_LANDER_MODEL_PROFILE='sts-live-20261010',KSP_LANDER_TAEM_PROFILE_DIAGNOSTICS='1',KSL_EXPERIMENT_LEAD_BOW_FRACTION=str(fraction))
  args=[str(binary),'--configuration','Configuration/sts-live.json','--mass-kg','25123.15','--sample-index','0']
  path=HERE/f'fraction-{fraction:g}.log'
  print('running',fraction,flush=True)
  with path.open('w') as out:completed=subprocess.run(args,cwd=ROOT,env=env,stdout=out,stderr=subprocess.STDOUT)
  rows=[]
  for line in path.read_text().splitlines():
   if 'TAEM smooth trial:' in line:rows.append({k:v if k=='reason' else float(v) for k,v in re.findall(r'(\w+)=([^\s]+)',line)})
  valid=[r for r in rows if r['completed']]
  result={'leadBowFraction':fraction,'exitCode':completed.returncode,'logSHA256':hashlib.sha256(path.read_bytes()).hexdigest(),'trials':len(rows),'completed':len(valid),'jointlyFeasible':sum(r['feasible'] for r in rows),'closestAltitudeCompleted':min(valid,key=lambda r:r['endpoint']) if valid else None,'smallestTrackingErrorCompleted':min(valid,key=lambda r:r['track']) if valid else None,'contract':next((s for s in path.read_text().splitlines() if s.startswith('MM305 downstream-feasibility contract:')),None)}
  if fraction==0:
   assert result['trials']==2184 and result['completed']==1287 and result['closestAltitudeCompleted']['speed']==95.251
  report['runs'].append(result);(HERE/'result.json').write_text(json.dumps(report,indent=2)+'\n');print(json.dumps(result),flush=True)
