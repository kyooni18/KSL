#!/usr/bin/env python3
"""Expand radius coverage in a temporary native admission fixture."""
import hashlib,json,os,re,shlex,subprocess,tempfile
from pathlib import Path
HERE=Path(__file__).resolve().parent;ROOT=HERE.parents[3]
path=ROOT/'CLanding/taem_candidate_search.c';source=path.read_text()
needle='''                        *slot = tighter;
                }
'''
assert source.count(needle)==1
extra='''                for(int extra_step=1;extra_step<=2;++extra_step){
                    double larger_radius=hac_radius*(1.0+0.5*extra_step);
                    TaemFixedHacCandidate larger=evaluate_side(ends[end],
                        initial,&geometry,larger_radius,side,spacing,dt,
                        maximum_elapsed,stations);
                    if(taem_fixed_hac_candidate_preferred(ends[end],&larger,slot)||
                        (slot->status!=TAEM_PLAN_UNQUALIFIED &&
                         larger.route_built && !slot->route_built))
                        *slot=larger;
                }
'''
modified=source.replace(needle,needle+extra)
report={'sourceCommit':subprocess.check_output(['git','rev-parse','HEAD'],cwd=ROOT,text=True).strip(),
 'policy':'Temporary candidate-search object. Retains original 7 radii and adds 1.5x/2x configured radius; unchanged midpoint bow/profile/replay/Final gates. Focused STS sample0 only. No production adoption.',
 'sourceSHA256':hashlib.sha256(source.encode()).hexdigest(),'experimentSourceSHA256':hashlib.sha256(modified.encode()).hexdigest(),
 'inputHashes':{n:hashlib.sha256((ROOT/n).read_bytes()).hexdigest() for n in ['Configuration/sts-live.json','ShuttleSim/reference-model/sts_aero_live20261010.csv','ShuttleSim/reference-model/stsn_attitude_identified.ini','ShuttleSim/reference-model/kerbin_atmosphere_identified.csv','Validation/MM305FeasibilityPropertyTests.c']}}
subprocess.run(['make','-C','CLanding'],cwd=ROOT,check=True,stdout=subprocess.DEVNULL)
dry=subprocess.check_output(['make','-n','-C','CLanding','mm305-feasibility-test'],cwd=ROOT,text=True).replace('\\\n',' ')
link=next(shlex.split(s) for s in dry.splitlines() if s.startswith('clang ') and 'MM305FeasibilityPropertyTests.c' in s)
with tempfile.TemporaryDirectory(prefix='ksl-radius-search-') as temp:
 temp=Path(temp);src=temp/'search.c';obj=temp/'search.o';binary=temp/'fixture';src.write_text(modified)
 flags=link[1:link.index('../Validation/MM305FeasibilityPropertyTests.c')]
 subprocess.run(['clang',*flags,'-c',str(src),'-o',str(obj)],cwd=ROOT/'CLanding',check=True)
 link=[str(obj) if s=='build/taem_candidate_search.o' else s for s in link];link[link.index('-o')+1]=str(binary)
 subprocess.run(link,cwd=ROOT/'CLanding',check=True)
 report['binarySHA256']=hashlib.sha256(binary.read_bytes()).hexdigest()
 env=dict(os.environ,KSP_LANDER_MODEL_PROFILE='sts-live-20261010',KSP_LANDER_TAEM_PROFILE_DIAGNOSTICS='1',KSP_LANDER_TAEM_DIAGNOSTICS='1')
 args=[str(binary),'--configuration','Configuration/sts-live.json','--mass-kg','25123.15','--sample-index','0']
 report['environment']={k:env[k] for k in ['KSP_LANDER_MODEL_PROFILE','KSP_LANDER_TAEM_PROFILE_DIAGNOSTICS','KSP_LANDER_TAEM_DIAGNOSTICS']};report['arguments']=args[1:]
 log=HERE/'search.log'
 with log.open('w') as out:completed=subprocess.run(args,cwd=ROOT,env=env,stdout=out,stderr=subprocess.STDOUT)
 report['exitCode']=completed.returncode;report['logSHA256']=hashlib.sha256(log.read_bytes()).hexdigest()
 trials=[];profiles=[];radius=None
 for line in log.read_text().splitlines():
  if line.startswith('TAEM geometry:'):radius=float(re.search(r'radius=([^ ]+)',line).group(1))
  if line.startswith('TAEM smooth trial:'):
   r={k:v if k=='reason' else float(v) for k,v in re.findall(r'(\w+)=([^\s]+)',line)};r['radius']=radius;trials.append(r)
  if line.startswith('TAEM profile:'):
   r={k:float(v) for k,v in re.findall(r'(\w+)=([^\s]+)',line) if k!='reason'};r['reason']=line.split('reason=',1)[1];profiles.append(r)
 report['contract']=next((s for s in log.read_text().splitlines() if s.startswith('MM305 downstream-feasibility contract:')),None)
 report['smoothByRadius']={}
 for radius in sorted({r['radius'] for r in trials}):
  rs=[r for r in trials if r['radius']==radius];valid=[r for r in rs if r['completed']]
  report['smoothByRadius'][str(radius)]={'trials':len(rs),'completed':len(valid),'jointlyFeasible':sum(r['feasible'] for r in rs),'smallestTrackingErrorCompleted':min(valid,key=lambda r:r['track']) if valid else None}
 report['profileSummaries']=profiles
 assert set([18000.,24000.]) <= {r['radius'] for r in profiles}
 (HERE/'result.json').write_text(json.dumps(report,indent=2)+'\n')
 print(json.dumps({'exitCode':completed.returncode,'contract':report['contract'],'smoothByRadius':report['smoothByRadius']}),flush=True)
