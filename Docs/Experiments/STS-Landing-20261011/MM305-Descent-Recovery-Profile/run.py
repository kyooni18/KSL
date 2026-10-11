#!/usr/bin/env python3
"""Offline sustained-descent profile, with explicit experimental slope bound."""
import hashlib,json,os,re,shlex,subprocess,tempfile
from pathlib import Path
HERE=Path(__file__).resolve().parent;ROOT=HERE.parents[3]
p=ROOT/'CLanding/taem_planner.c';source=p.read_text()
needle='    if (altitude) *altitude = height;'
assert source.count(needle)==1
code='''    if(getenv("KSL_EXPERIMENT_DESCENT_RECOVERY")){
        double drop=h0-h1;
        double a=clamp(0.35+midpoint_offset/fmax(drop,1.0),0.05,0.65);
        double b=strtod(getenv("KSL_EXPERIMENT_RECOVERY_START"),NULL);
        a=fmin(a,b-0.05);
        double mean=(h1-h0)/total_length;
        double cruise=(mean-0.5*a*slope0-0.5*(1.0-b)*slope1)/
            (0.5*(1.0+b-a));
        double integral,profile_slope,second_x;
        if(x<a){
            double u=x/a;
            integral=a*(slope0*u+(cruise-slope0)*(u*u*u-0.5*u*u*u*u));
            profile_slope=slope0+(cruise-slope0)*u*u*(3.0-2.0*u);
            second_x=(cruise-slope0)*6.0*u*(1.0-u)/a;
        }else if(x<b){
            integral=0.5*a*(slope0+cruise)+(x-a)*cruise;
            profile_slope=cruise;second_x=0.0;
        }else{
            double u=(x-b)/(1.0-b);
            integral=0.5*a*(slope0+cruise)+(b-a)*cruise+
                (1.0-b)*(cruise*u+(slope1-cruise)*(u*u*u-0.5*u*u*u*u));
            profile_slope=cruise+(slope1-cruise)*u*u*(3.0-2.0*u);
            second_x=(slope1-cruise)*6.0*u*(1.0-u)/(1.0-b);
        }
        height=h0+total_length*integral+initial_sag*initial_bow;
        slope=profile_slope+initial_slope;
        second_derivative_x=second_x*total_length;
    }
'''
modified='#include <stdlib.h>\n'+source.replace(needle,code+needle)
report={'sourceCommit':subprocess.check_output(['git','rev-parse','HEAD'],cwd=ROOT,text=True).strip(),'policy':'Temporary analytic planner profile. Initial and Final altitude/FPA preserved. 13 midpoint bow values instead parameterize descent-ramp fraction .05-.65. Constant slope midsegment then recovery. Initial sag retained. Explicit experimental36deg reference bound, not production24deg qualification. All other gates unchanged. Focused sample0 only.','sourceSHA256':hashlib.sha256(source.encode()).hexdigest(),'experimentSourceSHA256':hashlib.sha256(modified.encode()).hexdigest(),'inputHashes':{n:hashlib.sha256((ROOT/n).read_bytes()).hexdigest() for n in ['Configuration/sts-live.json','ShuttleSim/reference-model/sts_aero_live20261010.csv','ShuttleSim/reference-model/stsn_attitude_identified.ini','ShuttleSim/reference-model/kerbin_atmosphere_identified.csv','Validation/MM305FeasibilityPropertyTests.c']},'runs':[]}
subprocess.run(['make','-C','CLanding'],cwd=ROOT,check=True,stdout=subprocess.DEVNULL)
dry=subprocess.check_output(['make','-n','-C','CLanding','mm305-feasibility-test'],cwd=ROOT,text=True).replace('\\\n',' ')
link=next(shlex.split(s) for s in dry.splitlines() if s.startswith('clang ') and 'MM305FeasibilityPropertyTests.c' in s)
with tempfile.TemporaryDirectory(prefix='ksl-descent-recovery-') as temp:
 temp=Path(temp);src=temp/'planner.c';obj=temp/'planner.o';binary=temp/'fixture';src.write_text(modified)
 flags=link[1:link.index('../Validation/MM305FeasibilityPropertyTests.c')]
 subprocess.run(['clang',*flags,'-c',str(src),'-o',str(obj)],cwd=ROOT/'CLanding',check=True)
 link=[str(obj) if x=='build/taem_planner.o' else x for x in link];link[link.index('-o')+1]=str(binary)
 subprocess.run(link,cwd=ROOT/'CLanding',check=True);report['binarySHA256']=hashlib.sha256(binary.read_bytes()).hexdigest()
 for name,limit,recovery in [('baseline',24,None),('shape24',24,.7),('shape36',36,.7),('shape36-late',36,.85)]:
  cfg=json.loads((ROOT/'Configuration/sts-live.json').read_text());cfg['guidance']['taemGlideSlope']=limit
  config=temp/f'{name}.json';config.write_text(json.dumps(cfg))
  env=dict(os.environ,KSP_LANDER_MODEL_PROFILE='sts-live-20261010',KSP_LANDER_TAEM_PROFILE_DIAGNOSTICS='1')
  env.pop('KSL_EXPERIMENT_DESCENT_RECOVERY',None);env.pop('KSL_EXPERIMENT_RECOVERY_START',None)
  if recovery is not None:env.update(KSL_EXPERIMENT_DESCENT_RECOVERY='1',KSL_EXPERIMENT_RECOVERY_START=str(recovery))
  args=[str(binary),'--configuration',str(config),'--mass-kg','25123.15','--sample-index','0'];log=HERE/f'{name}.log'
  print('running',name,flush=True)
  with log.open('w') as out:r=subprocess.run(args,cwd=ROOT,env=env,stdout=out,stderr=subprocess.STDOUT)
  trials=[]
  for line in log.read_text().splitlines():
   if line.startswith('TAEM smooth trial:'):trials.append({k:v if k=='reason' else float(v) for k,v in re.findall(r'(\w+)=([^\s]+)',line)})
  complete=[t for t in trials if t['completed']];feasible=[t for t in trials if t['feasible']]
  result={'name':name,'referenceSlopeLimitDeg':limit,'recoveryStartFraction':recovery,'exitCode':r.returncode,'configurationSHA256':hashlib.sha256(config.read_bytes()).hexdigest(),'logSHA256':hashlib.sha256(log.read_bytes()).hexdigest(),'trials':len(trials),'completed':len(complete),'referenceFeasible':len(feasible),'smallestTrackingErrorCompleted':min(complete,key=lambda t:t['track']) if complete else None,'closestAltitudeCompleted':min(complete,key=lambda t:t['endpoint']) if complete else None,'feasibleExamples':feasible[:10],'contract':next((s for s in log.read_text().splitlines() if s.startswith('MM305 downstream-feasibility contract:')),None)}
  if name=='baseline':assert result['trials']==2184 and result['completed']==1287 and result['smallestTrackingErrorCompleted']['track']==3040.205
  report['runs'].append(result);(HERE/'result.json').write_text(json.dumps(report,indent=2)+'\n');print(json.dumps(result),flush=True)
