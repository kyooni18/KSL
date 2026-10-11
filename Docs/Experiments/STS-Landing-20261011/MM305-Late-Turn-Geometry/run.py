#!/usr/bin/env python3
"""Test a stricter initial lead-prefix curvature envelope in temporary geometry."""
import hashlib,json,os,re,shlex,subprocess,tempfile
from pathlib import Path
HERE=Path(__file__).resolve().parent;ROOT=HERE.parents[3]
p=ROOT/'CLanding/taem_planner.c';source=p.read_text()
a=source.index('static double cubic_endpoint_curvature(');b=source.index('static bool cubic_arc_lookup',a);part=source[a:b]
needle='    return fabs(d.x*dd.y - d.y*dd.x) / (speed*speed*speed);'
assert part.count(needle)==1
code='''    double value=fabs(d.x*dd.y-d.y*dd.x)/(speed*speed*speed);
    if(t==0.0 && getenv("KSL_EXPERIMENT_LEAD_PREFIX_M")){
        double prefix=strtod(getenv("KSL_EXPERIMENT_LEAD_PREFIX_M"),NULL);
        double reserve=strtod(getenv("KSL_EXPERIMENT_LEAD_RESERVE"),NULL);
        Point previous=p0;
        double length=0.0, peak=value;
        for(int i=1;i<=128;++i){
            double u=(double)i/128.0;
            Point point=cubic(p0,p1,p2,p3,u);
            length+=distance(previous,point);previous=point;
            Point tangent=cubic_derivative(p0,p1,p2,p3,u);
            Point second=cubic_second_derivative(p0,p1,p2,p3,u);
            double norm=hypot(tangent.x,tangent.y);
            if(norm<1.0)return INFINITY;
            double curvature=fabs(tangent.x*second.y-tangent.y*second.x)/
                (norm*norm*norm);
            peak=fmax(peak,curvature);
            if(length>=prefix)break;
        }
        return peak/reserve;
    }
    return value;'''
modified='#include <stdlib.h>\n'+source[:a]+part.replace(needle,code)+source[b:]
report={'sourceCommit':subprocess.check_output(['git','rev-parse','HEAD'],cwd=ROOT,text=True).strip(),'policy':'Temporary cubic geometry selection. Prefix curvature/current inlet authority constrained more strictly, suppressing early turn. Original geometry regularity, peak bounds, profile, speed, Final and landing gates unchanged. Prefix bound is deliberately conservative as density evolves; not a new physical impossibility criterion. Focused sample0 only.', 'sourceSHA256':hashlib.sha256(source.encode()).hexdigest(),'experimentSourceSHA256':hashlib.sha256(modified.encode()).hexdigest(),'inputHashes':{n:hashlib.sha256((ROOT/n).read_bytes()).hexdigest() for n in ['Configuration/sts-live.json','ShuttleSim/reference-model/sts_aero_live20261010.csv','ShuttleSim/reference-model/stsn_attitude_identified.ini','ShuttleSim/reference-model/kerbin_atmosphere_identified.csv','Validation/MM305FeasibilityPropertyTests.c']},'runs':[]}
subprocess.run(['make','-C','CLanding'],cwd=ROOT,check=True,stdout=subprocess.DEVNULL)
dry=subprocess.check_output(['make','-n','-C','CLanding','mm305-feasibility-test'],cwd=ROOT,text=True).replace('\\\n',' ')
link=next(shlex.split(s) for s in dry.splitlines() if s.startswith('clang ') and 'MM305FeasibilityPropertyTests.c' in s)
with tempfile.TemporaryDirectory(prefix='ksl-late-turn-') as temp:
 temp=Path(temp);src=temp/'planner.c';obj=temp/'planner.o';binary=temp/'fixture';src.write_text(modified)
 flags=link[1:link.index('../Validation/MM305FeasibilityPropertyTests.c')]
 subprocess.run(['clang',*flags,'-c',str(src),'-o',str(obj)],cwd=ROOT/'CLanding',check=True)
 link=[str(obj) if x=='build/taem_planner.o' else x for x in link];link[link.index('-o')+1]=str(binary)
 subprocess.run(link,cwd=ROOT/'CLanding',check=True);report['binarySHA256']=hashlib.sha256(binary.read_bytes()).hexdigest()
 for name,prefix,reserve in [('baseline',None,None),('prefix4k',4000,.5),('prefix8k',8000,.5),('prefix8k-full',8000,1.0)]:
  env=dict(os.environ,KSP_LANDER_MODEL_PROFILE='sts-live-20261010',KSP_LANDER_TAEM_PROFILE_DIAGNOSTICS='1',KSP_LANDER_TAEM_DIAGNOSTICS='1');env.pop('KSL_EXPERIMENT_LEAD_PREFIX_M',None);env.pop('KSL_EXPERIMENT_LEAD_RESERVE',None)
  if prefix is not None:env.update(KSL_EXPERIMENT_LEAD_PREFIX_M=str(prefix),KSL_EXPERIMENT_LEAD_RESERVE=str(reserve))
  log=HERE/f'{name}.log';args=[str(binary),'--configuration','Configuration/sts-live.json','--mass-kg','25123.15','--sample-index','0'];print('running',name,flush=True)
  with log.open('w') as out:r=subprocess.run(args,cwd=ROOT,env=env,stdout=out,stderr=subprocess.STDOUT)
  trials=[];geometries=[]
  for line in log.read_text().splitlines():
   if line.startswith('TAEM smooth trial:'):trials.append({k:v if k=='reason' else float(v) for k,v in re.findall(r'(\w+)=([^\s]+)',line)})
   if line.startswith('TAEM geometry:'):geometries.append({k:float(v) for k,v in re.findall(r'(\w+)=([^\s]+)',line)})
  complete=[t for t in trials if t['completed']];feasible=[t for t in trials if t['feasible']]
  result={'name':name,'prefixM':prefix,'inletAuthorityFraction':reserve,'exitCode':r.returncode,'logSHA256':hashlib.sha256(log.read_bytes()).hexdigest(),'trials':len(trials),'completed':len(complete),'referenceFeasible':len(feasible),'smallestTrackingErrorCompleted':min(complete,key=lambda t:t['track']) if complete else None,'closestAltitudeCompleted':min(complete,key=lambda t:t['endpoint']) if complete else None,'geometryExamples':geometries[:10],'feasibleExamples':feasible[:10],'contract':next((s for s in log.read_text().splitlines() if s.startswith('MM305 downstream-feasibility contract:')),None)}
  if name=='baseline':assert result['trials']==2184 and result['completed']==1287
  report['runs'].append(result);(HERE/'result.json').write_text(json.dumps(report,indent=2)+'\n');print(json.dumps({k:v for k,v in result.items() if k!='geometryExamples'}),flush=True)
