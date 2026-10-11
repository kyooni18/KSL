#!/usr/bin/env python3
"""Test a turn-aware lower curvature bound in a temporary planner object."""
import hashlib,json,os,re,shlex,subprocess,tempfile
from pathlib import Path
HERE=Path(__file__).resolve().parent;ROOT=HERE.parents[3]
p=ROOT/'CLanding/taem_planner.c';source=p.read_text()
old='''    if (base_curvature <= curvature_max + 1e-9)
        return true;'''
new='''    double curvature_target=curvature_max;
    bool lower_bound_test=getenv("KSL_EXPERIMENT_INLET_LOWER_BOUND")!=NULL;
    if(lower_bound_test){
        double tangent=tan(bank_limit);
        if(!(tangent>1e-6))return false;
        double vertical_min=lateral_required/tangent;
        double curvature_min=(vertical_min-radial_required)/
            (geometry->airspeed_mps*geometry->ground_speed_mps);
        if(curvature_min>curvature_max)return false;
        curvature_target=fmax(curvature_min,fmin(curvature_max,base_curvature));
        if(fabs(curvature_target-base_curvature)<=1e-9)return true;
        fprintf(stderr,"INLET_CURVATURE base=%.12g min=%.12g max=%.12g target=%.12g lateral=%.6f verticalMin=%.6f verticalMax=%.6f\\n",
            base_curvature,curvature_min,curvature_max,curvature_target,
            lateral_required,vertical_min,vertical_available);
    }else if (base_curvature <= curvature_max + 1e-9)
        return true;'''
assert source.count(old)==1
modified="#include <stdlib.h>\n"+source.replace(old,new)
needle='(curvature_max - base_curvature) * (1.0 + slope0 * slope0)'
assert modified.count(needle)==1
modified=modified.replace(needle,'(curvature_target - base_curvature) * (1.0 + slope0 * slope0)')
report={'sourceCommit':subprocess.check_output(['git','rev-parse','HEAD'],cwd=ROOT,text=True).strip(),
 'policy':'Temporary planner object. Baseline unchanged; experiment clips initial profile curvature between permitted-lift upper bound and minimum vertical lift required for lateral turn inside bank limit. Retains finite sag correction and all profile/replay/Final gates. Focused sample0 only.',
 'sourceSHA256':hashlib.sha256(source.encode()).hexdigest(),'experimentSourceSHA256':hashlib.sha256(modified.encode()).hexdigest(),
 'inputHashes':{n:hashlib.sha256((ROOT/n).read_bytes()).hexdigest() for n in ['Configuration/sts-live.json','ShuttleSim/reference-model/sts_aero_live20261010.csv','ShuttleSim/reference-model/stsn_attitude_identified.ini','ShuttleSim/reference-model/kerbin_atmosphere_identified.csv','Validation/MM305FeasibilityPropertyTests.c']},'runs':[]}
subprocess.run(['make','-C','CLanding'],cwd=ROOT,check=True,stdout=subprocess.DEVNULL)
dry=subprocess.check_output(['make','-n','-C','CLanding','mm305-feasibility-test'],cwd=ROOT,text=True).replace('\\\n',' ')
link=next(shlex.split(s) for s in dry.splitlines() if s.startswith('clang ') and 'MM305FeasibilityPropertyTests.c' in s)
with tempfile.TemporaryDirectory(prefix='ksl-curvature-bounds-') as temp:
 temp=Path(temp);src=temp/'planner.c';obj=temp/'planner.o';binary=temp/'fixture';src.write_text(modified)
 flags=link[1:link.index('../Validation/MM305FeasibilityPropertyTests.c')]
 subprocess.run(['clang',*flags,'-c',str(src),'-o',str(obj)],cwd=ROOT/'CLanding',check=True)
 solver_source=(ROOT/'CLanding/terminal_solver.c').read_text()
 a=solver_source.index('static ProfileFlight fly_smooth_profile(');b=solver_source.index('static bool smooth_flight_ok',a)
 part=solver_source[a:b]
 marker='        if (terminal_propagator_step(m, &state, &demand.control) != TERMINAL_STEP_OK) {'
 assert part.count(marker)==1
 trace_code='        if(fabs(route->length_m-62326.028)<0.002 && fabs(route->profile_midpoint_offset_m+1150.990)<0.002 && step<=20)\n            fprintf(stderr,"BOUND_TRACE step=%zu altitude=%.9f refAltitude=%.9f refFpa=%.9f curvature=%.12g sag=%.9f sagLength=%.9f aoa=%.9f cmdAoa=%.9f cmdBank=%.9f\\n",step,altitude,reference.altitude_m,reference.flight_path_angle_deg,reference.vertical_curvature_per_m,route->profile_initial_sag_m,route->profile_initial_sag_length_m,state.attitude.aoa_rad*57.29577951308232,demand.control.angle_of_attack_rad*57.29577951308232,demand.control.bank_rad*57.29577951308232);\n'
 part=part.replace(marker,trace_code+marker)
 solver_modified=solver_source[:a]+part+solver_source[b:]
 solver_src=temp/'solver.c';solver_obj=temp/'solver.o';solver_src.write_text(solver_modified)
 report['traceSolverSourceSHA256']=hashlib.sha256(solver_modified.encode()).hexdigest()
 subprocess.run(['clang',*flags,'-c',str(solver_src),'-o',str(solver_obj)],cwd=ROOT/'CLanding',check=True)
 link=[str(solver_obj) if x=='build/terminal_solver.o' else x for x in link]
 link=[str(obj) if s=='build/taem_planner.o' else s for s in link];link[link.index('-o')+1]=str(binary)
 subprocess.run(link,cwd=ROOT/'CLanding',check=True)
 report['binarySHA256']=hashlib.sha256(binary.read_bytes()).hexdigest()
 for mode in ['baseline','bounded']:
  env=dict(os.environ,KSP_LANDER_MODEL_PROFILE='sts-live-20261010',KSP_LANDER_TAEM_PROFILE_DIAGNOSTICS='1');env.pop('KSL_EXPERIMENT_INLET_LOWER_BOUND',None)
  if mode=='bounded':env['KSL_EXPERIMENT_INLET_LOWER_BOUND']='1'
  args=[str(binary),'--configuration','Configuration/sts-live.json','--mass-kg','25123.15','--sample-index','0']
  log=HERE/f'{mode}.log';print('running',mode,flush=True)
  with log.open('w') as out:completed=subprocess.run(args,cwd=ROOT,env=env,stdout=out,stderr=subprocess.STDOUT)
  trials=[];bounds=[]
  for line in log.read_text().splitlines():
   if line.startswith('TAEM smooth trial:'):trials.append({k:v if k=='reason' else float(v) for k,v in re.findall(r'(\w+)=([^\s]+)',line)})
   if line.startswith('INLET_CURVATURE'):bounds.append({k:float(v) for k,v in re.findall(r'(\w+)=([^\s]+)',line)})
  valid=[r for r in trials if r['completed']]
  result={'mode':mode,'exitCode':completed.returncode,'logSHA256':hashlib.sha256(log.read_bytes()).hexdigest(),'trials':len(trials),'completed':len(valid),'jointlyFeasible':sum(r['feasible'] for r in trials),'smallestTrackingErrorCompleted':min(valid,key=lambda r:r['track']) if valid else None,'closestAltitudeCompleted':min(valid,key=lambda r:r['endpoint']) if valid else None,'lowerBoundCorrections':sum(r['target']>r['base'] for r in bounds),'upperBoundCorrections':sum(r['target']<r['base'] for r in bounds),'correctionExamples':bounds[:10], 'selectedProfileEarlyTrace':[ {k:float(v) for k,v in re.findall(r'(\w+)=([^\s]+)',line)} for line in log.read_text().splitlines() if line.startswith('BOUND_TRACE')], 'contract':next((s for s in log.read_text().splitlines() if s.startswith('MM305 downstream-feasibility contract:')),None)}
  if mode=='baseline':assert result['trials']==2184 and result['completed']==1287 and result['smallestTrackingErrorCompleted']['track']==3040.205
  report['runs'].append(result);(HERE/'result.json').write_text(json.dumps(report,indent=2)+'\n');print(json.dumps(result),flush=True)
a,b=[r['selectedProfileEarlyTrace'] for r in report['runs']]
assert len(a)==len(b)==42
report['earlyTraceComparison']={k:max(abs(x[k]-y[k]) for x,y in zip(a,b)) for k in ['altitude','refAltitude','refFpa','aoa','cmdAoa','cmdBank']}
(HERE/'result.json').write_text(json.dumps(report,indent=2)+'\n')
