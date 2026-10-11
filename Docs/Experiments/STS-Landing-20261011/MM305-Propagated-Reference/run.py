#!/usr/bin/env python3
"""Generate an independently integrated, propagated-authority reference."""
import hashlib,json,os,re,shlex,subprocess,tempfile
from pathlib import Path
HERE=Path(__file__).resolve().parent;ROOT=HERE.parents[3]
p=ROOT/'CLanding/terminal_solver.c';source=p.read_text()
a=source.index('static ProfileFlight fly_smooth_profile(');b=source.index('static bool smooth_flight_ok',a);part=source[a:b]
part=part.replace('    size_t cursor = 0, count = 0;', '''    bool dynamic_reference=getenv("KSL_EXPERIMENT_REFERENCE_RESPONSE")!=NULL;
    double reference_gamma=initial?geometry.flight_path_angle_deg/57.29577951308232:0.0;
    double reference_altitude=m->site.altitude+geometry.altitude_above_runway_m;
    double previous_station=0.0, previous_rate=0.0;
    size_t cursor = 0, count = 0;''')
needle='        double altitude = m->site.altitude + geometry.altitude_above_runway_m;'
assert part.count(needle)==1
code='''        if(dynamic_reference){
            double ds=fmax(0.0,reference.station_m-previous_station);
            if(step>0){
                double next_gamma=reference_gamma+previous_rate*dt;
                if(fabs(next_gamma)>80.0/57.29577951308232){
                    out.reason="integrated reference exceeded usable FPA";return out;
                }
                reference_altitude+=0.5*ds*(tan(reference_gamma)+tan(next_gamma));
                reference_gamma=next_gamma;
            }
            previous_station=reference.station_m;
            AeroForces flow=aero_compute(&m->world,&m->aero,state.position_i_m,
                state.velocity_i_mps,state.ut_s,state.mass_kg,0.0,0.0);
            double ceiling=fmin(m->vehicle.maximum_angle_of_attack,
                m->aero.alpha_deg[m->aero.alpha_count-1]);
            if(flow.mach<1.0 && m->vehicle.terminal_maximum_lift_angle_of_attack>0.0)
                ceiling=fmin(ceiling,m->vehicle.terminal_maximum_lift_angle_of_attack);
            if(state.trim_aoa_ceiling_rad>0.0)
                ceiling=fmin(ceiling,state.trim_aoa_ceiling_rad*57.29577951308232);
            double radius=v3_norm(state.position_i_m);
            double cap=m->vehicle.maximum_g_load*m->world.mu_m3_s2/(radius*radius);
            double maximum=0.0;
            for(size_t ai=0;ai<=m->aero.alpha_count;++ai){
                double alpha=ai<m->aero.alpha_count?m->aero.alpha_deg[ai]:ceiling;
                if(alpha<0.0||alpha>ceiling)continue;
                AeroForces f=aero_compute(&m->world,&m->aero,state.position_i_m,
                    state.velocity_i_mps,state.ut_s,state.mass_kg,
                    alpha/57.29577951308232,0.0);
                maximum=fmax(maximum,fmin(cap,fabs(f.lift_n)/state.mass_kg));
            }
            maximum*=0.95;
            double lateral=fabs(taem_tracker_lateral_demand(m,&geometry,&reference));
            double bank=fmin(m->vehicle.maximum_bank_angle,80.0)/57.29577951308232;
            double vertical_min,vertical_max;
            if(lateral<=maximum*sin(bank)){
                vertical_min=lateral/tan(bank);
                vertical_max=sqrt(fmax(0.0,maximum*maximum-lateral*lateral));
            }else vertical_min=vertical_max=maximum*cos(bank);
            double free_required=taem_tracker_vertical_lift_demand(m,&state,&geometry,0.0);
            double min_rate=(vertical_min-free_required)/geometry.airspeed_mps;
            double max_rate=(vertical_max-free_required)/geometry.airspeed_mps;
            double correction=atan((reference.altitude_m-reference_altitude)/
                fmax(15.0*geometry.ground_speed_mps,1.0));
            correction=fmax(-6.0/57.29577951308232,fmin(6.0/57.29577951308232,correction));
            double target=reference.flight_path_angle_deg/57.29577951308232+correction;
            double response=strtod(getenv("KSL_EXPERIMENT_REFERENCE_RESPONSE"),NULL);
            double requested=geometry.ground_speed_mps*reference.vertical_curvature_per_m+
                (target-reference_gamma)/response;
            requested=fmax(-8.0/57.29577951308232,fmin(8.0/57.29577951308232,requested));
            previous_rate=fmax(min_rate,fmin(max_rate,requested));
            reference.altitude_m=reference_altitude;
            reference.flight_path_angle_deg=reference_gamma*57.29577951308232;
            reference.vertical_curvature_per_m=previous_rate/geometry.ground_speed_mps;
        }
'''
part=part.replace(needle,code+needle);modified=source[:a]+part+source[b:]
report={'sourceCommit':subprocess.check_output(['git','rev-parse','HEAD'],cwd=ROOT,text=True).strip(),'policy':'Temporary solver. Reference gamma/altitude independently integrated over projected station, not reset to actual flight. Gamma-rate clipped using Mach/q/current polar, g cap, allowed incidence and lateral demand within bank limit. Original24deg reference/tracking/endpoint/160mps speed/Final gates unchanged. Focused sample0 only; actuator feasibility still requires replay.','sourceSHA256':hashlib.sha256(source.encode()).hexdigest(),'experimentSourceSHA256':hashlib.sha256(modified.encode()).hexdigest(),'inputHashes':{n:hashlib.sha256((ROOT/n).read_bytes()).hexdigest() for n in ['Configuration/sts-live.json','ShuttleSim/reference-model/sts_aero_live20261010.csv','ShuttleSim/reference-model/stsn_attitude_identified.ini','ShuttleSim/reference-model/kerbin_atmosphere_identified.csv','Validation/MM305FeasibilityPropertyTests.c']},'runs':[]}
subprocess.run(['make','-C','CLanding'],cwd=ROOT,check=True,stdout=subprocess.DEVNULL)
dry=subprocess.check_output(['make','-n','-C','CLanding','mm305-feasibility-test'],cwd=ROOT,text=True).replace('\\\n',' ')
link=next(shlex.split(s) for s in dry.splitlines() if s.startswith('clang ') and 'MM305FeasibilityPropertyTests.c' in s)
with tempfile.TemporaryDirectory(prefix='ksl-propagated-reference-') as temp:
 temp=Path(temp);src=temp/'solver.c';obj=temp/'solver.o';binary=temp/'fixture';src.write_text(modified)
 flags=link[1:link.index('../Validation/MM305FeasibilityPropertyTests.c')]
 subprocess.run(['clang',*flags,'-c',str(src),'-o',str(obj)],cwd=ROOT/'CLanding',check=True)
 link=[str(obj) if x=='build/terminal_solver.o' else x for x in link];link[link.index('-o')+1]=str(binary)
 subprocess.run(link,cwd=ROOT/'CLanding',check=True);report['binarySHA256']=hashlib.sha256(binary.read_bytes()).hexdigest()
 for name,response in [('baseline',None),('response2',2.05),('response8',8.0)]:
  env=dict(os.environ,KSP_LANDER_MODEL_PROFILE='sts-live-20261010',KSP_LANDER_TAEM_PROFILE_DIAGNOSTICS='1');env.pop('KSL_EXPERIMENT_REFERENCE_RESPONSE',None)
  if response is not None:env['KSL_EXPERIMENT_REFERENCE_RESPONSE']=str(response)
  log=HERE/f'{name}.log';args=[str(binary),'--configuration','Configuration/sts-live.json','--mass-kg','25123.15','--sample-index','0'];print('running',name,flush=True)
  with log.open('w') as out:r=subprocess.run(args,cwd=ROOT,env=env,stdout=out,stderr=subprocess.STDOUT)
  trials=[]
  for line in log.read_text().splitlines():
   if line.startswith('TAEM smooth trial:'):trials.append({k:v if k=='reason' else float(v) for k,v in re.findall(r'(\w+)=([^\s]+)',line)})
  complete=[t for t in trials if t['completed']];feasible=[t for t in trials if t['feasible']]
  slope_ok=[t for t in complete if t['steep']<=t['slopeLimit']]
  result={'slopeCompliantCompleted':len(slope_ok),'smallestTrackingErrorSlopeCompliant':min(slope_ok,key=lambda t:t['track']) if slope_ok else None,'name':name,'referenceResponseSeconds':response,'exitCode':r.returncode,'logSHA256':hashlib.sha256(log.read_bytes()).hexdigest(),'trials':len(trials),'completed':len(complete),'referenceFeasible':len(feasible),'smallestTrackingErrorCompleted':min(complete,key=lambda t:t['track']) if complete else None,'closestAltitudeCompleted':min(complete,key=lambda t:t['endpoint']) if complete else None,'feasibleExamples':feasible[:10],'contract':next((s for s in log.read_text().splitlines() if s.startswith('MM305 downstream-feasibility contract:')),None)}
  if name=='baseline':assert result['trials']==2184 and result['completed']==1287
  report['runs'].append(result);(HERE/'result.json').write_text(json.dumps(report,indent=2)+'\n');print(json.dumps(result),flush=True)
