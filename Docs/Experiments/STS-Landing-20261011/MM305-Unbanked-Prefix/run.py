#!/usr/bin/env python3
"""Focused staged diagnostic; not a composite-route qualification or adoption."""
import hashlib,json,os,re,shlex,subprocess,tempfile
from pathlib import Path
HERE=Path(__file__).resolve().parent;ROOT=HERE.parents[3]
p=ROOT/'Validation/MM305FeasibilityPropertyTests.c';source=p.read_text()
a=source.index('        TaemInterfaceCapture capture =');modified=source[:a]+r'''
        TaemInterfaceCapture capture = entry_mm305_admission_envelope(&g, &t, cfg.site.runway_heading, &p, &cfg);
        assert(capture.valid && capture.ready);
        const double target=strtod(getenv("KSL_EXPERIMENT_PREFIX_M"),NULL);
        const double aoa=strtod(getenv("KSL_EXPERIMENT_PREFIX_AOA"),NULL);
        double station=0, work=0, peakq=0, peakg=0;
        TaemGeometryState geo;assert(taem_geometry_state(&model,&state,&geo));
        const double initial_ut=state.ut_s;
        const double initial_energy=.5*geo.airspeed_mps*geo.airspeed_mps-
            model.world.mu_m3_s2/v3_norm(state.position_i_m);
        while(1){
            AeroForces f=aero_compute(&model.world,&model.aero,state.position_i_m,
                state.velocity_i_mps,state.ut_s,state.mass_kg,state.attitude.aoa_rad,state.attitude.bank_rad);
            double load=fabs(f.lift_n)/state.mass_kg/9.80665;
            peakq=fmax(peakq,f.dynamic_pressure_pa);peakg=fmax(peakg,load);
            if(f.dynamic_pressure_pa>model.vehicle.maximum_dynamic_pressure ||
               load>model.vehicle.maximum_g_load || geo.altitude_above_runway_m<=0 ||
               state.ut_s-initial_ut>120){
                printf("PREFIX_REJECT station=%.6f elapsed=%.6f peakq=%.6f peakg=%.6f\n",station,state.ut_s-initial_ut,peakq,peakg);
                return 2;
            }
            if(station>=target)break;
            double dt=fmin(.5,(target-station)/fmax(geo.ground_speed_mps,1));
            if(dt<1e-6)break;
            TerminalControl u={.angle_of_attack_rad=aoa*M_PI/180,.bank_rad=0,.dt_s=dt};
            double prior_speed=geo.ground_speed_mps;
            assert(terminal_propagator_step(&model,&state,&u)==TERMINAL_STEP_OK);
            assert(taem_geometry_state(&model,&state,&geo));
            station+=.5*(prior_speed+geo.ground_speed_mps)*dt;
            work-=f.drag_n*f.airspeed_mps/state.mass_kg*dt;
        }
        AeroForces f=aero_compute(&model.world,&model.aero,state.position_i_m,
            state.velocity_i_mps,state.ut_s,state.mass_kg,state.attitude.aoa_rad,state.attitude.bank_rad);
        double energy=.5*geo.airspeed_mps*geo.airspeed_mps-
            model.world.mu_m3_s2/v3_norm(state.position_i_m);
        printf("PREFIX_EXIT station=%.6f elapsed=%.6f altitude=%.6f along=%.6f cross=%.6f speed=%.6f mach=%.6f fpa=%.6f course=%.6f aoa=%.6f bank=%.6f peakq=%.6f peakg=%.6f energyChange=%.6f dragWork=%.6f\n",
            station,state.ut_s-initial_ut,geo.altitude_above_runway_m,geo.runway_along_m,
            geo.runway_cross_m,geo.airspeed_mps,f.mach,geo.flight_path_angle_deg,
            geo.course_deg,state.attitude.aoa_rad*180/M_PI,state.attitude.bank_rad*180/M_PI,
            peakq,peakg,energy-initial_energy,work);
        Mm305PlanRequest req={.valid=true,.request_ut=state.ut_s,
            .model_snapshot_id=model.snapshot_id,.state=state,
            .hac_radius_m=model.guidance.hac_radius,.upstream_end=0,
            .lift_scale=1,.drag_scale=1};
        Mm305PlanResult result=mm305_plan(&model,&req);
        printf("PREFIX_DOWNSTREAM valid=%d found=%d solveWall=%.6f diagnostic=%s\n",
            result.valid,result.found,result.solve_wall_s,result.diagnostic);
        if(result.found)puts("FOUND_DOWNSTREAM_ONLY: combined prefix/route and original admission remain unqualified.");
        return result.found?0:1;
    }
    return 3;
}
'''
modified='#include "terminal_propagator.h"\n'+modified.replace('    int total_admitted = 0, qualified = 0, contract_violations = 0;\n','')
report={'sourceCommit':subprocess.check_output(['git','rev-parse','HEAD'],cwd=ROOT,text=True).strip(),'policy':'Temporary focused sample0 fixture. Physically admitted original state, unbanked fixed-incidence prefix with native servo/forces; downstream planner from complete exit state under unchanged production gates. Prefix pressure/normal-load/ground/timeout checked. No original-state handoff or combined-route adoption/qualification; finite staged feasibility diagnostic only. Air-relative mechanical energy includes rotating-atmosphere effects, not an inertial drag-work identity. Planner found means native route and Final admission, not landing acceptance.', 'sourceSHA256':hashlib.sha256(source.encode()).hexdigest(),'experimentSourceSHA256':hashlib.sha256(modified.encode()).hexdigest(),'inputHashes':{n:hashlib.sha256((ROOT/n).read_bytes()).hexdigest() for n in ['Configuration/sts-live.json','ShuttleSim/reference-model/sts_aero_live20261010.csv','ShuttleSim/reference-model/stsn_attitude_identified.ini','ShuttleSim/reference-model/kerbin_atmosphere_identified.csv']},'runs':[]}
subprocess.run(['make','-C','CLanding'],cwd=ROOT,check=True,stdout=subprocess.DEVNULL)
dry=subprocess.check_output(['make','-n','-C','CLanding','mm305-feasibility-test'],cwd=ROOT,text=True).replace('\\\n',' ')
link=next(shlex.split(s) for s in dry.splitlines() if s.startswith('clang ') and 'MM305FeasibilityPropertyTests.c' in s)
with tempfile.TemporaryDirectory(prefix='ksl-unbanked-prefix-') as temp:
 temp=Path(temp);src=temp/'fixture.c';binary=temp/'fixture';src.write_text(modified)
 link=[str(src) if x=='../Validation/MM305FeasibilityPropertyTests.c' else x for x in link];link[link.index('-o')+1]=str(binary)
 subprocess.run(link,cwd=ROOT/'CLanding',check=True);report['binarySHA256']=hashlib.sha256(binary.read_bytes()).hexdigest()
 for name,distance,aoa in [('baseline',0,18),('prefix4k-aoa3',4000,3),('prefix8k-aoa3',8000,3),('prefix8k-aoa10',8000,10),('prefix12k-aoa10',12000,10)]:
  env=dict(os.environ,KSP_LANDER_MODEL_PROFILE='sts-live-20261010',KSP_LANDER_TAEM_PROFILE_DIAGNOSTICS='1',KSP_LANDER_TAEM_DIAGNOSTICS='1',KSL_EXPERIMENT_PREFIX_M=str(distance),KSL_EXPERIMENT_PREFIX_AOA=str(aoa))
  log=HERE/f'{name}.log';print('running',name,flush=True)
  with log.open('w') as out:r=subprocess.run([str(binary),'--configuration','Configuration/sts-live.json','--mass-kg','25123.15','--sample-index','0'],cwd=ROOT,env=env,stdout=out,stderr=subprocess.STDOUT)
  lines=log.read_text().splitlines();trials=[]
  for line in lines:
   if line.startswith('TAEM smooth trial:'):trials.append({k:v if k=='reason' else float(v) for k,v in re.findall(r'(\w+)=([^\s]+)',line)})
  complete=[t for t in trials if t['completed']]
  result={'name':name,'prefixM':distance,'commandAoADeg':aoa,'exitCode':r.returncode,'logSHA256':hashlib.sha256(log.read_bytes()).hexdigest(),'trials':len(trials),'completed':len(complete),'referenceFeasible':sum(bool(t['feasible']) for t in trials),'smallestTrackingErrorCompleted':min(complete,key=lambda t:t['track']) if complete else None,'closestAltitudeCompleted':min(complete,key=lambda t:t['endpoint']) if complete else None,'prefixExit':next((s for s in lines if s.startswith('PREFIX_EXIT')),None),'downstream':next((s for s in lines if s.startswith('PREFIX_DOWNSTREAM')),None),'prefixReject':next((s for s in lines if s.startswith('PREFIX_REJECT')),None)}
  if name=='baseline':assert result['trials']==2184 and result['completed']==1287
  report['runs'].append(result);(HERE/'result.json').write_text(json.dumps(report,indent=2)+'\n');print(json.dumps(result),flush=True)
