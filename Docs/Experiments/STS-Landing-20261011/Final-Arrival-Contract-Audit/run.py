#!/usr/bin/env python3
"""Audit native Final gate on synthetic aligned arrival snapshots, not trajectories."""
import collections,csv,hashlib,json,os,re,shlex,subprocess,tempfile
from pathlib import Path
HERE=Path(__file__).resolve().parent;ROOT=HERE.parents[3]
original=(ROOT/'Validation/MM305FeasibilityPropertyTests.c').read_text()
fixture='#include "guidance_internal.h"\n'+original[:original.index('    GuidanceMachine g;')]+r'''
    GuidanceMachine seed;guidance_machine_init(&seed);
    Telemetry inlet=create_telemetry(&p,&cfg,26000,2.5,-30000,0,mass_kg);
    TerminalDynamicState initial=telemetry_to_terminal_state(&model,&inlet,cfg.site.runway_heading);
    double distance=mm305_final_alignment_distance(&seed,&model,&initial);
    TerminalModel resolved;mm305_prepare_planning_model(&resolved,&model,distance,0,1,1,0);
    printf("RESOLVED distance=%.6f slope=%.6f target=%.6f speedFloor=%.6f\n",distance,resolved.guidance.final_glide_slope,resolved.guidance.final_alignment_speed,.88*resolved.guidance.final_alignment_speed);
    const double speeds[]={110,125,127.814225,130,140.8,145.2,160,165,180};
    const double fpas[]={-20,-25.7};
    const double aoas[]={4,8,12,13,14,15};
    for(int station=0;station<2;station++)for(size_t vi=0;vi<sizeof(speeds)/sizeof(*speeds);vi++)for(int fi=0;fi<2;fi++)for(int ai=0;ai<6;ai++){
        double d=station?9000:8000;TerminalModel arrival=model;
        mm305_apply_final_alignment(&arrival,&model,d);
        double height=d*tan(arrival.guidance.final_glide_slope*DEG2RAD);
        Telemetry t=create_telemetry(&p,&cfg,cfg.site.altitude+height,1,-d,0,mass_kg);
        t.true_air_speed=speeds[vi];t.flight_path_angle=fpas[fi];t.angle_of_attack=aoas[ai];
        t.horizontal_speed=t.true_air_speed*cos(t.flight_path_angle*DEG2RAD);
        t.vertical_speed=t.true_air_speed*sin(t.flight_path_angle*DEG2RAD);
        TerminalDynamicState state=telemetry_to_terminal_state(&arrival,&t,cfg.site.runway_heading);
        TaemTerminalEvaluation evaluation;
        bool admit=terminal_native_delivery_admissible(&arrival,&state,&seed,&evaluation);
        bool speedband=fabs(speeds[vi]-arrival.guidance.final_alignment_speed)<=.12*arrival.guidance.final_alignment_speed;
        printf("ARRIVAL station=%.6f speed=%.6f fpa=%.6f aoa=%.6f target=%.6f speedBand=%d admit=%d evaluationValid=%d evaluationFeasible=%d block=%d\n",d,speeds[vi],fpas[fi],aoas[ai],arrival.guidance.final_alignment_speed,speedband,admit,evaluation.valid,evaluation.feasible,evaluation.block_reason);
    }
    return 0;
}
'''
source=(ROOT/'CLanding/guidance/guidance_final.c').read_text()
needle='    if(evaluation)*evaluation=verdict;'
assert source.count(needle)==1
instrumented=source.replace(needle,r'''    RunwayCaptureEnvelope capture_eval=decision_runway_capture_envelope(&guidance,&t,geometry.course_deg,&planet,&cfg);
    fprintf(stderr,"FINAL_GATE station=%.6f speed=%.6f fpa=%.6f aoa=%.6f runwayMargin=%.6f captureSpeedMargin=%.6f loadMargin=%.6f approach=%d planFeasible=%d degraded=%d minimum=%.6f reference=%.6f trigger=%.6f planKineticMargin=%.6f contractValid=%d energyMargin=%.6f altitudeMargin=%.6f fpaMargin=%.6f verdict=%d block=%d reason=%s\n",m->guidance.final_approach_distance,geometry.airspeed_mps,geometry.flight_path_angle_deg,state->attitude.aoa_rad*RAD2DEG,capture_eval.runway_remaining.margin,capture_eval.speed.margin,capture_eval.load.margin,approach,plan.feasible,plan.degraded_floor,plan.minimum_speed,plan.reference_speed,plan.trigger_altitude,plan.predicted_kinetic_margin,contract.valid,contract.specific_energy_margin,contract.altitude_margin,contract.flight_path_angle_margin,verdict.feasible,verdict.block_reason,plan.reject_reason?plan.reject_reason:"none");
'''+needle)
planning=(ROOT/'CLanding/guidance/final/planning.inc').read_text()
needle='    contract.specific_energy_margin=fmin(fmin(low_energy_margin,high_energy_margin),'
assert planning.count(needle)==1
energy_print=r'''    fprintf(stderr,"ENERGY station=%.6f speed=%.6f fpa=%.6f aoa=%.6f low=%.6f high=%.6f touchdown=%.6f predictedGate=%.6f gateMaximum=%.6f predictedTouchdown=%.6f\n",distance,t->true_air_speed,t->flight_path_angle,t->angle_of_attack,low_energy_margin,high_energy_margin,touchdown_energy_margin,predicted_gate_speed,gate_speed,predicted_touchdown_speed);
'''
instrumented_planning=planning.replace(needle,energy_print+needle)
report={'sourceCommit':subprocess.check_output(['git','rev-parse','HEAD'],cwd=ROOT,text=True).strip(),'policy':'Synthetic aligned snapshots at8/9km geometric20deg altitude, speeds110..180,FPA-20/-25.7,AoA4/8/12/13/14/15. Native admission wrapper uses one native-force observation and cold guidance history. No propagation, trajectory, tail, plant certification, route adoption or live flight. Instrumentation only; planner speedband unchanged and emergency low-energy ownership not exercised.', 'hashes':{'fixture':hashlib.sha256(fixture.encode()).hexdigest(),'instrumentedFinal':hashlib.sha256(instrumented.encode()).hexdigest(),'instrumentedPlanning':hashlib.sha256(instrumented_planning.encode()).hexdigest(),**{p:hashlib.sha256((ROOT/p).read_bytes()).hexdigest() for p in ['CLanding/terminal_solver.c','CLanding/guidance/guidance_taem.c','CLanding/guidance/guidance_final.c','CLanding/guidance/final/planning.inc','Configuration/sts-live.json','ShuttleSim/reference-model/sts_aero_live20261010.csv','ShuttleSim/reference-model/stsn_attitude_identified.ini','ShuttleSim/reference-model/kerbin_atmosphere_identified.csv']}},'arrivals':[],'gates':[],'energy':[]}
subprocess.run(['make','-C','CLanding'],cwd=ROOT,check=True,stdout=subprocess.DEVNULL)
dry=subprocess.check_output(['make','-n','-C','CLanding','mm305-feasibility-test'],cwd=ROOT,text=True).replace('\\\n',' ')
link=next(shlex.split(s) for s in dry.splitlines() if s.startswith('clang ') and 'MM305FeasibilityPropertyTests.c' in s)
with tempfile.TemporaryDirectory(prefix='ksl-final-contract-') as temp:
 temp=Path(temp);(temp/'final').mkdir();(temp/'final/planning.inc').write_text(instrumented_planning);p=temp/'final.c';p.write_text(instrumented);obj=temp/'final.o';f=temp/'fixture.c';f.write_text(fixture);binary=temp/'fixture'
 flags=link[1:link.index('../Validation/MM305FeasibilityPropertyTests.c')]
 subprocess.run(['clang',*flags,'-c',str(p),'-o',str(obj)],cwd=ROOT/'CLanding',check=True)
 link=[str(obj) if x=='build/guidance_final.o' else str(f) if x=='../Validation/MM305FeasibilityPropertyTests.c' else x for x in link];link[link.index('-o')+1]=str(binary)
 subprocess.run(link,cwd=ROOT/'CLanding',check=True);report['binarySHA256']=hashlib.sha256(binary.read_bytes()).hexdigest()
 log=HERE/'trace.log';env=dict(os.environ,KSP_LANDER_MODEL_PROFILE='sts-live-20261010')
 with log.open('w') as out:r=subprocess.run([str(binary),'--configuration','Configuration/sts-live.json','--mass-kg','25123.15','--sample-index','0'],cwd=ROOT,env=env,stdout=out,stderr=subprocess.STDOUT)
 assert r.returncode==0
 report['logSHA256']=hashlib.sha256(log.read_bytes()).hexdigest()
 for line in log.read_text().splitlines():
  if line.startswith('RESOLVED'):report['resolved']={k:float(v) for k,v in re.findall(r'(\w+)=([^\s]+)',line)}
  elif line.startswith('ENERGY'):report['energy'].append({k:float(v) for k,v in re.findall(r'(\w+)=([^\s]+)',line)})
  elif line.startswith('ARRIVAL'):report['arrivals'].append({k:float(v) for k,v in re.findall(r'(\w+)=([^\s]+)',line)})
  elif line.startswith('FINAL_GATE'):report['gates'].append({k:v if k=='reason' else float(v) for k,v in re.findall(r'(\w+)=([^\s]+)',line)})
 assert len(report['arrivals'])==len(report['gates'])==len(report['energy'])==216
 assert all(all(abs(a[k]-g[k])<1e-5 for k in ['station','speed','fpa','aoa']) for a,g in zip(report['arrivals'],report['gates']))
 report['summary']={'admitted':sum(a['admit'] for a in report['arrivals']),'admittedOutsideSpeedBand':sum(a['admit'] and not a['speedBand'] for a in report['arrivals']),'speedBandButRejected':sum(a['speedBand'] and not a['admit'] for a in report['arrivals']),'reasons':dict(collections.Counter(g['reason'] for g in report['gates']))}
 (HERE/'result.json').write_text(json.dumps(report,indent=2)+'\n')
 for name in ['arrivals','gates','energy']:
  with (HERE/f'{name}.csv').open('w') as out:w=csv.DictWriter(out,fieldnames=report[name][0].keys());w.writeheader();w.writerows(report[name])
 print(json.dumps({'resolved':report['resolved'],'summary':report['summary']}),flush=True)
