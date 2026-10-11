#!/usr/bin/env python3
"""Passive frozen point-mass response experiment at late real replay states."""
import hashlib,json,os,re,shlex,subprocess,tempfile
from pathlib import Path
HERE=Path(__file__).resolve().parent;ROOT=HERE.parents[3]
sequence=(ROOT/'CLanding/guidance/final/sequence.inc').read_text()
a=sequence.index('static double terminal_predict_touchdown_speed(');b=sequence.index('#include "adaptive_flare.inc"',a)
function=sequence[a:b]
clone=function.replace('terminal_predict_touchdown_speed(', 'experiment_frozen_forecast(',1).replace('const TerminalFlarePlan*p){','const TerminalFlarePlan*p,int mode,double experiment_dt){',1)
clone=clone.replace('    const double DT=.25,TAU=1.5;','''    const double DT=experiment_dt,TAU=1.5;
    AttitudeModel servo;attitude_seed(&servo);
    const char*path=getenv("KSP_LANDER_TERMINAL_ATTITUDE");
    if(!path||!attitude_load_ini(&servo,path))return NAN;
    servo.direct=false;
    servo.aoa_rad=t->angle_of_attack*DEG2RAD;
    servo.aoa_rate_rad_s=t->has_angle_of_attack_rate?t->angle_of_attack_rate*DEG2RAD:0.0;
    servo.cmd_aoa_rad=servo.requested_aoa_rad=servo.aoa_rad;
    servo.bank_rad=servo.cmd_bank_rad=servo.requested_bank_rad=0;
    double elapsed=0,drag_work=0,max_aoa=0,initial_h=height,initial_v=V;
    double latch_time=NAN;
    int steps=(int)ceil(300.0/DT);
''')
clone=clone.replace('for(int i=0;i<1200&&height>0.0&&V>5.0;++i)', 'for(int i=0;i<steps&&height>0.0&&V>5.0;++i)')
clone=clone.replace('if(height<=target_latch)latched=true;','if(height<=target_latch){latched=true;latch_time=elapsed;}')
needle='''        double L=0.0,D=0.0;terminal_aero_q(g,&tc,v,aoa,&L,&D);
        V+=(-D*tc.dynamic_pressure/m-p->g0*sin(gamma))*DT;
        if(latched)sink-=accel_up*DT;
        height-=sink*DT;'''
assert clone.count(needle)==1
clone=clone.replace(needle,'''        double flown_aoa=aoa;
        if(mode==2){
            attitude_set_command(&servo,aoa*DEG2RAD,0);
            /* Native propagation also evaluates forces before advancing attitude. */
            flown_aoa=servo.aoa_rad*RAD2DEG;
        }
        double L=0.0,D=0.0;terminal_aero_q(g,&tc,v,flown_aoa,&L,&D);
        double achieved=L*tc.dynamic_pressure*p->lift_ratio/m-p->g0*cos(gamma);
        if(mode>0 && latched)accel_up=achieved;
        drag_work+=D*tc.dynamic_pressure*V/m*DT;
        V+=(-D*tc.dynamic_pressure/m-p->g0*sin(gamma))*DT;
        if(latched)sink-=accel_up*DT;
        height-=sink*DT;
        if(mode==2)attitude_step(&servo,tc.dynamic_pressure,DT);
        elapsed+=DT;max_aoa=fmax(max_aoa,flown_aoa);
        if(((int)round(elapsed/DT))%(int)fmax(1,round(1.0/DT))==0 || height<=0)
            fprintf(stderr,"FROZEN_STEP ut=%.6f mode=%d dt=%.6f elapsed=%.6f height=%.6f speed=%.6f sink=%.6f commandAoa=%.6f actualAoa=%.6f achieved=%.6f dragWork=%.6f\\n",t->ut,mode,DT,elapsed,height,V,sink,aoa,flown_aoa,achieved,drag_work);
''')
clone=clone.replace('    return height<=0.0&&isfinite(V)&&V>0.0?V:NAN;','''    double result=height<=0.0&&isfinite(V)&&V>0.0?V:NAN;
    fprintf(stderr,"FROZEN_RESULT ut=%.6f mode=%d dt=%.6f initialHeight=%.6f initialSpeed=%.6f initialSink=%.6f initialAoa=%.6f initialAoaRate=%.6f rateAvailable=%d trigger=%.6f arrest=%.6f liftRatio=%.6f speed=%.6f sink=%.6f height=%.6f elapsed=%.6f dragWork=%.6f maxAoa=%.6f latchTime=%.6f\\n",t->ut,mode,DT,initial_h,initial_v,fmax(0,-t->vertical_speed),t->angle_of_attack,t->angle_of_attack_rate,t->has_angle_of_attack_rate,p->preflare_trigger_altitude,p->arrest_accel,p->lift_ratio,result,sink,height,elapsed,drag_work,max_aoa,latch_time);
    return result;''')
call='''        const char*target=getenv("KSL_FROZEN_UT");
        if(target&&fabs(t->ut-strtod(target,NULL))<.015){
            double repeated=experiment_frozen_forecast(g,t,v,height,V,sink_now,false,&flare_plan,0,.25);
            if(!(fabs(repeated-predicted_touchdown_speed)<1e-8))abort();
            (void)experiment_frozen_forecast(g,t,v,height,V,sink_now,false,&flare_plan,1,.25);
            (void)experiment_frozen_forecast(g,t,v,height,V,sink_now,false,&flare_plan,2,.25);
            (void)experiment_frozen_forecast(g,t,v,height,V,sink_now,false,&flare_plan,1,.05);
            (void)experiment_frozen_forecast(g,t,v,height,V,sink_now,false,&flare_plan,2,.05);
            if(g->preflare_trigger_altitude>0){
                /* Counterfactual retained trigger; never sent to actual controls. */
                TerminalFlarePlan retained=flare_plan;
                retained.preflare_trigger_altitude=g->preflare_trigger_altitude;
                (void)experiment_frozen_forecast(g,t,v,height,V,sink_now,false,&retained,0,.25);
                (void)experiment_frozen_forecast(g,t,v,height,V,sink_now,false,&retained,2,.05);
            }
        }
'''
modified=sequence[:b]+clone+sequence[b:]
needle='        predicted_touchdown_speed=terminal_predict_touchdown_speed(g,t,v,height,V,sink_now,false,&flare_plan);'
assert modified.count(needle)==1;modified=modified.replace(needle,needle+'\n'+call)
final='#include "shuttlesim/attitude.h"\n'+(ROOT/'CLanding/guidance/guidance_final.c').read_text()
report={'sourceCommit':subprocess.check_output(['git','rev-parse','HEAD'],cwd=ROOT,text=True).strip(),'policy':'Passive cloned forecast at real late pre-latch states, retaining learned force/history/frozen plan. Mode0 reproduces original pointmass exactly;mode1 replaces ideal sink acceleration by achieved learned normal lift;mode2 additionally native ideal AoA servo seeded measuredAoA/rate; initial servo command equals measuredAoA, zero bank/directfalse. Scalar sink equations and pre-latch constant-FPA assumption retained, not native3D/FCS/bodyplant or landingqualification. dt.25/.05 comparison. L5 also tests counterfactual retained trigger despite current plan invalid; never adopted. Main controller result untouched.', 'hashes':{'sequenceOriginal':hashlib.sha256(sequence.encode()).hexdigest(),'sequenceExperimental':hashlib.sha256(modified.encode()).hexdigest(),'finalExperimental':hashlib.sha256(final.encode()).hexdigest()},'runs':[]}
subprocess.run(['make','-C','CLanding'],cwd=ROOT,check=True,stdout=subprocess.DEVNULL)
dry=subprocess.check_output(['make','-n','-C','CLanding','mm305-feasibility-test'],cwd=ROOT,text=True).replace('\\\n',' ')
fixture_link=next(shlex.split(s) for s in dry.splitlines() if s.startswith('clang ') and 'MM305FeasibilityPropertyTests.c' in s)
flags=fixture_link[1:fixture_link.index('../Validation/MM305FeasibilityPropertyTests.c')]+['-Iguidance/final']
dry=subprocess.check_output(['make','-n','-B','-C','CLanding','all'],cwd=ROOT,text=True).replace('\\\n',' ')
link=next(shlex.split(s) for s in dry.splitlines() if s.startswith('clang ') and s.endswith('-o build/landing_backend'))
with tempfile.TemporaryDirectory(prefix='ksl-frozen-response-') as temp:
 temp=Path(temp);(temp/'final').mkdir();(temp/'final/sequence.inc').write_text(modified);src=temp/'final.c';src.write_text(final);obj=temp/'final.o';binary=temp/'landing_backend'
 subprocess.run(['clang',*flags,'-c',str(src),'-o',str(obj)],cwd=ROOT/'CLanding',check=True)
 link=[str(obj) if x=='build/guidance_final.o' else x for x in link];link[link.index('-o')+1]=str(binary)
 subprocess.run(link,cwd=ROOT/'CLanding',check=True);report['binarySHA256']=hashlib.sha256(binary.read_bytes()).hexdigest()
 for name,ut,scenario,configuration,prior_path in [('l5',2462.65,'Docs/Experiments/STS-Landing-20261010/L5-72km/mm305-continuous.ini','Docs/Experiments/STS-Landing-20261010/L5-72km/configuration.json','runs/20261011T112746Z-final-forecast-audit/simulation-result.json'),('l7',7021.45,'Docs/Experiments/STS-Landing-20261011/L7-85km-10deg/mm305-taem.ini','Configuration/sts-live.json','runs/20261011T112809Z-final-forecast-audit/simulation-result.json')]:
  env=dict(os.environ,KSL_DATA_ROOT=str(ROOT),KSP_LANDER_MODEL_PROFILE='sts-live-20261010',KSP_LANDER_FINAL_TRACE='1',KSL_FROZEN_UT=str(ut))
  args=[str(ROOT/'.venv/bin/python'),'ShuttleSim/scripts/run_guidance.py','--scenario',scenario,'--configuration',configuration,'--attitude','Docs/Experiments/STS-Landing-20261010/Codex-Flare-Offline/inputs/att-live.ini','--engage','engageReentry','--direct-control','--skip-build','--backend-build-dir',str(temp),'--no-mirror','--quiet-progress','--label',f'frozen-response-{name}','--max-sim-time','2400','--web-telemetry-port','0']
  print('running',name,flush=True);out=subprocess.run(args,cwd=ROOT,env=env,text=True,capture_output=True)
  (HERE/f'{name}-runner.log').write_text(out.stdout+out.stderr)
  result=json.loads(out.stdout.strip().splitlines()[-1]);prior=json.loads((ROOT/prior_path).read_text())
  assert result['simulatorSummary']==prior['simulatorSummary'],'passive instrumentation changed flight'
  log=Path(result['backendStderr']);rows=[]
  for line in log.read_text().splitlines():
   if line.startswith('FROZEN_RESULT'):rows.append({k:float(v) for k,v in re.findall(r'(\w+)=([^\s]+)',line)})
  assert len(rows)==(7 if name=='l5' else 5)
  record={'name':name,'targetUT':ut,'runResult':result,'priorResult':prior_path,'identicalFlightSummary':True,'forecasts':rows,'manifest':json.loads(Path(result['manifest']).read_text()),'logSHA256':hashlib.sha256(log.read_bytes()).hexdigest()}
  report['runs'].append(record);(HERE/'result.json').write_text(json.dumps(report,indent=2)+'\n');print(json.dumps({'name':name,'forecasts':rows}),flush=True)
