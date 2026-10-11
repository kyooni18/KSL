#!/usr/bin/env python3
"""Trace a rejected profile in a separate offline fixture binary."""
import hashlib,json,os,re,shlex,subprocess,tempfile
from pathlib import Path
HERE=Path(__file__).resolve().parent;ROOT=HERE.parents[3]
solver=ROOT/'CLanding/terminal_solver.c';source=solver.read_text()
a=source.index('static ProfileFlight fly_smooth_profile(');b=source.index('static bool smooth_flight_ok',a)
part=source[a:b]
old='    size_t cursor = 0, count = 0;'
assert part.count(old)==1
part=part.replace(old,'''    bool trace_target = fabs(route->length_m-62326.028)<0.002 &&
        fabs(route->profile_midpoint_offset_m+1150.990)<0.002;
    static unsigned next_trace_id=0;
    unsigned trace_id=trace_target?++next_trace_id:0;
    size_t cursor = 0, count = 0;''')
old='        if (terminal_propagator_step(m, &state, &demand.control) != TERMINAL_STEP_OK) {'
assert part.count(old)==1
part=part.replace(old,'''        if(trace_target) {
            AeroForces f=aero_compute(&m->world,&m->aero,state.position_i_m,
                state.velocity_i_mps,state.ut_s,state.mass_kg,
                state.attitude.aoa_rad,state.attitude.bank_rad);
            double ceiling=fmin(m->vehicle.maximum_angle_of_attack,
                m->aero.alpha_deg[m->aero.alpha_count-1]);
            if(f.mach<1.0 && m->vehicle.terminal_maximum_lift_angle_of_attack>0.0)
                ceiling=fmin(ceiling,m->vehicle.terminal_maximum_lift_angle_of_attack);
            if(state.trim_aoa_ceiling_rad>0.0)
                ceiling=fmin(ceiling,state.trim_aoa_ceiling_rad*57.29577951308232);
            double maximum_lift=0.0;
            for(size_t ai=0;ai<=m->aero.alpha_count;++ai){
                double alpha=ai<m->aero.alpha_count?m->aero.alpha_deg[ai]:ceiling;
                if(alpha<0.0 || alpha>ceiling)continue;
                AeroForces candidate=aero_compute(&m->world,&m->aero,
                    state.position_i_m,state.velocity_i_mps,state.ut_s,
                    state.mass_kg,alpha/57.29577951308232,0.0);
                maximum_lift=fmax(maximum_lift,fabs(candidate.lift_n)/state.mass_kg);
            }
            fprintf(stderr,"PROFILE_AUTHORITY id=%u step=%zu elapsed=%.6f dt=%.6f station=%.6f altitude=%.6f refAltitude=%.6f speed=%.6f groundSpeed=%.6f mach=%.6f q=%.6f fpa=%.6f refFpa=%.6f aoa=%.6f bank=%.6f cmdAoa=%.6f cmdBank=%.6f requestedAoa=%.6f liftAccel=%.6f dragAccel=%.6f maximumLift=%.6f requiredVertical=%.6f selectedVertical=%.6f requiredLateral=%.6f availableLateral=%.6f crossError=%.6f courseError=%.6f verticalOK=%d lateralOK=%d\\n",
                trace_id,step,step*dt,dt,reference.station_m,altitude,
                reference.altitude_m,geometry.airspeed_mps,geometry.ground_speed_mps,
                f.mach,f.dynamic_pressure_pa,geometry.flight_path_angle_deg,
                reference.flight_path_angle_deg,state.attitude.aoa_rad*57.29577951308232,
                state.attitude.bank_rad*57.29577951308232,
                demand.control.angle_of_attack_rad*57.29577951308232,
                demand.control.bank_rad*57.29577951308232,
                state.attitude.requested_aoa_rad*57.29577951308232,
                f.lift_n/state.mass_kg,f.drag_n/state.mass_kg,maximum_lift,
                demand.required_vertical_lift_mps2,demand.delivered_vertical_lift_mps2,
                demand.required_lateral_accel_mps2,demand.available_lateral_accel_mps2,
                demand.cross_track_error_m,demand.course_error_deg,
                demand.vertical_authority_ok,demand.lateral_authority_ok);
        }
''' + old)
modified=source[:a]+part+source[b:]
report={'sourceCommit':subprocess.check_output(['git','rev-parse','HEAD'],cwd=ROOT,text=True).strip(),'sourceSHA256':hashlib.sha256(source.encode()).hexdigest(),'instrumentedSourceSHA256':hashlib.sha256(modified.encode()).hexdigest(),'policy':'Temporary instrumentation only. No command or propagation changes, no gate changes. Focused sample0. Maximum lift is the permitted-incidence table envelope, ignoring load cap and attitude limits. Actual incidence can exceed the command cap transiently.'}
report['inputHashes']={n:hashlib.sha256((ROOT/n).read_bytes()).hexdigest() for n in ['Configuration/sts-live.json','ShuttleSim/reference-model/sts_aero_live20261010.csv','ShuttleSim/reference-model/stsn_attitude_identified.ini','ShuttleSim/reference-model/kerbin_atmosphere_identified.csv','Validation/MM305FeasibilityPropertyTests.c']}
subprocess.run(['make','-C','CLanding'],cwd=ROOT,check=True,stdout=subprocess.DEVNULL)
dry=subprocess.check_output(['make','-n','-C','CLanding','mm305-feasibility-test'],cwd=ROOT,text=True).replace('\\\n',' ')
link=next(shlex.split(s) for s in dry.splitlines() if s.startswith('clang ') and 'MM305FeasibilityPropertyTests.c' in s)
with tempfile.TemporaryDirectory(prefix='ksl-authority-trace-') as temp:
 temp=Path(temp);src=temp/'solver.c';obj=temp/'solver.o';binary=temp/'fixture';src.write_text(modified)
 flags=link[1:link.index('../Validation/MM305FeasibilityPropertyTests.c')]
 subprocess.run(['clang',*flags,'-c',str(src),'-o',str(obj)],cwd=ROOT/'CLanding',check=True)
 link=[str(obj) if s=='build/terminal_solver.o' else s for s in link];link[link.index('-o')+1]=str(binary)
 subprocess.run(link,cwd=ROOT/'CLanding',check=True)
 report['binarySHA256']=hashlib.sha256(binary.read_bytes()).hexdigest()
 args=[str(binary),'--configuration','Configuration/sts-live.json','--mass-kg','25123.15','--sample-index','0']
 report['fixtureArguments']=args[1:];env=dict(os.environ,KSP_LANDER_MODEL_PROFILE='sts-live-20261010',KSP_LANDER_TAEM_PROFILE_DIAGNOSTICS='1')
 report['environment']={k:env[k] for k in ['KSP_LANDER_MODEL_PROFILE','KSP_LANDER_TAEM_PROFILE_DIAGNOSTICS']}
 log=HERE/'trace.log'
 with log.open('w') as out:completed=subprocess.run(args,cwd=ROOT,env=env,stdout=out,stderr=subprocess.STDOUT)
 report['exitCode']=completed.returncode;report['logSHA256']=hashlib.sha256(log.read_bytes()).hexdigest()
 assert completed.returncode==1
 (HERE/'provenance.json').write_text(json.dumps(report,indent=2)+'\n')
