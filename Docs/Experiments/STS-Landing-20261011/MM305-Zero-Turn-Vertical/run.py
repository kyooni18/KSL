#!/usr/bin/env python3
"""Finite zero-lateral-demand control diagnostic, not runway-route qualification."""
import csv,hashlib,json,os,re,shlex,subprocess,tempfile
from pathlib import Path
HERE=Path(__file__).resolve().parent;ROOT=HERE.parents[3]
original=(ROOT/'Validation/MM305FeasibilityPropertyTests.c').read_text()
fixture=original[:original.index('        TaemInterfaceCapture capture =')].replace('    int total_admitted = 0, qualified = 0, contract_violations = 0;\n','')
fixture='#include "taem_planner.h"\n#include "terminal_propagator.h"\nextern bool experiment_reference(const TerminalModel*,const TerminalDynamicState*,const TaemGeometryState*,double,double,TaemRoute*);\nextern TaemPathReference experiment_eval(const TaemRoute*,double);\n'+fixture+r'''
        TaemInterfaceCapture capture=entry_mm305_admission_envelope(&g,&t,cfg.site.runway_heading,&p,&cfg);
        assert(capture.valid&&capture.ready);
        TerminalDynamicState initial=state;TaemGeometryState start;
        assert(taem_geometry_state(&model,&state,&start));
        double drop=model.site.altitude+start.altitude_above_runway_m-
            (model.site.altitude+model.guidance.final_approach_distance*tan(model.guidance.final_glide_slope*M_PI/180));
        for(int trial=0;trial<448;trial++){
            int li=trial/13,bi=trial%13;
            double length=40000+10000*li,bow=-.3*drop+.6*drop*bi/12;
            if(trial>=91){int n=trial-91;length=60000+1000*(n/17);bow=-2000+250*(n%17);}
            TaemRoute route;
            if(!experiment_reference(&model,&initial,&start,length,bow,&route)){puts("INIT_FAIL");return 2;}
            state=initial;TaemGeometryState geo=start;
            double station=0,elapsed=0,worst=0,steep=0,qmax=0,gmax=0;
            for(int j=0;j<=4096;j++){
                TaemPathReference ref=experiment_eval(&route,length*j/4096.0);
                steep=fmax(steep,-ref.flight_path_angle_deg);
            }
            const char*reason="timeout";bool completed=false;
            while(elapsed<=420){
                TaemPathReference ref=experiment_eval(&route,station);
                ref.runway_along_m=geo.runway_along_m;ref.runway_cross_m=geo.runway_cross_m;
                ref.course_deg=geo.course_deg;ref.curvature_right_per_m=0;
                worst=fmax(worst,fabs(model.site.altitude+geo.altitude_above_runway_m-ref.altitude_m));
                AeroForces f=aero_compute(&model.world,&model.aero,state.position_i_m,state.velocity_i_mps,state.ut_s,state.mass_kg,state.attitude.aoa_rad,state.attitude.bank_rad);
                double load=fabs(f.lift_n)/state.mass_kg/9.80665;
                qmax=fmax(qmax,f.dynamic_pressure_pa);gmax=fmax(gmax,load);
                if(qmax>model.vehicle.maximum_dynamic_pressure||gmax>model.vehicle.maximum_g_load){reason="limits";break;}
                if(geo.altitude_above_runway_m<=0){reason="ground";break;}
                if(station>=length-1e-5){completed=true;reason="completed";break;}
                double dt=fmin(.5,(length-station)/fmax(geo.ground_speed_mps,1));
                if(dt<1e-6){completed=true;reason="completed";break;}
                TaemTrackerOutput demand=taem_tracker_update(&model,&state,&geo,&ref,dt);
                if(!demand.valid){reason="tracker";break;}
                assert(fabs(demand.required_lateral_accel_mps2)<1e-6);
                double speed=geo.ground_speed_mps;
                if(terminal_propagator_step(&model,&state,&demand.control)!=TERMINAL_STEP_OK||!taem_geometry_state(&model,&state,&geo)){reason="propagation";break;}
                station+=.5*(speed+geo.ground_speed_mps)*dt;elapsed+=dt;
            }
            double endpoint=fabs(model.site.altitude+geo.altitude_above_runway_m-route.profile_final_altitude_m);
            bool feasible=completed&&steep<=model.guidance.taem_glide_slope+1e-6&&worst<=300&&endpoint<=150&&fabs(geo.airspeed_mps-model.guidance.final_alignment_speed)<=.12*model.guidance.final_alignment_speed;
            printf("ZERO_TURN length=%.6f bow=%.6f completed=%d feasible=%d elapsed=%.6f station=%.6f altitude=%.6f speed=%.6f fpa=%.6f steep=%.6f track=%.6f endpoint=%.6f qmax=%.6f gmax=%.6f sag=%.6f sagLength=%.6f reason=%s\n",length,bow,completed,feasible,elapsed,station,model.site.altitude+geo.altitude_above_runway_m,geo.airspeed_mps,geo.flight_path_angle_deg,steep,worst,endpoint,qmax,gmax,route.profile_initial_sag_m,route.profile_initial_sag_length_m,reason);
        }
        return 0;
    }
    return 3;
}
'''
planner=(ROOT/'CLanding/taem_planner.c').read_text()+r'''
bool experiment_reference(const TerminalModel*m,const TerminalDynamicState*s,const TaemGeometryState*g,double length,double bow,TaemRoute*r){
    memset(r,0,sizeof(*r));r->valid=true;r->count=3;r->lead_count=1;r->arc_count=1;r->hac.radius_m=1;
    r->length_m=r->lead_length_m=r->profile_total_length_m=length;
    r->p0_along_m=g->runway_along_m;r->p0_cross_m=r->p1_cross_m=r->p2_cross_m=r->p3_cross_m=g->runway_cross_m;
    r->p1_along_m=r->p0_along_m+length/3;r->p2_along_m=r->p0_along_m+2*length/3;r->p3_along_m=r->p0_along_m+length;
    for(size_t i=0;i<TAEM_ROUTE_LEAD_LUT_POINTS;i++)r->lead_arc_fraction_lut[i]=(float)i/(TAEM_ROUTE_LEAD_LUT_POINTS-1);
    r->initial_course_deg=r->runway_heading_deg=m->site.runway_heading;
    r->profile_start_altitude_m=m->site.altitude+g->altitude_above_runway_m;r->profile_start_fpa_deg=g->flight_path_angle_deg;
    r->profile_final_altitude_m=m->site.altitude+m->guidance.final_approach_distance*tan(radians(m->guidance.final_glide_slope));
    r->profile_final_slope_deg=m->guidance.final_glide_slope;r->profile_midpoint_offset_m=bow;
    return taem_route_limit_initial_vertical_authority(m,s,g,r);
}
TaemPathReference experiment_eval(const TaemRoute*r,double station){
    TaemPathReference ref={.station_m=station};
    vertical_profile(&(TerminalModel){.site.altitude=r->profile_final_altitude_m,.guidance.taem_glide_slope=r->profile_final_slope_deg},r->profile_start_altitude_m,r->profile_start_fpa_deg,r->profile_total_length_m,station,r->profile_midpoint_offset_m,0,0,0,0,r->profile_initial_sag_m,r->profile_initial_sag_length_m,&ref.altitude_m,&ref.flight_path_angle_deg,&ref.vertical_curvature_per_m);
    return ref;
}
'''
report={'sourceCommit':subprocess.check_output(['git','rev-parse','HEAD'],cwd=ROOT,text=True).strip(),'policy':'Focused configured sample0. Native tracker, servo and unpowered forces with zero lateral demand each step; native analytic Hermite/bow and initial upper authority shaping. No bank, physical runway endpoint, route adoption or Final tail. 7 lengths40..100km and13 bows +/-.3drop, then refined60..80km every1km with bows-2000..2000 every250m; same24deg/300mtrack/150mendpoint/160+-12%speed diagnostics. q/load/ground/timechecked; slope sampled4097stations. Not a proof that every straight profile fails or an optimistic mathematical bound on banked routes.', 'hashes':{'fixture':hashlib.sha256(fixture.encode()).hexdigest(),'planner':hashlib.sha256(planner.encode()).hexdigest(),**{p:hashlib.sha256((ROOT/p).read_bytes()).hexdigest() for p in ['Configuration/sts-live.json','ShuttleSim/reference-model/sts_aero_live20261010.csv','ShuttleSim/reference-model/stsn_attitude_identified.ini','ShuttleSim/reference-model/kerbin_atmosphere_identified.csv','Validation/MM305FeasibilityPropertyTests.c']}},'trials':[]}
subprocess.run(['make','-C','CLanding'],cwd=ROOT,check=True,stdout=subprocess.DEVNULL)
dry=subprocess.check_output(['make','-n','-C','CLanding','mm305-feasibility-test'],cwd=ROOT,text=True).replace('\\\n',' ')
link=next(shlex.split(s) for s in dry.splitlines() if s.startswith('clang ') and 'MM305FeasibilityPropertyTests.c' in s)
with tempfile.TemporaryDirectory(prefix='ksl-zero-turn-') as temp:
 temp=Path(temp);p=temp/'planner.c';p.write_text(planner);obj=temp/'planner.o';f=temp/'fixture.c';f.write_text(fixture);binary=temp/'fixture'
 flags=link[1:link.index('../Validation/MM305FeasibilityPropertyTests.c')]
 subprocess.run(['clang',*flags,'-c',str(p),'-o',str(obj)],cwd=ROOT/'CLanding',check=True)
 link=[str(obj) if x=='build/taem_planner.o' else str(f) if x=='../Validation/MM305FeasibilityPropertyTests.c' else x for x in link];link[link.index('-o')+1]=str(binary)
 subprocess.run(link,cwd=ROOT/'CLanding',check=True);report['binarySHA256']=hashlib.sha256(binary.read_bytes()).hexdigest()
 log=HERE/'trace.log';env=dict(os.environ,KSP_LANDER_MODEL_PROFILE='sts-live-20261010')
 with log.open('w') as out:r=subprocess.run([str(binary),'--configuration','Configuration/sts-live.json','--mass-kg','25123.15','--sample-index','0'],cwd=ROOT,env=env,stdout=out,stderr=subprocess.STDOUT)
 assert r.returncode==0
 report['logSHA256']=hashlib.sha256(log.read_bytes()).hexdigest()
 for line in log.read_text().splitlines():
  if line.startswith('ZERO_TURN'):report['trials'].append({k:v if k=='reason' else float(v) for k,v in re.findall(r'(\w+)=([^\s]+)',line)})
 assert len(report['trials'])==448
 complete=[t for t in report['trials'] if t['completed']];slope=[t for t in complete if t['steep']<=24+1e-6]
 report['summary']={'completed':len(complete),'feasible':sum(t['feasible'] for t in complete),'slopeCompliantCompleted':len(slope),'bestTrackingCompleted':min(complete,key=lambda t:t['track']) if complete else None,'bestTrackingSlopeCompliant':min(slope,key=lambda t:t['track']) if slope else None,'maximumSpeedSlopeTrackingEndpoint':max((t for t in slope if t['track']<=300 and t['endpoint']<=150),key=lambda t:t['speed'],default=None),'feasibleTrials':[t for t in complete if t['feasible']]}
 (HERE/'result.json').write_text(json.dumps(report,indent=2)+'\n')
 with (HERE/'trials.csv').open('w') as out:w=csv.DictWriter(out,fieldnames=report['trials'][0].keys());w.writeheader();w.writerows(report['trials'])
 print(json.dumps(report['summary']),flush=True)
