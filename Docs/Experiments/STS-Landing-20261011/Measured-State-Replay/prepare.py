#!/usr/bin/env python3
"""Build measured-course/body-state fixtures; leave historical goldens intact."""
import hashlib
import json
import math
import sys
from pathlib import Path
HERE=Path(__file__).resolve().parent
ROOT=HERE.parents[3]
sys.path.insert(0,str(ROOT/'ShuttleSim/tools'))
from identify_stsn_aero import _merge
LOGS={'l5':'2026-10-10T07-54-07Z','l6':'2026-10-10T08-28-20Z',
      'l7':'2026-10-11T06-40-54Z'}
def dot(a,b):return sum(x*y for x,y in zip(a,b))
def cross(a,b):return [a[1]*b[2]-a[2]*b[1],a[2]*b[0]-a[0]*b[2],a[0]*b[1]-a[1]*b[0]]
def norm(a):return [x/math.sqrt(dot(a,a)) for x in a]
def fixture(flight,phase,session,record,state):
    pos=state['position']['inertial'];velocity=state['motion']['velocity']
    axis=session['planet']['northAxis'];omega=session['planet']['rotationalSpeed']
    rotation=cross([x*omega for x in axis],pos)
    surf=[v-w for v,w in zip(velocity,rotation)]
    up=norm(pos);east=norm(cross(axis,up));north=cross(up,east);flow=norm(surf)
    speed=math.sqrt(dot(surf,surf))
    course=math.degrees(math.atan2(dot(surf,east),dot(surf,north)))%360
    fpa=math.degrees(math.asin(dot(flow,up)))
    attitude=state['attitude'];theta,phi,psi=[math.radians(attitude[k]) for k in ('pitch','roll','heading')]
    horizontal=[math.cos(psi)*n+math.sin(psi)*e for n,e in zip(north,east)]
    forward=[math.cos(theta)*h+math.sin(theta)*u for h,u in zip(horizontal,up)]
    right0=[math.cos(psi)*e-math.sin(psi)*n for n,e in zip(north,east)]
    up0=cross(right0,forward)
    bodyup=[math.cos(phi)*u+math.sin(phi)*r for u,r in zip(up0,right0)]
    bodyright=cross(forward,bodyup)
    windright=norm(cross(flow,up));windup=cross(windright,flow)
    bank=math.degrees(math.atan2(dot(bodyup,windright),dot(bodyup,windup)))
    aoa=math.degrees(math.atan2(-dot(flow,bodyup),dot(flow,forward)))
    beta=math.degrees(math.asin(dot(flow,bodyright)))
    # Recorded groups are asynchronously cached; reject a gross frame mismatch.
    assert abs(speed-state['motion']['trueAirSpeed'])<1
    assert abs(fpa-state['motion']['flightPathAngle'])<.1, (flight,phase,record['ut'],fpa,state['motion']['flightPathAngle'])
    assert abs(aoa-attitude['angleOfAttack'])<.15
    assert abs(beta-attitude['sideslip'])<.15
    name=f'{flight}-{phase}'
    data={'name':f'sts-measured-{name}','ut0':record['ut'],
          'latitude_deg':state['position']['latitude'],'longitude_deg':state['position']['longitude'],
          'altitude_m':state['position']['altitude'],'heading_deg':course,
          'surface_speed_mps':speed,'flight_path_angle_deg':fpa,'mass_kg':state['vehicle']['mass'],
          'initial_aoa_deg':aoa,'initial_bank_deg':bank,'initial_sideslip_deg':beta,
          'initial_body_pitch_deg':attitude['pitch'],'initial_body_roll_deg':attitude['roll'],
          'initial_body_heading_deg':attitude['heading'],
          'initial_body_pitch_rate_deg_s':attitude['bodyPitchRate'],
          'initial_body_roll_rate_deg_s':attitude['bodyRollRate'],
          'initial_body_yaw_rate_deg_s':attitude['bodyYawRate'],
          'deorbit_delta_v_mps':0,'deorbit_duration_s':0,'deorbit_delay_s':0,
          'runway_latitude_deg':-.0486111111,'runway_longitude_deg':-74.7283333333,
          'runway_elevation_m':70,'runway_heading_deg':90,'runway_length_m':2500,'runway_width_m':70}
    (HERE/f'{name}.ini').write_text('# Measured course, body Euler pose and angular rates; native source in sources.json.\n'+'\n'.join(f'{k}={v}' for k,v in data.items())+'\n')
    return {'ut':record['ut'],'bodyHeadingDeg':attitude['heading'],'courseDeg':course,
            'headingCourseDifferenceDeg':(attitude['heading']-course+180)%360-180,
            'loggedEulerBankDeg':attitude['roll'],'windBankDeg':bank,
            'computedAoADeg':aoa,'computedBetaDeg':beta,'surfaceSpeedMps':speed,
            'fixture':data,'nativeFields':state}
report={}
for flight,stamp in LOGS.items():
    stamps=[(stamp,{'mm305','final'})] if flight!='l7' else [(stamp,{'mm305'}),('2026-10-11T08-34-41Z',{'final'})]
    found={};skipped=[];sources=[]
    for stamp,allowed in stamps:
        path=ROOT/'FlightLogs'/f'{stamp}-STS-vehicle.jsonl'
        source={'sourceLog':str(path.relative_to(ROOT)),'sha256':hashlib.sha256(path.read_bytes()).hexdigest()}
        sources.append(source)
        session={};state={}
        for line in path.read_text().splitlines():
            rec=json.loads(line);kind=rec.get('recordType')
            if kind=='sessionStart':session=rec;continue
            if kind=='vehicleKeyframe':state=rec['fields']
            elif kind=='vehicleDelta':_merge(state,rec['fields'])
            else:continue
            phase={'TAEM':'mm305','Final Approach':'final'}.get(state['state']['phase'])
            if phase in allowed and phase not in found:
                copied=json.loads(json.dumps(state))
                try:
                    found[phase]={**fixture(flight,phase,session,rec,copied),**source}
                except AssertionError as error:
                    skipped.append({'ut':rec['ut'],'phase':phase,'sourceLog':source['sourceLog'],'reason':str(error) or 'vector/angle group mismatch'})
            if allowed.issubset(found):break
    report[flight]={'sources':sources,'states':found,'skippedIncoherentFrames':skipped}
    assert len(found)==2,(flight,'no coherent state for both phases')
high=json.loads(json.dumps(report['l5']['states']['mm305']))
high['fixture']['name']='sts-synthetic-high-measured-mm305'
high['fixture']['altitude_m']+=2000
high['syntheticChange']='L5 measured MM305 state with altitude +2000m; all other inlet quantities held fixed.'
(HERE/'high-mm305.ini').write_text('# Synthetic L5 measured state plus 2000m altitude.\n'+'\n'.join(f'{k}={v}' for k,v in high['fixture'].items())+'\n')
report['high']={'states':{'mm305':high}}
(HERE/'sources.json').write_text(json.dumps(report,indent=2)+'\n')
for k,v in report.items():
    print(k,{p:round(s['headingCourseDifferenceDeg'],3) for p,s in v['states'].items()})
