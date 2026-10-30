#!/usr/bin/env python3
"""Recompute runway geometry and target smoothness from native flight records."""
import argparse, base64, csv, gzip, json, math, pathlib, re

def dot(a,b): return sum(x*y for x,y in zip(a,b))
def scale(a,k): return [x*k for x in a]
def sub(a,b): return [x-y for x,y in zip(a,b)]
def norm(a): return math.sqrt(dot(a,a))
def unit(a): return scale(a,1/max(1e-30,norm(a)))
def vector(a): return [a[k] for k in ('x','y','z')]
def wrap(x): return math.atan2(math.sin(x),math.cos(x))
def analyze(path):
    cfg=next(path.glob('*configuration.json'))
    c=json.loads(cfg.read_text());r=c['runway'];g=c['guidance'];v=c['vehicle'];R=c['planet']['radius']
    lat,lon=r['latitude'],r['longitude']
    up=[math.cos(lat)*math.cos(lon),math.cos(lat)*math.sin(lon),math.sin(lat)]
    east=[-math.sin(lon),math.cos(lon),0];north=[-math.sin(lat)*math.cos(lon),-math.sin(lat)*math.sin(lon),math.cos(lat)]
    along=[n*math.cos(r['heading'])+e*math.sin(r['heading']) for n,e in zip(north,east)]
    right=[e*math.cos(r['heading'])-n*math.sin(r['heading']) for n,e in zip(north,east)]
    span=max(250,g['finalSpeed']**2/g['preflareAccelerationReserve']*(g['outerGlideAngle']-g['shallowGlideAngle']))
    shallow=g['touchdownAlong']-g['shallowSegmentLength'];pre=shallow-span
    ph=v['gearHeight']+g['shallowSegmentLength']*math.tan(g['shallowGlideAngle'])+span*(math.tan(g['outerGlideAngle'])+math.tan(g['shallowGlideAngle']))/2
    exitheight=ph-math.tan(g['outerGlideAngle'])*(-g['finalDistance']-pre)
    entry=c.get('entry',c.get('entrySettings',{}));minimum=min(entry.get('terminalMinimumAltitude',22000),g.get('hacAcquisitionAltitude',exitheight),exitheight)
    maximum=entry.get('terminalMaximumAltitude',30000);maxmach=entry.get('terminalTargetMach',2.5)+entry.get('terminalMachHalfWidth',.35)+.1
    maxspeed=g.get('terminalPlanningMaximumSpeed',1300)
    capture_radius=entry.get('captureTargetRange')
    energy_source=pathlib.Path(__file__).resolve().parents[1]/'Sources/KSLGuidance/Energy/EnergyManagementEngine.swift'
    touchdown_fraction=float(re.search(r'minimumTouchdownFraction\s*=\s*([0-9.]+)',energy_source.read_text()).group(1))
    file=path/'live.jsonl';opener=open
    if not file.exists():file=path/'controller.jsonl.gz';opener=gzip.open
    rows=[];phases=[];forecasts=[];lastforecast=None;firstsearch=None;handoff=None;touchdown=None;stop=None
    with opener(file,'rt') as stream:
      for line in stream:
        try:a=json.loads(line);a=a.get('record',a);s=a['state'];d=a['guidance']
        except (ValueError,KeyError):continue
        p=vector(s['position']);vel=vector(s['velocity']);u=unit(p);vertical=dot(vel,u);horizontal=sub(vel,scale(u,vertical))
        tangent=sub(u,scale(up,dot(u,up)));angle=math.atan2(norm(tangent),dot(u,up));chart=scale(unit(tangent),R*angle)
        localEast=unit([-u[1],u[0],0]);localNorth=[-u[2]*localEast[1],u[2]*localEast[0],u[0]*localEast[1]-u[1]*localEast[0]]
        course=math.atan2(dot(vel,localEast),dot(vel,localNorth));toward=sub(up,scale(u,dot(up,u)));bearing=math.atan2(dot(toward,localEast),dot(toward,localNorth))
        h=norm(p)-R;ae=s['aerodynamics'];t=s['time'];target=d['target'];cap=d.get('capture',{})
        gates=[h<=maximum+max(0,-vertical)*g['terminalReplanInterval'],h>=minimum,ae['mach']<=maxmach,ae['airspeed']<=maxspeed]
        row=dict(time=t,altitude=h,mach=ae['mach'],range=R*angle,along=dot(chart,along),cross=dot(chart,right),course=math.degrees(course),headingError=math.degrees(wrap(bearing-course)),verticalSpeed=vertical,speed=ae['airspeed'],bankTarget=math.degrees(target['bank']),alphaTarget=math.degrees(target['angleOfAttack']),alphaActual=math.degrees(ae['angleOfAttack']),phase=d['phase'],searchEnvelope=all(gates),gateAltitudeUpper=gates[0],gateAltitudeLower=gates[1],gateMach=gates[2],gateSpeed=gates[3],mm305Searched=cap.get('mm305Searched',False),energyMargin=min(cap.get('excessEnergyMargin') or -1e300,cap.get('deficitEnergyMargin') or -1e300))
        if capture_radius is not None:
          row['searchEnvelope']=R*angle <= capture_radius+norm(horizontal)*g['terminalReplanInterval']
        if not rows:t0=t
        row['elapsed']=t-t0
        if not phases or phases[-1]['phase']!=d['phase']:phases.append(dict(phase=d['phase'],elapsed=t-t0,altitude=h,mach=ae['mach']))
        if a.get('planningProposal'):
          proposal=json.loads(base64.b64decode(a['planningProposal']))
          terminal=proposal.get('terminal')
          if terminal:
            envelope=terminal.get('envelope') or {}
            slow=envelope.get('slowestTouchdown');fast=envelope.get('fastestTouchdown')
            if slow is not None and fast is not None:
              row['energyMargin']=min(v['maximumTouchdownSpeed']**2-slow**2,
                fast**2-(v['landingSpeed']*touchdown_fraction)**2)/(2*9.80665)
        if row['mm305Searched'] and firstsearch is None:firstsearch=row.copy()
        if cap.get('mm305PlanAccepted') and handoff is None:handoff=row.copy()
        if s.get('contact') in ('mainGear','rolling','stopped') and touchdown is None:touchdown=row.copy()
        if d['phase']=='complete':stop=row.copy()
        if cap.get('predictedAt')!=lastforecast and cap.get('predictedAt') is not None:forecasts.append(cap);lastforecast=cap['predictedAt']
        rows.append(row)
    def metrics(key):
      data=[r for r in rows if r['phase']=='entry'];rates=[];acc=[];changes=0;lastsign=0
      for a,b in zip(data,data[1:]):
        dt=b['time']-a['time']
        if dt<=0:continue
        rate=(b[key]-a[key])/dt;rates.append((b['time'],rate));sign=1 if b[key]>0 else -1 if b[key]<0 else 0
        if lastsign and sign and sign!=lastsign:changes+=1
        if sign:lastsign=sign
      for a,b in zip(rates,rates[1:]):
        if b[0]>a[0]:acc.append((b[1]-a[1])/(b[0]-a[0]))
      return dict(signChanges=changes,rateDirectionChanges=sum(a[1]*b[1]<0 for a,b in zip(rates,rates[1:])),peakRate=max(map(lambda x:abs(x[1]),rates),default=0),peakAcceleration=max(map(abs,acc),default=0))
    errors=[r['alphaTarget']-r['alphaActual'] for r in rows if r['phase']=='entry']
    eligible=next((r for r in rows if r['searchEnvelope']),None)
    result=dict(source=str(file),captureTargetRange=capture_radius,phases=phases,firstSearchEnvelope=eligible,firstMM305Search=firstsearch,handoff=handoff,touchdown=touchdown,fullStop=stop,lastState=rows[-1],forecastCount=len(forecasts),lastForecast=forecasts[-1] if forecasts else None,bank=metrics('bankTarget'),alpha=metrics('alphaTarget'),alphaTargetMinusActual=dict(mean=sum(errors)/max(1,len(errors)),minimum=min(errors,default=0),maximum=max(errors,default=0),rms=math.sqrt(sum(x*x for x in errors)/max(1,len(errors)))),legacyGateLimits=dict(minimumAltitude=minimum,maximumAltitude=maximum,maximumMach=maxmach,maximumSpeed=maxspeed))
    result['machSnapshots']={str(m):min((r for r in rows if r['mach']>0),key=lambda r:abs(r['mach']-m)) for m in [7,5,3.8,2.95,2.5,2,1,.5]}
    with (path/'capture-geometry.csv').open('w') as f:
      writer=csv.DictWriter(f,fieldnames=rows[0].keys());writer.writeheader();writer.writerows(rows)
    (path/'capture-analysis.json').write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps({k:result[k] for k in ['phases','firstSearchEnvelope','firstMM305Search','handoff','touchdown','fullStop','bank','alpha','alphaTargetMinusActual','legacyGateLimits']},indent=2))
if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('run',type=pathlib.Path);analyze(p.parse_args().run)
