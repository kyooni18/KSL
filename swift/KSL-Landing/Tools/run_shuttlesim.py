#!/usr/bin/env python3
"""Independent ShuttleSim lockstep runner for the real native Swift controller.
Never changes simulator source, attitude coefficients or aerodynamic tables.
"""
from __future__ import annotations
from native_telemetry_publication import NativeTelemetryPublication
import argparse, gzip, hashlib, json, math, pathlib, select, socket, subprocess, time

ROOT = pathlib.Path(__file__).resolve().parents[3]
PACKAGE = pathlib.Path(__file__).resolve().parents[1]
WIRE_KEYS = {'step','throttle','thrust_dir','pitch_input','roll_input','yaw_input',
             'gear_down','brakes','airbrakes','wheel_steering'}

def digest(path: pathlib.Path) -> str:
    h=hashlib.sha256()
    with path.open('rb') as f:
        for block in iter(lambda:f.read(1024*1024),b''):h.update(block)
    return h.hexdigest()

def run(args: argparse.Namespace) -> dict:
    output=args.output.resolve(); output.mkdir(parents=True,exist_ok=False)
    sim=ROOT/'ShuttleSim/build/shuttlesim'
    binary=PACKAGE/'.build/release/ksl-landing'
    attitude=ROOT/'ShuttleSim/reference-model/sts_attitude_live20261028.ini'
    aero=ROOT/'ShuttleSim/reference-model/sts_aero_cells20261029.csv'
    atmosphere=ROOT/'ShuttleSim/reference-model/kerbin_atmosphere_identified.csv'
    scenario=args.scenario.resolve()
    if args.mass_factor != 1:
        lines=scenario.read_text().splitlines(); found=False
        for i,line in enumerate(lines):
            if line.startswith('mass_kg='):
                lines[i]='mass_kg='+str(float(line.split('=',1)[1])*args.mass_factor);found=True
        if not found:raise ValueError('Payload mass-only dispersion requires an explicit mass_kg')
        scenario=output/'mass-only-scenario.ini';scenario.write_text('\n'.join(lines)+'\n')
    rx=socket.socket(socket.AF_INET,socket.SOCK_DGRAM);rx.bind(('127.0.0.1',0));rx.settimeout(5)
    reservation=socket.socket(socket.AF_INET,socket.SOCK_DGRAM);reservation.bind(('127.0.0.1',0))
    command_port=reservation.getsockname()[1]
    sim_args=[str(sim),'--scenario',str(scenario),'--atmosphere',str(atmosphere),
              '--aero',str(aero),'--attitude',str(attitude),'--dt','0.02','--rate','max',
              '--telemetry-hz','10','--max-sim-time',str(args.duration),
              '--plant-lift-scale',str(args.lift_scale),'--plant-drag-scale',str(args.drag_scale),
              '--command-port',str(command_port),'--telemetry-port',str(rx.getsockname()[1]),
              '--web-telemetry-port',str(args.web_port if args.publish_web else 0),
              '--lockstep','--quiet','--stop-confirmation-seconds','3']
    # Adopt the exact runway specification used by the independent scenario;
    # this changes adapter configuration, never simulator geometry or physics.
    native=json.loads(subprocess.check_output([str(binary),'configuration'],cwd=ROOT))
    ini={}
    for line in scenario.read_text().splitlines():
        if '=' in line and not line.lstrip().startswith('#'):
            k,v=line.split('=',1);ini[k.strip()]=v.strip()
    for key,target in [('runway_latitude_deg','latitude'),('runway_longitude_deg','longitude'),('runway_heading_deg','heading')]:
        if key in ini:native['runway'][target]=math.radians(float(ini[key]))
    for key,target in [('runway_elevation_m','elevation'),('runway_length_m','length'),('runway_width_m','width')]:
        if key in ini:native['runway'][target]=float(ini[key])
    configuration=output/'native-configuration.json';configuration.write_text(json.dumps(native,indent=2))
    controller_args=[str(binary),'shuttlesim-controller','--attitude-model',str(attitude),
                     '--configuration',str(configuration),'--window-index',str(args.window_index)]
    if args.legacy_window is not None:
        controller_args+=['--legacy-window',str(args.legacy_window)]
    if args.orbit_experiment:
        controller_args+=['--unqualified-orbit-experiment','--dry-mass',str(args.dry_mass),'--engine-isp','285']
    summary={'outcome':'not-started','evidence':'independent-ShuttleSim-offline-unqualified',
             'category':args.category,'liftScale':args.lift_scale,'dragScale':args.drag_scale,
             'massFactor':args.mass_factor,'payloadPositionModel':'unavailable',
             'simulatorPhysicsModified':False,'sources':{str(p.relative_to(ROOT)) if p.is_relative_to(ROOT) else str(p):digest(p)
                  for p in [sim,binary,attitude,aero,atmosphere,scenario,configuration]},
             'command':controller_args,'simulatorCommand':sim_args,
             'phases':[],'steps':0,'lowPressureUnsupportedSteps':0,'maxGeometryErrorMeters':0,
             'maximumDynamicPressure':0,'maximumLoad':0,'maximumAbsCrossTrack':0,
             'saturatedSteps':0,'touchdown':None,'rolloutStopped':False}
    processes=[]; start=time.monotonic(); last_print=-math.inf; last_telemetry_ut=None
    publication=None
    try:
        if args.publish_web:
            publication=NativeTelemetryPublication(output,scenario,configuration,args.category,time.time())
        with (output/'simulator.stderr.log').open('w') as se, (output/'controller.stderr.log').open('w') as ce, \
             gzip.open(output/'telemetry.jsonl.gz','wt') as tlog,gzip.open(output/'controller.jsonl.gz','wt') as clog:
            reservation.close()
            simulator=subprocess.Popen(sim_args,cwd=ROOT,stdout=subprocess.PIPE,stderr=se,text=True)
            controller=subprocess.Popen(controller_args,cwd=ROOT,stdin=subprocess.PIPE,stdout=subprocess.PIPE,stderr=ce,text=True,bufsize=1)
            processes=[controller,simulator]
            while time.monotonic()-start < args.wall_limit:
                try: packet,_=rx.recvfrom(65535)
                except socket.timeout:
                    if simulator.poll() is None and controller.poll() is None:continue
                    summary['outcome']='process-ended';break
                telemetry=json.loads(packet)
                if telemetry.get('type')!='telemetry':continue
                tlog.write(packet.decode()+'\n')
                # UDP retries and the terminal publication may repeat one UT.
                # Preserve the packet, never drive the controller twice at it.
                if last_telemetry_ut == telemetry.get('ut'):
                    summary['duplicateTelemetryPackets']=summary.get('duplicateTelemetryPackets',0)+1
                    continue
                last_telemetry_ut=telemetry['ut']
                controller.stdin.write(packet.decode()+'\n');controller.stdin.flush()
                ready,_,_=select.select([controller.stdout],[],[],min(args.wall_limit,1800))
                if not ready:summary['outcome']='controller-response-timeout';break
                line=controller.stdout.readline()
                if not line:summary['outcome']='controller-error';break
                command=json.loads(line);clog.write(line)
                summary['steps']+=1
                phase=command['phase']
                if not summary['phases'] or summary['phases'][-1]!=phase:summary['phases'].append(phase)
                if 'plan' in command:summary['planning']=command['plan']
                if 'burnResult' in command:summary['burn']=command['burnResult']
                if 'entryError' in command:summary['entryError']=command['entryError']
                summary['lowPressureUnsupportedSteps']+=int(command.get('lowPressureControlUnsupported',False))
                summary['maxGeometryErrorMeters']=max(summary['maxGeometryErrorMeters'],command.get('geometryErrorMeters',0))
                summary['maximumDynamicPressure']=max(summary['maximumDynamicPressure'],telemetry['aero']['q_pa'])
                summary['maximumLoad']=max(summary['maximumLoad'],abs(telemetry['aero']['lift_n'])/(telemetry['vehicle']['mass_kg']*9.80665))
                summary['maximumAbsCrossTrack']=max(summary['maximumAbsCrossTrack'],abs(telemetry['runway']['cross_m']))
                control=command.get('record',{}).get('control',{})
                summary['saturatedSteps']+=int(any(control.get(k,False) for k in ['pitchSaturated','rollSaturated','yawSaturated']))
                summary['lastTelemetry']=telemetry
                summary['lastGuidance']=command.get('record',{}).get('guidance')
                if publication is not None:
                    publication.record(telemetry,command,summary['steps'])
                if telemetry['ground']['touchdown_seen'] and summary['touchdown'] is None:
                    summary['touchdown']={'time':telemetry['ut'],'runway':telemetry['runway'],
                        'velocity':telemetry['velocity'],'ground':telemetry['ground']}
                summary['rolloutStopped']=telemetry['ground']['stopped'] and telemetry['ground']['on_runway_touchdown']
                if telemetry['sim_time']-last_print>=200 or command.get('terminal'):
                    print(json.dumps({'t':telemetry['sim_time'],'phase':phase,'height':telemetry['position']['altitude_m'],
                         'speed':telemetry['velocity']['air_mps'],'cross':telemetry['runway']['cross_m']},separators=(',',':')),flush=True)
                    last_print=telemetry['sim_time']
                if command.get('terminal'):
                    summary['outcome']='offline-success' if phase=='complete' and summary['rolloutStopped'] else 'offline-failure'
                    break
                # A strict whitelist prevents nested diagnostic JSON from being
                # accidentally parsed as simulator commands by its C parser.
                wire={k:v for k,v in command.items() if k in WIRE_KEYS}
                rx.sendto(json.dumps(wire,separators=(',',':')).encode(),('127.0.0.1',command_port))
                if telemetry['sim_time']>=args.duration-0.1:summary['outcome']='simulation-horizon';break
            else:summary['outcome']='wall-limit'
    except Exception as exc:
        summary['outcome']='runner-error';summary['error']=repr(exc)
    finally:
        for process in processes:
            if process.poll() is None:
                process.terminate()
                try:process.wait(timeout=5)
                except subprocess.TimeoutExpired:process.kill();process.wait()
        rx.close();reservation.close()
        summary['wallSeconds']=time.monotonic()-start
        summary['sourceHashesUnchanged']=all(digest(ROOT/p if not p.startswith('/') else pathlib.Path(p))==value for p,value in summary['sources'].items())
        summary['controllerError']=(output/'controller.stderr.log').read_text() if (output/'controller.stderr.log').exists() else ''
        for line in (output/'simulator.stderr.log').read_text().splitlines():
            if line.startswith('SHUTTLESIM_SUMMARY '):
                summary['independentGroundSummary']=json.loads(line.partition(' ')[2])
        summary['stopDwellConfirmation']=bool(summary['phases'] and summary['phases'][-1]=='complete')
        (output/'summary.json').write_text(json.dumps(summary,indent=2)+'\n')
        if publication is not None:
            publication.finish(summary)
    print('RESULT',summary['outcome'],'steps',summary['steps'],'wall',round(summary['wallSeconds'],2),flush=True)
    return summary

def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--scenario',type=pathlib.Path,default=PACKAGE/'Fixtures/sts-polar100-orbit.ini')
    p.add_argument('--output',type=pathlib.Path,required=True)
    p.add_argument('--orbit-experiment',action='store_true')
    p.add_argument('--window-index',type=int,default=0,help='Rank in the landing-aware deorbit selection')
    p.add_argument('--legacy-window',type=int,help='Deliberately fly raw orbital-search candidate N (envelope test)')
    p.add_argument('--dry-mass',type=float,default=21000)
    p.add_argument('--lift-scale',type=float,default=1)
    p.add_argument('--drag-scale',type=float,default=1)
    p.add_argument('--mass-factor',type=float,default=1)
    p.add_argument('--category',choices=['nominal','mass-only-dispersion','moderate-exploratory','extreme-stress'],default='nominal')
    p.add_argument('--duration',type=float,default=30000)
    p.add_argument('--publish-web',action='store_true',help='Publish native run and replay in ShuttleSim telemetry web')
    p.add_argument('--web-port',type=int,default=8797,help='ShuttleSim telemetry UDP port for the simulator-mode web server')
    p.add_argument('--wall-limit',type=float,default=600)
    a=p.parse_args()
    if min(a.lift_scale,a.drag_scale,a.mass_factor,a.duration,a.wall_limit)<=0:p.error('Scales and limits must be positive')
    result=run(a)
    raise SystemExit(0 if result['outcome']=='offline-success' else 2)
if __name__=='__main__':main()
