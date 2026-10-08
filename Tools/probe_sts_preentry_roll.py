#!/usr/bin/env python3
"""Short original STS vacuum roll-response bench, not an entry/landing attempt."""
import argparse
import hashlib
import json
import math
from pathlib import Path
import subprocess
import sys
import time
from headless_flight import restore_quicksave, secure_failed_test_state
from ksp_test_guard import connect

ROOT = Path(__file__).resolve().parents[1]


def validate_sample(s, start_ut, config):
    if not all(math.isfinite(s[k]) for k in ('ut','altitude','q','thrust','roll_rate','mass')):
        raise RuntimeError('Non-finite bench telemetry')
    if s['altitude'] <= 70000 or s['q'] > 1e-6:
        raise RuntimeError('Vacuum bench crossed atmospheric boundary')
    if abs(s['thrust']) > 1 or s['SAS'] or s['RCS']:
        raise RuntimeError('Propulsion or assistance active')
    if abs(s['roll_rate']) > config['guidance']['entryRollRate']:
        raise RuntimeError('Roll motion limit exceeded')
    if not start_ut-.01 <= s['ut'] <= start_ut+3:
        raise RuntimeError('Bench clock outside allowed window')


def body_roll_rate(angular, rotation):
    x,y,z,w = rotation
    body_y = 2*(x*y-z*w)*angular[0] + (1-2*(x*x+z*z))*angular[1] + 2*(y*z+x*w)*angular[2]
    return -body_y*180/math.pi


def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--profile',type=Path,required=True)
    ap.add_argument('--output',type=Path,required=True)
    args=ap.parse_args()
    profile=json.loads(args.profile.read_text())
    config=json.loads((ROOT/'Configuration/sts.json').read_text())
    if config['vehicle']['modelId']!='STS' or profile['vessel']!='STS':
        raise RuntimeError('Original STS required')
    if profile['saveName']!='STS-campaign-72km-20261008':
        raise RuntimeError('Wrong checkpoint')
    if hashlib.sha256(Path(profile['savePath']).read_bytes()).hexdigest()!=profile['saveSHA256']:
        raise RuntimeError('Checkpoint hash changed')
    report={'scope':'Vacuum reaction-wheel roll bench. No Entry engagement or qualification; not coordinated atmospheric authority.','profile':profile,'samples':[],'sourceSHA256':hashlib.sha256(Path(__file__).read_bytes()).hexdigest()}
    conn=None
    args.output.parent.mkdir(parents=True,exist_ok=True)
    try:
        restore_quicksave(ROOT,profile['saveName'])
        prepared=subprocess.run([sys.executable,str(ROOT/'Tools/settle_known_checkpoint.py'),'--profile',str(args.profile),'--save',profile['saveName']],capture_output=True,text=True,timeout=30,check=True)
        report['startup']=json.loads(prepared.stdout.strip().splitlines()[-1])
        conn=connect('Original STS vacuum roll bench',timeout=8)
        sc=conn.space_center;v=sc.active_vessel
        if v.name!='STS' or not conn.krpc.paused or abs(v.mass-profile['mass'])>1:
            raise RuntimeError('Bench requires frozen coherent original STS')
        v.control.throttle=0;v.control.sas=False;v.control.rcs=False
        torque=v.available_reaction_wheel_torque;inertia=v.moment_of_inertia
        nominal=abs(torque[0][1])/inertia[1]*180/math.pi
        if not math.isfinite(nominal) or nominal<=0:
            raise RuntimeError('No reported roll actuator authority')
        # Target acceleration uses only 1 deg/s² (below the existing 1.8 deg/s²
        # reference limit). Commands remain small, with a symmetric brake pulse.
        amplitude=min(.15,1.0/nominal,config['guidance']['entryRollAcceleration']/nominal)
        report.update(torque=torque,inertia=inertia,nominal_roll_accel=nominal,amplitude=amplitude)
        flight=v.flight();frame=v.orbit.body.non_rotating_reference_frame
        definitions={'ut':(getattr,sc,'ut'),'angular':(v.angular_velocity,frame),'rotation':(v.rotation,frame),
                     'altitude':(getattr,flight,'mean_altitude'),'q':(getattr,flight,'dynamic_pressure'),
                     'thrust':(getattr,v,'thrust'),'mass':(getattr,v,'mass'),
                     'SAS':(getattr,v.control,'sas'),'RCS':(getattr,v.control,'rcs'),'control_roll':(getattr,v.control,'roll')}
        streams={k:conn.add_stream(*definition) for k,definition in definitions.items()}
        for stream in streams.values():stream.rate=50;stream.start()
        def read():
            s={k:f() for k,f in streams.items()}
            s['roll_rate']=body_roll_rate(s['angular'],s['rotation'])
            s['wall_time']=time.monotonic()
            return s
        initial=read();start=initial['ut'];validate_sample(initial,start,config)
        if abs(initial['roll_rate'])>.1:
            raise RuntimeError('Initial roll rate too large for small pulse bench')
        v.control.roll=0
        previous=start;last_command=None;deadline=time.monotonic()+8
        conn.krpc.paused=False
        while time.monotonic()<deadline:
            s=read();validate_sample(s,start,config)
            if s['ut'] <= previous:
                time.sleep(.005);continue
            if s['ut']-previous>.16:
                raise RuntimeError('Bench missed its observation interval')
            elapsed=s['ut']-start
            command=0 if elapsed<.3 else amplitude if elapsed<.7 else -amplitude if elapsed<1.1 else 0
            s['elapsed']=elapsed;s['issued_roll']=command
            report['samples'].append(s)
            if command!=last_command:v.control.roll=command;last_command=command
            previous=s['ut']
            if elapsed>=1.5:
                report['completed']=True;break
        else:raise RuntimeError('Bench did not finish within wall-clock deadline')
    except BaseException as exc:
        report['error']=str(exc)
        raise
    finally:
        try:
            if conn:
                try:
                    conn.krpc.paused=True
                    sc.active_vessel.control.roll=0
                finally:conn.close()
        finally:
            try:report['cleanup']=secure_failed_test_state(ROOT,profile['saveName'],True)
            finally:args.output.write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps({'completed':report.get('completed',False),'samples':len(report['samples']),'output':str(args.output)}))


if __name__=='__main__':main()
