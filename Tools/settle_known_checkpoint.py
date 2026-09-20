#!/usr/bin/env python3
"""Freeze a KSP checkpoint as soon as independently measured physics is coherent."""
from __future__ import annotations
import argparse,hashlib,json,time
from pathlib import Path
from ksp_test_guard import connect

def main() -> None:
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--profile',type=Path,required=True)
    parser.add_argument('--save',required=True)
    args=parser.parse_args();profile=json.loads(args.profile.read_text())
    if profile['saveName']!=args.save:
        raise ValueError('Physics profile belongs to a different saved flight')
    source=Path(profile['savePath'])
    if source.name!=args.save+'.sfs':raise ValueError('Profile save path does not match requested save')
    if hashlib.sha256(source.read_bytes()).hexdigest()!=profile['saveSHA256']:
        raise ValueError('Saved flight changed since its physics profile was certified')
    conn=connect('KSP measured-checkpoint startup',timeout=8)
    try:
        sc=conn.space_center;v=sc.active_vessel
        if v.name!=profile['vessel']:raise ValueError('Wrong active vessel')
        v.control.throttle=0.0;v.control.sas=False;v.control.rcs=False
        # Preserve the checkpoint's control deflections during the short unpack.
        # Zeroing the saved pitch before the controller owned it caused a
        # measured 40-degree nose-down excursion in the former startup window.
        deadline=time.monotonic()+20;first_coherent=None
        while time.monotonic()<deadline:
            ut=float(sc.ut);mass=float(v.mass);dry=float(v.dry_mass)
            parts=len(v.parts.all);thrust=float(v.thrust)
            if abs(thrust)>1.0:raise RuntimeError('Unexpected propulsion in unpowered checkpoint')
            coherent=(abs(mass-profile['mass'])<=max(1.0,profile['mass']*.001) and
                abs(dry-profile['dryMass'])<=max(1.0,profile['dryMass']*.001) and
                parts==profile['partCount'])
            if coherent:
                if first_coherent is None:first_coherent=ut
                if conn.krpc.paused or ut-first_coherent>=.06:
                    conn.krpc.paused=True
                    flight=v.flight()
                    state={'ready':True,'ut':float(sc.ut),'vessel':v.name,
                        'meanAltitude':float(flight.mean_altitude),'mass':float(v.mass),
                        'dryMass':float(v.dry_mass),'availableThrust':float(v.available_thrust),
                        'actualThrust':float(v.thrust),'partCount':parts,'paused':bool(conn.krpc.paused),
                        'railsWarpFactor':int(sc.rails_warp_factor),'physicsWarpFactor':int(sc.physics_warp_factor),
                        'pitch':float(flight.pitch),'aoa':float(flight.angle_of_attack),
                        'profile':str(args.profile)}
                    print(json.dumps(state),flush=True);return
            else:first_coherent=None
            time.sleep(.02)
        raise RuntimeError(f'Known physics mismatch: ut={ut}, mass={mass}, dry={dry}, parts={parts}, thrust={thrust}')
    finally:
        # A failed preparation must never leave an uncontrolled descending craft.
        try:conn.krpc.paused=True
        finally:conn.close()

if __name__=='__main__':main()
