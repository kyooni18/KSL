#!/usr/bin/env python3
"""Feed decoded v32 measured states to offline MM304; never imports a KSP client."""
import json
import math
import os
from pathlib import Path
import subprocess

ROOT = Path(__file__).resolve().parents[1]
_default_log = ROOT / 'FlightLogs/2026-09-14T12-55-49Z-STS-N-vehicle.jsonl'
LOG = Path(os.environ.get('KSP_MM304_REPLAY_LOG', str(_default_log)))
if not LOG.is_absolute():
    LOG = ROOT / LOG

def merge(target, update):
    for key, value in update.items():
        if isinstance(value, dict):
            merge(target.setdefault(key, {}), value)
        else:
            target[key] = value

def dot(a, b):
    return sum(x*y for x, y in zip(a, b))

def cross(a, b):
    return [a[1]*b[2]-a[2]*b[1], a[2]*b[0]-a[0]*b[2], a[0]*b[1]-a[1]*b[0]]

def norm(a):
    length = math.sqrt(dot(a,a))
    return [x/length for x in a]

fields = {}
rows = []
previous_ut = -math.inf
for line in LOG.open():
    row = json.loads(line)
    if 'fields' not in row:
        continue
    if row['recordType'] == 'vehicleKeyframe':
        fields = {}
    merge(fields, row['fields'])
    if fields['state']['phase'] != 'MM304 Entry' or row['ut'] <= previous_ut:
        continue
    previous_ut = row['ut']
    position, motion = fields['position'], fields['motion']
    attitude, aero = fields['attitude'], fields['aero']
    guidance = fields['guidance']
    # Logged inertial vectors already use canonical (north = +Z) coordinates.
    up = norm(position['inertial'])
    north = norm([-up[2]*up[0], -up[2]*up[1], 1-up[2]*up[2]])
    east = norm(cross(north,up))
    rotation = cross([0,0,.000291570900559802],position['inertial'])
    air = [v-r for v,r in zip(motion['velocity'],rotation)]
    course = math.degrees(math.atan2(dot(air,east),dot(air,north))) % 360
    values = [row['ut'],position['altitude'],motion['trueAirSpeed'],motion['horizontalSpeed'],
        motion['verticalSpeed'],motion['flightPathAngle'],attitude['roll'],attitude['angleOfAttack'],
        attitude['pitch'],attitude['coordinateRollRate'],attitude['angleOfAttackRate'],
        aero['dynamicPressure'],aero['liftForce'],aero['dragForce'],fields['vehicle']['mass'],
        aero['gForce'],aero['stallFraction'],int(aero['stallFractionMeasured']),
        guidance['runwayAlongTrack'],guidance['runwayCrossTrack'],course,attitude['heading'],
        position['latitude'],position['longitude'],guidance['rangeToSite'],aero['mach'],
        *position['inertial'],*motion['velocity'],fields['runtime']['guidanceComputeMilliseconds']]
    if any(x is None for x in values):
        continue  # Initial samples may lack coordinate-rate telemetry.
    assert len(values)==33 and all(math.isfinite(x) for x in values)
    rows.append(' '.join(format(x,'.17g') for x in values))
subprocess.run([str(ROOT/'CLanding/build/mm304_offline_tests'),'--replay'],
    input='\n'.join(rows)+'\n',text=True,check=True)
