#!/usr/bin/env python3
"""Compare decoded Euler rates against body-rate kinematics, not torque fits."""
import gzip
import json
import math
from pathlib import Path
HERE=Path(__file__).resolve().parent

def analyze(row):
    pitch_errors=[];heading_errors=[]
    with gzip.open(row['result']['guidance'],'rt') as stream:
        for line in stream:
            t=json.loads(line).get('snapshot',{}).get('telemetry',{})
            keys=('pitch','roll','bodyPitchRate','bodyYawRate','pitchRate','headingRate')
            if any(t.get(k) is None for k in keys) or t.get('vesselSituation')=='landed':
                continue
            if t.get('trueAirSpeed',0)<100 or abs(t['pitch'])>60:
                continue
            phi=math.radians(t['roll']);theta=math.radians(t['pitch'])
            pitch_rate=t['bodyPitchRate']*math.cos(phi)-t['bodyYawRate']*math.sin(phi)
            heading_rate=(t['bodyPitchRate']*math.sin(phi)+t['bodyYawRate']*math.cos(phi))/math.cos(theta)
            pitch_errors.append(abs(t['pitchRate']-pitch_rate))
            heading_errors.append(abs(t['headingRate']-heading_rate))
    def stats(values):
        values.sort()
        return {'n':len(values),'medianAbsoluteErrorDegPerSecond':values[len(values)//2],
                'p95AbsoluteErrorDegPerSecond':values[int(.95*(len(values)-1))]} if values else {}
    return {'pitchEulerRate':stats(pitch_errors),'headingEulerRate':stats(heading_errors)}

prior=json.loads((HERE.parent/'Body-Coordinate-Plant/result.json').read_text())
current=json.loads((HERE/'result.json').read_text())
report={'selection':'native snapshots with body rates, airspeed>=100m/s and |pitch|<=60deg; finite-difference coordinate rates compared to instantaneous ZYX kinematics. A diagnostic, not moment fitting.',
        'before':analyze(prior[0]),'after':analyze(current[0])}
(HERE/'rate-check.json').write_text(json.dumps(report,indent=2)+'\n')
