"""Decode the repeat flight's native stream and preserve contact chronology."""
import hashlib
import json
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[4]
sys.path.insert(0, str(ROOT))
from ShuttleSim.tools.identify_stsn_aero import _merge

RUN = ROOT / 'runs/20261011T092911Z-live-l7-current-verification'
SOURCE = ROOT / 'FlightLogs/2026-10-11T09-29-11Z-STS-vehicle.jsonl'
state, frames, transitions = {}, [], []
for line in SOURCE.read_text().splitlines():
    record = json.loads(line)
    kind = record.get('recordType')
    if kind == 'vehicleKeyframe':
        state = record['fields']
    elif kind == 'vehicleDelta':
        _merge(state, record['fields'])
    else:
        continue
    frame = json.loads(json.dumps({'ut': record['ut'], **state}))
    frames.append(frame)
    phase = state['state']['phase']
    if not transitions or transitions[-1]['state']['phase'] != phase:
        transitions.append(frame)
contact_index = next((i for i, f in enumerate(frames) if f['state']['vesselSituation'] == 'landed'), None)
observer = [json.loads(line) for line in (RUN / 'independent-observer.jsonl').read_text().splitlines()]
result = {
    'run': str(RUN.relative_to(ROOT)), 'save': 'ksl-20261011T064053Z-live-85km-10deg-landing-mm305',
    'guidanceCommit': '61fb088', 'flightSetupCommit': '61fb088', 'recordCommitBeforeFlightEnd': 'aeee7b7', 'initialUTAtPause': 6570.790097692647, 'explicitResumeUT': 6717.930097695858, 'sourceLog': str(SOURCE.relative_to(ROOT)),
    'sourceLogSHA256': hashlib.sha256(SOURCE.read_bytes()).hexdigest(),
    'status': json.loads((RUN / 'status.json').read_text()),
    'nativeSamples': len(frames), 'transitions': transitions,
    'lastFlyingBeforeFirstLanded': frames[contact_index-1] if contact_index else None,
    'firstLanded': frames[contact_index] if contact_index is not None else None,
    'lastNative': frames[-1],
    'preGroundResponseSample': next(f for f in frames if abs(f['ut']-7055.97009770323)<0.001),
    'phaseCounts': {p: sum(f['state']['phase']==p for f in frames) for p in {f['state']['phase'] for f in frames}},
    'finalProbe': json.loads((RUN / 'final-probe.json').read_text()), 'observerSamples': len(observer),
    'lastObserver': observer[-1] if observer else None,
    'nativeInvariants': {
        'sasOff': all(not f['state']['sasEnabled'] for f in frames),
        'airbrakesOff': all(not f['state']['airbrakes'] for f in frames),
        'zeroThrust': all(abs(f['vehicle']['currentThrust']) < 1 for f in frames),
        'zeroThrottle': all(f['command']['targetThrottle'] == 0 for f in frames),
    },
    'sampledObserverInvariants': {
        'rcsOff': all(not f['rcs'] for f in observer),
        'sasOff': all(not f['sas'] for f in observer),
        'zeroThrottle': all(f['throttle'] == 0 for f in observer),
        'zeroThrust': all(abs(f['thrust']) < 1 for f in observer),
        'speedbrakeActionGroupOff': all(not f['speedbrakeActionGroup'] for f in observer),
    },
}
Path(__file__).with_name('result.json').write_text(json.dumps(result, indent=2)+'\n')
for f in transitions:
    print(f['state']['phase'], f['ut'], f['motion']['trueAirSpeed'], f['guidance']['runwayAlongTrack'])
print('status', result['status']['state'])
