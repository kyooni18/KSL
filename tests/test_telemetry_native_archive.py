"""Native Swift KSP flight archives must be replayable without kRPC or KSP."""
import json
import math
from pathlib import Path

import pytest

from Tools import archive_native_swift_flight as native
from Tools.telemetry_web import list_ksp_runs, load_ksp_replay


def record(ut: float, phase: str, stopped: bool) -> dict:
    radius = 601_000.0 - ut
    return {
        'sequence': int(ut*10),
        'state': {
            'time': ut, 'position': {'x': radius, 'y': 0, 'z': 0},
            'velocity': {'x': -1, 'y': 150 if not stopped else 0, 'z': 0},
            'attitude': {'w': math.sqrt(0.5), 'x': 0, 'y': 0, 'z': math.sqrt(0.5)},
            'angularVelocity': {'x': 0, 'y': 0, 'z': 0},
            'aerodynamics': {'airspeed': 150 if not stopped else 0, 'mach': 0.5,
                             'angleOfAttack': math.radians(5), 'bank': 0,
                             'lift': 5000, 'drag': 100, 'dynamicPressure': 450},
            'atmosphere': {'pressure': 400, 'density': 0.1},
            'contact': 'stopped' if stopped else 'airborne',
            'mass': 22_000, 'radarAltitude': radius-600_000,
            'systems': {'gearDown': True, 'throttle': 0, 'brakes': stopped,
                        'thrust': 0, 'sas': False, 'airbrakes': False},
            'propulsion': {'availableThrust': 120_000}},
        'guidance': {
            'phase': phase, 'target': {'angleOfAttack': math.radians(5),
                                      'bank': 0, 'desiredFlightPath': 0,
                                      'gearDown': True},
            'violations': ([{'kind': 'runwayExcursion', 'message': 'Off runway'}]
                           if stopped else [])},
        'orbitalCommand': {'throttle': 0},
        'command': {'wheelBrake': int(stopped)},
    }


def test_native_archive_appears_in_ksp_replay_and_preserves_real_values(tmp_path, monkeypatch):
    root = tmp_path
    monkeypatch.setattr(native, 'ROOT', root)
    run = root / 'runs' / '20261101T1551KST-test-replay'
    run.mkdir(parents=True)
    plan = {'configuration': {'runway': {
        'name': 'KSC Runway 09', 'latitude': 0, 'longitude': 0,
        'heading': math.pi/2, 'elevation': 70, 'length': 2500, 'width': 70}}}
    (run/'live-sts-flight.jsonl.deorbit-plan.json').write_text(json.dumps(plan))
    with (run/'live-sts-flight.jsonl').open('w') as file:
        for step in range(24):
            phase = 'orbitalCoast' if step < 6 else 'entry' if step < 17 else 'touchdown'
            file.write(json.dumps(record(float(step), phase, step >= 18))+'\n')
    manifest = native.archive(run, root/'FlightLogs')
    assert manifest['frames'] >= 20
    assert manifest['firstStoppedUT'] == 18
    assert manifest['lastUT'] >= 20
    assert manifest['final']['vesselSituation'] == 'off-runway contact'

    runs = list_ksp_runs(root)
    assert len(runs) == 1 and runs[0]['runId'] == run.name
    assert runs[0]['hasReplay'] is True
    replay = load_ksp_replay(root, run.name, max_frames=100)
    assert replay is not None and len(replay['frames']) == manifest['frames']
    first = replay['frames'][0]
    last = replay['frames'][-1]
    assert first['telemetry']['meanAltitude'] == pytest.approx(1000)
    assert first['telemetry']['heading'] == pytest.approx(90)
    assert first['telemetry']['trueAirSpeed'] == 150
    assert first['guidanceState']['entryPlanValid'] is False
    assert last['phase'] == 'TOUCHDOWN'
    assert last['telemetry']['meanAltitude'] == pytest.approx(980)
    assert last['telemetry']['vesselSituation'] == 'off-runway contact'
    assert replay['run']['campaignIdentity'].startswith('ksp:swift-native:')
    with pytest.raises(FileExistsError):
        native.archive(run, root/'FlightLogs')


def test_raw_geodetic_and_attitude_are_not_neutralized():
    fields = native.native_keyframe(record(7.0, 'entry', False),
                                    {'latitude': 0.0, 'longitude': 0.0, 'runwayHeading': 90})
    assert fields['position']['latitude'] == pytest.approx(0)
    assert fields['position']['longitude'] == pytest.approx(0)
    assert fields['attitude']['pitch'] == pytest.approx(0)
    assert fields['attitude']['heading'] == pytest.approx(90)
    assert fields['motion']['horizontalSpeed'] == pytest.approx(150)
    assert fields['command']['targetAoA'] == pytest.approx(5)


def test_native_run_auto_indexes_and_replays_without_operator_export(tmp_path, monkeypatch):
    root = tmp_path
    monkeypatch.setattr(native, 'ROOT', root)
    run = root/'runs'/'20261101T1700KST-auto-index-entry'
    run.mkdir(parents=True)
    configuration = {'runway': {'name': 'KSC Runway 27',
        'latitude': 0.0, 'longitude': 0.0, 'heading': 3*math.pi/2,
        'length': 2500, 'width': 70, 'elevation': 70}}
    (run/'live-sts-flight.jsonl.configuration.json').write_text(json.dumps(configuration))
    with (run/'live-sts-flight.jsonl').open('w') as stream:
        for step in range(40):
            phase = 'entry' if step < 25 else 'touchdown'
            stream.write(json.dumps(record(float(step), phase, step >= 28))+'\n')

    # There are no FlightLogs files, only the default native journal.
    assert not (root/'FlightLogs').exists()
    catalog = list_ksp_runs(root)
    assert len(catalog) == 1
    assert catalog[0]['runId'] == run.name
    assert catalog[0]['hasReplay'] is True
    assert catalog[0]['terminalPhase'] == 'TOUCHDOWN'

    playback = load_ksp_replay(root, run.name, 100)
    assert playback is not None
    assert len(playback['frames']) > 20
    assert playback['run']['runId'] == run.name
    assert playback['frames'][-1]['telemetry']['vesselSituation'] == 'off-runway contact'
    assert (root/'FlightLogs'/f'{run.name}-vehicle.jsonl').exists()
    assert (root/'FlightLogs'/f'{run.name}-planner.jsonl').exists()
    assert (run/'telemetry-archive-manifest.json').exists()
    # This route is a read-only replay and must not rewrite source journal.
    assert len((run/'live-sts-flight.jsonl').read_text().splitlines()) == 40


def test_native_run_rejects_traversal_and_absent_source(tmp_path, monkeypatch):
    from Tools.telemetry_web import _native_swift_flight_source
    monkeypatch.setattr(native, 'ROOT', tmp_path)
    assert _native_swift_flight_source(tmp_path, '../outside') is None
    assert _native_swift_flight_source(tmp_path, 'not-present') is None
    assert load_ksp_replay(tmp_path, '../outside', 100) is None
