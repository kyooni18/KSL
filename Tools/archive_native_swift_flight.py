#!/usr/bin/env python3
"""Archive an observed native Swift KSL flight for the existing KSP REPLAY UI.

Reads native JSONL without touching kRPC or the running flight controller.
Produces compact keyframes (true measured states only) and phase/target records
in FlightLogs' existing session/vehicle/planner format, so the live web server
can list and replay the run without any server-side compatibility guessing.
"""
from __future__ import annotations

import argparse
import datetime as dt
import json
import math
import os
from pathlib import Path
import re
import tempfile
from typing import Any

ROOT = Path(__file__).resolve().parents[1]
RADIUS = 600_000.0
PHASES = {
    'orbitalCoast': 'ORBITAL COAST', 'burnSetup': 'BURN SETUP',
    'deorbitBurn': 'DEORBIT BURN', 'entryCoast': 'ENTRY COAST',
    'entry': 'MM304 ENTRY', 'acquisition': 'MM305 TAEM',
    'headingAlignment': 'MM305 HAC', 'approach': 'FINAL APPROACH',
    'preflare': 'PREFLARE', 'flare': 'FLARE', 'touchdown': 'TOUCHDOWN',
    'rollout': 'ROLLOUT', 'complete': 'COMPLETE', 'abort': 'ABORT',
}


def scalar(value: Any, default: float = 0.0) -> float:
    try:
        result = float(value)
        return result if math.isfinite(result) else default
    except (TypeError, ValueError, OverflowError):
        return default


def dot(a: list[float], b: list[float]) -> float:
    return sum(x * y for x, y in zip(a, b))


def cross(a: list[float], b: list[float]) -> list[float]:
    return [a[1]*b[2]-a[2]*b[1], a[2]*b[0]-a[0]*b[2], a[0]*b[1]-a[1]*b[0]]


def vector(v: dict[str, Any]) -> list[float]:
    return [scalar(v.get(k)) for k in ('x', 'y', 'z')]


def rotation(q: dict[str, Any], basis: list[float]) -> list[float]:
    axes = [scalar(q.get(k)) for k in ('x', 'y', 'z')]
    t = [2*x for x in cross(axes, basis)]
    c = cross(axes, t)
    w = scalar(q.get('w'), 1)
    return [basis[i] + w*t[i] + c[i] for i in range(3)]


def degrees(value: Any) -> float | None:
    try:
        numeric = float(value)
        return math.degrees(numeric) if math.isfinite(numeric) else None
    except (TypeError, ValueError):
        return None


def geodetic(position: list[float]) -> tuple[float, float, float, list[float], list[float], list[float]]:
    radius = math.sqrt(dot(position, position))
    if not math.isfinite(radius) or radius <= 0:
        raise ValueError('Invalid native position vector')
    up = [v/radius for v in position]
    longitude = math.atan2(position[1], position[0])
    latitude = math.asin(max(-1.0, min(1.0, up[2])))
    east = [-math.sin(longitude), math.cos(longitude), 0.0]
    north = cross(up, east)
    return math.degrees(latitude), math.degrees(longitude), radius - RADIUS, up, north, east


def runway_geometry(latitude: float, longitude: float, site: dict[str, Any]) -> dict[str, float]:
    if not site:
        return {}
    slat = math.radians(scalar(site.get('latitude')))
    slon = math.radians(scalar(site.get('longitude')))
    lat = math.radians(latitude)
    dlon = math.radians(longitude) - slon
    a = math.sin((lat-slat)/2)**2 + math.cos(slat)*math.cos(lat)*math.sin(dlon/2)**2
    distance = 2*RADIUS*math.asin(min(1.0, math.sqrt(max(0.0, a))))
    y = math.sin(dlon)*math.cos(lat)
    x = math.cos(slat)*math.sin(lat)-math.sin(slat)*math.cos(lat)*math.cos(dlon)
    bearing = math.atan2(y, x)
    heading = math.radians(scalar(site.get('runwayHeading')))
    delta = bearing-heading
    return {'rangeToSite': distance, 'bearingToSite': math.degrees(bearing)%360,
            'runwayAlongTrack': distance*math.cos(delta),
            'runwayCrossTrack': distance*math.sin(delta)}


def native_keyframe(record: dict[str, Any], site: dict[str, Any]) -> dict[str, Any]:
    state = record['state']
    aero = state.get('aerodynamics') or {}
    atmosphere = state.get('atmosphere') or {}
    systems = state.get('systems') or {}
    guidance = record.get('guidance') or {}
    target = guidance.get('target') or {}
    velocity = vector(state.get('velocity') or {})
    latitude, longitude, altitude, up, north, east = geodetic(vector(state['position']))
    vertical = dot(velocity, up)
    horizontal = math.sqrt(max(0, dot(velocity, velocity) - vertical*vertical))
    airspeed = scalar(aero.get('airspeed'))
    fpa = math.degrees(math.atan2(vertical, horizontal)) if horizontal or vertical else 0.0
    q = state.get('attitude') or {}
    forward = rotation(q, [1, 0, 0])
    right = rotation(q, [0, 1, 0])
    roof = rotation(q, [0, 0, -1])
    heading = math.degrees(math.atan2(dot(forward, east), dot(forward, north)))%360
    pitch = math.degrees(math.asin(max(-1.0, min(1.0, dot(forward, up)))))
    roll = -math.degrees(math.atan2(dot(right, up), dot(roof, up)))
    situation = str(state.get('contact') or 'airborne')
    violated = guidance.get('violations') or []
    off_runway = any(v.get('kind') == 'runwayExcursion' for v in violated if isinstance(v, dict))
    phase = PHASES.get(str(guidance.get('phase')), str(guidance.get('phase') or 'UNKNOWN').upper())
    gm = runway_geometry(latitude, longitude, site)
    return {
        'state': {'phase': phase, 'controlProfile': 'native Swift',
                  'vesselSituation': 'off-runway contact' if off_runway else situation,
                  'automation': True, 'paused': False,
                  'gear': bool(systems.get('gearDown')), 'brakes': bool(systems.get('brakes')),
                  'airbrakes': bool(systems.get('airbrakes')),
                  'sasEnabled': bool(systems.get('sas'))},
        'position': {'latitude': latitude, 'longitude': longitude,
                     'altitude': altitude, 'radarAltitude': state.get('radarAltitude')},
        'motion': {'trueAirSpeed': airspeed, 'horizontalSpeed': horizontal,
                   'verticalSpeed': vertical, 'flightPathAngle': fpa,
                   'groundTrackHeading': math.degrees(math.atan2(dot(velocity, east), dot(velocity, north)))%360},
        'attitude': {'pitch': pitch, 'heading': heading, 'roll': roll,
                     'angleOfAttack': degrees(aero.get('angleOfAttack')),
                     'sideslip': degrees(aero.get('sideslip')),
                     'pitchRate': degrees((state.get('angularVelocity') or {}).get('y')),
                     'rollRate': degrees((state.get('angularVelocity') or {}).get('x')),
                     'yawRate': degrees((state.get('angularVelocity') or {}).get('z'))},
        'aero': {'dynamicPressure': aero.get('dynamicPressure'), 'staticPressure': atmosphere.get('pressure'),
                 'density': atmosphere.get('density'), 'mach': aero.get('mach'),
                 'liftForce': aero.get('lift'), 'dragForce': aero.get('drag'),
                 'gForce': abs(scalar(aero.get('lift')))/max(1.0, scalar(state.get('mass'))*9.80665)},
        'vehicle': {'mass': state.get('mass'), 'availableThrust': (state.get('propulsion') or {}).get('availableThrust'),
                    'currentThrust': systems.get('thrust')},
        'guidance': gm,
        'command': {'targetPitch': (degrees(target.get('angleOfAttack')) or 0) +
                                  (degrees(target.get('desiredFlightPath')) or 0),
                    'targetHeading': None, 'targetRoll': degrees(target.get('bank')),
                    'targetAoA': degrees(target.get('angleOfAttack')),
                    'targetThrottle': (record.get('orbitalCommand') or {}).get('throttle'),
                    'gear': bool(target.get('gearDown')),
                    'brakes': scalar((record.get('command') or {}).get('wheelBrake')) > 0.5},
    }


def started_at(run: Path) -> str:
    match = re.match(r'^(\d{8})T(\d{4})KST', run.name)
    if match:
        date = dt.datetime.strptime(match.group(1)+match.group(2), '%Y%m%d%H%M')
        return date.replace(tzinfo=dt.timezone(dt.timedelta(hours=9))).astimezone(dt.timezone.utc).isoformat()
    return dt.datetime.fromtimestamp(run.stat().st_mtime, tz=dt.timezone.utc).isoformat()


def site_from_plan(run: Path) -> dict[str, Any]:
    plans = sorted(run.glob('live*.jsonl.deorbit-plan.json'))
    configurations = sorted(run.glob('live*.jsonl.configuration.json'))
    # A direct-entry `fly` has no orbital deorbit plan. Reuse its actual
    # recorded selected-runway configuration instead of inventing one.
    source = plans[0] if plans else (configurations[0] if configurations else None)
    if source is None:
        raise ValueError('No native runway configuration is available')
    data = json.loads(source.read_text())
    runway = (data.get('configuration') or data).get('runway') or {}
    site = {'name': runway.get('name') or 'KSC Runway',
            'latitude': degrees(runway.get('latitude')),
            'longitude': degrees(runway.get('longitude')),
            'runwayHeading': degrees(runway.get('heading')),
            'altitude': runway.get('elevation'),
            'runwayLength': runway.get('length'),
            'runwayWidth': runway.get('width')}
    if site['latitude'] is None or site['longitude'] is None or site['runwayHeading'] is None:
        raise ValueError('Native deorbit plan has no valid runway geometry')
    return site


def archive(run: Path, output: Path, interval: float = 1.0, *, overwrite: bool = False) -> dict[str, Any]:
    run = run.resolve()
    output = output.resolve()
    if not run.is_dir() or not run.is_relative_to((ROOT/'runs').resolve()):
        raise ValueError('Run must exist inside the local KSL runs directory')
    if not 0.25 <= interval <= 30:
        raise ValueError('Sample interval must be 0.25..30 seconds')
    run_id = run.name
    source_candidates = sorted(run.glob('live*.jsonl'))
    if not source_candidates:
        raise ValueError('Native flight JSONL missing')
    source = next((p for p in source_candidates if 'sts-flight' in p.name), source_candidates[0])
    site = site_from_plan(run)
    output.mkdir(parents=True, exist_ok=True)
    vehicle = output/f'{run_id}-vehicle.jsonl'
    planner = output/f'{run_id}-planner.jsonl'
    if (vehicle.exists() or planner.exists()) and not overwrite:
        raise FileExistsError('Archive already exists; refusing to overwrite previously published replay')
    stamp = started_at(run)
    session = {'recordType': 'sessionStart', 'schemaVersion': 1,
               'startedUTC': stamp, 'sessionId': f'native-swift-{run_id}',
               'campaignIdentity': f'ksp:swift-native:{run_id}',
               'vessel': 'STS', 'rotationIndex': 0,
               'configuration': {'site': site},
               'nativeSource': str(source.relative_to(ROOT))}
    meta = {'source': str(source.relative_to(ROOT)), 'runId': run_id,
            'site': site, 'intervalSeconds': interval}
    source_size = source.stat().st_size
    first_ut: float | None = None
    last_ut: float | None = None
    first_stopped_at: float | None = None
    last_phase: str | None = None
    count = 0
    native_count = 0
    final_fields: dict[str, Any] = {}
    vf = tempfile.NamedTemporaryFile('w', encoding='utf-8', dir=output, prefix='archive-vehicle-', delete=False)
    pf = tempfile.NamedTemporaryFile('w', encoding='utf-8', dir=output, prefix='archive-planner-', delete=False)
    try:
        session_vehicle = {**session, 'stream': 'vehicle', 'peer': {'path': str(planner), 'stream': 'planner'}}
        session_planner = {**session, 'stream': 'planner', 'peer': {'path': str(vehicle), 'stream': 'vehicle'}}
        vf.write(json.dumps(session_vehicle, separators=(',', ':'))+'\n')
        pf.write(json.dumps(session_planner, separators=(',', ':'))+'\n')
        with source.open('rb') as native:
            while native.tell() < source_size:
                raw = native.readline()
                if not raw or native.tell() > source_size:
                    break
                try:
                    record = json.loads(raw)
                except (json.JSONDecodeError, UnicodeDecodeError):
                    continue
                state = record.get('state')
                guidance = record.get('guidance')
                if not isinstance(state, dict) or not isinstance(guidance, dict):
                    continue
                ut = scalar(state.get('time'), float('nan'))
                if not math.isfinite(ut):
                    continue
                native_count += 1
                phase = str(guidance.get('phase') or 'unknown')
                if first_ut is None:
                    first_ut = ut
                if first_stopped_at is None and phase in ('touchdown','rollout','complete') and state.get('contact') == 'stopped':
                    first_stopped_at = ut
                changed = phase != last_phase
                if last_ut is not None and ut < last_ut - 1e-6:
                    continue
                if last_ut is not None and ut - last_ut < interval and not changed and first_stopped_at != ut:
                    if first_stopped_at is not None and ut-first_stopped_at > 3:
                        break
                    continue
                fields = native_keyframe(record, site)
                vf.write(json.dumps({'recordType': 'vehicleKeyframe', 'sessionId': session['sessionId'],
                                     'tickSequence': record.get('sequence'), 'ut': ut,
                                     'fields': fields}, separators=(',', ':'), allow_nan=False)+'\n')
                target = guidance.get('target') or {}
                entry = guidance.get('entry') or {}
                capture = guidance.get('capture') or {}
                reasons = [str(v.get('message')) for v in guidance.get('violations') or []
                           if isinstance(v, dict) and v.get('message')]
                current_plan = {'valid': False,  # A logged command is not a certified future trajectory.
                                'targetAoA': degrees(target.get('angleOfAttack')),
                                'targetBank': degrees(target.get('bank')),
                                'terminalReady': capture.get('mm305PlanAccepted') is True,
                                'taemRangeError': entry.get('rangeError'),
                                'plannedUT': ut,
                                'limitingReason': '; '.join(reasons) or None}
                pf.write(json.dumps({'recordType': 'plannerSample', 'sessionId': session['sessionId'],
                                     'tickSequence': record.get('sequence'), 'ut': ut,
                                     'phase': fields['state']['phase'], 'currentPlan': current_plan,
                                     'terminalGeometry': {}, 'trajectoryIncluded': False},
                                    separators=(',', ':'), allow_nan=False)+'\n')
                count += 1
                last_ut = ut
                last_phase = phase
                final_fields = fields
                if first_stopped_at is not None and ut-first_stopped_at >= 2:
                    break
        if count < 10 or first_ut is None or last_ut is None:
            raise ValueError(f'Insufficient native records for replay: {count}')
        vf.flush();os.fsync(vf.fileno());pf.flush();os.fsync(pf.fileno())
        vf.close(); pf.close()
        # The catalog keys on planner sessionStart. Publish the vehicle first
        # and planner last so partially archived runs never appear selectable.
        os.replace(vf.name, vehicle)
        os.replace(pf.name, planner)
    finally:
        if not vf.closed: vf.close()
        if not pf.closed: pf.close()
        for temp in (vf.name, pf.name):
            if Path(temp).exists():Path(temp).unlink()
    meta.update({'sourceBytes': source_size, 'frames': count, 'sourceRecordsRead': native_count,
                 'firstUT': first_ut, 'lastUT': last_ut, 'firstStoppedUT': first_stopped_at,
                 'lastPhase': last_phase,
                 'final': {'altitude': final_fields.get('position', {}).get('altitude'),
                           'latitude': final_fields.get('position', {}).get('latitude'),
                           'longitude': final_fields.get('position', {}).get('longitude'),
                           'runwayAlongTrack': final_fields.get('guidance', {}).get('runwayAlongTrack'),
                           'runwayCrossTrack': final_fields.get('guidance', {}).get('runwayCrossTrack'),
                           'vesselSituation': final_fields.get('state', {}).get('vesselSituation')},
                 'vehiclePath': str(vehicle), 'plannerPath': str(planner)})
    (run/'telemetry-archive-manifest.json').write_text(json.dumps(meta, indent=2, ensure_ascii=False)+'\n')
    return meta


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--run', type=Path, required=True)
    parser.add_argument('--out', type=Path, default=ROOT/'FlightLogs')
    parser.add_argument('--interval', type=float, default=1.0)
    args = parser.parse_args()
    print(json.dumps(archive(args.run, args.out, args.interval), indent=2, ensure_ascii=False))


if __name__ == '__main__':
    main()
