#!/usr/bin/env python3
"""Read-only KSL recorded-flight extraction; no controller or simulator imports.

Run from the repository root. Outputs only the designated research directory.
The support calculation reproduces the inspected TAEM expression, not an
independent force envelope. Observed lift is never treated as maximum lift.
"""
from pathlib import Path
import argparse
import collections
import csv
import hashlib
import json
import math
import subprocess


def sha(path):
    h = hashlib.sha256()
    with path.open('rb') as f:
        for b in iter(lambda: f.read(1024 * 1024), b''):
            h.update(b)
    return h.hexdigest()


def vec(d):
    return [d[k] for k in ('x', 'y', 'z')]


def dot(a, b):
    return sum(x*y for x, y in zip(a, b))


def norm(a):
    return math.sqrt(dot(a, a))


def cross(a, b):
    return [a[1]*b[2]-a[2]*b[1], a[2]*b[0]-a[0]*b[2], a[0]*b[1]-a[1]*b[0]]


def scaled(a, k):
    return [x*k for x in a]


def add(a, b):
    return [x+y for x, y in zip(a, b)]


def counters(rows, key):
    return dict(collections.Counter(str(r.get(key)) for r in rows))


def span(rows, key):
    xs = [r[key] for r in rows if r.get(key) is not None]
    return {'min': min(xs), 'max': max(xs)} if xs else None


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--run', default='runs/20261102T152517Z-dessert-150ms-live')
    p.add_argument('--output', default='Validation/TAEM-Replacement-Research-20261103')
    args = p.parse_args()
    root = Path.cwd()
    run = root / args.run
    out = root / args.output
    manifest = json.loads((run/'provenance.json').read_text())
    config = json.loads((run/'live.jsonl.configuration.json').read_text())
    log_stat = (run/'live.jsonl').stat()
    log_identity_before = (log_stat.st_size, log_stat.st_mtime_ns)
    planet, runway = config['planet'], config['runway']
    lat, lon, hdg = runway['latitude'], runway['longitude'], runway['heading']
    up0 = [math.cos(lat)*math.cos(lon), math.cos(lat)*math.sin(lon), math.sin(lat)]
    east = [-math.sin(lon), math.cos(lon), 0]
    north = cross(up0, east)
    along_axis = add(scaled(north, math.cos(hdg)), scaled(east, math.sin(hdg)))
    right_axis = cross(along_axis, up0)

    def reduce(row, line_no):
        s, g = row['state'], row['guidance']
        a, ctl = s['aerodynamics'], row.get('control', {})
        pos, vel = vec(s['position']), vec(s['velocity'])
        radius = norm(pos)
        up = scaled(pos, 1/radius)
        vs = dot(vel, up)
        vh = math.sqrt(max(0, dot(vel, vel)-vs*vs))
        gamma = math.atan2(vs, vh)
        tangent = add(up, scaled(up0, -max(-1, min(1, dot(up0, up)))))
        tlen = norm(tangent)
        angle = math.atan2(tlen, max(-1, min(1, dot(up0, up))))
        chart = scaled(tangent, planet['radius']*angle/tlen) if tlen > 1e-12 else [0, 0, 0]
        diagnostic = g.get('diagnostic', '')
        flags = {}
        for name in ['descentFits', 'descentCheck', 'referenceTracking', 'downmoded']:
            tag = name+'='
            flags[name] = diagnostic.split(tag, 1)[1].split()[0].rstrip(';') if tag in diagnostic else None
        return {
            'line': line_no, 'sequence': row.get('sequence'), 'ut': s['time'],
            'phase': g['phase'], 'asl_m': radius-planet['radius'],
            'runway_height_m': radius-planet['radius']-runway['elevation'],
            'along_m': dot(chart, along_axis), 'cross_right_m': dot(chart, right_axis),
            'airspeed_mps': a['airspeed'], 'groundspeed_mps': norm(vel), 'horizontal_mps': vh,
            'vertical_mps': vs, 'gamma_deg': math.degrees(gamma), 'q_pa': a['dynamicPressure'],
            'alpha_deg': math.degrees(a['angleOfAttack']), 'bank_deg': math.degrees(a['bank']),
            'target_alpha_deg': math.degrees(g['target']['angleOfAttack']),
            'target_bank_deg': math.degrees(g['target']['bank']),
            'measured_lift_mps2': a['lift']/s['mass'], 'measured_drag_mps2': a['drag']/s['mass'],
            'guidance_support_mps2': max(0, (planet['gravitationalParameter']/radius**2-vh**2/radius)*math.cos(gamma)),
            'measured_forces': a.get('measuredForces'), 'mass_kg': s['mass'],
            'rcs_observed': s['systems'].get('rcs'), 'throttle': s['systems'].get('throttle'),
            'thrust_n': s['systems'].get('thrust'),
            'roll_response_s': ctl.get('rollResponseSeconds'), 'pitch_response_s': ctl.get('pitchResponseSeconds'),
            'roll_saturated': ctl.get('rollSaturated'), 'pitch_saturated': ctl.get('pitchSaturated'),
            'trim_evidence': ctl.get('trimEvidence'),
            'part_count': s.get('vesselConfiguration', {}).get('partCount'), **flags,
        }

    taem = []
    phase_counts = collections.Counter()
    root_keys = collections.Counter()
    events = []
    dynamics_changes = []
    first = None
    before_first = None
    previous_state_row = None
    final_state = None
    total_lines = 0
    for line_no, text in enumerate((run/'live.jsonl').open(), 1):
        total_lines = line_no
        row = json.loads(text)
        root_keys.update(row.keys())
        if row.get('dynamicsChange'):
            dynamics_changes.append({'line': line_no, 'change': row['dynamicsChange']})
        if not isinstance(row.get('state'), dict):
            events.append({'line': line_no, 'keys': list(row), 'type': row.get('type'), 'event': row.get('event')})
            continue
        g = row.get('guidance', {})
        phase_counts[g.get('phase', 'no-guidance')] += 1
        final_state = {'line': line_no, 'sequence': row.get('sequence'), 'time': row['state']['time'], 'phase': g.get('phase'), 'contact': row['state'].get('contact')}
        if g.get('diagnostic', '').startswith('TAEM '):
            reduced = reduce(row, line_no)
            if first is None:
                first = row
                before_first = previous_state_row
            taem.append(reduced)
        previous_state_row = row
    if not taem:
        raise RuntimeError('No TAEM diagnostics found')
    t0 = taem[0]['ut']
    for r in taem:
        r['elapsed_s'] = r['ut']-t0
    first_minute = [r for r in taem if r['elapsed_s'] <= 60]
    checkpoints = []
    for elapsed in [0, 10, 30, 60]:
        r = next((r for r in taem if r['elapsed_s'] >= elapsed), None)
        if r:
            checkpoints.append({'requested_elapsed_s': elapsed, **r})
    files = ['provenance.json', 'source.patch', 'command.json', 'configuration.json',
             'live.jsonl.configuration.json', 'live.jsonl', 'ksl-landing'] + list(manifest['resources'])
    identities = {f: {'sha256': sha(run/f), 'bytes': (run/f).stat().st_size} for f in files}
    after = (run/'live.jsonl').stat()
    if (after.st_size, after.st_mtime_ns) != log_identity_before:
        raise RuntimeError('Recorded log changed during extraction; do not mix evidence epochs')
    resources = {f: identities[f]['sha256'] == digest for f, digest in manifest['resources'].items()}
    baseline = manifest['revision']
    backend_diff = subprocess.check_output(['git', 'diff', '--name-only', baseline, '--', 'swift/KSL-Landing']).decode().splitlines()
    untracked_sources = subprocess.check_output(['git', 'ls-files', '--others', '--exclude-standard', 'swift/KSL-Landing/Sources']).decode().splitlines()
    source_resources = {
        f: sha(root/'swift/KSL-Landing/Sources/KSLPhysics/Resources'/Path(f).name) == identities[f]['sha256']
        for f in manifest['resources'] if f.endswith(('.csv', '.json'))
    }
    csv_resource = run/'KSL-Landing_KSLPhysics.bundle/Contents/Resources/STSAerodynamics.csv'
    config_summary = {k: config.get(k) for k in ['runway', 'planet', 'vehicle', 'guidance', 'physicsArchive']}
    evidence = {
        'purpose': 'Deterministic recorded-data extraction only. No model/control/flight execution.',
        'run': args.run, 'source_revision': baseline, 'manifest': manifest,
        'identity': identities,
        'identity_checks': {
            'frozen_executable_matches_manifest': identities['ksl-landing']['sha256'] == manifest['executableSHA256'],
            'input_configuration_matches_manifest': identities['configuration.json']['sha256'] == manifest['configurationSHA256'],
            'resources_match_manifest': resources, 'source_patch_empty': identities['source.patch']['bytes'] == 0,
            'backend_diff_from_manifest_revision': backend_diff,
            'untracked_backend_source_files': untracked_sources,
            'frozen_resources_match_current_backend_resources': source_resources,
        },
        'effective_configuration': config_summary,
        'aero_table_header': csv_resource.open().readline().rstrip(),
        'total_log_lines': total_lines, 'state_guidance_phase_counts': dict(phase_counts),
        'top_level_key_occurrences': dict(root_keys),
        'non_state_event_count': len(events), 'non_state_event_examples': events[:12],
        'recorded_dynamics_changes': dynamics_changes,
        'taem_record_count': len(taem), 'taem_phase_counts': counters(taem, 'phase'),
        'taem_reference_tracking_counts': counters(taem, 'referenceTracking'),
        'taem_failure_counts': counters(taem, 'descentCheck'),
        'first_taem_diagnostic': first['guidance']['diagnostic'],
        'first_taem_violations': first['guidance'].get('violations'),
        'first_minute': {
            'definition': 'TAEM records with 0 <= UT - first_TAEM_UT <= 60 seconds; no resampling.',
            'count': len(first_minute), 'first_line': first_minute[0]['line'], 'last_line': first_minute[-1]['line'],
            'first_ut': t0, 'last_elapsed_s': first_minute[-1]['elapsed_s'],
            'max_gap_s': max(b['ut']-a['ut'] for a, b in zip(first_minute, first_minute[1:])),
            'target_bank_deg': span(first_minute, 'target_bank_deg'),
            'measured_bank_deg': span(first_minute, 'bank_deg'),
            'phase_counts': counters(first_minute, 'phase'),
            'reference_tracking_counts': counters(first_minute, 'referenceTracking'),
            'failure_counts': counters(first_minute, 'descentCheck'),
            'rcs_observed_counts': counters(first_minute, 'rcs_observed'),
            'measured_forces_counts': counters(first_minute, 'measured_forces'),
            'roll_saturated_counts': counters(first_minute, 'roll_saturated'),
            'pitch_saturated_counts': counters(first_minute, 'pitch_saturated'),
            'throttle': span(first_minute, 'throttle'), 'thrust_n': span(first_minute, 'thrust_n'),
            'roll_response_s': span(first_minute, 'roll_response_s'), 'pitch_response_s': span(first_minute, 'pitch_response_s'),
        },
        'first_nonzero_taem_target_bank': next((r for r in taem if r['target_bank_deg'] != 0), None),
        'first_rcs_off_taem': next((r for r in taem if r['rcs_observed'] is False), None),
        'checkpoints': checkpoints, 'last_state_record': final_state,
        'reconstruction_limits': [
            'A frozen executable hash corroborates the manifest; no independent bit-for-bit rebuild was performed.',
            'The manifest source.patch is empty; unchanged backend tracked files and absent untracked sources were checked.',
            'Full telemetry history is retained in the original log. A row contains diagnostics, not a serialized complete FlightControlSnapshot/limiter/estimator object.',
            'This extraction does not reconstruct the private online lift gain, maximum-lift bound, trim estimator or full allocator intermediate values.',
            'Observed lift is not maximum lift. Support is the current TAEM expression, not a universal requirement for descending turns.',
            'The initial effective configuration has no physicsArchive; passive/trajectory calibration exists and must be included in a future warmed replay.',
            'No alternative command or trajectory was evaluated; no reachability or flight-readiness claim follows.',
        ],
    }
    out.mkdir(parents=True, exist_ok=True)
    (out/'flight-evidence.json').write_text(json.dumps(evidence, indent=2)+'\n')
    (out/'flight-seed.json').write_text(json.dumps({'description': 'Recorded rows, not a complete private controller snapshot.', 'previous_record': before_first, 'first_taem_record': first}, indent=2)+'\n')
    with (out/'flight-first-minute.csv').open('w', newline='') as f:
        w = csv.DictWriter(f, fieldnames=list(first_minute[0]), lineterminator='\n')
        w.writeheader(); w.writerows(first_minute)
    print(json.dumps({'identity_checks': evidence['identity_checks'], 'first_minute': evidence['first_minute'],
                      'checkpoints': [{k:r[k] for k in ['line','ut','elapsed_s','asl_m','along_m','cross_right_m','airspeed_mps','q_pa','target_bank_deg','bank_deg','measured_lift_mps2','guidance_support_mps2']} for r in checkpoints],
                      'first_nonzero_target_elapsed_s': evidence['first_nonzero_taem_target_bank']['elapsed_s'] if evidence['first_nonzero_taem_target_bank'] else None,
                      'taem_record_count':len(taem), 'taem_phase_counts':evidence['taem_phase_counts'],
                      'outputs': [str(out/f) for f in ['flight-evidence.json','flight-seed.json','flight-first-minute.csv']]}, indent=2))


if __name__ == '__main__':
    main()
