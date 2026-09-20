#!/usr/bin/env python3
"""Clone a KSP checkpoint with propulsion disabled, preserving flight physics state."""
from __future__ import annotations
import argparse
import hashlib
import json
from pathlib import Path

ENGINE_FIELDS = {
    'independentThrottle': 'False',
    'independentThrottlePercentage': '0',
    'EngineIgnited': 'False',
    'engineShutdown': 'True',
    'currentThrottle': '0',
    'thrustPercentage': '0',
}

def disable_propulsion(text: str, vessel_name: str = 'STS-N') -> tuple[str, list[dict]]:
    stack: list[dict[str, str]] = []
    pending = ''
    changes: list[dict] = []
    output: list[str] = []
    engine_count = 0
    control_count = 0
    for number, raw in enumerate(text.splitlines(keepends=True), 1):
        line = raw.strip()
        if line == '{':
            stack.append({'type': pending})
            pending = ''
        elif line == '}':
            if not stack:
                raise ValueError(f'Unbalanced closing brace on line {number}')
            stack.pop()
        elif '=' in line and stack:
            key, value = (part.strip() for part in line.split('=', 1))
            node = stack[-1]
            if key == 'name':
                node['name'] = value
            selected = any(n['type'] == 'VESSEL' and n.get('name') == vessel_name for n in stack)
            replacement = None
            if selected:
                if node['type'] == 'MODULE' and node.get('name', '').startswith('ModuleEngines'):
                    if key == 'name':
                        engine_count += 1
                    replacement = ENGINE_FIELDS.get(key)
                elif node['type'] == 'CTRLSTATE' and key == 'mainThrottle':
                    control_count += 1
                    replacement = '0'
                elif node['type'] == 'ACTIONGROUPS' and key in ('SAS', 'RCS'):
                    replacement = 'False' + (',' + value.split(',', 1)[1] if ',' in value else '')
            if replacement is not None and replacement != value:
                lhs, rhs = raw.split('=', 1)
                whitespace = rhs[:len(rhs)-len(rhs.lstrip())]
                ending = '\r\n' if raw.endswith('\r\n') else '\n' if raw.endswith('\n') else ''
                raw = lhs + '=' + whitespace + replacement + ending
                changes.append({'line': number, 'node': node['type'], 'field': key, 'old': value, 'new': replacement})
        elif line and not line.startswith('//'):
            pending = line
        output.append(raw)
    if stack:
        raise ValueError('Unclosed KSP config node')
    if engine_count == 0 or control_count != 1:
        raise ValueError(f'Expected engines and one control state for {vessel_name}; got {engine_count}, {control_count}')
    return ''.join(output), changes

def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('source', type=Path)
    parser.add_argument('destination', type=Path)
    parser.add_argument('--vessel', default='STS-N')
    args = parser.parse_args()
    if args.source.resolve() == args.destination.resolve():
        parser.error('Use a new checkpoint name; the source must be preserved.')
    if args.destination.exists():
        parser.error('Destination already exists; use a fresh checkpoint name.')
    original = args.source.read_text()
    modified, changes = disable_propulsion(original, args.vessel)
    again, more = disable_propulsion(modified, args.vessel)
    if again != modified or more:
        raise RuntimeError('Propulsion changes are not idempotent')
    args.destination.write_text(modified)
    manifest = {'source': str(args.source), 'destination': str(args.destination),
        'sourceSHA256': hashlib.sha256(original.encode()).hexdigest(),
        'destinationSHA256': hashlib.sha256(modified.encode()).hexdigest(),
        'vessel': args.vessel, 'changes': changes,
        'preserved': 'All non-propulsion fields, including position, velocity, orbit, mass/resources, attitude, and control surfaces.'}
    args.destination.with_suffix('.unpowered.json').write_text(json.dumps(manifest, indent=2)+'\n')
    print(json.dumps(manifest, indent=2))

if __name__ == '__main__':
    main()
