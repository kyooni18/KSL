"""Read KSP ConfigNode files without interpreting document text as commands."""
import argparse
import hashlib
import json
from pathlib import Path


def parse(text):
    root = {'name': 'ROOT', 'values': {}, 'nodes': []}
    stack = [root]
    pending = None
    for raw in text.splitlines():
        line = raw.split('//', 1)[0].strip()
        if not line:
            continue
        if line == '{':
            if pending is None:
                raise ValueError('Unnamed ConfigNode')
            node = {'name': pending, 'values': {}, 'nodes': []}
            stack[-1]['nodes'].append(node)
            stack.append(node)
            pending = None
        elif line == '}':
            if len(stack) == 1:
                raise ValueError('Unexpected closing brace')
            stack.pop()
        elif '=' in line:
            key, value = line.split('=', 1)
            stack[-1]['values'].setdefault(key.strip(), []).append(value.strip())
        else:
            pending = line
    if len(stack) != 1:
        raise ValueError('Unclosed ConfigNode')
    return root


def manifest(path, meta):
    data = Path(path).read_bytes()
    root = parse(data.decode('utf-8-sig'))
    parts = [n for n in root['nodes'] if n['name'] == 'PART']
    metadata = parse(Path(meta).read_text())['values'] if meta else {}
    engines = []
    resources = {}
    for node in parts:
        values = node['values']
        name = values['part'][0].rsplit('_', 1)[0]
        if name in ('LiquidEngineRE-I2', 'LiquidEngineLV-TX87', 'LiquidEngineLV-T91',
                    'engineLargeSkipper_v2', 'engineLargeSkipper', 'Pollux'):
            engines.append({'name': name, 'id': values['part'][0],
                            'position': [float(v) for v in values['pos'][0].split(',')],
                            'stage': int(values['istg'][0])})
        for child in node['nodes']:
            if child['name'] == 'RESOURCE':
                v = child['values']
                resource = v['name'][0]
                resources[resource] = resources.get(resource, 0)+float(v['amount'][0])
    return {'ship': root['values']['ship'][0], 'parts': len(parts),
            'sha256': hashlib.sha256(data).hexdigest(),
            'editor_mass_t': float(metadata['totalMass'][0]) if metadata else None,
            'engines': engines, 'resources': resources}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('craft')
    parser.add_argument('--meta')
    args = parser.parse_args()
    print(json.dumps(manifest(args.craft, args.meta), indent=2))


if __name__ == '__main__':
    main()
