#!/usr/bin/env python3
"""Inventory existing native recorded response support, not flight simulations.

Anchor/part count is only a partial craft identity. It is used to find leads,
not to certify compatibility. All real aerodynamic samples remain unchanged.
"""
import argparse
import gzip
import hashlib
import json
import lzma
import math
from pathlib import Path


def inspect(path, seed):
    opener = lzma.open if path.suffix == '.xz' else gzip.open if path.suffix == '.gz' else open
    digest = hashlib.sha256()
    result = {'path': str(path), 'records': 0, 'RCSOffAirborneRecords': 0,
              'partialIdentityRecords': 0, 'seedMachOrHigherRecords': 0,
              'seedMachOrHigherRCSOffRecords': 0, 'maximumRCSOffMach': None,
              'recordingIssues': [], 'lead': None}
    best = math.inf
    with opener(path, 'rb') as stream:
        for line in stream:
            digest.update(line)
            try:
                r = json.loads(line)
            except ValueError:
                result['recordingIssues'].append('malformed JSON'); continue
            if not isinstance(r, dict) or not isinstance(r.get('state'), dict) or 'guidance' not in r:
                continue
            result['records'] += 1
            s = r['state']; a = s.get('aerodynamics', {})
            authority = s.get('controlAuthority')
            if r.get('orbitalCommand') or s.get('contact') != 'airborne' or not s.get('appliedControls') or not authority:
                continue
            if not s.get('systems', {}).get('rcs', True):
                result['RCSOffAirborneRecords'] += 1
                result['maximumRCSOffMach'] = max(result['maximumRCSOffMach'] or 0, a['mach'])
            identity = s.get('vesselConfiguration', {})
            if identity.get('anchorPart') != seed['vesselConfiguration']['anchorPart'] or identity.get('partCount') != seed['vesselConfiguration']['partCount']:
                continue
            result['partialIdentityRecords'] += 1
            if a['mach'] >= seed['aerodynamics']['mach']:
                result['seedMachOrHigherRecords'] += 1
                if not s['systems']['rcs']:
                    result['seedMachOrHigherRCSOffRecords'] += 1
            if s['systems']['rcs']:
                continue
            # Query distance only: relative dimensionless offsets, not a
            # trajectory objective, acceptance tolerance, or physical scaling.
            distance = sum(((a[k]-seed['aerodynamics'][k])/seed['aerodynamics'][k])**2
                           for k in ('mach', 'angleOfAttack', 'dynamicPressure'))
            if distance < best:
                best = distance
                result['lead'] = {'time': s['time'], 'sequence': r['sequence'], 'phase': r['guidance']['phase'],
                    'mach': a['mach'], 'q': a['dynamicPressure'], 'alpha': a['angleOfAttack'],
                    'bank': a['bank'], 'mass': s['mass'], 'inertia': authority['inertia'],
                    'vesselConfiguration': identity, 'queryDistance': distance}
    result['decompressedSHA256'] = digest.hexdigest()
    return result


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--root', type=Path, default=Path('runs'))
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    seed = json.loads(Path('Validation/TAEM-Replacement-Research-20261103/flight-seed.json').read_text())['first_taem_record']['state']
    stems = {'live.jsonl', 'live-sts-flight.jsonl', 'swift-live.jsonl', 'flight.jsonl', 'orbit-live.jsonl'}
    paths = sorted(p for p in args.root.rglob('*') if p.is_file() and
                   (p.name in stems or (p.suffix in {'.xz', '.gz'} and p.stem in stems)))
    results = []
    for i, path in enumerate(paths):
        result = inspect(path, seed)
        results.append(result)
        if (i+1) % 10 == 0:
            print(f'inspected {i+1}/{len(paths)} recorded sources', flush=True)
    args.output.write_text(json.dumps({'evidence': 'existing-recorded-native-response-source-inventory',
      'seedTime': seed['time'], 'sources': results,
      'limitations': ['Read-only inventory, not a synthetic test or flight campaign.',
        'Partial identity is a lead only; matching snapshot anchor/part count does not prove unchanged control allocation or model.',
        'Different control modes and genuine mass/inertia states must be retained, not scaled to manufacture compatibility.',
        'Hashes cover decompressed source bytes; identical hashes are duplicate evidence, not independent flights.']}, indent=2, sort_keys=True)+'\n')
    print('completed', len(paths), 'sources; partial-identity high-Mach RCS-off leads:',
          [(r['path'], r['seedMachOrHigherRCSOffRecords']) for r in results if r['seedMachOrHigherRCSOffRecords']], flush=True)
