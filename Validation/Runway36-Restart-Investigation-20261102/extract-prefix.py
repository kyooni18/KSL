#!/usr/bin/env python3
"""Recreate the exact recorded entry-to-first-TAEM prefix; no KSP interaction."""
import hashlib
import json
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
OUT = Path(__file__).resolve().parent
RUN = ROOT / 'runs/20261102T122809Z-dessert-normal-oms-restart'
SOURCE = RUN / 'live.jsonl'

def sha(path):
    h = hashlib.sha256()
    with path.open('rb') as f:
        for chunk in iter(lambda: f.read(1024*1024), b''):
            h.update(chunk)
    return h.hexdigest()

count = 0
with SOURCE.open() as src, (OUT/'entry-through-handoff.jsonl').open('w') as dest:
    for line in src:
        record = json.loads(line)
        phase = record.get('guidance', {}).get('phase')
        if phase not in ('entry', 'acquisition'):
            continue
        dest.write(line)
        count += 1
        if phase == 'acquisition':
            break
assert count == 4240, count
print(json.dumps({'records': count, 'source': str(SOURCE.relative_to(ROOT)),
                  'sourceSHA256': sha(SOURCE),
                  'prefixSHA256': sha(OUT/'entry-through-handoff.jsonl')}, indent=2))
