#!/usr/bin/env python3
"""Retain simulator contact summaries separately from backend acceptance."""
import json
from pathlib import Path
HERE=Path(__file__).resolve().parent
output=[]
for row in json.loads((HERE/'runs.json').read_text())+json.loads((HERE/'final-runs.json').read_text()):
    lines=(HERE/(row['label']+'.log')).read_text().splitlines()
    result=next(json.loads(line) for line in reversed(lines) if line.startswith('{"success"'))
    manifest=json.loads(Path(result['manifest']).read_text())
    output.append({**row,'result':result,'provenance':manifest['provenance'],
                   'physicsSources':manifest['physicsSources'],
                   'guidanceModelSources':manifest['guidanceModelSources']})
(HERE/'result.json').write_text(json.dumps(output,indent=2)+'\n')
