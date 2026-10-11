#!/usr/bin/env python3
"""Measured-inlet comparison, never live control; candidates remain unqualified."""
import gzip
import json
import os
from pathlib import Path
import subprocess
import sys
HERE=Path(__file__).resolve().parent
ROOT=HERE.parents[3]
MODELS={name:HERE.parent/'Body-Coordinate-Rate-Fix'/f'att-body-{name}.ini' for name in ('existing','l4','l7a')}
results=[]
for model,attitude in MODELS.items():
    for flight in (('high',) if '--high-only' in sys.argv else ('l5','l6','l7')):
        for phase in (('mm305',) if flight=='high' else ('mm305','final')):
            label=f'measured-{model}-{flight}-{phase}'
            config=(ROOT/'Configuration/sts-live.json' if flight=='l7' else
                    ROOT/'Docs/Experiments/STS-Landing-20261010/L5-72km/configuration.json')
            args=[sys.executable,str(ROOT/'ShuttleSim/scripts/run_guidance.py'),
                  '--scenario',str(HERE/f'{flight}-{phase}.ini'),'--configuration',str(config),
                  '--attitude',str(attitude),'--direct-control','--engage',
                  'engageReentry' if phase=='mm305' else 'engageFinalTest',
                  '--max-sim-time','1200','--skip-build','--no-mirror','--quiet-progress','--label',label]
            print('Starting',label,flush=True)
            with (HERE/f'{label}.log').open('w') as out:
                proc=subprocess.run(args,cwd=ROOT,env=dict(os.environ,KSP_LANDER_MODEL_PROFILE='sts-live-20261010',KSP_LANDER_ROOT=str(ROOT)),stdout=out,stderr=subprocess.STDOUT,timeout=600)
            lines=(HERE/f'{label}.log').read_text().splitlines()
            result=next(json.loads(line) for line in reversed(lines) if line.startswith('{"success"'))
            manifest=json.loads(Path(result['manifest']).read_text())
            with gzip.open(result['sim'],'rt') as stream: initial=json.loads(stream.readline())
            # End-to-end packet preservation check, before control starts.
            sources=json.loads((HERE/'sources.json').read_text())[flight]['states'][phase]['fixture']
            for field,key in [('body_pitch_deg','initial_body_pitch_deg'),('body_roll_deg','initial_body_roll_deg'),('body_heading_deg','initial_body_heading_deg'),
                              ('body_pitch_rate_deg_s','initial_body_pitch_rate_deg_s'),('body_roll_rate_deg_s','initial_body_roll_rate_deg_s'),('body_yaw_rate_deg_s','initial_body_yaw_rate_deg_s')]:
                assert abs(initial['attitude'][field]-sources[key])<1e-4,(label,field)
            assert abs(initial['attitude']['heading_deg']-sources['heading_deg'])<1e-3
            results.append({'label':label,'exitCode':proc.returncode,'args':args,'initialPacket':initial,
                            'result':result,'provenance':manifest['provenance']})
            (HERE/('high-result.json' if '--high-only' in sys.argv else 'result.json')).write_text(json.dumps(results,indent=2)+'\n')
            print('Finished',label,'exit',proc.returncode,flush=True)
