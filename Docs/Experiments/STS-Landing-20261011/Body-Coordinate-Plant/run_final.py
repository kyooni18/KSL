#!/usr/bin/env python3
"""Isolate recorded L5 Final from the unqualified banked TAEM plant."""
import json
import os
from pathlib import Path
import subprocess
import sys
HERE=Path(__file__).resolve().parent
ROOT=HERE.parents[3]
results=[]
for model in ('legacy','existing','l4','l7a'):
    label=f'body-final-{model}-l5'
    attitude=(ROOT/'Docs/Experiments/STS-Landing-20261010/Codex-Flare-Offline/inputs/att-live.ini'
              if model=='legacy' else HERE/f'att-body-{model}.ini')
    args=[sys.executable,str(ROOT/'ShuttleSim/scripts/run_guidance.py'),
          '--scenario',str(HERE/'l5-live-final.ini'),
          '--configuration',str(ROOT/'Docs/Experiments/STS-Landing-20261010/L5-72km/configuration.json'),
          '--attitude',str(attitude),'--direct-control','--engage','engageFinalTest',
          '--max-sim-time','600','--skip-build','--no-mirror','--quiet-progress','--label',label]
    print('Starting',label,flush=True)
    with (HERE/f'{label}.log').open('w') as out:
        proc=subprocess.run(args,cwd=ROOT,env=dict(os.environ,KSP_LANDER_MODEL_PROFILE='sts-live-20261010',KSP_LANDER_ROOT=str(ROOT)),stdout=out,stderr=subprocess.STDOUT,timeout=600)
    results.append({'label':label,'exitCode':proc.returncode,'args':args})
    (HERE/'final-runs.json').write_text(json.dumps(results,indent=2)+'\n')
    print('Finished',label,'exit',proc.returncode,flush=True)
