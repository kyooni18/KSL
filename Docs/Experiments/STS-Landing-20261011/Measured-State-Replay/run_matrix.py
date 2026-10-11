#!/usr/bin/env python3
"""Execute the explicit KSL identification matrix and retain terminal states."""
import json
import os
from pathlib import Path
import subprocess
HERE=Path(__file__).resolve().parent
ROOT=HERE.parents[3]
results=[]
for flight in ('l5','l6','l7','high'):
    name=f'{flight}-measured-mm305';label=f'matrix-{name}'
    args=[str(ROOT/'bin/ksl'),'simulate',name,'--label',label,'--no-color']
    print('Starting',name,flush=True)
    proc=subprocess.run(args,cwd=ROOT,env=dict(os.environ,KSL_DATA_ROOT=str(ROOT)),text=True,stdout=subprocess.PIPE,stderr=subprocess.STDOUT,timeout=600)
    (HERE/f'{label}.log').write_text(proc.stdout)
    run_dir=Path(next(line.split('Run directory:',1)[1].strip() for line in proc.stdout.splitlines() if line.startswith('Run directory:')))
    status=json.loads((run_dir/'status.json').read_text())
    results.append({'scenario':name,'exitCode':proc.returncode,'args':args,'runDirectory':str(run_dir),'status':status})
    (HERE/'matrix.json').write_text(json.dumps(results,indent=2)+'\n')
    print('Finished',name,status['state'],flush=True)
