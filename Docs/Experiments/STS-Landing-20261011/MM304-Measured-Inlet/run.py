#!/usr/bin/env python3
"""Compare MM304 inlet geometry and body-coordinate plant; never live control."""
import gzip,json,os,subprocess,sys
from pathlib import Path
HERE=Path(__file__).resolve().parent
ROOT=HERE.parents[3]
config=HERE/'configuration.json'
if not config.exists():config.write_text((ROOT/'Configuration/sts-live.json').read_text())
models={'legacy':ROOT/'Docs/Experiments/STS-Landing-20261010/Codex-Flare-Offline/inputs/att-live.ini',
        'body':HERE.parent/'Body-Coordinate-Rate-Fix/att-body-existing.ini'}
results=[]
for flight in ('l5','l7'):
 for model,attitude in models.items():
  label=f'entry-measured-{flight}-{model}'
  args=[sys.executable,str(ROOT/'ShuttleSim/scripts/run_guidance.py'),'--scenario',str(HERE/f'{flight}-mm304.ini'),
        '--configuration',str(config),'--attitude',str(attitude),'--direct-control','--engage','engageReentry',
        '--stop-at-phase','TAEM','--max-sim-time','900','--skip-build','--no-mirror','--quiet-progress','--label',label]
  print('Starting',label,flush=True)
  with (HERE/f'{label}.log').open('w') as out:
   p=subprocess.run(args,cwd=ROOT,env=dict(os.environ,KSP_LANDER_MODEL_PROFILE='sts-live-20261010',KSP_LANDER_ENTRY_TRACE='1',KSP_LANDER_ROOT=str(ROOT)),stdout=out,stderr=subprocess.STDOUT,timeout=600)
  lines=(HERE/f'{label}.log').read_text().splitlines()
  r=next(json.loads(x) for x in reversed(lines) if x.startswith('{"success"'))
  manifest=json.loads(Path(r['manifest']).read_text())
  with gzip.open(r['guidance'],'rt') as stream:
   snapshots=[json.loads(line) for line in stream]
  last=next(x['snapshot'] for x in reversed(snapshots) if x.get('type')=='snapshot')
  r['terminalSnapshot']={k:last.get(k) for k in ('phase','telemetry','statusMessage')}
  results.append(dict(label=label,args=args,exitCode=p.returncode,result=r,provenance=manifest['provenance']))
  (HERE/'result.json').write_text(json.dumps(results,indent=2)+'\n')
  print('Finished',label,p.returncode,flush=True)
