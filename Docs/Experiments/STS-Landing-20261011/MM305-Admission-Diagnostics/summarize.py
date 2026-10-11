#!/usr/bin/env python3
"""Summarize joint, rather than independently minimized, route trial evidence."""
import hashlib,json,re
from pathlib import Path
HERE=Path(__file__).resolve().parent;ROOT=HERE.parents[3]
lines=(HERE/'focused-sts.log').read_text().splitlines();trials=[];timeouts=[]
for line in lines:
 if 'TAEM smooth trial:' not in line and 'TAEM constant profile timeout:' not in line:continue
 values={k:(float(v) if k!='reason' else v) for k,v in re.findall(r'(\w+)=([^\s]+)',line)}
 (trials if 'smooth trial:' in line else timeouts).append(values)
completed=[r for r in trials if r['completed']]
report={'sourceLogs':{k:hashlib.sha256((HERE/k).read_bytes()).hexdigest() for k in ['full-sts.log','focused-sts.log']},
 'fullBatchCommand':['CLanding/build/mm305_feasibility_tests','--configuration','Configuration/sts-live.json','--mass-kg','25123.15'],
 'fullBatchSourceCommit':'2cc042c',
 'binaryHashScope':'focused diagnostic build; not the original full-batch binary',
 'fullBatchEnvironment':{'KSP_LANDER_MODEL_PROFILE':'sts-live-20261010'},
 'fullBatchResult':'5 physically admitted; 0 qualified; 0 gate/adoption violations; failed non-vacuity, exit 1',
 'focusedCommand':['make','-C','CLanding','mm305-feasibility-test','MM305_FEASIBILITY_ARGS=--configuration Configuration/sts-live.json --mass-kg 25123.15 --sample-index 0'],
 'focusedEnvironment':{'KSP_LANDER_MODEL_PROFILE':'sts-live-20261010','KSP_LANDER_TAEM_PROFILE_DIAGNOSTICS':'1'},
 'focusedScope':'sample 0 only; not full qualification; expected failure remains',
 'smoothTrialCount':len(trials),'completed':len(completed),'jointlyFeasible':sum(r['feasible'] for r in trials),
 'closestAltitudeCompletedTrials':sorted(completed,key=lambda r:r['endpoint'])[:10],
 'constantProfileTimeouts':timeouts,
 'attitudeComparison':{},'inputHashes':{}}
keys={'pitch_wn','pitch_zeta','roll_wn','roll_zeta','max_pitch_rate_deg_s','max_roll_rate_deg_s','max_pitch_accel_deg_s2','max_roll_accel_deg_s2','pitch_full_authority_q_pa','roll_full_authority_q_pa'}
for relative in ['Configuration/sts-live.json','ShuttleSim/reference-model/stsn_attitude_identified.ini','Docs/Experiments/STS-Landing-20261010/Codex-Flare-Offline/inputs/att-live.ini','CLanding/build/mm305_feasibility_tests']:
 p=ROOT/relative;report['inputHashes'][relative]=hashlib.sha256(p.read_bytes()).hexdigest()
 if p.suffix=='.ini':
  report['attitudeComparison'][relative]={k:float(v) for k,v in (x.split('=',1) for x in p.read_text().splitlines() if '=' in x and not x.startswith('#')) if k in keys}
values=list(report['attitudeComparison'].values());assert values[0]==values[1]
assert len(trials)==2184 and not report['jointlyFeasible']
(HERE/'result.json').write_text(json.dumps(report,indent=2)+'\n')
